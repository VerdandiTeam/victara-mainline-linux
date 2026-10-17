// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/fixp-arith.h>
#include <linux/iio/consumer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#define BMS_CC_DATA_CTL			0x42
#define BMS_HOLD_OREG_DATA		BIT(0)
#define BMS_CC_CLEAR_CTL		0x43
#define BMS_CLEAR_CC			BIT(7)
#define BMS_CC_DATA0			0x8a
#define BMS_OCV_FOR_SOC_DATA0		0x90
#define BMS_VSENSE_AVG_DATA0		0x98
#define BMS_SOC_STORAGE			0xb0
#define BMS_SOC_STORAGE_MASK		GENMASK(7, 1)
#define BMS_SOC_STORAGE_INVALID		0x7f
#define BMS_IAVG_STORAGE		0xb1
#define BMS_IAVG_STORAGE_INVALID	0xff
#define BMS_CHARGE_INCREASE_STORAGE	0xbd
#define BMS_CHARGE_CYCLE_STORAGE	0xbe

#define IADC_BMS_REVISION2		0x01
#define IADC_BMS_ADC_CH_SEL_CTL		0x48
#define IADC_BMS_ADC_CH_SEL_MASK	GENMASK(2, 0)
#define IADC_BMS_EXTERNAL_RSENSE	1
#define IADC_BMS_ADC_INT_RSNSN_CTL	0x49
#define IADC_BMS_INT_RSNSN_MASK		GENMASK(1, 0)
#define IADC_BMS_INT_RSNSN_EXT_RSENSE	2
#define IADC_BMS_FAST_AVG_EN		0x5b
#define IADC_BMS_FAST_AVG_EXT_RSENSE	BIT(7)
#define IADC_BMS_OLD_DIG_MAJOR_MAX	3

#define PON_WARM_RESET_REASON1		0x80a
#define PON_WARM_RESET_REASON2		0x80b

#define VADC_INTRINSIC_OFFSET		0x6000
#define VADC_UV_PER_BIT_MUL		97656
#define VADC_UV_PER_BIT_DIV		1000
#define VADC_REF_625MV_UV		625000
#define VBAT_PRESCALE			3

#define CC_UV_PER_BIT_MUL		542535
#define CC_UV_PER_BIT_DIV		100000
#define CC_READING_TICKS		56
#define SLEEP_CLK_HZ			32764
#define SECONDS_PER_HOUR		3600

#define BMS_MIN_OCV_UV			2000000
#define BMS_DEFAULT_TEMP_DECIDEGC	250
#define BMS_THERM_VREF_MV		1800
#define BMS_FCC_TEMP_MAX		8
#define BMS_SF_COLS_MAX			8
#define BMS_SF_ROWS_MAX			32
#define BMS_IAVG_SAMPLES		16
#define BMS_IAVG_MIN_MA			250
#define BMS_IAVG_MIN_SEC		2
#define BMS_UUC_SEC_PER_PERCENT		60
#define BMS_RBATT_EST_PM		500
#define BMS_IAVG_STEP_MA		10
#define BMS_IGNORE_SOC_TEMP_DECIDEGC	50
#define BMS_LOW_VOLTAGE_MARGIN_UV	20000
#define BMS_LOW_VOLTAGE_SAMPLES		3
#define BMS_LOW_VOLTAGE_UPDATE_MS	1000
#define BMS_UPDATE_MS			20000
#define BMS_HOLD_US			100

struct pm8941_bms_therm_point {
	int mv;
	int decidegc;
};

static const struct pm8941_bms_therm_point pm8941_bms_therm_table[] = {
	{ 1512, -400 }, { 1494, -350 }, { 1471, -300 }, { 1443, -250 }, { 1407, -200 },
	{ 1365, -150 }, { 1315, -100 }, { 1258, -50 }, { 1194, 0 }, { 1126, 50 },
	{ 1054, 100 }, { 981, 150 }, { 909, 200 }, { 839, 250 }, { 772, 300 },
	{ 711, 350 }, { 656, 400 }, { 606, 450 }, { 562, 500 }, { 524, 550 },
	{ 491, 600 }, { 462, 650 }, { 437, 700 }, { 416, 750 }, { 398, 800 },
};

struct pm8941_bms {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *battery;
	struct power_supply *charger;
	struct power_supply_battery_info *info;
	struct iio_channel *vbat;
	struct iio_channel *ref_625mv;
	struct iio_channel *ref_1250mv;
	struct iio_channel *therm;
	struct iio_channel *gnd_ref;
	struct iio_channel *vdd_vadc;
	struct delayed_work work;
	struct mutex lock;

	u32 reg;
	u32 iadc_reg;
	u32 rsense_uohm;

	int fcc_len;
	u32 fcc_temp_degc[BMS_FCC_TEMP_MAX];
	u32 fcc_uah[BMS_FCC_TEMP_MAX];

	int rbatt_mohm;
	int sf_cols;
	int sf_rows;
	u32 sf_temp_degc[BMS_SF_COLS_MAX];
	u32 sf_pc[BMS_SF_ROWS_MAX];
	u32 sf[BMS_SF_ROWS_MAX * BMS_SF_COLS_MAX];

	u16 ocv_raw;
	int ocv_uv;
	int ocv_pm;
	s64 cc_uah;

	int iavg_samples[BMS_IAVG_SAMPLES];
	int iavg_idx;
	int iavg_count;
	u64 last_update_ns;
	int iavg_ma;
	int uuc_pm;
	int uuc_uah;

	int shutdown_soc_limit;
	bool low_voltage;
	int cutoff_count;

	int cycle_count;
	int charge_increase;
	int charge_start_soc;

	bool ready;
	int temp;
	int fcc_now_uah;
	int charge_full_uah;
	int charge_now_uah;
	int capacity;
	int status;
};

static int pm8941_bms_hold(struct pm8941_bms *bms, bool hold)
{
	int ret;

	ret = regmap_write_bits(bms->regmap, bms->reg + BMS_CC_DATA_CTL, BMS_HOLD_OREG_DATA,
				hold ? BMS_HOLD_OREG_DATA : 0);
	if (ret)
		return ret;

	if (hold)
		fsleep(BMS_HOLD_US);

	return 0;
}

static int pm8941_bms_clear_cc(struct pm8941_bms *bms)
{
	int ret;

	ret = regmap_write_bits(bms->regmap, bms->reg + BMS_CC_CLEAR_CTL, BMS_CLEAR_CC, BMS_CLEAR_CC);
	if (ret)
		return ret;

	fsleep(BMS_HOLD_US);

	return regmap_write_bits(bms->regmap, bms->reg + BMS_CC_CLEAR_CTL, BMS_CLEAR_CC, 0);
}

static int pm8941_bms_read_raw(struct pm8941_bms *bms, u16 *ocv_raw, s64 *cc_raw)
{
	u8 ocv[2], cc[5];
	int ret, ret2;

	ret = pm8941_bms_hold(bms, true);
	if (ret)
		return ret;

	ret = regmap_bulk_read(bms->regmap, bms->reg + BMS_OCV_FOR_SOC_DATA0, ocv, sizeof(ocv));
	if (!ret)
		ret = regmap_bulk_read(bms->regmap, bms->reg + BMS_CC_DATA0, cc, sizeof(cc));

	ret2 = pm8941_bms_hold(bms, false);
	if (ret)
		return ret;

	if (ret2)
		return ret2;

	*ocv_raw = ocv[0] | ocv[1] << 8;
	*cc_raw = sign_extend64((u64)cc[0] | (u64)cc[1] << 8 | (u64)cc[2] << 16 |
				(u64)cc[3] << 24 | (u64)cc[4] << 32, 35);

	return pm8941_bms_clear_cc(bms);
}

static int pm8941_bms_current_ua(struct pm8941_bms *bms, int *current_ua)
{
	u8 vsense[2];
	s64 vsense_uv;
	int ret, ret2;

	ret = pm8941_bms_hold(bms, true);
	if (ret)
		return ret;

	ret = regmap_bulk_read(bms->regmap, bms->reg + BMS_VSENSE_AVG_DATA0, vsense, sizeof(vsense));

	ret2 = pm8941_bms_hold(bms, false);
	if (ret)
		return ret;

	if (ret2)
		return ret2;

	vsense_uv = div_s64((s64)(s16)(vsense[0] | vsense[1] << 8) * CC_UV_PER_BIT_MUL, CC_UV_PER_BIT_DIV);
	*current_ua = -div_s64(vsense_uv * 1000000, bms->rsense_uohm);

	return 0;
}

static int pm8941_bms_ocv_raw_to_uv(struct pm8941_bms *bms, u16 raw)
{
	int code_625, code_1250;
	s64 uv;

	if (raw <= VADC_INTRINSIC_OFFSET)
		return 0;

	if (!iio_read_channel_raw(bms->ref_625mv, &code_625) &&
	    !iio_read_channel_raw(bms->ref_1250mv, &code_1250) &&
	    code_1250 > code_625)
		uv = VADC_REF_625MV_UV + div_s64((s64)(raw - code_625) * VADC_REF_625MV_UV, code_1250 - code_625);
	else
		uv = div_s64((s64)(raw - VADC_INTRINSIC_OFFSET) * VADC_UV_PER_BIT_MUL, VADC_UV_PER_BIT_DIV);

	return uv * VBAT_PRESCALE;
}

static int pm8941_bms_vbat_uv(struct pm8941_bms *bms, int *vbat_uv)
{
	int raw, ret;

	ret = iio_read_channel_raw(bms->vbat, &raw);
	if (ret < 0)
		return ret;

	*vbat_uv = pm8941_bms_ocv_raw_to_uv(bms, raw);

	return 0;
}

static s64 pm8941_bms_cc_raw_to_uah(struct pm8941_bms *bms, s64 cc_raw)
{
	s64 cc_uv, cc_pvh;

	cc_uv = div_s64(cc_raw * CC_UV_PER_BIT_MUL, CC_UV_PER_BIT_DIV);
	cc_pvh = div_s64(cc_uv * CC_READING_TICKS * 100000, SLEEP_CLK_HZ * SECONDS_PER_HOUR) * 10;

	return div_s64(cc_pvh, bms->rsense_uohm);
}

static int pm8941_bms_read_temp(struct pm8941_bms *bms)
{
	const struct pm8941_bms_therm_point *t = pm8941_bms_therm_table;
	int n = ARRAY_SIZE(pm8941_bms_therm_table);
	int raw, gnd, vdd, mv, i;

	if (!bms->therm || iio_read_channel_raw(bms->therm, &raw) < 0 ||
	    iio_read_channel_raw(bms->gnd_ref, &gnd) < 0 || iio_read_channel_raw(bms->vdd_vadc, &vdd) < 0 ||
	    vdd <= gnd)
		return bms->temp;

	mv = (raw - gnd) * BMS_THERM_VREF_MV / (vdd - gnd);

	if (mv >= t[0].mv)
		return t[0].decidegc;

	for (i = 1; i < n; i++)
		if (mv >= t[i].mv)
			return fixp_linear_interpolate(t[i].mv, t[i].decidegc, t[i - 1].mv, t[i - 1].decidegc, mv);

	return t[n - 1].decidegc;
}

static int pm8941_bms_table_pm(const struct power_supply_battery_ocv_table *table, int len, int ocv_uv)
{
	int i;

	if (ocv_uv >= table[0].ocv)
		return table[0].capacity * 10;

	for (i = 1; i < len; i++)
		if (ocv_uv >= table[i].ocv)
			return fixp_linear_interpolate(table[i].ocv, table[i].capacity * 10,
						       table[i - 1].ocv, table[i - 1].capacity * 10, ocv_uv);

	return table[len - 1].capacity * 10;
}

static int pm8941_bms_table_ocv(const struct power_supply_battery_ocv_table *table, int len, int pm)
{
	int i;

	if (pm >= table[0].capacity * 10)
		return table[0].ocv;

	for (i = 1; i < len; i++)
		if (pm >= table[i].capacity * 10)
			return fixp_linear_interpolate(table[i].capacity * 10, table[i].ocv,
						       table[i - 1].capacity * 10, table[i - 1].ocv, pm);

	return table[len - 1].ocv;
}

static int pm8941_bms_table_lookup(struct pm8941_bms *bms, int index, int x, bool to_ocv)
{
	const struct power_supply_battery_ocv_table *table = bms->info->ocv_table[index];
	int len = bms->info->ocv_table_size[index];

	return to_ocv ? pm8941_bms_table_ocv(table, len, x) : pm8941_bms_table_pm(table, len, x);
}

static int pm8941_bms_lookup(struct pm8941_bms *bms, int x, int temp, bool to_ocv)
{
	struct power_supply_battery_info *info = bms->info;
	int n = 0, i, lo, hi;

	while (n < POWER_SUPPLY_OCV_TEMP_MAX && info->ocv_table[n] && info->ocv_table_size[n] > 0)
		n++;

	if (!n)
		return 0;

	for (i = 0; i < n; i++)
		if (temp <= info->ocv_temp[i] * 10)
			break;

	if (i == 0)
		return pm8941_bms_table_lookup(bms, 0, x, to_ocv);

	if (i == n)
		return pm8941_bms_table_lookup(bms, n - 1, x, to_ocv);

	lo = pm8941_bms_table_lookup(bms, i - 1, x, to_ocv);
	hi = pm8941_bms_table_lookup(bms, i, x, to_ocv);

	return fixp_linear_interpolate(info->ocv_temp[i - 1] * 10, lo, info->ocv_temp[i] * 10, hi, temp);
}

static int pm8941_bms_fcc_uah(struct pm8941_bms *bms, int temp)
{
	int n = bms->fcc_len, i;

	if (!n)
		return bms->info->charge_full_design_uah;

	for (i = 0; i < n; i++)
		if (temp <= (s32)bms->fcc_temp_degc[i] * 10)
			break;

	if (i == 0)
		return bms->fcc_uah[0];

	if (i == n)
		return bms->fcc_uah[n - 1];

	return fixp_linear_interpolate((s32)bms->fcc_temp_degc[i - 1] * 10, bms->fcc_uah[i - 1],
				       (s32)bms->fcc_temp_degc[i] * 10, bms->fcc_uah[i], temp);
}

static int pm8941_bms_update_temp(struct pm8941_bms *bms)
{
	int full_pm;

	bms->temp = pm8941_bms_read_temp(bms);
	bms->fcc_now_uah = pm8941_bms_fcc_uah(bms, bms->temp);

	full_pm = pm8941_bms_lookup(bms, bms->info->voltage_max_design_uv, bms->temp, false);
	bms->charge_full_uah = div_s64((s64)bms->fcc_now_uah * full_pm, 1000);

	return full_pm;
}

static int pm8941_bms_sf_at_row(struct pm8941_bms *bms, int row, int temp)
{
	const u32 *sf = &bms->sf[row * bms->sf_cols];
	int n = bms->sf_cols, i;

	for (i = 0; i < n; i++)
		if (temp <= (s32)bms->sf_temp_degc[i] * 10)
			break;

	if (i == 0)
		return sf[0];

	if (i == n)
		return sf[n - 1];

	return fixp_linear_interpolate((s32)bms->sf_temp_degc[i - 1] * 10, sf[i - 1],
				       (s32)bms->sf_temp_degc[i] * 10, sf[i], temp);
}

static int pm8941_bms_rbatt_mohm(struct pm8941_bms *bms, int pm, int temp)
{
	int n = bms->sf_rows, i, scale;

	if (!n)
		return bms->rbatt_mohm;

	for (i = 0; i < n; i++)
		if (pm >= (int)bms->sf_pc[i] * 10)
			break;

	if (i == 0)
		scale = pm8941_bms_sf_at_row(bms, 0, temp);
	else if (i == n)
		scale = pm8941_bms_sf_at_row(bms, n - 1, temp);
	else
		scale = fixp_linear_interpolate(bms->sf_pc[i] * 10, pm8941_bms_sf_at_row(bms, i, temp),
						bms->sf_pc[i - 1] * 10, pm8941_bms_sf_at_row(bms, i - 1, temp), pm);

	return bms->rbatt_mohm * scale / 100;
}

static int pm8941_bms_iavg_ma(struct pm8941_bms *bms, s64 cc_delta_uah)
{
	u64 now = ktime_get_boottime_ns();
	u64 delta_s = div_u64(now - bms->last_update_ns, NSEC_PER_SEC);
	int i, sum = 0;

	bms->last_update_ns = now;

	if (delta_s >= BMS_IAVG_MIN_SEC) {
		bms->iavg_samples[bms->iavg_idx] = max_t(s64, div64_s64(cc_delta_uah * 3600, delta_s * 1000),
							 BMS_IAVG_MIN_MA);
		bms->iavg_idx = (bms->iavg_idx + 1) % BMS_IAVG_SAMPLES;

		if (bms->iavg_count < BMS_IAVG_SAMPLES)
			bms->iavg_count++;
	}

	if (!bms->iavg_count)
		return BMS_IAVG_MIN_MA;

	for (i = 0; i < bms->iavg_count; i++)
		sum += bms->iavg_samples[i];

	return DIV_ROUND_CLOSEST(sum, bms->iavg_count);
}

static int pm8941_bms_uuc_pm(struct pm8941_bms *bms, int iavg_ma)
{
	int vcut_uv = bms->info->voltage_min_design_uv;
	int pc, ocv_uv, rbatt = 0, prev_rbatt = 0, uuc_rbatt;
	s64 delta = 0, prev_delta = 0;

	for (pc = 0; pc <= 100; pc++) {
		ocv_uv = pm8941_bms_lookup(bms, pc * 10, bms->temp, true);
		rbatt = pm8941_bms_rbatt_mohm(bms, pc * 10, bms->temp);
		delta = (s64)ocv_uv - ((s64)rbatt * iavg_ma + vcut_uv);

		if (delta > 0)
			break;

		prev_delta = delta;
		prev_rbatt = rbatt;
	}

	if (pc == 0 || pc > 100 || delta == prev_delta)
		uuc_rbatt = rbatt;
	else
		uuc_rbatt = prev_rbatt + div64_s64((s64)(rbatt - prev_rbatt) * -prev_delta, delta - prev_delta);

	return clamp(pm8941_bms_lookup(bms, uuc_rbatt * iavg_ma + vcut_uv, bms->temp, false), 0, 1000);
}

static void pm8941_bms_update_uuc(struct pm8941_bms *bms, s64 cc_delta_uah)
{
	u64 prev_ns = bms->last_update_ns;
	int iavg_ma, new_pm, max_change;

	iavg_ma = pm8941_bms_iavg_ma(bms, cc_delta_uah);
	bms->iavg_ma = iavg_ma;
	new_pm = pm8941_bms_uuc_pm(bms, iavg_ma);

	max_change = max_t(u32, (u32)div_u64(bms->last_update_ns - prev_ns, NSEC_PER_SEC) / BMS_UUC_SEC_PER_PERCENT, 1) * 10;

	if (bms->uuc_pm < 0 || abs(new_pm - bms->uuc_pm) <= max_change)
		bms->uuc_pm = new_pm;
	else if (new_pm > bms->uuc_pm)
		bms->uuc_pm += max_change;
	else
		bms->uuc_pm -= max_change;

	bms->uuc_uah = div_s64((s64)bms->fcc_now_uah * bms->uuc_pm, 1000);

	dev_dbg(bms->dev, "iavg %d mA, uuc %d permille (target %d), %d uAh\n", iavg_ma, bms->uuc_pm, new_pm,
		bms->uuc_uah);
}

static int pm8941_bms_charger_status(struct pm8941_bms *bms)
{
	union power_supply_propval val;

	if (power_supply_get_property(bms->charger, POWER_SUPPLY_PROP_STATUS, &val))
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return val.intval;
}

static int pm8941_bms_capacity(struct pm8941_bms *bms, int *charge_now_uah)
{
	int charge_full_uah = bms->charge_full_uah - bms->uuc_uah;
	s64 ocv_charge_uah;

	if (charge_full_uah <= 0)
		return -EINVAL;

	ocv_charge_uah = div_s64((s64)bms->fcc_now_uah * bms->ocv_pm, 1000);
	*charge_now_uah = clamp_t(s64, ocv_charge_uah - bms->cc_uah - bms->uuc_uah, 0, charge_full_uah);

	return DIV_ROUND_CLOSEST_ULL((u64)*charge_now_uah * 100, charge_full_uah);
}

static int pm8941_bms_low_voltage(struct pm8941_bms *bms, int status, int capacity)
{
	int vcut_uv = bms->info->voltage_min_design_uv;
	bool low = false;
	int vbat_uv;

	if (status != POWER_SUPPLY_STATUS_CHARGING && status != POWER_SUPPLY_STATUS_FULL &&
	    !pm8941_bms_vbat_uv(bms, &vbat_uv)) {
		low = vbat_uv <= vcut_uv + BMS_LOW_VOLTAGE_MARGIN_UV;
		bms->cutoff_count = vbat_uv <= vcut_uv ? bms->cutoff_count + 1 : 0;

		if (bms->cutoff_count >= BMS_LOW_VOLTAGE_SAMPLES)
			capacity = 0;
		else if (capacity <= 0 && vbat_uv > vcut_uv)
			capacity = 1;
	} else {
		bms->cutoff_count = 0;
	}

	if (low != bms->low_voltage) {
		bms->low_voltage = low;

		if (low)
			pm_stay_awake(bms->dev);
		else
			pm_relax(bms->dev);

		dev_info(bms->dev, "battery voltage %s low threshold\n", low ? "below" : "back above");
	}

	return capacity;
}

static void pm8941_bms_count_cycles(struct pm8941_bms *bms, int status, int capacity)
{
	u8 cycles[2];

	if (status == POWER_SUPPLY_STATUS_CHARGING && bms->status != POWER_SUPPLY_STATUS_CHARGING) {
		bms->charge_start_soc = capacity;
		return;
	}

	if (bms->status != POWER_SUPPLY_STATUS_CHARGING || status == POWER_SUPPLY_STATUS_CHARGING ||
	    bms->charge_start_soc < 0)
		return;

	if (capacity > bms->charge_start_soc) {
		bms->charge_increase += capacity - bms->charge_start_soc;

		if (bms->charge_increase > 100) {
			bms->cycle_count++;
			bms->charge_increase %= 100;
		}

		cycles[0] = bms->cycle_count & 0xff;
		cycles[1] = bms->cycle_count >> 8;

		regmap_write(bms->regmap, bms->reg + BMS_CHARGE_INCREASE_STORAGE, bms->charge_increase);
		regmap_bulk_write(bms->regmap, bms->reg + BMS_CHARGE_CYCLE_STORAGE, cycles, sizeof(cycles));
	}

	bms->charge_start_soc = -1;
}

static void pm8941_bms_backup_soc(struct pm8941_bms *bms, int capacity)
{
	unsigned int soc = BMS_SOC_STORAGE_INVALID, iavg = 0;

	if (bms->temp > BMS_IGNORE_SOC_TEMP_DECIDEGC)
		soc = capacity + 1;

	if (bms->iavg_ma > BMS_IAVG_MIN_MA)
		iavg = min((bms->iavg_ma - BMS_IAVG_MIN_MA) / BMS_IAVG_STEP_MA, BMS_IAVG_STORAGE_INVALID - 1);

	regmap_update_bits(bms->regmap, bms->reg + BMS_SOC_STORAGE, BMS_SOC_STORAGE_MASK,
			   FIELD_PREP(BMS_SOC_STORAGE_MASK, soc));
	regmap_write(bms->regmap, bms->reg + BMS_IAVG_STORAGE, iavg);
}

static void pm8941_bms_update(struct pm8941_bms *bms)
{
	int status, charge_now_uah, capacity, full_pm, old_temp;
	s64 cc_delta_uah;
	bool changed;
	u16 ocv_raw;
	s64 cc_raw;
	int ret;

	status = pm8941_bms_charger_status(bms);

	mutex_lock(&bms->lock);

	if (!bms->ready) {
		mutex_unlock(&bms->lock);
		return;
	}

	ret = pm8941_bms_read_raw(bms, &ocv_raw, &cc_raw);
	if (ret) {
		mutex_unlock(&bms->lock);
		dev_err(bms->dev, "failed to read BMS: %pe\n", ERR_PTR(ret));
		return;
	}

	old_temp = bms->temp;
	full_pm = pm8941_bms_update_temp(bms);
	cc_delta_uah = pm8941_bms_cc_raw_to_uah(bms, cc_raw);

	dev_dbg(bms->dev, "ocv_raw 0x%04x cc_raw %lld cc_uah %lld temp %d fcc %d full %d\n", ocv_raw, cc_raw,
		cc_delta_uah, bms->temp, bms->fcc_now_uah, bms->charge_full_uah);

	if (ocv_raw != bms->ocv_raw) {
		bms->ocv_raw = ocv_raw;
		bms->ocv_uv = pm8941_bms_ocv_raw_to_uv(bms, ocv_raw);
		bms->ocv_pm = pm8941_bms_lookup(bms, bms->ocv_uv, bms->temp, false);
		bms->cc_uah = 0;
		dev_dbg(bms->dev, "new OCV %d uV, %d permille\n", bms->ocv_uv, bms->ocv_pm);
	}

	bms->cc_uah += cc_delta_uah;

	if (status == POWER_SUPPLY_STATUS_FULL && bms->status != POWER_SUPPLY_STATUS_FULL) {
		bms->ocv_pm = full_pm;
		bms->cc_uah = 0;
	}

	pm8941_bms_update_uuc(bms, cc_delta_uah);

	capacity = pm8941_bms_capacity(bms, &charge_now_uah);
	if (capacity < 0) {
		mutex_unlock(&bms->lock);
		return;
	}

	capacity = pm8941_bms_low_voltage(bms, status, capacity);

	pm8941_bms_count_cycles(bms, status, capacity);
	pm8941_bms_backup_soc(bms, capacity);

	changed = capacity != bms->capacity || status != bms->status || bms->temp / 10 != old_temp / 10;

	bms->charge_now_uah = charge_now_uah;
	bms->capacity = capacity;
	bms->status = status;

	mutex_unlock(&bms->lock);

	if (changed)
		power_supply_changed(bms->battery);
}

static void pm8941_bms_work(struct work_struct *work)
{
	struct pm8941_bms *bms = container_of(work, struct pm8941_bms, work.work);

	pm8941_bms_update(bms);

	queue_delayed_work(system_freezable_wq, &bms->work,
			   msecs_to_jiffies(bms->low_voltage ? BMS_LOW_VOLTAGE_UPDATE_MS : BMS_UPDATE_MS));
}

static int pm8941_bms_get_property(struct power_supply *psy, enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pm8941_bms *bms = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bms->status;
		return 0;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bms->capacity;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!bms->ready)
			return -ENODATA;

		val->intval = bms->info->charge_full_design_uah;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bms->charge_full_uah - bms->uuc_uah;
		return 0;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bms->temp;
		return 0;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bms->cycle_count;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bms->charge_now_uah;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = bms->ocv_uv;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return pm8941_bms_vbat_uv(bms, &val->intval);

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&bms->lock);
		ret = pm8941_bms_current_ua(bms, &val->intval);
		mutex_unlock(&bms->lock);
		return ret;

	default:
		return -EINVAL;
	}
}

static void pm8941_bms_external_power_changed(struct power_supply *psy)
{
	struct pm8941_bms *bms = power_supply_get_drvdata(psy);

	mod_delayed_work(system_freezable_wq, &bms->work, 0);
}

static const enum power_supply_property pm8941_bms_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc pm8941_bms_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pm8941_bms_properties,
	.num_properties = ARRAY_SIZE(pm8941_bms_properties),
	.get_property = pm8941_bms_get_property,
	.external_power_changed = pm8941_bms_external_power_changed,
};

static int pm8941_bms_setup_rsense(struct pm8941_bms *bms)
{
	unsigned int sel, rev;
	int ret;

	ret = regmap_read(bms->regmap, bms->iadc_reg + IADC_BMS_ADC_CH_SEL_CTL, &sel);
	if (ret)
		return ret;

	if (FIELD_GET(IADC_BMS_ADC_CH_SEL_MASK, sel) != IADC_BMS_EXTERNAL_RSENSE) {
		dev_info(bms->dev, "switching BMS IADC to external rsense\n");

		ret = regmap_update_bits(bms->regmap, bms->iadc_reg + IADC_BMS_ADC_CH_SEL_CTL,
					 IADC_BMS_ADC_CH_SEL_MASK, IADC_BMS_EXTERNAL_RSENSE);
		if (ret)
			return ret;

		ret = pm8941_bms_clear_cc(bms);
		if (ret)
			return ret;
	}

	ret = regmap_read(bms->regmap, bms->iadc_reg + IADC_BMS_REVISION2, &rev);
	if (ret)
		return ret;

	dev_dbg(bms->dev, "iadc ch_sel 0x%02x rev2 0x%02x\n", sel, rev);

	if (rev > IADC_BMS_OLD_DIG_MAJOR_MAX)
		return regmap_update_bits(bms->regmap, bms->iadc_reg + IADC_BMS_ADC_INT_RSNSN_CTL,
					  IADC_BMS_INT_RSNSN_MASK, IADC_BMS_INT_RSNSN_EXT_RSENSE);

	return regmap_update_bits(bms->regmap, bms->iadc_reg + IADC_BMS_FAST_AVG_EN,
				  IADC_BMS_FAST_AVG_EXT_RSENSE, IADC_BMS_FAST_AVG_EXT_RSENSE);
}

static int pm8941_bms_read_cycles(struct pm8941_bms *bms)
{
	unsigned int increase;
	u8 cycles[2];
	int ret;

	ret = regmap_read(bms->regmap, bms->reg + BMS_CHARGE_INCREASE_STORAGE, &increase);
	if (ret)
		return ret;

	ret = regmap_bulk_read(bms->regmap, bms->reg + BMS_CHARGE_CYCLE_STORAGE, cycles, sizeof(cycles));
	if (ret)
		return ret;

	if (increase != 0xff)
		bms->charge_increase = increase;

	if (cycles[0] != 0xff || cycles[1] != 0xff)
		bms->cycle_count = cycles[0] | cycles[1] << 8;

	return 0;
}

static void pm8941_bms_restore_soc(struct pm8941_bms *bms)
{
	unsigned int stored, iavg;
	int soc, calculated, charge_now_uah;
	s64 target_uah;

	if (regmap_read(bms->regmap, bms->reg + BMS_SOC_STORAGE, &stored) ||
	    regmap_read(bms->regmap, bms->reg + BMS_IAVG_STORAGE, &iavg))
		return;

	soc = (int)FIELD_GET(BMS_SOC_STORAGE_MASK, stored) - 1;

	if (soc >= 0 && soc <= 100 && iavg != BMS_IAVG_STORAGE_INVALID)
		bms->iavg_samples[0] = BMS_IAVG_MIN_MA + iavg * BMS_IAVG_STEP_MA;

	bms->iavg_ma = bms->iavg_samples[0];
	bms->uuc_pm = pm8941_bms_uuc_pm(bms, bms->iavg_ma);
	bms->uuc_uah = div_s64((s64)bms->fcc_now_uah * bms->uuc_pm, 1000);

	calculated = pm8941_bms_capacity(bms, &charge_now_uah);

	if (soc < 0 || soc > 100 || calculated < 0 || bms->shutdown_soc_limit < 0 ||
	    abs(soc - calculated) > bms->shutdown_soc_limit) {
		dev_info(bms->dev, "not using stored SoC %d%% (calculated %d%%)\n", soc, calculated);
		return;
	}

	target_uah = div_s64((s64)soc * (bms->charge_full_uah - bms->uuc_uah), 100) + bms->uuc_uah + bms->cc_uah;
	bms->ocv_pm = div64_s64(target_uah * 1000, bms->fcc_now_uah);

	dev_info(bms->dev, "using stored SoC %d%% (calculated %d%%, iavg %d mA)\n", soc, calculated, bms->iavg_ma);
}

static int pm8941_bms_init_ocv(struct pm8941_bms *bms)
{
	unsigned int reason1, reason2;
	int vbat_uv, current_ua, rbatt_mohm;
	s64 cc_raw;
	int ret;

	ret = pm8941_bms_read_raw(bms, &bms->ocv_raw, &cc_raw);
	if (ret)
		return ret;

	ret = regmap_read(bms->regmap, PON_WARM_RESET_REASON1, &reason1);
	if (!ret)
		ret = regmap_read(bms->regmap, PON_WARM_RESET_REASON2, &reason2);
	if (ret)
		return ret;

	ret = pm8941_bms_current_ua(bms, &current_ua);
	if (ret)
		return ret;

	pm8941_bms_update_temp(bms);

	bms->ocv_uv = pm8941_bms_ocv_raw_to_uv(bms, bms->ocv_raw);
	bms->cc_uah = pm8941_bms_cc_raw_to_uah(bms, cc_raw);

	if (bms->ocv_uv < BMS_MIN_OCV_UV || reason1 || reason2) {
		ret = pm8941_bms_vbat_uv(bms, &vbat_uv);
		if (ret < 0)
			return ret;

		rbatt_mohm = pm8941_bms_rbatt_mohm(bms, BMS_RBATT_EST_PM, bms->temp);

		dev_info(bms->dev, "PON OCV %d uV unusable (warm reset 0x%x 0x%x), estimating\n",
			 bms->ocv_uv, reason1, reason2);

		bms->ocv_uv = vbat_uv - div_s64((s64)current_ua * rbatt_mohm, 1000);
		bms->cc_uah = 0;
	}

	bms->ocv_pm = pm8941_bms_lookup(bms, bms->ocv_uv, bms->temp, false);

	bms->iavg_samples[0] = max(-current_ua / 1000, BMS_IAVG_MIN_MA);
	bms->iavg_idx = 1;
	bms->iavg_count = 1;
	bms->last_update_ns = ktime_get_boottime_ns();
	bms->uuc_pm = -1;

	dev_info(bms->dev, "OCV %d uV, %d permille, CC %lld uAh, temp %d, FCC %d uAh, full %d uAh, I %d uA\n",
		 bms->ocv_uv, bms->ocv_pm, bms->cc_uah, bms->temp, bms->fcc_now_uah, bms->charge_full_uah, current_ua);

	pm8941_bms_restore_soc(bms);

	return 0;
}

static int pm8941_bms_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config psy_cfg = {};
	struct pm8941_bms *bms;
	u32 regs[2], limit;
	int ret;

	bms = devm_kzalloc(dev, sizeof(*bms), GFP_KERNEL);
	if (!bms)
		return -ENOMEM;

	bms->dev = dev;
	bms->status = POWER_SUPPLY_STATUS_UNKNOWN;
	bms->temp = BMS_DEFAULT_TEMP_DECIDEGC;

	bms->regmap = dev_get_regmap(dev->parent, NULL);
	if (!bms->regmap)
		return -ENODEV;

	ret = device_property_read_u32_array(dev, "reg", regs, ARRAY_SIZE(regs));
	if (ret)
		return dev_err_probe(dev, ret, "failed to read reg\n");

	bms->reg = regs[0];
	bms->iadc_reg = regs[1];

	ret = device_property_read_u32(dev, "qcom,rsense-micro-ohms", &bms->rsense_uohm);
	if (ret || !bms->rsense_uohm)
		return dev_err_probe(dev, -EINVAL, "missing qcom,rsense-micro-ohms\n");

	bms->vbat = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(bms->vbat))
		return dev_err_probe(dev, PTR_ERR(bms->vbat), "failed to get vbat channel\n");

	bms->ref_625mv = devm_iio_channel_get(dev, "ref-625mv");
	if (IS_ERR(bms->ref_625mv))
		return dev_err_probe(dev, PTR_ERR(bms->ref_625mv), "failed to get ref-625mv channel\n");

	bms->ref_1250mv = devm_iio_channel_get(dev, "ref-1250mv");
	if (IS_ERR(bms->ref_1250mv))
		return dev_err_probe(dev, PTR_ERR(bms->ref_1250mv), "failed to get ref-1250mv channel\n");

	bms->therm = devm_iio_channel_get(dev, "therm");
	if (IS_ERR(bms->therm)) {
		ret = PTR_ERR(bms->therm);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to get therm channel\n");

		bms->therm = NULL;
	}

	if (bms->therm) {
		bms->gnd_ref = devm_iio_channel_get(dev, "gnd-ref");
		if (IS_ERR(bms->gnd_ref))
			return dev_err_probe(dev, PTR_ERR(bms->gnd_ref), "failed to get gnd-ref channel\n");

		bms->vdd_vadc = devm_iio_channel_get(dev, "vdd-vadc");
		if (IS_ERR(bms->vdd_vadc))
			return dev_err_probe(dev, PTR_ERR(bms->vdd_vadc), "failed to get vdd-vadc channel\n");
	}

	ret = device_property_count_u32(dev, "qcom,fcc-temp-celsius");
	if (ret > 0) {
		bms->fcc_len = ret;

		if (bms->fcc_len > BMS_FCC_TEMP_MAX ||
		    device_property_count_u32(dev, "qcom,fcc-microamp-hours") != bms->fcc_len)
			return dev_err_probe(dev, -EINVAL, "bad qcom,fcc-temp-celsius / qcom,fcc-microamp-hours\n");

		ret = device_property_read_u32_array(dev, "qcom,fcc-temp-celsius", bms->fcc_temp_degc, bms->fcc_len);
		if (!ret)
			ret = device_property_read_u32_array(dev, "qcom,fcc-microamp-hours", bms->fcc_uah, bms->fcc_len);
		if (ret)
			return dev_err_probe(dev, ret, "failed to read FCC table\n");
	}

	ret = device_property_count_u32(dev, "qcom,rbatt-sf-celsius");
	if (ret > 0) {
		bms->sf_cols = ret;
		bms->sf_rows = device_property_count_u32(dev, "qcom,rbatt-sf-capacity");

		if (bms->sf_cols > BMS_SF_COLS_MAX || bms->sf_rows <= 0 || bms->sf_rows > BMS_SF_ROWS_MAX ||
		    device_property_count_u32(dev, "qcom,rbatt-sf") != bms->sf_rows * bms->sf_cols)
			return dev_err_probe(dev, -EINVAL, "bad qcom,rbatt-sf tables\n");

		ret = device_property_read_u32_array(dev, "qcom,rbatt-sf-celsius", bms->sf_temp_degc, bms->sf_cols);
		if (!ret)
			ret = device_property_read_u32_array(dev, "qcom,rbatt-sf-capacity", bms->sf_pc, bms->sf_rows);
		if (!ret)
			ret = device_property_read_u32_array(dev, "qcom,rbatt-sf", bms->sf, bms->sf_rows * bms->sf_cols);
		if (ret)
			return dev_err_probe(dev, ret, "failed to read rbatt-sf tables\n");
	}

	bms->charger = devm_power_supply_get_by_reference(dev, "power-supplies");
	if (IS_ERR(bms->charger))
		return dev_err_probe(dev, PTR_ERR(bms->charger), "failed to get charger\n");

	if (!bms->charger)
		return -EPROBE_DEFER;

	ret = devm_mutex_init(dev, &bms->lock);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &bms->work, pm8941_bms_work);
	if (ret)
		return ret;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	bms->shutdown_soc_limit = -1;
	if (!device_property_read_u32(dev, "qcom,shutdown-soc-valid-limit-percent", &limit))
		bms->shutdown_soc_limit = min_t(u32, limit, 100);

	bms->charge_start_soc = -1;

	ret = pm8941_bms_read_cycles(bms);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read charge cycles\n");

	ret = pm8941_bms_setup_rsense(bms);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up rsense\n");

	psy_cfg.drv_data = bms;
	psy_cfg.fwnode = dev_fwnode(dev);

	bms->battery = devm_power_supply_register(dev, &pm8941_bms_desc, &psy_cfg);
	if (IS_ERR(bms->battery))
		return dev_err_probe(dev, PTR_ERR(bms->battery), "failed to register battery\n");

	ret = power_supply_get_battery_info(bms->battery, &bms->info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get battery info\n");

	if (bms->info->charge_full_design_uah <= 0 || bms->info->voltage_max_design_uv <= 0)
		return dev_err_probe(dev, -EINVAL, "battery needs charge-full-design and voltage-max-design\n");

	bms->rbatt_mohm = max(bms->info->factory_internal_resistance_uohm, 0) / 1000;

	if (!pm8941_bms_lookup(bms, bms->info->voltage_max_design_uv, BMS_DEFAULT_TEMP_DECIDEGC, false))
		return dev_err_probe(dev, -EINVAL, "battery needs ocv-capacity tables\n");

	mutex_lock(&bms->lock);

	ret = pm8941_bms_init_ocv(bms);
	if (!ret)
		bms->ready = true;

	mutex_unlock(&bms->lock);

	if (ret)
		return dev_err_probe(dev, ret, "failed to read initial OCV\n");

	platform_set_drvdata(pdev, bms);

	queue_delayed_work(system_freezable_wq, &bms->work, 0);

	return 0;
}

static int pm8941_bms_resume(struct device *dev)
{
	struct pm8941_bms *bms = dev_get_drvdata(dev);

	mod_delayed_work(system_freezable_wq, &bms->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pm8941_bms_pm_ops, NULL, pm8941_bms_resume);

static const struct of_device_id pm8941_bms_of_match[] = {
	{ .compatible = "qcom,pm8941-bms" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8941_bms_of_match);

static struct platform_driver pm8941_bms_driver = {
	.driver = {
		.name = "qcom-pm8941-bms",
		.of_match_table = pm8941_bms_of_match,
		.pm = pm_sleep_ptr(&pm8941_bms_pm_ops),
	},
	.probe = pm8941_bms_probe,
};
module_platform_driver(pm8941_bms_driver);

MODULE_DESCRIPTION("Qualcomm PM8941 BMS fuel gauge driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
