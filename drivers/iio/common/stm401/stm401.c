// SPDX-License-Identifier: GPL-2.0-only
/*
 * Motorola STM401 sensor hub
 *
 * Based on the downstream Motorola driver (drivers/misc/stm401),
 * Copyright (C) 2010-2013 Motorola Mobility LLC
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/timekeeping.h>
#include <linux/units.h>
#include <linux/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#define STM401_REG_REV_ID			0x01
#define STM401_REG_ERROR_STATUS			0x02
#define STM401_REG_LOWPOWER			0x03
#define STM401_REG_AP_POSIX_TIME		0x10
#define STM401_REG_ACCEL_UPDATE_RATE		0x16
#define STM401_REG_MAG_UPDATE_RATE		0x17
#define STM401_REG_GYRO_UPDATE_RATE		0x19
#define STM401_REG_NONWAKESENSOR_CONFIG		0x1a
#define STM401_REG_WAKESENSOR_CONFIG		0x1b
#define STM401_REG_MOTION_DUR			0x20
#define STM401_REG_QUAT_6AXIS_UPDATE_RATE	0x21
#define STM401_REG_ZRMOTION_DUR			0x22
#define STM401_REG_QUAT_9AXIS_UPDATE_RATE	0x23
#define STM401_REG_ALGO_CONFIG			0x26
#define STM401_REG_ALGO_INT_STATUS		0x27
#define STM401_REG_GENERIC_INT_STATUS		0x28
#define STM401_REG_PROX_SETTINGS		0x33
#define STM401_REG_LUX_TABLE			0x34
#define STM401_REG_BRIGHTNESS_TABLE		0x35
#define STM401_REG_STEP_COUNTER_UPDATE_RATE	0x36
#define STM401_REG_WAKESENSOR_STATUS		0x39
#define STM401_REG_INTERRUPT_STATUS		0x3a
#define STM401_REG_ACCEL_X			0x3b
#define STM401_REG_STEP_COUNTER			0x3e
#define STM401_REG_COVER_DATA			0x40
#define STM401_REG_TEMPERATURE			0x41
#define STM401_REG_GYRO_X			0x43
#define STM401_REG_QUATERNION			0x44
#define STM401_REG_MAG_HX			0x49
#define STM401_REG_FLAT_DATA			0x4b
#define STM401_REG_CAMERA			0x4c
#define STM401_REG_SIM				0x4e
#define STM401_REG_CHOPCHOP			0x4f
#define STM401_REG_LIFT				0x51
#define STM401_REG_ALS_LUX			0x6a
#define STM401_REG_PROXIMITY			0x6c
#define STM401_REG_STOWED			0x6d

#define STM401_M_ACCEL				BIT(0)
#define STM401_M_GYRO				BIT(1)
#define STM401_M_ECOMPASS			BIT(3)
#define STM401_M_TEMPERATURE			BIT(4)
#define STM401_M_ALS				BIT(5)
#define STM401_M_STEP_COUNTER			BIT(7)
#define STM401_M_QUAT_6AXIS			BIT(9)
#define STM401_M_QUAT_9AXIS			BIT(18)

#define STM401_M_PROXIMITY			BIT(1)
#define STM401_M_COVER				BIT(3)
#define STM401_M_LIFT				BIT(5)
#define STM401_M_HUB_RESET			BIT(7)
#define STM401_M_FLATUP				BIT(8)
#define STM401_M_FLATDOWN			BIT(9)
#define STM401_M_STOWED				BIT(10)
#define STM401_M_CAMERA_ACT			BIT(11)
#define STM401_M_SIM				BIT(13)
#define STM401_M_CHOPCHOP			BIT(14)
#define STM401_M_LOG_MSG			BIT(15)

#define STM401_ERROR_STATUS_SIZE		128
#define STM401_LIGHTING_TABLE_SIZE		32
#define STM401_MAX_DATA				16
#define STM401_MAX_CHANNELS			4

#define STM401_I2C_RETRIES			5
#define STM401_I2C_RETRY_DELAY_MS		13
#define STM401_RESET_RETRY_DELAY_MS		200
#define STM401_RESET_DELAY_MS			50
#define STM401_RESET_ATTEMPTS			5
#define STM401_WAKE_TIMEOUT_US			5000
#define STM401_DEFAULT_DELAY_MS			100
#define STM401_SAMPLE_TIMEOUT_MS		1000

enum stm401_sensor_id {
	STM401_ACCEL,
	STM401_GYRO,
	STM401_MAGN,
	STM401_LIGHT,
	STM401_PROXIMITY,
	STM401_TEMP,
	STM401_GAME_ROTATION,
	STM401_ROTATION,
	STM401_STEPS,
	STM401_NUM_SENSORS,
};

enum stm401_gesture_id {
	STM401_GESTURE_CHOPCHOP,
	STM401_GESTURE_CAMERA,
	STM401_GESTURE_LIFT,
	STM401_GESTURE_SIGNIFICANT_MOTION,
	STM401_GESTURE_FLAT_UP,
	STM401_GESTURE_FLAT_DOWN,
	STM401_GESTURE_STOWED,
	STM401_NUM_GESTURES,
};

struct stm401_sensor_info {
	const char *name;
	bool wake;
	bool on_change;
	u32 mask;
	u8 data_reg;
	u8 read_len;
	u8 offset;
	u8 rate_reg;
	const struct iio_chan_spec *channels;
	int num_channels;
	int scan_values;
	int scale_type;
	int scale_val;
	int scale_val2;
};

struct stm401_gesture_info {
	const char *label;
	u32 mask;
	bool stateful;
	u8 data_reg;
	u8 read_len;
	unsigned int key;
};

struct stm401_hub;

struct stm401_sensor {
	struct stm401_hub *hub;
	const struct stm401_sensor_info *info;
	struct iio_dev *indio_dev;

	unsigned int users;
	u8 delay_ms;

	s32 values[STM401_MAX_CHANNELS];
	bool valid;
	struct completion sample;
};

struct stm401_hub {
	struct i2c_client *client;
	struct device *dev;

	struct mutex lock;
	bool suspended;
	bool wake_pending;
	bool lowpower;

	struct regulator_bulk_data supplies[2];

	struct gpio_desc *reset_gpio;
	struct gpio_desc *boot_gpio;
	struct gpio_desc *wake_gpio;
	struct gpio_desc *wake_ack_gpio;

	int irq;
	int wake_irq;

	u16 lux_table[STM401_LIGHTING_TABLE_SIZE];
	u8 brightness_table[STM401_LIGHTING_TABLE_SIZE];
	u32 prox_settings[4];

	u32 nonwake_mask;
	u16 wake_mask;

	struct stm401_sensor sensors[STM401_NUM_SENSORS];

	struct iio_dev *gestures;
	u16 gesture_events;
	u32 gesture_raw[STM401_NUM_GESTURES];

	struct input_dev *input;
	bool input_open;

	u8 buf[STM401_ERROR_STATUS_SIZE + 1];
};

#define STM401_AXIS_CHANNEL(_type, _axis, _index)				\
{										\
	.type = _type,								\
	.modified = 1,								\
	.channel2 = IIO_MOD_##_axis,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |			\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.scan_index = _index,							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = 32,							\
		.storagebits = 32,						\
		.endianness = IIO_CPU,						\
	},									\
}

#define STM401_SCALAR_CHANNEL(_type, _sign, _rate)				\
{										\
	.type = _type,								\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |				\
			      BIT(IIO_CHAN_INFO_SCALE) | (_rate),		\
	.scan_index = 0,							\
	.scan_type = {								\
		.sign = _sign,							\
		.realbits = 32,							\
		.storagebits = 32,						\
		.endianness = IIO_CPU,						\
	},									\
}

#define STM401_QUAT_CHANNEL							\
{										\
	.type = IIO_ROT,							\
	.modified = 1,								\
	.channel2 = IIO_MOD_QUATERNION,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |				\
			      BIT(IIO_CHAN_INFO_SCALE) |			\
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = 0,							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = 32,							\
		.storagebits = 32,						\
		.repeat = 4,							\
		.endianness = IIO_CPU,						\
	},									\
}

static const struct iio_chan_spec stm401_accel_channels[] = {
	STM401_AXIS_CHANNEL(IIO_ACCEL, X, 0),
	STM401_AXIS_CHANNEL(IIO_ACCEL, Y, 1),
	STM401_AXIS_CHANNEL(IIO_ACCEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec stm401_gyro_channels[] = {
	STM401_AXIS_CHANNEL(IIO_ANGL_VEL, X, 0),
	STM401_AXIS_CHANNEL(IIO_ANGL_VEL, Y, 1),
	STM401_AXIS_CHANNEL(IIO_ANGL_VEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec stm401_magn_channels[] = {
	STM401_AXIS_CHANNEL(IIO_MAGN, X, 0),
	STM401_AXIS_CHANNEL(IIO_MAGN, Y, 1),
	STM401_AXIS_CHANNEL(IIO_MAGN, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec stm401_light_channels[] = {
	STM401_SCALAR_CHANNEL(IIO_LIGHT, 'u', 0),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec stm401_proximity_channels[] = {
	STM401_SCALAR_CHANNEL(IIO_PROXIMITY, 'u', 0),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec stm401_temp_channels[] = {
	STM401_SCALAR_CHANNEL(IIO_TEMP, 's', 0),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec stm401_quat_channels[] = {
	STM401_QUAT_CHANNEL,
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec stm401_steps_channels[] = {
	STM401_SCALAR_CHANNEL(IIO_STEPS, 'u', BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct stm401_sensor_info stm401_sensor_infos[STM401_NUM_SENSORS] = {
	[STM401_ACCEL] = {
		.name = "stm401-accel",
		.mask = STM401_M_ACCEL,
		.data_reg = STM401_REG_ACCEL_X,
		.read_len = 6,
		.rate_reg = STM401_REG_ACCEL_UPDATE_RATE,
		.channels = stm401_accel_channels,
		.num_channels = ARRAY_SIZE(stm401_accel_channels),
		.scan_values = 3,
		.scale_type = IIO_VAL_INT_PLUS_NANO,
		.scale_val = 0,
		.scale_val2 = 9576806,
	},
	[STM401_GYRO] = {
		.name = "stm401-gyro",
		.mask = STM401_M_GYRO,
		.data_reg = STM401_REG_GYRO_X,
		.read_len = 6,
		.rate_reg = STM401_REG_GYRO_UPDATE_RATE,
		.channels = stm401_gyro_channels,
		.num_channels = ARRAY_SIZE(stm401_gyro_channels),
		.scan_values = 3,
		.scale_type = IIO_VAL_INT_PLUS_NANO,
		.scale_val = 0,
		.scale_val2 = 1065264,
	},
	[STM401_MAGN] = {
		.name = "stm401-magn",
		.mask = STM401_M_ECOMPASS,
		.data_reg = STM401_REG_MAG_HX,
		.read_len = 13,
		.rate_reg = STM401_REG_MAG_UPDATE_RATE,
		.channels = stm401_magn_channels,
		.num_channels = ARRAY_SIZE(stm401_magn_channels),
		.scan_values = 3,
		.scale_type = IIO_VAL_FRACTIONAL,
		.scale_val = 1,
		.scale_val2 = 1600,
	},
	[STM401_LIGHT] = {
		.name = "stm401-light",
		.mask = STM401_M_ALS,
		.data_reg = STM401_REG_ALS_LUX,
		.read_len = 2,
		.channels = stm401_light_channels,
		.num_channels = ARRAY_SIZE(stm401_light_channels),
		.scan_values = 1,
		.scale_type = IIO_VAL_INT,
		.scale_val = 1,
	},
	[STM401_PROXIMITY] = {
		.name = "stm401-proximity",
		.wake = true,
		.mask = STM401_M_PROXIMITY,
		.data_reg = STM401_REG_PROXIMITY,
		.read_len = 1,
		.channels = stm401_proximity_channels,
		.num_channels = ARRAY_SIZE(stm401_proximity_channels),
		.scan_values = 1,
		.scale_type = IIO_VAL_INT,
		.scale_val = 1,
	},
	[STM401_TEMP] = {
		.name = "stm401-temp",
		.on_change = true,
		.mask = STM401_M_TEMPERATURE,
		.data_reg = STM401_REG_TEMPERATURE,
		.read_len = 2,
		.channels = stm401_temp_channels,
		.num_channels = ARRAY_SIZE(stm401_temp_channels),
		.scan_values = 1,
		.scale_type = IIO_VAL_INT,
		.scale_val = 100,
	},
	[STM401_GAME_ROTATION] = {
		.name = "stm401-game-rotation",
		.mask = STM401_M_QUAT_6AXIS,
		.data_reg = STM401_REG_QUATERNION,
		.read_len = 16,
		.offset = 0,
		.rate_reg = STM401_REG_QUAT_6AXIS_UPDATE_RATE,
		.channels = stm401_quat_channels,
		.num_channels = ARRAY_SIZE(stm401_quat_channels),
		.scan_values = 4,
		.scale_type = IIO_VAL_FRACTIONAL,
		.scale_val = 1,
		.scale_val2 = 32768,
	},
	[STM401_ROTATION] = {
		.name = "stm401-rotation",
		.mask = STM401_M_QUAT_9AXIS,
		.data_reg = STM401_REG_QUATERNION,
		.read_len = 16,
		.offset = 8,
		.rate_reg = STM401_REG_QUAT_9AXIS_UPDATE_RATE,
		.channels = stm401_quat_channels,
		.num_channels = ARRAY_SIZE(stm401_quat_channels),
		.scan_values = 4,
		.scale_type = IIO_VAL_FRACTIONAL,
		.scale_val = 1,
		.scale_val2 = 32768,
	},
	[STM401_STEPS] = {
		.name = "stm401-steps",
		.on_change = true,
		.mask = STM401_M_STEP_COUNTER,
		.data_reg = STM401_REG_STEP_COUNTER,
		.read_len = 8,
		.rate_reg = STM401_REG_STEP_COUNTER_UPDATE_RATE,
		.channels = stm401_steps_channels,
		.num_channels = ARRAY_SIZE(stm401_steps_channels),
		.scan_values = 1,
		.scale_type = IIO_VAL_INT,
		.scale_val = 1,
	},
};

static const struct stm401_gesture_info stm401_gesture_infos[STM401_NUM_GESTURES] = {
	[STM401_GESTURE_CHOPCHOP] = {
		.label = "chopchop",
		.mask = STM401_M_CHOPCHOP,
		.data_reg = STM401_REG_CHOPCHOP,
		.read_len = 2,
		.key = KEY_LIGHTS_TOGGLE,
	},
	[STM401_GESTURE_CAMERA] = {
		.label = "camera",
		.mask = STM401_M_CAMERA_ACT,
		.data_reg = STM401_REG_CAMERA,
		.read_len = 2,
		.key = KEY_CAMERA,
	},
	[STM401_GESTURE_LIFT] = {
		.label = "lift",
		.mask = STM401_M_LIFT,
		.data_reg = STM401_REG_LIFT,
		.read_len = 12,
		.key = KEY_WAKEUP,
	},
	[STM401_GESTURE_SIGNIFICANT_MOTION] = {
		.label = "significant-motion",
		.mask = STM401_M_SIM,
		.data_reg = STM401_REG_SIM,
		.read_len = 2,
	},
	[STM401_GESTURE_FLAT_UP] = {
		.label = "flat-up",
		.mask = STM401_M_FLATUP,
		.stateful = true,
		.data_reg = STM401_REG_FLAT_DATA,
		.read_len = 1,
	},
	[STM401_GESTURE_FLAT_DOWN] = {
		.label = "flat-down",
		.mask = STM401_M_FLATDOWN,
		.stateful = true,
		.data_reg = STM401_REG_FLAT_DATA,
		.read_len = 1,
	},
	[STM401_GESTURE_STOWED] = {
		.label = "stowed",
		.mask = STM401_M_STOWED,
		.stateful = true,
		.data_reg = STM401_REG_STOWED,
		.read_len = 1,
	},
};

static const struct iio_event_spec stm401_gesture_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_NONE,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec stm401_state_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_EITHER,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

#define STM401_GESTURE_CHANNEL(_index, _event)					\
{										\
	.type = IIO_GESTURE,							\
	.indexed = 1,								\
	.channel = _index,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.event_spec = &(_event),						\
	.num_event_specs = 1,							\
	.scan_index = -1,							\
}

static const struct iio_chan_spec stm401_gesture_channels[] = {
	STM401_GESTURE_CHANNEL(STM401_GESTURE_CHOPCHOP, stm401_gesture_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_CAMERA, stm401_gesture_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_LIFT, stm401_gesture_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_SIGNIFICANT_MOTION, stm401_gesture_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_FLAT_UP, stm401_state_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_FLAT_DOWN, stm401_state_event),
	STM401_GESTURE_CHANNEL(STM401_GESTURE_STOWED, stm401_state_event),
};

static const u16 stm401_default_lux_table[] = { 6, 70, 86, 1174, 2800, 8000 };
static const u8 stm401_default_brightness_table[] = { 12, 20, 25, 55, 80, 111, 165 };

static void stm401_wake(struct stm401_hub *hub)
{
	int timeout = STM401_WAKE_TIMEOUT_US;

	if (!hub->lowpower)
		return;

	gpiod_set_value_cansleep(hub->wake_gpio, 1);

	while (!gpiod_get_value_cansleep(hub->wake_ack_gpio) && timeout--)
		udelay(1);

	if (timeout <= 0)
		dev_err(hub->dev, "sensor hub wakeup timeout\n");
}

static void stm401_sleep(struct stm401_hub *hub)
{
	if (!hub->lowpower)
		return;

	gpiod_set_value_cansleep(hub->wake_gpio, 0);
	udelay(1);
}

static int stm401_write(struct stm401_hub *hub, const u8 *buf, int len)
{
	int tries = 0;
	int ret;

	do {
		ret = i2c_master_send(hub->client, buf, len);
		if (ret == len)
			return 0;

		msleep(STM401_I2C_RETRY_DELAY_MS);
	} while (++tries < STM401_I2C_RETRIES);

	dev_err(hub->dev, "write of reg 0x%02x failed: %d\n", buf[0], ret);

	return ret < 0 ? ret : -EIO;
}

static int stm401_read(struct stm401_hub *hub, u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = hub->client->addr,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = hub->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int tries = 0;
	int ret;

	do {
		ret = i2c_transfer(hub->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			return 0;

		msleep(STM401_I2C_RETRY_DELAY_MS);
	} while (++tries < STM401_I2C_RETRIES);

	dev_err(hub->dev, "read of reg 0x%02x failed: %d\n", reg, ret);

	return ret < 0 ? ret : -EIO;
}

static int stm401_write_u8(struct stm401_hub *hub, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };

	return stm401_write(hub, buf, sizeof(buf));
}

static int stm401_write_masks(struct stm401_hub *hub)
{
	u8 nonwake[4] = {
		STM401_REG_NONWAKESENSOR_CONFIG,
		hub->nonwake_mask & 0xff,
		(hub->nonwake_mask >> 8) & 0xff,
		(hub->nonwake_mask >> 16) & 0xff,
	};
	u8 wake[3] = {
		STM401_REG_WAKESENSOR_CONFIG,
		hub->wake_mask & 0xff,
		hub->wake_mask >> 8,
	};
	int ret;

	ret = stm401_write(hub, nonwake, sizeof(nonwake));
	if (ret)
		return ret;

	return stm401_write(hub, wake, sizeof(wake));
}

static int stm401_update_masks(struct stm401_hub *hub)
{
	u32 nonwake = 0;
	u16 wake = 0;
	int i;

	for (i = 0; i < STM401_NUM_SENSORS; i++) {
		struct stm401_sensor *sensor = &hub->sensors[i];

		if (!sensor->users)
			continue;

		if (sensor->info->wake)
			wake |= sensor->info->mask;
		else
			nonwake |= sensor->info->mask;
	}

	for (i = 0; i < STM401_NUM_GESTURES; i++) {
		if (hub->gesture_events & BIT(i))
			wake |= stm401_gesture_infos[i].mask;
	}

	if (hub->input_open)
		wake |= STM401_M_COVER;

	if (nonwake == hub->nonwake_mask && wake == hub->wake_mask)
		return 0;

	hub->nonwake_mask = nonwake;
	hub->wake_mask = wake;

	if (hub->suspended)
		return 0;

	return stm401_write_masks(hub);
}

static void stm401_detect_lowpower(struct stm401_hub *hub)
{
	u8 buf[2];
	int timeout = STM401_WAKE_TIMEOUT_US;
	int ret;

	hub->lowpower = false;

	if (!hub->wake_gpio || !hub->wake_ack_gpio)
		return;

	gpiod_set_value_cansleep(hub->wake_gpio, 1);

	while (!gpiod_get_value_cansleep(hub->wake_ack_gpio) && timeout--)
		udelay(1);

	ret = stm401_read(hub, STM401_REG_LOWPOWER, buf, sizeof(buf));
	if (ret) {
		dev_err(hub->dev, "failed to read low power support: %d\n", ret);
		return;
	}

	if (buf[1] != 1)
		return;

	ret = stm401_write_u8(hub, STM401_REG_LOWPOWER, 1);
	if (ret)
		return;

	hub->lowpower = true;
}

static int stm401_load_brightness_table(struct stm401_hub *hub)
{
	u8 buf[2 * STM401_LIGHTING_TABLE_SIZE + 1];
	int ret;
	int i;

	buf[0] = STM401_REG_LUX_TABLE;
	for (i = 0; i < STM401_LIGHTING_TABLE_SIZE; i++)
		put_unaligned_be16(hub->lux_table[i], &buf[2 * i + 1]);

	ret = stm401_write(hub, buf, 2 * STM401_LIGHTING_TABLE_SIZE + 1);
	if (ret)
		return ret;

	buf[0] = STM401_REG_BRIGHTNESS_TABLE;
	memcpy(&buf[1], hub->brightness_table, STM401_LIGHTING_TABLE_SIZE);

	return stm401_write(hub, buf, STM401_LIGHTING_TABLE_SIZE + 1);
}

static int stm401_write_prox_settings(struct stm401_hub *hub)
{
	u8 buf[8];

	buf[0] = STM401_REG_PROX_SETTINGS;
	put_unaligned_be16(hub->prox_settings[0], &buf[1]);
	put_unaligned_be16(hub->prox_settings[1], &buf[3]);
	put_unaligned_be16(hub->prox_settings[2], &buf[5]);
	buf[7] = hub->prox_settings[3];

	return stm401_write(hub, buf, sizeof(buf));
}

static int stm401_write_time(struct stm401_hub *hub)
{
	u8 buf[5];

	buf[0] = STM401_REG_AP_POSIX_TIME;
	put_unaligned_be32(lower_32_bits(ktime_get_real_seconds()), &buf[1]);

	return stm401_write(hub, buf, sizeof(buf));
}

static void stm401_reset(struct stm401_hub *hub)
{
	gpiod_set_value_cansleep(hub->reset_gpio, 1);
	msleep(STM401_RESET_DELAY_MS);
	gpiod_set_value_cansleep(hub->reset_gpio, 0);
	msleep(STM401_RESET_DELAY_MS);
}

static int stm401_reset_and_init(struct stm401_hub *hub)
{
	u8 rev;
	u8 status[3];
	int attempts = 0;
	int ret;
	int i;

	gpiod_set_value_cansleep(hub->boot_gpio, 0);

	do {
		stm401_reset(hub);
		stm401_detect_lowpower(hub);

		ret = stm401_read(hub, STM401_REG_REV_ID, &rev, 1);
		if (ret)
			msleep(STM401_RESET_RETRY_DELAY_MS);
	} while (ret && ++attempts < STM401_RESET_ATTEMPTS);

	if (ret) {
		dev_err(hub->dev, "hub not responding after reset: %d\n", ret);
		goto out;
	}

	dev_info(hub->dev, "firmware revision 0x%02x, low power %s\n", rev, hub->lowpower ? "on" : "off");

	for (i = 0; i < STM401_NUM_SENSORS; i++) {
		const struct stm401_sensor_info *info = hub->sensors[i].info;

		if (!info->rate_reg)
			continue;

		ret = stm401_write_u8(hub, info->rate_reg, hub->sensors[i].delay_ms);
		if (ret)
			goto out;
	}

	ret = stm401_write_masks(hub);
	if (ret)
		goto out;

	ret = stm401_write_u8(hub, STM401_REG_MOTION_DUR, 0);
	if (ret)
		goto out;

	ret = stm401_write_u8(hub, STM401_REG_ZRMOTION_DUR, 0);
	if (ret)
		goto out;

	stm401_read(hub, STM401_REG_INTERRUPT_STATUS, status, 3);
	stm401_read(hub, STM401_REG_WAKESENSOR_STATUS, status, 2);

	ret = stm401_write_prox_settings(hub);
	if (ret)
		goto out;

	ret = stm401_load_brightness_table(hub);
	if (ret)
		goto out;

	ret = stm401_write_time(hub);

out:
	stm401_sleep(hub);

	return ret;
}

static void stm401_parse_sample(struct stm401_sensor *sensor, const u8 *data)
{
	const struct stm401_sensor_info *info = sensor->info;
	const u8 *p = data + info->offset;
	int i;

	switch (info->data_reg) {
	case STM401_REG_PROXIMITY:
		sensor->values[0] = p[0];
		break;
	case STM401_REG_ALS_LUX:
		sensor->values[0] = get_unaligned_be16(p);
		break;
	case STM401_REG_STEP_COUNTER:
		sensor->values[0] = ((u32)get_unaligned_be16(p + 2) << 16) | get_unaligned_be16(p);
		break;
	default:
		for (i = 0; i < info->scan_values; i++)
			sensor->values[i] = (s16)get_unaligned_be16(p + 2 * i);
		break;
	}
}

static void stm401_push_sample(struct stm401_sensor *sensor, s64 timestamp)
{
	IIO_DECLARE_BUFFER_WITH_TS(s32, scan, STM401_MAX_CHANNELS) = { };

	sensor->valid = true;
	complete_all(&sensor->sample);

	if (!iio_buffer_enabled(sensor->indio_dev))
		return;

	memcpy(scan, sensor->values, sensor->info->scan_values * sizeof(s32));
	iio_push_to_buffers_with_ts(sensor->indio_dev, scan, sizeof(scan), timestamp);
}

static int stm401_handle_sensor(struct stm401_hub *hub, struct stm401_sensor *sensor, s64 timestamp)
{
	u8 data[STM401_MAX_DATA];
	int ret;

	ret = stm401_read(hub, sensor->info->data_reg, data, min_t(int, sensor->info->read_len, sizeof(data)));
	if (ret)
		return ret;

	stm401_parse_sample(sensor, data);
	stm401_push_sample(sensor, timestamp);

	return 0;
}

static void stm401_handle_quaternions(struct stm401_hub *hub, u32 status, s64 timestamp)
{
	struct stm401_sensor *game = &hub->sensors[STM401_GAME_ROTATION];
	struct stm401_sensor *rot = &hub->sensors[STM401_ROTATION];
	u8 data[16];

	if (stm401_read(hub, STM401_REG_QUATERNION, data, sizeof(data)))
		return;

	if (status & STM401_M_QUAT_6AXIS) {
		stm401_parse_sample(game, data);
		stm401_push_sample(game, timestamp);
	}

	if (status & STM401_M_QUAT_9AXIS) {
		stm401_parse_sample(rot, data);
		stm401_push_sample(rot, timestamp);
	}
}

static irqreturn_t stm401_irq_thread(int irq, void *data)
{
	struct stm401_hub *hub = data;
	s64 timestamp = iio_get_time_ns(hub->sensors[STM401_ACCEL].indio_dev);
	u8 buf[3];
	u32 status;
	int i;

	mutex_lock(&hub->lock);

	if (hub->suspended)
		goto out;

	stm401_wake(hub);

	if (stm401_read(hub, STM401_REG_INTERRUPT_STATUS, buf, sizeof(buf)))
		goto out_sleep;

	status = buf[0] | (buf[1] << 8) | (buf[2] << 16);

	for (i = 0; i < STM401_NUM_SENSORS; i++) {
		struct stm401_sensor *sensor = &hub->sensors[i];

		if (sensor->info->wake || !(status & sensor->info->mask))
			continue;

		if (i == STM401_GAME_ROTATION || i == STM401_ROTATION)
			continue;

		stm401_handle_sensor(hub, sensor, timestamp);
	}

	if (status & (STM401_M_QUAT_6AXIS | STM401_M_QUAT_9AXIS))
		stm401_handle_quaternions(hub, status, timestamp);

out_sleep:
	stm401_sleep(hub);
out:
	mutex_unlock(&hub->lock);

	return IRQ_HANDLED;
}

static void stm401_report_gesture(struct stm401_hub *hub, int id, s64 timestamp)
{
	const struct stm401_gesture_info *info = &stm401_gesture_infos[id];
	u8 data[12];
	u64 code;

	if (stm401_read(hub, info->data_reg, data, info->read_len))
		return;

	if (info->stateful) {
		code = IIO_UNMOD_EVENT_CODE(IIO_GESTURE, id, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_EITHER);
		hub->gesture_raw[id] = !!data[0];
	} else {
		code = IIO_UNMOD_EVENT_CODE(IIO_GESTURE, id, IIO_EV_TYPE_GESTURE, IIO_EV_DIR_NONE);
		hub->gesture_raw[id]++;
	}

	dev_dbg(hub->dev, "gesture %s: %*ph\n", info->label, info->read_len, data);

	if (hub->gesture_events & BIT(id))
		iio_push_event(hub->gestures, code, timestamp);

	if (info->key && hub->input) {
		input_report_key(hub->input, info->key, 1);
		input_sync(hub->input);
		input_report_key(hub->input, info->key, 0);
		input_sync(hub->input);
	}
}

static void stm401_report_cover(struct stm401_hub *hub)
{
	u8 state;

	if (stm401_read(hub, STM401_REG_COVER_DATA, &state, 1))
		return;

	input_report_switch(hub->input, SW_LID, !!state);
	input_sync(hub->input);
}

static void stm401_handle_wake_irq(struct stm401_hub *hub)
{
	s64 timestamp = iio_get_time_ns(hub->gestures);
	u8 buf[3];
	u16 status;
	int i;

	stm401_wake(hub);

	if (stm401_read(hub, STM401_REG_WAKESENSOR_STATUS, buf, 2))
		goto out;

	status = buf[0] | (buf[1] << 8);

	stm401_read(hub, STM401_REG_ALGO_INT_STATUS, buf, 3);
	stm401_read(hub, STM401_REG_GENERIC_INT_STATUS, buf, 1);

	if (status & STM401_M_LOG_MSG) {
		if (!stm401_read(hub, STM401_REG_ERROR_STATUS, hub->buf, STM401_ERROR_STATUS_SIZE)) {
			hub->buf[STM401_ERROR_STATUS_SIZE] = 0;
			dev_err(hub->dev, "hub error: %s\n", hub->buf);
		}
	}

	if (status & STM401_M_HUB_RESET) {
		dev_err(hub->dev, "hub requested a reset\n");
		stm401_reset_and_init(hub);
		return;
	}

	if (status & STM401_M_PROXIMITY)
		stm401_handle_sensor(hub, &hub->sensors[STM401_PROXIMITY], timestamp);

	if ((status & STM401_M_COVER) && hub->input)
		stm401_report_cover(hub);

	for (i = 0; i < STM401_NUM_GESTURES; i++) {
		if (status & stm401_gesture_infos[i].mask)
			stm401_report_gesture(hub, i, timestamp);
	}

	if (status & STM401_M_SIM) {
		hub->gesture_events &= ~BIT(STM401_GESTURE_SIGNIFICANT_MOTION);
		stm401_update_masks(hub);
	}

out:
	stm401_sleep(hub);
}

static irqreturn_t stm401_wake_irq_handler(int irq, void *data)
{
	struct stm401_hub *hub = data;

	pm_wakeup_event(hub->dev, 1000);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t stm401_wake_irq_thread(int irq, void *data)
{
	struct stm401_hub *hub = data;

	mutex_lock(&hub->lock);

	if (hub->suspended)
		hub->wake_pending = true;
	else
		stm401_handle_wake_irq(hub);

	mutex_unlock(&hub->lock);

	return IRQ_HANDLED;
}

static int stm401_sensor_get(struct stm401_sensor *sensor)
{
	struct stm401_hub *hub = sensor->hub;
	int ret;

	mutex_lock(&hub->lock);

	if (sensor->users++) {
		mutex_unlock(&hub->lock);
		return 0;
	}

	sensor->valid = false;
	reinit_completion(&sensor->sample);

	stm401_wake(hub);
	ret = stm401_update_masks(hub);
	stm401_sleep(hub);

	if (ret)
		sensor->users--;

	mutex_unlock(&hub->lock);

	return ret;
}

static void stm401_sensor_put(struct stm401_sensor *sensor)
{
	struct stm401_hub *hub = sensor->hub;

	mutex_lock(&hub->lock);

	if (!--sensor->users) {
		stm401_wake(hub);
		stm401_update_masks(hub);
		stm401_sleep(hub);
	}

	mutex_unlock(&hub->lock);
}

static int stm401_read_sample(struct stm401_sensor *sensor, const struct iio_chan_spec *chan, int *val, int *val2)
{
	struct stm401_hub *hub = sensor->hub;
	int index = chan->scan_index;
	long timeout;
	int ret;
	int i;

	ret = stm401_sensor_get(sensor);
	if (ret)
		return ret;

	if ((sensor->info->wake || sensor->info->on_change) && !sensor->valid) {
		mutex_lock(&hub->lock);
		stm401_wake(hub);
		ret = stm401_handle_sensor(hub, sensor, iio_get_time_ns(sensor->indio_dev));
		stm401_sleep(hub);
		mutex_unlock(&hub->lock);
	}

	if (!ret) {
		timeout = wait_for_completion_interruptible_timeout(&sensor->sample,
								    msecs_to_jiffies(STM401_SAMPLE_TIMEOUT_MS));
		if (timeout == 0)
			ret = -ETIMEDOUT;
		else if (timeout < 0)
			ret = timeout;
	}

	if (!ret) {
		mutex_lock(&hub->lock);

		if (chan->type == IIO_ROT) {
			for (i = 0; i < 4; i++)
				val[i] = sensor->values[i];

			*val2 = 4;
			ret = IIO_VAL_INT_MULTIPLE;
		} else {
			*val = sensor->values[index];
			ret = IIO_VAL_INT;
		}

		mutex_unlock(&hub->lock);
	}

	stm401_sensor_put(sensor);

	return ret;
}

static int stm401_read_raw_multi(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int max_len,
				 int *vals, int *val_len, long mask)
{
	struct stm401_sensor *sensor = iio_device_get_drvdata(indio_dev);
	const struct stm401_sensor_info *info = sensor->info;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (max_len < 4)
			return -EINVAL;

		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		ret = stm401_read_sample(sensor, chan, vals, val_len);

		iio_device_release_direct(indio_dev);

		if (ret == IIO_VAL_INT)
			*val_len = 1;

		return ret;

	case IIO_CHAN_INFO_SCALE:
		vals[0] = info->scale_val;
		vals[1] = info->scale_val2;
		*val_len = 2;

		return info->scale_type;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!info->rate_reg)
			return -EINVAL;

		vals[0] = 1000;
		vals[1] = sensor->delay_ms;
		*val_len = 2;

		return IIO_VAL_FRACTIONAL;

	default:
		return -EINVAL;
	}
}

static int stm401_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct stm401_sensor *sensor = iio_device_get_drvdata(indio_dev);
	struct stm401_hub *hub = sensor->hub;
	u64 micro_hz;
	u8 delay_ms;
	int ret = 0;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || !sensor->info->rate_reg)
		return -EINVAL;

	if (val < 0 || val2 < 0 || (val == 0 && val2 == 0))
		return -EINVAL;

	micro_hz = (u64)val * MICRO + val2;
	delay_ms = clamp_t(u64, DIV_ROUND_CLOSEST_ULL(1000ULL * MICRO, micro_hz), 1, U8_MAX);

	mutex_lock(&hub->lock);

	sensor->delay_ms = delay_ms;

	if (!hub->suspended) {
		stm401_wake(hub);
		ret = stm401_write_u8(hub, sensor->info->rate_reg, delay_ms);
		stm401_sleep(hub);
	}

	mutex_unlock(&hub->lock);

	return ret;
}

static int stm401_write_raw_get_fmt(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, long mask)
{
	return IIO_VAL_INT_PLUS_MICRO;
}

static const struct iio_info stm401_sensor_iio_info = {
	.read_raw_multi = stm401_read_raw_multi,
	.write_raw = stm401_write_raw,
	.write_raw_get_fmt = stm401_write_raw_get_fmt,
};

static int stm401_buffer_preenable(struct iio_dev *indio_dev)
{
	return stm401_sensor_get(iio_device_get_drvdata(indio_dev));
}

static int stm401_buffer_postdisable(struct iio_dev *indio_dev)
{
	stm401_sensor_put(iio_device_get_drvdata(indio_dev));

	return 0;
}

static const struct iio_buffer_setup_ops stm401_buffer_ops = {
	.preenable = stm401_buffer_preenable,
	.postdisable = stm401_buffer_postdisable,
};

static int stm401_gesture_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2,
				   long mask)
{
	struct stm401_hub *hub = iio_device_get_drvdata(indio_dev);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&hub->lock);
	*val = hub->gesture_raw[chan->channel];
	mutex_unlock(&hub->lock);

	return IIO_VAL_INT;
}

static int stm401_gesture_read_label(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, char *label)
{
	return sysfs_emit(label, "%s\n", stm401_gesture_infos[chan->channel].label);
}

static int stm401_gesture_read_event_config(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
					    enum iio_event_type type, enum iio_event_direction dir)
{
	struct stm401_hub *hub = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&hub->lock);
	ret = !!(hub->gesture_events & BIT(chan->channel));
	mutex_unlock(&hub->lock);

	return ret;
}

static int stm401_gesture_write_event_config(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
					     enum iio_event_type type, enum iio_event_direction dir, bool state)
{
	struct stm401_hub *hub = iio_device_get_drvdata(indio_dev);
	const struct stm401_gesture_info *info = &stm401_gesture_infos[chan->channel];
	u8 data;
	int ret;

	mutex_lock(&hub->lock);

	if (state)
		hub->gesture_events |= BIT(chan->channel);
	else
		hub->gesture_events &= ~BIT(chan->channel);

	stm401_wake(hub);

	ret = stm401_update_masks(hub);

	if (!ret && state && info->stateful && !hub->suspended && !stm401_read(hub, info->data_reg, &data, 1))
		hub->gesture_raw[chan->channel] = !!data;

	stm401_sleep(hub);

	mutex_unlock(&hub->lock);

	return ret;
}

static const struct iio_info stm401_gesture_iio_info = {
	.read_raw = stm401_gesture_read_raw,
	.read_label = stm401_gesture_read_label,
	.read_event_config = stm401_gesture_read_event_config,
	.write_event_config = stm401_gesture_write_event_config,
};

static int stm401_input_open(struct input_dev *input)
{
	struct stm401_hub *hub = input_get_drvdata(input);
	int ret;

	mutex_lock(&hub->lock);

	hub->input_open = true;

	stm401_wake(hub);
	ret = stm401_update_masks(hub);
	if (!ret && !hub->suspended)
		stm401_report_cover(hub);
	stm401_sleep(hub);

	mutex_unlock(&hub->lock);

	return ret;
}

static void stm401_input_close(struct input_dev *input)
{
	struct stm401_hub *hub = input_get_drvdata(input);

	mutex_lock(&hub->lock);

	hub->input_open = false;

	stm401_wake(hub);
	stm401_update_masks(hub);
	stm401_sleep(hub);

	mutex_unlock(&hub->lock);
}

static int stm401_register_sensor(struct stm401_hub *hub, int id)
{
	const struct stm401_sensor_info *info = &stm401_sensor_infos[id];
	struct stm401_sensor *sensor = &hub->sensors[id];
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(hub->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	sensor->hub = hub;
	sensor->info = info;
	sensor->indio_dev = indio_dev;
	sensor->delay_ms = STM401_DEFAULT_DELAY_MS;
	init_completion(&sensor->sample);

	indio_dev->name = info->name;
	indio_dev->info = &stm401_sensor_iio_info;
	indio_dev->channels = info->channels;
	indio_dev->num_channels = info->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(indio_dev, sensor);

	return devm_iio_kfifo_buffer_setup(hub->dev, indio_dev, &stm401_buffer_ops);
}

static int stm401_register_gestures(struct stm401_hub *hub)
{
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(hub->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name = "stm401-gestures";
	indio_dev->info = &stm401_gesture_iio_info;
	indio_dev->channels = stm401_gesture_channels;
	indio_dev->num_channels = ARRAY_SIZE(stm401_gesture_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(indio_dev, hub);

	hub->gestures = indio_dev;

	return 0;
}

static int stm401_register_input(struct stm401_hub *hub)
{
	struct input_dev *input;
	int i;

	input = devm_input_allocate_device(hub->dev);
	if (!input)
		return -ENOMEM;

	input->name = "STM401 sensor hub gestures";
	input->id.bustype = BUS_I2C;
	input->open = stm401_input_open;
	input->close = stm401_input_close;
	input_set_drvdata(input, hub);

	for (i = 0; i < STM401_NUM_GESTURES; i++) {
		if (stm401_gesture_infos[i].key)
			input_set_capability(input, EV_KEY, stm401_gesture_infos[i].key);
	}

	input_set_capability(input, EV_SW, SW_LID);

	hub->input = input;

	return input_register_device(input);
}

static int stm401_parse_dt(struct stm401_hub *hub)
{
	struct device *dev = hub->dev;
	u32 table[STM401_LIGHTING_TABLE_SIZE];
	int lux_count;
	int brightness_count;
	int i;

	lux_count = device_property_count_u32(dev, "motorola,lux-table");
	brightness_count = device_property_count_u32(dev, "motorola,brightness-table");

	if (lux_count <= 0 || brightness_count <= 0) {
		lux_count = ARRAY_SIZE(stm401_default_lux_table);
		brightness_count = ARRAY_SIZE(stm401_default_brightness_table);

		memcpy(hub->lux_table, stm401_default_lux_table, sizeof(stm401_default_lux_table));
		memcpy(hub->brightness_table, stm401_default_brightness_table, sizeof(stm401_default_brightness_table));
	} else {
		if (lux_count >= STM401_LIGHTING_TABLE_SIZE - 1 || brightness_count != lux_count + 1)
			return dev_err_probe(dev, -EINVAL, "invalid lux/brightness tables\n");

		device_property_read_u32_array(dev, "motorola,lux-table", table, lux_count);
		for (i = 0; i < lux_count; i++)
			hub->lux_table[i] = table[i];

		device_property_read_u32_array(dev, "motorola,brightness-table", table, brightness_count);
		for (i = 0; i < brightness_count; i++)
			hub->brightness_table[i] = table[i];
	}

	hub->lux_table[lux_count] = 0xffff;

	hub->prox_settings[0] = 0x006e;
	hub->prox_settings[1] = 0x0050;
	hub->prox_settings[2] = 0x0064;
	hub->prox_settings[3] = 0x04;
	device_property_read_u32_array(dev, "motorola,prox-settings", hub->prox_settings, ARRAY_SIZE(hub->prox_settings));

	return 0;
}

static void stm401_disable_regulators(void *data)
{
	struct stm401_hub *hub = data;

	regulator_bulk_disable(ARRAY_SIZE(hub->supplies), hub->supplies);
}

static int stm401_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct stm401_hub *hub;
	int ret;
	int i;

	hub = devm_kzalloc(dev, sizeof(*hub), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	hub->client = client;
	hub->dev = dev;
	i2c_set_clientdata(client, hub);

	ret = devm_mutex_init(dev, &hub->lock);
	if (ret)
		return ret;

	ret = stm401_parse_dt(hub);
	if (ret)
		return ret;

	hub->supplies[0].supply = "vio";
	hub->supplies[1].supply = "vdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(hub->supplies), hub->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	hub->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(hub->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(hub->reset_gpio), "failed to get reset gpio\n");

	hub->boot_gpio = devm_gpiod_get_optional(dev, "boot", GPIOD_OUT_LOW);
	if (IS_ERR(hub->boot_gpio))
		return dev_err_probe(dev, PTR_ERR(hub->boot_gpio), "failed to get boot gpio\n");

	hub->wake_gpio = devm_gpiod_get_optional(dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(hub->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(hub->wake_gpio), "failed to get wakeup gpio\n");

	hub->wake_ack_gpio = devm_gpiod_get_optional(dev, "wakeup-ack", GPIOD_IN);
	if (IS_ERR(hub->wake_ack_gpio))
		return dev_err_probe(dev, PTR_ERR(hub->wake_ack_gpio), "failed to get wakeup-ack gpio\n");

	hub->irq = fwnode_irq_get_byname(dev_fwnode(dev), "data");
	if (hub->irq < 0)
		return dev_err_probe(dev, hub->irq, "failed to get data irq\n");

	hub->wake_irq = fwnode_irq_get_byname(dev_fwnode(dev), "wakeup");
	if (hub->wake_irq < 0)
		return dev_err_probe(dev, hub->wake_irq, "failed to get wakeup irq\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(hub->supplies), hub->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, stm401_disable_regulators, hub);
	if (ret)
		return ret;

	for (i = 0; i < STM401_NUM_SENSORS; i++) {
		ret = stm401_register_sensor(hub, i);
		if (ret)
			return ret;
	}

	ret = stm401_register_gestures(hub);
	if (ret)
		return ret;

	mutex_lock(&hub->lock);
	ret = stm401_reset_and_init(hub);
	mutex_unlock(&hub->lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise hub\n");

	ret = stm401_register_input(hub);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, hub->irq, NULL, stm401_irq_thread, IRQF_ONESHOT, "stm401", hub);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request data irq\n");

	ret = devm_request_threaded_irq(dev, hub->wake_irq, stm401_wake_irq_handler, stm401_wake_irq_thread,
					IRQF_ONESHOT, "stm401-wakeup", hub);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request wakeup irq\n");

	for (i = 0; i < STM401_NUM_SENSORS; i++) {
		ret = devm_iio_device_register(dev, hub->sensors[i].indio_dev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register %s\n", stm401_sensor_infos[i].name);
	}

	ret = devm_iio_device_register(dev, hub->gestures);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gestures\n");

	return 0;
}

static int stm401_suspend(struct device *dev)
{
	struct stm401_hub *hub = dev_get_drvdata(dev);

	disable_irq(hub->irq);

	mutex_lock(&hub->lock);
	hub->suspended = true;
	mutex_unlock(&hub->lock);

	if (device_may_wakeup(dev))
		enable_irq_wake(hub->wake_irq);

	return 0;
}

static int stm401_resume(struct device *dev)
{
	struct stm401_hub *hub = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(hub->wake_irq);

	mutex_lock(&hub->lock);

	hub->suspended = false;

	stm401_wake(hub);
	stm401_write_masks(hub);
	stm401_sleep(hub);

	if (hub->wake_pending) {
		hub->wake_pending = false;
		stm401_handle_wake_irq(hub);
	}

	mutex_unlock(&hub->lock);

	enable_irq(hub->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(stm401_pm_ops, stm401_suspend, stm401_resume);

static const struct of_device_id stm401_of_match[] = {
	{ .compatible = "motorola,stm401" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm401_of_match);

static const struct i2c_device_id stm401_id[] = {
	{ "stm401" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stm401_id);

static struct i2c_driver stm401_driver = {
	.driver = {
		.name = "stm401",
		.of_match_table = stm401_of_match,
		.pm = pm_sleep_ptr(&stm401_pm_ops),
	},
	.probe = stm401_probe,
	.id_table = stm401_id,
};
module_i2c_driver(stm401_driver);

MODULE_DESCRIPTION("Motorola STM401 sensor hub");
MODULE_LICENSE("GPL");
