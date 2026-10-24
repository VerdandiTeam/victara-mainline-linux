// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/interconnect-clk.h>
#include <linux/interconnect-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_opp.h>
#include <linux/pm_wakeup.h>

#include <dt-bindings/soc/qcom,krait-l2-cache.h>

#define KRAIT_L2_ICC_FIRST_ID	1000

static int krait_l2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct icc_clk_data *data;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get L2 clock\n");

	ret = devm_pm_opp_set_clkname(dev, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set OPP clock\n");

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add OPP table\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->clk = clk;
	data->name = "l2";
	data->master_id = MASTER_KRAIT_L2;
	data->slave_id = SLAVE_KRAIT_L2;
	data->opp = true;

	return devm_icc_clk_register(dev, KRAIT_L2_ICC_FIRST_ID, 1, data);
}

static int krait_l2_suspend(struct device *dev)
{
	device_set_awake_path(dev);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(krait_l2_pm_ops, krait_l2_suspend, NULL);

static const struct of_device_id krait_l2_match_table[] = {
	{ .compatible = "qcom,krait-l2-cache" },
	{}
};
MODULE_DEVICE_TABLE(of, krait_l2_match_table);

static struct platform_driver krait_l2_driver = {
	.probe = krait_l2_probe,
	.driver = {
		.name = "qcom-krait-l2-cache",
		.of_match_table = krait_l2_match_table,
		.pm = pm_sleep_ptr(&krait_l2_pm_ops),
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(krait_l2_driver);

MODULE_DESCRIPTION("Qualcomm Krait L2 cache scaling driver");
MODULE_LICENSE("GPL");
