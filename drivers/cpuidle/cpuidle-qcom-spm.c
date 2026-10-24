// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 *
 * SAW power controller driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <soc/qcom/spm.h>

#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>

#include "dt_idle_states.h"

struct cpuidle_qcom_spm_data {
	struct cpuidle_driver cpuidle_driver;
	struct spm_driver_data *spm;
};

static struct spm_driver_data *l2_spm;
static struct device *l2_pd_dev;
static DEFINE_RAW_SPINLOCK(l2_lock);
static unsigned int l2_cpus_down;
static bool l2_rpm_sleep;

static int qcom_pm_collapse(unsigned long int flags)
{
	if (flags == QCOM_SCM_CPU_PWR_DOWN_L2_OFF)
		flush_cache_all();

	qcom_scm_cpu_power_down(flags);

	/*
	 * Returns here only if there was a pending interrupt and we did not
	 * power down as a result.
	 */
	return -1;
}

static unsigned long qcom_l2_enter(bool s2idle)
{
	unsigned long flags = QCOM_SCM_CPU_PWR_DOWN_L2_ON;

	if (!l2_spm)
		return flags;

	raw_spin_lock(&l2_lock);

	if (++l2_cpus_down == num_online_cpus()) {
		if (s2idle && l2_pd_dev) {
			spm_set_low_power_mode(l2_spm, PM_SLEEP_MODE_PC);
			dev_pm_genpd_suspend(l2_pd_dev);
			l2_rpm_sleep = true;
			flags = QCOM_SCM_CPU_PWR_DOWN_L2_OFF;
		} else {
			spm_set_low_power_mode(l2_spm, PM_SLEEP_MODE_SPC);
			flags = QCOM_SCM_CPU_PWR_DOWN_L2_GDHS;
		}
	}

	raw_spin_unlock(&l2_lock);

	return flags;
}

static void qcom_l2_exit(void)
{
	if (!l2_spm)
		return;

	raw_spin_lock(&l2_lock);

	if (l2_cpus_down-- == num_online_cpus()) {
		if (l2_rpm_sleep) {
			dev_pm_genpd_resume(l2_pd_dev);
			l2_rpm_sleep = false;
		}

		spm_set_low_power_mode(l2_spm, PM_SLEEP_MODE_STBY);
	}

	raw_spin_unlock(&l2_lock);
}

static int qcom_cpu_spc(struct spm_driver_data *drv)
{
	unsigned long flags;
	int ret;

	spm_set_low_power_mode(drv, PM_SLEEP_MODE_SPC);

	flags = qcom_l2_enter(false);

	ret = cpu_suspend(flags, qcom_pm_collapse);

	qcom_l2_exit();

	/*
	 * ARM common code executes WFI without calling into our driver and
	 * if the SPM mode is not reset, then we may accidentally power down the
	 * cpu when we intended only to gate the cpu clock.
	 * Ensure the state is set to standby before returning.
	 */
	spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);

	return ret;
}

static __cpuidle int spm_enter_idle_state(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int idx)
{
	struct cpuidle_qcom_spm_data *data = container_of(drv, struct cpuidle_qcom_spm_data,
							  cpuidle_driver);

	return CPU_PM_CPU_IDLE_ENTER_PARAM(qcom_cpu_spc, idx, data->spm);
}

static __cpuidle int spm_enter_s2idle_state(struct cpuidle_device *dev,
					    struct cpuidle_driver *drv, int idx)
{
	struct cpuidle_qcom_spm_data *data = container_of(drv, struct cpuidle_qcom_spm_data,
							  cpuidle_driver);
	unsigned long flags;
	int ret;

	if (cpu_pm_enter())
		return -1;

	spm_set_low_power_mode(data->spm, PM_SLEEP_MODE_SPC);

	flags = qcom_l2_enter(true);

	ct_cpuidle_enter();
	ret = cpu_suspend(flags, qcom_pm_collapse);
	ct_cpuidle_exit();

	qcom_l2_exit();

	spm_set_low_power_mode(data->spm, PM_SLEEP_MODE_STBY);

	cpu_pm_exit();

	return ret ? -1 : idx;
}

static struct cpuidle_driver qcom_spm_idle_driver = {
	.name = "qcom_spm",
	.owner = THIS_MODULE,
	.states[0] = {
		.enter			= spm_enter_idle_state,
		.exit_latency		= 1,
		.target_residency	= 1,
		.power_usage		= UINT_MAX,
		.name			= "WFI",
		.desc			= "ARM WFI",
	}
};

static const struct of_device_id qcom_idle_state_match[] = {
	{ .compatible = "qcom,idle-state-spc", .data = spm_enter_idle_state },
	{ },
};

static int spm_cpuidle_register(struct device *cpuidle_dev, int cpu)
{
	struct platform_device *pdev;
	struct device_node *cpu_node, *saw_node;
	struct cpuidle_qcom_spm_data *data;
	int ret;

	cpu_node = of_cpu_device_node_get(cpu);
	if (!cpu_node)
		return -ENODEV;

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	of_node_put(cpu_node);
	if (!saw_node)
		return -ENODEV;

	pdev = of_find_device_by_node(saw_node);
	of_node_put(saw_node);
	if (!pdev)
		return -ENODEV;

	data = devm_kzalloc(cpuidle_dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		put_device(&pdev->dev);
		return -ENOMEM;
	}

	data->spm = dev_get_drvdata(&pdev->dev);
	put_device(&pdev->dev);
	if (!data->spm)
		return -EINVAL;

	data->cpuidle_driver = qcom_spm_idle_driver;
	data->cpuidle_driver.cpumask = (struct cpumask *)cpumask_of(cpu);

	ret = dt_init_idle_driver(&data->cpuidle_driver,
				  qcom_idle_state_match, 1);
	if (ret <= 0)
		return ret ? : -ENODEV;

	if (l2_pd_dev)
		data->cpuidle_driver.states[1].enter_s2idle = spm_enter_s2idle_state;

	return cpuidle_register(&data->cpuidle_driver, NULL);
}

static int spm_cpuidle_find_l2(struct device *dev)
{
	struct device_node *cpu_node, *l2_node, *saw_node;
	struct platform_device *pdev;
	struct of_phandle_args args;
	struct spm_driver_data *spm;
	int ret;

	cpu_node = of_cpu_device_node_get(0);
	if (!cpu_node)
		return 0;

	l2_node = of_parse_phandle(cpu_node, "next-level-cache", 0);
	of_node_put(cpu_node);
	if (!l2_node)
		return 0;

	saw_node = of_parse_phandle(l2_node, "qcom,saw", 0);
	if (!saw_node)
		goto put_l2;

	pdev = of_find_device_by_node(saw_node);
	of_node_put(saw_node);
	if (!pdev)
		goto put_l2;

	spm = dev_get_drvdata(&pdev->dev);
	put_device(&pdev->dev);
	if (!spm)
		goto put_l2;

	if (!of_parse_phandle_with_args(l2_node, "power-domains", "#power-domain-cells", 0, &args)) {
		ret = of_genpd_add_device(&args, dev);
		of_node_put(args.np);
		if (ret) {
			of_node_put(l2_node);
			return ret == -ENOENT ? -EPROBE_DEFER : ret;
		}

		pm_runtime_irq_safe(dev);

		ret = devm_pm_runtime_enable(dev);
		if (!ret)
			ret = pm_runtime_resume_and_get(dev);

		if (ret) {
			pm_genpd_remove_device(dev);
			of_node_put(l2_node);
			return ret;
		}

		dev_pm_syscore_device(dev, true);
		l2_pd_dev = dev;
	}

	l2_spm = spm;

	pr_info("qcom-spm-cpuidle: L2 GDHS enabled%s\n", l2_pd_dev ? ", L2 PC + RPM sleep in s2idle" : "");

put_l2:
	of_node_put(l2_node);

	return 0;
}

static int spm_cpuidle_drv_probe(struct platform_device *pdev)
{
	int cpu, ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	ret = qcom_scm_set_warm_boot_addr(cpu_resume_arm);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "set warm boot addr failed");

	ret = spm_cpuidle_find_l2(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "L2 power domain attach failed");

	for_each_present_cpu(cpu) {
		ret = spm_cpuidle_register(&pdev->dev, cpu);
		if (ret && ret != -ENODEV) {
			dev_err(&pdev->dev,
				"Cannot register for CPU%d: %d\n", cpu, ret);
		}
	}

	return 0;
}

static struct platform_driver spm_cpuidle_driver = {
	.probe = spm_cpuidle_drv_probe,
	.driver = {
		.name = "qcom-spm-cpuidle",
		.suppress_bind_attrs = true,
	},
};

static bool __init qcom_spm_find_any_cpu(void)
{
	struct device_node *cpu_node, *saw_node;

	for_each_of_cpu_node(cpu_node) {
		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		if (of_device_is_available(saw_node)) {
			of_node_put(saw_node);
			of_node_put(cpu_node);
			return true;
		}
		of_node_put(saw_node);
	}
	return false;
}

static int __init qcom_spm_cpuidle_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&spm_cpuidle_driver);
	if (ret)
		return ret;

	/* Make sure there is actually any CPU managed by the SPM */
	if (!qcom_spm_find_any_cpu())
		return 0;

	pdev = platform_device_register_simple("qcom-spm-cpuidle",
					       -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&spm_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(qcom_spm_cpuidle_init);
