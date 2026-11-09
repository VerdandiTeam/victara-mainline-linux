// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

static unsigned int input_boost_freq = 1497600;
module_param(input_boost_freq, uint, 0644);
MODULE_PARM_DESC(input_boost_freq, "Minimum CPU frequency in kHz while boosted, 0 disables boosting");

static unsigned int input_boost_ms = 100;
module_param(input_boost_ms, uint, 0644);
MODULE_PARM_DESC(input_boost_ms, "Boost duration in ms after the last input event");

static DEFINE_PER_CPU(struct freq_qos_request, boost_req);
static DEFINE_MUTEX(boost_lock);

static bool boosted;

static struct work_struct boost_work;
static struct delayed_work unboost_work;

static void boost_set_all(unsigned int freq)
{
	unsigned int cpu;

	mutex_lock(&boost_lock);

	for_each_possible_cpu(cpu) {
		struct freq_qos_request *req = &per_cpu(boost_req, cpu);

		if (freq_qos_request_active(req))
			freq_qos_update_request(req, freq);
	}

	WRITE_ONCE(boosted, freq != 0);

	mutex_unlock(&boost_lock);
}

static void do_boost(struct work_struct *work)
{
	unsigned int freq = READ_ONCE(input_boost_freq);

	if (!freq)
		return;

	boost_set_all(freq);
	mod_delayed_work(system_highpri_wq, &unboost_work, msecs_to_jiffies(READ_ONCE(input_boost_ms)));
}

static void do_unboost(struct work_struct *work)
{
	boost_set_all(0);
}

static void cpuboost_input_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if (!READ_ONCE(input_boost_freq))
		return;

	if (!READ_ONCE(boosted)) {
		queue_work(system_highpri_wq, &boost_work);
		return;
	}

	mod_delayed_work(system_highpri_wq, &unboost_work, msecs_to_jiffies(READ_ONCE(input_boost_ms)));
}

static int cpuboost_input_connect(struct input_handler *handler, struct input_dev *dev,
				  const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu-boost";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);

err_free:
	kfree(handle);

	return ret;
}

static void cpuboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuboost_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] = BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT | INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler cpuboost_input_handler = {
	.event = cpuboost_input_event,
	.connect = cpuboost_input_connect,
	.disconnect = cpuboost_input_disconnect,
	.passive_observer = true,
	.name = "cpu-boost",
	.id_table = cpuboost_ids,
};

static void boost_policy_add(struct cpufreq_policy *policy)
{
	struct freq_qos_request *req = &per_cpu(boost_req, cpumask_first(policy->related_cpus));
	int ret;

	mutex_lock(&boost_lock);

	if (!freq_qos_request_active(req)) {
		ret = freq_qos_add_request(&policy->constraints, req, FREQ_QOS_MIN, 0);
		if (ret < 0)
			pr_err("cpu-boost: CPU%u QoS request failed: %d\n", policy->cpu, ret);
	}

	mutex_unlock(&boost_lock);
}

static void boost_policy_remove(struct cpufreq_policy *policy)
{
	struct freq_qos_request *req = &per_cpu(boost_req, cpumask_first(policy->related_cpus));

	mutex_lock(&boost_lock);

	if (freq_qos_request_active(req))
		freq_qos_remove_request(req);

	mutex_unlock(&boost_lock);
}

static int boost_policy_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event == CPUFREQ_CREATE_POLICY)
		boost_policy_add(policy);
	else if (event == CPUFREQ_REMOVE_POLICY)
		boost_policy_remove(policy);

	return NOTIFY_OK;
}

static struct notifier_block boost_policy_nb = {
	.notifier_call = boost_policy_notify,
};

static int __init cpu_boost_init(void)
{
	unsigned int cpu;
	int ret;

	INIT_WORK(&boost_work, do_boost);
	INIT_DELAYED_WORK(&unboost_work, do_unboost);

	ret = input_register_handler(&cpuboost_input_handler);
	if (ret)
		return ret;

	ret = cpufreq_register_notifier(&boost_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		input_unregister_handler(&cpuboost_input_handler);
		cancel_work_sync(&boost_work);
		cancel_delayed_work_sync(&unboost_work);
		return ret;
	}

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;

		boost_policy_add(policy);
		cpufreq_cpu_put(policy);
	}

	return 0;
}
late_initcall(cpu_boost_init);

static void __exit cpu_boost_exit(void)
{
	unsigned int cpu;

	input_unregister_handler(&cpuboost_input_handler);
	cancel_work_sync(&boost_work);
	cancel_delayed_work_sync(&unboost_work);
	cpufreq_unregister_notifier(&boost_policy_nb, CPUFREQ_POLICY_NOTIFIER);

	mutex_lock(&boost_lock);

	for_each_possible_cpu(cpu) {
		struct freq_qos_request *req = &per_cpu(boost_req, cpu);

		if (freq_qos_request_active(req))
			freq_qos_remove_request(req);
	}

	mutex_unlock(&boost_lock);
}
module_exit(cpu_boost_exit);

MODULE_DESCRIPTION("Raise CPU minimum frequency briefly on input events");
MODULE_LICENSE("GPL");
