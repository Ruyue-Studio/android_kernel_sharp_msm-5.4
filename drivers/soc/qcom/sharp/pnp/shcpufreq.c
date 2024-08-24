/* drivers/soc/qcom/sharp/pnp/shcpufreq.c
 *
 * Copyright (C) 2010 Sharp Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_panel.h>
#include <linux/of.h>

#include <linux/pm_qos.h>
#include <linux/slab.h>

#include <linux/cpufreq.h>

#include <linux/input.h>
#include <linux/cpu.h>

/* --------------------------------------------------------------------------------------
 *  display status function
 * --------------------------------------------------------------------------------------
*/
static bool sh_disp_on_input = true;
static ssize_t show_sh_disp_on_input(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sh_disp_on_input);
}

struct kobject *shcpufreq_kobj;
static struct kobj_attribute sh_disp_on_input_attr = __ATTR(sh_disp_on_input, S_IRUGO, show_sh_disp_on_input, NULL);
static struct notifier_block drm_notif;
static int drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int blank;
	struct drm_panel_notifier *evdata = data;
	bool sh_disp_on_input_prev;

	if (evdata && evdata->data) {
		blank = *(int *)(evdata->data);
		sh_disp_on_input_prev = sh_disp_on_input;
		switch (event) {
		case DRM_PANEL_EARLY_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_POWERDOWN)
				sh_disp_on_input = false;
			break;
		case DRM_PANEL_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_UNBLANK)
				sh_disp_on_input = true;
			break;
		default:
			break;
		}
		if (sh_disp_on_input_prev != sh_disp_on_input)
			sysfs_notify(shcpufreq_kobj, NULL, "sh_disp_on_input");

		pr_debug("display status notify event:%lu blank:%d sh_disp_on_input:%d\n", event, blank, sh_disp_on_input);
	} else {
		pr_debug("display status notify event:%lu blank:- sh_disp_on_input:%d\n", event, sh_disp_on_input);
	}

	return 0;
}

static struct drm_panel *active_panel;
static int check_dt(struct device_node *np)
{
	int i;
	int count;
	int defer_check = 0;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			return 0;
		} else if (PTR_ERR(panel) == -EPROBE_DEFER) {
			defer_check = 1;
		}
	}
	if (defer_check==1)
		return -EPROBE_DEFER;

	return -ENODEV;
}

/* --------------------------------------------------------------------------------------
 *  input boost function
 * --------------------------------------------------------------------------------------
*/
static unsigned int input_boost_ms = 500;
module_param_named(
	input_boost_ms, input_boost_ms, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static unsigned int input_boost_freq_silver = 1305600;
module_param_named(
	input_boost_freq_silver, input_boost_freq_silver, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static unsigned int input_boost_freq_gold = 1324800;
module_param_named(
	input_boost_freq_gold, input_boost_freq_gold, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static unsigned int input_boost_freq_gold_plus = 1305600;
module_param_named(
	input_boost_freq_gold_plus, input_boost_freq_gold_plus, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static DEFINE_PER_CPU(unsigned int, qos_min_freq) = 0;

static struct workqueue_struct *cpu_boost_wq;
static struct work_struct input_boost_work;

static struct delayed_work input_boost_rem;
static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

static DEFINE_PER_CPU(struct freq_qos_request, qos_req_min);

static void boost_adjust_notify(struct cpufreq_policy *policy)
{
	unsigned int cpu = policy->cpu;
	struct freq_qos_request *req;
	int ret;

	pr_debug("CPU%u policy min before boost: %u kHz\n",
			 cpu, policy->min);
	pr_debug("CPU%u boost min: %u kHz\n", cpu, per_cpu(qos_min_freq, cpu));

	req = &per_cpu(qos_req_min, cpu);
	ret = freq_qos_update_request(req, per_cpu(qos_min_freq, cpu));
	if (ret < 0)
		pr_err("Failed to update min freq constraint in boost_adjust: %u\n",
								per_cpu(qos_min_freq, cpu));

	return;
}

static void update_policy_online(void)
{
	unsigned int i;
	struct cpufreq_policy *policy;
	struct cpumask online_cpus;
	/* Re-evaluate policy to trigger adjust notifier for online CPUs */
	get_online_cpus();
	online_cpus = *cpu_online_mask;
	for_each_cpu(i, &online_cpus) {
		policy = cpufreq_cpu_get(i);
		if (!policy) {
			pr_err("%s: cpufreq policy not found for cpu%d\n",
							__func__, i);
			return;
		}

		cpumask_andnot(&online_cpus, &online_cpus,
						policy->related_cpus);
		boost_adjust_notify(policy);
	}
	put_online_cpus();
}

static void do_input_boost_rem(struct work_struct *work)
{
	unsigned int i;

	/* Reset the input_boost_min for all CPUs in the system */
	pr_debug("Resetting input boost min for all CPUs\n");
	for_each_possible_cpu(i) {
		per_cpu(qos_min_freq, i) = 0;
	}

	/* Update policies for all online CPUs */
	update_policy_online();
}

static void do_input_boost(struct work_struct *work)
{
	unsigned int i;

	cancel_delayed_work_sync(&input_boost_rem);

	/* Set the input_boost_min for all CPUs in the system */
	pr_debug("Setting input boost min for all CPUs\n");
	for_each_possible_cpu(i) {
		if (i <= 3)
			per_cpu(qos_min_freq, i) = input_boost_freq_silver;
		if (i >= 4 && i <= 6)
			per_cpu(qos_min_freq, i) = input_boost_freq_gold;
		if (i == 7)
			per_cpu(qos_min_freq, i) = input_boost_freq_gold_plus;
	}

	/* Update policies for all online CPUs */
	update_policy_online();

	queue_delayed_work(cpu_boost_wq, &input_boost_rem,
					msecs_to_jiffies(input_boost_ms));
}

static int is_ignorable_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	struct input_dev *dev = handle->dev;

	if (sh_disp_on_input == false) {
		if (((type == EV_KEY) && (code == KEY_POWER) && (value == 1)) ||
			(!strcmp(dev->name, "shub_ex_notify") && (type == EV_ABS) && (code == ABS_Y) && (value & 0x100)) ||
			((type == EV_KEY) && (code == KEY_VOICESEARCH) && (value == 1))) {
			pr_debug("start clock boosted by input event dev_name = %s type = %d code = %d value = %d disp_status = %d\n", dev->name, type, code, value, sh_disp_on_input);
			return false;
		}
	}
	pr_debug("ignore input event dev_name = %s type = %d code = %d value = %d disp_status = %d\n", dev->name, type, code, value, sh_disp_on_input);
	return true;
}

static void cpuboost_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	u64 now;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	if (is_ignorable_event(handle, type, code, value))
		return;

	if (work_pending(&input_boost_work))
		return;

	queue_work(cpu_boost_wq, &input_boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpuboost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpuboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuboost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	/* pick up */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD( ABS_Y)] =
			BIT_MASK( ABS_Y) },
	},
	{ },
};

static struct input_handler cpuboost_input_handler = {
	.event          = cpuboost_input_event,
	.connect        = cpuboost_input_connect,
	.disconnect     = cpuboost_input_disconnect,
	.name           = "sh_input_boost",
	.id_table       = cpuboost_ids,
};

/* --------------------------------------------------------------------------------------
 *  driver Interface
 * --------------------------------------------------------------------------------------
*/
static int sh_cpufreq_probe(struct platform_device *pdev)
{
	int cpu, ret;
	struct device_node *dp;
	struct cpufreq_policy *policy;
	struct freq_qos_request *req;

	if (!pdev)
		return -EFAULT;

	if (!pdev->dev.of_node)
		return -ENODEV;

	dp = pdev->dev.of_node;
	if (!active_panel)
		check_dt(dp);

	if (active_panel) {
		drm_notif.notifier_call = drm_notifier_callback;
		drm_panel_notifier_register(active_panel, &drm_notif);
	} else {
		pr_err("%s: active_panel not found\n", __func__);
		return -ESRCH;
	}

	cpu_boost_wq = alloc_workqueue("cpuboost_wq", WQ_HIGHPRI, 0);
	if (!cpu_boost_wq) {
		pr_err("%s: Failed to allocate cpu_boost workqueue\n", __func__);
		return -EFAULT;
	}

	INIT_WORK(&input_boost_work, do_input_boost);
	INIT_DELAYED_WORK(&input_boost_rem, do_input_boost_rem);

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_err("%s: cpufreq policy not found for cpu%d\n",
							__func__, cpu);
			return -ESRCH;
		}

		req = &per_cpu(qos_req_min, cpu);
		ret = freq_qos_add_request(&policy->constraints, req,
						FREQ_QOS_MIN, policy->min);
		if (ret < 0) {
			pr_err("%s: Failed to add min freq constraint (%d)\n",
							__func__, ret);
			return ret;
		}
	}

	ret = input_register_handler(&cpuboost_input_handler);
	if (ret < 0)
		pr_err("%s: Failed to register input handler (%d)\n",
						__func__, ret);

	return ret;
}

static int sh_cpufreq_remove(struct platform_device *pdev)
{
	int cpu;
	struct freq_qos_request *req;

	if (active_panel)
		drm_panel_notifier_unregister(active_panel, &drm_notif);

	if (cpu_boost_wq)
		destroy_workqueue(cpu_boost_wq);

	for_each_possible_cpu(cpu) {
		req = &per_cpu(qos_req_min, cpu);
		if (req && freq_qos_request_active(req))
			freq_qos_remove_request(req);

		per_cpu(qos_min_freq, cpu) = 0;
	}

	input_unregister_handler(&cpuboost_input_handler);

	return 0;
}

static const struct of_device_id sh_cpufreq_of_match[] = {
	{ .compatible = "sharp,cpufreq", },
	{},
};

static struct platform_driver sh_cpufreq_driver = {
	.probe = sh_cpufreq_probe,
	.remove = sh_cpufreq_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sh_cpufreq",
		.of_match_table = sh_cpufreq_of_match,
	},
};

static int __init sh_cpufreq_init(void)
{
	int ret;

	shcpufreq_kobj = kobject_create_and_add("shcpufreq",
						&cpu_subsys.dev_root->kobj);
	if (!shcpufreq_kobj)
		pr_err("Failed to initialize sysfs node for shcpufreq.\n");

	ret = sysfs_create_file(shcpufreq_kobj, &sh_disp_on_input_attr.attr);
	if (ret)
		pr_err("Failed to create sh_disp_on_input node: %d\n", ret);

	return platform_driver_register(&sh_cpufreq_driver);
}

static void __exit sh_cpufreq_exit(void)
{
	platform_driver_unregister(&sh_cpufreq_driver);
}

module_init(sh_cpufreq_init);
module_exit(sh_cpufreq_exit);

MODULE_AUTHOR("SHARP CORPORATION");
MODULE_DESCRIPTION("sharp cpufreq module");
MODULE_LICENSE("GPL");
