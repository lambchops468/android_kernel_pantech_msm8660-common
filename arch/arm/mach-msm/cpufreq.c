/* arch/arm/mach-msm/cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, The Linux Foundation. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/msm_tsens_throttle.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <mach/socinfo.h>
#include <mach/cpufreq.h>

#include "acpuclock.h"

#ifdef CONFIG_SMP
struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	int frequency;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *msm_cpufreq_wq;
#endif

struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);

static int override_cpu;

struct cpu_freq {
	uint32_t max;
	uint32_t min;
	uint32_t allowed_max;
	uint32_t allowed_min;
};

static DEFINE_PER_CPU(struct cpu_freq, cpu_freq_info);

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int new_freq)
{
	int ret = 0;
	struct cpufreq_freqs freqs;
	struct cpu_freq *limit = &per_cpu(cpu_freq_info, policy->cpu);

	// If set_cpu_freq is called under normal conditions, these limits
	// should already be applied by msm_cpufreq_verify().
	if (new_freq > limit->allowed_max) {
		pr_warn("%s: Target frequency %u max-limited to %u.\n",
			__func__, new_freq,
			limit->allowed_max);
		new_freq = limit->allowed_max;
	}

	if (new_freq < limit->allowed_min) {
		pr_warn("%s: Target frequency %u min-limited to %u.\n",
			__func__, new_freq,
			limit->allowed_min);
		new_freq = limit->allowed_min;
	}

	freqs.old = policy->cur;
	if (override_cpu) {
		if (policy->cur == policy->max)
			return 0;
		else
			freqs.new = policy->max;
	} else
		freqs.new = new_freq;
	freqs.cpu = policy->cpu;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = acpuclk_set_rate(policy->cpu, new_freq, SETRATE_CPUFREQ);
	if (!ret)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

#ifdef CONFIG_SMP
static int __cpuinit msm_cpufreq_cpu_callback(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
		break;
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		break;
	case CPU_DOWN_FAILED:
	case CPU_DOWN_FAILED_FROZEN:
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_cpufreq_cpu_notifier = {
	.notifier_call = msm_cpufreq_cpu_callback,
};

static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status = set_cpu_freq(cpu_work->policy, cpu_work->frequency);
	complete(&cpu_work->complete);
}
#endif

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	struct cpufreq_frequency_table *table;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
	cpumask_var_t mask;

	if (!cpu_active(policy->cpu)) {
		pr_info("cpufreq: cpu %d is not active.\n", policy->cpu);
		return -ENODEV;
	}

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;
#endif

	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		pr_debug("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);
#endif

#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->policy = policy;
	cpu_work->frequency = table[index].frequency;
	cpu_work->status = -ENODEV;

	cpumask_clear(mask);
	cpumask_set_cpu(policy->cpu, mask);
	if (cpumask_equal(mask, &current->cpus_allowed)) {
		ret = set_cpu_freq(cpu_work->policy, cpu_work->frequency);
		goto done;
	} else {
		cancel_work_sync(&cpu_work->work);
		INIT_COMPLETION(cpu_work->complete);
		queue_work_on(policy->cpu, msm_cpufreq_wq, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
	}

	ret = cpu_work->status;
#else
	ret = set_cpu_freq(policy, table[index].frequency);
#endif

done:
#ifdef CONFIG_SMP
	free_cpumask_var(mask);
#endif
	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table =
		cpufreq_frequency_get_table(policy->cpu);
	struct cpu_freq *limit = &per_cpu(cpu_freq_info, policy->cpu);
	int ret;

	if (freq_table == NULL) {
		pr_warn("%s: cpufreq driver not initialized yet.\n", __func__);
		return -ENODEV;
	}

	if (limit->min == 0) {
		// limit table is not initialized yet
		cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
						policy->cpuinfo.max_freq);
	} else {
		// Clamp policy->{min,max} to thermal throttle limits
		// We apply the thermal throttle limits here before the cpufreq
		// governor runs instead of after the cpufreq governor because
		// we want the governor to have accurate state.
		cpufreq_verify_within_limits(policy, limit->allowed_min,
				limit->allowed_max);
	}

	// Ensure there is at least one valid frequency that the hardware
	// supports. policy->max may be bumped up.
	ret = cpufreq_frequency_table_verify(policy, freq_table);
	if (ret) {
		return ret;
	}
	return 0;
}

static unsigned int msm_cpufreq_get_freq(unsigned int cpu)
{
	return acpuclk_get_rate(cpu);
}

static inline int msm_cpufreq_limits_init(void)
{
	int cpu = 0;
	int i = 0;
	struct cpufreq_frequency_table *table = NULL;
	uint32_t min = (uint32_t) -1;
	uint32_t max = 0;
	uint32_t freq;
	struct cpu_freq *limit = NULL;

	for_each_possible_cpu(cpu) {
		limit = &per_cpu(cpu_freq_info, cpu);
		table = cpufreq_frequency_get_table(cpu);
		if (table == NULL) {
			pr_err("%s: error reading cpufreq table for cpu %d\n",
					__func__, cpu);
			min = 0;
			max = ~0;
		} else {
			for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END);
									i++) {
				freq = table[i].frequency;
				if (freq == CPUFREQ_ENTRY_INVALID)
					continue;
				if (freq > max)
					max = freq;
				if (freq < min)
					min = freq;
			}
		}
		limit->allowed_min = min;
		limit->allowed_max = max;
		limit->min = min;
		limit->max = max;
	}

	return 0;
}

int msm_cpufreq_set_freq_limits(uint32_t cpu, uint32_t min, uint32_t max)
{
	struct cpu_freq *limit = &per_cpu(cpu_freq_info, cpu);
        struct cpufreq_frequency_table *freq_table =
		cpufreq_frequency_get_table(0);
	int i;
	uint32_t new_max, new_min;
	bool exists_valid_frequency_in_range;

	if (limit->min == 0 || freq_table == NULL) {
		pr_warn("%s: Attempted call before cpufreq driver is "
			"initialized.\n", __func__);
		return -ENODEV;
	}

	if (min == MSM_CPUFREQ_NO_LIMIT) {
		new_min = limit->min;
	} else {
		new_min = min;
	}

	if (max == MSM_CPUFREQ_NO_LIMIT) {
		new_max = limit->max;
	} else {
		new_max = max;
	}

	exists_valid_frequency_in_range = false;
	for (i = 0; (freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = freq_table[i].frequency;
		if (freq == CPUFREQ_ENTRY_INVALID)
			continue;
		if ((freq >= new_min) && (freq <= new_max)) {
			exists_valid_frequency_in_range = true;
			break;
		}
	}
	if (!exists_valid_frequency_in_range) {
		pr_warn("%s: Attempted to set invalid limits: New min: "
			"%u, max: %u, but there are no valid cpu scaling "
			"frequencies in the new limits.\n",
			__func__, new_min, new_max);
		return -EINVAL;
	}

	if (new_min >= limit->min && new_min <= limit->max &&
			new_min <= new_max && new_max >= limit->min &&
			new_max <= limit->max && new_max >= new_min) {
		limit->allowed_min = new_min;
		limit->allowed_max = new_max;
	} else {
		pr_warn("%s: Attempted to set invalid limits: New min: "
			"%u, max: %u. Allowed min: %u, max: %u\n", __func__,
			new_min, new_max, limit->min, limit->max);
		return -EINVAL;
	}

	pr_debug("%s: Limiting cpu %d min = %d, max = %d\n", __func__, cpu,
			limit->allowed_min, limit->allowed_max);

	return 0;
}
EXPORT_SYMBOL(msm_cpufreq_set_freq_limits);

static int __cpuinit msm_cpufreq_init(struct cpufreq_policy *policy)
{
	int cur_freq;
	int index;
	struct cpufreq_frequency_table *table;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
#endif

	if (cpu_is_apq8064())
		return -ENODEV;

	table = cpufreq_frequency_get_table(policy->cpu);
	if (cpufreq_frequency_table_cpuinfo(policy, table)) {
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
		policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_MIN;
		policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	}
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#endif

	cur_freq = acpuclk_get_rate(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_H, &index) &&
	    cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_L, &index)) {
		pr_info("cpufreq: cpu%d at invalid freq: %d\n",
				policy->cpu, cur_freq);
		return -EINVAL;
	}

	if (cur_freq != table[index].frequency) {
		int ret = 0;
		ret = acpuclk_set_rate(policy->cpu, table[index].frequency,
				SETRATE_CPUFREQ);
		if (ret)
			return ret;
		pr_info("cpufreq: cpu%d init at %d switching to %d\n",
				policy->cpu, cur_freq, table[index].frequency);
		cur_freq = table[index].frequency;
	}

	policy->cur = cur_freq;

	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;
#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);
#endif

	return 0;
}

static int msm_cpufreq_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
	}

	return NOTIFY_DONE;
}

static int msm_cpufreq_resume(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return NOTIFY_DONE;
}

static int msm_cpufreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return msm_cpufreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return msm_cpufreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static ssize_t store_mfreq(struct sysdev_class *class,
			struct sysdev_class_attribute *attr,
			const char *buf, size_t count)
{
	u64 val;

	if (strict_strtoull(buf, 0, &val) < 0) {
		pr_err("Invalid parameter to mfreq\n");
		return 0;
	}
	if (val)
		override_cpu = 1;
	else
		override_cpu = 0;
	return count;
}

static SYSDEV_CLASS_ATTR(mfreq, 0200, NULL, store_mfreq);

static struct freq_attr *msm_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.get		= msm_cpufreq_get_freq,
	.name		= "msm",
	.attr		= msm_freq_attr,
};

static struct notifier_block msm_cpufreq_pm_notifier = {
	.notifier_call = msm_cpufreq_pm_event,
};

static int __init msm_cpufreq_register(void)
{
	int cpu;

	int err = sysfs_create_file(&cpu_sysdev_class.kset.kobj,
			&attr_mfreq.attr);
	if (err)
		pr_err("Failed to create sysfs mfreq\n");

	// Frequency table should already be setup by cpufreq_table_init()
	// in acpuclock-presto.c.
	msm_cpufreq_limits_init();

	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

#ifdef CONFIG_SMP
	msm_cpufreq_wq = create_workqueue("msm-cpufreq");

	register_hotcpu_notifier(&msm_cpufreq_cpu_notifier);
#endif

	register_pm_notifier(&msm_cpufreq_pm_notifier);
	err = cpufreq_register_driver(&msm_cpufreq_driver);
	if (err)
		return err;

	err = msm_tsens_throttle_init();
	if (err)
		pr_err("Failed to register msm_tsens_throttle driver\n");

	return 0;
}

late_initcall(msm_cpufreq_register);
