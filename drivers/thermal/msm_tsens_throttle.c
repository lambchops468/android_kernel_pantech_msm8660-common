/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Author: Alexander Lam <lambchop468 (at) gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Implement a passive cooling device by throttling the CPU frequency.
 *
 * This driver should be used with msm_tsens.c .
 *
 * To set the throttling level manually:
 * > echo 5 > /sys/class/thermal/cooling_device0/cur_state
 * Higher numbers correspond to lower frequency limits.
 *
 * To set up automatic throttling with a target maximum CPU temperature of 48
 * degrees C:
 * > msm_tsens=`for tz in /sys/class/thermal/thermal_zone*; do
 *	[[ $(cat "$tz/type") == "tsens_tz_sensor0" ]] && echo "$tz";
 *	done`
 * > echo 48 > $msm_tsens/trip_point_4_temp
 * > echo -n enabled > $msm_tsens/trip_point_4_type
 * > echo -n enabled > $msm_tsens/mode
 *
 * The msm_tsens driver sets up the hardware sensor so that a new temperature
 * reading is generated once per second. If the temperature crosses above
 * the trip temp, a hardware interrupt is generated. The msm_tsens driver will
 * call thermal_zone_device_update(), which reads the current temperature from
 * the sensor's register. Since the temperature is above the trip temp,
 * thermal_zone_device_passive() is called, which enables passive cooling.
 *
 * As long as passive cooling is enabled, the kernel will poll the current
 * temperature once per second. No further hardware interrupts are generated.
 * After each temperature check, if the temperature is at or above the trip
 * point, and the temperature is increasing then CPU throttling will be
 * increased by one step.
 * If the temperature is below the trip point or if the temperature is
 * decreasing, CPU throttling is decreased by one step.
 *
 * Once CPU throttling is completely disabled, passive cooling is disabled and
 * temperature polling is stopped until the next msm_tsens hardware interrupt.
 */
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/thermal.h>
#include <mach/cpufreq.h>

// From arch/arm/mach-msm/acpuclock-presto.c
#define MAX_AXI 310500

// When throttle_level == 0 , no throttling is occuring.
// If throttle_level > 0, then the CPU's highest throttle_level frequencies are
// disabled. throttle_level indexes into avail_freqs.
static long throttle_level = 0;
static struct thermal_cooling_device* tc_dev;
// Used to hold a reference to the cpufreq driver so it can't be unloaded.
static struct cpufreq_policy *cpufreq_policy;
// Array of available CPU frequencies, sorted highest (fast) to lowest (slow).
static uint32_t *avail_freqs;
static size_t max_throttle_level;
static struct mutex tsens_throttle_lock;

// Sets the frequency limit at max_freq for cpu.
static void update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;

	ret = msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, max_freq);
	if (ret) {
		if (max_freq != MSM_CPUFREQ_NO_LIMIT)
			pr_warn("msm_tsens_throttle: Failed to set cpu%d max "
					"frequency to %u (%d)\n",
					cpu, max_freq, ret);
		else
			pr_warn("msm_tsens_throttle: Failed to reset cpu%d max "
					"frequency (%d)\n", cpu, ret);
		return;
	}

	if (max_freq != MSM_CPUFREQ_NO_LIMIT)
		pr_info("msm_tsens_throttle: Limiting cpu%d max frequency to "
				"%u\n",
				cpu, max_freq);
	else
		pr_info("msm_tsens_throttle: Max frequency reset for cpu%u\n",
				cpu);
}

// Applies the current frequency limit to cpu.
static void set_cpu_max_freq(int cpu) {
	int ret = 0;

	ret = cpufreq_update_policy(cpu);
	if (ret) {
		pr_warn("msm_tsens_throttle: Failed to set cpu%d policy (%d)\n",
			cpu, ret);
	}
}

static int tsens_throttle_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state) {
	if (!cdev || !state) {
		return -EINVAL;
	}
	mutex_lock(&tsens_throttle_lock);
	*state = max_throttle_level;
	mutex_unlock(&tsens_throttle_lock);
	return 0;
}

static int tsens_throttle_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state) {
	if (!cdev || !state) {
		return -EINVAL;
	}
	mutex_lock(&tsens_throttle_lock);
	*state = throttle_level;
	mutex_unlock(&tsens_throttle_lock);
	return 0;
}

static int tsens_throttle_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state) {
	int ret, cpu;
	uint32_t max_freq;

	if (!cdev || state > max_throttle_level) {
		return -EINVAL;
	}

	if (state == 0) {
		max_freq = MSM_CPUFREQ_NO_LIMIT;
	} else {
		max_freq = avail_freqs[state];
	}

	mutex_lock(&tsens_throttle_lock);

	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, max_freq);
	}

	for_each_online_cpu(cpu) {
		set_cpu_max_freq(cpu);
	}

	throttle_level = state;

	mutex_unlock(&tsens_throttle_lock);
	return 0;
}

static struct thermal_cooling_device_ops tsens_throttle_ops = {
	.get_max_state = tsens_throttle_get_max_state,
	.get_cur_state = tsens_throttle_get_cur_state,
	.set_cur_state = tsens_throttle_set_cur_state,
};

// Reversed comparison for reversed sort order.
static int long_rev_cmp(void *ap, void *bp) {
	uint32_t a = *((uint32_t *)ap);
	uint32_t b = *((uint32_t *)bp);
	return b - a;
}

static int __devinit tsens_throttle_probe(struct platform_device *pdev) {
	int i, j, ret = 0;
	size_t freq_table_size;
	uint32_t freq;
	struct cpufreq_frequency_table *freq_table;

	mutex_init(&tsens_throttle_lock);

	/* cpufreq_cpu_get() should only be done on CPU0 (the boot cpu). For
	 * other CPUs, the policy is destroyed/created on cpu hotplug (which
	 * happens during suspend). cpufreq_cpu_get() gets the msm cpufreq
	 * driver and prevents it from being unloaded.
	 */
	cpufreq_policy = cpufreq_cpu_get(0);
	if (!cpufreq_policy) {
		pr_err("%s: Could not get cpufreq driver\n", __func__);
		return -ENODEV;
	}

	freq_table = cpufreq_frequency_get_table(0);
	if (!freq_table) {
		pr_err("%s: Could not get cpufreq table\n", __func__);
		ret = -ENODEV;
		goto freq_table_err;
	}

	// TODO(AZL): Try skipping every other frequency (or even more skipping)
	freq_table_size = 0;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = freq_table[i].frequency;
		// frequencies < MAX_AXI cause lots of problems, like random
		// lock-ups and hotplug issues.
		if (freq >= MAX_AXI && freq != CPUFREQ_ENTRY_INVALID)
			freq_table_size++;
	}

	if (freq_table_size == 0) {
		pr_err("%s: Frequency table has no usable frequencies\n",
			__func__);
		ret = -ENODEV;
		goto alloc_freq_err;
	}

	avail_freqs = kcalloc(freq_table_size, sizeof(uint32_t), GFP_KERNEL);
	if (!avail_freqs) {
		pr_err("%s: Could not allocate frequency table\n", __func__);
		ret = -ENOMEM;
		goto alloc_freq_err;
	}

	j = 0;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = freq_table[i].frequency;
		if (freq >= MAX_AXI && freq != CPUFREQ_ENTRY_INVALID) {
			avail_freqs[j++] = freq_table[i].frequency;
			BUG_ON(j > freq_table_size);
		}
	}
	sort(avail_freqs, freq_table_size, sizeof(uint32_t), long_rev_cmp,
		NULL);
	max_throttle_level = freq_table_size - 1;

	tc_dev = thermal_cooling_device_register("Processor", NULL,
		&tsens_throttle_ops);
	if (IS_ERR(tc_dev)) {
		pr_err("%s: Could not register (%d)\n", __func__,
			PTR_ERR(tc_dev));
		ret = PTR_ERR(tc_dev);
		goto register_fail;
	}

	pr_notice("%s: OK\n", __func__);

	return 0;

register_fail:
	kfree(avail_freqs);
alloc_freq_err:
freq_table_err:
	if (cpufreq_policy)
		cpufreq_cpu_put(cpufreq_policy);

	return ret;
}

static int __devexit tsens_throttle_remove(struct platform_device *pdev) {
	thermal_cooling_device_unregister(tc_dev);

	if (avail_freqs)
		kfree(avail_freqs);

	if (cpufreq_policy)
		cpufreq_cpu_put(cpufreq_policy);

	mutex_destroy(&tsens_throttle_lock);

	return 0;
}

static struct platform_driver tsens_throttle_driver = {
	.probe	= tsens_throttle_probe,
	.remove	= __devexit_p(tsens_throttle_remove),
	.driver	= {
		.name = "tsens-throttle",
		.owner = THIS_MODULE,
	},
};

/* tsens_throttle_probe() calls cpufreq_cpu_get(), but it fails if the cpufreq
 * driver is not registered yet.  The cpufreq driver is registered by
 * msm_cpufreq_register(), which is  a late_initcall(). Thus it runs after
 * module_init(), which means this driver cannot be a module. It also cannot be
 * a late_initcall()  driver because linker order determines which
 * late_initcall() will run first. So, we have to call this init function
 * directly from msm_cpufreq_register().
 */
int __init msm_tsens_throttle_init(void)
{
	return platform_driver_register(&tsens_throttle_driver);
}
