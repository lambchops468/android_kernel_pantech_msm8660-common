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
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/thermal.h>
#include <mach/cpufreq.h>

// From arch/arm/mach-msm/acpuclock-presto.c
#define MAX_AXI 310500
#define MAX_FREQUENCY_TABLE_SIZE 15

// The current throttle level applied to hardware.
// When throttle_level == 0 , no throttling is occuring.
// If throttle_level > 0, then the CPU's highest throttle_level frequencies are
// disabled. throttle_level indexes into avail_freqs.
static unsigned long throttle_level = 0;
// Throttle level requested by thermal kernel subsystem. This includes the sysfs
// interface and any thermal zones that need to throttle the CPU, like the the
// one implemented by the msm_tsens driver.
static unsigned long requested_throttle_level = 0;
// Throttle level used when the screen is off. Can be set by kernel parameter,
// but defaults 1/3 of the available throttling range.
static unsigned long screen_off_throttle_level = 0;
// Whether the screen is currently off.
static bool screen_is_off = false;

static struct thermal_cooling_device* tc_dev;
// Used to hold a reference to the cpufreq driver so it can't be unloaded.
static struct cpufreq_policy *cpufreq_policy;
// Array of available CPU frequencies, sorted highest (fast) to lowest (slow).
static uint32_t *avail_freqs;
static size_t max_throttle_level = 0;
static DEFINE_MUTEX(tsens_throttle_lock);

static int screen_off_throttle_level_set(const char *val,
						struct kernel_param *kp);
static struct kernel_param_ops screen_off_param_ops = {
	.set = screen_off_throttle_level_set,
	.get = param_get_long,
};
module_param_cb(screen_off_throttle_level, &screen_off_param_ops,
		&screen_off_throttle_level, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(screen_off_throttle_level, "The throttle level to set when "
						"the screen turns off.");

static void msm_throttle_early_suspend(struct early_suspend *h);
static void msm_throttle_late_resume(struct early_suspend *h);
struct early_suspend msm_throttle_early_suspend_ops = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = msm_throttle_early_suspend,
	.resume = msm_throttle_late_resume,
};


// Sets the frequency limit at max_freq for cpu.
static void update_cpu_max_freq(int cpu, uint32_t max_freq) {
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
	}
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
	*state = requested_throttle_level;
	mutex_unlock(&tsens_throttle_lock);
	return 0;
}

static void __tsens_throttle_set_cur_state() {
	int cpu;
	uint32_t max_freq;
	unsigned long state;

	if (!screen_is_off) {
		state = requested_throttle_level;
	} else if (requested_throttle_level > screen_off_throttle_level) {
		state = requested_throttle_level;
	} else {
		state = screen_off_throttle_level;
	}

	if (max_throttle_level == 0) {
		return 0;
	}

	if (state > max_throttle_level) {
		pr_err("msm_tsens_throttle: Clamped internal throttle level to "
			"maximum supported level.\n");
		state = max_throttle_level;
	}

	if (throttle_level == state) {
		return 0;
	}

	if (state == 0) {
		max_freq = MSM_CPUFREQ_NO_LIMIT;
	} else {
		max_freq = avail_freqs[state];
	}

	if (max_freq != MSM_CPUFREQ_NO_LIMIT) {
		pr_info("msm_tsens_throttle: %s CPU max freq to "
				"%u\n", throttle_level < state
					? "Throttling"
					: "Unthrottling",
				max_freq/1000);
	} else {
		pr_info("msm_tsens_throttle: CPU max frequency reset\n");
	}


	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, max_freq);
	}

	for_each_online_cpu(cpu) {
		set_cpu_max_freq(cpu);
	}

	throttle_level = state;
}

static int tsens_throttle_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state) {
	if (!cdev || state > max_throttle_level) {
		return -EINVAL;
	}

	mutex_lock(&tsens_throttle_lock);
	requested_throttle_level = state;
	__tsens_throttle_set_cur_state();
	mutex_unlock(&tsens_throttle_lock);

	return 0;
}

static void msm_throttle_early_suspend(struct early_suspend *h) {
	mutex_lock(&tsens_throttle_lock);

	screen_is_off = true;
	__tsens_throttle_set_cur_state();

	mutex_unlock(&tsens_throttle_lock);
}

static void msm_throttle_late_resume(struct early_suspend *h) {
	mutex_lock(&tsens_throttle_lock);

	screen_is_off = false;
	__tsens_throttle_set_cur_state();

	mutex_unlock(&tsens_throttle_lock);
}

static void print_screen_off_throttle_speed() {
	pr_info("msm_tsens_throttle: CPU Screen-Off Speed: %u MHz\n",
		avail_freqs[screen_off_throttle_level]/1000);
}

static int screen_off_throttle_level_set(const char *val,
						struct kernel_param *kp)
{
	int rc = 0;

	mutex_lock(&tsens_throttle_lock);

	rc = param_set_long(val, kp);
	if (rc) {
		goto out;
	}

	// If kernel param is set on boot, max_throttle_level might not be
	// initialized yet, so finish here.
	if (max_throttle_level == 0) {
		goto out;
	}

	if (screen_off_throttle_level > max_throttle_level) {
		rc = -EINVAL;
		goto out;
	}

	print_screen_off_throttle_speed();

	__tsens_throttle_set_cur_state();

out:
	mutex_unlock(&tsens_throttle_lock);
	return rc;
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

// If freq_table is too large, then make a new smaller table and point
// freq_table at it. freq_table_size is updated with the size of the smaller
// table. The old freq_table is freed.
// If there is an error, nothing is freed.
static int shrink_freq_table(uint32_t **freq_table, size_t *freq_table_size) {
	int i, j, skip, skip_size, ret = 0;
	size_t shrunk_freq_table_size;
	uint32_t* shrunk_freq_table;

	if (*freq_table_size <= MAX_FREQUENCY_TABLE_SIZE) {
		return 0;
	}

	// Round up to ensure new frequency table is smaller than
	// MAX_FREQUENCY_TABLE_SIZE.
	skip_size = (*freq_table_size + MAX_FREQUENCY_TABLE_SIZE - 1)
			/ MAX_FREQUENCY_TABLE_SIZE;
	BUG_ON(skip_size < 1);

	// Save room for the maximum frequency.
	shrunk_freq_table_size++;

	// Round up because we need to make room for the lowest frequency.
	// (freq_table_size - 1) is the size of the freq table except for the
	// maximum frequency, which was already accounted for.
	shrunk_freq_table_size += ((*freq_table_size - 1) + skip_size - 1)
		/ skip_size;

	// Fill freqs
	shrunk_freq_table = kcalloc(shrunk_freq_table_size, sizeof(uint32_t),
								GFP_KERNEL);
	if (!shrunk_freq_table) {
		pr_err("%s: Could not allocate frequency table\n", __func__);
		ret = -ENOMEM;
		goto alloc_err;
	}

	j = 0;
	skip = 0;
	// Save the maximum frequency.
	shrunk_freq_table[j++] = (*freq_table)[0];
	// Save every skip-nd (i.e., 2nd) frequency.
	for (i = 1; i < *freq_table_size; i++) {
		if (++skip < skip_size) {
			continue;
		}
		shrunk_freq_table[j++] = (*freq_table)[i];
		BUG_ON(j > shrunk_freq_table_size);
		skip = 0;
	}

	if (skip > 0) {
		// lowest frequency was not copied in the loop above, so do it
		// now.
		shrunk_freq_table[j++] = (*freq_table)[*freq_table_size-1];
		BUG_ON(j > shrunk_freq_table_size);
	}

	kfree(*freq_table);
	*freq_table = shrunk_freq_table;
	*freq_table_size = shrunk_freq_table_size;
	return 0;

alloc_err:
	return ret;
}

static void print_freq_table(uint32_t* freqs, size_t freqs_size) {
	// Room for a space, 5 digits of the frequency, and a '\0'
	size_t freq_str_space = 7;
	size_t buf_size = freqs_size*freq_str_space;
	size_t step = 0;
	char freqs_str[buf_size];
	char* freqs_str_p = freqs_str;
	int i;

	for (i = 0; i < freqs_size; i++) {
		step = snprintf(freqs_str_p, freq_str_space, " %d",
								freqs[i]/1000);
		freqs_str_p += step;
	}

	pr_info("msm_tsens_throttle: CPU Throttling Frequencies Mhz [%u]:%s\n",
								freqs_size,
								freqs_str);
}

static int setup_freq_table(struct cpufreq_frequency_table* freq_table) {
	int i, j, ret = 0;
	uint32_t freq;
	uint32_t *freqs;
	size_t freqs_size;

	// Count number of usable frequencies.
	freqs_size = 0;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = freq_table[i].frequency;
		// frequencies < MAX_AXI cause lots of problems, like random
		// lock-ups and hotplug issues.
		if (freq < MAX_AXI || freq == CPUFREQ_ENTRY_INVALID)
			continue;
		freqs_size++;
	}

	if (freqs_size == 0) {
		pr_err("%s: Frequency table has no usable frequencies\n",
			__func__);
		ret = -ENODEV;
		goto freq_table_err;
	}

	// Fill freqs
	freqs = kcalloc(freqs_size, sizeof(uint32_t), GFP_KERNEL);
	if (!freqs) {
		pr_err("%s: Could not allocate frequency table\n", __func__);
		ret = -ENOMEM;
		goto alloc_err;
	}

	j = 0;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = freq_table[i].frequency;
		if (freq >= MAX_AXI && freq != CPUFREQ_ENTRY_INVALID) {
			freqs[j++] = freq_table[i].frequency;
			BUG_ON(j > freqs_size);
		}
	}

	// Sort freqs from highest to lowest.
	sort(freqs, freqs_size, sizeof(uint32_t), long_rev_cmp,
		NULL);

	ret = shrink_freq_table(&freqs, &freqs_size);
	if (ret) {
		goto shrink_err;
	}

	print_freq_table(freqs, freqs_size);

	avail_freqs = freqs;
	max_throttle_level = freqs_size - 1;

	if (screen_off_throttle_level == 0) {
		screen_off_throttle_level = freqs_size < 2 ? 0 :
                                            freqs_size/2;
	} else if (screen_off_throttle_level > max_throttle_level) {
		pr_info("msm_tsens_throttle: Clamped screen_off_throttle_level "
			"to maximum supported level.\n");
		screen_off_throttle_level = max_throttle_level;
	}
	print_screen_off_throttle_speed();

	return 0;

shrink_err:
	kfree(freqs);
alloc_err:
freq_table_err:
	return ret;
}

static int __devinit tsens_throttle_probe(struct platform_device *pdev) {
	int ret = 0;
	struct cpufreq_frequency_table *freq_table;

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

	ret = setup_freq_table(freq_table);
	if (ret) {
		goto freq_table_err;
	}

	tc_dev = thermal_cooling_device_register("Processor", NULL,
		&tsens_throttle_ops);
	if (IS_ERR(tc_dev)) {
		pr_err("%s: Could not register (%d)\n", __func__,
			PTR_ERR(tc_dev));
		ret = PTR_ERR(tc_dev);
		goto register_fail;
	}

	register_early_suspend(&msm_throttle_early_suspend_ops);

	pr_notice("%s: OK\n", __func__);

	return 0;

register_fail:
	if (avail_freqs)
		kfree(avail_freqs);
freq_table_err:
	if (cpufreq_policy)
		cpufreq_cpu_put(cpufreq_policy);

	return ret;
}

static int __devexit tsens_throttle_remove(struct platform_device *pdev) {
	unregister_early_suspend(&msm_throttle_early_suspend_ops);

	thermal_cooling_device_unregister(tc_dev);

	if (avail_freqs)
		kfree(avail_freqs);

	if (cpufreq_policy)
		cpufreq_cpu_put(cpufreq_policy);

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
