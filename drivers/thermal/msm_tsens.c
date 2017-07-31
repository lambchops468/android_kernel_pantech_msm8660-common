/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
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
 * Qualcomm TSENS Thermal Manager driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <linux/pm.h>

// p15060, MDM boot up fail fix
#include <linux/gpio.h>

/* Trips: from very hot to very cold */
/* TODO(AZL): STAGE3 and STAGE0 don't work. No interrupt is generated, so
 * no event processing occurs for those trip points */
enum tsens_trip_type {
	TSENS_TRIP_STAGE3 = 0,
	TSENS_TRIP_STAGE2,
	TSENS_TRIP_STAGE1,
	TSENS_TRIP_STAGE0,
	TSENS_TRIP_NUM,
};

#define TSENS_NUM_SENSORS	1 /* There are 5 but only 1 is useful now */
#define TSENS_CAL_DEGC		30 /* degree C used for calibration */
#define TSENS_QFPROM_ADDR (MSM_QFPROM_BASE + 0x000000bc)
#define TSENS_QFPROM_RED_TEMP_SENSOR0_SHIFT 24
#define TSENS_QFPROM_TEMP_SENSOR0_SHIFT 16
#define TSENS_QFPROM_TEMP_SENSOR0_MASK (255 << TSENS_QFPROM_TEMP_SENSOR0_SHIFT)
#define TSENS_SLOPE (0.702)  /* slope in (degrees_C / ADC_code) */
#define TSENS_FACTOR (1000)  /* convert floating-point into integer */
#define TSENS_CONFIG 01      /* this setting found to be optimal */
#define TSENS_CONFIG_SHIFT 28
#define TSENS_CONFIG_MASK (3 << TSENS_CONFIG_SHIFT)
#define TSENS_CNTL_ADDR (MSM_CLK_CTL_BASE + 0x00003620)
#define TSENS_EN (1 << 0)
#define TSENS_SW_RST (1 << 1)
#define SENSOR0_EN (1 << 3)
#define SENSOR1_EN (1 << 4)
#define SENSOR2_EN (1 << 5)
#define SENSOR3_EN (1 << 6)
#define SENSOR4_EN (1 << 7)
#define TSENS_MIN_STATUS_MASK (1 << 8)
#define TSENS_LOWER_STATUS_CLR (1 << 9)
#define TSENS_UPPER_STATUS_CLR (1 << 10)
#define TSENS_MAX_STATUS_MASK (1 << 11)
#define TSENS_MEASURE_PERIOD 4 /* 1 sec. default as required by Willie */
#define TSENS_SLP_CLK_ENA (1 << 24)
#define TSENS_THRESHOLD_ADDR (MSM_CLK_CTL_BASE + 0x00003624)
#define TSENS_THRESHOLD_MAX_CODE (0xff)
#define TSENS_THRESHOLD_MAX_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 24)
#define TSENS_THRESHOLD_MIN_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 16)
#define TSENS_THRESHOLD_UPPER_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 8)
#define TSENS_THRESHOLD_LOWER_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 0)
/* Initial temperature threshold values */
#define TSENS_LOWER_LIMIT_TH   0x50
#define TSENS_UPPER_LIMIT_TH   0xdf
#define TSENS_MIN_LIMIT_TH     0x38
#define TSENS_MAX_LIMIT_TH     0xff

#define TSENS_S0_STATUS_ADDR (MSM_CLK_CTL_BASE + 0x00003628)
#define TSENS_INT_STATUS_ADDR (MSM_CLK_CTL_BASE + 0x0000363c)
#define TSENS_LOWER_INT_MASK (1 << 1)
#define TSENS_UPPER_INT_MASK (1 << 2)
#define TSENS_TRDY_MASK (1 << 7)

struct tsens_tm_device_sensor {
	struct thermal_zone_device	*tz_dev;
	enum thermal_device_mode	mode;
	unsigned int			sensor_num;
};

struct tsens_tm_device {
	struct tsens_tm_device_sensor sensor[TSENS_NUM_SENSORS];
	struct work_struct work;
	spinlock_t lock;
	bool prev_reading_avail;
	bool suspended;
	int offset;
	uint32_t pm_tsens_thr_data;
};

struct tsens_tm_device *tmdev;

/*******
 * TODO(AZL): fix upper/lower limit clr.
 * Fix interrupt locking/race
 * Use thermal sys ->notify
 * Cleanup printk
 * enable sensors by default
 */

// p15060, MDM boot up fail fix
bool mdm_reset = false;

/* Temperature on y axis and ADC-code on x-axis */
static int tsens_tz_code_to_degC(int adc_code)
{
	int degC, degcbeforefactor;
	degcbeforefactor = adc_code * (int)(TSENS_SLOPE * TSENS_FACTOR)
				+ tmdev->offset;
	if (degcbeforefactor == 0)
		degC = degcbeforefactor;
	else if (degcbeforefactor > 0)
		degC = (degcbeforefactor + TSENS_FACTOR/2) / TSENS_FACTOR;
	else  /* rounding for negative degrees */
		degC = (degcbeforefactor - TSENS_FACTOR/2) / TSENS_FACTOR;
	return degC;
}

static int tsens_tz_degC_to_code(int degC)
{
	int code = (degC * TSENS_FACTOR - tmdev->offset
			+ (int)(TSENS_FACTOR * TSENS_SLOPE)/2)
			/ (int)(TSENS_FACTOR * TSENS_SLOPE);
	if (code > 255) /* upper bound */
		code = 255;
	else if (code < 0) /* lower bound */
		code = 0;
	return code;
}

// Call this when reconfiguring trip points. Trip points may not cause an
// interrupt if the temperature was already exceeding the trip point
// temperatures, so manually kick-off any actions.
static void tsens_tz_force_update(struct thermal_zone_device *tz) {
	thermal_zone_device_update(tz);
}

static int tsens_tz_get_temp(struct thermal_zone_device *thermal,
			     unsigned long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;
	unsigned int code;

	if (!tm_sensor || !temp)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);

	if (tm_sensor->mode != THERMAL_DEVICE_ENABLED) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -EINVAL;
	}
	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}

	if (!tmdev->prev_reading_avail) {
		while (!(readl(TSENS_INT_STATUS_ADDR) & TSENS_TRDY_MASK))
			msleep(1);
		tmdev->prev_reading_avail = 1;
	}

	code = readl(TSENS_S0_STATUS_ADDR + (tm_sensor->sensor_num << 2));
	spin_unlock_irqrestore(&tmdev->lock, flags);

	*temp = tsens_tz_code_to_degC(code);

	return 0;
}

static int tsens_tz_get_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode *mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;

	if (!tm_sensor || !mode)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);
	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}

	*mode = tm_sensor->mode;
	spin_unlock_irqrestore(&tmdev->lock, flags);

	return 0;
}

static int tsens_tz_set_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;
	unsigned int reg, mask;

	if (!tm_sensor)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);
	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}

	if (mode != tm_sensor->mode) {
		pr_info("%s: mode: %d --> %d\n", __func__, tm_sensor->mode,
									 mode);

		reg = readl(TSENS_CNTL_ADDR);
		mask = 1 << (tm_sensor->sensor_num + 3);
		if (mode == THERMAL_DEVICE_ENABLED) {
			writel(reg | TSENS_SW_RST, TSENS_CNTL_ADDR);
			reg |= mask | TSENS_SLP_CLK_ENA | TSENS_EN;
			tmdev->prev_reading_avail = 0;
		} else {
			reg &= ~mask;
			if (!(reg & (((1 << TSENS_NUM_SENSORS) - 1) << 3)))
				reg &= ~(TSENS_SLP_CLK_ENA | TSENS_EN);
		}

		writel(reg, TSENS_CNTL_ADDR);
	}
	tm_sensor->mode = mode;
	spin_unlock_irqrestore(&tmdev->lock, flags);

// p15060, MDM boot up fail fix
// MDM2AP_STATUS       -> 134
// AP2MDM_PMIC_RESET_N -> 131
#if (1)
    if( !mdm_reset )
    {
        if (gpio_get_value(134) == 0)
        {
            pr_info("MDM Status PIN is 0 , MDM reset now\n");
            gpio_direction_output(131, 1);
            msleep(4000);
            gpio_direction_output(131, 0);
        }
        mdm_reset = true;
    }

#endif
	tsens_tz_force_update(thermal);
	return 0;
}

static int tsens_tz_get_trip_type(struct thermal_zone_device *thermal,
				   int trip, enum thermal_trip_type *type)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;

	if (!tm_sensor || trip < 0 || !type)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);
	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}
	spin_unlock_irqrestore(&tmdev->lock, flags);

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	case TSENS_TRIP_STAGE2:
		*type = THERMAL_TRIP_CONFIGURABLE_HI;
		break;
	case TSENS_TRIP_STAGE1:
		*type = THERMAL_TRIP_CONFIGURABLE_LOW;
		break;
	case TSENS_TRIP_STAGE0:
		*type = THERMAL_TRIP_CRITICAL_LOW;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tsens_tz_activate_trip_type(struct thermal_zone_device *thermal,
			int trip, enum thermal_trip_activation_mode mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;
	unsigned int reg_cntl, reg_th, code, hi_code, lo_code, mask;

	if (!tm_sensor || trip < 0)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);

	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}

	lo_code = 0;
	hi_code = TSENS_THRESHOLD_MAX_CODE;

	reg_cntl = readl(TSENS_CNTL_ADDR);
	reg_th = readl(TSENS_THRESHOLD_ADDR);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK) >> 24;
		mask = TSENS_MAX_STATUS_MASK;

		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE2:
		code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK) >> 8;
		mask = TSENS_UPPER_STATUS_CLR;

		if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE1:
		code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK) >> 0;
		mask = TSENS_LOWER_STATUS_CLR;

		if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	case TSENS_TRIP_STAGE0:
		code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK) >> 16;
		mask = TSENS_MIN_STATUS_MASK;

		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	default:
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -EINVAL;
	}

	if (mode == THERMAL_TRIP_ACTIVATION_DISABLED) {
		writel(reg_cntl | mask, TSENS_CNTL_ADDR);
		spin_unlock_irqrestore(&tmdev->lock, flags);
	} else {
		if (code < lo_code || code > hi_code) {
			spin_unlock_irqrestore(&tmdev->lock, flags);
			return -EINVAL;
		}
		writel(reg_cntl & ~mask, TSENS_CNTL_ADDR);
		spin_unlock_irqrestore(&tmdev->lock, flags);

		tsens_tz_force_update(thermal);
	}

	return 0;
}

static int tsens_tz_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;
	unsigned int reg;

	if (!tm_sensor || trip < 0 || !temp)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);

	if (tmdev->suspended) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return -ENODEV;
	}

	reg = readl(TSENS_THRESHOLD_ADDR);
	spin_unlock_irqrestore(&tmdev->lock, flags);

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		reg = (reg & TSENS_THRESHOLD_MAX_LIMIT_MASK) >> 24;
		break;
	case TSENS_TRIP_STAGE2:
		reg = (reg & TSENS_THRESHOLD_UPPER_LIMIT_MASK) >> 8;
		break;
	case TSENS_TRIP_STAGE1:
		reg = (reg & TSENS_THRESHOLD_LOWER_LIMIT_MASK) >> 0;
		break;
	case TSENS_TRIP_STAGE0:
		reg = (reg & TSENS_THRESHOLD_MIN_LIMIT_MASK) >> 16;
		break;
	default:
		return -EINVAL;
	}

	*temp = tsens_tz_code_to_degC(reg);

	return 0;
}

static int tsens_tz_get_crit_temp(struct thermal_zone_device *thermal,
				  unsigned long *temp)
{
	return tsens_tz_get_trip_temp(thermal, TSENS_TRIP_STAGE3, temp);
}

static int tsens_tz_set_trip_temp(struct thermal_zone_device *thermal,
				   int trip, long temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned long flags;
	unsigned int reg_th, reg_cntl;
	int code, hi_code, lo_code, code_err_chk, ret = 0;

	code_err_chk = code = tsens_tz_degC_to_code(temp);
	if (!tm_sensor || trip < 0)
		return -EINVAL;

	spin_lock_irqsave(&tmdev->lock, flags);

	if (tmdev->suspended) {
		ret = -ENODEV;
		goto done;
	}

	lo_code = 0;
	hi_code = TSENS_THRESHOLD_MAX_CODE;

	reg_cntl = readl(TSENS_CNTL_ADDR);
	reg_th = readl(TSENS_THRESHOLD_ADDR);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code <<= 24;
		reg_th &= ~TSENS_THRESHOLD_MAX_LIMIT_MASK;

		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE2:
		code <<= 8;
		reg_th &= ~TSENS_THRESHOLD_UPPER_LIMIT_MASK;

		if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE1:
		reg_th &= ~TSENS_THRESHOLD_LOWER_LIMIT_MASK;

		if (!(reg_cntl & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	case TSENS_TRIP_STAGE0:
		code <<= 16;
		reg_th &= ~TSENS_THRESHOLD_MIN_LIMIT_MASK;

		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(reg_cntl & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	default:
		ret = -EINVAL;
		goto done;
	}

	if (code_err_chk < lo_code || code_err_chk > hi_code) {
		ret = -EINVAL;
		goto done;
	}

	writel(reg_th | code, TSENS_THRESHOLD_ADDR);

done:
	spin_unlock_irqrestore(&tmdev->lock, flags);
	if (!ret)
		tsens_tz_force_update(thermal);

	return ret;
}

static void notify_uspace_tsens(struct thermal_zone_device *tz) {
	/* Currently only Sensor0 is supported. We added support
	   to notify only the supported Sensor and this portion
	   needs to be revisited once other sensors are supported */
	sysfs_notify(&tz->device.kobj, NULL, "type");
}

static int tsens_tz_notify(struct thermal_zone_device *tz, int trip,
			enum thermal_trip_type trip_type) {
	notify_uspace_tsens(tz);
	return 0;
}

static struct thermal_zone_device_ops tsens_thermal_zone_ops = {
	.get_temp = tsens_tz_get_temp,
	.get_mode = tsens_tz_get_mode,
	.set_mode = tsens_tz_set_mode,
	.get_trip_type = tsens_tz_get_trip_type,
	.activate_trip_type = tsens_tz_activate_trip_type,
	.get_trip_temp = tsens_tz_get_trip_temp,
	.set_trip_temp = tsens_tz_set_trip_temp,
	.get_crit_temp = tsens_tz_get_crit_temp,
	.notify = tsens_tz_notify,
};

static void update_tsens_fn(struct work_struct *work)
{
	int i;
	struct thermal_zone_device *tz;
	struct tsens_tm_device *tm = container_of(work, struct tsens_tm_device,
					work);

	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		tz = tm->sensor[i].tz_dev;
		if (!tz)
			continue;
		thermal_zone_device_update(tz);
	}
}

static irqreturn_t tsens_isr(int irq, void *data)
{
	unsigned long flags;
	unsigned int reg = readl(TSENS_CNTL_ADDR);

	spin_lock_irqsave(&tmdev->lock, flags);
	writel(reg | TSENS_LOWER_STATUS_CLR | TSENS_UPPER_STATUS_CLR,
			TSENS_CNTL_ADDR);
	spin_unlock_irqrestore(&tmdev->lock, flags);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t tsens_isr_thread(int irq, void *data)
{
	struct tsens_tm_device *tm = data;
	unsigned long flags;
	unsigned int threshold, threshold_low, i, code, reg, sensor, mask;
	bool upper_th_x, lower_th_x;

	spin_lock_irqsave(&tmdev->lock, flags);

	mask = ~(TSENS_LOWER_STATUS_CLR | TSENS_UPPER_STATUS_CLR);
	threshold = readl(TSENS_THRESHOLD_ADDR);
	threshold_low = threshold & TSENS_THRESHOLD_LOWER_LIMIT_MASK;
	threshold = (threshold & TSENS_THRESHOLD_UPPER_LIMIT_MASK) >> 8;
	reg = sensor = readl(TSENS_CNTL_ADDR);
	sensor &= (SENSOR0_EN | SENSOR1_EN | SENSOR2_EN |
						SENSOR3_EN | SENSOR4_EN);
	sensor >>= 3;
	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		if (sensor & 1) {
			/* Call thermal_zone_device_update() */
			schedule_work(&tm->work);
		}
		sensor >>= 1;
	}
	writel(reg & mask, TSENS_CNTL_ADDR);

	spin_unlock_irqrestore(&tmdev->lock, flags);
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int tsens_suspend(struct device *dev)
{
	unsigned long flags;
	unsigned int reg;

	disable_irq(TSENS_UPPER_LOWER_INT);

	spin_lock_irqsave(&tmdev->lock, flags);
	tmdev->suspended = 1;

	tmdev->pm_tsens_thr_data = readl_relaxed(TSENS_THRESHOLD_ADDR);
	reg = readl_relaxed(TSENS_CNTL_ADDR);
	writel_relaxed(reg & ~(TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	tmdev->prev_reading_avail = 0;

	spin_unlock_irqrestore(&tmdev->lock, flags);

	mb();
	return 0;
}

static int tsens_resume(struct device *dev)
{
	unsigned long flags;
	unsigned int reg;

	spin_lock_irqsave(&tmdev->lock, flags);

	reg = readl_relaxed(TSENS_CNTL_ADDR);
	writel_relaxed(reg | TSENS_SW_RST, TSENS_CNTL_ADDR);
	reg |= TSENS_SLP_CLK_ENA | TSENS_EN | (TSENS_MEASURE_PERIOD << 16) |
		TSENS_MIN_STATUS_MASK | TSENS_MAX_STATUS_MASK |
		(((1 << TSENS_NUM_SENSORS) - 1) << 3);

	reg = (reg & ~TSENS_CONFIG_MASK) | (TSENS_CONFIG << TSENS_CONFIG_SHIFT);
	writel_relaxed(reg, TSENS_CNTL_ADDR);

	if (tmdev->sensor->mode == THERMAL_DEVICE_DISABLED) {
		writel_relaxed(reg & ~((((1 << TSENS_NUM_SENSORS) - 1) << 3)
			| TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	}

	writel_relaxed(tmdev->pm_tsens_thr_data, TSENS_THRESHOLD_ADDR);

	tmdev->suspended = 0;
	spin_unlock_irqrestore(&tmdev->lock, flags);

	enable_irq(TSENS_UPPER_LOWER_INT);
	mb();
	return 0;
}

static const struct dev_pm_ops tsens_pm_ops = {
	.suspend	= tsens_suspend,
	.resume		= tsens_resume,
};
#endif

static int __devinit tsens_tm_probe(struct platform_device *pdev)
{
	unsigned int reg, i, calib_data, calib_data_backup;
	int rc;

	calib_data = (readl(TSENS_QFPROM_ADDR) & TSENS_QFPROM_TEMP_SENSOR0_MASK)
					>> TSENS_QFPROM_TEMP_SENSOR0_SHIFT;
	calib_data_backup = readl(TSENS_QFPROM_ADDR)
					>> TSENS_QFPROM_RED_TEMP_SENSOR0_SHIFT;

	if (calib_data_backup)
		calib_data = calib_data_backup;

	if (!calib_data) {
		pr_err("%s: No temperature sensor data for calibration"
						" in QFPROM!\n", __func__);
		return -ENODEV;
	}

	tmdev = kzalloc(sizeof(struct tsens_tm_device), GFP_KERNEL);
	if (tmdev == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	spin_lock_init(&tmdev->lock);

	platform_set_drvdata(pdev, tmdev);

	tmdev->offset = TSENS_FACTOR * TSENS_CAL_DEGC
			- (int)(TSENS_FACTOR * TSENS_SLOPE) * calib_data;
	tmdev->prev_reading_avail = 0;

	INIT_WORK(&tmdev->work, update_tsens_fn);

	reg = readl(TSENS_CNTL_ADDR);
	writel(reg | TSENS_SW_RST, TSENS_CNTL_ADDR);
	reg |= TSENS_SLP_CLK_ENA | TSENS_EN | (TSENS_MEASURE_PERIOD << 16) |
		TSENS_LOWER_STATUS_CLR | TSENS_UPPER_STATUS_CLR |
		TSENS_MIN_STATUS_MASK | TSENS_MAX_STATUS_MASK |
		(((1 << TSENS_NUM_SENSORS) - 1) << 3);

	/* set TSENS_CONFIG bits (bits 29:28 of TSENS_CNTL) to '01';
		this setting found to be optimal. */
	reg = (reg & ~TSENS_CONFIG_MASK) | (TSENS_CONFIG << TSENS_CONFIG_SHIFT);

	writel(reg, TSENS_CNTL_ADDR);

	writel((TSENS_LOWER_LIMIT_TH << 0) | (TSENS_UPPER_LIMIT_TH << 8) |
		(TSENS_MIN_LIMIT_TH << 16) | (TSENS_MAX_LIMIT_TH << 24),
			TSENS_THRESHOLD_ADDR);

	tmdev->suspended = 0;

	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		char name[17];
		sprintf(name, "tsens_tz_sensor%d", i);

		tmdev->sensor[i].mode = THERMAL_DEVICE_ENABLED;
		tmdev->sensor[i].tz_dev = thermal_zone_device_register(name,
				TSENS_TRIP_NUM, &tmdev->sensor[i],
				&tsens_thermal_zone_ops, 0, 0, 0, 0);
		if (tmdev->sensor[i].tz_dev == NULL) {
			pr_err("%s: thermal_zone_device_register() failed.\n",
			__func__);
			kfree(tmdev);
			return -ENODEV;
		}
		tmdev->sensor[i].sensor_num = i;
		tmdev->sensor[i].mode = THERMAL_DEVICE_DISABLED;
	}

	rc = request_threaded_irq(TSENS_UPPER_LOWER_INT, tsens_isr,
		tsens_isr_thread, 0, "tsens", tmdev);
	if (rc < 0) {
		pr_err("%s: request_irq FAIL: %d\n", __func__, rc);
		kfree(tmdev);
		return rc;
	}

	writel(reg & ~((((1 << TSENS_NUM_SENSORS) - 1) << 3)
			| TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	pr_notice("%s: OK\n", __func__);
	return 0;
}

static int __devexit tsens_tm_remove(struct platform_device *pdev)
{
	struct tsens_tm_device *tmdev = platform_get_drvdata(pdev);
	unsigned long flags;
	unsigned int reg, i;

	spin_lock_irqsave(&tmdev->lock, flags);
	reg = readl(TSENS_CNTL_ADDR);
	writel(reg & ~(TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	spin_unlock_irqrestore(&tmdev->lock, flags);

	for (i = 0; i < TSENS_NUM_SENSORS; i++)
		thermal_zone_device_unregister(tmdev->sensor[i].tz_dev);
	platform_set_drvdata(pdev, NULL);
	free_irq(TSENS_UPPER_LOWER_INT, tmdev);
	kfree(tmdev);

	return 0;
}

static struct platform_driver tsens_tm_driver = {
	.probe	= tsens_tm_probe,
	.remove	= __devexit_p(tsens_tm_remove),
	.driver	= {
		.name = "tsens-tm",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &tsens_pm_ops,
#endif
	},
};

static int __init tsens_init(void)
{
	return platform_driver_register(&tsens_tm_driver);
}

static void __exit tsens_exit(void)
{
	platform_driver_unregister(&tsens_tm_driver);
}

module_init(tsens_init);
module_exit(tsens_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM Temperature Sensor driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:tsens-tm");
