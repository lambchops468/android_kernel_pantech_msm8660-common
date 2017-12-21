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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <linux/pm.h>

// p15060, MDM boot up fail fix
#include <linux/gpio.h>

/* Trips: from very hot to very cold
 *
 * When temperature > STAGE2, an interrupt is generated. Then, the high
 * temperature interrupt is disabled and will not fire again until an interrupt
 * has fired for temperature < STAGE1.
 * When temperature < STAGE1 an interrupt is generated. Then, the high
 * temperature interrupt is disabled and will not fire again until an interrupt
 * has fired for temperature > STAGE2
 * A temp read occurs regularly (configured by TSENS_MEASURE_PERIOD)
 *
 * STAGE3 trip action is an immediate shutdown via hardware. The kernel is not
 * involved.
 * STAGE0 trip action is untested, but is probably an immediate shutdown.
 *
 * STAGE12 is implemented using STAGE1 & STAGE2. It is used to drive the
 * msm_tsens_throttle driver. STAGE2's trip temperature is set to the
 * temperature that starts throttling the CPU. STAGE1's trip temperature is set
 * to STAGE2 - 1 and is used to rearm the STAGE2 interrupt.
 */
enum tsens_trip_type {
	TSENS_TRIP_STAGE3 = 0,
	TSENS_TRIP_STAGE2,
	TSENS_TRIP_STAGE1,
	TSENS_TRIP_STAGE0,
	TSENS_TRIP_STAGE12,
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
#define TSENS_STATUS_MASK ~(TSENS_MIN_STATUS_MASK | \
				TSENS_LOWER_STATUS_CLR | \
				TSENS_UPPER_STATUS_CLR | \
				TSENS_MAX_STATUS_MASK)
#define TSENS_MEASURE_PERIOD 4 /* 1 sec. default as required by Willie */
#define TSENS_SLP_CLK_ENA (1 << 24)
#define TSENS_THRESHOLD_ADDR (MSM_CLK_CTL_BASE + 0x00003624)
#define TSENS_THRESHOLD_MAX_CODE (0xff)
#define TSENS_THRESHOLD_MAX_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 24)
#define TSENS_THRESHOLD_MIN_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 16)
#define TSENS_THRESHOLD_UPPER_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 8)
#define TSENS_THRESHOLD_LOWER_LIMIT_MASK (TSENS_THRESHOLD_MAX_CODE << 0)
/* Initial temperature threshold values, for when stage12_enabled = true
 * Factory Thresholds are:
 *  TSENS_LOWER_LIMIT_TH   0x50  // -7 deg C
 *  TSENS_UPPER_LIMIT_TH   0xdf  // 94 deg C
 *  TSENS_MIN_LIMIT_TH     0x38  // -23 deg C
 *  TSENS_MAX_LIMIT_TH     0xff  // 116 deg C
 */
#define TSENS_LOWER_LIMIT_TH   0xa2  // 51 deg C - cpu unthrottle temp
#define TSENS_UPPER_LIMIT_TH   0xa3  // 52 deg C - cpu throttle temp
#define TSENS_MIN_LIMIT_TH     0x38  //-23 deg C - cold emergency shutdown temp
#define TSENS_MAX_LIMIT_TH     0xc4  // 75 deg C - hot emergency shutdown temp

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
	// Whether TSENS_TRIP_STAGE1 & TSENS_TRIP_STAGE2 are bound together to
	// implement TSENS_TRIP_STAGE12 or if they are independent.
	enum thermal_trip_activation_mode stage12_enabled;
	int offset;
	uint32_t disabled_trips;
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
	struct tsens_tm_device_sensor *tm_sensor = tz->devdata;
	unsigned long flags;

	if (!tm_sensor)
		return;

	spin_lock_irqsave(&tmdev->lock, flags);
	if (tm_sensor->mode != THERMAL_DEVICE_ENABLED) {
		spin_unlock_irqrestore(&tmdev->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&tmdev->lock, flags);

	thermal_zone_device_update(tz);
}

static int tsens_tz_bind(struct thermal_zone_device* tz,
			struct thermal_cooling_device *cdev) {
	if (!tz || !cdev) {
		return -EINVAL;
	}
	// Only bind Processor cooling devices.
	if (strncmp("Processor", cdev->type, sizeof("Processor"))) {
		return 0;
	}
	return thermal_zone_bind_cooling_device(tz, TSENS_TRIP_STAGE12, cdev);
}

static int tsens_tz_unbind(struct thermal_zone_device* tz,
			struct thermal_cooling_device *cdev) {
	if (!tz || !cdev) {
		return -EINVAL;
	}
	// Only bind Processor cooling devices.
	if (strncmp("Processor", cdev->type, sizeof("Processor"))) {
		return 0;
	}
	return thermal_zone_unbind_cooling_device(tz, TSENS_TRIP_STAGE12, cdev);
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
	do {
		// Check to ensure that the state of the device hasn't changed
		// while unlocked.
		if (tm_sensor->mode != THERMAL_DEVICE_ENABLED) {
			spin_unlock_irqrestore(&tmdev->lock, flags);
			return -EINVAL;
		}
		if (tmdev->suspended) {
			spin_unlock_irqrestore(&tmdev->lock, flags);
			return -ENODEV;
		}
		
		if (!tmdev->prev_reading_avail) {
			if (readl(TSENS_INT_STATUS_ADDR) & TSENS_TRDY_MASK) {
				tmdev->prev_reading_avail = 1;
			} else {
				// Unlock before yielding to another thread.
				spin_unlock_irqrestore(&tmdev->lock, flags);
				msleep(1);
				spin_lock_irqsave(&tmdev->lock, flags);
				// Go back to the top of the loop to perform
				// checks.
			}
		}
	} while (!tmdev->prev_reading_avail);

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
	case TSENS_TRIP_STAGE12:
		*type = THERMAL_TRIP_PASSIVE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

// reg_th is the tsens threshold register (TSENS_THRESHOLD_ADDR)
// trip is one of tsens_trip_type
// code is the adc code given by tsens_tz_degC_to_code()
//
// Check if the given code will be valid when set as trip's threshold.
// Checks the adjacent enabled thresholds to ensure that the new code is not
// less than a colder trip and not more than a hotter trip.
static int tsens_validate_adjacent_thresholds(unsigned int reg_th,
						int trip, unsigned int code) {
	unsigned int hi_code, lo_code;

	hi_code = UINT_MAX;
	lo_code = 0;

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		if (!(tmdev->disabled_trips & TSENS_UPPER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(tmdev->disabled_trips & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(tmdev->disabled_trips & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE2:
		if (!(tmdev->disabled_trips & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		if (!(tmdev->disabled_trips & TSENS_LOWER_STATUS_CLR))
			lo_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(tmdev->disabled_trips & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		break;
	case TSENS_TRIP_STAGE1:
		if (!(tmdev->disabled_trips & TSENS_MIN_STATUS_MASK))
			lo_code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK)
									>> 16;
		if (!(tmdev->disabled_trips & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(tmdev->disabled_trips & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	case TSENS_TRIP_STAGE0:
		if (!(tmdev->disabled_trips & TSENS_LOWER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK);
		else if (!(tmdev->disabled_trips & TSENS_UPPER_STATUS_CLR))
			hi_code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK)
									>> 8;
		else if (!(tmdev->disabled_trips & TSENS_MAX_STATUS_MASK))
			hi_code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK)
									>> 24;
		break;
	default:
		return -EINVAL;
	}

	if (code < lo_code || code > hi_code) {
		return -EINVAL;
	}

	return 0;
}

static int __tsens_tz_get_trip_mode(struct thermal_zone_device *thermal,
				int trip,
				enum thermal_trip_activation_mode *mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned int mask;
	int ret;

	if (!tm_sensor || trip < 0)
		return -EINVAL;

	if (tmdev->suspended) {
		return -ENODEV;
	}

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		mask = TSENS_MAX_STATUS_MASK;
		break;
	case TSENS_TRIP_STAGE2:
		mask = TSENS_UPPER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE1:
		mask = TSENS_LOWER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE0:
		mask = TSENS_MIN_STATUS_MASK;
		break;
	default:
		return -EINVAL;
	}

	if (tmdev->disabled_trips & mask) {
		*mode = THERMAL_TRIP_ACTIVATION_DISABLED;
	} else {
		*mode = THERMAL_TRIP_ACTIVATION_ENABLED;
	}

	return 0;
}

static int __tsens_tz_activate_trip_type(struct thermal_zone_device *thermal,
					int trip,
					enum thermal_trip_activation_mode mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned int reg_cntl, reg_th, code, mask;
	int ret;

	if (!tm_sensor || trip < 0)
		return -EINVAL;

	if (tmdev->suspended) {
		return -ENODEV;
	}

	reg_cntl = readl(TSENS_CNTL_ADDR);
	reg_th = readl(TSENS_THRESHOLD_ADDR);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code = (reg_th & TSENS_THRESHOLD_MAX_LIMIT_MASK) >> 24;
		mask = TSENS_MAX_STATUS_MASK;
		break;
	case TSENS_TRIP_STAGE2:
		code = (reg_th & TSENS_THRESHOLD_UPPER_LIMIT_MASK) >> 8;
		mask = TSENS_UPPER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE1:
		code = (reg_th & TSENS_THRESHOLD_LOWER_LIMIT_MASK) >> 0;
		mask = TSENS_LOWER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE0:
		code = (reg_th & TSENS_THRESHOLD_MIN_LIMIT_MASK) >> 16;
		mask = TSENS_MIN_STATUS_MASK;
		break;
	default:
		return -EINVAL;
	}


	if (mode == THERMAL_TRIP_ACTIVATION_DISABLED) {
		tmdev->disabled_trips |= mask;
		writel(reg_cntl | mask, TSENS_CNTL_ADDR);
	} else {
		ret = tsens_validate_adjacent_thresholds(reg_th, trip, code);
		if (ret)
			return ret;

		tmdev->disabled_trips &= ~mask;
		writel(reg_cntl & ~mask, TSENS_CNTL_ADDR);
	}

	return 0;
}

static int tsens_tz_activate_trip_type(struct thermal_zone_device *thermal,
			int trip, enum thermal_trip_activation_mode mode)
{
	enum thermal_trip_activation_mode old_stage1_mode;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&tmdev->lock, flags);
	switch (trip) {
	case TSENS_TRIP_STAGE2:
	case TSENS_TRIP_STAGE1:
		// Manipulating STAGE1 or STAGE2 directly disables the combined
		// behavior for implementing STAGE12.
		tmdev->stage12_enabled = THERMAL_TRIP_ACTIVATION_DISABLED;
		// fall through
	case TSENS_TRIP_STAGE3:
	case TSENS_TRIP_STAGE0:
		ret = __tsens_tz_activate_trip_type(thermal, trip, mode);
		break;
	case TSENS_TRIP_STAGE12:
		ret = __tsens_tz_get_trip_mode(thermal, TSENS_TRIP_STAGE1,
							&old_stage1_mode);
		if (ret)
			break;

		ret = __tsens_tz_activate_trip_type(thermal, TSENS_TRIP_STAGE1,
							mode);
		if (ret)
			break;
		// Note that it is possible that sensor's interrupt might fire
		// here.

		ret = __tsens_tz_activate_trip_type(thermal,
						TSENS_TRIP_STAGE2,
						mode);
		if (ret) {
			// Undo TSENS_TRIP_STAGE1
			__tsens_tz_activate_trip_type(thermal,
				TSENS_TRIP_STAGE1, old_stage1_mode);
			break;
		}

		tmdev->stage12_enabled = mode;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&tmdev->lock, flags);
	if (!ret)
		tsens_tz_force_update(thermal);
	return ret;
}

static int __tsens_tz_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned int reg;

	if (!tm_sensor || trip < 0 || !temp)
		return -EINVAL;

	if (tmdev->suspended) {
		return -ENODEV;
	}

	reg = readl(TSENS_THRESHOLD_ADDR);

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		reg = (reg & TSENS_THRESHOLD_MAX_LIMIT_MASK) >> 24;
		break;
	case TSENS_TRIP_STAGE2:
	case TSENS_TRIP_STAGE12:
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

static int tsens_tz_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&tmdev->lock, flags);

	if (trip != TSENS_TRIP_STAGE12) {
		ret = __tsens_tz_get_trip_temp(thermal, trip, temp);
	} else {
		ret = __tsens_tz_get_trip_temp(thermal, TSENS_TRIP_STAGE2,
									temp);
		// If stage12 is not enabled, disable passive cooling by
		// returning an absurdly large trip temperature.
		// We call __tsens_tz_get_trip_temp() first to validate
		// arguments.
		if (!ret && !tmdev->stage12_enabled)
			*temp = LONG_MAX;
	}

	spin_unlock_irqrestore(&tmdev->lock, flags);
	return ret;
}

static int tsens_tz_get_crit_temp(struct thermal_zone_device *thermal,
				  unsigned long *temp)
{
	return tsens_tz_get_trip_temp(thermal, TSENS_TRIP_STAGE3, temp);
}

static int __tsens_tz_set_trip_temp(struct thermal_zone_device *thermal,
				   int trip, long temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned int reg_th, code, code_err_chk;
	int ret;

	code_err_chk = code = tsens_tz_degC_to_code(temp);
	if (!tm_sensor || trip < 0)
		return -EINVAL;

	if (tmdev->suspended) {
		return -ENODEV;
	}

	reg_th = readl(TSENS_THRESHOLD_ADDR);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code <<= 24;
		reg_th &= ~TSENS_THRESHOLD_MAX_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE2:
		code <<= 8;
		reg_th &= ~TSENS_THRESHOLD_UPPER_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE1:
		reg_th &= ~TSENS_THRESHOLD_LOWER_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE0:
		code <<= 16;
		reg_th &= ~TSENS_THRESHOLD_MIN_LIMIT_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = tsens_validate_adjacent_thresholds(reg_th, trip, code_err_chk);
	if (ret) {
		return ret;
	}

	writel(reg_th | code, TSENS_THRESHOLD_ADDR);

	return 0;
}

static int tsens_tz_set_trip_temp(struct thermal_zone_device *thermal,
				   int trip, long temp)
{
	unsigned long flags;
	unsigned long old_temp1, old_temp2;
	int ret;

	spin_lock_irqsave(&tmdev->lock, flags);
	if (trip != TSENS_TRIP_STAGE12) {
		ret = __tsens_tz_set_trip_temp(thermal, trip, temp);
	} else {
		ret = __tsens_tz_get_trip_temp(thermal, TSENS_TRIP_STAGE1,
						&old_temp1);
		if (ret)
			goto out;

		ret = __tsens_tz_get_trip_temp(thermal, TSENS_TRIP_STAGE2,
						&old_temp2);
		if (ret)
			goto out;

		if (temp > old_temp2) {
			// Change STAGE2 first so STAGE1 won't overlap with
			// STAGE2.
			ret = __tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE2,
							temp);
			if (ret)
				goto out;

			ret = __tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE1,
							temp-1);
			if (ret) {
				// Undo TSENS_TRIP_STAGE2
				__tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE2,
							old_temp2);
				goto out;
			}
		} else {
			// Change STAGE1 first so STAGE2 won't overlap with
			// STAGE1.
			ret = __tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE1,
							temp-1);
			if (ret)
				goto out;

			ret = __tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE2,
							temp);
			if (ret) {
				// Undo TSENS_TRIP_STAGE1
				__tsens_tz_set_trip_temp(thermal,
							TSENS_TRIP_STAGE1,
							old_temp1);
				goto out;
			}
		}
	}
out:
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
	.bind = tsens_tz_bind,
	.unbind = tsens_tz_unbind,
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
	// Disable lower & upper interrupt so that we won't loop back into
	// this function when we return from this function and the kernel
	// re-enables our interrupt.
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

	mask = ~(TSENS_LOWER_STATUS_CLR | TSENS_UPPER_STATUS_CLR) |
		tmdev->disabled_trips;
	threshold = readl(TSENS_THRESHOLD_ADDR);
	threshold_low = threshold & TSENS_THRESHOLD_LOWER_LIMIT_MASK;
	threshold = (threshold & TSENS_THRESHOLD_UPPER_LIMIT_MASK) >> 8;
	reg = sensor = readl(TSENS_CNTL_ADDR);
	sensor &= (SENSOR0_EN | SENSOR1_EN | SENSOR2_EN |
						SENSOR3_EN | SENSOR4_EN);
	sensor >>= 3;
	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		if (sensor & 1) {
			code = readl(TSENS_S0_STATUS_ADDR + (i << 2));
			upper_th_x = code >= threshold;
			lower_th_x = code <= threshold_low;
			if (upper_th_x) {
				// Disable upper limit interrupt until lower
				// interrupt occurs.
				mask |= TSENS_UPPER_STATUS_CLR;
			}
			if (lower_th_x) {
				// Disable lower limit interrupt until upper
				// interrupt occurs.
				mask |= TSENS_LOWER_STATUS_CLR;
			}
			if (upper_th_x || lower_th_x) {
				/* Call thermal_zone_device_update() */
				schedule_work(&tm->work);
				pr_info("msm_tsens trip point triggered by "
					"current temperature (%d degrees)\n",
					tsens_tz_code_to_degC(code));
			}
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

	tmdev->pm_tsens_thr_data = readl(TSENS_THRESHOLD_ADDR);
	reg = readl(TSENS_CNTL_ADDR);
	writel(reg & ~(TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	tmdev->prev_reading_avail = 0;

	spin_unlock_irqrestore(&tmdev->lock, flags);

	return 0;
}

static int tsens_resume(struct device *dev)
{
	unsigned long flags;
	unsigned int reg;

	spin_lock_irqsave(&tmdev->lock, flags);

	reg = readl(TSENS_CNTL_ADDR);
	writel(reg | TSENS_SW_RST, TSENS_CNTL_ADDR);

	writel(tmdev->pm_tsens_thr_data, TSENS_THRESHOLD_ADDR);

	reg |= TSENS_SLP_CLK_ENA | TSENS_EN | (TSENS_MEASURE_PERIOD << 16) |
		(((1 << TSENS_NUM_SENSORS) - 1) << 3);
	reg = (reg & TSENS_STATUS_MASK) | tmdev->disabled_trips;

	reg = (reg & ~TSENS_CONFIG_MASK) | (TSENS_CONFIG << TSENS_CONFIG_SHIFT);
	writel(reg, TSENS_CNTL_ADDR);

	if (tmdev->sensor->mode == THERMAL_DEVICE_DISABLED) {
		writel(reg & ~((((1 << TSENS_NUM_SENSORS) - 1) << 3)
			| TSENS_SLP_CLK_ENA | TSENS_EN), TSENS_CNTL_ADDR);
	}

	tmdev->suspended = 0;
	spin_unlock_irqrestore(&tmdev->lock, flags);

	enable_irq(TSENS_UPPER_LOWER_INT);
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

	// Set default trip threshold temperatures.
	writel((TSENS_LOWER_LIMIT_TH << 0) | (TSENS_UPPER_LIMIT_TH << 8) |
		(TSENS_MIN_LIMIT_TH << 16) | (TSENS_MAX_LIMIT_TH << 24),
			TSENS_THRESHOLD_ADDR);

	// Enable sensors.
	reg |= TSENS_SLP_CLK_ENA | TSENS_EN | (TSENS_MEASURE_PERIOD << 16) |
		(((1 << TSENS_NUM_SENSORS) - 1) << 3);
	// Enable all trip points.
	reg &= TSENS_STATUS_MASK;
	tmdev->disabled_trips = 0;
	tmdev->stage12_enabled = THERMAL_TRIP_ACTIVATION_ENABLED;
	/* set TSENS_CONFIG bits (bits 29:28 of TSENS_CNTL) to '01';
		this setting found to be optimal. */
	reg = (reg & ~TSENS_CONFIG_MASK) | (TSENS_CONFIG << TSENS_CONFIG_SHIFT);

	writel(reg, TSENS_CNTL_ADDR);

	tmdev->suspended = 0;

	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		char name[17];
		sprintf(name, "tsens_tz_sensor%d", i);

		tmdev->sensor[i].mode = THERMAL_DEVICE_ENABLED;
		tmdev->sensor[i].sensor_num = i;
		tmdev->sensor[i].tz_dev = thermal_zone_device_register(name,
				TSENS_TRIP_NUM, &tmdev->sensor[i],
				// Polling faster than 1000ms is not useful
				// because hardware updates at 1 Hz.
				&tsens_thermal_zone_ops, 1, 1, 1000, 0);
		if (tmdev->sensor[i].tz_dev == NULL) {
			pr_err("%s: thermal_zone_device_register() failed.\n",
			__func__);
			rc = -ENODEV;
			goto register_err;
		}
	}

	rc = request_threaded_irq(TSENS_UPPER_LOWER_INT, tsens_isr,
		tsens_isr_thread, 0, "tsens", tmdev);
	if (rc < 0) {
		pr_err("%s: request_irq FAIL: %d\n", __func__, rc);
		goto register_err;
	}

	pr_notice("%s: OK\n", __func__);
	return 0;

register_err:
	for (i = 0; i < TSENS_NUM_SENSORS; i++) {
		if (!tmdev->sensor[i].tz_dev) {
			continue;
		}
		thermal_zone_device_unregister(tmdev->sensor[i].tz_dev);
	}

	kfree(tmdev);

	return rc;
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
