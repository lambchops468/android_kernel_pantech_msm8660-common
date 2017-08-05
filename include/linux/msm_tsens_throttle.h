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

#ifndef __MSM_TSENS_THROTTLE_H
#define __MSM_TSENS_THROTTLE_H

#ifdef CONFIG_THERMAL_TSENS_THROTTLE
int msm_tsens_throttle_init(void);
#else
int msm_tsens_throttle_init(void) { return 0; }
#endif

#endif /* __MSM_TSENS_THROTTLE_H */
