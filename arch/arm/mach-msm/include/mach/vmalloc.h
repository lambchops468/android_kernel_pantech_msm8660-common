/* arch/arm/mach-msm/include/mach/vmalloc.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, The Linux Foundation. All rights reserved.
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

#ifndef __ASM_ARCH_MSM_VMALLOC_H
#define __ASM_ARCH_MSM_VMALLOC_H

#ifdef CONFIG_VMSPLIT_2G
#define VMALLOC_END	  (PAGE_OFFSET + 0x7A000000)
#elif defined(CONFIG_VMSPLIT_3G_OPT)
/* This value allows all 1GB physical ram to fit in low memory and eliminate use
 * of high memory. See PAGE_OFFSET in arch/arm/Kconfig
 */
#define VMALLOC_END	  (PAGE_OFFSET + 0x48000000)
#else
#define VMALLOC_END   (PAGE_OFFSET + 0x3A000000)
#endif

#endif

