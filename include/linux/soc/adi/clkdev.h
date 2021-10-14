// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef SOC_ADI_CLKDEV_H
#define SOC_ADI_CLKDEV_H
/**
 * @todo the next step is to unify the arm/mach-sc5xx clkdev headers
 * underneath linux/soc/adi/
 * for now just forward to them so that we have one entry point, and only
 * sc59x-64 has its own header here
 */

#ifdef CONFIG_ARCH_SC59X_64
#include <linux/soc/adi/mach-sc59x/clkdev.h>
#else
#include <mach/clkdev.h>
#endif

#endif
