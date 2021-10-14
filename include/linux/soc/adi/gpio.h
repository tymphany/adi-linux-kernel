// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * @todo next step is to unify the arm/mach-sc5xx headers under linux/soc/adi
 * for now we forward to them based on platform
 */

#ifndef SOC_ADI_GPIO_H
#define SOC_ADI_GPIO_H

#ifdef CONFIG_ARCH_SC59X_64
#include <linux/soc/adi/mach-sc59x/gpio.h>
#else
#include <mach/gpio.h>
#endif

#endif

