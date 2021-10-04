/*
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ARCH_GPIO_H__
#define __ARCH_GPIO_H__

#ifdef CONFIG_ARCH_SC59X_64
#include <linux/soc/adi/mach-sc59x/sc59x.h>
#else
#include <mach/gpio-sc59x.h>
#endif

#include <asm/irq.h>
#include <asm/errno.h>
#include <asm-generic/gpio.h>

/*
 * gpio port registers layout
 */
struct gpio_port_t {
	u32 port_fer;
	u32 port_fer_set;
	u32 port_fer_clear;
	u32 data;
	u32 data_set;
	u32 data_clear;
	u32 dir;
	u32 dir_set;
	u32 dir_clear;
	u32 inen;
	u32 inen_set;
	u32 inen_clear;
	u32 port_mux;
	u32 toggle;
	u32 polar;
	u32 polar_set;
	u32 polar_clear;
	u32 lock;
	u32 spare;
	u32 padding[12];
	u32 revid;
};

/*
 * gpio pint registers layout
 */
struct gpio_pint_regs {
	u32 mask_set;
	u32 mask_clear;
	u32 request;
	u32 assign;
	u32 edge_set;
	u32 edge_clear;
	u32 invert_set;
	u32 invert_clear;
	u32 pinstate;
	u32 latch;
	u32 __pad0[2];
};

#ifdef CONFIG_MACH_SC59X_DT
int softconfig_of_set_active_pin_output(struct device *dev,
				struct device_node *np, const char *propname, int index,
				int *pin, int *active_flag, bool active);

int softconfig_of_set_group_active_pins_output(struct device *dev,
				struct device_node *np, const char *propname, bool active);
#endif

#endif /* __ARCH_GPIO_H__ */
