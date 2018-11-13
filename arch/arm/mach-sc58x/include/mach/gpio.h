/*
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ARCH_GPIO_H__
#define __ARCH_GPIO_H__

#include <mach/gpio-sc58x.h>

#include <asm/irq.h>
#include <asm/errno.h>
#include <asm-generic/gpio.h>

/*
 * gpio port registers layout
 */
struct gpio_port_t {
	unsigned long port_fer;
	unsigned long port_fer_set;
	unsigned long port_fer_clear;
	unsigned long data;
	unsigned long data_set;
	unsigned long data_clear;
	unsigned long dir;
	unsigned long dir_set;
	unsigned long dir_clear;
	unsigned long inen;
	unsigned long inen_set;
	unsigned long inen_clear;
	unsigned long port_mux;
	unsigned long toggle;
	unsigned long polar;
	unsigned long polar_set;
	unsigned long polar_clear;
	unsigned long lock;
	unsigned long spare;
	unsigned long padding[12];
	unsigned long revid;
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

#ifdef CONFIG_MACH_SC58X_DT
int softconfig_of_set_active_pin_output(struct device *dev,
				struct device_node *np, const char *propname, int index,
				int *pin, int *active_flag, bool active);

int softconfig_of_set_group_active_pins_output(struct device *dev,
				struct device_node *np, const char *propname, bool active);
#endif

#endif /* __ARCH_GPIO_H__ */
