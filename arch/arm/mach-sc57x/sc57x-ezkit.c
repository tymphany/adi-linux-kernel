/*
 * machine start entries for ADI processor on-chip memory
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/io.h>

#include <linux/soc/adi/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/sc57x.h>
#include <sound/sc5xx-dai.h>
#include <linux/of_gpio.h>
#include <mach/gpio.h>

#include "core.h"

void pads_init(void)
{
	/* nothing to be done here for dai in sc57x */
}
EXPORT_SYMBOL(pads_init);

/**
 * Active all reboot-pins from softconfig as listed in the board dts file
 */
static void sc57x_ezkit_restart(enum reboot_mode mode, const char *cmd)
{
	struct device_node *np;

	if (!of_machine_is_compatible(sc57x_dt_board_compat[0]))
		goto restart_out;

	np = of_find_node_by_name(NULL, "softconfig_default");
	if (!np)
		goto restart_out;
	softconfig_of_set_group_active_pins_output(
						NULL, np, "reboot-pins", true);
	of_node_put(np);

restart_out:
	sc57x_restart(mode, cmd);
}

/**
 * Inactive all boot-pins from softconfig as listed in the board dts file
 */
static int __init sc57x_softconfig_init(void)
{
	int ret = 0;
	struct device_node *np;

	if (!of_machine_is_compatible(sc57x_dt_board_compat[0]))
		return -ENODEV;

	np = of_find_node_by_name(NULL, "softconfig_default");
	if (!np)
		return -ENODEV;
	ret = softconfig_of_set_group_active_pins_output(
						NULL, np, "boot-pins", false);
	of_node_put(np);

	return ret;
}
subsys_initcall_sync(sc57x_softconfig_init);

static const char * const sc57x_dt_board_compat[] __initconst = {
	"adi,sc57x",
	NULL
};

DT_MACHINE_START(SC57X_DT, "SC57x-EZKIT (Device Tree Support)")
	.map_io		= sc57x_map_io,
	.init_early	= sc57x_init_early,
	.init_time	= sc57x_timer_init,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
	.restart        = sc57x_ezkit_restart,
MACHINE_END
