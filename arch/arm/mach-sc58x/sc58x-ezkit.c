/*
 * machine start entries for ADI processor on-chip memory
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/sc58x.h>

#include "core.h"

#ifdef CONFIG_MACH_SC58X_DT
static const char * const sc58x_dt_board_compat[] __initconst = {
	"adi,sc58x",
	NULL
};

DT_MACHINE_START(SC58X_DT, "SC58x-EZKIT (Device Tree Support)")
	.map_io		= sc58x_map_io,
	.init_early	= sc58x_init_early,
	.init_time	= sc58x_timer_init,
	.init_machine	= sc58x_init,
	.dt_compat	= sc58x_dt_board_compat,
	.restart        = sc58x_restart,
MACHINE_END

#else
MACHINE_START(SC58X, "SC58x-EZKIT")
	.atag_offset	= 0x100,
	.map_io		= sc58x_map_io,
	.init_early	= sc58x_init_early,
	.init_irq	= sc58x_init_irq,
	.init_time	= sc58x_timer_init,
	.init_machine	= sc58x_init,
	.restart        = sc58x_restart,
MACHINE_END
#endif
