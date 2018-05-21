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

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/sc57x.h>

#include "core.h"

#ifdef CONFIG_MACH_SC57X_DT
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
	.restart        = sc57x_restart,
MACHINE_END

#else
MACHINE_START(SC57X, "SC57x-EZKIT")
	.atag_offset	= 0x100,
	.map_io		= sc57x_map_io,
	.init_early	= sc57x_init_early,
	.init_irq	= sc57x_init_irq,
	.init_time	= sc57x_timer_init,
	.init_machine	= sc57x_init,
	.restart        = sc57x_restart,
MACHINE_END
#endif
