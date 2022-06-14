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

#include <asm/mach/arch.h>
#include <mach/sc57x.h>

#include "core.h"

void pads_init(void)
{
	/* nothing to be done here for dai in sc57x */
}
EXPORT_SYMBOL(pads_init);

static const char * const sc57x_dt_board_compat[] __initconst = {
	"adi,sc57x",
	NULL
};

// @todo this needs to be in a header and defined in one place, under soc/adi
extern void __init adsp_sc5xx_timer_core_init(void);

DT_MACHINE_START(SC57X_DT, "SC57x-EZKIT (Device Tree Support)")
	.map_io		= sc57x_map_io,
	.init_early	= sc57x_init_early,
	.init_time	= adsp_sc5xx_timer_core_init,,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
	.restart        = sc57x_restart,
MACHINE_END
