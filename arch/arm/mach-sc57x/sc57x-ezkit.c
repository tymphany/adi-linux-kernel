/*
 * machine start entries for ADI processor on-chip memory
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <asm/mach/arch.h>

#include "core.h"

static const char * const sc57x_dt_board_compat[] __initconst = {
	"adi,sc57x",
	NULL
};

DT_MACHINE_START(SC57X_DT, "SC57x-EZKIT (Device Tree Support)")
	.l2c_aux_val = 0,
	.l2c_aux_mask = ~0,
	.init_early	= sc57x_init_early,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
MACHINE_END
