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

#include <linux/soc/adi/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/sc59x.h>
#include <sound/sc5xx-dai.h>
#include <linux/of_gpio.h>
#include <mach/gpio.h>

#include "core.h"

static const char * const sc59x_dt_board_compat[] __initconst = {
	"adi,sc59x",
	NULL
};

DT_MACHINE_START(SC59X_DT, "SC59x-EZKIT (Device Tree Support)")
	.l2c_aux_val = 0,
	.l2c_aux_mask = ~0,
	.init_machine	= sc59x_init,
	.dt_compat	= sc59x_dt_board_compat,
MACHINE_END
