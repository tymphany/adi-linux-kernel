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

extern void __init adsp_sc5xx_timer_core_init(void);

DT_MACHINE_START(SC59X_DT, "SC59x-EZKIT (Device Tree Support)")
	.map_io		= sc59x_map_io,
	.init_time	= adsp_sc5xx_timer_core_init,
	.init_machine	= sc59x_init,
	.dt_compat	= sc59x_dt_board_compat,
MACHINE_END
