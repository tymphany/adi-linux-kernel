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
#include <sound/sc5xx-dai.h>
#include <sound/sc5xx-sru.h>

#include "core.h"

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT)
void sru_init(void)
{
	/* set DAI0_PIN02 to input */
	SRU(0, LOW, DAI0_PBEN02_I);
	/* route DAI0_PIN02 to SPT0_ACLK */
	SRU(0, DAI0_PB02_O, SPT0_ACLK_I);
	SRU(0, DAI0_PB02_O, SPT0_BCLK_I);
	/*set DAI0_PIN04 to input */
	SRU(0, LOW, DAI0_PBEN04_I);
	/* route DAI0_PIN04 to SPT0_AFS */
	SRU(0, DAI0_PB04_O, SPT0_AFS_I);
	SRU(0, DAI0_PB04_O, SPT0_BFS_I);
	/* set DAI0_PIN01 to output */
	SRU(0, HIGH, DAI0_PBEN01_I);
	/* route SPT0_AD0 to DAI0_PIN01 */
	SRU(0, SPT0_AD0_O, DAI0_PB01_I);

	/* set DAI0_PIN12 to input */
	SRU(0, LOW, DAI0_PBEN12_I);
	/* route DAI0_PIN12 to SPT0_BCLK */
	SRU(0, DAI0_PB12_O, SPT0_BCLK_I);
	/*set DAI0_PIN20 to input */
	SRU(0, LOW, DAI0_PBEN20_I);
	/* route DAI0_PIN20 to SPT0_BFS */
	SRU(0, DAI0_PB20_O, SPT0_BFS_I);
	/* set DAI0_PIN06 to input */
	SRU(0, LOW, DAI0_PBEN06_I);
	/* route DAI0_PIN06 to SPT0_BD0 */
	SRU(0, DAI0_PB06_O, SPT0_BD0_I);
}
EXPORT_SYMBOL(sru_init);
#endif
void pads_init(void)
{
	/* nothing to be done here for dai in sc57x */
}
EXPORT_SYMBOL(pads_init);

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
