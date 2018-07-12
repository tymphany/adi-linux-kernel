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
#include <sound/sc5xx-dai.h>
#include <sound/sc5xx-sru.h>

#include "core.h"
#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT)
void sru_init(void)
{
	/* set DAI1_PIN02 to input */
	SRU(1, LOW, DAI1_PBEN02_I);
	/* route DAI1_PIN02 to SPT4_ACLK */
	SRU(1, DAI1_PB02_O, SPT4_ACLK_I);
	SRU(1, DAI1_PB02_O, SPT4_BCLK_I);
	/*set DAI1_PIN04 to input */
	SRU(1, LOW, DAI1_PBEN04_I);
	/* route DAI1_PIN04 to SPT4_AFS */
	SRU(1, DAI1_PB04_O, SPT4_AFS_I);
	SRU(1, DAI1_PB04_O, SPT4_BFS_I);
	/* set DAI1_PIN01 to output */
	SRU(1, HIGH, DAI1_PBEN01_I);
	/* route SPT4_AD0 to DAI1_PIN01 */
	SRU(1, SPT4_AD0_O, DAI1_PB01_I);

	/* set DAI1_PIN12 to input */
	SRU(1, LOW, DAI1_PBEN12_I);
	/* route DAI1_PIN12 to SPT4_BCLK */
	SRU(1, DAI1_PB12_O, SPT4_BCLK_I);
	/*set DAI1_PIN20 to input */
	SRU(1, LOW, DAI1_PBEN20_I);
	/* route DAI1_PIN20 to SPT4_BFS */
	SRU(1, DAI1_PB20_O, SPT4_BFS_I);
	/* set DAI1_PIN06 to input */
	SRU(1, LOW, DAI1_PBEN06_I);
	/* route DAI1_PIN06 to SPT4_BD0 */
	SRU(1, DAI1_PB06_O, SPT4_BD0_I);
}
EXPORT_SYMBOL(sru_init);
#endif
void pads_init(void)
{
	writel(0xffffffff, __io_address(REG_PADS0_DAI0_IE));
	writel(0xffffffff, __io_address(REG_PADS0_DAI1_IE));
}
EXPORT_SYMBOL(pads_init);

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
