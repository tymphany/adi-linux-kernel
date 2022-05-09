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
#include <mach/sc58x.h>
#include <sound/sc5xx-dai.h>
#include <sound/sc5xx-sru.h>
#include <linux/of_gpio.h>
#include <mach/gpio.h>

#include "core.h"

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT) || IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
void sru_init(void)
{
#if IS_ENABLED(CONFIG_MACH_SC589_MINI)
	SRU(0, HIGH, DAI0_PBEN05_I);
	SRU(0, LOW, DAI0_PB05_I);
	SRU(0, LOW, DAI0_PBEN06_I);

	/* Set up pins for ADAU1761 */
	/* set DAI0_PIN01 for ADAU1761 DAC data as an output */
	SRU(0, HIGH, DAI0_PBEN01_I);
	/* set DAI0_PIN02 for ADAU1761 ADC data as an input */
	SRU(0, LOW, DAI0_PBEN02_I);
	/* set DAI0_PIN03 for ADAU1761 CLK as an input */
	SRU(0, LOW, DAI0_PBEN03_I);
	/* set DAI0_PIN04 for ADAU1761 FS as an input */
	SRU(0, LOW, DAI0_PBEN04_I);

	/* Connect ADAU1761 signals to SPORT0 */
	/* route ADAU1761 ADC pin to SPT0 BD0 input */
	SRU(0, DAI0_PB02_O, SPT0_BD0_I);
	/* route SPT0A AD0 output to 1761 DAC pin */
	SRU(0, SPT0_AD0_O, DAI0_PB01_I);
	/* route ADAU1761 BCLK pin to SPORT0A clock input */
	SRU(0, DAI0_PB03_O, SPT0_ACLK_I);
	/* route ADAU1761 BCLK pin to SPORT0B clock input */
	SRU(0, DAI0_PB03_O, SPT0_BCLK_I);
	/* route ADAU1761 FS pin to SPORT0A frame sync */
	SRU(0, DAI0_PB04_O, SPT0_AFS_I);
	/* route ADAU1761 FS pin to SPORT0B frame sync */
	SRU(0, DAI0_PB04_O, SPT0_BFS_I);
#else
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
#endif
}
EXPORT_SYMBOL(sru_init);
#endif

void pads_init(void)
{
	writel(0xffffffff, __io_address(REG_PADS0_DAI0_IE));
	writel(0xffffffff, __io_address(REG_PADS0_DAI1_IE));
}
EXPORT_SYMBOL(pads_init);

static const char * const sc58x_dt_board_compat[] __initconst = {
	"adi,sc58x",
	NULL
};

static void sc58x_ezkit_restart(enum reboot_mode mode, const char *cmd)
{
       sc58x_restart(mode, cmd);
}

extern void __init adsp_sc5xx_timer_core_init(void);

DT_MACHINE_START(SC58X_DT, "SC58x-EZKIT (Device Tree Support)")
	.map_io		= sc58x_map_io,
	.init_early	= sc58x_init_early,
	.init_time	= adsp_sc5xx_timer_core_init,
	.init_machine	= sc58x_init,
	.dt_compat	= sc58x_dt_board_compat,
	.restart        = sc58x_ezkit_restart
MACHINE_END