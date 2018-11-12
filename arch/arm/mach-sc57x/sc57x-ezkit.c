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
#include <linux/of_gpio.h>
#include <mach/gpio.h>

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

/**
 * softconfig_of_set_active_pin_output()
 * @dev:	device that will be interacted with
 * @np:		device node to get pin property from
 * @propname:	property name of gpio list
 * @index:		index of the specified GPIO
 * @pin:		pin number pointer to fill in
 * @active_flag:    active flag pointer to fill in
 * @active:		if active as the gpio flag property in dts
 *
 * Set the state of the specified active pin for peripheral from softconfig
 * with a given gpio list property name, index of the specified GPIO and a
 * given node. If @dev is NULL, requested GPIO will be freed at the end of this
 * function, otherwise requested GPIO in this function will be automatically
 * freed on driver detach. It returns error if it fails otherwise 0 on success.
 */
int softconfig_of_set_active_pin_output(struct device *dev,
				struct device_node *np, const char *propname, int index,
				int *pin, int *active_flag, bool active)
{
	int ret = 0;
	int pin_num, flag_active_low, output;
	bool need_free = true;
	enum of_gpio_flags flag;

	pin_num = of_get_named_gpio_flags(np, propname, index, &flag);
	if (!gpio_is_valid(pin_num)) {
		pr_err("%s, invalid %s %d\n", __func__, propname, pin_num);
		return -ENODEV;
	}

	if (dev == NULL)
		ret = gpio_request(pin_num, propname);
	else {
		need_free = false;
		ret = devm_gpio_request(dev, pin_num, dev_name(dev));
	}
	if (ret == -EBUSY) {
		need_free = false;
		pr_debug("%s, %s %d is busy now!\n", __func__, propname, pin_num);
	} else if (ret < 0) {
		pr_err("%s, can't request %s %d, err: %d\n",
							__func__, propname, pin_num, ret);
		return ret;
	}

	flag_active_low = (flag & OF_GPIO_ACTIVE_LOW ? 1 : 0);

	if (active)
		output = flag_active_low ? 0 : 1;
	else
		output = flag_active_low ? 1 : 0;
	ret = gpio_direction_output(pin_num, output);
	if (ret < 0) {
		pr_err("%s, can't set direction for %s pin %d, err: %d\n",
							__func__, propname, pin_num, ret);
		goto out;
	}

	*pin = pin_num;
	*active_flag = flag_active_low;

out:
	if (need_free)
		gpio_free(pin_num);
	return ret;
}
EXPORT_SYMBOL(softconfig_of_set_active_pin_output);

/**
 * softconfig_of_set_group_active_pins_output()
 * @dev:	device that will be interacted with
 * @np:		device node to get pin property from
 * @propname:	property name of gpio list
 * @active:		if active as the gpio flag property in dts
 *
 * Set the state of the group active pins for peripheral from softconfig with
 * a given gpio list property name and a given node. If @dev is NULL, requested
 * GPIO will be freed at the end of this function, otherwise requested GPIO in
 * this function will be automatically freed on driver detach.
 * It returns error if it fails otherwise 0 on success.
 */
int softconfig_of_set_group_active_pins_output(struct device *dev,
				struct device_node *np, const char *propname, bool active)
{
	int ret = 0;
	int i, nb = 0;

	nb = of_gpio_named_count(np, propname);
	for (i = 0; i < nb; i++) {
		int pin, flag;
		ret = softconfig_of_set_active_pin_output(
						dev, np, propname, i, &pin, &flag, active);
		if (ret)
			return ret;
	}

	return ret;
}
EXPORT_SYMBOL(softconfig_of_set_group_active_pins_output);

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

DT_MACHINE_START(SC57X_DT, "SC57x-EZKIT (Device Tree Support)")
	.map_io		= sc57x_map_io,
	.init_early	= sc57x_init_early,
	.init_time	= sc57x_timer_init,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
	.restart        = sc57x_ezkit_restart,
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
