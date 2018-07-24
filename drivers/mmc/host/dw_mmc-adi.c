/*
 * ADI Specific Extensions for Synopsys DW Multimedia Card Interface driver
 *
 * Copyright (c) 2014 - 2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <mach/portmux.h>
#include <mach/cpu.h>

#include "dw_mmc-pltfm.h"
#include "dw_mmc.h"

static int dwmmc_adi_priv_init(struct dw_mci *host)
{
	int spu_securep_id, ret = 0;

	struct device_node *node = host->dev->of_node;
	struct pinctrl *pctrl;
	struct pinctrl_state *pstate;

	/*initialize pin mux*/
	pctrl = devm_pinctrl_get(host->dev);

	pstate = pinctrl_lookup_state(pctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pstate))
		return ret;

	ret	= pinctrl_select_state(pctrl, pstate);
	if (ret)
		return ret;

	/*disable msi spu*/
	ret = of_property_read_u32(node, "spu_securep_id", &spu_securep_id);
	if (ret)
		return ret;
	set_spu_securep_msec(spu_securep_id, true);

	return 0;
}

static const struct dw_mci_drv_data adi_drv_data = {
	.init				= dwmmc_adi_priv_init,
};

static const struct of_device_id dwmmc_adi_match[] = {
	{ .compatible = "adi,mmc",
			.data = &adi_drv_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dwmmc_adi_match);

static int dwmmc_adi_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;

	match = of_match_node(dwmmc_adi_match, pdev->dev.of_node);

	if (!match)
		return -1;

	drv_data = match->data;

	return dw_mci_pltfm_register(pdev, drv_data);
}

static struct platform_driver dwmmc_adi_pltfm_driver = {
	.probe		= dwmmc_adi_probe,
	.remove		= dw_mci_pltfm_remove,
	.driver		= {
		.name			= "dwmmc_adi",
		.of_match_table = dwmmc_adi_match,
	},
};

module_platform_driver(dwmmc_adi_pltfm_driver);

MODULE_DESCRIPTION("ADI Specific DW-MSHC Driver Extension");
MODULE_AUTHOR("Hao Liang <hliang1025@gmail.com>");
MODULE_LICENSE("GPL v2");
