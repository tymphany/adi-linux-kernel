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

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

static const struct of_device_id dwmmc_adi_match[] = {
	{ .compatible = "adi,mmc", },
	{},
};
MODULE_DEVICE_TABLE(of, dwmmc_adi_match);

static int dwmmc_adi_probe(struct platform_device *pdev)
{
	return dw_mci_pltfm_register(pdev, NULL);
}

static int dwmmc_adi_remove(struct platform_device *pdev)
{
	return dw_mci_pltfm_remove(pdev);
}

static struct platform_driver dwmmc_adi_pltfm_driver = {
	.probe		= dwmmc_adi_probe,
	.remove		= dwmmc_adi_remove,
	.driver		= {
		.name			= "dwmmc_adi",
		.of_match_table = dwmmc_adi_match,
	},
};

module_platform_driver(dwmmc_adi_pltfm_driver);

MODULE_DESCRIPTION("ADI Specific DW-MSHC Driver Extension");
MODULE_AUTHOR("Hao Liang <hliang1025@gmail.com>");
MODULE_LICENSE("GPL v2");
