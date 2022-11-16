// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI Specific Extensions for Synopsys DW Multimedia Card Interface driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
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
