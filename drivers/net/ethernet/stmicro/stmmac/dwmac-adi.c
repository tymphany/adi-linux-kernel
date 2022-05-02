/*
 * dwmac-adi.c Analog Devices EMAC driver for sc5xx
 *
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include "stmmac.h"
#include "stmmac_platform.h"

static int dwmac_adi_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(&pdev->dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int dwmac_adi_remove(struct platform_device *pdev)
{
	return stmmac_dvr_remove(&pdev->dev);
}

static const struct of_device_id dwmac_adi_match[] = {
	{ .compatible = "adi,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_adi_match);

static struct platform_driver dwmac_adi_driver = {
	.probe  = dwmac_adi_probe,
	.remove = dwmac_adi_remove,
	.driver = {
		.name           = "adi-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_adi_match),
	},
};
module_platform_driver(dwmac_adi_driver);

MODULE_DESCRIPTION("EMAC driver for ADI SC5xx based boards");
MODULE_LICENSE("GPL v2");
