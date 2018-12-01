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
#include <mach/gpio.h>

#include "stmmac.h"
#include "stmmac_platform.h"

struct adi_dwmac {
	unsigned int enable_pin;
	unsigned int enable_pin_active_low;
};

static int dwmac_adi_probe(struct platform_device *pdev)
{
	struct adi_dwmac *dwmac;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	int ret;

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}

		if (likely(of_count_phandle_with_args(pdev->dev.of_node,
							"enable-pin", NULL) > 0)) {
			if (softconfig_of_set_active_pin_output(&pdev->dev,
						pdev->dev.of_node, "enable-pin", 0, &dwmac->enable_pin,
						&dwmac->enable_pin_active_low, true))
				return -ENODEV;
		}

	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	plat_dat->bsp_priv = dwmac;
	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int dwmac_adi_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct adi_dwmac *dwmac = (struct adi_dwmac *)priv->plat->bsp_priv;
	int ret = stmmac_dvr_remove(&pdev->dev);

	if (dwmac->enable_pin && gpio_is_valid(dwmac->enable_pin))
		gpio_direction_output(dwmac->enable_pin,
					    dwmac->enable_pin_active_low ? 1 : 0);

	return ret;
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
