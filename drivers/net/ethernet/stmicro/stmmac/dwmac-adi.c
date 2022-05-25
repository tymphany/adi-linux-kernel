/*
 * dwmac-adi.c Analog Devices EMAC driver for sc5xx
 *
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adi_system_config.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define ADI_PHYISEL_MII 0
#define ADI_PHYISEL_RGMII 1
#define ADI_PHYISEL_RMII 2

static int dwmac_adi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct plat_stmmacenet_data *plat_dat;
	struct regmap *regmap;
	struct stmmac_resources stmmac_res;
	int ret;
	int emac_alias;
	u32 val;

	if (!of_property_read_bool(np, "adi,skip-phyconfig")) {

		emac_alias = of_alias_get_id(np, "ethernet");
		if (emac_alias < 0) {
			dev_err(&pdev->dev, "Failed to get EMAC alias id\n");
			return NULL;
		}

		regmap = system_config_regmap_lookup_by_phandle(np, "adi,system-config");
		if (IS_ERR(regmap)) {
			if (PTR_ERR(regmap) == -EPROBE_DEFER)
				return PTR_ERR(regmap);

			dev_err(&pdev->dev, "adi,system-config regmap not connected\n");
			return PTR_ERR(regmap);
		}

		if(emac_alias == 0){
			ret = of_get_phy_mode(np);
			if (ret < 0) {
				dev_err(dev, "phy-mode must be specified to configure interface\n");
				return ret;
			}

			val = 0;
			switch (ret) {
			case PHY_INTERFACE_MODE_MII:
				val = ADI_PHYISEL_MII;
				break;
			case PHY_INTERFACE_MODE_RMII:
				val = ADI_PHYISEL_RMII;
				break;
			case PHY_INTERFACE_MODE_RGMII:
			case PHY_INTERFACE_MODE_RGMII_ID:
			case PHY_INTERFACE_MODE_RGMII_RXID:
			case PHY_INTERFACE_MODE_RGMII_TXID:
				val = ADI_PHYISEL_RGMII;
				break;
			default:
				dev_err(dev, "Unsupported PHY interface mode %d selected\n", ret);
				return -EINVAL;
			}

			/* write config registers if available */
			regmap_write(regmap, ADI_SYSTEM_REG_EMAC0_EMACRESET, 0);
			regmap_write(regmap, ADI_SYSTEM_REG_EMAC0_PHYISEL, val);
			regmap_write(regmap, ADI_SYSTEM_REG_EMAC0_EMACRESET, 1);
			regmap_write(regmap, ADI_SYSTEM_REG_EMAC0_ENDIANNESS, 0);
		}else if(emac_alias == 1){
			regmap_write(regmap, ADI_SYSTEM_REG_EMAC1_ENDIANNESS, 0);
		}
	}

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	ret = stmmac_dvr_probe(dev, plat_dat, &stmmac_res);
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
