// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AD7879/AD7889 touchscreen (SPI bus)
 *
 * Copyright (C) 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/input.h>	/* BUS_SPI */
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "ad7879.h"

#define AD7879_DEVID		0x7A	/* AD7879/AD7889 */

#define MAX_SPI_FREQ_HZ      5000000

#define AD7879_CMD_MAGIC     0xE0
#define AD7879_CMD_READ      BIT(2)

static const struct regmap_config ad7879_spi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = 15,
	.read_flag_mask = AD7879_CMD_MAGIC | AD7879_CMD_READ,
	.write_flag_mask = AD7879_CMD_MAGIC,
};

#ifdef CONFIG_OF
#include <linux/gpio.h>
#include <linux/platform_data/ad7879.h>
static const struct of_device_id ad7879_spi_dt_ids[] = {
	{ .compatible = "adi,ad7879", },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7879_spi_dt_ids);
#endif

static int ad7879_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
#ifdef CONFIG_OF
	const struct of_device_id *match;
	int irq, ret = 0;

	match = of_match_device(ad7879_spi_dt_ids, &spi->dev);
	if (!match) {
		dev_err(&spi->dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(spi->dev.of_node, "gpio",&irq);
	if(ret)
	  return ret;
	spi->irq = gpio_to_irq(irq);
#endif

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	regmap = devm_regmap_init_spi(spi, &ad7879_spi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return ad7879_probe(&spi->dev, regmap, spi->irq, BUS_SPI, AD7879_DEVID);
}

static struct spi_driver ad7879_spi_driver = {
	.driver = {
		.name	= "ad7879",
		.pm	= &ad7879_pm_ops,
		.of_match_table = of_match_ptr(ad7879_spi_dt_ids),
	},
	.probe		= ad7879_spi_probe,
};

module_spi_driver(ad7879_spi_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("AD7879(-1) touchscreen SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ad7879");
