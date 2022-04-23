// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADSP PORT gpio driver.
 *
 * Copyright (C) 2022, Analog Devices, Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adsp-gpio-port.h>

static int adsp_gpio_direction_input(struct gpio_chip *chip, unsigned offset) {
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);
	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_CLEAR);
	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_SET);
	return 0;
}

static int adsp_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_CLEAR);

	if (value)
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_SET);
	else
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_CLEAR);

	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_SET);
	return 0;
}

static void adsp_gpio_set_value(struct gpio_chip *chip, unsigned offset, int value) {
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	if (value)
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_SET);
	else
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_CLEAR);
}

static int adsp_gpio_get_value(struct gpio_chip *chip, unsigned offset) {
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	return !!(__adsp_gpio_readw(port, ADSP_PORT_REG_DATA) & BIT(offset));
}

static int adsp_gpio_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct adsp_gpio_port *gpio;
	struct resource *res;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(gpio->regs))
		return PTR_ERR(gpio->regs);

	platform_set_drvdata(pdev, gpio);

	spin_lock_init(&gpio->lock);

	gpio->gpio.label = "adsp-gpio";
	gpio->gpio.direction_input = adsp_gpio_direction_input;
	gpio->gpio.direction_output = adsp_gpio_direction_output;
	gpio->gpio.get = adsp_gpio_get_value;
	gpio->gpio.set = adsp_gpio_set_value;
	gpio->gpio.request = gpiochip_generic_request;
	gpio->gpio.free = gpiochip_generic_free;
	gpio->gpio.of_node = dev->of_node;
	gpio->gpio.base = -1;
	gpio->gpio.ngpio = ADSP_PORT_NGPIO;

	ret = devm_gpiochip_add_data(dev, &gpio->gpio, gpio);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id adsp_gpio_of_match[] = {
	{ .compatible = "adi,adsp-port-gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, adsp_gpio_of_match);

static struct platform_driver adsp_gpio_driver = {
	.driver = {
		.name = "adsp-port-gpio",
		.of_match_table = adsp_gpio_of_match,
	},
	.probe = adsp_gpio_probe,
};

static int __init adsp_gpio_init(void) {
	return platform_driver_register(&adsp_gpio_driver);
}

arch_initcall(adsp_gpio_init);
