// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Driver for Analog Devices Reset Control Unit
 *
 * Copyright (c) 2021 Analog Devices
 * Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/types.h>

#define ADI_RCU_REBOOT_PRIORITY		255

#define ADI_RCU_REG_CONTROL					0x00
#define ADI_RCU_REG_STATUS					0x04
#define ADI_RCU_REG_CORE_RESET_CONTROL		0x08
#define ADI_RCU_REG_CORE_RESET_STATUS		0x0c

#define ADI_RCU_CONTROL_SYSRST		BIT(0)

struct adi_rcu {
	struct notifier_block reboot_notifier;
	void __iomem *ioaddr;
	struct device *dev;
};

static struct adi_rcu *to_adi_rcu(struct notifier_block *nb) {
	return container_of(nb, struct adi_rcu, reboot_notifier);
}

static int adi_rcu_reboot(struct notifier_block *nb, unsigned long mode, void *cmd)
{
	struct adi_rcu *adi_rcu = to_adi_rcu(nb);
	u32 val;

	dev_info(dev, "Reboot requested\n");

	val = readl(adi_rcu->ioaddr + ADI_RCU_REG_CONTROL);
	writel(val | ADI_RCU_CONTROL_SYSRST, adi_rcu->ioaddr);

	dev_err(adi_rcu->dev, "Unable to reboot via RCU\n");
	return NOTIFY_DONE;
}

static int adi_rcu_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct adi_rcu *adi_rcu = NULL;
	struct resource *res;
	void __iomem *base;
	int ret;

	adi_rcu = devm_kzalloc(dev, sizeof(*adi_rcu), GFP_KERNEL);
	if (!adi_rcu) {
		dev_err(dev, "Cannot allocate RCU memory\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, adi_rcu);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get RCU base address\n");
		return -ENODEV;
	}

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		dev_err(dev, "Cannot map RCU base address\n");
		return PTR_ERR(base);
	}

	adi_rcu->ioaddr = base;
	adi_rcu->dev = dev;
	adi_rcu->reboot_notifier.priority = ADI_RCU_REBOOT_PRIORITY;
	adi_rcu->reboot_notifier.notifier_call = adi_rcu_reboot;
	ret = register_restart_handler(&adi_rcu->reboot_notifier);
	if (ret) {
		dev_err(dev, "Unable to register restart handler: %d\n", ret);
		return ret;
	}

	return 0;
}

static int adi_rcu_remove(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct adi_rcu *adi_rcu = dev_get_drvdata(dev);

	unregister_restart_handler(&adi_rcu->reboot_notifier);
	return 0;
}

static const struct of_device_id adi_rcu_match[] = {
	{ .compatible = "adi,reset-controller" },
	{}
};
MODULE_DEVICE_TABLE(of, adi_rcu_match);

static struct platform_driver adi_rcu_driver = {
	.probe = adi_rcu_probe,
	.remove = adi_rcu_remove,
	.driver = {
		.name = "ADI Reset Control Unit",
		.of_match_table = of_match_ptr(adi_rcu_match),
	},
};
module_platform_driver(adi_rcu_driver);

MODULE_DESCRIPTION("Analog Devices RCU driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
