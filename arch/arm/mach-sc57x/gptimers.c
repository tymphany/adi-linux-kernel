/*
 * gp timer driver for ADI processor on-chip memory
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <mach/sc57x.h>
#include <mach/cpu.h>

static LIST_HEAD(gptimers_list);
static DEFINE_SPINLOCK(gptimers_lock);

#define GPTIMER_OFFSET 0x20
#define GET_TIMER_BASE(id) ((struct gptimer3 *) \
	(__io_address(TIMER0_CONFIG) + GPTIMER_OFFSET * id))
#define GPTIMER_CFG_OFF   0x0
#define GPTIMER_COUNT_OFF 0x4
#define GPTIMER_PER_OFF   0x8
#define GPTIMER_WID_OFF   0xC
#define GPTIMER_DLY_OFF   0x10

static struct gptimer3_group_regs* const group_base = __io_address(TIMER_GROUP);
static struct gptimer3* const timer0_base = __io_address(TIMER0_CONFIG);

void set_gptimer_pwidth(struct sc57x_gptimer *timer, uint32_t value)
{
	writel(value, timer->io_base + GPTIMER_WID_OFF);
}
EXPORT_SYMBOL(set_gptimer_pwidth);

void set_gptimer_period(struct sc57x_gptimer *timer, uint32_t period)
{
	writel(period, timer->io_base + GPTIMER_PER_OFF);
}
EXPORT_SYMBOL(set_gptimer_period);

uint32_t get_gptimer_count(struct sc57x_gptimer *timer)
{
	return readl(timer->io_base + GPTIMER_COUNT_OFF);
}
EXPORT_SYMBOL(get_gptimer_count);

uint16_t get_gptimer_status(void)
{
	return readw(&group_base->data_ilat);
}
EXPORT_SYMBOL(get_gptimer_status);

void set_gptimer_status(uint16_t status)
{
	writew(status, &group_base->data_ilat);
}
EXPORT_SYMBOL(set_gptimer_status);

void set_gptimer_stopcfg(uint16_t mask)
{
	writew(mask, &group_base->stop_cfg_set);
}
EXPORT_SYMBOL(set_gptimer_stopcfg);

void set_gptimer_config(struct sc57x_gptimer *timer, uint16_t config)
{
	writew(config, timer->io_base + GPTIMER_CFG_OFF);
}
EXPORT_SYMBOL(set_gptimer_config);

void enable_gptimers(uint16_t mask)
{
	uint16_t imask;
	imask = readw(&group_base->data_imsk);
	imask &= ~mask;
	writew(imask, &group_base->data_imsk);
	writew(mask & 0xFF, &group_base->enable);
}
EXPORT_SYMBOL(enable_gptimers);

static void _disable_gptimers(uint16_t mask)
{
	writew(mask, &group_base->disable);
}

void disable_gptimers(uint16_t mask)
{
	set_gptimer_stopcfg(mask);
	_disable_gptimers(mask);
}
EXPORT_SYMBOL(disable_gptimers);

struct sc57x_gptimer *gptimer_request(int id)
{
	struct sc57x_gptimer *t = NULL;
	unsigned long flags;
	int found;

	spin_lock_irqsave(&gptimers_lock, flags);
	list_for_each_entry(t, &gptimers_list, node) {
		if (t->reserved)
			continue;
		if (id < 0) {
			found = 1;
			break;
		}

		if (id == t->id) {
			found = 1;
			break;
		}
	}

	spin_unlock_irqrestore(&gptimers_lock, flags);
	if (found)
		t->reserved = 1;

	return t;
}
EXPORT_SYMBOL(gptimer_request);

int gptimer_free(struct sc57x_gptimer *timer)
{
	if (!timer)
		return -EINVAL;

	WARN_ON(!timer->reserved);
	timer->reserved = 0;

	return 0;
}
EXPORT_SYMBOL(gptimer_free);

static int sc57x_gptimer_probe(struct platform_device *pdev)
{
	unsigned long flags;
	struct sc57x_gptimer *timer;
	int irq;
	struct resource *mem;
	struct device *dev = &pdev->dev;

	if (!dev->of_node) {
		dev_err(dev, "%s: no platform device.\n", __func__);
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "%s: no gptimer irq.\n", __func__);
		return -ENODEV;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "%s: no memory resource.\n", __func__);
		return -ENODEV;
	}

	timer = devm_kzalloc(dev, sizeof(struct sc57x_gptimer), GFP_KERNEL);
	if (!timer) {
		dev_err(dev, "%s: no memory.\n", __func__);
		return -ENOMEM;
	}

	timer->io_base = devm_ioremap_resource(dev, mem);
	if (IS_ERR(timer->io_base))
		return PTR_ERR(timer->io_base);
	timer->irq = irq;
	timer->pdev = pdev;

	timer->id = of_alias_get_id(dev->of_node, "timer");

	spin_lock_irqsave(&gptimers_lock, flags);
	list_add_tail(&timer->node, &gptimers_list);
	spin_unlock_irqrestore(&gptimers_lock, flags);

	return 0;
}

static int sc57x_gptimer_remove(struct platform_device *pdev)
{
	struct sc57x_gptimer *timer;
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&gptimers_lock, flags);
	list_for_each_entry(timer, &gptimers_list, node)
		if (!strcmp(dev_name(&timer->pdev->dev),
					dev_name(&pdev->dev))) {
			list_del(&timer->node);
			ret = 0;
			break;
		}
	spin_unlock_irqrestore(&gptimers_lock, flags);

	return ret;
}

static const struct of_device_id sc57x_gptimer_match[] = {
	{
		.compatible = "adi,sc57x-timer",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sc57x_gptimer_match);

static struct platform_driver sc57x_gptimer_driver = {
	.probe = sc57x_gptimer_probe,
	.remove = sc57x_gptimer_remove,
	.driver = {
		.name = "sc57x_gptimer",
		.of_match_table = of_match_ptr(sc57x_gptimer_match),
	},
};

static int __init sc57x_gptimer_init(void)
{
	return platform_driver_register(&sc57x_gptimer_driver);
}
arch_initcall(sc57x_gptimer_init);

static void __exit sc57x_gptimer_exit(void)
{
	platform_driver_unregister(&sc57x_gptimer_driver);
}
