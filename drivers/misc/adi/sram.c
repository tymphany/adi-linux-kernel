/*
 * SRAM Controller driver for ADI processor on-chip memory
 *
 * Copyright 2014-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/genalloc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <mach/cpu.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <asm/io.h>
#ifdef CONFIG_ARCH_SC59X
#include <mach/sc59x.h>
#elif defined(CONFIG_ARCH_SC58X)
#include <mach/sc58x.h>
#elif defined(CONFIG_ARCH_SC57X)
#include <mach/sc57x.h>
#endif
#include <mach/sram.h>

#define SRAM_DRV_NAME		"sram_controller"

static struct device *sram_dev;
static void __iomem *l2_ctl_vaddr;

static irqreturn_t sram_ecc_err(int irq, void *dev_id)
{
	int status;

	pr_err("SRAM ECC error happened\n");
	status = readl(l2_ctl_vaddr + L2CTL0_STAT_OFFSET);
	pr_err("status 0x%x ctl %x\n", status, readl(l2_ctl_vaddr));

	if (status & 0x1)
		printk(KERN_ERR "Core channel error type:0x%x, addr:0x%x\n",
			readl(l2_ctl_vaddr + L2CTL0_ET0_OFFSET),
			readl(l2_ctl_vaddr + L2CTL0_EADDR0_OFFSET));
	if (status & 0x2)
		printk(KERN_ERR "System channel error type:0x%x, addr:0x%x\n",
			readl(l2_ctl_vaddr + L2CTL0_ET1_OFFSET),
			readl(l2_ctl_vaddr + L2CTL0_EADDR1_OFFSET));

	status = status >> 8;
	if (status)
		pr_err("SRAM Bank%d error, addr:0x%x\n", status,
			readl(l2_ctl_vaddr + L2CTL0_ERRADDR0_OFFSET + status));
	panic("Can't recover from the SRAM ECC error.");

	return IRQ_HANDLED;
}

static int adi_sram_show(struct seq_file *s, void *data)
{
	struct device_node *sram_node;
	struct gen_pool *sram_pool = NULL;
	size_t pool_size = 0, avail = 0, used = 0;
	int index, count = 0;
	const __be32 *sram_pbase;

	count = of_count_phandle_with_args(sram_dev->of_node, "adi,sram", NULL);
	if (!count) {
		pr_err("no adi,sram phandle defined in sram controller\n");
		return -ENODEV;
	}

	for (index = 0; index < count; index++) {
		/* Get the name and addr of the sram pool */
		sram_node = of_parse_phandle(sram_dev->of_node, "adi,sram", index);
		if (!sram_node) {
			pr_err("Unable to parse phandle\n");
			return -ENODEV;
		}

		sram_pbase = of_get_address(sram_node, 0, NULL, NULL);
		if (!sram_pbase) {
			pr_err("Unable to get phandle address\n");
			return -ENODEV;
		}
		seq_printf(s,"%s@%x:\n",
			sram_node->name, be32_to_cpu(*sram_pbase));

		/* Get the sram pool info */
		sram_pool = of_gen_pool_get(sram_dev->of_node, "adi,sram", index);
		if (!sram_pool) {
			pr_err("sram_pool not available\n");
			of_node_put(sram_node);
			return -ENOMEM;
		}

		/* Calculate the sram total/available/used size */
		pool_size = gen_pool_size(sram_pool);
		avail = gen_pool_avail(sram_pool);
		used = pool_size - avail;
		seq_printf(s,"\tTotal size: %d B\n\tUsed sram: %d B\n\tAvail sram: %d B\n",
			pool_size, used, avail);

		of_node_put(sram_node);
	}

	return 0;
}

static int adi_sram_open(struct inode *inode, struct file *file)
{
	return single_open(file, adi_sram_show, NULL);
}
static const struct file_operations adi_sram_fops = {
	.open 		= adi_sram_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct of_device_id adi_sram_of_match[] = {
	{ .compatible = "adi,sram-controller" },
	{ },
};
MODULE_DEVICE_TABLE(of, adi_sram_of_match);

static int adi_sram_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq;
	struct proc_dir_entry *d;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	const struct of_device_id *match;
	const __be32 *l2_ctl_phys;
	resource_size_t res_size = 0x100;

	sram_dev = &pdev->dev;

	/* create sram proc show data */
	d = proc_create("sraminfo", 0, NULL, &adi_sram_fops);
	if (!d)
		return -ENOMEM;

	/* sram-ecc-err */
	match = of_match_device(of_match_ptr(adi_sram_of_match), dev);
	if (!match) {
		dev_err(dev, "No sram-controller dts node specified\n");
		return -ENOENT;
	}
	child = of_get_child_by_name(pdev->dev.of_node, "sram-ecc-err");
	if (!child) {
		dev_err(dev, "No sram-ecc-err dts node specified\n");
		return -ENOENT;
	}

	irq = irq_of_parse_and_map(child, 0);
	if (!irq) {
		dev_err(dev, "invalid irq from dts node\n");
		return -ENOENT;
	}
	l2_ctl_phys = of_get_address(child, 0, NULL, NULL);
	l2_ctl_vaddr = devm_ioremap(dev, be32_to_cpu(*l2_ctl_phys), res_size);


	writel(readl(l2_ctl_vaddr + L2CTL0_STAT_OFFSET),
				(l2_ctl_vaddr + L2CTL0_STAT_OFFSET));

	ret = devm_request_irq(dev, irq, sram_ecc_err, 0,
			"sram-ecc-err", dev);
	if (unlikely(ret < 0)) {
		pr_err("Fail to request SRAM ECC error interrupt.ret:%d\n", ret);
	}

	return ret;
}

static int adi_sram_remove(struct platform_device *pdev)
{
	remove_proc_entry("sraminfo", NULL);
	return 0;
}

static struct platform_driver adi_sram_driver = {
	.probe = adi_sram_probe,
	.remove = adi_sram_remove,
	.driver = {
		.name	= SRAM_DRV_NAME,
		.of_match_table = of_match_ptr(adi_sram_of_match),
	},
};

module_platform_driver(adi_sram_driver);

MODULE_DESCRIPTION("ADI on-chip sram Controller Driver");
MODULE_LICENSE("GPL");
