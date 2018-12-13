/*
 * SRAM mmap misc driver for ADI processor on-chip memory
 *
 * Copyright 2014-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/genalloc.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <mach/cpu.h>
#include <mach/hardware.h>
#ifdef CONFIG_ARCH_SC58X
#include <mach/sc58x.h>
#elif defined(CONFIG_ARCH_SC57X)
#include <mach/sc57x.h>
#endif

#define SRAM_MMAP_DRV_NAME		"sram_mmap"

struct adi_sram_mmap {
	struct miscdevice miscdev;
	struct gen_pool *sram_pool;
};

struct mmap_private_data {
	struct gen_pool *pool;
	unsigned long vaddr;
};

static void mmap_open(struct vm_area_struct *vma)
{
	struct mmap_private_data *pdata = vma->vm_private_data;
	size_t sram_size = vma->vm_end - vma->vm_start;

	/* Alloc the virtual address from specific sram_pool */
	pdata->vaddr = gen_pool_alloc(pdata->pool, sram_size);
	if (!pdata->vaddr) {
		pr_err("Faied to alloc memory from sram pool!\n");
	}
}

static void mmap_close(struct vm_area_struct *vma)
{
	struct mmap_private_data *pdata = vma->vm_private_data;
	size_t sram_size = vma->vm_end - vma->vm_start;

	gen_pool_free(pdata->pool, pdata->vaddr, sram_size);
	kzfree(pdata);
}

struct vm_operations_struct sram_mmap_vm_ops = {
	.open =     mmap_open,
	.close =    mmap_close,
};

static int sram_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct adi_sram_mmap *sram = container_of(fp->private_data,
				struct adi_sram_mmap, miscdev);
	struct mmap_private_data *pdata;
	size_t sram_size = vma->vm_end - vma->vm_start;
	unsigned long paddr;
	int ret = 0;

	/*  Allocate private pdata */
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata) {
		pr_err("Unable to allocate private pdata\n");
		return -ENOMEM;
	}

	pdata->pool = sram->sram_pool;
	vma->vm_private_data = pdata;
	vma->vm_ops = &sram_mmap_vm_ops;
	vma->vm_ops->open(vma);

	if(!pdata->vaddr) {
		ret = -EAGAIN;
		goto out_free;
	}

	paddr = gen_pool_virt_to_phys(pdata->pool, pdata->vaddr);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start,
				__phys_to_pfn(paddr), sram_size,
				vma->vm_page_prot)) {
		pr_err("Unable to mmap sram\n");
		ret = -EAGAIN;
		goto out_free;
	}
	pr_info("sram mmaped 0x%lx : 0x%lx successfully!\n",
		paddr, paddr + sram_size);

	return 0;

out_free:
	kzfree(pdata);
	return ret;
}

static const struct file_operations sram_fops = {
	.mmap		= sram_mmap,
};

static const struct of_device_id adi_sram_mmap_of_match[] = {
	{ .compatible = "adi,sram-mmap" },
	{ },
};
MODULE_DEVICE_TABLE(of,adi_sram_mmap_of_match);

static int adi_sram_mmap_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct adi_sram_mmap *sram;
	const struct of_device_id *match;
	struct device *dev;

	dev = &pdev->dev;

	/* Allocate sram device data */
	sram = devm_kzalloc(dev, sizeof(*sram), GFP_KERNEL);
	if (!sram) {
		pr_err("Unable to allocate sram device data\n");
		return -ENOMEM;
	}

	match = of_match_device(of_match_ptr(adi_sram_mmap_of_match), &pdev->dev);
	if(!match) {
		pr_err("No sram mmap device defined in dts file\n");
		return -ENODEV;
	}
	sram->sram_pool = of_gen_pool_get(dev->of_node, "adi,sram", 0);
	if (!sram->sram_pool) {
		pr_err("Unable to get sram pool!\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, sram);

	sram->miscdev.minor = MISC_DYNAMIC_MINOR;
	sram->miscdev.name = SRAM_MMAP_DRV_NAME;
	sram->miscdev.fops = &sram_fops;
	sram->miscdev.parent = dev;

	ret = misc_register(&sram->miscdev);
	if (ret < 0)
		pr_err("Faied to register sram mmap misc device\n");

	return ret;
}

static int adi_sram_mmap_remove(struct platform_device *pdev)
{
	struct adi_sram_mmap *sram = dev_get_drvdata(&pdev->dev);
	misc_deregister(&sram->miscdev);

	return 0;
}

static struct platform_driver adi_sram_mmap_driver = {
	.probe = adi_sram_mmap_probe,
	.remove = adi_sram_mmap_remove,
	.driver = {
		.name = SRAM_MMAP_DRV_NAME,
		.of_match_table = of_match_ptr(adi_sram_mmap_of_match),
	},
};

module_platform_driver(adi_sram_mmap_driver);
MODULE_DESCRIPTION("SRAM mmap misc driver for ADI processor on-chip memory");
MODULE_LICENSE("GPL");
