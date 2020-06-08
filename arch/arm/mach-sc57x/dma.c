/*
 * dma.c - SC57x DMA implementation
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/vmalloc.h>

#include <mach/hardware.h>
#include <mach/sc57x.h>
#include <mach/cpu.h>
#include <mach/dma.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

struct dma_channel dma_ch[MAX_DMA_CHANNELS];
EXPORT_SYMBOL(dma_ch);

int channel2irq(unsigned int channel)
{
	if (channel >= MAX_DMA_CHANNELS)
		return -ENOENT;

	return dma_ch[channel].irq;
}

#ifdef CONFIG_PROC_FS
static int proc_dma_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < MAX_DMA_CHANNELS; ++i)
		if (dma_channel_active(i))
			seq_printf(m, "%2d: %s\n", i, dma_ch[i].device_id);

	return 0;
}

static int proc_dma_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_dma_show, NULL);
}

static const struct file_operations proc_dma_operations = {
	.open		= proc_dma_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_dma_init(void)
{
	proc_create("dma", 0, NULL, &proc_dma_operations);
	return 0;
}
late_initcall(proc_dma_init);
#endif

/**
 *	request_dma - request a DMA channel
 *
 * Request the specific DMA channel from the system if it's available.
 */
int request_dma(unsigned int channel, const char *device_id)
{
	pr_debug("request_dma() : BEGIN\n");

	if (!dma_ch[channel].regs) {
		pr_warn("DMA channel %d is not registed\n", channel);
		return -ENXIO;
	}

	if (device_id == NULL)
		pr_warn("request_dma(%u): no device_id given\n", channel);

	if (atomic_cmpxchg(&dma_ch[channel].chan_status, 0, 1)) {
		pr_debug("DMA CHANNEL IN USE\n");
		return -EBUSY;
	}

	dma_ch[channel].device_id = device_id;
	dma_ch[channel].callback = 0;
	set_spu_securep_msec(dma_ch[channel].spu_securep_id, true);

	/* This is to be enabled by putting a restriction -
	 * you have to request DMA, before doing any operations on
	 * descriptor/channel
	 */
	pr_debug("request_dma() : END\n");
	return 0;
}
EXPORT_SYMBOL(request_dma);

int set_dma_callback(unsigned int channel, irq_handler_t callback, void *data)
{
	int ret;
	unsigned int irq;

	BUG_ON(channel >= MAX_DMA_CHANNELS || !callback ||
			!atomic_read(&dma_ch[channel].chan_status));

	irq = dma_ch[channel].irq;
	ret = request_irq(irq, callback, 0, dma_ch[channel].device_id, data);
	if (ret)
		return ret;

	dma_ch[channel].data = data;
	dma_ch[channel].callback = callback;

	return 0;
}
EXPORT_SYMBOL(set_dma_callback);

/**
 *	clear_dma_buffer - clear DMA fifos for specified channel
 *
 * Set the Buffer Clear bit in the Configuration register of specific DMA
 * channel. This will stop the descriptor based DMA operation.
 */
static void clear_dma_buffer(unsigned int chan)
{
	void *cfg = &dma_ch[chan].regs->cfg;

	iowrite32(ioread32(cfg) | RESTART, cfg);
	iowrite32(ioread32(cfg) & ~RESTART, cfg);
}

void free_dma(unsigned int channel)
{
	pr_debug("freedma() : BEGIN\n");
	BUG_ON(channel >= MAX_DMA_CHANNELS ||
			!atomic_read(&dma_ch[channel].chan_status));

	/* Halt the DMA */
	disable_dma(channel);
	clear_dma_buffer(channel);

	if (dma_ch[channel].callback) {
		free_irq(dma_ch[channel].irq, dma_ch[channel].data);
		dma_ch[channel].callback = 0;
	}

	/* Clear the DMA Variable in the Channel */
	atomic_set(&dma_ch[channel].chan_status, 0);

	set_spu_securep_msec(dma_ch[channel].spu_securep_id, false);

	pr_debug("freedma() : END\n");
}
EXPORT_SYMBOL(free_dma);
#if 0
void __init early_dma_memcpy(void *pdst, const void *psrc, size_t size)
{
	unsigned long dst = __pa(pdst);
	unsigned long src = __pa(psrc);
	struct dma_register *dst_ch, *src_ch;

	/* We assume that everything is 4 byte aligned, so include
	 * a basic sanity check
	 */
	BUG_ON(dst % 4);
	BUG_ON(src % 4);
	BUG_ON(size % 4);

	dst_ch = ioremap(MDMA_D0_NEXT_DESC_PTR, 0x7F);
	if (!dst_ch) {
		pr_err("Cannot map MEM DEST DMA0 IO range\n");
		return;
	}
	src_ch = ioremap(MDMA_S0_NEXT_DESC_PTR, 0x7F);
	if (!src_ch) {
		pr_err("Cannot map MEM SRC DMA0 IO range\n");
		iounmap(dst_ch);
		return;
	}

	if (ioread32(&src_ch->cfg)) {
		while (!(ioread32(&dst_ch->irq_status) & DMA_DONE))
			continue;
		iowrite32(0, &src_ch->cfg);
	}

	dmac_map_area(psrc, size, DMA_TO_DEVICE);
	dmac_map_area((const void *)pdst, size, DMA_FROM_DEVICE);

	/* Destination */
	iowrite32(dst, &dst_ch->start_addr);
	iowrite32(size >> 2, &dst_ch->x_count);
	iowrite32(1 << 2, &dst_ch->x_modify);
	iowrite32(DMA_DONE | DMA_ERR, &dst_ch->irq_status);

	/* Source */
	iowrite32(src, &src_ch->start_addr);
	iowrite32(size >> 2, &src_ch->x_count);
	iowrite32(1 << 2, &src_ch->x_modify);
	iowrite32(DMA_DONE | DMA_ERR, &src_ch->irq_status);

	/* Enable */
	iowrite32(DMAEN | WDSIZE_32, &src_ch->cfg);
	iowrite32(WNR | DI_EN_X | DMAEN | WDSIZE_32, &dst_ch->cfg);

	while (!(ioread32(&dst_ch->irq_status) & DMA_DONE))
		continue;

	iowrite32(0, &src_ch->cfg);
	iowrite32(0, &dst_ch->cfg);

	dmac_unmap_area(psrc, size, DMA_TO_DEVICE);
	dmac_unmap_area((const void *)pdst, size, DMA_FROM_DEVICE);

	iounmap(src_ch);
	iounmap(dst_ch);
}
#endif

#if defined(CH_MEM_STREAM2_SRC)
#define CH_MEMCPY_SRC		CH_MEM_STREAM2_SRC
#define CH_MEMCPY_DEST		CH_MEM_STREAM2_DEST
#else
#define CH_MEMCPY_SRC		CH_MEM_STREAM0_SRC
#define CH_MEMCPY_DEST		CH_MEM_STREAM0_DEST
#endif

/**
 *	__dma_memcpy - program the MDMA registers
 *
 * Actually program MDMA0 and wait for the transfer to finish.  Disable IRQs
 * while programming registers so that everything is fully configured.  Wait
 * for DMA to finish with IRQs enabled.  If interrupted, the initial DMA_DONE
 * check will make sure we don't clobber any existing transfer.
 */
static void __dma_memcpy(u32 daddr, s16 dmod, u32 saddr, s16 smod, size_t cnt, u32 conf)
{
	static DEFINE_SPINLOCK(mdma_lock);
	unsigned long flags;

	spin_lock_irqsave(&mdma_lock, flags);

	if (get_dma_config(CH_MEMCPY_DEST))
		while (get_dma_curr_irqstat(CH_MEMCPY_DEST) & DMA_RUN_MASK ||
			get_dma_curr_irqstat(CH_MEMCPY_SRC) & DMA_RUN_MASK)
			continue;

	if (conf & DMA2D) {
		/* For larger bit sizes, we've already divided down cnt so it
		 * is no longer a multiple of 64k.  So we have to break down
		 * the limit here so it is a multiple of the incoming size.
		 * There is no limitation here in terms of total size other
		 * than the hardware though as the bits lost in the shift are
		 * made up by MODIFY (== we can hit the whole address space).
		 * X: (2^(16 - 0)) * 1 == (2^(16 - 1)) * 2 == (2^(16 - 2)) * 4
		 */
		u32 shift = abs(dmod) >> 1;
		size_t ycnt = cnt >> (16 - shift);
		cnt = 1 << (16 - shift);
		set_dma_y_count(CH_MEMCPY_DEST, ycnt);
		set_dma_y_count(CH_MEMCPY_SRC, ycnt);
		set_dma_y_modify(CH_MEMCPY_DEST, dmod);
		set_dma_y_modify(CH_MEMCPY_SRC, smod);
	}

	set_dma_start_addr(CH_MEMCPY_DEST, daddr);
	set_dma_x_count(CH_MEMCPY_DEST, cnt);
	set_dma_x_modify(CH_MEMCPY_DEST, dmod);
	clear_dma_irqstat(CH_MEMCPY_DEST);

	set_dma_start_addr(CH_MEMCPY_SRC, saddr);
	set_dma_x_count(CH_MEMCPY_SRC, cnt);
	set_dma_x_modify(CH_MEMCPY_SRC, smod);
	clear_dma_irqstat(CH_MEMCPY_SRC);

	set_dma_config(CH_MEMCPY_SRC, DMAEN | conf);
	if (conf & DMA2D)
		set_dma_config(CH_MEMCPY_DEST, WNR | DI_EN_Y | DMAEN | conf);
	else
		set_dma_config(CH_MEMCPY_DEST, WNR | DI_EN_X | DMAEN | conf);

	spin_unlock_irqrestore(&mdma_lock, flags);

	while (!(get_dma_curr_irqstat(CH_MEMCPY_DEST) & DMA_DONE))
		if (get_dma_config(CH_MEMCPY_SRC))
			continue;
		else
			return;

	clear_dma_irqstat(CH_MEMCPY_DEST);

	set_dma_config(CH_MEMCPY_SRC, 0);
	set_dma_config(CH_MEMCPY_DEST, 0);
}

extern struct static_vm *find_static_vm_vaddr(void *vaddr);

/**
 *	_dma_memcpy - translate C memcpy settings into MDMA settings
 *
 * Handle all the high level steps before we touch the MDMA registers.  So
 * handle direction, tweaking of sizes, and formatting of addresses.
 */
static dma_addr_t _dma_memcpy(dma_addr_t pdst, dma_addr_t psrc, size_t size)
{
	u32 conf, shift;
	s16 mod;
	dma_addr_t dst = pdst;
	dma_addr_t src = psrc;

	if (size == 0)
		return (dma_addr_t)NULL;

	if (dst % 4 == 0 && src % 4 == 0 && size % 4 == 0) {
		conf = WDSIZE_32;
		shift = 2;
	} else if (dst % 2 == 0 && src % 2 == 0 && size % 2 == 0) {
		conf = WDSIZE_16;
		shift = 1;
	} else {
		conf = WDSIZE_8;
		shift = 0;
	}

	/* If the two memory regions have a chance of overlapping, make
	 * sure the memcpy still works as expected.  Do this by having the
	 * copy run backwards instead.
	 */
	mod = 1 << shift;
	if (src < dst) {
		mod *= -1;
		dst += size + mod;
		src += size + mod;
	}
	size >>= shift;

	__dma_memcpy(dst, mod, src, mod, size, conf);

	return dst;
}

/**
 * dma_memcpy -	DMA memcpy between two DMA/bus adresses
 * @pdst: destn address
 * @psrc: src address
 * @size: DMA transfer len
 *
 * Perform a DMA memcpy between two DMA/bus addresses.
 */
dma_addr_t dma_memcpy(dma_addr_t pdst, const dma_addr_t psrc, size_t size)
{
	return _dma_memcpy(pdst, psrc, size);
}
EXPORT_SYMBOL(dma_memcpy);

/**
 *	safe_dma_memcpy - DMA memcpy w/argument checking
 *
 * Verify arguments are safe before heading to dma_memcpy().
 */
dma_addr_t safe_dma_memcpy(dma_addr_t dst, const dma_addr_t src, size_t size)
{
	if (!access_ok(VERIFY_WRITE, dst, size))
		return (dma_addr_t)NULL;
	if (!access_ok(VERIFY_READ, src, size))
		return (dma_addr_t)NULL;
	return dma_memcpy(dst, src, size);
}
EXPORT_SYMBOL(safe_dma_memcpy);

#ifdef CONFIG_OF
static const struct of_device_id adi_dma_of_match[] = {
	{
		.compatible = "adi,dma2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_dma_of_match);
#endif

static int adi_dma_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	struct dma_channel *dma = NULL;
	int spu_securep_id, ret = 0;
	char *id_str;
	unsigned long id;

	id_str = strchr(node->full_name, '@');
	if (id_str != NULL && (++id_str) != NULL)
		ret = kstrtoul(id_str, 10, &id);

	if (ret != 0 || id < 0 || id >= MAX_DMA_CHANNELS) {
		dev_err(&pdev->dev, "Invalid ADI DMA channel.\n");
		return -ENOENT;
	}

	dma = &dma_ch[id];

	atomic_set(&dma->chan_status, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_error;
	}

	dma->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dma->regs)) {
		dev_err(&pdev->dev, "Fail to remap io resources\n");
		ret = PTR_ERR(dma->regs);
		goto out_error;
	}

	dma->irq = platform_get_irq(pdev, 0);
	if (dma->irq == 0) {
		dev_err(&pdev->dev, "No ADI DMA IRQ specified\n");
		ret = -ENOENT;
		goto out_error;
	}

	match = of_match_device(of_match_ptr(adi_dma_of_match), &pdev->dev);
	if (match && !of_property_read_u32(node, "spu_securep_id",
		&spu_securep_id))
		dma->spu_securep_id = spu_securep_id;

	return 0;

out_error:
	return ret;
}

static struct platform_driver adi_dma_driver = {
	.probe          = adi_dma_probe,
	.driver         = {
		.name   = "adi-dma2",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(adi_dma_of_match),
	},
};

static int __init adi_dma_init(void)
{
	int ret;

	pr_info("ADI DMA2 Controller\n");

	ret = platform_driver_register(&adi_dma_driver);
	if (ret) {
		pr_err("fail to register adi-dma2\n");
		return ret;
	}

	/* Mark MEMDMA Channel as requested since we're using it internally */
	request_dma(CH_MEMCPY_DEST, "SC57x dma_memcpy");
	request_dma(CH_MEMCPY_SRC, "SC57x dma_memcpy");

	return 0;
}
arch_initcall(adi_dma_init);
