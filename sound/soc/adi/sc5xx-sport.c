/*
 * sc5xx_sport.c Analog Devices SC5XX SPORT driver
 *
 * Copyright (c) 2015-2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/cpu.h>
#include <mach/dma.h>
#include <mach/portmux.h>
#include <sound/sc5xx-sru.h>
#include <sound/sc5xx-dai.h>

#include "sc5xx-sport.h"

int sport_set_tx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI)
		return -EBUSY;
	iowrite32(params->spctl | SPORT_CTL_SPTRAN, &sport->tx_regs->spctl);
	iowrite32(params->div, &sport->tx_regs->div);
	iowrite32(params->spmctl, &sport->tx_regs->spmctl);
	iowrite32(params->spcs0, &sport->tx_regs->spcs0);
	return 0;
}
EXPORT_SYMBOL(sport_set_tx_params);

int sport_set_rx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->rx_regs->spctl) & SPORT_CTL_SPENPRI)
		return -EBUSY;
	iowrite32(params->spctl & ~SPORT_CTL_SPTRAN, &sport->rx_regs->spctl);
	iowrite32(params->div, &sport->rx_regs->div);
	iowrite32(params->spmctl, &sport->rx_regs->spmctl);
	iowrite32(params->spcs0, &sport->rx_regs->spcs0);
	return 0;
}
EXPORT_SYMBOL(sport_set_rx_params);

static int compute_wdsize(size_t wdsize)
{
	switch (wdsize) {
	case 1:
		return WDSIZE_8 | PSIZE_8;
	case 2:
		return WDSIZE_16 | PSIZE_16;
	default:
		return WDSIZE_32 | PSIZE_32;
	}
}

void sport_tx_start(struct sport_device *sport)
{
	set_dma_next_desc_addr(sport->tx_dma_chan,
			(void *)sport->tx_desc_phy);
	set_dma_config(sport->tx_dma_chan, DMAFLOW_LIST | DI_EN
			| compute_wdsize(sport->wdsize) | NDSIZE_6);
	enable_dma(sport->tx_dma_chan);
	iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
}
EXPORT_SYMBOL(sport_tx_start);

void sport_rx_start(struct sport_device *sport)
{
	set_dma_next_desc_addr(sport->rx_dma_chan,
			(void *)sport->rx_desc_phy);
	set_dma_config(sport->rx_dma_chan, DMAFLOW_LIST | DI_EN | WNR
			| compute_wdsize(sport->wdsize) | NDSIZE_6);
	enable_dma(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
}
EXPORT_SYMBOL(sport_rx_start);

void sport_tx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	disable_dma(sport->tx_dma_chan);
}
EXPORT_SYMBOL(sport_tx_stop);

void sport_rx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	disable_dma(sport->rx_dma_chan);
}
EXPORT_SYMBOL(sport_rx_stop);

void sport_set_tx_callback(struct sport_device *sport,
		void (*tx_callback)(void *), void *tx_data)
{
	sport->tx_callback = tx_callback;
	sport->tx_data = tx_data;
}
EXPORT_SYMBOL(sport_set_tx_callback);

void sport_set_rx_callback(struct sport_device *sport,
		void (*rx_callback)(void *), void *rx_data)
{
	sport->rx_callback = rx_callback;
	sport->rx_data = rx_data;
}
EXPORT_SYMBOL(sport_set_rx_callback);

static void setup_desc(struct sport_device *sport, int fragcount,
		size_t fragsize, unsigned int cfg, int tx)
{
	struct dmasg *desc;
	unsigned long desc_phy;
	void *buf;
	int i;

	if (tx) {
		desc = sport->tx_desc;
		desc_phy = (unsigned long)sport->tx_desc_phy;
		buf = sport->tx_buf;
	} else {
		desc = sport->rx_desc;
		desc_phy = (unsigned long)sport->rx_desc_phy;
		buf = sport->rx_buf;
	}

	for (i = 0; i < fragcount; ++i) {
		desc[i].next_desc_addr  = (void *)(desc_phy
				+ (i + 1) * sizeof(struct dmasg));
		desc[i].start_addr = (unsigned long)buf + i * fragsize;
		desc[i].cfg = cfg;
		desc[i].x_count = fragsize / sport->wdsize;
		desc[i].x_modify = sport->wdsize;
		desc[i].y_count = 0;
		desc[i].y_modify = 0;
	}

	/* make circular */
	desc[fragcount-1].next_desc_addr = (void *)desc_phy;
}

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize)
{
	unsigned int cfg;

	if (sport->tx_desc)
		dma_free_coherent(NULL, sport->tx_desc_size,
				sport->tx_desc, sport->tx_desc_phy);

	sport->tx_desc = dma_alloc_coherent(NULL,
			fragcount * sizeof(struct dmasg),
			&sport->tx_desc_phy, 0);
	sport->tx_desc_size = fragcount * sizeof(struct dmasg);
	if (!sport->tx_desc)
		return -ENOMEM;

	sport->tx_buf = buf;
	sport->tx_fragsize = fragsize;
	sport->tx_frags = fragcount;
	cfg = DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize)
		| NDSIZE_6 | DMAEN;

	setup_desc(sport, fragcount, fragsize, cfg, 1);

	return 0;
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize)
{
	unsigned int cfg;

	if (sport->rx_desc)
		dma_free_coherent(NULL, sport->rx_desc_size,
				sport->rx_desc, sport->rx_desc_phy);

	sport->rx_desc = dma_alloc_coherent(NULL,
			fragcount * sizeof(struct dmasg),
			&sport->rx_desc_phy, 0);
	sport->rx_desc_size = fragcount * sizeof(struct dmasg);
	if (!sport->rx_desc)
		return -ENOMEM;

	sport->rx_buf = buf;
	sport->rx_fragsize = fragsize;
	sport->rx_frags = fragcount;
	cfg = DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize)
		| WNR | NDSIZE_6 | DMAEN;

	setup_desc(sport, fragcount, fragsize, cfg, 0);

	return 0;
}
EXPORT_SYMBOL(sport_config_rx_dma);

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	unsigned long curr = get_dma_curr_addr(sport->tx_dma_chan);

	return (unsigned char *)curr - sport->tx_buf;
}
EXPORT_SYMBOL(sport_curr_offset_tx);

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	unsigned long curr = get_dma_curr_addr(sport->rx_dma_chan);

	return (unsigned char *)curr - sport->rx_buf;
}
EXPORT_SYMBOL(sport_curr_offset_rx);

static irqreturn_t sport_tx_irq(int irq, void *dev_id)
{
	struct sport_device *sport = dev_id;
	static unsigned long status;

	status = get_dma_curr_irqstat(sport->tx_dma_chan);
	if (status & (DMA_DONE|DMA_ERR))
		clear_dma_irqstat(sport->tx_dma_chan);

	if (sport->tx_callback)
		sport->tx_callback(sport->tx_data);
	return IRQ_HANDLED;
}

static irqreturn_t sport_rx_irq(int irq, void *dev_id)
{
	struct sport_device *sport = dev_id;
	unsigned long status;

	status = get_dma_curr_irqstat(sport->rx_dma_chan);
	if (status & (DMA_DONE|DMA_ERR))
		clear_dma_irqstat(sport->rx_dma_chan);

	if (sport->rx_callback)
		sport->rx_callback(sport->rx_data);
	return IRQ_HANDLED;
}

static irqreturn_t sport_err_irq(int irq, void *dev_id)
{
	struct sport_device *sport = dev_id;
	struct device *dev = &sport->pdev->dev;

	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_DERRPRI)
		dev_err(dev, "sport error: TUVF\n");
	if (ioread32(&sport->rx_regs->spctl) & SPORT_CTL_DERRPRI)
		dev_err(dev, "sport error: ROVF\n");

	return IRQ_HANDLED;
}

static int sport_get_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	if (!dev->of_node) {
		dev_err(dev, "No device tree node\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No tx MEM resource\n");
		return -ENODEV;
	}
	sport->tx_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(sport->tx_regs)) {
		dev_err(dev, "Failed to map tx registers\n");
		return PTR_ERR(sport->tx_regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "No rx MEM resource\n");
		return -ENODEV;
	}
	sport->rx_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(sport->rx_regs)) {
		dev_err(dev, "Failed to map rx registers\n");
		return PTR_ERR(sport->rx_regs);
	}

	ret = of_property_read_u32_index(dev->of_node,
			"dma-channel", 0, &sport->tx_dma_chan);
	if (ret) {
		dev_err(dev, "No tx DMA resource\n");
		return -ENODEV;
	}

	ret = of_property_read_u32_index(dev->of_node,
			"dma-channel", 1, &sport->rx_dma_chan);

	if (ret) {
		dev_err(dev, "No rx DMA resource\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "No tx error irq resource\n");
		return -ENODEV;
	}
	sport->tx_err_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(dev, "No rx error irq resource\n");
		return -ENODEV;
	}
	sport->rx_err_irq = res->start;

	return 0;
}

static int sport_request_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	ret = request_dma(sport->tx_dma_chan, "SPORT TX Data");
	if (ret) {
		dev_err(dev, "Unable to allocate DMA channel for sport tx\n");
		return ret;
	}
	set_dma_callback(sport->tx_dma_chan, sport_tx_irq, sport);

	ret = request_dma(sport->rx_dma_chan, "SPORT RX Data");
	if (ret) {
		dev_err(dev, "Unable to allocate DMA channel for sport rx\n");
		goto err_rx_dma;
	}
	set_dma_callback(sport->rx_dma_chan, sport_rx_irq, sport);

	ret = request_irq(sport->tx_err_irq, sport_err_irq,
			0, "SPORT TX ERROR", sport);
	if (ret) {
		dev_err(dev, "Unable to allocate tx error IRQ for sport\n");
		goto err_tx_irq;
	}

	ret = request_irq(sport->rx_err_irq, sport_err_irq,
			0, "SPORT RX ERROR", sport);
	if (ret) {
		dev_err(dev, "Unable to allocate rx error IRQ for sport\n");
		goto err_rx_irq;
	}

	return 0;
err_rx_irq:
	free_irq(sport->tx_err_irq, sport);
err_tx_irq:
	free_dma(sport->rx_dma_chan);
err_rx_dma:
	free_dma(sport->tx_dma_chan);
	return ret;
}

static void sport_free_resource(struct sport_device *sport)
{
	free_irq(sport->rx_err_irq, sport);
	free_irq(sport->tx_err_irq, sport);
	free_dma(sport->rx_dma_chan);
	free_dma(sport->tx_dma_chan);
}

struct sport_device *sport_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sport_device *sport;
	int ret;

	pads_init();
	sru_init();

	sport = kzalloc(sizeof(*sport), GFP_KERNEL);
	if (!sport) {
		dev_err(dev, "Unable to allocate memory for sport device\n");
		return NULL;
	}
	sport->pdev = pdev;

	ret = sport_get_resource(sport);
	if (ret)
	  goto err_free_data;

	ret = sport_request_resource(sport);
	if (ret)
	  goto err_free_data;

	dev_info(dev, "SPORT create success\n");
	return sport;

err_free_data:
	kfree(sport);
	return NULL;
}
EXPORT_SYMBOL(sport_create);

void sport_delete(struct sport_device *sport)
{
	if (sport->tx_desc)
		dma_free_coherent(NULL, sport->tx_desc_size,
				sport->tx_desc, sport->tx_desc_phy);
	if (sport->rx_desc)
		dma_free_coherent(NULL, sport->rx_desc_size,
				sport->rx_desc, sport->rx_desc_phy);
	sport_free_resource(sport);
	kfree(sport);
}
EXPORT_SYMBOL(sport_delete);

MODULE_DESCRIPTION("Analog Devices SC5XX SPORT driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
