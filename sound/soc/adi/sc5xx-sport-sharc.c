/*
 * sc5xx-sport-sharc.c Analog Devices SC5XX SHARC SPORT driver.
 * Data proccessed and feed into SPORT DMA buff by a SHARC core.
 * Code based on sc5xx-sport.c
 *
 * Copyright (c) 2015-2021 Analog Devices Inc.
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
#include <sound/pcm.h>

#include <mach/icc.h>
#include "sc5xx-sport.h"

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

// TODO make the constants configurable, SHARC_MAX_MSG in kconfig, SHARC_DMA_X_BUF_FRAGMENTS in api
#define SHARC_MAX_MSG 64

// 2 is minimum form smooth playback/record
#define SHARC_DMA_PLAYBACK_BUF_FRAGMENTS 5 // size of dma buffer sharc is writing to during playback, this adds latecy to the audio signal
#define SHARC_DMA_RECORD_BUF_FRAGMENTS 5 // size of dma buffer sharc is reading from during record, this adds latecy to the audio signal

enum sharc_msg_id{
	SHARC_MSG_PLAYBACK_INIT  = 1,
	SHARC_MSG_PLAYBACK_START = 2,
	SHARC_MSG_PLAYBACK_STOP  = 3,
	SHARC_MSG_PLAYBACK_PAUSE  = 4,
	SHARC_MSG_PLAYBACK_RESUME  = 5,
	SHARC_MSG_PLAYBACK_BUF  = 6,
	SHARC_MSG_PLAYBACK_FRAG_READY  = 7,

	SHARC_MSG_RECORD_INIT  = 21,
	SHARC_MSG_RECORD_START = 22,
	SHARC_MSG_RECORD_STOP  = 23,
	SHARC_MSG_RECORD_PAUSE  = 24,
	SHARC_MSG_RECORD_RESUME  = 25,
	SHARC_MSG_RECORD_BUF  = 26,
	SHARC_MSG_RECORD_FRAG_READY  = 27,

	SHARC_MSG_PLAYBACK_INIT_ACK  = 31,
	SHARC_MSG_PLAYBACK_START_ACK = 32,
	SHARC_MSG_PLAYBACK_STOP_ACK  = 33,
	SHARC_MSG_PLAYBACK_PAUSE_ACK  = 34,
	SHARC_MSG_PLAYBACK_RESUME_ACK  = 35,
	SHARC_MSG_PLAYBACK_BUF_ACK  = 36,
	SHARC_MSG_PLAYBACK_FRAG_READY_ACK  = 37,

	SHARC_MSG_RECORD_INIT_ACK  = 41,
	SHARC_MSG_RECORD_START_ACK = 42,
	SHARC_MSG_RECORD_STOP_ACK  = 43,
	SHARC_MSG_RECORD_PAUSE_ACK  = 44,
	SHARC_MSG_RECORD_RESUME_ACK  = 45,
	SHARC_MSG_RECORD_BUF_ACK  = 46,
	SHARC_MSG_RECORD_FRAG_READY_ACK  = 47,

};

struct sharc_audio_buf {
	void *buf;
	u32 offset;
	u32 buf_size;
	u32 frag_size;
};

union sharc_msg_payload{
		uint8_t bytes[32];
		uint32_t ui;
		struct sharc_audio_buf audio_buf[2];
};

struct sharc_msg {
	u32 wait_ack;
	u32 id;
	union sharc_msg_payload payload;
};


void send_sharc_msg(struct sport_device *sport, u32 id, union sharc_msg_payload *payload){
	struct sharc_msg *msg = &sport->messages[sport->message_queue_pointer];

	if(msg->wait_ack){
		// No space for new message
		BUG();
	}

	msg->id = id;
	if(payload)
		msg->payload = *payload;
	else
		memset(&msg->payload, 0, sizeof(union sharc_msg_payload));

	msg->wait_ack = 1;
	
	sport->message_queue_pointer += 1;
	if(sport->message_queue_pointer >= SHARC_MAX_MSG){
		sport->message_queue_pointer = 0;
	}

	wmb(); // drain writebuffer
	platform_send_ipi_cpu(1, 0); //FIXME change fixed core number
}

void read_sharc_messages(struct sport_device *sport){
	struct sharc_msg *msg = &sport->received_messages[sport->receive_message_queue_pointer];
	unsigned long flags;

	if(msg->wait_ack == 0){
		//got an ICC interrupt but no data available
		//we are out of sync, reset the counters
		sport->receive_message_queue_pointer = 0;
		sport->message_queue_pointer = 0;
		msg = &sport->received_messages[sport->receive_message_queue_pointer];
	}

	while(msg->wait_ack){
		dev_dbg(&sport->pdev->dev, "Got message id %d, payload ui %d\n", msg->id, msg->payload.ui);
		switch(msg->id){
			case SHARC_MSG_PLAYBACK_INIT_ACK:
				break;
			case SHARC_MSG_PLAYBACK_START_ACK:
				//enable DMA, after SHARC ACKs START
				set_dma_next_desc_addr(sport->tx_dma_chan,
						(void *)sport->tx_desc_phy);
				set_dma_config(sport->tx_dma_chan, DMAFLOW_LIST | DI_EN
						| compute_wdsize(sport->wdsize) | NDSIZE_6);
				enable_dma(sport->tx_dma_chan);
				iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI,
						&sport->tx_regs->spctl);

				//update buffer pointer
				spin_lock_irqsave(&sport->icc_spinlock, flags);
				sport->sharc_tx_buf_pos += msg->payload.ui * sport->tx_fragsize;
				sport->sharc_tx_buf_pos %= sport->tx_buf_size;
				spin_unlock_irqrestore(&sport->icc_spinlock, flags);
				if (sport->tx_callback)
					sport->tx_callback(sport->tx_data);
				break;
			case SHARC_MSG_PLAYBACK_STOP_ACK:
				break;
			case SHARC_MSG_PLAYBACK_PAUSE_ACK:
				break;
			case SHARC_MSG_PLAYBACK_RESUME_ACK:
				break;
			case SHARC_MSG_PLAYBACK_BUF_ACK:
				break;
			case SHARC_MSG_PLAYBACK_FRAG_READY_ACK:
				spin_lock_irqsave(&sport->icc_spinlock, flags);
				sport->sharc_tx_buf_pos += msg->payload.ui * sport->tx_fragsize;
				sport->sharc_tx_buf_pos %= sport->tx_buf_size;
				spin_unlock_irqrestore(&sport->icc_spinlock, flags);
				if (sport->tx_callback)
					sport->tx_callback(sport->tx_data);
				break;
			default:
				BUG();
				break;
		}

		msg->wait_ack = 0;
		sport->receive_message_queue_pointer += 1;
		if (sport->receive_message_queue_pointer >= SHARC_MAX_MSG){
			sport->receive_message_queue_pointer = 0;
		}
		msg = &sport->received_messages[sport->receive_message_queue_pointer];
	}
}

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

void sport_tx_start(struct sport_device *sport)
{
	union sharc_msg_payload payload;

	payload.ui = SHARC_DMA_PLAYBACK_BUF_FRAGMENTS;
	send_sharc_msg(sport, SHARC_MSG_PLAYBACK_START, &payload);
}
EXPORT_SYMBOL(sport_tx_start);

void sport_rx_start(struct sport_device *sport)
{
}
EXPORT_SYMBOL(sport_rx_start);

void sport_tx_stop(struct sport_device *sport)
{
	send_sharc_msg(sport, SHARC_MSG_PLAYBACK_STOP, NULL);
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	disable_dma(sport->tx_dma_chan);
}
EXPORT_SYMBOL(sport_tx_stop);

void sport_rx_stop(struct sport_device *sport)
{
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
	union sharc_msg_payload payload;

	if (sport->tx_desc)
		dma_free_coherent(&sport->pdev->dev, sport->tx_desc_size,
				sport->tx_desc, sport->tx_desc_phy);

	//TODO make sport->tx_frags configurable in api, allocate acording dma buf size and tx_desc count
	//sport->tx_frags = SHARC_DMA_PLAYBACK_BUF_FRAGMENTS;

	sport->tx_desc = dma_alloc_coherent(&sport->pdev->dev, fragcount * sizeof(struct dmasg), &sport->tx_desc_phy, GFP_KERNEL);
	sport->tx_desc_size = fragcount * sizeof(struct dmasg);
	if (!sport->tx_desc)
		return -ENOMEM;

	if (sport->sharc_tx_buf)
		dma_free_coherent(&sport->pdev->dev, sport->tx_buf_size, sport->sharc_tx_buf, sport->sharc_tx_buf_phy);
	sport->sharc_tx_buf = dma_alloc_coherent(&sport->pdev->dev, fragsize * fragcount, &sport->sharc_tx_buf_phy, GFP_KERNEL);
	if (!sport->sharc_tx_buf)
		return -ENOMEM;

	sport->tx_buf = sport->sharc_tx_buf_phy;
	sport->tx_fragsize = fragsize;
	sport->tx_frags = fragcount;
	sport->tx_buf_size = fragsize * fragcount;
	sport->sharc_tx_buf_pos = 0;

	cfg = DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize)
		| NDSIZE_6 | DMAEN;

	setup_desc(sport, fragcount, fragsize, cfg, 1);

	payload.audio_buf[0].buf=buf;
	payload.audio_buf[0].offset = 0;
	payload.audio_buf[0].buf_size = fragsize * fragcount;
	payload.audio_buf[0].frag_size = fragsize;

	payload.audio_buf[1].buf=sport->sharc_tx_buf_phy;
	payload.audio_buf[1].offset = 0;
	payload.audio_buf[1].buf_size = fragsize * fragcount;
	payload.audio_buf[1].frag_size = fragsize;
	send_sharc_msg(sport, SHARC_MSG_PLAYBACK_BUF, &payload);

	return 0;
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize)
{
}
EXPORT_SYMBOL(sport_config_rx_dma);

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	unsigned long flags;
	unsigned long off;

	spin_lock_irqsave(&sport->icc_spinlock, flags);
	off = sport->sharc_tx_buf_pos;
	spin_unlock_irqrestore(&sport->icc_spinlock, flags);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_tx);

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	unsigned long flags;
	unsigned long off;

	spin_lock_irqsave(&sport->icc_spinlock, flags);
	off = sport->sharc_rx_buf_pos;
	spin_unlock_irqrestore(&sport->icc_spinlock, flags);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_rx);

irqreturn_t sharc_ICC_irq_threded_handler(int irq, void *dev_instance){
	struct sport_device *sport = (struct sport_device *)dev_instance;

	read_sharc_messages(sport);

	return IRQ_HANDLED;
}

static irqreturn_t sport_tx_irq(int irq, void *dev_id)
{
	struct sport_device *sport = dev_id;
	static unsigned long status, flags;
	union sharc_msg_payload payload;

	status = get_dma_curr_irqstat(sport->tx_dma_chan);
	if (status & (DMA_DONE|DMA_ERR))
		clear_dma_irqstat(sport->tx_dma_chan);

	payload.ui = 1;
	send_sharc_msg(sport, SHARC_MSG_PLAYBACK_FRAG_READY, &payload);

	return IRQ_HANDLED;
}

static irqreturn_t sport_rx_irq(int irq, void *dev_id)
{
	struct sport_device *sport = dev_id;
	unsigned long status;

	status = get_dma_curr_irqstat(sport->rx_dma_chan);
	if (status & (DMA_DONE|DMA_ERR))
		clear_dma_irqstat(sport->rx_dma_chan);

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

	sport->icc_irq = platform_get_irq(pdev, 2);
	if (sport->icc_irq <= 0) {
		dev_err(dev, "No ICC IRQ specified\n");
		return -ENOENT;
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	sport->icc_irq_type = (res->flags & IORESOURCE_BITS) | IRQF_PERCPU;

	return 0;
}

static int sport_request_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	int ret;
	const u32 sharc_msg_buf_size = SHARC_MAX_MSG * sizeof(struct sharc_msg);

	ret = devm_request_threaded_irq(dev, sport->icc_irq, NULL, sharc_ICC_irq_threded_handler, sport->icc_irq_type | IRQF_ONESHOT,
			"ICC receive IRQ", sport);

	if (ret) {
		dev_err(dev, "Fail to request ICC receive IRQ\n");
		return -ENOENT;
	}

	sport->messages = ioremap_nocache(0x20001000, sharc_msg_buf_size);
	sport->message_queue_pointer = 0;
	memset(sport->messages, 0, sharc_msg_buf_size);

	sport->received_messages = ioremap_nocache(0x20001000 + sharc_msg_buf_size, sharc_msg_buf_size);;
	sport->receive_message_queue_pointer = 0;
	memset(sport->received_messages, 0, sharc_msg_buf_size);

	/*An interrupt with empty message tells SHARC to reset its message counters*/
	platform_send_ipi_cpu(1, 0); //FIXME change fixed core number

	spin_lock_init(&sport->icc_spinlock);

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
	iounmap(sport->messages);
	iounmap(sport->received_messages);
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

	if (sport->sharc_tx_buf)
		dma_free_coherent(&sport->pdev->dev, sport->tx_buf_size, sport->sharc_tx_buf, sport->sharc_tx_buf_phy);
	if (sport->sharc_rx_buf)
		dma_free_coherent(&sport->pdev->dev, sport->rx_buf_size, sport->sharc_rx_buf, sport->sharc_rx_buf_phy);

	if (sport->tx_desc)
		dma_free_coherent(&sport->pdev->dev, sport->tx_desc_size,
				sport->tx_desc, sport->tx_desc_phy);
	if (sport->rx_desc)
		dma_free_coherent(&sport->pdev->dev, sport->rx_desc_size,
				sport->rx_desc, sport->rx_desc_phy);
	sport_free_resource(sport);
	kfree(sport);
}
EXPORT_SYMBOL(sport_delete);

MODULE_DESCRIPTION("Analog Devices SC5XX SPORT driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
MODULE_LICENSE("GPL v2");
