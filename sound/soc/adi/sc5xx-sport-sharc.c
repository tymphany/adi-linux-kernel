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
#include <linux/kobject.h>
#include <linux/atomic.h>

#include <linux/soc/adi/cpu.h>
#include <linux/soc/adi/dma.h>
#include <linux/soc/adi/portmux.h>
#include <sound/sc5xx-sru.h>
#include <sound/sc5xx-dai.h>
#include <sound/pcm.h>
#include <linux/rpmsg.h>

#include "sc5xx-sport.h"

#define _DEBUG 0

#define SHARC_MSG_TIMEOUT usecs_to_jiffies(1000)

#define SHARC0_ALSA_RPMSG_REMOTE_ADDR 101
#define SHARC1_ALSA_RPMSG_REMOTE_ADDR 102

static struct sport_device *sport_devices[1];

int sport_tx_stop(struct sport_device *sport);

void sharc_playback_underrun_uevent(struct sport_device *sport, int core){
	char _env[64];
	char *envp[]={_env, NULL};
	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_UNDERRUN", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

void sharc_record_overrun_uevent(struct sport_device *sport, int core){
	char _env[64];
	char *envp[]={_env, NULL};
	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_OVERRUN", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

void sharc_msg_dropped_uevent(struct sport_device *sport, int core){
	char _env[64];
	char *envp[]={_env, NULL};
	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_MSG_DROPPED", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

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

static int sport_playback_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *frags)
{
	struct sport_device *sport = (struct sport_device *)icap->priv;

	mutex_lock(&sport->sharc_tx_buf_pos_lock);
	sport->sharc_tx_buf_pos += frags->frags * sport->tx_fragsize;
	if(sport->sharc_tx_buf_pos >= sport->sharc_tx_dma_buf.bytes) {
		sport->sharc_tx_buf_pos = sport->sharc_tx_buf_pos - sport->sharc_tx_dma_buf.bytes;
	}
	mutex_unlock(&sport->sharc_tx_buf_pos_lock);
	sport->tx_callback(sport->tx_data);
	return 0;
}

static int sport_record_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *frags)
{
	struct sport_device *sport = (struct sport_device *)icap->priv;

	mutex_lock(&sport->sharc_rx_buf_pos_lock);
	sport->sharc_rx_buf_pos += frags->frags * sport->rx_fragsize;
	if(sport->sharc_rx_buf_pos >= sport->sharc_rx_dma_buf.bytes) {
		sport->sharc_rx_buf_pos = sport->sharc_rx_buf_pos - sport->sharc_rx_dma_buf.bytes;
	}
	mutex_unlock(&sport->sharc_rx_buf_pos_lock);

	sport->rx_callback(sport->rx_data);
	return 0;
}

struct icap_application_callbacks sport_icap_callbacks = {
	.playback_frag_ready = sport_playback_frag_ready_cb,
	.record_frag_ready = sport_record_frag_ready_cb,
};

int sport_set_tx_params(struct sport_device *sport,
			struct sport_params *params)
{
	int ret;

	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI){
		//try to stop tx
		dev_warn(&sport->pdev->dev, "tx pcm is running during playback init, stoping ...\n");
		sport_tx_stop(sport);
		if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI){
			return -EBUSY;
		}
	}

	ret = icap_get_device_features(&sport->icap[0], &sport->icap_sport_features);
	if (ret) {
		return ret;
	}

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

void sport_tx_start_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_tx_start_work);
	int ret;
	ret = icap_playback_start(&sport->icap[0]); //TODO select core;
	if (ret) {
		dev_err(&sport->pdev->dev, "tx_start error: %d", ret);
	}
}

int sport_tx_start(struct sport_device *sport)
{
	int32_t ret;
	ret = queue_work(system_highpri_wq, &sport->send_tx_start_work);
	if (ret == 0) {
		return -EIO;
	}

	//enable DMA, after SHARC ACKs START
	set_dma_next_desc_addr(sport->tx_dma_chan, sport->tx_desc_phy);
	set_dma_config(sport->tx_dma_chan, DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize) | NDSIZE_6);
	enable_dma(sport->tx_dma_chan);
	iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI, &sport->tx_regs->spctl);

	return 0;
}
EXPORT_SYMBOL(sport_tx_start);

void sport_rx_start_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_rx_start_work);
	int ret;
	ret = icap_record_start(&sport->icap[0]); //TODO select core;
	if (ret) {
		dev_err(&sport->pdev->dev, "rx_start error: %d", ret);
	}
}

int sport_rx_start(struct sport_device *sport)
{
	int ret;
	set_dma_next_desc_addr(sport->rx_dma_chan,
			sport->rx_desc_phy);
	set_dma_config(sport->rx_dma_chan, DMAFLOW_LIST | DI_EN | WNR
			| compute_wdsize(sport->wdsize) | NDSIZE_6);
	enable_dma(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	
	ret = queue_work(system_highpri_wq, &sport->send_rx_start_work);
	return !ret;
}
EXPORT_SYMBOL(sport_rx_start);

void sport_tx_stop_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_tx_stop_work);
	int ret;
	ret = icap_playback_stop(&sport->icap[0]); //TODO select core;
	if (ret) {
		dev_err(&sport->pdev->dev, "tx_stop error: %d", ret);
	}
	sport->pending_tx_stop = 0;
	wake_up_interruptible_all(&sport->pending_tx_stop_event);
}

int sport_tx_stop(struct sport_device *sport)
{
	int ret;
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	disable_dma(sport->tx_dma_chan);

	/*
	 * Can't send icap message and wait for response here as we can be in interrupt context.
	 * FRAG_READY callbacks are processed in the rpmsg interrupt context.
	 * The FRAG_READY cb calls snd_pcm_period_elapsed() which can call stop trigger on xrun event.
	 * Waiting here blocks entire rpmsg communication on the rpdev.
	 */
	sport->pending_tx_stop = 1;
	ret = queue_work(system_highpri_wq, &sport->send_tx_stop_work);
	return !ret;
}
EXPORT_SYMBOL(sport_tx_stop);


void sport_rx_stop_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_rx_stop_work);
	int ret;
	ret = icap_record_stop(&sport->icap[0]); //TODO select core;
	if (ret) {
		dev_err(&sport->pdev->dev, "rx_stop error: %d", ret);
	}
	sport->pending_rx_stop = 0;
	wake_up_interruptible_all(&sport->pending_rx_stop_event);
}

int sport_rx_stop(struct sport_device *sport)
{
	int ret;
	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	disable_dma(sport->rx_dma_chan);

	/*
	 * Can't send icap message and wait for response here as we can be in interrupt context.
	 * FRAG_READY callbacks are processed in the rpmsg interrupt context.
	 * The FRAG_READY cb calls snd_pcm_period_elapsed() which can call stop trigger on xrun event.
	 * Waiting here blocks entire rpmsg communication on the rpdev.
	 */
	sport->pending_rx_stop = 1;
	ret = queue_work(system_highpri_wq, &sport->send_rx_stop_work);
	return !ret;
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
	u32 desc_phy;
	u32 buf;
	int i;

	if (tx) {
		desc = sport->tx_desc;
		desc_phy = lower_32_bits(sport->tx_desc_phy);
		buf = lower_32_bits(sport->tx_buf);
	} else {
		desc = sport->rx_desc;
		desc_phy = lower_32_bits(sport->rx_desc_phy);
		buf = lower_32_bits(sport->rx_buf);
	}

	for (i = 0; i < fragcount; ++i) {
		desc[i].next_desc_addr  = (desc_phy + (i + 1) * sizeof(struct dmasg));
		desc[i].start_addr = buf + i * fragsize;
		desc[i].cfg = cfg;
		desc[i].x_count = fragsize / sport->wdsize;
		desc[i].x_modify = sport->wdsize;
		desc[i].y_count = 0;
		desc[i].y_modify = 0;
	}

	/* make circular */
	desc[fragcount-1].next_desc_addr = desc_phy;
}

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	unsigned int cfg;
	struct icap_buf_descriptor audio_buf;
	int ret;

	ret = wait_event_interruptible(sport->pending_tx_stop_event, !sport->pending_tx_stop);
	if (ret) {
		return ret;
	}

	if (sport->tx_desc)
		dma_free_coherent(&sport->pdev->dev, sport->tx_desc_size,
				sport->tx_desc, sport->tx_desc_phy);

	sport->tx_desc = dma_alloc_coherent(&sport->pdev->dev, fragcount * sizeof(struct dmasg), &sport->tx_desc_phy, GFP_KERNEL);
	sport->tx_desc_size = fragcount * sizeof(struct dmasg);
	if (!sport->tx_desc)
		return -ENOMEM;

	/* Allocate buffer for SHARC output - DMA, prefers iram pool, if not available it fallbacks to CMA */
	snd_dma_free_pages(&sport->sharc_tx_dma_buf);
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_IRAM, &sport->pdev->dev, fragsize * fragcount, &sport->sharc_tx_dma_buf);
	if (ret){
		return ret;
	}

	sport->tx_buf = sport->sharc_tx_dma_buf.addr;
	sport->tx_fragsize = fragsize;
	sport->tx_frags = fragcount;
	sport->sharc_tx_buf_pos = 0;

	cfg = DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize)
		| NDSIZE_6 | DMAEN;

	setup_desc(sport, fragcount, fragsize, cfg, 1);

	sport->tx_substream = substream;

	/*
	 * Potiential error here ignored as the device could be already initialized by record
	 * return -ERESTARTSYS and -ETIMEDOUT only
	 */
	ret = icap_request_device_init(&sport->icap[0], sport->sport_channel);
	if (ret == -ERESTARTSYS || ret == -ETIMEDOUT ){
		return ret;
	}

	if (sport->tx_dma_icap_buf_id != -1) {
		ret = icap_remove_playback_dst(&sport->icap[0], sport->tx_dma_icap_buf_id);
		if (ret) {
			dev_err(&sport->pdev->dev, "tx_stop dst remove error: %d", ret);
		}
		sport->tx_dma_icap_buf_id = -1;
	}

	if (sport->tx_alsa_icap_buf_id != -1) {
		ret = icap_remove_playback_src(&sport->icap[0], sport->tx_alsa_icap_buf_id);
		if (ret) {
			dev_err(&sport->pdev->dev, "tx_stop src remove error: %d", ret);
		}
		sport->tx_alsa_icap_buf_id = -1;
	}

	// Set ALSA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-playback", sport->pdev->name);
	audio_buf.device_id = -1;
	audio_buf.buf = (uint64_t)buf;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format
	audio_buf.channels = params_channels(&sport->tx_hw_params);
	audio_buf.format = params_format(&sport->tx_hw_params);
	audio_buf.rate = params_rate(&sport->tx_hw_params);

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"ALSA  playback buf PA:0x%px size:%d fragsize:%d\n",
		(void*)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_playback_src(&sport->icap[0], &audio_buf, &sport->tx_alsa_icap_buf_id); //TODO select core
	if (ret) {
		return ret;
	}

	// Set DMA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-dma-playback", sport->pdev->name);
	audio_buf.device_id = sport->sport_channel;
	audio_buf.buf = (uint64_t)sport->sharc_tx_dma_buf.addr;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format - same as ALSA buffer
	audio_buf.channels = params_channels(&sport->tx_hw_params);
	audio_buf.format = params_format(&sport->tx_hw_params);
	audio_buf.rate = params_rate(&sport->tx_hw_params);

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"SHARC playback buf PA:0x%px size:%d fragsize:%d\n",
		(void*)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_playback_dst(&sport->icap[0], &audio_buf, &sport->tx_dma_icap_buf_id); //TODO select core
	if (ret) {
		icap_remove_playback_src(&sport->icap[0], sport->tx_alsa_icap_buf_id);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	unsigned int cfg;
	struct icap_buf_descriptor audio_buf;
	int ret;

	ret = wait_event_interruptible(sport->pending_rx_stop_event, !sport->pending_rx_stop);
	if (ret) {
		return ret;
	}

	if (sport->rx_desc)
		dma_free_coherent(&sport->pdev->dev, sport->rx_desc_size,
				sport->rx_desc, sport->rx_desc_phy);

	sport->rx_desc = dma_alloc_coherent(&sport->pdev->dev, fragcount * sizeof(struct dmasg), &sport->rx_desc_phy, GFP_KERNEL);
	sport->rx_desc_size = fragcount * sizeof(struct dmasg);
	if (!sport->rx_desc)
		return -ENOMEM;

	/* Allocate buffer for SHARC input - DMA, prefers iram pool, if not available it fallbacks to CMA */
	snd_dma_free_pages(&sport->sharc_rx_dma_buf);
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_IRAM, &sport->pdev->dev, fragsize * fragcount, &sport->sharc_rx_dma_buf);
	if (ret){
		return ret;
	}

	sport->rx_buf = sport->sharc_rx_dma_buf.addr;
	sport->rx_fragsize = fragsize;
	sport->rx_frags = fragcount;
	sport->sharc_rx_buf_pos = 0;

	cfg = DMAFLOW_LIST | DI_EN | compute_wdsize(sport->wdsize)
		| WNR | NDSIZE_6 | DMAEN;

	setup_desc(sport, fragcount, fragsize, cfg, 0);

	sport->rx_substream = substream;

	/*
	 * Potiential error here ignored as the device could be already initialized by playback
	 * return -ERESTARTSYS and -ETIMEDOUT only
	 */
	ret = icap_request_device_init(&sport->icap[0], sport->sport_channel);
	if (ret == -ERESTARTSYS || ret == -ETIMEDOUT ){
		return ret;
	}

	if (sport->rx_alsa_icap_buf_id != -1) {
		ret = icap_remove_record_dst(&sport->icap[0], sport->rx_alsa_icap_buf_id);
		if (ret) {
			dev_err(&sport->pdev->dev, "tx_stop dst remove error: %d", ret);
		}
		sport->rx_alsa_icap_buf_id = -1;
	}

	if (sport->rx_dma_icap_buf_id != -1) {
		ret = icap_remove_record_src(&sport->icap[0], sport->rx_dma_icap_buf_id);
		if (ret) {
			dev_err(&sport->pdev->dev, "tx_stop src remove error: %d", ret);
		}
		sport->rx_dma_icap_buf_id = -1;
	}

	// Set ALSA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-record", sport->pdev->name);
	audio_buf.device_id = -1;
	audio_buf.buf = (uint64_t)buf;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format
	audio_buf.channels = params_channels(&sport->rx_hw_params);
	audio_buf.format = params_format(&sport->rx_hw_params);
	audio_buf.rate = params_rate(&sport->rx_hw_params);

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"ALSA  record buf PA:0x%px size:%d fragsize:%d\n",
		(void*)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_record_dst(&sport->icap[0], &audio_buf, &sport->rx_alsa_icap_buf_id); //TODO select core
	if (ret) {
		return ret;
	}

	// Set DMA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-dma-record", sport->pdev->name);
	audio_buf.device_id = sport->sport_channel;
	audio_buf.buf = (uint64_t)sport->sharc_rx_dma_buf.addr;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format - same as ALSA buffer
	audio_buf.channels = params_channels(&sport->rx_hw_params);
	audio_buf.format = params_format(&sport->rx_hw_params);
	audio_buf.rate = params_rate(&sport->rx_hw_params);

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"SHARC record buf PA:0x%px size:%d fragsize:%d\n",
		(void*)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_record_src(&sport->icap[0], &audio_buf, &sport->rx_dma_icap_buf_id); //TODO select core
	if (ret) {
		icap_remove_record_dst(&sport->icap[0], sport->rx_alsa_icap_buf_id);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(sport_config_rx_dma);

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	unsigned long off;

	mutex_lock(&sport->sharc_tx_buf_pos_lock);
	off = sport->sharc_tx_buf_pos;
	mutex_unlock(&sport->sharc_tx_buf_pos_lock);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_tx);

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	unsigned long off;

	mutex_lock(&sport->sharc_rx_buf_pos_lock);
	off = sport->sharc_rx_buf_pos;
	mutex_unlock(&sport->sharc_rx_buf_pos_lock);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_rx);

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

	ret = of_property_read_u32(dev->of_node,
			"sport-channel", &sport->sport_channel);
	if (ret) {
		dev_err(dev, "No sport-channel resource\n");
		return -ENODEV;
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

	ret = request_dma(sport->rx_dma_chan, "SPORT RX Data");
	if (ret) {
		dev_err(dev, "Unable to allocate DMA channel for sport rx\n");
		goto err_rx_dma;
	}

	/* NOTE: tx_irq, rx_irq and err_irqs handled by SHARC core*/

	return 0;

err_rx_dma:
	free_dma(sport->tx_dma_chan);
	return ret;
}

static void sport_free_resource(struct sport_device *sport)
{
	free_dma(sport->rx_dma_chan);
	free_dma(sport->tx_dma_chan);
}

int rpmsg_icap_sport_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct sport_device *sport;
	union icap_remote_addr src_addr;
	int ret;

	sport = sport_devices[0]; // TODO add support for multiple sport devices

	if(sport == NULL)
		return -ENODEV;

	src_addr.rpmsg_addr = src;
	ret = icap_parse_msg(&sport->icap[0], &src_addr, data, len); //TODO select core
	if (ret) {
		if (ret == -ICAP_ERROR_TIMEOUT) {
			dev_notice_ratelimited(&rpdev->dev, "ICAP timedout expired for the response\n");
		} else {
			dev_err_ratelimited(&rpdev->dev, "ICAP parse msg error: %d\n", ret);
		}
	}
	return ret;
}
EXPORT_SYMBOL(rpmsg_icap_sport_cb);

int rpmsg_icap_sport_probe(struct rpmsg_device *rpdev)
{
	struct sport_device *sport;
	int sharc_core = 0;
	int ret;

	sport = sport_devices[0]; // TODO add support for multiple sport device

	if(sport == NULL)
		return -ENODEV;

/*	switch(rpdev->dst){
		case SHARC0_ALSA_RPMSG_REMOTE_ADDR:
			sharc_core = 0;
			break;
		case SHARC1_ALSA_RPMSG_REMOTE_ADDR:
			sharc_core = 1;
			break;
		default:
			dev_err(&sport->pdev->dev, "rpmsg sharc-alsa device probe with wrong endpoint address: %d\n", rpdev->dst);
			return -1;
	}
	*/

	dev_set_drvdata(&rpdev->dev, sport);

	ret = icap_application_init(&sport->icap[sharc_core], &sport_icap_callbacks, (void*)rpdev->ept, (void*)sport);
	if (ret) {
		goto error_out;
	}

	dev_info(&sport->pdev->dev, "sharc-alsa client device is attached, addr: 0x%03x\n", rpdev->dst);
	return 0;

error_out:
	dev_err(&sport->pdev->dev, "sharc-alsa client device error, addr: 0x%03x, err: %d\n", rpdev->dst, ret);
	return ret;
}
EXPORT_SYMBOL(rpmsg_icap_sport_probe);

void rpmsg_icap_sport_remove(struct rpmsg_device *rpdev)
{
	struct sport_device *sport = (struct sport_device *)dev_get_drvdata(&rpdev->dev);

	if(sport == NULL)
		return;

	icap_application_deinit(&sport->icap[0]); //todo select core

	//TODO stop active streams
	dev_info(&rpdev->dev, "sharc-alsa client device is removed, addr: 0x%03x\n", rpdev->dst);
}
EXPORT_SYMBOL(rpmsg_icap_sport_remove);

struct sport_device *sport_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sport_device *sport;
	int ret;

	sport = kzalloc(sizeof(*sport), GFP_KERNEL);
	if (!sport) {
		dev_err(dev, "Unable to allocate memory for sport device\n");
		return ERR_PTR(-ENOMEM);
	}
	sport->pdev = pdev;

	ret = sport_get_resource(sport);
	if (ret)
	  goto err_free_data;

	ret = sport_request_resource(sport);
	if (ret)
	  goto err_free_data;

	mutex_init(&sport->sharc_tx_buf_pos_lock);
	mutex_init(&sport->sharc_rx_buf_pos_lock);
	INIT_WORK(&sport->send_tx_start_work, sport_tx_start_work_func);
	INIT_WORK(&sport->send_rx_start_work, sport_rx_start_work_func);
	INIT_WORK(&sport->send_tx_stop_work, sport_tx_stop_work_func);
	INIT_WORK(&sport->send_rx_stop_work, sport_rx_stop_work_func);

	init_waitqueue_head(&sport->pending_tx_stop_event);
	init_waitqueue_head(&sport->pending_rx_stop_event);

	sport->tx_alsa_icap_buf_id = -1;
	sport->tx_dma_icap_buf_id = -1;
	sport->rx_alsa_icap_buf_id = -1;
	sport->rx_dma_icap_buf_id = -1;

	sport_devices[0] = sport; // TODO add multiple sport devices support

	dev_info(dev, "SPORT create success, SHARC-ALSA (PCM steram send to sharc for processing)\n");
	return sport;

err_free_data:
	kfree(sport);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(sport_create);

void sport_delete(struct sport_device *sport)
{
	sport_devices[0] = NULL; // TODO add multiple sport devices support

	snd_dma_free_pages(&sport->sharc_tx_dma_buf);
	snd_dma_free_pages(&sport->sharc_rx_dma_buf);

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
