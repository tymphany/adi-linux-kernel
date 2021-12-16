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

/* NOTE: All messages except FRAG_READY, OVERRUN and UNDERRUN
 * are sent to SHARC core which responds with corresponding ACK.
 * The FRAG_READY, OVERRUN and UNDERRUN and commands are sent by SHARC core
 * after it services SPORT a DMA interrupt and feed a processed fragments to/from the DMA buffer.
 */
enum sharc_msg_id{
	SHARC_MSG_INIT  = 0,

	SHARC_MSG_PLAYBACK_INIT  = 1,
	SHARC_MSG_PLAYBACK_START = 2,
	SHARC_MSG_PLAYBACK_STOP  = 3,
	SHARC_MSG_PLAYBACK_PAUSE  = 4,
	SHARC_MSG_PLAYBACK_RESUME  = 5,
	SHARC_MSG_PLAYBACK_FRAG_READY  = 6,
	SHARC_MSG_PLAYBACK_UNDERRUN  = 7,

	SHARC_MSG_RECORD_INIT  = 21,
	SHARC_MSG_RECORD_START = 22,
	SHARC_MSG_RECORD_STOP  = 23,
	SHARC_MSG_RECORD_PAUSE  = 24,
	SHARC_MSG_RECORD_RESUME  = 25,
	SHARC_MSG_RECORD_FRAG_READY  = 26,
	SHARC_MSG_RECORD_OVERRUN  = 27,

	SHARC_MSG_INIT_ACK  = 40,

	SHARC_MSG_PLAYBACK_INIT_ACK  = 41,
	SHARC_MSG_PLAYBACK_START_ACK = 42,
	SHARC_MSG_PLAYBACK_STOP_ACK  = 43,
	SHARC_MSG_PLAYBACK_PAUSE_ACK  = 44,
	SHARC_MSG_PLAYBACK_RESUME_ACK  = 45,
	SHARC_MSG_PLAYBACK_FRAG_READY_ACK  = 46,
	SHARC_MSG_PLAYBACK_UNDERRUN_ACK  = 47,

	SHARC_MSG_RECORD_INIT_ACK  = 61,
	SHARC_MSG_RECORD_START_ACK = 62,
	SHARC_MSG_RECORD_STOP_ACK  = 63,
	SHARC_MSG_RECORD_PAUSE_ACK  = 64,
	SHARC_MSG_RECORD_RESUME_ACK  = 65,
	SHARC_MSG_RECORD_FRAG_READY_ACK  = 66,
	SHARC_MSG_RECORD_OVERRUN_ACK  = 67,

	SHARC_MSG_ERROR_MSG_DROPPED  = 100,
};

struct sharc_audio_buf_format {
	u32 frame_size;
	u32 channels;
	u32 pcm_format;
	u32 pcm_rate;
};

struct sharc_audio_buf {
	void *buf;
	u32 offset;
	u32 buf_size;
	u32 frag_size;
	u32 fragcount;
	u32 frames_per_frag;
	struct sharc_audio_buf_format format;
};

union sharc_msg_payload{
		uint8_t bytes[32];
		uint32_t ui;
		struct sharc_audio_buf audio_buf[2];
};

struct sharc_msg {
	u32 id;
	union sharc_msg_payload payload;
};

int sport_tx_stop(struct sport_device *sport);
int send_sharc_msg(struct sport_device *sport, int core, enum sharc_msg_id id, union sharc_msg_payload *payload);

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

int send_sharc_msg(struct sport_device *sport, int core, enum sharc_msg_id id, union sharc_msg_payload *payload){
	struct sharc_msg msg;
	char _env[64];
	char *envp[]={_env, NULL};
	int ret = 0;

	mutex_lock(&sport->rpmsg_lock);

	if(sport->sharc_rpmsg[core] == NULL){
		ret = -ENODEV;
		goto send_sharc_msg_error;
	}

	msg.id = id;
	memset(&msg.payload, 0, sizeof(union sharc_msg_payload));
	if(payload)
		msg.payload = *payload;

	ret = rpmsg_send(sport->sharc_rpmsg[core]->ept, &msg, sizeof(msg));
	if(ret < 0){
		goto send_sharc_msg_error;
	}

	ret = wait_for_completion_interruptible_timeout(&sport->sharc_msg_ack_complete[core], SHARC_MSG_TIMEOUT);
	if(ret > 0){
		//dev_dbg(&sport->pdev->dev, "SHARC_%d msg acked\n", core);
		ret = 0;
	}else if(ret < 0){
		if (ret == -ERESTARTSYS){
			dev_info(&sport->pdev->dev, "SHARC_%d comm interrupted\n", core);
		}else{
			dev_err(&sport->pdev->dev, "SHARC_%d comm error %d\n", core, ret);
		}
	}else{
		//timeout
		snprintf(_env, sizeof(_env), "EVENT=SHARC%d_TIMEOUT_%d", core, id);
		kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
		ret = -ETIMEDOUT;
	}

send_sharc_msg_error:
	mutex_unlock(&sport->rpmsg_lock);
	return ret;
}

void parse_sharc_messages(struct sport_device *sport, int core, struct sharc_msg *msg){

	// wakeup waiting timeout worker
	complete(&sport->sharc_msg_ack_complete[core]);

	//handle received message
	switch(msg->id){
		case SHARC_MSG_INIT_ACK:
			break;
		case SHARC_MSG_PLAYBACK_INIT_ACK:
			break;

		case SHARC_MSG_PLAYBACK_START_ACK:
			//enable DMA, after SHARC ACKs START
			set_dma_next_desc_addr(sport->tx_dma_chan,
					sport->tx_desc_phy);
			set_dma_config(sport->tx_dma_chan, DMAFLOW_LIST | DI_EN
					| compute_wdsize(sport->wdsize) | NDSIZE_6);
			enable_dma(sport->tx_dma_chan);
			iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI,
					&sport->tx_regs->spctl);

			//update buffer pointer
			mutex_lock(&sport->sharc_tx_buf_pos_lock);
			sport->sharc_tx_buf_pos += msg->payload.ui * sport->tx_fragsize;
			if(sport->sharc_tx_buf_pos >= sport->sharc_tx_dma_buf.bytes){
				sport->sharc_tx_buf_pos = sport->sharc_tx_buf_pos - sport->sharc_tx_dma_buf.bytes;
			}
			mutex_unlock(&sport->sharc_tx_buf_pos_lock);

			sport->tx_callback(sport->tx_data);
			break;

		case SHARC_MSG_PLAYBACK_STOP_ACK:
			break;
		case SHARC_MSG_PLAYBACK_PAUSE_ACK:
			break;
		case SHARC_MSG_PLAYBACK_RESUME_ACK:
			break;

		case SHARC_MSG_PLAYBACK_FRAG_READY:
			mutex_lock(&sport->sharc_tx_buf_pos_lock);
			sport->sharc_tx_buf_pos += msg->payload.ui * sport->tx_fragsize;
			if(sport->sharc_tx_buf_pos >= sport->sharc_tx_dma_buf.bytes){
				sport->sharc_tx_buf_pos = sport->sharc_tx_buf_pos - sport->sharc_tx_dma_buf.bytes;
			}
			mutex_unlock(&sport->sharc_tx_buf_pos_lock);
			send_sharc_msg(sport, core, SHARC_MSG_PLAYBACK_FRAG_READY_ACK, NULL);

			sport->tx_callback(sport->tx_data);
			break;

		case SHARC_MSG_PLAYBACK_UNDERRUN:
			sharc_playback_underrun_uevent(sport, core);
			send_sharc_msg(sport, core, SHARC_MSG_PLAYBACK_UNDERRUN_ACK, NULL);
			break;

		case SHARC_MSG_RECORD_INIT_ACK:
			break;
		case SHARC_MSG_RECORD_START_ACK:
			break;
		case SHARC_MSG_RECORD_STOP_ACK:
			break;
		case SHARC_MSG_RECORD_PAUSE_ACK:
			break;
		case SHARC_MSG_RECORD_RESUME_ACK:
			break;

		case SHARC_MSG_RECORD_FRAG_READY:
			mutex_lock(&sport->sharc_rx_buf_pos_lock);
			sport->sharc_rx_buf_pos += msg->payload.ui * sport->rx_fragsize;
			if(sport->sharc_rx_buf_pos >= sport->sharc_rx_dma_buf.bytes){
				sport->sharc_rx_buf_pos = sport->sharc_rx_buf_pos - sport->sharc_rx_dma_buf.bytes;
			}
			mutex_unlock(&sport->sharc_rx_buf_pos_lock);

			send_sharc_msg(sport, core, SHARC_MSG_RECORD_FRAG_READY_ACK, NULL);
			sport->rx_callback(sport->rx_data);
			break;

		case SHARC_MSG_RECORD_OVERRUN:
			sharc_record_overrun_uevent(sport, core);
			send_sharc_msg(sport, core, SHARC_MSG_RECORD_OVERRUN_ACK, NULL);
			break;

		case SHARC_MSG_ERROR_MSG_DROPPED:
			sharc_msg_dropped_uevent(sport, core);
			break;

		default:
#if _DEBUG
			BUG(); // Unknown command
#endif
			break;
	}
}

int sport_set_tx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI){
		//try to stop tx
		dev_warn(&sport->pdev->dev, "tx pcm is running during playback init, stoping ...\n");
		sport_tx_stop(sport);
		if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI){
			return -EBUSY;
		}
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

int sport_tx_start(struct sport_device *sport)
{
	return send_sharc_msg(sport, 0, SHARC_MSG_PLAYBACK_START, NULL); //TODO select core
}
EXPORT_SYMBOL(sport_tx_start);

int sport_rx_start(struct sport_device *sport)
{
	set_dma_next_desc_addr(sport->rx_dma_chan,
			sport->rx_desc_phy);
	set_dma_config(sport->rx_dma_chan, DMAFLOW_LIST | DI_EN | WNR
			| compute_wdsize(sport->wdsize) | NDSIZE_6);
	enable_dma(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	return send_sharc_msg(sport, 0, SHARC_MSG_RECORD_START, NULL); //TODO select core;
}
EXPORT_SYMBOL(sport_rx_start);

int sport_tx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	disable_dma(sport->tx_dma_chan);
	return send_sharc_msg(sport, 0, SHARC_MSG_PLAYBACK_STOP, NULL); //TODO select core;
}
EXPORT_SYMBOL(sport_tx_stop);

int sport_rx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	disable_dma(sport->rx_dma_chan); //TODO ?? move to SHARC_MSG_RECORD_STOP_ACK or its timeout worker ??
	return send_sharc_msg(sport, 0, SHARC_MSG_RECORD_STOP, NULL); //TODO select core;
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
	dma_addr_t buf;
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
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	unsigned int cfg;
	union sharc_msg_payload payload;
	int ret;

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

	// Set ALSA buffer size and pointer
	payload.audio_buf[0].buf = buf;
	payload.audio_buf[0].offset = 0;
	payload.audio_buf[0].buf_size = fragsize * fragcount;
	payload.audio_buf[0].frag_size = fragsize;
	payload.audio_buf[0].fragcount = fragcount;
	payload.audio_buf[0].frames_per_frag = bytes_to_frames(substream->runtime, fragsize);
	// Set audio data format
	payload.audio_buf[0].format.channels = params_channels(&sport->tx_hw_params);
	payload.audio_buf[0].format.pcm_format = params_format(&sport->tx_hw_params);
	payload.audio_buf[0].format.frame_size = payload.audio_buf[0].format.channels * (snd_pcm_format_physical_width(payload.audio_buf[0].format.pcm_format)/8);
	payload.audio_buf[0].format.pcm_rate = params_rate(&sport->tx_hw_params);

	// Set DMA buffer size and pointer
	payload.audio_buf[1].buf = (void*)sport->sharc_tx_dma_buf.addr;
	payload.audio_buf[1].offset = 0;
	payload.audio_buf[1].buf_size = fragsize * fragcount;
	payload.audio_buf[1].frag_size = fragsize;
	payload.audio_buf[1].fragcount = fragcount;
	payload.audio_buf[1].frames_per_frag = bytes_to_frames(substream->runtime, fragsize);
	// Set audio data format - same as ALSA buffer
	payload.audio_buf[1].format = payload.audio_buf[0].format;

#if _DEBUG
	dev_info(&sport->pdev->dev, "ALSA  playback buf PA:0x%08x size:%d fragsize:%d fragcount:%d\n", (u32)payload.audio_buf[0].buf, payload.audio_buf[0].buf_size, payload.audio_buf[0].frag_size, payload.audio_buf[0].fragcount);
	dev_info(&sport->pdev->dev, "SHARC playback buf PA:0x%08x size:%d fragsize:%d fragcount:%d\n", (u32)payload.audio_buf[1].buf, payload.audio_buf[1].buf_size, payload.audio_buf[1].frag_size, payload.audio_buf[1].fragcount);
#endif

	return send_sharc_msg(sport, 0, SHARC_MSG_PLAYBACK_INIT, &payload); //TODO select core
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	unsigned int cfg;
	union sharc_msg_payload payload;
	int ret;

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

	// Set ALSA buffer size and pointer
	payload.audio_buf[1].buf = buf;
	payload.audio_buf[1].offset = 0;
	payload.audio_buf[1].buf_size = fragsize * fragcount;
	payload.audio_buf[1].frag_size = fragsize;
	payload.audio_buf[1].fragcount = fragcount;
	payload.audio_buf[1].frames_per_frag = bytes_to_frames(substream->runtime, fragsize);
	// Set audio data format
	payload.audio_buf[1].format.channels = params_channels(&sport->rx_hw_params);
	payload.audio_buf[1].format.pcm_format = params_format(&sport->rx_hw_params);
	payload.audio_buf[1].format.frame_size = payload.audio_buf[1].format.channels * (snd_pcm_format_physical_width(payload.audio_buf[1].format.pcm_format)/8);
	payload.audio_buf[1].format.pcm_rate = params_rate(&sport->rx_hw_params);

	// Set DMA buffer size and pointer
	payload.audio_buf[0].buf = (void*)sport->sharc_rx_dma_buf.addr;
	payload.audio_buf[0].offset = 0;
	payload.audio_buf[0].buf_size = fragsize * fragcount;
	payload.audio_buf[0].frag_size = fragsize;
	payload.audio_buf[0].fragcount = fragcount;
	payload.audio_buf[0].frames_per_frag = bytes_to_frames(substream->runtime, fragsize);
	// Set audio data format - same as ALSA buffer
	payload.audio_buf[0].format = payload.audio_buf[1].format;

#if _DEBUG
	dev_info(&sport->pdev->dev, "ALSA  record buf PA:0x%08x size:%d fragsize:%d fragcount:%d\n", (u32)payload.audio_buf[1].buf, payload.audio_buf[1].buf_size, payload.audio_buf[1].frag_size, payload.audio_buf[1].fragcount);
	dev_info(&sport->pdev->dev, "SHARC record buf PA:0x%08x size:%d fragsize:%d fragcount:%d\n", (u32)payload.audio_buf[0].buf, payload.audio_buf[0].buf_size, payload.audio_buf[0].frag_size, payload.audio_buf[0].fragcount);
#endif

	return send_sharc_msg(sport, 0, SHARC_MSG_RECORD_INIT, &payload); //TODO select core
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

int rpmsg_sharc_alsa_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct sharc_msg *msg = (struct sharc_msg *)data;
	struct sport_device *sport;
	int core;

	sport = sport_devices[0]; // TODO add support for multiple sport devices

	if(sport == NULL)
		return -ENODEV;

	//Sanity check
	if(len != sizeof(struct sharc_msg)){
		dev_err(&rpdev->dev, "Wrong message size %d expected %ld\n", len, sizeof(struct sharc_msg));
	}

	for(core = 0; core < SHARC_CORES_NUM; core++){
		if(sport_devices[0]->sharc_rpmsg[core] == rpdev)
			break;
	}

	if(core >= SHARC_CORES_NUM){
		dev_err(&rpdev->dev, "No recipient\n");
		return -ENODEV;
	}

	parse_sharc_messages(sport, core, msg);
	return 0;
}
EXPORT_SYMBOL(rpmsg_sharc_alsa_cb);

int rpmsg_sharc_alsa_probe(struct rpmsg_device *rpdev)
{
	int sharc_core;
	struct sport_device *sport;
	union sharc_msg_payload payload;

	sport = sport_devices[0]; // TODO add support for multiple sport devices

	if(sport == NULL)
		return -ENODEV;

	switch(rpdev->dst){
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
	sport->sharc_rpmsg[sharc_core] = rpdev;
	dev_info(&sport->pdev->dev, "sharc-alsa client device is attached, addr: 0x%03x\n", rpdev->dst);

	payload.ui = sport->sport_channel;
	send_sharc_msg(sport, sharc_core, SHARC_MSG_INIT, &payload);

	return 0;
}
EXPORT_SYMBOL(rpmsg_sharc_alsa_probe);

void rpmsg_sharc_alsa_remove(struct rpmsg_device *rpdev)
{
	int i;
	struct sport_device *sport;

	sport = sport_devices[0]; // TODO add support for multiple sport devices

	if(sport == NULL)
		return;

	for(i = 0; i < SHARC_CORES_NUM; i++){
		if (sport->sharc_rpmsg[i] == rpdev)
			sport->sharc_rpmsg[i] = NULL;
			//TODO stop active streams
	}
	dev_info(&sport->pdev->dev, "sharc-alsa client device is removed, addr: 0x%03x\n", rpdev->dst);
}
EXPORT_SYMBOL(rpmsg_sharc_alsa_remove);

struct sport_device *sport_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sport_device *sport;
	int ret;
	int i;

	pads_init();
	sru_init();

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

	mutex_init(&sport->rpmsg_lock);
	mutex_init(&sport->sharc_tx_buf_pos_lock);
	mutex_init(&sport->sharc_rx_buf_pos_lock);

	for (i = 0; i < SHARC_CORES_NUM; i++){
		init_completion(&sport->sharc_msg_ack_complete[i]);
	}

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
	int i;

	sport_devices[0] = NULL; // TODO add multiple sport devices support

	//wakeup all the workers before destroying workqueue so we don't wait for timeouts
	for(i = 0; i < SHARC_CORES_NUM; i++){
		complete_all(&sport->sharc_msg_ack_complete[i]);
	}

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
