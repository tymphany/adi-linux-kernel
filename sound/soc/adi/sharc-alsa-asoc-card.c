// SPDX-License-Identifier: GPL-2.0-only
/*
 * sharc-alsa-asoc-card.c - Analog Devices SHARC-ALSA ASoC card driver
 *
 * Copyright 2021 Analog Devices Inc.
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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/rpmsg.h>
#include <linux/workqueue.h>

#include "icap/include/icap_application.h"

#define SHARC_ALSA_SUBDEV_MAX 16

enum sharc_alsa_state {
	SHARC_ALSA_STOPPED = 0,
	SHARC_ALSA_RUNNING = 1,
	SHARC_ALSA_STOPPING = 2,
	SHARC_ALSA_STARTING = 3,
};

struct sharc_alsa_subdev {
	spinlock_t buf_pos_lock;
	uint32_t state;
	size_t buf_frags;
	size_t buf_frag_pos;
	uint32_t buf_id;
	struct snd_pcm_substream *substream;
	struct wait_queue_head pending_stop_event;
};

struct sharc_alsa_card_data {
	struct device *dev;
	struct rpmsg_device *rpdev;
	struct icap_instance icap;

	u32 subdev_num;
	struct sharc_alsa_subdev subdevs[SHARC_ALSA_SUBDEV_MAX];

	u32 card_id;
	char card_name[64];
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_links[SHARC_ALSA_SUBDEV_MAX];

	struct snd_soc_dai_link_component cpu_links[SHARC_ALSA_SUBDEV_MAX];
	struct snd_soc_dai_link_component codec_links[SHARC_ALSA_SUBDEV_MAX];
	struct snd_soc_dai_link_component platform_links[SHARC_ALSA_SUBDEV_MAX];

	struct platform_device *asoc_cpu_devs[SHARC_ALSA_SUBDEV_MAX];
	struct platform_device *asoc_codec_devs[SHARC_ALSA_SUBDEV_MAX];
	struct platform_device *asoc_platform_devs[SHARC_ALSA_SUBDEV_MAX];

	u32 cpu_num;
	u32 codec_num;
	u32 platform_num;

	struct work_struct delayed_probe_work;
	struct work_struct send_start_work;
	struct work_struct send_stop_work;

	spinlock_t start_stop_spinlock;
	u32 stopping;
	u32 starting;

	int card_registered;
};

struct sharc_alsa_component_data {
	char dai_driver_name[64];
	struct snd_soc_dai_driver dai_driver;
	char component_driver_name[64];
	struct snd_soc_component_driver component_driver;
};

#define SHARC_ALSA_RATES	SNDRV_PCM_RATE_8000_192000
#define SHARC_ALSA_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE | \
			SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static const struct snd_pcm_hardware sharc_alsa_pcm_params = {
	.info =	SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats =	SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 0x10000,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/32,
	.buffer_bytes_max	= 0x20000, /* 128 kbytes */
	.fifo_size			= 16,
};

static int sharc_alsa_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sharc_alsa_card_data *sharc_alsa = snd_soc_card_get_drvdata(card);
	union icap_remote_addr src_addr;
	int ret;

	src_addr.rpmsg_addr = src;
	ret = icap_parse_msg(&sharc_alsa->icap, &src_addr, data, len);
	if (ret && (ret != -ICAP_ERROR_INIT)) {
		if (ret == -ICAP_ERROR_TIMEOUT) {
			dev_notice_ratelimited(&rpdev->dev, "ICAP late response\n");
		} else {
			dev_err_ratelimited(&rpdev->dev, "ICAP parse msg error: %d\n", ret);
		}
	}
	return 0;
}

static int sharc_alsa_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	size_t size = params_buffer_bytes(params);
	snd_pcm_lib_malloc_pages(substream, size);
	return 0;
}


static int sharc_alsa_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_lib_free_pages(substream);
	return 0;
}


static snd_pcm_uframes_t sharc_alsa_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sharc_alsa_card_data *sharc_alsa = runtime->private_data;
	int subdev_id = substream->pcm->device;
	struct sharc_alsa_subdev *subdev = &sharc_alsa->subdevs[subdev_id];
	unsigned long flags;
	unsigned int period;
	snd_pcm_uframes_t frames;

	spin_lock_irqsave(&subdev->buf_pos_lock, flags);
	period = subdev->buf_frag_pos;
	spin_unlock_irqrestore(&subdev->buf_pos_lock, flags);

	frames = period * runtime->period_size;
	return frames;
}

static int sharc_alsa_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sharc_alsa_card_data *sharc_alsa = snd_soc_card_get_drvdata(rtd->card);

	snd_soc_set_runtime_hwparams(substream, &sharc_alsa_pcm_params);

	runtime->private_data = sharc_alsa;
	return 0;
}

static int sharc_alsa_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sharc_alsa_card_data *sharc_alsa = runtime->private_data;
	struct icap_instance *icap = &sharc_alsa->icap;
	int period_bytes = frames_to_bytes(runtime, runtime->period_size);
	struct icap_buf_descriptor icap_buf;
	int subdev_id = substream->pcm->device;
	struct sharc_alsa_subdev *subdev = &sharc_alsa->subdevs[subdev_id];
	int ret = 0;

	ret = wait_event_interruptible(subdev->pending_stop_event, subdev->state == SHARC_ALSA_STOPPED);
	if (ret) {
		return ret;
	}

	subdev->substream = substream;
	subdev->buf_frag_pos = 0;
	subdev->buf_frags = runtime->periods;

	icap_buf.subdev_id = (uint32_t)subdev_id;
	icap_buf.buf = (uint64_t)runtime->dma_addr;
	icap_buf.buf_size = runtime->periods * period_bytes;
	icap_buf.type = ICAP_BUF_CIRCURAL;
	icap_buf.gap_size = 0; // continous circural
	icap_buf.frag_size = period_bytes;
	icap_buf.channels = runtime->channels;
	icap_buf.format = runtime->format;
	icap_buf.rate = runtime->rate;
	icap_buf.report_frags = 1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		snprintf(icap_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-playback-%d", sharc_alsa->card_name, subdev_id);

		if (subdev->buf_id != -1) {
			ret = icap_remove_src(icap, subdev->buf_id);
			if (ret) {
				return ret;
			}
			subdev->buf_id = -1;
		}
		ret = icap_add_src(icap, &icap_buf);
		if (ret < 0) {
			return ret;
		}
		subdev->buf_id = (uint32_t)ret;

	} else {

		snprintf(icap_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-capture-%d", sharc_alsa->card_name, subdev_id);

		if (subdev->buf_id != -1) {
			ret = icap_remove_dst(icap, subdev->buf_id);
			if (ret) {
				return ret;
			}
			subdev->buf_id = -1;
		}
		ret = icap_add_dst(icap, &icap_buf);
		if (ret < 0) {
			return ret;
		}
		subdev->buf_id = (uint32_t)ret;
	}

	return ret;
}

static int sharc_alsa_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sharc_alsa_card_data *sharc_alsa = runtime->private_data;
	int subdev_id = substream->pcm->device;
	struct sharc_alsa_subdev *sharc_alsa_subdev = &sharc_alsa->subdevs[subdev_id];
	unsigned long flags;

	spin_lock_irqsave(&sharc_alsa->start_stop_spinlock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		sharc_alsa_subdev->state = SHARC_ALSA_STARTING;
		sharc_alsa->starting = 1;
		queue_work(system_highpri_wq, &sharc_alsa->send_start_work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		sharc_alsa_subdev->state = SHARC_ALSA_STOPPING;
		sharc_alsa->stopping = 1;
		queue_work(system_highpri_wq, &sharc_alsa->send_stop_work);
		break;
	}

	spin_unlock_irqrestore(&sharc_alsa->start_stop_spinlock, flags);

	return 0;
}

static const struct snd_pcm_ops sharc_alsa_pcm_ops = {
	.open		= sharc_alsa_pcm_open,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sharc_alsa_pcm_hw_params,
	.hw_free	= sharc_alsa_pcm_hw_free,
	.prepare	= sharc_alsa_pcm_prepare,
	.trigger	= sharc_alsa_pcm_trigger,
	.pointer	= sharc_alsa_pcm_pointer,
};

static int sharc_alsa_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	size_t size = sharc_alsa_pcm_params.buffer_bytes_max;/* 128KiB */
	int ret = 0;
	struct snd_pcm_substream *substream;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
				SNDRV_DMA_TYPE_DEV, card->dev, size, size);
	return 0;
}

static int32_t sharc_alsa_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *buf_frags)
{
	struct sharc_alsa_card_data *sharc_alsa = (struct sharc_alsa_card_data *)icap->priv;
	struct sharc_alsa_subdev *subdev;
	int i;
	unsigned long flags;

	for (i = 0; i < sharc_alsa->subdev_num; i++){
		subdev = &sharc_alsa->subdevs[i];
		if (subdev->buf_id == buf_frags->buf_id) {
			spin_lock_irqsave(&subdev->buf_pos_lock, flags);
			subdev->buf_frag_pos += buf_frags->frags;
			if(subdev->buf_frag_pos >= subdev->buf_frags) {
				subdev->buf_frag_pos = subdev->buf_frag_pos - subdev->buf_frags;
			}
			spin_unlock_irqrestore(&subdev->buf_pos_lock, flags);
			snd_pcm_period_elapsed(subdev->substream);
		}
	}
	return 0;
}

struct icap_application_callbacks icap_application_callbacks = {
	.frag_ready = sharc_alsa_frag_ready_cb,
};

static void sharc_alsa_start_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_start_work);
	struct sharc_alsa_subdev *subdev;
	unsigned long flags;
	int ret, i, reschedule;

	spin_lock_irqsave(&sharc_alsa->start_stop_spinlock, flags);

	for (i = 0; i < sharc_alsa->subdev_num; i++){
		subdev = &sharc_alsa->subdevs[i];
		if (subdev->state == SHARC_ALSA_STARTING) {
			subdev->state = SHARC_ALSA_RUNNING;
			spin_unlock_irqrestore(&sharc_alsa->start_stop_spinlock, flags);
			ret = icap_start(&sharc_alsa->icap, i);
			if (ret) {
				dev_err(sharc_alsa->dev, "ICAP subdev%d start error %d", i, ret);
			}
			spin_lock_irqsave(&sharc_alsa->start_stop_spinlock, flags);
		}
	}
	reschedule = sharc_alsa->starting;
	sharc_alsa->starting = 0;

	spin_unlock_irqrestore(&sharc_alsa->start_stop_spinlock, flags);

	if (reschedule) {
		queue_work(system_highpri_wq, &sharc_alsa->send_stop_work);
	}
}

static void sharc_alsa_stop_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_stop_work);
	struct sharc_alsa_subdev *subdev;
	unsigned long flags;
	int ret, i, reschedule;

	spin_lock_irqsave(&sharc_alsa->start_stop_spinlock, flags);

	for (i = 0; i < sharc_alsa->subdev_num; i++){
		subdev = &sharc_alsa->subdevs[i];
		if (subdev->state == SHARC_ALSA_STOPPING) {
			subdev->state = SHARC_ALSA_STOPPED;
			spin_unlock_irqrestore(&sharc_alsa->start_stop_spinlock, flags);
			ret = icap_stop(&sharc_alsa->icap, i);
			if (ret) {
				dev_err(sharc_alsa->dev, "ICAP subdev%d stop error %d", i, ret);
			}
			spin_lock_irqsave(&sharc_alsa->start_stop_spinlock, flags);
			wake_up_interruptible_all(&subdev->pending_stop_event);
		}
	}
	reschedule = sharc_alsa->stopping;
	sharc_alsa->stopping = 0;

	spin_unlock_irqrestore(&sharc_alsa->start_stop_spinlock, flags);

	if (reschedule) {
		queue_work(system_highpri_wq, &sharc_alsa->send_stop_work);
	}
}

static void sharc_alsa_delayed_probe(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, delayed_probe_work);
	struct device *dev = sharc_alsa->dev;
	struct rpmsg_device *rpdev = sharc_alsa->rpdev;
	struct icap_instance *icap = &sharc_alsa->icap;
	const u8 card_id = sharc_alsa->card_id;
	struct sharc_alsa_component_data sharc_alsa_component;
	struct sharc_alsa_component_data *sharc_alsa_component_priv;
	struct icap_subdevice_features features;
	uint32_t subdev_num, i, link_id;
	char link_name[64];
	int32_t ret = 0;

	/* init Inter Core Audio protocol */
	ret = icap_application_init(icap, sharc_alsa->card_name, &icap_application_callbacks, (void*)rpdev, (void*)sharc_alsa);
	if (ret) {
		dev_err(dev, "Failed to init ICAP: %d\n", ret);
		return;
	}

	ret = icap_get_subdevices(icap);
	if (ret < 0) {
		dev_err(dev, "Get subdevices error: %d", ret);
		goto sharc_alsa_delayed_probe_fail;
	}
	subdev_num = (uint32_t)ret;
	subdev_num = subdev_num > SHARC_ALSA_SUBDEV_MAX ? SHARC_ALSA_SUBDEV_MAX : subdev_num;
	sharc_alsa->subdev_num = subdev_num;

	/* Initilize ASoC dai_link, one for each subdevice */
	for (i = 0; i < subdev_num; i++) {

		spin_lock_init(&sharc_alsa->subdevs[i].buf_pos_lock);
		init_waitqueue_head(&sharc_alsa->subdevs[i].pending_stop_event);
		sharc_alsa->subdevs[i].state = SHARC_ALSA_STOPPED;

		link_id = card_id * SHARC_ALSA_SUBDEV_MAX + i;

		ret = icap_get_subdevice_features(icap, i, &features);
		if (ret) {
			dev_err(dev, "Get subdev%d features error: %d", i, ret);
			goto sharc_alsa_delayed_probe_fail;
		}

		if ((features.type != ICAP_DEV_PLAYBACK) && (features.type != ICAP_DEV_RECORD)) {
			dev_err(dev, "Unsupported subdev%d type %d", i, features.type);
			goto sharc_alsa_delayed_probe_fail;
		}

		/* Initilize ASoC common component and dai driver for codec and cpu */
		memset(&sharc_alsa_component, 0, sizeof(sharc_alsa_component));
		if (features.type == ICAP_DEV_PLAYBACK) {
			if (features.src_buf_max < 1) {
				dev_err(dev, "Invalid number of max source bufs for playback subdev%d, need at least 1", i);
				ret = -EINVAL;
				goto sharc_alsa_delayed_probe_fail;
			}
			sharc_alsa_component.dai_driver.playback.stream_name = "Playback";
			sharc_alsa_component.dai_driver.playback.channels_min = features.channels_min;
			sharc_alsa_component.dai_driver.playback.channels_max = features.channels_max;
			sharc_alsa_component.dai_driver.playback.rates = features.rates;
			sharc_alsa_component.dai_driver.playback.formats = features.formats;
		} else {
			/* Capture */
			if (features.dst_buf_max < 1) {
				dev_err(dev, "Invalid number of max dest bufs for capture subdev%d, need at least 1", i);
				ret = -EINVAL;
				goto sharc_alsa_delayed_probe_fail;
			}
			sharc_alsa_component.dai_driver.capture.stream_name = "Capture";
			sharc_alsa_component.dai_driver.capture.channels_min = features.channels_min;
			sharc_alsa_component.dai_driver.capture.channels_max = features.channels_max;
			sharc_alsa_component.dai_driver.capture.rates = features.rates;
			sharc_alsa_component.dai_driver.capture.formats = features.formats;
		}

		sharc_alsa_component.component_driver.idle_bias_on = 1;
		sharc_alsa_component.component_driver.use_pmdown_time = 1;
		sharc_alsa_component.component_driver.endianness = 1;
		sharc_alsa_component.component_driver.non_legacy_dai_naming = 1;

		/* Initilize ASoC codec component and dai driver one for each ICAP subdevice */
		sprintf(sharc_alsa_component.dai_driver_name, "sharc-alsa-codec-dai_%d", link_id);
		sharc_alsa_component.dai_driver.name = sharc_alsa_component.dai_driver_name;
		sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-codec_%d", link_id);
		sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;

		sharc_alsa->asoc_codec_devs[sharc_alsa->codec_num] = platform_device_register_data(
			dev, "sharc-alsa-codec", link_id, &sharc_alsa_component, sizeof(sharc_alsa_component));
		if (IS_ERR(sharc_alsa->asoc_codec_devs[sharc_alsa->codec_num])){
			dev_err(dev, "Failed to register codec component\n");
			ret = PTR_ERR(sharc_alsa->asoc_codec_devs[sharc_alsa->codec_num]);
			goto sharc_alsa_delayed_probe_fail;
		}
		sharc_alsa->codec_num++;

		/* Initilize ASoC cpu component and dai drivers */
		sprintf(sharc_alsa_component.dai_driver_name, "sharc-alsa-cpu-dai_%d", link_id);
		sharc_alsa_component.dai_driver.name = sharc_alsa_component.dai_driver_name;
		sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-cpu_%d", link_id);
		sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;

		sharc_alsa->asoc_cpu_devs[sharc_alsa->cpu_num] = platform_device_register_data(
			dev, "sharc-alsa-cpu", link_id, &sharc_alsa_component, sizeof(sharc_alsa_component));
		if (IS_ERR(sharc_alsa->asoc_cpu_devs[sharc_alsa->cpu_num])){
			dev_err(dev, "Failed to register cpu component\n");
			ret = PTR_ERR(sharc_alsa->asoc_cpu_devs[sharc_alsa->cpu_num]);
			goto sharc_alsa_delayed_probe_fail;
		}
		sharc_alsa->cpu_num++;


		/* Initilize ASoC platfrom driver */
		memset(&sharc_alsa_component, 0, sizeof(sharc_alsa_component));
		sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-platform_%d", link_id);
		sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;
		sharc_alsa_component.component_driver.pcm_new = sharc_alsa_pcm_new;
		sharc_alsa_component.component_driver.ops = &sharc_alsa_pcm_ops;

		sharc_alsa->asoc_platform_devs[sharc_alsa->platform_num] = platform_device_register_data(
			dev, "sharc-alsa-platform", link_id, &sharc_alsa_component, sizeof(sharc_alsa_component));
		if (IS_ERR(sharc_alsa->asoc_platform_devs[sharc_alsa->platform_num])){
			dev_err(dev, "Failed to register platform component\n");
			ret = PTR_ERR(sharc_alsa->asoc_platform_devs[sharc_alsa->platform_num]);
			goto sharc_alsa_delayed_probe_fail;
		}
		sharc_alsa->platform_num++;

		/* Initilize ASoC links */
		sharc_alsa_component_priv = (struct sharc_alsa_component_data *) dev_get_platdata(&sharc_alsa->asoc_codec_devs[i]->dev);
		sharc_alsa->codec_links[i].of_node = NULL;
		sharc_alsa->codec_links[i].name = dev_name(&sharc_alsa->asoc_codec_devs[i]->dev);
		sharc_alsa->codec_links[i].dai_name = sharc_alsa_component_priv->dai_driver_name;

		sharc_alsa_component_priv = (struct sharc_alsa_component_data *) dev_get_platdata(&sharc_alsa->asoc_cpu_devs[i]->dev);
		sharc_alsa->cpu_links[i].of_node = NULL;
		sharc_alsa->cpu_links[i].name = dev_name(&sharc_alsa->asoc_cpu_devs[i]->dev);
		sharc_alsa->cpu_links[i].dai_name = sharc_alsa_component_priv->dai_driver_name;

		sharc_alsa->platform_links[i].of_node = NULL;
		sharc_alsa->platform_links[i].name = dev_name(&sharc_alsa->asoc_platform_devs[i]->dev);
		sharc_alsa->platform_links[i].dai_name = NULL;

		if (features.type == ICAP_DEV_PLAYBACK) {
			sharc_alsa->dai_links[i].playback_only = 1;
			snprintf(link_name, 64, "sharc-alsa-playback-%d", link_id);
		} else {
			/* Capture */
			sharc_alsa->dai_links[i].capture_only = 1;
			snprintf(link_name, 64, "sharc-alsa-capture-%d", link_id);
		}
		sharc_alsa->dai_links[i].name = devm_kstrdup(dev, link_name, GFP_KERNEL);
		sharc_alsa->dai_links[i].stream_name = sharc_alsa->dai_links[i].name;
		sharc_alsa->dai_links[i].cpus = &sharc_alsa->cpu_links[i];
		sharc_alsa->dai_links[i].num_cpus = 1;
		sharc_alsa->dai_links[i].codecs = &sharc_alsa->codec_links[i];
		sharc_alsa->dai_links[i].num_codecs = 1;
		sharc_alsa->dai_links[i].platforms = &sharc_alsa->platform_links[i];
		sharc_alsa->dai_links[i].num_platforms = 1;
		sharc_alsa->dai_links[i].init = NULL;
		sharc_alsa->dai_links[i].ops = NULL;
	}

	/* Initilize ASoC card */
	sharc_alsa->card.name = sharc_alsa->card_name;
	sharc_alsa->card.owner		= THIS_MODULE;
	sharc_alsa->card.dev		= dev;
	sharc_alsa->card.probe		= NULL;
	sharc_alsa->card.dai_link	= sharc_alsa->dai_links;
	sharc_alsa->card.num_links	= subdev_num;

	ret = snd_soc_register_card(&sharc_alsa->card);
	if (ret < 0)
		goto sharc_alsa_delayed_probe_fail;
	sharc_alsa->card_registered = 1;

	dev_info(dev, "sharc-alsa card probed for rpmsg endpoint addr: 0x%03x\n", rpdev->dst);

	return;

sharc_alsa_delayed_probe_fail:
	for (i = 0; i < sharc_alsa->codec_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_codec_devs[i]);
	}
	for (i = 0; i < sharc_alsa->cpu_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_cpu_devs[i]);
	}
	for (i = 0; i < sharc_alsa->platform_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_platform_devs[i]);
	}
	sharc_alsa->codec_num = 0;
	sharc_alsa->cpu_num = 0;
	sharc_alsa->platform_num = 0;

	icap_application_deinit(&sharc_alsa->icap);
	return;
}

static int sharc_alsa_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct sharc_alsa_card_data *sharc_alsa;
	int i;
	int ret = 0;

	sharc_alsa = devm_kzalloc(dev, sizeof(struct sharc_alsa_card_data), GFP_KERNEL);
	if (!sharc_alsa) {
		return -ENOMEM;
	}

	sharc_alsa->card_id = rpdev->dst;
	sprintf(sharc_alsa->card_name, "sharc-alsa-card_%d", sharc_alsa->card_id);

	sharc_alsa->rpdev = rpdev;
	sharc_alsa->dev = dev;
	INIT_WORK(&sharc_alsa->delayed_probe_work, sharc_alsa_delayed_probe);
	INIT_WORK(&sharc_alsa->send_start_work, sharc_alsa_start_func);
	INIT_WORK(&sharc_alsa->send_stop_work, sharc_alsa_stop_func);

	for (i = 0; i < SHARC_ALSA_SUBDEV_MAX; i++){
		sharc_alsa->subdevs[i].buf_id = -1;
	}

	/*
	 * The snd_soc_register_card sets the driver_data in `struct device` for its own use
	 * Use drvdata of the `struct snd_soc_card` to keep the sharc_alsa pointer, which is needed in remove.
	 */
	sharc_alsa->card.dev = dev;
	dev_set_drvdata(dev, &sharc_alsa->card);
	snd_soc_card_set_drvdata(&sharc_alsa->card, sharc_alsa);

	/*
	 * This probe function is in interrupt context (rpmsg handling)
	 * Can't wait for ICAP response here, do the rest of probe in the system_wq workqueue.
	 */
	ret = queue_work(system_highpri_wq, &sharc_alsa->delayed_probe_work);

	return !ret;
}

static void sharc_alsa_remove(struct rpmsg_device *rpdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sharc_alsa_card_data *sharc_alsa = snd_soc_card_get_drvdata(card);
	int32_t ret, i;

	/* Cancel probe work */
	cancel_work_sync(&sharc_alsa->delayed_probe_work);

	if (sharc_alsa->card_registered) {
		snd_soc_unregister_card(&sharc_alsa->card);
	}

	for (i = 0; i < sharc_alsa->codec_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_codec_devs[i]);
	}
	for (i = 0; i < sharc_alsa->cpu_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_cpu_devs[i]);
	}
	for (i = 0; i < sharc_alsa->platform_num; i++) {
		platform_device_unregister(sharc_alsa->asoc_platform_devs[i]);
	}

	/* Cancel all other works */
	cancel_work_sync(&sharc_alsa->send_start_work);
	cancel_work_sync(&sharc_alsa->send_stop_work);

	ret = icap_application_deinit(&sharc_alsa->icap);
	if (ret) {
		dev_err(&rpdev->dev, "ICAP deinit failed %d\n", ret);
	}

	dev_err(&rpdev->dev, "sharc-alsa card removed for rpmsg endpoint addr: 0x%03x\n", rpdev->dst);
}

static int sharc_alsa_component_probe(struct platform_device *pdev)
{
	struct sharc_alsa_component_data *sharc_alsa_component = (struct sharc_alsa_component_data *) dev_get_platdata(&pdev->dev);
	struct snd_soc_dai_driver *dai_drv;
	int dai_num;

	if (sharc_alsa_component->dai_driver.name){
		dai_drv = &sharc_alsa_component->dai_driver;
		dai_num = 1;
	}else{
		dai_drv = NULL;
		dai_num = 0;
	}
	return devm_snd_soc_register_component(&pdev->dev, &sharc_alsa_component->component_driver, dai_drv, dai_num);
}

static struct rpmsg_device_id sharc_alsa_id_table[] = {
	{ .name = "sharc-alsa" },
	{ },
};
static struct rpmsg_driver sharc_alsa_rpmsg_driver = {
	.drv.name  = KBUILD_MODNAME,
	.drv.owner = THIS_MODULE,
	.id_table  = sharc_alsa_id_table,
	.probe     = sharc_alsa_probe,
	.callback  = sharc_alsa_rpmsg_cb,
	.remove    = sharc_alsa_remove,
};

static struct platform_driver sharc_alsa_cpu_driver = {
	.driver = {
		.name = "sharc-alsa-cpu",
	},
	.probe = sharc_alsa_component_probe,
};

static struct platform_driver sharc_alsa_codec_driver = {
	.driver = {
		.name = "sharc-alsa-codec",
	},
	.probe = sharc_alsa_component_probe,
};

static struct platform_driver sharc_alsa_platform_driver = {
	.driver = {
		.name = "sharc-alsa-platform",
	},
	.probe = sharc_alsa_component_probe,
};

static int sharc_alsa_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&sharc_alsa_cpu_driver);
	if (ret != 0){
		pr_err("sharc_alsa: failed to register cpu driver\n");
		goto fail_cpu_driver;
	}

	ret = platform_driver_register(&sharc_alsa_codec_driver);
	if (ret != 0){
		pr_err("sharc_alsa: failed to register codec driver\n");
		goto fail_codec_driver;
	}

	ret = platform_driver_register(&sharc_alsa_platform_driver);
	if (ret != 0){
		pr_err("sharc_alsa: failed to register platform driver\n");
		goto fail_platform_driver;
	}

	ret = register_rpmsg_driver(&sharc_alsa_rpmsg_driver);
	if (ret < 0) {
		pr_err("sharc_alsa: failed to register rpmsg driver\n");
		goto fail_rpmsg_driver;
	}

	return 0;

fail_rpmsg_driver:
	platform_driver_unregister(&sharc_alsa_platform_driver);
fail_platform_driver:
	platform_driver_unregister(&sharc_alsa_codec_driver);
fail_codec_driver:
	platform_driver_unregister(&sharc_alsa_cpu_driver);
fail_cpu_driver:
	return ret;
}
module_init(sharc_alsa_driver_init);

static void sharc_alsa_driver_exit(void)
{
	unregister_rpmsg_driver(&sharc_alsa_rpmsg_driver);
	platform_driver_unregister(&sharc_alsa_platform_driver);
	platform_driver_unregister(&sharc_alsa_codec_driver);
	platform_driver_unregister(&sharc_alsa_cpu_driver);
}
module_exit(sharc_alsa_driver_exit);

MODULE_DESCRIPTION("Analog Devices SHARC-ALSA ASoC card driver");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("sharc-alsa");
