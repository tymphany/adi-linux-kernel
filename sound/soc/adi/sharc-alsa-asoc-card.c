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

struct sharc_alsa_card_data {
	struct device *dev;
	struct rpmsg_device *rpdev;
	char card_name[64];
	struct icap_instance icap;
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;

	struct snd_soc_dai_link_component cpu_link;
	struct snd_soc_dai_link_component codec_link;
	struct snd_soc_dai_link_component platform_link;

	struct platform_device *asoc_cpu_dev;
	struct platform_device *asoc_codec_dev;
	struct platform_device *asoc_platform_dev;

	struct snd_pcm_substream *tx_substream;
	struct snd_pcm_substream *rx_substream;

	size_t tx_dma_buf_frames;
	size_t rx_dma_buf_frames;
	size_t tx_fragsize;
	size_t rx_fragsize;
	size_t tx_buf_pos;
	size_t rx_buf_pos;
	struct mutex tx_buf_pos_lock;
	struct mutex rx_buf_pos_lock;
	uint32_t tx_buf_id;
	uint32_t rx_buf_id;

	struct work_struct delayed_probe_work;

	struct work_struct send_tx_start_work;
	struct work_struct send_rx_start_work;

	struct work_struct send_tx_stop_work;
	struct work_struct send_rx_stop_work;
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


static int sharc_alsa_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sharc_alsa_card_data *sharc_alsa = snd_soc_card_get_drvdata(card);
	union icap_remote_addr src_addr;
	int ret;

	src_addr.rpmsg_addr = src;
	ret = icap_parse_msg(&sharc_alsa->icap, &src_addr, data, len);
	if (ret) {
		dev_err(&rpdev->dev, "ICAP error: %d\n", ret);
	}
	return ret;
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
	unsigned int period ;
	snd_pcm_uframes_t frames;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		mutex_lock(&sharc_alsa->tx_buf_pos_lock);
		period = sharc_alsa->tx_buf_pos;
		mutex_unlock(&sharc_alsa->tx_buf_pos_lock);
	}else{
		mutex_lock(&sharc_alsa->rx_buf_pos_lock);
		period = sharc_alsa->rx_buf_pos;
		mutex_unlock(&sharc_alsa->rx_buf_pos_lock);
	}

	frames = period * runtime->period_size;
	return frames;
}

static const struct snd_pcm_hardware sharc_alsa_pcm_params = {
	/* Random values to keep userspace happy when checking constraints */
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.buffer_bytes_max	= 128*1024,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,
	.periods_max		= 128,
};

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
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sharc_alsa->tx_substream = substream;

		snprintf(icap_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-playback", sharc_alsa->card_name);
		icap_buf.device_id = -1;
		icap_buf.buf = (uint64_t)runtime->dma_addr;
		icap_buf.buf_size = runtime->periods * period_bytes;
		icap_buf.type = ICAP_BUF_CIRCURAL;
		icap_buf.gap_size = 0; // continous circural
		icap_buf.frag_size = period_bytes;
		icap_buf.channels = runtime->channels;
		icap_buf.format = runtime->format;
		icap_buf.rate = runtime->rate;

		sharc_alsa->tx_dma_buf_frames = runtime->periods;

		if (sharc_alsa->tx_buf_id != -1) {
			ret = icap_remove_playback_src(icap, sharc_alsa->tx_buf_id);
			if (ret) {
				return ret;
			}
			sharc_alsa->tx_buf_id = -1;
		}
		ret = icap_add_playback_src(icap, &icap_buf, &sharc_alsa->tx_buf_id);

	} else {
	}

	return ret;
}

static int sharc_alsa_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sharc_alsa_card_data *sharc_alsa = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ret = queue_work(system_highpri_wq, &sharc_alsa->send_tx_start_work);
		else
			ret = queue_work(system_highpri_wq, &sharc_alsa->send_rx_start_work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ret = queue_work(system_highpri_wq, &sharc_alsa->send_tx_stop_work);
		else
			ret = queue_work(system_highpri_wq, &sharc_alsa->send_rx_stop_work);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
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

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
				SNDRV_DMA_TYPE_DEV, card->dev, size, size);
	return 0;
}

static int32_t sharc_alsa_playback_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *buf_frags)
{
	struct sharc_alsa_card_data *sharc_alsa = (struct sharc_alsa_card_data *)icap->priv;

	mutex_lock(&sharc_alsa->tx_buf_pos_lock);
	sharc_alsa->tx_buf_pos += buf_frags->frags;
	if(sharc_alsa->tx_buf_pos >= sharc_alsa->tx_dma_buf_frames) {
		sharc_alsa->tx_buf_pos = sharc_alsa->tx_buf_pos - sharc_alsa->tx_dma_buf_frames;
	}
	mutex_unlock(&sharc_alsa->tx_buf_pos_lock);

	snd_pcm_period_elapsed(sharc_alsa->tx_substream);
	return 0;
}

struct icap_application_callbacks icap_application_callbacks = {
	.playback_frag_ready = sharc_alsa_playback_frag_ready_cb,
};


static void sharc_alsa_tx_start_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_tx_start_work);
	int ret;
	ret = icap_playback_start(&sharc_alsa->icap);
	if (ret) {
		dev_err(sharc_alsa->dev, "ICAP start error %d", ret);
	}
}

static void sharc_alsa_rx_start_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_rx_start_work);
	int ret;
	ret = icap_record_start(&sharc_alsa->icap);
	if (ret) {
		dev_err(sharc_alsa->dev, "ICAP start record error %d", ret);
	}
}

static void sharc_alsa_tx_stop_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_tx_stop_work);
	int ret;
	ret = icap_playback_stop(&sharc_alsa->icap);
	if (ret) {
		dev_err(sharc_alsa->dev, "ICAP stop playback error %d", ret);
	}
}

static void sharc_alsa_rx_stop_func(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, send_tx_stop_work);
	int ret;
	ret = icap_playback_stop(&sharc_alsa->icap);
	if (ret) {
		dev_err(sharc_alsa->dev, "ICAP stop record error %d", ret);
	}
}

static void sharc_alsa_delayed_probe(struct work_struct *work)
{
	struct sharc_alsa_card_data *sharc_alsa = container_of(work, struct sharc_alsa_card_data, delayed_probe_work);
	struct device *dev = sharc_alsa->dev;
	struct rpmsg_device *rpdev = sharc_alsa->rpdev;
	const u8 card_id = rpdev->dst;
	struct sharc_alsa_component_data sharc_alsa_component;
	struct sharc_alsa_component_data *sharc_alsa_component_priv;
	struct icap_device_features device_features;
	int ret = 0;

	/* init Inter Core Audio protocol */
	ret = icap_application_init(&sharc_alsa->icap, &icap_application_callbacks, (void*)rpdev, (void*)sharc_alsa);
	if (ret) {
		dev_err(dev, "Failed to init ICAP: %d\n", ret);
		return;
	}

	ret = icap_get_device_features(&sharc_alsa->icap, &device_features);
	if (ret) {
		dev_err(dev, "Failed to read ICAP device features: %d\n", ret);
		return;
	}

	/* Initilize ASoC cpu component and dai drivers */
	memset(&sharc_alsa_component, 0, sizeof(sharc_alsa_component));
	sprintf(sharc_alsa_component.dai_driver_name, "sharc-alsa-cpu-dai_%d", card_id);
	sharc_alsa_component.dai_driver.name = sharc_alsa_component.dai_driver_name;
	sharc_alsa_component.dai_driver.playback.stream_name = "Playback";
	sharc_alsa_component.dai_driver.playback.channels_min = device_features.channels_min;
	sharc_alsa_component.dai_driver.playback.channels_max = device_features.channels_max;
	sharc_alsa_component.dai_driver.playback.rates = device_features.rates;
	sharc_alsa_component.dai_driver.playback.formats = device_features.formats;
	sharc_alsa_component.dai_driver.capture.stream_name = "Capture";
	sharc_alsa_component.dai_driver.capture.channels_min = device_features.channels_min;
	sharc_alsa_component.dai_driver.capture.channels_max = device_features.channels_max;
	sharc_alsa_component.dai_driver.capture.rates = device_features.rates;
	sharc_alsa_component.dai_driver.capture.formats = device_features.formats;

	sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-cpu_%d", card_id);
	sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;
	sharc_alsa_component.component_driver.idle_bias_on = 1;
	sharc_alsa_component.component_driver.use_pmdown_time = 1;
	sharc_alsa_component.component_driver.endianness = 1;
	sharc_alsa_component.component_driver.non_legacy_dai_naming = 1;

	sharc_alsa->asoc_cpu_dev = platform_device_register_data(
		dev,"sharc-alsa-cpu", card_id,	&sharc_alsa_component, sizeof(sharc_alsa_component));
	if (IS_ERR(sharc_alsa->asoc_cpu_dev)){
		dev_err(dev, "Failed to register cpu component\n");
		ret = PTR_ERR(sharc_alsa->asoc_cpu_dev);
		goto fail_cpu_dev;
	}

	/* Initilize ASoC codec component and dai drivers */
	memset(&sharc_alsa_component, 0, sizeof(sharc_alsa_component));
	sprintf(sharc_alsa_component.dai_driver_name, "sharc-alsa-codec-dai_%d", card_id);
	sharc_alsa_component.dai_driver.name = sharc_alsa_component.dai_driver_name;
	sharc_alsa_component.dai_driver.playback.stream_name = "Playback";
	sharc_alsa_component.dai_driver.playback.channels_min = device_features.channels_min;
	sharc_alsa_component.dai_driver.playback.channels_max = device_features.channels_max;
	sharc_alsa_component.dai_driver.playback.rates = device_features.rates;
	sharc_alsa_component.dai_driver.playback.formats = device_features.formats;
	sharc_alsa_component.dai_driver.capture.stream_name = "Capture";
	sharc_alsa_component.dai_driver.capture.channels_min = device_features.channels_min;
	sharc_alsa_component.dai_driver.capture.channels_max = device_features.channels_max;
	sharc_alsa_component.dai_driver.capture.rates = device_features.rates;
	sharc_alsa_component.dai_driver.capture.formats = device_features.formats;

	sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-codec_%d", card_id);
	sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;
	sharc_alsa_component.component_driver.idle_bias_on = 1;
	sharc_alsa_component.component_driver.use_pmdown_time = 1;
	sharc_alsa_component.component_driver.endianness = 1;
	sharc_alsa_component.component_driver.non_legacy_dai_naming = 1;

	sharc_alsa->asoc_codec_dev = platform_device_register_data(
		dev,"sharc-alsa-codec", card_id,	&sharc_alsa_component, sizeof(sharc_alsa_component));
	if (IS_ERR(sharc_alsa->asoc_codec_dev)){
		dev_err(dev, "Failed to register codec component\n");
		ret = PTR_ERR(sharc_alsa->asoc_codec_dev);
		goto fail_codec_dev;
	}

	/* Initilize ASoC platfrom driver */
	memset(&sharc_alsa_component, 0, sizeof(sharc_alsa_component));
	sprintf(sharc_alsa_component.component_driver_name, "sharc-alsa-platform_%d", card_id);
	sharc_alsa_component.component_driver.name = sharc_alsa_component.component_driver_name;
	sharc_alsa_component.component_driver.pcm_new = sharc_alsa_pcm_new;
	sharc_alsa_component.component_driver.ops = &sharc_alsa_pcm_ops;

	sharc_alsa->asoc_platform_dev = platform_device_register_data(
		dev,"sharc-alsa-platform", card_id,	&sharc_alsa_component, sizeof(sharc_alsa_component));
	if (IS_ERR(sharc_alsa->asoc_platform_dev)){
		dev_err(dev, "Failed to register platform component\n");
		ret = PTR_ERR(sharc_alsa->asoc_platform_dev);
		goto fail_platform_dev;
	}

	/* Initilize ASoC links */
	sharc_alsa_component_priv = (struct sharc_alsa_component_data *) dev_get_platdata(&sharc_alsa->asoc_cpu_dev->dev);
	sharc_alsa->cpu_link.of_node = NULL;
	sharc_alsa->cpu_link.name = dev_name(&sharc_alsa->asoc_cpu_dev->dev);
	sharc_alsa->cpu_link.dai_name = sharc_alsa_component_priv->dai_driver_name;

	sharc_alsa_component_priv = (struct sharc_alsa_component_data *) dev_get_platdata(&sharc_alsa->asoc_codec_dev->dev);
	sharc_alsa->codec_link.of_node = NULL;
	sharc_alsa->codec_link.name = dev_name(&sharc_alsa->asoc_codec_dev->dev);
	sharc_alsa->codec_link.dai_name = sharc_alsa_component_priv->dai_driver_name;

	sharc_alsa->platform_link.of_node = NULL;
	sharc_alsa->platform_link.name = dev_name(&sharc_alsa->asoc_platform_dev->dev);
	sharc_alsa->platform_link.dai_name = NULL;

	sharc_alsa->dai_link.name = sharc_alsa->card_name;
	sharc_alsa->dai_link.stream_name = sharc_alsa->card_name;
	sharc_alsa->dai_link.cpus = &sharc_alsa->cpu_link;
	sharc_alsa->dai_link.num_cpus = 1;
	sharc_alsa->dai_link.codecs = &sharc_alsa->codec_link;
	sharc_alsa->dai_link.num_codecs = 1;
	sharc_alsa->dai_link.platforms = &sharc_alsa->platform_link;
	sharc_alsa->dai_link.num_platforms = 1;
	sharc_alsa->dai_link.init = NULL;
	sharc_alsa->dai_link.ops = NULL;

	/* Initilize ASoC card */
	sprintf(sharc_alsa->card_name, "sharc-alsa-card_%d", card_id);
	sharc_alsa->card.name = sharc_alsa->card_name;
	sharc_alsa->card.owner		= THIS_MODULE;
	sharc_alsa->card.dev		= dev;
	sharc_alsa->card.probe		= NULL;
	sharc_alsa->card.dai_link	= &sharc_alsa->dai_link;
	sharc_alsa->card.num_links	= 1;

	ret = snd_soc_register_card(&sharc_alsa->card);
	if (ret < 0)
		goto fail_card;

	dev_info(dev, "sharc-alsa card probed for rpmsg endpoint addr: 0x%03x\n", rpdev->dst);

	return;

fail_card:
	platform_device_unregister(sharc_alsa->asoc_platform_dev);
fail_platform_dev:
	platform_device_unregister(sharc_alsa->asoc_codec_dev);
fail_codec_dev:
	platform_device_unregister(sharc_alsa->asoc_cpu_dev);
fail_cpu_dev:
	icap_application_deinit(&sharc_alsa->icap);
	return;
	
}

static int sharc_alsa_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct sharc_alsa_card_data *sharc_alsa;
	int ret = 0;

	sharc_alsa = devm_kzalloc(dev, sizeof(struct sharc_alsa_card_data), GFP_KERNEL);
	if (!sharc_alsa) {
		return -ENOMEM;
	}

	sharc_alsa->rpdev = rpdev;
	sharc_alsa->dev = dev;
	INIT_WORK(&sharc_alsa->delayed_probe_work, sharc_alsa_delayed_probe);
	INIT_WORK(&sharc_alsa->send_tx_start_work, sharc_alsa_tx_start_func);
	INIT_WORK(&sharc_alsa->send_rx_start_work, sharc_alsa_rx_start_func);
	INIT_WORK(&sharc_alsa->send_tx_stop_work, sharc_alsa_tx_stop_func);
	INIT_WORK(&sharc_alsa->send_rx_stop_work, sharc_alsa_rx_stop_func);

	mutex_init(&sharc_alsa->tx_buf_pos_lock);
	mutex_init(&sharc_alsa->rx_buf_pos_lock);

	sharc_alsa->tx_buf_id = -1;
	sharc_alsa->rx_buf_id = -1;

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
	ret = schedule_work(&sharc_alsa->delayed_probe_work);

	return !ret;
}

static void sharc_alsa_remove(struct rpmsg_device *rpdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sharc_alsa_card_data *sharc_alsa = snd_soc_card_get_drvdata(card);

	/* Cancel probe work */
	cancel_work_sync(&sharc_alsa->delayed_probe_work);

	snd_soc_unregister_card(&sharc_alsa->card);
	platform_device_unregister(sharc_alsa->asoc_cpu_dev);
	platform_device_unregister(sharc_alsa->asoc_codec_dev);
	platform_device_unregister(sharc_alsa->asoc_platform_dev);

	/* Cancel all other works */
	cancel_work_sync(&sharc_alsa->send_tx_start_work);
	cancel_work_sync(&sharc_alsa->send_rx_start_work);
	cancel_work_sync(&sharc_alsa->send_tx_stop_work);
	cancel_work_sync(&sharc_alsa->send_rx_stop_work);

	icap_application_deinit(&sharc_alsa->icap);

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
