/*
 * sc5xx-asoc-card.c Analog Devices ASoC Machine driver for sc5xx
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
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <mach/dma.h>
#include <mach/portmux.h>

#include "../codecs/adau1962.h"
#include "../codecs/adau1977.h"

static int sc5xx_adau1962_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt, rx_mask = 0;
	unsigned int slot_width = 0;
	int ret, slots = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 1: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 16;
		rx_mask = 0x1;
		break;
	case 4: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 4;
		rx_mask = 0xf;
		break;
	case 8: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 8;
		rx_mask = 0xff;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	return snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
				 slots, slot_width);

}

static const struct snd_soc_ops adau1962_ops = {
	.hw_params = sc5xx_adau1962_hw_params,
};

static int __maybe_unused sc5xx_adau1962_init(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_codec_set_sysclk(rtd->codec, ADAU1962_SYSCLK,
			ADAU1962_SYSCLK_SRC_MCLK, 24576000, SND_SOC_CLOCK_IN);
}

static int sc5xx_adau1979_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret, slots = 0;
	unsigned int slot_width = 0;
	unsigned int fmt, rx_mask = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 1: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 16;
		rx_mask = 0x1;
		break;
	case 4: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 4;
		rx_mask = 0xf;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
		slot_width = 24;
		break;
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
					slots, slot_width);
	return ret;
}

static const struct snd_soc_ops adau1979_ops = {
	.hw_params = sc5xx_adau1979_hw_params,
};

static int __maybe_unused sc5xx_adau1979_init(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_codec_set_sysclk(rtd->codec, ADAU1977_SYSCLK,
			ADAU1977_SYSCLK_SRC_MCLK, 24576000, SND_SOC_CLOCK_IN);
}

/* Digital audio interface glue - connect codec <--> CPU */
static struct snd_soc_dai_link sc5xx_asoc_dai_links[] = {
#ifdef CONFIG_SND_SC5XX_ADAU1962
	{
		.name = "adau1962",
		.stream_name = "ADAU1962",
		.codec_dai_name = "adau1962-hifi",
		.platform_name = "sc5xx-pcm-audio",
		.init = sc5xx_adau1962_init,
		.ops = &adau1962_ops,
	},
#endif
#ifdef CONFIG_SND_SC5XX_ADAU1979
	{
		.name = "adau1979",
		.stream_name = "ADAU1979",
		.codec_dai_name = "adau1977-hifi",
		.platform_name = "sc5xx-pcm-audio",
		.init = sc5xx_adau1979_init,
		.ops = &adau1979_ops,
	},
#endif
};

/* ADI sc5xx audio machine driver */
static struct snd_soc_card sc5xx_asoc_card = {
	.name = "sc5xx-asoc-card",
	.owner = THIS_MODULE,
	.dai_link = sc5xx_asoc_dai_links,
	.num_links = ARRAY_SIZE(sc5xx_asoc_dai_links),
};

static int sc5xx_asoc_probe(struct platform_device *pdev)
{
	int id = 0;
	sc5xx_asoc_card.dev = &pdev->dev;

#ifdef CONFIG_SND_SC5XX_ADAU1962
	sc5xx_asoc_dai_links[id].cpu_of_node =
			of_parse_phandle(pdev->dev.of_node, "adi,cpu-dai", 0);
	sc5xx_asoc_dai_links[id++].codec_of_node =
			of_parse_phandle(pdev->dev.of_node, "adi,codec", 0);
#endif
#ifdef CONFIG_SND_SC5XX_ADAU1979
	sc5xx_asoc_dai_links[id].cpu_of_node =
			of_parse_phandle(pdev->dev.of_node, "adi,cpu-dai", 0);
	sc5xx_asoc_dai_links[id++].codec_of_node =
			of_parse_phandle(pdev->dev.of_node, "adi,codec", 1);
#endif
	return devm_snd_soc_register_card(&pdev->dev, &sc5xx_asoc_card);
}

#ifdef CONFIG_OF
static const struct of_device_id sc5xx_asoc_of_match[] = {
	{ .compatible = "sc5xx,asoc-card" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc5xx_asoc_of_match);
#endif

static struct platform_driver sc5xx_asoc_driver = {
	.driver = {
		.name = "snd-sc5xx",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(sc5xx_asoc_of_match),
	},
	.probe = sc5xx_asoc_probe,
};
module_platform_driver(sc5xx_asoc_driver);

MODULE_DESCRIPTION("ASoC Machine driver for ADI SC5xx based boards");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
