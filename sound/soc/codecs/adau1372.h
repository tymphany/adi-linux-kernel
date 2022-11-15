/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices adau1372 codec driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Lars-Peter Clausen <lars@metafoo.de>
 */

#ifndef SOUND_SOC_CODECS_ADAU1372_H
#define SOUND_SOC_CODECS_ADAU1372_H

#include <linux/regmap.h>

struct device;

int adau1372_probe(struct device *dev, struct regmap *regmap,
	void (*switch_mode)(struct device *dev));

int adau1372_remove(struct device *dev);

extern const struct regmap_config adau1372_regmap_config;

#endif
