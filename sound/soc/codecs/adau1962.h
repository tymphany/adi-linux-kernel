/*
 * Analog Devices adau1962 codec driver
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

#ifndef __ADAU1962_H__
#define __ADAU1962_H__

#include <linux/regmap.h>

struct device;

int adau1962_probe(struct device *dev, struct regmap *regmap,
	void (*switch_mode)(struct device *dev));

extern const struct regmap_config adau1962_regmap_config;

enum adau1962_clk_id {
	ADAU1962_SYSCLK,
};

enum adau1962_sysclk_src {
	ADAU1962_SYSCLK_SRC_MCLK,
	ADAU1962_SYSCLK_SRC_LRCLK,
};

#endif
