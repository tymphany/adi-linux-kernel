// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef CLK_ADI_CLK_H
#define CLK_ADI_CLK_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct clk *sc59x_cgu_pll(struct device *dev, const char *name,
	const char *parent_name, void __iomem *base, u8 shift, u8 width,
	u32 m_offset, spinlock_t *lock);

#endif
