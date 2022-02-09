// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * CGU PLL driver for ADI SC59X processors
 *
 * (c) Copyright 2022, Analog Devices
 * Author: Greg Malysa <greg.malysa@timesys.com>
 */

#include <asm/io.h>
#include <linux/clk-provider.h>

#include "clk.h"

struct clk_sc59x_cgu_pll {
	struct clk_hw hw;
	struct device *dev;
	void __iomem *base;
	spinlock_t *lock;
	int prepared;
	u32 mask;
	u32 max;
	u32 m_offset;
	u8 shift;
};

struct clk_sc59x_cgu_pll *to_clk_sc59x_cgu_pll(struct clk_hw *hw) {
	return container_of(hw, struct clk_sc59x_cgu_pll, hw);
}

/*
 * For now, prepare/unprepare do nothing because we want to leave the PLLs running
 * but eventually this could be used to bypass or disable the PLLs if desired
 */
static int sc59x_cgu_pll_prepare(struct clk_hw *hw) {
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	pll->prepared = 1;
	return 0;
}

static int sc59x_cgu_pll_is_prepared(struct clk_hw *hw) {
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	return pll->prepared;
}

static void sc59x_cgu_pll_unprepare(struct clk_hw *hw) {
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	pll->prepared = 0;
}

static long sc59x_cgu_pll_round_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long *parent_rate)
{
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	struct clk_hw *parent_hw;
	unsigned long prate = *parent_rate;
	int parent_inc;
	unsigned long m, m2, new_rate, nr2, prate2;

	parent_hw = clk_hw_get_parent(hw);

	m = rate / prate / 2;

	if (m > pll->max) {
		// cannot scale this far, need bigger input
		parent_inc = m / pll->max;
		prate = clk_hw_round_rate(parent_hw, prate * (parent_inc + 1));
	}
	else if (m == 0) {
		dev_err(pll->dev, "Cannot use VCO to reduce parent clock rate, requested %lu, "
			"clamping to %lu\n", rate, prate);
		return prate;
	}

	new_rate = prate * m;

	if (new_rate != rate) {
		// Check if we could get an integer match by halving parent rate since we
		// know at least about the DF bit before the VCO, although we don't know
		// if we're already using it or not
		prate2 = clk_hw_round_rate(parent_hw, prate / 2);
		m2 = rate / prate2;
		nr2 = prate * m2;
		if (m2 <= pll->max && nr2 == rate) {
			m = m2;
			new_rate = nr2;
			prate = prate2;
		}
	}

	*parent_rate = prate;
	return new_rate;
}

static unsigned long sc59x_cgu_pll_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	u32 reg = readl(pll->base);
	u32 m = ((reg & pll->mask) >> pll->shift) + pll->m_offset;

	if (m == 0)
		m = pll->max;
	return parent_rate * m * 2;
}

static int sc59x_cgu_pll_set_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct clk_sc59x_cgu_pll *pll = to_clk_sc59x_cgu_pll(hw);
	u32 m;

	m = (rate / parent_rate / 2) - pll->m_offset;
	if (m >= pll->max)
		m = 0;

	// reminder for implementation: lock around read/modify to control reg
	dev_err(pll->dev, "set_rate not permitted yet, but we would write %d to m\n", m);
	return -ENOENT;
}

static const struct clk_ops clk_sc59x_cgu_pll_ops = {
	.prepare = sc59x_cgu_pll_prepare,
	.unprepare = sc59x_cgu_pll_unprepare,
	.is_prepared = sc59x_cgu_pll_is_prepared,
	.recalc_rate = sc59x_cgu_pll_recalc_rate,
	.round_rate = sc59x_cgu_pll_round_rate,
	.set_rate = sc59x_cgu_pll_set_rate,
};

struct clk *sc59x_cgu_pll(struct device *dev, const char *name,
	const char *parent_name, void __iomem *base, u8 shift, u8 width,
	u32 m_offset, spinlock_t *lock)
{
	struct clk_sc59x_cgu_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = devm_kzalloc(dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &clk_sc59x_cgu_pll_ops;

	pll->base = base;
	pll->hw.init = &init;
	pll->dev = dev;
	pll->lock = lock;
	pll->shift = shift;
	pll->mask = GENMASK(width-1, 0) << shift;
	pll->max = pll->mask + 1;
	pll->m_offset = m_offset;

	clk = clk_register(dev, &pll->hw);
	if (IS_ERR(clk)) {
		dev_err(dev, "Failed to register sc59x_cgu_pll with code %lu\n",
			PTR_ERR(clk));
	}

	return clk;
}

