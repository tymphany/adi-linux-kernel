/* linux/arch/arm/mach-sc57x/clock.c
 *
 * clock support for ADI processor
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <asm/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/mutex.h>
#include <mach/clkdev.h>
#include <mach/hardware.h>
#include <mach/sc57x.h>

#define NEEDS_INITIALIZATION     BIT(2)
#define CGU0_CTL_DF (1 << 0)

#define CGU0_CTL_MSEL_SHIFT 8
#define CGU0_CTL_MSEL_MASK (0x7f << 8)

#define CGU0_STAT_PLLEN (1 << 0)
#define CGU0_STAT_PLLBP (1 << 1)
#define CGU0_STAT_PLLLK (1 << 2)
#define CGU0_STAT_CLKSALGN (1 << 3)
#define CGU0_STAT_CCBF0 (1 << 4)
#define CGU0_STAT_CCBF1 (1 << 5)
#define CGU0_STAT_SCBF0 (1 << 6)
#define CGU0_STAT_SCBF1 (1 << 7)
#define CGU0_STAT_DCBF (1 << 8)
#define CGU0_STAT_OCBF (1 << 9)
#define CGU0_STAT_ADDRERR (1 << 16)
#define CGU0_STAT_LWERR (1 << 17)
#define CGU0_STAT_DIVERR (1 << 18)
#define CGU0_STAT_WDFMSERR (1 << 19)
#define CGU0_STAT_WDIVERR (1 << 20)
#define CGU0_STAT_PLOCKERR (1 << 21)

#define CGU0_DIV_CSEL_SHIFT 0
#define CGU0_DIV_CSEL_MASK 0x0000001F
#define CGU0_DIV_S0SEL_SHIFT 5
#define CGU0_DIV_S0SEL_MASK (0x7 << CGU0_DIV_S0SEL_SHIFT)
#define CGU0_DIV_SYSSEL_SHIFT 8
#define CGU0_DIV_SYSSEL_MASK (0x1f << CGU0_DIV_SYSSEL_SHIFT)
#define CGU0_DIV_S1SEL_SHIFT 13
#define CGU0_DIV_S1SEL_MASK (0x7 << CGU0_DIV_S1SEL_SHIFT)
#define CGU0_DIV_DSEL_SHIFT 16
#define CGU0_DIV_DSEL_MASK (0x1f << CGU0_DIV_DSEL_SHIFT)
#define CGU0_DIV_OSEL_SHIFT 22
#define CGU0_DIV_OSEL_MASK (0x7f << CGU0_DIV_OSEL_SHIFT)
#define CGU0_DIV_UPDT (0x1 << 30)

#define CGU_DIV_VAL \
	((CONFIG_CCLK_DIV   << CGU0_DIV_CSEL_SHIFT) | \
	(CONFIG_SCLK_DIV << CGU0_DIV_SYSSEL_SHIFT) | \
	(CONFIG_SCLK0_DIV  << CGU0_DIV_S0SEL_SHIFT) | \
	(CONFIG_SCLK1_DIV  << CGU0_DIV_S1SEL_SHIFT))

int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long ret = 0;

	if (clk->ops && clk->ops->get_rate)
		ret = clk->ops->get_rate(clk);
	clk->rate = ret;

	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	long ret = -EIO;
	return ret;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EIO;
	if (clk->ops && clk->ops->set_rate)
		ret = clk->ops->set_rate(clk, rate);

	if (!ret)
		clk->rate = rate;

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

static void clk_reg_write_mask(void __iomem *reg, uint32_t val, uint32_t mask)
{
	u32 val2;

	val2 = readl(reg);
	val2 &= ~mask;
	val2 |= val;
	writel(val2, reg);
}

int wait_for_pll_align(void)
{
	int i = 10000;
	while (i-- &&
		(readl(__io_address(REG_CGU0_STAT)) & CGU0_STAT_CLKSALGN))
			continue;

	if (readl(__io_address(REG_CGU0_STAT)) & CGU0_STAT_CLKSALGN) {
		pr_crit(KERN_CRIT "fail to align clk\n");
		return -1;
	}

	return 0;
}

/*
 * These are fixed clocks.
 */
void dummy_enable(struct clk *clk)
{
}

unsigned long dummy_get_rate(struct clk *clk)
{
	if (!clk->parent)
		return 0;
	if (clk->rate)
		return clk->rate;
	clk->rate = clk->parent->rate;

	return clk->rate;
}

unsigned long pll_get_rate(struct clk *clk)
{
	u32 df;
	u32 msel;
	u32 ctl = readl(__io_address(REG_CGU0_CTL));
	u32 stat = readl(__io_address(REG_CGU0_STAT));

	if (!clk->parent)
		return 0;

	if (stat & CGU0_STAT_PLLBP)
		return 0;
	msel = (ctl & CGU0_CTL_MSEL_MASK) >> CGU0_CTL_MSEL_SHIFT;
	df = (ctl &  CGU0_CTL_DF);

	return clk->parent->rate / (df + 1) * msel;
}

unsigned long pll_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div;
	div = rate / clk->parent->rate;
	return clk->parent->rate * div;
}

int pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 msel;
	u32 stat = readl(__io_address(REG_CGU0_STAT));

	if (!(stat & CGU0_STAT_PLLEN))
		return -EBUSY;
	if (!(stat & CGU0_STAT_PLLLK))
		return -EBUSY;
	if (wait_for_pll_align())
		return -EBUSY;
	msel = rate / clk->parent->rate / 2;
	clk_reg_write_mask(__io_address(REG_CGU0_CTL),
			msel << CGU0_CTL_MSEL_SHIFT,
			CGU0_CTL_MSEL_MASK);
	clk->rate = rate;
	return 0;
}

unsigned long sys_clk_get_rate(struct clk *clk)
{
	u32 div = readl(__io_address(REG_CGU0_DIV));

	div = (div & clk->mask) >> clk->shift;
	if (div == 0)
		div = clk->mask - 1;

	return clk_get_rate(clk->parent) / div;
}

int sys_clk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 csel;
	u32 stat = readl(__io_address(REG_CGU0_STAT));

	if (!(stat & CGU0_STAT_PLLEN))
		return -EBUSY;
	if (!(stat & CGU0_STAT_PLLLK))
		return -EBUSY;
	if (wait_for_pll_align())
		return -EBUSY;

	if (clk->parent->rate == 0)
		return -EINVAL;

	csel = clk->parent->rate / rate;

	pr_debug("%s rate %ld csel %x\n", clk->name, rate, csel);

	clk_reg_write_mask(__io_address(REG_CGU0_DIV),
			csel << clk->shift,
			clk->mask);

	return 0;
}

static unsigned long msi_get_rate(struct clk *clk)
{
	if (clk->rate)
		return clk->rate;
	if (!clk->parent)
		return 0;
	clk->rate = clk->parent->rate / 2;

	return clk->rate;
}

static struct clk_ops dummy_clk_ops = {
	.enable = dummy_enable,
	.get_rate = dummy_get_rate,
};

static struct clk_ops pll_ops = {
	.enable = dummy_enable,
	.get_rate = pll_get_rate,
	.set_rate = pll_set_rate,
};

static struct clk_ops sys_clk_ops = {
	.enable = dummy_enable,
	.get_rate = sys_clk_get_rate,
	.set_rate = sys_clk_set_rate,
};

static struct clk_ops msi_clk_ops = {
	.enable = dummy_enable,
	.get_rate = msi_get_rate,
};

static struct clk vco_clk = {
	.name = "VCO_CLK",
	.rate	= 25000000,
};

static struct clk cgu0_pll_clk = {
	.name = "CGU0_PLL",
	.parent = &vco_clk,
	.ops = &pll_ops,
	.flags = NEEDS_INITIALIZATION,
};

static struct clk cgu1_pll_clk __attribute__ ((unused)) = {
	.name = "CGU1_PLL",
	.parent = &vco_clk,
	.ops = &pll_ops,
	.flags = NEEDS_INITIALIZATION,
};

static struct clk cgu0_cclk = {
	.name = "CGU0_CCLK",
	.parent = &cgu0_pll_clk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_CSEL_MASK,
	.shift      = CGU0_DIV_CSEL_SHIFT,
};

static struct clk cgu0_sysclk = {
	.name = "CGU0_SYSCLK",
	.parent = &cgu0_pll_clk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_SYSSEL_MASK,
	.shift      = CGU0_DIV_SYSSEL_SHIFT,
};

static struct clk cgu0_sys0_clk = {
	.name = "CGU0_SYS0",
	.parent = &cgu0_sysclk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_S0SEL_MASK,
	.shift      = CGU0_DIV_S0SEL_SHIFT,
};

static struct clk cgu0_sys1_clk __attribute__ ((unused)) = {
	.name = "CGU0_SYS1",
	.parent = &cgu0_sysclk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_S1SEL_MASK,
	.shift      = CGU0_DIV_S1SEL_SHIFT,
};

static struct clk cgu0_dclk = {
	.name = "CGU0_DCLK",
	.parent = &cgu0_pll_clk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_DSEL_MASK,
	.shift      = CGU0_DIV_DSEL_SHIFT,
};

static struct clk cgu0_oclk = {
	.name = "CGU0_OCLK",
	.parent = &cgu0_pll_clk,
	.ops = &sys_clk_ops,
	.flags = NEEDS_INITIALIZATION,
	.mask       = CGU0_DIV_OSEL_MASK,
	.shift      = CGU0_DIV_OSEL_SHIFT,
};

static struct clk uartclk = {
	.parent = &cgu0_sys0_clk,
	.ops = &dummy_clk_ops,
};

static struct clk canclk = {
	.parent = &cgu0_sys0_clk,
	.ops = &dummy_clk_ops,
};

static struct clk spiclk = {
	.parent = &cgu0_sys0_clk,
	.ops = &dummy_clk_ops,
};

static struct clk wdtclk = {
	.parent = &cgu0_sys0_clk,
	.ops = &dummy_clk_ops,
};

static struct clk stmmacclk = {
	.parent = &cgu0_sys0_clk,
	.ops = &dummy_clk_ops,
};

static struct clk msiclk = {
	.parent = &cgu0_oclk,
	.ops = &msi_clk_ops,
};

static struct clk_lookup lookups[] = {
	{	/* fpga core clock */
		.con_id		= "vco_clk",
		.clk		= &vco_clk,
	}, {	/* AMBA bus clock */
		.con_id		= "cgu0_pll_clk",
		.clk		= &cgu0_pll_clk,
	}, {
		.con_id		= "cgu0_sysclk",
		.clk		= &cgu0_sysclk,
	}, {
		.con_id		= "cgu0_cclk",
		.clk		= &cgu0_cclk,
	}, {
		.con_id		= "cgu0_sys0_clk",
		.clk		= &cgu0_sys0_clk,
	}, {
		.con_id		= "cgu0_dclk",
		.clk		= &cgu0_dclk,
	}, {
		.con_id		= "cgu0_oclk",
		.clk		= &cgu0_oclk,
	}, {	/* UART0 */
		.con_id		= "adi-uart4",
		.clk		= &uartclk,
	}, {	/* CAN */
		.con_id		= "can",
		.clk		= &canclk,
	}, {
		.con_id		= "spi",
		.clk		= &spiclk,
	}, {	/* Watchdog */
		.con_id		= "adi-watchdog",
		.clk		= &wdtclk,
	}, {	/* MSI biu clock*/
		.con_id		= "biu",
		.clk		= &cgu0_sys0_clk,
	}, {	/* MSI ciu clock*/
		.con_id		= "ciu",
		.clk		= &msiclk,
	}, {
		.con_id		= "stmmaceth",
		.clk		= &stmmacclk,
	}
};

u_long get_sclk(void)
{
	struct clk *clk;
	unsigned long clk_rate = 0;

	clk = clk_get(NULL, "cgu0_sys0_clk");
	if (IS_ERR(clk))
		return 0;

	clk_rate = clk_get_rate(clk);

	clk_put(clk);

	return clk_rate;
}
EXPORT_SYMBOL(get_sclk);

static void dump_init_clock_rate(void)
{
	int i;
	struct clk *clkp;

	pr_info("dump init clock rate\n");
	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkp = lookups[i].clk;
		if (clkp->flags & NEEDS_INITIALIZATION)
			pr_info("%s %ld MHz\n", clkp->name,
				clk_get_rate(clkp) / 1000000);
	}
}

static void init_cgu(u32 cgu_div)
{
	u32 stat;
	u32 div;
	u32 mask;

	stat = readl(__io_address(REG_CGU0_STAT));
	if (!(stat & CGU0_STAT_PLLEN))
		return;
	if (!(stat & CGU0_STAT_PLLLK))
		return;
	if (wait_for_pll_align())
		return;

	mask = CGU0_DIV_CSEL_MASK | CGU0_DIV_SYSSEL_MASK |
		CGU0_DIV_S0SEL_MASK | CGU0_DIV_S1SEL_MASK;
	div = readl(__io_address(REG_CGU0_DIV));
	if ((div & mask) == cgu_div)
		return;
	div &= ~mask;
	writel(div | cgu_div | CGU0_DIV_UPDT, __io_address(REG_CGU0_DIV));
	if (wait_for_pll_align())
		return;
}

void __init sc57x_clock_init(void)
{
	int i;
	struct clk *clkp;

	init_cgu(CGU_DIV_VAL);

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkp = lookups[i].clk;
		if (clkp->flags & NEEDS_INITIALIZATION)
			clk_get_rate(clkp);
	}
	dump_init_clock_rate();
	clkdev_add_table(lookups, ARRAY_SIZE(lookups));
}
