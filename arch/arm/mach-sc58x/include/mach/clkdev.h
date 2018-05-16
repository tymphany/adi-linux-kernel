/*
 * Copyright (C) 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_MACH_CLKDEV_H
#define __ASM_MACH_CLKDEV_H

#include <linux/clk.h>

#ifdef CONFIG_CCLK_DIV_1
# define CONFIG_CCLK_DIV 1
#endif

#ifdef CONFIG_CCLK_DIV_2
# define CONFIG_CCLK_DIV 2
#endif

#ifdef CONFIG_CCLK_DIV_4
# define CONFIG_CCLK_DIV 4
#endif

#ifdef CONFIG_CCLK_DIV_8
# define CONFIG_CCLK_DIV 8
#endif

struct clk_ops {
	void                    (*enable)(struct clk *);
	void                    (*disable)(struct clk *);
	unsigned long           (*get_rate)(struct clk *);
	int                     (*set_rate)(struct clk *, unsigned long);
};

struct clk {
	const char *name;
	const struct clk_ops	*ops;
	struct clk              *parent;
	unsigned long		rate;
	const struct icst_params *params;
	void __iomem		*reg;
	unsigned long		mask;
	unsigned long		shift;
	unsigned long		flags;
};

#define __clk_get(clk) ({ 1; })
#define __clk_put(clk) do { } while (0)

extern u_long get_sclk(void);
extern u_long get_cclk(void);

#endif
