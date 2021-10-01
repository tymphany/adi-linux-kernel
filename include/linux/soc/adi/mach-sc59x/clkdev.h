/*
 * Copyright (C) 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_MACH_CLKDEV_H
#define __ASM_MACH_CLKDEV_H

#include <linux/clk.h>

#ifdef CONFIG_SC59X_CCLK_DIV_1
# define CONFIG_CCLK_DIV 1
#endif

#ifdef CONFIG_SC59X_CCLK_DIV_2
# define CONFIG_CCLK_DIV 2
#endif

#ifdef CONFIG_SC59X_CCLK_DIV_4
# define CONFIG_CCLK_DIV 4
#endif

#ifdef CONFIG_SC59X_CCLK_DIV_8
# define CONFIG_CCLK_DIV 8
#endif

struct clk_adi {
	const char *name;
	const struct clk_ops_adi	*ops;
	struct clk_adi              *parent;
	unsigned long		rate;
	const struct icst_params *params;
	void __iomem		*reg;
	unsigned long		mask;
	unsigned long		shift;
	unsigned long		flags;
};

struct clk_ops_adi {
	void                    (*enable)(struct clk_adi * a);
	void                    (*disable)(struct clk_adi * a);
	unsigned long           (*get_rate)(struct clk_adi * a);
	unsigned long           (*set_rate)(struct clk_adi * a, unsigned long b);
};

struct clk_lookup_adi {
	const char		*con_id;
	struct clk_adi		*clk_adi;
};

extern u_long get_sclk(void);
extern u_long get_cclk(void);

#endif
