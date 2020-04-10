/*
 * sc5xx clock scaling
 *
 * Copyright 2008-2020 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <mach/clkdev.h>


static struct cpufreq_frequency_table sc5xx_freq_table[] = {
	{
		.frequency = CPUFREQ_TABLE_END,
		.driver_data = 0,
	},
	{
		.frequency = CPUFREQ_TABLE_END,
		.driver_data = 1,
	},
	{
		.frequency = CPUFREQ_TABLE_END,
		.driver_data = 2,
	},
	{
		.frequency = CPUFREQ_TABLE_END,
		.driver_data = 0,
	},
};

static struct sc5xx_dpm_state {
	unsigned int csel; /* system clock divider */
} dpm_state_table[3];


/**************************************************************************/
static void __init sc5xx_init_tables(unsigned long cclk, unsigned long sclk)
{

	unsigned long csel;
	int index;

	csel = 1;

	for (index = 0;  csel <= 3 && index < 3; index++, csel++) {
		sc5xx_freq_table[index].frequency = cclk >> index;
		dpm_state_table[index].csel = csel;

		pr_debug("cpufreq: freq:%d csel:0x%x\n",
			sc5xx_freq_table[index].frequency,
			dpm_state_table[index].csel);
	}
}

unsigned long cpu_get_cclk(void)
{
	struct clk *clk;
	unsigned long rate = 0;

	clk = clk_get(NULL, "cgu0_cclk");
	if (IS_ERR(clk))
		return -ENODEV;

	rate = clk_get_rate(clk);
	clk_put(clk);

	return rate;
}

static unsigned int sc5xx_getfreq_khz(unsigned int cpu)
{
	return cpu_get_cclk() / 1000;
}

unsigned long cpu_set_cclk(int cpu, unsigned long new)
{
	struct clk *clk;
	int ret;

	clk = clk_get(NULL, "cgu0_cclk");
	if (IS_ERR(clk))
		return -ENODEV;

	ret = clk_set_rate(clk, new);
	clk_put(clk);
	return ret;
}

static int sc5xx_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int index)
{
	static unsigned long lpj_ref;
	static unsigned int  lpj_ref_freq;
	unsigned int old_freq, new_freq;
	int ret = 0;

	old_freq = sc5xx_getfreq_khz(0);
	new_freq = sc5xx_freq_table[index].frequency;

	ret = cpu_set_cclk(policy->cpu, new_freq * 1000);
	if (ret != 0) {
		WARN_ONCE(ret, "cpufreq set freq failed %d\n", ret);
		return ret;
	}

	if (!lpj_ref_freq) {
		lpj_ref = loops_per_jiffy;
		lpj_ref_freq = old_freq;
	}
	if (new_freq != old_freq) {
		loops_per_jiffy = cpufreq_scale(lpj_ref,
				lpj_ref_freq, new_freq);
	}

	return ret;
}

static int sc5xx_freq_init(struct cpufreq_policy *policy)
{

	unsigned long cclk, sclk;

	cclk = cpu_get_cclk() / 1000;
	sclk = get_sclk() / 1000;

	sc5xx_init_tables(cclk, sclk);

	policy->cpuinfo.transition_latency = 50000; /* 50us assumed */
	policy->freq_table = sc5xx_freq_table;

	return 0;
}

static struct cpufreq_driver sc5xx_cpufreq_driver = {
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sc5xx_cpufreq_target,
	.get = sc5xx_getfreq_khz,
	.init = sc5xx_freq_init,
	.name = "sc5xx cpufreq",
	.attr = cpufreq_generic_attr,
};

static int __init sc5xx_cpufreq_init(void)
{
	return cpufreq_register_driver(&sc5xx_cpufreq_driver);
}

static void __exit sc5xx_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&sc5xx_cpufreq_driver);
}

MODULE_DESCRIPTION("cpufreq driver for sc5xx");
MODULE_LICENSE("GPL");

module_init(sc5xx_cpufreq_init);
module_exit(sc5xx_cpufreq_exit);
