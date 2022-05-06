// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * clock support for ADI processor
 *
 * Copyright 2018 Analog Devices Inc.
 * Author: Greg Malysa <greg.malysa@timesys.com>
 */

#include <dt-bindings/clock/adi-sc5xx-clock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/spinlock.h>

#include "clk.h"

#define CGU_CTL         0x00 //0x3108D000
#define CGU_PLLCTL      0x04 //0x3108D004
#define CGU_STAT        0x08 //0x3108D008
#define CGU_DIV         0x0C //0x3108D00C
#define CGU_CLKOUTSEL   0x10 //0x3108D010
#define CGU_OSCWDCTL    0x14 //0x3108D014
#define CGU_TSCTL       0x18 //0x3108D018
#define CGU_TSVALUE0    0x1C //0x3108D01C
#define CGU_TSVALUE1    0x20 //0x3108D020
#define CGU_TSCOUNT0    0x24 //0x3108D024
#define CGU_TSCOUNT1    0x28 //0x3108D028
#define CGU_CCBF_DIS    0x2C //0x3108D02C
#define CGU_CCBF_STAT   0x30 //0x3108D030
#define CGU_SCBF_DIS    0x38 //0x3108D038
#define CGU_SCBF_STAT   0x3C //0x3108D03C
#define CGU_REVID       0x48 //0x3108D048

#define CDU_CFG0     0x00 //0x3108F000
#define CDU_CFG1     0x04 //0x3108F004
#define CDU_CFG2     0x08 //0x3108F008
#define CDU_CFG3     0x0C //0x3108F00C
#define CDU_CFG4     0x10 //0x3108F010
#define CDU_CFG5     0x14 //0x3108F014
#define CDU_CFG6     0x18 //0x3108F018
#define CDU_CFG7     0x1C //0x3108F01C
#define CDU_CFG8     0x20 //0x3108F020
#define CDU_CFG9     0x24 //0x3108F024

#define CDU_CLKINSEL 0x44 //0x3108F044

#define CGU_MSEL_SHIFT 8
#define CGU_MSEL_WIDTH 7

#define CDU_MUX_SIZE 4
#define CDU_MUX_SHIFT 1
#define CDU_MUX_WIDTH 2
#define CDU_EN_BIT 0

static DEFINE_SPINLOCK(cdu_lock);

static struct clk *clks[ADSP_SC58X_CLK_END];
static struct clk_onecell_data clk_data;

static const char *cgu1_in_sels[] = {"sys_clkin0", "sys_clkin1"};
static const char *sharc0_sels[] = {"cclk0_0", "sysclk_0", "dummy", "dummy"};
static const char *sharc1_sels[] = {"cclk0_0", "sysclk_0", "dummy", "dummy"};
static const char *arm_sels[] = {"cclk1_0", "sysclk_0", "dummy", "dummy"};
static const char *cdu_ddr_sels[] = {"dclk_0", "dclk_1", "dummy", "dummy"};
static const char *can_sels[] = {"oclk_0", "oclk_1", "dclk_1", "dummy"};
static const char *spdif_sels[] = {"oclk_0", "oclk_1", "dclk_1", "dclk_0"};
static const char *reserved_sels[] = {"sclk0_0", "oclk_0", "dummy", "dummy"};
static const char *gige_sels[] = {"sclk0_0", "sclk1_1", "cclk0_1", "oclk_0"};
static const char *lp_sels[] = {"sclk0_0", "sclk0_1", "cclk1_1", "dclk_1"};
static const char *sdio_sels[] = {"oclk_0_half", "cclk1_1_half", "cclk1_1", "dclk_1"};

/**
 * All CDU clock muxes are the same size
 */
static inline struct clk *cdu_mux(struct device *dev, const char *name,
	void __iomem *reg, const char * const *parents)
{
	return clk_register_mux(dev, name, parents, CDU_MUX_SIZE,
		CLK_SET_RATE_PARENT, reg, CDU_MUX_SHIFT, CDU_MUX_WIDTH, 0,
		&cdu_lock);
}

static inline struct clk *cgu_divider(struct device *dev, const char *name,
	const char *parent, void __iomem *reg, u8 shift, u8 width, u8 extra_flags)
{
	return clk_register_divider(dev, name, parent, CLK_SET_RATE_PARENT,
		reg, shift, width, CLK_DIVIDER_MAX_AT_ZERO | extra_flags, &cdu_lock);
}

static inline struct clk *cdu_gate(struct device *dev, const char *name,
	const char *parent, void __iomem *reg, u32 flags)
{
	return clk_register_gate(dev, name, parent, CLK_SET_RATE_PARENT | flags,
		reg, CDU_EN_BIT, 0, &cdu_lock);
}

static inline struct clk *cgu_gate(struct device *dev, const char *name,
	const char *parent, void __iomem *reg, u8 bit)
{
	return clk_register_gate(dev, name, parent, CLK_SET_RATE_PARENT, reg, bit,
		CLK_GATE_SET_TO_DISABLE, &cdu_lock);
}

static int cdu_check_clocks(struct device *dev, struct clk *clks[], size_t count) {
	size_t i;

	for (i = 0; i < count; ++i) {
		if (IS_ERR(clks[i])) {
			dev_err(dev, "Clock %lu failed to register: %ld\n", i, PTR_ERR(clks[i]));
			return PTR_ERR(clks[i]);
		}
	}

	return 0;
}

static int sc58x_clock_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *cgu0;
	void __iomem *cgu1;
	void __iomem *cdu;
	int ret;
	int i;

	cgu0 = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cgu0)) {
		dev_err(dev, "Unable to remap CGU0 address (resource 0)\n");
		return PTR_ERR(cgu0);
	}

	cgu1 = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(cgu1)) {
		dev_err(dev, "Unable to remap CGU1 address (resource 1)\n");
		return PTR_ERR(cgu1);
	}

	cdu = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(cdu)) {
		dev_err(dev, "Unable to remap CDU address (resource 2)\n");
		return PTR_ERR(cdu);
	}

	// Input clock configuration
	clks[ADSP_SC58X_CLK_DUMMY] = clk_register_fixed_rate(dev, "dummy", NULL, 0, 0);
	clks[ADSP_SC58X_CLK_SYS_CLKIN0] = of_clk_get_by_name(np, "sys_clkin0");
	clks[ADSP_SC58X_CLK_SYS_CLKIN1] = of_clk_get_by_name(np, "sys_clkin1");
	clks[ADSP_SC58X_CLK_CGU1_IN] = clk_register_mux(dev, "cgu1_in_sel",
		cgu1_in_sels, 2, CLK_SET_RATE_PARENT, cdu + CDU_CLKINSEL, 0, 1, 0,
		&cdu_lock);

	// CGU configuration and internal clocks
	clks[ADSP_SC58X_CLK_CGU0_PLL_IN] = clk_register_divider(dev, "cgu0_df",
		"sys_clkin0", CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 0, 1, 0, &cdu_lock);
	clks[ADSP_SC58X_CLK_CGU1_PLL_IN] = clk_register_divider(dev, "cgu1_df",
		"cgu1_in_sel", CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 0, 1, 0, &cdu_lock);

	// VCO output inside PLL
	clks[ADSP_SC58X_CLK_CGU0_VCO_OUT] = sc5xx_cgu_pll(dev, "cgu0_vco", "cgu0_df",
		cgu0 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, &cdu_lock);
	clks[ADSP_SC58X_CLK_CGU1_VCO_OUT] = sc5xx_cgu_pll(dev, "cgu1_vco", "cgu1_df",
		cgu1 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, &cdu_lock);

	// Final PLL output
	clks[ADSP_SC58X_CLK_CGU0_PLLCLK] = clk_register_fixed_factor(dev,
		"cgu0_pllclk", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 1);
	clks[ADSP_SC58X_CLK_CGU1_PLLCLK] = clk_register_fixed_factor(dev,
		"cgu1_pllclk", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 1);

	// Dividers from pll output
	clks[ADSP_SC58X_CLK_CGU0_CDIV] = cgu_divider(dev, "cgu0_cdiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 0, 5, 0);
	clks[ADSP_SC58X_CLK_CGU0_SYSCLK] = cgu_divider(dev, "sysclk_0", "cgu0_pllclk",
		cgu0 + CGU_DIV, 8, 5, 0);
	clks[ADSP_SC58X_CLK_CGU0_DDIV] = cgu_divider(dev, "cgu0_ddiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 16, 5, 0);
	clks[ADSP_SC58X_CLK_CGU0_ODIV] = cgu_divider(dev, "cgu0_odiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 22, 7, 0);
	clks[ADSP_SC58X_CLK_CGU0_S0SELDIV] = cgu_divider(dev, "cgu0_s0seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 5, 3, 0);
	clks[ADSP_SC58X_CLK_CGU0_S1SELDIV] = cgu_divider(dev, "cgu0_s1seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 13, 3, 0);

	clks[ADSP_SC58X_CLK_CGU1_CDIV] = cgu_divider(dev, "cgu1_cdiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 0, 5, 0);
	clks[ADSP_SC58X_CLK_CGU1_SYSCLK] = cgu_divider(dev, "sysclk_1", "cgu1_pllclk",
		cgu1 + CGU_DIV, 8, 5, 0);
	clks[ADSP_SC58X_CLK_CGU1_DDIV] = cgu_divider(dev, "cgu1_ddiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 16, 5, 0);
	clks[ADSP_SC58X_CLK_CGU1_ODIV] = cgu_divider(dev, "cgu1_odiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 22, 7, 0);
	clks[ADSP_SC58X_CLK_CGU1_S0SELDIV] = cgu_divider(dev, "cgu1_s0seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 5, 3, 0);
	clks[ADSP_SC58X_CLK_CGU1_S1SELDIV] = cgu_divider(dev, "cgu1_s1seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 13, 3, 0);

	// Gates to enable CGU outputs
	clks[ADSP_SC58X_CLK_CGU0_CCLK0] = cgu_gate(dev, "cclk0_0", "cgu0_cdiv",
		cgu0 + CGU_CCBF_DIS, 0);
	clks[ADSP_SC58X_CLK_CGU0_CCLK1] = cgu_gate(dev, "cclk1_0", "cgu0_cdiv",
		cgu1 + CGU_CCBF_DIS, 1);
	clks[ADSP_SC58X_CLK_CGU0_OCLK] = cgu_gate(dev, "oclk_0", "cgu0_odiv",
		cgu0 + CGU_SCBF_DIS, 3);
	clks[ADSP_SC58X_CLK_CGU0_DCLK] = cgu_gate(dev, "dclk_0", "cgu0_ddiv",
		cgu0 + CGU_SCBF_DIS, 2);
	clks[ADSP_SC58X_CLK_CGU0_SCLK1] = cgu_gate(dev, "sclk1_0", "cgu0_s1seldiv",
		cgu0 + CGU_SCBF_DIS, 1);
	clks[ADSP_SC58X_CLK_CGU0_SCLK0] = cgu_gate(dev, "sclk0_0", "cgu0_s0seldiv",
		cgu0 + CGU_SCBF_DIS, 0);

	clks[ADSP_SC58X_CLK_CGU1_CCLK0] = cgu_gate(dev, "cclk0_1", "cgu1_cdiv",
		cgu1 + CGU_CCBF_DIS, 0);
	clks[ADSP_SC58X_CLK_CGU1_CCLK1] = cgu_gate(dev, "cclk1_1", "cgu1_cdiv",
		cgu1 + CGU_CCBF_DIS, 1);
	clks[ADSP_SC58X_CLK_CGU1_OCLK] = cgu_gate(dev, "oclk_1", "cgu1_odiv",
		cgu1 + CGU_SCBF_DIS, 3);
	clks[ADSP_SC58X_CLK_CGU1_DCLK] = cgu_gate(dev, "dclk_1", "cgu1_ddiv",
		cgu1 + CGU_SCBF_DIS, 2);
	clks[ADSP_SC58X_CLK_CGU1_SCLK1] = cgu_gate(dev, "sclk1_1", "cgu1_s1seldiv",
		cgu1 + CGU_SCBF_DIS, 1);
	clks[ADSP_SC58X_CLK_CGU1_SCLK0] = cgu_gate(dev, "sclk0_1", "cgu1_s0seldiv",
		cgu1 + CGU_SCBF_DIS, 0);

	// Extra half rate clocks generated in the CDU
	clks[ADSP_SC58X_CLK_OCLK0_HALF] = clk_register_fixed_factor(dev, "oclk_0_half",
		"oclk_0", CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC58X_CLK_CCLK1_1_HALF] = clk_register_fixed_factor(dev, "cclk1_1_half",
		"cclk1_1", CLK_SET_RATE_PARENT, 1, 2);

	// CDU output muxes
	clks[ADSP_SC58X_CLK_SHARC0_SEL] = cdu_mux(dev, "sharc0_sel", cdu + CDU_CFG0,
		sharc0_sels);
	clks[ADSP_SC58X_CLK_SHARC1_SEL] = cdu_mux(dev, "sharc1_sel", cdu + CDU_CFG1,
		sharc1_sels);
	clks[ADSP_SC58X_CLK_ARM_SEL] = cdu_mux(dev, "arm_sel", cdu + CDU_CFG2,
		arm_sels);
	clks[ADSP_SC58X_CLK_CDU_DDR_SEL] = cdu_mux(dev, "cdu_ddr_sel", cdu + CDU_CFG3,
		cdu_ddr_sels);
	clks[ADSP_SC58X_CLK_CAN_SEL] = cdu_mux(dev, "can_sel", cdu + CDU_CFG4,
		can_sels);
	clks[ADSP_SC58X_CLK_SPDIF_SEL] = cdu_mux(dev, "spdif_sel", cdu + CDU_CFG5,
		spdif_sels);
	clks[ADSP_SC58X_CLK_RESERVED_SEL] = cdu_mux(dev, "reserved_sel", cdu + CDU_CFG6,
		reserved_sels);
	clks[ADSP_SC58X_CLK_GIGE_SEL] = cdu_mux(dev, "gige_sel", cdu + CDU_CFG7,
		gige_sels);
	clks[ADSP_SC58X_CLK_LP_SEL] = cdu_mux(dev, "lp_sel", cdu + CDU_CFG8, lp_sels);
	clks[ADSP_SC58X_CLK_SDIO_SEL] = cdu_mux(dev, "sdio_sel", cdu + CDU_CFG9, sdio_sels);

	// CDU output enable gates
	clks[ADSP_SC58X_CLK_SHARC0] = cdu_gate(dev, "sharc0", "sharc0_sel",
		cdu + CDU_CFG0, CLK_IS_CRITICAL);
	clks[ADSP_SC58X_CLK_SHARC1] = cdu_gate(dev, "sharc1", "sharc1_sel",
		cdu + CDU_CFG1, CLK_IS_CRITICAL);
	clks[ADSP_SC58X_CLK_ARM] = cdu_gate(dev, "arm", "arm_sel", cdu + CDU_CFG2,
		CLK_IS_CRITICAL);
	clks[ADSP_SC58X_CLK_CDU_DDR] = cdu_gate(dev, "cdu_ddr", "cdu_ddr_sel",
		cdu + CDU_CFG3, CLK_IS_CRITICAL);
	clks[ADSP_SC58X_CLK_CAN] = cdu_gate(dev, "can", "can_sel", cdu + CDU_CFG4, 0);
	clks[ADSP_SC58X_CLK_SPDIF] = cdu_gate(dev, "spdif", "spdif_sel", cdu + CDU_CFG5,
		0);
	clks[ADSP_SC58X_CLK_RESERVED] = cdu_gate(dev, "reserved", "reserved_sel", cdu + CDU_CFG6, 0);
	clks[ADSP_SC58X_CLK_GIGE] = cdu_gate(dev, "gige", "gige_sel", cdu + CDU_CFG7, 0);
	clks[ADSP_SC58X_CLK_LP] = cdu_gate(dev, "lp", "lp_sel", cdu + CDU_CFG8, 0);
	clks[ADSP_SC58X_CLK_SDIO] = cdu_gate(dev, "sdio", "sdio_sel", cdu + CDU_CFG9, 0);

	ret = cdu_check_clocks(dev, clks, ARRAY_SIZE(clks));
	if (ret)
		goto cleanup;

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (ret < 0) {
		dev_err(dev, "Failed to register SoC clock information\n");
		goto cleanup;
	}

	return 0;

cleanup:
	for (i = 0; i < ARRAY_SIZE(clks); i++)
		clk_unregister(clks[i]);

	return ret;
}

static const struct of_device_id sc58x_clk_of_match[] = {
	{ .compatible = "adi,sc58x-clocks" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc58x_clk_of_match);

static struct platform_driver sc58x_clk_driver = {
	.probe = sc58x_clock_probe,
	.driver = {
		.name = "sc58x-clocks",
		.of_match_table = sc58x_clk_of_match,
	},
};
module_platform_driver(sc58x_clk_driver);
