// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * clock support for ADI processor
 *
 * Copyright 2018 Analog Devices Inc.
 * Author: Greg Malysa <greg.malysa@timesys.com>
 */

#include <dt-bindings/clock/adi-sc59x-clock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/spinlock.h>

#include "clk.h"

#define CGU_CTL         0x00
#define CGU_PLLCTL      0x04
#define CGU_STAT        0x08
#define CGU_DIV         0x0C
#define CGU_CLKOUTSEL   0x10
#define CGU_OSCWDCTL    0x14
#define CGU_TSCTL       0x18
#define CGU_TSVALUE0    0x1C
#define CGU_TSVALUE1    0x20
#define CGU_TSCOUNT0    0x24
#define CGU_TSCOUNT1    0x28
#define CGU_CCBF_DIS    0x2C
#define CGU_CCBF_STAT   0x30
#define CGU_SCBF_DIS    0x38
#define CGU_SCBF_STAT   0x3C
#define CGU_DIVEX       0x40
#define CGU_REVID       0x48

#define CDU_CFG0     0x00
#define CDU_CFG1     0x04
#define CDU_CFG2     0x08
#define CDU_CFG3     0x0C
#define CDU_CFG4     0x10
#define CDU_CFG5     0x14
#define CDU_CFG6     0x18
#define CDU_CFG7     0x1C
#define CDU_CFG8     0x20
#define CDU_CFG9     0x24
#define CDU_CFG10    0x28
#define CDU_CFG11    0x2C	// Unused
#define CDU_CFG12    0x30
#define CDU_CFG13    0x34
#define CDU_CFG14    0x38

#define PLL3_OFFSET 0x2c

#define CDU_CLKINSEL 0x44

#define CGU_MSEL_SHIFT 8
#define CGU_MSEL_WIDTH 7

#define PLL3_MSEL_SHIFT 4
#define PLL3_MSEL_WIDTH 7

#define CDU_MUX_SIZE 4
#define CDU_MUX_SHIFT 1
#define CDU_MUX_WIDTH 2
#define CDU_EN_BIT 0

static DEFINE_SPINLOCK(cdu_lock);

static struct clk *clks[ADSP_SC59X_CLK_END];
static struct clk_onecell_data clk_data;

static const char *cgu1_in_sels[] = {"sys_clkin0", "sys_clkin1"};
static const char *cgu0_s1sels[] = {"cgu0_s1seldiv", "cgu0_s1selexdiv"};
static const char *cgu1_s0sels[] = {"cgu1_s0seldiv", "cgu1_s0selexdiv"};
static const char *cgu1_s1sels[] = {"cgu1_s1seldiv", "cgu1_s1selexdiv"};
static const char *sharc0_sels[] = {"cclk0_0", "dummy", "dummy", "dummy"};
static const char *sharc1_sels[] = {"cclk0_0", "dummy", "dummy", "dummy"};
static const char *arm_sels[] = {"dummy", "dummy", "cclk2_0", "cclk2_1"};
static const char *cdu_ddr_sels[] = {"dclk_0", "dclk_1", "dummy", "dummy"};
static const char *can_sels[] = {"dummy", "oclk_1", "dummy", "dummy"};
static const char *spdif_sels[] = {"sclk1_0", "dummy", "dummy", "dummy"};
static const char *spi_sels[] = {"sclk0_0", "oclk_0", "dummy", "dummy"};
static const char *gige_sels[] = {"sclk0_0", "sclk0_1", "dummy", "dummy"};
static const char *lp_sels[] = {"oclk_0", "sclk0_0", "cclk0_1", "dummy"};
static const char *lp_ddr_sels[] = {"oclk_0", "dclk_0", "sysclk_1", "dummy"};
static const char *ospi_refclk_sels[] = {"sysclk_0", "sclk0_0", "sclk1_1", "dummy"};
static const char *trace_sels[] = {"sclk0_0", "dummy", "dummy", "dummy"};
static const char *emmc_sels[] = {"oclk_0", "sclk0_1", "dclk_0_half", "dclk_1_half"};
static const char *emmc_timer_sels[] = {"dummy", "sclk1_1_half", "dummy", "dummy"};

static const char *ddr_sels[] = {"cdu_ddr", "3pll_ddiv"};

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

static int sc59x_clock_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *cgu0;
	void __iomem *cgu1;
	void __iomem *cdu;
	void __iomem *pll3;
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

	pll3 = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(pll3)) {
		dev_err(dev, "Unable to remap PLL3 control register (resource 3)\n");
		return PTR_ERR(pll3);
	}

	// We only access this one register for pll3
	pll3 = pll3 + PLL3_OFFSET;

	// Input clock configuration
	clks[ADSP_SC59X_CLK_DUMMY] = clk_register_fixed_rate(dev, "dummy", NULL, 0, 0);
	clks[ADSP_SC59X_CLK_SYS_CLKIN0] = of_clk_get_by_name(np, "sys_clkin0");
	clks[ADSP_SC59X_CLK_SYS_CLKIN1] = of_clk_get_by_name(np, "sys_clkin1");
	clks[ADSP_SC59X_CLK_CGU1_IN] = clk_register_mux(dev, "cgu1_in_sel",
		cgu1_in_sels, 2, CLK_SET_RATE_PARENT, cdu + CDU_CLKINSEL, 0, 1, 0,
		&cdu_lock);

	// 3rd pll reuses cgu1 clk in selection, feeds directly into 3pll df
	// changing the cgu1 in sel mux will affect 3pll so reuse the same clocks

	// CGU configuration and internal clocks
	clks[ADSP_SC59X_CLK_CGU0_PLL_IN] = clk_register_divider(dev, "cgu0_df",
		"sys_clkin0", CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 0, 1, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_CGU1_PLL_IN] = clk_register_divider(dev, "cgu1_df",
		"cgu1_in_sel", CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 0, 1, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_3PLL_PLL_IN] = clk_register_divider(dev, "3pll_df",
		"cgu1_in_sel", CLK_SET_RATE_PARENT, pll3, 3, 1, 0, &cdu_lock);

	// VCO output inside PLL
	clks[ADSP_SC59X_CLK_CGU0_VCO_OUT] = sc59x_cgu_pll(dev, "cgu0_vco", "cgu0_df",
		cgu0 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_CGU1_VCO_OUT] = sc59x_cgu_pll(dev, "cgu1_vco", "cgu1_df",
		cgu1 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_3PLL_VCO_OUT] = sc59x_cgu_pll(dev, "3pll_vco", "3pll_df",
		pll3, PLL3_MSEL_SHIFT, PLL3_MSEL_WIDTH, 1, &cdu_lock);

	// Final PLL output
	clks[ADSP_SC59X_CLK_CGU0_PLLCLK] = clk_register_fixed_factor(dev,
		"cgu0_pllclk", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC59X_CLK_CGU1_PLLCLK] = clk_register_fixed_factor(dev,
		"cgu1_pllclk", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC59X_CLK_3PLL_PLLCLK] = clk_register_fixed_factor(dev,
		"3pll_pllclk", "3pll_vco", CLK_SET_RATE_PARENT, 1, 2);

	// Dividers from pll output
	clks[ADSP_SC59X_CLK_CGU0_CDIV] = cgu_divider(dev, "cgu0_cdiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 0, 5, 0);
	clks[ADSP_SC59X_CLK_CGU0_SYSCLK] = cgu_divider(dev, "sysclk_0", "cgu0_pllclk",
		cgu0 + CGU_DIV, 8, 5, 0);
	clks[ADSP_SC59X_CLK_CGU0_DDIV] = cgu_divider(dev, "cgu0_ddiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 16, 5, 0);
	clks[ADSP_SC59X_CLK_CGU0_ODIV] = cgu_divider(dev, "cgu0_odiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 22, 7, 0);
	clks[ADSP_SC59X_CLK_CGU0_S0SELDIV] = cgu_divider(dev, "cgu0_s0seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 5, 3, 0);
	clks[ADSP_SC59X_CLK_CGU0_S1SELDIV] = cgu_divider(dev, "cgu0_s1seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 13, 3, 0);
	clks[ADSP_SC59X_CLK_CGU0_S1SELEXDIV] = cgu_divider(dev, "cgu0_s1selexdiv",
		"cgu0_pllclk", cgu0 + CGU_DIVEX, 16, 8, 0);
	clks[ADSP_SC59X_CLK_CGU0_S1SEL] = clk_register_mux(dev, "cgu0_sclk1sel",
		cgu0_s1sels, 2, CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 17, 1, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_CGU0_CCLK2] = clk_register_fixed_factor(dev,
		"cclk2_0", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 3);

	clks[ADSP_SC59X_CLK_CGU1_CDIV] = cgu_divider(dev, "cgu1_cdiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 0, 5, 0);
	clks[ADSP_SC59X_CLK_CGU1_SYSCLK] = cgu_divider(dev, "sysclk_1", "cgu1_pllclk",
		cgu1 + CGU_DIV, 8, 5, 0);
	clks[ADSP_SC59X_CLK_CGU1_DDIV] = cgu_divider(dev, "cgu1_ddiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 16, 5, 0);
	clks[ADSP_SC59X_CLK_CGU1_ODIV] = cgu_divider(dev, "cgu1_odiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 22, 7, 0);
	clks[ADSP_SC59X_CLK_CGU1_S0SELDIV] = cgu_divider(dev, "cgu1_s0seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 5, 3, 0);
	clks[ADSP_SC59X_CLK_CGU1_S1SELDIV] = cgu_divider(dev, "cgu1_s1seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 13, 3, 0);
	clks[ADSP_SC59X_CLK_CGU1_S0SELEXDIV] = cgu_divider(dev, "cgu1_s0selexdiv",
		"cgu1_pllclk", cgu1 + CGU_DIVEX, 0, 8, 0);
	clks[ADSP_SC59X_CLK_CGU1_S1SELEXDIV] = cgu_divider(dev, "cgu1_s1selexdiv",
		"cgu1_pllclk", cgu1 + CGU_DIVEX, 16, 8, 0);
	clks[ADSP_SC59X_CLK_CGU1_S0SEL] = clk_register_mux(dev, "cgu1_sclk0sel",
		cgu1_s0sels, 2, CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 16, 1, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_CGU1_S1SEL] = clk_register_mux(dev, "cgu1_sclk1sel",
		cgu1_s1sels, 2, CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 17, 1, 0, &cdu_lock);
	clks[ADSP_SC59X_CLK_CGU1_CCLK2] = clk_register_fixed_factor(dev,
		"cclk2_1", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 3);

	clks[ADSP_SC59X_CLK_3PLL_DDIV] = clk_register_divider(dev, "3pll_ddiv",
		"3pll_pllclk", CLK_SET_RATE_PARENT, pll3, 12, 5, 0, &cdu_lock);

	// Gates to enable CGU outputs
	clks[ADSP_SC59X_CLK_CGU0_CCLK0] = cgu_gate(dev, "cclk0_0", "cgu0_cdiv",
		cgu0 + CGU_CCBF_DIS, 0);
	clks[ADSP_SC59X_CLK_CGU0_OCLK] = cgu_gate(dev, "oclk_0", "cgu0_odiv",
		cgu0 + CGU_SCBF_DIS, 3);
	clks[ADSP_SC59X_CLK_CGU0_DCLK] = cgu_gate(dev, "dclk_0", "cgu0_ddiv",
		cgu0 + CGU_SCBF_DIS, 2);
	clks[ADSP_SC59X_CLK_CGU0_SCLK1] = cgu_gate(dev, "sclk1_0", "cgu0_sclk1sel",
		cgu0 + CGU_SCBF_DIS, 1);
	clks[ADSP_SC59X_CLK_CGU0_SCLK0] = cgu_gate(dev, "sclk0_0", "cgu0_s0seldiv",
		cgu0 + CGU_SCBF_DIS, 0);

	clks[ADSP_SC59X_CLK_CGU1_CCLK0] = cgu_gate(dev, "cclk0_1", "cgu1_cdiv",
		cgu1 + CGU_CCBF_DIS, 0);
	clks[ADSP_SC59X_CLK_CGU1_OCLK] = cgu_gate(dev, "oclk_1", "cgu1_odiv",
		cgu1 + CGU_SCBF_DIS, 3);
	clks[ADSP_SC59X_CLK_CGU1_DCLK] = cgu_gate(dev, "dclk_1", "cgu1_ddiv",
		cgu1 + CGU_SCBF_DIS, 2);
	clks[ADSP_SC59X_CLK_CGU1_SCLK1] = cgu_gate(dev, "sclk1_1", "cgu1_sclk1sel",
		cgu1 + CGU_SCBF_DIS, 1);
	clks[ADSP_SC59X_CLK_CGU1_SCLK0] = cgu_gate(dev, "sclk0_1", "cgu1_sclk0sel",
		cgu1 + CGU_SCBF_DIS, 0);

	// Extra half rate clocks generated in the CDU
	clks[ADSP_SC59X_CLK_DCLK0_HALF] = clk_register_fixed_factor(dev, "dclk_0_half",
		"dclk_0", CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC59X_CLK_DCLK1_HALF] = clk_register_fixed_factor(dev, "dclk_1_half",
		"dclk_1", CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC59X_CLK_CGU1_SCLK1_HALF] = clk_register_fixed_factor(dev,
		"sclk1_1_half", "sclk1_1", CLK_SET_RATE_PARENT, 1, 2);

	// CDU output muxes
	clks[ADSP_SC59X_CLK_SHARC0_SEL] = cdu_mux(dev, "sharc0_sel", cdu + CDU_CFG0,
		sharc0_sels);
	clks[ADSP_SC59X_CLK_SHARC1_SEL] = cdu_mux(dev, "sharc1_sel", cdu + CDU_CFG1,
		sharc1_sels);
	clks[ADSP_SC59X_CLK_ARM_SEL] = cdu_mux(dev, "arm_sel", cdu + CDU_CFG2,
		arm_sels);
	clks[ADSP_SC59X_CLK_CDU_DDR_SEL] = cdu_mux(dev, "cdu_ddr_sel", cdu + CDU_CFG3,
		cdu_ddr_sels);
	clks[ADSP_SC59X_CLK_CAN_SEL] = cdu_mux(dev, "can_sel", cdu + CDU_CFG4,
		can_sels);
	clks[ADSP_SC59X_CLK_SPDIF_SEL] = cdu_mux(dev, "spdif_sel", cdu + CDU_CFG5,
		spdif_sels);
	clks[ADSP_SC59X_CLK_SPI_SEL] = cdu_mux(dev, "spi_sel", cdu + CDU_CFG6,
		spi_sels);
	clks[ADSP_SC59X_CLK_GIGE_SEL] = cdu_mux(dev, "gige_sel", cdu + CDU_CFG7,
		gige_sels);
	clks[ADSP_SC59X_CLK_LP_SEL] = cdu_mux(dev, "lp_sel", cdu + CDU_CFG8, lp_sels);
	clks[ADSP_SC59X_CLK_LP_DDR_SEL] = cdu_mux(dev, "lp_ddr_sel", cdu + CDU_CFG9,
		lp_ddr_sels);
	clks[ADSP_SC59X_CLK_OSPI_REFCLK_SEL] = cdu_mux(dev, "ospi_refclk_sel",
		cdu + CDU_CFG10, ospi_refclk_sels);
	clks[ADSP_SC59X_CLK_TRACE_SEL] = cdu_mux(dev, "trace_sel", cdu + CDU_CFG12,
		trace_sels);
	clks[ADSP_SC59X_CLK_EMMC_SEL] = cdu_mux(dev, "emmc_sel", cdu + CDU_CFG13,
		emmc_sels);
	clks[ADSP_SC59X_CLK_EMMC_TIMER_QMC_SEL] = cdu_mux(dev, "emmc_timer_qmc_sel",
		cdu + CDU_CFG14, emmc_timer_sels);

	// CDU output enable gates
	clks[ADSP_SC59X_CLK_SHARC0] = cdu_gate(dev, "sharc0", "sharc0_sel",
		cdu + CDU_CFG0, CLK_IS_CRITICAL);
	clks[ADSP_SC59X_CLK_SHARC1] = cdu_gate(dev, "sharc1", "sharc1_sel",
		cdu + CDU_CFG1, CLK_IS_CRITICAL);
	clks[ADSP_SC59X_CLK_ARM] = cdu_gate(dev, "arm", "arm_sel", cdu + CDU_CFG2,
		CLK_IS_CRITICAL);
	clks[ADSP_SC59X_CLK_CDU_DDR] = cdu_gate(dev, "cdu_ddr", "cdu_ddr_sel",
		cdu + CDU_CFG3, 0);
	clks[ADSP_SC59X_CLK_CAN] = cdu_gate(dev, "can", "can_sel", cdu + CDU_CFG4, 0);
	clks[ADSP_SC59X_CLK_SPDIF] = cdu_gate(dev, "spdif", "spdif_sel", cdu + CDU_CFG5,
		0);
	clks[ADSP_SC59X_CLK_SPI] = cdu_gate(dev, "spi", "spi_sel", cdu + CDU_CFG6, 0);
	clks[ADSP_SC59X_CLK_GIGE] = cdu_gate(dev, "gige", "gige_sel", cdu + CDU_CFG7,
		CLK_IS_CRITICAL);
	clks[ADSP_SC59X_CLK_LP] = cdu_gate(dev, "lp", "lp_sel", cdu + CDU_CFG8, 0);
	clks[ADSP_SC59X_CLK_LP_DDR] = cdu_gate(dev, "lp_ddr", "lp_ddr_sel",
		cdu + CDU_CFG9, 0);
	clks[ADSP_SC59X_CLK_OSPI_REFCLK] = cdu_gate(dev, "ospi_refclk", "ospi_refclk_sel",
		cdu + CDU_CFG10, CLK_IS_CRITICAL);
	clks[ADSP_SC59X_CLK_TRACE] = cdu_gate(dev, "trace", "trace_sel", cdu + CDU_CFG12,
		0);
	clks[ADSP_SC59X_CLK_EMMC] = cdu_gate(dev, "emmc", "emmc_sel", cdu + CDU_CFG13, 0);
	clks[ADSP_SC59X_CLK_EMMC_TIMER_QMC] = cdu_gate(dev, "emmc_timer_qmc",
		"emmc_timer_qmc_sel", cdu + CDU_CFG14, 0);

	// Dedicated DDR output mux
	clks[ADSP_SC59X_CLK_DDR] = clk_register_mux(dev, "ddr", ddr_sels, 2,
		CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, pll3, 11, 1, 0, &cdu_lock);

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

static const struct of_device_id sc59x_clk_of_match[] = {
	{ .compatible = "adi,sc59x-clocks" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc59x_clk_of_match);

static struct platform_driver sc59x_clk_driver = {
	.probe = sc59x_clock_probe,
	.driver = {
		.name = "sc59x-clocks",
		.of_match_table = sc59x_clk_of_match,
	},
};
module_platform_driver(sc59x_clk_driver);
