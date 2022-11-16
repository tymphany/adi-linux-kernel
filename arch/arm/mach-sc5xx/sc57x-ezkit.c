// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Machine entries for the sc573 ezkit
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/init.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/siginfo.h>
#include <asm/signal.h>

#include "core.h"

static const char * const sc57x_dt_board_compat[] __initconst = {
	"adi,sc57x",
	NULL
};

static bool first_fault = true;

static int sc57x_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	if (fsr == 0x1c06 && first_fault) {
		first_fault = false;

		/*
		 * These faults with code 0x1c06 happens for no good reason,
		 * possibly left over from the CFE boot loader.
		 */
		pr_warn("External imprecise Data abort at addr=%#lx, fsr=%#x ignored.\n",
				addr, fsr);
		return 0;
	}

	/* Others should cause a fault */
	return 1;
}

#if IS_ENABLED(CONFIG_VIDEO_ADI_CAPTURE)
#include <linux/videodev2.h>
#include <media/adi/adi_capture.h>
#include <media/adi/ppi.h>

#if IS_ENABLED(CONFIG_VIDEO_ADV7842)
#include <media/i2c/adv7842.h>
static struct v4l2_input adv7842_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_IN_CAP_STD,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_IN_CAP_STD,
	},
	{
		.index = 2,
		.name = "Component",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
	{
		.index = 3,
		.name = "VGA",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
	{
		.index = 4,
		.name = "HDMI",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
};

static struct adi_cap_route adv7842_routes[] = {
	{
		.input = ADV7842_SELECT_SDP_CVBS,
		.output = 0,
		.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FLDSEL
				| EPPI_CTL_ACTIVE656),
	},
	{
		.input = ADV7842_SELECT_SDP_YC,
		.output = 0,
	},
	{
		.input = ADV7842_SELECT_VGA_COMP,
		.output = 0,
	},
	{
		.input = ADV7842_SELECT_VGA_RGB,
		.output = 0,
	},
	{
		.input = ADV7842_SELECT_HDMI_PORT_A,
		.output = 1,
		.ppi_control = (EPPI_CTL_SPLTWRD | PACK_EN | DLEN_16
				| EPPI_CTL_FS1HI_FS2HI | EPPI_CTL_POLC2
				| EPPI_CTL_SYNC2 | EPPI_CTL_NON656),
	},
};

static struct adv7842_platform_data adv7842_data = {
	.bus_order = ADV7842_BUS_ORDER_RGB,
	.op_format_mode_sel = ADV7842_OP_FORMAT_MODE0,
	.ain_sel = ADV7842_AIN10_11_12_NC_SYNC_4_1,
	.mode = ADV7842_MODE_SDP,
	.vid_std_select = ADV7842_SDP_VID_STD_CVBS_SD_4x1,
	.i2c_sdp_io = 0x40,
	.i2c_sdp = 0x41,
	.i2c_cp = 0x42,
	.i2c_vdp = 0x43,
	.i2c_afe = 0x44,
	.i2c_hdmi = 0x45,
	.i2c_repeater = 0x46,
	.i2c_edid = 0x47,
	.i2c_infoframe = 0x48,
	.i2c_cec = 0x49,
	.i2c_avlink = 0x4a,
	.blank_data = 1,
	.llc_dll_phase = 3,
	.hdmi_free_run_enable = 1,
	.sdp_csc_coeff = {
		.manual = true,
		.scaling = 2,
		.A1 = 0x03a7,
		.A2 = 0x1e91,
		.A3 = 0x1de2,
		.A4 = 0x7d00,
		.B1 = 0x03a7,
		.B2 = 0x0761,
		.B3 = 0x0000,
		.B4 = 0x7900,
		.C1 = 0x03a7,
		.C2 = 0x0000,
		.C3 = 0x0429,
		.C4 = 0x7900,
	},
};

static struct adi_capture_config adi_capture_data = {
	.inputs = adv7842_inputs,
	.num_inputs = ARRAY_SIZE(adv7842_inputs),
	.routes = adv7842_routes,
	.board_info = {
		.type = "adv7842",
		.addr = 0x20,
		.platform_data = (void *)&adv7842_data,
	},
	.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FLDSEL
			| EPPI_CTL_ACTIVE656),
};
#endif
#endif

#if IS_ENABLED(CONFIG_VIDEO_ADI_DISPLAY)
#include <linux/videodev2.h>
#include <media/adi/adi_display.h>
#include <media/adi/ppi.h>

#if IS_ENABLED(CONFIG_VIDEO_ADV7343)
#include <media/i2c/adv7343.h>

static struct v4l2_output adv7343_outputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
	{
		.index = 2,
		.name = "Component",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
};

static struct disp_route adv7343_routes[] = {
	{
		.output = ADV7343_COMPOSITE_ID,
	},
	{
		.output = ADV7343_SVIDEO_ID,
	},
	{
		.output = ADV7343_COMPONENT_ID,
	},
};

static struct adv7343_platform_data adv7343_data = {
	.mode_config = {
		.sleep_mode = false,
		.pll_control = false,
		.dac = {1, 1, 1, 1, 1, 1},
	},
	.sd_config = {
		.sd_dac_out = {0},
	},
};

static struct adi_display_config adi_display_data = {
	.outputs = adv7343_outputs,
	.num_outputs = ARRAY_SIZE(adv7343_outputs),
	.routes = adv7343_routes,
	.board_info = {
		.type = "adv7343",
		.addr = 0x2b,
		.platform_data = (void *)&adv7343_data,
	},
	.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FS1LO_FS2LO
			| EPPI_CTL_POLC3 | EPPI_CTL_BLANKGEN | EPPI_CTL_SYNC2
		| EPPI_CTL_NON656 | EPPI_CTL_DIR),
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_ADV7511)
#include <media/i2c/adv7511.h>

static struct v4l2_output adv7511_outputs[] = {
	{
		.index = 0,
		.name = "HDMI",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_OUT_CAP_CUSTOM_TIMINGS,
	},
};

static struct disp_route adv7511_routes[] = {
	{
		.output = ADV7511_PORT_HDMI_ID,
	},
};

static struct adv7511_platform_data adv7511_data = {
	.i2c_edid = 0x7e,
	.i2c_cec = 0x78,
	.i2c_pktmem = 0x70,
	.cec_clk = 12000000,
};

static struct adi_display_config adi_display_data = {
	.outputs = adv7511_outputs,
	.num_outputs = ARRAY_SIZE(adv7511_outputs),
	.routes = adv7511_routes,
	.board_info = {
		.type = "adv7511",
		.addr = 0x39,
		.platform_data = (void *)&adv7511_data,
	},
	.ppi_control = (EPPI_CTL_SPLTWRD | PACK_EN | DLEN_16
			| EPPI_CTL_FS1LO_FS2LO | EPPI_CTL_POLC3
			| EPPI_CTL_IFSGEN | EPPI_CTL_SYNC2
			| EPPI_CTL_NON656 | EPPI_CTL_DIR),
};
#endif
#endif

static const struct of_dev_auxdata sc57x_auxdata_lookup[] __initconst = {
#if IS_ENABLED(CONFIG_VIDEO_ADI_DISPLAY)
	OF_DEV_AUXDATA("adi,disp", 0x3102D000, "adi_display.0", &adi_display_data),
#endif
#if IS_ENABLED(CONFIG_VIDEO_ADI_CAPTURE)
	OF_DEV_AUXDATA("adi,cap", 0x3102D000, "adi_capture.0", &adi_capture_data),
#endif
	{},
};

static void __init sc57x_init_early(void)
{
	hook_fault_code(16 + 6, sc57x_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
}

static void __init sc57x_init(void)
{
	pr_info("%s: registering device resources\n", __func__);
	of_platform_default_populate(NULL, sc57x_auxdata_lookup, NULL);
	sc5xx_init_ethernet();
}

DT_MACHINE_START(SC57X_DT, "SC57x-EZKIT (Device Tree Support)")
	.l2c_aux_val = 0,
	.l2c_aux_mask = ~0,
	.init_early	= sc57x_init_early,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
MACHINE_END
