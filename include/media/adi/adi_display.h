#ifndef _ADI_DISPLAY_H_
#define _ADI_DISPLAY_H_

#include <linux/i2c.h>

struct v4l2_output;
struct ppi_info;

struct disp_route {
	u32 output;
	u32 config;
	u32 ppi_control;
};

struct adi_display_config {
	/* card name */
	char *card_name;
	/* outputs available at the sub device */
	struct v4l2_output *outputs;
	/* number of outputs supported */
	int num_outputs;
	/* routing information for each output */
	struct disp_route *routes;
	/* i2c bus adapter no */
	int i2c_adapter_id;
	/* i2c subdevice board info */
	struct i2c_board_info board_info;
	/* ppi board info */
	const struct ppi_info *ppi_info;
	/* ppi control */
	u32 ppi_control;
	/* ppi interrupt mask */
	u32 int_mask;
	/* horizontal blanking pixels */
	int blank_pixels;
};

#endif
