/*
 * Common portmux header file
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _PORTMUX_H_
#define _PORTMUX_H_

#define P_IDENT(x)	((x) & 0x1FF)
#define P_FUNCT(x)	(((x) & 0x3) << 9)
#define P_FUNCT2MUX(x)	(((x) >> 9) & 0x3)
#define P_DEFINED	0x8000
#define P_UNDEF		0x4000
#define P_MAYSHARE	0x2000
#define P_DONTCARE	0x1000

#define peripheral_request(per, label) 0
#define peripheral_free(per)
#define peripheral_request_list(per, label) \
	((pdev && pdev->dev.pins) ? pinctrl_select_state(pdev->dev.pins->p, \
	pdev->dev.pins->default_state) : 0)

#define peripheral_free_list(per)

int pinmux_request(unsigned short per, const char *label);
void pinmux_free(unsigned short per);
int pinmux_request_list(const unsigned short per[], const char *label);
void pinmux_free_list(const unsigned short per[]);

#include <linux/err.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/gpio.h>
#include <mach/sc58x.h>

#endif				/* _PORTMUX_H_ */
