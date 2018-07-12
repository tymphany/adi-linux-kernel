/*
 * Copyright (C) 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ADI_H__
#define __ADI_H__

static int get_int_prop(struct device_node *node, const char *s);

#define REG_USB_VBUS_CTL	0x380
#define REG_USB_ID_CTL		0x382
#define REG_USB_PHY_CTL		0x394
#define REG_USB_PLL_OSC		0x398
#define REG_USB_UTMI_CTL	0x39c

#endif
