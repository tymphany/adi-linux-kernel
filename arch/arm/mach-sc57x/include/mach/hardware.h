/*
 * Copyright (C) 2013 - 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

/* macro to get at MMIO space when running virtually */
#define IO_ADDRESS(x)		(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define __io_address(n)		((void __iomem __force *)IO_ADDRESS(n))

#endif
