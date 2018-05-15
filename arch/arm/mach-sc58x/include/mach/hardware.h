/*
 * Copyright (C) 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

/*
 * PCI space virtual addresses
 */
#define SC58X_PCI_VIRT_BASE		(void __iomem *)0xe8000000ul
#define SC58X_CFG_VIRT_BASE		(void __iomem *)0xe9000000ul

/* macro to get at MMIO space when running virtually */
#define IO_ADDRESS(x)		(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define __io_address(n)		((void __iomem __force *)IO_ADDRESS(n))

#endif
