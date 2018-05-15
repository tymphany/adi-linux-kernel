/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_ARCH_SC58X_H
#define __ASM_ARCH_SC58X_H

#include <linux/of_platform.h>
#include <linux/reboot.h>

extern void __init sc58x_init(void);
extern void __init sc58x_init_early(void);
extern void __init sc58x_init_irq(void);
extern void __init sc58x_map_io(void);
extern void sc58x_timer_init(void);
extern void sc58x_restart(enum reboot_mode, const char *);
extern void sc58x_clock_init(void);
#endif
