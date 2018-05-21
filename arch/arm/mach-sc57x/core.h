/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_ARCH_SC57X_H
#define __ASM_ARCH_SC57X_H

#include <linux/of_platform.h>
#include <linux/reboot.h>

extern void __init sc57x_init(void);
extern void __init sc57x_init_early(void);
extern void __init sc57x_init_irq(void);
extern void __init sc57x_map_io(void);
extern void sc57x_timer_init(void);
extern void sc57x_restart(enum reboot_mode, const char *);
extern void sc57x_clock_init(void);
#endif
