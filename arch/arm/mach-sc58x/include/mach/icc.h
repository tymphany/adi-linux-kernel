/*
 * Copyright 2010-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_ICC_H
#define _MACH_ICC_H

#include <asm/cacheflush.h>

/* arch specific */
#define sm_atomic_read(v) ioread16(v)
#define sm_atomic_write(i, v) iowrite16(v, i)

#define ICC_CODE_START		0x20080000

#define invalidate_dcache_range(start, end) __sync_cache_range_r((void *)start, end - start)
#define flush_dcache_range(start, end) __sync_cache_range_w((void *)start, end - start)
#define arm_core_id()	0
#define CCTRL_CORE1     1
#define CCTRL_CORE2     2

#define CMD_CORE_START              _IO('b', 0)
#define CMD_CORE_STOP               _IO('b', 1)
#define CMD_SET_SVECT1              _IO('m', 17)
#define CMD_SET_SVECT2              _IO('m', 18)

void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_send_ipi(cpumask_t callmap, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu);

#endif
