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

void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_send_ipi(cpumask_t callmap, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu);
void platform_res_manage_free_irq(uint16_t subid);
void platform_ipi_init(void);

#endif
