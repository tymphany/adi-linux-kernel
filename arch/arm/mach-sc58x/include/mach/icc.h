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

/* RCU0 (reset core unit) register structure */
struct rcu_reg {
	/* RCU0 Control Register */
	uint32_t reg_rcu_ctl;							/* 0x00 */
	/* RCU0 Status Register */
	uint32_t reg_rcu_stat;							/* 0x04 */
	/* RCU0 Core Reset Control Register */
	uint32_t reg_rcu_crctl;							/* 0x08 */
	/* RCU0 Core Reset Status Register */
	uint32_t reg_rcu_crstat;						/* 0x0c */
	/* RCU0 System Interface Disable Register */
	uint32_t reg_rcu_sidis;							/* 0x10 */
	/* RCU0 System Interface Status Register */
	uint32_t reg_rcu_sistat;						/* 0x14 */
	/* reg pad from 0x18 to 0x1b */
	uint8_t	 pad_0x18_0x1b[0x1c - 0x18];			/* 0x18 ~ 0x1b */
	/* RCU0 Boot Code Register */
	uint32_t reg_rcu_bcode;							/* 0x1c */
	/* Software Vector Register 0 to 2 */
	uint32_t reg_rcu_svect0;						/* 0x20 */
	uint32_t reg_rcu_svect1;						/* 0x24 */
	uint32_t reg_rcu_svect2;						/* 0x28 */
	/* reg pad from 0x2c to 0x63 */
	uint8_t	 pad_0x2c_0x63[0x64 - 0x2c];			/* 0x2c ~ 0x63 */
	/* RCU0 Message Set Bits Register */
	uint32_t reg_rcu_msg_set;						/* 0x64 */
};

void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_send_ipi(cpumask_t callmap, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu);
void platform_res_manage_free_irq(uint16_t subid);
void platform_ipi_init(void);

#endif
