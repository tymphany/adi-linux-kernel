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

#define ICC_CODE_START		0x20000000

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
	/* reg pad from 0x10 to 0x17 */
	uint8_t	 pad_0x10_0x17[0x18 - 0x10];			/* 0x10 ~ 0x17 */
	/* RCU0 System Reset Status Register */
	uint32_t reg_rcu_srrqstat;						/* 0x18 */
	/* RCU0 System Interface Disable Register */
	uint32_t reg_rcu_sidis;							/* 0x1c */
	/* RCU0 System Interface Status Register */
	uint32_t reg_rcu_sistat;						/* 0x20 */
	/* reg pad from 0x24 to 0x27 */
	uint8_t	 pad_0x24_0x27[0x28 - 0x24];			/* 0x24 ~ 0x27 */
	/* RCU0 Boot Code Register */
	uint32_t reg_rcu_bcode;							/* 0x28 */
	/* Software Vector Register 0 to 2 */
	uint32_t reg_rcu_svect0;						/* 0x2c */
	uint32_t reg_rcu_svect1;						/* 0x30 */
	uint32_t reg_rcu_svect2;						/* 0x34 */
	/* reg pad from 0x38 to 0x6b */
	uint8_t	 pad_0x38_0x6b[0x6C - 0x38];			/* 0x38 ~ 0x6b */
	/* RCU0 Message Register */
	uint32_t reg_rcu_msg;							/* 0x6C */
	/* RCU0 Message Set Bits Register */
	uint32_t reg_rcu_msg_set;						/* 0x70 */
	/* RCU0 Message Clear Bits Register */
	uint32_t reg_rcu_msg_clr;						/* 0x74 */
};

void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_send_ipi(cpumask_t callmap, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu);
void platform_res_manage_free_irq(uint16_t subid);
void platform_ipi_init(void);

#endif
