/*
 * twi.h - TWI register access header
 *
 * Copyright 2014 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_TWI_H__
#define __ASM_TWI_H__

#include <linux/types.h>
#include <mach/sc58x.h>
#include <mach/anomaly.h>
#include <mach/clkdev.h>

#define DEFINE_TWI_REG(reg_name, reg) \
static inline u16 read_##reg_name(struct adi_twi_iface *iface) \
	{ return ioread16(&iface->regs_base->reg); } \
static inline void write_##reg_name(struct adi_twi_iface *iface, u16 v) \
	{ iowrite16(v, &iface->regs_base->reg); }

DEFINE_TWI_REG(CLKDIV, clkdiv)
DEFINE_TWI_REG(CONTROL, control)
DEFINE_TWI_REG(SLAVE_CTL, slave_ctl)
DEFINE_TWI_REG(SLAVE_STAT, slave_stat)
DEFINE_TWI_REG(SLAVE_ADDR, slave_addr)
DEFINE_TWI_REG(MASTER_CTL, master_ctl)
DEFINE_TWI_REG(MASTER_STAT, master_stat)
DEFINE_TWI_REG(MASTER_ADDR, master_addr)
DEFINE_TWI_REG(INT_STAT, int_stat)
DEFINE_TWI_REG(INT_MASK, int_mask)
DEFINE_TWI_REG(FIFO_CTL, fifo_ctl)
DEFINE_TWI_REG(FIFO_STAT, fifo_stat)
DEFINE_TWI_REG(XMT_DATA8, xmt_data8)
DEFINE_TWI_REG(XMT_DATA16, xmt_data16)
#if !ANOMALY_16000030
DEFINE_TWI_REG(RCV_DATA8, rcv_data8)
DEFINE_TWI_REG(RCV_DATA16, rcv_data16)
#else
static inline u16 read_RCV_DATA8(struct adi_twi_iface *iface)
{
	u16 ret;
	unsigned long flags;

	local_irq_save(flags);
	ret = ioread16(&iface->regs_base->rcv_data8);
	local_irq_restore(flags);

	return ret;
}

static inline u16 read_RCV_DATA16(struct adi_twi_iface *iface)
{
	u16 ret;
	unsigned long flags;

	local_irq_save(flags);
	ret = ioread16(&iface->regs_base->rcv_data16);
	local_irq_restore(flags);

	return ret;
}
#endif

#endif
