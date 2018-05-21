/*
 * isa_dma.h - SC57x DMA defines/structures/etc...
 *
 * Copyright 2013 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_DMA_H__
#define __ASM_DMA_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <mach/sc57x.h>
#include <mach/dma-sc57x.h>
#include <asm-generic/dma.h>
#include <asm/io.h>

/* DMA_CONFIG Masks */
#define DMAEN			0x00000001	/* DMA Channel Enable */
#define WNR			0x00000002	/* Channel Direction (W/R*) */

#define PSIZE_8			0x00000000	/* Transfer Word Size = 16 */
#define PSIZE_16		0x00000010	/* Transfer Word Size = 16 */
#define PSIZE_32		0x00000020	/* Transfer Word Size = 32 */
#define PSIZE_64		0x00000030	/* Transfer Word Size = 32 */

#define WDSIZE_8		0x00000000	/* Transfer Word Size = 8 */
#define WDSIZE_16		0x00000100	/* Transfer Word Size = 16 */
#define WDSIZE_32		0x00000200	/* Transfer Word Size = 32 */
#define WDSIZE_64		0x00000300	/* Transfer Word Size = 32 */
#define WDSIZE_128		0x00000400	/* Transfer Word Size = 32 */
#define WDSIZE_256		0x00000500	/* Transfer Word Size = 32 */

#define DMA2D			0x04000000	/* DMA Mode (2D/1D*) */
#define RESTART			0x00000004	/* DMA Buffer Clear SYNC */

#define DI_EN_X			0x00100000	/* Data Interrupt Enable in X count */
#define DI_EN_Y			0x00200000	/* Data Interrupt Enable in Y count */
#define DI_EN_P			0x00300000	/* Data Interrupt Enable in Peripheral */
#define DI_EN			DI_EN_X		/* Data Interrupt Enable */

#define NDSIZE_0		0x00000000	/* Next Descriptor Size = 1 */
#define NDSIZE_1		0x00010000	/* Next Descriptor Size = 2 */
#define NDSIZE_2		0x00020000	/* Next Descriptor Size = 3 */
#define NDSIZE_3		0x00030000	/* Next Descriptor Size = 4 */
#define NDSIZE_4		0x00040000	/* Next Descriptor Size = 5 */
#define NDSIZE_5		0x00050000	/* Next Descriptor Size = 6 */
#define NDSIZE_6		0x00060000	/* Next Descriptor Size = 7 */
#define NDSIZE			0x00070000	/* Next Descriptor Size */
#define NDSIZE_OFFSET		16		/* Next Descriptor Size Offset */

#define DMAFLOW_LIST		0x00004000	/* Descriptor List Mode */
#define DMAFLOW_LARGE		DMAFLOW_LIST
#define DMAFLOW_ARRAY		0x00005000	/* Descriptor Array Mode */
#define DMAFLOW_LIST_DEMAND	0x00006000	/* Descriptor Demand List Mode */
#define DMAFLOW_ARRAY_DEMAND	0x00007000	/* Descriptor Demand Array Mode */

#define DMA_RUN_DFETCH		0x00000100	/* DMA Running Fetch */
#define DMA_RUN			0x00000200	/* DMA Running Trans */
#define DMA_RUN_WAIT_TRIG	0x00000300	/* DMA Running WAIT TRIG */
#define DMA_RUN_WAIT_ACK	0x00000400	/* DMA Running WAIT ACK */
#define DMA_RUN_MASK		0x00000700	/* DMA Running Bits Mask */


#define DMAFLOW			0x000007000	/* Flow Control */
#define DMAFLOW_STOP		0x000000000	/* Stop Mode */
#define DMAFLOW_AUTO		0x000001000	/* Autobuffer Mode */

/* DMA_IRQ_STATUS Masks */
#define DMA_DONE		0x1	/* DMA Completion Interrupt Status */
#define DMA_ERR			0x2	/* DMA Error Interrupt Status */
#define DMA_PIRQ		0x4	/* DMA Peripheral Error Interrupt Status */

struct dma_desc_array {
	unsigned long start_addr;
	unsigned long cfg;
	unsigned long x_count;
	long x_modify;
} __packed;

struct dmasg {
	void *next_desc_addr;
	unsigned long start_addr;
	unsigned long cfg;
	unsigned long x_count;
	long x_modify;
	unsigned long y_count;
	long y_modify;
} __packed;

struct dma_register {
	void *next_desc_ptr;	/* DMA Next Descriptor Pointer register */
	unsigned long start_addr;	/* DMA Start address  register */

	unsigned long cfg;	/* DMA Configuration register */

	unsigned long x_count;	/* DMA x_count register */

	long x_modify;	/* DMA x_modify register */

	unsigned long y_count;	/* DMA y_count register */

	long y_modify;	/* DMA y_modify register */

	unsigned long reserved;
	unsigned long reserved2;

	void *curr_desc_ptr;	/* DMA Current Descriptor Pointer
					   register */
	void *prev_desc_ptr;	/* DMA previous initial Descriptor Pointer
					   register */
	unsigned long curr_addr_ptr;	/* DMA Current Address Pointer
						   register */
	unsigned long irq_status;	/* DMA irq status register */

	unsigned long curr_x_count;	/* DMA Current x-count register */

	unsigned long curr_y_count;	/* DMA Current y-count register */

	unsigned long reserved3;

	unsigned long bw_limit_count;	/* DMA band width limit count register */
	unsigned long curr_bw_limit_count;	/* DMA Current band width limit
							count register */
	unsigned long bw_monitor_count;	/* DMA band width limit count register */
	unsigned long curr_bw_monitor_count;	/* DMA Current band width limit
							count register */
};

struct dma_channel {
	const char *device_id;
	atomic_t chan_status;
	struct dma_register *regs;
	struct dmasg *sg;		/* large mode descriptor */
	unsigned int irq;
	irq_handler_t callback;
	unsigned int spu_securep_id;
	void *data;
};

/*******************************************************************************
*	DMA API's
*******************************************************************************/
extern struct dma_channel dma_ch[MAX_DMA_CHANNELS];
extern int channel2irq(unsigned int channel);

static inline void set_dma_start_addr(unsigned int channel, unsigned long addr)
{
	writel(addr, &dma_ch[channel].regs->start_addr);
}
static inline void set_dma_next_desc_addr(unsigned int channel, void *addr)
{
	writel(addr, &dma_ch[channel].regs->next_desc_ptr);
}
static inline void set_dma_curr_desc_addr(unsigned int channel, void *addr)
{
	writel(addr, &dma_ch[channel].regs->curr_desc_ptr);
}
static inline void set_dma_x_count(unsigned int channel, unsigned long x_count)
{
	writel(x_count, &dma_ch[channel].regs->x_count);
}
static inline void set_dma_y_count(unsigned int channel, unsigned long y_count)
{
	writel(y_count, &dma_ch[channel].regs->y_count);
}
static inline void set_dma_x_modify(unsigned int channel, long x_modify)
{
	writel(x_modify, &dma_ch[channel].regs->x_modify);
}
static inline void set_dma_y_modify(unsigned int channel, long y_modify)
{
	writel(y_modify, &dma_ch[channel].regs->y_modify);
}
static inline void set_dma_config(unsigned int channel, unsigned long config)
{
	writel(config, &dma_ch[channel].regs->cfg);
}
static inline void set_dma_curr_addr(unsigned int channel, unsigned long addr)
{
	writel(addr, &dma_ch[channel].regs->curr_addr_ptr);
}

/* set_dma_config parameter valid values */
/* direction */
#define DIR_READ		0
#define DIR_WRITE		1

/* flow_mode */
#define DMA_FLOW_STOP		0
#define DMA_FLOW_AUTO		1
#define DMA_FLOW_LIST		4
#define DMA_FLOW_ARRAY		5
#define DMA_FLOW_LIST_DEMAND	6
#define DMA_FLOW_ARRAY_DEMAND	7

/* intr_mode */
#define INTR_DISABLE		0
#define INTR_ON_PERI		1
#define INTR_ON_BUF		2
#define INTR_ON_ROW		3

/* char dma_mode */
#define DIMENSION_LINEAR	0
#define DIMENSION_2D		1

/* mem_width */
#define DATA_SIZE_8		0
#define DATA_SIZE_16		1
#define DATA_SIZE_32		2
#define DATA_SIZE_64		3
#define DATA_SIZE_128		4
#define DATA_SIZE_256		5

/* syncmode */
#define DMA_NOSYNC_KEEP_DMA_BUF	0
#define DMA_SYNC_RESTART	1

/* peri_width */
#define PDATA_SIZE_8		0
#define PDATA_SIZE_16		1
#define PDATA_SIZE_32		2
#define PDATA_SIZE_64		3

static inline unsigned long
gen_dma_config2(char direction, char flow_mode, char intr_mode,
		     char dma_mode, char mem_width, char syncmode, char peri_width)
{
	unsigned long config = 0;

	switch (intr_mode) {
	case INTR_ON_BUF:
		if (dma_mode == DIMENSION_2D)
			config = DI_EN_Y;
		else
			config = DI_EN_X;
		break;
	case INTR_ON_ROW:
		config = DI_EN_X;
		break;
	case INTR_ON_PERI:
		config = DI_EN_P;
		break;
	};

	return config | (direction << 1) | (mem_width << 8) | (dma_mode << 26) |
		(flow_mode << 12) | (syncmode << 2) | (peri_width << 4);
}

static inline unsigned long
gen_dma_config(char direction, char flow_mode,
		    char intr_mode, char dma_mode, char mem_width, char syncmode)
{
	return gen_dma_config2(direction, flow_mode, intr_mode, dma_mode,
		mem_width, syncmode, mem_width);
}

static inline unsigned long get_dma_curr_irqstat(unsigned int channel)
{
	return readl(&dma_ch[channel].regs->irq_status);
}
static inline unsigned long get_dma_curr_xcount(unsigned int channel)
{
	return readl(&dma_ch[channel].regs->curr_x_count);
}
static inline unsigned long get_dma_curr_ycount(unsigned int channel)
{
	return readl(&dma_ch[channel].regs->curr_y_count);
}
static inline void *get_dma_next_desc_ptr(unsigned int channel)
{
	return (void *)readl(&dma_ch[channel].regs->next_desc_ptr);
}
static inline void *get_dma_curr_desc_ptr(unsigned int channel)
{
	return (void *)readl(&dma_ch[channel].regs->curr_desc_ptr);
}
static inline unsigned long get_dma_config(unsigned int channel)
{
	return readl(&dma_ch[channel].regs->cfg);
}
static inline unsigned long get_dma_curr_addr(unsigned int channel)
{
	return readl(&dma_ch[channel].regs->curr_addr_ptr);
}

static inline void set_dma_sg(unsigned int channel, struct dmasg *sg, int ndsize)
{
	writel((void *)__pa(sg), &dma_ch[channel].regs->next_desc_ptr);
	writel((dma_ch[channel].regs->cfg & ~NDSIZE) |
		((ndsize << NDSIZE_OFFSET) & NDSIZE),
		&dma_ch[channel].regs->cfg);
}

static inline int dma_channel_active(unsigned int channel)
{
	return atomic_read(&dma_ch[channel].chan_status);
}

static inline void disable_dma(unsigned int channel)
{
	writel(readl(&dma_ch[channel].regs->cfg) & ~DMAEN,
		&dma_ch[channel].regs->cfg);
}
static inline void enable_dma(unsigned int channel)
{
	writel(readl(&dma_ch[channel].regs->cfg) | DMAEN,
		&dma_ch[channel].regs->cfg);
}
int set_dma_callback(unsigned int channel, irq_handler_t callback, void *data);

static inline void dma_disable_irq(unsigned int channel)
{
	disable_irq(dma_ch[channel].irq);
}
static inline void dma_disable_irq_nosync(unsigned int channel)
{
	disable_irq_nosync(dma_ch[channel].irq);
}
static inline void dma_enable_irq(unsigned int channel)
{
	enable_irq(dma_ch[channel].irq);
}
static inline void clear_dma_irqstat(unsigned int channel)
{
	writel(DMA_DONE | DMA_ERR | DMA_PIRQ,
		&dma_ch[channel].regs->irq_status);
}

void *dma_memcpy(void *dest, const void *src, size_t count);
void *dma_memcpy_nocache(void *dest, const void *src, size_t count);
void *safe_dma_memcpy(void *dest, const void *src, size_t count);
void early_dma_memcpy(void *dest, const void *src, size_t count);
void early_dma_memcpy_done(void);

#endif
