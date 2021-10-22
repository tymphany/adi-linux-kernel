/*
 * isa_dma.h - SC59x DMA defines/structures/etc...
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_DMA_H__
#define __ASM_DMA_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/soc/adi/sc59x.h>
#include <asm-generic/dma.h>
#include <asm/io.h>

#define CH_SPORT0_TX                   0
#define CH_SPORT0_RX                   1
#define CH_SPORT1_TX                   2
#define CH_SPORT1_RX                   3
#define CH_SPORT2_TX                   4
#define CH_SPORT2_RX                   5
#define CH_SPORT3_TX                   6
#define CH_SPORT3_RX                   7
#define CH_MEM_STREAM0_SRC             8
#define CH_MEM_STREAM0_DEST            9
#define CH_SPORT4_TX                  10
#define CH_SPORT4_RX                  11
#define CH_SPORT5_TX                  12
#define CH_SPORT5_RX                  13
#define CH_SPORT6_TX                  14
#define CH_SPORT6_RX                  15
#define CH_SPORT7_TX                  16
#define CH_SPORT7_RX                  17
#define CH_MEM_STREAM1_SRC            18
#define CH_MEM_STREAM1_DEST           19
#define CH_UART0_TX                   20
#define CH_UART0_RX                   21
#define CH_SPI0_TX                    22
#define CH_SPI0_RX                    23
#define CH_SPI1_TX                    24
#define CH_SPI1_RX                    25
#define CH_SPI2_TX                    26
#define CH_SPI2_RX                    27
#define CH_EPPI0_CH0                  28
#define CH_EPPI0_CH1                  29
#define CH_LP0                        30
#define CH_HAE_OUT                    31
#define CH_HAE_IN0                    32
#define CH_HAE_IN1                    33
#define CH_UART1_TX                   34
#define CH_UART1_RX                   35
#define CH_LP1                        36
#define CH_UART2_TX                   37
#define CH_UART2_RX                   38
#define CH_MEM_STREAM2_SRC            39
#define CH_MEM_STREAM2_DEST           40
#define CH_FFTA0_RX                   41
#define CH_FFTA0_TX                   42
#define CH_MEM_STREAM3_SRC            43
#define CH_MEM_STREAM3_DEST           44

#define MAX_DMA_CHANNELS	57

#define MDMA_S0_NEXT_DESC_PTR	(REG_DMA8_DSCPTR_NXT)
#define MDMA_S0_CONFIG		(REG_DMA8_CFG)
#define MDMA_D0_NEXT_DESC_PTR	(REG_DMA9_DSCPTR_NXT)
#define MDMA_D0_CONFIG		(REG_DMA9_CFG)
#define MDMA_D0_IRQ_STATUS	(REG_DMA9_STAT)
#define MDMA_S1_NEXT_DESC_PTR	(REG_DMA18_DSCPTR_NXT)
#define MDMA_S1_CONFIG		(REG_DMA18_CFG)
#define MDMA_D1_NEXT_DESC_PTR	(REG_DMA19_DSCPTR_NXT)
#define MDMA_D1_CONFIG		(REG_DMA19_CFG)
#define MDMA_D1_IRQ_STATUS	(REG_DMA19_STAT)

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

#define ADI_DMA_NEXT_DESC		0x00
#define ADI_DMA_ADDRSTART		0x04
#define ADI_DMA_CFG				0x08
#define ADI_DMA_XCNT			0x0c
#define ADI_DMA_XMOD			0x10
#define ADI_DMA_YCNT			0x14
#define ADI_DMA_YMOD			0x18
#define ADI_DMA_DSCPTR_CUR		0x24
#define ADI_DMA_DSCPTR_PRV		0x28
#define ADI_DMA_ADDR_CUR		0x2c
#define ADI_DMA_STAT			0x30
#define ADI_DMA_XCNT_CUR		0x34
#define ADI_DMA_YCNT_CUR		0x38
#define ADI_DMA_BWLCNT			0x40
#define ADI_DMA_BWLCNT_CUR		0x44
#define ADI_DMA_BWMCNT			0x48
#define ADI_DMA_BWMCNT_CUR		0x4c

/* @todo scatter-gather mode doesn't work yet */
struct dma_channel {
	const char *device_id;
	atomic_t chan_status;
	void __iomem *ioaddr;
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

static inline void set_dma_start_addr(unsigned int channel, dma_addr_t addr)
{
	writel(lower_32_bits(addr), dma_ch[channel].ioaddr + ADI_DMA_ADDRSTART);
}
static inline void set_dma_next_desc_addr(unsigned int channel, dma_addr_t addr)
{
	writel(lower_32_bits(addr), dma_ch[channel].ioaddr + ADI_DMA_NEXT_DESC);
}
static inline void set_dma_curr_desc_addr(unsigned int channel, dma_addr_t addr)
{
	writel(lower_32_bits(addr), dma_ch[channel].ioaddr + ADI_DMA_DSCPTR_CUR);
}
static inline void set_dma_x_count(unsigned int channel, unsigned long x_count)
{
	writel(x_count, dma_ch[channel].ioaddr + ADI_DMA_XCNT);
}
static inline void set_dma_y_count(unsigned int channel, unsigned long y_count)
{
	writel(y_count, dma_ch[channel].ioaddr + ADI_DMA_YCNT);
}
static inline void set_dma_x_modify(unsigned int channel, long x_modify)
{
	writel(x_modify, dma_ch[channel].ioaddr + ADI_DMA_XMOD);
}
static inline void set_dma_y_modify(unsigned int channel, long y_modify)
{
	writel(y_modify, dma_ch[channel].ioaddr + ADI_DMA_YMOD);
}
static inline void set_dma_config(unsigned int channel, unsigned long config)
{
	writel(config, dma_ch[channel].ioaddr + ADI_DMA_CFG);
}
static inline void set_dma_curr_addr(unsigned int channel, dma_addr_t addr)
{
	writel(lower_32_bits(addr), dma_ch[channel].ioaddr + ADI_DMA_ADDR_CUR);
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
	return readl(dma_ch[channel].ioaddr + ADI_DMA_STAT);
}
static inline unsigned long get_dma_curr_xcount(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_XCNT_CUR);
}
static inline unsigned long get_dma_curr_ycount(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_YCNT_CUR);
}
static inline dma_addr_t get_dma_next_desc_ptr(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_NEXT_DESC);
}
static inline dma_addr_t get_dma_curr_desc_ptr(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_DSCPTR_CUR);
}
static inline unsigned long get_dma_config(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_CFG);
}
static inline unsigned long get_dma_curr_addr(unsigned int channel)
{
	return readl(dma_ch[channel].ioaddr + ADI_DMA_ADDR_CUR);
}

static inline void set_dma_sg(unsigned int channel, struct dmasg *sg, int ndsize)
{
	u32 cfg;
	writel(lower_32_bits(__pa(sg)), dma_ch[channel].ioaddr + ADI_DMA_NEXT_DESC);
	cfg = readl(dma_ch[channel].ioaddr + ADI_DMA_CFG);
	writel((cfg & ~NDSIZE) | ((ndsize << NDSIZE_OFFSET) & NDSIZE),
		dma_ch[channel].ioaddr + ADI_DMA_CFG);
}

static inline int dma_channel_active(unsigned int channel)
{
	return atomic_read(&dma_ch[channel].chan_status);
}

static inline void disable_dma(unsigned int channel)
{
	writel(readl(dma_ch[channel].ioaddr + ADI_DMA_CFG) & ~DMAEN,
		dma_ch[channel].ioaddr + ADI_DMA_CFG);
}
static inline void enable_dma(unsigned int channel)
{
	writel(readl(dma_ch[channel].ioaddr + ADI_DMA_CFG) | DMAEN,
		dma_ch[channel].ioaddr + ADI_DMA_CFG);
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
	writel(DMA_DONE | DMA_ERR | DMA_PIRQ, dma_ch[channel].ioaddr + ADI_DMA_STAT);
}

dma_addr_t dma_memcpy(dma_addr_t dest, const dma_addr_t src, size_t count);
dma_addr_t safe_dma_memcpy(dma_addr_t dest, const dma_addr_t src, size_t count);
void early_dma_memcpy(void *dest, const void *src, size_t count);
void early_dma_memcpy_done(void);

#endif
