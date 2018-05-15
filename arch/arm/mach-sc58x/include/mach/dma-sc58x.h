/* mach/dma.h - arch-specific DMA defines
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _DMA_SC58X_H_
#define _DMA_SC58X_H_

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

#define MAX_DMA_CHANNELS	45

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

#endif
