/*
 * Copyright (C) 2014 - 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#define IRQ_GIC_START		    32

#define IRQ_SEC_ERR		        (IRQ_GIC_START + 0)	     /* SEC Error */
#define IRQ_CGU0_EVT		    (IRQ_GIC_START + 1)	     /* CGU Event */
#define IRQ_CGU1_EVT		    (IRQ_GIC_START + 2)	     /* CGU Event */
#define IRQ_WATCH0		        (IRQ_GIC_START + 3)	     /* Watchdog0 Interrupt */
#define IRQ_WATCH1		        (IRQ_GIC_START + 4)	     /* Watchdog1 Interrupt */
#define IRQ_WATCH2		        (IRQ_GIC_START + 5)	     /* Watchdog2 Interrupt */
#define IRQ_OTPC0_ERR		    (IRQ_GIC_START + 6)	     /* OTPC0 Dual bit Error Interrupt */
#define IRQ_TMU0_FAULT		    (IRQ_GIC_START + 7)	     /* TMU0 Fault Event */
#define IRQ_TMU0_ALERT		    (IRQ_GIC_START + 8)	     /* TMU0 Fault Alert */
#define IRQ_TAPC0_KEYFAIL	    (IRQ_GIC_START + 9)	     /* Test/User Key Fail Interrupt */
#define IRQ_L2CTL0_ECC_ERR	    (IRQ_GIC_START + 10)	     /* L2 ECC Error */
#define IRQ_L2CTL0_ECC_WARN	    (IRQ_GIC_START + 11)	     /* L2 ECC Waring */
#define IRQ_L2CTL0_EVT          (IRQ_GIC_START + 12)		/* Scrub/Initialization Done */
#define IRQ_MEC0_EEIRQ0         (IRQ_GIC_START + 13)		/* ECC Error Interrupt Request */
#define IRQ_MEC1_EEIRQ0         (IRQ_GIC_START + 14)		/* ECC Error Interrupt Request */
#define IRQ_MEC2_EEIRQ0         (IRQ_GIC_START + 15)		/* ECC Error Interrupt Request */
#define IRQ_MEC0_EWIRQ0         (IRQ_GIC_START + 16)		/* ECC Warning Interrupt Request */
#define IRQ_MEC1_EWIRQ0         (IRQ_GIC_START + 17)		/* ECC Warning Interrupt Request */
#define IRQ_MEC2_EWIRQ0         (IRQ_GIC_START + 18)		/* ECC Warning Interrupt Request */
#define IRQ_MEC0_PEIRQ0         (IRQ_GIC_START + 19)		/* Parity Error Interrupt Request */
#define IRQ_MEC0_PEIRQ1         (IRQ_GIC_START + 20)		/* Parity Error Interrupt Request */
#define IRQ_MEC0_PEIRQ2         (IRQ_GIC_START + 21)		/* Parity Error Interrupt Request */
#define IRQ_MEC0_PEIRQ3         (IRQ_GIC_START + 22)		/* Parity Error Interrupt Request */
#define IRQ_MEC1_PEIRQ0         (IRQ_GIC_START + 23)		/* Parity Error Interrupt Request */
#define IRQ_MEC1_PEIRQ1         (IRQ_GIC_START + 24)		/* Parity Error Interrupt Request */
#define IRQ_MEC1_PEIRQ2         (IRQ_GIC_START + 25)		/* Parity Error Interrupt Request */
#define IRQ_MEC1_PEIRQ3         (IRQ_GIC_START + 26)		/* Parity Error Interrupt Request */
#define IRQ_MEC2_PEIRQ0         (IRQ_GIC_START + 27)		/* Parity Error Interrupt Request */
#define IRQ_MEC2_PEIRQ1         (IRQ_GIC_START + 28)		/* Parity Error Interrupt Request */
#define IRQ_MEC2_PEIRQ2         (IRQ_GIC_START + 29)		/* Parity Error Interrupt Request */
#define IRQ_MEC2_PEIRQ3         (IRQ_GIC_START + 30)		/* Parity Error Interrupt Request */
#define IRQ_C0_L2CC             (IRQ_GIC_START + 31)		/* Core 0 L2CC Interrupt */
#define IRQ_C1_IRQ0             (IRQ_GIC_START + 32)	     /* CORE1 Data Read Interrupt */
#define IRQ_C1_IRQ1             (IRQ_GIC_START + 33)	     /* CORE1 Data Write Interrupt */
#define IRQ_C1_IRQ2             (IRQ_GIC_START + 34)	     /* CORE1 Instruction Read Interrupt */
#define IRQ_C1_IDLE             (IRQ_GIC_START + 35)	     /* CORE1 Idle */
#define IRQ_C2_IRQ0             (IRQ_GIC_START + 36)	     /* CORE2 Data Read Interrupt */
#define IRQ_C2_IRQ1             (IRQ_GIC_START + 37)	     /* CORE2 Data Write Interrupt */
#define IRQ_C2_IRQ2             (IRQ_GIC_START + 38)	     /* CORE2 Instruction Read Interrupt */
#define IRQ_C2_IDLE             (IRQ_GIC_START + 39)	     /* CORE2 Idle */
#define IRQ_DAI0_IRQH		    (IRQ_GIC_START + 40)	     /* DAI0 High Priority Interrupt */
#define IRQ_TIMER0		        (IRQ_GIC_START + 41)	     /* Timer 0 Interrupt */
#define IRQ_TIMER1		        (IRQ_GIC_START + 42)	     /* Timer 1 Interrupt */
#define IRQ_TIMER2		        (IRQ_GIC_START + 43)	     /* Timer 2 Interrupt */
#define IRQ_TIMER3		        (IRQ_GIC_START + 44)	     /* Timer 3 Interrupt */
#define IRQ_ACM0_EVT_MISS	    (IRQ_GIC_START + 45)	     /* ACM0 Event Miss */
#define IRQ_ACM0_EVT_COMPLETE   (IRQ_GIC_START + 46)	     /* ACM0 Event Complete */
#define IRQ_PINT0		        (IRQ_GIC_START + 47)	     /* PINT0 Interrupt */
#define IRQ_PINT1		        (IRQ_GIC_START + 48)	     /* PINT1 Interrupt */
#define IRQ_PINT2		        (IRQ_GIC_START + 49)	     /* PINT2 Interrupt */
#define IRQ_PINT3		        (IRQ_GIC_START + 50)	     /* PINT3 Interrupt */
#define IRQ_PINT4		        (IRQ_GIC_START + 51)	     /* PINT4 Interrupt */
#define IRQ_SOFT0		        (IRQ_GIC_START + 52)	     /* Software-Driven Interrupt 0 */
#define IRQ_SOFT1		        (IRQ_GIC_START + 53)	     /* Software-Driven Interrupt 1 */
#define IRQ_SOFT2		        (IRQ_GIC_START + 54)	     /* Software-Driven Interrupt 2 */
#define IRQ_SOFT3		        (IRQ_GIC_START + 55)	     /* Software-Driven Interrupt 3 */
#define IRQ_SOFT4		        (IRQ_GIC_START + 56)	     /* Software-Driven Interrupt 4 */
#define IRQ_SOFT5		        (IRQ_GIC_START + 57)	     /* Software-Driven Interrupt 5 */
#define IRQ_SOFT6		        (IRQ_GIC_START + 58)	     /* Software-Driven Interrupt 6 */
#define IRQ_SOFT7		        (IRQ_GIC_START + 59)	     /* Software-Driven Interrupt 7 */
#define IRQ_SPORT0_TX		    (IRQ_GIC_START + 60)	     /* SPORT0 Channel A DMA */
#define IRQ_SPORT0_TX_STAT	    (IRQ_GIC_START + 61)	     /* SPORT0 Channel A Status */
#define IRQ_SPORT0_RX		    (IRQ_GIC_START + 62)	     /* SPORT0 Channel B DMA */
#define IRQ_SPORT0_RX_STAT	    (IRQ_GIC_START + 63)	     /* SPORT0 Channel B Status */
#define IRQ_SPORT1_TX		    (IRQ_GIC_START + 64)	     /* SPORT1 Channel A DMA */
#define IRQ_SPORT1_TX_STAT	    (IRQ_GIC_START + 65)	     /* SPORT1 Channel A Status */
#define IRQ_SPORT1_RX		    (IRQ_GIC_START + 66)	     /* SPORT1 Channel B DMA */
#define IRQ_SPORT1_RX_STAT	    (IRQ_GIC_START + 67)	     /* SPORT1 Channel B Status */
#define IRQ_SPI2_TX		        (IRQ_GIC_START + 68)	     /* SPI2 TX DMA Channel */
#define IRQ_SPI2_RX		        (IRQ_GIC_START + 69)	     /* SPI2 RX DMA Channel */
#define IRQ_SPI2_STAT		    (IRQ_GIC_START + 70)	     /* SPI2 Status Interrupt */
#define IRQ_SPI2_ERR		    (IRQ_GIC_START + 71)	     /* SPI2 Error Interrupt */
#define IRQ_TIMER4		        (IRQ_GIC_START + 72)	     /* TIMER0 Timer 4 */
#define IRQ_TIMER5		        (IRQ_GIC_START + 73)	     /* TIMER0Timer 5 */
#define IRQ_TIMER6		        (IRQ_GIC_START + 74)	     /* TIMER0 Timer 6 */
#define IRQ_TIMER7		        (IRQ_GIC_START + 75)	     /* TIMER0 Timer 7 */
#define IRQ_TIMER_STAT		    (IRQ_GIC_START + 76)	     /* TIMER0 Status */
#define IRQ_LP0                 (IRQ_GIC_START + 77)	     /* LP0 DMA Data */
#define IRQ_LP0_STAT            (IRQ_GIC_START + 78)	     /* LP0 Status */
#define IRQ_LP1                 (IRQ_GIC_START + 79)	     /* LP1 DMA Data */
#define IRQ_LP1_STAT            (IRQ_GIC_START + 80)	     /* LP1 Status */
#define IRQ_EPPI0_CH0           (IRQ_GIC_START + 81)	     /* EPPI0 DMA Channel 0 */
#define IRQ_EPPI0_CH1           (IRQ_GIC_START + 82)	     /* EPPI0 DMA Channel 1 */
#define IRQ_EPPI0_STAT          (IRQ_GIC_START + 83)	     /* EPPI0 Status */
#define IRQ_CAN0_RX             (IRQ_GIC_START + 84)	     /* CAN0 Receive */
#define IRQ_CAN0_TX             (IRQ_GIC_START + 85)	     /* CAN0 Transmit */
#define IRQ_CAN0_STAT           (IRQ_GIC_START + 86)	     /* CAN0 Status */
#define IRQ_CAN1_RX             (IRQ_GIC_START + 87)	     /* CAN1 Receive */
#define IRQ_CAN1_TX             (IRQ_GIC_START + 88)	     /* CAN1 Transmit */
#define IRQ_CAN1_STAT           (IRQ_GIC_START + 89)	     /* CAN1 Status */
#define IRQ_SPORT2_TX           (IRQ_GIC_START + 90)	     /* SPORT2 Channel A DMA */
#define IRQ_SPORT2_TX_STAT      (IRQ_GIC_START + 91)	     /* SPORT2 Channel A Status */
#define IRQ_SPORT2_RX           (IRQ_GIC_START + 92)	     /* SPORT2 Channel B DMA */
#define IRQ_SPORT2_RX_STAT      (IRQ_GIC_START + 93)	     /* SPORT2 Channel B Status */
#define IRQ_SPORT3_TX           (IRQ_GIC_START + 94)	     /* SPORT3 Channel A DMA */
#define IRQ_SPORT3_TX_STAT      (IRQ_GIC_START + 95)	     /* SPORT3 Channel A Status */
#define IRQ_SPORT3_RX           (IRQ_GIC_START + 96)	     /* SPORT3 Channel B DMA */
#define IRQ_SPORT3_RX_STAT      (IRQ_GIC_START + 97)	     /* SPORT3 Channel B Status */
#define IRQ_SPI0_TX             (IRQ_GIC_START + 98)	     /* SPI0TX DMA Channel */
#define IRQ_SPI0_RX             (IRQ_GIC_START + 99)	     /* SPI0RX DMA Channel */
#define IRQ_SPI0_STAT           (IRQ_GIC_START + 100)	     /* SPI0Status */
#define IRQ_SPI0_ERR            (IRQ_GIC_START + 101)	     /* SPI0Error */
#define IRQ_SPI1_TX             (IRQ_GIC_START + 102)	     /* SPI1TX DMA Channel */
#define IRQ_SPI1_RX             (IRQ_GIC_START + 103)	     /* SPI1RX DMA Channel */
#define IRQ_SPI1_STAT           (IRQ_GIC_START + 104)	     /* SPI1Status */
#define IRQ_SPI1_ERR            (IRQ_GIC_START + 105)	     /* SPI1Error */
#define IRQ_UART0_TX            (IRQ_GIC_START + 106)	     /* UART0Transmit DMA */
#define IRQ_UART0_RX            (IRQ_GIC_START + 107)	     /* UART0Receive DMA */
#define IRQ_UART0_STAT          (IRQ_GIC_START + 108)	     /* UART0Status */
#define IRQ_UART1_TX            (IRQ_GIC_START + 109)	     /* UART1Transmit DMA */
#define IRQ_UART1_RX            (IRQ_GIC_START + 110)	     /* UART1Receive DMA */
#define IRQ_UART1_STAT          (IRQ_GIC_START + 111)	     /* UART1Status */
#define IRQ_UART2_TX            (IRQ_GIC_START + 112)	     /* UART2Transmit DMA */
#define IRQ_UART2_RX            (IRQ_GIC_START + 113)	     /* UART2Receive DMA */
#define IRQ_UART2_STAT          (IRQ_GIC_START + 114)	     /* UART2Status */
#define IRQ_TWI0                (IRQ_GIC_START + 115)	     /* TWI0Data Interrupt */
#define IRQ_TWI1                (IRQ_GIC_START + 116)	     /* TWI1Data Interrupt */
#define IRQ_TWI2                (IRQ_GIC_START + 117)	     /* TWI2Data Interrupt */
#define IRQ_CNT0_STAT           (IRQ_GIC_START + 118)	     /* CNT0Status */
#define IRQ_ECT_C1_EVT          (IRQ_GIC_START + 119)	     /* Core1 CTI Event  (CTI 1) */
#define IRQ_ECT_C2_EVT          (IRQ_GIC_START + 120)	     /* Core2 CTI Event (CTI2) */
#define IRQ_PKIC0_IRQ           (IRQ_GIC_START + 121)	     /* PublicKey Interrupt (PKA, TRNG, SL) */
#define IRQ_PKTE0_IRQ           (IRQ_GIC_START + 122)	     /* SecurityPacket Engine Interrupt */
#define IRQ_MSI0_STAT           (IRQ_GIC_START + 123)	     /* MSI0Status */
#define IRQ_USB0_STAT           (IRQ_GIC_START + 124)	     /* USB0Status/FIFO Data Ready */
#define IRQ_USB0_DATA           (IRQ_GIC_START + 125)	     /* USB0DMA Status/Transfer Complete */
#define IRQ_TRU0_INT4           (IRQ_GIC_START + 126)	     /* TRU0Interrupt 4 */
#define IRQ_TRU0_INT5           (IRQ_GIC_START + 127)	     /* TRU0Interrupt 5 */
#define IRQ_TRU0_INT6           (IRQ_GIC_START + 128)	     /* TRU0Interrupt 6 */
#define IRQ_TRU0_INT7           (IRQ_GIC_START + 129)	     /* TRU0Interrupt 7 */
#define IRQ_TRU0_INT8           (IRQ_GIC_START + 130)	     /* TRU0Interrupt 8 */
#define IRQ_TRU0_INT9           (IRQ_GIC_START + 131)	     /* TRU0Interrupt 9 */
#define IRQ_TRU0_INT10          (IRQ_GIC_START + 132)	     /* TRU0Interrupt 10 */
#define IRQ_TRU0_INT11          (IRQ_GIC_START + 133)	     /* TRU0Interrupt 11 */
#define IRQ_DAI0_IRQL           (IRQ_GIC_START + 134)	     /* DAI0Low Priority Interrupt */
#define IRQ_EMAC0_STAT          (IRQ_GIC_START + 135)	     /* EMAC0Status */
#define IRQ_FIR0_DMA            (IRQ_GIC_START + 136)	     /* FIR0DMA */
#define IRQ_FIR0_STAT           (IRQ_GIC_START + 137)	     /* FIR0Status */
#define IRQ_IIR0_DMA            (IRQ_GIC_START + 138)	     /* IIR0DMA */
#define IRQ_IIR0_STAT           (IRQ_GIC_START + 139)	     /* IIR0Status */
#define IRQ_HADC0_EVT           (IRQ_GIC_START + 140)	     /* HADC0Interrupt */
#define IRQ_MLB0_INT0           (IRQ_GIC_START + 141)	     /* MLB0AHB interrupt 0 */
#define IRQ_MLB0_INT1           (IRQ_GIC_START + 142)	     /* MLB0AHB Interrupt 1 */
#define IRQ_MLB0_STAT           (IRQ_GIC_START + 143)	     /* MLB0Status */
#define IRQ_MDMA3_SRC           (IRQ_GIC_START + 144)	     /* MaxBW DMA Channel 0 */
#define IRQ_MDMA3_DST           (IRQ_GIC_START + 145)	     /* MaxBW DMA Channel 1 */
#define IRQ_MDMA2_SRC           (IRQ_GIC_START + 146)	     /* EnhBW DMA Channel 0 */
#define IRQ_MDMA2_DST           (IRQ_GIC_START + 147)	     /* EnhBW DMA Channel 1 */
#define IRQ_EMDMA0_DONE         (IRQ_GIC_START + 148)	     /* EMDMA0DMA Done */
#define IRQ_EMDMA1_DONE         (IRQ_GIC_START + 149)	     /* EMDMA1DMA Done */
#define IRQ_MDMA0_SRC_CRC       (IRQ_GIC_START + 150)	     /* MDMASource 0 (Standard BW DMA)/CRC0 In */
#define IRQ_MDMA0_DST_CRC       (IRQ_GIC_START + 151)	     /* MDMADest 0 (Standard BW DMA)/CRC0 Out */
#define IRQ_MDMA1_SRC_CRC       (IRQ_GIC_START + 152)	     /* MDMASource 1 (Standard BW DMA)/CRC1 In) */
#define IRQ_MDMA1_DST_CRC       (IRQ_GIC_START + 153)	     /* MDMADest 1 (Standard BW DMA)/CRC1 Out) */
#define IRQ_CRC0_DCNTEXP        (IRQ_GIC_START + 154)	     /* CRC0Datacount expiration */
#define IRQ_CRC1_DCNTEXP        (IRQ_GIC_START + 155)	     /* CRC1Datacount expiration */
#define IRQ_CRC0_ERR            (IRQ_GIC_START + 156)	     /* CRC0Error */
#define IRQ_CRC1_ERR            (IRQ_GIC_START + 157)	     /* CRC1Error */
#define IRQ_SPORT0_TX_ERR       (IRQ_GIC_START + 158)	     /* SPORT0Channel A DMA Error */
#define IRQ_SPORT0_RX_ERR       (IRQ_GIC_START + 159)	     /* SPORT0Channel B DMA Error */
#define IRQ_SPORT1_TX_ERR       (IRQ_GIC_START + 160)	     /* SPORT1Channel A DMA Error */
#define IRQ_SPORT1_RX_ERR       (IRQ_GIC_START + 161)	     /* SPORT1Channel B DMA Error */
#define IRQ_SPI2_TXDMA_ERR      (IRQ_GIC_START + 162)	     /* SPI2TX DMA Channel Error */
#define IRQ_SPI2_RXDMA_ERR      (IRQ_GIC_START + 163)	     /* SPI2RX DMA Channel Error */
#define IRQ_SPORT2_TX_ERR       (IRQ_GIC_START + 164)	     /* SPORT2Channel A DMA Error */
#define IRQ_SPORT2_RX_ERR       (IRQ_GIC_START + 165)	     /* SPORT2Channel B DMA Error */
#define IRQ_SPORT3_TX_ERR       (IRQ_GIC_START + 166)	     /* SPORT3Channel A DMA Error */
#define IRQ_SPORT3_RX_ERR       (IRQ_GIC_START + 167)	     /* SPORT3Channel B DMA Error */
#define IRQ_SPI0_TXDMA_ERR      (IRQ_GIC_START + 168)	     /* SPI0TX DMA Channel Error */
#define IRQ_SPI0_RXDMA_ERR      (IRQ_GIC_START + 169)	     /* SPI0RX DMA Channel Error */
#define IRQ_SPI1_TXDMA_ERR      (IRQ_GIC_START + 170)	     /* SPI1TX DMA Channel Error */
#define IRQ_SPI1_RXDMA_ERR      (IRQ_GIC_START + 171)	     /* SPI1RX DMA Channel Error */
#define IRQ_UART0_TX_ERR        (IRQ_GIC_START + 172)	     /* UART0Transmit DMA Error */
#define IRQ_UART0_RX_ERR        (IRQ_GIC_START + 173)	     /* UART0Receive DMA Error */
#define IRQ_UART1_TX_ERR        (IRQ_GIC_START + 174)	     /* UART1Transmit DMA Error */
#define IRQ_UART1_RX_ERR        (IRQ_GIC_START + 175)	     /* UART1Receive DMA Error */
#define IRQ_UART2_TX_ERR        (IRQ_GIC_START + 176)	     /* UART2Transmit DMA Error */
#define IRQ_UART2_RX_ERR        (IRQ_GIC_START + 177)	     /* UART2Receive DMA Error */
#define IRQ_LP0_ERR             (IRQ_GIC_START + 178)	     /* LP0DMA Data Error */
#define IRQ_LP1_ERR             (IRQ_GIC_START + 179)	     /* LP1DMA Data Error */
#define IRQ_EPPI0_CH0_ERR       (IRQ_GIC_START + 180)	     /* EPPI0DMA Channel 0 Error */
#define IRQ_EPPI0_CH1_ERR       (IRQ_GIC_START + 181)	     /* EPPI0DMA Channel 1 Error */
#define IRQ_MDMA0_SRC_ERR       (IRQ_GIC_START + 182)	     /* StandardBW Souce 0 DMA Channel Error */
#define IRQ_MDMA0_DST_ERR       (IRQ_GIC_START + 183)	     /* StandardBW Dest 0 DMA Channel Error */
#define IRQ_MDMA1_SRC_ERR       (IRQ_GIC_START + 184)	     /* StandardBW Souce 1 DMA Channel Error */
#define IRQ_MDMA1_DST_ERR       (IRQ_GIC_START + 185)	     /* StandardBW Dest 1 DMA Channel Error */
#define IRQ_MDMA2_SRC_ERR       (IRQ_GIC_START + 186)	     /* EnhBW DMA Channel 0 Error */
#define IRQ_MDMA2_DST_ERR       (IRQ_GIC_START + 187)	     /* EnhBW DMA Channel 1 Error */
#define IRQ_MDMA3_SRC_ERR       (IRQ_GIC_START + 188)	     /* MaxBW DMA Channel 0 Error */
#define IRQ_MDMA3_DST_ERR       (IRQ_GIC_START + 189)	     /* MaxBW DMA Channel 1 Error */
#define IRQ_SWU0_EVT            (IRQ_GIC_START + 190)	     /* SWU0Event SMC */
#define IRQ_SWU1_EVT            (IRQ_GIC_START + 191)	     /* SWU1Event DL2_0 */
#define IRQ_SWU2_EVT            (IRQ_GIC_START + 192)	     /* SWU2Event CL2_0 */
#define IRQ_SWU7_EVT            (IRQ_GIC_START + 193)	     /* SWU7Event Core0_DB */
#define IRQ_SWU8_EVT            (IRQ_GIC_START + 194)	     /* SWU8Event Core0_EB */
#define IRQ_SWU9_EVT            (IRQ_GIC_START + 195)	     /* SWU9Event Core1_DB */
#define IRQ_SWU10_EVT           (IRQ_GIC_START + 196)	     /* SWU10Event Core1_EB */
#define IRQ_SWU11_EVT           (IRQ_GIC_START + 197)	     /* SWU11Event SMMR */
#define IRQ_SWU12_EVT           (IRQ_GIC_START + 198)	     /* SWU12Event SPIF */
#define IRQ_SWU13_EVT           (IRQ_GIC_START + 199)	     /* SWU13Event DMC0_A */
#define IRQ_SPU0_INT            (IRQ_GIC_START + 200)	     /* SPU0Event */
#define IRQ_SMPU0_AGGR_INT      (IRQ_GIC_START + 201)	     /* SMPUAggregated Event */
#define IRQ_EMAC0_PWR           (IRQ_GIC_START + 202)	     /* EMAC0Power */
#define IRQ_TRU0_INT0           (IRQ_GIC_START + 203)	     /* TRU0 Interrupt0 */
#define IRQ_TRU0_INT1           (IRQ_GIC_START + 204)	     /* TRU0 Interrupt1 */
#define IRQ_TRU0_INT2           (IRQ_GIC_START + 205)	     /* TRU0 Interrupt2 */
#define IRQ_TRU0_INT3           (IRQ_GIC_START + 206)	     /* TRU0 Interrupt3 */
#define IRQ_CTI_EVT             (IRQ_GIC_START + 207)	     /* Core 0 CTI Event */
#define IRQ_PMU_EVT             (IRQ_GIC_START + 208)	     /* Core 0 Perf Monitor Unit */
#define IRQ_SYS_MBIST0_FAIL     (IRQ_GIC_START + 209)		 /* MBIST Fail Interrupt */
#define IRQ_SYS_MBIST0_RDY      (IRQ_GIC_START + 210)		 /* MBIST Ready Interrupt */
#define IRQ_SYS_C0_INITDONE     (IRQ_GIC_START + 211)		 /* Core 0 Memory Initialization Done */
