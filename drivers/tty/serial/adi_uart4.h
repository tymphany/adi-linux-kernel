/*
 * ADI UART4 Serial Driver
 *
 * Copyright 2013-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _ADI_UART4_H
#define _ADI_UART4_H

/* UART_CTL Masks */
#define UCEN                     0x1  /* Enable UARTx Clocks */
#define LOOP_ENA                 0x2  /* Loopback Mode Enable */
#define UMOD_MDB                 0x10  /* Enable MDB Mode */
#define UMOD_IRDA                0x20  /* Enable IrDA Mode */
#define UMOD_MASK                0x30  /* Uart Mode Mask */
#define WLS(x)                   (((x-5) & 0x03) << 8)  /* Word Length Select */
#define WLS_MASK                 0x300  /* Word length Select Mask */
#define WLS_OFFSET               8      /* Word length Select Offset */
#define STB                      0x1000  /* Stop Bits */
#define STBH                     0x2000  /* Half Stop Bits */
#define PEN                      0x4000  /* Parity Enable */
#define EPS                      0x8000  /* Even Parity Select */
#define STP                      0x10000  /* Stick Parity */
#define FPE                      0x20000  /* Force Parity Error On Transmit */
#define FFE                      0x40000  /* Force Framing Error On Transmit */
#define SB                       0x80000  /* Set Break */
#define LCR_MASK		 (SB | STP | EPS | PEN | STB | WLS_MASK)
#define FCPOL                    0x400000  /* Flow Control Pin Polarity */
#define RPOLC                    0x800000  /* IrDA RX Polarity Change */
#define TPOLC                    0x1000000  /* IrDA TX Polarity Change */
#define MRTS                     0x2000000  /* Manual Request To Send */
#define XOFF                     0x4000000  /* Transmitter Off */
#define ARTS                     0x8000000  /* Automatic Request To Send */
#define ACTS                     0x10000000  /* Automatic Clear To Send */
#define RFIT                     0x20000000  /* Receive FIFO IRQ Threshold */
#define RFRT                     0x40000000  /* Receive FIFO RTS Threshold */

/* UART_STAT Masks */
#define DR                       0x01  /* Data Ready */
#define OE                       0x02  /* Overrun Error */
#define PE                       0x04  /* Parity Error */
#define FE                       0x08  /* Framing Error */
#define BI                       0x10  /* Break Interrupt */
#define THRE                     0x20  /* THR Empty */
#define TEMT                     0x80  /* TSR and UART_THR Empty */
#define TFI                      0x100  /* Transmission Finished Indicator */

#define ASTKY                    0x200  /* Address Sticky */
#define ADDR                     0x400  /* Address bit status */
#define RO			 0x800  /* Reception Ongoing */
#define SCTS                     0x1000  /* Sticky CTS */
#define CTS                      0x10000  /* Clear To Send */
#define RFCS                     0x20000  /* Receive FIFO Count Status */

/* UART_CLOCK Masks */
#define EDBO                     0x80000000 /* Enable Devide by One */

#endif
