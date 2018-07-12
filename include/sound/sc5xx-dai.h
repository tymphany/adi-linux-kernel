/*
 * sc5xx-dai - Analog Devices SC5XX DAI code
 *
 * Copyright (c) 2017-2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __SC5XX_DAI_H_
#define __SC5XX_DAI_H_

/*	DAI0	*/
#define REG_DAI0_CLK0		0x310C90C0
#define REG_DAI0_CLK1		0x310C90C4
#define REG_DAI0_CLK2		0x310C90C8
#define REG_DAI0_CLK3		0x310C90CC
#define REG_DAI0_CLK4		0x310C90D0
#define REG_DAI0_CLK5		0x310C90D4
#define REG_DAI0_DAT0		0x310C9100
#define REG_DAI0_DAT1		0x310C9104
#define REG_DAI0_DAT2		0x310C9108
#define REG_DAI0_DAT3		0x310C910C
#define REG_DAI0_DAT4		0x310C9110
#define REG_DAI0_DAT5		0x310C9114
#define REG_DAI0_DAT6		0x310C9118
#define REG_DAI0_FS0		0x310C9140
#define REG_DAI0_FS1		0x310C9144
#define REG_DAI0_FS2		0x310C9148
#define REG_DAI0_FS4		0x310C9150
#define REG_DAI0_PIN0		0x310C9180
#define REG_DAI0_PIN1		0x310C9184
#define REG_DAI0_PIN2		0x310C9188
#define REG_DAI0_PIN3		0x310C918C
#define REG_DAI0_PIN4		0x310C9190
#define REG_DAI0_MISC0		0x310C91C0
#define REG_DAI0_MISC1		0x310C91C4
#define REG_DAI0_PBEN0		0x310C91E0
#define REG_DAI0_PBEN1		0x310C91E4
#define REG_DAI0_PBEN2		0x310C91E8
#define REG_DAI0_PBEN3		0x310C91EC
#define REG_DAI0_IMSK_FE	0x310C9200
#define REG_DAI0_IMSK_RE	0x310C9204
#define REG_DAI0_IMSK_PRI	0x310C9210
#define REG_DAI0_IRPTL_H	0x310C9220
#define REG_DAI0_IRPTL_L	0x310C9224
#define REG_DAI0_IRPTL_HS	0x310C9230
#define REG_DAI0_IRPTL_LS	0x310C9234
#define REG_DAI0_PIN_STAT	0x310C92E4

/*	DAI1	*/
#define REG_DAI1_CLK0          0x310CB0C0
#define REG_DAI1_CLK1          0x310CB0C4
#define REG_DAI1_CLK2          0x310CB0C8
#define REG_DAI1_CLK3          0x310CB0CC
#define REG_DAI1_CLK4          0x310CB0D0
#define REG_DAI1_CLK5          0x310CB0D4
#define REG_DAI1_DAT0          0x310CB100
#define REG_DAI1_DAT1          0x310CB104
#define REG_DAI1_DAT2          0x310CB108
#define REG_DAI1_DAT3          0x310CB10C
#define REG_DAI1_DAT4          0x310CB110
#define REG_DAI1_DAT5          0x310CB114
#define REG_DAI1_DAT6          0x310CB118
#define REG_DAI1_FS0           0x310CB140
#define REG_DAI1_FS1           0x310CB144
#define REG_DAI1_FS2           0x310CB148
#define REG_DAI1_FS4           0x310CB150
#define REG_DAI1_PIN0          0x310CB180
#define REG_DAI1_PIN1          0x310CB184
#define REG_DAI1_PIN2          0x310CB188
#define REG_DAI1_PIN3          0x310CB18C
#define REG_DAI1_PIN4          0x310CB190
#define REG_DAI1_MISC0         0x310CB1C0
#define REG_DAI1_MISC1         0x310CB1C4
#define REG_DAI1_PBEN0         0x310CB1E0
#define REG_DAI1_PBEN1         0x310CB1E4
#define REG_DAI1_PBEN2         0x310CB1E8
#define REG_DAI1_PBEN3         0x310CB1EC
#define REG_DAI1_IMSK_FE       0x310CB200
#define REG_DAI1_IMSK_RE       0x310CB204
#define REG_DAI1_IMSK_PRI      0x310CB210
#define REG_DAI1_IRPTL_H       0x310CB220
#define REG_DAI1_IRPTL_L       0x310CB224
#define REG_DAI1_IRPTL_HS      0x310CB230
#define REG_DAI1_IRPTL_LS      0x310CB234
#define REG_DAI1_PIN_STAT      0x310CB2E4

/* Pads init function for dai  */
void pads_init(void);

#endif	/*__SC5XX_DAI_H_ */
