/*
 * Copyright (C) 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */


	.macro	addruart, rp, rv, tmp
	ldr	\rp, = 0x31003000			@ System peripherals (phys address)
	ldr	\rv, = 0xF4003000			@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	str	\rd, [\rx, #(0x24)]			@ Write to Transmitter Holding Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(8)]			@ Read Status Register
	tst	\rd, #(0x20)				@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm


	.macro	busyuart,rd,rx
	.endm

	.macro	add_softbp,tmp
	ldr     \tmp,=0
123:
	tst	\tmp, #(1)
	beq	123b
	.endm
