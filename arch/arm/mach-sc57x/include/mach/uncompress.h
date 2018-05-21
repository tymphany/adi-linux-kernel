/*
 * Copyright 2014 - 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#define AMBA_UART_DR	(*(volatile unsigned char *)0x31003024)
#define AMBA_UART_FR	(*(volatile unsigned char *)0x31003008)

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (!(AMBA_UART_FR & (1 << 5)))
		barrier();

	AMBA_UART_DR = c;
}

static inline void flush(void)
{
	while (AMBA_UART_FR & (1 << 3))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
