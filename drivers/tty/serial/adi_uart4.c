// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI UART4 Serial Driver
 *
 * Copyright 2013-2021 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/clk.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqflags.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/compiler.h>

#include <linux/soc/adi/hardware.h>
#include <linux/soc/adi/portmux.h>
#include <linux/soc/adi/gpio.h>

#include "adi_uart4.h"
#ifdef CONFIG_ARCH_SC59X
#include <mach/sc59x.h>
#elif defined(CONFIG_ARCH_SC59X_64)
#include <linux/soc/adi/sc59x.h>
#elif defined(CONFIG_ARCH_SC58X)
#include <mach/sc58x.h>
#elif defined(CONFIG_ARCH_SC57X)
#include <mach/sc57x.h>
#endif

#if defined(CONFIG_SERIAL_ADI_UART4_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#define DRIVER_NAME "adi-uart4"

struct uart4_reg {
	u32 revid;
	u32 control;
	u32 status;
	u32 scr;
	u32 clock;
	u32 emask;
	u32 emaskst;
	u32 emaskcl;
	u32 rbr;
	u32 thr;
	u32 taip;
	u32 tsr;
	u32 rsr;
	u32 txdiv_cnt;
	u32 rxdiv_cnt;
};

struct adi_uart4_serial_port {
	struct uart_port port;
	struct device *dev;
	unsigned int old_status;
	int tx_irq;
	int rx_irq;
	int status_irq;
	unsigned int lsr;
	/* DMA specific fields */
	int tx_done;
	int tx_count;
	dma_addr_t tx_dma_phy;
	struct scatterlist tx_sgl;
	struct circ_buf rx_dma_buf;
	dma_addr_t rx_dma_phy;
	dma_cookie_t rx_cookie;
	struct timer_list rx_dma_timer;
	spinlock_t rx_lock;
	struct dma_chan *tx_dma_channel;
	struct dma_chan *rx_dma_channel;
	struct work_struct tx_dma_workqueue;
	/* Hardware flow control specific fields */
	unsigned int hwflow_mode;
	unsigned int cts_pin;
	unsigned int rts_pin;
	unsigned int enable_pin;
	unsigned int enable_pin_active_low;
	unsigned int hwflow_en_pin;
	unsigned int hwflow_en_pin_active_low;
	bool hwflow_en;
};


#define ADI_UART_NO_HWFLOW	0
#define ADI_UART_HWFLOW_PERI	1
#define ADI_UART_HWFLOW_GPIO	2


#define ADI_UART_NR_PORTS 3
static struct adi_uart4_serial_port *adi_uart4_serial_ports[ADI_UART_NR_PORTS];

/* UART_IER Masks */
#define ERBFI                    0x01  /* Enable Receive Buffer Full Interrupt */
#define ETBEI                    0x02  /* Enable Transmit Buffer Empty Interrupt */
#define ELSI                     0x04  /* Enable RX Status Interrupt */
#define EDSSI                    0x08  /* Enable Modem Status Interrupt */
#define EDTPTI                   0x10  /* Enable DMA Transmit PIRQ Interrupt */
#define ETFI                     0x20  /* Enable Transmission Finished Interrupt */
#define ERFCI                    0x40  /* Enable Receive FIFO Count Interrupt */

# define OFFSET_REDIV            0x00  /* Version ID Register             */
# define OFFSET_CTL              0x04  /* Control Register                */
# define OFFSET_STAT             0x08  /* Status Register                 */
# define OFFSET_SCR              0x0C  /* SCR Scratch Register            */
# define OFFSET_CLK              0x10  /* Clock Rate Register             */
# define OFFSET_IER              0x14  /* Interrupt Enable Register       */
# define OFFSET_IER_SET          0x18  /* Set Interrupt Enable Register   */
# define OFFSET_IER_CLEAR        0x1C  /* Clear Interrupt Enable Register */
# define OFFSET_RBR              0x20  /* Receive Buffer register         */
# define OFFSET_THR              0x24  /* Transmit Holding register       */

#define UART_GET_CHAR(p)      readl(p->port.membase + OFFSET_RBR)
#define UART_GET_CLK(p)       readl(p->port.membase + OFFSET_CLK)
#define UART_GET_CTL(p)       readl(p->port.membase + OFFSET_CTL)
#define UART_GET_GCTL(p)      UART_GET_CTL(p)
#define UART_GET_LCR(p)       UART_GET_CTL(p)
#define UART_GET_MCR(p)       UART_GET_CTL(p)
#define UART_GET_STAT(p)      readl(p->port.membase + OFFSET_STAT)
#define UART_GET_MSR(p)       UART_GET_STAT(p)

#define UART_PUT_CHAR(p, v)   writel(v, p->port.membase + OFFSET_THR)
#define UART_PUT_CLK(p, v)    writel(v, p->port.membase + OFFSET_CLK)
#define UART_PUT_CTL(p, v)    writel(v, p->port.membase + OFFSET_CTL)
#define UART_PUT_GCTL(p, v)   UART_PUT_CTL(p, v)
#define UART_PUT_LCR(p, v)    UART_PUT_CTL(p, v)
#define UART_PUT_MCR(p, v)    UART_PUT_CTL(p, v)
#define UART_PUT_STAT(p, v)   writel(v, p->port.membase + OFFSET_STAT)

#define UART_CLEAR_IER(p, v)  writel(v, p->port.membase + OFFSET_IER_CLEAR)
#define UART_GET_IER(p)       readl(p->port.membase + OFFSET_IER)
#define UART_SET_IER(p, v)    writel(v, p->port.membase + OFFSET_IER_SET)

#define UART_CLEAR_DLAB(p)    /* MMRs not muxed on BF60x */
#define UART_SET_DLAB(p)      /* MMRs not muxed on BF60x */

#define UART_CLEAR_LSR(p)     UART_PUT_STAT(p, -1)
#define UART_GET_LSR(p)       UART_GET_STAT(p)
#define UART_PUT_LSR(p, v)    UART_PUT_STAT(p, v)

/* This handles hard CTS/RTS */
#define UART_CTSRTS_HARD
#define UART_CLEAR_SCTS(p)      UART_PUT_STAT(p, SCTS)
#define UART_GET_CTS(x)         (UART_GET_MSR(x) & CTS)
#define UART_DISABLE_RTS(x)     UART_PUT_MCR(x, UART_GET_MCR(x) & ~(ARTS | MRTS))
#define UART_ENABLE_RTS(x)      UART_PUT_MCR(x, UART_GET_MCR(x) | MRTS | ARTS)
#define UART_ENABLE_INTS(x, v)  UART_SET_IER(x, v)
#define UART_DISABLE_INTS(x)    UART_CLEAR_IER(x, 0xF)


#define DMA_RX_XCOUNT		512
#define DMA_RX_YCOUNT		(PAGE_SIZE / DMA_RX_XCOUNT)

#define DMA_RX_FLUSH_JIFFIES	(msecs_to_jiffies(50))

static void adi_uart4_serial_dma_tx_chars(struct adi_uart4_serial_port *uart);
static void adi_uart4_serial_dma_tx(void *data);
static void adi_uart4_serial_tx_chars(struct adi_uart4_serial_port *uart);
static void adi_uart4_serial_reset_irda(struct uart_port *port);

static unsigned int adi_uart4_serial_get_mctrl(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	if (!uart->hwflow_mode || !uart->hwflow_en)
		return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;

	/* CTS PIN is negative assertive. */
	if (UART_GET_CTS(uart))
		return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
	else
		return TIOCM_DSR | TIOCM_CAR;
}

static void adi_uart4_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	if (!uart->hwflow_mode || !uart->hwflow_en)
		return;

	/* RTS PIN is negative assertive. */
	if (mctrl & TIOCM_RTS)
		UART_ENABLE_RTS(uart);
	else
		UART_DISABLE_RTS(uart);
}

/*
 * Handle any change of modem status signal.
 */
static irqreturn_t adi_uart4_serial_mctrl_cts_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;
	unsigned int status = adi_uart4_serial_get_mctrl(&uart->port);
	struct tty_struct *tty = uart->port.state->port.tty;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
		UART_CLEAR_SCTS(uart);
		if (tty->hw_stopped) {
			if (status) {
				tty->hw_stopped = 0;
				uart_write_wakeup(&uart->port);
			}
		} else {
			if (!status)
				tty->hw_stopped = 1;
		}
	}

	uart_handle_cts_change(&uart->port, status & TIOCM_CTS);

	return IRQ_HANDLED;
}

/*
 * interrupts are disabled on entry
 */
static void adi_uart4_serial_stop_tx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	struct circ_buf *xmit = &uart->port.state->xmit;

	while (!(UART_GET_LSR(uart) & TEMT))
		cpu_relax();

	if (uart->tx_dma_channel) {
		if (!uart->tx_done)
			dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);
		dmaengine_terminate_sync(uart->tx_dma_channel);
		xmit->tail = (xmit->tail + uart->tx_count) &
			(UART_XMIT_SIZE - 1);
		uart->port.icount.tx += uart->tx_count;
		uart->tx_count = 0;
		uart->tx_done = 1;
	} else {
		/* Clear TFI bit */
		UART_PUT_LSR(uart, TFI);
		UART_CLEAR_IER(uart, ETBEI);
	}
}

/*
 * port is locked and interrupts are disabled
 */
static void adi_uart4_serial_start_tx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	struct tty_struct *tty = uart->port.state->port.tty;

	/*
	 * To avoid losting RX interrupt, we reset IR function
	 * before sending data.
	 */
	if (tty->termios.c_line == N_IRDA)
		adi_uart4_serial_reset_irda(port);

	if (uart->tx_dma_channel) {
		if (uart->tx_done)
			adi_uart4_serial_dma_tx_chars(uart);
	} else {
		UART_SET_IER(uart, ETBEI);
		adi_uart4_serial_tx_chars(uart);
	}
}

/*
 * Interrupts are enabled
 */
static void adi_uart4_serial_stop_rx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);

	UART_CLEAR_IER(uart, ERBFI);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void adi_uart4_serial_enable_ms(struct uart_port *port)
{
}

static void adi_uart4_serial_rx_chars(struct adi_uart4_serial_port *uart)
{
	unsigned int status, ch, flg;

	status = UART_GET_LSR(uart);
	UART_CLEAR_LSR(uart);

	ch = UART_GET_CHAR(uart);
	uart->port.icount.rx++;

	if (status & BI) {
		uart->port.icount.brk++;
		if (uart_handle_break(&uart->port))
			goto ignore_char;
		status &= ~(PE | FE);
	}
	if (status & PE)
		uart->port.icount.parity++;
	if (status & OE)
		uart->port.icount.overrun++;
	if (status & FE)
		uart->port.icount.frame++;

	status &= uart->port.read_status_mask;

	if (status & BI)
		flg = TTY_BREAK;
	else if (status & PE)
		flg = TTY_PARITY;
	else if (status & FE)
		flg = TTY_FRAME;
	else
		flg = TTY_NORMAL;

	if (uart_handle_sysrq_char(&uart->port, ch))
		goto ignore_char;

	uart_insert_char(&uart->port, status, OE, ch, flg);

 ignore_char:
	tty_flip_buffer_push(&uart->port.state->port);
}

static void adi_uart4_serial_tx_chars(struct adi_uart4_serial_port *uart)
{
	struct circ_buf *xmit = &uart->port.state->xmit;

	if (uart_circ_empty(xmit) || uart_tx_stopped(&uart->port)) {
		/* Clear TFI bit */
		UART_PUT_LSR(uart, TFI);
		/* Anomaly notes:
		 *  05000215 -	we always clear ETBEI within last UART TX
		 *		interrupt to end a string. It is always set
		 *		when start a new tx.
		 */
		UART_CLEAR_IER(uart, ETBEI);
		return;
	}

	if (uart->port.x_char) {
		UART_PUT_CHAR(uart, uart->port.x_char);
		uart->port.icount.tx++;
		uart->port.x_char = 0;
	}

	while ((UART_GET_LSR(uart) & THRE) && xmit->tail != xmit->head) {
		UART_PUT_CHAR(uart, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uart->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uart->port);
}

static irqreturn_t adi_uart4_serial_rx_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;

	while (UART_GET_LSR(uart) & DR)
		adi_uart4_serial_rx_chars(uart);

	return IRQ_HANDLED;
}

static irqreturn_t adi_uart4_serial_tx_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;

	spin_lock(&uart->port.lock);
	if (UART_GET_LSR(uart) & THRE)
		adi_uart4_serial_tx_chars(uart);
	spin_unlock(&uart->port.lock);

	return IRQ_HANDLED;
}

static void adi_uart4_serial_dma_tx_chars(struct adi_uart4_serial_port *uart)
{
	struct dma_async_tx_descriptor *desc;
	struct circ_buf *xmit = &uart->port.state->xmit;

	uart->tx_done = 0;

	if (uart_circ_empty(xmit) || uart_tx_stopped(&uart->port)) {
		uart->tx_count = 0;
		uart->tx_done = 1;
		return;
	}

	if (uart->port.x_char) {
		UART_PUT_CHAR(uart, uart->port.x_char);
		uart->port.icount.tx++;
		uart->port.x_char = 0;
	}

	uart->tx_count = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);
	if (uart->tx_count > (UART_XMIT_SIZE - xmit->tail))
		uart->tx_count = UART_XMIT_SIZE - xmit->tail;

	sg_init_one(&uart->tx_sgl, xmit->buf + xmit->tail, uart->tx_count);
	dma_map_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);

	desc = dmaengine_prep_slave_sg(uart->tx_dma_channel, &uart->tx_sgl, 1,
		DMA_MEM_TO_DEV, 0);
	if (!desc) {
		dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);
		return;
	}

	desc->callback = adi_uart4_serial_dma_tx;
	desc->callback_param = uart;

	dmaengine_submit(desc);
	dma_async_issue_pending(uart->tx_dma_channel);
	UART_SET_IER(uart, ETBEI);
}

static void adi_uart4_serial_dma_rx_chars(struct adi_uart4_serial_port *uart)
{
	int i, flg, status;

	mod_timer(&uart->rx_dma_timer, jiffies + DMA_RX_FLUSH_JIFFIES);

	status = UART_GET_LSR(uart);
	UART_CLEAR_LSR(uart);

	uart->port.icount.rx +=
		CIRC_CNT(uart->rx_dma_buf.head, uart->rx_dma_buf.tail,
		UART_XMIT_SIZE);

	if (status & BI) {
		uart->port.icount.brk++;
		if (uart_handle_break(&uart->port))
			goto dma_ignore_char;
		status &= ~(PE | FE);
	}
	if (status & PE)
		uart->port.icount.parity++;
	if (status & OE)
		uart->port.icount.overrun++;
	if (status & FE)
		uart->port.icount.frame++;

	status &= uart->port.read_status_mask;

	if (status & BI)
		flg = TTY_BREAK;
	else if (status & PE)
		flg = TTY_PARITY;
	else if (status & FE)
		flg = TTY_FRAME;
	else
		flg = TTY_NORMAL;

	for (i = uart->rx_dma_buf.tail; ; i++) {
		if (i >= UART_XMIT_SIZE)
			i = 0;
		if (i == uart->rx_dma_buf.head)
			break;
		if (!uart_handle_sysrq_char(&uart->port, uart->rx_dma_buf.buf[i]))
			uart_insert_char(&uart->port, status, OE,
				uart->rx_dma_buf.buf[i], flg);
	}

 dma_ignore_char:
	tty_flip_buffer_push(&uart->port.state->port);
}

void adi_uart4_serial_rx_dma_timeout(struct timer_list *list)
{
	struct adi_uart4_serial_port *uart =
		container_of(list, struct adi_uart4_serial_port, rx_dma_timer);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned long flags;

	dmaengine_pause(uart->rx_dma_channel);
	spin_lock_irqsave(&uart->rx_lock, flags);

	status = dmaengine_tx_status(uart->rx_dma_channel, uart->rx_cookie, &state);

	if (DMA_ERROR == status) {
		dev_err(uart->dev, "Error in RX DMA\n");
		goto exit;
	}

	// Because resume will reset us to the start of the buffer, reset the tail
	// pointer to 0 after, but use the previous tail for an offset buffer slice
	// that timed out
	uart->rx_dma_buf.head = UART_XMIT_SIZE - state.residue;
	adi_uart4_serial_dma_rx_chars(uart);
	uart->rx_dma_buf.tail = 0;

exit:
	spin_unlock_irqrestore(&uart->rx_lock, flags);
	dmaengine_resume(uart->rx_dma_channel);
}

static void adi_uart4_serial_dma_tx(void *data)
{
	struct adi_uart4_serial_port *uart = data;
	struct circ_buf *xmit = &uart->port.state->xmit;
	unsigned long flags;

	dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);

	spin_lock_irqsave(&uart->port.lock, flags);
	/* Anomaly notes:
	 *  05000215 -	we always clear ETBEI within last UART TX
	 *		interrupt to end a string. It is always set
	 *		when start a new tx.
	 */
	UART_CLEAR_IER(uart, ETBEI);
	uart->port.icount.tx += uart->tx_count;
	if (!(xmit->tail == 0 && xmit->head == 0)) {
		xmit->tail = (xmit->tail + uart->tx_count) & (UART_XMIT_SIZE - 1);

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&uart->port);
	}

	adi_uart4_serial_dma_tx_chars(uart);
	spin_unlock_irqrestore(&uart->port.lock, flags);
}

static void adi_uart4_serial_dma_rx(void *data)
{
	struct adi_uart4_serial_port *uart = data;
	struct dma_tx_state state;
	enum dma_status status;
	unsigned long flags;
	int pos;

	spin_lock_irqsave(&uart->rx_lock, flags);

	status = dmaengine_tx_status(uart->rx_dma_channel, uart->rx_cookie, &state);

	if (DMA_ERROR == status) {
		dev_err(uart->dev, "Error in RX DMA\n");
		spin_unlock_irqrestore(&uart->rx_lock, flags);
		return;
	}

	// Update tail to start of the current block, so that we can receive multiple
	// full blocks or a partial block not at the start of the buffer in event of
	// a timeout
	uart->rx_dma_buf.head = UART_XMIT_SIZE - state.residue;
	pos = (uart->rx_dma_buf.head - 1) & (UART_XMIT_SIZE - 1);
	pos = (pos / DMA_RX_XCOUNT) * DMA_RX_XCOUNT;
	uart->rx_dma_buf.tail = pos;
	adi_uart4_serial_dma_rx_chars(uart);

	spin_unlock_irqrestore(&uart->rx_lock, flags);
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int adi_uart4_serial_tx_empty(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	unsigned int lsr;

	lsr = UART_GET_LSR(uart);
	if (lsr & TEMT)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void adi_uart4_serial_break_ctl(struct uart_port *port, int break_state)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	u32 lcr = UART_GET_LCR(uart);
	if (break_state)
		lcr |= SB;
	else
		lcr &= ~SB;
	UART_PUT_LCR(uart, lcr);
}

static int adi_uart4_serial_startup(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	struct dma_slave_config dma_config = {0};
	struct dma_async_tx_descriptor *desc;
	int ret;

	uart->tx_done = 1;

	if (uart->tx_dma_channel) {
		// RX channel
		uart->rx_dma_buf.buf = dmam_alloc_coherent(uart->dev, UART_XMIT_SIZE,
			&uart->rx_dma_phy, GFP_KERNEL);
		if (!uart->rx_dma_buf.buf)
			return -ENOMEM;

		uart->rx_dma_buf.head = 0;
		uart->rx_dma_buf.tail = 0;

		dma_config.direction = DMA_DEV_TO_MEM;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		// src_addr is not configured because we're attached to peripheral
		dma_config.src_maxburst = 1;
		dma_config.dst_maxburst = 1;

		ret = dmaengine_slave_config(uart->rx_dma_channel, &dma_config);
		if (ret) {
			dev_err(uart->dev, "Error configuring RX DMA channel\n");
			return -EINVAL;
		}

		desc = dmaengine_prep_dma_cyclic(uart->rx_dma_channel, uart->rx_dma_phy,
			UART_XMIT_SIZE, DMA_RX_XCOUNT, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
		desc->callback = adi_uart4_serial_dma_rx;
		desc->callback_param = uart;

		uart->rx_cookie = dmaengine_submit(desc);
		dma_async_issue_pending(uart->rx_dma_channel);

		timer_setup(&uart->rx_dma_timer, adi_uart4_serial_rx_dma_timeout, 0);
		mod_timer(&uart->rx_dma_timer, jiffies + DMA_RX_FLUSH_JIFFIES);

		// TX channel
		dma_config = (struct dma_slave_config) {0};
		dma_config.direction = DMA_MEM_TO_DEV;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		// dst_addr is not configured because we're attached to peripheral
		dma_config.dst_maxburst = 1;
		dma_config.src_maxburst = 1;

		ret = dmaengine_slave_config(uart->tx_dma_channel, &dma_config);
		if (ret) {
			dev_err(uart->dev, "Error configuring TX DMA channel\n");
			return -EINVAL;
		}
	} else {
		if (request_irq(uart->rx_irq, adi_uart4_serial_rx_int, 0,
			"ADI_UART_RX", uart)) {
			dev_err(port->dev, "Unable to attach UART RX int\n");
			return -EBUSY;
		}

		if (request_irq(uart->tx_irq, adi_uart4_serial_tx_int, 0,
			"ADI_UART_TX", uart)) {
			dev_err(port->dev, "Unable to attach UART TX int\n");
			free_irq(uart->rx_irq, uart);
			return -EBUSY;
		}
	}

	if (uart->hwflow_mode == ADI_UART_HWFLOW_GPIO) {
		if (request_irq(gpio_to_irq(uart->cts_pin),
			adi_uart4_serial_mctrl_cts_int,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			0, "ADI_UART_CTS", uart)) {
			uart->hwflow_mode = ADI_UART_NO_HWFLOW;
			dev_err(port->dev, "Unable to attach UART CTS int.\n");
		}
		if (gpio_request(uart->rts_pin, DRIVER_NAME)) {
			dev_err(port->dev,
				"fail to request RTS PIN at GPIO_%d\n",
				uart->rts_pin);
			uart->hwflow_mode = ADI_UART_NO_HWFLOW;
			free_irq(gpio_to_irq(uart->cts_pin), uart);
		} else
			gpio_direction_output(uart->rts_pin, 0);

	} else if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
		if (request_irq(uart->status_irq,
			adi_uart4_serial_mctrl_cts_int,
			0, "ADI_UART_MODEM_STATUS", uart)) {
			uart->hwflow_mode = ADI_UART_NO_HWFLOW;
			dev_info(port->dev,
				"Unable to attach UART Modem Status int.\n");
		}

		/* CTS RTS PINs are negative assertive. */
		UART_PUT_MCR(uart, UART_GET_MCR(uart) | ACTS);
		UART_SET_IER(uart, EDSSI);
	}

	UART_SET_IER(uart, ERBFI);
	return 0;
}

static void adi_uart4_serial_shutdown(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);

	dev_info(uart->dev, "in serial_shutdown\n");

	if (uart->tx_dma_channel) {
		dmaengine_terminate_sync(uart->tx_dma_channel);
		dmaengine_terminate_sync(uart->rx_dma_channel);
		dma_unmap_single(uart->dev, uart->tx_dma_phy, UART_XMIT_SIZE, DMA_TO_DEVICE);
		dmam_free_coherent(uart->dev, UART_XMIT_SIZE, uart->rx_dma_buf.buf,
			uart->rx_dma_phy);
		del_timer(&uart->rx_dma_timer);
	} else {
		free_irq(uart->rx_irq, uart);
		free_irq(uart->tx_irq, uart);
	}

	if (uart->hwflow_mode == ADI_UART_HWFLOW_GPIO) {
		free_irq(gpio_to_irq(uart->cts_pin), uart);
		gpio_free(uart->rts_pin);
	} else if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI)
		free_irq(uart->status_irq, uart);
}

static void adi_uart4_serial_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int ier, lcr = 0;
	unsigned long timeout;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI)
		termios->c_cflag |= CRTSCTS;

	switch (termios->c_cflag & CSIZE) {
	case CS8:
		lcr = WLS(8);
		break;
	case CS7:
		lcr = WLS(7);
		break;
	case CS6:
		lcr = WLS(6);
		break;
	case CS5:
		lcr = WLS(5);
		break;
	default:
		dev_err(port->dev, "%s: word length not supported\n",
			__func__);
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= STB;
	if (termios->c_cflag & PARENB)
		lcr |= PEN;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPS;
	if (termios->c_cflag & CMSPAR)
		lcr |= STP;
	if (termios->c_cflag & CRTSCTS)
		uart->hwflow_en = true;
	else
		uart->hwflow_en = false;

	spin_lock_irqsave(&uart->port.lock, flags);

	port->read_status_mask = OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= (FE | PE);
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= BI;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= FE | PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= OE;
	}

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);

	/* Wait till the transfer buffer is empty */
	timeout = jiffies + msecs_to_jiffies(10);
	while (UART_GET_GCTL(uart) & UCEN && !(UART_GET_LSR(uart) & TEMT))
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				"timeout waiting for TX buffer empty\n");
			break;
		}

	/* Wait till the transfer buffer is empty */
	timeout = jiffies + msecs_to_jiffies(10);
	while (UART_GET_GCTL(uart) & UCEN && !(UART_GET_LSR(uart) & TEMT))
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				"timeout waiting for TX buffer empty\n");
			break;
		}

	/* Disable UART */
	ier = UART_GET_IER(uart);
	UART_PUT_GCTL(uart, UART_GET_GCTL(uart) & ~UCEN);
	UART_DISABLE_INTS(uart);

	/* Set DLAB in LCR to Access CLK */
	UART_SET_DLAB(uart);

	UART_PUT_CLK(uart, quot);

	/* Clear DLAB in LCR to Access THR RBR IER */
	UART_CLEAR_DLAB(uart);

	UART_PUT_LCR(uart, (UART_GET_LCR(uart) & ~LCR_MASK) | lcr);

	/* Enable UART */
	UART_ENABLE_INTS(uart, ier);
	UART_PUT_GCTL(uart, UART_GET_GCTL(uart) | UCEN);

	/* Port speed changed, update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&uart->port.lock, flags);
}

static const char *adi_uart4_serial_type(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);

	return uart->port.type == PORT_BFIN ? "ADI-UART4" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void adi_uart4_serial_release_port(struct uart_port *port)
{
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int adi_uart4_serial_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void adi_uart4_serial_config_port(struct uart_port *port, int flags)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);

	if (flags & UART_CONFIG_TYPE &&
	    adi_uart4_serial_request_port(&uart->port) == 0)
		uart->port.type = PORT_BFIN;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_BFIN and PORT_UNKNOWN
 */
static int
adi_uart4_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

/*
 * Enable the IrDA function if tty->ldisc.num is N_IRDA.
 * In other cases, disable IrDA function.
 */
static void adi_uart4_serial_set_ldisc(struct uart_port *port, struct ktermios *termios)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	unsigned int val;

	switch (termios->c_line) {
	case N_IRDA:
		val = UART_GET_GCTL(uart);
		val |= (UMOD_IRDA | RPOLC);
		UART_PUT_GCTL(uart, val);
		break;
	default:
		val = UART_GET_GCTL(uart);
		val &= ~(UMOD_MASK | RPOLC);
		UART_PUT_GCTL(uart, val);
	}
}

static void adi_uart4_serial_reset_irda(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	unsigned int val;

	val = UART_GET_GCTL(uart);
	val &= ~(UMOD_MASK | RPOLC);
	UART_PUT_GCTL(uart, val);
	val |= (UMOD_IRDA | RPOLC);
	UART_PUT_GCTL(uart, val);
}

#ifdef CONFIG_CONSOLE_POLL
static void adi_uart4_serial_poll_put_char(struct uart_port *port, unsigned char chr)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);

	while (!(UART_GET_LSR(uart) & THRE))
		cpu_relax();

	UART_CLEAR_DLAB(uart);
	UART_PUT_CHAR(uart, (unsigned char)chr);
}

static int adi_uart4_serial_poll_get_char(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	unsigned char chr;

	while (!(UART_GET_LSR(uart) & DR))
		cpu_relax();

	UART_CLEAR_DLAB(uart);
	chr = UART_GET_CHAR(uart);

	return chr;
}
#endif


static struct uart_ops adi_uart4_serial_pops = {
	.tx_empty	= adi_uart4_serial_tx_empty,
	.set_mctrl	= adi_uart4_serial_set_mctrl,
	.get_mctrl	= adi_uart4_serial_get_mctrl,
	.stop_tx	= adi_uart4_serial_stop_tx,
	.start_tx	= adi_uart4_serial_start_tx,
	.stop_rx	= adi_uart4_serial_stop_rx,
	.enable_ms	= adi_uart4_serial_enable_ms,
	.break_ctl	= adi_uart4_serial_break_ctl,
	.startup	= adi_uart4_serial_startup,
	.shutdown	= adi_uart4_serial_shutdown,
	.set_termios	= adi_uart4_serial_set_termios,
	.set_ldisc	= adi_uart4_serial_set_ldisc,
	.type		= adi_uart4_serial_type,
	.release_port	= adi_uart4_serial_release_port,
	.request_port	= adi_uart4_serial_request_port,
	.config_port	= adi_uart4_serial_config_port,
	.verify_port	= adi_uart4_serial_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char	= adi_uart4_serial_poll_put_char,
	.poll_get_char	= adi_uart4_serial_poll_get_char,
#endif
};

#ifdef CONFIG_SERIAL_ADI_UART4_CONSOLE
static void adi_uart4_serial_console_putchar(struct uart_port *port, int ch)
{
	struct adi_uart4_serial_port *uart =
		container_of(port, struct adi_uart4_serial_port, port);
	while (!(UART_GET_LSR(uart) & THRE))
		barrier();
	UART_PUT_CHAR(uart, ch);
}

static void __init
adi_uart4_serial_console_get_options(struct adi_uart4_serial_port *uart, int *baud,
			   int *parity, int *bits)
{
	unsigned int status;

	status = UART_GET_IER(uart) & (ERBFI | ETBEI);
	if (status == (ERBFI | ETBEI)) {
		/* ok, the port was enabled */
		u32 lcr, clk;

		lcr = UART_GET_LCR(uart);

		*parity = 'n';
		if (lcr & PEN) {
			if (lcr & EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}
		*bits = ((lcr & WLS_MASK) >> WLS_OFFSET) + 5;

		/* Set DLAB in LCR to Access CLK */
		UART_SET_DLAB(uart);

		clk = UART_GET_CLK(uart);

		/* Clear DLAB in LCR to Access THR RBR IER */
		UART_CLEAR_DLAB(uart);

		*baud = 5000000 / (16*clk);
	}
	pr_debug("%s:baud = %d, parity = %c, bits= %d\n", __func__, *baud, *parity, *bits);
}
static void
adi_uart4_serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct adi_uart4_serial_port *uart = adi_uart4_serial_ports[co->index];
	unsigned long flags;

	spin_lock_irqsave(&uart->port.lock, flags);
	uart_console_write(&uart->port, s, count, adi_uart4_serial_console_putchar);
	spin_unlock_irqrestore(&uart->port.lock, flags);

}

static int __init
adi_uart4_serial_console_setup(struct console *co, char *options)
{
	struct adi_uart4_serial_port *uart;
	int baud = 57600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index < 0 || co->index >= ADI_UART_NR_PORTS)
		return -ENODEV;

	uart = adi_uart4_serial_ports[co->index];
	if (!uart)
		return -ENODEV;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI)
		flow = 'r';

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		adi_uart4_serial_console_get_options(uart, &baud, &parity, &bits);

	return uart_set_options(&uart->port, co, baud, parity, bits, flow);
}

static struct uart_driver adi_uart4_serial_reg;

static struct console adi_uart4_serial_console = {
	.name		= "ttySC",
	.write		= adi_uart4_serial_console_write,
	.device		= uart_console_device,
	.setup		= adi_uart4_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &adi_uart4_serial_reg,
};


#define ADI_SERIAL_UART4_CONSOLE &adi_uart4_serial_console
#else
#define ADI_SERIAL_UART4_CONSOLE NULL
#endif

static struct uart_driver adi_uart4_serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= DRIVER_NAME,
	.dev_name		= "ttySC",
	.major			= TTY_MAJOR,
#ifdef CONFIG_ARCH_SC59X_64
	//Other serial drivers are using 64 -- can probably disable in the future and set this back to 64
	.minor			= 74,
#else
	.minor			= 64,
#endif
	.nr			= ADI_UART_NR_PORTS,
	.cons			= ADI_SERIAL_UART4_CONSOLE,
};

static int adi_uart4_serial_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);

	return uart_suspend_port(&adi_uart4_serial_reg, &uart->port);
}

static int adi_uart4_serial_resume(struct platform_device *pdev)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);

	return uart_resume_port(&adi_uart4_serial_reg, &uart->port);
}

static int adi_uart4_serial_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk* clk;
	struct adi_uart4_serial_port *uart = NULL;
	int ret = 0;
	int uartid;
	struct dma_chan *tx_dma_channel = NULL;
	struct dma_chan *rx_dma_channel = NULL;

	dev_info(dev, "Serial probe\n");

	if (pdev->dev.of_node) {
		uartid = of_alias_get_id(pdev->dev.of_node, "serial");
		tx_dma_channel = dma_request_chan(dev, "tx");
		rx_dma_channel = dma_request_chan(dev, "rx");
	} else {
		uartid = pdev->id;
	}

	if (uartid < 0) {
		dev_err(&pdev->dev, "failed to get alias/pdev id, errno %d\n",
				uartid);
		ret = -ENODEV;
		return ret;
	}

	clk = clk_get(&pdev->dev, "adi-uart4");
	if (IS_ERR(clk)) {
		return -ENODEV;
	}

	if (adi_uart4_serial_ports[uartid] == NULL) {

		struct pinctrl *pctrl;
		struct pinctrl_state *pstate;

		pctrl = NULL;//devm_pinctrl_get(&pdev->dev);
		if (pctrl) {
			pstate = pinctrl_lookup_state(pctrl, PINCTRL_STATE_DEFAULT);
			if (IS_ERR(pstate))
				return ret;

			ret = pinctrl_select_state(pctrl, pstate);
			if (ret)
				return ret;
		}
		uart = kzalloc(sizeof(*uart), GFP_KERNEL);
		if (!uart) {
			dev_err(&pdev->dev,
				"fail to malloc adi_uart4_serial_port\n");
			return -ENOMEM;
		}

		adi_uart4_serial_ports[uartid] = uart;
		uart->dev = &pdev->dev;

		spin_lock_init(&uart->port.lock);
		uart->port.uartclk   = clk_get_rate(clk);
		uart->port.fifosize  = 8;
		uart->port.ops       = &adi_uart4_serial_pops;
		uart->port.line      = uartid;
		uart->port.iotype    = UPIO_MEM;
		uart->port.flags     = UPF_BOOT_AUTOCONF;
		clk_put(clk);

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (res == NULL) {
			dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
			ret = -ENOENT;
			goto out_error_unmap;
		}

		uart->tx_irq = platform_get_irq(pdev, 0);
		uart->rx_irq = platform_get_irq(pdev, 1);
		uart->status_irq = platform_get_irq(pdev, 2);
		uart->port.irq = uart->rx_irq;

		uart->port.membase = devm_ioremap(&pdev->dev, res->start,
						resource_size(res));
		if (!uart->port.membase) {
			dev_err(&pdev->dev, "Cannot map uart IO\n");
			return -ENXIO;
		}

		uart->tx_dma_channel = tx_dma_channel;
		uart->rx_dma_channel = rx_dma_channel;
		spin_lock_init(&uart->rx_lock);
		uart->tx_done	    = 1;
		uart->tx_count	    = 0;

		if (of_get_property(pdev->dev.of_node,
				"adi,uart-has-rtscts", NULL))
			uart->hwflow_mode = ADI_UART_HWFLOW_PERI;
		else
			uart->hwflow_mode = ADI_UART_NO_HWFLOW;

		if (pctrl && uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
			pstate = pinctrl_lookup_state(pctrl, "hwflow");
			ret = pinctrl_select_state(pctrl, pstate);
			if (ret)
				uart->hwflow_mode = ADI_UART_NO_HWFLOW;
		}

#ifndef CONFIG_ARCH_SC59X_64
		if (pdev->dev.of_node) {
			if (likely(of_count_phandle_with_args(pdev->dev.of_node,
							"enable-pin", NULL) > 0)) {
				ret = softconfig_of_set_active_pin_output(&pdev->dev,
							pdev->dev.of_node, "enable-pin", 0,
							&uart->enable_pin, &uart->enable_pin_active_low,
							true);
				if (ret)
					goto out_error_unmap;
			}

			if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI &&
						likely(of_count_phandle_with_args(pdev->dev.of_node,
							"hwflow-en-pin", NULL) > 0)) {
				ret = softconfig_of_set_active_pin_output(&pdev->dev,
							pdev->dev.of_node, "hwflow-en-pin", 0,
							&uart->hwflow_en_pin,
							&uart->hwflow_en_pin_active_low, true);
				if (ret) {
					uart->hwflow_mode = ADI_UART_NO_HWFLOW;
					goto out_error_unmap;
				}
			}
		}
#endif
	}


	uart = adi_uart4_serial_ports[uartid];
	uart->port.dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, uart);

	ret = uart_add_one_port(&adi_uart4_serial_reg, &uart->port);

	if (!ret) {
		return 0;
	}

	if (uart) {
out_error_unmap:
		kfree(uart);
		adi_uart4_serial_ports[uartid] = NULL;
	}

	return ret;
}

static int adi_uart4_serial_remove(struct platform_device *pdev)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (uart) {
		uart_remove_one_port(&adi_uart4_serial_reg, &uart->port);

		dma_release_channel(uart->tx_dma_channel);
		dma_release_channel(uart->rx_dma_channel);

		iounmap(uart->port.membase);
		if (uart->hwflow_en_pin && gpio_is_valid(uart->hwflow_en_pin))
			gpio_direction_output(uart->hwflow_en_pin,
						        uart->hwflow_en_pin_active_low ? 1 : 0);
		if (uart->enable_pin)
			gpio_direction_output(uart->enable_pin,
						        uart->enable_pin_active_low ? 1 : 0);
		kfree(uart);
		adi_uart4_serial_ports[uart->port.line] = NULL;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adi_uart_dt_match[] = {
	{ .compatible = "arm,adi-uart4"},
	{},
};
MODULE_DEVICE_TABLE(of, adi_uart_dt_match);
#endif

static struct platform_driver adi_uart4_serial_driver = {
	.probe		= adi_uart4_serial_probe,
	.remove		= adi_uart4_serial_remove,
	.suspend	= adi_uart4_serial_suspend,
	.resume		= adi_uart4_serial_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(adi_uart_dt_match),
#endif
	},
};

static int __init adi_uart4_serial_init(void)
{
	int ret;

	pr_info("ADI serial driver\n");

	ret = uart_register_driver(&adi_uart4_serial_reg);
	if (ret) {
		pr_err("failed to register %s:%d\n",
			adi_uart4_serial_reg.driver_name, ret);
	}

	ret = platform_driver_register(&adi_uart4_serial_driver);
	if (ret) {
		pr_err("fail to register ADI uart\n");
		uart_unregister_driver(&adi_uart4_serial_reg);
	}

	return ret;
}

static void __exit adi_uart4_serial_exit(void)
{
	platform_driver_unregister(&adi_uart4_serial_driver);
	uart_unregister_driver(&adi_uart4_serial_reg);
}

module_init(adi_uart4_serial_init);
module_exit(adi_uart4_serial_exit);


/* Early Console Support */

#define STAT_OFFSET	0x08
#define THR_OFFSET	0x24
#define THRE		(1 << 5)

static inline u32 adi_uart_read(struct uart_port *port, u32 off)
{
	return readl(port->membase + off);
}

static inline void adi_uart_write(struct uart_port *port, u32 val,
				  u32 off)
{
	writel(val, port->membase + off);
}


static void adi_uart_wait_bit_set(struct uart_port *port, unsigned int offset,
				  u32 bit)
{
	while (!(adi_uart_read(port, offset) & bit))
		cpu_relax();
}


static void adi_uart_console_putchar(struct uart_port *port, int ch)
{
	/* wait for the hardware fifo to clear up */
	adi_uart_wait_bit_set(port, STAT_OFFSET, THRE);

	/* queue the character for transmission */
	adi_uart_write(port, ch, THR_OFFSET);
}


static void adi_uart_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, adi_uart_console_putchar);
}


static int __init adi_uart_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = adi_uart_early_write;
	return 0;
}

EARLYCON_DECLARE(adi_uart, adi_uart_early_console_setup);
