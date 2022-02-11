// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SPI3 controller driver
 *
 * Copyright (c) 2014 - 2022 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/adi_spi3.h>
#include <linux/types.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <linux/soc/adi/portmux.h>
#else
#include <asm/portmux.h>
#endif

enum adi_spi_state {
	START_STATE,
	RUNNING_STATE,
	DONE_STATE,
	ERROR_STATE
};

struct adi_spi_master;

struct adi_spi_transfer_ops {
	void (*write) (struct adi_spi_master *);
	void (*read) (struct adi_spi_master *);
	void (*duplex) (struct adi_spi_master *);
};

/* runtime info for spi master */
struct adi_spi_master {
	/* SPI framework hookup */
	struct spi_master *master;
	struct device *dev;

	/* Regs base of SPI controller */
	struct adi_spi_regs __iomem *regs;

	/* Pin request list */
	u16 *pin_req;

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct adi_spi_device *cur_chip;
	unsigned transfer_len;

	/* transfer buffer */
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;

	/* dma info */
	struct dma_chan *tx_dma;
	struct dma_chan *rx_dma;
	int dummy_tx;
	int dummy_rx;
	int tx_mapped;
	int rx_mapped;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;
	struct scatterlist tx_sgl;
	struct scatterlist rx_sgl;
	int tx_num;
	int rx_num;

	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

	struct clk *sclk;
	unsigned long sclk_rate;
	enum adi_spi_state state;

	const struct adi_spi_transfer_ops *ops;
};

struct adi_spi_device {
	u32 control;
	u32 clock;
	u32 ssel;

	u16 cs_chg_udelay; /* Some devices require > 255usec delay */
	u32 cs_gpio;
	u32 tx_dummy_val; /* tx value for rx only transfer */
	bool enable_dma;
	const struct adi_spi_transfer_ops *ops;
};

static void adi_spi_rx_dma_isr(void *data);

static void adi_spi_enable(struct adi_spi_master *drv_data)
{
	u32 ctl;

	ctl = ioread32(&drv_data->regs->control);
	ctl |= SPI_CTL_EN;
	iowrite32(ctl, &drv_data->regs->control);
}

static void adi_spi_disable(struct adi_spi_master *drv_data)
{
	u32 ctl;

	ctl = ioread32(&drv_data->regs->control);
	ctl &= ~SPI_CTL_EN;
	iowrite32(ctl, &drv_data->regs->control);
}

static void adi_spi_dma_unmap_tx(struct adi_spi_master *drv_data) {
	if (drv_data->tx && drv_data->dummy_tx) {
		kfree(drv_data->tx);
		drv_data->tx = NULL;
		drv_data->dummy_tx = 0;
	}

	if (drv_data->tx_mapped) {
		dma_unmap_sg(drv_data->dev, &drv_data->tx_sgl, 1, DMA_TO_DEVICE);
		drv_data->tx_mapped = 0;
	}
}

static void adi_spi_dma_unmap_rx(struct adi_spi_master *drv_data) {
	if (drv_data->rx && drv_data->dummy_rx) {
		kfree(drv_data->tx);
		drv_data->tx = NULL;
		drv_data->dummy_tx = 0;
	}

	if (drv_data->rx_mapped) {
		dma_unmap_sg(drv_data->dev, &drv_data->rx_sgl, 1, DMA_FROM_DEVICE);
		drv_data->rx_mapped = 0;
	}
}

static void adi_spi_dma_unmap(struct adi_spi_master *drv_data) {
	adi_spi_dma_unmap_tx(drv_data);
	adi_spi_dma_unmap_rx(drv_data);
}

static void adi_spi_dma_terminate(struct adi_spi_master *drv_data) {
	dmaengine_terminate_sync(drv_data->tx_dma);
	dmaengine_terminate_sync(drv_data->rx_dma);
}

/* Caculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 sclk, u32 speed_hz)
{
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static int adi_spi_flush(struct adi_spi_master *drv_data)
{
	unsigned long limit = loops_per_jiffy << 1;

	/* wait for stop and clear stat */
	while (!(ioread32(&drv_data->regs->status) & SPI_STAT_SPIF) && --limit)
		cpu_relax();

	iowrite32(0xFFFFFFFF, &drv_data->regs->status);

	return limit;
}

/* Chip select operation functions for cs_change flag */
static void adi_spi_cs_active(struct adi_spi_master *drv_data, struct adi_spi_device *chip)
{
	gpio_set_value(chip->cs_gpio, 0);
}

static void adi_spi_cs_deactive(struct adi_spi_master *drv_data,
				struct adi_spi_device *chip)
{
	gpio_set_value(chip->cs_gpio, 1);

	/* Move delay here for consistency */
	if (chip->cs_chg_udelay)
		udelay(chip->cs_chg_udelay);
}

/* stop controller and re-config current chip*/
static void adi_spi_restore_state(struct adi_spi_master *drv_data)
{
	struct adi_spi_device *chip = drv_data->cur_chip;

	/* Clear status and disable clock */
	iowrite32(0xFFFFFFFF, &drv_data->regs->status);
	iowrite32(0x0, &drv_data->regs->rx_control);
	iowrite32(0x0, &drv_data->regs->tx_control);
	adi_spi_disable(drv_data);

	/* Load the registers */
	iowrite32(chip->control, &drv_data->regs->control);
	iowrite32(chip->clock, &drv_data->regs->clock);

	adi_spi_enable(drv_data);
	drv_data->tx_num = drv_data->rx_num = 0;
	adi_spi_cs_active(drv_data, chip);
}

/* discard invalid rx data and empty rfifo */
static inline void dummy_read(struct adi_spi_master *drv_data)
{
	while (!(ioread32(&drv_data->regs->status) & SPI_STAT_RFE))
		ioread32(&drv_data->regs->rfifo);
}

static void adi_spi_u8_write(struct adi_spi_master *drv_data)
{
	while (drv_data->tx < drv_data->tx_end) {
		iowrite32(*(u8 *)(drv_data->tx++), &drv_data->regs->tfifo);
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv_data->regs->rfifo);
	}
}

static void adi_spi_u8_read(struct adi_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;
	struct spi_transfer *t = drv_data->cur_transfer;

	while (drv_data->rx < drv_data->rx_end) {
		if (t->rx_nbits != SPI_NBITS_QUAD)
			iowrite32(tx_val, &drv_data->regs->tfifo);
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = ioread32(&drv_data->regs->rfifo);
	}
}

static void adi_spi_u8_duplex(struct adi_spi_master *drv_data)
{
	while (drv_data->rx < drv_data->rx_end) {
		iowrite32(*(u8 *)(drv_data->tx++), &drv_data->regs->tfifo);
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = ioread32(&drv_data->regs->rfifo);
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u8 = {
	.write  = adi_spi_u8_write,
	.read   = adi_spi_u8_read,
	.duplex = adi_spi_u8_duplex,
};

static void adi_spi_u16_write(struct adi_spi_master *drv_data)
{
	while (drv_data->tx < drv_data->tx_end) {
		iowrite32(*(u16 *)drv_data->tx, &drv_data->regs->tfifo);
		drv_data->tx += 2;
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv_data->regs->rfifo);
	}
}

static void adi_spi_u16_read(struct adi_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;
	struct spi_transfer *t = drv_data->cur_transfer;

	while (drv_data->rx < drv_data->rx_end) {
		if (t->rx_nbits != SPI_NBITS_QUAD)
			iowrite32(tx_val, &drv_data->regs->tfifo);
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = ioread32(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static void adi_spi_u16_duplex(struct adi_spi_master *drv_data)
{
	while (drv_data->rx < drv_data->rx_end) {
		iowrite32(*(u16 *)drv_data->tx, &drv_data->regs->tfifo);
		drv_data->tx += 2;
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = ioread32(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u16 = {
	.write  = adi_spi_u16_write,
	.read   = adi_spi_u16_read,
	.duplex = adi_spi_u16_duplex,
};

static void adi_spi_u32_write(struct adi_spi_master *drv_data)
{
	while (drv_data->tx < drv_data->tx_end) {
		iowrite32(*(u32 *)drv_data->tx, &drv_data->regs->tfifo);
		drv_data->tx += 4;
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv_data->regs->rfifo);
	}
}

static void adi_spi_u32_read(struct adi_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;
	struct spi_transfer *t = drv_data->cur_transfer;

	while (drv_data->rx < drv_data->rx_end) {
		if (t->rx_nbits != SPI_NBITS_QUAD)
			iowrite32(tx_val, &drv_data->regs->tfifo);
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = ioread32(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static void adi_spi_u32_duplex(struct adi_spi_master *drv_data)
{
	while (drv_data->rx < drv_data->rx_end) {
		iowrite32(*(u32 *)drv_data->tx, &drv_data->regs->tfifo);
		drv_data->tx += 4;
		while (ioread32(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = ioread32(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u32 = {
	.write  = adi_spi_u32_write,
	.read   = adi_spi_u32_read,
	.duplex = adi_spi_u32_duplex,
};


/* test if there is more transfer to be done */
static void adi_spi_next_transfer(struct adi_spi_master *drv)
{
	struct spi_message *msg = drv->cur_msg;
	struct spi_transfer *t = drv->cur_transfer;

	/* Move to next transfer */
	if (t->transfer_list.next != &msg->transfers) {
		drv->cur_transfer = list_entry(t->transfer_list.next,
			       struct spi_transfer, transfer_list);
		drv->state = RUNNING_STATE;
	} else {
		drv->state = DONE_STATE;
		drv->cur_transfer = NULL;
	}
}

static void adi_spi_giveback(struct adi_spi_master *drv_data)
{
	struct adi_spi_device *chip = drv_data->cur_chip;

	adi_spi_cs_deactive(drv_data, chip);
	spi_finalize_current_message(drv_data->master);
}

static int adi_spi_setup_transfer(struct adi_spi_master *drv)
{
	struct spi_transfer *t = drv->cur_transfer;
	u32 cr, cr_width;

	if (t->tx_buf) {
		drv->tx = (void *)t->tx_buf;
		drv->tx_end = drv->tx + t->len;
	} else {
		drv->tx = NULL;
	}

	if (t->rx_buf) {
		drv->rx = t->rx_buf;
		drv->rx_end = drv->rx + t->len;
	} else {
		drv->rx = NULL;
	}

	drv->transfer_len = t->len;

	/* bits per word setup */
	switch (t->bits_per_word) {
	case 8:
		cr_width = SPI_CTL_SIZE08;
		drv->ops = &adi_spi_transfer_ops_u8;
		break;
	case 16:
		cr_width = SPI_CTL_SIZE16;
		drv->ops = &adi_spi_transfer_ops_u16;
		break;
	case 32:
		cr_width = SPI_CTL_SIZE32;
		drv->ops = &adi_spi_transfer_ops_u32;
		break;
	default:
		return -EINVAL;
	}
	cr = ioread32(&drv->regs->control) & ~SPI_CTL_SIZE;
	cr |= cr_width;

	cr &= ~SPI_CTL_SOSI;
	cr &= ~SPI_CTL_MIOM;
	if (t->rx_nbits == SPI_NBITS_QUAD || t->tx_nbits == SPI_NBITS_QUAD)
		cr |= SPI_CTL_MIO_QUAD;
	else if (t->rx_nbits == SPI_NBITS_DUAL || t->tx_nbits == SPI_NBITS_DUAL)
		cr |= SPI_CTL_MIO_DUAL;

	iowrite32(cr, &drv->regs->control);

	/* speed setup */
	iowrite32(hz_to_spi_clock(drv->sclk_rate, t->speed_hz), &drv->regs->clock);
	return 0;
}

static int adi_spi_dma_xfer(struct adi_spi_master *drv_data)
{
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;

	dummy_read(drv_data);

	drv_data->dummy_rx = 0;
	drv_data->dummy_tx = 0;

	if (!drv_data->rx) {
		drv_data->rx = kzalloc(drv_data->transfer_len, GFP_ATOMIC);
		drv_data->dummy_rx = 1;
		if (!drv_data->rx)
			goto cleanup;
	}

	if (!drv_data->tx) {
		drv_data->tx = kzalloc(drv_data->transfer_len, GFP_ATOMIC);
		drv_data->dummy_tx = 1;
		if (!drv_data->tx)
			goto cleanup;
	}

	sg_init_one(&drv_data->tx_sgl, drv_data->tx, drv_data->transfer_len);
	drv_data->tx_mapped = dma_map_sg(drv_data->dev, &drv_data->tx_sgl, 1,
		DMA_TO_DEVICE);

	if (!drv_data->tx_mapped) {
		dev_err(drv_data->dev, "unable to map tx buffer\n");
		goto cleanup;
	}

	tx_desc = dmaengine_prep_slave_sg(drv_data->tx_dma, &drv_data->tx_sgl, 1,
		DMA_MEM_TO_DEV, 0);

	if (!tx_desc) {
		dev_err(drv_data->dev, "unable to allocate tx dma descriptor\n");
		goto cleanup;
	}

	drv_data->tx_cookie = dmaengine_submit(tx_desc);
	dma_async_issue_pending(drv_data->tx_dma);

	sg_init_one(&drv_data->rx_sgl, drv_data->rx, drv_data->transfer_len);
	drv_data->rx_mapped = dma_map_sg(drv_data->dev, &drv_data->rx_sgl, 1,
		DMA_FROM_DEVICE);

	if (!drv_data->rx_mapped) {
		dev_err(drv_data->dev, "unable to map rx buffer\n");
		goto cleanup;
	}

	rx_desc = dmaengine_prep_slave_sg(drv_data->rx_dma, &drv_data->rx_sgl, 1,
		DMA_DEV_TO_MEM, 0);

	if (!rx_desc) {
		dev_err(drv_data->dev, "unable to allocate rx dma descriptor\n");
		goto cleanup;
	}

	rx_desc->callback = adi_spi_rx_dma_isr;
	rx_desc->callback_param = drv_data;
	drv_data->rx_cookie = dmaengine_submit(rx_desc);
	dma_async_issue_pending(drv_data->rx_dma);

	// Make sure we start rx/tx at the same time by disabling everything before
	// configuring them to use RTI+TTI
	adi_spi_disable(drv_data);
	iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE,
			&drv_data->regs->rx_control);
	iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TDR_NF,
			&drv_data->regs->tx_control);
	adi_spi_enable(drv_data);

	return 0;

cleanup:
	adi_spi_dma_terminate(drv_data);
	adi_spi_dma_unmap(drv_data);
	return -ENOENT;
}

static int adi_spi_pio_xfer(struct adi_spi_master *drv_data)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer *t = drv_data->cur_transfer;

	dummy_read(drv_data);

	iowrite32(SPI_RXCTL_REN,
			&drv_data->regs->rx_control);
	iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI,
			&drv_data->regs->tx_control);

	if (!drv_data->rx) {
		/* write only half duplex */
		drv_data->ops->write(drv_data);
		if (drv_data->tx != drv_data->tx_end)
			return -EIO;
	} else if (!drv_data->tx) {
		/* read only half duplex */
		if (t->rx_nbits == SPI_NBITS_QUAD) {
			iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI,
					&drv_data->regs->rx_control);
			iowrite32(0, &drv_data->regs->tx_control);
		}

		drv_data->ops->read(drv_data);
		if (drv_data->rx != drv_data->rx_end)
			return -EIO;
	} else {
		/* full duplex mode */
		drv_data->ops->duplex(drv_data);
		if (drv_data->tx != drv_data->tx_end)
			return -EIO;
	}

	if (!adi_spi_flush(drv_data))
		return -EIO;
	msg->actual_length += drv_data->transfer_len;
	tasklet_schedule(&drv_data->pump_transfers);
	return 0;
}

static void adi_spi_pump_transfers(unsigned long data)
{
	struct adi_spi_master *drv_data = (struct adi_spi_master *)data;
	struct spi_message *msg = NULL;
	struct spi_transfer *t = NULL;
	struct adi_spi_device *chip = NULL;
	int ret;

	/* Get current state information */
	msg = drv_data->cur_msg;
	t = drv_data->cur_transfer;
	chip = drv_data->cur_chip;

	/* Handle for abort */
	if (drv_data->state == ERROR_STATE) {
		msg->status = -EIO;
		adi_spi_giveback(drv_data);
		return;
	}

	if (drv_data->state == RUNNING_STATE) {
		if (t->delay_usecs)
			udelay(t->delay_usecs);
		if (t->cs_change)
			adi_spi_cs_deactive(drv_data, chip);
		adi_spi_next_transfer(drv_data);
		t = drv_data->cur_transfer;
	}
	/* Handle end of message */
	if (drv_data->state == DONE_STATE) {
		msg->status = 0;
		adi_spi_giveback(drv_data);
		return;
	}

	if ((t->len == 0) || (t->tx_buf == NULL && t->rx_buf == NULL)) {
		/* Schedule next transfer tasklet */
		tasklet_schedule(&drv_data->pump_transfers);
		return;
	}

	ret = adi_spi_setup_transfer(drv_data);
	if (ret) {
		msg->status = ret;
		adi_spi_giveback(drv_data);
	}

	iowrite32(0xFFFFFFFF, &drv_data->regs->status);
	adi_spi_cs_active(drv_data, chip);
	drv_data->state = RUNNING_STATE;

	if (chip->enable_dma)
		ret = adi_spi_dma_xfer(drv_data);
	else
		ret = adi_spi_pio_xfer(drv_data);
	if (ret) {
		msg->status = ret;
		adi_spi_giveback(drv_data);
	}
}

static int adi_spi_transfer_one_message(struct spi_master *master,
					struct spi_message *m)
{
	struct adi_spi_master *drv_data = spi_master_get_devdata(master);

	drv_data->cur_msg = m;
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);
	adi_spi_restore_state(drv_data);

	drv_data->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
					    struct spi_transfer, transfer_list);

	tasklet_schedule(&drv_data->pump_transfers);
	return 0;
}

static int adi_spi_setup(struct spi_device *spi)
{
	struct adi_spi_master *drv_data = spi_master_get_devdata(spi->master);
	struct adi_spi_device *chip = spi_get_ctldata(spi);
	u32 ctl_reg = SPI_CTL_ODM | SPI_CTL_PSSE;
	int ret = -EINVAL;

	if (!chip) {
		struct adi_spi3_chip *chip_info = spi->controller_data;

		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		if (chip_info) {
			if (chip_info->control & ~ctl_reg) {
				dev_err(&spi->dev,
					"do not set bits that the SPI framework manages\n");
				goto error;
			}
			chip->control = chip_info->control;
			chip->cs_chg_udelay = chip_info->cs_chg_udelay;
			chip->tx_dummy_val = chip_info->tx_dummy_val;
			chip->enable_dma = chip_info->enable_dma;
		} else if (spi->dev.of_node) {
			if (of_find_property(spi->dev.of_node,
						"dma-mode", NULL))
				chip->enable_dma = true;
		}
		chip->cs_gpio = spi->chip_select;
		ret = gpio_request_one(chip->cs_gpio, GPIOF_OUT_INIT_HIGH,
					dev_name(&spi->dev));
		if (ret) {
			dev_err(&spi->dev, "gpio_request_one() error\n");
			goto error;
		}
		spi_set_ctldata(spi, chip);
	}

	/* force a default base state */
	chip->control &= ctl_reg;

	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	chip->control |= SPI_CTL_MSTR;
	/* we choose software to controll cs */
	chip->control &= ~SPI_CTL_ASSEL;

	chip->clock = hz_to_spi_clock(drv_data->sclk_rate, spi->max_speed_hz);

	adi_spi_cs_deactive(drv_data, chip);

	return 0;
error:
	if (chip) {
		kfree(chip);
		spi_set_ctldata(spi, NULL);
	}

	return ret;
}

static void adi_spi_cleanup(struct spi_device *spi)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (!chip)
		return;

	gpio_free(chip->cs_gpio);
	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

static void adi_spi_rx_dma_isr(void *data)
{
	struct adi_spi_master *drv_data = data;
	struct spi_message *msg = drv_data->cur_msg;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv_data->rx_dma, drv_data->rx_cookie, &state);
	if (DMA_ERROR == status) {
		drv_data->state = ERROR_STATE;
		dev_err(&drv_data->master->dev, "spi rx dma error\n");
	}

	msg->actual_length += drv_data->transfer_len;
	iowrite32(0, &drv_data->regs->tx_control);
	iowrite32(0, &drv_data->regs->rx_control);

	adi_spi_dma_unmap(drv_data);
	tasklet_schedule(&drv_data->pump_transfers);
}

static irqreturn_t spi_irq_err(int irq, void *dev_id)
{
	struct adi_spi_master *drv_data = dev_id;
	u32 status;

	status = ioread32(&drv_data->regs->status);
	if (status & SPI_STAT_ROE)
		dev_err(&drv_data->master->dev, "spi rx overrun\n");
	iowrite32(status, &drv_data->regs->status);
	drv_data->state = ERROR_STATE;
	iowrite32(0, &drv_data->regs->tx_control);
	iowrite32(0, &drv_data->regs->rx_control);

	adi_spi_dma_terminate(drv_data);
	adi_spi_dma_unmap(drv_data);
	tasklet_schedule(&drv_data->pump_transfers);
	return IRQ_HANDLED;
}

static const struct of_device_id adi_spi_of_match[] = {
	{
		.compatible = "adi,spi3",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_spi_of_match);

static int adi_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_spi3_master *info = dev_get_platdata(dev);
	struct spi_master *master;
	struct adi_spi_master *drv_data;
	struct resource *mem, *res;
	unsigned int num_cs;
	struct clk *sclk;
	int ret;

	if (!info && !dev->of_node) {
		dev_err(dev, "platform data missing!\n");
		return -ENODEV;
	}

	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	ret = of_property_read_u32(dev->of_node, "num-cs", &num_cs);
	if (ret) {
		dev_err(dev, "can not get total number of cs\n");
		return ret;
	}

	/* allocate master with space for drv_data */
	master = devm_spi_alloc_master(dev, sizeof(*drv_data));
	if (!master) {
		dev_err(dev, "can not alloc spi_master\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, master);

	/* the mode bits supported by this driver */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST |
				SPI_TX_DUAL | SPI_TX_QUAD |
				SPI_RX_DUAL | SPI_RX_QUAD;

	master->dev.of_node = dev->of_node;
	master->bus_num = -1;
	master->num_chipselect = num_cs;
	master->cleanup = adi_spi_cleanup;
	master->setup = adi_spi_setup;
	master->transfer_one_message = adi_spi_transfer_one_message;
	master->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);

	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;
	drv_data->sclk = sclk;
	drv_data->sclk_rate = clk_get_rate(sclk);
	drv_data->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv_data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv_data->regs)) {
		dev_err(dev, "Could not map spiv3 memory, check device tree\n");
		return PTR_ERR(drv_data->regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "can not get spi error irq\n");
		return -ENXIO;
	}
	ret = devm_request_irq(dev, res->start, spi_irq_err,
			0, "SPI ERROR", drv_data);
	if (ret) {
		dev_err(dev, "can not request spi error irq\n");
		return ret;
	}

	iowrite32(SPI_CTL_MSTR | SPI_CTL_CPHA, &drv_data->regs->control);
	iowrite32(0x0000FE00, &drv_data->regs->ssel);
	iowrite32(0x0, &drv_data->regs->delay);
	iowrite32(SPI_IMSK_SET_ROM, &drv_data->regs->emaskst);

	tasklet_init(&drv_data->pump_transfers,
			adi_spi_pump_transfers, (unsigned long)drv_data);

	drv_data->tx_dma = dma_request_chan(dev, "tx");
	if (!drv_data->tx_dma) {
		dev_err(dev, "Could not get TX DMA channel\n");
		return -ENOENT;
	}

	drv_data->rx_dma = dma_request_chan(dev, "rx");
	if (!drv_data->rx_dma) {
		dev_err(dev, "Could not get RX DMA channel\n");
		ret = -ENOENT;
		goto err_free_tx_dma;
	}

	ret = clk_prepare_enable(drv_data->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");
		goto err_free_rx_dma;
	}

	/* register with the SPI framework */
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(dev, "can not  register spi master\n");
		goto err_free_rx_dma;
	}

	dev_info(dev, "registered ADI SPI controller %s\n",
					dev_name(&master->dev));
	return ret;

err_free_rx_dma:
	dma_release_channel(drv_data->rx_dma);

err_free_tx_dma:
	dma_release_channel(drv_data->tx_dma);

	return ret;
}

static int adi_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct adi_spi_master *drv_data = spi_master_get_devdata(master);

	adi_spi_disable(drv_data);
	clk_disable_unprepare(drv_data->sclk);
	dma_release_channel(drv_data->tx_dma);
	dma_release_channel(drv_data->rx_dma);
	return 0;
}

static int __maybe_unused adi_spi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct adi_spi_master *drv_data = spi_master_get_devdata(master);

	spi_master_suspend(master);

	drv_data->control = ioread32(&drv_data->regs->control);
	drv_data->ssel = ioread32(&drv_data->regs->ssel);

	iowrite32(SPI_CTL_MSTR | SPI_CTL_CPHA, &drv_data->regs->control);
	iowrite32(0x0000FE00, &drv_data->regs->ssel);
	dmaengine_pause(drv_data->tx_dma);
	dmaengine_pause(drv_data->rx_dma);

	return 0;
}

static int __maybe_unused adi_spi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct adi_spi_master *drv_data = spi_master_get_devdata(master);
	int ret = 0;

	/* bootrom may modify spi and dma status when resume in spi boot mode */
	dmaengine_terminate_sync(drv_data->rx_dma);

	dmaengine_resume(drv_data->rx_dma);
	dmaengine_resume(drv_data->tx_dma);
	iowrite32(drv_data->control, &drv_data->regs->control);
	iowrite32(drv_data->ssel, &drv_data->regs->ssel);

	ret = spi_master_resume(master);
	if (ret) {
		dma_release_channel(drv_data->tx_dma);
		dma_release_channel(drv_data->rx_dma);
	}

	return ret;
}

static const struct dev_pm_ops adi_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adi_spi_suspend, adi_spi_resume)
};

MODULE_ALIAS("platform:adi-spi3");
static struct platform_driver adi_spi_driver = {
	.driver	= {
		.name	= "adi-spi3",
		.pm     = &adi_spi_pm_ops,
		.of_match_table = of_match_ptr(adi_spi_of_match),
	},
	.remove		= adi_spi_remove,
};

module_platform_driver_probe(adi_spi_driver, adi_spi_probe);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
