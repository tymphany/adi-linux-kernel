/*
 * adi-nl8048.c NEC WVGA LCD NL8048HL11-01B driver for sc58x
 *
 * Copyright (c) 2012 - 2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/ppi.h>
#include <mach/dma.h>
#include <mach/portmux.h>
#include <mach/cpu.h>
#else
#include <asm/adi_ppi.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#endif

#define WIDTH 800
#define HEIGHT 480
#define BPP 16
#define MEM_SIZE (WIDTH * HEIGHT * 2)

struct adi_fb_par {
	u32 pseudo_pal[16];
	struct spi_device *spi;
	int dma_ch;
	int irq_err;
	struct adi_eppi3_regs *reg;
	int user;
	int cs;
};

static struct fb_fix_screeninfo adi_fb_fix = {
	.id             = KBUILD_MODNAME,
	.type           = FB_TYPE_PACKED_PIXELS,
	.visual         = FB_VISUAL_TRUECOLOR,
	.xpanstep       = 0,
	.ypanstep       = 0,
	.line_length    = WIDTH * BPP / 8,
	.accel          = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo adi_fb_var = {
	.bits_per_pixel         = BPP,
	.activate               = FB_ACTIVATE_TEST,
	.xres                   = WIDTH,
	.yres                   = HEIGHT,
	.xres_virtual           = WIDTH,
	.yres_virtual           = HEIGHT,
	.height                 = -1,
	.width                  = -1,
	.left_margin            = 0,
	.right_margin           = 0,
	.upper_margin           = 0,
	.lower_margin           = 0,
	.red                    = {11, 5, 0},
	.green                  = {5, 6, 0},
	.blue                   = {0, 5, 0},
	.transp                 = {0, 0, 0},
};

static u8 lcd_init_regs[] = {
	3, 0x01,
	0, 0x00,
	1, 0x01,
	4, 0x00,
	5, 0x14,
	6, 0x24,
	16, 0xD7,
	17, 0x00,
	18, 0x00,
	19, 0x55,
	20, 0x01,
	21, 0x70,
	22, 0x1E,
	23, 0x25,
	24, 0x25,
	25, 0x02,
	26, 0x02,
	27, 0xA0,
	32, 0x2F,
	33, 0x0F,
	34, 0x0F,
	35, 0x0F,
	36, 0x0F,
	37, 0x0F,
	38, 0x0F,
	39, 0x00,
	40, 0x02,
	41, 0x02,
	42, 0x02,
	43, 0x0F,
	44, 0x0F,
	45, 0x0F,
	46, 0x0F,
	47, 0x0F,
	48, 0x0F,
	49, 0x0F,
	50, 0x00,
	51, 0x02,
	52, 0x02,
	53, 0x02,
	80, 0x0C,
	83, 0x42,
	84, 0x42,
	85, 0x41,
	86, 0x14,
	89, 0x88,
	90, 0x01,
	91, 0x00,
	92, 0x02,
	93, 0x0C,
	94, 0x1C,
	95, 0x27,
	98, 0x49,
	99, 0x27,
	102, 0x76,
	103, 0x27,
	112, 0x01,
	113, 0x0E,
	114, 0x02,
	115, 0x0C,
	118, 0x0C,
	121, 0x20,
	130, 0x00,
	131, 0x00,
	132, 0xFC,
	134, 0x00,
	136, 0x00,
	138, 0x00,
	139, 0x00,
	140, 0x00,
	141, 0xFC,
	143, 0x00,
	145, 0x00,
	147, 0x00,
	148, 0x00,
	149, 0x00,
	150, 0xFC,
	152, 0x00,
	154, 0x00,
	156, 0x00,
	157, 0x00,
};

static int soft_switch_config(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	adapter = i2c_get_adapter(0);
	if (!adapter)
		return -EPROBE_DEFER;
	client = i2c_new_dummy(adapter, 0x27);
	i2c_smbus_write_byte_data(client, 0x03, 0xbf);
	i2c_smbus_write_byte_data(client, 0x05, 0xff);
	i2c_smbus_write_byte_data(client, 0x07, 0xdf);
	i2c_smbus_write_byte_data(client, 0x09, 0xff);
	i2c_smbus_write_byte_data(client, 0x0b, 0xb3);
	i2c_smbus_write_byte_data(client, 0x0d, 0x7f);

	i2c_smbus_write_byte_data(client, 0x02, 0x00);
	i2c_smbus_write_byte_data(client, 0x04, 0x00);
	i2c_smbus_write_byte_data(client, 0x06, 0x00);
	i2c_smbus_write_byte_data(client, 0x08, 0x00);
	i2c_smbus_write_byte_data(client, 0x0a, 0x00);
	i2c_smbus_write_byte_data(client, 0x0c, 0x00);
	i2c_unregister_device(client);
	i2c_put_adapter(adapter);
	return 0;
}

static int lcd_write_reg(struct spi_device *spi, u8 reg, u8 val)
{
	struct spi_message msg;
	struct spi_transfer x;
	u8 command[4];

	command[0] = 0;
	command[1] = reg;
	command[2] = 1;
	command[3] = val;
	spi_message_init(&msg);
	memset(&x, 0, sizeof(x));
	x.tx_buf = command;
	x.len = 4;
	spi_message_add_tail(&x, &msg);
	return spi_sync(spi, &msg);
}

static void lcd_write_regs(struct spi_device *spi, u8 *regs, int len)
{
	int i;

	for (i = 0; i < len; i += 2)
		lcd_write_reg(spi, regs[i], regs[i + 1]);
}

static int dummy_probe(struct spi_device *spi)
{
	return 0;
}

static int dummy_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver dummy_driver = {
	.driver.name = "dummy",
	.probe       = dummy_probe,
	.remove      = dummy_remove,
};

static int lcd_startup(struct adi_fb_par *par)
{
	int ret;
	struct spi_master *master;
	struct spi_board_info board = {
		.modalias               = "dummy",
		.max_speed_hz           = 5000000,
		.bus_num                = 0,
		.chip_select            = par->cs,
	};

	master = spi_busnum_to_master(0);
	if (!master)
		return -EPROBE_DEFER;
	par->spi = spi_new_device(master, &board);
	if (!par->spi)
		return -EINVAL;
	ret = spi_register_driver(&dummy_driver);
	if (ret < 0) {
		spi_unregister_device(par->spi);
		return ret;
	}
	lcd_write_regs(par->spi, lcd_init_regs, ARRAY_SIZE(lcd_init_regs));
	udelay(20);
	lcd_write_reg(par->spi, 2, 0x00);
	return 0;
}

static void lcd_shutdown(struct adi_fb_par *par)
{
	lcd_write_reg(par->spi, 16, 0x05);
	udelay(20);
	lcd_write_reg(par->spi, 16, 0x01);
	udelay(20);
	lcd_write_reg(par->spi, 16, 0x00);
	udelay(20);
	lcd_write_reg(par->spi, 3, 0x01);
	spi_unregister_device(par->spi);
	spi_unregister_driver(&dummy_driver);
}

static void start_ppi(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct adi_fb_par *par = info->par;
	struct adi_eppi3_regs *reg = par->reg;
	int dma_config, ppi_control;
	int bytes_per_line, dma_size;

	/* setup dma */
	dma_config = DMAFLOW_AUTO | RESTART | PSIZE_32 | DMA2D;

	bytes_per_line = var->xres * var->bits_per_pixel / 8;
	if (!(bytes_per_line % 32) && !(info->fix.smem_start % 32)) {
		dma_config |= WDSIZE_256;
		dma_size = 32;
	} else if (!(bytes_per_line % 16) && !(info->fix.smem_start % 16)) {
		dma_config |= WDSIZE_128;
		dma_size = 16;
	} else if (!(bytes_per_line % 8) && !(info->fix.smem_start % 8)) {
		dma_config |= WDSIZE_64;
		dma_size = 8;
	} else {
		dma_config |= WDSIZE_32;
		dma_size = 4;
	}
	set_dma_x_count(par->dma_ch, bytes_per_line / dma_size);
	set_dma_x_modify(par->dma_ch, dma_size);
	set_dma_y_count(par->dma_ch, var->yres);
	set_dma_y_modify(par->dma_ch, dma_size);
	set_dma_start_addr(par->dma_ch, info->fix.smem_start);
	set_dma_config(par->dma_ch, dma_config);

	/* setup ppi */
	ppi_control = EPPI_CTL_PACKEN | EPPI_CTL_DLEN16 | EPPI_CTL_FS1LO_FS2LO
			| EPPI_CTL_POLC0 | EPPI_CTL_IFSGEN | EPPI_CTL_SYNC2
			| EPPI_CTL_NON656 | EPPI_CTL_DIR | EPPI_CTL_EN;

	iowrite32(811, &reg->line);
	iowrite32(488, &reg->frame);
	iowrite32(5, &reg->hdly);
	iowrite32(5, &reg->vdly);
	iowrite32(var->xres, &reg->hcnt);
	iowrite32(var->yres, &reg->vcnt);
	iowrite32(1, &reg->fs1_wlhb);
	iowrite32(811, &reg->fs1_paspl);
	iowrite32(811, &reg->fs2_wlvb);
	iowrite32(811 * 488, &reg->fs2_palpf);

	enable_dma(par->dma_ch);
	iowrite32(ppi_control, &reg->ctl);
}

static void stop_ppi(struct fb_info *info)
{
	struct adi_fb_par *par = info->par;
	struct adi_eppi3_regs *reg = par->reg;

	iowrite32(0, &reg->ctl);
	clear_dma_irqstat(par->dma_ch);
	disable_dma(par->dma_ch);
}

static irqreturn_t eppi_irq_err(int irq, void *dev_id)
{
	struct fb_info *info = dev_id;
	struct adi_fb_par *par = info->par;
	struct adi_eppi3_regs *reg = par->reg;

	iowrite32(0xc0ff, &reg->stat);

	return IRQ_HANDLED;
}

static int adi_fb_open(struct fb_info *info, int user)
{
	struct adi_fb_par *par = info->par;
	int ret;

	if (!par->user) {
		ret = request_dma(par->dma_ch, "EPPI DMA");
		if (ret) {
			dev_err(info->dev, "Can't allocate DMA channel for EPPI\n");
			return ret;
		}

		ret = request_irq(par->irq_err, eppi_irq_err, 0, "EPPI ERROR", info);
		if (ret) {
			dev_err(info->dev, "Can't allocate IRQ for EPPI\n");
			goto err;
		}
		start_ppi(info);
	}
	par->user++;
	return 0;
err:
	free_dma(par->dma_ch);
	return ret;
}

static int adi_fb_release(struct fb_info *info, int user)
{
	struct adi_fb_par *par = info->par;

	par->user--;
	if (!par->user) {
		stop_ppi(info);
		free_irq(par->irq_err, info);
		free_dma(par->dma_ch);
	}
	return 0;
}

static int adi_fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	int bpp = var->bits_per_pixel;
	int line_length;
	/* check color depth */
	if (bpp != 16)
		return -EINVAL;

	/* various resolution checks */
	if (info->var.xres != var->xres
			|| info->var.yres != var->yres
			|| info->var.xres_virtual != var->xres_virtual
			|| info->var.yres_virtual != var->yres_virtual)
		return -EINVAL;

	/* check memory limit */
	line_length = var->xres_virtual * bpp / 8;
	if (line_length * var->yres_virtual > info->fix.smem_len)
		return -EINVAL;

	var->red = info->var.red;
	var->green = info->var.green;
	var->blue = info->var.blue;
	var->transp = info->var.transp;

	return 0;
}

static int adi_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return -EINVAL;
}

static struct fb_ops adi_fb_ops = {
	.owner                  = THIS_MODULE,
	.fb_open                = adi_fb_open,
	.fb_release             = adi_fb_release,
	.fb_check_var           = adi_fb_check_var,
	.fb_fillrect            = cfb_fillrect,
	.fb_copyarea            = cfb_copyarea,
	.fb_imageblit           = cfb_imageblit,
	.fb_cursor              = adi_fb_cursor,
};

#ifdef CONFIG_OF
static const struct of_device_id adi_lcd_of_match[] = {
	{
		.compatible = "sc58x,lcd",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_lcd_of_match);
#endif

static int adi_nl8048_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info;
	struct adi_fb_par *par;
	dma_addr_t dma_handle;
	struct resource *res;
	int ret;

#ifdef CONFIG_PINCTRL
	if (IS_ERR(devm_pinctrl_get_select(dev, "ppi_16b"))) {
		dev_err(dev, "Fail to request ppi peripheral pins.\n");
		return -EBUSY;
	}
#endif

	info = framebuffer_alloc(sizeof(struct adi_fb_par), dev);
	if (!info)
		return -ENOMEM;
	par = info->par;

	if (dev->of_node) {
		int id;
		ret = of_property_read_u32_index(dev->of_node,
					"dma-channel", 0, &par->dma_ch);
		if (ret) {
			dev_err(dev, "can not get dma node\n");
			goto err_mem;
		}
		ret = of_property_read_u32_index(dev->of_node,
					"cs-gpio", 0, &par->cs);
		if (ret) {
			dev_err(dev, "can not get cs gpio\n");
			goto err_mem;
		}
		ret = of_property_read_u32_index(dev->of_node,
					"spu_securep_id", 0, &id);
		if (ret) {
			dev_err(dev, "can not get secure peripheral id\n");
			goto err_mem;
		}
		set_spu_securep_msec(id, true);
	} else {
		res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
		if (!res) {
			dev_err(dev, "can not get dma resource\n");
			ret = -ENXIO;
			goto err_mem;
		}
		par->dma_ch = res->start;
		par->cs = (int)dev->platform_data;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "can not get irq resource\n");
		ret = -ENXIO;
		goto err_mem;
	}
	par->irq_err = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	par->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(par->reg)) {
		dev_err(dev, "can not get mem resource\n");
		ret = PTR_ERR(par->reg);
		goto err_mem;
	}

	par->user = 0;

	soft_switch_config();
	ret = lcd_startup(par);
	if (ret < 0) {
		dev_err(dev, "Can't start LCD\n");
		goto err_mem;
	}

	info->screen_base = dma_alloc_coherent(NULL, MEM_SIZE,
			&dma_handle, GFP_KERNEL);
	if (!info->screen_base) {
		ret = -ENOMEM;
		dev_err(dev, "Can't alloc dma buffer\n");
		goto err_spi;
	}
	adi_fb_fix.smem_start = (unsigned long)dma_handle;
	adi_fb_fix.smem_len = MEM_SIZE;
	info->fix = adi_fb_fix;
	info->var = adi_fb_var;
	info->fbops = &adi_fb_ops;
	info->pseudo_palette = par->pseudo_pal;
	info->flags = FBINFO_DEFAULT;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0) {
		dev_err(dev, "Can't alloc cmap\n");
		goto err_dma;
	}

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "Can't register frame buffer\n");
		goto err_cmap;
	}
	
	platform_set_drvdata(pdev, info);

	dev_info(dev, "LCD %s registered@%p,dma=%d,irq=%d,cs=%d\n",
			dev_name(info->dev), par->reg, par->dma_ch,
			par->irq_err, par->cs);
	return 0;
err_cmap:
	fb_dealloc_cmap(&info->cmap);
err_dma:
	dma_free_coherent(NULL, MEM_SIZE, info->screen_base, 0);
err_spi:
	lcd_shutdown(info->par);
err_mem:
	framebuffer_release(info);
	return ret;
}

static int adi_nl8048_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	lcd_shutdown(info->par);
	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	dma_free_coherent(NULL, MEM_SIZE, info->screen_base, 0);
	framebuffer_release(info);
	return 0;
}
static int __maybe_unused adi_nl8048_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct adi_fb_par *par = info->par;

	if (par->user)
		stop_ppi(info);
	lcd_write_reg(par->spi, 2, 0x01);
	mdelay(40);
	return 0;
}
static int __maybe_unused adi_nl8048_resume(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct adi_fb_par *par = info->par;

	soft_switch_config();
	lcd_write_reg(par->spi, 2, 0x00);
	if (par->user)
		start_ppi(info);
	return 0;
}

static struct platform_driver adi_nl8048_driver = {
	.probe  = adi_nl8048_probe,
	.remove = adi_nl8048_remove,
	.suspend = adi_nl8048_suspend,
	.resume = adi_nl8048_resume,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(adi_lcd_of_match),
	},
};
module_platform_driver(adi_nl8048_driver);

MODULE_DESCRIPTION("NEC WVGA LCD NL8048HL11-01B driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
