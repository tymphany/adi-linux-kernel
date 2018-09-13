/*
 * Analog Devices video capture driver
 *
 * Copyright (c) 2011 - 2018 Analog Devices Inc.
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

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/adi/adi_capture.h>
#include <media/adi/ppi.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/dma.h>
#else
#include <asm/dma.h>
#endif

#define CAPTURE_DRV_NAME        "adi_capture"

struct adi_cap_format {
	char *desc;
	u32 pixelformat;
	u32 mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
};

struct adi_cap_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct adi_cap_device {
	/* capture device instance */
	struct v4l2_device v4l2_dev;
	/* v4l2 control handler */
	struct v4l2_ctrl_handler ctrl_handler;
	/* device node data */
	struct video_device video_dev;
	/* sub device instance */
	struct v4l2_subdev *sd;
	/* capture config */
	struct adi_capture_config *cfg;
	/* ppi interface */
	struct ppi_if *ppi;
	/* current input */
	unsigned int cur_input;
	/* current selected standard */
	v4l2_std_id std;
	/* current selected dv_timings */
	struct v4l2_dv_timings dv_timings;
	/* used to store pixel format */
	struct v4l2_pix_format fmt;
	/* bits per pixel*/
	int bpp;
	/* data length for ppi in bits */
	int dlen;
	/* used to store sensor supported format */
	struct adi_cap_format *sensor_formats;
	/* number of sensor formats array */
	int num_sensor_formats;
	/* pointing to current video buffer */
	struct adi_cap_buffer *cur_frm;
	/* buffer queue used in videobuf2 */
	struct vb2_queue buffer_queue;
	/* queue of filled frames */
	struct list_head dma_queue;
	/* used in videobuf2 callback */
	spinlock_t lock;
	/* used to access capture device */
	struct mutex mutex;
	/* used to wait ppi to complete one transfer */
	struct completion comp;
	/* prepare to stop */
	bool stop;
	/* vb2 buffer sequence counter */
	unsigned sequence;
};

static const struct adi_cap_format adi_cap_formats[] = {
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = MEDIA_BUS_FMT_UYVY8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved YUYV",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code   = MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = MEDIA_BUS_FMT_UYVY8_1X16,
		.bpp         = 16,
		.dlen        = 16,
	},
	{
		.desc        = "RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.mbus_code   = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "RGB 444",
		.pixelformat = V4L2_PIX_FMT_RGB444,
		.mbus_code   = MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE,
		.bpp         = 16,
		.dlen        = 8,
	},

};
#define adi_cap_MAX_FMTS ARRAY_SIZE(adi_cap_formats)

static irqreturn_t adi_cap_isr(int irq, void *dev_id);

static struct adi_cap_buffer *to_adi_cap_vb(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct adi_cap_buffer, vb);
}

static int adi_cap_init_sensor_formats(struct adi_cap_device *adi_cap_dev)
{
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct adi_cap_format *sf;
	unsigned int num_formats = 0;
	int i, j;
	int num = 0;

	while (!v4l2_subdev_call(adi_cap_dev->sd, pad,
				enum_mbus_code, NULL, &code)) {
		num_formats++;
		code.index++;
	}
	if (!num_formats)
		return -ENXIO;

	sf = kcalloc(num_formats, sizeof(*sf), GFP_KERNEL);
	if (!sf)
		return -ENOMEM;

	for (i = 0; i < num_formats; i++) {
		code.index = i;
		v4l2_subdev_call(adi_cap_dev->sd, pad,
				enum_mbus_code, NULL, &code);
		for (j = 0; j < adi_cap_MAX_FMTS; j++)
			if (code.code == adi_cap_formats[j].mbus_code)
				break;
		if (j == adi_cap_MAX_FMTS)
			continue;
		sf[num++] = adi_cap_formats[j];
	}
	if (num == 0) {
		/* we don't allow this sensor working with our bridge */
		kfree(sf);
		return -EINVAL;
	}
	adi_cap_dev->sensor_formats = sf;
	adi_cap_dev->num_sensor_formats = num;
	return 0;
}

static void adi_cap_free_sensor_formats(struct adi_cap_device *adi_cap_dev)
{
	adi_cap_dev->num_sensor_formats = 0;
	kfree(adi_cap_dev->sensor_formats);
	adi_cap_dev->sensor_formats = NULL;
}

static int adi_cap_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], struct device *alloc_devs[])
{
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vq);

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2;

	if (*nplanes)
		return sizes[0] < adi_cap_dev->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = adi_cap_dev->fmt.sizeimage;

	return 0;
}

static int adi_cap_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = adi_cap_dev->fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&adi_cap_dev->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);

	vbuf->field = adi_cap_dev->fmt.field;

	return 0;
}

static void adi_cap_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct adi_cap_buffer *buf = to_adi_cap_vb(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&adi_cap_dev->lock, flags);
	list_add_tail(&buf->list, &adi_cap_dev->dma_queue);
	spin_unlock_irqrestore(&adi_cap_dev->lock, flags);
}

static void adi_cap_buffer_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct adi_cap_buffer *buf = to_adi_cap_vb(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&adi_cap_dev->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&adi_cap_dev->lock, flags);
}

static int adi_cap_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = adi_cap_dev->ppi;
	struct adi_cap_buffer *buf, *tmp;
	struct ppi_params params;
	dma_addr_t addr;
	int ret;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&adi_cap_dev->v4l2_dev, "stream on failed in subdev\n");
		goto err;
	}

	/* set ppi params */
	params.width = adi_cap_dev->fmt.width;
	params.height = adi_cap_dev->fmt.height;
	params.bpp = adi_cap_dev->bpp;
	params.dlen = adi_cap_dev->dlen;
	params.ppi_control = adi_cap_dev->cfg->ppi_control;
	params.int_mask = adi_cap_dev->cfg->int_mask;
	if (adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input].capabilities
			& V4L2_IN_CAP_DV_TIMINGS) {
		struct v4l2_bt_timings *bt = &adi_cap_dev->dv_timings.bt;

		params.hdelay = bt->hsync + bt->hbackporch;
		params.vdelay = bt->vsync + bt->vbackporch;
		params.line = V4L2_DV_BT_FRAME_WIDTH(bt);
		params.frame = V4L2_DV_BT_FRAME_HEIGHT(bt);
	} else if (adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input].capabilities
			& V4L2_IN_CAP_STD) {
		params.hdelay = 0;
		params.vdelay = 0;
		if (adi_cap_dev->std & V4L2_STD_525_60) {
			params.line = 858;
			params.frame = 525;
		} else {
			params.line = 864;
			params.frame = 625;
		}
	} else {
		params.hdelay = 0;
		params.vdelay = 0;
		params.line = params.width + adi_cap_dev->cfg->blank_pixels;
		params.frame = params.height;
	}
	ret = ppi->ops->set_params(ppi, &params);
	if (ret < 0) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Error in setting ppi params\n");
		goto err;
	}

	/* attach ppi DMA irq handler */
	ret = ppi->ops->attach_irq(ppi, adi_cap_isr);
	if (ret < 0) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Error in attaching interrupt handler\n");
		goto err;
	}

	adi_cap_dev->sequence = 0;

	reinit_completion(&adi_cap_dev->comp);
	adi_cap_dev->stop = false;

	/* get the next frame from the dma queue */
	adi_cap_dev->cur_frm = list_entry(adi_cap_dev->dma_queue.next,
					struct adi_cap_buffer, list);
	/* remove buffer from the dma queue */
	list_del_init(&adi_cap_dev->cur_frm->list);
	addr = vb2_dma_contig_plane_dma_addr(&adi_cap_dev->cur_frm->vb.vb2_buf,
						0);
	/* update DMA address */
	ppi->ops->update_addr(ppi, (unsigned long)addr);
	/* enable ppi */
	ppi->ops->start(ppi);

	return 0;

err:
	list_for_each_entry_safe(buf, tmp, &adi_cap_dev->dma_queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}

	return ret;
}

static void adi_cap_stop_streaming(struct vb2_queue *vq)
{
	struct adi_cap_device *adi_cap_dev = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = adi_cap_dev->ppi;
	int ret;

	adi_cap_dev->stop = true;
	wait_for_completion(&adi_cap_dev->comp);
	ppi->ops->stop(ppi);
	ppi->ops->detach_irq(ppi);
	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"stream off failed in subdev\n");

	/* release all active buffers */
	if (adi_cap_dev->cur_frm)
		vb2_buffer_done(&adi_cap_dev->cur_frm->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);

	while (!list_empty(&adi_cap_dev->dma_queue)) {
		adi_cap_dev->cur_frm = list_entry(adi_cap_dev->dma_queue.next,
						struct adi_cap_buffer, list);
		list_del_init(&adi_cap_dev->cur_frm->list);
		vb2_buffer_done(&adi_cap_dev->cur_frm->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops adi_cap_video_qops = {
	.queue_setup            = adi_cap_queue_setup,
	.buf_prepare            = adi_cap_buffer_prepare,
	.buf_cleanup            = adi_cap_buffer_cleanup,
	.buf_queue              = adi_cap_buffer_queue,
	.wait_prepare           = vb2_ops_wait_prepare,
	.wait_finish            = vb2_ops_wait_finish,
	.start_streaming        = adi_cap_start_streaming,
	.stop_streaming         = adi_cap_stop_streaming,
};

static irqreturn_t adi_cap_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct adi_cap_device *adi_cap_dev = ppi->priv;
	struct vb2_v4l2_buffer *vbuf = &adi_cap_dev->cur_frm->vb;
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	dma_addr_t addr;

	spin_lock(&adi_cap_dev->lock);

	if (!list_empty(&adi_cap_dev->dma_queue)) {
		vb->timestamp = ktime_get_ns();
		if (ppi->err) {
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
			ppi->err = false;
		} else {
			vbuf->sequence = adi_cap_dev->sequence++;
			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		}
		adi_cap_dev->cur_frm = list_entry(adi_cap_dev->dma_queue.next,
				struct adi_cap_buffer, list);
		list_del_init(&adi_cap_dev->cur_frm->list);
	} else {
		/* clear error flag, we will get a new frame */
		if (ppi->err)
			ppi->err = false;
	}

	ppi->ops->stop(ppi);

	if (adi_cap_dev->stop) {
		complete(&adi_cap_dev->comp);
	} else {
		addr = vb2_dma_contig_plane_dma_addr(
				&adi_cap_dev->cur_frm->vb.vb2_buf, 0);
		ppi->ops->update_addr(ppi, (unsigned long)addr);
		ppi->ops->start(ppi);
	}

	spin_unlock(&adi_cap_dev->lock);

	return IRQ_HANDLED;
}

static int adi_cap_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_STD))
		return -ENODATA;

	return v4l2_subdev_call(adi_cap_dev->sd, video, querystd, std);
}

static int adi_cap_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_STD))
		return -ENODATA;

	*std = adi_cap_dev->std;
	return 0;
}

static int adi_cap_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;
	int ret;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_STD))
		return -ENODATA;

	if (vb2_is_busy(&adi_cap_dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_std, std);
	if (ret < 0)
		return ret;

	adi_cap_dev->std = std;
	return 0;
}

static int adi_cap_enum_dv_timings(struct file *file, void *priv,
				struct v4l2_enum_dv_timings *timings)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_DV_TIMINGS))
		return -ENODATA;

	timings->pad = 0;

	return v4l2_subdev_call(adi_cap_dev->sd, pad,
			enum_dv_timings, timings);
}

static int adi_cap_query_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_DV_TIMINGS))
		return -ENODATA;

	return v4l2_subdev_call(adi_cap_dev->sd, video,
				query_dv_timings, timings);
}

static int adi_cap_g_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_DV_TIMINGS))
		return -ENODATA;

	*timings = adi_cap_dev->dv_timings;
	return 0;
}

static int adi_cap_s_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_input input;
	int ret;

	input = adi_cap_dev->cfg->inputs[adi_cap_dev->cur_input];
	if (!(input.capabilities & V4L2_IN_CAP_DV_TIMINGS))
		return -ENODATA;

	if (vb2_is_busy(&adi_cap_dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_dv_timings, timings);
	if (ret < 0)
		return ret;

	adi_cap_dev->dv_timings = *timings;
	return 0;
}

static int adi_cap_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct adi_capture_config *config = adi_cap_dev->cfg;
	int ret;
	u32 status;

	if (input->index >= config->num_inputs)
		return -EINVAL;

	*input = config->inputs[input->index];
	/* get input status */
	ret = v4l2_subdev_call(adi_cap_dev->sd, video, g_input_status, &status);
	if (!ret)
		input->status = status;
	return 0;
}

static int adi_cap_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);

	*index = adi_cap_dev->cur_input;
	return 0;
}

static int adi_cap_s_input(struct file *file, void *priv, unsigned int index)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct adi_capture_config *config = adi_cap_dev->cfg;
	struct adi_cap_route *route;
	int ret;

	if (vb2_is_busy(&adi_cap_dev->buffer_queue))
		return -EBUSY;

	if (index >= config->num_inputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_routing,
				route->input, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&adi_cap_dev->v4l2_dev, "Failed to set input\n");
		return ret;
	}
	adi_cap_dev->cur_input = index;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;
	return 0;
}

static int adi_cap_try_format(struct adi_cap_device *adi_cap,
				struct v4l2_pix_format *pixfmt,
				struct adi_cap_format *adi_cap_fmt)
{
	struct adi_cap_format *sf = adi_cap->sensor_formats;
	struct adi_cap_format *fmt = NULL;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret, i;

	for (i = 0; i < adi_cap->num_sensor_formats; i++) {
		fmt = &sf[i];
		if (pixfmt->pixelformat == fmt->pixelformat)
			break;
	}
	if (i == adi_cap->num_sensor_formats)
		fmt = &sf[0];

	v4l2_fill_mbus_format(&format.format, pixfmt, fmt->mbus_code);
	ret = v4l2_subdev_call(adi_cap->sd, pad, set_fmt, &pad_cfg,
				&format);
	if (ret < 0)
		return ret;
	v4l2_fill_pix_format(pixfmt, &format.format);
	if (adi_cap_fmt) {
		for (i = 0; i < adi_cap->num_sensor_formats; i++) {
			fmt = &sf[i];
			if (format.format.code == fmt->mbus_code)
				break;
		}
		*adi_cap_fmt = *fmt;
	}
	pixfmt->bytesperline = pixfmt->width * fmt->bpp / 8;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	return 0;
}

static int adi_cap_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct adi_cap_format *sf = adi_cap_dev->sensor_formats;

	if (fmt->index >= adi_cap_dev->num_sensor_formats)
		return -EINVAL;

	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strlcpy(fmt->description,
		sf[fmt->index].desc,
		sizeof(fmt->description));
	fmt->pixelformat = sf[fmt->index].pixelformat;
	return 0;
}

static int adi_cap_try_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return adi_cap_try_format(adi_cap_dev, pixfmt, NULL);
}

static int adi_cap_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);

	fmt->fmt.pix = adi_cap_dev->fmt;
	return 0;
}

static int adi_cap_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct adi_cap_format adi_cap_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret;

	if (vb2_is_busy(&adi_cap_dev->buffer_queue))
		return -EBUSY;

	/* see if format works */
	ret = adi_cap_try_format(adi_cap_dev, pixfmt, &adi_cap_fmt);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&format.format, pixfmt, adi_cap_fmt.mbus_code);
	ret = v4l2_subdev_call(adi_cap_dev->sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;
	adi_cap_dev->fmt = *pixfmt;
	adi_cap_dev->bpp = adi_cap_fmt.bpp;
	adi_cap_dev->dlen = adi_cap_fmt.dlen;
	return 0;
}

static int adi_cap_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "ADI SC5XX Platform", sizeof(cap->bus_info));
	strlcpy(cap->card, adi_cap_dev->cfg->card_name, sizeof(cap->card));
	return 0;
}

static int adi_cap_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(adi_cap_dev->sd, video, g_parm, a);
}

static int adi_cap_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(adi_cap_dev->sd, video, s_parm, a);
}

static int adi_cap_log_status(struct file *file, void *priv)
{
	struct adi_cap_device *adi_cap_dev = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&adi_cap_dev->v4l2_dev, 0, core, log_status);
	return 0;
}

static const struct v4l2_ioctl_ops adi_cap_ioctl_ops = {
	.vidioc_querycap         = adi_cap_querycap,
	.vidioc_g_fmt_vid_cap    = adi_cap_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = adi_cap_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap    = adi_cap_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap  = adi_cap_try_fmt_vid_cap,
	.vidioc_enum_input       = adi_cap_enum_input,
	.vidioc_g_input          = adi_cap_g_input,
	.vidioc_s_input          = adi_cap_s_input,
	.vidioc_querystd         = adi_cap_querystd,
	.vidioc_s_std            = adi_cap_s_std,
	.vidioc_g_std            = adi_cap_g_std,
	.vidioc_s_dv_timings     = adi_cap_s_dv_timings,
	.vidioc_g_dv_timings     = adi_cap_g_dv_timings,
	.vidioc_query_dv_timings = adi_cap_query_dv_timings,
	.vidioc_enum_dv_timings  = adi_cap_enum_dv_timings,
	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_create_bufs      = vb2_ioctl_create_bufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_expbuf           = vb2_ioctl_expbuf,
	.vidioc_streamon         = vb2_ioctl_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,
	.vidioc_g_parm           = adi_cap_g_parm,
	.vidioc_s_parm           = adi_cap_s_parm,
	.vidioc_log_status       = adi_cap_log_status,
};

static const struct v4l2_file_operations adi_cap_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
	.poll = vb2_fop_poll
};

#ifdef CONFIG_OF
static int get_int_prop(struct device_node *dn, const char *s)
{
	int ret;
	u32 val;

	ret = of_property_read_u32(dn, s, &val);
	if (ret)
		return 0;
	return val;
}

static const struct of_device_id adi_cap_match[] = {
	{ .compatible = "adi,cap", },
	{},
};
MODULE_DEVICE_TABLE(of, adi_cap_match);

static struct adi_capture_config *adi_get_config(struct platform_device *pdev)
{
	struct adi_capture_config *config;
	struct device_node *node = pdev->dev.of_node;
	struct ppi_info *info;
	const char *string;
	struct resource *res;

	config = (struct adi_capture_config *)pdev->dev.platform_data;
	if (!config) {
		dev_err(&pdev->dev, "failed to get platform data\n");
		return NULL;
	}

	/*Initialize ppi info struct*/
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "failed to alloc ppi info\n");
		return NULL;
	}
	info->type = (enum ppi_type)get_int_prop(node, "type");
	info->dma_ch = get_int_prop(node, "dma-channel");
	info->irq_err = platform_get_irq(pdev, 0);
	info->spu = get_int_prop(node, "spu_securep_id");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->base = devm_ioremap_resource(&pdev->dev, res);
	if (!(info->base)) {
		dev_err(&pdev->dev, "failed to get mem resource");
		return NULL;
	}

	config->i2c_adapter_id = get_int_prop(node, "i2c_bus_id");
	of_property_read_string(node, "card-name", &string);
	config->card_name = (char *)string;
	config->ppi_info = info;

	return config;
}
#endif

static int adi_cap_probe(struct platform_device *pdev)
{
	struct adi_cap_device *adi_cap_dev;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct adi_capture_config *config;
	struct vb2_queue *q;
	struct adi_cap_route *route;
	struct of_device_id *match;
	struct device *dev = &pdev->dev;
	int ret;

#ifdef CONFIG_OF
	match = of_match_device(adi_cap_match, &pdev->dev);
	if (!match) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	if (dev->of_node)
		config = adi_get_config(pdev);
#else
	config = pdev->dev.platform_data;
	if (!config || !config->num_inputs) {
		v4l2_err(pdev->dev.driver, "Unable to get board config\n");
		return -ENODEV;
	}
#endif

	adi_cap_dev = kzalloc(sizeof(*adi_cap_dev), GFP_KERNEL);
	if (!adi_cap_dev)
		return -ENOMEM;

	adi_cap_dev->cfg = config;

	adi_cap_dev->ppi = ppi_create_instance(pdev, config->ppi_info);
	if (!adi_cap_dev->ppi) {
		v4l2_err(pdev->dev.driver, "Unable to create ppi\n");
		ret = -ENODEV;
		goto err_free_dev;
	}
	adi_cap_dev->ppi->priv = adi_cap_dev;

	vfd = &adi_cap_dev->video_dev;
	/* initialize field of video device */
	vfd->release            = video_device_release_empty;
	vfd->fops               = &adi_cap_fops;
	vfd->ioctl_ops          = &adi_cap_ioctl_ops;
	vfd->tvnorms            = 0;
	vfd->v4l2_dev           = &adi_cap_dev->v4l2_dev;
	strncpy(vfd->name, CAPTURE_DRV_NAME, sizeof(vfd->name));

	ret = v4l2_device_register(&pdev->dev, &adi_cap_dev->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err_free_ppi;
	}
	v4l2_info(&adi_cap_dev->v4l2_dev, "v4l2 device registered\n");

	adi_cap_dev->v4l2_dev.ctrl_handler = &adi_cap_dev->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&adi_cap_dev->ctrl_handler, 0);
	if (ret) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Unable to init control handler\n");
		goto err_unreg_v4l2;
	}

	spin_lock_init(&adi_cap_dev->lock);
	/* initialize queue */
	q = &adi_cap_dev->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = adi_cap_dev;
	q->buf_struct_size = sizeof(struct adi_cap_buffer);
	q->ops = &adi_cap_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &adi_cap_dev->mutex;
	q->min_buffers_needed = 1;
	q->dev = &pdev->dev;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_handler;

	mutex_init(&adi_cap_dev->mutex);
	init_completion(&adi_cap_dev->comp);

	/* init video dma queues */
	INIT_LIST_HEAD(&adi_cap_dev->dma_queue);

	vfd->lock = &adi_cap_dev->mutex;
	vfd->queue = q;

	/* register video device */
	ret = video_register_device(&adi_cap_dev->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Unable to register video device\n");
		goto err_free_handler;
	}
	video_set_drvdata(&adi_cap_dev->video_dev, adi_cap_dev);
	v4l2_info(&adi_cap_dev->v4l2_dev, "video device registered as: %s\n",
			video_device_node_name(vfd));

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(config->i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Unable to find i2c adapter\n");
		ret = -ENODEV;
		goto err_unreg_vdev;

	}
	adi_cap_dev->sd = v4l2_i2c_new_subdev_board(&adi_cap_dev->v4l2_dev,
						 i2c_adap,
						 &config->board_info,
						 NULL);
	if (adi_cap_dev->sd) {
		int i;

		/* update tvnorms from the sub devices */
		for (i = 0; i < config->num_inputs; i++)
			vfd->tvnorms |= config->inputs[i].std;
	} else {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Unable to register sub device\n");
		ret = -ENODEV;
		goto err_unreg_vdev;
	}

	v4l2_info(&adi_cap_dev->v4l2_dev, "v4l2 sub device registered\n");

	/*
	 * explicitly set input, otherwise some boards
	 * may not work at the state as we expected
	 */
	route = &config->routes[0];
	ret = v4l2_subdev_call(adi_cap_dev->sd, video, s_routing,
				route->input, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&adi_cap_dev->v4l2_dev, "Failed to set input\n");
		goto err_unreg_vdev;
	}
	adi_cap_dev->cur_input = 0;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;

	/* now we can probe the default state */
	if (config->inputs[0].capabilities & V4L2_IN_CAP_STD) {
		v4l2_std_id std;
		ret = v4l2_subdev_call(adi_cap_dev->sd, video, g_std, &std);
		if (ret) {
			v4l2_err(&adi_cap_dev->v4l2_dev,
					"Unable to get std\n");
			goto err_unreg_vdev;
		}
		adi_cap_dev->std = std;
	}
	if (config->inputs[0].capabilities & V4L2_IN_CAP_DV_TIMINGS) {
		struct v4l2_dv_timings dv_timings;
		ret = v4l2_subdev_call(adi_cap_dev->sd, video,
				g_dv_timings, &dv_timings);
		if (ret) {
			v4l2_err(&adi_cap_dev->v4l2_dev,
					"Unable to get dv timings\n");
			goto err_unreg_vdev;
		}
		adi_cap_dev->dv_timings = dv_timings;
	}
	ret = adi_cap_init_sensor_formats(adi_cap_dev);
	if (ret) {
		v4l2_err(&adi_cap_dev->v4l2_dev,
				"Unable to create sensor formats table\n");
		goto err_unreg_vdev;
	}
	return 0;
err_unreg_vdev:
	video_unregister_device(&adi_cap_dev->video_dev);
err_free_handler:
	v4l2_ctrl_handler_free(&adi_cap_dev->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&adi_cap_dev->v4l2_dev);
err_free_ppi:
	ppi_delete_instance(adi_cap_dev->ppi);
err_free_dev:
	kfree(adi_cap_dev);
	return ret;
}

static int adi_cap_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct adi_cap_device *adi_cap_dev = container_of(v4l2_dev,
						struct adi_cap_device, v4l2_dev);

	adi_cap_free_sensor_formats(adi_cap_dev);
	video_unregister_device(&adi_cap_dev->video_dev);
	v4l2_ctrl_handler_free(&adi_cap_dev->ctrl_handler);
	v4l2_device_unregister(v4l2_dev);
	ppi_delete_instance(adi_cap_dev->ppi);
	kfree(adi_cap_dev);
	return 0;
}

static struct platform_driver adi_cap_driver = {
	.driver = {
		.name  = CAPTURE_DRV_NAME,
#ifdef CONFIG_OF
		.of_match_table = adi_cap_match,
#endif
	},
	.probe = adi_cap_probe,
	.remove = adi_cap_remove,
};
module_platform_driver(adi_cap_driver);

MODULE_DESCRIPTION("Analog Devices SC5XX video capture driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
