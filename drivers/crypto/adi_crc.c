/*
 * Cryptographic API.
 *
 * Support ADI SC5XX CRC HW acceleration.
 *
 * Copyright 2012 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <asm/unaligned.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/portmux.h>
#include <mach/dma.h>
#else
#include <asm/dma.h>
#include <asm/portmux.h>
#endif

#include <asm/io.h>

#include "adi_crc.h"

#define CRC_CCRYPTO_QUEUE_LENGTH	5

#define DRIVER_NAME "adi-hmac-crc"
#define CHKSUM_DIGEST_SIZE      4
#define CHKSUM_BLOCK_SIZE       1
#define BUFLEN					256

#define CRC_MAX_DMA_DESC	100

struct adi_crypto_crc {
	struct list_head	list;
	struct device		*dev;
	spinlock_t			lock;

	int			irq;
	int			dma_ch;
	u32			poly;
	struct crc_register *regs;

	struct ahash_request	*req; /* current request in operation */
	struct dma_desc_array	*sg_cpu; /* virt addr of sg dma descriptors */
	dma_addr_t		sg_dma; /* phy addr of sg dma descriptors */
	u8				*sg_mid_buf;
	dma_addr_t		sg_mid_dma; /* phy addr of sg mid buffer */

	struct tasklet_struct	done_task;
	struct crypto_queue		queue; /* waiting requests */

	u8			busy:1;			/* crc device in operation flag */
};

static struct adi_crypto_crc_list {
	struct list_head	dev_list;
	spinlock_t		lock;
} crc_list;

struct adi_crypto_crc_reqctx {
	struct adi_crypto_crc	*crc;

	unsigned int		total;		/* total request bytes */
	size_t				sg_buflen;	/* bytes for this update */
	unsigned int		sg_nents;
	struct scatterlist  sg_list[2]; /* chained sg list */

	size_t		bufnext_len;
	size_t		buflast_len;
	u8			bufnext[CHKSUM_DIGEST_SIZE]; /* extra bytes for next udpate */
	u8			buflast[CHKSUM_DIGEST_SIZE]; /* extra bytes from last udpate */

	u8			xmit_buf[BUFLEN] __attribute__((aligned(sizeof(u32))));
};

struct adi_crypto_crc_ctx {
	struct adi_crypto_crc	*crc;
	u32			key;
};


/*
 * derive number of elements in scatterlist
 */
static int sg_count(struct scatterlist *sg_list)
{
	struct scatterlist *sg = sg_list;
	int sg_nents = 1;

	if (sg_list == NULL)
		return 0;

	while (!sg_is_last(sg)) {
		sg_nents++;
		sg = sg_next(sg);
	}

	return sg_nents;
}

/*
 * get element in scatter list by given index
 */
static struct scatterlist *sg_get(struct scatterlist *sg_list, unsigned int nents,
				unsigned int index)
{
	struct scatterlist *sg = NULL;
	int i;

	for_each_sg(sg_list, sg, nents, i)
		if (i == index)
			break;

	return sg;
}

static int adi_crypto_crc_init_hw(struct adi_crypto_crc *crc, u32 key)
{
	writel(0, &crc->regs->datacntrld);
	writel(MODE_CALC_CRC << OPMODE_OFFSET, &crc->regs->control);
	writel(key, &crc->regs->curresult);

	/* setup CRC interrupts */
	writel(CMPERRI | DCNTEXPI, &crc->regs->status);
	writel(CMPERRI | DCNTEXPI, &crc->regs->intrenset);

	return 0;
}

static int adi_crypto_crc_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc;

	dev_dbg(ctx->crc->dev, "crc_init\n");
	spin_lock_bh(&crc_list.lock);
	list_for_each_entry(crc, &crc_list.dev_list, list) {
		crc_ctx->crc = crc;
		break;
	}
	spin_unlock_bh(&crc_list.lock);

	if (sg_nents(req->src) > CRC_MAX_DMA_DESC) {
		dev_dbg(ctx->crc->dev, "init: requested sg list is too big > %d\n",
			CRC_MAX_DMA_DESC);
		return -EINVAL;
	}

	ctx->crc = crc;
	ctx->bufnext_len = 0;
	ctx->buflast_len = 0;
	ctx->sg_nents = 0;
	ctx->sg_buflen = 0;
	ctx->total = 0;
	memset(ctx->sg_list, 0, 2 * sizeof(ctx->sg_list[0]));

	dev_dbg(ctx->crc->dev, "init: digest size: %d\n",
		crypto_ahash_digestsize(tfm));

	return adi_crypto_crc_init_hw(crc, crc_ctx->key);
}

static int adi_crypto_crc_export(struct ahash_request *req, void *out)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int adi_crypto_crc_import(struct ahash_request *req, const void *in)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static void adi_crypto_crc_config_dma(struct adi_crypto_crc *crc)
{
	struct scatterlist *sg;
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(crc->req);
	int i = 0, j = 0;
	unsigned long dma_config;
	unsigned int dma_count;
	unsigned int dma_addr;
	unsigned int mid_dma_count = 0;
	int dma_mod;

	dma_map_sg(crc->dev, ctx->sg_list, ctx->sg_nents, DMA_TO_DEVICE);

	for_each_sg(ctx->sg_list, sg, ctx->sg_nents, j) {
		dma_addr = sg_dma_address(sg);
		/* deduce extra bytes in last sg */
		if (sg_is_last(sg))
			dma_count = sg_dma_len(sg) - ctx->bufnext_len;
		else
			dma_count = sg_dma_len(sg);

		if (mid_dma_count) {
			/* Append last middle dma buffer to 4 bytes with first
			   bytes in current sg buffer. Move addr of current
			   sg and deduce the length of current sg.
			 */
			memcpy(crc->sg_mid_buf +(i << 2) + mid_dma_count,
				sg_virt(sg),
				CHKSUM_DIGEST_SIZE - mid_dma_count);
			dma_addr += CHKSUM_DIGEST_SIZE - mid_dma_count;
			dma_count -= CHKSUM_DIGEST_SIZE - mid_dma_count;

			dma_config = DMAFLOW_ARRAY | RESTART | NDSIZE_3 |
				DMAEN | PSIZE_32 | WDSIZE_32;

			/* setup new dma descriptor for next middle dma */
			crc->sg_cpu[i].start_addr = crc->sg_mid_dma + (i << 2);
			crc->sg_cpu[i].cfg = dma_config;
			crc->sg_cpu[i].x_count = 1;
			crc->sg_cpu[i].x_modify = CHKSUM_DIGEST_SIZE;
			dev_dbg(crc->dev, "%d: crc_dma: start_addr:0x%lx, "
				"cfg:0x%lx, x_count:0x%lx, x_modify:0x%lx\n",
				i, crc->sg_cpu[i].start_addr,
				crc->sg_cpu[i].cfg, crc->sg_cpu[i].x_count,
				crc->sg_cpu[i].x_modify);
			i++;
		}

		dma_config = DMAFLOW_ARRAY | RESTART | NDSIZE_3 | DMAEN | PSIZE_32;
		/* chop current sg dma len to multiple of 32 bits */
		mid_dma_count = dma_count % 4;
		dma_count &= ~0x3;

		if (dma_addr % 4 == 0) {
			dma_config |= WDSIZE_32;
			dma_count >>= 2;
			dma_mod = 4;
		} else if (dma_addr % 2 == 0) {
			dma_config |= WDSIZE_16;
			dma_count >>= 1;
			dma_mod = 2;
		} else {
			dma_config |= WDSIZE_8;
			dma_mod = 1;
		}

		crc->sg_cpu[i].start_addr = dma_addr;
		crc->sg_cpu[i].cfg = dma_config;
		crc->sg_cpu[i].x_count = dma_count;
		crc->sg_cpu[i].x_modify = dma_mod;
		dev_dbg(crc->dev, "%d: crc_dma: start_addr:0x%lx, "
			"cfg:0x%lx, x_count:0x%lx, x_modify:0x%lx\n",
			i, crc->sg_cpu[i].start_addr,
			crc->sg_cpu[i].cfg, crc->sg_cpu[i].x_count,
			crc->sg_cpu[i].x_modify);
		i++;

		if (mid_dma_count) {
			/* copy extra bytes to next middle dma buffer */
			memcpy(crc->sg_mid_buf + (i << 2),
				(u8*)sg_virt(sg) + (dma_count << 2),
				mid_dma_count);
		}
	}

	dma_config = DMAFLOW_ARRAY | RESTART | NDSIZE_3 | DMAEN | PSIZE_32 | WDSIZE_32;
	/* For final update req, append the buffer for next update as well*/
	if (ctx->bufnext_len) {
		crc->sg_cpu[i].start_addr = dma_map_single(crc->dev, ctx->bufnext,
						CHKSUM_DIGEST_SIZE, DMA_TO_DEVICE);
		crc->sg_cpu[i].cfg = dma_config;
		crc->sg_cpu[i].x_count = 1;
		crc->sg_cpu[i].x_modify = CHKSUM_DIGEST_SIZE;
		dev_dbg(crc->dev, "%d: crc_dma: start_addr:0x%lx, "
			"cfg:0x%lx, x_count:0x%lx, x_modify:0x%lx\n",
			i, crc->sg_cpu[i].start_addr,
			crc->sg_cpu[i].cfg, crc->sg_cpu[i].x_count,
			crc->sg_cpu[i].x_modify);
		i++;
	}

	if (i == 0)
		return;

	/* Set the last descriptor to stop mode */
	crc->sg_cpu[i - 1].cfg &= ~(DMAFLOW | NDSIZE);
	crc->sg_cpu[i - 1].cfg |= DI_EN;
	set_dma_curr_desc_addr(crc->dma_ch, (unsigned long *)crc->sg_dma);
	set_dma_x_count(crc->dma_ch, 0);
	set_dma_x_modify(crc->dma_ch, 0);
	set_dma_config(crc->dma_ch, dma_config);
}

static int adi_crypto_crc_handle_queue(struct adi_crypto_crc *crc,
				  struct ahash_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct adi_crypto_crc_reqctx *ctx;
	struct scatterlist *sg;
	int ret = 0;
	int nsg, i, j;
	unsigned int nextlen;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&crc->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&crc->queue, req);
	if (crc->busy) {
		spin_unlock_irqrestore(&crc->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&crc->queue);
	async_req = crypto_dequeue_request(&crc->queue);
	if (async_req)
		crc->busy = 1;
	spin_unlock_irqrestore(&crc->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	crc->req = req;
	ctx = ahash_request_ctx(req);

	dev_dbg(crc->dev, "handling new req, total %d bytes\n", ctx->total);
	if (ctx->total == 0) {
		crc->busy = 0;
		return 0;
	}

	/* Pack last crc update buffer to 32bit */
	memset(ctx->bufnext + ctx->bufnext_len, 0,
			CHKSUM_DIGEST_SIZE - ctx->bufnext_len);

	/* Chop crc buffer size to multiple of 32 bit */
	nsg = ctx->sg_nents = sg_count(ctx->sg_list);
	ctx->sg_buflen = ctx->buflast_len + ctx->total;
	ctx->bufnext_len = ctx->sg_buflen % 4;
	ctx->sg_buflen &= ~0x3;

	if (ctx->bufnext_len) {
		/* copy extra bytes to buffer for next update */
		memset(ctx->bufnext, 0, CHKSUM_DIGEST_SIZE);
		nextlen = ctx->bufnext_len;
		for (i = nsg - 1; i >= 0; i--) {
			sg = sg_get(ctx->sg_list, nsg, i);
			j = min(nextlen, sg_dma_len(sg));
			memcpy(ctx->bufnext + nextlen - j,
				sg_virt(sg) + sg_dma_len(sg) - j, j);
			if (j == sg_dma_len(sg))
				ctx->sg_nents--;
			nextlen -= j;
			if (nextlen == 0)
				break;
		}
		ctx->sg_buflen += CHKSUM_DIGEST_SIZE;
	}

	/* set CRC data count before start DMA */
	writel(ctx->sg_buflen >> 2, &crc->regs->datacnt);

	/* setup and enable CRC DMA */
	adi_crypto_crc_config_dma(crc);

	/* finally kick off CRC operation */
	reg = readl(&crc->regs->control);
	writel(reg | BLKEN, &crc->regs->control);

	return -EINPROGRESS;
}

static int adi_crypto_crc_update(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	int ret = 0;

	if (!req->nbytes)
		return 0;

	dev_dbg(ctx->crc->dev, "crc_update\n");
	if (ctx->total + req->nbytes <= BUFLEN) {
		scatterwalk_map_and_copy(ctx->xmit_buf + ctx->total, req->src,
					0, req->nbytes, 0);
		ctx->total += req->nbytes;
	} else
		ret = -ENOBUFS;

	return ret;
}

static int adi_crypto_crc_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	dev_dbg(ctx->crc->dev, "crc_final\n");
	if (!ctx->total)
		put_unaligned_le32(crc_ctx->key, req->result);
	else
		sg_init_one(ctx->sg_list, ctx->xmit_buf, ctx->total);

	crc_ctx->key = 0;

	return adi_crypto_crc_handle_queue(ctx->crc, req);
}

static int adi_crypto_crc_finup(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	int ret = 0;

	dev_dbg(ctx->crc->dev, "crc_finishupdate\n");

	ret = adi_crypto_crc_update(req);
	if (ret == -ENOBUFS)
		return ret;

	return adi_crypto_crc_final(req);
}

static int adi_crypto_crc_digest(struct ahash_request *req)
{
	return adi_crypto_crc_init(req) ?: adi_crypto_crc_finup(req);
}

static int adi_crypto_crc_setkey(struct crypto_ahash *tfm, const u8 *key,
			unsigned int keylen)
{
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);

	dev_dbg(crc_ctx->crc->dev, "crc_setkey\n");
	if (keylen != CHKSUM_DIGEST_SIZE) {
		crypto_ahash_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	crc_ctx->key = get_unaligned_le32(key);

	return 0;
}

static int adi_crypto_crc_cra_init(struct crypto_tfm *tfm)
{
	struct adi_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);

	crc_ctx->key = 0;
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct adi_crypto_crc_reqctx));

	return 0;
}

static void adi_crypto_crc_cra_exit(struct crypto_tfm *tfm)
{
}

static struct ahash_alg algs = {
	.init		= adi_crypto_crc_init,
	.update		= adi_crypto_crc_update,
	.final		= adi_crypto_crc_final,
	.finup		= adi_crypto_crc_finup,
	.digest		= adi_crypto_crc_digest,
	.setkey		= adi_crypto_crc_setkey,
	.export		= adi_crypto_crc_export,
	.import		= adi_crypto_crc_import,
	.halg.digestsize	= CHKSUM_DIGEST_SIZE,
	.halg.statesize		= sizeof(struct adi_crypto_crc_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(crc32)",
		.cra_driver_name	= DRIVER_NAME,
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
						CRYPTO_ALG_ASYNC |
						CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize		= CHKSUM_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct adi_crypto_crc_ctx),
		.cra_alignmask		= 3,
		.cra_module		= THIS_MODULE,
		.cra_init		= adi_crypto_crc_cra_init,
		.cra_exit		= adi_crypto_crc_cra_exit,
	}
};

static void adi_crypto_crc_done_task(unsigned long data)
{
	struct adi_crypto_crc *crc = (struct adi_crypto_crc *)data;

	adi_crypto_crc_handle_queue(crc, NULL);
}

static irqreturn_t adi_crypto_crc_handler(int irq, void *dev_id)
{
	struct adi_crypto_crc *crc = dev_id;
	u32 reg;

	if (readl(&crc->regs->status) & DCNTEXP) {
		writel(DCNTEXP, &crc->regs->status);
		put_unaligned_le32(readl(&crc->regs->result),
				crc->req->result);

		reg = readl(&crc->regs->control);
		writel(reg & ~BLKEN, &crc->regs->control);
		crc->busy = 0;

		if (crc->req->base.complete)
			crc->req->base.complete(&crc->req->base, 0);

		tasklet_schedule(&crc->done_task);

		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

/**
 *	adi_crypto_crc_suspend - suspend crc device
 *	@pdev: device being suspended
 *	@state: requested suspend state
 */
static __maybe_unused int adi_crypto_crc_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct adi_crypto_crc *crc = platform_get_drvdata(pdev);
	int i = 100000;

	while ((readl(&crc->regs->control) & BLKEN) && --i)
		cpu_relax();

	if (i == 0)
		return -EBUSY;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adi_crypto_of_match[] = {
	{
		.compatible = "adi,hmac-crc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_crypto_of_match);
#endif

/**
 *	adi_crypto_crc_probe - Initialize module
 *
 */
static int adi_crypto_crc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct adi_crypto_crc *crc;
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	unsigned int timeout = 100000;
	int ret;

	crc = devm_kzalloc(dev, sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		dev_err(&pdev->dev, "fail to malloc adi_crypto_crc\n");
		return -ENOMEM;
	}

	crc->dev = dev;

	INIT_LIST_HEAD(&crc->list);
	spin_lock_init(&crc->lock);
	tasklet_init(&crc->done_task, adi_crypto_crc_done_task, (unsigned long)crc);
	crypto_init_queue(&crc->queue, CRC_CCRYPTO_QUEUE_LENGTH);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	crc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR((void *)crc->regs)) {
		dev_err(&pdev->dev, "Cannot map CRC IO\n");
		return PTR_ERR((void *)crc->regs);
	}

	crc->irq = platform_get_irq(pdev, 0);
	if (crc->irq < 0) {
		dev_err(&pdev->dev, "No CRC DCNTEXP IRQ specified\n");
		return -ENOENT;
	}

	ret = devm_request_irq(dev, crc->irq, adi_crypto_crc_handler,
			IRQF_SHARED, dev_name(dev), crc);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request ADI crc irq\n");
		return ret;
	}

	match = of_match_device(of_match_ptr(adi_crypto_of_match), &pdev->dev);
	if (match) {
		if (of_property_read_u32(node, "dma_channel", &crc->dma_ch))
			return -ENOENT;
		of_property_read_u32(node, "crypto_crc_poly", &crc->poly);
	} else {
		res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
		if (res == NULL) {
			dev_err(&pdev->dev, "No CRC DMA channel specified\n");
			return -ENOENT;
		}
		crc->dma_ch = res->start;
		crc->poly = (u32)pdev->dev.platform_data;
	}

	ret = request_dma(crc->dma_ch, dev_name(dev));
	if (ret) {
		dev_err(&pdev->dev, "Unable to attach ADI CRC DMA channel\n");
		return ret;
	}

	crc->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &crc->sg_dma, GFP_KERNEL);
	if (crc->sg_cpu == NULL) {
		ret = -ENOMEM;
		goto out_error_dma;
	}
	/*
	 * need at most CRC_MAX_DMA_DESC sg + CRC_MAX_DMA_DESC middle  +
	 * 1 last + 1 next dma descriptors
	 */
	crc->sg_mid_buf = (u8 *)(crc->sg_cpu + ((CRC_MAX_DMA_DESC + 1) << 1));
	crc->sg_mid_dma = crc->sg_dma + sizeof(struct dma_desc_array)
			* ((CRC_MAX_DMA_DESC + 1) << 1);

	writel(0, &crc->regs->control);
	writel(crc->poly, &crc->regs->poly);

	while (!(readl(&crc->regs->status) & LUTDONE) && (--timeout) > 0)
		cpu_relax();

	if (timeout == 0)
		dev_info(&pdev->dev, "init crc poly timeout\n");

	platform_set_drvdata(pdev, crc);

	spin_lock(&crc_list.lock);
	list_add(&crc->list, &crc_list.dev_list);
	spin_unlock(&crc_list.lock);

	if (list_is_singular(&crc_list.dev_list)) {
		ret = crypto_register_ahash(&algs);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't register crypto ahash device\n");
			goto out_error_dma;
		}
	}

	dev_info(&pdev->dev, "initialized\n");

	return 0;

out_error_dma:
	if (crc->sg_cpu)
		dma_free_coherent(&pdev->dev, PAGE_SIZE, crc->sg_cpu, crc->sg_dma);
	free_dma(crc->dma_ch);

	return ret;
}

/**
 *	adi_crypto_crc_remove - Initialize module
 *
 */
static int adi_crypto_crc_remove(struct platform_device *pdev)
{
	struct adi_crypto_crc *crc = platform_get_drvdata(pdev);

	if (!crc)
		return -ENODEV;

	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	spin_unlock(&crc_list.lock);

	crypto_unregister_ahash(&algs);
	tasklet_kill(&crc->done_task);
	free_dma(crc->dma_ch);

	return 0;
}

static struct platform_driver adi_crypto_crc_driver = {
	.probe     = adi_crypto_crc_probe,
	.remove    = adi_crypto_crc_remove,
	.suspend   = adi_crypto_crc_suspend,
	.driver    = {
		.name  = DRIVER_NAME,
		.of_match_table = of_match_ptr(adi_crypto_of_match),
	},
};

/**
 *	adi_crypto_crc_mod_init - Initialize module
 *
 *	Checks the module params and registers the platform driver.
 *	Real work is in the platform probe function.
 */
static int __init adi_crypto_crc_mod_init(void)
{
	int ret;

	pr_info("ADI hardware CRC crypto driver\n");

	INIT_LIST_HEAD(&crc_list.dev_list);
	spin_lock_init(&crc_list.lock);

	ret = platform_driver_register(&adi_crypto_crc_driver);
	if (ret) {
		pr_err("unable to register driver\n");
		return ret;
	}

	return 0;
}

/**
 *	adi_crypto_crc_mod_exit - Deinitialize module
 */
static void __exit adi_crypto_crc_mod_exit(void)
{
	platform_driver_unregister(&adi_crypto_crc_driver);
}

module_init(adi_crypto_crc_mod_init);
module_exit(adi_crypto_crc_mod_exit);

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("ADI SC5XX CRC hardware crypto driver");
MODULE_LICENSE("GPL");
