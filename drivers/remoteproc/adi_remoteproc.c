// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Device SHARC Image Loader for SC5XX processors
 *
 * Copyright (C) 2020, 2021 Analog Devices
 */

#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/elf.h>
#include <linux/virtio_ids.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/soc/adi/hardware.h>
#include <linux/soc/adi/sec.h>
#include <linux/soc/adi/dma.h>
#include <linux/soc/adi/icc.h>
#include <linux/soc/adi/rcu.h>
#include "remoteproc_internal.h"

/* The VERIFY_LDR_DATA macro is used to verify LDR data loaded to
 * the target memory.
 *
 * Disable this macro by default to speed up the whole process
 * To enable it by adding below line:
 * #define VERIFY_LDR_DATA
 */

#define VERIFY_LDR_DATA

/* location of bootrom that loops idle */
#define SHARC_IDLE_ADDR			(0x00090004)

#define SPU_MDMA0_SRC_ID		88
#define SPU_MDMA0_DST_ID		89

#define CORE_INIT_TIMEOUT msecs_to_jiffies(2000)

#if defined(VERIFY_LDR_DATA)
#define MEMORY_COUNT 2
#else
#define MEMORY_COUNT 1
#endif

#define ADI_FW_LDR 0
#define ADI_FW_ELF 1

#define NUM_TABLE_ENTRIES         1
/* Resource table for the given remote */
struct adi_sharc_resource_table {
	struct resource_table table_hdr;
	unsigned int offset[NUM_TABLE_ENTRIES];
	struct fw_rsc_hdr rsc_hdr;
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring vring[2];
} __packed;

#define VRING_ALIGN 0x1000
#define VRING_SIZE 0x8000

static struct adi_sharc_resource_table resources_sharc0 = {
	.table_hdr = {
		/* resource table header */
		1, 								 /* version */
		NUM_TABLE_ENTRIES, /* number of table entries */
		{0, 0,},					 /* reserved fields */
	},
	.offset = {offsetof(struct adi_sharc_resource_table, rsc_hdr),},
	/* virtio device entry */
	.rsc_hdr = {RSC_VDEV,}, /* virtio dev type */
	.rpmsg_vdev = {
		VIRTIO_ID_RPMSG, /* it's rpmsg virtio */
		1, /* kick sharc0 */
		/* 1<<0 is VIRTIO_RPMSG_F_NS bit defined in virtio_rpmsg_bus.c */
		1<<0, 0, 0, 0, /* dfeatures, gfeatures, config len, status */
		2, /* num_of_vrings */
		{0, 0,}, /* reserved */
	},
	.vring = {
		{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_SIZE, 1, 0}, /* da allocated by remoteproc driver */
		{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_SIZE, 1, 0}, /* da allocated by remoteproc driver */
	},
};

static struct adi_sharc_resource_table resources_sharc1 = {
	.table_hdr = {
		/* resource table header */
		1, 								 /* version */
		NUM_TABLE_ENTRIES, /* number of table entries */
		{0, 0,},					 /* reserved fields */
	},
	.offset = {offsetof(struct adi_sharc_resource_table, rsc_hdr),},
	/* virtio device entry */
	.rsc_hdr = {RSC_VDEV,}, /* virtio dev type */
	.rpmsg_vdev = {
		VIRTIO_ID_RPMSG, /* it's rpmsg virtio */
		2, /* kick sharc0 */
		/* 1<<0 is VIRTIO_RPMSG_F_NS bit defined in virtio_rpmsg_bus.c */
		1<<0, 0, 0, 0, /* dfeatures, gfeatures, config len, status */
		2, /* num_of_vrings */
		{0, 0,}, /* reserved */
	},
	.vring = {
		{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_SIZE, 2, 0}, /* da allocated by remoteproc driver */
		{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_SIZE, 2, 0}, /* da allocated by remoteproc driver */
	},
};

struct adi_rproc_data {
	struct device *dev;
	struct rproc *rproc;
	struct adi_rcu *rcu;
	const char *firmware_name;
	int core_id;
	int core_irq;
	void *mem_virt;
	dma_addr_t mem_handle;
	size_t fw_size;
	unsigned long ldr_load_addr;
	int firmware_format;
	void __iomem *L1_shared_base;
	void __iomem *L2_shared_base;
	struct completion sharc_platform_init_complete;
	struct workqueue_struct *core_workqueue;
	int wait_platform_init;
	struct delayed_work core_kick_work;
	u64 l1_da_range[2];
	u64 l2_da_range[2];
	u32 rsc_offset;
};

typedef struct block_code_flag {
	uint32_t bCode:4,			/* 0-3 */
			 bFlag_save:1,		/* 4 */
			 bFlag_aux:1,		/* 5 */
			 bReserved:1,		/* 6 */
			 bFlag_forward:1,	/* 7 */
			 bFlag_fill:1,		/* 8 */
			 bFlag_quickboot:1, /* 9 */
			 bFlag_callback:1,	/* 10 */
			 bFlag_init:1,		/* 11 */
			 bFlag_ignore:1,	/* 12 */
			 bFlag_indirect:1,	/* 13 */
			 bFlag_first:1,		/* 14 */
			 bFlag_final:1,		/* 15 */
			 bHdrCHK:8,			/* 16-23 */
			 bHdrSIGN:8;		/* 0xAD, 0xAC or 0xAB */
} BCODE_FLAG_t;

typedef struct ldr_hdr {
	BCODE_FLAG_t bcode_flag;
	uint32_t target_addr;
	uint32_t byte_count;
	uint32_t argument;
} LDR_Ehdr_t;

static int adi_core_set_svect(struct adi_rproc_data *rproc_data,
					unsigned long svect)
{
	int coreid = rproc_data->core_id;

	if (svect && (coreid == 1))
		adi_rcu_writel(svect, rproc_data->rcu, ADI_RCU_REG_SVECT1);
	else if (svect && (coreid == 2))
		adi_rcu_writel(svect, rproc_data->rcu, ADI_RCU_REG_SVECT2);
	else {
		dev_err(rproc_data->dev, "%s, invalid svect:0x%lx, cord_id:%d\n",
						__func__, svect, coreid);
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t sharc_virtio_irq_threaded_handler(int irq, void *p);

static int adi_core_start(struct adi_rproc_data *rproc_data)
{
	rproc_data->wait_platform_init = 1;
	return adi_rcu_start_core(rproc_data->rcu, rproc_data->core_id);
}

static int adi_core_reset(struct adi_rproc_data *rproc_data)
{
	return adi_rcu_reset_core(rproc_data->rcu, rproc_data->core_id);
}

static int adi_core_stop(struct adi_rproc_data *rproc_data)
{
	cancel_delayed_work_sync(&rproc_data->core_kick_work);
	return adi_rcu_stop_core(rproc_data->rcu,
		rproc_data->core_id, rproc_data->core_irq);
}

/* @todo this needs to return status */
static void ldr_load(struct adi_rproc_data *rproc_data)
{
	LDR_Ehdr_t* block_hdr = NULL;
	uint8_t* virbuf = (uint8_t*) rproc_data->mem_virt;
	dma_addr_t phybuf = rproc_data->mem_handle;
	int offset;
#if defined(VERIFY_LDR_DATA)
	int i;
	uint32_t verfied = 0;
	uint8_t* pCompareBuffer;
	uint8_t* pVerifyBuffer;
#endif

	do {
		/* read the header */
		block_hdr = (LDR_Ehdr_t*) virbuf;

		/* Overwrite the ldr_load_addr */
		if (block_hdr->bcode_flag.bFlag_first)
			rproc_data->ldr_load_addr = (unsigned long)block_hdr->target_addr;

		if (!block_hdr->bcode_flag.bFlag_ignore && block_hdr->byte_count) {
			if (block_hdr->bcode_flag.bFlag_fill) {
				dma_addr_t fillphy;
				void *fill = dma_alloc_coherent(rproc_data->dev,
					block_hdr->byte_count, &fillphy, GFP_KERNEL);

				if (!fill)
					return;

				memset(fill, block_hdr->argument, block_hdr->byte_count);
				dma_memcpy(block_hdr->target_addr, fillphy, block_hdr->byte_count);
				dma_free_coherent(rproc_data->dev, block_hdr->byte_count,
					fill, fillphy);
			}
			else {
				dma_memcpy(block_hdr->target_addr, phybuf + sizeof(LDR_Ehdr_t), block_hdr->byte_count);

#if defined(VERIFY_LDR_DATA)
				pCompareBuffer = virbuf + sizeof(LDR_Ehdr_t);
				pVerifyBuffer = virbuf + rproc_data->fw_size;

				dma_memcpy(phybuf + rproc_data->fw_size, block_hdr->target_addr,
							block_hdr->byte_count);

				/* check the data */
				for (i = 0; i < block_hdr->byte_count; i++) {
					if (pCompareBuffer[i] != pVerifyBuffer[i]) {
						dev_err(rproc_data->dev, "dirty data, pCompareBuffer[%d]:0x%x,\
							pVerifyBuffer[%d]:0x%x\n",
							i, pCompareBuffer[i], i, pVerifyBuffer[i]);
						verfied++;
						break;
					}
				}
#endif
			}
		}

		if (block_hdr->bcode_flag.bFlag_final)
			break;

		offset = sizeof(LDR_Ehdr_t) + (block_hdr->bcode_flag.bFlag_fill ?
							0 : block_hdr->byte_count);
		virbuf += offset;
		phybuf += offset;
	} while (1);

#if defined(VERIFY_LDR_DATA)
	if (verfied == 0)
		dev_err(rproc_data->dev, "success to verify all the data\n");
	else
		dev_err(rproc_data->dev, "fail to verify all the data %d\n", verfied);
#endif

}

static int adi_valid_firmware(struct rproc *rproc, const struct firmware *fw)
{
	LDR_Ehdr_t *adi_ldr_hdr = (LDR_Ehdr_t *)fw->data;

	if (!adi_ldr_hdr->byte_count
				&& (adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAD
					|| adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAC
					|| adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAB))
		return ADI_FW_LDR;

	if(!rproc_elf_sanity_check(rproc, fw)){
		dev_err(&rproc->dev, "ELF format not supported\n");
		return -ENOTSUPP;
	}

	dev_err(&rproc->dev, "## No valid image at address 0x%08x\n", (unsigned int)fw->data);
	return -EINVAL;
}

void set_spu_securep_msec(uint16_t n, bool msec);
static void enable_spu(void)
{
	set_spu_securep_msec(SPU_MDMA0_SRC_ID, true);
	set_spu_securep_msec(SPU_MDMA0_DST_ID, true);
}

static void disable_spu(void)
{
	set_spu_securep_msec(SPU_MDMA0_SRC_ID, false);
	set_spu_securep_msec(SPU_MDMA0_DST_ID, false);
}

static int adi_ldr_load(struct adi_rproc_data *rproc_data,
						const struct firmware *fw)
{
	rproc_data->fw_size = fw->size;
	if (!rproc_data->mem_virt) {
		rproc_data->mem_virt = dma_alloc_coherent(rproc_data->dev,
							  fw->size * MEMORY_COUNT,
							  &rproc_data->mem_handle,
							  GFP_KERNEL);
		if (rproc_data->mem_virt == NULL) {
			dev_err(rproc_data->dev, "Unable to allocate memory\n");
			return -ENOMEM;
		}
	}

	memcpy((char*)rproc_data->mem_virt, fw->data, fw->size);

	enable_spu();
	ldr_load(rproc_data);
	disable_spu();

	return 0;
}

/*
 * adi_rproc_load: parse and load ADI SHARC LDR file into memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_load(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	switch (rproc_data->firmware_format){
		case ADI_FW_LDR:
			ret = adi_ldr_load(rproc_data, fw);
			break;
		case ADI_FW_ELF:
			ret = rproc_elf_load_segments(rproc, fw);
			break;
		default:
			BUG();
			break;
	}

	if (ret) {
		dev_err(rproc_data->dev, "Failed to load ldr, ret:%d\n", ret);
	}

	return ret;
}

/*
 * adi_rproc_start: to start run the applicaiton which is loaded in memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_start(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, rproc_data->ldr_load_addr);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;

	return adi_core_start(rproc_data);
}

/*
 * adi_rproc_stop: to stop the running applicaiton in DSP
 * This would be called when user run the stop command
 * echo stop > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_stop(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, SHARC_IDLE_ADDR);
	if (ret)
		return ret;

	ret = adi_core_stop(rproc_data);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;


	if (rproc_data->mem_virt) {
		memset(rproc_data->mem_virt, 0, rproc_data->fw_size * MEMORY_COUNT);
		dma_free_coherent(rproc_data->dev, rproc_data->fw_size * MEMORY_COUNT,
						rproc_data->mem_virt, rproc_data->mem_handle);
		rproc_data->mem_virt = NULL;
		rproc_data->fw_size = 0;
	}

	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;
	return ret;
}

static int adi_ldr_load_rsc_table(struct rproc *rproc, const struct firmware *fw){
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	struct resource_table *table = NULL;
	size_t size;

	if(rproc_data->core_id == 1){
		table = (struct resource_table *)(&resources_sharc0);
		size = sizeof(resources_sharc0);
	}else{
		table = (struct resource_table *)(&resources_sharc1);
		size = sizeof(resources_sharc1);
	}

	rproc->cached_table = kmemdup(table, size, GFP_KERNEL);
	if (!rproc->cached_table){
		return -ENOMEM;
	}

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = size;

	return 0;
}

static int adi_rproc_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	switch (rproc_data->firmware_format){
		case ADI_FW_LDR:
			ret = adi_ldr_load_rsc_table(rproc, fw);
			break;
		case ADI_FW_ELF:
			ret = rproc_elf_load_rsc_table(rproc, fw);
			break;
		default:
			BUG();
			break;
	}
	return ret;
}

static struct resource_table *adi_ldr_find_loaded_rsc_table(struct rproc *rproc, const struct firmware *fw){
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	u64 da;

	/* resource table stored in L2 memory */
	da = rproc_data->l2_da_range[0] + rproc_data->rsc_offset;

	/* core 2 resource table is located right after core 1 resource table */
	if (rproc_data->core_id == 2)
		da += sizeof(resources_sharc0);

	return rproc_da_to_va(rproc, da, sizeof(resources_sharc0));
}

static struct resource_table *adi_rproc_find_loaded_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	struct resource_table * ret = NULL;

	switch (rproc_data->firmware_format){
		case ADI_FW_LDR:
			ret = adi_ldr_find_loaded_rsc_table(rproc, fw);
			break;
		case ADI_FW_ELF:
			ret = rproc_elf_find_loaded_rsc_table(rproc, fw);
			break;
		default:
			BUG();
			break;
	}
	return ret;
}

static irqreturn_t sharc_virtio_irq_threaded_handler(int irq, void *p){
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)p;
	struct adi_sharc_resource_table *table = (struct adi_sharc_resource_table *)adi_ldr_find_loaded_rsc_table(rproc_data->rproc, NULL);

	/* Process incoming buffers on all our vrings */
	if(rproc_data->wait_platform_init){
		complete(&rproc_data->sharc_platform_init_complete);
	}

	rproc_vq_interrupt(rproc_data->rproc, table->vring[0].notifyid);
	rproc_vq_interrupt(rproc_data->rproc, table->vring[1].notifyid);

	return IRQ_HANDLED;
}

/* kick a virtqueue */
static void adi_rproc_kick(struct rproc *rproc, int vqid)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	while(rproc_data->wait_platform_init){
		ret = wait_for_completion_interruptible_timeout(&rproc_data->sharc_platform_init_complete, CORE_INIT_TIMEOUT);
		if(ret > 0){
			rproc_data->wait_platform_init = 0;
		}else if(ret < 0){
			if (ret != -ERESTARTSYS){
				dev_err(rproc_data->dev, "Core%d init error %d\n", rproc_data->core_id, ret);
			}
		}else{
			dev_warn(rproc_data->dev, "Core%d init timeout\n", rproc_data->core_id);
			// Delay the kick until core is initialized
			queue_delayed_work(rproc_data->core_workqueue, &rproc_data->core_kick_work, CORE_INIT_TIMEOUT);
			return;
		}
	}

	platform_send_ipi_cpu(rproc_data->core_id, 0);
}

static void adi_rproc_kick_work(struct work_struct *work){
	struct delayed_work *dw = to_delayed_work(work);
	struct adi_rproc_data *rproc_data = container_of(dw, struct adi_rproc_data, core_kick_work);

	adi_rproc_kick(rproc_data->rproc, 0);
}

static int adi_rproc_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;

	/* Check if it is a LDR or ELF file */
	rproc_data->firmware_format = adi_valid_firmware(rproc, fw);

	if (rproc_data->firmware_format < 0)
		return rproc_data->firmware_format;
	else
		return 0;
}

static u32 adi_rproc_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	u32 ret;

	switch (rproc_data->firmware_format){
		case ADI_FW_LDR:
			ret = 0;
			break;
		case ADI_FW_ELF:
			ret = rproc_elf_get_boot_addr(rproc, fw);
			break;
		default:
			BUG();
			break;
	}
	return ret;
}

static void *adi_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	void __iomem *L1_shared_base = rproc_data->L1_shared_base;
	void __iomem *L2_shared_base = rproc_data->L2_shared_base;
	void *ret = NULL;

	if (len == 0)
		return NULL;

	if (da >= rproc_data->l1_da_range[0] && da < rproc_data->l1_da_range[1])
		ret = L1_shared_base + da;
	else if (da >= rproc_data->l2_da_range[0] && da < rproc_data->l2_da_range[1])
		ret = L2_shared_base + (da - rproc_data->l2_da_range[0]);

	return ret;
}

static const struct rproc_ops adi_rproc_ops = {
	.start = adi_rproc_start,
	.stop = adi_rproc_stop,
	.kick		= adi_rproc_kick,
	.load = adi_rproc_load,
	.da_to_va = adi_da_to_va,
	.parse_fw = adi_rproc_load_rsc_table,
	.find_loaded_rsc_table = adi_rproc_find_loaded_rsc_table,
	.sanity_check = adi_rproc_sanity_check,
	.get_boot_addr = adi_rproc_get_boot_addr,
};

static int adi_remoteproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_rproc_data *rproc_data;
	struct device_node *np = dev->of_node;
	struct adi_rcu *adi_rcu;
	struct rproc *rproc;
	struct resource *res;
	u32 addr[2];
	int irq, irq_flags;
	int ret;
	const char *name;

	ret = of_property_read_string(np, "firmware-name", &name);
	if (ret) {
		dev_err(dev, "Unable to get firmware-name property\n");
		return ret;
	}

	adi_rcu = get_adi_rcu_from_node(dev);
	if (IS_ERR(adi_rcu))
		return PTR_ERR(adi_rcu);

	rproc = rproc_alloc(dev, np->name, &adi_rproc_ops,
					name, sizeof(*rproc_data));
	if (!rproc) {
		dev_err(dev, "Unable to allocate remoteproc\n");
		ret = -ENOMEM;
		goto free_adi_rcu;
	}

	rproc_data = (struct adi_rproc_data *)rproc->priv;
	platform_set_drvdata(pdev, rproc);

	/* for now device addresses are represented as 32 bits and expanded to 64
	 * here in driver code */
	if (of_property_read_u32_array(np, "adi,l1-da", addr, 2)) {
		dev_err(dev, "Missing adi,l1-da with L1 device address range information\n");
		ret = -ENODEV;
		goto free_rproc;
	}
	rproc_data->l1_da_range[0] = addr[0];
	rproc_data->l1_da_range[1] = addr[1];

	if (of_property_read_u32_array(np, "adi,l2-da", addr, 2)) {
		dev_err(dev, "Missing adi,l2-da with L2 device address range information\n");
		ret = -ENODEV;
		goto free_rproc;
	}
	rproc_data->l2_da_range[0] = addr[0];
	rproc_data->l2_da_range[1] = addr[1];

	if (of_property_read_u32(np, "adi,resource-table-offset",
		&rproc_data->rsc_offset))
	{
		dev_err(dev, "Missing adi,resource-table-offset in device tree\n");
		ret = -ENODEV;
		goto free_rproc;
	}

	rproc_data->core_workqueue = alloc_workqueue("Core workqueue",
		WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!rproc_data->core_workqueue) {
		dev_err(dev, "Unable to allocate core workqueue\n");
		goto free_rproc;
	}

	init_completion(&rproc_data->sharc_platform_init_complete);
	INIT_DELAYED_WORK(&rproc_data->core_kick_work, adi_rproc_kick_work);

	ret = of_property_read_u32(np, "core-id", &rproc_data->core_id);
	if (ret) {
		dev_err(dev, "Unable to get core-id property\n");
		goto free_workqueue;
	}

	ret = of_property_read_u32(np, "core-irq", &rproc_data->core_irq);
	if (ret) {
		dev_err(dev, "Unable to get core-irq property\n");
		goto free_workqueue;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "No ICC IRQ specified\n");
		ret = -ENOENT;
		goto free_workqueue;
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	irq_flags = (res->flags & IORESOURCE_BITS) | IRQF_PERCPU | IRQF_SHARED;

	ret = devm_request_threaded_irq(dev, irq, NULL,
		sharc_virtio_irq_threaded_handler, irq_flags | IRQF_ONESHOT,
		"ICC virtio IRQ", rproc_data);
	if (ret) {
		dev_err(rproc_data->dev, "Fail to request ICC receive IRQ\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get L1 base address (reg 0)\n");
		ret = -ENODEV;
		goto free_workqueue;
	}
	rproc_data->L1_shared_base = devm_ioremap_wc(dev,
		res->start, resource_size(res));
	if (IS_ERR(rproc_data->L1_shared_base)) {
		dev_err(dev, "Cannot map L1 shared memory\n");
		ret = PTR_ERR(rproc_data->L1_shared_base);
		goto free_workqueue;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "Cannot get L2 base address (reg 1)\n");
		ret = -ENODEV;
		goto free_workqueue;
	}
	rproc_data->L2_shared_base = devm_ioremap_wc(dev,
		res->start, resource_size(res));
	if (IS_ERR(rproc_data->L2_shared_base)) {
		dev_err(dev, "Cannot map L2 shared memory\n");
		ret = PTR_ERR(rproc_data->L2_shared_base);
		goto free_workqueue;
	}

	rproc_data->dev = &pdev->dev;
	rproc_data->rcu = adi_rcu;
	rproc_data->rproc = rproc;
	rproc_data->firmware_name = name;
	rproc_data->mem_virt = NULL;
	rproc_data->fw_size = 0;
	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to add rproc\n");
		goto free_workqueue;
	}

	return 0;

free_workqueue:
	destroy_workqueue(rproc_data->core_workqueue);

free_rproc:
	rproc_free(rproc);

free_adi_rcu:
	put_adi_rcu(adi_rcu);

	return ret;
}

static int adi_remoteproc_remove(struct platform_device *pdev)
{
	struct adi_rproc_data *rproc_data = platform_get_drvdata(pdev);

	put_adi_rcu(rproc_data->rcu);
	rproc_del(rproc_data->rproc);
	rproc_free(rproc_data->rproc);

	return 0;
}

static const struct of_device_id adi_rproc_of_match[] = {
	{ .compatible = "adi,remoteproc" },
	{ },
};
MODULE_DEVICE_TABLE(of, adi_rproc_of_match);

static struct platform_driver adi_rproc_driver = {
	.probe = adi_remoteproc_probe,
	.remove = adi_remoteproc_remove,
	.driver = {
		.name = "adi_remoteproc",
		.of_match_table = of_match_ptr(adi_rproc_of_match),
	},
};
module_platform_driver(adi_rproc_driver);

MODULE_DESCRIPTION("Analog Device sc5xx SHARC Image Loader");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Chen <jian.chen@analog.com>");
