/*
 * drivers/crypto/adi/adi-pkte.c
 *
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2021 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#include <asm/cacheflush.h>

#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <crypto/engine.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/hash.h>

#include <linux/dma-mapping.h>

#include "adi-pkte.h"

static u32 Key[8] ={0x0,0x0,0x0,0x0,0,0,0,0};
static u32 IV[4] = {0x0,0x0,0x0,0x0};
static u32 IDigest[8] ={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
static u32 ODigest[8] ={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

static DECLARE_WAIT_QUEUE_HEAD(wq_processing);
static int processing = 0;

static DECLARE_WAIT_QUEUE_HEAD(wq_ready);
static bool ready = 0;

struct adi_ctx {
	struct crypto_engine_ctx enginectx;
	struct adi_dev	*hdev;
};

struct adi_request_ctx {
	struct adi_dev	*hdev;
	unsigned long		op;

	u8 digest[SHA256_DIGEST_SIZE] __aligned(sizeof(u32));
	size_t			digcnt;
	size_t			bufcnt;
	size_t			buflen;

	struct scatterlist	*sg;
	unsigned int		offset;
	unsigned int		total;

	int			nents;

	u32			*hw_context;
};

struct adi_algs_info {
	struct ahash_alg	*algs_list;
	size_t			size;
};

struct adi_pdata {
	struct adi_algs_info	*algs_info;
	size_t				algs_info_size;
};

struct adi_dev {
	struct list_head	list;
	struct device		*dev;
	struct clk		*clk;
	struct reset_control	*rst;
	void __iomem		*io_base;
	phys_addr_t		phys_base;

	struct ahash_request	*req;
	struct crypto_engine	*engine;

	int			err;
	unsigned long		flags;

	const struct adi_pdata	*pdata;

	ADI_PKTE_DEVICE * pkte_device;
	bool src_count_set;
	u32 src_bytes_available;
	u32 ring_pos_produce;
	u32 ring_pos_consume;
	dma_addr_t dma_handle;

	u8			secret_key[PKTE_MAX_KEY_SIZE];
	int			secret_keylen;
};

struct adi_drv {
	struct list_head	dev_list;
	spinlock_t		lock; /* List protection access */
};

static struct adi_drv adi = {
	.dev_list = LIST_HEAD_INIT(adi.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(adi.lock),
};

static inline u32 adi_read(struct adi_dev *hdev, u32 offset)
{
	return readl(hdev->io_base + offset);
}

static inline void adi_write(struct adi_dev *hdev,
					u32 offset, u32 value)
{
	writel(value, hdev->io_base + offset);
}

u32 adi_physical_address(struct adi_dev * hdev, u32 variableAddress){
	#ifdef PKTE_USE_SRAM
		return PKTE_SRAM_ADDRESS + variableAddress - ((u32)hdev->pkte_device);
	#else
		return hdev->dma_handle + variableAddress - ((u32)hdev->pkte_device);
	#endif
}

static void adi_reset_pkte(struct adi_dev * hdev){
	u32 temp;

	//Reset the packet engine
	temp = adi_read(hdev, CFG_OFFSET);
	adi_write(hdev, CFG_OFFSET, temp | 0x03);
	adi_write(hdev, CFG_OFFSET, temp &~ 0x03);
}

static void adi_init_state(struct adi_dev *hdev){
	adi_write(hdev, STATE_IV_OFFSET(0), hdev->pkte_device->pPkteDescriptor.State.STATE_IV0);
	adi_write(hdev, STATE_IV_OFFSET(1), hdev->pkte_device->pPkteDescriptor.State.STATE_IV1);
	adi_write(hdev, STATE_IV_OFFSET(2), hdev->pkte_device->pPkteDescriptor.State.STATE_IV2);
	adi_write(hdev, STATE_IV_OFFSET(3), hdev->pkte_device->pPkteDescriptor.State.STATE_IV3);
	adi_write(hdev, STATE_BYTE_CNT_OFFSET(0), hdev->pkte_device->pPkteDescriptor.State.STATE_BYTE_CNT0);
	adi_write(hdev, STATE_BYTE_CNT_OFFSET(1), hdev->pkte_device->pPkteDescriptor.State.STATE_BYTE_CNT1);
	adi_write(hdev, STATE_IDIGEST_OFFSET(0), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST0);
	adi_write(hdev, STATE_IDIGEST_OFFSET(1), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST1);
	adi_write(hdev, STATE_IDIGEST_OFFSET(2), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST2);
	adi_write(hdev, STATE_IDIGEST_OFFSET(3), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST3);
	adi_write(hdev, STATE_IDIGEST_OFFSET(4), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST4);
	adi_write(hdev, STATE_IDIGEST_OFFSET(5), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST5);
	adi_write(hdev, STATE_IDIGEST_OFFSET(6), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST6);
	adi_write(hdev, STATE_IDIGEST_OFFSET(7), hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST7);
}

static void adi_init_spe(struct adi_dev *hdev){
	u8 pos = hdev->flags & PKTE_AUTONOMOUS_MODE ? hdev->ring_pos_consume : 0;	
	adi_write(hdev, CTL_STAT_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_CTRL_STAT);
	adi_write(hdev, SRC_ADDR_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_SOURCE_ADDR);
	adi_write(hdev, DEST_ADDR_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR);
	adi_write(hdev, SA_ADDR_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_SA_ADDR);
	adi_write(hdev, STATE_ADDR_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_STATE_ADDR);
	adi_write(hdev, USERID_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_USER_ID);
}

static void adi_init_sa(struct adi_dev *hdev){
	adi_write(hdev, SA_CMD_OFFSET(0), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0);
	adi_write(hdev, SA_CMD_OFFSET(1), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD1);
	adi_write(hdev, SA_KEY_OFFSET(0), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY0);
	adi_write(hdev, SA_KEY_OFFSET(1), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY1);
	adi_write(hdev, SA_KEY_OFFSET(2), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY2);
	adi_write(hdev, SA_KEY_OFFSET(3), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY3);
	adi_write(hdev, SA_KEY_OFFSET(4), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY4);
	adi_write(hdev, SA_KEY_OFFSET(5), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY5);
	adi_write(hdev, SA_KEY_OFFSET(6), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY6);
	adi_write(hdev, SA_KEY_OFFSET(7), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY7);
	adi_write(hdev, SA_IDIGEST_OFFSET(0), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST0);
	adi_write(hdev, SA_IDIGEST_OFFSET(1), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST1);
	adi_write(hdev, SA_IDIGEST_OFFSET(2), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST2);
	adi_write(hdev, SA_IDIGEST_OFFSET(3), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST3);
	adi_write(hdev, SA_IDIGEST_OFFSET(4), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST4);
	adi_write(hdev, SA_IDIGEST_OFFSET(5), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST5);
	adi_write(hdev, SA_IDIGEST_OFFSET(6), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST6);
	adi_write(hdev, SA_IDIGEST_OFFSET(7), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST7);
	adi_write(hdev, SA_ODIGEST_OFFSET(0), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST0);
	adi_write(hdev, SA_ODIGEST_OFFSET(1), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST1);
	adi_write(hdev, SA_ODIGEST_OFFSET(2), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST2);
	adi_write(hdev, SA_ODIGEST_OFFSET(3), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST3);
	adi_write(hdev, SA_ODIGEST_OFFSET(4), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST4);
	adi_write(hdev, SA_ODIGEST_OFFSET(5), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST5);
	adi_write(hdev, SA_ODIGEST_OFFSET(6), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST6);
	adi_write(hdev, SA_ODIGEST_OFFSET(7), hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST7);
}

static void adi_init_ring(struct adi_dev *hdev){
	u32 temp;
	temp = adi_physical_address(hdev, (u32)&hdev->pkte_device->pPkteDescriptor.CmdDescriptor[0]);		
	adi_write(hdev, CDRBASE_ADDR_OFFSET, temp);
	temp = adi_physical_address(hdev, (u32)&hdev->pkte_device->pPkteDescriptor.ResultDescriptor[0]);
	adi_write(hdev, RDRBASE_ADDR_OFFSET, temp);
	temp = (PKTE_RING_BUFFERS-1)<<BITP_PKTE_RING_CFG_RINGSZ;
	adi_write(hdev, RING_CFG_OFFSET, temp);
	temp = 0<<BITP_PKTE_RING_THRESH_CDRTHRSH|0<<BITP_PKTE_RING_THRESH_RDRTHRSH;
	adi_write(hdev, RING_THRESH_OFFSET, temp);
}

static void adi_source_data(struct adi_dev *hdev, u32 size){
	u8 pos = hdev->flags & PKTE_AUTONOMOUS_MODE ? hdev->ring_pos_consume : 0;
	if( (!hdev->src_count_set) || (hdev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) ){
		hdev->src_count_set = 1;
		hdev->pkte_device->pPkteList.nSrcSize = size;
		hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_LENGTH = (u32)BITM_PKTE_LEN_HSTRDY|hdev->pkte_device->pPkteList.nSrcSize;
		if(! (hdev->flags & PKTE_AUTONOMOUS_MODE))
			adi_write(hdev, LEN_OFFSET, hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_LENGTH);
		dev_dbg(hdev->dev, "adi_source_data: LEN set to %x\n", hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_LENGTH);
	}
}

static void adi_configure_cdr(struct adi_dev *hdev){
	u8 pos = hdev->flags & PKTE_AUTONOMOUS_MODE ? hdev->ring_pos_consume : 0;
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_CTRL_STAT = (u32)0x1|(hdev->pkte_device->pPkteList.pCommand.final_hash_condition<<4);
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_SOURCE_ADDR = adi_physical_address(hdev, (u32)&hdev->pkte_device->source[hdev->ring_pos_consume][0]);
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR = adi_physical_address(hdev, (u32)&hdev->pkte_device->destination[0]);
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_SA_ADDR = adi_physical_address(hdev, (u32)&hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0);
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_STATE_ADDR = adi_physical_address(hdev, (u32)&hdev->pkte_device->pPkteDescriptor.State.STATE_IV0);
	hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_USER_ID = hdev->pkte_device->pPkteList.nUserID;
}

static void adi_config_sa_para(struct adi_dev *hdev){
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0 = 
		(hdev->pkte_device->pPkteList.pCommand.opcode<<BITP_PKTE_SA_CMD0_OPCD)|
		(hdev->pkte_device->pPkteList.pCommand.direction<<BITP_PKTE_SA_CMD0_DIR)|
		((u32)0x0 <<BITP_PKTE_SA_CMD0_OPGRP)|
		((u32)0x3 <<BITP_PKTE_SA_CMD0_PADTYPE)|
		(hdev->pkte_device->pPkteList.pCommand.cipher<<BITP_PKTE_SA_CMD0_CIPHER)|
		(hdev->pkte_device->pPkteList.pCommand.hash <<BITP_PKTE_SA_CMD0_HASH)|
		(u32)0x1<<BITP_PKTE_SA_CMD0_SCPAD|
		(hdev->pkte_device->pPkteList.pCommand.digest_length<<BITP_PKTE_SA_CMD0_DIGESTLEN)|
		((u32)0x2 <<BITP_PKTE_SA_CMD0_IVSRC)|
		(hdev->pkte_device->pPkteList.pCommand.hash_source <<BITP_PKTE_SA_CMD0_HASHSRC)|
		(u32)0x0<<BITP_PKTE_SA_CMD0_SVIV|
		(u32)0x1<<BITP_PKTE_SA_CMD0_SVHASH;

	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD1=
		(u32)0x1<<BITP_PKTE_SA_CMD1_CPYDGST|
		(hdev->pkte_device->pPkteList.pCommand.hash_mode<<BITP_PKTE_SA_CMD1_HMAC)|
		(hdev->pkte_device->pPkteList.pCommand.cipher_mode<<BITP_PKTE_SA_CMD1_CIPHERMD)|
		(hdev->pkte_device->pPkteList.pCommand.aes_key_length<<BITP_PKTE_SA_CMD1_AESKEYLEN)|
		(hdev->pkte_device->pPkteList.pCommand.aes_des_key<<BITP_PKTE_SA_CMD1_AESDECKEY);
}

static void adi_config_sa_key(struct adi_dev *hdev, u32 Key[]){
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY0 = Key[0];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY1 = Key[1];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY2 = Key[2];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY3 = Key[3];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY4 = Key[4];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY5 = Key[5];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY6 = Key[6];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Key.SA_KEY7 = Key[7];
}

static void adi_config_idigest(struct adi_dev *hdev, u32 Idigest[]){
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST0 = Idigest[0];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST1 = Idigest[1];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST2 = Idigest[2];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST3 = Idigest[3];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST4 = Idigest[4];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST5 = Idigest[5];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST6 = Idigest[6];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Idigest.SA_IDIGEST7 = Idigest[7];
}

static void adi_config_odigest(struct adi_dev *hdev, u32 Odigest[])
{
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST0 = Odigest[0];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST1 = Odigest[1];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST2 = Odigest[2];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST3 = Odigest[3];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST4 = Odigest[4];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST5 = Odigest[5];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST6 = Odigest[6];
	hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Odigest.SA_ODIGEST7 = Odigest[7];
}

static void adi_config_state(struct adi_dev *hdev, u32 IV[])
{
	hdev->pkte_device->pPkteDescriptor.State.STATE_IV0 = IV[0];
	hdev->pkte_device->pPkteDescriptor.State.STATE_IV1 = IV[1];
	hdev->pkte_device->pPkteDescriptor.State.STATE_IV2 = IV[2];
	hdev->pkte_device->pPkteDescriptor.State.STATE_IV3 = IV[3];
	hdev->pkte_device->pPkteDescriptor.State.STATE_BYTE_CNT0 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_BYTE_CNT1 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST0 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST1 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST2 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST3 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST4 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST5 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST6 = (u32)0x0;
	hdev->pkte_device->pPkteDescriptor.State.STATE_IDIGEST7 = (u32)0x0;
}

static void adi_reset_state(struct adi_dev *hdev){
	hdev->src_count_set = 0;
	hdev->src_bytes_available = 0;
	hdev->ring_pos_produce = 0;
	hdev->ring_pos_consume = 0;
	hdev->flags &= ~(PKTE_FLAGS_STARTED|
					 PKTE_FLAGS_COMPLETE|
					 PKTE_FLAGS_FINAL|
					 PKTE_FLAGS_FINUP|
					 PKTE_FLAGS_HMAC|
					 PKTE_FLAGS_HMAC_KEY_PREPARED);
}

static void adi_write_packet(struct adi_dev *hdev, u32 *source)
{
	u32 i, nLength, SIZE, nPadLength = 0, nTotalSize;
	u32 *temp;
	u32 temp1;
	uint8_t iBufIncrement = 0;

	temp = source;
	temp1 = (adi_read(hdev, BUF_PTR_OFFSET) & BITM_PKTE_BUF_PTR_INBUF) >> BITP_PKTE_BUF_PTR_INBUF;
	temp1 += DATAIO_BUF_OFFSET;

	nLength = hdev->src_bytes_available;
	SIZE = (nLength > 128)? 128: (nLength);

	if(SIZE % 4){
		for(i=0;i<1+SIZE/4; i++)
		{
			adi_write(hdev, temp1, *temp);
			temp1+=4;
			temp++;
		}
	}else{
		for(i=0;i<SIZE/4; i++)
		{
			adi_write(hdev, temp1, *temp);
			temp1+=4;
			temp++;
		}
	}

	dev_dbg(hdev->dev, "adi_write_packet: wrote %x bytes\n", SIZE);

	switch(hdev->pkte_device->pPkteList.pCommand.opcode){
		case opcode_hash:
			//Nothing to do
			break;
		default:
			//Pad to 16 byte alignment/boundaries
			if(SIZE % 16 !=0)
			{
				nTotalSize = ((SIZE/16)+1)*16;
				nPadLength = nTotalSize - SIZE;
				for(i=0;i< nPadLength; i++)
				{
					writeb(0x00, hdev->io_base + temp1);
					temp1++;
				}
			}
			break;
	}

	hdev->pkte_device->pPkteList.pSource = hdev->pkte_device->pPkteList.pSource + SIZE/(u32)4;

	iBufIncrement= (uint8_t)SIZE + (uint8_t)nPadLength;
	if(iBufIncrement % 4){
		iBufIncrement += (4-iBufIncrement%4);
	}

	if(iBufIncrement > hdev->src_bytes_available){
		hdev->src_bytes_available = 0;
	}else{
		hdev->src_bytes_available -= iBufIncrement;
	}

	adi_write(hdev, INBUF_INCR_OFFSET, (u32) iBufIncrement);
}

static void adi_read_packet(struct adi_dev *hdev, u32 *destination)
{
	u32 i=0;
	u32 nLength=0, SIZE=0;
	uint8_t iBufIncrement = 0;
	u32 *temp;
	u32 temp1 = 0, temp2 = 0;
	u8 pos;
	temp = destination;
	temp2 = adi_read(hdev, IMSK_STAT_OFFSET);
	pos = hdev->flags & PKTE_AUTONOMOUS_MODE ? hdev->ring_pos_consume : 0;

	if((temp2 & BITM_PKTE_IMSK_EN_OPDN) != BITM_PKTE_IMSK_EN_OPDN)
	{
		temp1 = DATAIO_BUF_OFFSET;
	
		nLength = (u32)(hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR + 
					hdev->pkte_device->pPkteList.nSrcSize - 
					adi_physical_address(hdev, (u32)hdev->pkte_device->pPkteList.pDestination));
		SIZE = (nLength>128)? 128:nLength;

		if(hdev->pkte_device->pPkteList.pCommand.opcode == opcode_hash)
		{
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_md5){
				SIZE = 4*4;  /* Destination will be 4 word hash value */
			}

			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha1){
				SIZE = 5*4;  /* Destination will be 5 word hash value */
			}

			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha224){
				SIZE = 7*4;  /* Destination will be 7 word hash value */
			}

			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha256){
				SIZE = 8*4;  /* Destination will be 8 word hash value */
			}
			if(SIZE % 4){
				dev_dbg(hdev->dev, "adi_read_packet: Unhandled Case - 4-byte alignment would fail\n");
			}else{
				for(i=0;i<SIZE/4; i++)
				{
					*temp = adi_read(hdev, temp1);
					dev_dbg(hdev->dev, "adi_read_packet read %x\n", *temp);
					temp1+=4;
					temp++;
				}
			}
		}
		else if(hdev->pkte_device->pPkteList.pCommand.opcode == opcode_encrypt_hash)
		{	
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_md5){
				SIZE = SIZE+ 4*4;  /* Destination will be 4 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha1){
				SIZE = SIZE+ 5*4;  /* Destination will be 5 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha224){
				SIZE = SIZE + 7*4;  /* Destination will be 7 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha256){
				SIZE = SIZE + 8*4;  /* Destination will be 8 word hash value */
			}


			for(i=0;i<SIZE; i++)
			{
				*temp = adi_read(hdev, temp1);
				temp1++;
				temp++;
			}
		}

		else{
			if(SIZE % 16 !=0){
				SIZE= ((SIZE/16)+1)*16;  /*For encryption 16 byte boundary is required */
			}
			for(i=0;i<SIZE; i++)
			{
				*temp = adi_read(hdev, temp1);
				temp1++;
				temp++;
			}
		}


		//hdev->pkte_device->pPkteList.pDestination = hdev->pkte_device->pPkteList.pDestination + SIZE/4;
		iBufIncrement = (uint8_t)SIZE;
		if(iBufIncrement % 4){
			iBufIncrement = ((iBufIncrement/4)+1)*4;
		}
		adi_write(hdev, OUTBUF_DECR_OFFSET, iBufIncrement);
	}
	else
	{
		nLength = (u32)(hdev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR + hdev->pkte_device->pPkteList.nSrcSize - adi_physical_address(hdev, (u32)hdev->pkte_device->pPkteList.pDestination));
		SIZE = nLength;

		if(hdev->pkte_device->pPkteList.pCommand.opcode == opcode_hash)
		{
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_md5){
				SIZE = 4*4;  /* Destination will be 4 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha1){
				SIZE = 5*4;  /* Destination will be 5 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha224){
				SIZE = 7*4;  /* Destination will be 7 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha256){
				SIZE = 8*4;  /* Destination will be 8 word hash value */
			}

			if(SIZE % 4){
				dev_dbg(hdev->dev, "adi_read_packet: Unhandled Case - 4-byte alignment would fail\n");
			}else{
				for(i=0;i<SIZE/4; i++)
				{
					*temp = adi_read(hdev, temp1);
					dev_dbg(hdev->dev, "adi_read_packet read %x\n", *temp);
					temp1+=4;
					temp++;
				}
			}
		}
		else if(hdev->pkte_device->pPkteList.pCommand.opcode == opcode_encrypt_hash)
		{
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_md5){
				SIZE = SIZE+ 4*4;  /* Destination will be 4 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha1){
				SIZE = SIZE+ 5*4;  /* Destination will be 5 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha224){
				SIZE = SIZE + 7*4;  /* Destination will be 7 word hash value */
			}
			if(hdev->pkte_device->pPkteList.pCommand.hash == hash_sha256){
				SIZE = SIZE + 8*4;  /* Destination will be 8 word hash value */
			}


			for(i=0;i<SIZE; i++)
			{
				*temp = adi_read(hdev, temp1);
				temp1++;
				temp++;
			}
		}
		else{
			if(SIZE % 16 !=0){
				SIZE= ((SIZE/16)+1)*16;  /*For encryption 16 byte boundary is required */
			}
			for(i=0;i<SIZE; i++)
			{
				*temp = adi_read(hdev, temp1);
				temp1++;
				temp++;
			}
		}
		hdev->pkte_device->pPkteList.pDestination = hdev->pkte_device->pPkteList.pDestination + SIZE/4;
	}
}

static void adi_append_sg(struct adi_request_ctx *rctx, struct adi_dev *hdev)
{
	size_t count;

	dev_dbg(rctx->hdev->dev, "adi_append_sg\n");

	while ((rctx->bufcnt < rctx->buflen) && rctx->total) {
		count = min(rctx->sg->length - rctx->offset, rctx->total);
		count = min(count, rctx->buflen - rctx->bufcnt);

		if (count <= 0) {
			if ((rctx->sg->length == 0) && !sg_is_last(rctx->sg)) {
				rctx->sg = sg_next(rctx->sg);
				continue;
			} else {
				break;
			}
		}

		scatterwalk_map_and_copy(((u8*)&hdev->pkte_device->source[hdev->ring_pos_produce][0]) + rctx->bufcnt, rctx->sg,
					 rctx->offset, count, 0);

		rctx->bufcnt += count;
		rctx->offset += count;
		rctx->total -= count;

		if(hdev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)){
			if(rctx->bufcnt >= PKTE_BUFLEN){
				hdev->ring_pos_produce++;
				if(hdev->ring_pos_produce >= PKTE_RING_BUFFERS){
					hdev->ring_pos_produce = 0;
				}
			}
		}

		if (rctx->offset == rctx->sg->length) {
			rctx->sg = sg_next(rctx->sg);
			if (rctx->sg)
				rctx->offset = 0;
			else
				rctx->total = 0;
		}
	}
}


static void adi_start_engine(struct adi_dev *hdev){
	u32 temp;

	dev_dbg(hdev->dev, "%s\n", __func__);

	//Set up configuration structures
	adi_configure_cdr(hdev);
	adi_config_sa_para(hdev);
	adi_config_sa_key(hdev, Key);
	adi_config_idigest(hdev, IDigest);
	adi_config_odigest(hdev, ODigest);
	adi_config_state(hdev, IV);

	if(hdev->flags & (PKTE_AUTONOMOUS_MODE | PKTE_TCM_MODE) ){
		for(temp = 1; temp < PKTE_RING_BUFFERS; temp++){
			memcpy(&hdev->pkte_device->pPkteDescriptor.SARecord[temp],
				&hdev->pkte_device->pPkteDescriptor.SARecord[0], sizeof(SA));
			memcpy(&hdev->pkte_device->pPkteDescriptor.CmdDescriptor[temp],
				&hdev->pkte_device->pPkteDescriptor.CmdDescriptor[0], sizeof(PE_CDR));
		}
	}

	if(hdev->flags & PKTE_HOST_MODE)
		adi_write(hdev, SA_RDY_OFFSET, 1);

	//Reset and release the packet engine
	adi_reset_pkte(hdev);

	temp = adi_read(hdev, CFG_OFFSET);
	if(hdev->flags & PKTE_HOST_MODE){
		dev_dbg(hdev->dev, "%s PKTE_HOST_MODE selected\n", __func__);
		temp |= BITM_PKTE_HOST_MODE<<8;
		adi_write(hdev, CLK_CTL_OFFSET, 1);
	}else if(hdev->flags & PKTE_TCM_MODE){
		dev_dbg(hdev->dev, "%s PKTE_TCM_MODE selected\n", __func__);
		temp |= BITM_PKTE_TCM_MODE<<8;
	}else if(hdev->flags & PKTE_AUTONOMOUS_MODE){
		dev_dbg(hdev->dev, "%s PKTE_AUTONOMOUS_MODE selected\n", __func__);
		adi_init_ring(hdev);
		temp |= (u32)1<<10;
		temp |= BITM_PKTE_AUTONOMOUS_MODE<<8;
	}
	adi_write(hdev, CFG_OFFSET, temp);

	adi_init_sa(hdev);
	adi_init_state(hdev);
	adi_init_spe(hdev);
}

static void adi_prep_engine(struct adi_dev *hdev, u32 hash_mode){

	hdev->src_count_set = 0;
	hdev->src_bytes_available = 0;
	hdev->ring_pos_produce = 0;
	hdev->ring_pos_consume = 0;
	ready = 0;
	processing = 0;

	hdev->pkte_device->pPkteList.pCommand.opcode = opcode_hash;
	hdev->pkte_device->pPkteList.pCommand.direction = dir_outbound;
	hdev->pkte_device->pPkteList.pCommand.cipher = cipher_null;
	hdev->pkte_device->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;
	hdev->pkte_device->pPkteList.pCommand.hash_mode = hash_mode;
	hdev->pkte_device->pPkteList.pCommand.aes_key_length = aes_key_length_other;
	hdev->pkte_device->pPkteList.pCommand.aes_des_key = aes_key;
	hdev->pkte_device->pPkteList.pCommand.hash_source = hash_source_no_load;

	hdev->flags &= ~PKTE_FLAGS_STARTED;

	adi_start_engine(hdev);
}

static int adi_process_packet(struct adi_dev *hdev,
				   size_t length, int final)
{
	unsigned int len32;
	u32 temp;

	len32 = DIV_ROUND_UP(length, sizeof(u32));

	dev_dbg(hdev->dev, "%s: length: %zd, final: %x len32 %i\n",
		__func__, length, final, len32);

	//Set final hash condition
	if(final){
		hdev->flags |= PKTE_FLAGS_FINAL;
		dev_dbg(hdev->dev, "%s final hash condition set\n", __func__);	
		if(! (hdev->flags & PKTE_AUTONOMOUS_MODE)){		
			hdev->pkte_device->pPkteList.pCommand.final_hash_condition = final_hash;
			temp = (u32)0x1|(final_hash<<BITP_PKTE_CTL_STAT_HASHFINAL);
			adi_write(hdev, CTL_STAT_OFFSET, temp);
		}else{
			hdev->pkte_device->pPkteList.pCommand.final_hash_condition = final_hash;
		}
	}

	if(final && (length == 0)){
		ready = 1;
		wake_up_interruptible(&wq_ready);
		return 0;
	}

	if(hdev->flags & (PKTE_TCM_MODE | PKTE_HOST_MODE)){
		wait_event_interruptible(wq_processing, processing == 0);
		processing = 1;
	}
	while( ! (adi_read(hdev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN) );
	while( ! (adi_read(hdev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY) );

	hdev->pkte_device->pPkteList.pSource = &hdev->pkte_device->source[hdev->ring_pos_consume][0];

	if(hdev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)){
		adi_configure_cdr(hdev);
	}

	hdev->pkte_device->pPkteList.pDestination = &hdev->pkte_device->destination[0];

	adi_source_data(hdev, length);

	hdev->src_bytes_available = length;

	if(hdev->flags & PKTE_TCM_MODE){
		adi_config_sa_para(hdev);
	}

	//Continue with previous operation
	if(hdev->flags & PKTE_FLAGS_STARTED){
		dev_dbg(hdev->dev, "%s hash_source_state set\n", __func__);	

		if(hdev->flags & PKTE_AUTONOMOUS_MODE){
			temp = hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0;
			temp &= ~BITM_PKTE_SA_CMD0_HASHSRC;
			temp |= hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
			hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0 = temp;
		}else{
			//Set HASHSRC to hash_source_state
			temp = adi_read(hdev, SA_CMD_OFFSET(0));
			temp &= ~BITM_PKTE_SA_CMD0_HASHSRC;
			temp |= hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
			hdev->pkte_device->pPkteDescriptor.SARecord[hdev->ring_pos_consume].SA_Para.SA_CMD0 = temp;
			adi_write(hdev, SA_CMD_OFFSET(0), temp);
		}
	}else{
		hdev->flags |= PKTE_FLAGS_STARTED;
	}


#ifndef PKTE_USE_SRAM
	//The dma_alloc_coherent region does not appear to be uncached on the SC594
	//Let's manually flush the cache before we trigger the DMA operation
	if (current->mm) {
		int i;
		for (i = 0; i < VMACACHE_SIZE; i++) {
			if (!current->vmacache.vmas[i])
				continue;
			flush_cache_range(current->vmacache.vmas[i],
					   (long)hdev->pkte_device, 
					  ((long)hdev->pkte_device) + sizeof(ADI_PKTE_DEVICE));
		}
	}
#endif

	if(hdev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)){
		hdev->ring_pos_consume++;
		if(adi_read(hdev, RDSC_CNT_OFFSET)){
			adi_write(hdev, RDSC_DECR_OFFSET, 0x1);
		}
		if(hdev->ring_pos_consume >= PKTE_RING_BUFFERS){
			hdev->ring_pos_consume = 0;
		}
	}

	if(hdev->flags & PKTE_HOST_MODE){
		adi_write(hdev, CDSC_CNT_OFFSET, 1);

		adi_write(hdev, SA_RDY_OFFSET, adi_physical_address(hdev, (u32)&hdev->pkte_device->pPkteDescriptor.State));

		adi_write(hdev, BUF_THRESH_OFFSET, (u32)128<<BITP_PKTE_BUF_THRESH_INBUF | 
											(u32)128<<BITP_PKTE_BUF_THRESH_OUTBUF);

		adi_write(hdev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_OPDN|BITM_PKTE_IMSK_EN_IBUFTHRSH);
	}else if(hdev->flags & PKTE_TCM_MODE){
		adi_init_spe(hdev);
		adi_init_ring(hdev);
		adi_write(hdev, CDSC_CNT_OFFSET, 1);
		adi_write(hdev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
	}else if(hdev->flags & PKTE_AUTONOMOUS_MODE){

		adi_write(hdev, CDSC_CNT_OFFSET, 1);

		adi_write(hdev, BUF_THRESH_OFFSET, (u32)128<<BITP_PKTE_BUF_THRESH_INBUF | 
											(u32)128<<BITP_PKTE_BUF_THRESH_OUTBUF);

		if(final){
			adi_write(hdev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
		}
	}

	return 0;
}

static struct adi_dev *adi_find_dev(struct adi_ctx *ctx)
{
	struct adi_dev *hdev = NULL, *tmp;

	spin_lock_bh(&adi.lock);
	if (!ctx->hdev) {
		list_for_each_entry(tmp, &adi.dev_list, list) {
			hdev = tmp;
			break;
		}
		ctx->hdev = hdev;
	} else {
		hdev = ctx->hdev;
	}

	spin_unlock_bh(&adi.lock);

	return hdev;
}

static void adi_prepare_secret_key(struct adi_dev *hdev, struct adi_request_ctx *rctx){
	u32 i;
	u8 * source_bytewise;

	//Compute the inner digest / inner key
	adi_prep_engine(hdev, hash_mode_standard);
	memset(&hdev->pkte_device->source[hdev->ring_pos_consume][0], 0, INNER_OUTER_KEY_SIZE);
	memcpy(&hdev->pkte_device->source[hdev->ring_pos_consume][0], hdev->secret_key, hdev->secret_keylen);
	source_bytewise = (u8*)&hdev->pkte_device->source[hdev->ring_pos_consume][0];
	for (i=0; i < INNER_OUTER_KEY_SIZE; i++) {
	        *(source_bytewise+i) ^= 0x36;
	}
	adi_process_packet(hdev, INNER_OUTER_KEY_SIZE, 0);
	if(!(hdev->flags & PKTE_HOST_MODE))
		while( ! (adi_read(hdev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN) );
	while( ! (adi_read(hdev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY) );
	if(hdev->flags & PKTE_HOST_MODE)
		adi_read_packet(hdev, &hdev->pkte_device->pPkteList.pDestination[0]);
	memcpy(IDigest, &hdev->pkte_device->destination[0], hdev->pkte_device->pPkteList.pCommand.digest_length*4);

	//Compute the outer digest / outer key
	adi_prep_engine(hdev, hash_mode_standard);
	memset(&hdev->pkte_device->source[hdev->ring_pos_consume][0], 0, INNER_OUTER_KEY_SIZE);
	memcpy(&hdev->pkte_device->source[hdev->ring_pos_consume][0], hdev->secret_key, hdev->secret_keylen);
	source_bytewise = (u8*)&hdev->pkte_device->source[hdev->ring_pos_consume][0];
	for (i=0; i < INNER_OUTER_KEY_SIZE; i++) {
			*(source_bytewise+i) ^= 0x5c;
	}
	adi_process_packet(hdev, INNER_OUTER_KEY_SIZE, 0);
	if(!(hdev->flags & PKTE_HOST_MODE))
		while( ! (adi_read(hdev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN) );
	while( ! (adi_read(hdev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY) );
	if(hdev->flags & PKTE_HOST_MODE)
		adi_read_packet(hdev, &hdev->pkte_device->pPkteList.pDestination[0]);	
	memcpy(ODigest, &hdev->pkte_device->destination[0], hdev->pkte_device->pPkteList.pCommand.digest_length*4);


	//Now set the engine to compute the HMAC with the data it will receive in subsequent steps
	adi_prep_engine(hdev, hash_mode_hmac);

	hdev->flags |= PKTE_FLAGS_HMAC_KEY_PREPARED;
}

static int adi_setkey(struct crypto_ahash *tfm,
				 const u8 *key, unsigned int keylen)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(tfm);
	struct adi_dev *hdev = adi_find_dev(ctx);
	char temp[256];
	int i = 0, j = 0;
	dev_dbg(hdev->dev, "%s\n", __func__);

	if (keylen <= PKTE_MAX_KEY_SIZE) {

		for(i = 0, j = 0; i < keylen; i++)
			j += sprintf(&temp[j], "%x ", key[i]);
		temp[j] = 0;
		dev_dbg(hdev->dev, "%s HMAC key: %s\n", __func__, temp);

		if (keylen > INNER_OUTER_KEY_SIZE) {
			dev_err(hdev->dev, "Key lengths > %d currently unsupported\n", INNER_OUTER_KEY_SIZE);
			return -ENODEV;
		}else{
			memcpy(hdev->secret_key, key, keylen);
			hdev->secret_keylen = keylen;
		}

	} else {
		return -ENOMEM;
	}

	return 0;
}

static int adi_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_ctx *ctx = crypto_ahash_ctx(tfm);
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = adi_find_dev(ctx);

	rctx->hdev = hdev;
	rctx->digcnt = crypto_ahash_digestsize(tfm);

	dev_dbg(hdev->dev, "adi_init STAT=0x%08x CTL_STAT=0x%08x\n", \
				adi_read(hdev, STAT_OFFSET), adi_read(hdev, CTL_STAT_OFFSET));

	hdev->pkte_device->pPkteList.pCommand.opcode = opcode_hash;
	hdev->pkte_device->pPkteList.pCommand.direction = dir_outbound;
	hdev->pkte_device->pPkteList.pCommand.cipher = cipher_null;
	hdev->pkte_device->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;
	hdev->pkte_device->pPkteList.pCommand.hash_mode = hash_mode_standard;
	hdev->pkte_device->pPkteList.pCommand.aes_key_length = aes_key_length_other;
	hdev->pkte_device->pPkteList.pCommand.aes_des_key = aes_key;
	hdev->pkte_device->pPkteList.pCommand.hash_source = hash_source_no_load;
	hdev->pkte_device->pPkteList.pCommand.final_hash_condition = not_final_hash;

	switch (rctx->digcnt) {
		case MD5_DIGEST_SIZE:
			dev_dbg(hdev->dev, "adi_init, selected MD5 hashing\n");
			hdev->pkte_device->pPkteList.pCommand.hash = hash_md5;
			hdev->pkte_device->pPkteList.pCommand.digest_length = digest_length4;
			break;
		case SHA1_DIGEST_SIZE:
			dev_dbg(hdev->dev, "adi_init, selected SHA1 hashing\n");
			hdev->pkte_device->pPkteList.pCommand.hash = hash_sha1;
			hdev->pkte_device->pPkteList.pCommand.digest_length = digest_length5;
			break;
		case SHA224_DIGEST_SIZE:
			dev_dbg(hdev->dev, "adi_init, selected SHA224 hashing\n");
			hdev->pkte_device->pPkteList.pCommand.hash = hash_sha224;
			hdev->pkte_device->pPkteList.pCommand.digest_length = digest_length7;
			if(unlikely(hdev->flags & PKTE_FLAGS_HMAC)){
					dev_err(hdev->dev, "HMAC computation currently unsupported for SHA224\n");
					adi_reset_state(hdev);
					return -ENODEV;
			}
			break;
		case SHA256_DIGEST_SIZE:
			dev_dbg(hdev->dev, "adi_init, selected SHA256 hashing\n");
			hdev->pkte_device->pPkteList.pCommand.hash = hash_sha256;
			hdev->pkte_device->pPkteList.pCommand.digest_length = digest_length8;
			break;
		default:
			dev_dbg(hdev->dev, "adi_init, unknown hashing request\n");
			return -EINVAL;
	}

	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->offset = 0;
	rctx->buflen = PKTE_BUFLEN;

	hdev->ring_pos_produce = 0;
	hdev->ring_pos_consume = 0;
	processing = 0;

	return 0;
}

static int adi_update_req(struct adi_dev *hdev)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(hdev->req);
	int bufcnt, err = 0, final;

	dev_dbg(hdev->dev, "%s flags %lx\n", __func__, hdev->flags);

	final = (hdev->flags & PKTE_FLAGS_FINUP);

	if(rctx->total > rctx->buflen){
		while (rctx->total > rctx->buflen){
			adi_append_sg(rctx, hdev);	
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(hdev, bufcnt, 0);
		}
	}else{
		if ((rctx->total >= (rctx->buflen)) ||
			   (rctx->bufcnt + rctx->total >= (rctx->buflen))) {
			adi_append_sg(rctx, hdev);
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(hdev, bufcnt, 0);
		}
	}

	adi_append_sg(rctx, hdev);

	if (final) {
		bufcnt = rctx->bufcnt;
		rctx->bufcnt = 0;
		err = adi_process_packet(hdev, bufcnt, 1);
	}

	return err;
}

static int adi_final_req(struct adi_dev *hdev)
{
	struct ahash_request *req = hdev->req;
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	int err;
	int buflen;

	dev_dbg(hdev->dev, "%s\n", __func__);

	buflen = rctx->bufcnt;
	rctx->bufcnt = 0;

	err = adi_process_packet(hdev, buflen, 1);

	return err;
}

static void adi_copy_hash(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;
	u32 *hash = (u32 *)rctx->digest;
	u32 i;

	dev_dbg(hdev->dev, "%s\n", __func__);

	rctx->digcnt = rctx->hdev->pkte_device->pPkteList.pCommand.digest_length*4;

	//Endian Swap!
	if(hdev->pkte_device->pPkteList.pCommand.hash == hash_md5){
		memcpy(hash, rctx->hdev->pkte_device->pPkteList.pDestination, rctx->digcnt);
	}else{
		for(i = 0; i < rctx->digcnt/4; i++){
			hash[i] = __builtin_bswap32(rctx->hdev->pkte_device->pPkteList.pDestination[i]);
		}
	}

}

static int adi_finish(struct ahash_request *req)
{

	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s\n", __func__);

	adi_reset_state(hdev);

	if (!req->result){
		dev_dbg(hdev->dev, "%s: error\n", __func__);
		return -EINVAL;
	}

	memcpy(req->result, rctx->digest, rctx->digcnt);

	return 0;
}

static void adi_finish_req(struct ahash_request *req, int err)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s\n", __func__);

	if (!err && (PKTE_FLAGS_FINAL & hdev->flags)) {
		wait_event_interruptible(wq_ready, ready == 1);
		while( ! (adi_read(hdev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY) );

		//if(hdev->flags & PKTE_HOST_MODE)
			adi_read_packet(hdev, &hdev->pkte_device->pPkteList.pDestination[0]);

		adi_copy_hash(req);
		err = adi_finish(req);
	}

	crypto_finalize_hash_request(hdev->engine, req, err);
}

static int adi_hw_init(struct adi_dev *hdev,
				  struct adi_request_ctx *rctx, u32 nbytes)
{


	dev_dbg(hdev->dev, "%s\n", __func__);

	hdev->src_count_set = 0;
	hdev->src_bytes_available = 0;
	ready = 0;
	processing = 0;

	if (likely(!(hdev->flags & PKTE_FLAGS_HMAC))){
		//Continue with previous operation
		if(!(hdev->flags & PKTE_FLAGS_STARTED)){
			adi_start_engine(hdev);
		}
	}

	return 0;
}

static int adi_one_request(struct crypto_engine *engine, void *areq);
static int adi_prepare_req(struct crypto_engine *engine, void *areq);

static int adi_handle_queue(struct adi_dev *hdev,
				   struct ahash_request *req)
{
	return crypto_transfer_hash_request_to_engine(hdev->engine, req);
}

static int adi_prepare_req(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *hdev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;

	dev_dbg(hdev->dev, "%s\n", __func__);

	if (!hdev)
		return -ENODEV;

	hdev->req = req;

	rctx = ahash_request_ctx(req);

	dev_dbg(hdev->dev, "processing new req, op: %lu, nbytes %d\n",
		rctx->op, req->nbytes);

	if(unlikely(hdev->flags & PKTE_FLAGS_HMAC)){
		if(req->nbytes > PKTE_BUFLEN){
			dev_err(hdev->dev, "HMAC computation unsupported on > %d bytes \n", PKTE_BUFLEN);
			adi_reset_state(hdev);
			return -ENODEV;
		}
	}

	return adi_hw_init(hdev, rctx, req->nbytes);
}

static int adi_one_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *hdev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;
	int err = 0;

	dev_dbg(hdev->dev, "%s\n", __func__);

	if (!hdev)
		return -ENODEV;

	hdev->req = req;

	rctx = ahash_request_ctx(req);

	switch(rctx->op){
		case PKTE_OP_UPDATE:
			err = adi_update_req(hdev);
			break;
		case PKTE_OP_FINAL:
			err = adi_final_req(hdev);
			break;
	}

	if (err != -EINPROGRESS)
		adi_finish_req(req, err);

	return 0;

}

static int adi_enqueue(struct ahash_request *req, unsigned int op)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s\n", __func__);

	rctx->op = op;

	return adi_handle_queue(hdev, req);
}

static int adi_update(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s\n", __func__);

	if (unlikely(hdev->flags & PKTE_FLAGS_HMAC)){
		if(hdev->flags & PKTE_HOST_MODE){
			dev_err(hdev->dev, "HMAC computation not yet supported while in Direct Host Mode\n");
			return -ENODEV;
		}
		if(!(hdev->flags & PKTE_FLAGS_HMAC_KEY_PREPARED))
			adi_prepare_secret_key(hdev, rctx);
	}

	if (!req->nbytes)
		return 0;

	rctx->total = req->nbytes;
	rctx->sg = req->src;
	rctx->offset = 0;

	if ((rctx->bufcnt + rctx->total <= rctx->buflen)) {
		adi_append_sg(rctx, hdev);
		return 0;
	}

	return adi_enqueue(req, PKTE_OP_UPDATE);
}

static int adi_final(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s\n", __func__);

	hdev->flags |= PKTE_FLAGS_FINUP;

	return adi_enqueue(req, PKTE_OP_FINAL);
}

static int adi_finup(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *hdev = rctx->hdev;

	dev_dbg(hdev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_digest(struct ahash_request *req)
{
	return adi_init(req) ?: adi_finup(req);
}

static int adi_export(struct ahash_request *req, void *out)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *hdev = adi_find_dev(ctx);

	dev_dbg(hdev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_import(struct ahash_request *req, const void *in)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *hdev = adi_find_dev(ctx);

	dev_dbg(hdev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_cra_init_algs(struct crypto_tfm *tfm,
					const char *algs_hmac_name)
{
	struct adi_ctx *ctx = crypto_tfm_ctx(tfm);
	struct adi_dev *hdev = adi_find_dev(ctx);

	dev_dbg(hdev->dev, "%s\n", __func__);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct adi_request_ctx));

	hdev->secret_keylen = 0;
	if (algs_hmac_name){
		hdev->flags |= PKTE_FLAGS_HMAC;
	}

	ctx->enginectx.op.do_one_request = adi_one_request;
	ctx->enginectx.op.prepare_request = adi_prepare_req;
	ctx->enginectx.op.unprepare_request = NULL;

	return 0;
}

static int adi_cra_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, NULL);
}

static int adi_cra_md5_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "md5");
}

static int adi_cra_sha1_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha1");
}

static int adi_cra_sha224_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha224");
}

static int adi_cra_sha256_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha256");
}

static irqreturn_t adi_irq_handler(int irq, void *dev_id)
{
	struct adi_dev *hdev = dev_id;
	u32 temp = adi_read(hdev, IMSK_STAT_OFFSET);
	u32 temp2;

	dev_dbg(hdev->dev, "%s %x\n", __func__, temp);

	if(temp & BITM_PKTE_IMSK_EN_RINGERR){
		dev_dbg(hdev->dev, "%s: RINGERR, check error codes %x", __func__, adi_read(hdev, RING_STAT_OFFSET));
		adi_write(hdev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RINGERR);
	}

	if(temp & BITM_PKTE_IMSK_EN_PROCERR){
		dev_dbg(hdev->dev, "%s: PROCERR, check error codes %x", __func__, adi_read(hdev, CTL_STAT_OFFSET));
		adi_write(hdev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_PROCERR);
	}

	if(temp & BITM_PKTE_IMSK_EN_RDRTHRSH){
		if(hdev->flags & PKTE_TCM_MODE){
			adi_write(hdev, RDSC_DECR_OFFSET, 0x1);
		}

		adi_write(hdev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
		temp2 = adi_read(hdev, IMSK_DIS_OFFSET);

		adi_write(hdev, IMSK_DIS_OFFSET, temp2 | BITM_PKTE_IMSK_EN_RDRTHRSH);


		if(hdev->pkte_device->pPkteList.pCommand.final_hash_condition == final_hash){
			ready = 1;
			hdev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}

		if(hdev->flags & PKTE_TCM_MODE){
			processing = 0;
			wake_up_interruptible(&wq_processing);
		}
	}

	if(temp & BITM_PKTE_IMSK_EN_OPDN){
		adi_write(hdev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_OPDN);
		temp2 = adi_read(hdev, IMSK_DIS_OFFSET);
		adi_write(hdev, IMSK_DIS_OFFSET, temp2 | BITM_PKTE_IMSK_EN_OPDN|BITM_PKTE_IMSK_EN_IBUFTHRSH);
		adi_write(hdev, RDSC_DECR_OFFSET, 0x1);

		if(hdev->pkte_device->pPkteList.pCommand.final_hash_condition == final_hash){
			ready = 1;
			hdev->flags |= PKTE_FLAGS_COMPLETE;
			wake_up_interruptible(&wq_ready);
		}
		processing = 0;
		wake_up_interruptible(&wq_processing);
	}

	if(temp & BITM_PKTE_IMSK_EN_IBUFTHRSH){
		if(hdev->src_bytes_available){
			adi_write_packet(hdev, &hdev->pkte_device->pPkteList.pSource[0]);
			adi_write(hdev, INT_CLR_OFFSET, BITM_PKTE_IMSK_EN_IBUFTHRSH);
		}else{
			adi_write(hdev, INT_EN_OFFSET, 0);			
		}
	}

	return IRQ_HANDLED;
}

static struct ahash_alg algs_md5_sha1[] = {
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.halg = {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "md5",
				.cra_driver_name = "adi-md5",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.setkey = adi_setkey,
		.halg = {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "hmac(md5)",
				.cra_driver_name = "adi-hmac-md5",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_md5_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "sha1",
				.cra_driver_name = "adi-sha1",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.setkey = adi_setkey,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "hmac(sha1)",
				.cra_driver_name = "adi-hmac-sha1",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_sha1_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
};

static struct ahash_alg algs_sha224_sha256[] = {
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.halg = {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "sha224",
				.cra_driver_name = "adi-sha224",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA224_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.setkey = adi_setkey,
		.export = adi_export,
		.import = adi_import,
		.halg = {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "hmac(sha224)",
				.cra_driver_name = "adi-hmac-sha224",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA224_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_sha224_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "sha256",
				.cra_driver_name = "adi-sha256",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA256_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
	{
		.init = adi_init,
		.update = adi_update,
		.final = adi_final,
		.finup = adi_finup,
		.digest = adi_digest,
		.export = adi_export,
		.import = adi_import,
		.setkey = adi_setkey,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct adi_request_ctx),
			.base = {
				.cra_name = "hmac(sha256)",
				.cra_driver_name = "adi-hmac-sha256",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA256_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_alignmask = 3,
				.cra_init = adi_cra_sha256_init,
				.cra_module = THIS_MODULE,
			}
		}
	},
};

static int adi_register_algs(struct adi_dev *hdev)
{
	unsigned int i, j;
	int err;

	for (i = 0; i < hdev->pdata->algs_info_size; i++) {
		for (j = 0; j < hdev->pdata->algs_info[i].size; j++) {
			err = crypto_register_ahash(
				&hdev->pdata->algs_info[i].algs_list[j]);
			if (err)
				goto err_algs;
		}
	}

	return 0;
err_algs:
	dev_dbg(hdev->dev, "Algo %d : %d failed\n", i, j);
	for (; i--; ) {
		for (; j--;)
			crypto_unregister_ahash(
				&hdev->pdata->algs_info[i].algs_list[j]);
	}

	return err;
}

static int adi_unregister_algs(struct adi_dev *hdev)
{
	unsigned int i, j;

	for (i = 0; i < hdev->pdata->algs_info_size; i++) {
		for (j = 0; j < hdev->pdata->algs_info[i].size; j++)
			crypto_unregister_ahash(
				&hdev->pdata->algs_info[i].algs_list[j]);
	}

	return 0;
}


static struct adi_algs_info adi_algs_info_adi[] = {
	{
		.algs_list	= algs_md5_sha1,
		.size		= ARRAY_SIZE(algs_md5_sha1),
	},
	{
		.algs_list	= algs_sha224_sha256,
		.size		= ARRAY_SIZE(algs_sha224_sha256),
	},
};

static const struct adi_pdata adi_pdata_adi = {
	.algs_info	= adi_algs_info_adi,
	.algs_info_size	= ARRAY_SIZE(adi_algs_info_adi),
};

static const struct of_device_id adi_of_match[] = {
	{
		.compatible = "adi,pkte",
		.data = &adi_pdata_adi,
	},
	{},
};

MODULE_DEVICE_TABLE(of, adi_of_match);

static int adi_get_of_match(struct adi_dev *hdev,
				   struct device *dev)
{
	hdev->pdata = of_device_get_match_data(dev);
	if (!hdev->pdata) {
		dev_dbg(dev, "no compatible OF match\n");
		return -EINVAL;
	}

	return 0;
}

static int adi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;	
	struct adi_dev *hdev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret, irq;
	u32 temp;
	const char *mode;

	hdev = devm_kzalloc(dev, sizeof(*hdev), GFP_KERNEL);
	if (!hdev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdev->io_base = devm_ioremap_resource(dev, res);

	if (IS_ERR(hdev->io_base))
		return PTR_ERR(hdev->io_base);

	hdev->phys_base = res->start;

	ret = adi_get_of_match(hdev, dev);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hdev->dev = dev;

	ret = of_property_read_string(np, "mode", &mode);
	if (ret < 0) {
		if (ret != -EINVAL)
			dev_err(hdev->dev, "%s: could not parse mode\n", of_node_full_name(np));
		return -EINVAL;
	}

	if(strcmp(mode, "tcm") == 0){
		hdev->flags |= PKTE_TCM_MODE;
		dev_info(hdev->dev, "Selected TCM mode\n");
	}else if(strcmp(mode, "dhm") == 0){
		hdev->flags |= PKTE_HOST_MODE;
		dev_info(hdev->dev, "Selected DHM mode\n");
	}else{
		hdev->flags |= PKTE_AUTONOMOUS_MODE;
		dev_info(hdev->dev, "Selected ARM mode\n");		
	}

	#ifdef PKTE_USE_SRAM
		hdev->pkte_device = ioremap(PKTE_SRAM_ADDRESS, sizeof(ADI_PKTE_DEVICE));
		dev_dbg(hdev->dev, "sram/ioremap %x @ %x\n", sizeof(ADI_PKTE_DEVICE), (u32)hdev->pkte_device);
	#else
		hdev->pkte_device = dma_alloc_coherent(hdev->dev, sizeof(ADI_PKTE_DEVICE), &hdev->dma_handle, GFP_KERNEL);
		dev_dbg(hdev->dev, "dma_alloc_coherent %x @ %x\n", sizeof(ADI_PKTE_DEVICE), (u32)hdev->pkte_device);
	#endif

	//Disable interrupts until we're using the PKTE
	temp = adi_read(hdev, INT_CFG_OFFSET);
	adi_write(hdev, INT_CFG_OFFSET, temp & ~BITM_PKTE_INT_CFG_TYPE); /* PKTE0 Interrupt Level Sensitive */
	adi_write(hdev, INT_CLR_OFFSET, 0x00071E03); /* PKTE0 Interrupt Clear Register */
	adi_write(hdev, IMSK_DIS_OFFSET,
									 BITM_PKTE_IMSK_EN_IFERR|
									 BITM_PKTE_IMSK_EN_RDRTHRSH|
									 BITM_PKTE_IMSK_EN_RINGERR|
									 BITM_PKTE_IMSK_EN_PROCERR|
									 BITM_PKTE_IMSK_EN_HLT|
									 BITM_PKTE_IMSK_EN_CDRTHRSH|
									 BITM_PKTE_IMSK_EN_OPDN|
									 BITM_PKTE_IMSK_EN_IBUFTHRSH |
									 BITM_PKTE_IMSK_EN_OBUFTHRSH); /* Disable All Interrupts */

	ret = devm_request_irq(dev, irq, adi_irq_handler,
					IRQF_SHARED,
					dev_name(dev), hdev);	
	if (ret) {
		dev_dbg(dev, "Cannot grab IRQ\n");
		return ret;
	}

	platform_set_drvdata(pdev, hdev);

	spin_lock(&adi.lock);
	list_add_tail(&hdev->list, &adi.dev_list);
	spin_unlock(&adi.lock);

	/* Initialize crypto engine */
	hdev->engine = crypto_engine_alloc_init(dev, 1);
	if (!hdev->engine) {
		ret = -ENOMEM;
		goto err_engine;
	}

	ret = crypto_engine_start(hdev->engine);
	if (ret)
		goto err_engine_start;

	/* Register algos */
	ret = adi_register_algs(hdev);
	if (ret)
		goto err_algs;

	return 0;

err_algs:
err_engine_start:
	crypto_engine_exit(hdev->engine);
err_engine:
	spin_lock(&adi.lock);
	list_del(&hdev->list);
	spin_unlock(&adi.lock);

	return ret;
}

static int adi_remove(struct platform_device *pdev)
{
	struct adi_dev *hdev;

	hdev = platform_get_drvdata(pdev);
	if (!hdev)
		return -ENODEV;

	adi_unregister_algs(hdev);

	crypto_engine_exit(hdev->engine);

	spin_lock(&adi.lock);
	list_del(&hdev->list);
	spin_unlock(&adi.lock);

#ifndef PKTE_USE_SRAM
	dma_free_coherent(hdev->dev, sizeof(ADI_PKTE_DEVICE), hdev->pkte_device, hdev->dma_handle);
#endif

	return 0;
}

static struct platform_driver adi_driver = {
	.probe		= adi_probe,
	.remove		= adi_remove,
	.driver		= {
		.name	= "adi-hash",
		.of_match_table	= adi_of_match,
	}
};

module_platform_driver(adi_driver);

MODULE_DESCRIPTION("ADI SHA1/224/256 & MD5 HW Acceleration");
MODULE_AUTHOR("Nathan Barrett-Morrison <nathan.morrison@timesys.com>");
MODULE_LICENSE("GPL v2");
 
