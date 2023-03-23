/*
 * drivers/crypto/adi/adi-pkte-skcipher.c
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
#include <crypto/scatterwalk.h>
#include <linux/dma-mapping.h>

/*Hashing*/
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/hash.h>

/*Symmetric Crytography*/
#include <crypto/aes.h>
#include <crypto/internal/des.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>

#include "adi-pkte.h"
#include "adi-pkte-hash.h"

void adi_write_packet(struct adi_dev *pkte_dev, u32 *source)
{
	u32 i, nLength, SIZE, nPadLength = 0, nTotalSize;
	u32 *temp;
	u32 temp1;
	uint8_t iBufIncrement = 0;

	temp = source;
	temp1 = (adi_read(pkte_dev, BUF_PTR_OFFSET) & BITM_PKTE_BUF_PTR_INBUF) >> BITP_PKTE_BUF_PTR_INBUF;
	temp1 += DATAIO_BUF_OFFSET;

	nLength = pkte_dev->src_bytes_available;
	SIZE = (nLength > 128) ? 128 : (nLength);

	if (SIZE % 4) {
		for (i = 0; i < 1+SIZE/4; i++)
		{
			adi_write(pkte_dev, temp1, *temp);
			temp1 += 4;
			temp++;
		}
	} else {
		for (i = 0; i < SIZE/4; i++)
		{
			adi_write(pkte_dev, temp1, *temp);
			temp1 += 4;
			temp++;
		}
	}

	dev_dbg(pkte_dev->dev, "adi_write_packet: wrote %x bytes\n", SIZE);

	switch (pkte_dev->pkte_device->pPkteList.pCommand.opcode) {
		case opcode_hash:
			//Nothing to do
			break;
		default:
			//Pad to 16 byte alignment/boundaries
			if (SIZE % 16 != 0)
			{
				nTotalSize = ((SIZE/16)+1)*16;
				nPadLength = nTotalSize - SIZE;
				for (i = 0; i < nPadLength; i++)
				{
					writeb(0x00, pkte_dev->io_base + temp1);
					temp1++;
				}
			}
			break;
	}

	pkte_dev->pkte_device->pPkteList.pSource = pkte_dev->pkte_device->pPkteList.pSource + SIZE/(u32)4;

	iBufIncrement = (uint8_t)SIZE + (uint8_t)nPadLength;
	if (iBufIncrement % 4) {
		iBufIncrement += (4-iBufIncrement%4);
	}

	if (iBufIncrement > pkte_dev->src_bytes_available) {
		pkte_dev->src_bytes_available = 0;
	} else {
		pkte_dev->src_bytes_available -= iBufIncrement;
	}

	adi_write(pkte_dev, INBUF_INCR_OFFSET, (u32) iBufIncrement);
}

static void adi_read_packet(struct adi_dev *pkte_dev, u32 *destination)
{
	u32 i = 0;
	u32 nLength = 0, SIZE = 0;
	uint8_t iBufIncrement = 0;
	u32 *temp;
	u32 temp1 = 0, temp2 = 0;
	u8 pos;
	temp = destination;
	temp2 = adi_read(pkte_dev, IMSK_STAT_OFFSET);
	pos = pkte_dev->flags & PKTE_AUTONOMOUS_MODE ? pkte_dev->ring_pos_consume : 0;

	if ((temp2 & BITM_PKTE_IMSK_EN_OPDN) != BITM_PKTE_IMSK_EN_OPDN)
	{
		temp1 = DATAIO_BUF_OFFSET;

		nLength = (u32)(pkte_dev->pkte_device->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR +
					pkte_dev->pkte_device->pPkteList.nSrcSize -
					adi_physical_address(pkte_dev, (u32)pkte_dev->pkte_device->pPkteList.pDestination));
		SIZE = (nLength > 128) ? 128:nLength;

		if (pkte_dev->pkte_device->pPkteList.pCommand.opcode == opcode_hash)
		{
			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_md5) {
				SIZE = 4*4;  /* Destination will be 4 word hash value */
			}

			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha1) {
				SIZE = 5*4;  /* Destination will be 5 word hash value */
			}

			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha224) {
				SIZE = 7*4;  /* Destination will be 7 word hash value */
			}

			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha256) {
				SIZE = 8*4;  /* Destination will be 8 word hash value */
			}
			if (SIZE % 4) {
				dev_dbg(pkte_dev->dev, "adi_read_packet: Unhandled Case - 4-byte alignment would fail\n");
			} else {
				for (i = 0; i < SIZE/4; i++)
				{
					*temp = adi_read(pkte_dev, temp1);
					dev_dbg(pkte_dev->dev, "adi_read_packet read %x\n", *temp);
					temp1 += 4;
					temp++;
				}
			}
		} else if(pkte_dev->pkte_device->pPkteList.pCommand.opcode == opcode_encrypt_hash)
		{
			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_md5) {
				SIZE = SIZE + 4*4;  /* Destination will be 4 word hash value */
			}
			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha1) {
				SIZE = SIZE + 5*4;  /* Destination will be 5 word hash value */
			}
			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha224) {
				SIZE = SIZE + 7*4;  /* Destination will be 7 word hash value */
			}
			if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_sha256) {
				SIZE = SIZE + 8*4;  /* Destination will be 8 word hash value */
			}


			for (i = 0; i < SIZE; i++)
			{
				*temp = adi_read(pkte_dev, temp1);
				temp1++;
				temp++;
			}
		}

		else {
			if (SIZE % 16 != 0) {
				SIZE = ((SIZE/16)+1)*16;  /*For encryption 16 byte boundary is required */
			}
			for (i = 0; i < SIZE; i++)
			{
				*temp = adi_read(pkte_dev, temp1);
				temp1++;
				temp++;
			}
		}


		//pkte_dev->pkte_device->pPkteList.pDestination = pkte_dev->pkte_device->pPkteList.pDestination + SIZE/4;
		iBufIncrement = (uint8_t)SIZE;
		if (iBufIncrement % 4) {
			iBufIncrement = ((iBufIncrement/4)+1)*4;
		}
		adi_write(pkte_dev, OUTBUF_DECR_OFFSET, iBufIncrement);
	}
}

static void adi_append_sg(struct adi_request_ctx *rctx, struct adi_dev *pkte_dev)
{
	size_t count;

	dev_dbg(rctx->pkte_dev->dev, "adi_append_sg\n");

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

		scatterwalk_map_and_copy(((u8 *)&pkte_dev->pkte_device->source[pkte_dev->ring_pos_produce][0]) + rctx->bufcnt, rctx->sg,
					 rctx->offset, count, 0);

		rctx->bufcnt += count;
		rctx->offset += count;
		rctx->total -= count;

		if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
			if (rctx->bufcnt >= PKTE_BUFLEN) {
				pkte_dev->ring_pos_produce++;
				if (pkte_dev->ring_pos_produce >= PKTE_RING_BUFFERS) {
					pkte_dev->ring_pos_produce = 0;
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

static void adi_prep_engine(struct adi_dev *pkte_dev, u32 hash_mode)
{

	pkte_dev->src_count_set = 0;
	pkte_dev->src_bytes_available = 0;
	pkte_dev->ring_pos_produce = 0;
	pkte_dev->ring_pos_consume = 0;
	ready = 0;
	processing = 0;

	pkte_dev->pkte_device->pPkteList.pCommand.opcode = opcode_hash;
	pkte_dev->pkte_device->pPkteList.pCommand.direction = dir_outbound;
	pkte_dev->pkte_device->pPkteList.pCommand.cipher = cipher_null;
	pkte_dev->pkte_device->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;
	pkte_dev->pkte_device->pPkteList.pCommand.hash_mode = hash_mode;
	pkte_dev->pkte_device->pPkteList.pCommand.aes_key_length = aes_key_length_other;
	pkte_dev->pkte_device->pPkteList.pCommand.aes_des_key = aes_key;
	pkte_dev->pkte_device->pPkteList.pCommand.hash_source = hash_source_no_load;

	pkte_dev->flags &= ~PKTE_FLAGS_STARTED;

	adi_start_engine(pkte_dev);
}

static int adi_process_packet(struct adi_dev *pkte_dev,
				   size_t length, int final)
{
	unsigned int len32;
	u32 temp;

	len32 = DIV_ROUND_UP(length, sizeof(u32));

	dev_dbg(pkte_dev->dev, "%s: length: %zd, final: %x len32 %i\n",
		__func__, length, final, len32);

	//Set final hash condition
	if (final) {
		pkte_dev->flags |= PKTE_FLAGS_FINAL;
		dev_dbg(pkte_dev->dev, "%s final hash condition set\n", __func__);
		if (!(pkte_dev->flags & PKTE_AUTONOMOUS_MODE)) {		
			pkte_dev->pkte_device->pPkteList.pCommand.final_hash_condition = final_hash;
			temp = (u32)0x1|(final_hash<<BITP_PKTE_CTL_STAT_HASHFINAL);
			adi_write(pkte_dev, CTL_STAT_OFFSET, temp);
		} else {
			pkte_dev->pkte_device->pPkteList.pCommand.final_hash_condition = final_hash;
		}
	}

	if (final && (length == 0)) {
		ready = 1;
		wake_up_interruptible(&wq_ready);
		return 0;
	}

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_HOST_MODE)) {
		wait_event_interruptible(wq_processing, processing == 0);
		processing = 1;
	}
	while( ! (adi_read(pkte_dev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN) );
	while (!(adi_read(pkte_dev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY));

	pkte_dev->pkte_device->pPkteList.pSource = &pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0];

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
		adi_configure_cdr(pkte_dev);
	}

	pkte_dev->pkte_device->pPkteList.pDestination = &pkte_dev->pkte_device->destination[0];

	adi_source_data(pkte_dev, length);

	pkte_dev->src_bytes_available = length;

	if (pkte_dev->flags & PKTE_TCM_MODE) {
		adi_config_sa_para(pkte_dev);
	}

	//Continue with previous operation
	if (pkte_dev->flags & PKTE_FLAGS_STARTED) {
		dev_dbg(pkte_dev->dev, "%s hash_source_state set\n", __func__);

		if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
			temp = pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume].SA_Para.SA_CMD0;
			temp &= ~BITM_PKTE_SA_CMD0_HASHSRC;
			temp |= hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
			pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume].SA_Para.SA_CMD0 = temp;
		} else {
			//Set HASHSRC to hash_source_state
			temp = adi_read(pkte_dev, SA_CMD_OFFSET(0));
			temp &= ~BITM_PKTE_SA_CMD0_HASHSRC;
			temp |= hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
			pkte_dev->pkte_device->pPkteDescriptor.SARecord[pkte_dev->ring_pos_consume].SA_Para.SA_CMD0 = temp;
			adi_write(pkte_dev, SA_CMD_OFFSET(0), temp);
		}
	} else {
		pkte_dev->flags |= PKTE_FLAGS_STARTED;
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
					   (long)pkte_dev->pkte_device,
					  ((long)pkte_dev->pkte_device) + sizeof(ADI_PKTE_DEVICE));
		}
	}
#endif

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
		pkte_dev->ring_pos_consume++;
		if (adi_read(pkte_dev, RDSC_CNT_OFFSET)) {
			adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);
		}
		if (pkte_dev->ring_pos_consume >= PKTE_RING_BUFFERS) {
			pkte_dev->ring_pos_consume = 0;
		}
	}

	if (pkte_dev->flags & PKTE_HOST_MODE) {
		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);

		adi_write(pkte_dev, SA_RDY_OFFSET, adi_physical_address(pkte_dev, (u32)&pkte_dev->pkte_device->pPkteDescriptor.State));

		adi_write(pkte_dev, BUF_THRESH_OFFSET, (u32)128<<BITP_PKTE_BUF_THRESH_INBUF |
											(u32)128<<BITP_PKTE_BUF_THRESH_OUTBUF);

		adi_write(pkte_dev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_OPDN|BITM_PKTE_IMSK_EN_IBUFTHRSH);
	} else if (pkte_dev->flags & PKTE_TCM_MODE) {
		adi_init_spe(pkte_dev);
		adi_init_ring(pkte_dev);
		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);
		adi_write(pkte_dev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
	} else if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {

		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);

		adi_write(pkte_dev, BUF_THRESH_OFFSET, (u32)128<<BITP_PKTE_BUF_THRESH_INBUF |
											(u32)128<<BITP_PKTE_BUF_THRESH_OUTBUF);

		if (final) {
			adi_write(pkte_dev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
		}
	}

	return 0;
}

static void adi_prepare_secret_key(struct adi_dev *pkte_dev, struct adi_request_ctx *rctx)
{
	u32 i;
	u8 *source_bytewise;

	//Compute the inner digest / inner key
	adi_prep_engine(pkte_dev, hash_mode_standard);
	memset(&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0], 0, INNER_OUTER_KEY_SIZE);
	memcpy(&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0], pkte_dev->secret_key, pkte_dev->secret_keylen);
	source_bytewise = (u8 *)&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0];
	for (i = 0; i < INNER_OUTER_KEY_SIZE; i++) {
		*(source_bytewise+i) ^= 0x36;
	}
	adi_process_packet(pkte_dev, INNER_OUTER_KEY_SIZE, 0);
	if (!(pkte_dev->flags & PKTE_HOST_MODE))
		while (!(adi_read(pkte_dev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN));
	while (!(adi_read(pkte_dev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY));
	if (pkte_dev->flags & PKTE_HOST_MODE)
		adi_read_packet(pkte_dev, &pkte_dev->pkte_device->pPkteList.pDestination[0]);
	memcpy(IDigest, &pkte_dev->pkte_device->destination[0], pkte_dev->pkte_device->pPkteList.pCommand.digest_length*4);

	//Compute the outer digest / outer key
	adi_prep_engine(pkte_dev, hash_mode_standard);
	memset(&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0], 0, INNER_OUTER_KEY_SIZE);
	memcpy(&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0], pkte_dev->secret_key, pkte_dev->secret_keylen);
	source_bytewise = (u8 *)&pkte_dev->pkte_device->source[pkte_dev->ring_pos_consume][0];
	for (i = 0; i < INNER_OUTER_KEY_SIZE; i++) {
			*(source_bytewise+i) ^= 0x5c;
	}
	adi_process_packet(pkte_dev, INNER_OUTER_KEY_SIZE, 0);
	if (!(pkte_dev->flags & PKTE_HOST_MODE))
		while (!(adi_read(pkte_dev, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN));
	while (!(adi_read(pkte_dev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY));
	if (pkte_dev->flags & PKTE_HOST_MODE)
		adi_read_packet(pkte_dev, &pkte_dev->pkte_device->pPkteList.pDestination[0]);
	memcpy(ODigest, &pkte_dev->pkte_device->destination[0], pkte_dev->pkte_device->pPkteList.pCommand.digest_length*4);


	//Now set the engine to compute the HMAC with the data it will receive in subsequent steps
	adi_prep_engine(pkte_dev, hash_mode_hmac);

	pkte_dev->flags |= PKTE_FLAGS_HMAC_KEY_PREPARED;
}

static int adi_setkey(struct crypto_ahash *tfm,
				 const u8 *key, unsigned int keylen)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(tfm);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	char temp[256];
	int i = 0, j = 0;
	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	if (keylen <= PKTE_MAX_KEY_SIZE) {

#ifdef DEBUG_PKTE
		for (i = 0, j = 0; i < keylen; i++)
			j += sprintf(&temp[j], "%x ", key[i]);
		temp[j] = 0;
		dev_dbg(pkte_dev->dev, "%s HMAC key: %s\n", __func__, temp);
#endif

		if (keylen > INNER_OUTER_KEY_SIZE) {
			dev_err(pkte_dev->dev, "Key lengths > %d currently unsupported\n", INNER_OUTER_KEY_SIZE);
			return -ENODEV;
		} else {
			memcpy(pkte_dev->secret_key, key, keylen);
			pkte_dev->secret_keylen = keylen;
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
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	rctx->pkte_dev = pkte_dev;
	rctx->digcnt = crypto_ahash_digestsize(tfm);

	dev_dbg(pkte_dev->dev, "adi_init STAT=0x%08x CTL_STAT=0x%08x\n", \
				adi_read(pkte_dev, STAT_OFFSET), adi_read(pkte_dev, CTL_STAT_OFFSET));

	pkte_dev->pkte_device->pPkteList.pCommand.opcode = opcode_hash;
	pkte_dev->pkte_device->pPkteList.pCommand.direction = dir_outbound;
	pkte_dev->pkte_device->pPkteList.pCommand.cipher = cipher_null;
	pkte_dev->pkte_device->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;
	pkte_dev->pkte_device->pPkteList.pCommand.hash_mode = hash_mode_standard;
	pkte_dev->pkte_device->pPkteList.pCommand.aes_key_length = aes_key_length_other;
	pkte_dev->pkte_device->pPkteList.pCommand.aes_des_key = aes_key;
	pkte_dev->pkte_device->pPkteList.pCommand.hash_source = hash_source_no_load;
	pkte_dev->pkte_device->pPkteList.pCommand.final_hash_condition = not_final_hash;

	switch (rctx->digcnt) {
		case MD5_DIGEST_SIZE:
			dev_dbg(pkte_dev->dev, "adi_init, selected MD5 hashing\n");
			pkte_dev->pkte_device->pPkteList.pCommand.hash = hash_md5;
			pkte_dev->pkte_device->pPkteList.pCommand.digest_length = digest_length4;
			break;
		case SHA1_DIGEST_SIZE:
			dev_dbg(pkte_dev->dev, "adi_init, selected SHA1 hashing\n");
			pkte_dev->pkte_device->pPkteList.pCommand.hash = hash_sha1;
			pkte_dev->pkte_device->pPkteList.pCommand.digest_length = digest_length5;
			break;
		case SHA224_DIGEST_SIZE:
			dev_dbg(pkte_dev->dev, "adi_init, selected SHA224 hashing\n");
			pkte_dev->pkte_device->pPkteList.pCommand.hash = hash_sha224;
			pkte_dev->pkte_device->pPkteList.pCommand.digest_length = digest_length7;
			if (unlikely(pkte_dev->flags & PKTE_FLAGS_HMAC)) {
					dev_err(pkte_dev->dev, "HMAC computation currently unsupported for SHA224\n");
					adi_reset_state(pkte_dev);
					return -ENODEV;
			}
			break;
		case SHA256_DIGEST_SIZE:
			dev_dbg(pkte_dev->dev, "adi_init, selected SHA256 hashing\n");
			pkte_dev->pkte_device->pPkteList.pCommand.hash = hash_sha256;
			pkte_dev->pkte_device->pPkteList.pCommand.digest_length = digest_length8;
			break;
		default:
			dev_dbg(pkte_dev->dev, "adi_init, unknown hashing request\n");
			return -EINVAL;
	}

	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->offset = 0;
	rctx->buflen = PKTE_BUFLEN;

	pkte_dev->ring_pos_produce = 0;
	pkte_dev->ring_pos_consume = 0;
	processing = 0;

	return 0;
}

static int adi_update_req(struct adi_dev *pkte_dev)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(pkte_dev->req);
	int bufcnt, err = 0, final;

	dev_dbg(pkte_dev->dev, "%s flags %lx\n", __func__, pkte_dev->flags);

	final = (pkte_dev->flags & PKTE_FLAGS_FINUP);

	if (rctx->total > rctx->buflen) {
		while (rctx->total > rctx->buflen) {
			adi_append_sg(rctx, pkte_dev);
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(pkte_dev, bufcnt, 0);
		}
	} else {
		if ((rctx->total >= (rctx->buflen)) ||
			   (rctx->bufcnt + rctx->total >= (rctx->buflen))) {
			adi_append_sg(rctx, pkte_dev);
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(pkte_dev, bufcnt, 0);
		}
	}

	adi_append_sg(rctx, pkte_dev);

	if (final) {
		bufcnt = rctx->bufcnt;
		rctx->bufcnt = 0;
		err = adi_process_packet(pkte_dev, bufcnt, 1);
	}

	return err;
}

static int adi_final_req(struct adi_dev *pkte_dev)
{
	struct ahash_request *req = pkte_dev->req;
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	int err;
	int buflen;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	buflen = rctx->bufcnt;
	rctx->bufcnt = 0;

	err = adi_process_packet(pkte_dev, buflen, 1);

	return err;
}

static void adi_copy_hash(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;
	u32 *hash = (u32 *)rctx->digest;
	u32 i;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	rctx->digcnt = rctx->pkte_dev->pkte_device->pPkteList.pCommand.digest_length*4;

	//Endian Swap!
	if (pkte_dev->pkte_device->pPkteList.pCommand.hash == hash_md5) {
		memcpy(hash, rctx->pkte_dev->pkte_device->pPkteList.pDestination, rctx->digcnt);
	} else {
		for (i = 0; i < rctx->digcnt/4; i++) {
			hash[i] = __builtin_bswap32(rctx->pkte_dev->pkte_device->pPkteList.pDestination[i]);
		}
	}

}

static int adi_finish(struct ahash_request *req)
{

	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	adi_reset_state(pkte_dev);

	if (!req->result) {
		dev_dbg(pkte_dev->dev, "%s: error\n", __func__);
		return -EINVAL;
	}

	memcpy(req->result, rctx->digest, rctx->digcnt);

	return 0;
}

static void adi_finish_req(struct ahash_request *req, int err)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	if (!err && (PKTE_FLAGS_FINAL & pkte_dev->flags)) {
		wait_event_interruptible(wq_ready, ready == 1);
		while (!(adi_read(pkte_dev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY));

		//if(pkte_dev->flags & PKTE_HOST_MODE)
			adi_read_packet(pkte_dev, &pkte_dev->pkte_device->pPkteList.pDestination[0]);

		adi_copy_hash(req);
		err = adi_finish(req);
	}

	crypto_finalize_hash_request(pkte_dev->engine, req, err);
}

static int adi_one_request(struct crypto_engine *engine, void *areq);
static int adi_prepare_req(struct crypto_engine *engine, void *areq);

static int adi_handle_queue(struct adi_dev *pkte_dev,
				   struct ahash_request *req)
{
	return crypto_transfer_hash_request_to_engine(pkte_dev->engine, req);
}

static int adi_prepare_req(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	if (!pkte_dev)
		return -ENODEV;

	pkte_dev->req = req;

	rctx = ahash_request_ctx(req);

	dev_dbg(pkte_dev->dev, "processing new req, op: %lu, nbytes %d\n",
		rctx->op, req->nbytes);

	if (unlikely(pkte_dev->flags & PKTE_FLAGS_HMAC)) {
		if (req->nbytes > PKTE_BUFLEN) {
			dev_err(pkte_dev->dev, "HMAC computation unsupported on > %d bytes\n", PKTE_BUFLEN);
			adi_reset_state(pkte_dev);
			return -ENODEV;
		}
	}

	return adi_hw_init(pkte_dev);
}

static int adi_one_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;
	int err = 0;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	if (!pkte_dev)
		return -ENODEV;

	pkte_dev->req = req;

	rctx = ahash_request_ctx(req);

	switch (rctx->op) {
		case PKTE_OP_UPDATE:
			err = adi_update_req(pkte_dev);
			break;
		case PKTE_OP_FINAL:
			err = adi_final_req(pkte_dev);
			break;
	}

	if (err != -EINPROGRESS)
		adi_finish_req(req, err);

	return 0;

}

static int adi_enqueue(struct ahash_request *req, unsigned int op)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	rctx->op = op;

	return adi_handle_queue(pkte_dev, req);
}

static int adi_update(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	if (unlikely(pkte_dev->flags & PKTE_FLAGS_HMAC)) {
		if(req->nbytes > 257){
			dev_err(pkte_dev->dev, "HMAC operations cannot currently be chained.  Cannot process > 257 bytes\n");
			return -ENODEV;
		}
		if (pkte_dev->flags & PKTE_HOST_MODE) {
			dev_err(pkte_dev->dev, "HMAC computation not yet supported while in Direct Host Mode\n");
			return -ENODEV;
		}
		if (!(pkte_dev->flags & PKTE_FLAGS_HMAC_KEY_PREPARED))
			adi_prepare_secret_key(pkte_dev, rctx);
	}

	if (!req->nbytes)
		return 0;

	rctx->total = req->nbytes;
	rctx->sg = req->src;
	rctx->offset = 0;

	if ((rctx->bufcnt + rctx->total <= rctx->buflen)) {
		adi_append_sg(rctx, pkte_dev);
		return 0;
	}

	return adi_enqueue(req, PKTE_OP_UPDATE);
}

static int adi_final(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	pkte_dev->flags |= PKTE_FLAGS_FINUP;

	return adi_enqueue(req, PKTE_OP_FINAL);
}

static int adi_finup(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_digest(struct ahash_request *req)
{
	return adi_init(req) ?: adi_finup(req);
}

static int adi_export(struct ahash_request *req, void *out)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_import(struct ahash_request *req, const void *in)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_cra_init_algs(struct crypto_tfm *tfm,
					const char *algs_hmac_name)
{
	struct adi_ctx *ctx = crypto_tfm_ctx(tfm);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	dev_dbg(pkte_dev->dev, "%s\n", __func__);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct adi_request_ctx));

	pkte_dev->secret_keylen = 0;
	if (algs_hmac_name) {
		pkte_dev->flags |= PKTE_FLAGS_HMAC;
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
				.cra_priority = 1,
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
				.cra_priority = 1,
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
				.cra_priority = 1,
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
				.cra_priority = 1,
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

ADI_ALGS_INFO adi_algs_info_adi[NUM_HASH_CATEGORIES] = {
	{
		.algs_list	= algs_md5_sha1,
		.size		= ARRAY_SIZE(algs_md5_sha1),
	},
	{
		.algs_list	= algs_sha224_sha256,
		.size		= ARRAY_SIZE(algs_sha224_sha256),
	},
};
