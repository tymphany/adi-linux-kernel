// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * @todo next step is to unify the arm/mach-sc5xx headers under linux/soc/adi
 * for now we forward to them based on platform
 */

#ifndef SOC_ADI_ICC_H
#define SOC_ADI_ICC_H

#include <linux/device.h>
#include <linux/types.h>

#ifdef CONFIG_ARCH_SC59X_64
#include <linux/soc/adi/mach-sc59x/icc.h>
#else
#include <mach/icc.h>
#endif

#define ADI_RESOURCE_TABLE_TAG "AD-RESOURCE-TBL"
#define ADI_RESOURCE_TABLE_INIT_MAGIC (0xADE0AD0E)
#define ADI_RESOURCE_TABLE_VERSION (1)
struct adi_resource_table_hdr {
	u8 tag[16];
	u32 version;
	u32 initialized;
	u32 reserved[8];
}__packed;

struct adi_tru;

struct adi_tru *get_adi_tru_from_node(struct device *dev);
void put_adi_tru(struct adi_tru *tru);
int adi_tru_trigger_device(struct adi_tru *tru, struct device *dev);
int adi_tru_trigger(struct adi_tru *tru, u32 master);

#endif
