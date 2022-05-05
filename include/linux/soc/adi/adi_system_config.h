// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef SOC_ADI_ADI_SYSTEM_CONFIG_H
#define SOC_ADI_ADI_SYSTEM_CONFIG_H

#include <linux/soc/adi/system_config.h>

/*
 * All possible system register IDs across all platforms supported by this driver
 */
enum adi_system_reg_id {
	/* EMAC0 PTP Clock 0 source select */
	ADI_SYSTEM_REG_EMAC0_PTPCLK0 = 0,
	/* EMAC0 EMACRESET line */
	ADI_SYSTEM_REG_EMAC0_EMACRESET,
	/* EMAC0 PHY interface select */
	ADI_SYSTEM_REG_EMAC0_PHYISEL,
	/* EMAC0 PTP_AUXIN input enable */
	ADI_SYSTEM_REG_EMAC0_AUXIE,
	/* Endianness for EMAC0 DMA */
	ADI_SYSTEM_REG_EMAC0_ENDIANNESS,
	/* Endianness for EMAC1 DMA */
	ADI_SYSTEM_REG_EMAC1_ENDIANNESS,

	__ADI_SYSTEM_REG_COUNT
};

#endif
