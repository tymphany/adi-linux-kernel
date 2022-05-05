// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PADS-related system config register driver
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adi_system_config.h>

#define ADI_SYSREG_BITS(_id, _offset, _width, _shift) \
	{ \
		.id = ADI_SYSTEM_REG_##_id, \
		.offset = _offset, \
		.mask = GENMASK(_width-1,0) << _shift, \
		.shift = _shift, \
		.is_bits = true, \
	}

static struct system_register adi_pads_regs[] = {
	/* fields in PADS CFG0 at offset +0x04 */
	ADI_SYSREG_BITS(EMAC0_PTPCLK0, 0x04, 2, 0),
	ADI_SYSREG_BITS(EMAC0_EMACRESET, 0x04, 1, 2),
	ADI_SYSREG_BITS(EMAC0_PHYISEL, 0x04, 2, 3),
	ADI_SYSREG_BITS(EMAC0_AUXIE, 0x04, 1, 17),
#ifdef CONFIG_ARCH_SC59X_64
	ADI_SYSREG_BITS(EMAC0_ENDIANNESS, 0x04, 1, 19),
	ADI_SYSREG_BITS(EMAC1_ENDIANNESS, 0x04, 1, 20),
#endif
};

static struct system_config adi_pads_config = {
	.registers = adi_pads_regs,
	.len = ARRAY_SIZE(adi_pads_regs),
	.max_register = __ADI_SYSTEM_REG_COUNT,
};

int adi_pads_probe(struct platform_device *pdev) {
	return system_config_probe(pdev, &adi_pads_config);
}

int adi_pads_remove(struct platform_device *pdev) {
	return system_config_remove(pdev);
}

static const struct of_device_id pads_dt_ids[] = {
	{ .compatible = "adi,pads-system-config", },
	{ }
};
MODULE_DEVICE_TABLE(of, pads_dt_ids);

static struct platform_driver pads_driver = {
	.driver = {
		.name = "adi-pads-system-config",
		.of_match_table = pads_dt_ids,
	},
	.probe = adi_pads_probe,
	.remove = adi_pads_remove,
};
module_platform_driver(pads_driver);

MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_DESCRIPTION("ADI ADSP PADS CFG-based System Configuration Driver");
MODULE_LICENSE("GPL");
