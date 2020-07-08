/*
 * sc58x SEC
 *
 * Copyright 2014 - 2020 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/printk.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <mach/sc58x.h>
#include <mach/hardware.h>
#include <mach/sec.h>

static DEFINE_SPINLOCK(lock);

struct sec_chip_data {
	void __iomem *common_base;
	void __iomem *sci_base;
	void __iomem *ssi_base;
};

static struct sec_chip_data sec_data;
static bool enable_sec = true;

void sec_init(void __iomem *common_base, void __iomem *sci_base,
		 void __iomem *ssi_base)
{
	struct sec_chip_data *sec = &sec_data;

	pr_info("sec init...");
	sec->common_base = common_base;
	sec->sci_base = sci_base;
	sec->ssi_base = ssi_base;

	if (enable_sec) {

	/* Disable SYSCD_RESETb input and clear the RCU0 reset status */
	writel(0x0, __io_address(REG_RCU0_CTL));
	writel(0xf, __io_address(REG_RCU0_STAT));

	/* reset the SEC controller */
	writel(0x2, sec->common_base + SEC_GCTL);
	writel(0x2, sec->common_base + SEC_FCTL);
# if CONFIG_ARCH_SC58X_SLAVECORE_COUNT == 1
	writel(0x2, sec->sci_base + SEC_CCTL);
# endif
# if CONFIG_ARCH_SC58X_SLAVECORE_COUNT == 2
	writel(0x2, sec->sci_base + SEC_CCTL);
	writel(0x2, sec->sci_base + SEC_SCI_OFF + SEC_CCTL);
# endif
	udelay(100);

	/* enable SEC fault event */
	writel(0x1, sec->common_base + SEC_GCTL);

	/* ANOMALY 36100004 Spurious External Fault event occurs when FCTL
	 * is re-programmed when currently active fault is not cleared
	 */
	writel(0xc0, sec->common_base + SEC_FCTL);
	writel(0xc1, sec->common_base + SEC_FCTL);

	/* Enable SYSCD_RESETb input */
	writel(0x100, __io_address(REG_RCU0_CTL));

	pr_info("enabled\n");

	} else {
		pr_info("skipped\n");
	}

#ifdef CONFIG_ADI_WATCHDOG
	/* enable SEC fault source for watchdog0 */
	sec_enable_ssi(3, true, true);
#endif
}

void sec_raise_irq(unsigned int irq)
{
	unsigned long flags;
	unsigned int sid = irq - 32;
	struct sec_chip_data *sec = &sec_data;

	spin_lock_irqsave(&lock, flags);
	writel(sid, sec->common_base + SEC_RAISE);
	spin_unlock_irqrestore(&lock, flags);
}

void sec_enable_ssi(unsigned int sid, bool fault, bool source)
{
	unsigned long flags;
	uint32_t reg_sctl;
	struct sec_chip_data *sec = &sec_data;

	spin_lock_irqsave(&lock, flags);

	reg_sctl = readl(sec->ssi_base + 8 * sid);

	if (fault)
		reg_sctl |= SEC_SCTL_FAULT_EN;
	else
		reg_sctl |= SEC_SCTL_INT_EN;

	if (source)
		reg_sctl |= SEC_SCTL_SRC_EN;

	writel(reg_sctl, sec->ssi_base + 8 * sid);

	spin_unlock_irqrestore(&lock, flags);
}

void sec_enable_sci(unsigned int coreid)
{
	unsigned long flags;
	uint32_t reg_cctl;
	unsigned int id = coreid - 1 ;
	struct sec_chip_data *sec = &sec_data;

	spin_lock_irqsave(&lock, flags);
	reg_cctl = readl(sec->sci_base + 0x40 * id);

	reg_cctl |= SEC_CCTL_EN;
	writel(reg_cctl, sec->sci_base + 0x40 * id);
	spin_unlock_irqrestore(&lock, flags);
}

void sec_set_ssi_coreid(unsigned int sid, unsigned int coreid)
{
	unsigned long flags;
	uint32_t reg_sctl;
	struct sec_chip_data *sec = &sec_data;

	if (coreid > CONFIG_ARCH_SC58X_SLAVECORE_COUNT || coreid == 0)
		return;

	spin_lock_irqsave(&lock, flags);

	reg_sctl = readl(sec->ssi_base + 8 * sid);
	reg_sctl &= ((uint32_t)~SEC_SCTL_CTG);
	reg_sctl |= ((coreid << 24) & SEC_SCTL_CTG),
	writel(reg_sctl, sec->ssi_base + 8 * sid);

	spin_unlock_irqrestore(&lock, flags);
}

static int __init early_sec_init(char *buf)
{
	if (strncmp(buf, "no", 2) == 0)
		enable_sec = false;

	return 0;
}

early_param("enable_sec", early_sec_init);
