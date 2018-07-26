/*
 * ADSP-SC57x Core Control Driver
 *
 * Copyright 2015 - 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/sc57x.h>
#include <mach/icc.h>
#include <asm/io.h>

static void adi_core_start(unsigned int coreid)
{
/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
	if (!(readl(__io_address(REG_RCU0_CRCTL)) & 1 << coreid)) {
		/* disable the system interface */
		writel(readl(__io_address(REG_RCU0_SIDIS)) | 1 << (coreid-1),
			   __io_address(REG_RCU0_SIDIS));
		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while(readl(__io_address(REG_RCU0_SISTAT) &
		 *             1 << (coreid-1)!=1); */
		udelay(50);
		/* put core in reset */
		writel(readl(__io_address(REG_RCU0_CRCTL)) | 1 << coreid,
			   __io_address(REG_RCU0_CRCTL));
		/* reenable the system interface */
		writel(readl(__io_address(REG_RCU0_SIDIS)) & ~(1 << (coreid-1)),
			   __io_address(REG_RCU0_SIDIS));
		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while(readl(__io_address(REG_RCU0_SISTAT) &
		 *             1 << (coreid-1)!=0); */
		udelay(50);
	}
	/* move core out of reset */
	writel(readl(__io_address(REG_RCU0_CRCTL)) & ~(1 << coreid),
			__io_address(REG_RCU0_CRCTL));
	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
	/* notify CCES */
	writel(1 << (18 + coreid), __io_address(REG_RCU0_MSG_SET));
}

static void adi_core_stop(unsigned int coreid)
{
	if (readl(__io_address(REG_RCU0_CRCTL)) & 1 << coreid)
		return;
	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
	/* disable the system interface */
	writel(readl(__io_address(REG_RCU0_SIDIS)) | 1 << (coreid - 1),
			__io_address(REG_RCU0_SIDIS));
	/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
	/* while (readl(REG_RCU0_SISTAT) & 1 << (coreid - 1) != 1); */
	udelay(50);
	/* put core in reset */
	writel(readl(__io_address(REG_RCU0_CRCTL)) | 1 << coreid,
			__io_address(REG_RCU0_CRCTL));
	/* reenable the system interface */
	writel(readl(__io_address(REG_RCU0_SIDIS)) & ~(1 << (coreid - 1)),
			__io_address(REG_RCU0_SIDIS));
	/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
	/* while (readl(REG_RCU0_SISTAT) & 1 << (coreid - 1) != 0); */
	udelay(50);
	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
}

static void adi_set_svect(unsigned int core_id, unsigned int svect)
{
	if (svect && (core_id == 1))
		writel(svect, __io_address(REG_RCU0_SVECT1));
	else if (svect && (core_id == 2))
		writel(svect, __io_address(REG_RCU0_SVECT2));
}

static long core_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case CMD_SET_SVECT1:
		adi_set_svect(1, arg);
		break;
	case CMD_SET_SVECT2:
		adi_set_svect(2, arg);
		break;
	case CMD_CORE_START:
		adi_core_start(arg);
		break;
	case CMD_CORE_STOP:
		adi_core_stop(arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct file_operations core_fops = {
		.owner          = THIS_MODULE,
		.unlocked_ioctl = core_ioctl,
		.llseek         = noop_llseek,
};

static struct miscdevice core_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = "corectrl",
		.fops  = &core_fops,
};

static int __init adi_core_init(void)
{
	return misc_register(&core_dev);
}

static void __exit adi_core_exit(void)
{
    misc_deregister(&core_dev);
}

module_init(adi_core_init);
module_exit(adi_core_exit);

MODULE_DESCRIPTION("SC57x Core Control Support");
MODULE_LICENSE("GPL v2");
