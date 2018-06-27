/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/gfp.h>
#include <linux/bitops.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy.h>
#include <linux/sched_clock.h>

#include <asm/irq.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/cpu.h>
#include <mach/dma.h>
#include <mach/sc58x.h>
#include <mach/irqs.h>
#include <mach/clkdev.h>
#include <mach/sec.h>

#include "core.h"

#define TIMER_CLOCKSOURCE 1
#define TIMER_CLOCKEVENT  0

static struct sc58x_gptimer *timer_clock, *timer_event;

void __init sc58x_init_irq(void)
{
	gic_init(0, 32,
		__io_address(SC58X_GIC_PORT0),
		__io_address(SC58X_GIC_PORT1));
}

static struct map_desc sc58x_io_desc[] __initdata __maybe_unused = {
	{
		.virtual	=  IO_ADDRESS(SYS_MMR_BASE),
		.pfn		= __phys_to_pfn(SYS_MMR_BASE),
		.length		= SYS_MMR_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	=  IO_ADDRESS(SYS_L2_START),
		.pfn		= __phys_to_pfn(SYS_L2_START),
		.length		= SZ_16K,
		.type		= MT_MEMORY_RWX_NONCACHED,
	}, {
		.virtual	=  IO_ADDRESS(SYS_SRAM_BASE),
		.pfn		= __phys_to_pfn(SYS_SRAM_BASE),
		.length		= SYS_SRAM_SIZE,
		.type		= MT_MEMORY_RWX,
	},
};

void __init sc58x_map_io(void)
{
	iotable_init(sc58x_io_desc, ARRAY_SIZE(sc58x_io_desc));
}

void sc58x_restart(enum reboot_mode mode, const char *cmd)
{
	writel(1, __io_address(REG_RCU0_CTL));
}


#include <asm/siginfo.h>
#include <asm/signal.h>


static bool first_fault = true;

static int sc58x_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	if (fsr == 0x1c06 && first_fault) {
		first_fault = false;

		/*
		 * These faults with code 0x1c06 happens for no good reason,
		 * possibly left over from the CFE boot loader.
		 */
		pr_warn("External imprecise Data abort at addr=%#lx, fsr=%#x ignored.\n",
				addr, fsr);

		/* Returning non-zero causes fault display and panic */
		return 0;
	}

	/* Others should cause a fault */
	return 1;
}

/* Early initializations */
void __init sc58x_init_early(void)
{
	/* Install our hook */
	hook_fault_code(16 + 6, sc58x_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
	sc58x_clock_init();
}

#ifdef CONFIG_OF
static const struct of_dev_auxdata sc58x_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("adi,adi2-pinctrl", 0, "pinctrl-adi2.0", NULL),
	OF_DEV_AUXDATA("arm,adi-uart4", UART0_REVID, "adi-uart4.0", NULL),
	OF_DEV_AUXDATA("arm,adi-watchdog", REG_WDOG0_CTL, "adi-watchdog.0", NULL),
	{},
};

static struct of_device_id sc58x_of_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{},
};
#endif

#define DP83865_PHY_ID          0x20005c7a
#define REG_DP83865_AUX_CTRL    0x12
#define BITP_AUX_CTRL_RGMII_EN  12
#define RGMII_3COM_MODE         3
static int sc58x_dp83865_fixup(struct phy_device *phydev)
{
	int  phy_data = 0;

	phy_data = phy_read(phydev, REG_DP83865_AUX_CTRL);

	/* enable 3com mode for RGMII */
	phy_write(phydev, REG_DP83865_AUX_CTRL,
			     (RGMII_3COM_MODE << BITP_AUX_CTRL_RGMII_EN) | phy_data);

	return 0;
}

#define DP83848_PHY_ID          0x20005c90
#define REG_DP83848_PHY_MICR    0x11
#define BITM_PHY_MICR_INTEN     0x2
#define BITM_PHY_MICR_INT_OE    0x1
static int sc58x_dp83848_fixup(struct phy_device *phydev)
{
	phy_write(phydev, REG_DP83848_PHY_MICR,
				BITM_PHY_MICR_INTEN | BITM_PHY_MICR_INT_OE);

	return 0;
}

static void sc58x_init_ethernet(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* select RGMII as the external PHY interface for EMAC0 */
		writel((readl(__io_address(REG_PADS0_PCFG0)) |
		        BITM_PADS_PCFG0_EMACPHYISEL | BITM_PADS_PCFG0_EMACRESET),
		        __io_address(REG_PADS0_PCFG0));
		/* register fixup to be run for PHYs */
		phy_register_fixup_for_uid(DP83865_PHY_ID, 0xffffffff,
				sc58x_dp83865_fixup);
		phy_register_fixup_for_uid(DP83848_PHY_ID, 0xffffffff,
				sc58x_dp83848_fixup);
	}
}

void __init sc58x_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	l2x0_of_init(0, ~0UL);
#endif

	pr_info("%s: registering device resources\n", __func__);

	sec_init(__io_address(SEC_COMMON_BASE), __io_address(SEC_SCI_BASE),
			__io_address(SEC_SSI_BASE));
#ifdef CONFIG_OF
	of_platform_populate(NULL, sc58x_of_bus_ids,
				sc58x_auxdata_lookup, NULL);
#endif
	sc58x_init_ethernet();
}

static void __iomem *spu_base;

void set_spu_securep_msec(uint16_t n, bool msec)
{
	void __iomem *p = (void __iomem *)(spu_base + 0xA00 + 4 * n);
	u32 securep = ioread32(p);

	if (msec)
		iowrite32(securep | 0x3, p);
	else
		iowrite32(securep & ~0x3, p);
}
EXPORT_SYMBOL(set_spu_securep_msec);

static int __init spu_init(void)
{
	spu_base = ioremap(REG_SPU0_CTL, 0x1000);

	return 0;
}
arch_initcall(spu_init);

void __init setup_gptimer(struct sc58x_gptimer *timer)
{
	int id = timer->id;

	disable_gptimers(1 << id);
	set_gptimer_config(timer, TIMER_OUT_DIS
			| TIMER_MODE_PWM_CONT | TIMER_PULSE_HI | TIMER_IRQ_PER);
	set_gptimer_period(timer, 0xFFFFFFFF);
	set_gptimer_pwidth(timer, 0xFFFFFFFE);

	enable_gptimers(1 << id);
}

static u64 read_gptimer(struct clocksource *cs)
{
	return (u64) get_gptimer_count(timer_clock);
}

static struct clocksource cs_gptimer = {
	.name           = "cs_gptimer",
	.rating         = 350,
	.read           = read_gptimer,
	.mask           = CLOCKSOURCE_MASK(32),
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init cs_gptimer_init(void)
{
	setup_gptimer(timer_clock);

	if (clocksource_register_hz(&cs_gptimer, get_sclk()))
		panic("failed to register clocksource");

	return 0;
}

static int gptmr_set_next_event(unsigned long cycles,
		struct clock_event_device *evt)
{
	int id = timer_event->id;

	disable_gptimers(1 << id);

	/* it starts counting three SCLK cycles after the TIMENx bit is set */
	set_gptimer_pwidth(timer_event, cycles - 3);
	enable_gptimers(1 << id);
	return 0;
}

static int gptmr_set_state_periodic(struct clock_event_device *evt)
{
	int id = timer_event->id;

	disable_gptimers(1 << id);
	set_gptimer_config(timer_event, TIMER_OUT_DIS
				| TIMER_MODE_PWM_CONT | TIMER_PULSE_HI |
				TIMER_IRQ_PER);

	set_gptimer_period(timer_event, get_sclk() / HZ);
	set_gptimer_pwidth(timer_event, get_sclk() / HZ - 1);
	enable_gptimers(1 << id);
	return 0;
}

static int gptmr_set_state_oneshot(struct clock_event_device *evt)
{
	int id = timer_event->id;

	while(1);
	disable_gptimers(1 << id);
	set_gptimer_config(timer_event, TIMER_OUT_DIS | TIMER_MODE_PWM
			| TIMER_PULSE_HI | TIMER_IRQ_WID_DLY);
	set_gptimer_period(timer_event, 0);
	return 0;
}

static int gptmr_set_state_shutdown(struct clock_event_device *evt)
{
	int id = timer_event->id;

	disable_gptimers(1 << id);
	return 0;
}

static void gptmr_ack(int id)
{
	set_gptimer_status(1 << id);
}

static u64 notrace gptmr_read_sched(void)
{
	return (u32)get_gptimer_count(timer_clock);
}

irqreturn_t gptmr_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	/*
	 * We want to ACK before we handle so that we can handle smaller timer
	 * intervals.  This way if the timer expires again while we're handling
	 * things, we're more likely to see that 2nd int rather than swallowing
	 * it by ACKing the int at the end of this handler.
	 */
	gptmr_ack(TIMER_CLOCKEVENT);
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction gptmr_irq = {
	.name           = "SC58x GPTimer0",
	.flags          = IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = gptmr_interrupt,
};

static struct clock_event_device clockevent_gptmr = {
	.name           = "sc58x_gptimer0",
	.rating         = 300,
	.shift          = 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	.set_next_event = gptmr_set_next_event,
	.set_state_periodic = gptmr_set_state_periodic,
	.set_state_oneshot = gptmr_set_state_oneshot,
	.set_state_shutdown = gptmr_set_state_shutdown,
};


static void __init gptmr_clockevent_init(struct clock_event_device *evt)
{
	unsigned long clock_tick;

	clock_tick = get_sclk();
	evt->mult = div_sc(clock_tick, NSEC_PER_SEC, evt->shift);
	evt->max_delta_ns = clockevent_delta2ns(-1, evt);
	evt->min_delta_ns = clockevent_delta2ns(100, evt);

	evt->cpumask = cpumask_of(0);

	clockevents_register_device(evt);
}

static struct sc58x_gptimer *sc58x_timer_of_init(struct device_node *node)
{
	void __iomem *base;
	int irq;
	int id;
	struct sc58x_gptimer *timer = NULL;

	id = of_alias_get_id(node, "timer");
	if (id < 0)
		panic("Can't timer id");

	base = of_iomap(node, 0);
	if (!base)
		panic("Can't remap registers");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	timer = kzalloc(sizeof(struct sc58x_gptimer), GFP_KERNEL);
	if (!timer) {
		pr_err("%s: no memory.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	timer->id = id;
	timer->io_base = base;
	timer->irq = irq;

	return timer;
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init sc58x_timer_init(void)
{
	struct device_node *np, *clocksrc_np = NULL, *clockevent_np = NULL;

	for_each_compatible_node(np, NULL, "adi,sc58x-timer-core") {
		if (!clocksrc_np &&
			(of_alias_get_id(np, "timer") == TIMER_CLOCKSOURCE)) {
			clocksrc_np = np;
		}

		if (!clockevent_np &&
			(of_alias_get_id(np, "timer") == TIMER_CLOCKEVENT)) {
			clockevent_np = np;
		}

	}

	timer_clock = sc58x_timer_of_init(clocksrc_np);
	timer_event = sc58x_timer_of_init(clockevent_np);

	clockevent_gptmr.irq = timer_event->irq;

	cs_gptimer_init();

	sched_clock_register(gptmr_read_sched, 32, get_sclk());

	setup_irq(timer_event->irq, &gptmr_irq);
	gptmr_irq.dev_id = &clockevent_gptmr;
	gptmr_clockevent_init(&clockevent_gptmr);

}
