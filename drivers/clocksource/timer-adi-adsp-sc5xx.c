#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

#include <linux/soc/adi/cpu.h>

#include "timer-of.h"

#define TIMER_CLOCKSOURCE 1
#define TIMER_CLOCKEVENT  0

#define CGU_DIV         0x0C //0x3108D00C

static struct sc5xx_gptimer *timer_clock, *timer_event;

static const struct of_device_id sc5xx_fixed_dt_ids[] = {
	{ .compatible = "fixed-clock", },
	{}
};

static unsigned long get_sclk(void){
	unsigned long sclk_rate;
	struct device_node *np;
	u32 sys_clkin0, df_div, vco_mult, sysclk0_div;

	for_each_matching_node(np, sc5xx_fixed_dt_ids){
		if(strcmp(np->name, "sys-clkin0") == 0){
			of_property_read_u32(np, "clock-frequency", &sys_clkin0);
			break;
		}
	}

	df_div = (ioread32(timer_clock->cgu0_ctl) & 0x1) == 0x1 ? 2 : 1;
	vco_mult = ioread32(timer_clock->cgu0_ctl) >> 8 & 0x7F;
	sysclk0_div = ioread32(timer_clock->cgu0_ctl + CGU_DIV) >> 8 & 0x1F;

	sclk_rate = (sys_clkin0 / df_div) * vco_mult / sysclk0_div;

	return sclk_rate;
}

void __init setup_gptimer(struct sc5xx_gptimer *timer)
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
	.name           = "sc5xx_gptimer0",
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

static struct sc5xx_gptimer *sc5xx_timer_of_init(struct device_node *node)
{
	void __iomem *base, *cgu0_ctl;
	int irq;
	int id;
	struct sc5xx_gptimer *timer = NULL;

	id = of_alias_get_id(node, "timer");
	if (id < 0)
		panic("Can't timer id");

	base = of_iomap(node, 0);
	if (!base)
		panic("Can't remap registers");

	cgu0_ctl = of_iomap(node, 1);
	if (!base)
		panic("Can't remap registers");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	timer = kzalloc(sizeof(struct sc5xx_gptimer), GFP_KERNEL);
	if (!timer) {
		pr_err("%s: no memory.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	timer->id = id;
	timer->io_base = base;
	timer->cgu0_ctl = cgu0_ctl;
	timer->irq = irq;

	return timer;
}

static const struct of_device_id sc5xx_timer_core_dt_ids[] = {
	{ .compatible = "adi,sc5xx-timer-core", },
	{}
};

int __init adsp_sc5xx_timer_core_init(void)
{
	struct device_node *np, *clocksrc_np = NULL, *clockevent_np = NULL;
	int ret;

	for_each_matching_node(np, sc5xx_timer_core_dt_ids){
		if (!clocksrc_np && (of_alias_get_id(np, "timer") == TIMER_CLOCKSOURCE)) {
			clocksrc_np = np;
		}

		if (!clockevent_np && (of_alias_get_id(np, "timer") == TIMER_CLOCKEVENT)) {
			clockevent_np = np;
		}
	}

	timer_clock = sc5xx_timer_of_init(clocksrc_np);
	timer_event = sc5xx_timer_of_init(clockevent_np);

	clockevent_gptmr.irq = timer_event->irq;

	map_gptimers();

	cs_gptimer_init();

	sched_clock_register(gptmr_read_sched, 32, get_sclk());

	setup_irq(timer_event->irq, &gptmr_irq);
	gptmr_irq.dev_id = &clockevent_gptmr;
	gptmr_clockevent_init(&clockevent_gptmr);

	return ret;
}
