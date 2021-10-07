// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thermal monitoring unit driver for ADI SC59x series SoCs
 *
 * Author: Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* Register offsets */
#define SC59X_TMU_CTL				0x00
#define SC59X_TMU_TEMP				0x04
#define SC59X_TMU_AVG				0x08
#define SC59X_TMU_FLT_LIM_HI		0x0c
#define SC59X_TMU_ALRT_LIM_HI		0x10
#define SC59X_TMU_FLT_LIM_LO		0x14
#define SC59X_TMU_ALRT_LIM_LO		0x18
#define SC59X_TMU_STATUS			0x1c
/* no 0x20 register in map */
#define SC59X_TMU_GAIN				0x24
#define SC59X_TMU_IMSK				0x28
#define SC59X_TMU_OFFSET			0x2c
/* no 0x30 register in map */
#define SC59X_TMU_CNV_BLANK			0x34
#define SC59X_TMU_REFR_CNTR			0x38

/* Register bit definitions */
#define SC59X_TMU_CTL_TMEN_FORCE	BIT(13)
#define SC59X_TMU_CTL_SCLKDIV		GENMASK(11, 4)
#define SC59X_TMU_CTL_TMEN			BIT(1)
#define SC59X_TMU_CTL_TMPU			BIT(0)

#define SC59X_TMU_AVG_ENABLE		BIT(0)

#define SC59X_TMU_STAT_FLTHI		BIT(4)
#define SC59X_TMU_STAT_ALRTHI		BIT(5)

#define SC59X_TMU_IMSK_FLTHI		BIT(0)
#define SC59X_TMU_IMSK_ALRTHI		BIT(1)

/* TMU supports an alert trip and a fault trip with programmable limits */
#define SC59X_THERMAL_TRIPS			2
#define SC59X_THERMAL_TRIP_MASK		0x03
#define SC59X_ALERT_TRIP			0
#define SC59X_FAULT_TRIP			1

/* delay in milliseconds when polling for passive cooling */
#define SC59X_PASSIVE_DELAY			1000

/* Minimum value for triggers */
#define SC59X_LIM_HI_MIN			60

struct sc59x_thermal_data {
	void __iomem *ioaddr;
	struct thermal_zone_device *tzdev;
	struct device *dev;
	struct work_struct work;
	int last_temp;
	enum thermal_device_mode mode;
};

static int sc59x_get_mode(struct thermal_zone_device *tzdev,
	enum thermal_device_mode *mode)
{
	struct sc59x_thermal_data *data = tzdev->devdata;
	*mode = data->mode;
	return 0;
}

static int sc59x_set_mode(struct thermal_zone_device *tzdev,
	enum thermal_device_mode mode)
{
	// @todo enable/disable, we're just always enabled for now
	return 0;
}

static int sc59x_get_temp(struct thermal_zone_device *tzdev, int *temp) {
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp;

	/* data is 9.7 fixed point representing temperature in celsius
	 * linux expects integer millicelsius */
	tmu_temp = readl(data->ioaddr + SC59X_TMU_TEMP);
	*temp = (tmu_temp * 1000) >> 7;

	return 0;
}

static int sc59x_get_trip_type(struct thermal_zone_device *tzdev, int trip,
	enum thermal_trip_type *type)
{
	struct sc59x_thermal_data *data = tzdev->devdata;

	switch (trip) {
	case SC59X_ALERT_TRIP:
		*type = THERMAL_TRIP_PASSIVE;
		break;
	case SC59X_FAULT_TRIP:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		dev_err(data->dev, "requested invalid trip type for %d\n", trip);
		return -EINVAL;
	}

	return 0;
}

static int sc59x_get_trip_temp(struct thermal_zone_device *tzdev, int trip, int *temp) {
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp = 0;

	switch (trip) {
	case SC59X_ALERT_TRIP:
		tmu_temp = readl(data->ioaddr + SC59X_TMU_ALRT_LIM_HI);
		break;
	case SC59X_FAULT_TRIP:
		tmu_temp = readl(data->ioaddr + SC59X_TMU_FLT_LIM_HI);
		break;
	default:
		dev_err(data->dev, "requested invalid trip temp for %d\n", trip);
		return -EINVAL;
	}

	/* temp limits are integer celsius */
	*temp = tmu_temp * 1000;

	return 0;
}

static int sc59x_get_crit_temp(struct thermal_zone_device *tzdev, int *temp) {
	return sc59x_get_trip_temp(tzdev, SC59X_FAULT_TRIP, temp);
}

static int sc59x_set_trip_temp(struct thermal_zone_device *tzdev, int trip, int temp) {
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp;

	/* temp limits are integer celsius */
	tmu_temp = temp / 1000;

	if (tmu_temp < SC59X_LIM_HI_MIN) {
		dev_err(data->dev, "trip limits must be at least 60 C, tried to set %d for trip %d\n", tmu_temp, trip);
		return -EINVAL;
	}

	switch (trip) {
	case SC59X_ALERT_TRIP:
		writel(tmu_temp, data->ioaddr + SC59X_TMU_ALRT_LIM_HI);
		break;
	case SC59X_FAULT_TRIP:
		writel(tmu_temp, data->ioaddr + SC59X_TMU_FLT_LIM_HI);
		break;
	default:
		dev_err(data->dev, "tried to set trip temp for invalid trip %d\n", trip);
		return -EINVAL;
	}

	dev_info(data->dev, "set trip %d temperature to %d C\n", trip, tmu_temp);

	return 0;
}

static struct thermal_zone_device_ops sc59x_tmu_ops = {
	.get_temp = sc59x_get_temp,
	.get_mode = sc59x_get_mode,
	.set_mode = sc59x_set_mode,
	.get_trip_type = sc59x_get_trip_type,
	.get_trip_temp = sc59x_get_trip_temp,
	.get_crit_temp = sc59x_get_crit_temp,
	.set_trip_temp = sc59x_set_trip_temp,
};

static void sc59x_work_handler(struct work_struct *work) {
	struct sc59x_thermal_data *data = container_of(work, struct sc59x_thermal_data, work);

	thermal_zone_device_update(data->tzdev, THERMAL_EVENT_UNSPECIFIED);

	/* clearing interrupts may reset temperature register contents */
	writel(SC59X_TMU_STAT_ALRTHI | SC59X_TMU_STAT_FLTHI, data->ioaddr + SC59X_TMU_STATUS);
}

static irqreturn_t sc59x_thermal_irq(int irq, void *irqdata) {
	struct sc59x_thermal_data *data = irqdata;

	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static int sc59x_thermal_probe(struct platform_device *pdev) {
	struct sc59x_thermal_data *data;
	struct device *dev = &pdev->dev;
	int irq;
	int ret;
	u32 gain, offset, fault, alert;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, data);
	data->dev = dev;

	data->ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->ioaddr)) {
		dev_err(dev, "Could not map thermal monitoring unit: %ld\n", PTR_ERR(data->ioaddr));
		return PTR_ERR(data->ioaddr);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Failed to find IRQ: %d\n", irq);
		return irq;
	}

	INIT_WORK(&data->work, sc59x_work_handler);

	ret = devm_request_threaded_irq(dev, irq, sc59x_thermal_irq, NULL, 0, "sc59x_thermal", data);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ thread: %d\n", ret);
		return ret;
	}

	fault = SC59X_LIM_HI_MIN;
	of_property_read_u32(dev->of_node, "adi,trip-fault", &fault);
	if (fault < SC59X_LIM_HI_MIN) {
		dev_info(dev, "Fault high limit clamped to 60 C, %d given\n", fault);
		fault = SC59X_LIM_HI_MIN;
	}
	writel(fault, data->ioaddr + SC59X_TMU_FLT_LIM_HI);

	alert = SC59X_LIM_HI_MIN;
	of_property_read_u32(dev->of_node, "adi,trip-alert", &alert);
	if (alert < SC59X_LIM_HI_MIN) {
		dev_info(dev, "Alert high limit clamped to 60 C, %d given\n", fault);
		alert = SC59X_LIM_HI_MIN;
	}
	writel(alert, data->ioaddr + SC59X_TMU_ALRT_LIM_HI);

	gain = 1;
	of_property_read_u32(dev->of_node, "adi,gain", &gain);
	writel(gain, data->ioaddr + SC59X_TMU_GAIN);

	offset = 0;
	of_property_read_u32(dev->of_node, "adi,offset", &offset);
	writel(offset, data->ioaddr + SC59X_TMU_OFFSET);

	if (of_property_read_bool(dev->of_node, "adi,average")) {
		writel(SC59X_TMU_AVG_ENABLE, data->ioaddr + SC59X_TMU_AVG);
		dev_info(dev, "Averaging enabled\n");
	}

	data->tzdev = thermal_zone_device_register("sc59x_thermal_zone",
		SC59X_THERMAL_TRIPS, SC59X_THERMAL_TRIP_MASK, data,
		&sc59x_tmu_ops, NULL, SC59X_PASSIVE_DELAY, 0);
	if (IS_ERR(data->tzdev)) {
		dev_err(dev, "Failed to register thermal zone device: %ld\n", PTR_ERR(data->tzdev));
		return PTR_ERR(data->tzdev);
	}

	/* Unmask interrupts */
	writel(SC59X_TMU_IMSK_FLTHI | SC59X_TMU_IMSK_ALRTHI, data->ioaddr + SC59X_TMU_IMSK);

	/* Enable TMU in continuous operation */
	writel(SC59X_TMU_CTL_TMEN_FORCE | SC59X_TMU_CTL_TMPU, data->ioaddr + SC59X_TMU_CTL);

	data->mode = THERMAL_DEVICE_ENABLED;
	dev_info(dev, "Fault limit: %d C, Alert limit: %d C\n", fault, alert);
	dev_info(dev, "Gain 0x%x, offset %d\n", gain, offset);
	dev_info(dev, "SC59x Thermal Monitoring Unit active\n");
	return 0;
}

static int sc59x_thermal_remove(struct platform_device *pdev) {
	struct sc59x_thermal_data *data = platform_get_drvdata(pdev);
	thermal_zone_device_unregister(data->tzdev);
	return 0;
}

static const struct of_device_id of_sc59x_tmu_match[] = {
	{ .compatible = "adi,sc59x-thermal" },
	{ }
};

MODULE_DEVICE_TABLE(of, of_sc59x_tmu_match);

static struct platform_driver sc59x_tmu_thermal = {
	.driver = {
		.name = "adi_sc59x_tmu",
		.of_match_table = of_sc59x_tmu_match
	},
	.probe = sc59x_thermal_probe,
	.remove = sc59x_thermal_remove,
};

module_platform_driver(sc59x_tmu_thermal);

MODULE_DESCRIPTION("Thermal driver for ADI SC59X Thermal Monitoring Unit");
MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_LICENSE("GPL v2");
