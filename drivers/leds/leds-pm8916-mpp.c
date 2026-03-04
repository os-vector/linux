// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Kerigan Creighton
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

#define PM8916_MPP_REG_MODE_CTL		0x40
#define PM8916_MPP_REG_VIN_CTL		0x41
#define PM8916_MPP_REG_EN_CTL		0x46
#define PM8916_MPP_REG_SINK_CTL		0x4c

#define PM8916_MPP_MODE_SINK		0x60
#define PM8916_MPP_MODE_MASK		0x70

#define PM8916_MPP_EN_ENABLE		0x80
#define PM8916_MPP_EN_MASK		0x80

#define PM8916_MPP_SINK_MASK		0x07
#define PM8916_MPP_CURRENT_MIN		5
#define PM8916_MPP_CURRENT_MAX		40
#define PM8916_MPP_CURRENT_STEP		5

#define PM8916_MPP_VIN_VPH		0x00
#define PM8916_MPP_VIN_MASK		0x03

#define PM8916_MPP_SRC_DTEST1		0x08
#define PM8916_MPP_SRC_MASK		0x0f

struct pm8916_mpp_led {
	struct regmap *map;
	u32 reg;
	u8 dtest;
	struct led_classdev cdev;
};

static int pm8916_mpp_led_brightness_set(struct led_classdev *cled,
					  enum led_brightness brightness)
{
	struct pm8916_mpp_led *led = container_of(cled, struct pm8916_mpp_led, cdev);
	unsigned int current_setting;
	int ret;

	if (brightness == 0) {
		ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_EN_CTL,
					 PM8916_MPP_EN_MASK, 0);
		if (ret)
			return ret;
		return 0;
	}

	current_setting = (brightness * (PM8916_MPP_CURRENT_MAX - PM8916_MPP_CURRENT_MIN)) /
			  cled->max_brightness;
	current_setting = current_setting / PM8916_MPP_CURRENT_STEP;
	if (current_setting > 7)
		current_setting = 7;

	ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_SINK_CTL,
				 PM8916_MPP_SINK_MASK, current_setting);
	if (ret)
		return ret;

	ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_EN_CTL,
				 PM8916_MPP_EN_MASK, PM8916_MPP_EN_ENABLE);

	return ret;
}

static int pm8916_mpp_led_setup(struct pm8916_mpp_led *led)
{
	int ret;

	ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_MODE_CTL,
				 PM8916_MPP_MODE_MASK, PM8916_MPP_MODE_SINK);
	if (ret)
		return ret;

	ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_VIN_CTL,
				 PM8916_MPP_VIN_MASK, PM8916_MPP_VIN_VPH);
	if (ret)
		return ret;

	if (led->dtest) {
		ret = regmap_update_bits(led->map, led->reg + PM8916_MPP_REG_MODE_CTL,
					 PM8916_MPP_SRC_MASK, led->dtest);
		if (ret)
			return ret;
	}

	return 0;
}

static int pm8916_mpp_led_probe(struct platform_device *pdev)
{
	struct led_init_data init_data = {};
	struct device *dev = &pdev->dev;
	struct pm8916_mpp_led *led;
	struct device_node *np = dev_of_node(dev);
	struct regmap *map;
	int ret;
	u32 dtest = 0;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	map = dev_get_regmap(dev->parent, NULL);
	if (!map) {
		dev_err(dev, "Parent regmap unavailable\n");
		return -ENXIO;
	}
	led->map = map;

	ret = of_property_read_u32(np, "reg", &led->reg);
	if (ret) {
		dev_err(dev, "no register offset specified\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "qcom,dtest", &dtest);
	led->dtest = dtest;

	ret = pm8916_mpp_led_setup(led);
	if (ret) {
		dev_err(dev, "Failed to set up MPP LED\n");
		return ret;
	}

	led->cdev.brightness_set_blocking = pm8916_mpp_led_brightness_set;
    
	u32 max_brightness = LED_FULL;
	of_property_read_u32(np, "max-brightness", &max_brightness);
	led->cdev.max_brightness = max_brightness;

	init_data.fwnode = of_fwnode_handle(np);

	led->cdev.brightness = LED_OFF;
	pm8916_mpp_led_brightness_set(&led->cdev, LED_OFF);

	ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
	if (ret) {
		dev_err(dev, "Failed to register LED for %pOF\n", np);
		return ret;
	}

	return 0;
}

static const struct of_device_id pm8916_mpp_led_id_table[] = {
	{ .compatible = "qcom,pm8916-mpp-led" },
	{ .compatible = "qcom,pm8909-mpp-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8916_mpp_led_id_table);

static struct platform_driver pm8916_mpp_led_driver = {
	.probe		= pm8916_mpp_led_probe,
	.driver		= {
		.name	= "pm8916-mpp-led",
		.of_match_table = pm8916_mpp_led_id_table,
	},
};
module_platform_driver(pm8916_mpp_led_driver);

MODULE_DESCRIPTION("PM8916/PM8909 MPP LED driver for current sink mode");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pm8916-mpp-led");