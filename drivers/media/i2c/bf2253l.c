// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for the BF2253L
 * Copyright (C) 2026 Kerigan Creighton
 *
 * Based on gc1066.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define BF2253L_REG_CHIP_ID		CCI_REG16(0xfc)
#define BF2253L_CHIP_ID			0x2253

#define BF2253L_XCLK_FREQ		(24 * HZ_PER_MHZ)

#define BF2253L_LINK_FREQ		330000000ULL
#define BF2253L_PIXEL_RATE		66000000ULL

#define BF2253L_WIDTH			1600U
#define BF2253L_HEIGHT			1200U

#define BF2253L_FRAME_WIDTH		1780U
#define BF2253L_FRAME_HEIGHT		1236U

#define BF2253L_HBLANK			(BF2253L_FRAME_WIDTH  - BF2253L_WIDTH)
#define BF2253L_VBLANK			(BF2253L_FRAME_HEIGHT - BF2253L_HEIGHT)

#define BF2253L_REG_EXPOSURE_H		CCI_REG8(0x6b)
#define BF2253L_REG_EXPOSURE_L		CCI_REG8(0x6c)

#define BF2253L_REG_GAIN		CCI_REG8(0x6a)
#define BF2253L_GAIN_MIN		15
#define BF2253L_GAIN_MAX		79
#define BF2253L_GAIN_DEFAULT		47

#define BF2253L_REG_STREAM		CCI_REG8(0xe0)

static const struct cci_reg_sequence bf2253l_init_regs[] = {
	{CCI_REG8(0x00), 0x22},	/* mirror and invert pixels */
	{CCI_REG8(0xe1), 0x06},
	{CCI_REG8(0xe2), 0x06},
	{CCI_REG8(0xe3), 0x0e},
	{CCI_REG8(0xe4), 0x60},
	{CCI_REG8(0xe5), 0x67},
	{CCI_REG8(0xe6), 0x02},
	{CCI_REG8(0xe8), 0x94},
	{CCI_REG8(0x01), 0x14},
	{CCI_REG8(0x03), 0x98},
	{CCI_REG8(0x27), 0x21},
	{CCI_REG8(0x29), 0x20},
	{CCI_REG8(0x59), 0x10},
	{CCI_REG8(0x5a), 0x10},
	{CCI_REG8(0x5c), 0x11},
	{CCI_REG8(0x5d), 0x73},
	{CCI_REG8(0x6a), 0x2f},	/* gain = 47 (default, ~3.9×) */
	{CCI_REG8(0x6b), 0x0e},	/* exposure high byte */
	{CCI_REG8(0x6c), 0x7e},	/* exposure low byte */
	{CCI_REG8(0x6f), 0x10},
	{CCI_REG8(0x70), 0x08},
	{CCI_REG8(0x71), 0x05},
	{CCI_REG8(0x72), 0x10},
	{CCI_REG8(0x73), 0x08},
	{CCI_REG8(0x74), 0x05},
	{CCI_REG8(0x75), 0x06},
	{CCI_REG8(0x76), 0x20},
	{CCI_REG8(0x77), 0x03},
	{CCI_REG8(0x78), 0x0e},
	{CCI_REG8(0x79), 0x08},
	{CCI_REG8(0x00), 0x22},	/* mirror and invert pixels (re-assert) */
};

static const struct cci_reg_sequence bf2253l_start_regs[] = {
	{CCI_REG8(0xe0), 0x00},	/* stream on */
	{CCI_REG8(0x00), 0x22},	/* mirror and invert */
	{CCI_REG8(0x01), 0x14},	/* timing mode */
};

static const struct cci_reg_sequence bf2253l_stop_regs[] = {
	{CCI_REG8(0xe0), 0x01},	/* stream off */
	{CCI_REG8(0x01), 0x1c},	/* low-power timing mode */
};

static const char * const bf2253l_supply_names[] = {
	"vddio",	/* DOVDD — L4 1.8 V  */
	"dvdd",		/* DVDD  — external reg 1.5 V */
	"avdd",		/* AVDD  — L17 2.85 V */
};

#define BF2253L_NUM_SUPPLIES ARRAY_SIZE(bf2253l_supply_names)

struct bf2253l_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
};

struct bf2253l {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[BF2253L_NUM_SUPPLIES];

	struct bf2253l_ctrls ctrls;
};

static inline struct bf2253l *to_bf2253l(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct bf2253l, sd);
}

/*
 * power_setting[]:
 *  1. PWDN  → HIGH (standby) (already done)
 *  2. RESET → LOW  (in reset) (already done)
 *  3. DOVDD on (VIO)
 *  4. DVDD  on (VDIG)
 *  5. MCLK  on
 *  6. AVDD  on (VANA)
 *  7. PWDN  → LOW  (exit standby)
 *  8. RESET → HIGH (exit reset)
 */
static int bf2253l_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bf2253l *sensor = to_bf2253l(sd);
	int ret;

	ret = regulator_enable(sensor->supplies[0].consumer);
	if (ret) {
		dev_err(dev, "failed to enable vddio: %d\n", ret);
		return ret;
	}
	usleep_range(5000, 6000);

	ret = regulator_enable(sensor->supplies[1].consumer);
	if (ret) {
		dev_err(dev, "failed to enable dvdd: %d\n", ret);
		goto err_vddio;
	}
	usleep_range(5000, 6000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(dev, "failed to enable xclk: %d\n", ret);
		goto err_dvdd;
	}
	usleep_range(5000, 6000);

	ret = regulator_enable(sensor->supplies[2].consumer);
	if (ret) {
		dev_err(dev, "failed to enable avdd: %d\n", ret);
		goto err_clk;
	}
	usleep_range(10000, 11000);

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 0);
	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	msleep(50);

	return 0;

err_clk:
	clk_disable_unprepare(sensor->xclk);
err_dvdd:
	regulator_disable(sensor->supplies[1].consumer);
err_vddio:
	regulator_disable(sensor->supplies[0].consumer);
	return ret;
}

static int bf2253l_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bf2253l *sensor = to_bf2253l(sd);

	clk_disable_unprepare(sensor->xclk);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);	/* assert reset */
	gpiod_set_value_cansleep(sensor->powerdown_gpio, 1);	/* enter standby */

	regulator_bulk_disable(BF2253L_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static int bf2253l_identify_module(struct bf2253l *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	u64 chip_id;
	int ret;

	ret = cci_read(sensor->regmap, BF2253L_REG_CHIP_ID, &chip_id, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (chip_id != BF2253L_CHIP_ID) {
		dev_err(&client->dev,
			"chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			BF2253L_CHIP_ID, chip_id);
		return -ENXIO;
	}

	return 0;
}

static int bf2253l_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bf2253l *sensor = container_of(ctrl->handler,
					      struct bf2253l, ctrls.handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, BF2253L_REG_EXPOSURE_H,
			  (ctrl->val >> 8) & 0xff, &ret);
		cci_write(sensor->regmap, BF2253L_REG_EXPOSURE_L,
			  ctrl->val & 0xff, &ret);
		break;
	case V4L2_CID_GAIN:
		cci_write(sensor->regmap, BF2253L_REG_GAIN, ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops bf2253l_ctrl_ops = {
	.s_ctrl = bf2253l_s_ctrl,
};

static int bf2253l_init_controls(struct bf2253l *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct bf2253l_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	static const s64 link_freq_menu[] = { BF2253L_LINK_FREQ };
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 6);
	if (ret)
		return ret;

	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, &bf2253l_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      BF2253L_PIXEL_RATE,
					      BF2253L_PIXEL_RATE, 1,
					      BF2253L_PIXEL_RATE);
	if (ctrls->pixel_rate)
		ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, &bf2253l_ctrl_ops,
						  V4L2_CID_LINK_FREQ,
						  ARRAY_SIZE(link_freq_menu) - 1,
						  0, link_freq_menu);
	if (ctrls->link_freq)
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->hblank = v4l2_ctrl_new_std(hdl, &bf2253l_ctrl_ops,
					  V4L2_CID_HBLANK,
					  BF2253L_HBLANK, BF2253L_HBLANK, 1,
					  BF2253L_HBLANK);
	if (ctrls->hblank)
		ctrls->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->vblank = v4l2_ctrl_new_std(hdl, &bf2253l_ctrl_ops,
					  V4L2_CID_VBLANK,
					  BF2253L_VBLANK, BF2253L_VBLANK, 1,
					  BF2253L_VBLANK);
	if (ctrls->vblank)
		ctrls->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->exposure = v4l2_ctrl_new_std(hdl, &bf2253l_ctrl_ops,
					    V4L2_CID_EXPOSURE,
					    1, BF2253L_HEIGHT - 1, 1, 600);

	ctrls->gain = v4l2_ctrl_new_std(hdl, &bf2253l_ctrl_ops,
					V4L2_CID_GAIN,
					BF2253L_GAIN_MIN, BF2253L_GAIN_MAX, 1,
					BF2253L_GAIN_DEFAULT);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(&client->dev, "control init failed: %d\n", ret);
		goto error;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

error:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int bf2253l_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (on) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			return ret;
	} else {
		pm_runtime_put(&client->dev);
	}

	return ret;
}

static const struct v4l2_subdev_core_ops bf2253l_core_ops = {
	.s_power = bf2253l_s_power,
};

static int bf2253l_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, 0);
	format->width = BF2253L_WIDTH;
	format->height = BF2253L_HEIGHT;
	format->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int bf2253l_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int bf2253l_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = BF2253L_WIDTH;
	fse->max_width = BF2253L_WIDTH;
	fse->min_height = BF2253L_HEIGHT;
	fse->max_height = BF2253L_HEIGHT;

	return 0;
}

static int bf2253l_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct bf2253l *sensor = to_bf2253l(sd);
	struct v4l2_mbus_framefmt *format;

	fmt->format.width = BF2253L_WIDTH;
	fmt->format.height = BF2253L_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	format = v4l2_subdev_state_get_format(sd_state, fmt->pad);
	*format = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
					 BF2253L_PIXEL_RATE);
		__v4l2_ctrl_s_ctrl(sensor->ctrls.hblank, BF2253L_HBLANK);
		__v4l2_ctrl_s_ctrl(sensor->ctrls.vblank, BF2253L_VBLANK);
	}

	return 0;
}

static int bf2253l_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = BF2253L_WIDTH;
		sel->r.height = BF2253L_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops bf2253l_pad_ops = {
	.enum_mbus_code = bf2253l_enum_mbus_code,
	.enum_frame_size = bf2253l_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = bf2253l_set_pad_format,
	.get_selection = bf2253l_get_selection,
};

static int bf2253l_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct bf2253l *sensor = to_bf2253l(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0) {
			dev_err(&client->dev,
				"s_stream: pm_runtime_resume_and_get failed: %d\n", ret);
			return ret;
		}

		ret = cci_multi_reg_write(sensor->regmap, bf2253l_init_regs,
					  ARRAY_SIZE(bf2253l_init_regs), NULL);
		if (ret) {
			dev_err(&client->dev,
				"failed to write init regs: %d\n", ret);
			goto err_rpm_put;
		}

		msleep(50);

		ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
		if (ret) {
			dev_err(&client->dev,
				"failed to apply controls: %d\n", ret);
			goto err_rpm_put;
		}

		ret = cci_multi_reg_write(sensor->regmap, bf2253l_start_regs,
					  ARRAY_SIZE(bf2253l_start_regs), NULL);
		if (ret) {
			dev_err(&client->dev,
				"failed to start streaming: %d\n", ret);
			goto err_rpm_put;
		}

		msleep(10);
	} else {
		ret = cci_multi_reg_write(sensor->regmap, bf2253l_stop_regs,
					  ARRAY_SIZE(bf2253l_stop_regs), NULL);
		if (ret)
			dev_err(&client->dev,
				"failed to stop streaming: %d\n", ret);

		pm_runtime_put_autosuspend(&client->dev);
	}

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(&client->dev);
	return ret;
}

static const struct v4l2_subdev_video_ops bf2253l_video_ops = {
	.s_stream = bf2253l_s_stream,
};

static const struct v4l2_subdev_ops bf2253l_subdev_ops = {
	.core  = &bf2253l_core_ops,
	.video = &bf2253l_video_ops,
	.pad   = &bf2253l_pad_ops,
};

static const struct v4l2_subdev_internal_ops bf2253l_subdev_internal_ops = {
	.init_state = bf2253l_init_state,
};

static int bf2253l_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 1) {
		dev_err(dev, "only 1 data lane is supported\n");
		ret = -EINVAL;
		goto out;
	}

	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto out;
	}

	if (ep_cfg.link_frequencies[0] != BF2253L_LINK_FREQ) {
		dev_err(dev,
			"invalid link frequency: expected %llu, got %llu\n",
			BF2253L_LINK_FREQ, ep_cfg.link_frequencies[0]);
		ret = -EINVAL;
	}

out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	return ret;
}

static int bf2253l_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct bf2253l *sensor;
	unsigned int xclk_freq;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sensor->sd, client, &bf2253l_subdev_ops);
	sensor->sd.internal_ops = &bf2253l_subdev_internal_ops;

	ret = bf2253l_check_hwcfg(dev);
	if (ret)
		return ret;

	sensor->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "failed to get xclk\n");

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq < 23000000 || xclk_freq > 25000000) {
		dev_err(dev,
			"xclk frequency not supported: %u Hz (expected 24 MHz)\n",
			xclk_freq);
		return -EINVAL;
	}

	for (i = 0; i < BF2253L_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = bf2253l_supply_names[i];

	ret = devm_regulator_bulk_get(dev, BF2253L_NUM_SUPPLIES,
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "failed to get reset gpio\n");

	sensor->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->powerdown_gpio),
				     "failed to get powerdown gpio\n");

	sensor->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to initialize CCI\n");

	ret = bf2253l_power_on(dev);
	if (ret)
		return ret;

	ret = bf2253l_identify_module(sensor);
	if (ret)
		goto error_power_off;

	ret = bf2253l_init_controls(sensor);
	if (ret)
		goto error_power_off;

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	sensor->sd.state_lock = sensor->ctrls.handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "subdev init failed: %d\n", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor subdev: %d\n", ret);
		goto error_subdev_cleanup;
	}

	dev_info(dev, "bf2253l probed successfully\n");
	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&sensor->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
error_media_entity:
	media_entity_cleanup(&sensor->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
error_power_off:
	bf2253l_power_off(dev);
	return ret;
}

static void bf2253l_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2253l *sensor = to_bf2253l(sd);

	v4l2_subdev_cleanup(sd);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		bf2253l_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id bf2253l_dt_ids[] = {
	{ .compatible = "byd,bf2253l" },
	{ }
};
MODULE_DEVICE_TABLE(of, bf2253l_dt_ids);

static const struct dev_pm_ops bf2253l_pm_ops = {
	RUNTIME_PM_OPS(bf2253l_power_off, bf2253l_power_on, NULL)
};

static struct i2c_driver bf2253l_i2c_driver = {
	.driver = {
		.name		= "bf2253l",
		.of_match_table	= bf2253l_dt_ids,
		.pm		= pm_ptr(&bf2253l_pm_ops),
	},
	.probe	= bf2253l_probe,
	.remove	= bf2253l_remove,
};

module_i2c_driver(bf2253l_i2c_driver);

MODULE_AUTHOR("kercre123");
MODULE_DESCRIPTION("BF2253L camera sensor driver");
MODULE_LICENSE("GPL");