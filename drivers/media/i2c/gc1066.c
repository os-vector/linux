// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Galaxycore GC1066 camera sensor
 * Copyright (C) 2026 Kerigan Creighton
 *
 * Based on gc2145.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/* Chip ID */
#define GC1066_REG_CHIP_ID	CCI_REG16(0xf0)
#define GC1066_CHIP_ID		0x1066

#define GC1066_REG_PAGE_SELECT	CCI_REG8(0xfe)
#define GC1066_XCLK_FREQ	(24 * HZ_PER_MHZ)
#define GC1066_NATIVE_WIDTH	1296U
#define GC1066_NATIVE_HEIGHT	736U

#define GC1066_LINK_FREQ	456000000ULL

#define GC1066_PIXEL_RATE	91200000ULL

#define GC1066_HBLANK		0x09a3  // 2467 - 1280 = 1187
#define GC1066_VBLANK		0x0030  // 768 - 720 = 48

static const struct cci_reg_sequence gc1066_init_regs[] = {
	{CCI_REG8(0xf7), 0x01},
	{CCI_REG8(0xf8), 0x12},
	{CCI_REG8(0xf9), 0x0a},
	{CCI_REG8(0xfc), 0x0e},
	{CCI_REG8(0xfd), 0x00},

	/* analog */
	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x03), 0x02},  /* Exposure[12:8] */
	{CCI_REG8(0x04), 0xb5},  /* Exposure[7:0] */
	{CCI_REG8(0x05), 0x02},
	{CCI_REG8(0x06), 0x48},
	{CCI_REG8(0x07), 0x00},  /* vblank */
	{CCI_REG8(0x08), 0x10},
	{CCI_REG8(0x09), 0x00},  /* row start */
	{CCI_REG8(0x0a), 0x20},
	{CCI_REG8(0x0b), 0x00},  /* col start */
	{CCI_REG8(0x0c), 0x20},
	{CCI_REG8(0x0d), 0x02},  /* height: 736 */
	{CCI_REG8(0x0e), 0xe0},
	{CCI_REG8(0x0f), 0x05},  /* width: 1296 */
	{CCI_REG8(0x10), 0x10},
	{CCI_REG8(0x17), 0xd5},  /* CFA */
	{CCI_REG8(0x18), 0x02},
	{CCI_REG8(0x19), 0x2b},
	{CCI_REG8(0x1b), 0xf1},
	{CCI_REG8(0x1c), 0x2c},
	{CCI_REG8(0x1e), 0x70},
	{CCI_REG8(0x1f), 0xa0},  /* gamma */
	{CCI_REG8(0x20), 0xca},  /* gamma offset */
	{CCI_REG8(0x25), 0xc1},  /* txl_s_mode */
	{CCI_REG8(0x26), 0x0e},
	{CCI_REG8(0x27), 0x20},  /* txl_drv_mode */
	{CCI_REG8(0x29), 0x40},  /* EQP */
	{CCI_REG8(0x2b), 0x88},  /* init_rampb_mode */
	{CCI_REG8(0x2f), 0xf4},
	{CCI_REG8(0x32), 0x1a},
	{CCI_REG8(0x37), 0xff},
	{CCI_REG8(0x38), 0x03},  /* head_acc_en off */
	{CCI_REG8(0xcd), 0xa8},
	{CCI_REG8(0xce), 0xcd},
	{CCI_REG8(0xd1), 0x9d},  /* vref */
	{CCI_REG8(0xd3), 0x33},
	{CCI_REG8(0xd8), 0xa0},
	{CCI_REG8(0xd9), 0xfa},
	{CCI_REG8(0xda), 0xc3},  /* ctdsun */
	{CCI_REG8(0xe1), 0x24},  /* post tx width x4 */
	{CCI_REG8(0xe3), 0x01},
	{CCI_REG8(0xe4), 0xf8},
	{CCI_REG8(0xe6), 0x20},  /* ramps_offset */
	{CCI_REG8(0xe7), 0xca},  /* wen_U */
	{CCI_REG8(0xe8), 0x00},
	{CCI_REG8(0xe9), 0x00},
	{CCI_REG8(0xea), 0x00},
	{CCI_REG8(0xeb), 0x00},

	/* ISP related */
	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x80), 0x50},
	{CCI_REG8(0x88), 0x03},
	{CCI_REG8(0x89), 0x03},
	{CCI_REG8(0x8a), 0xb3},
	{CCI_REG8(0x8d), 0x03},

	/* crop */
	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x90), 0x01},  /* crop enable */
	{CCI_REG8(0x91), 0x00},  /* crop y start */
	{CCI_REG8(0x92), 0x02},
	{CCI_REG8(0x93), 0x00},  /* crop x start */
	{CCI_REG8(0x94), 0x01},
	{CCI_REG8(0x95), 0x02},  /* crop height: 720 */
	{CCI_REG8(0x96), 0xd0},
	{CCI_REG8(0x97), 0x05},  /* crop width: 1280 */
	{CCI_REG8(0x98), 0x00},

	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x40), 0x22},
	{CCI_REG8(0x4e), 0x3c},
	{CCI_REG8(0x4f), 0x00},
	{CCI_REG8(0x60), 0x00},
	{CCI_REG8(0x61), 0x80},

	/* gain */
	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0xb0), 0x80},  /* global gain: 2x (U2.6: 0x40=1x, 0x80=2x) */
	{CCI_REG8(0xb1), 0x01},
	{CCI_REG8(0xb2), 0x00},
	{CCI_REG8(0xb3), 0x80},  /* AWB R gain */
	{CCI_REG8(0xb4), 0x40},  /* AWB G gain */
	{CCI_REG8(0xb5), 0x80},  /* AWB B gain */
	{CCI_REG8(0xb6), 0x00},

	{GC1066_REG_PAGE_SELECT, 0x02},
	{CCI_REG8(0x80), 0x08},
	{CCI_REG8(0x83), 0x60},
	{CCI_REG8(0x84), 0x80},
	{CCI_REG8(0x85), 0x60},
	{CCI_REG8(0x86), 0x30},
	{CCI_REG8(0x87), 0xa0},
	{CCI_REG8(0x88), 0x80},
	{CCI_REG8(0x89), 0x40},

	{GC1066_REG_PAGE_SELECT, 0x02},
	{CCI_REG8(0xa0), 0x48},
	{CCI_REG8(0xa3), 0x45},

	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x68), 0xc7},
	{CCI_REG8(0x6c), 0x83},
	{CCI_REG8(0x6e), 0xc4},

	{GC1066_REG_PAGE_SELECT, 0x00},
	{CCI_REG8(0x3c), 0x00},
	{CCI_REG8(0x3d), 0x00},
	{CCI_REG8(0x3f), 0x00},

	// mipi
	{GC1066_REG_PAGE_SELECT, 0x03},
	{CCI_REG8(0x01), 0x03},  /* Enable PHY lanes */
	{CCI_REG8(0x02), 0x33},  /* PHY config */
	{CCI_REG8(0x03), 0x93},  /* PHY config with delays */
	{CCI_REG8(0x04), 0x04},  /* FIFO level MSB */
	{CCI_REG8(0x05), 0x00},  /* FIFO level LSB */
	{CCI_REG8(0x06), 0x80},  /* FIFO mode */
	{CCI_REG8(0x10), 0x80},  /* CSI2 mode: disable initially */
	{CCI_REG8(0x11), 0x2b},  /* MIPI data type: RAW10 */
	{CCI_REG8(0x12), 0x40},  /* LWC LSB */
	{CCI_REG8(0x13), 0x06},  /* LWC MSB (1600 = 0x640) */
	{CCI_REG8(0x15), 0x02},  /* DPHY mode */
	{CCI_REG8(0x21), 0x10},  /* T_LPX */
	{CCI_REG8(0x22), 0x03},  /* T_CLK_HS_PREPARE */
	{CCI_REG8(0x23), 0x30},  /* T_CLK_ZERO */
	{CCI_REG8(0x24), 0x02},  /* T_CLK_PRE */
	{CCI_REG8(0x25), 0x15},  /* T_CLK_POST */
	{CCI_REG8(0x26), 0x05},  /* T_CLK_TRAIL */
	{CCI_REG8(0x29), 0x03},  /* T_HS_PREPARE */
	{CCI_REG8(0x2a), 0x0a},  /* T_HS_ZERO */
	{CCI_REG8(0x2b), 0x05},  /* T_HS_TRAIL */
	{GC1066_REG_PAGE_SELECT, 0x00},
};

static const struct cci_reg_sequence gc1066_start_regs[] = {
	{GC1066_REG_PAGE_SELECT, 0x03},
	{CCI_REG8(0x10), 0x90},
	{GC1066_REG_PAGE_SELECT, 0x00},
};

static const struct cci_reg_sequence gc1066_stop_regs[] = {
	{GC1066_REG_PAGE_SELECT, 0x03},
	{CCI_REG8(0x10), 0x80},
	{GC1066_REG_PAGE_SELECT, 0x00},
};

static const char * const gc1066_supply_name[] = {
	"vddio",
	"avdd",
	"dvdd",
};

#define GC1066_NUM_SUPPLIES ARRAY_SIZE(gc1066_supply_name)

struct gc1066_mode {
	unsigned int width;
	unsigned int height;
	unsigned long pixel_rate;
	unsigned int hblank;
	unsigned int vblank;
};

static const struct gc1066_mode gc1066_mode_720p = {
	.width = 1280,
	.height = 720,
	.pixel_rate = GC1066_PIXEL_RATE,
	.hblank = GC1066_HBLANK,
	.vblank = GC1066_VBLANK,
};

struct gc1066_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
};

struct gc1066 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[GC1066_NUM_SUPPLIES];

	struct gc1066_ctrls ctrls;

	const struct gc1066_mode *mode;
};

static inline struct gc1066 *to_gc1066(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct gc1066, sd);
}

static int gc1066_s_power(struct v4l2_subdev *sd, int on)
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

static const struct v4l2_subdev_core_ops gc1066_core_ops = {
	.s_power = gc1066_s_power,
};

static int gc1066_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, 0);
	format->width = 1280;
	format->height = 720;
	format->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int gc1066_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0) {
		return -EINVAL;
	}

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int gc1066_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = 1280;
	fse->max_width = 1280;
	fse->min_height = 720;
	fse->max_height = 720;

	return 0;
}

static int gc1066_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct gc1066 *gc1066 = to_gc1066(sd);
	struct v4l2_mbus_framefmt *format;

	fmt->format.width = 1280;
	fmt->format.height = 720;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	format = v4l2_subdev_state_get_format(sd_state, fmt->pad);
	*format = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		__v4l2_ctrl_s_ctrl_int64(gc1066->ctrls.pixel_rate,
					 gc1066->mode->pixel_rate);
		__v4l2_ctrl_s_ctrl(gc1066->ctrls.hblank, gc1066->mode->hblank);
		__v4l2_ctrl_s_ctrl(gc1066->ctrls.vblank, gc1066->mode->vblank);
	}

	return 0;
}

static int gc1066_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = 1280;
		sel->r.height = 720;
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = GC1066_NATIVE_WIDTH;
		sel->r.height = GC1066_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops gc1066_pad_ops = {
	.enum_mbus_code = gc1066_enum_mbus_code,
	.enum_frame_size = gc1066_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc1066_set_pad_format,
	.get_selection = gc1066_get_selection,
};

static int gc1066_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc1066 *gc1066 = to_gc1066(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&gc1066->sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0) {
			dev_err(&client->dev, "s_stream: pm_runtime_resume_and_get failed: %d\n", ret);
			return ret;
		}

		ret = cci_multi_reg_write(gc1066->regmap, gc1066_init_regs,
					  ARRAY_SIZE(gc1066_init_regs), NULL);
		if (ret) {
			dev_err(&client->dev, "failed to write init regs: %d\n", ret);
			goto err_rpm_put;
		}

		msleep(50);

		ret = __v4l2_ctrl_handler_setup(&gc1066->ctrls.handler);
		if (ret) {
			dev_err(&client->dev, "failed to apply controls: %d\n", ret);
			goto err_rpm_put;
		}

		ret = cci_multi_reg_write(gc1066->regmap, gc1066_start_regs,
					  ARRAY_SIZE(gc1066_start_regs), NULL);
		if (ret) {
			dev_err(&client->dev, "failed to start streaming: %d\n", ret);
			goto err_rpm_put;
		}

		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x03, NULL);
		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, NULL);

		msleep(10);
	} else {
		ret = cci_multi_reg_write(gc1066->regmap, gc1066_stop_regs,
					  ARRAY_SIZE(gc1066_stop_regs), NULL);
		if (ret)
			dev_err(&client->dev, "failed to stop streaming: %d\n", ret);

		pm_runtime_put_autosuspend(&client->dev);
	}

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(&client->dev);
	return ret;
}

static const struct v4l2_subdev_video_ops gc1066_video_ops = {
	.s_stream = gc1066_s_stream,
};

static const struct v4l2_subdev_ops gc1066_subdev_ops = {
	.core = &gc1066_core_ops,
	.video = &gc1066_video_ops,
	.pad = &gc1066_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc1066_subdev_internal_ops = {
	.init_state = gc1066_init_state,
};

static int gc1066_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc1066 *gc1066 = to_gc1066(sd);
	int ret;

	// downstream has a 1ms delay
	for (int i = 0; i < GC1066_NUM_SUPPLIES; i++) {
		ret = regulator_enable(gc1066->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s: %d\n",
				gc1066_supply_name[i], ret);
			goto reg_off;
		}
		usleep_range(1000, 2000);
	}
	gpiod_set_value_cansleep(gc1066->powerdown_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(gc1066->powerdown_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(gc1066->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(gc1066->reset_gpio, 0);
	usleep_range(10000, 11000);

	ret = clk_prepare_enable(gc1066->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto reg_off;
	}

	msleep(100);

	return 0;

reg_off:
	regulator_bulk_disable(GC1066_NUM_SUPPLIES, gc1066->supplies);
	return ret;
}

static int gc1066_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc1066 *gc1066 = to_gc1066(sd);

	clk_disable_unprepare(gc1066->xclk);

	gpiod_set_value_cansleep(gc1066->reset_gpio, 1);
	gpiod_set_value_cansleep(gc1066->powerdown_gpio, 1);

	regulator_bulk_disable(GC1066_NUM_SUPPLIES, gc1066->supplies);

	return 0;
}

static int gc1066_identify_module(struct gc1066 *gc1066)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc1066->sd);
	int ret;
	u64 chip_id;

	ret = cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, NULL);
	if (ret) {
		dev_err(&client->dev, "identify_module: page select write FAILED: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);

	ret = cci_read(gc1066->regmap, GC1066_REG_CHIP_ID, &chip_id, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (chip_id != GC1066_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: expected 0x%x, got 0x%llx\n",
			GC1066_CHIP_ID, chip_id);
		return -ENXIO;
	}

	return 0;
}

/* V4L2 controls */
static int gc1066_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc1066 *gc1066 = container_of(ctrl->handler, struct gc1066,
					     ctrls.handler);
	struct v4l2_subdev *sd = &gc1066->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(&client->dev)) {
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, &ret);
		cci_write(gc1066->regmap, CCI_REG8(0x03), (ctrl->val >> 8) & 0x1f, &ret);
		cci_write(gc1066->regmap, CCI_REG8(0x04), ctrl->val & 0xff, &ret);
		break;
	case V4L2_CID_GAIN:
		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, &ret);
		ret = cci_write(gc1066->regmap, CCI_REG8(0xb0), ctrl->val, NULL);
		break;
	case V4L2_CID_RED_BALANCE:
		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, &ret);
		ret = cci_write(gc1066->regmap, CCI_REG8(0xb3), ctrl->val, NULL);
		break;
	case V4L2_CID_BLUE_BALANCE:
		cci_write(gc1066->regmap, GC1066_REG_PAGE_SELECT, 0x00, &ret);
		ret = cci_write(gc1066->regmap, CCI_REG8(0xb5), ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc1066_ctrl_ops = {
	.s_ctrl = gc1066_s_ctrl,
};

static int gc1066_init_controls(struct gc1066 *gc1066)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc1066->sd);
	struct gc1066_ctrls *ctrls = &gc1066->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_fwnode_device_properties props;
	static const s64 link_freq_menu[] = { GC1066_LINK_FREQ };
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 8);
	if (ret)
		return ret;

	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      GC1066_PIXEL_RATE,
					      GC1066_PIXEL_RATE, 1,
					      GC1066_PIXEL_RATE);
	if (ctrls->pixel_rate)
		ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, &gc1066_ctrl_ops,
						  V4L2_CID_LINK_FREQ,
						  ARRAY_SIZE(link_freq_menu) - 1,
						  0, link_freq_menu);
	if (ctrls->link_freq)
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->hblank = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					  V4L2_CID_HBLANK,
					  GC1066_HBLANK, GC1066_HBLANK, 1,
					  GC1066_HBLANK);
	if (ctrls->hblank)
		ctrls->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->vblank = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					  V4L2_CID_VBLANK,
					  GC1066_VBLANK, GC1066_VBLANK, 1,
					  GC1066_VBLANK);
	if (ctrls->vblank)
		ctrls->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->exposure = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					    V4L2_CID_EXPOSURE,
					    1, 1477, 1, 693);

	ctrls->gain = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					V4L2_CID_GAIN,
					1, 0xff, 1, 0x80);

	ctrls->red_balance = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
					       V4L2_CID_RED_BALANCE,
					       0x01, 0xff, 1, 0x80);

	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, &gc1066_ctrl_ops,
						V4L2_CID_BLUE_BALANCE,
						0x01, 0xff, 1, 0x80);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(&client->dev, "control init failed: %d\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &gc1066_ctrl_ops, &props);
	if (ret)
		goto error;

	gc1066->sd.ctrl_handler = hdl;

	return 0;

error:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int gc1066_get_regulators(struct gc1066 *gc1066)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc1066->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < GC1066_NUM_SUPPLIES; i++)
		gc1066->supplies[i].supply = gc1066_supply_name[i];

	ret = devm_regulator_bulk_get(&client->dev, GC1066_NUM_SUPPLIES,
				      gc1066->supplies);
	if (ret) {
		dev_err(&client->dev, "failed to get regulators: %d\n", ret);
		return ret;
	}

	return 0;
}

static int gc1066_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	fwnode_handle_put(endpoint);
	if (ret) {
		return ret;
	}

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

	if (ep_cfg.link_frequencies[0] != GC1066_LINK_FREQ) {
		dev_err(dev, "invalid link frequency: expected %lld, got %lld\n",
			GC1066_LINK_FREQ, ep_cfg.link_frequencies[0]);
		ret = -EINVAL;
	}

out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	return ret;
}

static int gc1066_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc1066 *gc1066;
	unsigned int xclk_freq;
	int ret;

	gc1066 = devm_kzalloc(dev, sizeof(*gc1066), GFP_KERNEL);
	if (!gc1066) {
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&gc1066->sd, client, &gc1066_subdev_ops);
	gc1066->sd.internal_ops = &gc1066_subdev_internal_ops;

	ret = gc1066_check_hwcfg(dev);
	if (ret)
		return ret;

	gc1066->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc1066->xclk))
		return dev_err_probe(dev, PTR_ERR(gc1066->xclk),
				     "failed to get xclk\n");

	xclk_freq = clk_get_rate(gc1066->xclk);
	// a little bit of leeway in case of a few MHz off
	if (xclk_freq < 23000000 || xclk_freq > 25000000) {
		dev_err(dev, "xclk frequency not supported: %d Hz (expected 24 MHz)\n",
			xclk_freq);
		return -EINVAL;
	}

	ret = gc1066_get_regulators(gc1066);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to get regulators\n");
	}

	gc1066->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(gc1066->reset_gpio)) {
		return dev_err_probe(dev, PTR_ERR(gc1066->reset_gpio),
				     "failed to get reset gpio\n");
	}

	gc1066->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(gc1066->powerdown_gpio)) {
		return dev_err_probe(dev, PTR_ERR(gc1066->powerdown_gpio),
				     "failed to get powerdown gpio\n");
	}

	gc1066->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc1066->regmap)) {
		return dev_err_probe(dev, PTR_ERR(gc1066->regmap),
				     "failed to initialize CCI\n");
	}

	ret = gc1066_power_on(dev);
	if (ret)
		return ret;

	ret = gc1066_identify_module(gc1066);
	if (ret)
		goto error_power_off;

	gc1066->mode = &gc1066_mode_720p;

	ret = gc1066_init_controls(gc1066);
	if (ret) {
		goto error_power_off;
	}

	gc1066->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc1066->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	gc1066->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc1066->sd.entity, 1, &gc1066->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	gc1066->sd.state_lock = gc1066->ctrls.handler.lock;
	ret = v4l2_subdev_init_finalize(&gc1066->sd);
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

	ret = v4l2_async_register_subdev_sensor(&gc1066->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor subdev: %d\n", ret);
		goto error_subdev_cleanup;
	}

	dev_info(dev, "gc1066 probed successfully\n");

	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&gc1066->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

error_media_entity:
	media_entity_cleanup(&gc1066->sd.entity);

error_handler_free:
	v4l2_ctrl_handler_free(&gc1066->ctrls.handler);

error_power_off:
	gc1066_power_off(dev);

	return ret;
}

static void gc1066_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc1066 *gc1066 = to_gc1066(sd);

	v4l2_subdev_cleanup(sd);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc1066->ctrls.handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc1066_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id gc1066_dt_ids[] = {
	{ .compatible = "galaxycore,gc1066" },
	{},
};

MODULE_DEVICE_TABLE(of, gc1066_dt_ids);

static const struct dev_pm_ops gc1066_pm_ops = {
	RUNTIME_PM_OPS(gc1066_power_off, gc1066_power_on, NULL)
};

static struct i2c_driver gc1066_i2c_driver = {
	.driver = {
		.name = "gc1066",
		.of_match_table = gc1066_dt_ids,
		.pm = pm_ptr(&gc1066_pm_ops),
	},
	.probe = gc1066_probe,
	.remove = gc1066_remove,
};

module_i2c_driver(gc1066_i2c_driver);

MODULE_AUTHOR("kercre123");
MODULE_DESCRIPTION("GalaxyCore GC1066 sensor driver");
MODULE_LICENSE("GPL");