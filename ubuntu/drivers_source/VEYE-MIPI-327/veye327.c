// SPDX-License-Identifier: GPL-2.0
/*
 * veye327 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif
//test
#define VEYE327_LINK_FREQ_297MHZ		297000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define VEYE327_PIXEL_RATE		(VEYE327_LINK_FREQ_297MHZ * 2 * 2 / 8)
#define VEYE327_XVCLK_FREQ		24000000

#define CHIP_ID				0x06
#define VEYE327_REG_CHIP_ID		0x0001
//start streaming and stop streaming
#define VEYE327_REG_CTRL_MODE		0x000B
#define VEYE327_MODE_SW_STANDBY		0x0
#define VEYE327_MODE_STREAMING		0x1

#define VEYE327_VTS_MAX			0x7fff

/*
#define VEYE327_REG_EXPOSURE		0x3500
#define	VEYE327_EXPOSURE_MIN		4
#define	VEYE327_EXPOSURE_STEP		1


#define VEYE327_REG_GAIN_H		0x3508
#define VEYE327_REG_GAIN_L		0x3509
#define VEYE327_GAIN_H_MASK		0x07
#define VEYE327_GAIN_H_SHIFT		8
#define VEYE327_GAIN_L_MASK		0xff
#define VEYE327_GAIN_MIN			0x10
#define VEYE327_GAIN_MAX			0xf8
#define VEYE327_GAIN_STEP		1
#define VEYE327_GAIN_DEFAULT		0x10

#define VEYE327_REG_TEST_PATTERN		0x5040
#define VEYE327_TEST_PATTERN_ENABLE	0x80
#define VEYE327_TEST_PATTERN_DISABLE	0x0

#define VEYE327_REG_VTS			0x380e
*/
#define REG_NULL			0xFFFF

#define VEYE327_REG_VALUE_08BIT		1
#define VEYE327_REG_VALUE_16BIT		2
#define VEYE327_REG_VALUE_24BIT		3

#define VEYE327_LANES			2
#define VEYE327_BITS_PER_SAMPLE		16

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define VEYE327_NAME			"veye327"

//all this do not needed!
static const char * const veye327_supply_names[] = {
	//"avdd",		/* Analog power */
	//"dovdd",	/* Digital I/O power */
	//"dvdd",		/* Digital core power */
};

#define VEYE327_NUM_SUPPLIES ARRAY_SIZE(veye327_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct veye327_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	//u32 exp_def;
	const struct regval *reg_list;
};

struct veye327 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[VEYE327_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct veye327_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_veye327(sd) container_of(sd, struct veye327, subdev)

/*
 *
 */
static const struct regval veye327_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * max_framerate 30fps
 * mipi_datarate per lane 594Mbps
 */
static const struct regval veye327_1920x1080_regs[] = {
	{REG_NULL, 0x00},
};

static const struct veye327_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
        //do not need this,will check it!
		//.exp_def = 0x0600,
		.hts_def = 2200,
		.vts_def = 1125,
		.reg_list = veye327_1920x1080_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	VEYE327_LINK_FREQ_297MHZ
};

static const char * const veye327_test_pattern_menu[] = {
	"Disabled",
};

/* Write registers up to 4 at a time */
static int veye327_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int veye327_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = veye327_write_reg(client, regs[i].addr,
				       VEYE327_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int veye327_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int veye327_get_reso_dist(const struct veye327_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct veye327_mode *
veye327_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = veye327_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int veye327_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct veye327 *veye327 = to_veye327(sd);
	const struct veye327_mode *mode;
	s64 h_blank, vblank_def;
    struct device *dev = &veye327->client->dev;
	mutex_lock(&veye327->mutex);

	mode = veye327_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
    dev_info(dev, "veye327_set_fmt width %d height %d\n", fmt->format.width,fmt->format.height);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&veye327->mutex);
		return -ENOTTY;
#endif
	} else {
		veye327->cur_mode = mode;
        //useless haha
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(veye327->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(veye327->vblank, vblank_def,
					 VEYE327_VTS_MAX - mode->height,
					 1, vblank_def);
                     
	}

	mutex_unlock(&veye327->mutex);

	return 0;
}

static int veye327_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct veye327 *veye327 = to_veye327(sd);
	const struct veye327_mode *mode = veye327->cur_mode;
   // struct device *dev = &veye327->client->dev;
	mutex_lock(&veye327->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&veye327->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		fmt->format.field = V4L2_FIELD_NONE;
	}
   // dev_info(dev, "veye327_get_fmt width %d height %d\n", fmt->format.width,fmt->format.height);
	mutex_unlock(&veye327->mutex);

	return 0;
}

static int veye327_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_UYVY8_2X8;

	return 0;
}

static int veye327_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}
#if 0
static int veye327_enable_test_pattern(struct veye327 *veye327, u32 pattern)
{
	//u32 val;
    return 0;
	/*if (pattern)
		val = (pattern - 1) | VEYE327_TEST_PATTERN_ENABLE;
	else
		val = VEYE327_TEST_PATTERN_DISABLE;

	return veye327_write_reg(veye327->client, VEYE327_REG_TEST_PATTERN,
				VEYE327_REG_VALUE_08BIT, val);*/
}
#endif

static int veye327_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct veye327 *veye327 = to_veye327(sd);
	const struct veye327_mode *mode = veye327->cur_mode;
   // struct device *dev = &veye327->client->dev;
    
	mutex_lock(&veye327->mutex);
	fi->interval.numerator = 10000;
	fi->interval.denominator = mode->max_fps * 10000;
	mutex_unlock(&veye327->mutex);
    
   // dev_info(dev, "veye327_g_frame_interval  %d\n",mode->max_fps);
	return 0;
}

static void veye327_get_module_inf(struct veye327 *veye327,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, VEYE327_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, veye327->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, veye327->len_name, sizeof(inf->base.lens));
}

static long veye327_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct veye327 *veye327 = to_veye327(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		if (veye327->module_name)
			veye327_get_module_inf(veye327, (struct rkmodule_inf *)arg);
		else
			ret = -ENOIOCTLCMD;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long veye327_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = veye327_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = veye327_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __veye327_start_stream(struct veye327 *veye327)
{
	int ret;

	ret = veye327_write_array(veye327->client, veye327->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&veye327->mutex);
	ret = v4l2_ctrl_handler_setup(&veye327->ctrl_handler);
	mutex_lock(&veye327->mutex);
	if (ret)
		return ret;

	return veye327_write_reg(veye327->client, VEYE327_REG_CTRL_MODE,
				VEYE327_REG_VALUE_08BIT, VEYE327_MODE_STREAMING);
}

static int __veye327_stop_stream(struct veye327 *veye327)
{
	return veye327_write_reg(veye327->client, VEYE327_REG_CTRL_MODE,
				VEYE327_REG_VALUE_08BIT, VEYE327_MODE_SW_STANDBY);
}

static int veye327_s_stream(struct v4l2_subdev *sd, int on)
{
	struct veye327 *veye327 = to_veye327(sd);
	struct i2c_client *client = veye327->client;
	int ret = 0;
    struct device *dev = &veye327->client->dev;
	mutex_lock(&veye327->mutex);
	on = !!on;
	if (on == veye327->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __veye327_start_stream(veye327);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
        //wait for stream
        msleep(40);
	} else {
		__veye327_stop_stream(veye327);
		pm_runtime_put(&client->dev);
	}

    dev_info(dev, "veye327_s_stream \n");
	veye327->streaming = on;

unlock_and_return:
	mutex_unlock(&veye327->mutex);

	return ret;
}

static int veye327_s_power(struct v4l2_subdev *sd, int on)
{
	struct veye327 *veye327 = to_veye327(sd);
	struct i2c_client *client = veye327->client;
	int ret = 0;

	mutex_lock(&veye327->mutex);

	/* If the power state is not modified - no work to do. */
	if (veye327->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = veye327_write_array(veye327->client, veye327_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		veye327->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		veye327->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&veye327->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 veye327_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, VEYE327_XVCLK_FREQ / 1000 / 1000);
}

static int __veye327_power_on(struct veye327 *veye327)
{
	int ret;
	u32 delay_us;
	struct device *dev = &veye327->client->dev;

	if (!IS_ERR_OR_NULL(veye327->pins_default)) {
		ret = pinctrl_select_state(veye327->pinctrl,
					   veye327->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_prepare_enable(veye327->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(veye327->reset_gpio))
		gpiod_set_value_cansleep(veye327->reset_gpio, 0);

	ret = regulator_bulk_enable(VEYE327_NUM_SUPPLIES, veye327->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(veye327->reset_gpio))
		gpiod_set_value_cansleep(veye327->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(veye327->pwdn_gpio))
		gpiod_set_value_cansleep(veye327->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = veye327_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(veye327->xvclk);

	return ret;
}

static void __veye327_power_off(struct veye327 *veye327)
{
	int ret;
	struct device *dev = &veye327->client->dev;

	if (!IS_ERR(veye327->pwdn_gpio))
		gpiod_set_value_cansleep(veye327->pwdn_gpio, 0);
	clk_disable_unprepare(veye327->xvclk);
	if (!IS_ERR(veye327->reset_gpio))
		gpiod_set_value_cansleep(veye327->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(veye327->pins_sleep)) {
		ret = pinctrl_select_state(veye327->pinctrl,
					   veye327->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(VEYE327_NUM_SUPPLIES, veye327->supplies);
}

static int veye327_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);

	return __veye327_power_on(veye327);
}

static int veye327_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);

	__veye327_power_off(veye327);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int veye327_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct veye327 *veye327 = to_veye327(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct veye327_mode *def_mode = &supported_modes[0];

	mutex_lock(&veye327->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&veye327->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops veye327_pm_ops = {
	SET_RUNTIME_PM_OPS(veye327_runtime_suspend,
			   veye327_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops veye327_internal_ops = {
	.open = veye327_open,
};
#endif

static const struct v4l2_subdev_core_ops veye327_core_ops = {
	.s_power = veye327_s_power,
	.ioctl = veye327_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = veye327_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops veye327_video_ops = {
	.s_stream = veye327_s_stream,
	.g_frame_interval = veye327_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops veye327_pad_ops = {
	.enum_mbus_code = veye327_enum_mbus_code,
	.enum_frame_size = veye327_enum_frame_sizes,
	.get_fmt = veye327_get_fmt,
	.set_fmt = veye327_set_fmt,
};

static const struct v4l2_subdev_ops veye327_subdev_ops = {
	.core	= &veye327_core_ops,
	.video	= &veye327_video_ops,
	.pad	= &veye327_pad_ops,
};

static int veye327_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct veye327 *veye327 = container_of(ctrl->handler,
					     struct veye327, ctrl_handler);
	struct i2c_client *client = veye327->client;
	//s64 max;
	int ret = 0;
#if 0
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = veye327->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(veye327->exposure,
					 veye327->exposure->minimum, max,
					 veye327->exposure->step,
					 veye327->exposure->default_value);
		break;
	}
#endif
	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
    //not supported yet
	#if 0
    case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = veye327_write_reg(veye327->client, VEYE327_REG_EXPOSURE,
				       VEYE327_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = veye327_write_reg(veye327->client, VEYE327_REG_GAIN_H,
				       VEYE327_REG_VALUE_08BIT,
				       (ctrl->val >> VEYE327_GAIN_H_SHIFT) & VEYE327_GAIN_H_MASK);
		ret |= veye327_write_reg(veye327->client, VEYE327_REG_GAIN_L,
				       VEYE327_REG_VALUE_08BIT,
				       ctrl->val & VEYE327_GAIN_L_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = veye327_write_reg(veye327->client, VEYE327_REG_VTS,
				       VEYE327_REG_VALUE_16BIT,
				       ctrl->val + veye327->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = veye327_enable_test_pattern(veye327, ctrl->val);
		break;
    #endif
    case V4L2_CID_VBLANK:
        ret = 0;
        break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops veye327_ctrl_ops = {
	.s_ctrl = veye327_set_ctrl,
};

static int veye327_initialize_controls(struct veye327 *veye327)
{
	const struct veye327_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64  vblank_def;//exposure_max,
	u32 h_blank;
	int ret;

	handler = &veye327->ctrl_handler;
	mode = veye327->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 5);//8-->5
	if (ret)
		return ret;
	handler->lock = &veye327->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, VEYE327_PIXEL_RATE, 1, VEYE327_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	veye327->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (veye327->hblank)
		veye327->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	veye327->vblank = v4l2_ctrl_new_std(handler, &veye327_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				VEYE327_VTS_MAX - mode->height,
				1, vblank_def);
    //xumm add this to only
    //if (veye327->vblank)
	//	veye327->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
#if 0
	exposure_max = mode->vts_def - 4;
	veye327->exposure = v4l2_ctrl_new_std(handler, &veye327_ctrl_ops,
				V4L2_CID_EXPOSURE, VEYE327_EXPOSURE_MIN,
				exposure_max, VEYE327_EXPOSURE_STEP,
				mode->exp_def);

	veye327->anal_gain = v4l2_ctrl_new_std(handler, &veye327_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, VEYE327_GAIN_MIN,
				VEYE327_GAIN_MAX, VEYE327_GAIN_STEP,
				VEYE327_GAIN_DEFAULT);

	veye327->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&veye327_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(veye327_test_pattern_menu) - 1,
				0, 0, veye327_test_pattern_menu);
#endif
	if (handler->error) {
		ret = handler->error;
		dev_err(&veye327->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	veye327->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int veye327_check_sensor_id(struct veye327 *veye327,
				  struct i2c_client *client)
{
	struct device *dev = &veye327->client->dev;
	u32 id = 0;
	int ret;

	ret = veye327_read_reg(client, VEYE327_REG_CHIP_ID,
			      VEYE327_REG_VALUE_08BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%02x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected VEYE%02x sensor\n", CHIP_ID);

	return 0;
}

static int veye327_configure_regulators(struct veye327 *veye327)
{
	//unsigned int i;
    //just return
    return 0;
    /*
	for (i = 0; i < VEYE327_NUM_SUPPLIES; i++)
		veye327->supplies[i].supply = veye327_supply_names[i];

	return devm_regulator_bulk_get(&veye327->client->dev,
				       VEYE327_NUM_SUPPLIES,
				       veye327->supplies);
    */
}

static int veye327_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct veye327 *veye327;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	veye327 = devm_kzalloc(dev, sizeof(*veye327), GFP_KERNEL);
	if (!veye327)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &veye327->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &veye327->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &veye327->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &veye327->len_name);
	if (ret) {
		dev_warn(dev, "could not get module information!\n");
	}

	veye327->client = client;
	veye327->cur_mode = &supported_modes[0];
    //we do not need ext clk
/*
	veye327->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(veye327->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	ret = clk_set_rate(veye327->xvclk, VEYE327_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(veye327->xvclk) != VEYE327_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
*/
	veye327->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(veye327->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	veye327->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(veye327->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	veye327->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(veye327->pinctrl)) {
		veye327->pins_default =
			pinctrl_lookup_state(veye327->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(veye327->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		veye327->pins_sleep =
			pinctrl_lookup_state(veye327->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(veye327->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = veye327_configure_regulators(veye327);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&veye327->mutex);

	sd = &veye327->subdev;
	v4l2_i2c_subdev_init(sd, client, &veye327_subdev_ops);
	ret = veye327_initialize_controls(veye327);
	if (ret)
		goto err_destroy_mutex;

	ret = __veye327_power_on(veye327);
	if (ret)
		goto err_free_handler;

	ret = veye327_check_sensor_id(veye327, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &veye327_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	veye327->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &veye327->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	if (veye327->module_facing && veye327->module_name) {
		memset(facing, 0, sizeof(facing));
		if (strcmp(veye327->module_facing, "back") == 0)
			facing[0] = 'b';
		else
			facing[0] = 'f';

		snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
			 veye327->module_index, facing,
			 VEYE327_NAME, dev_name(sd->dev));
	}

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__veye327_power_off(veye327);
err_free_handler:
	v4l2_ctrl_handler_free(&veye327->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&veye327->mutex);

	return ret;
}

static int veye327_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&veye327->ctrl_handler);
	mutex_destroy(&veye327->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__veye327_power_off(veye327);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id veye327_of_match[] = {
	{ .compatible = "veye,veye327" },
	{},
};
MODULE_DEVICE_TABLE(of, veye327_of_match);
#endif

static const struct i2c_device_id veye327_match_id[] = {
	{ "veye,veye327", 0 },
	{ },
};

static struct i2c_driver veye327_i2c_driver = {
	.driver = {
		.name = VEYE327_NAME,
		.pm = &veye327_pm_ops,
		.of_match_table = of_match_ptr(veye327_of_match),
	},
	.probe		= &veye327_probe,
	.remove		= &veye327_remove,
	.id_table	= veye327_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&veye327_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&veye327_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("VEYE veye327 sensor driver");
MODULE_LICENSE("GPL v2");
