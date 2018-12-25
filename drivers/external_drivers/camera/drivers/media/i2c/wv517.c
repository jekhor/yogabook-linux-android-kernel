/*
 * Support for wv517 vcm driver.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include "wv517.h"

static struct wv517_device wv517_dev;

#if 0
static int wv517_i2c_rd8(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2] = { reg };

	msg[0].addr = WV517_VCM_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = WV517_VCM_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	*val = 0;

	if (i2c_transfer(client->adapter, msg, 2) != 2){
		return -EIO;
		}
	*val = buf[1];

	return 0;
}

static int wv517_i2c_wr8(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2] = { reg, val};

	msg.addr = WV517_VCM_ADDR;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}
#endif
#if 1
static int wv517_i2c_wr16(struct i2c_client *client, u8 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3] = { reg, (u8)(val >> 8), (u8)(val & 0xff)};

	msg.addr = WV517_VCM_ADDR;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}
#endif
int wv517_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	value = clamp(value, 0, WV517_MAX_FOCUS_POS);
        //printk("yangsy write data:%x to %x \n", value, WV517_DATA_M);
	
	ret = wv517_i2c_wr16(client,WV517_DATA_M, value);
	if (ret < 0)
		return ret;
	getnstimeofday(&wv517_dev.focus_time);
	wv517_dev.focus = value;

	return 0;
}

int wv517_vcm_power_up(struct v4l2_subdev *sd)
{
#if 1
	//vcm's power up along with camera sensor, so in the routine ,we set default setting.
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	//set 12.6ms  0x43 0x44 0x211
	ret = wv517_i2c_wr16(client,WV517_DSC_M, 0x211);
	if (ret < 0)
		return ret;
	return 0;
#endif
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 value;
	int i;
	int step = WV517_CLICK_REDUCTION_STEP;

	/* Enable power */
	ret = wv517_dev.platform_data->power_ctrl(sd, 1);
	if (ret)
		return ret;
	ret = wv517_i2c_wr8(client, WV517_PD, 0);
	if (ret < 0)
		goto fail_powerdown;
	/* Wait t_OPR for VBAT to stabilize */
	usleep_range(100, 110);

	/* Detect device */
	ret = wv517_i2c_rd8(client, WV517_SACT, &value);
	if (ret < 0)
		goto fail_powerdown;

	/*
	 * WORKAROUND: for module P8V12F-203 which are used on
	 * Cherrytrail Refresh Davis Reef AoB, register SACT is not
	 * returning default value as spec. But VCM works as expected and
	 * root cause is still under discussion with vendor.
	 * workaround here to avoid aborting the power up sequence and just
	 * give a warning about this error.
	 */
	if (value != WV517_SACT_DEFAULT_VAL)
		dev_warn(&client->dev, "%s error, incorrect ID\n", __func__);

	/* Initialize according to recommended settings */
	ret = wv517_i2c_wr8(client, WV517_CONTROL,
			     WV517_CONTROL_SW_LINEAR |
			     WV517_CONTROL_S_SAC4 |
			     WV517_CONTROL_OCP_DISABLE |
			     WV517_CONTROL_UVLO_DISABLE);
	if (ret < 0)
		goto fail_powerdown;
	ret = wv517_i2c_wr8(client, WV517_SACT,
			     WV517_SACT_MULT_TWO |
			     WV517_SACT_PERIOD_8_8MS);
	if (ret < 0)
		goto fail_powerdown;

	/* Wait t_MODE after changing from switching to linear mode */
	usleep_range(85, 95);

	/* Minimize the click sounds from the lens during power up */
	i = WV517_LENS_MOVE_POSITION;
	while (i <= wv517_dev.focus) {
		ret = wv517_i2c_wr16(client, WV517_DATA_M, i);
		if (ret) {
			dev_err(&client->dev, "%s: write failed\n", __func__);
			break;
		}
		msleep(WV517_CLICK_REDUCTION_SLEEP);
		i += step;
	}

	ret = wv517_t_focus_abs(sd, wv517_dev.focus);
	if (ret)
		return ret;

	wv517_dev.initialized = true;

	return 0;

fail_powerdown:
	dev_err(&client->dev, "%s error, powerup failed\n", __func__);
	wv517_dev.platform_data->power_ctrl(sd, 0);
	return ret;
#else
	return 0;
#endif
}

int wv517_vcm_power_down(struct v4l2_subdev *sd)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int i;
	int step = WV517_CLICK_REDUCTION_STEP;

	/* Minimize the click sounds from the lens during power down */
	i = min(wv517_dev.focus, WV517_DEFAULT_FOCUS_POSITION) - step;
	while (i >= WV517_LENS_MOVE_POSITION) {
		ret = wv517_i2c_wr16(client, WV517_DATA_M, i);
		if (ret) {
			dev_err(&client->dev, "%s: write failed\n", __func__);
			break;
		}
		msleep(WV517_CLICK_REDUCTION_SLEEP);
		i -= step;
	}

	ret = wv517_i2c_wr8(client, WV517_PD, 1);
	if (ret)
		dev_err(&client->dev, "%s: write WV517_PD to 1 failed\n",
				__func__);

	return wv517_dev.platform_data->power_ctrl(sd, 0);
#else
	return 0;
#endif
}

int wv517_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	static const struct timespec move_time = {
		.tv_sec = 0,
		.tv_nsec = 60000000
	};
	struct timespec current_time, finish_time, delta_time;

	getnstimeofday(&current_time);
	finish_time = timespec_add(wv517_dev.focus_time, move_time);
	delta_time = timespec_sub(current_time, finish_time);
	if (delta_time.tv_sec >= 0 && delta_time.tv_nsec >= 0) {
		*value = ATOMISP_FOCUS_HP_COMPLETE |
			 ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
	} else {
		*value = ATOMISP_FOCUS_STATUS_MOVING |
			 ATOMISP_FOCUS_HP_IN_PROGRESS;
	}

	return 0;
}

int wv517_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	return -EINVAL;
}

int wv517_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	return wv517_t_focus_abs(sd, wv517_dev.focus + value);
}

int wv517_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	*value  = wv517_dev.focus;
	return 0;
}
int wv517_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	return 0;
}

int wv517_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	return 0;
}

int wv517_vcm_init(struct v4l2_subdev *sd)
{
	wv517_dev.platform_data = camera_get_af_platform_data();
	wv517_dev.focus = WV517_DEFAULT_FOCUS_POSITION;
	return (NULL == wv517_dev.platform_data) ? -ENODEV : 0;
}
