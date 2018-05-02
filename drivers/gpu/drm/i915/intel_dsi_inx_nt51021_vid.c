/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "intel_dsi_inx_nt51021_vid.h"
extern unsigned int yeti_hw_ver;

#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
static int gcabc_ce_register;

extern int dsi_vc_send_long(struct intel_dsi *intel_dsi, int channel,
			    u8 data_type, const u8 *data, int len);

static int inx_nt51021_set_gamma(int newgamma,struct intel_dsi *intel_dsi)
{
	return 0;
}

static int inx_nt51021_get_effect_index(char *name)
{
	int i = 0;
	struct lcd_effect_data *effect_data= inx_nt51021_data.lcd_effects;

	for(i = 0; i < effect_data->supported_effects; i++)
		if(!strcmp(name, effect_data->lcd_effects[i].name))
			return i;

	return -1;
}
static int inx_nt51021_set_effect(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0,  ret = 0;
	int effect_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_effect_data *effect_data= inx_nt51021_data.lcd_effects;
	struct lcd_effect effect = effect_data->lcd_effects[effect_index];
	struct lcd_effect_cmd effect_cmd = effect.lcd_effect_cmds[level];
	int cmd_nums = effect_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = 1;

//	if(level < 0 || level > effect.max_level)
//		return -EINVAL;

	//store the effect level
	effect_data->lcd_effects[effect_index].current_level = level;

/*
	if(effect_index == 0) {
		gcabc_ce_register &=  (~NT51021_REGISTER_CABC_MASK);
		gcabc_ce_register  |=  effect_cmd.lcd_cmds[2].cmds[1];
		 effect_cmd.lcd_cmds[2].cmds[1] = gcabc_ce_register;
	} else if(effect_index == 2) {
		gcabc_ce_register &=  (~NT51021_REGISTER_CE_MASK);
		gcabc_ce_register |=  effect_cmd.lcd_cmds[2].cmds[1];
		effect_cmd.lcd_cmds[2].cmds[1] = gcabc_ce_register;
	}
*/
	for(i = 0; i < cmd_nums; i++){
		cmd = effect_cmd.lcd_cmds[i];
		dsi_vc_send_long(dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE, cmd.cmds, cmd.len);
		mdelay(3);
	}

	DRM_INFO("[LCD]: %s line=%d effect_index=%d level=%d,  cabc_ce_reg=%d\n",__func__,__LINE__,effect_index,level, gcabc_ce_register);
	return ret;

}

static int inx_nt51021_get_current_level(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= inx_nt51021_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}

static int inx_nt51021_set_mode(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0, ret = 0;
	int mode_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_mode_data *mode_data= inx_nt51021_data.lcd_modes;
	struct lcd_mode mode = mode_data->lcd_modes[mode_index];
	struct lcd_mode_cmd mode_cmd = mode.lcd_mode_cmds[level];
	int cmd_nums = mode_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = true;

	/*if(level < 0 || level > effect.max_level)*/
		/*return -EINVAL;*/

	for(i = 0; i < cmd_nums; i++){
		cmd = mode_cmd.lcd_cmds[i];
		dsi_vc_dcs_write(dsi, 0, cmd.cmds, cmd.len);
	}

	//store the effect level
	mode_data->lcd_modes[mode_index].mode_status = 1;

	return ret;

}
int inx_nt51021_get_supported_mode(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_mode_data *mode_data= inx_nt51021_data.lcd_modes;
	struct lcd_mode *mode ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_mode *hal_lcd_mode;

	if(index >= mode_data->supported_modes)
		return -EINVAL;

	mode= &mode_data->lcd_modes[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_mode = &hal_panel_data->mode[index];
	/*hal_lcd_mode->mode_status = mode->mode_status;*/

	return 0;
}
static int inx_nt51021_get_supported_effect(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= inx_nt51021_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}
static int inx_nt51021_get_effect_levels(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= inx_nt51021_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}
static struct lcd_panel_dev inx_nt51021__panel_device = {
	.name = "NT51021_INX_1920x1200_8",
	.status = OFF,
	.set_effect = inx_nt51021_set_effect,
	.get_current_level = inx_nt51021_get_current_level,
	.get_effect_index_by_name = inx_nt51021_get_effect_index,
	.set_mode = inx_nt51021_set_mode,
	.get_supported_mode = inx_nt51021_get_supported_mode,
	.get_supported_effect = inx_nt51021_get_supported_effect,
	.get_effect_levels = inx_nt51021_get_effect_levels,
	.set_gamma = inx_nt51021_set_gamma,
};
#endif


void  inx_nt51021_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	DRM_DEBUG_KMS("\n");
	if (pipe == 0) {
		connector->display_info.width_mm = 216;
		connector->display_info.height_mm = 135;
	}

	return;
}

bool inx_nt51021_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */
	DRM_DEBUG_KMS("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 1;
	intel_dsi->clock_stop = 0;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->dual_link = MIPI_DUAL_LINK_NONE;
	intel_dsi->pixel_overlap = 0;
#if 0
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 1;
	intel_dsi->bw_timer = 0x820;
	intel_dsi->clk_lp_to_hs_count = 0xa;
	intel_dsi->clk_hs_to_lp_count = 0x14;
	intel_dsi->video_frmt_cfg_bits = 0;
	//intel_dsi->dphy_reg = 0x3c1fc51f;
	//intel_dsi->port = 0; /* PORT_A by default */
#endif

	intel_dsi->backlight_off_delay =50;
	intel_dsi->backlight_on_delay = 0;
	intel_dsi->panel_on_delay = 0;
	intel_dsi->panel_off_delay = 0;
	intel_dsi->panel_pwr_cycle_delay = 0;

#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	inx_nt51021__panel_device.dsi = intel_dsi;
	lenovo_lcd_panel_register(&inx_nt51021__panel_device);
#endif

	return true;
}



struct drm_display_mode *inx_nt51021_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("\n");
	mode = dev_priv->vbt.lfp_lvds_vbt_mode;

	mode->vrefresh = 60;
	mode->hdisplay = 1200;
	mode->vdisplay = 1920;

	/* Calculate */
	mode->hsync_start = mode->hdisplay + 80;
	mode->hsync_end = mode->hsync_start + 1;
	mode->htotal = mode->hsync_end + 32;

	mode->vsync_start = mode->vdisplay + 35;
	mode->vsync_end = mode->vsync_start + 1;
	mode->vtotal = mode->vsync_end + 25;
	mode->clock = mode->htotal*mode->vtotal*mode->vrefresh/HZ;
	return mode;
}
static void inx_nt51021_panel_reset(struct intel_dsi_device *dsi)
{
	// now use generic panel reset
	return;
}

static void inx_nt51021_disable_panel_power(struct intel_dsi_device *dsi)
{
	return;
}

static void inx_nt51021_send_otp_cmds(struct intel_dsi_device *dsi)
{
	/*for sending panel specified initial sequence*/
	//struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	return;
}

static void inx_nt51021_enable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	//struct hal_panel_ctrl_data  hal_panel_ctrl;

	DRM_DEBUG_KMS("\n");
	intel_dsi->hs=0;

	dsi_vc_dcs_write_1(intel_dsi, 0, 0X01, 0X00);
	msleep(20);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X8C, 0X80);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XC7, 0X50);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XC6, 0X50);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XC5, 0X50);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XC4, 0X50);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X85, 0X04);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X86, 0X08);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0XAA);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X11);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9C, 0X10);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA9, 0X4B);

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);
	msleep(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);
	msleep(50);

#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	inx_nt51021__panel_device.status = ON;
#endif
	
	inx_nt51021_get_current_level(&inx_nt51021__panel_device.hal_panel_ctrl);
	printk("cabc level is %d,index is %d\n",inx_nt51021__panel_device.hal_panel_ctrl.level,inx_nt51021__panel_device.hal_panel_ctrl.index);
	inx_nt51021_set_effect(&inx_nt51021__panel_device.hal_panel_ctrl,intel_dsi);
	

}

static void inx_nt51021_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	intel_dsi->hs=0;
	DRM_INFO("%s\n", __func__);
	//dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);
	//mdelay(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);
	mdelay(100);
#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	inx_nt51021__panel_device.status = OFF;
#endif

}

void inx_nt51021_enable_bklt(struct intel_dsi_device *dsi)
{
	return;
}

void inx_nt51021_disable_bklt(struct intel_dsi_device *dsi)
{
	return;
}

void inx_nt51021_set_brightness(struct intel_dsi_device *dsi,u32 level)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	DRM_DEBUG_KMS("%s, brightness %d\n", __func__, level);
	if(level > 255)
		level = 255;
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9F, level);
	return;
}

void inx_nt51021_power_on(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	int retval = 0;
        unsigned int lcd_pwr_en = 279;//GPIO SE51
        unsigned int lcd_rst = 406;//GPIO N65

	DRM_DEBUG_KMS("\n");

	if(yeti_hw_ver >= 0x3)
		lcd_pwr_en = 401;//GPIO N60
       
	retval = gpio_request(lcd_pwr_en, "enable_lcd_pwr");
        if (retval) {
            pr_err("%s: Failed to get gpio %d (code: %d)", __func__, lcd_pwr_en, retval);
            return;
         }

        retval = gpio_request(lcd_rst, "lcd_rst");
        if (retval) {
            pr_err("%s: Failed to get gpio %d (code: %d)", __func__, lcd_rst, retval);
            gpio_free(lcd_pwr_en);

            return;
         }

        //msleep(20);
	/*lcd_pwr_en to high for lcd 3.3v*/
        gpio_direction_output(lcd_pwr_en, 1);
	msleep(5);
        //gpio_direction_output(lcd_rst, 1);
       // msleep(10);
        gpio_direction_output(lcd_rst,0 );
        msleep(10);
        gpio_direction_output(lcd_rst, 1);
        msleep(20);


        gpio_free(lcd_pwr_en);
        gpio_free(lcd_rst);


#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	inx_nt51021__panel_device.status = ON;
	mutex_lock(&dev_priv->bk_status_lock);
	dev_priv->bk_status = true;
	mutex_unlock(&dev_priv->bk_status_lock);	
	DRM_INFO("%s--ON\n", __func__);
#endif
	return;
}


void inx_nt51021_power_off(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	int retval = 0;
        unsigned int lcd_pwr_en = 279;//GPIO SE51
        unsigned int lcd_rst = 406;//GPIO N65

#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	inx_nt51021__panel_device.status = OFF;
	mutex_lock(&dev_priv->bk_status_lock);
	dev_priv->bk_status = false;
	mutex_unlock(&dev_priv->bk_status_lock);	
	DRM_INFO("%s--OFF\n", __func__);
#endif
	DRM_DEBUG_KMS("\n");
	
	if(yeti_hw_ver >= 0x3)
                lcd_pwr_en = 401;//N60
	
        retval = gpio_request(lcd_pwr_en, "enable_lcd_pwr");
        if (retval) {
            pr_err("%s: Failed to get gpio %d (code: %d)", __func__, lcd_pwr_en, retval);
            return;
         }

        retval = gpio_request(lcd_rst, "lcd_rst");
        if (retval) {
            pr_err("%s: Failed to get gpio %d (code: %d)", __func__, lcd_rst, retval);
            gpio_free(lcd_pwr_en);

            return;
         }

       // msleep(100);
        gpio_direction_output(lcd_rst, 0);
      //  msleep(1);
        gpio_direction_output(lcd_pwr_en,0 );
        msleep(500);

         gpio_free(lcd_pwr_en);
         gpio_free(lcd_rst);

	return;
}



/* Callbacks. We might not need them all. */

struct intel_dsi_dev_ops inx_nt51021_dsi_display_ops = {
	.init = inx_nt51021_init,
	.get_info = inx_nt51021_vid_get_panel_info,
	.get_modes = inx_nt51021_get_modes,
	.panel_reset = inx_nt51021_panel_reset,
	.disable_panel_power = inx_nt51021_disable_panel_power,
	.send_otp_cmds = inx_nt51021_send_otp_cmds,
	.enable = inx_nt51021_enable,
	.disable = inx_nt51021_disable,
	.power_on = inx_nt51021_power_on,
	.power_off = inx_nt51021_power_off,
	.enable_backlight = inx_nt51021_enable_bklt,
	.disable_backlight = inx_nt51021_disable_bklt,
//	.set_brightness = inx_nt51021_set_brightness,
};
