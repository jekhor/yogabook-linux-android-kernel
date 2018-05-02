/*
 * Copyright © 2006-2010 Intel Corporation
 * Copyright (c) 2006 Dave Airlie <airlied@linux.ie>
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
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *      Dave Airlie <airlied@linux.ie>
 *      Jesse Barnes <jesse.barnes@intel.com>
 *      Chris Wilson <chris@chris-wilson.co.uk>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/moduleparam.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include "intel_drv.h"
#include "intel_dsi.h"
#include "i915_drv.h"
#include <linux/platform_data/lp855x.h>
#include "intel_dsi_cmd.h"

#define BL_IC_I2C_BUS		3
#define BL_IC_I2C_ADDR		0x2c

/* LP8555/7 Registers */
#define LP8557_BL_CMD				0x00
#define LP8557_BRIGHTNESS_CTRL		0x04
#define LP8557_CONFIG				0x10
#define LP8557_CURRENT_CONFIG		0x11

#define LP8557_BL_MASK				0x01
#define LP8557_BL_ON				0x01
#define LP8557_BL_OFF				0x00

#define LP8557_CURRENT_MAX			0x5

#define	BLADE_BACKLIGHT_CONTROL_BY_I2C		1

#define BLADEIII_10A_INIT_BL_LEVEL			102
#define BLADEIII_10A_LOW_BATTERY_BL_LEVEL		5

/* Backlight control type */
#define BL_CTRL_BY_PANEL	1
#define BL_CTRL_BY_I2C		2

//#define BL_REFER_BATTERY_LEVEL	1
#ifdef BL_REFER_BATTERY_LEVEL
extern int read_rsoc(int m); //liulc1
#endif
//extern int chv_get_lcd_id(void);

static int
chv_exec_i2c(u8 bus_number, struct i2c_msg *msg, u8 msg_cnt)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	u8 retries = 5;

	DRM_DEBUG_DRIVER("%s: bus_number = %u, slave_add = 0x%x, msg_flags = 0x%x, reg_offset = 0x%x\n",
		__func__, bus_number, msg->addr, msg->flags, msg->buf[0]);

	adapter = i2c_get_adapter(bus_number);
	if (!adapter) {
		DRM_ERROR("i2c_get_adapter(%u) failed.\n",
				(bus_number + 1));
		ret = -EPERM;
		goto out;
	}

	do {
		ret =  i2c_transfer(adapter, msg, msg_cnt);
		if (ret == msg_cnt) {
			DRM_DEBUG_DRIVER("%s: i2c transfer succeed.\n", __func__);
			break;
		} else if (ret == -EAGAIN)
			usleep_range(1000, 2500);
		else if (ret != 1) {
			DRM_ERROR("i2c transfer failed %d\n", ret);
			break;
		}
	} while (--retries);

	if (retries == 0)
		DRM_ERROR("i2c transfer failed");

out:
	DRM_DEBUG_DRIVER("%s return %d.\n", __func__, ret);
	return ret;
}

static unsigned char chv_read_reg(unsigned char reg )
{
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	msg[0].addr =BL_IC_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr =BL_IC_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 1;

	ret = chv_exec_i2c(BL_IC_I2C_BUS, msg, 2);
	if(ret == 2) {
		DRM_DEBUG_DRIVER("read reg = %x data ok, data[0] = %d\n", reg, data[0]);
	}

	return data[0];
}

extern int read_backlight_id(void);
int read_backlight_id(void)
{
	if (chv_read_reg(0x1) != 0)
		return -1;
	else
		return 0;
}

static int inline get_backlight_controllor_type(void)
{
//	if(chv_get_lcd_id() == 1)
//		return BL_CTRL_BY_I2C;

	return BL_CTRL_BY_I2C;
	//return BL_CTRL_BY_PANEL;
}

void
intel_fixed_panel_mode(const struct drm_display_mode *fixed_mode,
		       struct drm_display_mode *adjusted_mode)
{
	drm_mode_copy(adjusted_mode, fixed_mode);

	drm_mode_set_crtcinfo(adjusted_mode, 0);
}

/**
 * intel_find_panel_downclock - find the reduced downclock for LVDS in EDID
 * @dev: drm device
 * @fixed_mode : panel native mode
 * @connector: LVDS/eDP connector
 *
 * Return downclock_avail
 * Find the reduced downclock for LVDS/eDP in EDID.
 */
struct drm_display_mode *
intel_find_panel_downclock(struct drm_device *dev,
			struct drm_display_mode *fixed_mode,
			struct drm_connector *connector)
{
	struct drm_display_mode *scan, *tmp_mode;
	int temp_downclock;

	temp_downclock = fixed_mode->clock;
	tmp_mode = NULL;

	list_for_each_entry(scan, &connector->probed_modes, head) {
		/*
		 * If one mode has the same resolution with the fixed_panel
		 * mode while they have the different refresh rate, it means
		 * that the reduced downclock is found. In such
		 * case we can set the different FPx0/1 to dynamically select
		 * between low and high frequency.
		 */
		if (scan->hdisplay == fixed_mode->hdisplay &&
		    scan->hsync_start == fixed_mode->hsync_start &&
		    scan->hsync_end == fixed_mode->hsync_end &&
		    scan->htotal == fixed_mode->htotal &&
		    scan->vdisplay == fixed_mode->vdisplay &&
		    scan->vsync_start == fixed_mode->vsync_start &&
		    scan->vsync_end == fixed_mode->vsync_end &&
		    scan->vtotal == fixed_mode->vtotal) {
			if (scan->clock < temp_downclock) {
				/*
				 * The downclock is already found. But we
				 * expect to find the lower downclock.
				 */
				temp_downclock = scan->clock;
				tmp_mode = scan;
			}
		}
	}

	if (temp_downclock < fixed_mode->clock)
		return drm_mode_duplicate(dev, tmp_mode);
	else
		return NULL;
}

/* adjusted_mode has been preset to be the panel's fixed mode */
void
intel_pch_panel_fitting(struct intel_crtc *intel_crtc,
			struct intel_crtc_config *pipe_config,
			int fitting_mode)
{
	struct drm_display_mode *adjusted_mode;
	int x, y, width, height;

	adjusted_mode = &pipe_config->adjusted_mode;

	x = y = width = height = 0;

	/* Native modes don't need fitting */
	if (adjusted_mode->hdisplay == pipe_config->pipe_src_w &&
	    adjusted_mode->vdisplay == pipe_config->pipe_src_h)
		goto done;

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		width = pipe_config->pipe_src_w;
		height = pipe_config->pipe_src_h;
		x = (adjusted_mode->hdisplay - width + 1)/2;
		y = (adjusted_mode->vdisplay - height + 1)/2;
		break;

	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		{
			u32 scaled_width = adjusted_mode->hdisplay
				* pipe_config->pipe_src_h;
			u32 scaled_height = pipe_config->pipe_src_w
				* adjusted_mode->vdisplay;
			if (scaled_width > scaled_height) { /* pillar */
				width = scaled_height / pipe_config->pipe_src_h;
				if (width & 1)
					width++;
				x = (adjusted_mode->hdisplay - width + 1) / 2;
				y = 0;
				height = adjusted_mode->vdisplay;
			} else if (scaled_width < scaled_height) { /* letter */
				height = scaled_width / pipe_config->pipe_src_w;
				if (height & 1)
				    height++;
				y = (adjusted_mode->vdisplay - height + 1) / 2;
				x = 0;
				width = adjusted_mode->hdisplay;
			} else {
				x = y = 0;
				width = adjusted_mode->hdisplay;
				height = adjusted_mode->vdisplay;
			}
		}
		break;

	case DRM_MODE_SCALE_FULLSCREEN:
		x = y = 0;
		width = adjusted_mode->hdisplay;
		height = adjusted_mode->vdisplay;
		break;

	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

done:
	pipe_config->pch_pfit.pos = (x << 16) | y;
	pipe_config->pch_pfit.size = (width << 16) | height;
	pipe_config->pch_pfit.enabled = pipe_config->pch_pfit.size != 0;
}

static void
centre_horizontally(struct drm_display_mode *mode,
		    int width)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the hsync and hblank widths constant */
	sync_width = mode->crtc_hsync_end - mode->crtc_hsync_start;
	blank_width = mode->crtc_hblank_end - mode->crtc_hblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->hdisplay - width + 1) / 2;
	border += border & 1; /* make the border even */

	mode->crtc_hdisplay = width;
	mode->crtc_hblank_start = width + border;
	mode->crtc_hblank_end = mode->crtc_hblank_start + blank_width;

	mode->crtc_hsync_start = mode->crtc_hblank_start + sync_pos;
	mode->crtc_hsync_end = mode->crtc_hsync_start + sync_width;
}

static void
centre_vertically(struct drm_display_mode *mode,
		  int height)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the vsync and vblank widths constant */
	sync_width = mode->crtc_vsync_end - mode->crtc_vsync_start;
	blank_width = mode->crtc_vblank_end - mode->crtc_vblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->vdisplay - height + 1) / 2;

	mode->crtc_vdisplay = height;
	mode->crtc_vblank_start = height + border;
	mode->crtc_vblank_end = mode->crtc_vblank_start + blank_width;

	mode->crtc_vsync_start = mode->crtc_vblank_start + sync_pos;
	mode->crtc_vsync_end = mode->crtc_vsync_start + sync_width;
}

static inline u32 panel_fitter_scaling(u32 source, u32 target)
{
	/*
	 * Floating point operation is not supported. So the FACTOR
	 * is defined, which can avoid the floating point computation
	 * when calculating the panel ratio.
	 */
#define ACCURACY 12
#define FACTOR (1 << ACCURACY)
	u32 ratio = source * FACTOR / target;
	return (FACTOR * ratio + FACTOR/2) / FACTOR;
}

static void i965_scale_aspect(struct intel_crtc_config *pipe_config,
			      u32 *pfit_control)
{
	struct drm_display_mode *adjusted_mode = &pipe_config->adjusted_mode;
	u32 scaled_width = adjusted_mode->hdisplay *
		pipe_config->pipe_src_h;
	u32 scaled_height = pipe_config->pipe_src_w *
		adjusted_mode->vdisplay;

	/* 965+ is easy, it does everything in hw */
	if (scaled_width > scaled_height)
		*pfit_control |= PFIT_ENABLE |
			PFIT_SCALING_PILLAR;
	else if (scaled_width < scaled_height)
		*pfit_control |= PFIT_ENABLE |
			PFIT_SCALING_LETTER;
	else if (adjusted_mode->hdisplay != pipe_config->pipe_src_w)
		*pfit_control |= PFIT_ENABLE | PFIT_SCALING_AUTO;
}

static void i9xx_scale_aspect(struct intel_crtc_config *pipe_config,
			      u32 *pfit_control, u32 *pfit_pgm_ratios,
			      u32 *border)
{
	struct drm_display_mode *adjusted_mode = &pipe_config->adjusted_mode;
	u32 scaled_width = adjusted_mode->hdisplay *
		pipe_config->pipe_src_h;
	u32 scaled_height = pipe_config->pipe_src_w *
		adjusted_mode->vdisplay;
	u32 bits;

	/*
	 * For earlier chips we have to calculate the scaling
	 * ratio by hand and program it into the
	 * PFIT_PGM_RATIO register
	 */
	if (scaled_width > scaled_height) { /* pillar */
		centre_horizontally(adjusted_mode,
				    scaled_height /
				    pipe_config->pipe_src_h);

		*border = LVDS_BORDER_ENABLE;
		if (pipe_config->pipe_src_h != adjusted_mode->vdisplay) {
			bits = panel_fitter_scaling(pipe_config->pipe_src_h,
						    adjusted_mode->vdisplay);

			*pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
					     bits << PFIT_VERT_SCALE_SHIFT);
			*pfit_control |= (PFIT_ENABLE |
					  VERT_INTERP_BILINEAR |
					  HORIZ_INTERP_BILINEAR);
		}
	} else if (scaled_width < scaled_height) { /* letter */
		centre_vertically(adjusted_mode,
				  scaled_width /
				  pipe_config->pipe_src_w);

		*border = LVDS_BORDER_ENABLE;
		if (pipe_config->pipe_src_w != adjusted_mode->hdisplay) {
			bits = panel_fitter_scaling(pipe_config->pipe_src_w,
						    adjusted_mode->hdisplay);

			*pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
					     bits << PFIT_VERT_SCALE_SHIFT);
			*pfit_control |= (PFIT_ENABLE |
					  VERT_INTERP_BILINEAR |
					  HORIZ_INTERP_BILINEAR);
		}
	} else {
		/* Aspects match, Let hw scale both directions */
		*pfit_control |= (PFIT_ENABLE |
				  VERT_AUTO_SCALE | HORIZ_AUTO_SCALE |
				  VERT_INTERP_BILINEAR |
				  HORIZ_INTERP_BILINEAR);
	}
}

void intel_gmch_panel_fitting(struct intel_crtc *intel_crtc,
			      struct intel_crtc_config *pipe_config,
			      int fitting_mode)
{
	struct drm_device *dev = intel_crtc->base.dev;
	u32 pfit_control = 0, pfit_pgm_ratios = 0, border = 0;
	struct drm_display_mode *mode, *adjusted_mode;
	uint32_t scaling_src_w, scaling_src_h = 0;

	intel_crtc->base.panning_en = false;

	mode = &pipe_config->requested_mode;
	adjusted_mode = &pipe_config->adjusted_mode;

	if (IS_VALLEYVIEW(dev)) {
		scaling_src_w = ((intel_crtc->scaling_src_size >>
				SCALING_SRCSIZE_SHIFT) &
				SCALING_SRCSIZE_MASK) + 1;
		scaling_src_h = (intel_crtc->scaling_src_size &
				SCALING_SRCSIZE_MASK) + 1;

		/* The input src size should be < 2kx2k */
		if ((scaling_src_w > PFIT_SIZE_LIMIT) ||
			(scaling_src_h > PFIT_SIZE_LIMIT)) {
			DRM_ERROR("Wrong panel fitter input src conf");
			goto out;
		}

		if (fitting_mode == AUTOSCALE)
			pfit_control = PFIT_SCALING_AUTO;
		else if (fitting_mode == PILLARBOX)
			pfit_control = PFIT_SCALING_PILLAR;
		else if (fitting_mode == LETTERBOX)
			pfit_control = PFIT_SCALING_LETTER;
		else {
			pipe_config->gmch_pfit.control &= ~PFIT_ENABLE;
			intel_crtc->base.panning_en = false;
			goto out1;
		}
		pfit_control |= (PFIT_ENABLE | (intel_crtc->pipe
					<< PFIT_PIPE_SHIFT));
		intel_crtc->base.panning_en = true;
		goto out;

		/* Native modes don't need fitting */
		if (adjusted_mode->hdisplay == mode->hdisplay &&
		    adjusted_mode->vdisplay == mode->vdisplay)
			goto out;
	}

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		/*
		 * For centered modes, we have to calculate border widths &
		 * heights and modify the values programmed into the CRTC.
		 */
		centre_horizontally(adjusted_mode, pipe_config->pipe_src_w);
		centre_vertically(adjusted_mode, pipe_config->pipe_src_h);
		border = LVDS_BORDER_ENABLE;
		break;
	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		if (INTEL_INFO(dev)->gen >= 4)
			i965_scale_aspect(pipe_config, &pfit_control);
		else
			i9xx_scale_aspect(pipe_config, &pfit_control,
					  &pfit_pgm_ratios, &border);
		break;
	case DRM_MODE_SCALE_FULLSCREEN:
		/*
		 * Full scaling, even if it changes the aspect ratio.
		 * Fortunately this is all done for us in hw.
		 */
		if (pipe_config->pipe_src_h != adjusted_mode->vdisplay ||
		    pipe_config->pipe_src_w != adjusted_mode->hdisplay) {
			pfit_control |= PFIT_ENABLE;
			if (INTEL_INFO(dev)->gen >= 4)
				pfit_control |= PFIT_SCALING_AUTO;
			else
				pfit_control |= (VERT_AUTO_SCALE |
						 VERT_INTERP_BILINEAR |
						 HORIZ_AUTO_SCALE |
						 HORIZ_INTERP_BILINEAR);
		}
		break;
	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

	/* 965+ wants fuzzy fitting */
	/* FIXME: handle multiple panels by failing gracefully */
	if (INTEL_INFO(dev)->gen >= 4)
		pfit_control |= ((intel_crtc->pipe << PFIT_PIPE_SHIFT) |
				 PFIT_FILTER_FUZZY);

	/* Make sure pre-965 set dither correctly for 18bpp panels. */
	if (INTEL_INFO(dev)->gen < 4 && pipe_config->pipe_bpp == 18)
		pfit_control |= PANEL_8TO6_DITHER_ENABLE;

out:
	if ((pfit_control & PFIT_ENABLE) == 0) {
		pfit_control = 0;
		pfit_pgm_ratios = 0;
		intel_crtc->scaling_src_size = 0;
	}

	pipe_config->gmch_pfit.control = pfit_control;

out1:
	pipe_config->gmch_pfit.pgm_ratios = pfit_pgm_ratios;
	pipe_config->gmch_pfit.lvds_border_bits = border;
}

enum drm_connector_status
intel_panel_detect(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Assume that the BIOS does not lie through the OpRegion... */
	if (!i915.panel_ignore_lid && dev_priv->opregion.lid_state) {
		return ioread32(dev_priv->opregion.lid_state) & 0x1 ?
			connector_status_connected :
			connector_status_disconnected;
	}

	switch (i915.panel_ignore_lid) {
	case -2:
		return connector_status_connected;
	case -1:
		return connector_status_disconnected;
	default:
		return connector_status_unknown;
	}
}

static u32 intel_panel_compute_brightness(struct intel_connector *connector,
					  u32 val)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;

	WARN_ON(panel->backlight.max == 0);

	if (i915.invert_brightness < 0)
		return val;

	if (i915.invert_brightness > 0 ||
	    dev_priv->quirks & QUIRK_INVERT_BRIGHTNESS) {
		return panel->backlight.max - val;
	}

	return val;
}

static u32 bdw_get_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	return I915_READ(BLC_PWM_PCH_CTL2) & BACKLIGHT_DUTY_CYCLE_MASK;
}

static u32 pch_get_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	return I915_READ(BLC_PWM_CPU_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
}

static u32 i9xx_get_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 val;

	val = I915_READ(BLC_PWM_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
	if (INTEL_INFO(dev)->gen < 4)
		val >>= 1;

	if (panel->backlight.combination_mode) {
		u8 lbpc;

		pci_read_config_byte(dev->pdev, PCI_LBPC, &lbpc);
		val *= lbpc;
	}

	return val;
}

static u32 vlv_get_mipi_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = 0;

	if (dev_priv->vbt.dsi.config->pmic_soc_blc)
		ret = (~lpio_bl_read(0, LPIO_PWM_CTRL)) & 0xff;
	else
		ret = intel_soc_pmic_readb(PMIC_PWM_LEVEL);

	if ((ret < 0) || (ret > 255)) {
		DRM_ERROR("get backlight out of range. ret = %d\n", ret);
		ret = 153;
	}

	return ret;
}

static u32 _vlv_get_backlight(struct drm_device *dev, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	return I915_READ(VLV_BLC_PWM_CTL(pipe)) &
			BACKLIGHT_DUTY_CYCLE_MASK;
}

static u32 vlv_get_backlight_lpss_pwm(struct intel_connector *connector)
{
	return lpio_bl_read(0, LPIO_PWM_CTRL) & 0xff;
}
static u32 vlv_get_backlight_dis_ddi(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	enum pipe pipe = intel_get_pipe_from_connector(connector);

	return _vlv_get_backlight(dev, pipe);
}

static u32 intel_panel_get_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val;

	if (dev_priv->vbt.has_mipi) {
		val = dev_priv->display.get_backlight(connector);
	} else {
		mutex_lock(&dev_priv->backlight_lock);

		val = dev_priv->display.get_backlight(connector);

		/* When DPST is enabled, reading the backlight register will
		 * give the DPST adjusted backlight value. Since DPST works
		 * without user knowing a perceived difference in the backlight,
		 * the programmed backlight isn't the correct value to return.
		 * So, get the user perceived backlight level from DPST. */
		if (dev_priv->dpst.enabled)
			val = i915_dpst_get_brightness(dev);
		else
			val = intel_panel_compute_brightness(connector, val);

		mutex_unlock(&dev_priv->backlight_lock);
	}

	DRM_DEBUG_DRIVER("get backlight PWM = %d\n", val);
	return val;
}

static void bdw_set_backlight(struct intel_connector *connector, u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(BLC_PWM_PCH_CTL2) & ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_PCH_CTL2, val | level);
}

static void pch_set_backlight(struct intel_connector *connector, u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp;

	tmp = I915_READ(BLC_PWM_CPU_CTL) & ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_CPU_CTL, tmp | level);
}

static void i9xx_set_backlight(struct intel_connector *connector, u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 tmp, mask;

	WARN_ON(panel->backlight.max == 0);

	if (panel->backlight.combination_mode) {
		u8 lbpc;

		lbpc = level * 0xfe / panel->backlight.max + 1;
		level /= lbpc;
		pci_write_config_byte(dev->pdev, PCI_LBPC, lbpc);
	}

	if (IS_GEN4(dev)) {
		mask = BACKLIGHT_DUTY_CYCLE_MASK;
	} else {
		level <<= 1;
		mask = BACKLIGHT_DUTY_CYCLE_MASK_PNV;
	}

	tmp = I915_READ(BLC_PWM_CTL) & ~mask;
	I915_WRITE(BLC_PWM_CTL, tmp | level);
}

static void vlv_set_backlight_lpss_pwm(struct intel_connector *connector,
						u32 level)
{
	lpio_bl_write_bits(0, LPIO_PWM_CTRL, (0xff - level), 0xFF);
	lpio_bl_update(0, LPIO_PWM_CTRL);

}
static void vlv_set_backlight_dis_ddi(struct intel_connector *connector,
						u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	u32 tmp;

	tmp = I915_READ(VLV_BLC_PWM_CTL(pipe)) &
		~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(VLV_BLC_PWM_CTL(pipe), tmp | level);
}

static void vlv_set_mipi_backlight(struct intel_connector *connector, u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* backlight control for BladeIII 10A */
	struct intel_dsi *intel_dsi = NULL;
	struct drm_crtc *crtc = NULL;
	struct intel_encoder *encoder = NULL;
	struct intel_dsi_device *dsi = NULL;
	struct intel_panel *panel = &connector->panel;

	if(panel->backlight.enabled == false) {
		DRM_INFO("backlight has been disabled.\n");
		return;
	}

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		for_each_encoder_on_crtc(dev, crtc, encoder) {
			if (encoder->type == INTEL_OUTPUT_DSI){
				intel_dsi = enc_to_intel_dsi(&encoder->base);
				dsi = &(intel_dsi->dev);
			}
		}
	}

	if (intel_dsi->dev.dev_ops->set_brightness)
		intel_dsi->dev.dev_ops->set_brightness(&intel_dsi->dev,level);

	if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
		/* FixMe: if level is zero still a pulse is observed consuming
		 * power. To fix this issue if requested level is zero then
		 * disable pwm and enabled it again if brightness changes
		 */
		lpio_bl_write_bits(0, LPIO_PWM_CTRL, (0xff - level), 0xFF);
		lpio_bl_update(0, LPIO_PWM_CTRL);
	} else
		dsi_soc_pmic_writeb(PMIC_PWM_LEVEL, level);
}

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
static void vlv_set_backlight_to_ic(struct intel_connector *connector, u32 level)
{
	u8 transmit_buffer[4] = {0};
	struct i2c_msg msg;

	DRM_DEBUG_KMS("%s: set backlight to %u by I2C.\n", __func__, level);

	msg.addr = BL_IC_I2C_ADDR;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = transmit_buffer;

	transmit_buffer[0] = LP8557_BRIGHTNESS_CTRL;
	transmit_buffer[1] = (level & 0xff);
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
}
#endif

void
intel_panel_actually_set_backlight(struct intel_connector *connector, u32 level)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
#ifdef BL_REFER_BATTERY_LEVEL
	int battery_a = read_rsoc(0);
	int battery_b = read_rsoc(1);

	if((battery_a < 3) && (battery_b < 3) &&
			(level > BLADEIII_10A_LOW_BATTERY_BL_LEVEL)) {
		DRM_INFO("%s: Low Battery [%d, %d]. Use low backlight level.",
			__func__, battery_a, battery_b);
		level = BLADEIII_10A_LOW_BATTERY_BL_LEVEL;
	}
#endif

	DRM_DEBUG_DRIVER("set backlight PWM = %d\n", level);

	level = intel_panel_compute_brightness(connector, level);
	dev_priv->display.set_backlight(connector, level);

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
	if(get_backlight_controllor_type() == BL_CTRL_BY_I2C)
		vlv_set_backlight_to_ic(connector, level);
#endif

}

/* set backlight brightness to level in range [0..max] */
void intel_panel_set_backlight(struct intel_connector *connector, u32 level,
			       u32 max)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	u32 freq;
	u64 n;

	if (!panel->backlight.present || pipe == INVALID_PIPE)
		return;

	mutex_lock(&dev_priv->backlight_lock);

	WARN_ON(panel->backlight.max == 0);

	/* scale to hardware max, but be careful to not overflow */
	freq = panel->backlight.max;
	n = (u64)level * freq;
	do_div(n, max);
	level = n;

	panel->backlight.level = level;
	DRM_DEBUG_KMS("level = %d\n", level);
	if (panel->backlight.device)
		panel->backlight.device->props.brightness = level;

	if (panel->backlight.enabled) {
		if (dev_priv->dpst.enabled && !dev_priv->vbt.has_mipi)
			i915_dpst_set_brightness(dev, level);
		else if (!dev_priv->vbt.has_mipi)
			intel_panel_actually_set_backlight(connector, level);
	}

	mutex_unlock(&dev_priv->backlight_lock);

	if (dev_priv->vbt.has_mipi && dev_priv->dpst.enabled)
		i915_dpst_set_brightness(dev, level);
	else if (dev_priv->vbt.has_mipi)
		intel_panel_actually_set_backlight(connector, level);
}

static void pch_disable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp;

	intel_panel_actually_set_backlight(connector, 0);

	tmp = I915_READ(BLC_PWM_CPU_CTL2);
	I915_WRITE(BLC_PWM_CPU_CTL2, tmp & ~BLM_PWM_ENABLE);

	tmp = I915_READ(BLC_PWM_PCH_CTL1);
	I915_WRITE(BLC_PWM_PCH_CTL1, tmp & ~BLM_PCH_PWM_ENABLE);
}

static void i9xx_disable_backlight(struct intel_connector *connector)
{
	intel_panel_actually_set_backlight(connector, 0);
}

static void i965_disable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp;

	intel_panel_actually_set_backlight(connector, 0);

	tmp = I915_READ(BLC_PWM_CTL2);
	I915_WRITE(BLC_PWM_CTL2, tmp & ~BLM_PWM_ENABLE);
}


static void vlv_disable_backlight_lpss_pwm(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	intel_panel_actually_set_backlight(connector, 0);
	/* disable the backlight enable signal */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			PANEL1_BKLTEN_GPIONC_10_PCONF0, VLV_GPIO_CFG);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			PANEL1_BKLTEN_GPIONC_10_PAD, VLV_GPIO_INPUT_DIS);
	udelay(500);
	lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x00, LPIO_PWM_ENABLE_MASK);
}

static void vlv_disable_backlight_dis_ddi(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	u32 tmp;

	intel_panel_actually_set_backlight(connector, 0);

	tmp = I915_READ(VLV_BLC_PWM_CTL2(pipe));
	I915_WRITE(VLV_BLC_PWM_CTL2(pipe), tmp & ~BLM_PWM_ENABLE);
}

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
static void vlv_disable_backlight_ic(struct intel_connector *connector)
{
	u8 transmit_buffer[4] = {0};
	struct i2c_msg msg;

	DRM_INFO("%s: disable backlight IC by I2C command.\n", __func__);
	msg.addr = BL_IC_I2C_ADDR;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = transmit_buffer;

	transmit_buffer[0] = LP8557_BL_CMD;
	transmit_buffer[1] = LP8557_BL_OFF;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	mdelay(10);
}
#endif

static void vlv_disable_mipi_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = NULL;
	struct drm_crtc *crtc = NULL;
	struct intel_encoder *encoder = NULL;
	struct intel_panel *panel = &connector->panel;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		for_each_encoder_on_crtc(dev, crtc, encoder) {
			if (encoder->type == INTEL_OUTPUT_DSI)
				intel_dsi = enc_to_intel_dsi(&encoder->base);
		}
	}

	intel_panel_actually_set_backlight(connector, 0);

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
	if(get_backlight_controllor_type() == BL_CTRL_BY_I2C)
		vlv_disable_backlight_ic(connector);
	else {
		if (intel_dsi != NULL && intel_dsi->dev.dev_ops->disable_backlight)
			intel_dsi->dev.dev_ops->disable_backlight(&intel_dsi->dev);
	}
#else
	if (intel_dsi != NULL && intel_dsi->dev.dev_ops->disable_backlight)
		intel_dsi->dev.dev_ops->disable_backlight(&intel_dsi->dev);
#endif

	if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
		lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x00, 0x80000000);
	}

	panel->backlight.enabled = false;
}

void intel_panel_disable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);

	if (!panel->backlight.present || pipe == INVALID_PIPE)
		return;

	/*
	 * Do not disable backlight on the vgaswitcheroo path. When switching
	 * away from i915, the other client may depend on i915 to handle the
	 * backlight. This will leave the backlight on unnecessarily when
	 * another client is not activated.
	 */
	if (dev->switch_power_state == DRM_SWITCH_POWER_CHANGING) {
		DRM_DEBUG_DRIVER("Skipping backlight disable on vga switch\n");
		return;
	}

	mutex_lock(&dev_priv->backlight_lock);

	panel->backlight.enabled = false;

	if (!dev_priv->vbt.has_mipi) {
		dev_priv->display.disable_backlight(connector);
		mutex_unlock(&dev_priv->backlight_lock);
		return;
	}

	mutex_unlock(&dev_priv->backlight_lock);

	/*
	 * in case of MIPI we use PMIC cals which cannot be done within
	 * mutex
	 */
	dev_priv->display.disable_backlight(connector);
}

static void bdw_enable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 pch_ctl1, pch_ctl2;

	pch_ctl1 = I915_READ(BLC_PWM_PCH_CTL1);
	if (pch_ctl1 & BLM_PCH_PWM_ENABLE) {
		DRM_DEBUG_KMS("pch backlight already enabled\n");
		pch_ctl1 &= ~BLM_PCH_PWM_ENABLE;
		I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1);
	}

	pch_ctl2 = panel->backlight.max << 16;
	I915_WRITE(BLC_PWM_PCH_CTL2, pch_ctl2);

	pch_ctl1 = 0;
	if (panel->backlight.active_low_pwm)
		pch_ctl1 |= BLM_PCH_POLARITY;

	/* BDW always uses the pch pwm controls. */
	pch_ctl1 |= BLM_PCH_OVERRIDE_ENABLE;

	I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1);
	POSTING_READ(BLC_PWM_PCH_CTL1);
	I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1 | BLM_PCH_PWM_ENABLE);

	/* This won't stick until the above enable. */
	intel_panel_actually_set_backlight(connector, panel->backlight.level);
}

static uint32_t compute_pwm_base(uint16_t freq)
{
	uint32_t base_unit;
	if (freq < 400)
		freq = 400;
	/*The PWM block is clocked by the 25MHz oscillator clock.
	 * The output frequency can be estimated with the equation:
	 * Target frequency = XOSC * Base_unit_value/256
	 */
	base_unit = (freq * 256) / 25;

	/* Also Base_unit_value need to converted to QM.N notation
	 * to program the value in register
	 * Using the following for converting to Q8.8 notation
	 * For QM.N representation, consider a floating point variable 'a' :
	 * Step 1: Calculate b = a* 2^N , where N is the fractional length
	 * of the variable.
	 * Note that a is represented in decimal.
	 * Step 2: Round the value of 'b' to the nearest integer value.
	 * For example:
	 * RoundOff (1.05) --> 1
	 * RoundOff (1.5)  --> 2
	 * Step 3: Convert 'b' from decimal to binary representation and
	 * name the new variable 'c'
	 */
	base_unit = base_unit * 256;
	base_unit = DIV_ROUND_CLOSEST(base_unit, 1000000);

	return base_unit;
}

static void lpio_enable_backlight(struct drm_device *dev)
{
	uint32_t pwm_base;
	struct drm_i915_private *dev_priv = dev->dev_private;

#if 0	/* don't need it for BladeIII 10A */
	uint32_t val;

	/* GPIOC_94 config to PWM0 function */
	val = vlv_gps_core_read(dev_priv, GP_CAMERASB07_GPIONC_22_PCONF0);
	vlv_gps_core_write(dev_priv, GP_CAMERASB07_GPIONC_22_PCONF0,
				0x2000CC01);
	vlv_gps_core_write(dev_priv, GP_CAMERASB07_GPIONC_22_PAD, 0x5);
#endif

	/* PWM enable
	 * Assuming only 1 LFP
	 */
	pwm_base = compute_pwm_base(dev_priv->vbt.backlight.pwm_freq_hz);
	pwm_base = pwm_base << 8;
	lpio_bl_write(0, LPIO_PWM_CTRL, pwm_base);
	lpio_bl_update(0, LPIO_PWM_CTRL);
	lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x80000000,
			0x80000000);
	lpio_bl_update(0, LPIO_PWM_CTRL);
}

static void pch_enable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	enum transcoder cpu_transcoder =
		intel_pipe_to_cpu_transcoder(dev_priv, pipe);
	u32 cpu_ctl2, pch_ctl1, pch_ctl2;

	cpu_ctl2 = I915_READ(BLC_PWM_CPU_CTL2);
	if (cpu_ctl2 & BLM_PWM_ENABLE) {
		WARN(1, "cpu backlight already enabled\n");
		cpu_ctl2 &= ~BLM_PWM_ENABLE;
		I915_WRITE(BLC_PWM_CPU_CTL2, cpu_ctl2);
	}

	pch_ctl1 = I915_READ(BLC_PWM_PCH_CTL1);
	if (pch_ctl1 & BLM_PCH_PWM_ENABLE) {
		DRM_DEBUG_KMS("pch backlight already enabled\n");
		pch_ctl1 &= ~BLM_PCH_PWM_ENABLE;
		I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1);
	}

	if (cpu_transcoder == TRANSCODER_EDP)
		cpu_ctl2 = BLM_TRANSCODER_EDP;
	else
		cpu_ctl2 = BLM_PIPE(cpu_transcoder);
	I915_WRITE(BLC_PWM_CPU_CTL2, cpu_ctl2);
	POSTING_READ(BLC_PWM_CPU_CTL2);
	I915_WRITE(BLC_PWM_CPU_CTL2, cpu_ctl2 | BLM_PWM_ENABLE);

	/* This won't stick until the above enable. */
	if (dev_priv->dpst.enabled)
		i915_dpst_set_brightness(dev, panel->backlight.level);
	else
		intel_panel_actually_set_backlight(connector,
						   panel->backlight.level);

	pch_ctl2 = panel->backlight.max << 16;
	I915_WRITE(BLC_PWM_PCH_CTL2, pch_ctl2);

	pch_ctl1 = 0;
	if (panel->backlight.active_low_pwm)
		pch_ctl1 |= BLM_PCH_POLARITY;

	I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1);
	POSTING_READ(BLC_PWM_PCH_CTL1);
	I915_WRITE(BLC_PWM_PCH_CTL1, pch_ctl1 | BLM_PCH_PWM_ENABLE);
}

static void i9xx_enable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 ctl, freq;

	ctl = I915_READ(BLC_PWM_CTL);
	if (ctl & BACKLIGHT_DUTY_CYCLE_MASK_PNV) {
		WARN(1, "backlight already enabled\n");
		I915_WRITE(BLC_PWM_CTL, 0);
	}

	freq = panel->backlight.max;
	if (panel->backlight.combination_mode)
		freq /= 0xff;

	ctl = freq << 17;
	if (panel->backlight.combination_mode)
		ctl |= BLM_LEGACY_MODE;
	if (IS_PINEVIEW(dev) && panel->backlight.active_low_pwm)
		ctl |= BLM_POLARITY_PNV;

	I915_WRITE(BLC_PWM_CTL, ctl);
	POSTING_READ(BLC_PWM_CTL);

	/* XXX: combine this into above write? */
	intel_panel_actually_set_backlight(connector, panel->backlight.level);
}

static void i965_enable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	u32 ctl, ctl2, freq;

	ctl2 = I915_READ(BLC_PWM_CTL2);
	if (ctl2 & BLM_PWM_ENABLE) {
		WARN(1, "backlight already enabled\n");
		ctl2 &= ~BLM_PWM_ENABLE;
		I915_WRITE(BLC_PWM_CTL2, ctl2);
	}

	freq = panel->backlight.max;
	if (panel->backlight.combination_mode)
		freq /= 0xff;

	ctl = freq << 16;
	I915_WRITE(BLC_PWM_CTL, ctl);

	ctl2 = BLM_PIPE(pipe);
	if (panel->backlight.combination_mode)
		ctl2 |= BLM_COMBINATION_MODE;
	if (panel->backlight.active_low_pwm)
		ctl2 |= BLM_POLARITY_I965;
	I915_WRITE(BLC_PWM_CTL2, ctl2);
	POSTING_READ(BLC_PWM_CTL2);
	I915_WRITE(BLC_PWM_CTL2, ctl2 | BLM_PWM_ENABLE);

	intel_panel_actually_set_backlight(connector, panel->backlight.level);
}


static void vlv_enable_backlight_lpss_pwm(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	uint32_t val;

	panel->backlight.max = 0xFF;
	val = vlv_get_backlight_lpss_pwm(connector);
	panel->backlight.level =
		intel_panel_compute_brightness(connector, val);
	lpio_enable_backlight(dev);

	/* Backlight enable */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			PANEL1_BKLTEN_GPIONC_10_PCONF0, VLV_GPIO_CFG);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			PANEL1_BKLTEN_GPIONC_10_PAD, VLV_GPIO_INPUT_EN);
	panel->backlight.enabled = true;

}
static void vlv_enable_backlight_dis_ddi(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);
	u32 ctl, ctl2;

	ctl2 = I915_READ(VLV_BLC_PWM_CTL2(pipe));
	if (ctl2 & BLM_PWM_ENABLE) {
		WARN(1, "backlight already enabled\n");
		ctl2 &= ~BLM_PWM_ENABLE;
		I915_WRITE(VLV_BLC_PWM_CTL2(pipe), ctl2);
	}

	ctl = panel->backlight.max << 16;
	I915_WRITE(VLV_BLC_PWM_CTL(pipe), ctl);

	intel_panel_actually_set_backlight(connector,
			panel->backlight.level);

	ctl2 = 0;
	if (panel->backlight.active_low_pwm)
		ctl2 |= BLM_POLARITY_I965;
	I915_WRITE(VLV_BLC_PWM_CTL2(pipe), ctl2);
	POSTING_READ(VLV_BLC_PWM_CTL2(pipe));
	I915_WRITE(VLV_BLC_PWM_CTL2(pipe), ctl2 | BLM_PWM_ENABLE);
}

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
static void vlv_enable_backlight_ic(struct intel_connector *connector)
{
	u8 transmit_buffer[4] = {0};
	struct i2c_msg msg;

	DRM_INFO("%s: enable backlight IC by I2C command.\n", __func__);

	msg.addr = BL_IC_I2C_ADDR;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = transmit_buffer;

	transmit_buffer[0] = LP8557_BL_CMD;
	transmit_buffer[1] = LP8557_BL_ON;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	mdelay(10);
}
#endif

static int vlv_setup_backlight_ic(struct intel_connector *connector);
static void vlv_enable_mipi_backlight(struct intel_connector *connector)
{
	struct intel_panel *panel = &connector->panel;
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = NULL;
	struct drm_crtc *crtc = NULL;
	struct intel_encoder *encoder = NULL;
	static bool bBacklight_Mode_seted = false;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		for_each_encoder_on_crtc(dev, crtc, encoder) {
			if (encoder->type == INTEL_OUTPUT_DSI)
				intel_dsi = enc_to_intel_dsi(&encoder->base);
		}
	}

	if (bBacklight_Mode_seted == false) {
		/* setup backlight IC working mode */
		vlv_setup_backlight_ic(connector);
		bBacklight_Mode_seted = true;
	}


	if (dev_priv->vbt.dsi.config->pmic_soc_blc)
		lpio_enable_backlight(dev);

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
	if(get_backlight_controllor_type() == BL_CTRL_BY_I2C)
		vlv_enable_backlight_ic(connector);
	else {
		if (intel_dsi != NULL && intel_dsi->dev.dev_ops->enable_backlight)
			intel_dsi->dev.dev_ops->enable_backlight(&intel_dsi->dev);
	}
#else
	if (intel_dsi != NULL && intel_dsi->dev.dev_ops->enable_backlight)
		intel_dsi->dev.dev_ops->enable_backlight(&intel_dsi->dev);
#endif

	panel->backlight.enabled = true;

	intel_panel_actually_set_backlight(connector, panel->backlight.level);
}

void intel_panel_enable_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	enum pipe pipe = intel_get_pipe_from_connector(connector);

	if (!panel->backlight.present || pipe == INVALID_PIPE)
		return;

	DRM_DEBUG_KMS("pipe %c\n", pipe_name(pipe));

	mutex_lock(&dev_priv->backlight_lock);

	WARN_ON(panel->backlight.max == 0);

#if 0
	if (panel->backlight.level == 0) {
		panel->backlight.level = panel->backlight.max;
		if (panel->backlight.device)
			panel->backlight.device->props.brightness =
				panel->backlight.level;
	}
#endif

	panel->backlight.enabled = true;
	if (!dev_priv->vbt.has_mipi) {
		dev_priv->display.enable_backlight(connector);
		mutex_unlock(&dev_priv->backlight_lock);
		return;
	}

	mutex_unlock(&dev_priv->backlight_lock);

	/*
	 * in case of MIPI, PMIC is used which cannot be called
	 * within mutex
	 */
	dev_priv->display.enable_backlight(connector);
}

#if IS_ENABLED(CONFIG_BACKLIGHT_CLASS_DEVICE)
static int intel_backlight_device_update_status(struct backlight_device *bd)
{
	struct intel_connector *connector = bl_get_data(bd);
	struct drm_device *dev = connector->base.dev;

	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	DRM_DEBUG_KMS("updating intel_backlight, brightness=%d/%d\n",
		      bd->props.brightness, bd->props.max_brightness);
	intel_panel_set_backlight(connector, bd->props.brightness,
				  bd->props.max_brightness);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);
	return 0;
}

static int intel_backlight_device_get_brightness(struct backlight_device *bd)
{
	struct intel_connector *connector = bl_get_data(bd);
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	intel_runtime_pm_get(dev_priv);
	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	ret = intel_panel_get_backlight(connector);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);
	intel_runtime_pm_put(dev_priv);

	return ret;
}

static const struct backlight_ops intel_backlight_device_ops = {
	.update_status = intel_backlight_device_update_status,
	.get_brightness = intel_backlight_device_get_brightness,
};

static int intel_backlight_device_register(struct intel_connector *connector)
{
	struct intel_panel *panel = &connector->panel;
	struct backlight_properties props;

	if (WARN_ON(panel->backlight.device))
		return -ENODEV;

	BUG_ON(panel->backlight.max == 0);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = panel->backlight.level;
	props.max_brightness = panel->backlight.max;

	/*
	 * Note: using the same name independent of the connector prevents
	 * registration of multiple backlight devices in the driver.
	 */
	panel->backlight.device =
		backlight_device_register("intel_backlight",
					  connector->base.kdev,
					  connector,
					  &intel_backlight_device_ops, &props);

	if (IS_ERR(panel->backlight.device)) {
		DRM_ERROR("Failed to register backlight: %ld\n",
			  PTR_ERR(panel->backlight.device));
		panel->backlight.device = NULL;
		return -ENODEV;
	}
	return 0;
}

static void intel_backlight_device_unregister(struct intel_connector *connector)
{
	struct intel_panel *panel = &connector->panel;

	if (panel->backlight.device) {
		backlight_device_unregister(panel->backlight.device);
		panel->backlight.device = NULL;
	}
}
#else /* CONFIG_BACKLIGHT_CLASS_DEVICE */
static int intel_backlight_device_register(struct intel_connector *connector)
{
	return 0;
}
static void intel_backlight_device_unregister(struct intel_connector *connector)
{
}
#endif /* CONFIG_BACKLIGHT_CLASS_DEVICE */

/*
 * Note: The setup hooks can't assume pipe is set!
 *
 * XXX: Query mode clock or hardware clock and program PWM modulation frequency
 * appropriately when it's 0. Use VBT and/or sane defaults.
 */
static int bdw_setup_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 pch_ctl1, pch_ctl2, val;

	pch_ctl1 = I915_READ(BLC_PWM_PCH_CTL1);
	panel->backlight.active_low_pwm = pch_ctl1 & BLM_PCH_POLARITY;

	pch_ctl2 = I915_READ(BLC_PWM_PCH_CTL2);
	panel->backlight.max = pch_ctl2 >> 16;
	if (!panel->backlight.max)
		return -ENODEV;

	val = bdw_get_backlight(connector);
	panel->backlight.level = intel_panel_compute_brightness(connector, val);

	panel->backlight.enabled = (pch_ctl1 & BLM_PCH_PWM_ENABLE) &&
		panel->backlight.level != 0;

	return 0;
}

static int pch_setup_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 cpu_ctl2, pch_ctl1, pch_ctl2, val;

	pch_ctl1 = I915_READ(BLC_PWM_PCH_CTL1);
	panel->backlight.active_low_pwm = pch_ctl1 & BLM_PCH_POLARITY;

	pch_ctl2 = I915_READ(BLC_PWM_PCH_CTL2);
	panel->backlight.max = pch_ctl2 >> 16;
	if (!panel->backlight.max)
		return -ENODEV;

	val = pch_get_backlight(connector);
	panel->backlight.level = intel_panel_compute_brightness(connector, val);

	cpu_ctl2 = I915_READ(BLC_PWM_CPU_CTL2);
	panel->backlight.enabled = (cpu_ctl2 & BLM_PWM_ENABLE) &&
		(pch_ctl1 & BLM_PCH_PWM_ENABLE) && panel->backlight.level != 0;

	return 0;
}

static int i9xx_setup_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 ctl, val;

	ctl = I915_READ(BLC_PWM_CTL);

	if (IS_GEN2(dev) || IS_I915GM(dev) || IS_I945GM(dev))
		panel->backlight.combination_mode = ctl & BLM_LEGACY_MODE;

	if (IS_PINEVIEW(dev))
		panel->backlight.active_low_pwm = ctl & BLM_POLARITY_PNV;

	panel->backlight.max = ctl >> 17;
	if (panel->backlight.combination_mode)
		panel->backlight.max *= 0xff;

	if (!panel->backlight.max)
		return -ENODEV;

	val = i9xx_get_backlight(connector);
	panel->backlight.level = intel_panel_compute_brightness(connector, val);

	panel->backlight.enabled = panel->backlight.level != 0;

	return 0;
}

static int i965_setup_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	u32 ctl, ctl2, val;

	ctl2 = I915_READ(BLC_PWM_CTL2);
	panel->backlight.combination_mode = ctl2 & BLM_COMBINATION_MODE;
	panel->backlight.active_low_pwm = ctl2 & BLM_POLARITY_I965;

	ctl = I915_READ(BLC_PWM_CTL);
	panel->backlight.max = ctl >> 16;
	if (panel->backlight.combination_mode)
		panel->backlight.max *= 0xff;

	if (!panel->backlight.max)
		return -ENODEV;

	val = i9xx_get_backlight(connector);
	panel->backlight.level = intel_panel_compute_brightness(connector, val);

	panel->backlight.enabled = (ctl2 & BLM_PWM_ENABLE) &&
		panel->backlight.level != 0;

	return 0;
}

static int vlv_setup_backlight_lpss_pwm(struct intel_connector *connector)
{
	struct intel_panel *panel = &connector->panel;
	u32 val;

	panel->backlight.max = 0xFF;
	val = vlv_get_backlight_lpss_pwm(connector);
	panel->backlight.level =
		intel_panel_compute_brightness(connector, val);
	panel->backlight.enabled =
		lpio_bl_read(0, LPIO_PWM_CTRL) & LPIO_PWM_ENABLE_MASK;

	return 0;
}

static int vlv_setup_backlight_dis_ddi(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &connector->panel;
	struct intel_dp *intel_dp = enc_to_intel_dp(&connector->encoder->base);
	enum pipe pipe;
	u32 ctl, ctl2, val;

	for_each_pipe(pipe) {
		u32 cur_val;

		if (pipe >= PIPE_C)
			continue;

		cur_val = I915_READ(VLV_BLC_PWM_CTL(pipe));

		/* Skip if the modulation freq is already set */
		if (cur_val & ~BACKLIGHT_DUTY_CYCLE_MASK)
			continue;

		cur_val &= BACKLIGHT_DUTY_CYCLE_MASK;
		I915_WRITE(VLV_BLC_PWM_CTL(pipe), (0xf42 << 16) |
				cur_val);
	}

	pipe = vlv_power_sequencer_pipe(intel_dp);

	ctl2 = I915_READ(VLV_BLC_PWM_CTL2(pipe));
	panel->backlight.active_low_pwm = ctl2 & BLM_POLARITY_I965;

	ctl = I915_READ(VLV_BLC_PWM_CTL(pipe));
	panel->backlight.max = ctl >> 16;
	if (!panel->backlight.max)
		return -ENODEV;

	val = _vlv_get_backlight(dev, pipe);
	panel->backlight.level =
		intel_panel_compute_brightness(connector, val);

	panel->backlight.enabled = (ctl2 & BLM_PWM_ENABLE) &&
		panel->backlight.level != 0;

	return 0;
}

static int vlv_setup_mipi_backlight(struct intel_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct intel_panel *panel = &connector->panel;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val;
#ifdef BL_REFER_BATTERY_LEVEL
	int battery_a = read_rsoc(0);
	int battery_b = read_rsoc(1);
#endif

	panel->backlight.max = 0xFF;

	val = vlv_get_mipi_backlight(connector);
	panel->backlight.level = intel_panel_compute_brightness(connector, val);

	if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
		panel->backlight.enabled = lpio_bl_read(0, LPIO_PWM_CTRL) &
			0x80000000;
	} else {
		panel->backlight.enabled =
			(dsi_soc_pmic_readb(PMIC_PWM_EN) & 0x1) &&
			panel->backlight.level != 0;
	}

#if (BLADEIII_10A_INIT_BL_LEVEL > 0)	/* for BladeIII 10A */
	panel->backlight.enabled = true;
#ifdef BL_REFER_BATTERY_LEVEL
	if((battery_a < 3) && (battery_b < 3) ) {
		DRM_INFO("%s: Low Battery [%d, %d]. Use low backlight level.",
			__func__, battery_a, battery_b);
		panel->backlight.level = BLADEIII_10A_LOW_BATTERY_BL_LEVEL;
	} else
#endif
		panel->backlight.level = BLADEIII_10A_INIT_BL_LEVEL;
#endif

	return 0;
}

static int vlv_setup_backlight_ic(struct intel_connector *connector)
{
#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
	struct intel_panel *panel = &connector->panel;
#endif
	u8 transmit_buffer[4] = {0};
	struct i2c_msg msg;

	DRM_INFO("%s: setup backlight IC with I2C command.\n", __func__);

	/* FIXME: need to add judgement for different panel/board */
	msg.addr = BL_IC_I2C_ADDR;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = transmit_buffer;

	read_backlight_id();

	transmit_buffer[0] = LP8557_BL_CMD;
	transmit_buffer[1] = LP8557_BL_OFF;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	msleep(10);

#ifdef BLADE_BACKLIGHT_CONTROL_BY_I2C
	if(get_backlight_controllor_type() == BL_CTRL_BY_I2C) {
		DRM_INFO("%s: setup backlight to %u.\n", __func__, panel->backlight.level);
		transmit_buffer[0] = LP8557_BRIGHTNESS_CTRL;
		transmit_buffer[1] = panel->backlight.level;
		chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);

		DRM_INFO("%s: setup backlight IC control to I2C mode.\n", __func__);
		transmit_buffer[0] = LP8557_CONFIG;
		transmit_buffer[1] = 0x87;
		chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
		msleep(10);
	} else {
		DRM_INFO("%s: setup backlight IC control to PWM mode.\n", __func__);
		transmit_buffer[0] = LP8557_CONFIG;
		transmit_buffer[1] = 0x84;
		chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
		msleep(10);
	}
#else
	DRM_INFO("%s: setup backlight IC control to PWM mode.\n", __func__);
	transmit_buffer[0] = LP8557_CONFIG;
	transmit_buffer[1] = 0x84;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	msleep(10);
#endif

	/* set backlight current */
	transmit_buffer[0] = LP8557_CURRENT_CONFIG;
	transmit_buffer[1] = LP8557_CURRENT_MAX;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	mdelay(10);

	transmit_buffer[0] = LP8557_BL_CMD;
	transmit_buffer[1] = LP8557_BL_ON;
	chv_exec_i2c(BL_IC_I2C_BUS, &msg, 1);
	msleep(10);

	return 0;
}

int intel_panel_setup_backlight(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_panel *panel = &intel_connector->panel;
	int ret;

	if (!dev_priv->vbt.backlight.present) {
		DRM_DEBUG_KMS("native backlight control not available per VBT\n");
		return 0;
	}

	if (dev_priv->vbt.has_mipi)
		ret = dev_priv->display.setup_backlight(intel_connector);
	else {
		/* set level and max in panel struct */
		mutex_lock(&dev_priv->backlight_lock);
		ret = dev_priv->display.setup_backlight(intel_connector);
		mutex_unlock(&dev_priv->backlight_lock);
	}

	if (ret) {
		DRM_DEBUG_KMS("failed to setup backlight for connector %s\n",
			      connector->name);
		return ret;
	}

	intel_backlight_device_register(intel_connector);

	panel->backlight.present = true;

	DRM_INFO("backlight initialized, %s, brightness %u/%u, "
		      "sysfs interface %sregistered\n",
		      panel->backlight.enabled ? "enabled" : "disabled",
		      panel->backlight.level, panel->backlight.max,
		      panel->backlight.device ? "" : "not ");

	return 0;
}

void intel_panel_destroy_backlight(struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_panel *panel = &intel_connector->panel;

	panel->backlight.present = false;
	intel_backlight_device_unregister(intel_connector);
}

/* Set up chip specific backlight functions */
void intel_panel_init_backlight_funcs(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (IS_BROADWELL(dev)) {
		dev_priv->display.setup_backlight = bdw_setup_backlight;
		dev_priv->display.enable_backlight = bdw_enable_backlight;
		dev_priv->display.disable_backlight = pch_disable_backlight;
		dev_priv->display.set_backlight = bdw_set_backlight;
		dev_priv->display.get_backlight = bdw_get_backlight;
	} else if (HAS_PCH_SPLIT(dev)) {
		dev_priv->display.setup_backlight = pch_setup_backlight;
		dev_priv->display.enable_backlight = pch_enable_backlight;
		dev_priv->display.disable_backlight = pch_disable_backlight;
		dev_priv->display.set_backlight = pch_set_backlight;
		dev_priv->display.get_backlight = pch_get_backlight;
	} else if (IS_VALLEYVIEW(dev)) {
		if (dev_priv->vbt.has_mipi) {
			dev_priv->display.setup_backlight =
						vlv_setup_mipi_backlight;
			dev_priv->display.enable_backlight =
						vlv_enable_mipi_backlight;
			dev_priv->display.disable_backlight =
						vlv_disable_mipi_backlight;
			dev_priv->display.set_backlight =
						vlv_set_mipi_backlight;
			dev_priv->display.get_backlight =
						vlv_get_mipi_backlight;
		} else {
			if (dev_priv->vbt.backlight.controller ==
					DISPLAY_DDI_BKL_CNTL) {
				dev_priv->display.setup_backlight =
					vlv_setup_backlight_dis_ddi;
				dev_priv->display.enable_backlight =
					vlv_enable_backlight_dis_ddi;
				dev_priv->display.disable_backlight =
					vlv_disable_backlight_dis_ddi;
				dev_priv->display.set_backlight =
					vlv_set_backlight_dis_ddi;
				dev_priv->display.get_backlight =
					vlv_get_backlight_dis_ddi;
			} else if (dev_priv->vbt.backlight.controller ==
					LPSS_PWM_BKL_CNTL) {
				dev_priv->display.setup_backlight =
					vlv_setup_backlight_lpss_pwm;
				dev_priv->display.enable_backlight =
					vlv_enable_backlight_lpss_pwm;
				dev_priv->display.disable_backlight =
					vlv_disable_backlight_lpss_pwm;
				dev_priv->display.set_backlight =
					vlv_set_backlight_lpss_pwm;
				dev_priv->display.get_backlight =
					vlv_get_backlight_lpss_pwm;
			}
		}
	} else if (IS_GEN4(dev)) {
		dev_priv->display.setup_backlight = i965_setup_backlight;
		dev_priv->display.enable_backlight = i965_enable_backlight;
		dev_priv->display.disable_backlight = i965_disable_backlight;
		dev_priv->display.set_backlight = i9xx_set_backlight;
		dev_priv->display.get_backlight = i9xx_get_backlight;
	} else {
		dev_priv->display.setup_backlight = i9xx_setup_backlight;
		dev_priv->display.enable_backlight = i9xx_enable_backlight;
		dev_priv->display.disable_backlight = i9xx_disable_backlight;
		dev_priv->display.set_backlight = i9xx_set_backlight;
		dev_priv->display.get_backlight = i9xx_get_backlight;
	}
}

int intel_panel_init(struct intel_panel *panel,
		     struct drm_display_mode *fixed_mode,
		     struct drm_display_mode *downclock_mode)
{
	panel->fixed_mode = fixed_mode;
	panel->downclock_mode = downclock_mode;

	return 0;
}

void intel_panel_fini(struct intel_panel *panel)
{
	struct intel_connector *intel_connector =
		container_of(panel, struct intel_connector, panel);

	if (panel->fixed_mode)
		drm_mode_destroy(intel_connector->base.dev, panel->fixed_mode);

	if (panel->downclock_mode)
		drm_mode_destroy(intel_connector->base.dev,
				panel->downclock_mode);
}
