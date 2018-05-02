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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <linux/sysrq.h>
#include <linux/slab.h>
#include "i915_drm.h"
#include "drm_crtc.h"
#include "i915_drv.h"
#include "intel_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"

/*
 * DPST (Display Power Savings Technology) is a power savings features
 * which reduces the backlight while enhancing the image such that the
 * user does not perceive any difference in the image quality. The backlight
 * reduction can provide power savings
 *
 * The DPST IOCTL implemented in this file can be used by the DPST a user-mode
 * module. The IOCTL provides methods to initialize the DPST hardware,
 * manage DPST interrupts, and to apply the new backlight and image enhancement
 * values.
 *
 * The user mode module will initialize the DPST hardware when it starts up.
 * The kernel will notify user mode module of any DPST histogram interrupts.
 * When the user mode module receives a notification of these interrupts, it
 * will query the kernel for all the DPST histogram data. Using this data,
 * the user mode module will calculate new backlight and image enhancement
 * values and provides those values to the kernel to program into the DPST
 * hardware.
 */

static bool
i915_dpst_save_conn_on_edp(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_connector *i_connector = NULL;
	struct drm_connector *d_connector;
	enum pipe new_pipe;
	bool ret = false;

	list_for_each_entry(d_connector, &dev->mode_config.connector_list, head)
	{
		i_connector = to_intel_connector(d_connector);
		if (i_connector->encoder
			&& (i_connector->encoder->type == INTEL_OUTPUT_EDP ||
			i_connector->encoder->type == INTEL_OUTPUT_DSI)) {
			if (i_connector->encoder->base.crtc == NULL)
				continue;
			dev_priv->dpst.connector = i_connector;
			new_pipe = to_intel_crtc(i_connector->encoder->base.crtc)->pipe;
			if (new_pipe != dev_priv->dpst.pipe)
				dev_priv->dpst.pipe_mismatch = true;
			dev_priv->dpst.pipe = new_pipe;
			ret = true;
		}
	}

	/* Sanitize disabled_by_hdmi if we know correct dpst pipe */
	if (ret && i915.enable_dpst_wa && dev_priv->dpst.disabled_by_hdmi
			&& dev_priv->dpst.pipe != PIPE_B)
		dev_priv->dpst.disabled_by_hdmi = false;

	return ret;
}

static bool i915_dpst_get_hdmi_status(struct drm_device *dev)
{
	struct intel_connector *i_connector = NULL;
	struct drm_connector *d_connector;

	list_for_each_entry(d_connector, &dev->mode_config.connector_list, head)
	{
		i_connector = to_intel_connector(d_connector);
		if (i_connector->encoder
			&& (i_connector->encoder->type == INTEL_OUTPUT_HDMI)) {
			return intel_hdmi_live_status(d_connector);
		}
	}

	return false;
}

int i915_dpst_sanitize_wa(struct drm_device *dev, int enable_dpst_wa)
{
	/* This W/A should be enabled only on CHV */
	if (IS_CHERRYVIEW(dev) && enable_dpst_wa)
		return 1;

	return 0;
}

void i915_dpst_wa_action(struct drm_device *dev, bool hdmi_enabled)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool dpst_pipe_known = i915_dpst_save_conn_on_edp(dev);

	if (!hdmi_enabled && dev_priv->dpst.disabled_by_hdmi) {
		/* HDMI display unplugged, so we can re-enable DPST again */
		dev_priv->dpst.disabled_by_hdmi = false;
		i915_dpst_display_on(dev);
	} else if (hdmi_enabled && !dev_priv->dpst.disabled_by_hdmi &&
			dpst_pipe_known && dev_priv->dpst.pipe == PIPE_B) {
		/*
		 * If HDMI is disconnected during sleep, it might not be
		 * detected. We need to check the hdmi live status to avoid
		 * this fake DPST disabling.
		 */
		if (i915_dpst_get_hdmi_status(dev)) {
			/*
			 * Do not use DPST on PIPE_B if a HDMI display
			 * is connected.
			 */
			if (dev_priv->dpst.user_enable)
				i915_dpst_enable_disable(dev, 0);
			dev_priv->dpst.disabled_by_hdmi = true;
		}
	} else if (hdmi_enabled && !dpst_pipe_known) {
		/*
		 * We will make sure to sanitize disabled_by_hdmi when we have
		 * correct DPST pipe information.
		 */
		dev_priv->dpst.disabled_by_hdmi = true;
	}
}

static int
i915_dpst_clear_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(dev_priv->dpst.reg.blm_hist_guard,
			I915_READ(dev_priv->dpst.reg.blm_hist_guard) | HISTOGRAM_EVENT_STATUS);
	return 0;
}

static int
i915_dpst_enable_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 blm_hist_ctl;

	if (dev_priv->dpst.is_video_mode_enabled &&
			!dev_priv->is_video_playing)
		return 0;

	/*
	 * Do not switch on the DPST when it is disabled by HDMI WA
	 * Only HDMI disconnect event should be able to re-enable DPST.
	 */
	if (i915.enable_dpst_wa && dev_priv->dpst.disabled_by_hdmi)
		return 0;

	dev_priv->dpst.enabled = true;
	dev_priv->dpst.blc_adjustment = DPST_MAX_FACTOR ;

	/* Enable histogram logic to collect data */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl |= IE_HISTOGRAM_ENABLE | HSV_INTENSITY_MODE;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	/* Wait for VBLANK since the histogram enabling logic takes affect
	 * at the next vblank */
	intel_wait_for_vblank(dev, dev_priv->dpst.pipe);

	/* Clear pending interrupt bit. Clearing the pending interrupt bit
	 * must be not be done at the same time as enabling the
	 * interrupt. */
	I915_WRITE(dev_priv->dpst.reg.blm_hist_guard,
			I915_READ(dev_priv->dpst.reg.blm_hist_guard) | HISTOGRAM_EVENT_STATUS);

	/* Enable histogram interrupts */
	I915_WRITE(dev_priv->dpst.reg.blm_hist_guard,
			I915_READ(dev_priv->dpst.reg.blm_hist_guard) | HISTOGRAM_INTERRUPT_ENABLE);

	/* DPST interrupt in DE_IER is enabled in irq_postinstall */

	return 0;
}

static int
i915_dpst_disable_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	u32 blm_hist_guard, blm_hist_ctl;

	dev_priv->dpst.enabled = false;

	/* Disable histogram interrupts. It is OK to clear pending interrupts
	 * and disable interrupts at the same time. */
	blm_hist_guard = I915_READ(dev_priv->dpst.reg.blm_hist_guard);
	blm_hist_guard |= HISTOGRAM_EVENT_STATUS; /* clear pending interrupts */
	blm_hist_guard &= ~HISTOGRAM_INTERRUPT_ENABLE;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_guard, blm_hist_guard);

	/* Disable histogram logic */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl &= ~IE_HISTOGRAM_ENABLE;

	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	/* DPST interrupt in DE_IER register is disabled in irq_uninstall */

	return 0;
}

static int
i915_dpst_set_user_enable(struct drm_device *dev, bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = 0;

	dev_priv->dpst.user_enable = enable;

	if (enable) {
		if (!dev_priv->dpst.kernel_disable && !dev_priv->dpst.enabled)
			return i915_dpst_enable_hist_interrupt(dev);
	} else {
		/* User disabling invalidates any saved settings */
		dev_priv->dpst.saved.is_valid = false;

		if (dev_priv->dpst.enabled)
			ret = i915_dpst_disable_hist_interrupt(dev);
	}

	return ret;
}

static int
i915_dpst_apply_luma(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &dev_priv->dpst.connector->panel;

	u32 diet_factor, i;
	u32 blm_hist_ctl;

	/* This is an invalid call if we are disabled by the user
	 * If pipe_mismatch is true, the luma data is calculated from the
	 * histogram from old pipe, ignore it */
	if (!dev_priv->dpst.user_enable
			|| dev_priv->dpst.pipe_mismatch)
		return -EINVAL;

	/* This is not an invalid call if we are disabled by the kernel,
	 * because kernel disabling is transparent to the user and can easily
	 * occur before user has completed in-progress adjustments. If in fact
	 * we are disabled by the kernel, we must store the incoming values for
	 * later restore. Image enhancement values are stored on the hardware,
	 * because they will be safely ignored if the table is not enabled. */

	/* Setup register to access image enhancement value from
	 * index 0.*/
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl |= BIN_REG_FUNCTION_SELECT_IE;
	blm_hist_ctl &= ~BIN_REGISTER_INDEX_MASK;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	/* Program the image enhancement data passed from user mode. */
	for (i = 0; i < DPST_DIET_ENTRY_COUNT; i++) {
		diet_factor = ioctl_data->ie_container.
			dpst_ie_st.factor_present[i] * 0x200 / 10000;
		I915_WRITE(dev_priv->dpst.reg.blm_hist_bin, diet_factor);
	}

	if (dev_priv->dpst.kernel_disable) {
		dev_priv->dpst.saved.is_valid = true;
		dev_priv->dpst.saved.blc_adjustment =
			ioctl_data->ie_container.dpst_blc_factor;
		return 0;
	}

	/* Backlight settings */
	dev_priv->dpst.blc_adjustment =
	ioctl_data->ie_container.dpst_blc_factor;

	/* Avoid warning messages */
	mutex_lock(&dev_priv->backlight_lock);
	i915_dpst_set_brightness(dev, panel->backlight.level);
	mutex_unlock(&dev_priv->backlight_lock);

	/* Enable Image Enhancement Table */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl |= dev_priv->dpst.ie_mod_table_enable |
		ENHANCEMENT_MODE_MULT;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	return 0;
}

static void
i915_dpst_save_luma(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Only save if user mode has indeed applied valid settings which
	 * we determine by checking that the IE mod table was enabled */
	if (!(I915_READ(dev_priv->dpst.reg.blm_hist_ctl) &
				dev_priv->dpst.ie_mod_table_enable))
		return;

	/* IE mod table entries are saved in the hardware even if the table
	 * is disabled, so we only need to save the backlight adjustment */
	dev_priv->dpst.saved.is_valid = true;
	dev_priv->dpst.saved.blc_adjustment = dev_priv->dpst.blc_adjustment;
}

static void
i915_dpst_restore_luma(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &dev_priv->dpst.connector->panel;
	u32 blm_hist_ctl;

	/* Only restore if valid settings were previously saved */
	if (!dev_priv->dpst.saved.is_valid)
		return;

	dev_priv->dpst.blc_adjustment = dev_priv->dpst.saved.blc_adjustment;

	/* Avoid warning messages */
	mutex_lock(&dev_priv->backlight_lock);
	i915_dpst_set_brightness(dev, panel->backlight.level);
	mutex_unlock(&dev_priv->backlight_lock);

	/* IE mod table entries are saved in the hardware even if the table
	 * is disabled, so we only need to re-enable the table */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl |= dev_priv->dpst.ie_mod_table_enable |
		ENHANCEMENT_MODE_MULT;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);
}

static int
i915_dpst_get_bin_data(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 blm_hist_ctl, blm_hist_bin;
	u32 ioctl_type = ioctl_data->dpst_ioctl_type;
	int index;
	struct dpst_histogram_status *hist_stat =
		&(ioctl_data->hist_status);
	struct dpst_histogram_status_legacy *hist_stat_legacy =
		&(ioctl_data->hist_status_legacy);

	/* We may be disabled by request from kernel or user. Kernel mode
	 * disablement is without user mode knowledge. Kernel mode disablement
	 * can occur between the signal to user and user's follow-up call to
	 * retrieve the data, so return the data as usual. User mode
	 * disablement makes this an invalid call, so return error. */
	if (!dev_priv->dpst.enabled && !dev_priv->dpst.user_enable)
		return -EINVAL;
	else if (!dev_priv->dpst.enabled &&
			ioctl_type == DPST_GET_BIN_DATA) {
		/* Convey to user that dpst has been disabled
		 * from kernel. */
		ioctl_data->hist_status.dpst_disable = 1;
		return 0;
	}

	/* Setup register to access bin data from index 0 */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl = blm_hist_ctl & ~(BIN_REGISTER_INDEX_MASK |
						BIN_REG_FUNCTION_SELECT_IE);
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	/* Read all bin data */
	for (index = 0; index < HIST_BIN_COUNT; index++) {
		blm_hist_bin = I915_READ(dev_priv->dpst.reg.blm_hist_bin);

		if (!(blm_hist_bin & BUSY_BIT)) {
			if (ioctl_type == DPST_GET_BIN_DATA)
				hist_stat->histogram_bins.status[index]
				   = blm_hist_bin &
				    dev_priv->dpst.reg.blm_hist_bin_count_mask;
			else
				hist_stat_legacy->histogram_bins.status[index]
				   = blm_hist_bin &
				    dev_priv->dpst.reg.blm_hist_bin_count_mask;
		} else {
			/* Engine is busy. Reset index to 0 to grab
			 * fresh histogram data */
			index = -1;
			blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
			blm_hist_ctl = blm_hist_ctl & ~BIN_REGISTER_INDEX_MASK;
			I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);
		}
	}

	return 0;
}

static u32
i915_dpst_get_resolution(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	struct drm_display_mode *mode;
	u32 resolution = 0;

	/* Get information about current display mode */
	crtc = intel_get_crtc_for_pipe(dev, dev_priv->dpst.pipe);
	if (!crtc)
		return 0;

	mode = intel_crtc_mode_get(dev, crtc);
	if (mode)
		resolution = mode->hdisplay * mode->vdisplay;

	kfree(mode);
	return resolution;
}

static int
i915_dpst_update_registers(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (IS_HASWELL(dev)) {
		dev_priv->dpst.reg.blm_hist_ctl = BLM_HIST_CTL;
		dev_priv->dpst.reg.blm_hist_guard = BLM_HIST_GUARD;
		dev_priv->dpst.reg.blm_hist_bin = BLM_HIST_BIN;
		dev_priv->dpst.reg.blm_hist_bin_count_mask = BIN_COUNT_MASK_4M;
		dev_priv->dpst.ie_mod_table_enable = IE_MOD_TABLE_ENABLE;
	} else if (IS_VALLEYVIEW(dev)) {
		dev_priv->dpst.reg.blm_hist_ctl =
				VLV_BLC_HIST_CTL(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_guard =
				VLV_BLC_HIST_GUARD(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_bin =
				VLV_BLC_HIST_BIN(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_bin_count_mask = BIN_COUNT_MASK_4M;
		dev_priv->dpst.ie_mod_table_enable = VLV_IE_MOD_TABLE_ENABLE;
	} else if (IS_BROADWELL(dev)) {
		dev_priv->dpst.reg.blm_hist_ctl =
				BDW_DPST_CTL_PIPE(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_guard =
				BDW_DPST_GUARD_PIPE(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_bin =
				BDW_DPST_BIN_PIPE(dev_priv->dpst.pipe);
		dev_priv->dpst.reg.blm_hist_bin_count_mask =
				BIN_COUNT_MASK_16M;
		dev_priv->dpst.ie_mod_table_enable = IE_MOD_TABLE_ENABLE;
	} else {
		DRM_ERROR("DPST not supported on this platform\n");
		return -EINVAL;
	}

	return 0;
};

static bool
i915_dpst_update_context(struct  drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 cur_resolution, blm_hist_guard, gb_threshold;

	cur_resolution = i915_dpst_get_resolution(dev);

	if (0 == cur_resolution)
		return false;

	if (dev_priv->dpst.init_image_res != cur_resolution) {
		DRM_ERROR("DPST does not support resolution switch on the fly");
		return false;
	}

	gb_threshold = (DEFAULT_GUARDBAND_VAL * cur_resolution)/1000;
	/* BDW+, threshold will * 4 by hardware automatically*/
	if (IS_BROADWELL(dev))
		gb_threshold /= 4;

	if (0 != i915_dpst_update_registers(dev))
		return false;

	/* Setup guardband delays and threshold */
	blm_hist_guard = I915_READ(dev_priv->dpst.reg.blm_hist_guard);
	blm_hist_guard |= (dev_priv->dpst.gb_delay << 22)
			| gb_threshold;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_guard, blm_hist_guard);

	return true;
}

void
i915_dpst_display_off(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 blm_hist_ctl;

	/* Check if dpst is user enabled*/
	if (!dev_priv->dpst.user_enable)
		return;

	mutex_lock(&dev_priv->dpst.ioctl_lock);
	/* Set the flag to reject all the subsequent DPST ioctls
	 * till the Display is turned on again
	 */
	dev_priv->dpst.display_off = true;

	i915_dpst_disable_hist_interrupt(dev);
	/*
	 * Disabling Image Enhancement bit during display off to avoid
	 * applying incorrect IE upon resume
	 */
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_ctl &= ~(dev_priv->dpst.ie_mod_table_enable);
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);
	mutex_unlock(&dev_priv->dpst.ioctl_lock);

	/* Send a fake signal to user, so that the user can be notified
	 * to reset the dpst context, to avoid any mismatch of blc_adjusment
	 * between user and kernel on resume. */
	i915_dpst_irq_handler(dev, dev_priv->dpst.pipe);
}

void
i915_dpst_display_on(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!dev_priv->dpst.user_enable
			|| !i915_dpst_save_conn_on_edp(dev))
		return;

	mutex_lock(&dev_priv->dpst.ioctl_lock);

	if (i915_dpst_update_context(dev)
			&& !dev_priv->dpst.kernel_disable)
		i915_dpst_enable_hist_interrupt(dev);

	dev_priv->dpst.display_off = false;
	mutex_unlock(&dev_priv->dpst.ioctl_lock);
}

static int
i915_dpst_init(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct pid *cur_pid;

	dev_priv->dpst.signal = ioctl_data->init_data.sig_num;
	dev_priv->dpst.gb_delay = ioctl_data->init_data.gb_delay;
	dev_priv->dpst.pipe_mismatch = false;


	/* Store info needed to talk to user mode */
	cur_pid = get_task_pid(current, PIDTYPE_PID);
	put_pid(dev_priv->dpst.pid);
	dev_priv->dpst.pid = cur_pid;
	dev_priv->dpst.signal = ioctl_data->init_data.sig_num;
	dev_priv->dpst.kernel_disable = true;

	if (!i915_dpst_save_conn_on_edp(dev))
		return -EINVAL;

	ioctl_data->init_data.image_res = i915_dpst_get_resolution(dev);
	dev_priv->dpst.init_image_res = ioctl_data->init_data.image_res;

	/* Re-using the hist_reg_values variable for sending the
	 * video-mode status to kernel from user mode process */
	if (IS_VALLEYVIEW(dev))
		dev_priv->dpst.is_video_mode_enabled =
					ioctl_data->init_data.hist_reg_values;

	if (!i915_dpst_update_context(dev))
		return -EINVAL;

	/* Init is complete so request enablement */
	return i915_dpst_set_user_enable(dev, true);
}

u32
i915_dpst_get_brightness(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &dev_priv->dpst.connector->panel;

	if (!dev_priv->dpst.enabled)
		return 0;

	/* return the last (non-dpst) set backlight level */
	return panel->backlight.level;
}

/* called by multi-process, be cautious to avoid race condition */
void
i915_dpst_set_brightness(struct drm_device *dev, u32 brightness_val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 backlight_level = brightness_val;
	/*
	 * When DPST is enabled only for video mode, we want to phase out when
	 * video ends even though DPST is disabled. The same is true when we
	 * disable dpst as part of pipe B - pipe C workaround.
	 */
	if (!(dev_priv->dpst.enabled || dev_priv->dpst.is_video_mode_enabled
				|| dev_priv->dpst.disabled_by_hdmi))
		return;

	/* Calculate the backlight after it has been reduced by "dpst
	 * blc adjustment" percent . blc_adjustment value is stored
	 * after multiplying by 100, so we have to divide by 100 2nd time
	 * to get to the correct value */
	backlight_level = ((brightness_val *
				dev_priv->dpst.blc_adjustment)/100)/100;
	intel_panel_actually_set_backlight(dev_priv->dpst.connector
			, backlight_level);
}

void
i915_dpst_irq_handler(struct drm_device *dev, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Check if user-mode need to be aware of this interrupt */
	if (pipe != dev_priv->dpst.pipe)
		return;

	/* reset to false when get the interrupt on current pipe */
	dev_priv->dpst.pipe_mismatch = false;

	/* Notify user mode of the interrupt */
	if (dev_priv->dpst.pid != NULL) {
		if (kill_pid_info(dev_priv->dpst.signal, SEND_SIG_FORCED,
							dev_priv->dpst.pid)) {
			put_pid(dev_priv->dpst.pid);
			dev_priv->dpst.pid = NULL;
		}
	}
}

int
i915_dpst_context(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct dpst_initialize_context *ioctl_data = NULL;
	struct drm_i915_private *dev_priv = dev->dev_private;

	int ret = -EINVAL;

	if (!data)
		return -EINVAL;

	if (!I915_HAS_DPST(dev))
		return -EINVAL;

	ioctl_data = (struct dpst_initialize_context *) data;

	/* mode_config lock is required to avoid warning messages in the
	 * backlight change path. And backlight is changed for these 2
	 * types of ioctl only.
	 */
	if (ioctl_data->dpst_ioctl_type == DPST_APPLY_LUMA ||
		ioctl_data->dpst_ioctl_type == DPST_DISABLE)
		drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);

	/* Can be called from multiple usermode, prevent race condition */
	mutex_lock(&dev_priv->dpst.ioctl_lock);

	/* If Display is currently off (could be power gated also),
	 * don't service the ioctls other than GET_BIN_DATA
	 */
	if (dev_priv->dpst.display_off &&
			ioctl_data->dpst_ioctl_type != DPST_GET_BIN_DATA) {
		DRM_DEBUG_KMS("Display is off\n");
		mutex_unlock(&dev_priv->dpst.ioctl_lock);
		if (ioctl_data->dpst_ioctl_type == DPST_APPLY_LUMA ||
			ioctl_data->dpst_ioctl_type == DPST_DISABLE)
			drm_modeset_unlock(&dev->mode_config.connection_mutex);

		return -EINVAL;
	}

	switch (ioctl_data->dpst_ioctl_type) {
	case DPST_ENABLE:
		ret = i915_dpst_set_user_enable(dev, true);
	break;

	case DPST_DISABLE:
		ret = i915_dpst_set_user_enable(dev, false);
	break;

	case DPST_INIT_DATA:
		ret = i915_dpst_init(dev, ioctl_data);
	break;

	case DPST_GET_BIN_DATA_LEGACY:
	case DPST_GET_BIN_DATA:
		ret = i915_dpst_get_bin_data(dev, ioctl_data);
	break;

	case DPST_APPLY_LUMA:
		ret = i915_dpst_apply_luma(dev, ioctl_data);
	break;

	case DPST_RESET_HISTOGRAM_STATUS:
		ret = i915_dpst_clear_hist_interrupt(dev);
	break;

	default:
		DRM_ERROR("Invalid DPST ioctl type\n");
	break;
	}

	mutex_unlock(&dev_priv->dpst.ioctl_lock);
	if (ioctl_data->dpst_ioctl_type == DPST_APPLY_LUMA ||
		ioctl_data->dpst_ioctl_type == DPST_DISABLE)
		drm_modeset_unlock(&dev->mode_config.connection_mutex);

	return ret;
}

int
i915_dpst_set_kernel_disable(struct drm_device *dev, bool disable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_panel *panel = &dev_priv->dpst.connector->panel;
	int ret = 0;

	if (!I915_HAS_DPST(dev))
		return -EINVAL;

	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	mutex_lock(&dev_priv->dpst.ioctl_lock);

	dev_priv->dpst.kernel_disable = disable;

	if (disable && dev_priv->dpst.enabled) {
		i915_dpst_save_luma(dev);
		ret = i915_dpst_disable_hist_interrupt(dev);
		/* Setting blc level to default */
		dev_priv->dpst.blc_adjustment = DPST_MAX_FACTOR;
		mutex_lock(&dev_priv->backlight_lock);
		intel_panel_actually_set_backlight(dev_priv->dpst.connector,
				panel->backlight.level);
		mutex_unlock(&dev_priv->backlight_lock);
	} else if (!disable && dev_priv->dpst.user_enable) {
		ret = i915_dpst_enable_hist_interrupt(dev);
		if (!ret)
			i915_dpst_restore_luma(dev);
	}

	mutex_unlock(&dev_priv->dpst.ioctl_lock);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);

	return ret;
}

int i915_dpst_enable_disable(struct drm_device *dev, unsigned int val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (val == 1 && !dev_priv->dpst.enabled &&
			!dev_priv->dpst.kernel_disable)
		i915_dpst_enable_hist_interrupt(dev);
	else if (val == 0 && (dev_priv->dpst.enabled))
		i915_dpst_disable_hist_interrupt(dev);

	/* Send a fake signal to start the process */
	i915_dpst_irq_handler(dev, dev_priv->dpst.pipe);

	return 0;
}
