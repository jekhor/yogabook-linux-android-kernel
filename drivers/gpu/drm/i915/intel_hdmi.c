/*
 * Copyright 2006 Dave Airlie <airlied@linux.ie>
 * Copyright © 2006-2009 Intel Corporation
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
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/string.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"

#define HDMI_UEVENT_MAX_LENGTH 20

#define LIMIT_BW_MAX_HDISPLAY	1280
#define LIMIT_BW_MAX_VDISPLAY	800

/* CEA Mode 4 - 1280x720@60Hz */
struct drm_display_mode hdmi_fallback_mode = {
	DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER,
	74250,
	1280, 1390, 1430, 1650, 0,
	720, 725, 730, 750, 0,
	DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	.vrefresh = 60
};

static struct drm_device *intel_hdmi_to_dev(struct intel_hdmi *intel_hdmi)
{
	return hdmi_to_dig_port(intel_hdmi)->base.base.dev;
}

static void
assert_hdmi_port_disabled(struct intel_hdmi *intel_hdmi)
{
	struct drm_device *dev = intel_hdmi_to_dev(intel_hdmi);
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t enabled_bits;

	enabled_bits = HAS_DDI(dev) ? DDI_BUF_CTL_ENABLE : SDVO_ENABLE;

	WARN(I915_READ(intel_hdmi->hdmi_reg) & enabled_bits,
	     "HDMI port enabled, expecting disabled\n");
}

struct intel_hdmi *enc_to_intel_hdmi(struct drm_encoder *encoder)
{
	struct intel_digital_port *intel_dig_port =
		container_of(encoder, struct intel_digital_port, base.base);
	return &intel_dig_port->hdmi;
}

static struct intel_hdmi *intel_attached_hdmi(struct drm_connector *connector)
{
	return enc_to_intel_hdmi(&intel_attached_encoder(connector)->base);
}

static u32 g4x_infoframe_index(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_SELECT_AVI;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_SELECT_SPD;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return VIDEO_DIP_SELECT_VENDOR;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 g4x_infoframe_enable(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return VIDEO_DIP_ENABLE_VENDOR;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 hsw_infoframe_enable(enum hdmi_infoframe_type type)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI_HSW;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD_HSW;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return VIDEO_DIP_ENABLE_VS_HSW;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static u32 hsw_infoframe_data_reg(enum hdmi_infoframe_type type,
				  enum transcoder cpu_transcoder,
				  struct drm_i915_private *dev_priv)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		return HSW_TVIDEO_DIP_AVI_DATA(cpu_transcoder);
	case HDMI_INFOFRAME_TYPE_SPD:
		return HSW_TVIDEO_DIP_SPD_DATA(cpu_transcoder);
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return HSW_TVIDEO_DIP_VS_DATA(cpu_transcoder);
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", type);
		return 0;
	}
}

static void g4x_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(VIDEO_DIP_CTL);
	int i;

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(VIDEO_DIP_CTL, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VIDEO_DIP_DATA, *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(VIDEO_DIP_DATA, 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(VIDEO_DIP_CTL, val);
	POSTING_READ(VIDEO_DIP_CTL);
}

static void ibx_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void cpt_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	/* The DIP control register spec says that we need to update the AVI
	 * infoframe without clearing its enable bit */
	if (type != HDMI_INFOFRAME_TYPE_AVI)
		val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void vlv_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int i, reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(type);

	val &= ~g4x_infoframe_enable(type);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(type);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void hsw_write_infoframe(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	u32 ctl_reg = HSW_TVIDEO_DIP_CTL(intel_crtc->config.cpu_transcoder);
	u32 data_reg;
	int i;
	u32 val = I915_READ(ctl_reg);

	data_reg = hsw_infoframe_data_reg(type,
					  intel_crtc->config.cpu_transcoder,
					  dev_priv);
	if (data_reg == 0)
		return;

	val &= ~hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(data_reg + i, *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(data_reg + i, 0);
	mmiowb();

	val |= hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);
	POSTING_READ(ctl_reg);
}

/*
 * The data we write to the DIP data buffer registers is 1 byte bigger than the
 * HDMI infoframe size because of an ECC/reserved byte at position 3 (starting
 * at 0). It's also a byte used by DisplayPort so the same DIP registers can be
 * used for both technologies.
 *
 * DW0: Reserved/ECC/DP | HB2 | HB1 | HB0
 * DW1:       DB3       | DB2 | DB1 | DB0
 * DW2:       DB7       | DB6 | DB5 | DB4
 * DW3: ...
 *
 * (HB is Header Byte, DB is Data Byte)
 *
 * The hdmi pack() functions don't know about that hardware specific hole so we
 * trick them by giving an offset into the buffer and moving back the header
 * bytes by one.
 */
static void intel_write_infoframe(struct drm_encoder *encoder,
				  union hdmi_infoframe *frame)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	uint8_t buffer[VIDEO_DIP_DATA_SIZE];
	ssize_t len;

	/* see comment above for the reason for this offset */
	len = hdmi_infoframe_pack(frame, buffer + 1, sizeof(buffer) - 1);
	if (len < 0)
		return;

	/* Insert the 'hole' (see big comment above) at position 3 */
	buffer[0] = buffer[1];
	buffer[1] = buffer[2];
	buffer[2] = buffer[3];
	buffer[3] = 0;
	len++;

	intel_hdmi->write_infoframe(encoder, frame->any.type, buffer, len);
}

static void intel_hdmi_compute_color_range(struct intel_hdmi *intel_hdmi,
					struct drm_display_mode *mode)
{
	if (intel_hdmi->color_range_auto) {
		/* See CEA-861-E - 5.1 Default Encoding Parameters */
		if (intel_hdmi->has_hdmi_sink &&
		    drm_match_cea_mode(mode) > 1)
			intel_hdmi->color_range = HDMI_COLOR_RANGE_16_235;
		else
			intel_hdmi->color_range = 0;
	}
}

static void intel_hdmi_set_avi_infoframe(struct drm_encoder *encoder,
					 struct drm_display_mode *adjusted_mode)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	union hdmi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame.avi,
						       adjusted_mode);
	if (ret < 0) {
		DRM_ERROR("couldn't fill AVI infoframe\n");
		return;
	}

	/* If the EDID mentions color range to be selectable, then
	 * compute color_range for the incoming mode and set in AVI
	 * infoframe accordingly.
	*/
	if (intel_hdmi->rgb_quant_range_selectable) {
		intel_hdmi_compute_color_range(intel_hdmi, adjusted_mode);

		if (intel_hdmi->color_range)
			frame.avi.quantization_range =
				HDMI_QUANTIZATION_RANGE_LIMITED;
		else
			frame.avi.quantization_range =
				HDMI_QUANTIZATION_RANGE_FULL;
	}

	intel_write_infoframe(encoder, &frame);
}

static void intel_hdmi_set_spd_infoframe(struct drm_encoder *encoder)
{
	union hdmi_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame.spd, "Intel", "Integrated gfx");
	if (ret < 0) {
		DRM_ERROR("couldn't fill SPD infoframe\n");
		return;
	}

	frame.spd.sdi = HDMI_SPD_SDI_PC;

	intel_write_infoframe(encoder, &frame);
}

static void
intel_hdmi_set_hdmi_infoframe(struct drm_encoder *encoder,
			      struct drm_display_mode *adjusted_mode)
{
	union hdmi_infoframe frame;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame.vendor.hdmi,
							  adjusted_mode);
	if (ret < 0)
		return;

	intel_write_infoframe(encoder, &frame);
}

static void g4x_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	u32 reg = VIDEO_DIP_CTL;
	u32 val = I915_READ(reg);
	u32 port = VIDEO_DIP_PORT(intel_dig_port->port);

	assert_hdmi_port_disabled(intel_hdmi);

	/* If the registers were not initialized yet, they might be zeroes,
	 * which means we're selecting the AVI DIP and we're setting its
	 * frequency to once. This seems to really confuse the HW and make
	 * things stop working (the register spec says the AVI always needs to
	 * be sent every VSync). So here we avoid writing to the register more
	 * than we need and also explicitly select the AVI DIP and explicitly
	 * set its frequency to every VSync. Avoiding to write it twice seems to
	 * be enough to solve the problem, but being defensive shouldn't hurt us
	 * either. */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!enable) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~VIDEO_DIP_ENABLE_VENDOR;

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
	intel_hdmi_set_hdmi_infoframe(encoder, adjusted_mode);
}

static void ibx_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);
	u32 port = VIDEO_DIP_PORT(intel_dig_port->port);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!enable) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
	intel_hdmi_set_hdmi_infoframe(encoder, adjusted_mode);
}

static void cpt_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!enable) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~(VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI);
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	/* Set both together, unset both together: see the spec. */
	val |= VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
	intel_hdmi_set_hdmi_infoframe(encoder, adjusted_mode);
}

static void vlv_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);
	u32 port = VIDEO_DIP_PORT(intel_dig_port->port);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!enable) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_AVI | VIDEO_DIP_ENABLE_VENDOR |
		 VIDEO_DIP_ENABLE_GAMUT | VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
	intel_hdmi_set_hdmi_infoframe(encoder, adjusted_mode);
}

static void hsw_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = HSW_TVIDEO_DIP_CTL(intel_crtc->config.cpu_transcoder);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	if (!enable) {
		I915_WRITE(reg, 0);
		POSTING_READ(reg);
		return;
	}

	val &= ~(VIDEO_DIP_ENABLE_VSC_HSW | VIDEO_DIP_ENABLE_GCP_HSW |
		 VIDEO_DIP_ENABLE_VS_HSW | VIDEO_DIP_ENABLE_GMP_HSW);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
	intel_hdmi_set_hdmi_infoframe(encoder, adjusted_mode);
}

static void intel_hdmi_prepare(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_display_mode *adjusted_mode = &crtc->config.adjusted_mode;
	u32 hdmi_val;

	hdmi_val = SDVO_ENCODING_HDMI;
	if (!HAS_PCH_SPLIT(dev))
		hdmi_val |= intel_hdmi->color_range;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
		hdmi_val |= SDVO_VSYNC_ACTIVE_HIGH;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
		hdmi_val |= SDVO_HSYNC_ACTIVE_HIGH;

	if (crtc->config.pipe_bpp > 24)
		hdmi_val |= HDMI_COLOR_FORMAT_12bpc;
	else
		hdmi_val |= SDVO_COLOR_FORMAT_8bpc;

	if (crtc->config.has_hdmi_sink)
		hdmi_val |= HDMI_MODE_SELECT_HDMI;

	if (crtc->config.has_audio) {
		WARN_ON(!crtc->config.has_hdmi_sink);
		DRM_DEBUG_DRIVER("Enabling HDMI audio on pipe %c\n",
				 pipe_name(crtc->pipe));
		hdmi_val |= SDVO_AUDIO_ENABLE;
		intel_write_eld(&encoder->base, adjusted_mode);
	}

	if (HAS_PCH_CPT(dev))
		hdmi_val |= SDVO_PIPE_SEL_CPT(crtc->pipe);
	else if (IS_CHERRYVIEW(dev))
		hdmi_val |= SDVO_PIPE_SEL_CHV(crtc->pipe);
	else
		hdmi_val |= SDVO_PIPE_SEL(crtc->pipe);

	I915_WRITE(intel_hdmi->hdmi_reg, hdmi_val);
	POSTING_READ(intel_hdmi->hdmi_reg);
}

static bool intel_hdmi_get_hw_state(struct intel_encoder *encoder,
				    enum pipe *pipe)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	enum intel_display_power_domain power_domain;
	u32 tmp;

	power_domain = intel_display_port_power_domain(encoder);
	if (!intel_display_power_enabled(dev_priv, power_domain))
		return false;

	tmp = I915_READ(intel_hdmi->hdmi_reg);

	if (!(tmp & SDVO_ENABLE))
		return false;

	if (HAS_PCH_CPT(dev))
		*pipe = PORT_TO_PIPE_CPT(tmp);
	else if (IS_CHERRYVIEW(dev))
		*pipe = SDVO_PORT_TO_PIPE_CHV(tmp);
	else
		*pipe = PORT_TO_PIPE(tmp);

	return true;
}

static void intel_hdmi_get_config(struct intel_encoder *encoder,
				  struct intel_crtc_config *pipe_config)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	u32 tmp, flags = 0;
	int dotclock;

	tmp = I915_READ(intel_hdmi->hdmi_reg);

	if (tmp & SDVO_HSYNC_ACTIVE_HIGH)
		flags |= DRM_MODE_FLAG_PHSYNC;
	else
		flags |= DRM_MODE_FLAG_NHSYNC;

	if (tmp & SDVO_VSYNC_ACTIVE_HIGH)
		flags |= DRM_MODE_FLAG_PVSYNC;
	else
		flags |= DRM_MODE_FLAG_NVSYNC;

	if (tmp & HDMI_MODE_SELECT_HDMI)
		pipe_config->has_hdmi_sink = true;

	if (tmp & SDVO_AUDIO_ENABLE)
		pipe_config->has_audio = true;

	pipe_config->adjusted_mode.flags |= flags;

	if ((tmp & SDVO_COLOR_FORMAT_MASK) == HDMI_COLOR_FORMAT_12bpc)
		dotclock = pipe_config->port_clock * 2 / 3;
	else
		dotclock = pipe_config->port_clock;

	if (HAS_PCH_SPLIT(dev_priv->dev))
		ironlake_check_encoder_dotclock(pipe_config, dotclock);

	pipe_config->adjusted_mode.crtc_clock = dotclock;
}

static void intel_enable_hdmi(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	u32 temp;
	u32 enable_bits = SDVO_ENABLE;

	if (intel_crtc->config.has_audio)
		enable_bits |= SDVO_AUDIO_ENABLE;

	temp = I915_READ(intel_hdmi->hdmi_reg);

	/* HW workaround for IBX, we need to move the port to transcoder A
	 * before disabling it, so restore the transcoder select bit here. */
	if (HAS_PCH_IBX(dev))
		enable_bits |= SDVO_PIPE_SEL(intel_crtc->pipe);

	/* HW workaround, need to toggle enable bit off and on for 12bpc, but
	 * we do this anyway which shows more stable in testing.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp & ~SDVO_ENABLE);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}

	temp |= enable_bits;

	I915_WRITE(intel_hdmi->hdmi_reg, temp);
	POSTING_READ(intel_hdmi->hdmi_reg);

	/* HW workaround, need to write this twice for issue that may result
	 * in first write getting masked.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}
}

static void vlv_enable_hdmi(struct intel_encoder *encoder)
{
}

static void intel_disable_hdmi(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	u32 temp;
	u32 enable_bits = SDVO_ENABLE | SDVO_AUDIO_ENABLE;

	temp = I915_READ(intel_hdmi->hdmi_reg);

	/* HW workaround for IBX, we need to move the port to transcoder A
	 * before disabling it. */
	if (HAS_PCH_IBX(dev)) {
		struct drm_crtc *crtc = encoder->base.crtc;
		int pipe = crtc ? to_intel_crtc(crtc)->pipe : -1;

		if (temp & SDVO_PIPE_B_SELECT) {
			temp &= ~SDVO_PIPE_B_SELECT;
			I915_WRITE(intel_hdmi->hdmi_reg, temp);
			POSTING_READ(intel_hdmi->hdmi_reg);

			/* Again we need to write this twice. */
			I915_WRITE(intel_hdmi->hdmi_reg, temp);
			POSTING_READ(intel_hdmi->hdmi_reg);

			/* Transcoder selection bits only update
			 * effectively on vblank. */
			if (crtc)
				intel_wait_for_vblank(dev, pipe);
			else
				msleep(50);
		}
	}

	/* HW workaround, need to toggle enable bit off and on for 12bpc, but
	 * we do this anyway which shows more stable in testing.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp & ~SDVO_ENABLE);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}

	temp &= ~enable_bits;

	I915_WRITE(intel_hdmi->hdmi_reg, temp);
	POSTING_READ(intel_hdmi->hdmi_reg);

	/* HW workaround, need to write this twice for issue that may result
	 * in first write getting masked.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->hdmi_reg, temp);
		POSTING_READ(intel_hdmi->hdmi_reg);
	}
}

static int hdmi_portclock_limit(struct intel_hdmi *hdmi, bool respect_dvi_limit)
{
	struct drm_device *dev = intel_hdmi_to_dev(hdmi);
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool is_chv_t3 = IS_CHERRYVIEW(dev) &&
			((dev_priv->dev->pdev->revision &
				PCI_CHV_REV_ID_PACKAGE_TYPE_MASK) ==
					PCI_CHV_REV_ID_PACKAGE_TYPE_T3);

	if ((respect_dvi_limit && !hdmi->has_hdmi_sink) || IS_G4X(dev))
		return 165000;
	else if (IS_HASWELL(dev) || (INTEL_INFO(dev)->gen >= 8 && !is_chv_t3))
		return 300000;
	else
		return 225000;
}

static enum drm_mode_status
intel_hdmi_mode_valid(struct drm_connector *connector,
		      struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;
	if (mode->clock > hdmi_portclock_limit(intel_attached_hdmi(connector),
					       true))
		return MODE_CLOCK_HIGH;
	if (mode->clock < 20000)
		return MODE_CLOCK_LOW;

	/*
	 * WAR
	 * For limitation in memory chanel bandwidth.
	 */
	if ((i915.limitbw) && (IS_VALLEYVIEW(dev))) {
		if (mode->vdisplay > LIMIT_BW_MAX_VDISPLAY)
			return MODE_BAD_VVALUE;
		if (mode->hdisplay > LIMIT_BW_MAX_HDISPLAY)
			return MODE_BAD_HVALUE;
	}

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	return MODE_OK;
}

static bool hdmi_12bpc_possible(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct intel_encoder *encoder;
	int count = 0, count_hdmi = 0;

	if (!HAS_PCH_SPLIT(dev))
		return false;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, base.head) {
		if (encoder->new_crtc != crtc)
			continue;

		count_hdmi += encoder->type == INTEL_OUTPUT_HDMI;
		count++;
	}

	/*
	 * HDMI 12bpc affects the clocks, so it's only possible
	 * when not cloning with other encoder types.
	 */
	return count_hdmi > 0 && count_hdmi == count;
}

bool intel_hdmi_compute_config(struct intel_encoder *encoder,
			       struct intel_crtc_config *pipe_config)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_display_mode *adjusted_mode = &pipe_config->adjusted_mode;
	int clock_12bpc = pipe_config->adjusted_mode.crtc_clock * 3 / 2;
	int portclock_limit = hdmi_portclock_limit(intel_hdmi, false);
	int desired_bpp;

	pipe_config->has_hdmi_sink = intel_hdmi->has_hdmi_sink;

	intel_hdmi_compute_color_range(intel_hdmi, adjusted_mode);

	if (intel_hdmi->color_range)
		pipe_config->limited_color_range = true;

	if (HAS_PCH_SPLIT(dev) && !HAS_DDI(dev))
		pipe_config->has_pch_encoder = true;

	if (pipe_config->has_hdmi_sink && intel_hdmi->has_audio)
		pipe_config->has_audio = true;

	/*
	 * HDMI is either 12 or 8, so if the display lets 10bpc sneak
	 * through, clamp it down. Note that g4x/vlv don't support 12bpc hdmi
	 * outputs. We also need to check that the higher clock still fits
	 * within limits.
	 */
	if (pipe_config->pipe_bpp > 8*3 && pipe_config->has_hdmi_sink &&
	    clock_12bpc <= portclock_limit &&
	    hdmi_12bpc_possible(encoder->new_crtc)) {
		DRM_DEBUG_KMS("picking bpc to 12 for HDMI output\n");
		desired_bpp = 12*3;

		/* Need to adjust the port link by 1.5x for 12bpc. */
		pipe_config->port_clock = clock_12bpc;
	} else {
		DRM_DEBUG_KMS("picking bpc to 8 for HDMI output\n");
		desired_bpp = 8*3;
	}

	if (!pipe_config->bw_constrained) {
		DRM_DEBUG_KMS("forcing pipe bpc to %i for HDMI\n", desired_bpp);
		pipe_config->pipe_bpp = desired_bpp;
	}

	if (adjusted_mode->crtc_clock > portclock_limit) {
		DRM_DEBUG_KMS("too high HDMI clock, rejecting mode\n");
		return false;
	}

	return true;
}

static bool vlv_hdmi_live_status(struct drm_device *dev,
			struct intel_hdmi *intel_hdmi)
{
	u32 bit = 0;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_digital_port *intel_dig_port =
					hdmi_to_dig_port(intel_hdmi);

	DRM_DEBUG_KMS("Reading Live status");
	switch (intel_dig_port->port) {
	case PORT_B:
		bit = HDMIB_HOTPLUG_LIVE_STATUS;
		break;
	case PORT_C:
		bit = HDMIC_HOTPLUG_LIVE_STATUS;
		break;
	case PORT_D:
		bit = HDMID_HOTPLUG_LIVE_STATUS;
		break;
	default:
		DRM_ERROR("No valid HDMI port\n");
	}

	/* Return results of connector connection status */
	return I915_READ(PORT_HOTPLUG_STAT) & bit;
}

/*
 * intel_hdmi_live_status: detect live status of HDMI
 * if device is Gen 7 and above, read the live status reg
 * else, do not block the detection, return true
 */
bool intel_hdmi_live_status(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);

	if (INTEL_INFO(dev)->gen > 6) {

		/* Todo: Implement for other Gen7 and above archs */
		if (IS_VALLEYVIEW(dev))
			return vlv_hdmi_live_status(dev, intel_hdmi);
	}
	return true;
}

/*
 * intel_hdmi_send_uevent: inform usespace about an event
 */
void intel_hdmi_send_uevent(struct drm_device *dev, char *uevent)
{
	char *envp[] = {uevent, NULL};

	/* Notify usp the change */
	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
}

/* Read DDC and get EDID */
struct edid *intel_hdmi_get_edid(struct drm_connector *connector, bool force)
{
	bool current_state = false;
	bool saved_state = false;
	struct edid *new_edid = NULL;
	struct i2c_adapter *adapter = NULL;
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	u32 hotplug_status = dev_priv->hotplug_status;
	enum port hdmi_port = hdmi_to_dig_port(intel_hdmi)->port;
	unsigned int retry = HDMI_EDID_RETRY_COUNT;

	if (!intel_hdmi) {
		DRM_ERROR("Invalid input to get hdmi\n");
		return NULL;
	}

	/* Get the saved status from top half */
	saved_state = hotplug_status &
		(1 << (HDMI_LIVE_STATUS_BASE - hdmi_port));

	/*
	 * A few monitors are slow to respond on EDID and
	 * live status, So read live status multiple times
	 * within a max delay of 30ms
	 */
	do {
		mdelay(HDMI_LIVE_STATUS_DELAY_STEP);
		current_state = intel_hdmi_live_status(connector);
		if (current_state)
			break;
	} while (retry--);

	if (current_state != saved_state)
		DRM_DEBUG_DRIVER("Saved HDMI status != current status\n");

	/* Read EDID if live status or saved status is up, or we are forced */
	if (current_state || saved_state || force) {

		adapter = intel_gmbus_get_adapter(dev_priv,
					intel_hdmi->ddc_bus);
		if (!adapter) {
			DRM_ERROR("Get_hdmi cant get adapter\n");
			return NULL;
		}

		/*
		 * A few monitors issue EDID after some delay, so give them
		 * a few chances but no longer than 30ms
		 *
		 * Force bit will be at bootup,
		 * decrease retry times to optimize boot progress
		 */
		retry = force ? 1 : 3;
READ_EDID:
		new_edid = drm_get_edid(connector, adapter);
		if (!new_edid) {
			if (retry--) {
				mdelay(HDMI_LIVE_STATUS_DELAY_STEP);
				goto READ_EDID;
			}

			if (!force)
				DRM_ERROR("Get_hdmi cant read edid\n");
			return NULL;
		}

		DRM_DEBUG_KMS("Live status up, got EDID");
	}

	return new_edid;
}

/*
 * Encoder's Hot plug function
 * Caller must hold mode_config mutex
 * Read EDID based on Live status register
 */
void intel_hdmi_hot_plug(struct intel_encoder *intel_encoder)
{
	struct drm_encoder *encoder = &intel_encoder->base;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector = NULL;
	struct edid *edid = NULL;
	bool need_event = false;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe = 0;

	connector = &intel_hdmi->attached_connector->base;

	/* check if simulate is in progress for current port */
	if (dev_priv->simulate_dp_in_progress & intel_encoder->hpd_pin) {
		DRM_DEBUG_KMS("Simulate DP disconnect in progress\n");
		return;
	}

	/*
	 * We are here, means there is a HDMI hot-plug
	 * Lets try to get EDID
	 */
	edid = intel_hdmi_get_edid(connector, false);

	if (edid) {
		if (connector->status == connector_status_connected) {
			DRM_DEBUG_DRIVER("Hdmi: Monitor connected\n");
			need_event = true;
		}
	} else {
		if (connector->status == connector_status_disconnected) {
			DRM_DEBUG_DRIVER("Hdmi: Monitor disconnected\n");
			if (intel_encoder->type == INTEL_OUTPUT_HDMI &&
					to_intel_crtc(encoder->crtc)) {
				pipe = to_intel_crtc(encoder->crtc)->pipe;
				dev_priv->gamma_enabled[pipe] = false;
				dev_priv->csc_enabled[pipe] = false;
			}
			need_event = true;
		}
	}

	/*
	 * need_event
	 * This check is required for HDMI compliance when a few
	 * analyzers are capable of generating on-the-fly EDID,
	 * and they just send a couple of connect and disconnect
	 * events on EDID change.
	 * Consider this case:
	 * 1. HDMI connect event with EDID 1 with mode 1
	 * 2. Bottom half calls hot_plug() and detect, connector
	 * status = connected
	 * 3. EDID switch to test EDID, first disconnect event
	 * (This is smaller in duration)
	 * 4. Bottom half calls hot_plug()
	 * 5. By the time hot_plug gets schedules, connect call comes,
	 * with new EDID
	 * 6. Bottom half calls detect() which reports status = connected
	 * again
	 * 7. Bottom half checks previous status = current status =
	 * connected so no event sent to userspace.
	 * 8. In this way, the usp is never informed about EDID change,
	 * so HDMI tests fail So if we are in HDMI hot_plug, there is
	 * some event
	 */
	if (need_event) {
		DRM_DEBUG_DRIVER("Sending self event");
		intel_hdmi_send_uevent(dev, "HOTPLUG=1");
	}

	/* Update EDID, kfree is NULL protected */
	kfree(intel_hdmi->edid);
	intel_hdmi->edid = edid;
}

static enum drm_connector_status
intel_hdmi_detect(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct intel_digital_port *intel_dig_port =
		hdmi_to_dig_port(intel_hdmi);
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct edid *edid = NULL;
	enum intel_display_power_domain power_domain;
	enum drm_connector_status status = connector_status_disconnected;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n",
		      connector->base.id, connector->name);

	power_domain = intel_display_port_power_domain(intel_encoder);
	intel_display_power_get(dev_priv, power_domain);

	/* If its force detection, dont read EDID again */
	if (force) {
		status = connector->status;
		goto det_out;
	}

	intel_hdmi->has_hdmi_sink = false;
	intel_hdmi->has_audio = false;
	intel_hdmi->rgb_quant_range_selectable = false;

	edid = intel_hdmi->edid;

	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL) {
			status = connector_status_connected;
			if (intel_hdmi->force_audio != HDMI_AUDIO_OFF_DVI)
				intel_hdmi->has_hdmi_sink =
						drm_detect_hdmi_monitor(edid);
			intel_hdmi->has_audio = drm_detect_monitor_audio(edid);
			intel_hdmi->rgb_quant_range_selectable =
				drm_rgb_quant_range_selectable(edid);

			if (I915_HAS_DPST(dev) && i915.enable_dpst_wa)
				i915_dpst_wa_action(dev, true);
			DRM_DEBUG_DRIVER("Got edid, HDMI connected\n");
		} else {
			DRM_ERROR("No digital form EDID? Using stored one\n");
			goto det_out;
		}
	} else {
		DRM_DEBUG_DRIVER("No edid, HDMI disconnected\n");
		status = connector_status_disconnected;
		intel_cleanup_modes(connector);
		intel_hdmi->edid_mode_count = 0;
	}

	if (status == connector_status_connected) {
		/*
		* If HDMI status is conencted, the event to audio will be
		* sent on basis of current audio status,
		* but if its disconnected, the
		* status will be sent based on previous audio status
		*/
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
		if ((status != i915_hdmi_state) && (IS_VALLEYVIEW(dev))) {
			if (intel_hdmi->has_audio)
				intel_hdmi->notify_had = 1;
			i915_hdmi_state = status;
		}
#endif
		if (intel_hdmi->force_audio != HDMI_AUDIO_AUTO)
			intel_hdmi->has_audio =
				(intel_hdmi->force_audio == HDMI_AUDIO_ON);
		intel_encoder->type = INTEL_OUTPUT_HDMI;
	} else if (dev_priv->audio_port == intel_dig_port) {
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
		if ((status != i915_hdmi_state) && (IS_VALLEYVIEW(dev))) {
#ifdef CONFIG_EXTCON
			if (strlen(dev_priv->hotplug_switch.name) != 0)
				extcon_set_state(
				&dev_priv->hotplug_switch, 0);
#endif
			chv_set_lpe_audio_reg_pipe(dev, INTEL_OUTPUT_HDMI,
					intel_dig_port->port);
			/* Send a disconnect event to audio */
			mid_hdmi_audio_signal_event(dev_priv->dev,
				HAD_EVENT_HOT_UNPLUG);
			i915_hdmi_state = status;
		}
#endif
		dev_priv->audio_port = NULL;
	}

det_out:
	intel_display_power_put(dev_priv, power_domain);

	return status;
}

static int intel_hdmi_get_modes(struct drm_connector *connector)
{
	struct intel_encoder *intel_encoder = intel_attached_encoder(connector);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&intel_encoder->base);
	struct intel_digital_port *intel_dig_port =
				hdmi_to_dig_port(intel_hdmi);
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	enum intel_display_power_domain power_domain;
	struct edid *edid = NULL;
	int ret = 0;

	/* We should parse the EDID data and find out if it's an HDMI sink so
	 * we can send audio to it.
	 */

	power_domain = intel_display_port_power_domain(intel_encoder);
	intel_display_power_get(dev_priv, power_domain);

	/* No need to read modes if no connection */
	if ((connector->status != connector_status_connected) &&
		(dev_priv->audio_port == intel_dig_port)) {
#ifdef CONFIG_EXTCON
		if (strlen(dev_priv->hotplug_switch.name) != 0)
			extcon_set_state(&dev_priv->hotplug_switch, 0);
#endif
		dev_priv->audio_port = NULL;
		goto e_out;
	}

	DRM_DEBUG_DRIVER("Reading modes from EDID");

	/*
	 * EDID was saved in detect, re-use that if available, avoid
	 * reading EDID everytime. If __unlikely(EDID not available),
	 * read now
	 */
	edid =  intel_hdmi->edid;
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		if (!dev_priv->audio_port) {
			dev_priv->audio_port = intel_dig_port;
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
			drm_edid_to_eld(connector, edid);
			if (intel_hdmi->notify_had) {
				chv_set_lpe_audio_reg_pipe(dev, INTEL_OUTPUT_HDMI,
						intel_dig_port->port);
				hdmi_get_eld(connector->eld);
#ifdef CONFIG_EXTCON
				if (strlen(dev_priv->hotplug_switch.name) != 0) {
					extcon_set_state(
					&dev_priv->hotplug_switch, 1);
				}
#endif
			}
		}
#endif
	}
	intel_hdmi->notify_had = 0;

	/* Update the mode status */
	intel_hdmi->edid_mode_count = ret;

e_out:
	intel_display_power_put(dev_priv, power_domain);

	return ret;
}

static bool
intel_hdmi_detect_audio(struct drm_connector *connector)
{
	struct intel_encoder *intel_encoder = intel_attached_encoder(connector);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&intel_encoder->base);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	enum intel_display_power_domain power_domain;
	struct edid *edid;
	bool has_audio = false;

	power_domain = intel_display_port_power_domain(intel_encoder);
	intel_display_power_get(dev_priv, power_domain);

	edid = drm_get_edid(connector,
			    intel_gmbus_get_adapter(dev_priv,
						    intel_hdmi->ddc_bus));
	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL)
			has_audio = drm_detect_monitor_audio(edid);
		kfree(edid);
	}

	intel_display_power_put(dev_priv, power_domain);

	return has_audio;
}

static int
intel_hdmi_set_property(struct drm_connector *connector,
			struct drm_property *property,
			uint64_t val)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct intel_digital_port *intel_dig_port =
		hdmi_to_dig_port(intel_hdmi);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_encoder *encoder = intel_connector->encoder;
	struct intel_crtc *intel_crtc = encoder->new_crtc;
	int ret;

	ret = drm_object_property_set_value(&connector->base, property, val);
	if (ret)
		return ret;

	if (property == dev_priv->force_audio_property) {
		enum hdmi_force_audio i = val;
		bool has_audio;

		if (i == intel_hdmi->force_audio)
			return 0;

		intel_hdmi->force_audio = i;

		if (i == HDMI_AUDIO_AUTO)
			has_audio = intel_hdmi_detect_audio(connector);
		else
			has_audio = (i == HDMI_AUDIO_ON);

		if (i == HDMI_AUDIO_OFF_DVI)
			intel_hdmi->has_hdmi_sink = 0;

		intel_hdmi->has_audio = has_audio;
		goto done;
	}

	if (property == dev_priv->broadcast_rgb_property) {
		bool old_auto = intel_hdmi->color_range_auto;
		uint32_t old_range = intel_hdmi->color_range;

		switch (val) {
		case INTEL_BROADCAST_RGB_AUTO:
			intel_hdmi->color_range_auto = true;
			break;
		case INTEL_BROADCAST_RGB_FULL:
			intel_hdmi->color_range_auto = false;
			intel_hdmi->color_range = 0;
			break;
		case INTEL_BROADCAST_RGB_LIMITED:
			intel_hdmi->color_range_auto = false;
			intel_hdmi->color_range = HDMI_COLOR_RANGE_16_235;
			break;
		default:
			return -EINVAL;
		}

		if (old_auto == intel_hdmi->color_range_auto &&
		    old_range == intel_hdmi->color_range)
			return 0;

		goto done;
	}

	if (property == dev_priv->scaling_src_size_property) {
		intel_crtc->scaling_src_size = val;
		DRM_DEBUG_DRIVER("src size = %u\n",
			intel_crtc->scaling_src_size);
		return 0;
	}

	if (property == dev_priv->force_ddr_low_freq_property) {
		vlv_force_ddr_low_frequency(dev_priv, val);
		return 0;
	}

	return -EINVAL;

done:
	if (intel_dig_port->base.base.crtc)
		intel_crtc_restore_mode(intel_dig_port->base.base.crtc);

	return 0;
}

static void intel_hdmi_pre_enable(struct intel_encoder *encoder)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct drm_display_mode *adjusted_mode =
		&intel_crtc->config.adjusted_mode;

	intel_hdmi_prepare(encoder);

	intel_hdmi->set_infoframes(&encoder->base,
				   intel_crtc->config.has_hdmi_sink,
				   adjusted_mode);
}

static void vlv_hdmi_pre_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct intel_hdmi *intel_hdmi = &dport->hdmi;
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	struct drm_display_mode *adjusted_mode =
		&intel_crtc->config.adjusted_mode;
	enum dpio_channel port = vlv_dport_to_channel(dport);
	int pipe = intel_crtc->pipe;
	u32 val;

	/* Enable clock channels for this port */
	mutex_lock(&dev_priv->dpio_lock);
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW8(port));
	val = 0;
	if (pipe)
		val |= (1<<21);
	else
		val &= ~(1<<21);
	val |= 0x001000c4;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW8(port), val);

	/* HDMI 1.0V-2dB */
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW5(port), 0);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW4(port), 0x2b245f5f);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW2(port), 0x5578b83a);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW3(port), 0x0c782040);
	vlv_dpio_write(dev_priv, pipe, VLV_TX3_DW4(port), 0x2b247878);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW11(port), 0x00030000);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW9(port), 0x00002000);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW5(port), DPIO_TX_OCALINIT_EN);

	/* Program lane clock */
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW14(port), 0x00760018);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW23(port), 0x00400888);
	mutex_unlock(&dev_priv->dpio_lock);

	intel_hdmi->set_infoframes(&encoder->base,
				   intel_crtc->config.has_hdmi_sink,
				   adjusted_mode);

	intel_enable_hdmi(encoder);

	vlv_wait_port_ready(dev_priv, dport);
}

static void vlv_hdmi_pre_pll_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	enum dpio_channel port = vlv_dport_to_channel(dport);
	int pipe = intel_crtc->pipe;

	intel_hdmi_prepare(encoder);

	/* Program Tx lane resets to default */
	mutex_lock(&dev_priv->dpio_lock);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW0(port),
			 DPIO_PCS_TX_LANE2_RESET |
			 DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW1(port),
			 DPIO_PCS_CLK_CRI_RXEB_EIOS_EN |
			 DPIO_PCS_CLK_CRI_RXDIGFILTSG_EN |
			 (1<<DPIO_PCS_CLK_DATAWIDTH_SHIFT) |
			 DPIO_PCS_CLK_SOFT_RESET);

	/* Fix up inter-pair skew failure */
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW12(port), 0x00750f00);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW11(port), 0x00001500);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW14(port), 0x40400000);

	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW9(port), 0x00002000);
	vlv_dpio_write(dev_priv, pipe, VLV_TX_DW5(port), DPIO_TX_OCALINIT_EN);
	mutex_unlock(&dev_priv->dpio_lock);
}

static void chv_hdmi_pre_pll_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	enum dpio_channel ch = vlv_dport_to_channel(dport);
	enum pipe pipe = intel_crtc->pipe;
	u32 val;

	intel_hdmi_prepare(encoder);

	mutex_lock(&dev_priv->dpio_lock);

	/* program left/right clock distribution */
	if (pipe != PIPE_B) {
		val = vlv_dpio_read(dev_priv, pipe, _CHV_CMN_DW5_CH0);
		val &= ~(CHV_BUFLEFTENA1_MASK | CHV_BUFRIGHTENA1_MASK);
		if (ch == DPIO_CH0)
			val |= CHV_BUFLEFTENA1_FORCE;
		if (ch == DPIO_CH1)
			val |= CHV_BUFRIGHTENA1_FORCE;
		vlv_dpio_write(dev_priv, pipe, _CHV_CMN_DW5_CH0, val);
	} else {
		val = vlv_dpio_read(dev_priv, pipe, _CHV_CMN_DW1_CH1);
		val &= ~(CHV_BUFLEFTENA2_MASK | CHV_BUFRIGHTENA2_MASK);
		if (ch == DPIO_CH0)
			val |= CHV_BUFLEFTENA2_FORCE;
		if (ch == DPIO_CH1)
			val |= CHV_BUFRIGHTENA2_FORCE;
		vlv_dpio_write(dev_priv, pipe, _CHV_CMN_DW1_CH1, val);
	}

	/* program clock channel usage */
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW8(ch));
	val |= CHV_PCS_USEDCLKCHANNEL_OVRRIDE;
	if (pipe != PIPE_B)
		val &= ~CHV_PCS_USEDCLKCHANNEL;
	else
		val |= CHV_PCS_USEDCLKCHANNEL;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW8(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW8(ch));
	val |= CHV_PCS_USEDCLKCHANNEL_OVRRIDE;
	if (pipe != PIPE_B)
		val &= ~CHV_PCS_USEDCLKCHANNEL;
	else
		val |= CHV_PCS_USEDCLKCHANNEL;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW8(ch), val);

	/*
	 * This a a bit weird since generally CL
	 * matches the pipe, but here we need to
	 * pick the CL based on the port.
	 */
	val = vlv_dpio_read(dev_priv, pipe, CHV_CMN_DW19(ch));
	if (pipe != PIPE_B)
		val &= ~CHV_CMN_USEDCLKCHANNEL;
	else
		val |= CHV_CMN_USEDCLKCHANNEL;
	vlv_dpio_write(dev_priv, pipe, CHV_CMN_DW19(ch), val);

	mutex_unlock(&dev_priv->dpio_lock);
}

static void vlv_hdmi_post_disable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	enum dpio_channel port = vlv_dport_to_channel(dport);
	int pipe = intel_crtc->pipe;

	/* Reset lanes to avoid HDMI flicker (VLV w/a) */
	mutex_lock(&dev_priv->dpio_lock);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW0(port), 0x00000000);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS_DW1(port), 0x00e00060);
	mutex_unlock(&dev_priv->dpio_lock);
}

static void chv_hdmi_post_disable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	enum dpio_channel ch = vlv_dport_to_channel(dport);
	enum pipe pipe = intel_crtc->pipe;
	u32 val;

	mutex_lock(&dev_priv->dpio_lock);

	/* Propagate soft reset to data lane reset */
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW1(ch));
	val |= CHV_PCS_REQ_SOFTRESET_EN;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW1(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW1(ch));
	val |= CHV_PCS_REQ_SOFTRESET_EN;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW1(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW0(ch));
	val &= ~(DPIO_PCS_TX_LANE2_RESET | DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW0(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW0(ch));
	val &= ~(DPIO_PCS_TX_LANE2_RESET | DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW0(ch), val);

	mutex_unlock(&dev_priv->dpio_lock);
}

static void chv_hdmi_pre_enable(struct intel_encoder *encoder)
{
	struct intel_digital_port *dport = enc_to_dig_port(&encoder->base);
	struct drm_device *dev = encoder->base.dev;
	struct intel_hdmi *intel_hdmi = &dport->hdmi;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(encoder->base.crtc);
	struct drm_display_mode *adjusted_mode =
		&intel_crtc->config.adjusted_mode;
	enum dpio_channel ch = vlv_dport_to_channel(dport);
	int pipe = intel_crtc->pipe;
	int data, i, stagger;
	u32 val;

	mutex_lock(&dev_priv->dpio_lock);

	/* allow hardware to manage TX FIFO reset source */
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW11(ch));
	val &= ~DPIO_LANEDESKEW_STRAP_OVRD;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW11(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW11(ch));
	val &= ~DPIO_LANEDESKEW_STRAP_OVRD;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW11(ch), val);

	/* Deassert soft data lane reset*/
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW1(ch));
	val |= CHV_PCS_REQ_SOFTRESET_EN;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW1(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW1(ch));
	val |= CHV_PCS_REQ_SOFTRESET_EN;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW1(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW0(ch));
	val |= (DPIO_PCS_TX_LANE2_RESET | DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW0(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW0(ch));
	val |= (DPIO_PCS_TX_LANE2_RESET | DPIO_PCS_TX_LANE1_RESET);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW0(ch), val);

	/* Program Tx latency optimal setting */
	for (i = 0; i < 4; i++) {
		/* Set the upar bit */
		data = (i == 1) ? 0x0 : 0x1;
		vlv_dpio_write(dev_priv, pipe, CHV_TX_DW14(ch, i),
				data << DPIO_UPAR_SHIFT);
	}

	/* Data lane stagger programming */
	if (intel_crtc->config.port_clock > 270000)
		stagger = 0x18;
	else if (intel_crtc->config.port_clock > 135000)
		stagger = 0xd;
	else if (intel_crtc->config.port_clock > 67500)
		stagger = 0x7;
	else if (intel_crtc->config.port_clock > 33750)
		stagger = 0x4;
	else
		stagger = 0x2;

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW11(ch));
	val |= DPIO_TX2_STAGGER_MASK(0x1f);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW11(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW11(ch));
	val |= DPIO_TX2_STAGGER_MASK(0x1f);
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW11(ch), val);

	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW12(ch),
		       DPIO_LANESTAGGER_STRAP(stagger) |
		       DPIO_LANESTAGGER_STRAP_OVRD |
		       DPIO_TX1_STAGGER_MASK(0x1f) |
		       DPIO_TX1_STAGGER_MULT(6) |
		       DPIO_TX2_STAGGER_MULT(0));

	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW12(ch),
		       DPIO_LANESTAGGER_STRAP(stagger) |
		       DPIO_LANESTAGGER_STRAP_OVRD |
		       DPIO_TX1_STAGGER_MASK(0x1f) |
		       DPIO_TX1_STAGGER_MULT(7) |
		       DPIO_TX2_STAGGER_MULT(5));
/* FIXME: Fix up value only after power analysis */
	/* Clear calc init */
	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW10(ch));
	val &= ~(DPIO_PCS_SWING_CALC_TX0_TX2 | DPIO_PCS_SWING_CALC_TX1_TX3);
	val &= ~(DPIO_PCS_TX1DEEMP_MASK | DPIO_PCS_TX2DEEMP_MASK);
	val |= DPIO_PCS_TX1DEEMP_9P5 | DPIO_PCS_TX2DEEMP_9P5;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW10(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW10(ch));
	val &= ~(DPIO_PCS_SWING_CALC_TX0_TX2 | DPIO_PCS_SWING_CALC_TX1_TX3);
	val &= ~(DPIO_PCS_TX1DEEMP_MASK | DPIO_PCS_TX2DEEMP_MASK);
	val |= DPIO_PCS_TX1DEEMP_9P5 | DPIO_PCS_TX2DEEMP_9P5;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW10(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW9(ch));
	val &= ~(DPIO_PCS_TX1MARGIN_MASK | DPIO_PCS_TX2MARGIN_MASK);
	val |= DPIO_PCS_TX1MARGIN_000 | DPIO_PCS_TX2MARGIN_000;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW9(ch), val);

	val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW9(ch));
	val &= ~(DPIO_PCS_TX1MARGIN_MASK | DPIO_PCS_TX2MARGIN_MASK);
	val |= DPIO_PCS_TX1MARGIN_000 | DPIO_PCS_TX2MARGIN_000;
	vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW9(ch), val);

	/* Program 4k and non-4k modes based on clock value */
	if (adjusted_mode->clock <= 162000) {
		/* Programs non-4k modes here */
		/* FIXME: Program the support xxx V-dB */
		/* Use 800 mV-0dB */
		for (i = 0; i < 4; i++) {
			val = vlv_dpio_read(dev_priv, pipe, CHV_TX_DW4(ch, i));
			val &= ~DPIO_SWING_DEEMPH9P5_MASK;
			val &= ~DPIO_SWING_DEEMPH6P0_MASK;
			val |= 0x80 << DPIO_SWING_DEEMPH9P5_SHIFT;
			val |= 0x80 << DPIO_SWING_DEEMPH6P0_SHIFT;
			vlv_dpio_write(dev_priv, pipe, CHV_TX_DW4(ch, i), val);
		}

		for (i = 0; i < 4; i++) {
			val = vlv_dpio_read(dev_priv, pipe, CHV_TX_DW2(ch, i));
			val &= ~DPIO_SWING_MARGIN000_MASK;
			val &= ~DPIO_UNIQ_TRANS_SCALE_MASK;
			val |= 0x80 << DPIO_SWING_MARGIN000_SHIFT;
			val |= 0x98 << DPIO_UNIQ_TRANS_SCALE_SHIFT;
			vlv_dpio_write(dev_priv, pipe, CHV_TX_DW2(ch, i), val);
		}

		/* Disable unique transition scale */
		for (i = 0; i < 4; i++) {
			val = vlv_dpio_read(dev_priv, pipe, CHV_TX_DW3(ch, i));
			val &= ~DPIO_TX_UNIQ_TRANS_SCALE_EN;
			val |= 0x80 << DPIO_SWING_MARGIN101_SHIFT;
			val |= 0x4 << DPIO_DOWN_SCALE_AMP_METHOD_SHIFT;
			vlv_dpio_write(dev_priv, pipe, CHV_TX_DW3(ch, i), val);
		}
		/* Start swing calculation */
			val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW10(ch));
			val &= ~DPIO_PCS_SWING_DEEMPH_CALC_MASK;
			val |= DPIO_PCS_DEEMPH_CALC_TX0_TX2 |
				DPIO_PCS_DEEMPH_CALC_TX1_TX3;
			vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW10(ch), val);

			val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW10(ch));
			val &= ~DPIO_PCS_SWING_DEEMPH_CALC_MASK;
			val |= DPIO_PCS_DEEMPH_CALC_TX0_TX2 |
				DPIO_PCS_DEEMPH_CALC_TX1_TX3;
			vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW10(ch), val);
	} else {
		/* Programs 4k-modes here */
		/* Use 800 mV-0dB */
		for (i = 0; i < 4; i++) {
			if (i == 3) {
				val = vlv_dpio_read(dev_priv, pipe,
					CHV_TX_DW4(ch, i));
				val &= ~DPIO_SWING_DEEMPH9P5_MASK;
				val &= ~DPIO_SWING_DEEMPH6P0_MASK;
				val |= 0x80 << DPIO_SWING_DEEMPH9P5_SHIFT;
				val |= 0x80 << DPIO_SWING_DEEMPH6P0_SHIFT;
				vlv_dpio_write(dev_priv, pipe,
					CHV_TX_DW4(ch, i), val);
			} else {
				val = vlv_dpio_read(dev_priv, pipe,
					CHV_TX_DW4(ch, i));
				val &= ~DPIO_SWING_DEEMPH9P5_MASK;
				val &= ~DPIO_SWING_DEEMPH6P0_MASK;
				val |= 0x60 << DPIO_SWING_DEEMPH9P5_SHIFT;
				val |= 0x60 << DPIO_SWING_DEEMPH6P0_SHIFT;
				vlv_dpio_write(dev_priv, pipe,
					CHV_TX_DW4(ch, i), val);
			}
		}

		for (i = 0; i < 4; i++) {
			val = vlv_dpio_read(dev_priv, pipe, CHV_TX_DW2(ch, i));
			val &= ~DPIO_SWING_MARGIN000_MASK;
			val &= ~DPIO_UNIQ_TRANS_SCALE_MASK;
			val |= 0xa0 << DPIO_SWING_MARGIN000_SHIFT;
			val |= 0x98 << DPIO_UNIQ_TRANS_SCALE_SHIFT;
			vlv_dpio_write(dev_priv, pipe, CHV_TX_DW2(ch, i), val);
		}

		/* Disable unique transition scale */
		for (i = 0; i < 4; i++) {
			val = vlv_dpio_read(dev_priv, pipe, CHV_TX_DW3(ch, i));
			val &= ~DPIO_TX_UNIQ_TRANS_SCALE_EN;
			val |= 0xa0 << DPIO_SWING_MARGIN101_SHIFT;
			val |= 0x4 << DPIO_DOWN_SCALE_AMP_METHOD_SHIFT;
			vlv_dpio_write(dev_priv, pipe, CHV_TX_DW3(ch, i), val);
		}

		/* Start swing calculation */
			val = vlv_dpio_read(dev_priv, pipe, VLV_PCS01_DW10(ch));
			val &= ~DPIO_PCS_SWING_DEEMPH_CALC_MASK;
			val |= DPIO_PCS_DEEMPH_CALC_TX0_TX2 |
				DPIO_PCS_DEEMPH_CALC_TX1_TX3;
			vlv_dpio_write(dev_priv, pipe, VLV_PCS01_DW10(ch), val);

			val = vlv_dpio_read(dev_priv, pipe, VLV_PCS23_DW10(ch));
			val &= ~DPIO_PCS_SWING_DEEMPH_CALC_MASK;
			val |= DPIO_PCS_DEEMPH_CALC_TX0_TX2 |
				DPIO_PCS_DEEMPH_CALC_TX1_TX3;
			vlv_dpio_write(dev_priv, pipe, VLV_PCS23_DW10(ch), val);
	}

	/* LRC Bypass */
	val = vlv_dpio_read(dev_priv, pipe, CHV_CMN_DW30);
	val |= DPIO_LRC_BYPASS;
	vlv_dpio_write(dev_priv, pipe, CHV_CMN_DW30, val);

	mutex_unlock(&dev_priv->dpio_lock);

	intel_hdmi->set_infoframes(&encoder->base,
				   intel_crtc->config.has_hdmi_sink,
				   adjusted_mode);

	intel_enable_hdmi(encoder);

	vlv_wait_port_ready(dev_priv, dport);
}

static void intel_hdmi_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
	kfree(connector);
}

static const struct drm_connector_funcs intel_hdmi_connector_funcs = {
	.dpms = intel_connector_dpms,
	.detect = intel_hdmi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_hdmi_set_property,
	.destroy = intel_hdmi_destroy,
};

static const struct drm_connector_helper_funcs intel_hdmi_connector_helper_funcs = {
	.get_modes = intel_hdmi_get_modes,
	.mode_valid = intel_hdmi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_encoder_funcs intel_hdmi_enc_funcs = {
	.destroy = intel_encoder_destroy,
};

static void
intel_hdmi_add_properties(struct intel_hdmi *intel_hdmi, struct drm_connector *connector)
{
	intel_attach_force_audio_property(connector);
	intel_attach_broadcast_rgb_property(connector);
	intel_attach_force_pfit_property(connector);
	intel_attach_scaling_src_size_property(connector);
	intel_attach_force_ddr_low_freq_property(connector);
	intel_hdmi->color_range_auto = true;
}

static void intel_hdmi_setup_ddc_vbt(struct drm_i915_private *dev_priv,
				     struct intel_digital_port *intel_dig_port)
{
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	enum port port = intel_dig_port->port;
	int i;

	for (i = 0; i < dev_priv->vbt.child_dev_num; i++) {
		if (port != dev_priv->vbt.child_dev[i].common.dvo_port)
			continue;

		intel_hdmi->ddc_bus = dev_priv->vbt.child_dev[i].common.ddc_pin;
		break;
	}

	DRM_INFO("hdmi port %d, ddc_bus %d, hpd_pin %d\n",
		 port, intel_hdmi->ddc_bus, intel_encoder->hpd_pin);
}

void intel_hdmi_init_connector(struct intel_digital_port *intel_dig_port,
			       struct intel_connector *intel_connector)
{
	struct drm_connector *connector = &intel_connector->base;
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_device *dev = intel_encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum port port = intel_dig_port->port;

	drm_connector_init(dev, connector, &intel_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &intel_hdmi_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	connector->stereo_allowed = 1;

	switch (port) {
	case PORT_B:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPB;
		intel_encoder->hpd_pin = HPD_PORT_B;
		break;
	case PORT_C:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPC;
		intel_encoder->hpd_pin = HPD_PORT_C;
		break;
	case PORT_D:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPD;
		intel_encoder->hpd_pin = HPD_PORT_D;
		break;
	case PORT_A:
		intel_encoder->hpd_pin = HPD_PORT_A;
		/* Internal port only for eDP. */
	default:
		BUG();
	}

	/* for cherryview, we prefer the ddc_bus number from VBT */
	if (IS_CHERRYVIEW(dev))
		intel_hdmi_setup_ddc_vbt(dev_priv, intel_dig_port);

	if (IS_VALLEYVIEW(dev)) {
		intel_hdmi->write_infoframe = vlv_write_infoframe;
		intel_hdmi->set_infoframes = vlv_set_infoframes;
	} else if (!HAS_PCH_SPLIT(dev)) {
		intel_hdmi->write_infoframe = g4x_write_infoframe;
		intel_hdmi->set_infoframes = g4x_set_infoframes;
	} else if (HAS_DDI(dev)) {
		intel_hdmi->write_infoframe = hsw_write_infoframe;
		intel_hdmi->set_infoframes = hsw_set_infoframes;
	} else if (HAS_PCH_IBX(dev)) {
		intel_hdmi->write_infoframe = ibx_write_infoframe;
		intel_hdmi->set_infoframes = ibx_set_infoframes;
	} else {
		intel_hdmi->write_infoframe = cpt_write_infoframe;
		intel_hdmi->set_infoframes = cpt_set_infoframes;
	}

	if (HAS_DDI(dev))
		intel_connector->get_hw_state = intel_ddi_connector_get_hw_state;
	else
		intel_connector->get_hw_state = intel_connector_get_hw_state;
	intel_connector->unregister = intel_connector_unregister;

	intel_hdmi_add_properties(intel_hdmi, connector);

	intel_connector_attach_encoder(intel_connector, intel_encoder);
	drm_connector_register(connector);

	/* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written
	 * 0xd.  Failure to do so will result in spurious interrupts being
	 * generated on the port when a cable is not attached.
	 */
	if (IS_G4X(dev) && !IS_GM45(dev)) {
		u32 temp = I915_READ(PEG_BAND_GAP_DATA);
		I915_WRITE(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
	}

	/* Load initialized connector */
	intel_hdmi->attached_connector = intel_connector;

	/*
	 * Probe the first state of HDMI forcefully. This is required as
	 * EDID read is happening only it hot_plug() functions, but in
	 * few configurations kms or fb_console drivers call detect
	 * not the hot_plug(). After init, we can detect plug-in/out in
	 * hot-plug functions
	 */
	intel_hdmi->edid = intel_hdmi_get_edid(connector, true);

	/* Update the first status */
	connector->status = intel_hdmi_detect(connector, false);

	intel_hdmi->skip_port_check = false;
}

void intel_hdmi_init(struct drm_device *dev, int hdmi_reg, enum port port)
{
	struct intel_digital_port *intel_dig_port;
	struct intel_encoder *intel_encoder;
	struct intel_connector *intel_connector;

	intel_dig_port = kzalloc(sizeof(*intel_dig_port), GFP_KERNEL);
	if (!intel_dig_port)
		return;

	intel_connector = kzalloc(sizeof(*intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dig_port);
		return;
	}

	intel_encoder = &intel_dig_port->base;

	drm_encoder_init(dev, &intel_encoder->base, &intel_hdmi_enc_funcs,
			 DRM_MODE_ENCODER_TMDS);

	intel_encoder->compute_config = intel_hdmi_compute_config;
	intel_encoder->disable = intel_disable_hdmi;
	intel_encoder->get_hw_state = intel_hdmi_get_hw_state;
	intel_encoder->get_config = intel_hdmi_get_config;
	if (IS_CHERRYVIEW(dev)) {
		intel_encoder->pre_pll_enable = chv_hdmi_pre_pll_enable;
		intel_encoder->pre_enable = chv_hdmi_pre_enable;
		intel_encoder->enable = vlv_enable_hdmi;
		intel_encoder->post_disable = chv_hdmi_post_disable;
	} else if (IS_VALLEYVIEW(dev)) {
		intel_encoder->pre_pll_enable = vlv_hdmi_pre_pll_enable;
		intel_encoder->pre_enable = vlv_hdmi_pre_enable;
		intel_encoder->enable = vlv_enable_hdmi;
		intel_encoder->post_disable = vlv_hdmi_post_disable;
	} else {
		intel_encoder->pre_enable = intel_hdmi_pre_enable;
		intel_encoder->enable = intel_enable_hdmi;
	}

	intel_encoder->hot_plug = intel_hdmi_hot_plug;
	intel_encoder->type = INTEL_OUTPUT_HDMI;
	if (IS_CHERRYVIEW(dev)) {
		if (port == PORT_D)
			intel_encoder->crtc_mask = 1 << 2;
		else
			intel_encoder->crtc_mask = (1 << 0) | (1 << 1);
	} else {
		intel_encoder->crtc_mask = (1 << 0) | (1 << 1) | (1 << 2);
	}
	intel_encoder->cloneable = 1 << INTEL_OUTPUT_ANALOG;
	/*
	 * BSpec is unclear about HDMI+HDMI cloning on g4x, but it seems
	 * to work on real hardware. And since g4x can send infoframes to
	 * only one port anyway, nothing is lost by allowing it.
	 */
	if (IS_G4X(dev))
		intel_encoder->cloneable |= 1 << INTEL_OUTPUT_HDMI;

	intel_dig_port->port = port;
	intel_dig_port->hdmi.hdmi_reg = hdmi_reg;
	intel_dig_port->dp.output_reg = 0;

	intel_hdmi_init_connector(intel_dig_port, intel_connector);
	intel_connector->panel.fitting_mode = 0;
}
