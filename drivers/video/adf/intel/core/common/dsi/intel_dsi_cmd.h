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
 */

#ifndef _INTEL_DSI_DSI_H
#define _INTEL_DSI_DSI_H

#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include <video/mipi_display.h>
#include "core/common/dsi/dsi_pipe.h"
#include "core/common/dsi/dsi_config.h"
#include "intel_adf_device.h"

#define DPI_LP_MODE_EN	false
#define DPI_HS_MODE_EN	true

void band_gap_reset(struct dsi_pipe *dsi_pipe);
void adf_dsi_hs_mode_enable(struct dsi_pipe *dsi_pipe, bool enable);

int adf_dsi_vc_dcs_write(struct dsi_pipe *dsi_pipe, int channel,
		const u8 *data, int len, enum port port);

int adf_dsi_vc_generic_write(struct dsi_pipe *dsi_pipe, int channel,
			const u8 *data, int len, enum port port);

int adf_dsi_vc_dcs_read(struct dsi_pipe *dsi_pipe, int channel, u8 dcs_cmd,
		u8 *buf, int buflen, enum port port);

int adf_dsi_vc_generic_read(struct dsi_pipe *dsi_pipe, int channel,
		u8 *reqdata, int reqlen, u8 *buf, int buflen, enum port port);

int adf_dpi_send_cmd(struct dsi_pipe *dsi_pipe, u32 cmd, bool hs);


inline int adf_dsi_vc_dcs_write_0(struct dsi_pipe *dsi_pipe,
				  int channel, u8 dcs_cmd,
				  enum port port);
inline int adf_dsi_vc_dcs_write_1(struct dsi_pipe *dsi_pipe,
				  int channel, u8 dcs_cmd, u8 param,
				  enum port port);
inline int adf_dsi_vc_generic_write_0(struct dsi_pipe *dsi_pipe,
				      int channel, enum port port);
inline int adf_dsi_vc_generic_write_1(struct dsi_pipe *dsi_pipe,
				      int channel, u8 param, enum port port);
inline int adf_dsi_vc_generic_write_2(struct dsi_pipe *dsi_pipe,
				      int channel, u8 param1, u8 param2,
				      enum port port);

#endif /* _INTEL_DSI_DSI_H */
