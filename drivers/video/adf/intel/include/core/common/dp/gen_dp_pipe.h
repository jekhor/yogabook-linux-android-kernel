/*
 * Copyright (C) 2014, Intel Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Create on 15 Sep 2014
 * Author: Sivakumar Thulasimani <sivakumar.thulasimani@intel.com>
 */

#ifndef _INTEL_DP_PIPE_H_
#define _INTEL_DP_PIPE_H_

#include <drm/drmP.h>
#include <core/intel_dc_config.h>
#include <core/common/dp/dp_panel.h>

#define DP_LINK_BW_1_62	    0x06
#define DP_LINK_BW_2_7      0x0a
#define DP_LINK_BW_5_4      0x14

#define EDP_PSR_RECEIVER_CAP_SIZE 2
/* eDP PSR related fields */
struct edp_psr {
	u8 dpcd[EDP_PSR_RECEIVER_CAP_SIZE];
	bool sink_support;
	bool source_ok;
	bool setup_done;
	bool link_train_on_exit;
	struct mutex lock;
	struct delayed_work work;
	struct dp_pipe *enabled;
	struct intel_pipeline *pipeline;
	unsigned long entry_ts;
	atomic_t update_pending;
};

/* eDP DRRS related fields */
struct edp_drrs_platform_ops {
	void (*init)(struct intel_pipeline *pipeline);
	void (*exit)(struct intel_pipeline *pipeline);
	void (*set_drrs_state)(struct intel_pipeline *pipeline);
};

/* Used by dp and fdi links */
struct intel_link_m_n {
	u32        tu;
	u32        gmch_m;
	u32        gmch_n;
	u32        link_m;
	u32        link_n;
};

struct edp_drrs {
	/* M1, N1 for normal mode */
	struct intel_link_m_n fixed_mn;
	/* M2, N2 for downclock mode */
	struct intel_link_m_n downclock_mn;
	struct edp_drrs_platform_ops *platform_ops;
};

struct dp_pipe {
	struct edp_psr psr;
	struct edp_drrs drrs;
	struct intel_pipe base;
	struct intel_pipeline *pipeline;
	struct dp_panel panel;
	struct drm_mode_modeinfo preferred_mode;
	struct drm_mode_modeinfo current_mode;
	struct link_params link_params;
	u8 dpms_state;
	bool panel_present;
};

static inline struct dp_pipe *to_dp_pipe(struct intel_pipe *pipe)
{
	return container_of(pipe, struct dp_pipe, base);
}

u32 dp_pipe_init(struct dp_pipe *pipe, struct device *dev,
	struct intel_plane *primary_plane, u8 idx,
	struct intel_pipeline *pipeline, enum intel_pipe_type type);

u32 dp_pipe_destroy(struct dp_pipe *pipe);
void dp_pipe_compute_m_n(u32 bits_per_pixel, u32 nlanes,
			u32 pixel_clock, u32 link_clock,
			struct intel_link_m_n *m_n);

int intel_adf_dp_hot_plug(struct dp_pipe *dp_pipe);
int intel_dp_self_modeset(struct dp_pipe *dp_pipe);
extern int
intel_adf_dp_handle_events(struct dp_pipe *dp_pipe, u32 events);

struct drrs_encoder_ops *intel_get_edp_drrs_ops(void);
struct edp_drrs_platform_ops *get_vlv_edp_drrs_ops(void);
#endif /* _INTEL_DP_PIPE_H_ */
