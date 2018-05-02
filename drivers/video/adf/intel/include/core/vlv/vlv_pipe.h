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
 */

#ifndef _VLV_PIPE_H_
#define _VLV_PIPE_H_

#include <core/intel_dc_config.h>

struct vlv_pipe {
	u32 offset;
	enum pipe pipe_id;
	u32 status_offset;
	u32 scan_line_offset;
	u32 frame_count_offset;

	u32 htotal_offset;
	u32 hblank_offset;
	u32 hsync_offset;
	u32 vtotal_offset;
	u32 vblank_offset;
	u32 vsync_offset;
	u32 gamma_offset;

	u32 datam1_offset;
	u32 datan1_offset;
	u32 linkm1_offset;
	u32 linkn1_offset;

	/* m2_n2 values for eDP DRRS */
	u32 datam2_offset;
	u32 datan2_offset;
	u32 linkm2_offset;
	u32 linkn2_offset;

	u32 src_size_offset;

	/* eDP PSR related registers */
	u32 psr_ctrl_offset;
	u32 psr_sts_offset;
	u32 psr_crc1_offset;
	u32 psr_crc2_offset;
	u32 psr_vsc_sdp_offset;

};

void vlv_pipe_program_m_n(struct vlv_pipe *pipe,
		struct intel_link_m_n *m_n);
void vlv_pipe_program_m2_n2(struct vlv_pipe *pipe,
		struct intel_link_m_n *m_n);
bool vlv_pipe_vblank_on(struct vlv_pipe *pipe);
bool vlv_pipe_vblank_off(struct vlv_pipe *pipe);
bool vlv_pipe_wait_for_vblank(struct vlv_pipe *pipe);
bool vlv_pipe_wait_for_pll_lock(struct vlv_pipe *pipe);
u32 vlv_pipe_program_timings(struct vlv_pipe *pipe,
		struct drm_mode_modeinfo *mode,
		enum intel_pipe_type type, u8 bpp);
u32 vlv_pipe_enable(struct vlv_pipe *pipe,
		struct drm_mode_modeinfo *mode);
int vlv_pipe_disable(struct vlv_pipe *pipe);
u32 vlv_pipe_set_event(struct vlv_pipe *pipe, u32 event, bool enabled);
u32 vlv_pipe_get_event(struct vlv_pipe *pipe, u32 *event);
void vlv_pipe_evade_vblank(struct vlv_pipe *pipe,
		struct drm_mode_modeinfo *mode, bool *wait_for_vblank);
void vlv_pipe_pre_validate(struct intel_pipe *pipe,
		struct intel_adf_post_custom_data *custom);
bool vlv_pipe_init(struct vlv_pipe *pipe, enum pipe enum_pipe);
bool vlv_pipe_destroy(struct vlv_pipe *pipe);

#endif /* _VLV_PIPE_H_ */
