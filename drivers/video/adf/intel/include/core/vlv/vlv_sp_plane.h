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
 */

#ifndef VLV_SP_PLANE_H
#define VLV_SP_PLANE_H

#include "core/intel_dc_config.h"
#include "core/common/overlay.h"
#include "vlv_dc_hw.h"

struct sp_plane_regs_value {
	u32 dspcntr;
	u32 stride;
	u32 pos;
	u32 size;
	u32 const_alpha;
	u32 blend;
	unsigned long linearoff;
	unsigned long tileoff;
	unsigned long surfaddr;
};

struct vlv_sp_plane_context {
	struct sp_plane_regs_value regs;
	u32 plane;
	u32 pipe;
};

struct vlv_sp_plane;

bool vlv_sp_plane_is_enabled(struct vlv_sp_plane *splane);

struct vlv_sp_plane {
	struct intel_plane base;
	u32 offset;
	bool enabled;
	bool alpha_updated;
	bool blend_updated;
	struct vlv_mpo_plane *mpo_plane;
	struct vlv_sp_plane_context ctx;
};

extern uint32_t *chv_hfilters[];
extern uint32_t *chv_vfilters[];
extern uint32_t chv_hab_filter[64];
extern uint32_t chv_vab_filter[32];

static inline struct vlv_sp_plane *to_vlv_sp_plane(struct intel_plane *plane)
{
	return container_of(plane, struct vlv_sp_plane, base);
}

int vlv_sp_plane_init(struct vlv_sp_plane *splane,
		struct intel_pipeline *pipeline, struct device *dev, u8 idx);
void vlv_sp_plane_destroy(struct vlv_sp_plane *splane);

extern long intel_overlay_engine_obj_ioctl(struct adf_obj *obj,
		unsigned int cmd, unsigned long arg);

#endif
