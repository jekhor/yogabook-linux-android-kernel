/*
 * Copyright © 2008-2010 Intel Corporation
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
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *    Zou Nan hai <nanhai.zou@intel.com>
 *    Xiang Hai hao<haihao.xiang@intel.com>
 *
 */

#include <drm/drmP.h>
#include "i915_drv.h"
#include <drm/i915_drm.h>
#include "i915_trace.h"
#include "intel_sync.h"
#include "intel_drv.h"
#include "i915_scheduler.h"

bool
intel_ring_initialized(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;

	if (!dev)
		return false;

	if (i915.enable_execlists) {
		struct intel_context *dctx = ring->default_context;
		struct intel_ringbuffer *ringbuf = dctx->engine[ring->id].ringbuf;

		return ringbuf->obj;
	} else
		return ring->buffer && ring->buffer->obj;
}

int __intel_ring_space(int head, int tail, int size)
{
	int space = head - tail;
	if (space <= 0)
		space += size;
	return space - I915_RING_FREE_SPACE;
}

void intel_ring_update_space(struct intel_ringbuffer *ringbuf)
{
	if (ringbuf->last_retired_head != -1) {
		ringbuf->head = ringbuf->last_retired_head;
		ringbuf->last_retired_head = -1;
	}

	ringbuf->space = __intel_ring_space(ringbuf->head & HEAD_ADDR,
					    ringbuf->tail, ringbuf->size);
}

int intel_ring_space(struct intel_ringbuffer *ringbuf)
{
	intel_ring_update_space(ringbuf);
	return ringbuf->space;
}

bool intel_ring_stopped(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	return dev_priv->gpu_error.stop_rings & intel_ring_flag(ring);
}

void __intel_ring_advance(struct intel_engine_cs *ring)
{
	struct intel_ringbuffer *ringbuf = ring->buffer;

	intel_ring_advance(ring);

	if (intel_ring_stopped(ring))
		return;
	ring->write_tail(ring, ringbuf->tail);
}

static int
gen2_render_ring_flush(struct intel_engine_cs *ring,
		       u32	invalidate_domains,
		       u32	flush_domains)
{
	u32 cmd;
	int ret;

	cmd = MI_FLUSH;
	if (((invalidate_domains|flush_domains) & I915_GEM_DOMAIN_RENDER) == 0)
		cmd |= MI_NO_WRITE_FLUSH;

	if (invalidate_domains & I915_GEM_DOMAIN_SAMPLER)
		cmd |= MI_READ_FLUSH;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int
gen4_render_ring_flush(struct intel_engine_cs *ring,
		       u32	invalidate_domains,
		       u32	flush_domains)
{
	struct drm_device *dev = ring->dev;
	u32 cmd;
	int ret;

	/*
	 * read/write caches:
	 *
	 * I915_GEM_DOMAIN_RENDER is always invalidated, but is
	 * only flushed if MI_NO_WRITE_FLUSH is unset.  On 965, it is
	 * also flushed at 2d versus 3d pipeline switches.
	 *
	 * read-only caches:
	 *
	 * I915_GEM_DOMAIN_SAMPLER is flushed on pre-965 if
	 * MI_READ_FLUSH is set, and is always flushed on 965.
	 *
	 * I915_GEM_DOMAIN_COMMAND may not exist?
	 *
	 * I915_GEM_DOMAIN_INSTRUCTION, which exists on 965, is
	 * invalidated when MI_EXE_FLUSH is set.
	 *
	 * I915_GEM_DOMAIN_VERTEX, which exists on 965, is
	 * invalidated with every MI_FLUSH.
	 *
	 * TLBs:
	 *
	 * On 965, TLBs associated with I915_GEM_DOMAIN_COMMAND
	 * and I915_GEM_DOMAIN_CPU in are invalidated at PTE write and
	 * I915_GEM_DOMAIN_RENDER and I915_GEM_DOMAIN_SAMPLER
	 * are flushed at any MI_FLUSH.
	 */

	cmd = MI_FLUSH | MI_NO_WRITE_FLUSH;
	if ((invalidate_domains|flush_domains) & I915_GEM_DOMAIN_RENDER)
		cmd &= ~MI_NO_WRITE_FLUSH;
	if (invalidate_domains & I915_GEM_DOMAIN_INSTRUCTION)
		cmd |= MI_EXE_FLUSH;

	if (invalidate_domains & I915_GEM_DOMAIN_COMMAND &&
	    (IS_G4X(dev) || IS_GEN5(dev)))
		cmd |= MI_INVALIDATE_ISP;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

/**
 * Emits a PIPE_CONTROL with a non-zero post-sync operation, for
 * implementing two workarounds on gen6.  From section 1.4.7.1
 * "PIPE_CONTROL" of the Sandy Bridge PRM volume 2 part 1:
 *
 * [DevSNB-C+{W/A}] Before any depth stall flush (including those
 * produced by non-pipelined state commands), software needs to first
 * send a PIPE_CONTROL with no bits set except Post-Sync Operation !=
 * 0.
 *
 * [Dev-SNB{W/A}]: Before a PIPE_CONTROL with Write Cache Flush Enable
 * =1, a PIPE_CONTROL with any non-zero post-sync-op is required.
 *
 * And the workaround for these two requires this workaround first:
 *
 * [Dev-SNB{W/A}]: Pipe-control with CS-stall bit set must be sent
 * BEFORE the pipe-control with a post-sync op and no write-cache
 * flushes.
 *
 * And this last workaround is tricky because of the requirements on
 * that bit.  From section 1.4.7.2.3 "Stall" of the Sandy Bridge PRM
 * volume 2 part 1:
 *
 *     "1 of the following must also be set:
 *      - Render Target Cache Flush Enable ([12] of DW1)
 *      - Depth Cache Flush Enable ([0] of DW1)
 *      - Stall at Pixel Scoreboard ([1] of DW1)
 *      - Depth Stall ([13] of DW1)
 *      - Post-Sync Operation ([13] of DW1)
 *      - Notify Enable ([8] of DW1)"
 *
 * The cache flushes require the workaround flush that triggered this
 * one, so we can't use it.  Depth stall would trigger the same.
 * Post-sync nonzero is what triggered this second workaround, so we
 * can't use that one either.  Notify enable is IRQs, which aren't
 * really our business.  That leaves only stall at scoreboard.
 */
static int
intel_emit_post_sync_nonzero_flush(struct intel_engine_cs *ring)
{
	u32 scratch_addr = ring->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	int ret;


	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	intel_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			PIPE_CONTROL_STALL_AT_SCOREBOARD);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT); /* address */
	intel_ring_emit(ring, 0); /* low dword */
	intel_ring_emit(ring, 0); /* high dword */
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	intel_ring_emit(ring, PIPE_CONTROL_QW_WRITE);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT); /* address */
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int
gen6_render_ring_flush(struct intel_engine_cs *ring,
                         u32 invalidate_domains, u32 flush_domains)
{
	u32 flags = 0;
	u32 scratch_addr = ring->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	int ret;

	/* Force SNB workarounds for PIPE_CONTROL flushes */
	ret = intel_emit_post_sync_nonzero_flush(ring);
	if (ret)
		return ret;

	/* Just flush everything.  Experiments have shown that reducing the
	 * number of bits based on the write domains has little performance
	 * impact.
	 */
	if (flush_domains) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		/*
		 * Ensure that any following seqno writes only happen
		 * when the render cache is indeed flushed.
		 */
		flags |= PIPE_CONTROL_CS_STALL;
	}
	if (invalidate_domains) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		/*
		 * TLB invalidate requires a post-sync write.
		 */
		flags |= PIPE_CONTROL_QW_WRITE | PIPE_CONTROL_CS_STALL;
	}

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4));
	intel_ring_emit(ring, flags);
	intel_ring_emit(ring, scratch_addr | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	return 0;
}

static int
gen7_render_ring_cs_stall_wa(struct intel_engine_cs *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4));
	intel_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			      PIPE_CONTROL_STALL_AT_SCOREBOARD);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	return 0;
}

static int gen7_ring_fbc_flush(struct intel_engine_cs *ring, u32 value)
{
	int ret;

	if (!ring->fbc_dirty)
		return 0;

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;
	/* WaFbcNukeOn3DBlt:ivb/hsw */
	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, MSG_FBC_REND_STATE);
	intel_ring_emit(ring, value);
	intel_ring_emit(ring, MI_STORE_REGISTER_MEM(1) | MI_SRM_LRM_GLOBAL_GTT);
	intel_ring_emit(ring, MSG_FBC_REND_STATE);
	intel_ring_emit(ring, ring->scratch.gtt_offset + 256);
	intel_ring_advance(ring);

	ring->fbc_dirty = false;
	return 0;
}

static int
gen7_render_ring_flush(struct intel_engine_cs *ring,
		       u32 invalidate_domains, u32 flush_domains)
{
	u32 flags = 0;
	u32 scratch_addr = ring->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	int ret;

	/*
	 * Ensure that any following seqno writes only happen when the render
	 * cache is indeed flushed.
	 *
	 * Workaround: 4th PIPE_CONTROL command (except the ones with only
	 * read-cache invalidate bits set) must have the CS_STALL bit set. We
	 * don't try to be clever and just set it unconditionally.
	 */
	flags |= PIPE_CONTROL_CS_STALL;

	/* Just flush everything.  Experiments have shown that reducing the
	 * number of bits based on the write domains has little performance
	 * impact.
	 */
	if (flush_domains) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
	}
	if (invalidate_domains) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		/*
		 * TLB invalidate requires a post-sync write.
		 */
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_GLOBAL_GTT_IVB;

		/* Workaround: we must issue a pipe_control with CS-stall bit
		 * set before a pipe_control command that has the state cache
		 * invalidate bit set. */
		gen7_render_ring_cs_stall_wa(ring);
	}

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4));
	intel_ring_emit(ring, flags);
	intel_ring_emit(ring, scratch_addr);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	if (!invalidate_domains && flush_domains)
		return gen7_ring_fbc_flush(ring, FBC_REND_NUKE);

	return 0;
}

static int
gen8_render_ring_flush(struct intel_engine_cs *ring,
		       u32 invalidate_domains, u32 flush_domains)
{
	u32 flags = 0;
	u32 scratch_addr = ring->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	int ret;

	flags |= PIPE_CONTROL_CS_STALL;

	if (flush_domains) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
	}
	if (invalidate_domains) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_GLOBAL_GTT_IVB;
	}

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(6));
	intel_ring_emit(ring, flags);
	intel_ring_emit(ring, scratch_addr);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, 0);
	intel_ring_emit(ring, 0);
	intel_ring_advance(ring);

	return 0;

}

static void ring_write_tail(struct intel_engine_cs *ring,
			    u32 value)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	I915_WRITE_TAIL(ring, value);
}

int intel_ring_disable(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	if (ring && ring->disable)
		return ring->disable(ring, ctx);
	else {
		DRM_ERROR("ring disable not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_disable(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t ring_ctl;
	uint32_t mi_mode;
	uint32_t retries = 10000;

	/* Request the ring to go idle */
	I915_WRITE_MODE(ring, _MASKED_BIT_ENABLE(RING_MODE_STOP));

	/* Wait for idle */
	do {
		mi_mode = I915_READ_MODE(ring);
	} while (retries-- && !(mi_mode & RING_MODE_IDLE));

	if (retries == 0) {
		DRM_ERROR("timed out trying to disable ring %d\n", ring->id);
		return -ETIMEDOUT;
	}

	/* Disable the ring */
	ring_ctl = I915_READ_CTL(ring);
	ring_ctl &= (RING_NR_PAGES | RING_REPORT_MASK);
	I915_WRITE_CTL(ring, ring_ctl);
	ring_ctl = I915_READ_CTL(ring); /* Barrier read */

	return ((ring_ctl & RING_VALID) == 0) ? 0 : -EIO;
}

int intel_ring_enable(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	if (ring && ring->enable)
		return ring->enable(ring, ctx);
	else {
		DRM_ERROR("ring enable not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_enable(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t ring_ctl;
	uint32_t mode;

	/* Clear the MI_MODE stop bit */
	I915_WRITE_MODE(ring, _MASKED_BIT_DISABLE(RING_MODE_STOP));
	mode = I915_READ_MODE(ring); /* Barrier read */

	/* Enable the ring */
	ring_ctl = I915_READ_CTL(ring);
	ring_ctl &= (RING_NR_PAGES | RING_REPORT_MASK);
	I915_WRITE_CTL(ring, ring_ctl | RING_VALID);
	ring_ctl = I915_READ_CTL(ring); /* Barrier read */

	return ((ring_ctl & RING_VALID) == 0) ? -EIO : 0;
}

int intel_ring_save(struct intel_engine_cs *ring, struct intel_context *ctx,
		u32 flags)
{
	if (ring && ring->save)
		return ring->save(ring, ctx, ring->saved_state,
			I915_RING_CONTEXT_SIZE, flags);
	else {
		DRM_ERROR("ring save not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_save(struct intel_engine_cs *ring, struct intel_context *ctx,
			   uint32_t *data, uint32_t data_size, u32 flags)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ringbuffer *ringbuf = ring->buffer;
	uint32_t idx = 0;
	uint32_t gen = INTEL_INFO(dev)->gen;
	uint32_t head;
	uint32_t tail;
	uint32_t head_addr;
	uint32_t tail_addr;
	int clamp_to_tail = 0;

	/* Ring save only added for gen >= 7 */
	WARN_ON(gen < 7);

	/* Save common registers */
	if (data_size < GEN7_COMMON_RING_CTX_SIZE)
		return -EINVAL;

	head = I915_READ_HEAD(ring);
	tail = I915_READ_TAIL(ring);

	/*
	 * head_addr and tail_addr are the head and tail values
	 * excluding ring wrapping information and aligned to DWORD
	 * boundary
	 */
	head_addr = head & HEAD_ADDR;
	tail_addr = tail & TAIL_ADDR;

	/*
	 * The head must always chase the tail.
	 * If the tail is beyond the head then do not allow
	 * the head to overtake it. If the tail is less than
	 * the head then the tail has already wrapped and
	 * there is no problem in advancing the head or even
	 * wrapping the head back to 0 as worst case it will
	 * become equal to tail
	 */
	if (head_addr <= tail_addr)
		clamp_to_tail = 1;

	if (flags & FORCE_ADVANCE) {

		/* Force head pointer to next QWORD boundary */
		head_addr &= ~0x7;
		head_addr += 8;
		DRM_DEBUG_TDR("Forced head to 0x%08x\n", (unsigned int) head_addr);

	} else if (head & 0x7) {

		/* Ensure head pointer is pointing to a QWORD boundary */
		DRM_DEBUG_TDR("Rounding up head 0x%08x\n", (unsigned int) head);
		head += 0x7;
		head &= ~0x7;
		head_addr = head;
	}

	if (clamp_to_tail && (head_addr > tail_addr)) {
		head_addr = tail_addr;
	} else if (head_addr >= ringbuf->size) {
		/* Wrap head back to start if it exceeds ring size*/
		head_addr = 0;
	}

	/* Update the register */
	head &= ~HEAD_ADDR;
	head |= (head_addr & HEAD_ADDR);

	/* Saved with enable = 0 */
	data[idx++] = I915_READ_CTL(ring) & (RING_NR_PAGES | RING_REPORT_MASK);

	data[idx++] = (flags & RESET_HEAD_TAIL) ? 0 : tail;

	if (flags & RESET_HEAD_TAIL) {
		/* Save head as 0 so head is reset on restore */
		data[idx++] = 0;
	} else {
		/* Head will already have advanced to next instruction location
		* even if the current instruction caused a hang, so we just
		* save the current value as the value to restart at */
		data[idx++] = head;
	}

	data[idx++] = I915_READ_START(ring);

	/* Workaround for reading DCLV registers for gen < 8 */
	data[idx++] = (gen < 8) ?
		I915_READ(RING_PP_DIR_DCLV(&dev_priv->ring[VCS]))
		: I915_READ(RING_PP_DIR_DCLV(ring));

	data[idx++] = (gen < 8) ?
		I915_READ(RING_PP_DIR_BASE(&dev_priv->ring[VCS]))
		: I915_READ(RING_PP_DIR_BASE(ring));

	switch (ring->id) {
	case RCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_RCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(RENDER_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(CACHE_MODE_1);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(_3D_CHICKEN3);
		data[idx++] = I915_READ(GAM_ECOCHK);
		data[idx++] = I915_READ(GFX_MODE_GEN7);
		data[idx++] = I915_READ(GEN6_RBSYNC);
		data[idx++] = I915_READ(GEN7_FF_THREAD_MODE);
		data[idx++] = I915_READ(RS_CHICKEN(ring->mmio_base));
		data[idx++] = I915_READ(WAIT_FOR_RC6_EXIT(ring->mmio_base));
		data[idx++] = I915_READ(FF_SLICE_CS_CHICKEN2(ring->mmio_base));
		break;

	case VCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_VCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(BSD_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_EXCC_GEN7(ring));
		data[idx++] = I915_READ(GAC_ECO_BITS);
		data[idx++] = I915_READ(RING_MODE_GEN7(ring));
		data[idx++] = I915_READ(GEN6_VRSYNC);
		data[idx++] = I915_READ(RING_MAX_IDLE(ring->mmio_base));
		break;

	case BCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_BCS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(BLT_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_EXCC_GEN7(ring));
		data[idx++] = I915_READ(GAB_CTL);
		data[idx++] = I915_READ(RING_MODE_GEN7(ring));
		data[idx++] = I915_READ(GEN6_BRSYNC);
		data[idx++] = I915_READ(GEN6_BVSYNC);
		data[idx++] = I915_READ(RING_MAX_IDLE(ring->mmio_base));
		break;

	case VECS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_VECS_RING_CTX_SIZE))
			return -EINVAL;

		data[idx++] = I915_READ(VEBOX_HWS_PGA_GEN7);
		data[idx++] = I915_READ(RING_MI_MODE(ring->mmio_base));
		data[idx++] = I915_READ(RING_IMR(ring->mmio_base));
		data[idx++] = I915_READ(RING_UHPTR(ring->mmio_base));
		data[idx++] = I915_READ(RING_INSTPM(ring->mmio_base));
		data[idx++] = I915_READ(RING_EXCC_GEN7(ring));
		data[idx++] = I915_READ(RING_MODE_GEN7(ring));
		data[idx++] = I915_READ(GEN6_VEVSYNC);
		break;

	default:
		DRM_ERROR("Invalid ring ID %d\n", ring->id);
		break;
	}

	return 0;
}

int intel_ring_restore(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	if (ring && ring->restore)
		return ring->restore(ring, ctx, ring->saved_state,
			I915_RING_CONTEXT_SIZE);
	else {
		DRM_ERROR("ring restore not supported\n");
		return -EINVAL;
	}
}

static int
gen6_ring_restore(struct intel_engine_cs *ring, struct intel_context *ctx,
		uint32_t *data, uint32_t data_size)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t idx = 0;
	uint32_t x;

	/* NOTE: Registers are restored in reverse order from when
	* they were saved. */
	switch (ring->id) {
	case RCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_RCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = GEN7_COMMON_RING_CTX_SIZE + GEN7_RCS_RING_CTX_SIZE - 1;

		I915_WRITE(FF_SLICE_CS_CHICKEN2(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(WAIT_FOR_RC6_EXIT(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RS_CHICKEN(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(GEN7_FF_THREAD_MODE, data[idx--]);
		I915_WRITE(GEN6_RBSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(GAM_ECOCHK, data[idx--]);
		I915_WRITE(_3D_CHICKEN3,
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_MI_MODE(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(CACHE_MODE_1,
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_INSTPM(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RENDER_HWS_PGA_GEN7, data[idx--]);
		break;

	case VCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_VCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = GEN7_COMMON_RING_CTX_SIZE + GEN7_VCS_RING_CTX_SIZE - 1;
		I915_WRITE(RING_MAX_IDLE(ring->mmio_base), data[idx--]);
		I915_WRITE(GEN6_VRSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(GAC_ECO_BITS, data[idx--]);
		I915_WRITE(RING_EXCC_GEN7(ring),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_INSTPM(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(BSD_HWS_PGA_GEN7, data[idx--]);
		break;

	case BCS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_BCS_RING_CTX_SIZE))
			return -EINVAL;

		idx = GEN7_COMMON_RING_CTX_SIZE + GEN7_BCS_RING_CTX_SIZE - 1;

		I915_WRITE(RING_MAX_IDLE(ring->mmio_base), data[idx--]);
		I915_WRITE(GEN6_BVSYNC, data[idx--]);
		I915_WRITE(GEN6_BRSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(GAB_CTL, data[idx--]);
		I915_WRITE(RING_EXCC_GEN7(ring),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_INSTPM(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base),
			_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(BLT_HWS_PGA_GEN7, data[idx--]);
		break;

	case VECS:
		if (data_size < (GEN7_COMMON_RING_CTX_SIZE + GEN7_VECS_RING_CTX_SIZE))
			return -EINVAL;

		idx = GEN7_COMMON_RING_CTX_SIZE + GEN7_VECS_RING_CTX_SIZE - 1;

		I915_WRITE(GEN6_VEVSYNC, data[idx--]);
		I915_WRITE(RING_MODE_GEN7(ring),
				_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_EXCC_GEN7(ring),
				_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_INSTPM(ring->mmio_base),
				_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(RING_UHPTR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_IMR(ring->mmio_base), data[idx--]);
		I915_WRITE(RING_MI_MODE(ring->mmio_base),
				_MASKED_BIT_ENABLE_ALL(data[idx--]));
		I915_WRITE(VEBOX_HWS_PGA_GEN7, data[idx--]);
		break;

	default:
		DRM_ERROR("Invalid ring ID %d\n", ring->id);
		break;
	}

	/* Restore common registers */
	if (data_size < GEN7_COMMON_RING_CTX_SIZE)
		return -EINVAL;

	idx = GEN7_COMMON_RING_CTX_SIZE - 1;

	I915_WRITE(RING_PP_DIR_BASE(ring), data[idx--]);
	I915_WRITE(RING_PP_DIR_DCLV(ring), data[idx--]);

	/* Write ring base address before head/tail as it clears head to 0 */
	I915_WRITE_START(ring, data[idx--]);
	x = I915_READ_START(ring);
	I915_WRITE_HEAD(ring, data[idx--]);
	I915_WRITE_TAIL(ring, data[idx--]);
	I915_WRITE_CTL(ring, data[idx--]);

	return 0;
}

int intel_ring_invalidate_tlb(struct intel_engine_cs *ring)
{
	if (ring && ring->invalidate_tlb)
		return ring->invalidate_tlb(ring);
	else {
		DRM_ERROR("ring invalidate tlb not supported\n");
		return -EINVAL;
	}
}

void intel_gpu_reset_resample(struct intel_engine_cs *ring,
		struct intel_context *ctx)
{
	if (!ring) {
		WARN(!ring, "Ring is null! Could not resample!");
		return;
	}

	if (i915.enable_execlists) {
		struct drm_i915_private *dev_priv = ring->dev->dev_private;
		uint32_t ring_tail;
		uint32_t ring_head;
		struct intel_ringbuffer *ringbuf;

		if (!ctx) {
			WARN(!ring, "Context is null! Could not resample!");
			return;
		}

		/* Reset context based on ring state */
		ring_tail = I915_READ_TAIL(ring) & TAIL_ADDR;
		ring_head = I915_READ_HEAD(ring) & HEAD_ADDR;
		ringbuf = ctx->engine[ring->id].ringbuf;

		I915_WRITE_HEAD_CTX(ring, ctx, ring_head);
		I915_WRITE_TAIL_CTX(ring, ctx, ring_tail);
		ringbuf->head = ring_head;
		ringbuf->tail = ring_tail;
		ringbuf->last_retired_head = -1;
		intel_ring_update_space(ringbuf);
	}
}

void intel_gpu_engine_reset_resample(struct intel_engine_cs *ring,
		struct intel_context *ctx)
{
	struct intel_ringbuffer *ringbuf = NULL;
	struct drm_i915_private *dev_priv;

	if (!ring) {
		WARN(1, "Ring is null! Could not resample!");
		return;
	}

	dev_priv = ring->dev->dev_private;

	if (!drm_core_check_feature(ring->dev, DRIVER_MODESET))
		i915_kernel_lost_context(ring->dev);
	else {

		if (i915.enable_execlists) {
			if (!ctx) {
				WARN(1, "Context is null! Could not resample!");
				return;
			}

			ringbuf = ctx->engine[ring->id].ringbuf;

			/*
			 * In gen8+ context head is restored during reset and
			 * we can use it as a reference to set up the new
			 * driver state.
			 */
			I915_READ_HEAD_CTX(ring, ctx, ringbuf->head);

			/*
			 * Do not resample tail in execlist mode.
			 *
			 * If we run in execlist mode there will be an execlist
			 * queue in place to manage ring submissions. In that
			 * case the hardware (and the current ring context
			 * state) will be lagging behind the execlist queue.
			 * Since the execlist queue depends on the tail value
			 * of the ring buffer to keep track of the most recent
			 * submission to the execlist queue we would be
			 * breaking the execlist queue by overwriting this
			 * value with the older tail value currently set in the
			 * ring. Let the execlist queue handle the up to date
			 * tail value and don't overwrite it with older values
			 * from the current ring state.
			 */
		} else {
			ringbuf = ring->buffer;
			ringbuf->head = I915_READ_HEAD(ring);
			ringbuf->tail = I915_READ_TAIL(ring) & TAIL_ADDR;
		}

		ringbuf->last_retired_head = -1;
		intel_ring_update_space(ringbuf);
	}
}

u64 intel_ring_get_active_head(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	u64 acthd;

	if (INTEL_INFO(ring->dev)->gen >= 8)
		acthd = I915_READ64_2x32(RING_ACTHD(ring->mmio_base),
					 RING_ACTHD_UDW(ring->mmio_base));
	else if (INTEL_INFO(ring->dev)->gen >= 4)
		acthd = I915_READ(RING_ACTHD(ring->mmio_base));
	else
		acthd = I915_READ(ACTHD);

	return acthd;
}

static void ring_setup_phys_status_page(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	u32 addr;

	addr = dev_priv->status_page_dmah->busaddr;
	if (INTEL_INFO(ring->dev)->gen >= 4)
		addr |= (dev_priv->status_page_dmah->busaddr >> 28) & 0xf0;
	I915_WRITE(HWS_PGA, addr);
}

static bool stop_ring(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = to_i915(ring->dev);
	struct intel_context *ctx = ring->default_context;

	if (!IS_GEN2(ring->dev)) {
		I915_WRITE_MODE(ring, _MASKED_BIT_ENABLE(RING_MODE_STOP));
		if (wait_for_atomic((I915_READ_MODE(ring) & RING_MODE_IDLE) != 0,
				1000)) {
			DRM_ERROR("%s :timed out trying to stop ring\n", ring->name);
			return false;
		}
	}

	if (I915_WRITE_CTL_CTX_MMIO(ring, ctx, 0))
			return false;

	if (I915_WRITE_HEAD_CTX_MMIO(ring, ctx, 0))
			return false;

	ring->write_tail(ring, 0);

	if (!IS_GEN2(ring->dev)) {
		(void)I915_READ_CTL(ring);
		I915_WRITE_MODE(ring, _MASKED_BIT_DISABLE(RING_MODE_STOP));
	}

	return (I915_READ_HEAD(ring) & HEAD_ADDR) == 0;
}

static int init_ring_common(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ringbuffer *ringbuf = ring->buffer;
	struct drm_i915_gem_object *obj = ringbuf->obj;
	int ret = 0;

	gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	if (!stop_ring(ring)) {
		/* G45 ring initialization often fails to reset head to zero */
		DRM_DEBUG_KMS("%s head not reset to zero "
			      "ctl %08x head %08x tail %08x start %08x\n",
			      ring->name,
			      I915_READ_CTL(ring),
			      I915_READ_HEAD(ring),
			      I915_READ_TAIL(ring),
			      I915_READ_START(ring));

		if (!stop_ring(ring)) {
			DRM_ERROR("failed to set %s head to zero "
				  "ctl %08x head %08x tail %08x start %08x\n",
				  ring->name,
				  I915_READ_CTL(ring),
				  I915_READ_HEAD(ring),
				  I915_READ_TAIL(ring),
				  I915_READ_START(ring));
			ret = -EIO;
			goto out;
		}
	}

	if (I915_NEED_GFX_HWS(dev))
		intel_ring_setup_status_page(ring);
	else
		ring_setup_phys_status_page(ring);

	/* Initialize the ring. This must happen _after_ we've cleared the ring
	 * registers with the above sequence (the readback of the HEAD registers
	 * also enforces ordering), otherwise the hw might lose the new ring
	 * register values. */
	I915_WRITE_START(ring, i915_gem_obj_ggtt_offset(obj));
	I915_WRITE_CTL(ring,
			((ringbuf->size - PAGE_SIZE) & RING_NR_PAGES)
			| RING_VALID);

	/* If the head is still not zero, the ring is dead */
	if (wait_for((I915_READ_CTL(ring) & RING_VALID) != 0 &&
		     I915_READ_START(ring) == i915_gem_obj_ggtt_offset(obj) &&
		     (I915_READ_HEAD(ring) & HEAD_ADDR) == 0, 50)) {
		DRM_ERROR("%s initialization failed "
			  "ctl %08x (valid? %d) head %08x tail %08x start %08x [expected %08lx]\n",
			  ring->name,
			  I915_READ_CTL(ring), I915_READ_CTL(ring) & RING_VALID,
			  I915_READ_HEAD(ring), I915_READ_TAIL(ring),
			  I915_READ_START(ring), (unsigned long)i915_gem_obj_ggtt_offset(obj));
		ret = -EIO;
		goto out;
	}

	if (!drm_core_check_feature(ring->dev, DRIVER_MODESET))
		i915_kernel_lost_context(ring->dev);
	else {
		ringbuf->last_retired_head = -1;
		ringbuf->head = I915_READ_HEAD(ring);
		ringbuf->tail = I915_READ_TAIL(ring) & TAIL_ADDR;
		intel_ring_update_space(ringbuf);
	}


out:
	gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return ret;
}

void
intel_fini_pipe_control(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;

	if (ring->scratch.obj == NULL)
		return;

	if (INTEL_INFO(dev)->gen >= 5) {
		kunmap(sg_page(ring->scratch.obj->pages->sgl));
		i915_gem_object_ggtt_unpin(ring->scratch.obj);
	}

	drm_gem_object_unreference(&ring->scratch.obj->base);
	ring->scratch.obj = NULL;
}

int
intel_init_pipe_control(struct intel_engine_cs *ring)
{
	int ret;

	if (ring->scratch.obj)
		return 0;

	ring->scratch.obj = i915_gem_alloc_object(ring->dev, 4096);
	if (ring->scratch.obj == NULL) {
		DRM_ERROR("Failed to allocate seqno page\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = i915_gem_object_set_cache_level(ring->scratch.obj, I915_CACHE_LLC);
	if (ret)
		goto err_unref;

	ret = i915_gem_obj_ggtt_pin(ring->scratch.obj, 4096, 0);
	if (ret)
		goto err_unref;

	ring->scratch.gtt_offset = i915_gem_obj_ggtt_offset(ring->scratch.obj);
	ring->scratch.cpu_page = kmap(sg_page(ring->scratch.obj->pages->sgl));
	if (ring->scratch.cpu_page == NULL) {
		ret = -ENOMEM;
		goto err_unpin;
	}

	DRM_DEBUG_DRIVER("%s pipe control offset: 0x%08x\n",
			 ring->name, ring->scratch.gtt_offset);
	return 0;

err_unpin:
	i915_gem_object_ggtt_unpin(ring->scratch.obj);
err_unref:
	drm_gem_object_unreference(&ring->scratch.obj->base);
err:
	return ret;
}

u32
get_pipe_control_scratch_addr(struct intel_engine_cs *ring)
{
	if (ring->scratch.obj == NULL)
		return 0;

	return ring->scratch.gtt_offset;
}

static int intel_ring_workarounds_emit(struct intel_engine_cs *ring,
				       struct intel_context *ctx)
{
	int ret, i;
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_workarounds *w = &dev_priv->workarounds;

	if (WARN_ON(w->count == 0))
		return 0;

	ring->gpu_caches_dirty = true;
	ret = intel_ring_flush_all_caches(ring);
	if (ret)
		return ret;

	ret = intel_ring_begin(ring, w->count * 3);
	if (ret)
		return ret;

	for (i = 0; i < w->count; i++) {
		intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
		intel_ring_emit(ring, w->reg[i].addr);
		intel_ring_emit(ring, w->reg[i].value);
	}

	intel_ring_advance(ring);

	ring->gpu_caches_dirty = true;
	ret = intel_ring_flush_all_caches(ring);
	if (ret)
		return ret;

	DRM_DEBUG_DRIVER("Number of Workarounds emitted: %d\n", w->count);

	return 0;
}

static int wa_add(struct drm_i915_private *dev_priv,
		  const u32 addr, const u32 val, const u32 mask)
{
	const u32 idx = dev_priv->workarounds.count;

	if (WARN_ON(idx >= I915_MAX_WA_REGS))
		return -ENOSPC;

	dev_priv->workarounds.reg[idx].addr = addr;
	dev_priv->workarounds.reg[idx].value = val;
	dev_priv->workarounds.reg[idx].mask = mask;

	dev_priv->workarounds.count++;

	return 0;
}

#define WA_REG(addr, val, mask) { \
		const int r = wa_add(dev_priv, (addr), (val), (mask)); \
		if (r) \
			return r; \
	}

#define WA_SET_BIT_MASKED(addr, mask) WA_REG(addr, \
				    _MASKED_BIT_ENABLE(mask), (mask) & 0xffff)

#define WA_CLR_BIT_MASKED(addr, mask) WA_REG(addr, \
				    _MASKED_BIT_DISABLE(mask), (mask) & 0xffff)

#define WA_SET_BIT(addr, mask) WA_REG(addr, I915_READ(addr) | (mask), mask)
#define WA_CLR_BIT(addr, mask) WA_REG(addr, I915_READ(addr) & ~(mask), mask)

#define WA_WRITE(addr, val) WA_REG(addr, val, 0xffffffff)

static int bdw_init_workarounds(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* WaDisablePartialInstShootdown:bdw */
	/* WaDisableThreadStallDopClockGating:bdw (pre-production) */
	WA_SET_BIT_MASKED(GEN8_ROW_CHICKEN,
		  PARTIAL_INSTRUCTION_SHOOTDOWN_DISABLE |
		  STALL_DOP_GATING_DISABLE);

	/* WaDisableDopClockGating:bdw */
	WA_SET_BIT_MASKED(GEN7_ROW_CHICKEN2,
		  DOP_CLOCK_GATING_DISABLE);

	WA_SET_BIT_MASKED(HALF_SLICE_CHICKEN3,
			  GEN8_SAMPLER_POWER_BYPASS_DIS);

	/* Use Force Non-Coherent whenever executing a 3D context. This is a
	 * workaround for for a possible hang in the unlikely event a TLB
	 * invalidation occurs during a PSD flush.
	 */
	/* WaDisableFenceDestinationToSLM:bdw (GT3 pre-production) */
	WA_SET_BIT_MASKED(HDC_CHICKEN0,
			  HDC_FORCE_NON_COHERENT |
			  (IS_BDW_GT3(dev) ? HDC_FENCE_DEST_SLM_DISABLE : 0));

	/* Wa4x4STCOptimizationDisable:bdw */
	WA_SET_BIT_MASKED(CACHE_MODE_1,
		  GEN8_4x4_STC_OPTIMIZATION_DISABLE);

	/*
	 * BSpec recommends 8x4 when MSAA is used,
	 * however in practice 16x4 seems fastest.
	 *
	 * Note that PS/WM thread counts depend on the WIZ hashing
	 * disable bit, which we don't touch here, but it's good
	 * to keep in mind (see 3DSTATE_PS and 3DSTATE_WM).
	 */
	WA_SET_BIT_MASKED(GEN7_GT_MODE,
		  GEN6_WIZ_HASHING_MASK | GEN6_WIZ_HASHING_16x4);

	return 0;
}

static int chv_init_workarounds(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* WaDisablePartialInstShootdown:chv */
	/* WaDisableThreadStallDopClockGating:chv */
	/* WaClearFlowControlGpgpuContextSave:chv */
	WA_SET_BIT_MASKED(GEN8_ROW_CHICKEN,
			  PARTIAL_INSTRUCTION_SHOOTDOWN_DISABLE |
			  FLOW_CONTROL_ENABLE |
			  STALL_DOP_GATING_DISABLE);

	/* Use Force Non-Coherent whenever executing a 3D context. This is a
	 * workaround for a possible hang in the unlikely event a TLB
	 * invalidation occurs during a PSD flush.
	 */
	/* WaForceEnableNonCoherent:chv */
	/* WaHdcDisableFetchWhenMasked:chv */
	WA_SET_BIT_MASKED(HDC_CHICKEN0,
			  HDC_FORCE_NON_COHERENT |
			  HDC_DONOT_FETCH_MEM_WHEN_MASKED);

	/* Wa4x4STCOptimizationDisable:chv */
	WA_SET_BIT_MASKED(CACHE_MODE_1, GEN8_4x4_STC_OPTIMIZATION_DISABLE);

	return 0;
}

int init_workarounds_ring(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	WARN_ON(ring->id != RCS);

	dev_priv->workarounds.count = 0;

	if (IS_BROADWELL(dev))
		return bdw_init_workarounds(ring);

	if (IS_CHERRYVIEW(dev))
		return chv_init_workarounds(ring);

	return 0;
}

static int init_render_ring(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 imr;
	int ret = init_ring_common(ring);
	if (ret)
		return ret;

	/* WaTimedSingleVertexDispatch:cl,bw,ctg,elk,ilk,snb */
	if (INTEL_INFO(dev)->gen >= 4 && INTEL_INFO(dev)->gen < 7)
		I915_WRITE(MI_MODE, _MASKED_BIT_ENABLE(VS_TIMER_DISPATCH));

	/* We need to disable the AsyncFlip performance optimisations in order
	 * to use MI_WAIT_FOR_EVENT within the CS. It should already be
	 * programmed to '1' on all products.
	 *
	 * WaDisableAsyncFlipPerfMode:snb,ivb,hsw,vlv,bdw,chv
	 */
	if (INTEL_INFO(dev)->gen >= 6)
		I915_WRITE(MI_MODE, _MASKED_BIT_ENABLE(ASYNC_FLIP_PERF_DISABLE));

	/* Required for the hardware to program scanline values for waiting */
	/* WaEnableFlushTlbInvalidationMode:snb */
	if (INTEL_INFO(dev)->gen == 6)
		I915_WRITE(GFX_MODE,
			   _MASKED_BIT_ENABLE(GFX_TLB_INVALIDATE_EXPLICIT));

	/* WaBCSVCSTlbInvalidationMode:ivb,vlv,hsw */
	if (IS_GEN7(dev))
		I915_WRITE(GFX_MODE_GEN7,
			   _MASKED_BIT_ENABLE(GFX_TLB_INVALIDATE_EXPLICIT) |
			   _MASKED_BIT_ENABLE(GFX_REPLAY_MODE));

	if (IS_GEN6(dev)) {
		/* From the Sandybridge PRM, volume 1 part 3, page 24:
		 * "If this bit is set, STCunit will have LRA as replacement
		 *  policy. [...] This bit must be reset.  LRA replacement
		 *  policy is not supported."
		 */
		I915_WRITE(CACHE_MODE_0,
			   _MASKED_BIT_DISABLE(CM0_STC_EVICT_DISABLE_LRA_SNB));
	}

	if (INTEL_INFO(dev)->gen >= 6)
		I915_WRITE(INSTPM, _MASKED_BIT_ENABLE(INSTPM_FORCE_ORDERING));

	imr = ~0;
	if (HAS_L3_DPF(dev))
		imr &= ~GT_PARITY_ERROR(dev);
	if (IS_GEN7(dev))
		imr &= ~GT_RENDER_PERFMON_BUFFER_INTERRUPT;
	if (INTEL_INFO(dev)->gen >= 7)
		imr &= ~GT_GEN6_RENDER_WATCHDOG_INTERRUPT;
	I915_WRITE_IMR(ring, imr);

	return init_workarounds_ring(ring);
}

static void render_ring_cleanup(struct intel_engine_cs *ring)
{
	intel_fini_pipe_control(ring);
}

static int gen6_signal(struct intel_engine_cs *signaller,
		       unsigned int num_dwords)
{
	struct drm_device *dev = signaller->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *useless;
	int i, ret;

	/* NB: In order to be able to do semaphore MBOX updates for varying
	 * number of rings, it's easiest if we round up each individual update
	 * to a multiple of 2 (since ring updates must always be a multiple of
	 * 2) even though the actual update only requires 3 dwords.
	 */
#define MBOX_UPDATE_DWORDS 4
	if (i915_semaphore_is_enabled(dev))
		num_dwords += ((I915_NUM_RINGS-1) * MBOX_UPDATE_DWORDS);
	else
		return intel_ring_begin(signaller, num_dwords);

	ret = intel_ring_begin(signaller, num_dwords);
	if (ret)
		return ret;
#undef MBOX_UPDATE_DWORDS

	for_each_ring(useless, dev_priv, i) {
		u32 mbox_reg = signaller->semaphore.mbox.signal[i];
		if (mbox_reg != GEN6_NOSYNC) {
			u32 seqno = i915_gem_request_get_seqno(
					   signaller->outstanding_lazy_request);
			intel_ring_emit(signaller, MI_LOAD_REGISTER_IMM(1));
			intel_ring_emit(signaller, mbox_reg);
			intel_ring_emit(signaller, seqno);
			intel_ring_emit(signaller, MI_NOOP);
		} else {
			intel_ring_emit(signaller, MI_NOOP);
			intel_ring_emit(signaller, MI_NOOP);
			intel_ring_emit(signaller, MI_NOOP);
			intel_ring_emit(signaller, MI_NOOP);
		}
	}

	return 0;
}

/**
 * gen6_add_request - Update the semaphore mailbox registers
 * 
 * @ring - ring that is adding a request
 * @seqno - return seqno stuck into the ring
 *
 * Update the mailbox registers in the *other* rings with the current seqno.
 * This acts like a signal in the canonical semaphore.
 */
static int
gen6_add_request(struct intel_engine_cs *ring)
{
	int ret;

	ret = ring->semaphore.signal(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
	intel_ring_emit(ring, I915_GEM_HWS_INDEX << MI_STORE_DWORD_INDEX_SHIFT);
	intel_ring_emit(ring,
		    i915_gem_request_get_seqno(ring->outstanding_lazy_request));
	intel_ring_emit(ring, MI_USER_INTERRUPT);
	__intel_ring_advance(ring);

	return 0;
}

static inline bool i915_gem_has_seqno_wrapped(struct drm_device *dev,
					      u32 seqno)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	return dev_priv->last_seqno < seqno;
}

/**
 * intel_ring_sync - sync the waiter to the signaller on seqno
 *
 * @waiter - ring that is waiting
 * @signaller - ring which has, or will signal
 * @seqno - seqno which the waiter will block on
 */
static int
gen6_ring_sync(struct intel_engine_cs *waiter,
	       struct intel_engine_cs *signaller,
	       u32 seqno)
{
	u32 dw1 = MI_SEMAPHORE_MBOX |
		  MI_SEMAPHORE_COMPARE |
		  MI_SEMAPHORE_REGISTER;
	u32 wait_mbox = signaller->semaphore.mbox.wait[waiter->id];
	int ret;

	/* Arithmetic on sequence numbers is unreliable with a scheduler. */
	BUG_ON(i915_scheduler_is_enabled(signaller->dev));

	/* Throughout all of the GEM code, seqno passed implies our current
	 * seqno is >= the last seqno executed. However for hardware the
	 * comparison is strictly greater than.
	 */
	seqno -= 1;

	WARN_ON(wait_mbox == MI_SEMAPHORE_SYNC_INVALID);

	ret = intel_ring_begin(waiter, 4);
	if (ret)
		return ret;

	/* If seqno wrap happened, omit the wait with no-ops */
	if (likely(!i915_gem_has_seqno_wrapped(waiter->dev, seqno))) {
		intel_ring_emit(waiter, dw1 | wait_mbox);
		intel_ring_emit(waiter, seqno);
		intel_ring_emit(waiter, 0);
		intel_ring_emit(waiter, MI_NOOP);
	} else {
		intel_ring_emit(waiter, MI_NOOP);
		intel_ring_emit(waiter, MI_NOOP);
		intel_ring_emit(waiter, MI_NOOP);
		intel_ring_emit(waiter, MI_NOOP);
	}
	intel_ring_advance(waiter);

	return 0;
}

#define PIPE_CONTROL_FLUSH(ring__, addr__)					\
do {									\
	intel_ring_emit(ring__, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |		\
		 PIPE_CONTROL_DEPTH_STALL);				\
	intel_ring_emit(ring__, (addr__) | PIPE_CONTROL_GLOBAL_GTT);			\
	intel_ring_emit(ring__, 0);							\
	intel_ring_emit(ring__, 0);							\
} while (0)

static int
pc_render_add_request(struct intel_engine_cs *ring)
{
	u32 scratch_addr = ring->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	int ret;

	/* For Ironlake, MI_USER_INTERRUPT was deprecated and apparently
	 * incoherent with writes to memory, i.e. completely fubar,
	 * so we need to use PIPE_NOTIFY instead.
	 *
	 * However, we also need to workaround the qword write
	 * incoherence by flushing the 6 PIPE_NOTIFY buffers out to
	 * memory before requesting an interrupt.
	 */
	ret = intel_ring_begin(ring, 32);
	if (ret)
		return ret;

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |
			PIPE_CONTROL_WRITE_FLUSH |
			PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE);
	intel_ring_emit(ring, ring->scratch.gtt_offset | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring,
		    i915_gem_request_get_seqno(ring->outstanding_lazy_request));
	intel_ring_emit(ring, 0);
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 2 * CACHELINE_BYTES; /* write to separate cachelines */
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 2 * CACHELINE_BYTES;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 2 * CACHELINE_BYTES;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 2 * CACHELINE_BYTES;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);
	scratch_addr += 2 * CACHELINE_BYTES;
	PIPE_CONTROL_FLUSH(ring, scratch_addr);

	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(4) | PIPE_CONTROL_QW_WRITE |
			PIPE_CONTROL_WRITE_FLUSH |
			PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE |
			PIPE_CONTROL_NOTIFY);
	intel_ring_emit(ring, ring->scratch.gtt_offset | PIPE_CONTROL_GLOBAL_GTT);
	intel_ring_emit(ring,
		    i915_gem_request_get_seqno(ring->outstanding_lazy_request));
	intel_ring_emit(ring, 0);
	__intel_ring_advance(ring);

	return 0;
}

static u32
gen6_ring_get_seqno(struct intel_engine_cs *ring, bool lazy_coherency)
{
	/* Workaround to force correct ordering between irq and seqno writes on
	 * ivb (and maybe also on snb) by reading from a CS register (like
	 * ACTHD) before reading the status page. */
	if (!lazy_coherency) {
		struct drm_i915_private *dev_priv = ring->dev->dev_private;
		POSTING_READ(RING_ACTHD(ring->mmio_base));
	}

	return intel_read_status_page(ring, I915_GEM_HWS_INDEX);
}

static u32
ring_get_seqno(struct intel_engine_cs *ring, bool lazy_coherency)
{
	return intel_read_status_page(ring, I915_GEM_HWS_INDEX);
}

static void
ring_set_seqno(struct intel_engine_cs *ring, u32 seqno)
{
	intel_write_status_page(ring, I915_GEM_HWS_INDEX, seqno);
}

static u32
pc_render_get_seqno(struct intel_engine_cs *ring, bool lazy_coherency)
{
	return ring->scratch.cpu_page[0];
}

static void
pc_render_set_seqno(struct intel_engine_cs *ring, u32 seqno)
{
	ring->scratch.cpu_page[0] = seqno;
}

static bool
gen5_ring_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0)
		ilk_enable_gt_irq(dev_priv, ring->irq_enable_mask);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
gen5_ring_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0)
		ilk_disable_gt_irq(dev_priv, ring->irq_enable_mask);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
i9xx_ring_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		dev_priv->irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE(IMR, dev_priv->irq_mask);
		POSTING_READ(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
i9xx_ring_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		dev_priv->irq_mask |= ring->irq_enable_mask;
		I915_WRITE(IMR, dev_priv->irq_mask);
		POSTING_READ(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
i8xx_ring_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		dev_priv->irq_mask &= ~ring->irq_enable_mask;
		I915_WRITE16(IMR, dev_priv->irq_mask);
		POSTING_READ16(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
i8xx_ring_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		dev_priv->irq_mask |= ring->irq_enable_mask;
		I915_WRITE16(IMR, dev_priv->irq_mask);
		POSTING_READ16(IMR);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

/* gen6_ring_invalidate_tlb
 * GFX soft resets do not invalidate TLBs, it is up to
 * GFX driver to explicitly invalidate TLBs post reset.
 */
static int gen6_ring_invalidate_tlb(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 reg;
	int ret;

	if ((INTEL_INFO(dev)->gen < 6) || (INTEL_INFO(dev)->gen >= 8) ||
			(!ring->stop) || (!ring->start))
		return -EINVAL;

	/* stop the ring before sync_flush */
	ret = ring->stop(ring);
	if ((ret) && (ret != -EALREADY))
		DRM_ERROR("%s: unable to stop the ring\n", ring->name);

	/* Invalidate TLB */
	reg = RING_INSTPM(ring->mmio_base);

	/* ring should be idle before issuing a sync flush */
	WARN_ON((I915_READ_MODE(ring) & RING_MODE_IDLE) == 0);

	I915_WRITE(reg, _MASKED_BIT_ENABLE(INSTPM_TLB_INVALIDATE |
				INSTPM_SYNC_FLUSH));
	if (wait_for((I915_READ(reg) & INSTPM_SYNC_FLUSH) == 0, 1000))
		DRM_ERROR("%s: wait for SyncFlush to complete timed out\n",
				ring->name);

	/* only start if stop was sucessfull */
	if (!ret)
		ring->start(ring);

	return 0;
}

void intel_ring_setup_status_page(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	u32 mmio = 0;

	/* The ring status page addresses are no longer next to the rest of
	 * the ring registers as of gen7.
	 */
	if (IS_GEN7(dev)) {
		switch (ring->id) {
		case RCS:
			mmio = RENDER_HWS_PGA_GEN7;
			break;
		case BCS:
			mmio = BLT_HWS_PGA_GEN7;
			break;
		/*
		 * VCS2 actually doesn't exist on Gen7. Only shut up
		 * gcc switch check warning
		 */
		case VCS2:
		case VCS:
			mmio = BSD_HWS_PGA_GEN7;
			break;
		case VECS:
			mmio = VEBOX_HWS_PGA_GEN7;
			break;
		}
	} else if (IS_GEN6(ring->dev)) {
		mmio = RING_HWS_PGA_GEN6(ring->mmio_base);
	} else {
		/* XXX: gen8 returns to sanity */
		mmio = RING_HWS_PGA(ring->mmio_base);
	}

	I915_WRITE(mmio, (u32)ring->status_page.gfx_addr);
	POSTING_READ(mmio);

	/*
	 * Flush the TLB for this page
	 *
	 * FIXME: These two bits have disappeared on gen8, so a question
	 * arises: do we still need this and if so how should we go about
	 * invalidating the TLB?
	 */
	if (INTEL_INFO(dev)->gen >= 6 && INTEL_INFO(dev)->gen < 8) {
		u32 reg = RING_INSTPM(ring->mmio_base);

		/* ring should be idle before issuing a sync flush*/
		WARN_ON((I915_READ_MODE(ring) & RING_MODE_IDLE) == 0);

		I915_WRITE(reg,
			   _MASKED_BIT_ENABLE(INSTPM_TLB_INVALIDATE |
					      INSTPM_SYNC_FLUSH));
		if (wait_for((I915_READ(reg) & INSTPM_SYNC_FLUSH) == 0,
			     1000))
			DRM_ERROR("%s: wait for SyncFlush to complete for TLB invalidation timed out\n",
				  ring->name);
	}
}

static int
bsd_ring_flush(struct intel_engine_cs *ring,
	       u32     invalidate_domains,
	       u32     flush_domains)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_FLUSH);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);
	return 0;
}

static int
i9xx_add_request(struct intel_engine_cs *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
	intel_ring_emit(ring, I915_GEM_HWS_INDEX << MI_STORE_DWORD_INDEX_SHIFT);
	intel_ring_emit(ring,
		    i915_gem_request_get_seqno(ring->outstanding_lazy_request));
	intel_ring_emit(ring, MI_USER_INTERRUPT);
	__intel_ring_advance(ring);

	return 0;
}

static bool
gen6_ring_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
	       return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		unsigned mask = I915_READ_IMR(ring);
		mask &= ~ring->irq_enable_mask;
		I915_WRITE_IMR(ring, mask);
		ilk_enable_gt_irq(dev_priv, ring->irq_enable_mask);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
gen6_ring_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		unsigned imr = I915_READ_IMR(ring);
		imr |= ring->irq_enable_mask;
		I915_WRITE_IMR(ring, imr);
		ilk_disable_gt_irq(dev_priv, ring->irq_enable_mask);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
hsw_vebox_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		I915_WRITE_IMR(ring, ~ring->irq_enable_mask);
		snb_enable_pm_irq(dev_priv, ring->irq_enable_mask);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
hsw_vebox_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		I915_WRITE_IMR(ring, ~0);
		snb_disable_pm_irq(dev_priv, ring->irq_enable_mask);
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static bool
gen8_ring_get_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (!dev->irq_enabled)
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (ring->irq_refcount++ == 0) {
		if (HAS_L3_DPF(dev) && ring->id == RCS) {
			I915_WRITE_IMR(ring,
				       ~(ring->irq_enable_mask |
					 GT_RENDER_L3_PARITY_ERROR_INTERRUPT));
		} else {
			I915_WRITE_IMR(ring, ~ring->irq_enable_mask);
		}
		POSTING_READ(RING_IMR(ring->mmio_base));
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void
gen8_ring_put_irq(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--ring->irq_refcount == 0) {
		if (HAS_L3_DPF(dev) && ring->id == RCS) {
			I915_WRITE_IMR(ring,
				       ~GT_RENDER_L3_PARITY_ERROR_INTERRUPT);
		} else {
			I915_WRITE_IMR(ring, ~0);
		}
		POSTING_READ(RING_IMR(ring->mmio_base));
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static int
i965_dispatch_execbuffer(struct intel_engine_cs *ring,
			 u64 offset, u32 length,
			 unsigned flags)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring,
			MI_BATCH_BUFFER_START |
			MI_BATCH_GTT |
			(flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE_I965));
	intel_ring_emit(ring, offset);
	intel_ring_advance(ring);

	return 0;
}

/* Just userspace ABI convention to limit the wa batch bo to a resonable size */
#define I830_BATCH_LIMIT (256*1024)
static int
i830_dispatch_execbuffer(struct intel_engine_cs *ring,
				u64 offset, u32 len,
				unsigned flags)
{
	int ret;

	if (flags & I915_DISPATCH_PINNED) {
		ret = intel_ring_begin(ring, 4);
		if (ret)
			return ret;

		intel_ring_emit(ring, MI_BATCH_BUFFER);
		intel_ring_emit(ring, offset | (flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE));
		intel_ring_emit(ring, offset + len - 8);
		intel_ring_emit(ring, MI_NOOP);
		intel_ring_advance(ring);
	} else {
		u32 cs_offset = ring->scratch.gtt_offset;

		if (len > I830_BATCH_LIMIT)
			return -ENOSPC;

		ret = intel_ring_begin(ring, 9+3);
		if (ret)
			return ret;
		/* Blit the batch (which has now all relocs applied) to the stable batch
		 * scratch bo area (so that the CS never stumbles over its tlb
		 * invalidation bug) ... */
		intel_ring_emit(ring, XY_SRC_COPY_BLT_CMD |
				XY_SRC_COPY_BLT_WRITE_ALPHA |
				XY_SRC_COPY_BLT_WRITE_RGB);
		intel_ring_emit(ring, BLT_DEPTH_32 | BLT_ROP_GXCOPY | 4096);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, (DIV_ROUND_UP(len, 4096) << 16) | 1024);
		intel_ring_emit(ring, cs_offset);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, 4096);
		intel_ring_emit(ring, offset);
		intel_ring_emit(ring, MI_FLUSH);

		/* ... and execute it. */
		intel_ring_emit(ring, MI_BATCH_BUFFER);
		intel_ring_emit(ring, cs_offset | (flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE));
		intel_ring_emit(ring, cs_offset + len - 8);
		intel_ring_advance(ring);
	}

	return 0;
}

static int
i915_dispatch_execbuffer(struct intel_engine_cs *ring,
			 u64 offset, u32 len,
			 unsigned flags)
{
	int ret;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_BATCH_BUFFER_START | MI_BATCH_GTT);
	intel_ring_emit(ring, offset | (flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE));
	intel_ring_advance(ring);

	return 0;
}

static void cleanup_status_page(struct intel_engine_cs *ring)
{
	struct drm_i915_gem_object *obj;

	obj = ring->status_page.obj;
	if (obj == NULL)
		return;

	kunmap(sg_page(obj->pages->sgl));
	i915_gem_object_ggtt_unpin(obj);
	drm_gem_object_unreference(&obj->base);
	ring->status_page.obj = NULL;
}

static int init_status_page(struct intel_engine_cs *ring)
{
	struct drm_i915_gem_object *obj;

	if ((obj = ring->status_page.obj) == NULL) {
		int ret;

		obj = i915_gem_alloc_object(ring->dev, 4096);
		if (obj == NULL) {
			DRM_ERROR("Failed to allocate status page\n");
			return -ENOMEM;
		}

		ret = i915_gem_object_set_cache_level(obj, I915_CACHE_LLC);
		if (ret)
			goto err_unref;

		ret = i915_gem_obj_ggtt_pin(obj, 4096, 0);
		if (ret) {
err_unref:
			drm_gem_object_unreference(&obj->base);
			return ret;
		}

		ring->status_page.obj = obj;
	}

	ring->status_page.gfx_addr = i915_gem_obj_ggtt_offset(obj);
	ring->status_page.page_addr = kmap(sg_page(obj->pages->sgl));
	memset(ring->status_page.page_addr, 0, PAGE_SIZE);

	DRM_DEBUG_DRIVER("%s hws offset: 0x%08x\n",
			ring->name, ring->status_page.gfx_addr);

	return 0;
}

static int init_phys_status_page(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;

	if (!dev_priv->status_page_dmah) {
		dev_priv->status_page_dmah =
			drm_pci_alloc(ring->dev, PAGE_SIZE, PAGE_SIZE);
		if (!dev_priv->status_page_dmah)
			return -ENOMEM;
	}

	ring->status_page.page_addr = dev_priv->status_page_dmah->vaddr;
	memset(ring->status_page.page_addr, 0, PAGE_SIZE);

	return 0;
}

void intel_unpin_ringbuffer_obj(struct intel_ringbuffer *ringbuf)
{
	iounmap(ringbuf->virtual_start);
	ringbuf->virtual_start = NULL;
	i915_gem_object_ggtt_unpin(ringbuf->obj);
}

int intel_pin_and_map_ringbuffer_obj(struct drm_device *dev,
				     struct intel_ringbuffer *ringbuf)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_i915_gem_object *obj = ringbuf->obj;
	int ret;

	ret = i915_gem_obj_ggtt_pin(obj, PAGE_SIZE, PIN_MAPPABLE);
	if (ret)
		return ret;

	ret = i915_gem_object_set_to_gtt_domain(obj, true);
	if (ret) {
		i915_gem_object_ggtt_unpin(obj);
		return ret;
	}

	ringbuf->virtual_start = ioremap_wc(dev_priv->gtt.mappable_base +
			i915_gem_obj_ggtt_offset(obj), ringbuf->size);
	if (ringbuf->virtual_start == NULL) {
		i915_gem_object_ggtt_unpin(obj);
		return -EINVAL;
	}

	return 0;
}

void intel_destroy_ringbuffer_obj(struct intel_ringbuffer *ringbuf)
{
	drm_gem_object_unreference(&ringbuf->obj->base);
	ringbuf->obj = NULL;
}

int intel_alloc_ringbuffer_obj(struct drm_device *dev,
			       struct intel_ringbuffer *ringbuf)
{
	struct drm_i915_gem_object *obj;

	obj = NULL;
	if (!HAS_LLC(dev))
		obj = i915_gem_object_create_stolen(dev, ringbuf->size);
	if (obj == NULL)
		obj = i915_gem_alloc_object(dev, ringbuf->size);
	if (obj == NULL)
		return -ENOMEM;

	/* mark ring buffers as read-only from GPU side by default */
	obj->gt_ro = 1;

	ringbuf->obj = obj;

	return 0;
}

static int intel_init_ring_buffer(struct drm_device *dev,
				  struct intel_engine_cs *ring)
{
	struct intel_ringbuffer *ringbuf = ring->buffer;
	int ret;

	if (ringbuf == NULL) {
		ringbuf = kzalloc(sizeof(*ringbuf), GFP_KERNEL);
		if (!ringbuf)
			return -ENOMEM;
		ring->buffer = ringbuf;
	}

	ring->dev = dev;
	INIT_LIST_HEAD(&ring->active_list);
	INIT_LIST_HEAD(&ring->request_list);
	spin_lock_init(&ring->reqlist_lock);
	INIT_LIST_HEAD(&ring->delayed_free_list);
	INIT_LIST_HEAD(&ring->execlist_queue);
	ringbuf->size = 32 * PAGE_SIZE;
	ringbuf->ring = ring;
	memset(ring->semaphore.sync_seqno, 0, sizeof(ring->semaphore.sync_seqno));

	init_waitqueue_head(&ring->irq_queue);

	if (I915_NEED_GFX_HWS(dev)) {
		ret = init_status_page(ring);
		if (ret)
			goto error;
	} else {
		BUG_ON(ring->id != RCS);
		ret = init_phys_status_page(ring);
		if (ret)
			goto error;
	}

	if (ringbuf->obj == NULL) {
		ret = intel_alloc_ringbuffer_obj(dev, ringbuf);
		if (ret) {
			DRM_ERROR("Failed to allocate ringbuffer %s: %d\n",
					ring->name, ret);
			goto error;
		}

		ret = intel_pin_and_map_ringbuffer_obj(dev, ringbuf);
		if (ret) {
			DRM_ERROR("Failed to pin and map ringbuffer %s: %d\n",
					ring->name, ret);
			intel_destroy_ringbuffer_obj(ringbuf);
			goto error;
		}
	}

	/* Workaround an erratum on the i830 which causes a hang if
	 * the TAIL pointer points to within the last 2 cachelines
	 * of the buffer.
	 */
	ringbuf->effective_size = ringbuf->size;
	if (IS_I830(dev) || IS_845G(dev))
		ringbuf->effective_size -= 2 * CACHELINE_BYTES;

	ret = i915_cmd_parser_init_ring(ring);
	if (ret)
		goto error;

	ret = ring->init(ring);
	if (ret)
		goto error;

	return 0;

error:
	kfree(ringbuf);
	ring->buffer = NULL;
	return ret;
}

void intel_cleanup_ring_buffer(struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv;
	struct intel_ringbuffer *ringbuf;

	if (!intel_ring_initialized(ring))
		return;

	dev_priv = to_i915(ring->dev);
	ringbuf = ring->buffer;

	intel_stop_ring_buffer(ring);
	WARN_ON(!IS_GEN2(ring->dev) &&
			(I915_READ_MODE(ring) & RING_MODE_IDLE) == 0);

	intel_unpin_ringbuffer_obj(ringbuf);
	intel_destroy_ringbuffer_obj(ringbuf);
	i915_gem_request_assign(&ring->outstanding_lazy_request, NULL);

	if (ring->cleanup)
		ring->cleanup(ring);

	cleanup_status_page(ring);

	i915_cmd_parser_fini_ring(ring);

	kfree(ringbuf);
	ring->buffer = NULL;
}

/* Write a specific seqno value to the HWS page so that
 * we can identify the cause of any hangs. NB: req can be
 * null in the case of clearing the active request, in this
 * case, a seqno of zero is written. */
int i915_write_active_request(struct intel_engine_cs *ring,
			      struct drm_i915_gem_request *req)
{
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
	intel_ring_emit(ring, I915_GEM_ACTIVE_SEQNO_INDEX <<
			MI_STORE_DWORD_INDEX_SHIFT);
	intel_ring_emit(ring, i915_gem_request_get_seqno(req));
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

static int intel_ring_wait_request(struct intel_engine_cs *ring, int n)
{
	struct intel_ringbuffer *ringbuf = ring->buffer;
	struct drm_i915_gem_request *request;
	int ret;

	if (intel_ring_space(ringbuf) >= n)
		return 0;

	list_for_each_entry(request, &ring->request_list, list) {
		if (__intel_ring_space(request->tail, ringbuf->tail,
				       ringbuf->size) >= n) {
			break;
		}
	}

	if (&request->list == &ring->request_list)
		return -ENOSPC;

	ret = i915_wait_request(request);
	if (ret)
		return ret;

	i915_gem_retire_requests_ring(ring);

	return 0;
}

static int ring_wait_for_space(struct intel_engine_cs *ring, int n)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ringbuffer *ringbuf = ring->buffer;
	unsigned long end;
	int ret;

	ret = intel_ring_wait_request(ring, n);
	if (ret != -ENOSPC)
		return ret;

	/* force the tail write in case we have been skipping them */
	__intel_ring_advance(ring);

	/*
	 * With GEM the hang check should kick us out of the loop,
	 * leaving it early runs the risk of corrupting GEM state (due
	 * to running on almost untested codepaths). But on resume
	 * timers don't work yet, so prevent a complete hang in that
	 * case by choosing an insanely large timeout.
	 */
	end = jiffies + 60 * HZ;

	ret = 0;
	trace_i915_ring_wait_begin(ring);
	do {
		if (intel_ring_space(ringbuf) >= n)
			break;

		ringbuf->head = I915_READ_HEAD(ring);
		if (intel_ring_space(ringbuf) >= n)
			break;

		if (!drm_core_check_feature(dev, DRIVER_MODESET) &&
		    dev->primary->master) {
			struct drm_i915_master_private *master_priv = dev->primary->master->driver_priv;
			if (master_priv->sarea_priv)
				master_priv->sarea_priv->perf_boxes |= I915_BOX_WAIT;
		}

		msleep(1);

		if (dev_priv->mm.interruptible && signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		ret = i915_gem_check_wedge(&dev_priv->gpu_error,
					   dev_priv->mm.interruptible, ring);
		if (ret)
			break;

		if (time_after(jiffies, end)) {
			ret = -EBUSY;
			break;
		}
	} while (1);
	trace_i915_ring_wait_end(ring);
	return ret;
}

static int intel_wrap_ring_buffer(struct intel_engine_cs *ring)
{
	uint32_t __iomem *virt;
	struct intel_ringbuffer *ringbuf = ring->buffer;
	int rem = ringbuf->size - ringbuf->tail;

	if (ringbuf->space < rem) {
		int ret = ring_wait_for_space(ring, rem);
		if (ret)
			return ret;
	}

	virt = ringbuf->virtual_start + ringbuf->tail;
	rem /= 4;
	while (rem--)
		iowrite32(MI_NOOP, virt++);

	ringbuf->tail = 0;
	intel_ring_update_space(ringbuf);

	return 0;
}

int intel_ring_idle(struct intel_engine_cs *ring, bool flush)
{
	struct drm_i915_gem_request *req;
	int ret = 0;

	if (flush) {
		/* We need to add any requests required to flush the objects and ring */
		if (ring->outstanding_lazy_request) {
			ret = i915_add_request(ring);
			if (ret)
				return ret;
		}

		/* If there is anything outstanding within the scheduler then
		 * give up now as the submission of such work requires the
		 * mutex lock. While the lock is definitely held at this point
		 * (i915_wait_request will BUG if called without), the driver
		 * is not necessarily at a safe point to start submitting ring
		 * work. */
		if (!i915_scheduler_is_ring_idle(ring))
			return -EAGAIN;
	}

	/* Wait upon the last request to be completed.
	 *
	 * NB: With a scheduler, requests might complete out of order (or
	 * even be pre-empted and not complete at all). Thus the 'last'
	 * request could change between it being the wait starting and the
	 * wait completing. Hence a loop while not empty is required. */
	while(!list_empty(&ring->request_list)) {
		req = list_entry(ring->request_list.prev,
				 struct drm_i915_gem_request,
				 list);

		ret = i915_wait_request(req);
		if (ret)
			return ret;

		i915_gem_retire_requests_ring(ring);
	}

	return 0;
}

int
intel_ring_alloc_request(struct intel_engine_cs *ring, struct intel_context *ctx)
{
	struct drm_i915_gem_request *request;
	struct drm_i915_private *dev_private = ring->dev->dev_private;

	WARN_ON(ctx == NULL);

	if (ring->outstanding_lazy_request)
		return 0;

	request = kzalloc(sizeof(*request), GFP_KERNEL);
	if (request == NULL)
		return -ENOMEM;

	kref_init(&request->ref);
	request->ring = ring;
	request->ringbuf = ring->buffer;
	request->ctx = ctx;
	i915_gem_context_reference(request->ctx);
	request->uniq = dev_private->request_uniq++;

	ring->outstanding_lazy_request = request;
	return 0;
}

static int __intel_ring_prepare(struct intel_engine_cs *ring,
				int bytes)
{
	struct intel_ringbuffer *ringbuf = ring->buffer;
	int ret;

	if (unlikely(ringbuf->tail + bytes > ringbuf->effective_size)) {
		ret = intel_wrap_ring_buffer(ring);
		if (unlikely(ret))
			return ret;
	}

	if (unlikely(ringbuf->space < bytes)) {
		ret = ring_wait_for_space(ring, bytes);
		if (unlikely(ret))
			return ret;
	}

	return 0;
}

int intel_ring_begin(struct intel_engine_cs *ring,
		     int num_dwords)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	struct intel_ringbuffer *ringbuf = ring->buffer;
	int ret;

	ret = i915_gem_check_wedge(&dev_priv->gpu_error,
				   dev_priv->mm.interruptible, ring);
	if (ret)
		return ret;

	ret = __intel_ring_prepare(ring, num_dwords * sizeof(uint32_t));
	if (ret)
		return ret;

	/* Preallocate the olr before touching the ring */
	ret = intel_ring_alloc_request(ring, ring->default_context);
	if (ret)
		return ret;

	ringbuf->space -= num_dwords * sizeof(uint32_t);
	return 0;
}

/* Align the ring tail to a cacheline boundary */
int intel_ring_cacheline_align(struct intel_engine_cs *ring)
{
	int num_dwords = (ring->buffer->tail & (CACHELINE_BYTES - 1)) / sizeof(uint32_t);
	int ret;

	if (num_dwords == 0)
		return 0;

	num_dwords = CACHELINE_BYTES / sizeof(uint32_t) - num_dwords;
	ret = intel_ring_begin(ring, num_dwords);
	if (ret)
		return ret;

	while (num_dwords--)
		intel_ring_emit(ring, MI_NOOP);

	intel_ring_advance(ring);

	return 0;
}

/* Test to see if the ring has sufficient space to submit a given piece of work
 * without causing a stall */
int intel_ring_test_space(struct intel_engine_cs *ring, int min_space)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;
	struct intel_ringbuffer *ringbuf  = ring->buffer;

	/* There is a separate LRC version of this code. */
	BUG_ON(i915.enable_execlists);

	if (ringbuf->space < min_space) {
		/* Need to update the actual ring space. Otherwise, the system
		 * hangs forever testing a software copy of the space value that
		 * never changes!
		 */
		ringbuf->head  = I915_READ_HEAD(ring);
		ringbuf->space = intel_ring_space(ringbuf);

		if (ringbuf->space < min_space)
			return -EAGAIN;
	}

	return 0;
}

void intel_ring_init_seqno(struct intel_engine_cs *ring, u32 seqno)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* It is safe to have work pending in the OLR but only if it has not
	 * yet had a seqno assigned. */
	BUG_ON(ring->outstanding_lazy_request && ring->outstanding_lazy_request->seqno);

	if (INTEL_INFO(dev)->gen == 6 || INTEL_INFO(dev)->gen == 7) {
		I915_WRITE(RING_SYNC_0(ring->mmio_base), 0);
		I915_WRITE(RING_SYNC_1(ring->mmio_base), 0);
		if (HAS_VEBOX(dev))
			I915_WRITE(RING_SYNC_2(ring->mmio_base), 0);
	}

	ring->set_seqno(ring, seqno);
	ring->last_read_seqno = 0;
}

static void gen6_bsd_ring_write_tail(struct intel_engine_cs *ring,
				     u32 value)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;

       /* Every tail move must follow the sequence below */

	/* Disable notification that the ring is IDLE. The GT
	 * will then assume that it is busy and bring it out of rc6.
	 */
	I915_WRITE(GEN6_BSD_SLEEP_PSMI_CONTROL,
		   _MASKED_BIT_ENABLE(GEN6_BSD_SLEEP_MSG_DISABLE));

	/* Clear the context id. Here be magic! */
	I915_WRITE64(GEN6_BSD_RNCID, 0x0);

	/* Wait for the ring not to be idle, i.e. for it to wake up. */
	if (wait_for((I915_READ(GEN6_BSD_SLEEP_PSMI_CONTROL) &
		      GEN6_BSD_SLEEP_INDICATOR) == 0,
		     50))
		DRM_ERROR("timed out waiting for the BSD ring to wake up\n");

	/* Now that the ring is fully powered up, update the tail */
	I915_WRITE_TAIL(ring, value);
	POSTING_READ(RING_TAIL(ring->mmio_base));

	/* Let the ring send IDLE messages to the GT again,
	 * and so let it sleep to conserve power when idle.
	 */
	I915_WRITE(GEN6_BSD_SLEEP_PSMI_CONTROL,
		   _MASKED_BIT_DISABLE(GEN6_BSD_SLEEP_MSG_DISABLE));
}

static int gen6_bsd_ring_flush(struct intel_engine_cs *ring,
			       u32 invalidate, u32 flush)
{
	uint32_t cmd;
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	cmd = MI_FLUSH_DW;
	if (INTEL_INFO(ring->dev)->gen >= 8)
		cmd += 1;
	/*
	 * Bspec vol 1c.5 - video engine command streamer:
	 * "If ENABLED, all TLBs will be invalidated once the flush
	 * operation is complete. This bit is only valid when the
	 * Post-Sync Operation field is a value of 1h or 3h."
	 */
	if (invalidate & I915_GEM_GPU_DOMAINS)
		cmd |= MI_INVALIDATE_TLB | MI_INVALIDATE_BSD |
			MI_FLUSH_DW_STORE_INDEX | MI_FLUSH_DW_OP_STOREDW;
	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_ADDR | MI_FLUSH_DW_USE_GTT);
	if (INTEL_INFO(ring->dev)->gen >= 8) {
		intel_ring_emit(ring, 0); /* upper addr */
		intel_ring_emit(ring, 0); /* value */
	} else  {
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
	}
	intel_ring_advance(ring);
	return 0;
}

static int
gen8_pipe_control_disable_protected_mem(struct intel_engine_cs *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 8);
	if (ret)
		return ret;

	/* Pipe Control */
	intel_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));	/* DW0 */
	intel_ring_emit(ring, 0x81010a0);	/* DW1 - Disable
							protect mem */
	intel_ring_emit(ring, 0);		/* DW2 */
	intel_ring_emit(ring, 0);		/* DW3 */
	intel_ring_emit(ring, 0);		/* DW4 */
	intel_ring_emit(ring, 0);		/* DW5 */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_advance(ring);

	return 0;
}

static int
gen8_ring_dispatch_execbuffer(struct intel_engine_cs *ring,
			      u64 offset, u32 len,
			      unsigned flags)
{
	bool ppgtt = USES_PPGTT(ring->dev) && !(flags & I915_DISPATCH_SECURE);
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	/* FIXME(BDW): Address space and security selectors. */
	intel_ring_emit(ring, MI_BATCH_BUFFER_START_GEN8 | (ppgtt<<8) |
			(flags &
			 I915_DISPATCH_RS ? MI_BATCH_RESOURCE_STREAMER : 0));
	intel_ring_emit(ring, lower_32_bits(offset));
	intel_ring_emit(ring, upper_32_bits(offset));
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	/* Send pipe control with protected memory disable if requested */
	if (flags & I915_DISPATCH_LAUNCH_CB2) {
		ret = gen8_pipe_control_disable_protected_mem(ring);
		if (ret)
			return ret;
	}

	return ret;
}

static int
launch_cb2(struct intel_engine_cs *ring)
{
	int			i;
	int			ret = 0;
	uint32_t		hws_pga;
	struct drm_i915_private	*dev_priv = ring->dev->dev_private;

	/* Get HW Status Page address & point to its center */
	hws_pga = 0x800 + (I915_READ(RENDER_HWS_PGA_GEN7) & 0xFFFFF000);

	ret = intel_ring_begin(ring, 8);
	if (ret)
		return ret;

	/* Pipe Control */
	intel_ring_emit(ring, 0x7a000003);	/* PipeControl DW0 */
	intel_ring_emit(ring, 0x01510bc);	/* DW1 */
	intel_ring_emit(ring, 0);		/* DW2 */
	intel_ring_emit(ring, 0);		/* DW3 */
	intel_ring_emit(ring, 0);		/* DW4 */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_advance(ring);

	/* Insert 20 Store Data Immediate commands */
	for (i = 0; i < 20; i++) {
		ret = intel_ring_begin(ring, 4);
		if (ret)
			return ret;

		intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
		intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_INDEX <<
					MI_STORE_DWORD_INDEX_SHIFT);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
		intel_ring_advance(ring);
	}

	ret = intel_ring_begin(ring, 12);
	if (ret)
		return ret;

	/* Start CB2 */
	intel_ring_emit(ring, 0x18800800);	/* BB Start - CB2 */
	intel_ring_emit(ring, 0);		/* Address */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */

	/* Pipe Control */
	intel_ring_emit(ring, 0x7a000003);	/* PipeControl DW0 */
	intel_ring_emit(ring, 0x01510bc);	/* DW1 */
	intel_ring_emit(ring, 0);		/* DW2 */
	intel_ring_emit(ring, 0);		/* DW3 */
	intel_ring_emit(ring, 0);		/* DW4 */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */
	intel_ring_emit(ring, 0);		/* NOOP */

	intel_ring_advance(ring);

	/* Add another 20 Store Data Immediate commands */
	for (i = 0; i < 20; i++) {
		ret = intel_ring_begin(ring, 4);
		if (ret)
			return ret;

		intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
		intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_INDEX <<
					MI_STORE_DWORD_INDEX_SHIFT);
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
		intel_ring_advance(ring);
	}

	return ret;
}

static int
hsw_ring_dispatch_execbuffer(struct intel_engine_cs *ring,
			      u64 offset, u32 len,
			      unsigned flags)
{
	int ret = 0;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring,
			MI_BATCH_BUFFER_START | MI_BATCH_PPGTT_HSW |
			(flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE_HSW));
	/* bit0-7 is the length on GEN6+ */
	intel_ring_emit(ring, offset);
	intel_ring_advance(ring);

	/* Execute CB2 if requested */
	if (flags & I915_DISPATCH_LAUNCH_CB2)
		ret = launch_cb2(ring);

	return ret;
}

static int
gen6_ring_dispatch_execbuffer(struct intel_engine_cs *ring,
			      u64 offset, u32 len,
			      unsigned flags)
{
	int ret = 0;

	ret = intel_ring_begin(ring, 2);
	if (ret)
		return ret;

	intel_ring_emit(ring,
			MI_BATCH_BUFFER_START |
			(flags & I915_DISPATCH_SECURE ? 0 : MI_BATCH_NON_SECURE_I965));
	/* bit0-7 is the length on GEN6+ */
	intel_ring_emit(ring, offset);
	intel_ring_advance(ring);

	/* Execute CB2 if requested */
	if (flags & I915_DISPATCH_LAUNCH_CB2) {
		if (IS_VALLEYVIEW(ring->dev))
			ret = launch_cb2(ring);
	}

	return ret;
}

static int
gen6_ring_stop(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* check if ring is already stopped */
	if (I915_READ_MODE(ring) & RING_MODE_STOP)
		return -EALREADY;

	/* Request the ring to go idle */
	I915_WRITE_MODE(ring, _MASKED_BIT_ENABLE(RING_MODE_STOP));

	/* Wait for idle */
	if (wait_for_atomic((I915_READ_MODE(ring) & RING_MODE_IDLE) != 0,
			    1000)) {
		DRM_ERROR("%s :timed out trying to stop ring", ring->name);
		return -ETIMEDOUT;
	}

	return 0;
}

static int
gen6_ring_start(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t mode;

	/* Clear the MI_MODE stop bit */
	I915_WRITE_MODE(ring, _MASKED_BIT_DISABLE(RING_MODE_STOP));
	mode = I915_READ_MODE(ring);    /* Barrier read */

	return 0;
}

/* Blitter support (SandyBridge+) */

static int gen6_ring_flush(struct intel_engine_cs *ring,
			   u32 invalidate, u32 flush)
{
	struct drm_device *dev = ring->dev;
	uint32_t cmd;
	int ret;

	ret = intel_ring_begin(ring, 4);
	if (ret)
		return ret;

	cmd = MI_FLUSH_DW;
	if (INTEL_INFO(ring->dev)->gen >= 8)
		cmd += 1;
	/*
	 * Bspec vol 1c.3 - blitter engine command streamer:
	 * "If ENABLED, all TLBs will be invalidated once the flush
	 * operation is complete. This bit is only valid when the
	 * Post-Sync Operation field is a value of 1h or 3h."
	 */
	if (invalidate & I915_GEM_DOMAIN_RENDER)
		cmd |= MI_INVALIDATE_TLB | MI_FLUSH_DW_STORE_INDEX |
			MI_FLUSH_DW_OP_STOREDW;
	intel_ring_emit(ring, cmd);
	intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_ADDR | MI_FLUSH_DW_USE_GTT);
	if (INTEL_INFO(ring->dev)->gen >= 8) {
		intel_ring_emit(ring, 0); /* upper addr */
		intel_ring_emit(ring, 0); /* value */
	} else  {
		intel_ring_emit(ring, 0);
		intel_ring_emit(ring, MI_NOOP);
	}
	intel_ring_advance(ring);

	if (IS_GEN7(dev) && !invalidate && flush)
		return gen7_ring_fbc_flush(ring, FBC_REND_CACHE_CLEAN);

	return 0;
}

int intel_init_render_ring_buffer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[RCS];
	int ret;

	ring->name = "render ring";
	ring->id = RCS;
	ring->mmio_base = RENDER_RING_BASE;

	if (INTEL_INFO(dev)->gen >= 6) {
		ring->add_request = gen6_add_request;
		ring->flush = gen7_render_ring_flush;
		if (INTEL_INFO(dev)->gen == 6)
			ring->flush = gen6_render_ring_flush;
		if (INTEL_INFO(dev)->gen >= 8) {
			ring->init_context = intel_ring_workarounds_emit;
			ring->flush = gen8_render_ring_flush;
			ring->irq_get = gen8_ring_get_irq;
			ring->irq_put = gen8_ring_put_irq;
		} else {
			ring->irq_get = gen6_ring_get_irq;
			ring->irq_put = gen6_ring_put_irq;
		}
		ring->irq_enable_mask = GT_RENDER_USER_INTERRUPT;
		if (IS_VALLEYVIEW(dev))
			ring->get_seqno = ring_get_seqno;
		else
			ring->get_seqno = gen6_ring_get_seqno;
		ring->set_seqno = ring_set_seqno;
		ring->semaphore.sync_to = gen6_ring_sync;
		ring->semaphore.signal = gen6_signal;
		/*
		 * The current semaphore is only applied on pre-gen8 platform.
		 * And there is no VCS2 ring on the pre-gen8 platform. So the
		 * semaphore between RCS and VCS2 is initialized as INVALID.
		 * Gen8 will initialize the sema between VCS2 and RCS later.
		 */
		ring->semaphore.mbox.wait[RCS] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore.mbox.wait[VCS] = MI_SEMAPHORE_SYNC_RV;
		ring->semaphore.mbox.wait[BCS] = MI_SEMAPHORE_SYNC_RB;
		ring->semaphore.mbox.wait[VECS] = MI_SEMAPHORE_SYNC_RVE;
		ring->semaphore.mbox.wait[VCS2] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore.mbox.signal[RCS] = GEN6_NOSYNC;
		ring->semaphore.mbox.signal[VCS] = GEN6_VRSYNC;
		ring->semaphore.mbox.signal[BCS] = GEN6_BRSYNC;
		ring->semaphore.mbox.signal[VECS] = GEN6_VERSYNC;
		ring->semaphore.mbox.signal[VCS2] = GEN6_NOSYNC;
		ring->enable = gen6_ring_enable;
		ring->disable = gen6_ring_disable;
		ring->save = gen6_ring_save;
		ring->restore = gen6_ring_restore;
		ring->start = gen6_ring_start;
		ring->stop = gen6_ring_stop;
		ring->invalidate_tlb = gen6_ring_invalidate_tlb;
	} else if (IS_GEN5(dev)) {
		ring->add_request = pc_render_add_request;
		ring->flush = gen4_render_ring_flush;
		ring->get_seqno = pc_render_get_seqno;
		ring->set_seqno = pc_render_set_seqno;
		ring->irq_get = gen5_ring_get_irq;
		ring->irq_put = gen5_ring_put_irq;
		ring->irq_enable_mask = GT_RENDER_USER_INTERRUPT |
					GT_RENDER_PIPECTL_NOTIFY_INTERRUPT;
	} else {
		ring->add_request = i9xx_add_request;
		if (INTEL_INFO(dev)->gen < 4)
			ring->flush = gen2_render_ring_flush;
		else
			ring->flush = gen4_render_ring_flush;
		ring->get_seqno = ring_get_seqno;
		ring->set_seqno = ring_set_seqno;
		if (IS_GEN2(dev)) {
			ring->irq_get = i8xx_ring_get_irq;
			ring->irq_put = i8xx_ring_put_irq;
		} else {
			ring->irq_get = i9xx_ring_get_irq;
			ring->irq_put = i9xx_ring_put_irq;
		}
		ring->irq_enable_mask = I915_USER_INTERRUPT;
	}
	ring->write_tail = ring_write_tail;
	if (IS_HASWELL(dev))
		ring->dispatch_execbuffer = hsw_ring_dispatch_execbuffer;
	else if (IS_GEN8(dev))
		ring->dispatch_execbuffer = gen8_ring_dispatch_execbuffer;
	else if (INTEL_INFO(dev)->gen >= 6)
		ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	else if (INTEL_INFO(dev)->gen >= 4)
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	else if (IS_I830(dev) || IS_845G(dev))
		ring->dispatch_execbuffer = i830_dispatch_execbuffer;
	else
		ring->dispatch_execbuffer = i915_dispatch_execbuffer;
	ring->init = init_render_ring;
	ring->cleanup = render_ring_cleanup;

	/* Workaround batchbuffer to combat CS tlb bug. */
	if (HAS_BROKEN_CS_TLB(dev)) {
		struct drm_i915_gem_object *obj;
		int ret;

		obj = i915_gem_alloc_object(dev, I830_BATCH_LIMIT);
		if (obj == NULL) {
			DRM_ERROR("Failed to allocate batch bo\n");
			return -ENOMEM;
		}

		ret = i915_gem_obj_ggtt_pin(obj, 0, 0);
		if (ret != 0) {
			drm_gem_object_unreference(&obj->base);
			DRM_ERROR("Failed to ping batch bo\n");
			return ret;
		}

		ring->scratch.obj = obj;
		ring->scratch.gtt_offset = i915_gem_obj_ggtt_offset(obj);
	}

	ret = intel_init_ring_buffer(dev, ring);
	if (ret)
		return ret;

	if (INTEL_INFO(dev)->gen >= 5) {
		ret = intel_init_pipe_control(ring);
		if (ret)
			return ret;
	}

	return 0;
}

int intel_render_ring_init_dri(struct drm_device *dev, u64 start, u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[RCS];
	struct intel_ringbuffer *ringbuf = ring->buffer;
	int ret;

	if (ringbuf == NULL) {
		ringbuf = kzalloc(sizeof(*ringbuf), GFP_KERNEL);
		if (!ringbuf)
			return -ENOMEM;
		ring->buffer = ringbuf;
	}

	ring->name = "render ring";
	ring->id = RCS;
	ring->mmio_base = RENDER_RING_BASE;

	if (INTEL_INFO(dev)->gen >= 6) {
		/* non-kms not supported on gen6+ */
		ret = -ENODEV;
		goto err_ringbuf;
	}

	/* Note: gem is not supported on gen5/ilk without kms (the corresponding
	 * gem_init ioctl returns with -ENODEV). Hence we do not need to set up
	 * the special gen5 functions. */
	ring->add_request = i9xx_add_request;
	if (INTEL_INFO(dev)->gen < 4)
		ring->flush = gen2_render_ring_flush;
	else
		ring->flush = gen4_render_ring_flush;
	ring->get_seqno = ring_get_seqno;
	ring->set_seqno = ring_set_seqno;
	if (IS_GEN2(dev)) {
		ring->irq_get = i8xx_ring_get_irq;
		ring->irq_put = i8xx_ring_put_irq;
	} else {
		ring->irq_get = i9xx_ring_get_irq;
		ring->irq_put = i9xx_ring_put_irq;
	}
	ring->irq_enable_mask = I915_USER_INTERRUPT;
	ring->write_tail = ring_write_tail;
	if (INTEL_INFO(dev)->gen >= 4)
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	else if (IS_I830(dev) || IS_845G(dev))
		ring->dispatch_execbuffer = i830_dispatch_execbuffer;
	else
		ring->dispatch_execbuffer = i915_dispatch_execbuffer;
	ring->init = init_render_ring;
	ring->cleanup = render_ring_cleanup;

	ring->dev = dev;
	INIT_LIST_HEAD(&ring->active_list);
	INIT_LIST_HEAD(&ring->request_list);

	ringbuf->size = size;
	ringbuf->effective_size = ringbuf->size;
	if (IS_I830(ring->dev) || IS_845G(ring->dev))
		ringbuf->effective_size -= 2 * CACHELINE_BYTES;

	ringbuf->virtual_start = ioremap_wc(start, size);
	if (ringbuf->virtual_start == NULL) {
		DRM_ERROR("can not ioremap virtual address for"
			  " ring buffer\n");
		ret = -ENOMEM;
		goto err_ringbuf;
	}

	if (!I915_NEED_GFX_HWS(dev)) {
		ret = init_phys_status_page(ring);
		if (ret)
			goto err_vstart;
	}

	return 0;

err_vstart:
	iounmap(ringbuf->virtual_start);
err_ringbuf:
	kfree(ringbuf);
	ring->buffer = NULL;
	return ret;
}

int intel_init_bsd_ring_buffer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[VCS];

	ring->name = "bsd ring";
	ring->id = VCS;

	ring->write_tail = ring_write_tail;
	if (INTEL_INFO(dev)->gen >= 6) {
		ring->mmio_base = GEN6_BSD_RING_BASE;
		/* gen6 bsd needs a special wa for tail updates */
		if (IS_GEN6(dev))
			ring->write_tail = gen6_bsd_ring_write_tail;
		ring->flush = gen6_bsd_ring_flush;
		ring->add_request = gen6_add_request;
		ring->get_seqno = gen6_ring_get_seqno;
		ring->set_seqno = ring_set_seqno;
		if (INTEL_INFO(dev)->gen >= 8) {
			ring->irq_enable_mask =
				GT_RENDER_USER_INTERRUPT << GEN8_VCS1_IRQ_SHIFT;
			ring->irq_get = gen8_ring_get_irq;
			ring->irq_put = gen8_ring_put_irq;
			ring->dispatch_execbuffer =
				gen8_ring_dispatch_execbuffer;
		} else {
			ring->irq_enable_mask = GT_BSD_USER_INTERRUPT;
			ring->irq_get = gen6_ring_get_irq;
			ring->irq_put = gen6_ring_put_irq;
			ring->dispatch_execbuffer =
				gen6_ring_dispatch_execbuffer;
		}
		ring->semaphore.sync_to = gen6_ring_sync;
		ring->semaphore.signal = gen6_signal;
		/*
		 * The current semaphore is only applied on pre-gen8 platform.
		 * And there is no VCS2 ring on the pre-gen8 platform. So the
		 * semaphore between VCS and VCS2 is initialized as INVALID.
		 * Gen8 will initialize the sema between VCS2 and VCS later.
		 */
		ring->semaphore.mbox.wait[RCS] = MI_SEMAPHORE_SYNC_VR;
		ring->semaphore.mbox.wait[VCS] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore.mbox.wait[BCS] = MI_SEMAPHORE_SYNC_VB;
		ring->semaphore.mbox.wait[VECS] = MI_SEMAPHORE_SYNC_VVE;
		ring->semaphore.mbox.wait[VCS2] = MI_SEMAPHORE_SYNC_INVALID;
		ring->semaphore.mbox.signal[RCS] = GEN6_RVSYNC;
		ring->semaphore.mbox.signal[VCS] = GEN6_NOSYNC;
		ring->semaphore.mbox.signal[BCS] = GEN6_BVSYNC;
		ring->semaphore.mbox.signal[VECS] = GEN6_VEVSYNC;
		ring->semaphore.mbox.signal[VCS2] = GEN6_NOSYNC;
		ring->enable = gen6_ring_enable;
		ring->disable = gen6_ring_disable;
		ring->save = gen6_ring_save;
		ring->restore = gen6_ring_restore;
		ring->start = gen6_ring_start;
		ring->stop = gen6_ring_stop;
		ring->invalidate_tlb = gen6_ring_invalidate_tlb;
	} else {
		ring->mmio_base = BSD_RING_BASE;
		ring->flush = bsd_ring_flush;
		ring->add_request = i9xx_add_request;
		ring->get_seqno = ring_get_seqno;
		ring->set_seqno = ring_set_seqno;
		if (IS_GEN5(dev)) {
			ring->irq_enable_mask = ILK_BSD_USER_INTERRUPT;
			ring->irq_get = gen5_ring_get_irq;
			ring->irq_put = gen5_ring_put_irq;
		} else {
			ring->irq_enable_mask = I915_BSD_USER_INTERRUPT;
			ring->irq_get = i9xx_ring_get_irq;
			ring->irq_put = i9xx_ring_put_irq;
		}
		ring->dispatch_execbuffer = i965_dispatch_execbuffer;
	}
	ring->init = init_ring_common;

	/* Enable the timeout counter for watchdog reset */
	I915_WRITE_IMR(ring, ~GT_GEN6_BSD_WATCHDOG_INTERRUPT);

	return intel_init_ring_buffer(dev, ring);
}

/**
 * Initialize the second BSD ring for Broadwell GT3.
 * It is noted that this only exists on Broadwell GT3.
 */
int intel_init_bsd2_ring_buffer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[VCS2];

	if ((INTEL_INFO(dev)->gen != 8)) {
		DRM_ERROR("No dual-BSD ring on non-BDW machine\n");
		return -EINVAL;
	}

	ring->name = "bds2_ring";
	ring->id = VCS2;

	ring->write_tail = ring_write_tail;
	ring->mmio_base = GEN8_BSD2_RING_BASE;
	ring->flush = gen6_bsd_ring_flush;
	ring->add_request = gen6_add_request;
	ring->get_seqno = gen6_ring_get_seqno;
	ring->set_seqno = ring_set_seqno;
	ring->irq_enable_mask =
			GT_RENDER_USER_INTERRUPT << GEN8_VCS2_IRQ_SHIFT;
	ring->irq_get = gen8_ring_get_irq;
	ring->irq_put = gen8_ring_put_irq;
	ring->dispatch_execbuffer =
			gen8_ring_dispatch_execbuffer;
	ring->semaphore.sync_to = gen6_ring_sync;
	ring->semaphore.signal = gen6_signal;
	/*
	 * The current semaphore is only applied on the pre-gen8. And there
	 * is no bsd2 ring on the pre-gen8. So now the semaphore_register
	 * between VCS2 and other ring is initialized as invalid.
	 * Gen8 will initialize the sema between VCS2 and other ring later.
	 */
	ring->semaphore.mbox.wait[RCS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[VCS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[BCS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[VECS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[VCS2] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.signal[RCS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[VCS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[BCS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[VECS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[VCS2] = GEN6_NOSYNC;
	ring->enable = gen6_ring_enable;
	ring->disable = gen6_ring_disable;
	ring->save = gen6_ring_save;
	ring->restore = gen6_ring_restore;
	ring->start = gen6_ring_start;
	ring->stop = gen6_ring_stop;
	ring->invalidate_tlb = gen6_ring_invalidate_tlb;

	ring->init = init_ring_common;

	return intel_init_ring_buffer(dev, ring);
}

int intel_init_blt_ring_buffer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[BCS];

	ring->name = "blitter ring";
	ring->id = BCS;

	ring->mmio_base = BLT_RING_BASE;
	ring->write_tail = ring_write_tail;
	ring->flush = gen6_ring_flush;
	ring->add_request = gen6_add_request;
	ring->get_seqno = gen6_ring_get_seqno;
	ring->set_seqno = ring_set_seqno;
	if (INTEL_INFO(dev)->gen >= 8) {
		ring->irq_enable_mask =
			GT_RENDER_USER_INTERRUPT << GEN8_BCS_IRQ_SHIFT;
		ring->irq_get = gen8_ring_get_irq;
		ring->irq_put = gen8_ring_put_irq;
		ring->dispatch_execbuffer = gen8_ring_dispatch_execbuffer;
	} else {
		ring->irq_enable_mask = GT_BLT_USER_INTERRUPT;
		ring->irq_get = gen6_ring_get_irq;
		ring->irq_put = gen6_ring_put_irq;
		ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	}
	ring->semaphore.sync_to = gen6_ring_sync;
	ring->semaphore.signal = gen6_signal;
	/*
	 * The current semaphore is only applied on pre-gen8 platform. And
	 * there is no VCS2 ring on the pre-gen8 platform. So the semaphore
	 * between BCS and VCS2 is initialized as INVALID.
	 * Gen8 will initialize the sema between BCS and VCS2 later.
	 */
	ring->semaphore.mbox.wait[RCS] = MI_SEMAPHORE_SYNC_BR;
	ring->semaphore.mbox.wait[VCS] = MI_SEMAPHORE_SYNC_BV;
	ring->semaphore.mbox.wait[BCS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[VECS] = MI_SEMAPHORE_SYNC_BVE;
	ring->semaphore.mbox.wait[VCS2] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.signal[RCS] = GEN6_RBSYNC;
	ring->semaphore.mbox.signal[VCS] = GEN6_VBSYNC;
	ring->semaphore.mbox.signal[BCS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[VECS] = GEN6_VEBSYNC;
	ring->semaphore.mbox.signal[VCS2] = GEN6_NOSYNC;
	ring->enable = gen6_ring_enable;
	ring->disable = gen6_ring_disable;
	ring->save = gen6_ring_save;
	ring->restore = gen6_ring_restore;
	ring->start = gen6_ring_start;
	ring->stop = gen6_ring_stop;
	ring->invalidate_tlb = gen6_ring_invalidate_tlb;

	ring->init = init_ring_common;

	return intel_init_ring_buffer(dev, ring);
}

int intel_init_vebox_ring_buffer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[VECS];

	ring->name = "video enhancement ring";
	ring->id = VECS;

	ring->mmio_base = VEBOX_RING_BASE;
	ring->write_tail = ring_write_tail;
	ring->flush = gen6_ring_flush;
	ring->add_request = gen6_add_request;
	ring->get_seqno = gen6_ring_get_seqno;
	ring->set_seqno = ring_set_seqno;

	if (INTEL_INFO(dev)->gen >= 8) {
		ring->irq_enable_mask =
			GT_RENDER_USER_INTERRUPT << GEN8_VECS_IRQ_SHIFT;
		ring->irq_get = gen8_ring_get_irq;
		ring->irq_put = gen8_ring_put_irq;
		ring->dispatch_execbuffer = gen8_ring_dispatch_execbuffer;
	} else {
		ring->irq_enable_mask = PM_VEBOX_USER_INTERRUPT;
		ring->irq_get = hsw_vebox_get_irq;
		ring->irq_put = hsw_vebox_put_irq;
		ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	}
	ring->semaphore.sync_to = gen6_ring_sync;
	ring->semaphore.signal = gen6_signal;
	ring->semaphore.mbox.wait[RCS] = MI_SEMAPHORE_SYNC_VER;
	ring->semaphore.mbox.wait[VCS] = MI_SEMAPHORE_SYNC_VEV;
	ring->semaphore.mbox.wait[BCS] = MI_SEMAPHORE_SYNC_VEB;
	ring->semaphore.mbox.wait[VECS] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.wait[VCS2] = MI_SEMAPHORE_SYNC_INVALID;
	ring->semaphore.mbox.signal[RCS] = GEN6_RVESYNC;
	ring->semaphore.mbox.signal[VCS] = GEN6_VVESYNC;
	ring->semaphore.mbox.signal[BCS] = GEN6_BVESYNC;
	ring->semaphore.mbox.signal[VECS] = GEN6_NOSYNC;
	ring->semaphore.mbox.signal[VCS2] = GEN6_NOSYNC;
	ring->irq_enable_mask = PM_VEBOX_USER_INTERRUPT;
	ring->irq_get = hsw_vebox_get_irq;
	ring->irq_put = hsw_vebox_put_irq;
	ring->dispatch_execbuffer = gen6_ring_dispatch_execbuffer;
	ring->enable = gen6_ring_enable;
	ring->disable = gen6_ring_disable;
	ring->start = gen6_ring_start;
	ring->stop = gen6_ring_stop;
	ring->save = gen6_ring_save;
	ring->restore = gen6_ring_restore;
	ring->invalidate_tlb = gen6_ring_invalidate_tlb;

	ring->init = init_ring_common;

	return intel_init_ring_buffer(dev, ring);
}

int
intel_ring_start_watchdog(struct intel_engine_cs *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 10);
	if (ret)
		return ret;

	/* i915_reg.h includes a warning to place a MI_NOOP
	* before a MI_LOAD_REGISTER_IMM*/
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_emit(ring, MI_NOOP);

	/* Set counter period */
	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_THRESH(ring->mmio_base));
	intel_ring_emit(ring, ring->watchdog_threshold);
	intel_ring_emit(ring, MI_NOOP);

	/* Start counter */
	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_CNTR(ring->mmio_base));
	intel_ring_emit(ring, WATCHDOG_ENABLE);
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

int
intel_ring_stop_watchdog(struct intel_engine_cs *ring)
{
	int ret;

	ret = intel_ring_begin(ring, 6);
	if (ret)
		return ret;

	/* i915_reg.h includes a warning to place a MI_NOOP
	* before a MI_LOAD_REGISTER_IMM*/
	intel_ring_emit(ring, MI_NOOP);
	intel_ring_emit(ring, MI_NOOP);

	intel_ring_emit(ring, MI_LOAD_REGISTER_IMM(1));
	intel_ring_emit(ring, RING_CNTR(ring->mmio_base));

	switch (ring->id) {
	default:
	case RCS:
		intel_ring_emit(ring, RCS_WATCHDOG_DISABLE);
		break;
	case VCS:
		intel_ring_emit(ring, VCS_WATCHDOG_DISABLE);
		break;
	}

	intel_ring_emit(ring, MI_NOOP);
	intel_ring_advance(ring);

	return 0;
}

int
intel_ring_flush_all_caches(struct intel_engine_cs *ring)
{
	int ret;

	if (!ring->gpu_caches_dirty)
		return 0;

	ret = ring->flush(ring, 0, I915_GEM_GPU_DOMAINS);
	if (ret)
		return ret;

	trace_i915_gem_ring_flush(ring, 0, I915_GEM_GPU_DOMAINS);

	if (IS_VALLEYVIEW(ring->dev) && IS_GEN7(ring->dev)) {
		/*
		 * WaReadAfterWriteHazard
		 * Send a number of Store Data commands here to finish
		 * flushing hardware pipeline.This is needed in the case
		 * where the next workload tries reading from the same
		 * surface that this batch writes to. Without these StoreDWs,
		 * not all of the data will actually be flushd to the surface
		 * by the time the next batch starts reading it, possibly
		 * causing a small amount of corruption.
		 */
		int i;
		ret = intel_ring_begin(ring, 4 * 12);
		if (ret)
			return ret;
		for (i = 0; i < 12; i++) {
			intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
			intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_INDEX <<
						MI_STORE_DWORD_INDEX_SHIFT);
			intel_ring_emit(ring, 0);
			intel_ring_emit(ring, MI_NOOP);
		}
		intel_ring_advance(ring);
	}

	ring->gpu_caches_dirty = false;
	return 0;
}

int
intel_ring_invalidate_all_caches(struct intel_engine_cs *ring)
{
	uint32_t flush_domains;
	int ret;

	if (IS_VALLEYVIEW(ring->dev) && IS_GEN7(ring->dev)) {
		/*
		 * WaTlbInvalidateStoreDataBefore:vlv
		 * Before pipecontrol with TLB invalidate set, need 2 store
		 * data cmds (such as MI_STORE_DATA_IMM or MI_STORE_DATA_INDEX)
		 * Without this, hardware cannot guarantee the cmd after the
		 * PIPE_CONTROL with TLB inv will not use the old TLB values.
		 */
		int i;
		ret = intel_ring_begin(ring, 4 * 2);
		if (ret)
			return ret;
		for (i = 0; i < 2; i++) {
			intel_ring_emit(ring, MI_STORE_DWORD_INDEX);
			intel_ring_emit(ring, I915_GEM_HWS_SCRATCH_INDEX <<
						MI_STORE_DWORD_INDEX_SHIFT);
			intel_ring_emit(ring, 0);
			intel_ring_emit(ring, MI_NOOP);
		}
		intel_ring_advance(ring);
	}

	flush_domains = 0;
	if (ring->gpu_caches_dirty)
		flush_domains = I915_GEM_GPU_DOMAINS;

	ret = ring->flush(ring, I915_GEM_GPU_DOMAINS, flush_domains);
	if (ret)
		return ret;

	trace_i915_gem_ring_flush(ring, I915_GEM_GPU_DOMAINS, flush_domains);

	ring->gpu_caches_dirty = false;
	return 0;
}

void
intel_stop_ring_buffer(struct intel_engine_cs *ring)
{
	int ret;

	if (!intel_ring_initialized(ring))
		return;

	ret = intel_ring_idle(ring, true);
	if (ret && !i915_reset_in_progress(&to_i915(ring->dev)->gpu_error))
		DRM_ERROR("failed to quiesce %s whilst cleaning up: %d\n",
			  ring->name, ret);

	stop_ring(ring);
}
