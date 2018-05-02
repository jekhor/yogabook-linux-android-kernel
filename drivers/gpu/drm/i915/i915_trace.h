#if !defined(_I915_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _I915_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#include <drm/drmP.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_ringbuffer.h"
#include "i915_scheduler.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM i915
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)
#define TRACE_INCLUDE_FILE i915_trace

/* atomic update */
TRACE_EVENT(i915_atomic_update_start,
	    TP_PROTO(struct intel_crtc *crtc),
	    TP_ARGS(crtc),
	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(bool, pfit_changed)
			     __field(u32, pfit_control)
			     __field(u32, scaling_src_size)
			     __field(u32, frame)
			     __field(u32, scanline)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->pfit_changed = ((struct drm_i915_private *)
			     crtc->base.dev->dev_private)->pfit_changed;
			   __entry->pfit_control = crtc->pfit_control;
			   __entry->scaling_src_size = crtc->scaling_src_size;
			   __entry->frame =
			     crtc->base.dev->driver->get_vblank_counter(
			     crtc->base.dev, crtc->pipe);
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u, pf[%s]:ctrl=%x size=%x",
		      pipe_name(__entry->pipe), __entry->frame,
		       __entry->scanline, __entry->pfit_changed ? "Y":"N",
		       __entry->pfit_control, __entry->scaling_src_size)
);

TRACE_EVENT(i915_atomic_update_end,
	    TP_PROTO(struct intel_crtc *crtc),
	    TP_ARGS(crtc),
	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, frame)
			     __field(u32, scanline)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->frame =
			     crtc->base.dev->driver->get_vblank_counter(
			     crtc->base.dev, crtc->pipe);
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u",
		      pipe_name(__entry->pipe), __entry->frame,
		       __entry->scanline)
);

TRACE_EVENT(i915_maxfifo_update,
	    TP_PROTO(struct intel_crtc *crtc, bool enable),
	    TP_ARGS(crtc, enable),
	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, frame)
			     __field(u32, scanline)
			     __field(bool, enable)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->frame =
			     crtc->base.dev->driver->get_vblank_counter(
			     crtc->base.dev, crtc->pipe);
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   __entry->enable = enable;
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u, maxfifo=%s",
		      pipe_name(__entry->pipe), __entry->frame,
		       __entry->scanline,
		       __entry->enable ? "Enabled":"Disabled")
);

TRACE_EVENT(i915_plane_info,
	    TP_PROTO(struct intel_crtc *crtc,
		     struct drm_mode_set_display_plane *plane, u32 disp_flag),
	    TP_ARGS(crtc, plane, disp_flag),
	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, obj_type)
			     __field(u32, fb_id)
			     __field(u32, flags)
			     __field(u32, disp_flag)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->obj_type = plane->obj_type;
			   __entry->fb_id = plane->fb_id;
			   __entry->flags = plane->flags;
			   __entry->disp_flag = disp_flag;
			   ),

	    TP_printk("pipe %c, obj_type=%x, fb_id=%u, flags=%x, disp_flag=%x",
		      pipe_name(__entry->pipe), __entry->obj_type,
		       __entry->fb_id, __entry->flags,  __entry->disp_flag)
);

/* pipe updates */

TRACE_EVENT(i915_pipe_update_start,
	    TP_PROTO(struct intel_crtc *crtc, u32 min, u32 max),
	    TP_ARGS(crtc, min, max),

	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, frame)
			     __field(u32, scanline)
			     __field(u32, min)
			     __field(u32, max)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->frame = crtc->base.dev->driver->get_vblank_counter(crtc->base.dev,
										       crtc->pipe);
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   __entry->min = min;
			   __entry->max = max;
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u, min=%u, max=%u",
		      pipe_name(__entry->pipe), __entry->frame,
		       __entry->scanline, __entry->min, __entry->max)
);

TRACE_EVENT(i915_pipe_update_vblank_evaded,
	    TP_PROTO(struct intel_crtc *crtc, u32 min, u32 max, u32 frame),
	    TP_ARGS(crtc, min, max, frame),

	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, frame)
			     __field(u32, scanline)
			     __field(u32, min)
			     __field(u32, max)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->frame = frame;
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   __entry->min = min;
			   __entry->max = max;
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u, min=%u, max=%u",
		      pipe_name(__entry->pipe), __entry->frame,
		       __entry->scanline, __entry->min, __entry->max)
);

TRACE_EVENT(i915_pipe_update_end,
	    TP_PROTO(struct intel_crtc *crtc, u32 frame),
	    TP_ARGS(crtc, frame),

	    TP_STRUCT__entry(
			     __field(enum pipe, pipe)
			     __field(u32, frame)
			     __field(u32, scanline)
			     ),

	    TP_fast_assign(
			   __entry->pipe = crtc->pipe;
			   __entry->frame = frame;
			   __entry->scanline = intel_get_crtc_scanline(crtc);
			   ),

	    TP_printk("pipe %c, frame=%u, scanline=%u",
		      pipe_name(__entry->pipe), __entry->frame,
		      __entry->scanline)
);

/* object tracking */

TRACE_EVENT(i915_gem_object_create,
	    TP_PROTO(struct drm_i915_gem_object *obj),
	    TP_ARGS(obj),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u32, size)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->size = obj->base.size;
			   ),

	    TP_printk("obj=%p, size=%u", __entry->obj, __entry->size)
);

TRACE_EVENT(i915_vma_bind,
	    TP_PROTO(struct i915_vma *vma, unsigned flags),
	    TP_ARGS(vma, flags),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(struct i915_address_space *, vm)
			     __field(u64, offset)
			     __field(u32, size)
			     __field(unsigned, flags)
			     ),

	    TP_fast_assign(
			   __entry->obj = vma->obj;
			   __entry->vm = vma->vm;
			   __entry->offset = vma->node.start;
			   __entry->size = vma->node.size;
			   __entry->flags = flags;
			   ),

	    TP_printk("obj=%p, offset=%016llx size=%x%s vm=%p",
		      __entry->obj, __entry->offset, __entry->size,
		      __entry->flags & PIN_MAPPABLE ? ", mappable" : "",
		      __entry->vm)
);

TRACE_EVENT(i915_vma_unbind,
	    TP_PROTO(struct i915_vma *vma),
	    TP_ARGS(vma),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(struct i915_address_space *, vm)
			     __field(u64, offset)
			     __field(u32, size)
			     ),

	    TP_fast_assign(
			   __entry->obj = vma->obj;
			   __entry->vm = vma->vm;
			   __entry->offset = vma->node.start;
			   __entry->size = vma->node.size;
			   ),

	    TP_printk("obj=%p, offset=%016llx size=%x vm=%p",
		      __entry->obj, __entry->offset, __entry->size, __entry->vm)
);

#define VM_TO_TRACE_NAME(vm) \
	(i915_is_ggtt(vm) ? "GGTT" : \
		      "Private VM")

DECLARE_EVENT_CLASS(i915_va,
	TP_PROTO(struct i915_address_space *vm, u64 start, u64 length, const char *name),
	TP_ARGS(vm, start, length, name),

	TP_STRUCT__entry(
		__field(struct i915_address_space *, vm)
		__field(u64, start)
		__field(u64, end)
		__string(name, name)
	),

	TP_fast_assign(
		__entry->vm = vm;
		__entry->start = start;
		__entry->end = start + length;
		__assign_str(name, name);
	),

	TP_printk("vm=%p (%s), 0x%llx-0x%llx",
		  __entry->vm, __get_str(name),  __entry->start, __entry->end)
);

DEFINE_EVENT(i915_va, i915_va_alloc,
	     TP_PROTO(struct i915_address_space *vm, u64 start, u64 length, const char *name),
	     TP_ARGS(vm, start, length, name)
);

DECLARE_EVENT_CLASS(i915_page_table_entry,
	TP_PROTO(struct i915_address_space *vm, u32 pde, u64 start, u64 pde_shift),
	TP_ARGS(vm, pde, start, pde_shift),

	TP_STRUCT__entry(
		__field(struct i915_address_space *, vm)
		__field(u32, pde)
		__field(u64, start)
		__field(u64, end)
	),

	TP_fast_assign(
		__entry->vm = vm;
		__entry->pde = pde;
		__entry->start = start;
		__entry->end = (start + (1ULL << pde_shift)) & ~((1ULL << pde_shift)-1);
	),

	TP_printk("vm=%p, pde=%d (0x%llx-0x%llx)",
		  __entry->vm, __entry->pde, __entry->start, __entry->end)
);

DEFINE_EVENT(i915_page_table_entry, i915_page_table_entry_alloc,
	     TP_PROTO(struct i915_address_space *vm, u32 pde, u64 start, u64 pde_shift),
	     TP_ARGS(vm, pde, start, pde_shift)
);

/* Avoid extra math because we only support two sizes. The format is defined by
 * bitmap_scnprintf. Each 32 bits is 8 HEX digits followed by comma */
#define TRACE_PT_SIZE(bits) \
	((((bits) == 1024) ? 288 : 144) + 1)

DECLARE_EVENT_CLASS(i915_page_table_entry_update,
	TP_PROTO(struct i915_address_space *vm, u32 pde,
		 struct i915_page_table_entry *pt, u32 first, u32 len, size_t bits),
	TP_ARGS(vm, pde, pt, first, len, bits),

	TP_STRUCT__entry(
		__field(struct i915_address_space *, vm)
		__field(u32, pde)
		__field(u32, first)
		__field(u32, last)
		__dynamic_array(char, cur_ptes, TRACE_PT_SIZE(bits))
	),

	TP_fast_assign(
		__entry->vm = vm;
		__entry->pde = pde;
		__entry->first = first;
		__entry->last = first + len;

		bitmap_scnprintf(__get_str(cur_ptes),
				 TRACE_PT_SIZE(bits),
				 pt->used_ptes,
				 bits);
	),

	TP_printk("vm=%p, pde=%d, updating %u:%u\t%s",
		  __entry->vm, __entry->pde, __entry->last, __entry->first,
		  __get_str(cur_ptes))
);

DEFINE_EVENT(i915_page_table_entry_update, i915_page_table_entry_map,
	TP_PROTO(struct i915_address_space *vm, u32 pde,
		 struct i915_page_table_entry *pt, u32 first, u32 len, size_t bits),
	TP_ARGS(vm, pde, pt, first, len, bits)
);

TRACE_EVENT(i915_gem_object_change_domain,
	    TP_PROTO(struct drm_i915_gem_object *obj, u32 old_read, u32 old_write),
	    TP_ARGS(obj, old_read, old_write),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u32, read_domains)
			     __field(u32, write_domain)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->read_domains = obj->base.read_domains | (old_read << 16);
			   __entry->write_domain = obj->base.write_domain | (old_write << 16);
			   ),

	    TP_printk("obj=%p, read=%02x=>%02x, write=%02x=>%02x",
		      __entry->obj,
		      __entry->read_domains >> 16,
		      __entry->read_domains & 0xffff,
		      __entry->write_domain >> 16,
		      __entry->write_domain & 0xffff)
);

TRACE_EVENT(i915_gem_object_pwrite,
	    TP_PROTO(struct drm_i915_gem_object *obj, u32 offset, u32 len),
	    TP_ARGS(obj, offset, len),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u32, offset)
			     __field(u32, len)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->offset = offset;
			   __entry->len = len;
			   ),

	    TP_printk("obj=%p, offset=%u, len=%u",
		      __entry->obj, __entry->offset, __entry->len)
);

TRACE_EVENT(i915_gem_object_pread,
	    TP_PROTO(struct drm_i915_gem_object *obj, u32 offset, u32 len),
	    TP_ARGS(obj, offset, len),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u32, offset)
			     __field(u32, len)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->offset = offset;
			   __entry->len = len;
			   ),

	    TP_printk("obj=%p, offset=%u, len=%u",
		      __entry->obj, __entry->offset, __entry->len)
);

TRACE_EVENT(i915_gem_object_fault,
	    TP_PROTO(struct drm_i915_gem_object *obj, u32 index, bool gtt, bool write),
	    TP_ARGS(obj, index, gtt, write),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u32, index)
			     __field(bool, gtt)
			     __field(bool, write)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->index = index;
			   __entry->gtt = gtt;
			   __entry->write = write;
			   ),

	    TP_printk("obj=%p, %s index=%u %s",
		      __entry->obj,
		      __entry->gtt ? "GTT" : "CPU",
		      __entry->index,
		      __entry->write ? ", writable" : "")
);

DECLARE_EVENT_CLASS(i915_gem_object,
	    TP_PROTO(struct drm_i915_gem_object *obj),
	    TP_ARGS(obj),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   ),

	    TP_printk("obj=%p", __entry->obj)
);

DEFINE_EVENT(i915_gem_object, i915_gem_object_clflush,
	     TP_PROTO(struct drm_i915_gem_object *obj),
	     TP_ARGS(obj)
);

DEFINE_EVENT(i915_gem_object, i915_gem_object_destroy,
	    TP_PROTO(struct drm_i915_gem_object *obj),
	    TP_ARGS(obj)
);

TRACE_EVENT(i915_gem_evict,
	    TP_PROTO(struct drm_device *dev, u32 size, u32 align, unsigned flags),
	    TP_ARGS(dev, size, align, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, size)
			     __field(u32, align)
			     __field(unsigned, flags)
			    ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->size = size;
			   __entry->align = align;
			   __entry->flags = flags;
			  ),

	    TP_printk("dev=%d, size=%d, align=%d %s",
		      __entry->dev, __entry->size, __entry->align,
		      __entry->flags & PIN_MAPPABLE ? ", mappable" : "")
);

TRACE_EVENT(i915_gem_evict_everything,
	    TP_PROTO(struct drm_device *dev),
	    TP_ARGS(dev),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			    ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			  ),

	    TP_printk("dev=%d", __entry->dev)
);

TRACE_EVENT(i915_gem_evict_vm,
	    TP_PROTO(struct i915_address_space *vm),
	    TP_ARGS(vm),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(struct i915_address_space *, vm)
			    ),

	    TP_fast_assign(
			   __entry->dev = vm->dev->primary->index;
			   __entry->vm = vm;
			  ),

	    TP_printk("dev=%d, vm=%p", __entry->dev, __entry->vm)
);

TRACE_EVENT(i915_gem_ring_sync_to,
	    TP_PROTO(struct intel_engine_cs *from,
		     struct intel_engine_cs *to,
		     struct drm_i915_gem_request *req),
	    TP_ARGS(from, to, req),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, sync_from)
			     __field(u32, sync_to)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = from->dev->primary->index;
			   __entry->sync_from = from->id;
			   __entry->sync_to = to->id;
			   __entry->seqno = i915_gem_request_get_seqno(req);
			   ),

	    TP_printk("dev=%u, sync-from=%u, sync-to=%u, seqno=%u",
		      __entry->dev,
		      __entry->sync_from, __entry->sync_to,
		      __entry->seqno)
);

TRACE_EVENT(i915_gem_ring_dispatch,
	    TP_PROTO(struct drm_i915_gem_request *req, u32 flags),
	    TP_ARGS(req, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     __field(u32, seqno)
			     __field(u32, flags)
			     ),

	    TP_fast_assign(
			   struct intel_engine_cs *ring =
						i915_gem_request_get_ring(req);
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   __entry->seqno = i915_gem_request_get_seqno(req);
			   __entry->flags = flags;
			   i915_trace_irq_get(ring, req);
			   ),

	    TP_printk("dev=%u, ring=%u, seqno=%u, flags=%x",
		      __entry->dev, __entry->ring, __entry->seqno, __entry->flags)
);

TRACE_EVENT(i915_gem_ring_flush,
	    TP_PROTO(struct intel_engine_cs *ring, u32 invalidate, u32 flush),
	    TP_ARGS(ring, invalidate, flush),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     __field(u32, invalidate)
			     __field(u32, flush)
			     ),

	    TP_fast_assign(
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   __entry->invalidate = invalidate;
			   __entry->flush = flush;
			   ),

	    TP_printk("dev=%u, ring=%x, invalidate=%04x, flush=%04x",
		      __entry->dev, __entry->ring,
		      __entry->invalidate, __entry->flush)
);

DECLARE_EVENT_CLASS(i915_gem_request,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     __field(u32, completed)
			     ),

	    TP_fast_assign(
			   struct intel_engine_cs *ring =
						i915_gem_request_get_ring(req);
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   __entry->uniq = req ? req->uniq : 0;
			   __entry->seqno = i915_gem_request_get_seqno(req);
			   __entry->completed = i915_gem_request_completed(req);
			   ),
	    TP_printk("dev=%u, ring=%u, uniq=%u, seqno=%u, completed=%u",
			__entry->dev, __entry->ring, __entry->uniq,
			__entry->seqno, __entry->completed)
);

DEFINE_EVENT(i915_gem_request, i915_gem_request_add,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req)
);

TRACE_EVENT(i915_gem_request_notify,
	    TP_PROTO(struct intel_engine_cs *ring),
	    TP_ARGS(ring),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   __entry->seqno = ring->get_seqno(ring, false);
			   ),

	    TP_printk("dev=%u, ring=%u, seqno=%u",
		      __entry->dev, __entry->ring, __entry->seqno)
);

DEFINE_EVENT(i915_gem_request, i915_gem_request_retire,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req)
);

DEFINE_EVENT(i915_gem_request, i915_gem_request_complete,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req)
);

DEFINE_EVENT(i915_gem_request, i915_gem_request_complete_loop,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req)
);

TRACE_EVENT(i915_gem_request_wait_begin,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     __field(bool, blocking)
			     ),

	    /* NB: the blocking information is racy since mutex_is_locked
	     * doesn't check that the current thread holds the lock. The only
	     * other option would be to pass the boolean information of whether
	     * or not the class was blocking down through the stack which is
	     * less desirable.
	     */
	    TP_fast_assign(
			   struct intel_engine_cs *ring =
						i915_gem_request_get_ring(req);
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   __entry->uniq = req ? req->uniq : 0;
			   __entry->seqno = i915_gem_request_get_seqno(req);
			   __entry->blocking =
				     mutex_is_locked(&ring->dev->struct_mutex);
			   ),

	    TP_printk("dev=%u, ring=%u, uniq=%u, seqno=%u, blocking=%s",
		      __entry->dev, __entry->ring, __entry->uniq,
		      __entry->seqno, __entry->blocking ?  "yes (NB)" : "no")
);

DEFINE_EVENT(i915_gem_request, i915_gem_request_wait_end,
	    TP_PROTO(struct drm_i915_gem_request *req),
	    TP_ARGS(req)
);

DECLARE_EVENT_CLASS(i915_ring,
	    TP_PROTO(struct intel_engine_cs *ring),
	    TP_ARGS(ring),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, ring)
			     ),

	    TP_fast_assign(
			   __entry->dev = ring->dev->primary->index;
			   __entry->ring = ring->id;
			   ),

	    TP_printk("dev=%u, ring=%u", __entry->dev, __entry->ring)
);

DEFINE_EVENT(i915_ring, i915_ring_wait_begin,
	    TP_PROTO(struct intel_engine_cs *ring),
	    TP_ARGS(ring)
);

DEFINE_EVENT(i915_ring, i915_ring_wait_end,
	    TP_PROTO(struct intel_engine_cs *ring),
	    TP_ARGS(ring)
);

TRACE_EVENT(i915_flip_request,
	    TP_PROTO(int plane, struct drm_i915_gem_object *obj),

	    TP_ARGS(plane, obj),

	    TP_STRUCT__entry(
		    __field(int, plane)
		    __field(struct drm_i915_gem_object *, obj)
		    ),

	    TP_fast_assign(
		    __entry->plane = plane;
		    __entry->obj = obj;
		    ),

	    TP_printk("plane=%d, obj=%p", __entry->plane, __entry->obj)
);

TRACE_EVENT(i915_flip_complete,
	    TP_PROTO(int plane, struct drm_i915_gem_object *obj),

	    TP_ARGS(plane, obj),

	    TP_STRUCT__entry(
		    __field(int, plane)
		    __field(struct drm_i915_gem_object *, obj)
		    ),

	    TP_fast_assign(
		    __entry->plane = plane;
		    __entry->obj = obj;
		    ),

	    TP_printk("plane=%d, obj=%p", __entry->plane, __entry->obj)
);

TRACE_EVENT_CONDITION(i915_reg_rw,
	TP_PROTO(bool write, u32 reg, u64 val, int len, bool trace),

	TP_ARGS(write, reg, val, len, trace),

	TP_CONDITION(trace),

	TP_STRUCT__entry(
		__field(u64, val)
		__field(u32, reg)
		__field(u16, write)
		__field(u16, len)
		),

	TP_fast_assign(
		__entry->val = (u64)val;
		__entry->reg = reg;
		__entry->write = write;
		__entry->len = len;
		),

	TP_printk("%s reg=0x%x, len=%d, val=(0x%x, 0x%x)",
		__entry->write ? "write" : "read",
		__entry->reg, __entry->len,
		(u32)(__entry->val & 0xffffffff),
		(u32)(__entry->val >> 32))
);

TRACE_EVENT(intel_gpu_freq_change,
	    TP_PROTO(u32 freq),
	    TP_ARGS(freq),

	    TP_STRUCT__entry(
			     __field(u32, freq)
			     ),

	    TP_fast_assign(
			   __entry->freq = freq;
			   ),

	    TP_printk("new_freq=%u", __entry->freq)
);

TRACE_EVENT(i915_scheduler_queue,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring  = ring->id;
			   __entry->uniq  = node ? node->params.request->uniq  : 0;
			   __entry->seqno = node ? node->params.request->seqno : 0;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d",
		      __entry->ring, __entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_scheduler_fly,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring  = ring->id;
			   __entry->uniq  = node ? node->params.request->uniq  : 0;
			   __entry->seqno = node ? node->params.request->seqno : 0;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d",
		      __entry->ring, __entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_scheduler_unfly,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring  = ring->id;
			   __entry->uniq  = node ? node->params.request->uniq  : 0;
			   __entry->seqno = node ? node->params.request->seqno : 0;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d",
		      __entry->ring, __entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_scheduler_landing,
	    TP_PROTO(struct intel_engine_cs *ring, u32 seqno,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, seqno, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     __field(u32, status)
			     ),

	    TP_fast_assign(
			   __entry->ring   = ring->id;
			   __entry->uniq   = node ? node->params.request->uniq  : 0;
			   __entry->seqno  = seqno;
			   __entry->status = node ? node->status : ~0U;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d, status=%d",
		      __entry->ring, __entry->uniq, __entry->seqno, __entry->status)
);

TRACE_EVENT(i915_scheduler_remove,
	    TP_PROTO(struct intel_engine_cs *ring,
		     u32 min_seqno, bool do_submit),
	    TP_ARGS(ring, min_seqno, do_submit),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, min_seqno)
			     __field(bool, do_submit)
			     ),

	    TP_fast_assign(
			   __entry->ring      = ring->id;
			   __entry->min_seqno = min_seqno;
			   __entry->do_submit = do_submit;
			   ),

	    TP_printk("ring=%d, min_seqno = %d, do_submit=%d",
		      __entry->ring, __entry->min_seqno, __entry->do_submit)
);

TRACE_EVENT(i915_scheduler_destroy,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring  = ring->id;
			   __entry->uniq  = node ? node->params.request->uniq  : 0;
			   __entry->seqno = node ? node->params.request->seqno : 0;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d",
		      __entry->ring, __entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_scheduler_pop_from_queue,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring  = ring->id;
			   __entry->uniq  = node ? node->params.request->uniq  : 0;
			   __entry->seqno = node ? node->params.request->seqno : 0;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d",
		      __entry->ring, __entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_scheduler_node_state_change,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_scheduler_queue_entry *node),
	    TP_ARGS(ring, node),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, uniq)
			     __field(u32, seqno)
			     __field(u32, status)
			     ),

	    TP_fast_assign(
			   __entry->ring   = ring->id;
			   __entry->uniq   = node ? node->params.request->uniq  : 0;
			   __entry->seqno  = node->params.request->seqno;
			   __entry->status = node->status;
			   ),

	    TP_printk("ring=%d, uniq=%d, seqno=%d, status=%d",
		      __entry->ring, __entry->uniq, __entry->seqno, __entry->status)
);


TRACE_EVENT(i915_scheduler_irq,
	TP_PROTO(struct i915_scheduler *scheduler,
		 struct intel_engine_cs *ring,
		 uint32_t seqno, bool direct_submit),
	TP_ARGS(scheduler, ring, seqno, direct_submit),

	TP_STRUCT__entry(
		__field(u32, ring_id)
		__field(u32, seqno)
		__field(u32, last_seqno)
		__field(bool, direct_submit)
		),

	TP_fast_assign(
		__entry->ring_id    = ring->id;
		__entry->seqno	    = seqno;
		__entry->last_seqno =
		    direct_submit ? 0 : scheduler->last_irq_seqno[ring->id];
		__entry->direct_submit = direct_submit;
		),

	TP_printk("ring=%d,seqno=%d,last_seqno=%d, direct_submit=%d",
		__entry->ring_id, __entry->seqno, __entry->last_seqno,
		__entry->direct_submit)
);

TRACE_EVENT(i915_gem_ring_queue,
	    TP_PROTO(struct intel_engine_cs *ring,
		     struct i915_execbuffer_params *params),
	    TP_ARGS(ring, params),

	    TP_STRUCT__entry(
			     __field(u32, ring)
			     __field(u32, seqno)
			     ),

	    TP_fast_assign(
			   __entry->ring   = ring->id;
			   __entry->seqno  = params->request->seqno;
			   ),

	    TP_printk("ring=%d, seqno=%d", __entry->ring, __entry->seqno)
);

/**
 * DOC: i915_ppgtt_create and i915_ppgtt_release tracepoints
 *
 * With full ppgtt enabled each process using drm will allocate at least one
 * translation table. With these traces it is possible to keep track of the
 * allocation and of the lifetime of the tables; this can be used during
 * testing/debug to verify that we are not leaking ppgtts.
 * These traces identify the ppgtt through the vm pointer, which is also printed
 * by the i915_vma_bind and i915_vma_unbind tracepoints.
 */
DECLARE_EVENT_CLASS(i915_ppgtt,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm),

	TP_STRUCT__entry(
			__field(struct i915_address_space *, vm)
			__field(u32, dev)
	),

	TP_fast_assign(
			__entry->vm = vm;
			__entry->dev = vm->dev->primary->index;
	),

	TP_printk("dev=%u, vm=%p", __entry->dev, __entry->vm)
)

DEFINE_EVENT(i915_ppgtt, i915_ppgtt_create,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm)
);

DEFINE_EVENT(i915_ppgtt, i915_ppgtt_release,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm)
);

/**
 * DOC: i915_context_create and i915_context_free tracepoints
 *
 * These tracepoints are used to track creation and deletion of contexts.
 * If full ppgtt is enabled, they also print the address of the vm assigned to
 * the context.
 */
DECLARE_EVENT_CLASS(i915_context,
	TP_PROTO(struct intel_context *ctx),
	TP_ARGS(ctx),

	TP_STRUCT__entry(
			__field(u32, dev)
			__field(struct intel_context *, ctx)
			__field(struct i915_address_space *, vm)
	),

	TP_fast_assign(
			__entry->ctx = ctx;
			__entry->vm = ctx->ppgtt ? &ctx->ppgtt->base : NULL;
			__entry->dev = ctx->file_priv->dev_priv->dev->primary->index;
	),

	TP_printk("dev=%u, ctx=%p, ctx_vm=%p",
		  __entry->dev, __entry->ctx, __entry->vm)
)

DEFINE_EVENT(i915_context, i915_context_create,
	TP_PROTO(struct intel_context *ctx),
	TP_ARGS(ctx)
);

DEFINE_EVENT(i915_context, i915_context_free,
	TP_PROTO(struct intel_context *ctx),
	TP_ARGS(ctx)
);

/**
 * DOC: switch_mm tracepoint
 *
 * This tracepoint allows tracking of the mm switch, which is an important point
 * in the lifetime of the vm in the legacy submission path. This tracepoint is
 * called only if full ppgtt is enabled.
 */
TRACE_EVENT(switch_mm,
	TP_PROTO(struct intel_engine_cs *ring, struct intel_context *to),

	TP_ARGS(ring, to),

	TP_STRUCT__entry(
			__field(u32, ring)
			__field(struct intel_context *, to)
			__field(struct i915_address_space *, vm)
			__field(u32, dev)
	),

	TP_fast_assign(
			__entry->ring = ring->id;
			__entry->to = to;
			__entry->vm = to->ppgtt ? &to->ppgtt->base : NULL;
			__entry->dev = ring->dev->primary->index;
	),

	TP_printk("dev=%u, ring=%u, ctx=%p, ctx_vm=%p",
		  __entry->dev, __entry->ring, __entry->to, __entry->vm)
);

DECLARE_EVENT_CLASS(i915_suspend_resume_enter,
	TP_PROTO(struct drm_device *dev),
	TP_ARGS(dev),

	TP_STRUCT__entry(
			__field(u32, dev)
	),

	TP_fast_assign(
			__entry->dev = dev ? dev->primary->index : -1;
	),

	TP_printk("dev = %u", __entry->dev)
);

DECLARE_EVENT_CLASS(i915_suspend_resume_exit,
	TP_PROTO(struct drm_device *dev, int ret),
	TP_ARGS(dev, ret),

	TP_STRUCT__entry(
			__field(u32, dev)
			__field(int, ret)
	),

	TP_fast_assign(
			__entry->dev = dev ? dev->primary->index : -1;
			__entry->ret = ret;
	),

	TP_printk("dev = %u, ret = %d", __entry->dev, __entry->ret)
);

DEFINE_EVENT(i915_suspend_resume_enter, i915_pm_suspend_enter,
	TP_PROTO(struct drm_device *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(i915_suspend_resume_exit, i915_pm_suspend_exit,
	TP_PROTO(struct drm_device *dev, int ret),
	TP_ARGS(dev, ret)
);

DEFINE_EVENT(i915_suspend_resume_enter, i915_pm_resume_enter,
	TP_PROTO(struct drm_device *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(i915_suspend_resume_exit, i915_pm_resume_exit,
	TP_PROTO(struct drm_device *dev, int ret),
	TP_ARGS(dev, ret)
);

DEFINE_EVENT(i915_suspend_resume_enter, intel_runtime_suspend_enter,
	TP_PROTO(struct drm_device *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(i915_suspend_resume_exit, intel_runtime_suspend_exit,
	TP_PROTO(struct drm_device *dev, int ret),
	TP_ARGS(dev, ret)
);

DEFINE_EVENT(i915_suspend_resume_enter, intel_runtime_resume_enter,
	TP_PROTO(struct drm_device *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(i915_suspend_resume_exit, intel_runtime_resume_exit,
	TP_PROTO(struct drm_device *dev, int ret),
	TP_ARGS(dev, ret)
);

TRACE_EVENT(execlists_elsp_write,
	TP_PROTO(struct intel_engine_cs *ring, u32 desc0,
			u32 desc1, u32 desc2, u32 desc3),

	TP_ARGS(ring, desc0, desc1, desc2, desc3),

	TP_STRUCT__entry(
		__field(u32, ring)
		__field(u32, desc0)
		__field(u32, desc1)
		__field(u32, desc2)
		__field(u32, desc3)
	),

	TP_fast_assign(
		__entry->ring = ring->id;
		__entry->desc0 = desc0;
		__entry->desc1 = desc1;
		__entry->desc2 = desc2;
		__entry->desc3 = desc3;
	),

	TP_printk("ring=%u,desc0=%u,desc1=%u,desc2=%u,desc3=%u",
		__entry->ring, __entry->desc0, __entry->desc1,
		__entry->desc2, __entry->desc3)
);


TRACE_EVENT(execlists_context_unqueue,
	TP_PROTO(struct intel_engine_cs *ring,
		 struct intel_ctx_submit_request *req0,
		 struct intel_ctx_submit_request *req1),

	TP_ARGS(ring, req0, req1),

	TP_STRUCT__entry(
		__field(u32, ring)
		__field(struct intel_context *, ctx0)
		__field(u32, req0_tail)
		__field(struct intel_context *, ctx1)
		__field(u32, req1_tail)
	),

	TP_fast_assign(
		__entry->ring = ring->id;
		__entry->ctx0 = req0 ? req0->ctx : NULL;
		__entry->req0_tail = req0 ? req0->tail : 0;
		__entry->ctx1 = req1 ? req1->ctx : NULL;
		__entry->req1_tail = req1 ? req1->tail : 0;
	),

	TP_printk("ring=%u,req0->ctx=%p,req0->tail=%u,req1->ctx=%p,req1->tail=%u",
		__entry->ring, __entry->ctx0, __entry->req0_tail,
		__entry->ctx1, __entry->req1_tail)
);

TRACE_EVENT(intel_execlists_handle_ctx_events,
	TP_PROTO(struct intel_engine_cs *ring, u32 status_pointer,
			u32 read_pointer),

	TP_ARGS(ring, status_pointer, read_pointer),

	TP_STRUCT__entry(
		__field(u32, ring)
		__field(u32, status_pointer)
		__field(u32, read_pointer)
	),

	TP_fast_assign(
		__entry->ring = ring->id;
		__entry->status_pointer = status_pointer;
		__entry->read_pointer = read_pointer;
	),

	TP_printk("ring=%d,read_pointer=%d,write_pointer=%d,status=0x%x",
		__entry->ring, __entry->read_pointer,
		__entry->status_pointer & 0x07, __entry->status_pointer)
);

TRACE_EVENT(execlists_context_queue,
	TP_PROTO(struct intel_ctx_submit_request *req),

	TP_ARGS(req),

	TP_STRUCT__entry(
		__field(struct intel_context *, ctx)
		__field(u32, ring)
		__field(u32, tail)
	),

	TP_fast_assign(
		__entry->ctx  = req->ctx;
		__entry->ring = req->ring->id;
		__entry->tail = req->tail;
	),

	TP_printk("ring=%u,ctx=%p,req->tail=%u",
		__entry->ring, __entry->ctx, __entry->tail)
);

/* queue_retire_work - begin */
TRACE_EVENT(queue_retire_work,
	TP_PROTO(u64 time, bool bSuccess),

	TP_ARGS(time, bSuccess),

	TP_STRUCT__entry(
			__field(u64, time)
			__field(bool, bSuccess)
	),

	TP_fast_assign(
			__entry->time = time;
			__entry->bSuccess = bSuccess;
	),

	TP_printk("time=%lld, queue retire work sucess=%d",
		__entry->time, __entry->bSuccess)
);
/* queue_retire_work - end */

TRACE_EVENT(i915_gem_request_complete_begin,
	TP_PROTO(struct intel_engine_cs *ring, u32 seqno),

	TP_ARGS(ring, seqno),

	TP_STRUCT__entry(
		__field(u32, ring)
		__field(u32, seqno)
		__field(u32, last_seqno)
	),

	TP_fast_assign(
	    __entry->ring = ring->id;
	    __entry->seqno = seqno;
	    __entry->last_seqno = ring->last_read_seqno;
	),

	TP_printk("ring=%u,seqno=%d,last_seqno=%d",
		__entry->ring, __entry->seqno, __entry->last_seqno)
);

TRACE_EVENT(i915_gem_retire_work_handler,
	TP_PROTO(struct drm_device *dev, bool idle),

	TP_ARGS(dev, idle),

	TP_STRUCT__entry(
		__field(struct drm_device *, dev)
		__field(bool, idle)
	),

	TP_fast_assign(
		__entry->dev = dev;
		__entry->idle = idle;
	),

	TP_printk("dev=%p,idle=%d",
		__entry->dev, __entry->idle)
);

TRACE_EVENT(i915_gem_context_reference,
	TP_PROTO(struct intel_context *ctx),
	TP_ARGS(ctx),

	TP_STRUCT__entry(
			__field(void *, ctx)
			__field(u32, refcnt)
			),

	TP_fast_assign(
			__entry->ctx   = ctx;
			__entry->refcnt = atomic_read(&(&ctx->ref)->refcount);
			),

	TP_printk("ctx=%p, refcnt before +1=%d",
		__entry->ctx, __entry->refcnt)
);

TRACE_EVENT(i915_gem_context_unreference,
	TP_PROTO(struct intel_context *ctx),
	TP_ARGS(ctx),

	TP_STRUCT__entry(
			__field(void *, ctx)
			__field(u32, refcnt)
			),

	TP_fast_assign(
			__entry->ctx   = ctx;
			__entry->refcnt  = atomic_read(&(&ctx->ref)->refcount);
			),

	TP_printk("ctx=%p, refcnt before -1=%d",
		__entry->ctx, __entry->refcnt)
);

TRACE_EVENT(i915_gem_request_reference,
	TP_PROTO(struct drm_i915_gem_request *req),
	TP_ARGS(req),

	TP_STRUCT__entry(
			__field(u32, seqno)
			__field(u32, uniq)
			__field(u32, refcnt)
			),

	TP_fast_assign(
			__entry->uniq = req ? req->uniq : 0;
			__entry->seqno = i915_gem_request_get_seqno(req);
			__entry->refcnt  = atomic_read(&(&req->ref)->refcount);
			),

	TP_printk("uniq=%d, seqno=%d, refcnt before +1=%d",
		__entry->uniq, __entry->seqno, __entry->refcnt)
);

TRACE_EVENT(i915_gem_request_unreference,
	TP_PROTO(struct drm_i915_gem_request *req),
	TP_ARGS(req),

	TP_STRUCT__entry(
			__field(u32, seqno)
			__field(u32, uniq)
			__field(u32, refcnt)
			),

	TP_fast_assign(
			__entry->uniq = req ? req->uniq : 0;
			__entry->seqno = i915_gem_request_get_seqno(req);
			__entry->refcnt = atomic_read(&(&req->ref)->refcount);
			),

	TP_printk("uniq=%d,seqno=%d, refcnt before -1=%d",
		__entry->uniq, __entry->seqno, __entry->refcnt)
);

TRACE_EVENT(i915_gem_object_move_to_active,
	TP_PROTO(struct drm_i915_gem_object *obj,
			struct drm_i915_gem_request *req),
	TP_ARGS(obj, req),

	TP_STRUCT__entry(
			__field(void *, obj)
			__field(u32, uniq)
			__field(u32, seqno)
			),

	TP_fast_assign(
			__entry->obj  = obj;
			__entry->uniq = req ? req->uniq : 0;
			__entry->seqno = i915_gem_request_get_seqno(req);
			),

	TP_printk("obj=%p, uniq=%d, seqno=%d", __entry->obj,
		__entry->uniq, __entry->seqno)
);

TRACE_EVENT(i915_gem_object_move_to_inactive,
	TP_PROTO(struct drm_i915_gem_object *obj),
	TP_ARGS(obj),

	TP_STRUCT__entry(
			__field(void *, obj)
			),

	TP_fast_assign(
			__entry->obj  = obj;
			),

	TP_printk("obj=%p", __entry->obj)
);

#endif /* _I915_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
