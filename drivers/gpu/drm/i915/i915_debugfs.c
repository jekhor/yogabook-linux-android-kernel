/*
 * Copyright © 2008 Intel Corporation
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
 *    Keith Packard <keithp@keithp.com>
 *
 */

#include <linux/seq_file.h>
#include <linux/circ_buf.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/list_sort.h>
#include <asm/msr-index.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include "intel_ringbuffer.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "intel_clrmgr.h"
#include "i915_scheduler.h"
#include "intel_esd_handler.h"
extern struct esd_private i915_esd;

enum {
	ACTIVE_LIST,
	INACTIVE_LIST,
	PINNED_LIST,
};

static const char *yesno(int v)
{
	return v ? "yes" : "no";
}

/* As the drm_debugfs_init() routines are called before dev->dev_private is
 * allocated we need to hook into the minor for release. */
static int
drm_add_fake_info_node(struct drm_minor *minor,
		       struct dentry *ent,
		       const void *key)
{
	struct drm_info_node *node;

	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (node == NULL) {
		debugfs_remove(ent);
		return -ENOMEM;
	}

	node->minor = minor;
	node->dent = ent;
	node->info_ent = (void *) key;

	mutex_lock(&minor->debugfs_lock);
	list_add(&node->list, &minor->debugfs_list);
	mutex_unlock(&minor->debugfs_lock);

	return 0;
}

static int i915_capabilities(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	const struct intel_device_info *info = INTEL_INFO(dev);

	seq_printf(m, "gen: %d\n", info->gen);
	seq_printf(m, "pch: %d\n", INTEL_PCH_TYPE(dev));
#define PRINT_FLAG(x)  seq_printf(m, #x ": %s\n", yesno(info->x))
#define SEP_SEMICOLON ;
	DEV_INFO_FOR_EACH_FLAG(PRINT_FLAG, SEP_SEMICOLON);
#undef PRINT_FLAG
#undef SEP_SEMICOLON

	return 0;
}

static const char *get_pin_flag(struct drm_i915_gem_object *obj)
{
	if (obj->user_pin_count > 0)
		return "P";
	else if (i915_gem_obj_is_pinned(obj))
		return "p";
	else
		return " ";
}

static const char *get_tiling_flag(struct drm_i915_gem_object *obj)
{
	switch (obj->tiling_mode) {
	default:
	case I915_TILING_NONE: return " ";
	case I915_TILING_X: return "X";
	case I915_TILING_Y: return "Y";
	}
}

static inline const char *get_global_flag(struct drm_i915_gem_object *obj)
{
	return obj->has_global_gtt_mapping ? "g" : " ";
}

ssize_t i915_display_register_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	int len = 0;
	char buf[] = "====>Version 0.4.<====\n"
					"CMD format: [r/a] [reg_addr] [num/end_addr]\n"
					"Display register list:\n"
					"\t0x05000 to 0x05FFF\tGMBUS and I/O Control\n"
					"\t0x06000 to 0x062FF\tDisplay Clocks and Clock Gating\n"
					"\t0x0A000 to 0x0B7FF\tDisplay Palette A/B/C Registers\n"
					"\t0x50500 to 0x51004\tWIDI Inbound messages\n"
					"\t0x60000 to 0x600FF\tDisplay Pipeline A\n"
					"\t0x61000 to 0x610FF\tDisplay Pipeline B\n"
					"\t0x63000 to 0x630FF\tDisplay Pipeline C\n"
					"\t0x61100 to 0x611FF\tDisplay Port Control Registers\n"
					"\t0x61200 to 0x613FF\tPanel Fitting/Power Sequencing/LVDS Control\n"
					"\t0x62000 to 0x62FFF\tHD Audio Control\n"
					"\t0x64000 to 0x64FFF\tDisplayPort Registers\n"
					"\t0x65000 to 0x65FFF\tLPE Audio Registers\n"
					"\t0x70000 to 0x7FFFF\tDisplay Pipeline, Display Planes, Cursor Planes, and VGA Control Registers\n";

	len = sizeof(buf);
	return simple_read_from_buffer(ubuf, max, ppos,
		(const void *) buf, len);
}

/*
* use to read display register.
*/
ssize_t i915_display_register_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char *buf = NULL;
	char op = '0';
	int  reg = 0, num = 0, start = 0, end = 0;
	unsigned int  val = 0, reg_val = 0;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Display register: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc((count + 1), GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Display register: Out of mem\n");
		ret = -ENOMEM;
		goto EXIT;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Display register: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}
	buf[count] = '\0';

	sscanf(buf, "%c %x %x", &op, &reg, &val);

	if (op != 'r' && op != 'a') {
		DRM_ERROR("The input format is not right!\n");
		DRM_ERROR("[r/a] [reg_addr] [num/end_addr]\n");
		DRM_ERROR("for exampe: r 70184		(read register 70184.)\n");
		DRM_ERROR("for exmape: a 60000 60010(read all registers start at 60000 and end at 60010.\n)");
		ret = -EINVAL;
		goto EXIT;
	}

	if ((reg % 0x4) != 0) {
		DRM_ERROR("the register address should aligned to 4 byte.please refrence display controller specification.\n");
		ret = -EINVAL;
		goto EXIT;
	}

	if (op == 'r') {
		do {
			reg_val = I915_READ(dev_priv->info.display_mmio_offset + reg);
			DRM_INFO("Read: reg=0x%08x , \tval=0x%08x .\n", reg, reg_val);
			num++;
			reg += 4;
		} while (num < val);
	}
	if (op == 'a') {
		start = reg;
		end = val;
		if (start >= end) {
			DRM_ERROR("The end address is smaller than the start address.\n");
			ret = -EINVAL;
			goto EXIT;
		}
		reg += dev_priv->info.display_mmio_offset;
		DRM_INFO("START ADDR: 0x%08x, END ADDR: 0x%08x.\n", start, end);
		num = (end - start)/16;
		while (num > 0) {
			DRM_INFO("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				(reg - dev_priv->info.display_mmio_offset), I915_READ(reg), I915_READ(reg + 0x4), I915_READ(reg + 0x8), I915_READ(reg + 0xc));
			reg += 0x10;
			num--;
		}
		num = (end - start)%16;
		if (num >= 0xc) {
			DRM_INFO("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				(reg - dev_priv->info.display_mmio_offset), I915_READ(reg), I915_READ(reg + 0x4), I915_READ(reg + 0x8), I915_READ(reg + 0xc));
		} else if (num >= 0x8) {
			DRM_INFO("0x%08x: 0x%08x 0x%08x 0x%08x\n",
				(reg - dev_priv->info.display_mmio_offset), I915_READ(reg), I915_READ(reg + 0x4), I915_READ(reg + 0x8));
		} else if (num >= 0x4) {
			DRM_INFO("0x%08x: 0x%08x 0x%08x\n",
				(reg - dev_priv->info.display_mmio_offset), I915_READ(reg), I915_READ(reg + 0x4));
		} else if (num > 0) {
			DRM_INFO("0x%08x: 0x%08x\n",
				(reg - dev_priv->info.display_mmio_offset), I915_READ(reg));
		}
	}

	ret = count;

EXIT:
	if (buf)
		kfree(buf);

	return ret;
}

const struct file_operations i915_display_register_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_display_register_read,
	.write = i915_display_register_write,
	.llseek = default_llseek,
};



static void
describe_obj(struct seq_file *m, struct drm_i915_gem_object *obj)
{
	struct i915_vma *vma;
	int pin_count = 0;

	seq_printf(m, "%p: %s%s%s %8zdKiB %02x %02x %u %u %u%s%s%s",
		   &obj->base,
		   get_pin_flag(obj),
		   get_tiling_flag(obj),
		   get_global_flag(obj),
		   obj->base.size / 1024,
		   obj->base.read_domains,
		   obj->base.write_domain,
		   i915_gem_request_get_seqno(obj->last_read_req),
		   i915_gem_request_get_seqno(obj->last_write_req),
		   i915_gem_request_get_seqno(obj->last_fenced_req),
		   i915_cache_level_str(obj->cache_level),
		   obj->dirty ? " dirty" : "",
		   obj->madv == I915_MADV_DONTNEED ? " purgeable" : "");
	if (obj->base.name)
		seq_printf(m, " (name: %d)", obj->base.name);
	list_for_each_entry(vma, &obj->vma_list, vma_link)
		if (vma->pin_count > 0)
			pin_count++;
		seq_printf(m, " (pinned x %d)", pin_count);
	if (obj->pin_display)
		seq_printf(m, " (display)");
	if (obj->fence_reg != I915_FENCE_REG_NONE)
		seq_printf(m, " (fence: %d)", obj->fence_reg);
	list_for_each_entry(vma, &obj->vma_list, vma_link) {
		if (!i915_is_ggtt(vma->vm))
			seq_puts(m, " (pp");
		else
			seq_puts(m, " (g");
		seq_printf(m, "gtt offset: %08lx, size: %08lx)",
			   vma->node.start, vma->node.size);
	}
	if (obj->stolen)
		seq_printf(m, " (stolen: %08lx)", obj->stolen->start);
	if (obj->pin_mappable || obj->fault_mappable) {
		char s[3], *t = s;
		if (obj->pin_mappable)
			*t++ = 'p';
		if (obj->fault_mappable)
			*t++ = 'f';
		*t = '\0';
		seq_printf(m, " (%s mappable)", s);
	}
	if (obj->last_read_req != NULL)
		seq_printf(m, " (%s)",
			   i915_gem_request_get_ring(obj->last_read_req)->name);
}

static void describe_ctx(struct seq_file *m, struct intel_context *ctx)
{
	seq_putc(m, ctx->legacy_hw_ctx.initialized ? 'I' : 'i');
	seq_putc(m, ctx->remap_slice ? 'R' : 'r');
	seq_putc(m, ' ');
}

static int i915_gem_object_list_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct list_head *head;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_address_space *vm = &dev_priv->gtt.base;
	struct i915_vma *vma;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	/* FIXME: the user of this interface might want more than just GGTT */
	switch (list) {
	case ACTIVE_LIST:
		seq_puts(m, "Active:\n");
		head = &vm->active_list;
		break;
	case INACTIVE_LIST:
		seq_puts(m, "Inactive:\n");
		head = &vm->inactive_list;
		break;
	default:
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(vma, head, mm_list) {
		seq_printf(m, "   ");
		describe_obj(m, vma->obj);
		seq_printf(m, "\n");
		total_obj_size += vma->obj->base.size;
		total_gtt_size += vma->node.size;
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);
	return 0;
}

static int obj_rank_by_stolen(void *priv,
			      struct list_head *A, struct list_head *B)
{
	struct drm_i915_gem_object *a =
		container_of(A, struct drm_i915_gem_object, obj_exec_link);
	struct drm_i915_gem_object *b =
		container_of(B, struct drm_i915_gem_object, obj_exec_link);

	return a->stolen->start - b->stolen->start;
}

static int i915_gem_stolen_list_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	LIST_HEAD(stolen);
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (obj->stolen == NULL)
			continue;

		list_add(&obj->obj_exec_link, &stolen);

		total_obj_size += obj->base.size;
		total_gtt_size += i915_gem_obj_ggtt_size(obj);
		count++;
	}
	list_for_each_entry(obj, &dev_priv->mm.unbound_list, global_list) {
		if (obj->stolen == NULL)
			continue;

		list_add(&obj->obj_exec_link, &stolen);

		total_obj_size += obj->base.size;
		count++;
	}
	list_sort(NULL, &stolen, obj_rank_by_stolen);
	seq_puts(m, "Stolen:\n");
	while (!list_empty(&stolen)) {
		obj = list_first_entry(&stolen, typeof(*obj), obj_exec_link);
		seq_puts(m, "   ");
		describe_obj(m, obj);
		seq_putc(m, '\n');
		list_del_init(&obj->obj_exec_link);
	}
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);
	return 0;
}

#define count_objects(list, member) do { \
	list_for_each_entry(obj, list, member) { \
		size += i915_gem_obj_ggtt_size(obj); \
		++count; \
		if (obj->map_and_fenceable) { \
			mappable_size += i915_gem_obj_ggtt_size(obj); \
			++mappable_count; \
		} \
	} \
} while (0)

struct file_stats {
	struct drm_i915_file_private *file_priv;
	int count;
	size_t total, unbound;
	size_t global, shared;
	size_t active, inactive;
};

static int per_file_stats(int id, void *ptr, void *data)
{
	struct drm_i915_gem_object *obj = ptr;
	struct file_stats *stats = data;
	struct i915_vma *vma;

	stats->count++;
	stats->total += obj->base.size;

	if (obj->base.name || obj->base.dma_buf)
		stats->shared += obj->base.size;

	if (USES_FULL_PPGTT(obj->base.dev)) {
		list_for_each_entry(vma, &obj->vma_list, vma_link) {
			struct i915_hw_ppgtt *ppgtt;

			if (!drm_mm_node_allocated(&vma->node))
				continue;

			if (i915_is_ggtt(vma->vm)) {
				stats->global += obj->base.size;
				continue;
			}

			ppgtt = container_of(vma->vm, struct i915_hw_ppgtt, base);
			if (ppgtt->file_priv != stats->file_priv)
				continue;

			if (obj->active) /* XXX per-vma statistic */
				stats->active += obj->base.size;
			else
				stats->inactive += obj->base.size;

			return 0;
		}
	} else {
		if (i915_gem_obj_ggtt_bound(obj)) {
			stats->global += obj->base.size;
			if (obj->active)
				stats->active += obj->base.size;
			else
				stats->inactive += obj->base.size;
			return 0;
		}
	}

	if (!list_empty(&obj->global_list))
		stats->unbound += obj->base.size;

	return 0;
}

#define count_vmas(list, member) do { \
	list_for_each_entry(vma, list, member) { \
		size += i915_gem_obj_ggtt_size(vma->obj); \
		++count; \
		if (vma->obj->map_and_fenceable) { \
			mappable_size += i915_gem_obj_ggtt_size(vma->obj); \
			++mappable_count; \
		} \
	} \
} while (0)

static int i915_gem_object_info(struct seq_file *m, void* data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 count, mappable_count, purgeable_count;
	size_t size, mappable_size, purgeable_size;
	struct drm_i915_gem_object *obj;
	struct i915_address_space *vm = &dev_priv->gtt.base;
	struct drm_file *file;
	struct i915_vma *vma;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "%u objects, %zu bytes\n",
		   dev_priv->mm.object_count,
		   dev_priv->mm.object_memory);

	size = count = mappable_size = mappable_count = 0;
	count_objects(&dev_priv->mm.bound_list, global_list);
	seq_printf(m, "%u [%u] objects, %zu [%zu] bytes in gtt\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_vmas(&vm->active_list, mm_list);
	seq_printf(m, "  %u [%u] active objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_vmas(&vm->inactive_list, mm_list);
	seq_printf(m, "  %u [%u] inactive objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = purgeable_size = purgeable_count = 0;
	list_for_each_entry(obj, &dev_priv->mm.unbound_list, global_list) {
		size += obj->base.size, ++count;
		if (obj->madv == I915_MADV_DONTNEED)
			purgeable_size += obj->base.size, ++purgeable_count;
	}
	seq_printf(m, "%u unbound objects, %zu bytes\n", count, size);

	size = count = mappable_size = mappable_count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (obj->fault_mappable) {
			size += i915_gem_obj_ggtt_size(obj);
			++count;
		}
		if (obj->pin_mappable) {
			mappable_size += i915_gem_obj_ggtt_size(obj);
			++mappable_count;
		}
		if (obj->madv == I915_MADV_DONTNEED) {
			purgeable_size += obj->base.size;
			++purgeable_count;
		}
	}
	seq_printf(m, "%u purgeable objects, %zu bytes\n",
		   purgeable_count, purgeable_size);
	seq_printf(m, "%u pinned mappable objects, %zu bytes\n",
		   mappable_count, mappable_size);
	seq_printf(m, "%u fault mappable objects, %zu bytes\n",
		   count, size);

	seq_printf(m, "%zu [%lu] gtt total\n",
		   dev_priv->gtt.base.total,
		   dev_priv->gtt.mappable_end - dev_priv->gtt.base.start);

	seq_putc(m, '\n');
	list_for_each_entry_reverse(file, &dev->filelist, lhead) {
		struct file_stats stats;
		struct task_struct *task;

		memset(&stats, 0, sizeof(stats));
		stats.file_priv = file->driver_priv;
		spin_lock(&file->table_lock);
		idr_for_each(&file->object_idr, per_file_stats, &stats);
		spin_unlock(&file->table_lock);
		/*
		 * Although we have a valid reference on file->pid, that does
		 * not guarantee that the task_struct who called get_pid() is
		 * still alive (e.g. get_pid(current) => fork() => exit()).
		 * Therefore, we need to protect this ->comm access using RCU.
		 */
		rcu_read_lock();
		task = pid_task(file->pid, PIDTYPE_PID);
		seq_printf(m, "%s: %u objects, %zu bytes (%zu active, %zu inactive, %zu global, %zu shared, %zu unbound)\n",
			   task ? task->comm : "<unknown>",
			   stats.count,
			   stats.total,
			   stats.active,
			   stats.inactive,
			   stats.global,
			   stats.shared,
			   stats.unbound);
		rcu_read_unlock();
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_gtt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, &dev_priv->mm.bound_list, global_list) {
		if (list == PINNED_LIST && !i915_gem_obj_is_pinned(obj))
			continue;

		seq_puts(m, "   ");
		describe_obj(m, obj);
		seq_putc(m, '\n');
		total_obj_size += obj->base.size;
		total_gtt_size += i915_gem_obj_ggtt_size(obj);
		count++;
	}

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);

	return 0;
}

static int i915_gem_pageflip_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	unsigned long flags;
	struct intel_crtc *crtc;

	for_each_intel_crtc(dev, crtc) {
		const char pipe = pipe_name(crtc->pipe);
		const char plane = plane_name(crtc->plane);
		struct intel_unpin_work *work;

		spin_lock_irqsave(&dev->event_lock, flags);
		work = crtc->unpin_work;
		if (work == NULL) {
			seq_printf(m, "No flip due on pipe %c (plane %c)\n",
				   pipe, plane);
		} else {
			if (atomic_read(&work->pending) < INTEL_FLIP_COMPLETE) {
				seq_printf(m, "Flip queued on pipe %c (plane %c)\n",
					   pipe, plane);
			} else {
				seq_printf(m, "Flip pending (waiting for vsync) on pipe %c (plane %c)\n",
					   pipe, plane);
			}
			if (work->enable_stall_check)
				seq_puts(m, "Stall check enabled, ");
			else
				seq_puts(m, "Stall check waiting for page flip ioctl, ");
			seq_printf(m, "%d prepares\n", atomic_read(&work->pending));

			if (work->old_fb_obj) {
				struct drm_i915_gem_object *obj = work->old_fb_obj;
				if (obj)
					seq_printf(m, "Old framebuffer gtt_offset 0x%08lx\n",
						   i915_gem_obj_ggtt_offset(obj));
			}
			if (work->pending_flip_obj) {
				struct drm_i915_gem_object *obj = work->pending_flip_obj;
				if (obj)
					seq_printf(m, "New framebuffer gtt_offset 0x%08lx\n",
						   i915_gem_obj_ggtt_offset(obj));
			}
		}
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	return 0;
}

static int i915_gem_request_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	struct drm_i915_gem_request *gem_request;
	int ret, count, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	count = 0;
	for_each_ring(ring, dev_priv, i) {
		if (list_empty(&ring->request_list))
			continue;

		seq_printf(m, "%s requests:\n", ring->name);
		list_for_each_entry(gem_request,
				    &ring->request_list,
				    list) {
			seq_printf(m, "    %d @ %d\n",
				   gem_request->seqno,
				   (int) (jiffies - gem_request->emitted_jiffies));
		}
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	if (count == 0)
		seq_puts(m, "No requests\n");

	return 0;
}

static void i915_ring_seqno_info(struct seq_file *m,
				 struct intel_engine_cs *ring)
{
	if (ring->get_seqno) {
		seq_printf(m, "Current sequence (%s): %u\n",
			   ring->name, ring->get_seqno(ring, false));
	}
}

static int i915_gem_seqno_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	for_each_ring(ring, dev_priv, i)
		i915_ring_seqno_info(m, ring);

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}


static int i915_interrupt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	int ret, i, pipe;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	if (IS_CHERRYVIEW(dev)) {
		int i;
		seq_printf(m, "Master Interrupt Control:\t%08x\n",
			   I915_READ(GEN8_MASTER_IRQ));

		seq_printf(m, "Display IER:\t%08x\n",
			   I915_READ(VLV_IER));
		seq_printf(m, "Display IIR:\t%08x\n",
			   I915_READ(VLV_IIR));
		seq_printf(m, "Display IIR_RW:\t%08x\n",
			   I915_READ(VLV_IIR_RW));
		seq_printf(m, "Display IMR:\t%08x\n",
			   I915_READ(VLV_IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));

		seq_printf(m, "Port hotplug:\t%08x\n",
			   I915_READ(PORT_HOTPLUG_EN));
		seq_printf(m, "DPFLIPSTAT:\t%08x\n",
			   I915_READ(VLV_DPFLIPSTAT));
		seq_printf(m, "DPINVGTT:\t%08x\n",
			   I915_READ(DPINVGTT));

		for (i = 0; i < 4; i++) {
			seq_printf(m, "GT Interrupt IMR %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IMR(i)));
			seq_printf(m, "GT Interrupt IIR %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IIR(i)));
			seq_printf(m, "GT Interrupt IER %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IER(i)));
		}

		seq_printf(m, "PCU interrupt mask:\t%08x\n",
			   I915_READ(GEN8_PCU_IMR));
		seq_printf(m, "PCU interrupt identity:\t%08x\n",
			   I915_READ(GEN8_PCU_IIR));
		seq_printf(m, "PCU interrupt enable:\t%08x\n",
			   I915_READ(GEN8_PCU_IER));
	} else if (INTEL_INFO(dev)->gen >= 8) {
		seq_printf(m, "Master Interrupt Control:\t%08x\n",
			   I915_READ(GEN8_MASTER_IRQ));

		for (i = 0; i < 4; i++) {
			seq_printf(m, "GT Interrupt IMR %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IMR(i)));
			seq_printf(m, "GT Interrupt IIR %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IIR(i)));
			seq_printf(m, "GT Interrupt IER %d:\t%08x\n",
				   i, I915_READ(GEN8_GT_IER(i)));
		}

		for_each_pipe(pipe) {
			seq_printf(m, "Pipe %c IMR:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(GEN8_DE_PIPE_IMR(pipe)));
			seq_printf(m, "Pipe %c IIR:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(GEN8_DE_PIPE_IIR(pipe)));
			seq_printf(m, "Pipe %c IER:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(GEN8_DE_PIPE_IER(pipe)));
		}

		seq_printf(m, "Display Engine port interrupt mask:\t%08x\n",
			   I915_READ(GEN8_DE_PORT_IMR));
		seq_printf(m, "Display Engine port interrupt identity:\t%08x\n",
			   I915_READ(GEN8_DE_PORT_IIR));
		seq_printf(m, "Display Engine port interrupt enable:\t%08x\n",
			   I915_READ(GEN8_DE_PORT_IER));

		seq_printf(m, "Display Engine misc interrupt mask:\t%08x\n",
			   I915_READ(GEN8_DE_MISC_IMR));
		seq_printf(m, "Display Engine misc interrupt identity:\t%08x\n",
			   I915_READ(GEN8_DE_MISC_IIR));
		seq_printf(m, "Display Engine misc interrupt enable:\t%08x\n",
			   I915_READ(GEN8_DE_MISC_IER));

		seq_printf(m, "PCU interrupt mask:\t%08x\n",
			   I915_READ(GEN8_PCU_IMR));
		seq_printf(m, "PCU interrupt identity:\t%08x\n",
			   I915_READ(GEN8_PCU_IIR));
		seq_printf(m, "PCU interrupt enable:\t%08x\n",
			   I915_READ(GEN8_PCU_IER));
	} else if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "Display IER:\t%08x\n",
			   I915_READ(VLV_IER));
		seq_printf(m, "Display IIR:\t%08x\n",
			   I915_READ(VLV_IIR));
		seq_printf(m, "Display IIR_RW:\t%08x\n",
			   I915_READ(VLV_IIR_RW));
		seq_printf(m, "Display IMR:\t%08x\n",
			   I915_READ(VLV_IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));

		seq_printf(m, "Master IER:\t%08x\n",
			   I915_READ(VLV_MASTER_IER));

		seq_printf(m, "Render IER:\t%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Render IIR:\t%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Render IMR:\t%08x\n",
			   I915_READ(GTIMR));

		seq_printf(m, "PM IER:\t\t%08x\n",
			   I915_READ(GEN6_PMIER));
		seq_printf(m, "PM IIR:\t\t%08x\n",
			   I915_READ(GEN6_PMIIR));
		seq_printf(m, "PM IMR:\t\t%08x\n",
			   I915_READ(GEN6_PMIMR));

		seq_printf(m, "Port hotplug:\t%08x\n",
			   I915_READ(PORT_HOTPLUG_EN));
		seq_printf(m, "DPFLIPSTAT:\t%08x\n",
			   I915_READ(VLV_DPFLIPSTAT));
		seq_printf(m, "DPINVGTT:\t%08x\n",
			   I915_READ(DPINVGTT));

	} else if (!HAS_PCH_SPLIT(dev)) {
		seq_printf(m, "Interrupt enable:    %08x\n",
			   I915_READ(IER));
		seq_printf(m, "Interrupt identity:  %08x\n",
			   I915_READ(IIR));
		seq_printf(m, "Interrupt mask:      %08x\n",
			   I915_READ(IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:         %08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));
	} else {
		seq_printf(m, "North Display Interrupt enable:		%08x\n",
			   I915_READ(DEIER));
		seq_printf(m, "North Display Interrupt identity:	%08x\n",
			   I915_READ(DEIIR));
		seq_printf(m, "North Display Interrupt mask:		%08x\n",
			   I915_READ(DEIMR));
		seq_printf(m, "South Display Interrupt enable:		%08x\n",
			   I915_READ(SDEIER));
		seq_printf(m, "South Display Interrupt identity:	%08x\n",
			   I915_READ(SDEIIR));
		seq_printf(m, "South Display Interrupt mask:		%08x\n",
			   I915_READ(SDEIMR));
		seq_printf(m, "Graphics Interrupt enable:		%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Graphics Interrupt identity:		%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Graphics Interrupt mask:		%08x\n",
			   I915_READ(GTIMR));
	}
	for_each_ring(ring, dev_priv, i) {
		if (INTEL_INFO(dev)->gen >= 6) {
			seq_printf(m,
				   "Graphics Interrupt mask (%s):	%08x\n",
				   ring->name, I915_READ_IMR(ring));
		}
		i915_ring_seqno_info(m, ring);
	}
	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_fence_regs_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "Reserved fences = %d\n", dev_priv->fence_reg_start);
	seq_printf(m, "Total fences = %d\n", dev_priv->num_fence_regs);
	for (i = 0; i < dev_priv->num_fence_regs; i++) {
		struct drm_i915_gem_object *obj = dev_priv->fence_regs[i].obj;

		seq_printf(m, "Fence %d, pin count = %d, object = ",
			   i, dev_priv->fence_regs[i].pin_count);
		if (obj == NULL)
			seq_puts(m, "unused");
		else
			describe_obj(m, obj);
		seq_putc(m, '\n');
	}

	mutex_unlock(&dev->struct_mutex);
	return 0;
}

static int i915_hws_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	const u32 *hws;
	int i;

	ring = &dev_priv->ring[(uintptr_t)node->info_ent->data];
	hws = ring->status_page.page_addr;
	if (hws == NULL)
		return 0;

	for (i = 0; i < 4096 / sizeof(u32) / 4; i += 4) {
		seq_printf(m, "0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			   i * 4,
			   hws[i], hws[i + 1], hws[i + 2], hws[i + 3]);
	}
	return 0;
}

static ssize_t
i915_error_state_write(struct file *filp,
		       const char __user *ubuf,
		       size_t cnt,
		       loff_t *ppos)
{
	struct i915_error_state_file_priv *error_priv = filp->private_data;
	struct drm_device *dev = error_priv->dev;
	int ret;

	DRM_DEBUG_DRIVER("Resetting error state\n");

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	i915_destroy_error_state(dev);
	mutex_unlock(&dev->struct_mutex);

	return cnt;
}

static int i915_error_state_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct i915_error_state_file_priv *error_priv;

	error_priv = kzalloc(sizeof(*error_priv), GFP_KERNEL);
	if (!error_priv)
		return -ENOMEM;

	error_priv->dev = dev;

	i915_error_state_get(dev, error_priv);

	file->private_data = error_priv;

	return 0;
}

static int i915_error_state_release(struct inode *inode, struct file *file)
{
	struct i915_error_state_file_priv *error_priv = file->private_data;

	i915_error_state_put(error_priv);
	kfree(error_priv);

	return 0;
}

static ssize_t i915_error_state_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *pos)
{
	struct i915_error_state_file_priv *error_priv = file->private_data;
	struct drm_i915_error_state_buf error_str;
	loff_t tmp_pos = 0;
	ssize_t ret_count = 0;
	int ret;

	ret = i915_error_state_buf_init(&error_str, count, *pos);
	if (ret)
		return ret;

	ret = i915_error_state_to_str(&error_str, error_priv);
	if (ret)
		goto out;

	ret_count = simple_read_from_buffer(userbuf, count, &tmp_pos,
					    error_str.buf,
					    error_str.bytes);

	if (ret_count < 0)
		ret = ret_count;
	else
		*pos = error_str.start + ret_count;
out:
	i915_error_state_buf_release(&error_str);
	return ret ?: ret_count;
}

static const struct file_operations i915_error_state_fops = {
	.owner = THIS_MODULE,
	.open = i915_error_state_open,
	.read = i915_error_state_read,
	.write = i915_error_state_write,
	.llseek = default_llseek,
	.release = i915_error_state_release,
};

static int
i915_next_seqno_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	*val = dev_priv->next_seqno;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int
i915_next_seqno_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = i915_gem_set_seqno(dev, val);
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_next_seqno_fops,
			i915_next_seqno_get, i915_next_seqno_set,
			"0x%llx\n");

static int i915_rstdby_delays(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 crstanddelay;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	crstanddelay = I915_READ16(CRSTANDVID);

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "w/ctx: %d, w/o ctx: %d\n", (crstanddelay >> 8) & 0x3f, (crstanddelay & 0x3f));

	return 0;
}

static int
i915_scheduler_priority_max_get(void *data, u64 *val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	*val = (u64) scheduler->priority_level_max;
	return 0;
}

static int
i915_scheduler_priority_max_set(void *data, u64 val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	scheduler->priority_level_max = (u32) val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_scheduler_priority_max_fops,
			i915_scheduler_priority_max_get,
			i915_scheduler_priority_max_set,
			"0x%llx\n");

static int
i915_scheduler_priority_bump_get(void *data, u64 *val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	*val = (u64) scheduler->priority_level_bump;
	return 0;
}

static int
i915_scheduler_priority_bump_set(void *data, u64 val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	scheduler->priority_level_bump = (u32) val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_scheduler_priority_bump_fops,
			i915_scheduler_priority_bump_get,
			i915_scheduler_priority_bump_set,
			"0x%llx\n");

static int
i915_scheduler_priority_preempt_get(void *data, u64 *val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	*val = (u64) scheduler->priority_level_preempt;
	return 0;
}

static int
i915_scheduler_priority_preempt_set(void *data, u64 val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	scheduler->priority_level_preempt = (u32) val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_scheduler_priority_preempt_fops,
			i915_scheduler_priority_preempt_get,
			i915_scheduler_priority_preempt_set,
			"0x%llx\n");

static int
i915_scheduler_min_flying_get(void *data, u64 *val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	*val = (u64) scheduler->min_flying;
	return 0;
}

static int
i915_scheduler_min_flying_set(void *data, u64 val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	scheduler->min_flying = (u32) val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_scheduler_min_flying_fops,
			i915_scheduler_min_flying_get,
			i915_scheduler_min_flying_set,
			"0x%llx\n");

static int
i915_scheduler_file_queue_max_get(void *data, u64 *val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	*val = (u64) scheduler->file_queue_max;
	return 0;
}

static int
i915_scheduler_file_queue_max_set(void *data, u64 val)
{
	struct drm_device       *dev       = data;
	struct drm_i915_private *dev_priv  = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;

	scheduler->file_queue_max = (u32) val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_scheduler_file_queue_max_fops,
			i915_scheduler_file_queue_max_get,
			i915_scheduler_file_queue_max_set,
			"0x%llx\n");

static int i915_frequency_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = 0;

	intel_runtime_pm_get(dev_priv);

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	if (IS_GEN5(dev)) {
		u16 rgvswctl = I915_READ16(MEMSWCTL);
		u16 rgvstat = I915_READ16(MEMSTAT_ILK);

		seq_printf(m, "Requested P-state: %d\n", (rgvswctl >> 8) & 0xf);
		seq_printf(m, "Requested VID: %d\n", rgvswctl & 0x3f);
		seq_printf(m, "Current VID: %d\n", (rgvstat & MEMSTAT_VID_MASK) >>
			   MEMSTAT_VID_SHIFT);
		seq_printf(m, "Current P-state: %d\n",
			   (rgvstat & MEMSTAT_PSTATE_MASK) >> MEMSTAT_PSTATE_SHIFT);
	} else if (IS_GEN6(dev) || (IS_GEN7(dev) && !IS_VALLEYVIEW(dev)) ||
		   IS_BROADWELL(dev)) {
		u32 gt_perf_status = I915_READ(GEN6_GT_PERF_STATUS);
		u32 rp_state_limits = I915_READ(GEN6_RP_STATE_LIMITS);
		u32 rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
		u32 rpmodectl, rpinclimit, rpdeclimit;
		u32 rpstat, cagf, reqf;
		u32 rpupei, rpcurup, rpprevup;
		u32 rpdownei, rpcurdown, rpprevdown;
		int max_freq;

		/* RPSTAT1 is in the GT power well */
		ret = mutex_lock_interruptible(&dev->struct_mutex);
		if (ret)
			goto out;

		gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

		reqf = I915_READ(GEN6_RPNSWREQ);
		reqf &= ~GEN6_TURBO_DISABLE;
		if (IS_HASWELL(dev) || IS_BROADWELL(dev))
			reqf >>= 24;
		else
			reqf >>= 25;
		reqf *= GT_FREQUENCY_MULTIPLIER;

		rpmodectl = I915_READ(GEN6_RP_CONTROL);
		rpinclimit = I915_READ(GEN6_RP_UP_THRESHOLD);
		rpdeclimit = I915_READ(GEN6_RP_DOWN_THRESHOLD);

		rpstat = I915_READ(GEN6_RPSTAT1);
		rpupei = I915_READ(GEN6_RP_CUR_UP_EI);
		rpcurup = I915_READ(GEN6_RP_CUR_UP);
		rpprevup = I915_READ(GEN6_RP_PREV_UP);
		rpdownei = I915_READ(GEN6_RP_CUR_DOWN_EI);
		rpcurdown = I915_READ(GEN6_RP_CUR_DOWN);
		rpprevdown = I915_READ(GEN6_RP_PREV_DOWN);
		if (IS_HASWELL(dev) || IS_BROADWELL(dev))
			cagf = (rpstat & HSW_CAGF_MASK) >> HSW_CAGF_SHIFT;
		else
			cagf = (rpstat & GEN6_CAGF_MASK) >> GEN6_CAGF_SHIFT;
		cagf *= GT_FREQUENCY_MULTIPLIER;

		gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);
		mutex_unlock(&dev->struct_mutex);

		seq_printf(m, "PM IER=0x%08x IMR=0x%08x ISR=0x%08x IIR=0x%08x, MASK=0x%08x\n",
			   I915_READ(GEN6_PMIER),
			   I915_READ(GEN6_PMIMR),
			   I915_READ(GEN6_PMISR),
			   I915_READ(GEN6_PMIIR),
			   I915_READ(GEN6_PMINTRMSK));
		seq_printf(m, "GT_PERF_STATUS: 0x%08x\n", gt_perf_status);
		seq_printf(m, "Render p-state ratio: %d\n",
			   (gt_perf_status & 0xff00) >> 8);
		seq_printf(m, "Render p-state VID: %d\n",
			   gt_perf_status & 0xff);
		seq_printf(m, "Render p-state limit: %d\n",
			   rp_state_limits & 0xff);
		seq_printf(m, "RPSTAT1: 0x%08x\n", rpstat);
		seq_printf(m, "RPMODECTL: 0x%08x\n", rpmodectl);
		seq_printf(m, "RPINCLIMIT: 0x%08x\n", rpinclimit);
		seq_printf(m, "RPDECLIMIT: 0x%08x\n", rpdeclimit);
		seq_printf(m, "RPNSWREQ: %dMHz\n", reqf);
		seq_printf(m, "CAGF: %dMHz\n", cagf);
		seq_printf(m, "RP CUR UP EI: %dus\n", rpupei &
			   GEN6_CURICONT_MASK);
		seq_printf(m, "RP CUR UP: %dus\n", rpcurup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV UP: %dus\n", rpprevup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP CUR DOWN EI: %dus\n", rpdownei &
			   GEN6_CURIAVG_MASK);
		seq_printf(m, "RP CUR DOWN: %dus\n", rpcurdown &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV DOWN: %dus\n", rpprevdown &
			   GEN6_CURBSYTAVG_MASK);

		max_freq = (rp_state_cap & 0xff0000) >> 16;
		seq_printf(m, "Lowest (RPN) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		max_freq = (rp_state_cap & 0xff00) >> 8;
		seq_printf(m, "Nominal (RP1) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		max_freq = rp_state_cap & 0xff;
		seq_printf(m, "Max non-overclocked (RP0) frequency: %dMHz\n",
			   max_freq * GT_FREQUENCY_MULTIPLIER);

		seq_printf(m, "Max overclocked frequency: %dMHz\n",
			   dev_priv->rps.max_freq * GT_FREQUENCY_MULTIPLIER);
	} else if (IS_VALLEYVIEW(dev)) {
		u32 freq_sts;

		mutex_lock(&dev_priv->rps.hw_lock);
		freq_sts = vlv_punit_read(dev_priv, PUNIT_REG_GPU_FREQ_STS);
		seq_printf(m, "PUNIT_REG_GPU_FREQ_STS: 0x%08x\n", freq_sts);
		seq_printf(m, "DDR freq: %d MHz\n", dev_priv->mem_freq);

		seq_printf(m, "max GPU freq: %d MHz\n",
			   vlv_gpu_freq(dev_priv, dev_priv->rps.max_freq));

		seq_printf(m, "min GPU freq: %d MHz\n",
			   vlv_gpu_freq(dev_priv, dev_priv->rps.min_freq));

		seq_printf(m, "efficient (RPe) frequency: %d MHz\n",
			   vlv_gpu_freq(dev_priv, dev_priv->rps.efficient_freq));

		seq_printf(m, "current GPU freq: %d MHz\n",
			   vlv_gpu_freq(dev_priv, (freq_sts >> 8) & 0xff));
		mutex_unlock(&dev_priv->rps.hw_lock);
	} else {
		seq_puts(m, "no P-state info available\n");
	}

out:
	intel_runtime_pm_put(dev_priv);
	return ret;
}

static int i915_delayfreq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 delayfreq;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	for (i = 0; i < 16; i++) {
		delayfreq = I915_READ(PXVFREQ_BASE + i * 4);
		seq_printf(m, "P%02dVIDFREQ: 0x%08x (VID: %d)\n", i, delayfreq,
			   (delayfreq & PXVFREQ_PX_MASK) >> PXVFREQ_PX_SHIFT);
	}

	intel_runtime_pm_put(dev_priv);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static inline int MAP_TO_MV(int map)
{
	return 1250 - (map * 25);
}

static int i915_inttoext_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 inttoext;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	for (i = 1; i <= 32; i++) {
		inttoext = I915_READ(INTTOEXT_BASE_ILK + i * 4);
		seq_printf(m, "INTTOEXT%02d: 0x%08x\n", i, inttoext);
	}

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int ironlake_drpc_info(struct seq_file *m)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rgvmodectl, rstdbyctl;
	u16 crstandvid;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	rgvmodectl = I915_READ(MEMMODECTL);
	rstdbyctl = I915_READ(RSTDBYCTL);
	crstandvid = I915_READ16(CRSTANDVID);

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "HD boost: %s\n", (rgvmodectl & MEMMODE_BOOST_EN) ?
		   "yes" : "no");
	seq_printf(m, "Boost freq: %d\n",
		   (rgvmodectl & MEMMODE_BOOST_FREQ_MASK) >>
		   MEMMODE_BOOST_FREQ_SHIFT);
	seq_printf(m, "HW control enabled: %s\n",
		   rgvmodectl & MEMMODE_HWIDLE_EN ? "yes" : "no");
	seq_printf(m, "SW control enabled: %s\n",
		   rgvmodectl & MEMMODE_SWMODE_EN ? "yes" : "no");
	seq_printf(m, "Gated voltage change: %s\n",
		   rgvmodectl & MEMMODE_RCLK_GATE ? "yes" : "no");
	seq_printf(m, "Starting frequency: P%d\n",
		   (rgvmodectl & MEMMODE_FSTART_MASK) >> MEMMODE_FSTART_SHIFT);
	seq_printf(m, "Max P-state: P%d\n",
		   (rgvmodectl & MEMMODE_FMAX_MASK) >> MEMMODE_FMAX_SHIFT);
	seq_printf(m, "Min P-state: P%d\n", (rgvmodectl & MEMMODE_FMIN_MASK));
	seq_printf(m, "RS1 VID: %d\n", (crstandvid & 0x3f));
	seq_printf(m, "RS2 VID: %d\n", ((crstandvid >> 8) & 0x3f));
	seq_printf(m, "Render standby enabled: %s\n",
		   (rstdbyctl & RCX_SW_EXIT) ? "no" : "yes");
	seq_puts(m, "Current RS state: ");
	switch (rstdbyctl & RSX_STATUS_MASK) {
	case RSX_STATUS_ON:
		seq_puts(m, "on\n");
		break;
	case RSX_STATUS_RC1:
		seq_puts(m, "RC1\n");
		break;
	case RSX_STATUS_RC1E:
		seq_puts(m, "RC1E\n");
		break;
	case RSX_STATUS_RS1:
		seq_puts(m, "RS1\n");
		break;
	case RSX_STATUS_RS2:
		seq_puts(m, "RS2 (RC6)\n");
		break;
	case RSX_STATUS_RS3:
		seq_puts(m, "RC3 (RC6+)\n");
		break;
	default:
		seq_puts(m, "unknown\n");
		break;
	}

	return 0;
}

static int vlv_drpc_info(struct seq_file *m)
{

	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rpmodectl1, rcctl1, rcstatus;
	unsigned fw_rendercount = 0, fw_mediacount = 0;

	intel_runtime_pm_get(dev_priv);

	rcstatus = I915_READ(VLV_GTLC_PW_STATUS);
	rpmodectl1 = I915_READ(GEN6_RP_CONTROL);
	rcctl1 = I915_READ(GEN6_RC_CONTROL);

	intel_runtime_pm_put(dev_priv);

	seq_printf(m, "Video Turbo Mode: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_MEDIA_TURBO));
	seq_printf(m, "Turbo enabled: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_ENABLE));
	seq_printf(m, "HW control enabled: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_ENABLE));
	seq_printf(m, "SW control enabled: %s\n",
		   yesno((rpmodectl1 & GEN6_RP_MEDIA_MODE_MASK) ==
			  GEN6_RP_MEDIA_SW_MODE));
	seq_printf(m, "RC6 Enabled: %s\n",
		   yesno(rcctl1 & (GEN7_RC_CTL_TO_MODE |
					GEN6_RC_CTL_EI_MODE(1))));
	seq_printf(m, "Render Power Well: %s\n",
				(rcstatus & VLV_GTLC_PW_RENDER_STATUS_MASK) ? "Up" : "Down");
	seq_printf(m, "Media Power Well: %s\n",
				(rcstatus & VLV_GTLC_PW_MEDIA_STATUS_MASK) ? "Up" : "Down");

	seq_printf(m, "Render RC6 residency since boot: %u\n",
		   I915_READ(VLV_GT_RENDER_RC6));
	seq_printf(m, "Media RC6 residency since boot: %u\n",
		   I915_READ(VLV_GT_MEDIA_RC6));

	spin_lock_irq(&dev_priv->uncore.lock);
	fw_rendercount = dev_priv->uncore.fw_rendercount;
	fw_mediacount = dev_priv->uncore.fw_mediacount;
	spin_unlock_irq(&dev_priv->uncore.lock);

	seq_printf(m, "Forcewake Render Count = %u\n", fw_rendercount);
	seq_printf(m, "Forcewake Media Count = %u\n", fw_mediacount);


	return 0;
}


static int gen6_drpc_info(struct seq_file *m)
{

	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rpmodectl1, gt_core_status, rcctl1, rc6vids = 0;
	unsigned forcewake_count;
	int count = 0, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	spin_lock_irq(&dev_priv->uncore.lock);
	forcewake_count = dev_priv->uncore.forcewake_count;
	spin_unlock_irq(&dev_priv->uncore.lock);

	if (forcewake_count) {
		seq_puts(m, "RC information inaccurate because somebody "
			    "holds a forcewake reference \n");
	} else {
		/* NB: we cannot use forcewake, else we read the wrong values */
		while (count++ < 50 && (I915_READ_NOTRACE(FORCEWAKE_ACK) & 1))
			udelay(10);
		seq_printf(m, "RC information accurate: %s\n", yesno(count < 51));
	}

	gt_core_status = readl(dev_priv->regs + GEN6_GT_CORE_STATUS);
	trace_i915_reg_rw(false, GEN6_GT_CORE_STATUS, gt_core_status, 4, true);

	rpmodectl1 = I915_READ(GEN6_RP_CONTROL);
	rcctl1 = I915_READ(GEN6_RC_CONTROL);
	mutex_unlock(&dev->struct_mutex);
	mutex_lock(&dev_priv->rps.hw_lock);
	sandybridge_pcode_read(dev_priv, GEN6_PCODE_READ_RC6VIDS, &rc6vids);
	mutex_unlock(&dev_priv->rps.hw_lock);

	intel_runtime_pm_put(dev_priv);

	seq_printf(m, "Video Turbo Mode: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_MEDIA_TURBO));
	seq_printf(m, "HW control enabled: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_ENABLE));
	seq_printf(m, "SW control enabled: %s\n",
		   yesno((rpmodectl1 & GEN6_RP_MEDIA_MODE_MASK) ==
			  GEN6_RP_MEDIA_SW_MODE));
	seq_printf(m, "RC1e Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC1e_ENABLE));
	seq_printf(m, "RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6_ENABLE));
	seq_printf(m, "Deep RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6p_ENABLE));
	seq_printf(m, "Deepest RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6pp_ENABLE));
	seq_puts(m, "Current RC state: ");
	switch (gt_core_status & GEN6_RCn_MASK) {
	case GEN6_RC0:
		if (gt_core_status & GEN6_CORE_CPD_STATE_MASK)
			seq_puts(m, "Core Power Down\n");
		else
			seq_puts(m, "on\n");
		break;
	case GEN6_RC3:
		seq_puts(m, "RC3\n");
		break;
	case GEN6_RC6:
		seq_puts(m, "RC6\n");
		break;
	case GEN6_RC7:
		seq_puts(m, "RC7\n");
		break;
	default:
		seq_puts(m, "Unknown\n");
		break;
	}

	seq_printf(m, "Core Power Down: %s\n",
		   yesno(gt_core_status & GEN6_CORE_CPD_STATE_MASK));

	/* Not exactly sure what this is */
	seq_printf(m, "RC6 \"Locked to RPn\" residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6_LOCKED));
	seq_printf(m, "RC6 residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6));
	seq_printf(m, "RC6+ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6p));
	seq_printf(m, "RC6++ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6pp));

	seq_printf(m, "RC6   voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 0) & 0xff)));
	seq_printf(m, "RC6+  voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 8) & 0xff)));
	seq_printf(m, "RC6++ voltage: %dmV\n",
		   GEN6_DECODE_RC6_VID(((rc6vids >> 16) & 0xff)));
	return 0;
}

static int i915_drpc_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;

	if (IS_VALLEYVIEW(dev))
		return vlv_drpc_info(m);
	else if (IS_GEN6(dev) || IS_GEN7(dev))
		return gen6_drpc_info(m);
	else
		return ironlake_drpc_info(m);
}

static int i915_fbc_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_FBC(dev)) {
		seq_puts(m, "FBC unsupported on this chipset\n");
		return 0;
	}

	intel_runtime_pm_get(dev_priv);

	if (intel_fbc_enabled(dev)) {
		seq_puts(m, "FBC enabled\n");
	} else {
		seq_puts(m, "FBC disabled: ");
		switch (dev_priv->fbc.no_fbc_reason) {
		case FBC_OK:
			seq_puts(m, "FBC actived, but currently disabled in hardware");
			break;
		case FBC_UNSUPPORTED:
			seq_puts(m, "unsupported by this chipset");
			break;
		case FBC_NO_OUTPUT:
			seq_puts(m, "no outputs");
			break;
		case FBC_STOLEN_TOO_SMALL:
			seq_puts(m, "not enough stolen memory");
			break;
		case FBC_UNSUPPORTED_MODE:
			seq_puts(m, "mode not supported");
			break;
		case FBC_MODE_TOO_LARGE:
			seq_puts(m, "mode too large");
			break;
		case FBC_BAD_PLANE:
			seq_puts(m, "FBC unsupported on plane");
			break;
		case FBC_NOT_TILED:
			seq_puts(m, "scanout buffer not tiled");
			break;
		case FBC_MULTIPLE_PIPES:
			seq_puts(m, "multiple pipes are enabled");
			break;
		case FBC_MODULE_PARAM:
			seq_puts(m, "disabled per module param (default off)");
			break;
		case FBC_CHIP_DEFAULT:
			seq_puts(m, "disabled per chip default");
			break;
		case FBC_DEBUG_FS:
			seq_puts(m, "disabled per debugfs");
			break;
		default:
			seq_puts(m, "unknown reason");
		}
		seq_putc(m, '\n');
	}

	intel_runtime_pm_put(dev_priv);

	return 0;
}

static int i915_fbc_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_FBC(dev))
		return -ENODEV;

	*val = dev_priv->fbc.disable;

	return 0;
}

static int i915_fbc_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	if (!HAS_FBC(dev))
		return -ENODEV;

	if (dev_priv->fbc.disable == (bool)val)
		return 0;

	drm_modeset_lock_all(dev);

	DRM_DEBUG_DRIVER("Setting FBC disable %s\n",
			 val ? "true" : "false");

	dev_priv->fbc.disable = (bool)val;

	/* Reset enabled crtc to force FBC state update */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
		if (crtc->enabled)
			intel_crtc_restore_mode(crtc);

	drm_modeset_unlock_all(dev);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_fbc_disable_fops,
			i915_fbc_disable_get, i915_fbc_disable_set,
			"%llu\n");

static int i915_ips_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_IPS(dev)) {
		seq_puts(m, "not supported\n");
		return 0;
	}

	intel_runtime_pm_get(dev_priv);

	if (IS_BROADWELL(dev) || I915_READ(IPS_CTL) & IPS_ENABLE)
		seq_puts(m, "enabled\n");
	else
		seq_puts(m, "disabled\n");

	intel_runtime_pm_put(dev_priv);

	return 0;
}

static int i915_ips_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_IPS(dev))
		return -ENODEV;

	*val = dev_priv->ips_disable;

	return 0;
}

static int i915_ips_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	if (!HAS_IPS(dev))
		return -ENODEV;

	if (dev_priv->ips_disable == (bool)val)
		return 0;

	drm_modeset_lock_all(dev);

	DRM_DEBUG_DRIVER("Setting IPS disable %s\n",
			 val ? "true" : "false");

	dev_priv->ips_disable = (bool)val;

	/* Reset enabled crtc to force IPS state update */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
		if (crtc->enabled)
			intel_crtc_restore_mode(crtc);

	drm_modeset_unlock_all(dev);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_ips_disable_fops,
			i915_ips_disable_get, i915_ips_disable_set,
			"%llu\n");

static int i915_sr_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool sr_enabled = false;

	intel_runtime_pm_get(dev_priv);

	if (HAS_PCH_SPLIT(dev))
		sr_enabled = I915_READ(WM1_LP_ILK) & WM1_LP_SR_EN;
	else if (IS_CRESTLINE(dev) || IS_I945G(dev) || IS_I945GM(dev))
		sr_enabled = I915_READ(FW_BLC_SELF) & FW_BLC_SELF_EN;
	else if (IS_I915GM(dev))
		sr_enabled = I915_READ(INSTPM) & INSTPM_SELF_EN;
	else if (IS_PINEVIEW(dev))
		sr_enabled = I915_READ(DSPFW3) & PINEVIEW_SELF_REFRESH_EN;
	else if (IS_VALLEYVIEW(dev))
		sr_enabled = I915_READ(FW_BLC_SELF_VLV) & FW_CSPWRDWNEN;

	intel_runtime_pm_put(dev_priv);

	seq_printf(m, "self-refresh: %s\n",
		   sr_enabled ? "enabled" : "disabled");

	return 0;
}

static int i915_sr_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = dev_priv->sr_disable;

	return 0;
}

static int i915_sr_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	if (dev_priv->sr_disable == (bool)val)
		return 0;

	drm_modeset_lock_all(dev);

	DRM_DEBUG_DRIVER("Setting CxSR disable %s\n",
			 val ? "true" : "false");

	dev_priv->sr_disable = (bool)val;

	/* Reset enabled crtc to force CxSR state update */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
		if (crtc->enabled)
			intel_crtc_restore_mode(crtc);

	drm_modeset_unlock_all(dev);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_sr_disable_fops,
			i915_sr_disable_get, i915_sr_disable_set,
			"%llu\n");

static int i915_dpst_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;
	const int columns = 4;
	u32 blm_hist_ctl, blm_hist_guard;

	if (!I915_HAS_DPST(dev)) {
		seq_puts(m, "not supported\n");
		return 0;
	}

	/* Check if dpst is initialized by UMD? */
	if (!dev_priv->dpst.reg.blm_hist_ctl) {
		seq_puts(m, "DPST feature not enabled\n");
		return 0;
	}

	mutex_lock(&dev_priv->dpst.ioctl_lock);

	intel_display_power_get(dev_priv, PIPE_A);
	blm_hist_ctl = I915_READ(dev_priv->dpst.reg.blm_hist_ctl);
	blm_hist_guard = I915_READ(dev_priv->dpst.reg.blm_hist_guard);

	seq_printf(m, "histogram logic: %s\n",
		   blm_hist_ctl & IE_HISTOGRAM_ENABLE ?
		   "enabled" : "disabled");

	seq_printf(m, "histogram interrupts: %s\n",
		   blm_hist_guard & HISTOGRAM_INTERRUPT_ENABLE ?
		   "enabled" : "disabled");

	seq_printf(m, "backlight adjustment: %u%%\n",
		   dev_priv->dpst.blc_adjustment * 100 / DPST_MAX_FACTOR);

	seq_printf(m, "IE modification table: %s\n",
		   blm_hist_ctl & dev_priv->dpst.ie_mod_table_enable ?
		   "enabled" : "disabled");

	blm_hist_ctl |= BIN_REG_FUNCTION_SELECT_IE;
	blm_hist_ctl &= ~BIN_REGISTER_INDEX_MASK;
	I915_WRITE(dev_priv->dpst.reg.blm_hist_ctl, blm_hist_ctl);

	seq_puts(m, "IE modification table values...");
	for (i = 0; i < DPST_DIET_ENTRY_COUNT; i++) {
		if (i % columns == 0)
			seq_printf(m, "\nbins %02d-%02d:", i, i + columns - 1);
		seq_printf(m, "%10x", I915_READ(dev_priv->dpst.reg.blm_hist_bin));
	}
	seq_puts(m, "\n");
	intel_display_power_put(dev_priv, PIPE_A);

	mutex_unlock(&dev_priv->dpst.ioctl_lock);

	return 0;
}

static int i915_dpst_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = !dev_priv->dpst.enabled;

	return 0;
}

static int i915_dpst_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;

	DRM_DEBUG_DRIVER("Setting DPST disable %s\n",
			 val ? "true" : "false");

	return i915_dpst_set_kernel_disable(dev, val);
}

DEFINE_SIMPLE_ATTRIBUTE(i915_dpst_disable_fops,
			i915_dpst_disable_get, i915_dpst_disable_set,
			"%llu\n");

static int i915_emon_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long temp, chipset, gfx;
	int ret;

	if (!IS_GEN5(dev))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	temp = i915_mch_val(dev_priv);
	chipset = i915_chipset_val(dev_priv);
	gfx = i915_gfx_val(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "GMCH temp: %ld\n", temp);
	seq_printf(m, "Chipset power: %ld\n", chipset);
	seq_printf(m, "GFX power: %ld\n", gfx);
	seq_printf(m, "Total power: %ld\n", chipset + gfx);

	return 0;
}

static int i915_ring_freq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = 0;
	int gpu_freq, ia_freq;

	if (!(IS_GEN6(dev) || IS_GEN7(dev))) {
		seq_puts(m, "unsupported on this chipset\n");
		return 0;
	}

	intel_runtime_pm_get(dev_priv);

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		goto out;

	seq_puts(m, "GPU freq (MHz)\tEffective CPU freq (MHz)\tEffective Ring freq (MHz)\n");

	for (gpu_freq = dev_priv->rps.min_freq_softlimit;
	     gpu_freq <= dev_priv->rps.max_freq_softlimit;
	     gpu_freq++) {
		ia_freq = gpu_freq;
		sandybridge_pcode_read(dev_priv,
				       GEN6_PCODE_READ_MIN_FREQ_TABLE,
				       &ia_freq);
		seq_printf(m, "%d\t\t%d\t\t\t\t%d\n",
			   gpu_freq * GT_FREQUENCY_MULTIPLIER,
			   ((ia_freq >> 0) & 0xff) * 100,
			   ((ia_freq >> 8) & 0xff) * 100);
	}

	mutex_unlock(&dev_priv->rps.hw_lock);

out:
	intel_runtime_pm_put(dev_priv);
	return ret;
}

static int i915_gfxec(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	seq_printf(m, "GFXEC: %ld\n", (unsigned long)I915_READ(0x112f4));
	intel_runtime_pm_put(dev_priv);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_opregion(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_opregion *opregion = &dev_priv->opregion;
	void *data = kmalloc(OPREGION_SIZE, GFP_KERNEL);
	int ret;

	if (data == NULL)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		goto out;

	if (opregion->header) {
		memcpy_fromio(data, opregion->header, OPREGION_SIZE);
		seq_write(m, data, OPREGION_SIZE);
	}

	mutex_unlock(&dev->struct_mutex);

out:
	kfree(data);
	return 0;
}

static int i915_gem_framebuffer_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct intel_fbdev *ifbdev = NULL;
	struct intel_framebuffer *fb;

#ifdef CONFIG_DRM_I915_FBDEV
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	ifbdev = dev_priv->fbdev;
	if (!ifbdev) {
		mutex_unlock(&dev->mode_config.mutex);
		return -ENOMEM;
	}

	fb = to_intel_framebuffer(ifbdev->helper.fb);

	seq_printf(m, "fbcon size: %d x %d, depth %d, %d bpp, refcount %d, obj ",
		   fb->base.width,
		   fb->base.height,
		   fb->base.depth,
		   fb->base.bits_per_pixel,
		   atomic_read(&fb->base.refcount.refcount));
	describe_obj(m, fb->obj);
	seq_putc(m, '\n');
	mutex_unlock(&dev->mode_config.mutex);
#endif

	mutex_lock(&dev->mode_config.fb_lock);
	list_for_each_entry(fb, &dev->mode_config.fb_list, base.head) {
		if (ifbdev && &fb->base == ifbdev->helper.fb)
			continue;

		seq_printf(m, "user size: %d x %d, depth %d, %d bpp, refcount %d, obj ",
			   fb->base.width,
			   fb->base.height,
			   fb->base.depth,
			   fb->base.bits_per_pixel,
			   atomic_read(&fb->base.refcount.refcount));
		describe_obj(m, fb->obj);
		seq_putc(m, '\n');
	}
	mutex_unlock(&dev->mode_config.fb_lock);

	return 0;
}

static void describe_ctx_ringbuf(struct seq_file *m,
				 struct intel_ringbuffer *ringbuf)
{
	seq_printf(m, " (ringbuffer, space: %d, head: %u, tail: %u, last head: %d)",
		   ringbuf->space, ringbuf->head, ringbuf->tail,
		   ringbuf->last_retired_head);
}

static int i915_context_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	struct intel_context *ctx;
	uint32_t count = 0;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	if (dev_priv->ips.pwrctx) {
		seq_puts(m, "power context ");
		describe_obj(m, dev_priv->ips.pwrctx);
		seq_putc(m, '\n');
		count++;
	}

	if (dev_priv->ips.renderctx) {
		seq_puts(m, "render context ");
		describe_obj(m, dev_priv->ips.renderctx);
		seq_putc(m, '\n');
		count++;
	}

	list_for_each_entry(ctx, &dev_priv->context_list, link) {
		if (!i915.enable_execlists &&
		    ctx->legacy_hw_ctx.rcs_state == NULL)
			continue;

		seq_puts(m, "HW context ");
		describe_ctx(m, ctx);
		count++;
		for_each_ring(ring, dev_priv, i) {
			if (ring->default_context == ctx)
				seq_printf(m, "(default context %s) ",
					   ring->name);
		}

		if (i915.enable_execlists) {
			seq_putc(m, '\n');
			for_each_ring(ring, dev_priv, i) {
				struct drm_i915_gem_object *ctx_obj =
					ctx->engine[i].state;
				struct intel_ringbuffer *ringbuf =
					ctx->engine[i].ringbuf;

				seq_printf(m, "%s: ", ring->name);
				if (ctx_obj)
					describe_obj(m, ctx_obj);
				if (ringbuf)
					describe_ctx_ringbuf(m, ringbuf);
				seq_putc(m, '\n');
			}
		} else {
			describe_obj(m, ctx->legacy_hw_ctx.rcs_state);
		}

		seq_putc(m, '\n');
	}

	seq_printf(m, "Total: %d contexts\n", count);

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static void i915_dump_lrc_obj(struct seq_file *m,
			      struct intel_engine_cs *ring,
			      struct drm_i915_gem_object *ctx_obj)
{
	struct page *page;
	uint32_t *reg_state;
	int j;
	unsigned long ggtt_offset = 0;

	if (ctx_obj == NULL) {
		seq_printf(m, "Context on %s with no gem object\n",
			   ring->name);
		return;
	}

	seq_printf(m, "CONTEXT: %s %u\n", ring->name,
		   intel_execlists_ctx_id(ctx_obj));

	if (!i915_gem_obj_ggtt_bound(ctx_obj))
		seq_puts(m, "\tNot bound in GGTT\n");
	else
		ggtt_offset = i915_gem_obj_ggtt_offset(ctx_obj);

	if (i915_gem_object_get_pages(ctx_obj)) {
		seq_puts(m, "\tFailed to get pages for context object\n");
		return;
	}

	page = i915_gem_object_get_page(ctx_obj, 1);
	if (!WARN_ON(page == NULL)) {
		reg_state = kmap_atomic(page);

		for (j = 0; j < 0x600 / sizeof(u32) / 4; j += 4) {
			seq_printf(m, "\t[0x%08lx] 0x%08x 0x%08x 0x%08x 0x%08x\n",
				   ggtt_offset + 4096 + (j * 4),
				   reg_state[j], reg_state[j + 1],
				   reg_state[j + 2], reg_state[j + 3]);
		}
		kunmap_atomic(reg_state);
	}

	seq_putc(m, '\n');
}

static int i915_dump_lrc(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	struct intel_context *ctx;
	int ret, i;

	if (!i915.enable_execlists) {
		seq_printf(m, "Logical Ring Contexts are disabled\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	list_for_each_entry(ctx, &dev_priv->context_list, link) {
		for_each_ring(ring, dev_priv, i) {
			if (ring->default_context != ctx)
				i915_dump_lrc_obj(m, ring,
						  ctx->engine[i].state);
		}
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_execlists(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	u32 status_pointer;
	u8 read_pointer;
	u8 write_pointer;
	u32 status;
	u32 ctx_id;
	struct list_head *cursor;
	int ring_id, i;
	int ret;

	if (!i915.enable_execlists) {
		seq_puts(m, "Logical Ring Contexts are disabled\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for_each_ring(ring, dev_priv, ring_id) {
		struct intel_ctx_submit_request *head_req = NULL;
		int count = 0;
		unsigned long flags;

		seq_printf(m, "%s\n", ring->name);

		status = I915_READ(RING_EXECLIST_STATUS(ring));
		ctx_id = I915_READ(RING_EXECLIST_STATUS_CTX_ID(ring));
		seq_printf(m, "\tExeclist status: 0x%08X, context: %u\n",
			   status, ctx_id);

		status_pointer = I915_READ(RING_CONTEXT_STATUS_PTR(ring));
		seq_printf(m, "\tStatus pointer: 0x%08X\n", status_pointer);

		read_pointer = ring->next_context_status_buffer;
		write_pointer = status_pointer & 0x07;
		if (read_pointer > write_pointer)
			write_pointer += 6;
		seq_printf(m, "\tRead pointer: 0x%08X, write pointer 0x%08X\n",
			   read_pointer, write_pointer);

		for (i = 0; i < 6; i++) {
			status = I915_READ(RING_CONTEXT_STATUS_BUF(ring) + 8*i);
			ctx_id = I915_READ(RING_CONTEXT_STATUS_BUF(ring) + 8*i + 4);

			seq_printf(m, "\tStatus buffer %d: 0x%08X, context: %u\n",
				   i, status, ctx_id);
		}

		spin_lock_irqsave(&ring->execlist_lock, flags);
		list_for_each(cursor, &ring->execlist_queue)
			count++;
		head_req = list_first_entry_or_null(&ring->execlist_queue,
				struct intel_ctx_submit_request, execlist_link);
		spin_unlock_irqrestore(&ring->execlist_lock, flags);

		seq_printf(m, "\t%d requests in queue\n", count);
		if (head_req) {
			struct drm_i915_gem_object *ctx_obj;

			ctx_obj = head_req->ctx->engine[ring_id].state;
			seq_printf(m, "\tHead request id: %u\n",
				   intel_execlists_ctx_id(ctx_obj));
			seq_printf(m, "\tHead request tail: %u\n",
				   head_req->tail);
		}

		seq_putc(m, '\n');
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gen6_forcewake_count_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned forcewake_count = 0, fw_rendercount = 0, fw_mediacount = 0;

	spin_lock_irq(&dev_priv->uncore.lock);
	if (IS_VALLEYVIEW(dev)) {
		fw_rendercount = dev_priv->uncore.fw_rendercount;
		fw_mediacount = dev_priv->uncore.fw_mediacount;
	} else
		forcewake_count = dev_priv->uncore.forcewake_count;
	spin_unlock_irq(&dev_priv->uncore.lock);

	if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "fw_rendercount = %u\n", fw_rendercount);
		seq_printf(m, "fw_mediacount = %u\n", fw_mediacount);
	} else
		seq_printf(m, "forcewake count = %u\n", forcewake_count);

	return 0;
}

static const char *swizzle_string(unsigned swizzle)
{
	switch (swizzle) {
	case I915_BIT_6_SWIZZLE_NONE:
		return "none";
	case I915_BIT_6_SWIZZLE_9:
		return "bit9";
	case I915_BIT_6_SWIZZLE_9_10:
		return "bit9/bit10";
	case I915_BIT_6_SWIZZLE_9_11:
		return "bit9/bit11";
	case I915_BIT_6_SWIZZLE_9_10_11:
		return "bit9/bit10/bit11";
	case I915_BIT_6_SWIZZLE_9_17:
		return "bit9/bit17";
	case I915_BIT_6_SWIZZLE_9_10_17:
		return "bit9/bit10/bit17";
	case I915_BIT_6_SWIZZLE_UNKNOWN:
		return "unknown";
	}

	return "bug";
}

static int i915_swizzle_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	seq_printf(m, "bit6 swizzle for X-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_x));
	seq_printf(m, "bit6 swizzle for Y-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_y));

	if (IS_GEN3(dev) || IS_GEN4(dev)) {
		seq_printf(m, "DDC = 0x%08x\n",
			   I915_READ(DCC));
		seq_printf(m, "C0DRB3 = 0x%04x\n",
			   I915_READ16(C0DRB3));
		seq_printf(m, "C1DRB3 = 0x%04x\n",
			   I915_READ16(C1DRB3));
	} else if (INTEL_INFO(dev)->gen >= 6) {
		seq_printf(m, "MAD_DIMM_C0 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C0));
		seq_printf(m, "MAD_DIMM_C1 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C1));
		seq_printf(m, "MAD_DIMM_C2 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C2));
		seq_printf(m, "TILECTL = 0x%08x\n",
			   I915_READ(TILECTL));
		if (IS_GEN8(dev))
			seq_printf(m, "GAMTARBMODE = 0x%08x\n",
				   I915_READ(GAMTARBMODE));
		else
			seq_printf(m, "ARB_MODE = 0x%08x\n",
				   I915_READ(ARB_MODE));
		seq_printf(m, "DISP_ARB_CTL = 0x%08x\n",
			   I915_READ(DISP_ARB_CTL));
	}
	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int per_file_ctx(int id, void *ptr, void *data)
{
	struct intel_context *ctx = ptr;
	struct seq_file *m = data;
	struct i915_hw_ppgtt *ppgtt = ctx->ppgtt;

	if (!ppgtt) {
		seq_printf(m, "  no ppgtt for context %d\n",
			   ctx->user_handle);
		return 0;
	}

	if (i915_gem_context_is_default(ctx))
		seq_puts(m, "  default context:\n");
	else
		seq_printf(m, "  context %d:\n", ctx->user_handle);
	ppgtt->debug_dump(ppgtt, m);

	return 0;
}

static void gen8_ppgtt_info(struct seq_file *m, struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	struct i915_hw_ppgtt *ppgtt = dev_priv->mm.aliasing_ppgtt;
	struct i915_address_space *vm;
	int unused, i;


	if (ppgtt) {
		for_each_ring(ring, dev_priv, unused) {
			seq_printf(m, "%s\n", ring->name);
			for (i = 0; i < 4; i++) {
				u32 offset = 0x270 + i * 8;
				u64 pdp = I915_READ(ring->mmio_base + offset + 4);
				pdp <<= 32;
				pdp |= I915_READ(ring->mmio_base + offset);
				seq_printf(m, "\tPDP%d 0x%016llx\n", i, pdp);
			}
		}
	} else {
		i = 0;
		list_for_each_entry(vm, &dev_priv->vm_list, global_link) {
			if (i915_is_ggtt(vm))
				continue;
			i++;
			ppgtt = i915_vm_to_ppgtt(vm);
			seq_printf(m, "PPGTT %p - references\n", ppgtt);
		}
		seq_printf(m, "Number of PPGTTs active: %d\n", i);
	}
}

static void gen6_ppgtt_info(struct seq_file *m, struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	struct drm_file *file;
	int i;

	if (INTEL_INFO(dev)->gen == 6)
		seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(GFX_MODE));

	for_each_ring(ring, dev_priv, i) {
		seq_printf(m, "%s\n", ring->name);
		if (INTEL_INFO(dev)->gen == 7)
			seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(RING_MODE_GEN7(ring)));
		seq_printf(m, "PP_DIR_BASE: 0x%08x\n", I915_READ(RING_PP_DIR_BASE(ring)));
		seq_printf(m, "PP_DIR_BASE_READ: 0x%08x\n", I915_READ(RING_PP_DIR_BASE_READ(ring)));
		seq_printf(m, "PP_DIR_DCLV: 0x%08x\n", I915_READ(RING_PP_DIR_DCLV(ring)));
	}
	seq_printf(m, "ECOCHK: 0x%08x\n\n", I915_READ(GAM_ECOCHK));

	if (dev_priv->mm.aliasing_ppgtt) {
		struct i915_hw_ppgtt *ppgtt = dev_priv->mm.aliasing_ppgtt;

		seq_puts(m, "aliasing PPGTT:\n");
		seq_printf(m, "pd gtt offset: 0x%08x\n", ppgtt->pd.pd_offset);

		ppgtt->debug_dump(ppgtt, m);
	}

	list_for_each_entry_reverse(file, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv = file->driver_priv;

		seq_printf(m, "proc: %s\n",
			   get_pid_task(file->pid, PIDTYPE_PID)->comm);
		idr_for_each(&file_priv->context_idr, per_file_ctx, m);
	}
}

static int i915_ppgtt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	int ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	if (INTEL_INFO(dev)->gen >= 8)
		gen8_ppgtt_info(m, dev);
	else if (INTEL_INFO(dev)->gen >= 6)
		gen6_ppgtt_info(m, dev);

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_llc(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Size calculation for LLC is a bit of a pain. Ignore for now. */
	seq_printf(m, "LLC: %s\n", yesno(HAS_LLC(dev)));
	seq_printf(m, "eLLC: %zuMB\n", dev_priv->ellc_size);

	return 0;
}

static int i915_edp_psr_status(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 psrperf = 0;
	bool enabled = false;

	intel_runtime_pm_get(dev_priv);

	seq_printf(m, "Sink_Support: %s\n", yesno(dev_priv->psr.sink_support));
	seq_printf(m, "Source_OK: %s\n", yesno(dev_priv->psr.source_ok));

	enabled = HAS_PSR(dev) &&
		I915_READ(EDP_PSR_CTL(dev)) & EDP_PSR_ENABLE;
	seq_printf(m, "Enabled: %s\n", yesno(enabled));

	if (HAS_PSR(dev))
		psrperf = I915_READ(EDP_PSR_PERF_CNT(dev)) &
			EDP_PSR_PERF_CNT_MASK;
	seq_printf(m, "Performance_Counter: %u\n", psrperf);

	intel_runtime_pm_put(dev_priv);
	return 0;
}

static int i915_sink_crc(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct intel_encoder *encoder;
	struct intel_connector *connector;
	struct intel_dp *intel_dp = NULL;
	int ret;
	u8 crc[6];

	drm_modeset_lock_all(dev);
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    base.head) {

		if (connector->base.dpms != DRM_MODE_DPMS_ON)
			continue;

		if (!connector->base.encoder)
			continue;

		encoder = to_intel_encoder(connector->base.encoder);
		if (encoder->type != INTEL_OUTPUT_EDP)
			continue;

		intel_dp = enc_to_intel_dp(&encoder->base);

		ret = intel_dp_sink_crc(intel_dp, crc);
		if (ret)
			goto out;

		seq_printf(m, "%02x%02x%02x%02x%02x%02x\n",
			   crc[0], crc[1], crc[2],
			   crc[3], crc[4], crc[5]);
		goto out;
	}
	ret = -ENODEV;
out:
	drm_modeset_unlock_all(dev);
	return ret;
}

static int i915_energy_uJ(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u64 power;
	u32 units;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	intel_runtime_pm_get(dev_priv);

	rdmsrl(MSR_RAPL_POWER_UNIT, power);
	power = (power & 0x1f00) >> 8;
	units = 1000000 / (1 << power); /* convert to uJ */
	power = I915_READ(MCH_SECP_NRG_STTS);
	power *= units;

	intel_runtime_pm_put(dev_priv);

	seq_printf(m, "%llu", (long long unsigned)power);

	return 0;
}

static int i915_pc8_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_HASWELL(dev) && !IS_BROADWELL(dev)) {
		seq_puts(m, "not supported\n");
		return 0;
	}

	seq_printf(m, "GPU idle: %s\n", yesno(!dev_priv->mm.busy));
	seq_printf(m, "IRQs disabled: %s\n",
		   yesno(dev_priv->pm.irqs_disabled));

	return 0;
}

static const char *power_domain_str(enum intel_display_power_domain domain)
{
	switch (domain) {
	case POWER_DOMAIN_PIPE_A:
		return "PIPE_A";
	case POWER_DOMAIN_PIPE_B:
		return "PIPE_B";
	case POWER_DOMAIN_PIPE_C:
		return "PIPE_C";
	case POWER_DOMAIN_PIPE_A_PANEL_FITTER:
		return "PIPE_A_PANEL_FITTER";
	case POWER_DOMAIN_PIPE_B_PANEL_FITTER:
		return "PIPE_B_PANEL_FITTER";
	case POWER_DOMAIN_PIPE_C_PANEL_FITTER:
		return "PIPE_C_PANEL_FITTER";
	case POWER_DOMAIN_TRANSCODER_A:
		return "TRANSCODER_A";
	case POWER_DOMAIN_TRANSCODER_B:
		return "TRANSCODER_B";
	case POWER_DOMAIN_TRANSCODER_C:
		return "TRANSCODER_C";
	case POWER_DOMAIN_TRANSCODER_EDP:
		return "TRANSCODER_EDP";
	case POWER_DOMAIN_PORT_DDI_A_2_LANES:
		return "PORT_DDI_A_2_LANES";
	case POWER_DOMAIN_PORT_DDI_A_4_LANES:
		return "PORT_DDI_A_4_LANES";
	case POWER_DOMAIN_PORT_DDI_B_2_LANES:
		return "PORT_DDI_B_2_LANES";
	case POWER_DOMAIN_PORT_DDI_B_4_LANES:
		return "PORT_DDI_B_4_LANES";
	case POWER_DOMAIN_PORT_DDI_C_2_LANES:
		return "PORT_DDI_C_2_LANES";
	case POWER_DOMAIN_PORT_DDI_C_4_LANES:
		return "PORT_DDI_C_4_LANES";
	case POWER_DOMAIN_PORT_DDI_D_2_LANES:
		return "PORT_DDI_D_2_LANES";
	case POWER_DOMAIN_PORT_DDI_D_4_LANES:
		return "PORT_DDI_D_4_LANES";
	case POWER_DOMAIN_PORT_DSI:
		return "PORT_DSI";
	case POWER_DOMAIN_PORT_CRT:
		return "PORT_CRT";
	case POWER_DOMAIN_PORT_OTHER:
		return "PORT_OTHER";
	case POWER_DOMAIN_VGA:
		return "VGA";
	case POWER_DOMAIN_AUDIO:
		return "AUDIO";
	case POWER_DOMAIN_INIT:
		return "INIT";
	default:
		WARN_ON(1);
		return "?";
	}
}

static int i915_power_domain_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_power_domains *power_domains = &dev_priv->power_domains;
	int i;

	mutex_lock(&power_domains->lock);

	seq_printf(m, "%-25s %s\n", "Power well/domain", "Use count");
	for (i = 0; i < power_domains->power_well_count; i++) {
		struct i915_power_well *power_well;
		enum intel_display_power_domain power_domain;

		power_well = &power_domains->power_wells[i];
		seq_printf(m, "%-25s %d\n", power_well->name,
			   power_well->count);

		for (power_domain = 0; power_domain < POWER_DOMAIN_NUM;
		     power_domain++) {
			if (!(BIT(power_domain) & power_well->domains))
				continue;

			seq_printf(m, "  %-23s %d\n",
				 power_domain_str(power_domain),
				 power_domains->domain_use_count[power_domain]);
		}
	}

	mutex_unlock(&power_domains->lock);

	return 0;
}

static void intel_seq_print_mode(struct seq_file *m, int tabs,
				 struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < tabs; i++)
		seq_putc(m, '\t');

	seq_printf(m, "id %d:\"%s\" freq %d clock %d hdisp %d hss %d hse %d htot %d vdisp %d vss %d vse %d vtot %d type 0x%x flags 0x%x\n",
		   mode->base.id, mode->name,
		   mode->vrefresh, mode->clock,
		   mode->hdisplay, mode->hsync_start,
		   mode->hsync_end, mode->htotal,
		   mode->vdisplay, mode->vsync_start,
		   mode->vsync_end, mode->vtotal,
		   mode->type, mode->flags);
}

static void intel_encoder_info(struct seq_file *m,
			       struct intel_crtc *intel_crtc,
			       struct intel_encoder *intel_encoder)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_crtc *crtc = &intel_crtc->base;
	struct intel_connector *intel_connector;
	struct drm_encoder *encoder;

	encoder = &intel_encoder->base;
	seq_printf(m, "\tencoder %d: type: %s, connectors:\n",
		   encoder->base.id, encoder->name);
	for_each_connector_on_encoder(dev, encoder, intel_connector) {
		struct drm_connector *connector = &intel_connector->base;
		seq_printf(m, "\t\tconnector %d: type: %s, status: %s",
			   connector->base.id,
			   connector->name,
			   drm_get_connector_status_name(connector->status));
		if (connector->status == connector_status_connected) {
			struct drm_display_mode *mode = &crtc->mode;
			seq_printf(m, ", mode:\n");
			intel_seq_print_mode(m, 2, mode);
		} else {
			seq_putc(m, '\n');
		}
	}
}

static void intel_crtc_info(struct seq_file *m, struct intel_crtc *intel_crtc)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_crtc *crtc = &intel_crtc->base;
	struct intel_encoder *intel_encoder;

	seq_printf(m, "\tfb: %d, pos: %dx%d, size: %dx%d\n",
		   crtc->primary->fb->base.id, crtc->x, crtc->y,
		   crtc->primary->fb->width, crtc->primary->fb->height);
	for_each_encoder_on_crtc(dev, crtc, intel_encoder)
		intel_encoder_info(m, intel_crtc, intel_encoder);
}

static void intel_panel_info(struct seq_file *m, struct intel_panel *panel)
{
	struct drm_display_mode *mode = panel->fixed_mode;

	seq_printf(m, "\tfixed mode:\n");
	intel_seq_print_mode(m, 2, mode);
}

static void intel_dp_info(struct seq_file *m,
			  struct intel_connector *intel_connector)
{
	struct intel_encoder *intel_encoder = intel_connector->encoder;
	struct intel_dp *intel_dp = enc_to_intel_dp(&intel_encoder->base);

	seq_printf(m, "\tDPCD rev: %x\n", intel_dp->dpcd[DP_DPCD_REV]);
	seq_printf(m, "\taudio support: %s\n", intel_dp->has_audio ? "yes" :
		   "no");
	if (intel_encoder->type == INTEL_OUTPUT_EDP)
		intel_panel_info(m, &intel_connector->panel);
}

static void intel_hdmi_info(struct seq_file *m,
			    struct intel_connector *intel_connector)
{
	struct intel_encoder *intel_encoder = intel_connector->encoder;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&intel_encoder->base);

	seq_printf(m, "\taudio support: %s\n", intel_hdmi->has_audio ? "yes" :
		   "no");
}

static void intel_lvds_info(struct seq_file *m,
			    struct intel_connector *intel_connector)
{
	intel_panel_info(m, &intel_connector->panel);
}

static void intel_connector_info(struct seq_file *m,
				 struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_encoder *intel_encoder = intel_connector->encoder;
	struct drm_display_mode *mode;

	seq_printf(m, "connector %d: type %s, status: %s\n",
		   connector->base.id, connector->name,
		   drm_get_connector_status_name(connector->status));
	if (connector->status == connector_status_connected) {
		seq_printf(m, "\tname: %s\n", connector->display_info.name);
		seq_printf(m, "\tphysical dimensions: %dx%dmm\n",
			   connector->display_info.width_mm,
			   connector->display_info.height_mm);
		seq_printf(m, "\tsubpixel order: %s\n",
			   drm_get_subpixel_order_name(connector->display_info.subpixel_order));
		seq_printf(m, "\tCEA rev: %d\n",
			   connector->display_info.cea_rev);
	}
	if (intel_encoder->type == INTEL_OUTPUT_DISPLAYPORT ||
	    intel_encoder->type == INTEL_OUTPUT_EDP)
		intel_dp_info(m, intel_connector);
	else if (intel_encoder->type == INTEL_OUTPUT_HDMI)
		intel_hdmi_info(m, intel_connector);
	else if (intel_encoder->type == INTEL_OUTPUT_LVDS)
		intel_lvds_info(m, intel_connector);

	seq_printf(m, "\tmodes:\n");
	list_for_each_entry(mode, &connector->modes, head)
		intel_seq_print_mode(m, 2, mode);
}

static bool cursor_active(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 state;

	if (IS_845G(dev) || IS_I865G(dev))
		state = I915_READ(_CURACNTR) & CURSOR_ENABLE;
	else
		state = I915_READ(CURCNTR(pipe)) & CURSOR_MODE;

	return state;
}

static bool cursor_position(struct drm_device *dev, int pipe, int *x, int *y)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pos;

	pos = I915_READ(CURPOS(pipe));

	*x = (pos >> CURSOR_X_SHIFT) & CURSOR_POS_MASK;
	if (pos & (CURSOR_POS_SIGN << CURSOR_X_SHIFT))
		*x = -*x;

	*y = (pos >> CURSOR_Y_SHIFT) & CURSOR_POS_MASK;
	if (pos & (CURSOR_POS_SIGN << CURSOR_Y_SHIFT))
		*y = -*y;

	return cursor_active(dev, pipe);
}

static int i915_display_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc;
	struct drm_connector *connector;

	intel_runtime_pm_get(dev_priv);
	drm_modeset_lock_all(dev);
	seq_printf(m, "CRTC info\n");
	seq_printf(m, "---------\n");
	for_each_intel_crtc(dev, crtc) {
		bool active;
		int x, y;

		seq_printf(m, "CRTC %d: pipe: %c, active: %s\n",
			   crtc->base.base.id, pipe_name(crtc->pipe),
			   yesno(crtc->active));
		if (crtc->active) {
			intel_crtc_info(m, crtc);

			active = cursor_position(dev, crtc->pipe, &x, &y);
			seq_printf(m, "\tcursor visible? %s, position (%d, %d), addr 0x%08x, active? %s\n",
				   yesno(crtc->cursor_base),
				   x, y, crtc->cursor_addr,
				   yesno(active));
		}

		seq_printf(m, "\tunderrun reporting: cpu=%s pch=%s \n",
			   yesno(!crtc->cpu_fifo_underrun_disabled),
			   yesno(!crtc->pch_fifo_underrun_disabled));
	}

	seq_printf(m, "\n");
	seq_printf(m, "Connector info\n");
	seq_printf(m, "--------------\n");
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		intel_connector_info(m, connector);
	}
	drm_modeset_unlock_all(dev);
	intel_runtime_pm_put(dev_priv);

	return 0;
}

static int i915_runtime_pm_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *drm_dev = node->minor->dev;
	struct device *dev = drm_dev->dev;
	struct pci_dev *pdev = to_pci_dev(dev);

	seq_printf(m, "%-25s 0x%x\n", "pci state:", pdev->current_state);
	seq_printf(m, "%-25s %d\n", "usage count:",
					atomic_read(&dev->power.usage_count));

	return 0;
}

static int i915_wa_registers(struct seq_file *m, void *unused)
{
	int i;
	int ret;
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	intel_runtime_pm_get(dev_priv);

	seq_printf(m, "Workarounds applied: %d\n", dev_priv->workarounds.count);
	for (i = 0; i < dev_priv->workarounds.count; ++i) {
		u32 addr, mask, value, read;
		bool ok;

		addr = dev_priv->workarounds.reg[i].addr;
		mask = dev_priv->workarounds.reg[i].mask;
		value = dev_priv->workarounds.reg[i].value;
		read = I915_READ(addr);
		ok = (value & mask) == (read & mask);
		seq_printf(m, "0x%X: 0x%08X, mask: 0x%08X, read: 0x%08x, status: %s\n",
			   addr, value, mask, read, ok ? "OK" : "FAIL");
	}

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_scheduler_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler   *scheduler = dev_priv->scheduler;
	struct i915_scheduler_stats *stats = scheduler->stats;
	struct i915_scheduler_stats_nodes node_stats[I915_NUM_RINGS];
	struct intel_engine_cs *ring;
	char   str[50 * (I915_NUM_RINGS + 1)], name[50], *ptr;
	int ret, i, r;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

#define PRINT_VAR(name, fmt, var)					\
	do {								\
		sprintf(str, "%-22s", name);				\
		ptr = str + strlen(str);				\
		for_each_ring(ring, dev_priv, r) {			\
			sprintf(ptr, " %10" fmt, var);			\
			ptr += strlen(ptr);				\
		}							\
		seq_printf(m, "%s\n", str);				\
	} while (0)

	PRINT_VAR("Ring name:",             "s", dev_priv->ring[r].name);
	PRINT_VAR("  Ring seqno",           "d", ring->get_seqno(ring, false));
	seq_putc(m, '\n');

	seq_puts(m, "Batch submissions:\n");
	PRINT_VAR("  Queued",               "u", stats[r].queued);
	PRINT_VAR("  Submitted",            "u", stats[r].submitted);
	PRINT_VAR("  Completed",            "u", stats[r].completed);
	PRINT_VAR("  Expired",              "u", stats[r].expired);
	seq_putc(m, '\n');

	seq_puts(m, "Flush counts:\n");
	PRINT_VAR("  By object",            "u", stats[r].flush_obj);
	PRINT_VAR("  By request",           "u", stats[r].flush_req);
	PRINT_VAR("  Blanket",              "u", stats[r].flush_all);
	PRINT_VAR("  Entries bumped",       "u", stats[r].flush_bump);
	PRINT_VAR("  Entries submitted",    "u", stats[r].flush_submit);
	seq_putc(m, '\n');

	seq_puts(m, "Miscellaneous:\n");
	PRINT_VAR("  ExecEarly retry",      "u", stats[r].exec_early);
	PRINT_VAR("  ExecFinal requeue",    "u", stats[r].exec_again);
	PRINT_VAR("  ExecFinal killed",     "u", stats[r].exec_dead);
	PRINT_VAR("  Fence wait",           "u", stats[r].fence_wait);
	PRINT_VAR("  Fence wait again",     "u", stats[r].fence_again);
	PRINT_VAR("  Fence wait ignore",    "u", stats[r].fence_ignore);
	PRINT_VAR("  Fence supplied",       "u", stats[r].fence_got);
	PRINT_VAR("  Hung flying",          "u", stats[r].kill_flying);
	PRINT_VAR("  Hung queued",          "u", stats[r].kill_queued);
	seq_putc(m, '\n');

	seq_puts(m, "Queue contents:\n");
	for_each_ring(ring, dev_priv, i)
		i915_scheduler_query_stats(ring, node_stats + ring->id);

	for (i = 0; i < (i915_sqs_MAX + 1); i++) {
		sprintf(name, "  %s", i915_scheduler_queue_status_str(i));
		PRINT_VAR(name, "d", node_stats[r].counts[i]);
	}
	seq_putc(m, '\n');

#undef PRINT_VAR

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

struct pipe_crc_info {
	const char *name;
	struct drm_device *dev;
	enum pipe pipe;
};

static int i915_pipe_crc_open(struct inode *inode, struct file *filep)
{
	struct pipe_crc_info *info = inode->i_private;
	struct drm_i915_private *dev_priv = info->dev->dev_private;
	struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[info->pipe];

	if (info->pipe >= INTEL_INFO(info->dev)->num_pipes)
		return -ENODEV;

	spin_lock_irq(&pipe_crc->lock);

	if (pipe_crc->opened) {
		spin_unlock_irq(&pipe_crc->lock);
		return -EBUSY; /* already open */
	}

	pipe_crc->opened = true;
	filep->private_data = inode->i_private;

	spin_unlock_irq(&pipe_crc->lock);

	return 0;
}

static int i915_pipe_crc_release(struct inode *inode, struct file *filep)
{
	struct pipe_crc_info *info = inode->i_private;
	struct drm_i915_private *dev_priv = info->dev->dev_private;
	struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[info->pipe];

	spin_lock_irq(&pipe_crc->lock);
	pipe_crc->opened = false;
	spin_unlock_irq(&pipe_crc->lock);

	return 0;
}

/* (6 fields, 8 chars each, space separated (5) + '\n') */
#define PIPE_CRC_LINE_LEN	(6 * 8 + 5 + 1)
/* account for \'0' */
#define PIPE_CRC_BUFFER_LEN	(PIPE_CRC_LINE_LEN + 1)

static int pipe_crc_data_count(struct intel_pipe_crc *pipe_crc)
{
	assert_spin_locked(&pipe_crc->lock);
	return CIRC_CNT(pipe_crc->head, pipe_crc->tail,
			INTEL_PIPE_CRC_ENTRIES_NR);
}

static ssize_t
i915_pipe_crc_read(struct file *filep, char __user *user_buf, size_t count,
		   loff_t *pos)
{
	struct pipe_crc_info *info = filep->private_data;
	struct drm_device *dev = info->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[info->pipe];
	char buf[PIPE_CRC_BUFFER_LEN];
	int head, tail, n_entries, n;
	ssize_t bytes_read;

	/*
	 * Don't allow user space to provide buffers not big enough to hold
	 * a line of data.
	 */
	if (count < PIPE_CRC_LINE_LEN)
		return -EINVAL;

	if (pipe_crc->source == INTEL_PIPE_CRC_SOURCE_NONE)
		return 0;

	/* nothing to read */
	spin_lock_irq(&pipe_crc->lock);
	while (pipe_crc_data_count(pipe_crc) == 0) {
		int ret;

		if (filep->f_flags & O_NONBLOCK) {
			spin_unlock_irq(&pipe_crc->lock);
			return -EAGAIN;
		}

		ret = wait_event_interruptible_lock_irq(pipe_crc->wq,
				pipe_crc_data_count(pipe_crc), pipe_crc->lock);
		if (ret) {
			spin_unlock_irq(&pipe_crc->lock);
			return ret;
		}
	}

	/* We now have one or more entries to read */
	head = pipe_crc->head;
	tail = pipe_crc->tail;
	n_entries = min((size_t)CIRC_CNT(head, tail, INTEL_PIPE_CRC_ENTRIES_NR),
			count / PIPE_CRC_LINE_LEN);
	spin_unlock_irq(&pipe_crc->lock);

	bytes_read = 0;
	n = 0;
	do {
		struct intel_pipe_crc_entry *entry = &pipe_crc->entries[tail];
		int ret;

		bytes_read += snprintf(buf, PIPE_CRC_BUFFER_LEN,
				       "%8u %8x %8x %8x %8x %8x\n",
				       entry->frame, entry->crc[0],
				       entry->crc[1], entry->crc[2],
				       entry->crc[3], entry->crc[4]);

		ret = copy_to_user(user_buf + n * PIPE_CRC_LINE_LEN,
				   buf, PIPE_CRC_LINE_LEN);
		if (ret == PIPE_CRC_LINE_LEN)
			return -EFAULT;

		BUILD_BUG_ON_NOT_POWER_OF_2(INTEL_PIPE_CRC_ENTRIES_NR);
		tail = (tail + 1) & (INTEL_PIPE_CRC_ENTRIES_NR - 1);
		n++;
	} while (--n_entries);

	spin_lock_irq(&pipe_crc->lock);
	pipe_crc->tail = tail;
	spin_unlock_irq(&pipe_crc->lock);

	return bytes_read;
}

static const struct file_operations i915_pipe_crc_fops = {
	.owner = THIS_MODULE,
	.open = i915_pipe_crc_open,
	.read = i915_pipe_crc_read,
	.release = i915_pipe_crc_release,
};

static struct pipe_crc_info i915_pipe_crc_data[I915_MAX_PIPES] = {
	{
		.name = "i915_pipe_A_crc",
		.pipe = PIPE_A,
	},
	{
		.name = "i915_pipe_B_crc",
		.pipe = PIPE_B,
	},
	{
		.name = "i915_pipe_C_crc",
		.pipe = PIPE_C,
	},
};

static int i915_pipe_crc_create(struct dentry *root, struct drm_minor *minor,
				enum pipe pipe)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;
	struct pipe_crc_info *info = &i915_pipe_crc_data[pipe];

	info->dev = dev;
	ent = debugfs_create_file(info->name, S_IRUGO, root, info,
				  &i915_pipe_crc_fops);
	if (!ent)
		return -ENOMEM;

	return drm_add_fake_info_node(minor, ent, info);
}

static const char * const pipe_crc_sources[] = {
	"none",
	"plane1",
	"plane2",
	"pf",
	"pipe",
	"TV",
	"DP-B",
	"DP-C",
	"DP-D",
	"auto",
};

static const char *pipe_crc_source_name(enum intel_pipe_crc_source source)
{
	BUILD_BUG_ON(ARRAY_SIZE(pipe_crc_sources) != INTEL_PIPE_CRC_SOURCE_MAX);
	return pipe_crc_sources[source];
}

static int display_crc_ctl_show(struct seq_file *m, void *data)
{
	struct drm_device *dev = m->private;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;

	for (i = 0; i < I915_MAX_PIPES; i++)
		seq_printf(m, "%c %s\n", pipe_name(i),
			   pipe_crc_source_name(dev_priv->pipe_crc[i].source));

	return 0;
}

static int display_crc_ctl_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;

	return single_open(file, display_crc_ctl_show, dev);
}

static int i8xx_pipe_crc_ctl_reg(enum intel_pipe_crc_source *source,
				 uint32_t *val)
{
	if (*source == INTEL_PIPE_CRC_SOURCE_AUTO)
		*source = INTEL_PIPE_CRC_SOURCE_PIPE;

	switch (*source) {
	case INTEL_PIPE_CRC_SOURCE_PIPE:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_INCLUDE_BORDER_I8XX;
		break;
	case INTEL_PIPE_CRC_SOURCE_NONE:
		*val = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int i9xx_pipe_crc_auto_source(struct drm_device *dev, enum pipe pipe,
				     enum intel_pipe_crc_source *source)
{
	struct intel_encoder *encoder;
	struct intel_crtc *crtc;
	struct intel_digital_port *dig_port;
	int ret = 0;

	*source = INTEL_PIPE_CRC_SOURCE_PIPE;

	drm_modeset_lock_all(dev);
	list_for_each_entry(encoder, &dev->mode_config.encoder_list,
			    base.head) {
		if (!encoder->base.crtc)
			continue;

		crtc = to_intel_crtc(encoder->base.crtc);

		if (crtc->pipe != pipe)
			continue;

		switch (encoder->type) {
		case INTEL_OUTPUT_TVOUT:
			*source = INTEL_PIPE_CRC_SOURCE_TV;
			break;
		case INTEL_OUTPUT_DISPLAYPORT:
		case INTEL_OUTPUT_EDP:
			dig_port = enc_to_dig_port(&encoder->base);
			switch (dig_port->port) {
			case PORT_B:
				*source = INTEL_PIPE_CRC_SOURCE_DP_B;
				break;
			case PORT_C:
				*source = INTEL_PIPE_CRC_SOURCE_DP_C;
				break;
			case PORT_D:
				*source = INTEL_PIPE_CRC_SOURCE_DP_D;
				break;
			default:
				WARN(1, "nonexisting DP port %c\n",
				     port_name(dig_port->port));
				break;
			}
			break;
		}
	}
	drm_modeset_unlock_all(dev);

	return ret;
}

static int vlv_pipe_crc_ctl_reg(struct drm_device *dev,
				enum pipe pipe,
				enum intel_pipe_crc_source *source,
				uint32_t *val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool need_stable_symbols = false;

	if (*source == INTEL_PIPE_CRC_SOURCE_AUTO) {
		int ret = i9xx_pipe_crc_auto_source(dev, pipe, source);
		if (ret)
			return ret;
	}

	switch (*source) {
	case INTEL_PIPE_CRC_SOURCE_PIPE:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PIPE_VLV;
		break;
	case INTEL_PIPE_CRC_SOURCE_DP_B:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_DP_B_VLV;
		need_stable_symbols = true;
		break;
	case INTEL_PIPE_CRC_SOURCE_DP_C:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_DP_C_VLV;
		need_stable_symbols = true;
		break;
	case INTEL_PIPE_CRC_SOURCE_NONE:
		*val = 0;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * When the pipe CRC tap point is after the transcoders we need
	 * to tweak symbol-level features to produce a deterministic series of
	 * symbols for a given frame. We need to reset those features only once
	 * a frame (instead of every nth symbol):
	 *   - DC-balance: used to ensure a better clock recovery from the data
	 *     link (SDVO)
	 *   - DisplayPort scrambling: used for EMI reduction
	 */
	if (need_stable_symbols) {
		uint32_t tmp = I915_READ(PORT_DFT2_G4X);

		tmp |= DC_BALANCE_RESET_VLV;
		if (pipe == PIPE_A)
			tmp |= PIPE_A_SCRAMBLE_RESET;
		else
			tmp |= PIPE_B_SCRAMBLE_RESET;

		I915_WRITE(PORT_DFT2_G4X, tmp);
	}

	return 0;
}

static int i9xx_pipe_crc_ctl_reg(struct drm_device *dev,
				 enum pipe pipe,
				 enum intel_pipe_crc_source *source,
				 uint32_t *val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool need_stable_symbols = false;

	if (*source == INTEL_PIPE_CRC_SOURCE_AUTO) {
		int ret = i9xx_pipe_crc_auto_source(dev, pipe, source);
		if (ret)
			return ret;
	}

	switch (*source) {
	case INTEL_PIPE_CRC_SOURCE_PIPE:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PIPE_I9XX;
		break;
	case INTEL_PIPE_CRC_SOURCE_TV:
		if (!SUPPORTS_TV(dev))
			return -EINVAL;
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_TV_PRE;
		break;
	case INTEL_PIPE_CRC_SOURCE_DP_B:
		if (!IS_G4X(dev))
			return -EINVAL;
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_DP_B_G4X;
		need_stable_symbols = true;
		break;
	case INTEL_PIPE_CRC_SOURCE_DP_C:
		if (!IS_G4X(dev))
			return -EINVAL;
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_DP_C_G4X;
		need_stable_symbols = true;
		break;
	case INTEL_PIPE_CRC_SOURCE_DP_D:
		if (!IS_G4X(dev))
			return -EINVAL;
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_DP_D_G4X;
		need_stable_symbols = true;
		break;
	case INTEL_PIPE_CRC_SOURCE_NONE:
		*val = 0;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * When the pipe CRC tap point is after the transcoders we need
	 * to tweak symbol-level features to produce a deterministic series of
	 * symbols for a given frame. We need to reset those features only once
	 * a frame (instead of every nth symbol):
	 *   - DC-balance: used to ensure a better clock recovery from the data
	 *     link (SDVO)
	 *   - DisplayPort scrambling: used for EMI reduction
	 */
	if (need_stable_symbols) {
		uint32_t tmp = I915_READ(PORT_DFT2_G4X);

		WARN_ON(!IS_G4X(dev));

		I915_WRITE(PORT_DFT_I9XX,
			   I915_READ(PORT_DFT_I9XX) | DC_BALANCE_RESET);

		if (pipe == PIPE_A)
			tmp |= PIPE_A_SCRAMBLE_RESET;
		else
			tmp |= PIPE_B_SCRAMBLE_RESET;

		I915_WRITE(PORT_DFT2_G4X, tmp);
	}

	return 0;
}

static void vlv_undo_pipe_scramble_reset(struct drm_device *dev,
					 enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t tmp = I915_READ(PORT_DFT2_G4X);

	if (pipe == PIPE_A)
		tmp &= ~PIPE_A_SCRAMBLE_RESET;
	else
		tmp &= ~PIPE_B_SCRAMBLE_RESET;
	if (!(tmp & PIPE_SCRAMBLE_RESET_MASK))
		tmp &= ~DC_BALANCE_RESET_VLV;
	I915_WRITE(PORT_DFT2_G4X, tmp);

}

static void g4x_undo_pipe_scramble_reset(struct drm_device *dev,
					 enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t tmp = I915_READ(PORT_DFT2_G4X);

	if (pipe == PIPE_A)
		tmp &= ~PIPE_A_SCRAMBLE_RESET;
	else
		tmp &= ~PIPE_B_SCRAMBLE_RESET;
	I915_WRITE(PORT_DFT2_G4X, tmp);

	if (!(tmp & PIPE_SCRAMBLE_RESET_MASK)) {
		I915_WRITE(PORT_DFT_I9XX,
			   I915_READ(PORT_DFT_I9XX) & ~DC_BALANCE_RESET);
	}
}

static int ilk_pipe_crc_ctl_reg(enum intel_pipe_crc_source *source,
				uint32_t *val)
{
	if (*source == INTEL_PIPE_CRC_SOURCE_AUTO)
		*source = INTEL_PIPE_CRC_SOURCE_PIPE;

	switch (*source) {
	case INTEL_PIPE_CRC_SOURCE_PLANE1:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PRIMARY_ILK;
		break;
	case INTEL_PIPE_CRC_SOURCE_PLANE2:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_SPRITE_ILK;
		break;
	case INTEL_PIPE_CRC_SOURCE_PIPE:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PIPE_ILK;
		break;
	case INTEL_PIPE_CRC_SOURCE_NONE:
		*val = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ivb_pipe_crc_ctl_reg(enum intel_pipe_crc_source *source,
				uint32_t *val)
{
	if (*source == INTEL_PIPE_CRC_SOURCE_AUTO)
		*source = INTEL_PIPE_CRC_SOURCE_PF;

	switch (*source) {
	case INTEL_PIPE_CRC_SOURCE_PLANE1:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PRIMARY_IVB;
		break;
	case INTEL_PIPE_CRC_SOURCE_PLANE2:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_SPRITE_IVB;
		break;
	case INTEL_PIPE_CRC_SOURCE_PF:
		*val = PIPE_CRC_ENABLE | PIPE_CRC_SOURCE_PF_IVB;
		break;
	case INTEL_PIPE_CRC_SOURCE_NONE:
		*val = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pipe_crc_set_source(struct drm_device *dev, enum pipe pipe,
			       enum intel_pipe_crc_source source)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_pipe_crc *pipe_crc = NULL;
	u32 val = 0; /* shut up gcc */
	int ret;

	if ((pipe < PIPE_A) || (pipe >= I915_MAX_PIPES))
		return -EINVAL;

	pipe_crc = &dev_priv->pipe_crc[pipe];

	if (pipe_crc->source == source)
		return 0;

	/* forbid changing the source without going back to 'none' */
	if (pipe_crc->source && source)
		return -EINVAL;

	if (IS_GEN2(dev))
		ret = i8xx_pipe_crc_ctl_reg(&source, &val);
	else if (INTEL_INFO(dev)->gen < 5)
		ret = i9xx_pipe_crc_ctl_reg(dev, pipe, &source, &val);
	else if (IS_VALLEYVIEW(dev))
		ret = vlv_pipe_crc_ctl_reg(dev,pipe, &source, &val);
	else if (IS_GEN5(dev) || IS_GEN6(dev))
		ret = ilk_pipe_crc_ctl_reg(&source, &val);
	else
		ret = ivb_pipe_crc_ctl_reg(&source, &val);

	if (ret != 0)
		return ret;

	/* none -> real source transition */
	if (source) {
		DRM_DEBUG_DRIVER("collecting CRCs for pipe %c, %s\n",
				 pipe_name(pipe), pipe_crc_source_name(source));

		pipe_crc->entries = kzalloc(sizeof(*pipe_crc->entries) *
					    INTEL_PIPE_CRC_ENTRIES_NR,
					    GFP_KERNEL);
		if (!pipe_crc->entries)
			return -ENOMEM;

		spin_lock_irq(&pipe_crc->lock);
		pipe_crc->head = 0;
		pipe_crc->tail = 0;
		spin_unlock_irq(&pipe_crc->lock);
	}

	pipe_crc->source = source;

	I915_WRITE(PIPE_CRC_CTL(pipe), val);
	POSTING_READ(PIPE_CRC_CTL(pipe));

	/* real source -> none transition */
	if (source == INTEL_PIPE_CRC_SOURCE_NONE) {
		struct intel_pipe_crc_entry *entries;

		DRM_DEBUG_DRIVER("stopping CRCs for pipe %c\n",
				 pipe_name(pipe));

		intel_wait_for_vblank(dev, pipe);

		spin_lock_irq(&pipe_crc->lock);
		entries = pipe_crc->entries;
		pipe_crc->entries = NULL;
		spin_unlock_irq(&pipe_crc->lock);

		kfree(entries);

		if (IS_G4X(dev))
			g4x_undo_pipe_scramble_reset(dev, pipe);
		else if (IS_VALLEYVIEW(dev))
			vlv_undo_pipe_scramble_reset(dev, pipe);
	}

	return 0;
}

/*
 * Parse pipe CRC command strings:
 *   command: wsp* object wsp+ name wsp+ source wsp*
 *   object: 'pipe'
 *   name: (A | B | C)
 *   source: (none | plane1 | plane2 | pf)
 *   wsp: (#0x20 | #0x9 | #0xA)+
 *
 * eg.:
 *  "pipe A plane1"  ->  Start CRC computations on plane1 of pipe A
 *  "pipe A none"    ->  Stop CRC
 */
static int display_crc_ctl_tokenize(char *buf, char *words[], int max_words)
{
	int n_words = 0;

	while (*buf) {
		char *end;

		/* skip leading white space */
		buf = skip_spaces(buf);
		if (!*buf)
			break;	/* end of buffer */

		/* find end of word */
		for (end = buf; *end && !isspace(*end); end++)
			;

		if (n_words == max_words) {
			DRM_DEBUG_DRIVER("too many words, allowed <= %d\n",
					 max_words);
			return -EINVAL;	/* ran out of words[] before bytes */
		}

		if (*end)
			*end++ = '\0';
		words[n_words++] = buf;
		buf = end;
	}

	return n_words;
}

enum intel_pipe_crc_object {
	PIPE_CRC_OBJECT_PIPE,
};

static const char * const pipe_crc_objects[] = {
	"pipe",
};

static int
display_crc_ctl_parse_object(const char *buf, enum intel_pipe_crc_object *o)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pipe_crc_objects); i++)
		if (!strcmp(buf, pipe_crc_objects[i])) {
			*o = i;
			return 0;
		    }

	return -EINVAL;
}

static int display_crc_ctl_parse_pipe(const char *buf, enum pipe *pipe)
{
	const char name = buf[0];

	if (name < 'A' || name >= pipe_name(I915_MAX_PIPES))
		return -EINVAL;

	*pipe = name - 'A';

	return 0;
}

static int
display_crc_ctl_parse_source(const char *buf, enum intel_pipe_crc_source *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pipe_crc_sources); i++)
		if (!strcmp(buf, pipe_crc_sources[i])) {
			*s = i;
			return 0;
		    }

	return -EINVAL;
}

static int display_crc_ctl_parse(struct drm_device *dev, char *buf, size_t len)
{
#define N_WORDS 3
	int n_words;
	char *words[N_WORDS];
	enum pipe pipe;
	enum intel_pipe_crc_object object;
	enum intel_pipe_crc_source source;

	n_words = display_crc_ctl_tokenize(buf, words, N_WORDS);
	if (n_words != N_WORDS) {
		DRM_DEBUG_DRIVER("tokenize failed, a command is %d words\n",
				 N_WORDS);
		return -EINVAL;
	}

	if (display_crc_ctl_parse_object(words[0], &object) < 0) {
		DRM_DEBUG_DRIVER("unknown object %s\n", words[0]);
		return -EINVAL;
	}

	if (display_crc_ctl_parse_pipe(words[1], &pipe) < 0) {
		DRM_DEBUG_DRIVER("unknown pipe %s\n", words[1]);
		return -EINVAL;
	}

	if (display_crc_ctl_parse_source(words[2], &source) < 0) {
		DRM_DEBUG_DRIVER("unknown source %s\n", words[2]);
		return -EINVAL;
	}

	return pipe_crc_set_source(dev, pipe, source);
}

static ssize_t display_crc_ctl_write(struct file *file, const char __user *ubuf,
				     size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;
	char *tmpbuf;
	int ret;

	if (len == 0)
		return 0;

	if (len > PAGE_SIZE - 1) {
		DRM_DEBUG_DRIVER("expected <%lu bytes into pipe crc control\n",
				 PAGE_SIZE);
		return -E2BIG;
	}

	tmpbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!tmpbuf)
		return -ENOMEM;

	if (copy_from_user(tmpbuf, ubuf, len)) {
		ret = -EFAULT;
		goto out;
	}
	tmpbuf[len] = '\0';

	ret = display_crc_ctl_parse(dev, tmpbuf, len);

out:
	kfree(tmpbuf);
	if (ret < 0)
		return ret;

	*offp += len;
	return len;
}

static const struct file_operations i915_display_crc_ctl_fops = {
	.owner = THIS_MODULE,
	.open = display_crc_ctl_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = display_crc_ctl_write
};

static void wm_latency_show(struct seq_file *m, const uint16_t wm[5])
{
	struct drm_device *dev = m->private;
	int num_levels = ilk_wm_max_level(dev) + 1;
	int level;

	drm_modeset_lock_all(dev);

	for (level = 0; level < num_levels; level++) {
		unsigned int latency = wm[level];

		/* WM1+ latency values in 0.5us units */
		if (level > 0)
			latency *= 5;

		seq_printf(m, "WM%d %u (%u.%u usec)\n",
			   level, wm[level],
			   latency / 10, latency % 10);
	}

	drm_modeset_unlock_all(dev);
}

static int pri_wm_latency_show(struct seq_file *m, void *data)
{
	struct drm_device *dev = m->private;

	wm_latency_show(m, to_i915(dev)->wm.pri_latency);

	return 0;
}

static int spr_wm_latency_show(struct seq_file *m, void *data)
{
	struct drm_device *dev = m->private;

	wm_latency_show(m, to_i915(dev)->wm.spr_latency);

	return 0;
}

static int cur_wm_latency_show(struct seq_file *m, void *data)
{
	struct drm_device *dev = m->private;

	wm_latency_show(m, to_i915(dev)->wm.cur_latency);

	return 0;
}

static int pri_wm_latency_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;

	if (!HAS_PCH_SPLIT(dev))
		return -ENODEV;

	return single_open(file, pri_wm_latency_show, dev);
}

static int spr_wm_latency_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;

	if (!HAS_PCH_SPLIT(dev))
		return -ENODEV;

	return single_open(file, spr_wm_latency_show, dev);
}

static int cur_wm_latency_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;

	if (!HAS_PCH_SPLIT(dev))
		return -ENODEV;

	return single_open(file, cur_wm_latency_show, dev);
}

static ssize_t wm_latency_write(struct file *file, const char __user *ubuf,
				size_t len, loff_t *offp, uint16_t wm[5])
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;
	uint16_t new[5] = { 0 };
	int num_levels = ilk_wm_max_level(dev) + 1;
	int level;
	int ret;
	char tmp[32];

	if (len >= sizeof(tmp))
		return -EINVAL;

	if (copy_from_user(tmp, ubuf, len))
		return -EFAULT;

	tmp[len] = '\0';

	ret = sscanf(tmp, "%hu %hu %hu %hu %hu", &new[0], &new[1], &new[2], &new[3], &new[4]);
	if (ret != num_levels)
		return -EINVAL;

	drm_modeset_lock_all(dev);

	for (level = 0; level < num_levels; level++)
		wm[level] = new[level];

	drm_modeset_unlock_all(dev);

	return len;
}


static ssize_t pri_wm_latency_write(struct file *file, const char __user *ubuf,
				    size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;

	return wm_latency_write(file, ubuf, len, offp, to_i915(dev)->wm.pri_latency);
}

static ssize_t spr_wm_latency_write(struct file *file, const char __user *ubuf,
				    size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;

	return wm_latency_write(file, ubuf, len, offp, to_i915(dev)->wm.spr_latency);
}

static ssize_t cur_wm_latency_write(struct file *file, const char __user *ubuf,
				    size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct drm_device *dev = m->private;

	return wm_latency_write(file, ubuf, len, offp, to_i915(dev)->wm.cur_latency);
}

static const struct file_operations i915_pri_wm_latency_fops = {
	.owner = THIS_MODULE,
	.open = pri_wm_latency_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = pri_wm_latency_write
};

static const struct file_operations i915_spr_wm_latency_fops = {
	.owner = THIS_MODULE,
	.open = spr_wm_latency_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = spr_wm_latency_write
};

static const struct file_operations i915_cur_wm_latency_fops = {
	.owner = THIS_MODULE,
	.open = cur_wm_latency_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = cur_wm_latency_write
};

static ssize_t
i915_sb_punit_read(struct file *filp, char __user *ubuf, size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[] = "punit read is not defined";
	u32 rval, len;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (!dev_priv->debug.punit_read)
		return 0;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	mutex_lock(&dev_priv->rps.hw_lock);
	rval = vlv_punit_read(dev_priv, dev_priv->debug.punit_read);
	mutex_unlock(&dev_priv->rps.hw_lock);

	len = snprintf(buf, sizeof(buf),
			"0x%x: 0x%x\n", dev_priv->debug.punit_read, rval);
	if (len > sizeof(buf))
		len = sizeof(buf);

	dev_priv->debug.punit_read = 0;
	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}


static ssize_t
i915_sb_punit_write(struct file *filp, const char __user *ubuf, size_t cnt,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[20];
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;

		buf[cnt] = 0;

		ret = kstrtoul(buf, 0,
				(unsigned long *)&dev_priv->debug.punit_read);
		if (ret)
			return -EINVAL;
	}

	return cnt;
}

static const struct file_operations i915_punit_read_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_sb_punit_read,
	.write = i915_sb_punit_write,
	.llseek = default_llseek,
};

static ssize_t
i915_sb_fuse_read(struct file *filp, char __user *ubuf, size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[] = "punit read is not defined";
	u32 rval, len;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (!dev_priv->debug.fuse_read)
		return 0;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	mutex_lock(&dev_priv->rps.hw_lock);
	rval = vlv_nc_read(dev_priv, dev_priv->debug.fuse_read);
	mutex_unlock(&dev_priv->rps.hw_lock);

	len = snprintf(buf, sizeof(buf),
			"0x%x: 0x%x\n", dev_priv->debug.fuse_read, rval);
	if (len > sizeof(buf))
		len = sizeof(buf);

	dev_priv->debug.fuse_read = 0;
	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_sb_fuse_write(struct file *filp, const char __user *ubuf, size_t cnt,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[20];
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;

		buf[cnt] = 0;

		ret = kstrtoul(buf, 0,
				(unsigned long *)&dev_priv->debug.fuse_read);
		if (ret)
			return -EINVAL;
	}

	return cnt;
}

static const struct file_operations i915_fuse_read_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_sb_fuse_read,
	.write = i915_sb_fuse_write,
	.llseek = default_llseek,
};

static int
i915_wedged_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = atomic_read(&dev_priv->gpu_error.reset_counter);

	return 0;
}

static int
i915_wedged_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	intel_runtime_pm_get(dev_priv);

	if (!i915_reset_in_progress(&dev_priv->gpu_error)) {
		switch (val) {
		case RCS:
			DRM_INFO("Manual RCS reset\n");
			i915_handle_error(dev,
					  &dev_priv->ring[RCS].hangcheck, 0,
					  "Manual RCS reset");
			break;
		case VCS:
			DRM_INFO("Manual VCS reset\n");
			i915_handle_error(dev,
					  &dev_priv->ring[VCS].hangcheck, 0,
					  "Manual VCS reset");
			break;
		case BCS:
			DRM_INFO("Manual BCS reset\n");
			i915_handle_error(dev,
					  &dev_priv->ring[BCS].hangcheck, 0,
					  "Manual BCS reset");
			break;
		case VECS:
			DRM_INFO("Manual VECS reset\n");
			i915_handle_error(dev,
					  &dev_priv->ring[VECS].hangcheck, 0,
					  "Manual VECS reset");
			break;
		case VCS2:
			DRM_INFO("Manual VCS2 reset\n");
			i915_handle_error(dev,
					  &dev_priv->ring[VCS2].hangcheck, 0,
					  "Manual VCS2 reset");
			break;
		default:
			DRM_INFO("Manual global reset\n");
			i915_handle_error(dev, NULL, 0, "Manual global reset");
			break;
		}
	}

	intel_runtime_pm_put(dev_priv);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_wedged_fops,
			i915_wedged_get, i915_wedged_set,
			"%llu\n");

static int
i915_fake_ctx_submission_inconsistency_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring;
	unsigned i;

	DRM_INFO("Faked inconsistent context submission state: %x\n",
		dev_priv->gpu_error.faked_lost_ctx_event_irq);

	for_each_ring(ring, dev_priv, i) {
		u32 fake_cnt =
			(dev_priv->gpu_error.faked_lost_ctx_event_irq >> (i<<2)) & 0xf;

		DRM_INFO("%s: Faking %s [%u IRQs left to drop]\n",
			ring->name,
			fake_cnt?"enabled":"disabled",
			fake_cnt);
	}

	*val = (u64) dev_priv->gpu_error.faked_lost_ctx_event_irq;

	return 0;
}

static int
i915_fake_ctx_submission_inconsistency_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 fake_status;

	/*
	 * val contains the new faked_lost_ctx_event_irq word that is to be
	 * merged with the already set faked_lost_ctx_event_irq word.
	 *
	 * val == 0 means clear all previously set fake bits.
	 *
	 * Each nibble contains a number between 0-15 denoting the number of
	 * interrupts left to lose on the engine that nibble corresponds to.
	 *
	 * RCS: faked_lost_ctx_event_irq[3:0]
	 * VCS: faked_lost_ctx_event_irq[7:4]
	 * BCS: faked_lost_ctx_event_irq[11:8]
	 * VECS: faked_lost_ctx_event_irq[15:12]
	 * VCS2: faked_lost_ctx_event_irq[19:16]
	 *
	 * The number in each nibble is decremented by the interrupt handler in
	 * intel_lrc.c once the faked interrupt loss is executed. If a
	 * targetted interrupt is received when bit corresponding to that
	 * engine is set that interrupt will be dropped without side-effects.
	 */

	fake_status =
		dev_priv->gpu_error.faked_lost_ctx_event_irq;

	DRM_INFO("Faking lost context event IRQ (new status: %x, old status: %x)\n",
		(u32) val, fake_status);

	if (val) {
		dev_priv->gpu_error.faked_lost_ctx_event_irq |= ((u32) val);
	} else {
		DRM_INFO("Clearing lost context event IRQ mask\n");

		dev_priv->gpu_error.faked_lost_ctx_event_irq = 0;
	}


	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_fake_ctx_submission_inconsistency_fops,
			i915_fake_ctx_submission_inconsistency_get,
			i915_fake_ctx_submission_inconsistency_set,
			"%llu\n");

static const char *ringid_to_str(enum intel_ring_id ring_id)
{
	switch (ring_id) {
	case RCS:
		return "RCS";
	case VCS:
		return "VCS";
	case BCS:
		return "BCS";
	case VECS:
		return "VECS";
	case VCS2:
		return "VCS2";
	}

	return "unknown";
}

static ssize_t
i915_ring_hangcheck_read(struct file *filp, char __user *ubuf,
			 size_t max, loff_t *ppos)
{
	int i;
	int len;
	char buf[300];
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/*
	 * Returns the total number of times the rings
	 * have hung and been reset since boot
	 */
	len = scnprintf(buf, sizeof(buf), "GPU=0x%08lX,",
			dev_priv->gpu_error.total_resets);

	for (i = 0; i < I915_NUM_RINGS; ++i)
		len += scnprintf(buf + len, sizeof(buf) - len,
				 "%s=0x%08lX,",
				 ringid_to_str(i),
				 (long unsigned)
				 dev_priv->ring[i].hangcheck.total);

	for (i = 0; i < I915_NUM_RINGS; ++i)
		len += scnprintf(buf + len, sizeof(buf) - len,
				 "%s_T=0x%08lX,",
				 ringid_to_str(i),
				 (long unsigned)
				 dev_priv->ring[i].hangcheck.tdr_count);

	for (i = 0; i < I915_NUM_RINGS; ++i)
		len += scnprintf(buf + len, sizeof(buf) - len,
				 "%s_W=0x%08lX,",
				 ringid_to_str(i),
				 (long unsigned)
				 dev_priv->ring[i].hangcheck.watchdog_count);
	len += scnprintf(buf + len - 1, sizeof(buf) - len, "\n");

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static const struct file_operations i915_ring_hangcheck_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_ring_hangcheck_read,
	.llseek = default_llseek,
};

static int
i915_ring_stop_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = dev_priv->gpu_error.stop_rings;

	return 0;
}

static int
i915_ring_stop_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	DRM_DEBUG_DRIVER("Stopping rings 0x%08llx\n", val);

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	dev_priv->gpu_error.stop_rings = val;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_ring_stop_fops,
			i915_ring_stop_get, i915_ring_stop_set,
			"0x%08llx\n");

static int
i915_ring_missed_irq_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = dev_priv->gpu_error.missed_irq_rings;
	return 0;
}

static int
i915_ring_missed_irq_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	/* Lock against concurrent debugfs callers */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	dev_priv->gpu_error.missed_irq_rings = val;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_ring_missed_irq_fops,
			i915_ring_missed_irq_get, i915_ring_missed_irq_set,
			"0x%08llx\n");

static int
i915_ring_test_irq_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	*val = dev_priv->gpu_error.test_irq_rings;

	return 0;
}

static int
i915_ring_test_irq_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	DRM_DEBUG_DRIVER("Masking interrupts on rings 0x%08llx\n", val);

	/* Lock against concurrent debugfs callers */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	dev_priv->gpu_error.test_irq_rings = val;
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_ring_test_irq_fops,
			i915_ring_test_irq_get, i915_ring_test_irq_set,
			"0x%08llx\n");

#define DROP_UNBOUND 0x1
#define DROP_BOUND 0x2
#define DROP_RETIRE 0x4
#define DROP_ACTIVE 0x8
#define DROP_ALL (DROP_UNBOUND | \
		  DROP_BOUND | \
		  DROP_RETIRE | \
		  DROP_ACTIVE)
static int
i915_drop_caches_get(void *data, u64 *val)
{
	*val = DROP_ALL;

	return 0;
}

static int
i915_drop_caches_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	DRM_DEBUG("Dropping caches: 0x%08llx\n", val);

	/* No need to check and wait for gpu resets, only libdrm auto-restarts
	 * on ioctls on -EAGAIN. */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	if (val & DROP_ACTIVE) {
		ret = i915_gpu_idle(dev);
		if (ret)
			goto unlock;
	}

	if (val & (DROP_RETIRE | DROP_ACTIVE))
		i915_gem_retire_requests(dev);

	if (val & DROP_BOUND)
		i915_gem_shrink(dev_priv, LONG_MAX, I915_SHRINK_BOUND);

	if (val & DROP_UNBOUND)
		i915_gem_shrink(dev_priv, LONG_MAX, I915_SHRINK_UNBOUND);

unlock:
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_drop_caches_fops,
			i915_drop_caches_get, i915_drop_caches_set,
			"0x%08llx\n");

static int
i915_max_freq_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev))
		*val = vlv_gpu_freq(dev_priv, dev_priv->rps.max_freq_softlimit);
	else
		*val = dev_priv->rps.max_freq_softlimit * GT_FREQUENCY_MULTIPLIER;
	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

static int
i915_max_freq_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rp_state_cap, hw_max, hw_min;
	int ret;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Manually setting max freq to %llu\n", val);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	/*
	 * Turbo will still be enabled, but won't go above the set value.
	 */
	if (IS_VALLEYVIEW(dev)) {
		val = vlv_freq_opcode(dev_priv, val);

		hw_max = dev_priv->rps.max_freq;
		hw_min = dev_priv->rps.min_freq;
	} else {
		do_div(val, GT_FREQUENCY_MULTIPLIER);

		rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
		hw_max = dev_priv->rps.max_freq;
		hw_min = (rp_state_cap >> 16) & 0xff;
	}

	if (val < hw_min || val > hw_max || val < dev_priv->rps.min_freq_softlimit) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -EINVAL;
	}

	dev_priv->rps.max_freq_softlimit = val;

	if (IS_VALLEYVIEW(dev))
		valleyview_set_rps(dev, val);
	else
		gen6_set_rps(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_max_freq_fops,
			i915_max_freq_get, i915_max_freq_set,
			"%llu\n");

static int
i915_min_freq_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev))
		*val = vlv_gpu_freq(dev_priv, dev_priv->rps.min_freq_softlimit);
	else
		*val = dev_priv->rps.min_freq_softlimit * GT_FREQUENCY_MULTIPLIER;
	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

static int
i915_min_freq_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rp_state_cap, hw_max, hw_min;
	int ret;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Manually setting min freq to %llu\n", val);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	/*
	 * Turbo will still be enabled, but won't go below the set value.
	 */
	if (IS_VALLEYVIEW(dev)) {
		val = vlv_freq_opcode(dev_priv, val);

		hw_max = dev_priv->rps.max_freq;
		hw_min = dev_priv->rps.min_freq;
	} else {
		do_div(val, GT_FREQUENCY_MULTIPLIER);

		rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
		hw_max = dev_priv->rps.max_freq;
		hw_min = (rp_state_cap >> 16) & 0xff;
	}

	if (val < hw_min || val > hw_max || val > dev_priv->rps.max_freq_softlimit) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -EINVAL;
	}

	dev_priv->rps.min_freq_softlimit = val;

	if (IS_VALLEYVIEW(dev))
		valleyview_set_rps(dev, val);
	else
		gen6_set_rps(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_min_freq_fops,
			i915_min_freq_get, i915_min_freq_set,
			"%llu\n");

static int i915_cur_freq_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	*val = dev_priv->rps.cur_freq * GT_FREQUENCY_MULTIPLIER;

	return 0;
}

static int i915_cur_freq_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u64 freq = val;
	int ret;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	/* Must be in manual mode to guarantee setting will persist. */
	if (!dev_priv->rps.manual_mode) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -EINVAL;
	}

	do_div(val, GT_FREQUENCY_MULTIPLIER);

	if (val < dev_priv->rps.min_freq_softlimit || val > dev_priv->rps.max_freq_softlimit) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return -EINVAL;
	}

	DRM_DEBUG_DRIVER("Setting current freq to %llu\n", freq);

	gen6_set_rps(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_cur_freq_fops,
			i915_cur_freq_get, i915_cur_freq_set,
			"%llu\n");

static int i915_rps_disable_boost_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return -ENODEV;

	*val = dev_priv->rps.debugfs_disable_boost;

	return 0;
}

static int i915_rps_disable_boost_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Setting RPS disable Boost-Idle mode to %s\n",
				 val ? "on" : "off");

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	dev_priv->rps.debugfs_disable_boost = val;

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_rps_disable_boost_fops,
		i915_rps_disable_boost_get, i915_rps_disable_boost_set,
			"%llu\n");

static int i915_rps_manual_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if ((INTEL_INFO(dev)->gen < 6) || IS_VALLEYVIEW(dev))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	*val = dev_priv->rps.manual_mode;

	return 0;
}

static int i915_rps_manual_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	if ((INTEL_INFO(dev)->gen < 6) || IS_VALLEYVIEW(dev))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Setting RPS mode to %s\n",
			 val ? "manual" : "normal");

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	gen6_set_rps_mode(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_rps_manual_fops,
			i915_rps_manual_get, i915_rps_manual_set,
			"%llu\n");

static int i915_rps_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_VALLEYVIEW(dev))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	*val = dev_priv->rps.rps_disable;

	return 0;
}

static int i915_rps_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	if (!IS_VALLEYVIEW(dev))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Setting RPS disable %s\n",
			 val ? "true" : "false");

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	vlv_set_rps_mode(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_rps_disable_fops,
			i915_rps_disable_get, i915_rps_disable_set,
			"%llu\n");

static int i915_rc6_disable_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if ((INTEL_INFO(dev)->gen < 6))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	*val = dev_priv->rps.rc6_disable;

	return 0;
}

static int i915_rc6_disable_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	if ((INTEL_INFO(dev)->gen < 6))
		return -ENODEV;

	flush_delayed_work(&dev_priv->rps.delayed_resume_work);

	DRM_DEBUG_DRIVER("Setting RC6 disable %s\n",
			 val ? "true" : "false");

	ret = mutex_lock_interruptible(&dev_priv->rps.hw_lock);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev))
		vlv_set_rc6_mode(dev, val);
	else
		gen6_set_rc6_mode(dev, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_rc6_disable_fops,
			i915_rc6_disable_get, i915_rc6_disable_set,
			"%llu\n");

static int
i915_cache_sharing_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 snpcr;
	int ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	intel_runtime_pm_get(dev_priv);

	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);

	intel_runtime_pm_put(dev_priv);
	mutex_unlock(&dev_priv->dev->struct_mutex);

	*val = (snpcr & GEN6_MBC_SNPCR_MASK) >> GEN6_MBC_SNPCR_SHIFT;

	return 0;
}

static int
i915_cache_sharing_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 snpcr;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	if (val > 3)
		return -EINVAL;

	intel_runtime_pm_get(dev_priv);
	DRM_DEBUG_DRIVER("Manually setting uncore sharing to %llu\n", val);

	/* Update the cache sharing policy here as well */
	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);
	snpcr &= ~GEN6_MBC_SNPCR_MASK;
	snpcr |= (val << GEN6_MBC_SNPCR_SHIFT);
	I915_WRITE(GEN6_MBCUNIT_SNPCR, snpcr);

	intel_runtime_pm_put(dev_priv);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i915_cache_sharing_fops,
			i915_cache_sharing_get, i915_cache_sharing_set,
			"%llu\n");

#define TIMESTAMP_BUFFER_LEN  100U

ssize_t i915_timestamp_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int len = 0;
	char buf[TIMESTAMP_BUFFER_LEN] = {0,};
	unsigned long flags;
	u32 gpu_ts_hi, gpu_ts_lo, gpu_ts_hi_new;
	u64 gpu_ts;
	unsigned int cpu;
	u32 sec, nsec;
	u64 ftrace_ts;

	local_irq_save(flags);
	cpu = smp_processor_id();

	gpu_ts_hi = I915_READ(RING_TIMESTAMP_HI(RENDER_RING_BASE));
	gpu_ts_lo = I915_READ(RING_TIMESTAMP_LO(RENDER_RING_BASE));
	gpu_ts_hi_new = I915_READ(RING_TIMESTAMP_HI(RENDER_RING_BASE));
	if (gpu_ts_hi_new != gpu_ts_hi) {
		gpu_ts_lo = I915_READ(RING_TIMESTAMP_LO(RENDER_RING_BASE));
		gpu_ts_hi = gpu_ts_hi_new;
	}
	gpu_ts = (u64)gpu_ts_hi << 32 | gpu_ts_lo;

	ftrace_ts = ftrace_now(cpu);
	local_irq_restore(flags);

	nsec = do_div(ftrace_ts, NSEC_PER_SEC);
	sec = (u32) ftrace_ts;

	len = snprintf(buf, TIMESTAMP_BUFFER_LEN,
		      "CPU%03u %u.%09u s\nGPU %llu ticks\n",
		      cpu, sec, nsec, gpu_ts);

	return simple_read_from_buffer(ubuf, max, ppos,
				       (const void *) buf, sizeof(buf));
}

static const struct file_operations i915_timestamp_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_timestamp_read,
	.write = NULL,
	.llseek = default_llseek,
};

static int i915_forcewake_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static int i915_forcewake_release(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static const struct file_operations i915_forcewake_fops = {
	.owner = THIS_MODULE,
	.open = i915_forcewake_open,
	.release = i915_forcewake_release,
};

static ssize_t i915_connector_reset_read(struct file *filp, char __user *ubuf,
	size_t max, loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct intel_connector *connector;
	char *tmpbuf;
	int ret_count = 0;

	tmpbuf = kmalloc(max + 1, GFP_KERNEL);
	if (!tmpbuf)
		return -ENOMEM;

	mutex_lock(&dev->mode_config.mutex);
	snprintf(&tmpbuf[ret_count], max - ret_count, "List Connectors:\n");
	ret_count = strlen(tmpbuf);

	list_for_each_entry(connector, &dev->mode_config.connector_list,
				base.head) {
	    switch (connector->encoder->type) {
	    case INTEL_OUTPUT_DSI:
		snprintf(&tmpbuf[ret_count], max - ret_count,
			"\tID=%d; Active:%d; Type=DSI\n",
			connector->base.base.id,
			connector->encoder->connectors_active);
		break;
	    case INTEL_OUTPUT_EDP:
		snprintf(&tmpbuf[ret_count], max - ret_count,
			"\tID=%d; Active:%d; Type=EDP\n",
			connector->base.base.id,
			connector->encoder->connectors_active);
		break;
	    default:
		DRM_DEBUG_DRIVER("Unsupported Type.ID=%d;Type=Unknow[%d]\n",
			connector->base.base.id,
			connector->encoder->type);
		break;
	    }
	    ret_count = strlen(tmpbuf);
	}
	ret_count = simple_read_from_buffer(ubuf, max, ppos,
		(const void *)tmpbuf, ret_count);
	mutex_unlock(&dev->mode_config.mutex);
	kfree(tmpbuf);
	return ret_count;
}

static ssize_t i915_connector_reset_write(struct file *filp,
			const char __user *ubuf,
			size_t cnt, loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct intel_connector *connector;
	u32 connector_id;
	char *tmpbuf;
	ssize_t ret;

	if (cnt == 0)
		return cnt;
	tmpbuf = kmalloc(cnt + 1, GFP_KERNEL);
	if (!tmpbuf)
		return -ENOMEM;

	if (copy_from_user(tmpbuf, ubuf, cnt)) {
		kfree(tmpbuf);
		return -EFAULT;
	}
	tmpbuf[cnt] = '\0';
	ret = kstrtou32(tmpbuf, 0, &connector_id);
	kfree(tmpbuf);
	if (ret)
		return -EINVAL;
#ifndef FEATURE_ESD_RESET
	return cnt;
#endif
	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			base.head) {
		if (connector->base.base.id == connector_id) {
			i915_esd.dsi_esd_state = DISPLAY_DSI_STATE_HANG;
			i915_esd.reset_flasg = ESD_IRQ_RESET;

			intel_connector_reset(&connector->base);
			DRM_DEBUG_DRIVER("Try to reset Connector %d\n",
				connector_id);
		}
	}
	mutex_unlock(&dev->mode_config.mutex);

	return cnt;
}

static const struct file_operations i915_connector_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_connector_reset_read,
	.write = i915_connector_reset_write,
	.llseek = default_llseek,
};

static int i915_forcewake_create(struct dentry *root, struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file("i915_forcewake_user",
				  S_IRUSR,
				  root, dev,
				  &i915_forcewake_fops);
	if (!ent)
		return -ENOMEM;

	return drm_add_fake_info_node(minor, ent, &i915_forcewake_fops);
}

static int i915_debugfs_create(struct dentry *root,
			       struct drm_minor *minor,
			       const char *name,
			       const struct file_operations *fops)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file(name,
				  S_IRUGO | S_IWUSR,
				  root, dev,
				  fops);
	if (!ent)
		return -ENOMEM;

	return drm_add_fake_info_node(minor, ent, fops);
}

static const struct drm_info_list i915_debugfs_list[] = {
	{"i915_capabilities", i915_capabilities, 0},
	{"i915_gem_objects", i915_gem_object_info, 0},
	{"i915_gem_gtt", i915_gem_gtt_info, 0},
	{"i915_gem_pinned", i915_gem_gtt_info, 0, (void *) PINNED_LIST},
	{"i915_gem_active", i915_gem_object_list_info, 0, (void *) ACTIVE_LIST},
	{"i915_gem_inactive", i915_gem_object_list_info, 0, (void *) INACTIVE_LIST},
	{"i915_gem_stolen", i915_gem_stolen_list_info },
	{"i915_gem_pageflip", i915_gem_pageflip_info, 0},
	{"i915_gem_request", i915_gem_request_info, 0},
	{"i915_gem_seqno", i915_gem_seqno_info, 0},
	{"i915_gem_fence_regs", i915_gem_fence_regs_info, 0},
	{"i915_gem_interrupt", i915_interrupt_info, 0},
	{"i915_gem_hws", i915_hws_info, 0, (void *)RCS},
	{"i915_gem_hws_blt", i915_hws_info, 0, (void *)BCS},
	{"i915_gem_hws_bsd", i915_hws_info, 0, (void *)VCS},
	{"i915_gem_hws_vebox", i915_hws_info, 0, (void *)VECS},
	{"i915_rstdby_delays", i915_rstdby_delays, 0},
	{"i915_frequency_info", i915_frequency_info, 0},
	{"i915_delayfreq_table", i915_delayfreq_table, 0},
	{"i915_inttoext_table", i915_inttoext_table, 0},
	{"i915_drpc_info", i915_drpc_info, 0},
	{"i915_emon_status", i915_emon_status, 0},
	{"i915_ring_freq_table", i915_ring_freq_table, 0},
	{"i915_gfxec", i915_gfxec, 0},
	{"i915_fbc_status", i915_fbc_status, 0},
	{"i915_ips_status", i915_ips_status, 0},
	{"i915_sr_status", i915_sr_status, 0},
	{"i915_dpst_status", i915_dpst_status, 0},
	{"i915_opregion", i915_opregion, 0},
	{"i915_gem_framebuffer", i915_gem_framebuffer_info, 0},
	{"i915_context_status", i915_context_status, 0},
	{"i915_dump_lrc", i915_dump_lrc, 0},
	{"i915_execlists", i915_execlists, 0},
	{"i915_gen6_forcewake_count", i915_gen6_forcewake_count_info, 0},
	{"i915_swizzle_info", i915_swizzle_info, 0},
	{"i915_ppgtt_info", i915_ppgtt_info, 0},
	{"i915_llc", i915_llc, 0},
	{"i915_edp_psr_status", i915_edp_psr_status, 0},
	{"i915_sink_crc_eDP1", i915_sink_crc, 0},
	{"i915_energy_uJ", i915_energy_uJ, 0},
	{"i915_pc8_status", i915_pc8_status, 0},
	{"i915_power_domain_info", i915_power_domain_info, 0},
	{"i915_runtime_pm_info", i915_runtime_pm_info, 0},
	{"i915_display_info", i915_display_info, 0},
	{"i915_wa_registers", i915_wa_registers, 0},
	{"i915_scheduler_info", i915_scheduler_info, 0},
};
#define I915_DEBUGFS_ENTRIES ARRAY_SIZE(i915_debugfs_list)

static const struct i915_debugfs_files {
	const char *name;
	const struct file_operations *fops;
} i915_debugfs_files[] = {
	{"i915_wedged", &i915_wedged_fops},
	{"i915_fake_ctx_inconsistency", &i915_fake_ctx_submission_inconsistency_fops},
	{"i915_max_freq", &i915_max_freq_fops},
	{"i915_min_freq", &i915_min_freq_fops},
	{"i915_cur_freq", &i915_cur_freq_fops},
	{"i915_rps_manual", &i915_rps_manual_fops},
	{"i915_rps_disable", &i915_rps_disable_fops},
	{"i915_rps_disable_boost", &i915_rps_disable_boost_fops},
	{"i915_rc6_disable", &i915_rc6_disable_fops},
	{"i915_ips_disable", &i915_ips_disable_fops},
	{"i915_fbc_disable", &i915_fbc_disable_fops},
	{"i915_sr_disable", &i915_sr_disable_fops},
	{"i915_dpst_disable", &i915_dpst_disable_fops},
	{"i915_cache_sharing", &i915_cache_sharing_fops},
	{"i915_ring_stop", &i915_ring_stop_fops},
	{"i915_ring_missed_irq", &i915_ring_missed_irq_fops},
	{"i915_ring_test_irq", &i915_ring_test_irq_fops},
	{"i915_gem_drop_caches", &i915_drop_caches_fops},
	{"i915_ring_hangcheck", &i915_ring_hangcheck_fops},
	{"i915_error_state", &i915_error_state_fops},
	{"i915_next_seqno", &i915_next_seqno_fops},
	{"i915_scheduler_priority_max", &i915_scheduler_priority_max_fops},
	{"i915_scheduler_priority_bump", &i915_scheduler_priority_bump_fops},
	{"i915_scheduler_priority_preempt", &i915_scheduler_priority_preempt_fops},
	{"i915_scheduler_min_flying", &i915_scheduler_min_flying_fops},
	{"i915_scheduler_file_queue_max", &i915_scheduler_file_queue_max_fops},
	{"i915_display_crc_ctl", &i915_display_crc_ctl_fops},
	{"i915_pri_wm_latency", &i915_pri_wm_latency_fops},
	{"i915_spr_wm_latency", &i915_spr_wm_latency_fops},
	{"i915_cur_wm_latency", &i915_cur_wm_latency_fops},
	{"i915_timestamp", &i915_timestamp_fops},
	{"i915_punit_read", &i915_punit_read_fops},
	{"i915_fuse_read", &i915_fuse_read_fops},
	{"i915_connector_reset", &i915_connector_reset_fops},
};

void intel_display_crc_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;

	for_each_pipe(pipe) {
		struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[pipe];

		pipe_crc->opened = false;
		spin_lock_init(&pipe_crc->lock);
		init_waitqueue_head(&pipe_crc->wq);
	}
}

int i915_debugfs_init(struct drm_minor *minor)
{
	int ret, i;

	ret = i915_forcewake_create(minor->debugfs_root, minor);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(i915_pipe_crc_data); i++) {
		ret = i915_pipe_crc_create(minor->debugfs_root, minor, i);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(i915_debugfs_files); i++) {
		ret = i915_debugfs_create(minor->debugfs_root, minor,
					  i915_debugfs_files[i].name,
					  i915_debugfs_files[i].fops);
		if (ret)
			return ret;
	}


	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"display_reg",
					&i915_display_register_fops);
	if (ret)
		return ret;

	return drm_debugfs_create_files(i915_debugfs_list,
					I915_DEBUGFS_ENTRIES,
					minor->debugfs_root, minor);
}

void i915_debugfs_cleanup(struct drm_minor *minor)
{
	int i;

	drm_debugfs_remove_files(i915_debugfs_list,
				 I915_DEBUGFS_ENTRIES, minor);

	drm_debugfs_remove_files((struct drm_info_list *) &i915_forcewake_fops,
				 1, minor);

	for (i = 0; i < ARRAY_SIZE(i915_pipe_crc_data); i++) {
		struct drm_info_list *info_list =
			(struct drm_info_list *)&i915_pipe_crc_data[i];

		drm_debugfs_remove_files(info_list, 1, minor);
	}

	for (i = 0; i < ARRAY_SIZE(i915_debugfs_files); i++) {
		struct drm_info_list *info_list =
			(struct drm_info_list *) i915_debugfs_files[i].fops;

		drm_debugfs_remove_files(info_list, 1, minor);
	}
}
