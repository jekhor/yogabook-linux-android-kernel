/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef	__ATOMISP_FOPS_H__
#define	__ATOMISP_FOPS_H__
#include "atomisp_subdev.h"

int atomisp_q_video_buffers_to_css(struct atomisp_sub_device *asd,
			     struct atomisp_video_pipe *pipe,
			     enum atomisp_input_stream_id stream_id,
			     enum atomisp_css_buffer_type css_buf_type,
			     enum atomisp_css_pipe_id css_pipe_id);

int atomisp_q_s3a_buffers_to_css(struct atomisp_sub_device *asd,
			   enum atomisp_input_stream_id stream_id,
			   enum atomisp_css_pipe_id css_pipe_id);

int atomisp_q_dis_buffers_to_css(struct atomisp_sub_device *asd,
			   enum atomisp_input_stream_id stream_id,
			   enum atomisp_css_pipe_id css_pipe_id);

unsigned int atomisp_dev_users(struct atomisp_device *isp);
unsigned int atomisp_sub_dev_users(struct atomisp_sub_device *asd);

/*
 * Memory help functions for image frame and private parameters
 */

int atomisp_videobuf_mmap_mapper(struct videobuf_queue *q,
				     struct vm_area_struct *vma);

int atomisp_qbuf_to_css(struct atomisp_device *isp,
			struct atomisp_video_pipe *pipe,
			struct videobuf_buffer *vb);

int atomisp_qbuffers_to_css(struct atomisp_sub_device *asd);

extern const struct v4l2_file_operations atomisp_fops;

#endif /* __ATOMISP_FOPS_H__ */
