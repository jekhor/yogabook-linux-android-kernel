/*
 * Support for dw9719 vcm driver.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
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

#ifndef __WV517_H__
#define __WV517_H__

#include <linux/atomisp_platform.h>
#include <linux/types.h>

#define WV517_VCM_ADDR	 (0x18 >> 1)

/* wv517 device structure */
struct wv517_device {
	struct timespec timestamp_t_focus_abs;
	s16 number_of_steps;
	bool initialized;		/* true if wv517 is detected */
	s32 focus;			/* Current focus value */
	struct timespec focus_time;	/* Time when focus was last time set */
	__u8 buffer[4];			/* Used for i2c transactions */
	const struct camera_af_platform_data *platform_data;
};

#define WV517_MAX_FOCUS_POS	1023

/* Register addresses */
#define WV517_DIRECT_DAC			0x00
#define WV517_CONTROL			0x01
#define WV517_DATA_M			0x41
#define WV517_DATA_L			0x42
#define WV517_SW			0x04
#define WV517_SACT			0x05
#define WV517_FLAG			0x10
#define WV517_DSC_M			0x43
#define WV517_DSC_L			0x44

#define WV517_CONTROL_SW_LINEAR	BIT(0)
#define WV517_CONTROL_S_SAC4		(BIT(1) | BIT(3))
#define WV517_CONTROL_OCP_DISABLE	BIT(4)
#define WV517_CONTROL_UVLO_DISABLE	BIT(5)

#define WV517_SACT_MULT_TWO		0x00
#define WV517_SACT_PERIOD_8_8MS	0x19
#define WV517_SACT_DEFAULT_VAL		0x60

#define WV517_CLICK_REDUCTION_STEP	 30 /* in vcm units */
#define WV517_CLICK_REDUCTION_SLEEP	 20 /* in milliseconds */
#define WV517_LENS_MOVE_POSITION	120 /* in vcm units */
#define WV517_DEFAULT_FOCUS_POSITION	300 /* in vcm units */

#endif /* __WV517_H__ */
