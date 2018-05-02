/*
 * Copyright 2013 Intel Corporation
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
 */
#ifndef _INTEL_ESD_HANDLE_H
#define _INTEL_ESD_HANDLE_H
#include <linux/spinlock.h>

//#define FEATURE_ESD_RESET
#define ESD_IRQ_GPIO_NUM                334 /*ISH_GPIO_9 used for hang detect,check with hardware design*/
#define ESD_GPIO_INT_NAME               "DSI ESD Hange IRQ"

/*gpio config address, allign with gpio num*/
#define CHV_GPIO_TE_CFG0		0x4828
#define CHV_GPIO_TE_CFG1		0x482c

enum intel_esd_state {
        DISPLAY_DSI_STATE_OK,
        DISPLAY_DSI_STATE_HANG,
        DISPLAY_DSI_STATE_DONE,
};

enum intel_esd_reset_flasg {
        ESD_NONE,
        ESD_DEBUG_RESET,
        ESD_IRQ_RESET,
};

 struct esd_private {
        int esd_gpio_irq_mapped;
       enum intel_esd_state dsi_esd_state;
       enum intel_esd_reset_flasg reset_flasg;
       int flip_fail_count;
        struct drm_connector *esd_connector;
        struct work_struct esd_work;
        struct delayed_work esd_delay_work;
       struct mutex reset_mutex;
 };

void intel_dsi_esd_init(struct drm_connector *connector);

void intel_dsi_esd_destroy(struct drm_connector *connector);

void intel_dsi_esd_post_dpms_handler(struct drm_connector *connector,
	int mode);

#endif /*end of _INTEL_ESD_HANDLE_H*/
