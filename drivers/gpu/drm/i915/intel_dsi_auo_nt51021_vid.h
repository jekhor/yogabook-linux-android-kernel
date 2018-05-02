/*
 * Copyright (C) 2013 Intel Corporation
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
 * Authors:
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 */
#ifndef __DSI_MOD_AUO_NT51021_H__
#define __DSI_MOD_AUO_NT51021_H__

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"
#if 1
#include "lenovo_lcd_panel.h"

#define DELAY_TYPE		0xff

static struct lcd_cmd cabcoff_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0xC0}, 2},
};

static struct lcd_cmd cabcon_ui_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x80}, 2},
};

static struct lcd_cmd cabcon_still_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x40}, 2},
};

static struct lcd_cmd cabcon_movingi_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x00}, 2},
};


static struct lcd_cmd ceoff_cmds[]  = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x00}, 2},
	{{0x91, 0x20}, 2},
	{{0xbd, 0x36}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x10}, 2},
	{{0x91, 0xa0}, 2},
	{{0x92, 0x20}, 2},
	{{0x93, 0x20}, 2},
	{{0x94, 0x20}, 2},
	{{0x95, 0x20}, 2},
	{{0x96, 0x20}, 2},
	{{0x97, 0x20}, 2},
	{{0x98, 0x97}, 2},  
};

static struct lcd_cmd ceon_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x30}, 2},
	{{0x91, 0xA0}, 2},
	{{0xbd, 0x76}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x1b}, 2},
	{{0x91, 0x91}, 2},
	{{0x92, 0x00}, 2},
	{{0x93, 0x04}, 2},
	{{0x94, 0x06}, 2},
	{{0x95, 0x02}, 2},
	{{0x96, 0x04}, 2},
	{{0x97, 0x07}, 2},
	{{0x98, 0x2f}, 2},  
};

static struct lcd_cmd cabcoff_ceoff_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0xC0}, 2},
	{{0x91, 0x20}, 2},
	{{0xbd, 0x36}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x10}, 2},
	{{0x91, 0xa0}, 2},
	{{0x92, 0x20}, 2},
	{{0x93, 0x20}, 2},
	{{0x94, 0x20}, 2},
	{{0x95, 0x20}, 2},
	{{0x96, 0x20}, 2},
	{{0x97, 0x20}, 2},
	{{0x98, 0x97}, 2},  
};

static struct lcd_cmd cabcon_ceoff_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x40}, 2},
	{{0x91, 0x20}, 2},
	{{0xbd, 0x36}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x10}, 2},
	{{0x91, 0xa0}, 2},
	{{0x92, 0x20}, 2},
	{{0x93, 0x20}, 2},
	{{0x94, 0x20}, 2},
	{{0x95, 0x20}, 2},
	{{0x96, 0x20}, 2},
	{{0x97, 0x20}, 2},
	{{0x98, 0x97}, 2},  


};

static struct lcd_cmd cabcoff_ceon_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0xf0}, 2},
	{{0x91, 0xA0}, 2},
	{{0xbd, 0x76}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x1b}, 2},
	{{0x91, 0x91}, 2},
	{{0x92, 0x00}, 2},
	{{0x93, 0x04}, 2},
	{{0x94, 0x06}, 2},
	{{0x95, 0x02}, 2},
	{{0x96, 0x04}, 2},
	{{0x97, 0x07}, 2},
	{{0x98, 0x2f}, 2},

};

static struct lcd_cmd cabcon_ceon_cmds[] = {
	{{0x83, 0xbb}, 2},
	{{0x84, 0x22}, 2},
	{{0x90, 0x70}, 2},
	{{0x91, 0xA0}, 2},
	{{0xbd, 0x76}, 2},
	{{0x83, 0xcc}, 2},
	{{0x84, 0x33}, 2},
	{{0x90, 0x1b}, 2},
	{{0x91, 0x91}, 2},
	{{0x92, 0x00}, 2},
	{{0x93, 0x04}, 2},
	{{0x94, 0x06}, 2},
	{{0x95, 0x02}, 2},
	{{0x96, 0x04}, 2},
	{{0x97, 0x07}, 2},
	{{0x98, 0x2f}, 2},

};


static struct lcd_effect_cmd cabc_and_ce_effect_cmds[] = {
	{ ARRAY_SIZE(cabcoff_ceoff_cmds), cabcoff_ceoff_cmds},
	{ ARRAY_SIZE(cabcon_ceoff_cmds), cabcon_ceoff_cmds},
	{ ARRAY_SIZE(cabcoff_ceon_cmds), cabcoff_ceon_cmds},
	{ ARRAY_SIZE(cabcon_ceon_cmds), cabcon_ceon_cmds},
};

static struct lcd_effect_cmd cabc_effect_cmds[] = {
	{ ARRAY_SIZE(cabcoff_cmds), cabcoff_cmds},
	{ ARRAY_SIZE(cabcon_ui_cmds), cabcon_ui_cmds},
	{ ARRAY_SIZE(cabcon_still_cmds), cabcon_still_cmds},
	{ ARRAY_SIZE(cabcon_movingi_cmds), cabcon_movingi_cmds},
};

static struct lcd_effect_cmd ce_effect_cmds[] = {
	{ARRAY_SIZE(ceoff_cmds), ceoff_cmds},
	{ARRAY_SIZE(ceon_cmds), ceon_cmds},
};

static struct lcd_mode_cmd standard_mode_cmds[] = {
    {ARRAY_SIZE(cabcon_ceon_cmds), cabcon_ceon_cmds},
};
static struct lcd_mode_cmd camera_mode_cmds[] = {
    {ARRAY_SIZE(cabcoff_ceoff_cmds), cabcoff_ceoff_cmds},
};

static struct lcd_mode auo_nt51021_mode[] = {
    {"standard",  standard_mode_cmds,1},
    {"camera",  camera_mode_cmds,1},
};
static struct lcd_mode_data auo_nt51021_mode_data = { auo_nt51021_mode, ARRAY_SIZE(auo_nt51021_mode)};

static struct lcd_effect auo_nt51021_effect[] = {
	{"cabc", ARRAY_SIZE(cabc_effect_cmds), 0, cabc_effect_cmds},
	{"cabc_and_ce", ARRAY_SIZE(cabc_and_ce_effect_cmds), 0, cabc_and_ce_effect_cmds},
	{"ce", ARRAY_SIZE(ce_effect_cmds), 1, ce_effect_cmds},
};

static struct lcd_effect_data auo_nt51021_effect_data = { auo_nt51021_effect, ARRAY_SIZE(auo_nt51021_effect)};

static struct lcd_data auo_nt51021_data = {
	&auo_nt51021_effect_data,
	&auo_nt51021_mode_data,
	//NULL,
};
#endif
#endif /* __DSI_MOD_AUO_NT51021_H__ */
