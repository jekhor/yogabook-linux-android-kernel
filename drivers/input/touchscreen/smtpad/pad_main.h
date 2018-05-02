/*
 * Copyright (c) 2014 Lenovo
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __PAD_MAIN_H
#define __PAD_MAIN_H

struct pad_pointer{
	int finger;
	int x;
	int y;
};

int notify_touch_event(int index,int x,int y,int w);

#endif
