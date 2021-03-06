/**
 * Copyright (c) 2010 - 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
**/

#ifndef __ADDR_SUPPORT_H_INCLUDED__
#define __ADDR_SUPPORT_H_INCLUDED__

/*
 * Compute the byte address offset of a struct or array member from the base struct base
 *
 * Note:
 *	- The header defining the type "T" must be included
 *	- This macro works on all cells, but not necessarily with the same output
 *	- Works for arrays, structs, arrays of structs, arrays in structs etc.
 */
#define offsetof(T, x) ((unsigned)&(((T *)0)->x))
#define OFFSET_OF(T, x) offsetof(T, x)

#endif /* __ADDR_SUPPORT_H_INCLUDED__ */
