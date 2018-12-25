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

#ifndef __STORAGE_CLASS_H_INCLUDED__
#define __STORAGE_CLASS_H_INCLUDED__

#define STORAGE_CLASS_EXTERN extern

#if defined(_MSC_VER)
#define STORAGE_CLASS_INLINE static __inline
#elif defined(__HIVECC)
#define STORAGE_CLASS_INLINE static inline
#else
#define STORAGE_CLASS_INLINE static inline
#endif

#endif /* __STORAGE_CLASS_H_INCLUDED__ */
