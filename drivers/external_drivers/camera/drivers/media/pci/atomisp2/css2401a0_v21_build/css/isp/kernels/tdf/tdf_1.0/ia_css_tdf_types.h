/**
Support for Intel Camera Imaging ISP subsystem.
Copyright (c) 2010 - 2015, Intel Corporation.

This program is free software; you can redistribute it and/or modify it
under the terms and conditions of the GNU General Public License,
version 2, as published by the Free Software Foundation.

This program is distributed in the hope it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
more details.
*/

#ifndef __IA_CSS_TDF_TYPES_H
#define __IA_CSS_TDF_TYPES_H

/** @file
* CSS-API header file for Transform Domain Filter parameters.
*/

#include "type_support.h"

/** Transform Domain Filter configuration
 *
 * \brief TDF public parameters.
 * \details Struct with all parameters for the TDF kernel that can be set
 * from the CSS API.
 *
 * ISP2.6.1: TDF is used.
 */
struct ia_css_tdf_config {
	int32_t thres_flat_table[64];	/**< Final optimized strength table of NR for flat region. */
	int32_t thres_detail_table[64];	/**< Final optimized strength table of NR for detail region. */
	int32_t epsilon_0;		/**< Coefficient to control variance for dark area (for flat region). */
	int32_t epsilon_1;		/**< Coefficient to control variance for bright area (for flat region). */
	int32_t eps_scale_text;		/**< Epsilon scaling coefficient for texture region. */
	int32_t eps_scale_edge;		/**< Epsilon scaling coefficient for edge region. */
	int32_t sepa_flat;		/**< Threshold to judge flat (edge < m_Flat_thre). */
	int32_t sepa_edge;		/**< Threshold to judge edge (edge > m_Edge_thre). */
	int32_t blend_flat;		/**< Blending ratio at flat region. */
	int32_t blend_text;		/**< Blending ratio at texture region. */
	int32_t blend_edge;		/**< Blending ratio at edge region. */
	int32_t shading_gain;		/**< Gain of Shading control. */
	int32_t shading_base_gain;	/**< Base Gain of Shading control. */
	int32_t local_y_gain;		/**< Gain of local luminance control. */
	int32_t local_y_base_gain;	/**< Base gain of local luminance control. */
	int32_t rad_x_origin;		/**< Initial x coord. for radius computation. */
	int32_t rad_y_origin;		/**< Initial y coord. for radius computation. */
};

#endif /* __IA_CSS_TDF_TYPES_H */

