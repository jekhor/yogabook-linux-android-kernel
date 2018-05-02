/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2010 - 2015 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */

#ifndef __IA_CSS_MMU_PRIVATE_H
#define __IA_CSS_MMU_PRIVATE_H

#include "system_local.h"

/*
 * This function sets the L1 pagetable address.
 * After power-up of the ISP the L1 pagetable can be set.
 * Once being set the L1 pagetable is protected against
 * further modifications.
 */
void
sh_css_mmu_set_page_table_base_index(hrt_data base_index);

#endif /* __IA_CSS_MMU_PRIVATE_H */
