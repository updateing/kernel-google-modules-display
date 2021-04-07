/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS201 DQE CAL INTERNAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DQE_CAL_INTERNAL_H__
#define __SAMSUNG_DQE_CAL_INTERNAL_H__

#ifdef CONFIG_SOC_GS201
void dqe_reg_set_rcd_en_internal(u32 id, bool en);
#else
static inline void dqe_reg_set_rcd_en_internal(u32 id, bool en) {}
#endif

#endif /* __SAMSUNG_DPP_CAL_INTERNAL_H__ */
