/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS201 DPP CAL INTERNAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DPP_CAL_INTERNAL_H__
#define __SAMSUNG_DPP_CAL_INTERNAL_H__

#ifdef CONFIG_SOC_GS201
void rcd_reg_init(u32 id);
int rcd_reg_deinit(u32 id, bool reset, const unsigned long attr);
void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr);
#else
static inline void rcd_reg_init(u32 id) {}
static inline void rcd_reg_deinit(u32 id, bool reset, const unsigned long attr) {}
static inline void rcd_reg_configure_params(u32 id, struct dpp_params_info *p,
		const unsigned long attr) {}
#endif

#endif /* __SAMSUNG_DPP_CAL_INTERNAL_H__ */
