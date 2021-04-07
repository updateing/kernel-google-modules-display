// SPDX-License-Identifier: GPL-2.0-only
/*
 * cal_9855/dqe_regs.c
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <dqe_cal.h>

#include "regs-dqe.h"
#include "dqe_cal_internal.h"

void dqe_reg_set_rcd_en_internal(u32 dqe_id, bool en)
{
	dqe_write(dqe_id, DQE_RCD, DQE_RCD_EN(en ? 1 : 0));
}
