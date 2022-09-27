// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *
 * Samsung DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>

#include "../exynos_drm_dp.h"

static void __iomem *clk_regs1 = NULL;
static void __iomem *clk_regs2 = NULL;
static void __iomem *clk_regs3 = NULL;

void dp_enable_dposc(struct dp_device *dp)
{
	if (dp->res.dposc_clk) {
		clk_prepare_enable(dp->res.dposc_clk);
		dp_info(dp, "DPOSC in CLK(%ld)\n", clk_get_rate(dp->res.dposc_clk));
		if (clk_get_rate(dp->res.dposc_clk) != 40000000)
			WARN_ON(1);
	} else {
		if (clk_regs1 == NULL) {
			clk_regs1 = ioremap((phys_addr_t)0x11000670, 0x10);
			if (!clk_regs1) {
				pr_err("HSI0_CLK SFR ioremap is failed\n");
				return;
			}
		} else
			pr_info("Already has clk_regs1\n");

		if (clk_regs2 == NULL) {
			clk_regs2 = ioremap((phys_addr_t)0x26041888, 0x10);
			if (!clk_regs2) {
				pr_err("HSI0_CLK SFR ioremap is failed\n");
				return;
			}
		} else
			pr_info("Already has clk_regs2\n");

		if (clk_regs3 == NULL) {
			clk_regs3 = ioremap((phys_addr_t)0x26041090, 0x10);
			if (!clk_regs3) {
				pr_err("HSI0_CLK SFR ioremap is failed\n");
				return;
			}
		} else
			pr_info("Already has clk_regs3\n");

		writel(0x10, clk_regs1);
		writel(0x9, clk_regs2);
		writel(0x1, clk_regs3);
		pr_info("MUX_CLKCMU_HSI0_DPOSC_USER[0x1100_0670](0x%08x)\n", readl(clk_regs1));
		pr_info("DIV_CLKCMU_HSI0_DPOSC[0x2604_1888](0x%08x)\n", readl(clk_regs2));
		pr_info("MUX_CLKCMU_HSI0_DPOSC[0x2604_1090](0x%08x)\n", readl(clk_regs3));
	}
}

void dp_disable_dposc(struct dp_device *dp)
{
	if (dp->res.dposc_clk)
		clk_disable_unprepare(dp->res.dposc_clk);
}

int dp_get_clock(struct dp_device *dp)
{
	int ret = 0;

	dp->res.dposc_clk = devm_clk_get(dp->dev, "dposc_clk");
	if (IS_ERR_OR_NULL(dp->res.dposc_clk)) {
		dp_info(dp, "failed to get dposc_clk(optional)\n");
		dp->res.dposc_clk = NULL;
		//ret = -EINVAL;
		return 0;
	}

	if (ret == 0)
		dp_info(dp, "Success to get dp clocks resources\n");

	return ret;
}
