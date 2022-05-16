/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Samsung DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DP_H__
#define __EXYNOS_DRM_DP_H__

#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <linux/extcon.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"

int get_dp_log_level(void);

#define dp_info(dp, fmt, ...)	\
pr_info("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_warn(dp, fmt, ...)	\
pr_warn("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_err(dp, fmt, ...)	\
pr_err("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

#define dp_debug(dp, fmt, ...)	\
pr_debug("%s: "fmt, dp->dev->driver->name, ##__VA_ARGS__)

extern struct dp_device *dp_drvdata;

enum dp_state {
	DP_STATE_INIT,
	DP_STATE_ON,
	DP_STATE_RUN,
};

enum hotplug_state {
	EXYNOS_HPD_UNPLUG = 0,
	EXYNOS_HPD_PLUG,
	EXYNOS_HPD_IRQ,
};

struct dp_resources {
	int aux_ch_mux_gpio;
	int irq;
	void __iomem *link_regs;
	void __iomem *phy_regs;
};

struct dp_device {
	struct drm_encoder encoder;
	struct drm_connector connector;

	enum exynos_drm_output_type output_type;

	struct device *dev;
	struct dp_resources res;

	struct mutex hpd_lock;

	/* HPD State */
	enum hotplug_state hpd_current_state;
	struct mutex hpd_state_lock;

	/* DP Driver State */
	enum dp_state state;

	/* PDIC / ExtCon */
	struct extcon_dev *edev;
	struct notifier_block dp_typec_nb;
	int notifier_registered;
};

static inline struct dp_device *get_dp_drvdata(void)
{
	return dp_drvdata;
}

#endif // __EXYNOS_DRM_DP_H__
