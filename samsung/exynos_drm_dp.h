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
#include <drm/drm_dp_helper.h>
#include <linux/extcon.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"

#include "dp_cal.h"


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

struct dp_link {
	u8  revision;
	u32  link_rate;
	u8  num_lanes;
	bool enhanced_frame;
};

struct dp_host {
	u32 link_rate;
	u8   num_lanes;
	u8   support_tps;
	bool fast_training;
	bool enhanced_frame;
	bool ssc;
	bool scrambler;

	/* Link Training */
	u8  volt_swing_max;
	u8  pre_emphasis_max;
	u8  vol_swing_level[MAX_LANE_CNT];
	u8  pre_empha_level[MAX_LANE_CNT];
	u8  max_reach_value[MAX_LANE_CNT];
};

struct dp_sink {
	u32  link_rate;
	u8   num_lanes;
	u8   support_tps;
	bool fast_training;
	bool enhanced_frame;
	bool ssc;
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
	struct drm_dp_aux dp_aux;

	enum exynos_drm_output_type output_type;

	struct device *dev;
	struct dp_resources res;

	struct workqueue_struct *dp_wq;
	struct delayed_work hpd_plug_work;
	struct delayed_work hpd_unplug_work;

	struct mutex hpd_lock;
	struct mutex training_lock;

	/* HPD State */
	enum hotplug_state hpd_current_state;
	struct mutex hpd_state_lock;

	/* DP Driver State */
	enum dp_state state;

	/* DP Capabilities */
	struct dp_link link;
	struct dp_host host;
	struct dp_sink sink;

	/* PDIC / ExtCon */
	struct extcon_dev *edev;
	struct notifier_block dp_typec_nb;
	int notifier_registered;

	/* DP HW Configurations */
	struct dp_hw_config hw_config;
};

static inline struct dp_device *get_dp_drvdata(void)
{
	return dp_drvdata;
}

#endif // __EXYNOS_DRM_DP_H__
