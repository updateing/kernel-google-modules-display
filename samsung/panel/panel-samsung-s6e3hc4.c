// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3hc4 AMOLED LCD panel driver.
 *
 * Copyright (c) 2021 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drm_vblank.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>
#include <trace/dpu_trace.h>

#include "panel-samsung-drv.h"

/**
 * enum s6e3hc4_panel_feature - features supported by this panel
 * @FEAT_HBM: high brightness mode
 * @FEAT_IRC_OFF: IRC compensation off state
 * @FEAT_EARLY_EXIT: early exit from a long frame
 * @FEAT_OP_NS: normal speed (not high speed)
 * @FEAT_FRAME_AUTO: automatic (not manual) frame control
 * @FEAT_MAX: placeholder, counter for number of features
 *
 * The following features are correlated, if one or more of them change, the others need
 * to be updated unconditionally.
 */
enum s6e3hc4_panel_feature {
	FEAT_HBM = 0,
	FEAT_IRC_OFF,
	FEAT_EARLY_EXIT,
	FEAT_OP_NS,
	FEAT_FRAME_AUTO,
	FEAT_MAX,
};

/**
 * struct s6e3hc4_panel - panel specific runtime info
 *
 * This struct maintains s6e3hc4 panel specific runtime info, any fixed details about panel should
 * most likely go into struct exynos_panel_desc. The variables with the prefix hw_ keep track of the
 * features that were actually committed to hardware, and should be modified after sending cmds to panel,
 * i.e. updating hw state.
 */
struct s6e3hc4_panel {
	/** @base: base panel struct */
	struct exynos_panel base;
	/** @feat: software or working correlated features, not guaranteed to be effective in panel */
	DECLARE_BITMAP(feat, FEAT_MAX);
	/** @hw_feat: correlated states effective in panel */
	DECLARE_BITMAP(hw_feat, FEAT_MAX);
	/** @hw_vrefresh: vrefresh rate effective in panel */
	u32 hw_vrefresh;
	/** @hw_idle_vrefresh: idle vrefresh rate effective in panel */
	u32 hw_idle_vrefresh;
	/**
	 * @auto_mode_vrefresh: indicates current minimum refresh rate while in auto mode,
	 *			if 0 it means that auto mode is not enabled
	 */
	u32 auto_mode_vrefresh;
	/** @force_changeable_te: force changeable TE (instead of fixed) during early exit */
	bool force_changeable_te;
};

#define to_spanel(ctx) container_of(ctx, struct s6e3hc4_panel, base)

static const unsigned char PPS_SETTING[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0C, 0x30,
	0x05, 0xA0, 0x00, 0x34, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x05, 0xC6,
	0x00, 0x0A, 0x00, 0x0C, 0x01, 0xE2, 0x01, 0x78,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define S6E3HC4_WRCTRLD_DIMMING_BIT    0x08
#define S6E3HC4_WRCTRLD_BCTRL_BIT      0x20
#define S6E3HC4_WRCTRLD_HBM_BIT        0xC0
#define S6E3HC4_WRCTRLD_LOCAL_HBM_BIT  0x10

#define S6E3HC4_TE2_CHANGEABLE 0x03
#define S6E3HC4_TE2_FIXED      0x21

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[]   = { 0xF0, 0xA5, 0xA5 };
static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 sleep_in[] = { 0x10 };
static const u8 freq_update[] = { 0xF7, 0x0F };

static const struct exynos_dsi_cmd s6e3hc4_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 20),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFC, 0x5A, 0x5A),
	EXYNOS_DSI_CMD(sleep_in, 100),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xB0, 0x00, 0x0D, 0xFE),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFE, 0x02),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFC, 0xA5, 0xA5),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc4_off);

static const struct exynos_dsi_cmd s6e3hc4_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 17),
	EXYNOS_DSI_CMD0(unlock_cmd_f0),

	/* changeable TE: sync on*/
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x00),

	/* enable fast exit */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x01, 0x60),
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00),	/* 30Hz */
	EXYNOS_DSI_CMD0(lock_cmd_f0),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc4_lp);

static const struct exynos_dsi_cmd s6e3hc4_lp_off_cmds[] = {
	EXYNOS_DSI_CMD0(display_off),
	EXYNOS_DSI_CMD0(unlock_cmd_f0),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x52, 0x94), /* global para */
	EXYNOS_DSI_CMD_SEQ(0x94, 0x00), /* AOD low mode off */
	EXYNOS_DSI_CMD_SEQ(0x53, 0x20),
	EXYNOS_DSI_CMD0(lock_cmd_f0),
};

static const struct exynos_dsi_cmd s6e3hc4_lp_low_cmds[] = {
	EXYNOS_DSI_CMD0(unlock_cmd_f0),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x01, 0x60), /* global para */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00), /* freq set */
	EXYNOS_DSI_CMD_SEQ(0x53, 0x25),	/* aod 10 nit */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x52, 0x94), /* global para */
	EXYNOS_DSI_CMD_SEQ(0x94, 0x01, 0x07, 0x98, 0x02), /* AOD low Mode, 10 nit */
	EXYNOS_DSI_CMD0(lock_cmd_f0),
	EXYNOS_DSI_CMD0(display_on)
};

static const struct exynos_dsi_cmd s6e3hc4_lp_high_cmds[] = {
	EXYNOS_DSI_CMD0(unlock_cmd_f0),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x01, 0x60), /* global para */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00), /* freq set */
	EXYNOS_DSI_CMD_SEQ(0x53, 0x24),	/* aod 50 nit */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x52, 0x94), /* global para */
	EXYNOS_DSI_CMD_SEQ(0x94, 0x00, 0x07, 0x98, 0x02), /* AOD high Mode, 50 nit */
	EXYNOS_DSI_CMD0(lock_cmd_f0),
	EXYNOS_DSI_CMD0(display_on)
};

static const struct exynos_binned_lp s6e3hc4_binned_lp[] = {
	BINNED_LP_MODE("off", 0, s6e3hc4_lp_off_cmds),
	/* rising time = 16, falling time = 48 */
	BINNED_LP_MODE_TIMING("low", 80, s6e3hc4_lp_low_cmds, 16, 48),
	BINNED_LP_MODE_TIMING("high", 2047, s6e3hc4_lp_high_cmds, 16, 48)
};

static u8 s6e3hc4_get_te2_option(struct exynos_panel *ctx)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);

	if (!ctx || !ctx->current_mode)
		return S6E3HC4_TE2_CHANGEABLE;

	if (ctx->current_mode->exynos_mode.is_lp_mode ||
	    (test_bit(FEAT_EARLY_EXIT, spanel->feat)))
		return S6E3HC4_TE2_FIXED;

	return S6E3HC4_TE2_CHANGEABLE;
}

static void s6e3hc4_update_te2(struct exynos_panel *ctx)
{
	struct exynos_panel_te2_timing timing;
	u32 rising, falling;
	u8 option = s6e3hc4_get_te2_option(ctx);
	int ret;

	if (!ctx)
		return;

	ret = exynos_panel_get_current_mode_te2(ctx, &timing);
	if (!ret) {
		rising = timing.rising_edge;
		falling = timing.falling_edge;
		if (option == S6E3HC4_TE2_FIXED) {
			/* fixed TE2 has 2H shift */
			rising += 2;
			falling += 2;
		}
	} else if (ret == -EAGAIN) {
		dev_dbg(ctx->dev, "Panel is not ready, use default setting\n");
	} else {
		return;
	}

	dev_dbg(ctx->dev,
		"TE2 updated: option %s, idle %s, rising=0x%x falling=0x%x\n",
		(option == S6E3HC4_TE2_CHANGEABLE) ? "changeable" : "fixed",
		ctx->panel_idle_vrefresh ? "active" : "inactive",
		rising, falling);

	EXYNOS_DCS_BUF_ADD_SET(ctx, unlock_cmd_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x42, 0xF2);
	EXYNOS_DCS_BUF_ADD(ctx, 0xF2, 0x0D);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x01, 0xB9);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB9, option);
	if (option == S6E3HC4_TE2_FIXED) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x2E, 0xB9);
		EXYNOS_DCS_BUF_ADD(ctx, (rising >> 8) & 0xF, rising & 0xFF,
			(rising >> 8) & 0xF, rising & 0xFF,
			(falling >> 8) & 0xF, falling & 0xFF);
	} else {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x1E, 0xB9);
		EXYNOS_DCS_BUF_ADD(ctx, (rising >> 8) & 0xF, rising & 0xFF,
			(falling >> 8) & 0xF, falling & 0xFF);
	}

	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, lock_cmd_f0);
}

static inline bool is_auto_mode_allowed(struct exynos_panel *ctx)
{
	/* don't want to enable auto mode/early exit during hbm or dimming on */
	if (IS_HBM_ON(ctx->hbm_mode) || ctx->dimming_on)
		return false;

	if (ctx->idle_delay_ms) {
		const unsigned int delta_ms = panel_get_idle_time_delta(ctx);

		if (delta_ms < ctx->idle_delay_ms)
			return false;
	}

	return ctx->panel_idle_enabled;
}

static u32 s6e3hc4_get_min_idle_vrefresh(struct exynos_panel *ctx,
					const struct exynos_panel_mode *pmode)
{
	const int vrefresh = drm_mode_vrefresh(&pmode->mode);
	int min_idle_vrefresh = ctx->min_vrefresh;

	if ((min_idle_vrefresh < 0) || !is_auto_mode_allowed(ctx))
		return 0;

	if (min_idle_vrefresh <= 10)
		min_idle_vrefresh = 10;
	else if (min_idle_vrefresh <= 30)
		min_idle_vrefresh = 30;
	else if (min_idle_vrefresh <= 60)
		min_idle_vrefresh = 60;
	else
		return 0;

	if (min_idle_vrefresh >= vrefresh) {
		dev_dbg(ctx->dev, "min idle vrefresh (%d) higher than target (%d)\n",
				min_idle_vrefresh, vrefresh);
		return 0;
	}

	return min_idle_vrefresh;
}

static void s6e3hc4_update_panel_feat(struct exynos_panel *ctx,
	const struct exynos_panel_mode *pmode, bool enforce)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);
	u32 vrefresh, idle_vrefresh = spanel->auto_mode_vrefresh;
	u8 val;
	DECLARE_BITMAP(changed_feat, FEAT_MAX);

	if (pmode)
		vrefresh = drm_mode_vrefresh(&pmode->mode);
	else
		vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	if (enforce) {
		bitmap_fill(changed_feat, FEAT_MAX);
	} else {
		bitmap_xor(changed_feat, spanel->feat, spanel->hw_feat, FEAT_MAX);
		if (bitmap_empty(changed_feat, FEAT_MAX) &&
			vrefresh == spanel->hw_vrefresh &&
			idle_vrefresh == spanel->hw_idle_vrefresh)
			return;
	}

	spanel->hw_vrefresh = vrefresh;
	spanel->hw_idle_vrefresh = idle_vrefresh;
	bitmap_copy(spanel->hw_feat, spanel->feat, FEAT_MAX);
	dev_dbg(ctx->dev,
		"op=%s ee=%s hbm=%s irc=%s fi=%s fps=%u idle_fps=%u\n",
		test_bit(FEAT_OP_NS, spanel->feat) ? "ns" : "hs",
		test_bit(FEAT_EARLY_EXIT, spanel->feat) ? "on" : "off",
		test_bit(FEAT_HBM, spanel->feat) ? "on" : "off",
		test_bit(FEAT_IRC_OFF, spanel->feat) ? "off" : "on",
		test_bit(FEAT_FRAME_AUTO, spanel->feat) ? "auto" : "manual",
		vrefresh,
		idle_vrefresh);

	EXYNOS_DCS_BUF_ADD_SET(ctx, unlock_cmd_f0);

	/* TE setting */
	if (test_bit(FEAT_EARLY_EXIT, changed_feat)) {
		if (test_bit(FEAT_EARLY_EXIT, spanel->feat) && !spanel->force_changeable_te) {
			/* Fixed TE */
			EXYNOS_DCS_BUF_ADD(ctx, 0xB9, 0x51);
			/* TE width setting */
			EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x08, 0xB9);
			EXYNOS_DCS_BUF_ADD(ctx, 0xB9, 0x07, 0xBA, 0x07, 0xBC,
				0x07, 0xBA, 0x07, 0xBC);
		} else {
			/* Changeable TE */
			EXYNOS_DCS_BUF_ADD(ctx, 0xB9, 0x00);
		}
	}

	/* IRC setting */
	if (test_bit(FEAT_IRC_OFF, changed_feat)) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0x9B, 0x92);
		val = test_bit(FEAT_IRC_OFF, spanel->feat) ? 0x01 : 0x21;
		EXYNOS_DCS_BUF_ADD(ctx, 0x92, val);
	}

	/*
	 * Operating Mode: NS or HS
	 *
	 * Description: the configs could possibly be overrided by frequency setting,
	 * depending on FI mode.
	 */
	if (test_bit(FEAT_OP_NS, changed_feat)) {
		/* mode set */
		EXYNOS_DCS_BUF_ADD(ctx, 0xF2, 0x01);
		val = test_bit(FEAT_OP_NS, spanel->feat) ? 0x18 : 0x00;
		EXYNOS_DCS_BUF_ADD(ctx, 0x60, val);
	}

	/*
	 * Note: the following command sequence should be sent as a whole if one of panel
	 * state defined by enum panel_state changes or at turning on panel, or unexpected
	 * behaviors will be seen, e.g. black screen, flicker.
	 */

	/*
	 * Early-exit: enable or disable
	 *
	 * Description: early-exit sequence overrides some configs HBM set.
	 */
	if (test_bit(FEAT_EARLY_EXIT, spanel->feat)) {
		if (test_bit(FEAT_HBM, spanel->feat))
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x21, 0x00, 0x83, 0x03, 0x01);
		else
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x21, 0x01, 0x83, 0x03, 0x03);
	} else {
		if (test_bit(FEAT_HBM, spanel->feat))
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x21, 0x80, 0x03, 0x03, 0x01);
		else
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x21, 0x81, 0x83, 0x03, 0x03);
	}
	if (test_bit(FEAT_EARLY_EXIT, spanel->feat) ||
		!test_bit(FEAT_HBM, spanel->feat)) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x0C, 0xBD);
		EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x00, 0x00);
	}
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x10, 0xBD);
	val = test_bit(FEAT_EARLY_EXIT, spanel->feat) ? 0x10 : 0x00;
	EXYNOS_DCS_BUF_ADD(ctx, 0xBD, val);
	if (test_bit(FEAT_HBM, spanel->feat)) {
		if (test_bit(FEAT_OP_NS, spanel->feat)) {
			/* suppose that frequency = 60 */
			EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x4E, 0xBD);
			EXYNOS_DCS_BUF_ADD(ctx,
				0xBD, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x00, 0x14);
		} else {
			/* suppose that frequency = 120 */
			EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x1E, 0xBD);
			EXYNOS_DCS_BUF_ADD(ctx,
				0xBD, 0x00, 0x00, 0x00, 0x02, 0x00, 0x06, 0x00, 0x16);
		}
	}

	/*
	 * Frequency setting: FI, frequency, idle frequency
	 *
	 * Description: this sequence possibly overrides some configs early-exit
	 * and operation set, depending on FI mode.
	 */
	if (test_bit(FEAT_FRAME_AUTO, spanel->feat)) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x0C, 0xBD);
		EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x00, 0x00);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x12, 0xBD);
		if (test_bit(FEAT_OP_NS, spanel->feat)) {
			if (idle_vrefresh == 10)
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBD, 0x00, 0x00, 0x12, 0x00, 0x02, 0x01);
			/* idle_vrefresh == 30 */
			else
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBD, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00);
		} else {
			if (idle_vrefresh == 10)
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBD, 0x00, 0x00, 0x16, 0x00, 0x06, 0x01);
			else if (idle_vrefresh == 30)
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBD, 0x00, 0x00, 0x06, 0x00, 0x02, 0x01);
			/* idle_vrefresh == 60 */
			else
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBD, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00);
		}
		EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x23);
	} else {
		EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x21);
		if (test_bit(FEAT_OP_NS, spanel->feat)) {
			if (vrefresh == 10)
				val = 0x1B;
			else if (vrefresh == 30)
				val = 0x19;
			else
				val = 0x18;
		} else {
			if (vrefresh == 10)
				val = 0x03;
			else if (vrefresh == 30)
				val = 0x02;
			else if (vrefresh == 60)
				val = 0x01;
			else
				val = 0x00;
		}
		EXYNOS_DCS_BUF_ADD(ctx, 0x60, val);
	}

	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, lock_cmd_f0);;
}

static void s6e3hc4_update_refresh_mode(struct exynos_panel *ctx,
					const struct exynos_panel_mode *pmode,
					const u32 idle_vrefresh)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);

	dev_dbg(ctx->dev, "%s: mode: %s set idle_vrefresh: %u\n", __func__,
		pmode->mode.name, idle_vrefresh);

	if (idle_vrefresh) {
		set_bit(FEAT_EARLY_EXIT, spanel->feat);
		set_bit(FEAT_FRAME_AUTO, spanel->feat);
	} else {
		clear_bit(FEAT_EARLY_EXIT, spanel->feat);
		clear_bit(FEAT_FRAME_AUTO, spanel->feat);
	}

	spanel->auto_mode_vrefresh = idle_vrefresh;
	/* when mode is explicitly set (manual) panel idle effect would be disabled */
	ctx->panel_idle_vrefresh = 0;
	s6e3hc4_update_panel_feat(ctx, pmode, false);

}

static void s6e3hc4_change_frequency(struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	u32 vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 idle_vrefresh = 0;

	if (vrefresh > ctx->op_hz) {
		dev_err(ctx->dev,
		"invalid freq setting: op_hz=%u, vrefresh=%u\n",
		ctx->op_hz, vrefresh);
		return;
	}

	if (pmode->idle_mode == IDLE_MODE_ON_INACTIVITY)
		idle_vrefresh = s6e3hc4_get_min_idle_vrefresh(ctx, pmode);

	s6e3hc4_update_refresh_mode(ctx, pmode, idle_vrefresh);

	dev_dbg(ctx->dev, "change to %u hz)\n", vrefresh);
}

static bool s6e3hc4_set_self_refresh(struct exynos_panel *ctx, bool enable)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	struct s6e3hc4_panel *spanel = to_spanel(ctx);
	u32 idle_vrefresh;

	if (unlikely(!pmode))
		return false;

	/* self refresh is not supported in lp mode since that always makes use of early exit */
	if (pmode->exynos_mode.is_lp_mode)
		return false;

	idle_vrefresh = s6e3hc4_get_min_idle_vrefresh(ctx, pmode);

	if (pmode->idle_mode != IDLE_MODE_ON_SELF_REFRESH) {
		/*
		 * if idle mode is on inactivity, may need to update the target fps for auto mode,
		 * or switch to manual mode if idle should be disabled (idle_vrefresh=0)
		 */
		if ((pmode->idle_mode == IDLE_MODE_ON_INACTIVITY) &&
			(spanel->auto_mode_vrefresh != idle_vrefresh)) {
			dev_dbg(ctx->dev,
				"early exit update needed for mode: %s (idle_vrefresh: %u)\n",
				pmode->mode.name, idle_vrefresh);
			s6e3hc4_update_refresh_mode(ctx, pmode, idle_vrefresh);
			return true;
		}
		return false;
	}

	if (!enable)
		idle_vrefresh = 0;

	/* if there's no change in idle state then skip cmds */
	if (ctx->panel_idle_vrefresh == idle_vrefresh)
		return false;

	DPU_ATRACE_BEGIN(__func__);
	dev_dbg(ctx->dev, "set idle vrefresh: %u for mode %s\n",
		idle_vrefresh, pmode->mode.name);
	if (idle_vrefresh) {
		/* enable early exit while going into idle */
		set_bit(FEAT_EARLY_EXIT, spanel->feat);
		/* enable auto mode */
		set_bit(FEAT_FRAME_AUTO, spanel->feat);
	} else {
		/* disable early exit after coming out of idle */
		clear_bit(FEAT_EARLY_EXIT, spanel->feat);
		/* disable auto mode */
		clear_bit(FEAT_FRAME_AUTO, spanel->feat);
	}

	spanel->auto_mode_vrefresh = idle_vrefresh;
	ctx->panel_idle_vrefresh = idle_vrefresh;
	s6e3hc4_update_panel_feat(ctx, pmode, false);

	backlight_state_changed(ctx->bl);
	DPU_ATRACE_END(__func__);

	return true;
}

static int s6e3hc4_atomic_check(struct exynos_panel *ctx, struct drm_atomic_state *state)
{
	struct drm_connector *conn = &ctx->exynos_connector.base;
	struct drm_connector_state *new_conn_state = drm_atomic_get_new_connector_state(state, conn);
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;

	if (!new_conn_state || !new_conn_state->crtc)
		return 0;

	new_crtc_state = drm_atomic_get_new_crtc_state(state, new_conn_state->crtc);
	old_crtc_state = drm_atomic_get_old_crtc_state(state, new_conn_state->crtc);
	if (!old_crtc_state || !new_crtc_state || !new_crtc_state->active)
		return 0;

	if ((ctx->panel_idle_enabled && old_crtc_state->self_refresh_active) ||
	    !drm_atomic_crtc_effectively_active(old_crtc_state)) {
		struct drm_display_mode *mode = &new_crtc_state->adjusted_mode;

		/* set clock to max refresh rate on self refresh exit or resume due to early exit */
		mode->clock = mode->htotal * mode->vtotal * 120 / 1000;

		if (mode->clock != new_crtc_state->mode.clock) {
			new_crtc_state->mode_changed = true;
			dev_dbg(ctx->dev, "raise mode (%s) clock to 120hz on %s\n",
				mode->name,
				old_crtc_state->self_refresh_active ? "self refresh exit" : "resume");
		}
	} else if (old_crtc_state->active_changed &&
		   (old_crtc_state->adjusted_mode.clock != old_crtc_state->mode.clock)) {
		/* clock hacked in last commit due to self refresh exit or resume, undo that */
		new_crtc_state->mode_changed = true;
		new_crtc_state->adjusted_mode.clock = new_crtc_state->mode.clock;
		dev_dbg(ctx->dev, "restore mode (%s) clock after self refresh exit or resume\n",
			new_crtc_state->mode.name);
	}

	return 0;
}

static void s6e3hc4_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3HC4_WRCTRLD_BCTRL_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= S6E3HC4_WRCTRLD_HBM_BIT;

	if (ctx->hbm.local_hbm.enabled)
		val |= S6E3HC4_WRCTRLD_LOCAL_HBM_BIT;

	if (ctx->dimming_on)
		val |= S6E3HC4_WRCTRLD_DIMMING_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, hbm: %s, dimming: %s local_hbm: %s)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off",
		ctx->dimming_on ? "on" : "off",
		ctx->hbm.local_hbm.enabled ? "on" : "off");

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

static void s6e3hc4_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	u32 vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 delay_us = mult_frac(1000, 1020, vrefresh);

	EXYNOS_DCS_WRITE_TABLE(ctx, display_off);
	usleep_range(delay_us, delay_us + 10);
	s6e3hc4_write_display_mode(ctx, &pmode->mode);
	s6e3hc4_change_frequency(ctx, pmode);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static const struct exynos_dsi_cmd s6e3hc4_init_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(0x9D, 0x01),			/* PPS_DSC_EN */

	EXYNOS_DSI_CMD_SEQ_DELAY_REV(PANEL_REV_GE(PANEL_REV_DVT1), 120, 0x11),
	EXYNOS_DSI_CMD_SEQ_DELAY_REV(PANEL_REV_LT(PANEL_REV_DVT1), 130, 0x11),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFC, 0x5A, 0x5A),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xB0, 0x00, 0x0D, 0xFE),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFE, 0x00),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xFC, 0xA5, 0xA5),

	EXYNOS_DSI_CMD_SEQ(0x35),			/* TE on */

	EXYNOS_DSI_CMD0(unlock_cmd_f0),

	/* FFC: 165 Mhz, 1% tolerance */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xB0, 0x00, 0x36, 0xC5),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_LT(PANEL_REV_DVT1), 0xC5, 0x11, 0x10, 0x50, 0x05, 0x4E, 0x74),

	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x3C, 0xB9),	/* SEQ_GLOBAL_TSP_SYNC */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x19, 0x09),		/* SEQ_TSP_SYNC_ON */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x40, 0xB9),	/* SEQ_GLOBAL_TSP_SYNC_SETTING */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x30, 0x03),		/* SEQ_TSP_SYNC_SETTING*/
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x05, 0xF2),	/* SEQ_GLOBAL_TSP_SYNC_ON */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0xC8, 0xC0),		/* SEQ_TSP_SYNC_ON */
	EXYNOS_DSI_CMD0(freq_update),
	EXYNOS_DSI_CMD0(lock_cmd_f0),

	EXYNOS_DSI_CMD_SEQ(0x2A, 0x00, 0x00, 0x05, 0x9F), /* CASET */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x0C, 0x2F), /* PASET */
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc4_init);

static int s6e3hc4_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}
	mode = &pmode->mode;

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	/* DSC related configuration */
	EXYNOS_PPS_LONG_WRITE(ctx); /* PPS_SETTING */
	exynos_panel_send_cmd_set(ctx, &s6e3hc4_init_cmd_set);
	s6e3hc4_update_panel_feat(ctx, pmode, true);
	s6e3hc4_write_display_mode(ctx, mode); /* dimming and HBM */
	s6e3hc4_change_frequency(ctx, pmode);

	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	return 0;
}

static int s6e3hc4_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct s6e3hc4_panel *spanel = to_spanel(ctx);

	/* panel register state gets reset after disabling hardware */
	bitmap_clear(spanel->hw_feat, 0, FEAT_MAX);
	spanel->hw_vrefresh = 60;
	spanel->hw_idle_vrefresh = 0;

	return exynos_panel_disable(panel);
}

/*
 * 120hz auto mode takes at least 2 frames to start lowering refresh rate in addition to
 * time to next vblank. Use just over 2 frames time to consider worst case scenario
 */
#define EARLY_EXIT_THRESHOLD_US 17000

/**
 * s6e3hc4_trigger_early_exit - trigger early exit command to panel
 * @ctx: panel struct
 *
 * Sends a command to panel to indicate a frame is about to come in case its been a while since the
 * last frame update and auto mode may have started to take effect and lowering refresh rate
 */
static void s6e3hc4_trigger_early_exit(struct exynos_panel *ctx)
{
	const ktime_t delta = ktime_sub(ktime_get(), ctx->last_commit_ts);
	const s64 delta_us = ktime_to_us(delta);

	if (delta_us < EARLY_EXIT_THRESHOLD_US) {
		dev_dbg(ctx->dev, "skip early exit. %lldus since last commit\n",
			delta_us);
		return;
	}

	/* triggering early exit causes a switch to 120hz */
	ctx->last_mode_set_ts = ktime_get();

	DPU_ATRACE_BEGIN(__func__);

	if (ctx->idle_delay_ms) {
		const struct exynos_panel_mode *pmode = ctx->current_mode;

		dev_dbg(ctx->dev, "%s: disable auto idle mode for: %s\n",
				__func__, pmode->mode.name);
		s6e3hc4_update_refresh_mode(ctx, pmode, 0);
	} else {
		dev_dbg(ctx->dev, "sending early exit out cmd\n");
		EXYNOS_DCS_BUF_ADD_SET(ctx, unlock_cmd_f0);
		EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
		EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, lock_cmd_f0);
	}

	DPU_ATRACE_END(__func__);
}

static void s6e3hc4_commit_done(struct exynos_panel *ctx)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);

	if (!ctx->current_mode)
		return;

	if (test_bit(FEAT_EARLY_EXIT, spanel->feat))
		s6e3hc4_trigger_early_exit(ctx);
}

static void s6e3hc4_set_hbm_mode(struct exynos_panel *ctx,
				 enum exynos_hbm_mode mode)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	if (mode == ctx->hbm_mode)
		return;

	if (unlikely(!pmode))
		return;

	ctx->hbm_mode = mode;

	if (IS_HBM_ON(mode)) {
		set_bit(FEAT_HBM, spanel->feat);
		/* b/202738999 enforce IRC on */
#ifndef DPU_FACTORY_BUILD
		if (mode == HBM_ON_IRC_ON)
			clear_bit(FEAT_IRC_OFF, spanel->feat);
		else
			set_bit(FEAT_IRC_OFF, spanel->feat);
#endif
		s6e3hc4_update_panel_feat(ctx, NULL, false);
		s6e3hc4_write_display_mode(ctx, &pmode->mode);
	} else {
		clear_bit(FEAT_HBM, spanel->feat);
		clear_bit(FEAT_IRC_OFF, spanel->feat);
		s6e3hc4_write_display_mode(ctx, &pmode->mode);
		s6e3hc4_update_panel_feat(ctx, NULL, false);
	}
}

static void s6e3hc4_set_dimming_on(struct exynos_panel *ctx,
				 bool dimming_on)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	ctx->dimming_on = dimming_on;
	s6e3hc4_write_display_mode(ctx, &pmode->mode);
}

static const struct exynos_dsi_cmd s6e3hc4_lhbm_extra_cmds[] = {
	EXYNOS_DSI_CMD0(unlock_cmd_f0),

	/* global para */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x02, 0x1E, 0x92),
	/* area set */
	EXYNOS_DSI_CMD_SEQ(0x92, 0x20, 0x88, 0x71, 0x39, 0x8A, 0x01),
	/* global para */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x02, 0x24, 0x92),
	/* center position set, x: 0x2D0, y: 0x939 */
	EXYNOS_DSI_CMD_SEQ(0x92, 0x2D, 0x09, 0x39),
	/* global para */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x02, 0x27, 0x92),
	/* circle size set, radius: 6 mm */
	EXYNOS_DSI_CMD_SEQ(0x92, 0x78),

	EXYNOS_DSI_CMD0(lock_cmd_f0)
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc4_lhbm_extra);

static void s6e3hc4_set_local_hbm_mode(struct exynos_panel *ctx,
				 bool local_hbm_en)
{
	const struct exynos_panel_mode *pmode;
	const u32 flags = PANEL_CMD_SET_IGNORE_VBLANK | PANEL_CMD_SET_BATCH;

	if (ctx->hbm.local_hbm.enabled == local_hbm_en)
		return;

	pmode = ctx->current_mode;
	if (unlikely(pmode == NULL)) {
		dev_err(ctx->dev, "%s: unknown current mode\n", __func__);
		return;
	}

	ctx->hbm.local_hbm.enabled = local_hbm_en;
	if (local_hbm_en)
		exynos_panel_send_cmd_set_flags(ctx,
			&s6e3hc4_lhbm_extra_cmd_set, flags);
	s6e3hc4_write_display_mode(ctx, &pmode->mode);
}

static void s6e3hc4_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{

	if (ctx->hbm.local_hbm.enabled == true)
		dev_warn(ctx->dev, "do mode change (`%s`) unexpectedly when LHBM is ON\n",
			pmode->mode.name);

	s6e3hc4_change_frequency(ctx, pmode);
}

static bool s6e3hc4_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct drm_display_mode *c = &ctx->current_mode->mode;
	const struct drm_display_mode *n = &pmode->mode;

	/* seamless mode set can happen if active region resolution is same */
	return (c->vdisplay == n->vdisplay) && (c->hdisplay == n->hdisplay) &&
	       (c->flags == n->flags);
}

static int s6e3hc4_set_op_hz(struct exynos_panel *ctx, unsigned int hz)
{
	struct s6e3hc4_panel *spanel = to_spanel(ctx);
	u32 vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	if (vrefresh > hz) {
		dev_err(ctx->dev, "invalid op_hz=%d for vrefresh=%d\n",
			hz, vrefresh);
		return -EINVAL;
	}

	ctx->op_hz = hz;
	if (hz == 60)
		set_bit(FEAT_OP_NS, spanel->feat);
	else
		clear_bit(FEAT_OP_NS, spanel->feat);
	s6e3hc4_update_panel_feat(ctx, NULL, false);
	dev_info(ctx->dev, "set op_hz at %d\n", hz);
	return 0;
}

static void s6e3hc4_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 rev = ((build_code & 0xE0) >> 3) | (build_code & 0x03);

	exynos_panel_get_panel_rev(ctx, rev);
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 350,
	.te_var = 1,
};

static const u32 s6e3hc4_bl_range[] = {
	94, 180, 270, 360, 2047
};

static const struct exynos_panel_mode s6e3hc4_modes[] = {
	{
		/* 1440x3120 @ 60Hz */
		.mode = {
			.name = "1440x3120x60",
			.clock = 298620,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.type = DRM_MODE_TYPE_PREFERRED,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 16,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_UNSUPPORTED,
	},
	{
		/* 1440x3120 @ 120Hz */
		.mode = {
			.name = "1440x3120x120",
			.clock = 597240,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 16,
			.falling_edge = 48,
		},
		.idle_mode = IDLE_MODE_ON_SELF_REFRESH,
	},
};

static const struct exynos_panel_mode s6e3hc4_lp_mode = {
	.mode = {
		/* 1440x3120 @ 30Hz */
		.name = "1440x3120x30",
		.clock = 149310,
		.hdisplay = 1440,
		.hsync_start = 1440 + 80, // add hfp
		.hsync_end = 1440 + 80 + 24, // add hsa
		.htotal = 1440 + 80 + 24 + 36, // add hbp
		.vdisplay = 3120,
		.vsync_start = 3120 + 12, // add vfp
		.vsync_end = 3120 + 12 + 4, // add vsa
		.vtotal = 3120 + 12 + 4 + 14, // add vbp
		.flags = 0,
		.width_mm = 71,
		.height_mm = 155,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 52,
		},
		.underrun_param = &underrun_param,
		.is_lp_mode = true,
	}
};

static void s6e3hc4_panel_init(struct exynos_panel *ctx)
{
	struct dentry *csroot = ctx->debugfs_cmdset_entry;
	struct s6e3hc4_panel *spanel = to_spanel(ctx);

	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc4_init_cmd_set, "init");
	debugfs_create_bool("force_changeable_te", 0644, ctx->debugfs_entry,
				&spanel->force_changeable_te);
}

static int s6e3hc4_panel_probe(struct mipi_dsi_device *dsi)
{
	struct s6e3hc4_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	spanel->base.op_hz = 120;
	spanel->hw_vrefresh = 60;
	return exynos_panel_common_init(dsi, &spanel->base);
}

static const struct drm_panel_funcs s6e3hc4_drm_funcs = {
	.disable = s6e3hc4_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc4_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs s6e3hc4_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3hc4_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3hc4_set_hbm_mode,
	.set_dimming_on = s6e3hc4_set_dimming_on,
	.set_local_hbm_mode = s6e3hc4_set_local_hbm_mode,
	.is_mode_seamless = s6e3hc4_is_mode_seamless,
	.mode_set = s6e3hc4_mode_set,
	.panel_init = s6e3hc4_panel_init,
	.get_panel_rev = s6e3hc4_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = s6e3hc4_update_te2,
	.commit_done = s6e3hc4_commit_done,
	.atomic_check = s6e3hc4_atomic_check,
	.set_self_refresh = s6e3hc4_set_self_refresh,
	.set_op_hz = s6e3hc4_set_op_hz,
};

const struct brightness_capability s6e3hc4_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 500,
		},
		.level = {
			.min = 4,
			.max = 2047,
		},
		.percentage = {
			.min = 0,
			.max = 50,
		},
	},
	.hbm = {
		.nits = {
			.min = 550,
			.max = 1000,
		},
		.level = {
			.min = 2238,
			.max = 3949,
		},
		.percentage = {
			.min = 50,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3hc4 = {
	.dsc_pps = PPS_SETTING,
	.dsc_pps_len = ARRAY_SIZE(PPS_SETTING),
	.data_lane_cnt = 4,
	.max_brightness = 3949,
	.dft_brightness = 1023,
	.brt_capability = &s6e3hc4_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 10000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc4_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc4_bl_range),
	.modes = s6e3hc4_modes,
	.num_modes = ARRAY_SIZE(s6e3hc4_modes),
	.off_cmd_set = &s6e3hc4_off_cmd_set,
	.lp_mode = &s6e3hc4_lp_mode,
	.lp_cmd_set = &s6e3hc4_lp_cmd_set,
	.binned_lp = s6e3hc4_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc4_binned_lp),
	.panel_func = &s6e3hc4_drm_funcs,
	.exynos_panel_func = &s6e3hc4_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3hc4", .data = &samsung_s6e3hc4 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = s6e3hc4_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc4",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Shiyong Li<shiyongli@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3hc4 panel driver");
MODULE_LICENSE("GPL");
