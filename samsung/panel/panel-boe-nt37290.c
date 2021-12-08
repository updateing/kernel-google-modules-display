// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based nt37290 AMOLED LCD panel driver.
 *
 * Copyright (c) 2021 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

/**
 * enum nt37290_panel_feature - features supported by this panel
 * @G10_FEAT_EARLY_EXIT: early exit from a long frame
 * @G10_FEAT_FRAME_AUTO: automatic (not manual) frame control
 * @G10_FEAT_MAX: placeholder, counter for number of features
 *
 * The following features are correlated, if one or more of them change, the others need
 * to be updated unconditionally.
 */
enum nt37290_panel_feature {
	G10_FEAT_EARLY_EXIT = 0,
	G10_FEAT_FRAME_AUTO,
	G10_FEAT_MAX,
};

/**
 * struct nt37290_panel - panel specific runtime info
 *
 * This struct maintains nt37290 panel specific runtime info, any fixed details about panel
 * should most likely go into struct exynos_panel_desc. The variables with the prefix hw_ keep
 * track of the features that were actually committed to hardware, and should be modified
 * after sending cmds to panel, i.e. updating hw state.
 */
struct nt37290_panel {
	/** @base: base panel struct */
	struct exynos_panel base;
	/** @feat: software/working correlated features, not guaranteed to be effective in panel */
	DECLARE_BITMAP(feat, G10_FEAT_MAX);
	/** @hw_feat: correlated states effective in panel */
	DECLARE_BITMAP(hw_feat, G10_FEAT_MAX);
	/** @hw_vrefresh: vrefresh rate effective in panel */
	u32 hw_vrefresh;
};

#define to_spanel(ctx) container_of(ctx, struct nt37290_panel, base)

static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 cmd2_page0[] = { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00 };
static const u8 stream_2c[] = { 0x2C };

static const struct exynos_dsi_cmd nt37290_lp_cmds[] = {
	/* enter AOD */
	EXYNOS_DSI_CMD_SEQ(0x39),
	/* manual mode (no frame skip) */
	EXYNOS_DSI_CMD_SEQ(0x2F, 0x00),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_lp);

static const struct exynos_dsi_cmd nt37290_lp_off_cmds[] = {
	EXYNOS_DSI_CMD0(display_off),
};

static const struct exynos_dsi_cmd nt37290_lp_low_cmds[] = {
	/* 10 nit */
	EXYNOS_DSI_CMD_SEQ_DELAY(9, 0x51, 0x00, 0x00, 0x00, 0x00, 0x03, 0x33),
	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DSI_CMD(stream_2c, 9),
	EXYNOS_DSI_CMD0(stream_2c),
	EXYNOS_DSI_CMD0(display_on),
};

static const struct exynos_dsi_cmd nt37290_lp_high_cmds[] = {
	/* 50 nit */
	EXYNOS_DSI_CMD_SEQ_DELAY(9, 0x51, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE),
	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DSI_CMD(stream_2c, 9),
	EXYNOS_DSI_CMD0(stream_2c),
	EXYNOS_DSI_CMD0(display_on),
};

static const struct exynos_binned_lp nt37290_binned_lp[] = {
	BINNED_LP_MODE("off", 0, nt37290_lp_off_cmds),
	/* TODO: specify TE2 timing (default rising = 0, falling = 19) */
	BINNED_LP_MODE_TIMING("low", 80, nt37290_lp_low_cmds, 0, 19),
	BINNED_LP_MODE_TIMING("high", 2047, nt37290_lp_high_cmds, 0, 19),
};

static const struct exynos_dsi_cmd nt37290_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 100),
	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x10),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_off);

static const struct exynos_dsi_cmd nt37290_init_cmds[] = {
	/* CMD1 */
	/* set for higher MIPI speed: 1346Mbps */
	EXYNOS_DSI_CMD_SEQ(0x1F, 0xF0),
	/* gamma curve */
	EXYNOS_DSI_CMD_SEQ(0x26, 0x00),
	/* row address */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x0C, 0x2F),
	/* TE output line */
	EXYNOS_DSI_CMD_SEQ(0x35),
	/* select brightness value */
	EXYNOS_DSI_CMD_SEQ(0x51, 0x03, 0xF8, 0x03, 0xF8, 0x0F, 0xFE),
	/* control brightness */
	EXYNOS_DSI_CMD_SEQ(0x53, 0x20),
	EXYNOS_DSI_CMD_SEQ(0x5A, 0x01),
	/* DSC: slice 24, 2 decoder */
	EXYNOS_DSI_CMD_SEQ(0x90, 0x03, 0x03),
	EXYNOS_DSI_CMD_SEQ(0x91, 0x89, 0x28, 0x00, 0x18, 0xD2, 0x00, 0x02,
			   0x86, 0x02, 0x83, 0x00, 0x0A, 0x04, 0x86, 0x03,
			   0x2E, 0x10, 0xF0),
	/* change refresh frame to 1 after 2Ch command in skip mode */
	EXYNOS_DSI_CMD0(cmd2_page0),
	EXYNOS_DSI_CMD_SEQ(0xBA, 0x00),

	/* CMD2 Page 1 */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xC5, 0x00, 0x0B, 0x0B, 0x0B),

	/* CMD3 Page 0 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x80),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x1B),
	EXYNOS_DSI_CMD_SEQ(0xF4, 0x55),
	/* CMD3 Page 1 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x81),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x12),
	EXYNOS_DSI_CMD_SEQ(0xF5, 0x00),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x09),
	EXYNOS_DSI_CMD_SEQ(0xF9, 0x10),
	/* CMD3 Page 3 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x83),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x14),
	EXYNOS_DSI_CMD_SEQ(0xF8, 0x0D),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xF9, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFA, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x06),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x01),
	EXYNOS_DSI_CMD_SEQ(0xFC, 0x06),
	/* CMD3 Page 4 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xAA, 0x55, 0xA5, 0x84),
	EXYNOS_DSI_CMD_SEQ(0x6F, 0x1C),
	EXYNOS_DSI_CMD_SEQ(0xF8, 0x3A),

	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x11),
};
static DEFINE_EXYNOS_CMD_SET(nt37290_init);

static void nt37290_update_panel_feat(struct exynos_panel *ctx,
	const struct exynos_panel_mode *pmode, bool enforce)
{
	struct nt37290_panel *spanel = to_spanel(ctx);
	u32 vrefresh;
	DECLARE_BITMAP(changed_feat, G10_FEAT_MAX);
	bool ee, fi;

	if (pmode)
		vrefresh = drm_mode_vrefresh(&pmode->mode);
	else
		vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	if (enforce) {
		bitmap_fill(changed_feat, G10_FEAT_MAX);
	} else {
		bitmap_xor(changed_feat, spanel->feat, spanel->hw_feat, G10_FEAT_MAX);
		if (bitmap_empty(changed_feat, G10_FEAT_MAX) &&
			vrefresh == spanel->hw_vrefresh)
			return;
	}

	spanel->hw_vrefresh = vrefresh;
	bitmap_copy(spanel->hw_feat, spanel->feat, G10_FEAT_MAX);
	ee = test_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
	fi = test_bit(G10_FEAT_FRAME_AUTO, spanel->feat);

	dev_dbg(ctx->dev, "ee=%s fi=%s vrefresh=%u\n",
		ee ? "on" : "off", fi ? "auto" : "manual", vrefresh);

	if (vrefresh == 120 && !fi) {
		/* freq_mode_hs */
		EXYNOS_DCS_BUF_ADD(ctx, 0x2F, 0x00);
		/* restore TE timing (no shift) */
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x00);
	} else {
		/* freq_mode_hs */
		EXYNOS_DCS_BUF_ADD(ctx, 0x2F, 0x00);
		/* freq_ctrl_hs */
		EXYNOS_DCS_BUF_ADD(ctx, 0x2F, 0x30);
		/* early exit */
		EXYNOS_DCS_BUF_ADD(ctx, 0x5A, !ee);

		/* TODO: add support of auto frame insertion */
		if (!fi) {
			/* auto frame insertion off (manual) */
			EXYNOS_DCS_BUF_ADD_SET(ctx, cmd2_page0);
			EXYNOS_DCS_BUF_ADD(ctx, 0x6F, 0x1C);
			if (vrefresh == 60)
				EXYNOS_DCS_BUF_ADD(ctx,
					0xBA, 0x91, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00);
		}

		EXYNOS_DCS_BUF_ADD(ctx, 0x2C);
		/* TE shift 8.2ms */
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x44, 0x00, 0x01);
	}
}

static void nt37290_change_frequency(struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	u32 vrefresh = drm_mode_vrefresh(&pmode->mode);
	struct nt37290_panel *spanel = to_spanel(ctx);
	bool was_lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;

	clear_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
	clear_bit(G10_FEAT_FRAME_AUTO, spanel->feat);

	/* need to send 2Fh command while exiting AOD */
	nt37290_update_panel_feat(ctx, pmode, was_lp_mode);

	dev_dbg(ctx->dev, "change to %u hz, was_lp_mode %d\n", vrefresh, was_lp_mode);
}

static void nt37290_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	if (!is_panel_active(ctx))
		return;

	/* exit AOD */
	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 34, 0x38);

	nt37290_change_frequency(ctx, pmode);

	/* 2Ch needs to be sent twice in next 2 vsync */
	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 34, stream_2c);
	EXYNOS_DCS_WRITE_TABLE(ctx, stream_2c);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int nt37290_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct nt37290_panel *spanel = to_spanel(ctx);
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);
	exynos_panel_send_cmd_set(ctx, &nt37290_init_cmd_set);

	/* TODO: add support of early exit and auto */
	clear_bit(G10_FEAT_EARLY_EXIT, spanel->feat);
	clear_bit(G10_FEAT_FRAME_AUTO, spanel->feat);
	nt37290_update_panel_feat(ctx, pmode, true);

	if (!pmode->exynos_mode.is_lp_mode)
		EXYNOS_DCS_WRITE_TABLE(ctx, display_on);
	else
		exynos_panel_set_lp_mode(ctx, pmode);

	return 0;
}

static int nt37290_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	struct nt37290_panel *spanel = to_spanel(ctx);

	/* panel register state gets reset after disabling hardware */
	bitmap_clear(spanel->hw_feat, 0, G10_FEAT_MAX);
	spanel->hw_vrefresh = 60;

	return exynos_panel_disable(panel);
}

static void nt37290_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	if (!is_panel_active(ctx))
		return;

	nt37290_change_frequency(ctx, pmode);
}

static bool nt37290_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct drm_display_mode *c = &ctx->current_mode->mode;
	const struct drm_display_mode *n = &pmode->mode;

	/* seamless mode set can happen if active region resolution is same */
	return (c->vdisplay == n->vdisplay) && (c->hdisplay == n->hdisplay) &&
	       (c->flags == n->flags);
}

static void nt37290_get_panel_rev(struct exynos_panel *ctx, u32 id)
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

static const u32 nt37290_bl_range[] = {
	94, 180, 270, 360, 2047
};

/* Truncate 8-bit signed value to 6-bit signed value */
#define TO_6BIT_SIGNED(v) (v & 0x3F)

static const struct drm_dsc_config nt37290_dsc_cfg = {
	.first_line_bpg_offset = 13,
	.rc_range_params = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{4, 10, TO_6BIT_SIGNED(-10)},
		{5, 10, TO_6BIT_SIGNED(-10)},
		{5, 11, TO_6BIT_SIGNED(-10)},
		{5, 11, TO_6BIT_SIGNED(-12)},
		{8, 12, TO_6BIT_SIGNED(-12)},
		{12, 13, TO_6BIT_SIGNED(-12)},
	},
};

#define NT37290_DSC_CONFIG \
	.dsc = { \
		.enabled = true, \
		.dsc_count = 2, \
		.slice_count = 2, \
		.slice_height = 24, \
		.cfg = &nt37290_dsc_cfg, \
	}

static const struct exynos_panel_mode nt37290_modes[] = {
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
			.width_mm = 76,
			.height_mm = 160,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
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
			.width_mm = 76,
			.height_mm = 160,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			NT37290_DSC_CONFIG,
			.underrun_param = &underrun_param,
		},
	},
};

static const struct exynos_panel_mode nt37290_lp_mode = {
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
		.width_mm = 76,
		.height_mm = 160,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		NT37290_DSC_CONFIG,
		.underrun_param = &underrun_param,
		.is_lp_mode = true,
	},
};

static void nt37290_panel_init(struct exynos_panel *ctx)
{
	struct dentry *csroot = ctx->debugfs_cmdset_entry;

	exynos_panel_debugfs_create_cmdset(ctx, csroot, &nt37290_init_cmd_set, "init");
}

static int nt37290_panel_probe(struct mipi_dsi_device *dsi)
{
	struct nt37290_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	spanel->hw_vrefresh = 60;

	return exynos_panel_common_init(dsi, &spanel->base);
}

static const struct drm_panel_funcs nt37290_drm_funcs = {
	.disable = nt37290_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = nt37290_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs nt37290_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = nt37290_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.is_mode_seamless = nt37290_is_mode_seamless,
	.mode_set = nt37290_mode_set,
	.panel_init = nt37290_panel_init,
	.get_panel_rev = nt37290_get_panel_rev,
};

const struct brightness_capability nt37290_brightness_capability = {
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
			.max = 62,
		},
	},
	.hbm = {
		.nits = {
			.min = 550,
			.max = 800,
		},
		.level = {
			.min = 2232,
			.max = 3152,
		},
		.percentage = {
			.min = 62,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc boe_nt37290 = {
	.data_lane_cnt = 4,
	.max_brightness = 3152,
	.dft_brightness = 1023,
	.brt_capability = &nt37290_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = nt37290_bl_range,
	.bl_num_ranges = ARRAY_SIZE(nt37290_bl_range),
	.modes = nt37290_modes,
	.num_modes = ARRAY_SIZE(nt37290_modes),
	.off_cmd_set = &nt37290_off_cmd_set,
	.lp_mode = &nt37290_lp_mode,
	.lp_cmd_set = &nt37290_lp_cmd_set,
	.binned_lp = nt37290_binned_lp,
	.num_binned_lp = ARRAY_SIZE(nt37290_binned_lp),
	.panel_func = &nt37290_drm_funcs,
	.exynos_panel_func = &nt37290_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "boe,nt37290", .data = &boe_nt37290 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = nt37290_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-boe-nt37290",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Chris Lu <luchris@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based BOE nt37290 panel driver");
MODULE_LICENSE("GPL");
