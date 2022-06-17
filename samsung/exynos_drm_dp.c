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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <video/videomode.h>

#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "exynos_drm_dp.h"

struct dp_device *dp_drvdata;
EXPORT_SYMBOL(dp_drvdata);

#define DP_SUPPORT_TPS(_v) BIT((_v)-1)

static inline struct dp_device *encoder_to_dp(struct drm_encoder *e)
{
	return container_of(e, struct dp_device, encoder);
}

static inline struct dp_device *dp_aux_to_dp(struct drm_dp_aux *a)
{
	return container_of(a, struct dp_device, dp_aux);
}

static enum hotplug_state dp_get_hpd_state(struct dp_device *dp)
{
	enum hotplug_state hpd_current_state;

	mutex_lock(&dp->hpd_state_lock);
	hpd_current_state = dp->hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);

	return hpd_current_state;
}

static void dp_set_hpd_state(struct dp_device *dp,
			     enum hotplug_state hpd_current_state)
{
	mutex_lock(&dp->hpd_state_lock);
	dp->hpd_current_state = hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);
}

static void dp_init_info(struct dp_device *dp)
{
	dp->state = DP_STATE_INIT;
	dp->hpd_current_state = EXYNOS_HPD_UNPLUG;
	dp->output_type = EXYNOS_DISPLAY_TYPE_DP0_SST1;
	dp->bist_used = false;
}

static u32 dp_get_max_link_rate(struct dp_device *dp)
{
	return min(dp->host.link_rate, dp->sink.link_rate);
}

static u8 dp_get_max_num_lanes(struct dp_device *dp)
{
	return min(dp->host.num_lanes, dp->sink.num_lanes);
}

static u8 dp_get_supported_pattern(struct dp_device *dp)
{
	return fls(dp->host.support_tps & dp->sink.support_tps);
}

static bool dp_get_enhanced_mode(struct dp_device *dp)
{
	return (dp->host.enhanced_frame && dp->sink.enhanced_frame);
}

static bool dp_get_ssc(struct dp_device *dp)
{
	return (dp->host.ssc && dp->sink.ssc);
}

#define MAX_VOLTAGE_LEVEL 3
#define MAX_PREEMPH_LEVEL 3

static void dp_fill_host_caps(struct dp_device *dp)
{
	dp->host.num_lanes = MAX_LANE_CNT;
	dp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1);
	dp->host.volt_swing_max = MAX_VOLTAGE_LEVEL;
	dp->host.pre_emphasis_max = MAX_PREEMPH_LEVEL;
	dp->host.support_tps = DP_SUPPORT_TPS(1) | DP_SUPPORT_TPS(2) |
			       DP_SUPPORT_TPS(3) | DP_SUPPORT_TPS(4);
	dp->host.fast_training = false;
	dp->host.enhanced_frame = true;
	dp->host.scrambler = true;
	dp->host.ssc = false;
}

static void dp_fill_sink_caps(struct dp_device *dp,
			      u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	dp->sink.link_rate = dp->link.link_rate;
	dp->sink.num_lanes = dp->link.num_lanes;
	dp->sink.enhanced_frame = dp->link.enhanced_frame;

	/* Set SSC support */
	dp->sink.ssc = !!(dpcd[DP_MAX_DOWNSPREAD] & DP_MAX_DOWNSPREAD_0_5);

	/* Set TPS support */
	dp->sink.support_tps = DP_SUPPORT_TPS(1) | DP_SUPPORT_TPS(2);
	if (drm_dp_tps3_supported(dpcd))
		dp->sink.support_tps |= DP_SUPPORT_TPS(3);
	if (drm_dp_tps4_supported(dpcd))
		dp->sink.support_tps |= DP_SUPPORT_TPS(4);

	/* Set fast link support */
	dp->sink.fast_training = drm_dp_fast_training_cap(dpcd);
}

// Callback function for DRM DP Helper
static ssize_t dp_aux_transfer(struct drm_dp_aux *dp_aux,
			       struct drm_dp_aux_msg *msg)
{
	struct dp_device *dp = dp_aux_to_dp(dp_aux);
	int ret;

	switch (msg->request) {
	case DP_AUX_NATIVE_WRITE:
		ret = dp_hw_write_dpcd_burst(msg->address, msg->size,
					     msg->buffer);
		break;
	case DP_AUX_NATIVE_READ:
		ret = dp_hw_read_dpcd_burst(msg->address, msg->size,
					    msg->buffer);
		break;
	default:
		dp_err(dp, "unsupported DP Request(0x%X)\n", msg->request);
		return -EINVAL;
	}

	if (ret)
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
	else {
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		ret = msg->size;
	}

	// Should be return size or error code
	return ret;
}

// Callback function for DRM EDID Helper
static int dp_get_edid_block(void *data, u8 *edid, unsigned int block,
			     size_t length)
{
	dp_hw_read_edid(block, length, edid);

	return 0;
}

//------------------------------------------------------------------------------
//
static int dp_check_dp_sink(struct dp_device *dp)
{
	int sink_count = drm_dp_read_sink_count(&dp->dp_aux);

	if (sink_count < 0) {
		dp_err(dp, "failed to read DP Sink count\n");
		return sink_count;
	} else if (sink_count > 2) {
		// Now, only 1 DP Sink should be supported.
		dp_err(dp, "DP sink count is %d\n", sink_count);
		return -EPERM;
	}

	return 0;
}

static int dp_sink_power_up(struct dp_device *dp, bool up)
{
	u8 val = 0;
	int ret;

	if (dp->link.revision >= DP_DPCD_REV_11) {
		ret = drm_dp_dpcd_readb(&dp->dp_aux, DP_SET_POWER, &val);
		if (ret < 0) {
			dp_err(dp,
			       "DP Sink: failed to read sink power state\n");
			return ret;
		}

		val &= ~DP_SET_POWER_MASK;
		if (up)
			val |= DP_SET_POWER_D0;
		else
			val |= DP_SET_POWER_D3;

		ret = drm_dp_dpcd_writeb(&dp->dp_aux, DP_SET_POWER, val);
		if (ret < 0) {
			dp_err(dp,
			       "DP Sink: failed to write sink power state\n");
			return ret;
		}

		usleep_range(1000, 2000);
	}

	dp_info(dp, "DP Sink: sink power state set to %s\n", up ? "D0" : "D3");
	return 0;
}

static u32 dp_get_training_interval_us(struct dp_device *dp, u32 interval)
{
	if (interval == 0)
		return 400;
	else if (interval < 5)
		return 4000 << (interval - 1);
	else
		dp_err(dp, "returned wrong trainig interval(%u)\n", interval);

	return 0;
}

static void dp_print_swing_level(struct dp_device *dp)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;

	dp_debug(dp, "Volt_Swing  : %02X %02X %02X %02X\n", vol_swing_level[0],
		 vol_swing_level[1], vol_swing_level[2], vol_swing_level[3]);
	dp_debug(dp, "Pre_Emphasis: %02X %02X %02X %02X\n", pre_empha_level[0],
		 pre_empha_level[1], pre_empha_level[2], pre_empha_level[3]);
}

static void dp_validate_link_training(struct dp_device *dp, bool *cr_done,
				      bool *same_before_adjust,
				      bool *max_swing_reached,
				      u8 before_cr[MAX_LANE_CNT],
				      u8 after_cr[DP_LINK_STATUS_SIZE])
{
	u8 *req_vol = dp->host.vol_swing_level;
	u8 *req_pre = dp->host.pre_empha_level;
	u8 *req_max = dp->host.max_reach_value;

	u8 adjust_volt, adjust_pre;
	bool same_pre, same_volt;
	int i;

	*same_before_adjust = false;
	*max_swing_reached = false;
	*cr_done = drm_dp_clock_recovery_ok(after_cr, dp->link.num_lanes);

	for (i = 0; i < dp->link.num_lanes; i++) {
		req_max[i] = 0;

		adjust_volt = drm_dp_get_adjust_request_voltage(after_cr, i);
		req_vol[i] = min(adjust_volt, dp->host.volt_swing_max);
		if (req_vol[i] == dp->host.volt_swing_max)
			req_max[i] |= DP_TRAIN_MAX_SWING_REACHED;

		adjust_pre =
			drm_dp_get_adjust_request_pre_emphasis(after_cr, i) >>
			DP_TRAIN_PRE_EMPHASIS_SHIFT;
		req_pre[i] = min(adjust_pre, dp->host.pre_emphasis_max);
		if (req_pre[i] == dp->host.pre_emphasis_max)
			req_max[i] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		same_pre = (before_cr[i] & DP_TRAIN_PRE_EMPHASIS_MASK) ==
			   req_pre[i] << DP_TRAIN_PRE_EMPHASIS_SHIFT;
		same_volt = (before_cr[i] & DP_TRAIN_VOLTAGE_SWING_MASK) ==
			    req_vol[i];
		if (same_pre && same_volt)
			*same_before_adjust = true;

		if (!*cr_done && req_vol[i] + req_pre[i] >= 3) {
			*max_swing_reached = true;
			return;
		}
	}
}

static void dp_set_hwconfig_dplink(struct dp_device *dp)
{
	u8 link_rate = drm_dp_link_rate_to_bw_code(dp->link.link_rate);

	if (link_rate == DP_LINK_BW_1_62)
		dp->hw_config.link_rate = LINK_RATE_RBR;
	else if (link_rate == DP_LINK_BW_2_7)
		dp->hw_config.link_rate = LINK_RATE_HBR;
	else if (link_rate == DP_LINK_BW_5_4)
		dp->hw_config.link_rate = LINK_RATE_HBR2;
	else if (link_rate == DP_LINK_BW_8_1)
		dp->hw_config.link_rate = LINK_RATE_HBR3;

	dp->hw_config.num_lanes = dp->link.num_lanes;
}

static void dp_set_hwconfig_video(struct dp_device *dp)
{
	dp->hw_config.enhanced_mode = dp_get_enhanced_mode(dp);
	dp->hw_config.use_fec = false;
	dp->hw_config.use_ssc = dp_get_ssc(dp);
}

static int dp_link_configure(struct dp_device *dp)
{
	u8 values[2];
	int err;

	values[0] = drm_dp_link_rate_to_bw_code(dp->link.link_rate);
	values[1] = dp->link.num_lanes;

	if (dp_get_enhanced_mode(dp))
		values[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;

	err = drm_dp_dpcd_write(&dp->dp_aux, DP_LINK_BW_SET, values,
				sizeof(values));
	if (err < 0)
		return err;

	return 0;
}

static void dp_init_link_training_cr(struct dp_device *dp)
{
	/* Disable DP Training Pattern */
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	/* Reconfigure DP HW */
	dp_set_hwconfig_dplink(dp);
	dp_set_hwconfig_video(dp);
	dp_hw_reinit(&dp->hw_config);
	dp_info(dp, "HW configured with Rate(%d) and Lanes(%u)\n",
		dp->hw_config.link_rate, dp->hw_config.num_lanes);

	/* Reconfigure DP Link */
	dp_link_configure(dp);

	/* Set DP Training Pattern in DP PHY */
	dp_hw_set_training_pattern(TRAINING_PATTERN_1);

	/* Enable DP Training Pattern */
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_1 | DP_LINK_SCRAMBLING_DISABLE);
}

static bool dp_do_link_training_cr(struct dp_device *dp, u32 interval_us)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;
	u8 *max_reach_value = dp->host.max_reach_value;

	u8 lanes_data[MAX_LANE_CNT]; // before_cr
	u8 link_status[DP_LINK_STATUS_SIZE]; // after_cr
	bool cr_done;
	bool same_before_adjust, max_swing_reached, try_max_swing = false;
	u8 fail_counter_short = 0, fail_counter_long = 0;
	int i;

	dp_info(dp, "Link Training CR Phase with BW(%u) and Lanes(%u)\n",
		dp->link.link_rate, dp->link.num_lanes);

	for (i = 0; i < MAX_LANE_CNT; i++) {
		vol_swing_level[i] = 0;
		pre_empha_level[i] = 0;
		max_reach_value[i] = 0;
	}

	dp_init_link_training_cr(dp);

	// Voltage Swing
	do {
		// Set Voltage
		dp_print_swing_level(dp);
		dp_hw_set_voltage_and_pre_emphasis(&dp->hw_config,
						   (u8 *)vol_swing_level,
						   (u8 *)pre_empha_level);

		// Write Link Training
		for (i = 0; i < dp->link.num_lanes; i++)
			lanes_data[i] = (pre_empha_level[i]
					 << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
					vol_swing_level[i] | max_reach_value[i];
		drm_dp_dpcd_write(&dp->dp_aux, DP_TRAINING_LANE0_SET,
				  lanes_data, dp->link.num_lanes);

		udelay(interval_us);

		// Read Link Status
		drm_dp_dpcd_read_link_status(&dp->dp_aux, link_status);

		// Validate CR
		dp_validate_link_training(dp, &cr_done, &same_before_adjust,
					  &max_swing_reached, lanes_data,
					  link_status);

		if (max_swing_reached) {
			dp_info(dp, "requested to adjust max swing level\n");
			if (!try_max_swing) {
				try_max_swing = true;
				continue;
			} else {
				dp_err(dp, "reached max swing level\n");
				goto err;
			}
		}

		if (cr_done) {
			dp_print_swing_level(dp);
			dp_info(dp, "CR Done. Move to Training_EQ.\n");
			return true;
		}

		fail_counter_long++;

		if (same_before_adjust) {
			dp_info(dp, "requested same level. Retry...\n");
			fail_counter_short++;
			continue;
		}

		fail_counter_short = 0;
	} while (fail_counter_short < 5 && fail_counter_long < 10);

err:
	dp_err(dp, "failed Link Training CR phase with BW(%u) and Lanes(%u)\n",
	       dp->link.link_rate, dp->link.num_lanes);
	return false;
}

static void dp_init_link_training_eq(struct dp_device *dp, u8 tps)
{
	/* Set DP Training Pattern */
	if (tps | DP_SUPPORT_TPS(4)) {
		dp_hw_set_training_pattern(TRAINING_PATTERN_4);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_4);
	} else if (tps | DP_SUPPORT_TPS(3)) {
		dp_hw_set_training_pattern(TRAINING_PATTERN_3);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_3 |
					   DP_LINK_SCRAMBLING_DISABLE);
	} else {
		dp_hw_set_training_pattern(TRAINING_PATTERN_2);
		drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_2 |
					   DP_LINK_SCRAMBLING_DISABLE);
	}
}

static bool dp_do_link_training_eq(struct dp_device *dp, u32 interval_us,
				   u8 tps)
{
	u8 *vol_swing_level = dp->host.vol_swing_level;
	u8 *pre_empha_level = dp->host.pre_empha_level;
	u8 *max_reach_value = dp->host.max_reach_value;

	u8 lanes_data[MAX_LANE_CNT]; // before_cr
	u8 link_status[DP_LINK_STATUS_SIZE]; // after_cr
	bool cr_done;
	bool same_before_adjust, max_swing_reached, try_max_swing = false;
	u8 fail_counter = 0;
	int i;

	dp_info(dp, "Link Training EQ Phase with BW(%u) and Lanes(%u)\n",
		dp->link.link_rate, dp->link.num_lanes);

	dp_init_link_training_eq(dp, tps);

	do {
		// Set Voltage
		dp_print_swing_level(dp);
		dp_hw_set_voltage_and_pre_emphasis(&dp->hw_config,
						   (u8 *)vol_swing_level,
						   (u8 *)pre_empha_level);

		// Write Link Training
		for (i = 0; i < dp->link.num_lanes; i++)
			lanes_data[i] = (pre_empha_level[i]
					 << DP_TRAIN_PRE_EMPHASIS_SHIFT) |
					vol_swing_level[i] | max_reach_value[i];
		drm_dp_dpcd_write(&dp->dp_aux, DP_TRAINING_LANE0_SET,
				  lanes_data, dp->link.num_lanes);

		udelay(interval_us);

		// Read Link Status
		drm_dp_dpcd_read_link_status(&dp->dp_aux, link_status);

		// Validate EQ
		dp_validate_link_training(dp, &cr_done, &same_before_adjust,
					  &max_swing_reached, lanes_data,
					  link_status);

		if (max_swing_reached) {
			dp_info(dp, "requested to adjust max swing level\n");
			if (!try_max_swing) {
				try_max_swing = true;
				continue;
			} else {
				dp_err(dp, "reached max swing level\n");
				goto err;
			}
		}

		if (cr_done) {
			if (drm_dp_channel_eq_ok(link_status,
						 dp->link.num_lanes)) {
				dp_print_swing_level(dp);
				dp_info(dp,
					"EQ Done. Move to Training_Done.\n");
				return true;
			}
		}

		fail_counter++;
	} while (fail_counter < 5);

err:
	dp_err(dp, "failed Link Training EQ phase with BW(%u) and Lanes(%u)\n",
	       dp->link.link_rate, dp->link.num_lanes);
	return false;
}

static void dp_get_lower_link_rate(struct dp_link *link)
{
	switch (drm_dp_link_rate_to_bw_code(link->link_rate)) {
	case DP_LINK_BW_2_7:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62);
		break;
	case DP_LINK_BW_5_4:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7);
		break;
	case DP_LINK_BW_8_1:
		link->link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4);
		break;
	}
}

static int dp_do_full_link_training(struct dp_device *dp, u32 interval_us)
{
	const u8 supported_tps = dp_get_supported_pattern(dp);
	bool training_done = false;

	do {
		// Link Training: CR (Clock Revovery)
		if (!dp_do_link_training_cr(dp, interval_us)) {
			if (drm_dp_link_rate_to_bw_code(dp->link.link_rate) !=
			    DP_LINK_BW_1_62) {
				dp_get_lower_link_rate(&dp->link);

				dp_info(dp,
					"reducing link rate to %u during CR phase\n",
					dp->link.link_rate);
				continue;
			} else if (dp->link.num_lanes > 1) {
				dp->link.num_lanes >>= 1;
				dp->link.link_rate = dp_get_max_link_rate(dp);

				dp_info(dp,
					"reducing lanes number to %u during CR phase\n",
					dp->link.num_lanes);
				continue;
			}

			dp_err(dp, "Link training failed during CR phase\n");
			goto err;
		}

		// Link Training: Channel Equalization
		if (!dp_do_link_training_eq(dp, interval_us, supported_tps)) {
			if (dp->link.num_lanes > 1) {
				dp->link.num_lanes >>= 1;

				dp_info(dp,
					"reducing lanes number to %u during EQ phase\n",
					dp->link.num_lanes);
				continue;
			} else if (drm_dp_link_rate_to_bw_code(
					   dp->link.link_rate) !=
				   DP_LINK_BW_1_62) {
				dp_get_lower_link_rate(&dp->link);
				dp->link.num_lanes = dp_get_max_num_lanes(dp);

				dp_info(dp,
					"reducing link rate to %u during EQ phase\n",
					dp->link.link_rate);
				continue;
			}

			dp_err(dp, "Link training failed during EQ phase\n");
			goto err;
		}

		training_done = true;
	} while (!training_done);

	dp_info(dp, "Link training is done with BW(%u) and Lanes(%u)\n",
		dp->link.link_rate, dp->link.num_lanes);

	dp_hw_set_training_pattern(NORAMAL_DATA);
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   dp->host.scrambler ? 0 : DP_LINK_SCRAMBLING_DISABLE);

	return 0;
err:
	dp_info(dp, "Link training is failed\n");

	dp_hw_set_training_pattern(NORAMAL_DATA);
	drm_dp_dpcd_writeb(&dp->dp_aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	return -EIO;
}

static int dp_link_up(struct dp_device *dp)
{
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	u8 val = 0;
	unsigned int addr;
	u32 interval, interval_us;
	int ret;

	mutex_lock(&dp->training_lock);

	// Read DP Sink device's Capabilities
	drm_dp_dpcd_readb(&dp->dp_aux, DP_TRAINING_AUX_RD_INTERVAL, &val);
	if (val & DP_EXTENDED_RECEIVER_CAP_FIELD_PRESENT)
		addr = DP_DP13_DPCD_REV;
	else
		addr = DP_DPCD_REV;

	ret = drm_dp_dpcd_read(&dp->dp_aux, addr, dpcd, DP_RECEIVER_CAP_SIZE);
	if (ret < 0) {
		dp_err(dp, "failed to read DP Sink device capabilities\n");
		mutex_unlock(&dp->training_lock);
		return ret;
	}

	dp->link.revision = dpcd[0];
	dp->link.link_rate = drm_dp_max_link_rate(dpcd);
	dp->link.num_lanes = drm_dp_max_lane_count(dpcd);
	dp->link.enhanced_frame = drm_dp_enhanced_frame_cap(dpcd);
	dp_info(dp, "DP Sink: DPCD_Rev_%X, Rate(%u Mbps), Num_lanes(%u)\n",
		dp->link.revision, dp->link.link_rate / 100,
		dp->link.num_lanes);

	// Power DP Sink device Up
	dp_sink_power_up(dp, true);

	// Set & Adjust Sink Capabilities
	dp_fill_sink_caps(dp, dpcd);
	dp->sink.link_rate = min(dp->host.link_rate, dp->sink.link_rate);
	dp->sink.num_lanes = min(dp->host.num_lanes, dp->sink.num_lanes);

	// Link Training
	interval = dpcd[DP_TRAINING_AUX_RD_INTERVAL] & DP_TRAINING_AUX_RD_MASK;
	interval_us = dp_get_training_interval_us(dp, interval);
	if (!interval_us || dp_do_full_link_training(dp, interval_us)) {
		dp_err(dp, "failed to train DP Link\n");
		mutex_unlock(&dp->training_lock);
		return -EIO;
	}

	mutex_unlock(&dp->training_lock);
	return 0;
}

static int dp_link_down(struct dp_device *dp)
{
	if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG) {
		dp_sink_power_up(dp, false);
	}

	return 0;
}

static enum bit_depth dp_get_bpc(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_display_info *display_info = &connector->display_info;

	if (display_info->bpc == 8)
		return BPC_8;
	else if (display_info->bpc == 10)
		return BPC_10;
	else
		return BPC_6;
}

static void dp_set_video_timing(struct dp_device *dp)
{
	struct videomode vm;

	drm_display_mode_to_videomode(&dp->cur_mode, &vm);

	dp->hw_config.vtiming.clock = dp->cur_mode.clock;

	dp->hw_config.vtiming.htotal = dp->cur_mode.htotal;
	dp->hw_config.vtiming.vtotal = dp->cur_mode.vtotal;
	dp->hw_config.vtiming.hfp = vm.hfront_porch;
	dp->hw_config.vtiming.hbp = vm.hback_porch;
	dp->hw_config.vtiming.hactive = vm.hactive;
	dp->hw_config.vtiming.vfp = vm.vfront_porch;
	dp->hw_config.vtiming.vbp = vm.vback_porch;
	dp->hw_config.vtiming.vactive = vm.vactive;

	dp->hw_config.vtiming.vsync_polarity =
		(dp->cur_mode.flags & DRM_MODE_FLAG_NVSYNC) ? SYNC_NEGATIVE :
							      SYNC_POSITIVE;
	dp->hw_config.vtiming.hsync_polarity =
		(dp->cur_mode.flags & DRM_MODE_FLAG_NHSYNC) ? SYNC_NEGATIVE :
							      SYNC_POSITIVE;
}

static u8 dp_get_vic(struct dp_device *dp)
{
	dp->cur_mode_vic = drm_match_cea_mode(&dp->cur_mode);
	return dp->cur_mode_vic;
}

static int dp_make_avi_infoframe_data(struct dp_device *dp,
				      struct infoframe *avi_infoframe)
{
	int i;

	avi_infoframe->type_code = INFOFRAME_PACKET_TYPE_AVI;
	avi_infoframe->version_number = AVI_INFOFRAME_VERSION;
	avi_infoframe->length = AVI_INFOFRAME_LENGTH;

	for (i = 0; i < AVI_INFOFRAME_LENGTH; i++)
		avi_infoframe->data[i] = 0x00;

	avi_infoframe->data[0] |= ACTIVE_FORMAT_INFO_PRESENT;
	avi_infoframe->data[1] |= ACTIVE_PORTION_ASPECT_RATIO;
	avi_infoframe->data[3] = dp_get_vic(dp);

	return 0;
}

static int dp_set_avi_infoframe(struct dp_device *dp)
{
	struct infoframe avi_infoframe;

	dp_make_avi_infoframe_data(dp, &avi_infoframe);
	dp_hw_send_avi_infoframe(avi_infoframe);

	return 0;
}

static int dp_make_spd_infoframe_data(struct infoframe *spd_infoframe)
{
	spd_infoframe->type_code = 0x83;
	spd_infoframe->version_number = 0x1;
	spd_infoframe->length = 25;

	strncpy(&spd_infoframe->data[0], "SEC.GED", 8);

	return 0;
}

static int dp_set_spd_infoframe(void)
{
	struct infoframe spd_infoframe;

	memset(&spd_infoframe, 0, sizeof(spd_infoframe));
	dp_make_spd_infoframe_data(&spd_infoframe);
	dp_hw_send_spd_infoframe(spd_infoframe);

	return 0;
}

static void dp_enable(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);

	mutex_lock(&dp->cmd_lock);

	dp->hw_config.bpc = dp_get_bpc(dp);
	dp->hw_config.range = VESA_RANGE;
	dp_set_video_timing(dp);

	if (dp->bist_used) {
		dp->hw_config.bist_mode = true;
		dp->hw_config.bist_type = COLOR_BAR;

		dp_hw_set_bist_video_config(&dp->hw_config);
	}

	dp_set_avi_infoframe(dp);
	dp_set_spd_infoframe();

	dp_hw_start();
	dp_info(dp, "enabled DP as cur_mode = %s@%d\n", dp->cur_mode.name,
		drm_mode_vrefresh(&dp->cur_mode));

	dp->state = DP_STATE_RUN;
	dp_info(dp, "%s: DP State changed to RUN\n", __func__);

	mutex_unlock(&dp->cmd_lock);
}

static void dp_disable(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);

	mutex_lock(&dp->cmd_lock);

	if (dp->state == DP_STATE_RUN) {
		dp_hw_stop();
		dp_info(dp, "disabled DP\n");

		dp->state = DP_STATE_ON;
		dp_info(dp, "%s: DP State changed to ON\n", __func__);
	} else
		dp_info(dp, "%s: DP State is not RUN\n", __func__);

	mutex_unlock(&dp->cmd_lock);
}

// For BIST
static void dp_parse_edid(struct dp_device *dp, struct edid *edid)
{
	u8 *edid_vendor = dp->sink.edid_manufacturer;
	u32 edid_prod_id = 0;

	edid_vendor[0] = ((edid->mfg_id[0] & 0x7c) >> 2) + '@';
	edid_vendor[1] = (((edid->mfg_id[0] & 0x3) << 3) |
			  ((edid->mfg_id[1] & 0xe0) >> 5)) +
			 '@';
	edid_vendor[2] = (edid->mfg_id[1] & 0x1f) + '@';

	edid_prod_id |= EDID_PRODUCT_ID(edid);

	dp->sink.edid_product = edid_prod_id;
	dp->sink.edid_serial = edid->serial;

	drm_edid_get_monitor_name(edid, dp->sink.sink_name, SINK_NAME_LEN);

	dp_info(dp, "Sink Manufacturer: %s\n", dp->sink.edid_manufacturer);
	dp_info(dp, "Sink Product: %x\n", dp->sink.edid_product);
	dp_info(dp, "Sink Serial: %x\n", dp->sink.edid_serial);
	dp_info(dp, "Sink Name: %s\n", dp->sink.sink_name);
}

static void dp_clean_drm_modes(struct dp_device *dp)
{
	struct drm_display_mode *mode, *t;
	struct drm_connector *connector = &dp->connector;

	memset(&dp->cur_mode, 0, sizeof(struct drm_display_mode));
	memset(&dp->pref_mode, 0, sizeof(struct drm_display_mode));

	list_for_each_entry_safe (mode, t, &connector->probed_modes, head) {
		list_del(&mode->head);
		drm_mode_destroy(connector->dev, mode);
	}
}

static const struct drm_display_mode mode_vga[1] = {
	/* 1 - 640x480@60Hz 4:3 */
	{
		DRM_MODE("640x480", DRM_MODE_TYPE_DRIVER, 25175, 640, 656, 752,
			 800, 0, 480, 490, 492, 525, 0,
			 DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
		.picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3,
	}
};

static void dp_mode_set_fail_safe(struct dp_device *dp)
{
	dp->hw_config.bpc = BPC_6;
	drm_mode_copy(&dp->cur_mode, mode_vga);
	dp->fail_safe = true;
}

static bool dp_find_prefer_mode(struct dp_device *dp)
{
	struct drm_display_mode *mode, *t;
	struct drm_connector *connector = &dp->connector;
	bool found = false;

	list_for_each_entry_safe (mode, t, &connector->probed_modes, head) {
		if ((mode->type & DRM_MODE_TYPE_PREFERRED)) {
			dp_info(dp,
				"pref: %s@%d, type: %d, stat: %d, ratio: %d\n",
				mode->name, drm_mode_vrefresh(mode), mode->type,
				mode->status, mode->picture_aspect_ratio);
			drm_mode_copy(&dp->pref_mode, mode);
			drm_mode_copy(&dp->cur_mode, mode);
			found = true;
			break;
		}
	}

	if (!found) {
		dp_info(dp, "there are no valid mode, fail-safe\n");
		dp_mode_set_fail_safe(dp);
	}

	return found;
}

static void dp_on_by_hpd_plug(struct dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_device *dev = connector->dev;

	if (dp->bist_used) {
		/* Parse EDID for BIST video */
		struct edid *edid =
			drm_do_get_edid(connector, dp_get_edid_block, dp);
		int num_modes;

		if (edid) {
			dp_parse_edid(dp, edid);

			mutex_lock(&dev->mode_config.mutex);
			dp_clean_drm_modes(dp);
			num_modes = drm_add_edid_modes(connector, edid);
			mutex_unlock(&dev->mode_config.mutex);
			kfree(edid);

			if (num_modes)
				dp_find_prefer_mode(dp);
		}

		dp->state = DP_STATE_ON;
		dp_info(dp, "%s: DP State changed to ON\n", __func__);

		/* Enable BIST video */
		dp_enable(&dp->encoder);
	}
}

static void dp_off_by_hpd_plug(struct dp_device *dp)
{
	if (dp->state == DP_STATE_RUN) {
		/* Disable video */
		dp_disable(&dp->encoder);
	}
}

/* Delayed Works */
static void dp_work_hpd_plug(struct work_struct *work)
{
	struct dp_device *dp = get_dp_drvdata();

	if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG) {
		mutex_lock(&dp->hpd_lock);

		pm_runtime_get_sync(dp->dev);
		dp_debug(dp, "pm_rtm_get_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));
		pm_stay_awake(dp->dev);

		/* PHY power on */
		usleep_range(10000, 10030);
		dp_hw_init(&dp->hw_config); /* for AUX ch read/write. */
		usleep_range(10000, 11000);

		if (dp_check_dp_sink(dp) < 0) {
			dp_err(dp, "failed to check DP Sink status\n");
			goto HPD_FAIL;
		}

		if (dp_link_up(dp)) {
			dp_err(dp, "failed to DP Link Up\n");
			goto HPD_FAIL;
		}

		dp_on_by_hpd_plug(dp);

		mutex_unlock(&dp->hpd_lock);
	}

	return;

HPD_FAIL:
	dp_err(dp, "HPD FAIL Check CCIC or USB!!\n");
	dp_set_hpd_state(dp, EXYNOS_HPD_UNPLUG);
	dp_info(dp, "DP HPD changed to EXYNOS_HPD_UNPLUG\n");

	dp_hw_deinit();
	pm_relax(dp->dev);

	dp_init_info(dp);
	pm_runtime_put_sync(dp->dev);
	dp_debug(dp, "pm_rtm_put_sync usage_cnt(%d)\n",
		 atomic_read(&dp->dev->power.usage_count));

	/* in error case, add delay to avoid very short interval reconnection */
	msleep(300);

	mutex_unlock(&dp->hpd_lock);
}

static void dp_work_hpd_unplug(struct work_struct *work)
{
	struct dp_device *dp = get_dp_drvdata();

	if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
		mutex_lock(&dp->hpd_lock);

		dp_off_by_hpd_plug(dp);
		dp_link_down(dp);

		/* PHY power off */
		dp_hw_deinit();

		pm_runtime_put_sync(dp->dev);
		dp_debug(dp, "pm_rtm_put_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));
		pm_relax(dp->dev);

		dp->state = DP_STATE_INIT;
		dp_info(dp, "%s: DP State changed to INIT\n", __func__);

		mutex_unlock(&dp->hpd_lock);
	}
}

/* ExtCon Handshaking Functions */
static void dp_hpd_changed(struct dp_device *dp, enum hotplug_state state)
{
	if (dp_get_hpd_state(dp) == state) {
		dp_debug(dp, "DP HPD is same state (%x): Skip\n", state);
		return;
	}

	if (state == EXYNOS_HPD_PLUG) {
		dp_info(dp, "DP HPD changed to EXYNOS_HPD_PLUG\n");
		dp_set_hpd_state(dp, state);
		queue_delayed_work(dp->dp_wq, &dp->hpd_plug_work, 0);
	} else if (state == EXYNOS_HPD_UNPLUG) {
		dp_info(dp, "DP HPD changed to EXYNOS_HPD_UNPLUG\n");
		dp_set_hpd_state(dp, state);
		queue_delayed_work(dp->dp_wq, &dp->hpd_unplug_work, 0);
	} else
		dp_err(dp, "DP HPD changed to abnormal state(%d)\n", state);
}

static int usb_typec_dp_notification(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct dp_device *dp = container_of(nb, struct dp_device, dp_typec_nb);
	union extcon_property_value property = { 0 };
	int ret;

	if (action == 1) {
		ret = extcon_get_property(dp->edev, EXTCON_DISP_DP,
					  EXTCON_PROP_DISP_HPD, &property);
		if (!ret) {
			if (property.intval == 1) {
				dp_info(dp,
					"%s: USB Type-C is HPD PLUG status\n",
					__func__);

				if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
					memset(&dp->hw_config, 0,
					       sizeof(struct dp_hw_config));

					ret = extcon_get_property(
						dp->edev, EXTCON_DISP_DP,
						EXTCON_PROP_DISP_ORIENTATION,
						&property);
					if (!ret) {
						dp_info(dp,
							"%s: disp_orientation = %d\n",
							__func__,
							property.intval);
						dp->hw_config.orient_type =
							(enum plug_orientation)(
								property.intval);
					}

					ret = extcon_get_property(
						dp->edev, EXTCON_DISP_DP,
						EXTCON_PROP_DISP_PIN_CONFIG,
						&property);
					if (!ret) {
						dp_info(dp,
							"%s: disp_pin_config = %d\n",
							__func__,
							property.intval);
						dp->hw_config.pin_type =
							(enum pin_assignment)(
								property.intval);

						if (dp->hw_config.pin_type ==
							    PIN_TYPE_A ||
						    dp->hw_config.pin_type ==
							    PIN_TYPE_C ||
						    dp->hw_config.pin_type ==
							    PIN_TYPE_E)
							dp->hw_config.num_lanes =
								4;
						else
							dp->hw_config.num_lanes =
								2;
					}

					dp_hpd_changed(dp, EXYNOS_HPD_PLUG);
				} else
					dp_warn(dp, "%s: HPD is already set\n",
						__func__);
			} else {
				dp_info(dp,
					"%s: USB Type-C is HPD UNPLUG status\n",
					__func__);

				if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG)
					dp_hpd_changed(dp, EXYNOS_HPD_UNPLUG);
			}
		} else
			dp_err(dp, "%s: extcon_get_property() is failed\n",
			       __func__);
	} else {
		dp_info(dp, "%s: USB Type-C is not in display ALT mode\n",
			__func__);

		if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG)
			dp_hpd_changed(dp, EXYNOS_HPD_UNPLUG);
	}

	return NOTIFY_OK;
}

static void dp_register_extcon_notifier(void)
{
	struct dp_device *dp = get_dp_drvdata();
	int ret;

	if (!dp->notifier_registered) {
		dp->dp_typec_nb.notifier_call = usb_typec_dp_notification;
		ret = extcon_register_notifier(dp->edev, EXTCON_DISP_DP,
					       &dp->dp_typec_nb);
		if (ret < 0)
			dp_err(dp, "failed to register PDIC Notifier\n");

		dp->notifier_registered = 1;
		dp_debug(dp, "PDIC Notifier has been registered\n");
	}
}

/* DP DRM Connector Helper Functions */
static enum drm_mode_status dp_mode_valid(struct drm_encoder *encoder,
					  const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void dp_atomic_mode_set(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
}

static int dp_atomic_check(struct drm_encoder *encoder,
			   struct drm_crtc_state *crtc_state,
			   struct drm_connector_state *state)
{
	return 0;
}

static const struct drm_encoder_helper_funcs dp_encoder_helper_funcs = {
	.mode_valid = dp_mode_valid,
	.atomic_mode_set = dp_atomic_mode_set,
	.enable = dp_enable,
	.disable = dp_disable,
	.atomic_check = dp_atomic_check,
};

/* DP DRM Encoder Functions */
static const struct drm_encoder_funcs dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

/* DP DRM Connector Functions */
static const struct drm_connector_funcs dp_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* DP DRM Connector Helper Functions */
static int dp_detect(struct drm_connector *connector,
		     struct drm_modeset_acquire_ctx *ctx, bool force)
{
	return connector_status_unknown;
}

static int dp_get_modes(struct drm_connector *connector)
{
	return 0;
}

static const struct drm_connector_helper_funcs dp_connector_helper_funcs = {
	.detect_ctx = dp_detect,
	.get_modes = dp_get_modes,
};

/* DP DRM Component Functions */
static int dp_create_connector(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_connector *connector = &dp->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector, &dp_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dp_err(dp, "failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &dp_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static int dp_create_dp_aux(struct dp_device *dp)
{
	drm_dp_aux_init(&dp->dp_aux);
	dp->dp_aux.dev = dp->dev;
	dp->dp_aux.transfer = dp_aux_transfer;

	return 0;
}

static int dp_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_device *drm_dev = data;
	int ret = 0;

	drm_encoder_init(drm_dev, encoder, &dp_encoder_funcs,
			 DRM_MODE_ENCODER_LVDS, NULL);
	drm_encoder_helper_add(encoder, &dp_encoder_helper_funcs);

	encoder->possible_crtcs =
		exynos_drm_get_possible_crtcs(encoder, dp->output_type);
	if (!encoder->possible_crtcs) {
		dp_err(dp, "failed to get possible crtc, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return -ENOTSUPP;
	}

	ret = dp_create_connector(encoder);
	if (ret) {
		dp_err(dp, "failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	ret = dp_create_dp_aux(dp);
	if (ret) {
		dp_err(dp, "failed to create dp_aux ret = %d\n", ret);
		return ret;
	}

	dp_fill_host_caps(dp);
	dp_info(dp, "DP Driver has been binded\n");

	dp_register_extcon_notifier();

	return ret;
}

static void dp_unbind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);

	dp_debug(dp, "%s +\n", __func__);
	dp_debug(dp, "%s -\n", __func__);
}

static const struct component_ops dp_component_ops = {
	.bind = dp_bind,
	.unbind = dp_unbind,
};

/* DP DRM Driver */
static int dp_parse_dt(struct dp_device *dp, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dp_err(dp, "no device tree information\n");
		return -EINVAL;
	}

	if (!of_property_read_bool(dev->of_node, "extcon")) {
		dp_err(dp, "no extcon in device tree\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t dp_irq_handler(int irq, void *dev_data)
{
	return IRQ_HANDLED;
}

static int dp_init_resources(struct dp_device *dp, struct platform_device *pdev)
{
	struct resource *res;
	u64 addr_phy = 0x110F0000;
	u64 size_phy = 0x2800;
	int ret = 0;

	/* DP Link SFR */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dp_err(dp, "failed to get mem resource\n");
		return -ENOENT;
	}

	dp->res.link_regs = devm_ioremap_resource(dp->dev, res);
	if (IS_ERR(dp->res.link_regs)) {
		dp_err(dp, "failed to remap DP LINK SFR region\n");
		return -EINVAL;
	}
	dp_regs_desc_init(dp->res.link_regs, res->start, "LINK", REGS_LINK,
			  SST1);

	/* USBDP Combo PHY SFR */
	// ToDo: USBDP PHY is shared with USB Driver.
	// In order to avoid double resource mapping, here uses hard-code.
	// Need to revisit
	dp->res.phy_regs = ioremap((phys_addr_t)addr_phy, size_phy);
	if (IS_ERR(dp->res.phy_regs)) {
		dp_err(dp, "failed to remap USBDP Combo PHY SFR region\n");
		return -EINVAL;
	}
	dp_regs_desc_init(dp->res.phy_regs, (phys_addr_t)addr_phy, "PHY",
			  REGS_PHY, SST1);

	/* DP Interrupt */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dp_err(dp, "failed to get irq resource\n");
		return -ENOENT;
	}

	dp->res.irq = res->start;
	ret = devm_request_irq(dp->dev, res->start, dp_irq_handler, 0,
			       pdev->name, dp);
	if (ret) {
		dp_err(dp, "failed to install DP irq\n");
		return -EINVAL;
	}
	disable_irq(dp->res.irq);

	return 0;
}

static int dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dp_device *dp = NULL;
	int ret = 0;

	dp = devm_kzalloc(dev, sizeof(struct dp_device), GFP_KERNEL);
	if (!dp) {
		dev_err(dev, "failed to allocate dp device.\n");
		return -ENOMEM;
	}
	dp->dev = dev;

	ret = dp_parse_dt(dp, dev);
	if (ret) {
		dp_err(dp, "failed to parse dp device tree.\n");
		return ret;
	}

	ret = dp_init_resources(dp, pdev);
	if (ret) {
		dp_err(dp, "failed to init dp resources.\n");
		return ret;
	}

	platform_set_drvdata(pdev, dp);

	/* Driver Initialization */
	dp_drvdata = dp;
	dp_init_info(dp);

	dma_set_mask(dev, DMA_BIT_MASK(32));

	mutex_init(&dp->cmd_lock);
	mutex_init(&dp->hpd_lock);
	mutex_init(&dp->hpd_state_lock);
	mutex_init(&dp->training_lock);

	/* Create WorkQueue & Delayed Works*/
	dp->dp_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!dp->dp_wq) {
		dp_err(dp, "create wq failed.\n");
		ret = -ENOMEM;
		goto err;
	}

	INIT_DELAYED_WORK(&dp->hpd_plug_work, dp_work_hpd_plug);
	INIT_DELAYED_WORK(&dp->hpd_unplug_work, dp_work_hpd_unplug);

	/* Register callback to ExtCon */
	dp->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR_OR_NULL(dp->edev)) {
		dp_err(dp, "error while retrieving extcon dev\n");
		ret = -ENOMEM;
		goto err;
	}

	pm_runtime_enable(dev);

	dp_info(dp, "DP Driver has been probed.\n");
	return component_add(dp->dev, &dp_component_ops);

err:
	return ret;
}

static int dp_remove(struct platform_device *pdev)
{
	struct dp_device *dp = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	mutex_destroy(&dp->cmd_lock);
	mutex_destroy(&dp->hpd_lock);
	mutex_destroy(&dp->hpd_state_lock);
	mutex_destroy(&dp->training_lock);

	destroy_workqueue(dp->dp_wq);

	dp_info(dp, "DP Driver has been removed\n");
	return 0;
}

static const struct of_device_id dp_of_match[] = {
	{ .compatible = "samsung,exynos-dp" },
	{},
};
MODULE_DEVICE_TABLE(of, dp_of_match);

struct platform_driver dp_driver
	__refdata = { .probe = dp_probe,
		      .remove = dp_remove,
		      .driver = {
			      .name = "exynos-drmdp",
			      .owner = THIS_MODULE,
			      .of_match_table = of_match_ptr(dp_of_match),
		      } };

MODULE_AUTHOR("YongWook Shin <yongwook.shin@samsung.com>");
MODULE_DESCRIPTION("Samusung DisplayPort driver");
MODULE_LICENSE("GPL");
