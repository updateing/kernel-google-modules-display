// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Display Port driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <cal_config.h>
#include <dp_cal.h>

#include "regs-dp.h"

/* DP register access functions */
static struct cal_regs_desc regs_dp[REGS_DP_TYPE_MAX][SST_MAX];

#define dp_regs_desc(id)	  (&regs_dp[REGS_LINK][id])
#define dp_read(id, offset)	  cal_read(dp_regs_desc(id), offset)
#define dp_write(id, offset, val) cal_write(dp_regs_desc(id), offset, val)
#define dp_read_mask(id, offset, mask)                                         \
	cal_read_mask(dp_regs_desc(id), offset, mask)
#define dp_write_mask(id, offset, val, mask)                                   \
	cal_write_mask(dp_regs_desc(id), offset, val, mask)

/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DP_TYPE_MAX, SST_MAX);
	cal_regs_desc_set(regs_dp, regs, start, name, type, id);
}

/*
 * USBDP Combo PHY Control Functions
 */

/*
 * DP Link Control Functions
 */

/* System Common Registers */
// System Device Version
static u32 dp_reg_get_version(void)
{
	return dp_read(SST1, SYSTEM_DEVICE_VERSION);
}

// System SW Reset Control Configuration
static void dp_reg_set_swreset(void)
{
	u32 val;

	dp_write_mask(SST1, SYSTEM_SW_RESET_CONTROL, SW_RESET, SW_RESET_MASK);

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_SW_RESET_CONTROL,
				      val, !SW_RESET_GET(val), 10, 2000))
		cal_log_err(0, "%s is timeout.\n", __func__);
}

// System Clock Control Configuration
static u32 dp_reg_get_gfmux_status(void)
{
	return GFMUX_STATUS_TXCLK_GET(dp_read_mask(SST1, SYSTEM_CLK_CONTROL,
						   GFMUX_STATUS_TXCLK_MASK));
}

static void dp_reg_set_txclk_osc(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_OSCCLK,
		      TXCLK_SEL_MASK);
}
static void dp_reg_set_txclk_sel_mode_by_txclksel(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_MODE_BY_TXCLK_SEL,
		      TXCLK_SEL_MODE_MASK);
}

// System Main Link Lane Count Configuration
static void dp_reg_set_lane_count(u8 lane_cnt)
{
	dp_write_mask(SST1, SYSTEM_MAIN_LINK_LANE_COUNT,
		      LANE_COUNT_SET(lane_cnt), LANE_COUNT_SET_MASK);
}

// System SW Function Enable Configuration
static void dp_reg_set_sw_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SW_FUNCTION_ENABLE, SW_FUNC_EN_SET(en),
		      SW_FUNC_EN_MASK);
}

// System Common Module Functions Enable Configuration
static void dp_reg_set_hdcp22_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE,
		      HDCP22_FUNC_EN_SET(en), HDCP22_FUNC_EN_MASK);
}

static void dp_reg_set_hdcp13_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE,
		      HDCP13_FUNC_EN_SET(en), HDCP13_FUNC_EN_MASK);
}

static void dp_reg_set_gtc_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, GTC_FUNC_EN_SET(en),
		      GTC_FUNC_EN_MASK);
}

static void dp_reg_set_pcs_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, PCS_FUNC_EN_SET(en),
		      PCS_FUNC_EN_MASK);
}

static void dp_reg_set_aux_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_COMMON_FUNCTION_ENABLE, AUX_FUNC_EN_SET(en),
		      AUX_FUNC_EN_MASK);
}

static void dp_hw_set_initial_common_funcs(void)
{
	dp_reg_set_hdcp22_func_en(0);
	dp_reg_set_hdcp13_func_en(0);
	dp_reg_set_gtc_func_en(0);
	dp_reg_set_pcs_func_en(0);
	dp_reg_set_aux_func_en(1);
}

// System SST1 Function Enable Configuration

// System Miscellaneous Control Configuration

// System HPD Control Configuration

// System PLL Lock Control Configuration

/* OSC Clock Divider Control Registers */
static void dp_reg_set_osc_clk_div(unsigned int mhz)
{
	dp_write_mask(SST1, OSC_CLK_DIV_HPD, HPD_EVENT_CLK_COUNT_SET(mhz),
		      HPD_EVENT_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_HDCP_10US, I2C_GEN10US_TIMER_SET(mhz),
		      I2C_GEN10US_TIMER_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_GTC_1MS, GTC_1MS_OSC_CLK_COUNT_SET(mhz),
		      GTC_1MS_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_1US, AUX_1US_OSC_CLK_COUNT_SET(mhz),
		      AUX_1US_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_MAN_UI,
		      AUX_MAN_UI_OSC_CLK_COUNT_SET(mhz),
		      AUX_MAN_UI_OSC_CLK_COUNT_MASK);
	dp_write_mask(SST1, OSC_CLK_DIV_AUX_10US,
		      AUX_10US_OSC_CLK_COUNT_SET(mhz),
		      AUX_10US_OSC_CLK_COUNT_MASK);
}

/* System Interrupt Registers */
static void dp_reg_set_common_interrupt_mask(u32 en, u32 intr_mask)
{
	if (en)
		dp_write_mask(SST1, SYSTEM_IRQ_COMMON_STATUS_MASK, intr_mask,
			      intr_mask);
	else
		dp_write_mask(SST1, SYSTEM_IRQ_COMMON_STATUS_MASK, 0,
			      intr_mask);
}

static void dp_hw_set_common_interrupt(u32 en)
{
	// Mask all common interrupts
	dp_reg_set_common_interrupt_mask(0, ~0);

	if (en) {
		// HDCP Interrupts

		// HPD Interrupts : Not used

		// AUX Interrupts

		// PLL Interrupt

		// SW Interrupt
	}
}

/* AUX Control Registers */
static void dp_reg_set_aux_reply_timeout(void)
{
	dp_write_mask(SST1, AUX_CONTROL, AUX_REPLY_TIMER_MODE_1800US,
		      AUX_REPLY_TIMER_MODE_MASK);
}

static void dp_reg_set_aux_pn(enum plug_orientation orient)
{
	u32 val = (orient == PLUG_FLIPPED) ? AUX_PN_INVERT : AUX_PN_NORMAL;

	dp_write_mask(SST1, AUX_CONTROL, val, AUX_PN_INV_MASK);
}

static int dp_reg_do_aux_transaction(void)
{
	u32 val;

	/*
	 * When write 1 to AUX_TRAN_START reg, AUX transaction starts.
	 * Once AUX transaction is completed, AUX_TRAN_START reg is auto cleared.
	 */
	dp_write_mask(SST1, AUX_TRANSACTION_START, AUX_TRAN_START,
		      AUX_TRAN_START_MASK);

	if (readl_poll_timeout_atomic(
		    regs_dp[REGS_LINK][SST1].regs + AUX_TRANSACTION_START, val,
		    !AUX_TRAN_START_GET(val), 10, 50000)) {
		cal_log_err(0, "%s is timeout.\n", __func__);
		return -ETIME;
	} else
		return 0;
}

static void dp_reg_aux_ch_buf_clr(void)
{
	dp_write_mask(SST1, AUX_BUFFER_CLEAR, AUX_BUF_CLR, AUX_BUF_CLR_MASK);
}

static void dp_reg_set_aux_ch_command(enum dp_aux_ch_command_type aux_ch_mode)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL, REQ_COMM_SET(aux_ch_mode),
		      REQ_COMM_MASK);
}

static void dp_reg_set_aux_ch_address(u32 aux_ch_address)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL, REQ_ADDR_SET(aux_ch_address),
		      REQ_ADDR_MASK);
}

static void dp_reg_set_aux_ch_length(u32 aux_ch_length)
{
	dp_write_mask(SST1, AUX_REQUEST_CONTROL,
		      REQ_LENGTH_SET(aux_ch_length - 1), REQ_LENGTH_MASK);
}

static void dp_reg_aux_defer_ctrl(u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_write_mask(SST1, AUX_COMMAND_CONTROL, DEFER_CTRL_EN_SET(val),
		      DEFER_CTRL_EN_MASK);
}

static int dp_reg_check_aux_monitors(void)
{
	u32 val0 = dp_read(SST1, AUX_MONITOR_1);
	u32 val1 = dp_read(SST1, AUX_MONITOR_2);

	if ((AUX_CMD_STATUS_GET(val0) != AUX_CMD_STATUS_OK) || val1) {
		cal_log_err(0, "AUX_MONITOR_1 : 0x%X, AUX_MONITOR_2 : 0x%X\n",
			    val0, val1);
		cal_log_err(
			0,
			"AUX_CONTROL : 0x%X, AUX_REQUEST_CONTROL : 0x%X, AUX_COMMAND_CONTROL : 0x%X\n",
			dp_read(SST1, AUX_CONTROL),
			dp_read(SST1, AUX_REQUEST_CONTROL),
			dp_read(SST1, AUX_COMMAND_CONTROL));

		usleep_range(400, 410);
		return -EIO;
	} else
		return 0;
}

static void dp_reg_aux_ch_send_buf(u8 *buf, u32 length)
{
	int i;

	for (i = 0; i < length; i++)
		dp_write_mask(SST1, AUX_TX_DATA_SET(i), TX_DATA_SET(buf[i], i),
			      TX_DATA_MASK(i));
}

static void dp_reg_aux_ch_received_buf(u8 *buf, u32 length)
{
	int i;

	for (i = 0; i < length; i++)
		buf[i] = RX_DATA_GET(dp_read_mask(SST1, AUX_RX_DATA_SET(i),
						  RX_DATA_MASK(i)),
				     i);
}

/* GTC (Global Time Code) Control Registers */

/* PCS (Scrambler/Encoder/FEC) Control Registers */
// PCS Control

// PCS Lane Control
static void dp_reg_set_lane_map(u32 lane0, u32 lane1, u32 lane2, u32 lane3)
{
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE0_MAP_SET(lane0),
		      LANE0_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE1_MAP_SET(lane1),
		      LANE1_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE2_MAP_SET(lane2),
		      LANE2_MAP_MASK);
	dp_write_mask(SST1, PCS_LANE_CONTROL, LANE3_MAP_SET(lane3),
		      LANE3_MAP_MASK);
}

static void dp_hw_set_lane_map_config(struct dp_hw_config *hw_config)
{
	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
		if (hw_config->orient_type == PLUG_NORMAL)
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_1,
					    LOGICAL_LANE_2, LOGICAL_LANE_0);
		else
			dp_reg_set_lane_map(LOGICAL_LANE_2, LOGICAL_LANE_0,
					    LOGICAL_LANE_3, LOGICAL_LANE_1);
		break;

	case PIN_TYPE_B:
		if (hw_config->orient_type == PLUG_NORMAL)
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_2,
					    LOGICAL_LANE_1, LOGICAL_LANE_0);
		else
			dp_reg_set_lane_map(LOGICAL_LANE_1, LOGICAL_LANE_0,
					    LOGICAL_LANE_2, LOGICAL_LANE_3);
		break;

	case PIN_TYPE_C:
	case PIN_TYPE_D:
	case PIN_TYPE_E:
	case PIN_TYPE_F:
		if (hw_config->orient_type == PLUG_NORMAL) {
			cal_log_info(0, "%s: Type_E & Plug_Normal\n", __func__);
			dp_reg_set_lane_map(LOGICAL_LANE_3, LOGICAL_LANE_2,
					    LOGICAL_LANE_0, LOGICAL_LANE_1);
		} else {
			cal_log_info(0, "%s: Type_E & Plug_Flipped\n",
				     __func__);
			dp_reg_set_lane_map(LOGICAL_LANE_0, LOGICAL_LANE_1,
					    LOGICAL_LANE_3, LOGICAL_LANE_2);
		}
		break;

	default:
		break;
	}
}

// PCS Test Pattern Control

/* HDCP Control Registers */

/* SST1 Control Registers */
// SST1 Main

// SST1 Interrupts

// SST1 Video M/N Value

// SST1 Active Symbol SW Control

// SST1 Video Control

// SST1 Video Timing

// SST1 Video BIST Control

// SST1 Audio Control

// SST1 InfoFrame Control

//---------------------------------------
/* DPCD (DisplayPort Configuration Data) Read/Write Interfaces through AUX channel */
static int dp_reg_set_aux_ch_operation_enable(void)
{
	if (dp_reg_do_aux_transaction())
		return -ETIME;

	if (dp_reg_check_aux_monitors())
		return -EIO;

	return 0;
}

static int dp_write_dpcd(u32 address, u32 length, u8 *data)
{
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout();
		dp_reg_set_aux_ch_command(DPCD_WRITE);
		dp_reg_set_aux_ch_address(address);
		dp_reg_set_aux_ch_length(length);
		dp_reg_aux_ch_send_buf(data, length);
		ret = dp_reg_set_aux_ch_operation_enable();
		if (ret == 0)
			break;

		retry_cnt--;
	}

	return ret;
}

static int dp_read_dpcd(u32 address, u32 length, u8 *data)
{
	int ret;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_set_aux_ch_command(DPCD_READ);
		dp_reg_set_aux_ch_address(address);
		dp_reg_set_aux_ch_length(length);
		dp_reg_aux_ch_buf_clr();
		dp_reg_aux_defer_ctrl(1);
		dp_reg_set_aux_reply_timeout();
		ret = dp_reg_set_aux_ch_operation_enable();
		if (ret == 0)
			break;

		retry_cnt--;
	}

	if (ret == 0)
		dp_reg_aux_ch_received_buf(data, length);

	return ret;
}

int dp_hw_write_dpcd_burst(u32 address, u32 length, u8 *data)
{
	int ret = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;
	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_write_dpcd(address + i, buf_length, data + i);
		if (ret != 0) {
			cal_log_err(0, "dp_reg_dpcd_write_burst fail\n");
			break;
		}
	}

	return ret;
}

int dp_hw_read_dpcd_burst(u32 address, u32 length, u8 *data)
{
	int ret = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;

	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_read_dpcd(address + i, buf_length, data + i);
		if (ret != 0) {
			cal_log_err(0, "dp_reg_dpcd_read_burst fail\n");
			break;
		}
	}

	return ret;
}

/* DP Hardware Control Interfaces */
void dp_hw_init(struct dp_hw_config *hw_config)
{
	cal_log_info(0, "DP Link Version = 0x%X\n", dp_reg_get_version());

	/* Apply Soft Reset */
	dp_reg_set_swreset();
	cal_log_debug(0, "reset DP Link\n");

	/* Set system clock to OSC */
	dp_reg_set_txclk_sel_mode_by_txclksel();
	dp_reg_set_osc_clk_div(OSC_CLK);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC with %d MHz: Mux(%u)\n",
		      OSC_CLK, dp_reg_get_gfmux_status());

	/*
	 * USBDP PHY Initialization
	 */

	/*
	 * DP Link Initialization
	 */
	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type);
	dp_hw_set_lane_map_config(hw_config);
	cal_log_debug(0, "set lane count & map\n");

	/* Set System Common Functions Enable */
	dp_hw_set_initial_common_funcs();
	cal_log_debug(0, "set common function\n");

	/* Set System SW Functions Enable */
	dp_reg_set_sw_func_en(1);
	cal_log_debug(0, "set sw function\n");

	/* Set Interrupts */
	dp_hw_set_common_interrupt(1);
	cal_log_debug(0, "set interrupts\n");
}

void dp_hw_deinit(void)
{
	dp_hw_set_common_interrupt(0);
	dp_reg_set_sw_func_en(0);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());
}
