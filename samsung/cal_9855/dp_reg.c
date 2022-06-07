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
#include "regs-usbdpphy.h"

/* DP register access functions */
static struct cal_regs_desc regs_dp[REGS_DP_TYPE_MAX][SST_MAX];

#define dp_regs_desc(id)	  (&regs_dp[REGS_LINK][id])
#define dp_read(id, offset)	  cal_read(dp_regs_desc(id), offset)
#define dp_write(id, offset, val) cal_write(dp_regs_desc(id), offset, val)
#define dp_read_mask(id, offset, mask)                                         \
	cal_read_mask(dp_regs_desc(id), offset, mask)
#define dp_write_mask(id, offset, val, mask)                                   \
	cal_write_mask(dp_regs_desc(id), offset, val, mask)

#define dp_phy_regs_desc(id)	(&regs_dp[REGS_PHY][id])
#define dp_phy_read(id, offset) cal_read(dp_phy_regs_desc(id), offset)
#define dp_phy_write(id, offset, val)                                          \
	cal_write(dp_phy_regs_desc(id), offset, val)
#define dp_phy_read_mask(id, offset, mask)                                     \
	cal_read_mask(dp_phy_regs_desc(id), offset, mask)
#define dp_phy_write_mask(id, offset, val, mask)                               \
	cal_write_mask(dp_phy_regs_desc(id), offset, val, mask)

/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id)
{
	cal_regs_desc_check(type, id, REGS_DP_TYPE_MAX, SST_MAX);
	cal_regs_desc_set(regs_dp, regs, start, name, type, id);
}

u32 phy_tune_parameters[4][4][5] = {
	/* {amp, post, pre, idrv, accdrv} */
	{
		/* Swing Level_0 */
		{ 0x22, 0x10, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_0 */
		{ 0x26, 0x15, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_1 */
		{ 0x26, 0x17, 0x43, 0x82, 0x00 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x1C, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_1 */
		{ 0x26, 0x10, 0x42, 0x82, 0x00 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x14, 0x42, 0x83, 0x30 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x18, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x18, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_2 */
		{ 0x2A, 0x10, 0x42, 0x83, 0x30 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x17, 0x43, 0x83, 0x38 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_3 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x10, 0x43, 0x83, 0x30 }, /* Pre-emphasis Level_3 */
	},
};

/*
 * USBDP Combo PHY Control Functions
 */

/* USBDP PHY Common Registers */
static void dpphy_reg_reset(u32 en)
{
	dp_phy_write_mask(SST1, CMN_REG00BD, DP_INIT_RSTN_SET(~en),
			  DP_INIT_RSTN_MASK);
	udelay(2);
	dp_phy_write_mask(SST1, CMN_REG00BD, DP_CMN_RSTN_SET(~en),
			  DP_CMN_RSTN_MASK);
	udelay(2);
}

static void dpphy_reg_set_default_config(struct dp_hw_config *hw_config)
{
	dp_phy_write_mask(SST1, CMN_REG0008, OVRD_AUX_DISABLE,
			  OVRD_AUX_EN_MASK);

	/* DP Default Settings */
	dp_phy_write_mask(SST1, CMN_REG0189, DP_TX_CLK_INV_RBR_SET(15),
			  DP_TX_CLK_INV_RBR_MASK);
	dp_phy_write_mask(SST1, CMN_REG0189, DP_TX_CLK_INV_HBR_SET(15),
			  DP_TX_CLK_INV_HBR_MASK);
	dp_phy_write_mask(SST1, CMN_REG018A, DP_TX_CLK_INV_HBR2_SET(15),
			  DP_TX_CLK_INV_HBR2_MASK);
	dp_phy_write_mask(SST1, CMN_REG018A, DP_TX_CLK_INV_HBR3_SET(15),
			  DP_TX_CLK_INV_HBR3_MASK);
	dp_phy_write_mask(SST1, CMN_REG005E, ANA_ROPLL_ANA_VCI_SEL_SET(1),
			  ANA_ROPLL_ANA_VCI_SEL_MASK);

	/* ROPLL for DP Settings for 19.2MHz Ref.Clk */
	// ROPLL_PMS_MDIV
	dp_phy_write(SST1, CMN_REG0066, 0xD4);
	dp_phy_write(SST1, CMN_REG0067, 0x8E);
	dp_phy_write(SST1, CMN_REG0068, 0x8E);
	dp_phy_write(SST1, CMN_REG0069, 0xD4);
	// ROPLL_PMS_MDIV_AFC
	dp_phy_write(SST1, CMN_REG006C, 0xD4);
	dp_phy_write(SST1, CMN_REG006D, 0x8E);
	dp_phy_write(SST1, CMN_REG006E, 0x8E);
	dp_phy_write(SST1, CMN_REG006F, 0xD4);
	// ROPLL_SDM_DENOMINATOR
	dp_phy_write(SST1, CMN_REG007B, 0x2C);
	dp_phy_write(SST1, CMN_REG007C, 0x14);
	dp_phy_write(SST1, CMN_REG007D, 0x14);
	dp_phy_write(SST1, CMN_REG007E, 0x2C);
	// ROPLL_SDM_NUMERATOR
	dp_phy_write(SST1, CMN_REG0082, 0x11);
	dp_phy_write(SST1, CMN_REG0083, 0x0A);
	dp_phy_write(SST1, CMN_REG0084, 0x0A);
	dp_phy_write(SST1, CMN_REG0085, 0x11);
	// ROPLL_SDC_N
	dp_phy_write(SST1, CMN_REG0087, 0x01);
	dp_phy_write(SST1, CMN_REG0088, 0x00);
	dp_phy_write(SST1, CMN_REG0089, 0x03);
	// ROPLL_SDC_NUMERATOR
	dp_phy_write(SST1, CMN_REG008C, 0x02);
	dp_phy_write(SST1, CMN_REG008D, 0x09);
	dp_phy_write(SST1, CMN_REG008E, 0x09);
	dp_phy_write(SST1, CMN_REG008F, 0x02);
	// ROPLL_SDC_DENOMINATOR
	dp_phy_write(SST1, CMN_REG0092, 0x0A);
	dp_phy_write(SST1, CMN_REG0093, 0x14);
	dp_phy_write(SST1, CMN_REG0094, 0x14);
	dp_phy_write(SST1, CMN_REG0095, 0x0A);

	if (hw_config->use_ssc) {
		// ROPLL_SSC_FM_DEVIATION
		// ROPLL_SSC_FM_FREQ
	}
}

static void dpphy_reg_set_link_bw(enum dp_link_rate_type link_rate)
{
	dp_phy_write_mask(SST1, CMN_REG00B9, DP_TX_LINK_BW_SET(link_rate),
			  DP_TX_LINK_BW_MASK);
}

static void dpphy_reg_set_lane_config(struct dp_hw_config *hw_config)
{
	u32 lane_mux_config = 0;
	u32 lane_en_config = 0;
	u32 lane1_config = 0;
	u32 lane3_config = 0;

	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
	case PIN_TYPE_C:
	case PIN_TYPE_E:
		/* 4 Lanes Mode Configuration */
		lane_mux_config = LANE_MUX_SEL_DP_LN3 | LANE_MUX_SEL_DP_LN2 |
				  LANE_MUX_SEL_DP_LN1 | LANE_MUX_SEL_DP_LN0;
		lane_en_config = DP_LANE_EN_LN3 | DP_LANE_EN_LN2 |
				 DP_LANE_EN_LN1 | DP_LANE_EN_LN0;

		lane1_config = OVRD_LN1_TX_RXD_COMP_EN | OVRD_LN1_TX_RXD_EN |
			       LN1_TX_SER_40BIT_EN_SP;
		lane3_config = OVRD_LN3_TX_RXD_COMP_EN | OVRD_LN3_TX_RXD_EN |
			       LN3_TX_SER_40BIT_EN_SP;
		break;

	case PIN_TYPE_B:
	case PIN_TYPE_D:
	case PIN_TYPE_F:
		/* 4 Lanes Mode Configuration */
		if (hw_config->orient_type == PLUG_NORMAL) {
			lane_mux_config =
				LANE_MUX_SEL_DP_LN3 | LANE_MUX_SEL_DP_LN2;
			lane_en_config = DP_LANE_EN_LN3 | DP_LANE_EN_LN2;

			lane1_config = LN1_TX_SER_40BIT_EN_SP;
			lane3_config = OVRD_LN3_TX_RXD_COMP_EN |
				       OVRD_LN3_TX_RXD_EN |
				       LN3_TX_SER_40BIT_EN_SP;
		} else {
			lane_mux_config =
				LANE_MUX_SEL_DP_LN1 | LANE_MUX_SEL_DP_LN0;
			lane_en_config = DP_LANE_EN_LN1 | DP_LANE_EN_LN0;

			lane1_config = OVRD_LN1_TX_RXD_COMP_EN |
				       OVRD_LN1_TX_RXD_EN |
				       LN1_TX_SER_40BIT_EN_SP;
			lane3_config = LN3_TX_SER_40BIT_EN_SP;
		}
		break;

	default:
		break;
	}

	dp_phy_write_mask(SST1, CMN_REG00B8, lane_mux_config,
			  LANE_MUX_SEL_DP_MASK);
	dp_phy_write_mask(SST1, CMN_REG00B9, lane_en_config, DP_LANE_EN_MASK);

	dp_phy_write_mask(SST1, TRSV_REG0413, lane1_config,
			  OVRD_LN1_TX_RXD_COMP_EN | OVRD_LN1_TX_RXD_EN |
				  LN1_TX_SER_40BIT_EN_SP);
	dp_phy_write_mask(SST1, TRSV_REG0813, lane3_config,
			  OVRD_LN3_TX_RXD_COMP_EN | OVRD_LN3_TX_RXD_EN |
				  LN3_TX_SER_40BIT_EN_SP);
}

static void dpphy_reg_set_ssc(u32 en)
{
	dp_phy_write_mask(SST1, CMN_REG00B8, SSC_EN_SET(en), SSC_EN_MASK);
}

/* USBDP PHY TRSV Registers */
static void dpphy_reg_set_lane0_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0204,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0205,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0206,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0207,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0208,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane1_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0404,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0405,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0406,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0407,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0408,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane2_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0604,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0605,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0606,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0607,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0608,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

static void dpphy_reg_set_lane3_tx_level(u32 amp_lvl, u32 pre_emp_lvl)
{
	dp_phy_write(SST1, TRSV_REG0804,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][0]);
	dp_phy_write(SST1, TRSV_REG0805,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][1]);
	dp_phy_write(SST1, TRSV_REG0806,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][2]);
	dp_phy_write(SST1, TRSV_REG0807,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][3]);
	dp_phy_write(SST1, TRSV_REG0808,
		     phy_tune_parameters[amp_lvl][pre_emp_lvl][4]);
}

/* USBDP PHY functions */
static void dpphy_reg_init(struct dp_hw_config *hw_config)
{
	/* Hold PHY soft reset */
	dpphy_reg_reset(1);

	/* USBDP PHY Settings */
	dpphy_reg_set_default_config(hw_config);
	dpphy_reg_set_link_bw(hw_config->link_rate);
	dpphy_reg_set_lane_config(hw_config);
	dpphy_reg_set_ssc(hw_config->use_ssc ? 1 : 0);

	/* wait for 60us */
	udelay(60);

	/* Release PHY soft reset */
	dpphy_reg_reset(0);
}

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

static void dp_reg_set_txclk_txclk(void)
{
	dp_write_mask(SST1, SYSTEM_CLK_CONTROL, TXCLK_SEL_TXCLK,
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
static void dp_reg_set_sst1_video_func_en(u32 en)
{
	dp_write_mask(SST1, SYSTEM_SST1_FUNCTION_ENABLE,
		      SST1_VIDEO_FUNC_EN_SET(en), SST1_VIDEO_FUNC_EN_MASK);
}

// System Miscellaneous Control Configuration

// System HPD Control Configuration

// System PLL Lock Control Configuration
static int dp_reg_wait_phy_pll_lock(void)
{
	u32 val;

	if (readl_poll_timeout_atomic(regs_dp[REGS_LINK][SST1].regs +
					      SYSTEM_PLL_LOCK_CONTROL,
				      val, PLL_LOCK_STATUS_GET(val), 10, 200)) {
		cal_log_err(0, "%s is timeout.\n", __func__);
		return -ETIME;
	} else
		return 0;
}

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
static void dp_reg_set_training_pattern(dp_training_pattern pattern)
{
	dp_write_mask(SST1, PCS_CONTROL, LINK_TRAINING_PATTERN_SET(pattern),
		      LINK_TRAINING_PATTERN_MASK);
}

static void dp_reg_set_scramble_bypass(u32 en)
{
	dp_write_mask(SST1, PCS_CONTROL, SCRAMBLE_BYPASS_SET(en),
		      SCRAMBLE_BYPASS_MASK);
}

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
static void dp_reg_set_quality_pattern(u32 en)
{
	dp_write_mask(SST1, PCS_TEST_PATTERN_CONTROL,
		      LINK_QUALITY_PATTERN_SET(en), LINK_QUALITY_PATTERN_MASK);
}

/* HDCP Control Registers */

/* SST1 Control Registers */
// SST1 Main
static void dp_reg_set_enhanced_mode(u32 en)
{
	dp_write_mask(SST1, SST1_MAIN_CONTROL, ENHANCED_MODE_SET(en),
		      ENHANCED_MODE_MASK);
}

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
	dpphy_reg_init(hw_config);
	cal_log_debug(0, "init USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	dp_reg_wait_phy_pll_lock();
	cal_log_debug(0, "locked PHY PLL\n");

	/* Set system clock to TXCLK */
	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

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

void dp_hw_reinit(struct dp_hw_config *hw_config)
{
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* USBDP PHY Re-initialization */
	dpphy_reg_init(hw_config);
	cal_log_debug(0, "reconfig USBDP Combo PHY\n");

	/* Wait for PHY PLL Lock */
	dp_reg_wait_phy_pll_lock();
	cal_log_debug(0, "locked PHY PLL\n");

	dp_reg_set_txclk_txclk();
	cal_log_debug(0, "set system clk to TX: Mux(%u)\n",
		      dp_reg_get_gfmux_status());

	/* Set Lane Map Configuration */
	dp_reg_set_lane_count(hw_config->num_lanes);
	dp_reg_set_aux_pn(hw_config->orient_type);
	dp_hw_set_lane_map_config(hw_config);
	cal_log_debug(0, "set lane count & map\n");

	/* Set System Common Functions Enable */
	dp_reg_set_pcs_func_en(1);
	cal_log_debug(0, "set common function\n");

	/* Set Single-Stream Transport Functions Enable */
	dp_reg_set_enhanced_mode(hw_config->enhanced_mode ? 1 : 0);
	dp_reg_set_sst1_video_func_en(1);
	cal_log_debug(0, "set sst function\n");
}

void dp_hw_deinit(void)
{
	dp_hw_set_common_interrupt(0);
	dp_reg_set_sw_func_en(0);
	dp_reg_set_txclk_osc();
	cal_log_debug(0, "set system clk to OSC: Mux(%u)\n",
		      dp_reg_get_gfmux_status());
}

void dp_hw_set_training_pattern(dp_training_pattern pattern)
{
	dp_reg_set_quality_pattern(0);
	dp_reg_set_training_pattern(pattern);

	if (pattern == NORAMAL_DATA || pattern == TRAINING_PATTERN_4)
		dp_reg_set_scramble_bypass(0);
	else
		dp_reg_set_scramble_bypass(1);
}

void dp_hw_set_voltage_and_pre_emphasis(struct dp_hw_config *hw_config,
					u8 *voltage, u8 *pre_emphasis)
{
	switch (hw_config->pin_type) {
	case PIN_TYPE_A:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane0_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane1_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane2_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane2_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane3_tx_level(voltage[1],
						     pre_emphasis[1]);
		}
		break;

	case PIN_TYPE_B:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane2_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane3_tx_level(voltage[1],
						     pre_emphasis[1]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane1_tx_level(voltage[0],
						     pre_emphasis[0]);
		}
		break;

	case PIN_TYPE_C:
	case PIN_TYPE_E:
		if (hw_config->orient_type == PLUG_NORMAL) {
			cal_log_info(0, "%s: Type_E & Plug_Normal\n", __func__);
			dpphy_reg_set_lane0_tx_level(voltage[2],
						     pre_emphasis[2]);
			dpphy_reg_set_lane1_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane2_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			cal_log_info(0, "%s: Type_E & Plug_Flipped\n",
				     __func__);
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane2_tx_level(voltage[3],
						     pre_emphasis[3]);
			dpphy_reg_set_lane3_tx_level(voltage[2],
						     pre_emphasis[2]);
		}
		break;

	case PIN_TYPE_D:
	case PIN_TYPE_F:
		if (hw_config->orient_type == PLUG_NORMAL) {
			dpphy_reg_set_lane2_tx_level(voltage[1],
						     pre_emphasis[1]);
			dpphy_reg_set_lane3_tx_level(voltage[0],
						     pre_emphasis[0]);
		} else {
			dpphy_reg_set_lane0_tx_level(voltage[0],
						     pre_emphasis[0]);
			dpphy_reg_set_lane1_tx_level(voltage[1],
						     pre_emphasis[1]);
		}
		break;

	default:
		break;
	}
}
