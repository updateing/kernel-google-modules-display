/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for DisplayPort CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DP_CAL_H__
#define __SAMSUNG_DP_CAL_H__

/* Register definition */
enum sst_id {
	SST1 = 0,
	SST2,
	SST_MAX
};

enum dp_regs_type {
	REGS_LINK = 0,
	REGS_PHY,
	REGS_DP_TYPE_MAX
};

/* DFP_D (USB Type-C) */
// It should be matched with enum in \include\linux\usb\typec_dp.h
enum pin_assignment {
	PIN_TYPE_A = 0,		/* Not supported after Alt Mode Spec v1.0b */
	PIN_TYPE_B,		/* Not supported after Alt Mode Spec v1.0b */
	PIN_TYPE_C,
	PIN_TYPE_D,
	PIN_TYPE_E,
	PIN_TYPE_F,		/* Not supported after Alt Mode Spec v1.0b */
};

// It should be matched with enum typec_orientation in \include\linux\usb\typec.h
enum plug_orientation {
	PLUG_NONE = 0,		// TYPEC_ORIENTATION_NONE
	PLUG_NORMAL,		// TYPEC_ORIENTATION_NORMAL
	PLUG_FLIPPED,		// TYPEC_ORIENTATION_REVERSE
};

/* AUX Ch Definitions for DPCD/EDID */
#define DPCD_BUF_SIZE 12

#define AUX_DATA_BUF_COUNT 16
#define AUX_RETRY_COUNT 3

#define MAX_EDID_BLOCK 4
#define EDID_BLOCK_SIZE 128
#define EDID_ADDRESS 0x50

enum dp_aux_ch_command_type {
	I2C_WRITE = 0x4,
	I2C_READ = 0x5,
	DPCD_WRITE = 0x8,
	DPCD_READ = 0x9,
};

enum dp_aux_ch_cmd_status {
	AUX_CMD_STATUS_OK			= 0,
	AUX_CMD_STATUS_NACK_ERROR,
	AUX_CMD_STATUS_TIMEOUT_ERROR,
	AUX_CMD_STATUS_UNKNOWN_ERROR,
	AUX_CMD_STATUS_MUCH_DEFER_ERROR,
	AUX_CMD_STATUS_TX_SHORT_ERROR,
	AUX_CMD_STATUS_RX_SHORT_ERROR,
	AUX_CMD_STATUS_NACK_WITHOUT_M_ERROR,
	AUX_CMD_STATUS_I2C_NACK_ERROR,
};


/* Display HW enum definition */
// Link Config
#define MAX_LANE_CNT 4

enum dp_link_rate_type {
	LINK_RATE_RBR	= 0,	// DP_LINK_BW_1_62
	LINK_RATE_HBR,		// DP_LINK_BW_2_7
	LINK_RATE_HBR2,		// DP_LINK_BW_5_4
	LINK_RATE_HBR3,		// DP_LINK_BW_8_1
};

typedef enum {
	NORAMAL_DATA = 0,
	TRAINING_PATTERN_1 = 1,
	TRAINING_PATTERN_2 = 2,
	TRAINING_PATTERN_3 = 3,
	TRAINING_PATTERN_4 = 5,
} dp_training_pattern;

// Interrupts
enum dp_interrupt_mask {
	PLL_LOCK_CHG_INT_MASK,
	HOTPLUG_CHG_INT_MASK,
	HPD_LOST_INT_MASK,
	PLUG_INT_MASK,
	HPD_IRQ_INT_MASK,
	RPLY_RECEIV_INT_MASK,
	AUX_ERR_INT_MASK,
	HDCP_LINK_CHECK_INT_MASK,
	HDCP_LINK_FAIL_INT_MASK,
	HDCP_R0_READY_INT_MASK,
	VIDEO_FIFO_UNDER_FLOW_MASK,
	VSYNC_DET_INT_MASK,
	AUDIO_FIFO_UNDER_RUN_INT_MASK,
	AUDIO_FIFO_OVER_RUN_INT_MASK,

	ALL_INT_MASK
};

// Video Config
enum video_mode {
	VIDEO_MODE_MASTER = 0,
	VIDEO_MODE_SLAVE,
};

enum dp_dynamic_range_type {
	VESA_RANGE = 0,   /* (0 ~ 255) */
	CEA_RANGE = 1,    /* (16 ~ 235) */
};

enum bit_depth {
	BPC_6 = 0,
	BPC_8,
	BPC_10,
};

enum color_format {
	COLOR_RGB = 0,
};

enum sync_polarity {
	SYNC_POSITIVE = 0,
	SYNC_NEGATIVE,
};

struct video_timing {
	u32 clock;
	u32 htotal;
	u32 vtotal;
	u32 hfp;
	u32 hbp;
	u32 hactive;
	u32 vfp;
	u32 vbp;
	u32 vactive;
	enum sync_polarity vsync_polarity;
	enum sync_polarity hsync_polarity;
};

// BIST Config
enum bist_pattern_type {
	// Normal BIST Patterns
	COLOR_BAR = 0,
	WGB_BAR,
	MW_BAR,
	// CTS BIST Patterns
	CTS_COLOR_RAMP,
	CTS_BLACK_WHITE,
	CTS_COLOR_SQUARE_VESA,
	CTS_COLOR_SQUARE_CEA,
};

// InfoFrame
#define	INFOFRAME_PACKET_TYPE_AVI 0x82		/** Auxiliary Video information InfoFrame */
#define MAX_INFOFRAME_LENGTH 27

#define AVI_INFOFRAME_VERSION 0x02
#define AVI_INFOFRAME_LENGTH 0x0D
#define ACTIVE_FORMAT_INFO_PRESENT (1 << 4)	/* No Active Format Information */
#define ACTIVE_PORTION_ASPECT_RATIO (0x8 << 0)	/* Same as Picture Aspect Ratio */

struct infoframe {
	u8 type_code;
	u8 version_number;
	u8 length;
	u8 data[MAX_INFOFRAME_LENGTH];
};


/*
 * DisplayPort HW Configuration Structure Definition
 * It can be used CAL function and driver layer
 */
struct dp_hw_config {
	/* USB Type-C */
	enum pin_assignment pin_type;
	enum plug_orientation orient_type;

	/* DP Link */
	enum dp_link_rate_type link_rate;
	u32 num_lanes;

	/* Video */
	enum dp_dynamic_range_type range;
	enum bit_depth bpc;
	struct video_timing vtiming;
	bool enhanced_mode;
	bool use_fec;
	bool use_ssc;

	/* BIST Mode */
	bool bist_mode;
	enum bist_pattern_type bist_type;

	/* USBDP combo phy enable ref count */
	atomic_t usbdp_phy_en_cnt;
};


/* DP memory map interface */
void dp_regs_desc_init(void __iomem *regs, phys_addr_t start, const char *name,
		       enum dp_regs_type type, unsigned int id);

/* DPCD (DisplayPort Configuration Data) Read/Write Interfaces */
int dp_hw_write_dpcd_burst(u32 address, u32 length, u8 *data);
int dp_hw_read_dpcd_burst(u32 address, u32 length, u8 *data);
int dp_hw_read_edid(u8 block_cnt, u32 length, u8 *data);

/* DP Hardware Control Interfaces */
void dp_hw_init(struct dp_hw_config *hw_config);
void dp_hw_reinit(struct dp_hw_config *hw_config);
void dp_hw_deinit(struct dp_hw_config *hw_config);
void dp_hw_start(void);
void dp_hw_stop(void);

void dp_hw_set_video_config(struct dp_hw_config *hw_config);
int  dp_hw_set_bist_video_config(struct dp_hw_config *hw_config);

void dp_hw_get_intr_source(bool *common, bool *str, bool *pcs);
u32  dp_hw_get_and_clear_common_intr(void);
u32  dp_hw_get_and_clear_video_intr(void);

void dp_hw_send_avi_infoframe(struct infoframe avi_infoframe);
void dp_hw_send_spd_infoframe(struct infoframe spd_infoframe);

void dp_hw_set_training_pattern(dp_training_pattern pattern);
void dp_hw_set_voltage_and_pre_emphasis(struct dp_hw_config *hw_config, u8 *voltage, u8 *pre_emphasis);

/* USB Interface Prototypes for Combo-PHY Handshaking */
extern int dwc3_exynos_phy_enable(int owner, bool on);

#endif /* __SAMSUNG_DP_CAL_H__ */
