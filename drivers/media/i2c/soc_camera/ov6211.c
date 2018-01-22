/*
 * ov6211 Camera Driver
 *
 * Copyright (C) 2018 Dylan Laduranty <dylan.laduranty@mesotic.com>
 *
 * Based on ov2640 drivers.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-image-sizes.h>

#define VAL_SET(x, mask, rshift, lshift)  \
		((((x) >> rshift) & mask) << lshift)

/*
 * System Control registers
 */
#define OV6211_SC_MODE_SELECT 			0x0100
#define OV6211_SC_SOFT_RESET  			0x0103
#define OV6211_SC_FAST_STD_CTRL 		0x0106
#define OV6211_SC_SCCB_ID1				0x0109
#define OV6211_SC_REG1					0x3001
#define OV6211_SC_REG5					0x3005
#define OV6211_SC_REG9					0x3009
#define OV6211_SC_CHIP_ID_H			0x300A
#define OV6211_SC_CHIP_ID_L			0x300B
#define OV6211_SC_REG0C				0x300C
#define OV6211_SC_REG10				0x3010
#define OV6211_SC_MIPI_PHY1			0x3012
#define OV6211_SC_MIPI_PHY2			0x3013
#define OV6211_SC_MIPI_SC_CTRL0		0x3014
#define OV6211_SC_MIPI_SC_CTRL1		0x3015
#define OV6211_SC_CLKRST0				0x3016
#define OV6211_SC_CLKRST1				0x3017
#define OV6211_SC_CLKRST2				0x3018
#define OV6211_SC_CLKRST3				0x3019
#define OV6211_SC_CLKRST4				0x301A
#define OV6211_SC_CLKRST5				0x301B
#define OV6211_SC_CLKRST6				0x301C
#define OV6211_SC_FREX_RST_MASK0		0x301D
#define OV6211_SC_CLK_SEL				0x301E
#define OV6211_SC_MIPI_SC_CTRL0		0x301F
#define OV6211_SC_LOW_POWER_CTRL		0x3023
#define OV6211_SC_REG27				0x3027
#define OV6211_SC_GP_IO_IN1			0x3029
#define OV6211_SC_GP_IO_IN2			0x302A
#define OV6211_SC_SCCB_ID2				0X302B
#define OV6211_SC_AUTOSLEEP_PERIOD_UU	0x302C
#define OV6211_SC_AUTOSLEEP_PERIOD_UL	0x302D
#define OV6211_SC_AUTOSLEEP_PERIOD_LU	0x302E
#define OV6211_SC_AUTOSLEEP_PERIOD_LL	0x302F
#define OV6211_SC_LP_CTRL0				0x3030
#define OV6211_SC_IO_OEN_SLEEP			0x3032
#define OV6211_SC_IO_OEN_SLEEP2		0x3033
#define OV6211_SC_REG37				0x3037
#define OV6211_SC_REG3B				0x303B

/*
 * PLL Control registers
 */
#define OV6211_PLL_PLL0				0x3080
#define OV6211_PLL_PPL1				0x3081
#define OV6211_PLL_PLL2				0x3082
#define OV6211_PLL_PLL18				0x3098
#define OV6211_PLL_PLL19				0x3099
#define OV6211_PLL_PLL1A				0x309A
#define OV6211_PLL_PLL1B				0x309B
#define OV6211_PLL_PLL1C				0x309C
#define OV6211_PLL_PLL1D				0x309D
#define OV6211_PLL_PLL1E				0x309E
#define OV6211_PLL_VT_PIX_CLK_DIV		0x30B0
#define OV6211_PLL_VT_SYS_CLK_DIV		0x30B1
#define OV6211_PLL_MULTIPLIER			0x30B3
#define OV6211_PLL_PRE_DIV				0x30B4
#define OV6211_PLL_OP_PIX_CLK_DIV		0x30B5
#define OV6211_PLL_OP_SYS_CLK_DIV		0x30B6

/*
 * SCCB and group hold Registers
 */
#define OV6211_SB_SRB_CTRL				0x3106
#define OV6211_SB_SWITCH				0x31FF
#define OV6211_SB_GROUP_ADR0			0x3200
#define OV6211_SB_GROUP_ADR1			0x3201
#define OV6211_SB_GROUP_LEN0			0x3204
#define OV6211_SB_GROUP_LEN1			0x3205
#define OV6211_SB_GROUP_ACCESS			0x3208
#define OV6211_SB_GROUP0_PERIOD			0x3209
#define OV6211_SB_GROUP1_PERIOD			0x320A
#define OV6211_SB_SW_CTRL				0x320B
#define OV6211_SB_SRAM_TEST				0x320C
#define OV6211_SB_GROUP_ACT				0x320D
#define OV6211_SB_FM_CNT_GROUP0			0x320E
#define OV6211_SB_FM_CNT_GROUP1			0x320F

/*
 * Manual AWB gain Control
 */
#define OV6211_AWB_RED_GAIN_H			0x3400
#define OV6211_AWB_RED_GAIN_L			0x3401
#define OV6211_AWB_GREEN_GAIN_H			0x3402
#define OV6211_AWB_GREEN_GAIN_L			0x3403
#define OV6211_AWB_BLUE_GAIN_H			0x3404
#define OV6211_AWB_BLUE_GAIN_L			0x3405
#define OV6211_AWB_MAN_CTRL				0x3406

/*
 * Manual AEC / AGC
 */
#define OV6211_AEC_EXPO_UL				0x3500
#define OV6211_AEC_EXPO_LU				0x3501
#define OV6211_AEC_EXPO_LL				0x3502
#define OV6211_AEC_MAN					0x3503
#define OV6211_AEC_MAN_SNR_GAIN_H		0x3504
#define OV6211_AEC_MAN_SNR_GAIN_L		0x3505
#define OV6211_AEC_GAIN_CONVERT			0x3509
#define OV6211_AGC_ADJ_H				0x350A
#define OV6211_AGC_ADJ_L				0x350B
#define OV6211_AGC_GAIN_FORMAT0			0x5D00
#define OV6211_AGC_GAIN_FORMAT1			0x5D01
#define OV6211_AGC_DIG_COMP_CTRL0		0x5F00
#define OV6211_AGC_DIG_COMP_CTRL2		0x5F02
#define OV6211_AGC_DIG_COMP_CTRL3		0x5F03
#define OV6211_AGC_DIG_COMP_CTRL4		0x5F04
#define OV6211_AGC_DIG_COMP_CTRL5		0x5F05

/*
 * Analog Control Registers
 */
#define OV6211_ANA_CORE6				0x3666

/*
 * Timings Control registers
 */
#define OV6211_TIMING_X_ADDR_START_H	0x3800
#define OV6211_TIMING_X_ADDR_START_L	0x3801
#define OV6211_TIMING_Y_ADDR_START_H	0x3802
#define OV6211_TIMING_Y_ADDR_START_L	0x3803
#define OV6211_TIMING_X_ADDR_END_H		0x3804
#define OV6211_TIMING_X_ADDR_END_L		0x3805
#define OV6211_TIMING_Y_ADDR_END_H		0x3806
#define OV6211_TIMING_Y_ADDR_END_L		0x3807
#define OV6211_TIMING_X_OUTPUT_SIZE_H	0x3808
#define OV6211_TIMING_X_OUTPUT_SIZE_L	0x3809
#define OV6211_TIMING_Y_OUTPUT_SIZE_H	0x380A
#define OV6211_TIMING_Y_OUTPUT_SIZE_L	0x380B
#define OV6211_TIMING_HTS_H				0x380C
#define OV6211_TIMING_HTS_L				0x380D
#define OV6211_TIMING_VTS_H				0x380E
#define OV6211_TIMING_VTS_L				0x380F
#define OV6211_TIMING_ISP_X_WIN_H		0x3810
#define OV6211_TIMING_ISP_X_WIN_L		0x3811
#define OV6211_TIMING_ISP_Y_WIN_H		0x3812
#define OV6211_TIMING_ISP_Y_WIN_L		0x3813
#define OV6211_TIMING_X_INC				0x3814
#define OV6211_TIMING_Y_INC				0x3815
#define OV6211_TIMING_FORMAT1			0x3820
#define OV6211_TIMING_FORMAT2			0x3821
#define OV6211_TIMING_REG22				0x3822
#define OV6211_TIMING_REG23				0x3823
#define OV6211_TIMING_CS_RST_FSIN_H		0x3824
#define OV6211_TIMING_CS_RST_FSIN_L		0x3825
#define OV6211_TIMING_RST_FSIN_H		0x3826
#define OV6211_TIMING_RST_FSIN_L		0x3827
#define OV6211_TIMING_FVTS_H			0x3828
#define OV6211_TIMING_FVTS_L			0x3829
#define OV6211_TIMING_REG2A				0x382A
#define OV6211_TIMING_REG2B				0x382B
#define OV6211_TIMING_REG2C				0x382C
#define OV6211_TIMING_REG2D				0x382D
#define OV6211_TIMING_REG2E				0x382E

/*
 * PWM and strobe Control
 */
#define OV6211_LED_PWM_REG00			0x3B80
#define OV6211_LED_PWM_REG01			0x3B81
#define OV6211_LED_PWM_REG02			0x3B82
#define OV6211_LED_PWM_REG03			0x3B83
#define OV6211_LED_PWM_REG04			0x3B84
#define OV6211_LED_PWM_REG05			0x3B85
#define OV6211_LED_PWM_REG06			0x3B86
#define OV6211_LED_PWM_REG07			0x3B87
#define OV6211_LED_PWM_REG08			0x3B88
#define OV6211_LED_PWM_REG09			0x3B89
#define OV6211_LED_PWM_REG0A			0x3B8A
#define OV6211_LED_PWM_REG0B			0x3B8B
#define OV6211_LED_PWM_REG0C			0x3B8C
#define OV6211_LED_PWM_REG0D			0x3B8D
#define OV6211_LED_PWM_REG0E			0x3B8E
#define OV6211_LED_PWM_REG0F			0x3B8F
#define OV6211_LED_PWM_REG10			0x3B90
#define OV6211_LED_PWM_REG11			0x3B91
#define OV6211_LED_PWM_REG12			0x3B92
#define OV6211_LED_PWM_REG13			0x3B93
#define OV6211_LED_PWM_REG14			0x3B94
#define OV6211_LED_PWM_REG15			0x3B95
#define OV6211_LED_PWM_REG16			0x3B96
#define OV6211_LED_PWM_REG17			0x3B97

/*
 * OTP control Registers
 */
#define OV6211_OTP_PROGRAM_CTRL			0x3D80
#define OV6211_OTP_LOAD_CTRL			0x3D81
#define OV6211_OTP_PROGRAM_PULSE		0x3D82
#define OV6211_OTP_LOAD_PULSE			0x3D83
#define OV6211_OTP_MODE_CTRL			0x3D84
#define OV6211_OTP_START_ADDR			0x3D85
#define OV6211_OTP_END_ADDR				0x3D86
#define OV6211_OTP_PS2CS				0x3D87

/*
 * BLC Control Registers
 */
#define OV6211_BLC_CTRL_OO				0x4000
#define OV6211_BLC_CTRL_01				0x4001
#define OV6211_BLC_AUTO					0x4002
#define OV6211_BLC_FREEZE				0X4003
#define OV6211_BLC_NUM					0x4004
#define OV6211_BLC_MAN_CTRL				0x4005
#define OV6211_BLC_WIN					0x4007
#define OV6211_BLC_STABLE_RANGE			0x4008
#define OV6211_BLC_TARGET				0x4009
#define OV6211_BLC_MAN_LEVEL0_H			0x400C
#define OV6211_BLC_MAN_LEVEL0_L			0x400D
#define OV6211_BLC_MAN_LEVEL1_H			0x400E
#define OV6211_BLC_MAN_LEVEL1_L			0x400F
#define OV6211_BLC_MAN_LEVEL2_H			0x4010
#define OV6211_BLC_MAN_LEVEL2_L			0x4011
#define OV6211_BLC_MAN_LEVEL3_H			0x4012
#define OV6211_BLC_MAN_LEVEL3_L			0x4013
#define OV6211_BLC_LEVEL0_H				0x402C
#define OV6211_BLC_LEVEL0_L				0x402D
#define OV6211_BLC_LEVEL1_H				0x402E
#define OV6211_BLC_LEVEL1_L				0x402F
#define OV6211_BLC_LEVEL2_H				0x4030
#define OV6211_BLC_LEVEL2_L				0X4031
#define OV6211_BLC_LEVEL3_H				0x4032
#define OV6211_BLC_LEVEL3_L				0x4033
#define OV6211_BLC_SLOPE_LEVEL0_H		0x4034
#define OV6211_BLC_SLOPE_LEVEL0_L		0x4035
#define OV6211_BLC_SLOPE_LEVEL1_H		0x4036
#define OV6211_BLC_SLOPE_LEVEL1_L		0x4037
#define OV6211_BLC_SLOPE_LEVEL2_H		0x4038
#define OV6211_BLC_SLOPE_LEVEL2_L		0x4039
#define OV6211_BLC_SLOPE_LEVEL3_H		0x403A
#define OV6211_BLC_SLOPE_LEVEL3_L		0x403B
#define OV6211_BLC_AVG					0x404E
#define OV6211_BLC_CTRL_4F				0x404F
#define OV6211_BLC_CTRL_50				0x4050
#define OV6211_BLC_CTRL_51				0x4051

/*
 * Frame control Registers
 */
#define OV6211_FRAME_CTRL0				0x4240
#define OV6211_FRAME_ON_NUM				0X4241
#define OV6211_FRAME_OFF_NUM			0x4242
#define OV6211_FRAME_CTRL3				0x4243

/*
 * Format Control registers
 */
#define OV6211_FC_DATA_MAX_H			0x4300
#define OV6211_FC_DATA_MIN_H			0x4301
#define OV6211_FC_CLIP_L				0x4302
#define OV6211_FC_FORMAT_CTRL3			0x4303
#define OV6211_FC_F0RMAT_CTRL4			0x4304
#define OV6211_FC_EMBED_CTRL			0x4307
#define OV6211_FC_VSYNC_WIDTH_H			0x4311
#define OV6211_FC_VSYNC_WIDTH_L			0x4312
#define OV6211_FC_VSYNC_CTRL			0x4313
#define OV6211_FC_VSYNC_DELAY1			0x4314
#define OV6211_FC_VSYNC_DELAY2			0x4315
#define OV6211_FC_VSYNC_DELAY3			0x4316
#define OV6211_FC_TST_PATTERN_CTRL		0x4320
#define OV6211_FC_SOLID_B_H				0x4322
#define OV6211_FC_SOLID_B_L				0x4323
#define OV6211_FC_SOLID_GB_H			0x4324
#define OV6211_FC_SOLID_GB_L			0x4325
#define OV6211_FC_SOLID_R_H				0x4326
#define OV6211_FC_SOLID_R_L				0x4327
#define OV6211_FC_SOLID_GR_H			0x4328

/*
 * Frame Control registers
 */
#define OV6211_VFIFO_READ_START_H		0x4600
#define OV6211_VFIFO_READ_START_L		0x4601
#define OV6211_VFIFO_CTRL2				0x4602
#define OV6211_VFIFO_STATUS				0x4603

/*
 * MIPI Top Control registers
 */
#define OV6211_MIPI_CTRL_00				0x4800
#define OV6211_MIPI_CTRL_O1				0x4801
#define OV6211_MIPI_CTRL_02				0x4802
#define OV6211_MIPI_CTRL_03				0x4803
#define OV6211_MIPI_CTRL_04				0x4804
#define OV6211_MIPI_CTRL_05				0x4805
#define OV6211_MIPI_CTRL_06				0x4806
#define OV6211_MIPI_MAX_FRAME_CNT_H		0x4810
#define OV6211_MIPI_MAX_FRAME_CNT_L		0x4811
#define OV6211_MIPI_SHORT_PKT_CNT_H		0x4812
#define OV6211_MIPI_SHORT_PKT_CNT_L		0x4813
#define OV6211_MIPI_CTRL_14				0X4814
#define OV6211_MIPI_DT_SPKT				0x4815
#define OV6211_MIPI_HS_ZERO_MIN_H		0x4018
#define OV6211_MIPI_HS_ZERO_MIN_L		0x4019
#define OV6211_MIPI_HS_TRAIL_MIN_H		0x401A
#define OV6211_MIPI_HS_TRAIL_MIN_L		0x401B
#define OV6211_MIPI_CLK_ZERO_MIN_H		0x401C
#define OV6211_MIPI_CLK_ZERO_MIN_L		0x401D
#define OV6211_MIPI_PREPARE_MIN_H		0x401E
#define OV6211_MIPI_PREPARE_MIN_L		0x401F
#define OV6211_MIPI_CLK_POST_MIN_H		0x4820
#define OV6211_MIPI_CLK_POST_MIN_L		0x4821
#define OV6211_MIPI_CLK_TRAIL_MIN_H		0x4822
#define OV6211_MIPI_CLK_TRAIL_MIN_L		0x4823
#define OV6211_MIPI_LPX_P_MIN_H			0x4824
#define OV6211_MIPI_LPX_P_MIN_L			0x4825
#define OV6211_MIPI_HS_PREPARE_MIN_H	0x4826
#define OV6211_MIPI_HS_PREPARE_MIN_L	0x4827
#define OV6211_MIPI_HS_EXIT_MIN_H		0x4828
#define OV6211_MIPI_HS_EXIT_MIN_L		0x4829
#define OV6211_MIPI_UI_HS_ZERO_MIN		0x482A
#define OV6211_MIPI_UI_HS_TRAIL_MIN		0x482B
#define OV6211_MIPI_UI_CLK_ZERO_MIN		0x482C
#define OV6211_MIPI_UI_CLK_PREPARE_MIN	0x482D
#define OV6211_MIPI_UI_CLK_POST_MIN		0x482E
#define OV6211_MIPI_UI_CLK_TRAIL_MIN	0x482F
#define OV6211_MIPI_UI_LPX_P_MIN		0x4830
#define OV6211_MIPI_UI_HS_PREPARE_MIN	0x4831
#define OV6211_MIPI_UI_HS_EXIT_MIN		0x4832
#define OV6211_MIPI_REG_MIN_H			0x4833
#define OV6211_MIPI_REG_MIN_L			0x4834
#define OV6211_MIPI_REG_MAX_H			0x4835
#define OV6211_MIPI_REG_MAX_L			0x4836
#define OV6211_MIPI_PCLK_PERIOD			0x4837
#define OV6211_MIPI_WKUP_DLY			0x4838
#define OV6211_MIPI_DIR_DLY				0x483A
#define OV6211_MIPI_LP_GPIO				0x483B
#define OV6211_MIPI_CTRL_3C				0x483C
#define OV6211_MIPI_T_TA_GO				0x483D
#define OV6211_MIPI_T_TA_SURE			0x483E
#define OV6211_MIPI_T_TA_GET			0x483F
#define OV6211_MIPI_CLIP_MAX_H			0x4846
#define OV6211_MIPI_CLIP_MAX_L			0x4847
#define OV6211_MIPI_CLIP_MIN_H			0x4848
#define OV6211_MIPI_CLIP_MIN_L			0x4849
#define OV6211_MIPI_REG_INTR_MAN		0x4850
#define OV6211_MIPI_REG_TX_WR			0x4851
#define OV6211_MIPI_REG_TX_STOP			0x4852
#define OV6211_MIPI_REG_TA_ACK			0x4853
#define OV6211_MIPI_REG_TA_REQ			0x4854
#define OV6211_MIPI_CTRL_60				0x4860
#define OV6211_MIPI_HD_SK_REG0			0x4861
#define OV6211_MIPI_HD_SK_REG1			0x4862
#define OV6211_MIPI_HD_SK_REG2			0x4863
#define OV6211_MIPI_HD_SK_REG3			0x4864
#define OV6211_MIPI_ST					0x4865

/*
 * ISP Top Registers
 */
#define OV6211_ISP_CTRL_00				0x5000
#define OV6211_ISP_CTRL_01				0x5001
#define OV6211_ISP_CTRL_02				0x5002
#define OV6211_ISP_CTRL_03				0x5003
#define OV6211_ISP_CTRL_04				0x5004
#define OV6211_ISP_CTRL_05				0x5005
#define OV6211_ISP_CTRL_06				0x5006
#define OV6211_ISP_CTRL_07				0x5007
#define OV6211_ISP_CTRL_08				0x5008
#define OV6211_ISP_CTRL_09				0x5009
#define OV6211_ISP_SENSOR_BIAS			0x5010
#define OV6211_ISP_LINEAR_GAIN			0x5011
#define OV6211_ISP_AWB_GAIN_R_H			0x5012
#define OV6211_ISP_AWB_GAIN_R_L			0x5013
#define OV6211_ISP_AWB_GAIN_G_H			0x5014
#define OV6211_ISP_AWB_GAIN_G_L			0x5015
#define OV6211_ISP_AWB_GAIN_B_H			0x5016
#define OV6211_ISP_AWB_GAIN_B_L			0x5017
#define OV6211_ISP_PRE_00				0x5E00
#define OV6211_ISP_PRE_01				0x5E01
#define OV6211_ISP_PRE_02				0x5E02
#define OV6211_ISP_PRE_03				0x5E03
#define OV6211_ISP_PRE_04				0x5E04
#define OV6211_ISP_PRE_05				0x5E05
#define OV6211_ISP_PRE_06				0x5E06
#define OV6211_ISP_PRE_07				0x5E07
#define OV6211_ISP_PRE_08				0x5E08

/*
 * Window Control registers
 */
#define OV6211_WIN_XSTART_OFF_H			0x5A00
#define OV6211_WIN_XSTART_OFF_L			0x5A01
#define OV6211_WIN_YSTART_OFF_H			0x5A02
#define OV6211_WIN_YSTART_OFF_L			0x5A03
#define OV6211_WIN_MAN_WIDTH_H			0x5A04
#define OV6211_WIN_MAN_WIDTH_L			0x5A05
#define OV6211_WIN_MAN_HEIGHT_H			0x5A06
#define OV6211_WIN_MAN_HEIGHT_L			0x5A07
#define OV6211_WIN_MAN					0x5A08
#define OV6211_WIN_PX_CNT_H				0x5A09
#define OV6211_WIN_PX_CNT_L				0x5A0A
#define OV6211_WIN_LN_CNT_H				0x5A0B
#define OV6211_WIN_LN_CNT_L				0x5A0C






/*
 * ID
 */
#define MANUFACTURER_ID	0xFFFF
#define PID_ov6211	0x2642
#define VERSION(idh, idl) ((idh << 8) | (idl & 0xFF))

/*
 * Struct
 */
struct regval_list {
	u16 reg_num;
	u8 value;
};

struct ov6211_win_size {
	char				*name;
	u32				width;
	u32				height;
	const struct regval_list	*regs;
};


struct ov6211_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct v4l2_clk			*clk;
	const struct ov6211_win_size	*win;

	struct soc_camera_subdev_desc	ssdd_dt;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
};

/*
 * Registers settings
 */

#define ENDMARKER { 0xff, 0xff }

static const struct regval_list ov6211_init_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ 0x2c,   0xff },
	{ 0x2e,   0xdf },
	{ BANK_SEL, BANK_SEL_SENS },
	{ 0x3c,   0x32 },
	{ CLKRC, CLKRC_DIV_SET(1) },
	{ COM2, COM2_OCAP_Nx_SET(3) },
	{ REG04, REG04_DEF | REG04_HREF_EN },
	{ COM8,  COM8_DEF | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN },
	{ COM9, COM9_AGC_GAIN_8x | 0x08},
	{ 0x2c,   0x0c },
	{ 0x33,   0x78 },
	{ 0x3a,   0x33 },
	{ 0x3b,   0xfb },
	{ 0x3e,   0x00 },
	{ 0x43,   0x11 },
	{ 0x16,   0x10 },
	{ 0x39,   0x02 },
	{ 0x35,   0x88 },
	{ 0x22,   0x0a },
	{ 0x37,   0x40 },
	{ 0x23,   0x00 },
	{ ARCOM2, 0xa0 },
	{ 0x06,   0x02 },
	{ 0x06,   0x88 },
	{ 0x07,   0xc0 },
	{ 0x0d,   0xb7 },
	{ 0x0e,   0x01 },
	{ 0x4c,   0x00 },
	{ 0x4a,   0x81 },
	{ 0x21,   0x99 },
	{ AEW,    0x40 },
	{ AEB,    0x38 },
	{ VV,     VV_HIGH_TH_SET(0x08) | VV_LOW_TH_SET(0x02) },
	{ 0x5c,   0x00 },
	{ 0x63,   0x00 },
	{ FLL,    0x22 },
	{ COM3,   0x38 | COM3_BAND_AUTO },
	{ REG5D,  0x55 },
	{ REG5E,  0x7d },
	{ REG5F,  0x7d },
	{ REG60,  0x55 },
	{ HISTO_LOW,   0x70 },
	{ HISTO_HIGH,  0x80 },
	{ 0x7c,   0x05 },
	{ 0x20,   0x80 },
	{ 0x28,   0x30 },
	{ 0x6c,   0x00 },
	{ 0x6d,   0x80 },
	{ 0x6e,   0x00 },
	{ 0x70,   0x02 },
	{ 0x71,   0x94 },
	{ 0x73,   0xc1 },
	{ 0x3d,   0x34 },
	{ COM7, COM7_RES_UXGA | COM7_ZOOM_EN },
	{ 0x5a,   0x57 },
	{ BD50,   0xbb },
	{ BD60,   0x9c },
	{ BANK_SEL, BANK_SEL_DSP },
	{ 0xe5,   0x7f },
	{ MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
	{ 0x41,   0x24 },
	{ RESET, RESET_JPEG | RESET_DVP },
	{ 0x76,   0xff },
	{ 0x33,   0xa0 },
	{ 0x42,   0x20 },
	{ 0x43,   0x18 },
	{ 0x4c,   0x00 },
	{ CTRL3, CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
	{ 0x88,   0x3f },
	{ 0xd7,   0x03 },
	{ 0xd9,   0x10 },
	{ R_DVP_SP , R_DVP_SP_AUTO_MODE | 0x2 },
	{ 0xc8,   0x08 },
	{ 0xc9,   0x80 },
	{ BPADDR, 0x00 },
	{ BPDATA, 0x00 },
	{ BPADDR, 0x03 },
	{ BPDATA, 0x48 },
	{ BPDATA, 0x48 },
	{ BPADDR, 0x08 },
	{ BPDATA, 0x20 },
	{ BPDATA, 0x10 },
	{ BPDATA, 0x0e },
	{ 0x90,   0x00 },
	{ 0x91,   0x0e },
	{ 0x91,   0x1a },
	{ 0x91,   0x31 },
	{ 0x91,   0x5a },
	{ 0x91,   0x69 },
	{ 0x91,   0x75 },
	{ 0x91,   0x7e },
	{ 0x91,   0x88 },
	{ 0x91,   0x8f },
	{ 0x91,   0x96 },
	{ 0x91,   0xa3 },
	{ 0x91,   0xaf },
	{ 0x91,   0xc4 },
	{ 0x91,   0xd7 },
	{ 0x91,   0xe8 },
	{ 0x91,   0x20 },
	{ 0x92,   0x00 },
	{ 0x93,   0x06 },
	{ 0x93,   0xe3 },
	{ 0x93,   0x03 },
	{ 0x93,   0x03 },
	{ 0x93,   0x00 },
	{ 0x93,   0x02 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x96,   0x00 },
	{ 0x97,   0x08 },
	{ 0x97,   0x19 },
	{ 0x97,   0x02 },
	{ 0x97,   0x0c },
	{ 0x97,   0x24 },
	{ 0x97,   0x30 },
	{ 0x97,   0x28 },
	{ 0x97,   0x26 },
	{ 0x97,   0x02 },
	{ 0x97,   0x98 },
	{ 0x97,   0x80 },
	{ 0x97,   0x00 },
	{ 0x97,   0x00 },
	{ 0xa4,   0x00 },
	{ 0xa8,   0x00 },
	{ 0xc5,   0x11 },
	{ 0xc6,   0x51 },
	{ 0xbf,   0x80 },
	{ 0xc7,   0x10 },
	{ 0xb6,   0x66 },
	{ 0xb8,   0xA5 },
	{ 0xb7,   0x64 },
	{ 0xb9,   0x7C },
	{ 0xb3,   0xaf },
	{ 0xb4,   0x97 },
	{ 0xb5,   0xFF },
	{ 0xb0,   0xC5 },
	{ 0xb1,   0x94 },
	{ 0xb2,   0x0f },
	{ 0xc4,   0x5c },
	{ 0xa6,   0x00 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x1b },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0x7f,   0x00 },
	{ 0xe5,   0x1f },
	{ 0xe1,   0x77 },
	{ 0xdd,   0x7f },
	{ CTRL0,  CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
	ENDMARKER,
};

/*
 * Register settings for window size
 * The preamble, setup the internal DSP to input an UXGA (1600x1200) image.
 * Then the different zooming configurations will setup the output image size.
 */
static const struct regval_list ov6211_size_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ RESET, RESET_DVP },
	{ HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
	{ VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
	{ CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
		 CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
	{ HSIZE, HSIZE_SET(UXGA_WIDTH) },
	{ VSIZE, VSIZE_SET(UXGA_HEIGHT) },
	{ XOFFL, XOFFL_SET(0) },
	{ YOFFL, YOFFL_SET(0) },
	{ VHYX, VHYX_HSIZE_SET(UXGA_WIDTH) | VHYX_VSIZE_SET(UXGA_HEIGHT) |
		VHYX_XOFF_SET(0) | VHYX_YOFF_SET(0)},
	{ TEST, TEST_HSIZE_SET(UXGA_WIDTH) },
	ENDMARKER,
};

#define PER_SIZE_REG_SEQ(x, y, v_div, h_div, pclk_div)	\
	{ CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(v_div) |	\
		 CTRLI_H_DIV_SET(h_div)},		\
	{ ZMOW, ZMOW_OUTW_SET(x) },			\
	{ ZMOH, ZMOH_OUTH_SET(y) },			\
	{ ZMHH, ZMHH_OUTW_SET(x) | ZMHH_OUTH_SET(y) },	\
	{ R_DVP_SP, pclk_div },				\
	{ RESET, 0x00}

static const struct regval_list ov6211_qcif_regs[] = {
	PER_SIZE_REG_SEQ(QCIF_WIDTH, QCIF_HEIGHT, 3, 3, 4),
	ENDMARKER,
};

static const struct regval_list ov6211_qvga_regs[] = {
	PER_SIZE_REG_SEQ(QVGA_WIDTH, QVGA_HEIGHT, 2, 2, 4),
	ENDMARKER,
};

static const struct regval_list ov6211_cif_regs[] = {
	PER_SIZE_REG_SEQ(CIF_WIDTH, CIF_HEIGHT, 2, 2, 8),
	ENDMARKER,
};

static const struct regval_list ov6211_vga_regs[] = {
	PER_SIZE_REG_SEQ(VGA_WIDTH, VGA_HEIGHT, 0, 0, 2),
	ENDMARKER,
};

static const struct regval_list ov6211_svga_regs[] = {
	PER_SIZE_REG_SEQ(SVGA_WIDTH, SVGA_HEIGHT, 1, 1, 2),
	ENDMARKER,
};

static const struct regval_list ov6211_xga_regs[] = {
	PER_SIZE_REG_SEQ(XGA_WIDTH, XGA_HEIGHT, 0, 0, 2),
	{ CTRLI,    0x00},
	ENDMARKER,
};

static const struct regval_list ov6211_sxga_regs[] = {
	PER_SIZE_REG_SEQ(SXGA_WIDTH, SXGA_HEIGHT, 0, 0, 2),
	{ CTRLI,    0x00},
	{ R_DVP_SP, 2 | R_DVP_SP_AUTO_MODE },
	ENDMARKER,
};

static const struct regval_list ov6211_uxga_regs[] = {
	PER_SIZE_REG_SEQ(UXGA_WIDTH, UXGA_HEIGHT, 0, 0, 0),
	{ CTRLI,    0x00},
	{ R_DVP_SP, 0 | R_DVP_SP_AUTO_MODE },
	ENDMARKER,
};

#define ov6211_SIZE(n, w, h, r) \
	{.name = n, .width = w , .height = h, .regs = r }

static const struct ov6211_win_size ov6211_supported_win_sizes[] = {
	ov6211_SIZE("100x100", 100, 100, ov6211_100x100_regs),
	ov6211_SIZE("200x200", 200, 200, ov6211_200x200_regs),
	ov6211_SIZE("400x400", 400, 400, ov6211_400x400_regs),
};

/*
 * Register settings for pixel formats
 */
static const struct regval_list ov6211_format_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static const struct regval_list ov6211_yuyv_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_YUV422 },
	{ 0xd7, 0x03 },
	{ 0x33, 0xa0 },
	{ 0xe5, 0x1f },
	{ 0xe1, 0x67 },
	{ RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static const struct regval_list ov6211_uyvy_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_LBYTE_FIRST | IMAGE_MODE_YUV422 },
	{ 0xd7, 0x01 },
	{ 0x33, 0xa0 },
	{ 0xe1, 0x67 },
	{ RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static const struct regval_list ov6211_rgb565_be_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_RGB565 },
	{ 0xd7, 0x03 },
	{ RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static const struct regval_list ov6211_rgb565_le_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_LBYTE_FIRST | IMAGE_MODE_RGB565 },
	{ 0xd7, 0x03 },
	{ RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static u32 ov6211_codes[] = {
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_RGB565_2X8_BE,
	MEDIA_BUS_FMT_RGB565_2X8_LE,
};

/*
 * General functions
 */
static struct ov6211_priv *to_ov6211(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov6211_priv,
			    subdev);
}

static int ov6211_write_array(struct i2c_client *client,
			      const struct regval_list *vals)
{
	int ret;

	while ((vals->reg_num != 0xff) || (vals->value != 0xff)) {
		ret = i2c_smbus_write_byte_data(client,
						vals->reg_num, vals->value);
		dev_vdbg(&client->dev, "array: 0x%02x, 0x%02x",
			 vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

static int ov6211_mask_set(struct i2c_client *client,
			   u8  reg, u8  mask, u8  set)
{
	s32 val = i2c_smbus_read_byte_data(client, reg);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return i2c_smbus_write_byte_data(client, reg, val);
}

static int ov6211_reset(struct i2c_client *client)
{
	int ret;
	const struct regval_list reset_seq[] = {
		{BANK_SEL, BANK_SEL_SENS},
		{COM7, COM7_SRST},
		ENDMARKER,
	};

	ret = ov6211_write_array(client, reset_seq);
	if (ret)
		goto err;

	msleep(5);
err:
	dev_dbg(&client->dev, "%s: (ret %d)", __func__, ret);
	return ret;
}

/*
 * soc_camera_ops functions
 */
static int ov6211_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov6211_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov6211_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	u8 val;
	int ret;

	ret = i2c_smbus_write_byte_data(client, BANK_SEL, BANK_SEL_SENS);
	if (ret < 0)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		val = ctrl->val ? REG04_VFLIP_IMG : 0x00;
		return ov6211_mask_set(client, REG04, REG04_VFLIP_IMG, val);
	case V4L2_CID_HFLIP:
		val = ctrl->val ? REG04_HFLIP_IMG : 0x00;
		return ov6211_mask_set(client, REG04, REG04_HFLIP_IMG, val);
	}

	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov6211_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(client, reg->reg);
	if (ret < 0)
		return ret;

	reg->val = ret;

	return 0;
}

static int ov6211_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff)
		return -EINVAL;

	return i2c_smbus_write_byte_data(client, reg->reg, reg->val);
}
#endif

static int ov6211_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct ov6211_priv *priv = to_ov6211(client);

	return soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
}

/* Select the nearest higher resolution for capture */
static const struct ov6211_win_size *ov6211_select_win(u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(ov6211_supported_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(ov6211_supported_win_sizes); i++) {
		if (ov6211_supported_win_sizes[i].width  >= *width &&
		    ov6211_supported_win_sizes[i].height >= *height) {
			*width = ov6211_supported_win_sizes[i].width;
			*height = ov6211_supported_win_sizes[i].height;
			return &ov6211_supported_win_sizes[i];
		}
	}

	*width = ov6211_supported_win_sizes[default_size].width;
	*height = ov6211_supported_win_sizes[default_size].height;
	return &ov6211_supported_win_sizes[default_size];
}

static int ov6211_set_params(struct i2c_client *client, u32 *width, u32 *height,
			     u32 code)
{
	struct ov6211_priv       *priv = to_ov6211(client);
	const struct regval_list *selected_cfmt_regs;
	int ret;

	/* select win */
	priv->win = ov6211_select_win(width, height);

	/* select format */
	priv->cfmt_code = 0;
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
		dev_dbg(&client->dev, "%s: Selected cfmt RGB565 BE", __func__);
		selected_cfmt_regs = ov6211_rgb565_be_regs;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		dev_dbg(&client->dev, "%s: Selected cfmt RGB565 LE", __func__);
		selected_cfmt_regs = ov6211_rgb565_le_regs;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		dev_dbg(&client->dev, "%s: Selected cfmt YUYV (YUV422)", __func__);
		selected_cfmt_regs = ov6211_yuyv_regs;
		break;
	default:
	case MEDIA_BUS_FMT_UYVY8_2X8:
		dev_dbg(&client->dev, "%s: Selected cfmt UYVY", __func__);
		selected_cfmt_regs = ov6211_uyvy_regs;
	}

	/* reset hardware */
	ov6211_reset(client);

	/* initialize the sensor with default data */
	dev_dbg(&client->dev, "%s: Init default", __func__);
	ret = ov6211_write_array(client, ov6211_init_regs);
	if (ret < 0)
		goto err;

	/* select preamble */
	dev_dbg(&client->dev, "%s: Set size to %s", __func__, priv->win->name);
	ret = ov6211_write_array(client, ov6211_size_change_preamble_regs);
	if (ret < 0)
		goto err;

	/* set size win */
	ret = ov6211_write_array(client, priv->win->regs);
	if (ret < 0)
		goto err;

	/* cfmt preamble */
	dev_dbg(&client->dev, "%s: Set cfmt", __func__);
	ret = ov6211_write_array(client, ov6211_format_change_preamble_regs);
	if (ret < 0)
		goto err;

	/* set cfmt */
	ret = ov6211_write_array(client, selected_cfmt_regs);
	if (ret < 0)
		goto err;

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	ov6211_reset(client);
	priv->win = NULL;

	return ret;
}

static int ov6211_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov6211_priv *priv = to_ov6211(client);

	if (format->pad)
		return -EINVAL;

	if (!priv->win) {
		u32 width = SVGA_WIDTH, height = SVGA_HEIGHT;
		priv->win = ov6211_select_win(&width, &height);
		priv->cfmt_code = MEDIA_BUS_FMT_UYVY8_2X8;
	}

	mf->width	= priv->win->width;
	mf->height	= priv->win->height;
	mf->code	= priv->cfmt_code;

	switch (mf->code) {
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		mf->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
		mf->colorspace = V4L2_COLORSPACE_JPEG;
	}
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov6211_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (format->pad)
		return -EINVAL;

	/*
	 * select suitable win, but don't store it
	 */
	ov6211_select_win(&mf->width, &mf->height);

	mf->field	= V4L2_FIELD_NONE;

	switch (mf->code) {
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		mf->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		mf->code = MEDIA_BUS_FMT_UYVY8_2X8;
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
		mf->colorspace = V4L2_COLORSPACE_JPEG;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return ov6211_set_params(client, &mf->width,
					 &mf->height, mf->code);
	cfg->try_fmt = *mf;
	return 0;
}

static int ov6211_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov6211_codes))
		return -EINVAL;

	code->code = ov6211_codes[code->index];
	return 0;
}

static int ov6211_get_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_selection *sel)
{
	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = UXGA_WIDTH;
		sel->r.height = UXGA_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov6211_video_probe(struct i2c_client *client)
{
	struct ov6211_priv *priv = to_ov6211(client);
	u8 verh,verl;
	const char *devname;
	int ret;

	ret = ov6211_s_power(&priv->subdev, 1);
	if (ret < 0)
		return ret;

	/*
	 * check and show product ID and manufacturer ID
	 */

	verl  = i2c_smbus_read_byte_data(client, OV6211_SC_CHIP_ID_L);
	verh  = i2c_smbus_read_byte_data(client, OV6211_SC_CHIP_ID_H);

	switch (VERSION(verh, verl)) {
	case PID_ov6211:
		devname     = "ov6211";
		break;
	default:
		dev_err(&client->dev,
			"Product ID error %x:%x\n", pid, ver);
		ret = -ENODEV;
		goto done;
	}

	dev_info(&client->dev,
		 "%s Product ID %0x:%0x Manufacturer ID %x:%x\n",
		 devname, pid, ver, midh, midl);

	ret = v4l2_ctrl_handler_setup(&priv->hdl);

done:
	ov6211_s_power(&priv->subdev, 0);
	return ret;
}

static const struct v4l2_ctrl_ops ov6211_ctrl_ops = {
	.s_ctrl = ov6211_s_ctrl,
};

static struct v4l2_subdev_core_ops ov6211_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov6211_g_register,
	.s_register	= ov6211_s_register,
#endif
	.s_power	= ov6211_s_power,
};

static int ov6211_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
		V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = soc_camera_apply_board_flags(ssdd, cfg);

	return 0;
}

static struct v4l2_subdev_video_ops ov6211_subdev_video_ops = {
	.s_stream	= ov6211_s_stream,
	.g_mbus_config	= ov6211_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov6211_subdev_pad_ops = {
	.enum_mbus_code = ov6211_enum_mbus_code,
	.get_selection	= ov6211_get_selection,
	.get_fmt	= ov6211_get_fmt,
	.set_fmt	= ov6211_set_fmt,
};

static struct v4l2_subdev_ops ov6211_subdev_ops = {
	.core	= &ov6211_subdev_core_ops,
	.video	= &ov6211_subdev_video_ops,
	.pad	= &ov6211_subdev_pad_ops,
};

/* OF probe functions */
static int ov6211_hw_power(struct device *dev, int on)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ov6211_priv *priv = to_ov6211(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
			__func__, on ? "ENABLE" : "DISABLE");

	if (priv->pwdn_gpio)
		gpiod_direction_output(priv->pwdn_gpio, !on);

	return 0;
}

static int ov6211_hw_reset(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ov6211_priv *priv = to_ov6211(client);

	if (priv->resetb_gpio) {
		/* Active the resetb pin to perform a reset pulse */
		gpiod_direction_output(priv->resetb_gpio, 1);
		usleep_range(3000, 5000);
		gpiod_direction_output(priv->resetb_gpio, 0);
	}

	return 0;
}

static int ov6211_probe_dt(struct i2c_client *client,
		struct ov6211_priv *priv)
{
	/* Request the reset GPIO deasserted */
	priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
			GPIOD_OUT_LOW);
	if (!priv->resetb_gpio)
		dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
	else if (IS_ERR(priv->resetb_gpio))
		return PTR_ERR(priv->resetb_gpio);

	/* Request the power down GPIO asserted */
	priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
			GPIOD_OUT_HIGH);
	if (!priv->pwdn_gpio)
		dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
	else if (IS_ERR(priv->pwdn_gpio))
		return PTR_ERR(priv->pwdn_gpio);

	/* Initialize the soc_camera_subdev_desc */
	priv->ssdd_dt.power = ov6211_hw_power;
	priv->ssdd_dt.reset = ov6211_hw_reset;
	client->dev.platform_data = &priv->ssdd_dt;

	return 0;
}

/*
 * i2c_driver functions
 */
static int ov6211_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov6211_priv	*priv;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int			ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"ov6211: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ov6211_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->clk = v4l2_clk_get(&client->dev, "pclk");
	if (IS_ERR(priv->clk))
		return -EPROBE_DEFER;

	if (!ssdd && !client->dev.of_node) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		ret = -EINVAL;
		goto err_clk;
	}

	if (!ssdd) {
		ret = ov6211_probe_dt(client, priv);
		if (ret)
			goto err_clk;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov6211_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov6211_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov6211_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_clk;
	}

	ret = ov6211_video_probe(client);
	if (ret < 0)
		goto err_videoprobe;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&adapter->dev, "ov6211 Probed\n");

	return 0;

err_videoprobe:
	v4l2_ctrl_handler_free(&priv->hdl);
err_clk:
	v4l2_clk_put(priv->clk);
	return ret;
}

static int ov6211_remove(struct i2c_client *client)
{
	struct ov6211_priv       *priv = to_ov6211(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	return 0;
}

static const struct i2c_device_id ov6211_id[] = {
	{ "ov6211", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov6211_id);

static const struct of_device_id ov6211_of_match[] = {
	{.compatible = "ovti,ov6211", },
	{},
};
MODULE_DEVICE_TABLE(of, ov6211_of_match);

static struct i2c_driver ov6211_i2c_driver = {
	.driver = {
		.name = "ov6211",
		.of_match_table = of_match_ptr(ov6211_of_match),
	},
	.probe    = ov6211_probe,
	.remove   = ov6211_remove,
	.id_table = ov6211_id,
};

module_i2c_driver(ov6211_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omni Vision 6211 sensor");
MODULE_AUTHOR("Dylan Laduranty");
MODULE_LICENSE("GPL v2");
