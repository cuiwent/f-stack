/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_TYPE_H_
#define _YUSUR2_TYPE_H_

/*
 * The following is a brief description of the error categories used by the
 * ERROR_REPORT* macros.
 *
 * - YUSUR2_ERROR_INVALID_STATE
 * This category is for errors which represent a serious failure state that is
 * unexpected, and could be potentially harmful to device operation. It should
 * not be used for errors relating to issues that can be worked around or
 * ignored.
 *
 * - YUSUR2_ERROR_POLLING
 * This category is for errors related to polling/timeout issues and should be
 * used in any case where the timeout occurred, or a failure to obtain a lock,
 * or failure to receive data within the time limit.
 *
 * - YUSUR2_ERROR_CAUTION
 * This category should be used for reporting issues that may be the cause of
 * other errors, such as temperature warnings. It should indicate an event which
 * could be serious, but hasn't necessarily caused problems yet.
 *
 * - YUSUR2_ERROR_SOFTWARE
 * This category is intended for errors due to software state preventing
 * something. The category is not intended for errors due to bad arguments, or
 * due to unsupported features. It should be used when a state occurs which
 * prevents action but is not a serious issue.
 *
 * - YUSUR2_ERROR_ARGUMENT
 * This category is for when a bad or invalid argument is passed. It should be
 * used whenever a function is called and error checking has detected the
 * argument is wrong or incorrect.
 *
 * - YUSUR2_ERROR_UNSUPPORTED
 * This category is for errors which are due to unsupported circumstances or
 * configuration issues. It should not be used when the issue is due to an
 * invalid argument, but for when something has occurred that is unsupported
 * (Ex: Flow control autonegotiation or an unsupported SFP+ module.)
 */

#include "yusur2_osdep.h"

/* Override this by setting IOMEM in your yusur2_osdep.h header */

/* Vendor ID */
#define YUSUR2_INTEL_VENDOR_ID			0x8086

/* Device IDs */
#define YUSUR2_DEV_ID_82598			0x10B6
#define YUSUR2_DEV_ID_82598_BX			0x1508
#define YUSUR2_DEV_ID_82598AF_DUAL_PORT		0x10C6
#define YUSUR2_DEV_ID_82598AF_SINGLE_PORT	0x10C7
#define YUSUR2_DEV_ID_82598AT			0x10C8
#define YUSUR2_DEV_ID_82598AT2			0x150B
#define YUSUR2_DEV_ID_82598EB_SFP_LOM		0x10DB
#define YUSUR2_DEV_ID_82598EB_CX4		0x10DD
#define YUSUR2_DEV_ID_82598_CX4_DUAL_PORT	0x10EC
#define YUSUR2_DEV_ID_82598_DA_DUAL_PORT		0x10F1
#define YUSUR2_DEV_ID_82598_SR_DUAL_PORT_EM	0x10E1
#define YUSUR2_DEV_ID_82598EB_XF_LR		0x10F4
#define YUSUR2_DEV_ID_82599_KX4			0x10F7
#define YUSUR2_DEV_ID_82599_KX4_MEZZ		0x1514
#define YUSUR2_DEV_ID_82599_KR			0x1517
#define YUSUR2_DEV_ID_82599_COMBO_BACKPLANE	0x10F8
#define YUSUR2_SUBDEV_ID_82599_KX4_KR_MEZZ	0x000C
#define YUSUR2_DEV_ID_82599_CX4			0x10F9
#define YUSUR2_DEV_ID_82599_SFP			0x10FB
#define YUSUR2_SUBDEV_ID_82599_SFP		0x11A9
#define YUSUR2_SUBDEV_ID_82599_SFP_WOL0		0x1071
#define YUSUR2_SUBDEV_ID_82599_RNDC		0x1F72
#define YUSUR2_SUBDEV_ID_82599_560FLR		0x17D0
#define YUSUR2_SUBDEV_ID_82599_ECNA_DP		0x0470
#define YUSUR2_SUBDEV_ID_82599_SP_560FLR		0x211B
#define YUSUR2_SUBDEV_ID_82599_LOM_SNAP6		0x2159
#define YUSUR2_SUBDEV_ID_82599_SFP_1OCP		0x000D
#define YUSUR2_SUBDEV_ID_82599_SFP_2OCP		0x0008
#define YUSUR2_SUBDEV_ID_82599_SFP_LOM_OEM1	0x8976
#define YUSUR2_SUBDEV_ID_82599_SFP_LOM_OEM2	0x06EE
#define YUSUR2_DEV_ID_82599_BACKPLANE_FCOE	0x152A
#define YUSUR2_DEV_ID_82599_SFP_FCOE		0x1529
#define YUSUR2_DEV_ID_82599_SFP_EM		0x1507
#define YUSUR2_DEV_ID_82599_SFP_SF2		0x154D
#define YUSUR2_DEV_ID_82599_SFP_SF_QP		0x154A
#define YUSUR2_DEV_ID_82599_QSFP_SF_QP		0x1558
#define YUSUR2_DEV_ID_82599EN_SFP		0x1557
#define YUSUR2_SUBDEV_ID_82599EN_SFP_OCP1	0x0001
#define YUSUR2_DEV_ID_82599_XAUI_LOM		0x10FC
#define YUSUR2_DEV_ID_82599_T3_LOM		0x151C
#define YUSUR2_DEV_ID_82599_VF			0x10ED
#define YUSUR2_DEV_ID_82599_VF_HV		0x152E
#define YUSUR2_DEV_ID_X540T			0x1528
#define YUSUR2_DEV_ID_X540_VF			0x1515
#define YUSUR2_DEV_ID_X540_VF_HV			0x1530
#define YUSUR2_DEV_ID_X540T1			0x1560
#define YUSUR2_DEV_ID_X550T			0x1563
#define YUSUR2_DEV_ID_X550T1			0x15D1
/* Placeholder value, pending official value. */
#define YUSUR2_DEV_ID_X550EM_A_KR		0x15C2
#define YUSUR2_DEV_ID_X550EM_A_KR_L		0x15C3
#define YUSUR2_DEV_ID_X550EM_A_SFP_N		0x15C4
#define YUSUR2_DEV_ID_X550EM_A_SGMII		0x15C6
#define YUSUR2_DEV_ID_X550EM_A_SGMII_L		0x15C7
#define YUSUR2_DEV_ID_X550EM_A_10G_T		0x15C8
#define YUSUR2_DEV_ID_X550EM_A_QSFP		0x15CA
#define YUSUR2_DEV_ID_X550EM_A_QSFP_N		0x15CC
#define YUSUR2_DEV_ID_X550EM_A_SFP		0x15CE
#define YUSUR2_DEV_ID_X550EM_A_1G_T		0x15E4
#define YUSUR2_DEV_ID_X550EM_A_1G_T_L		0x15E5
#define YUSUR2_DEV_ID_X550EM_X_KX4		0x15AA
#define YUSUR2_DEV_ID_X550EM_X_KR		0x15AB
#define YUSUR2_DEV_ID_X550EM_X_SFP		0x15AC
#define YUSUR2_DEV_ID_X550EM_X_10G_T		0x15AD
#define YUSUR2_DEV_ID_X550EM_X_1G_T		0x15AE
#define YUSUR2_DEV_ID_X550EM_X_XFI		0x15B0
#define YUSUR2_DEV_ID_X550_VF_HV			0x1564
#define YUSUR2_DEV_ID_X550_VF			0x1565
#define YUSUR2_DEV_ID_X550EM_A_VF		0x15C5
#define YUSUR2_DEV_ID_X550EM_A_VF_HV		0x15B4
#define YUSUR2_DEV_ID_X550EM_X_VF		0x15A8
#define YUSUR2_DEV_ID_X550EM_X_VF_HV		0x15A9

#define YUSUR2_CAT(r, m) YUSUR2_##r##m

#define YUSUR2_BY_MAC(_hw, r) ((_hw)->mvals[YUSUR2_CAT(r, _IDX)])

/* General Registers */
#define YUSUR2_CTRL		0x00000
#define YUSUR2_STATUS		0x00008
#define YUSUR2_CTRL_EXT		0x00018
#define YUSUR2_ESDP		0x00020
#define YUSUR2_EODSDP		0x00028
#define YUSUR2_I2CCTL_82599	0x00028
#define YUSUR2_I2CCTL		YUSUR2_I2CCTL_82599
#define YUSUR2_I2CCTL_X540	YUSUR2_I2CCTL_82599
#define YUSUR2_I2CCTL_X550	0x15F5C
#define YUSUR2_I2CCTL_X550EM_x	YUSUR2_I2CCTL_X550
#define YUSUR2_I2CCTL_X550EM_a	YUSUR2_I2CCTL_X550
#define YUSUR2_I2CCTL_BY_MAC(_hw) YUSUR2_BY_MAC((_hw), I2CCTL)
#define YUSUR2_PHY_GPIO		0x00028
#define YUSUR2_MAC_GPIO		0x00030
#define YUSUR2_PHYINT_STATUS0	0x00100
#define YUSUR2_PHYINT_STATUS1	0x00104
#define YUSUR2_PHYINT_STATUS2	0x00108
#define YUSUR2_LEDCTL		0x00200
#define YUSUR2_FRTIMER		0x00048
#define YUSUR2_TCPTIMER		0x0004C
#define YUSUR2_CORESPARE		0x00600
#define YUSUR2_EXVET		0x05078

/* NVM Registers */
#define YUSUR2_EEC		0x10010
#define YUSUR2_EEC_X540		YUSUR2_EEC
#define YUSUR2_EEC_X550		YUSUR2_EEC
#define YUSUR2_EEC_X550EM_x	YUSUR2_EEC
#define YUSUR2_EEC_X550EM_a	0x15FF8
#define YUSUR2_EEC_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), EEC)

#define YUSUR2_EERD		0x10014
#define YUSUR2_EEWR		0x10018

#define YUSUR2_FLA		0x1001C
#define YUSUR2_FLA_X540		YUSUR2_FLA
#define YUSUR2_FLA_X550		YUSUR2_FLA
#define YUSUR2_FLA_X550EM_x	YUSUR2_FLA
#define YUSUR2_FLA_X550EM_a	0x15F68
#define YUSUR2_FLA_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), FLA)

#define YUSUR2_EEMNGCTL	0x10110
#define YUSUR2_EEMNGDATA	0x10114
#define YUSUR2_FLMNGCTL	0x10118
#define YUSUR2_FLMNGDATA	0x1011C
#define YUSUR2_FLMNGCNT	0x10120
#define YUSUR2_FLOP	0x1013C

#define YUSUR2_GRC		0x10200
#define YUSUR2_GRC_X540		YUSUR2_GRC
#define YUSUR2_GRC_X550		YUSUR2_GRC
#define YUSUR2_GRC_X550EM_x	YUSUR2_GRC
#define YUSUR2_GRC_X550EM_a	0x15F64
#define YUSUR2_GRC_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), GRC)

#define YUSUR2_SRAMREL		0x10210
#define YUSUR2_SRAMREL_X540	YUSUR2_SRAMREL
#define YUSUR2_SRAMREL_X550	YUSUR2_SRAMREL
#define YUSUR2_SRAMREL_X550EM_x	YUSUR2_SRAMREL
#define YUSUR2_SRAMREL_X550EM_a	0x15F6C
#define YUSUR2_SRAMREL_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SRAMREL)

#define YUSUR2_PHYDBG	0x10218

/* General Receive Control */
#define YUSUR2_GRC_MNG	0x00000001 /* Manageability Enable */
#define YUSUR2_GRC_APME	0x00000002 /* APM enabled in EEPROM */

#define YUSUR2_VPDDIAG0	0x10204
#define YUSUR2_VPDDIAG1	0x10208

/* I2CCTL Bit Masks */
#define YUSUR2_I2C_CLK_IN		0x00000001
#define YUSUR2_I2C_CLK_IN_X540		YUSUR2_I2C_CLK_IN
#define YUSUR2_I2C_CLK_IN_X550		0x00004000
#define YUSUR2_I2C_CLK_IN_X550EM_x	YUSUR2_I2C_CLK_IN_X550
#define YUSUR2_I2C_CLK_IN_X550EM_a	YUSUR2_I2C_CLK_IN_X550
#define YUSUR2_I2C_CLK_IN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), I2C_CLK_IN)

#define YUSUR2_I2C_CLK_OUT		0x00000002
#define YUSUR2_I2C_CLK_OUT_X540		YUSUR2_I2C_CLK_OUT
#define YUSUR2_I2C_CLK_OUT_X550		0x00000200
#define YUSUR2_I2C_CLK_OUT_X550EM_x	YUSUR2_I2C_CLK_OUT_X550
#define YUSUR2_I2C_CLK_OUT_X550EM_a	YUSUR2_I2C_CLK_OUT_X550
#define YUSUR2_I2C_CLK_OUT_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), I2C_CLK_OUT)

#define YUSUR2_I2C_DATA_IN		0x00000004
#define YUSUR2_I2C_DATA_IN_X540		YUSUR2_I2C_DATA_IN
#define YUSUR2_I2C_DATA_IN_X550		0x00001000
#define YUSUR2_I2C_DATA_IN_X550EM_x	YUSUR2_I2C_DATA_IN_X550
#define YUSUR2_I2C_DATA_IN_X550EM_a	YUSUR2_I2C_DATA_IN_X550
#define YUSUR2_I2C_DATA_IN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), I2C_DATA_IN)

#define YUSUR2_I2C_DATA_OUT		0x00000008
#define YUSUR2_I2C_DATA_OUT_X540		YUSUR2_I2C_DATA_OUT
#define YUSUR2_I2C_DATA_OUT_X550		0x00000400
#define YUSUR2_I2C_DATA_OUT_X550EM_x	YUSUR2_I2C_DATA_OUT_X550
#define YUSUR2_I2C_DATA_OUT_X550EM_a	YUSUR2_I2C_DATA_OUT_X550
#define YUSUR2_I2C_DATA_OUT_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), I2C_DATA_OUT)

#define YUSUR2_I2C_DATA_OE_N_EN		0
#define YUSUR2_I2C_DATA_OE_N_EN_X540	YUSUR2_I2C_DATA_OE_N_EN
#define YUSUR2_I2C_DATA_OE_N_EN_X550	0x00000800
#define YUSUR2_I2C_DATA_OE_N_EN_X550EM_x	YUSUR2_I2C_DATA_OE_N_EN_X550
#define YUSUR2_I2C_DATA_OE_N_EN_X550EM_a	YUSUR2_I2C_DATA_OE_N_EN_X550
#define YUSUR2_I2C_DATA_OE_N_EN_BY_MAC(_hw) YUSUR2_BY_MAC((_hw), I2C_DATA_OE_N_EN)

#define YUSUR2_I2C_BB_EN			0
#define YUSUR2_I2C_BB_EN_X540		YUSUR2_I2C_BB_EN
#define YUSUR2_I2C_BB_EN_X550		0x00000100
#define YUSUR2_I2C_BB_EN_X550EM_x	YUSUR2_I2C_BB_EN_X550
#define YUSUR2_I2C_BB_EN_X550EM_a	YUSUR2_I2C_BB_EN_X550
#define YUSUR2_I2C_BB_EN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), I2C_BB_EN)

#define YUSUR2_I2C_CLK_OE_N_EN		0
#define YUSUR2_I2C_CLK_OE_N_EN_X540	YUSUR2_I2C_CLK_OE_N_EN
#define YUSUR2_I2C_CLK_OE_N_EN_X550	0x00002000
#define YUSUR2_I2C_CLK_OE_N_EN_X550EM_x	YUSUR2_I2C_CLK_OE_N_EN_X550
#define YUSUR2_I2C_CLK_OE_N_EN_X550EM_a	YUSUR2_I2C_CLK_OE_N_EN_X550
#define YUSUR2_I2C_CLK_OE_N_EN_BY_MAC(_hw) YUSUR2_BY_MAC((_hw), I2C_CLK_OE_N_EN)
#define YUSUR2_I2C_CLOCK_STRETCHING_TIMEOUT	500

#define YUSUR2_I2C_THERMAL_SENSOR_ADDR	0xF8
#define YUSUR2_EMC_INTERNAL_DATA		0x00
#define YUSUR2_EMC_INTERNAL_THERM_LIMIT	0x20
#define YUSUR2_EMC_DIODE1_DATA		0x01
#define YUSUR2_EMC_DIODE1_THERM_LIMIT	0x19
#define YUSUR2_EMC_DIODE2_DATA		0x23
#define YUSUR2_EMC_DIODE2_THERM_LIMIT	0x1A

#define YUSUR2_MAX_SENSORS		3

struct yusur2_thermal_diode_data {
	u8 location;
	u8 temp;
	u8 caution_thresh;
	u8 max_op_thresh;
};

struct yusur2_thermal_sensor_data {
	struct yusur2_thermal_diode_data sensor[YUSUR2_MAX_SENSORS];
};


#define NVM_OROM_OFFSET		0x17
#define NVM_OROM_BLK_LOW	0x83
#define NVM_OROM_BLK_HI		0x84
#define NVM_OROM_PATCH_MASK	0xFF
#define NVM_OROM_SHIFT		8

#define NVM_VER_MASK		0x00FF /* version mask */
#define NVM_VER_SHIFT		8     /* version bit shift */
#define NVM_OEM_PROD_VER_PTR	0x1B  /* OEM Product version block pointer */
#define NVM_OEM_PROD_VER_CAP_OFF 0x1  /* OEM Product version format offset */
#define NVM_OEM_PROD_VER_OFF_L	0x2   /* OEM Product version offset low */
#define NVM_OEM_PROD_VER_OFF_H	0x3   /* OEM Product version offset high */
#define NVM_OEM_PROD_VER_CAP_MASK 0xF /* OEM Product version cap mask */
#define NVM_OEM_PROD_VER_MOD_LEN 0x3  /* OEM Product version module length */
#define NVM_ETK_OFF_LOW		0x2D  /* version low order word */
#define NVM_ETK_OFF_HI		0x2E  /* version high order word */
#define NVM_ETK_SHIFT		16    /* high version word shift */
#define NVM_VER_INVALID		0xFFFF
#define NVM_ETK_VALID		0x8000
#define NVM_INVALID_PTR		0xFFFF
#define NVM_VER_SIZE		32    /* version sting size */

struct yusur2_nvm_version {
	u32 etk_id;
	u8  nvm_major;
	u16 nvm_minor;
	u8  nvm_id;

	bool oem_valid;
	u8   oem_major;
	u8   oem_minor;
	u16  oem_release;

	bool or_valid;
	u8  or_major;
	u16 or_build;
	u8  or_patch;

};

/* Interrupt Registers */
#define YUSUR2_EICR		0x00800
#define YUSUR2_EICS		0x00808
#define YUSUR2_EIMS		0x00880
#define YUSUR2_EIMC		0x00888
#define YUSUR2_EIAC		0x00810
#define YUSUR2_EIAM		0x00890
#define YUSUR2_EICS_EX(_i)	(0x00A90 + (_i) * 4)
#define YUSUR2_EIMS_EX(_i)	(0x00AA0 + (_i) * 4)
#define YUSUR2_EIMC_EX(_i)	(0x00AB0 + (_i) * 4)
#define YUSUR2_EIAM_EX(_i)	(0x00AD0 + (_i) * 4)
/* 82599 EITR is only 12 bits, with the lower 3 always zero */
/*
 * 82598 EITR is 16 bits but set the limits based on the max
 * supported by all yusur2 hardware
 */
#define YUSUR2_MAX_INT_RATE	488281
#define YUSUR2_MIN_INT_RATE	956
#define YUSUR2_MAX_EITR		0x00000FF8
#define YUSUR2_MIN_EITR		8
#define YUSUR2_EITR(_i)		(((_i) <= 23) ? (0x00820 + ((_i) * 4)) : \
				 (0x012300 + (((_i) - 24) * 4)))
#define YUSUR2_EITR_ITR_INT_MASK	0x00000FF8
#define YUSUR2_EITR_LLI_MOD	0x00008000
#define YUSUR2_EITR_CNT_WDIS	0x80000000
#define YUSUR2_IVAR(_i)		(0x00900 + ((_i) * 4)) /* 24 at 0x900-0x960 */
#define YUSUR2_IVAR_MISC		0x00A00 /* misc MSI-X interrupt causes */
#define YUSUR2_EITRSEL		0x00894
#define YUSUR2_MSIXT		0x00000 /* MSI-X Table. 0x0000 - 0x01C */
#define YUSUR2_MSIXPBA		0x02000 /* MSI-X Pending bit array */
#define YUSUR2_PBACL(_i)	(((_i) == 0) ? (0x11068) : (0x110C0 + ((_i) * 4)))
#define YUSUR2_GPIE		0x00898

/* Flow Control Registers */
#define YUSUR2_FCADBUL		0x03210
#define YUSUR2_FCADBUH		0x03214
#define YUSUR2_FCAMACL		0x04328
#define YUSUR2_FCAMACH		0x0432C
#define YUSUR2_FCRTH_82599(_i)	(0x03260 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_FCRTL_82599(_i)	(0x03220 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_PFCTOP		0x03008
#define YUSUR2_FCTTV(_i)		(0x03200 + ((_i) * 4)) /* 4 of these (0-3) */
#define YUSUR2_FCRTL(_i)		(0x03220 + ((_i) * 8)) /* 8 of these (0-7) */
#define YUSUR2_FCRTH(_i)		(0x03260 + ((_i) * 8)) /* 8 of these (0-7) */
#define YUSUR2_FCRTV		0x032A0
#define YUSUR2_FCCFG		0x03D00
#define YUSUR2_TFCS		0x0CE00

/* Receive DMA Registers */
#define YUSUR2_RDBAL(_i)	(((_i) < 64) ? (0x01000 + ((_i) * 0x40)) : \
			 (0x0D000 + (((_i) - 64) * 0x40)))
#define YUSUR2_RDBAH(_i)	(((_i) < 64) ? (0x01004 + ((_i) * 0x40)) : \
			 (0x0D004 + (((_i) - 64) * 0x40)))
#define YUSUR2_RDLEN(_i)	(((_i) < 64) ? (0x01008 + ((_i) * 0x40)) : \
			 (0x0D008 + (((_i) - 64) * 0x40)))
#define YUSUR2_RDH(_i)	(((_i) < 64) ? (0x01010 + ((_i) * 0x40)) : \
			 (0x0D010 + (((_i) - 64) * 0x40)))
#define YUSUR2_RDT(_i)	(((_i) < 64) ? (0x01018 + ((_i) * 0x40)) : \
			 (0x0D018 + (((_i) - 64) * 0x40)))
#define YUSUR2_RXDCTL(_i)	(((_i) < 64) ? (0x01028 + ((_i) * 0x40)) : \
				 (0x0D028 + (((_i) - 64) * 0x40)))
#define YUSUR2_RSCCTL(_i)	(((_i) < 64) ? (0x0102C + ((_i) * 0x40)) : \
				 (0x0D02C + (((_i) - 64) * 0x40)))
#define YUSUR2_RSCDBU	0x03028
#define YUSUR2_RDDCC	0x02F20
#define YUSUR2_RXMEMWRAP	0x03190
#define YUSUR2_STARCTRL	0x03024
/*
 * Split and Replication Receive Control Registers
 * 00-15 : 0x02100 + n*4
 * 16-64 : 0x01014 + n*0x40
 * 64-127: 0x0D014 + (n-64)*0x40
 */
#define YUSUR2_SRRCTL(_i)	(((_i) <= 15) ? (0x02100 + ((_i) * 4)) : \
				 (((_i) < 64) ? (0x01014 + ((_i) * 0x40)) : \
				 (0x0D014 + (((_i) - 64) * 0x40))))
/*
 * Rx DCA Control Register:
 * 00-15 : 0x02200 + n*4
 * 16-64 : 0x0100C + n*0x40
 * 64-127: 0x0D00C + (n-64)*0x40
 */
#define YUSUR2_DCA_RXCTRL(_i)	(((_i) <= 15) ? (0x02200 + ((_i) * 4)) : \
				 (((_i) < 64) ? (0x0100C + ((_i) * 0x40)) : \
				 (0x0D00C + (((_i) - 64) * 0x40))))
#define YUSUR2_RDRXCTL		0x02F00
/* 8 of these 0x03C00 - 0x03C1C */
#define YUSUR2_RXPBSIZE(_i)	(0x03C00 + ((_i) * 4))
#define YUSUR2_RXCTRL		0x03000
#define YUSUR2_DROPEN		0x03D04
#define YUSUR2_RXPBSIZE_SHIFT	10
#define YUSUR2_RXPBSIZE_MASK	0x000FFC00

/* Receive Registers */
#define YUSUR2_RXCSUM		0x05000
#define YUSUR2_RFCTL		0x05008
#define YUSUR2_DRECCCTL		0x02F08
#define YUSUR2_DRECCCTL_DISABLE	0
#define YUSUR2_DRECCCTL2		0x02F8C

/* Multicast Table Array - 128 entries */
#define YUSUR2_MTA(_i)		(0x05200 + ((_i) * 4))
#define YUSUR2_RAL(_i)		(((_i) <= 15) ? (0x05400 + ((_i) * 8)) : \
				 (0x0A200 + ((_i) * 8)))
#define YUSUR2_RAH(_i)		(((_i) <= 15) ? (0x05404 + ((_i) * 8)) : \
				 (0x0A204 + ((_i) * 8)))
#define YUSUR2_MPSAR_LO(_i)	(0x0A600 + ((_i) * 8))
#define YUSUR2_MPSAR_HI(_i)	(0x0A604 + ((_i) * 8))
/* Packet split receive type */
#define YUSUR2_PSRTYPE(_i)	(((_i) <= 15) ? (0x05480 + ((_i) * 4)) : \
				 (0x0EA00 + ((_i) * 4)))
/* array of 4096 1-bit vlan filters */
#define YUSUR2_VFTA(_i)		(0x0A000 + ((_i) * 4))
/*array of 4096 4-bit vlan vmdq indices */
#define YUSUR2_VFTAVIND(_j, _i)	(0x0A200 + ((_j) * 0x200) + ((_i) * 4))
#define YUSUR2_FCTRL		0x05080
#define YUSUR2_VLNCTRL		0x05088
#define YUSUR2_MCSTCTRL		0x05090
#define YUSUR2_MRQC		0x05818
#define YUSUR2_SAQF(_i)	(0x0E000 + ((_i) * 4)) /* Source Address Queue Filter */
#define YUSUR2_DAQF(_i)	(0x0E200 + ((_i) * 4)) /* Dest. Address Queue Filter */
#define YUSUR2_SDPQF(_i)	(0x0E400 + ((_i) * 4)) /* Src Dest. Addr Queue Filter */
#define YUSUR2_FTQF(_i)	(0x0E600 + ((_i) * 4)) /* Five Tuple Queue Filter */
#define YUSUR2_ETQF(_i)	(0x05128 + ((_i) * 4)) /* EType Queue Filter */
#define YUSUR2_ETQS(_i)	(0x0EC00 + ((_i) * 4)) /* EType Queue Select */
#define YUSUR2_SYNQF	0x0EC30 /* SYN Packet Queue Filter */
#define YUSUR2_RQTC	0x0EC70
#define YUSUR2_MTQC	0x08120
#define YUSUR2_VLVF(_i)	(0x0F100 + ((_i) * 4))  /* 64 of these (0-63) */
#define YUSUR2_VLVFB(_i)	(0x0F200 + ((_i) * 4))  /* 128 of these (0-127) */
#define YUSUR2_VMVIR(_i)	(0x08000 + ((_i) * 4))  /* 64 of these (0-63) */
#define YUSUR2_PFFLPL		0x050B0
#define YUSUR2_PFFLPH		0x050B4
#define YUSUR2_VT_CTL		0x051B0
#define YUSUR2_PFMAILBOX(_i)	(0x04B00 + (4 * (_i))) /* 64 total */
/* 64 Mailboxes, 16 DW each */
#define YUSUR2_PFMBMEM(_i)	(0x13000 + (64 * (_i)))
#define YUSUR2_PFMBICR(_i)	(0x00710 + (4 * (_i))) /* 4 total */
#define YUSUR2_PFMBIMR(_i)	(0x00720 + (4 * (_i))) /* 4 total */
#define YUSUR2_VFRE(_i)		(0x051E0 + ((_i) * 4))
#define YUSUR2_VFTE(_i)		(0x08110 + ((_i) * 4))
#define YUSUR2_VMECM(_i)		(0x08790 + ((_i) * 4))
#define YUSUR2_QDE		0x2F04
#define YUSUR2_VMTXSW(_i)	(0x05180 + ((_i) * 4)) /* 2 total */
#define YUSUR2_VMOLR(_i)		(0x0F000 + ((_i) * 4)) /* 64 total */
#define YUSUR2_UTA(_i)		(0x0F400 + ((_i) * 4))
#define YUSUR2_MRCTL(_i)		(0x0F600 + ((_i) * 4))
#define YUSUR2_VMRVLAN(_i)	(0x0F610 + ((_i) * 4))
#define YUSUR2_VMRVM(_i)		(0x0F630 + ((_i) * 4))
#define YUSUR2_LVMMC_RX		0x2FA8
#define YUSUR2_LVMMC_TX		0x8108
#define YUSUR2_LMVM_RX		0x2FA4
#define YUSUR2_LMVM_TX		0x8124
#define YUSUR2_WQBR_RX(_i)	(0x2FB0 + ((_i) * 4)) /* 4 total */
#define YUSUR2_WQBR_TX(_i)	(0x8130 + ((_i) * 4)) /* 4 total */
#define YUSUR2_L34T_IMIR(_i)	(0x0E800 + ((_i) * 4)) /*128 of these (0-127)*/
#define YUSUR2_RXFECCERR0	0x051B8
#define YUSUR2_LLITHRESH		0x0EC90
#define YUSUR2_IMIR(_i)		(0x05A80 + ((_i) * 4))  /* 8 of these (0-7) */
#define YUSUR2_IMIREXT(_i)	(0x05AA0 + ((_i) * 4))  /* 8 of these (0-7) */
#define YUSUR2_IMIRVP		0x05AC0
#define YUSUR2_VMD_CTL		0x0581C
#define YUSUR2_RETA(_i)		(0x05C00 + ((_i) * 4))  /* 32 of these (0-31) */
#define YUSUR2_ERETA(_i)		(0x0EE80 + ((_i) * 4))  /* 96 of these (0-95) */
#define YUSUR2_RSSRK(_i)		(0x05C80 + ((_i) * 4))  /* 10 of these (0-9) */

/* Registers for setting up RSS on X550 with SRIOV
 * _p - pool number (0..63)
 * _i - index (0..10 for PFVFRSSRK, 0..15 for PFVFRETA)
 */
#define YUSUR2_PFVFMRQC(_p)	(0x03400 + ((_p) * 4))
#define YUSUR2_PFVFRSSRK(_i, _p)	(0x018000 + ((_i) * 4) + ((_p) * 0x40))
#define YUSUR2_PFVFRETA(_i, _p)	(0x019000 + ((_i) * 4) + ((_p) * 0x40))

/* Flow Director registers */
#define YUSUR2_FDIRCTRL	0x0EE00
#define YUSUR2_FDIRHKEY	0x0EE68
#define YUSUR2_FDIRSKEY	0x0EE6C
#define YUSUR2_FDIRDIP4M	0x0EE3C
#define YUSUR2_FDIRSIP4M	0x0EE40
#define YUSUR2_FDIRTCPM	0x0EE44
#define YUSUR2_FDIRUDPM	0x0EE48
#define YUSUR2_FDIRSCTPM	0x0EE78
#define YUSUR2_FDIRIP6M	0x0EE74
#define YUSUR2_FDIRM	0x0EE70

/* Flow Director Stats registers */
#define YUSUR2_FDIRFREE	0x0EE38
#define YUSUR2_FDIRLEN	0x0EE4C
#define YUSUR2_FDIRUSTAT	0x0EE50
#define YUSUR2_FDIRFSTAT	0x0EE54
#define YUSUR2_FDIRMATCH	0x0EE58
#define YUSUR2_FDIRMISS	0x0EE5C

/* Flow Director Programming registers */
#define YUSUR2_FDIRSIPv6(_i) (0x0EE0C + ((_i) * 4)) /* 3 of these (0-2) */
#define YUSUR2_FDIRIPSA	0x0EE18
#define YUSUR2_FDIRIPDA	0x0EE1C
#define YUSUR2_FDIRPORT	0x0EE20
#define YUSUR2_FDIRVLAN	0x0EE24
#define YUSUR2_FDIRHASH	0x0EE28
#define YUSUR2_FDIRCMD	0x0EE2C

/* Transmit DMA registers */
#define YUSUR2_TDBAL(_i)		(0x06000 + ((_i) * 0x40)) /* 32 of them (0-31)*/
#define YUSUR2_TDBAH(_i)		(0x06004 + ((_i) * 0x40))
#define YUSUR2_TDLEN(_i)		(0x06008 + ((_i) * 0x40))
#define YUSUR2_TDH(_i)		(0x06010 + ((_i) * 0x40))
#define YUSUR2_TDT(_i)		(0x06018 + ((_i) * 0x40))
#define YUSUR2_TXDCTL(_i)	(0x06028 + ((_i) * 0x40))
#define YUSUR2_TDWBAL(_i)	(0x06038 + ((_i) * 0x40))
#define YUSUR2_TDWBAH(_i)	(0x0603C + ((_i) * 0x40))
#define YUSUR2_DTXCTL		0x07E00

#define YUSUR2_DMATXCTL		0x04A80
#define YUSUR2_PFVFSPOOF(_i)	(0x08200 + ((_i) * 4)) /* 8 of these 0 - 7 */
#define YUSUR2_PFDTXGSWC		0x08220
#define YUSUR2_DTXMXSZRQ		0x08100
#define YUSUR2_DTXTCPFLGL	0x04A88
#define YUSUR2_DTXTCPFLGH	0x04A8C
#define YUSUR2_LBDRPEN		0x0CA00
#define YUSUR2_TXPBTHRESH(_i)	(0x04950 + ((_i) * 4)) /* 8 of these 0 - 7 */

#define YUSUR2_DMATXCTL_TE	0x1 /* Transmit Enable */
#define YUSUR2_DMATXCTL_NS	0x2 /* No Snoop LSO hdr buffer */
#define YUSUR2_DMATXCTL_GDV	0x8 /* Global Double VLAN */
#define YUSUR2_DMATXCTL_MDP_EN	0x20 /* Bit 5 */
#define YUSUR2_DMATXCTL_MBINTEN	0x40 /* Bit 6 */
#define YUSUR2_DMATXCTL_VT_SHIFT	16  /* VLAN EtherType */

#define YUSUR2_PFDTXGSWC_VT_LBEN	0x1 /* Local L2 VT switch enable */

/* Anti-spoofing defines */
#define YUSUR2_SPOOF_MACAS_MASK		0xFF
#define YUSUR2_SPOOF_VLANAS_MASK		0xFF00
#define YUSUR2_SPOOF_VLANAS_SHIFT	8
#define YUSUR2_SPOOF_ETHERTYPEAS		0xFF000000
#define YUSUR2_SPOOF_ETHERTYPEAS_SHIFT	16
#define YUSUR2_PFVFSPOOF_REG_COUNT	8
/* 16 of these (0-15) */
#define YUSUR2_DCA_TXCTRL(_i)		(0x07200 + ((_i) * 4))
/* Tx DCA Control register : 128 of these (0-127) */
#define YUSUR2_DCA_TXCTRL_82599(_i)	(0x0600C + ((_i) * 0x40))
#define YUSUR2_TIPG			0x0CB00
#define YUSUR2_TXPBSIZE(_i)		(0x0CC00 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_MNGTXMAP			0x0CD10
#define YUSUR2_TIPG_FIBER_DEFAULT	3
#define YUSUR2_TXPBSIZE_SHIFT		10

/* Wake up registers */
#define YUSUR2_WUC	0x05800
#define YUSUR2_WUFC	0x05808
#define YUSUR2_WUS	0x05810
#define YUSUR2_IPAV	0x05838
#define YUSUR2_IP4AT	0x05840 /* IPv4 table 0x5840-0x5858 */
#define YUSUR2_IP6AT	0x05880 /* IPv6 table 0x5880-0x588F */

#define YUSUR2_WUPL	0x05900
#define YUSUR2_WUPM	0x05A00 /* wake up pkt memory 0x5A00-0x5A7C */
#define YUSUR2_PROXYS	0x05F60 /* Proxying Status Register */
#define YUSUR2_PROXYFC	0x05F64 /* Proxying Filter Control Register */
#define YUSUR2_VXLANCTRL	0x0000507C /* Rx filter VXLAN UDPPORT Register */

/* masks for accessing VXLAN and GENEVE UDP ports */
#define YUSUR2_VXLANCTRL_VXLAN_UDPPORT_MASK	0x0000ffff /* VXLAN port */
#define YUSUR2_VXLANCTRL_GENEVE_UDPPORT_MASK	0xffff0000 /* GENEVE port */
#define YUSUR2_VXLANCTRL_ALL_UDPPORT_MASK	0xffffffff /* GENEVE/VXLAN */
#define YUSUR2_VXLANCTRL_GENEVE_UDPPORT_SHIFT	16

#define YUSUR2_FHFT(_n)	(0x09000 + ((_n) * 0x100)) /* Flex host filter table */
/* Ext Flexible Host Filter Table */
#define YUSUR2_FHFT_EXT(_n)	(0x09800 + ((_n) * 0x100))
#define YUSUR2_FHFT_EXT_X550(_n)	(0x09600 + ((_n) * 0x100))

/* Four Flexible Filters are supported */
#define YUSUR2_FLEXIBLE_FILTER_COUNT_MAX		4
/* Six Flexible Filters are supported */
#define YUSUR2_FLEXIBLE_FILTER_COUNT_MAX_6	6
/* Eight Flexible Filters are supported */
#define YUSUR2_FLEXIBLE_FILTER_COUNT_MAX_8	8
#define YUSUR2_EXT_FLEXIBLE_FILTER_COUNT_MAX	2

/* Each Flexible Filter is at most 128 (0x80) bytes in length */
#define YUSUR2_FLEXIBLE_FILTER_SIZE_MAX		128
#define YUSUR2_FHFT_LENGTH_OFFSET		0xFC  /* Length byte in FHFT */
#define YUSUR2_FHFT_LENGTH_MASK			0x0FF /* Length in lower byte */

/* Definitions for power management and wakeup registers */
/* Wake Up Control */
#define YUSUR2_WUC_PME_EN	0x00000002 /* PME Enable */
#define YUSUR2_WUC_PME_STATUS	0x00000004 /* PME Status */
#define YUSUR2_WUC_WKEN		0x00000010 /* Enable PE_WAKE_N pin assertion  */

/* Wake Up Filter Control */
#define YUSUR2_WUFC_LNKC	0x00000001 /* Link Status Change Wakeup Enable */
#define YUSUR2_WUFC_MAG	0x00000002 /* Magic Packet Wakeup Enable */
#define YUSUR2_WUFC_EX	0x00000004 /* Directed Exact Wakeup Enable */
#define YUSUR2_WUFC_MC	0x00000008 /* Directed Multicast Wakeup Enable */
#define YUSUR2_WUFC_BC	0x00000010 /* Broadcast Wakeup Enable */
#define YUSUR2_WUFC_ARP	0x00000020 /* ARP Request Packet Wakeup Enable */
#define YUSUR2_WUFC_IPV4	0x00000040 /* Directed IPv4 Packet Wakeup Enable */
#define YUSUR2_WUFC_IPV6	0x00000080 /* Directed IPv6 Packet Wakeup Enable */
#define YUSUR2_WUFC_MNG	0x00000100 /* Directed Mgmt Packet Wakeup Enable */

#define YUSUR2_WUFC_IGNORE_TCO	0x00008000 /* Ignore WakeOn TCO packets */
#define YUSUR2_WUFC_FLX0	0x00010000 /* Flexible Filter 0 Enable */
#define YUSUR2_WUFC_FLX1	0x00020000 /* Flexible Filter 1 Enable */
#define YUSUR2_WUFC_FLX2	0x00040000 /* Flexible Filter 2 Enable */
#define YUSUR2_WUFC_FLX3	0x00080000 /* Flexible Filter 3 Enable */
#define YUSUR2_WUFC_FLX4	0x00100000 /* Flexible Filter 4 Enable */
#define YUSUR2_WUFC_FLX5	0x00200000 /* Flexible Filter 5 Enable */
#define YUSUR2_WUFC_FLX_FILTERS		0x000F0000 /* Mask for 4 flex filters */
#define YUSUR2_WUFC_FLX_FILTERS_6	0x003F0000 /* Mask for 6 flex filters */
#define YUSUR2_WUFC_FLX_FILTERS_8	0x00FF0000 /* Mask for 8 flex filters */
#define YUSUR2_WUFC_FW_RST_WK	0x80000000 /* Ena wake on FW reset assertion */
/* Mask for Ext. flex filters */
#define YUSUR2_WUFC_EXT_FLX_FILTERS	0x00300000
#define YUSUR2_WUFC_ALL_FILTERS		0x000F00FF /* Mask all 4 flex filters */
#define YUSUR2_WUFC_ALL_FILTERS_6	0x003F00FF /* Mask all 6 flex filters */
#define YUSUR2_WUFC_ALL_FILTERS_8	0x00FF00FF /* Mask all 8 flex filters */
#define YUSUR2_WUFC_FLX_OFFSET	16 /* Offset to the Flexible Filters bits */

/* Wake Up Status */
#define YUSUR2_WUS_LNKC		YUSUR2_WUFC_LNKC
#define YUSUR2_WUS_MAG		YUSUR2_WUFC_MAG
#define YUSUR2_WUS_EX		YUSUR2_WUFC_EX
#define YUSUR2_WUS_MC		YUSUR2_WUFC_MC
#define YUSUR2_WUS_BC		YUSUR2_WUFC_BC
#define YUSUR2_WUS_ARP		YUSUR2_WUFC_ARP
#define YUSUR2_WUS_IPV4		YUSUR2_WUFC_IPV4
#define YUSUR2_WUS_IPV6		YUSUR2_WUFC_IPV6
#define YUSUR2_WUS_MNG		YUSUR2_WUFC_MNG
#define YUSUR2_WUS_FLX0		YUSUR2_WUFC_FLX0
#define YUSUR2_WUS_FLX1		YUSUR2_WUFC_FLX1
#define YUSUR2_WUS_FLX2		YUSUR2_WUFC_FLX2
#define YUSUR2_WUS_FLX3		YUSUR2_WUFC_FLX3
#define YUSUR2_WUS_FLX4		YUSUR2_WUFC_FLX4
#define YUSUR2_WUS_FLX5		YUSUR2_WUFC_FLX5
#define YUSUR2_WUS_FLX_FILTERS	YUSUR2_WUFC_FLX_FILTERS
#define YUSUR2_WUS_FW_RST_WK	YUSUR2_WUFC_FW_RST_WK
/* Proxy Status */
#define YUSUR2_PROXYS_EX		0x00000004 /* Exact packet received */
#define YUSUR2_PROXYS_ARP_DIR	0x00000020 /* ARP w/filter match received */
#define YUSUR2_PROXYS_NS		0x00000200 /* IPV6 NS received */
#define YUSUR2_PROXYS_NS_DIR	0x00000400 /* IPV6 NS w/DA match received */
#define YUSUR2_PROXYS_ARP	0x00000800 /* ARP request packet received */
#define YUSUR2_PROXYS_MLD	0x00001000 /* IPv6 MLD packet received */

/* Proxying Filter Control */
#define YUSUR2_PROXYFC_ENABLE	0x00000001 /* Port Proxying Enable */
#define YUSUR2_PROXYFC_EX	0x00000004 /* Directed Exact Proxy Enable */
#define YUSUR2_PROXYFC_ARP_DIR	0x00000020 /* Directed ARP Proxy Enable */
#define YUSUR2_PROXYFC_NS	0x00000200 /* IPv6 Neighbor Solicitation */
#define YUSUR2_PROXYFC_ARP	0x00000800 /* ARP Request Proxy Enable */
#define YUSUR2_PROXYFC_MLD	0x00000800 /* IPv6 MLD Proxy Enable */
#define YUSUR2_PROXYFC_NO_TCO	0x00008000 /* Ignore TCO packets */

#define YUSUR2_WUPL_LENGTH_MASK	0xFFFF

/* DCB registers */
#define YUSUR2_DCB_MAX_TRAFFIC_CLASS	8
#define YUSUR2_RMCS		0x03D00
#define YUSUR2_DPMCS		0x07F40
#define YUSUR2_PDPMCS		0x0CD00
#define YUSUR2_RUPPBMR		0x050A0
#define YUSUR2_RT2CR(_i)		(0x03C20 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RT2SR(_i)		(0x03C40 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_TDTQ2TCCR(_i)	(0x0602C + ((_i) * 0x40)) /* 8 of these (0-7) */
#define YUSUR2_TDTQ2TCSR(_i)	(0x0622C + ((_i) * 0x40)) /* 8 of these (0-7) */
#define YUSUR2_TDPT2TCCR(_i)	(0x0CD20 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_TDPT2TCSR(_i)	(0x0CD40 + ((_i) * 4)) /* 8 of these (0-7) */

/* Power Management */
/* DMA Coalescing configuration */
struct yusur2_dmac_config {
	u16	watchdog_timer; /* usec units */
	bool	fcoe_en;
	u32	link_speed;
	u8	fcoe_tc;
	u8	num_tcs;
};

/*
 * DMA Coalescing threshold Rx PB TC[n] value in Kilobyte by link speed.
 * DMACRXT = 10Gbps = 10,000 bits / usec = 1250 bytes / usec 70 * 1250 ==
 * 87500 bytes [85KB]
 */
#define YUSUR2_DMACRXT_10G		0x55
#define YUSUR2_DMACRXT_1G		0x09
#define YUSUR2_DMACRXT_100M		0x01

/* DMA Coalescing registers */
#define YUSUR2_DMCMNGTH			0x15F20 /* Management Threshold */
#define YUSUR2_DMACR			0x02400 /* Control register */
#define YUSUR2_DMCTH(_i)			(0x03300 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_DMCTLX			0x02404 /* Time to Lx request */
/* DMA Coalescing register fields */
#define YUSUR2_DMCMNGTH_DMCMNGTH_MASK	0x000FFFF0 /* Mng Threshold mask */
#define YUSUR2_DMCMNGTH_DMCMNGTH_SHIFT	4 /* Management Threshold shift */
#define YUSUR2_DMACR_DMACWT_MASK		0x0000FFFF /* Watchdog Timer mask */
#define YUSUR2_DMACR_HIGH_PRI_TC_MASK	0x00FF0000
#define YUSUR2_DMACR_HIGH_PRI_TC_SHIFT	16
#define YUSUR2_DMACR_EN_MNG_IND		0x10000000 /* Enable Mng Indications */
#define YUSUR2_DMACR_LX_COAL_IND		0x40000000 /* Lx Coalescing indicate */
#define YUSUR2_DMACR_DMAC_EN		0x80000000 /* DMA Coalescing Enable */
#define YUSUR2_DMCTH_DMACRXT_MASK	0x000001FF /* Receive Threshold mask */
#define YUSUR2_DMCTLX_TTLX_MASK		0x00000FFF /* Time to Lx request mask */

/* EEE registers */
#define YUSUR2_EEER			0x043A0 /* EEE register */
#define YUSUR2_EEE_STAT			0x04398 /* EEE Status */
#define YUSUR2_EEE_SU			0x04380 /* EEE Set up */
#define YUSUR2_EEE_SU_TEEE_DLY_SHIFT	26
#define YUSUR2_TLPIC			0x041F4 /* EEE Tx LPI count */
#define YUSUR2_RLPIC			0x041F8 /* EEE Rx LPI count */

/* EEE register fields */
#define YUSUR2_EEER_TX_LPI_EN		0x00010000 /* Enable EEE LPI TX path */
#define YUSUR2_EEER_RX_LPI_EN		0x00020000 /* Enable EEE LPI RX path */
#define YUSUR2_EEE_STAT_NEG		0x20000000 /* EEE support neg on link */
#define YUSUR2_EEE_RX_LPI_STATUS		0x40000000 /* RX Link in LPI status */
#define YUSUR2_EEE_TX_LPI_STATUS		0x80000000 /* TX Link in LPI status */

/* Security Control Registers */
#define YUSUR2_SECTXCTRL		0x08800
#define YUSUR2_SECTXSTAT		0x08804
#define YUSUR2_SECTXBUFFAF	0x08808
#define YUSUR2_SECTXMINIFG	0x08810
#define YUSUR2_SECRXCTRL		0x08D00
#define YUSUR2_SECRXSTAT		0x08D04

/* Security Bit Fields and Masks */
#define YUSUR2_SECTXCTRL_SECTX_DIS	0x00000001
#define YUSUR2_SECTXCTRL_TX_DIS		0x00000002
#define YUSUR2_SECTXCTRL_STORE_FORWARD	0x00000004

#define YUSUR2_SECTXSTAT_SECTX_RDY	0x00000001
#define YUSUR2_SECTXSTAT_ECC_TXERR	0x00000002

#define YUSUR2_SECRXCTRL_SECRX_DIS	0x00000001
#define YUSUR2_SECRXCTRL_RX_DIS		0x00000002

#define YUSUR2_SECRXSTAT_SECRX_RDY	0x00000001
#define YUSUR2_SECRXSTAT_ECC_RXERR	0x00000002

/* LinkSec (MacSec) Registers */
#define YUSUR2_LSECTXCAP		0x08A00
#define YUSUR2_LSECRXCAP		0x08F00
#define YUSUR2_LSECTXCTRL	0x08A04
#define YUSUR2_LSECTXSCL		0x08A08 /* SCI Low */
#define YUSUR2_LSECTXSCH		0x08A0C /* SCI High */
#define YUSUR2_LSECTXSA		0x08A10
#define YUSUR2_LSECTXPN0		0x08A14
#define YUSUR2_LSECTXPN1		0x08A18
#define YUSUR2_LSECTXKEY0(_n)	(0x08A1C + (4 * (_n))) /* 4 of these (0-3) */
#define YUSUR2_LSECTXKEY1(_n)	(0x08A2C + (4 * (_n))) /* 4 of these (0-3) */
#define YUSUR2_LSECRXCTRL	0x08F04
#define YUSUR2_LSECRXSCL		0x08F08
#define YUSUR2_LSECRXSCH		0x08F0C
#define YUSUR2_LSECRXSA(_i)	(0x08F10 + (4 * (_i))) /* 2 of these (0-1) */
#define YUSUR2_LSECRXPN(_i)	(0x08F18 + (4 * (_i))) /* 2 of these (0-1) */
#define YUSUR2_LSECRXKEY(_n, _m)	(0x08F20 + ((0x10 * (_n)) + (4 * (_m))))
#define YUSUR2_LSECTXUT		0x08A3C /* OutPktsUntagged */
#define YUSUR2_LSECTXPKTE	0x08A40 /* OutPktsEncrypted */
#define YUSUR2_LSECTXPKTP	0x08A44 /* OutPktsProtected */
#define YUSUR2_LSECTXOCTE	0x08A48 /* OutOctetsEncrypted */
#define YUSUR2_LSECTXOCTP	0x08A4C /* OutOctetsProtected */
#define YUSUR2_LSECRXUT		0x08F40 /* InPktsUntagged/InPktsNoTag */
#define YUSUR2_LSECRXOCTD	0x08F44 /* InOctetsDecrypted */
#define YUSUR2_LSECRXOCTV	0x08F48 /* InOctetsValidated */
#define YUSUR2_LSECRXBAD		0x08F4C /* InPktsBadTag */
#define YUSUR2_LSECRXNOSCI	0x08F50 /* InPktsNoSci */
#define YUSUR2_LSECRXUNSCI	0x08F54 /* InPktsUnknownSci */
#define YUSUR2_LSECRXUNCH	0x08F58 /* InPktsUnchecked */
#define YUSUR2_LSECRXDELAY	0x08F5C /* InPktsDelayed */
#define YUSUR2_LSECRXLATE	0x08F60 /* InPktsLate */
#define YUSUR2_LSECRXOK(_n)	(0x08F64 + (0x04 * (_n))) /* InPktsOk */
#define YUSUR2_LSECRXINV(_n)	(0x08F6C + (0x04 * (_n))) /* InPktsInvalid */
#define YUSUR2_LSECRXNV(_n)	(0x08F74 + (0x04 * (_n))) /* InPktsNotValid */
#define YUSUR2_LSECRXUNSA	0x08F7C /* InPktsUnusedSa */
#define YUSUR2_LSECRXNUSA	0x08F80 /* InPktsNotUsingSa */

/* LinkSec (MacSec) Bit Fields and Masks */
#define YUSUR2_LSECTXCAP_SUM_MASK	0x00FF0000
#define YUSUR2_LSECTXCAP_SUM_SHIFT	16
#define YUSUR2_LSECRXCAP_SUM_MASK	0x00FF0000
#define YUSUR2_LSECRXCAP_SUM_SHIFT	16

#define YUSUR2_LSECTXCTRL_EN_MASK	0x00000003
#define YUSUR2_LSECTXCTRL_DISABLE	0x0
#define YUSUR2_LSECTXCTRL_AUTH		0x1
#define YUSUR2_LSECTXCTRL_AUTH_ENCRYPT	0x2
#define YUSUR2_LSECTXCTRL_AISCI		0x00000020
#define YUSUR2_LSECTXCTRL_PNTHRSH_MASK	0xFFFFFF00
#define YUSUR2_LSECTXCTRL_RSV_MASK	0x000000D8

#define YUSUR2_LSECRXCTRL_EN_MASK	0x0000000C
#define YUSUR2_LSECRXCTRL_EN_SHIFT	2
#define YUSUR2_LSECRXCTRL_DISABLE	0x0
#define YUSUR2_LSECRXCTRL_CHECK		0x1
#define YUSUR2_LSECRXCTRL_STRICT		0x2
#define YUSUR2_LSECRXCTRL_DROP		0x3
#define YUSUR2_LSECRXCTRL_PLSH		0x00000040
#define YUSUR2_LSECRXCTRL_RP		0x00000080
#define YUSUR2_LSECRXCTRL_RSV_MASK	0xFFFFFF33

/* IpSec Registers */
#define YUSUR2_IPSTXIDX		0x08900
#define YUSUR2_IPSTXSALT		0x08904
#define YUSUR2_IPSTXKEY(_i)	(0x08908 + (4 * (_i))) /* 4 of these (0-3) */
#define YUSUR2_IPSRXIDX		0x08E00
#define YUSUR2_IPSRXIPADDR(_i)	(0x08E04 + (4 * (_i))) /* 4 of these (0-3) */
#define YUSUR2_IPSRXSPI		0x08E14
#define YUSUR2_IPSRXIPIDX	0x08E18
#define YUSUR2_IPSRXKEY(_i)	(0x08E1C + (4 * (_i))) /* 4 of these (0-3) */
#define YUSUR2_IPSRXSALT		0x08E2C
#define YUSUR2_IPSRXMOD		0x08E30

#define YUSUR2_SECTXCTRL_STORE_FORWARD_ENABLE	0x4

/* DCB registers */
#define YUSUR2_RTRPCS		0x02430
#define YUSUR2_RTTDCS		0x04900
#define YUSUR2_RTTDCS_ARBDIS	0x00000040 /* DCB arbiter disable */
#define YUSUR2_RTTPCS		0x0CD00
#define YUSUR2_RTRUP2TC		0x03020
#define YUSUR2_RTTUP2TC		0x0C800
#define YUSUR2_RTRPT4C(_i)	(0x02140 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_TXLLQ(_i)		(0x082E0 + ((_i) * 4)) /* 4 of these (0-3) */
#define YUSUR2_RTRPT4S(_i)	(0x02160 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RTTDT2C(_i)	(0x04910 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RTTDT2S(_i)	(0x04930 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RTTPT2C(_i)	(0x0CD20 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RTTPT2S(_i)	(0x0CD40 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_RTTDQSEL		0x04904
#define YUSUR2_RTTDT1C		0x04908
#define YUSUR2_RTTDT1S		0x0490C
#define YUSUR2_RTTQCNCR		0x08B00
#define YUSUR2_RTTQCNTG		0x04A90
#define YUSUR2_RTTBCNRD		0x0498C
#define YUSUR2_RTTQCNRR		0x0498C
#define YUSUR2_RTTDTECC		0x04990
#define YUSUR2_RTTDTECC_NO_BCN	0x00000100

#define YUSUR2_RTTBCNRC			0x04984
#define YUSUR2_RTTBCNRC_RS_ENA		0x80000000
#define YUSUR2_RTTBCNRC_RF_DEC_MASK	0x00003FFF
#define YUSUR2_RTTBCNRC_RF_INT_SHIFT	14
#define YUSUR2_RTTBCNRC_RF_INT_MASK \
	(YUSUR2_RTTBCNRC_RF_DEC_MASK << YUSUR2_RTTBCNRC_RF_INT_SHIFT)
#define YUSUR2_RTTBCNRM	0x04980
#define YUSUR2_RTTQCNRM	0x04980

/* BCN (for DCB) Registers */
#define YUSUR2_RTTBCNRS	0x04988
#define YUSUR2_RTTBCNCR	0x08B00
#define YUSUR2_RTTBCNACH	0x08B04
#define YUSUR2_RTTBCNACL	0x08B08
#define YUSUR2_RTTBCNTG	0x04A90
#define YUSUR2_RTTBCNIDX	0x08B0C
#define YUSUR2_RTTBCNCP	0x08B10
#define YUSUR2_RTFRTIMER	0x08B14
#define YUSUR2_RTTBCNRTT	0x05150
#define YUSUR2_RTTBCNRD	0x0498C

/* FCoE DMA Context Registers */
/* FCoE Direct DMA Context */
#define YUSUR2_FCDDC(_i, _j)	(0x20000 + ((_i) * 0x4) + ((_j) * 0x10))
#define YUSUR2_FCPTRL		0x02410 /* FC User Desc. PTR Low */
#define YUSUR2_FCPTRH		0x02414 /* FC USer Desc. PTR High */
#define YUSUR2_FCBUFF		0x02418 /* FC Buffer Control */
#define YUSUR2_FCDMARW		0x02420 /* FC Receive DMA RW */
#define YUSUR2_FCBUFF_VALID	(1 << 0)   /* DMA Context Valid */
#define YUSUR2_FCBUFF_BUFFSIZE	(3 << 3)   /* User Buffer Size */
#define YUSUR2_FCBUFF_WRCONTX	(1 << 7)   /* 0: Initiator, 1: Target */
#define YUSUR2_FCBUFF_BUFFCNT	0x0000ff00 /* Number of User Buffers */
#define YUSUR2_FCBUFF_OFFSET	0xffff0000 /* User Buffer Offset */
#define YUSUR2_FCBUFF_BUFFSIZE_SHIFT	3
#define YUSUR2_FCBUFF_BUFFCNT_SHIFT	8
#define YUSUR2_FCBUFF_OFFSET_SHIFT	16
#define YUSUR2_FCDMARW_WE		(1 << 14)   /* Write enable */
#define YUSUR2_FCDMARW_RE		(1 << 15)   /* Read enable */
#define YUSUR2_FCDMARW_FCOESEL		0x000001ff  /* FC X_ID: 11 bits */
#define YUSUR2_FCDMARW_LASTSIZE		0xffff0000  /* Last User Buffer Size */
#define YUSUR2_FCDMARW_LASTSIZE_SHIFT	16
/* FCoE SOF/EOF */
#define YUSUR2_TEOFF		0x04A94 /* Tx FC EOF */
#define YUSUR2_TSOFF		0x04A98 /* Tx FC SOF */
#define YUSUR2_REOFF		0x05158 /* Rx FC EOF */
#define YUSUR2_RSOFF		0x051F8 /* Rx FC SOF */
/* FCoE Filter Context Registers */
#define YUSUR2_FCD_ID		0x05114 /* FCoE D_ID */
#define YUSUR2_FCSMAC		0x0510C /* FCoE Source MAC */
#define YUSUR2_FCFLTRW_SMAC_HIGH_SHIFT	16
/* FCoE Direct Filter Context */
#define YUSUR2_FCDFC(_i, _j)	(0x28000 + ((_i) * 0x4) + ((_j) * 0x10))
#define YUSUR2_FCDFCD(_i)	(0x30000 + ((_i) * 0x4))
#define YUSUR2_FCFLT		0x05108 /* FC FLT Context */
#define YUSUR2_FCFLTRW		0x05110 /* FC Filter RW Control */
#define YUSUR2_FCPARAM		0x051d8 /* FC Offset Parameter */
#define YUSUR2_FCFLT_VALID	(1 << 0)   /* Filter Context Valid */
#define YUSUR2_FCFLT_FIRST	(1 << 1)   /* Filter First */
#define YUSUR2_FCFLT_SEQID	0x00ff0000 /* Sequence ID */
#define YUSUR2_FCFLT_SEQCNT	0xff000000 /* Sequence Count */
#define YUSUR2_FCFLTRW_RVALDT	(1 << 13)  /* Fast Re-Validation */
#define YUSUR2_FCFLTRW_WE	(1 << 14)  /* Write Enable */
#define YUSUR2_FCFLTRW_RE	(1 << 15)  /* Read Enable */
/* FCoE Receive Control */
#define YUSUR2_FCRXCTRL		0x05100 /* FC Receive Control */
#define YUSUR2_FCRXCTRL_FCOELLI	(1 << 0)   /* Low latency interrupt */
#define YUSUR2_FCRXCTRL_SAVBAD	(1 << 1)   /* Save Bad Frames */
#define YUSUR2_FCRXCTRL_FRSTRDH	(1 << 2)   /* EN 1st Read Header */
#define YUSUR2_FCRXCTRL_LASTSEQH	(1 << 3)   /* EN Last Header in Seq */
#define YUSUR2_FCRXCTRL_ALLH	(1 << 4)   /* EN All Headers */
#define YUSUR2_FCRXCTRL_FRSTSEQH	(1 << 5)   /* EN 1st Seq. Header */
#define YUSUR2_FCRXCTRL_ICRC	(1 << 6)   /* Ignore Bad FC CRC */
#define YUSUR2_FCRXCTRL_FCCRCBO	(1 << 7)   /* FC CRC Byte Ordering */
#define YUSUR2_FCRXCTRL_FCOEVER	0x00000f00 /* FCoE Version: 4 bits */
#define YUSUR2_FCRXCTRL_FCOEVER_SHIFT	8
/* FCoE Redirection */
#define YUSUR2_FCRECTL		0x0ED00 /* FC Redirection Control */
#define YUSUR2_FCRETA0		0x0ED10 /* FC Redirection Table 0 */
#define YUSUR2_FCRETA(_i)	(YUSUR2_FCRETA0 + ((_i) * 4)) /* FCoE Redir */
#define YUSUR2_FCRECTL_ENA	0x1 /* FCoE Redir Table Enable */
#define YUSUR2_FCRETASEL_ENA	0x2 /* FCoE FCRETASEL bit */
#define YUSUR2_FCRETA_SIZE	8 /* Max entries in FCRETA */
#define YUSUR2_FCRETA_ENTRY_MASK	0x0000007f /* 7 bits for the queue index */
#define YUSUR2_FCRETA_SIZE_X550	32 /* Max entries in FCRETA */
/* Higher 7 bits for the queue index */
#define YUSUR2_FCRETA_ENTRY_HIGH_MASK	0x007F0000
#define YUSUR2_FCRETA_ENTRY_HIGH_SHIFT	16

/* Stats registers */
#define YUSUR2_CRCERRS	0x04000
#define YUSUR2_ILLERRC	0x04004
#define YUSUR2_ERRBC	0x04008
#define YUSUR2_MSPDC	0x04010
#define YUSUR2_MPC(_i)	(0x03FA0 + ((_i) * 4)) /* 8 of these 3FA0-3FBC*/
#define YUSUR2_MLFC	0x04034
#define YUSUR2_MRFC	0x04038
#define YUSUR2_RLEC	0x04040
#define YUSUR2_LXONTXC	0x03F60
#define YUSUR2_LXONRXC	0x0CF60
#define YUSUR2_LXOFFTXC	0x03F68
#define YUSUR2_LXOFFRXC	0x0CF68
#define YUSUR2_LXONRXCNT		0x041A4
#define YUSUR2_LXOFFRXCNT	0x041A8
#define YUSUR2_PXONRXCNT(_i)	(0x04140 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_PXOFFRXCNT(_i)	(0x04160 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_PXON2OFFCNT(_i)	(0x03240 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_PXONTXC(_i)	(0x03F00 + ((_i) * 4)) /* 8 of these 3F00-3F1C*/
#define YUSUR2_PXONRXC(_i)	(0x0CF00 + ((_i) * 4)) /* 8 of these CF00-CF1C*/
#define YUSUR2_PXOFFTXC(_i)	(0x03F20 + ((_i) * 4)) /* 8 of these 3F20-3F3C*/
#define YUSUR2_PXOFFRXC(_i)	(0x0CF20 + ((_i) * 4)) /* 8 of these CF20-CF3C*/
#define YUSUR2_PRC64		0x0405C
#define YUSUR2_PRC127		0x04060
#define YUSUR2_PRC255		0x04064
#define YUSUR2_PRC511		0x04068
#define YUSUR2_PRC1023		0x0406C
#define YUSUR2_PRC1522		0x04070
#define YUSUR2_GPRC		0x04074
#define YUSUR2_BPRC		0x04078
#define YUSUR2_MPRC		0x0407C
#define YUSUR2_GPTC		0x04080
#define YUSUR2_GORCL		0x04088
#define YUSUR2_GORCH		0x0408C
#define YUSUR2_GOTCL		0x04090
#define YUSUR2_GOTCH		0x04094
#define YUSUR2_RNBC(_i)		(0x03FC0 + ((_i) * 4)) /* 8 of these 3FC0-3FDC*/
#define YUSUR2_RUC		0x040A4
#define YUSUR2_RFC		0x040A8
#define YUSUR2_ROC		0x040AC
#define YUSUR2_RJC		0x040B0
#define YUSUR2_MNGPRC		0x040B4
#define YUSUR2_MNGPDC		0x040B8
#define YUSUR2_MNGPTC		0x0CF90
#define YUSUR2_TORL		0x040C0
#define YUSUR2_TORH		0x040C4
#define YUSUR2_TPR		0x040D0
#define YUSUR2_TPT		0x040D4
#define YUSUR2_PTC64		0x040D8
#define YUSUR2_PTC127		0x040DC
#define YUSUR2_PTC255		0x040E0
#define YUSUR2_PTC511		0x040E4
#define YUSUR2_PTC1023		0x040E8
#define YUSUR2_PTC1522		0x040EC
#define YUSUR2_MPTC		0x040F0
#define YUSUR2_BPTC		0x040F4
#define YUSUR2_XEC		0x04120
#define YUSUR2_SSVPC		0x08780

#define YUSUR2_RQSMR(_i)	(0x02300 + ((_i) * 4))
#define YUSUR2_TQSMR(_i)	(((_i) <= 7) ? (0x07300 + ((_i) * 4)) : \
			 (0x08600 + ((_i) * 4)))
#define YUSUR2_TQSM(_i)	(0x08600 + ((_i) * 4))

#define YUSUR2_QPRC(_i)	(0x01030 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QPTC(_i)	(0x06030 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QBRC(_i)	(0x01034 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QBTC(_i)	(0x06034 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QBRC_L(_i)	(0x01034 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QBRC_H(_i)	(0x01038 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QPRDC(_i)		(0x01430 + ((_i) * 0x40)) /* 16 of these */
#define YUSUR2_QBTC_L(_i)	(0x08700 + ((_i) * 0x8)) /* 16 of these */
#define YUSUR2_QBTC_H(_i)	(0x08704 + ((_i) * 0x8)) /* 16 of these */
#define YUSUR2_FCCRC		0x05118 /* Num of Good Eth CRC w/ Bad FC CRC */
#define YUSUR2_FCOERPDC		0x0241C /* FCoE Rx Packets Dropped Count */
#define YUSUR2_FCLAST		0x02424 /* FCoE Last Error Count */
#define YUSUR2_FCOEPRC		0x02428 /* Number of FCoE Packets Received */
#define YUSUR2_FCOEDWRC		0x0242C /* Number of FCoE DWords Received */
#define YUSUR2_FCOEPTC		0x08784 /* Number of FCoE Packets Transmitted */
#define YUSUR2_FCOEDWTC		0x08788 /* Number of FCoE DWords Transmitted */
#define YUSUR2_FCCRC_CNT_MASK	0x0000FFFF /* CRC_CNT: bit 0 - 15 */
#define YUSUR2_FCLAST_CNT_MASK	0x0000FFFF /* Last_CNT: bit 0 - 15 */
#define YUSUR2_O2BGPTC		0x041C4
#define YUSUR2_O2BSPC		0x087B0
#define YUSUR2_B2OSPC		0x041C0
#define YUSUR2_B2OGPRC		0x02F90
#define YUSUR2_BUPRC		0x04180
#define YUSUR2_BMPRC		0x04184
#define YUSUR2_BBPRC		0x04188
#define YUSUR2_BUPTC		0x0418C
#define YUSUR2_BMPTC		0x04190
#define YUSUR2_BBPTC		0x04194
#define YUSUR2_BCRCERRS		0x04198
#define YUSUR2_BXONRXC		0x0419C
#define YUSUR2_BXOFFRXC		0x041E0
#define YUSUR2_BXONTXC		0x041E4
#define YUSUR2_BXOFFTXC		0x041E8

/* Management */
#define YUSUR2_MAVTV(_i)		(0x05010 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_MFUTP(_i)		(0x05030 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_MANC		0x05820
#define YUSUR2_MFVAL		0x05824
#define YUSUR2_MANC2H		0x05860
#define YUSUR2_MDEF(_i)		(0x05890 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_MIPAF		0x058B0
#define YUSUR2_MMAL(_i)		(0x05910 + ((_i) * 8)) /* 4 of these (0-3) */
#define YUSUR2_MMAH(_i)		(0x05914 + ((_i) * 8)) /* 4 of these (0-3) */
#define YUSUR2_FTFT		0x09400 /* 0x9400-0x97FC */
#define YUSUR2_METF(_i)		(0x05190 + ((_i) * 4)) /* 4 of these (0-3) */
#define YUSUR2_MDEF_EXT(_i)	(0x05160 + ((_i) * 4)) /* 8 of these (0-7) */
#define YUSUR2_LSWFW		0x15F14
#define YUSUR2_BMCIP(_i)		(0x05050 + ((_i) * 4)) /* 0x5050-0x505C */
#define YUSUR2_BMCIPVAL		0x05060
#define YUSUR2_BMCIP_IPADDR_TYPE	0x00000001
#define YUSUR2_BMCIP_IPADDR_VALID	0x00000002

/* Management Bit Fields and Masks */
#define YUSUR2_MANC_MPROXYE	0x40000000 /* Management Proxy Enable */
#define YUSUR2_MANC_RCV_TCO_EN	0x00020000 /* Rcv TCO packet enable */
#define YUSUR2_MANC_EN_BMC2OS	0x10000000 /* Ena BMC2OS and OS2BMC traffic */
#define YUSUR2_MANC_EN_BMC2OS_SHIFT	28

/* Firmware Semaphore Register */
#define YUSUR2_FWSM_MODE_MASK	0xE
#define YUSUR2_FWSM_TS_ENABLED	0x1
#define YUSUR2_FWSM_FW_MODE_PT	0x4
#define YUSUR2_FWSM_FW_NVM_RECOVERY_MODE (1 << 5)
#define YUSUR2_FWSM_EXT_ERR_IND_MASK 0x01F80000
#define YUSUR2_FWSM_FW_VAL_BIT	(1 << 15)

/* ARC Subsystem registers */
#define YUSUR2_HICR		0x15F00
#define YUSUR2_FWSTS		0x15F0C
#define YUSUR2_HSMC0R		0x15F04
#define YUSUR2_HSMC1R		0x15F08
#define YUSUR2_SWSR		0x15F10
#define YUSUR2_HFDR		0x15FE8
#define YUSUR2_FLEX_MNG		0x15800 /* 0x15800 - 0x15EFC */

#define YUSUR2_HICR_EN		0x01  /* Enable bit - RO */
/* Driver sets this bit when done to put command in RAM */
#define YUSUR2_HICR_C		0x02
#define YUSUR2_HICR_SV		0x04  /* Status Validity */
#define YUSUR2_HICR_FW_RESET_ENABLE	0x40
#define YUSUR2_HICR_FW_RESET	0x80

/* PCI-E registers */
#define YUSUR2_GCR		0x11000
#define YUSUR2_GTV		0x11004
#define YUSUR2_FUNCTAG		0x11008
#define YUSUR2_GLT		0x1100C
#define YUSUR2_PCIEPIPEADR	0x11004
#define YUSUR2_PCIEPIPEDAT	0x11008
#define YUSUR2_GSCL_1		0x11010
#define YUSUR2_GSCL_2		0x11014
#define YUSUR2_GSCL_1_X540	YUSUR2_GSCL_1
#define YUSUR2_GSCL_2_X540	YUSUR2_GSCL_2
#define YUSUR2_GSCL_3		0x11018
#define YUSUR2_GSCL_4		0x1101C
#define YUSUR2_GSCN_0		0x11020
#define YUSUR2_GSCN_1		0x11024
#define YUSUR2_GSCN_2		0x11028
#define YUSUR2_GSCN_3		0x1102C
#define YUSUR2_GSCN_0_X540	YUSUR2_GSCN_0
#define YUSUR2_GSCN_1_X540	YUSUR2_GSCN_1
#define YUSUR2_GSCN_2_X540	YUSUR2_GSCN_2
#define YUSUR2_GSCN_3_X540	YUSUR2_GSCN_3
#define YUSUR2_FACTPS		0x10150
#define YUSUR2_FACTPS_X540	YUSUR2_FACTPS
#define YUSUR2_GSCL_1_X550	0x11800
#define YUSUR2_GSCL_2_X550	0x11804
#define YUSUR2_GSCL_1_X550EM_x	YUSUR2_GSCL_1_X550
#define YUSUR2_GSCL_2_X550EM_x	YUSUR2_GSCL_2_X550
#define YUSUR2_GSCN_0_X550	0x11820
#define YUSUR2_GSCN_1_X550	0x11824
#define YUSUR2_GSCN_2_X550	0x11828
#define YUSUR2_GSCN_3_X550	0x1182C
#define YUSUR2_GSCN_0_X550EM_x	YUSUR2_GSCN_0_X550
#define YUSUR2_GSCN_1_X550EM_x	YUSUR2_GSCN_1_X550
#define YUSUR2_GSCN_2_X550EM_x	YUSUR2_GSCN_2_X550
#define YUSUR2_GSCN_3_X550EM_x	YUSUR2_GSCN_3_X550
#define YUSUR2_FACTPS_X550	YUSUR2_FACTPS
#define YUSUR2_FACTPS_X550EM_x	YUSUR2_FACTPS
#define YUSUR2_GSCL_1_X550EM_a	YUSUR2_GSCL_1_X550
#define YUSUR2_GSCL_2_X550EM_a	YUSUR2_GSCL_2_X550
#define YUSUR2_GSCN_0_X550EM_a	YUSUR2_GSCN_0_X550
#define YUSUR2_GSCN_1_X550EM_a	YUSUR2_GSCN_1_X550
#define YUSUR2_GSCN_2_X550EM_a	YUSUR2_GSCN_2_X550
#define YUSUR2_GSCN_3_X550EM_a	YUSUR2_GSCN_3_X550
#define YUSUR2_FACTPS_X550EM_a	0x15FEC
#define YUSUR2_FACTPS_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), FACTPS)

#define YUSUR2_PCIEANACTL	0x11040
#define YUSUR2_SWSM		0x10140
#define YUSUR2_SWSM_X540		YUSUR2_SWSM
#define YUSUR2_SWSM_X550		YUSUR2_SWSM
#define YUSUR2_SWSM_X550EM_x	YUSUR2_SWSM
#define YUSUR2_SWSM_X550EM_a	0x15F70
#define YUSUR2_SWSM_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SWSM)

#define YUSUR2_FWSM		0x10148
#define YUSUR2_FWSM_X540		YUSUR2_FWSM
#define YUSUR2_FWSM_X550		YUSUR2_FWSM
#define YUSUR2_FWSM_X550EM_x	YUSUR2_FWSM
#define YUSUR2_FWSM_X550EM_a	0x15F74
#define YUSUR2_FWSM_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), FWSM)

#define YUSUR2_SWFW_SYNC		YUSUR2_GSSR
#define YUSUR2_SWFW_SYNC_X540	YUSUR2_SWFW_SYNC
#define YUSUR2_SWFW_SYNC_X550	YUSUR2_SWFW_SYNC
#define YUSUR2_SWFW_SYNC_X550EM_x	YUSUR2_SWFW_SYNC
#define YUSUR2_SWFW_SYNC_X550EM_a	0x15F78
#define YUSUR2_SWFW_SYNC_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SWFW_SYNC)

#define YUSUR2_GSSR		0x10160
#define YUSUR2_MREVID		0x11064
#define YUSUR2_DCA_ID		0x11070
#define YUSUR2_DCA_CTRL		0x11074

/* PCI-E registers 82599-Specific */
#define YUSUR2_GCR_EXT		0x11050
#define YUSUR2_GSCL_5_82599	0x11030
#define YUSUR2_GSCL_6_82599	0x11034
#define YUSUR2_GSCL_7_82599	0x11038
#define YUSUR2_GSCL_8_82599	0x1103C
#define YUSUR2_GSCL_5_X540	YUSUR2_GSCL_5_82599
#define YUSUR2_GSCL_6_X540	YUSUR2_GSCL_6_82599
#define YUSUR2_GSCL_7_X540	YUSUR2_GSCL_7_82599
#define YUSUR2_GSCL_8_X540	YUSUR2_GSCL_8_82599
#define YUSUR2_PHYADR_82599	0x11040
#define YUSUR2_PHYDAT_82599	0x11044
#define YUSUR2_PHYCTL_82599	0x11048
#define YUSUR2_PBACLR_82599	0x11068
#define YUSUR2_CIAA		0x11088
#define YUSUR2_CIAD		0x1108C
#define YUSUR2_CIAA_82599	YUSUR2_CIAA
#define YUSUR2_CIAD_82599	YUSUR2_CIAD
#define YUSUR2_CIAA_X540		YUSUR2_CIAA
#define YUSUR2_CIAD_X540		YUSUR2_CIAD
#define YUSUR2_GSCL_5_X550	0x11810
#define YUSUR2_GSCL_6_X550	0x11814
#define YUSUR2_GSCL_7_X550	0x11818
#define YUSUR2_GSCL_8_X550	0x1181C
#define YUSUR2_GSCL_5_X550EM_x	YUSUR2_GSCL_5_X550
#define YUSUR2_GSCL_6_X550EM_x	YUSUR2_GSCL_6_X550
#define YUSUR2_GSCL_7_X550EM_x	YUSUR2_GSCL_7_X550
#define YUSUR2_GSCL_8_X550EM_x	YUSUR2_GSCL_8_X550
#define YUSUR2_CIAA_X550		0x11508
#define YUSUR2_CIAD_X550		0x11510
#define YUSUR2_CIAA_X550EM_x	YUSUR2_CIAA_X550
#define YUSUR2_CIAD_X550EM_x	YUSUR2_CIAD_X550
#define YUSUR2_GSCL_5_X550EM_a	YUSUR2_GSCL_5_X550
#define YUSUR2_GSCL_6_X550EM_a	YUSUR2_GSCL_6_X550
#define YUSUR2_GSCL_7_X550EM_a	YUSUR2_GSCL_7_X550
#define YUSUR2_GSCL_8_X550EM_a	YUSUR2_GSCL_8_X550
#define YUSUR2_CIAA_X550EM_a	YUSUR2_CIAA_X550
#define YUSUR2_CIAD_X550EM_a	YUSUR2_CIAD_X550
#define YUSUR2_CIAA_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), CIAA)
#define YUSUR2_CIAD_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), CIAD)
#define YUSUR2_PICAUSE		0x110B0
#define YUSUR2_PIENA		0x110B8
#define YUSUR2_CDQ_MBR_82599	0x110B4
#define YUSUR2_PCIESPARE		0x110BC
#define YUSUR2_MISC_REG_82599	0x110F0
#define YUSUR2_ECC_CTRL_0_82599	0x11100
#define YUSUR2_ECC_CTRL_1_82599	0x11104
#define YUSUR2_ECC_STATUS_82599	0x110E0
#define YUSUR2_BAR_CTRL_82599	0x110F4

/* PCI Express Control */
#define YUSUR2_GCR_CMPL_TMOUT_MASK	0x0000F000
#define YUSUR2_GCR_CMPL_TMOUT_10ms	0x00001000
#define YUSUR2_GCR_CMPL_TMOUT_RESEND	0x00010000
#define YUSUR2_GCR_CAP_VER2		0x00040000

#define YUSUR2_GCR_EXT_MSIX_EN		0x80000000
#define YUSUR2_GCR_EXT_BUFFERS_CLEAR	0x40000000
#define YUSUR2_GCR_EXT_VT_MODE_16	0x00000001
#define YUSUR2_GCR_EXT_VT_MODE_32	0x00000002
#define YUSUR2_GCR_EXT_VT_MODE_64	0x00000003
#define YUSUR2_GCR_EXT_SRIOV		(YUSUR2_GCR_EXT_MSIX_EN | \
					 YUSUR2_GCR_EXT_VT_MODE_64)
#define YUSUR2_GCR_EXT_VT_MODE_MASK	0x00000003
/* Time Sync Registers */
#define YUSUR2_TSYNCRXCTL	0x05188 /* Rx Time Sync Control register - RW */
#define YUSUR2_TSYNCTXCTL	0x08C00 /* Tx Time Sync Control register - RW */
#define YUSUR2_RXSTMPL	0x051E8 /* Rx timestamp Low - RO */
#define YUSUR2_RXSTMPH	0x051A4 /* Rx timestamp High - RO */
#define YUSUR2_RXSATRL	0x051A0 /* Rx timestamp attribute low - RO */
#define YUSUR2_RXSATRH	0x051A8 /* Rx timestamp attribute high - RO */
#define YUSUR2_RXMTRL	0x05120 /* RX message type register low - RW */
#define YUSUR2_TXSTMPL	0x08C04 /* Tx timestamp value Low - RO */
#define YUSUR2_TXSTMPH	0x08C08 /* Tx timestamp value High - RO */
#define YUSUR2_SYSTIML	0x08C0C /* System time register Low - RO */
#define YUSUR2_SYSTIMH	0x08C10 /* System time register High - RO */
#define YUSUR2_SYSTIMR	0x08C58 /* System time register Residue - RO */
#define YUSUR2_TIMINCA	0x08C14 /* Increment attributes register - RW */
#define YUSUR2_TIMADJL	0x08C18 /* Time Adjustment Offset register Low - RW */
#define YUSUR2_TIMADJH	0x08C1C /* Time Adjustment Offset register High - RW */
#define YUSUR2_TSAUXC	0x08C20 /* TimeSync Auxiliary Control register - RW */
#define YUSUR2_TRGTTIML0	0x08C24 /* Target Time Register 0 Low - RW */
#define YUSUR2_TRGTTIMH0	0x08C28 /* Target Time Register 0 High - RW */
#define YUSUR2_TRGTTIML1	0x08C2C /* Target Time Register 1 Low - RW */
#define YUSUR2_TRGTTIMH1	0x08C30 /* Target Time Register 1 High - RW */
#define YUSUR2_CLKTIML	0x08C34 /* Clock Out Time Register Low - RW */
#define YUSUR2_CLKTIMH	0x08C38 /* Clock Out Time Register High - RW */
#define YUSUR2_FREQOUT0	0x08C34 /* Frequency Out 0 Control register - RW */
#define YUSUR2_FREQOUT1	0x08C38 /* Frequency Out 1 Control register - RW */
#define YUSUR2_AUXSTMPL0	0x08C3C /* Auxiliary Time Stamp 0 register Low - RO */
#define YUSUR2_AUXSTMPH0	0x08C40 /* Auxiliary Time Stamp 0 register High - RO */
#define YUSUR2_AUXSTMPL1	0x08C44 /* Auxiliary Time Stamp 1 register Low - RO */
#define YUSUR2_AUXSTMPH1	0x08C48 /* Auxiliary Time Stamp 1 register High - RO */
#define YUSUR2_TSIM	0x08C68 /* TimeSync Interrupt Mask Register - RW */
#define YUSUR2_TSICR	0x08C60 /* TimeSync Interrupt Cause Register - WO */
#define YUSUR2_TSSDP	0x0003C /* TimeSync SDP Configuration Register - RW */

/* Diagnostic Registers */
#define YUSUR2_RDSTATCTL		0x02C20
#define YUSUR2_RDSTAT(_i)	(0x02C00 + ((_i) * 4)) /* 0x02C00-0x02C1C */
#define YUSUR2_RDHMPN		0x02F08
#define YUSUR2_RIC_DW(_i)	(0x02F10 + ((_i) * 4))
#define YUSUR2_RDPROBE		0x02F20
#define YUSUR2_RDMAM		0x02F30
#define YUSUR2_RDMAD		0x02F34
#define YUSUR2_TDHMPN		0x07F08
#define YUSUR2_TDHMPN2		0x082FC
#define YUSUR2_TXDESCIC		0x082CC
#define YUSUR2_TIC_DW(_i)	(0x07F10 + ((_i) * 4))
#define YUSUR2_TIC_DW2(_i)	(0x082B0 + ((_i) * 4))
#define YUSUR2_TDPROBE		0x07F20
#define YUSUR2_TXBUFCTRL		0x0C600
#define YUSUR2_TXBUFDATA0	0x0C610
#define YUSUR2_TXBUFDATA1	0x0C614
#define YUSUR2_TXBUFDATA2	0x0C618
#define YUSUR2_TXBUFDATA3	0x0C61C
#define YUSUR2_RXBUFCTRL		0x03600
#define YUSUR2_RXBUFDATA0	0x03610
#define YUSUR2_RXBUFDATA1	0x03614
#define YUSUR2_RXBUFDATA2	0x03618
#define YUSUR2_RXBUFDATA3	0x0361C
#define YUSUR2_PCIE_DIAG(_i)	(0x11090 + ((_i) * 4)) /* 8 of these */
#define YUSUR2_RFVAL		0x050A4
#define YUSUR2_MDFTC1		0x042B8
#define YUSUR2_MDFTC2		0x042C0
#define YUSUR2_MDFTFIFO1		0x042C4
#define YUSUR2_MDFTFIFO2		0x042C8
#define YUSUR2_MDFTS		0x042CC
#define YUSUR2_RXDATAWRPTR(_i)	(0x03700 + ((_i) * 4)) /* 8 of these 3700-370C*/
#define YUSUR2_RXDESCWRPTR(_i)	(0x03710 + ((_i) * 4)) /* 8 of these 3710-371C*/
#define YUSUR2_RXDATARDPTR(_i)	(0x03720 + ((_i) * 4)) /* 8 of these 3720-372C*/
#define YUSUR2_RXDESCRDPTR(_i)	(0x03730 + ((_i) * 4)) /* 8 of these 3730-373C*/
#define YUSUR2_TXDATAWRPTR(_i)	(0x0C700 + ((_i) * 4)) /* 8 of these C700-C70C*/
#define YUSUR2_TXDESCWRPTR(_i)	(0x0C710 + ((_i) * 4)) /* 8 of these C710-C71C*/
#define YUSUR2_TXDATARDPTR(_i)	(0x0C720 + ((_i) * 4)) /* 8 of these C720-C72C*/
#define YUSUR2_TXDESCRDPTR(_i)	(0x0C730 + ((_i) * 4)) /* 8 of these C730-C73C*/
#define YUSUR2_PCIEECCCTL	0x1106C
#define YUSUR2_RXWRPTR(_i)	(0x03100 + ((_i) * 4)) /* 8 of these 3100-310C*/
#define YUSUR2_RXUSED(_i)	(0x03120 + ((_i) * 4)) /* 8 of these 3120-312C*/
#define YUSUR2_RXRDPTR(_i)	(0x03140 + ((_i) * 4)) /* 8 of these 3140-314C*/
#define YUSUR2_RXRDWRPTR(_i)	(0x03160 + ((_i) * 4)) /* 8 of these 3160-310C*/
#define YUSUR2_TXWRPTR(_i)	(0x0C100 + ((_i) * 4)) /* 8 of these C100-C10C*/
#define YUSUR2_TXUSED(_i)	(0x0C120 + ((_i) * 4)) /* 8 of these C120-C12C*/
#define YUSUR2_TXRDPTR(_i)	(0x0C140 + ((_i) * 4)) /* 8 of these C140-C14C*/
#define YUSUR2_TXRDWRPTR(_i)	(0x0C160 + ((_i) * 4)) /* 8 of these C160-C10C*/
#define YUSUR2_PCIEECCCTL0	0x11100
#define YUSUR2_PCIEECCCTL1	0x11104
#define YUSUR2_RXDBUECC		0x03F70
#define YUSUR2_TXDBUECC		0x0CF70
#define YUSUR2_RXDBUEST		0x03F74
#define YUSUR2_TXDBUEST		0x0CF74
#define YUSUR2_PBTXECC		0x0C300
#define YUSUR2_PBRXECC		0x03300
#define YUSUR2_GHECCR		0x110B0

/* MAC Registers */
#define YUSUR2_PCS1GCFIG		0x04200
#define YUSUR2_PCS1GLCTL		0x04208
#define YUSUR2_PCS1GLSTA		0x0420C
#define YUSUR2_PCS1GDBG0		0x04210
#define YUSUR2_PCS1GDBG1		0x04214
#define YUSUR2_PCS1GANA		0x04218
#define YUSUR2_PCS1GANLP		0x0421C
#define YUSUR2_PCS1GANNP		0x04220
#define YUSUR2_PCS1GANLPNP	0x04224
#define YUSUR2_HLREG0		0x04240
#define YUSUR2_HLREG1		0x04244
#define YUSUR2_PAP		0x04248
#define YUSUR2_MACA		0x0424C
#define YUSUR2_APAE		0x04250
#define YUSUR2_ARD		0x04254
#define YUSUR2_AIS		0x04258
#define YUSUR2_MSCA		0x0425C
#define YUSUR2_MSRWD		0x04260
#define YUSUR2_MLADD		0x04264
#define YUSUR2_MHADD		0x04268
#define YUSUR2_MAXFRS		0x04268
#define YUSUR2_TREG		0x0426C
#define YUSUR2_PCSS1		0x04288
#define YUSUR2_PCSS2		0x0428C
#define YUSUR2_XPCSS		0x04290
#define YUSUR2_MFLCN		0x04294
#define YUSUR2_SERDESC		0x04298
#define YUSUR2_MAC_SGMII_BUSY	0x04298
#define YUSUR2_MACS		0x0429C
#define YUSUR2_AUTOC		0x042A0
#define YUSUR2_LINKS		0x042A4
#define YUSUR2_LINKS2		0x04324
#define YUSUR2_AUTOC2		0x042A8
#define YUSUR2_AUTOC3		0x042AC
#define YUSUR2_ANLP1		0x042B0
#define YUSUR2_ANLP2		0x042B4
#define YUSUR2_MACC		0x04330
#define YUSUR2_ATLASCTL		0x04800
#define YUSUR2_MMNGC		0x042D0
#define YUSUR2_ANLPNP1		0x042D4
#define YUSUR2_ANLPNP2		0x042D8
#define YUSUR2_KRPCSFC		0x042E0
#define YUSUR2_KRPCSS		0x042E4
#define YUSUR2_FECS1		0x042E8
#define YUSUR2_FECS2		0x042EC
#define YUSUR2_SMADARCTL		0x14F10
#define YUSUR2_MPVC		0x04318
#define YUSUR2_SGMIIC		0x04314

/* Statistics Registers */
#define YUSUR2_RXNFGPC		0x041B0
#define YUSUR2_RXNFGBCL		0x041B4
#define YUSUR2_RXNFGBCH		0x041B8
#define YUSUR2_RXDGPC		0x02F50
#define YUSUR2_RXDGBCL		0x02F54
#define YUSUR2_RXDGBCH		0x02F58
#define YUSUR2_RXDDGPC		0x02F5C
#define YUSUR2_RXDDGBCL		0x02F60
#define YUSUR2_RXDDGBCH		0x02F64
#define YUSUR2_RXLPBKGPC		0x02F68
#define YUSUR2_RXLPBKGBCL	0x02F6C
#define YUSUR2_RXLPBKGBCH	0x02F70
#define YUSUR2_RXDLPBKGPC	0x02F74
#define YUSUR2_RXDLPBKGBCL	0x02F78
#define YUSUR2_RXDLPBKGBCH	0x02F7C
#define YUSUR2_TXDGPC		0x087A0
#define YUSUR2_TXDGBCL		0x087A4
#define YUSUR2_TXDGBCH		0x087A8

#define YUSUR2_RXDSTATCTRL	0x02F40

/* Copper Pond 2 link timeout */
#define YUSUR2_VALIDATE_LINK_READY_TIMEOUT 50

/* Omer CORECTL */
#define YUSUR2_CORECTL			0x014F00
/* BARCTRL */
#define YUSUR2_BARCTRL			0x110F4
#define YUSUR2_BARCTRL_FLSIZE		0x0700
#define YUSUR2_BARCTRL_FLSIZE_SHIFT	8
#define YUSUR2_BARCTRL_CSRSIZE		0x2000

/* RSCCTL Bit Masks */
#define YUSUR2_RSCCTL_RSCEN	0x01
#define YUSUR2_RSCCTL_MAXDESC_1	0x00
#define YUSUR2_RSCCTL_MAXDESC_4	0x04
#define YUSUR2_RSCCTL_MAXDESC_8	0x08
#define YUSUR2_RSCCTL_MAXDESC_16	0x0C
#define YUSUR2_RSCCTL_TS_DIS	0x02

/* RSCDBU Bit Masks */
#define YUSUR2_RSCDBU_RSCSMALDIS_MASK	0x0000007F
#define YUSUR2_RSCDBU_RSCACKDIS		0x00000080

/* RDRXCTL Bit Masks */
#define YUSUR2_RDRXCTL_RDMTS_1_2		0x00000000 /* Rx Desc Min THLD Size */
#define YUSUR2_RDRXCTL_CRCSTRIP		0x00000002 /* CRC Strip */
#define YUSUR2_RDRXCTL_PSP		0x00000004 /* Pad Small Packet */
#define YUSUR2_RDRXCTL_MVMEN		0x00000020
#define YUSUR2_RDRXCTL_RSC_PUSH_DIS	0x00000020
#define YUSUR2_RDRXCTL_DMAIDONE		0x00000008 /* DMA init cycle done */
#define YUSUR2_RDRXCTL_RSC_PUSH		0x00000080
#define YUSUR2_RDRXCTL_AGGDIS		0x00010000 /* Aggregation disable */
#define YUSUR2_RDRXCTL_RSCFRSTSIZE	0x003E0000 /* RSC First packet size */
#define YUSUR2_RDRXCTL_RSCLLIDIS		0x00800000 /* Disable RSC compl on LLI*/
#define YUSUR2_RDRXCTL_RSCACKC		0x02000000 /* must set 1 when RSC ena */
#define YUSUR2_RDRXCTL_FCOE_WRFIX	0x04000000 /* must set 1 when RSC ena */
#define YUSUR2_RDRXCTL_MBINTEN		0x10000000
#define YUSUR2_RDRXCTL_MDP_EN		0x20000000

/* RQTC Bit Masks and Shifts */
#define YUSUR2_RQTC_SHIFT_TC(_i)	((_i) * 4)
#define YUSUR2_RQTC_TC0_MASK	(0x7 << 0)
#define YUSUR2_RQTC_TC1_MASK	(0x7 << 4)
#define YUSUR2_RQTC_TC2_MASK	(0x7 << 8)
#define YUSUR2_RQTC_TC3_MASK	(0x7 << 12)
#define YUSUR2_RQTC_TC4_MASK	(0x7 << 16)
#define YUSUR2_RQTC_TC5_MASK	(0x7 << 20)
#define YUSUR2_RQTC_TC6_MASK	(0x7 << 24)
#define YUSUR2_RQTC_TC7_MASK	(0x7 << 28)

/* PSRTYPE.RQPL Bit masks and shift */
#define YUSUR2_PSRTYPE_RQPL_MASK		0x7
#define YUSUR2_PSRTYPE_RQPL_SHIFT	29

/* CTRL Bit Masks */
#define YUSUR2_CTRL_GIO_DIS	0x00000004 /* Global IO Master Disable bit */
#define YUSUR2_CTRL_LNK_RST	0x00000008 /* Link Reset. Resets everything. */
#define YUSUR2_CTRL_RST		0x04000000 /* Reset (SW) */
#define YUSUR2_CTRL_RST_MASK	(YUSUR2_CTRL_LNK_RST | YUSUR2_CTRL_RST)

/* FACTPS */
#define YUSUR2_FACTPS_MNGCG	0x20000000 /* Manageblility Clock Gated */
#define YUSUR2_FACTPS_LFS	0x40000000 /* LAN Function Select */

/* MHADD Bit Masks */
#define YUSUR2_MHADD_MFS_MASK	0xFFFF0000
#define YUSUR2_MHADD_MFS_SHIFT	16

/* Extended Device Control */
#define YUSUR2_CTRL_EXT_PFRSTD	0x00004000 /* Physical Function Reset Done */
#define YUSUR2_CTRL_EXT_NS_DIS	0x00010000 /* No Snoop disable */
#define YUSUR2_CTRL_EXT_RO_DIS	0x00020000 /* Relaxed Ordering disable */
#define YUSUR2_CTRL_EXT_DRV_LOAD	0x10000000 /* Driver loaded bit for FW */

/* Direct Cache Access (DCA) definitions */
#define YUSUR2_DCA_CTRL_DCA_ENABLE	0x00000000 /* DCA Enable */
#define YUSUR2_DCA_CTRL_DCA_DISABLE	0x00000001 /* DCA Disable */

#define YUSUR2_DCA_CTRL_DCA_MODE_CB1	0x00 /* DCA Mode CB1 */
#define YUSUR2_DCA_CTRL_DCA_MODE_CB2	0x02 /* DCA Mode CB2 */

#define YUSUR2_DCA_RXCTRL_CPUID_MASK	0x0000001F /* Rx CPUID Mask */
#define YUSUR2_DCA_RXCTRL_CPUID_MASK_82599	0xFF000000 /* Rx CPUID Mask */
#define YUSUR2_DCA_RXCTRL_CPUID_SHIFT_82599	24 /* Rx CPUID Shift */
#define YUSUR2_DCA_RXCTRL_DESC_DCA_EN	(1 << 5) /* Rx Desc enable */
#define YUSUR2_DCA_RXCTRL_HEAD_DCA_EN	(1 << 6) /* Rx Desc header ena */
#define YUSUR2_DCA_RXCTRL_DATA_DCA_EN	(1 << 7) /* Rx Desc payload ena */
#define YUSUR2_DCA_RXCTRL_DESC_RRO_EN	(1 << 9) /* Rx rd Desc Relax Order */
#define YUSUR2_DCA_RXCTRL_DATA_WRO_EN	(1 << 13) /* Rx wr data Relax Order */
#define YUSUR2_DCA_RXCTRL_HEAD_WRO_EN	(1 << 15) /* Rx wr header RO */

#define YUSUR2_DCA_TXCTRL_CPUID_MASK	0x0000001F /* Tx CPUID Mask */
#define YUSUR2_DCA_TXCTRL_CPUID_MASK_82599	0xFF000000 /* Tx CPUID Mask */
#define YUSUR2_DCA_TXCTRL_CPUID_SHIFT_82599	24 /* Tx CPUID Shift */
#define YUSUR2_DCA_TXCTRL_DESC_DCA_EN	(1 << 5) /* DCA Tx Desc enable */
#define YUSUR2_DCA_TXCTRL_DESC_RRO_EN	(1 << 9) /* Tx rd Desc Relax Order */
#define YUSUR2_DCA_TXCTRL_DESC_WRO_EN	(1 << 11) /* Tx Desc writeback RO bit */
#define YUSUR2_DCA_TXCTRL_DATA_RRO_EN	(1 << 13) /* Tx rd data Relax Order */
#define YUSUR2_DCA_MAX_QUEUES_82598	16 /* DCA regs only on 16 queues */

/* MSCA Bit Masks */
#define YUSUR2_MSCA_NP_ADDR_MASK		0x0000FFFF /* MDI Addr (new prot) */
#define YUSUR2_MSCA_NP_ADDR_SHIFT	0
#define YUSUR2_MSCA_DEV_TYPE_MASK	0x001F0000 /* Dev Type (new prot) */
#define YUSUR2_MSCA_DEV_TYPE_SHIFT	16 /* Register Address (old prot */
#define YUSUR2_MSCA_PHY_ADDR_MASK	0x03E00000 /* PHY Address mask */
#define YUSUR2_MSCA_PHY_ADDR_SHIFT	21 /* PHY Address shift*/
#define YUSUR2_MSCA_OP_CODE_MASK		0x0C000000 /* OP CODE mask */
#define YUSUR2_MSCA_OP_CODE_SHIFT	26 /* OP CODE shift */
#define YUSUR2_MSCA_ADDR_CYCLE		0x00000000 /* OP CODE 00 (addr cycle) */
#define YUSUR2_MSCA_WRITE		0x04000000 /* OP CODE 01 (wr) */
#define YUSUR2_MSCA_READ			0x0C000000 /* OP CODE 11 (rd) */
#define YUSUR2_MSCA_READ_AUTOINC		0x08000000 /* OP CODE 10 (rd auto inc)*/
#define YUSUR2_MSCA_ST_CODE_MASK		0x30000000 /* ST Code mask */
#define YUSUR2_MSCA_ST_CODE_SHIFT	28 /* ST Code shift */
#define YUSUR2_MSCA_NEW_PROTOCOL		0x00000000 /* ST CODE 00 (new prot) */
#define YUSUR2_MSCA_OLD_PROTOCOL		0x10000000 /* ST CODE 01 (old prot) */
#define YUSUR2_MSCA_MDI_COMMAND		0x40000000 /* Initiate MDI command */
#define YUSUR2_MSCA_MDI_IN_PROG_EN	0x80000000 /* MDI in progress ena */

/* MSRWD bit masks */
#define YUSUR2_MSRWD_WRITE_DATA_MASK	0x0000FFFF
#define YUSUR2_MSRWD_WRITE_DATA_SHIFT	0
#define YUSUR2_MSRWD_READ_DATA_MASK	0xFFFF0000
#define YUSUR2_MSRWD_READ_DATA_SHIFT	16

/* Atlas registers */
#define YUSUR2_ATLAS_PDN_LPBK		0x24
#define YUSUR2_ATLAS_PDN_10G		0xB
#define YUSUR2_ATLAS_PDN_1G		0xC
#define YUSUR2_ATLAS_PDN_AN		0xD

/* Atlas bit masks */
#define YUSUR2_ATLASCTL_WRITE_CMD	0x00010000
#define YUSUR2_ATLAS_PDN_TX_REG_EN	0x10
#define YUSUR2_ATLAS_PDN_TX_10G_QL_ALL	0xF0
#define YUSUR2_ATLAS_PDN_TX_1G_QL_ALL	0xF0
#define YUSUR2_ATLAS_PDN_TX_AN_QL_ALL	0xF0

/* Omer bit masks */
#define YUSUR2_CORECTL_WRITE_CMD		0x00010000

/* Device Type definitions for new protocol MDIO commands */
#define YUSUR2_MDIO_ZERO_DEV_TYPE		0x0
#define YUSUR2_MDIO_PMA_PMD_DEV_TYPE		0x1
#define YUSUR2_MDIO_PCS_DEV_TYPE			0x3
#define YUSUR2_MDIO_PHY_XS_DEV_TYPE		0x4
#define YUSUR2_MDIO_AUTO_NEG_DEV_TYPE		0x7
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_DEV_TYPE	0x1E   /* Device 30 */
#define YUSUR2_TWINAX_DEV			1

#define YUSUR2_MDIO_COMMAND_TIMEOUT	100 /* PHY Timeout for 1 GB mode */

#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_CONTROL		0x0 /* VS1 Ctrl Reg */
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_STATUS		0x1 /* VS1 Status Reg */
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_LINK_STATUS	0x0008 /* 1 = Link Up */
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_SPEED_STATUS	0x0010 /* 0-10G, 1-1G */
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_10G_SPEED		0x0018
#define YUSUR2_MDIO_VENDOR_SPECIFIC_1_1G_SPEED		0x0010

#define YUSUR2_MDIO_AUTO_NEG_CONTROL	0x0 /* AUTO_NEG Control Reg */
#define YUSUR2_MDIO_AUTO_NEG_STATUS	0x1 /* AUTO_NEG Status Reg */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STAT	0xC800 /* AUTO_NEG Vendor Status Reg */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_TX_ALARM 0xCC00 /* AUTO_NEG Vendor TX Reg */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_TX_ALARM2 0xCC01 /* AUTO_NEG Vendor Tx Reg */
#define YUSUR2_MDIO_AUTO_NEG_VEN_LSC	0x1 /* AUTO_NEG Vendor Tx LSC */
#define YUSUR2_MDIO_AUTO_NEG_ADVT	0x10 /* AUTO_NEG Advt Reg */
#define YUSUR2_MDIO_AUTO_NEG_LP		0x13 /* AUTO_NEG LP Status Reg */
#define YUSUR2_MDIO_AUTO_NEG_EEE_ADVT	0x3C /* AUTO_NEG EEE Advt Reg */
#define YUSUR2_AUTO_NEG_10GBASE_EEE_ADVT	0x8  /* AUTO NEG EEE 10GBaseT Advt */
#define YUSUR2_AUTO_NEG_1000BASE_EEE_ADVT 0x4  /* AUTO NEG EEE 1000BaseT Advt */
#define YUSUR2_AUTO_NEG_100BASE_EEE_ADVT	0x2  /* AUTO NEG EEE 100BaseT Advt */
#define YUSUR2_MDIO_PHY_XS_CONTROL	0x0 /* PHY_XS Control Reg */
#define YUSUR2_MDIO_PHY_XS_RESET		0x8000 /* PHY_XS Reset */
#define YUSUR2_MDIO_PHY_ID_HIGH		0x2 /* PHY ID High Reg*/
#define YUSUR2_MDIO_PHY_ID_LOW		0x3 /* PHY ID Low Reg*/
#define YUSUR2_MDIO_PHY_SPEED_ABILITY	0x4 /* Speed Ability Reg */
#define YUSUR2_MDIO_PHY_SPEED_10G	0x0001 /* 10G capable */
#define YUSUR2_MDIO_PHY_SPEED_1G		0x0010 /* 1G capable */
#define YUSUR2_MDIO_PHY_SPEED_100M	0x0020 /* 100M capable */
#define YUSUR2_MDIO_PHY_EXT_ABILITY	0xB /* Ext Ability Reg */
#define YUSUR2_MDIO_PHY_10GBASET_ABILITY		0x0004 /* 10GBaseT capable */
#define YUSUR2_MDIO_PHY_1000BASET_ABILITY	0x0020 /* 1000BaseT capable */
#define YUSUR2_MDIO_PHY_100BASETX_ABILITY	0x0080 /* 100BaseTX capable */
#define YUSUR2_MDIO_PHY_SET_LOW_POWER_MODE	0x0800 /* Set low power mode */
#define YUSUR2_AUTO_NEG_LP_STATUS	0xE820 /* AUTO NEG Rx LP Status Reg */
#define YUSUR2_AUTO_NEG_LP_1000BASE_CAP	0x8000 /* AUTO NEG Rx LP 1000BaseT Cap */
#define YUSUR2_AUTO_NEG_LP_10GBASE_CAP	0x0800 /* AUTO NEG Rx LP 10GBaseT Cap */
#define YUSUR2_AUTO_NEG_10GBASET_STAT	0x0021 /* AUTO NEG 10G BaseT Stat */

#define YUSUR2_MDIO_TX_VENDOR_ALARMS_3		0xCC02 /* Vendor Alarms 3 Reg */
#define YUSUR2_MDIO_TX_VENDOR_ALARMS_3_RST_MASK	0x3 /* PHY Reset Complete Mask */
#define YUSUR2_MDIO_GLOBAL_RES_PR_10 0xC479 /* Global Resv Provisioning 10 Reg */
#define YUSUR2_MDIO_POWER_UP_STALL		0x8000 /* Power Up Stall */
#define YUSUR2_MDIO_GLOBAL_INT_CHIP_STD_MASK	0xFF00 /* int std mask */
#define YUSUR2_MDIO_GLOBAL_CHIP_STD_INT_FLAG	0xFC00 /* chip std int flag */
#define YUSUR2_MDIO_GLOBAL_INT_CHIP_VEN_MASK	0xFF01 /* int chip-wide mask */
#define YUSUR2_MDIO_GLOBAL_INT_CHIP_VEN_FLAG	0xFC01 /* int chip-wide mask */
#define YUSUR2_MDIO_GLOBAL_ALARM_1		0xCC00 /* Global alarm 1 */
#define YUSUR2_MDIO_GLOBAL_ALM_1_DEV_FAULT	0x0010 /* device fault */
#define YUSUR2_MDIO_GLOBAL_ALM_1_HI_TMP_FAIL	0x4000 /* high temp failure */
#define YUSUR2_MDIO_GLOBAL_FAULT_MSG	0xC850 /* Global Fault Message */
#define YUSUR2_MDIO_GLOBAL_FAULT_MSG_HI_TMP	0x8007 /* high temp failure */
#define YUSUR2_MDIO_GLOBAL_INT_MASK		0xD400 /* Global int mask */
#define YUSUR2_MDIO_GLOBAL_AN_VEN_ALM_INT_EN	0x1000 /* autoneg vendor alarm int enable */
#define YUSUR2_MDIO_GLOBAL_ALARM_1_INT		0x4 /* int in Global alarm 1 */
#define YUSUR2_MDIO_GLOBAL_VEN_ALM_INT_EN	0x1 /* vendor alarm int enable */
#define YUSUR2_MDIO_GLOBAL_STD_ALM2_INT		0x200 /* vendor alarm2 int mask */
#define YUSUR2_MDIO_GLOBAL_INT_HI_TEMP_EN	0x4000 /* int high temp enable */
#define YUSUR2_MDIO_GLOBAL_INT_DEV_FAULT_EN 0x0010 /* int dev fault enable */
#define YUSUR2_MDIO_PMA_PMD_CONTROL_ADDR	0x0000 /* PMA/PMD Control Reg */
#define YUSUR2_MDIO_PMA_PMD_SDA_SCL_ADDR	0xC30A /* PHY_XS SDA/SCL Addr Reg */
#define YUSUR2_MDIO_PMA_PMD_SDA_SCL_DATA	0xC30B /* PHY_XS SDA/SCL Data Reg */
#define YUSUR2_MDIO_PMA_PMD_SDA_SCL_STAT	0xC30C /* PHY_XS SDA/SCL Status Reg */
#define YUSUR2_MDIO_PMA_TX_VEN_LASI_INT_MASK 0xD401 /* PHY TX Vendor LASI */
#define YUSUR2_MDIO_PMA_TX_VEN_LASI_INT_EN   0x1 /* PHY TX Vendor LASI enable */
#define YUSUR2_MDIO_PMD_STD_TX_DISABLE_CNTR 0x9 /* Standard Transmit Dis Reg */
#define YUSUR2_MDIO_PMD_GLOBAL_TX_DISABLE 0x0001 /* PMD Global Transmit Dis */

#define YUSUR2_PCRC8ECL		0x0E810 /* PCR CRC-8 Error Count Lo */
#define YUSUR2_PCRC8ECH		0x0E811 /* PCR CRC-8 Error Count Hi */
#define YUSUR2_PCRC8ECH_MASK	0x1F
#define YUSUR2_LDPCECL		0x0E820 /* PCR Uncorrected Error Count Lo */
#define YUSUR2_LDPCECH		0x0E821 /* PCR Uncorrected Error Count Hi */

/* MII clause 22/28 definitions */
#define YUSUR2_MDIO_PHY_LOW_POWER_MODE	0x0800

#define YUSUR2_MDIO_XENPAK_LASI_STATUS		0x9005 /* XENPAK LASI Status register*/
#define YUSUR2_XENPAK_LASI_LINK_STATUS_ALARM	0x1 /* Link Status Alarm change */

#define YUSUR2_MDIO_AUTO_NEG_LINK_STATUS		0x4 /* Indicates if link is up */

#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_MASK	0x7 /* Speed/Duplex Mask */
#define YUSUR2_MDIO_AUTO_NEG_VEN_STAT_SPEED_MASK		0x6 /* Speed Mask */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_10M_HALF	0x0 /* 10Mb/s Half Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_10M_FULL	0x1 /* 10Mb/s Full Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_100M_HALF	0x2 /* 100Mb/s Half Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_100M_FULL	0x3 /* 100Mb/s Full Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_1GB_HALF	0x4 /* 1Gb/s Half Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_1GB_FULL	0x5 /* 1Gb/s Full Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_10GB_HALF	0x6 /* 10Gb/s Half Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_10GB_FULL	0x7 /* 10Gb/s Full Duplex */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_1GB		0x4 /* 1Gb/s */
#define YUSUR2_MDIO_AUTO_NEG_VENDOR_STATUS_10GB		0x6 /* 10Gb/s */

#define YUSUR2_MII_10GBASE_T_AUTONEG_CTRL_REG	0x20   /* 10G Control Reg */
#define YUSUR2_MII_AUTONEG_VENDOR_PROVISION_1_REG 0xC400 /* 1G Provisioning 1 */
#define YUSUR2_MII_AUTONEG_XNP_TX_REG		0x17   /* 1G XNP Transmit */
#define YUSUR2_MII_AUTONEG_ADVERTISE_REG		0x10   /* 100M Advertisement */
#define YUSUR2_MII_10GBASE_T_ADVERTISE		0x1000 /* full duplex, bit:12*/
#define YUSUR2_MII_1GBASE_T_ADVERTISE_XNP_TX	0x4000 /* full duplex, bit:14*/
#define YUSUR2_MII_1GBASE_T_ADVERTISE		0x8000 /* full duplex, bit:15*/
#define YUSUR2_MII_2_5GBASE_T_ADVERTISE		0x0400
#define YUSUR2_MII_5GBASE_T_ADVERTISE		0x0800
#define YUSUR2_MII_100BASE_T_ADVERTISE		0x0100 /* full duplex, bit:8 */
#define YUSUR2_MII_100BASE_T_ADVERTISE_HALF	0x0080 /* half duplex, bit:7 */
#define YUSUR2_MII_RESTART			0x200
#define YUSUR2_MII_AUTONEG_COMPLETE		0x20
#define YUSUR2_MII_AUTONEG_LINK_UP		0x04
#define YUSUR2_MII_AUTONEG_REG			0x0

#define YUSUR2_PHY_REVISION_MASK		0xFFFFFFF0
#define YUSUR2_MAX_PHY_ADDR		32

/* PHY IDs*/
#define TN1010_PHY_ID	0x00A19410
#define TNX_FW_REV	0xB
#define X540_PHY_ID	0x01540200
#define X550_PHY_ID2	0x01540223
#define X550_PHY_ID3	0x01540221
#define X557_PHY_ID	0x01540240
#define X557_PHY_ID2	0x01540250
#define AQ_FW_REV	0x20
#define QT2022_PHY_ID	0x0043A400
#define ATH_PHY_ID	0x03429050

/* PHY Types */
#define YUSUR2_M88E1500_E_PHY_ID	0x01410DD0
#define YUSUR2_M88E1543_E_PHY_ID	0x01410EA0

/* Special PHY Init Routine */
#define YUSUR2_PHY_INIT_OFFSET_NL	0x002B
#define YUSUR2_PHY_INIT_END_NL		0xFFFF
#define YUSUR2_CONTROL_MASK_NL		0xF000
#define YUSUR2_DATA_MASK_NL		0x0FFF
#define YUSUR2_CONTROL_SHIFT_NL		12
#define YUSUR2_DELAY_NL			0
#define YUSUR2_DATA_NL			1
#define YUSUR2_CONTROL_NL		0x000F
#define YUSUR2_CONTROL_EOL_NL		0x0FFF
#define YUSUR2_CONTROL_SOL_NL		0x0000

/* General purpose Interrupt Enable */
#define YUSUR2_SDP0_GPIEN	0x00000001 /* SDP0 */
#define YUSUR2_SDP1_GPIEN	0x00000002 /* SDP1 */
#define YUSUR2_SDP2_GPIEN	0x00000004 /* SDP2 */
#define YUSUR2_SDP0_GPIEN_X540	0x00000002 /* SDP0 on X540 and X550 */
#define YUSUR2_SDP1_GPIEN_X540	0x00000004 /* SDP1 on X540 and X550 */
#define YUSUR2_SDP2_GPIEN_X540	0x00000008 /* SDP2 on X540 and X550 */
#define YUSUR2_SDP0_GPIEN_X550	YUSUR2_SDP0_GPIEN_X540
#define YUSUR2_SDP1_GPIEN_X550	YUSUR2_SDP1_GPIEN_X540
#define YUSUR2_SDP2_GPIEN_X550	YUSUR2_SDP2_GPIEN_X540
#define YUSUR2_SDP0_GPIEN_X550EM_x	YUSUR2_SDP0_GPIEN_X540
#define YUSUR2_SDP1_GPIEN_X550EM_x	YUSUR2_SDP1_GPIEN_X540
#define YUSUR2_SDP2_GPIEN_X550EM_x	YUSUR2_SDP2_GPIEN_X540
#define YUSUR2_SDP0_GPIEN_X550EM_a	YUSUR2_SDP0_GPIEN_X540
#define YUSUR2_SDP1_GPIEN_X550EM_a	YUSUR2_SDP1_GPIEN_X540
#define YUSUR2_SDP2_GPIEN_X550EM_a	YUSUR2_SDP2_GPIEN_X540
#define YUSUR2_SDP0_GPIEN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SDP0_GPIEN)
#define YUSUR2_SDP1_GPIEN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SDP1_GPIEN)
#define YUSUR2_SDP2_GPIEN_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), SDP2_GPIEN)

#define YUSUR2_GPIE_MSIX_MODE	0x00000010 /* MSI-X mode */
#define YUSUR2_GPIE_OCD		0x00000020 /* Other Clear Disable */
#define YUSUR2_GPIE_EIMEN	0x00000040 /* Immediate Interrupt Enable */
#define YUSUR2_GPIE_EIAME	0x40000000
#define YUSUR2_GPIE_PBA_SUPPORT	0x80000000
#define YUSUR2_GPIE_RSC_DELAY_SHIFT	11
#define YUSUR2_GPIE_VTMODE_MASK	0x0000C000 /* VT Mode Mask */
#define YUSUR2_GPIE_VTMODE_16	0x00004000 /* 16 VFs 8 queues per VF */
#define YUSUR2_GPIE_VTMODE_32	0x00008000 /* 32 VFs 4 queues per VF */
#define YUSUR2_GPIE_VTMODE_64	0x0000C000 /* 64 VFs 2 queues per VF */

/* Packet Buffer Initialization */
#define YUSUR2_MAX_PACKET_BUFFERS	8

#define YUSUR2_TXPBSIZE_20KB	0x00005000 /* 20KB Packet Buffer */
#define YUSUR2_TXPBSIZE_40KB	0x0000A000 /* 40KB Packet Buffer */
#define YUSUR2_RXPBSIZE_48KB	0x0000C000 /* 48KB Packet Buffer */
#define YUSUR2_RXPBSIZE_64KB	0x00010000 /* 64KB Packet Buffer */
#define YUSUR2_RXPBSIZE_80KB	0x00014000 /* 80KB Packet Buffer */
#define YUSUR2_RXPBSIZE_128KB	0x00020000 /* 128KB Packet Buffer */
#define YUSUR2_RXPBSIZE_MAX	0x00080000 /* 512KB Packet Buffer */
#define YUSUR2_TXPBSIZE_MAX	0x00028000 /* 160KB Packet Buffer */

#define YUSUR2_TXPKT_SIZE_MAX	0xA /* Max Tx Packet size */
#define YUSUR2_MAX_PB		8

/* Packet buffer allocation strategies */
enum {
	PBA_STRATEGY_EQUAL	= 0, /* Distribute PB space equally */
#define PBA_STRATEGY_EQUAL	PBA_STRATEGY_EQUAL
	PBA_STRATEGY_WEIGHTED	= 1, /* Weight front half of TCs */
#define PBA_STRATEGY_WEIGHTED	PBA_STRATEGY_WEIGHTED
};

/* Transmit Flow Control status */
#define YUSUR2_TFCS_TXOFF	0x00000001
#define YUSUR2_TFCS_TXOFF0	0x00000100
#define YUSUR2_TFCS_TXOFF1	0x00000200
#define YUSUR2_TFCS_TXOFF2	0x00000400
#define YUSUR2_TFCS_TXOFF3	0x00000800
#define YUSUR2_TFCS_TXOFF4	0x00001000
#define YUSUR2_TFCS_TXOFF5	0x00002000
#define YUSUR2_TFCS_TXOFF6	0x00004000
#define YUSUR2_TFCS_TXOFF7	0x00008000

/* TCP Timer */
#define YUSUR2_TCPTIMER_KS		0x00000100
#define YUSUR2_TCPTIMER_COUNT_ENABLE	0x00000200
#define YUSUR2_TCPTIMER_COUNT_FINISH	0x00000400
#define YUSUR2_TCPTIMER_LOOP		0x00000800
#define YUSUR2_TCPTIMER_DURATION_MASK	0x000000FF

/* HLREG0 Bit Masks */
#define YUSUR2_HLREG0_TXCRCEN		0x00000001 /* bit  0 */
#define YUSUR2_HLREG0_RXCRCSTRP		0x00000002 /* bit  1 */
#define YUSUR2_HLREG0_JUMBOEN		0x00000004 /* bit  2 */
#define YUSUR2_HLREG0_TXPADEN		0x00000400 /* bit 10 */
#define YUSUR2_HLREG0_TXPAUSEEN		0x00001000 /* bit 12 */
#define YUSUR2_HLREG0_RXPAUSEEN		0x00004000 /* bit 14 */
#define YUSUR2_HLREG0_LPBK		0x00008000 /* bit 15 */
#define YUSUR2_HLREG0_MDCSPD		0x00010000 /* bit 16 */
#define YUSUR2_HLREG0_CONTMDC		0x00020000 /* bit 17 */
#define YUSUR2_HLREG0_CTRLFLTR		0x00040000 /* bit 18 */
#define YUSUR2_HLREG0_PREPEND		0x00F00000 /* bits 20-23 */
#define YUSUR2_HLREG0_PRIPAUSEEN		0x01000000 /* bit 24 */
#define YUSUR2_HLREG0_RXPAUSERECDA	0x06000000 /* bits 25-26 */
#define YUSUR2_HLREG0_RXLNGTHERREN	0x08000000 /* bit 27 */
#define YUSUR2_HLREG0_RXPADSTRIPEN	0x10000000 /* bit 28 */

/* VMD_CTL bitmasks */
#define YUSUR2_VMD_CTL_VMDQ_EN		0x00000001
#define YUSUR2_VMD_CTL_VMDQ_FILTER	0x00000002

/* VT_CTL bitmasks */
#define YUSUR2_VT_CTL_DIS_DEFPL		0x20000000 /* disable default pool */
#define YUSUR2_VT_CTL_REPLEN		0x40000000 /* replication enabled */
#define YUSUR2_VT_CTL_VT_ENABLE		0x00000001  /* Enable VT Mode */
#define YUSUR2_VT_CTL_POOL_SHIFT		7
#define YUSUR2_VT_CTL_POOL_MASK		(0x3F << YUSUR2_VT_CTL_POOL_SHIFT)

/* VMOLR bitmasks */
#define YUSUR2_VMOLR_UPE		0x00400000 /* unicast promiscuous */
#define YUSUR2_VMOLR_VPE		0x00800000 /* VLAN promiscuous */
#define YUSUR2_VMOLR_AUPE	0x01000000 /* accept untagged packets */
#define YUSUR2_VMOLR_ROMPE	0x02000000 /* accept packets in MTA tbl */
#define YUSUR2_VMOLR_ROPE	0x04000000 /* accept packets in UC tbl */
#define YUSUR2_VMOLR_BAM		0x08000000 /* accept broadcast packets */
#define YUSUR2_VMOLR_MPE		0x10000000 /* multicast promiscuous */

/* VFRE bitmask */
#define YUSUR2_VFRE_ENABLE_ALL	0xFFFFFFFF

#define YUSUR2_VF_INIT_TIMEOUT	200 /* Number of retries to clear RSTI */

/* RDHMPN and TDHMPN bitmasks */
#define YUSUR2_RDHMPN_RDICADDR		0x007FF800
#define YUSUR2_RDHMPN_RDICRDREQ		0x00800000
#define YUSUR2_RDHMPN_RDICADDR_SHIFT	11
#define YUSUR2_TDHMPN_TDICADDR		0x003FF800
#define YUSUR2_TDHMPN_TDICRDREQ		0x00800000
#define YUSUR2_TDHMPN_TDICADDR_SHIFT	11

#define YUSUR2_RDMAM_MEM_SEL_SHIFT		13
#define YUSUR2_RDMAM_DWORD_SHIFT			9
#define YUSUR2_RDMAM_DESC_COMP_FIFO		1
#define YUSUR2_RDMAM_DFC_CMD_FIFO		2
#define YUSUR2_RDMAM_RSC_HEADER_ADDR		3
#define YUSUR2_RDMAM_TCN_STATUS_RAM		4
#define YUSUR2_RDMAM_WB_COLL_FIFO		5
#define YUSUR2_RDMAM_QSC_CNT_RAM			6
#define YUSUR2_RDMAM_QSC_FCOE_RAM		7
#define YUSUR2_RDMAM_QSC_QUEUE_CNT		8
#define YUSUR2_RDMAM_QSC_QUEUE_RAM		0xA
#define YUSUR2_RDMAM_QSC_RSC_RAM			0xB
#define YUSUR2_RDMAM_DESC_COM_FIFO_RANGE		135
#define YUSUR2_RDMAM_DESC_COM_FIFO_COUNT		4
#define YUSUR2_RDMAM_DFC_CMD_FIFO_RANGE		48
#define YUSUR2_RDMAM_DFC_CMD_FIFO_COUNT		7
#define YUSUR2_RDMAM_RSC_HEADER_ADDR_RANGE	32
#define YUSUR2_RDMAM_RSC_HEADER_ADDR_COUNT	4
#define YUSUR2_RDMAM_TCN_STATUS_RAM_RANGE	256
#define YUSUR2_RDMAM_TCN_STATUS_RAM_COUNT	9
#define YUSUR2_RDMAM_WB_COLL_FIFO_RANGE		8
#define YUSUR2_RDMAM_WB_COLL_FIFO_COUNT		4
#define YUSUR2_RDMAM_QSC_CNT_RAM_RANGE		64
#define YUSUR2_RDMAM_QSC_CNT_RAM_COUNT		4
#define YUSUR2_RDMAM_QSC_FCOE_RAM_RANGE		512
#define YUSUR2_RDMAM_QSC_FCOE_RAM_COUNT		5
#define YUSUR2_RDMAM_QSC_QUEUE_CNT_RANGE		32
#define YUSUR2_RDMAM_QSC_QUEUE_CNT_COUNT		4
#define YUSUR2_RDMAM_QSC_QUEUE_RAM_RANGE		128
#define YUSUR2_RDMAM_QSC_QUEUE_RAM_COUNT		8
#define YUSUR2_RDMAM_QSC_RSC_RAM_RANGE		32
#define YUSUR2_RDMAM_QSC_RSC_RAM_COUNT		8

#define YUSUR2_TXDESCIC_READY	0x80000000

/* Receive Checksum Control */
#define YUSUR2_RXCSUM_IPPCSE	0x00001000 /* IP payload checksum enable */
#define YUSUR2_RXCSUM_PCSD	0x00002000 /* packet checksum disabled */

/* FCRTL Bit Masks */
#define YUSUR2_FCRTL_XONE	0x80000000 /* XON enable */
#define YUSUR2_FCRTH_FCEN	0x80000000 /* Packet buffer fc enable */

/* PAP bit masks*/
#define YUSUR2_PAP_TXPAUSECNT_MASK	0x0000FFFF /* Pause counter mask */

/* RMCS Bit Masks */
#define YUSUR2_RMCS_RRM			0x00000002 /* Rx Recycle Mode enable */
/* Receive Arbitration Control: 0 Round Robin, 1 DFP */
#define YUSUR2_RMCS_RAC			0x00000004
/* Deficit Fixed Prio ena */
#define YUSUR2_RMCS_DFP			YUSUR2_RMCS_RAC
#define YUSUR2_RMCS_TFCE_802_3X		0x00000008 /* Tx Priority FC ena */
#define YUSUR2_RMCS_TFCE_PRIORITY	0x00000010 /* Tx Priority FC ena */
#define YUSUR2_RMCS_ARBDIS		0x00000040 /* Arbitration disable bit */

/* FCCFG Bit Masks */
#define YUSUR2_FCCFG_TFCE_802_3X		0x00000008 /* Tx link FC enable */
#define YUSUR2_FCCFG_TFCE_PRIORITY	0x00000010 /* Tx priority FC enable */

/* Interrupt register bitmasks */

/* Extended Interrupt Cause Read */
#define YUSUR2_EICR_RTX_QUEUE	0x0000FFFF /* RTx Queue Interrupt */
#define YUSUR2_EICR_FLOW_DIR	0x00010000 /* FDir Exception */
#define YUSUR2_EICR_RX_MISS	0x00020000 /* Packet Buffer Overrun */
#define YUSUR2_EICR_PCI		0x00040000 /* PCI Exception */
#define YUSUR2_EICR_MAILBOX	0x00080000 /* VF to PF Mailbox Interrupt */
#define YUSUR2_EICR_LSC		0x00100000 /* Link Status Change */
#define YUSUR2_EICR_LINKSEC	0x00200000 /* PN Threshold */
#define YUSUR2_EICR_MNG		0x00400000 /* Manageability Event Interrupt */
#define YUSUR2_EICR_TS		0x00800000 /* Thermal Sensor Event */
#define YUSUR2_EICR_TIMESYNC	0x01000000 /* Timesync Event */
#define YUSUR2_EICR_GPI_SDP0	0x01000000 /* Gen Purpose Interrupt on SDP0 */
#define YUSUR2_EICR_GPI_SDP1	0x02000000 /* Gen Purpose Interrupt on SDP1 */
#define YUSUR2_EICR_GPI_SDP2	0x04000000 /* Gen Purpose Interrupt on SDP2 */
#define YUSUR2_EICR_ECC		0x10000000 /* ECC Error */
#define YUSUR2_EICR_GPI_SDP0_X540 0x02000000 /* Gen Purpose Interrupt on SDP0 */
#define YUSUR2_EICR_GPI_SDP1_X540 0x04000000 /* Gen Purpose Interrupt on SDP1 */
#define YUSUR2_EICR_GPI_SDP2_X540 0x08000000 /* Gen Purpose Interrupt on SDP2 */
#define YUSUR2_EICR_GPI_SDP0_X550	YUSUR2_EICR_GPI_SDP0_X540
#define YUSUR2_EICR_GPI_SDP1_X550	YUSUR2_EICR_GPI_SDP1_X540
#define YUSUR2_EICR_GPI_SDP2_X550	YUSUR2_EICR_GPI_SDP2_X540
#define YUSUR2_EICR_GPI_SDP0_X550EM_x	YUSUR2_EICR_GPI_SDP0_X540
#define YUSUR2_EICR_GPI_SDP1_X550EM_x	YUSUR2_EICR_GPI_SDP1_X540
#define YUSUR2_EICR_GPI_SDP2_X550EM_x	YUSUR2_EICR_GPI_SDP2_X540
#define YUSUR2_EICR_GPI_SDP0_X550EM_a	YUSUR2_EICR_GPI_SDP0_X540
#define YUSUR2_EICR_GPI_SDP1_X550EM_a	YUSUR2_EICR_GPI_SDP1_X540
#define YUSUR2_EICR_GPI_SDP2_X550EM_a	YUSUR2_EICR_GPI_SDP2_X540
#define YUSUR2_EICR_GPI_SDP0_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), EICR_GPI_SDP0)
#define YUSUR2_EICR_GPI_SDP1_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), EICR_GPI_SDP1)
#define YUSUR2_EICR_GPI_SDP2_BY_MAC(_hw)	YUSUR2_BY_MAC((_hw), EICR_GPI_SDP2)

#define YUSUR2_EICR_PBUR		0x10000000 /* Packet Buffer Handler Error */
#define YUSUR2_EICR_DHER		0x20000000 /* Descriptor Handler Error */
#define YUSUR2_EICR_TCP_TIMER	0x40000000 /* TCP Timer */
#define YUSUR2_EICR_OTHER	0x80000000 /* Interrupt Cause Active */

/* Extended Interrupt Cause Set */
#define YUSUR2_EICS_RTX_QUEUE	YUSUR2_EICR_RTX_QUEUE /* RTx Queue Interrupt */
#define YUSUR2_EICS_FLOW_DIR	YUSUR2_EICR_FLOW_DIR  /* FDir Exception */
#define YUSUR2_EICS_RX_MISS	YUSUR2_EICR_RX_MISS   /* Pkt Buffer Overrun */
#define YUSUR2_EICS_PCI		YUSUR2_EICR_PCI /* PCI Exception */
#define YUSUR2_EICS_MAILBOX	YUSUR2_EICR_MAILBOX   /* VF to PF Mailbox Int */
#define YUSUR2_EICS_LSC		YUSUR2_EICR_LSC /* Link Status Change */
#define YUSUR2_EICS_MNG		YUSUR2_EICR_MNG /* MNG Event Interrupt */
#define YUSUR2_EICS_TIMESYNC	YUSUR2_EICR_TIMESYNC /* Timesync Event */
#define YUSUR2_EICS_GPI_SDP0	YUSUR2_EICR_GPI_SDP0 /* SDP0 Gen Purpose Int */
#define YUSUR2_EICS_GPI_SDP1	YUSUR2_EICR_GPI_SDP1 /* SDP1 Gen Purpose Int */
#define YUSUR2_EICS_GPI_SDP2	YUSUR2_EICR_GPI_SDP2 /* SDP2 Gen Purpose Int */
#define YUSUR2_EICS_ECC		YUSUR2_EICR_ECC /* ECC Error */
#define YUSUR2_EICS_GPI_SDP0_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP0_BY_MAC(_hw)
#define YUSUR2_EICS_GPI_SDP1_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP1_BY_MAC(_hw)
#define YUSUR2_EICS_GPI_SDP2_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP2_BY_MAC(_hw)
#define YUSUR2_EICS_PBUR		YUSUR2_EICR_PBUR /* Pkt Buf Handler Err */
#define YUSUR2_EICS_DHER		YUSUR2_EICR_DHER /* Desc Handler Error */
#define YUSUR2_EICS_TCP_TIMER	YUSUR2_EICR_TCP_TIMER /* TCP Timer */
#define YUSUR2_EICS_OTHER	YUSUR2_EICR_OTHER /* INT Cause Active */

/* Extended Interrupt Mask Set */
#define YUSUR2_EIMS_RTX_QUEUE	YUSUR2_EICR_RTX_QUEUE /* RTx Queue Interrupt */
#define YUSUR2_EIMS_FLOW_DIR	YUSUR2_EICR_FLOW_DIR /* FDir Exception */
#define YUSUR2_EIMS_RX_MISS	YUSUR2_EICR_RX_MISS /* Packet Buffer Overrun */
#define YUSUR2_EIMS_PCI		YUSUR2_EICR_PCI /* PCI Exception */
#define YUSUR2_EIMS_MAILBOX	YUSUR2_EICR_MAILBOX   /* VF to PF Mailbox Int */
#define YUSUR2_EIMS_LSC		YUSUR2_EICR_LSC /* Link Status Change */
#define YUSUR2_EIMS_MNG		YUSUR2_EICR_MNG /* MNG Event Interrupt */
#define YUSUR2_EIMS_TS		YUSUR2_EICR_TS /* Thermal Sensor Event */
#define YUSUR2_EIMS_TIMESYNC	YUSUR2_EICR_TIMESYNC /* Timesync Event */
#define YUSUR2_EIMS_GPI_SDP0	YUSUR2_EICR_GPI_SDP0 /* SDP0 Gen Purpose Int */
#define YUSUR2_EIMS_GPI_SDP1	YUSUR2_EICR_GPI_SDP1 /* SDP1 Gen Purpose Int */
#define YUSUR2_EIMS_GPI_SDP2	YUSUR2_EICR_GPI_SDP2 /* SDP2 Gen Purpose Int */
#define YUSUR2_EIMS_ECC		YUSUR2_EICR_ECC /* ECC Error */
#define YUSUR2_EIMS_GPI_SDP0_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP0_BY_MAC(_hw)
#define YUSUR2_EIMS_GPI_SDP1_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP1_BY_MAC(_hw)
#define YUSUR2_EIMS_GPI_SDP2_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP2_BY_MAC(_hw)
#define YUSUR2_EIMS_PBUR		YUSUR2_EICR_PBUR /* Pkt Buf Handler Err */
#define YUSUR2_EIMS_DHER		YUSUR2_EICR_DHER /* Descr Handler Error */
#define YUSUR2_EIMS_TCP_TIMER	YUSUR2_EICR_TCP_TIMER /* TCP Timer */
#define YUSUR2_EIMS_OTHER	YUSUR2_EICR_OTHER /* INT Cause Active */

/* Extended Interrupt Mask Clear */
#define YUSUR2_EIMC_RTX_QUEUE	YUSUR2_EICR_RTX_QUEUE /* RTx Queue Interrupt */
#define YUSUR2_EIMC_FLOW_DIR	YUSUR2_EICR_FLOW_DIR /* FDir Exception */
#define YUSUR2_EIMC_RX_MISS	YUSUR2_EICR_RX_MISS /* Packet Buffer Overrun */
#define YUSUR2_EIMC_PCI		YUSUR2_EICR_PCI /* PCI Exception */
#define YUSUR2_EIMC_MAILBOX	YUSUR2_EICR_MAILBOX /* VF to PF Mailbox Int */
#define YUSUR2_EIMC_LSC		YUSUR2_EICR_LSC /* Link Status Change */
#define YUSUR2_EIMC_MNG		YUSUR2_EICR_MNG /* MNG Event Interrupt */
#define YUSUR2_EIMC_TIMESYNC	YUSUR2_EICR_TIMESYNC /* Timesync Event */
#define YUSUR2_EIMC_GPI_SDP0	YUSUR2_EICR_GPI_SDP0 /* SDP0 Gen Purpose Int */
#define YUSUR2_EIMC_GPI_SDP1	YUSUR2_EICR_GPI_SDP1 /* SDP1 Gen Purpose Int */
#define YUSUR2_EIMC_GPI_SDP2	YUSUR2_EICR_GPI_SDP2  /* SDP2 Gen Purpose Int */
#define YUSUR2_EIMC_ECC		YUSUR2_EICR_ECC /* ECC Error */
#define YUSUR2_EIMC_GPI_SDP0_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP0_BY_MAC(_hw)
#define YUSUR2_EIMC_GPI_SDP1_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP1_BY_MAC(_hw)
#define YUSUR2_EIMC_GPI_SDP2_BY_MAC(_hw)	YUSUR2_EICR_GPI_SDP2_BY_MAC(_hw)
#define YUSUR2_EIMC_PBUR		YUSUR2_EICR_PBUR /* Pkt Buf Handler Err */
#define YUSUR2_EIMC_DHER		YUSUR2_EICR_DHER /* Desc Handler Err */
#define YUSUR2_EIMC_TCP_TIMER	YUSUR2_EICR_TCP_TIMER /* TCP Timer */
#define YUSUR2_EIMC_OTHER	YUSUR2_EICR_OTHER /* INT Cause Active */

#define YUSUR2_EIMS_ENABLE_MASK ( \
				YUSUR2_EIMS_RTX_QUEUE	| \
				YUSUR2_EIMS_LSC		| \
				YUSUR2_EIMS_TCP_TIMER	| \
				YUSUR2_EIMS_OTHER)

/* Immediate Interrupt Rx (A.K.A. Low Latency Interrupt) */
#define YUSUR2_IMIR_PORT_IM_EN	0x00010000  /* TCP port enable */
#define YUSUR2_IMIR_PORT_BP	0x00020000  /* TCP port check bypass */
#define YUSUR2_IMIREXT_SIZE_BP	0x00001000  /* Packet size bypass */
#define YUSUR2_IMIREXT_CTRL_URG	0x00002000  /* Check URG bit in header */
#define YUSUR2_IMIREXT_CTRL_ACK	0x00004000  /* Check ACK bit in header */
#define YUSUR2_IMIREXT_CTRL_PSH	0x00008000  /* Check PSH bit in header */
#define YUSUR2_IMIREXT_CTRL_RST	0x00010000  /* Check RST bit in header */
#define YUSUR2_IMIREXT_CTRL_SYN	0x00020000  /* Check SYN bit in header */
#define YUSUR2_IMIREXT_CTRL_FIN	0x00040000  /* Check FIN bit in header */
#define YUSUR2_IMIREXT_CTRL_BP	0x00080000  /* Bypass check of control bits */
#define YUSUR2_IMIR_SIZE_BP_82599	0x00001000 /* Packet size bypass */
#define YUSUR2_IMIR_CTRL_URG_82599	0x00002000 /* Check URG bit in header */
#define YUSUR2_IMIR_CTRL_ACK_82599	0x00004000 /* Check ACK bit in header */
#define YUSUR2_IMIR_CTRL_PSH_82599	0x00008000 /* Check PSH bit in header */
#define YUSUR2_IMIR_CTRL_RST_82599	0x00010000 /* Check RST bit in header */
#define YUSUR2_IMIR_CTRL_SYN_82599	0x00020000 /* Check SYN bit in header */
#define YUSUR2_IMIR_CTRL_FIN_82599	0x00040000 /* Check FIN bit in header */
#define YUSUR2_IMIR_CTRL_BP_82599	0x00080000 /* Bypass chk of ctrl bits */
#define YUSUR2_IMIR_LLI_EN_82599		0x00100000 /* Enables low latency Int */
#define YUSUR2_IMIR_RX_QUEUE_MASK_82599	0x0000007F /* Rx Queue Mask */
#define YUSUR2_IMIR_RX_QUEUE_SHIFT_82599	21 /* Rx Queue Shift */
#define YUSUR2_IMIRVP_PRIORITY_MASK	0x00000007 /* VLAN priority mask */
#define YUSUR2_IMIRVP_PRIORITY_EN	0x00000008 /* VLAN priority enable */

#define YUSUR2_MAX_FTQF_FILTERS		128
#define YUSUR2_FTQF_PROTOCOL_MASK	0x00000003
#define YUSUR2_FTQF_PROTOCOL_TCP		0x00000000
#define YUSUR2_FTQF_PROTOCOL_UDP		0x00000001
#define YUSUR2_FTQF_PROTOCOL_SCTP	2
#define YUSUR2_FTQF_PRIORITY_MASK	0x00000007
#define YUSUR2_FTQF_PRIORITY_SHIFT	2
#define YUSUR2_FTQF_POOL_MASK		0x0000003F
#define YUSUR2_FTQF_POOL_SHIFT		8
#define YUSUR2_FTQF_5TUPLE_MASK_MASK	0x0000001F
#define YUSUR2_FTQF_5TUPLE_MASK_SHIFT	25
#define YUSUR2_FTQF_SOURCE_ADDR_MASK	0x1E
#define YUSUR2_FTQF_DEST_ADDR_MASK	0x1D
#define YUSUR2_FTQF_SOURCE_PORT_MASK	0x1B
#define YUSUR2_FTQF_DEST_PORT_MASK	0x17
#define YUSUR2_FTQF_PROTOCOL_COMP_MASK	0x0F
#define YUSUR2_FTQF_POOL_MASK_EN		0x40000000
#define YUSUR2_FTQF_QUEUE_ENABLE		0x80000000

/* Interrupt clear mask */
#define YUSUR2_IRQ_CLEAR_MASK	0xFFFFFFFF

/* Interrupt Vector Allocation Registers */
#define YUSUR2_IVAR_REG_NUM		25
#define YUSUR2_IVAR_REG_NUM_82599	64
#define YUSUR2_IVAR_TXRX_ENTRY		96
#define YUSUR2_IVAR_RX_ENTRY		64
#define YUSUR2_IVAR_RX_QUEUE(_i)		(0 + (_i))
#define YUSUR2_IVAR_TX_QUEUE(_i)		(64 + (_i))
#define YUSUR2_IVAR_TX_ENTRY		32

#define YUSUR2_IVAR_TCP_TIMER_INDEX	96 /* 0 based index */
#define YUSUR2_IVAR_OTHER_CAUSES_INDEX	97 /* 0 based index */

#define YUSUR2_MSIX_VECTOR(_i)		(0 + (_i))

#define YUSUR2_IVAR_ALLOC_VAL		0x80 /* Interrupt Allocation valid */

/* ETYPE Queue Filter/Select Bit Masks */
#define YUSUR2_MAX_ETQF_FILTERS		8
#define YUSUR2_ETQF_FCOE			0x08000000 /* bit 27 */
#define YUSUR2_ETQF_BCN			0x10000000 /* bit 28 */
#define YUSUR2_ETQF_TX_ANTISPOOF		0x20000000 /* bit 29 */
#define YUSUR2_ETQF_1588			0x40000000 /* bit 30 */
#define YUSUR2_ETQF_FILTER_EN		0x80000000 /* bit 31 */
#define YUSUR2_ETQF_POOL_ENABLE		(1 << 26) /* bit 26 */
#define YUSUR2_ETQF_POOL_SHIFT		20

#define YUSUR2_ETQS_RX_QUEUE		0x007F0000 /* bits 22:16 */
#define YUSUR2_ETQS_RX_QUEUE_SHIFT	16
#define YUSUR2_ETQS_LLI			0x20000000 /* bit 29 */
#define YUSUR2_ETQS_QUEUE_EN		0x80000000 /* bit 31 */

/*
 * ETQF filter list: one static filter per filter consumer. This is
 *		   to avoid filter collisions later. Add new filters
 *		   here!!
 *
 * Current filters:
 *	EAPOL 802.1x (0x888e): Filter 0
 *	FCoE (0x8906):	 Filter 2
 *	1588 (0x88f7):	 Filter 3
 *	FIP  (0x8914):	 Filter 4
 *	LLDP (0x88CC):	 Filter 5
 *	LACP (0x8809):	 Filter 6
 *	FC   (0x8808):	 Filter 7
 */
#define YUSUR2_ETQF_FILTER_EAPOL		0
#define YUSUR2_ETQF_FILTER_FCOE		2
#define YUSUR2_ETQF_FILTER_1588		3
#define YUSUR2_ETQF_FILTER_FIP		4
#define YUSUR2_ETQF_FILTER_LLDP		5
#define YUSUR2_ETQF_FILTER_LACP		6
#define YUSUR2_ETQF_FILTER_FC		7
/* VLAN Control Bit Masks */
#define YUSUR2_VLNCTRL_VET		0x0000FFFF  /* bits 0-15 */
#define YUSUR2_VLNCTRL_CFI		0x10000000  /* bit 28 */
#define YUSUR2_VLNCTRL_CFIEN		0x20000000  /* bit 29 */
#define YUSUR2_VLNCTRL_VFE		0x40000000  /* bit 30 */
#define YUSUR2_VLNCTRL_VME		0x80000000  /* bit 31 */

/* VLAN pool filtering masks */
#define YUSUR2_VLVF_VIEN			0x80000000  /* filter is valid */
#define YUSUR2_VLVF_ENTRIES		64
#define YUSUR2_VLVF_VLANID_MASK		0x00000FFF
/* Per VF Port VLAN insertion rules */
#define YUSUR2_VMVIR_VLANA_DEFAULT	0x40000000 /* Always use default VLAN */
#define YUSUR2_VMVIR_VLANA_NEVER		0x80000000 /* Never insert VLAN tag */

#define YUSUR2_ETHERNET_IEEE_VLAN_TYPE	0x8100  /* 802.1q protocol */

/* STATUS Bit Masks */
#define YUSUR2_STATUS_LAN_ID		0x0000000C /* LAN ID */
#define YUSUR2_STATUS_LAN_ID_SHIFT	2 /* LAN ID Shift*/
#define YUSUR2_STATUS_GIO		0x00080000 /* GIO Master Ena Status */

#define YUSUR2_STATUS_LAN_ID_0	0x00000000 /* LAN ID 0 */
#define YUSUR2_STATUS_LAN_ID_1	0x00000004 /* LAN ID 1 */

/* ESDP Bit Masks */
#define YUSUR2_ESDP_SDP0		0x00000001 /* SDP0 Data Value */
#define YUSUR2_ESDP_SDP1		0x00000002 /* SDP1 Data Value */
#define YUSUR2_ESDP_SDP2		0x00000004 /* SDP2 Data Value */
#define YUSUR2_ESDP_SDP3		0x00000008 /* SDP3 Data Value */
#define YUSUR2_ESDP_SDP4		0x00000010 /* SDP4 Data Value */
#define YUSUR2_ESDP_SDP5		0x00000020 /* SDP5 Data Value */
#define YUSUR2_ESDP_SDP6		0x00000040 /* SDP6 Data Value */
#define YUSUR2_ESDP_SDP7		0x00000080 /* SDP7 Data Value */
#define YUSUR2_ESDP_SDP0_DIR	0x00000100 /* SDP0 IO direction */
#define YUSUR2_ESDP_SDP1_DIR	0x00000200 /* SDP1 IO direction */
#define YUSUR2_ESDP_SDP2_DIR	0x00000400 /* SDP1 IO direction */
#define YUSUR2_ESDP_SDP3_DIR	0x00000800 /* SDP3 IO direction */
#define YUSUR2_ESDP_SDP4_DIR	0x00001000 /* SDP4 IO direction */
#define YUSUR2_ESDP_SDP5_DIR	0x00002000 /* SDP5 IO direction */
#define YUSUR2_ESDP_SDP6_DIR	0x00004000 /* SDP6 IO direction */
#define YUSUR2_ESDP_SDP7_DIR	0x00008000 /* SDP7 IO direction */
#define YUSUR2_ESDP_SDP0_NATIVE	0x00010000 /* SDP0 IO mode */
#define YUSUR2_ESDP_SDP1_NATIVE	0x00020000 /* SDP1 IO mode */


/* LEDCTL Bit Masks */
#define YUSUR2_LED_IVRT_BASE		0x00000040
#define YUSUR2_LED_BLINK_BASE		0x00000080
#define YUSUR2_LED_MODE_MASK_BASE	0x0000000F
#define YUSUR2_LED_OFFSET(_base, _i)	(_base << (8 * (_i)))
#define YUSUR2_LED_MODE_SHIFT(_i)	(8*(_i))
#define YUSUR2_LED_IVRT(_i)	YUSUR2_LED_OFFSET(YUSUR2_LED_IVRT_BASE, _i)
#define YUSUR2_LED_BLINK(_i)	YUSUR2_LED_OFFSET(YUSUR2_LED_BLINK_BASE, _i)
#define YUSUR2_LED_MODE_MASK(_i)	YUSUR2_LED_OFFSET(YUSUR2_LED_MODE_MASK_BASE, _i)
#define YUSUR2_X557_LED_MANUAL_SET_MASK	(1 << 8)
#define YUSUR2_X557_MAX_LED_INDEX	3
#define YUSUR2_X557_LED_PROVISIONING	0xC430

/* LED modes */
#define YUSUR2_LED_LINK_UP	0x0
#define YUSUR2_LED_LINK_10G	0x1
#define YUSUR2_LED_MAC		0x2
#define YUSUR2_LED_FILTER	0x3
#define YUSUR2_LED_LINK_ACTIVE	0x4
#define YUSUR2_LED_LINK_1G	0x5
#define YUSUR2_LED_ON		0xE
#define YUSUR2_LED_OFF		0xF

/* AUTOC Bit Masks */
#define YUSUR2_AUTOC_KX4_KX_SUPP_MASK 0xC0000000
#define YUSUR2_AUTOC_KX4_SUPP	0x80000000
#define YUSUR2_AUTOC_KX_SUPP	0x40000000
#define YUSUR2_AUTOC_PAUSE	0x30000000
#define YUSUR2_AUTOC_ASM_PAUSE	0x20000000
#define YUSUR2_AUTOC_SYM_PAUSE	0x10000000
#define YUSUR2_AUTOC_RF		0x08000000
#define YUSUR2_AUTOC_PD_TMR	0x06000000
#define YUSUR2_AUTOC_AN_RX_LOOSE	0x01000000
#define YUSUR2_AUTOC_AN_RX_DRIFT	0x00800000
#define YUSUR2_AUTOC_AN_RX_ALIGN	0x007C0000
#define YUSUR2_AUTOC_FECA	0x00040000
#define YUSUR2_AUTOC_FECR	0x00020000
#define YUSUR2_AUTOC_KR_SUPP	0x00010000
#define YUSUR2_AUTOC_AN_RESTART	0x00001000
#define YUSUR2_AUTOC_FLU		0x00000001
#define YUSUR2_AUTOC_LMS_SHIFT	13
#define YUSUR2_AUTOC_LMS_10G_SERIAL	(0x3 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_KX4_KX_KR	(0x4 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_SGMII_1G_100M	(0x5 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN	(0x6 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_KX4_KX_KR_SGMII	(0x7 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_MASK		(0x7 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_1G_LINK_NO_AN	(0x0 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_10G_LINK_NO_AN	(0x1 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_1G_AN		(0x2 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_KX4_AN		(0x4 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_KX4_AN_1G_AN	(0x6 << YUSUR2_AUTOC_LMS_SHIFT)
#define YUSUR2_AUTOC_LMS_ATTACH_TYPE	(0x7 << YUSUR2_AUTOC_10G_PMA_PMD_SHIFT)

#define YUSUR2_AUTOC_1G_PMA_PMD_MASK	0x00000200
#define YUSUR2_AUTOC_1G_PMA_PMD_SHIFT	9
#define YUSUR2_AUTOC_10G_PMA_PMD_MASK	0x00000180
#define YUSUR2_AUTOC_10G_PMA_PMD_SHIFT	7
#define YUSUR2_AUTOC_10G_XAUI	(0x0 << YUSUR2_AUTOC_10G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_10G_KX4	(0x1 << YUSUR2_AUTOC_10G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_10G_CX4	(0x2 << YUSUR2_AUTOC_10G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_1G_BX	(0x0 << YUSUR2_AUTOC_1G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_1G_KX	(0x1 << YUSUR2_AUTOC_1G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_1G_SFI	(0x0 << YUSUR2_AUTOC_1G_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC_1G_KX_BX	(0x1 << YUSUR2_AUTOC_1G_PMA_PMD_SHIFT)

#define YUSUR2_AUTOC2_UPPER_MASK	0xFFFF0000
#define YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_MASK	0x00030000
#define YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_SHIFT	16
#define YUSUR2_AUTOC2_10G_KR	(0x0 << YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC2_10G_XFI	(0x1 << YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC2_10G_SFI	(0x2 << YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_SHIFT)
#define YUSUR2_AUTOC2_LINK_DISABLE_ON_D3_MASK	0x50000000
#define YUSUR2_AUTOC2_LINK_DISABLE_MASK		0x70000000

#define YUSUR2_MACC_FLU		0x00000001
#define YUSUR2_MACC_FSV_10G	0x00030000
#define YUSUR2_MACC_FS		0x00040000
#define YUSUR2_MAC_RX2TX_LPBK	0x00000002

/* Veto Bit definiton */
#define YUSUR2_MMNGC_MNG_VETO	0x00000001

/* LINKS Bit Masks */
#define YUSUR2_LINKS_KX_AN_COMP	0x80000000
#define YUSUR2_LINKS_UP		0x40000000
#define YUSUR2_LINKS_SPEED	0x20000000
#define YUSUR2_LINKS_MODE	0x18000000
#define YUSUR2_LINKS_RX_MODE	0x06000000
#define YUSUR2_LINKS_TX_MODE	0x01800000
#define YUSUR2_LINKS_XGXS_EN	0x00400000
#define YUSUR2_LINKS_SGMII_EN	0x02000000
#define YUSUR2_LINKS_PCS_1G_EN	0x00200000
#define YUSUR2_LINKS_1G_AN_EN	0x00100000
#define YUSUR2_LINKS_KX_AN_IDLE	0x00080000
#define YUSUR2_LINKS_1G_SYNC	0x00040000
#define YUSUR2_LINKS_10G_ALIGN	0x00020000
#define YUSUR2_LINKS_10G_LANE_SYNC	0x00017000
#define YUSUR2_LINKS_TL_FAULT		0x00001000
#define YUSUR2_LINKS_SIGNAL		0x00000F00

#define YUSUR2_LINKS_SPEED_NON_STD	0x08000000
#define YUSUR2_LINKS_SPEED_82599		0x30000000
#define YUSUR2_LINKS_SPEED_10G_82599	0x30000000
#define YUSUR2_LINKS_SPEED_1G_82599	0x20000000
#define YUSUR2_LINKS_SPEED_100_82599	0x10000000
#define YUSUR2_LINKS_SPEED_10_X550EM_A	0x00000000
#define YUSUR2_LINK_UP_TIME		90 /* 9.0 Seconds */
#define YUSUR2_AUTO_NEG_TIME		45 /* 4.5 Seconds */

#define YUSUR2_LINKS2_AN_SUPPORTED	0x00000040

/* PCS1GLSTA Bit Masks */
#define YUSUR2_PCS1GLSTA_LINK_OK		1
#define YUSUR2_PCS1GLSTA_SYNK_OK		0x10
#define YUSUR2_PCS1GLSTA_AN_COMPLETE	0x10000
#define YUSUR2_PCS1GLSTA_AN_PAGE_RX	0x20000
#define YUSUR2_PCS1GLSTA_AN_TIMED_OUT	0x40000
#define YUSUR2_PCS1GLSTA_AN_REMOTE_FAULT	0x80000
#define YUSUR2_PCS1GLSTA_AN_ERROR_RWS	0x100000

#define YUSUR2_PCS1GANA_SYM_PAUSE	0x80
#define YUSUR2_PCS1GANA_ASM_PAUSE	0x100

/* PCS1GLCTL Bit Masks */
#define YUSUR2_PCS1GLCTL_AN_1G_TIMEOUT_EN 0x00040000 /* PCS 1G autoneg to en */
#define YUSUR2_PCS1GLCTL_FLV_LINK_UP	1
#define YUSUR2_PCS1GLCTL_FORCE_LINK	0x20
#define YUSUR2_PCS1GLCTL_LOW_LINK_LATCH	0x40
#define YUSUR2_PCS1GLCTL_AN_ENABLE	0x10000
#define YUSUR2_PCS1GLCTL_AN_RESTART	0x20000

/* ANLP1 Bit Masks */
#define YUSUR2_ANLP1_PAUSE		0x0C00
#define YUSUR2_ANLP1_SYM_PAUSE		0x0400
#define YUSUR2_ANLP1_ASM_PAUSE		0x0800
#define YUSUR2_ANLP1_AN_STATE_MASK	0x000f0000

/* SW Semaphore Register bitmasks */
#define YUSUR2_SWSM_SMBI		0x00000001 /* Driver Semaphore bit */
#define YUSUR2_SWSM_SWESMBI	0x00000002 /* FW Semaphore bit */
#define YUSUR2_SWSM_WMNG		0x00000004 /* Wake MNG Clock */
#define YUSUR2_SWFW_REGSMP	0x80000000 /* Register Semaphore bit 31 */

/* SW_FW_SYNC/GSSR definitions */
#define YUSUR2_GSSR_EEP_SM		0x0001
#define YUSUR2_GSSR_PHY0_SM		0x0002
#define YUSUR2_GSSR_PHY1_SM		0x0004
#define YUSUR2_GSSR_MAC_CSR_SM		0x0008
#define YUSUR2_GSSR_FLASH_SM		0x0010
#define YUSUR2_GSSR_NVM_UPDATE_SM	0x0200
#define YUSUR2_GSSR_SW_MNG_SM		0x0400
#define YUSUR2_GSSR_TOKEN_SM	0x40000000 /* SW bit for shared access */
#define YUSUR2_GSSR_SHARED_I2C_SM 0x1806 /* Wait for both phys and both I2Cs */
#define YUSUR2_GSSR_I2C_MASK	0x1800
#define YUSUR2_GSSR_NVM_PHY_MASK	0xF

/* FW Status register bitmask */
#define YUSUR2_FWSTS_FWRI	0x00000200 /* Firmware Reset Indication */

/* EEC Register */
#define YUSUR2_EEC_SK		0x00000001 /* EEPROM Clock */
#define YUSUR2_EEC_CS		0x00000002 /* EEPROM Chip Select */
#define YUSUR2_EEC_DI		0x00000004 /* EEPROM Data In */
#define YUSUR2_EEC_DO		0x00000008 /* EEPROM Data Out */
#define YUSUR2_EEC_FWE_MASK	0x00000030 /* FLASH Write Enable */
#define YUSUR2_EEC_FWE_DIS	0x00000010 /* Disable FLASH writes */
#define YUSUR2_EEC_FWE_EN	0x00000020 /* Enable FLASH writes */
#define YUSUR2_EEC_FWE_SHIFT	4
#define YUSUR2_EEC_REQ		0x00000040 /* EEPROM Access Request */
#define YUSUR2_EEC_GNT		0x00000080 /* EEPROM Access Grant */
#define YUSUR2_EEC_PRES		0x00000100 /* EEPROM Present */
#define YUSUR2_EEC_ARD		0x00000200 /* EEPROM Auto Read Done */
#define YUSUR2_EEC_FLUP		0x00800000 /* Flash update command */
#define YUSUR2_EEC_SEC1VAL	0x02000000 /* Sector 1 Valid */
#define YUSUR2_EEC_FLUDONE	0x04000000 /* Flash update done */
/* EEPROM Addressing bits based on type (0-small, 1-large) */
#define YUSUR2_EEC_ADDR_SIZE	0x00000400
#define YUSUR2_EEC_SIZE		0x00007800 /* EEPROM Size */
#define YUSUR2_EERD_MAX_ADDR	0x00003FFF /* EERD alows 14 bits for addr. */

#define YUSUR2_EEC_SIZE_SHIFT		11
#define YUSUR2_EEPROM_WORD_SIZE_SHIFT	6
#define YUSUR2_EEPROM_OPCODE_BITS	8

/* FLA Register */
#define YUSUR2_FLA_LOCKED	0x00000040

/* Part Number String Length */
#define YUSUR2_PBANUM_LENGTH	11

/* Checksum and EEPROM pointers */
#define YUSUR2_PBANUM_PTR_GUARD		0xFAFA
#define YUSUR2_EEPROM_CHECKSUM		0x3F
#define YUSUR2_EEPROM_SUM		0xBABA
#define YUSUR2_EEPROM_CTRL_4		0x45
#define YUSUR2_EE_CTRL_4_INST_ID		0x10
#define YUSUR2_EE_CTRL_4_INST_ID_SHIFT	4
#define YUSUR2_PCIE_ANALOG_PTR		0x03
#define YUSUR2_ATLAS0_CONFIG_PTR		0x04
#define YUSUR2_PHY_PTR			0x04
#define YUSUR2_ATLAS1_CONFIG_PTR		0x05
#define YUSUR2_OPTION_ROM_PTR		0x05
#define YUSUR2_PCIE_GENERAL_PTR		0x06
#define YUSUR2_PCIE_CONFIG0_PTR		0x07
#define YUSUR2_PCIE_CONFIG1_PTR		0x08
#define YUSUR2_CORE0_PTR			0x09
#define YUSUR2_CORE1_PTR			0x0A
#define YUSUR2_MAC0_PTR			0x0B
#define YUSUR2_MAC1_PTR			0x0C
#define YUSUR2_CSR0_CONFIG_PTR		0x0D
#define YUSUR2_CSR1_CONFIG_PTR		0x0E
#define YUSUR2_PCIE_ANALOG_PTR_X550	0x02
#define YUSUR2_SHADOW_RAM_SIZE_X550	0x4000
#define YUSUR2_YUSUR2_PCIE_GENERAL_SIZE	0x24
#define YUSUR2_PCIE_CONFIG_SIZE		0x08
#define YUSUR2_EEPROM_LAST_WORD		0x41
#define YUSUR2_FW_PTR			0x0F
#define YUSUR2_PBANUM0_PTR		0x15
#define YUSUR2_PBANUM1_PTR		0x16
#define YUSUR2_ALT_MAC_ADDR_PTR		0x37
#define YUSUR2_FREE_SPACE_PTR		0X3E

/* External Thermal Sensor Config */
#define YUSUR2_ETS_CFG			0x26
#define YUSUR2_ETS_LTHRES_DELTA_MASK	0x07C0
#define YUSUR2_ETS_LTHRES_DELTA_SHIFT	6
#define YUSUR2_ETS_TYPE_MASK		0x0038
#define YUSUR2_ETS_TYPE_SHIFT		3
#define YUSUR2_ETS_TYPE_EMC		0x000
#define YUSUR2_ETS_NUM_SENSORS_MASK	0x0007
#define YUSUR2_ETS_DATA_LOC_MASK		0x3C00
#define YUSUR2_ETS_DATA_LOC_SHIFT	10
#define YUSUR2_ETS_DATA_INDEX_MASK	0x0300
#define YUSUR2_ETS_DATA_INDEX_SHIFT	8
#define YUSUR2_ETS_DATA_HTHRESH_MASK	0x00FF

#define YUSUR2_SAN_MAC_ADDR_PTR		0x28
#define YUSUR2_DEVICE_CAPS		0x2C
#define YUSUR2_82599_SERIAL_NUMBER_MAC_ADDR	0x11
#define YUSUR2_X550_SERIAL_NUMBER_MAC_ADDR	0x04

#define YUSUR2_PCIE_MSIX_82599_CAPS	0x72
#define YUSUR2_MAX_MSIX_VECTORS_82599	0x40
#define YUSUR2_PCIE_MSIX_82598_CAPS	0x62
#define YUSUR2_MAX_MSIX_VECTORS_82598	0x13

/* MSI-X capability fields masks */
#define YUSUR2_PCIE_MSIX_TBL_SZ_MASK	0x7FF

/* Legacy EEPROM word offsets */
#define YUSUR2_ISCSI_BOOT_CAPS		0x0033
#define YUSUR2_ISCSI_SETUP_PORT_0	0x0030
#define YUSUR2_ISCSI_SETUP_PORT_1	0x0034

/* EEPROM Commands - SPI */
#define YUSUR2_EEPROM_MAX_RETRY_SPI	5000 /* Max wait 5ms for RDY signal */
#define YUSUR2_EEPROM_STATUS_RDY_SPI	0x01
#define YUSUR2_EEPROM_READ_OPCODE_SPI	0x03  /* EEPROM read opcode */
#define YUSUR2_EEPROM_WRITE_OPCODE_SPI	0x02  /* EEPROM write opcode */
#define YUSUR2_EEPROM_A8_OPCODE_SPI	0x08  /* opcode bit-3 = addr bit-8 */
#define YUSUR2_EEPROM_WREN_OPCODE_SPI	0x06  /* EEPROM set Write Ena latch */
/* EEPROM reset Write Enable latch */
#define YUSUR2_EEPROM_WRDI_OPCODE_SPI	0x04
#define YUSUR2_EEPROM_RDSR_OPCODE_SPI	0x05  /* EEPROM read Status reg */
#define YUSUR2_EEPROM_WRSR_OPCODE_SPI	0x01  /* EEPROM write Status reg */
#define YUSUR2_EEPROM_ERASE4K_OPCODE_SPI	0x20  /* EEPROM ERASE 4KB */
#define YUSUR2_EEPROM_ERASE64K_OPCODE_SPI	0xD8  /* EEPROM ERASE 64KB */
#define YUSUR2_EEPROM_ERASE256_OPCODE_SPI	0xDB  /* EEPROM ERASE 256B */

/* EEPROM Read Register */
#define YUSUR2_EEPROM_RW_REG_DATA	16 /* data offset in EEPROM read reg */
#define YUSUR2_EEPROM_RW_REG_DONE	2 /* Offset to READ done bit */
#define YUSUR2_EEPROM_RW_REG_START	1 /* First bit to start operation */
#define YUSUR2_EEPROM_RW_ADDR_SHIFT	2 /* Shift to the address bits */
#define YUSUR2_NVM_POLL_WRITE		1 /* Flag for polling for wr complete */
#define YUSUR2_NVM_POLL_READ		0 /* Flag for polling for rd complete */

#define NVM_INIT_CTRL_3		0x38
#define NVM_INIT_CTRL_3_LPLU	0x8
#define NVM_INIT_CTRL_3_D10GMP_PORT0 0x40
#define NVM_INIT_CTRL_3_D10GMP_PORT1 0x100

#define YUSUR2_ETH_LENGTH_OF_ADDRESS	6

#define YUSUR2_EEPROM_PAGE_SIZE_MAX	128
#define YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT	256 /* words rd in burst */
#define YUSUR2_EEPROM_WR_BUFFER_MAX_COUNT	256 /* words wr in burst */
#define YUSUR2_EEPROM_CTRL_2		1 /* EEPROM CTRL word 2 */
#define YUSUR2_EEPROM_CCD_BIT		2

#ifndef YUSUR2_EEPROM_GRANT_ATTEMPTS
#define YUSUR2_EEPROM_GRANT_ATTEMPTS	1000 /* EEPROM attempts to gain grant */
#endif

/* Number of 5 microseconds we wait for EERD read and
 * EERW write to complete */
#define YUSUR2_EERD_EEWR_ATTEMPTS	100000

/* # attempts we wait for flush update to complete */
#define YUSUR2_FLUDONE_ATTEMPTS		20000

#define YUSUR2_PCIE_CTRL2		0x5   /* PCIe Control 2 Offset */
#define YUSUR2_PCIE_CTRL2_DUMMY_ENABLE	0x8   /* Dummy Function Enable */
#define YUSUR2_PCIE_CTRL2_LAN_DISABLE	0x2   /* LAN PCI Disable */
#define YUSUR2_PCIE_CTRL2_DISABLE_SELECT	0x1   /* LAN Disable Select */

#define YUSUR2_SAN_MAC_ADDR_PORT0_OFFSET		0x0
#define YUSUR2_SAN_MAC_ADDR_PORT1_OFFSET		0x3
#define YUSUR2_DEVICE_CAPS_ALLOW_ANY_SFP		0x1
#define YUSUR2_DEVICE_CAPS_FCOE_OFFLOADS		0x2
#define YUSUR2_DEVICE_CAPS_NO_CROSSTALK_WR	(1 << 7)
#define YUSUR2_FW_LESM_PARAMETERS_PTR		0x2
#define YUSUR2_FW_LESM_STATE_1			0x1
#define YUSUR2_FW_LESM_STATE_ENABLED		0x8000 /* LESM Enable bit */
#define YUSUR2_FW_LESM_2_STATES_ENABLED_MASK	0x1F
#define YUSUR2_FW_LESM_2_STATES_ENABLED		0x12
#define YUSUR2_FW_LESM_STATE0_10G_ENABLED	0x6FFF
#define YUSUR2_FW_LESM_STATE1_10G_ENABLED	0x4FFF
#define YUSUR2_FW_LESM_STATE0_10G_DISABLED	0x0FFF
#define YUSUR2_FW_LESM_STATE1_10G_DISABLED	0x2FFF
#define YUSUR2_FW_LESM_PORT0_STATE0_OFFSET	0x2
#define YUSUR2_FW_LESM_PORT0_STATE1_OFFSET	0x3
#define YUSUR2_FW_LESM_PORT1_STATE0_OFFSET	0x6
#define YUSUR2_FW_LESM_PORT1_STATE1_OFFSET	0x7
#define YUSUR2_FW_PASSTHROUGH_PATCH_CONFIG_PTR	0x4
#define YUSUR2_FW_PATCH_VERSION_4		0x7
#define YUSUR2_FCOE_IBA_CAPS_BLK_PTR		0x33 /* iSCSI/FCOE block */
#define YUSUR2_FCOE_IBA_CAPS_FCOE		0x20 /* FCOE flags */
#define YUSUR2_ISCSI_FCOE_BLK_PTR		0x17 /* iSCSI/FCOE block */
#define YUSUR2_ISCSI_FCOE_FLAGS_OFFSET		0x0 /* FCOE flags */
#define YUSUR2_ISCSI_FCOE_FLAGS_ENABLE		0x1 /* FCOE flags enable bit */
#define YUSUR2_ALT_SAN_MAC_ADDR_BLK_PTR		0x27 /* Alt. SAN MAC block */
#define YUSUR2_ALT_SAN_MAC_ADDR_CAPS_OFFSET	0x0 /* Alt SAN MAC capability */
#define YUSUR2_ALT_SAN_MAC_ADDR_PORT0_OFFSET	0x1 /* Alt SAN MAC 0 offset */
#define YUSUR2_ALT_SAN_MAC_ADDR_PORT1_OFFSET	0x4 /* Alt SAN MAC 1 offset */
#define YUSUR2_ALT_SAN_MAC_ADDR_WWNN_OFFSET	0x7 /* Alt WWNN prefix offset */
#define YUSUR2_ALT_SAN_MAC_ADDR_WWPN_OFFSET	0x8 /* Alt WWPN prefix offset */
#define YUSUR2_ALT_SAN_MAC_ADDR_CAPS_SANMAC	0x0 /* Alt SAN MAC exists */
#define YUSUR2_ALT_SAN_MAC_ADDR_CAPS_ALTWWN	0x1 /* Alt WWN base exists */

/* FW header offset */
#define YUSUR2_X540_FW_PASSTHROUGH_PATCH_CONFIG_PTR	0x4
#define YUSUR2_X540_FW_MODULE_MASK			0x7FFF
/* 4KB multiplier */
#define YUSUR2_X540_FW_MODULE_LENGTH			0x1000
/* version word 2 (month & day) */
#define YUSUR2_X540_FW_PATCH_VERSION_2		0x5
/* version word 3 (silicon compatibility & year) */
#define YUSUR2_X540_FW_PATCH_VERSION_3		0x6
/* version word 4 (major & minor numbers) */
#define YUSUR2_X540_FW_PATCH_VERSION_4		0x7

#define YUSUR2_DEVICE_CAPS_WOL_PORT0_1	0x4 /* WoL supported on ports 0 & 1 */
#define YUSUR2_DEVICE_CAPS_WOL_PORT0	0x8 /* WoL supported on port 0 */
#define YUSUR2_DEVICE_CAPS_WOL_MASK	0xC /* Mask for WoL capabilities */

/* PCI Bus Info */
#define YUSUR2_PCI_DEVICE_STATUS		0xAA
#define YUSUR2_PCI_DEVICE_STATUS_TRANSACTION_PENDING	0x0020
#define YUSUR2_PCI_LINK_STATUS		0xB2
#define YUSUR2_PCI_DEVICE_CONTROL2	0xC8
#define YUSUR2_PCI_LINK_WIDTH		0x3F0
#define YUSUR2_PCI_LINK_WIDTH_1		0x10
#define YUSUR2_PCI_LINK_WIDTH_2		0x20
#define YUSUR2_PCI_LINK_WIDTH_4		0x40
#define YUSUR2_PCI_LINK_WIDTH_8		0x80
#define YUSUR2_PCI_LINK_SPEED		0xF
#define YUSUR2_PCI_LINK_SPEED_2500	0x1
#define YUSUR2_PCI_LINK_SPEED_5000	0x2
#define YUSUR2_PCI_LINK_SPEED_8000	0x3
#define YUSUR2_PCI_HEADER_TYPE_REGISTER	0x0E
#define YUSUR2_PCI_HEADER_TYPE_MULTIFUNC	0x80
#define YUSUR2_PCI_DEVICE_CONTROL2_16ms	0x0005

#define YUSUR2_PCIDEVCTRL2_TIMEO_MASK	0xf
#define YUSUR2_PCIDEVCTRL2_16_32ms_def	0x0
#define YUSUR2_PCIDEVCTRL2_50_100us	0x1
#define YUSUR2_PCIDEVCTRL2_1_2ms		0x2
#define YUSUR2_PCIDEVCTRL2_16_32ms	0x5
#define YUSUR2_PCIDEVCTRL2_65_130ms	0x6
#define YUSUR2_PCIDEVCTRL2_260_520ms	0x9
#define YUSUR2_PCIDEVCTRL2_1_2s		0xa
#define YUSUR2_PCIDEVCTRL2_4_8s		0xd
#define YUSUR2_PCIDEVCTRL2_17_34s	0xe

/* Number of 100 microseconds we wait for PCI Express master disable */
#define YUSUR2_PCI_MASTER_DISABLE_TIMEOUT	800

/* Check whether address is multicast. This is little-endian specific check.*/
#define YUSUR2_IS_MULTICAST(Address) \
		(bool)(((u8 *)(Address))[0] & ((u8)0x01))

/* Check whether an address is broadcast. */
#define YUSUR2_IS_BROADCAST(Address) \
		((((u8 *)(Address))[0] == ((u8)0xff)) && \
		(((u8 *)(Address))[1] == ((u8)0xff)))

/* RAH */
#define YUSUR2_RAH_VIND_MASK	0x003C0000
#define YUSUR2_RAH_VIND_SHIFT	18
#define YUSUR2_RAH_AV		0x80000000
#define YUSUR2_CLEAR_VMDQ_ALL	0xFFFFFFFF

/* Header split receive */
#define YUSUR2_RFCTL_ISCSI_DIS		0x00000001
#define YUSUR2_RFCTL_ISCSI_DWC_MASK	0x0000003E
#define YUSUR2_RFCTL_ISCSI_DWC_SHIFT	1
#define YUSUR2_RFCTL_RSC_DIS		0x00000020
#define YUSUR2_RFCTL_NFSW_DIS		0x00000040
#define YUSUR2_RFCTL_NFSR_DIS		0x00000080
#define YUSUR2_RFCTL_NFS_VER_MASK	0x00000300
#define YUSUR2_RFCTL_NFS_VER_SHIFT	8
#define YUSUR2_RFCTL_NFS_VER_2		0
#define YUSUR2_RFCTL_NFS_VER_3		1
#define YUSUR2_RFCTL_NFS_VER_4		2
#define YUSUR2_RFCTL_IPV6_DIS		0x00000400
#define YUSUR2_RFCTL_IPV6_XSUM_DIS	0x00000800
#define YUSUR2_RFCTL_IPFRSP_DIS		0x00004000
#define YUSUR2_RFCTL_IPV6_EX_DIS		0x00010000
#define YUSUR2_RFCTL_NEW_IPV6_EXT_DIS	0x00020000

/* Transmit Config masks */
#define YUSUR2_TXDCTL_ENABLE		0x02000000 /* Ena specific Tx Queue */
#define YUSUR2_TXDCTL_SWFLSH		0x04000000 /* Tx Desc. wr-bk flushing */
#define YUSUR2_TXDCTL_WTHRESH_SHIFT	16 /* shift to WTHRESH bits */
/* Enable short packet padding to 64 bytes */
#define YUSUR2_TX_PAD_ENABLE		0x00000400
#define YUSUR2_JUMBO_FRAME_ENABLE	0x00000004  /* Allow jumbo frames */
/* This allows for 16K packets + 4k for vlan */
#define YUSUR2_MAX_FRAME_SZ		0x40040000

#define YUSUR2_TDWBAL_HEAD_WB_ENABLE	0x1 /* Tx head write-back enable */
#define YUSUR2_TDWBAL_SEQNUM_WB_ENABLE	0x2 /* Tx seq# write-back enable */

/* Receive Config masks */
#define YUSUR2_RXCTRL_RXEN		0x00000001 /* Enable Receiver */
#define YUSUR2_RXCTRL_DMBYPS		0x00000002 /* Desc Monitor Bypass */
#define YUSUR2_RXDCTL_ENABLE		0x02000000 /* Ena specific Rx Queue */
#define YUSUR2_RXDCTL_SWFLSH		0x04000000 /* Rx Desc wr-bk flushing */
#define YUSUR2_RXDCTL_RLPMLMASK		0x00003FFF /* X540 supported only */
#define YUSUR2_RXDCTL_RLPML_EN		0x00008000
#define YUSUR2_RXDCTL_VME		0x40000000 /* VLAN mode enable */

#define YUSUR2_TSAUXC_EN_CLK		0x00000004
#define YUSUR2_TSAUXC_SYNCLK		0x00000008
#define YUSUR2_TSAUXC_SDP0_INT		0x00000040
#define YUSUR2_TSAUXC_EN_TT0		0x00000001
#define YUSUR2_TSAUXC_EN_TT1		0x00000002
#define YUSUR2_TSAUXC_ST0		0x00000010
#define YUSUR2_TSAUXC_DISABLE_SYSTIME	0x80000000

#define YUSUR2_TSSDP_TS_SDP0_SEL_MASK	0x000000C0
#define YUSUR2_TSSDP_TS_SDP0_CLK0	0x00000080
#define YUSUR2_TSSDP_TS_SDP0_EN		0x00000100

#define YUSUR2_TSYNCTXCTL_VALID		0x00000001 /* Tx timestamp valid */
#define YUSUR2_TSYNCTXCTL_ENABLED	0x00000010 /* Tx timestamping enabled */

#define YUSUR2_TSYNCRXCTL_VALID		0x00000001 /* Rx timestamp valid */
#define YUSUR2_TSYNCRXCTL_TYPE_MASK	0x0000000E /* Rx type mask */
#define YUSUR2_TSYNCRXCTL_TYPE_L2_V2	0x00
#define YUSUR2_TSYNCRXCTL_TYPE_L4_V1	0x02
#define YUSUR2_TSYNCRXCTL_TYPE_L2_L4_V2	0x04
#define YUSUR2_TSYNCRXCTL_TYPE_ALL	0x08
#define YUSUR2_TSYNCRXCTL_TYPE_EVENT_V2	0x0A
#define YUSUR2_TSYNCRXCTL_ENABLED	0x00000010 /* Rx Timestamping enabled */
#define YUSUR2_TSYNCRXCTL_TSIP_UT_EN	0x00800000 /* Rx Timestamp in Packet */
#define YUSUR2_TSYNCRXCTL_TSIP_UP_MASK	0xFF000000 /* Rx Timestamp UP Mask */

#define YUSUR2_TSIM_SYS_WRAP		0x00000001
#define YUSUR2_TSIM_TXTS			0x00000002
#define YUSUR2_TSIM_TADJ			0x00000080

#define YUSUR2_TSICR_SYS_WRAP		YUSUR2_TSIM_SYS_WRAP
#define YUSUR2_TSICR_TXTS		YUSUR2_TSIM_TXTS
#define YUSUR2_TSICR_TADJ		YUSUR2_TSIM_TADJ

#define YUSUR2_RXMTRL_V1_CTRLT_MASK	0x000000FF
#define YUSUR2_RXMTRL_V1_SYNC_MSG	0x00
#define YUSUR2_RXMTRL_V1_DELAY_REQ_MSG	0x01
#define YUSUR2_RXMTRL_V1_FOLLOWUP_MSG	0x02
#define YUSUR2_RXMTRL_V1_DELAY_RESP_MSG	0x03
#define YUSUR2_RXMTRL_V1_MGMT_MSG	0x04

#define YUSUR2_RXMTRL_V2_MSGID_MASK	0x0000FF00
#define YUSUR2_RXMTRL_V2_SYNC_MSG	0x0000
#define YUSUR2_RXMTRL_V2_DELAY_REQ_MSG	0x0100
#define YUSUR2_RXMTRL_V2_PDELAY_REQ_MSG	0x0200
#define YUSUR2_RXMTRL_V2_PDELAY_RESP_MSG	0x0300
#define YUSUR2_RXMTRL_V2_FOLLOWUP_MSG	0x0800
#define YUSUR2_RXMTRL_V2_DELAY_RESP_MSG	0x0900
#define YUSUR2_RXMTRL_V2_PDELAY_FOLLOWUP_MSG 0x0A00
#define YUSUR2_RXMTRL_V2_ANNOUNCE_MSG	0x0B00
#define YUSUR2_RXMTRL_V2_SIGNALLING_MSG	0x0C00
#define YUSUR2_RXMTRL_V2_MGMT_MSG	0x0D00

#define YUSUR2_FCTRL_SBP		0x00000002 /* Store Bad Packet */
#define YUSUR2_FCTRL_MPE		0x00000100 /* Multicast Promiscuous Ena*/
#define YUSUR2_FCTRL_UPE		0x00000200 /* Unicast Promiscuous Ena */
#define YUSUR2_FCTRL_BAM		0x00000400 /* Broadcast Accept Mode */
#define YUSUR2_FCTRL_PMCF	0x00001000 /* Pass MAC Control Frames */
#define YUSUR2_FCTRL_DPF		0x00002000 /* Discard Pause Frame */
/* Receive Priority Flow Control Enable */
#define YUSUR2_FCTRL_RPFCE	0x00004000
#define YUSUR2_FCTRL_RFCE	0x00008000 /* Receive Flow Control Ena */
#define YUSUR2_MFLCN_PMCF	0x00000001 /* Pass MAC Control Frames */
#define YUSUR2_MFLCN_DPF		0x00000002 /* Discard Pause Frame */
#define YUSUR2_MFLCN_RPFCE	0x00000004 /* Receive Priority FC Enable */
#define YUSUR2_MFLCN_RFCE	0x00000008 /* Receive FC Enable */
#define YUSUR2_MFLCN_RPFCE_MASK	0x00000FF4 /* Rx Priority FC bitmap mask */
#define YUSUR2_MFLCN_RPFCE_SHIFT	4 /* Rx Priority FC bitmap shift */

/* Multiple Receive Queue Control */
#define YUSUR2_MRQC_RSSEN	0x00000001  /* RSS Enable */
#define YUSUR2_MRQC_MRQE_MASK	0xF /* Bits 3:0 */
#define YUSUR2_MRQC_RT8TCEN	0x00000002 /* 8 TC no RSS */
#define YUSUR2_MRQC_RT4TCEN	0x00000003 /* 4 TC no RSS */
#define YUSUR2_MRQC_RTRSS8TCEN	0x00000004 /* 8 TC w/ RSS */
#define YUSUR2_MRQC_RTRSS4TCEN	0x00000005 /* 4 TC w/ RSS */
#define YUSUR2_MRQC_VMDQEN	0x00000008 /* VMDq2 64 pools no RSS */
#define YUSUR2_MRQC_VMDQRSS32EN	0x0000000A /* VMDq2 32 pools w/ RSS */
#define YUSUR2_MRQC_VMDQRSS64EN	0x0000000B /* VMDq2 64 pools w/ RSS */
#define YUSUR2_MRQC_VMDQRT8TCEN	0x0000000C /* VMDq2/RT 16 pool 8 TC */
#define YUSUR2_MRQC_VMDQRT4TCEN	0x0000000D /* VMDq2/RT 32 pool 4 TC */
#define YUSUR2_MRQC_L3L4TXSWEN	0x00008000 /* Enable L3/L4 Tx switch */
#define YUSUR2_MRQC_RSS_FIELD_MASK	0xFFFF0000
#define YUSUR2_MRQC_RSS_FIELD_IPV4_TCP	0x00010000
#define YUSUR2_MRQC_RSS_FIELD_IPV4	0x00020000
#define YUSUR2_MRQC_RSS_FIELD_IPV6_EX_TCP 0x00040000
#define YUSUR2_MRQC_RSS_FIELD_IPV6_EX	0x00080000
#define YUSUR2_MRQC_RSS_FIELD_IPV6	0x00100000
#define YUSUR2_MRQC_RSS_FIELD_IPV6_TCP	0x00200000
#define YUSUR2_MRQC_RSS_FIELD_IPV4_UDP	0x00400000
#define YUSUR2_MRQC_RSS_FIELD_IPV6_UDP	0x00800000
#define YUSUR2_MRQC_RSS_FIELD_IPV6_EX_UDP 0x01000000
#define YUSUR2_MRQC_MULTIPLE_RSS		0x00002000
#define YUSUR2_MRQC_L3L4TXSWEN		0x00008000

/* Queue Drop Enable */
#define YUSUR2_QDE_ENABLE	0x00000001
#define YUSUR2_QDE_HIDE_VLAN	0x00000002
#define YUSUR2_QDE_IDX_MASK	0x00007F00
#define YUSUR2_QDE_IDX_SHIFT	8
#define YUSUR2_QDE_WRITE		0x00010000
#define YUSUR2_QDE_READ		0x00020000

#define YUSUR2_TXD_POPTS_IXSM	0x01 /* Insert IP checksum */
#define YUSUR2_TXD_POPTS_TXSM	0x02 /* Insert TCP/UDP checksum */
#define YUSUR2_TXD_CMD_EOP	0x01000000 /* End of Packet */
#define YUSUR2_TXD_CMD_IFCS	0x02000000 /* Insert FCS (Ethernet CRC) */
#define YUSUR2_TXD_CMD_IC	0x04000000 /* Insert Checksum */
#define YUSUR2_TXD_CMD_RS	0x08000000 /* Report Status */
#define YUSUR2_TXD_CMD_DEXT	0x20000000 /* Desc extension (0 = legacy) */
#define YUSUR2_TXD_CMD_VLE	0x40000000 /* Add VLAN tag */
#define YUSUR2_TXD_STAT_DD	0x00000001 /* Descriptor Done */

#define YUSUR2_RXDADV_IPSEC_STATUS_SECP		0x00020000
#define YUSUR2_RXDADV_IPSEC_ERROR_INVALID_PROTOCOL 0x08000000
#define YUSUR2_RXDADV_IPSEC_ERROR_INVALID_LENGTH	0x10000000
#define YUSUR2_RXDADV_IPSEC_ERROR_AUTH_FAILED	0x18000000
#define YUSUR2_RXDADV_IPSEC_ERROR_BIT_MASK	0x18000000
/* Multiple Transmit Queue Command Register */
#define YUSUR2_MTQC_RT_ENA	0x1 /* DCB Enable */
#define YUSUR2_MTQC_VT_ENA	0x2 /* VMDQ2 Enable */
#define YUSUR2_MTQC_64Q_1PB	0x0 /* 64 queues 1 pack buffer */
#define YUSUR2_MTQC_32VF		0x8 /* 4 TX Queues per pool w/32VF's */
#define YUSUR2_MTQC_64VF		0x4 /* 2 TX Queues per pool w/64VF's */
#define YUSUR2_MTQC_4TC_4TQ	0x8 /* 4 TC if RT_ENA and VT_ENA */
#define YUSUR2_MTQC_8TC_8TQ	0xC /* 8 TC if RT_ENA or 8 TQ if VT_ENA */

/* Receive Descriptor bit definitions */
#define YUSUR2_RXD_STAT_DD	0x01 /* Descriptor Done */
#define YUSUR2_RXD_STAT_EOP	0x02 /* End of Packet */
#define YUSUR2_RXD_STAT_FLM	0x04 /* FDir Match */
#define YUSUR2_RXD_STAT_VP	0x08 /* IEEE VLAN Packet */
#define YUSUR2_RXDADV_NEXTP_MASK	0x000FFFF0 /* Next Descriptor Index */
#define YUSUR2_RXDADV_NEXTP_SHIFT	0x00000004
#define YUSUR2_RXD_STAT_UDPCS	0x10 /* UDP xsum calculated */
#define YUSUR2_RXD_STAT_L4CS	0x20 /* L4 xsum calculated */
#define YUSUR2_RXD_STAT_IPCS	0x40 /* IP xsum calculated */
#define YUSUR2_RXD_STAT_PIF	0x80 /* passed in-exact filter */
#define YUSUR2_RXD_STAT_CRCV	0x100 /* Speculative CRC Valid */
#define YUSUR2_RXD_STAT_OUTERIPCS	0x100 /* Cloud IP xsum calculated */
#define YUSUR2_RXD_STAT_VEXT	0x200 /* 1st VLAN found */
#define YUSUR2_RXD_STAT_UDPV	0x400 /* Valid UDP checksum */
#define YUSUR2_RXD_STAT_DYNINT	0x800 /* Pkt caused INT via DYNINT */
#define YUSUR2_RXD_STAT_LLINT	0x800 /* Pkt caused Low Latency Interrupt */
#define YUSUR2_RXD_STAT_TSIP	0x08000 /* Time Stamp in packet buffer */
#define YUSUR2_RXD_STAT_TS	0x10000 /* Time Stamp */
#define YUSUR2_RXD_STAT_SECP	0x20000 /* Security Processing */
#define YUSUR2_RXD_STAT_LB	0x40000 /* Loopback Status */
#define YUSUR2_RXD_STAT_ACK	0x8000 /* ACK Packet indication */
#define YUSUR2_RXD_ERR_CE	0x01 /* CRC Error */
#define YUSUR2_RXD_ERR_LE	0x02 /* Length Error */
#define YUSUR2_RXD_ERR_PE	0x08 /* Packet Error */
#define YUSUR2_RXD_ERR_OSE	0x10 /* Oversize Error */
#define YUSUR2_RXD_ERR_USE	0x20 /* Undersize Error */
#define YUSUR2_RXD_ERR_TCPE	0x40 /* TCP/UDP Checksum Error */
#define YUSUR2_RXD_ERR_IPE	0x80 /* IP Checksum Error */
#define YUSUR2_RXDADV_ERR_MASK		0xfff00000 /* RDESC.ERRORS mask */
#define YUSUR2_RXDADV_ERR_SHIFT		20 /* RDESC.ERRORS shift */
#define YUSUR2_RXDADV_ERR_OUTERIPER	0x04000000 /* CRC IP Header error */
#define YUSUR2_RXDADV_ERR_RXE		0x20000000 /* Any MAC Error */
#define YUSUR2_RXDADV_ERR_FCEOFE		0x80000000 /* FCEOFe/IPE */
#define YUSUR2_RXDADV_ERR_FCERR		0x00700000 /* FCERR/FDIRERR */
#define YUSUR2_RXDADV_ERR_FDIR_LEN	0x00100000 /* FDIR Length error */
#define YUSUR2_RXDADV_ERR_FDIR_DROP	0x00200000 /* FDIR Drop error */
#define YUSUR2_RXDADV_ERR_FDIR_COLL	0x00400000 /* FDIR Collision error */
#define YUSUR2_RXDADV_ERR_HBO	0x00800000 /*Header Buffer Overflow */
#define YUSUR2_RXDADV_ERR_CE	0x01000000 /* CRC Error */
#define YUSUR2_RXDADV_ERR_LE	0x02000000 /* Length Error */
#define YUSUR2_RXDADV_ERR_PE	0x08000000 /* Packet Error */
#define YUSUR2_RXDADV_ERR_OSE	0x10000000 /* Oversize Error */
#define YUSUR2_RXDADV_ERR_USE	0x20000000 /* Undersize Error */
#define YUSUR2_RXDADV_ERR_TCPE	0x40000000 /* TCP/UDP Checksum Error */
#define YUSUR2_RXDADV_ERR_IPE	0x80000000 /* IP Checksum Error */
#define YUSUR2_RXD_VLAN_ID_MASK	0x0FFF  /* VLAN ID is in lower 12 bits */
#define YUSUR2_RXD_PRI_MASK	0xE000  /* Priority is in upper 3 bits */
#define YUSUR2_RXD_PRI_SHIFT	13
#define YUSUR2_RXD_CFI_MASK	0x1000  /* CFI is bit 12 */
#define YUSUR2_RXD_CFI_SHIFT	12

#define YUSUR2_RXDADV_STAT_DD		YUSUR2_RXD_STAT_DD  /* Done */
#define YUSUR2_RXDADV_STAT_EOP		YUSUR2_RXD_STAT_EOP /* End of Packet */
#define YUSUR2_RXDADV_STAT_FLM		YUSUR2_RXD_STAT_FLM /* FDir Match */
#define YUSUR2_RXDADV_STAT_VP		YUSUR2_RXD_STAT_VP  /* IEEE VLAN Pkt */
#define YUSUR2_RXDADV_STAT_MASK		0x000fffff /* Stat/NEXTP: bit 0-19 */
#define YUSUR2_RXDADV_STAT_FCEOFS	0x00000040 /* FCoE EOF/SOF Stat */
#define YUSUR2_RXDADV_STAT_FCSTAT	0x00000030 /* FCoE Pkt Stat */
#define YUSUR2_RXDADV_STAT_FCSTAT_NOMTCH	0x00000000 /* 00: No Ctxt Match */
#define YUSUR2_RXDADV_STAT_FCSTAT_NODDP	0x00000010 /* 01: Ctxt w/o DDP */
#define YUSUR2_RXDADV_STAT_FCSTAT_FCPRSP	0x00000020 /* 10: Recv. FCP_RSP */
#define YUSUR2_RXDADV_STAT_FCSTAT_DDP	0x00000030 /* 11: Ctxt w/ DDP */
#define YUSUR2_RXDADV_STAT_TS		0x00010000 /* IEEE1588 Time Stamp */
#define YUSUR2_RXDADV_STAT_TSIP		0x00008000 /* Time Stamp in packet buffer */

/* PSRTYPE bit definitions */
#define YUSUR2_PSRTYPE_TCPHDR	0x00000010
#define YUSUR2_PSRTYPE_UDPHDR	0x00000020
#define YUSUR2_PSRTYPE_IPV4HDR	0x00000100
#define YUSUR2_PSRTYPE_IPV6HDR	0x00000200
#define YUSUR2_PSRTYPE_L2HDR	0x00001000

/* SRRCTL bit definitions */
#define YUSUR2_SRRCTL_BSIZEPKT_SHIFT	10 /* so many KBs */
#define YUSUR2_SRRCTL_BSIZEHDRSIZE_SHIFT	2 /* 64byte resolution (>> 6)
					   * + at bit 8 offset (<< 8)
					   *  = (<< 2)
					   */
#define YUSUR2_SRRCTL_RDMTS_SHIFT	22
#define YUSUR2_SRRCTL_RDMTS_MASK		0x01C00000
#define YUSUR2_SRRCTL_DROP_EN		0x10000000
#define YUSUR2_SRRCTL_BSIZEPKT_MASK	0x0000007F
#define YUSUR2_SRRCTL_BSIZEHDR_MASK	0x00003F00
#define YUSUR2_SRRCTL_DESCTYPE_LEGACY	0x00000000
#define YUSUR2_SRRCTL_DESCTYPE_ADV_ONEBUF 0x02000000
#define YUSUR2_SRRCTL_DESCTYPE_HDR_SPLIT	0x04000000
#define YUSUR2_SRRCTL_DESCTYPE_HDR_REPLICATION_LARGE_PKT 0x08000000
#define YUSUR2_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS 0x0A000000
#define YUSUR2_SRRCTL_DESCTYPE_MASK	0x0E000000

#define YUSUR2_RXDPS_HDRSTAT_HDRSP	0x00008000
#define YUSUR2_RXDPS_HDRSTAT_HDRLEN_MASK	0x000003FF

#define YUSUR2_RXDADV_RSSTYPE_MASK	0x0000000F
#define YUSUR2_RXDADV_PKTTYPE_MASK	0x0000FFF0
#define YUSUR2_RXDADV_PKTTYPE_MASK_EX	0x0001FFF0
#define YUSUR2_RXDADV_HDRBUFLEN_MASK	0x00007FE0
#define YUSUR2_RXDADV_RSCCNT_MASK	0x001E0000
#define YUSUR2_RXDADV_RSCCNT_SHIFT	17
#define YUSUR2_RXDADV_HDRBUFLEN_SHIFT	5
#define YUSUR2_RXDADV_SPLITHEADER_EN	0x00001000
#define YUSUR2_RXDADV_SPH		0x8000

/* RSS Hash results */
#define YUSUR2_RXDADV_RSSTYPE_NONE	0x00000000
#define YUSUR2_RXDADV_RSSTYPE_IPV4_TCP	0x00000001
#define YUSUR2_RXDADV_RSSTYPE_IPV4	0x00000002
#define YUSUR2_RXDADV_RSSTYPE_IPV6_TCP	0x00000003
#define YUSUR2_RXDADV_RSSTYPE_IPV6_EX	0x00000004
#define YUSUR2_RXDADV_RSSTYPE_IPV6	0x00000005
#define YUSUR2_RXDADV_RSSTYPE_IPV6_TCP_EX 0x00000006
#define YUSUR2_RXDADV_RSSTYPE_IPV4_UDP	0x00000007
#define YUSUR2_RXDADV_RSSTYPE_IPV6_UDP	0x00000008
#define YUSUR2_RXDADV_RSSTYPE_IPV6_UDP_EX 0x00000009

/* RSS Packet Types as indicated in the receive descriptor. */
#define YUSUR2_RXDADV_PKTTYPE_NONE	0x00000000
#define YUSUR2_RXDADV_PKTTYPE_IPV4	0x00000010 /* IPv4 hdr present */
#define YUSUR2_RXDADV_PKTTYPE_IPV4_EX	0x00000020 /* IPv4 hdr + extensions */
#define YUSUR2_RXDADV_PKTTYPE_IPV6	0x00000040 /* IPv6 hdr present */
#define YUSUR2_RXDADV_PKTTYPE_IPV6_EX	0x00000080 /* IPv6 hdr + extensions */
#define YUSUR2_RXDADV_PKTTYPE_TCP	0x00000100 /* TCP hdr present */
#define YUSUR2_RXDADV_PKTTYPE_UDP	0x00000200 /* UDP hdr present */
#define YUSUR2_RXDADV_PKTTYPE_SCTP	0x00000400 /* SCTP hdr present */
#define YUSUR2_RXDADV_PKTTYPE_NFS	0x00000800 /* NFS hdr present */
#define YUSUR2_RXDADV_PKTTYPE_GENEVE	0x00000800 /* GENEVE hdr present */
#define YUSUR2_RXDADV_PKTTYPE_VXLAN	0x00000800 /* VXLAN hdr present */
#define YUSUR2_RXDADV_PKTTYPE_TUNNEL	0x00010000 /* Tunnel type */
#define YUSUR2_RXDADV_PKTTYPE_IPSEC_ESP	0x00001000 /* IPSec ESP */
#define YUSUR2_RXDADV_PKTTYPE_IPSEC_AH	0x00002000 /* IPSec AH */
#define YUSUR2_RXDADV_PKTTYPE_LINKSEC	0x00004000 /* LinkSec Encap */
#define YUSUR2_RXDADV_PKTTYPE_ETQF	0x00008000 /* PKTTYPE is ETQF index */
#define YUSUR2_RXDADV_PKTTYPE_ETQF_MASK	0x00000070 /* ETQF has 8 indices */
#define YUSUR2_RXDADV_PKTTYPE_ETQF_SHIFT	4 /* Right-shift 4 bits */

/* Security Processing bit Indication */
#define YUSUR2_RXDADV_LNKSEC_STATUS_SECP		0x00020000
#define YUSUR2_RXDADV_LNKSEC_ERROR_NO_SA_MATCH	0x08000000
#define YUSUR2_RXDADV_LNKSEC_ERROR_REPLAY_ERROR	0x10000000
#define YUSUR2_RXDADV_LNKSEC_ERROR_BIT_MASK	0x18000000
#define YUSUR2_RXDADV_LNKSEC_ERROR_BAD_SIG	0x18000000

/* Masks to determine if packets should be dropped due to frame errors */
#define YUSUR2_RXD_ERR_FRAME_ERR_MASK ( \
				YUSUR2_RXD_ERR_CE | \
				YUSUR2_RXD_ERR_LE | \
				YUSUR2_RXD_ERR_PE | \
				YUSUR2_RXD_ERR_OSE | \
				YUSUR2_RXD_ERR_USE)

#define YUSUR2_RXDADV_ERR_FRAME_ERR_MASK ( \
				YUSUR2_RXDADV_ERR_CE | \
				YUSUR2_RXDADV_ERR_LE | \
				YUSUR2_RXDADV_ERR_PE | \
				YUSUR2_RXDADV_ERR_OSE | \
				YUSUR2_RXDADV_ERR_USE)

#define YUSUR2_RXDADV_ERR_FRAME_ERR_MASK_82599	YUSUR2_RXDADV_ERR_RXE

/* Multicast bit mask */
#define YUSUR2_MCSTCTRL_MFE	0x4

/* Number of Transmit and Receive Descriptors must be a multiple of 8 */
#define YUSUR2_REQ_TX_DESCRIPTOR_MULTIPLE	8
#define YUSUR2_REQ_RX_DESCRIPTOR_MULTIPLE	8
#define YUSUR2_REQ_TX_BUFFER_GRANULARITY		1024

/* Vlan-specific macros */
#define YUSUR2_RX_DESC_SPECIAL_VLAN_MASK	0x0FFF /* VLAN ID in lower 12 bits */
#define YUSUR2_RX_DESC_SPECIAL_PRI_MASK	0xE000 /* Priority in upper 3 bits */
#define YUSUR2_RX_DESC_SPECIAL_PRI_SHIFT	0x000D /* Priority in upper 3 of 16 */
#define YUSUR2_TX_DESC_SPECIAL_PRI_SHIFT	YUSUR2_RX_DESC_SPECIAL_PRI_SHIFT

/* SR-IOV specific macros */
#define YUSUR2_MBVFICR_INDEX(vf_number)	(vf_number >> 4)
#define YUSUR2_MBVFICR(_i)		(0x00710 + ((_i) * 4))
#define YUSUR2_VFLRE(_i)			(((_i & 1) ? 0x001C0 : 0x00600))
#define YUSUR2_VFLREC(_i)		 (0x00700 + ((_i) * 4))
/* Translated register #defines */
#define YUSUR2_PVFCTRL(P)	(0x00300 + (4 * (P)))
#define YUSUR2_PVFSTATUS(P)	(0x00008 + (0 * (P)))
#define YUSUR2_PVFLINKS(P)	(0x042A4 + (0 * (P)))
#define YUSUR2_PVFRTIMER(P)	(0x00048 + (0 * (P)))
#define YUSUR2_PVFMAILBOX(P)	(0x04C00 + (4 * (P)))
#define YUSUR2_PVFRXMEMWRAP(P)	(0x03190 + (0 * (P)))
#define YUSUR2_PVTEICR(P)	(0x00B00 + (4 * (P)))
#define YUSUR2_PVTEICS(P)	(0x00C00 + (4 * (P)))
#define YUSUR2_PVTEIMS(P)	(0x00D00 + (4 * (P)))
#define YUSUR2_PVTEIMC(P)	(0x00E00 + (4 * (P)))
#define YUSUR2_PVTEIAC(P)	(0x00F00 + (4 * (P)))
#define YUSUR2_PVTEIAM(P)	(0x04D00 + (4 * (P)))
#define YUSUR2_PVTEITR(P)	(((P) < 24) ? (0x00820 + ((P) * 4)) : \
				 (0x012300 + (((P) - 24) * 4)))
#define YUSUR2_PVTIVAR(P)	(0x12500 + (4 * (P)))
#define YUSUR2_PVTIVAR_MISC(P)	(0x04E00 + (4 * (P)))
#define YUSUR2_PVTRSCINT(P)	(0x12000 + (4 * (P)))
#define YUSUR2_VFPBACL(P)	(0x110C8 + (4 * (P)))
#define YUSUR2_PVFRDBAL(P)	((P < 64) ? (0x01000 + (0x40 * (P))) \
				 : (0x0D000 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFRDBAH(P)	((P < 64) ? (0x01004 + (0x40 * (P))) \
				 : (0x0D004 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFRDLEN(P)	((P < 64) ? (0x01008 + (0x40 * (P))) \
				 : (0x0D008 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFRDH(P)		((P < 64) ? (0x01010 + (0x40 * (P))) \
				 : (0x0D010 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFRDT(P)		((P < 64) ? (0x01018 + (0x40 * (P))) \
				 : (0x0D018 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFRXDCTL(P)	((P < 64) ? (0x01028 + (0x40 * (P))) \
				 : (0x0D028 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFSRRCTL(P)	((P < 64) ? (0x01014 + (0x40 * (P))) \
				 : (0x0D014 + (0x40 * ((P) - 64))))
#define YUSUR2_PVFPSRTYPE(P)	(0x0EA00 + (4 * (P)))
#define YUSUR2_PVFTDBAL(P)	(0x06000 + (0x40 * (P)))
#define YUSUR2_PVFTDBAH(P)	(0x06004 + (0x40 * (P)))
#define YUSUR2_PVFTDLEN(P)	(0x06008 + (0x40 * (P)))
#define YUSUR2_PVFTDH(P)		(0x06010 + (0x40 * (P)))
#define YUSUR2_PVFTDT(P)		(0x06018 + (0x40 * (P)))
#define YUSUR2_PVFTXDCTL(P)	(0x06028 + (0x40 * (P)))
#define YUSUR2_PVFTDWBAL(P)	(0x06038 + (0x40 * (P)))
#define YUSUR2_PVFTDWBAH(P)	(0x0603C + (0x40 * (P)))
#define YUSUR2_PVFDCA_RXCTRL(P)	(((P) < 64) ? (0x0100C + (0x40 * (P))) \
				 : (0x0D00C + (0x40 * ((P) - 64))))
#define YUSUR2_PVFDCA_TXCTRL(P)	(0x0600C + (0x40 * (P)))
#define YUSUR2_PVFGPRC(x)	(0x0101C + (0x40 * (x)))
#define YUSUR2_PVFGPTC(x)	(0x08300 + (0x04 * (x)))
#define YUSUR2_PVFGORC_LSB(x)	(0x01020 + (0x40 * (x)))
#define YUSUR2_PVFGORC_MSB(x)	(0x0D020 + (0x40 * (x)))
#define YUSUR2_PVFGOTC_LSB(x)	(0x08400 + (0x08 * (x)))
#define YUSUR2_PVFGOTC_MSB(x)	(0x08404 + (0x08 * (x)))
#define YUSUR2_PVFMPRC(x)	(0x0D01C + (0x40 * (x)))

#define YUSUR2_PVFTDWBALn(q_per_pool, vf_number, vf_q_index) \
		(YUSUR2_PVFTDWBAL((q_per_pool)*(vf_number) + (vf_q_index)))
#define YUSUR2_PVFTDWBAHn(q_per_pool, vf_number, vf_q_index) \
		(YUSUR2_PVFTDWBAH((q_per_pool)*(vf_number) + (vf_q_index)))

#define YUSUR2_PVFTDHn(q_per_pool, vf_number, vf_q_index) \
		(YUSUR2_PVFTDH((q_per_pool)*(vf_number) + (vf_q_index)))
#define YUSUR2_PVFTDTn(q_per_pool, vf_number, vf_q_index) \
		(YUSUR2_PVFTDT((q_per_pool)*(vf_number) + (vf_q_index)))

/* Little Endian defines */
#ifndef __le16
#define __le16  u16
#endif
#ifndef __le32
#define __le32  u32
#endif
#ifndef __le64
#define __le64  u64

#endif
#ifndef __be16
/* Big Endian defines */
#define __be16  u16
#define __be32  u32
#define __be64  u64

#endif
enum yusur2_fdir_pballoc_type {
	YUSUR2_FDIR_PBALLOC_NONE = 0,
	YUSUR2_FDIR_PBALLOC_64K  = 1,
	YUSUR2_FDIR_PBALLOC_128K = 2,
	YUSUR2_FDIR_PBALLOC_256K = 3,
};

/* Flow Director register values */
#define YUSUR2_FDIRCTRL_PBALLOC_64K		0x00000001
#define YUSUR2_FDIRCTRL_PBALLOC_128K		0x00000002
#define YUSUR2_FDIRCTRL_PBALLOC_256K		0x00000003
#define YUSUR2_FDIRCTRL_INIT_DONE		0x00000008
#define YUSUR2_FDIRCTRL_PERFECT_MATCH		0x00000010
#define YUSUR2_FDIRCTRL_REPORT_STATUS		0x00000020
#define YUSUR2_FDIRCTRL_REPORT_STATUS_ALWAYS	0x00000080
#define YUSUR2_FDIRCTRL_DROP_Q_SHIFT		8
#define YUSUR2_FDIRCTRL_DROP_Q_MASK		0x00007F00
#define YUSUR2_FDIRCTRL_FLEX_SHIFT		16
#define YUSUR2_FDIRCTRL_DROP_NO_MATCH		0x00008000
#define YUSUR2_FDIRCTRL_FILTERMODE_SHIFT		21
#define YUSUR2_FDIRCTRL_FILTERMODE_MACVLAN	0x0001 /* bit 23:21, 001b */
#define YUSUR2_FDIRCTRL_FILTERMODE_CLOUD		0x0002 /* bit 23:21, 010b */
#define YUSUR2_FDIRCTRL_SEARCHLIM		0x00800000
#define YUSUR2_FDIRCTRL_FILTERMODE_MASK		0x00E00000
#define YUSUR2_FDIRCTRL_MAX_LENGTH_SHIFT		24
#define YUSUR2_FDIRCTRL_FULL_THRESH_MASK		0xF0000000
#define YUSUR2_FDIRCTRL_FULL_THRESH_SHIFT	28

#define YUSUR2_FDIRTCPM_DPORTM_SHIFT		16
#define YUSUR2_FDIRUDPM_DPORTM_SHIFT		16
#define YUSUR2_FDIRIP6M_DIPM_SHIFT		16
#define YUSUR2_FDIRM_VLANID			0x00000001
#define YUSUR2_FDIRM_VLANP			0x00000002
#define YUSUR2_FDIRM_POOL			0x00000004
#define YUSUR2_FDIRM_L4P				0x00000008
#define YUSUR2_FDIRM_FLEX			0x00000010
#define YUSUR2_FDIRM_DIPv6			0x00000020
#define YUSUR2_FDIRM_L3P				0x00000040

#define YUSUR2_FDIRIP6M_INNER_MAC	0x03F0 /* bit 9:4 */
#define YUSUR2_FDIRIP6M_TUNNEL_TYPE	0x0800 /* bit 11 */
#define YUSUR2_FDIRIP6M_TNI_VNI		0xF000 /* bit 15:12 */
#define YUSUR2_FDIRIP6M_TNI_VNI_24	0x1000 /* bit 12 */
#define YUSUR2_FDIRIP6M_ALWAYS_MASK	0x040F /* bit 10, 3:0 */

#define YUSUR2_FDIRFREE_FREE_MASK		0xFFFF
#define YUSUR2_FDIRFREE_FREE_SHIFT		0
#define YUSUR2_FDIRFREE_COLL_MASK		0x7FFF0000
#define YUSUR2_FDIRFREE_COLL_SHIFT		16
#define YUSUR2_FDIRLEN_MAXLEN_MASK		0x3F
#define YUSUR2_FDIRLEN_MAXLEN_SHIFT		0
#define YUSUR2_FDIRLEN_MAXHASH_MASK		0x7FFF0000
#define YUSUR2_FDIRLEN_MAXHASH_SHIFT		16
#define YUSUR2_FDIRUSTAT_ADD_MASK		0xFFFF
#define YUSUR2_FDIRUSTAT_ADD_SHIFT		0
#define YUSUR2_FDIRUSTAT_REMOVE_MASK		0xFFFF0000
#define YUSUR2_FDIRUSTAT_REMOVE_SHIFT		16
#define YUSUR2_FDIRFSTAT_FADD_MASK		0x00FF
#define YUSUR2_FDIRFSTAT_FADD_SHIFT		0
#define YUSUR2_FDIRFSTAT_FREMOVE_MASK		0xFF00
#define YUSUR2_FDIRFSTAT_FREMOVE_SHIFT		8
#define YUSUR2_FDIRPORT_DESTINATION_SHIFT	16
#define YUSUR2_FDIRVLAN_FLEX_SHIFT		16
#define YUSUR2_FDIRHASH_BUCKET_VALID_SHIFT	15
#define YUSUR2_FDIRHASH_SIG_SW_INDEX_SHIFT	16

#define YUSUR2_FDIRCMD_CMD_MASK			0x00000003
#define YUSUR2_FDIRCMD_CMD_ADD_FLOW		0x00000001
#define YUSUR2_FDIRCMD_CMD_REMOVE_FLOW		0x00000002
#define YUSUR2_FDIRCMD_CMD_QUERY_REM_FILT	0x00000003
#define YUSUR2_FDIRCMD_FILTER_VALID		0x00000004
#define YUSUR2_FDIRCMD_FILTER_UPDATE		0x00000008
#define YUSUR2_FDIRCMD_IPv6DMATCH		0x00000010
#define YUSUR2_FDIRCMD_L4TYPE_UDP		0x00000020
#define YUSUR2_FDIRCMD_L4TYPE_TCP		0x00000040
#define YUSUR2_FDIRCMD_L4TYPE_SCTP		0x00000060
#define YUSUR2_FDIRCMD_IPV6			0x00000080
#define YUSUR2_FDIRCMD_CLEARHT			0x00000100
#define YUSUR2_FDIRCMD_DROP			0x00000200
#define YUSUR2_FDIRCMD_INT			0x00000400
#define YUSUR2_FDIRCMD_LAST			0x00000800
#define YUSUR2_FDIRCMD_COLLISION			0x00001000
#define YUSUR2_FDIRCMD_QUEUE_EN			0x00008000
#define YUSUR2_FDIRCMD_FLOW_TYPE_SHIFT		5
#define YUSUR2_FDIRCMD_RX_QUEUE_SHIFT		16
#define YUSUR2_FDIRCMD_TUNNEL_FILTER_SHIFT	23
#define YUSUR2_FDIRCMD_VT_POOL_SHIFT		24
#define YUSUR2_FDIR_INIT_DONE_POLL		10
#define YUSUR2_FDIRCMD_CMD_POLL			10
#define YUSUR2_FDIRCMD_TUNNEL_FILTER		0x00800000
#define YUSUR2_FDIR_DROP_QUEUE			127


/* Manageablility Host Interface defines */
#define YUSUR2_HI_MAX_BLOCK_BYTE_LENGTH	1792 /* Num of bytes in range */
#define YUSUR2_HI_MAX_BLOCK_DWORD_LENGTH	448 /* Num of dwords in range */
#define YUSUR2_HI_COMMAND_TIMEOUT	500 /* Process HI command limit */
#define YUSUR2_HI_FLASH_ERASE_TIMEOUT	1000 /* Process Erase command limit */
#define YUSUR2_HI_FLASH_UPDATE_TIMEOUT	5000 /* Process Update command limit */
#define YUSUR2_HI_FLASH_APPLY_TIMEOUT	0 /* Process Apply command limit */
#define YUSUR2_HI_PHY_MGMT_REQ_TIMEOUT	2000 /* Wait up to 2 seconds */

/* CEM Support */
#define FW_CEM_HDR_LEN			0x4
#define FW_CEM_CMD_DRIVER_INFO		0xDD
#define FW_CEM_CMD_DRIVER_INFO_LEN	0x5
#define FW_CEM_CMD_RESERVED		0X0
#define FW_CEM_UNUSED_VER		0x0
#define FW_CEM_MAX_RETRIES		3
#define FW_CEM_RESP_STATUS_SUCCESS	0x1
#define FW_CEM_DRIVER_VERSION_SIZE	39 /* +9 would send 48 bytes to fw */
#define FW_READ_SHADOW_RAM_CMD		0x31
#define FW_READ_SHADOW_RAM_LEN		0x6
#define FW_WRITE_SHADOW_RAM_CMD		0x33
#define FW_WRITE_SHADOW_RAM_LEN		0xA /* 8 plus 1 WORD to write */
#define FW_SHADOW_RAM_DUMP_CMD		0x36
#define FW_SHADOW_RAM_DUMP_LEN		0
#define FW_DEFAULT_CHECKSUM		0xFF /* checksum always 0xFF */
#define FW_NVM_DATA_OFFSET		3
#define FW_MAX_READ_BUFFER_SIZE		1024
#define FW_DISABLE_RXEN_CMD		0xDE
#define FW_DISABLE_RXEN_LEN		0x1
#define FW_PHY_MGMT_REQ_CMD		0x20
#define FW_PHY_TOKEN_REQ_CMD		0xA
#define FW_PHY_TOKEN_REQ_LEN		2
#define FW_PHY_TOKEN_REQ		0
#define FW_PHY_TOKEN_REL		1
#define FW_PHY_TOKEN_OK			1
#define FW_PHY_TOKEN_RETRY		0x80
#define FW_PHY_TOKEN_DELAY		5	/* milliseconds */
#define FW_PHY_TOKEN_WAIT		5	/* seconds */
#define FW_PHY_TOKEN_RETRIES ((FW_PHY_TOKEN_WAIT * 1000) / FW_PHY_TOKEN_DELAY)
#define FW_INT_PHY_REQ_CMD		0xB
#define FW_INT_PHY_REQ_LEN		10
#define FW_INT_PHY_REQ_READ		0
#define FW_INT_PHY_REQ_WRITE		1
#define FW_PHY_ACT_REQ_CMD		5
#define FW_PHY_ACT_DATA_COUNT		4
#define FW_PHY_ACT_REQ_LEN		(4 + 4 * FW_PHY_ACT_DATA_COUNT)
#define FW_PHY_ACT_INIT_PHY		1
#define FW_PHY_ACT_SETUP_LINK		2
#define FW_PHY_ACT_LINK_SPEED_10	(1u << 0)
#define FW_PHY_ACT_LINK_SPEED_100	(1u << 1)
#define FW_PHY_ACT_LINK_SPEED_1G	(1u << 2)
#define FW_PHY_ACT_LINK_SPEED_2_5G	(1u << 3)
#define FW_PHY_ACT_LINK_SPEED_5G	(1u << 4)
#define FW_PHY_ACT_LINK_SPEED_10G	(1u << 5)
#define FW_PHY_ACT_LINK_SPEED_20G	(1u << 6)
#define FW_PHY_ACT_LINK_SPEED_25G	(1u << 7)
#define FW_PHY_ACT_LINK_SPEED_40G	(1u << 8)
#define FW_PHY_ACT_LINK_SPEED_50G	(1u << 9)
#define FW_PHY_ACT_LINK_SPEED_100G	(1u << 10)
#define FW_PHY_ACT_SETUP_LINK_PAUSE_SHIFT 16
#define FW_PHY_ACT_SETUP_LINK_PAUSE_MASK (3u << \
					  FW_PHY_ACT_SETUP_LINK_PAUSE_SHIFT)
#define FW_PHY_ACT_SETUP_LINK_PAUSE_NONE 0u
#define FW_PHY_ACT_SETUP_LINK_PAUSE_TX	1u
#define FW_PHY_ACT_SETUP_LINK_PAUSE_RX	2u
#define FW_PHY_ACT_SETUP_LINK_PAUSE_RXTX 3u
#define FW_PHY_ACT_SETUP_LINK_LP	(1u << 18)
#define FW_PHY_ACT_SETUP_LINK_HP	(1u << 19)
#define FW_PHY_ACT_SETUP_LINK_EEE	(1u << 20)
#define FW_PHY_ACT_SETUP_LINK_AN	(1u << 22)
#define FW_PHY_ACT_SETUP_LINK_RSP_DOWN	(1u << 0)
#define FW_PHY_ACT_GET_LINK_INFO	3
#define FW_PHY_ACT_GET_LINK_INFO_EEE	(1u << 19)
#define FW_PHY_ACT_GET_LINK_INFO_FC_TX	(1u << 20)
#define FW_PHY_ACT_GET_LINK_INFO_FC_RX	(1u << 21)
#define FW_PHY_ACT_GET_LINK_INFO_POWER	(1u << 22)
#define FW_PHY_ACT_GET_LINK_INFO_AN_COMPLETE	(1u << 24)
#define FW_PHY_ACT_GET_LINK_INFO_TEMP	(1u << 25)
#define FW_PHY_ACT_GET_LINK_INFO_LP_FC_TX	(1u << 28)
#define FW_PHY_ACT_GET_LINK_INFO_LP_FC_RX	(1u << 29)
#define FW_PHY_ACT_FORCE_LINK_DOWN	4
#define FW_PHY_ACT_FORCE_LINK_DOWN_OFF	(1u << 0)
#define FW_PHY_ACT_PHY_SW_RESET		5
#define FW_PHY_ACT_PHY_HW_RESET		6
#define FW_PHY_ACT_GET_PHY_INFO		7
#define FW_PHY_ACT_UD_2			0x1002
#define FW_PHY_ACT_UD_2_10G_KR_EEE	(1u << 6)
#define FW_PHY_ACT_UD_2_10G_KX4_EEE	(1u << 5)
#define FW_PHY_ACT_UD_2_1G_KX_EEE	(1u << 4)
#define FW_PHY_ACT_UD_2_10G_T_EEE	(1u << 3)
#define FW_PHY_ACT_UD_2_1G_T_EEE	(1u << 2)
#define FW_PHY_ACT_UD_2_100M_TX_EEE	(1u << 1)
#define FW_PHY_ACT_RETRIES		50
#define FW_PHY_INFO_SPEED_MASK		0xFFFu
#define FW_PHY_INFO_ID_HI_MASK		0xFFFF0000u
#define FW_PHY_INFO_ID_LO_MASK		0x0000FFFFu

/* Host Interface Command Structures */

#ifdef C99
#pragma pack(push, 1)
#else
#pragma pack (1)
#endif /* C99 */

struct yusur2_hic_hdr {
	u8 cmd;
	u8 buf_len;
	union {
		u8 cmd_resv;
		u8 ret_status;
	} cmd_or_resp;
	u8 checksum;
};

struct yusur2_hic_hdr2_req {
	u8 cmd;
	u8 buf_lenh;
	u8 buf_lenl;
	u8 checksum;
};

struct yusur2_hic_hdr2_rsp {
	u8 cmd;
	u8 buf_lenl;
	u8 buf_lenh_status;	/* 7-5: high bits of buf_len, 4-0: status */
	u8 checksum;
};

union yusur2_hic_hdr2 {
	struct yusur2_hic_hdr2_req req;
	struct yusur2_hic_hdr2_rsp rsp;
};

struct yusur2_hic_drv_info {
	struct yusur2_hic_hdr hdr;
	u8 port_num;
	u8 ver_sub;
	u8 ver_build;
	u8 ver_min;
	u8 ver_maj;
	u8 pad; /* end spacing to ensure length is mult. of dword */
	u16 pad2; /* end spacing to ensure length is mult. of dword2 */
};

struct yusur2_hic_drv_info2 {
	struct yusur2_hic_hdr hdr;
	u8 port_num;
	u8 ver_sub;
	u8 ver_build;
	u8 ver_min;
	u8 ver_maj;
	char driver_string[FW_CEM_DRIVER_VERSION_SIZE];
};

/* These need to be dword aligned */
struct yusur2_hic_read_shadow_ram {
	union yusur2_hic_hdr2 hdr;
	u32 address;
	u16 length;
	u16 pad2;
	u16 data;
	u16 pad3;
};

struct yusur2_hic_write_shadow_ram {
	union yusur2_hic_hdr2 hdr;
	u32 address;
	u16 length;
	u16 pad2;
	u16 data;
	u16 pad3;
};

struct yusur2_hic_disable_rxen {
	struct yusur2_hic_hdr hdr;
	u8  port_number;
	u8  pad2;
	u16 pad3;
};

struct yusur2_hic_phy_token_req {
	struct yusur2_hic_hdr hdr;
	u8 port_number;
	u8 command_type;
	u16 pad;
};

struct yusur2_hic_internal_phy_req {
	struct yusur2_hic_hdr hdr;
	u8 port_number;
	u8 command_type;
	__be16 address;
	u16 rsv1;
	__be32 write_data;
	u16 pad;
};

struct yusur2_hic_internal_phy_resp {
	struct yusur2_hic_hdr hdr;
	__be32 read_data;
};

struct yusur2_hic_phy_activity_req {
	struct yusur2_hic_hdr hdr;
	u8 port_number;
	u8 pad;
	__le16 activity_id;
	__be32 data[FW_PHY_ACT_DATA_COUNT];
};

struct yusur2_hic_phy_activity_resp {
	struct yusur2_hic_hdr hdr;
	__be32 data[FW_PHY_ACT_DATA_COUNT];
};

#ifdef C99
#pragma pack(pop)
#else
#pragma pack()
#endif /* C99 */

/* Transmit Descriptor - Legacy */
struct yusur2_legacy_tx_desc {
	u64 buffer_addr; /* Address of the descriptor's data buffer */
	union {
		__le32 data;
		struct {
			__le16 length; /* Data buffer length */
			u8 cso; /* Checksum offset */
			u8 cmd; /* Descriptor control */
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status; /* Descriptor status */
			u8 css; /* Checksum start */
			__le16 vlan;
		} fields;
	} upper;
};

/* Transmit Descriptor - Advanced */
union yusur2_adv_tx_desc {
	struct {
		__le64 buffer_addr; /* Address of descriptor's data buf */
		__le32 cmd_type_len;
		__le32 olinfo_status;
	} read;
	struct {
		__le64 rsvd; /* Reserved */
		__le32 nxtseq_seed;
		__le32 status;
	} wb;
};

/* Receive Descriptor - Legacy */
struct yusur2_legacy_rx_desc {
	__le64 buffer_addr; /* Address of the descriptor's data buffer */
	__le16 length; /* Length of data DMAed into data buffer */
	__le16 csum; /* Packet checksum */
	u8 status;   /* Descriptor status */
	u8 errors;   /* Descriptor Errors */
	__le16 vlan;
};

/* Receive Descriptor - Advanced */
union yusur2_adv_rx_desc {
	struct {
		__le64 pkt_addr; /* Packet buffer address */
		__le64 hdr_addr; /* Header buffer address */
	} read;
	struct {
		struct {
			union {
				__le32 data;
				struct {
					__le16 pkt_info; /* RSS, Pkt type */
					__le16 hdr_info; /* Splithdr, hdrlen */
				} hs_rss;
			} lo_dword;
			union {
				__le32 rss; /* RSS Hash */
				struct {
					__le16 ip_id; /* IP id */
					__le16 csum; /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error; /* ext status/error */
			__le16 length; /* Packet length */
			__le16 vlan; /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

/* Context descriptors */
struct yusur2_adv_tx_context_desc {
	__le32 vlan_macip_lens;
	__le32 seqnum_seed;
	__le32 type_tucmd_mlhl;
	__le32 mss_l4len_idx;
};

/* Adv Transmit Descriptor Config Masks */
#define YUSUR2_ADVTXD_DTALEN_MASK	0x0000FFFF /* Data buf length(bytes) */
#define YUSUR2_ADVTXD_MAC_LINKSEC	0x00040000 /* Insert LinkSec */
#define YUSUR2_ADVTXD_MAC_TSTAMP		0x00080000 /* IEEE1588 time stamp */
#define YUSUR2_ADVTXD_IPSEC_SA_INDEX_MASK 0x000003FF /* IPSec SA index */
#define YUSUR2_ADVTXD_IPSEC_ESP_LEN_MASK	0x000001FF /* IPSec ESP length */
#define YUSUR2_ADVTXD_DTYP_MASK		0x00F00000 /* DTYP mask */
#define YUSUR2_ADVTXD_DTYP_CTXT		0x00200000 /* Adv Context Desc */
#define YUSUR2_ADVTXD_DTYP_DATA		0x00300000 /* Adv Data Descriptor */
#define YUSUR2_ADVTXD_DCMD_EOP		YUSUR2_TXD_CMD_EOP  /* End of Packet */
#define YUSUR2_ADVTXD_DCMD_IFCS		YUSUR2_TXD_CMD_IFCS /* Insert FCS */
#define YUSUR2_ADVTXD_DCMD_RS		YUSUR2_TXD_CMD_RS /* Report Status */
#define YUSUR2_ADVTXD_DCMD_DDTYP_ISCSI	0x10000000 /* DDP hdr type or iSCSI */
#define YUSUR2_ADVTXD_DCMD_DEXT		YUSUR2_TXD_CMD_DEXT /* Desc ext 1=Adv */
#define YUSUR2_ADVTXD_DCMD_VLE		YUSUR2_TXD_CMD_VLE  /* VLAN pkt enable */
#define YUSUR2_ADVTXD_DCMD_TSE		0x80000000 /* TCP Seg enable */
#define YUSUR2_ADVTXD_STAT_DD		YUSUR2_TXD_STAT_DD  /* Descriptor Done */
#define YUSUR2_ADVTXD_STAT_SN_CRC	0x00000002 /* NXTSEQ/SEED pres in WB */
#define YUSUR2_ADVTXD_STAT_RSV		0x0000000C /* STA Reserved */
#define YUSUR2_ADVTXD_IDX_SHIFT		4 /* Adv desc Index shift */
#define YUSUR2_ADVTXD_CC			0x00000080 /* Check Context */
#define YUSUR2_ADVTXD_POPTS_SHIFT	8  /* Adv desc POPTS shift */
#define YUSUR2_ADVTXD_POPTS_IXSM		(YUSUR2_TXD_POPTS_IXSM << \
					 YUSUR2_ADVTXD_POPTS_SHIFT)
#define YUSUR2_ADVTXD_POPTS_TXSM		(YUSUR2_TXD_POPTS_TXSM << \
					 YUSUR2_ADVTXD_POPTS_SHIFT)
#define YUSUR2_ADVTXD_POPTS_ISCO_1ST	0x00000000 /* 1st TSO of iSCSI PDU */
#define YUSUR2_ADVTXD_POPTS_ISCO_MDL	0x00000800 /* Middle TSO of iSCSI PDU */
#define YUSUR2_ADVTXD_POPTS_ISCO_LAST	0x00001000 /* Last TSO of iSCSI PDU */
/* 1st&Last TSO-full iSCSI PDU */
#define YUSUR2_ADVTXD_POPTS_ISCO_FULL	0x00001800
#define YUSUR2_ADVTXD_POPTS_RSV		0x00002000 /* POPTS Reserved */
#define YUSUR2_ADVTXD_PAYLEN_SHIFT	14 /* Adv desc PAYLEN shift */
#define YUSUR2_ADVTXD_MACLEN_SHIFT	9  /* Adv ctxt desc mac len shift */
#define YUSUR2_ADVTXD_VLAN_SHIFT		16  /* Adv ctxt vlan tag shift */
#define YUSUR2_ADVTXD_TUCMD_IPV4		0x00000400 /* IP Packet Type: 1=IPv4 */
#define YUSUR2_ADVTXD_TUCMD_IPV6		0x00000000 /* IP Packet Type: 0=IPv6 */
#define YUSUR2_ADVTXD_TUCMD_L4T_UDP	0x00000000 /* L4 Packet TYPE of UDP */
#define YUSUR2_ADVTXD_TUCMD_L4T_TCP	0x00000800 /* L4 Packet TYPE of TCP */
#define YUSUR2_ADVTXD_TUCMD_L4T_SCTP	0x00001000 /* L4 Packet TYPE of SCTP */
#define YUSUR2_ADVTXD_TUCMD_L4T_RSV	0x00001800 /* RSV L4 Packet TYPE */
#define YUSUR2_ADVTXD_TUCMD_MKRREQ	0x00002000 /* req Markers and CRC */
#define YUSUR2_ADVTXD_POPTS_IPSEC	0x00000400 /* IPSec offload request */
#define YUSUR2_ADVTXD_TUCMD_IPSEC_TYPE_ESP 0x00002000 /* IPSec Type ESP */
#define YUSUR2_ADVTXD_TUCMD_IPSEC_ENCRYPT_EN 0x00004000/* ESP Encrypt Enable */
#define YUSUR2_ADVTXT_TUCMD_FCOE		0x00008000 /* FCoE Frame Type */
#define YUSUR2_ADVTXD_FCOEF_EOF_MASK	(0x3 << 10) /* FC EOF index */
#define YUSUR2_ADVTXD_FCOEF_SOF		((1 << 2) << 10) /* FC SOF index */
#define YUSUR2_ADVTXD_FCOEF_PARINC	((1 << 3) << 10) /* Rel_Off in F_CTL */
#define YUSUR2_ADVTXD_FCOEF_ORIE		((1 << 4) << 10) /* Orientation End */
#define YUSUR2_ADVTXD_FCOEF_ORIS		((1 << 5) << 10) /* Orientation Start */
#define YUSUR2_ADVTXD_FCOEF_EOF_N	(0x0 << 10) /* 00: EOFn */
#define YUSUR2_ADVTXD_FCOEF_EOF_T	(0x1 << 10) /* 01: EOFt */
#define YUSUR2_ADVTXD_FCOEF_EOF_NI	(0x2 << 10) /* 10: EOFni */
#define YUSUR2_ADVTXD_FCOEF_EOF_A	(0x3 << 10) /* 11: EOFa */
#define YUSUR2_ADVTXD_L4LEN_SHIFT	8  /* Adv ctxt L4LEN shift */
#define YUSUR2_ADVTXD_MSS_SHIFT		16  /* Adv ctxt MSS shift */

#define YUSUR2_ADVTXD_OUTER_IPLEN	16 /* Adv ctxt OUTERIPLEN shift */
#define YUSUR2_ADVTXD_TUNNEL_LEN 	24 /* Adv ctxt TUNNELLEN shift */
#define YUSUR2_ADVTXD_TUNNEL_TYPE_SHIFT	16 /* Adv Tx Desc Tunnel Type shift */
#define YUSUR2_ADVTXD_OUTERIPCS_SHIFT	17 /* Adv Tx Desc OUTERIPCS Shift */
#define YUSUR2_ADVTXD_TUNNEL_TYPE_NVGRE	1  /* Adv Tx Desc Tunnel Type NVGRE */
/* Adv Tx Desc OUTERIPCS Shift for X550EM_a */
#define YUSUR2_ADVTXD_OUTERIPCS_SHIFT_X550EM_a	26
/* Autonegotiation advertised speeds */
typedef u32 yusur2_autoneg_advertised;
/* Link speed */
typedef u32 yusur2_link_speed;
#define YUSUR2_LINK_SPEED_UNKNOWN	0
#define YUSUR2_LINK_SPEED_10_FULL	0x0002
#define YUSUR2_LINK_SPEED_100_FULL	0x0008
#define YUSUR2_LINK_SPEED_1GB_FULL	0x0020
#define YUSUR2_LINK_SPEED_2_5GB_FULL	0x0400
#define YUSUR2_LINK_SPEED_5GB_FULL	0x0800
#define YUSUR2_LINK_SPEED_10GB_FULL	0x0080
#define YUSUR2_LINK_SPEED_82598_AUTONEG	(YUSUR2_LINK_SPEED_1GB_FULL | \
					 YUSUR2_LINK_SPEED_10GB_FULL)
#define YUSUR2_LINK_SPEED_82599_AUTONEG	(YUSUR2_LINK_SPEED_100_FULL | \
					 YUSUR2_LINK_SPEED_1GB_FULL | \
					 YUSUR2_LINK_SPEED_10GB_FULL)

/* Physical layer type */
typedef u64 yusur2_physical_layer;
#define YUSUR2_PHYSICAL_LAYER_UNKNOWN		0
#define YUSUR2_PHYSICAL_LAYER_10GBASE_T		0x00001
#define YUSUR2_PHYSICAL_LAYER_1000BASE_T		0x00002
#define YUSUR2_PHYSICAL_LAYER_100BASE_TX		0x00004
#define YUSUR2_PHYSICAL_LAYER_SFP_PLUS_CU	0x00008
#define YUSUR2_PHYSICAL_LAYER_10GBASE_LR		0x00010
#define YUSUR2_PHYSICAL_LAYER_10GBASE_LRM	0x00020
#define YUSUR2_PHYSICAL_LAYER_10GBASE_SR		0x00040
#define YUSUR2_PHYSICAL_LAYER_10GBASE_KX4	0x00080
#define YUSUR2_PHYSICAL_LAYER_10GBASE_CX4	0x00100
#define YUSUR2_PHYSICAL_LAYER_1000BASE_KX	0x00200
#define YUSUR2_PHYSICAL_LAYER_1000BASE_BX	0x00400
#define YUSUR2_PHYSICAL_LAYER_10GBASE_KR		0x00800
#define YUSUR2_PHYSICAL_LAYER_10GBASE_XAUI	0x01000
#define YUSUR2_PHYSICAL_LAYER_SFP_ACTIVE_DA	0x02000
#define YUSUR2_PHYSICAL_LAYER_1000BASE_SX	0x04000
#define YUSUR2_PHYSICAL_LAYER_10BASE_T		0x08000
#define YUSUR2_PHYSICAL_LAYER_2500BASE_KX	0x10000

/* Flow Control Data Sheet defined values
 * Calculation and defines taken from 802.1bb Annex O
 */

/* BitTimes (BT) conversion */
#define YUSUR2_BT2KB(BT)		((BT + (8 * 1024 - 1)) / (8 * 1024))
#define YUSUR2_B2BT(BT)		(BT * 8)

/* Calculate Delay to respond to PFC */
#define YUSUR2_PFC_D	672

/* Calculate Cable Delay */
#define YUSUR2_CABLE_DC	5556 /* Delay Copper */
#define YUSUR2_CABLE_DO	5000 /* Delay Optical */

/* Calculate Interface Delay X540 */
#define YUSUR2_PHY_DC	25600 /* Delay 10G BASET */
#define YUSUR2_MAC_DC	8192  /* Delay Copper XAUI interface */
#define YUSUR2_XAUI_DC	(2 * 2048) /* Delay Copper Phy */

#define YUSUR2_ID_X540	(YUSUR2_MAC_DC + YUSUR2_XAUI_DC + YUSUR2_PHY_DC)

/* Calculate Interface Delay 82598, 82599 */
#define YUSUR2_PHY_D	12800
#define YUSUR2_MAC_D	4096
#define YUSUR2_XAUI_D	(2 * 1024)

#define YUSUR2_ID	(YUSUR2_MAC_D + YUSUR2_XAUI_D + YUSUR2_PHY_D)

/* Calculate Delay incurred from higher layer */
#define YUSUR2_HD	6144

/* Calculate PCI Bus delay for low thresholds */
#define YUSUR2_PCI_DELAY	10000

/* Calculate X540 delay value in bit times */
#define YUSUR2_DV_X540(_max_frame_link, _max_frame_tc) \
			((36 * \
			  (YUSUR2_B2BT(_max_frame_link) + \
			   YUSUR2_PFC_D + \
			   (2 * YUSUR2_CABLE_DC) + \
			   (2 * YUSUR2_ID_X540) + \
			   YUSUR2_HD) / 25 + 1) + \
			 2 * YUSUR2_B2BT(_max_frame_tc))

/* Calculate 82599, 82598 delay value in bit times */
#define YUSUR2_DV(_max_frame_link, _max_frame_tc) \
			((36 * \
			  (YUSUR2_B2BT(_max_frame_link) + \
			   YUSUR2_PFC_D + \
			   (2 * YUSUR2_CABLE_DC) + \
			   (2 * YUSUR2_ID) + \
			   YUSUR2_HD) / 25 + 1) + \
			 2 * YUSUR2_B2BT(_max_frame_tc))

/* Calculate low threshold delay values */
#define YUSUR2_LOW_DV_X540(_max_frame_tc) \
			(2 * YUSUR2_B2BT(_max_frame_tc) + \
			(36 * YUSUR2_PCI_DELAY / 25) + 1)
#define YUSUR2_LOW_DV(_max_frame_tc) \
			(2 * YUSUR2_LOW_DV_X540(_max_frame_tc))

/* Software ATR hash keys */
#define YUSUR2_ATR_BUCKET_HASH_KEY	0x3DAD14E2
#define YUSUR2_ATR_SIGNATURE_HASH_KEY	0x174D3614

/* Software ATR input stream values and masks */
#define YUSUR2_ATR_HASH_MASK		0x7fff
#define YUSUR2_ATR_L4TYPE_MASK		0x3
#define YUSUR2_ATR_L4TYPE_UDP		0x1
#define YUSUR2_ATR_L4TYPE_TCP		0x2
#define YUSUR2_ATR_L4TYPE_SCTP		0x3
#define YUSUR2_ATR_L4TYPE_IPV6_MASK	0x4
#define YUSUR2_ATR_L4TYPE_TUNNEL_MASK	0x10
enum yusur2_atr_flow_type {
	YUSUR2_ATR_FLOW_TYPE_IPV4	= 0x0,
	YUSUR2_ATR_FLOW_TYPE_UDPV4	= 0x1,
	YUSUR2_ATR_FLOW_TYPE_TCPV4	= 0x2,
	YUSUR2_ATR_FLOW_TYPE_SCTPV4	= 0x3,
	YUSUR2_ATR_FLOW_TYPE_IPV6	= 0x4,
	YUSUR2_ATR_FLOW_TYPE_UDPV6	= 0x5,
	YUSUR2_ATR_FLOW_TYPE_TCPV6	= 0x6,
	YUSUR2_ATR_FLOW_TYPE_SCTPV6	= 0x7,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_IPV4	= 0x10,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_UDPV4	= 0x11,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_TCPV4	= 0x12,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_SCTPV4	= 0x13,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_IPV6	= 0x14,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_UDPV6	= 0x15,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_TCPV6	= 0x16,
	YUSUR2_ATR_FLOW_TYPE_TUNNELED_SCTPV6	= 0x17,
};

/* Flow Director ATR input struct. */
union yusur2_atr_input {
	/*
	 * Byte layout in order, all values with MSB first:
	 *
	 * vm_pool	- 1 byte
	 * flow_type	- 1 byte
	 * vlan_id	- 2 bytes
	 * src_ip	- 16 bytes
	 * inner_mac	- 6 bytes
	 * cloud_mode	- 2 bytes
	 * tni_vni	- 4 bytes
	 * dst_ip	- 16 bytes
	 * src_port	- 2 bytes
	 * dst_port	- 2 bytes
	 * flex_bytes	- 2 bytes
	 * bkt_hash	- 2 bytes
	 */
	struct {
		u8 vm_pool;
		u8 flow_type;
		__be16 vlan_id;
		__be32 dst_ip[4];
		__be32 src_ip[4];
		u8 inner_mac[6];
		__be16 tunnel_type;
		__be32 tni_vni;
		__be16 src_port;
		__be16 dst_port;
		__be16 flex_bytes;
		__be16 bkt_hash;
	} formatted;
	__be32 dword_stream[14];
};

/* Flow Director compressed ATR hash input struct */
union yusur2_atr_hash_dword {
	struct {
		u8 vm_pool;
		u8 flow_type;
		__be16 vlan_id;
	} formatted;
	__be32 ip;
	struct {
		__be16 src;
		__be16 dst;
	} port;
	__be16 flex_bytes;
	__be32 dword;
};


#define YUSUR2_MVALS_INIT(m)	\
	YUSUR2_CAT(EEC, m),		\
	YUSUR2_CAT(FLA, m),		\
	YUSUR2_CAT(GRC, m),		\
	YUSUR2_CAT(SRAMREL, m),		\
	YUSUR2_CAT(FACTPS, m),		\
	YUSUR2_CAT(SWSM, m),		\
	YUSUR2_CAT(SWFW_SYNC, m),	\
	YUSUR2_CAT(FWSM, m),		\
	YUSUR2_CAT(SDP0_GPIEN, m),	\
	YUSUR2_CAT(SDP1_GPIEN, m),	\
	YUSUR2_CAT(SDP2_GPIEN, m),	\
	YUSUR2_CAT(EICR_GPI_SDP0, m),	\
	YUSUR2_CAT(EICR_GPI_SDP1, m),	\
	YUSUR2_CAT(EICR_GPI_SDP2, m),	\
	YUSUR2_CAT(CIAA, m),		\
	YUSUR2_CAT(CIAD, m),		\
	YUSUR2_CAT(I2C_CLK_IN, m),	\
	YUSUR2_CAT(I2C_CLK_OUT, m),	\
	YUSUR2_CAT(I2C_DATA_IN, m),	\
	YUSUR2_CAT(I2C_DATA_OUT, m),	\
	YUSUR2_CAT(I2C_DATA_OE_N_EN, m),	\
	YUSUR2_CAT(I2C_BB_EN, m),	\
	YUSUR2_CAT(I2C_CLK_OE_N_EN, m),	\
	YUSUR2_CAT(I2CCTL, m)

enum yusur2_mvals {
	YUSUR2_MVALS_INIT(_IDX),
	YUSUR2_MVALS_IDX_LIMIT
};

/*
 * Unavailable: The FCoE Boot Option ROM is not present in the flash.
 * Disabled: Present; boot order is not set for any targets on the port.
 * Enabled: Present; boot order is set for at least one target on the port.
 */
enum yusur2_fcoe_boot_status {
	yusur2_fcoe_bootstatus_disabled = 0,
	yusur2_fcoe_bootstatus_enabled = 1,
	yusur2_fcoe_bootstatus_unavailable = 0xFFFF
};

enum yusur2_eeprom_type {
	yusur2_eeprom_uninitialized = 0,
	yusur2_eeprom_spi,
	yusur2_flash,
	yusur2_eeprom_none /* No NVM support */
};

enum yusur2_mac_type {
	yusur2_mac_unknown = 0,
	yusur2_mac_82598EB,
	yusur2_mac_82599EB,
	yusur2_mac_82599_vf,
	yusur2_mac_X540,
	yusur2_mac_X540_vf,
	yusur2_mac_X550,
	yusur2_mac_X550EM_x,
	yusur2_mac_X550EM_a,
	yusur2_mac_X550_vf,
	yusur2_mac_X550EM_x_vf,
	yusur2_mac_X550EM_a_vf,
	yusur2_num_macs
};

enum yusur2_phy_type {
	yusur2_phy_unknown = 0,
	yusur2_phy_none,
	yusur2_phy_tn,
	yusur2_phy_aq,
	yusur2_phy_x550em_kr,
	yusur2_phy_x550em_kx4,
	yusur2_phy_x550em_xfi,
	yusur2_phy_x550em_ext_t,
	yusur2_phy_ext_1g_t,
	yusur2_phy_cu_unknown,
	yusur2_phy_qt,
	yusur2_phy_xaui,
	yusur2_phy_nl,
	yusur2_phy_sfp_passive_tyco,
	yusur2_phy_sfp_passive_unknown,
	yusur2_phy_sfp_active_unknown,
	yusur2_phy_sfp_avago,
	yusur2_phy_sfp_ftl,
	yusur2_phy_sfp_ftl_active,
	yusur2_phy_sfp_unknown,
	yusur2_phy_sfp_intel,
	yusur2_phy_qsfp_passive_unknown,
	yusur2_phy_qsfp_active_unknown,
	yusur2_phy_qsfp_intel,
	yusur2_phy_qsfp_unknown,
	yusur2_phy_sfp_unsupported, /*Enforce bit set with unsupported module*/
	yusur2_phy_sgmii,
	yusur2_phy_fw,
	yusur2_phy_generic
};

/*
 * SFP+ module type IDs:
 *
 * ID	Module Type
 * =============
 * 0	SFP_DA_CU
 * 1	SFP_SR
 * 2	SFP_LR
 * 3	SFP_DA_CU_CORE0 - 82599-specific
 * 4	SFP_DA_CU_CORE1 - 82599-specific
 * 5	SFP_SR/LR_CORE0 - 82599-specific
 * 6	SFP_SR/LR_CORE1 - 82599-specific
 */
enum yusur2_sfp_type {
	yusur2_sfp_type_da_cu = 0,
	yusur2_sfp_type_sr = 1,
	yusur2_sfp_type_lr = 2,
	yusur2_sfp_type_da_cu_core0 = 3,
	yusur2_sfp_type_da_cu_core1 = 4,
	yusur2_sfp_type_srlr_core0 = 5,
	yusur2_sfp_type_srlr_core1 = 6,
	yusur2_sfp_type_da_act_lmt_core0 = 7,
	yusur2_sfp_type_da_act_lmt_core1 = 8,
	yusur2_sfp_type_1g_cu_core0 = 9,
	yusur2_sfp_type_1g_cu_core1 = 10,
	yusur2_sfp_type_1g_sx_core0 = 11,
	yusur2_sfp_type_1g_sx_core1 = 12,
	yusur2_sfp_type_1g_lx_core0 = 13,
	yusur2_sfp_type_1g_lx_core1 = 14,
	yusur2_sfp_type_1g_lha_core0 = 15,
	yusur2_sfp_type_1g_lha_core1 = 16,
	yusur2_sfp_type_not_present = 0xFFFE,
	yusur2_sfp_type_unknown = 0xFFFF
};

enum yusur2_media_type {
	yusur2_media_type_unknown = 0,
	yusur2_media_type_fiber,
	yusur2_media_type_fiber_qsfp,
	yusur2_media_type_copper,
	yusur2_media_type_backplane,
	yusur2_media_type_cx4,
	yusur2_media_type_virtual
};

/* Flow Control Settings */
enum yusur2_fc_mode {
	yusur2_fc_none = 0,
	yusur2_fc_rx_pause,
	yusur2_fc_tx_pause,
	yusur2_fc_full,
	yusur2_fc_default
};

/* Smart Speed Settings */
#define YUSUR2_SMARTSPEED_MAX_RETRIES	3
enum yusur2_smart_speed {
	yusur2_smart_speed_auto = 0,
	yusur2_smart_speed_on,
	yusur2_smart_speed_off
};

/* PCI bus types */
enum yusur2_bus_type {
	yusur2_bus_type_unknown = 0,
	yusur2_bus_type_pci,
	yusur2_bus_type_pcix,
	yusur2_bus_type_pci_express,
	yusur2_bus_type_internal,
	yusur2_bus_type_reserved
};

/* PCI bus speeds */
enum yusur2_bus_speed {
	yusur2_bus_speed_unknown	= 0,
	yusur2_bus_speed_33	= 33,
	yusur2_bus_speed_66	= 66,
	yusur2_bus_speed_100	= 100,
	yusur2_bus_speed_120	= 120,
	yusur2_bus_speed_133	= 133,
	yusur2_bus_speed_2500	= 2500,
	yusur2_bus_speed_5000	= 5000,
	yusur2_bus_speed_8000	= 8000,
	yusur2_bus_speed_reserved
};

/* PCI bus widths */
enum yusur2_bus_width {
	yusur2_bus_width_unknown	= 0,
	yusur2_bus_width_pcie_x1	= 1,
	yusur2_bus_width_pcie_x2	= 2,
	yusur2_bus_width_pcie_x4	= 4,
	yusur2_bus_width_pcie_x8	= 8,
	yusur2_bus_width_32	= 32,
	yusur2_bus_width_64	= 64,
	yusur2_bus_width_reserved
};

struct yusur2_addr_filter_info {
	u32 num_mc_addrs;
	u32 rar_used_count;
	u32 mta_in_use;
	u32 overflow_promisc;
	bool user_set_promisc;
};

/* Bus parameters */
struct yusur2_bus_info {
	enum yusur2_bus_speed speed;
	enum yusur2_bus_width width;
	enum yusur2_bus_type type;

	u16 func;
	u8 lan_id;
	u16 instance_id;
};

/* Flow control parameters */
struct yusur2_fc_info {
	u32 high_water[YUSUR2_DCB_MAX_TRAFFIC_CLASS]; /* Flow Ctrl High-water */
	u32 low_water[YUSUR2_DCB_MAX_TRAFFIC_CLASS]; /* Flow Ctrl Low-water */
	u16 pause_time; /* Flow Control Pause timer */
	bool send_xon; /* Flow control send XON */
	bool strict_ieee; /* Strict IEEE mode */
	bool disable_fc_autoneg; /* Do not autonegotiate FC */
	bool fc_was_autonegged; /* Is current_mode the result of autonegging? */
	enum yusur2_fc_mode current_mode; /* FC mode in effect */
	enum yusur2_fc_mode requested_mode; /* FC mode requested by caller */
};

/* Statistics counters collected by the MAC */
struct yusur2_hw_stats {
	u64 crcerrs;
	u64 illerrc;
	u64 errbc;
	u64 mspdc;
	u64 mpctotal;
	u64 mpc[8];
	u64 mlfc;
	u64 mrfc;
	u64 rlec;
	u64 lxontxc;
	u64 lxonrxc;
	u64 lxofftxc;
	u64 lxoffrxc;
	u64 pxontxc[8];
	u64 pxonrxc[8];
	u64 pxofftxc[8];
	u64 pxoffrxc[8];
	u64 prc64;
	u64 prc127;
	u64 prc255;
	u64 prc511;
	u64 prc1023;
	u64 prc1522;
	u64 gprc;
	u64 bprc;
	u64 mprc;
	u64 gptc;
	u64 gorc;
	u64 gotc;
	u64 rnbc[8];
	u64 ruc;
	u64 rfc;
	u64 roc;
	u64 rjc;
	u64 mngprc;
	u64 mngpdc;
	u64 mngptc;
	u64 tor;
	u64 tpr;
	u64 tpt;
	u64 ptc64;
	u64 ptc127;
	u64 ptc255;
	u64 ptc511;
	u64 ptc1023;
	u64 ptc1522;
	u64 mptc;
	u64 bptc;
	u64 xec;
	u64 qprc[16];
	u64 qptc[16];
	u64 qbrc[16];
	u64 qbtc[16];
	u64 qprdc[16];
	u64 pxon2offc[8];
	u64 fdirustat_add;
	u64 fdirustat_remove;
	u64 fdirfstat_fadd;
	u64 fdirfstat_fremove;
	u64 fdirmatch;
	u64 fdirmiss;
	u64 fccrc;
	u64 fclast;
	u64 fcoerpdc;
	u64 fcoeprc;
	u64 fcoeptc;
	u64 fcoedwrc;
	u64 fcoedwtc;
	u64 fcoe_noddp;
	u64 fcoe_noddp_ext_buff;
	u64 ldpcec;
	u64 pcrc8ec;
	u64 b2ospc;
	u64 b2ogprc;
	u64 o2bgptc;
	u64 o2bspc;
};

/* forward declaration */
struct yusur2_hw;

/* iterator type for walking multicast address lists */
typedef u8* (*yusur2_mc_addr_itr) (struct yusur2_hw *hw, u8 **mc_addr_ptr,
				  u32 *vmdq);

/* Function pointer table */
struct yusur2_eeprom_operations {
	s32 (*init_params)(struct yusur2_hw *);
	s32 (*read)(struct yusur2_hw *, u16, u16 *);
	s32 (*read_buffer)(struct yusur2_hw *, u16, u16, u16 *);
	s32 (*write)(struct yusur2_hw *, u16, u16);
	s32 (*write_buffer)(struct yusur2_hw *, u16, u16, u16 *);
	s32 (*validate_checksum)(struct yusur2_hw *, u16 *);
	s32 (*update_checksum)(struct yusur2_hw *);
	s32 (*calc_checksum)(struct yusur2_hw *);
};

struct yusur2_mac_operations {
	s32 (*init_hw)(struct yusur2_hw *);
	s32 (*reset_hw)(struct yusur2_hw *);
	s32 (*start_hw)(struct yusur2_hw *);
	s32 (*clear_hw_cntrs)(struct yusur2_hw *);
	void (*enable_relaxed_ordering)(struct yusur2_hw *);
	enum yusur2_media_type (*get_media_type)(struct yusur2_hw *);
	u64 (*get_supported_physical_layer)(struct yusur2_hw *);
	s32 (*get_mac_addr)(struct yusur2_hw *, u8 *);
	s32 (*get_san_mac_addr)(struct yusur2_hw *, u8 *);
	s32 (*set_san_mac_addr)(struct yusur2_hw *, u8 *);
	s32 (*get_device_caps)(struct yusur2_hw *, u16 *);
	s32 (*get_wwn_prefix)(struct yusur2_hw *, u16 *, u16 *);
	s32 (*get_fcoe_boot_status)(struct yusur2_hw *, u16 *);
	s32 (*stop_adapter)(struct yusur2_hw *);
	s32 (*get_bus_info)(struct yusur2_hw *);
	void (*set_lan_id)(struct yusur2_hw *);
	s32 (*read_analog_reg8)(struct yusur2_hw*, u32, u8*);
	s32 (*write_analog_reg8)(struct yusur2_hw*, u32, u8);
	s32 (*setup_sfp)(struct yusur2_hw *);
	s32 (*enable_rx_dma)(struct yusur2_hw *, u32);
	s32 (*disable_sec_rx_path)(struct yusur2_hw *);
	s32 (*enable_sec_rx_path)(struct yusur2_hw *);
	s32 (*acquire_swfw_sync)(struct yusur2_hw *, u32);
	void (*release_swfw_sync)(struct yusur2_hw *, u32);
	void (*init_swfw_sync)(struct yusur2_hw *);
	s32 (*prot_autoc_read)(struct yusur2_hw *, bool *, u32 *);
	s32 (*prot_autoc_write)(struct yusur2_hw *, u32, bool);
	s32 (*negotiate_api_version)(struct yusur2_hw *hw, int api);

	/* Link */
	void (*disable_tx_laser)(struct yusur2_hw *);
	void (*enable_tx_laser)(struct yusur2_hw *);
	void (*flap_tx_laser)(struct yusur2_hw *);
	s32 (*setup_link)(struct yusur2_hw *, yusur2_link_speed, bool);
	s32 (*setup_mac_link)(struct yusur2_hw *, yusur2_link_speed, bool);
	s32 (*check_link)(struct yusur2_hw *, yusur2_link_speed *, bool *, bool);
	s32 (*get_link_capabilities)(struct yusur2_hw *, yusur2_link_speed *,
				     bool *);
	void (*set_rate_select_speed)(struct yusur2_hw *, yusur2_link_speed);

	/* Packet Buffer manipulation */
	void (*setup_rxpba)(struct yusur2_hw *, int, u32, int);

	/* LED */
	s32 (*led_on)(struct yusur2_hw *, u32);
	s32 (*led_off)(struct yusur2_hw *, u32);
	s32 (*blink_led_start)(struct yusur2_hw *, u32);
	s32 (*blink_led_stop)(struct yusur2_hw *, u32);
	s32 (*init_led_link_act)(struct yusur2_hw *);

	/* RAR, Multicast, VLAN */
	s32 (*set_rar)(struct yusur2_hw *, u32, u8 *, u32, u32);
	s32 (*set_uc_addr)(struct yusur2_hw *, u32, u8 *);
	s32 (*clear_rar)(struct yusur2_hw *, u32);
	s32 (*insert_mac_addr)(struct yusur2_hw *, u8 *, u32);
	s32 (*set_vmdq)(struct yusur2_hw *, u32, u32);
	s32 (*set_vmdq_san_mac)(struct yusur2_hw *, u32);
	s32 (*clear_vmdq)(struct yusur2_hw *, u32, u32);
	s32 (*init_rx_addrs)(struct yusur2_hw *);
	s32 (*update_uc_addr_list)(struct yusur2_hw *, u8 *, u32,
				   yusur2_mc_addr_itr);
	s32 (*update_mc_addr_list)(struct yusur2_hw *, u8 *, u32,
				   yusur2_mc_addr_itr, bool clear);
	s32 (*enable_mc)(struct yusur2_hw *);
	s32 (*disable_mc)(struct yusur2_hw *);
	s32 (*clear_vfta)(struct yusur2_hw *);
	s32 (*set_vfta)(struct yusur2_hw *, u32, u32, bool, bool);
	s32 (*set_vlvf)(struct yusur2_hw *, u32, u32, bool, u32 *, u32,
			bool);
	s32 (*init_uta_tables)(struct yusur2_hw *);
	void (*set_mac_anti_spoofing)(struct yusur2_hw *, bool, int);
	void (*set_vlan_anti_spoofing)(struct yusur2_hw *, bool, int);
	s32 (*update_xcast_mode)(struct yusur2_hw *, int);
	s32 (*set_rlpml)(struct yusur2_hw *, u16);

	/* Flow Control */
	s32 (*fc_enable)(struct yusur2_hw *);
	s32 (*setup_fc)(struct yusur2_hw *);
	void (*fc_autoneg)(struct yusur2_hw *);

	/* Manageability interface */
	s32 (*set_fw_drv_ver)(struct yusur2_hw *, u8, u8, u8, u8, u16,
			      const char *);
	s32 (*get_thermal_sensor_data)(struct yusur2_hw *);
	s32 (*init_thermal_sensor_thresh)(struct yusur2_hw *hw);
	void (*get_rtrup2tc)(struct yusur2_hw *hw, u8 *map);
	void (*disable_rx)(struct yusur2_hw *hw);
	void (*enable_rx)(struct yusur2_hw *hw);
	void (*set_source_address_pruning)(struct yusur2_hw *, bool,
					   unsigned int);
	void (*set_ethertype_anti_spoofing)(struct yusur2_hw *, bool, int);
	s32 (*dmac_update_tcs)(struct yusur2_hw *hw);
	s32 (*dmac_config_tcs)(struct yusur2_hw *hw);
	s32 (*dmac_config)(struct yusur2_hw *hw);
	s32 (*setup_eee)(struct yusur2_hw *hw, bool enable_eee);
	s32 (*read_iosf_sb_reg)(struct yusur2_hw *, u32, u32, u32 *);
	s32 (*write_iosf_sb_reg)(struct yusur2_hw *, u32, u32, u32);
	void (*disable_mdd)(struct yusur2_hw *hw);
	void (*enable_mdd)(struct yusur2_hw *hw);
	void (*mdd_event)(struct yusur2_hw *hw, u32 *vf_bitmap);
	void (*restore_mdd_vf)(struct yusur2_hw *hw, u32 vf);
	bool (*fw_recovery_mode)(struct yusur2_hw *hw);
};

struct yusur2_phy_operations {
	s32 (*identify)(struct yusur2_hw *);
	s32 (*identify_sfp)(struct yusur2_hw *);
	s32 (*init)(struct yusur2_hw *);
	s32 (*reset)(struct yusur2_hw *);
	s32 (*read_reg)(struct yusur2_hw *, u32, u32, u16 *);
	s32 (*write_reg)(struct yusur2_hw *, u32, u32, u16);
	s32 (*read_reg_mdi)(struct yusur2_hw *, u32, u32, u16 *);
	s32 (*write_reg_mdi)(struct yusur2_hw *, u32, u32, u16);
	s32 (*setup_link)(struct yusur2_hw *);
	s32 (*setup_internal_link)(struct yusur2_hw *);
	s32 (*setup_link_speed)(struct yusur2_hw *, yusur2_link_speed, bool);
	s32 (*check_link)(struct yusur2_hw *, yusur2_link_speed *, bool *);
	s32 (*get_firmware_version)(struct yusur2_hw *, u16 *);
	s32 (*read_i2c_byte)(struct yusur2_hw *, u8, u8, u8 *);
	s32 (*write_i2c_byte)(struct yusur2_hw *, u8, u8, u8);
	s32 (*read_i2c_sff8472)(struct yusur2_hw *, u8 , u8 *);
	s32 (*read_i2c_eeprom)(struct yusur2_hw *, u8 , u8 *);
	s32 (*write_i2c_eeprom)(struct yusur2_hw *, u8, u8);
	void (*i2c_bus_clear)(struct yusur2_hw *);
	s32 (*check_overtemp)(struct yusur2_hw *);
	s32 (*set_phy_power)(struct yusur2_hw *, bool on);
	s32 (*enter_lplu)(struct yusur2_hw *);
	s32 (*handle_lasi)(struct yusur2_hw *hw);
	s32 (*read_i2c_byte_unlocked)(struct yusur2_hw *, u8 offset, u8 addr,
				      u8 *value);
	s32 (*write_i2c_byte_unlocked)(struct yusur2_hw *, u8 offset, u8 addr,
				       u8 value);
};

struct yusur2_link_operations {
	s32 (*read_link)(struct yusur2_hw *, u8 addr, u16 reg, u16 *val);
	s32 (*read_link_unlocked)(struct yusur2_hw *, u8 addr, u16 reg,
				  u16 *val);
	s32 (*write_link)(struct yusur2_hw *, u8 addr, u16 reg, u16 val);
	s32 (*write_link_unlocked)(struct yusur2_hw *, u8 addr, u16 reg,
				   u16 val);
};

struct yusur2_link_info {
	struct yusur2_link_operations ops;
	u8 addr;
};

struct yusur2_eeprom_info {
	struct yusur2_eeprom_operations ops;
	enum yusur2_eeprom_type type;
	u32 semaphore_delay;
	u16 word_size;
	u16 address_bits;
	u16 word_page_size;
	u16 ctrl_word_3;
};

#define YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED	0x01
struct yusur2_mac_info {
	struct yusur2_mac_operations ops;
	enum yusur2_mac_type type;
	u8 addr[YUSUR2_ETH_LENGTH_OF_ADDRESS];
	u8 perm_addr[YUSUR2_ETH_LENGTH_OF_ADDRESS];
	u8 san_addr[YUSUR2_ETH_LENGTH_OF_ADDRESS];
	/* prefix for World Wide Node Name (WWNN) */
	u16 wwnn_prefix;
	/* prefix for World Wide Port Name (WWPN) */
	u16 wwpn_prefix;
#define YUSUR2_MAX_MTA			128
	u32 mta_shadow[YUSUR2_MAX_MTA];
	s32 mc_filter_type;
	u32 mcft_size;
	u32 vft_size;
	u32 num_rar_entries;
	u32 rar_highwater;
	u32 rx_pb_size;
	u32 max_tx_queues;
	u32 max_rx_queues;
	u32 orig_autoc;
	u8  san_mac_rar_index;
	bool get_link_status;
	u32 orig_autoc2;
	u16 max_msix_vectors;
	bool arc_subsystem_valid;
	bool orig_link_settings_stored;
	bool autotry_restart;
	u8 flags;
	struct yusur2_thermal_sensor_data  thermal_sensor_data;
	bool thermal_sensor_enabled;
	struct yusur2_dmac_config dmac_config;
	bool set_lben;
	u32  max_link_up_time;
	u8   led_link_act;
};

struct yusur2_phy_info {
	struct yusur2_phy_operations ops;
	enum yusur2_phy_type type;
	u32 addr;
	u32 id;
	enum yusur2_sfp_type sfp_type;
	bool sfp_setup_needed;
	u32 revision;
	enum yusur2_media_type media_type;
	u32 phy_semaphore_mask;
	bool reset_disable;
	yusur2_autoneg_advertised autoneg_advertised;
	yusur2_link_speed speeds_supported;
	yusur2_link_speed eee_speeds_supported;
	yusur2_link_speed eee_speeds_advertised;
	enum yusur2_smart_speed smart_speed;
	bool smart_speed_active;
	bool multispeed_fiber;
	bool reset_if_overtemp;
	bool qsfp_shared_i2c_bus;
	u32 nw_mng_if_sel;
};

#include "yusur2_mbx.h"

struct yusur2_mbx_operations {
	void (*init_params)(struct yusur2_hw *hw);
	s32  (*read)(struct yusur2_hw *, u32 *, u16,  u16);
	s32  (*write)(struct yusur2_hw *, u32 *, u16, u16);
	s32  (*read_posted)(struct yusur2_hw *, u32 *, u16,  u16);
	s32  (*write_posted)(struct yusur2_hw *, u32 *, u16, u16);
	s32  (*check_for_msg)(struct yusur2_hw *, u16);
	s32  (*check_for_ack)(struct yusur2_hw *, u16);
	s32  (*check_for_rst)(struct yusur2_hw *, u16);
};

struct yusur2_mbx_stats {
	u32 msgs_tx;
	u32 msgs_rx;

	u32 acks;
	u32 reqs;
	u32 rsts;
};

struct yusur2_mbx_info {
	struct yusur2_mbx_operations ops;
	struct yusur2_mbx_stats stats;
	u32 timeout;
	u32 usec_delay;
	u32 v2p_mailbox;
	u16 size;
};

struct yusur2_hw {
	u8 IOMEM *hw_addr;
	void *back;
	struct yusur2_mac_info mac;
	struct yusur2_addr_filter_info addr_ctrl;
	struct yusur2_fc_info fc;
	struct yusur2_phy_info phy;
	struct yusur2_link_info link;
	struct yusur2_eeprom_info eeprom;
	struct yusur2_bus_info bus;
	struct yusur2_mbx_info mbx;
	const u32 *mvals;
	u16 device_id;
	u16 vendor_id;
	u16 subsystem_device_id;
	u16 subsystem_vendor_id;
	u8 revision_id;
	bool adapter_stopped;
	int api_version;
	bool force_full_reset;
	bool allow_unsupported_sfp;
	bool wol_enabled;
	bool need_crosstalk_fix;
};

#define yusur2_call_func(hw, func, params, error) \
		(func != NULL) ? func params : error


/* Error Codes */
#define YUSUR2_SUCCESS				0
#define YUSUR2_ERR_EEPROM			-1
#define YUSUR2_ERR_EEPROM_CHECKSUM		-2
#define YUSUR2_ERR_PHY				-3
#define YUSUR2_ERR_CONFIG			-4
#define YUSUR2_ERR_PARAM				-5
#define YUSUR2_ERR_MAC_TYPE			-6
#define YUSUR2_ERR_UNKNOWN_PHY			-7
#define YUSUR2_ERR_LINK_SETUP			-8
#define YUSUR2_ERR_ADAPTER_STOPPED		-9
#define YUSUR2_ERR_INVALID_MAC_ADDR		-10
#define YUSUR2_ERR_DEVICE_NOT_SUPPORTED		-11
#define YUSUR2_ERR_MASTER_REQUESTS_PENDING	-12
#define YUSUR2_ERR_INVALID_LINK_SETTINGS		-13
#define YUSUR2_ERR_AUTONEG_NOT_COMPLETE		-14
#define YUSUR2_ERR_RESET_FAILED			-15
#define YUSUR2_ERR_SWFW_SYNC			-16
#define YUSUR2_ERR_PHY_ADDR_INVALID		-17
#define YUSUR2_ERR_I2C				-18
#define YUSUR2_ERR_SFP_NOT_SUPPORTED		-19
#define YUSUR2_ERR_SFP_NOT_PRESENT		-20
#define YUSUR2_ERR_SFP_NO_INIT_SEQ_PRESENT	-21
#define YUSUR2_ERR_NO_SAN_ADDR_PTR		-22
#define YUSUR2_ERR_FDIR_REINIT_FAILED		-23
#define YUSUR2_ERR_EEPROM_VERSION		-24
#define YUSUR2_ERR_NO_SPACE			-25
#define YUSUR2_ERR_OVERTEMP			-26
#define YUSUR2_ERR_FC_NOT_NEGOTIATED		-27
#define YUSUR2_ERR_FC_NOT_SUPPORTED		-28
#define YUSUR2_ERR_SFP_SETUP_NOT_COMPLETE	-30
#define YUSUR2_ERR_PBA_SECTION			-31
#define YUSUR2_ERR_INVALID_ARGUMENT		-32
#define YUSUR2_ERR_HOST_INTERFACE_COMMAND	-33
#define YUSUR2_ERR_OUT_OF_MEM			-34
#define YUSUR2_ERR_FEATURE_NOT_SUPPORTED		-36
#define YUSUR2_ERR_EEPROM_PROTECTED_REGION	-37
#define YUSUR2_ERR_FDIR_CMD_INCOMPLETE		-38
#define YUSUR2_ERR_FW_RESP_INVALID		-39
#define YUSUR2_ERR_TOKEN_RETRY			-40

#define YUSUR2_NOT_IMPLEMENTED			0x7FFFFFFF

#define YUSUR2_FUSES0_GROUP(_i)		(0x11158 + ((_i) * 4))
#define YUSUR2_FUSES0_300MHZ		(1 << 5)
#define YUSUR2_FUSES0_REV_MASK		(3 << 6)

#define YUSUR2_KRM_PORT_CAR_GEN_CTRL(P)	((P) ? 0x8010 : 0x4010)
#define YUSUR2_KRM_LINK_S1(P)		((P) ? 0x8200 : 0x4200)
#define YUSUR2_KRM_LINK_CTRL_1(P)	((P) ? 0x820C : 0x420C)
#define YUSUR2_KRM_AN_CNTL_1(P)		((P) ? 0x822C : 0x422C)
#define YUSUR2_KRM_AN_CNTL_4(P)		((P) ? 0x8238 : 0x4238)
#define YUSUR2_KRM_AN_CNTL_8(P)		((P) ? 0x8248 : 0x4248)
#define YUSUR2_KRM_PCS_KX_AN(P)		((P) ? 0x9918 : 0x5918)
#define YUSUR2_KRM_PCS_KX_AN_LP(P)	((P) ? 0x991C : 0x591C)
#define YUSUR2_KRM_SGMII_CTRL(P)		((P) ? 0x82A0 : 0x42A0)
#define YUSUR2_KRM_LP_BASE_PAGE_HIGH(P)	((P) ? 0x836C : 0x436C)
#define YUSUR2_KRM_DSP_TXFFE_STATE_4(P)	((P) ? 0x8634 : 0x4634)
#define YUSUR2_KRM_DSP_TXFFE_STATE_5(P)	((P) ? 0x8638 : 0x4638)
#define YUSUR2_KRM_RX_TRN_LINKUP_CTRL(P)	((P) ? 0x8B00 : 0x4B00)
#define YUSUR2_KRM_PMD_DFX_BURNIN(P)	((P) ? 0x8E00 : 0x4E00)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20(P)	((P) ? 0x9054 : 0x5054)
#define YUSUR2_KRM_TX_COEFF_CTRL_1(P)	((P) ? 0x9520 : 0x5520)
#define YUSUR2_KRM_RX_ANA_CTL(P)		((P) ? 0x9A00 : 0x5A00)

#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SFI_10G_DA		~(0x3 << 20)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SFI_10G_SR		(1u << 20)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SFI_10G_LR		(0x2 << 20)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SGMII_EN		(1u << 25)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_AN37_EN		(1u << 26)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_AN_EN		(1u << 27)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_10M		~(0x7 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_100M		(1u << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_1G		(0x2 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_10G		(0x3 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_AN		(0x4 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_2_5G		(0x7 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_SPEED_MASK		(0x7 << 28)
#define YUSUR2_KRM_PMD_FLX_MASK_ST20_FW_AN_RESTART	(1u << 31)

#define YUSUR2_KRM_PORT_CAR_GEN_CTRL_NELB_32B		(1 << 9)
#define YUSUR2_KRM_PORT_CAR_GEN_CTRL_NELB_KRPCS		(1 << 11)

#define YUSUR2_KRM_LINK_CTRL_1_TETH_FORCE_SPEED_MASK	(0x7 << 8)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_FORCE_SPEED_1G	(2 << 8)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_FORCE_SPEED_10G	(4 << 8)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_SGMII_EN		(1 << 12)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_CLAUSE_37_EN	(1 << 13)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_FEC_REQ		(1 << 14)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_CAP_FEC		(1 << 15)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_CAP_KX		(1 << 16)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_CAP_KR		(1 << 18)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_EEE_CAP_KX		(1 << 24)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_EEE_CAP_KR		(1 << 26)
#define YUSUR2_KRM_LINK_S1_MAC_AN_COMPLETE		(1 << 28)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_ENABLE		(1 << 29)
#define YUSUR2_KRM_LINK_CTRL_1_TETH_AN_RESTART		(1 << 31)

#define YUSUR2_KRM_AN_CNTL_1_SYM_PAUSE			(1 << 28)
#define YUSUR2_KRM_AN_CNTL_1_ASM_PAUSE			(1 << 29)
#define YUSUR2_KRM_PCS_KX_AN_SYM_PAUSE			(1 << 1)
#define YUSUR2_KRM_PCS_KX_AN_ASM_PAUSE			(1 << 2)
#define YUSUR2_KRM_PCS_KX_AN_LP_SYM_PAUSE		(1 << 2)
#define YUSUR2_KRM_PCS_KX_AN_LP_ASM_PAUSE		(1 << 3)
#define YUSUR2_KRM_AN_CNTL_4_ECSR_AN37_OVER_73		(1 << 29)
#define YUSUR2_KRM_AN_CNTL_8_LINEAR			(1 << 0)
#define YUSUR2_KRM_AN_CNTL_8_LIMITING			(1 << 1)

#define YUSUR2_KRM_LP_BASE_PAGE_HIGH_SYM_PAUSE		(1 << 10)
#define YUSUR2_KRM_LP_BASE_PAGE_HIGH_ASM_PAUSE		(1 << 11)

#define YUSUR2_KRM_SGMII_CTRL_MAC_TAR_FORCE_100_D	(1 << 12)
#define YUSUR2_KRM_SGMII_CTRL_MAC_TAR_FORCE_10_D		(1 << 19)

#define YUSUR2_KRM_DSP_TXFFE_STATE_C0_EN			(1 << 6)
#define YUSUR2_KRM_DSP_TXFFE_STATE_CP1_CN1_EN		(1 << 15)
#define YUSUR2_KRM_DSP_TXFFE_STATE_CO_ADAPT_EN		(1 << 16)

#define YUSUR2_KRM_RX_TRN_LINKUP_CTRL_CONV_WO_PROTOCOL	(1 << 4)
#define YUSUR2_KRM_RX_TRN_LINKUP_CTRL_PROTOCOL_BYPASS	(1 << 2)

#define YUSUR2_KRM_PMD_DFX_BURNIN_TX_RX_KR_LB_MASK	(0x3 << 16)

#define YUSUR2_KRM_TX_COEFF_CTRL_1_CMINUS1_OVRRD_EN	(1 << 1)
#define YUSUR2_KRM_TX_COEFF_CTRL_1_CPLUS1_OVRRD_EN	(1 << 2)
#define YUSUR2_KRM_TX_COEFF_CTRL_1_CZERO_EN		(1 << 3)
#define YUSUR2_KRM_TX_COEFF_CTRL_1_OVRRD_EN		(1 << 31)

#define YUSUR2_SB_IOSF_INDIRECT_CTRL	0x00011144
#define YUSUR2_SB_IOSF_INDIRECT_DATA	0x00011148

#define YUSUR2_SB_IOSF_CTRL_ADDR_SHIFT		0
#define YUSUR2_SB_IOSF_CTRL_ADDR_MASK		0xFF
#define YUSUR2_SB_IOSF_CTRL_RESP_STAT_SHIFT	18
#define YUSUR2_SB_IOSF_CTRL_RESP_STAT_MASK	\
				(0x3 << YUSUR2_SB_IOSF_CTRL_RESP_STAT_SHIFT)
#define YUSUR2_SB_IOSF_CTRL_CMPL_ERR_SHIFT	20
#define YUSUR2_SB_IOSF_CTRL_CMPL_ERR_MASK	\
				(0xFF << YUSUR2_SB_IOSF_CTRL_CMPL_ERR_SHIFT)
#define YUSUR2_SB_IOSF_CTRL_TARGET_SELECT_SHIFT	28
#define YUSUR2_SB_IOSF_CTRL_TARGET_SELECT_MASK	0x7
#define YUSUR2_SB_IOSF_CTRL_BUSY_SHIFT		31
#define YUSUR2_SB_IOSF_CTRL_BUSY		(1 << YUSUR2_SB_IOSF_CTRL_BUSY_SHIFT)
#define YUSUR2_SB_IOSF_TARGET_KR_PHY	0

#define YUSUR2_NW_MNG_IF_SEL		0x00011178
#define YUSUR2_NW_MNG_IF_SEL_MDIO_ACT	(1u << 1)
#define YUSUR2_NW_MNG_IF_SEL_MDIO_IF_MODE	(1u << 2)
#define YUSUR2_NW_MNG_IF_SEL_EN_SHARED_MDIO	(1u << 13)
#define YUSUR2_NW_MNG_IF_SEL_PHY_SPEED_10M	(1u << 17)
#define YUSUR2_NW_MNG_IF_SEL_PHY_SPEED_100M	(1u << 18)
#define YUSUR2_NW_MNG_IF_SEL_PHY_SPEED_1G	(1u << 19)
#define YUSUR2_NW_MNG_IF_SEL_PHY_SPEED_2_5G	(1u << 20)
#define YUSUR2_NW_MNG_IF_SEL_PHY_SPEED_10G	(1u << 21)
#define YUSUR2_NW_MNG_IF_SEL_SGMII_ENABLE	(1u << 25)
#define YUSUR2_NW_MNG_IF_SEL_INT_PHY_MODE (1 << 24) /* X552 reg field only */
#define YUSUR2_NW_MNG_IF_SEL_MDIO_PHY_ADD_SHIFT 3
#define YUSUR2_NW_MNG_IF_SEL_MDIO_PHY_ADD	\
				(0x1F << YUSUR2_NW_MNG_IF_SEL_MDIO_PHY_ADD_SHIFT)

/* Code Command (Flash I/F Interface) */
#define YUSUR2_HOST_INTERFACE_FLASH_READ_CMD			0x30
#define YUSUR2_HOST_INTERFACE_SHADOW_RAM_READ_CMD		0x31
#define YUSUR2_HOST_INTERFACE_FLASH_WRITE_CMD			0x32
#define YUSUR2_HOST_INTERFACE_SHADOW_RAM_WRITE_CMD		0x33
#define YUSUR2_HOST_INTERFACE_FLASH_MODULE_UPDATE_CMD		0x34
#define YUSUR2_HOST_INTERFACE_FLASH_BLOCK_EREASE_CMD		0x35
#define YUSUR2_HOST_INTERFACE_SHADOW_RAM_DUMP_CMD		0x36
#define YUSUR2_HOST_INTERFACE_FLASH_INFO_CMD			0x37
#define YUSUR2_HOST_INTERFACE_APPLY_UPDATE_CMD			0x38
#define YUSUR2_HOST_INTERFACE_MASK_CMD				0x000000FF

#endif /* _YUSUR2_TYPE_H_ */
