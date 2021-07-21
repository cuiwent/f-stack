/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_PHY_H_
#define _YUSUR2_PHY_H_

#include "yusur2_type.h"
#define YUSUR2_I2C_EEPROM_DEV_ADDR	0xA0
#define YUSUR2_I2C_EEPROM_DEV_ADDR2	0xA2
#define YUSUR2_I2C_EEPROM_BANK_LEN	0xFF

/* EEPROM byte offsets */
#define YUSUR2_SFF_IDENTIFIER		0x0
#define YUSUR2_SFF_IDENTIFIER_SFP	0x3
#define YUSUR2_SFF_VENDOR_OUI_BYTE0	0x25
#define YUSUR2_SFF_VENDOR_OUI_BYTE1	0x26
#define YUSUR2_SFF_VENDOR_OUI_BYTE2	0x27
#define YUSUR2_SFF_1GBE_COMP_CODES	0x6
#define YUSUR2_SFF_10GBE_COMP_CODES	0x3
#define YUSUR2_SFF_CABLE_TECHNOLOGY	0x8
#define YUSUR2_SFF_CABLE_SPEC_COMP	0x3C
#define YUSUR2_SFF_SFF_8472_SWAP		0x5C
#define YUSUR2_SFF_SFF_8472_COMP		0x5E
#define YUSUR2_SFF_SFF_8472_OSCB		0x6E
#define YUSUR2_SFF_SFF_8472_ESCB		0x76
#define YUSUR2_SFF_IDENTIFIER_QSFP_PLUS	0xD
#define YUSUR2_SFF_QSFP_VENDOR_OUI_BYTE0	0xA5
#define YUSUR2_SFF_QSFP_VENDOR_OUI_BYTE1	0xA6
#define YUSUR2_SFF_QSFP_VENDOR_OUI_BYTE2	0xA7
#define YUSUR2_SFF_QSFP_CONNECTOR	0x82
#define YUSUR2_SFF_QSFP_10GBE_COMP	0x83
#define YUSUR2_SFF_QSFP_1GBE_COMP	0x86
#define YUSUR2_SFF_QSFP_CABLE_LENGTH	0x92
#define YUSUR2_SFF_QSFP_DEVICE_TECH	0x93

/* Bitmasks */
#define YUSUR2_SFF_DA_PASSIVE_CABLE	0x4
#define YUSUR2_SFF_DA_ACTIVE_CABLE	0x8
#define YUSUR2_SFF_DA_SPEC_ACTIVE_LIMITING	0x4
#define YUSUR2_SFF_1GBASESX_CAPABLE	0x1
#define YUSUR2_SFF_1GBASELX_CAPABLE	0x2
#define YUSUR2_SFF_1GBASET_CAPABLE	0x8
#define YUSUR2_SFF_1GBASELHA_CAPABLE	0x10
#define YUSUR2_SFF_10GBASESR_CAPABLE	0x10
#define YUSUR2_SFF_10GBASELR_CAPABLE	0x20
#define YUSUR2_SFF_SOFT_RS_SELECT_MASK	0x8
#define YUSUR2_SFF_SOFT_RS_SELECT_10G	0x8
#define YUSUR2_SFF_SOFT_RS_SELECT_1G	0x0
#define YUSUR2_SFF_ADDRESSING_MODE	0x4
#define YUSUR2_SFF_QSFP_DA_ACTIVE_CABLE	0x1
#define YUSUR2_SFF_QSFP_DA_PASSIVE_CABLE	0x8
#define YUSUR2_SFF_QSFP_CONNECTOR_NOT_SEPARABLE	0x23
#define YUSUR2_SFF_QSFP_TRANSMITER_850NM_VCSEL	0x0
#define YUSUR2_I2C_EEPROM_READ_MASK	0x100
#define YUSUR2_I2C_EEPROM_STATUS_MASK	0x3
#define YUSUR2_I2C_EEPROM_STATUS_NO_OPERATION	0x0
#define YUSUR2_I2C_EEPROM_STATUS_PASS	0x1
#define YUSUR2_I2C_EEPROM_STATUS_FAIL	0x2
#define YUSUR2_I2C_EEPROM_STATUS_IN_PROGRESS	0x3

#define YUSUR2_CS4227			0xBE	/* CS4227 address */
#define YUSUR2_CS4227_GLOBAL_ID_LSB	0
#define YUSUR2_CS4227_GLOBAL_ID_MSB	1
#define YUSUR2_CS4227_SCRATCH		2
#define YUSUR2_CS4227_GLOBAL_ID_VALUE	0x03E5
#define YUSUR2_CS4227_EFUSE_PDF_SKU	0x19F
#define YUSUR2_CS4223_SKU_ID		0x0010	/* Quad port */
#define YUSUR2_CS4227_SKU_ID		0x0014	/* Dual port */
#define YUSUR2_CS4227_RESET_PENDING	0x1357
#define YUSUR2_CS4227_RESET_COMPLETE	0x5AA5
#define YUSUR2_CS4227_RETRIES		15
#define YUSUR2_CS4227_EFUSE_STATUS	0x0181
#define YUSUR2_CS4227_LINE_SPARE22_MSB	0x12AD	/* Reg to program speed */
#define YUSUR2_CS4227_LINE_SPARE24_LSB	0x12B0	/* Reg to program EDC */
#define YUSUR2_CS4227_HOST_SPARE22_MSB	0x1AAD	/* Reg to program speed */
#define YUSUR2_CS4227_HOST_SPARE24_LSB	0x1AB0	/* Reg to program EDC */
#define YUSUR2_CS4227_EEPROM_STATUS	0x5001
#define YUSUR2_CS4227_EEPROM_LOAD_OK	0x0001
#define YUSUR2_CS4227_SPEED_1G		0x8000
#define YUSUR2_CS4227_SPEED_10G		0
#define YUSUR2_CS4227_EDC_MODE_CX1	0x0002
#define YUSUR2_CS4227_EDC_MODE_SR	0x0004
#define YUSUR2_CS4227_EDC_MODE_DIAG	0x0008
#define YUSUR2_CS4227_RESET_HOLD		500	/* microseconds */
#define YUSUR2_CS4227_RESET_DELAY	450	/* milliseconds */
#define YUSUR2_CS4227_CHECK_DELAY	30	/* milliseconds */
#define YUSUR2_PE			0xE0	/* Port expander address */
#define YUSUR2_PE_OUTPUT			1	/* Output register offset */
#define YUSUR2_PE_CONFIG			3	/* Config register offset */
#define YUSUR2_PE_BIT1			(1 << 1)

/* Flow control defines */
#define YUSUR2_TAF_SYM_PAUSE		0x400
#define YUSUR2_TAF_ASM_PAUSE		0x800

/* Bit-shift macros */
#define YUSUR2_SFF_VENDOR_OUI_BYTE0_SHIFT	24
#define YUSUR2_SFF_VENDOR_OUI_BYTE1_SHIFT	16
#define YUSUR2_SFF_VENDOR_OUI_BYTE2_SHIFT	8

/* Vendor OUIs: format of OUI is 0x[byte0][byte1][byte2][00] */
#define YUSUR2_SFF_VENDOR_OUI_TYCO	0x00407600
#define YUSUR2_SFF_VENDOR_OUI_FTL	0x00906500
#define YUSUR2_SFF_VENDOR_OUI_AVAGO	0x00176A00
#define YUSUR2_SFF_VENDOR_OUI_INTEL	0x001B2100

/* I2C SDA and SCL timing parameters for standard mode */
#define YUSUR2_I2C_T_HD_STA	4
#define YUSUR2_I2C_T_LOW		5
#define YUSUR2_I2C_T_HIGH	4
#define YUSUR2_I2C_T_SU_STA	5
#define YUSUR2_I2C_T_HD_DATA	5
#define YUSUR2_I2C_T_SU_DATA	1
#define YUSUR2_I2C_T_RISE	1
#define YUSUR2_I2C_T_FALL	1
#define YUSUR2_I2C_T_SU_STO	4
#define YUSUR2_I2C_T_BUF		5

#ifndef YUSUR2_SFP_DETECT_RETRIES
#define YUSUR2_SFP_DETECT_RETRIES	10

#endif /* YUSUR2_SFP_DETECT_RETRIES */
#define YUSUR2_TN_LASI_STATUS_REG	0x9005
#define YUSUR2_TN_LASI_STATUS_TEMP_ALARM	0x0008

/* SFP+ SFF-8472 Compliance */
#define YUSUR2_SFF_SFF_8472_UNSUP	0x00

s32 yusur2_init_phy_ops_generic(struct yusur2_hw *hw);
bool yusur2_validate_phy_addr(struct yusur2_hw *hw, u32 phy_addr);
enum yusur2_phy_type yusur2_get_phy_type_from_id(u32 phy_id);
s32 yusur2_get_phy_id(struct yusur2_hw *hw);
s32 yusur2_identify_phy_generic(struct yusur2_hw *hw);
s32 yusur2_reset_phy_generic(struct yusur2_hw *hw);
s32 yusur2_read_phy_reg_mdi(struct yusur2_hw *hw, u32 reg_addr, u32 device_type,
			   u16 *phy_data);
s32 yusur2_write_phy_reg_mdi(struct yusur2_hw *hw, u32 reg_addr, u32 device_type,
			    u16 phy_data);
s32 yusur2_read_phy_reg_generic(struct yusur2_hw *hw, u32 reg_addr,
			       u32 device_type, u16 *phy_data);
s32 yusur2_write_phy_reg_generic(struct yusur2_hw *hw, u32 reg_addr,
				u32 device_type, u16 phy_data);
s32 yusur2_setup_phy_link_generic(struct yusur2_hw *hw);
s32 yusur2_setup_phy_link_speed_generic(struct yusur2_hw *hw,
				       yusur2_link_speed speed,
				       bool autoneg_wait_to_complete);
s32 yusur2_get_copper_link_capabilities_generic(struct yusur2_hw *hw,
					       yusur2_link_speed *speed,
					       bool *autoneg);
s32 yusur2_check_reset_blocked(struct yusur2_hw *hw);

/* PHY specific */
s32 yusur2_check_phy_link_tnx(struct yusur2_hw *hw,
			     yusur2_link_speed *speed,
			     bool *link_up);
s32 yusur2_setup_phy_link_tnx(struct yusur2_hw *hw);
s32 yusur2_get_phy_firmware_version_tnx(struct yusur2_hw *hw,
				       u16 *firmware_version);
s32 yusur2_get_phy_firmware_version_generic(struct yusur2_hw *hw,
					   u16 *firmware_version);

s32 yusur2_reset_phy_nl(struct yusur2_hw *hw);
s32 yusur2_set_copper_phy_power(struct yusur2_hw *hw, bool on);
s32 yusur2_identify_module_generic(struct yusur2_hw *hw);
s32 yusur2_identify_sfp_module_generic(struct yusur2_hw *hw);
u64 yusur2_get_supported_phy_sfp_layer_generic(struct yusur2_hw *hw);
s32 yusur2_identify_qsfp_module_generic(struct yusur2_hw *hw);
s32 yusur2_get_sfp_init_sequence_offsets(struct yusur2_hw *hw,
					u16 *list_offset,
					u16 *data_offset);
s32 yusur2_tn_check_overtemp(struct yusur2_hw *hw);
s32 yusur2_read_i2c_byte_generic(struct yusur2_hw *hw, u8 byte_offset,
				u8 dev_addr, u8 *data);
s32 yusur2_read_i2c_byte_generic_unlocked(struct yusur2_hw *hw, u8 byte_offset,
					 u8 dev_addr, u8 *data);
s32 yusur2_write_i2c_byte_generic(struct yusur2_hw *hw, u8 byte_offset,
				 u8 dev_addr, u8 data);
s32 yusur2_write_i2c_byte_generic_unlocked(struct yusur2_hw *hw, u8 byte_offset,
					  u8 dev_addr, u8 data);
s32 yusur2_read_i2c_eeprom_generic(struct yusur2_hw *hw, u8 byte_offset,
				  u8 *eeprom_data);
s32 yusur2_write_i2c_eeprom_generic(struct yusur2_hw *hw, u8 byte_offset,
				   u8 eeprom_data);
void yusur2_i2c_bus_clear(struct yusur2_hw *hw);
s32 yusur2_read_i2c_combined_generic_int(struct yusur2_hw *, u8 addr, u16 reg,
					u16 *val, bool lock);
s32 yusur2_write_i2c_combined_generic_int(struct yusur2_hw *, u8 addr, u16 reg,
					 u16 val, bool lock);
#endif /* _YUSUR2_PHY_H_ */
