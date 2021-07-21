/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_82599_H_
#define _YUSUR2_82599_H_

s32 yusur2_get_link_capabilities_82599(struct yusur2_hw *hw,
				      yusur2_link_speed *speed, bool *autoneg);
enum yusur2_media_type yusur2_get_media_type_82599(struct yusur2_hw *hw);
void yusur2_disable_tx_laser_multispeed_fiber(struct yusur2_hw *hw);
void yusur2_enable_tx_laser_multispeed_fiber(struct yusur2_hw *hw);
void yusur2_flap_tx_laser_multispeed_fiber(struct yusur2_hw *hw);
void yusur2_set_hard_rate_select_speed(struct yusur2_hw *hw,
					yusur2_link_speed speed);
s32 yusur2_setup_mac_link_smartspeed(struct yusur2_hw *hw,
				    yusur2_link_speed speed,
				    bool autoneg_wait_to_complete);
s32 yusur2_start_mac_link_82599(struct yusur2_hw *hw,
			       bool autoneg_wait_to_complete);
s32 yusur2_setup_mac_link_82599(struct yusur2_hw *hw, yusur2_link_speed speed,
			       bool autoneg_wait_to_complete);
s32 yusur2_setup_sfp_modules_82599(struct yusur2_hw *hw);
void yusur2_init_mac_link_ops_82599(struct yusur2_hw *hw);
s32 yusur2_reset_hw_82599(struct yusur2_hw *hw);
s32 yusur2_read_analog_reg8_82599(struct yusur2_hw *hw, u32 reg, u8 *val);
s32 yusur2_write_analog_reg8_82599(struct yusur2_hw *hw, u32 reg, u8 val);
s32 yusur2_start_hw_82599(struct yusur2_hw *hw);
s32 yusur2_identify_phy_82599(struct yusur2_hw *hw);
s32 yusur2_init_phy_ops_82599(struct yusur2_hw *hw);
u64 yusur2_get_supported_physical_layer_82599(struct yusur2_hw *hw);
s32 yusur2_enable_rx_dma_82599(struct yusur2_hw *hw, u32 regval);
s32 yusur2_prot_autoc_read_82599(struct yusur2_hw *hw, bool *locked, u32 *reg_val);
s32 yusur2_prot_autoc_write_82599(struct yusur2_hw *hw, u32 reg_val, bool locked);
#endif /* _YUSUR2_82599_H_ */
