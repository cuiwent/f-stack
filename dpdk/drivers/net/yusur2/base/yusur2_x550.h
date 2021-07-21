/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_X550_H_
#define _YUSUR2_X550_H_

#include "yusur2_type.h"

s32 yusur2_dmac_config_X550(struct yusur2_hw *hw);
s32 yusur2_dmac_config_tcs_X550(struct yusur2_hw *hw);
s32 yusur2_dmac_update_tcs_X550(struct yusur2_hw *hw);

s32 yusur2_get_bus_info_X550em(struct yusur2_hw *hw);
s32 yusur2_init_eeprom_params_X550(struct yusur2_hw *hw);
s32 yusur2_update_eeprom_checksum_X550(struct yusur2_hw *hw);
s32 yusur2_calc_eeprom_checksum_X550(struct yusur2_hw *hw);
s32 yusur2_calc_checksum_X550(struct yusur2_hw *hw, u16 *buffer, u32 buffer_size);
s32 yusur2_validate_eeprom_checksum_X550(struct yusur2_hw *hw, u16 *checksum_val);
s32 yusur2_update_flash_X550(struct yusur2_hw *hw);
s32 yusur2_write_ee_hostif_buffer_X550(struct yusur2_hw *hw,
				      u16 offset, u16 words, u16 *data);
s32 yusur2_write_ee_hostif_X550(struct yusur2_hw *hw, u16 offset,
			       u16 data);
s32 yusur2_read_ee_hostif_buffer_X550(struct yusur2_hw *hw,
				     u16 offset, u16 words, u16 *data);
s32 yusur2_read_ee_hostif_X550(struct yusur2_hw *hw, u16 offset,
u16				*data);
s32 yusur2_write_ee_hostif_data_X550(struct yusur2_hw *hw, u16 offset,
				    u16 data);
void yusur2_set_source_address_pruning_X550(struct yusur2_hw *hw, bool enable,
					   unsigned int pool);
void yusur2_set_ethertype_anti_spoofing_X550(struct yusur2_hw *hw,
					    bool enable, int vf);
s32 yusur2_write_iosf_sb_reg_x550(struct yusur2_hw *hw, u32 reg_addr,
				 u32 device_type, u32 data);
s32 yusur2_read_iosf_sb_reg_x550(struct yusur2_hw *hw, u32 reg_addr,
	u32 device_type, u32 *data);
s32 yusur2_set_fw_drv_ver_x550(struct yusur2_hw *hw, u8 maj, u8 min,
			      u8 build, u8 ver, u16 len, const char *str);
s32 yusur2_get_phy_token(struct yusur2_hw *);
s32 yusur2_put_phy_token(struct yusur2_hw *);
s32 yusur2_write_iosf_sb_reg_x550a(struct yusur2_hw *hw, u32 reg_addr,
	u32 device_type, u32 data);
s32 yusur2_read_iosf_sb_reg_x550a(struct yusur2_hw *hw, u32 reg_addr,
	u32 device_type, u32 *data);
void yusur2_disable_mdd_X550(struct yusur2_hw *hw);
void yusur2_enable_mdd_X550(struct yusur2_hw *hw);
void yusur2_mdd_event_X550(struct yusur2_hw *hw, u32 *vf_bitmap);
void yusur2_restore_mdd_vf_X550(struct yusur2_hw *hw, u32 vf);
enum yusur2_media_type yusur2_get_media_type_X550em(struct yusur2_hw *hw);
s32 yusur2_setup_sfp_modules_X550em(struct yusur2_hw *hw);
s32 yusur2_get_link_capabilities_X550em(struct yusur2_hw *hw,
				       yusur2_link_speed *speed, bool *autoneg);
void yusur2_init_mac_link_ops_X550em(struct yusur2_hw *hw);
s32 yusur2_reset_hw_X550em(struct yusur2_hw *hw);
s32 yusur2_init_phy_ops_X550em(struct yusur2_hw *hw);
s32 yusur2_setup_kr_x550em(struct yusur2_hw *hw);
s32 yusur2_init_ext_t_x550em(struct yusur2_hw *hw);
s32 yusur2_setup_internal_phy_t_x550em(struct yusur2_hw *hw);
s32 yusur2_setup_phy_loopback_x550em(struct yusur2_hw *hw);
u64 yusur2_get_supported_physical_layer_X550em(struct yusur2_hw *hw);
void yusur2_disable_rx_x550(struct yusur2_hw *hw);
s32 yusur2_get_lcd_t_x550em(struct yusur2_hw *hw, yusur2_link_speed *lcd_speed);
s32 yusur2_enter_lplu_t_x550em(struct yusur2_hw *hw);
s32 yusur2_acquire_swfw_sync_X550em(struct yusur2_hw *hw, u32 mask);
void yusur2_release_swfw_sync_X550em(struct yusur2_hw *hw, u32 mask);
s32 yusur2_setup_fc_X550em(struct yusur2_hw *hw);
s32 yusur2_setup_mac_link_sfp_x550em(struct yusur2_hw *hw,
				    yusur2_link_speed speed,
				    bool autoneg_wait_to_complete);
s32 yusur2_setup_mac_link_sfp_x550a(struct yusur2_hw *hw,
				    yusur2_link_speed speed,
				    bool autoneg_wait_to_complete);
s32 yusur2_read_phy_reg_x550a(struct yusur2_hw *hw, u32 reg_addr,
			       u32 device_type, u16 *phy_data);
s32 yusur2_write_phy_reg_x550a(struct yusur2_hw *hw, u32 reg_addr,
				u32 device_type, u16 phy_data);
s32 yusur2_setup_fc_fiber_x550em_a(struct yusur2_hw *hw);
s32 yusur2_setup_fc_backplane_x550em_a(struct yusur2_hw *hw);
s32 yusur2_setup_fc_sgmii_x550em_a(struct yusur2_hw *hw);
void yusur2_fc_autoneg_fiber_x550em_a(struct yusur2_hw *hw);
void yusur2_fc_autoneg_backplane_x550em_a(struct yusur2_hw *hw);
void yusur2_fc_autoneg_sgmii_x550em_a(struct yusur2_hw *hw);
s32 yusur2_handle_lasi_ext_t_x550em(struct yusur2_hw *hw);
s32 yusur2_setup_mac_link_t_X550em(struct yusur2_hw *hw,
				  yusur2_link_speed speed,
				  bool autoneg_wait_to_complete);
s32 yusur2_check_link_t_X550em(struct yusur2_hw *hw, yusur2_link_speed *speed,
			      bool *link_up, bool link_up_wait_to_complete);
s32 yusur2_reset_phy_t_X550em(struct yusur2_hw *hw);
s32 yusur2_identify_sfp_module_X550em(struct yusur2_hw *hw);
s32 yusur2_led_on_t_X550em(struct yusur2_hw *hw, u32 led_idx);
s32 yusur2_led_off_t_X550em(struct yusur2_hw *hw, u32 led_idx);
bool yusur2_fw_recovery_mode_X550(struct yusur2_hw *hw);
#endif /* _YUSUR2_X550_H_ */
