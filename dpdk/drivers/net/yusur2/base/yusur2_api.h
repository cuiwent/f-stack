/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_API_H_
#define _YUSUR2_API_H_

#include "yusur2_type.h"

void yusur2_dcb_get_rtrup2tc(struct yusur2_hw *hw, u8 *map);

s32 yusur2_init_shared_code(struct yusur2_hw *hw);

extern s32 yusur2_init_ops_82598(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_82599(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_X540(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_X550(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_X550EM(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_X550EM_x(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_X550EM_a(struct yusur2_hw *hw);
extern s32 yusur2_init_ops_vf(struct yusur2_hw *hw);

s32 yusur2_set_mac_type(struct yusur2_hw *hw);
s32 yusur2_init_hw(struct yusur2_hw *hw);
s32 yusur2_reset_hw(struct yusur2_hw *hw);
s32 yusur2_start_hw(struct yusur2_hw *hw);
void yusur2_enable_relaxed_ordering(struct yusur2_hw *hw);
s32 yusur2_clear_hw_cntrs(struct yusur2_hw *hw);
enum yusur2_media_type yusur2_get_media_type(struct yusur2_hw *hw);
s32 yusur2_get_mac_addr(struct yusur2_hw *hw, u8 *mac_addr);
s32 yusur2_get_bus_info(struct yusur2_hw *hw);
u32 yusur2_get_num_of_tx_queues(struct yusur2_hw *hw);
u32 yusur2_get_num_of_rx_queues(struct yusur2_hw *hw);
s32 yusur2_stop_adapter(struct yusur2_hw *hw);
s32 yusur2_read_pba_num(struct yusur2_hw *hw, u32 *pba_num);
s32 yusur2_read_pba_string(struct yusur2_hw *hw, u8 *pba_num, u32 pba_num_size);

s32 yusur2_identify_phy(struct yusur2_hw *hw);
s32 yusur2_reset_phy(struct yusur2_hw *hw);
s32 yusur2_read_phy_reg(struct yusur2_hw *hw, u32 reg_addr, u32 device_type,
		       u16 *phy_data);
s32 yusur2_write_phy_reg(struct yusur2_hw *hw, u32 reg_addr, u32 device_type,
			u16 phy_data);

s32 yusur2_setup_phy_link(struct yusur2_hw *hw);
s32 yusur2_setup_internal_phy(struct yusur2_hw *hw);
s32 yusur2_check_phy_link(struct yusur2_hw *hw,
			 yusur2_link_speed *speed,
			 bool *link_up);
s32 yusur2_setup_phy_link_speed(struct yusur2_hw *hw,
			       yusur2_link_speed speed,
			       bool autoneg_wait_to_complete);
s32 yusur2_set_phy_power(struct yusur2_hw *, bool on);
void yusur2_disable_tx_laser(struct yusur2_hw *hw);
void yusur2_enable_tx_laser(struct yusur2_hw *hw);
void yusur2_flap_tx_laser(struct yusur2_hw *hw);
s32 yusur2_setup_link(struct yusur2_hw *hw, yusur2_link_speed speed,
		     bool autoneg_wait_to_complete);
s32 yusur2_setup_mac_link(struct yusur2_hw *hw, yusur2_link_speed speed,
			 bool autoneg_wait_to_complete);
s32 yusur2_check_link(struct yusur2_hw *hw, yusur2_link_speed *speed,
		     bool *link_up, bool link_up_wait_to_complete);
s32 yusur2_get_link_capabilities(struct yusur2_hw *hw, yusur2_link_speed *speed,
				bool *autoneg);
s32 yusur2_led_on(struct yusur2_hw *hw, u32 index);
s32 yusur2_led_off(struct yusur2_hw *hw, u32 index);
s32 yusur2_blink_led_start(struct yusur2_hw *hw, u32 index);
s32 yusur2_blink_led_stop(struct yusur2_hw *hw, u32 index);

s32 yusur2_init_eeprom_params(struct yusur2_hw *hw);
s32 yusur2_write_eeprom(struct yusur2_hw *hw, u16 offset, u16 data);
s32 yusur2_write_eeprom_buffer(struct yusur2_hw *hw, u16 offset,
			      u16 words, u16 *data);
s32 yusur2_read_eeprom(struct yusur2_hw *hw, u16 offset, u16 *data);
s32 yusur2_read_eeprom_buffer(struct yusur2_hw *hw, u16 offset,
			     u16 words, u16 *data);

s32 yusur2_validate_eeprom_checksum(struct yusur2_hw *hw, u16 *checksum_val);
s32 yusur2_update_eeprom_checksum(struct yusur2_hw *hw);

s32 yusur2_insert_mac_addr(struct yusur2_hw *hw, u8 *addr, u32 vmdq);
s32 yusur2_set_rar(struct yusur2_hw *hw, u32 index, u8 *addr, u32 vmdq,
		  u32 enable_addr);
s32 yusur2_clear_rar(struct yusur2_hw *hw, u32 index);
s32 yusur2_set_vmdq(struct yusur2_hw *hw, u32 rar, u32 vmdq);
s32 yusur2_set_vmdq_san_mac(struct yusur2_hw *hw, u32 vmdq);
s32 yusur2_clear_vmdq(struct yusur2_hw *hw, u32 rar, u32 vmdq);
s32 yusur2_init_rx_addrs(struct yusur2_hw *hw);
u32 yusur2_get_num_rx_addrs(struct yusur2_hw *hw);
s32 yusur2_update_uc_addr_list(struct yusur2_hw *hw, u8 *addr_list,
			      u32 addr_count, yusur2_mc_addr_itr func);
s32 yusur2_update_mc_addr_list(struct yusur2_hw *hw, u8 *mc_addr_list,
			      u32 mc_addr_count, yusur2_mc_addr_itr func,
			      bool clear);
void yusur2_add_uc_addr(struct yusur2_hw *hw, u8 *addr_list, u32 vmdq);
s32 yusur2_enable_mc(struct yusur2_hw *hw);
s32 yusur2_disable_mc(struct yusur2_hw *hw);
s32 yusur2_clear_vfta(struct yusur2_hw *hw);
s32 yusur2_set_vfta(struct yusur2_hw *hw, u32 vlan,
		   u32 vind, bool vlan_on, bool vlvf_bypass);
s32 yusur2_set_vlvf(struct yusur2_hw *hw, u32 vlan, u32 vind,
		   bool vlan_on, u32 *vfta_delta, u32 vfta,
		   bool vlvf_bypass);
s32 yusur2_fc_enable(struct yusur2_hw *hw);
s32 yusur2_setup_fc(struct yusur2_hw *hw);
s32 yusur2_set_fw_drv_ver(struct yusur2_hw *hw, u8 maj, u8 min, u8 build,
			 u8 ver, u16 len, char *driver_ver);
s32 yusur2_get_thermal_sensor_data(struct yusur2_hw *hw);
s32 yusur2_init_thermal_sensor_thresh(struct yusur2_hw *hw);
void yusur2_set_mta(struct yusur2_hw *hw, u8 *mc_addr);
s32 yusur2_get_phy_firmware_version(struct yusur2_hw *hw,
				   u16 *firmware_version);
s32 yusur2_read_analog_reg8(struct yusur2_hw *hw, u32 reg, u8 *val);
s32 yusur2_write_analog_reg8(struct yusur2_hw *hw, u32 reg, u8 val);
s32 yusur2_init_uta_tables(struct yusur2_hw *hw);
s32 yusur2_read_i2c_eeprom(struct yusur2_hw *hw, u8 byte_offset, u8 *eeprom_data);
u64 yusur2_get_supported_physical_layer(struct yusur2_hw *hw);
s32 yusur2_enable_rx_dma(struct yusur2_hw *hw, u32 regval);
s32 yusur2_disable_sec_rx_path(struct yusur2_hw *hw);
s32 yusur2_enable_sec_rx_path(struct yusur2_hw *hw);
s32 yusur2_mng_fw_enabled(struct yusur2_hw *hw);
s32 yusur2_reinit_fdir_tables_82599(struct yusur2_hw *hw);
s32 yusur2_init_fdir_signature_82599(struct yusur2_hw *hw, u32 fdirctrl);
s32 yusur2_init_fdir_perfect_82599(struct yusur2_hw *hw, u32 fdirctrl,
					bool cloud_mode);
void yusur2_fdir_add_signature_filter_82599(struct yusur2_hw *hw,
					   union yusur2_atr_hash_dword input,
					   union yusur2_atr_hash_dword common,
					   u8 queue);
s32 yusur2_fdir_set_input_mask_82599(struct yusur2_hw *hw,
				    union yusur2_atr_input *input_mask, bool cloud_mode);
s32 yusur2_fdir_write_perfect_filter_82599(struct yusur2_hw *hw,
					  union yusur2_atr_input *input,
					  u16 soft_id, u8 queue, bool cloud_mode);
s32 yusur2_fdir_erase_perfect_filter_82599(struct yusur2_hw *hw,
					  union yusur2_atr_input *input,
					  u16 soft_id);
s32 yusur2_fdir_add_perfect_filter_82599(struct yusur2_hw *hw,
					union yusur2_atr_input *input,
					union yusur2_atr_input *mask,
					u16 soft_id,
					u8 queue,
					bool cloud_mode);
void yusur2_atr_compute_perfect_hash_82599(union yusur2_atr_input *input,
					  union yusur2_atr_input *mask);
u32 yusur2_atr_compute_sig_hash_82599(union yusur2_atr_hash_dword input,
				     union yusur2_atr_hash_dword common);
bool yusur2_verify_lesm_fw_enabled_82599(struct yusur2_hw *hw);
s32 yusur2_read_i2c_byte(struct yusur2_hw *hw, u8 byte_offset, u8 dev_addr,
			u8 *data);
s32 yusur2_read_i2c_byte_unlocked(struct yusur2_hw *hw, u8 byte_offset,
				 u8 dev_addr, u8 *data);
s32 yusur2_read_link(struct yusur2_hw *hw, u8 addr, u16 reg, u16 *val);
s32 yusur2_read_link_unlocked(struct yusur2_hw *hw, u8 addr, u16 reg, u16 *val);
s32 yusur2_write_i2c_byte(struct yusur2_hw *hw, u8 byte_offset, u8 dev_addr,
			 u8 data);
void yusur2_set_fdir_drop_queue_82599(struct yusur2_hw *hw, u8 dropqueue);
s32 yusur2_write_i2c_byte_unlocked(struct yusur2_hw *hw, u8 byte_offset,
				  u8 dev_addr, u8 data);
s32 yusur2_write_link(struct yusur2_hw *hw, u8 addr, u16 reg, u16 val);
s32 yusur2_write_link_unlocked(struct yusur2_hw *hw, u8 addr, u16 reg, u16 val);
s32 yusur2_write_i2c_eeprom(struct yusur2_hw *hw, u8 byte_offset, u8 eeprom_data);
s32 yusur2_get_san_mac_addr(struct yusur2_hw *hw, u8 *san_mac_addr);
s32 yusur2_set_san_mac_addr(struct yusur2_hw *hw, u8 *san_mac_addr);
s32 yusur2_get_device_caps(struct yusur2_hw *hw, u16 *device_caps);
s32 yusur2_acquire_swfw_semaphore(struct yusur2_hw *hw, u32 mask);
void yusur2_release_swfw_semaphore(struct yusur2_hw *hw, u32 mask);
void yusur2_init_swfw_semaphore(struct yusur2_hw *hw);
s32 yusur2_get_wwn_prefix(struct yusur2_hw *hw, u16 *wwnn_prefix,
			 u16 *wwpn_prefix);
s32 yusur2_get_fcoe_boot_status(struct yusur2_hw *hw, u16 *bs);
s32 yusur2_dmac_config(struct yusur2_hw *hw);
s32 yusur2_dmac_update_tcs(struct yusur2_hw *hw);
s32 yusur2_dmac_config_tcs(struct yusur2_hw *hw);
s32 yusur2_setup_eee(struct yusur2_hw *hw, bool enable_eee);
void yusur2_set_source_address_pruning(struct yusur2_hw *hw, bool enable,
				      unsigned int vf);
void yusur2_set_ethertype_anti_spoofing(struct yusur2_hw *hw, bool enable,
				       int vf);
s32 yusur2_read_iosf_sb_reg(struct yusur2_hw *hw, u32 reg_addr,
			u32 device_type, u32 *phy_data);
s32 yusur2_write_iosf_sb_reg(struct yusur2_hw *hw, u32 reg_addr,
			u32 device_type, u32 phy_data);
void yusur2_disable_mdd(struct yusur2_hw *hw);
void yusur2_enable_mdd(struct yusur2_hw *hw);
void yusur2_mdd_event(struct yusur2_hw *hw, u32 *vf_bitmap);
void yusur2_restore_mdd_vf(struct yusur2_hw *hw, u32 vf);
bool yusur2_fw_recovery_mode(struct yusur2_hw *hw);
s32 yusur2_enter_lplu(struct yusur2_hw *hw);
s32 yusur2_handle_lasi(struct yusur2_hw *hw);
void yusur2_set_rate_select_speed(struct yusur2_hw *hw, yusur2_link_speed speed);
void yusur2_disable_rx(struct yusur2_hw *hw);
void yusur2_enable_rx(struct yusur2_hw *hw);
s32 yusur2_negotiate_fc(struct yusur2_hw *hw, u32 adv_reg, u32 lp_reg,
			u32 adv_sym, u32 adv_asm, u32 lp_sym, u32 lp_asm);

#endif /* _YUSUR2_API_H_ */
