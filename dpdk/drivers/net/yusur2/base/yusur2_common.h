/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_COMMON_H_
#define _YUSUR2_COMMON_H_

#include "yusur2_type.h"
#define YUSUR2_WRITE_REG64(hw, reg, value) \
	do { \
		YUSUR2_WRITE_REG(hw, reg, (u32) value); \
		YUSUR2_WRITE_REG(hw, reg + 4, (u32) (value >> 32)); \
	} while (0)
#define YUSUR2_REMOVED(a) (0)
struct yusur2_pba {
	u16 word[2];
	u16 *pba_block;
};

void yusur2_dcb_get_rtrup2tc_generic(struct yusur2_hw *hw, u8 *map);

u16 yusur2_get_pcie_msix_count_generic(struct yusur2_hw *hw);
s32 yusur2_init_ops_generic(struct yusur2_hw *hw);
s32 yusur2_init_hw_generic(struct yusur2_hw *hw);
s32 yusur2_start_hw_generic(struct yusur2_hw *hw);
s32 yusur2_start_hw_gen2(struct yusur2_hw *hw);
s32 yusur2_clear_hw_cntrs_generic(struct yusur2_hw *hw);
s32 yusur2_read_pba_num_generic(struct yusur2_hw *hw, u32 *pba_num);
s32 yusur2_read_pba_string_generic(struct yusur2_hw *hw, u8 *pba_num,
				  u32 pba_num_size);
s32 yusur2_read_pba_raw(struct yusur2_hw *hw, u16 *eeprom_buf,
		       u32 eeprom_buf_size, u16 max_pba_block_size,
		       struct yusur2_pba *pba);
s32 yusur2_write_pba_raw(struct yusur2_hw *hw, u16 *eeprom_buf,
			u32 eeprom_buf_size, struct yusur2_pba *pba);
s32 yusur2_get_pba_block_size(struct yusur2_hw *hw, u16 *eeprom_buf,
			     u32 eeprom_buf_size, u16 *pba_block_size);
s32 yusur2_get_mac_addr_generic(struct yusur2_hw *hw, u8 *mac_addr);
s32 yusur2_get_bus_info_generic(struct yusur2_hw *hw);
void yusur2_set_pci_config_data_generic(struct yusur2_hw *hw, u16 link_status);
void yusur2_set_lan_id_multi_port_pcie(struct yusur2_hw *hw);
s32 yusur2_stop_adapter_generic(struct yusur2_hw *hw);

s32 yusur2_led_on_generic(struct yusur2_hw *hw, u32 index);
s32 yusur2_led_off_generic(struct yusur2_hw *hw, u32 index);
s32 yusur2_init_led_link_act_generic(struct yusur2_hw *hw);

s32 yusur2_init_eeprom_params_generic(struct yusur2_hw *hw);
s32 yusur2_write_eeprom_generic(struct yusur2_hw *hw, u16 offset, u16 data);
s32 yusur2_write_eeprom_buffer_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
					       u16 words, u16 *data);
s32 yusur2_read_eerd_generic(struct yusur2_hw *hw, u16 offset, u16 *data);
s32 yusur2_read_eerd_buffer_generic(struct yusur2_hw *hw, u16 offset,
				   u16 words, u16 *data);
s32 yusur2_write_eewr_generic(struct yusur2_hw *hw, u16 offset, u16 data);
s32 yusur2_write_eewr_buffer_generic(struct yusur2_hw *hw, u16 offset,
				    u16 words, u16 *data);
s32 yusur2_read_eeprom_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
				       u16 *data);
s32 yusur2_read_eeprom_buffer_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
					      u16 words, u16 *data);
s32 yusur2_calc_eeprom_checksum_generic(struct yusur2_hw *hw);
s32 yusur2_validate_eeprom_checksum_generic(struct yusur2_hw *hw,
					   u16 *checksum_val);
s32 yusur2_update_eeprom_checksum_generic(struct yusur2_hw *hw);
s32 yusur2_poll_eerd_eewr_done(struct yusur2_hw *hw, u32 ee_reg);

s32 yusur2_set_rar_generic(struct yusur2_hw *hw, u32 index, u8 *addr, u32 vmdq,
			  u32 enable_addr);
s32 yusur2_clear_rar_generic(struct yusur2_hw *hw, u32 index);
s32 yusur2_init_rx_addrs_generic(struct yusur2_hw *hw);
s32 yusur2_update_mc_addr_list_generic(struct yusur2_hw *hw, u8 *mc_addr_list,
				      u32 mc_addr_count,
				      yusur2_mc_addr_itr func, bool clear);
s32 yusur2_update_uc_addr_list_generic(struct yusur2_hw *hw, u8 *addr_list,
				      u32 addr_count, yusur2_mc_addr_itr func);
s32 yusur2_enable_mc_generic(struct yusur2_hw *hw);
s32 yusur2_disable_mc_generic(struct yusur2_hw *hw);
s32 yusur2_enable_rx_dma_generic(struct yusur2_hw *hw, u32 regval);
s32 yusur2_disable_sec_rx_path_generic(struct yusur2_hw *hw);
s32 yusur2_enable_sec_rx_path_generic(struct yusur2_hw *hw);

s32 yusur2_fc_enable_generic(struct yusur2_hw *hw);
bool yusur2_device_supports_autoneg_fc(struct yusur2_hw *hw);
void yusur2_fc_autoneg(struct yusur2_hw *hw);
s32 yusur2_setup_fc_generic(struct yusur2_hw *hw);

s32 yusur2_validate_mac_addr(u8 *mac_addr);
s32 yusur2_acquire_swfw_sync(struct yusur2_hw *hw, u32 mask);
void yusur2_release_swfw_sync(struct yusur2_hw *hw, u32 mask);
s32 yusur2_disable_pcie_master(struct yusur2_hw *hw);

s32 yusur2_prot_autoc_read_generic(struct yusur2_hw *hw, bool *, u32 *reg_val);
s32 yusur2_prot_autoc_write_generic(struct yusur2_hw *hw, u32 reg_val, bool locked);

s32 yusur2_blink_led_start_generic(struct yusur2_hw *hw, u32 index);
s32 yusur2_blink_led_stop_generic(struct yusur2_hw *hw, u32 index);

s32 yusur2_get_san_mac_addr_generic(struct yusur2_hw *hw, u8 *san_mac_addr);
s32 yusur2_set_san_mac_addr_generic(struct yusur2_hw *hw, u8 *san_mac_addr);

s32 yusur2_set_vmdq_generic(struct yusur2_hw *hw, u32 rar, u32 vmdq);
s32 yusur2_set_vmdq_san_mac_generic(struct yusur2_hw *hw, u32 vmdq);
s32 yusur2_clear_vmdq_generic(struct yusur2_hw *hw, u32 rar, u32 vmdq);
s32 yusur2_insert_mac_addr_generic(struct yusur2_hw *hw, u8 *addr, u32 vmdq);
s32 yusur2_init_uta_tables_generic(struct yusur2_hw *hw);
s32 yusur2_set_vfta_generic(struct yusur2_hw *hw, u32 vlan,
			 u32 vind, bool vlan_on, bool vlvf_bypass);
s32 yusur2_set_vlvf_generic(struct yusur2_hw *hw, u32 vlan, u32 vind,
			   bool vlan_on, u32 *vfta_delta, u32 vfta,
			   bool vlvf_bypass);
s32 yusur2_clear_vfta_generic(struct yusur2_hw *hw);
s32 yusur2_find_vlvf_slot(struct yusur2_hw *hw, u32 vlan, bool vlvf_bypass);

s32 yusur2_check_mac_link_generic(struct yusur2_hw *hw,
			       yusur2_link_speed *speed,
			       bool *link_up, bool link_up_wait_to_complete);

s32 yusur2_get_wwn_prefix_generic(struct yusur2_hw *hw, u16 *wwnn_prefix,
				 u16 *wwpn_prefix);

s32 yusur2_get_fcoe_boot_status_generic(struct yusur2_hw *hw, u16 *bs);
void yusur2_set_mac_anti_spoofing(struct yusur2_hw *hw, bool enable, int vf);
void yusur2_set_vlan_anti_spoofing(struct yusur2_hw *hw, bool enable, int vf);
s32 yusur2_get_device_caps_generic(struct yusur2_hw *hw, u16 *device_caps);
void yusur2_set_rxpba_generic(struct yusur2_hw *hw, int num_pb, u32 headroom,
			     int strategy);
void yusur2_enable_relaxed_ordering_gen2(struct yusur2_hw *hw);
s32 yusur2_set_fw_drv_ver_generic(struct yusur2_hw *hw, u8 maj, u8 min,
				 u8 build, u8 ver, u16 len, const char *str);
u8 yusur2_calculate_checksum(u8 *buffer, u32 length);
s32 yusur2_host_interface_command(struct yusur2_hw *hw, u32 *buffer,
				 u32 length, u32 timeout, bool return_data);
s32 yusur2_hic_unlocked(struct yusur2_hw *, u32 *buffer, u32 length, u32 timeout);
s32 yusur2_shutdown_fw_phy(struct yusur2_hw *);
s32 yusur2_fw_phy_activity(struct yusur2_hw *, u16 activity,
			  u32 (*data)[FW_PHY_ACT_DATA_COUNT]);
void yusur2_clear_tx_pending(struct yusur2_hw *hw);

extern s32 yusur2_reset_pipeline_82599(struct yusur2_hw *hw);
extern void yusur2_stop_mac_link_on_d3_82599(struct yusur2_hw *hw);
bool yusur2_mng_present(struct yusur2_hw *hw);
bool yusur2_mng_enabled(struct yusur2_hw *hw);

#define YUSUR2_I2C_THERMAL_SENSOR_ADDR	0xF8
#define YUSUR2_EMC_INTERNAL_DATA		0x00
#define YUSUR2_EMC_INTERNAL_THERM_LIMIT	0x20
#define YUSUR2_EMC_DIODE1_DATA		0x01
#define YUSUR2_EMC_DIODE1_THERM_LIMIT	0x19
#define YUSUR2_EMC_DIODE2_DATA		0x23
#define YUSUR2_EMC_DIODE2_THERM_LIMIT	0x1A
#define YUSUR2_EMC_DIODE3_DATA		0x2A
#define YUSUR2_EMC_DIODE3_THERM_LIMIT	0x30

s32 yusur2_get_thermal_sensor_data_generic(struct yusur2_hw *hw);
s32 yusur2_init_thermal_sensor_thresh_generic(struct yusur2_hw *hw);

void yusur2_get_etk_id(struct yusur2_hw *hw, struct yusur2_nvm_version *nvm_ver);
void yusur2_get_oem_prod_version(struct yusur2_hw *hw,
				struct yusur2_nvm_version *nvm_ver);
void yusur2_get_orom_version(struct yusur2_hw *hw,
			    struct yusur2_nvm_version *nvm_ver);
void yusur2_disable_rx_generic(struct yusur2_hw *hw);
void yusur2_enable_rx_generic(struct yusur2_hw *hw);
s32 yusur2_setup_mac_link_multispeed_fiber(struct yusur2_hw *hw,
					  yusur2_link_speed speed,
					  bool autoneg_wait_to_complete);
void yusur2_set_soft_rate_select_speed(struct yusur2_hw *hw,
				      yusur2_link_speed speed);
#endif /* YUSUR2_COMMON */
