/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_82598_H_
#define _YUSUR2_82598_H_

u32 yusur2_get_pcie_msix_count_82598(struct yusur2_hw *hw);
s32 yusur2_fc_enable_82598(struct yusur2_hw *hw);
s32 yusur2_start_hw_82598(struct yusur2_hw *hw);
void yusur2_enable_relaxed_ordering_82598(struct yusur2_hw *hw);
s32 yusur2_set_vmdq_82598(struct yusur2_hw *hw, u32 rar, u32 vmdq);
s32 yusur2_set_vfta_82598(struct yusur2_hw *hw, u32 vlan, u32 vind, bool vlan_on,
			 bool vlvf_bypass);
s32 yusur2_read_analog_reg8_82598(struct yusur2_hw *hw, u32 reg, u8 *val);
s32 yusur2_write_analog_reg8_82598(struct yusur2_hw *hw, u32 reg, u8 val);
s32 yusur2_read_i2c_eeprom_82598(struct yusur2_hw *hw, u8 byte_offset,
				u8 *eeprom_data);
u64 yusur2_get_supported_physical_layer_82598(struct yusur2_hw *hw);
s32 yusur2_init_phy_ops_82598(struct yusur2_hw *hw);
void yusur2_set_lan_id_multi_port_pcie_82598(struct yusur2_hw *hw);
void yusur2_set_pcie_completion_timeout(struct yusur2_hw *hw);
s32 yusur2_enable_rx_dma_82598(struct yusur2_hw *hw, u32 regval);
#endif /* _YUSUR2_82598_H_ */
