/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_X540_H_
#define _YUSUR2_X540_H_

#include "yusur2_type.h"

s32 yusur2_get_link_capabilities_X540(struct yusur2_hw *hw,
				     yusur2_link_speed *speed, bool *autoneg);
enum yusur2_media_type yusur2_get_media_type_X540(struct yusur2_hw *hw);
s32 yusur2_setup_mac_link_X540(struct yusur2_hw *hw, yusur2_link_speed speed,
			      bool link_up_wait_to_complete);
s32 yusur2_reset_hw_X540(struct yusur2_hw *hw);
s32 yusur2_start_hw_X540(struct yusur2_hw *hw);
u64 yusur2_get_supported_physical_layer_X540(struct yusur2_hw *hw);

s32 yusur2_init_eeprom_params_X540(struct yusur2_hw *hw);
s32 yusur2_read_eerd_X540(struct yusur2_hw *hw, u16 offset, u16 *data);
s32 yusur2_read_eerd_buffer_X540(struct yusur2_hw *hw, u16 offset, u16 words,
				u16 *data);
s32 yusur2_write_eewr_X540(struct yusur2_hw *hw, u16 offset, u16 data);
s32 yusur2_write_eewr_buffer_X540(struct yusur2_hw *hw, u16 offset, u16 words,
				 u16 *data);
s32 yusur2_update_eeprom_checksum_X540(struct yusur2_hw *hw);
s32 yusur2_validate_eeprom_checksum_X540(struct yusur2_hw *hw, u16 *checksum_val);
s32 yusur2_calc_eeprom_checksum_X540(struct yusur2_hw *hw);
s32 yusur2_update_flash_X540(struct yusur2_hw *hw);

s32 yusur2_acquire_swfw_sync_X540(struct yusur2_hw *hw, u32 mask);
void yusur2_release_swfw_sync_X540(struct yusur2_hw *hw, u32 mask);
void yusur2_init_swfw_sync_X540(struct yusur2_hw *hw);

s32 yusur2_blink_led_start_X540(struct yusur2_hw *hw, u32 index);
s32 yusur2_blink_led_stop_X540(struct yusur2_hw *hw, u32 index);
#endif /* _YUSUR2_X540_H_ */

