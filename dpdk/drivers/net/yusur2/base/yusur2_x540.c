
/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include "yusur2_x540.h"
#include "yusur2_type.h"
#include "yusur2_api.h"
#include "yusur2_common.h"
#include "yusur2_phy.h"

#define YUSUR2_X540_MAX_TX_QUEUES	128
#define YUSUR2_X540_MAX_RX_QUEUES	128
#define YUSUR2_X540_RAR_ENTRIES		128
#define YUSUR2_X540_MC_TBL_SIZE		128
#define YUSUR2_X540_VFT_TBL_SIZE		128
#define YUSUR2_X540_RX_PB_SIZE		384

STATIC s32 yusur2_poll_flash_update_done_X540(struct yusur2_hw *hw);
STATIC s32 yusur2_get_swfw_sync_semaphore(struct yusur2_hw *hw);
STATIC void yusur2_release_swfw_sync_semaphore(struct yusur2_hw *hw);

/**
 *  yusur2_init_ops_X540 - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for X540.
 *  Does not touch the hardware.
 **/
s32 yusur2_init_ops_X540(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;
	struct yusur2_phy_info *phy = &hw->phy;
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val;

	DEBUGFUNC("yusur2_init_ops_X540");

	ret_val = yusur2_init_phy_ops_generic(hw);
	ret_val = yusur2_init_ops_generic(hw);


	/* EEPROM */
	eeprom->ops.init_params = yusur2_init_eeprom_params_X540;
	eeprom->ops.read = yusur2_read_eerd_X540;
	eeprom->ops.read_buffer = yusur2_read_eerd_buffer_X540;
	eeprom->ops.write = yusur2_write_eewr_X540;
	eeprom->ops.write_buffer = yusur2_write_eewr_buffer_X540;
	eeprom->ops.update_checksum = yusur2_update_eeprom_checksum_X540;
	eeprom->ops.validate_checksum = yusur2_validate_eeprom_checksum_X540;
	eeprom->ops.calc_checksum = yusur2_calc_eeprom_checksum_X540;

	/* PHY */
	phy->ops.init = yusur2_init_phy_ops_generic;
	phy->ops.reset = NULL;
	phy->ops.set_phy_power = yusur2_set_copper_phy_power;

	/* MAC */
	mac->ops.reset_hw = yusur2_reset_hw_X540;
	mac->ops.enable_relaxed_ordering = yusur2_enable_relaxed_ordering_gen2;
	mac->ops.get_media_type = yusur2_get_media_type_X540;
	mac->ops.get_supported_physical_layer =
				    yusur2_get_supported_physical_layer_X540;
	mac->ops.read_analog_reg8 = NULL;
	mac->ops.write_analog_reg8 = NULL;
	mac->ops.start_hw = yusur2_start_hw_X540;
	mac->ops.get_san_mac_addr = yusur2_get_san_mac_addr_generic;
	mac->ops.set_san_mac_addr = yusur2_set_san_mac_addr_generic;
	mac->ops.get_device_caps = yusur2_get_device_caps_generic;
	mac->ops.get_wwn_prefix = yusur2_get_wwn_prefix_generic;
	mac->ops.get_fcoe_boot_status = yusur2_get_fcoe_boot_status_generic;
	mac->ops.acquire_swfw_sync = yusur2_acquire_swfw_sync_X540;
	mac->ops.release_swfw_sync = yusur2_release_swfw_sync_X540;
	mac->ops.init_swfw_sync = yusur2_init_swfw_sync_X540;
	mac->ops.disable_sec_rx_path = yusur2_disable_sec_rx_path_generic;
	mac->ops.enable_sec_rx_path = yusur2_enable_sec_rx_path_generic;

	/* RAR, Multicast, VLAN */
	mac->ops.set_vmdq = yusur2_set_vmdq_generic;
	mac->ops.set_vmdq_san_mac = yusur2_set_vmdq_san_mac_generic;
	mac->ops.clear_vmdq = yusur2_clear_vmdq_generic;
	mac->ops.insert_mac_addr = yusur2_insert_mac_addr_generic;
	mac->rar_highwater = 1;
	mac->ops.set_vfta = yusur2_set_vfta_generic;
	mac->ops.set_vlvf = yusur2_set_vlvf_generic;
	mac->ops.clear_vfta = yusur2_clear_vfta_generic;
	mac->ops.init_uta_tables = yusur2_init_uta_tables_generic;
	mac->ops.set_mac_anti_spoofing = yusur2_set_mac_anti_spoofing;
	mac->ops.set_vlan_anti_spoofing = yusur2_set_vlan_anti_spoofing;

	/* Link */
	mac->ops.get_link_capabilities =
				yusur2_get_copper_link_capabilities_generic;
	mac->ops.setup_link = yusur2_setup_mac_link_X540;
	mac->ops.setup_rxpba = yusur2_set_rxpba_generic;
	mac->ops.check_link = yusur2_check_mac_link_generic;


	mac->mcft_size		= YUSUR2_X540_MC_TBL_SIZE;
	mac->vft_size		= YUSUR2_X540_VFT_TBL_SIZE;
	mac->num_rar_entries	= YUSUR2_X540_RAR_ENTRIES;
	mac->rx_pb_size		= YUSUR2_X540_RX_PB_SIZE;
	mac->max_rx_queues	= YUSUR2_X540_MAX_RX_QUEUES;
	mac->max_tx_queues	= YUSUR2_X540_MAX_TX_QUEUES;
	mac->max_msix_vectors	= yusur2_get_pcie_msix_count_generic(hw);

	/*
	 * FWSM register
	 * ARC supported; valid only if manageability features are
	 * enabled.
	 */
	mac->arc_subsystem_valid = !!(YUSUR2_READ_REG(hw, YUSUR2_FWSM_BY_MAC(hw))
				     & YUSUR2_FWSM_MODE_MASK);

	hw->mbx.ops.init_params = yusur2_init_mbx_params_pf;

	/* LEDs */
	mac->ops.blink_led_start = yusur2_blink_led_start_X540;
	mac->ops.blink_led_stop = yusur2_blink_led_stop_X540;

	/* Manageability interface */
	mac->ops.set_fw_drv_ver = yusur2_set_fw_drv_ver_generic;

	mac->ops.get_rtrup2tc = yusur2_dcb_get_rtrup2tc_generic;

	return ret_val;
}

/**
 *  yusur2_get_link_capabilities_X540 - Determines link capabilities
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @autoneg: true when autoneg or autotry is enabled
 *
 *  Determines the link capabilities by reading the AUTOC register.
 **/
s32 yusur2_get_link_capabilities_X540(struct yusur2_hw *hw,
				     yusur2_link_speed *speed,
				     bool *autoneg)
{
	yusur2_get_copper_link_capabilities_generic(hw, speed, autoneg);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_media_type_X540 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
enum yusur2_media_type yusur2_get_media_type_X540(struct yusur2_hw *hw)
{
	UNREFERENCED_1PARAMETER(hw);
	return yusur2_media_type_copper;
}

/**
 *  yusur2_setup_mac_link_X540 - Sets the auto advertised capabilities
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 **/
s32 yusur2_setup_mac_link_X540(struct yusur2_hw *hw,
			      yusur2_link_speed speed,
			      bool autoneg_wait_to_complete)
{
	DEBUGFUNC("yusur2_setup_mac_link_X540");
	return hw->phy.ops.setup_link_speed(hw, speed, autoneg_wait_to_complete);
}

/**
 *  yusur2_reset_hw_X540 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, and perform a reset.
 **/
s32 yusur2_reset_hw_X540(struct yusur2_hw *hw)
{
	s32 status;
	u32 ctrl, i;
	u32 swfw_mask = hw->phy.phy_semaphore_mask;

	DEBUGFUNC("yusur2_reset_hw_X540");

	/* Call adapter stop to disable tx/rx and clear interrupts */
	status = hw->mac.ops.stop_adapter(hw);
	if (status != YUSUR2_SUCCESS)
		goto reset_hw_out;

	/* flush pending Tx transactions */
	yusur2_clear_tx_pending(hw);

mac_reset_top:
	status = hw->mac.ops.acquire_swfw_sync(hw, swfw_mask);
	if (status != YUSUR2_SUCCESS) {
		ERROR_REPORT2(YUSUR2_ERROR_CAUTION,
			"semaphore failed with %d", status);
		return YUSUR2_ERR_SWFW_SYNC;
	}
	ctrl = YUSUR2_CTRL_RST;
	ctrl |= YUSUR2_READ_REG(hw, YUSUR2_CTRL);
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL, ctrl);
	YUSUR2_WRITE_FLUSH(hw);
	hw->mac.ops.release_swfw_sync(hw, swfw_mask);

	/* Poll for reset bit to self-clear indicating reset is complete */
	for (i = 0; i < 10; i++) {
		usec_delay(1);
		ctrl = YUSUR2_READ_REG(hw, YUSUR2_CTRL);
		if (!(ctrl & YUSUR2_CTRL_RST_MASK))
			break;
	}

	if (ctrl & YUSUR2_CTRL_RST_MASK) {
		status = YUSUR2_ERR_RESET_FAILED;
		ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			     "Reset polling failed to complete.\n");
	}
	msec_delay(100);

	/*
	 * Double resets are required for recovery from certain error
	 * conditions.  Between resets, it is necessary to stall to allow time
	 * for any pending HW events to complete.
	 */
	if (hw->mac.flags & YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED) {
		hw->mac.flags &= ~YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED;
		goto mac_reset_top;
	}

	/* Set the Rx packet buffer size. */
	YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(0), 384 << YUSUR2_RXPBSIZE_SHIFT);

	/* Store the permanent mac address */
	hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

	/*
	 * Store MAC address from RAR0, clear receive address registers, and
	 * clear the multicast table.  Also reset num_rar_entries to 128,
	 * since we modify this value when programming the SAN MAC address.
	 */
	hw->mac.num_rar_entries = 128;
	hw->mac.ops.init_rx_addrs(hw);

	/* Store the permanent SAN mac address */
	hw->mac.ops.get_san_mac_addr(hw, hw->mac.san_addr);

	/* Add the SAN MAC address to the RAR only if it's a valid address */
	if (yusur2_validate_mac_addr(hw->mac.san_addr) == 0) {
		/* Save the SAN MAC RAR index */
		hw->mac.san_mac_rar_index = hw->mac.num_rar_entries - 1;

		hw->mac.ops.set_rar(hw, hw->mac.san_mac_rar_index,
				    hw->mac.san_addr, 0, YUSUR2_RAH_AV);

		/* clear VMDq pool/queue selection for this RAR */
		hw->mac.ops.clear_vmdq(hw, hw->mac.san_mac_rar_index,
				       YUSUR2_CLEAR_VMDQ_ALL);

		/* Reserve the last RAR for the SAN MAC address */
		hw->mac.num_rar_entries--;
	}

	/* Store the alternative WWNN/WWPN prefix */
	hw->mac.ops.get_wwn_prefix(hw, &hw->mac.wwnn_prefix,
				   &hw->mac.wwpn_prefix);

reset_hw_out:
	return status;
}

/**
 *  yusur2_start_hw_X540 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
s32 yusur2_start_hw_X540(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_start_hw_X540");

	ret_val = yusur2_start_hw_generic(hw);
	if (ret_val != YUSUR2_SUCCESS)
		goto out;

	ret_val = yusur2_start_hw_gen2(hw);

out:
	return ret_val;
}

/**
 *  yusur2_get_supported_physical_layer_X540 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
u64 yusur2_get_supported_physical_layer_X540(struct yusur2_hw *hw)
{
	u64 physical_layer = YUSUR2_PHYSICAL_LAYER_UNKNOWN;
	u16 ext_ability = 0;

	DEBUGFUNC("yusur2_get_supported_physical_layer_X540");

	hw->phy.ops.read_reg(hw, YUSUR2_MDIO_PHY_EXT_ABILITY,
	YUSUR2_MDIO_PMA_PMD_DEV_TYPE, &ext_ability);
	if (ext_ability & YUSUR2_MDIO_PHY_10GBASET_ABILITY)
		physical_layer |= YUSUR2_PHYSICAL_LAYER_10GBASE_T;
	if (ext_ability & YUSUR2_MDIO_PHY_1000BASET_ABILITY)
		physical_layer |= YUSUR2_PHYSICAL_LAYER_1000BASE_T;
	if (ext_ability & YUSUR2_MDIO_PHY_100BASETX_ABILITY)
		physical_layer |= YUSUR2_PHYSICAL_LAYER_100BASE_TX;

	return physical_layer;
}

/**
 *  yusur2_init_eeprom_params_X540 - Initialize EEPROM params
 *  @hw: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters yusur2_eeprom_info within the
 *  yusur2_hw struct in order to set up EEPROM access.
 **/
s32 yusur2_init_eeprom_params_X540(struct yusur2_hw *hw)
{
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	u32 eec;
	u16 eeprom_size;

	DEBUGFUNC("yusur2_init_eeprom_params_X540");

	if (eeprom->type == yusur2_eeprom_uninitialized) {
		eeprom->semaphore_delay = 10;
		eeprom->type = yusur2_flash;

		eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));
		eeprom_size = (u16)((eec & YUSUR2_EEC_SIZE) >>
				    YUSUR2_EEC_SIZE_SHIFT);
		eeprom->word_size = 1 << (eeprom_size +
					  YUSUR2_EEPROM_WORD_SIZE_SHIFT);

		DEBUGOUT2("Eeprom params: type = %d, size = %d\n",
			  eeprom->type, eeprom->word_size);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_eerd_X540- Read EEPROM word using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 yusur2_read_eerd_X540(struct yusur2_hw *hw, u16 offset, u16 *data)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_read_eerd_X540");
	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM) ==
	    YUSUR2_SUCCESS) {
		status = yusur2_read_eerd_generic(hw, offset, data);
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);
	} else {
		status = YUSUR2_ERR_SWFW_SYNC;
	}

	return status;
}

/**
 *  yusur2_read_eerd_buffer_X540- Read EEPROM word(s) using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the EERD register.
 **/
s32 yusur2_read_eerd_buffer_X540(struct yusur2_hw *hw,
				u16 offset, u16 words, u16 *data)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_read_eerd_buffer_X540");
	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM) ==
	    YUSUR2_SUCCESS) {
		status = yusur2_read_eerd_buffer_generic(hw, offset,
							words, data);
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);
	} else {
		status = YUSUR2_ERR_SWFW_SYNC;
	}

	return status;
}

/**
 *  yusur2_write_eewr_X540 - Write EEPROM word using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @data: word write to the EEPROM
 *
 *  Write a 16 bit word to the EEPROM using the EEWR register.
 **/
s32 yusur2_write_eewr_X540(struct yusur2_hw *hw, u16 offset, u16 data)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_write_eewr_X540");
	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM) ==
	    YUSUR2_SUCCESS) {
		status = yusur2_write_eewr_generic(hw, offset, data);
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);
	} else {
		status = YUSUR2_ERR_SWFW_SYNC;
	}

	return status;
}

/**
 *  yusur2_write_eewr_buffer_X540 - Write EEPROM word(s) using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @words: number of words
 *  @data: word(s) write to the EEPROM
 *
 *  Write a 16 bit word(s) to the EEPROM using the EEWR register.
 **/
s32 yusur2_write_eewr_buffer_X540(struct yusur2_hw *hw,
				 u16 offset, u16 words, u16 *data)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_write_eewr_buffer_X540");
	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM) ==
	    YUSUR2_SUCCESS) {
		status = yusur2_write_eewr_buffer_generic(hw, offset,
							 words, data);
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);
	} else {
		status = YUSUR2_ERR_SWFW_SYNC;
	}

	return status;
}

/**
 *  yusur2_calc_eeprom_checksum_X540 - Calculates and returns the checksum
 *
 *  This function does not use synchronization for EERD and EEWR. It can
 *  be used internally by function which utilize yusur2_acquire_swfw_sync_X540.
 *
 *  @hw: pointer to hardware structure
 *
 *  Returns a negative error code on error, or the 16-bit checksum
 **/
s32 yusur2_calc_eeprom_checksum_X540(struct yusur2_hw *hw)
{
	u16 i, j;
	u16 checksum = 0;
	u16 length = 0;
	u16 pointer = 0;
	u16 word = 0;
	u16 ptr_start = YUSUR2_PCIE_ANALOG_PTR;

	/* Do not use hw->eeprom.ops.read because we do not want to take
	 * the synchronization semaphores here. Instead use
	 * yusur2_read_eerd_generic
	 */

	DEBUGFUNC("yusur2_calc_eeprom_checksum_X540");

	/* Include 0x0 up to YUSUR2_EEPROM_CHECKSUM; do not include the
	 * checksum itself
	 */
	for (i = 0; i < YUSUR2_EEPROM_CHECKSUM; i++) {
		if (yusur2_read_eerd_generic(hw, i, &word)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}
		checksum += word;
	}

	/* Include all data from pointers 0x3, 0x6-0xE.  This excludes the
	 * FW, PHY module, and PCIe Expansion/Option ROM pointers.
	 */
	for (i = ptr_start; i < YUSUR2_FW_PTR; i++) {
		if (i == YUSUR2_PHY_PTR || i == YUSUR2_OPTION_ROM_PTR)
			continue;

		if (yusur2_read_eerd_generic(hw, i, &pointer)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}

		/* Skip pointer section if the pointer is invalid. */
		if (pointer == 0xFFFF || pointer == 0 ||
		    pointer >= hw->eeprom.word_size)
			continue;

		if (yusur2_read_eerd_generic(hw, pointer, &length)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}

		/* Skip pointer section if length is invalid. */
		if (length == 0xFFFF || length == 0 ||
		    (pointer + length) >= hw->eeprom.word_size)
			continue;

		for (j = pointer + 1; j <= pointer + length; j++) {
			if (yusur2_read_eerd_generic(hw, j, &word)) {
				DEBUGOUT("EEPROM read failed\n");
				return YUSUR2_ERR_EEPROM;
			}
			checksum += word;
		}
	}

	checksum = (u16)YUSUR2_EEPROM_SUM - checksum;

	return (s32)checksum;
}

/**
 *  yusur2_validate_eeprom_checksum_X540 - Validate EEPROM checksum
 *  @hw: pointer to hardware structure
 *  @checksum_val: calculated checksum
 *
 *  Performs checksum calculation and validates the EEPROM checksum.  If the
 *  caller does not need checksum_val, the value can be NULL.
 **/
s32 yusur2_validate_eeprom_checksum_X540(struct yusur2_hw *hw,
					u16 *checksum_val)
{
	s32 status;
	u16 checksum;
	u16 read_checksum = 0;

	DEBUGFUNC("yusur2_validate_eeprom_checksum_X540");

	/* Read the first word from the EEPROM. If this times out or fails, do
	 * not continue or we could be in for a very long wait while every
	 * EEPROM read fails
	 */
	status = hw->eeprom.ops.read(hw, 0, &checksum);
	if (status) {
		DEBUGOUT("EEPROM read failed\n");
		return status;
	}

	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM))
		return YUSUR2_ERR_SWFW_SYNC;

	status = hw->eeprom.ops.calc_checksum(hw);
	if (status < 0)
		goto out;

	checksum = (u16)(status & 0xffff);

	/* Do not use hw->eeprom.ops.read because we do not want to take
	 * the synchronization semaphores twice here.
	 */
	status = yusur2_read_eerd_generic(hw, YUSUR2_EEPROM_CHECKSUM,
					 &read_checksum);
	if (status)
		goto out;

	/* Verify read checksum from EEPROM is the same as
	 * calculated checksum
	 */
	if (read_checksum != checksum) {
		ERROR_REPORT1(YUSUR2_ERROR_INVALID_STATE,
			     "Invalid EEPROM checksum");
		status = YUSUR2_ERR_EEPROM_CHECKSUM;
	}

	/* If the user cares, return the calculated checksum */
	if (checksum_val)
		*checksum_val = checksum;

out:
	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);

	return status;
}

/**
 * yusur2_update_eeprom_checksum_X540 - Updates the EEPROM checksum and flash
 * @hw: pointer to hardware structure
 *
 * After writing EEPROM to shadow RAM using EEWR register, software calculates
 * checksum and updates the EEPROM and instructs the hardware to update
 * the flash.
 **/
s32 yusur2_update_eeprom_checksum_X540(struct yusur2_hw *hw)
{
	s32 status;
	u16 checksum;

	DEBUGFUNC("yusur2_update_eeprom_checksum_X540");

	/* Read the first word from the EEPROM. If this times out or fails, do
	 * not continue or we could be in for a very long wait while every
	 * EEPROM read fails
	 */
	status = hw->eeprom.ops.read(hw, 0, &checksum);
	if (status) {
		DEBUGOUT("EEPROM read failed\n");
		return status;
	}

	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM))
		return YUSUR2_ERR_SWFW_SYNC;

	status = hw->eeprom.ops.calc_checksum(hw);
	if (status < 0)
		goto out;

	checksum = (u16)(status & 0xffff);

	/* Do not use hw->eeprom.ops.write because we do not want to
	 * take the synchronization semaphores twice here.
	 */
	status = yusur2_write_eewr_generic(hw, YUSUR2_EEPROM_CHECKSUM, checksum);
	if (status)
		goto out;

	status = yusur2_update_flash_X540(hw);

out:
	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);

	return status;
}

/**
 *  yusur2_update_flash_X540 - Instruct HW to copy EEPROM to Flash device
 *  @hw: pointer to hardware structure
 *
 *  Set FLUP (bit 23) of the EEC register to instruct Hardware to copy
 *  EEPROM from shadow RAM to the flash device.
 **/
s32 yusur2_update_flash_X540(struct yusur2_hw *hw)
{
	u32 flup;
	s32 status;

	DEBUGFUNC("yusur2_update_flash_X540");

	status = yusur2_poll_flash_update_done_X540(hw);
	if (status == YUSUR2_ERR_EEPROM) {
		DEBUGOUT("Flash update time out\n");
		goto out;
	}

	flup = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw)) | YUSUR2_EEC_FLUP;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), flup);

	status = yusur2_poll_flash_update_done_X540(hw);
	if (status == YUSUR2_SUCCESS)
		DEBUGOUT("Flash update complete\n");
	else
		DEBUGOUT("Flash update time out\n");

	if (hw->mac.type == yusur2_mac_X540 && hw->revision_id == 0) {
		flup = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

		if (flup & YUSUR2_EEC_SEC1VAL) {
			flup |= YUSUR2_EEC_FLUP;
			YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), flup);
		}

		status = yusur2_poll_flash_update_done_X540(hw);
		if (status == YUSUR2_SUCCESS)
			DEBUGOUT("Flash update complete\n");
		else
			DEBUGOUT("Flash update time out\n");
	}
out:
	return status;
}

/**
 *  yusur2_poll_flash_update_done_X540 - Poll flash update status
 *  @hw: pointer to hardware structure
 *
 *  Polls the FLUDONE (bit 26) of the EEC Register to determine when the
 *  flash update is done.
 **/
STATIC s32 yusur2_poll_flash_update_done_X540(struct yusur2_hw *hw)
{
	u32 i;
	u32 reg;
	s32 status = YUSUR2_ERR_EEPROM;

	DEBUGFUNC("yusur2_poll_flash_update_done_X540");

	for (i = 0; i < YUSUR2_FLUDONE_ATTEMPTS; i++) {
		reg = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));
		if (reg & YUSUR2_EEC_FLUDONE) {
			status = YUSUR2_SUCCESS;
			break;
		}
		msec_delay(5);
	}

	if (i == YUSUR2_FLUDONE_ATTEMPTS)
		ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			     "Flash update status polling timed out");

	return status;
}

/**
 *  yusur2_acquire_swfw_sync_X540 - Acquire SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SWFW semaphore thought the SW_FW_SYNC register for
 *  the specified function (CSR, PHY0, PHY1, NVM, Flash)
 **/
s32 yusur2_acquire_swfw_sync_X540(struct yusur2_hw *hw, u32 mask)
{
	u32 swmask = mask & YUSUR2_GSSR_NVM_PHY_MASK;
	u32 fwmask = swmask << 5;
	u32 swi2c_mask = mask & YUSUR2_GSSR_I2C_MASK;
	u32 timeout = 200;
	u32 hwmask = 0;
	u32 swfw_sync;
	u32 i;

	DEBUGFUNC("yusur2_acquire_swfw_sync_X540");

	if (swmask & YUSUR2_GSSR_EEP_SM)
		hwmask |= YUSUR2_GSSR_FLASH_SM;

	/* SW only mask doesn't have FW bit pair */
	if (mask & YUSUR2_GSSR_SW_MNG_SM)
		swmask |= YUSUR2_GSSR_SW_MNG_SM;

	swmask |= swi2c_mask;
	fwmask |= swi2c_mask << 2;
	if (hw->mac.type >= yusur2_mac_X550)
		timeout = 1000;

	for (i = 0; i < timeout; i++) {
		/* SW NVM semaphore bit is used for access to all
		 * SW_FW_SYNC bits (not just NVM)
		 */
		if (yusur2_get_swfw_sync_semaphore(hw)) {
			DEBUGOUT("Failed to get NVM access and register semaphore, returning YUSUR2_ERR_SWFW_SYNC\n");
			return YUSUR2_ERR_SWFW_SYNC;
		}

		swfw_sync = YUSUR2_READ_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw));
		if (!(swfw_sync & (fwmask | swmask | hwmask))) {
			swfw_sync |= swmask;
			YUSUR2_WRITE_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw),
					swfw_sync);
			yusur2_release_swfw_sync_semaphore(hw);
			return YUSUR2_SUCCESS;
		}
		/* Firmware currently using resource (fwmask), hardware
		 * currently using resource (hwmask), or other software
		 * thread currently using resource (swmask)
		 */
		yusur2_release_swfw_sync_semaphore(hw);
		msec_delay(5);
	}

	/* If the resource is not released by the FW/HW the SW can assume that
	 * the FW/HW malfunctions. In that case the SW should set the SW bit(s)
	 * of the requested resource(s) while ignoring the corresponding FW/HW
	 * bits in the SW_FW_SYNC register.
	 */
	if (yusur2_get_swfw_sync_semaphore(hw)) {
		DEBUGOUT("Failed to get NVM semaphore and register semaphore while forcefully ignoring FW semaphore bit(s) and setting SW semaphore bit(s), returning YUSUR2_ERR_SWFW_SYNC\n");
		return YUSUR2_ERR_SWFW_SYNC;
	}
	swfw_sync = YUSUR2_READ_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw));
	if (swfw_sync & (fwmask | hwmask)) {
		swfw_sync |= swmask;
		YUSUR2_WRITE_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw), swfw_sync);
		yusur2_release_swfw_sync_semaphore(hw);
		msec_delay(5);
		return YUSUR2_SUCCESS;
	}
	/* If the resource is not released by other SW the SW can assume that
	 * the other SW malfunctions. In that case the SW should clear all SW
	 * flags that it does not own and then repeat the whole process once
	 * again.
	 */
	if (swfw_sync & swmask) {
		u32 rmask = YUSUR2_GSSR_EEP_SM | YUSUR2_GSSR_PHY0_SM |
			    YUSUR2_GSSR_PHY1_SM | YUSUR2_GSSR_MAC_CSR_SM |
			    YUSUR2_GSSR_SW_MNG_SM;

		if (swi2c_mask)
			rmask |= YUSUR2_GSSR_I2C_MASK;
		yusur2_release_swfw_sync_X540(hw, rmask);
		yusur2_release_swfw_sync_semaphore(hw);
		DEBUGOUT("Resource not released by other SW, returning YUSUR2_ERR_SWFW_SYNC\n");
		return YUSUR2_ERR_SWFW_SYNC;
	}
	yusur2_release_swfw_sync_semaphore(hw);
	DEBUGOUT("Returning error YUSUR2_ERR_SWFW_SYNC\n");

	return YUSUR2_ERR_SWFW_SYNC;
}

/**
 *  yusur2_release_swfw_sync_X540 - Release SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SWFW semaphore through the SW_FW_SYNC register
 *  for the specified function (CSR, PHY0, PHY1, EVM, Flash)
 **/
void yusur2_release_swfw_sync_X540(struct yusur2_hw *hw, u32 mask)
{
	u32 swmask = mask & (YUSUR2_GSSR_NVM_PHY_MASK | YUSUR2_GSSR_SW_MNG_SM);
	u32 swfw_sync;

	DEBUGFUNC("yusur2_release_swfw_sync_X540");

	if (mask & YUSUR2_GSSR_I2C_MASK)
		swmask |= mask & YUSUR2_GSSR_I2C_MASK;
	yusur2_get_swfw_sync_semaphore(hw);

	swfw_sync = YUSUR2_READ_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw));
	swfw_sync &= ~swmask;
	YUSUR2_WRITE_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw), swfw_sync);

	yusur2_release_swfw_sync_semaphore(hw);
	msec_delay(2);
}

/**
 *  yusur2_get_swfw_sync_semaphore - Get hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  Sets the hardware semaphores so SW/FW can gain control of shared resources
 **/
STATIC s32 yusur2_get_swfw_sync_semaphore(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_ERR_EEPROM;
	u32 timeout = 2000;
	u32 i;
	u32 swsm;

	DEBUGFUNC("yusur2_get_swfw_sync_semaphore");

	/* Get SMBI software semaphore between device drivers first */
	for (i = 0; i < timeout; i++) {
		/*
		 * If the SMBI bit is 0 when we read it, then the bit will be
		 * set and we have the semaphore
		 */
		swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM_BY_MAC(hw));
		if (!(swsm & YUSUR2_SWSM_SMBI)) {
			status = YUSUR2_SUCCESS;
			break;
		}
		usec_delay(50);
	}

	/* Now get the semaphore between SW/FW through the REGSMP bit */
	if (status == YUSUR2_SUCCESS) {
		for (i = 0; i < timeout; i++) {
			swsm = YUSUR2_READ_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw));
			if (!(swsm & YUSUR2_SWFW_REGSMP))
				break;

			usec_delay(50);
		}

		/*
		 * Release semaphores and return error if SW NVM semaphore
		 * was not granted because we don't have access to the EEPROM
		 */
		if (i >= timeout) {
			ERROR_REPORT1(YUSUR2_ERROR_POLLING,
				"REGSMP Software NVM semaphore not granted.\n");
			yusur2_release_swfw_sync_semaphore(hw);
			status = YUSUR2_ERR_EEPROM;
		}
	} else {
		ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			     "Software semaphore SMBI between device drivers "
			     "not granted.\n");
	}

	return status;
}

/**
 *  yusur2_release_swfw_sync_semaphore - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function clears hardware semaphore bits.
 **/
STATIC void yusur2_release_swfw_sync_semaphore(struct yusur2_hw *hw)
{
	u32 swsm;

	DEBUGFUNC("yusur2_release_swfw_sync_semaphore");

	/* Release both semaphores by writing 0 to the bits REGSMP and SMBI */

	swsm = YUSUR2_READ_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw));
	swsm &= ~YUSUR2_SWFW_REGSMP;
	YUSUR2_WRITE_REG(hw, YUSUR2_SWFW_SYNC_BY_MAC(hw), swsm);

	swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM_BY_MAC(hw));
	swsm &= ~YUSUR2_SWSM_SMBI;
	YUSUR2_WRITE_REG(hw, YUSUR2_SWSM_BY_MAC(hw), swsm);

	YUSUR2_WRITE_FLUSH(hw);
}

/**
 *  yusur2_init_swfw_sync_X540 - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function reset hardware semaphore bits for a semaphore that may
 *  have be left locked due to a catastrophic failure.
 **/
void yusur2_init_swfw_sync_X540(struct yusur2_hw *hw)
{
	u32 rmask;

	/* First try to grab the semaphore but we don't need to bother
	 * looking to see whether we got the lock or not since we do
	 * the same thing regardless of whether we got the lock or not.
	 * We got the lock - we release it.
	 * We timeout trying to get the lock - we force its release.
	 */
	yusur2_get_swfw_sync_semaphore(hw);
	yusur2_release_swfw_sync_semaphore(hw);

	/* Acquire and release all software resources. */
	rmask = YUSUR2_GSSR_EEP_SM | YUSUR2_GSSR_PHY0_SM |
		YUSUR2_GSSR_PHY1_SM | YUSUR2_GSSR_MAC_CSR_SM |
		YUSUR2_GSSR_SW_MNG_SM;

	rmask |= YUSUR2_GSSR_I2C_MASK;
	yusur2_acquire_swfw_sync_X540(hw, rmask);
	yusur2_release_swfw_sync_X540(hw, rmask);
}

/**
 * yusur2_blink_led_start_X540 - Blink LED based on index.
 * @hw: pointer to hardware structure
 * @index: led number to blink
 *
 * Devices that implement the version 2 interface:
 *   X540
 **/
s32 yusur2_blink_led_start_X540(struct yusur2_hw *hw, u32 index)
{
	u32 macc_reg;
	u32 ledctl_reg;
	yusur2_link_speed speed;
	bool link_up;

	DEBUGFUNC("yusur2_blink_led_start_X540");

	if (index > 3)
		return YUSUR2_ERR_PARAM;

	/*
	 * Link should be up in order for the blink bit in the LED control
	 * register to work. Force link and speed in the MAC if link is down.
	 * This will be reversed when we stop the blinking.
	 */
	hw->mac.ops.check_link(hw, &speed, &link_up, false);
	if (link_up == false) {
		macc_reg = YUSUR2_READ_REG(hw, YUSUR2_MACC);
		macc_reg |= YUSUR2_MACC_FLU | YUSUR2_MACC_FSV_10G | YUSUR2_MACC_FS;
		YUSUR2_WRITE_REG(hw, YUSUR2_MACC, macc_reg);
	}
	/* Set the LED to LINK_UP + BLINK. */
	ledctl_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);
	ledctl_reg &= ~YUSUR2_LED_MODE_MASK(index);
	ledctl_reg |= YUSUR2_LED_BLINK(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, ledctl_reg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_blink_led_stop_X540 - Stop blinking LED based on index.
 * @hw: pointer to hardware structure
 * @index: led number to stop blinking
 *
 * Devices that implement the version 2 interface:
 *   X540
 **/
s32 yusur2_blink_led_stop_X540(struct yusur2_hw *hw, u32 index)
{
	u32 macc_reg;
	u32 ledctl_reg;

	if (index > 3)
		return YUSUR2_ERR_PARAM;

	DEBUGFUNC("yusur2_blink_led_stop_X540");

	/* Restore the LED to its default value. */
	ledctl_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);
	ledctl_reg &= ~YUSUR2_LED_MODE_MASK(index);
	ledctl_reg |= YUSUR2_LED_LINK_ACTIVE << YUSUR2_LED_MODE_SHIFT(index);
	ledctl_reg &= ~YUSUR2_LED_BLINK(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, ledctl_reg);

	/* Unforce link and speed in the MAC. */
	macc_reg = YUSUR2_READ_REG(hw, YUSUR2_MACC);
	macc_reg &= ~(YUSUR2_MACC_FLU | YUSUR2_MACC_FSV_10G | YUSUR2_MACC_FS);
	YUSUR2_WRITE_REG(hw, YUSUR2_MACC, macc_reg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}
