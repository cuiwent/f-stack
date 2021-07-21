/* SPDX-License-Identifier: BSD-3-Clause
 *Copyright(c) 2018-2021 Yusur
 */

#include "yusur2_type.h"
#include "yusur2_82599.h"
#include "yusur2_api.h"
#include "yusur2_common.h"
#include "yusur2_phy.h"

#define YUSUR2_82599_MAX_TX_QUEUES 128
#define YUSUR2_82599_MAX_RX_QUEUES 128
#define YUSUR2_82599_RAR_ENTRIES   128
#define YUSUR2_82599_MC_TBL_SIZE   128
#define YUSUR2_82599_VFT_TBL_SIZE  128
#define YUSUR2_82599_RX_PB_SIZE	  512

STATIC s32 yusur2_setup_copper_link_82599(struct yusur2_hw *hw,
					 yusur2_link_speed speed,
					 bool autoneg_wait_to_complete);
STATIC s32 yusur2_verify_fw_version_82599(struct yusur2_hw *hw);
STATIC s32 yusur2_read_eeprom_82599(struct yusur2_hw *hw,
				   u16 offset, u16 *data);
STATIC s32 yusur2_read_eeprom_buffer_82599(struct yusur2_hw *hw, u16 offset,
					  u16 words, u16 *data);
STATIC s32 yusur2_read_i2c_byte_82599(struct yusur2_hw *hw, u8 byte_offset,
					u8 dev_addr, u8 *data);
STATIC s32 yusur2_write_i2c_byte_82599(struct yusur2_hw *hw, u8 byte_offset,
					u8 dev_addr, u8 data);

void yusur2_init_mac_link_ops_82599(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;

	DEBUGFUNC("yusur2_init_mac_link_ops_82599");

	/*
	 * enable the laser control functions for SFP+ fiber
	 * and MNG not enabled
	 */
	if ((mac->ops.get_media_type(hw) == yusur2_media_type_fiber) &&
	    !yusur2_mng_enabled(hw)) {
		mac->ops.disable_tx_laser =
				       yusur2_disable_tx_laser_multispeed_fiber;
		mac->ops.enable_tx_laser =
					yusur2_enable_tx_laser_multispeed_fiber;
		mac->ops.flap_tx_laser = yusur2_flap_tx_laser_multispeed_fiber;

	} else {
		mac->ops.disable_tx_laser = NULL;
		mac->ops.enable_tx_laser = NULL;
		mac->ops.flap_tx_laser = NULL;
	}

	if (hw->phy.multispeed_fiber) {
		/* Set up dual speed SFP+ support */
		mac->ops.setup_link = yusur2_setup_mac_link_multispeed_fiber;
		mac->ops.setup_mac_link = yusur2_setup_mac_link_82599;
		mac->ops.set_rate_select_speed =
					       yusur2_set_hard_rate_select_speed;
	} else {
		if ((yusur2_get_media_type(hw) == yusur2_media_type_backplane) &&
		     (hw->phy.smart_speed == yusur2_smart_speed_auto ||
		      hw->phy.smart_speed == yusur2_smart_speed_on) &&
		      !yusur2_verify_lesm_fw_enabled_82599(hw)) {
			mac->ops.setup_link = yusur2_setup_mac_link_smartspeed;
		} else {
			mac->ops.setup_link = yusur2_setup_mac_link_82599;
		}
	}
}

/**
 *  yusur2_init_phy_ops_82599 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during init_shared_code because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
s32 yusur2_init_phy_ops_82599(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;
	struct yusur2_phy_info *phy = &hw->phy;
	s32 ret_val = YUSUR2_SUCCESS;
	u32 esdp;

	DEBUGFUNC("yusur2_init_phy_ops_82599");

	if (hw->device_id == YUSUR2_DEV_ID_82599_QSFP_SF_QP) {
		/* Store flag indicating I2C bus access control unit. */
		hw->phy.qsfp_shared_i2c_bus = TRUE;

		/* Initialize access to QSFP+ I2C bus */
		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		esdp |= YUSUR2_ESDP_SDP0_DIR;
		esdp &= ~YUSUR2_ESDP_SDP1_DIR;
		esdp &= ~YUSUR2_ESDP_SDP0;
		esdp &= ~YUSUR2_ESDP_SDP0_NATIVE;
		esdp &= ~YUSUR2_ESDP_SDP1_NATIVE;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);

		phy->ops.read_i2c_byte = yusur2_read_i2c_byte_82599;
		phy->ops.write_i2c_byte = yusur2_write_i2c_byte_82599;
	}
	/* Identify the PHY or SFP module */
	ret_val = phy->ops.identify(hw);
	if (ret_val == YUSUR2_ERR_SFP_NOT_SUPPORTED)
		goto init_phy_ops_out;

	/* Setup function pointers based on detected SFP module and speeds */
	yusur2_init_mac_link_ops_82599(hw);
	if (hw->phy.sfp_type != yusur2_sfp_type_unknown)
		hw->phy.ops.reset = NULL;

	/* If copper media, overwrite with copper function pointers */
	if (mac->ops.get_media_type(hw) == yusur2_media_type_copper) {
		mac->ops.setup_link = yusur2_setup_copper_link_82599;
		mac->ops.get_link_capabilities =
				  yusur2_get_copper_link_capabilities_generic;
	}

	/* Set necessary function pointers based on PHY type */
	switch (hw->phy.type) {
	case yusur2_phy_tn:
		phy->ops.setup_link = yusur2_setup_phy_link_tnx;
		phy->ops.check_link = yusur2_check_phy_link_tnx;
		phy->ops.get_firmware_version =
			     yusur2_get_phy_firmware_version_tnx;
		break;
	default:
		break;
	}
init_phy_ops_out:
	return ret_val;
}

s32 yusur2_setup_sfp_modules_82599(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;
	u16 list_offset, data_offset, data_value;

	DEBUGFUNC("yusur2_setup_sfp_modules_82599");

	if (hw->phy.sfp_type != yusur2_sfp_type_unknown) {
		yusur2_init_mac_link_ops_82599(hw);

		hw->phy.ops.reset = NULL;

		ret_val = yusur2_get_sfp_init_sequence_offsets(hw, &list_offset,
							      &data_offset);
		if (ret_val != YUSUR2_SUCCESS)
			goto setup_sfp_out;

		/* PHY config will finish before releasing the semaphore */
		ret_val = hw->mac.ops.acquire_swfw_sync(hw,
							YUSUR2_GSSR_MAC_CSR_SM);
		if (ret_val != YUSUR2_SUCCESS) {
			ret_val = YUSUR2_ERR_SWFW_SYNC;
			goto setup_sfp_out;
		}

		if (hw->eeprom.ops.read(hw, ++data_offset, &data_value))
			goto setup_sfp_err;
		while (data_value != 0xffff) {
			YUSUR2_WRITE_REG(hw, YUSUR2_CORECTL, data_value);
			YUSUR2_WRITE_FLUSH(hw);
			if (hw->eeprom.ops.read(hw, ++data_offset, &data_value))
				goto setup_sfp_err;
		}

		/* Release the semaphore */
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM);
		/* Delay obtaining semaphore again to allow FW access
		 * prot_autoc_write uses the semaphore too.
		 */
		msec_delay(hw->eeprom.semaphore_delay);

		/* Restart DSP and set SFI mode */
		ret_val = hw->mac.ops.prot_autoc_write(hw,
			hw->mac.orig_autoc | YUSUR2_AUTOC_LMS_10G_SERIAL,
			false);

		if (ret_val) {
			DEBUGOUT("sfp module setup not complete\n");
			ret_val = YUSUR2_ERR_SFP_SETUP_NOT_COMPLETE;
			goto setup_sfp_out;
		}

	}

setup_sfp_out:
	return ret_val;

setup_sfp_err:
	/* Release the semaphore */
	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM);
	/* Delay obtaining semaphore again to allow FW access */
	msec_delay(hw->eeprom.semaphore_delay);
	ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
		      "eeprom read at offset %d failed", data_offset);
	return YUSUR2_ERR_PHY;
}

/**
 *  yusur2_prot_autoc_read_82599 - Hides MAC differences needed for AUTOC read
 *  @hw: pointer to hardware structure
 *  @locked: Return the if we locked for this read.
 *  @reg_val: Value we read from AUTOC
 *
 *  For this part (82599) we need to wrap read-modify-writes with a possible
 *  FW/SW lock.  It is assumed this lock will be freed with the next
 *  prot_autoc_write_82599().
 */
s32 yusur2_prot_autoc_read_82599(struct yusur2_hw *hw, bool *locked, u32 *reg_val)
{
	s32 ret_val;

	*locked = false;
	 /* If LESM is on then we need to hold the SW/FW semaphore. */
	if (yusur2_verify_lesm_fw_enabled_82599(hw)) {
		ret_val = hw->mac.ops.acquire_swfw_sync(hw,
					YUSUR2_GSSR_MAC_CSR_SM);
		if (ret_val != YUSUR2_SUCCESS)
			return YUSUR2_ERR_SWFW_SYNC;

		*locked = true;
	}

	*reg_val = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	return YUSUR2_SUCCESS;
}

/**
 * yusur2_prot_autoc_write_82599 - Hides MAC differences needed for AUTOC write
 * @hw: pointer to hardware structure
 * @autoc: value to write to AUTOC
 * @locked: bool to indicate whether the SW/FW lock was already taken by
 *           previous proc_autoc_read_82599.
 *
 * This part (82599) may need to hold the SW/FW lock around all writes to
 * AUTOC. Likewise after a write we need to do a pipeline reset.
 */
s32 yusur2_prot_autoc_write_82599(struct yusur2_hw *hw, u32 autoc, bool locked)
{
	s32 ret_val = YUSUR2_SUCCESS;

	/* Blocked by MNG FW so bail */
	if (yusur2_check_reset_blocked(hw))
		goto out;

	/* We only need to get the lock if:
	 *  - We didn't do it already (in the read part of a read-modify-write)
	 *  - LESM is enabled.
	 */
	if (!locked && yusur2_verify_lesm_fw_enabled_82599(hw)) {
		ret_val = hw->mac.ops.acquire_swfw_sync(hw,
					YUSUR2_GSSR_MAC_CSR_SM);
		if (ret_val != YUSUR2_SUCCESS)
			return YUSUR2_ERR_SWFW_SYNC;

		locked = true;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, autoc);
	ret_val = yusur2_reset_pipeline_82599(hw);

out:
	/* Free the SW/FW semaphore as we either grabbed it here or
	 * already had it when this function was called.
	 */
	if (locked)
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM);

	return ret_val;
}

/**
 *  yusur2_init_ops_82599 - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for 82599.
 *  Does not touch the hardware.
 **/

s32 yusur2_init_ops_82599(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;
	struct yusur2_phy_info *phy = &hw->phy;
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val;

	DEBUGFUNC("yusur2_init_ops_82599");

	yusur2_init_phy_ops_generic(hw);
	ret_val = yusur2_init_ops_generic(hw);

	/* PHY */
	phy->ops.identify = yusur2_identify_phy_82599;
	phy->ops.init = yusur2_init_phy_ops_82599;

	/* MAC */
	mac->ops.reset_hw = yusur2_reset_hw_82599;
	mac->ops.enable_relaxed_ordering = yusur2_enable_relaxed_ordering_gen2;
	mac->ops.get_media_type = yusur2_get_media_type_82599;
	mac->ops.get_supported_physical_layer =
				    yusur2_get_supported_physical_layer_82599;
	mac->ops.disable_sec_rx_path = yusur2_disable_sec_rx_path_generic;
	mac->ops.enable_sec_rx_path = yusur2_enable_sec_rx_path_generic;
	mac->ops.enable_rx_dma = yusur2_enable_rx_dma_82599;
	mac->ops.read_analog_reg8 = yusur2_read_analog_reg8_82599;
	mac->ops.write_analog_reg8 = yusur2_write_analog_reg8_82599;
	mac->ops.start_hw = yusur2_start_hw_82599;
	mac->ops.get_san_mac_addr = yusur2_get_san_mac_addr_generic;
	mac->ops.set_san_mac_addr = yusur2_set_san_mac_addr_generic;
	mac->ops.get_device_caps = yusur2_get_device_caps_generic;
	mac->ops.get_wwn_prefix = yusur2_get_wwn_prefix_generic;
	mac->ops.get_fcoe_boot_status = yusur2_get_fcoe_boot_status_generic;
	mac->ops.prot_autoc_read = yusur2_prot_autoc_read_82599;
	mac->ops.prot_autoc_write = yusur2_prot_autoc_write_82599;

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
	mac->ops.setup_sfp = yusur2_setup_sfp_modules_82599;
	mac->ops.set_mac_anti_spoofing = yusur2_set_mac_anti_spoofing;
	mac->ops.set_vlan_anti_spoofing = yusur2_set_vlan_anti_spoofing;

	/* Link */
	mac->ops.get_link_capabilities = yusur2_get_link_capabilities_82599;
	mac->ops.check_link = yusur2_check_mac_link_generic;
	mac->ops.setup_rxpba = yusur2_set_rxpba_generic;
	yusur2_init_mac_link_ops_82599(hw);

	mac->mcft_size		= YUSUR2_82599_MC_TBL_SIZE;
	mac->vft_size		= YUSUR2_82599_VFT_TBL_SIZE;
	mac->num_rar_entries	= YUSUR2_82599_RAR_ENTRIES;
	mac->rx_pb_size		= YUSUR2_82599_RX_PB_SIZE;
	mac->max_rx_queues	= YUSUR2_82599_MAX_RX_QUEUES;
	mac->max_tx_queues	= YUSUR2_82599_MAX_TX_QUEUES;
	mac->max_msix_vectors	= yusur2_get_pcie_msix_count_generic(hw);

	mac->arc_subsystem_valid = !!(YUSUR2_READ_REG(hw, YUSUR2_FWSM_BY_MAC(hw))
				      & YUSUR2_FWSM_MODE_MASK);

	hw->mbx.ops.init_params = yusur2_init_mbx_params_pf;

	/* EEPROM */
	eeprom->ops.read = yusur2_read_eeprom_82599;
	eeprom->ops.read_buffer = yusur2_read_eeprom_buffer_82599;

	/* Manageability interface */
	mac->ops.set_fw_drv_ver = yusur2_set_fw_drv_ver_generic;

	mac->ops.get_thermal_sensor_data =
					 yusur2_get_thermal_sensor_data_generic;
	mac->ops.init_thermal_sensor_thresh =
				      yusur2_init_thermal_sensor_thresh_generic;

	mac->ops.get_rtrup2tc = yusur2_dcb_get_rtrup2tc_generic;

	return ret_val;
}

/**
 *  yusur2_get_link_capabilities_82599 - Determines link capabilities
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @autoneg: true when autoneg or autotry is enabled
 *
 *  Determines the link capabilities by reading the AUTOC register.
 **/
s32 yusur2_get_link_capabilities_82599(struct yusur2_hw *hw,
				      yusur2_link_speed *speed,
				      bool *autoneg)
{
	s32 status = YUSUR2_SUCCESS;
	u32 autoc = 0;

	DEBUGFUNC("yusur2_get_link_capabilities_82599");


	/* Check if 1G SFP module. */
	if (hw->phy.sfp_type == yusur2_sfp_type_1g_cu_core0 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_cu_core1 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_lha_core0 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_lha_core1 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_lx_core0 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_lx_core1 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_sx_core0 ||
	    hw->phy.sfp_type == yusur2_sfp_type_1g_sx_core1) {
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		goto out;
	}

	/*
	 * Determine link capabilities based on the stored value of AUTOC,
	 * which represents EEPROM defaults.  If AUTOC value has not
	 * been stored, use the current register values.
	 */
	if (hw->mac.orig_link_settings_stored)
		autoc = hw->mac.orig_autoc;
	else
		autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);

	switch (autoc & YUSUR2_AUTOC_LMS_MASK) {
	case YUSUR2_AUTOC_LMS_1G_LINK_NO_AN:
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = false;
		break;

	case YUSUR2_AUTOC_LMS_10G_LINK_NO_AN:
		*speed = YUSUR2_LINK_SPEED_10GB_FULL;
		*autoneg = false;
		break;

	case YUSUR2_AUTOC_LMS_1G_AN:
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case YUSUR2_AUTOC_LMS_10G_SERIAL:
		*speed = YUSUR2_LINK_SPEED_10GB_FULL;
		*autoneg = false;
		break;

	case YUSUR2_AUTOC_LMS_KX4_KX_KR:
	case YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
		if (autoc & YUSUR2_AUTOC_KR_SUPP)
			*speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (autoc & YUSUR2_AUTOC_KX4_SUPP)
			*speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (autoc & YUSUR2_AUTOC_KX_SUPP)
			*speed |= YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case YUSUR2_AUTOC_LMS_KX4_KX_KR_SGMII:
		*speed = YUSUR2_LINK_SPEED_100_FULL;
		if (autoc & YUSUR2_AUTOC_KR_SUPP)
			*speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (autoc & YUSUR2_AUTOC_KX4_SUPP)
			*speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (autoc & YUSUR2_AUTOC_KX_SUPP)
			*speed |= YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case YUSUR2_AUTOC_LMS_SGMII_1G_100M:
		*speed = YUSUR2_LINK_SPEED_1GB_FULL | YUSUR2_LINK_SPEED_100_FULL;
		*autoneg = false;
		break;

	default:
		status = YUSUR2_ERR_LINK_SETUP;
		goto out;
		break;
	}

	if (hw->phy.multispeed_fiber) {
		*speed |= YUSUR2_LINK_SPEED_10GB_FULL |
			  YUSUR2_LINK_SPEED_1GB_FULL;

		/* QSFP must not enable full auto-negotiation
		 * Limited autoneg is enabled at 1G
		 */
		if (hw->phy.media_type == yusur2_media_type_fiber_qsfp)
			*autoneg = false;
		else
			*autoneg = true;
	}

out:
	return status;
}

/**
 *  yusur2_get_media_type_82599 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
enum yusur2_media_type yusur2_get_media_type_82599(struct yusur2_hw *hw)
{
	enum yusur2_media_type media_type;

	DEBUGFUNC("yusur2_get_media_type_82599");

	/* Detect if there is a copper PHY attached. */
	switch (hw->phy.type) {
	case yusur2_phy_cu_unknown:
	case yusur2_phy_tn:
		media_type = yusur2_media_type_copper;
		goto out;
	default:
		break;
	}

	switch (hw->device_id) {
	case YUSUR2_DEV_ID_82599_KX4:
	case YUSUR2_DEV_ID_82599_KX4_MEZZ:
	case YUSUR2_DEV_ID_82599_COMBO_BACKPLANE:
	case YUSUR2_DEV_ID_82599_KR:
	case YUSUR2_DEV_ID_82599_BACKPLANE_FCOE:
	case YUSUR2_DEV_ID_82599_XAUI_LOM:
		/* Default device ID is mezzanine card KX/KX4 */
		media_type = yusur2_media_type_backplane;
		break;
	case YUSUR2_DEV_ID_82599_SFP:
	case YUSUR2_DEV_ID_82599_SFP_FCOE:
	case YUSUR2_DEV_ID_82599_SFP_EM:
	case YUSUR2_DEV_ID_82599_SFP_SF2:
	case YUSUR2_DEV_ID_82599_SFP_SF_QP:
	case YUSUR2_DEV_ID_82599EN_SFP:
		media_type = yusur2_media_type_fiber;
		break;
	case YUSUR2_DEV_ID_82599_CX4:
		media_type = yusur2_media_type_cx4;
		break;
	case YUSUR2_DEV_ID_82599_T3_LOM:
		media_type = yusur2_media_type_copper;
		break;
	case YUSUR2_DEV_ID_82599_QSFP_SF_QP:
		media_type = yusur2_media_type_fiber_qsfp;
		break;
	default:
		media_type = yusur2_media_type_unknown;
		break;
	}
out:
	return media_type;
}

/**
 *  yusur2_stop_mac_link_on_d3_82599 - Disables link on D3
 *  @hw: pointer to hardware structure
 *
 *  Disables link during D3 power down sequence.
 *
 **/
void yusur2_stop_mac_link_on_d3_82599(struct yusur2_hw *hw)
{
	u32 autoc2_reg;
	u16 ee_ctrl_2 = 0;

	DEBUGFUNC("yusur2_stop_mac_link_on_d3_82599");
	yusur2_read_eeprom(hw, YUSUR2_EEPROM_CTRL_2, &ee_ctrl_2);

	if (!yusur2_mng_present(hw) && !hw->wol_enabled &&
	    ee_ctrl_2 & YUSUR2_EEPROM_CCD_BIT) {
		autoc2_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC2);
		autoc2_reg |= YUSUR2_AUTOC2_LINK_DISABLE_ON_D3_MASK;
		YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC2, autoc2_reg);
	}
}

/**
 *  yusur2_start_mac_link_82599 - Setup MAC link settings
 *  @hw: pointer to hardware structure
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Configures link settings based on values in the yusur2_hw struct.
 *  Restarts the link.  Performs autonegotiation if needed.
 **/
s32 yusur2_start_mac_link_82599(struct yusur2_hw *hw,
			       bool autoneg_wait_to_complete)
{
	u32 autoc_reg;
	u32 links_reg;
	u32 i;
	s32 status = YUSUR2_SUCCESS;
	bool got_lock = false;

	DEBUGFUNC("yusur2_start_mac_link_82599");


	/*  reset_pipeline requires us to hold this lock as it writes to
	 *  AUTOC.
	 */
	if (yusur2_verify_lesm_fw_enabled_82599(hw)) {
		status = hw->mac.ops.acquire_swfw_sync(hw,
						       YUSUR2_GSSR_MAC_CSR_SM);
		if (status != YUSUR2_SUCCESS)
			goto out;

		got_lock = true;
	}

	/* Restart link */
	yusur2_reset_pipeline_82599(hw);

	if (got_lock)
		hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM);

	/* Only poll for autoneg to complete if specified to do so */
	if (autoneg_wait_to_complete) {
		autoc_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
		if ((autoc_reg & YUSUR2_AUTOC_LMS_MASK) ==
		     YUSUR2_AUTOC_LMS_KX4_KX_KR ||
		    (autoc_reg & YUSUR2_AUTOC_LMS_MASK) ==
		     YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN ||
		    (autoc_reg & YUSUR2_AUTOC_LMS_MASK) ==
		     YUSUR2_AUTOC_LMS_KX4_KX_KR_SGMII) {
			links_reg = 0; /* Just in case Autoneg time = 0 */
			for (i = 0; i < YUSUR2_AUTO_NEG_TIME; i++) {
				links_reg = YUSUR2_READ_REG(hw, YUSUR2_LINKS);
				if (links_reg & YUSUR2_LINKS_KX_AN_COMP)
					break;
				msec_delay(100);
			}
			if (!(links_reg & YUSUR2_LINKS_KX_AN_COMP)) {
				status = YUSUR2_ERR_AUTONEG_NOT_COMPLETE;
				DEBUGOUT("Autoneg did not complete.\n");
			}
		}
	}

	/* Add delay to filter out noises during initial link setup */
	msec_delay(50);

out:
	return status;
}

/**
 *  yusur2_disable_tx_laser_multispeed_fiber - Disable Tx laser
 *  @hw: pointer to hardware structure
 *
 *  The base drivers may require better control over SFP+ module
 *  PHY states.  This includes selectively shutting down the Tx
 *  laser on the PHY, effectively halting physical link.
 **/
void yusur2_disable_tx_laser_multispeed_fiber(struct yusur2_hw *hw)
{
	u32 esdp_reg = YUSUR2_READ_REG(hw, YUSUR2_ESDP);

	/* Blocked by MNG FW so bail */
	if (yusur2_check_reset_blocked(hw))
		return;

	/* Disable Tx laser; allow 100us to go dark per spec */
	esdp_reg |= YUSUR2_ESDP_SDP3;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp_reg);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(100);
}

/**
 *  yusur2_enable_tx_laser_multispeed_fiber - Enable Tx laser
 *  @hw: pointer to hardware structure
 *
 *  The base drivers may require better control over SFP+ module
 *  PHY states.  This includes selectively turning on the Tx
 *  laser on the PHY, effectively starting physical link.
 **/
void yusur2_enable_tx_laser_multispeed_fiber(struct yusur2_hw *hw)
{
	u32 esdp_reg = YUSUR2_READ_REG(hw, YUSUR2_ESDP);

	/* Enable Tx laser; allow 100ms to light up */
	esdp_reg &= ~YUSUR2_ESDP_SDP3;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp_reg);
	YUSUR2_WRITE_FLUSH(hw);
	msec_delay(100);
}

/**
 *  yusur2_flap_tx_laser_multispeed_fiber - Flap Tx laser
 *  @hw: pointer to hardware structure
 *
 *  When the driver changes the link speeds that it can support,
 *  it sets autotry_restart to true to indicate that we need to
 *  initiate a new autotry session with the link partner.  To do
 *  so, we set the speed then disable and re-enable the Tx laser, to
 *  alert the link partner that it also needs to restart autotry on its
 *  end.  This is consistent with true clause 37 autoneg, which also
 *  involves a loss of signal.
 **/
void yusur2_flap_tx_laser_multispeed_fiber(struct yusur2_hw *hw)
{
	DEBUGFUNC("yusur2_flap_tx_laser_multispeed_fiber");

	/* Blocked by MNG FW so bail */
	if (yusur2_check_reset_blocked(hw))
		return;

	if (hw->mac.autotry_restart) {
		yusur2_disable_tx_laser_multispeed_fiber(hw);
		yusur2_enable_tx_laser_multispeed_fiber(hw);
		hw->mac.autotry_restart = false;
	}
}

/**
 *  yusur2_set_hard_rate_select_speed - Set module link speed
 *  @hw: pointer to hardware structure
 *  @speed: link speed to set
 *
 *  Set module link speed via RS0/RS1 rate select pins.
 */
void yusur2_set_hard_rate_select_speed(struct yusur2_hw *hw,
					yusur2_link_speed speed)
{
	u32 esdp_reg = YUSUR2_READ_REG(hw, YUSUR2_ESDP);

	switch (speed) {
	case YUSUR2_LINK_SPEED_10GB_FULL:
		esdp_reg |= (YUSUR2_ESDP_SDP5_DIR | YUSUR2_ESDP_SDP5);
		break;
	case YUSUR2_LINK_SPEED_1GB_FULL:
		esdp_reg &= ~YUSUR2_ESDP_SDP5;
		esdp_reg |= YUSUR2_ESDP_SDP5_DIR;
		break;
	default:
		DEBUGOUT("Invalid fixed module speed\n");
		return;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp_reg);
	YUSUR2_WRITE_FLUSH(hw);
}

/**
 *  yusur2_setup_mac_link_smartspeed - Set MAC link speed using SmartSpeed
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Implements the Intel SmartSpeed algorithm.
 **/
s32 yusur2_setup_mac_link_smartspeed(struct yusur2_hw *hw,
				    yusur2_link_speed speed,
				    bool autoneg_wait_to_complete)
{
	s32 status = YUSUR2_SUCCESS;
	yusur2_link_speed link_speed = YUSUR2_LINK_SPEED_UNKNOWN;
	s32 i, j;
	bool link_up = false;
	u32 autoc_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);

	DEBUGFUNC("yusur2_setup_mac_link_smartspeed");

	 /* Set autoneg_advertised value based on input link speed */
	hw->phy.autoneg_advertised = 0;

	if (speed & YUSUR2_LINK_SPEED_10GB_FULL)
		hw->phy.autoneg_advertised |= YUSUR2_LINK_SPEED_10GB_FULL;

	if (speed & YUSUR2_LINK_SPEED_1GB_FULL)
		hw->phy.autoneg_advertised |= YUSUR2_LINK_SPEED_1GB_FULL;

	if (speed & YUSUR2_LINK_SPEED_100_FULL)
		hw->phy.autoneg_advertised |= YUSUR2_LINK_SPEED_100_FULL;

	/*
	 * Implement Intel SmartSpeed algorithm.  SmartSpeed will reduce the
	 * autoneg advertisement if link is unable to be established at the
	 * highest negotiated rate.  This can sometimes happen due to integrity
	 * issues with the physical media connection.
	 */

	/* First, try to get link with full advertisement */
	hw->phy.smart_speed_active = false;
	for (j = 0; j < YUSUR2_SMARTSPEED_MAX_RETRIES; j++) {
		status = yusur2_setup_mac_link_82599(hw, speed,
						    autoneg_wait_to_complete);
		if (status != YUSUR2_SUCCESS)
			goto out;

		/*
		 * Wait for the controller to acquire link.  Per IEEE 802.3ap,
		 * Section 73.10.2, we may have to wait up to 500ms if KR is
		 * attempted, or 200ms if KX/KX4/BX/BX4 is attempted, per
		 * Table 9 in the AN MAS.
		 */
		for (i = 0; i < 5; i++) {
			msec_delay(100);

			/* If we have link, just jump out */
			status = yusur2_check_link(hw, &link_speed, &link_up,
						  false);
			if (status != YUSUR2_SUCCESS)
				goto out;

			if (link_up)
				goto out;
		}
	}

	/*
	 * We didn't get link.  If we advertised KR plus one of KX4/KX
	 * (or BX4/BX), then disable KR and try again.
	 */
	if (((autoc_reg & YUSUR2_AUTOC_KR_SUPP) == 0) ||
	    ((autoc_reg & YUSUR2_AUTOC_KX4_KX_SUPP_MASK) == 0))
		goto out;

	/* Turn SmartSpeed on to disable KR support */
	hw->phy.smart_speed_active = true;
	status = yusur2_setup_mac_link_82599(hw, speed,
					    autoneg_wait_to_complete);
	if (status != YUSUR2_SUCCESS)
		goto out;

	/*
	 * Wait for the controller to acquire link.  600ms will allow for
	 * the AN link_fail_inhibit_timer as well for multiple cycles of
	 * parallel detect, both 10g and 1g. This allows for the maximum
	 * connect attempts as defined in the AN MAS table 73-7.
	 */
	for (i = 0; i < 6; i++) {
		msec_delay(100);

		/* If we have link, just jump out */
		status = yusur2_check_link(hw, &link_speed, &link_up, false);
		if (status != YUSUR2_SUCCESS)
			goto out;

		if (link_up)
			goto out;
	}

	/* We didn't get link.  Turn SmartSpeed back off. */
	hw->phy.smart_speed_active = false;
	status = yusur2_setup_mac_link_82599(hw, speed,
					    autoneg_wait_to_complete);

out:
	if (link_up && (link_speed == YUSUR2_LINK_SPEED_1GB_FULL))
		DEBUGOUT("Smartspeed has downgraded the link speed "
		"from the maximum advertised\n");
	return status;
}

/**
 *  yusur2_setup_mac_link_82599 - Set MAC link speed
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Set the link speed in the AUTOC register and restarts link.
 **/
s32 yusur2_setup_mac_link_82599(struct yusur2_hw *hw,
			       yusur2_link_speed speed,
			       bool autoneg_wait_to_complete)
{
	bool autoneg = false;
	s32 status = YUSUR2_SUCCESS;
	u32 pma_pmd_1g, link_mode;
	u32 current_autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC); /* holds the value of AUTOC register at this current point in time */
	u32 orig_autoc = 0; /* holds the cached value of AUTOC register */
	u32 autoc = current_autoc; /* Temporary variable used for comparison purposes */
	u32 autoc2 = YUSUR2_READ_REG(hw, YUSUR2_AUTOC2);
	u32 pma_pmd_10g_serial = autoc2 & YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_MASK;
	u32 links_reg;
	u32 i;
	yusur2_link_speed link_capabilities = YUSUR2_LINK_SPEED_UNKNOWN;

	DEBUGFUNC("yusur2_setup_mac_link_82599");

	/* Check to see if speed passed in is supported. */
	status = yusur2_get_link_capabilities(hw, &link_capabilities, &autoneg);
	if (status)
		goto out;

	speed &= link_capabilities;

	if (speed == YUSUR2_LINK_SPEED_UNKNOWN) {
		status = YUSUR2_ERR_LINK_SETUP;
		goto out;
	}

	/* Use stored value (EEPROM defaults) of AUTOC to find KR/KX4 support*/
	if (hw->mac.orig_link_settings_stored)
		orig_autoc = hw->mac.orig_autoc;
	else
		orig_autoc = autoc;

	link_mode = autoc & YUSUR2_AUTOC_LMS_MASK;
	pma_pmd_1g = autoc & YUSUR2_AUTOC_1G_PMA_PMD_MASK;

	if (link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR ||
	    link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN ||
	    link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR_SGMII) {
		/* Set KX4/KX/KR support according to speed requested */
		autoc &= ~(YUSUR2_AUTOC_KX4_KX_SUPP_MASK | YUSUR2_AUTOC_KR_SUPP);
		if (speed & YUSUR2_LINK_SPEED_10GB_FULL) {
			if (orig_autoc & YUSUR2_AUTOC_KX4_SUPP)
				autoc |= YUSUR2_AUTOC_KX4_SUPP;
			if ((orig_autoc & YUSUR2_AUTOC_KR_SUPP) &&
			    (hw->phy.smart_speed_active == false))
				autoc |= YUSUR2_AUTOC_KR_SUPP;
		}
		if (speed & YUSUR2_LINK_SPEED_1GB_FULL)
			autoc |= YUSUR2_AUTOC_KX_SUPP;
	} else if ((pma_pmd_1g == YUSUR2_AUTOC_1G_SFI) &&
		   (link_mode == YUSUR2_AUTOC_LMS_1G_LINK_NO_AN ||
		    link_mode == YUSUR2_AUTOC_LMS_1G_AN)) {
		/* Switch from 1G SFI to 10G SFI if requested */
		if ((speed == YUSUR2_LINK_SPEED_10GB_FULL) &&
		    (pma_pmd_10g_serial == YUSUR2_AUTOC2_10G_SFI)) {
			autoc &= ~YUSUR2_AUTOC_LMS_MASK;
			autoc |= YUSUR2_AUTOC_LMS_10G_SERIAL;
		}
	} else if ((pma_pmd_10g_serial == YUSUR2_AUTOC2_10G_SFI) &&
		   (link_mode == YUSUR2_AUTOC_LMS_10G_SERIAL)) {
		/* Switch from 10G SFI to 1G SFI if requested */
		if ((speed == YUSUR2_LINK_SPEED_1GB_FULL) &&
		    (pma_pmd_1g == YUSUR2_AUTOC_1G_SFI)) {
			autoc &= ~YUSUR2_AUTOC_LMS_MASK;
			if (autoneg || hw->phy.type == yusur2_phy_qsfp_intel)
				autoc |= YUSUR2_AUTOC_LMS_1G_AN;
			else
				autoc |= YUSUR2_AUTOC_LMS_1G_LINK_NO_AN;
		}
	}

	if (autoc != current_autoc) {
		/* Restart link */
		status = hw->mac.ops.prot_autoc_write(hw, autoc, false);
		if (status != YUSUR2_SUCCESS)
			goto out;

		/* Only poll for autoneg to complete if specified to do so */
		if (autoneg_wait_to_complete) {
			if (link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR ||
			    link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN ||
			    link_mode == YUSUR2_AUTOC_LMS_KX4_KX_KR_SGMII) {
				links_reg = 0; /*Just in case Autoneg time=0*/
				for (i = 0; i < YUSUR2_AUTO_NEG_TIME; i++) {
					links_reg =
					       YUSUR2_READ_REG(hw, YUSUR2_LINKS);
					if (links_reg & YUSUR2_LINKS_KX_AN_COMP)
						break;
					msec_delay(100);
				}
				if (!(links_reg & YUSUR2_LINKS_KX_AN_COMP)) {
					status =
						YUSUR2_ERR_AUTONEG_NOT_COMPLETE;
					DEBUGOUT("Autoneg did not complete.\n");
				}
			}
		}

		/* Add delay to filter out noises during initial link setup */
		msec_delay(50);
	}

out:
	return status;
}

/**
 *  yusur2_setup_copper_link_82599 - Set the PHY autoneg advertised field
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true if waiting is needed to complete
 *
 *  Restarts link on PHY and MAC based on settings passed in.
 **/
STATIC s32 yusur2_setup_copper_link_82599(struct yusur2_hw *hw,
					 yusur2_link_speed speed,
					 bool autoneg_wait_to_complete)
{
	s32 status;

	DEBUGFUNC("yusur2_setup_copper_link_82599");

	/* Setup the PHY according to input speed */
	status = hw->phy.ops.setup_link_speed(hw, speed,
					      autoneg_wait_to_complete);
	/* Set up MAC */
	yusur2_start_mac_link_82599(hw, autoneg_wait_to_complete);

	return status;
}

/**
 *  yusur2_reset_hw_82599 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 yusur2_reset_hw_82599(struct yusur2_hw *hw)
{
	yusur2_link_speed link_speed;
	s32 status;
	u32 ctrl = 0;
	u32 i, autoc, autoc2;
	u32 curr_lms;
	bool link_up = false;

	DEBUGFUNC("yusur2_reset_hw_82599");

	/* Call adapter stop to disable tx/rx and clear interrupts */
	status = hw->mac.ops.stop_adapter(hw);
	if (status != YUSUR2_SUCCESS)
		goto reset_hw_out;

	/* flush pending Tx transactions */
	yusur2_clear_tx_pending(hw);

	/* PHY ops must be identified and initialized prior to reset */

	/* Identify PHY and related function pointers */
	status = hw->phy.ops.init(hw);

	if (status == YUSUR2_ERR_SFP_NOT_SUPPORTED)
		goto reset_hw_out;

	/* Setup SFP module if there is one present. */
	if (hw->phy.sfp_setup_needed) {
		status = hw->mac.ops.setup_sfp(hw);
		hw->phy.sfp_setup_needed = false;
	}

	if (status == YUSUR2_ERR_SFP_NOT_SUPPORTED)
		goto reset_hw_out;

	/* Reset PHY */
	if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL)
		hw->phy.ops.reset(hw);

	/* remember AUTOC from before we reset */
	curr_lms = YUSUR2_READ_REG(hw, YUSUR2_AUTOC) & YUSUR2_AUTOC_LMS_MASK;

mac_reset_top:
	/*
	 * Issue global reset to the MAC.  Needs to be SW reset if link is up.
	 * If link reset is used when link is up, it might reset the PHY when
	 * mng is using it.  If link is down or the flag to force full link
	 * reset is set, then perform link reset.
	 */
	ctrl = YUSUR2_CTRL_LNK_RST;
	if (!hw->force_full_reset) {
		hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
		if (link_up)
			ctrl = YUSUR2_CTRL_RST;
	}

	ctrl |= YUSUR2_READ_REG(hw, YUSUR2_CTRL);
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL, ctrl);
	YUSUR2_WRITE_FLUSH(hw);

	/* Poll for reset bit to self-clear meaning reset is complete */
	for (i = 0; i < 10; i++) {
		usec_delay(1);
		ctrl = YUSUR2_READ_REG(hw, YUSUR2_CTRL);
		if (!(ctrl & YUSUR2_CTRL_RST_MASK))
			break;
	}

	if (ctrl & YUSUR2_CTRL_RST_MASK) {
		status = YUSUR2_ERR_RESET_FAILED;
		DEBUGOUT("Reset polling failed to complete.\n");
	}

	msec_delay(50);

	/*
	 * Double resets are required for recovery from certain error
	 * conditions.  Between resets, it is necessary to stall to
	 * allow time for any pending HW events to complete.
	 */
	if (hw->mac.flags & YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED) {
		hw->mac.flags &= ~YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED;
		goto mac_reset_top;
	}

	/*
	 * Store the original AUTOC/AUTOC2 values if they have not been
	 * stored off yet.  Otherwise restore the stored original
	 * values since the reset operation sets back to defaults.
	 */
	autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	autoc2 = YUSUR2_READ_REG(hw, YUSUR2_AUTOC2);

	/* Enable link if disabled in NVM */
	if (autoc2 & YUSUR2_AUTOC2_LINK_DISABLE_MASK) {
		autoc2 &= ~YUSUR2_AUTOC2_LINK_DISABLE_MASK;
		YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC2, autoc2);
		YUSUR2_WRITE_FLUSH(hw);
	}

	if (hw->mac.orig_link_settings_stored == false) {
		hw->mac.orig_autoc = autoc;
		hw->mac.orig_autoc2 = autoc2;
		hw->mac.orig_link_settings_stored = true;
	} else {

		/* If MNG FW is running on a multi-speed device that
		 * doesn't autoneg with out driver support we need to
		 * leave LMS in the state it was before we MAC reset.
		 * Likewise if we support WoL we don't want change the
		 * LMS state.
		 */
		if ((hw->phy.multispeed_fiber && yusur2_mng_enabled(hw)) ||
		    hw->wol_enabled)
			hw->mac.orig_autoc =
				(hw->mac.orig_autoc & ~YUSUR2_AUTOC_LMS_MASK) |
				curr_lms;

		if (autoc != hw->mac.orig_autoc) {
			status = hw->mac.ops.prot_autoc_write(hw,
							hw->mac.orig_autoc,
							false);
			if (status != YUSUR2_SUCCESS)
				goto reset_hw_out;
		}

		if ((autoc2 & YUSUR2_AUTOC2_UPPER_MASK) !=
		    (hw->mac.orig_autoc2 & YUSUR2_AUTOC2_UPPER_MASK)) {
			autoc2 &= ~YUSUR2_AUTOC2_UPPER_MASK;
			autoc2 |= (hw->mac.orig_autoc2 &
				   YUSUR2_AUTOC2_UPPER_MASK);
			YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC2, autoc2);
		}
	}

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
 * yusur2_fdir_check_cmd_complete - poll to check whether FDIRCMD is complete
 * @hw: pointer to hardware structure
 * @fdircmd: current value of FDIRCMD register
 */
STATIC s32 yusur2_fdir_check_cmd_complete(struct yusur2_hw *hw, u32 *fdircmd)
{
	int i;

	for (i = 0; i < YUSUR2_FDIRCMD_CMD_POLL; i++) {
		*fdircmd = YUSUR2_READ_REG(hw, YUSUR2_FDIRCMD);
		if (!(*fdircmd & YUSUR2_FDIRCMD_CMD_MASK))
			return YUSUR2_SUCCESS;
		usec_delay(10);
	}

	return YUSUR2_ERR_FDIR_CMD_INCOMPLETE;
}

/**
 *  yusur2_reinit_fdir_tables_82599 - Reinitialize Flow Director tables.
 *  @hw: pointer to hardware structure
 **/
s32 yusur2_reinit_fdir_tables_82599(struct yusur2_hw *hw)
{
	s32 err;
	int i;
	u32 fdirctrl = YUSUR2_READ_REG(hw, YUSUR2_FDIRCTRL);
	u32 fdircmd;
	fdirctrl &= ~YUSUR2_FDIRCTRL_INIT_DONE;

	DEBUGFUNC("yusur2_reinit_fdir_tables_82599");

	/*
	 * Before starting reinitialization process,
	 * FDIRCMD.CMD must be zero.
	 */
	err = yusur2_fdir_check_cmd_complete(hw, &fdircmd);
	if (err) {
		DEBUGOUT("Flow Director previous command did not complete, aborting table re-initialization.\n");
		return err;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRFREE, 0);
	YUSUR2_WRITE_FLUSH(hw);
	/*
	 * 82599 adapters flow director init flow cannot be restarted,
	 * Workaround 82599 silicon errata by performing the following steps
	 * before re-writing the FDIRCTRL control register with the same value.
	 * - write 1 to bit 8 of FDIRCMD register &
	 * - write 0 to bit 8 of FDIRCMD register
	 */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD,
			(YUSUR2_READ_REG(hw, YUSUR2_FDIRCMD) |
			 YUSUR2_FDIRCMD_CLEARHT));
	YUSUR2_WRITE_FLUSH(hw);
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD,
			(YUSUR2_READ_REG(hw, YUSUR2_FDIRCMD) &
			 ~YUSUR2_FDIRCMD_CLEARHT));
	YUSUR2_WRITE_FLUSH(hw);
	/*
	 * Clear FDIR Hash register to clear any leftover hashes
	 * waiting to be programmed.
	 */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRHASH, 0x00);
	YUSUR2_WRITE_FLUSH(hw);

	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCTRL, fdirctrl);
	YUSUR2_WRITE_FLUSH(hw);

	/* Poll init-done after we write FDIRCTRL register */
	for (i = 0; i < YUSUR2_FDIR_INIT_DONE_POLL; i++) {
		if (YUSUR2_READ_REG(hw, YUSUR2_FDIRCTRL) &
				   YUSUR2_FDIRCTRL_INIT_DONE)
			break;
		msec_delay(1);
	}
	if (i >= YUSUR2_FDIR_INIT_DONE_POLL) {
		DEBUGOUT("Flow Director Signature poll time exceeded!\n");
		return YUSUR2_ERR_FDIR_REINIT_FAILED;
	}

	/* Clear FDIR statistics registers (read to clear) */
	YUSUR2_READ_REG(hw, YUSUR2_FDIRUSTAT);
	YUSUR2_READ_REG(hw, YUSUR2_FDIRFSTAT);
	YUSUR2_READ_REG(hw, YUSUR2_FDIRMATCH);
	YUSUR2_READ_REG(hw, YUSUR2_FDIRMISS);
	YUSUR2_READ_REG(hw, YUSUR2_FDIRLEN);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_fdir_enable_82599 - Initialize Flow Director control registers
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register
 **/
STATIC void yusur2_fdir_enable_82599(struct yusur2_hw *hw, u32 fdirctrl)
{
	int i;

	DEBUGFUNC("yusur2_fdir_enable_82599");

	/* Prime the keys for hashing */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRHKEY, YUSUR2_ATR_BUCKET_HASH_KEY);
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRSKEY, YUSUR2_ATR_SIGNATURE_HASH_KEY);

	/*
	 * Poll init-done after we write the register.  Estimated times:
	 *      10G: PBALLOC = 11b, timing is 60us
	 *       1G: PBALLOC = 11b, timing is 600us
	 *     100M: PBALLOC = 11b, timing is 6ms
	 *
	 *     Multiple these timings by 4 if under full Rx load
	 *
	 * So we'll poll for YUSUR2_FDIR_INIT_DONE_POLL times, sleeping for
	 * 1 msec per poll time.  If we're at line rate and drop to 100M, then
	 * this might not finish in our poll time, but we can live with that
	 * for now.
	 */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCTRL, fdirctrl);
	YUSUR2_WRITE_FLUSH(hw);
	for (i = 0; i < YUSUR2_FDIR_INIT_DONE_POLL; i++) {
		if (YUSUR2_READ_REG(hw, YUSUR2_FDIRCTRL) &
				   YUSUR2_FDIRCTRL_INIT_DONE)
			break;
		msec_delay(1);
	}

	if (i >= YUSUR2_FDIR_INIT_DONE_POLL)
		DEBUGOUT("Flow Director poll time exceeded!\n");
}

/**
 *  yusur2_init_fdir_signature_82599 - Initialize Flow Director signature filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *	     contains just the value of the Rx packet buffer allocation
 **/
s32 yusur2_init_fdir_signature_82599(struct yusur2_hw *hw, u32 fdirctrl)
{
	DEBUGFUNC("yusur2_init_fdir_signature_82599");

	/*
	 * Continue setup of fdirctrl register bits:
	 *  Move the flexible bytes to use the ethertype - shift 6 words
	 *  Set the maximum length per hash bucket to 0xA filters
	 *  Send interrupt when 64 filters are left
	 */
	fdirctrl |= (0x6 << YUSUR2_FDIRCTRL_FLEX_SHIFT) |
		    (0xA << YUSUR2_FDIRCTRL_MAX_LENGTH_SHIFT) |
		    (4 << YUSUR2_FDIRCTRL_FULL_THRESH_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	yusur2_fdir_enable_82599(hw, fdirctrl);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_fdir_perfect_82599 - Initialize Flow Director perfect filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *	     contains just the value of the Rx packet buffer allocation
 *  @cloud_mode: true - cloud mode, false - other mode
 **/
s32 yusur2_init_fdir_perfect_82599(struct yusur2_hw *hw, u32 fdirctrl,
			bool cloud_mode)
{
	UNREFERENCED_1PARAMETER(cloud_mode);
	DEBUGFUNC("yusur2_init_fdir_perfect_82599");

	/*
	 * Continue setup of fdirctrl register bits:
	 *  Turn perfect match filtering on
	 *  Report hash in RSS field of Rx wb descriptor
	 *  Initialize the drop queue to queue 127
	 *  Move the flexible bytes to use the ethertype - shift 6 words
	 *  Set the maximum length per hash bucket to 0xA filters
	 *  Send interrupt when 64 (0x4 * 16) filters are left
	 */
	fdirctrl |= YUSUR2_FDIRCTRL_PERFECT_MATCH |
		    YUSUR2_FDIRCTRL_REPORT_STATUS |
		    (YUSUR2_FDIR_DROP_QUEUE << YUSUR2_FDIRCTRL_DROP_Q_SHIFT) |
		    (0x6 << YUSUR2_FDIRCTRL_FLEX_SHIFT) |
		    (0xA << YUSUR2_FDIRCTRL_MAX_LENGTH_SHIFT) |
		    (4 << YUSUR2_FDIRCTRL_FULL_THRESH_SHIFT);

	if (cloud_mode)
		fdirctrl |=(YUSUR2_FDIRCTRL_FILTERMODE_CLOUD <<
					YUSUR2_FDIRCTRL_FILTERMODE_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	yusur2_fdir_enable_82599(hw, fdirctrl);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_fdir_drop_queue_82599 - Set Flow Director drop queue
 *  @hw: pointer to hardware structure
 *  @dropqueue: Rx queue index used for the dropped packets
 **/
void yusur2_set_fdir_drop_queue_82599(struct yusur2_hw *hw, u8 dropqueue)
{
	u32 fdirctrl;

	DEBUGFUNC("yusur2_set_fdir_drop_queue_82599");
	/* Clear init done bit and drop queue field */
	fdirctrl = YUSUR2_READ_REG(hw, YUSUR2_FDIRCTRL);
	fdirctrl &= ~(YUSUR2_FDIRCTRL_DROP_Q_MASK | YUSUR2_FDIRCTRL_INIT_DONE);

	/* Set drop queue */
	fdirctrl |= (dropqueue << YUSUR2_FDIRCTRL_DROP_Q_SHIFT);
	if ((hw->mac.type == yusur2_mac_X550) ||
	    (hw->mac.type == yusur2_mac_X550EM_x) ||
	    (hw->mac.type == yusur2_mac_X550EM_a))
		fdirctrl |= YUSUR2_FDIRCTRL_DROP_NO_MATCH;

	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD,
			(YUSUR2_READ_REG(hw, YUSUR2_FDIRCMD) |
			 YUSUR2_FDIRCMD_CLEARHT));
	YUSUR2_WRITE_FLUSH(hw);
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD,
			(YUSUR2_READ_REG(hw, YUSUR2_FDIRCMD) &
			 ~YUSUR2_FDIRCMD_CLEARHT));
	YUSUR2_WRITE_FLUSH(hw);

	/* write hashes and fdirctrl register, poll for completion */
	yusur2_fdir_enable_82599(hw, fdirctrl);
}

/*
 * These defines allow us to quickly generate all of the necessary instructions
 * in the function below by simply calling out YUSUR2_COMPUTE_SIG_HASH_ITERATION
 * for values 0 through 15
 */
#define YUSUR2_ATR_COMMON_HASH_KEY \
		(YUSUR2_ATR_BUCKET_HASH_KEY & YUSUR2_ATR_SIGNATURE_HASH_KEY)
#define YUSUR2_COMPUTE_SIG_HASH_ITERATION(_n) \
do { \
	u32 n = (_n); \
	if (YUSUR2_ATR_COMMON_HASH_KEY & (0x01 << n)) \
		common_hash ^= lo_hash_dword >> n; \
	else if (YUSUR2_ATR_BUCKET_HASH_KEY & (0x01 << n)) \
		bucket_hash ^= lo_hash_dword >> n; \
	else if (YUSUR2_ATR_SIGNATURE_HASH_KEY & (0x01 << n)) \
		sig_hash ^= lo_hash_dword << (16 - n); \
	if (YUSUR2_ATR_COMMON_HASH_KEY & (0x01 << (n + 16))) \
		common_hash ^= hi_hash_dword >> n; \
	else if (YUSUR2_ATR_BUCKET_HASH_KEY & (0x01 << (n + 16))) \
		bucket_hash ^= hi_hash_dword >> n; \
	else if (YUSUR2_ATR_SIGNATURE_HASH_KEY & (0x01 << (n + 16))) \
		sig_hash ^= hi_hash_dword << (16 - n); \
} while (0)

/**
 *  yusur2_atr_compute_sig_hash_82599 - Compute the signature hash
 *  @input: input bitstream to compute the hash on
 *  @common: compressed common input dword
 *
 *  This function is almost identical to the function above but contains
 *  several optimizations such as unwinding all of the loops, letting the
 *  compiler work out all of the conditional ifs since the keys are static
 *  defines, and computing two keys at once since the hashed dword stream
 *  will be the same for both keys.
 **/
u32 yusur2_atr_compute_sig_hash_82599(union yusur2_atr_hash_dword input,
				     union yusur2_atr_hash_dword common)
{
	u32 hi_hash_dword, lo_hash_dword, flow_vm_vlan;
	u32 sig_hash = 0, bucket_hash = 0, common_hash = 0;

	/* record the flow_vm_vlan bits as they are a key part to the hash */
	flow_vm_vlan = YUSUR2_NTOHL(input.dword);

	/* generate common hash dword */
	hi_hash_dword = YUSUR2_NTOHL(common.dword);

	/* low dword is word swapped version of common */
	lo_hash_dword = (hi_hash_dword >> 16) | (hi_hash_dword << 16);

	/* apply flow ID/VM pool/VLAN ID bits to hash words */
	hi_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan >> 16);

	/* Process bits 0 and 16 */
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the VLAN until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(1);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(2);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(3);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(4);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(5);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(6);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(7);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(8);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(9);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(10);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(11);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(12);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(13);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(14);
	YUSUR2_COMPUTE_SIG_HASH_ITERATION(15);

	/* combine common_hash result with signature and bucket hashes */
	bucket_hash ^= common_hash;
	bucket_hash &= YUSUR2_ATR_HASH_MASK;

	sig_hash ^= common_hash << 16;
	sig_hash &= YUSUR2_ATR_HASH_MASK << 16;

	/* return completed signature hash */
	return sig_hash ^ bucket_hash;
}

/**
 *  yusur2_atr_add_signature_filter_82599 - Adds a signature hash filter
 *  @hw: pointer to hardware structure
 *  @input: unique input dword
 *  @common: compressed common input dword
 *  @queue: queue index to direct traffic to
 *
 * Note that the tunnel bit in input must not be set when the hardware
 * tunneling support does not exist.
 **/
void yusur2_fdir_add_signature_filter_82599(struct yusur2_hw *hw,
					   union yusur2_atr_hash_dword input,
					   union yusur2_atr_hash_dword common,
					   u8 queue)
{
	u64 fdirhashcmd;
	u8 flow_type;
	bool tunnel;
	u32 fdircmd;

	DEBUGFUNC("yusur2_fdir_add_signature_filter_82599");

	/*
	 * Get the flow_type in order to program FDIRCMD properly
	 * lowest 2 bits are FDIRCMD.L4TYPE, third lowest bit is FDIRCMD.IPV6
	 * fifth is FDIRCMD.TUNNEL_FILTER
	 */
	tunnel = !!(input.formatted.flow_type & YUSUR2_ATR_L4TYPE_TUNNEL_MASK);
	flow_type = input.formatted.flow_type &
		    (YUSUR2_ATR_L4TYPE_TUNNEL_MASK - 1);
	switch (flow_type) {
	case YUSUR2_ATR_FLOW_TYPE_TCPV4:
	case YUSUR2_ATR_FLOW_TYPE_UDPV4:
	case YUSUR2_ATR_FLOW_TYPE_SCTPV4:
	case YUSUR2_ATR_FLOW_TYPE_TCPV6:
	case YUSUR2_ATR_FLOW_TYPE_UDPV6:
	case YUSUR2_ATR_FLOW_TYPE_SCTPV6:
		break;
	default:
		DEBUGOUT(" Error on flow type input\n");
		return;
	}

	/* configure FDIRCMD register */
	fdircmd = YUSUR2_FDIRCMD_CMD_ADD_FLOW | YUSUR2_FDIRCMD_FILTER_UPDATE |
		  YUSUR2_FDIRCMD_LAST | YUSUR2_FDIRCMD_QUEUE_EN;
	fdircmd |= (u32)flow_type << YUSUR2_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << YUSUR2_FDIRCMD_RX_QUEUE_SHIFT;
	if (tunnel)
		fdircmd |= YUSUR2_FDIRCMD_TUNNEL_FILTER;

	/*
	 * The lower 32-bits of fdirhashcmd is for FDIRHASH, the upper 32-bits
	 * is for FDIRCMD.  Then do a 64-bit register write from FDIRHASH.
	 */
	fdirhashcmd = (u64)fdircmd << 32;
	fdirhashcmd |= yusur2_atr_compute_sig_hash_82599(input, common);
	YUSUR2_WRITE_REG64(hw, YUSUR2_FDIRHASH, fdirhashcmd);

	DEBUGOUT2("Tx Queue=%x hash=%x\n", queue, (u32)fdirhashcmd);

	return;
}

#define YUSUR2_COMPUTE_BKT_HASH_ITERATION(_n) \
do { \
	u32 n = (_n); \
	if (YUSUR2_ATR_BUCKET_HASH_KEY & (0x01 << n)) \
		bucket_hash ^= lo_hash_dword >> n; \
	if (YUSUR2_ATR_BUCKET_HASH_KEY & (0x01 << (n + 16))) \
		bucket_hash ^= hi_hash_dword >> n; \
} while (0)

/**
 *  yusur2_atr_compute_perfect_hash_82599 - Compute the perfect filter hash
 *  @input: input bitstream to compute the hash on
 *  @input_mask: mask for the input bitstream
 *
 *  This function serves two main purposes.  First it applies the input_mask
 *  to the atr_input resulting in a cleaned up atr_input data stream.
 *  Secondly it computes the hash and stores it in the bkt_hash field at
 *  the end of the input byte stream.  This way it will be available for
 *  future use without needing to recompute the hash.
 **/
void yusur2_atr_compute_perfect_hash_82599(union yusur2_atr_input *input,
					  union yusur2_atr_input *input_mask)
{

	u32 hi_hash_dword, lo_hash_dword, flow_vm_vlan;
	u32 bucket_hash = 0;
	u32 hi_dword = 0;
	u32 i = 0;

	/* Apply masks to input data */
	for (i = 0; i < 14; i++)
		input->dword_stream[i]  &= input_mask->dword_stream[i];

	/* record the flow_vm_vlan bits as they are a key part to the hash */
	flow_vm_vlan = YUSUR2_NTOHL(input->dword_stream[0]);

	/* generate common hash dword */
	for (i = 1; i <= 13; i++)
		hi_dword ^= input->dword_stream[i];
	hi_hash_dword = YUSUR2_NTOHL(hi_dword);

	/* low dword is word swapped version of common */
	lo_hash_dword = (hi_hash_dword >> 16) | (hi_hash_dword << 16);

	/* apply flow ID/VM pool/VLAN ID bits to hash words */
	hi_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan >> 16);

	/* Process bits 0 and 16 */
	YUSUR2_COMPUTE_BKT_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the VLAN until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	for (i = 1; i <= 15; i++)
		YUSUR2_COMPUTE_BKT_HASH_ITERATION(i);

	/*
	 * Limit hash to 13 bits since max bucket count is 8K.
	 * Store result at the end of the input stream.
	 */
	input->formatted.bkt_hash = bucket_hash & 0x1FFF;
}

/**
 *  yusur2_get_fdirtcpm_82599 - generate a TCP port from atr_input_masks
 *  @input_mask: mask to be bit swapped
 *
 *  The source and destination port masks for flow director are bit swapped
 *  in that bit 15 effects bit 0, 14 effects 1, 13, 2 etc.  In order to
 *  generate a correctly swapped value we need to bit swap the mask and that
 *  is what is accomplished by this function.
 **/
STATIC u32 yusur2_get_fdirtcpm_82599(union yusur2_atr_input *input_mask)
{
	u32 mask = YUSUR2_NTOHS(input_mask->formatted.dst_port);
	mask <<= YUSUR2_FDIRTCPM_DPORTM_SHIFT;
	mask |= YUSUR2_NTOHS(input_mask->formatted.src_port);
	mask = ((mask & 0x55555555) << 1) | ((mask & 0xAAAAAAAA) >> 1);
	mask = ((mask & 0x33333333) << 2) | ((mask & 0xCCCCCCCC) >> 2);
	mask = ((mask & 0x0F0F0F0F) << 4) | ((mask & 0xF0F0F0F0) >> 4);
	return ((mask & 0x00FF00FF) << 8) | ((mask & 0xFF00FF00) >> 8);
}

/*
 * These two macros are meant to address the fact that we have registers
 * that are either all or in part big-endian.  As a result on big-endian
 * systems we will end up byte swapping the value to little-endian before
 * it is byte swapped again and written to the hardware in the original
 * big-endian format.
 */
#define YUSUR2_STORE_AS_BE32(_value) \
	(((u32)(_value) >> 24) | (((u32)(_value) & 0x00FF0000) >> 8) | \
	 (((u32)(_value) & 0x0000FF00) << 8) | ((u32)(_value) << 24))

#define YUSUR2_WRITE_REG_BE32(a, reg, value) \
	YUSUR2_WRITE_REG((a), (reg), YUSUR2_STORE_AS_BE32(YUSUR2_NTOHL(value)))

#define YUSUR2_STORE_AS_BE16(_value) \
	YUSUR2_NTOHS(((u16)(_value) >> 8) | ((u16)(_value) << 8))

s32 yusur2_fdir_set_input_mask_82599(struct yusur2_hw *hw,
				    union yusur2_atr_input *input_mask, bool cloud_mode)
{
	/* mask IPv6 since it is currently not supported */
	u32 fdirm = YUSUR2_FDIRM_DIPv6;
	u32 fdirtcpm;
	u32 fdirip6m;
	UNREFERENCED_1PARAMETER(cloud_mode);
	DEBUGFUNC("yusur2_fdir_set_atr_input_mask_82599");

	/*
	 * Program the relevant mask registers.  If src/dst_port or src/dst_addr
	 * are zero, then assume a full mask for that field.  Also assume that
	 * a VLAN of 0 is unspecified, so mask that out as well.  L4type
	 * cannot be masked out in this implementation.
	 *
	 * This also assumes IPv4 only.  IPv6 masking isn't supported at this
	 * point in time.
	 */

	/* verify bucket hash is cleared on hash generation */
	if (input_mask->formatted.bkt_hash)
		DEBUGOUT(" bucket hash should always be 0 in mask\n");

	/* Program FDIRM and verify partial masks */
	switch (input_mask->formatted.vm_pool & 0x7F) {
	case 0x0:
		fdirm |= YUSUR2_FDIRM_POOL;
	case 0x7F:
		break;
	default:
		DEBUGOUT(" Error on vm pool mask\n");
		return YUSUR2_ERR_CONFIG;
	}

	switch (input_mask->formatted.flow_type & YUSUR2_ATR_L4TYPE_MASK) {
	case 0x0:
		fdirm |= YUSUR2_FDIRM_L4P;
		if (input_mask->formatted.dst_port ||
		    input_mask->formatted.src_port) {
			DEBUGOUT(" Error on src/dst port mask\n");
			return YUSUR2_ERR_CONFIG;
		}
	case YUSUR2_ATR_L4TYPE_MASK:
		break;
	default:
		DEBUGOUT(" Error on flow type mask\n");
		return YUSUR2_ERR_CONFIG;
	}

	switch (YUSUR2_NTOHS(input_mask->formatted.vlan_id) & 0xEFFF) {
	case 0x0000:
		/* mask VLAN ID */
		fdirm |= YUSUR2_FDIRM_VLANID;
		/* fall through */
	case 0x0FFF:
		/* mask VLAN priority */
		fdirm |= YUSUR2_FDIRM_VLANP;
		break;
	case 0xE000:
		/* mask VLAN ID only */
		fdirm |= YUSUR2_FDIRM_VLANID;
		/* fall through */
	case 0xEFFF:
		/* no VLAN fields masked */
		break;
	default:
		DEBUGOUT(" Error on VLAN mask\n");
		return YUSUR2_ERR_CONFIG;
	}

	switch (input_mask->formatted.flex_bytes & 0xFFFF) {
	case 0x0000:
		/* Mask Flex Bytes */
		fdirm |= YUSUR2_FDIRM_FLEX;
		/* fall through */
	case 0xFFFF:
		break;
	default:
		DEBUGOUT(" Error on flexible byte mask\n");
		return YUSUR2_ERR_CONFIG;
	}

	if (cloud_mode) {
		fdirm |= YUSUR2_FDIRM_L3P;
		fdirip6m = ((u32) 0xFFFFU << YUSUR2_FDIRIP6M_DIPM_SHIFT);
		fdirip6m |= YUSUR2_FDIRIP6M_ALWAYS_MASK;

		switch (input_mask->formatted.inner_mac[0] & 0xFF) {
		case 0x00:
			/* Mask inner MAC, fall through */
			fdirip6m |= YUSUR2_FDIRIP6M_INNER_MAC;
		case 0xFF:
			break;
		default:
			DEBUGOUT(" Error on inner_mac byte mask\n");
			return YUSUR2_ERR_CONFIG;
		}

		switch (input_mask->formatted.tni_vni & 0xFFFFFFFF) {
		case 0x0:
			/* Mask vxlan id */
			fdirip6m |= YUSUR2_FDIRIP6M_TNI_VNI;
			break;
		case 0x00FFFFFF:
			fdirip6m |= YUSUR2_FDIRIP6M_TNI_VNI_24;
			break;
		case 0xFFFFFFFF:
			break;
		default:
			DEBUGOUT(" Error on TNI/VNI byte mask\n");
			return YUSUR2_ERR_CONFIG;
		}

		switch (input_mask->formatted.tunnel_type & 0xFFFF) {
		case 0x0:
			/* Mask turnnel type, fall through */
			fdirip6m |= YUSUR2_FDIRIP6M_TUNNEL_TYPE;
		case 0xFFFF:
			break;
		default:
			DEBUGOUT(" Error on tunnel type byte mask\n");
			return YUSUR2_ERR_CONFIG;
		}
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRIP6M, fdirip6m);

		/* Set all bits in FDIRTCPM, FDIRUDPM, FDIRSCTPM,
		 * FDIRSIP4M and FDIRDIP4M in cloud mode to allow
		 * L3/L3 packets to tunnel.
		 */
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRTCPM, 0xFFFFFFFF);
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRUDPM, 0xFFFFFFFF);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRDIP4M, 0xFFFFFFFF);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIP4M, 0xFFFFFFFF);
		switch (hw->mac.type) {
		case yusur2_mac_X550:
		case yusur2_mac_X550EM_x:
		case yusur2_mac_X550EM_a:
			YUSUR2_WRITE_REG(hw, YUSUR2_FDIRSCTPM, 0xFFFFFFFF);
			break;
		default:
			break;
		}
	}

	/* Now mask VM pool and destination IPv6 - bits 5 and 2 */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRM, fdirm);

	if (!cloud_mode) {
		/* store the TCP/UDP port masks, bit reversed from port
		 * layout */
		fdirtcpm = yusur2_get_fdirtcpm_82599(input_mask);

		/* write both the same so that UDP and TCP use the same mask */
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRTCPM, ~fdirtcpm);
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRUDPM, ~fdirtcpm);
		/* also use it for SCTP */
		switch (hw->mac.type) {
		case yusur2_mac_X550:
		case yusur2_mac_X550EM_x:
		case yusur2_mac_X550EM_a:
			YUSUR2_WRITE_REG(hw, YUSUR2_FDIRSCTPM, ~fdirtcpm);
			break;
		default:
			break;
		}

		/* store source and destination IP masks (big-enian) */
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIP4M,
				     ~input_mask->formatted.src_ip[0]);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRDIP4M,
				     ~input_mask->formatted.dst_ip[0]);
	}
	return YUSUR2_SUCCESS;
}

s32 yusur2_fdir_write_perfect_filter_82599(struct yusur2_hw *hw,
					  union yusur2_atr_input *input,
					  u16 soft_id, u8 queue, bool cloud_mode)
{
	u32 fdirport, fdirvlan, fdirhash, fdircmd;
	u32 addr_low, addr_high;
	u32 cloud_type = 0;
	s32 err;
	UNREFERENCED_1PARAMETER(cloud_mode);

	DEBUGFUNC("yusur2_fdir_write_perfect_filter_82599");
	if (!cloud_mode) {
		/* currently IPv6 is not supported, must be programmed with 0 */
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(0),
				     input->formatted.src_ip[0]);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(1),
				     input->formatted.src_ip[1]);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(2),
				     input->formatted.src_ip[2]);

		/* record the source address (big-endian) */
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRIPSA,
			input->formatted.src_ip[0]);

		/* record the first 32 bits of the destination address
		 * (big-endian) */
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRIPDA,
			input->formatted.dst_ip[0]);

		/* record source and destination port (little-endian)*/
		fdirport = YUSUR2_NTOHS(input->formatted.dst_port);
		fdirport <<= YUSUR2_FDIRPORT_DESTINATION_SHIFT;
		fdirport |= YUSUR2_NTOHS(input->formatted.src_port);
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRPORT, fdirport);
	}

	/* record VLAN (little-endian) and flex_bytes(big-endian) */
	fdirvlan = YUSUR2_STORE_AS_BE16(input->formatted.flex_bytes);
	fdirvlan <<= YUSUR2_FDIRVLAN_FLEX_SHIFT;
	fdirvlan |= YUSUR2_NTOHS(input->formatted.vlan_id);
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRVLAN, fdirvlan);

	if (cloud_mode) {
		if (input->formatted.tunnel_type != 0)
			cloud_type = 0x80000000;

		addr_low = ((u32)input->formatted.inner_mac[0] |
				((u32)input->formatted.inner_mac[1] << 8) |
				((u32)input->formatted.inner_mac[2] << 16) |
				((u32)input->formatted.inner_mac[3] << 24));
		addr_high = ((u32)input->formatted.inner_mac[4] |
				((u32)input->formatted.inner_mac[5] << 8));
		cloud_type |= addr_high;
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(0), addr_low);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(1), cloud_type);
		YUSUR2_WRITE_REG_BE32(hw, YUSUR2_FDIRSIPv6(2), input->formatted.tni_vni);
	}

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << YUSUR2_FDIRHASH_SIG_SW_INDEX_SHIFT;
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRHASH, fdirhash);

	/*
	 * flush all previous writes to make certain registers are
	 * programmed prior to issuing the command
	 */
	YUSUR2_WRITE_FLUSH(hw);

	/* configure FDIRCMD register */
	fdircmd = YUSUR2_FDIRCMD_CMD_ADD_FLOW | YUSUR2_FDIRCMD_FILTER_UPDATE |
		  YUSUR2_FDIRCMD_LAST | YUSUR2_FDIRCMD_QUEUE_EN;
	if (queue == YUSUR2_FDIR_DROP_QUEUE)
		fdircmd |= YUSUR2_FDIRCMD_DROP;
	if (input->formatted.flow_type & YUSUR2_ATR_L4TYPE_TUNNEL_MASK)
		fdircmd |= YUSUR2_FDIRCMD_TUNNEL_FILTER;
	fdircmd |= input->formatted.flow_type << YUSUR2_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << YUSUR2_FDIRCMD_RX_QUEUE_SHIFT;
	fdircmd |= (u32)input->formatted.vm_pool << YUSUR2_FDIRCMD_VT_POOL_SHIFT;

	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD, fdircmd);
	err = yusur2_fdir_check_cmd_complete(hw, &fdircmd);
	if (err) {
		DEBUGOUT("Flow Director command did not complete!\n");
		return err;
	}

	return YUSUR2_SUCCESS;
}

s32 yusur2_fdir_erase_perfect_filter_82599(struct yusur2_hw *hw,
					  union yusur2_atr_input *input,
					  u16 soft_id)
{
	u32 fdirhash;
	u32 fdircmd;
	s32 err;

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << YUSUR2_FDIRHASH_SIG_SW_INDEX_SHIFT;
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRHASH, fdirhash);

	/* flush hash to HW */
	YUSUR2_WRITE_FLUSH(hw);

	/* Query if filter is present */
	YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD, YUSUR2_FDIRCMD_CMD_QUERY_REM_FILT);

	err = yusur2_fdir_check_cmd_complete(hw, &fdircmd);
	if (err) {
		DEBUGOUT("Flow Director command did not complete!\n");
		return err;
	}

	/* if filter exists in hardware then remove it */
	if (fdircmd & YUSUR2_FDIRCMD_FILTER_VALID) {
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRHASH, fdirhash);
		YUSUR2_WRITE_FLUSH(hw);
		YUSUR2_WRITE_REG(hw, YUSUR2_FDIRCMD,
				YUSUR2_FDIRCMD_CMD_REMOVE_FLOW);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_fdir_add_perfect_filter_82599 - Adds a perfect filter
 *  @hw: pointer to hardware structure
 *  @input: input bitstream
 *  @input_mask: mask for the input bitstream
 *  @soft_id: software index for the filters
 *  @queue: queue index to direct traffic to
 *  @cloud_mode: unused
 *
 *  Note that the caller to this function must lock before calling, since the
 *  hardware writes must be protected from one another.
 **/
s32 yusur2_fdir_add_perfect_filter_82599(struct yusur2_hw *hw,
					union yusur2_atr_input *input,
					union yusur2_atr_input *input_mask,
					u16 soft_id, u8 queue, bool cloud_mode)
{
	s32 err = YUSUR2_ERR_CONFIG;
	UNREFERENCED_1PARAMETER(cloud_mode);

	DEBUGFUNC("yusur2_fdir_add_perfect_filter_82599");

	/*
	 * Check flow_type formatting, and bail out before we touch the hardware
	 * if there's a configuration issue
	 */
	switch (input->formatted.flow_type) {
	case YUSUR2_ATR_FLOW_TYPE_IPV4:
	case YUSUR2_ATR_FLOW_TYPE_TUNNELED_IPV4:
		input_mask->formatted.flow_type = YUSUR2_ATR_L4TYPE_IPV6_MASK;
		if (input->formatted.dst_port || input->formatted.src_port) {
			DEBUGOUT(" Error on src/dst port\n");
			return YUSUR2_ERR_CONFIG;
		}
		break;
	case YUSUR2_ATR_FLOW_TYPE_SCTPV4:
	case YUSUR2_ATR_FLOW_TYPE_TUNNELED_SCTPV4:
		if (input->formatted.dst_port || input->formatted.src_port) {
			DEBUGOUT(" Error on src/dst port\n");
			return YUSUR2_ERR_CONFIG;
		}
		/* fall through */
	case YUSUR2_ATR_FLOW_TYPE_TCPV4:
	case YUSUR2_ATR_FLOW_TYPE_TUNNELED_TCPV4:
	case YUSUR2_ATR_FLOW_TYPE_UDPV4:
	case YUSUR2_ATR_FLOW_TYPE_TUNNELED_UDPV4:
		input_mask->formatted.flow_type = YUSUR2_ATR_L4TYPE_IPV6_MASK |
						  YUSUR2_ATR_L4TYPE_MASK;
		break;
	default:
		DEBUGOUT(" Error on flow type input\n");
		return err;
	}

	/* program input mask into the HW */
	err = yusur2_fdir_set_input_mask_82599(hw, input_mask, cloud_mode);
	if (err)
		return err;

	/* apply mask and compute/store hash */
	yusur2_atr_compute_perfect_hash_82599(input, input_mask);

	/* program filters to filter memory */
	return yusur2_fdir_write_perfect_filter_82599(hw, input,
						     soft_id, queue, cloud_mode);
}

/**
 *  yusur2_read_analog_reg8_82599 - Reads 8 bit Omer analog register
 *  @hw: pointer to hardware structure
 *  @reg: analog register to read
 *  @val: read value
 *
 *  Performs read operation to Omer analog register specified.
 **/
s32 yusur2_read_analog_reg8_82599(struct yusur2_hw *hw, u32 reg, u8 *val)
{
	u32  core_ctl;

	DEBUGFUNC("yusur2_read_analog_reg8_82599");

	YUSUR2_WRITE_REG(hw, YUSUR2_CORECTL, YUSUR2_CORECTL_WRITE_CMD |
			(reg << 8));
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(10);
	core_ctl = YUSUR2_READ_REG(hw, YUSUR2_CORECTL);
	*val = (u8)core_ctl;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_write_analog_reg8_82599 - Writes 8 bit Omer analog register
 *  @hw: pointer to hardware structure
 *  @reg: atlas register to write
 *  @val: value to write
 *
 *  Performs write operation to Omer analog register specified.
 **/
s32 yusur2_write_analog_reg8_82599(struct yusur2_hw *hw, u32 reg, u8 val)
{
	u32  core_ctl;

	DEBUGFUNC("yusur2_write_analog_reg8_82599");

	core_ctl = (reg << 8) | val;
	YUSUR2_WRITE_REG(hw, YUSUR2_CORECTL, core_ctl);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(10);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_start_hw_82599 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
s32 yusur2_start_hw_82599(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_start_hw_82599");

	ret_val = yusur2_start_hw_generic(hw);
	if (ret_val != YUSUR2_SUCCESS)
		goto out;

	ret_val = yusur2_start_hw_gen2(hw);
	if (ret_val != YUSUR2_SUCCESS)
		goto out;

	/* We need to run link autotry after the driver loads */
	hw->mac.autotry_restart = true;

	if (ret_val == YUSUR2_SUCCESS)
		ret_val = yusur2_verify_fw_version_82599(hw);
out:
	return ret_val;
}

/**
 *  yusur2_identify_phy_82599 - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 *  If PHY already detected, maintains current PHY type in hw struct,
 *  otherwise executes the PHY detection routine.
 **/
s32 yusur2_identify_phy_82599(struct yusur2_hw *hw)
{
	s32 status;

	DEBUGFUNC("yusur2_identify_phy_82599");

	/* Detect PHY if not unknown - returns success if already detected. */
	status = yusur2_identify_phy_generic(hw);
	if (status != YUSUR2_SUCCESS) {
		/* 82599 10GBASE-T requires an external PHY */
		if (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper)
			return status;
		else
			status = yusur2_identify_module_generic(hw);
	}

	/* Set PHY type none if no PHY detected */
	if (hw->phy.type == yusur2_phy_unknown) {
		hw->phy.type = yusur2_phy_none;
		return YUSUR2_SUCCESS;
	}

	/* Return error if SFP module has been detected but is not supported */
	if (hw->phy.type == yusur2_phy_sfp_unsupported)
		return YUSUR2_ERR_SFP_NOT_SUPPORTED;

	return status;
}

/**
 *  yusur2_get_supported_physical_layer_82599 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
u64 yusur2_get_supported_physical_layer_82599(struct yusur2_hw *hw)
{
	u64 physical_layer = YUSUR2_PHYSICAL_LAYER_UNKNOWN;
	u32 autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	u32 autoc2 = YUSUR2_READ_REG(hw, YUSUR2_AUTOC2);
	u32 pma_pmd_10g_serial = autoc2 & YUSUR2_AUTOC2_10G_SERIAL_PMA_PMD_MASK;
	u32 pma_pmd_10g_parallel = autoc & YUSUR2_AUTOC_10G_PMA_PMD_MASK;
	u32 pma_pmd_1g = autoc & YUSUR2_AUTOC_1G_PMA_PMD_MASK;
	u16 ext_ability = 0;

	DEBUGFUNC("yusur2_get_support_physical_layer_82599");

	hw->phy.ops.identify(hw);

	switch (hw->phy.type) {
	case yusur2_phy_tn:
	case yusur2_phy_cu_unknown:
		hw->phy.ops.read_reg(hw, YUSUR2_MDIO_PHY_EXT_ABILITY,
		YUSUR2_MDIO_PMA_PMD_DEV_TYPE, &ext_ability);
		if (ext_ability & YUSUR2_MDIO_PHY_10GBASET_ABILITY)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_10GBASE_T;
		if (ext_ability & YUSUR2_MDIO_PHY_1000BASET_ABILITY)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_1000BASE_T;
		if (ext_ability & YUSUR2_MDIO_PHY_100BASETX_ABILITY)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_100BASE_TX;
		goto out;
	default:
		break;
	}

	switch (autoc & YUSUR2_AUTOC_LMS_MASK) {
	case YUSUR2_AUTOC_LMS_1G_AN:
	case YUSUR2_AUTOC_LMS_1G_LINK_NO_AN:
		if (pma_pmd_1g == YUSUR2_AUTOC_1G_KX_BX) {
			physical_layer = YUSUR2_PHYSICAL_LAYER_1000BASE_KX |
			    YUSUR2_PHYSICAL_LAYER_1000BASE_BX;
			goto out;
		} else
			/* SFI mode so read SFP module */
			goto sfp_check;
		break;
	case YUSUR2_AUTOC_LMS_10G_LINK_NO_AN:
		if (pma_pmd_10g_parallel == YUSUR2_AUTOC_10G_CX4)
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_CX4;
		else if (pma_pmd_10g_parallel == YUSUR2_AUTOC_10G_KX4)
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_KX4;
		else if (pma_pmd_10g_parallel == YUSUR2_AUTOC_10G_XAUI)
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_XAUI;
		goto out;
		break;
	case YUSUR2_AUTOC_LMS_10G_SERIAL:
		if (pma_pmd_10g_serial == YUSUR2_AUTOC2_10G_KR) {
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_KR;
			goto out;
		} else if (pma_pmd_10g_serial == YUSUR2_AUTOC2_10G_SFI)
			goto sfp_check;
		break;
	case YUSUR2_AUTOC_LMS_KX4_KX_KR:
	case YUSUR2_AUTOC_LMS_KX4_KX_KR_1G_AN:
		if (autoc & YUSUR2_AUTOC_KX_SUPP)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_1000BASE_KX;
		if (autoc & YUSUR2_AUTOC_KX4_SUPP)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_10GBASE_KX4;
		if (autoc & YUSUR2_AUTOC_KR_SUPP)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_10GBASE_KR;
		goto out;
		break;
	default:
		goto out;
		break;
	}

sfp_check:
	/* SFP check must be done last since DA modules are sometimes used to
	 * test KR mode -  we need to id KR mode correctly before SFP module.
	 * Call identify_sfp because the pluggable module may have changed */
	physical_layer = yusur2_get_supported_phy_sfp_layer_generic(hw);
out:
	return physical_layer;
}

/**
 *  yusur2_enable_rx_dma_82599 - Enable the Rx DMA unit on 82599
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit for 82599
 **/
s32 yusur2_enable_rx_dma_82599(struct yusur2_hw *hw, u32 regval)
{

	DEBUGFUNC("yusur2_enable_rx_dma_82599");

	/*
	 * Workaround for 82599 silicon errata when enabling the Rx datapath.
	 * If traffic is incoming before we enable the Rx unit, it could hang
	 * the Rx DMA unit.  Therefore, make sure the security engine is
	 * completely disabled prior to enabling the Rx unit.
	 */

	hw->mac.ops.disable_sec_rx_path(hw);

	if (regval & YUSUR2_RXCTRL_RXEN)
		yusur2_enable_rx(hw);
	else
		yusur2_disable_rx(hw);

	hw->mac.ops.enable_sec_rx_path(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_verify_fw_version_82599 - verify FW version for 82599
 *  @hw: pointer to hardware structure
 *
 *  Verifies that installed the firmware version is 0.6 or higher
 *  for SFI devices. All 82599 SFI devices should have version 0.6 or higher.
 *
 *  Returns YUSUR2_ERR_EEPROM_VERSION if the FW is not present or
 *  if the FW version is not supported.
 **/
STATIC s32 yusur2_verify_fw_version_82599(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_ERR_EEPROM_VERSION;
	u16 fw_offset, fw_ptp_cfg_offset;
	u16 fw_version;

	DEBUGFUNC("yusur2_verify_fw_version_82599");

	/* firmware check is only necessary for SFI devices */
	if (hw->phy.media_type != yusur2_media_type_fiber) {
		status = YUSUR2_SUCCESS;
		goto fw_version_out;
	}

	/* get the offset to the Firmware Module block */
	if (hw->eeprom.ops.read(hw, YUSUR2_FW_PTR, &fw_offset)) {
		ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
			      "eeprom read at offset %d failed", YUSUR2_FW_PTR);
		return YUSUR2_ERR_EEPROM_VERSION;
	}

	if ((fw_offset == 0) || (fw_offset == 0xFFFF))
		goto fw_version_out;

	/* get the offset to the Pass Through Patch Configuration block */
	if (hw->eeprom.ops.read(hw, (fw_offset +
				 YUSUR2_FW_PASSTHROUGH_PATCH_CONFIG_PTR),
				 &fw_ptp_cfg_offset)) {
		ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
			      "eeprom read at offset %d failed",
			      fw_offset +
			      YUSUR2_FW_PASSTHROUGH_PATCH_CONFIG_PTR);
		return YUSUR2_ERR_EEPROM_VERSION;
	}

	if ((fw_ptp_cfg_offset == 0) || (fw_ptp_cfg_offset == 0xFFFF))
		goto fw_version_out;

	/* get the firmware version */
	if (hw->eeprom.ops.read(hw, (fw_ptp_cfg_offset +
			    YUSUR2_FW_PATCH_VERSION_4), &fw_version)) {
		ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
			      "eeprom read at offset %d failed",
			      fw_ptp_cfg_offset + YUSUR2_FW_PATCH_VERSION_4);
		return YUSUR2_ERR_EEPROM_VERSION;
	}

	if (fw_version > 0x5)
		status = YUSUR2_SUCCESS;

fw_version_out:
	return status;
}

/**
 *  yusur2_verify_lesm_fw_enabled_82599 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool yusur2_verify_lesm_fw_enabled_82599(struct yusur2_hw *hw)
{
	bool lesm_enabled = false;
	u16 fw_offset, fw_lesm_param_offset, fw_lesm_state;
	s32 status;

	DEBUGFUNC("yusur2_verify_lesm_fw_enabled_82599");

	/* get the offset to the Firmware Module block */
	status = hw->eeprom.ops.read(hw, YUSUR2_FW_PTR, &fw_offset);

	if ((status != YUSUR2_SUCCESS) ||
	    (fw_offset == 0) || (fw_offset == 0xFFFF))
		goto out;

	/* get the offset to the LESM Parameters block */
	status = hw->eeprom.ops.read(hw, (fw_offset +
				     YUSUR2_FW_LESM_PARAMETERS_PTR),
				     &fw_lesm_param_offset);

	if ((status != YUSUR2_SUCCESS) ||
	    (fw_lesm_param_offset == 0) || (fw_lesm_param_offset == 0xFFFF))
		goto out;

	/* get the LESM state word */
	status = hw->eeprom.ops.read(hw, (fw_lesm_param_offset +
				     YUSUR2_FW_LESM_STATE_1),
				     &fw_lesm_state);

	if ((status == YUSUR2_SUCCESS) &&
	    (fw_lesm_state & YUSUR2_FW_LESM_STATE_ENABLED))
		lesm_enabled = true;

out:
	return lesm_enabled;
}

/**
 *  yusur2_read_eeprom_buffer_82599 - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
STATIC s32 yusur2_read_eeprom_buffer_82599(struct yusur2_hw *hw, u16 offset,
					  u16 words, u16 *data)
{
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val = YUSUR2_ERR_CONFIG;

	DEBUGFUNC("yusur2_read_eeprom_buffer_82599");

	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == yusur2_eeprom_spi) &&
	    (offset + (words - 1) <= YUSUR2_EERD_MAX_ADDR))
		ret_val = yusur2_read_eerd_buffer_generic(hw, offset, words,
							 data);
	else
		ret_val = yusur2_read_eeprom_buffer_bit_bang_generic(hw, offset,
								    words,
								    data);

	return ret_val;
}

/**
 *  yusur2_read_eeprom_82599 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
STATIC s32 yusur2_read_eeprom_82599(struct yusur2_hw *hw,
				   u16 offset, u16 *data)
{
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val = YUSUR2_ERR_CONFIG;

	DEBUGFUNC("yusur2_read_eeprom_82599");

	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == yusur2_eeprom_spi) &&
	    (offset <= YUSUR2_EERD_MAX_ADDR))
		ret_val = yusur2_read_eerd_generic(hw, offset, data);
	else
		ret_val = yusur2_read_eeprom_bit_bang_generic(hw, offset, data);

	return ret_val;
}

/**
 * yusur2_reset_pipeline_82599 - perform pipeline reset
 *
 *  @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  This function assumes the SW/FW lock is held.
 **/
s32 yusur2_reset_pipeline_82599(struct yusur2_hw *hw)
{
	s32 ret_val;
	u32 anlp1_reg = 0;
	u32 i, autoc_reg, autoc2_reg;

	/* Enable link if disabled in NVM */
	autoc2_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC2);
	if (autoc2_reg & YUSUR2_AUTOC2_LINK_DISABLE_MASK) {
		autoc2_reg &= ~YUSUR2_AUTOC2_LINK_DISABLE_MASK;
		YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC2, autoc2_reg);
		YUSUR2_WRITE_FLUSH(hw);
	}

	autoc_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	autoc_reg |= YUSUR2_AUTOC_AN_RESTART;
	/* Write AUTOC register with toggled LMS[2] bit and Restart_AN */
	YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC,
			autoc_reg ^ (0x4 << YUSUR2_AUTOC_LMS_SHIFT));
	/* Wait for AN to leave state 0 */
	for (i = 0; i < 10; i++) {
		msec_delay(4);
		anlp1_reg = YUSUR2_READ_REG(hw, YUSUR2_ANLP1);
		if (anlp1_reg & YUSUR2_ANLP1_AN_STATE_MASK)
			break;
	}

	if (!(anlp1_reg & YUSUR2_ANLP1_AN_STATE_MASK)) {
		DEBUGOUT("auto negotiation not completed\n");
		ret_val = YUSUR2_ERR_RESET_FAILED;
		goto reset_pipeline_out;
	}

	ret_val = YUSUR2_SUCCESS;

reset_pipeline_out:
	/* Write AUTOC register with original LMS field and Restart_AN */
	YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, autoc_reg);
	YUSUR2_WRITE_FLUSH(hw);

	return ret_val;
}

/**
 *  yusur2_read_i2c_byte_82599 - Reads 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to read
 *  @dev_addr: address to read from
 *  @data: value read
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
STATIC s32 yusur2_read_i2c_byte_82599(struct yusur2_hw *hw, u8 byte_offset,
				u8 dev_addr, u8 *data)
{
	u32 esdp;
	s32 status;
	s32 timeout = 200;

	DEBUGFUNC("yusur2_read_i2c_byte_82599");

	if (hw->phy.qsfp_shared_i2c_bus == TRUE) {
		/* Acquire I2C bus ownership. */
		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		esdp |= YUSUR2_ESDP_SDP0;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);

		while (timeout) {
			esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
			if (esdp & YUSUR2_ESDP_SDP1)
				break;

			msec_delay(5);
			timeout--;
		}

		if (!timeout) {
			DEBUGOUT("Driver can't access resource,"
				 " acquiring I2C bus timeout.\n");
			status = YUSUR2_ERR_I2C;
			goto release_i2c_access;
		}
	}

	status = yusur2_read_i2c_byte_generic(hw, byte_offset, dev_addr, data);

release_i2c_access:

	if (hw->phy.qsfp_shared_i2c_bus == TRUE) {
		/* Release I2C bus ownership. */
		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		esdp &= ~YUSUR2_ESDP_SDP0;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);
	}

	return status;
}

/**
 *  yusur2_write_i2c_byte_82599 - Writes 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to write
 *  @dev_addr: address to read from
 *  @data: value to write
 *
 *  Performs byte write operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
STATIC s32 yusur2_write_i2c_byte_82599(struct yusur2_hw *hw, u8 byte_offset,
				 u8 dev_addr, u8 data)
{
	u32 esdp;
	s32 status;
	s32 timeout = 200;

	DEBUGFUNC("yusur2_write_i2c_byte_82599");

	if (hw->phy.qsfp_shared_i2c_bus == TRUE) {
		/* Acquire I2C bus ownership. */
		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		esdp |= YUSUR2_ESDP_SDP0;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);

		while (timeout) {
			esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
			if (esdp & YUSUR2_ESDP_SDP1)
				break;

			msec_delay(5);
			timeout--;
		}

		if (!timeout) {
			DEBUGOUT("Driver can't access resource,"
				 " acquiring I2C bus timeout.\n");
			status = YUSUR2_ERR_I2C;
			goto release_i2c_access;
		}
	}

	status = yusur2_write_i2c_byte_generic(hw, byte_offset, dev_addr, data);

release_i2c_access:

	if (hw->phy.qsfp_shared_i2c_bus == TRUE) {
		/* Release I2C bus ownership. */
		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		esdp &= ~YUSUR2_ESDP_SDP0;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);
	}

	return status;
}
