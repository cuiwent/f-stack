/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include "yusur2_type.h"
#include "yusur2_sn2100.h"
#include "yusur2_api.h"
#include "yusur2_common.h"
#include "yusur2_phy.h"

#define YUSUR2_sn2100_MAX_TX_QUEUES 32
#define YUSUR2_sn2100_MAX_RX_QUEUES 64
#define YUSUR2_sn2100_RAR_ENTRIES   16
#define YUSUR2_sn2100_MC_TBL_SIZE  128
#define YUSUR2_sn2100_VFT_TBL_SIZE 128
#define YUSUR2_sn2100_RX_PB_SIZE   512

STATIC s32 yusur2_get_link_capabilities_sn2100(struct yusur2_hw *hw,
					     yusur2_link_speed *speed,
					     bool *autoneg);
STATIC enum yusur2_media_type yusur2_get_media_type_sn2100(struct yusur2_hw *hw);
STATIC s32 yusur2_start_mac_link_sn2100(struct yusur2_hw *hw,
				      bool autoneg_wait_to_complete);
STATIC s32 yusur2_check_mac_link_sn2100(struct yusur2_hw *hw,
				      yusur2_link_speed *speed, bool *link_up,
				      bool link_up_wait_to_complete);
STATIC s32 yusur2_setup_mac_link_sn2100(struct yusur2_hw *hw,
				      yusur2_link_speed speed,
				      bool autoneg_wait_to_complete);
STATIC s32 yusur2_setup_copper_link_sn2100(struct yusur2_hw *hw,
					 yusur2_link_speed speed,
					 bool autoneg_wait_to_complete);
STATIC s32 yusur2_reset_hw_sn2100(struct yusur2_hw *hw);
STATIC s32 yusur2_clear_vmdq_sn2100(struct yusur2_hw *hw, u32 rar, u32 vmdq);
STATIC s32 yusur2_clear_vfta_sn2100(struct yusur2_hw *hw);
STATIC void yusur2_set_rxpba_sn2100(struct yusur2_hw *hw, int num_pb,
				  u32 headroom, int strategy);
STATIC s32 yusur2_read_i2c_sff8472_sn2100(struct yusur2_hw *hw, u8 byte_offset,
					u8 *sff8472_data);
/**
 *  yusur2_set_pcie_completion_timeout - set pci-e completion timeout
 *  @hw: pointer to the HW structure
 *
 *  The defaults for sn2100 should be in the range of 50us to 50ms,
 *  however the hardware default for these parts is 500us to 1ms which is less
 *  than the 10ms recommended by the pci-e spec.  To address this we need to
 *  increase the value to either 10ms to 250ms for capability version 1 config,
 *  or 16ms to 55ms for version 2.
 **/
void yusur2_set_pcie_completion_timeout(struct yusur2_hw *hw)
{
	//TODO: check ...
#if 0
	u32 gcr = YUSUR2_READ_REG(hw, YUSUR2_GCR);
	u16 pcie_devctl2;

	/* only take action if timeout value is defaulted to 0 */
	if (gcr & YUSUR2_GCR_CMPL_TMOUT_MASK)
		goto out;

	/*
	 * if capababilities version is type 1 we can write the
	 * timeout of 10ms to 250ms through the GCR register
	 */
	if (!(gcr & YUSUR2_GCR_CAP_VER2)) {
		gcr |= YUSUR2_GCR_CMPL_TMOUT_10ms;
		goto out;
	}

	/*
	 * for version 2 capabilities we need to write the config space
	 * directly in order to set the completion timeout value for
	 * 16ms to 55ms
	 */
	pcie_devctl2 = YUSUR2_READ_PCIE_WORD(hw, YUSUR2_PCI_DEVICE_CONTROL2);
	pcie_devctl2 |= YUSUR2_PCI_DEVICE_CONTROL2_16ms;
	YUSUR2_WRITE_PCIE_WORD(hw, YUSUR2_PCI_DEVICE_CONTROL2, pcie_devctl2);
out:
	/* disable completion timeout resend */
	gcr &= ~YUSUR2_GCR_CMPL_TMOUT_RESEND;
	YUSUR2_WRITE_REG(hw, YUSUR2_GCR, gcr);
#endif
}

/**
 *  yusur2_init_ops_sn2100 - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for sn2100.
 *  Does not touch the hardware.
 **/
s32 yusur2_init_ops_sn2100(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;
	struct yusur2_phy_info *phy = &hw->phy;
	s32 ret_val;

	DEBUGFUNC("yusur2_init_ops_sn2100");

	ret_val = yusur2_init_phy_ops_generic(hw);
	ret_val = yusur2_init_ops_generic(hw);

	/* PHY */
	phy->ops.init = yusur2_init_phy_ops_sn2100;

	/* MAC */
	mac->ops.start_hw = yusur2_start_hw_sn2100;
	mac->ops.enable_relaxed_ordering = yusur2_enable_relaxed_ordering_sn2100;
	mac->ops.reset_hw = yusur2_reset_hw_sn2100;
	mac->ops.get_media_type = yusur2_get_media_type_sn2100;
	mac->ops.get_supported_physical_layer =
				yusur2_get_supported_physical_layer_sn2100;
	mac->ops.read_analog_reg8 = yusur2_read_analog_reg8_sn2100;
	mac->ops.write_analog_reg8 = yusur2_write_analog_reg8_sn2100;
	mac->ops.set_lan_id = yusur2_set_lan_id_multi_port_pcie_sn2100;
	mac->ops.enable_rx_dma = yusur2_enable_rx_dma_sn2100;

	/* RAR, Multicast, VLAN */
	mac->ops.set_vmdq = yusur2_set_vmdq_sn2100;
	mac->ops.clear_vmdq = yusur2_clear_vmdq_sn2100;
	mac->ops.set_vfta = yusur2_set_vfta_sn2100;
	mac->ops.set_vlvf = NULL;
	mac->ops.clear_vfta = yusur2_clear_vfta_sn2100;

	/* Flow Control */
	mac->ops.fc_enable = yusur2_fc_enable_sn2100;

	mac->mcft_size		= YUSUR2_sn2100_MC_TBL_SIZE;
	mac->vft_size		= YUSUR2_sn2100_VFT_TBL_SIZE;
	mac->num_rar_entries	= YUSUR2_sn2100_RAR_ENTRIES;
	mac->rx_pb_size		= YUSUR2_sn2100_RX_PB_SIZE;
	mac->max_rx_queues	= YUSUR2_sn2100_MAX_RX_QUEUES;
	mac->max_tx_queues	= YUSUR2_sn2100_MAX_TX_QUEUES;
	mac->max_msix_vectors	= yusur2_get_pcie_msix_count_generic(hw);

	/* SFP+ Module */
	phy->ops.read_i2c_eeprom = yusur2_read_i2c_eeprom_sn2100;
	phy->ops.read_i2c_sff8472 = yusur2_read_i2c_sff8472_sn2100;

	/* Link */
	mac->ops.check_link = yusur2_check_mac_link_sn2100;
	mac->ops.setup_link = yusur2_setup_mac_link_sn2100;
	mac->ops.flap_tx_laser = NULL;
	mac->ops.get_link_capabilities = yusur2_get_link_capabilities_sn2100;
	mac->ops.setup_rxpba = yusur2_set_rxpba_sn2100;

	/* Manageability interface */
	mac->ops.set_fw_drv_ver = NULL;

	mac->ops.get_rtrup2tc = NULL;

	return ret_val;
}

/**
 *  yusur2_init_phy_ops_sn2100 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during init_shared_code because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
s32 yusur2_init_phy_ops_sn2100(struct yusur2_hw *hw)
{
	struct yusur2_mac_info *mac = &hw->mac;
	struct yusur2_phy_info *phy = &hw->phy;
	s32 ret_val = YUSUR2_SUCCESS;
	u16 list_offset, data_offset;

	DEBUGFUNC("yusur2_init_phy_ops_sn2100");

	/* Identify the PHY */
	phy->ops.identify(hw);

	/* Overwrite the link function pointers if copper PHY */
	if (mac->ops.get_media_type(hw) == yusur2_media_type_copper) {
		mac->ops.setup_link = yusur2_setup_copper_link_sn2100;
		mac->ops.get_link_capabilities =
				yusur2_get_copper_link_capabilities_generic;
	}

	switch (hw->phy.type) {
	case yusur2_phy_tn:
		phy->ops.setup_link = yusur2_setup_phy_link_tnx;
		phy->ops.check_link = yusur2_check_phy_link_tnx;
		phy->ops.get_firmware_version =
					yusur2_get_phy_firmware_version_tnx;
		break;
	case yusur2_phy_nl:
		phy->ops.reset = yusur2_reset_phy_nl;

		/* Call SFP+ identify routine to get the SFP+ module type */
		ret_val = phy->ops.identify_sfp(hw);
		if (ret_val != YUSUR2_SUCCESS)
			goto out;
		else if (hw->phy.sfp_type == yusur2_sfp_type_unknown) {
			ret_val = YUSUR2_ERR_SFP_NOT_SUPPORTED;
			goto out;
		}

		/* Check to see if SFP+ module is supported */
		ret_val = yusur2_get_sfp_init_sequence_offsets(hw,
							      &list_offset,
							      &data_offset);
		if (ret_val != YUSUR2_SUCCESS) {
			ret_val = YUSUR2_ERR_SFP_NOT_SUPPORTED;
			goto out;
		}
		break;
	default:
		break;
	}

out:
	return ret_val;
}

/**
 *  yusur2_start_hw_sn2100 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function.
 *  Disables relaxed ordering Then set pcie completion timeout
 *
 **/
s32 yusur2_start_hw_sn2100(struct yusur2_hw *hw)
{
	u32 regval;
	u32 i;
	s32 ret_val = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_start_hw_sn2100");

	ret_val = yusur2_start_hw_generic(hw);
	if (ret_val)
		return ret_val;

	/* set the completion timeout for interface */
	yusur2_set_pcie_completion_timeout(hw);

	return ret_val;
}

/**
 *  yusur2_get_link_capabilities_sn2100 - Determines link capabilities
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @autoneg: boolean auto-negotiation value
 *
 *  Determines the link capabilities by reading the AUTOC register.
 **/
STATIC s32 yusur2_get_link_capabilities_sn2100(struct yusur2_hw *hw,
					     yusur2_link_speed *speed,
					     bool *autoneg)
{
	s32 status = YUSUR2_SUCCESS;
	u32 autoc = 0;

	DEBUGFUNC("yusur2_get_link_capabilities_sn2100");

	/*
	 * Determine link capabilities based on the stored value of AUTOC,
	 * which represents EEPROM defaults.  If AUTOC value has not been
	 * stored, use the current register value.
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

	case YUSUR2_AUTOC_LMS_KX4_AN:
	case YUSUR2_AUTOC_LMS_KX4_AN_1G_AN:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
		if (autoc & YUSUR2_AUTOC_KX4_SUPP)
			*speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (autoc & YUSUR2_AUTOC_KX_SUPP)
			*speed |= YUSUR2_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	default:
		status = YUSUR2_ERR_LINK_SETUP;
		break;
	}

	return status;
}

/**
 *  yusur2_get_media_type_sn2100 - Determines media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
STATIC enum yusur2_media_type yusur2_get_media_type_sn2100(struct yusur2_hw *hw)
{
	enum yusur2_media_type media_type;

	DEBUGFUNC("yusur2_get_media_type_sn2100");

	//TODO: check
#if 0

	/* Detect if there is a copper PHY attached. */
	switch (hw->phy.type) {
	case yusur2_phy_cu_unknown:
	case yusur2_phy_tn:
		media_type = yusur2_media_type_copper;
		goto out;
	default:
		break;
	}

	/* Media type for Isn2100 is based on device ID */
	switch (hw->device_id) {
	case YUSUR2_DEV_ID_sn2100:
	case YUSUR2_DEV_ID_sn2100_BX:
		/* Default device ID is mezzanine card KX/KX4 */
		media_type = yusur2_media_type_backplane;
		break;
	case YUSUR2_DEV_ID_sn2100AF_DUAL_PORT:
	case YUSUR2_DEV_ID_sn2100AF_SINGLE_PORT:
	case YUSUR2_DEV_ID_sn2100_DA_DUAL_PORT:
	case YUSUR2_DEV_ID_sn2100_SR_DUAL_PORT_EM:
	case YUSUR2_DEV_ID_sn2100EB_XF_LR:
	case YUSUR2_DEV_ID_sn2100EB_SFP_LOM:
		media_type = yusur2_media_type_fiber;
		break;
	case YUSUR2_DEV_ID_sn2100EB_CX4:
	case YUSUR2_DEV_ID_sn2100_CX4_DUAL_PORT:
		media_type = yusur2_media_type_cx4;
		break;
	case YUSUR2_DEV_ID_sn2100AT:
	case YUSUR2_DEV_ID_sn2100AT2:
		media_type = yusur2_media_type_copper;
		break;
	default:
		media_type = yusur2_media_type_unknown;
		break;
	}
out:
#endif
	media_type = yusur2_media_type_fiber;
	return media_type;
}

/**
 *  yusur2_fc_enable_sn2100 - Enable flow control
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to the current settings.
 **/
s32 yusur2_fc_enable_sn2100(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;
	u32 fctrl_reg;
	u32 rmcs_reg;
	u32 reg;
	u32 fcrtl, fcrth;
	u32 link_speed = 0;
	int i;
	bool link_up;

	DEBUGFUNC("yusur2_fc_enable_sn2100");

	/* Validate the water mark configuration */
	if (!hw->fc.pause_time) {
		ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	/* Low water mark of zero causes XOFF floods */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & yusur2_fc_tx_pause) &&
		    hw->fc.high_water[i]) {
			if (!hw->fc.low_water[i] ||
			    hw->fc.low_water[i] >= hw->fc.high_water[i]) {
				DEBUGOUT("Invalid water mark configuration\n");
				ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
				goto out;
			}
		}
	}

	/*
	 * On sn2100 having Rx FC on causes resets while doing 1G
	 * so if it's on turn it off once we know link_speed. For
	 * more details see sn2100 Specification update.
	 */
	hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
	if (link_up && link_speed == YUSUR2_LINK_SPEED_1GB_FULL) {
		switch (hw->fc.requested_mode) {
		case yusur2_fc_full:
			hw->fc.requested_mode = yusur2_fc_tx_pause;
			break;
		case yusur2_fc_rx_pause:
			hw->fc.requested_mode = yusur2_fc_none;
			break;
		default:
			/* no change */
			break;
		}
	}

	/* Negotiate the fc mode to use */
	yusur2_fc_autoneg(hw);

	/* Disable any previous flow control settings */
	fctrl_reg = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl_reg &= ~(YUSUR2_FCTRL_RFCE | YUSUR2_FCTRL_RPFCE);

	rmcs_reg = YUSUR2_READ_REG(hw, YUSUR2_RMCS);
	rmcs_reg &= ~(YUSUR2_RMCS_TFCE_PRIORITY | YUSUR2_RMCS_TFCE_802_3X);

	/*
	 * The possible values of fc.current_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *     we do not support receiving pause frames).
	 * 3: Both Rx and Tx flow control (symmetric) are enabled.
	 * other: Invalid.
	 */
	switch (hw->fc.current_mode) {
	case yusur2_fc_none:
		/*
		 * Flow control is disabled by software override or autoneg.
		 * The code below will actually disable it in the HW.
		 */
		break;
	case yusur2_fc_rx_pause:
		/*
		 * Rx Flow control is enabled and Tx Flow control is
		 * disabled by software override. Since there really
		 * isn't a way to advertise that we are capable of RX
		 * Pause ONLY, we will advertise that we support both
		 * symmetric and asymmetric Rx PAUSE.  Later, we will
		 * disable the adapter's ability to send PAUSE frames.
		 */
		fctrl_reg |= YUSUR2_FCTRL_RFCE;
		break;
	case yusur2_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		rmcs_reg |= YUSUR2_RMCS_TFCE_802_3X;
		break;
	case yusur2_fc_full:
		/* Flow control (both Rx and Tx) is enabled by SW override. */
		fctrl_reg |= YUSUR2_FCTRL_RFCE;
		rmcs_reg |= YUSUR2_RMCS_TFCE_802_3X;
		break;
	default:
		DEBUGOUT("Flow control param set incorrectly\n");
		ret_val = YUSUR2_ERR_CONFIG;
		goto out;
		break;
	}

	/* Set 802.3x based flow control settings. */
	fctrl_reg |= YUSUR2_FCTRL_DPF;
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl_reg);
	YUSUR2_WRITE_REG(hw, YUSUR2_RMCS, rmcs_reg);

	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & yusur2_fc_tx_pause) &&
		    hw->fc.high_water[i]) {
			fcrtl = (hw->fc.low_water[i] << 10) | YUSUR2_FCRTL_XONE;
			fcrth = (hw->fc.high_water[i] << 10) | YUSUR2_FCRTH_FCEN;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL(i), fcrtl);
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH(i), fcrth);
		} else {
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL(i), 0);
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH(i), 0);
		}

	}

	/* Configure pause time (2 TCs per register) */
	reg = hw->fc.pause_time * 0x00010001;
	for (i = 0; i < (YUSUR2_DCB_MAX_TRAFFIC_CLASS / 2); i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_FCTTV(i), reg);

	/* Configure flow control refresh threshold value */
	YUSUR2_WRITE_REG(hw, YUSUR2_FCRTV, hw->fc.pause_time / 2);

out:
	return ret_val;
}

/**
 *  yusur2_start_mac_link_sn2100 - Configures MAC link settings
 *  @hw: pointer to hardware structure
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Configures link settings based on values in the yusur2_hw struct.
 *  Restarts the link.  Performs autonegotiation if needed.
 **/
STATIC s32 yusur2_start_mac_link_sn2100(struct yusur2_hw *hw,
				      bool autoneg_wait_to_complete)
{
	u32 autoc_reg;
	u32 links_reg;
	u32 i;
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_start_mac_link_sn2100");

	/* Restart link */
	autoc_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	autoc_reg |= YUSUR2_AUTOC_AN_RESTART;
	YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, autoc_reg);

	/* Only poll for autoneg to complete if specified to do so */
	if (autoneg_wait_to_complete) {
		if ((autoc_reg & YUSUR2_AUTOC_LMS_MASK) ==
		     YUSUR2_AUTOC_LMS_KX4_AN ||
		    (autoc_reg & YUSUR2_AUTOC_LMS_MASK) ==
		     YUSUR2_AUTOC_LMS_KX4_AN_1G_AN) {
			links_reg = 0; /* Just in case Autoneg time = 0 */
			for (i = 0; i < YUSUR2_AUTO_NEG_TIME; i++) {
				links_reg = YUSUR2_READ_REG(hw, YUSUR2_LINKS);
				if (links_reg & YUSUR2_LINKS_KX_AN_COMP)
					break;
				msec_delay(100);
			}
			if (!(links_reg & YUSUR2_LINKS_KX_AN_COMP)) {
				status = YUSUR2_ERR_AUTONEG_NOT_COMPLETE;
				DEBUGOUT("Autonegotiation did not complete.\n");
			}
		}
	}

	/* Add delay to filter out noises during initial link setup */
	msec_delay(50);

	return status;
}

/**
 *  yusur2_validate_link_ready - Function looks for phy link
 *  @hw: pointer to hardware structure
 *
 *  Function indicates success when phy link is available. If phy is not ready
 *  within 5 seconds of MAC indicating link, the function returns error.
 **/
STATIC s32 yusur2_validate_link_ready(struct yusur2_hw *hw)
{
	//TODO: check phy link status
#if 0
	u32 timeout;
	u16 an_reg;

	if (hw->device_id != YUSUR2_DEV_ID_sn2100AT2)
		return YUSUR2_SUCCESS;

	for (timeout = 0;
	     timeout < YUSUR2_VALIDATE_LINK_READY_TIMEOUT; timeout++) {
		hw->phy.ops.read_reg(hw, YUSUR2_MDIO_AUTO_NEG_STATUS,
				     YUSUR2_MDIO_AUTO_NEG_DEV_TYPE, &an_reg);

		if ((an_reg & YUSUR2_MII_AUTONEG_COMPLETE) &&
		    (an_reg & YUSUR2_MII_AUTONEG_LINK_UP))
			break;

		msec_delay(100);
	}

	if (timeout == YUSUR2_VALIDATE_LINK_READY_TIMEOUT) {
		DEBUGOUT("Link was indicated but link is down\n");
		return YUSUR2_ERR_LINK_SETUP;
	}
#endif
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_check_mac_link_sn2100 - Get link/speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true is link is up, false otherwise
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
STATIC s32 yusur2_check_mac_link_sn2100(struct yusur2_hw *hw,
				      yusur2_link_speed *speed, bool *link_up,
				      bool link_up_wait_to_complete)
{
	u32 links_reg;
	u32 i;
	u16 link_reg, adapt_comp_reg;

	//TODO:check link status
	DEBUGFUNC("yusur2_check_mac_link_sn2100");
#if 0

	/*
	 * SERDES PHY requires us to read link status from undocumented
	 * register 0xC79F.  Bit 0 set indicates link is up/ready; clear
	 * indicates link down.  OxC00C is read to check that the XAUI lanes
	 * are active.  Bit 0 clear indicates active; set indicates inactive.
	 */
	if (hw->phy.type == yusur2_phy_nl) {
		hw->phy.ops.read_reg(hw, 0xC79F, YUSUR2_TWINAX_DEV, &link_reg);
		hw->phy.ops.read_reg(hw, 0xC79F, YUSUR2_TWINAX_DEV, &link_reg);
		hw->phy.ops.read_reg(hw, 0xC00C, YUSUR2_TWINAX_DEV,
				     &adapt_comp_reg);
		if (link_up_wait_to_complete) {
			for (i = 0; i < hw->mac.max_link_up_time; i++) {
				if ((link_reg & 1) &&
				    ((adapt_comp_reg & 1) == 0)) {
					*link_up = true;
					break;
				} else {
					*link_up = false;
				}
				msec_delay(100);
				hw->phy.ops.read_reg(hw, 0xC79F,
						     YUSUR2_TWINAX_DEV,
						     &link_reg);
				hw->phy.ops.read_reg(hw, 0xC00C,
						     YUSUR2_TWINAX_DEV,
						     &adapt_comp_reg);
			}
		} else {
			if ((link_reg & 1) && ((adapt_comp_reg & 1) == 0))
				*link_up = true;
			else
				*link_up = false;
		}

		if (*link_up == false)
			goto out;
	}

	links_reg = YUSUR2_READ_REG(hw, YUSUR2_LINKS);
	if (link_up_wait_to_complete) {
		for (i = 0; i < hw->mac.max_link_up_time; i++) {
			if (links_reg & YUSUR2_LINKS_UP) {
				*link_up = true;
				break;
			} else {
				*link_up = false;
			}
			msec_delay(100);
			links_reg = YUSUR2_READ_REG(hw, YUSUR2_LINKS);
		}
	} else {
		if (links_reg & YUSUR2_LINKS_UP)
			*link_up = true;
		else
			*link_up = false;
	}

	if (links_reg & YUSUR2_LINKS_SPEED)
		*speed = YUSUR2_LINK_SPEED_10GB_FULL;
	else
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;

	if ((hw->device_id == YUSUR2_DEV_ID_sn2100AT2) && (*link_up == true) &&
	    (yusur2_validate_link_ready(hw) != YUSUR2_SUCCESS))
		*link_up = false;

out:
#endif
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_setup_mac_link_sn2100 - Set MAC link speed
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Set the link speed in the AUTOC register and restarts link.
 **/
STATIC s32 yusur2_setup_mac_link_sn2100(struct yusur2_hw *hw,
				      yusur2_link_speed speed,
				      bool autoneg_wait_to_complete)
{
	bool autoneg = false;
	s32 status = YUSUR2_SUCCESS;
	yusur2_link_speed link_capabilities = YUSUR2_LINK_SPEED_UNKNOWN;
	u32 curr_autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	u32 autoc = curr_autoc;
	u32 link_mode = autoc & YUSUR2_AUTOC_LMS_MASK;

	DEBUGFUNC("yusur2_setup_mac_link_sn2100");

	/* Check to see if speed passed in is supported. */
	yusur2_get_link_capabilities(hw, &link_capabilities, &autoneg);
	speed &= link_capabilities;

	if (speed == YUSUR2_LINK_SPEED_UNKNOWN)
		status = YUSUR2_ERR_LINK_SETUP;

	/* Set KX4/KX support according to speed requested */
	else if (link_mode == YUSUR2_AUTOC_LMS_KX4_AN ||
		 link_mode == YUSUR2_AUTOC_LMS_KX4_AN_1G_AN) {
		autoc &= ~YUSUR2_AUTOC_KX4_KX_SUPP_MASK;
		if (speed & YUSUR2_LINK_SPEED_10GB_FULL)
			autoc |= YUSUR2_AUTOC_KX4_SUPP;
		if (speed & YUSUR2_LINK_SPEED_1GB_FULL)
			autoc |= YUSUR2_AUTOC_KX_SUPP;
		if (autoc != curr_autoc)
			YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, autoc);
	}

	if (status == YUSUR2_SUCCESS) {
		/*
		 * Setup and restart the link based on the new values in
		 * yusur2_hw This will write the AUTOC register based on the new
		 * stored values
		 */
		status = yusur2_start_mac_link_sn2100(hw,
						    autoneg_wait_to_complete);
	}

	return status;
}


/**
 *  yusur2_setup_copper_link_sn2100 - Set the PHY autoneg advertised field
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true if waiting is needed to complete
 *
 *  Sets the link speed in the AUTOC register in the MAC and restarts link.
 **/
STATIC s32 yusur2_setup_copper_link_sn2100(struct yusur2_hw *hw,
					 yusur2_link_speed speed,
					 bool autoneg_wait_to_complete)
{
	s32 status;

	DEBUGFUNC("yusur2_setup_copper_link_sn2100");

	/* Setup the PHY according to input speed */
	status = hw->phy.ops.setup_link_speed(hw, speed,
					      autoneg_wait_to_complete);
	/* Set up MAC */
	yusur2_start_mac_link_sn2100(hw, autoneg_wait_to_complete);

	return status;
}

/**
 *  yusur2_reset_hw_sn2100 - Performs hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks and
 *  clears all interrupts, performing a PHY reset, and performing a link (MAC)
 *  reset.
 **/
STATIC s32 yusur2_reset_hw_sn2100(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;
	s32 phy_status = YUSUR2_SUCCESS;
	u32 ctrl;
	u32 gheccr;
	u32 i;
	u32 autoc;
	u8  analog_val;

	DEBUGFUNC("yusur2_reset_hw_sn2100");

	/* Call adapter stop to disable tx/rx and clear interrupts */
	status = hw->mac.ops.stop_adapter(hw);
	if (status != YUSUR2_SUCCESS)
		goto reset_hw_out;

	/*
	 * Power up the Atlas Tx lanes if they are currently powered down.
	 * Atlas Tx lanes are powered down for MAC loopback tests, but
	 * they are not automatically restored on reset.
	 */
	hw->mac.ops.read_analog_reg8(hw, YUSUR2_ATLAS_PDN_LPBK, &analog_val);
	if (analog_val & YUSUR2_ATLAS_PDN_TX_REG_EN) {
		/* Enable Tx Atlas so packets can be transmitted again */
		hw->mac.ops.read_analog_reg8(hw, YUSUR2_ATLAS_PDN_LPBK,
					     &analog_val);
		analog_val &= ~YUSUR2_ATLAS_PDN_TX_REG_EN;
		hw->mac.ops.write_analog_reg8(hw, YUSUR2_ATLAS_PDN_LPBK,
					      analog_val);

		hw->mac.ops.read_analog_reg8(hw, YUSUR2_ATLAS_PDN_10G,
					     &analog_val);
		analog_val &= ~YUSUR2_ATLAS_PDN_TX_10G_QL_ALL;
		hw->mac.ops.write_analog_reg8(hw, YUSUR2_ATLAS_PDN_10G,
					      analog_val);

		hw->mac.ops.read_analog_reg8(hw, YUSUR2_ATLAS_PDN_1G,
					     &analog_val);
		analog_val &= ~YUSUR2_ATLAS_PDN_TX_1G_QL_ALL;
		hw->mac.ops.write_analog_reg8(hw, YUSUR2_ATLAS_PDN_1G,
					      analog_val);

		hw->mac.ops.read_analog_reg8(hw, YUSUR2_ATLAS_PDN_AN,
					     &analog_val);
		analog_val &= ~YUSUR2_ATLAS_PDN_TX_AN_QL_ALL;
		hw->mac.ops.write_analog_reg8(hw, YUSUR2_ATLAS_PDN_AN,
					      analog_val);
	}

	/* Reset PHY */
	if (hw->phy.reset_disable == false) {
		/* PHY ops must be identified and initialized prior to reset */

		/* Init PHY and function pointers, perform SFP setup */
		phy_status = hw->phy.ops.init(hw);
		if (phy_status == YUSUR2_ERR_SFP_NOT_SUPPORTED)
			goto reset_hw_out;
		if (phy_status == YUSUR2_ERR_SFP_NOT_PRESENT)
			goto mac_reset_top;

		hw->phy.ops.reset(hw);
	}

mac_reset_top:
	/*
	 * Issue global reset to the MAC.  This needs to be a SW reset.
	 * If link reset is used, it might reset the MAC when mng is using it
	 */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_CTRL) | YUSUR2_CTRL_RST;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL, ctrl);
	YUSUR2_WRITE_FLUSH(hw);

	/* Poll for reset bit to self-clear indicating reset is complete */
	for (i = 0; i < 10; i++) {
		usec_delay(1);
		ctrl = YUSUR2_READ_REG(hw, YUSUR2_CTRL);
		if (!(ctrl & YUSUR2_CTRL_RST))
			break;
	}
	if (ctrl & YUSUR2_CTRL_RST) {
		status = YUSUR2_ERR_RESET_FAILED;
		DEBUGOUT("Reset polling failed to complete.\n");
	}

	msec_delay(50);

	/*
	 * Double resets are required for recovery from certain error
	 * conditions.  Between resets, it is necessary to stall to allow time
	 * for any pending HW events to complete.
	 */
	if (hw->mac.flags & YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED) {
		hw->mac.flags &= ~YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED;
		goto mac_reset_top;
	}

	gheccr = YUSUR2_READ_REG(hw, YUSUR2_GHECCR);
	gheccr &= ~((1 << 21) | (1 << 18) | (1 << 9) | (1 << 6));
	YUSUR2_WRITE_REG(hw, YUSUR2_GHECCR, gheccr);

	/*
	 * Store the original AUTOC value if it has not been
	 * stored off yet.  Otherwise restore the stored original
	 * AUTOC value since the reset operation sets back to deaults.
	 */
	autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	if (hw->mac.orig_link_settings_stored == false) {
		hw->mac.orig_autoc = autoc;
		hw->mac.orig_link_settings_stored = true;
	} else if (autoc != hw->mac.orig_autoc) {
		YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, hw->mac.orig_autoc);
	}

	/* Store the permanent mac address */
	hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

	/*
	 * Store MAC address from RAR0, clear receive address registers, and
	 * clear the multicast table
	 */
	hw->mac.ops.init_rx_addrs(hw);

reset_hw_out:
	if (phy_status != YUSUR2_SUCCESS)
		status = phy_status;

	return status;
}

/**
 *  yusur2_set_vmdq_sn2100 - Associate a VMDq set index with a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq set index
 **/
s32 yusur2_set_vmdq_sn2100(struct yusur2_hw *hw, u32 rar, u32 vmdq)
{
	u32 rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_set_vmdq_sn2100");

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		DEBUGOUT1("RAR index %d is out of range.\n", rar);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(rar));
	rar_high &= ~YUSUR2_RAH_VIND_MASK;
	rar_high |= ((vmdq << YUSUR2_RAH_VIND_SHIFT) & YUSUR2_RAH_VIND_MASK);
	YUSUR2_WRITE_REG(hw, YUSUR2_RAH(rar), rar_high);
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_clear_vmdq_sn2100 - Disassociate a VMDq set index from an rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq clear index (not used in sn2100, but elsewhere)
 **/
STATIC s32 yusur2_clear_vmdq_sn2100(struct yusur2_hw *hw, u32 rar, u32 vmdq)
{
	u32 rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	UNREFERENCED_1PARAMETER(vmdq);

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		DEBUGOUT1("RAR index %d is out of range.\n", rar);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(rar));
	if (rar_high & YUSUR2_RAH_VIND_MASK) {
		rar_high &= ~YUSUR2_RAH_VIND_MASK;
		YUSUR2_WRITE_REG(hw, YUSUR2_RAH(rar), rar_high);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_vfta_sn2100 - Set VLAN filter table
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VFTA
 *  @vlan_on: boolean flag to turn on/off VLAN in VFTA
 *  @vlvf_bypass: boolean flag - unused
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 yusur2_set_vfta_sn2100(struct yusur2_hw *hw, u32 vlan, u32 vind,
			 bool vlan_on, bool vlvf_bypass)
{
	u32 regindex;
	u32 bitindex;
	u32 bits;
	u32 vftabyte;

	UNREFERENCED_1PARAMETER(vlvf_bypass);

	DEBUGFUNC("yusur2_set_vfta_sn2100");

	if (vlan > 4095)
		return YUSUR2_ERR_PARAM;

	/* Determine 32-bit word position in array */
	regindex = (vlan >> 5) & 0x7F;   /* upper seven bits */

	/* Determine the location of the (VMD) queue index */
	vftabyte =  ((vlan >> 3) & 0x03); /* bits (4:3) indicating byte array */
	bitindex = (vlan & 0x7) << 2;    /* lower 3 bits indicate nibble */

	/* Set the nibble for VMD queue index */
	bits = YUSUR2_READ_REG(hw, YUSUR2_VFTAVIND(vftabyte, regindex));
	bits &= (~(0x0F << bitindex));
	bits |= (vind << bitindex);
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTAVIND(vftabyte, regindex), bits);

	/* Determine the location of the bit for this VLAN id */
	bitindex = vlan & 0x1F;   /* lower five bits */

	bits = YUSUR2_READ_REG(hw, YUSUR2_VFTA(regindex));
	if (vlan_on)
		/* Turn on this VLAN id */
		bits |= (1 << bitindex);
	else
		/* Turn off this VLAN id */
		bits &= ~(1 << bitindex);
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(regindex), bits);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_clear_vfta_sn2100 - Clear VLAN filter table
 *  @hw: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
STATIC s32 yusur2_clear_vfta_sn2100(struct yusur2_hw *hw)
{
	u32 offset;
	u32 vlanbyte;

	DEBUGFUNC("yusur2_clear_vfta_sn2100");

	for (offset = 0; offset < hw->mac.vft_size; offset++)
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(offset), 0);

	for (vlanbyte = 0; vlanbyte < 4; vlanbyte++)
		for (offset = 0; offset < hw->mac.vft_size; offset++)
			YUSUR2_WRITE_REG(hw, YUSUR2_VFTAVIND(vlanbyte, offset),
					0);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_analog_reg8_sn2100 - Reads 8 bit Atlas analog register
 *  @hw: pointer to hardware structure
 *  @reg: analog register to read
 *  @val: read value
 *
 *  Performs read operation to Atlas analog register specified.
 **/
s32 yusur2_read_analog_reg8_sn2100(struct yusur2_hw *hw, u32 reg, u8 *val)
{
	u32  atlas_ctl;

	DEBUGFUNC("yusur2_read_analog_reg8_sn2100");

	YUSUR2_WRITE_REG(hw, YUSUR2_ATLASCTL,
			YUSUR2_ATLASCTL_WRITE_CMD | (reg << 8));
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(10);
	atlas_ctl = YUSUR2_READ_REG(hw, YUSUR2_ATLASCTL);
	*val = (u8)atlas_ctl;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_write_analog_reg8_sn2100 - Writes 8 bit Atlas analog register
 *  @hw: pointer to hardware structure
 *  @reg: atlas register to write
 *  @val: value to write
 *
 *  Performs write operation to Atlas analog register specified.
 **/
s32 yusur2_write_analog_reg8_sn2100(struct yusur2_hw *hw, u32 reg, u8 val)
{
	u32  atlas_ctl;

	DEBUGFUNC("yusur2_write_analog_reg8_sn2100");

	atlas_ctl = (reg << 8) | val;
	YUSUR2_WRITE_REG(hw, YUSUR2_ATLASCTL, atlas_ctl);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(10);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_i2c_phy_sn2100 - Reads 8 bit word over I2C interface.
 *  @hw: pointer to hardware structure
 *  @dev_addr: address to read from
 *  @byte_offset: byte offset to read from dev_addr
 *  @eeprom_data: value read
 *
 *  Performs 8 byte read operation to SFP module's EEPROM over I2C interface.
 **/
STATIC s32 yusur2_read_i2c_phy_sn2100(struct yusur2_hw *hw, u8 dev_addr,
				    u8 byte_offset, u8 *eeprom_data)
{
	s32 status = YUSUR2_SUCCESS;
	u16 sfp_addr = 0;
	u16 sfp_data = 0;
	u16 sfp_stat = 0;
	u16 gssr;
	u32 i;

	DEBUGFUNC("yusur2_read_i2c_phy_sn2100");

	if (YUSUR2_READ_REG(hw, YUSUR2_STATUS) & YUSUR2_STATUS_LAN_ID_1)
		gssr = YUSUR2_GSSR_PHY1_SM;
	else
		gssr = YUSUR2_GSSR_PHY0_SM;

	if (hw->mac.ops.acquire_swfw_sync(hw, gssr) != YUSUR2_SUCCESS)
		return YUSUR2_ERR_SWFW_SYNC;

	if (hw->phy.type == yusur2_phy_nl) {
		/*
		 * NetLogic phy SDA/SCL registers are at addresses 0xC30A to
		 * 0xC30D. These registers are used to talk to the SFP+
		 * module's EEPROM through the SDA/SCL (I2C) interface.
		 */
		sfp_addr = (dev_addr << 8) + byte_offset;
		sfp_addr = (sfp_addr | YUSUR2_I2C_EEPROM_READ_MASK);
		hw->phy.ops.write_reg_mdi(hw,
					  YUSUR2_MDIO_PMA_PMD_SDA_SCL_ADDR,
					  YUSUR2_MDIO_PMA_PMD_DEV_TYPE,
					  sfp_addr);

		/* Poll status */
		for (i = 0; i < 100; i++) {
			hw->phy.ops.read_reg_mdi(hw,
						YUSUR2_MDIO_PMA_PMD_SDA_SCL_STAT,
						YUSUR2_MDIO_PMA_PMD_DEV_TYPE,
						&sfp_stat);
			sfp_stat = sfp_stat & YUSUR2_I2C_EEPROM_STATUS_MASK;
			if (sfp_stat != YUSUR2_I2C_EEPROM_STATUS_IN_PROGRESS)
				break;
			msec_delay(10);
		}

		if (sfp_stat != YUSUR2_I2C_EEPROM_STATUS_PASS) {
			DEBUGOUT("EEPROM read did not pass.\n");
			status = YUSUR2_ERR_SFP_NOT_PRESENT;
			goto out;
		}

		/* Read data */
		hw->phy.ops.read_reg_mdi(hw, YUSUR2_MDIO_PMA_PMD_SDA_SCL_DATA,
					YUSUR2_MDIO_PMA_PMD_DEV_TYPE, &sfp_data);

		*eeprom_data = (u8)(sfp_data >> 8);
	} else {
		status = YUSUR2_ERR_PHY;
	}

out:
	hw->mac.ops.release_swfw_sync(hw, gssr);
	return status;
}

/**
 *  yusur2_read_i2c_eeprom_sn2100 - Reads 8 bit word over I2C interface.
 *  @hw: pointer to hardware structure
 *  @byte_offset: EEPROM byte offset to read
 *  @eeprom_data: value read
 *
 *  Performs 8 byte read operation to SFP module's EEPROM over I2C interface.
 **/
s32 yusur2_read_i2c_eeprom_sn2100(struct yusur2_hw *hw, u8 byte_offset,
				u8 *eeprom_data)
{
	return yusur2_read_i2c_phy_sn2100(hw, YUSUR2_I2C_EEPROM_DEV_ADDR,
					byte_offset, eeprom_data);
}

/**
 *  yusur2_read_i2c_sff8472_sn2100 - Reads 8 bit word over I2C interface.
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset at address 0xA2
 *  @sff8472_data: value read
 *
 *  Performs 8 byte read operation to SFP module's SFF-8472 data over I2C
 **/
STATIC s32 yusur2_read_i2c_sff8472_sn2100(struct yusur2_hw *hw, u8 byte_offset,
					u8 *sff8472_data)
{
	return yusur2_read_i2c_phy_sn2100(hw, YUSUR2_I2C_EEPROM_DEV_ADDR2,
					byte_offset, sff8472_data);
}

/**
 *  yusur2_get_supported_physical_layer_sn2100 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
u64 yusur2_get_supported_physical_layer_sn2100(struct yusur2_hw *hw)
{
	//TODO: check further...
#if 0
	u64 physical_layer = YUSUR2_PHYSICAL_LAYER_UNKNOWN;
	u32 autoc = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	u32 pma_pmd_10g = autoc & YUSUR2_AUTOC_10G_PMA_PMD_MASK;
	u32 pma_pmd_1g = autoc & YUSUR2_AUTOC_1G_PMA_PMD_MASK;
	u16 ext_ability = 0;

	DEBUGFUNC("yusur2_get_supported_physical_layer_sn2100");

	hw->phy.ops.identify(hw);

	/* Copper PHY must be checked before AUTOC LMS to determine correct
	 * physical layer because 10GBase-T PHYs use LMS = KX4/KX */
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
		if (pma_pmd_1g == YUSUR2_AUTOC_1G_KX)
			physical_layer = YUSUR2_PHYSICAL_LAYER_1000BASE_KX;
		else
			physical_layer = YUSUR2_PHYSICAL_LAYER_1000BASE_BX;
		break;
	case YUSUR2_AUTOC_LMS_10G_LINK_NO_AN:
		if (pma_pmd_10g == YUSUR2_AUTOC_10G_CX4)
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_CX4;
		else if (pma_pmd_10g == YUSUR2_AUTOC_10G_KX4)
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_KX4;
		else /* XAUI */
			physical_layer = YUSUR2_PHYSICAL_LAYER_UNKNOWN;
		break;
	case YUSUR2_AUTOC_LMS_KX4_AN:
	case YUSUR2_AUTOC_LMS_KX4_AN_1G_AN:
		if (autoc & YUSUR2_AUTOC_KX_SUPP)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_1000BASE_KX;
		if (autoc & YUSUR2_AUTOC_KX4_SUPP)
			physical_layer |= YUSUR2_PHYSICAL_LAYER_10GBASE_KX4;
		break;
	default:
		break;
	}

	if (hw->phy.type == yusur2_phy_nl) {
		hw->phy.ops.identify_sfp(hw);

		switch (hw->phy.sfp_type) {
		case yusur2_sfp_type_da_cu:
			physical_layer = YUSUR2_PHYSICAL_LAYER_SFP_PLUS_CU;
			break;
		case yusur2_sfp_type_sr:
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_SR;
			break;
		case yusur2_sfp_type_lr:
			physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_LR;
			break;
		default:
			physical_layer = YUSUR2_PHYSICAL_LAYER_UNKNOWN;
			break;
		}
	}

	switch (hw->device_id) {
	case YUSUR2_DEV_ID_sn2100_DA_DUAL_PORT:
		physical_layer = YUSUR2_PHYSICAL_LAYER_SFP_PLUS_CU;
		break;
	case YUSUR2_DEV_ID_sn2100AF_DUAL_PORT:
	case YUSUR2_DEV_ID_sn2100AF_SINGLE_PORT:
	case YUSUR2_DEV_ID_sn2100_SR_DUAL_PORT_EM:
		physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_SR;
		break;
	case YUSUR2_DEV_ID_sn2100EB_XF_LR:
		physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_LR;
		break;
	default:
		break;
	}

out:
#endif
	u64 physical_layer = YUSUR2_PHYSICAL_LAYER_10GBASE_T;
	return physical_layer;
}

/**
 *  yusur2_set_lan_id_multi_port_pcie_sn2100 - Set LAN id for PCIe multiple
 *  port devices.
 *  @hw: pointer to the HW structure
 *
 *  Calls common function and corrects issue with some single port devices
 *  that enable LAN1 but not LAN0.
 **/
void yusur2_set_lan_id_multi_port_pcie_sn2100(struct yusur2_hw *hw)
{
	struct yusur2_bus_info *bus = &hw->bus;
	u16 pci_gen = 0;
	u16 pci_ctrl2 = 0;

	DEBUGFUNC("yusur2_set_lan_id_multi_port_pcie_sn2100");

	yusur2_set_lan_id_multi_port_pcie(hw);

	/* check if LAN0 is disabled */
	hw->eeprom.ops.read(hw, YUSUR2_PCIE_GENERAL_PTR, &pci_gen);
	if ((pci_gen != 0) && (pci_gen != 0xFFFF)) {

		hw->eeprom.ops.read(hw, pci_gen + YUSUR2_PCIE_CTRL2, &pci_ctrl2);

		/* if LAN0 is completely disabled force function to 0 */
		if ((pci_ctrl2 & YUSUR2_PCIE_CTRL2_LAN_DISABLE) &&
		    !(pci_ctrl2 & YUSUR2_PCIE_CTRL2_DISABLE_SELECT) &&
		    !(pci_ctrl2 & YUSUR2_PCIE_CTRL2_DUMMY_ENABLE)) {

			bus->func = 0;
		}
	}
}

/**
 *  yusur2_enable_relaxed_ordering_sn2100 - enable relaxed ordering
 *  @hw: pointer to hardware structure
 *
 **/
void yusur2_enable_relaxed_ordering_sn2100(struct yusur2_hw *hw)
{
	//TODO: check...
	DEBUGFUNC("yusur2_enable_relaxed_ordering_sn2100");

}

/**
 * yusur2_set_rxpba_sn2100 - Initialize RX packet buffer
 * @hw: pointer to hardware structure
 * @num_pb: number of packet buffers to allocate
 * @headroom: reserve n KB of headroom
 * @strategy: packet buffer allocation strategy
 **/
STATIC void yusur2_set_rxpba_sn2100(struct yusur2_hw *hw, int num_pb,
				  u32 headroom, int strategy)
{
	u32 rxpktsize = YUSUR2_RXPBSIZE_64KB;
	u8 i = 0;
	UNREFERENCED_1PARAMETER(headroom);

	if (!num_pb)
		return;

	/* Setup Rx packet buffer sizes */
	switch (strategy) {
	case PBA_STRATEGY_WEIGHTED:
		/* Setup the first four at 80KB */
		rxpktsize = YUSUR2_RXPBSIZE_80KB;
		for (; i < 4; i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(i), rxpktsize);
		/* Setup the last four at 48KB...don't re-init i */
		rxpktsize = YUSUR2_RXPBSIZE_48KB;
		/* Fall Through */
	case PBA_STRATEGY_EQUAL:
	default:
		/* Divide the remaining Rx packet buffer evenly among the TCs */
		for (; i < YUSUR2_MAX_PACKET_BUFFERS; i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(i), rxpktsize);
		break;
	}

	/* Setup Tx packet buffer sizes */
	for (i = 0; i < YUSUR2_MAX_PACKET_BUFFERS; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_TXPBSIZE(i), YUSUR2_TXPBSIZE_40KB);
}

/**
 *  yusur2_enable_rx_dma_sn2100 - Enable the Rx DMA unit
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit
 **/
s32 yusur2_enable_rx_dma_sn2100(struct yusur2_hw *hw, u32 regval)
{
	DEBUGFUNC("yusur2_enable_rx_dma_sn2100");

	YUSUR2_WRITE_REG(hw, YUSUR2_RXCTRL, regval);

	return YUSUR2_SUCCESS;
}
