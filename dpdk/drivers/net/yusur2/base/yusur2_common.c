/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include "yusur2_common.h"
#include "yusur2_phy.h"
#include "yusur2_api.h"

STATIC s32 yusur2_acquire_eeprom(struct yusur2_hw *hw);
STATIC s32 yusur2_get_eeprom_semaphore(struct yusur2_hw *hw);
STATIC void yusur2_release_eeprom_semaphore(struct yusur2_hw *hw);
STATIC s32 yusur2_ready_eeprom(struct yusur2_hw *hw);
STATIC void yusur2_standby_eeprom(struct yusur2_hw *hw);
STATIC void yusur2_shift_out_eeprom_bits(struct yusur2_hw *hw, u16 data,
					u16 count);
STATIC u16 yusur2_shift_in_eeprom_bits(struct yusur2_hw *hw, u16 count);
STATIC void yusur2_raise_eeprom_clk(struct yusur2_hw *hw, u32 *eec);
STATIC void yusur2_lower_eeprom_clk(struct yusur2_hw *hw, u32 *eec);
STATIC void yusur2_release_eeprom(struct yusur2_hw *hw);

STATIC s32 yusur2_mta_vector(struct yusur2_hw *hw, u8 *mc_addr);
STATIC s32 yusur2_get_san_mac_addr_offset(struct yusur2_hw *hw,
					 u16 *san_mac_offset);
STATIC s32 yusur2_read_eeprom_buffer_bit_bang(struct yusur2_hw *hw, u16 offset,
					     u16 words, u16 *data);
STATIC s32 yusur2_write_eeprom_buffer_bit_bang(struct yusur2_hw *hw, u16 offset,
					      u16 words, u16 *data);
STATIC s32 yusur2_detect_eeprom_page_size_generic(struct yusur2_hw *hw,
						 u16 offset);

/**
 *  yusur2_init_ops_generic - Inits function ptrs
 *  @hw: pointer to the hardware structure
 *
 *  Initialize the function pointers.
 **/
s32 yusur2_init_ops_generic(struct yusur2_hw *hw)
{
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	struct yusur2_mac_info *mac = &hw->mac;
	u32 eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

	DEBUGFUNC("yusur2_init_ops_generic");

	/* EEPROM */
	eeprom->ops.init_params = yusur2_init_eeprom_params_generic;
	/* If EEPROM is valid (bit 8 = 1), use EERD otherwise use bit bang */
	if (eec & YUSUR2_EEC_PRES) {
		eeprom->ops.read = yusur2_read_eerd_generic;
		eeprom->ops.read_buffer = yusur2_read_eerd_buffer_generic;
	} else {
		eeprom->ops.read = yusur2_read_eeprom_bit_bang_generic;
		eeprom->ops.read_buffer =
				 yusur2_read_eeprom_buffer_bit_bang_generic;
	}
	eeprom->ops.write = yusur2_write_eeprom_generic;
	eeprom->ops.write_buffer = yusur2_write_eeprom_buffer_bit_bang_generic;
	eeprom->ops.validate_checksum =
				      yusur2_validate_eeprom_checksum_generic;
	eeprom->ops.update_checksum = yusur2_update_eeprom_checksum_generic;
	eeprom->ops.calc_checksum = yusur2_calc_eeprom_checksum_generic;

	/* MAC */
	mac->ops.init_hw = yusur2_init_hw_generic;
	mac->ops.reset_hw = NULL;
	mac->ops.start_hw = yusur2_start_hw_generic;
	mac->ops.clear_hw_cntrs = yusur2_clear_hw_cntrs_generic;
	mac->ops.get_media_type = NULL;
	mac->ops.get_supported_physical_layer = NULL;
	mac->ops.enable_rx_dma = yusur2_enable_rx_dma_generic;
	mac->ops.get_mac_addr = yusur2_get_mac_addr_generic;
	mac->ops.stop_adapter = yusur2_stop_adapter_generic;
	mac->ops.get_bus_info = yusur2_get_bus_info_generic;
	mac->ops.set_lan_id = yusur2_set_lan_id_multi_port_pcie;
	mac->ops.acquire_swfw_sync = yusur2_acquire_swfw_sync;
	mac->ops.release_swfw_sync = yusur2_release_swfw_sync;
	mac->ops.prot_autoc_read = yusur2_prot_autoc_read_generic;
	mac->ops.prot_autoc_write = yusur2_prot_autoc_write_generic;

	/* LEDs */
	mac->ops.led_on = yusur2_led_on_generic;
	mac->ops.led_off = yusur2_led_off_generic;
	mac->ops.blink_led_start = yusur2_blink_led_start_generic;
	mac->ops.blink_led_stop = yusur2_blink_led_stop_generic;
	mac->ops.init_led_link_act = yusur2_init_led_link_act_generic;

	/* RAR, Multicast, VLAN */
	mac->ops.set_rar = yusur2_set_rar_generic;
	mac->ops.clear_rar = yusur2_clear_rar_generic;
	mac->ops.insert_mac_addr = NULL;
	mac->ops.set_vmdq = NULL;
	mac->ops.clear_vmdq = NULL;
	mac->ops.init_rx_addrs = yusur2_init_rx_addrs_generic;
	mac->ops.update_uc_addr_list = yusur2_update_uc_addr_list_generic;
	mac->ops.update_mc_addr_list = yusur2_update_mc_addr_list_generic;
	mac->ops.enable_mc = yusur2_enable_mc_generic;
	mac->ops.disable_mc = yusur2_disable_mc_generic;
	mac->ops.clear_vfta = NULL;
	mac->ops.set_vfta = NULL;
	mac->ops.set_vlvf = NULL;
	mac->ops.init_uta_tables = NULL;
	mac->ops.enable_rx = yusur2_enable_rx_generic;
	mac->ops.disable_rx = yusur2_disable_rx_generic;

	/* Flow Control */
	mac->ops.fc_enable = yusur2_fc_enable_generic;
	mac->ops.setup_fc = yusur2_setup_fc_generic;
	mac->ops.fc_autoneg = yusur2_fc_autoneg;

	/* Link */
	mac->ops.get_link_capabilities = NULL;
	mac->ops.setup_link = NULL;
	mac->ops.check_link = NULL;
	mac->ops.dmac_config = NULL;
	mac->ops.dmac_update_tcs = NULL;
	mac->ops.dmac_config_tcs = NULL;

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_device_supports_autoneg_fc - Check if device supports autonegotiation
 * of flow control
 * @hw: pointer to hardware structure
 *
 * This function returns true if the device supports flow control
 * autonegotiation, and false if it does not.
 *
 **/
bool yusur2_device_supports_autoneg_fc(struct yusur2_hw *hw)
{
	bool supported = false;
//TODO: support autonegotiation
#if 0
	yusur2_link_speed speed;
	bool link_up;

	DEBUGFUNC("yusur2_device_supports_autoneg_fc");

	switch (hw->phy.media_type) {
	case yusur2_media_type_fiber_qsfp:
	case yusur2_media_type_fiber:
		/* flow control autoneg black list */
		switch (hw->device_id) {
		case YUSUR2_DEV_ID_X550EM_A_SFP:
		case YUSUR2_DEV_ID_X550EM_A_SFP_N:
		case YUSUR2_DEV_ID_X550EM_A_QSFP:
		case YUSUR2_DEV_ID_X550EM_A_QSFP_N:
			supported = false;
			break;
		default:
			hw->mac.ops.check_link(hw, &speed, &link_up, false);
			/* if link is down, assume supported */
			if (link_up)
				supported = speed == YUSUR2_LINK_SPEED_1GB_FULL ?
				true : false;
			else
				supported = true;
		}

		break;
	case yusur2_media_type_backplane:
		if (hw->device_id == YUSUR2_DEV_ID_X550EM_X_XFI)
			supported = false;
		else
			supported = true;
		break;
	case yusur2_media_type_copper:
		/* only some copper devices support flow control autoneg */
		switch (hw->device_id) {
		case YUSUR2_DEV_ID_82599_T3_LOM:
		case YUSUR2_DEV_ID_X540T:
		case YUSUR2_DEV_ID_X540T1:
		case YUSUR2_DEV_ID_X550T:
		case YUSUR2_DEV_ID_X550T1:
		case YUSUR2_DEV_ID_X550EM_X_10G_T:
		case YUSUR2_DEV_ID_X550EM_A_10G_T:
		case YUSUR2_DEV_ID_X550EM_A_1G_T:
		case YUSUR2_DEV_ID_X550EM_A_1G_T_L:
			supported = true;
			break;
		default:
			supported = false;
		}
	default:
		break;
	}

	if (!supported)
		ERROR_REPORT2(YUSUR2_ERROR_UNSUPPORTED,
			      "Device %x does not support flow control autoneg",
			      hw->device_id);
#endif
	return supported;
}

/**
 *  yusur2_setup_fc_generic - Set up flow control
 *  @hw: pointer to hardware structure
 *
 *  Called at init time to set up flow control.
 **/
s32 yusur2_setup_fc_generic(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;
//TODO: check..
#if 0
	u32 reg = 0, reg_bp = 0;
	u16 reg_cu = 0;
	bool locked = false;

	DEBUGFUNC("yusur2_setup_fc_generic");

	/* Validate the requested mode */
	if (hw->fc.strict_ieee && hw->fc.requested_mode == yusur2_fc_rx_pause) {
		ERROR_REPORT1(YUSUR2_ERROR_UNSUPPORTED,
			   "yusur2_fc_rx_pause not valid in strict IEEE mode\n");
		ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	/*
	 * 10gig parts do not have a word in the EEPROM to determine the
	 * default flow control setting, so we explicitly set it to full.
	 */
	if (hw->fc.requested_mode == yusur2_fc_default)
		hw->fc.requested_mode = yusur2_fc_full;

	/*
	 * Set up the 1G and 10G flow control advertisement registers so the
	 * HW will be able to do fc autoneg once the cable is plugged in.  If
	 * we link at 10G, the 1G advertisement is harmless and vice versa.
	 */
	switch (hw->phy.media_type) {
	case yusur2_media_type_backplane:
		/* some MAC's need RMW protection on AUTOC */
		ret_val = hw->mac.ops.prot_autoc_read(hw, &locked, &reg_bp);
		if (ret_val != YUSUR2_SUCCESS)
			goto out;

		/* fall through - only backplane uses autoc */
	case yusur2_media_type_fiber_qsfp:
	case yusur2_media_type_fiber:
		reg = YUSUR2_READ_REG(hw, YUSUR2_PCS1GANA);

		break;
	case yusur2_media_type_copper:
		hw->phy.ops.read_reg(hw, YUSUR2_MDIO_AUTO_NEG_ADVT,
				     YUSUR2_MDIO_AUTO_NEG_DEV_TYPE, &reg_cu);
		break;
	default:
		break;
	}

	/*
	 * The possible values of fc.requested_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *    we do not support receiving pause frames).
	 * 3: Both Rx and Tx flow control (symmetric) are enabled.
	 * other: Invalid.
	 */
	switch (hw->fc.requested_mode) {
	case yusur2_fc_none:
		/* Flow control completely disabled by software override. */
		reg &= ~(YUSUR2_PCS1GANA_SYM_PAUSE | YUSUR2_PCS1GANA_ASM_PAUSE);
		if (hw->phy.media_type == yusur2_media_type_backplane)
			reg_bp &= ~(YUSUR2_AUTOC_SYM_PAUSE |
				    YUSUR2_AUTOC_ASM_PAUSE);
		else if (hw->phy.media_type == yusur2_media_type_copper)
			reg_cu &= ~(YUSUR2_TAF_SYM_PAUSE | YUSUR2_TAF_ASM_PAUSE);
		break;
	case yusur2_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		reg |= YUSUR2_PCS1GANA_ASM_PAUSE;
		reg &= ~YUSUR2_PCS1GANA_SYM_PAUSE;
		if (hw->phy.media_type == yusur2_media_type_backplane) {
			reg_bp |= YUSUR2_AUTOC_ASM_PAUSE;
			reg_bp &= ~YUSUR2_AUTOC_SYM_PAUSE;
		} else if (hw->phy.media_type == yusur2_media_type_copper) {
			reg_cu |= YUSUR2_TAF_ASM_PAUSE;
			reg_cu &= ~YUSUR2_TAF_SYM_PAUSE;
		}
		break;
	case yusur2_fc_rx_pause:
		/*
		 * Rx Flow control is enabled and Tx Flow control is
		 * disabled by software override. Since there really
		 * isn't a way to advertise that we are capable of RX
		 * Pause ONLY, we will advertise that we support both
		 * symmetric and asymmetric Rx PAUSE, as such we fall
		 * through to the fc_full statement.  Later, we will
		 * disable the adapter's ability to send PAUSE frames.
		 */
	case yusur2_fc_full:
		/* Flow control (both Rx and Tx) is enabled by SW override. */
		reg |= YUSUR2_PCS1GANA_SYM_PAUSE | YUSUR2_PCS1GANA_ASM_PAUSE;
		if (hw->phy.media_type == yusur2_media_type_backplane)
			reg_bp |= YUSUR2_AUTOC_SYM_PAUSE |
				  YUSUR2_AUTOC_ASM_PAUSE;
		else if (hw->phy.media_type == yusur2_media_type_copper)
			reg_cu |= YUSUR2_TAF_SYM_PAUSE | YUSUR2_TAF_ASM_PAUSE;
		break;
	default:
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT,
			     "Flow control param set incorrectly\n");
		ret_val = YUSUR2_ERR_CONFIG;
		goto out;
		break;
	}

	if (hw->mac.type < yusur2_mac_X540) {
		/*
		 * Enable auto-negotiation between the MAC & PHY;
		 * the MAC will advertise clause 37 flow control.
		 */
		YUSUR2_WRITE_REG(hw, YUSUR2_PCS1GANA, reg);
		reg = YUSUR2_READ_REG(hw, YUSUR2_PCS1GLCTL);

		/* Disable AN timeout */
		if (hw->fc.strict_ieee)
			reg &= ~YUSUR2_PCS1GLCTL_AN_1G_TIMEOUT_EN;

		YUSUR2_WRITE_REG(hw, YUSUR2_PCS1GLCTL, reg);
		DEBUGOUT1("Set up FC; PCS1GLCTL = 0x%08X\n", reg);
	}

	/*
	 * AUTOC restart handles negotiation of 1G and 10G on backplane
	 * and copper. There is no need to set the PCS1GCTL register.
	 *
	 */
	if (hw->phy.media_type == yusur2_media_type_backplane) {
		reg_bp |= YUSUR2_AUTOC_AN_RESTART;
		ret_val = hw->mac.ops.prot_autoc_write(hw, reg_bp, locked);
		if (ret_val)
			goto out;
	} else if ((hw->phy.media_type == yusur2_media_type_copper) &&
		    (yusur2_device_supports_autoneg_fc(hw))) {
		hw->phy.ops.write_reg(hw, YUSUR2_MDIO_AUTO_NEG_ADVT,
				      YUSUR2_MDIO_AUTO_NEG_DEV_TYPE, reg_cu);
	}

	DEBUGOUT1("Set up FC; PCS1GLCTL = 0x%08X\n", reg);
out:
#endif
	return ret_val;
}

/**
 *  yusur2_start_hw_generic - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware by filling the bus info structure and media type, clears
 *  all on chip counters, initializes receive address registers, multicast
 *  table, VLAN filter table, calls routine to set up link and flow control
 *  settings, and leaves transmit and receive units disabled and uninitialized
 **/
s32 yusur2_start_hw_generic(struct yusur2_hw *hw)
{
	s32 ret_val;

	DEBUGFUNC("yusur2_start_hw_generic");

	/* Set the media type */
	hw->phy.media_type = hw->mac.ops.get_media_type(hw);

	/* PHY ops initialization must be done in reset_hw() */

	/* Clear the VLAN filter table */
	hw->mac.ops.clear_vfta(hw);

	/* Clear statistics registers */
	hw->mac.ops.clear_hw_cntrs(hw);

//TODO: snoop support
#if 0
	/* Set No Snoop Disable */
	ctrl_ext = YUSUR2_READ_REG(hw, YUSUR2_CTRL_EXT);
	ctrl_ext |= YUSUR2_CTRL_EXT_NS_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL_EXT, ctrl_ext);
	YUSUR2_WRITE_FLUSH(hw);
#endif

	/* Setup flow control */
	ret_val = yusur2_setup_fc(hw);
	if (ret_val != YUSUR2_SUCCESS && ret_val != YUSUR2_NOT_IMPLEMENTED) {
		DEBUGOUT1("Flow control setup failed, returning %d\n", ret_val);
		return ret_val;
	}
	/* Cache bit indicating need for crosstalk fix */
	hw->need_crosstalk_fix = false;

	/* Clear adapter stopped flag */
	hw->adapter_stopped = false;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_start_hw_gen2 - Init sequence for common device family
 *  @hw: pointer to hw structure
 *
 * Performs the init sequence common to the second generation
 * of 10 GbE devices.
 * Devices in the second generation:
 *     82599
 *     X540
 **/
s32 yusur2_start_hw_gen2(struct yusur2_hw *hw)
{
	u32 i;
	u32 regval;

	/* Clear the rate limiters */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTDQSEL, i);
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTBCNRC, 0);
	}
	YUSUR2_WRITE_FLUSH(hw);

	/* Disable relaxed ordering */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		regval = YUSUR2_READ_REG(hw, YUSUR2_DCA_TXCTRL_82599(i));
		regval &= ~YUSUR2_DCA_TXCTRL_DESC_WRO_EN;
		YUSUR2_WRITE_REG(hw, YUSUR2_DCA_TXCTRL_82599(i), regval);
	}

	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		regval = YUSUR2_READ_REG(hw, YUSUR2_DCA_RXCTRL(i));
		regval &= ~(YUSUR2_DCA_RXCTRL_DATA_WRO_EN |
			    YUSUR2_DCA_RXCTRL_HEAD_WRO_EN);
		YUSUR2_WRITE_REG(hw, YUSUR2_DCA_RXCTRL(i), regval);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_hw_generic - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
s32 yusur2_init_hw_generic(struct yusur2_hw *hw)
{
	s32 status;

	DEBUGFUNC("yusur2_init_hw_generic");

	/* Reset the hardware */
	status = hw->mac.ops.reset_hw(hw);

	if (status == YUSUR2_SUCCESS || status == YUSUR2_ERR_SFP_NOT_PRESENT) {
		/* Start the HW */
		status = hw->mac.ops.start_hw(hw);
	}

	/* Initialize the LED link active for LED blink support */
	if (hw->mac.ops.init_led_link_act)
		hw->mac.ops.init_led_link_act(hw);

	if (status != YUSUR2_SUCCESS)
		DEBUGOUT1("Failed to initialize HW, STATUS = %d\n", status);

	return status;
}

/**
 *  yusur2_clear_hw_cntrs_generic - Generic clear hardware counters
 *  @hw: pointer to hardware structure
 *
 *  Clears all hardware statistics counters by reading them from the hardware
 *  Statistics counters are clear on read.
 **/
s32 yusur2_clear_hw_cntrs_generic(struct yusur2_hw *hw)
{
//TODO: support hardware counters...
#if 0
	u16 i = 0;

	DEBUGFUNC("yusur2_clear_hw_cntrs_generic");

	YUSUR2_READ_REG(hw, YUSUR2_CRCERRS);
	YUSUR2_READ_REG(hw, YUSUR2_ILLERRC);
	YUSUR2_READ_REG(hw, YUSUR2_ERRBC);
	YUSUR2_READ_REG(hw, YUSUR2_MSPDC);
	for (i = 0; i < 8; i++)
		YUSUR2_READ_REG(hw, YUSUR2_MPC(i));

	YUSUR2_READ_REG(hw, YUSUR2_MLFC);
	YUSUR2_READ_REG(hw, YUSUR2_MRFC);
	YUSUR2_READ_REG(hw, YUSUR2_RLEC);
	YUSUR2_READ_REG(hw, YUSUR2_LXONTXC);
	YUSUR2_READ_REG(hw, YUSUR2_LXOFFTXC);
	if (hw->mac.type >= yusur2_mac_82599EB) {
		YUSUR2_READ_REG(hw, YUSUR2_LXONRXCNT);
		YUSUR2_READ_REG(hw, YUSUR2_LXOFFRXCNT);
	} else {
		YUSUR2_READ_REG(hw, YUSUR2_LXONRXC);
		YUSUR2_READ_REG(hw, YUSUR2_LXOFFRXC);
	}

	for (i = 0; i < 8; i++) {
		YUSUR2_READ_REG(hw, YUSUR2_PXONTXC(i));
		YUSUR2_READ_REG(hw, YUSUR2_PXOFFTXC(i));
		if (hw->mac.type >= yusur2_mac_82599EB) {
			YUSUR2_READ_REG(hw, YUSUR2_PXONRXCNT(i));
			YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXCNT(i));
		} else {
			YUSUR2_READ_REG(hw, YUSUR2_PXONRXC(i));
			YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXC(i));
		}
	}
	if (hw->mac.type >= yusur2_mac_82599EB)
		for (i = 0; i < 8; i++)
			YUSUR2_READ_REG(hw, YUSUR2_PXON2OFFCNT(i));
	YUSUR2_READ_REG(hw, YUSUR2_PRC64);
	YUSUR2_READ_REG(hw, YUSUR2_PRC127);
	YUSUR2_READ_REG(hw, YUSUR2_PRC255);
	YUSUR2_READ_REG(hw, YUSUR2_PRC511);
	YUSUR2_READ_REG(hw, YUSUR2_PRC1023);
	YUSUR2_READ_REG(hw, YUSUR2_PRC1522);
	YUSUR2_READ_REG(hw, YUSUR2_GPRC);
	YUSUR2_READ_REG(hw, YUSUR2_BPRC);
	YUSUR2_READ_REG(hw, YUSUR2_MPRC);
	YUSUR2_READ_REG(hw, YUSUR2_GPTC);
	YUSUR2_READ_REG(hw, YUSUR2_GORCL);
	YUSUR2_READ_REG(hw, YUSUR2_GORCH);
	YUSUR2_READ_REG(hw, YUSUR2_GOTCL);
	YUSUR2_READ_REG(hw, YUSUR2_GOTCH);
	if (hw->mac.type == yusur2_mac_82598EB)
		for (i = 0; i < 8; i++)
			YUSUR2_READ_REG(hw, YUSUR2_RNBC(i));
	YUSUR2_READ_REG(hw, YUSUR2_RUC);
	YUSUR2_READ_REG(hw, YUSUR2_RFC);
	YUSUR2_READ_REG(hw, YUSUR2_ROC);
	YUSUR2_READ_REG(hw, YUSUR2_RJC);
	YUSUR2_READ_REG(hw, YUSUR2_MNGPRC);
	YUSUR2_READ_REG(hw, YUSUR2_MNGPDC);
	YUSUR2_READ_REG(hw, YUSUR2_MNGPTC);
	YUSUR2_READ_REG(hw, YUSUR2_TORL);
	YUSUR2_READ_REG(hw, YUSUR2_TORH);
	YUSUR2_READ_REG(hw, YUSUR2_TPR);
	YUSUR2_READ_REG(hw, YUSUR2_TPT);
	YUSUR2_READ_REG(hw, YUSUR2_PTC64);
	YUSUR2_READ_REG(hw, YUSUR2_PTC127);
	YUSUR2_READ_REG(hw, YUSUR2_PTC255);
	YUSUR2_READ_REG(hw, YUSUR2_PTC511);
	YUSUR2_READ_REG(hw, YUSUR2_PTC1023);
	YUSUR2_READ_REG(hw, YUSUR2_PTC1522);
	YUSUR2_READ_REG(hw, YUSUR2_MPTC);
	YUSUR2_READ_REG(hw, YUSUR2_BPTC);
	for (i = 0; i < 16; i++) {
		YUSUR2_READ_REG(hw, YUSUR2_QPRC(i));
		YUSUR2_READ_REG(hw, YUSUR2_QPTC(i));
		if (hw->mac.type >= yusur2_mac_82599EB) {
			YUSUR2_READ_REG(hw, YUSUR2_QBRC_L(i));
			YUSUR2_READ_REG(hw, YUSUR2_QBRC_H(i));
			YUSUR2_READ_REG(hw, YUSUR2_QBTC_L(i));
			YUSUR2_READ_REG(hw, YUSUR2_QBTC_H(i));
			YUSUR2_READ_REG(hw, YUSUR2_QPRDC(i));
		} else {
			YUSUR2_READ_REG(hw, YUSUR2_QBRC(i));
			YUSUR2_READ_REG(hw, YUSUR2_QBTC(i));
		}
	}

	if (hw->mac.type == yusur2_mac_X550 || hw->mac.type == yusur2_mac_X540) {
		if (hw->phy.id == 0)
			yusur2_identify_phy(hw);
		hw->phy.ops.read_reg(hw, YUSUR2_PCRC8ECL,
				     YUSUR2_MDIO_PCS_DEV_TYPE, &i);
		hw->phy.ops.read_reg(hw, YUSUR2_PCRC8ECH,
				     YUSUR2_MDIO_PCS_DEV_TYPE, &i);
		hw->phy.ops.read_reg(hw, YUSUR2_LDPCECL,
				     YUSUR2_MDIO_PCS_DEV_TYPE, &i);
		hw->phy.ops.read_reg(hw, YUSUR2_LDPCECH,
				     YUSUR2_MDIO_PCS_DEV_TYPE, &i);
	}
#endif
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_pba_string_generic - Reads part number string from EEPROM
 *  @hw: pointer to hardware structure
 *  @pba_num: stores the part number string from the EEPROM
 *  @pba_num_size: part number string buffer length
 *
 *  Reads the part number string from the EEPROM.
 **/
s32 yusur2_read_pba_string_generic(struct yusur2_hw *hw, u8 *pba_num,
				  u32 pba_num_size)
{
	s32 ret_val;
	u16 data;
	u16 pba_ptr;
	u16 offset;
	u16 length;

	DEBUGFUNC("yusur2_read_pba_string_generic");

	if (pba_num == NULL) {
		DEBUGOUT("PBA string buffer was null\n");
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	ret_val = hw->eeprom.ops.read(hw, YUSUR2_PBANUM0_PTR, &data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	ret_val = hw->eeprom.ops.read(hw, YUSUR2_PBANUM1_PTR, &pba_ptr);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	/*
	 * if data is not ptr guard the PBA must be in legacy format which
	 * means pba_ptr is actually our second data word for the PBA number
	 * and we can decode it into an ascii string
	 */
	if (data != YUSUR2_PBANUM_PTR_GUARD) {
		DEBUGOUT("NVM PBA number is not stored as string\n");

		/* we will need 11 characters to store the PBA */
		if (pba_num_size < 11) {
			DEBUGOUT("PBA string buffer too small\n");
			return YUSUR2_ERR_NO_SPACE;
		}

		/* extract hex string from data and pba_ptr */
		pba_num[0] = (data >> 12) & 0xF;
		pba_num[1] = (data >> 8) & 0xF;
		pba_num[2] = (data >> 4) & 0xF;
		pba_num[3] = data & 0xF;
		pba_num[4] = (pba_ptr >> 12) & 0xF;
		pba_num[5] = (pba_ptr >> 8) & 0xF;
		pba_num[6] = '-';
		pba_num[7] = 0;
		pba_num[8] = (pba_ptr >> 4) & 0xF;
		pba_num[9] = pba_ptr & 0xF;

		/* put a null character on the end of our string */
		pba_num[10] = '\0';

		/* switch all the data but the '-' to hex char */
		for (offset = 0; offset < 10; offset++) {
			if (pba_num[offset] < 0xA)
				pba_num[offset] += '0';
			else if (pba_num[offset] < 0x10)
				pba_num[offset] += 'A' - 0xA;
		}

		return YUSUR2_SUCCESS;
	}

	ret_val = hw->eeprom.ops.read(hw, pba_ptr, &length);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	if (length == 0xFFFF || length == 0) {
		DEBUGOUT("NVM PBA number section invalid length\n");
		return YUSUR2_ERR_PBA_SECTION;
	}

	/* check if pba_num buffer is big enough */
	if (pba_num_size  < (((u32)length * 2) - 1)) {
		DEBUGOUT("PBA string buffer too small\n");
		return YUSUR2_ERR_NO_SPACE;
	}

	/* trim pba length from start of string */
	pba_ptr++;
	length--;

	for (offset = 0; offset < length; offset++) {
		ret_val = hw->eeprom.ops.read(hw, pba_ptr + offset, &data);
		if (ret_val) {
			DEBUGOUT("NVM Read Error\n");
			return ret_val;
		}
		pba_num[offset * 2] = (u8)(data >> 8);
		pba_num[(offset * 2) + 1] = (u8)(data & 0xFF);
	}
	pba_num[offset * 2] = '\0';

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_pba_num_generic - Reads part number from EEPROM
 *  @hw: pointer to hardware structure
 *  @pba_num: stores the part number from the EEPROM
 *
 *  Reads the part number from the EEPROM.
 **/
s32 yusur2_read_pba_num_generic(struct yusur2_hw *hw, u32 *pba_num)
{
	s32 ret_val;
	u16 data;

	DEBUGFUNC("yusur2_read_pba_num_generic");

	ret_val = hw->eeprom.ops.read(hw, YUSUR2_PBANUM0_PTR, &data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	} else if (data == YUSUR2_PBANUM_PTR_GUARD) {
		DEBUGOUT("NVM Not supported\n");
		return YUSUR2_NOT_IMPLEMENTED;
	}
	*pba_num = (u32)(data << 16);

	ret_val = hw->eeprom.ops.read(hw, YUSUR2_PBANUM1_PTR, &data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}
	*pba_num |= data;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_read_pba_raw
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @max_pba_block_size: PBA block size limit
 *  @pba: pointer to output PBA structure
 *
 *  Reads PBA from EEPROM image when eeprom_buf is not NULL.
 *  Reads PBA from physical EEPROM device when eeprom_buf is NULL.
 *
 **/
s32 yusur2_read_pba_raw(struct yusur2_hw *hw, u16 *eeprom_buf,
		       u32 eeprom_buf_size, u16 max_pba_block_size,
		       struct yusur2_pba *pba)
{
	s32 ret_val;
	u16 pba_block_size;

	if (pba == NULL)
		return YUSUR2_ERR_PARAM;

	if (eeprom_buf == NULL) {
		ret_val = hw->eeprom.ops.read_buffer(hw, YUSUR2_PBANUM0_PTR, 2,
						     &pba->word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > YUSUR2_PBANUM1_PTR) {
			pba->word[0] = eeprom_buf[YUSUR2_PBANUM0_PTR];
			pba->word[1] = eeprom_buf[YUSUR2_PBANUM1_PTR];
		} else {
			return YUSUR2_ERR_PARAM;
		}
	}

	if (pba->word[0] == YUSUR2_PBANUM_PTR_GUARD) {
		if (pba->pba_block == NULL)
			return YUSUR2_ERR_PARAM;

		ret_val = yusur2_get_pba_block_size(hw, eeprom_buf,
						   eeprom_buf_size,
						   &pba_block_size);
		if (ret_val)
			return ret_val;

		if (pba_block_size > max_pba_block_size)
			return YUSUR2_ERR_PARAM;

		if (eeprom_buf == NULL) {
			ret_val = hw->eeprom.ops.read_buffer(hw, pba->word[1],
							     pba_block_size,
							     pba->pba_block);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > (u32)(pba->word[1] +
					      pba_block_size)) {
				memcpy(pba->pba_block,
				       &eeprom_buf[pba->word[1]],
				       pba_block_size * sizeof(u16));
			} else {
				return YUSUR2_ERR_PARAM;
			}
		}
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_write_pba_raw
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @pba: pointer to PBA structure
 *
 *  Writes PBA to EEPROM image when eeprom_buf is not NULL.
 *  Writes PBA to physical EEPROM device when eeprom_buf is NULL.
 *
 **/
s32 yusur2_write_pba_raw(struct yusur2_hw *hw, u16 *eeprom_buf,
			u32 eeprom_buf_size, struct yusur2_pba *pba)
{
	s32 ret_val;

	if (pba == NULL)
		return YUSUR2_ERR_PARAM;

	if (eeprom_buf == NULL) {
		ret_val = hw->eeprom.ops.write_buffer(hw, YUSUR2_PBANUM0_PTR, 2,
						      &pba->word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > YUSUR2_PBANUM1_PTR) {
			eeprom_buf[YUSUR2_PBANUM0_PTR] = pba->word[0];
			eeprom_buf[YUSUR2_PBANUM1_PTR] = pba->word[1];
		} else {
			return YUSUR2_ERR_PARAM;
		}
	}

	if (pba->word[0] == YUSUR2_PBANUM_PTR_GUARD) {
		if (pba->pba_block == NULL)
			return YUSUR2_ERR_PARAM;

		if (eeprom_buf == NULL) {
			ret_val = hw->eeprom.ops.write_buffer(hw, pba->word[1],
							      pba->pba_block[0],
							      pba->pba_block);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > (u32)(pba->word[1] +
					      pba->pba_block[0])) {
				memcpy(&eeprom_buf[pba->word[1]],
				       pba->pba_block,
				       pba->pba_block[0] * sizeof(u16));
			} else {
				return YUSUR2_ERR_PARAM;
			}
		}
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_pba_block_size
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @pba_data_size: pointer to output variable
 *
 *  Returns the size of the PBA block in words. Function operates on EEPROM
 *  image if the eeprom_buf pointer is not NULL otherwise it accesses physical
 *  EEPROM device.
 *
 **/
s32 yusur2_get_pba_block_size(struct yusur2_hw *hw, u16 *eeprom_buf,
			     u32 eeprom_buf_size, u16 *pba_block_size)
{
	s32 ret_val;
	u16 pba_word[2];
	u16 length;

	DEBUGFUNC("yusur2_get_pba_block_size");

	if (eeprom_buf == NULL) {
		ret_val = hw->eeprom.ops.read_buffer(hw, YUSUR2_PBANUM0_PTR, 2,
						     &pba_word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > YUSUR2_PBANUM1_PTR) {
			pba_word[0] = eeprom_buf[YUSUR2_PBANUM0_PTR];
			pba_word[1] = eeprom_buf[YUSUR2_PBANUM1_PTR];
		} else {
			return YUSUR2_ERR_PARAM;
		}
	}

	if (pba_word[0] == YUSUR2_PBANUM_PTR_GUARD) {
		if (eeprom_buf == NULL) {
			ret_val = hw->eeprom.ops.read(hw, pba_word[1] + 0,
						      &length);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > pba_word[1])
				length = eeprom_buf[pba_word[1] + 0];
			else
				return YUSUR2_ERR_PARAM;
		}

		if (length == 0xFFFF || length == 0)
			return YUSUR2_ERR_PBA_SECTION;
	} else {
		/* PBA number in legacy format, there is no PBA Block. */
		length = 0;
	}

	if (pba_block_size != NULL)
		*pba_block_size = length;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_mac_addr_generic - Generic get MAC address
 *  @hw: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
s32 yusur2_get_mac_addr_generic(struct yusur2_hw *hw, u8 *mac_addr)
{
	u32 rar_high;
	u32 rar_low;
	u16 i;

	DEBUGFUNC("yusur2_get_mac_addr_generic");

	rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(0));
	rar_low = YUSUR2_READ_REG(hw, YUSUR2_RAL(0));

	for (i = 0; i < 4; i++)
		mac_addr[i] = (u8)(rar_low >> (i*8));

	for (i = 0; i < 2; i++)
		mac_addr[i+4] = (u8)(rar_high >> (i*8));

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_pci_config_data_generic - Generic store PCI bus info
 *  @hw: pointer to hardware structure
 *  @link_status: the link status returned by the PCI config space
 *
 *  Stores the PCI bus info (speed, width, type) within the yusur2_hw structure
 **/
void yusur2_set_pci_config_data_generic(struct yusur2_hw *hw, u16 link_status)
{
	struct yusur2_mac_info *mac = &hw->mac;

	if (hw->bus.type == yusur2_bus_type_unknown)
		hw->bus.type = yusur2_bus_type_pci_express;

	switch (link_status & YUSUR2_PCI_LINK_WIDTH) {
	case YUSUR2_PCI_LINK_WIDTH_1:
		hw->bus.width = yusur2_bus_width_pcie_x1;
		break;
	case YUSUR2_PCI_LINK_WIDTH_2:
		hw->bus.width = yusur2_bus_width_pcie_x2;
		break;
	case YUSUR2_PCI_LINK_WIDTH_4:
		hw->bus.width = yusur2_bus_width_pcie_x4;
		break;
	case YUSUR2_PCI_LINK_WIDTH_8:
		hw->bus.width = yusur2_bus_width_pcie_x8;
		break;
	default:
		hw->bus.width = yusur2_bus_width_unknown;
		break;
	}

	switch (link_status & YUSUR2_PCI_LINK_SPEED) {
	case YUSUR2_PCI_LINK_SPEED_2500:
		hw->bus.speed = yusur2_bus_speed_2500;
		break;
	case YUSUR2_PCI_LINK_SPEED_5000:
		hw->bus.speed = yusur2_bus_speed_5000;
		break;
	case YUSUR2_PCI_LINK_SPEED_8000:
		hw->bus.speed = yusur2_bus_speed_8000;
		break;
	default:
		hw->bus.speed = yusur2_bus_speed_unknown;
		break;
	}

	mac->ops.set_lan_id(hw);
}

/**
 *  yusur2_get_bus_info_generic - Generic set PCI bus info
 *  @hw: pointer to hardware structure
 *
 *  Gets the PCI bus info (speed, width, type) then calls helper function to
 *  store this data within the yusur2_hw structure.
 **/
s32 yusur2_get_bus_info_generic(struct yusur2_hw *hw)
{
	u16 link_status;

	DEBUGFUNC("yusur2_get_bus_info_generic");

	/* Get the negotiated link width and speed from PCI config space */
	link_status = YUSUR2_READ_PCIE_WORD(hw, YUSUR2_PCI_LINK_STATUS);

	yusur2_set_pci_config_data_generic(hw, link_status);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_lan_id_multi_port_pcie - Set LAN id for PCIe multiple port devices
 *  @hw: pointer to the HW structure
 *
 *  Determines the LAN function id by reading memory-mapped registers and swaps
 *  the port value if requested, and set MAC instance for devices that share
 *  CS4227.
 **/
void yusur2_set_lan_id_multi_port_pcie(struct yusur2_hw *hw)
{
//TODO:
#if 0
	struct yusur2_bus_info *bus = &hw->bus;
	u32 reg;
	u16 ee_ctrl_4;

	DEBUGFUNC("yusur2_set_lan_id_multi_port_pcie");

	reg = YUSUR2_READ_REG(hw, YUSUR2_STATUS);
	bus->func = (reg & YUSUR2_STATUS_LAN_ID) >> YUSUR2_STATUS_LAN_ID_SHIFT;
	bus->lan_id = (u8)bus->func;

	/* check for a port swap */
	reg = YUSUR2_READ_REG(hw, YUSUR2_FACTPS_BY_MAC(hw));
	if (reg & YUSUR2_FACTPS_LFS)
		bus->func ^= 0x1;

	/* Get MAC instance from EEPROM for configuring CS4227 */
	if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_SFP) {
		hw->eeprom.ops.read(hw, YUSUR2_EEPROM_CTRL_4, &ee_ctrl_4);
		bus->instance_id = (ee_ctrl_4 & YUSUR2_EE_CTRL_4_INST_ID) >>
				   YUSUR2_EE_CTRL_4_INST_ID_SHIFT;
	}
#endif
}

/**
 *  yusur2_stop_adapter_generic - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within yusur2_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
s32 yusur2_stop_adapter_generic(struct yusur2_hw *hw)
{
	u32 reg_val;
	u16 i;

	DEBUGFUNC("yusur2_stop_adapter_generic");

	/*
	 * Set the adapter_stopped flag so other driver functions stop touching
	 * the hardware
	 */
	hw->adapter_stopped = true;

	/* Disable the receive unit */
	yusur2_disable_rx(hw);

	/* Clear interrupt mask to stop interrupts from being generated */
	YUSUR2_WRITE_REG(hw, YUSUR2_EIMC, YUSUR2_IRQ_CLEAR_MASK);

	/* Clear any pending interrupts, flush previous writes */
	YUSUR2_READ_REG(hw, YUSUR2_EICR);

	/* Disable the transmit unit.  Each queue must be disabled. */
	for (i = 0; i < hw->mac.max_tx_queues; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_TXDCTL(i), YUSUR2_TXDCTL_SWFLSH);

	/* Disable the receive unit by stopping each queue */
	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		reg_val = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(i));
		reg_val &= ~YUSUR2_RXDCTL_ENABLE;
		reg_val |= YUSUR2_RXDCTL_SWFLSH;
		YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(i), reg_val);
	}

	/* flush all queues disables */
	YUSUR2_WRITE_FLUSH(hw);
	msec_delay(2);

	/*
	 * Prevent the PCI-E bus from hanging by disabling PCI-E master
	 * access and verify no pending requests
	 */
	return yusur2_disable_pcie_master(hw);
}

/**
 *  yusur2_init_led_link_act_generic - Store the LED index link/activity.
 *  @hw: pointer to hardware structure
 *
 *  Store the index for the link active LED. This will be used to support
 *  blinking the LED.
 **/
s32 yusur2_init_led_link_act_generic(struct yusur2_hw *hw)
{
//TODO:
#if 0
	struct yusur2_mac_info *mac = &hw->mac;
	u32 led_reg, led_mode;
	u8 i;

	led_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);

	/* Get LED link active from the LEDCTL register */
	for (i = 0; i < 4; i++) {
		led_mode = led_reg >> YUSUR2_LED_MODE_SHIFT(i);

		if ((led_mode & YUSUR2_LED_MODE_MASK_BASE) ==
		     YUSUR2_LED_LINK_ACTIVE) {
			mac->led_link_act = i;
			return YUSUR2_SUCCESS;
		}
	}

	/*
	 * If LEDCTL register does not have the LED link active set, then use
	 * known MAC defaults.
	 */
	switch (hw->mac.type) {
	case yusur2_mac_X550EM_a:
	case yusur2_mac_X550EM_x:
		mac->led_link_act = 1;
		break;
	default:
		mac->led_link_act = 2;
	}
#endif
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_led_on_generic - Turns on the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn on
 **/
s32 yusur2_led_on_generic(struct yusur2_hw *hw, u32 index)
{
	u32 led_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);

	DEBUGFUNC("yusur2_led_on_generic");

	if (index > 3)
		return YUSUR2_ERR_PARAM;

	/* To turn on the LED, set mode to ON. */
	led_reg &= ~YUSUR2_LED_MODE_MASK(index);
	led_reg |= YUSUR2_LED_ON << YUSUR2_LED_MODE_SHIFT(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, led_reg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_led_off_generic - Turns off the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn off
 **/
s32 yusur2_led_off_generic(struct yusur2_hw *hw, u32 index)
{
	u32 led_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);

	DEBUGFUNC("yusur2_led_off_generic");

	if (index > 3)
		return YUSUR2_ERR_PARAM;

	/* To turn off the LED, set mode to OFF. */
	led_reg &= ~YUSUR2_LED_MODE_MASK(index);
	led_reg |= YUSUR2_LED_OFF << YUSUR2_LED_MODE_SHIFT(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, led_reg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_eeprom_params_generic - Initialize EEPROM params
 *  @hw: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters yusur2_eeprom_info within the
 *  yusur2_hw struct in order to set up EEPROM access.
 **/
s32 yusur2_init_eeprom_params_generic(struct yusur2_hw *hw)
{
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	u32 eec;
	u16 eeprom_size;

	DEBUGFUNC("yusur2_init_eeprom_params_generic");

	if (eeprom->type == yusur2_eeprom_uninitialized) {
		eeprom->type = yusur2_eeprom_none;
		/* Set default semaphore delay to 10ms which is a well
		 * tested value */
		eeprom->semaphore_delay = 10;
		/* Clear EEPROM page size, it will be initialized as needed */
		eeprom->word_page_size = 0;

		/*
		 * Check for EEPROM present first.
		 * If not present leave as none
		 */
		eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));
		if (eec & YUSUR2_EEC_PRES) {
			eeprom->type = yusur2_eeprom_spi;

			/*
			 * SPI EEPROM is assumed here.  This code would need to
			 * change if a future EEPROM is not SPI.
			 */
			eeprom_size = (u16)((eec & YUSUR2_EEC_SIZE) >>
					    YUSUR2_EEC_SIZE_SHIFT);
			eeprom->word_size = 1 << (eeprom_size +
					     YUSUR2_EEPROM_WORD_SIZE_SHIFT);
		}

		if (eec & YUSUR2_EEC_ADDR_SIZE)
			eeprom->address_bits = 16;
		else
			eeprom->address_bits = 8;
		DEBUGOUT3("Eeprom params: type = %d, size = %d, address bits: "
			  "%d\n", eeprom->type, eeprom->word_size,
			  eeprom->address_bits);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_write_eeprom_buffer_bit_bang_generic - Write EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to write
 *  @words: number of word(s)
 *  @data: 16 bit word(s) to write to EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 yusur2_write_eeprom_buffer_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
					       u16 words, u16 *data)
{
	s32 status = YUSUR2_SUCCESS;
	u16 i, count;

	DEBUGFUNC("yusur2_write_eeprom_buffer_bit_bang_generic");

	hw->eeprom.ops.init_params(hw);

	if (words == 0) {
		status = YUSUR2_ERR_INVALID_ARGUMENT;
		goto out;
	}

	if (offset + words > hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		goto out;
	}

	/*
	 * The EEPROM page size cannot be queried from the chip. We do lazy
	 * initialization. It is worth to do that when we write large buffer.
	 */
	if ((hw->eeprom.word_page_size == 0) &&
	    (words > YUSUR2_EEPROM_PAGE_SIZE_MAX))
		yusur2_detect_eeprom_page_size_generic(hw, offset);

	/*
	 * We cannot hold synchronization semaphores for too long
	 * to avoid other entity starvation. However it is more efficient
	 * to read in bursts than synchronizing access for each word.
	 */
	for (i = 0; i < words; i += YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT) {
		count = (words - i) / YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT > 0 ?
			YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT : (words - i);
		status = yusur2_write_eeprom_buffer_bit_bang(hw, offset + i,
							    count, &data[i]);

		if (status != YUSUR2_SUCCESS)
			break;
	}

out:
	return status;
}

/**
 *  yusur2_write_eeprom_buffer_bit_bang - Writes 16 bit word(s) to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @words: number of word(s)
 *  @data: 16 bit word(s) to be written to the EEPROM
 *
 *  If yusur2_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
STATIC s32 yusur2_write_eeprom_buffer_bit_bang(struct yusur2_hw *hw, u16 offset,
					      u16 words, u16 *data)
{
	s32 status;
	u16 word;
	u16 page_size;
	u16 i;
	u8 write_opcode = YUSUR2_EEPROM_WRITE_OPCODE_SPI;

	DEBUGFUNC("yusur2_write_eeprom_buffer_bit_bang");

	/* Prepare the EEPROM for writing  */
	status = yusur2_acquire_eeprom(hw);

	if (status == YUSUR2_SUCCESS) {
		if (yusur2_ready_eeprom(hw) != YUSUR2_SUCCESS) {
			yusur2_release_eeprom(hw);
			status = YUSUR2_ERR_EEPROM;
		}
	}

	if (status == YUSUR2_SUCCESS) {
		for (i = 0; i < words; i++) {
			yusur2_standby_eeprom(hw);

			/*  Send the WRITE ENABLE command (8 bit opcode )  */
			yusur2_shift_out_eeprom_bits(hw,
						   YUSUR2_EEPROM_WREN_OPCODE_SPI,
						   YUSUR2_EEPROM_OPCODE_BITS);

			yusur2_standby_eeprom(hw);

			/*
			 * Some SPI eeproms use the 8th address bit embedded
			 * in the opcode
			 */
			if ((hw->eeprom.address_bits == 8) &&
			    ((offset + i) >= 128))
				write_opcode |= YUSUR2_EEPROM_A8_OPCODE_SPI;

			/* Send the Write command (8-bit opcode + addr) */
			yusur2_shift_out_eeprom_bits(hw, write_opcode,
						    YUSUR2_EEPROM_OPCODE_BITS);
			yusur2_shift_out_eeprom_bits(hw, (u16)((offset + i) * 2),
						    hw->eeprom.address_bits);

			page_size = hw->eeprom.word_page_size;

			/* Send the data in burst via SPI*/
			do {
				word = data[i];
				word = (word >> 8) | (word << 8);
				yusur2_shift_out_eeprom_bits(hw, word, 16);

				if (page_size == 0)
					break;

				/* do not wrap around page */
				if (((offset + i) & (page_size - 1)) ==
				    (page_size - 1))
					break;
			} while (++i < words);

			yusur2_standby_eeprom(hw);
			msec_delay(10);
		}
		/* Done with writing - release the EEPROM */
		yusur2_release_eeprom(hw);
	}

	return status;
}

/**
 *  yusur2_write_eeprom_generic - Writes 16 bit value to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @data: 16 bit word to be written to the EEPROM
 *
 *  If yusur2_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
s32 yusur2_write_eeprom_generic(struct yusur2_hw *hw, u16 offset, u16 data)
{
	s32 status;

	DEBUGFUNC("yusur2_write_eeprom_generic");

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		goto out;
	}

	status = yusur2_write_eeprom_buffer_bit_bang(hw, offset, 1, &data);

out:
	return status;
}

/**
 *  yusur2_read_eeprom_buffer_bit_bang_generic - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @data: read 16 bit words(s) from EEPROM
 *  @words: number of word(s)
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 yusur2_read_eeprom_buffer_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
					      u16 words, u16 *data)
{
	s32 status = YUSUR2_SUCCESS;
	u16 i, count;

	DEBUGFUNC("yusur2_read_eeprom_buffer_bit_bang_generic");

	hw->eeprom.ops.init_params(hw);

	if (words == 0) {
		status = YUSUR2_ERR_INVALID_ARGUMENT;
		goto out;
	}

	if (offset + words > hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		goto out;
	}

	/*
	 * We cannot hold synchronization semaphores for too long
	 * to avoid other entity starvation. However it is more efficient
	 * to read in bursts than synchronizing access for each word.
	 */
	for (i = 0; i < words; i += YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT) {
		count = (words - i) / YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT > 0 ?
			YUSUR2_EEPROM_RD_BUFFER_MAX_COUNT : (words - i);

		status = yusur2_read_eeprom_buffer_bit_bang(hw, offset + i,
							   count, &data[i]);

		if (status != YUSUR2_SUCCESS)
			break;
	}

out:
	return status;
}

/**
 *  yusur2_read_eeprom_buffer_bit_bang - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit word(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
STATIC s32 yusur2_read_eeprom_buffer_bit_bang(struct yusur2_hw *hw, u16 offset,
					     u16 words, u16 *data)
{
	s32 status;
	u16 word_in;
	u8 read_opcode = YUSUR2_EEPROM_READ_OPCODE_SPI;
	u16 i;

	DEBUGFUNC("yusur2_read_eeprom_buffer_bit_bang");

	/* Prepare the EEPROM for reading  */
	status = yusur2_acquire_eeprom(hw);

	if (status == YUSUR2_SUCCESS) {
		if (yusur2_ready_eeprom(hw) != YUSUR2_SUCCESS) {
			yusur2_release_eeprom(hw);
			status = YUSUR2_ERR_EEPROM;
		}
	}

	if (status == YUSUR2_SUCCESS) {
		for (i = 0; i < words; i++) {
			yusur2_standby_eeprom(hw);
			/*
			 * Some SPI eeproms use the 8th address bit embedded
			 * in the opcode
			 */
			if ((hw->eeprom.address_bits == 8) &&
			    ((offset + i) >= 128))
				read_opcode |= YUSUR2_EEPROM_A8_OPCODE_SPI;

			/* Send the READ command (opcode + addr) */
			yusur2_shift_out_eeprom_bits(hw, read_opcode,
						    YUSUR2_EEPROM_OPCODE_BITS);
			yusur2_shift_out_eeprom_bits(hw, (u16)((offset + i) * 2),
						    hw->eeprom.address_bits);

			/* Read the data. */
			word_in = yusur2_shift_in_eeprom_bits(hw, 16);
			data[i] = (word_in >> 8) | (word_in << 8);
		}

		/* End this read operation */
		yusur2_release_eeprom(hw);
	}

	return status;
}

/**
 *  yusur2_read_eeprom_bit_bang_generic - Read EEPROM word using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @data: read 16 bit value from EEPROM
 *
 *  Reads 16 bit value from EEPROM through bit-bang method
 **/
s32 yusur2_read_eeprom_bit_bang_generic(struct yusur2_hw *hw, u16 offset,
				       u16 *data)
{
	s32 status;

	DEBUGFUNC("yusur2_read_eeprom_bit_bang_generic");

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		goto out;
	}

	status = yusur2_read_eeprom_buffer_bit_bang(hw, offset, 1, data);

out:
	return status;
}

/**
 *  yusur2_read_eerd_buffer_generic - Read EEPROM word(s) using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of word(s)
 *  @data: 16 bit word(s) from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the EERD register.
 **/
s32 yusur2_read_eerd_buffer_generic(struct yusur2_hw *hw, u16 offset,
				   u16 words, u16 *data)
{
	u32 eerd;
	s32 status = YUSUR2_SUCCESS;
	u32 i;

	DEBUGFUNC("yusur2_read_eerd_buffer_generic");

	hw->eeprom.ops.init_params(hw);

	if (words == 0) {
		status = YUSUR2_ERR_INVALID_ARGUMENT;
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT, "Invalid EEPROM words");
		goto out;
	}

	if (offset >= hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT, "Invalid EEPROM offset");
		goto out;
	}

	for (i = 0; i < words; i++) {
		eerd = ((offset + i) << YUSUR2_EEPROM_RW_ADDR_SHIFT) |
		       YUSUR2_EEPROM_RW_REG_START;

		YUSUR2_WRITE_REG(hw, YUSUR2_EERD, eerd);
		status = yusur2_poll_eerd_eewr_done(hw, YUSUR2_NVM_POLL_READ);

		if (status == YUSUR2_SUCCESS) {
			data[i] = (YUSUR2_READ_REG(hw, YUSUR2_EERD) >>
				   YUSUR2_EEPROM_RW_REG_DATA);
		} else {
			DEBUGOUT("Eeprom read timed out\n");
			goto out;
		}
	}
out:
	return status;
}

/**
 *  yusur2_detect_eeprom_page_size_generic - Detect EEPROM page size
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be used as a scratch pad
 *
 *  Discover EEPROM page size by writing marching data at given offset.
 *  This function is called only when we are writing a new large buffer
 *  at given offset so the data would be overwritten anyway.
 **/
STATIC s32 yusur2_detect_eeprom_page_size_generic(struct yusur2_hw *hw,
						 u16 offset)
{
	u16 data[YUSUR2_EEPROM_PAGE_SIZE_MAX];
	s32 status = YUSUR2_SUCCESS;
	u16 i;

	DEBUGFUNC("yusur2_detect_eeprom_page_size_generic");

	for (i = 0; i < YUSUR2_EEPROM_PAGE_SIZE_MAX; i++)
		data[i] = i;

	hw->eeprom.word_page_size = YUSUR2_EEPROM_PAGE_SIZE_MAX;
	status = yusur2_write_eeprom_buffer_bit_bang(hw, offset,
					     YUSUR2_EEPROM_PAGE_SIZE_MAX, data);
	hw->eeprom.word_page_size = 0;
	if (status != YUSUR2_SUCCESS)
		goto out;

	status = yusur2_read_eeprom_buffer_bit_bang(hw, offset, 1, data);
	if (status != YUSUR2_SUCCESS)
		goto out;

	/*
	 * When writing in burst more than the actual page size
	 * EEPROM address wraps around current page.
	 */
	hw->eeprom.word_page_size = YUSUR2_EEPROM_PAGE_SIZE_MAX - data[0];

	DEBUGOUT1("Detected EEPROM page size = %d words.",
		  hw->eeprom.word_page_size);
out:
	return status;
}

/**
 *  yusur2_read_eerd_generic - Read EEPROM word using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 yusur2_read_eerd_generic(struct yusur2_hw *hw, u16 offset, u16 *data)
{
	return yusur2_read_eerd_buffer_generic(hw, offset, 1, data);
}

/**
 *  yusur2_write_eewr_buffer_generic - Write EEPROM word(s) using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @words: number of word(s)
 *  @data: word(s) write to the EEPROM
 *
 *  Write a 16 bit word(s) to the EEPROM using the EEWR register.
 **/
s32 yusur2_write_eewr_buffer_generic(struct yusur2_hw *hw, u16 offset,
				    u16 words, u16 *data)
{
	u32 eewr;
	s32 status = YUSUR2_SUCCESS;
	u16 i;

	DEBUGFUNC("yusur2_write_eewr_generic");

	hw->eeprom.ops.init_params(hw);

	if (words == 0) {
		status = YUSUR2_ERR_INVALID_ARGUMENT;
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT, "Invalid EEPROM words");
		goto out;
	}

	if (offset >= hw->eeprom.word_size) {
		status = YUSUR2_ERR_EEPROM;
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT, "Invalid EEPROM offset");
		goto out;
	}

	for (i = 0; i < words; i++) {
		eewr = ((offset + i) << YUSUR2_EEPROM_RW_ADDR_SHIFT) |
			(data[i] << YUSUR2_EEPROM_RW_REG_DATA) |
			YUSUR2_EEPROM_RW_REG_START;

		status = yusur2_poll_eerd_eewr_done(hw, YUSUR2_NVM_POLL_WRITE);
		if (status != YUSUR2_SUCCESS) {
			DEBUGOUT("Eeprom write EEWR timed out\n");
			goto out;
		}

		YUSUR2_WRITE_REG(hw, YUSUR2_EEWR, eewr);

		status = yusur2_poll_eerd_eewr_done(hw, YUSUR2_NVM_POLL_WRITE);
		if (status != YUSUR2_SUCCESS) {
			DEBUGOUT("Eeprom write EEWR timed out\n");
			goto out;
		}
	}

out:
	return status;
}

/**
 *  yusur2_write_eewr_generic - Write EEPROM word using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @data: word write to the EEPROM
 *
 *  Write a 16 bit word to the EEPROM using the EEWR register.
 **/
s32 yusur2_write_eewr_generic(struct yusur2_hw *hw, u16 offset, u16 data)
{
	return yusur2_write_eewr_buffer_generic(hw, offset, 1, &data);
}

/**
 *  yusur2_poll_eerd_eewr_done - Poll EERD read or EEWR write status
 *  @hw: pointer to hardware structure
 *  @ee_reg: EEPROM flag for polling
 *
 *  Polls the status bit (bit 1) of the EERD or EEWR to determine when the
 *  read or write is done respectively.
 **/
s32 yusur2_poll_eerd_eewr_done(struct yusur2_hw *hw, u32 ee_reg)
{
	u32 i;
	u32 reg;
	s32 status = YUSUR2_ERR_EEPROM;

	DEBUGFUNC("yusur2_poll_eerd_eewr_done");

	for (i = 0; i < YUSUR2_EERD_EEWR_ATTEMPTS; i++) {
		if (ee_reg == YUSUR2_NVM_POLL_READ)
			reg = YUSUR2_READ_REG(hw, YUSUR2_EERD);
		else
			reg = YUSUR2_READ_REG(hw, YUSUR2_EEWR);

		if (reg & YUSUR2_EEPROM_RW_REG_DONE) {
			status = YUSUR2_SUCCESS;
			break;
		}
		usec_delay(5);
	}

	if (i == YUSUR2_EERD_EEWR_ATTEMPTS)
		ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			     "EEPROM read/write done polling timed out");

	return status;
}

/**
 *  yusur2_acquire_eeprom - Acquire EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *
 *  Prepares EEPROM for access using bit-bang method. This function should
 *  be called before issuing a command to the EEPROM.
 **/
STATIC s32 yusur2_acquire_eeprom(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;
	u32 eec;
	u32 i;

	DEBUGFUNC("yusur2_acquire_eeprom");

	if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_EEP_SM)
	    != YUSUR2_SUCCESS)
		status = YUSUR2_ERR_SWFW_SYNC;

	if (status == YUSUR2_SUCCESS) {
		eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

		/* Request EEPROM Access */
		eec |= YUSUR2_EEC_REQ;
		YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);

		for (i = 0; i < YUSUR2_EEPROM_GRANT_ATTEMPTS; i++) {
			eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));
			if (eec & YUSUR2_EEC_GNT)
				break;
			usec_delay(5);
		}

		/* Release if grant not acquired */
		if (!(eec & YUSUR2_EEC_GNT)) {
			eec &= ~YUSUR2_EEC_REQ;
			YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
			DEBUGOUT("Could not acquire EEPROM grant\n");

			hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);
			status = YUSUR2_ERR_EEPROM;
		}

		/* Setup EEPROM for Read/Write */
		if (status == YUSUR2_SUCCESS) {
			/* Clear CS and SK */
			eec &= ~(YUSUR2_EEC_CS | YUSUR2_EEC_SK);
			YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
			YUSUR2_WRITE_FLUSH(hw);
			usec_delay(1);
		}
	}
	return status;
}

/**
 *  yusur2_get_eeprom_semaphore - Get hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  Sets the hardware semaphores so EEPROM access can occur for bit-bang method
 **/
STATIC s32 yusur2_get_eeprom_semaphore(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_ERR_EEPROM;
	u32 timeout = 2000;
	u32 i;
	u32 swsm;

	DEBUGFUNC("yusur2_get_eeprom_semaphore");


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

	if (i == timeout) {
		DEBUGOUT("Driver can't access the Eeprom - SMBI Semaphore "
			 "not granted.\n");
		/*
		 * this release is particularly important because our attempts
		 * above to get the semaphore may have succeeded, and if there
		 * was a timeout, we should unconditionally clear the semaphore
		 * bits to free the driver to make progress
		 */
		yusur2_release_eeprom_semaphore(hw);

		usec_delay(50);
		/*
		 * one last try
		 * If the SMBI bit is 0 when we read it, then the bit will be
		 * set and we have the semaphore
		 */
		swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM_BY_MAC(hw));
		if (!(swsm & YUSUR2_SWSM_SMBI))
			status = YUSUR2_SUCCESS;
	}

	/* Now get the semaphore between SW/FW through the SWESMBI bit */
	if (status == YUSUR2_SUCCESS) {
		for (i = 0; i < timeout; i++) {
			swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM_BY_MAC(hw));

			/* Set the SW EEPROM semaphore bit to request access */
			swsm |= YUSUR2_SWSM_SWESMBI;
			YUSUR2_WRITE_REG(hw, YUSUR2_SWSM_BY_MAC(hw), swsm);

			/*
			 * If we set the bit successfully then we got the
			 * semaphore.
			 */
			swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM_BY_MAC(hw));
			if (swsm & YUSUR2_SWSM_SWESMBI)
				break;

			usec_delay(50);
		}

		/*
		 * Release semaphores and return error if SW EEPROM semaphore
		 * was not granted because we don't have access to the EEPROM
		 */
		if (i >= timeout) {
			ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			    "SWESMBI Software EEPROM semaphore not granted.\n");
			yusur2_release_eeprom_semaphore(hw);
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
 *  yusur2_release_eeprom_semaphore - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function clears hardware semaphore bits.
 **/
STATIC void yusur2_release_eeprom_semaphore(struct yusur2_hw *hw)
{
	u32 swsm;

	DEBUGFUNC("yusur2_release_eeprom_semaphore");

	swsm = YUSUR2_READ_REG(hw, YUSUR2_SWSM);

	/* Release both semaphores by writing 0 to the bits SWESMBI and SMBI */
	swsm &= ~(YUSUR2_SWSM_SWESMBI | YUSUR2_SWSM_SMBI);
	YUSUR2_WRITE_REG(hw, YUSUR2_SWSM, swsm);
	YUSUR2_WRITE_FLUSH(hw);
}

/**
 *  yusur2_ready_eeprom - Polls for EEPROM ready
 *  @hw: pointer to hardware structure
 **/
STATIC s32 yusur2_ready_eeprom(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;
	u16 i;
	u8 spi_stat_reg;

	DEBUGFUNC("yusur2_ready_eeprom");

	/*
	 * Read "Status Register" repeatedly until the LSB is cleared.  The
	 * EEPROM will signal that the command has been completed by clearing
	 * bit 0 of the internal status register.  If it's not cleared within
	 * 5 milliseconds, then error out.
	 */
	for (i = 0; i < YUSUR2_EEPROM_MAX_RETRY_SPI; i += 5) {
		yusur2_shift_out_eeprom_bits(hw, YUSUR2_EEPROM_RDSR_OPCODE_SPI,
					    YUSUR2_EEPROM_OPCODE_BITS);
		spi_stat_reg = (u8)yusur2_shift_in_eeprom_bits(hw, 8);
		if (!(spi_stat_reg & YUSUR2_EEPROM_STATUS_RDY_SPI))
			break;

		usec_delay(5);
		yusur2_standby_eeprom(hw);
	};

	/*
	 * On some parts, SPI write time could vary from 0-20mSec on 3.3V
	 * devices (and only 0-5mSec on 5V devices)
	 */
	if (i >= YUSUR2_EEPROM_MAX_RETRY_SPI) {
		DEBUGOUT("SPI EEPROM Status error\n");
		status = YUSUR2_ERR_EEPROM;
	}

	return status;
}

/**
 *  yusur2_standby_eeprom - Returns EEPROM to a "standby" state
 *  @hw: pointer to hardware structure
 **/
STATIC void yusur2_standby_eeprom(struct yusur2_hw *hw)
{
	u32 eec;

	DEBUGFUNC("yusur2_standby_eeprom");

	eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

	/* Toggle CS to flush commands */
	eec |= YUSUR2_EEC_CS;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(1);
	eec &= ~YUSUR2_EEC_CS;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  yusur2_shift_out_eeprom_bits - Shift data bits out to the EEPROM.
 *  @hw: pointer to hardware structure
 *  @data: data to send to the EEPROM
 *  @count: number of bits to shift out
 **/
STATIC void yusur2_shift_out_eeprom_bits(struct yusur2_hw *hw, u16 data,
					u16 count)
{
	u32 eec;
	u32 mask;
	u32 i;

	DEBUGFUNC("yusur2_shift_out_eeprom_bits");

	eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

	/*
	 * Mask is used to shift "count" bits of "data" out to the EEPROM
	 * one bit at a time.  Determine the starting bit based on count
	 */
	mask = 0x01 << (count - 1);

	for (i = 0; i < count; i++) {
		/*
		 * A "1" is shifted out to the EEPROM by setting bit "DI" to a
		 * "1", and then raising and then lowering the clock (the SK
		 * bit controls the clock input to the EEPROM).  A "0" is
		 * shifted out to the EEPROM by setting "DI" to "0" and then
		 * raising and then lowering the clock.
		 */
		if (data & mask)
			eec |= YUSUR2_EEC_DI;
		else
			eec &= ~YUSUR2_EEC_DI;

		YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
		YUSUR2_WRITE_FLUSH(hw);

		usec_delay(1);

		yusur2_raise_eeprom_clk(hw, &eec);
		yusur2_lower_eeprom_clk(hw, &eec);

		/*
		 * Shift mask to signify next bit of data to shift in to the
		 * EEPROM
		 */
		mask = mask >> 1;
	};

	/* We leave the "DI" bit set to "0" when we leave this routine. */
	eec &= ~YUSUR2_EEC_DI;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
	YUSUR2_WRITE_FLUSH(hw);
}

/**
 *  yusur2_shift_in_eeprom_bits - Shift data bits in from the EEPROM
 *  @hw: pointer to hardware structure
 *  @count: number of bits to shift
 **/
STATIC u16 yusur2_shift_in_eeprom_bits(struct yusur2_hw *hw, u16 count)
{
	u32 eec;
	u32 i;
	u16 data = 0;

	DEBUGFUNC("yusur2_shift_in_eeprom_bits");

	/*
	 * In order to read a register from the EEPROM, we need to shift
	 * 'count' bits in from the EEPROM. Bits are "shifted in" by raising
	 * the clock input to the EEPROM (setting the SK bit), and then reading
	 * the value of the "DO" bit.  During this "shifting in" process the
	 * "DI" bit should always be clear.
	 */
	eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

	eec &= ~(YUSUR2_EEC_DO | YUSUR2_EEC_DI);

	for (i = 0; i < count; i++) {
		data = data << 1;
		yusur2_raise_eeprom_clk(hw, &eec);

		eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

		eec &= ~(YUSUR2_EEC_DI);
		if (eec & YUSUR2_EEC_DO)
			data |= 1;

		yusur2_lower_eeprom_clk(hw, &eec);
	}

	return data;
}

/**
 *  yusur2_raise_eeprom_clk - Raises the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eec: EEC register's current value
 **/
STATIC void yusur2_raise_eeprom_clk(struct yusur2_hw *hw, u32 *eec)
{
	DEBUGFUNC("yusur2_raise_eeprom_clk");

	/*
	 * Raise the clock input to the EEPROM
	 * (setting the SK bit), then delay
	 */
	*eec = *eec | YUSUR2_EEC_SK;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), *eec);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  yusur2_lower_eeprom_clk - Lowers the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eec: EEC's current value
 **/
STATIC void yusur2_lower_eeprom_clk(struct yusur2_hw *hw, u32 *eec)
{
	DEBUGFUNC("yusur2_lower_eeprom_clk");

	/*
	 * Lower the clock input to the EEPROM (clearing the SK bit), then
	 * delay
	 */
	*eec = *eec & ~YUSUR2_EEC_SK;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), *eec);
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  yusur2_release_eeprom - Release EEPROM, release semaphores
 *  @hw: pointer to hardware structure
 **/
STATIC void yusur2_release_eeprom(struct yusur2_hw *hw)
{
	u32 eec;

	DEBUGFUNC("yusur2_release_eeprom");

	eec = YUSUR2_READ_REG(hw, YUSUR2_EEC_BY_MAC(hw));

	eec |= YUSUR2_EEC_CS;  /* Pull CS high */
	eec &= ~YUSUR2_EEC_SK; /* Lower SCK */

	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);
	YUSUR2_WRITE_FLUSH(hw);

	usec_delay(1);

	/* Stop requesting EEPROM access */
	eec &= ~YUSUR2_EEC_REQ;
	YUSUR2_WRITE_REG(hw, YUSUR2_EEC_BY_MAC(hw), eec);

	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_EEP_SM);

	/* Delay before attempt to obtain semaphore again to allow FW access */
	msec_delay(hw->eeprom.semaphore_delay);
}

/**
 *  yusur2_calc_eeprom_checksum_generic - Calculates and returns the checksum
 *  @hw: pointer to hardware structure
 *
 *  Returns a negative error code on error, or the 16-bit checksum
 **/
s32 yusur2_calc_eeprom_checksum_generic(struct yusur2_hw *hw)
{
	u16 i;
	u16 j;
	u16 checksum = 0;
	u16 length = 0;
	u16 pointer = 0;
	u16 word = 0;

	DEBUGFUNC("yusur2_calc_eeprom_checksum_generic");

	/* Include 0x0-0x3F in the checksum */
	for (i = 0; i < YUSUR2_EEPROM_CHECKSUM; i++) {
		if (hw->eeprom.ops.read(hw, i, &word)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}
		checksum += word;
	}

	/* Include all data from pointers except for the fw pointer */
	for (i = YUSUR2_PCIE_ANALOG_PTR; i < YUSUR2_FW_PTR; i++) {
		if (hw->eeprom.ops.read(hw, i, &pointer)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}

		/* If the pointer seems invalid */
		if (pointer == 0xFFFF || pointer == 0)
			continue;

		if (hw->eeprom.ops.read(hw, pointer, &length)) {
			DEBUGOUT("EEPROM read failed\n");
			return YUSUR2_ERR_EEPROM;
		}

		if (length == 0xFFFF || length == 0)
			continue;

		for (j = pointer + 1; j <= pointer + length; j++) {
			if (hw->eeprom.ops.read(hw, j, &word)) {
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
 *  yusur2_validate_eeprom_checksum_generic - Validate EEPROM checksum
 *  @hw: pointer to hardware structure
 *  @checksum_val: calculated checksum
 *
 *  Performs checksum calculation and validates the EEPROM checksum.  If the
 *  caller does not need checksum_val, the value can be NULL.
 **/
s32 yusur2_validate_eeprom_checksum_generic(struct yusur2_hw *hw,
					   u16 *checksum_val)
{
	s32 status = YUSUR2_SUCCESS;
//TODO:
#if 0
	s32 status;
	u16 checksum;
	u16 read_checksum = 0;

	DEBUGFUNC("yusur2_validate_eeprom_checksum_generic");

	/* Read the first word from the EEPROM. If this times out or fails, do
	 * not continue or we could be in for a very long wait while every
	 * EEPROM read fails
	 */
	status = hw->eeprom.ops.read(hw, 0, &checksum);
	if (status) {
		DEBUGOUT("EEPROM read failed\n");
		return status;
	}

	status = hw->eeprom.ops.calc_checksum(hw);
	if (status < 0)
		return status;

	checksum = (u16)(status & 0xffff);

	status = hw->eeprom.ops.read(hw, YUSUR2_EEPROM_CHECKSUM, &read_checksum);
	if (status) {
		DEBUGOUT("EEPROM read failed\n");
		return status;
	}

	/* Verify read checksum from EEPROM is the same as
	 * calculated checksum
	 */
	if (read_checksum != checksum)
		status = YUSUR2_ERR_EEPROM_CHECKSUM;

	/* If the user cares, return the calculated checksum */
	if (checksum_val)
		*checksum_val = checksum;
#endif
	return status;
}

/**
 *  yusur2_update_eeprom_checksum_generic - Updates the EEPROM checksum
 *  @hw: pointer to hardware structure
 **/
s32 yusur2_update_eeprom_checksum_generic(struct yusur2_hw *hw)
{
	s32 status;
	u16 checksum;

	DEBUGFUNC("yusur2_update_eeprom_checksum_generic");

	/* Read the first word from the EEPROM. If this times out or fails, do
	 * not continue or we could be in for a very long wait while every
	 * EEPROM read fails
	 */
	status = hw->eeprom.ops.read(hw, 0, &checksum);
	if (status) {
		DEBUGOUT("EEPROM read failed\n");
		return status;
	}

	status = hw->eeprom.ops.calc_checksum(hw);
	if (status < 0)
		return status;

	checksum = (u16)(status & 0xffff);

	status = hw->eeprom.ops.write(hw, YUSUR2_EEPROM_CHECKSUM, checksum);

	return status;
}

/**
 *  yusur2_validate_mac_addr - Validate MAC address
 *  @mac_addr: pointer to MAC address.
 *
 *  Tests a MAC address to ensure it is a valid Individual Address.
 **/
s32 yusur2_validate_mac_addr(u8 *mac_addr)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_validate_mac_addr");

	/* Make sure it is not a multicast address */
	if (YUSUR2_IS_MULTICAST(mac_addr)) {
		status = YUSUR2_ERR_INVALID_MAC_ADDR;
	/* Not a broadcast address */
	} else if (YUSUR2_IS_BROADCAST(mac_addr)) {
		status = YUSUR2_ERR_INVALID_MAC_ADDR;
	/* Reject the zero address */
	} else if (mac_addr[0] == 0 && mac_addr[1] == 0 && mac_addr[2] == 0 &&
		   mac_addr[3] == 0 && mac_addr[4] == 0 && mac_addr[5] == 0) {
		status = YUSUR2_ERR_INVALID_MAC_ADDR;
	}
	return status;
}

/**
 *  yusur2_set_rar_generic - Set Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *
 *  Puts an ethernet address into a receive address register.
 **/
s32 yusur2_set_rar_generic(struct yusur2_hw *hw, u32 index, u8 *addr, u32 vmdq,
			  u32 enable_addr)
{
	u32 rar_low, rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_set_rar_generic");

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		ERROR_REPORT2(YUSUR2_ERROR_ARGUMENT,
			     "RAR index %d is out of range.\n", index);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	/* setup VMDq pool selection before this RAR gets enabled */
	hw->mac.ops.set_vmdq(hw, index, vmdq);

	/*
	 * HW expects these in little endian so we reverse the byte
	 * order from network order (big endian) to little endian
	 */
	rar_low = ((u32)addr[0] |
		   ((u32)addr[1] << 8) |
		   ((u32)addr[2] << 16) |
		   ((u32)addr[3] << 24));
	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(index));
	rar_high &= ~(0x0000FFFF | YUSUR2_RAH_AV);
	rar_high |= ((u32)addr[4] | ((u32)addr[5] << 8));

	if (enable_addr != 0)
		rar_high |= YUSUR2_RAH_AV;

	YUSUR2_WRITE_REG(hw, YUSUR2_RAL(index), rar_low);
	YUSUR2_WRITE_REG(hw, YUSUR2_RAH(index), rar_high);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_clear_rar_generic - Remove Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *
 *  Clears an ethernet address from a receive address register.
 **/
s32 yusur2_clear_rar_generic(struct yusur2_hw *hw, u32 index)
{
	u32 rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_clear_rar_generic");

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		ERROR_REPORT2(YUSUR2_ERROR_ARGUMENT,
			     "RAR index %d is out of range.\n", index);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(index));
	rar_high &= ~(0x0000FFFF | YUSUR2_RAH_AV);

	YUSUR2_WRITE_REG(hw, YUSUR2_RAL(index), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_RAH(index), rar_high);

	/* clear VMDq pool/queue selection for this RAR */
	hw->mac.ops.clear_vmdq(hw, index, YUSUR2_CLEAR_VMDQ_ALL);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_rx_addrs_generic - Initializes receive address filters.
 *  @hw: pointer to hardware structure
 *
 *  Places the MAC address in receive address register 0 and clears the rest
 *  of the receive address registers. Clears the multicast table. Assumes
 *  the receiver is in reset when the routine is called.
 **/
s32 yusur2_init_rx_addrs_generic(struct yusur2_hw *hw)
{
	u32 i;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_init_rx_addrs_generic");

	/*
	 * If the current mac address is valid, assume it is a software override
	 * to the permanent address.
	 * Otherwise, use the permanent address from the eeprom.
	 */
	if (yusur2_validate_mac_addr(hw->mac.addr) ==
	    YUSUR2_ERR_INVALID_MAC_ADDR) {
		/* Get the MAC address from the RAR0 for later reference */
		hw->mac.ops.get_mac_addr(hw, hw->mac.addr);

		DEBUGOUT3(" Keeping Current RAR0 Addr =%.2X %.2X %.2X ",
			  hw->mac.addr[0], hw->mac.addr[1],
			  hw->mac.addr[2]);
		DEBUGOUT3("%.2X %.2X %.2X\n", hw->mac.addr[3],
			  hw->mac.addr[4], hw->mac.addr[5]);
	} else {
		/* Setup the receive address. */
		DEBUGOUT("Overriding MAC Address in RAR[0]\n");
		DEBUGOUT3(" New MAC Addr =%.2X %.2X %.2X ",
			  hw->mac.addr[0], hw->mac.addr[1],
			  hw->mac.addr[2]);
		DEBUGOUT3("%.2X %.2X %.2X\n", hw->mac.addr[3],
			  hw->mac.addr[4], hw->mac.addr[5]);

		hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, YUSUR2_RAH_AV);
	}

	/* clear VMDq pool/queue selection for RAR 0 */
	hw->mac.ops.clear_vmdq(hw, 0, YUSUR2_CLEAR_VMDQ_ALL);

	hw->addr_ctrl.overflow_promisc = 0;

	hw->addr_ctrl.rar_used_count = 1;

	/* Zero out the other receive addresses. */
	DEBUGOUT1("Clearing RAR[1-%d]\n", rar_entries - 1);
	for (i = 1; i < rar_entries; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RAL(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_RAH(i), 0);
	}

	/* Clear the MTA */
	hw->addr_ctrl.mta_in_use = 0;
	YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL, hw->mac.mc_filter_type);

	DEBUGOUT(" Clearing MTA\n");
	for (i = 0; i < hw->mac.mcft_size; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_MTA(i), 0);

	yusur2_init_uta_tables(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_add_uc_addr - Adds a secondary unicast address.
 *  @hw: pointer to hardware structure
 *  @addr: new address
 *  @vmdq: VMDq "set" or "pool" index
 *
 *  Adds it to unused receive address register or goes into promiscuous mode.
 **/
void yusur2_add_uc_addr(struct yusur2_hw *hw, u8 *addr, u32 vmdq)
{
	u32 rar_entries = hw->mac.num_rar_entries;
	u32 rar;

	DEBUGFUNC("yusur2_add_uc_addr");

	DEBUGOUT6(" UC Addr = %.2X %.2X %.2X %.2X %.2X %.2X\n",
		  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	/*
	 * Place this address in the RAR if there is room,
	 * else put the controller into promiscuous mode
	 */
	if (hw->addr_ctrl.rar_used_count < rar_entries) {
		rar = hw->addr_ctrl.rar_used_count;
		hw->mac.ops.set_rar(hw, rar, addr, vmdq, YUSUR2_RAH_AV);
		DEBUGOUT1("Added a secondary address to RAR[%d]\n", rar);
		hw->addr_ctrl.rar_used_count++;
	} else {
		hw->addr_ctrl.overflow_promisc++;
	}

	DEBUGOUT("yusur2_add_uc_addr Complete\n");
}

/**
 *  yusur2_update_uc_addr_list_generic - Updates MAC list of secondary addresses
 *  @hw: pointer to hardware structure
 *  @addr_list: the list of new addresses
 *  @addr_count: number of addresses
 *  @next: iterator function to walk the address list
 *
 *  The given list replaces any existing list.  Clears the secondary addrs from
 *  receive address registers.  Uses unused receive address registers for the
 *  first secondary addresses, and falls back to promiscuous mode as needed.
 *
 *  Drivers using secondary unicast addresses must set user_set_promisc when
 *  manually putting the device into promiscuous mode.
 **/
s32 yusur2_update_uc_addr_list_generic(struct yusur2_hw *hw, u8 *addr_list,
				      u32 addr_count, yusur2_mc_addr_itr next)
{
	u8 *addr;
	u32 i;
	u32 old_promisc_setting = hw->addr_ctrl.overflow_promisc;
	u32 uc_addr_in_use;
	u32 fctrl;
	u32 vmdq;

	DEBUGFUNC("yusur2_update_uc_addr_list_generic");

	/*
	 * Clear accounting of old secondary address list,
	 * don't count RAR[0]
	 */
	uc_addr_in_use = hw->addr_ctrl.rar_used_count - 1;
	hw->addr_ctrl.rar_used_count -= uc_addr_in_use;
	hw->addr_ctrl.overflow_promisc = 0;

	/* Zero out the other receive addresses */
	DEBUGOUT1("Clearing RAR[1-%d]\n", uc_addr_in_use+1);
	for (i = 0; i < uc_addr_in_use; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RAL(1+i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_RAH(1+i), 0);
	}

	/* Add the new addresses */
	for (i = 0; i < addr_count; i++) {
		DEBUGOUT(" Adding the secondary addresses:\n");
		addr = next(hw, &addr_list, &vmdq);
		yusur2_add_uc_addr(hw, addr, vmdq);
	}

	if (hw->addr_ctrl.overflow_promisc) {
		/* enable promisc if not already in overflow or set by user */
		if (!old_promisc_setting && !hw->addr_ctrl.user_set_promisc) {
			DEBUGOUT(" Entering address overflow promisc mode\n");
			fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
			fctrl |= YUSUR2_FCTRL_UPE;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);
		}
	} else {
		/* only disable if set by overflow, not by user */
		if (old_promisc_setting && !hw->addr_ctrl.user_set_promisc) {
			DEBUGOUT(" Leaving address overflow promisc mode\n");
			fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
			fctrl &= ~YUSUR2_FCTRL_UPE;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);
		}
	}

	DEBUGOUT("yusur2_update_uc_addr_list_generic Complete\n");
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_mta_vector - Determines bit-vector in multicast table to set
 *  @hw: pointer to hardware structure
 *  @mc_addr: the multicast address
 *
 *  Extracts the 12 bits, from a multicast address, to determine which
 *  bit-vector to set in the multicast table. The hardware uses 12 bits, from
 *  incoming rx multicast addresses, to determine the bit-vector to check in
 *  the MTA. Which of the 4 combination, of 12-bits, the hardware uses is set
 *  by the MO field of the MCSTCTRL. The MO field is set during initialization
 *  to mc_filter_type.
 **/
STATIC s32 yusur2_mta_vector(struct yusur2_hw *hw, u8 *mc_addr)
{
	u32 vector = 0;

	DEBUGFUNC("yusur2_mta_vector");

	switch (hw->mac.mc_filter_type) {
	case 0:   /* use bits [47:36] of the address */
		vector = ((mc_addr[4] >> 4) | (((u16)mc_addr[5]) << 4));
		break;
	case 1:   /* use bits [46:35] of the address */
		vector = ((mc_addr[4] >> 3) | (((u16)mc_addr[5]) << 5));
		break;
	case 2:   /* use bits [45:34] of the address */
		vector = ((mc_addr[4] >> 2) | (((u16)mc_addr[5]) << 6));
		break;
	case 3:   /* use bits [43:32] of the address */
		vector = ((mc_addr[4]) | (((u16)mc_addr[5]) << 8));
		break;
	default:  /* Invalid mc_filter_type */
		DEBUGOUT("MC filter type param set incorrectly\n");
		ASSERT(0);
		break;
	}

	/* vector can only be 12-bits or boundary will be exceeded */
	vector &= 0xFFF;
	return vector;
}

/**
 *  yusur2_set_mta - Set bit-vector in multicast table
 *  @hw: pointer to hardware structure
 *  @mc_addr: Multicast address
 *
 *  Sets the bit-vector in the multicast table.
 **/
void yusur2_set_mta(struct yusur2_hw *hw, u8 *mc_addr)
{
	u32 vector;
	u32 vector_bit;
	u32 vector_reg;

	DEBUGFUNC("yusur2_set_mta");

	hw->addr_ctrl.mta_in_use++;

	vector = yusur2_mta_vector(hw, mc_addr);
	DEBUGOUT1(" bit-vector = 0x%03X\n", vector);

	/*
	 * The MTA is a register array of 128 32-bit registers. It is treated
	 * like an array of 4096 bits.  We want to set bit
	 * BitArray[vector_value]. So we figure out what register the bit is
	 * in, read it, OR in the new bit, then write back the new value.  The
	 * register is determined by the upper 7 bits of the vector value and
	 * the bit within that register are determined by the lower 5 bits of
	 * the value.
	 */
	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	hw->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}

/**
 *  yusur2_update_mc_addr_list_generic - Updates MAC list of multicast addresses
 *  @hw: pointer to hardware structure
 *  @mc_addr_list: the list of new multicast addresses
 *  @mc_addr_count: number of addresses
 *  @next: iterator function to walk the multicast address list
 *  @clear: flag, when set clears the table beforehand
 *
 *  When the clear flag is set, the given list replaces any existing list.
 *  Hashes the given addresses into the multicast table.
 **/
s32 yusur2_update_mc_addr_list_generic(struct yusur2_hw *hw, u8 *mc_addr_list,
				      u32 mc_addr_count, yusur2_mc_addr_itr next,
				      bool clear)
{
	u32 i;
	u32 vmdq;

	DEBUGFUNC("yusur2_update_mc_addr_list_generic");

	/*
	 * Set the new number of MC addresses that we are being requested to
	 * use.
	 */
	hw->addr_ctrl.num_mc_addrs = mc_addr_count;
	hw->addr_ctrl.mta_in_use = 0;

	/* Clear mta_shadow */
	if (clear) {
		DEBUGOUT(" Clearing MTA\n");
		memset(&hw->mac.mta_shadow, 0, sizeof(hw->mac.mta_shadow));
	}

	/* Update mta_shadow */
	for (i = 0; i < mc_addr_count; i++) {
		DEBUGOUT(" Adding the multicast addresses:\n");
		yusur2_set_mta(hw, next(hw, &mc_addr_list, &vmdq));
	}

	/* Enable mta */
	for (i = 0; i < hw->mac.mcft_size; i++)
		YUSUR2_WRITE_REG_ARRAY(hw, YUSUR2_MTA(0), i,
				      hw->mac.mta_shadow[i]);

	if (hw->addr_ctrl.mta_in_use > 0)
		YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL,
				YUSUR2_MCSTCTRL_MFE | hw->mac.mc_filter_type);

	DEBUGOUT("yusur2_update_mc_addr_list_generic Complete\n");
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_enable_mc_generic - Enable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Enables multicast address in RAR and the use of the multicast hash table.
 **/
s32 yusur2_enable_mc_generic(struct yusur2_hw *hw)
{
	struct yusur2_addr_filter_info *a = &hw->addr_ctrl;

	DEBUGFUNC("yusur2_enable_mc_generic");

	if (a->mta_in_use > 0)
		YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL, YUSUR2_MCSTCTRL_MFE |
				hw->mac.mc_filter_type);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_disable_mc_generic - Disable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Disables multicast address in RAR and the use of the multicast hash table.
 **/
s32 yusur2_disable_mc_generic(struct yusur2_hw *hw)
{
	struct yusur2_addr_filter_info *a = &hw->addr_ctrl;

	DEBUGFUNC("yusur2_disable_mc_generic");

	if (a->mta_in_use > 0)
		YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL, hw->mac.mc_filter_type);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_fc_enable_generic - Enable flow control
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to the current settings.
 **/
s32 yusur2_fc_enable_generic(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_SUCCESS;
	u32 mflcn_reg, fccfg_reg;
	u32 reg;
	u32 fcrtl, fcrth;
	int i;

	DEBUGFUNC("yusur2_fc_enable_generic");

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

	/* Negotiate the fc mode to use */
	hw->mac.ops.fc_autoneg(hw);

	/* Disable any previous flow control settings */
	mflcn_reg = YUSUR2_READ_REG(hw, YUSUR2_MFLCN);
	mflcn_reg &= ~(YUSUR2_MFLCN_RPFCE_MASK | YUSUR2_MFLCN_RFCE);

	fccfg_reg = YUSUR2_READ_REG(hw, YUSUR2_FCCFG);
	fccfg_reg &= ~(YUSUR2_FCCFG_TFCE_802_3X | YUSUR2_FCCFG_TFCE_PRIORITY);

	/*
	 * The possible values of fc.current_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *    we do not support receiving pause frames).
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
		mflcn_reg |= YUSUR2_MFLCN_RFCE;
		break;
	case yusur2_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		fccfg_reg |= YUSUR2_FCCFG_TFCE_802_3X;
		break;
	case yusur2_fc_full:
		/* Flow control (both Rx and Tx) is enabled by SW override. */
		mflcn_reg |= YUSUR2_MFLCN_RFCE;
		fccfg_reg |= YUSUR2_FCCFG_TFCE_802_3X;
		break;
	default:
		ERROR_REPORT1(YUSUR2_ERROR_ARGUMENT,
			     "Flow control param set incorrectly\n");
		ret_val = YUSUR2_ERR_CONFIG;
		goto out;
		break;
	}

	/* Set 802.3x based flow control settings. */
	mflcn_reg |= YUSUR2_MFLCN_DPF;
	YUSUR2_WRITE_REG(hw, YUSUR2_MFLCN, mflcn_reg);
	YUSUR2_WRITE_REG(hw, YUSUR2_FCCFG, fccfg_reg);


	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & yusur2_fc_tx_pause) &&
		    hw->fc.high_water[i]) {
			fcrtl = (hw->fc.low_water[i] << 10) | YUSUR2_FCRTL_XONE;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), fcrtl);
			fcrth = (hw->fc.high_water[i] << 10) | YUSUR2_FCRTH_FCEN;
		} else {
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), 0);
			/*
			 * In order to prevent Tx hangs when the internal Tx
			 * switch is enabled we must set the high water mark
			 * to the Rx packet buffer size - 24KB.  This allows
			 * the Tx switch to function even under heavy Rx
			 * workloads.
			 */
			fcrth = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(i)) - 24576;
		}

		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH_82599(i), fcrth);
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
 *  yusur2_negotiate_fc - Negotiate flow control
 *  @hw: pointer to hardware structure
 *  @adv_reg: flow control advertised settings
 *  @lp_reg: link partner's flow control settings
 *  @adv_sym: symmetric pause bit in advertisement
 *  @adv_asm: asymmetric pause bit in advertisement
 *  @lp_sym: symmetric pause bit in link partner advertisement
 *  @lp_asm: asymmetric pause bit in link partner advertisement
 *
 *  Find the intersection between advertised settings and link partner's
 *  advertised settings
 **/
s32 yusur2_negotiate_fc(struct yusur2_hw *hw, u32 adv_reg, u32 lp_reg,
		       u32 adv_sym, u32 adv_asm, u32 lp_sym, u32 lp_asm)
{
	if ((!(adv_reg)) ||  (!(lp_reg))) {
		ERROR_REPORT3(YUSUR2_ERROR_UNSUPPORTED,
			     "Local or link partner's advertised flow control "
			     "settings are NULL. Local: %x, link partner: %x\n",
			     adv_reg, lp_reg);
		return YUSUR2_ERR_FC_NOT_NEGOTIATED;
	}

	if ((adv_reg & adv_sym) && (lp_reg & lp_sym)) {
		/*
		 * Now we need to check if the user selected Rx ONLY
		 * of pause frames.  In this case, we had to advertise
		 * FULL flow control because we could not advertise RX
		 * ONLY. Hence, we must now check to see if we need to
		 * turn OFF the TRANSMISSION of PAUSE frames.
		 */
		if (hw->fc.requested_mode == yusur2_fc_full) {
			hw->fc.current_mode = yusur2_fc_full;
			DEBUGOUT("Flow Control = FULL.\n");
		} else {
			hw->fc.current_mode = yusur2_fc_rx_pause;
			DEBUGOUT("Flow Control=RX PAUSE frames only\n");
		}
	} else if (!(adv_reg & adv_sym) && (adv_reg & adv_asm) &&
		   (lp_reg & lp_sym) && (lp_reg & lp_asm)) {
		hw->fc.current_mode = yusur2_fc_tx_pause;
		DEBUGOUT("Flow Control = TX PAUSE frames only.\n");
	} else if ((adv_reg & adv_sym) && (adv_reg & adv_asm) &&
		   !(lp_reg & lp_sym) && (lp_reg & lp_asm)) {
		hw->fc.current_mode = yusur2_fc_rx_pause;
		DEBUGOUT("Flow Control = RX PAUSE frames only.\n");
	} else {
		hw->fc.current_mode = yusur2_fc_none;
		DEBUGOUT("Flow Control = NONE.\n");
	}
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_fc_autoneg_fiber - Enable flow control on 1 gig fiber
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according on 1 gig fiber.
 **/
STATIC s32 yusur2_fc_autoneg_fiber(struct yusur2_hw *hw)
{
	u32 pcs_anadv_reg, pcs_lpab_reg, linkstat;
	s32 ret_val = YUSUR2_ERR_FC_NOT_NEGOTIATED;

	/*
	 * On multispeed fiber at 1g, bail out if
	 * - link is up but AN did not complete, or if
	 * - link is up and AN completed but timed out
	 */

	linkstat = YUSUR2_READ_REG(hw, YUSUR2_PCS1GLSTA);
	if ((!!(linkstat & YUSUR2_PCS1GLSTA_AN_COMPLETE) == 0) ||
	    (!!(linkstat & YUSUR2_PCS1GLSTA_AN_TIMED_OUT) == 1)) {
		DEBUGOUT("Auto-Negotiation did not complete or timed out\n");
		goto out;
	}

	pcs_anadv_reg = YUSUR2_READ_REG(hw, YUSUR2_PCS1GANA);
	pcs_lpab_reg = YUSUR2_READ_REG(hw, YUSUR2_PCS1GANLP);

	ret_val =  yusur2_negotiate_fc(hw, pcs_anadv_reg,
				      pcs_lpab_reg, YUSUR2_PCS1GANA_SYM_PAUSE,
				      YUSUR2_PCS1GANA_ASM_PAUSE,
				      YUSUR2_PCS1GANA_SYM_PAUSE,
				      YUSUR2_PCS1GANA_ASM_PAUSE);

out:
	return ret_val;
}

/**
 *  yusur2_fc_autoneg_backplane - Enable flow control IEEE clause 37
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to IEEE clause 37.
 **/
STATIC s32 yusur2_fc_autoneg_backplane(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_ERR_FC_NOT_NEGOTIATED;
//TODO:
#if 0
	u32 links2, anlp1_reg, autoc_reg, links;

	/*
	 * On backplane, bail out if
	 * - backplane autoneg was not completed, or if
	 * - we are 82599 and link partner is not AN enabled
	 */
	links = YUSUR2_READ_REG(hw, YUSUR2_LINKS);
	if ((links & YUSUR2_LINKS_KX_AN_COMP) == 0) {
		DEBUGOUT("Auto-Negotiation did not complete\n");
		goto out;
	}

	if (hw->mac.type == yusur2_mac_82599EB) {
		links2 = YUSUR2_READ_REG(hw, YUSUR2_LINKS2);
		if ((links2 & YUSUR2_LINKS2_AN_SUPPORTED) == 0) {
			DEBUGOUT("Link partner is not AN enabled\n");
			goto out;
		}
	}
	/*
	 * Read the 10g AN autoc and LP ability registers and resolve
	 * local flow control settings accordingly
	 */
	autoc_reg = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	anlp1_reg = YUSUR2_READ_REG(hw, YUSUR2_ANLP1);

	ret_val = yusur2_negotiate_fc(hw, autoc_reg,
		anlp1_reg, YUSUR2_AUTOC_SYM_PAUSE, YUSUR2_AUTOC_ASM_PAUSE,
		YUSUR2_ANLP1_SYM_PAUSE, YUSUR2_ANLP1_ASM_PAUSE);

out:
#endif
	return ret_val;
}

/**
 *  yusur2_fc_autoneg_copper - Enable flow control IEEE clause 37
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to IEEE clause 37.
 **/
STATIC s32 yusur2_fc_autoneg_copper(struct yusur2_hw *hw)
{
	u16 technology_ability_reg = 0;
	u16 lp_technology_ability_reg = 0;

	hw->phy.ops.read_reg(hw, YUSUR2_MDIO_AUTO_NEG_ADVT,
			     YUSUR2_MDIO_AUTO_NEG_DEV_TYPE,
			     &technology_ability_reg);
	hw->phy.ops.read_reg(hw, YUSUR2_MDIO_AUTO_NEG_LP,
			     YUSUR2_MDIO_AUTO_NEG_DEV_TYPE,
			     &lp_technology_ability_reg);

	return yusur2_negotiate_fc(hw, (u32)technology_ability_reg,
				  (u32)lp_technology_ability_reg,
				  YUSUR2_TAF_SYM_PAUSE, YUSUR2_TAF_ASM_PAUSE,
				  YUSUR2_TAF_SYM_PAUSE, YUSUR2_TAF_ASM_PAUSE);
}

/**
 *  yusur2_fc_autoneg - Configure flow control
 *  @hw: pointer to hardware structure
 *
 *  Compares our advertised flow control capabilities to those advertised by
 *  our link partner, and determines the proper flow control mode to use.
 **/
void yusur2_fc_autoneg(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_ERR_FC_NOT_NEGOTIATED;
	yusur2_link_speed speed;
	bool link_up;

	DEBUGFUNC("yusur2_fc_autoneg");

	/*
	 * AN should have completed when the cable was plugged in.
	 * Look for reasons to bail out.  Bail out if:
	 * - FC autoneg is disabled, or if
	 * - link is not up.
	 */
	if (hw->fc.disable_fc_autoneg) {
		ERROR_REPORT1(YUSUR2_ERROR_UNSUPPORTED,
			     "Flow control autoneg is disabled");
		goto out;
	}

	hw->mac.ops.check_link(hw, &speed, &link_up, false);
	if (!link_up) {
		ERROR_REPORT1(YUSUR2_ERROR_SOFTWARE, "The link is down");
		goto out;
	}

	switch (hw->phy.media_type) {
	/* Autoneg flow control on fiber adapters */
	case yusur2_media_type_fiber_qsfp:
	case yusur2_media_type_fiber:
		if (speed == YUSUR2_LINK_SPEED_1GB_FULL)
			ret_val = yusur2_fc_autoneg_fiber(hw);
		break;

	/* Autoneg flow control on backplane adapters */
	case yusur2_media_type_backplane:
		ret_val = yusur2_fc_autoneg_backplane(hw);
		break;

	/* Autoneg flow control on copper adapters */
	case yusur2_media_type_copper:
		if (yusur2_device_supports_autoneg_fc(hw))
			ret_val = yusur2_fc_autoneg_copper(hw);
		break;

	default:
		break;
	}

out:
	if (ret_val == YUSUR2_SUCCESS) {
		hw->fc.fc_was_autonegged = true;
	} else {
		hw->fc.fc_was_autonegged = false;
		hw->fc.current_mode = hw->fc.requested_mode;
	}
}

/*
 * yusur2_pcie_timeout_poll - Return number of times to poll for completion
 * @hw: pointer to hardware structure
 *
 * System-wide timeout range is encoded in PCIe Device Control2 register.
 *
 * Add 10% to specified maximum and return the number of times to poll for
 * completion timeout, in units of 100 microsec.  Never return less than
 * 800 = 80 millisec.
 */
STATIC u32 yusur2_pcie_timeout_poll(struct yusur2_hw *hw)
{
	s16 devctl2;
	u32 pollcnt;

	devctl2 = YUSUR2_READ_PCIE_WORD(hw, YUSUR2_PCI_DEVICE_CONTROL2);
	devctl2 &= YUSUR2_PCIDEVCTRL2_TIMEO_MASK;

	switch (devctl2) {
	case YUSUR2_PCIDEVCTRL2_65_130ms:
		pollcnt = 1300;		/* 130 millisec */
		break;
	case YUSUR2_PCIDEVCTRL2_260_520ms:
		pollcnt = 5200;		/* 520 millisec */
		break;
	case YUSUR2_PCIDEVCTRL2_1_2s:
		pollcnt = 20000;	/* 2 sec */
		break;
	case YUSUR2_PCIDEVCTRL2_4_8s:
		pollcnt = 80000;	/* 8 sec */
		break;
	case YUSUR2_PCIDEVCTRL2_17_34s:
		pollcnt = 34000;	/* 34 sec */
		break;
	case YUSUR2_PCIDEVCTRL2_50_100us:	/* 100 microsecs */
	case YUSUR2_PCIDEVCTRL2_1_2ms:		/* 2 millisecs */
	case YUSUR2_PCIDEVCTRL2_16_32ms:		/* 32 millisec */
	case YUSUR2_PCIDEVCTRL2_16_32ms_def:	/* 32 millisec default */
	default:
		pollcnt = 800;		/* 80 millisec minimum */
		break;
	}

	/* add 10% to spec maximum */
	return (pollcnt * 11) / 10;
}

/**
 *  yusur2_disable_pcie_master - Disable PCI-express master access
 *  @hw: pointer to hardware structure
 *
 *  Disables PCI-Express master access and verifies there are no pending
 *  requests. YUSUR2_ERR_MASTER_REQUESTS_PENDING is returned if master disable
 *  bit hasn't caused the master requests to be disabled, else YUSUR2_SUCCESS
 *  is returned signifying master requests disabled.
 **/
s32 yusur2_disable_pcie_master(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_disable_pcie_master");
//TODO: need to check
#if 0
	u32 i, poll;
	u16 value;

	/* Always set this bit to ensure any future transactions are blocked */
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL, YUSUR2_CTRL_GIO_DIS);

	/* Exit if master requests are blocked */
	if (!(YUSUR2_READ_REG(hw, YUSUR2_STATUS) & YUSUR2_STATUS_GIO) ||
	    YUSUR2_REMOVED(hw->hw_addr))
		goto out;

	/* Poll for master request bit to clear */
	for (i = 0; i < YUSUR2_PCI_MASTER_DISABLE_TIMEOUT; i++) {
		usec_delay(100);
		if (!(YUSUR2_READ_REG(hw, YUSUR2_STATUS) & YUSUR2_STATUS_GIO))
			goto out;
	}

	/*
	 * Two consecutive resets are required via CTRL.RST per datasheet
	 * 5.2.5.3.2 Master Disable.  We set a flag to inform the reset routine
	 * of this need.  The first reset prevents new master requests from
	 * being issued by our device.  We then must wait 1usec or more for any
	 * remaining completions from the PCIe bus to trickle in, and then reset
	 * again to clear out any effects they may have had on our device.
	 */
	DEBUGOUT("GIO Master Disable bit didn't clear - requesting resets\n");
	hw->mac.flags |= YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED;

	if (hw->mac.type >= yusur2_mac_X550)
		goto out;

	/*
	 * Before proceeding, make sure that the PCIe block does not have
	 * transactions pending.
	 */
	poll = yusur2_pcie_timeout_poll(hw);
	for (i = 0; i < poll; i++) {
		usec_delay(100);
		value = YUSUR2_READ_PCIE_WORD(hw, YUSUR2_PCI_DEVICE_STATUS);
		if (YUSUR2_REMOVED(hw->hw_addr))
			goto out;
		if (!(value & YUSUR2_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
			goto out;
	}

	ERROR_REPORT1(YUSUR2_ERROR_POLLING,
		     "PCIe transaction pending bit also did not clear.\n");
	status = YUSUR2_ERR_MASTER_REQUESTS_PENDING;

out:
#endif
	return status;
}

/**
 *  yusur2_acquire_swfw_sync - Acquire SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
s32 yusur2_acquire_swfw_sync(struct yusur2_hw *hw, u32 mask)
{
	u32 gssr = 0;
	u32 swmask = mask;
	u32 fwmask = mask << 5;
	u32 timeout = 200;
	u32 i;

	DEBUGFUNC("yusur2_acquire_swfw_sync");

	for (i = 0; i < timeout; i++) {
		/*
		 * SW NVM semaphore bit is used for access to all
		 * SW_FW_SYNC bits (not just NVM)
		 */
		if (yusur2_get_eeprom_semaphore(hw))
			return YUSUR2_ERR_SWFW_SYNC;

		gssr = YUSUR2_READ_REG(hw, YUSUR2_GSSR);
		if (!(gssr & (fwmask | swmask))) {
			gssr |= swmask;
			YUSUR2_WRITE_REG(hw, YUSUR2_GSSR, gssr);
			yusur2_release_eeprom_semaphore(hw);
			return YUSUR2_SUCCESS;
		} else {
			/* Resource is currently in use by FW or SW */
			yusur2_release_eeprom_semaphore(hw);
			msec_delay(5);
		}
	}

	/* If time expired clear the bits holding the lock and retry */
	if (gssr & (fwmask | swmask))
		yusur2_release_swfw_sync(hw, gssr & (fwmask | swmask));

	msec_delay(5);
	return YUSUR2_ERR_SWFW_SYNC;
}

/**
 *  yusur2_release_swfw_sync - Release SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
void yusur2_release_swfw_sync(struct yusur2_hw *hw, u32 mask)
{
	u32 gssr;
	u32 swmask = mask;

	DEBUGFUNC("yusur2_release_swfw_sync");

	yusur2_get_eeprom_semaphore(hw);

	gssr = YUSUR2_READ_REG(hw, YUSUR2_GSSR);
	gssr &= ~swmask;
	YUSUR2_WRITE_REG(hw, YUSUR2_GSSR, gssr);

	yusur2_release_eeprom_semaphore(hw);
}

/**
 *  yusur2_disable_sec_rx_path_generic - Stops the receive data path
 *  @hw: pointer to hardware structure
 *
 *  Stops the receive data path and waits for the HW to internally empty
 *  the Rx security block
 **/
s32 yusur2_disable_sec_rx_path_generic(struct yusur2_hw *hw)
{
#define YUSUR2_MAX_SECRX_POLL 4000

	int i;
	int secrxreg;

	DEBUGFUNC("yusur2_disable_sec_rx_path_generic");


	secrxreg = YUSUR2_READ_REG(hw, YUSUR2_SECRXCTRL);
	secrxreg |= YUSUR2_SECRXCTRL_RX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECRXCTRL, secrxreg);
	for (i = 0; i < YUSUR2_MAX_SECRX_POLL; i++) {
		secrxreg = YUSUR2_READ_REG(hw, YUSUR2_SECRXSTAT);
		if (secrxreg & YUSUR2_SECRXSTAT_SECRX_RDY)
			break;
		else
			/* Use interrupt-safe sleep just in case */
			usec_delay(10);
	}

	/* For informational purposes only */
	if (i >= YUSUR2_MAX_SECRX_POLL)
		DEBUGOUT("Rx unit being enabled before security "
			 "path fully disabled.  Continuing with init.\n");

	return YUSUR2_SUCCESS;
}

/**
 *  prot_autoc_read_generic - Hides MAC differences needed for AUTOC read
 *  @hw: pointer to hardware structure
 *  @locked: bool to indicate whether the SW/FW lock was taken
 *  @reg_val: Value we read from AUTOC
 *
 *  The default case requires no protection so just to the register read.
 */
s32 yusur2_prot_autoc_read_generic(struct yusur2_hw *hw, bool *locked, u32 *reg_val)
{
	*locked = false;
	*reg_val = YUSUR2_READ_REG(hw, YUSUR2_AUTOC);
	return YUSUR2_SUCCESS;
}

/**
 * prot_autoc_write_generic - Hides MAC differences needed for AUTOC write
 * @hw: pointer to hardware structure
 * @reg_val: value to write to AUTOC
 * @locked: bool to indicate whether the SW/FW lock was already taken by
 *           previous read.
 *
 * The default case requires no protection so just to the register write.
 */
s32 yusur2_prot_autoc_write_generic(struct yusur2_hw *hw, u32 reg_val, bool locked)
{
	UNREFERENCED_1PARAMETER(locked);

	YUSUR2_WRITE_REG(hw, YUSUR2_AUTOC, reg_val);
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_enable_sec_rx_path_generic - Enables the receive data path
 *  @hw: pointer to hardware structure
 *
 *  Enables the receive data path.
 **/
s32 yusur2_enable_sec_rx_path_generic(struct yusur2_hw *hw)
{
	u32 secrxreg;

	DEBUGFUNC("yusur2_enable_sec_rx_path_generic");

	secrxreg = YUSUR2_READ_REG(hw, YUSUR2_SECRXCTRL);
	secrxreg &= ~YUSUR2_SECRXCTRL_RX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECRXCTRL, secrxreg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_enable_rx_dma_generic - Enable the Rx DMA unit
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit
 **/
s32 yusur2_enable_rx_dma_generic(struct yusur2_hw *hw, u32 regval)
{
	DEBUGFUNC("yusur2_enable_rx_dma_generic");

	if (regval & YUSUR2_RXCTRL_RXEN)
		yusur2_enable_rx(hw);
	else
		yusur2_disable_rx(hw);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_blink_led_start_generic - Blink LED based on index.
 *  @hw: pointer to hardware structure
 *  @index: led number to blink
 **/
s32 yusur2_blink_led_start_generic(struct yusur2_hw *hw, u32 index)
{
	yusur2_link_speed speed = 0;
	bool link_up = 0;
	u32 autoc_reg = 0;
	u32 led_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);
	s32 ret_val = YUSUR2_SUCCESS;
	bool locked = false;

	DEBUGFUNC("yusur2_blink_led_start_generic");

	if (index > 3)
		return YUSUR2_ERR_PARAM;

	/*
	 * Link must be up to auto-blink the LEDs;
	 * Force it if link is down.
	 */
	hw->mac.ops.check_link(hw, &speed, &link_up, false);

	if (!link_up) {
		ret_val = hw->mac.ops.prot_autoc_read(hw, &locked, &autoc_reg);
		if (ret_val != YUSUR2_SUCCESS)
			goto out;

		autoc_reg |= YUSUR2_AUTOC_AN_RESTART;
		autoc_reg |= YUSUR2_AUTOC_FLU;

		ret_val = hw->mac.ops.prot_autoc_write(hw, autoc_reg, locked);
		if (ret_val != YUSUR2_SUCCESS)
			goto out;

		YUSUR2_WRITE_FLUSH(hw);
		msec_delay(10);
	}

	led_reg &= ~YUSUR2_LED_MODE_MASK(index);
	led_reg |= YUSUR2_LED_BLINK(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, led_reg);
	YUSUR2_WRITE_FLUSH(hw);

out:
	return ret_val;
}

/**
 *  yusur2_blink_led_stop_generic - Stop blinking LED based on index.
 *  @hw: pointer to hardware structure
 *  @index: led number to stop blinking
 **/
s32 yusur2_blink_led_stop_generic(struct yusur2_hw *hw, u32 index)
{
	u32 autoc_reg = 0;
	u32 led_reg = YUSUR2_READ_REG(hw, YUSUR2_LEDCTL);
	s32 ret_val = YUSUR2_SUCCESS;
	bool locked = false;

	DEBUGFUNC("yusur2_blink_led_stop_generic");

	if (index > 3)
		return YUSUR2_ERR_PARAM;


	ret_val = hw->mac.ops.prot_autoc_read(hw, &locked, &autoc_reg);
	if (ret_val != YUSUR2_SUCCESS)
		goto out;

	autoc_reg &= ~YUSUR2_AUTOC_FLU;
	autoc_reg |= YUSUR2_AUTOC_AN_RESTART;

	ret_val = hw->mac.ops.prot_autoc_write(hw, autoc_reg, locked);
	if (ret_val != YUSUR2_SUCCESS)
		goto out;

	led_reg &= ~YUSUR2_LED_MODE_MASK(index);
	led_reg &= ~YUSUR2_LED_BLINK(index);
	led_reg |= YUSUR2_LED_LINK_ACTIVE << YUSUR2_LED_MODE_SHIFT(index);
	YUSUR2_WRITE_REG(hw, YUSUR2_LEDCTL, led_reg);
	YUSUR2_WRITE_FLUSH(hw);

out:
	return ret_val;
}

/**
 *  yusur2_get_san_mac_addr_offset - Get SAN MAC address offset from the EEPROM
 *  @hw: pointer to hardware structure
 *  @san_mac_offset: SAN MAC address offset
 *
 *  This function will read the EEPROM location for the SAN MAC address
 *  pointer, and returns the value at that location.  This is used in both
 *  get and set mac_addr routines.
 **/
STATIC s32 yusur2_get_san_mac_addr_offset(struct yusur2_hw *hw,
					 u16 *san_mac_offset)
{
	s32 ret_val;

	DEBUGFUNC("yusur2_get_san_mac_addr_offset");

	/*
	 * First read the EEPROM pointer to see if the MAC addresses are
	 * available.
	 */
	ret_val = hw->eeprom.ops.read(hw, YUSUR2_SAN_MAC_ADDR_PTR,
				      san_mac_offset);
	if (ret_val) {
		ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
			      "eeprom at offset %d failed",
			      YUSUR2_SAN_MAC_ADDR_PTR);
	}

	return ret_val;
}

/**
 *  yusur2_get_san_mac_addr_generic - SAN MAC address retrieval from the EEPROM
 *  @hw: pointer to hardware structure
 *  @san_mac_addr: SAN MAC address
 *
 *  Reads the SAN MAC address from the EEPROM, if it's available.  This is
 *  per-port, so set_lan_id() must be called before reading the addresses.
 *  set_lan_id() is called by identify_sfp(), but this cannot be relied
 *  upon for non-SFP connections, so we must call it here.
 **/
s32 yusur2_get_san_mac_addr_generic(struct yusur2_hw *hw, u8 *san_mac_addr)
{
	u16 san_mac_data, san_mac_offset;
	u8 i;
	s32 ret_val;

	DEBUGFUNC("yusur2_get_san_mac_addr_generic");

	/*
	 * First read the EEPROM pointer to see if the MAC addresses are
	 * available.  If they're not, no point in calling set_lan_id() here.
	 */
	ret_val = yusur2_get_san_mac_addr_offset(hw, &san_mac_offset);
	if (ret_val || san_mac_offset == 0 || san_mac_offset == 0xFFFF)
		goto san_mac_addr_out;

	/* make sure we know which port we need to program */
	hw->mac.ops.set_lan_id(hw);
	/* apply the port offset to the address offset */
	(hw->bus.func) ? (san_mac_offset += YUSUR2_SAN_MAC_ADDR_PORT1_OFFSET) :
			 (san_mac_offset += YUSUR2_SAN_MAC_ADDR_PORT0_OFFSET);
	for (i = 0; i < 3; i++) {
		ret_val = hw->eeprom.ops.read(hw, san_mac_offset,
					      &san_mac_data);
		if (ret_val) {
			ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
				      "eeprom read at offset %d failed",
				      san_mac_offset);
			goto san_mac_addr_out;
		}
		san_mac_addr[i * 2] = (u8)(san_mac_data);
		san_mac_addr[i * 2 + 1] = (u8)(san_mac_data >> 8);
		san_mac_offset++;
	}
	return YUSUR2_SUCCESS;

san_mac_addr_out:
	/*
	 * No addresses available in this EEPROM.  It's not an
	 * error though, so just wipe the local address and return.
	 */
	for (i = 0; i < 6; i++)
		san_mac_addr[i] = 0xFF;
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_san_mac_addr_generic - Write the SAN MAC address to the EEPROM
 *  @hw: pointer to hardware structure
 *  @san_mac_addr: SAN MAC address
 *
 *  Write a SAN MAC address to the EEPROM.
 **/
s32 yusur2_set_san_mac_addr_generic(struct yusur2_hw *hw, u8 *san_mac_addr)
{
	s32 ret_val;
	u16 san_mac_data, san_mac_offset;
	u8 i;

	DEBUGFUNC("yusur2_set_san_mac_addr_generic");

	/* Look for SAN mac address pointer.  If not defined, return */
	ret_val = yusur2_get_san_mac_addr_offset(hw, &san_mac_offset);
	if (ret_val || san_mac_offset == 0 || san_mac_offset == 0xFFFF)
		return YUSUR2_ERR_NO_SAN_ADDR_PTR;

	/* Make sure we know which port we need to write */
	hw->mac.ops.set_lan_id(hw);
	/* Apply the port offset to the address offset */
	(hw->bus.func) ? (san_mac_offset += YUSUR2_SAN_MAC_ADDR_PORT1_OFFSET) :
			 (san_mac_offset += YUSUR2_SAN_MAC_ADDR_PORT0_OFFSET);

	for (i = 0; i < 3; i++) {
		san_mac_data = (u16)((u16)(san_mac_addr[i * 2 + 1]) << 8);
		san_mac_data |= (u16)(san_mac_addr[i * 2]);
		hw->eeprom.ops.write(hw, san_mac_offset, san_mac_data);
		san_mac_offset++;
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_pcie_msix_count_generic - Gets MSI-X vector count
 *  @hw: pointer to hardware structure
 *
 *  Read PCIe configuration space, and get the MSI-X vector count from
 *  the capabilities table.
 **/
u16 yusur2_get_pcie_msix_count_generic(struct yusur2_hw *hw)
{
	u16 msix_count = 1;
//TODO:
#if 0
	u16 max_msix_count;
	u16 pcie_offset;

	switch (hw->mac.type) {
	case yusur2_mac_SN2100:
		pcie_offset = YUSUR2_PCIE_MSIX_82598_CAPS;
		max_msix_count = YUSUR2_MAX_MSIX_VECTORS_82598;
		break;
	case yusur2_mac_82599EB:
	case yusur2_mac_X540:
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		pcie_offset = YUSUR2_PCIE_MSIX_82599_CAPS;
		max_msix_count = YUSUR2_MAX_MSIX_VECTORS_82599;
		break;
	default:
		return msix_count;
	}

	DEBUGFUNC("yusur2_get_pcie_msix_count_generic");
	msix_count = YUSUR2_READ_PCIE_WORD(hw, pcie_offset);
	if (YUSUR2_REMOVED(hw->hw_addr))
		msix_count = 0;
	msix_count &= YUSUR2_PCIE_MSIX_TBL_SZ_MASK;

	/* MSI-X count is zero-based in HW */
	msix_count++;

	if (msix_count > max_msix_count)
		msix_count = max_msix_count;
#endif
	return msix_count;
}

/**
 *  yusur2_insert_mac_addr_generic - Find a RAR for this mac address
 *  @hw: pointer to hardware structure
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq pool to assign
 *
 *  Puts an ethernet address into a receive address register, or
 *  finds the rar that it is aleady in; adds to the pool list
 **/
s32 yusur2_insert_mac_addr_generic(struct yusur2_hw *hw, u8 *addr, u32 vmdq)
{
	static const u32 NO_EMPTY_RAR_FOUND = 0xFFFFFFFF;
	u32 first_empty_rar = NO_EMPTY_RAR_FOUND;
	u32 rar;
	u32 rar_low, rar_high;
	u32 addr_low, addr_high;

	DEBUGFUNC("yusur2_insert_mac_addr_generic");

	/* swap bytes for HW little endian */
	addr_low  = addr[0] | (addr[1] << 8)
			    | (addr[2] << 16)
			    | (addr[3] << 24);
	addr_high = addr[4] | (addr[5] << 8);

	/*
	 * Either find the mac_id in rar or find the first empty space.
	 * rar_highwater points to just after the highest currently used
	 * rar in order to shorten the search.  It grows when we add a new
	 * rar to the top.
	 */
	for (rar = 0; rar < hw->mac.rar_highwater; rar++) {
		rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(rar));

		if (((YUSUR2_RAH_AV & rar_high) == 0)
		    && first_empty_rar == NO_EMPTY_RAR_FOUND) {
			first_empty_rar = rar;
		} else if ((rar_high & 0xFFFF) == addr_high) {
			rar_low = YUSUR2_READ_REG(hw, YUSUR2_RAL(rar));
			if (rar_low == addr_low)
				break;    /* found it already in the rars */
		}
	}

	if (rar < hw->mac.rar_highwater) {
		/* already there so just add to the pool bits */
		yusur2_set_vmdq(hw, rar, vmdq);
	} else if (first_empty_rar != NO_EMPTY_RAR_FOUND) {
		/* stick it into first empty RAR slot we found */
		rar = first_empty_rar;
		yusur2_set_rar(hw, rar, addr, vmdq, YUSUR2_RAH_AV);
	} else if (rar == hw->mac.rar_highwater) {
		/* add it to the top of the list and inc the highwater mark */
		yusur2_set_rar(hw, rar, addr, vmdq, YUSUR2_RAH_AV);
		hw->mac.rar_highwater++;
	} else if (rar >= hw->mac.num_rar_entries) {
		return YUSUR2_ERR_INVALID_MAC_ADDR;
	}

	/*
	 * If we found rar[0], make sure the default pool bit (we use pool 0)
	 * remains cleared to be sure default pool packets will get delivered
	 */
	if (rar == 0)
		yusur2_clear_vmdq(hw, rar, 0);

	return rar;
}

/**
 *  yusur2_clear_vmdq_generic - Disassociate a VMDq pool index from a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
s32 yusur2_clear_vmdq_generic(struct yusur2_hw *hw, u32 rar, u32 vmdq)
{
	u32 mpsar_lo, mpsar_hi;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_clear_vmdq_generic");

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		ERROR_REPORT2(YUSUR2_ERROR_ARGUMENT,
			     "RAR index %d is out of range.\n", rar);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	mpsar_lo = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_LO(rar));
	mpsar_hi = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_HI(rar));

	if (YUSUR2_REMOVED(hw->hw_addr))
		goto done;

	if (!mpsar_lo && !mpsar_hi)
		goto done;

	if (vmdq == YUSUR2_CLEAR_VMDQ_ALL) {
		if (mpsar_lo) {
			YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(rar), 0);
			mpsar_lo = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_LO(rar));
		}
		if (mpsar_hi) {
			YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(rar), 0);
			mpsar_hi = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_HI(rar));
		}
	} else if (vmdq < 32) {
		mpsar_lo &= ~(1 << vmdq);
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(rar), mpsar_lo);
	} else {
		mpsar_hi &= ~(1 << (vmdq - 32));
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(rar), mpsar_hi);
	}

	/* was that the last pool using this rar? */
	if (mpsar_lo == 0 && mpsar_hi == 0 &&
	    rar != 0 && rar != hw->mac.san_mac_rar_index)
		hw->mac.ops.clear_rar(hw, rar);
done:
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_vmdq_generic - Associate a VMDq pool index with a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq pool index
 **/
s32 yusur2_set_vmdq_generic(struct yusur2_hw *hw, u32 rar, u32 vmdq)
{
	u32 mpsar;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("yusur2_set_vmdq_generic");

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		ERROR_REPORT2(YUSUR2_ERROR_ARGUMENT,
			     "RAR index %d is out of range.\n", rar);
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	if (vmdq < 32) {
		mpsar = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_LO(rar));
		mpsar |= 1 << vmdq;
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(rar), mpsar);
	} else {
		mpsar = YUSUR2_READ_REG(hw, YUSUR2_MPSAR_HI(rar));
		mpsar |= 1 << (vmdq - 32);
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(rar), mpsar);
	}
	return YUSUR2_SUCCESS;
}

/**
 *  This function should only be involved in the IOV mode.
 *  In IOV mode, Default pool is next pool after the number of
 *  VFs advertized and not 0.
 *  MPSAR table needs to be updated for SAN_MAC RAR [hw->mac.san_mac_rar_index]
 *
 *  yusur2_set_vmdq_san_mac - Associate default VMDq pool index with a rx address
 *  @hw: pointer to hardware struct
 *  @vmdq: VMDq pool index
 **/
s32 yusur2_set_vmdq_san_mac_generic(struct yusur2_hw *hw, u32 vmdq)
{
	u32 rar = hw->mac.san_mac_rar_index;

	DEBUGFUNC("yusur2_set_vmdq_san_mac");

	if (vmdq < 32) {
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(rar), 1 << vmdq);
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(rar), 0);
	} else {
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(rar), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(rar), 1 << (vmdq - 32));
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_uta_tables_generic - Initialize the Unicast Table Array
 *  @hw: pointer to hardware structure
 **/
s32 yusur2_init_uta_tables_generic(struct yusur2_hw *hw)
{
	int i;

	DEBUGFUNC("yusur2_init_uta_tables_generic");
	DEBUGOUT(" Clearing UTA\n");

	for (i = 0; i < 128; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_UTA(i), 0);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_find_vlvf_slot - find the vlanid or the first empty slot
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vlvf_bypass: true to find vlanid only, false returns first empty slot if
 *		  vlanid not found
 *
 *
 *  return the VLVF index where this VLAN id should be placed
 *
 **/
s32 yusur2_find_vlvf_slot(struct yusur2_hw *hw, u32 vlan, bool vlvf_bypass)
{
	s32 regindex, first_empty_slot;
	u32 bits;

	/* short cut the special case */
	if (vlan == 0)
		return 0;

	/* if vlvf_bypass is set we don't want to use an empty slot, we
	 * will simply bypass the VLVF if there are no entries present in the
	 * VLVF that contain our VLAN
	 */
	first_empty_slot = vlvf_bypass ? YUSUR2_ERR_NO_SPACE : 0;

	/* add VLAN enable bit for comparison */
	vlan |= YUSUR2_VLVF_VIEN;

	/* Search for the vlan id in the VLVF entries. Save off the first empty
	 * slot found along the way.
	 *
	 * pre-decrement loop covering (YUSUR2_VLVF_ENTRIES - 1) .. 1
	 */
	for (regindex = YUSUR2_VLVF_ENTRIES; --regindex;) {
		bits = YUSUR2_READ_REG(hw, YUSUR2_VLVF(regindex));
		if (bits == vlan)
			return regindex;
		if (!first_empty_slot && !bits)
			first_empty_slot = regindex;
	}

	/* If we are here then we didn't find the VLAN.  Return first empty
	 * slot we found during our search, else error.
	 */
	if (!first_empty_slot)
		ERROR_REPORT1(YUSUR2_ERROR_SOFTWARE, "No space in VLVF.\n");

	return first_empty_slot ? first_empty_slot : YUSUR2_ERR_NO_SPACE;
}

/**
 *  yusur2_set_vfta_generic - Set VLAN filter table
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VLVFB
 *  @vlan_on: boolean flag to turn on/off VLAN
 *  @vlvf_bypass: boolean flag indicating updating default pool is okay
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 yusur2_set_vfta_generic(struct yusur2_hw *hw, u32 vlan, u32 vind,
			   bool vlan_on, bool vlvf_bypass)
{
	u32 regidx, vfta_delta, vfta;
	s32 ret_val;

	DEBUGFUNC("yusur2_set_vfta_generic");

	if (vlan > 4095 || vind > 63)
		return YUSUR2_ERR_PARAM;

	/*
	 * this is a 2 part operation - first the VFTA, then the
	 * VLVF and VLVFB if VT Mode is set
	 * We don't write the VFTA until we know the VLVF part succeeded.
	 */

	/* Part 1
	 * The VFTA is a bitstring made up of 128 32-bit registers
	 * that enable the particular VLAN id, much like the MTA:
	 *    bits[11-5]: which register
	 *    bits[4-0]:  which bit in the register
	 */
	regidx = vlan / 32;
	vfta_delta = 1 << (vlan % 32);
	vfta = YUSUR2_READ_REG(hw, YUSUR2_VFTA(regidx));

	/*
	 * vfta_delta represents the difference between the current value
	 * of vfta and the value we want in the register.  Since the diff
	 * is an XOR mask we can just update the vfta using an XOR
	 */
	vfta_delta &= vlan_on ? ~vfta : vfta;
	vfta ^= vfta_delta;

	/* Part 2
	 * Call yusur2_set_vlvf_generic to set VLVFB and VLVF
	 */
	ret_val = yusur2_set_vlvf_generic(hw, vlan, vind, vlan_on, &vfta_delta,
					 vfta, vlvf_bypass);
	if (ret_val != YUSUR2_SUCCESS) {
		if (vlvf_bypass)
			goto vfta_update;
		return ret_val;
	}

vfta_update:
	/* Update VFTA now that we are ready for traffic */
	if (vfta_delta)
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(regidx), vfta);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_vlvf_generic - Set VLAN Pool Filter
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VLVFB
 *  @vlan_on: boolean flag to turn on/off VLAN in VLVF
 *  @vfta_delta: pointer to the difference between the current value of VFTA
 *		 and the desired value
 *  @vfta: the desired value of the VFTA
 *  @vlvf_bypass: boolean flag indicating updating default pool is okay
 *
 *  Turn on/off specified bit in VLVF table.
 **/
s32 yusur2_set_vlvf_generic(struct yusur2_hw *hw, u32 vlan, u32 vind,
			   bool vlan_on, u32 *vfta_delta, u32 vfta,
			   bool vlvf_bypass)
{
	u32 bits;
	s32 vlvf_index;

	DEBUGFUNC("yusur2_set_vlvf_generic");

	if (vlan > 4095 || vind > 63)
		return YUSUR2_ERR_PARAM;

	/* If VT Mode is set
	 *   Either vlan_on
	 *     make sure the vlan is in VLVF
	 *     set the vind bit in the matching VLVFB
	 *   Or !vlan_on
	 *     clear the pool bit and possibly the vind
	 */
	if (!(YUSUR2_READ_REG(hw, YUSUR2_VT_CTL) & YUSUR2_VT_CTL_VT_ENABLE))
		return YUSUR2_SUCCESS;

	vlvf_index = yusur2_find_vlvf_slot(hw, vlan, vlvf_bypass);
	if (vlvf_index < 0)
		return vlvf_index;

	bits = YUSUR2_READ_REG(hw, YUSUR2_VLVFB(vlvf_index * 2 + vind / 32));

	/* set the pool bit */
	bits |= 1 << (vind % 32);
	if (vlan_on)
		goto vlvf_update;

	/* clear the pool bit */
	bits ^= 1 << (vind % 32);

	if (!bits &&
	    !YUSUR2_READ_REG(hw, YUSUR2_VLVFB(vlvf_index * 2 + 1 - vind / 32))) {
		/* Clear VFTA first, then disable VLVF.  Otherwise
		 * we run the risk of stray packets leaking into
		 * the PF via the default pool
		 */
		if (*vfta_delta)
			YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(vlan / 32), vfta);

		/* disable VLVF and clear remaining bit from pool */
		YUSUR2_WRITE_REG(hw, YUSUR2_VLVF(vlvf_index), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VLVFB(vlvf_index * 2 + vind / 32), 0);

		return YUSUR2_SUCCESS;
	}

	/* If there are still bits set in the VLVFB registers
	 * for the VLAN ID indicated we need to see if the
	 * caller is requesting that we clear the VFTA entry bit.
	 * If the caller has requested that we clear the VFTA
	 * entry bit but there are still pools/VFs using this VLAN
	 * ID entry then ignore the request.  We're not worried
	 * about the case where we're turning the VFTA VLAN ID
	 * entry bit on, only when requested to turn it off as
	 * there may be multiple pools and/or VFs using the
	 * VLAN ID entry.  In that case we cannot clear the
	 * VFTA bit until all pools/VFs using that VLAN ID have also
	 * been cleared.  This will be indicated by "bits" being
	 * zero.
	 */
	*vfta_delta = 0;

vlvf_update:
	/* record pool change and enable VLAN ID if not already enabled */
	YUSUR2_WRITE_REG(hw, YUSUR2_VLVFB(vlvf_index * 2 + vind / 32), bits);
	YUSUR2_WRITE_REG(hw, YUSUR2_VLVF(vlvf_index), YUSUR2_VLVF_VIEN | vlan);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_clear_vfta_generic - Clear VLAN filter table
 *  @hw: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
s32 yusur2_clear_vfta_generic(struct yusur2_hw *hw)
{
	u32 offset;

	DEBUGFUNC("yusur2_clear_vfta_generic");

	for (offset = 0; offset < hw->mac.vft_size; offset++)
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(offset), 0);

	for (offset = 0; offset < YUSUR2_VLVF_ENTRIES; offset++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_VLVF(offset), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VLVFB(offset * 2), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VLVFB(offset * 2 + 1), 0);
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_need_crosstalk_fix - Determine if we need to do cross talk fix
 *  @hw: pointer to hardware structure
 *
 *  Contains the logic to identify if we need to verify link for the
 *  crosstalk fix
 **/
#if 0
static bool yusur2_need_crosstalk_fix(struct yusur2_hw *hw)
{

	/* Does FW say we need the fix */
	if (!hw->need_crosstalk_fix)
		return false;

	/* Only consider SFP+ PHYs i.e. media type fiber */
	switch (hw->mac.ops.get_media_type(hw)) {
	case yusur2_media_type_fiber:
	case yusur2_media_type_fiber_qsfp:
		break;
	default:
		return false;
	}

	return true;
}
#endif

/**
 *  yusur2_check_mac_link_generic - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 yusur2_check_mac_link_generic(struct yusur2_hw *hw, yusur2_link_speed *speed,
				 bool *link_up, bool link_up_wait_to_complete)
{
//TODO:
#if 0
	u32 links_reg, links_orig;
	u32 i;

	DEBUGFUNC("yusur2_check_mac_link_generic");

	/* If Crosstalk fix enabled do the sanity check of making sure
	 * the SFP+ cage is full.
	 */
	if (yusur2_need_crosstalk_fix(hw)) {
		u32 sfp_cage_full;

		switch (hw->mac.type) {
		case yusur2_mac_82599EB:
			sfp_cage_full = YUSUR2_READ_REG(hw, YUSUR2_ESDP) &
					YUSUR2_ESDP_SDP2;
			break;
		case yusur2_mac_X550EM_x:
		case yusur2_mac_X550EM_a:
			sfp_cage_full = YUSUR2_READ_REG(hw, YUSUR2_ESDP) &
					YUSUR2_ESDP_SDP0;
			break;
		default:
			/* sanity check - No SFP+ devices here */
			sfp_cage_full = false;
			break;
		}

		if (!sfp_cage_full) {
			*link_up = false;
			*speed = YUSUR2_LINK_SPEED_UNKNOWN;
			return YUSUR2_SUCCESS;
		}
	}

	/* clear the old state */
	links_orig = YUSUR2_READ_REG(hw, YUSUR2_LINKS);

	links_reg = YUSUR2_READ_REG(hw, YUSUR2_LINKS);

	if (links_orig != links_reg) {
		DEBUGOUT2("LINKS changed from %08X to %08X\n",
			  links_orig, links_reg);
	}

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

	switch (links_reg & YUSUR2_LINKS_SPEED_82599) {
	case YUSUR2_LINKS_SPEED_10G_82599:
		*speed = YUSUR2_LINK_SPEED_10GB_FULL;
		if (hw->mac.type >= yusur2_mac_X550) {
			if (links_reg & YUSUR2_LINKS_SPEED_NON_STD)
				*speed = YUSUR2_LINK_SPEED_2_5GB_FULL;
		}
		break;
	case YUSUR2_LINKS_SPEED_1G_82599:
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;
		break;
	case YUSUR2_LINKS_SPEED_100_82599:
		*speed = YUSUR2_LINK_SPEED_100_FULL;
		if (hw->mac.type == yusur2_mac_X550) {
			if (links_reg & YUSUR2_LINKS_SPEED_NON_STD)
				*speed = YUSUR2_LINK_SPEED_5GB_FULL;
		}
		break;
	case YUSUR2_LINKS_SPEED_10_X550EM_A:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
#ifdef PREBOOT_SUPPORT
		if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T ||
		    hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T_L ||
		    hw->device_id == YUSUR2_DEV_ID_X550EM_A_SGMII ||
		    hw->device_id == YUSUR2_DEV_ID_X550EM_A_SGMII_L)
			*speed = YUSUR2_LINK_SPEED_10_FULL;
#else
		if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T ||
		    hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T_L)
			*speed = YUSUR2_LINK_SPEED_10_FULL;
#endif /* PREBOOT_SUPPORT */
		break;
	default:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
	}
#endif
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_wwn_prefix_generic - Get alternative WWNN/WWPN prefix from
 *  the EEPROM
 *  @hw: pointer to hardware structure
 *  @wwnn_prefix: the alternative WWNN prefix
 *  @wwpn_prefix: the alternative WWPN prefix
 *
 *  This function will read the EEPROM from the alternative SAN MAC address
 *  block to check the support for the alternative WWNN/WWPN prefix support.
 **/
s32 yusur2_get_wwn_prefix_generic(struct yusur2_hw *hw, u16 *wwnn_prefix,
				 u16 *wwpn_prefix)
{
	u16 offset, caps;
	u16 alt_san_mac_blk_offset;

	DEBUGFUNC("yusur2_get_wwn_prefix_generic");

	/* clear output first */
	*wwnn_prefix = 0xFFFF;
	*wwpn_prefix = 0xFFFF;

	/* check if alternative SAN MAC is supported */
	offset = YUSUR2_ALT_SAN_MAC_ADDR_BLK_PTR;
	if (hw->eeprom.ops.read(hw, offset, &alt_san_mac_blk_offset))
		goto wwn_prefix_err;

	if ((alt_san_mac_blk_offset == 0) ||
	    (alt_san_mac_blk_offset == 0xFFFF))
		goto wwn_prefix_out;

	/* check capability in alternative san mac address block */
	offset = alt_san_mac_blk_offset + YUSUR2_ALT_SAN_MAC_ADDR_CAPS_OFFSET;
	if (hw->eeprom.ops.read(hw, offset, &caps))
		goto wwn_prefix_err;
	if (!(caps & YUSUR2_ALT_SAN_MAC_ADDR_CAPS_ALTWWN))
		goto wwn_prefix_out;

	/* get the corresponding prefix for WWNN/WWPN */
	offset = alt_san_mac_blk_offset + YUSUR2_ALT_SAN_MAC_ADDR_WWNN_OFFSET;
	if (hw->eeprom.ops.read(hw, offset, wwnn_prefix)) {
		ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
			      "eeprom read at offset %d failed", offset);
	}

	offset = alt_san_mac_blk_offset + YUSUR2_ALT_SAN_MAC_ADDR_WWPN_OFFSET;
	if (hw->eeprom.ops.read(hw, offset, wwpn_prefix))
		goto wwn_prefix_err;

wwn_prefix_out:
	return YUSUR2_SUCCESS;

wwn_prefix_err:
	ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
		      "eeprom read at offset %d failed", offset);
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_get_fcoe_boot_status_generic - Get FCOE boot status from EEPROM
 *  @hw: pointer to hardware structure
 *  @bs: the fcoe boot status
 *
 *  This function will read the FCOE boot status from the iSCSI FCOE block
 **/
s32 yusur2_get_fcoe_boot_status_generic(struct yusur2_hw *hw, u16 *bs)
{
	u16 offset, caps, flags;
	s32 status;

	DEBUGFUNC("yusur2_get_fcoe_boot_status_generic");

	/* clear output first */
	*bs = yusur2_fcoe_bootstatus_unavailable;

	/* check if FCOE IBA block is present */
	offset = YUSUR2_FCOE_IBA_CAPS_BLK_PTR;
	status = hw->eeprom.ops.read(hw, offset, &caps);
	if (status != YUSUR2_SUCCESS)
		goto out;

	if (!(caps & YUSUR2_FCOE_IBA_CAPS_FCOE))
		goto out;

	/* check if iSCSI FCOE block is populated */
	status = hw->eeprom.ops.read(hw, YUSUR2_ISCSI_FCOE_BLK_PTR, &offset);
	if (status != YUSUR2_SUCCESS)
		goto out;

	if ((offset == 0) || (offset == 0xFFFF))
		goto out;

	/* read fcoe flags in iSCSI FCOE block */
	offset = offset + YUSUR2_ISCSI_FCOE_FLAGS_OFFSET;
	status = hw->eeprom.ops.read(hw, offset, &flags);
	if (status != YUSUR2_SUCCESS)
		goto out;

	if (flags & YUSUR2_ISCSI_FCOE_FLAGS_ENABLE)
		*bs = yusur2_fcoe_bootstatus_enabled;
	else
		*bs = yusur2_fcoe_bootstatus_disabled;

out:
	return status;
}

/**
 *  yusur2_set_mac_anti_spoofing - Enable/Disable MAC anti-spoofing
 *  @hw: pointer to hardware structure
 *  @enable: enable or disable switch for MAC anti-spoofing
 *  @vf: Virtual Function pool - VF Pool to set for MAC anti-spoofing
 *
 **/
void yusur2_set_mac_anti_spoofing(struct yusur2_hw *hw, bool enable, int vf)
{
//TODO:
#if 0
	int vf_target_reg = vf >> 3;
	int vf_target_shift = vf % 8;
	u32 pfvfspoof;

	if (hw->mac.type == yusur2_mac_82598EB)
		return;

	pfvfspoof = YUSUR2_READ_REG(hw, YUSUR2_PFVFSPOOF(vf_target_reg));
	if (enable)
		pfvfspoof |= (1 << vf_target_shift);
	else
		pfvfspoof &= ~(1 << vf_target_shift);
	YUSUR2_WRITE_REG(hw, YUSUR2_PFVFSPOOF(vf_target_reg), pfvfspoof);
#endif
}

/**
 *  yusur2_set_vlan_anti_spoofing - Enable/Disable VLAN anti-spoofing
 *  @hw: pointer to hardware structure
 *  @enable: enable or disable switch for VLAN anti-spoofing
 *  @vf: Virtual Function pool - VF Pool to set for VLAN anti-spoofing
 *
 **/
void yusur2_set_vlan_anti_spoofing(struct yusur2_hw *hw, bool enable, int vf)
{
//TODO:
#if 0
	int vf_target_reg = vf >> 3;
	int vf_target_shift = vf % 8 + YUSUR2_SPOOF_VLANAS_SHIFT;
	u32 pfvfspoof;

	if (hw->mac.type == yusur2_mac_82598EB)
		return;

	pfvfspoof = YUSUR2_READ_REG(hw, YUSUR2_PFVFSPOOF(vf_target_reg));
	if (enable)
		pfvfspoof |= (1 << vf_target_shift);
	else
		pfvfspoof &= ~(1 << vf_target_shift);
	YUSUR2_WRITE_REG(hw, YUSUR2_PFVFSPOOF(vf_target_reg), pfvfspoof);
#endif
}

/**
 *  yusur2_get_device_caps_generic - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 yusur2_get_device_caps_generic(struct yusur2_hw *hw, u16 *device_caps)
{
	DEBUGFUNC("yusur2_get_device_caps_generic");

	hw->eeprom.ops.read(hw, YUSUR2_DEVICE_CAPS, device_caps);

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_enable_relaxed_ordering_gen2 - Enable relaxed ordering
 *  @hw: pointer to hardware structure
 *
 **/
void yusur2_enable_relaxed_ordering_gen2(struct yusur2_hw *hw)
{
	u32 regval;
	u32 i;

	DEBUGFUNC("yusur2_enable_relaxed_ordering_gen2");

	/* Enable relaxed ordering */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		regval = YUSUR2_READ_REG(hw, YUSUR2_DCA_TXCTRL_82599(i));
		regval |= YUSUR2_DCA_TXCTRL_DESC_WRO_EN;
		YUSUR2_WRITE_REG(hw, YUSUR2_DCA_TXCTRL_82599(i), regval);
	}

	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		regval = YUSUR2_READ_REG(hw, YUSUR2_DCA_RXCTRL(i));
		regval |= YUSUR2_DCA_RXCTRL_DATA_WRO_EN |
			  YUSUR2_DCA_RXCTRL_HEAD_WRO_EN;
		YUSUR2_WRITE_REG(hw, YUSUR2_DCA_RXCTRL(i), regval);
	}

}

/**
 *  yusur2_calculate_checksum - Calculate checksum for buffer
 *  @buffer: pointer to EEPROM
 *  @length: size of EEPROM to calculate a checksum for
 *  Calculates the checksum for some buffer on a specified length.  The
 *  checksum calculated is returned.
 **/
u8 yusur2_calculate_checksum(u8 *buffer, u32 length)
{
	u32 i;
	u8 sum = 0;

	DEBUGFUNC("yusur2_calculate_checksum");

	if (!buffer)
		return 0;

	for (i = 0; i < length; i++)
		sum += buffer[i];

	return (u8) (0 - sum);
}

/**
 *  yusur2_hic_unlocked - Issue command to manageability block unlocked
 *  @hw: pointer to the HW structure
 *  @buffer: command to write and where the return status will be placed
 *  @length: length of buffer, must be multiple of 4 bytes
 *  @timeout: time in ms to wait for command completion
 *
 *  Communicates with the manageability block. On success return YUSUR2_SUCCESS
 *  else returns semaphore error when encountering an error acquiring
 *  semaphore or YUSUR2_ERR_HOST_INTERFACE_COMMAND when command fails.
 *
 *  This function assumes that the YUSUR2_GSSR_SW_MNG_SM semaphore is held
 *  by the caller.
 **/
s32 yusur2_hic_unlocked(struct yusur2_hw *hw, u32 *buffer, u32 length,
		       u32 timeout)
{
	u32 hicr, i, fwsts;
	u16 dword_len;

	DEBUGFUNC("yusur2_hic_unlocked");

	if (!length || length > YUSUR2_HI_MAX_BLOCK_BYTE_LENGTH) {
		DEBUGOUT1("Buffer length failure buffersize=%d.\n", length);
		return YUSUR2_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Set bit 9 of FWSTS clearing FW reset indication */
	fwsts = YUSUR2_READ_REG(hw, YUSUR2_FWSTS);
	YUSUR2_WRITE_REG(hw, YUSUR2_FWSTS, fwsts | YUSUR2_FWSTS_FWRI);

	/* Check that the host interface is enabled. */
	hicr = YUSUR2_READ_REG(hw, YUSUR2_HICR);
	if (!(hicr & YUSUR2_HICR_EN)) {
		DEBUGOUT("YUSUR2_HOST_EN bit disabled.\n");
		return YUSUR2_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Calculate length in DWORDs. We must be DWORD aligned */
	if (length % sizeof(u32)) {
		DEBUGOUT("Buffer length failure, not aligned to dword");
		return YUSUR2_ERR_INVALID_ARGUMENT;
	}

	dword_len = length >> 2;

	/* The device driver writes the relevant command block
	 * into the ram area.
	 */
	for (i = 0; i < dword_len; i++)
		YUSUR2_WRITE_REG_ARRAY(hw, YUSUR2_FLEX_MNG,
				      i, YUSUR2_CPU_TO_LE32(buffer[i]));

	/* Setting this bit tells the ARC that a new command is pending. */
	YUSUR2_WRITE_REG(hw, YUSUR2_HICR, hicr | YUSUR2_HICR_C);

	for (i = 0; i < timeout; i++) {
		hicr = YUSUR2_READ_REG(hw, YUSUR2_HICR);
		if (!(hicr & YUSUR2_HICR_C))
			break;
		msec_delay(1);
	}

	/* Check command completion */
	if ((timeout && i == timeout) ||
	    !(YUSUR2_READ_REG(hw, YUSUR2_HICR) & YUSUR2_HICR_SV)) {
		ERROR_REPORT1(YUSUR2_ERROR_CAUTION,
			     "Command has failed with no status valid.\n");
		return YUSUR2_ERR_HOST_INTERFACE_COMMAND;
	}

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_host_interface_command - Issue command to manageability block
 *  @hw: pointer to the HW structure
 *  @buffer: contains the command to write and where the return status will
 *   be placed
 *  @length: length of buffer, must be multiple of 4 bytes
 *  @timeout: time in ms to wait for command completion
 *  @return_data: read and return data from the buffer (true) or not (false)
 *   Needed because FW structures are big endian and decoding of
 *   these fields can be 8 bit or 16 bit based on command. Decoding
 *   is not easily understood without making a table of commands.
 *   So we will leave this up to the caller to read back the data
 *   in these cases.
 *
 *  Communicates with the manageability block. On success return YUSUR2_SUCCESS
 *  else returns semaphore error when encountering an error acquiring
 *  semaphore or YUSUR2_ERR_HOST_INTERFACE_COMMAND when command fails.
 **/
s32 yusur2_host_interface_command(struct yusur2_hw *hw, u32 *buffer,
				 u32 length, u32 timeout, bool return_data)
{
	u32 hdr_size = sizeof(struct yusur2_hic_hdr);
	struct yusur2_hic_hdr *resp = (struct yusur2_hic_hdr *)buffer;
	u16 buf_len;
	s32 status;
	u32 bi;
	u32 dword_len;

	DEBUGFUNC("yusur2_host_interface_command");

	if (length == 0 || length > YUSUR2_HI_MAX_BLOCK_BYTE_LENGTH) {
		DEBUGOUT1("Buffer length failure buffersize=%d.\n", length);
		return YUSUR2_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Take management host interface semaphore */
	status = hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_SW_MNG_SM);
	if (status)
		return status;

	status = yusur2_hic_unlocked(hw, buffer, length, timeout);
	if (status)
		goto rel_out;

	if (!return_data)
		goto rel_out;

	/* Calculate length in DWORDs */
	dword_len = hdr_size >> 2;

	/* first pull in the header so we know the buffer length */
	for (bi = 0; bi < dword_len; bi++) {
		buffer[bi] = YUSUR2_READ_REG_ARRAY(hw, YUSUR2_FLEX_MNG, bi);
		YUSUR2_LE32_TO_CPUS((uintptr_t)&buffer[bi]);
	}

	/*
	 * If there is any thing in data position pull it in
	 * Read Flash command requires reading buffer length from
	 * two byes instead of one byte
	 */
	if (resp->cmd == YUSUR2_HOST_INTERFACE_FLASH_READ_CMD ||
	    resp->cmd == YUSUR2_HOST_INTERFACE_SHADOW_RAM_READ_CMD) {
		for (; bi < dword_len + 2; bi++) {
			buffer[bi] = YUSUR2_READ_REG_ARRAY(hw, YUSUR2_FLEX_MNG,
							  bi);
			YUSUR2_LE32_TO_CPUS(&buffer[bi]);
		}
		buf_len = (((u16)(resp->cmd_or_resp.ret_status) << 3)
				  & 0xF00) | resp->buf_len;
		hdr_size += (2 << 2);
	} else {
		buf_len = resp->buf_len;
	}
	if (!buf_len)
		goto rel_out;

	if (length < buf_len + hdr_size) {
		DEBUGOUT("Buffer not large enough for reply message.\n");
		status = YUSUR2_ERR_HOST_INTERFACE_COMMAND;
		goto rel_out;
	}

	/* Calculate length in DWORDs, add 3 for odd lengths */
	dword_len = (buf_len + 3) >> 2;

	/* Pull in the rest of the buffer (bi is where we left off) */
	for (; bi <= dword_len; bi++) {
		buffer[bi] = YUSUR2_READ_REG_ARRAY(hw, YUSUR2_FLEX_MNG, bi);
		YUSUR2_LE32_TO_CPUS((uintptr_t)&buffer[bi]);
	}

rel_out:
	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_SW_MNG_SM);

	return status;
}

/**
 *  yusur2_set_fw_drv_ver_generic - Sends driver version to firmware
 *  @hw: pointer to the HW structure
 *  @maj: driver version major number
 *  @min: driver version minor number
 *  @build: driver version build number
 *  @sub: driver version sub build number
 *  @len: unused
 *  @driver_ver: unused
 *
 *  Sends driver version number to firmware through the manageability
 *  block.  On success return YUSUR2_SUCCESS
 *  else returns YUSUR2_ERR_SWFW_SYNC when encountering an error acquiring
 *  semaphore or YUSUR2_ERR_HOST_INTERFACE_COMMAND when command fails.
 **/
s32 yusur2_set_fw_drv_ver_generic(struct yusur2_hw *hw, u8 maj, u8 min,
				 u8 build, u8 sub, u16 len,
				 const char *driver_ver)
{
	struct yusur2_hic_drv_info fw_cmd;
	int i;
	s32 ret_val = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_set_fw_drv_ver_generic");
	UNREFERENCED_2PARAMETER(len, driver_ver);

	fw_cmd.hdr.cmd = FW_CEM_CMD_DRIVER_INFO;
	fw_cmd.hdr.buf_len = FW_CEM_CMD_DRIVER_INFO_LEN;
	fw_cmd.hdr.cmd_or_resp.cmd_resv = FW_CEM_CMD_RESERVED;
	fw_cmd.port_num = (u8)hw->bus.func;
	fw_cmd.ver_maj = maj;
	fw_cmd.ver_min = min;
	fw_cmd.ver_build = build;
	fw_cmd.ver_sub = sub;
	fw_cmd.hdr.checksum = 0;
	fw_cmd.pad = 0;
	fw_cmd.pad2 = 0;
	fw_cmd.hdr.checksum = yusur2_calculate_checksum((u8 *)&fw_cmd,
				(FW_CEM_HDR_LEN + fw_cmd.hdr.buf_len));

	for (i = 0; i <= FW_CEM_MAX_RETRIES; i++) {
		ret_val = yusur2_host_interface_command(hw, (u32 *)&fw_cmd,
						       sizeof(fw_cmd),
						       YUSUR2_HI_COMMAND_TIMEOUT,
						       true);
		if (ret_val != YUSUR2_SUCCESS)
			continue;

		if (fw_cmd.hdr.cmd_or_resp.ret_status ==
		    FW_CEM_RESP_STATUS_SUCCESS)
			ret_val = YUSUR2_SUCCESS;
		else
			ret_val = YUSUR2_ERR_HOST_INTERFACE_COMMAND;

		break;
	}

	return ret_val;
}

/**
 * yusur2_set_rxpba_generic - Initialize Rx packet buffer
 * @hw: pointer to hardware structure
 * @num_pb: number of packet buffers to allocate
 * @headroom: reserve n KB of headroom
 * @strategy: packet buffer allocation strategy
 **/
void yusur2_set_rxpba_generic(struct yusur2_hw *hw, int num_pb, u32 headroom,
			     int strategy)
{
	u32 pbsize = hw->mac.rx_pb_size;
	int i = 0;
	u32 rxpktsize, txpktsize, txpbthresh;

	/* Reserve headroom */
	pbsize -= headroom;

	if (!num_pb)
		num_pb = 1;

	/* Divide remaining packet buffer space amongst the number of packet
	 * buffers requested using supplied strategy.
	 */
	switch (strategy) {
	case PBA_STRATEGY_WEIGHTED:
		/* yusur2_dcb_pba_80_48 strategy weight first half of packet
		 * buffer with 5/8 of the packet buffer space.
		 */
		rxpktsize = (pbsize * 5) / (num_pb * 4);
		pbsize -= rxpktsize * (num_pb / 2);
		rxpktsize <<= YUSUR2_RXPBSIZE_SHIFT;
		for (; i < (num_pb / 2); i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(i), rxpktsize);
		/* fall through - configure remaining packet buffers */
	case PBA_STRATEGY_EQUAL:
		rxpktsize = (pbsize / (num_pb - i)) << YUSUR2_RXPBSIZE_SHIFT;
		for (; i < num_pb; i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(i), rxpktsize);
		break;
	default:
		break;
	}

	/* Only support an equally distributed Tx packet buffer strategy. */
	txpktsize = YUSUR2_TXPBSIZE_MAX / num_pb;
	txpbthresh = (txpktsize / 1024) - YUSUR2_TXPKT_SIZE_MAX;
	for (i = 0; i < num_pb; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_TXPBSIZE(i), txpktsize);
		YUSUR2_WRITE_REG(hw, YUSUR2_TXPBTHRESH(i), txpbthresh);
	}

	/* Clear unused TCs, if any, to zero buffer size*/
	for (; i < YUSUR2_MAX_PB; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RXPBSIZE(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_TXPBSIZE(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_TXPBTHRESH(i), 0);
	}
}

/**
 * yusur2_clear_tx_pending - Clear pending TX work from the PCIe fifo
 * @hw: pointer to the hardware structure
 *
 * The 82599 and x540 MACs can experience issues if TX work is still pending
 * when a reset occurs.  This function prevents this by flushing the PCIe
 * buffers on the system.
 **/
void yusur2_clear_tx_pending(struct yusur2_hw *hw)
{
	u32 gcr_ext, hlreg0, i, poll;
	u16 value;

	/*
	 * If double reset is not requested then all transactions should
	 * already be clear and as such there is no work to do
	 */
	if (!(hw->mac.flags & YUSUR2_FLAGS_DOUBLE_RESET_REQUIRED))
		return;

	/*
	 * Set loopback enable to prevent any transmits from being sent
	 * should the link come up.  This assumes that the RXCTRL.RXEN bit
	 * has already been cleared.
	 */
	hlreg0 = YUSUR2_READ_REG(hw, YUSUR2_HLREG0);
	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, hlreg0 | YUSUR2_HLREG0_LPBK);

	/* Wait for a last completion before clearing buffers */
	YUSUR2_WRITE_FLUSH(hw);
	msec_delay(3);

	/*
	 * Before proceeding, make sure that the PCIe block does not have
	 * transactions pending.
	 */
	poll = yusur2_pcie_timeout_poll(hw);
	for (i = 0; i < poll; i++) {
		usec_delay(100);
		value = YUSUR2_READ_PCIE_WORD(hw, YUSUR2_PCI_DEVICE_STATUS);
		if (YUSUR2_REMOVED(hw->hw_addr))
			goto out;
		if (!(value & YUSUR2_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
			goto out;
	}

out:
	/* initiate cleaning flow for buffers in the PCIe transaction layer */
	gcr_ext = YUSUR2_READ_REG(hw, YUSUR2_GCR_EXT);
	YUSUR2_WRITE_REG(hw, YUSUR2_GCR_EXT,
			gcr_ext | YUSUR2_GCR_EXT_BUFFERS_CLEAR);

	/* Flush all writes and allow 20usec for all transactions to clear */
	YUSUR2_WRITE_FLUSH(hw);
	usec_delay(20);

	/* restore previous register values */
	YUSUR2_WRITE_REG(hw, YUSUR2_GCR_EXT, gcr_ext);
	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, hlreg0);
}

#if 0
STATIC const u8 yusur2_emc_temp_data[4] = {
	YUSUR2_EMC_INTERNAL_DATA,
	YUSUR2_EMC_DIODE1_DATA,
	YUSUR2_EMC_DIODE2_DATA,
	YUSUR2_EMC_DIODE3_DATA
};
STATIC const u8 yusur2_emc_therm_limit[4] = {
	YUSUR2_EMC_INTERNAL_THERM_LIMIT,
	YUSUR2_EMC_DIODE1_THERM_LIMIT,
	YUSUR2_EMC_DIODE2_THERM_LIMIT,
	YUSUR2_EMC_DIODE3_THERM_LIMIT
};
#endif

/**
 *  yusur2_get_thermal_sensor_data - Gathers thermal sensor data
 *  @hw: pointer to hardware structure
 *
 *  Returns the thermal sensor data structure
 **/
s32 yusur2_get_thermal_sensor_data_generic(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;
//TODO:
#if 0
	u16 ets_offset;
	u16 ets_cfg;
	u16 ets_sensor;
	u8  num_sensors;
	u8  sensor_index;
	u8  sensor_location;
	u8  i;
	struct yusur2_thermal_sensor_data *data = &hw->mac.thermal_sensor_data;

	DEBUGFUNC("yusur2_get_thermal_sensor_data_generic");

	/* Only support thermal sensors attached to 82599 physical port 0 */
	if ((hw->mac.type != yusur2_mac_82599EB) ||
	    (YUSUR2_READ_REG(hw, YUSUR2_STATUS) & YUSUR2_STATUS_LAN_ID_1)) {
		status = YUSUR2_NOT_IMPLEMENTED;
		goto out;
	}

	status = hw->eeprom.ops.read(hw, YUSUR2_ETS_CFG, &ets_offset);
	if (status)
		goto out;

	if ((ets_offset == 0x0000) || (ets_offset == 0xFFFF)) {
		status = YUSUR2_NOT_IMPLEMENTED;
		goto out;
	}

	status = hw->eeprom.ops.read(hw, ets_offset, &ets_cfg);
	if (status)
		goto out;

	if (((ets_cfg & YUSUR2_ETS_TYPE_MASK) >> YUSUR2_ETS_TYPE_SHIFT)
		!= YUSUR2_ETS_TYPE_EMC) {
		status = YUSUR2_NOT_IMPLEMENTED;
		goto out;
	}

	num_sensors = (ets_cfg & YUSUR2_ETS_NUM_SENSORS_MASK);
	if (num_sensors > YUSUR2_MAX_SENSORS)
		num_sensors = YUSUR2_MAX_SENSORS;

	for (i = 0; i < num_sensors; i++) {
		status = hw->eeprom.ops.read(hw, (ets_offset + 1 + i),
					     &ets_sensor);
		if (status)
			goto out;

		sensor_index = ((ets_sensor & YUSUR2_ETS_DATA_INDEX_MASK) >>
				YUSUR2_ETS_DATA_INDEX_SHIFT);
		sensor_location = ((ets_sensor & YUSUR2_ETS_DATA_LOC_MASK) >>
				   YUSUR2_ETS_DATA_LOC_SHIFT);

		if (sensor_location != 0) {
			status = hw->phy.ops.read_i2c_byte(hw,
					yusur2_emc_temp_data[sensor_index],
					YUSUR2_I2C_THERMAL_SENSOR_ADDR,
					&data->sensor[i].temp);
			if (status)
				goto out;
		}
	}
out:
#endif
	return status;
}

/**
 *  yusur2_init_thermal_sensor_thresh_generic - Inits thermal sensor thresholds
 *  @hw: pointer to hardware structure
 *
 *  Inits the thermal sensor thresholds according to the NVM map
 *  and save off the threshold and location values into mac.thermal_sensor_data
 **/
s32 yusur2_init_thermal_sensor_thresh_generic(struct yusur2_hw *hw)
{
	s32 status = YUSUR2_SUCCESS;
//TODO:
#if 0
	u16 offset;
	u16 ets_offset;
	u16 ets_cfg;
	u16 ets_sensor;
	u8  low_thresh_delta;
	u8  num_sensors;
	u8  sensor_index;
	u8  sensor_location;
	u8  therm_limit;
	u8  i;
	struct yusur2_thermal_sensor_data *data = &hw->mac.thermal_sensor_data;

	DEBUGFUNC("yusur2_init_thermal_sensor_thresh_generic");

	memset(data, 0, sizeof(struct yusur2_thermal_sensor_data));

	/* Only support thermal sensors attached to 82599 physical port 0 */
	if ((hw->mac.type != yusur2_mac_82599EB) ||
	    (YUSUR2_READ_REG(hw, YUSUR2_STATUS) & YUSUR2_STATUS_LAN_ID_1))
		return YUSUR2_NOT_IMPLEMENTED;

	offset = YUSUR2_ETS_CFG;
	if (hw->eeprom.ops.read(hw, offset, &ets_offset))
		goto eeprom_err;
	if ((ets_offset == 0x0000) || (ets_offset == 0xFFFF))
		return YUSUR2_NOT_IMPLEMENTED;

	offset = ets_offset;
	if (hw->eeprom.ops.read(hw, offset, &ets_cfg))
		goto eeprom_err;
	if (((ets_cfg & YUSUR2_ETS_TYPE_MASK) >> YUSUR2_ETS_TYPE_SHIFT)
		!= YUSUR2_ETS_TYPE_EMC)
		return YUSUR2_NOT_IMPLEMENTED;

	low_thresh_delta = ((ets_cfg & YUSUR2_ETS_LTHRES_DELTA_MASK) >>
			     YUSUR2_ETS_LTHRES_DELTA_SHIFT);
	num_sensors = (ets_cfg & YUSUR2_ETS_NUM_SENSORS_MASK);

	for (i = 0; i < num_sensors; i++) {
		offset = ets_offset + 1 + i;
		if (hw->eeprom.ops.read(hw, offset, &ets_sensor)) {
			ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
				      "eeprom read at offset %d failed",
				      offset);
			continue;
		}
		sensor_index = ((ets_sensor & YUSUR2_ETS_DATA_INDEX_MASK) >>
				YUSUR2_ETS_DATA_INDEX_SHIFT);
		sensor_location = ((ets_sensor & YUSUR2_ETS_DATA_LOC_MASK) >>
				   YUSUR2_ETS_DATA_LOC_SHIFT);
		therm_limit = ets_sensor & YUSUR2_ETS_DATA_HTHRESH_MASK;

		hw->phy.ops.write_i2c_byte(hw,
			yusur2_emc_therm_limit[sensor_index],
			YUSUR2_I2C_THERMAL_SENSOR_ADDR, therm_limit);

		if ((i < YUSUR2_MAX_SENSORS) && (sensor_location != 0)) {
			data->sensor[i].location = sensor_location;
			data->sensor[i].caution_thresh = therm_limit;
			data->sensor[i].max_op_thresh = therm_limit -
							low_thresh_delta;
		}
	}
#endif
	return status;
#if 0
eeprom_err:
	ERROR_REPORT2(YUSUR2_ERROR_INVALID_STATE,
		      "eeprom read at offset %d failed", offset);
	return YUSUR2_NOT_IMPLEMENTED;
#endif
}

/**
 *  yusur2_get_orom_version - Return option ROM from EEPROM
 *
 *  @hw: pointer to hardware structure
 *  @nvm_ver: pointer to output structure
 *
 *  if valid option ROM version, nvm_ver->or_valid set to true
 *  else nvm_ver->or_valid is false.
 **/
void yusur2_get_orom_version(struct yusur2_hw *hw,
			    struct yusur2_nvm_version *nvm_ver)
{
	u16 offset, eeprom_cfg_blkh, eeprom_cfg_blkl;

	nvm_ver->or_valid = false;
	/* Option Rom may or may not be present.  Start with pointer */
	hw->eeprom.ops.read(hw, NVM_OROM_OFFSET, &offset);

	/* make sure offset is valid */
	if ((offset == 0x0) || (offset == NVM_INVALID_PTR))
		return;

	hw->eeprom.ops.read(hw, offset + NVM_OROM_BLK_HI, &eeprom_cfg_blkh);
	hw->eeprom.ops.read(hw, offset + NVM_OROM_BLK_LOW, &eeprom_cfg_blkl);

	/* option rom exists and is valid */
	if ((eeprom_cfg_blkl | eeprom_cfg_blkh) == 0x0 ||
	    eeprom_cfg_blkl == NVM_VER_INVALID ||
	    eeprom_cfg_blkh == NVM_VER_INVALID)
		return;

	nvm_ver->or_valid = true;
	nvm_ver->or_major = eeprom_cfg_blkl >> NVM_OROM_SHIFT;
	nvm_ver->or_build = (eeprom_cfg_blkl << NVM_OROM_SHIFT) |
			    (eeprom_cfg_blkh >> NVM_OROM_SHIFT);
	nvm_ver->or_patch = eeprom_cfg_blkh & NVM_OROM_PATCH_MASK;
}

/**
 *  yusur2_get_oem_prod_version - Return OEM Product version
 *
 *  @hw: pointer to hardware structure
 *  @nvm_ver: pointer to output structure
 *
 *  if valid OEM product version, nvm_ver->oem_valid set to true
 *  else nvm_ver->oem_valid is false.
 **/
void yusur2_get_oem_prod_version(struct yusur2_hw *hw,
				struct yusur2_nvm_version *nvm_ver)
{
	u16 rel_num, prod_ver, mod_len, cap, offset;

	nvm_ver->oem_valid = false;
	hw->eeprom.ops.read(hw, NVM_OEM_PROD_VER_PTR, &offset);

	/* Return is offset to OEM Product Version block is invalid */
	if (offset == 0x0 || offset == NVM_INVALID_PTR)
		return;

	/* Read product version block */
	hw->eeprom.ops.read(hw, offset, &mod_len);
	hw->eeprom.ops.read(hw, offset + NVM_OEM_PROD_VER_CAP_OFF, &cap);

	/* Return if OEM product version block is invalid */
	if (mod_len != NVM_OEM_PROD_VER_MOD_LEN ||
	    (cap & NVM_OEM_PROD_VER_CAP_MASK) != 0x0)
		return;

	hw->eeprom.ops.read(hw, offset + NVM_OEM_PROD_VER_OFF_L, &prod_ver);
	hw->eeprom.ops.read(hw, offset + NVM_OEM_PROD_VER_OFF_H, &rel_num);

	/* Return if version is invalid */
	if ((rel_num | prod_ver) == 0x0 ||
	    rel_num == NVM_VER_INVALID || prod_ver == NVM_VER_INVALID)
		return;

	nvm_ver->oem_major = prod_ver >> NVM_VER_SHIFT;
	nvm_ver->oem_minor = prod_ver & NVM_VER_MASK;
	nvm_ver->oem_release = rel_num;
	nvm_ver->oem_valid = true;
}

/**
 *  yusur2_get_etk_id - Return Etrack ID from EEPROM
 *
 *  @hw: pointer to hardware structure
 *  @nvm_ver: pointer to output structure
 *
 *  word read errors will return 0xFFFF
 **/
void yusur2_get_etk_id(struct yusur2_hw *hw, struct yusur2_nvm_version *nvm_ver)
{
	u16 etk_id_l, etk_id_h;

	if (hw->eeprom.ops.read(hw, NVM_ETK_OFF_LOW, &etk_id_l))
		etk_id_l = NVM_VER_INVALID;
	if (hw->eeprom.ops.read(hw, NVM_ETK_OFF_HI, &etk_id_h))
		etk_id_h = NVM_VER_INVALID;

	/* The word order for the version format is determined by high order
	 * word bit 15.
	 */
	if ((etk_id_h & NVM_ETK_VALID) == 0) {
		nvm_ver->etk_id = etk_id_h;
		nvm_ver->etk_id |= (etk_id_l << NVM_ETK_SHIFT);
	} else {
		nvm_ver->etk_id = etk_id_l;
		nvm_ver->etk_id |= (etk_id_h << NVM_ETK_SHIFT);
	}
}


/**
 * yusur2_dcb_get_rtrup2tc_generic - read rtrup2tc reg
 * @hw: pointer to hardware structure
 * @map: pointer to u8 arr for returning map
 *
 * Read the rtrup2tc HW register and resolve its content into map
 **/
void yusur2_dcb_get_rtrup2tc_generic(struct yusur2_hw *hw, u8 *map)
{
//TODO:
#if 0
	u32 reg, i;

	reg = YUSUR2_READ_REG(hw, YUSUR2_RTRUP2TC);
	for (i = 0; i < YUSUR2_DCB_MAX_USER_PRIORITY; i++)
		map[i] = YUSUR2_RTRUP2TC_UP_MASK &
			(reg >> (i * YUSUR2_RTRUP2TC_UP_SHIFT));
	return;
#endif
}

void yusur2_disable_rx_generic(struct yusur2_hw *hw)
{
//TODO:
#if 0
	u32 pfdtxgswc;
	u32 rxctrl;

	rxctrl = YUSUR2_READ_REG(hw, YUSUR2_RXCTRL);
	if (rxctrl & YUSUR2_RXCTRL_RXEN) {
		if (hw->mac.type != yusur2_mac_82598EB) {
			pfdtxgswc = YUSUR2_READ_REG(hw, YUSUR2_PFDTXGSWC);
			if (pfdtxgswc & YUSUR2_PFDTXGSWC_VT_LBEN) {
				pfdtxgswc &= ~YUSUR2_PFDTXGSWC_VT_LBEN;
				YUSUR2_WRITE_REG(hw, YUSUR2_PFDTXGSWC, pfdtxgswc);
				hw->mac.set_lben = true;
			} else {
				hw->mac.set_lben = false;
			}
		}
		rxctrl &= ~YUSUR2_RXCTRL_RXEN;
		YUSUR2_WRITE_REG(hw, YUSUR2_RXCTRL, rxctrl);
	}
#endif
}

void yusur2_enable_rx_generic(struct yusur2_hw *hw)
{
//TODO:
#if 0
	u32 pfdtxgswc;
	u32 rxctrl;

	rxctrl = YUSUR2_READ_REG(hw, YUSUR2_RXCTRL);
	YUSUR2_WRITE_REG(hw, YUSUR2_RXCTRL, (rxctrl | YUSUR2_RXCTRL_RXEN));

	if (hw->mac.type != yusur2_mac_82598EB) {
		if (hw->mac.set_lben) {
			pfdtxgswc = YUSUR2_READ_REG(hw, YUSUR2_PFDTXGSWC);
			pfdtxgswc |= YUSUR2_PFDTXGSWC_VT_LBEN;
			YUSUR2_WRITE_REG(hw, YUSUR2_PFDTXGSWC, pfdtxgswc);
			hw->mac.set_lben = false;
		}
	}
#endif
}

/**
 * yusur2_mng_present - returns true when management capability is present
 * @hw: pointer to hardware structure
 */
bool yusur2_mng_present(struct yusur2_hw *hw)
{
//TODO:
#if 0
	u32 fwsm;

	if (hw->mac.type < yusur2_mac_82599EB)
		return false;

	fwsm = YUSUR2_READ_REG(hw, YUSUR2_FWSM_BY_MAC(hw));

	return !!(fwsm & YUSUR2_FWSM_FW_MODE_PT);
#endif
	return false;
}

/**
 * yusur2_mng_enabled - Is the manageability engine enabled?
 * @hw: pointer to hardware structure
 *
 * Returns true if the manageability engine is enabled.
 **/
bool yusur2_mng_enabled(struct yusur2_hw *hw)
{
//TODO:
#if 0
	u32 fwsm, manc, factps;

	fwsm = YUSUR2_READ_REG(hw, YUSUR2_FWSM_BY_MAC(hw));
	if ((fwsm & YUSUR2_FWSM_MODE_MASK) != YUSUR2_FWSM_FW_MODE_PT)
		return false;

	manc = YUSUR2_READ_REG(hw, YUSUR2_MANC);
	if (!(manc & YUSUR2_MANC_RCV_TCO_EN))
		return false;

	if (hw->mac.type <= yusur2_mac_X540) {
		factps = YUSUR2_READ_REG(hw, YUSUR2_FACTPS_BY_MAC(hw));
		if (factps & YUSUR2_FACTPS_MNGCG)
			return false;
	}
#endif
	return true;
}

/**
 *  yusur2_setup_mac_link_multispeed_fiber - Set MAC link speed
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Set the link speed in the MAC and/or PHY register and restarts link.
 **/
s32 yusur2_setup_mac_link_multispeed_fiber(struct yusur2_hw *hw,
					  yusur2_link_speed speed,
					  bool autoneg_wait_to_complete)
{
	yusur2_link_speed link_speed = YUSUR2_LINK_SPEED_UNKNOWN;
	yusur2_link_speed highest_link_speed = YUSUR2_LINK_SPEED_UNKNOWN;
	s32 status = YUSUR2_SUCCESS;
	u32 speedcnt = 0;
	u32 i = 0;
	bool autoneg, link_up = false;

	DEBUGFUNC("yusur2_setup_mac_link_multispeed_fiber");

	/* Mask off requested but non-supported speeds */
	status = yusur2_get_link_capabilities(hw, &link_speed, &autoneg);
	if (status != YUSUR2_SUCCESS)
		return status;

	speed &= link_speed;

	/* Try each speed one by one, highest priority first.  We do this in
	 * software because 10Gb fiber doesn't support speed autonegotiation.
	 */
	if (speed & YUSUR2_LINK_SPEED_10GB_FULL) {
		speedcnt++;
		highest_link_speed = YUSUR2_LINK_SPEED_10GB_FULL;

		/* Set the module link speed */
		switch (hw->phy.media_type) {
		case yusur2_media_type_fiber:
			yusur2_set_rate_select_speed(hw,
						    YUSUR2_LINK_SPEED_10GB_FULL);
			break;
		case yusur2_media_type_fiber_qsfp:
			/* QSFP module automatically detects MAC link speed */
			break;
		default:
			DEBUGOUT("Unexpected media type.\n");
			break;
		}

		/* Allow module to change analog characteristics (1G->10G) */
		msec_delay(40);

		status = yusur2_setup_mac_link(hw,
					      YUSUR2_LINK_SPEED_10GB_FULL,
					      autoneg_wait_to_complete);
		if (status != YUSUR2_SUCCESS)
			return status;

		/* Flap the Tx laser if it has not already been done */
		yusur2_flap_tx_laser(hw);

		/* Wait for the controller to acquire link.  Per IEEE 802.3ap,
		 * Section 73.10.2, we may have to wait up to 1000ms if KR is
		 * attempted.  82599 uses the same timing for 10g SFI.
		 */
		for (i = 0; i < 10; i++) {
			/* Wait for the link partner to also set speed */
			msec_delay(100);

			/* If we have link, just jump out */
			status = yusur2_check_link(hw, &link_speed,
						  &link_up, false);
			if (status != YUSUR2_SUCCESS)
				return status;

			if (link_up)
				goto out;
		}
	}

	if (speed & YUSUR2_LINK_SPEED_1GB_FULL) {
		speedcnt++;
		if (highest_link_speed == YUSUR2_LINK_SPEED_UNKNOWN)
			highest_link_speed = YUSUR2_LINK_SPEED_1GB_FULL;

		/* Set the module link speed */
		switch (hw->phy.media_type) {
		case yusur2_media_type_fiber:
			yusur2_set_rate_select_speed(hw,
						    YUSUR2_LINK_SPEED_1GB_FULL);
			break;
		case yusur2_media_type_fiber_qsfp:
			/* QSFP module automatically detects link speed */
			break;
		default:
			DEBUGOUT("Unexpected media type.\n");
			break;
		}

		/* Allow module to change analog characteristics (10G->1G) */
		msec_delay(40);

		status = yusur2_setup_mac_link(hw,
					      YUSUR2_LINK_SPEED_1GB_FULL,
					      autoneg_wait_to_complete);
		if (status != YUSUR2_SUCCESS)
			return status;

		/* Flap the Tx laser if it has not already been done */
		yusur2_flap_tx_laser(hw);

		/* Wait for the link partner to also set speed */
		msec_delay(100);

		/* If we have link, just jump out */
		status = yusur2_check_link(hw, &link_speed, &link_up, false);
		if (status != YUSUR2_SUCCESS)
			return status;

		if (link_up)
			goto out;
	}

	/* We didn't get link.  Configure back to the highest speed we tried,
	 * (if there was more than one).  We call ourselves back with just the
	 * single highest speed that the user requested.
	 */
	if (speedcnt > 1)
		status = yusur2_setup_mac_link_multispeed_fiber(hw,
						      highest_link_speed,
						      autoneg_wait_to_complete);

out:
	/* Set autoneg_advertised value based on input link speed */
	hw->phy.autoneg_advertised = 0;

	if (speed & YUSUR2_LINK_SPEED_10GB_FULL)
		hw->phy.autoneg_advertised |= YUSUR2_LINK_SPEED_10GB_FULL;

	if (speed & YUSUR2_LINK_SPEED_1GB_FULL)
		hw->phy.autoneg_advertised |= YUSUR2_LINK_SPEED_1GB_FULL;

	return status;
}

/**
 *  yusur2_set_soft_rate_select_speed - Set module link speed
 *  @hw: pointer to hardware structure
 *  @speed: link speed to set
 *
 *  Set module link speed via the soft rate select.
 */
void yusur2_set_soft_rate_select_speed(struct yusur2_hw *hw,
					yusur2_link_speed speed)
{
	s32 status;
	u8 rs, eeprom_data;

	switch (speed) {
	case YUSUR2_LINK_SPEED_10GB_FULL:
		/* one bit mask same as setting on */
		rs = YUSUR2_SFF_SOFT_RS_SELECT_10G;
		break;
	case YUSUR2_LINK_SPEED_1GB_FULL:
		rs = YUSUR2_SFF_SOFT_RS_SELECT_1G;
		break;
	default:
		DEBUGOUT("Invalid fixed module speed\n");
		return;
	}

	/* Set RS0 */
	status = hw->phy.ops.read_i2c_byte(hw, YUSUR2_SFF_SFF_8472_OSCB,
					   YUSUR2_I2C_EEPROM_DEV_ADDR2,
					   &eeprom_data);
	if (status) {
		DEBUGOUT("Failed to read Rx Rate Select RS0\n");
		goto out;
	}

	eeprom_data = (eeprom_data & ~YUSUR2_SFF_SOFT_RS_SELECT_MASK) | rs;

	status = hw->phy.ops.write_i2c_byte(hw, YUSUR2_SFF_SFF_8472_OSCB,
					    YUSUR2_I2C_EEPROM_DEV_ADDR2,
					    eeprom_data);
	if (status) {
		DEBUGOUT("Failed to write Rx Rate Select RS0\n");
		goto out;
	}

	/* Set RS1 */
	status = hw->phy.ops.read_i2c_byte(hw, YUSUR2_SFF_SFF_8472_ESCB,
					   YUSUR2_I2C_EEPROM_DEV_ADDR2,
					   &eeprom_data);
	if (status) {
		DEBUGOUT("Failed to read Rx Rate Select RS1\n");
		goto out;
	}

	eeprom_data = (eeprom_data & ~YUSUR2_SFF_SOFT_RS_SELECT_MASK) | rs;

	status = hw->phy.ops.write_i2c_byte(hw, YUSUR2_SFF_SFF_8472_ESCB,
					    YUSUR2_I2C_EEPROM_DEV_ADDR2,
					    eeprom_data);
	if (status) {
		DEBUGOUT("Failed to write Rx Rate Select RS1\n");
		goto out;
	}
out:
	return;
}
