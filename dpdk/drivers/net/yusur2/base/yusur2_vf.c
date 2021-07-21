/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */


#include "yusur2_api.h"
#include "yusur2_type.h"
#include "yusur2_vf.h"

#ifndef YUSUR2_VFWRITE_REG
#define YUSUR2_VFWRITE_REG YUSUR2_WRITE_REG
#endif
#ifndef YUSUR2_VFREAD_REG
#define YUSUR2_VFREAD_REG YUSUR2_READ_REG
#endif

/**
 *  yusur2_init_ops_vf - Initialize the pointers for vf
 *  @hw: pointer to hardware structure
 *
 *  This will assign function pointers, adapter-specific functions can
 *  override the assignment of generic function pointers by assigning
 *  their own adapter-specific function pointers.
 *  Does not touch the hardware.
 **/
s32 yusur2_init_ops_vf(struct yusur2_hw *hw)
{
	/* MAC */
	hw->mac.ops.init_hw = yusur2_init_hw_vf;
	hw->mac.ops.reset_hw = yusur2_reset_hw_vf;
	hw->mac.ops.start_hw = yusur2_start_hw_vf;
	/* Cannot clear stats on VF */
	hw->mac.ops.clear_hw_cntrs = NULL;
	hw->mac.ops.get_media_type = NULL;
	hw->mac.ops.get_mac_addr = yusur2_get_mac_addr_vf;
	hw->mac.ops.stop_adapter = yusur2_stop_adapter_vf;
	hw->mac.ops.get_bus_info = NULL;
	hw->mac.ops.negotiate_api_version = yusur2vf_negotiate_api_version;

	/* Link */
	hw->mac.ops.setup_link = yusur2_setup_mac_link_vf;
	hw->mac.ops.check_link = yusur2_check_mac_link_vf;
	hw->mac.ops.get_link_capabilities = NULL;

	/* RAR, Multicast, VLAN */
	hw->mac.ops.set_rar = yusur2_set_rar_vf;
	hw->mac.ops.set_uc_addr = yusur2vf_set_uc_addr_vf;
	hw->mac.ops.init_rx_addrs = NULL;
	hw->mac.ops.update_mc_addr_list = yusur2_update_mc_addr_list_vf;
	hw->mac.ops.update_xcast_mode = yusur2vf_update_xcast_mode;
	hw->mac.ops.enable_mc = NULL;
	hw->mac.ops.disable_mc = NULL;
	hw->mac.ops.clear_vfta = NULL;
	hw->mac.ops.set_vfta = yusur2_set_vfta_vf;
	hw->mac.ops.set_rlpml = yusur2vf_rlpml_set_vf;

	hw->mac.max_tx_queues = 1;
	hw->mac.max_rx_queues = 1;

	hw->mbx.ops.init_params = yusur2_init_mbx_params_vf;

	return YUSUR2_SUCCESS;
}

/* yusur2_virt_clr_reg - Set register to default (power on) state.
 *  @hw: pointer to hardware structure
 */
static void yusur2_virt_clr_reg(struct yusur2_hw *hw)
{
	int i;
	u32 vfsrrctl;
	u32 vfdca_rxctrl;
	u32 vfdca_txctrl;

	/* VRSRRCTL default values (BSIZEPACKET = 2048, BSIZEHEADER = 256) */
	vfsrrctl = 0x100 << YUSUR2_SRRCTL_BSIZEHDRSIZE_SHIFT;
	vfsrrctl |= 0x800 >> YUSUR2_SRRCTL_BSIZEPKT_SHIFT;

	/* DCA_RXCTRL default value */
	vfdca_rxctrl = YUSUR2_DCA_RXCTRL_DESC_RRO_EN |
		       YUSUR2_DCA_RXCTRL_DATA_WRO_EN |
		       YUSUR2_DCA_RXCTRL_HEAD_WRO_EN;

	/* DCA_TXCTRL default value */
	vfdca_txctrl = YUSUR2_DCA_TXCTRL_DESC_RRO_EN |
		       YUSUR2_DCA_TXCTRL_DESC_WRO_EN |
		       YUSUR2_DCA_TXCTRL_DATA_RRO_EN;

	YUSUR2_WRITE_REG(hw, YUSUR2_VFPSRTYPE, 0);

	for (i = 0; i < 7; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_VFRDH(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFRDT(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFRXDCTL(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFSRRCTL(i), vfsrrctl);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTDH(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTDT(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTXDCTL(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTDWBAH(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTDWBAL(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFDCA_RXCTRL(i), vfdca_rxctrl);
		YUSUR2_WRITE_REG(hw, YUSUR2_VFDCA_TXCTRL(i), vfdca_txctrl);
	}

	YUSUR2_WRITE_FLUSH(hw);
}

/**
 *  yusur2_start_hw_vf - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware by filling the bus info structure and media type, clears
 *  all on chip counters, initializes receive address registers, multicast
 *  table, VLAN filter table, calls routine to set up link and flow control
 *  settings, and leaves transmit and receive units disabled and uninitialized
 **/
s32 yusur2_start_hw_vf(struct yusur2_hw *hw)
{
	/* Clear adapter stopped flag */
	hw->adapter_stopped = false;

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_init_hw_vf - virtual function hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware and then starting
 *  the hardware
 **/
s32 yusur2_init_hw_vf(struct yusur2_hw *hw)
{
	s32 status = hw->mac.ops.start_hw(hw);

	hw->mac.ops.get_mac_addr(hw, hw->mac.addr);

	return status;
}

/**
 *  yusur2_reset_hw_vf - Performs hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by reseting the transmit and receive units, masks and
 *  clears all interrupts.
 **/
s32 yusur2_reset_hw_vf(struct yusur2_hw *hw)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	u32 timeout = YUSUR2_VF_INIT_TIMEOUT;
	s32 ret_val = YUSUR2_ERR_INVALID_MAC_ADDR;
	u32 msgbuf[YUSUR2_VF_PERMADDR_MSG_LEN];
	u8 *addr = (u8 *)(&msgbuf[1]);

	DEBUGFUNC("yusur2vf_reset_hw_vf");

	/* Call adapter stop to disable tx/rx and clear interrupts */
	hw->mac.ops.stop_adapter(hw);

	/* reset the api version */
	hw->api_version = yusur2_mbox_api_10;

	DEBUGOUT("Issuing a function level reset to MAC\n");

	YUSUR2_VFWRITE_REG(hw, YUSUR2_VFCTRL, YUSUR2_CTRL_RST);
	YUSUR2_WRITE_FLUSH(hw);

	msec_delay(50);

	/* we cannot reset while the RSTI / RSTD bits are asserted */
	while (!mbx->ops.check_for_rst(hw, 0) && timeout) {
		timeout--;
		usec_delay(5);
	}

	if (!timeout)
		return YUSUR2_ERR_RESET_FAILED;

	/* Reset VF registers to initial values */
	yusur2_virt_clr_reg(hw);

	/* mailbox timeout can now become active */
	mbx->timeout = YUSUR2_VF_MBX_INIT_TIMEOUT;

	msgbuf[0] = YUSUR2_VF_RESET;
	mbx->ops.write_posted(hw, msgbuf, 1, 0);

	msec_delay(10);

	/*
	 * set our "perm_addr" based on info provided by PF
	 * also set up the mc_filter_type which is piggy backed
	 * on the mac address in word 3
	 */
	ret_val = mbx->ops.read_posted(hw, msgbuf,
			YUSUR2_VF_PERMADDR_MSG_LEN, 0);
	if (ret_val)
		return ret_val;

	if (msgbuf[0] != (YUSUR2_VF_RESET | YUSUR2_VT_MSGTYPE_ACK) &&
	    msgbuf[0] != (YUSUR2_VF_RESET | YUSUR2_VT_MSGTYPE_NACK))
		return YUSUR2_ERR_INVALID_MAC_ADDR;

	if (msgbuf[0] == (YUSUR2_VF_RESET | YUSUR2_VT_MSGTYPE_ACK))
		memcpy(hw->mac.perm_addr, addr, YUSUR2_ETH_LENGTH_OF_ADDRESS);

	hw->mac.mc_filter_type = msgbuf[YUSUR2_VF_MC_TYPE_WORD];

	return ret_val;
}

/**
 *  yusur2_stop_adapter_vf - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within yusur2_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
s32 yusur2_stop_adapter_vf(struct yusur2_hw *hw)
{
	u32 reg_val;
	u16 i;

	/*
	 * Set the adapter_stopped flag so other driver functions stop touching
	 * the hardware
	 */
	hw->adapter_stopped = true;

	/* Clear interrupt mask to stop from interrupts being generated */
	YUSUR2_VFWRITE_REG(hw, YUSUR2_VTEIMC, YUSUR2_VF_IRQ_CLEAR_MASK);

	/* Clear any pending interrupts, flush previous writes */
	YUSUR2_VFREAD_REG(hw, YUSUR2_VTEICR);

	/* Disable the transmit unit.  Each queue must be disabled. */
	for (i = 0; i < hw->mac.max_tx_queues; i++)
		YUSUR2_VFWRITE_REG(hw, YUSUR2_VFTXDCTL(i), YUSUR2_TXDCTL_SWFLSH);

	/* Disable the receive unit by stopping each queue */
	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		reg_val = YUSUR2_VFREAD_REG(hw, YUSUR2_VFRXDCTL(i));
		reg_val &= ~YUSUR2_RXDCTL_ENABLE;
		YUSUR2_VFWRITE_REG(hw, YUSUR2_VFRXDCTL(i), reg_val);
	}
	/* Clear packet split and pool config */
	YUSUR2_WRITE_REG(hw, YUSUR2_VFPSRTYPE, 0);

	/* flush all queues disables */
	YUSUR2_WRITE_FLUSH(hw);
	msec_delay(2);

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

STATIC s32 yusur2vf_write_msg_read_ack(struct yusur2_hw *hw, u32 *msg,
				      u32 *retmsg, u16 size)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 retval = mbx->ops.write_posted(hw, msg, size, 0);

	if (retval)
		return retval;

	return mbx->ops.read_posted(hw, retmsg, size, 0);
}

/**
 *  yusur2_set_rar_vf - set device MAC address
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 **/
s32 yusur2_set_rar_vf(struct yusur2_hw *hw, u32 index, u8 *addr, u32 vmdq,
		     u32 enable_addr)
{
	u32 msgbuf[3];
	u8 *msg_addr = (u8 *)(&msgbuf[1]);
	s32 ret_val;
	UNREFERENCED_3PARAMETER(vmdq, enable_addr, index);

	memset(msgbuf, 0, 12);
	msgbuf[0] = YUSUR2_VF_SET_MAC_ADDR;
	memcpy(msg_addr, addr, 6);
	ret_val = yusur2vf_write_msg_read_ack(hw, msgbuf, msgbuf, 3);

	msgbuf[0] &= ~YUSUR2_VT_MSGTYPE_CTS;

	/* if nacked the address was rejected, use "perm_addr" */
	if (!ret_val &&
	    (msgbuf[0] == (YUSUR2_VF_SET_MAC_ADDR | YUSUR2_VT_MSGTYPE_NACK))) {
		yusur2_get_mac_addr_vf(hw, hw->mac.addr);
		return YUSUR2_ERR_MBX;
	}

	return ret_val;
}

/**
 *  yusur2_update_mc_addr_list_vf - Update Multicast addresses
 *  @hw: pointer to the HW structure
 *  @mc_addr_list: array of multicast addresses to program
 *  @mc_addr_count: number of multicast addresses to program
 *  @next: caller supplied function to return next address in list
 *  @clear: unused
 *
 *  Updates the Multicast Table Array.
 **/
s32 yusur2_update_mc_addr_list_vf(struct yusur2_hw *hw, u8 *mc_addr_list,
				 u32 mc_addr_count, yusur2_mc_addr_itr next,
				 bool clear)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	u32 msgbuf[YUSUR2_VFMAILBOX_SIZE];
	u16 *vector_list = (u16 *)&msgbuf[1];
	u32 vector;
	u32 cnt, i;
	u32 vmdq;

	UNREFERENCED_1PARAMETER(clear);

	DEBUGFUNC("yusur2_update_mc_addr_list_vf");

	/* Each entry in the list uses 1 16 bit word.  We have 30
	 * 16 bit words available in our HW msg buffer (minus 1 for the
	 * msg type).  That's 30 hash values if we pack 'em right.  If
	 * there are more than 30 MC addresses to add then punt the
	 * extras for now and then add code to handle more than 30 later.
	 * It would be unusual for a server to request that many multi-cast
	 * addresses except for in large enterprise network environments.
	 */

	DEBUGOUT1("MC Addr Count = %d\n", mc_addr_count);

	cnt = (mc_addr_count > 30) ? 30 : mc_addr_count;
	msgbuf[0] = YUSUR2_VF_SET_MULTICAST;
	msgbuf[0] |= cnt << YUSUR2_VT_MSGINFO_SHIFT;

	for (i = 0; i < cnt; i++) {
		vector = yusur2_mta_vector(hw, next(hw, &mc_addr_list, &vmdq));
		DEBUGOUT1("Hash value = 0x%03X\n", vector);
		vector_list[i] = (u16)vector;
	}

	return mbx->ops.write_posted(hw, msgbuf, YUSUR2_VFMAILBOX_SIZE, 0);
}

/**
 *  yusur2vf_update_xcast_mode - Update Multicast mode
 *  @hw: pointer to the HW structure
 *  @xcast_mode: new multicast mode
 *
 *  Updates the Multicast Mode of VF.
 **/
s32 yusur2vf_update_xcast_mode(struct yusur2_hw *hw, int xcast_mode)
{
	u32 msgbuf[2];
	s32 err;

	switch (hw->api_version) {
	case yusur2_mbox_api_12:
		/* New modes were introduced in 1.3 version */
		if (xcast_mode > YUSUR2VF_XCAST_MODE_ALLMULTI)
			return YUSUR2_ERR_FEATURE_NOT_SUPPORTED;
		/* Fall through */
	case yusur2_mbox_api_13:
		break;
	default:
		return YUSUR2_ERR_FEATURE_NOT_SUPPORTED;
	}

	msgbuf[0] = YUSUR2_VF_UPDATE_XCAST_MODE;
	msgbuf[1] = xcast_mode;

	err = yusur2vf_write_msg_read_ack(hw, msgbuf, msgbuf, 2);
	if (err)
		return err;

	msgbuf[0] &= ~YUSUR2_VT_MSGTYPE_CTS;
	if (msgbuf[0] == (YUSUR2_VF_UPDATE_XCAST_MODE | YUSUR2_VT_MSGTYPE_NACK))
		return YUSUR2_ERR_FEATURE_NOT_SUPPORTED;
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_set_vfta_vf - Set/Unset vlan filter table address
 *  @hw: pointer to the HW structure
 *  @vlan: 12 bit VLAN ID
 *  @vind: unused by VF drivers
 *  @vlan_on: if true then set bit, else clear bit
 *  @vlvf_bypass: boolean flag indicating updating default pool is okay
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 yusur2_set_vfta_vf(struct yusur2_hw *hw, u32 vlan, u32 vind,
		      bool vlan_on, bool vlvf_bypass)
{
	u32 msgbuf[2];
	s32 ret_val;
	UNREFERENCED_2PARAMETER(vind, vlvf_bypass);

	msgbuf[0] = YUSUR2_VF_SET_VLAN;
	msgbuf[1] = vlan;
	/* Setting the 8 bit field MSG INFO to TRUE indicates "add" */
	msgbuf[0] |= vlan_on << YUSUR2_VT_MSGINFO_SHIFT;

	ret_val = yusur2vf_write_msg_read_ack(hw, msgbuf, msgbuf, 2);
	if (!ret_val && (msgbuf[0] & YUSUR2_VT_MSGTYPE_ACK))
		return YUSUR2_SUCCESS;

	return ret_val | (msgbuf[0] & YUSUR2_VT_MSGTYPE_NACK);
}

/**
 *  yusur2_get_num_of_tx_queues_vf - Get number of TX queues
 *  @hw: pointer to hardware structure
 *
 *  Returns the number of transmit queues for the given adapter.
 **/
u32 yusur2_get_num_of_tx_queues_vf(struct yusur2_hw *hw)
{
	UNREFERENCED_1PARAMETER(hw);
	return YUSUR2_VF_MAX_TX_QUEUES;
}

/**
 *  yusur2_get_num_of_rx_queues_vf - Get number of RX queues
 *  @hw: pointer to hardware structure
 *
 *  Returns the number of receive queues for the given adapter.
 **/
u32 yusur2_get_num_of_rx_queues_vf(struct yusur2_hw *hw)
{
	UNREFERENCED_1PARAMETER(hw);
	return YUSUR2_VF_MAX_RX_QUEUES;
}

/**
 * yusur2_get_mac_addr_vf - Read device MAC address
 * @hw: pointer to the HW structure
 * @mac_addr: the MAC address
 **/
s32 yusur2_get_mac_addr_vf(struct yusur2_hw *hw, u8 *mac_addr)
{
	int i;

	for (i = 0; i < YUSUR2_ETH_LENGTH_OF_ADDRESS; i++)
		mac_addr[i] = hw->mac.perm_addr[i];

	return YUSUR2_SUCCESS;
}

s32 yusur2vf_set_uc_addr_vf(struct yusur2_hw *hw, u32 index, u8 *addr)
{
	u32 msgbuf[3], msgbuf_chk;
	u8 *msg_addr = (u8 *)(&msgbuf[1]);
	s32 ret_val;

	memset(msgbuf, 0, sizeof(msgbuf));
	/*
	 * If index is one then this is the start of a new list and needs
	 * indication to the PF so it can do it's own list management.
	 * If it is zero then that tells the PF to just clear all of
	 * this VF's macvlans and there is no new list.
	 */
	msgbuf[0] |= index << YUSUR2_VT_MSGINFO_SHIFT;
	msgbuf[0] |= YUSUR2_VF_SET_MACVLAN;
	msgbuf_chk = msgbuf[0];
	if (addr)
		memcpy(msg_addr, addr, 6);

	ret_val = yusur2vf_write_msg_read_ack(hw, msgbuf, msgbuf, 3);
	if (!ret_val) {
		msgbuf[0] &= ~YUSUR2_VT_MSGTYPE_CTS;

		if (msgbuf[0] == (msgbuf_chk | YUSUR2_VT_MSGTYPE_NACK))
			return YUSUR2_ERR_OUT_OF_MEM;
	}

	return ret_val;
}

/**
 *  yusur2_setup_mac_link_vf - Setup MAC link settings
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Set the link speed in the AUTOC register and restarts link.
 **/
s32 yusur2_setup_mac_link_vf(struct yusur2_hw *hw, yusur2_link_speed speed,
			    bool autoneg_wait_to_complete)
{
	UNREFERENCED_3PARAMETER(hw, speed, autoneg_wait_to_complete);
	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_check_mac_link_vf - Get link/speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true is link is up, false otherwise
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 yusur2_check_mac_link_vf(struct yusur2_hw *hw, yusur2_link_speed *speed,
			    bool *link_up, bool autoneg_wait_to_complete)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	struct yusur2_mac_info *mac = &hw->mac;
	s32 ret_val = YUSUR2_SUCCESS;
	u32 links_reg;
	u32 in_msg = 0;
	UNREFERENCED_1PARAMETER(autoneg_wait_to_complete);

	/* If we were hit with a reset drop the link */
	if (!mbx->ops.check_for_rst(hw, 0) || !mbx->timeout)
		mac->get_link_status = true;

	if (!mac->get_link_status)
		goto out;

	/* if link status is down no point in checking to see if pf is up */
	links_reg = YUSUR2_READ_REG(hw, YUSUR2_VFLINKS);
	if (!(links_reg & YUSUR2_LINKS_UP))
		goto out;

	/* for SFP+ modules and DA cables on 82599 it can take up to 500usecs
	 * before the link status is correct
	 */
	if (mac->type == yusur2_mac_82599_vf) {
		int i;

		for (i = 0; i < 5; i++) {
			usec_delay(100);
			links_reg = YUSUR2_READ_REG(hw, YUSUR2_VFLINKS);

			if (!(links_reg & YUSUR2_LINKS_UP))
				goto out;
		}
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
		/* Since Reserved in older MAC's */
		if (hw->mac.type >= yusur2_mac_X550)
			*speed = YUSUR2_LINK_SPEED_10_FULL;
		break;
	default:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
	}

	/* if the read failed it could just be a mailbox collision, best wait
	 * until we are called again and don't report an error
	 */
	if (mbx->ops.read(hw, &in_msg, 1, 0))
		goto out;

	if (!(in_msg & YUSUR2_VT_MSGTYPE_CTS)) {
		/* msg is not CTS and is NACK we must have lost CTS status */
		if (in_msg & YUSUR2_VT_MSGTYPE_NACK)
			ret_val = -1;
		goto out;
	}

	/* the pf is talking, if we timed out in the past we reinit */
	if (!mbx->timeout) {
		ret_val = -1;
		goto out;
	}

	/* if we passed all the tests above then the link is up and we no
	 * longer need to check for link
	 */
	mac->get_link_status = false;

out:
	*link_up = !mac->get_link_status;
	return ret_val;
}

/**
 *  yusur2vf_rlpml_set_vf - Set the maximum receive packet length
 *  @hw: pointer to the HW structure
 *  @max_size: value to assign to max frame size
 **/
s32 yusur2vf_rlpml_set_vf(struct yusur2_hw *hw, u16 max_size)
{
	u32 msgbuf[2];
	s32 retval;

	msgbuf[0] = YUSUR2_VF_SET_LPE;
	msgbuf[1] = max_size;

	retval = yusur2vf_write_msg_read_ack(hw, msgbuf, msgbuf, 2);
	if (retval)
		return retval;
	if ((msgbuf[0] & YUSUR2_VF_SET_LPE) &&
	    (msgbuf[0] & YUSUR2_VT_MSGTYPE_NACK))
		return YUSUR2_ERR_MBX;

	return 0;
}

/**
 *  yusur2vf_negotiate_api_version - Negotiate supported API version
 *  @hw: pointer to the HW structure
 *  @api: integer containing requested API version
 **/
int yusur2vf_negotiate_api_version(struct yusur2_hw *hw, int api)
{
	int err;
	u32 msg[3];

	/* Negotiate the mailbox API version */
	msg[0] = YUSUR2_VF_API_NEGOTIATE;
	msg[1] = api;
	msg[2] = 0;

	err = yusur2vf_write_msg_read_ack(hw, msg, msg, 3);
	if (!err) {
		msg[0] &= ~YUSUR2_VT_MSGTYPE_CTS;

		/* Store value and return 0 on success */
		if (msg[0] == (YUSUR2_VF_API_NEGOTIATE | YUSUR2_VT_MSGTYPE_ACK)) {
			hw->api_version = api;
			return 0;
		}

		err = YUSUR2_ERR_INVALID_ARGUMENT;
	}

	return err;
}

int yusur2vf_get_queues(struct yusur2_hw *hw, unsigned int *num_tcs,
		       unsigned int *default_tc)
{
	int err;
	u32 msg[5];

	/* do nothing if API doesn't support yusur2vf_get_queues */
	switch (hw->api_version) {
	case yusur2_mbox_api_11:
	case yusur2_mbox_api_12:
	case yusur2_mbox_api_13:
		break;
	default:
		return 0;
	}

	/* Fetch queue configuration from the PF */
	msg[0] = YUSUR2_VF_GET_QUEUES;
	msg[1] = msg[2] = msg[3] = msg[4] = 0;

	err = yusur2vf_write_msg_read_ack(hw, msg, msg, 5);
	if (!err) {
		msg[0] &= ~YUSUR2_VT_MSGTYPE_CTS;

		/*
		 * if we we didn't get an ACK there must have been
		 * some sort of mailbox error so we should treat it
		 * as such
		 */
		if (msg[0] != (YUSUR2_VF_GET_QUEUES | YUSUR2_VT_MSGTYPE_ACK))
			return YUSUR2_ERR_MBX;

		/* record and validate values from message */
		hw->mac.max_tx_queues = msg[YUSUR2_VF_TX_QUEUES];
		if (hw->mac.max_tx_queues == 0 ||
		    hw->mac.max_tx_queues > YUSUR2_VF_MAX_TX_QUEUES)
			hw->mac.max_tx_queues = YUSUR2_VF_MAX_TX_QUEUES;

		hw->mac.max_rx_queues = msg[YUSUR2_VF_RX_QUEUES];
		if (hw->mac.max_rx_queues == 0 ||
		    hw->mac.max_rx_queues > YUSUR2_VF_MAX_RX_QUEUES)
			hw->mac.max_rx_queues = YUSUR2_VF_MAX_RX_QUEUES;

		*num_tcs = msg[YUSUR2_VF_TRANS_VLAN];
		/* in case of unknown state assume we cannot tag frames */
		if (*num_tcs > hw->mac.max_rx_queues)
			*num_tcs = 1;

		*default_tc = msg[YUSUR2_VF_DEF_QUEUE];
		/* default to queue 0 on out-of-bounds queue number */
		if (*default_tc >= hw->mac.max_tx_queues)
			*default_tc = 0;
	}

	return err;
}
