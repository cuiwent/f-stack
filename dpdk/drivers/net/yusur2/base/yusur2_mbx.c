/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include "yusur2_type.h"
#include "yusur2_mbx.h"

/**
 *  yusur2_read_mbx - Reads a message from the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to read
 *
 *  returns SUCCESS if it successfully read message from buffer
 **/
s32 yusur2_read_mbx(struct yusur2_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_read_mbx");

	/* limit read to size of mailbox */
	if (size > mbx->size)
		size = mbx->size;

	if (mbx->ops.read)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);

	return ret_val;
}

/**
 *  yusur2_write_mbx - Write a message to the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
s32 yusur2_write_mbx(struct yusur2_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_SUCCESS;

	DEBUGFUNC("yusur2_write_mbx");

	if (size > mbx->size) {
		ret_val = YUSUR2_ERR_MBX;
		ERROR_REPORT2(YUSUR2_ERROR_ARGUMENT,
			     "Invalid mailbox message size %d", size);
	} else if (mbx->ops.write)
		ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	return ret_val;
}

/**
 *  yusur2_check_for_msg - checks to see if someone sent us mail
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 yusur2_check_for_msg(struct yusur2_hw *hw, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_check_for_msg");

	if (mbx->ops.check_for_msg)
		ret_val = mbx->ops.check_for_msg(hw, mbx_id);

	return ret_val;
}

/**
 *  yusur2_check_for_ack - checks to see if someone sent us ACK
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 yusur2_check_for_ack(struct yusur2_hw *hw, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_check_for_ack");

	if (mbx->ops.check_for_ack)
		ret_val = mbx->ops.check_for_ack(hw, mbx_id);

	return ret_val;
}

/**
 *  yusur2_check_for_rst - checks to see if other side has reset
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 yusur2_check_for_rst(struct yusur2_hw *hw, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_check_for_rst");

	if (mbx->ops.check_for_rst)
		ret_val = mbx->ops.check_for_rst(hw, mbx_id);

	return ret_val;
}

/**
 *  yusur2_poll_for_msg - Wait for message notification
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message notification
 **/
STATIC s32 yusur2_poll_for_msg(struct yusur2_hw *hw, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	DEBUGFUNC("yusur2_poll_for_msg");

	if (!countdown || !mbx->ops.check_for_msg)
		goto out;

	while (countdown && mbx->ops.check_for_msg(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		usec_delay(mbx->usec_delay);
	}

	if (countdown == 0)
		ERROR_REPORT2(YUSUR2_ERROR_POLLING,
			   "Polling for VF%d mailbox message timedout", mbx_id);

out:
	return countdown ? YUSUR2_SUCCESS : YUSUR2_ERR_MBX;
}

/**
 *  yusur2_poll_for_ack - Wait for message acknowledgement
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message acknowledgement
 **/
STATIC s32 yusur2_poll_for_ack(struct yusur2_hw *hw, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	DEBUGFUNC("yusur2_poll_for_ack");

	if (!countdown || !mbx->ops.check_for_ack)
		goto out;

	while (countdown && mbx->ops.check_for_ack(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		usec_delay(mbx->usec_delay);
	}

	if (countdown == 0)
		ERROR_REPORT2(YUSUR2_ERROR_POLLING,
			     "Polling for VF%d mailbox ack timedout", mbx_id);

out:
	return countdown ? YUSUR2_SUCCESS : YUSUR2_ERR_MBX;
}

/**
 *  yusur2_read_posted_mbx - Wait for message notification and receive message
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message notification and
 *  copied it into the receive buffer.
 **/
s32 yusur2_read_posted_mbx(struct yusur2_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_read_posted_mbx");

	if (!mbx->ops.read)
		goto out;

	ret_val = yusur2_poll_for_msg(hw, mbx_id);

	/* if ack received read message, otherwise we timed out */
	if (!ret_val)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);
out:
	return ret_val;
}

/**
 *  yusur2_write_posted_mbx - Write a message to the mailbox, wait for ack
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer and
 *  received an ack to that message within delay * timeout period
 **/
s32 yusur2_write_posted_mbx(struct yusur2_hw *hw, u32 *msg, u16 size,
			   u16 mbx_id)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_write_posted_mbx");

	/* exit if either we can't write or there isn't a defined timeout */
	if (!mbx->ops.write || !mbx->timeout)
		goto out;

	/* send msg */
	ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	/* if msg sent wait until we receive an ack */
	if (!ret_val)
		ret_val = yusur2_poll_for_ack(hw, mbx_id);
out:
	return ret_val;
}

/**
 *  yusur2_init_mbx_ops_generic - Initialize MB function pointers
 *  @hw: pointer to the HW structure
 *
 *  Setups up the mailbox read and write message function pointers
 **/
void yusur2_init_mbx_ops_generic(struct yusur2_hw *hw)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;

	mbx->ops.read_posted = yusur2_read_posted_mbx;
	mbx->ops.write_posted = yusur2_write_posted_mbx;
}

/**
 *  yusur2_read_v2p_mailbox - read v2p mailbox
 *  @hw: pointer to the HW structure
 *
 *  This function is used to read the v2p mailbox without losing the read to
 *  clear status bits.
 **/
STATIC u32 yusur2_read_v2p_mailbox(struct yusur2_hw *hw)
{
	u32 v2p_mailbox = YUSUR2_READ_REG(hw, YUSUR2_VFMAILBOX);

	v2p_mailbox |= hw->mbx.v2p_mailbox;
	hw->mbx.v2p_mailbox |= v2p_mailbox & YUSUR2_VFMAILBOX_R2C_BITS;

	return v2p_mailbox;
}

/**
 *  yusur2_check_for_bit_vf - Determine if a status bit was set
 *  @hw: pointer to the HW structure
 *  @mask: bitmask for bits to be tested and cleared
 *
 *  This function is used to check for the read to clear bits within
 *  the V2P mailbox.
 **/
STATIC s32 yusur2_check_for_bit_vf(struct yusur2_hw *hw, u32 mask)
{
	u32 v2p_mailbox = yusur2_read_v2p_mailbox(hw);
	s32 ret_val = YUSUR2_ERR_MBX;

	if (v2p_mailbox & mask)
		ret_val = YUSUR2_SUCCESS;

	hw->mbx.v2p_mailbox &= ~mask;

	return ret_val;
}

/**
 *  yusur2_check_for_msg_vf - checks to see if the PF has sent mail
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the PF has set the Status bit or else ERR_MBX
 **/
STATIC s32 yusur2_check_for_msg_vf(struct yusur2_hw *hw, u16 mbx_id)
{
	s32 ret_val = YUSUR2_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("yusur2_check_for_msg_vf");

	if (!yusur2_check_for_bit_vf(hw, YUSUR2_VFMAILBOX_PFSTS)) {
		ret_val = YUSUR2_SUCCESS;
		hw->mbx.stats.reqs++;
	}

	return ret_val;
}

/**
 *  yusur2_check_for_ack_vf - checks to see if the PF has ACK'd
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the PF has set the ACK bit or else ERR_MBX
 **/
STATIC s32 yusur2_check_for_ack_vf(struct yusur2_hw *hw, u16 mbx_id)
{
	s32 ret_val = YUSUR2_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("yusur2_check_for_ack_vf");

	if (!yusur2_check_for_bit_vf(hw, YUSUR2_VFMAILBOX_PFACK)) {
		ret_val = YUSUR2_SUCCESS;
		hw->mbx.stats.acks++;
	}

	return ret_val;
}

/**
 *  yusur2_check_for_rst_vf - checks to see if the PF has reset
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns true if the PF has set the reset done bit or else false
 **/
STATIC s32 yusur2_check_for_rst_vf(struct yusur2_hw *hw, u16 mbx_id)
{
	s32 ret_val = YUSUR2_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("yusur2_check_for_rst_vf");

	if (!yusur2_check_for_bit_vf(hw, (YUSUR2_VFMAILBOX_RSTD |
	    YUSUR2_VFMAILBOX_RSTI))) {
		ret_val = YUSUR2_SUCCESS;
		hw->mbx.stats.rsts++;
	}

	return ret_val;
}

/**
 *  yusur2_obtain_mbx_lock_vf - obtain mailbox lock
 *  @hw: pointer to the HW structure
 *
 *  return SUCCESS if we obtained the mailbox lock
 **/
STATIC s32 yusur2_obtain_mbx_lock_vf(struct yusur2_hw *hw)
{
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_obtain_mbx_lock_vf");

	/* Take ownership of the buffer */
	YUSUR2_WRITE_REG(hw, YUSUR2_VFMAILBOX, YUSUR2_VFMAILBOX_VFU);

	/* reserve mailbox for vf use */
	if (yusur2_read_v2p_mailbox(hw) & YUSUR2_VFMAILBOX_VFU)
		ret_val = YUSUR2_SUCCESS;

	return ret_val;
}

/**
 *  yusur2_write_mbx_vf - Write a message to the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
STATIC s32 yusur2_write_mbx_vf(struct yusur2_hw *hw, u32 *msg, u16 size,
			      u16 mbx_id)
{
	s32 ret_val;
	u16 i;

	UNREFERENCED_1PARAMETER(mbx_id);

	DEBUGFUNC("yusur2_write_mbx_vf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = yusur2_obtain_mbx_lock_vf(hw);
	if (ret_val)
		goto out_no_write;

	/* flush msg and acks as we are overwriting the message buffer */
	yusur2_check_for_msg_vf(hw, 0);
	yusur2_check_for_ack_vf(hw, 0);

	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		YUSUR2_WRITE_REG_ARRAY(hw, YUSUR2_VFMBMEM, i, msg[i]);

	/* update stats */
	hw->mbx.stats.msgs_tx++;

	/* Drop VFU and interrupt the PF to tell it a message has been sent */
	YUSUR2_WRITE_REG(hw, YUSUR2_VFMAILBOX, YUSUR2_VFMAILBOX_REQ);

out_no_write:
	return ret_val;
}

/**
 *  yusur2_read_mbx_vf - Reads a message from the inbox intended for vf
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to read
 *
 *  returns SUCCESS if it successfully read message from buffer
 **/
STATIC s32 yusur2_read_mbx_vf(struct yusur2_hw *hw, u32 *msg, u16 size,
			     u16 mbx_id)
{
	s32 ret_val = YUSUR2_SUCCESS;
	u16 i;

	DEBUGFUNC("yusur2_read_mbx_vf");
	UNREFERENCED_1PARAMETER(mbx_id);

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = yusur2_obtain_mbx_lock_vf(hw);
	if (ret_val)
		goto out_no_read;

	/* copy the message from the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = YUSUR2_READ_REG_ARRAY(hw, YUSUR2_VFMBMEM, i);

	/* Acknowledge receipt and release mailbox, then we're done */
	YUSUR2_WRITE_REG(hw, YUSUR2_VFMAILBOX, YUSUR2_VFMAILBOX_ACK);

	/* update stats */
	hw->mbx.stats.msgs_rx++;

out_no_read:
	return ret_val;
}

/**
 *  yusur2_init_mbx_params_vf - set initial values for vf mailbox
 *  @hw: pointer to the HW structure
 *
 *  Initializes the hw->mbx struct to correct values for vf mailbox
 */
void yusur2_init_mbx_params_vf(struct yusur2_hw *hw)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;

	/* start mailbox as timed out and let the reset_hw call set the timeout
	 * value to begin communications */
	mbx->timeout = 0;
	mbx->usec_delay = YUSUR2_VF_MBX_INIT_DELAY;

	mbx->size = YUSUR2_VFMAILBOX_SIZE;

	mbx->ops.read = yusur2_read_mbx_vf;
	mbx->ops.write = yusur2_write_mbx_vf;
	mbx->ops.read_posted = yusur2_read_posted_mbx;
	mbx->ops.write_posted = yusur2_write_posted_mbx;
	mbx->ops.check_for_msg = yusur2_check_for_msg_vf;
	mbx->ops.check_for_ack = yusur2_check_for_ack_vf;
	mbx->ops.check_for_rst = yusur2_check_for_rst_vf;

	mbx->stats.msgs_tx = 0;
	mbx->stats.msgs_rx = 0;
	mbx->stats.reqs = 0;
	mbx->stats.acks = 0;
	mbx->stats.rsts = 0;
}

STATIC s32 yusur2_check_for_bit_pf(struct yusur2_hw *hw, u32 mask, s32 index)
{
	u32 mbvficr = YUSUR2_READ_REG(hw, YUSUR2_MBVFICR(index));
	s32 ret_val = YUSUR2_ERR_MBX;

	if (mbvficr & mask) {
		ret_val = YUSUR2_SUCCESS;
		YUSUR2_WRITE_REG(hw, YUSUR2_MBVFICR(index), mask);
	}

	return ret_val;
}

/**
 *  yusur2_check_for_msg_pf - checks to see if the VF has sent mail
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 yusur2_check_for_msg_pf(struct yusur2_hw *hw, u16 vf_number)
{
	s32 ret_val = YUSUR2_ERR_MBX;
	s32 index = YUSUR2_MBVFICR_INDEX(vf_number);
	u32 vf_bit = vf_number % 16;

	DEBUGFUNC("yusur2_check_for_msg_pf");

	if (!yusur2_check_for_bit_pf(hw, YUSUR2_MBVFICR_VFREQ_VF1 << vf_bit,
				    index)) {
		ret_val = YUSUR2_SUCCESS;
		hw->mbx.stats.reqs++;
	}

	return ret_val;
}

/**
 *  yusur2_check_for_ack_pf - checks to see if the VF has ACKed
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 yusur2_check_for_ack_pf(struct yusur2_hw *hw, u16 vf_number)
{
	s32 ret_val = YUSUR2_ERR_MBX;
	s32 index = YUSUR2_MBVFICR_INDEX(vf_number);
	u32 vf_bit = vf_number % 16;

	DEBUGFUNC("yusur2_check_for_ack_pf");

	if (!yusur2_check_for_bit_pf(hw, YUSUR2_MBVFICR_VFACK_VF1 << vf_bit,
				    index)) {
		ret_val = YUSUR2_SUCCESS;
		hw->mbx.stats.acks++;
	}

	return ret_val;
}

/**
 *  yusur2_check_for_rst_pf - checks to see if the VF has reset
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 yusur2_check_for_rst_pf(struct yusur2_hw *hw, u16 vf_number)
{
	u32 reg_offset = (vf_number < 32) ? 0 : 1;
	u32 vf_shift = vf_number % 32;
	u32 vflre = 0;
	s32 ret_val = YUSUR2_ERR_MBX;

	DEBUGFUNC("yusur2_check_for_rst_pf");

	switch (hw->mac.type) {
	case yusur2_mac_82599EB:
		vflre = YUSUR2_READ_REG(hw, YUSUR2_VFLRE(reg_offset));
		break;
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
	case yusur2_mac_X540:
		vflre = YUSUR2_READ_REG(hw, YUSUR2_VFLREC(reg_offset));
		break;
	default:
		break;
	}

	if (vflre & (1 << vf_shift)) {
		ret_val = YUSUR2_SUCCESS;
		YUSUR2_WRITE_REG(hw, YUSUR2_VFLREC(reg_offset), (1 << vf_shift));
		hw->mbx.stats.rsts++;
	}

	return ret_val;
}

/**
 *  yusur2_obtain_mbx_lock_pf - obtain mailbox lock
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  return SUCCESS if we obtained the mailbox lock
 **/
STATIC s32 yusur2_obtain_mbx_lock_pf(struct yusur2_hw *hw, u16 vf_number)
{
	s32 ret_val = YUSUR2_ERR_MBX;
	u32 p2v_mailbox;

	DEBUGFUNC("yusur2_obtain_mbx_lock_pf");

	/* Take ownership of the buffer */
	YUSUR2_WRITE_REG(hw, YUSUR2_PFMAILBOX(vf_number), YUSUR2_PFMAILBOX_PFU);

	/* reserve mailbox for vf use */
	p2v_mailbox = YUSUR2_READ_REG(hw, YUSUR2_PFMAILBOX(vf_number));
	if (p2v_mailbox & YUSUR2_PFMAILBOX_PFU)
		ret_val = YUSUR2_SUCCESS;
	else
		ERROR_REPORT2(YUSUR2_ERROR_POLLING,
			   "Failed to obtain mailbox lock for VF%d", vf_number);


	return ret_val;
}

/**
 *  yusur2_write_mbx_pf - Places a message in the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
STATIC s32 yusur2_write_mbx_pf(struct yusur2_hw *hw, u32 *msg, u16 size,
			      u16 vf_number)
{
	s32 ret_val;
	u16 i;

	DEBUGFUNC("yusur2_write_mbx_pf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = yusur2_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_write;

	/* flush msg and acks as we are overwriting the message buffer */
	yusur2_check_for_msg_pf(hw, vf_number);
	yusur2_check_for_ack_pf(hw, vf_number);

	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		YUSUR2_WRITE_REG_ARRAY(hw, YUSUR2_PFMBMEM(vf_number), i, msg[i]);

	/* Interrupt VF to tell it a message has been sent and release buffer*/
	YUSUR2_WRITE_REG(hw, YUSUR2_PFMAILBOX(vf_number), YUSUR2_PFMAILBOX_STS);

	/* update stats */
	hw->mbx.stats.msgs_tx++;

out_no_write:
	return ret_val;

}

/**
 *  yusur2_read_mbx_pf - Read a message from the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @vf_number: the VF index
 *
 *  This function copies a message from the mailbox buffer to the caller's
 *  memory buffer.  The presumption is that the caller knows that there was
 *  a message due to a VF request so no polling for message is needed.
 **/
STATIC s32 yusur2_read_mbx_pf(struct yusur2_hw *hw, u32 *msg, u16 size,
			     u16 vf_number)
{
	s32 ret_val;
	u16 i;

	DEBUGFUNC("yusur2_read_mbx_pf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = yusur2_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_read;

	/* copy the message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = YUSUR2_READ_REG_ARRAY(hw, YUSUR2_PFMBMEM(vf_number), i);

	/* Acknowledge the message and release buffer */
	YUSUR2_WRITE_REG(hw, YUSUR2_PFMAILBOX(vf_number), YUSUR2_PFMAILBOX_ACK);

	/* update stats */
	hw->mbx.stats.msgs_rx++;

out_no_read:
	return ret_val;
}

/**
 *  yusur2_init_mbx_params_pf - set initial values for pf mailbox
 *  @hw: pointer to the HW structure
 *
 *  Initializes the hw->mbx struct to correct values for pf mailbox
 */
void yusur2_init_mbx_params_pf(struct yusur2_hw *hw)
{
	struct yusur2_mbx_info *mbx = &hw->mbx;

	if (hw->mac.type != yusur2_mac_82599EB &&
	    hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a &&
	    hw->mac.type != yusur2_mac_X540)
		return;

	mbx->timeout = 0;
	mbx->usec_delay = 0;

	mbx->size = YUSUR2_VFMAILBOX_SIZE;

	mbx->ops.read = yusur2_read_mbx_pf;
	mbx->ops.write = yusur2_write_mbx_pf;
	mbx->ops.read_posted = yusur2_read_posted_mbx;
	mbx->ops.write_posted = yusur2_write_posted_mbx;
	mbx->ops.check_for_msg = yusur2_check_for_msg_pf;
	mbx->ops.check_for_ack = yusur2_check_for_ack_pf;
	mbx->ops.check_for_rst = yusur2_check_for_rst_pf;

	mbx->stats.msgs_tx = 0;
	mbx->stats.msgs_rx = 0;
	mbx->stats.reqs = 0;
	mbx->stats.acks = 0;
	mbx->stats.rsts = 0;
}
