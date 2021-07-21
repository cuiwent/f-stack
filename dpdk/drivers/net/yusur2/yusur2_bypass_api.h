/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_BYPASS_API_H_
#define _YUSUR2_BYPASS_API_H_

#ifdef RTE_LIBRTE_YUSUR2_BYPASS

#include "yusur2_bypass_defines.h"
/**
 *  yusur2_bypass_rw_generic - Bit bang data into by_pass FW
 *
 *  @hw: pointer to hardware structure
 *  @cmd: Command we send to the FW
 *  @status: The reply from the FW
 *
 *  Bit-bangs the cmd to the by_pass FW status points to what is returned.
 **/
#define YUSUR2_BYPASS_BB_WAIT 1
static s32 yusur2_bypass_rw_generic(struct yusur2_hw *hw, u32 cmd, u32 *status)
{
	int i;
	u32 sck, sdi, sdo, dir_sck, dir_sdi, dir_sdo;
	u32 esdp;

	if (!status)
		return YUSUR2_ERR_PARAM;

	*status = 0;

	/* SDP vary by MAC type */
	switch (hw->mac.type) {
	case yusur2_mac_82599EB:
		sck = YUSUR2_ESDP_SDP7;
		sdi = YUSUR2_ESDP_SDP0;
		sdo = YUSUR2_ESDP_SDP6;
		dir_sck = YUSUR2_ESDP_SDP7_DIR;
		dir_sdi = YUSUR2_ESDP_SDP0_DIR;
		dir_sdo = YUSUR2_ESDP_SDP6_DIR;
		break;
	case yusur2_mac_X540:
		sck = YUSUR2_ESDP_SDP2;
		sdi = YUSUR2_ESDP_SDP0;
		sdo = YUSUR2_ESDP_SDP1;
		dir_sck = YUSUR2_ESDP_SDP2_DIR;
		dir_sdi = YUSUR2_ESDP_SDP0_DIR;
		dir_sdo = YUSUR2_ESDP_SDP1_DIR;
		break;
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		sck = YUSUR2_ESDP_SDP2;
		sdi = YUSUR2_ESDP_SDP0;
		sdo = YUSUR2_ESDP_SDP1;
		dir_sck = YUSUR2_ESDP_SDP2_DIR;
		dir_sdi = YUSUR2_ESDP_SDP0_DIR;
		dir_sdo = YUSUR2_ESDP_SDP1_DIR;
		break;
	default:
		return YUSUR2_ERR_DEVICE_NOT_SUPPORTED;
	}

	/* Set SDP pins direction */
	esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
	esdp |= dir_sck;	/* SCK as output */
	esdp |= dir_sdi;	/* SDI as output */
	esdp &= ~dir_sdo;	/* SDO as input */
	esdp |= sck;
	esdp |= sdi;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
	YUSUR2_WRITE_FLUSH(hw);
  //  TODO:
	msleep(YUSUR2_BYPASS_BB_WAIT);

	/* Generate start condition */
	esdp &= ~sdi;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
	YUSUR2_WRITE_FLUSH(hw);
	msleep(YUSUR2_BYPASS_BB_WAIT);

	esdp &= ~sck;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
	YUSUR2_WRITE_FLUSH(hw);
	msleep(YUSUR2_BYPASS_BB_WAIT);

	/* Clock out the new control word and clock in the status */
	for (i = 0; i < 32; i++) {
		if ((cmd >> (31 - i)) & 0x01) {
			esdp |= sdi;
			YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		} else {
			esdp &= ~sdi;
			YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		}
		YUSUR2_WRITE_FLUSH(hw);
		msleep(YUSUR2_BYPASS_BB_WAIT);

		esdp |= sck;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);
		msleep(YUSUR2_BYPASS_BB_WAIT);

		esdp &= ~sck;
		YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
		YUSUR2_WRITE_FLUSH(hw);
		msleep(YUSUR2_BYPASS_BB_WAIT);

		esdp = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		if (esdp & sdo)
			*status = (*status << 1) | 0x01;
		else
			*status = (*status << 1) | 0x00;
		msleep(YUSUR2_BYPASS_BB_WAIT);
	}

	/* stop condition */
	esdp |= sck;
	esdp &= ~sdi;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
	YUSUR2_WRITE_FLUSH(hw);
	msleep(YUSUR2_BYPASS_BB_WAIT);

	esdp |= sdi;
	YUSUR2_WRITE_REG(hw, YUSUR2_ESDP, esdp);
	YUSUR2_WRITE_FLUSH(hw);

	/* set the page bits to match the cmd that the status it belongs to */
	*status = (*status & 0x3fffffff) | (cmd & 0xc0000000);

	return 0;
}

/**
 * yusur2_bypass_valid_rd_generic - Verify valid return from bit-bang.
 *
 * If we send a write we can't be sure it took until we can read back
 * that same register.  It can be a problem as some of the feilds may
 * for valid reasons change between the time wrote the register and
 * we read it again to verify.  So this function check everything we
 * can check and then assumes it worked.
 *
 * @u32 in_reg - The register cmd for the bit-bang read.
 * @u32 out_reg - The register returned from a bit-bang read.
 **/
static bool yusur2_bypass_valid_rd_generic(u32 in_reg, u32 out_reg)
{
	u32 mask;

	/* Page must match for all control pages */
	if ((in_reg & BYPASS_PAGE_M) != (out_reg & BYPASS_PAGE_M))
		return false;

	switch (in_reg & BYPASS_PAGE_M) {
	case BYPASS_PAGE_CTL0:
		/* All the following can't change since the last write
		 *  - All the event actions
		 *  - The timeout value
		 */
		mask = BYPASS_AUX_ON_M | BYPASS_MAIN_ON_M |
		       BYPASS_MAIN_OFF_M | BYPASS_AUX_OFF_M |
		       BYPASS_WDTIMEOUT_M |
		       BYPASS_WDT_VALUE_M;
		if ((out_reg & mask) != (in_reg & mask))
			return false;

		/* 0x0 is never a valid value for bypass status */
		if (!(out_reg & BYPASS_STATUS_OFF_M))
			return false;
		break;
	case BYPASS_PAGE_CTL1:
		/* All the following can't change since the last write
		 *  - time valid bit
		 *  - time we last sent
		 */
		mask = BYPASS_CTL1_VALID_M | BYPASS_CTL1_TIME_M;
		if ((out_reg & mask) != (in_reg & mask))
			return false;
		break;
	case BYPASS_PAGE_CTL2:
		/* All we can check in this page is control number
		 * which is already done above.
		 */
		break;
	}

	/* We are as sure as we can be return true */
	return true;
}

/**
 *  yusur2_bypass_set_generic - Set a bypass field in the FW CTRL Regiter.
 *
 *  @hw: pointer to hardware structure
 *  @cmd: The control word we are setting.
 *  @event: The event we are setting in the FW.  This also happens to
 *	    be the mask for the event we are setting (handy)
 *  @action: The action we set the event to in the FW. This is in a
 *	     bit field that happens to be what we want to put in
 *	     the event spot (also handy)
 **/
static s32 yusur2_bypass_set_generic(struct yusur2_hw *hw, u32 ctrl, u32 event,
			     u32 action)
{
	u32 by_ctl = 0;
	u32 cmd, verify;
	u32 count = 0;

	/* Get current values */
	cmd = ctrl;	/* just reading only need control number */
	if (yusur2_bypass_rw_generic(hw, cmd, &by_ctl))
		return YUSUR2_ERR_INVALID_ARGUMENT;

	/* Set to new action */
	cmd = (by_ctl & ~event) | BYPASS_WE | action;
	if (yusur2_bypass_rw_generic(hw, cmd, &by_ctl))
		return YUSUR2_ERR_INVALID_ARGUMENT;

	/* Page 0 force a FW eeprom write which is slow so verify */
	if ((cmd & BYPASS_PAGE_M) == BYPASS_PAGE_CTL0) {
		verify = BYPASS_PAGE_CTL0;
		do {
			if (count++ > 5)
				return YUSUR2_BYPASS_FW_WRITE_FAILURE;

			if (yusur2_bypass_rw_generic(hw, verify, &by_ctl))
				return YUSUR2_ERR_INVALID_ARGUMENT;
		} while (!yusur2_bypass_valid_rd_generic(cmd, by_ctl));
	} else {
		/* We have give the FW time for the write to stick */
		msleep(100);
	}

	return 0;
}

/**
 *  yusur2_bypass_rd_eep_generic - Read the bypass FW eeprom address.
 *
 *  @hw: pointer to hardware structure
 *  @addr: The bypass eeprom address to read.
 *  @value: The 8b of data at the address above.
 **/
static s32 yusur2_bypass_rd_eep_generic(struct yusur2_hw *hw, u32 addr, u8 *value)
{
	u32 cmd;
	u32 status;


	/* send the request */
	cmd = BYPASS_PAGE_CTL2 | BYPASS_WE;
	cmd |= (addr << BYPASS_CTL2_OFFSET_SHIFT) & BYPASS_CTL2_OFFSET_M;
	if (yusur2_bypass_rw_generic(hw, cmd, &status))
		return YUSUR2_ERR_INVALID_ARGUMENT;

	/* We have give the FW time for the write to stick */
	msleep(100);

	/* now read the results */
	cmd &= ~BYPASS_WE;
	if (yusur2_bypass_rw_generic(hw, cmd, &status))
		return YUSUR2_ERR_INVALID_ARGUMENT;

	*value = status & BYPASS_CTL2_DATA_M;

	return 0;
}

#endif /* RTE_LIBRTE_YUSUR2_BYPASS */

#endif /* _YUSUR2_BYPASS_API_H_ */
