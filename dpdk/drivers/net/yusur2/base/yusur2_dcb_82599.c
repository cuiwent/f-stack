/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */


#include "yusur2_type.h"
#include "yusur2_dcb.h"
#include "yusur2_dcb_82599.h"

/**
 * yusur2_dcb_get_tc_stats_82599 - Returns status for each traffic class
 * @hw: pointer to hardware structure
 * @stats: pointer to statistics structure
 * @tc_count:  Number of elements in bwg_array.
 *
 * This function returns the status data for each of the Traffic Classes in use.
 */
s32 yusur2_dcb_get_tc_stats_82599(struct yusur2_hw *hw,
				 struct yusur2_hw_stats *stats,
				 u8 tc_count)
{
	int tc;

	DEBUGFUNC("dcb_get_tc_stats");

	if (tc_count > YUSUR2_DCB_MAX_TRAFFIC_CLASS)
		return YUSUR2_ERR_PARAM;

	/* Statistics pertaining to each traffic class */
	for (tc = 0; tc < tc_count; tc++) {
		/* Transmitted Packets */
		stats->qptc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QPTC(tc));
		/* Transmitted Bytes (read low first to prevent missed carry) */
		stats->qbtc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QBTC_L(tc));
		stats->qbtc[tc] +=
			(((u64)(YUSUR2_READ_REG(hw, YUSUR2_QBTC_H(tc)))) << 32);
		/* Received Packets */
		stats->qprc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QPRC(tc));
		/* Received Bytes (read low first to prevent missed carry) */
		stats->qbrc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QBRC_L(tc));
		stats->qbrc[tc] +=
			(((u64)(YUSUR2_READ_REG(hw, YUSUR2_QBRC_H(tc)))) << 32);

		/* Received Dropped Packet */
		stats->qprdc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QPRDC(tc));
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_get_pfc_stats_82599 - Return CBFC status data
 * @hw: pointer to hardware structure
 * @stats: pointer to statistics structure
 * @tc_count:  Number of elements in bwg_array.
 *
 * This function returns the CBFC status data for each of the Traffic Classes.
 */
s32 yusur2_dcb_get_pfc_stats_82599(struct yusur2_hw *hw,
				  struct yusur2_hw_stats *stats,
				  u8 tc_count)
{
	int tc;

	DEBUGFUNC("dcb_get_pfc_stats");

	if (tc_count > YUSUR2_DCB_MAX_TRAFFIC_CLASS)
		return YUSUR2_ERR_PARAM;

	for (tc = 0; tc < tc_count; tc++) {
		/* Priority XOFF Transmitted */
		stats->pxofftxc[tc] += YUSUR2_READ_REG(hw, YUSUR2_PXOFFTXC(tc));
		/* Priority XOFF Received */
		stats->pxoffrxc[tc] += YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXCNT(tc));
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_rx_arbiter_82599 - Config Rx Data arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 * @map: priority to tc assignments indexed by priority
 *
 * Configure Rx Packet Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_rx_arbiter_82599(struct yusur2_hw *hw, u16 *refill,
				      u16 *max, u8 *bwg_id, u8 *tsa,
				      u8 *map)
{
	u32 reg = 0;
	u32 credit_refill = 0;
	u32 credit_max = 0;
	u8  i = 0;

	/*
	 * Disable the arbiter before changing parameters
	 * (always enable recycle mode; WSP)
	 */
	reg = YUSUR2_RTRPCS_RRM | YUSUR2_RTRPCS_RAC | YUSUR2_RTRPCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTRPCS, reg);

	/*
	 * map all UPs to TCs. up_to_tc_bitmap for each TC has corresponding
	 * bits sets for the UPs that needs to be mappped to that TC.
	 * e.g if priorities 6 and 7 are to be mapped to a TC then the
	 * up_to_tc_bitmap value for that TC will be 11000000 in binary.
	 */
	reg = 0;
	for (i = 0; i < YUSUR2_DCB_MAX_USER_PRIORITY; i++)
		reg |= (map[i] << (i * YUSUR2_RTRUP2TC_UP_SHIFT));

	YUSUR2_WRITE_REG(hw, YUSUR2_RTRUP2TC, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		credit_refill = refill[i];
		credit_max = max[i];
		reg = credit_refill | (credit_max << YUSUR2_RTRPT4C_MCL_SHIFT);

		reg |= (u32)(bwg_id[i]) << YUSUR2_RTRPT4C_BWG_SHIFT;

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_RTRPT4C_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_RTRPT4C(i), reg);
	}

	/*
	 * Configure Rx packet plane (recycle mode; WSP) and
	 * enable arbiter
	 */
	reg = YUSUR2_RTRPCS_RRM | YUSUR2_RTRPCS_RAC;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTRPCS, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tx_desc_arbiter_82599 - Config Tx Desc. arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 *
 * Configure Tx Descriptor Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_tx_desc_arbiter_82599(struct yusur2_hw *hw, u16 *refill,
					   u16 *max, u8 *bwg_id, u8 *tsa)
{
	u32 reg, max_credits;
	u8  i;

	/* Clear the per-Tx queue credits; we use per-TC instead */
	for (i = 0; i < 128; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTDQSEL, i);
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTDT1C, 0);
	}

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		max_credits = max[i];
		reg = max_credits << YUSUR2_RTTDT2C_MCL_SHIFT;
		reg |= refill[i];
		reg |= (u32)(bwg_id[i]) << YUSUR2_RTTDT2C_BWG_SHIFT;

		if (tsa[i] == yusur2_dcb_tsa_group_strict_cee)
			reg |= YUSUR2_RTTDT2C_GSP;

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_RTTDT2C_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_RTTDT2C(i), reg);
	}

	/*
	 * Configure Tx descriptor plane (recycle mode; WSP) and
	 * enable arbiter
	 */
	reg = YUSUR2_RTTDCS_TDPAC | YUSUR2_RTTDCS_TDRM;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDCS, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tx_data_arbiter_82599 - Config Tx Data arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 * @map: priority to tc assignments indexed by priority
 *
 * Configure Tx Packet Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_tx_data_arbiter_82599(struct yusur2_hw *hw, u16 *refill,
					   u16 *max, u8 *bwg_id, u8 *tsa,
					   u8 *map)
{
	u32 reg;
	u8 i;

	/*
	 * Disable the arbiter before changing parameters
	 * (always enable recycle mode; SP; arb delay)
	 */
	reg = YUSUR2_RTTPCS_TPPAC | YUSUR2_RTTPCS_TPRM |
	      (YUSUR2_RTTPCS_ARBD_DCB << YUSUR2_RTTPCS_ARBD_SHIFT) |
	      YUSUR2_RTTPCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTPCS, reg);

	/*
	 * map all UPs to TCs. up_to_tc_bitmap for each TC has corresponding
	 * bits sets for the UPs that needs to be mappped to that TC.
	 * e.g if priorities 6 and 7 are to be mapped to a TC then the
	 * up_to_tc_bitmap value for that TC will be 11000000 in binary.
	 */
	reg = 0;
	for (i = 0; i < YUSUR2_DCB_MAX_USER_PRIORITY; i++)
		reg |= (map[i] << (i * YUSUR2_RTTUP2TC_UP_SHIFT));

	YUSUR2_WRITE_REG(hw, YUSUR2_RTTUP2TC, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		reg = refill[i];
		reg |= (u32)(max[i]) << YUSUR2_RTTPT2C_MCL_SHIFT;
		reg |= (u32)(bwg_id[i]) << YUSUR2_RTTPT2C_BWG_SHIFT;

		if (tsa[i] == yusur2_dcb_tsa_group_strict_cee)
			reg |= YUSUR2_RTTPT2C_GSP;

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_RTTPT2C_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_RTTPT2C(i), reg);
	}

	/*
	 * Configure Tx packet plane (recycle mode; SP; arb delay) and
	 * enable arbiter
	 */
	reg = YUSUR2_RTTPCS_TPPAC | YUSUR2_RTTPCS_TPRM |
	      (YUSUR2_RTTPCS_ARBD_DCB << YUSUR2_RTTPCS_ARBD_SHIFT);
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTPCS, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_pfc_82599 - Configure priority flow control
 * @hw: pointer to hardware structure
 * @pfc_en: enabled pfc bitmask
 * @map: priority to tc assignments indexed by priority
 *
 * Configure Priority Flow Control (PFC) for each traffic class.
 */
s32 yusur2_dcb_config_pfc_82599(struct yusur2_hw *hw, u8 pfc_en, u8 *map)
{
	u32 i, j, fcrtl, reg;
	u8 max_tc = 0;

	/* Enable Transmit Priority Flow Control */
	YUSUR2_WRITE_REG(hw, YUSUR2_FCCFG, YUSUR2_FCCFG_TFCE_PRIORITY);

	/* Enable Receive Priority Flow Control */
	reg = YUSUR2_READ_REG(hw, YUSUR2_MFLCN);
	reg |= YUSUR2_MFLCN_DPF;

	/*
	 * X540 supports per TC Rx priority flow control.  So
	 * clear all TCs and only enable those that should be
	 * enabled.
	 */
	reg &= ~(YUSUR2_MFLCN_RPFCE_MASK | YUSUR2_MFLCN_RFCE);

	if (hw->mac.type >= yusur2_mac_X540)
		reg |= pfc_en << YUSUR2_MFLCN_RPFCE_SHIFT;

	if (pfc_en)
		reg |= YUSUR2_MFLCN_RPFCE;

	YUSUR2_WRITE_REG(hw, YUSUR2_MFLCN, reg);

	for (i = 0; i < YUSUR2_DCB_MAX_USER_PRIORITY; i++) {
		if (map[i] > max_tc)
			max_tc = map[i];
	}


	/* Configure PFC Tx thresholds per TC */
	for (i = 0; i <= max_tc; i++) {
		int enabled = 0;

		for (j = 0; j < YUSUR2_DCB_MAX_USER_PRIORITY; j++) {
			if ((map[j] == i) && (pfc_en & (1 << j))) {
				enabled = 1;
				break;
			}
		}

		if (enabled) {
			reg = (hw->fc.high_water[i] << 10) | YUSUR2_FCRTH_FCEN;
			fcrtl = (hw->fc.low_water[i] << 10) | YUSUR2_FCRTL_XONE;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), fcrtl);
		} else {
			/*
			 * In order to prevent Tx hangs when the internal Tx
			 * switch is enabled we must set the high water mark
			 * to the Rx packet buffer size - 24KB.  This allows
			 * the Tx switch to function even under heavy Rx
			 * workloads.
			 */
			reg = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(i)) - 24576;
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), 0);
		}

		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH_82599(i), reg);
	}

	for (; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH_82599(i), 0);
	}

	/* Configure pause time (2 TCs per register) */
	reg = hw->fc.pause_time | (hw->fc.pause_time << 16);
	for (i = 0; i < (YUSUR2_DCB_MAX_TRAFFIC_CLASS / 2); i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_FCTTV(i), reg);

	/* Configure flow control refresh threshold value */
	YUSUR2_WRITE_REG(hw, YUSUR2_FCRTV, hw->fc.pause_time / 2);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tc_stats_82599 - Config traffic class statistics
 * @hw: pointer to hardware structure
 * @dcb_config: pointer to yusur2_dcb_config structure
 *
 * Configure queue statistics registers, all queues belonging to same traffic
 * class uses a single set of queue statistics counters.
 */
s32 yusur2_dcb_config_tc_stats_82599(struct yusur2_hw *hw,
				    struct yusur2_dcb_config *dcb_config)
{
	u32 reg = 0;
	u8  i   = 0;
	u8 tc_count = 8;
	bool vt_mode = false;

	if (dcb_config != NULL) {
		tc_count = dcb_config->num_tcs.pg_tcs;
		vt_mode = dcb_config->vt_mode;
	}

	if (!((tc_count == 8 && vt_mode == false) || tc_count == 4))
		return YUSUR2_ERR_PARAM;

	if (tc_count == 8 && vt_mode == false) {
		/*
		 * Receive Queues stats setting
		 * 32 RQSMR registers, each configuring 4 queues.
		 *
		 * Set all 16 queues of each TC to the same stat
		 * with TC 'n' going to stat 'n'.
		 */
		for (i = 0; i < 32; i++) {
			reg = 0x01010101 * (i / 4);
			YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), reg);
		}
		/*
		 * Transmit Queues stats setting
		 * 32 TQSM registers, each controlling 4 queues.
		 *
		 * Set all queues of each TC to the same stat
		 * with TC 'n' going to stat 'n'.
		 * Tx queues are allocated non-uniformly to TCs:
		 * 32, 32, 16, 16, 8, 8, 8, 8.
		 */
		for (i = 0; i < 32; i++) {
			if (i < 8)
				reg = 0x00000000;
			else if (i < 16)
				reg = 0x01010101;
			else if (i < 20)
				reg = 0x02020202;
			else if (i < 24)
				reg = 0x03030303;
			else if (i < 26)
				reg = 0x04040404;
			else if (i < 28)
				reg = 0x05050505;
			else if (i < 30)
				reg = 0x06060606;
			else
				reg = 0x07070707;
			YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(i), reg);
		}
	} else if (tc_count == 4 && vt_mode == false) {
		/*
		 * Receive Queues stats setting
		 * 32 RQSMR registers, each configuring 4 queues.
		 *
		 * Set all 16 queues of each TC to the same stat
		 * with TC 'n' going to stat 'n'.
		 */
		for (i = 0; i < 32; i++) {
			if (i % 8 > 3)
				/* In 4 TC mode, odd 16-queue ranges are
				 *  not used.
				*/
				continue;
			reg = 0x01010101 * (i / 8);
			YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), reg);
		}
		/*
		 * Transmit Queues stats setting
		 * 32 TQSM registers, each controlling 4 queues.
		 *
		 * Set all queues of each TC to the same stat
		 * with TC 'n' going to stat 'n'.
		 * Tx queues are allocated non-uniformly to TCs:
		 * 64, 32, 16, 16.
		 */
		for (i = 0; i < 32; i++) {
			if (i < 16)
				reg = 0x00000000;
			else if (i < 24)
				reg = 0x01010101;
			else if (i < 28)
				reg = 0x02020202;
			else
				reg = 0x03030303;
			YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(i), reg);
		}
	} else if (tc_count == 4 && vt_mode == true) {
		/*
		 * Receive Queues stats setting
		 * 32 RQSMR registers, each configuring 4 queues.
		 *
		 * Queue Indexing in 32 VF with DCB mode maps 4 TC's to each
		 * pool. Set all 32 queues of each TC across pools to the same
		 * stat with TC 'n' going to stat 'n'.
		 */
		for (i = 0; i < 32; i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), 0x03020100);
		/*
		 * Transmit Queues stats setting
		 * 32 TQSM registers, each controlling 4 queues.
		 *
		 * Queue Indexing in 32 VF with DCB mode maps 4 TC's to each
		 * pool. Set all 32 queues of each TC across pools to the same
		 * stat with TC 'n' going to stat 'n'.
		 */
		for (i = 0; i < 32; i++)
			YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(i), 0x03020100);
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_82599 - Configure general DCB parameters
 * @hw: pointer to hardware structure
 * @dcb_config: pointer to yusur2_dcb_config structure
 *
 * Configure general DCB parameters.
 */
s32 yusur2_dcb_config_82599(struct yusur2_hw *hw,
			   struct yusur2_dcb_config *dcb_config)
{
	u32 reg;
	u32 q;

	/* Disable the Tx desc arbiter so that MTQC can be changed */
	reg = YUSUR2_READ_REG(hw, YUSUR2_RTTDCS);
	reg |= YUSUR2_RTTDCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDCS, reg);

	reg = YUSUR2_READ_REG(hw, YUSUR2_MRQC);
	if (dcb_config->num_tcs.pg_tcs == 8) {
		/* Enable DCB for Rx with 8 TCs */
		switch (reg & YUSUR2_MRQC_MRQE_MASK) {
		case 0:
		case YUSUR2_MRQC_RT4TCEN:
			/* RSS disabled cases */
			reg = (reg & ~YUSUR2_MRQC_MRQE_MASK) |
			      YUSUR2_MRQC_RT8TCEN;
			break;
		case YUSUR2_MRQC_RSSEN:
		case YUSUR2_MRQC_RTRSS4TCEN:
			/* RSS enabled cases */
			reg = (reg & ~YUSUR2_MRQC_MRQE_MASK) |
			      YUSUR2_MRQC_RTRSS8TCEN;
			break;
		default:
			/*
			 * Unsupported value, assume stale data,
			 * overwrite no RSS
			 */
			ASSERT(0);
			reg = (reg & ~YUSUR2_MRQC_MRQE_MASK) |
			      YUSUR2_MRQC_RT8TCEN;
		}
	}
	if (dcb_config->num_tcs.pg_tcs == 4) {
		/* We support both VT-on and VT-off with 4 TCs. */
		if (dcb_config->vt_mode)
			reg = (reg & ~YUSUR2_MRQC_MRQE_MASK) |
			      YUSUR2_MRQC_VMDQRT4TCEN;
		else
			reg = (reg & ~YUSUR2_MRQC_MRQE_MASK) |
			      YUSUR2_MRQC_RTRSS4TCEN;
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_MRQC, reg);

	/* Enable DCB for Tx with 8 TCs */
	if (dcb_config->num_tcs.pg_tcs == 8)
		reg = YUSUR2_MTQC_RT_ENA | YUSUR2_MTQC_8TC_8TQ;
	else {
		/* We support both VT-on and VT-off with 4 TCs. */
		reg = YUSUR2_MTQC_RT_ENA | YUSUR2_MTQC_4TC_4TQ;
		if (dcb_config->vt_mode)
			reg |= YUSUR2_MTQC_VT_ENA;
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_MTQC, reg);

	/* Disable drop for all queues */
	for (q = 0; q < 128; q++)
		YUSUR2_WRITE_REG(hw, YUSUR2_QDE,
				(YUSUR2_QDE_WRITE | (q << YUSUR2_QDE_IDX_SHIFT)));

	/* Enable the Tx desc arbiter */
	reg = YUSUR2_READ_REG(hw, YUSUR2_RTTDCS);
	reg &= ~YUSUR2_RTTDCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDCS, reg);

	/* Enable Security TX Buffer IFG for DCB */
	reg = YUSUR2_READ_REG(hw, YUSUR2_SECTXMINIFG);
	reg |= YUSUR2_SECTX_DCB;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXMINIFG, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_hw_config_82599 - Configure and enable DCB
 * @hw: pointer to hardware structure
 * @link_speed: unused
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 * @map: priority to tc assignments indexed by priority
 *
 * Configure dcb settings and enable dcb mode.
 */
s32 yusur2_dcb_hw_config_82599(struct yusur2_hw *hw, int link_speed,
			      u16 *refill, u16 *max, u8 *bwg_id, u8 *tsa,
			      u8 *map)
{
	UNREFERENCED_1PARAMETER(link_speed);

	yusur2_dcb_config_rx_arbiter_82599(hw, refill, max, bwg_id, tsa,
					  map);
	yusur2_dcb_config_tx_desc_arbiter_82599(hw, refill, max, bwg_id,
					       tsa);
	yusur2_dcb_config_tx_data_arbiter_82599(hw, refill, max, bwg_id,
					       tsa, map);

	return YUSUR2_SUCCESS;
}

