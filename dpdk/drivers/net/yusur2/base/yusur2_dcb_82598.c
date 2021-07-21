/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */


#include "yusur2_type.h"
#include "yusur2_dcb.h"
#include "yusur2_dcb_82598.h"

/**
 * yusur2_dcb_get_tc_stats_82598 - Return status data for each traffic class
 * @hw: pointer to hardware structure
 * @stats: pointer to statistics structure
 * @tc_count:  Number of elements in bwg_array.
 *
 * This function returns the status data for each of the Traffic Classes in use.
 */
s32 yusur2_dcb_get_tc_stats_82598(struct yusur2_hw *hw,
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
		/* Transmitted Bytes */
		stats->qbtc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QBTC(tc));
		/* Received Packets */
		stats->qprc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QPRC(tc));
		/* Received Bytes */
		stats->qbrc[tc] += YUSUR2_READ_REG(hw, YUSUR2_QBRC(tc));
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_get_pfc_stats_82598 - Returns CBFC status data
 * @hw: pointer to hardware structure
 * @stats: pointer to statistics structure
 * @tc_count:  Number of elements in bwg_array.
 *
 * This function returns the CBFC status data for each of the Traffic Classes.
 */
s32 yusur2_dcb_get_pfc_stats_82598(struct yusur2_hw *hw,
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
		stats->pxoffrxc[tc] += YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXC(tc));
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_rx_arbiter_82598 - Config Rx data arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 *
 * Configure Rx Data Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_rx_arbiter_82598(struct yusur2_hw *hw, u16 *refill,
				      u16 *max, u8 *tsa)
{
	u32 reg = 0;
	u32 credit_refill = 0;
	u32 credit_max = 0;
	u8 i = 0;

	reg = YUSUR2_READ_REG(hw, YUSUR2_RUPPBMR) | YUSUR2_RUPPBMR_MQA;
	YUSUR2_WRITE_REG(hw, YUSUR2_RUPPBMR, reg);

	reg = YUSUR2_READ_REG(hw, YUSUR2_RMCS);
	/* Enable Arbiter */
	reg &= ~YUSUR2_RMCS_ARBDIS;
	/* Enable Receive Recycle within the BWG */
	reg |= YUSUR2_RMCS_RRM;
	/* Enable Deficit Fixed Priority arbitration*/
	reg |= YUSUR2_RMCS_DFP;

	YUSUR2_WRITE_REG(hw, YUSUR2_RMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		credit_refill = refill[i];
		credit_max = max[i];

		reg = credit_refill | (credit_max << YUSUR2_RT2CR_MCL_SHIFT);

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_RT2CR_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_RT2CR(i), reg);
	}

	reg = YUSUR2_READ_REG(hw, YUSUR2_RDRXCTL);
	reg |= YUSUR2_RDRXCTL_RDMTS_1_2;
	reg |= YUSUR2_RDRXCTL_MPBEN;
	reg |= YUSUR2_RDRXCTL_MCEN;
	YUSUR2_WRITE_REG(hw, YUSUR2_RDRXCTL, reg);

	reg = YUSUR2_READ_REG(hw, YUSUR2_RXCTRL);
	/* Make sure there is enough descriptors before arbitration */
	reg &= ~YUSUR2_RXCTRL_DMBYPS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXCTRL, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tx_desc_arbiter_82598 - Config Tx Desc. arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 *
 * Configure Tx Descriptor Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_tx_desc_arbiter_82598(struct yusur2_hw *hw,
					   u16 *refill, u16 *max, u8 *bwg_id,
					   u8 *tsa)
{
	u32 reg, max_credits;
	u8 i;

	reg = YUSUR2_READ_REG(hw, YUSUR2_DPMCS);

	/* Enable arbiter */
	reg &= ~YUSUR2_DPMCS_ARBDIS;
	reg |= YUSUR2_DPMCS_TSOEF;

	/* Configure Max TSO packet size 34KB including payload and headers */
	reg |= (0x4 << YUSUR2_DPMCS_MTSOS_SHIFT);

	YUSUR2_WRITE_REG(hw, YUSUR2_DPMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		max_credits = max[i];
		reg = max_credits << YUSUR2_TDTQ2TCCR_MCL_SHIFT;
		reg |= refill[i];
		reg |= (u32)(bwg_id[i]) << YUSUR2_TDTQ2TCCR_BWG_SHIFT;

		if (tsa[i] == yusur2_dcb_tsa_group_strict_cee)
			reg |= YUSUR2_TDTQ2TCCR_GSP;

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_TDTQ2TCCR_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_TDTQ2TCCR(i), reg);
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tx_data_arbiter_82598 - Config Tx data arbiter
 * @hw: pointer to hardware structure
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 *
 * Configure Tx Data Arbiter and credits for each traffic class.
 */
s32 yusur2_dcb_config_tx_data_arbiter_82598(struct yusur2_hw *hw,
					   u16 *refill, u16 *max, u8 *bwg_id,
					   u8 *tsa)
{
	u32 reg;
	u8 i;

	reg = YUSUR2_READ_REG(hw, YUSUR2_PDPMCS);
	/* Enable Data Plane Arbiter */
	reg &= ~YUSUR2_PDPMCS_ARBDIS;
	/* Enable DFP and Transmit Recycle Mode */
	reg |= (YUSUR2_PDPMCS_TPPAC | YUSUR2_PDPMCS_TRM);

	YUSUR2_WRITE_REG(hw, YUSUR2_PDPMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		reg = refill[i];
		reg |= (u32)(max[i]) << YUSUR2_TDPT2TCCR_MCL_SHIFT;
		reg |= (u32)(bwg_id[i]) << YUSUR2_TDPT2TCCR_BWG_SHIFT;

		if (tsa[i] == yusur2_dcb_tsa_group_strict_cee)
			reg |= YUSUR2_TDPT2TCCR_GSP;

		if (tsa[i] == yusur2_dcb_tsa_strict)
			reg |= YUSUR2_TDPT2TCCR_LSP;

		YUSUR2_WRITE_REG(hw, YUSUR2_TDPT2TCCR(i), reg);
	}

	/* Enable Tx packet buffer division */
	reg = YUSUR2_READ_REG(hw, YUSUR2_DTXCTL);
	reg |= YUSUR2_DTXCTL_ENDBUBD;
	YUSUR2_WRITE_REG(hw, YUSUR2_DTXCTL, reg);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_pfc_82598 - Config priority flow control
 * @hw: pointer to hardware structure
 * @pfc_en: enabled pfc bitmask
 *
 * Configure Priority Flow Control for each traffic class.
 */
s32 yusur2_dcb_config_pfc_82598(struct yusur2_hw *hw, u8 pfc_en)
{
	u32 fcrtl, reg;
	u8 i;

	/* Enable Transmit Priority Flow Control */
	reg = YUSUR2_READ_REG(hw, YUSUR2_RMCS);
	reg &= ~YUSUR2_RMCS_TFCE_802_3X;
	reg |= YUSUR2_RMCS_TFCE_PRIORITY;
	YUSUR2_WRITE_REG(hw, YUSUR2_RMCS, reg);

	/* Enable Receive Priority Flow Control */
	reg = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	reg &= ~(YUSUR2_FCTRL_RPFCE | YUSUR2_FCTRL_RFCE);

	if (pfc_en)
		reg |= YUSUR2_FCTRL_RPFCE;

	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, reg);

	/* Configure PFC Tx thresholds per TC */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		if (!(pfc_en & (1 << i))) {
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL(i), 0);
			YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH(i), 0);
			continue;
		}

		fcrtl = (hw->fc.low_water[i] << 10) | YUSUR2_FCRTL_XONE;
		reg = (hw->fc.high_water[i] << 10) | YUSUR2_FCRTH_FCEN;
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL(i), fcrtl);
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH(i), reg);
	}

	/* Configure pause time */
	reg = hw->fc.pause_time | (hw->fc.pause_time << 16);
	for (i = 0; i < (YUSUR2_DCB_MAX_TRAFFIC_CLASS / 2); i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_FCTTV(i), reg);

	/* Configure flow control refresh threshold value */
	YUSUR2_WRITE_REG(hw, YUSUR2_FCRTV, hw->fc.pause_time / 2);

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_config_tc_stats_82598 - Configure traffic class statistics
 * @hw: pointer to hardware structure
 *
 * Configure queue statistics registers, all queues belonging to same traffic
 * class uses a single set of queue statistics counters.
 */
s32 yusur2_dcb_config_tc_stats_82598(struct yusur2_hw *hw)
{
	u32 reg = 0;
	u8 i = 0;
	u8 j = 0;

	/* Receive Queues stats setting -  8 queues per statistics reg */
	for (i = 0, j = 0; i < 15 && j < 8; i = i + 2, j++) {
		reg = YUSUR2_READ_REG(hw, YUSUR2_RQSMR(i));
		reg |= ((0x1010101) * j);
		YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), reg);
		reg = YUSUR2_READ_REG(hw, YUSUR2_RQSMR(i + 1));
		reg |= ((0x1010101) * j);
		YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i + 1), reg);
	}
	/* Transmit Queues stats setting -  4 queues per statistics reg*/
	for (i = 0; i < 8; i++) {
		reg = YUSUR2_READ_REG(hw, YUSUR2_TQSMR(i));
		reg |= ((0x1010101) * i);
		YUSUR2_WRITE_REG(hw, YUSUR2_TQSMR(i), reg);
	}

	return YUSUR2_SUCCESS;
}

/**
 * yusur2_dcb_hw_config_82598 - Config and enable DCB
 * @hw: pointer to hardware structure
 * @link_speed: unused
 * @refill: refill credits index by traffic class
 * @max: max credits index by traffic class
 * @bwg_id: bandwidth grouping indexed by traffic class
 * @tsa: transmission selection algorithm indexed by traffic class
 *
 * Configure dcb settings and enable dcb mode.
 */
s32 yusur2_dcb_hw_config_82598(struct yusur2_hw *hw, int link_speed,
			      u16 *refill, u16 *max, u8 *bwg_id,
			      u8 *tsa)
{
	UNREFERENCED_1PARAMETER(link_speed);

	yusur2_dcb_config_rx_arbiter_82598(hw, refill, max, tsa);
	yusur2_dcb_config_tx_desc_arbiter_82598(hw, refill, max, bwg_id,
					       tsa);
	yusur2_dcb_config_tx_data_arbiter_82598(hw, refill, max, bwg_id,
					       tsa);
	yusur2_dcb_config_tc_stats_82598(hw);


	return YUSUR2_SUCCESS;
}
