/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_DCB_82599_H_
#define _YUSUR2_DCB_82599_H_

/* DCB register definitions */
#define YUSUR2_RTTDCS_TDPAC	0x00000001 /* 0 Round Robin,
					    * 1 WSP - Weighted Strict Priority
					    */
#define YUSUR2_RTTDCS_VMPAC	0x00000002 /* 0 Round Robin,
					    * 1 WRR - Weighted Round Robin
					    */
#define YUSUR2_RTTDCS_TDRM	0x00000010 /* Transmit Recycle Mode */
#define YUSUR2_RTTDCS_BDPM	0x00400000 /* Bypass Data Pipe - must clear! */
#define YUSUR2_RTTDCS_BPBFSM	0x00800000 /* Bypass PB Free Space - must
					     * clear!
					     */
#define YUSUR2_RTTDCS_SPEED_CHG	0x80000000 /* Link speed change */

/* Receive UP2TC mapping */
#define YUSUR2_RTRUP2TC_UP_SHIFT	3
#define YUSUR2_RTRUP2TC_UP_MASK	7
/* Transmit UP2TC mapping */
#define YUSUR2_RTTUP2TC_UP_SHIFT	3

#define YUSUR2_RTRPT4C_MCL_SHIFT	12 /* Offset to Max Credit Limit setting */
#define YUSUR2_RTRPT4C_BWG_SHIFT	9  /* Offset to BWG index */
#define YUSUR2_RTRPT4C_GSP	0x40000000 /* GSP enable bit */
#define YUSUR2_RTRPT4C_LSP	0x80000000 /* LSP enable bit */

#define YUSUR2_RDRXCTL_MPBEN	0x00000010 /* DMA config for multiple packet
					    * buffers enable
					    */
#define YUSUR2_RDRXCTL_MCEN	0x00000040 /* DMA config for multiple cores
					    * (RSS) enable
					    */

/* RTRPCS Bit Masks */
#define YUSUR2_RTRPCS_RRM	0x00000002 /* Receive Recycle Mode enable */
/* Receive Arbitration Control: 0 Round Robin, 1 DFP */
#define YUSUR2_RTRPCS_RAC	0x00000004
#define YUSUR2_RTRPCS_ARBDIS	0x00000040 /* Arbitration disable bit */

/* RTTDT2C Bit Masks */
#define YUSUR2_RTTDT2C_MCL_SHIFT	12
#define YUSUR2_RTTDT2C_BWG_SHIFT	9
#define YUSUR2_RTTDT2C_GSP	0x40000000
#define YUSUR2_RTTDT2C_LSP	0x80000000

#define YUSUR2_RTTPT2C_MCL_SHIFT	12
#define YUSUR2_RTTPT2C_BWG_SHIFT	9
#define YUSUR2_RTTPT2C_GSP	0x40000000
#define YUSUR2_RTTPT2C_LSP	0x80000000

/* RTTPCS Bit Masks */
#define YUSUR2_RTTPCS_TPPAC	0x00000020 /* 0 Round Robin,
					    * 1 SP - Strict Priority
					    */
#define YUSUR2_RTTPCS_ARBDIS	0x00000040 /* Arbiter disable */
#define YUSUR2_RTTPCS_TPRM	0x00000100 /* Transmit Recycle Mode enable */
#define YUSUR2_RTTPCS_ARBD_SHIFT	22
#define YUSUR2_RTTPCS_ARBD_DCB	0x4 /* Arbitration delay in DCB mode */

#define YUSUR2_TXPBTHRESH_DCB	0xA /* THRESH value for DCB mode */

/* SECTXMINIFG DCB */
#define YUSUR2_SECTX_DCB		0x00001F00 /* DCB TX Buffer SEC IFG */

/* BCN register definitions */
#define YUSUR2_RTTBCNRC_RF_INT_SHIFT	14
#define YUSUR2_RTTBCNRC_RS_ENA		0x80000000

#define YUSUR2_RTTBCNCR_MNG_CMTGI	0x00000001
#define YUSUR2_RTTBCNCR_MGN_BCNA_MODE	0x00000002
#define YUSUR2_RTTBCNCR_RSV7_11_SHIFT	5
#define YUSUR2_RTTBCNCR_G		0x00000400
#define YUSUR2_RTTBCNCR_I		0x00000800
#define YUSUR2_RTTBCNCR_H		0x00001000
#define YUSUR2_RTTBCNCR_VER_SHIFT	14
#define YUSUR2_RTTBCNCR_CMT_ETH_SHIFT	16

#define YUSUR2_RTTBCNACL_SMAC_L_SHIFT	16

#define YUSUR2_RTTBCNTG_BCNA_MODE	0x80000000

#define YUSUR2_RTTBCNRTT_TS_SHIFT	3
#define YUSUR2_RTTBCNRTT_TXQ_IDX_SHIFT	16

#define YUSUR2_RTTBCNRD_BCN_CLEAR_ALL	0x00000002
#define YUSUR2_RTTBCNRD_DRIFT_FAC_SHIFT	2
#define YUSUR2_RTTBCNRD_DRIFT_INT_SHIFT	16
#define YUSUR2_RTTBCNRD_DRIFT_ENA	0x80000000


/* DCB driver APIs */

/* DCB PFC */
s32 yusur2_dcb_config_pfc_82599(struct yusur2_hw *, u8, u8 *);

/* DCB stats */
s32 yusur2_dcb_config_tc_stats_82599(struct yusur2_hw *,
				    struct yusur2_dcb_config *);
s32 yusur2_dcb_get_tc_stats_82599(struct yusur2_hw *,
				 struct yusur2_hw_stats *, u8);
s32 yusur2_dcb_get_pfc_stats_82599(struct yusur2_hw *,
				  struct yusur2_hw_stats *, u8);

/* DCB config arbiters */
s32 yusur2_dcb_config_tx_desc_arbiter_82599(struct yusur2_hw *, u16 *, u16 *,
					   u8 *, u8 *);
s32 yusur2_dcb_config_tx_data_arbiter_82599(struct yusur2_hw *, u16 *, u16 *,
					   u8 *, u8 *, u8 *);
s32 yusur2_dcb_config_rx_arbiter_82599(struct yusur2_hw *, u16 *, u16 *, u8 *,
				      u8 *, u8 *);

/* DCB initialization */
s32 yusur2_dcb_config_82599(struct yusur2_hw *,
			   struct yusur2_dcb_config *);

s32 yusur2_dcb_hw_config_82599(struct yusur2_hw *, int, u16 *, u16 *, u8 *,
			      u8 *, u8 *);
#endif /* _YUSUR2_DCB_82959_H_ */
