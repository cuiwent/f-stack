/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_DCB_82598_H_
#define _YUSUR2_DCB_82598_H_

/* DCB register definitions */

#define YUSUR2_DPMCS_MTSOS_SHIFT	16
#define YUSUR2_DPMCS_TDPAC	0x00000001 /* 0 Round Robin,
					    * 1 DFP - Deficit Fixed Priority */
#define YUSUR2_DPMCS_TRM		0x00000010 /* Transmit Recycle Mode */
#define YUSUR2_DPMCS_ARBDIS	0x00000040 /* DCB arbiter disable */
#define YUSUR2_DPMCS_TSOEF	0x00080000 /* TSO Expand Factor: 0=x4, 1=x2 */

#define YUSUR2_RUPPBMR_MQA	0x80000000 /* Enable UP to queue mapping */

#define YUSUR2_RT2CR_MCL_SHIFT	12 /* Offset to Max Credit Limit setting */
#define YUSUR2_RT2CR_LSP		0x80000000 /* LSP enable bit */

#define YUSUR2_RDRXCTL_MPBEN	0x00000010 /* DMA config for multiple packet
					    * buffers enable */
#define YUSUR2_RDRXCTL_MCEN	0x00000040 /* DMA config for multiple cores
					    * (RSS) enable */

#define YUSUR2_TDTQ2TCCR_MCL_SHIFT	12
#define YUSUR2_TDTQ2TCCR_BWG_SHIFT	9
#define YUSUR2_TDTQ2TCCR_GSP	0x40000000
#define YUSUR2_TDTQ2TCCR_LSP	0x80000000

#define YUSUR2_TDPT2TCCR_MCL_SHIFT	12
#define YUSUR2_TDPT2TCCR_BWG_SHIFT	9
#define YUSUR2_TDPT2TCCR_GSP	0x40000000
#define YUSUR2_TDPT2TCCR_LSP	0x80000000

#define YUSUR2_PDPMCS_TPPAC	0x00000020 /* 0 Round Robin,
					    * 1 DFP - Deficit Fixed Priority */
#define YUSUR2_PDPMCS_ARBDIS	0x00000040 /* Arbiter disable */
#define YUSUR2_PDPMCS_TRM	0x00000100 /* Transmit Recycle Mode enable */

#define YUSUR2_DTXCTL_ENDBUBD	0x00000004 /* Enable DBU buffer division */

#define YUSUR2_TXPBSIZE_40KB	0x0000A000 /* 40KB Packet Buffer */
#define YUSUR2_RXPBSIZE_48KB	0x0000C000 /* 48KB Packet Buffer */
#define YUSUR2_RXPBSIZE_64KB	0x00010000 /* 64KB Packet Buffer */
#define YUSUR2_RXPBSIZE_80KB	0x00014000 /* 80KB Packet Buffer */

/* DCB driver APIs */

/* DCB PFC */
s32 yusur2_dcb_config_pfc_82598(struct yusur2_hw *, u8);

/* DCB stats */
s32 yusur2_dcb_config_tc_stats_82598(struct yusur2_hw *);
s32 yusur2_dcb_get_tc_stats_82598(struct yusur2_hw *,
				 struct yusur2_hw_stats *, u8);
s32 yusur2_dcb_get_pfc_stats_82598(struct yusur2_hw *,
				  struct yusur2_hw_stats *, u8);

/* DCB config arbiters */
s32 yusur2_dcb_config_tx_desc_arbiter_82598(struct yusur2_hw *, u16 *, u16 *,
					   u8 *, u8 *);
s32 yusur2_dcb_config_tx_data_arbiter_82598(struct yusur2_hw *, u16 *, u16 *,
					   u8 *, u8 *);
s32 yusur2_dcb_config_rx_arbiter_82598(struct yusur2_hw *, u16 *, u16 *, u8 *);

/* DCB initialization */
s32 yusur2_dcb_hw_config_82598(struct yusur2_hw *, int, u16 *, u16 *, u8 *, u8 *);
#endif /* _YUSUR2_DCB_82958_H_ */
