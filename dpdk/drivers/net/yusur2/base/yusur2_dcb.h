/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_DCB_H_
#define _YUSUR2_DCB_H_

#include "yusur2_type.h"

/* DCB defines */
/* DCB credit calculation defines */
#define YUSUR2_DCB_CREDIT_QUANTUM	64
#define YUSUR2_DCB_MAX_CREDIT_REFILL	200   /* 200 * 64B = 12800B */
#define YUSUR2_DCB_MAX_TSO_SIZE		(32 * 1024) /* Max TSO pkt size in DCB*/
#define YUSUR2_DCB_MAX_CREDIT		(2 * YUSUR2_DCB_MAX_CREDIT_REFILL)

/* 513 for 32KB TSO packet */
#define YUSUR2_DCB_MIN_TSO_CREDIT	\
	((YUSUR2_DCB_MAX_TSO_SIZE / YUSUR2_DCB_CREDIT_QUANTUM) + 1)

/* DCB configuration defines */
#define YUSUR2_DCB_MAX_USER_PRIORITY	8
#define YUSUR2_DCB_MAX_BW_GROUP		8
#define YUSUR2_DCB_BW_PERCENT		100

#define YUSUR2_DCB_TX_CONFIG		0
#define YUSUR2_DCB_RX_CONFIG		1

/* DCB capability defines */
#define YUSUR2_DCB_PG_SUPPORT	0x00000001
#define YUSUR2_DCB_PFC_SUPPORT	0x00000002
#define YUSUR2_DCB_BCN_SUPPORT	0x00000004
#define YUSUR2_DCB_UP2TC_SUPPORT	0x00000008
#define YUSUR2_DCB_GSP_SUPPORT	0x00000010

struct yusur2_dcb_support {
	u32 capabilities; /* DCB capabilities */

	/* Each bit represents a number of TCs configurable in the hw.
	 * If 8 traffic classes can be configured, the value is 0x80. */
	u8 traffic_classes;
	u8 pfc_traffic_classes;
};

enum yusur2_dcb_tsa {
	yusur2_dcb_tsa_ets = 0,
	yusur2_dcb_tsa_group_strict_cee,
	yusur2_dcb_tsa_strict
};

/* Traffic class bandwidth allocation per direction */
struct yusur2_dcb_tc_path {
	u8 bwg_id; /* Bandwidth Group (BWG) ID */
	u8 bwg_percent; /* % of BWG's bandwidth */
	u8 link_percent; /* % of link bandwidth */
	u8 up_to_tc_bitmap; /* User Priority to Traffic Class mapping */
	u16 data_credits_refill; /* Credit refill amount in 64B granularity */
	u16 data_credits_max; /* Max credits for a configured packet buffer
			       * in 64B granularity.*/
	enum yusur2_dcb_tsa tsa; /* Link or Group Strict Priority */
};

enum yusur2_dcb_pfc {
	yusur2_dcb_pfc_disabled = 0,
	yusur2_dcb_pfc_enabled,
	yusur2_dcb_pfc_enabled_txonly,
	yusur2_dcb_pfc_enabled_rxonly
};

/* Traffic class configuration */
struct yusur2_dcb_tc_config {
	struct yusur2_dcb_tc_path path[2]; /* One each for Tx/Rx */
	enum yusur2_dcb_pfc pfc; /* Class based flow control setting */

	u16 desc_credits_max; /* For Tx Descriptor arbitration */
	u8 tc; /* Traffic class (TC) */
};

enum yusur2_dcb_pba {
	/* PBA[0-7] each use 64KB FIFO */
	yusur2_dcb_pba_equal = PBA_STRATEGY_EQUAL,
	/* PBA[0-3] each use 80KB, PBA[4-7] each use 48KB */
	yusur2_dcb_pba_80_48 = PBA_STRATEGY_WEIGHTED
};

struct yusur2_dcb_num_tcs {
	u8 pg_tcs;
	u8 pfc_tcs;
};

struct yusur2_dcb_config {
	struct yusur2_dcb_tc_config tc_config[YUSUR2_DCB_MAX_TRAFFIC_CLASS];
	struct yusur2_dcb_support support;
	struct yusur2_dcb_num_tcs num_tcs;
	u8 bw_percentage[2][YUSUR2_DCB_MAX_BW_GROUP]; /* One each for Tx/Rx */
	bool pfc_mode_enable;
	bool round_robin_enable;

	enum yusur2_dcb_pba rx_pba_cfg;

	u32 dcb_cfg_version; /* Not used...OS-specific? */
	u32 link_speed; /* For bandwidth allocation validation purpose */
	bool vt_mode;
};

/* DCB driver APIs */

/* DCB rule checking */
s32 yusur2_dcb_check_config_cee(struct yusur2_dcb_config *);

/* DCB credits calculation */
s32 yusur2_dcb_calculate_tc_credits(u8 *, u16 *, u16 *, int);
s32 yusur2_dcb_calculate_tc_credits_cee(struct yusur2_hw *,
				       struct yusur2_dcb_config *, u32, u8);

/* DCB PFC */
s32 yusur2_dcb_config_pfc(struct yusur2_hw *, u8, u8 *);
s32 yusur2_dcb_config_pfc_cee(struct yusur2_hw *, struct yusur2_dcb_config *);

/* DCB stats */
s32 yusur2_dcb_config_tc_stats(struct yusur2_hw *);
s32 yusur2_dcb_get_tc_stats(struct yusur2_hw *, struct yusur2_hw_stats *, u8);
s32 yusur2_dcb_get_pfc_stats(struct yusur2_hw *, struct yusur2_hw_stats *, u8);

/* DCB config arbiters */
s32 yusur2_dcb_config_tx_desc_arbiter_cee(struct yusur2_hw *,
					 struct yusur2_dcb_config *);
s32 yusur2_dcb_config_tx_data_arbiter_cee(struct yusur2_hw *,
					 struct yusur2_dcb_config *);
s32 yusur2_dcb_config_rx_arbiter_cee(struct yusur2_hw *,
				    struct yusur2_dcb_config *);

/* DCB unpack routines */
void yusur2_dcb_unpack_pfc_cee(struct yusur2_dcb_config *, u8 *, u8 *);
void yusur2_dcb_unpack_refill_cee(struct yusur2_dcb_config *, int, u16 *);
void yusur2_dcb_unpack_max_cee(struct yusur2_dcb_config *, u16 *);
void yusur2_dcb_unpack_bwgid_cee(struct yusur2_dcb_config *, int, u8 *);
void yusur2_dcb_unpack_tsa_cee(struct yusur2_dcb_config *, int, u8 *);
void yusur2_dcb_unpack_map_cee(struct yusur2_dcb_config *, int, u8 *);
u8 yusur2_dcb_get_tc_from_up(struct yusur2_dcb_config *, int, u8);

/* DCB initialization */
s32 yusur2_dcb_hw_config(struct yusur2_hw *, u16 *, u16 *, u8 *, u8 *, u8 *);
s32 yusur2_dcb_hw_config_cee(struct yusur2_hw *, struct yusur2_dcb_config *);
#endif /* _YUSUR2_DCB_H_ */
