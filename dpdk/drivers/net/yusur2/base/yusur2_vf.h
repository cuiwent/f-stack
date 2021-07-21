/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_VF_H_
#define _YUSUR2_VF_H_

#include "yusur2_type.h"

#define YUSUR2_VF_IRQ_CLEAR_MASK	7
#define YUSUR2_VF_MAX_TX_QUEUES	8
#define YUSUR2_VF_MAX_RX_QUEUES	8

/* DCB define */
#define YUSUR2_VF_MAX_TRAFFIC_CLASS	8

#define YUSUR2_VFCTRL		0x00000
#define YUSUR2_VFSTATUS		0x00008
#define YUSUR2_VFLINKS		0x00010
#define YUSUR2_VFFRTIMER		0x00048
#define YUSUR2_VFRXMEMWRAP	0x03190
#define YUSUR2_VTEICR		0x00100
#define YUSUR2_VTEICS		0x00104
#define YUSUR2_VTEIMS		0x00108
#define YUSUR2_VTEIMC		0x0010C
#define YUSUR2_VTEIAC		0x00110
#define YUSUR2_VTEIAM		0x00114
#define YUSUR2_VTEITR(x)		(0x00820 + (4 * (x)))
#define YUSUR2_VTIVAR(x)		(0x00120 + (4 * (x)))
#define YUSUR2_VTIVAR_MISC	0x00140
#define YUSUR2_VTRSCINT(x)	(0x00180 + (4 * (x)))
/* define YUSUR2_VFPBACL  still says TBD in EAS */
#define YUSUR2_VFRDBAL(x)	(0x01000 + (0x40 * (x)))
#define YUSUR2_VFRDBAH(x)	(0x01004 + (0x40 * (x)))
#define YUSUR2_VFRDLEN(x)	(0x01008 + (0x40 * (x)))
#define YUSUR2_VFRDH(x)		(0x01010 + (0x40 * (x)))
#define YUSUR2_VFRDT(x)		(0x01018 + (0x40 * (x)))
#define YUSUR2_VFRXDCTL(x)	(0x01028 + (0x40 * (x)))
#define YUSUR2_VFSRRCTL(x)	(0x01014 + (0x40 * (x)))
#define YUSUR2_VFRSCCTL(x)	(0x0102C + (0x40 * (x)))
#define YUSUR2_VFPSRTYPE		0x00300
#define YUSUR2_VFTDBAL(x)	(0x02000 + (0x40 * (x)))
#define YUSUR2_VFTDBAH(x)	(0x02004 + (0x40 * (x)))
#define YUSUR2_VFTDLEN(x)	(0x02008 + (0x40 * (x)))
#define YUSUR2_VFTDH(x)		(0x02010 + (0x40 * (x)))
#define YUSUR2_VFTDT(x)		(0x02018 + (0x40 * (x)))
#define YUSUR2_VFTXDCTL(x)	(0x02028 + (0x40 * (x)))
#define YUSUR2_VFTDWBAL(x)	(0x02038 + (0x40 * (x)))
#define YUSUR2_VFTDWBAH(x)	(0x0203C + (0x40 * (x)))
#define YUSUR2_VFDCA_RXCTRL(x)	(0x0100C + (0x40 * (x)))
#define YUSUR2_VFDCA_TXCTRL(x)	(0x0200c + (0x40 * (x)))
#define YUSUR2_VFGPRC		0x0101C
#define YUSUR2_VFGPTC		0x0201C
#define YUSUR2_VFGORC_LSB	0x01020
#define YUSUR2_VFGORC_MSB	0x01024
#define YUSUR2_VFGOTC_LSB	0x02020
#define YUSUR2_VFGOTC_MSB	0x02024
#define YUSUR2_VFMPRC		0x01034
#define YUSUR2_VFMRQC		0x3000
#define YUSUR2_VFRSSRK(x)	(0x3100 + ((x) * 4))
#define YUSUR2_VFRETA(x)	(0x3200 + ((x) * 4))


struct yusur2vf_hw_stats {
	u64 base_vfgprc;
	u64 base_vfgptc;
	u64 base_vfgorc;
	u64 base_vfgotc;
	u64 base_vfmprc;

	u64 last_vfgprc;
	u64 last_vfgptc;
	u64 last_vfgorc;
	u64 last_vfgotc;
	u64 last_vfmprc;

	u64 vfgprc;
	u64 vfgptc;
	u64 vfgorc;
	u64 vfgotc;
	u64 vfmprc;

	u64 saved_reset_vfgprc;
	u64 saved_reset_vfgptc;
	u64 saved_reset_vfgorc;
	u64 saved_reset_vfgotc;
	u64 saved_reset_vfmprc;
};

s32 yusur2_init_ops_vf(struct yusur2_hw *hw);
s32 yusur2_init_hw_vf(struct yusur2_hw *hw);
s32 yusur2_start_hw_vf(struct yusur2_hw *hw);
s32 yusur2_reset_hw_vf(struct yusur2_hw *hw);
s32 yusur2_stop_adapter_vf(struct yusur2_hw *hw);
u32 yusur2_get_num_of_tx_queues_vf(struct yusur2_hw *hw);
u32 yusur2_get_num_of_rx_queues_vf(struct yusur2_hw *hw);
s32 yusur2_get_mac_addr_vf(struct yusur2_hw *hw, u8 *mac_addr);
s32 yusur2_setup_mac_link_vf(struct yusur2_hw *hw, yusur2_link_speed speed,
			    bool autoneg_wait_to_complete);
s32 yusur2_check_mac_link_vf(struct yusur2_hw *hw, yusur2_link_speed *speed,
			    bool *link_up, bool autoneg_wait_to_complete);
s32 yusur2_set_rar_vf(struct yusur2_hw *hw, u32 index, u8 *addr, u32 vmdq,
		     u32 enable_addr);
s32 yusur2vf_set_uc_addr_vf(struct yusur2_hw *hw, u32 index, u8 *addr);
s32 yusur2_update_mc_addr_list_vf(struct yusur2_hw *hw, u8 *mc_addr_list,
				 u32 mc_addr_count, yusur2_mc_addr_itr,
				 bool clear);
s32 yusur2vf_update_xcast_mode(struct yusur2_hw *hw, int xcast_mode);
s32 yusur2_set_vfta_vf(struct yusur2_hw *hw, u32 vlan, u32 vind,
		      bool vlan_on, bool vlvf_bypass);
s32 yusur2vf_rlpml_set_vf(struct yusur2_hw *hw, u16 max_size);
int yusur2vf_negotiate_api_version(struct yusur2_hw *hw, int api);
int yusur2vf_get_queues(struct yusur2_hw *hw, unsigned int *num_tcs,
		       unsigned int *default_tc);

#endif /* __YUSUR2_VF_H__ */
