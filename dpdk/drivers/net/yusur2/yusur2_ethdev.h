/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_ETHDEV_H_
#define _YUSUR2_ETHDEV_H_

#include <stdint.h>

#include "base/yusur2_type.h"
#include "base/yusur2_dcb.h"
#include "base/yusur2_dcb_82599.h"
#include "base/yusur2_dcb_82598.h"
#include "yusur2_bypass.h"
#ifdef RTE_LIBRTE_SECURITY
#include "yusur2_ipsec.h"
#endif
#include <rte_flow.h>
#include <rte_time.h>
#include <rte_hash.h>
#include <rte_pci.h>
#include <rte_bus_pci.h>
#include <rte_tm_driver.h>

/* need update link, bit flag */
#define YUSUR2_FLAG_NEED_LINK_UPDATE (uint32_t)(1 << 0)
#define YUSUR2_FLAG_MAILBOX          (uint32_t)(1 << 1)
#define YUSUR2_FLAG_PHY_INTERRUPT    (uint32_t)(1 << 2)
#define YUSUR2_FLAG_MACSEC           (uint32_t)(1 << 3)
#define YUSUR2_FLAG_NEED_LINK_CONFIG (uint32_t)(1 << 4)

/*
 * Defines that were not part of yusur2_type.h as they are not used by the
 * FreeBSD driver.
 */
#define YUSUR2_ADVTXD_MAC_1588       0x00080000 /* IEEE1588 Timestamp packet */
#define YUSUR2_RXD_STAT_TMST         0x10000    /* Timestamped Packet indication */
#define YUSUR2_ADVTXD_TUCMD_L4T_RSV  0x00001800 /* L4 Packet TYPE, resvd  */
#define YUSUR2_RXDADV_ERR_CKSUM_BIT  30
#define YUSUR2_RXDADV_ERR_CKSUM_MSK  3
#define YUSUR2_ADVTXD_MACLEN_SHIFT   9          /* Bit shift for l2_len */
#define YUSUR2_NB_STAT_MAPPING_REGS  32
#define YUSUR2_EXTENDED_VLAN	  (uint32_t)(1 << 26) /* EXTENDED VLAN ENABLE */
#define YUSUR2_VFTA_SIZE 128
#define YUSUR2_VLAN_TAG_SIZE 4
#define YUSUR2_HKEY_MAX_INDEX 10
#define YUSUR2_MAX_RX_QUEUE_NUM	128
#define YUSUR2_MAX_INTR_QUEUE_NUM	15
#define YUSUR2_VMDQ_DCB_NB_QUEUES     YUSUR2_MAX_RX_QUEUE_NUM
#define YUSUR2_DCB_NB_QUEUES          YUSUR2_MAX_RX_QUEUE_NUM
#define YUSUR2_NONE_MODE_TX_NB_QUEUES 64

#ifndef NBBY
#define NBBY	8	/* number of bits in a byte */
#endif
#define YUSUR2_HWSTRIP_BITMAP_SIZE (YUSUR2_MAX_RX_QUEUE_NUM / (sizeof(uint32_t) * NBBY))

/* EITR Interval is in 2048ns uinits for 1G and 10G link */
#define YUSUR2_EITR_INTERVAL_UNIT_NS	2048
#define YUSUR2_EITR_ITR_INT_SHIFT       3
#define YUSUR2_EITR_INTERVAL_US(us) \
	(((us) * 1000 / YUSUR2_EITR_INTERVAL_UNIT_NS << YUSUR2_EITR_ITR_INT_SHIFT) & \
		YUSUR2_EITR_ITR_INT_MASK)

#define YUSUR2_QUEUE_ITR_INTERVAL_DEFAULT	500 /* 500us */

/* Loopback operation modes */
#define YUSUR2_LPBK_NONE   0x0 /* Default value. Loopback is disabled. */
#define YUSUR2_LPBK_TX_RX  0x1 /* Tx->Rx loopback operation is enabled. */
/* X540-X550 specific loopback operations */
#define YUSUR2_MII_AUTONEG_ENABLE        0x1000 /* Auto-negociation enable (default = 1) */

#define YUSUR2_MAX_JUMBO_FRAME_SIZE      0x2600 /* Maximum Jumbo frame size. */

#define YUSUR2_RTTBCNRC_RF_INT_MASK_BASE 0x000003FF
#define YUSUR2_RTTBCNRC_RF_INT_MASK_M \
	(YUSUR2_RTTBCNRC_RF_INT_MASK_BASE << YUSUR2_RTTBCNRC_RF_INT_SHIFT)

#define YUSUR2_MAX_QUEUE_NUM_PER_VF  8

#define YUSUR2_SYN_FILTER_ENABLE         0x00000001 /* syn filter enable field */
#define YUSUR2_SYN_FILTER_QUEUE          0x000000FE /* syn filter queue field */
#define YUSUR2_SYN_FILTER_QUEUE_SHIFT    1          /* syn filter queue field shift */
#define YUSUR2_SYN_FILTER_SYNQFP         0x80000000 /* syn filter SYNQFP */

#define YUSUR2_ETQF_UP                   0x00070000 /* ethertype filter priority field */
#define YUSUR2_ETQF_SHIFT                16
#define YUSUR2_ETQF_UP_EN                0x00080000
#define YUSUR2_ETQF_ETHERTYPE            0x0000FFFF /* ethertype filter ethertype field */
#define YUSUR2_ETQF_MAX_PRI              7

#define YUSUR2_SDPQF_DSTPORT             0xFFFF0000 /* dst port field */
#define YUSUR2_SDPQF_DSTPORT_SHIFT       16         /* dst port field shift */
#define YUSUR2_SDPQF_SRCPORT             0x0000FFFF /* src port field */

#define YUSUR2_L34T_IMIR_SIZE_BP         0x00001000
#define YUSUR2_L34T_IMIR_RESERVE         0x00080000 /* bit 13 to 19 must be set to 1000000b. */
#define YUSUR2_L34T_IMIR_LLI             0x00100000
#define YUSUR2_L34T_IMIR_QUEUE           0x0FE00000
#define YUSUR2_L34T_IMIR_QUEUE_SHIFT     21
#define YUSUR2_5TUPLE_MAX_PRI            7
#define YUSUR2_5TUPLE_MIN_PRI            1

/* The overhead from MTU to max frame size. */
#define YUSUR2_ETH_OVERHEAD (RTE_ETHER_HDR_LEN + RTE_ETHER_CRC_LEN)

/* bit of VXLAN tunnel type | 7 bits of zeros  | 8 bits of zeros*/
#define YUSUR2_FDIR_VXLAN_TUNNEL_TYPE    0x8000
/* bit of NVGRE tunnel type | 7 bits of zeros  | 8 bits of zeros*/
#define YUSUR2_FDIR_NVGRE_TUNNEL_TYPE    0x0

#define YUSUR2_RSS_OFFLOAD_ALL ( \
	ETH_RSS_IPV4 | \
	ETH_RSS_NONFRAG_IPV4_TCP | \
	ETH_RSS_NONFRAG_IPV4_UDP | \
	ETH_RSS_IPV6 | \
	ETH_RSS_NONFRAG_IPV6_TCP | \
	ETH_RSS_NONFRAG_IPV6_UDP | \
	ETH_RSS_IPV6_EX | \
	ETH_RSS_IPV6_TCP_EX | \
	ETH_RSS_IPV6_UDP_EX)

#define YUSUR2_VF_IRQ_ENABLE_MASK        3          /* vf irq enable mask */
#define YUSUR2_VF_MAXMSIVECTOR           1

#define YUSUR2_MISC_VEC_ID               RTE_INTR_VEC_ZERO_OFFSET
#define YUSUR2_RX_VEC_START              RTE_INTR_VEC_RXTX_OFFSET

#define YUSUR2_SECTX_MINSECIFG_MASK      0x0000000F

#define YUSUR2_MACSEC_PNTHRSH            0xFFFFFE00

#define YUSUR2_MAX_FDIR_FILTER_NUM       (1024 * 32)
#define YUSUR2_MAX_L2_TN_FILTER_NUM      128

#define MAC_TYPE_FILTER_SUP_EXT(type)    do {\
	if ((type) != yusur2_mac_82599EB && (type) != yusur2_mac_X540)\
		return -ENOTSUP;\
} while (0)

#define MAC_TYPE_FILTER_SUP(type)    do {\
	if ((type) != yusur2_mac_82599EB && (type) != yusur2_mac_X540 &&\
		(type) != yusur2_mac_X550 && (type) != yusur2_mac_X550EM_x &&\
		(type) != yusur2_mac_X550EM_a)\
		return -ENOTSUP;\
} while (0)

/* Link speed for X550 auto negotiation */
#define YUSUR2_LINK_SPEED_X550_AUTONEG	(YUSUR2_LINK_SPEED_100_FULL | \
					 YUSUR2_LINK_SPEED_1GB_FULL | \
					 YUSUR2_LINK_SPEED_2_5GB_FULL | \
					 YUSUR2_LINK_SPEED_5GB_FULL | \
					 YUSUR2_LINK_SPEED_10GB_FULL)

/*
 * Information about the fdir mode.
 */
struct yusur2_hw_fdir_mask {
	uint16_t vlan_tci_mask;
	uint32_t src_ipv4_mask;
	uint32_t dst_ipv4_mask;
	uint16_t src_ipv6_mask;
	uint16_t dst_ipv6_mask;
	uint16_t src_port_mask;
	uint16_t dst_port_mask;
	uint16_t flex_bytes_mask;
	uint8_t  mac_addr_byte_mask;
	uint32_t tunnel_id_mask;
	uint8_t  tunnel_type_mask;
};

struct yusur2_fdir_filter {
	TAILQ_ENTRY(yusur2_fdir_filter) entries;
	union yusur2_atr_input yusur2_fdir; /* key of fdir filter*/
	uint32_t fdirflags; /* drop or forward */
	uint32_t fdirhash; /* hash value for fdir */
	uint8_t queue; /* assigned rx queue */
};

/* list of fdir filters */
TAILQ_HEAD(yusur2_fdir_filter_list, yusur2_fdir_filter);

struct yusur2_fdir_rule {
	struct yusur2_hw_fdir_mask mask;
	union yusur2_atr_input yusur2_fdir; /* key of fdir filter*/
	bool b_spec; /* If TRUE, yusur2_fdir, fdirflags, queue have meaning. */
	bool b_mask; /* If TRUE, mask has meaning. */
	enum rte_fdir_mode mode; /* IP, MAC VLAN, Tunnel */
	uint32_t fdirflags; /* drop or forward */
	uint32_t soft_id; /* an unique value for this rule */
	uint8_t queue; /* assigned rx queue */
	uint8_t flex_bytes_offset;
};

struct yusur2_hw_fdir_info {
	struct yusur2_hw_fdir_mask mask;
	uint8_t     flex_bytes_offset;
	uint16_t    collision;
	uint16_t    free;
	uint16_t    maxhash;
	uint8_t     maxlen;
	uint64_t    add;
	uint64_t    remove;
	uint64_t    f_add;
	uint64_t    f_remove;
	struct yusur2_fdir_filter_list fdir_list; /* filter list*/
	/* store the pointers of the filters, index is the hash value. */
	struct yusur2_fdir_filter **hash_map;
	struct rte_hash *hash_handle; /* cuckoo hash handler */
	bool mask_added; /* If already got mask from consistent filter */
};

struct yusur2_rte_flow_rss_conf {
	struct rte_flow_action_rss conf; /**< RSS parameters. */
	uint8_t key[YUSUR2_HKEY_MAX_INDEX * sizeof(uint32_t)]; /* Hash key. */
	uint16_t queue[YUSUR2_MAX_RX_QUEUE_NUM]; /**< Queues indices to use. */
};

/* structure for interrupt relative data */
struct yusur2_interrupt {
	uint32_t flags;
	uint32_t mask;
	/*to save original mask during delayed handler */
	uint32_t mask_original;
};

struct yusur2_stat_mapping_registers {
	uint32_t tqsm[YUSUR2_NB_STAT_MAPPING_REGS];
	uint32_t rqsmr[YUSUR2_NB_STAT_MAPPING_REGS];
};

struct yusur2_vfta {
	uint32_t vfta[YUSUR2_VFTA_SIZE];
};

struct yusur2_hwstrip {
	uint32_t bitmap[YUSUR2_HWSTRIP_BITMAP_SIZE];
};

/*
 * VF data which used by PF host only
 */
#define YUSUR2_MAX_VF_MC_ENTRIES		30
#define YUSUR2_MAX_MR_RULE_ENTRIES	4 /* number of mirroring rules supported */
#define YUSUR2_MAX_UTA                   128

struct yusur2_uta_info {
	uint8_t  uc_filter_type;
	uint16_t uta_in_use;
	uint32_t uta_shadow[YUSUR2_MAX_UTA];
};

#define YUSUR2_MAX_MIRROR_RULES 4  /* Maximum nb. of mirror rules. */

struct yusur2_mirror_info {
	struct rte_eth_mirror_conf mr_conf[YUSUR2_MAX_MIRROR_RULES];
	/**< store PF mirror rules configuration*/
};

struct yusur2_vf_info {
	uint8_t vf_mac_addresses[RTE_ETHER_ADDR_LEN];
	uint16_t vf_mc_hashes[YUSUR2_MAX_VF_MC_ENTRIES];
	uint16_t num_vf_mc_hashes;
	uint16_t default_vf_vlan_id;
	uint16_t vlans_enabled;
	bool clear_to_send;
	uint16_t tx_rate[YUSUR2_MAX_QUEUE_NUM_PER_VF];
	uint16_t vlan_count;
	uint8_t spoofchk_enabled;
	uint8_t api_version;
	uint16_t switch_domain_id;
	uint16_t xcast_mode;
};

/*
 *  Possible l4type of 5tuple filters.
 */
enum yusur2_5tuple_protocol {
	YUSUR2_FILTER_PROTOCOL_TCP = 0,
	YUSUR2_FILTER_PROTOCOL_UDP,
	YUSUR2_FILTER_PROTOCOL_SCTP,
	YUSUR2_FILTER_PROTOCOL_NONE,
};

TAILQ_HEAD(yusur2_5tuple_filter_list, yusur2_5tuple_filter);

struct yusur2_5tuple_filter_info {
	uint32_t dst_ip;
	uint32_t src_ip;
	uint16_t dst_port;
	uint16_t src_port;
	enum yusur2_5tuple_protocol proto;        /* l4 protocol. */
	uint8_t priority;        /* seven levels (001b-111b), 111b is highest,
				      used when more than one filter matches. */
	uint8_t dst_ip_mask:1,   /* if mask is 1b, do not compare dst ip. */
		src_ip_mask:1,   /* if mask is 1b, do not compare src ip. */
		dst_port_mask:1, /* if mask is 1b, do not compare dst port. */
		src_port_mask:1, /* if mask is 1b, do not compare src port. */
		proto_mask:1;    /* if mask is 1b, do not compare protocol. */
};

/* 5tuple filter structure */
struct yusur2_5tuple_filter {
	TAILQ_ENTRY(yusur2_5tuple_filter) entries;
	uint16_t index;       /* the index of 5tuple filter */
	struct yusur2_5tuple_filter_info filter_info;
	uint16_t queue;       /* rx queue assigned to */
};

#define YUSUR2_5TUPLE_ARRAY_SIZE \
	(RTE_ALIGN(YUSUR2_MAX_FTQF_FILTERS, (sizeof(uint32_t) * NBBY)) / \
	 (sizeof(uint32_t) * NBBY))

struct yusur2_ethertype_filter {
	uint16_t ethertype;
	uint32_t etqf;
	uint32_t etqs;
	/**
	 * If this filter is added by configuration,
	 * it should not be removed.
	 */
	bool     conf;
};

/*
 * Structure to store filters' info.
 */
struct yusur2_filter_info {
	uint8_t ethertype_mask;  /* Bit mask for every used ethertype filter */
	/* store used ethertype filters*/
	struct yusur2_ethertype_filter ethertype_filters[YUSUR2_MAX_ETQF_FILTERS];
	/* Bit mask for every used 5tuple filter */
	uint32_t fivetuple_mask[YUSUR2_5TUPLE_ARRAY_SIZE];
	struct yusur2_5tuple_filter_list fivetuple_list;
	/* store the SYN filter info */
	uint32_t syn_info;
	/* store the rss filter info */
	struct yusur2_rte_flow_rss_conf rss_info;
};

struct yusur2_l2_tn_key {
	enum rte_eth_tunnel_type          l2_tn_type;
	uint32_t                          tn_id;
};

struct yusur2_l2_tn_filter {
	TAILQ_ENTRY(yusur2_l2_tn_filter)    entries;
	struct yusur2_l2_tn_key             key;
	uint32_t                           pool;
};

TAILQ_HEAD(yusur2_l2_tn_filter_list, yusur2_l2_tn_filter);

struct yusur2_l2_tn_info {
	struct yusur2_l2_tn_filter_list      l2_tn_list;
	struct yusur2_l2_tn_filter         **hash_map;
	struct rte_hash                    *hash_handle;
	bool e_tag_en; /* e-tag enabled */
	bool e_tag_fwd_en; /* e-tag based forwarding enabled */
	bool e_tag_ether_type; /* ether type for e-tag */
};

struct rte_flow {
	enum rte_filter_type filter_type;
	void *rule;
};

struct yusur2_macsec_setting {
	uint8_t offload_en;
	uint8_t encrypt_en;
	uint8_t replayprotect_en;
};

/*
 * Statistics counters collected by the MACsec
 */
struct yusur2_macsec_stats {
	/* TX port statistics */
	uint64_t out_pkts_untagged;
	uint64_t out_pkts_encrypted;
	uint64_t out_pkts_protected;
	uint64_t out_octets_encrypted;
	uint64_t out_octets_protected;

	/* RX port statistics */
	uint64_t in_pkts_untagged;
	uint64_t in_pkts_badtag;
	uint64_t in_pkts_nosci;
	uint64_t in_pkts_unknownsci;
	uint64_t in_octets_decrypted;
	uint64_t in_octets_validated;

	/* RX SC statistics */
	uint64_t in_pkts_unchecked;
	uint64_t in_pkts_delayed;
	uint64_t in_pkts_late;

	/* RX SA statistics */
	uint64_t in_pkts_ok;
	uint64_t in_pkts_invalid;
	uint64_t in_pkts_notvalid;
	uint64_t in_pkts_unusedsa;
	uint64_t in_pkts_notusingsa;
};

/* The configuration of bandwidth */
struct yusur2_bw_conf {
	uint8_t tc_num; /* Number of TCs. */
};

/* Struct to store Traffic Manager shaper profile. */
struct yusur2_tm_shaper_profile {
	TAILQ_ENTRY(yusur2_tm_shaper_profile) node;
	uint32_t shaper_profile_id;
	uint32_t reference_count;
	struct rte_tm_shaper_params profile;
};

TAILQ_HEAD(yusur2_shaper_profile_list, yusur2_tm_shaper_profile);

/* node type of Traffic Manager */
enum yusur2_tm_node_type {
	YUSUR2_TM_NODE_TYPE_PORT,
	YUSUR2_TM_NODE_TYPE_TC,
	YUSUR2_TM_NODE_TYPE_QUEUE,
	YUSUR2_TM_NODE_TYPE_MAX,
};

/* Struct to store Traffic Manager node configuration. */
struct yusur2_tm_node {
	TAILQ_ENTRY(yusur2_tm_node) node;
	uint32_t id;
	uint32_t priority;
	uint32_t weight;
	uint32_t reference_count;
	uint16_t no;
	struct yusur2_tm_node *parent;
	struct yusur2_tm_shaper_profile *shaper_profile;
	struct rte_tm_node_params params;
};

TAILQ_HEAD(yusur2_tm_node_list, yusur2_tm_node);

/* The configuration of Traffic Manager */
struct yusur2_tm_conf {
	struct yusur2_shaper_profile_list shaper_profile_list;
	struct yusur2_tm_node *root; /* root node - port */
	struct yusur2_tm_node_list tc_list; /* node list for all the TCs */
	struct yusur2_tm_node_list queue_list; /* node list for all the queues */
	/**
	 * The number of added TC nodes.
	 * It should be no more than the TC number of this port.
	 */
	uint32_t nb_tc_node;
	/**
	 * The number of added queue nodes.
	 * It should be no more than the queue number of this port.
	 */
	uint32_t nb_queue_node;
	/**
	 * This flag is used to check if APP can change the TM node
	 * configuration.
	 * When it's true, means the configuration is applied to HW,
	 * APP should not change the configuration.
	 * As we don't support on-the-fly configuration, when starting
	 * the port, APP should call the hierarchy_commit API to set this
	 * flag to true. When stopping the port, this flag should be set
	 * to false.
	 */
	bool committed;
};

/*
 * Structure to store private data for each driver instance (for each port).
 */
struct yusur2_adapter {
	struct yusur2_hw             hw;
	struct yusur2_hw_stats       stats;
	struct yusur2_macsec_stats   macsec_stats;
	struct yusur2_macsec_setting	macsec_setting;
	struct yusur2_hw_fdir_info   fdir;
	struct yusur2_interrupt      intr;
	struct yusur2_stat_mapping_registers stat_mappings;
	struct yusur2_vfta           shadow_vfta;
	struct yusur2_hwstrip		hwstrip;
	struct yusur2_dcb_config     dcb_config;
	struct yusur2_mirror_info    mr_data;
	struct yusur2_vf_info        *vfdata;
	struct yusur2_uta_info       uta_info;
#ifdef RTE_LIBRTE_YUSUR2_BYPASS
	struct yusur2_bypass_info    bps;
#endif /* RTE_LIBRTE_YUSUR2_BYPASS */
	struct yusur2_filter_info    filter;
	struct yusur2_l2_tn_info     l2_tn;
	struct yusur2_bw_conf        bw_conf;
#ifdef RTE_LIBRTE_SECURITY
	struct yusur2_ipsec          ipsec;
#endif
	bool rx_bulk_alloc_allowed;
	bool rx_vec_allowed;
	struct rte_timecounter      systime_tc;
	struct rte_timecounter      rx_tstamp_tc;
	struct rte_timecounter      tx_tstamp_tc;
 	struct yusur2_tm_conf        tm_conf;

	/* For RSS reta table update */
	uint8_t rss_reta_updated;

	/* Used for VF link sync with PF's physical and logical (by checking
	 * mailbox status) link status.
	 */
	uint8_t pflink_fullchk;
	uint8_t mac_ctrl_frame_fwd;
	rte_atomic32_t link_thread_running;
	pthread_t link_thread_tid;
};

struct yusur2_vf_representor {
	uint16_t vf_id;
	uint16_t switch_domain_id;
	struct rte_eth_dev *pf_ethdev;
};

int yusur2_vf_representor_init(struct rte_eth_dev *ethdev, void *init_params);
int yusur2_vf_representor_uninit(struct rte_eth_dev *ethdev);

#define YUSUR2_DEV_PRIVATE_TO_HW(adapter)\
	(&((struct yusur2_adapter *)adapter)->hw)

#define YUSUR2_DEV_PRIVATE_TO_STATS(adapter) \
	(&((struct yusur2_adapter *)adapter)->stats)

#define YUSUR2_DEV_PRIVATE_TO_MACSEC_STATS(adapter) \
	(&((struct yusur2_adapter *)adapter)->macsec_stats)

#define YUSUR2_DEV_PRIVATE_TO_MACSEC_SETTING(adapter) \
	(&((struct yusur2_adapter *)adapter)->macsec_setting)

#define YUSUR2_DEV_PRIVATE_TO_INTR(adapter) \
	(&((struct yusur2_adapter *)adapter)->intr)

#define YUSUR2_DEV_PRIVATE_TO_FDIR_INFO(adapter) \
	(&((struct yusur2_adapter *)adapter)->fdir)

#define YUSUR2_DEV_PRIVATE_TO_STAT_MAPPINGS(adapter) \
	(&((struct yusur2_adapter *)adapter)->stat_mappings)

#define YUSUR2_DEV_PRIVATE_TO_VFTA(adapter) \
	(&((struct yusur2_adapter *)adapter)->shadow_vfta)

#define YUSUR2_DEV_PRIVATE_TO_HWSTRIP_BITMAP(adapter) \
	(&((struct yusur2_adapter *)adapter)->hwstrip)

#define YUSUR2_DEV_PRIVATE_TO_DCB_CFG(adapter) \
	(&((struct yusur2_adapter *)adapter)->dcb_config)

#define YUSUR2_DEV_PRIVATE_TO_P_VFDATA(adapter) \
	(&((struct yusur2_adapter *)adapter)->vfdata)

#define YUSUR2_DEV_PRIVATE_TO_PFDATA(adapter) \
	(&((struct yusur2_adapter *)adapter)->mr_data)

#define YUSUR2_DEV_PRIVATE_TO_UTA(adapter) \
	(&((struct yusur2_adapter *)adapter)->uta_info)

#define YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(adapter) \
	(&((struct yusur2_adapter *)adapter)->filter)

#define YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(adapter) \
	(&((struct yusur2_adapter *)adapter)->l2_tn)

#define YUSUR2_DEV_PRIVATE_TO_BW_CONF(adapter) \
	(&((struct yusur2_adapter *)adapter)->bw_conf)

#define YUSUR2_DEV_PRIVATE_TO_TM_CONF(adapter) \
	(&((struct yusur2_adapter *)adapter)->tm_conf)

#define YUSUR2_DEV_PRIVATE_TO_IPSEC(adapter)\
	(&((struct yusur2_adapter *)adapter)->ipsec)

/*
 * RX/TX function prototypes
 */
void yusur2_dev_clear_queues(struct rte_eth_dev *dev);

void yusur2_dev_free_queues(struct rte_eth_dev *dev);

void yusur2_dev_rx_queue_release(void *rxq);

void yusur2_dev_tx_queue_release(void *txq);

int  yusur2_dev_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc, unsigned int socket_id,
		const struct rte_eth_rxconf *rx_conf,
		struct rte_mempool *mb_pool);

int  yusur2_dev_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc, unsigned int socket_id,
		const struct rte_eth_txconf *tx_conf);

uint32_t yusur2_dev_rx_queue_count(struct rte_eth_dev *dev,
		uint16_t rx_queue_id);

int yusur2_dev_rx_descriptor_done(void *rx_queue, uint16_t offset);

int yusur2_dev_rx_descriptor_status(void *rx_queue, uint16_t offset);
int yusur2_dev_tx_descriptor_status(void *tx_queue, uint16_t offset);

int yusur2_dev_rx_init(struct rte_eth_dev *dev);

void yusur2_dev_tx_init(struct rte_eth_dev *dev);

int yusur2_dev_rxtx_start(struct rte_eth_dev *dev);

int yusur2_dev_rx_queue_start(struct rte_eth_dev *dev, uint16_t rx_queue_id);

int yusur2_dev_rx_queue_stop(struct rte_eth_dev *dev, uint16_t rx_queue_id);

int yusur2_dev_tx_queue_start(struct rte_eth_dev *dev, uint16_t tx_queue_id);

int yusur2_dev_tx_queue_stop(struct rte_eth_dev *dev, uint16_t tx_queue_id);

void yusur2_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo);

void yusur2_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo);

int yusur2vf_dev_rx_init(struct rte_eth_dev *dev);

void yusur2vf_dev_tx_init(struct rte_eth_dev *dev);

void yusur2vf_dev_rxtx_start(struct rte_eth_dev *dev);

uint16_t yusur2_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts);

uint16_t yusur2_recv_pkts_bulk_alloc(void *rx_queue, struct rte_mbuf **rx_pkts,
				    uint16_t nb_pkts);

uint16_t yusur2_recv_pkts_lro_single_alloc(void *rx_queue,
		struct rte_mbuf **rx_pkts, uint16_t nb_pkts);
uint16_t yusur2_recv_pkts_lro_bulk_alloc(void *rx_queue,
		struct rte_mbuf **rx_pkts, uint16_t nb_pkts);

uint16_t yusur2_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t yusur2_xmit_pkts_simple(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t yusur2_prep_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

int yusur2_dev_rss_hash_update(struct rte_eth_dev *dev,
			      struct rte_eth_rss_conf *rss_conf);

int yusur2_dev_rss_hash_conf_get(struct rte_eth_dev *dev,
				struct rte_eth_rss_conf *rss_conf);

uint16_t yusur2_reta_size_get(enum yusur2_mac_type mac_type);

uint32_t yusur2_reta_reg_get(enum yusur2_mac_type mac_type, uint16_t reta_idx);

uint32_t yusur2_mrqc_reg_get(enum yusur2_mac_type mac_type);

uint32_t yusur2_rssrk_reg_get(enum yusur2_mac_type mac_type, uint8_t i);

bool yusur2_rss_update_sp(enum yusur2_mac_type mac_type);

int yusur2_add_del_ntuple_filter(struct rte_eth_dev *dev,
			struct rte_eth_ntuple_filter *filter,
			bool add);
int yusur2_add_del_ethertype_filter(struct rte_eth_dev *dev,
			struct rte_eth_ethertype_filter *filter,
			bool add);
int yusur2_syn_filter_set(struct rte_eth_dev *dev,
			struct rte_eth_syn_filter *filter,
			bool add);
int
yusur2_dev_l2_tunnel_filter_add(struct rte_eth_dev *dev,
			       struct rte_eth_l2_tunnel_conf *l2_tunnel,
			       bool restore);
int
yusur2_dev_l2_tunnel_filter_del(struct rte_eth_dev *dev,
			       struct rte_eth_l2_tunnel_conf *l2_tunnel);
void yusur2_filterlist_init(void);
void yusur2_filterlist_flush(void);
/*
 * Flow director function prototypes
 */
int yusur2_fdir_configure(struct rte_eth_dev *dev);
int yusur2_fdir_set_input_mask(struct rte_eth_dev *dev);
int yusur2_fdir_set_flexbytes_offset(struct rte_eth_dev *dev,
				    uint16_t offset);
int yusur2_fdir_filter_program(struct rte_eth_dev *dev,
			      struct yusur2_fdir_rule *rule,
			      bool del, bool update);

void yusur2_configure_dcb(struct rte_eth_dev *dev);

int
yusur2_dev_link_update_share(struct rte_eth_dev *dev,
			    int wait_to_complete, int vf);

/*
 * misc function prototypes
 */
void yusur2_vlan_hw_filter_enable(struct rte_eth_dev *dev);

void yusur2_vlan_hw_filter_disable(struct rte_eth_dev *dev);

void yusur2_vlan_hw_strip_config(struct rte_eth_dev *dev);

int yusur2_pf_host_init(struct rte_eth_dev *eth_dev);

void yusur2_pf_host_uninit(struct rte_eth_dev *eth_dev);

void yusur2_pf_mbx_process(struct rte_eth_dev *eth_dev);

int yusur2_pf_host_configure(struct rte_eth_dev *eth_dev);

uint32_t yusur2_convert_vm_rx_mask_to_val(uint16_t rx_mask, uint32_t orig_val);

int yusur2_fdir_ctrl_func(struct rte_eth_dev *dev,
			enum rte_filter_op filter_op, void *arg);
void yusur2_fdir_filter_restore(struct rte_eth_dev *dev);
int yusur2_clear_all_fdir_filter(struct rte_eth_dev *dev);

extern const struct rte_flow_ops yusur2_flow_ops;

void yusur2_clear_all_ethertype_filter(struct rte_eth_dev *dev);
void yusur2_clear_all_ntuple_filter(struct rte_eth_dev *dev);
void yusur2_clear_syn_filter(struct rte_eth_dev *dev);
int yusur2_clear_all_l2_tn_filter(struct rte_eth_dev *dev);

int yusur2_disable_sec_tx_path_generic(struct yusur2_hw *hw);

int yusur2_enable_sec_tx_path_generic(struct yusur2_hw *hw);

int yusur2_vt_check(struct yusur2_hw *hw);
int yusur2_set_vf_rate_limit(struct rte_eth_dev *dev, uint16_t vf,
			    uint16_t tx_rate, uint64_t q_msk);
bool is_yusur2_supported(struct rte_eth_dev *dev);
int yusur2_tm_ops_get(struct rte_eth_dev *dev, void *ops);
void yusur2_tm_conf_init(struct rte_eth_dev *dev);
void yusur2_tm_conf_uninit(struct rte_eth_dev *dev);
int yusur2_set_queue_rate_limit(struct rte_eth_dev *dev, uint16_t queue_idx,
			       uint16_t tx_rate);
int yusur2_rss_conf_init(struct yusur2_rte_flow_rss_conf *out,
			const struct rte_flow_action_rss *in);
int yusur2_action_rss_same(const struct rte_flow_action_rss *comp,
			  const struct rte_flow_action_rss *with);
int yusur2_config_rss_filter(struct rte_eth_dev *dev,
		struct yusur2_rte_flow_rss_conf *conf, bool add);

void yusur2_dev_macsec_register_enable(struct rte_eth_dev *dev,
		struct yusur2_macsec_setting *macsec_setting);

void yusur2_dev_macsec_register_disable(struct rte_eth_dev *dev);

void yusur2_dev_macsec_setting_save(struct rte_eth_dev *dev,
		struct yusur2_macsec_setting *macsec_setting);

void yusur2_dev_macsec_setting_reset(struct rte_eth_dev *dev);

static inline int
yusur2_ethertype_filter_lookup(struct yusur2_filter_info *filter_info,
			      uint16_t ethertype)
{
	int i;

	for (i = 0; i < YUSUR2_MAX_ETQF_FILTERS; i++) {
		if (filter_info->ethertype_filters[i].ethertype == ethertype &&
		    (filter_info->ethertype_mask & (1 << i)))
			return i;
	}
	return -1;
}

static inline int
yusur2_ethertype_filter_insert(struct yusur2_filter_info *filter_info,
			      struct yusur2_ethertype_filter *ethertype_filter)
{
	int i;

	for (i = 0; i < YUSUR2_MAX_ETQF_FILTERS; i++) {
		if (!(filter_info->ethertype_mask & (1 << i))) {
			filter_info->ethertype_mask |= 1 << i;
			filter_info->ethertype_filters[i].ethertype =
				ethertype_filter->ethertype;
			filter_info->ethertype_filters[i].etqf =
				ethertype_filter->etqf;
			filter_info->ethertype_filters[i].etqs =
				ethertype_filter->etqs;
			filter_info->ethertype_filters[i].conf =
				ethertype_filter->conf;
			return i;
		}
	}
	return -1;
}

static inline int
yusur2_ethertype_filter_remove(struct yusur2_filter_info *filter_info,
			      uint8_t idx)
{
	if (idx >= YUSUR2_MAX_ETQF_FILTERS)
		return -1;
	filter_info->ethertype_mask &= ~(1 << idx);
	filter_info->ethertype_filters[idx].ethertype = 0;
	filter_info->ethertype_filters[idx].etqf = 0;
	filter_info->ethertype_filters[idx].etqs = 0;
	filter_info->ethertype_filters[idx].etqs = FALSE;
	return idx;
}

#endif /* _YUSUR2_ETHDEV_H_ */
