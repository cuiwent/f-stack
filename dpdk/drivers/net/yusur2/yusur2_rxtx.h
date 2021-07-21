/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_RXTX_H_
#define _YUSUR2_RXTX_H_

/*
 * Rings setup and release.
 *
 * TDBA/RDBA should be aligned on 16 byte boundary. But TDLEN/RDLEN should be
 * multiple of 128 bytes. So we align TDBA/RDBA on 128 byte boundary. This will
 * also optimize cache line size effect. H/W supports up to cache line size 128.
 */
#define	YUSUR2_ALIGN	128

#define YUSUR2_RXD_ALIGN	(YUSUR2_ALIGN / sizeof(union yusur2_adv_rx_desc))
#define YUSUR2_TXD_ALIGN	(YUSUR2_ALIGN / sizeof(union yusur2_adv_tx_desc))

/*
 * Maximum number of Ring Descriptors.
 *
 * Since RDLEN/TDLEN should be multiple of 128 bytes, the number of ring
 * descriptors should meet the following condition:
 *      (num_ring_desc * sizeof(rx/tx descriptor)) % 128 == 0
 */
#define	YUSUR2_MIN_RING_DESC	32
#define	YUSUR2_MAX_RING_DESC	4096

#define RTE_PMD_YUSUR2_TX_MAX_BURST 32
#define RTE_PMD_YUSUR2_RX_MAX_BURST 32
#define RTE_YUSUR2_TX_MAX_FREE_BUF_SZ 64

#define RTE_YUSUR2_DESCS_PER_LOOP    4

#ifdef RTE_YUSUR2_INC_VECTOR
#define RTE_YUSUR2_RXQ_REARM_THRESH      32
#define RTE_YUSUR2_MAX_RX_BURST          RTE_YUSUR2_RXQ_REARM_THRESH
#endif

#define RX_RING_SZ ((YUSUR2_MAX_RING_DESC + RTE_PMD_YUSUR2_RX_MAX_BURST) * \
		    sizeof(union yusur2_adv_rx_desc))

#ifdef RTE_PMD_PACKET_PREFETCH
#define rte_packet_prefetch(p)  rte_prefetch1(p)
#else
#define rte_packet_prefetch(p)  do {} while(0)
#endif

#define RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS  10
#define RTE_YUSUR2_WAIT_100_US               100
#define RTE_YUSUR2_VMTXSW_REGISTER_COUNT     2

#define YUSUR2_TX_MAX_SEG                    40

#define YUSUR2_TX_MIN_PKT_LEN		     14

#define YUSUR2_PACKET_TYPE_MASK_82599        0X7F
#define YUSUR2_PACKET_TYPE_MASK_X550         0X10FF
#define YUSUR2_PACKET_TYPE_MASK_TUNNEL       0XFF
#define YUSUR2_PACKET_TYPE_TUNNEL_BIT        0X1000

#define YUSUR2_PACKET_TYPE_MAX               0X80
#define YUSUR2_PACKET_TYPE_TN_MAX            0X100
#define YUSUR2_PACKET_TYPE_SHIFT             0X04

/**
 * Structure associated with each descriptor of the RX ring of a RX queue.
 */
struct yusur2_rx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with RX descriptor. */
};

struct yusur2_scattered_rx_entry {
	struct rte_mbuf *fbuf; /**< First segment of the fragmented packet. */
};

/**
 * Structure associated with each descriptor of the TX ring of a TX queue.
 */
struct yusur2_tx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
	uint16_t next_id; /**< Index of next descriptor in ring. */
	uint16_t last_id; /**< Index of last scattered descriptor. */
};

/**
 * Structure associated with each descriptor of the TX ring of a TX queue.
 */
struct yusur2_tx_entry_v {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
};

/**
 * Structure associated with each RX queue.
 */
struct yusur2_rx_queue {
	struct rte_mempool  *mb_pool; /**< mbuf pool to populate RX ring. */
	volatile union yusur2_adv_rx_desc *rx_ring; /**< RX ring virtual address. */
	uint64_t            rx_ring_phys_addr; /**< RX ring DMA address. */
	volatile uint32_t   *rdt_reg_addr; /**< RDT register address. */
	volatile uint32_t   *rdh_reg_addr; /**< RDH register address. */
	struct yusur2_rx_entry *sw_ring; /**< address of RX software ring. */
	struct yusur2_scattered_rx_entry *sw_sc_ring; /**< address of scattered Rx software ring. */
	struct rte_mbuf *pkt_first_seg; /**< First segment of current packet. */
	struct rte_mbuf *pkt_last_seg; /**< Last segment of current packet. */
	uint64_t            mbuf_initializer; /**< value to init mbufs */
	uint16_t            nb_rx_desc; /**< number of RX descriptors. */
	uint16_t            rx_tail;  /**< current value of RDT register. */
	uint16_t            nb_rx_hold; /**< number of held free RX desc. */
	uint16_t rx_nb_avail; /**< nr of staged pkts ready to ret to app */
	uint16_t rx_next_avail; /**< idx of next staged pkt to ret to app */
	uint16_t rx_free_trigger; /**< triggers rx buffer allocation */
	uint8_t            rx_using_sse;
	/**< indicates that vector RX is in use */
#ifdef RTE_LIBRTE_SECURITY
	uint8_t            using_ipsec;
	/**< indicates that IPsec RX feature is in use */
#endif
#ifdef RTE_YUSUR2_INC_VECTOR
	uint16_t            rxrearm_nb;     /**< number of remaining to be re-armed */
	uint16_t            rxrearm_start;  /**< the idx we start the re-arming from */
#endif
	uint16_t            rx_free_thresh; /**< max free RX desc to hold. */
	uint16_t            queue_id; /**< RX queue index. */
	uint16_t            reg_idx;  /**< RX queue register index. */
	uint16_t            pkt_type_mask;  /**< Packet type mask for different NICs. */
	uint16_t            port_id;  /**< Device port identifier. */
	uint8_t             crc_len;  /**< 0 if CRC stripped, 4 otherwise. */
	uint8_t             drop_en;  /**< If not 0, set SRRCTL.Drop_En. */
	uint8_t             rx_deferred_start; /**< not in global dev start. */
	/** flags to set in mbuf when a vlan is detected. */
	uint64_t            vlan_flags;
	uint64_t	    offloads; /**< Rx offloads with DEV_RX_OFFLOAD_* */
	/** need to alloc dummy mbuf, for wraparound when scanning hw ring */
	struct rte_mbuf fake_mbuf;
	/** hold packets to return to application */
	struct rte_mbuf *rx_stage[RTE_PMD_YUSUR2_RX_MAX_BURST*2];
};

/**
 * YUSUR2 CTX Constants
 */
enum yusur2_advctx_num {
	YUSUR2_CTX_0    = 0, /**< CTX0 */
	YUSUR2_CTX_1    = 1, /**< CTX1  */
	YUSUR2_CTX_NUM  = 2, /**< CTX NUMBER  */
};

/** Offload features */
union yusur2_tx_offload {
	uint64_t data[2];
	struct {
		uint64_t l2_len:7; /**< L2 (MAC) Header Length. */
		uint64_t l3_len:9; /**< L3 (IP) Header Length. */
		uint64_t l4_len:8; /**< L4 (TCP/UDP) Header Length. */
		uint64_t tso_segsz:16; /**< TCP TSO segment size */
		uint64_t vlan_tci:16;
		/**< VLAN Tag Control Identifier (CPU order). */

		/* fields for TX offloading of tunnels */
		uint64_t outer_l3_len:8; /**< Outer L3 (IP) Hdr Length. */
		uint64_t outer_l2_len:8; /**< Outer L2 (MAC) Hdr Length. */
#ifdef RTE_LIBRTE_SECURITY
		/* inline ipsec related*/
		uint64_t sa_idx:8;	/**< TX SA database entry index */
		uint64_t sec_pad_len:4;	/**< padding length */
#endif
	};
};

/*
 * Compare mask for vlan_macip_len.data,
 * should be in sync with yusur2_vlan_macip.f layout.
 * */
#define TX_VLAN_CMP_MASK        0xFFFF0000  /**< VLAN length - 16-bits. */
#define TX_MAC_LEN_CMP_MASK     0x0000FE00  /**< MAC length - 7-bits. */
#define TX_IP_LEN_CMP_MASK      0x000001FF  /**< IP  length - 9-bits. */
/** MAC+IP  length. */
#define TX_MACIP_LEN_CMP_MASK   (TX_MAC_LEN_CMP_MASK | TX_IP_LEN_CMP_MASK)

/**
 * Structure to check if new context need be built
 */

struct yusur2_advctx_info {
	uint64_t flags;           /**< ol_flags for context build. */
	/**< tx offload: vlan, tso, l2-l3-l4 lengths. */
	union yusur2_tx_offload tx_offload;
	/** compare mask for tx offload. */
	union yusur2_tx_offload tx_offload_mask;
};

/**
 * Structure associated with each TX queue.
 */
struct yusur2_tx_queue {
	/** TX ring virtual address. */
	volatile union yusur2_adv_tx_desc *tx_ring;
	uint64_t            tx_ring_phys_addr; /**< TX ring DMA address. */
	union {
		struct yusur2_tx_entry *sw_ring; /**< address of SW ring for scalar PMD. */
		struct yusur2_tx_entry_v *sw_ring_v; /**< address of SW ring for vector PMD */
	};
	volatile uint32_t   *tdt_reg_addr; /**< Address of TDT register. */
	uint16_t            nb_tx_desc;    /**< number of TX descriptors. */
	uint16_t            tx_tail;       /**< current value of TDT reg. */
	/**< Start freeing TX buffers if there are less free descriptors than
	     this value. */
	uint16_t            tx_free_thresh;
	/** Number of TX descriptors to use before RS bit is set. */
	uint16_t            tx_rs_thresh;
	/** Number of TX descriptors used since RS bit was set. */
	uint16_t            nb_tx_used;
	/** Index to last TX descriptor to have been cleaned. */
	uint16_t            last_desc_cleaned;
	/** Total number of TX descriptors ready to be allocated. */
	uint16_t            nb_tx_free;
	uint16_t tx_next_dd; /**< next desc to scan for DD bit */
	uint16_t tx_next_rs; /**< next desc to set RS bit */
	uint16_t            queue_id;      /**< TX queue index. */
	uint16_t            reg_idx;       /**< TX queue register index. */
	uint16_t            port_id;       /**< Device port identifier. */
	uint8_t             pthresh;       /**< Prefetch threshold register. */
	uint8_t             hthresh;       /**< Host threshold register. */
	uint8_t             wthresh;       /**< Write-back threshold reg. */
	uint64_t offloads; /**< Tx offload flags of DEV_TX_OFFLOAD_* */
	uint32_t            ctx_curr;      /**< Hardware context states. */
	/** Hardware context0 history. */
	struct yusur2_advctx_info ctx_cache[YUSUR2_CTX_NUM];
	const struct yusur2_txq_ops *ops;       /**< txq ops */
	uint8_t             tx_deferred_start; /**< not in global dev start. */
#ifdef RTE_LIBRTE_SECURITY
	uint8_t		    using_ipsec;
	/**< indicates that IPsec TX feature is in use */
#endif
};

struct yusur2_txq_ops {
	void (*release_mbufs)(struct yusur2_tx_queue *txq);
	void (*free_swring)(struct yusur2_tx_queue *txq);
	void (*reset)(struct yusur2_tx_queue *txq);
};

/*
 * Populate descriptors with the following info:
 * 1.) buffer_addr = phys_addr + headroom
 * 2.) cmd_type_len = DCMD_DTYP_FLAGS | pkt_len
 * 3.) olinfo_status = pkt_len << PAYLEN_SHIFT
 */

/* Defines for Tx descriptor */
#define DCMD_DTYP_FLAGS (YUSUR2_ADVTXD_DTYP_DATA |\
			 YUSUR2_ADVTXD_DCMD_IFCS |\
			 YUSUR2_ADVTXD_DCMD_DEXT |\
			 YUSUR2_ADVTXD_DCMD_EOP)


/* Takes an ethdev and a queue and sets up the tx function to be used based on
 * the queue parameters. Used in tx_queue_setup by primary process and then
 * in dev_init by secondary process when attaching to an existing ethdev.
 */
void yusur2_set_tx_function(struct rte_eth_dev *dev, struct yusur2_tx_queue *txq);

/**
 * Sets the rx_pkt_burst callback in the yusur2 rte_eth_dev instance.
 *
 * Sets the callback based on the device parameters:
 *  - yusur2_hw.rx_bulk_alloc_allowed
 *  - rte_eth_dev_data.scattered_rx
 *  - rte_eth_dev_data.lro
 *  - conditions checked in yusur2_rx_vec_condition_check()
 *
 *  This means that the parameters above have to be configured prior to calling
 *  to this function.
 *
 * @dev rte_eth_dev handle
 */
void yusur2_set_rx_function(struct rte_eth_dev *dev);

int yusur2_check_supported_loopback_mode(struct rte_eth_dev *dev);
uint16_t yusur2_recv_pkts_vec(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts);
uint16_t yusur2_recv_scattered_pkts_vec(void *rx_queue,
		struct rte_mbuf **rx_pkts, uint16_t nb_pkts);
int yusur2_rx_vec_dev_conf_condition_check(struct rte_eth_dev *dev);
int yusur2_rxq_vec_setup(struct yusur2_rx_queue *rxq);
void yusur2_rx_queue_release_mbufs_vec(struct yusur2_rx_queue *rxq);

extern const uint32_t yusur2_ptype_table[YUSUR2_PACKET_TYPE_MAX];
extern const uint32_t yusur2_ptype_table_tn[YUSUR2_PACKET_TYPE_TN_MAX];

#ifdef RTE_YUSUR2_INC_VECTOR

uint16_t yusur2_xmit_fixed_burst_vec(void *tx_queue, struct rte_mbuf **tx_pkts,
				    uint16_t nb_pkts);
int yusur2_txq_vec_setup(struct yusur2_tx_queue *txq);
#endif /* RTE_YUSUR2_INC_VECTOR */

uint64_t yusur2_get_tx_port_offloads(struct rte_eth_dev *dev);
uint64_t yusur2_get_rx_queue_offloads(struct rte_eth_dev *dev);
uint64_t yusur2_get_rx_port_offloads(struct rte_eth_dev *dev);
uint64_t yusur2_get_tx_queue_offloads(struct rte_eth_dev *dev);

#endif /* _YUSUR2_RXTX_H_ */
