/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include <sys/queue.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <inttypes.h>

#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_cycles.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ethdev_driver.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_sctp.h>
#include <rte_string_fns.h>
#include <rte_errno.h>
#include <rte_ip.h>
#include <rte_net.h>

#include "yusur2_logs.h"
#include "base/yusur2_api.h"
#include "yusur2_ethdev.h"
#include "base/yusur2_common.h"
#include "yusur2_rxtx.h"

#ifdef RTE_LIBRTE_IEEE1588
#define YUSUR2_TX_IEEE1588_TMST PKT_TX_IEEE1588_TMST
#else
#define YUSUR2_TX_IEEE1588_TMST 0
#endif
/* Bit Mask to indicate what bits required for building TX context */
#define YUSUR2_TX_OFFLOAD_MASK (			 \
		PKT_TX_OUTER_IPV6 |		 \
		PKT_TX_OUTER_IPV4 |		 \
		PKT_TX_IPV6 |			 \
		PKT_TX_IPV4 |			 \
		PKT_TX_VLAN_PKT |		 \
		PKT_TX_IP_CKSUM |		 \
		PKT_TX_L4_MASK |		 \
		PKT_TX_TCP_SEG |		 \
		PKT_TX_MACSEC |			 \
		PKT_TX_OUTER_IP_CKSUM |		 \
		PKT_TX_SEC_OFFLOAD |	 \
		YUSUR2_TX_IEEE1588_TMST)

#define YUSUR2_TX_OFFLOAD_NOTSUP_MASK \
		(PKT_TX_OFFLOAD_MASK ^ YUSUR2_TX_OFFLOAD_MASK)

#if 1
#define RTE_PMD_USE_PREFETCH
#endif

#ifdef RTE_PMD_USE_PREFETCH
/*
 * Prefetch a cache line into all cache levels.
 */
#define rte_yusur2_prefetch(p)   rte_prefetch0(p)
#else
#define rte_yusur2_prefetch(p)   do {} while (0)
#endif

/*********************************************************************
 *
 *  TX functions
 *
 **********************************************************************/

/*
 * Check for descriptors with their DD bit set and free mbufs.
 * Return the total number of buffers freed.
 */
static __rte_always_inline int
yusur2_tx_free_bufs(struct yusur2_tx_queue *txq)
{
	struct yusur2_tx_entry *txep;
	uint32_t status;
	int i, nb_free = 0;
	struct rte_mbuf *m, *free[RTE_YUSUR2_TX_MAX_FREE_BUF_SZ];

	/* check DD bit on threshold descriptor */
	status = txq->tx_ring[txq->tx_next_dd].wb.status;
	if (!(status & rte_cpu_to_le_32(YUSUR2_ADVTXD_STAT_DD)))
		return 0;

	/*
	 * first buffer to free from S/W ring is at index
	 * tx_next_dd - (tx_rs_thresh-1)
	 */
	txep = &(txq->sw_ring[txq->tx_next_dd - (txq->tx_rs_thresh - 1)]);

	for (i = 0; i < txq->tx_rs_thresh; ++i, ++txep) {
		/* free buffers one at a time */
		m = rte_pktmbuf_prefree_seg(txep->mbuf);
		txep->mbuf = NULL;

		if (unlikely(m == NULL))
			continue;

		if (nb_free >= RTE_YUSUR2_TX_MAX_FREE_BUF_SZ ||
		    (nb_free > 0 && m->pool != free[0]->pool)) {
			rte_mempool_put_bulk(free[0]->pool,
					     (void **)free, nb_free);
			nb_free = 0;
		}

		free[nb_free++] = m;
	}

	if (nb_free > 0)
		rte_mempool_put_bulk(free[0]->pool, (void **)free, nb_free);

	/* buffers were freed, update counters */
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free + txq->tx_rs_thresh);
	txq->tx_next_dd = (uint16_t)(txq->tx_next_dd + txq->tx_rs_thresh);
	if (txq->tx_next_dd >= txq->nb_tx_desc)
		txq->tx_next_dd = (uint16_t)(txq->tx_rs_thresh - 1);

	return txq->tx_rs_thresh;
}

/* Populate 4 descriptors with data from 4 mbufs */
static inline void
tx4(volatile union yusur2_adv_tx_desc *txdp, struct rte_mbuf **pkts)
{
	uint64_t buf_dma_addr;
	uint32_t pkt_len;
	int i;

	for (i = 0; i < 4; ++i, ++txdp, ++pkts) {
		buf_dma_addr = rte_mbuf_data_iova(*pkts);
		pkt_len = (*pkts)->data_len;

		/* write data to descriptor */
		txdp->read.buffer_addr = rte_cpu_to_le_64(buf_dma_addr);

		txdp->read.cmd_type_len =
			rte_cpu_to_le_32((uint32_t)DCMD_DTYP_FLAGS | pkt_len);

		txdp->read.olinfo_status =
			rte_cpu_to_le_32(pkt_len << YUSUR2_ADVTXD_PAYLEN_SHIFT);

		rte_prefetch0(&(*pkts)->pool);
	}
}

/* Populate 1 descriptor with data from 1 mbuf */
static inline void
tx1(volatile union yusur2_adv_tx_desc *txdp, struct rte_mbuf **pkts)
{
	uint64_t buf_dma_addr;
	uint32_t pkt_len;

	buf_dma_addr = rte_mbuf_data_iova(*pkts);
	pkt_len = (*pkts)->data_len;

	/* write data to descriptor */
	txdp->read.buffer_addr = rte_cpu_to_le_64(buf_dma_addr);
	txdp->read.cmd_type_len =
			rte_cpu_to_le_32((uint32_t)DCMD_DTYP_FLAGS | pkt_len);
	txdp->read.olinfo_status =
			rte_cpu_to_le_32(pkt_len << YUSUR2_ADVTXD_PAYLEN_SHIFT);
	rte_prefetch0(&(*pkts)->pool);
}

/*
 * Fill H/W descriptor ring with mbuf data.
 * Copy mbuf pointers to the S/W ring.
 */
static inline void
yusur2_tx_fill_hw_ring(struct yusur2_tx_queue *txq, struct rte_mbuf **pkts,
		      uint16_t nb_pkts)
{
	volatile union yusur2_adv_tx_desc *txdp = &(txq->tx_ring[txq->tx_tail]);
	struct yusur2_tx_entry *txep = &(txq->sw_ring[txq->tx_tail]);
	const int N_PER_LOOP = 4;
	const int N_PER_LOOP_MASK = N_PER_LOOP-1;
	int mainpart, leftover;
	int i, j;

	/*
	 * Process most of the packets in chunks of N pkts.  Any
	 * leftover packets will get processed one at a time.
	 */
	mainpart = (nb_pkts & ((uint32_t) ~N_PER_LOOP_MASK));
	leftover = (nb_pkts & ((uint32_t)  N_PER_LOOP_MASK));
	for (i = 0; i < mainpart; i += N_PER_LOOP) {
		/* Copy N mbuf pointers to the S/W ring */
		for (j = 0; j < N_PER_LOOP; ++j) {
			(txep + i + j)->mbuf = *(pkts + i + j);
		}
		tx4(txdp + i, pkts + i);
	}

	if (unlikely(leftover > 0)) {
		for (i = 0; i < leftover; ++i) {
			(txep + mainpart + i)->mbuf = *(pkts + mainpart + i);
			tx1(txdp + mainpart + i, pkts + mainpart + i);
		}
	}
}

static inline uint16_t
tx_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
	     uint16_t nb_pkts)
{
	struct yusur2_tx_queue *txq = (struct yusur2_tx_queue *)tx_queue;
	volatile union yusur2_adv_tx_desc *tx_r = txq->tx_ring;
	uint16_t n = 0;

	/*
	 * Begin scanning the H/W ring for done descriptors when the
	 * number of available descriptors drops below tx_free_thresh.  For
	 * each done descriptor, free the associated buffer.
	 */
	if (txq->nb_tx_free < txq->tx_free_thresh)
		yusur2_tx_free_bufs(txq);

	/* Only use descriptors that are available */
	nb_pkts = (uint16_t)RTE_MIN(txq->nb_tx_free, nb_pkts);
	if (unlikely(nb_pkts == 0))
		return 0;

	/* Use exactly nb_pkts descriptors */
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free - nb_pkts);

	/*
	 * At this point, we know there are enough descriptors in the
	 * ring to transmit all the packets.  This assumes that each
	 * mbuf contains a single segment, and that no new offloads
	 * are expected, which would require a new context descriptor.
	 */

	/*
	 * See if we're going to wrap-around. If so, handle the top
	 * of the descriptor ring first, then do the bottom.  If not,
	 * the processing looks just like the "bottom" part anyway...
	 */
	if ((txq->tx_tail + nb_pkts) > txq->nb_tx_desc) {
		n = (uint16_t)(txq->nb_tx_desc - txq->tx_tail);
		yusur2_tx_fill_hw_ring(txq, tx_pkts, n);

		/*
		 * We know that the last descriptor in the ring will need to
		 * have its RS bit set because tx_rs_thresh has to be
		 * a divisor of the ring size
		 */
		tx_r[txq->tx_next_rs].read.cmd_type_len |=
			rte_cpu_to_le_32(YUSUR2_ADVTXD_DCMD_RS);
		txq->tx_next_rs = (uint16_t)(txq->tx_rs_thresh - 1);

		txq->tx_tail = 0;
	}

	/* Fill H/W descriptor ring with mbuf data */
	yusur2_tx_fill_hw_ring(txq, tx_pkts + n, (uint16_t)(nb_pkts - n));
	txq->tx_tail = (uint16_t)(txq->tx_tail + (nb_pkts - n));

	/*
	 * Determine if RS bit should be set
	 * This is what we actually want:
	 *   if ((txq->tx_tail - 1) >= txq->tx_next_rs)
	 * but instead of subtracting 1 and doing >=, we can just do
	 * greater than without subtracting.
	 */
	if (txq->tx_tail > txq->tx_next_rs) {
		tx_r[txq->tx_next_rs].read.cmd_type_len |=
			rte_cpu_to_le_32(YUSUR2_ADVTXD_DCMD_RS);
		txq->tx_next_rs = (uint16_t)(txq->tx_next_rs +
						txq->tx_rs_thresh);
		if (txq->tx_next_rs >= txq->nb_tx_desc)
			txq->tx_next_rs = (uint16_t)(txq->tx_rs_thresh - 1);
	}

	/*
	 * Check for wrap-around. This would only happen if we used
	 * up to the last descriptor in the ring, no more, no less.
	 */
	if (txq->tx_tail >= txq->nb_tx_desc)
		txq->tx_tail = 0;

	/* update tail pointer */
	rte_wmb();
	YUSUR2_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, txq->tx_tail);

	return nb_pkts;
}

uint16_t
yusur2_xmit_pkts_simple(void *tx_queue, struct rte_mbuf **tx_pkts,
		       uint16_t nb_pkts)
{
	uint16_t nb_tx;

	/* Try to transmit at least chunks of TX_MAX_BURST pkts */
	if (likely(nb_pkts <= RTE_PMD_YUSUR2_TX_MAX_BURST))
		return tx_xmit_pkts(tx_queue, tx_pkts, nb_pkts);

	/* transmit more than the max burst, in chunks of TX_MAX_BURST */
	nb_tx = 0;
	while (nb_pkts) {
		uint16_t ret, n;

		n = (uint16_t)RTE_MIN(nb_pkts, RTE_PMD_YUSUR2_TX_MAX_BURST);
		ret = tx_xmit_pkts(tx_queue, &(tx_pkts[nb_tx]), n);
		nb_tx = (uint16_t)(nb_tx + ret);
		nb_pkts = (uint16_t)(nb_pkts - ret);
		if (ret < n)
			break;
	}

	return nb_tx;
}

#ifdef RTE_YUSUR2_INC_VECTOR
static uint16_t
yusur2_xmit_pkts_vec(void *tx_queue, struct rte_mbuf **tx_pkts,
		    uint16_t nb_pkts)
{
	uint16_t nb_tx = 0;
	struct yusur2_tx_queue *txq = (struct yusur2_tx_queue *)tx_queue;

	while (nb_pkts) {
		uint16_t ret, num;

		num = (uint16_t)RTE_MIN(nb_pkts, txq->tx_rs_thresh);
		ret = yusur2_xmit_fixed_burst_vec(tx_queue, &tx_pkts[nb_tx],
						 num);
		nb_tx += ret;
		nb_pkts -= ret;
		if (ret < num)
			break;
	}

	return nb_tx;
}
#endif

static inline void
yusur2_set_xmit_ctx(struct yusur2_tx_queue *txq,
		volatile struct yusur2_adv_tx_context_desc *ctx_txd,
		uint64_t ol_flags, union yusur2_tx_offload tx_offload,
		__rte_unused uint64_t *mdata)
{
	//TODO: check...
}

/*
 * Check which hardware context can be used. Use the existing match
 * or create a new context descriptor.
 */
static inline uint32_t
what_advctx_update(struct yusur2_tx_queue *txq, uint64_t flags,
		   union yusur2_tx_offload tx_offload)
{
	/* If match with the current used context */
	if (likely((txq->ctx_cache[txq->ctx_curr].flags == flags) &&
		   (txq->ctx_cache[txq->ctx_curr].tx_offload.data[0] ==
		    (txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data[0]
		     & tx_offload.data[0])) &&
		   (txq->ctx_cache[txq->ctx_curr].tx_offload.data[1] ==
		    (txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data[1]
		     & tx_offload.data[1]))))
		return txq->ctx_curr;

	/* What if match with the next context  */
	txq->ctx_curr ^= 1;
	if (likely((txq->ctx_cache[txq->ctx_curr].flags == flags) &&
		   (txq->ctx_cache[txq->ctx_curr].tx_offload.data[0] ==
		    (txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data[0]
		     & tx_offload.data[0])) &&
		   (txq->ctx_cache[txq->ctx_curr].tx_offload.data[1] ==
		    (txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data[1]
		     & tx_offload.data[1]))))
		return txq->ctx_curr;

	/* Mismatch, use the previous context */
	return YUSUR2_CTX_NUM;
}

static inline uint32_t
tx_desc_cksum_flags_to_olinfo(uint64_t ol_flags)
{
	uint32_t tmp = 0;

	if ((ol_flags & PKT_TX_L4_MASK) != PKT_TX_L4_NO_CKSUM)
		tmp |= YUSUR2_ADVTXD_POPTS_TXSM;
	if (ol_flags & PKT_TX_IP_CKSUM)
		tmp |= YUSUR2_ADVTXD_POPTS_IXSM;
	if (ol_flags & PKT_TX_TCP_SEG)
		tmp |= YUSUR2_ADVTXD_POPTS_TXSM;
	return tmp;
}

static inline uint32_t
tx_desc_ol_flags_to_cmdtype(uint64_t ol_flags)
{
	uint32_t cmdtype = 0;

	if (ol_flags & PKT_TX_VLAN_PKT)
		cmdtype |= YUSUR2_ADVTXD_DCMD_VLE;
	if (ol_flags & PKT_TX_TCP_SEG)
		cmdtype |= YUSUR2_ADVTXD_DCMD_TSE;
	if (ol_flags & PKT_TX_OUTER_IP_CKSUM)
		cmdtype |= (1 << YUSUR2_ADVTXD_OUTERIPCS_SHIFT);
	if (ol_flags & PKT_TX_MACSEC)
		cmdtype |= YUSUR2_ADVTXD_MAC_LINKSEC;
	return cmdtype;
}

/* Default RS bit threshold values */
#ifndef DEFAULT_TX_RS_THRESH
#define DEFAULT_TX_RS_THRESH   32
#endif
#ifndef DEFAULT_TX_FREE_THRESH
#define DEFAULT_TX_FREE_THRESH 32
#endif

/* Reset transmit descriptors after they have been used */
static inline int
yusur2_xmit_cleanup(struct yusur2_tx_queue *txq)
{
	struct yusur2_tx_entry *sw_ring = txq->sw_ring;
	volatile union yusur2_adv_tx_desc *txr = txq->tx_ring;
	uint16_t last_desc_cleaned = txq->last_desc_cleaned;
	uint16_t nb_tx_desc = txq->nb_tx_desc;
	uint16_t desc_to_clean_to;
	uint16_t nb_tx_to_clean;
	uint32_t status;

	/* Determine the last descriptor needing to be cleaned */
	desc_to_clean_to = (uint16_t)(last_desc_cleaned + txq->tx_rs_thresh);
	if (desc_to_clean_to >= nb_tx_desc)
		desc_to_clean_to = (uint16_t)(desc_to_clean_to - nb_tx_desc);

	/* Check to make sure the last descriptor to clean is done */
	desc_to_clean_to = sw_ring[desc_to_clean_to].last_id;
	status = txr[desc_to_clean_to].wb.status;
	if (!(status & rte_cpu_to_le_32(YUSUR2_TXD_STAT_DD))) {
		PMD_TX_FREE_LOG(DEBUG,
				"TX descriptor %4u is not done"
				"(port=%d queue=%d)",
				desc_to_clean_to,
				txq->port_id, txq->queue_id);
		/* Failed to clean any descriptors, better luck next time */
		return -(1);
	}

	/* Figure out how many descriptors will be cleaned */
	if (last_desc_cleaned > desc_to_clean_to)
		nb_tx_to_clean = (uint16_t)((nb_tx_desc - last_desc_cleaned) +
							desc_to_clean_to);
	else
		nb_tx_to_clean = (uint16_t)(desc_to_clean_to -
						last_desc_cleaned);

	PMD_TX_FREE_LOG(DEBUG,
			"Cleaning %4u TX descriptors: %4u to %4u "
			"(port=%d queue=%d)",
			nb_tx_to_clean, last_desc_cleaned, desc_to_clean_to,
			txq->port_id, txq->queue_id);

	/*
	 * The last descriptor to clean is done, so that means all the
	 * descriptors from the last descriptor that was cleaned
	 * up to the last descriptor with the RS bit set
	 * are done. Only reset the threshold descriptor.
	 */
	txr[desc_to_clean_to].wb.status = 0;

	/* Update the txq to reflect the last descriptor that was cleaned */
	txq->last_desc_cleaned = desc_to_clean_to;
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free + nb_tx_to_clean);

	/* No Error */
	return 0;
}

uint16_t
yusur2_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	struct yusur2_tx_queue *txq;
	struct yusur2_tx_entry *sw_ring;
	struct yusur2_tx_entry *txe, *txn;
	volatile union yusur2_adv_tx_desc *txr;
	volatile union yusur2_adv_tx_desc *txd, *txp;
	struct rte_mbuf     *tx_pkt;
	struct rte_mbuf     *m_seg;
	uint64_t buf_dma_addr;
	uint32_t olinfo_status;
	uint32_t cmd_type_len;
	uint32_t pkt_len;
	uint16_t slen;
	uint64_t ol_flags;
	uint16_t tx_id;
	uint16_t tx_last;
	uint16_t nb_tx;
	uint16_t nb_used;
	uint64_t tx_ol_req;
	uint32_t ctx = 0;
	uint32_t new_ctx;
	union yusur2_tx_offload tx_offload;
#ifdef RTE_LIBRTE_SECURITY
	uint8_t use_ipsec;
#endif

	tx_offload.data[0] = 0;
	tx_offload.data[1] = 0;
	txq = tx_queue;
	sw_ring = txq->sw_ring;
	txr     = txq->tx_ring;
	tx_id   = txq->tx_tail;
	txe = &sw_ring[tx_id];
	txp = NULL;

	/* Determine if the descriptor ring needs to be cleaned. */
	if (txq->nb_tx_free < txq->tx_free_thresh)
		yusur2_xmit_cleanup(txq);

	rte_prefetch0(&txe->mbuf->pool);

	/* TX loop */
	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		new_ctx = 0;
		tx_pkt = *tx_pkts++;
		pkt_len = tx_pkt->pkt_len;

		/*
		 * Determine how many (if any) context descriptors
		 * are needed for offload functionality.
		 */
		ol_flags = tx_pkt->ol_flags;
#ifdef RTE_LIBRTE_SECURITY
		use_ipsec = txq->using_ipsec && (ol_flags & PKT_TX_SEC_OFFLOAD);
#endif

		/* If hardware offload required */
		tx_ol_req = ol_flags & YUSUR2_TX_OFFLOAD_MASK;
		if (tx_ol_req) {
			tx_offload.l2_len = tx_pkt->l2_len;
			tx_offload.l3_len = tx_pkt->l3_len;
			tx_offload.l4_len = tx_pkt->l4_len;
			tx_offload.vlan_tci = tx_pkt->vlan_tci;
			tx_offload.tso_segsz = tx_pkt->tso_segsz;
			tx_offload.outer_l2_len = tx_pkt->outer_l2_len;
			tx_offload.outer_l3_len = tx_pkt->outer_l3_len;

			/* If new context need be built or reuse the exist ctx. */
			ctx = what_advctx_update(txq, tx_ol_req,
				tx_offload);
			/* Only allocate context descriptor if required*/
			new_ctx = (ctx == YUSUR2_CTX_NUM);
			ctx = txq->ctx_curr;
		}

		/*
		 * Keep track of how many descriptors are used this loop
		 * This will always be the number of segments + the number of
		 * Context descriptors required to transmit the packet
		 */
		nb_used = (uint16_t)(tx_pkt->nb_segs + new_ctx);

		if (txp != NULL &&
				nb_used + txq->nb_tx_used >= txq->tx_rs_thresh)
			/* set RS on the previous packet in the burst */
			txp->read.cmd_type_len |=
				rte_cpu_to_le_32(YUSUR2_TXD_CMD_RS);

		/*
		 * The number of descriptors that must be allocated for a
		 * packet is the number of segments of that packet, plus 1
		 * Context Descriptor for the hardware offload, if any.
		 * Determine the last TX descriptor to allocate in the TX ring
		 * for the packet, starting from the current position (tx_id)
		 * in the ring.
		 */
		tx_last = (uint16_t) (tx_id + nb_used - 1);

		/* Circular ring */
		if (tx_last >= txq->nb_tx_desc)
			tx_last = (uint16_t) (tx_last - txq->nb_tx_desc);

		PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u pktlen=%u"
			   " tx_first=%u tx_last=%u",
			   (unsigned) txq->port_id,
			   (unsigned) txq->queue_id,
			   (unsigned) pkt_len,
			   (unsigned) tx_id,
			   (unsigned) tx_last);

		/*
		 * Make sure there are enough TX descriptors available to
		 * transmit the entire packet.
		 * nb_used better be less than or equal to txq->tx_rs_thresh
		 */
		if (nb_used > txq->nb_tx_free) {
			PMD_TX_FREE_LOG(DEBUG,
					"Not enough free TX descriptors "
					"nb_used=%4u nb_free=%4u "
					"(port=%d queue=%d)",
					nb_used, txq->nb_tx_free,
					txq->port_id, txq->queue_id);

			if (yusur2_xmit_cleanup(txq) != 0) {
				/* Could not clean any descriptors */
				if (nb_tx == 0)
					return 0;
				goto end_of_tx;
			}

			/* nb_used better be <= txq->tx_rs_thresh */
			if (unlikely(nb_used > txq->tx_rs_thresh)) {
				PMD_TX_FREE_LOG(DEBUG,
					"The number of descriptors needed to "
					"transmit the packet exceeds the "
					"RS bit threshold. This will impact "
					"performance."
					"nb_used=%4u nb_free=%4u "
					"tx_rs_thresh=%4u. "
					"(port=%d queue=%d)",
					nb_used, txq->nb_tx_free,
					txq->tx_rs_thresh,
					txq->port_id, txq->queue_id);
				/*
				 * Loop here until there are enough TX
				 * descriptors or until the ring cannot be
				 * cleaned.
				 */
				while (nb_used > txq->nb_tx_free) {
					if (yusur2_xmit_cleanup(txq) != 0) {
						/*
						 * Could not clean any
						 * descriptors
						 */
						if (nb_tx == 0)
							return 0;
						goto end_of_tx;
					}
				}
			}
		}

		/*
		 * By now there are enough free TX descriptors to transmit
		 * the packet.
		 */

		/*
		 * Set common flags of all TX Data Descriptors.
		 *
		 * The following bits must be set in all Data Descriptors:
		 *   - YUSUR2_ADVTXD_DTYP_DATA
		 *   - YUSUR2_ADVTXD_DCMD_DEXT
		 *
		 * The following bits must be set in the first Data Descriptor
		 * and are ignored in the other ones:
		 *   - YUSUR2_ADVTXD_DCMD_IFCS
		 *   - YUSUR2_ADVTXD_MAC_1588
		 *   - YUSUR2_ADVTXD_DCMD_VLE
		 *
		 * The following bits must only be set in the last Data
		 * Descriptor:
		 *   - YUSUR2_TXD_CMD_EOP
		 *
		 * The following bits can be set in any Data Descriptor, but
		 * are only set in the last Data Descriptor:
		 *   - YUSUR2_TXD_CMD_RS
		 */
		cmd_type_len = YUSUR2_ADVTXD_DTYP_DATA |
			YUSUR2_ADVTXD_DCMD_IFCS | YUSUR2_ADVTXD_DCMD_DEXT;

#ifdef RTE_LIBRTE_IEEE1588
		if (ol_flags & PKT_TX_IEEE1588_TMST)
			cmd_type_len |= YUSUR2_ADVTXD_MAC_1588;
#endif

		olinfo_status = 0;
		if (tx_ol_req) {

			if (ol_flags & PKT_TX_TCP_SEG) {
				/* when TSO is on, paylen in descriptor is the
				 * not the packet len but the tcp payload len */
				pkt_len -= (tx_offload.l2_len +
					tx_offload.l3_len + tx_offload.l4_len);
			}

			/*
			 * Setup the TX Advanced Context Descriptor if required
			 */
			if (new_ctx) {
				volatile struct yusur2_adv_tx_context_desc *
				    ctx_txd;

				ctx_txd = (volatile struct
				    yusur2_adv_tx_context_desc *)
				    &txr[tx_id];

				txn = &sw_ring[txe->next_id];
				rte_prefetch0(&txn->mbuf->pool);

				if (txe->mbuf != NULL) {
					rte_pktmbuf_free_seg(txe->mbuf);
					txe->mbuf = NULL;
				}

				yusur2_set_xmit_ctx(txq, ctx_txd, tx_ol_req,
					tx_offload, &tx_pkt->udata64);

				txe->last_id = tx_last;
				tx_id = txe->next_id;
				txe = txn;
			}

			/*
			 * Setup the TX Advanced Data Descriptor,
			 * This path will go through
			 * whatever new/reuse the context descriptor
			 */
			cmd_type_len  |= tx_desc_ol_flags_to_cmdtype(ol_flags);
			olinfo_status |= tx_desc_cksum_flags_to_olinfo(ol_flags);
			olinfo_status |= ctx << YUSUR2_ADVTXD_IDX_SHIFT;
		}

		olinfo_status |= (pkt_len << YUSUR2_ADVTXD_PAYLEN_SHIFT);
#ifdef RTE_LIBRTE_SECURITY
		if (use_ipsec)
			olinfo_status |= YUSUR2_ADVTXD_POPTS_IPSEC;
#endif

		m_seg = tx_pkt;
		do {
			txd = &txr[tx_id];
			txn = &sw_ring[txe->next_id];
			rte_prefetch0(&txn->mbuf->pool);

			if (txe->mbuf != NULL)
				rte_pktmbuf_free_seg(txe->mbuf);
			txe->mbuf = m_seg;

			/*
			 * Set up Transmit Data Descriptor.
			 */
			slen = m_seg->data_len;
			buf_dma_addr = rte_mbuf_data_iova(m_seg);
			txd->read.buffer_addr =
				rte_cpu_to_le_64(buf_dma_addr);
			txd->read.cmd_type_len =
				rte_cpu_to_le_32(cmd_type_len | slen);
			txd->read.olinfo_status =
				rte_cpu_to_le_32(olinfo_status);
			txe->last_id = tx_last;
			tx_id = txe->next_id;
			txe = txn;
			m_seg = m_seg->next;
		} while (m_seg != NULL);

		/*
		 * The last packet data descriptor needs End Of Packet (EOP)
		 */
		cmd_type_len |= YUSUR2_TXD_CMD_EOP;
		txq->nb_tx_used = (uint16_t)(txq->nb_tx_used + nb_used);
		txq->nb_tx_free = (uint16_t)(txq->nb_tx_free - nb_used);

		/* Set RS bit only on threshold packets' last descriptor */
		if (txq->nb_tx_used >= txq->tx_rs_thresh) {
			PMD_TX_FREE_LOG(DEBUG,
					"Setting RS bit on TXD id="
					"%4u (port=%d queue=%d)",
					tx_last, txq->port_id, txq->queue_id);

			cmd_type_len |= YUSUR2_TXD_CMD_RS;

			/* Update txq RS bit counters */
			txq->nb_tx_used = 0;
			txp = NULL;
		} else
			txp = txd;

		txd->read.cmd_type_len |= rte_cpu_to_le_32(cmd_type_len);
	}

end_of_tx:
	/* set RS on last packet in the burst */
	if (txp != NULL)
		txp->read.cmd_type_len |= rte_cpu_to_le_32(YUSUR2_TXD_CMD_RS);

	rte_wmb();

	/*
	 * Set the Transmit Descriptor Tail (TDT)
	 */
	PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u tx_tail=%u nb_tx=%u",
		   (unsigned) txq->port_id, (unsigned) txq->queue_id,
		   (unsigned) tx_id, (unsigned) nb_tx);
	YUSUR2_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, tx_id);
	txq->tx_tail = tx_id;

	return nb_tx;
}

/*********************************************************************
 *
 *  TX prep functions
 *
 **********************************************************************/
uint16_t
yusur2_prep_pkts(void *tx_queue, struct rte_mbuf **tx_pkts, uint16_t nb_pkts)
{
	int i, ret;
	uint64_t ol_flags;
	struct rte_mbuf *m;
	struct yusur2_tx_queue *txq = (struct yusur2_tx_queue *)tx_queue;

	for (i = 0; i < nb_pkts; i++) {
		m = tx_pkts[i];
		ol_flags = m->ol_flags;

		/**
		 * Check if packet meets requirements for number of segments
		 *
		 * NOTE: for yusur2 it's always (40 - WTHRESH) for both TSO and
		 *       non-TSO
		 */

		if (m->nb_segs > YUSUR2_TX_MAX_SEG - txq->wthresh) {
			rte_errno = EINVAL;
			return i;
		}

		if (ol_flags & YUSUR2_TX_OFFLOAD_NOTSUP_MASK) {
			rte_errno = ENOTSUP;
			return i;
		}

		/* check the size of packet */
		if (m->pkt_len < YUSUR2_TX_MIN_PKT_LEN) {
			rte_errno = EINVAL;
			return i;
		}

#ifdef RTE_LIBRTE_ETHDEV_DEBUG
		ret = rte_validate_tx_offload(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
#endif
		ret = rte_net_intel_cksum_prepare(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
	}

	return i;
}

/*********************************************************************
 *
 *  RX functions
 *
 **********************************************************************/

#define YUSUR2_PACKET_TYPE_ETHER				0X00
#define YUSUR2_PACKET_TYPE_IPV4				0X01
#define YUSUR2_PACKET_TYPE_IPV4_TCP			0X11
#define YUSUR2_PACKET_TYPE_IPV4_UDP			0X21
#define YUSUR2_PACKET_TYPE_IPV4_SCTP			0X41
#define YUSUR2_PACKET_TYPE_IPV4_EXT			0X03
#define YUSUR2_PACKET_TYPE_IPV4_EXT_TCP			0X13
#define YUSUR2_PACKET_TYPE_IPV4_EXT_UDP			0X23
#define YUSUR2_PACKET_TYPE_IPV4_EXT_SCTP			0X43
#define YUSUR2_PACKET_TYPE_IPV6				0X04
#define YUSUR2_PACKET_TYPE_IPV6_TCP			0X14
#define YUSUR2_PACKET_TYPE_IPV6_UDP			0X24
#define YUSUR2_PACKET_TYPE_IPV6_SCTP			0X44
#define YUSUR2_PACKET_TYPE_IPV6_EXT			0X0C
#define YUSUR2_PACKET_TYPE_IPV6_EXT_TCP			0X1C
#define YUSUR2_PACKET_TYPE_IPV6_EXT_UDP			0X2C
#define YUSUR2_PACKET_TYPE_IPV6_EXT_SCTP			0X4C
#define YUSUR2_PACKET_TYPE_IPV4_IPV6			0X05
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_TCP			0X15
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_UDP			0X25
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_SCTP		0X45
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6			0X07
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_TCP		0X17
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_UDP		0X27
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_SCTP		0X47
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT			0X0D
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_TCP		0X1D
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_UDP		0X2D
#define YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_SCTP		0X4D
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT		0X0F
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_TCP		0X1F
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_UDP		0X2F
#define YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_SCTP	0X4F

#define YUSUR2_PACKET_TYPE_NVGRE                   0X00
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4              0X01
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_TCP          0X11
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_UDP          0X21
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_SCTP         0X41
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT          0X03
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_TCP      0X13
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_UDP      0X23
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_SCTP     0X43
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6              0X04
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_TCP          0X14
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_UDP          0X24
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_SCTP         0X44
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT          0X0C
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_TCP      0X1C
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_UDP      0X2C
#define YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_SCTP     0X4C
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6         0X05
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_TCP     0X15
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_UDP     0X25
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT     0X0D
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT_TCP 0X1D
#define YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT_UDP 0X2D

#define YUSUR2_PACKET_TYPE_VXLAN                   0X80
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4              0X81
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_TCP          0x91
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_UDP          0xA1
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_SCTP         0xC1
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT          0x83
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_TCP      0X93
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_UDP      0XA3
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_SCTP     0XC3
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6              0X84
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_TCP          0X94
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_UDP          0XA4
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_SCTP         0XC4
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT          0X8C
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_TCP      0X9C
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_UDP      0XAC
#define YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_SCTP     0XCC
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6         0X85
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_TCP     0X95
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_UDP     0XA5
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT     0X8D
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT_TCP 0X9D
#define YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT_UDP 0XAD

/**
 * Use 2 different table for normal packet and tunnel packet
 * to save the space.
 */
const uint32_t
	yusur2_ptype_table[YUSUR2_PACKET_TYPE_MAX] __rte_cache_aligned = {
	[YUSUR2_PACKET_TYPE_ETHER] = RTE_PTYPE_L2_ETHER,
	[YUSUR2_PACKET_TYPE_IPV4] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4,
	[YUSUR2_PACKET_TYPE_IPV4_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6,
	[YUSUR2_PACKET_TYPE_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV6_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6_EXT,
	[YUSUR2_PACKET_TYPE_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV6_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
	RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_IPV6_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_IPV4_EXT_IPV6_EXT_SCTP] =
		RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_TUNNEL_IP |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_SCTP,
};

const uint32_t
	yusur2_ptype_table_tn[YUSUR2_PACKET_TYPE_TN_MAX] __rte_cache_aligned = {
	[YUSUR2_PACKET_TYPE_NVGRE] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4_EXT,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6_EXT,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4 |
		RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6 |
		RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6_EXT |
		RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT_TCP] =
		RTE_PTYPE_L2_ETHER | RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_TUNNEL_GRE | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4 |
		RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6 |
		RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6 |
		RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6_EXT |
		RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV6_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV6_EXT |
		RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_IPV6_EXT_UDP] =
		RTE_PTYPE_L2_ETHER | RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_TUNNEL_GRE | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4 |
		RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4_EXT |
		RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4_EXT |
		RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_NVGRE_IPV4_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_TUNNEL_GRE |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4_EXT |
		RTE_PTYPE_INNER_L4_UDP,

	[YUSUR2_PACKET_TYPE_VXLAN] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4_EXT,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6_EXT,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4 | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT_TCP] =
		RTE_PTYPE_L2_ETHER | RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_L4_UDP | RTE_PTYPE_TUNNEL_VXLAN |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4 | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_UDP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV6_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_IPV6_EXT_UDP] =
		RTE_PTYPE_L2_ETHER | RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_L4_UDP | RTE_PTYPE_TUNNEL_VXLAN |
		RTE_PTYPE_INNER_L2_ETHER | RTE_PTYPE_INNER_L3_IPV4,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4 | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4_EXT | RTE_PTYPE_INNER_L4_SCTP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_TCP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4_EXT | RTE_PTYPE_INNER_L4_TCP,
	[YUSUR2_PACKET_TYPE_VXLAN_IPV4_EXT_UDP] = RTE_PTYPE_L2_ETHER |
		RTE_PTYPE_L3_IPV4_EXT_UNKNOWN | RTE_PTYPE_L4_UDP |
		RTE_PTYPE_TUNNEL_VXLAN | RTE_PTYPE_INNER_L2_ETHER |
		RTE_PTYPE_INNER_L3_IPV4_EXT | RTE_PTYPE_INNER_L4_UDP,
};

/* @note: fix yusur2_dev_supported_ptypes_get() if any change here. */
static inline uint32_t
yusur2_rxd_pkt_info_to_pkt_type(uint32_t pkt_info, uint16_t ptype_mask)
{

	if (unlikely(pkt_info & YUSUR2_RXDADV_PKTTYPE_ETQF))
		return RTE_PTYPE_UNKNOWN;

	pkt_info = (pkt_info >> YUSUR2_PACKET_TYPE_SHIFT) & ptype_mask;

	/* For tunnel packet */
	if (pkt_info & YUSUR2_PACKET_TYPE_TUNNEL_BIT) {
		/* Remove the tunnel bit to save the space. */
		pkt_info &= YUSUR2_PACKET_TYPE_MASK_TUNNEL;
		return yusur2_ptype_table_tn[pkt_info];
	}

	/**
	 * For x550, if it's not tunnel,
	 * tunnel type bit should be set to 0.
	 * Reuse 82599's mask.
	 */
	pkt_info &= YUSUR2_PACKET_TYPE_MASK_82599;

	return yusur2_ptype_table[pkt_info];
}

static inline uint64_t
yusur2_rxd_pkt_info_to_pkt_flags(uint16_t pkt_info)
{
	static uint64_t ip_rss_types_map[16] __rte_cache_aligned = {
		0, PKT_RX_RSS_HASH, PKT_RX_RSS_HASH, PKT_RX_RSS_HASH,
		0, PKT_RX_RSS_HASH, 0, PKT_RX_RSS_HASH,
		PKT_RX_RSS_HASH, 0, 0, 0,
		0, 0, 0,  PKT_RX_FDIR,
	};
#ifdef RTE_LIBRTE_IEEE1588
	static uint64_t ip_pkt_etqf_map[8] = {
		0, 0, 0, PKT_RX_IEEE1588_PTP,
		0, 0, 0, 0,
	};

	if (likely(pkt_info & YUSUR2_RXDADV_PKTTYPE_ETQF))
		return ip_pkt_etqf_map[(pkt_info >> 4) & 0X07] |
				ip_rss_types_map[pkt_info & 0XF];
	else
		return ip_rss_types_map[pkt_info & 0XF];
#else
	return ip_rss_types_map[pkt_info & 0XF];
#endif
}

static inline uint64_t
rx_desc_status_to_pkt_flags(uint32_t rx_status, uint64_t vlan_flags)
{
	uint64_t pkt_flags;

	/*
	 * Check if VLAN present only.
	 * Do not check whether L3/L4 rx checksum done by NIC or not,
	 * That can be found from rte_eth_rxmode.offloads flag
	 */
	pkt_flags = (rx_status & YUSUR2_RXD_STAT_VP) ?  vlan_flags : 0;

#ifdef RTE_LIBRTE_IEEE1588
	if (rx_status & YUSUR2_RXD_STAT_TMST)
		pkt_flags = pkt_flags | PKT_RX_IEEE1588_TMST;
#endif
	return pkt_flags;
}

static inline uint64_t
rx_desc_error_to_pkt_flags(uint32_t rx_status)
{
	uint64_t pkt_flags;

	/*
	 * Bit 31: IPE, IPv4 checksum error
	 * Bit 30: L4I, L4I integrity error
	 */
	static uint64_t error_to_pkt_flags_map[4] = {
		PKT_RX_IP_CKSUM_GOOD | PKT_RX_L4_CKSUM_GOOD,
		PKT_RX_IP_CKSUM_GOOD | PKT_RX_L4_CKSUM_BAD,
		PKT_RX_IP_CKSUM_BAD | PKT_RX_L4_CKSUM_GOOD,
		PKT_RX_IP_CKSUM_BAD | PKT_RX_L4_CKSUM_BAD
	};
	pkt_flags = error_to_pkt_flags_map[(rx_status >>
		YUSUR2_RXDADV_ERR_CKSUM_BIT) & YUSUR2_RXDADV_ERR_CKSUM_MSK];

	if ((rx_status & YUSUR2_RXD_STAT_OUTERIPCS) &&
	    (rx_status & YUSUR2_RXDADV_ERR_OUTERIPER)) {
		pkt_flags |= PKT_RX_EIP_CKSUM_BAD;
	}

#ifdef RTE_LIBRTE_SECURITY
	if (rx_status & YUSUR2_RXD_STAT_SECP) {
		pkt_flags |= PKT_RX_SEC_OFFLOAD;
		if (rx_status & YUSUR2_RXDADV_LNKSEC_ERROR_BAD_SIG)
			pkt_flags |= PKT_RX_SEC_OFFLOAD_FAILED;
	}
#endif

	return pkt_flags;
}

/*
 * LOOK_AHEAD defines how many desc statuses to check beyond the
 * current descriptor.
 * It must be a pound define for optimal performance.
 * Do not change the value of LOOK_AHEAD, as the yusur2_rx_scan_hw_ring
 * function only works with LOOK_AHEAD=8.
 */
#define LOOK_AHEAD 8
#if (LOOK_AHEAD != 8)
#error "PMD YUSUR2: LOOK_AHEAD must be 8\n"
#endif
static inline int
yusur2_rx_scan_hw_ring(struct yusur2_rx_queue *rxq)
{
	volatile union yusur2_adv_rx_desc *rxdp;
	struct yusur2_rx_entry *rxep;
	struct rte_mbuf *mb;
	uint16_t pkt_len;
	uint64_t pkt_flags;
	int nb_dd;
	uint32_t s[LOOK_AHEAD];
	uint32_t pkt_info[LOOK_AHEAD];
	int i, j, nb_rx = 0;
	uint32_t status;
	uint64_t vlan_flags = rxq->vlan_flags;

	/* get references to current descriptor and S/W ring entry */
	rxdp = &rxq->rx_ring[rxq->rx_tail];
	rxep = &rxq->sw_ring[rxq->rx_tail];

	status = rxdp->wb.upper.status_error;
	/* check to make sure there is at least 1 packet to receive */
	if (!(status & rte_cpu_to_le_32(YUSUR2_RXDADV_STAT_DD)))
		return 0;

	/*
	 * Scan LOOK_AHEAD descriptors at a time to determine which descriptors
	 * reference packets that are ready to be received.
	 */
	for (i = 0; i < RTE_PMD_YUSUR2_RX_MAX_BURST;
	     i += LOOK_AHEAD, rxdp += LOOK_AHEAD, rxep += LOOK_AHEAD) {
		/* Read desc statuses backwards to avoid race condition */
		for (j = 0; j < LOOK_AHEAD; j++)
			s[j] = rte_le_to_cpu_32(rxdp[j].wb.upper.status_error);

		rte_smp_rmb();

		/* Compute how many status bits were set */
		for (nb_dd = 0; nb_dd < LOOK_AHEAD &&
				(s[nb_dd] & YUSUR2_RXDADV_STAT_DD); nb_dd++)
			;

		for (j = 0; j < nb_dd; j++)
			pkt_info[j] = rte_le_to_cpu_32(rxdp[j].wb.lower.
						       lo_dword.data);

		nb_rx += nb_dd;

		/* Translate descriptor info to mbuf format */
		for (j = 0; j < nb_dd; ++j) {
			mb = rxep[j].mbuf;
			pkt_len = rte_le_to_cpu_16(rxdp[j].wb.upper.length) -
				  rxq->crc_len;
			mb->data_len = pkt_len;
			mb->pkt_len = pkt_len;
			mb->vlan_tci = rte_le_to_cpu_16(rxdp[j].wb.upper.vlan);

			/* convert descriptor fields to rte mbuf flags */
			pkt_flags = rx_desc_status_to_pkt_flags(s[j],
				vlan_flags);
			pkt_flags |= rx_desc_error_to_pkt_flags(s[j]);
			pkt_flags |= yusur2_rxd_pkt_info_to_pkt_flags
					((uint16_t)pkt_info[j]);
			mb->ol_flags = pkt_flags;
			mb->packet_type =
				yusur2_rxd_pkt_info_to_pkt_type
					(pkt_info[j], rxq->pkt_type_mask);

			if (likely(pkt_flags & PKT_RX_RSS_HASH))
				mb->hash.rss = rte_le_to_cpu_32(
				    rxdp[j].wb.lower.hi_dword.rss);
			else if (pkt_flags & PKT_RX_FDIR) {
				mb->hash.fdir.hash = rte_le_to_cpu_16(
				    rxdp[j].wb.lower.hi_dword.csum_ip.csum) &
				    YUSUR2_ATR_HASH_MASK;
				mb->hash.fdir.id = rte_le_to_cpu_16(
				    rxdp[j].wb.lower.hi_dword.csum_ip.ip_id);
			}
		}

		/* Move mbuf pointers from the S/W ring to the stage */
		for (j = 0; j < LOOK_AHEAD; ++j) {
			rxq->rx_stage[i + j] = rxep[j].mbuf;
		}

		/* stop if all requested packets could not be received */
		if (nb_dd != LOOK_AHEAD)
			break;
	}

	/* clear software ring entries so we can cleanup correctly */
	for (i = 0; i < nb_rx; ++i) {
		rxq->sw_ring[rxq->rx_tail + i].mbuf = NULL;
	}


	return nb_rx;
}

static inline int
yusur2_rx_alloc_bufs(struct yusur2_rx_queue *rxq, bool reset_mbuf)
{
	volatile union yusur2_adv_rx_desc *rxdp;
	struct yusur2_rx_entry *rxep;
	struct rte_mbuf *mb;
	uint16_t alloc_idx;
	__le64 dma_addr;
	int diag, i;

	/* allocate buffers in bulk directly into the S/W ring */
	alloc_idx = rxq->rx_free_trigger - (rxq->rx_free_thresh - 1);
	rxep = &rxq->sw_ring[alloc_idx];
	diag = rte_mempool_get_bulk(rxq->mb_pool, (void *)rxep,
				    rxq->rx_free_thresh);
	if (unlikely(diag != 0))
		return -ENOMEM;

	rxdp = &rxq->rx_ring[alloc_idx];
	for (i = 0; i < rxq->rx_free_thresh; ++i) {
		/* populate the static rte mbuf fields */
		mb = rxep[i].mbuf;
		if (reset_mbuf) {
			mb->port = rxq->port_id;
		}

		rte_mbuf_refcnt_set(mb, 1);
		mb->data_off = RTE_PKTMBUF_HEADROOM;

		/* populate the descriptors */
		dma_addr = rte_cpu_to_le_64(rte_mbuf_data_iova_default(mb));
		rxdp[i].read.hdr_addr = 0;
		rxdp[i].read.pkt_addr = dma_addr;
	}

	/* update state of internal queue structure */
	rxq->rx_free_trigger = rxq->rx_free_trigger + rxq->rx_free_thresh;
	if (rxq->rx_free_trigger >= rxq->nb_rx_desc)
		rxq->rx_free_trigger = rxq->rx_free_thresh - 1;

	/* no errors */
	return 0;
}

static inline uint16_t
yusur2_rx_fill_from_stage(struct yusur2_rx_queue *rxq, struct rte_mbuf **rx_pkts,
			 uint16_t nb_pkts)
{
	struct rte_mbuf **stage = &rxq->rx_stage[rxq->rx_next_avail];
	int i;

	/* how many packets are ready to return? */
	nb_pkts = (uint16_t)RTE_MIN(nb_pkts, rxq->rx_nb_avail);

	/* copy mbuf pointers to the application's packet list */
	for (i = 0; i < nb_pkts; ++i)
		rx_pkts[i] = stage[i];

	/* update internal queue state */
	rxq->rx_nb_avail = (uint16_t)(rxq->rx_nb_avail - nb_pkts);
	rxq->rx_next_avail = (uint16_t)(rxq->rx_next_avail + nb_pkts);

	return nb_pkts;
}

static inline uint16_t
rx_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
	     uint16_t nb_pkts)
{
	struct yusur2_rx_queue *rxq = (struct yusur2_rx_queue *)rx_queue;
	uint16_t nb_rx = 0;

	/* Any previously recv'd pkts will be returned from the Rx stage */
	if (rxq->rx_nb_avail)
		return yusur2_rx_fill_from_stage(rxq, rx_pkts, nb_pkts);

	/* Scan the H/W ring for packets to receive */
	nb_rx = (uint16_t)yusur2_rx_scan_hw_ring(rxq);

	/* update internal queue state */
	rxq->rx_next_avail = 0;
	rxq->rx_nb_avail = nb_rx;
	rxq->rx_tail = (uint16_t)(rxq->rx_tail + nb_rx);

	/* if required, allocate new buffers to replenish descriptors */
	if (rxq->rx_tail > rxq->rx_free_trigger) {
		uint16_t cur_free_trigger = rxq->rx_free_trigger;

		if (yusur2_rx_alloc_bufs(rxq, true) != 0) {
			int i, j;

			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);

			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed +=
				rxq->rx_free_thresh;

			/*
			 * Need to rewind any previous receives if we cannot
			 * allocate new buffers to replenish the old ones.
			 */
			rxq->rx_nb_avail = 0;
			rxq->rx_tail = (uint16_t)(rxq->rx_tail - nb_rx);
			for (i = 0, j = rxq->rx_tail; i < nb_rx; ++i, ++j)
				rxq->sw_ring[j].mbuf = rxq->rx_stage[i];

			return 0;
		}

		/* update tail pointer */
		rte_wmb();
		YUSUR2_PCI_REG_WRITE_RELAXED(rxq->rdt_reg_addr,
					    cur_free_trigger);
	}

	if (rxq->rx_tail >= rxq->nb_rx_desc)
		rxq->rx_tail = 0;

	/* received any packets this loop? */
	if (rxq->rx_nb_avail)
		return yusur2_rx_fill_from_stage(rxq, rx_pkts, nb_pkts);

	return 0;
}

/* split requests into chunks of size RTE_PMD_YUSUR2_RX_MAX_BURST */
uint16_t
yusur2_recv_pkts_bulk_alloc(void *rx_queue, struct rte_mbuf **rx_pkts,
			   uint16_t nb_pkts)
{
	uint16_t nb_rx;

	if (unlikely(nb_pkts == 0))
		return 0;

	if (likely(nb_pkts <= RTE_PMD_YUSUR2_RX_MAX_BURST))
		return rx_recv_pkts(rx_queue, rx_pkts, nb_pkts);

	/* request is relatively large, chunk it up */
	nb_rx = 0;
	while (nb_pkts) {
		uint16_t ret, n;

		n = (uint16_t)RTE_MIN(nb_pkts, RTE_PMD_YUSUR2_RX_MAX_BURST);
		ret = rx_recv_pkts(rx_queue, &rx_pkts[nb_rx], n);
		nb_rx = (uint16_t)(nb_rx + ret);
		nb_pkts = (uint16_t)(nb_pkts - ret);
		if (ret < n)
			break;
	}

	return nb_rx;
}

uint16_t
yusur2_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	struct yusur2_rx_queue *rxq;
	volatile union yusur2_adv_rx_desc *rx_ring;
	volatile union yusur2_adv_rx_desc *rxdp;
	struct yusur2_rx_entry *sw_ring;
	struct yusur2_rx_entry *rxe;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	union yusur2_adv_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr;
	uint32_t pkt_info;
	uint16_t pkt_len;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint64_t pkt_flags;
	uint64_t vlan_flags;

	nb_rx = 0;
	nb_hold = 0;
	rxq = rx_queue;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;
	vlan_flags = rxq->vlan_flags;
	while (nb_rx < nb_pkts) {
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		staterr = rxdp->wb.upper.status_error;
		if (!(staterr & rte_cpu_to_le_32(YUSUR2_RXDADV_STAT_DD)))
			break;
		rxd = *rxdp;

		/*
		 * End of packet.
		 *
		 * If the YUSUR2_RXDADV_STAT_EOP flag is not set, the RX packet
		 * is likely to be invalid and to be dropped by the various
		 * validation checks performed by the network stack.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy do not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
			   "ext_err_stat=0x%08x pkt_len=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			   (unsigned) rte_le_to_cpu_16(rxd.wb.upper.length));

		nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_yusur2_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_yusur2_prefetch(&rx_ring[rx_id]);
			rte_yusur2_prefetch(&sw_ring[rx_id]);
		}

		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
		rxdp->read.hdr_addr = 0;
		rxdp->read.pkt_addr = dma_addr;

		/*
		 * Initialize the returned mbuf.
		 * 1) setup generic mbuf fields:
		 *    - number of segments,
		 *    - next segment,
		 *    - packet length,
		 *    - RX port identifier.
		 * 2) integrate hardware offload data, if any:
		 *    - RSS flag & hash,
		 *    - IP checksum flag,
		 *    - VLAN TCI, if any,
		 *    - error flags.
		 */
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) -
				      rxq->crc_len);
		rxm->data_off = RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch((char *)rxm->buf_addr + rxm->data_off);
		rxm->nb_segs = 1;
		rxm->next = NULL;
		rxm->pkt_len = pkt_len;
		rxm->data_len = pkt_len;
		rxm->port = rxq->port_id;

		pkt_info = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);
		/* Only valid if PKT_RX_VLAN set in pkt_flags */
		rxm->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);

		pkt_flags = rx_desc_status_to_pkt_flags(staterr, vlan_flags);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		pkt_flags = pkt_flags |
			yusur2_rxd_pkt_info_to_pkt_flags((uint16_t)pkt_info);
		rxm->ol_flags = pkt_flags;
		rxm->packet_type =
			yusur2_rxd_pkt_info_to_pkt_type(pkt_info,
						       rxq->pkt_type_mask);

		if (likely(pkt_flags & PKT_RX_RSS_HASH))
			rxm->hash.rss = rte_le_to_cpu_32(
						rxd.wb.lower.hi_dword.rss);
		else if (pkt_flags & PKT_RX_FDIR) {
			rxm->hash.fdir.hash = rte_le_to_cpu_16(
					rxd.wb.lower.hi_dword.csum_ip.csum) &
					YUSUR2_ATR_HASH_MASK;
			rxm->hash.fdir.id = rte_le_to_cpu_16(
					rxd.wb.lower.hi_dword.csum_ip.ip_id);
		}
		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = rxm;
	}
	rxq->rx_tail = rx_id;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situtation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
				     (rxq->nb_rx_desc - 1) : (rx_id - 1));
		YUSUR2_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;
	return nb_rx;
}

/**
 * Detect an RSC descriptor.
 */
static inline uint32_t
yusur2_rsc_count(union yusur2_adv_rx_desc *rx)
{
	return (rte_le_to_cpu_32(rx->wb.lower.lo_dword.data) &
		YUSUR2_RXDADV_RSCCNT_MASK) >> YUSUR2_RXDADV_RSCCNT_SHIFT;
}

/**
 * yusur2_fill_cluster_head_buf - fill the first mbuf of the returned packet
 *
 * Fill the following info in the HEAD buffer of the Rx cluster:
 *    - RX port identifier
 *    - hardware offload data, if any:
 *      - RSS flag & hash
 *      - IP checksum flag
 *      - VLAN TCI, if any
 *      - error flags
 * @head HEAD of the packet cluster
 * @desc HW descriptor to get data from
 * @rxq Pointer to the Rx queue
 */
static inline void
yusur2_fill_cluster_head_buf(
	struct rte_mbuf *head,
	union yusur2_adv_rx_desc *desc,
	struct yusur2_rx_queue *rxq,
	uint32_t staterr)
{
	uint32_t pkt_info;
	uint64_t pkt_flags;

	head->port = rxq->port_id;

	/* The vlan_tci field is only valid when PKT_RX_VLAN is
	 * set in the pkt_flags field.
	 */
	head->vlan_tci = rte_le_to_cpu_16(desc->wb.upper.vlan);
	pkt_info = rte_le_to_cpu_32(desc->wb.lower.lo_dword.data);
	pkt_flags = rx_desc_status_to_pkt_flags(staterr, rxq->vlan_flags);
	pkt_flags |= rx_desc_error_to_pkt_flags(staterr);
	pkt_flags |= yusur2_rxd_pkt_info_to_pkt_flags((uint16_t)pkt_info);
	head->ol_flags = pkt_flags;
	head->packet_type =
		yusur2_rxd_pkt_info_to_pkt_type(pkt_info, rxq->pkt_type_mask);

	if (likely(pkt_flags & PKT_RX_RSS_HASH))
		head->hash.rss = rte_le_to_cpu_32(desc->wb.lower.hi_dword.rss);
	else if (pkt_flags & PKT_RX_FDIR) {
		head->hash.fdir.hash =
			rte_le_to_cpu_16(desc->wb.lower.hi_dword.csum_ip.csum)
							  & YUSUR2_ATR_HASH_MASK;
		head->hash.fdir.id =
			rte_le_to_cpu_16(desc->wb.lower.hi_dword.csum_ip.ip_id);
	}
}

/**
 * yusur2_recv_pkts_lro - receive handler for and LRO case.
 *
 * @rx_queue Rx queue handle
 * @rx_pkts table of received packets
 * @nb_pkts size of rx_pkts table
 * @bulk_alloc if TRUE bulk allocation is used for a HW ring refilling
 *
 * Handles the Rx HW ring completions when RSC feature is configured. Uses an
 * additional ring of yusur2_rsc_entry's that will hold the relevant RSC info.
 *
 * We use the same logic as in Linux and in FreeBSD yusur2 drivers:
 * 1) When non-EOP RSC completion arrives:
 *    a) Update the HEAD of the current RSC aggregation cluster with the new
 *       segment's data length.
 *    b) Set the "next" pointer of the current segment to point to the segment
 *       at the NEXTP index.
 *    c) Pass the HEAD of RSC aggregation cluster on to the next NEXTP entry
 *       in the sw_rsc_ring.
 * 2) When EOP arrives we just update the cluster's total length and offload
 *    flags and deliver the cluster up to the upper layers. In our case - put it
 *    in the rx_pkts table.
 *
 * Returns the number of received packets/clusters (according to the "bulk
 * receive" interface).
 */
static inline uint16_t
yusur2_recv_pkts_lro(void *rx_queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts,
		    bool bulk_alloc)
{
	struct yusur2_rx_queue *rxq = rx_queue;
	volatile union yusur2_adv_rx_desc *rx_ring = rxq->rx_ring;
	struct yusur2_rx_entry *sw_ring = rxq->sw_ring;
	struct yusur2_scattered_rx_entry *sw_sc_ring = rxq->sw_sc_ring;
	uint16_t rx_id = rxq->rx_tail;
	uint16_t nb_rx = 0;
	uint16_t nb_hold = rxq->nb_rx_hold;
	uint16_t prev_id = rxq->rx_tail;

	while (nb_rx < nb_pkts) {
		bool eop;
		struct yusur2_rx_entry *rxe;
		struct yusur2_scattered_rx_entry *sc_entry;
		struct yusur2_scattered_rx_entry *next_sc_entry;
		struct yusur2_rx_entry *next_rxe = NULL;
		struct rte_mbuf *first_seg;
		struct rte_mbuf *rxm;
		struct rte_mbuf *nmb = NULL;
		union yusur2_adv_rx_desc rxd;
		uint16_t data_len;
		uint16_t next_id;
		volatile union yusur2_adv_rx_desc *rxdp;
		uint32_t staterr;

next_desc:
		/*
		 * The code in this whole file uses the volatile pointer to
		 * ensure the read ordering of the status and the rest of the
		 * descriptor fields (on the compiler level only!!!). This is so
		 * UGLY - why not to just use the compiler barrier instead? DPDK
		 * even has the rte_compiler_barrier() for that.
		 *
		 * But most importantly this is just wrong because this doesn't
		 * ensure memory ordering in a general case at all. For
		 * instance, DPDK is supposed to work on Power CPUs where
		 * compiler barrier may just not be enough!
		 *
		 * I tried to write only this function properly to have a
		 * starting point (as a part of an LRO/RSC series) but the
		 * compiler cursed at me when I tried to cast away the
		 * "volatile" from rx_ring (yes, it's volatile too!!!). So, I'm
		 * keeping it the way it is for now.
		 *
		 * The code in this file is broken in so many other places and
		 * will just not work on a big endian CPU anyway therefore the
		 * lines below will have to be revisited together with the rest
		 * of the yusur2 PMD.
		 *
		 * TODO:
		 *    - Get rid of "volatile" and let the compiler do its job.
		 *    - Use the proper memory barrier (rte_rmb()) to ensure the
		 *      memory ordering below.
		 */
		rxdp = &rx_ring[rx_id];
		staterr = rte_le_to_cpu_32(rxdp->wb.upper.status_error);

		if (!(staterr & YUSUR2_RXDADV_STAT_DD))
			break;

		rxd = *rxdp;

		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
				  "staterr=0x%x data_len=%u",
			   rxq->port_id, rxq->queue_id, rx_id, staterr,
			   rte_le_to_cpu_16(rxd.wb.upper.length));

		if (!bulk_alloc) {
			nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
			if (nmb == NULL) {
				PMD_RX_LOG(DEBUG, "RX mbuf alloc failed "
						  "port_id=%u queue_id=%u",
					   rxq->port_id, rxq->queue_id);

				rte_eth_devices[rxq->port_id].data->
							rx_mbuf_alloc_failed++;
				break;
			}
		} else if (nb_hold > rxq->rx_free_thresh) {
			uint16_t next_rdt = rxq->rx_free_trigger;

			if (!yusur2_rx_alloc_bufs(rxq, false)) {
				rte_wmb();
				YUSUR2_PCI_REG_WRITE_RELAXED(rxq->rdt_reg_addr,
							    next_rdt);
				nb_hold -= rxq->rx_free_thresh;
			} else {
				PMD_RX_LOG(DEBUG, "RX bulk alloc failed "
						  "port_id=%u queue_id=%u",
					   rxq->port_id, rxq->queue_id);

				rte_eth_devices[rxq->port_id].data->
							rx_mbuf_alloc_failed++;
				break;
			}
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		eop = staterr & YUSUR2_RXDADV_STAT_EOP;

		next_id = rx_id + 1;
		if (next_id == rxq->nb_rx_desc)
			next_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_yusur2_prefetch(sw_ring[next_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 4 pointers
		 * to mbufs.
		 */
		if ((next_id & 0x3) == 0) {
			rte_yusur2_prefetch(&rx_ring[next_id]);
			rte_yusur2_prefetch(&sw_ring[next_id]);
		}

		rxm = rxe->mbuf;

		if (!bulk_alloc) {
			__le64 dma =
			  rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
			/*
			 * Update RX descriptor with the physical address of the
			 * new data buffer of the new allocated mbuf.
			 */
			rxe->mbuf = nmb;

			rxm->data_off = RTE_PKTMBUF_HEADROOM;
			rxdp->read.hdr_addr = 0;
			rxdp->read.pkt_addr = dma;
		} else
			rxe->mbuf = NULL;

		/*
		 * Set data length & data buffer address of mbuf.
		 */
		data_len = rte_le_to_cpu_16(rxd.wb.upper.length);
		rxm->data_len = data_len;

		if (!eop) {
			uint16_t nextp_id;
			/*
			 * Get next descriptor index:
			 *  - For RSC it's in the NEXTP field.
			 *  - For a scattered packet - it's just a following
			 *    descriptor.
			 */
			if (yusur2_rsc_count(&rxd))
				nextp_id =
					(staterr & YUSUR2_RXDADV_NEXTP_MASK) >>
						       YUSUR2_RXDADV_NEXTP_SHIFT;
			else
				nextp_id = next_id;

			next_sc_entry = &sw_sc_ring[nextp_id];
			next_rxe = &sw_ring[nextp_id];
			rte_yusur2_prefetch(next_rxe);
		}

		sc_entry = &sw_sc_ring[rx_id];
		first_seg = sc_entry->fbuf;
		sc_entry->fbuf = NULL;

		/*
		 * If this is the first buffer of the received packet,
		 * set the pointer to the first mbuf of the packet and
		 * initialize its context.
		 * Otherwise, update the total length and the number of segments
		 * of the current scattered packet, and update the pointer to
		 * the last mbuf of the current packet.
		 */
		if (first_seg == NULL) {
			first_seg = rxm;
			first_seg->pkt_len = data_len;
			first_seg->nb_segs = 1;
		} else {
			first_seg->pkt_len += data_len;
			first_seg->nb_segs++;
		}

		prev_id = rx_id;
		rx_id = next_id;

		/*
		 * If this is not the last buffer of the received packet, update
		 * the pointer to the first mbuf at the NEXTP entry in the
		 * sw_sc_ring and continue to parse the RX ring.
		 */
		if (!eop && next_rxe) {
			rxm->next = next_rxe->mbuf;
			next_sc_entry->fbuf = first_seg;
			goto next_desc;
		}

		/* Initialize the first mbuf of the returned packet */
		yusur2_fill_cluster_head_buf(first_seg, &rxd, rxq, staterr);

		/*
		 * Deal with the case, when HW CRC srip is disabled.
		 * That can't happen when LRO is enabled, but still could
		 * happen for scattered RX mode.
		 */
		first_seg->pkt_len -= rxq->crc_len;
		if (unlikely(rxm->data_len <= rxq->crc_len)) {
			struct rte_mbuf *lp;

			for (lp = first_seg; lp->next != rxm; lp = lp->next)
				;

			first_seg->nb_segs--;
			lp->data_len -= rxq->crc_len - rxm->data_len;
			lp->next = NULL;
			rte_pktmbuf_free_seg(rxm);
		} else
			rxm->data_len -= rxq->crc_len;

		/* Prefetch data of first segment, if configured to do so. */
		rte_packet_prefetch((char *)first_seg->buf_addr +
			first_seg->data_off);

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = first_seg;
	}

	/*
	 * Record index of the next RX descriptor to probe.
	 */
	rxq->rx_tail = rx_id;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situtation from the
	 * hardware point of view...
	 */
	if (!bulk_alloc && nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   rxq->port_id, rxq->queue_id, rx_id, nb_hold, nb_rx);

		rte_wmb();
		YUSUR2_PCI_REG_WRITE_RELAXED(rxq->rdt_reg_addr, prev_id);
		nb_hold = 0;
	}

	rxq->nb_rx_hold = nb_hold;
	return nb_rx;
}

uint16_t
yusur2_recv_pkts_lro_single_alloc(void *rx_queue, struct rte_mbuf **rx_pkts,
				 uint16_t nb_pkts)
{
	return yusur2_recv_pkts_lro(rx_queue, rx_pkts, nb_pkts, false);
}

uint16_t
yusur2_recv_pkts_lro_bulk_alloc(void *rx_queue, struct rte_mbuf **rx_pkts,
			       uint16_t nb_pkts)
{
	return yusur2_recv_pkts_lro(rx_queue, rx_pkts, nb_pkts, true);
}

/*********************************************************************
 *
 *  Queue management functions
 *
 **********************************************************************/

static void __attribute__((cold))
yusur2_tx_queue_release_mbufs(struct yusur2_tx_queue *txq)
{
	unsigned i;

	if (txq->sw_ring != NULL) {
		for (i = 0; i < txq->nb_tx_desc; i++) {
			if (txq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(txq->sw_ring[i].mbuf);
				txq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void __attribute__((cold))
yusur2_tx_free_swring(struct yusur2_tx_queue *txq)
{
	if (txq != NULL &&
	    txq->sw_ring != NULL)
		rte_free(txq->sw_ring);
}

static void __attribute__((cold))
yusur2_tx_queue_release(struct yusur2_tx_queue *txq)
{
	if (txq != NULL && txq->ops != NULL) {
		txq->ops->release_mbufs(txq);
		txq->ops->free_swring(txq);
		rte_free(txq);
	}
}

void __attribute__((cold))
yusur2_dev_tx_queue_release(void *txq)
{
	yusur2_tx_queue_release(txq);
}

/* (Re)set dynamic yusur2_tx_queue fields to defaults */
static void __attribute__((cold))
yusur2_reset_tx_queue(struct yusur2_tx_queue *txq)
{
	static const union yusur2_adv_tx_desc zeroed_desc = {{0}};
	struct yusur2_tx_entry *txe = txq->sw_ring;
	uint16_t prev, i;

	/* Zero out HW ring memory */
	for (i = 0; i < txq->nb_tx_desc; i++) {
		txq->tx_ring[i] = zeroed_desc;
	}

	/* Initialize SW ring entries */
	prev = (uint16_t) (txq->nb_tx_desc - 1);
	for (i = 0; i < txq->nb_tx_desc; i++) {
		volatile union yusur2_adv_tx_desc *txd = &txq->tx_ring[i];

		txd->wb.status = rte_cpu_to_le_32(YUSUR2_TXD_STAT_DD);
		txe[i].mbuf = NULL;
		txe[i].last_id = i;
		txe[prev].next_id = i;
		prev = i;
	}

	txq->tx_next_dd = (uint16_t)(txq->tx_rs_thresh - 1);
	txq->tx_next_rs = (uint16_t)(txq->tx_rs_thresh - 1);

	txq->tx_tail = 0;
	txq->nb_tx_used = 0;
	/*
	 * Always allow 1 descriptor to be un-allocated to avoid
	 * a H/W race condition
	 */
	txq->last_desc_cleaned = (uint16_t)(txq->nb_tx_desc - 1);
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_desc - 1);
	txq->ctx_curr = 0;
	memset((void *)&txq->ctx_cache, 0,
		YUSUR2_CTX_NUM * sizeof(struct yusur2_advctx_info));
}

static const struct yusur2_txq_ops def_txq_ops = {
	.release_mbufs = yusur2_tx_queue_release_mbufs,
	.free_swring = yusur2_tx_free_swring,
	.reset = yusur2_reset_tx_queue,
};

/* Takes an ethdev and a queue and sets up the tx function to be used based on
 * the queue parameters. Used in tx_queue_setup by primary process and then
 * in dev_init by secondary process when attaching to an existing ethdev.
 */
void __attribute__((cold))
yusur2_set_tx_function(struct rte_eth_dev *dev, struct yusur2_tx_queue *txq)
{
	/* Use a simple Tx queue (no offloads, no multi segs) if possible */
	if ((txq->offloads == 0) &&
#ifdef RTE_LIBRTE_SECURITY
			!(txq->using_ipsec) &&
#endif
			(txq->tx_rs_thresh >= RTE_PMD_YUSUR2_TX_MAX_BURST)) {
		PMD_INIT_LOG(DEBUG, "Using simple tx code path");
		dev->tx_pkt_prepare = NULL;
#ifdef RTE_YUSUR2_INC_VECTOR
		if (txq->tx_rs_thresh <= RTE_YUSUR2_TX_MAX_FREE_BUF_SZ &&
				(rte_eal_process_type() != RTE_PROC_PRIMARY ||
					yusur2_txq_vec_setup(txq) == 0)) {
			PMD_INIT_LOG(DEBUG, "Vector tx enabled.");
			dev->tx_pkt_burst = yusur2_xmit_pkts_vec;
		} else
#endif
		dev->tx_pkt_burst = yusur2_xmit_pkts_simple;
	} else {
		PMD_INIT_LOG(DEBUG, "Using full-featured tx code path");
		PMD_INIT_LOG(DEBUG,
				" - offloads = 0x%" PRIx64,
				txq->offloads);
		PMD_INIT_LOG(DEBUG,
				" - tx_rs_thresh = %lu " "[RTE_PMD_YUSUR2_TX_MAX_BURST=%lu]",
				(unsigned long)txq->tx_rs_thresh,
				(unsigned long)RTE_PMD_YUSUR2_TX_MAX_BURST);
		dev->tx_pkt_burst = yusur2_xmit_pkts;
		dev->tx_pkt_prepare = yusur2_prep_pkts;
	}
}

uint64_t
yusur2_get_tx_queue_offloads(struct rte_eth_dev *dev)
{
	RTE_SET_USED(dev);

	return 0;
}

uint64_t
yusur2_get_tx_port_offloads(struct rte_eth_dev *dev)
{
	uint64_t tx_offload_capa;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	//TODO: yusur2 offload support...haha
	tx_offload_capa =
		DEV_TX_OFFLOAD_VLAN_INSERT |
		DEV_TX_OFFLOAD_IPV4_CKSUM  |
		DEV_TX_OFFLOAD_UDP_CKSUM   |
		DEV_TX_OFFLOAD_TCP_CKSUM   |
		DEV_TX_OFFLOAD_SCTP_CKSUM  |
		DEV_TX_OFFLOAD_TCP_TSO     |
		DEV_TX_OFFLOAD_MULTI_SEGS;

	return tx_offload_capa;
}

int __attribute__((cold))
yusur2_dev_tx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_txconf *tx_conf)
{
	const struct rte_memzone *tz;
	struct yusur2_tx_queue *txq;
	struct yusur2_hw     *hw;
	uint16_t tx_rs_thresh, tx_free_thresh;
	uint64_t offloads;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	offloads = tx_conf->offloads | dev->data->dev_conf.txmode.offloads;

	/*
	 * Validate number of transmit descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of YUSUR2_ALIGN.
	 */
	if (nb_desc % YUSUR2_TXD_ALIGN != 0 ||
			(nb_desc > YUSUR2_MAX_RING_DESC) ||
			(nb_desc < YUSUR2_MIN_RING_DESC)) {
		return -EINVAL;
	}

	/*
	 * The following two parameters control the setting of the RS bit on
	 * transmit descriptors.
	 * TX descriptors will have their RS bit set after txq->tx_rs_thresh
	 * descriptors have been used.
	 * The TX descriptor ring will be cleaned after txq->tx_free_thresh
	 * descriptors are used or if the number of descriptors required
	 * to transmit a packet is greater than the number of free TX
	 * descriptors.
	 * The following constraints must be satisfied:
	 *  tx_rs_thresh must be greater than 0.
	 *  tx_rs_thresh must be less than the size of the ring minus 2.
	 *  tx_rs_thresh must be less than or equal to tx_free_thresh.
	 *  tx_rs_thresh must be a divisor of the ring size.
	 *  tx_free_thresh must be greater than 0.
	 *  tx_free_thresh must be less than the size of the ring minus 3.
	 *  tx_free_thresh + tx_rs_thresh must not exceed nb_desc.
	 * One descriptor in the TX ring is used as a sentinel to avoid a
	 * H/W race condition, hence the maximum threshold constraints.
	 * When set to zero use default values.
	 */
	tx_free_thresh = (uint16_t)((tx_conf->tx_free_thresh) ?
			tx_conf->tx_free_thresh : DEFAULT_TX_FREE_THRESH);
	/* force tx_rs_thresh to adapt an aggresive tx_free_thresh */
	tx_rs_thresh = (DEFAULT_TX_RS_THRESH + tx_free_thresh > nb_desc) ?
			nb_desc - tx_free_thresh : DEFAULT_TX_RS_THRESH;
	if (tx_conf->tx_rs_thresh > 0)
		tx_rs_thresh = tx_conf->tx_rs_thresh;
	if (tx_rs_thresh + tx_free_thresh > nb_desc) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh + tx_free_thresh must not "
			     "exceed nb_desc. (tx_rs_thresh=%u "
			     "tx_free_thresh=%u nb_desc=%u port = %d queue=%d)",
			     (unsigned int)tx_rs_thresh,
			     (unsigned int)tx_free_thresh,
			     (unsigned int)nb_desc,
			     (int)dev->data->port_id,
			     (int)queue_idx);
		return -(EINVAL);
	}
	if (tx_rs_thresh >= (nb_desc - 2)) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be less than the number "
			"of TX descriptors minus 2. (tx_rs_thresh=%u "
			"port=%d queue=%d)", (unsigned int)tx_rs_thresh,
			(int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}
	if (tx_rs_thresh > DEFAULT_TX_RS_THRESH) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be less or equal than %u. "
			"(tx_rs_thresh=%u port=%d queue=%d)",
			DEFAULT_TX_RS_THRESH, (unsigned int)tx_rs_thresh,
			(int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}
	if (tx_free_thresh >= (nb_desc - 3)) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be less than the "
			     "tx_free_thresh must be less than the number of "
			     "TX descriptors minus 3. (tx_free_thresh=%u "
			     "port=%d queue=%d)",
			     (unsigned int)tx_free_thresh,
			     (int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}
	if (tx_rs_thresh > tx_free_thresh) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be less than or equal to "
			     "tx_free_thresh. (tx_free_thresh=%u "
			     "tx_rs_thresh=%u port=%d queue=%d)",
			     (unsigned int)tx_free_thresh,
			     (unsigned int)tx_rs_thresh,
			     (int)dev->data->port_id,
			     (int)queue_idx);
		return -(EINVAL);
	}
	if ((nb_desc % tx_rs_thresh) != 0) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be a divisor of the "
			     "number of TX descriptors. (tx_rs_thresh=%u "
			     "port=%d queue=%d)", (unsigned int)tx_rs_thresh,
			     (int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}

	/*
	 * If rs_bit_thresh is greater than 1, then TX WTHRESH should be
	 * set to 0. If WTHRESH is greater than zero, the RS bit is ignored
	 * by the NIC and all descriptors are written back after the NIC
	 * accumulates WTHRESH descriptors.
	 */
	if ((tx_rs_thresh > 1) && (tx_conf->tx_thresh.wthresh != 0)) {
		PMD_INIT_LOG(ERR, "TX WTHRESH must be set to 0 if "
			     "tx_rs_thresh is greater than 1. (tx_rs_thresh=%u "
			     "port=%d queue=%d)", (unsigned int)tx_rs_thresh,
			     (int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}

	/* Free memory prior to re-allocation if needed... */
	if (dev->data->tx_queues[queue_idx] != NULL) {
		yusur2_tx_queue_release(dev->data->tx_queues[queue_idx]);
		dev->data->tx_queues[queue_idx] = NULL;
	}

	/* First allocate the tx queue data structure */
	txq = rte_zmalloc_socket("ethdev TX queue", sizeof(struct yusur2_tx_queue),
				 RTE_CACHE_LINE_SIZE, socket_id);
	if (txq == NULL)
		return -ENOMEM;

	/*
	 * Allocate TX ring hardware descriptors. A memzone large enough to
	 * handle the maximum ring size is allocated in order to allow for
	 * resizing in later calls to the queue setup function.
	 */
	tz = rte_eth_dma_zone_reserve(dev, "tx_ring", queue_idx,
			sizeof(union yusur2_adv_tx_desc) * YUSUR2_MAX_RING_DESC,
			YUSUR2_ALIGN, socket_id);
	if (tz == NULL) {
		yusur2_tx_queue_release(txq);
		return -ENOMEM;
	}

	txq->nb_tx_desc = nb_desc;
	txq->tx_rs_thresh = tx_rs_thresh;
	txq->tx_free_thresh = tx_free_thresh;
	txq->pthresh = tx_conf->tx_thresh.pthresh;
	txq->hthresh = tx_conf->tx_thresh.hthresh;
	txq->wthresh = tx_conf->tx_thresh.wthresh;
	txq->queue_id = queue_idx;
	txq->reg_idx = (uint16_t)((RTE_ETH_DEV_SRIOV(dev).active == 0) ?
		queue_idx : RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx + queue_idx);
	txq->port_id = dev->data->port_id;
	txq->offloads = offloads;
	txq->ops = &def_txq_ops;
	txq->tx_deferred_start = tx_conf->tx_deferred_start;
#ifdef RTE_LIBRTE_SECURITY
	txq->using_ipsec = !!(dev->data->dev_conf.txmode.offloads &
			DEV_TX_OFFLOAD_SECURITY);
#endif

	/*
	 * Modification to set VFTDT for virtual function if vf is detected
	 */

	txq->tdt_reg_addr = YUSUR2_PCI_REG_ADDR(hw, YUSUR2_TDT(txq->reg_idx));

	txq->tx_ring_phys_addr = tz->iova;
	txq->tx_ring = (union yusur2_adv_tx_desc *) tz->addr;

	/* Allocate software ring */
	txq->sw_ring = rte_zmalloc_socket("txq->sw_ring",
				sizeof(struct yusur2_tx_entry) * nb_desc,
				RTE_CACHE_LINE_SIZE, socket_id);
	if (txq->sw_ring == NULL) {
		yusur2_tx_queue_release(txq);
		return -ENOMEM;
	}
	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64,
		     txq->sw_ring, txq->tx_ring, txq->tx_ring_phys_addr);

	/* set up vector or scalar TX function as appropriate */
	yusur2_set_tx_function(dev, txq);

	txq->ops->reset(txq);

	dev->data->tx_queues[queue_idx] = txq;


	return 0;
}

/**
 * yusur2_free_sc_cluster - free the not-yet-completed scattered cluster
 *
 * The "next" pointer of the last segment of (not-yet-completed) RSC clusters
 * in the sw_rsc_ring is not set to NULL but rather points to the next
 * mbuf of this RSC aggregation (that has not been completed yet and still
 * resides on the HW ring). So, instead of calling for rte_pktmbuf_free() we
 * will just free first "nb_segs" segments of the cluster explicitly by calling
 * an rte_pktmbuf_free_seg().
 *
 * @m scattered cluster head
 */
static void __attribute__((cold))
yusur2_free_sc_cluster(struct rte_mbuf *m)
{
	uint16_t i, nb_segs = m->nb_segs;
	struct rte_mbuf *next_seg;

	for (i = 0; i < nb_segs; i++) {
		next_seg = m->next;
		rte_pktmbuf_free_seg(m);
		m = next_seg;
	}
}

static void __attribute__((cold))
yusur2_rx_queue_release_mbufs(struct yusur2_rx_queue *rxq)
{
	unsigned i;

#ifdef RTE_YUSUR2_INC_VECTOR
	/* SSE Vector driver has a different way of releasing mbufs. */
	if (rxq->rx_using_sse) {
		yusur2_rx_queue_release_mbufs_vec(rxq);
		return;
	}
#endif

	if (rxq->sw_ring != NULL) {
		for (i = 0; i < rxq->nb_rx_desc; i++) {
			if (rxq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(rxq->sw_ring[i].mbuf);
				rxq->sw_ring[i].mbuf = NULL;
			}
		}
		if (rxq->rx_nb_avail) {
			for (i = 0; i < rxq->rx_nb_avail; ++i) {
				struct rte_mbuf *mb;

				mb = rxq->rx_stage[rxq->rx_next_avail + i];
				rte_pktmbuf_free_seg(mb);
			}
			rxq->rx_nb_avail = 0;
		}
	}

	if (rxq->sw_sc_ring)
		for (i = 0; i < rxq->nb_rx_desc; i++)
			if (rxq->sw_sc_ring[i].fbuf) {
				yusur2_free_sc_cluster(rxq->sw_sc_ring[i].fbuf);
				rxq->sw_sc_ring[i].fbuf = NULL;
			}
}

static void __attribute__((cold))
yusur2_rx_queue_release(struct yusur2_rx_queue *rxq)
{
	if (rxq != NULL) {
		yusur2_rx_queue_release_mbufs(rxq);
		rte_free(rxq->sw_ring);
		rte_free(rxq->sw_sc_ring);
		rte_free(rxq);
	}
}

void __attribute__((cold))
yusur2_dev_rx_queue_release(void *rxq)
{
	yusur2_rx_queue_release(rxq);
}

/*
 * Check if Rx Burst Bulk Alloc function can be used.
 * Return
 *        0: the preconditions are satisfied and the bulk allocation function
 *           can be used.
 *  -EINVAL: the preconditions are NOT satisfied and the default Rx burst
 *           function must be used.
 */
static inline int __attribute__((cold))
check_rx_burst_bulk_alloc_preconditions(struct yusur2_rx_queue *rxq)
{
	int ret = 0;

	/*
	 * Make sure the following pre-conditions are satisfied:
	 *   rxq->rx_free_thresh >= RTE_PMD_YUSUR2_RX_MAX_BURST
	 *   rxq->rx_free_thresh < rxq->nb_rx_desc
	 *   (rxq->nb_rx_desc % rxq->rx_free_thresh) == 0
	 * Scattered packets are not supported.  This should be checked
	 * outside of this function.
	 */
	if (!(rxq->rx_free_thresh >= RTE_PMD_YUSUR2_RX_MAX_BURST)) {
		PMD_INIT_LOG(DEBUG, "Rx Burst Bulk Alloc Preconditions: "
			     "rxq->rx_free_thresh=%d, "
			     "RTE_PMD_YUSUR2_RX_MAX_BURST=%d",
			     rxq->rx_free_thresh, RTE_PMD_YUSUR2_RX_MAX_BURST);
		ret = -EINVAL;
	} else if (!(rxq->rx_free_thresh < rxq->nb_rx_desc)) {
		PMD_INIT_LOG(DEBUG, "Rx Burst Bulk Alloc Preconditions: "
			     "rxq->rx_free_thresh=%d, "
			     "rxq->nb_rx_desc=%d",
			     rxq->rx_free_thresh, rxq->nb_rx_desc);
		ret = -EINVAL;
	} else if (!((rxq->nb_rx_desc % rxq->rx_free_thresh) == 0)) {
		PMD_INIT_LOG(DEBUG, "Rx Burst Bulk Alloc Preconditions: "
			     "rxq->nb_rx_desc=%d, "
			     "rxq->rx_free_thresh=%d",
			     rxq->nb_rx_desc, rxq->rx_free_thresh);
		ret = -EINVAL;
	}

	return ret;
}

/* Reset dynamic yusur2_rx_queue fields back to defaults */
static void __attribute__((cold))
yusur2_reset_rx_queue(struct yusur2_adapter *adapter, struct yusur2_rx_queue *rxq)
{
	static const union yusur2_adv_rx_desc zeroed_desc = {{0}};
	unsigned i;
	uint16_t len = rxq->nb_rx_desc;

	/*
	 * By default, the Rx queue setup function allocates enough memory for
	 * YUSUR2_MAX_RING_DESC.  The Rx Burst bulk allocation function requires
	 * extra memory at the end of the descriptor ring to be zero'd out.
	 */
	if (adapter->rx_bulk_alloc_allowed)
		/* zero out extra memory */
		len += RTE_PMD_YUSUR2_RX_MAX_BURST;

	/*
	 * Zero out HW ring memory. Zero out extra memory at the end of
	 * the H/W ring so look-ahead logic in Rx Burst bulk alloc function
	 * reads extra memory as zeros.
	 */
	for (i = 0; i < len; i++) {
		rxq->rx_ring[i] = zeroed_desc;
	}

	/*
	 * initialize extra software ring entries. Space for these extra
	 * entries is always allocated
	 */
	memset(&rxq->fake_mbuf, 0x0, sizeof(rxq->fake_mbuf));
	for (i = rxq->nb_rx_desc; i < len; ++i) {
		rxq->sw_ring[i].mbuf = &rxq->fake_mbuf;
	}

	rxq->rx_nb_avail = 0;
	rxq->rx_next_avail = 0;
	rxq->rx_free_trigger = (uint16_t)(rxq->rx_free_thresh - 1);
	rxq->rx_tail = 0;
	rxq->nb_rx_hold = 0;
	rxq->pkt_first_seg = NULL;
	rxq->pkt_last_seg = NULL;

#ifdef RTE_YUSUR2_INC_VECTOR
	rxq->rxrearm_start = 0;
	rxq->rxrearm_nb = 0;
#endif
}

static int
yusur2_is_vf(struct rte_eth_dev *dev)
{
	return 0;
}

uint64_t
yusur2_get_rx_queue_offloads(struct rte_eth_dev *dev)
{
	//TODO:check...
	uint64_t offloads = 0;

	return offloads;
}

uint64_t
yusur2_get_rx_port_offloads(struct rte_eth_dev *dev)
{
	//TODO:check...
	uint64_t offloads;

	return offloads;
}

int __attribute__((cold))
yusur2_dev_rx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_rxconf *rx_conf,
			 struct rte_mempool *mp)
{
	const struct rte_memzone *rz;
	struct yusur2_rx_queue *rxq;
	struct yusur2_hw     *hw;
	uint16_t len;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	uint64_t offloads;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	offloads = rx_conf->offloads | dev->data->dev_conf.rxmode.offloads;

	/*
	 * Validate number of receive descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of YUSUR2_ALIGN.
	 */
	if (nb_desc % YUSUR2_RXD_ALIGN != 0 ||
			(nb_desc > YUSUR2_MAX_RING_DESC) ||
			(nb_desc < YUSUR2_MIN_RING_DESC)) {
		return -EINVAL;
	}

	/* Free memory prior to re-allocation if needed... */
	if (dev->data->rx_queues[queue_idx] != NULL) {
		yusur2_rx_queue_release(dev->data->rx_queues[queue_idx]);
		dev->data->rx_queues[queue_idx] = NULL;
	}

	/* First allocate the rx queue data structure */
	rxq = rte_zmalloc_socket("ethdev RX queue", sizeof(struct yusur2_rx_queue),
				 RTE_CACHE_LINE_SIZE, socket_id);
	if (rxq == NULL)
		return -ENOMEM;
	rxq->mb_pool = mp;
	rxq->nb_rx_desc = nb_desc;
	rxq->rx_free_thresh = rx_conf->rx_free_thresh;
	rxq->queue_id = queue_idx;
	rxq->reg_idx = (uint16_t)((RTE_ETH_DEV_SRIOV(dev).active == 0) ?
		queue_idx : RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx + queue_idx);
	rxq->port_id = dev->data->port_id;
	if (dev->data->dev_conf.rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC)
		rxq->crc_len = RTE_ETHER_CRC_LEN;
	else
		rxq->crc_len = 0;
	rxq->drop_en = rx_conf->rx_drop_en;
	rxq->rx_deferred_start = rx_conf->rx_deferred_start;
	rxq->offloads = offloads;

	/*
	 * The packet type in RX descriptor is different for different NICs.
	 * Some bits are used for x550 but reserved for other NICS.
	 * So set different masks for different NICs.
	 */
	//TODO:

	/*
	 * Allocate RX ring hardware descriptors. A memzone large enough to
	 * handle the maximum ring size is allocated in order to allow for
	 * resizing in later calls to the queue setup function.
	 */
	rz = rte_eth_dma_zone_reserve(dev, "rx_ring", queue_idx,
				      RX_RING_SZ, YUSUR2_ALIGN, socket_id);
	if (rz == NULL) {
		yusur2_rx_queue_release(rxq);
		return -ENOMEM;
	}

	/*
	 * Zero init all the descriptors in the ring.
	 */
	memset(rz->addr, 0, RX_RING_SZ);

	/*
	 * Modified to setup VFRDT for Virtual Function
	 */

	//TODO:
	// rxq->rdt_reg_addr =
	// 	YUSUR2_PCI_REG_ADDR(hw, YUSUR2_RDT(rxq->reg_idx));
	// rxq->rdh_reg_addr =
	// 	YUSUR2_PCI_REG_ADDR(hw, YUSUR2_RDH(rxq->reg_idx));


	rxq->rx_ring_phys_addr = rz->iova;
	rxq->rx_ring = (union yusur2_adv_rx_desc *) rz->addr;

	/*
	 * Certain constraints must be met in order to use the bulk buffer
	 * allocation Rx burst function. If any of Rx queues doesn't meet them
	 * the feature should be disabled for the whole port.
	 */
	if (check_rx_burst_bulk_alloc_preconditions(rxq)) {
		PMD_INIT_LOG(DEBUG, "queue[%d] doesn't meet Rx Bulk Alloc "
				    "preconditions - canceling the feature for "
				    "the whole port[%d]",
			     rxq->queue_id, rxq->port_id);
		adapter->rx_bulk_alloc_allowed = false;
	}

	/*
	 * Allocate software ring. Allow for space at the end of the
	 * S/W ring to make sure look-ahead logic in bulk alloc Rx burst
	 * function does not access an invalid memory region.
	 */
	len = nb_desc;
	if (adapter->rx_bulk_alloc_allowed)
		len += RTE_PMD_YUSUR2_RX_MAX_BURST;

	rxq->sw_ring = rte_zmalloc_socket("rxq->sw_ring",
					  sizeof(struct yusur2_rx_entry) * len,
					  RTE_CACHE_LINE_SIZE, socket_id);
	if (!rxq->sw_ring) {
		yusur2_rx_queue_release(rxq);
		return -ENOMEM;
	}

	/*
	 * Always allocate even if it's not going to be needed in order to
	 * simplify the code.
	 *
	 * This ring is used in LRO and Scattered Rx cases and Scattered Rx may
	 * be requested in yusur2_dev_rx_init(), which is called later from
	 * dev_start() flow.
	 */
	rxq->sw_sc_ring =
		rte_zmalloc_socket("rxq->sw_sc_ring",
				   sizeof(struct yusur2_scattered_rx_entry) * len,
				   RTE_CACHE_LINE_SIZE, socket_id);
	if (!rxq->sw_sc_ring) {
		yusur2_rx_queue_release(rxq);
		return -ENOMEM;
	}

	PMD_INIT_LOG(DEBUG, "sw_ring=%p sw_sc_ring=%p hw_ring=%p "
			    "dma_addr=0x%"PRIx64,
		     rxq->sw_ring, rxq->sw_sc_ring, rxq->rx_ring,
		     rxq->rx_ring_phys_addr);

	if (!rte_is_power_of_2(nb_desc)) {
		PMD_INIT_LOG(DEBUG, "queue[%d] doesn't meet Vector Rx "
				    "preconditions - canceling the feature for "
				    "the whole port[%d]",
			     rxq->queue_id, rxq->port_id);
		adapter->rx_vec_allowed = false;
	} else
		yusur2_rxq_vec_setup(rxq);

	dev->data->rx_queues[queue_idx] = rxq;

	yusur2_reset_rx_queue(adapter, rxq);

	return 0;
}

uint32_t
yusur2_dev_rx_queue_count(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
#define YUSUR2_RXQ_SCAN_INTERVAL 4
	volatile union yusur2_adv_rx_desc *rxdp;
	struct yusur2_rx_queue *rxq;
	uint32_t desc = 0;

	rxq = dev->data->rx_queues[rx_queue_id];
	rxdp = &(rxq->rx_ring[rxq->rx_tail]);

	while ((desc < rxq->nb_rx_desc) &&
		(rxdp->wb.upper.status_error &
			rte_cpu_to_le_32(YUSUR2_RXDADV_STAT_DD))) {
		desc += YUSUR2_RXQ_SCAN_INTERVAL;
		rxdp += YUSUR2_RXQ_SCAN_INTERVAL;
		if (rxq->rx_tail + desc >= rxq->nb_rx_desc)
			rxdp = &(rxq->rx_ring[rxq->rx_tail +
				desc - rxq->nb_rx_desc]);
	}

	return desc;
}

int
yusur2_dev_rx_descriptor_done(void *rx_queue, uint16_t offset)
{
	volatile union yusur2_adv_rx_desc *rxdp;
	struct yusur2_rx_queue *rxq = rx_queue;
	uint32_t desc;

	if (unlikely(offset >= rxq->nb_rx_desc))
		return 0;
	desc = rxq->rx_tail + offset;
	if (desc >= rxq->nb_rx_desc)
		desc -= rxq->nb_rx_desc;

	rxdp = &rxq->rx_ring[desc];
	return !!(rxdp->wb.upper.status_error &
			rte_cpu_to_le_32(YUSUR2_RXDADV_STAT_DD));
}

int
yusur2_dev_rx_descriptor_status(void *rx_queue, uint16_t offset)
{
	struct yusur2_rx_queue *rxq = rx_queue;
	volatile uint32_t *status;
	uint32_t nb_hold, desc;

	if (unlikely(offset >= rxq->nb_rx_desc))
		return -EINVAL;

#ifdef RTE_YUSUR2_INC_VECTOR
	if (rxq->rx_using_sse)
		nb_hold = rxq->rxrearm_nb;
	else
#endif
		nb_hold = rxq->nb_rx_hold;
	if (offset >= rxq->nb_rx_desc - nb_hold)
		return RTE_ETH_RX_DESC_UNAVAIL;

	desc = rxq->rx_tail + offset;
	if (desc >= rxq->nb_rx_desc)
		desc -= rxq->nb_rx_desc;

	status = &rxq->rx_ring[desc].wb.upper.status_error;
	if (*status & rte_cpu_to_le_32(YUSUR2_RXDADV_STAT_DD))
		return RTE_ETH_RX_DESC_DONE;

	return RTE_ETH_RX_DESC_AVAIL;
}

int
yusur2_dev_tx_descriptor_status(void *tx_queue, uint16_t offset)
{
	struct yusur2_tx_queue *txq = tx_queue;
	volatile uint32_t *status;
	uint32_t desc;

	if (unlikely(offset >= txq->nb_tx_desc))
		return -EINVAL;

	desc = txq->tx_tail + offset;
	/* go to next desc that has the RS bit */
	desc = ((desc + txq->tx_rs_thresh - 1) / txq->tx_rs_thresh) *
		txq->tx_rs_thresh;
	if (desc >= txq->nb_tx_desc) {
		desc -= txq->nb_tx_desc;
		if (desc >= txq->nb_tx_desc)
			desc -= txq->nb_tx_desc;
	}

	status = &txq->tx_ring[desc].wb.status;
	if (*status & rte_cpu_to_le_32(YUSUR2_ADVTXD_STAT_DD))
		return RTE_ETH_TX_DESC_DONE;

	return RTE_ETH_TX_DESC_FULL;
}

/*
 * Set up link loopback for X540/X550 mode Tx->Rx.
 */
static inline void __attribute__((cold))
yusur2_setup_loopback_link_x540_x550(struct yusur2_hw *hw, bool enable)
{
	uint32_t macc;
	PMD_INIT_FUNC_TRACE();

	u16 autoneg_reg = YUSUR2_MII_AUTONEG_REG;

	hw->phy.ops.read_reg(hw, YUSUR2_MDIO_AUTO_NEG_CONTROL,
			     YUSUR2_MDIO_AUTO_NEG_DEV_TYPE, &autoneg_reg);
	macc = YUSUR2_READ_REG(hw, YUSUR2_MACC);

	if (enable) {
		/* datasheet 15.2.1: disable AUTONEG (PHY Bit 7.0.C) */
		autoneg_reg |= YUSUR2_MII_AUTONEG_ENABLE;
		/* datasheet 15.2.1: MACC.FLU = 1 (force link up) */
		macc |= YUSUR2_MACC_FLU;
	} else {
		autoneg_reg &= ~YUSUR2_MII_AUTONEG_ENABLE;
		macc &= ~YUSUR2_MACC_FLU;
	}

	hw->phy.ops.write_reg(hw, YUSUR2_MDIO_AUTO_NEG_CONTROL,
			      YUSUR2_MDIO_AUTO_NEG_DEV_TYPE, autoneg_reg);

	YUSUR2_WRITE_REG(hw, YUSUR2_MACC, macc);
}

void __attribute__((cold))
yusur2_dev_clear_queues(struct rte_eth_dev *dev)
{
	unsigned i;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		struct yusur2_tx_queue *txq = dev->data->tx_queues[i];

		if (txq != NULL) {
			txq->ops->release_mbufs(txq);
			txq->ops->reset(txq);
		}
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		struct yusur2_rx_queue *rxq = dev->data->rx_queues[i];

		if (rxq != NULL) {
			yusur2_rx_queue_release_mbufs(rxq);
			yusur2_reset_rx_queue(adapter, rxq);
		}
	}
}

void
yusur2_dev_free_queues(struct rte_eth_dev *dev)
{
	unsigned i;

	PMD_INIT_FUNC_TRACE();

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		yusur2_dev_rx_queue_release(dev->data->rx_queues[i]);
		dev->data->rx_queues[i] = NULL;
	}
	dev->data->nb_rx_queues = 0;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		yusur2_dev_tx_queue_release(dev->data->tx_queues[i]);
		dev->data->tx_queues[i] = NULL;
	}
	dev->data->nb_tx_queues = 0;
}

/*********************************************************************
 *
 *  Device RX/TX init functions
 *
 **********************************************************************/

/**
 * Receive Side Scaling (RSS)
 * See section 7.1.2.8 in the following document:
 *     "Intel 82599 10 GbE Controller Datasheet" - Revision 2.1 October 2009
 *
 * Principles:
 * The source and destination IP addresses of the IP header and the source
 * and destination ports of TCP/UDP headers, if any, of received packets are
 * hashed against a configurable random key to compute a 32-bit RSS hash result.
 * The seven (7) LSBs of the 32-bit hash result are used as an index into a
 * 128-entry redirection table (RETA).  Each entry of the RETA provides a 3-bit
 * RSS output index which is used as the RX queue index where to store the
 * received packets.
 * The following output is supplied in the RX write-back descriptor:
 *     - 32-bit result of the Microsoft RSS hash function,
 *     - 4-bit RSS type field.
 */

/*
 * RSS random key supplied in section 7.1.2.8.3 of the Intel 82599 datasheet.
 * Used as the default key.
 */
static uint8_t rss_intel_key[40] = {
	0x6D, 0x5A, 0x56, 0xDA, 0x25, 0x5B, 0x0E, 0xC2,
	0x41, 0x67, 0x25, 0x3D, 0x43, 0xA3, 0x8F, 0xB0,
	0xD0, 0xCA, 0x2B, 0xCB, 0xAE, 0x7B, 0x30, 0xB4,
	0x77, 0xCB, 0x2D, 0xA3, 0x80, 0x30, 0xF2, 0x0C,
	0x6A, 0x42, 0xB7, 0x3B, 0xBE, 0xAC, 0x01, 0xFA,
};

static void
yusur2_rss_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw;
	uint32_t mrqc;
	uint32_t mrqc_reg;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mrqc_reg = yusur2_mrqc_reg_get(hw->mac.type);
	mrqc = YUSUR2_READ_REG(hw, mrqc_reg);
	mrqc &= ~YUSUR2_MRQC_RSSEN;
	YUSUR2_WRITE_REG(hw, mrqc_reg, mrqc);
}

static void
yusur2_hw_rss_hash_set(struct yusur2_hw *hw, struct rte_eth_rss_conf *rss_conf)
{
	uint8_t  *hash_key;
	uint32_t mrqc;
	uint32_t rss_key;
	uint64_t rss_hf;
	uint16_t i;
	uint32_t mrqc_reg;
	uint32_t rssrk_reg;

	mrqc_reg = yusur2_mrqc_reg_get(hw->mac.type);
	rssrk_reg = yusur2_rssrk_reg_get(hw->mac.type, 0);

	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Fill in RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key  = hash_key[(i * 4)];
			rss_key |= hash_key[(i * 4) + 1] << 8;
			rss_key |= hash_key[(i * 4) + 2] << 16;
			rss_key |= hash_key[(i * 4) + 3] << 24;
			YUSUR2_WRITE_REG_ARRAY(hw, rssrk_reg, i, rss_key);
		}
	}

	/* Set configured hashing protocols in MRQC register */
	rss_hf = rss_conf->rss_hf;
	mrqc = YUSUR2_MRQC_RSSEN; /* Enable RSS */
	if (rss_hf & ETH_RSS_IPV4)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV4;
	if (rss_hf & ETH_RSS_NONFRAG_IPV4_TCP)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hf & ETH_RSS_IPV6)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6;
	if (rss_hf & ETH_RSS_IPV6_EX)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6_EX;
	if (rss_hf & ETH_RSS_NONFRAG_IPV6_TCP)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hf & ETH_RSS_IPV6_TCP_EX)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6_EX_TCP;
	if (rss_hf & ETH_RSS_NONFRAG_IPV4_UDP)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hf & ETH_RSS_NONFRAG_IPV6_UDP)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hf & ETH_RSS_IPV6_UDP_EX)
		mrqc |= YUSUR2_MRQC_RSS_FIELD_IPV6_EX_UDP;
	YUSUR2_WRITE_REG(hw, mrqc_reg, mrqc);
}

int
yusur2_dev_rss_hash_update(struct rte_eth_dev *dev,
			  struct rte_eth_rss_conf *rss_conf)
{
	struct yusur2_hw *hw;
	uint32_t mrqc;
	uint64_t rss_hf;
	uint32_t mrqc_reg;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (!yusur2_rss_update_sp(hw->mac.type)) {
		PMD_DRV_LOG(ERR, "RSS hash update is not supported on this "
			"NIC.");
		return -ENOTSUP;
	}
	mrqc_reg = yusur2_mrqc_reg_get(hw->mac.type);

	/*
	 * Excerpt from section 7.1.2.8 Receive-Side Scaling (RSS):
	 *     "RSS enabling cannot be done dynamically while it must be
	 *      preceded by a software reset"
	 * Before changing anything, first check that the update RSS operation
	 * does not attempt to disable RSS, if RSS was enabled at
	 * initialization time, or does not attempt to enable RSS, if RSS was
	 * disabled at initialization time.
	 */
	rss_hf = rss_conf->rss_hf & YUSUR2_RSS_OFFLOAD_ALL;
	mrqc = YUSUR2_READ_REG(hw, mrqc_reg);
	if (!(mrqc & YUSUR2_MRQC_RSSEN)) { /* RSS disabled */
		if (rss_hf != 0) /* Enable RSS */
			return -(EINVAL);
		return 0; /* Nothing to do */
	}
	/* RSS enabled */
	if (rss_hf == 0) /* Disable RSS */
		return -(EINVAL);
	yusur2_hw_rss_hash_set(hw, rss_conf);
	return 0;
}

int
yusur2_dev_rss_hash_conf_get(struct rte_eth_dev *dev,
			    struct rte_eth_rss_conf *rss_conf)
{
	struct yusur2_hw *hw;
	uint8_t *hash_key;
	uint32_t mrqc;
	uint32_t rss_key;
	uint64_t rss_hf;
	uint16_t i;
	uint32_t mrqc_reg;
	uint32_t rssrk_reg;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mrqc_reg = yusur2_mrqc_reg_get(hw->mac.type);
	rssrk_reg = yusur2_rssrk_reg_get(hw->mac.type, 0);
	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Return RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key = YUSUR2_READ_REG_ARRAY(hw, rssrk_reg, i);
			hash_key[(i * 4)] = rss_key & 0x000000FF;
			hash_key[(i * 4) + 1] = (rss_key >> 8) & 0x000000FF;
			hash_key[(i * 4) + 2] = (rss_key >> 16) & 0x000000FF;
			hash_key[(i * 4) + 3] = (rss_key >> 24) & 0x000000FF;
		}
	}

	/* Get RSS functions configured in MRQC register */
	mrqc = YUSUR2_READ_REG(hw, mrqc_reg);
	if ((mrqc & YUSUR2_MRQC_RSSEN) == 0) { /* RSS is disabled */
		rss_conf->rss_hf = 0;
		return 0;
	}
	rss_hf = 0;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV4)
		rss_hf |= ETH_RSS_IPV4;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV4_TCP)
		rss_hf |= ETH_RSS_NONFRAG_IPV4_TCP;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6)
		rss_hf |= ETH_RSS_IPV6;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6_EX)
		rss_hf |= ETH_RSS_IPV6_EX;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6_TCP)
		rss_hf |= ETH_RSS_NONFRAG_IPV6_TCP;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6_EX_TCP)
		rss_hf |= ETH_RSS_IPV6_TCP_EX;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV4_UDP)
		rss_hf |= ETH_RSS_NONFRAG_IPV4_UDP;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6_UDP)
		rss_hf |= ETH_RSS_NONFRAG_IPV6_UDP;
	if (mrqc & YUSUR2_MRQC_RSS_FIELD_IPV6_EX_UDP)
		rss_hf |= ETH_RSS_IPV6_UDP_EX;
	rss_conf->rss_hf = rss_hf;
	return 0;
}

static void
yusur2_rss_configure(struct rte_eth_dev *dev)
{
	struct rte_eth_rss_conf rss_conf;
	struct yusur2_adapter *adapter;
	struct yusur2_hw *hw;
	uint32_t reta;
	uint16_t i;
	uint16_t j;
	uint16_t sp_reta_size;
	uint32_t reta_reg;

	PMD_INIT_FUNC_TRACE();
	adapter = dev->data->dev_private;
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	sp_reta_size = yusur2_reta_size_get(hw->mac.type);

	/*
	 * Fill in redirection table
	 * The byte-swap is needed because NIC registers are in
	 * little-endian order.
	 */
	if (adapter->rss_reta_updated == 0) {
		reta = 0;
		for (i = 0, j = 0; i < sp_reta_size; i++, j++) {
			reta_reg = yusur2_reta_reg_get(hw->mac.type, i);

			if (j == dev->data->nb_rx_queues)
				j = 0;
			reta = (reta << 8) | j;
			if ((i & 3) == 3)
				YUSUR2_WRITE_REG(hw, reta_reg,
						rte_bswap32(reta));
		}
	}

	/*
	 * Configure the RSS key and the RSS protocols used to compute
	 * the RSS hash of input packets.
	 */
	rss_conf = dev->data->dev_conf.rx_adv_conf.rss_conf;
	if ((rss_conf.rss_hf & YUSUR2_RSS_OFFLOAD_ALL) == 0) {
		yusur2_rss_disable(dev);
		return;
	}
	if (rss_conf.rss_key == NULL)
		rss_conf.rss_key = rss_intel_key; /* Default hash key */
	yusur2_hw_rss_hash_set(hw, &rss_conf);
}

#define NUM_VFTA_REGISTERS 128
#define NIC_RX_BUFFER_SIZE 0x200
#define X550_RX_BUFFER_SIZE 0x180
#define DCB_RX_CONFIG  1
#define DCB_TX_CONFIG  1
#define DCB_TX_PB      1024

static int __attribute__((cold))
yusur2_alloc_rx_queue_mbufs(struct yusur2_rx_queue *rxq)
{
	struct yusur2_rx_entry *rxe = rxq->sw_ring;
	uint64_t dma_addr;
	unsigned int i;

	/* Initialize software ring entries */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		volatile union yusur2_adv_rx_desc *rxd;
		struct rte_mbuf *mbuf = rte_mbuf_raw_alloc(rxq->mb_pool);

		if (mbuf == NULL) {
			PMD_INIT_LOG(ERR, "RX mbuf alloc failed queue_id=%u",
				     (unsigned) rxq->queue_id);
			return -ENOMEM;
		}

		mbuf->data_off = RTE_PKTMBUF_HEADROOM;
		mbuf->port = rxq->port_id;

		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(mbuf));
		rxd = &rxq->rx_ring[i];
		rxd->read.hdr_addr = 0;
		rxd->read.pkt_addr = dma_addr;
		rxe[i].mbuf = mbuf;
	}

	return 0;
}

static int
yusur2_config_vf_rss(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw;
	uint32_t mrqc;

	yusur2_rss_configure(dev);

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* MRQC: enable VF RSS */
	mrqc = YUSUR2_READ_REG(hw, YUSUR2_MRQC);
	mrqc &= ~YUSUR2_MRQC_MRQE_MASK;
	switch (RTE_ETH_DEV_SRIOV(dev).active) {
	case ETH_64_POOLS:
		mrqc |= YUSUR2_MRQC_VMDQRSS64EN;
		break;

	case ETH_32_POOLS:
		mrqc |= YUSUR2_MRQC_VMDQRSS32EN;
		break;

	default:
		PMD_INIT_LOG(ERR, "Invalid pool number in IOV mode with VMDQ RSS");
		return -EINVAL;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_MRQC, mrqc);

	return 0;
}

static int
yusur2_config_vf_default(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	switch (RTE_ETH_DEV_SRIOV(dev).active) {
	case ETH_64_POOLS:
		YUSUR2_WRITE_REG(hw, YUSUR2_MRQC,
			YUSUR2_MRQC_VMDQEN);
		break;

	case ETH_32_POOLS:
		YUSUR2_WRITE_REG(hw, YUSUR2_MRQC,
			YUSUR2_MRQC_VMDQRT4TCEN);
		break;

	case ETH_16_POOLS:
		YUSUR2_WRITE_REG(hw, YUSUR2_MRQC,
			YUSUR2_MRQC_VMDQRT8TCEN);
		break;
	default:
		PMD_INIT_LOG(ERR,
			"invalid pool number in IOV mode");
		break;
	}
	return 0;
}

static int
yusur2_dev_mq_rx_configure(struct rte_eth_dev *dev)
{
	//TODO: check
#if 0
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (hw->mac.type == yusur2_mac_82598EB)
		return 0;

	if (RTE_ETH_DEV_SRIOV(dev).active == 0) {
		/*
		 * SRIOV inactive scheme
		 * any DCB/RSS w/o VMDq multi-queue setting
		 */
		switch (dev->data->dev_conf.rxmode.mq_mode) {
		case ETH_MQ_RX_RSS:
		case ETH_MQ_RX_DCB_RSS:
		case ETH_MQ_RX_VMDQ_RSS:
			yusur2_rss_configure(dev);
			break;

		case ETH_MQ_RX_VMDQ_DCB:
			yusur2_vmdq_dcb_configure(dev);
			break;

		case ETH_MQ_RX_VMDQ_ONLY:
			yusur2_vmdq_rx_hw_configure(dev);
			break;

		case ETH_MQ_RX_NONE:
		default:
			/* if mq_mode is none, disable rss mode.*/
			yusur2_rss_disable(dev);
			break;
		}
	} else {
		/* SRIOV active scheme
		 * Support RSS together with SRIOV.
		 */
		switch (dev->data->dev_conf.rxmode.mq_mode) {
		case ETH_MQ_RX_RSS:
		case ETH_MQ_RX_VMDQ_RSS:
			yusur2_config_vf_rss(dev);
			break;
		case ETH_MQ_RX_VMDQ_DCB:
		case ETH_MQ_RX_DCB:
		/* In SRIOV, the configuration is the same as VMDq case */
			yusur2_vmdq_dcb_configure(dev);
			break;
		/* DCB/RSS together with SRIOV is not supported */
		case ETH_MQ_RX_VMDQ_DCB_RSS:
		case ETH_MQ_RX_DCB_RSS:
			PMD_INIT_LOG(ERR,
				"Could not support DCB/RSS with VMDq & SRIOV");
			return -1;
		default:
			yusur2_config_vf_default(dev);
			break;
		}
	}
#endif
	return 0;
}

static int
yusur2_dev_mq_tx_configure(struct rte_eth_dev *dev)
{
//TODO: multi-queue support..haha
#if 0
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t mtqc;
	uint32_t rttdcs;

	if (hw->mac.type == yusur2_mac_82598EB)
		return 0;

	/* disable arbiter before setting MTQC */
	rttdcs = YUSUR2_READ_REG(hw, YUSUR2_RTTDCS);
	rttdcs |= YUSUR2_RTTDCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDCS, rttdcs);

	if (RTE_ETH_DEV_SRIOV(dev).active == 0) {
		/*
		 * SRIOV inactive scheme
		 * any DCB w/o VMDq multi-queue setting
		 */
		if (dev->data->dev_conf.txmode.mq_mode == ETH_MQ_TX_VMDQ_ONLY)
			yusur2_vmdq_tx_hw_configure(hw);
		else {
			mtqc = YUSUR2_MTQC_64Q_1PB;
			YUSUR2_WRITE_REG(hw, YUSUR2_MTQC, mtqc);
		}
	} else {
		switch (RTE_ETH_DEV_SRIOV(dev).active) {

		/*
		 * SRIOV active scheme
		 * FIXME if support DCB together with VMDq & SRIOV
		 */
		case ETH_64_POOLS:
			mtqc = YUSUR2_MTQC_VT_ENA | YUSUR2_MTQC_64VF;
			break;
		case ETH_32_POOLS:
			mtqc = YUSUR2_MTQC_VT_ENA | YUSUR2_MTQC_32VF;
			break;
		case ETH_16_POOLS:
			mtqc = YUSUR2_MTQC_VT_ENA | YUSUR2_MTQC_RT_ENA |
				YUSUR2_MTQC_8TC_8TQ;
			break;
		default:
			mtqc = YUSUR2_MTQC_64Q_1PB;
			PMD_INIT_LOG(ERR, "invalid pool number in IOV mode");
		}
		YUSUR2_WRITE_REG(hw, YUSUR2_MTQC, mtqc);
	}

	/* re-enable arbiter */
	rttdcs &= ~YUSUR2_RTTDCS_ARBDIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDCS, rttdcs);
#endif
	return 0;
}

/**
 * yusur2_get_rscctl_maxdesc - Calculate the RSCCTL[n].MAXDESC for PF
 *
 * Return the RSCCTL[n].MAXDESC for 82599 and x540 PF devices according to the
 * spec rev. 3.0 chapter 8.2.3.8.13.
 *
 * @pool Memory pool of the Rx queue
 */
static inline uint32_t
yusur2_get_rscctl_maxdesc(struct rte_mempool *pool)
{
	struct rte_pktmbuf_pool_private *mp_priv = rte_mempool_get_priv(pool);

	/* MAXDESC * SRRCTL.BSIZEPKT must not exceed 64 KB minus one */
	uint16_t maxdesc =
		RTE_IPV4_MAX_PKT_LEN /
			(mp_priv->mbuf_data_room_size - RTE_PKTMBUF_HEADROOM);

	if (maxdesc >= 16)
		return YUSUR2_RSCCTL_MAXDESC_16;
	else if (maxdesc >= 8)
		return YUSUR2_RSCCTL_MAXDESC_8;
	else if (maxdesc >= 4)
		return YUSUR2_RSCCTL_MAXDESC_4;
	else
		return YUSUR2_RSCCTL_MAXDESC_1;
}

/**
 * yusur2_set_ivar - Setup the correct IVAR register for a particular MSIX
 * interrupt
 *
 * (Taken from FreeBSD tree)
 * (yes this is all very magic and confusing :)
 *
 * @dev port handle
 * @entry the register array entry
 * @vector the MSIX vector for this queue
 * @type RX/TX/MISC
 */
static void
yusur2_set_ivar(struct rte_eth_dev *dev, u8 entry, u8 vector, s8 type)
{
	//TODO: check...
}

void __attribute__((cold))
yusur2_set_rx_function(struct rte_eth_dev *dev)
{
	uint16_t i, rx_using_sse;
	struct yusur2_adapter *adapter = dev->data->dev_private;

	/*
	 * In order to allow Vector Rx there are a few configuration
	 * conditions to be met and Rx Bulk Allocation should be allowed.
	 */
	if (yusur2_rx_vec_dev_conf_condition_check(dev) ||
	    !adapter->rx_bulk_alloc_allowed) {
		PMD_INIT_LOG(DEBUG, "Port[%d] doesn't meet Vector Rx "
				    "preconditions or RTE_YUSUR2_INC_VECTOR is "
				    "not enabled",
			     dev->data->port_id);

		adapter->rx_vec_allowed = false;
	}

	/*
	 * Initialize the appropriate LRO callback.
	 *
	 * If all queues satisfy the bulk allocation preconditions
	 * (hw->rx_bulk_alloc_allowed is TRUE) then we may use bulk allocation.
	 * Otherwise use a single allocation version.
	 */
	if (dev->data->lro) {
		if (adapter->rx_bulk_alloc_allowed) {
			PMD_INIT_LOG(DEBUG, "LRO is requested. Using a bulk "
					   "allocation version");
			dev->rx_pkt_burst = yusur2_recv_pkts_lro_bulk_alloc;
		} else {
			PMD_INIT_LOG(DEBUG, "LRO is requested. Using a single "
					   "allocation version");
			dev->rx_pkt_burst = yusur2_recv_pkts_lro_single_alloc;
		}
	} else if (dev->data->scattered_rx) {
		/*
		 * Set the non-LRO scattered callback: there are Vector and
		 * single allocation versions.
		 */
		if (adapter->rx_vec_allowed) {
			PMD_INIT_LOG(DEBUG, "Using Vector Scattered Rx "
					    "callback (port=%d).",
				     dev->data->port_id);

			dev->rx_pkt_burst = yusur2_recv_scattered_pkts_vec;
		} else if (adapter->rx_bulk_alloc_allowed) {
			PMD_INIT_LOG(DEBUG, "Using a Scattered with bulk "
					   "allocation callback (port=%d).",
				     dev->data->port_id);
			dev->rx_pkt_burst = yusur2_recv_pkts_lro_bulk_alloc;
		} else {
			PMD_INIT_LOG(DEBUG, "Using Regualr (non-vector, "
					    "single allocation) "
					    "Scattered Rx callback "
					    "(port=%d).",
				     dev->data->port_id);

			dev->rx_pkt_burst = yusur2_recv_pkts_lro_single_alloc;
		}
	/*
	 * Below we set "simple" callbacks according to port/queues parameters.
	 * If parameters allow we are going to choose between the following
	 * callbacks:
	 *    - Vector
	 *    - Bulk Allocation
	 *    - Single buffer allocation (the simplest one)
	 */
	} else if (adapter->rx_vec_allowed) {
		PMD_INIT_LOG(DEBUG, "Vector rx enabled, please make sure RX "
				    "burst size no less than %d (port=%d).",
			     RTE_YUSUR2_DESCS_PER_LOOP,
			     dev->data->port_id);

		dev->rx_pkt_burst = yusur2_recv_pkts_vec;
	} else if (adapter->rx_bulk_alloc_allowed) {
		PMD_INIT_LOG(DEBUG, "Rx Burst Bulk Alloc Preconditions are "
				    "satisfied. Rx Burst Bulk Alloc function "
				    "will be used on port=%d.",
			     dev->data->port_id);

		dev->rx_pkt_burst = yusur2_recv_pkts_bulk_alloc;
	} else {
		PMD_INIT_LOG(DEBUG, "Rx Burst Bulk Alloc Preconditions are not "
				    "satisfied, or Scattered Rx is requested "
				    "(port=%d).",
			     dev->data->port_id);

		dev->rx_pkt_burst = yusur2_recv_pkts;
	}

	/* Propagate information about RX function choice through all queues. */

	rx_using_sse =
		(dev->rx_pkt_burst == yusur2_recv_scattered_pkts_vec ||
		dev->rx_pkt_burst == yusur2_recv_pkts_vec);

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		struct yusur2_rx_queue *rxq = dev->data->rx_queues[i];

		rxq->rx_using_sse = rx_using_sse;
#ifdef RTE_LIBRTE_SECURITY
		rxq->using_ipsec = !!(dev->data->dev_conf.rxmode.offloads &
				DEV_RX_OFFLOAD_SECURITY);
#endif
	}
}

/**
 * yusur2_set_rsc - configure RSC related port HW registers
 *
 * Configures the port's RSC related registers according to the 4.6.7.2 chapter
 * of 82599 Spec (x540 configuration is virtually the same).
 *
 * @dev port handle
 *
 * Returns 0 in case of success or a non-zero error code
 */
static int
yusur2_set_rsc(struct rte_eth_dev *dev)
{
	struct rte_eth_rxmode *rx_conf = &dev->data->dev_conf.rxmode;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_dev_info dev_info = { 0 };
	bool rsc_capable = false;
	uint16_t i;
	uint32_t rdrxctl;
	uint32_t rfctl;

	/* Sanity check */
	dev->dev_ops->dev_infos_get(dev, &dev_info);
	if (dev_info.rx_offload_capa & DEV_RX_OFFLOAD_TCP_LRO)
		rsc_capable = true;

	if (!rsc_capable && (rx_conf->offloads & DEV_RX_OFFLOAD_TCP_LRO)) {
		PMD_INIT_LOG(CRIT, "LRO is requested on HW that doesn't "
				   "support it");
		return -EINVAL;
	}

	/* RSC global configuration (chapter 4.6.7.2.1 of 82599 Spec) */

	if ((rx_conf->offloads & DEV_RX_OFFLOAD_KEEP_CRC) &&
	     (rx_conf->offloads & DEV_RX_OFFLOAD_TCP_LRO)) {
		/*
		 * According to chapter of 4.6.7.2.1 of the Spec Rev.
		 * 3.0 RSC configuration requires HW CRC stripping being
		 * enabled. If user requested both HW CRC stripping off
		 * and RSC on - return an error.
		 */
		PMD_INIT_LOG(CRIT, "LRO can't be enabled when HW CRC "
				    "is disabled");
		return -EINVAL;
	}

	/* RFCTL configuration  */
	rfctl = YUSUR2_READ_REG(hw, YUSUR2_RFCTL);
	if ((rsc_capable) && (rx_conf->offloads & DEV_RX_OFFLOAD_TCP_LRO))
		/*
		 * Since NFS packets coalescing is not supported - clear
		 * RFCTL.NFSW_DIS and RFCTL.NFSR_DIS when RSC is
		 * enabled.
		 */
		rfctl &= ~(YUSUR2_RFCTL_RSC_DIS | YUSUR2_RFCTL_NFSW_DIS |
			   YUSUR2_RFCTL_NFSR_DIS);
	else
		rfctl |= YUSUR2_RFCTL_RSC_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_RFCTL, rfctl);

	/* If LRO hasn't been requested - we are done here. */
	if (!(rx_conf->offloads & DEV_RX_OFFLOAD_TCP_LRO))
		return 0;

	/* Set RDRXCTL.RSCACKC bit */
	rdrxctl = YUSUR2_READ_REG(hw, YUSUR2_RDRXCTL);
	rdrxctl |= YUSUR2_RDRXCTL_RSCACKC;
	YUSUR2_WRITE_REG(hw, YUSUR2_RDRXCTL, rdrxctl);

	/* Per-queue RSC configuration (chapter 4.6.7.2.2 of 82599 Spec) */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		struct yusur2_rx_queue *rxq = dev->data->rx_queues[i];
		uint32_t srrctl =
			YUSUR2_READ_REG(hw, YUSUR2_SRRCTL(rxq->reg_idx));
		uint32_t rscctl =
			YUSUR2_READ_REG(hw, YUSUR2_RSCCTL(rxq->reg_idx));
		uint32_t psrtype =
			YUSUR2_READ_REG(hw, YUSUR2_PSRTYPE(rxq->reg_idx));
		uint32_t eitr =
			YUSUR2_READ_REG(hw, YUSUR2_EITR(rxq->reg_idx));

		/*
		 * yusur2 PMD doesn't support header-split at the moment.
		 *
		 * Following the 4.6.7.2.1 chapter of the 82599/x540
		 * Spec if RSC is enabled the SRRCTL[n].BSIZEHEADER
		 * should be configured even if header split is not
		 * enabled. We will configure it 128 bytes following the
		 * recommendation in the spec.
		 */
		srrctl &= ~YUSUR2_SRRCTL_BSIZEHDR_MASK;
		srrctl |= (128 << YUSUR2_SRRCTL_BSIZEHDRSIZE_SHIFT) &
					    YUSUR2_SRRCTL_BSIZEHDR_MASK;

		/*
		 * TODO: Consider setting the Receive Descriptor Minimum
		 * Threshold Size for an RSC case. This is not an obviously
		 * beneficiary option but the one worth considering...
		 */

		rscctl |= YUSUR2_RSCCTL_RSCEN;
		rscctl |= yusur2_get_rscctl_maxdesc(rxq->mb_pool);
		psrtype |= YUSUR2_PSRTYPE_TCPHDR;

		/*
		 * RSC: Set ITR interval corresponding to 2K ints/s.
		 *
		 * Full-sized RSC aggregations for a 10Gb/s link will
		 * arrive at about 20K aggregation/s rate.
		 *
		 * 2K inst/s rate will make only 10% of the
		 * aggregations to be closed due to the interrupt timer
		 * expiration for a streaming at wire-speed case.
		 *
		 * For a sparse streaming case this setting will yield
		 * at most 500us latency for a single RSC aggregation.
		 */
		eitr &= ~YUSUR2_EITR_ITR_INT_MASK;
		eitr |= YUSUR2_EITR_INTERVAL_US(YUSUR2_QUEUE_ITR_INTERVAL_DEFAULT);
		eitr |= YUSUR2_EITR_CNT_WDIS;

		YUSUR2_WRITE_REG(hw, YUSUR2_SRRCTL(rxq->reg_idx), srrctl);
		YUSUR2_WRITE_REG(hw, YUSUR2_RSCCTL(rxq->reg_idx), rscctl);
		YUSUR2_WRITE_REG(hw, YUSUR2_PSRTYPE(rxq->reg_idx), psrtype);
		YUSUR2_WRITE_REG(hw, YUSUR2_EITR(rxq->reg_idx), eitr);

		/*
		 * RSC requires the mapping of the queue to the
		 * interrupt vector.
		 */
		yusur2_set_ivar(dev, rxq->reg_idx, i, 0);
	}

	dev->data->lro = 1;

	PMD_INIT_LOG(DEBUG, "enabling LRO mode");

	return 0;
}

/*
 * Initializes Receive Unit.
 */
int __attribute__((cold))
yusur2_dev_rx_init(struct rte_eth_dev *dev)
{
	struct yusur2_hw     *hw;
	struct yusur2_rx_queue *rxq;
	uint64_t bus_addr;
	uint32_t rxctrl;
	uint32_t fctrl;
	uint32_t hlreg0;
	uint32_t maxfrs;
	uint32_t srrctl;
	uint32_t rdrxctl;
	uint32_t rxcsum;
	uint16_t buf_size;
	uint16_t i;
	struct rte_eth_rxmode *rx_conf = &dev->data->dev_conf.rxmode;
	int rc;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Make sure receives are disabled while setting
	 * up the RX context (registers, descriptor rings, etc.).
	 */
	rxctrl = YUSUR2_READ_REG(hw, YUSUR2_RXCTRL);
	YUSUR2_WRITE_REG(hw, YUSUR2_RXCTRL, rxctrl & ~YUSUR2_RXCTRL_RXEN);

	/* Enable receipt of broadcasted frames */
	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl |= YUSUR2_FCTRL_BAM;
	fctrl |= YUSUR2_FCTRL_DPF;
	fctrl |= YUSUR2_FCTRL_PMCF;
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	/*
	 * Configure CRC stripping, if any.
	 */
	hlreg0 = YUSUR2_READ_REG(hw, YUSUR2_HLREG0);
	if (rx_conf->offloads & DEV_RX_OFFLOAD_KEEP_CRC)
		hlreg0 &= ~YUSUR2_HLREG0_RXCRCSTRP;
	else
		hlreg0 |= YUSUR2_HLREG0_RXCRCSTRP;

	/*
	 * Configure jumbo frame support, if any.
	 */
	if (rx_conf->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME) {
		hlreg0 |= YUSUR2_HLREG0_JUMBOEN;
		maxfrs = YUSUR2_READ_REG(hw, YUSUR2_MAXFRS);
		maxfrs &= 0x0000FFFF;
		maxfrs |= (rx_conf->max_rx_pkt_len << 16);
		YUSUR2_WRITE_REG(hw, YUSUR2_MAXFRS, maxfrs);
	} else
		hlreg0 &= ~YUSUR2_HLREG0_JUMBOEN;

	/*
	 * If loopback mode is configured, set LPBK bit.
	 */
	if (dev->data->dev_conf.lpbk_mode != 0) {
		rc = yusur2_check_supported_loopback_mode(dev);
		if (rc < 0) {
			PMD_INIT_LOG(ERR, "Unsupported loopback mode");
			return rc;
		}
		hlreg0 |= YUSUR2_HLREG0_LPBK;
	} else {
		hlreg0 &= ~YUSUR2_HLREG0_LPBK;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, hlreg0);

	/*
	 * Assume no header split and no VLAN strip support
	 * on any Rx queue first .
	 */
	rx_conf->offloads &= ~DEV_RX_OFFLOAD_VLAN_STRIP;
	/* Setup RX queues */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];

		/*
		 * Reset crc_len in case it was changed after queue setup by a
		 * call to configure.
		 */
		if (rx_conf->offloads & DEV_RX_OFFLOAD_KEEP_CRC)
			rxq->crc_len = RTE_ETHER_CRC_LEN;
		else
			rxq->crc_len = 0;

		/* Setup the Base and Length of the Rx Descriptor Rings */
		bus_addr = rxq->rx_ring_phys_addr;
		YUSUR2_WRITE_REG(hw, YUSUR2_RDBAL(rxq->reg_idx),
				(uint32_t)(bus_addr & 0x00000000ffffffffULL));
		YUSUR2_WRITE_REG(hw, YUSUR2_RDBAH(rxq->reg_idx),
				(uint32_t)(bus_addr >> 32));
		YUSUR2_WRITE_REG(hw, YUSUR2_RDLEN(rxq->reg_idx),
				rxq->nb_rx_desc * sizeof(union yusur2_adv_rx_desc));
		YUSUR2_WRITE_REG(hw, YUSUR2_RDH(rxq->reg_idx), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_RDT(rxq->reg_idx), 0);

		/* Configure the SRRCTL register */
		srrctl = YUSUR2_SRRCTL_DESCTYPE_ADV_ONEBUF;

		/* Set if packets are dropped when no descriptors available */
		if (rxq->drop_en)
			srrctl |= YUSUR2_SRRCTL_DROP_EN;

		/*
		 * Configure the RX buffer size in the BSIZEPACKET field of
		 * the SRRCTL register of the queue.
		 * The value is in 1 KB resolution. Valid values can be from
		 * 1 KB to 16 KB.
		 */
		buf_size = (uint16_t)(rte_pktmbuf_data_room_size(rxq->mb_pool) -
			RTE_PKTMBUF_HEADROOM);
		srrctl |= ((buf_size >> YUSUR2_SRRCTL_BSIZEPKT_SHIFT) &
			   YUSUR2_SRRCTL_BSIZEPKT_MASK);

		YUSUR2_WRITE_REG(hw, YUSUR2_SRRCTL(rxq->reg_idx), srrctl);

		buf_size = (uint16_t) ((srrctl & YUSUR2_SRRCTL_BSIZEPKT_MASK) <<
				       YUSUR2_SRRCTL_BSIZEPKT_SHIFT);

		/* It adds dual VLAN length for supporting dual VLAN */
		if (dev->data->dev_conf.rxmode.max_rx_pkt_len +
					    2 * YUSUR2_VLAN_TAG_SIZE > buf_size)
			dev->data->scattered_rx = 1;
		if (rxq->offloads & DEV_RX_OFFLOAD_VLAN_STRIP)
			rx_conf->offloads |= DEV_RX_OFFLOAD_VLAN_STRIP;
	}

	if (rx_conf->offloads & DEV_RX_OFFLOAD_SCATTER)
		dev->data->scattered_rx = 1;

	/*
	 * Device configured with multiple RX queues.
	 */
	yusur2_dev_mq_rx_configure(dev);

	/*
	 * Setup the Checksum Register.
	 * Disable Full-Packet Checksum which is mutually exclusive with RSS.
	 * Enable IP/L4 checkum computation by hardware if requested to do so.
	 */
	rxcsum = YUSUR2_READ_REG(hw, YUSUR2_RXCSUM);
	rxcsum |= YUSUR2_RXCSUM_PCSD;
	if (rx_conf->offloads & DEV_RX_OFFLOAD_CHECKSUM)
		rxcsum |= YUSUR2_RXCSUM_IPPCSE;
	else
		rxcsum &= ~YUSUR2_RXCSUM_IPPCSE;

	YUSUR2_WRITE_REG(hw, YUSUR2_RXCSUM, rxcsum);

	rc = yusur2_set_rsc(dev);
	if (rc)
		return rc;

	yusur2_set_rx_function(dev);

	return 0;
}

/*
 * Initializes Transmit Unit.
 */
void __attribute__((cold))
yusur2_dev_tx_init(struct rte_eth_dev *dev)
{
//TODO: how to init yusur hw tx unit... important...
#if 0
	struct yusur2_hw     *hw;
	struct yusur2_tx_queue *txq;
	uint64_t bus_addr;
	uint32_t hlreg0;
	uint32_t txctrl;
	uint16_t i;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Enable TX CRC (checksum offload requirement) and hw padding
	 * (TSO requirement)
	 */
	hlreg0 = YUSUR2_READ_REG(hw, YUSUR2_HLREG0);
	hlreg0 |= (YUSUR2_HLREG0_TXCRCEN | YUSUR2_HLREG0_TXPADEN);
	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, hlreg0);

	/* Setup the Base and Length of the Tx Descriptor Rings */
	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];

		bus_addr = txq->tx_ring_phys_addr;
		YUSUR2_WRITE_REG(hw, YUSUR2_TDBAL(txq->reg_idx),
				(uint32_t)(bus_addr & 0x00000000ffffffffULL));
		YUSUR2_WRITE_REG(hw, YUSUR2_TDBAH(txq->reg_idx),
				(uint32_t)(bus_addr >> 32));
		YUSUR2_WRITE_REG(hw, YUSUR2_TDLEN(txq->reg_idx),
				txq->nb_tx_desc * sizeof(union yusur2_adv_tx_desc));
		/* Setup the HW Tx Head and TX Tail descriptor pointers */
		YUSUR2_WRITE_REG(hw, YUSUR2_TDH(txq->reg_idx), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_TDT(txq->reg_idx), 0);

		/*
		 * Disable Tx Head Writeback RO bit, since this hoses
		 * bookkeeping if things aren't delivered in order.
		 */
		switch (hw->mac.type) {
		case yusur2_mac_82598EB:
			txctrl = YUSUR2_READ_REG(hw,
						YUSUR2_DCA_TXCTRL(txq->reg_idx));
			txctrl &= ~YUSUR2_DCA_TXCTRL_DESC_WRO_EN;
			YUSUR2_WRITE_REG(hw, YUSUR2_DCA_TXCTRL(txq->reg_idx),
					txctrl);
			break;

		case yusur2_mac_82599EB:
		case yusur2_mac_X540:
		case yusur2_mac_X550:
		case yusur2_mac_X550EM_x:
		case yusur2_mac_X550EM_a:
		default:
			txctrl = YUSUR2_READ_REG(hw,
						YUSUR2_DCA_TXCTRL_82599(txq->reg_idx));
			txctrl &= ~YUSUR2_DCA_TXCTRL_DESC_WRO_EN;
			YUSUR2_WRITE_REG(hw, YUSUR2_DCA_TXCTRL_82599(txq->reg_idx),
					txctrl);
			break;
		}
	}

	/* Device configured with multiple TX queues. */
	yusur2_dev_mq_tx_configure(dev);
#endif
}

/*
 * Check if requested loopback mode is supported
 */
int
yusur2_check_supported_loopback_mode(struct rte_eth_dev *dev)
{
//TODO:
#if 0
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	return 0
#endif
}

/*
 * Set up link for 82599 loopback mode Tx->Rx.
 */
static inline void __attribute__((cold))
yusur2_setup_loopback_link_82599(struct yusur2_hw *hw)
{
	PMD_INIT_FUNC_TRACE();

	if (yusur2_verify_lesm_fw_enabled_82599(hw)) {
		if (hw->mac.ops.acquire_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM) !=
				YUSUR2_SUCCESS) {
			PMD_INIT_LOG(ERR, "Could not enable loopback mode");
			/* ignore error */
			return;
		}
	}

	/* Restart link */
	YUSUR2_WRITE_REG(hw,
			YUSUR2_AUTOC,
			YUSUR2_AUTOC_LMS_10G_LINK_NO_AN | YUSUR2_AUTOC_FLU);
	yusur2_reset_pipeline_82599(hw);

	hw->mac.ops.release_swfw_sync(hw, YUSUR2_GSSR_MAC_CSR_SM);
	msec_delay(50);
}


/*
 * Start Transmit and Receive Units.
 */
int __attribute__((cold))
yusur2_dev_rxtx_start(struct rte_eth_dev *dev)
{
	struct yusur2_hw     *hw;
	struct yusur2_tx_queue *txq;
	struct yusur2_rx_queue *rxq;
	uint32_t txdctl;
	uint32_t dmatxctl;
	uint32_t rxctrl;
	uint16_t i;
	int ret = 0;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];
		/* Setup Transmit Threshold Registers */
		txdctl = YUSUR2_READ_REG(hw, YUSUR2_TXDCTL(txq->reg_idx));
		txdctl |= txq->pthresh & 0x7F;
		txdctl |= ((txq->hthresh & 0x7F) << 8);
		txdctl |= ((txq->wthresh & 0x7F) << 16);
		YUSUR2_WRITE_REG(hw, YUSUR2_TXDCTL(txq->reg_idx), txdctl);
	}

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];
		if (!txq->tx_deferred_start) {
			ret = yusur2_dev_tx_queue_start(dev, i);
			if (ret < 0)
				return ret;
		}
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		if (!rxq->rx_deferred_start) {
			ret = yusur2_dev_rx_queue_start(dev, i);
			if (ret < 0)
				return ret;
		}
	}

	/* Enable Receive engine */
	rxctrl = YUSUR2_READ_REG(hw, YUSUR2_RXCTRL);
	rxctrl |= YUSUR2_RXCTRL_RXEN;
	hw->mac.ops.enable_rx_dma(hw, rxctrl);

	/* If loopback mode is enabled, set up the link accordingly */
	//TODO: loopback support

	return 0;
}

/*
 * Start Receive Units for specified queue.
 */
int __attribute__((cold))
yusur2_dev_rx_queue_start(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
//TODO: support...
	PMD_INIT_FUNC_TRACE();
#if 0
	struct yusur2_hw     *hw;
	struct yusur2_rx_queue *rxq;
	uint32_t rxdctl;
	int poll_ms;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	rxq = dev->data->rx_queues[rx_queue_id];

	/* Allocate buffers for descriptor rings */
	if (yusur2_alloc_rx_queue_mbufs(rxq) != 0) {
		PMD_INIT_LOG(ERR, "Could not alloc mbuf for queue:%d",
			     rx_queue_id);
		return -1;
	}
	rxdctl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx));
	rxdctl |= YUSUR2_RXDCTL_ENABLE;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx), rxdctl);

	/* Wait until RX Enable ready */
	poll_ms = RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS;
	do {
		rte_delay_ms(1);
		rxdctl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx));
	} while (--poll_ms && !(rxdctl & YUSUR2_RXDCTL_ENABLE));
	if (!poll_ms)
		PMD_INIT_LOG(ERR, "Could not enable Rx Queue %d", rx_queue_id);
	rte_wmb();
	YUSUR2_WRITE_REG(hw, YUSUR2_RDH(rxq->reg_idx), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_RDT(rxq->reg_idx), rxq->nb_rx_desc - 1);
	dev->data->rx_queue_state[rx_queue_id] = RTE_ETH_QUEUE_STATE_STARTED;
#endif
	return 0;
}

/*
 * Stop Receive Units for specified queue.
 */
int __attribute__((cold))
yusur2_dev_rx_queue_stop(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
//TODO: support...
	PMD_INIT_FUNC_TRACE();
#if 0
	struct yusur2_hw     *hw;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct yusur2_rx_queue *rxq;
	uint32_t rxdctl;
	int poll_ms;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	rxq = dev->data->rx_queues[rx_queue_id];

	rxdctl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx));
	rxdctl &= ~YUSUR2_RXDCTL_ENABLE;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx), rxdctl);

	/* Wait until RX Enable bit clear */
	poll_ms = RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS;
	do {
		rte_delay_ms(1);
		rxdctl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx));
	} while (--poll_ms && (rxdctl & YUSUR2_RXDCTL_ENABLE));
	if (!poll_ms)
		PMD_INIT_LOG(ERR, "Could not disable Rx Queue %d", rx_queue_id);

	rte_delay_us(RTE_YUSUR2_WAIT_100_US);

	yusur2_rx_queue_release_mbufs(rxq);
	yusur2_reset_rx_queue(adapter, rxq);
	dev->data->rx_queue_state[rx_queue_id] = RTE_ETH_QUEUE_STATE_STOPPED;
#endif
	return 0;
}


/*
 * Start Transmit Units for specified queue.
 */
int __attribute__((cold))
yusur2_dev_tx_queue_start(struct rte_eth_dev *dev, uint16_t tx_queue_id)
{
//TODO:check
#if 0
	struct yusur2_hw     *hw;
	struct yusur2_tx_queue *txq;
	uint32_t txdctl;
	int poll_ms;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	txq = dev->data->tx_queues[tx_queue_id];
	YUSUR2_WRITE_REG(hw, YUSUR2_TDH(txq->reg_idx), 0);
	txdctl = YUSUR2_READ_REG(hw, YUSUR2_TXDCTL(txq->reg_idx));
	txdctl |= YUSUR2_TXDCTL_ENABLE;
	YUSUR2_WRITE_REG(hw, YUSUR2_TXDCTL(txq->reg_idx), txdctl);

	/* Wait until TX Enable ready */
	if (hw->mac.type == yusur2_mac_82599EB) {
		poll_ms = RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS;
		do {
			rte_delay_ms(1);
			txdctl = YUSUR2_READ_REG(hw,
				YUSUR2_TXDCTL(txq->reg_idx));
		} while (--poll_ms && !(txdctl & YUSUR2_TXDCTL_ENABLE));
		if (!poll_ms)
			PMD_INIT_LOG(ERR, "Could not enable Tx Queue %d",
				tx_queue_id);
	}
	rte_wmb();
	YUSUR2_WRITE_REG(hw, YUSUR2_TDT(txq->reg_idx), 0);
	dev->data->tx_queue_state[tx_queue_id] = RTE_ETH_QUEUE_STATE_STARTED;
#endif
	return 0;
}

/*
 * Stop Transmit Units for specified queue.
 */
int __attribute__((cold))
yusur2_dev_tx_queue_stop(struct rte_eth_dev *dev, uint16_t tx_queue_id)
{
//TODO: check
#if 0
	struct yusur2_hw     *hw;
	struct yusur2_tx_queue *txq;
	uint32_t txdctl;
	uint32_t txtdh, txtdt;
	int poll_ms;

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	txq = dev->data->tx_queues[tx_queue_id];

	/* Wait until TX queue is empty */
	if (hw->mac.type == yusur2_mac_82599EB) {
		poll_ms = RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS;
		do {
			rte_delay_us(RTE_YUSUR2_WAIT_100_US);
			txtdh = YUSUR2_READ_REG(hw,
					       YUSUR2_TDH(txq->reg_idx));
			txtdt = YUSUR2_READ_REG(hw,
					       YUSUR2_TDT(txq->reg_idx));
		} while (--poll_ms && (txtdh != txtdt));
		if (!poll_ms)
			PMD_INIT_LOG(ERR,
				"Tx Queue %d is not empty when stopping.",
				tx_queue_id);
	}

	txdctl = YUSUR2_READ_REG(hw, YUSUR2_TXDCTL(txq->reg_idx));
	txdctl &= ~YUSUR2_TXDCTL_ENABLE;
	YUSUR2_WRITE_REG(hw, YUSUR2_TXDCTL(txq->reg_idx), txdctl);

	/* Wait until TX Enable bit clear */
	if (hw->mac.type == yusur2_mac_82599EB) {
		poll_ms = RTE_YUSUR2_REGISTER_POLL_WAIT_10_MS;
		do {
			rte_delay_ms(1);
			txdctl = YUSUR2_READ_REG(hw,
						YUSUR2_TXDCTL(txq->reg_idx));
		} while (--poll_ms && (txdctl & YUSUR2_TXDCTL_ENABLE));
		if (!poll_ms)
			PMD_INIT_LOG(ERR, "Could not disable Tx Queue %d",
				tx_queue_id);
	}

	if (txq->ops != NULL) {
		txq->ops->release_mbufs(txq);
		txq->ops->reset(txq);
	}
	dev->data->tx_queue_state[tx_queue_id] = RTE_ETH_QUEUE_STATE_STOPPED;
#endif
	return 0;
}

void
yusur2_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo)
{
	struct yusur2_rx_queue *rxq;

	rxq = dev->data->rx_queues[queue_id];

	qinfo->mp = rxq->mb_pool;
	qinfo->scattered_rx = dev->data->scattered_rx;
	qinfo->nb_desc = rxq->nb_rx_desc;

	qinfo->conf.rx_free_thresh = rxq->rx_free_thresh;
	qinfo->conf.rx_drop_en = rxq->drop_en;
	qinfo->conf.rx_deferred_start = rxq->rx_deferred_start;
	qinfo->conf.offloads = rxq->offloads;
}

void
yusur2_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo)
{
	struct yusur2_tx_queue *txq;

	txq = dev->data->tx_queues[queue_id];

	qinfo->nb_desc = txq->nb_tx_desc;

	qinfo->conf.tx_thresh.pthresh = txq->pthresh;
	qinfo->conf.tx_thresh.hthresh = txq->hthresh;
	qinfo->conf.tx_thresh.wthresh = txq->wthresh;

	qinfo->conf.tx_free_thresh = txq->tx_free_thresh;
	qinfo->conf.tx_rs_thresh = txq->tx_rs_thresh;
	qinfo->conf.offloads = txq->offloads;
	qinfo->conf.tx_deferred_start = txq->tx_deferred_start;
}

/*
 * [VF] Initializes Receive Unit.
 */
int __attribute__((cold))
yusur2vf_dev_rx_init(struct rte_eth_dev *dev)
{
	//TODO:

	return 0;
}

/*
 * [VF] Initializes Transmit Unit.
 */
void __attribute__((cold))
yusur2vf_dev_tx_init(struct rte_eth_dev *dev)
{
	//TODO:
}

/*
 * [VF] Start Transmit and Receive Units.
 */
void __attribute__((cold))
yusur2vf_dev_rxtx_start(struct rte_eth_dev *dev)
{
	//TODO:

}

int
yusur2_rss_conf_init(struct yusur2_rte_flow_rss_conf *out,
		    const struct rte_flow_action_rss *in)
{
	if (in->key_len > RTE_DIM(out->key) ||
	    in->queue_num > RTE_DIM(out->queue))
		return -EINVAL;
	out->conf = (struct rte_flow_action_rss){
		.func = in->func,
		.level = in->level,
		.types = in->types,
		.key_len = in->key_len,
		.queue_num = in->queue_num,
		.key = memcpy(out->key, in->key, in->key_len),
		.queue = memcpy(out->queue, in->queue,
				sizeof(*in->queue) * in->queue_num),
	};
	return 0;
}

int
yusur2_action_rss_same(const struct rte_flow_action_rss *comp,
		      const struct rte_flow_action_rss *with)
{
	return (comp->func == with->func &&
		comp->level == with->level &&
		comp->types == with->types &&
		comp->key_len == with->key_len &&
		comp->queue_num == with->queue_num &&
		!memcmp(comp->key, with->key, with->key_len) &&
		!memcmp(comp->queue, with->queue,
			sizeof(*with->queue) * with->queue_num));
}

int
yusur2_config_rss_filter(struct rte_eth_dev *dev,
		struct yusur2_rte_flow_rss_conf *conf, bool add)
{
	struct yusur2_hw *hw;
	uint32_t reta;
	uint16_t i;
	uint16_t j;
	uint16_t sp_reta_size;
	uint32_t reta_reg;
	struct rte_eth_rss_conf rss_conf = {
		.rss_key = conf->conf.key_len ?
			(void *)(uintptr_t)conf->conf.key : NULL,
		.rss_key_len = conf->conf.key_len,
		.rss_hf = conf->conf.types,
	};
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();
	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	sp_reta_size = yusur2_reta_size_get(hw->mac.type);

	if (!add) {
		if (yusur2_action_rss_same(&filter_info->rss_info.conf,
					  &conf->conf)) {
			yusur2_rss_disable(dev);
			memset(&filter_info->rss_info, 0,
				sizeof(struct yusur2_rte_flow_rss_conf));
			return 0;
		}
		return -EINVAL;
	}

	if (filter_info->rss_info.conf.queue_num)
		return -EINVAL;
	/* Fill in redirection table
	 * The byte-swap is needed because NIC registers are in
	 * little-endian order.
	 */
	reta = 0;
	for (i = 0, j = 0; i < sp_reta_size; i++, j++) {
		reta_reg = yusur2_reta_reg_get(hw->mac.type, i);

		if (j == conf->conf.queue_num)
			j = 0;
		reta = (reta << 8) | conf->conf.queue[j];
		if ((i & 3) == 3)
			YUSUR2_WRITE_REG(hw, reta_reg,
					rte_bswap32(reta));
	}

	/* Configure the RSS key and the RSS protocols used to compute
	 * the RSS hash of input packets.
	 */
	if ((rss_conf.rss_hf & YUSUR2_RSS_OFFLOAD_ALL) == 0) {
		yusur2_rss_disable(dev);
		return 0;
	}
	if (rss_conf.rss_key == NULL)
		rss_conf.rss_key = rss_intel_key; /* Default hash key */
	yusur2_hw_rss_hash_set(hw, &rss_conf);

	if (yusur2_rss_conf_init(&filter_info->rss_info, &conf->conf))
		return -EINVAL;

	return 0;
}

/* Stubs needed for linkage when CONFIG_RTE_YUSUR2_INC_VECTOR is set to 'n' */
__rte_weak int
yusur2_rx_vec_dev_conf_condition_check(struct rte_eth_dev __rte_unused *dev)
{
	return -1;
}

__rte_weak uint16_t
yusur2_recv_pkts_vec(
	void __rte_unused *rx_queue,
	struct rte_mbuf __rte_unused **rx_pkts,
	uint16_t __rte_unused nb_pkts)
{
	return 0;
}

__rte_weak uint16_t
yusur2_recv_scattered_pkts_vec(
	void __rte_unused *rx_queue,
	struct rte_mbuf __rte_unused **rx_pkts,
	uint16_t __rte_unused nb_pkts)
{
	return 0;
}

__rte_weak int
yusur2_rxq_vec_setup(struct yusur2_rx_queue __rte_unused *rxq)
{
	return -1;
}
