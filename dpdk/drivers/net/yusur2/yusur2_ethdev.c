/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include <sys/queue.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <rte_string_fns.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_cycles.h>

#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_bus_pci.h>
#include <rte_branch_prediction.h>
#include <rte_memory.h>
#include <rte_kvargs.h>
#include <rte_eal.h>
#include <rte_alarm.h>
#include <rte_ether.h>
#include <rte_ethdev_driver.h>
#include <rte_ethdev_pci.h>
#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_dev.h>
#include <rte_hash_crc.h>
#ifdef RTE_LIBRTE_SECURITY
#include <rte_security_driver.h>
#endif

#include "yusur2_logs.h"
#include "base/yusur2_api.h"
#include "base/yusur2_vf.h"
#include "base/yusur2_common.h"
#include "yusur2_ethdev.h"
#include "yusur2_bypass.h"
#include "yusur2_rxtx.h"
#include "base/yusur2_type.h"
#include "base/yusur2_phy.h"
#include "yusur2_regs.h"

/*
 * High threshold controlling when to start sending XOFF frames. Must be at
 * least 8 bytes less than receive packet buffer size. This value is in units
 * of 1024 bytes.
 */
#define YUSUR2_FC_HI    0x80

/*
 * Low threshold controlling when to start sending XON frames. This value is
 * in units of 1024 bytes.
 */
#define YUSUR2_FC_LO    0x40

/* Timer value included in XOFF frames. */
#define YUSUR2_FC_PAUSE 0x680

/*Default value of Max Rx Queue*/
#define YUSUR2_MAX_RX_QUEUE_NUM 128

#define YUSUR2_LINK_DOWN_CHECK_TIMEOUT 4000 /* ms */
#define YUSUR2_LINK_UP_CHECK_TIMEOUT   1000 /* ms */
#define YUSUR2_VMDQ_NUM_UC_MAC         4096 /* Maximum nb. of UC MAC addr. */

#define YUSUR2_MMW_SIZE_DEFAULT        0x4
#define YUSUR2_MMW_SIZE_JUMBO_FRAME    0x14
#define YUSUR2_MAX_RING_DESC           4096 /* replicate define from rxtx */

/*
 *  Default values for RX/TX configuration
 */
#define YUSUR2_DEFAULT_RX_FREE_THRESH  32
#define YUSUR2_DEFAULT_RX_PTHRESH      8
#define YUSUR2_DEFAULT_RX_HTHRESH      8
#define YUSUR2_DEFAULT_RX_WTHRESH      0

#define YUSUR2_DEFAULT_TX_FREE_THRESH  32
#define YUSUR2_DEFAULT_TX_PTHRESH      32
#define YUSUR2_DEFAULT_TX_HTHRESH      0
#define YUSUR2_DEFAULT_TX_WTHRESH      0
#define YUSUR2_DEFAULT_TX_RSBIT_THRESH 32

/* Bit shift and mask */
#define YUSUR2_4_BIT_WIDTH  (CHAR_BIT / 2)
#define YUSUR2_4_BIT_MASK   RTE_LEN2MASK(YUSUR2_4_BIT_WIDTH, uint8_t)
#define YUSUR2_8_BIT_WIDTH  CHAR_BIT
#define YUSUR2_8_BIT_MASK   UINT8_MAX

#define YUSUR2VF_PMD_NAME "rte_yusur2vf_pmd" /* PMD name */

#define YUSUR2_QUEUE_STAT_COUNTERS (sizeof(hw_stats->qprc) / sizeof(hw_stats->qprc[0]))

/* Additional timesync values. */
#define NSEC_PER_SEC             1000000000L
#define YUSUR2_INCVAL_10GB        0x66666666
#define YUSUR2_INCVAL_1GB         0x40000000
#define YUSUR2_INCVAL_100         0x50000000
#define YUSUR2_INCVAL_SHIFT_10GB  28
#define YUSUR2_INCVAL_SHIFT_1GB   24
#define YUSUR2_INCVAL_SHIFT_100   21
#define YUSUR2_INCVAL_SHIFT_82599 7
#define YUSUR2_INCPER_SHIFT_82599 24

#define YUSUR2_CYCLECOUNTER_MASK   0xffffffffffffffffULL

#define YUSUR2_VT_CTL_POOLING_MODE_MASK         0x00030000
#define YUSUR2_VT_CTL_POOLING_MODE_ETAG         0x00010000
#define YUSUR2_ETAG_ETYPE                       0x00005084
#define YUSUR2_ETAG_ETYPE_MASK                  0x0000ffff
#define YUSUR2_ETAG_ETYPE_VALID                 0x80000000
#define YUSUR2_RAH_ADTYPE                       0x40000000
#define YUSUR2_RAL_ETAG_FILTER_MASK             0x00003fff
#define YUSUR2_VMVIR_TAGA_MASK                  0x18000000
#define YUSUR2_VMVIR_TAGA_ETAG_INSERT           0x08000000
#define YUSUR2_VMTIR(_i) (0x00017000 + ((_i) * 4)) /* 64 of these (0-63) */
#define YUSUR2_QDE_STRIP_TAG                    0x00000004
#define YUSUR2_VTEICR_MASK                      0x07

#define YUSUR2_EXVET_VET_EXT_SHIFT              16
#define YUSUR2_DMATXCTL_VT_MASK                 0xFFFF0000

#define YUSUR2VF_DEVARG_PFLINK_FULLCHK		"pflink_fullchk"

static const char * const yusur2vf_valid_arguments[] = {
	YUSUR2VF_DEVARG_PFLINK_FULLCHK,
	NULL
};

static int eth_yusur2_dev_init(struct rte_eth_dev *eth_dev, void *init_params);
static int eth_yusur2_dev_uninit(struct rte_eth_dev *eth_dev);
static int yusur2_fdir_filter_init(struct rte_eth_dev *eth_dev);
static int yusur2_fdir_filter_uninit(struct rte_eth_dev *eth_dev);
static int yusur2_l2_tn_filter_init(struct rte_eth_dev *eth_dev);
static int yusur2_l2_tn_filter_uninit(struct rte_eth_dev *eth_dev);
static int yusur2_ntuple_filter_uninit(struct rte_eth_dev *eth_dev);
static int  yusur2_dev_configure(struct rte_eth_dev *dev);
static int  yusur2_dev_start(struct rte_eth_dev *dev);
static void yusur2_dev_stop(struct rte_eth_dev *dev);
static int  yusur2_dev_set_link_up(struct rte_eth_dev *dev);
static int  yusur2_dev_set_link_down(struct rte_eth_dev *dev);
static void yusur2_dev_close(struct rte_eth_dev *dev);
static int  yusur2_dev_reset(struct rte_eth_dev *dev);
static int yusur2_dev_promiscuous_enable(struct rte_eth_dev *dev);
static int yusur2_dev_promiscuous_disable(struct rte_eth_dev *dev);
static int yusur2_dev_allmulticast_enable(struct rte_eth_dev *dev);
static int yusur2_dev_allmulticast_disable(struct rte_eth_dev *dev);
static int yusur2_dev_link_update(struct rte_eth_dev *dev,
				int wait_to_complete);
static int yusur2_dev_stats_get(struct rte_eth_dev *dev,
				struct rte_eth_stats *stats);
static int yusur2_dev_xstats_get(struct rte_eth_dev *dev,
				struct rte_eth_xstat *xstats, unsigned n);
static int yusur2vf_dev_xstats_get(struct rte_eth_dev *dev,
				  struct rte_eth_xstat *xstats, unsigned n);
static int
yusur2_dev_xstats_get_by_id(struct rte_eth_dev *dev, const uint64_t *ids,
		uint64_t *values, unsigned int n);
static int yusur2_dev_stats_reset(struct rte_eth_dev *dev);
static int yusur2_dev_xstats_reset(struct rte_eth_dev *dev);
static int yusur2_dev_xstats_get_names(struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names,
	unsigned int size);
static int yusur2vf_dev_xstats_get_names(struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, unsigned limit);
static int yusur2_dev_xstats_get_names_by_id(
	struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names,
	const uint64_t *ids,
	unsigned int limit);
static int yusur2_dev_queue_stats_mapping_set(struct rte_eth_dev *eth_dev,
					     uint16_t queue_id,
					     uint8_t stat_idx,
					     uint8_t is_rx);
static int yusur2_fw_version_get(struct rte_eth_dev *dev, char *fw_version,
				 size_t fw_size);
static int yusur2_dev_info_get(struct rte_eth_dev *dev,
			      struct rte_eth_dev_info *dev_info);
static const uint32_t *yusur2_dev_supported_ptypes_get(struct rte_eth_dev *dev);
static int yusur2vf_dev_info_get(struct rte_eth_dev *dev,
				struct rte_eth_dev_info *dev_info);
static int yusur2_dev_mtu_set(struct rte_eth_dev *dev, uint16_t mtu);

static int yusur2_vlan_filter_set(struct rte_eth_dev *dev,
		uint16_t vlan_id, int on);
static int yusur2_vlan_tpid_set(struct rte_eth_dev *dev,
			       enum rte_vlan_type vlan_type,
			       uint16_t tpid_id);
static void yusur2_vlan_hw_strip_bitmap_set(struct rte_eth_dev *dev,
		uint16_t queue, bool on);
static void yusur2_vlan_strip_queue_set(struct rte_eth_dev *dev, uint16_t queue,
		int on);
static void yusur2_config_vlan_strip_on_all_queues(struct rte_eth_dev *dev,
						  int mask);
static int yusur2_vlan_offload_config(struct rte_eth_dev *dev, int mask);
static int yusur2_vlan_offload_set(struct rte_eth_dev *dev, int mask);
static void yusur2_vlan_hw_strip_enable(struct rte_eth_dev *dev, uint16_t queue);
static void yusur2_vlan_hw_strip_disable(struct rte_eth_dev *dev, uint16_t queue);
static void yusur2_vlan_hw_extend_enable(struct rte_eth_dev *dev);
static void yusur2_vlan_hw_extend_disable(struct rte_eth_dev *dev);

static int yusur2_dev_led_on(struct rte_eth_dev *dev);
static int yusur2_dev_led_off(struct rte_eth_dev *dev);
static int yusur2_flow_ctrl_get(struct rte_eth_dev *dev,
			       struct rte_eth_fc_conf *fc_conf);
static int yusur2_flow_ctrl_set(struct rte_eth_dev *dev,
			       struct rte_eth_fc_conf *fc_conf);
static int yusur2_priority_flow_ctrl_set(struct rte_eth_dev *dev,
		struct rte_eth_pfc_conf *pfc_conf);
static int yusur2_dev_rss_reta_update(struct rte_eth_dev *dev,
			struct rte_eth_rss_reta_entry64 *reta_conf,
			uint16_t reta_size);
static int yusur2_dev_rss_reta_query(struct rte_eth_dev *dev,
			struct rte_eth_rss_reta_entry64 *reta_conf,
			uint16_t reta_size);
static void yusur2_dev_link_status_print(struct rte_eth_dev *dev);
static int yusur2_dev_lsc_interrupt_setup(struct rte_eth_dev *dev, uint8_t on);
static int yusur2_dev_macsec_interrupt_setup(struct rte_eth_dev *dev);
static int yusur2_dev_rxq_interrupt_setup(struct rte_eth_dev *dev);
static int yusur2_dev_interrupt_get_status(struct rte_eth_dev *dev);
static int yusur2_dev_interrupt_action(struct rte_eth_dev *dev);
static void yusur2_dev_interrupt_handler(void *param);
static void yusur2_dev_interrupt_delayed_handler(void *param);
static void *yusur2_dev_setup_link_thread_handler(void *param);
static int yusur2_dev_wait_setup_link_complete(struct rte_eth_dev *dev,
					      uint32_t timeout_ms);

static int yusur2_add_rar(struct rte_eth_dev *dev,
			struct rte_ether_addr *mac_addr,
			uint32_t index, uint32_t pool);
static void yusur2_remove_rar(struct rte_eth_dev *dev, uint32_t index);
static int yusur2_set_default_mac_addr(struct rte_eth_dev *dev,
					   struct rte_ether_addr *mac_addr);
static void yusur2_dcb_init(struct yusur2_hw *hw, struct yusur2_dcb_config *dcb_config);
static bool is_device_supported(struct rte_eth_dev *dev,
				struct rte_pci_driver *drv);

/* For Virtual Function support */
static int eth_yusur2vf_dev_init(struct rte_eth_dev *eth_dev);
static int eth_yusur2vf_dev_uninit(struct rte_eth_dev *eth_dev);
static int  yusur2vf_dev_configure(struct rte_eth_dev *dev);
static int  yusur2vf_dev_start(struct rte_eth_dev *dev);
static int yusur2vf_dev_link_update(struct rte_eth_dev *dev,
				   int wait_to_complete);
static void yusur2vf_dev_stop(struct rte_eth_dev *dev);
static void yusur2vf_dev_close(struct rte_eth_dev *dev);
static int  yusur2vf_dev_reset(struct rte_eth_dev *dev);
static void yusur2vf_intr_disable(struct rte_eth_dev *dev);
static void yusur2vf_intr_enable(struct rte_eth_dev *dev);
static int yusur2vf_dev_stats_get(struct rte_eth_dev *dev,
		struct rte_eth_stats *stats);
static int yusur2vf_dev_stats_reset(struct rte_eth_dev *dev);
static int yusur2vf_vlan_filter_set(struct rte_eth_dev *dev,
		uint16_t vlan_id, int on);
static void yusur2vf_vlan_strip_queue_set(struct rte_eth_dev *dev,
		uint16_t queue, int on);
static int yusur2vf_vlan_offload_config(struct rte_eth_dev *dev, int mask);
static int yusur2vf_vlan_offload_set(struct rte_eth_dev *dev, int mask);
static void yusur2vf_set_vfta_all(struct rte_eth_dev *dev, bool on);
static int yusur2vf_dev_rx_queue_intr_enable(struct rte_eth_dev *dev,
					    uint16_t queue_id);
static int yusur2vf_dev_rx_queue_intr_disable(struct rte_eth_dev *dev,
					     uint16_t queue_id);
static void yusur2vf_set_ivar_map(struct yusur2_hw *hw, int8_t direction,
				 uint8_t queue, uint8_t msix_vector);
static void yusur2vf_configure_msix(struct rte_eth_dev *dev);
static int yusur2vf_dev_promiscuous_enable(struct rte_eth_dev *dev);
static int yusur2vf_dev_promiscuous_disable(struct rte_eth_dev *dev);
static int yusur2vf_dev_allmulticast_enable(struct rte_eth_dev *dev);
static int yusur2vf_dev_allmulticast_disable(struct rte_eth_dev *dev);

/* For Eth VMDQ APIs support */
static int yusur2_uc_hash_table_set(struct rte_eth_dev *dev, struct
		rte_ether_addr * mac_addr, uint8_t on);
static int yusur2_uc_all_hash_table_set(struct rte_eth_dev *dev, uint8_t on);
static int yusur2_mirror_rule_set(struct rte_eth_dev *dev,
		struct rte_eth_mirror_conf *mirror_conf,
		uint8_t rule_id, uint8_t on);
static int yusur2_mirror_rule_reset(struct rte_eth_dev *dev,
		uint8_t	rule_id);
static int yusur2_dev_rx_queue_intr_enable(struct rte_eth_dev *dev,
					  uint16_t queue_id);
static int yusur2_dev_rx_queue_intr_disable(struct rte_eth_dev *dev,
					   uint16_t queue_id);
static void yusur2_set_ivar_map(struct yusur2_hw *hw, int8_t direction,
			       uint8_t queue, uint8_t msix_vector);
static void yusur2_configure_msix(struct rte_eth_dev *dev);

static int yusur2vf_add_mac_addr(struct rte_eth_dev *dev,
				struct rte_ether_addr *mac_addr,
				uint32_t index, uint32_t pool);
static void yusur2vf_remove_mac_addr(struct rte_eth_dev *dev, uint32_t index);
static int yusur2vf_set_default_mac_addr(struct rte_eth_dev *dev,
					     struct rte_ether_addr *mac_addr);
static int yusur2_syn_filter_get(struct rte_eth_dev *dev,
			struct rte_eth_syn_filter *filter);
static int yusur2_syn_filter_handle(struct rte_eth_dev *dev,
			enum rte_filter_op filter_op,
			void *arg);
static int yusur2_add_5tuple_filter(struct rte_eth_dev *dev,
			struct yusur2_5tuple_filter *filter);
static void yusur2_remove_5tuple_filter(struct rte_eth_dev *dev,
			struct yusur2_5tuple_filter *filter);
static int yusur2_ntuple_filter_handle(struct rte_eth_dev *dev,
				enum rte_filter_op filter_op,
				void *arg);
static int yusur2_get_ntuple_filter(struct rte_eth_dev *dev,
			struct rte_eth_ntuple_filter *filter);
static int yusur2_ethertype_filter_handle(struct rte_eth_dev *dev,
				enum rte_filter_op filter_op,
				void *arg);
static int yusur2_get_ethertype_filter(struct rte_eth_dev *dev,
			struct rte_eth_ethertype_filter *filter);
static int yusur2_dev_filter_ctrl(struct rte_eth_dev *dev,
		     enum rte_filter_type filter_type,
		     enum rte_filter_op filter_op,
		     void *arg);
static int yusur2vf_dev_set_mtu(struct rte_eth_dev *dev, uint16_t mtu);

static int yusur2_dev_set_mc_addr_list(struct rte_eth_dev *dev,
				      struct rte_ether_addr *mc_addr_set,
				      uint32_t nb_mc_addr);
static int yusur2_dev_get_dcb_info(struct rte_eth_dev *dev,
				   struct rte_eth_dcb_info *dcb_info);

static int yusur2_get_reg_length(struct rte_eth_dev *dev);
static int yusur2_get_regs(struct rte_eth_dev *dev,
			    struct rte_dev_reg_info *regs);
static int yusur2_get_eeprom_length(struct rte_eth_dev *dev);
static int yusur2_get_eeprom(struct rte_eth_dev *dev,
				struct rte_dev_eeprom_info *eeprom);
static int yusur2_set_eeprom(struct rte_eth_dev *dev,
				struct rte_dev_eeprom_info *eeprom);

static int yusur2_get_module_info(struct rte_eth_dev *dev,
				 struct rte_eth_dev_module_info *modinfo);
static int yusur2_get_module_eeprom(struct rte_eth_dev *dev,
				   struct rte_dev_eeprom_info *info);

static int yusur2vf_get_reg_length(struct rte_eth_dev *dev);
static int yusur2vf_get_regs(struct rte_eth_dev *dev,
				struct rte_dev_reg_info *regs);

static int yusur2_timesync_enable(struct rte_eth_dev *dev);
static int yusur2_timesync_disable(struct rte_eth_dev *dev);
static int yusur2_timesync_read_rx_timestamp(struct rte_eth_dev *dev,
					    struct timespec *timestamp,
					    uint32_t flags);
static int yusur2_timesync_read_tx_timestamp(struct rte_eth_dev *dev,
					    struct timespec *timestamp);
static int yusur2_timesync_adjust_time(struct rte_eth_dev *dev, int64_t delta);
static int yusur2_timesync_read_time(struct rte_eth_dev *dev,
				   struct timespec *timestamp);
static int yusur2_timesync_write_time(struct rte_eth_dev *dev,
				   const struct timespec *timestamp);
static void yusur2vf_dev_interrupt_handler(void *param);

static int yusur2_dev_l2_tunnel_eth_type_conf
	(struct rte_eth_dev *dev, struct rte_eth_l2_tunnel_conf *l2_tunnel);
static int yusur2_dev_l2_tunnel_offload_set
	(struct rte_eth_dev *dev,
	 struct rte_eth_l2_tunnel_conf *l2_tunnel,
	 uint32_t mask,
	 uint8_t en);
static int yusur2_dev_l2_tunnel_filter_handle(struct rte_eth_dev *dev,
					     enum rte_filter_op filter_op,
					     void *arg);

static int yusur2_dev_udp_tunnel_port_add(struct rte_eth_dev *dev,
					 struct rte_eth_udp_tunnel *udp_tunnel);
static int yusur2_dev_udp_tunnel_port_del(struct rte_eth_dev *dev,
					 struct rte_eth_udp_tunnel *udp_tunnel);
static int yusur2_filter_restore(struct rte_eth_dev *dev);
static void yusur2_l2_tunnel_conf(struct rte_eth_dev *dev);
static int yusur2_wait_for_link_up(struct yusur2_hw *hw);

/*
 * Define VF Stats MACRO for Non "cleared on read" register
 */
#define UPDATE_VF_STAT(reg, last, cur)                          \
{                                                               \
	uint32_t latest = YUSUR2_READ_REG(hw, reg);              \
	cur += (latest - last) & UINT_MAX;                      \
	last = latest;                                          \
}

#define UPDATE_VF_STAT_36BIT(lsb, msb, last, cur)                \
{                                                                \
	u64 new_lsb = YUSUR2_READ_REG(hw, lsb);                   \
	u64 new_msb = YUSUR2_READ_REG(hw, msb);                   \
	u64 latest = ((new_msb << 32) | new_lsb);                \
	cur += (0x1000000000LL + latest - last) & 0xFFFFFFFFFLL; \
	last = latest;                                           \
}

#define YUSUR2_SET_HWSTRIP(h, q) do {\
		uint32_t idx = (q) / (sizeof((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof((h)->bitmap[0]) * NBBY); \
		(h)->bitmap[idx] |= 1 << bit;\
	} while (0)

#define YUSUR2_CLEAR_HWSTRIP(h, q) do {\
		uint32_t idx = (q) / (sizeof((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof((h)->bitmap[0]) * NBBY); \
		(h)->bitmap[idx] &= ~(1 << bit);\
	} while (0)

#define YUSUR2_GET_HWSTRIP(h, q, r) do {\
		uint32_t idx = (q) / (sizeof((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof((h)->bitmap[0]) * NBBY); \
		(r) = (h)->bitmap[idx] >> bit & 1;\
	} while (0)

int yusur2_logtype_init;
int yusur2_logtype_driver;

#ifdef RTE_LIBRTE_YUSUR2_DEBUG_RX
int yusur2_logtype_rx;
#endif
#ifdef RTE_LIBRTE_YUSUR2_DEBUG_TX
int yusur2_logtype_tx;
#endif
#ifdef RTE_LIBRTE_YUSUR2_DEBUG_TX_FREE
int yusur2_logtype_tx_free;
#endif

/*
 * The set of PCI devices this driver supports
 */
static const struct rte_pci_id pci_id_yusur2_map[] = {
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598_BX) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598AF_DUAL_PORT) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598AF_SINGLE_PORT) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598AT) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598AT2) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598EB_SFP_LOM) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598EB_CX4) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598_CX4_DUAL_PORT) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598_DA_DUAL_PORT) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598_SR_DUAL_PORT_EM) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82598EB_XF_LR) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_KX4) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_KX4_MEZZ) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_KR) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_COMBO_BACKPLANE) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_CX4) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_SFP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_BACKPLANE_FCOE) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_SFP_FCOE) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_SFP_EM) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_SFP_SF2) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_SFP_SF_QP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_QSFP_SF_QP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599EN_SFP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_XAUI_LOM) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_T3_LOM) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X540T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X540T1) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_SFP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_10G_T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_1G_T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550T1) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_KR) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_KR_L) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_SFP_N) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_SGMII) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_SGMII_L) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_10G_T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_QSFP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_QSFP_N) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_SFP) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_1G_T) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_1G_T_L) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_KX4) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_KR) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_XFI) },
#ifdef RTE_LIBRTE_YUSUR2_BYPASS
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_BYPASS) },
#endif
	{ .vendor_id = 0, /* sentinel */ },
};

/*
 * The set of PCI devices this driver supports (for 82599 VF)
 */
static const struct rte_pci_id pci_id_yusur2vf_map[] = {
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_VF) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_82599_VF_HV) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X540_VF) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X540_VF_HV) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550_VF_HV) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550_VF) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_VF) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_A_VF_HV) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_VF) },
	{ RTE_PCI_DEVICE(YUSUR2_INTEL_VENDOR_ID, YUSUR2_DEV_ID_X550EM_X_VF_HV) },
	{ .vendor_id = 0, /* sentinel */ },
};

static const struct rte_eth_desc_lim rx_desc_lim = {
	.nb_max = YUSUR2_MAX_RING_DESC,
	.nb_min = YUSUR2_MIN_RING_DESC,
	.nb_align = YUSUR2_RXD_ALIGN,
};

static const struct rte_eth_desc_lim tx_desc_lim = {
	.nb_max = YUSUR2_MAX_RING_DESC,
	.nb_min = YUSUR2_MIN_RING_DESC,
	.nb_align = YUSUR2_TXD_ALIGN,
	.nb_seg_max = YUSUR2_TX_MAX_SEG,
	.nb_mtu_seg_max = YUSUR2_TX_MAX_SEG,
};

static const struct eth_dev_ops yusur2_eth_dev_ops = {
	.dev_configure        = yusur2_dev_configure,
	.dev_start            = yusur2_dev_start,
	.dev_stop             = yusur2_dev_stop,
	.dev_set_link_up    = yusur2_dev_set_link_up,
	.dev_set_link_down  = yusur2_dev_set_link_down,
	.dev_close            = yusur2_dev_close,
	.dev_reset	      = yusur2_dev_reset,
	.promiscuous_enable   = yusur2_dev_promiscuous_enable,
	.promiscuous_disable  = yusur2_dev_promiscuous_disable,
	.allmulticast_enable  = yusur2_dev_allmulticast_enable,
	.allmulticast_disable = yusur2_dev_allmulticast_disable,
	.link_update          = yusur2_dev_link_update,
	.stats_get            = yusur2_dev_stats_get,
	.xstats_get           = yusur2_dev_xstats_get,
	.xstats_get_by_id     = yusur2_dev_xstats_get_by_id,
	.stats_reset          = yusur2_dev_stats_reset,
	.xstats_reset         = yusur2_dev_xstats_reset,
	.xstats_get_names     = yusur2_dev_xstats_get_names,
	.xstats_get_names_by_id = yusur2_dev_xstats_get_names_by_id,
	.queue_stats_mapping_set = yusur2_dev_queue_stats_mapping_set,
	.fw_version_get       = yusur2_fw_version_get,
	.dev_infos_get        = yusur2_dev_info_get,
	.dev_supported_ptypes_get = yusur2_dev_supported_ptypes_get,
	.mtu_set              = yusur2_dev_mtu_set,
	.vlan_filter_set      = yusur2_vlan_filter_set,
	.vlan_tpid_set        = yusur2_vlan_tpid_set,
	.vlan_offload_set     = yusur2_vlan_offload_set,
	.vlan_strip_queue_set = yusur2_vlan_strip_queue_set,
	.rx_queue_start	      = yusur2_dev_rx_queue_start,
	.rx_queue_stop        = yusur2_dev_rx_queue_stop,
	.tx_queue_start	      = yusur2_dev_tx_queue_start,
	.tx_queue_stop        = yusur2_dev_tx_queue_stop,
	.rx_queue_setup       = yusur2_dev_rx_queue_setup,
	.rx_queue_intr_enable = yusur2_dev_rx_queue_intr_enable,
	.rx_queue_intr_disable = yusur2_dev_rx_queue_intr_disable,
	.rx_queue_release     = yusur2_dev_rx_queue_release,
	.rx_queue_count       = yusur2_dev_rx_queue_count,
	.rx_descriptor_done   = yusur2_dev_rx_descriptor_done,
	.rx_descriptor_status = yusur2_dev_rx_descriptor_status,
	.tx_descriptor_status = yusur2_dev_tx_descriptor_status,
	.tx_queue_setup       = yusur2_dev_tx_queue_setup,
	.tx_queue_release     = yusur2_dev_tx_queue_release,
	.dev_led_on           = yusur2_dev_led_on,
	.dev_led_off          = yusur2_dev_led_off,
	.flow_ctrl_get        = yusur2_flow_ctrl_get,
	.flow_ctrl_set        = yusur2_flow_ctrl_set,
	.priority_flow_ctrl_set = yusur2_priority_flow_ctrl_set,
	.mac_addr_add         = yusur2_add_rar,
	.mac_addr_remove      = yusur2_remove_rar,
	.mac_addr_set         = yusur2_set_default_mac_addr,
	.uc_hash_table_set    = yusur2_uc_hash_table_set,
	.uc_all_hash_table_set  = yusur2_uc_all_hash_table_set,
	.mirror_rule_set      = yusur2_mirror_rule_set,
	.mirror_rule_reset    = yusur2_mirror_rule_reset,
	.set_queue_rate_limit = yusur2_set_queue_rate_limit,
	.reta_update          = yusur2_dev_rss_reta_update,
	.reta_query           = yusur2_dev_rss_reta_query,
	.rss_hash_update      = yusur2_dev_rss_hash_update,
	.rss_hash_conf_get    = yusur2_dev_rss_hash_conf_get,
	.filter_ctrl          = yusur2_dev_filter_ctrl,
	.set_mc_addr_list     = yusur2_dev_set_mc_addr_list,
	.rxq_info_get         = yusur2_rxq_info_get,
	.txq_info_get         = yusur2_txq_info_get,
	.timesync_enable      = yusur2_timesync_enable,
	.timesync_disable     = yusur2_timesync_disable,
	.timesync_read_rx_timestamp = yusur2_timesync_read_rx_timestamp,
	.timesync_read_tx_timestamp = yusur2_timesync_read_tx_timestamp,
	.get_reg              = yusur2_get_regs,
	.get_eeprom_length    = yusur2_get_eeprom_length,
	.get_eeprom           = yusur2_get_eeprom,
	.set_eeprom           = yusur2_set_eeprom,
	.get_module_info      = yusur2_get_module_info,
	.get_module_eeprom    = yusur2_get_module_eeprom,
	.get_dcb_info         = yusur2_dev_get_dcb_info,
	.timesync_adjust_time = yusur2_timesync_adjust_time,
	.timesync_read_time   = yusur2_timesync_read_time,
	.timesync_write_time  = yusur2_timesync_write_time,
	.l2_tunnel_eth_type_conf = yusur2_dev_l2_tunnel_eth_type_conf,
	.l2_tunnel_offload_set   = yusur2_dev_l2_tunnel_offload_set,
	.udp_tunnel_port_add  = yusur2_dev_udp_tunnel_port_add,
	.udp_tunnel_port_del  = yusur2_dev_udp_tunnel_port_del,
	.tm_ops_get           = yusur2_tm_ops_get,
};

/*
 * dev_ops for virtual function, bare necessities for basic vf
 * operation have been implemented
 */
static const struct eth_dev_ops yusur2vf_eth_dev_ops = {
	.dev_configure        = yusur2vf_dev_configure,
	.dev_start            = yusur2vf_dev_start,
	.dev_stop             = yusur2vf_dev_stop,
	.link_update          = yusur2vf_dev_link_update,
	.stats_get            = yusur2vf_dev_stats_get,
	.xstats_get           = yusur2vf_dev_xstats_get,
	.stats_reset          = yusur2vf_dev_stats_reset,
	.xstats_reset         = yusur2vf_dev_stats_reset,
	.xstats_get_names     = yusur2vf_dev_xstats_get_names,
	.dev_close            = yusur2vf_dev_close,
	.dev_reset	      = yusur2vf_dev_reset,
	.promiscuous_enable   = yusur2vf_dev_promiscuous_enable,
	.promiscuous_disable  = yusur2vf_dev_promiscuous_disable,
	.allmulticast_enable  = yusur2vf_dev_allmulticast_enable,
	.allmulticast_disable = yusur2vf_dev_allmulticast_disable,
	.dev_infos_get        = yusur2vf_dev_info_get,
	.dev_supported_ptypes_get = yusur2_dev_supported_ptypes_get,
	.mtu_set              = yusur2vf_dev_set_mtu,
	.vlan_filter_set      = yusur2vf_vlan_filter_set,
	.vlan_strip_queue_set = yusur2vf_vlan_strip_queue_set,
	.vlan_offload_set     = yusur2vf_vlan_offload_set,
	.rx_queue_setup       = yusur2_dev_rx_queue_setup,
	.rx_queue_release     = yusur2_dev_rx_queue_release,
	.rx_descriptor_done   = yusur2_dev_rx_descriptor_done,
	.rx_descriptor_status = yusur2_dev_rx_descriptor_status,
	.tx_descriptor_status = yusur2_dev_tx_descriptor_status,
	.tx_queue_setup       = yusur2_dev_tx_queue_setup,
	.tx_queue_release     = yusur2_dev_tx_queue_release,
	.rx_queue_intr_enable = yusur2vf_dev_rx_queue_intr_enable,
	.rx_queue_intr_disable = yusur2vf_dev_rx_queue_intr_disable,
	.mac_addr_add         = yusur2vf_add_mac_addr,
	.mac_addr_remove      = yusur2vf_remove_mac_addr,
	.set_mc_addr_list     = yusur2_dev_set_mc_addr_list,
	.rxq_info_get         = yusur2_rxq_info_get,
	.txq_info_get         = yusur2_txq_info_get,
	.mac_addr_set         = yusur2vf_set_default_mac_addr,
	.get_reg              = yusur2vf_get_regs,
	.reta_update          = yusur2_dev_rss_reta_update,
	.reta_query           = yusur2_dev_rss_reta_query,
	.rss_hash_update      = yusur2_dev_rss_hash_update,
	.rss_hash_conf_get    = yusur2_dev_rss_hash_conf_get,
};

/* store statistics names and its offset in stats structure */
struct rte_yusur2_xstats_name_off {
	char name[RTE_ETH_XSTATS_NAME_SIZE];
	unsigned offset;
};

static const struct rte_yusur2_xstats_name_off rte_yusur2_stats_strings[] = {
	{"rx_crc_errors", offsetof(struct yusur2_hw_stats, crcerrs)},
	{"rx_illegal_byte_errors", offsetof(struct yusur2_hw_stats, illerrc)},
	{"rx_error_bytes", offsetof(struct yusur2_hw_stats, errbc)},
	{"mac_local_errors", offsetof(struct yusur2_hw_stats, mlfc)},
	{"mac_remote_errors", offsetof(struct yusur2_hw_stats, mrfc)},
	{"rx_length_errors", offsetof(struct yusur2_hw_stats, rlec)},
	{"tx_xon_packets", offsetof(struct yusur2_hw_stats, lxontxc)},
	{"rx_xon_packets", offsetof(struct yusur2_hw_stats, lxonrxc)},
	{"tx_xoff_packets", offsetof(struct yusur2_hw_stats, lxofftxc)},
	{"rx_xoff_packets", offsetof(struct yusur2_hw_stats, lxoffrxc)},
	{"rx_size_64_packets", offsetof(struct yusur2_hw_stats, prc64)},
	{"rx_size_65_to_127_packets", offsetof(struct yusur2_hw_stats, prc127)},
	{"rx_size_128_to_255_packets", offsetof(struct yusur2_hw_stats, prc255)},
	{"rx_size_256_to_511_packets", offsetof(struct yusur2_hw_stats, prc511)},
	{"rx_size_512_to_1023_packets", offsetof(struct yusur2_hw_stats,
		prc1023)},
	{"rx_size_1024_to_max_packets", offsetof(struct yusur2_hw_stats,
		prc1522)},
	{"rx_broadcast_packets", offsetof(struct yusur2_hw_stats, bprc)},
	{"rx_multicast_packets", offsetof(struct yusur2_hw_stats, mprc)},
	{"rx_fragment_errors", offsetof(struct yusur2_hw_stats, rfc)},
	{"rx_undersize_errors", offsetof(struct yusur2_hw_stats, ruc)},
	{"rx_oversize_errors", offsetof(struct yusur2_hw_stats, roc)},
	{"rx_jabber_errors", offsetof(struct yusur2_hw_stats, rjc)},
	{"rx_management_packets", offsetof(struct yusur2_hw_stats, mngprc)},
	{"rx_management_dropped", offsetof(struct yusur2_hw_stats, mngpdc)},
	{"tx_management_packets", offsetof(struct yusur2_hw_stats, mngptc)},
	{"rx_total_packets", offsetof(struct yusur2_hw_stats, tpr)},
	{"rx_total_bytes", offsetof(struct yusur2_hw_stats, tor)},
	{"tx_total_packets", offsetof(struct yusur2_hw_stats, tpt)},
	{"tx_size_64_packets", offsetof(struct yusur2_hw_stats, ptc64)},
	{"tx_size_65_to_127_packets", offsetof(struct yusur2_hw_stats, ptc127)},
	{"tx_size_128_to_255_packets", offsetof(struct yusur2_hw_stats, ptc255)},
	{"tx_size_256_to_511_packets", offsetof(struct yusur2_hw_stats, ptc511)},
	{"tx_size_512_to_1023_packets", offsetof(struct yusur2_hw_stats,
		ptc1023)},
	{"tx_size_1024_to_max_packets", offsetof(struct yusur2_hw_stats,
		ptc1522)},
	{"tx_multicast_packets", offsetof(struct yusur2_hw_stats, mptc)},
	{"tx_broadcast_packets", offsetof(struct yusur2_hw_stats, bptc)},
	{"rx_mac_short_packet_dropped", offsetof(struct yusur2_hw_stats, mspdc)},
	{"rx_l3_l4_xsum_error", offsetof(struct yusur2_hw_stats, xec)},

	{"flow_director_added_filters", offsetof(struct yusur2_hw_stats,
		fdirustat_add)},
	{"flow_director_removed_filters", offsetof(struct yusur2_hw_stats,
		fdirustat_remove)},
	{"flow_director_filter_add_errors", offsetof(struct yusur2_hw_stats,
		fdirfstat_fadd)},
	{"flow_director_filter_remove_errors", offsetof(struct yusur2_hw_stats,
		fdirfstat_fremove)},
	{"flow_director_matched_filters", offsetof(struct yusur2_hw_stats,
		fdirmatch)},
	{"flow_director_missed_filters", offsetof(struct yusur2_hw_stats,
		fdirmiss)},

	{"rx_fcoe_crc_errors", offsetof(struct yusur2_hw_stats, fccrc)},
	{"rx_fcoe_dropped", offsetof(struct yusur2_hw_stats, fcoerpdc)},
	{"rx_fcoe_mbuf_allocation_errors", offsetof(struct yusur2_hw_stats,
		fclast)},
	{"rx_fcoe_packets", offsetof(struct yusur2_hw_stats, fcoeprc)},
	{"tx_fcoe_packets", offsetof(struct yusur2_hw_stats, fcoeptc)},
	{"rx_fcoe_bytes", offsetof(struct yusur2_hw_stats, fcoedwrc)},
	{"tx_fcoe_bytes", offsetof(struct yusur2_hw_stats, fcoedwtc)},
	{"rx_fcoe_no_direct_data_placement", offsetof(struct yusur2_hw_stats,
		fcoe_noddp)},
	{"rx_fcoe_no_direct_data_placement_ext_buff",
		offsetof(struct yusur2_hw_stats, fcoe_noddp_ext_buff)},

	{"tx_flow_control_xon_packets", offsetof(struct yusur2_hw_stats,
		lxontxc)},
	{"rx_flow_control_xon_packets", offsetof(struct yusur2_hw_stats,
		lxonrxc)},
	{"tx_flow_control_xoff_packets", offsetof(struct yusur2_hw_stats,
		lxofftxc)},
	{"rx_flow_control_xoff_packets", offsetof(struct yusur2_hw_stats,
		lxoffrxc)},
	{"rx_total_missed_packets", offsetof(struct yusur2_hw_stats, mpctotal)},
};

#define YUSUR2_NB_HW_STATS (sizeof(rte_yusur2_stats_strings) / \
			   sizeof(rte_yusur2_stats_strings[0]))

/* MACsec statistics */
static const struct rte_yusur2_xstats_name_off rte_yusur2_macsec_strings[] = {
	{"out_pkts_untagged", offsetof(struct yusur2_macsec_stats,
		out_pkts_untagged)},
	{"out_pkts_encrypted", offsetof(struct yusur2_macsec_stats,
		out_pkts_encrypted)},
	{"out_pkts_protected", offsetof(struct yusur2_macsec_stats,
		out_pkts_protected)},
	{"out_octets_encrypted", offsetof(struct yusur2_macsec_stats,
		out_octets_encrypted)},
	{"out_octets_protected", offsetof(struct yusur2_macsec_stats,
		out_octets_protected)},
	{"in_pkts_untagged", offsetof(struct yusur2_macsec_stats,
		in_pkts_untagged)},
	{"in_pkts_badtag", offsetof(struct yusur2_macsec_stats,
		in_pkts_badtag)},
	{"in_pkts_nosci", offsetof(struct yusur2_macsec_stats,
		in_pkts_nosci)},
	{"in_pkts_unknownsci", offsetof(struct yusur2_macsec_stats,
		in_pkts_unknownsci)},
	{"in_octets_decrypted", offsetof(struct yusur2_macsec_stats,
		in_octets_decrypted)},
	{"in_octets_validated", offsetof(struct yusur2_macsec_stats,
		in_octets_validated)},
	{"in_pkts_unchecked", offsetof(struct yusur2_macsec_stats,
		in_pkts_unchecked)},
	{"in_pkts_delayed", offsetof(struct yusur2_macsec_stats,
		in_pkts_delayed)},
	{"in_pkts_late", offsetof(struct yusur2_macsec_stats,
		in_pkts_late)},
	{"in_pkts_ok", offsetof(struct yusur2_macsec_stats,
		in_pkts_ok)},
	{"in_pkts_invalid", offsetof(struct yusur2_macsec_stats,
		in_pkts_invalid)},
	{"in_pkts_notvalid", offsetof(struct yusur2_macsec_stats,
		in_pkts_notvalid)},
	{"in_pkts_unusedsa", offsetof(struct yusur2_macsec_stats,
		in_pkts_unusedsa)},
	{"in_pkts_notusingsa", offsetof(struct yusur2_macsec_stats,
		in_pkts_notusingsa)},
};

#define YUSUR2_NB_MACSEC_STATS (sizeof(rte_yusur2_macsec_strings) / \
			   sizeof(rte_yusur2_macsec_strings[0]))

/* Per-queue statistics */
static const struct rte_yusur2_xstats_name_off rte_yusur2_rxq_strings[] = {
	{"mbuf_allocation_errors", offsetof(struct yusur2_hw_stats, rnbc)},
	{"dropped", offsetof(struct yusur2_hw_stats, mpc)},
	{"xon_packets", offsetof(struct yusur2_hw_stats, pxonrxc)},
	{"xoff_packets", offsetof(struct yusur2_hw_stats, pxoffrxc)},
};

#define YUSUR2_NB_RXQ_PRIO_STATS (sizeof(rte_yusur2_rxq_strings) / \
			   sizeof(rte_yusur2_rxq_strings[0]))
#define YUSUR2_NB_RXQ_PRIO_VALUES 8

static const struct rte_yusur2_xstats_name_off rte_yusur2_txq_strings[] = {
	{"xon_packets", offsetof(struct yusur2_hw_stats, pxontxc)},
	{"xoff_packets", offsetof(struct yusur2_hw_stats, pxofftxc)},
	{"xon_to_xoff_packets", offsetof(struct yusur2_hw_stats,
		pxon2offc)},
};

#define YUSUR2_NB_TXQ_PRIO_STATS (sizeof(rte_yusur2_txq_strings) / \
			   sizeof(rte_yusur2_txq_strings[0]))
#define YUSUR2_NB_TXQ_PRIO_VALUES 8

static const struct rte_yusur2_xstats_name_off rte_yusur2vf_stats_strings[] = {
	{"rx_multicast_packets", offsetof(struct yusur2vf_hw_stats, vfmprc)},
};

#define YUSUR2VF_NB_XSTATS (sizeof(rte_yusur2vf_stats_strings) /	\
		sizeof(rte_yusur2vf_stats_strings[0]))

/*
 * This function is the same as yusur2_is_sfp() in base/yusur2.h.
 */
static inline int
yusur2_is_sfp(struct yusur2_hw *hw)
{
	switch (hw->phy.type) {
	case yusur2_phy_sfp_avago:
	case yusur2_phy_sfp_ftl:
	case yusur2_phy_sfp_intel:
	case yusur2_phy_sfp_unknown:
	case yusur2_phy_sfp_passive_tyco:
	case yusur2_phy_sfp_passive_unknown:
		return 1;
	default:
		return 0;
	}
}

static inline int32_t
yusur2_pf_reset_hw(struct yusur2_hw *hw)
{
	uint32_t ctrl_ext;
	int32_t status;

	status = yusur2_reset_hw(hw);

	ctrl_ext = YUSUR2_READ_REG(hw, YUSUR2_CTRL_EXT);
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
	ctrl_ext |= YUSUR2_CTRL_EXT_PFRSTD;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL_EXT, ctrl_ext);
	YUSUR2_WRITE_FLUSH(hw);

	if (status == YUSUR2_ERR_SFP_NOT_PRESENT)
		status = YUSUR2_SUCCESS;
	return status;
}

static inline void
yusur2_enable_intr(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	YUSUR2_WRITE_REG(hw, YUSUR2_EIMS, intr->mask);
	YUSUR2_WRITE_FLUSH(hw);
}

/*
 * This function is based on yusur2_disable_intr() in base/yusur2.h.
 */
static void
yusur2_disable_intr(struct yusur2_hw *hw)
{
	PMD_INIT_FUNC_TRACE();

	if (hw->mac.type == yusur2_mac_82598EB) {
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMC, ~0);
	} else {
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMC, 0xFFFF0000);
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMC_EX(0), ~0);
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMC_EX(1), ~0);
	}
	YUSUR2_WRITE_FLUSH(hw);
}

/*
 * This function resets queue statistics mapping registers.
 * From Niantic datasheet, Initialization of Statistics section:
 * "...if software requires the queue counters, the RQSMR and TQSM registers
 * must be re-programmed following a device reset.
 */
static void
yusur2_reset_qstat_mappings(struct yusur2_hw *hw)
{
	uint32_t i;

	for (i = 0; i != YUSUR2_NB_STAT_MAPPING_REGS; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), 0);
		YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(i), 0);
	}
}


static int
yusur2_dev_queue_stats_mapping_set(struct rte_eth_dev *eth_dev,
				  uint16_t queue_id,
				  uint8_t stat_idx,
				  uint8_t is_rx)
{
#define QSM_REG_NB_BITS_PER_QMAP_FIELD 8
#define NB_QMAP_FIELDS_PER_QSM_REG 4
#define QMAP_FIELD_RESERVED_BITS_MASK 0x0f

	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	struct yusur2_stat_mapping_registers *stat_mappings =
		YUSUR2_DEV_PRIVATE_TO_STAT_MAPPINGS(eth_dev->data->dev_private);
	uint32_t qsmr_mask = 0;
	uint32_t clearing_mask = QMAP_FIELD_RESERVED_BITS_MASK;
	uint32_t q_map;
	uint8_t n, offset;

	if ((hw->mac.type != yusur2_mac_82599EB) &&
		(hw->mac.type != yusur2_mac_X540) &&
		(hw->mac.type != yusur2_mac_X550) &&
		(hw->mac.type != yusur2_mac_X550EM_x) &&
		(hw->mac.type != yusur2_mac_X550EM_a))
		return -ENOSYS;

	PMD_INIT_LOG(DEBUG, "Setting port %d, %s queue_id %d to stat index %d",
		     (int)(eth_dev->data->port_id), is_rx ? "RX" : "TX",
		     queue_id, stat_idx);

	n = (uint8_t)(queue_id / NB_QMAP_FIELDS_PER_QSM_REG);
	if (n >= YUSUR2_NB_STAT_MAPPING_REGS) {
		PMD_INIT_LOG(ERR, "Nb of stat mapping registers exceeded");
		return -EIO;
	}
	offset = (uint8_t)(queue_id % NB_QMAP_FIELDS_PER_QSM_REG);

	/* Now clear any previous stat_idx set */
	clearing_mask <<= (QSM_REG_NB_BITS_PER_QMAP_FIELD * offset);
	if (!is_rx)
		stat_mappings->tqsm[n] &= ~clearing_mask;
	else
		stat_mappings->rqsmr[n] &= ~clearing_mask;

	q_map = (uint32_t)stat_idx;
	q_map &= QMAP_FIELD_RESERVED_BITS_MASK;
	qsmr_mask = q_map << (QSM_REG_NB_BITS_PER_QMAP_FIELD * offset);
	if (!is_rx)
		stat_mappings->tqsm[n] |= qsmr_mask;
	else
		stat_mappings->rqsmr[n] |= qsmr_mask;

	PMD_INIT_LOG(DEBUG, "Set port %d, %s queue_id %d to stat index %d",
		     (int)(eth_dev->data->port_id), is_rx ? "RX" : "TX",
		     queue_id, stat_idx);
	PMD_INIT_LOG(DEBUG, "%s[%d] = 0x%08x", is_rx ? "RQSMR" : "TQSM", n,
		     is_rx ? stat_mappings->rqsmr[n] : stat_mappings->tqsm[n]);

	/* Now write the mapping in the appropriate register */
	if (is_rx) {
		PMD_INIT_LOG(DEBUG, "Write 0x%x to RX YUSUR2 stat mapping reg:%d",
			     stat_mappings->rqsmr[n], n);
		YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(n), stat_mappings->rqsmr[n]);
	} else {
		PMD_INIT_LOG(DEBUG, "Write 0x%x to TX YUSUR2 stat mapping reg:%d",
			     stat_mappings->tqsm[n], n);
		YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(n), stat_mappings->tqsm[n]);
	}
	return 0;
}

static void
yusur2_restore_statistics_mapping(struct rte_eth_dev *dev)
{
	struct yusur2_stat_mapping_registers *stat_mappings =
		YUSUR2_DEV_PRIVATE_TO_STAT_MAPPINGS(dev->data->dev_private);
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;

	/* write whatever was in stat mapping table to the NIC */
	for (i = 0; i < YUSUR2_NB_STAT_MAPPING_REGS; i++) {
		/* rx */
		YUSUR2_WRITE_REG(hw, YUSUR2_RQSMR(i), stat_mappings->rqsmr[i]);

		/* tx */
		YUSUR2_WRITE_REG(hw, YUSUR2_TQSM(i), stat_mappings->tqsm[i]);
	}
}

static void
yusur2_dcb_init(struct yusur2_hw *hw, struct yusur2_dcb_config *dcb_config)
{
	uint8_t i;
	struct yusur2_dcb_tc_config *tc;
	uint8_t dcb_max_tc = YUSUR2_DCB_MAX_TRAFFIC_CLASS;

	dcb_config->num_tcs.pg_tcs = dcb_max_tc;
	dcb_config->num_tcs.pfc_tcs = dcb_max_tc;
	for (i = 0; i < dcb_max_tc; i++) {
		tc = &dcb_config->tc_config[i];
		tc->path[YUSUR2_DCB_TX_CONFIG].bwg_id = i;
		tc->path[YUSUR2_DCB_TX_CONFIG].bwg_percent =
				 (uint8_t)(100/dcb_max_tc + (i & 1));
		tc->path[YUSUR2_DCB_RX_CONFIG].bwg_id = i;
		tc->path[YUSUR2_DCB_RX_CONFIG].bwg_percent =
				 (uint8_t)(100/dcb_max_tc + (i & 1));
		tc->pfc = yusur2_dcb_pfc_disabled;
	}

	/* Initialize default user to priority mapping, UPx->TC0 */
	tc = &dcb_config->tc_config[0];
	tc->path[YUSUR2_DCB_TX_CONFIG].up_to_tc_bitmap = 0xFF;
	tc->path[YUSUR2_DCB_RX_CONFIG].up_to_tc_bitmap = 0xFF;
	for (i = 0; i < YUSUR2_DCB_MAX_BW_GROUP; i++) {
		dcb_config->bw_percentage[YUSUR2_DCB_TX_CONFIG][i] = 100;
		dcb_config->bw_percentage[YUSUR2_DCB_RX_CONFIG][i] = 100;
	}
	dcb_config->rx_pba_cfg = yusur2_dcb_pba_equal;
	dcb_config->pfc_mode_enable = false;
	dcb_config->vt_mode = true;
	dcb_config->round_robin_enable = false;
	/* support all DCB capabilities in 82599 */
	dcb_config->support.capabilities = 0xFF;

	/*we only support 4 Tcs for X540, X550 */
	if (hw->mac.type == yusur2_mac_X540 ||
		hw->mac.type == yusur2_mac_X550 ||
		hw->mac.type == yusur2_mac_X550EM_x ||
		hw->mac.type == yusur2_mac_X550EM_a) {
		dcb_config->num_tcs.pg_tcs = 4;
		dcb_config->num_tcs.pfc_tcs = 4;
	}
}

/*
 * Ensure that all locks are released before first NVM or PHY access
 */
static void
yusur2_swfw_lock_reset(struct yusur2_hw *hw)
{
	uint16_t mask;

	/*
	 * Phy lock should not fail in this early stage. If this is the case,
	 * it is due to an improper exit of the application.
	 * So force the release of the faulty lock. Release of common lock
	 * is done automatically by swfw_sync function.
	 */
	mask = YUSUR2_GSSR_PHY0_SM << hw->bus.func;
	if (yusur2_acquire_swfw_semaphore(hw, mask) < 0) {
		PMD_DRV_LOG(DEBUG, "SWFW phy%d lock released", hw->bus.func);
	}
	yusur2_release_swfw_semaphore(hw, mask);

	/*
	 * These ones are more tricky since they are common to all ports; but
	 * swfw_sync retries last long enough (1s) to be almost sure that if
	 * lock can not be taken it is due to an improper lock of the
	 * semaphore.
	 */
	mask = YUSUR2_GSSR_EEP_SM | YUSUR2_GSSR_MAC_CSR_SM | YUSUR2_GSSR_SW_MNG_SM;
	if (yusur2_acquire_swfw_semaphore(hw, mask) < 0) {
		PMD_DRV_LOG(DEBUG, "SWFW common locks released");
	}
	yusur2_release_swfw_semaphore(hw, mask);
}

/*
 * This function is based on code in yusur2_attach() in base/yusur2.c.
 * It returns 0 on success.
 */
static int
eth_yusur2_dev_init(struct rte_eth_dev *eth_dev, void *init_params __rte_unused)
{
	struct yusur2_adapter *ad = eth_dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(eth_dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(eth_dev->data->dev_private);
	struct yusur2_hwstrip *hwstrip =
		YUSUR2_DEV_PRIVATE_TO_HWSTRIP_BITMAP(eth_dev->data->dev_private);
	struct yusur2_dcb_config *dcb_config =
		YUSUR2_DEV_PRIVATE_TO_DCB_CFG(eth_dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(eth_dev->data->dev_private);
	struct yusur2_bw_conf *bw_conf =
		YUSUR2_DEV_PRIVATE_TO_BW_CONF(eth_dev->data->dev_private);
	uint32_t ctrl_ext;
	uint16_t csum;
	int diag, i, ret;

	PMD_INIT_FUNC_TRACE();

	yusur2_dev_macsec_setting_reset(eth_dev);

	eth_dev->dev_ops = &yusur2_eth_dev_ops;
	eth_dev->rx_pkt_burst = &yusur2_recv_pkts;
	eth_dev->tx_pkt_burst = &yusur2_xmit_pkts;
	eth_dev->tx_pkt_prepare = &yusur2_prep_pkts;

	/*
	 * For secondary processes, we don't initialise any further as primary
	 * has already done this work. Only check we don't need a different
	 * RX and TX function.
	 */
	if (rte_eal_process_type() != RTE_PROC_PRIMARY) {
		struct yusur2_tx_queue *txq;
		/* TX queue function in primary, set by last queue initialized
		 * Tx queue may not initialized by primary process
		 */
		if (eth_dev->data->tx_queues) {
			txq = eth_dev->data->tx_queues[eth_dev->data->nb_tx_queues-1];
			yusur2_set_tx_function(eth_dev, txq);
		} else {
			/* Use default TX function if we get here */
			PMD_INIT_LOG(NOTICE, "No TX queues configured yet. "
				     "Using default TX function.");
		}

		yusur2_set_rx_function(eth_dev);

		return 0;
	}

	rte_atomic32_clear(&ad->link_thread_running);
	rte_eth_copy_pci_info(eth_dev, pci_dev);

	/* Vendor and Device ID need to be set before init of shared code */
	hw->device_id = pci_dev->id.device_id;
	hw->vendor_id = pci_dev->id.vendor_id;
	hw->hw_addr = (void *)pci_dev->mem_resource[0].addr;
	hw->allow_unsupported_sfp = 1;

	/* Initialize the shared code (base driver) */
#ifdef RTE_LIBRTE_YUSUR2_BYPASS
	diag = yusur2_bypass_init_shared_code(hw);
#else
	diag = yusur2_init_shared_code(hw);
#endif /* RTE_LIBRTE_YUSUR2_BYPASS */

	if (diag != YUSUR2_SUCCESS) {
		PMD_INIT_LOG(ERR, "Shared code init failed: %d", diag);
		return -EIO;
	}

	if (hw->mac.ops.fw_recovery_mode && hw->mac.ops.fw_recovery_mode(hw)) {
		PMD_INIT_LOG(ERR, "\nERROR: "
			"Firmware recovery mode detected. Limiting functionality.\n"
			"Refer to the Intel(R) Ethernet Adapters and Devices "
			"User Guide for details on firmware recovery mode.");
		return -EIO;
	}

	/* pick up the PCI bus settings for reporting later */
	yusur2_get_bus_info(hw);

	/* Unlock any pending hardware semaphore */
	yusur2_swfw_lock_reset(hw);

#ifdef RTE_LIBRTE_SECURITY
	/* Initialize security_ctx only for primary process*/
	if (yusur2_ipsec_ctx_create(eth_dev))
		return -ENOMEM;
#endif

	/* Initialize DCB configuration*/
	memset(dcb_config, 0, sizeof(struct yusur2_dcb_config));
	yusur2_dcb_init(hw, dcb_config);
	/* Get Hardware Flow Control setting */
	hw->fc.requested_mode = yusur2_fc_none;
	hw->fc.current_mode = yusur2_fc_none;
	hw->fc.pause_time = YUSUR2_FC_PAUSE;
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		hw->fc.low_water[i] = YUSUR2_FC_LO;
		hw->fc.high_water[i] = YUSUR2_FC_HI;
	}
	hw->fc.send_xon = 1;

	/* Make sure we have a good EEPROM before we read from it */
	diag = yusur2_validate_eeprom_checksum(hw, &csum);
	if (diag != YUSUR2_SUCCESS) {
		PMD_INIT_LOG(ERR, "The EEPROM checksum is not valid: %d", diag);
		return -EIO;
	}

#ifdef RTE_LIBRTE_YUSUR2_BYPASS
	diag = yusur2_bypass_init_hw(hw);
#else
	diag = yusur2_init_hw(hw);
#endif /* RTE_LIBRTE_YUSUR2_BYPASS */

	/*
	 * Devices with copper phys will fail to initialise if yusur2_init_hw()
	 * is called too soon after the kernel driver unbinding/binding occurs.
	 * The failure occurs in yusur2_identify_phy_generic() for all devices,
	 * but for non-copper devies, yusur2_identify_sfp_module_generic() is
	 * also called. See yusur2_identify_phy_82599(). The reason for the
	 * failure is not known, and only occuts when virtualisation features
	 * are disabled in the bios. A delay of 100ms  was found to be enough by
	 * trial-and-error, and is doubled to be safe.
	 */
	if (diag && (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper)) {
		rte_delay_ms(200);
		diag = yusur2_init_hw(hw);
	}

	if (diag == YUSUR2_ERR_SFP_NOT_PRESENT)
		diag = YUSUR2_SUCCESS;

	if (diag == YUSUR2_ERR_EEPROM_VERSION) {
		PMD_INIT_LOG(ERR, "This device is a pre-production adapter/"
			     "LOM.  Please be aware there may be issues associated "
			     "with your hardware.");
		PMD_INIT_LOG(ERR, "If you are experiencing problems "
			     "please contact your Intel or hardware representative "
			     "who provided you with this hardware.");
	} else if (diag == YUSUR2_ERR_SFP_NOT_SUPPORTED)
		PMD_INIT_LOG(ERR, "Unsupported SFP+ Module");
	if (diag) {
		PMD_INIT_LOG(ERR, "Hardware Initialization Failure: %d", diag);
		return -EIO;
	}

	/* Reset the hw statistics */
	yusur2_dev_stats_reset(eth_dev);

	/* disable interrupt */
	yusur2_disable_intr(hw);

	/* reset mappings for queue statistics hw counters*/
	yusur2_reset_qstat_mappings(hw);

	/* Allocate memory for storing MAC addresses */
	eth_dev->data->mac_addrs = rte_zmalloc("yusur2", RTE_ETHER_ADDR_LEN *
					       hw->mac.num_rar_entries, 0);
	if (eth_dev->data->mac_addrs == NULL) {
		PMD_INIT_LOG(ERR,
			     "Failed to allocate %u bytes needed to store "
			     "MAC addresses",
			     RTE_ETHER_ADDR_LEN * hw->mac.num_rar_entries);
		return -ENOMEM;
	}
	/* Copy the permanent MAC address */
	rte_ether_addr_copy((struct rte_ether_addr *)hw->mac.perm_addr,
			&eth_dev->data->mac_addrs[0]);

	/* Allocate memory for storing hash filter MAC addresses */
	eth_dev->data->hash_mac_addrs = rte_zmalloc(
		"yusur2", RTE_ETHER_ADDR_LEN * YUSUR2_VMDQ_NUM_UC_MAC, 0);
	if (eth_dev->data->hash_mac_addrs == NULL) {
		PMD_INIT_LOG(ERR,
			     "Failed to allocate %d bytes needed to store MAC addresses",
			     RTE_ETHER_ADDR_LEN * YUSUR2_VMDQ_NUM_UC_MAC);
		return -ENOMEM;
	}

	/* Pass the information to the rte_eth_dev_close() that it should also
	 * release the private port resources.
	 */
	eth_dev->data->dev_flags |= RTE_ETH_DEV_CLOSE_REMOVE;

	/* initialize the vfta */
	memset(shadow_vfta, 0, sizeof(*shadow_vfta));

	/* initialize the hw strip bitmap*/
	memset(hwstrip, 0, sizeof(*hwstrip));

	/* initialize PF if max_vfs not zero */
	ret = yusur2_pf_host_init(eth_dev);
	if (ret) {
		rte_free(eth_dev->data->mac_addrs);
		eth_dev->data->mac_addrs = NULL;
		rte_free(eth_dev->data->hash_mac_addrs);
		eth_dev->data->hash_mac_addrs = NULL;
		return ret;
	}

	ctrl_ext = YUSUR2_READ_REG(hw, YUSUR2_CTRL_EXT);
	/* let hardware know driver is loaded */
	ctrl_ext |= YUSUR2_CTRL_EXT_DRV_LOAD;
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
	ctrl_ext |= YUSUR2_CTRL_EXT_PFRSTD;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL_EXT, ctrl_ext);
	YUSUR2_WRITE_FLUSH(hw);

	if (yusur2_is_sfp(hw) && hw->phy.sfp_type != yusur2_sfp_type_not_present)
		PMD_INIT_LOG(DEBUG, "MAC: %d, PHY: %d, SFP+: %d",
			     (int) hw->mac.type, (int) hw->phy.type,
			     (int) hw->phy.sfp_type);
	else
		PMD_INIT_LOG(DEBUG, "MAC: %d, PHY: %d",
			     (int) hw->mac.type, (int) hw->phy.type);

	PMD_INIT_LOG(DEBUG, "port %d vendorID=0x%x deviceID=0x%x",
		     eth_dev->data->port_id, pci_dev->id.vendor_id,
		     pci_dev->id.device_id);

	rte_intr_callback_register(intr_handle,
				   yusur2_dev_interrupt_handler, eth_dev);

	/* enable uio/vfio intr/eventfd mapping */
	rte_intr_enable(intr_handle);

	/* enable support intr */
	yusur2_enable_intr(eth_dev);

	/* initialize filter info */
	memset(filter_info, 0,
	       sizeof(struct yusur2_filter_info));

	/* initialize 5tuple filter list */
	TAILQ_INIT(&filter_info->fivetuple_list);

	/* initialize flow director filter list & hash */
	yusur2_fdir_filter_init(eth_dev);

	/* initialize l2 tunnel filter list & hash */
	yusur2_l2_tn_filter_init(eth_dev);

	/* initialize flow filter lists */
	yusur2_filterlist_init();

	/* initialize bandwidth configuration info */
	memset(bw_conf, 0, sizeof(struct yusur2_bw_conf));

	/* initialize Traffic Manager configuration */
	yusur2_tm_conf_init(eth_dev);

	return 0;
}

static int
eth_yusur2_dev_uninit(struct rte_eth_dev *eth_dev)
{
	PMD_INIT_FUNC_TRACE();

	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return 0;

	yusur2_dev_close(eth_dev);

	return 0;
}

static int yusur2_ntuple_filter_uninit(struct rte_eth_dev *eth_dev)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(eth_dev->data->dev_private);
	struct yusur2_5tuple_filter *p_5tuple;

	while ((p_5tuple = TAILQ_FIRST(&filter_info->fivetuple_list))) {
		TAILQ_REMOVE(&filter_info->fivetuple_list,
			     p_5tuple,
			     entries);
		rte_free(p_5tuple);
	}
	memset(filter_info->fivetuple_mask, 0,
	       sizeof(uint32_t) * YUSUR2_5TUPLE_ARRAY_SIZE);

	return 0;
}

static int yusur2_fdir_filter_uninit(struct rte_eth_dev *eth_dev)
{
	struct yusur2_hw_fdir_info *fdir_info =
		YUSUR2_DEV_PRIVATE_TO_FDIR_INFO(eth_dev->data->dev_private);
	struct yusur2_fdir_filter *fdir_filter;

		if (fdir_info->hash_map)
		rte_free(fdir_info->hash_map);
	if (fdir_info->hash_handle)
		rte_hash_free(fdir_info->hash_handle);

	while ((fdir_filter = TAILQ_FIRST(&fdir_info->fdir_list))) {
		TAILQ_REMOVE(&fdir_info->fdir_list,
			     fdir_filter,
			     entries);
		rte_free(fdir_filter);
	}

	return 0;
}

static int yusur2_l2_tn_filter_uninit(struct rte_eth_dev *eth_dev)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(eth_dev->data->dev_private);
	struct yusur2_l2_tn_filter *l2_tn_filter;

	if (l2_tn_info->hash_map)
		rte_free(l2_tn_info->hash_map);
	if (l2_tn_info->hash_handle)
		rte_hash_free(l2_tn_info->hash_handle);

	while ((l2_tn_filter = TAILQ_FIRST(&l2_tn_info->l2_tn_list))) {
		TAILQ_REMOVE(&l2_tn_info->l2_tn_list,
			     l2_tn_filter,
			     entries);
		rte_free(l2_tn_filter);
	}

	return 0;
}

static int yusur2_fdir_filter_init(struct rte_eth_dev *eth_dev)
{
	struct yusur2_hw_fdir_info *fdir_info =
		YUSUR2_DEV_PRIVATE_TO_FDIR_INFO(eth_dev->data->dev_private);
	char fdir_hash_name[RTE_HASH_NAMESIZE];
	struct rte_hash_parameters fdir_hash_params = {
		.name = fdir_hash_name,
		.entries = YUSUR2_MAX_FDIR_FILTER_NUM,
		.key_len = sizeof(union yusur2_atr_input),
		.hash_func = rte_hash_crc,
		.hash_func_init_val = 0,
		.socket_id = rte_socket_id(),
	};

	TAILQ_INIT(&fdir_info->fdir_list);
	snprintf(fdir_hash_name, RTE_HASH_NAMESIZE,
		 "fdir_%s", eth_dev->device->name);
	fdir_info->hash_handle = rte_hash_create(&fdir_hash_params);
	if (!fdir_info->hash_handle) {
		PMD_INIT_LOG(ERR, "Failed to create fdir hash table!");
		return -EINVAL;
	}
	fdir_info->hash_map = rte_zmalloc("yusur2",
					  sizeof(struct yusur2_fdir_filter *) *
					  YUSUR2_MAX_FDIR_FILTER_NUM,
					  0);
	if (!fdir_info->hash_map) {
		PMD_INIT_LOG(ERR,
			     "Failed to allocate memory for fdir hash map!");
		return -ENOMEM;
	}
	fdir_info->mask_added = FALSE;

	return 0;
}

static int yusur2_l2_tn_filter_init(struct rte_eth_dev *eth_dev)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(eth_dev->data->dev_private);
	char l2_tn_hash_name[RTE_HASH_NAMESIZE];
	struct rte_hash_parameters l2_tn_hash_params = {
		.name = l2_tn_hash_name,
		.entries = YUSUR2_MAX_L2_TN_FILTER_NUM,
		.key_len = sizeof(struct yusur2_l2_tn_key),
		.hash_func = rte_hash_crc,
		.hash_func_init_val = 0,
		.socket_id = rte_socket_id(),
	};

	TAILQ_INIT(&l2_tn_info->l2_tn_list);
	snprintf(l2_tn_hash_name, RTE_HASH_NAMESIZE,
		 "l2_tn_%s", eth_dev->device->name);
	l2_tn_info->hash_handle = rte_hash_create(&l2_tn_hash_params);
	if (!l2_tn_info->hash_handle) {
		PMD_INIT_LOG(ERR, "Failed to create L2 TN hash table!");
		return -EINVAL;
	}
	l2_tn_info->hash_map = rte_zmalloc("yusur2",
				   sizeof(struct yusur2_l2_tn_filter *) *
				   YUSUR2_MAX_L2_TN_FILTER_NUM,
				   0);
	if (!l2_tn_info->hash_map) {
		PMD_INIT_LOG(ERR,
			"Failed to allocate memory for L2 TN hash map!");
		return -ENOMEM;
	}
	l2_tn_info->e_tag_en = FALSE;
	l2_tn_info->e_tag_fwd_en = FALSE;
	l2_tn_info->e_tag_ether_type = RTE_ETHER_TYPE_ETAG;

	return 0;
}
/*
 * Negotiate mailbox API version with the PF.
 * After reset API version is always set to the basic one (yusur2_mbox_api_10).
 * Then we try to negotiate starting with the most recent one.
 * If all negotiation attempts fail, then we will proceed with
 * the default one (yusur2_mbox_api_10).
 */
static void
yusur2vf_negotiate_api(struct yusur2_hw *hw)
{
	int32_t i;

	/* start with highest supported, proceed down */
	static const enum yusur2_pfvf_api_rev sup_ver[] = {
		yusur2_mbox_api_13,
		yusur2_mbox_api_12,
		yusur2_mbox_api_11,
		yusur2_mbox_api_10,
	};

	for (i = 0;
			i != RTE_DIM(sup_ver) &&
			yusur2vf_negotiate_api_version(hw, sup_ver[i]) != 0;
			i++)
		;
}

static void
generate_random_mac_addr(struct rte_ether_addr *mac_addr)
{
	uint64_t random;

	/* Set Organizationally Unique Identifier (OUI) prefix. */
	mac_addr->addr_bytes[0] = 0x00;
	mac_addr->addr_bytes[1] = 0x09;
	mac_addr->addr_bytes[2] = 0xC0;
	/* Force indication of locally assigned MAC address. */
	mac_addr->addr_bytes[0] |= RTE_ETHER_LOCAL_ADMIN_ADDR;
	/* Generate the last 3 bytes of the MAC address with a random number. */
	random = rte_rand();
	memcpy(&mac_addr->addr_bytes[3], &random, 3);
}

static int
devarg_handle_int(__rte_unused const char *key, const char *value,
		  void *extra_args)
{
	uint16_t *n = extra_args;

	if (value == NULL || extra_args == NULL)
		return -EINVAL;

	*n = (uint16_t)strtoul(value, NULL, 0);
	if (*n == USHRT_MAX && errno == ERANGE)
		return -1;

	return 0;
}

static void
yusur2vf_parse_devargs(struct yusur2_adapter *adapter,
		      struct rte_devargs *devargs)
{
	struct rte_kvargs *kvlist;
	uint16_t pflink_fullchk;

	if (devargs == NULL)
		return;

	kvlist = rte_kvargs_parse(devargs->args, yusur2vf_valid_arguments);
	if (kvlist == NULL)
		return;

	if (rte_kvargs_count(kvlist, YUSUR2VF_DEVARG_PFLINK_FULLCHK) == 1 &&
	    rte_kvargs_process(kvlist, YUSUR2VF_DEVARG_PFLINK_FULLCHK,
			       devarg_handle_int, &pflink_fullchk) == 0 &&
	    pflink_fullchk == 1)
		adapter->pflink_fullchk = 1;

	rte_kvargs_free(kvlist);
}

/*
 * Virtual Function device init
 */
static int
eth_yusur2vf_dev_init(struct rte_eth_dev *eth_dev)
{
	int diag;
	uint32_t tc, tcs;
	struct yusur2_adapter *ad = eth_dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(eth_dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(eth_dev->data->dev_private);
	struct yusur2_hwstrip *hwstrip =
		YUSUR2_DEV_PRIVATE_TO_HWSTRIP_BITMAP(eth_dev->data->dev_private);
	struct rte_ether_addr *perm_addr =
		(struct rte_ether_addr *)hw->mac.perm_addr;

	PMD_INIT_FUNC_TRACE();

	eth_dev->dev_ops = &yusur2vf_eth_dev_ops;
	eth_dev->rx_pkt_burst = &yusur2_recv_pkts;
	eth_dev->tx_pkt_burst = &yusur2_xmit_pkts;

	/* for secondary processes, we don't initialise any further as primary
	 * has already done this work. Only check we don't need a different
	 * RX function
	 */
	if (rte_eal_process_type() != RTE_PROC_PRIMARY) {
		struct yusur2_tx_queue *txq;
		/* TX queue function in primary, set by last queue initialized
		 * Tx queue may not initialized by primary process
		 */
		if (eth_dev->data->tx_queues) {
			txq = eth_dev->data->tx_queues[eth_dev->data->nb_tx_queues - 1];
			yusur2_set_tx_function(eth_dev, txq);
		} else {
			/* Use default TX function if we get here */
			PMD_INIT_LOG(NOTICE,
				     "No TX queues configured yet. Using default TX function.");
		}

		yusur2_set_rx_function(eth_dev);

		return 0;
	}

	rte_atomic32_clear(&ad->link_thread_running);
	yusur2vf_parse_devargs(eth_dev->data->dev_private,
			      pci_dev->device.devargs);

	rte_eth_copy_pci_info(eth_dev, pci_dev);

	hw->device_id = pci_dev->id.device_id;
	hw->vendor_id = pci_dev->id.vendor_id;
	hw->hw_addr = (void *)pci_dev->mem_resource[0].addr;

	/* initialize the vfta */
	memset(shadow_vfta, 0, sizeof(*shadow_vfta));

	/* initialize the hw strip bitmap*/
	memset(hwstrip, 0, sizeof(*hwstrip));

	/* Initialize the shared code (base driver) */
	diag = yusur2_init_shared_code(hw);
	if (diag != YUSUR2_SUCCESS) {
		PMD_INIT_LOG(ERR, "Shared code init failed for yusur2vf: %d", diag);
		return -EIO;
	}

	/* init_mailbox_params */
	hw->mbx.ops.init_params(hw);

	/* Reset the hw statistics */
	yusur2vf_dev_stats_reset(eth_dev);

	/* Disable the interrupts for VF */
	yusur2vf_intr_disable(eth_dev);

	hw->mac.num_rar_entries = 128; /* The MAX of the underlying PF */
	diag = hw->mac.ops.reset_hw(hw);

	/*
	 * The VF reset operation returns the YUSUR2_ERR_INVALID_MAC_ADDR when
	 * the underlying PF driver has not assigned a MAC address to the VF.
	 * In this case, assign a random MAC address.
	 */
	if ((diag != YUSUR2_SUCCESS) && (diag != YUSUR2_ERR_INVALID_MAC_ADDR)) {
		PMD_INIT_LOG(ERR, "VF Initialization Failure: %d", diag);
		/*
		 * This error code will be propagated to the app by
		 * rte_eth_dev_reset, so use a public error code rather than
		 * the internal-only YUSUR2_ERR_RESET_FAILED
		 */
		return -EAGAIN;
	}

	/* negotiate mailbox API version to use with the PF. */
	yusur2vf_negotiate_api(hw);

	/* Get Rx/Tx queue count via mailbox, which is ready after reset_hw */
	yusur2vf_get_queues(hw, &tcs, &tc);

	/* Allocate memory for storing MAC addresses */
	eth_dev->data->mac_addrs = rte_zmalloc("yusur2vf", RTE_ETHER_ADDR_LEN *
					       hw->mac.num_rar_entries, 0);
	if (eth_dev->data->mac_addrs == NULL) {
		PMD_INIT_LOG(ERR,
			     "Failed to allocate %u bytes needed to store "
			     "MAC addresses",
			     RTE_ETHER_ADDR_LEN * hw->mac.num_rar_entries);
		return -ENOMEM;
	}

	/* Pass the information to the rte_eth_dev_close() that it should also
	 * release the private port resources.
	 */
	eth_dev->data->dev_flags |= RTE_ETH_DEV_CLOSE_REMOVE;

	/* Generate a random MAC address, if none was assigned by PF. */
	if (rte_is_zero_ether_addr(perm_addr)) {
		generate_random_mac_addr(perm_addr);
		diag = yusur2_set_rar_vf(hw, 1, perm_addr->addr_bytes, 0, 1);
		if (diag) {
			rte_free(eth_dev->data->mac_addrs);
			eth_dev->data->mac_addrs = NULL;
			return diag;
		}
		PMD_INIT_LOG(INFO, "\tVF MAC address not assigned by Host PF");
		PMD_INIT_LOG(INFO, "\tAssign randomly generated MAC address "
			     "%02x:%02x:%02x:%02x:%02x:%02x",
			     perm_addr->addr_bytes[0],
			     perm_addr->addr_bytes[1],
			     perm_addr->addr_bytes[2],
			     perm_addr->addr_bytes[3],
			     perm_addr->addr_bytes[4],
			     perm_addr->addr_bytes[5]);
	}

	/* Copy the permanent MAC address */
	rte_ether_addr_copy(perm_addr, &eth_dev->data->mac_addrs[0]);

	/* reset the hardware with the new settings */
	diag = hw->mac.ops.start_hw(hw);
	switch (diag) {
	case  0:
		break;

	default:
		PMD_INIT_LOG(ERR, "VF Initialization Failure: %d", diag);
		return -EIO;
	}

	rte_intr_callback_register(intr_handle,
				   yusur2vf_dev_interrupt_handler, eth_dev);
	rte_intr_enable(intr_handle);
	yusur2vf_intr_enable(eth_dev);

	PMD_INIT_LOG(DEBUG, "port %d vendorID=0x%x deviceID=0x%x mac.type=%s",
		     eth_dev->data->port_id, pci_dev->id.vendor_id,
		     pci_dev->id.device_id, "yusur2_mac_82599_vf");

	return 0;
}

/* Virtual Function device uninit */

static int
eth_yusur2vf_dev_uninit(struct rte_eth_dev *eth_dev)
{
	PMD_INIT_FUNC_TRACE();

	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return 0;

	yusur2vf_dev_close(eth_dev);

	return 0;
}

static int
eth_yusur2_pci_probe(struct rte_pci_driver *pci_drv __rte_unused,
		struct rte_pci_device *pci_dev)
{
	char name[RTE_ETH_NAME_MAX_LEN];
	struct rte_eth_dev *pf_ethdev;
	struct rte_eth_devargs eth_da;
	int i, retval;

	if (pci_dev->device.devargs) {
		retval = rte_eth_devargs_parse(pci_dev->device.devargs->args,
				&eth_da);
		if (retval)
			return retval;
	} else
		memset(&eth_da, 0, sizeof(eth_da));

	retval = rte_eth_dev_create(&pci_dev->device, pci_dev->device.name,
		sizeof(struct yusur2_adapter),
		eth_dev_pci_specific_init, pci_dev,
		eth_yusur2_dev_init, NULL);

	if (retval || eth_da.nb_representor_ports < 1)
		return retval;

	pf_ethdev = rte_eth_dev_allocated(pci_dev->device.name);
	if (pf_ethdev == NULL)
		return -ENODEV;

	/* probe VF representor ports */
	for (i = 0; i < eth_da.nb_representor_ports; i++) {
		struct yusur2_vf_info *vfinfo;
		struct yusur2_vf_representor representor;

		vfinfo = *YUSUR2_DEV_PRIVATE_TO_P_VFDATA(
			pf_ethdev->data->dev_private);
		if (vfinfo == NULL) {
			PMD_DRV_LOG(ERR,
				"no virtual functions supported by PF");
			break;
		}

		representor.vf_id = eth_da.representor_ports[i];
		representor.switch_domain_id = vfinfo->switch_domain_id;
		representor.pf_ethdev = pf_ethdev;

		/* representor port net_bdf_port */
		snprintf(name, sizeof(name), "net_%s_representor_%d",
			pci_dev->device.name,
			eth_da.representor_ports[i]);

		retval = rte_eth_dev_create(&pci_dev->device, name,
			sizeof(struct yusur2_vf_representor), NULL, NULL,
			yusur2_vf_representor_init, &representor);

		if (retval)
			PMD_DRV_LOG(ERR, "failed to create yusur2 vf "
				"representor %s.", name);
	}

	return 0;
}

static int eth_yusur2_pci_remove(struct rte_pci_device *pci_dev)
{
	struct rte_eth_dev *ethdev;

	ethdev = rte_eth_dev_allocated(pci_dev->device.name);
	if (!ethdev)
		return 0;

	if (ethdev->data->dev_flags & RTE_ETH_DEV_REPRESENTOR)
		return rte_eth_dev_pci_generic_remove(pci_dev,
					yusur2_vf_representor_uninit);
	else
		return rte_eth_dev_pci_generic_remove(pci_dev,
						eth_yusur2_dev_uninit);
}

static struct rte_pci_driver rte_yusur2_pmd = {
	.id_table = pci_id_yusur2_map,
	.drv_flags = RTE_PCI_DRV_NEED_MAPPING | RTE_PCI_DRV_INTR_LSC,
	.probe = eth_yusur2_pci_probe,
	.remove = eth_yusur2_pci_remove,
};

static int eth_yusur2vf_pci_probe(struct rte_pci_driver *pci_drv __rte_unused,
	struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_probe(pci_dev,
		sizeof(struct yusur2_adapter), eth_yusur2vf_dev_init);
}

static int eth_yusur2vf_pci_remove(struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_remove(pci_dev, eth_yusur2vf_dev_uninit);
}

/*
 * virtual function driver struct
 */
static struct rte_pci_driver rte_yusur2vf_pmd = {
	.id_table = pci_id_yusur2vf_map,
	.drv_flags = RTE_PCI_DRV_NEED_MAPPING,
	.probe = eth_yusur2vf_pci_probe,
	.remove = eth_yusur2vf_pci_remove,
};

static int
yusur2_vlan_filter_set(struct rte_eth_dev *dev, uint16_t vlan_id, int on)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	uint32_t vfta;
	uint32_t vid_idx;
	uint32_t vid_bit;

	vid_idx = (uint32_t) ((vlan_id >> 5) & 0x7F);
	vid_bit = (uint32_t) (1 << (vlan_id & 0x1F));
	vfta = YUSUR2_READ_REG(hw, YUSUR2_VFTA(vid_idx));
	if (on)
		vfta |= vid_bit;
	else
		vfta &= ~vid_bit;
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(vid_idx), vfta);

	/* update local VFTA copy */
	shadow_vfta->vfta[vid_idx] = vfta;

	return 0;
}

static void
yusur2_vlan_strip_queue_set(struct rte_eth_dev *dev, uint16_t queue, int on)
{
	if (on)
		yusur2_vlan_hw_strip_enable(dev, queue);
	else
		yusur2_vlan_hw_strip_disable(dev, queue);
}

static int
yusur2_vlan_tpid_set(struct rte_eth_dev *dev,
		    enum rte_vlan_type vlan_type,
		    uint16_t tpid)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret = 0;
	uint32_t reg;
	uint32_t qinq;

	qinq = YUSUR2_READ_REG(hw, YUSUR2_DMATXCTL);
	qinq &= YUSUR2_DMATXCTL_GDV;

	switch (vlan_type) {
	case ETH_VLAN_TYPE_INNER:
		if (qinq) {
			reg = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
			reg = (reg & (~YUSUR2_VLNCTRL_VET)) | (uint32_t)tpid;
			YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, reg);
			reg = YUSUR2_READ_REG(hw, YUSUR2_DMATXCTL);
			reg = (reg & (~YUSUR2_DMATXCTL_VT_MASK))
				| ((uint32_t)tpid << YUSUR2_DMATXCTL_VT_SHIFT);
			YUSUR2_WRITE_REG(hw, YUSUR2_DMATXCTL, reg);
		} else {
			ret = -ENOTSUP;
			PMD_DRV_LOG(ERR, "Inner type is not supported"
				    " by single VLAN");
		}
		break;
	case ETH_VLAN_TYPE_OUTER:
		if (qinq) {
			/* Only the high 16-bits is valid */
			YUSUR2_WRITE_REG(hw, YUSUR2_EXVET, (uint32_t)tpid <<
					YUSUR2_EXVET_VET_EXT_SHIFT);
		} else {
			reg = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
			reg = (reg & (~YUSUR2_VLNCTRL_VET)) | (uint32_t)tpid;
			YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, reg);
			reg = YUSUR2_READ_REG(hw, YUSUR2_DMATXCTL);
			reg = (reg & (~YUSUR2_DMATXCTL_VT_MASK))
				| ((uint32_t)tpid << YUSUR2_DMATXCTL_VT_SHIFT);
			YUSUR2_WRITE_REG(hw, YUSUR2_DMATXCTL, reg);
		}

		break;
	default:
		ret = -EINVAL;
		PMD_DRV_LOG(ERR, "Unsupported VLAN type %d", vlan_type);
		break;
	}

	return ret;
}

void
yusur2_vlan_hw_filter_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t vlnctrl;

	PMD_INIT_FUNC_TRACE();

	/* Filter Table Disable */
	vlnctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
	vlnctrl &= ~YUSUR2_VLNCTRL_VFE;

	YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, vlnctrl);
}

void
yusur2_vlan_hw_filter_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	uint32_t vlnctrl;
	uint16_t i;

	PMD_INIT_FUNC_TRACE();

	/* Filter Table Enable */
	vlnctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
	vlnctrl &= ~YUSUR2_VLNCTRL_CFIEN;
	vlnctrl |= YUSUR2_VLNCTRL_VFE;

	YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, vlnctrl);

	/* write whatever is in local vfta copy */
	for (i = 0; i < YUSUR2_VFTA_SIZE; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(i), shadow_vfta->vfta[i]);
}

static void
yusur2_vlan_hw_strip_bitmap_set(struct rte_eth_dev *dev, uint16_t queue, bool on)
{
	struct yusur2_hwstrip *hwstrip =
		YUSUR2_DEV_PRIVATE_TO_HWSTRIP_BITMAP(dev->data->dev_private);
	struct yusur2_rx_queue *rxq;

	if (queue >= YUSUR2_MAX_RX_QUEUE_NUM)
		return;

	if (on)
		YUSUR2_SET_HWSTRIP(hwstrip, queue);
	else
		YUSUR2_CLEAR_HWSTRIP(hwstrip, queue);

	if (queue >= dev->data->nb_rx_queues)
		return;

	rxq = dev->data->rx_queues[queue];

	if (on) {
		rxq->vlan_flags = PKT_RX_VLAN | PKT_RX_VLAN_STRIPPED;
		rxq->offloads |= DEV_RX_OFFLOAD_VLAN_STRIP;
	} else {
		rxq->vlan_flags = PKT_RX_VLAN;
		rxq->offloads &= ~DEV_RX_OFFLOAD_VLAN_STRIP;
	}
}

static void
yusur2_vlan_hw_strip_disable(struct rte_eth_dev *dev, uint16_t queue)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	PMD_INIT_FUNC_TRACE();

	if (hw->mac.type == yusur2_mac_82598EB) {
		/* No queue level support */
		PMD_INIT_LOG(NOTICE, "82598EB not support queue level hw strip");
		return;
	}

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(queue));
	ctrl &= ~YUSUR2_RXDCTL_VME;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(queue), ctrl);

	/* record those setting for HW strip per queue */
	yusur2_vlan_hw_strip_bitmap_set(dev, queue, 0);
}

static void
yusur2_vlan_hw_strip_enable(struct rte_eth_dev *dev, uint16_t queue)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	PMD_INIT_FUNC_TRACE();

	if (hw->mac.type == yusur2_mac_82598EB) {
		/* No queue level supported */
		PMD_INIT_LOG(NOTICE, "82598EB not support queue level hw strip");
		return;
	}

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(queue));
	ctrl |= YUSUR2_RXDCTL_VME;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(queue), ctrl);

	/* record those setting for HW strip per queue */
	yusur2_vlan_hw_strip_bitmap_set(dev, queue, 1);
}

static void
yusur2_vlan_hw_extend_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	PMD_INIT_FUNC_TRACE();

	/* DMATXCTRL: Geric Double VLAN Disable */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_DMATXCTL);
	ctrl &= ~YUSUR2_DMATXCTL_GDV;
	YUSUR2_WRITE_REG(hw, YUSUR2_DMATXCTL, ctrl);

	/* CTRL_EXT: Global Double VLAN Disable */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_CTRL_EXT);
	ctrl &= ~YUSUR2_EXTENDED_VLAN;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL_EXT, ctrl);

}

static void
yusur2_vlan_hw_extend_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	PMD_INIT_FUNC_TRACE();

	/* DMATXCTRL: Geric Double VLAN Enable */
	ctrl  = YUSUR2_READ_REG(hw, YUSUR2_DMATXCTL);
	ctrl |= YUSUR2_DMATXCTL_GDV;
	YUSUR2_WRITE_REG(hw, YUSUR2_DMATXCTL, ctrl);

	/* CTRL_EXT: Global Double VLAN Enable */
	ctrl  = YUSUR2_READ_REG(hw, YUSUR2_CTRL_EXT);
	ctrl |= YUSUR2_EXTENDED_VLAN;
	YUSUR2_WRITE_REG(hw, YUSUR2_CTRL_EXT, ctrl);

	/* Clear pooling mode of PFVTCTL. It's required by X550. */
	if (hw->mac.type == yusur2_mac_X550 ||
	    hw->mac.type == yusur2_mac_X550EM_x ||
	    hw->mac.type == yusur2_mac_X550EM_a) {
		ctrl = YUSUR2_READ_REG(hw, YUSUR2_VT_CTL);
		ctrl &= ~YUSUR2_VT_CTL_POOLING_MODE_MASK;
		YUSUR2_WRITE_REG(hw, YUSUR2_VT_CTL, ctrl);
	}

	/*
	 * VET EXT field in the EXVET register = 0x8100 by default
	 * So no need to change. Same to VT field of DMATXCTL register
	 */
}

void
yusur2_vlan_hw_strip_config(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_rxmode *rxmode = &dev->data->dev_conf.rxmode;
	uint32_t ctrl;
	uint16_t i;
	struct yusur2_rx_queue *rxq;
	bool on;

	PMD_INIT_FUNC_TRACE();

	if (hw->mac.type == yusur2_mac_82598EB) {
		if (rxmode->offloads & DEV_RX_OFFLOAD_VLAN_STRIP) {
			ctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
			ctrl |= YUSUR2_VLNCTRL_VME;
			YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, ctrl);
		} else {
			ctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
			ctrl &= ~YUSUR2_VLNCTRL_VME;
			YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, ctrl);
		}
	} else {
		/*
		 * Other 10G NIC, the VLAN strip can be setup
		 * per queue in RXDCTL
		 */
		for (i = 0; i < dev->data->nb_rx_queues; i++) {
			rxq = dev->data->rx_queues[i];
			ctrl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx));
			if (rxq->offloads & DEV_RX_OFFLOAD_VLAN_STRIP) {
				ctrl |= YUSUR2_RXDCTL_VME;
				on = TRUE;
			} else {
				ctrl &= ~YUSUR2_RXDCTL_VME;
				on = FALSE;
			}
			YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(rxq->reg_idx), ctrl);

			/* record those setting for HW strip per queue */
			yusur2_vlan_hw_strip_bitmap_set(dev, i, on);
		}
	}
}

static void
yusur2_config_vlan_strip_on_all_queues(struct rte_eth_dev *dev, int mask)
{
	uint16_t i;
	struct rte_eth_rxmode *rxmode;
	struct yusur2_rx_queue *rxq;

	if (mask & ETH_VLAN_STRIP_MASK) {
		rxmode = &dev->data->dev_conf.rxmode;
		if (rxmode->offloads & DEV_RX_OFFLOAD_VLAN_STRIP)
			for (i = 0; i < dev->data->nb_rx_queues; i++) {
				rxq = dev->data->rx_queues[i];
				rxq->offloads |= DEV_RX_OFFLOAD_VLAN_STRIP;
			}
		else
			for (i = 0; i < dev->data->nb_rx_queues; i++) {
				rxq = dev->data->rx_queues[i];
				rxq->offloads &= ~DEV_RX_OFFLOAD_VLAN_STRIP;
			}
	}
}

static int
yusur2_vlan_offload_config(struct rte_eth_dev *dev, int mask)
{
	struct rte_eth_rxmode *rxmode;
	rxmode = &dev->data->dev_conf.rxmode;

	if (mask & ETH_VLAN_STRIP_MASK) {
		yusur2_vlan_hw_strip_config(dev);
	}

	if (mask & ETH_VLAN_FILTER_MASK) {
		if (rxmode->offloads & DEV_RX_OFFLOAD_VLAN_FILTER)
			yusur2_vlan_hw_filter_enable(dev);
		else
			yusur2_vlan_hw_filter_disable(dev);
	}

	if (mask & ETH_VLAN_EXTEND_MASK) {
		if (rxmode->offloads & DEV_RX_OFFLOAD_VLAN_EXTEND)
			yusur2_vlan_hw_extend_enable(dev);
		else
			yusur2_vlan_hw_extend_disable(dev);
	}

	return 0;
}

static int
yusur2_vlan_offload_set(struct rte_eth_dev *dev, int mask)
{
	yusur2_config_vlan_strip_on_all_queues(dev, mask);

	yusur2_vlan_offload_config(dev, mask);

	return 0;
}

static void
yusur2_vmdq_vlan_hw_filter_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	/* VLNCTRL: enable vlan filtering and allow all vlan tags through */
	uint32_t vlanctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);

	vlanctrl |= YUSUR2_VLNCTRL_VFE; /* enable vlan filters */
	YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, vlanctrl);
}

static int
yusur2_check_vf_rss_rxq_num(struct rte_eth_dev *dev, uint16_t nb_rx_q)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	switch (nb_rx_q) {
	case 1:
	case 2:
		RTE_ETH_DEV_SRIOV(dev).active = ETH_64_POOLS;
		break;
	case 4:
		RTE_ETH_DEV_SRIOV(dev).active = ETH_32_POOLS;
		break;
	default:
		return -EINVAL;
	}

	RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool =
		YUSUR2_MAX_RX_QUEUE_NUM / RTE_ETH_DEV_SRIOV(dev).active;
	RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx =
		pci_dev->max_vfs * RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
	return 0;
}

static int
yusur2_check_mq_mode(struct rte_eth_dev *dev)
{
	struct rte_eth_conf *dev_conf = &dev->data->dev_conf;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint16_t nb_rx_q = dev->data->nb_rx_queues;
	uint16_t nb_tx_q = dev->data->nb_tx_queues;

	if (RTE_ETH_DEV_SRIOV(dev).active != 0) {
		/* check multi-queue mode */
		switch (dev_conf->rxmode.mq_mode) {
		case ETH_MQ_RX_VMDQ_DCB:
			PMD_INIT_LOG(INFO, "ETH_MQ_RX_VMDQ_DCB mode supported in SRIOV");
			break;
		case ETH_MQ_RX_VMDQ_DCB_RSS:
			/* DCB/RSS VMDQ in SRIOV mode, not implement yet */
			PMD_INIT_LOG(ERR, "SRIOV active,"
					" unsupported mq_mode rx %d.",
					dev_conf->rxmode.mq_mode);
			return -EINVAL;
		case ETH_MQ_RX_RSS:
		case ETH_MQ_RX_VMDQ_RSS:
			dev->data->dev_conf.rxmode.mq_mode = ETH_MQ_RX_VMDQ_RSS;
			if (nb_rx_q <= RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool)
				if (yusur2_check_vf_rss_rxq_num(dev, nb_rx_q)) {
					PMD_INIT_LOG(ERR, "SRIOV is active,"
						" invalid queue number"
						" for VMDQ RSS, allowed"
						" value are 1, 2 or 4.");
					return -EINVAL;
				}
			break;
		case ETH_MQ_RX_VMDQ_ONLY:
		case ETH_MQ_RX_NONE:
			/* if nothing mq mode configure, use default scheme */
			dev->data->dev_conf.rxmode.mq_mode = ETH_MQ_RX_VMDQ_ONLY;
			break;
		default: /* ETH_MQ_RX_DCB, ETH_MQ_RX_DCB_RSS or ETH_MQ_TX_DCB*/
			/* SRIOV only works in VMDq enable mode */
			PMD_INIT_LOG(ERR, "SRIOV is active,"
					" wrong mq_mode rx %d.",
					dev_conf->rxmode.mq_mode);
			return -EINVAL;
		}

		switch (dev_conf->txmode.mq_mode) {
		case ETH_MQ_TX_VMDQ_DCB:
			PMD_INIT_LOG(INFO, "ETH_MQ_TX_VMDQ_DCB mode supported in SRIOV");
			dev->data->dev_conf.txmode.mq_mode = ETH_MQ_TX_VMDQ_DCB;
			break;
		default: /* ETH_MQ_TX_VMDQ_ONLY or ETH_MQ_TX_NONE */
			dev->data->dev_conf.txmode.mq_mode = ETH_MQ_TX_VMDQ_ONLY;
			break;
		}

		/* check valid queue number */
		if ((nb_rx_q > RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool) ||
		    (nb_tx_q > RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool)) {
			PMD_INIT_LOG(ERR, "SRIOV is active,"
					" nb_rx_q=%d nb_tx_q=%d queue number"
					" must be less than or equal to %d.",
					nb_rx_q, nb_tx_q,
					RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool);
			return -EINVAL;
		}
	} else {
		if (dev_conf->rxmode.mq_mode == ETH_MQ_RX_VMDQ_DCB_RSS) {
			PMD_INIT_LOG(ERR, "VMDQ+DCB+RSS mq_mode is"
					  " not supported.");
			return -EINVAL;
		}
		/* check configuration for vmdb+dcb mode */
		if (dev_conf->rxmode.mq_mode == ETH_MQ_RX_VMDQ_DCB) {
			const struct rte_eth_vmdq_dcb_conf *conf;

			if (nb_rx_q != YUSUR2_VMDQ_DCB_NB_QUEUES) {
				PMD_INIT_LOG(ERR, "VMDQ+DCB, nb_rx_q != %d.",
						YUSUR2_VMDQ_DCB_NB_QUEUES);
				return -EINVAL;
			}
			conf = &dev_conf->rx_adv_conf.vmdq_dcb_conf;
			if (!(conf->nb_queue_pools == ETH_16_POOLS ||
			       conf->nb_queue_pools == ETH_32_POOLS)) {
				PMD_INIT_LOG(ERR, "VMDQ+DCB selected,"
						" nb_queue_pools must be %d or %d.",
						ETH_16_POOLS, ETH_32_POOLS);
				return -EINVAL;
			}
		}
		if (dev_conf->txmode.mq_mode == ETH_MQ_TX_VMDQ_DCB) {
			const struct rte_eth_vmdq_dcb_tx_conf *conf;

			if (nb_tx_q != YUSUR2_VMDQ_DCB_NB_QUEUES) {
				PMD_INIT_LOG(ERR, "VMDQ+DCB, nb_tx_q != %d",
						 YUSUR2_VMDQ_DCB_NB_QUEUES);
				return -EINVAL;
			}
			conf = &dev_conf->tx_adv_conf.vmdq_dcb_tx_conf;
			if (!(conf->nb_queue_pools == ETH_16_POOLS ||
			       conf->nb_queue_pools == ETH_32_POOLS)) {
				PMD_INIT_LOG(ERR, "VMDQ+DCB selected,"
						" nb_queue_pools != %d and"
						" nb_queue_pools != %d.",
						ETH_16_POOLS, ETH_32_POOLS);
				return -EINVAL;
			}
		}

		/* For DCB mode check our configuration before we go further */
		if (dev_conf->rxmode.mq_mode == ETH_MQ_RX_DCB) {
			const struct rte_eth_dcb_rx_conf *conf;

			conf = &dev_conf->rx_adv_conf.dcb_rx_conf;
			if (!(conf->nb_tcs == ETH_4_TCS ||
			       conf->nb_tcs == ETH_8_TCS)) {
				PMD_INIT_LOG(ERR, "DCB selected, nb_tcs != %d"
						" and nb_tcs != %d.",
						ETH_4_TCS, ETH_8_TCS);
				return -EINVAL;
			}
		}

		if (dev_conf->txmode.mq_mode == ETH_MQ_TX_DCB) {
			const struct rte_eth_dcb_tx_conf *conf;

			conf = &dev_conf->tx_adv_conf.dcb_tx_conf;
			if (!(conf->nb_tcs == ETH_4_TCS ||
			       conf->nb_tcs == ETH_8_TCS)) {
				PMD_INIT_LOG(ERR, "DCB selected, nb_tcs != %d"
						" and nb_tcs != %d.",
						ETH_4_TCS, ETH_8_TCS);
				return -EINVAL;
			}
		}

		/*
		 * When DCB/VT is off, maximum number of queues changes,
		 * except for 82598EB, which remains constant.
		 */
		if (dev_conf->txmode.mq_mode == ETH_MQ_TX_NONE &&
				hw->mac.type != yusur2_mac_82598EB) {
			if (nb_tx_q > YUSUR2_NONE_MODE_TX_NB_QUEUES) {
				PMD_INIT_LOG(ERR,
					     "Neither VT nor DCB are enabled, "
					     "nb_tx_q > %d.",
					     YUSUR2_NONE_MODE_TX_NB_QUEUES);
				return -EINVAL;
			}
		}
	}
	return 0;
}

static int
yusur2_dev_configure(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_adapter *adapter = dev->data->dev_private;
	int ret;

	PMD_INIT_FUNC_TRACE();

	if (dev->data->dev_conf.rxmode.mq_mode & ETH_MQ_RX_RSS_FLAG)
		dev->data->dev_conf.rxmode.offloads |= DEV_RX_OFFLOAD_RSS_HASH;

	/* multipe queue mode checking */
	ret  = yusur2_check_mq_mode(dev);
	if (ret != 0) {
		PMD_DRV_LOG(ERR, "yusur2_check_mq_mode fails with %d.",
			    ret);
		return ret;
	}

	/* set flag to update link status after init */
	intr->flags |= YUSUR2_FLAG_NEED_LINK_UPDATE;

	/*
	 * Initialize to TRUE. If any of Rx queues doesn't meet the bulk
	 * allocation or vector Rx preconditions we will reset it.
	 */
	adapter->rx_bulk_alloc_allowed = true;
	adapter->rx_vec_allowed = true;

	return 0;
}

static void
yusur2_dev_phy_intr_setup(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	uint32_t gpie;

	/* only set up it on X550EM_X */
	if (hw->mac.type == yusur2_mac_X550EM_x) {
		gpie = YUSUR2_READ_REG(hw, YUSUR2_GPIE);
		gpie |= YUSUR2_SDP0_GPIEN_X550EM_x;
		YUSUR2_WRITE_REG(hw, YUSUR2_GPIE, gpie);
		if (hw->phy.type == yusur2_phy_x550em_ext_t)
			intr->mask |= YUSUR2_EICR_GPI_SDP0_X550EM_x;
	}
}

int
yusur2_set_vf_rate_limit(struct rte_eth_dev *dev, uint16_t vf,
			uint16_t tx_rate, uint64_t q_msk)
{
	struct yusur2_hw *hw;
	struct yusur2_vf_info *vfinfo;
	struct rte_eth_link link;
	uint8_t  nb_q_per_pool;
	uint32_t queue_stride;
	uint32_t queue_idx, idx = 0, vf_idx;
	uint32_t queue_end;
	uint16_t total_rate = 0;
	struct rte_pci_device *pci_dev;
	int ret;

	pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	ret = rte_eth_link_get_nowait(dev->data->port_id, &link);
	if (ret < 0)
		return ret;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (tx_rate > link.link_speed)
		return -EINVAL;

	if (q_msk == 0)
		return 0;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	vfinfo = *(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	nb_q_per_pool = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
	queue_stride = YUSUR2_MAX_RX_QUEUE_NUM / RTE_ETH_DEV_SRIOV(dev).active;
	queue_idx = vf * queue_stride;
	queue_end = queue_idx + nb_q_per_pool - 1;
	if (queue_end >= hw->mac.max_tx_queues)
		return -EINVAL;

	if (vfinfo) {
		for (vf_idx = 0; vf_idx < pci_dev->max_vfs; vf_idx++) {
			if (vf_idx == vf)
				continue;
			for (idx = 0; idx < RTE_DIM(vfinfo[vf_idx].tx_rate);
				idx++)
				total_rate += vfinfo[vf_idx].tx_rate[idx];
		}
	} else {
		return -EINVAL;
	}

	/* Store tx_rate for this vf. */
	for (idx = 0; idx < nb_q_per_pool; idx++) {
		if (((uint64_t)0x1 << idx) & q_msk) {
			if (vfinfo[vf].tx_rate[idx] != tx_rate)
				vfinfo[vf].tx_rate[idx] = tx_rate;
			total_rate += tx_rate;
		}
	}

	if (total_rate > dev->data->dev_link.link_speed) {
		/* Reset stored TX rate of the VF if it causes exceed
		 * link speed.
		 */
		memset(vfinfo[vf].tx_rate, 0, sizeof(vfinfo[vf].tx_rate));
		return -EINVAL;
	}

	/* Set RTTBCNRC of each queue/pool for vf X  */
	for (; queue_idx <= queue_end; queue_idx++) {
		if (0x1 & q_msk)
			yusur2_set_queue_rate_limit(dev, queue_idx, tx_rate);
		q_msk = q_msk >> 1;
	}

	return 0;
}

static int
yusur2_flow_ctrl_enable(struct rte_eth_dev *dev, struct yusur2_hw *hw)
{
	struct yusur2_adapter *adapter = dev->data->dev_private;
	int err;
	uint32_t mflcn;

	yusur2_setup_fc(hw);

	err = yusur2_fc_enable(hw);

	/* Not negotiated is not an error case */
	if (err == YUSUR2_SUCCESS || err == YUSUR2_ERR_FC_NOT_NEGOTIATED) {
		/*
		 *check if we want to forward MAC frames - driver doesn't
		 *have native capability to do that,
		 *so we'll write the registers ourselves
		 */

		mflcn = YUSUR2_READ_REG(hw, YUSUR2_MFLCN);

		/* set or clear MFLCN.PMCF bit depending on configuration */
		if (adapter->mac_ctrl_frame_fwd != 0)
			mflcn |= YUSUR2_MFLCN_PMCF;
		else
			mflcn &= ~YUSUR2_MFLCN_PMCF;

		YUSUR2_WRITE_REG(hw, YUSUR2_MFLCN, mflcn);
		YUSUR2_WRITE_FLUSH(hw);

		return 0;
	}
	return err;
}

/*
 * Configure device link speed and setup link.
 * It returns 0 on success.
 */
static int
yusur2_dev_start(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	uint32_t intr_vector = 0;
	int err, link_up = 0, negotiate = 0;
	uint32_t speed = 0;
	uint32_t allowed_speeds = 0;
	int mask = 0;
	int status;
	uint16_t vf, idx;
	uint32_t *link_speeds;
	struct yusur2_tm_conf *tm_conf =
		YUSUR2_DEV_PRIVATE_TO_TM_CONF(dev->data->dev_private);
	struct yusur2_macsec_setting *macsec_setting =
		YUSUR2_DEV_PRIVATE_TO_MACSEC_SETTING(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();

	/* Stop the link setup handler before resetting the HW. */
	yusur2_dev_wait_setup_link_complete(dev, 0);

	/* disable uio/vfio intr/eventfd mapping */
	rte_intr_disable(intr_handle);

	/* stop adapter */
	hw->adapter_stopped = 0;
	yusur2_stop_adapter(hw);

	/* reinitialize adapter
	 * this calls reset and start
	 */
	status = yusur2_pf_reset_hw(hw);
	if (status != 0)
		return -1;
	hw->mac.ops.start_hw(hw);
	hw->mac.get_link_status = true;

	/* configure PF module if SRIOV enabled */
	yusur2_pf_host_configure(dev);

	yusur2_dev_phy_intr_setup(dev);

	/* check and configure queue intr-vector mapping */
	if ((rte_intr_cap_multiple(intr_handle) ||
	     !RTE_ETH_DEV_SRIOV(dev).active) &&
	    dev->data->dev_conf.intr_conf.rxq != 0) {
		intr_vector = dev->data->nb_rx_queues;
		if (intr_vector > YUSUR2_MAX_INTR_QUEUE_NUM) {
			PMD_INIT_LOG(ERR, "At most %d intr queues supported",
					YUSUR2_MAX_INTR_QUEUE_NUM);
			return -ENOTSUP;
		}
		if (rte_intr_efd_enable(intr_handle, intr_vector))
			return -1;
	}

	if (rte_intr_dp_is_en(intr_handle) && !intr_handle->intr_vec) {
		intr_handle->intr_vec =
			rte_zmalloc("intr_vec",
				    dev->data->nb_rx_queues * sizeof(int), 0);
		if (intr_handle->intr_vec == NULL) {
			PMD_INIT_LOG(ERR, "Failed to allocate %d rx_queues"
				     " intr_vec", dev->data->nb_rx_queues);
			return -ENOMEM;
		}
	}

	/* confiugre msix for sleep until rx interrupt */
	yusur2_configure_msix(dev);

	/* initialize transmission unit */
	yusur2_dev_tx_init(dev);

	/* This can fail when allocating mbufs for descriptor rings */
	err = yusur2_dev_rx_init(dev);
	if (err) {
		PMD_INIT_LOG(ERR, "Unable to initialize RX hardware");
		goto error;
	}

	mask = ETH_VLAN_STRIP_MASK | ETH_VLAN_FILTER_MASK |
		ETH_VLAN_EXTEND_MASK;
	err = yusur2_vlan_offload_config(dev, mask);
	if (err) {
		PMD_INIT_LOG(ERR, "Unable to set VLAN offload");
		goto error;
	}

	if (dev->data->dev_conf.rxmode.mq_mode == ETH_MQ_RX_VMDQ_ONLY) {
		/* Enable vlan filtering for VMDq */
		yusur2_vmdq_vlan_hw_filter_enable(dev);
	}

	/* Configure DCB hw */
	yusur2_configure_dcb(dev);

	if (dev->data->dev_conf.fdir_conf.mode != RTE_FDIR_MODE_NONE) {
		err = yusur2_fdir_configure(dev);
		if (err)
			goto error;
	}

	/* Restore vf rate limit */
	if (vfinfo != NULL) {
		for (vf = 0; vf < pci_dev->max_vfs; vf++)
			for (idx = 0; idx < YUSUR2_MAX_QUEUE_NUM_PER_VF; idx++)
				if (vfinfo[vf].tx_rate[idx] != 0)
					yusur2_set_vf_rate_limit(
						dev, vf,
						vfinfo[vf].tx_rate[idx],
						1 << idx);
	}

	yusur2_restore_statistics_mapping(dev);

	err = yusur2_flow_ctrl_enable(dev, hw);
	if (err < 0) {
		PMD_INIT_LOG(ERR, "enable flow ctrl err");
		goto error;
	}

	err = yusur2_dev_rxtx_start(dev);
	if (err < 0) {
		PMD_INIT_LOG(ERR, "Unable to start rxtx queues");
		goto error;
	}

	/* Skip link setup if loopback mode is enabled. */
	if (dev->data->dev_conf.lpbk_mode != 0) {
		err = yusur2_check_supported_loopback_mode(dev);
		if (err < 0) {
			PMD_INIT_LOG(ERR, "Unsupported loopback mode");
			goto error;
		} else {
			goto skip_link_setup;
		}
	}

	if (yusur2_is_sfp(hw) && hw->phy.multispeed_fiber) {
		err = hw->mac.ops.setup_sfp(hw);
		if (err)
			goto error;
	}

	if (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper) {
		/* Turn on the copper */
		yusur2_set_phy_power(hw, true);
	} else {
		/* Turn on the laser */
		yusur2_enable_tx_laser(hw);
	}

	err = yusur2_check_link(hw, &speed, &link_up, 0);
	if (err)
		goto error;
	dev->data->dev_link.link_status = link_up;

	err = yusur2_get_link_capabilities(hw, &speed, &negotiate);
	if (err)
		goto error;

	switch (hw->mac.type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		allowed_speeds = ETH_LINK_SPEED_100M | ETH_LINK_SPEED_1G |
			ETH_LINK_SPEED_2_5G |  ETH_LINK_SPEED_5G |
			ETH_LINK_SPEED_10G;
		if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T ||
				hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T_L)
			allowed_speeds = ETH_LINK_SPEED_10M |
				ETH_LINK_SPEED_100M | ETH_LINK_SPEED_1G;
		break;
	default:
		allowed_speeds = ETH_LINK_SPEED_100M | ETH_LINK_SPEED_1G |
			ETH_LINK_SPEED_10G;
	}

	link_speeds = &dev->data->dev_conf.link_speeds;

	/* Ignore autoneg flag bit and check the validity of 
	 * link_speed 
	 */
	if (((*link_speeds) >> 1) & ~(allowed_speeds >> 1)) {
		PMD_INIT_LOG(ERR, "Invalid link setting");
		goto error;
	}

	speed = 0x0;
	if (*link_speeds == ETH_LINK_SPEED_AUTONEG) {
		switch (hw->mac.type) {
		case yusur2_mac_82598EB:
			speed = YUSUR2_LINK_SPEED_82598_AUTONEG;
			break;
		case yusur2_mac_82599EB:
		case yusur2_mac_X540:
			speed = YUSUR2_LINK_SPEED_82599_AUTONEG;
			break;
		case yusur2_mac_X550:
		case yusur2_mac_X550EM_x:
		case yusur2_mac_X550EM_a:
			speed = YUSUR2_LINK_SPEED_X550_AUTONEG;
			break;
		default:
			speed = YUSUR2_LINK_SPEED_82599_AUTONEG;
		}
	} else {
		if (*link_speeds & ETH_LINK_SPEED_10G)
			speed |= YUSUR2_LINK_SPEED_10GB_FULL;
		if (*link_speeds & ETH_LINK_SPEED_5G)
			speed |= YUSUR2_LINK_SPEED_5GB_FULL;
		if (*link_speeds & ETH_LINK_SPEED_2_5G)
			speed |= YUSUR2_LINK_SPEED_2_5GB_FULL;
		if (*link_speeds & ETH_LINK_SPEED_1G)
			speed |= YUSUR2_LINK_SPEED_1GB_FULL;
		if (*link_speeds & ETH_LINK_SPEED_100M)
			speed |= YUSUR2_LINK_SPEED_100_FULL;
		if (*link_speeds & ETH_LINK_SPEED_10M)
			speed |= YUSUR2_LINK_SPEED_10_FULL;
	}

	err = yusur2_setup_link(hw, speed, link_up);
	if (err)
		goto error;

skip_link_setup:

	if (rte_intr_allow_others(intr_handle)) {
		/* check if lsc interrupt is enabled */
		if (dev->data->dev_conf.intr_conf.lsc != 0)
			yusur2_dev_lsc_interrupt_setup(dev, TRUE);
		else
			yusur2_dev_lsc_interrupt_setup(dev, FALSE);
		yusur2_dev_macsec_interrupt_setup(dev);
	} else {
		rte_intr_callback_unregister(intr_handle,
					     yusur2_dev_interrupt_handler, dev);
		if (dev->data->dev_conf.intr_conf.lsc != 0)
			PMD_INIT_LOG(INFO, "lsc won't enable because of"
				     " no intr multiplex");
	}

	/* check if rxq interrupt is enabled */
	if (dev->data->dev_conf.intr_conf.rxq != 0 &&
	    rte_intr_dp_is_en(intr_handle))
		yusur2_dev_rxq_interrupt_setup(dev);

	/* enable uio/vfio intr/eventfd mapping */
	rte_intr_enable(intr_handle);

	/* resume enabled intr since hw reset */
	yusur2_enable_intr(dev);
	yusur2_l2_tunnel_conf(dev);
	yusur2_filter_restore(dev);

	if (tm_conf->root && !tm_conf->committed)
		PMD_DRV_LOG(WARNING,
			    "please call hierarchy_commit() "
			    "before starting the port");

	/* wait for the controller to acquire link */
	err = yusur2_wait_for_link_up(hw);
	if (err)
		goto error;

	/*
	 * Update link status right before return, because it may
	 * start link configuration process in a separate thread.
	 */
	yusur2_dev_link_update(dev, 0);

	/* setup the macsec setting register */
	if (macsec_setting->offload_en)
		yusur2_dev_macsec_register_enable(dev, macsec_setting);

	return 0;

error:
	PMD_INIT_LOG(ERR, "failure in yusur2_dev_start(): %d", err);
	yusur2_dev_clear_queues(dev);
	return -EIO;
}

/*
 * Stop device: disable rx and tx functions to allow for reconfiguring.
 */
static void
yusur2_dev_stop(struct rte_eth_dev *dev)
{
	struct rte_eth_link link;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	int vf;
	struct yusur2_tm_conf *tm_conf =
		YUSUR2_DEV_PRIVATE_TO_TM_CONF(dev->data->dev_private);

	if (hw->adapter_stopped)
		return;

	PMD_INIT_FUNC_TRACE();

	yusur2_dev_wait_setup_link_complete(dev, 0);

	/* disable interrupts */
	yusur2_disable_intr(hw);

	/* reset the NIC */
	yusur2_pf_reset_hw(hw);
	hw->adapter_stopped = 0;

	/* stop adapter */
	yusur2_stop_adapter(hw);

	for (vf = 0; vfinfo != NULL && vf < pci_dev->max_vfs; vf++)
		vfinfo[vf].clear_to_send = false;

	if (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper) {
		/* Turn off the copper */
		yusur2_set_phy_power(hw, false);
	} else {
		/* Turn off the laser */
		yusur2_disable_tx_laser(hw);
	}

	yusur2_dev_clear_queues(dev);

	/* Clear stored conf */
	dev->data->scattered_rx = 0;
	dev->data->lro = 0;

	/* Clear recorded link status */
	memset(&link, 0, sizeof(link));
	rte_eth_linkstatus_set(dev, &link);

	if (!rte_intr_allow_others(intr_handle))
		/* resume to the default handler */
		rte_intr_callback_register(intr_handle,
					   yusur2_dev_interrupt_handler,
					   (void *)dev);

	/* Clean datapath event and queue/vec mapping */
	rte_intr_efd_disable(intr_handle);
	if (intr_handle->intr_vec != NULL) {
		rte_free(intr_handle->intr_vec);
		intr_handle->intr_vec = NULL;
	}

	/* reset hierarchy commit */
	tm_conf->committed = false;

	adapter->rss_reta_updated = 0;

	hw->adapter_stopped = true;
}

/*
 * Set device link up: enable tx.
 */
static int
yusur2_dev_set_link_up(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (hw->mac.type == yusur2_mac_82599EB) {
#ifdef RTE_LIBRTE_YUSUR2_BYPASS
		if (hw->device_id == YUSUR2_DEV_ID_82599_BYPASS) {
			/* Not suported in bypass mode */
			PMD_INIT_LOG(ERR, "Set link up is not supported "
				     "by device id 0x%x", hw->device_id);
			return -ENOTSUP;
		}
#endif
	}

	if (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper) {
		/* Turn on the copper */
		yusur2_set_phy_power(hw, true);
	} else {
		/* Turn on the laser */
		yusur2_enable_tx_laser(hw);
		yusur2_dev_link_update(dev, 0);
	}

	return 0;
}

/*
 * Set device link down: disable tx.
 */
static int
yusur2_dev_set_link_down(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (hw->mac.type == yusur2_mac_82599EB) {
#ifdef RTE_LIBRTE_YUSUR2_BYPASS
		if (hw->device_id == YUSUR2_DEV_ID_82599_BYPASS) {
			/* Not suported in bypass mode */
			PMD_INIT_LOG(ERR, "Set link down is not supported "
				     "by device id 0x%x", hw->device_id);
			return -ENOTSUP;
		}
#endif
	}

	if (hw->mac.ops.get_media_type(hw) == yusur2_media_type_copper) {
		/* Turn off the copper */
		yusur2_set_phy_power(hw, false);
	} else {
		/* Turn off the laser */
		yusur2_disable_tx_laser(hw);
		yusur2_dev_link_update(dev, 0);
	}

	return 0;
}

/*
 * Reset and stop device.
 */
static void
yusur2_dev_close(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	int retries = 0;
	int ret;

	PMD_INIT_FUNC_TRACE();

	yusur2_pf_reset_hw(hw);

	yusur2_dev_stop(dev);

	yusur2_dev_free_queues(dev);

	yusur2_disable_pcie_master(hw);

	/* reprogram the RAR[0] in case user changed it. */
	yusur2_set_rar(hw, 0, hw->mac.addr, 0, YUSUR2_RAH_AV);

	dev->dev_ops = NULL;
	dev->rx_pkt_burst = NULL;
	dev->tx_pkt_burst = NULL;

	/* Unlock any pending hardware semaphore */
	yusur2_swfw_lock_reset(hw);

	/* disable uio intr before callback unregister */
	rte_intr_disable(intr_handle);

	do {
		ret = rte_intr_callback_unregister(intr_handle,
				yusur2_dev_interrupt_handler, dev);
		if (ret >= 0 || ret == -ENOENT) {
			break;
		} else if (ret != -EAGAIN) {
			PMD_INIT_LOG(ERR,
				"intr callback unregister failed: %d",
				ret);
		}
		rte_delay_ms(100);
	} while (retries++ < (10 + YUSUR2_LINK_UP_TIME));

	/* cancel the delay handler before remove dev */
	rte_eal_alarm_cancel(yusur2_dev_interrupt_delayed_handler, dev);

	/* uninitialize PF if max_vfs not zero */
	yusur2_pf_host_uninit(dev);

	/* remove all the fdir filters & hash */
	yusur2_fdir_filter_uninit(dev);

	/* remove all the L2 tunnel filters & hash */
	yusur2_l2_tn_filter_uninit(dev);

	/* Remove all ntuple filters of the device */
	yusur2_ntuple_filter_uninit(dev);

	/* clear all the filters list */
	yusur2_filterlist_flush();

	/* Remove all Traffic Manager configuration */
	yusur2_tm_conf_uninit(dev);

#ifdef RTE_LIBRTE_SECURITY
	rte_free(dev->security_ctx);
#endif

}

/*
 * Reset PF device.
 */
static int
yusur2_dev_reset(struct rte_eth_dev *dev)
{
	int ret;

	/* When a DPDK PMD PF begin to reset PF port, it should notify all
	 * its VF to make them align with it. The detailed notification
	 * mechanism is PMD specific. As to yusur2 PF, it is rather complex.
	 * To avoid unexpected behavior in VF, currently reset of PF with
	 * SR-IOV activation is not supported. It might be supported later.
	 */
	if (dev->data->sriov.active)
		return -ENOTSUP;

	ret = eth_yusur2_dev_uninit(dev);
	if (ret)
		return ret;

	ret = eth_yusur2_dev_init(dev, NULL);

	return ret;
}

static void
yusur2_read_stats_registers(struct yusur2_hw *hw,
			   struct yusur2_hw_stats *hw_stats,
			   struct yusur2_macsec_stats *macsec_stats,
			   uint64_t *total_missed_rx, uint64_t *total_qbrc,
			   uint64_t *total_qprc, uint64_t *total_qprdc)
{
	uint32_t bprc, lxon, lxoff, total;
	uint32_t delta_gprc = 0;
	unsigned i;
	/* Workaround for RX byte count not including CRC bytes when CRC
	 * strip is enabled. CRC bytes are removed from counters when crc_strip
	 * is disabled.
	 */
	int crc_strip = (YUSUR2_READ_REG(hw, YUSUR2_HLREG0) &
			YUSUR2_HLREG0_RXCRCSTRP);

	hw_stats->crcerrs += YUSUR2_READ_REG(hw, YUSUR2_CRCERRS);
	hw_stats->illerrc += YUSUR2_READ_REG(hw, YUSUR2_ILLERRC);
	hw_stats->errbc += YUSUR2_READ_REG(hw, YUSUR2_ERRBC);
	hw_stats->mspdc += YUSUR2_READ_REG(hw, YUSUR2_MSPDC);

	for (i = 0; i < 8; i++) {
		uint32_t mp = YUSUR2_READ_REG(hw, YUSUR2_MPC(i));

		/* global total per queue */
		hw_stats->mpc[i] += mp;
		/* Running comprehensive total for stats display */
		*total_missed_rx += hw_stats->mpc[i];
		if (hw->mac.type == yusur2_mac_82598EB) {
			hw_stats->rnbc[i] +=
			    YUSUR2_READ_REG(hw, YUSUR2_RNBC(i));
			hw_stats->pxonrxc[i] +=
				YUSUR2_READ_REG(hw, YUSUR2_PXONRXC(i));
			hw_stats->pxoffrxc[i] +=
				YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXC(i));
		} else {
			hw_stats->pxonrxc[i] +=
				YUSUR2_READ_REG(hw, YUSUR2_PXONRXCNT(i));
			hw_stats->pxoffrxc[i] +=
				YUSUR2_READ_REG(hw, YUSUR2_PXOFFRXCNT(i));
			hw_stats->pxon2offc[i] +=
				YUSUR2_READ_REG(hw, YUSUR2_PXON2OFFCNT(i));
		}
		hw_stats->pxontxc[i] +=
		    YUSUR2_READ_REG(hw, YUSUR2_PXONTXC(i));
		hw_stats->pxofftxc[i] +=
		    YUSUR2_READ_REG(hw, YUSUR2_PXOFFTXC(i));
	}
	for (i = 0; i < YUSUR2_QUEUE_STAT_COUNTERS; i++) {
		uint32_t delta_qprc = YUSUR2_READ_REG(hw, YUSUR2_QPRC(i));
		uint32_t delta_qptc = YUSUR2_READ_REG(hw, YUSUR2_QPTC(i));
		uint32_t delta_qprdc = YUSUR2_READ_REG(hw, YUSUR2_QPRDC(i));

		delta_gprc += delta_qprc;

		hw_stats->qprc[i] += delta_qprc;
		hw_stats->qptc[i] += delta_qptc;

		hw_stats->qbrc[i] += YUSUR2_READ_REG(hw, YUSUR2_QBRC_L(i));
		hw_stats->qbrc[i] +=
		    ((uint64_t)YUSUR2_READ_REG(hw, YUSUR2_QBRC_H(i)) << 32);
		if (crc_strip == 0)
			hw_stats->qbrc[i] -= delta_qprc * RTE_ETHER_CRC_LEN;

		hw_stats->qbtc[i] += YUSUR2_READ_REG(hw, YUSUR2_QBTC_L(i));
		hw_stats->qbtc[i] +=
		    ((uint64_t)YUSUR2_READ_REG(hw, YUSUR2_QBTC_H(i)) << 32);

		hw_stats->qprdc[i] += delta_qprdc;
		*total_qprdc += hw_stats->qprdc[i];

		*total_qprc += hw_stats->qprc[i];
		*total_qbrc += hw_stats->qbrc[i];
	}
	hw_stats->mlfc += YUSUR2_READ_REG(hw, YUSUR2_MLFC);
	hw_stats->mrfc += YUSUR2_READ_REG(hw, YUSUR2_MRFC);
	hw_stats->rlec += YUSUR2_READ_REG(hw, YUSUR2_RLEC);

	/*
	 * An errata states that gprc actually counts good + missed packets:
	 * Workaround to set gprc to summated queue packet receives
	 */
	hw_stats->gprc = *total_qprc;

	if (hw->mac.type != yusur2_mac_82598EB) {
		hw_stats->gorc += YUSUR2_READ_REG(hw, YUSUR2_GORCL);
		hw_stats->gorc += ((u64)YUSUR2_READ_REG(hw, YUSUR2_GORCH) << 32);
		hw_stats->gotc += YUSUR2_READ_REG(hw, YUSUR2_GOTCL);
		hw_stats->gotc += ((u64)YUSUR2_READ_REG(hw, YUSUR2_GOTCH) << 32);
		hw_stats->tor += YUSUR2_READ_REG(hw, YUSUR2_TORL);
		hw_stats->tor += ((u64)YUSUR2_READ_REG(hw, YUSUR2_TORH) << 32);
		hw_stats->lxonrxc += YUSUR2_READ_REG(hw, YUSUR2_LXONRXCNT);
		hw_stats->lxoffrxc += YUSUR2_READ_REG(hw, YUSUR2_LXOFFRXCNT);
	} else {
		hw_stats->lxonrxc += YUSUR2_READ_REG(hw, YUSUR2_LXONRXC);
		hw_stats->lxoffrxc += YUSUR2_READ_REG(hw, YUSUR2_LXOFFRXC);
		/* 82598 only has a counter in the high register */
		hw_stats->gorc += YUSUR2_READ_REG(hw, YUSUR2_GORCH);
		hw_stats->gotc += YUSUR2_READ_REG(hw, YUSUR2_GOTCH);
		hw_stats->tor += YUSUR2_READ_REG(hw, YUSUR2_TORH);
	}
	uint64_t old_tpr = hw_stats->tpr;

	hw_stats->tpr += YUSUR2_READ_REG(hw, YUSUR2_TPR);
	hw_stats->tpt += YUSUR2_READ_REG(hw, YUSUR2_TPT);

	if (crc_strip == 0)
		hw_stats->gorc -= delta_gprc * RTE_ETHER_CRC_LEN;

	uint64_t delta_gptc = YUSUR2_READ_REG(hw, YUSUR2_GPTC);
	hw_stats->gptc += delta_gptc;
	hw_stats->gotc -= delta_gptc * RTE_ETHER_CRC_LEN;
	hw_stats->tor -= (hw_stats->tpr - old_tpr) * RTE_ETHER_CRC_LEN;

	/*
	 * Workaround: mprc hardware is incorrectly counting
	 * broadcasts, so for now we subtract those.
	 */
	bprc = YUSUR2_READ_REG(hw, YUSUR2_BPRC);
	hw_stats->bprc += bprc;
	hw_stats->mprc += YUSUR2_READ_REG(hw, YUSUR2_MPRC);
	if (hw->mac.type == yusur2_mac_82598EB)
		hw_stats->mprc -= bprc;

	hw_stats->prc64 += YUSUR2_READ_REG(hw, YUSUR2_PRC64);
	hw_stats->prc127 += YUSUR2_READ_REG(hw, YUSUR2_PRC127);
	hw_stats->prc255 += YUSUR2_READ_REG(hw, YUSUR2_PRC255);
	hw_stats->prc511 += YUSUR2_READ_REG(hw, YUSUR2_PRC511);
	hw_stats->prc1023 += YUSUR2_READ_REG(hw, YUSUR2_PRC1023);
	hw_stats->prc1522 += YUSUR2_READ_REG(hw, YUSUR2_PRC1522);

	lxon = YUSUR2_READ_REG(hw, YUSUR2_LXONTXC);
	hw_stats->lxontxc += lxon;
	lxoff = YUSUR2_READ_REG(hw, YUSUR2_LXOFFTXC);
	hw_stats->lxofftxc += lxoff;
	total = lxon + lxoff;

	hw_stats->mptc += YUSUR2_READ_REG(hw, YUSUR2_MPTC);
	hw_stats->ptc64 += YUSUR2_READ_REG(hw, YUSUR2_PTC64);
	hw_stats->gptc -= total;
	hw_stats->mptc -= total;
	hw_stats->ptc64 -= total;
	hw_stats->gotc -= total * RTE_ETHER_MIN_LEN;

	hw_stats->ruc += YUSUR2_READ_REG(hw, YUSUR2_RUC);
	hw_stats->rfc += YUSUR2_READ_REG(hw, YUSUR2_RFC);
	hw_stats->roc += YUSUR2_READ_REG(hw, YUSUR2_ROC);
	hw_stats->rjc += YUSUR2_READ_REG(hw, YUSUR2_RJC);
	hw_stats->mngprc += YUSUR2_READ_REG(hw, YUSUR2_MNGPRC);
	hw_stats->mngpdc += YUSUR2_READ_REG(hw, YUSUR2_MNGPDC);
	hw_stats->mngptc += YUSUR2_READ_REG(hw, YUSUR2_MNGPTC);
	hw_stats->ptc127 += YUSUR2_READ_REG(hw, YUSUR2_PTC127);
	hw_stats->ptc255 += YUSUR2_READ_REG(hw, YUSUR2_PTC255);
	hw_stats->ptc511 += YUSUR2_READ_REG(hw, YUSUR2_PTC511);
	hw_stats->ptc1023 += YUSUR2_READ_REG(hw, YUSUR2_PTC1023);
	hw_stats->ptc1522 += YUSUR2_READ_REG(hw, YUSUR2_PTC1522);
	hw_stats->bptc += YUSUR2_READ_REG(hw, YUSUR2_BPTC);
	hw_stats->xec += YUSUR2_READ_REG(hw, YUSUR2_XEC);
	hw_stats->fccrc += YUSUR2_READ_REG(hw, YUSUR2_FCCRC);
	hw_stats->fclast += YUSUR2_READ_REG(hw, YUSUR2_FCLAST);
	/* Only read FCOE on 82599 */
	if (hw->mac.type != yusur2_mac_82598EB) {
		hw_stats->fcoerpdc += YUSUR2_READ_REG(hw, YUSUR2_FCOERPDC);
		hw_stats->fcoeprc += YUSUR2_READ_REG(hw, YUSUR2_FCOEPRC);
		hw_stats->fcoeptc += YUSUR2_READ_REG(hw, YUSUR2_FCOEPTC);
		hw_stats->fcoedwrc += YUSUR2_READ_REG(hw, YUSUR2_FCOEDWRC);
		hw_stats->fcoedwtc += YUSUR2_READ_REG(hw, YUSUR2_FCOEDWTC);
	}

	/* Flow Director Stats registers */
	if (hw->mac.type != yusur2_mac_82598EB) {
		hw_stats->fdirmatch += YUSUR2_READ_REG(hw, YUSUR2_FDIRMATCH);
		hw_stats->fdirmiss += YUSUR2_READ_REG(hw, YUSUR2_FDIRMISS);
		hw_stats->fdirustat_add += YUSUR2_READ_REG(hw,
					YUSUR2_FDIRUSTAT) & 0xFFFF;
		hw_stats->fdirustat_remove += (YUSUR2_READ_REG(hw,
					YUSUR2_FDIRUSTAT) >> 16) & 0xFFFF;
		hw_stats->fdirfstat_fadd += YUSUR2_READ_REG(hw,
					YUSUR2_FDIRFSTAT) & 0xFFFF;
		hw_stats->fdirfstat_fremove += (YUSUR2_READ_REG(hw,
					YUSUR2_FDIRFSTAT) >> 16) & 0xFFFF;
	}
	/* MACsec Stats registers */
	macsec_stats->out_pkts_untagged += YUSUR2_READ_REG(hw, YUSUR2_LSECTXUT);
	macsec_stats->out_pkts_encrypted +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECTXPKTE);
	macsec_stats->out_pkts_protected +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECTXPKTP);
	macsec_stats->out_octets_encrypted +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECTXOCTE);
	macsec_stats->out_octets_protected +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECTXOCTP);
	macsec_stats->in_pkts_untagged += YUSUR2_READ_REG(hw, YUSUR2_LSECRXUT);
	macsec_stats->in_pkts_badtag += YUSUR2_READ_REG(hw, YUSUR2_LSECRXBAD);
	macsec_stats->in_pkts_nosci += YUSUR2_READ_REG(hw, YUSUR2_LSECRXNOSCI);
	macsec_stats->in_pkts_unknownsci +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECRXUNSCI);
	macsec_stats->in_octets_decrypted +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECRXOCTD);
	macsec_stats->in_octets_validated +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECRXOCTV);
	macsec_stats->in_pkts_unchecked += YUSUR2_READ_REG(hw, YUSUR2_LSECRXUNCH);
	macsec_stats->in_pkts_delayed += YUSUR2_READ_REG(hw, YUSUR2_LSECRXDELAY);
	macsec_stats->in_pkts_late += YUSUR2_READ_REG(hw, YUSUR2_LSECRXLATE);
	for (i = 0; i < 2; i++) {
		macsec_stats->in_pkts_ok +=
			YUSUR2_READ_REG(hw, YUSUR2_LSECRXOK(i));
		macsec_stats->in_pkts_invalid +=
			YUSUR2_READ_REG(hw, YUSUR2_LSECRXINV(i));
		macsec_stats->in_pkts_notvalid +=
			YUSUR2_READ_REG(hw, YUSUR2_LSECRXNV(i));
	}
	macsec_stats->in_pkts_unusedsa += YUSUR2_READ_REG(hw, YUSUR2_LSECRXUNSA);
	macsec_stats->in_pkts_notusingsa +=
		YUSUR2_READ_REG(hw, YUSUR2_LSECRXNUSA);
}

/*
 * This function is based on yusur2_update_stats_counters() in yusur2/yusur2.c
 */
static int
yusur2_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
	struct yusur2_hw *hw =
			YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_hw_stats *hw_stats =
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	struct yusur2_macsec_stats *macsec_stats =
			YUSUR2_DEV_PRIVATE_TO_MACSEC_STATS(
				dev->data->dev_private);
	uint64_t total_missed_rx, total_qbrc, total_qprc, total_qprdc;
	unsigned i;

	total_missed_rx = 0;
	total_qbrc = 0;
	total_qprc = 0;
	total_qprdc = 0;

	yusur2_read_stats_registers(hw, hw_stats, macsec_stats, &total_missed_rx,
			&total_qbrc, &total_qprc, &total_qprdc);

	if (stats == NULL)
		return -EINVAL;

	/* Fill out the rte_eth_stats statistics structure */
	stats->ipackets = total_qprc;
	stats->ibytes = total_qbrc;
	stats->opackets = hw_stats->gptc;
	stats->obytes = hw_stats->gotc;

	for (i = 0; i < YUSUR2_QUEUE_STAT_COUNTERS; i++) {
		stats->q_ipackets[i] = hw_stats->qprc[i];
		stats->q_opackets[i] = hw_stats->qptc[i];
		stats->q_ibytes[i] = hw_stats->qbrc[i];
		stats->q_obytes[i] = hw_stats->qbtc[i];
		stats->q_errors[i] = hw_stats->qprdc[i];
	}

	/* Rx Errors */
	stats->imissed  = total_missed_rx;
	stats->ierrors  = hw_stats->crcerrs +
			  hw_stats->mspdc +
			  hw_stats->rlec +
			  hw_stats->ruc +
			  hw_stats->roc +
			  hw_stats->illerrc +
			  hw_stats->errbc +
			  hw_stats->rfc +
			  hw_stats->fccrc +
			  hw_stats->fclast;

	/* Tx Errors */
	stats->oerrors  = 0;
	return 0;
}

static int
yusur2_dev_stats_reset(struct rte_eth_dev *dev)
{
	struct yusur2_hw_stats *stats =
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);

	/* HW registers are cleared on read */
	yusur2_dev_stats_get(dev, NULL);

	/* Reset software totals */
	memset(stats, 0, sizeof(*stats));

	return 0;
}

/* This function calculates the number of xstats based on the current config */
static unsigned
yusur2_xstats_calc_num(void) {
	return YUSUR2_NB_HW_STATS + YUSUR2_NB_MACSEC_STATS +
		(YUSUR2_NB_RXQ_PRIO_STATS * YUSUR2_NB_RXQ_PRIO_VALUES) +
		(YUSUR2_NB_TXQ_PRIO_STATS * YUSUR2_NB_TXQ_PRIO_VALUES);
}

static int yusur2_dev_xstats_get_names(__rte_unused struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, __rte_unused unsigned int size)
{
	const unsigned cnt_stats = yusur2_xstats_calc_num();
	unsigned stat, i, count;

	if (xstats_names != NULL) {
		count = 0;

		/* Note: limit >= cnt_stats checked upstream
		 * in rte_eth_xstats_names()
		 */

		/* Extended stats from yusur2_hw_stats */
		for (i = 0; i < YUSUR2_NB_HW_STATS; i++) {
			strlcpy(xstats_names[count].name,
				rte_yusur2_stats_strings[i].name,
				sizeof(xstats_names[count].name));
			count++;
		}

		/* MACsec Stats */
		for (i = 0; i < YUSUR2_NB_MACSEC_STATS; i++) {
			strlcpy(xstats_names[count].name,
				rte_yusur2_macsec_strings[i].name,
				sizeof(xstats_names[count].name));
			count++;
		}

		/* RX Priority Stats */
		for (stat = 0; stat < YUSUR2_NB_RXQ_PRIO_STATS; stat++) {
			for (i = 0; i < YUSUR2_NB_RXQ_PRIO_VALUES; i++) {
				snprintf(xstats_names[count].name,
					sizeof(xstats_names[count].name),
					"rx_priority%u_%s", i,
					rte_yusur2_rxq_strings[stat].name);
				count++;
			}
		}

		/* TX Priority Stats */
		for (stat = 0; stat < YUSUR2_NB_TXQ_PRIO_STATS; stat++) {
			for (i = 0; i < YUSUR2_NB_TXQ_PRIO_VALUES; i++) {
				snprintf(xstats_names[count].name,
					sizeof(xstats_names[count].name),
					"tx_priority%u_%s", i,
					rte_yusur2_txq_strings[stat].name);
				count++;
			}
		}
	}
	return cnt_stats;
}

static int yusur2_dev_xstats_get_names_by_id(
	struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names,
	const uint64_t *ids,
	unsigned int limit)
{
	if (!ids) {
		const unsigned int cnt_stats = yusur2_xstats_calc_num();
		unsigned int stat, i, count;

		if (xstats_names != NULL) {
			count = 0;

			/* Note: limit >= cnt_stats checked upstream
			 * in rte_eth_xstats_names()
			 */

			/* Extended stats from yusur2_hw_stats */
			for (i = 0; i < YUSUR2_NB_HW_STATS; i++) {
				strlcpy(xstats_names[count].name,
					rte_yusur2_stats_strings[i].name,
					sizeof(xstats_names[count].name));
				count++;
			}

			/* MACsec Stats */
			for (i = 0; i < YUSUR2_NB_MACSEC_STATS; i++) {
				strlcpy(xstats_names[count].name,
					rte_yusur2_macsec_strings[i].name,
					sizeof(xstats_names[count].name));
				count++;
			}

			/* RX Priority Stats */
			for (stat = 0; stat < YUSUR2_NB_RXQ_PRIO_STATS; stat++) {
				for (i = 0; i < YUSUR2_NB_RXQ_PRIO_VALUES; i++) {
					snprintf(xstats_names[count].name,
					    sizeof(xstats_names[count].name),
					    "rx_priority%u_%s", i,
					    rte_yusur2_rxq_strings[stat].name);
					count++;
				}
			}

			/* TX Priority Stats */
			for (stat = 0; stat < YUSUR2_NB_TXQ_PRIO_STATS; stat++) {
				for (i = 0; i < YUSUR2_NB_TXQ_PRIO_VALUES; i++) {
					snprintf(xstats_names[count].name,
					    sizeof(xstats_names[count].name),
					    "tx_priority%u_%s", i,
					    rte_yusur2_txq_strings[stat].name);
					count++;
				}
			}
		}
		return cnt_stats;
	}

	uint16_t i;
	uint16_t size = yusur2_xstats_calc_num();
	struct rte_eth_xstat_name xstats_names_copy[size];

	yusur2_dev_xstats_get_names_by_id(dev, xstats_names_copy, NULL,
			size);

	for (i = 0; i < limit; i++) {
		if (ids[i] >= size) {
			PMD_INIT_LOG(ERR, "id value isn't valid");
			return -1;
		}
		strcpy(xstats_names[i].name,
				xstats_names_copy[ids[i]].name);
	}
	return limit;
}

static int yusur2vf_dev_xstats_get_names(__rte_unused struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, unsigned limit)
{
	unsigned i;

	if (limit < YUSUR2VF_NB_XSTATS && xstats_names != NULL)
		return -ENOMEM;

	if (xstats_names != NULL)
		for (i = 0; i < YUSUR2VF_NB_XSTATS; i++)
			strlcpy(xstats_names[i].name,
				rte_yusur2vf_stats_strings[i].name,
				sizeof(xstats_names[i].name));
	return YUSUR2VF_NB_XSTATS;
}

static int
yusur2_dev_xstats_get(struct rte_eth_dev *dev, struct rte_eth_xstat *xstats,
					 unsigned n)
{
	struct yusur2_hw *hw =
			YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_hw_stats *hw_stats =
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	struct yusur2_macsec_stats *macsec_stats =
			YUSUR2_DEV_PRIVATE_TO_MACSEC_STATS(
				dev->data->dev_private);
	uint64_t total_missed_rx, total_qbrc, total_qprc, total_qprdc;
	unsigned i, stat, count = 0;

	count = yusur2_xstats_calc_num();

	if (n < count)
		return count;

	total_missed_rx = 0;
	total_qbrc = 0;
	total_qprc = 0;
	total_qprdc = 0;

	yusur2_read_stats_registers(hw, hw_stats, macsec_stats, &total_missed_rx,
			&total_qbrc, &total_qprc, &total_qprdc);

	/* If this is a reset xstats is NULL, and we have cleared the
	 * registers by reading them.
	 */
	if (!xstats)
		return 0;

	/* Extended stats from yusur2_hw_stats */
	count = 0;
	for (i = 0; i < YUSUR2_NB_HW_STATS; i++) {
		xstats[count].value = *(uint64_t *)(((char *)hw_stats) +
				rte_yusur2_stats_strings[i].offset);
		xstats[count].id = count;
		count++;
	}

	/* MACsec Stats */
	for (i = 0; i < YUSUR2_NB_MACSEC_STATS; i++) {
		xstats[count].value = *(uint64_t *)(((char *)macsec_stats) +
				rte_yusur2_macsec_strings[i].offset);
		xstats[count].id = count;
		count++;
	}

	/* RX Priority Stats */
	for (stat = 0; stat < YUSUR2_NB_RXQ_PRIO_STATS; stat++) {
		for (i = 0; i < YUSUR2_NB_RXQ_PRIO_VALUES; i++) {
			xstats[count].value = *(uint64_t *)(((char *)hw_stats) +
					rte_yusur2_rxq_strings[stat].offset +
					(sizeof(uint64_t) * i));
			xstats[count].id = count;
			count++;
		}
	}

	/* TX Priority Stats */
	for (stat = 0; stat < YUSUR2_NB_TXQ_PRIO_STATS; stat++) {
		for (i = 0; i < YUSUR2_NB_TXQ_PRIO_VALUES; i++) {
			xstats[count].value = *(uint64_t *)(((char *)hw_stats) +
					rte_yusur2_txq_strings[stat].offset +
					(sizeof(uint64_t) * i));
			xstats[count].id = count;
			count++;
		}
	}
	return count;
}

static int
yusur2_dev_xstats_get_by_id(struct rte_eth_dev *dev, const uint64_t *ids,
		uint64_t *values, unsigned int n)
{
	if (!ids) {
		struct yusur2_hw *hw =
				YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
		struct yusur2_hw_stats *hw_stats =
				YUSUR2_DEV_PRIVATE_TO_STATS(
						dev->data->dev_private);
		struct yusur2_macsec_stats *macsec_stats =
				YUSUR2_DEV_PRIVATE_TO_MACSEC_STATS(
					dev->data->dev_private);
		uint64_t total_missed_rx, total_qbrc, total_qprc, total_qprdc;
		unsigned int i, stat, count = 0;

		count = yusur2_xstats_calc_num();

		if (!ids && n < count)
			return count;

		total_missed_rx = 0;
		total_qbrc = 0;
		total_qprc = 0;
		total_qprdc = 0;

		yusur2_read_stats_registers(hw, hw_stats, macsec_stats,
				&total_missed_rx, &total_qbrc, &total_qprc,
				&total_qprdc);

		/* If this is a reset xstats is NULL, and we have cleared the
		 * registers by reading them.
		 */
		if (!ids && !values)
			return 0;

		/* Extended stats from yusur2_hw_stats */
		count = 0;
		for (i = 0; i < YUSUR2_NB_HW_STATS; i++) {
			values[count] = *(uint64_t *)(((char *)hw_stats) +
					rte_yusur2_stats_strings[i].offset);
			count++;
		}

		/* MACsec Stats */
		for (i = 0; i < YUSUR2_NB_MACSEC_STATS; i++) {
			values[count] = *(uint64_t *)(((char *)macsec_stats) +
					rte_yusur2_macsec_strings[i].offset);
			count++;
		}

		/* RX Priority Stats */
		for (stat = 0; stat < YUSUR2_NB_RXQ_PRIO_STATS; stat++) {
			for (i = 0; i < YUSUR2_NB_RXQ_PRIO_VALUES; i++) {
				values[count] =
					*(uint64_t *)(((char *)hw_stats) +
					rte_yusur2_rxq_strings[stat].offset +
					(sizeof(uint64_t) * i));
				count++;
			}
		}

		/* TX Priority Stats */
		for (stat = 0; stat < YUSUR2_NB_TXQ_PRIO_STATS; stat++) {
			for (i = 0; i < YUSUR2_NB_TXQ_PRIO_VALUES; i++) {
				values[count] =
					*(uint64_t *)(((char *)hw_stats) +
					rte_yusur2_txq_strings[stat].offset +
					(sizeof(uint64_t) * i));
				count++;
			}
		}
		return count;
	}

	uint16_t i;
	uint16_t size = yusur2_xstats_calc_num();
	uint64_t values_copy[size];

	yusur2_dev_xstats_get_by_id(dev, NULL, values_copy, size);

	for (i = 0; i < n; i++) {
		if (ids[i] >= size) {
			PMD_INIT_LOG(ERR, "id value isn't valid");
			return -1;
		}
		values[i] = values_copy[ids[i]];
	}
	return n;
}

static int
yusur2_dev_xstats_reset(struct rte_eth_dev *dev)
{
	struct yusur2_hw_stats *stats =
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	struct yusur2_macsec_stats *macsec_stats =
			YUSUR2_DEV_PRIVATE_TO_MACSEC_STATS(
				dev->data->dev_private);

	unsigned count = yusur2_xstats_calc_num();

	/* HW registers are cleared on read */
	yusur2_dev_xstats_get(dev, NULL, count);

	/* Reset software totals */
	memset(stats, 0, sizeof(*stats));
	memset(macsec_stats, 0, sizeof(*macsec_stats));

	return 0;
}

static void
yusur2vf_update_stats(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2vf_hw_stats *hw_stats = (struct yusur2vf_hw_stats *)
			  YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);

	/* Good Rx packet, include VF loopback */
	UPDATE_VF_STAT(YUSUR2_VFGPRC,
	    hw_stats->last_vfgprc, hw_stats->vfgprc);

	/* Good Rx octets, include VF loopback */
	UPDATE_VF_STAT_36BIT(YUSUR2_VFGORC_LSB, YUSUR2_VFGORC_MSB,
	    hw_stats->last_vfgorc, hw_stats->vfgorc);

	/* Good Tx packet, include VF loopback */
	UPDATE_VF_STAT(YUSUR2_VFGPTC,
	    hw_stats->last_vfgptc, hw_stats->vfgptc);

	/* Good Tx octets, include VF loopback */
	UPDATE_VF_STAT_36BIT(YUSUR2_VFGOTC_LSB, YUSUR2_VFGOTC_MSB,
	    hw_stats->last_vfgotc, hw_stats->vfgotc);

	/* Rx Multicst Packet */
	UPDATE_VF_STAT(YUSUR2_VFMPRC,
	    hw_stats->last_vfmprc, hw_stats->vfmprc);
}

static int
yusur2vf_dev_xstats_get(struct rte_eth_dev *dev, struct rte_eth_xstat *xstats,
		       unsigned n)
{
	struct yusur2vf_hw_stats *hw_stats = (struct yusur2vf_hw_stats *)
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	unsigned i;

	if (n < YUSUR2VF_NB_XSTATS)
		return YUSUR2VF_NB_XSTATS;

	yusur2vf_update_stats(dev);

	if (!xstats)
		return 0;

	/* Extended stats */
	for (i = 0; i < YUSUR2VF_NB_XSTATS; i++) {
		xstats[i].id = i;
		xstats[i].value = *(uint64_t *)(((char *)hw_stats) +
			rte_yusur2vf_stats_strings[i].offset);
	}

	return YUSUR2VF_NB_XSTATS;
}

static int
yusur2vf_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
	struct yusur2vf_hw_stats *hw_stats = (struct yusur2vf_hw_stats *)
			  YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);

	yusur2vf_update_stats(dev);

	if (stats == NULL)
		return -EINVAL;

	stats->ipackets = hw_stats->vfgprc;
	stats->ibytes = hw_stats->vfgorc;
	stats->opackets = hw_stats->vfgptc;
	stats->obytes = hw_stats->vfgotc;
	return 0;
}

static int
yusur2vf_dev_stats_reset(struct rte_eth_dev *dev)
{
	struct yusur2vf_hw_stats *hw_stats = (struct yusur2vf_hw_stats *)
			YUSUR2_DEV_PRIVATE_TO_STATS(dev->data->dev_private);

	/* Sync HW register to the last stats */
	yusur2vf_dev_stats_get(dev, NULL);

	/* reset HW current stats*/
	hw_stats->vfgprc = 0;
	hw_stats->vfgorc = 0;
	hw_stats->vfgptc = 0;
	hw_stats->vfgotc = 0;

	return 0;
}

static int
yusur2_fw_version_get(struct rte_eth_dev *dev, char *fw_version, size_t fw_size)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	u16 eeprom_verh, eeprom_verl;
	u32 etrack_id;
	int ret;

	yusur2_read_eeprom(hw, 0x2e, &eeprom_verh);
	yusur2_read_eeprom(hw, 0x2d, &eeprom_verl);

	etrack_id = (eeprom_verh << 16) | eeprom_verl;
	ret = snprintf(fw_version, fw_size, "0x%08x", etrack_id);

	ret += 1; /* add the size of '\0' */
	if (fw_size < (u32)ret)
		return ret;
	else
		return 0;
}

static int
yusur2_dev_info_get(struct rte_eth_dev *dev, struct rte_eth_dev_info *dev_info)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_conf *dev_conf = &dev->data->dev_conf;

	dev_info->max_rx_queues = (uint16_t)hw->mac.max_rx_queues;
	dev_info->max_tx_queues = (uint16_t)hw->mac.max_tx_queues;
	if (RTE_ETH_DEV_SRIOV(dev).active == 0) {
		/*
		 * When DCB/VT is off, maximum number of queues changes,
		 * except for 82598EB, which remains constant.
		 */
		if (dev_conf->txmode.mq_mode == ETH_MQ_TX_NONE &&
				hw->mac.type != yusur2_mac_82598EB)
			dev_info->max_tx_queues = YUSUR2_NONE_MODE_TX_NB_QUEUES;
	}
	dev_info->min_rx_bufsize = 1024; /* cf BSIZEPACKET in SRRCTL register */
	dev_info->max_rx_pktlen = 15872; /* includes CRC, cf MAXFRS register */
	dev_info->max_mac_addrs = hw->mac.num_rar_entries;
	dev_info->max_hash_mac_addrs = YUSUR2_VMDQ_NUM_UC_MAC;
	dev_info->max_vfs = pci_dev->max_vfs;
	if (hw->mac.type == yusur2_mac_82598EB)
		dev_info->max_vmdq_pools = ETH_16_POOLS;
	else
		dev_info->max_vmdq_pools = ETH_64_POOLS;
	dev_info->max_mtu =  dev_info->max_rx_pktlen - YUSUR2_ETH_OVERHEAD;
	dev_info->min_mtu = RTE_ETHER_MIN_MTU;
	dev_info->vmdq_queue_num = dev_info->max_rx_queues;
	dev_info->rx_queue_offload_capa = yusur2_get_rx_queue_offloads(dev);
	dev_info->rx_offload_capa = (yusur2_get_rx_port_offloads(dev) |
				     dev_info->rx_queue_offload_capa);
	dev_info->tx_queue_offload_capa = yusur2_get_tx_queue_offloads(dev);
	dev_info->tx_offload_capa = yusur2_get_tx_port_offloads(dev);

	dev_info->default_rxconf = (struct rte_eth_rxconf) {
		.rx_thresh = {
			.pthresh = YUSUR2_DEFAULT_RX_PTHRESH,
			.hthresh = YUSUR2_DEFAULT_RX_HTHRESH,
			.wthresh = YUSUR2_DEFAULT_RX_WTHRESH,
		},
		.rx_free_thresh = YUSUR2_DEFAULT_RX_FREE_THRESH,
		.rx_drop_en = 0,
		.offloads = 0,
	};

	dev_info->default_txconf = (struct rte_eth_txconf) {
		.tx_thresh = {
			.pthresh = YUSUR2_DEFAULT_TX_PTHRESH,
			.hthresh = YUSUR2_DEFAULT_TX_HTHRESH,
			.wthresh = YUSUR2_DEFAULT_TX_WTHRESH,
		},
		.tx_free_thresh = YUSUR2_DEFAULT_TX_FREE_THRESH,
		.tx_rs_thresh = YUSUR2_DEFAULT_TX_RSBIT_THRESH,
		.offloads = 0,
	};

	dev_info->rx_desc_lim = rx_desc_lim;
	dev_info->tx_desc_lim = tx_desc_lim;

	dev_info->hash_key_size = YUSUR2_HKEY_MAX_INDEX * sizeof(uint32_t);
	dev_info->reta_size = yusur2_reta_size_get(hw->mac.type);
	dev_info->flow_type_rss_offloads = YUSUR2_RSS_OFFLOAD_ALL;

	dev_info->speed_capa = ETH_LINK_SPEED_1G | ETH_LINK_SPEED_10G;
	if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T ||
			hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T_L)
		dev_info->speed_capa = ETH_LINK_SPEED_10M |
			ETH_LINK_SPEED_100M | ETH_LINK_SPEED_1G;

	if (hw->mac.type == yusur2_mac_X540 ||
	    hw->mac.type == yusur2_mac_X540_vf ||
	    hw->mac.type == yusur2_mac_X550 ||
	    hw->mac.type == yusur2_mac_X550_vf) {
		dev_info->speed_capa |= ETH_LINK_SPEED_100M;
	}
	if (hw->mac.type == yusur2_mac_X550) {
		dev_info->speed_capa |= ETH_LINK_SPEED_2_5G;
		dev_info->speed_capa |= ETH_LINK_SPEED_5G;
	}

	/* Driver-preferred Rx/Tx parameters */
	dev_info->default_rxportconf.burst_size = 32;
	dev_info->default_txportconf.burst_size = 32;
	dev_info->default_rxportconf.nb_queues = 1;
	dev_info->default_txportconf.nb_queues = 1;
	dev_info->default_rxportconf.ring_size = 256;
	dev_info->default_txportconf.ring_size = 256;

	return 0;
}

static const uint32_t *
yusur2_dev_supported_ptypes_get(struct rte_eth_dev *dev)
{
	static const uint32_t ptypes[] = {
		/* For non-vec functions,
		 * refers to yusur2_rxd_pkt_info_to_pkt_type();
		 * for vec functions,
		 * refers to _recv_raw_pkts_vec().
		 */
		RTE_PTYPE_L2_ETHER,
		RTE_PTYPE_L3_IPV4,
		RTE_PTYPE_L3_IPV4_EXT,
		RTE_PTYPE_L3_IPV6,
		RTE_PTYPE_L3_IPV6_EXT,
		RTE_PTYPE_L4_SCTP,
		RTE_PTYPE_L4_TCP,
		RTE_PTYPE_L4_UDP,
		RTE_PTYPE_TUNNEL_IP,
		RTE_PTYPE_INNER_L3_IPV6,
		RTE_PTYPE_INNER_L3_IPV6_EXT,
		RTE_PTYPE_INNER_L4_TCP,
		RTE_PTYPE_INNER_L4_UDP,
		RTE_PTYPE_UNKNOWN
	};

	if (dev->rx_pkt_burst == yusur2_recv_pkts ||
	    dev->rx_pkt_burst == yusur2_recv_pkts_lro_single_alloc ||
	    dev->rx_pkt_burst == yusur2_recv_pkts_lro_bulk_alloc ||
	    dev->rx_pkt_burst == yusur2_recv_pkts_bulk_alloc)
		return ptypes;

#if defined(RTE_ARCH_X86) || defined(RTE_MACHINE_CPUFLAG_NEON)
	if (dev->rx_pkt_burst == yusur2_recv_pkts_vec ||
	    dev->rx_pkt_burst == yusur2_recv_scattered_pkts_vec)
		return ptypes;
#endif
	return NULL;
}

static int
yusur2vf_dev_info_get(struct rte_eth_dev *dev,
		     struct rte_eth_dev_info *dev_info)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	dev_info->max_rx_queues = (uint16_t)hw->mac.max_rx_queues;
	dev_info->max_tx_queues = (uint16_t)hw->mac.max_tx_queues;
	dev_info->min_rx_bufsize = 1024; /* cf BSIZEPACKET in SRRCTL reg */
	dev_info->max_rx_pktlen = 9728; /* includes CRC, cf MAXFRS reg */
	dev_info->max_mtu = dev_info->max_rx_pktlen - YUSUR2_ETH_OVERHEAD;
	dev_info->max_mac_addrs = hw->mac.num_rar_entries;
	dev_info->max_hash_mac_addrs = YUSUR2_VMDQ_NUM_UC_MAC;
	dev_info->max_vfs = pci_dev->max_vfs;
	if (hw->mac.type == yusur2_mac_82598EB)
		dev_info->max_vmdq_pools = ETH_16_POOLS;
	else
		dev_info->max_vmdq_pools = ETH_64_POOLS;
	dev_info->rx_queue_offload_capa = yusur2_get_rx_queue_offloads(dev);
	dev_info->rx_offload_capa = (yusur2_get_rx_port_offloads(dev) |
				     dev_info->rx_queue_offload_capa);
	dev_info->tx_queue_offload_capa = yusur2_get_tx_queue_offloads(dev);
	dev_info->tx_offload_capa = yusur2_get_tx_port_offloads(dev);
	dev_info->hash_key_size = YUSUR2_HKEY_MAX_INDEX * sizeof(uint32_t);
	dev_info->reta_size = yusur2_reta_size_get(hw->mac.type);
	dev_info->flow_type_rss_offloads = YUSUR2_RSS_OFFLOAD_ALL;

	dev_info->default_rxconf = (struct rte_eth_rxconf) {
		.rx_thresh = {
			.pthresh = YUSUR2_DEFAULT_RX_PTHRESH,
			.hthresh = YUSUR2_DEFAULT_RX_HTHRESH,
			.wthresh = YUSUR2_DEFAULT_RX_WTHRESH,
		},
		.rx_free_thresh = YUSUR2_DEFAULT_RX_FREE_THRESH,
		.rx_drop_en = 0,
		.offloads = 0,
	};

	dev_info->default_txconf = (struct rte_eth_txconf) {
		.tx_thresh = {
			.pthresh = YUSUR2_DEFAULT_TX_PTHRESH,
			.hthresh = YUSUR2_DEFAULT_TX_HTHRESH,
			.wthresh = YUSUR2_DEFAULT_TX_WTHRESH,
		},
		.tx_free_thresh = YUSUR2_DEFAULT_TX_FREE_THRESH,
		.tx_rs_thresh = YUSUR2_DEFAULT_TX_RSBIT_THRESH,
		.offloads = 0,
	};

	dev_info->rx_desc_lim = rx_desc_lim;
	dev_info->tx_desc_lim = tx_desc_lim;

	return 0;
}

static int
yusur2vf_check_link(struct yusur2_hw *hw, yusur2_link_speed *speed,
		   int *link_up, int wait_to_complete)
{
	struct yusur2_adapter *adapter = container_of(hw,
						     struct yusur2_adapter, hw);
	struct yusur2_mbx_info *mbx = &hw->mbx;
	struct yusur2_mac_info *mac = &hw->mac;
	uint32_t links_reg, in_msg;
	int ret_val = 0;

	/* If we were hit with a reset drop the link */
	if (!mbx->ops.check_for_rst(hw, 0) || !mbx->timeout)
		mac->get_link_status = true;

	if (!mac->get_link_status)
		goto out;

	/* if link status is down no point in checking to see if pf is up */
	links_reg = YUSUR2_READ_REG(hw, YUSUR2_VFLINKS);
	if (!(links_reg & YUSUR2_LINKS_UP))
		goto out;

	/* for SFP+ modules and DA cables on 82599 it can take up to 500usecs
	 * before the link status is correct
	 */
	if (mac->type == yusur2_mac_82599_vf && wait_to_complete) {
		int i;

		for (i = 0; i < 5; i++) {
			rte_delay_us(100);
			links_reg = YUSUR2_READ_REG(hw, YUSUR2_VFLINKS);

			if (!(links_reg & YUSUR2_LINKS_UP))
				goto out;
		}
	}

	switch (links_reg & YUSUR2_LINKS_SPEED_82599) {
	case YUSUR2_LINKS_SPEED_10G_82599:
		*speed = YUSUR2_LINK_SPEED_10GB_FULL;
		if (hw->mac.type >= yusur2_mac_X550) {
			if (links_reg & YUSUR2_LINKS_SPEED_NON_STD)
				*speed = YUSUR2_LINK_SPEED_2_5GB_FULL;
		}
		break;
	case YUSUR2_LINKS_SPEED_1G_82599:
		*speed = YUSUR2_LINK_SPEED_1GB_FULL;
		break;
	case YUSUR2_LINKS_SPEED_100_82599:
		*speed = YUSUR2_LINK_SPEED_100_FULL;
		if (hw->mac.type == yusur2_mac_X550) {
			if (links_reg & YUSUR2_LINKS_SPEED_NON_STD)
				*speed = YUSUR2_LINK_SPEED_5GB_FULL;
		}
		break;
	case YUSUR2_LINKS_SPEED_10_X550EM_A:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
		/* Since Reserved in older MAC's */
		if (hw->mac.type >= yusur2_mac_X550)
			*speed = YUSUR2_LINK_SPEED_10_FULL;
		break;
	default:
		*speed = YUSUR2_LINK_SPEED_UNKNOWN;
	}

	if (wait_to_complete == 0 && adapter->pflink_fullchk == 0) {
		if (*speed == YUSUR2_LINK_SPEED_UNKNOWN)
			mac->get_link_status = true;
		else
			mac->get_link_status = false;

		goto out;
	}

	/* if the read failed it could just be a mailbox collision, best wait
	 * until we are called again and don't report an error
	 */
	if (mbx->ops.read(hw, &in_msg, 1, 0))
		goto out;

	if (!(in_msg & YUSUR2_VT_MSGTYPE_CTS)) {
		/* msg is not CTS and is NACK we must have lost CTS status */
		if (in_msg & YUSUR2_VT_MSGTYPE_NACK)
			mac->get_link_status = false;
		goto out;
	}

	/* the pf is talking, if we timed out in the past we reinit */
	if (!mbx->timeout) {
		ret_val = -1;
		goto out;
	}

	/* if we passed all the tests above then the link is up and we no
	 * longer need to check for link
	 */
	mac->get_link_status = false;

out:
	*link_up = !mac->get_link_status;
	return ret_val;
}

/*
 * If @timeout_ms was 0, it means that it will not return until link complete.
 * It returns 1 on complete, return 0 on timeout.
 */
static int
yusur2_dev_wait_setup_link_complete(struct rte_eth_dev *dev, uint32_t timeout_ms)
{
#define WARNING_TIMEOUT    9000 /* 9s  in total */
	struct yusur2_adapter *ad = dev->data->dev_private;
	uint32_t timeout = timeout_ms ? timeout_ms : WARNING_TIMEOUT;

	while (rte_atomic32_read(&ad->link_thread_running)) {
		msec_delay(1);
		timeout--;

		if (timeout_ms) {
			if (!timeout)
				return 0;
		} else if (!timeout) {
			/* It will not return until link complete */
			timeout = WARNING_TIMEOUT;
			PMD_DRV_LOG(ERR, "YUSUR2 link thread not complete too long time!");
		}
	}

	return 1;
}

static void *
yusur2_dev_setup_link_thread_handler(void *param)
{
	struct rte_eth_dev *dev = (struct rte_eth_dev *)param;
	struct yusur2_adapter *ad = dev->data->dev_private;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	u32 speed;
	bool autoneg = false;

	pthread_detach(pthread_self());
	speed = hw->phy.autoneg_advertised;
	if (!speed)
		yusur2_get_link_capabilities(hw, &speed, &autoneg);

	yusur2_setup_link(hw, speed, true);

	intr->flags &= ~YUSUR2_FLAG_NEED_LINK_CONFIG;
	rte_atomic32_clear(&ad->link_thread_running);
	return NULL;
}

/*
 * In freebsd environment, nic_uio drivers do not support interrupts,
 * rte_intr_callback_register() will fail to register interrupts.
 * We can not make link status to change from down to up by interrupt
 * callback. So we need to wait for the controller to acquire link
 * when ports start.
 * It returns 0 on link up.
 */
static int
yusur2_wait_for_link_up(struct yusur2_hw *hw)
{
#ifdef RTE_EXEC_ENV_FREEBSD
	int err, i, link_up = 0;
	uint32_t speed = 0;
	const int nb_iter = 25;

	for (i = 0; i < nb_iter; i++) {
		err = yusur2_check_link(hw, &speed, &link_up, 0);
		if (err)
			return err;
		if (link_up)
			return 0;
		msec_delay(200);
	}

	return 0;
#else
	RTE_SET_USED(hw);
	return 0;
#endif
}

/* return 0 means link status changed, -1 means not changed */
int
yusur2_dev_link_update_share(struct rte_eth_dev *dev,
			    int wait_to_complete, int vf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_adapter *ad = dev->data->dev_private;
	struct rte_eth_link link;
	yusur2_link_speed link_speed = YUSUR2_LINK_SPEED_UNKNOWN;
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	int link_up;
	int diag;
	int wait = 1;
	u32 esdp_reg;

	memset(&link, 0, sizeof(link));
	link.link_status = ETH_LINK_DOWN;
	link.link_speed = ETH_SPEED_NUM_NONE;
	link.link_duplex = ETH_LINK_HALF_DUPLEX;
	link.link_autoneg = !(dev->data->dev_conf.link_speeds &
			ETH_LINK_SPEED_FIXED);

	hw->mac.get_link_status = true;

	if (intr->flags & YUSUR2_FLAG_NEED_LINK_CONFIG)
		return rte_eth_linkstatus_set(dev, &link);

	/* check if it needs to wait to complete, if lsc interrupt is enabled */
	if (wait_to_complete == 0 || dev->data->dev_conf.intr_conf.lsc != 0)
		wait = 0;

/* BSD has no interrupt mechanism, so force NIC status synchronization. */
#ifdef RTE_EXEC_ENV_FREEBSD
	wait = 1;
#endif

	if (vf)
		diag = yusur2vf_check_link(hw, &link_speed, &link_up, wait);
	else
		diag = yusur2_check_link(hw, &link_speed, &link_up, wait);

	if (diag != 0) {
		link.link_speed = ETH_SPEED_NUM_100M;
		link.link_duplex = ETH_LINK_FULL_DUPLEX;
		return rte_eth_linkstatus_set(dev, &link);
	}

	if (yusur2_get_media_type(hw) == yusur2_media_type_fiber) {
		esdp_reg = YUSUR2_READ_REG(hw, YUSUR2_ESDP);
		if ((esdp_reg & YUSUR2_ESDP_SDP3))
			link_up = 0;
	}

	if (link_up == 0) {
		if (yusur2_get_media_type(hw) == yusur2_media_type_fiber) {
			yusur2_dev_wait_setup_link_complete(dev, 0);
			if (rte_atomic32_test_and_set(&ad->link_thread_running)) {
				/* To avoid race condition between threads, set
				 * the YUSUR2_FLAG_NEED_LINK_CONFIG flag only
				 * when there is no link thread running.
				 */
				intr->flags |= YUSUR2_FLAG_NEED_LINK_CONFIG;
				if (rte_ctrl_thread_create(&ad->link_thread_tid,
					"yusur2-link-handler",
					NULL,
					yusur2_dev_setup_link_thread_handler,
					dev) < 0) {
					PMD_DRV_LOG(ERR,
						"Create link thread failed!");
					rte_atomic32_clear(&ad->link_thread_running);
				}
			} else {
				PMD_DRV_LOG(ERR,
					"Other link thread is running now!");
			}
		}
		return rte_eth_linkstatus_set(dev, &link);
	}

	link.link_status = ETH_LINK_UP;
	link.link_duplex = ETH_LINK_FULL_DUPLEX;

	switch (link_speed) {
	default:
	case YUSUR2_LINK_SPEED_UNKNOWN:
		if (hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T ||
			hw->device_id == YUSUR2_DEV_ID_X550EM_A_1G_T_L)
			link.link_speed = ETH_SPEED_NUM_10M;
		else
			link.link_speed = ETH_SPEED_NUM_100M;
		break;

	case YUSUR2_LINK_SPEED_10_FULL:
		link.link_speed = ETH_SPEED_NUM_10M;
		break;

	case YUSUR2_LINK_SPEED_100_FULL:
		link.link_speed = ETH_SPEED_NUM_100M;
		break;

	case YUSUR2_LINK_SPEED_1GB_FULL:
		link.link_speed = ETH_SPEED_NUM_1G;
		break;

	case YUSUR2_LINK_SPEED_2_5GB_FULL:
		link.link_speed = ETH_SPEED_NUM_2_5G;
		break;

	case YUSUR2_LINK_SPEED_5GB_FULL:
		link.link_speed = ETH_SPEED_NUM_5G;
		break;

	case YUSUR2_LINK_SPEED_10GB_FULL:
		link.link_speed = ETH_SPEED_NUM_10G;
		break;
	}

	return rte_eth_linkstatus_set(dev, &link);
}

static int
yusur2_dev_link_update(struct rte_eth_dev *dev, int wait_to_complete)
{
	return yusur2_dev_link_update_share(dev, wait_to_complete, 0);
}

static int
yusur2vf_dev_link_update(struct rte_eth_dev *dev, int wait_to_complete)
{
	return yusur2_dev_link_update_share(dev, wait_to_complete, 1);
}

static int
yusur2_dev_promiscuous_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl |= (YUSUR2_FCTRL_UPE | YUSUR2_FCTRL_MPE);
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	return 0;
}

static int
yusur2_dev_promiscuous_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl &= (~YUSUR2_FCTRL_UPE);
	if (dev->data->all_multicast == 1)
		fctrl |= YUSUR2_FCTRL_MPE;
	else
		fctrl &= (~YUSUR2_FCTRL_MPE);
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	return 0;
}

static int
yusur2_dev_allmulticast_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl |= YUSUR2_FCTRL_MPE;
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	return 0;
}

static int
yusur2_dev_allmulticast_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	if (dev->data->promiscuous == 1)
		return 0; /* must remain in all_multicast mode */

	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
	fctrl &= (~YUSUR2_FCTRL_MPE);
	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	return 0;
}

/**
 * It clears the interrupt causes and enables the interrupt.
 * It will be called once only during nic initialized.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 * @param on
 *  Enable or Disable.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
yusur2_dev_lsc_interrupt_setup(struct rte_eth_dev *dev, uint8_t on)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	yusur2_dev_link_status_print(dev);
	if (on)
		intr->mask |= YUSUR2_EICR_LSC;
	else
		intr->mask &= ~YUSUR2_EICR_LSC;

	return 0;
}

/**
 * It clears the interrupt causes and enables the interrupt.
 * It will be called once only during nic initialized.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
yusur2_dev_rxq_interrupt_setup(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	intr->mask |= YUSUR2_EICR_RTX_QUEUE;

	return 0;
}

/**
 * It clears the interrupt causes and enables the interrupt.
 * It will be called once only during nic initialized.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
yusur2_dev_macsec_interrupt_setup(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	intr->mask |= YUSUR2_EICR_LINKSEC;

	return 0;
}

/*
 * It reads ICR and sets flag (YUSUR2_EICR_LSC) for the link_update.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
yusur2_dev_interrupt_get_status(struct rte_eth_dev *dev)
{
	uint32_t eicr;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	/* clear all cause mask */
	yusur2_disable_intr(hw);

	/* read-on-clear nic registers here */
	eicr = YUSUR2_READ_REG(hw, YUSUR2_EICR);
	PMD_DRV_LOG(DEBUG, "eicr %x", eicr);

	intr->flags = 0;

	/* set flag for async link update */
	if (eicr & YUSUR2_EICR_LSC)
		intr->flags |= YUSUR2_FLAG_NEED_LINK_UPDATE;

	if (eicr & YUSUR2_EICR_MAILBOX)
		intr->flags |= YUSUR2_FLAG_MAILBOX;

	if (eicr & YUSUR2_EICR_LINKSEC)
		intr->flags |= YUSUR2_FLAG_MACSEC;

	if (hw->mac.type ==  yusur2_mac_X550EM_x &&
	    hw->phy.type == yusur2_phy_x550em_ext_t &&
	    (eicr & YUSUR2_EICR_GPI_SDP0_X550EM_x))
		intr->flags |= YUSUR2_FLAG_PHY_INTERRUPT;

	return 0;
}

/**
 * It gets and then prints the link status.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static void
yusur2_dev_link_status_print(struct rte_eth_dev *dev)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_eth_link link;

	rte_eth_linkstatus_get(dev, &link);

	if (link.link_status) {
		PMD_INIT_LOG(INFO, "Port %d: Link Up - speed %u Mbps - %s",
					(int)(dev->data->port_id),
					(unsigned)link.link_speed,
			link.link_duplex == ETH_LINK_FULL_DUPLEX ?
					"full-duplex" : "half-duplex");
	} else {
		PMD_INIT_LOG(INFO, " Port %d: Link Down",
				(int)(dev->data->port_id));
	}
	PMD_INIT_LOG(DEBUG, "PCI Address: " PCI_PRI_FMT,
				pci_dev->addr.domain,
				pci_dev->addr.bus,
				pci_dev->addr.devid,
				pci_dev->addr.function);
}

/*
 * It executes link_update after knowing an interrupt occurred.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
yusur2_dev_interrupt_action(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	int64_t timeout;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	PMD_DRV_LOG(DEBUG, "intr action type %d", intr->flags);

	if (intr->flags & YUSUR2_FLAG_MAILBOX) {
		yusur2_pf_mbx_process(dev);
		intr->flags &= ~YUSUR2_FLAG_MAILBOX;
	}

	if (intr->flags & YUSUR2_FLAG_PHY_INTERRUPT) {
		yusur2_handle_lasi(hw);
		intr->flags &= ~YUSUR2_FLAG_PHY_INTERRUPT;
	}

	if (intr->flags & YUSUR2_FLAG_NEED_LINK_UPDATE) {
		struct rte_eth_link link;

		/* get the link status before link update, for predicting later */
		rte_eth_linkstatus_get(dev, &link);

		yusur2_dev_link_update(dev, 0);

		/* likely to up */
		if (!link.link_status)
			/* handle it 1 sec later, wait it being stable */
			timeout = YUSUR2_LINK_UP_CHECK_TIMEOUT;
		/* likely to down */
		else
			/* handle it 4 sec later, wait it being stable */
			timeout = YUSUR2_LINK_DOWN_CHECK_TIMEOUT;

		yusur2_dev_link_status_print(dev);
		if (rte_eal_alarm_set(timeout * 1000,
				      yusur2_dev_interrupt_delayed_handler, (void *)dev) < 0)
			PMD_DRV_LOG(ERR, "Error setting alarm");
		else {
			/* remember original mask */
			intr->mask_original = intr->mask;
			/* only disable lsc interrupt */
			intr->mask &= ~YUSUR2_EIMS_LSC;
		}
	}

	PMD_DRV_LOG(DEBUG, "enable intr immediately");
	yusur2_enable_intr(dev);

	return 0;
}

/**
 * Interrupt handler which shall be registered for alarm callback for delayed
 * handling specific interrupt to wait for the stable nic state. As the
 * NIC interrupt state is not stable for yusur2 after link is just down,
 * it needs to wait 4 seconds to get the stable status.
 *
 * @param handle
 *  Pointer to interrupt handle.
 * @param param
 *  The address of parameter (struct rte_eth_dev *) regsitered before.
 *
 * @return
 *  void
 */
static void
yusur2_dev_interrupt_delayed_handler(void *param)
{
	struct rte_eth_dev *dev = (struct rte_eth_dev *)param;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t eicr;

	yusur2_disable_intr(hw);

	eicr = YUSUR2_READ_REG(hw, YUSUR2_EICR);
	if (eicr & YUSUR2_EICR_MAILBOX)
		yusur2_pf_mbx_process(dev);

	if (intr->flags & YUSUR2_FLAG_PHY_INTERRUPT) {
		yusur2_handle_lasi(hw);
		intr->flags &= ~YUSUR2_FLAG_PHY_INTERRUPT;
	}

	if (intr->flags & YUSUR2_FLAG_NEED_LINK_UPDATE) {
		yusur2_dev_link_update(dev, 0);
		intr->flags &= ~YUSUR2_FLAG_NEED_LINK_UPDATE;
		yusur2_dev_link_status_print(dev);
		_rte_eth_dev_callback_process(dev, RTE_ETH_EVENT_INTR_LSC,
					      NULL);
	}

	if (intr->flags & YUSUR2_FLAG_MACSEC) {
		_rte_eth_dev_callback_process(dev, RTE_ETH_EVENT_MACSEC,
					      NULL);
		intr->flags &= ~YUSUR2_FLAG_MACSEC;
	}

	/* restore original mask */
	intr->mask = intr->mask_original;
	intr->mask_original = 0;

	PMD_DRV_LOG(DEBUG, "enable intr in delayed handler S[%08x]", eicr);
	yusur2_enable_intr(dev);
	rte_intr_ack(intr_handle);
}

/**
 * Interrupt handler triggered by NIC  for handling
 * specific interrupt.
 *
 * @param handle
 *  Pointer to interrupt handle.
 * @param param
 *  The address of parameter (struct rte_eth_dev *) regsitered before.
 *
 * @return
 *  void
 */
static void
yusur2_dev_interrupt_handler(void *param)
{
	struct rte_eth_dev *dev = (struct rte_eth_dev *)param;

	yusur2_dev_interrupt_get_status(dev);
	yusur2_dev_interrupt_action(dev);
}

static int
yusur2_dev_led_on(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	return yusur2_led_on(hw, 0) == YUSUR2_SUCCESS ? 0 : -ENOTSUP;
}

static int
yusur2_dev_led_off(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	return yusur2_led_off(hw, 0) == YUSUR2_SUCCESS ? 0 : -ENOTSUP;
}

static int
yusur2_flow_ctrl_get(struct rte_eth_dev *dev, struct rte_eth_fc_conf *fc_conf)
{
	struct yusur2_hw *hw;
	uint32_t mflcn_reg;
	uint32_t fccfg_reg;
	int rx_pause;
	int tx_pause;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	fc_conf->pause_time = hw->fc.pause_time;
	fc_conf->high_water = hw->fc.high_water[0];
	fc_conf->low_water = hw->fc.low_water[0];
	fc_conf->send_xon = hw->fc.send_xon;
	fc_conf->autoneg = !hw->fc.disable_fc_autoneg;

	/*
	 * Return rx_pause status according to actual setting of
	 * MFLCN register.
	 */
	mflcn_reg = YUSUR2_READ_REG(hw, YUSUR2_MFLCN);
	if (mflcn_reg & YUSUR2_MFLCN_PMCF)
		fc_conf->mac_ctrl_frame_fwd = 1;
	else
		fc_conf->mac_ctrl_frame_fwd = 0;

	if (mflcn_reg & (YUSUR2_MFLCN_RPFCE | YUSUR2_MFLCN_RFCE))
		rx_pause = 1;
	else
		rx_pause = 0;

	/*
	 * Return tx_pause status according to actual setting of
	 * FCCFG register.
	 */
	fccfg_reg = YUSUR2_READ_REG(hw, YUSUR2_FCCFG);
	if (fccfg_reg & (YUSUR2_FCCFG_TFCE_802_3X | YUSUR2_FCCFG_TFCE_PRIORITY))
		tx_pause = 1;
	else
		tx_pause = 0;

	if (rx_pause && tx_pause)
		fc_conf->mode = RTE_FC_FULL;
	else if (rx_pause)
		fc_conf->mode = RTE_FC_RX_PAUSE;
	else if (tx_pause)
		fc_conf->mode = RTE_FC_TX_PAUSE;
	else
		fc_conf->mode = RTE_FC_NONE;

	return 0;
}

static int
yusur2_flow_ctrl_set(struct rte_eth_dev *dev, struct rte_eth_fc_conf *fc_conf)
{
	struct yusur2_hw *hw;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	int err;
	uint32_t rx_buf_size;
	uint32_t max_high_water;
	enum yusur2_fc_mode rte_fcmode_2_yusur2_fcmode[] = {
		yusur2_fc_none,
		yusur2_fc_rx_pause,
		yusur2_fc_tx_pause,
		yusur2_fc_full
	};

	PMD_INIT_FUNC_TRACE();

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	rx_buf_size = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(0));
	PMD_INIT_LOG(DEBUG, "Rx packet buffer size = 0x%x", rx_buf_size);

	/*
	 * At least reserve one Ethernet frame for watermark
	 * high_water/low_water in kilo bytes for yusur2
	 */
	max_high_water = (rx_buf_size -
			RTE_ETHER_MAX_LEN) >> YUSUR2_RXPBSIZE_SHIFT;
	if ((fc_conf->high_water > max_high_water) ||
		(fc_conf->high_water < fc_conf->low_water)) {
		PMD_INIT_LOG(ERR, "Invalid high/low water setup value in KB");
		PMD_INIT_LOG(ERR, "High_water must <= 0x%x", max_high_water);
		return -EINVAL;
	}

	hw->fc.requested_mode = rte_fcmode_2_yusur2_fcmode[fc_conf->mode];
	hw->fc.pause_time     = fc_conf->pause_time;
	hw->fc.high_water[0]  = fc_conf->high_water;
	hw->fc.low_water[0]   = fc_conf->low_water;
	hw->fc.send_xon       = fc_conf->send_xon;
	hw->fc.disable_fc_autoneg = !fc_conf->autoneg;
	adapter->mac_ctrl_frame_fwd = fc_conf->mac_ctrl_frame_fwd;

	err = yusur2_flow_ctrl_enable(dev, hw);
	if (err < 0) {
		PMD_INIT_LOG(ERR, "yusur2_flow_ctrl_enable = 0x%x", err);
		return -EIO;
	}
	return err;
}

/**
 *  yusur2_pfc_enable_generic - Enable flow control
 *  @hw: pointer to hardware structure
 *  @tc_num: traffic class number
 *  Enable flow control according to the current settings.
 */
static int
yusur2_dcb_pfc_enable_generic(struct yusur2_hw *hw, uint8_t tc_num)
{
	int ret_val = 0;
	uint32_t mflcn_reg, fccfg_reg;
	uint32_t reg;
	uint32_t fcrtl, fcrth;
	uint8_t i;
	uint8_t nb_rx_en;

	/* Validate the water mark configuration */
	if (!hw->fc.pause_time) {
		ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	/* Low water mark of zero causes XOFF floods */
	if (hw->fc.current_mode & yusur2_fc_tx_pause) {
		 /* High/Low water can not be 0 */
		if ((!hw->fc.high_water[tc_num]) || (!hw->fc.low_water[tc_num])) {
			PMD_INIT_LOG(ERR, "Invalid water mark configuration");
			ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
			goto out;
		}

		if (hw->fc.low_water[tc_num] >= hw->fc.high_water[tc_num]) {
			PMD_INIT_LOG(ERR, "Invalid water mark configuration");
			ret_val = YUSUR2_ERR_INVALID_LINK_SETTINGS;
			goto out;
		}
	}
	/* Negotiate the fc mode to use */
	yusur2_fc_autoneg(hw);

	/* Disable any previous flow control settings */
	mflcn_reg = YUSUR2_READ_REG(hw, YUSUR2_MFLCN);
	mflcn_reg &= ~(YUSUR2_MFLCN_RPFCE_SHIFT | YUSUR2_MFLCN_RFCE|YUSUR2_MFLCN_RPFCE);

	fccfg_reg = YUSUR2_READ_REG(hw, YUSUR2_FCCFG);
	fccfg_reg &= ~(YUSUR2_FCCFG_TFCE_802_3X | YUSUR2_FCCFG_TFCE_PRIORITY);

	switch (hw->fc.current_mode) {
	case yusur2_fc_none:
		/*
		 * If the count of enabled RX Priority Flow control >1,
		 * and the TX pause can not be disabled
		 */
		nb_rx_en = 0;
		for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
			reg = YUSUR2_READ_REG(hw, YUSUR2_FCRTH_82599(i));
			if (reg & YUSUR2_FCRTH_FCEN)
				nb_rx_en++;
		}
		if (nb_rx_en > 1)
			fccfg_reg |= YUSUR2_FCCFG_TFCE_PRIORITY;
		break;
	case yusur2_fc_rx_pause:
		/*
		 * Rx Flow control is enabled and Tx Flow control is
		 * disabled by software override. Since there really
		 * isn't a way to advertise that we are capable of RX
		 * Pause ONLY, we will advertise that we support both
		 * symmetric and asymmetric Rx PAUSE.  Later, we will
		 * disable the adapter's ability to send PAUSE frames.
		 */
		mflcn_reg |= YUSUR2_MFLCN_RPFCE;
		/*
		 * If the count of enabled RX Priority Flow control >1,
		 * and the TX pause can not be disabled
		 */
		nb_rx_en = 0;
		for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
			reg = YUSUR2_READ_REG(hw, YUSUR2_FCRTH_82599(i));
			if (reg & YUSUR2_FCRTH_FCEN)
				nb_rx_en++;
		}
		if (nb_rx_en > 1)
			fccfg_reg |= YUSUR2_FCCFG_TFCE_PRIORITY;
		break;
	case yusur2_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		fccfg_reg |= YUSUR2_FCCFG_TFCE_PRIORITY;
		break;
	case yusur2_fc_full:
		/* Flow control (both Rx and Tx) is enabled by SW override. */
		mflcn_reg |= YUSUR2_MFLCN_RPFCE;
		fccfg_reg |= YUSUR2_FCCFG_TFCE_PRIORITY;
		break;
	default:
		PMD_DRV_LOG(DEBUG, "Flow control param set incorrectly");
		ret_val = YUSUR2_ERR_CONFIG;
		goto out;
	}

	/* Set 802.3x based flow control settings. */
	mflcn_reg |= YUSUR2_MFLCN_DPF;
	YUSUR2_WRITE_REG(hw, YUSUR2_MFLCN, mflcn_reg);
	YUSUR2_WRITE_REG(hw, YUSUR2_FCCFG, fccfg_reg);

	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	if ((hw->fc.current_mode & yusur2_fc_tx_pause) &&
		hw->fc.high_water[tc_num]) {
		fcrtl = (hw->fc.low_water[tc_num] << 10) | YUSUR2_FCRTL_XONE;
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(tc_num), fcrtl);
		fcrth = (hw->fc.high_water[tc_num] << 10) | YUSUR2_FCRTH_FCEN;
	} else {
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(tc_num), 0);
		/*
		 * In order to prevent Tx hangs when the internal Tx
		 * switch is enabled we must set the high water mark
		 * to the maximum FCRTH value.  This allows the Tx
		 * switch to function even under heavy Rx workloads.
		 */
		fcrth = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(tc_num)) - 32;
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH_82599(tc_num), fcrth);

	/* Configure pause time (2 TCs per register) */
	reg = hw->fc.pause_time * 0x00010001;
	for (i = 0; i < (YUSUR2_DCB_MAX_TRAFFIC_CLASS / 2); i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_FCTTV(i), reg);

	/* Configure flow control refresh threshold value */
	YUSUR2_WRITE_REG(hw, YUSUR2_FCRTV, hw->fc.pause_time / 2);

out:
	return ret_val;
}

static int
yusur2_dcb_pfc_enable(struct rte_eth_dev *dev, uint8_t tc_num)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int32_t ret_val = YUSUR2_NOT_IMPLEMENTED;

	if (hw->mac.type != yusur2_mac_82598EB) {
		ret_val = yusur2_dcb_pfc_enable_generic(hw, tc_num);
	}
	return ret_val;
}

static int
yusur2_priority_flow_ctrl_set(struct rte_eth_dev *dev, struct rte_eth_pfc_conf *pfc_conf)
{
	int err;
	uint32_t rx_buf_size;
	uint32_t max_high_water;
	uint8_t tc_num;
	uint8_t  map[YUSUR2_DCB_MAX_USER_PRIORITY] = { 0 };
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_dcb_config *dcb_config =
		YUSUR2_DEV_PRIVATE_TO_DCB_CFG(dev->data->dev_private);

	enum yusur2_fc_mode rte_fcmode_2_yusur2_fcmode[] = {
		yusur2_fc_none,
		yusur2_fc_rx_pause,
		yusur2_fc_tx_pause,
		yusur2_fc_full
	};

	PMD_INIT_FUNC_TRACE();

	yusur2_dcb_unpack_map_cee(dcb_config, YUSUR2_DCB_RX_CONFIG, map);
	tc_num = map[pfc_conf->priority];
	rx_buf_size = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(tc_num));
	PMD_INIT_LOG(DEBUG, "Rx packet buffer size = 0x%x", rx_buf_size);
	/*
	 * At least reserve one Ethernet frame for watermark
	 * high_water/low_water in kilo bytes for yusur2
	 */
	max_high_water = (rx_buf_size -
			RTE_ETHER_MAX_LEN) >> YUSUR2_RXPBSIZE_SHIFT;
	if ((pfc_conf->fc.high_water > max_high_water) ||
	    (pfc_conf->fc.high_water <= pfc_conf->fc.low_water)) {
		PMD_INIT_LOG(ERR, "Invalid high/low water setup value in KB");
		PMD_INIT_LOG(ERR, "High_water must <= 0x%x", max_high_water);
		return -EINVAL;
	}

	hw->fc.requested_mode = rte_fcmode_2_yusur2_fcmode[pfc_conf->fc.mode];
	hw->fc.pause_time = pfc_conf->fc.pause_time;
	hw->fc.send_xon = pfc_conf->fc.send_xon;
	hw->fc.low_water[tc_num] =  pfc_conf->fc.low_water;
	hw->fc.high_water[tc_num] = pfc_conf->fc.high_water;

	err = yusur2_dcb_pfc_enable(dev, tc_num);

	/* Not negotiated is not an error case */
	if ((err == YUSUR2_SUCCESS) || (err == YUSUR2_ERR_FC_NOT_NEGOTIATED))
		return 0;

	PMD_INIT_LOG(ERR, "yusur2_dcb_pfc_enable = 0x%x", err);
	return -EIO;
}

static int
yusur2_dev_rss_reta_update(struct rte_eth_dev *dev,
			  struct rte_eth_rss_reta_entry64 *reta_conf,
			  uint16_t reta_size)
{
	uint16_t i, sp_reta_size;
	uint8_t j, mask;
	uint32_t reta, r;
	uint16_t idx, shift;
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t reta_reg;

	PMD_INIT_FUNC_TRACE();

	if (!yusur2_rss_update_sp(hw->mac.type)) {
		PMD_DRV_LOG(ERR, "RSS reta update is not supported on this "
			"NIC.");
		return -ENOTSUP;
	}

	sp_reta_size = yusur2_reta_size_get(hw->mac.type);
	if (reta_size != sp_reta_size) {
		PMD_DRV_LOG(ERR, "The size of hash lookup table configured "
			"(%d) doesn't match the number hardware can supported "
			"(%d)", reta_size, sp_reta_size);
		return -EINVAL;
	}

	for (i = 0; i < reta_size; i += YUSUR2_4_BIT_WIDTH) {
		idx = i / RTE_RETA_GROUP_SIZE;
		shift = i % RTE_RETA_GROUP_SIZE;
		mask = (uint8_t)((reta_conf[idx].mask >> shift) &
						YUSUR2_4_BIT_MASK);
		if (!mask)
			continue;
		reta_reg = yusur2_reta_reg_get(hw->mac.type, i);
		if (mask == YUSUR2_4_BIT_MASK)
			r = 0;
		else
			r = YUSUR2_READ_REG(hw, reta_reg);
		for (j = 0, reta = 0; j < YUSUR2_4_BIT_WIDTH; j++) {
			if (mask & (0x1 << j))
				reta |= reta_conf[idx].reta[shift + j] <<
							(CHAR_BIT * j);
			else
				reta |= r & (YUSUR2_8_BIT_MASK <<
						(CHAR_BIT * j));
		}
		YUSUR2_WRITE_REG(hw, reta_reg, reta);
	}
	adapter->rss_reta_updated = 1;

	return 0;
}

static int
yusur2_dev_rss_reta_query(struct rte_eth_dev *dev,
			 struct rte_eth_rss_reta_entry64 *reta_conf,
			 uint16_t reta_size)
{
	uint16_t i, sp_reta_size;
	uint8_t j, mask;
	uint32_t reta;
	uint16_t idx, shift;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t reta_reg;

	PMD_INIT_FUNC_TRACE();
	sp_reta_size = yusur2_reta_size_get(hw->mac.type);
	if (reta_size != sp_reta_size) {
		PMD_DRV_LOG(ERR, "The size of hash lookup table configured "
			"(%d) doesn't match the number hardware can supported "
			"(%d)", reta_size, sp_reta_size);
		return -EINVAL;
	}

	for (i = 0; i < reta_size; i += YUSUR2_4_BIT_WIDTH) {
		idx = i / RTE_RETA_GROUP_SIZE;
		shift = i % RTE_RETA_GROUP_SIZE;
		mask = (uint8_t)((reta_conf[idx].mask >> shift) &
						YUSUR2_4_BIT_MASK);
		if (!mask)
			continue;

		reta_reg = yusur2_reta_reg_get(hw->mac.type, i);
		reta = YUSUR2_READ_REG(hw, reta_reg);
		for (j = 0; j < YUSUR2_4_BIT_WIDTH; j++) {
			if (mask & (0x1 << j))
				reta_conf[idx].reta[shift + j] =
					((reta >> (CHAR_BIT * j)) &
						YUSUR2_8_BIT_MASK);
		}
	}

	return 0;
}

static int
yusur2_add_rar(struct rte_eth_dev *dev, struct rte_ether_addr *mac_addr,
				uint32_t index, uint32_t pool)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t enable_addr = 1;

	return yusur2_set_rar(hw, index, mac_addr->addr_bytes,
			     pool, enable_addr);
}

static void
yusur2_remove_rar(struct rte_eth_dev *dev, uint32_t index)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	yusur2_clear_rar(hw, index);
}

static int
yusur2_set_default_mac_addr(struct rte_eth_dev *dev, struct rte_ether_addr *addr)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	yusur2_remove_rar(dev, 0);
	yusur2_add_rar(dev, addr, 0, pci_dev->max_vfs);

	return 0;
}

static bool
is_device_supported(struct rte_eth_dev *dev, struct rte_pci_driver *drv)
{
	if (strcmp(dev->device->driver->name, drv->driver.name))
		return false;

	return true;
}

bool
is_yusur2_supported(struct rte_eth_dev *dev)
{
	return is_device_supported(dev, &rte_yusur2_pmd);
}

static int
yusur2_dev_mtu_set(struct rte_eth_dev *dev, uint16_t mtu)
{
	uint32_t hlreg0;
	uint32_t maxfrs;
	struct yusur2_hw *hw;
	struct rte_eth_dev_info dev_info;
	uint32_t frame_size = mtu + YUSUR2_ETH_OVERHEAD;
	struct rte_eth_dev_data *dev_data = dev->data;
	int ret;

	ret = yusur2_dev_info_get(dev, &dev_info);
	if (ret != 0)
		return ret;

	/* check that mtu is within the allowed range */
	if (mtu < RTE_ETHER_MIN_MTU || frame_size > dev_info.max_rx_pktlen)
		return -EINVAL;

	/* If device is started, refuse mtu that requires the support of
	 * scattered packets when this feature has not been enabled before.
	 */
	if (dev_data->dev_started && !dev_data->scattered_rx &&
	    (frame_size + 2 * YUSUR2_VLAN_TAG_SIZE >
	     dev->data->min_rx_buf_size - RTE_PKTMBUF_HEADROOM)) {
		PMD_INIT_LOG(ERR, "Stop port first.");
		return -EINVAL;
	}

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	hlreg0 = YUSUR2_READ_REG(hw, YUSUR2_HLREG0);

	/* switch to jumbo mode if needed */
	if (frame_size > RTE_ETHER_MAX_LEN) {
		dev->data->dev_conf.rxmode.offloads |=
			DEV_RX_OFFLOAD_JUMBO_FRAME;
		hlreg0 |= YUSUR2_HLREG0_JUMBOEN;
	} else {
		dev->data->dev_conf.rxmode.offloads &=
			~DEV_RX_OFFLOAD_JUMBO_FRAME;
		hlreg0 &= ~YUSUR2_HLREG0_JUMBOEN;
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, hlreg0);

	/* update max frame size */
	dev->data->dev_conf.rxmode.max_rx_pkt_len = frame_size;

	maxfrs = YUSUR2_READ_REG(hw, YUSUR2_MAXFRS);
	maxfrs &= 0x0000FFFF;
	maxfrs |= (dev->data->dev_conf.rxmode.max_rx_pkt_len << 16);
	YUSUR2_WRITE_REG(hw, YUSUR2_MAXFRS, maxfrs);

	return 0;
}

/*
 * Virtual Function operations
 */
static void
yusur2vf_intr_disable(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();

	/* Clear interrupt mask to stop from interrupts being generated */
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIMC, YUSUR2_VF_IRQ_CLEAR_MASK);

	YUSUR2_WRITE_FLUSH(hw);

	/* Clear mask value. */
	intr->mask = 0;
}

static void
yusur2vf_intr_enable(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();

	/* VF enable interrupt autoclean */
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIAM, YUSUR2_VF_IRQ_ENABLE_MASK);
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIAC, YUSUR2_VF_IRQ_ENABLE_MASK);
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIMS, YUSUR2_VF_IRQ_ENABLE_MASK);

	YUSUR2_WRITE_FLUSH(hw);

	/* Save YUSUR2_VTEIMS value to mask. */
	intr->mask = YUSUR2_VF_IRQ_ENABLE_MASK;
}

static int
yusur2vf_dev_configure(struct rte_eth_dev *dev)
{
	struct rte_eth_conf *conf = &dev->data->dev_conf;
	struct yusur2_adapter *adapter = dev->data->dev_private;

	PMD_INIT_LOG(DEBUG, "Configured Virtual Function port id: %d",
		     dev->data->port_id);

	if (dev->data->dev_conf.rxmode.mq_mode & ETH_MQ_RX_RSS_FLAG)
		dev->data->dev_conf.rxmode.offloads |= DEV_RX_OFFLOAD_RSS_HASH;

	/*
	 * VF has no ability to enable/disable HW CRC
	 * Keep the persistent behavior the same as Host PF
	 */
#ifndef RTE_LIBRTE_YUSUR2_PF_DISABLE_STRIP_CRC
	if (conf->rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC) {
		PMD_INIT_LOG(NOTICE, "VF can't disable HW CRC Strip");
		conf->rxmode.offloads &= ~DEV_RX_OFFLOAD_KEEP_CRC;
	}
#else
	if (!(conf->rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC)) {
		PMD_INIT_LOG(NOTICE, "VF can't enable HW CRC Strip");
		conf->rxmode.offloads |= DEV_RX_OFFLOAD_KEEP_CRC;
	}
#endif

	/*
	 * Initialize to TRUE. If any of Rx queues doesn't meet the bulk
	 * allocation or vector Rx preconditions we will reset it.
	 */
	adapter->rx_bulk_alloc_allowed = true;
	adapter->rx_vec_allowed = true;

	return 0;
}

static int
yusur2vf_dev_start(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t intr_vector = 0;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;

	int err, mask = 0;

	PMD_INIT_FUNC_TRACE();

	/* Stop the link setup handler before resetting the HW. */
	yusur2_dev_wait_setup_link_complete(dev, 0);

	err = hw->mac.ops.reset_hw(hw);

	/**
	 * In this case, reuses the MAC address assigned by VF
	 * initialization.
	 */
	if (err != YUSUR2_SUCCESS && err != YUSUR2_ERR_INVALID_MAC_ADDR) {
		PMD_INIT_LOG(ERR, "Unable to reset vf hardware (%d)", err);
		return err;
	}

	hw->mac.get_link_status = true;

	/* negotiate mailbox API version to use with the PF. */
	yusur2vf_negotiate_api(hw);

	yusur2vf_dev_tx_init(dev);

	/* This can fail when allocating mbufs for descriptor rings */
	err = yusur2vf_dev_rx_init(dev);
	if (err) {
		PMD_INIT_LOG(ERR, "Unable to initialize RX hardware (%d)", err);
		yusur2_dev_clear_queues(dev);
		return err;
	}

	/* Set vfta */
	yusur2vf_set_vfta_all(dev, 1);

	/* Set HW strip */
	mask = ETH_VLAN_STRIP_MASK | ETH_VLAN_FILTER_MASK |
		ETH_VLAN_EXTEND_MASK;
	err = yusur2vf_vlan_offload_config(dev, mask);
	if (err) {
		PMD_INIT_LOG(ERR, "Unable to set VLAN offload (%d)", err);
		yusur2_dev_clear_queues(dev);
		return err;
	}

	yusur2vf_dev_rxtx_start(dev);

	/* check and configure queue intr-vector mapping */
	if (rte_intr_cap_multiple(intr_handle) &&
	    dev->data->dev_conf.intr_conf.rxq) {
		/* According to datasheet, only vector 0/1/2 can be used,
		 * now only one vector is used for Rx queue
		 */
		intr_vector = 1;
		if (rte_intr_efd_enable(intr_handle, intr_vector))
			return -1;
	}

	if (rte_intr_dp_is_en(intr_handle) && !intr_handle->intr_vec) {
		intr_handle->intr_vec =
			rte_zmalloc("intr_vec",
				    dev->data->nb_rx_queues * sizeof(int), 0);
		if (intr_handle->intr_vec == NULL) {
			PMD_INIT_LOG(ERR, "Failed to allocate %d rx_queues"
				     " intr_vec", dev->data->nb_rx_queues);
			return -ENOMEM;
		}
	}
	yusur2vf_configure_msix(dev);

	/* When a VF port is bound to VFIO-PCI, only miscellaneous interrupt
	 * is mapped to VFIO vector 0 in eth_yusur2vf_dev_init( ).
	 * If previous VFIO interrupt mapping setting in eth_yusur2vf_dev_init( )
	 * is not cleared, it will fail when following rte_intr_enable( ) tries
	 * to map Rx queue interrupt to other VFIO vectors.
	 * So clear uio/vfio intr/evevnfd first to avoid failure.
	 */
	rte_intr_disable(intr_handle);

	rte_intr_enable(intr_handle);

	/* Re-enable interrupt for VF */
	yusur2vf_intr_enable(dev);

	/*
	 * Update link status right before return, because it may
	 * start link configuration process in a separate thread.
	 */
	yusur2vf_dev_link_update(dev, 0);

	hw->adapter_stopped = false;

	return 0;
}

static void
yusur2vf_dev_stop(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;

	if (hw->adapter_stopped)
		return;

	PMD_INIT_FUNC_TRACE();

	yusur2_dev_wait_setup_link_complete(dev, 0);

	yusur2vf_intr_disable(dev);

	hw->adapter_stopped = 1;
	yusur2_stop_adapter(hw);

	/*
	  * Clear what we set, but we still keep shadow_vfta to
	  * restore after device starts
	  */
	yusur2vf_set_vfta_all(dev, 0);

	/* Clear stored conf */
	dev->data->scattered_rx = 0;

	yusur2_dev_clear_queues(dev);

	/* Clean datapath event and queue/vec mapping */
	rte_intr_efd_disable(intr_handle);
	if (intr_handle->intr_vec != NULL) {
		rte_free(intr_handle->intr_vec);
		intr_handle->intr_vec = NULL;
	}

	adapter->rss_reta_updated = 0;
}

static void
yusur2vf_dev_close(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;

	PMD_INIT_FUNC_TRACE();

	yusur2_reset_hw(hw);

	yusur2vf_dev_stop(dev);

	yusur2_dev_free_queues(dev);

	/**
	 * Remove the VF MAC address ro ensure
	 * that the VF traffic goes to the PF
	 * after stop, close and detach of the VF
	 **/
	yusur2vf_remove_mac_addr(dev, 0);

	dev->dev_ops = NULL;
	dev->rx_pkt_burst = NULL;
	dev->tx_pkt_burst = NULL;

	rte_intr_disable(intr_handle);
	rte_intr_callback_unregister(intr_handle,
				     yusur2vf_dev_interrupt_handler, dev);
}

/*
 * Reset VF device
 */
static int
yusur2vf_dev_reset(struct rte_eth_dev *dev)
{
	int ret;

	ret = eth_yusur2vf_dev_uninit(dev);
	if (ret)
		return ret;

	ret = eth_yusur2vf_dev_init(dev);

	return ret;
}

static void yusur2vf_set_vfta_all(struct rte_eth_dev *dev, bool on)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	int i = 0, j = 0, vfta = 0, mask = 1;

	for (i = 0; i < YUSUR2_VFTA_SIZE; i++) {
		vfta = shadow_vfta->vfta[i];
		if (vfta) {
			mask = 1;
			for (j = 0; j < 32; j++) {
				if (vfta & mask)
					yusur2_set_vfta(hw, (i<<5)+j, 0,
						       on, false);
				mask <<= 1;
			}
		}
	}

}

static int
yusur2vf_vlan_filter_set(struct rte_eth_dev *dev, uint16_t vlan_id, int on)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vfta *shadow_vfta =
		YUSUR2_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	uint32_t vid_idx = 0;
	uint32_t vid_bit = 0;
	int ret = 0;

	PMD_INIT_FUNC_TRACE();

	/* vind is not used in VF driver, set to 0, check yusur2_set_vfta_vf */
	ret = yusur2_set_vfta(hw, vlan_id, 0, !!on, false);
	if (ret) {
		PMD_INIT_LOG(ERR, "Unable to set VF vlan");
		return ret;
	}
	vid_idx = (uint32_t) ((vlan_id >> 5) & 0x7F);
	vid_bit = (uint32_t) (1 << (vlan_id & 0x1F));

	/* Save what we set and retore it after device reset */
	if (on)
		shadow_vfta->vfta[vid_idx] |= vid_bit;
	else
		shadow_vfta->vfta[vid_idx] &= ~vid_bit;

	return 0;
}

static void
yusur2vf_vlan_strip_queue_set(struct rte_eth_dev *dev, uint16_t queue, int on)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	PMD_INIT_FUNC_TRACE();

	if (queue >= hw->mac.max_rx_queues)
		return;

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_RXDCTL(queue));
	if (on)
		ctrl |= YUSUR2_RXDCTL_VME;
	else
		ctrl &= ~YUSUR2_RXDCTL_VME;
	YUSUR2_WRITE_REG(hw, YUSUR2_RXDCTL(queue), ctrl);

	yusur2_vlan_hw_strip_bitmap_set(dev, queue, on);
}

static int
yusur2vf_vlan_offload_config(struct rte_eth_dev *dev, int mask)
{
	struct yusur2_rx_queue *rxq;
	uint16_t i;
	int on = 0;

	/* VF function only support hw strip feature, others are not support */
	if (mask & ETH_VLAN_STRIP_MASK) {
		for (i = 0; i < dev->data->nb_rx_queues; i++) {
			rxq = dev->data->rx_queues[i];
			on = !!(rxq->offloads &	DEV_RX_OFFLOAD_VLAN_STRIP);
			yusur2vf_vlan_strip_queue_set(dev, i, on);
		}
	}

	return 0;
}

static int
yusur2vf_vlan_offload_set(struct rte_eth_dev *dev, int mask)
{
	yusur2_config_vlan_strip_on_all_queues(dev, mask);

	yusur2vf_vlan_offload_config(dev, mask);

	return 0;
}

int
yusur2_vt_check(struct yusur2_hw *hw)
{
	uint32_t reg_val;

	/* if Virtualization Technology is enabled */
	reg_val = YUSUR2_READ_REG(hw, YUSUR2_VT_CTL);
	if (!(reg_val & YUSUR2_VT_CTL_VT_ENABLE)) {
		PMD_INIT_LOG(ERR, "VT must be enabled for this setting");
		return -1;
	}

	return 0;
}

static uint32_t
yusur2_uta_vector(struct yusur2_hw *hw, struct rte_ether_addr *uc_addr)
{
	uint32_t vector = 0;

	switch (hw->mac.mc_filter_type) {
	case 0:   /* use bits [47:36] of the address */
		vector = ((uc_addr->addr_bytes[4] >> 4) |
			(((uint16_t)uc_addr->addr_bytes[5]) << 4));
		break;
	case 1:   /* use bits [46:35] of the address */
		vector = ((uc_addr->addr_bytes[4] >> 3) |
			(((uint16_t)uc_addr->addr_bytes[5]) << 5));
		break;
	case 2:   /* use bits [45:34] of the address */
		vector = ((uc_addr->addr_bytes[4] >> 2) |
			(((uint16_t)uc_addr->addr_bytes[5]) << 6));
		break;
	case 3:   /* use bits [43:32] of the address */
		vector = ((uc_addr->addr_bytes[4]) |
			(((uint16_t)uc_addr->addr_bytes[5]) << 8));
		break;
	default:  /* Invalid mc_filter_type */
		break;
	}

	/* vector can only be 12-bits or boundary will be exceeded */
	vector &= 0xFFF;
	return vector;
}

static int
yusur2_uc_hash_table_set(struct rte_eth_dev *dev,
			struct rte_ether_addr *mac_addr, uint8_t on)
{
	uint32_t vector;
	uint32_t uta_idx;
	uint32_t reg_val;
	uint32_t uta_shift;
	uint32_t rc;
	const uint32_t yusur2_uta_idx_mask = 0x7F;
	const uint32_t yusur2_uta_bit_shift = 5;
	const uint32_t yusur2_uta_bit_mask = (0x1 << yusur2_uta_bit_shift) - 1;
	const uint32_t bit1 = 0x1;

	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_uta_info *uta_info =
		YUSUR2_DEV_PRIVATE_TO_UTA(dev->data->dev_private);

	/* The UTA table only exists on 82599 hardware and newer */
	if (hw->mac.type < yusur2_mac_82599EB)
		return -ENOTSUP;

	vector = yusur2_uta_vector(hw, mac_addr);
	uta_idx = (vector >> yusur2_uta_bit_shift) & yusur2_uta_idx_mask;
	uta_shift = vector & yusur2_uta_bit_mask;

	rc = ((uta_info->uta_shadow[uta_idx] >> uta_shift & bit1) != 0);
	if (rc == on)
		return 0;

	reg_val = YUSUR2_READ_REG(hw, YUSUR2_UTA(uta_idx));
	if (on) {
		uta_info->uta_in_use++;
		reg_val |= (bit1 << uta_shift);
		uta_info->uta_shadow[uta_idx] |= (bit1 << uta_shift);
	} else {
		uta_info->uta_in_use--;
		reg_val &= ~(bit1 << uta_shift);
		uta_info->uta_shadow[uta_idx] &= ~(bit1 << uta_shift);
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_UTA(uta_idx), reg_val);

	if (uta_info->uta_in_use > 0)
		YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL,
				YUSUR2_MCSTCTRL_MFE | hw->mac.mc_filter_type);
	else
		YUSUR2_WRITE_REG(hw, YUSUR2_MCSTCTRL, hw->mac.mc_filter_type);

	return 0;
}

static int
yusur2_uc_all_hash_table_set(struct rte_eth_dev *dev, uint8_t on)
{
	int i;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_uta_info *uta_info =
		YUSUR2_DEV_PRIVATE_TO_UTA(dev->data->dev_private);

	/* The UTA table only exists on 82599 hardware and newer */
	if (hw->mac.type < yusur2_mac_82599EB)
		return -ENOTSUP;

	if (on) {
		for (i = 0; i < ETH_VMDQ_NUM_UC_HASH_ARRAY; i++) {
			uta_info->uta_shadow[i] = ~0;
			YUSUR2_WRITE_REG(hw, YUSUR2_UTA(i), ~0);
		}
	} else {
		for (i = 0; i < ETH_VMDQ_NUM_UC_HASH_ARRAY; i++) {
			uta_info->uta_shadow[i] = 0;
			YUSUR2_WRITE_REG(hw, YUSUR2_UTA(i), 0);
		}
	}
	return 0;

}

uint32_t
yusur2_convert_vm_rx_mask_to_val(uint16_t rx_mask, uint32_t orig_val)
{
	uint32_t new_val = orig_val;

	if (rx_mask & ETH_VMDQ_ACCEPT_UNTAG)
		new_val |= YUSUR2_VMOLR_AUPE;
	if (rx_mask & ETH_VMDQ_ACCEPT_HASH_MC)
		new_val |= YUSUR2_VMOLR_ROMPE;
	if (rx_mask & ETH_VMDQ_ACCEPT_HASH_UC)
		new_val |= YUSUR2_VMOLR_ROPE;
	if (rx_mask & ETH_VMDQ_ACCEPT_BROADCAST)
		new_val |= YUSUR2_VMOLR_BAM;
	if (rx_mask & ETH_VMDQ_ACCEPT_MULTICAST)
		new_val |= YUSUR2_VMOLR_MPE;

	return new_val;
}

#define YUSUR2_MRCTL_VPME  0x01 /* Virtual Pool Mirroring. */
#define YUSUR2_MRCTL_UPME  0x02 /* Uplink Port Mirroring. */
#define YUSUR2_MRCTL_DPME  0x04 /* Downlink Port Mirroring. */
#define YUSUR2_MRCTL_VLME  0x08 /* VLAN Mirroring. */
#define YUSUR2_INVALID_MIRROR_TYPE(mirror_type) \
	((mirror_type) & ~(uint8_t)(ETH_MIRROR_VIRTUAL_POOL_UP | \
	ETH_MIRROR_UPLINK_PORT | ETH_MIRROR_DOWNLINK_PORT | ETH_MIRROR_VLAN))

static int
yusur2_mirror_rule_set(struct rte_eth_dev *dev,
		      struct rte_eth_mirror_conf *mirror_conf,
		      uint8_t rule_id, uint8_t on)
{
	uint32_t mr_ctl, vlvf;
	uint32_t mp_lsb = 0;
	uint32_t mv_msb = 0;
	uint32_t mv_lsb = 0;
	uint32_t mp_msb = 0;
	uint8_t i = 0;
	int reg_index = 0;
	uint64_t vlan_mask = 0;

	const uint8_t pool_mask_offset = 32;
	const uint8_t vlan_mask_offset = 32;
	const uint8_t dst_pool_offset = 8;
	const uint8_t rule_mr_offset  = 4;
	const uint8_t mirror_rule_mask = 0x0F;

	struct yusur2_mirror_info *mr_info =
			(YUSUR2_DEV_PRIVATE_TO_PFDATA(dev->data->dev_private));
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint8_t mirror_type = 0;

	if (yusur2_vt_check(hw) < 0)
		return -ENOTSUP;

	if (rule_id >= YUSUR2_MAX_MIRROR_RULES)
		return -EINVAL;

	if (YUSUR2_INVALID_MIRROR_TYPE(mirror_conf->rule_type)) {
		PMD_DRV_LOG(ERR, "unsupported mirror type 0x%x.",
			    mirror_conf->rule_type);
		return -EINVAL;
	}

	if (mirror_conf->rule_type & ETH_MIRROR_VLAN) {
		mirror_type |= YUSUR2_MRCTL_VLME;
		/* Check if vlan id is valid and find conresponding VLAN ID
		 * index in VLVF
		 */
		for (i = 0; i < YUSUR2_VLVF_ENTRIES; i++) {
			if (mirror_conf->vlan.vlan_mask & (1ULL << i)) {
				/* search vlan id related pool vlan filter
				 * index
				 */
				reg_index = yusur2_find_vlvf_slot(
						hw,
						mirror_conf->vlan.vlan_id[i],
						false);
				if (reg_index < 0)
					return -EINVAL;
				vlvf = YUSUR2_READ_REG(hw,
						      YUSUR2_VLVF(reg_index));
				if ((vlvf & YUSUR2_VLVF_VIEN) &&
				    ((vlvf & YUSUR2_VLVF_VLANID_MASK) ==
				      mirror_conf->vlan.vlan_id[i]))
					vlan_mask |= (1ULL << reg_index);
				else
					return -EINVAL;
			}
		}

		if (on) {
			mv_lsb = vlan_mask & 0xFFFFFFFF;
			mv_msb = vlan_mask >> vlan_mask_offset;

			mr_info->mr_conf[rule_id].vlan.vlan_mask =
						mirror_conf->vlan.vlan_mask;
			for (i = 0; i < ETH_VMDQ_MAX_VLAN_FILTERS; i++) {
				if (mirror_conf->vlan.vlan_mask & (1ULL << i))
					mr_info->mr_conf[rule_id].vlan.vlan_id[i] =
						mirror_conf->vlan.vlan_id[i];
			}
		} else {
			mv_lsb = 0;
			mv_msb = 0;
			mr_info->mr_conf[rule_id].vlan.vlan_mask = 0;
			for (i = 0; i < ETH_VMDQ_MAX_VLAN_FILTERS; i++)
				mr_info->mr_conf[rule_id].vlan.vlan_id[i] = 0;
		}
	}

	/**
	 * if enable pool mirror, write related pool mask register,if disable
	 * pool mirror, clear PFMRVM register
	 */
	if (mirror_conf->rule_type & ETH_MIRROR_VIRTUAL_POOL_UP) {
		mirror_type |= YUSUR2_MRCTL_VPME;
		if (on) {
			mp_lsb = mirror_conf->pool_mask & 0xFFFFFFFF;
			mp_msb = mirror_conf->pool_mask >> pool_mask_offset;
			mr_info->mr_conf[rule_id].pool_mask =
					mirror_conf->pool_mask;

		} else {
			mp_lsb = 0;
			mp_msb = 0;
			mr_info->mr_conf[rule_id].pool_mask = 0;
		}
	}
	if (mirror_conf->rule_type & ETH_MIRROR_UPLINK_PORT)
		mirror_type |= YUSUR2_MRCTL_UPME;
	if (mirror_conf->rule_type & ETH_MIRROR_DOWNLINK_PORT)
		mirror_type |= YUSUR2_MRCTL_DPME;

	/* read  mirror control register and recalculate it */
	mr_ctl = YUSUR2_READ_REG(hw, YUSUR2_MRCTL(rule_id));

	if (on) {
		mr_ctl |= mirror_type;
		mr_ctl &= mirror_rule_mask;
		mr_ctl |= mirror_conf->dst_pool << dst_pool_offset;
	} else {
		mr_ctl &= ~(mirror_conf->rule_type & mirror_rule_mask);
	}

	mr_info->mr_conf[rule_id].rule_type = mirror_conf->rule_type;
	mr_info->mr_conf[rule_id].dst_pool = mirror_conf->dst_pool;

	/* write mirrror control  register */
	YUSUR2_WRITE_REG(hw, YUSUR2_MRCTL(rule_id), mr_ctl);

	/* write pool mirrror control  register */
	if (mirror_conf->rule_type & ETH_MIRROR_VIRTUAL_POOL_UP) {
		YUSUR2_WRITE_REG(hw, YUSUR2_VMRVM(rule_id), mp_lsb);
		YUSUR2_WRITE_REG(hw, YUSUR2_VMRVM(rule_id + rule_mr_offset),
				mp_msb);
	}
	/* write VLAN mirrror control  register */
	if (mirror_conf->rule_type & ETH_MIRROR_VLAN) {
		YUSUR2_WRITE_REG(hw, YUSUR2_VMRVLAN(rule_id), mv_lsb);
		YUSUR2_WRITE_REG(hw, YUSUR2_VMRVLAN(rule_id + rule_mr_offset),
				mv_msb);
	}

	return 0;
}

static int
yusur2_mirror_rule_reset(struct rte_eth_dev *dev, uint8_t rule_id)
{
	int mr_ctl = 0;
	uint32_t lsb_val = 0;
	uint32_t msb_val = 0;
	const uint8_t rule_mr_offset = 4;

	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_mirror_info *mr_info =
		(YUSUR2_DEV_PRIVATE_TO_PFDATA(dev->data->dev_private));

	if (yusur2_vt_check(hw) < 0)
		return -ENOTSUP;

	if (rule_id >= YUSUR2_MAX_MIRROR_RULES)
		return -EINVAL;

	memset(&mr_info->mr_conf[rule_id], 0,
	       sizeof(struct rte_eth_mirror_conf));

	/* clear PFVMCTL register */
	YUSUR2_WRITE_REG(hw, YUSUR2_MRCTL(rule_id), mr_ctl);

	/* clear pool mask register */
	YUSUR2_WRITE_REG(hw, YUSUR2_VMRVM(rule_id), lsb_val);
	YUSUR2_WRITE_REG(hw, YUSUR2_VMRVM(rule_id + rule_mr_offset), msb_val);

	/* clear vlan mask register */
	YUSUR2_WRITE_REG(hw, YUSUR2_VMRVLAN(rule_id), lsb_val);
	YUSUR2_WRITE_REG(hw, YUSUR2_VMRVLAN(rule_id + rule_mr_offset), msb_val);

	return 0;
}

static int
yusur2vf_dev_rx_queue_intr_enable(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t vec = YUSUR2_MISC_VEC_ID;

	if (rte_intr_allow_others(intr_handle))
		vec = YUSUR2_RX_VEC_START;
	intr->mask |= (1 << vec);
	RTE_SET_USED(queue_id);
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIMS, intr->mask);

	rte_intr_ack(intr_handle);

	return 0;
}

static int
yusur2vf_dev_rx_queue_intr_disable(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	uint32_t vec = YUSUR2_MISC_VEC_ID;

	if (rte_intr_allow_others(intr_handle))
		vec = YUSUR2_RX_VEC_START;
	intr->mask &= ~(1 << vec);
	RTE_SET_USED(queue_id);
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEIMS, intr->mask);

	return 0;
}

static int
yusur2_dev_rx_queue_intr_enable(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	uint32_t mask;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	if (queue_id < 16) {
		yusur2_disable_intr(hw);
		intr->mask |= (1 << queue_id);
		yusur2_enable_intr(dev);
	} else if (queue_id < 32) {
		mask = YUSUR2_READ_REG(hw, YUSUR2_EIMS_EX(0));
		mask &= (1 << queue_id);
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMS_EX(0), mask);
	} else if (queue_id < 64) {
		mask = YUSUR2_READ_REG(hw, YUSUR2_EIMS_EX(1));
		mask &= (1 << (queue_id - 32));
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMS_EX(1), mask);
	}
	rte_intr_ack(intr_handle);

	return 0;
}

static int
yusur2_dev_rx_queue_intr_disable(struct rte_eth_dev *dev, uint16_t queue_id)
{
	uint32_t mask;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	if (queue_id < 16) {
		yusur2_disable_intr(hw);
		intr->mask &= ~(1 << queue_id);
		yusur2_enable_intr(dev);
	} else if (queue_id < 32) {
		mask = YUSUR2_READ_REG(hw, YUSUR2_EIMS_EX(0));
		mask &= ~(1 << queue_id);
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMS_EX(0), mask);
	} else if (queue_id < 64) {
		mask = YUSUR2_READ_REG(hw, YUSUR2_EIMS_EX(1));
		mask &= ~(1 << (queue_id - 32));
		YUSUR2_WRITE_REG(hw, YUSUR2_EIMS_EX(1), mask);
	}

	return 0;
}

static void
yusur2vf_set_ivar_map(struct yusur2_hw *hw, int8_t direction,
		     uint8_t queue, uint8_t msix_vector)
{
	uint32_t tmp, idx;

	if (direction == -1) {
		/* other causes */
		msix_vector |= YUSUR2_IVAR_ALLOC_VAL;
		tmp = YUSUR2_READ_REG(hw, YUSUR2_VTIVAR_MISC);
		tmp &= ~0xFF;
		tmp |= msix_vector;
		YUSUR2_WRITE_REG(hw, YUSUR2_VTIVAR_MISC, tmp);
	} else {
		/* rx or tx cause */
		msix_vector |= YUSUR2_IVAR_ALLOC_VAL;
		idx = ((16 * (queue & 1)) + (8 * direction));
		tmp = YUSUR2_READ_REG(hw, YUSUR2_VTIVAR(queue >> 1));
		tmp &= ~(0xFF << idx);
		tmp |= (msix_vector << idx);
		YUSUR2_WRITE_REG(hw, YUSUR2_VTIVAR(queue >> 1), tmp);
	}
}

/**
 * set the IVAR registers, mapping interrupt causes to vectors
 * @param hw
 *  pointer to yusur2_hw struct
 * @direction
 *  0 for Rx, 1 for Tx, -1 for other causes
 * @queue
 *  queue to map the corresponding interrupt to
 * @msix_vector
 *  the vector to map to the corresponding queue
 */
static void
yusur2_set_ivar_map(struct yusur2_hw *hw, int8_t direction,
		   uint8_t queue, uint8_t msix_vector)
{
	uint32_t tmp, idx;

	msix_vector |= YUSUR2_IVAR_ALLOC_VAL;
	if (hw->mac.type == yusur2_mac_82598EB) {
		if (direction == -1)
			direction = 0;
		idx = (((direction * 64) + queue) >> 2) & 0x1F;
		tmp = YUSUR2_READ_REG(hw, YUSUR2_IVAR(idx));
		tmp &= ~(0xFF << (8 * (queue & 0x3)));
		tmp |= (msix_vector << (8 * (queue & 0x3)));
		YUSUR2_WRITE_REG(hw, YUSUR2_IVAR(idx), tmp);
	} else if ((hw->mac.type == yusur2_mac_82599EB) ||
			(hw->mac.type == yusur2_mac_X540) ||
			(hw->mac.type == yusur2_mac_X550) ||
			(hw->mac.type == yusur2_mac_X550EM_x)) {
		if (direction == -1) {
			/* other causes */
			idx = ((queue & 1) * 8);
			tmp = YUSUR2_READ_REG(hw, YUSUR2_IVAR_MISC);
			tmp &= ~(0xFF << idx);
			tmp |= (msix_vector << idx);
			YUSUR2_WRITE_REG(hw, YUSUR2_IVAR_MISC, tmp);
		} else {
			/* rx or tx causes */
			idx = ((16 * (queue & 1)) + (8 * direction));
			tmp = YUSUR2_READ_REG(hw, YUSUR2_IVAR(queue >> 1));
			tmp &= ~(0xFF << idx);
			tmp |= (msix_vector << idx);
			YUSUR2_WRITE_REG(hw, YUSUR2_IVAR(queue >> 1), tmp);
		}
	}
}

static void
yusur2vf_configure_msix(struct rte_eth_dev *dev)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t q_idx;
	uint32_t vector_idx = YUSUR2_MISC_VEC_ID;
	uint32_t base = YUSUR2_MISC_VEC_ID;

	/* Configure VF other cause ivar */
	yusur2vf_set_ivar_map(hw, -1, 1, vector_idx);

	/* won't configure msix register if no mapping is done
	 * between intr vector and event fd.
	 */
	if (!rte_intr_dp_is_en(intr_handle))
		return;

	if (rte_intr_allow_others(intr_handle)) {
		base = YUSUR2_RX_VEC_START;
		vector_idx = YUSUR2_RX_VEC_START;
	}

	/* Configure all RX queues of VF */
	for (q_idx = 0; q_idx < dev->data->nb_rx_queues; q_idx++) {
		/* Force all queue use vector 0,
		 * as YUSUR2_VF_MAXMSIVECOTR = 1
		 */
		yusur2vf_set_ivar_map(hw, 0, q_idx, vector_idx);
		intr_handle->intr_vec[q_idx] = vector_idx;
		if (vector_idx < base + intr_handle->nb_efd - 1)
			vector_idx++;
	}

	/* As RX queue setting above show, all queues use the vector 0.
	 * Set only the ITR value of YUSUR2_MISC_VEC_ID.
	 */
	YUSUR2_WRITE_REG(hw, YUSUR2_VTEITR(YUSUR2_MISC_VEC_ID),
			YUSUR2_EITR_INTERVAL_US(YUSUR2_QUEUE_ITR_INTERVAL_DEFAULT)
			| YUSUR2_EITR_CNT_WDIS);
}

/**
 * Sets up the hardware to properly generate MSI-X interrupts
 * @hw
 *  board private structure
 */
static void
yusur2_configure_msix(struct rte_eth_dev *dev)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_intr_handle *intr_handle = &pci_dev->intr_handle;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t queue_id, base = YUSUR2_MISC_VEC_ID;
	uint32_t vec = YUSUR2_MISC_VEC_ID;
	uint32_t mask;
	uint32_t gpie;

	/* won't configure msix register if no mapping is done
	 * between intr vector and event fd
	 * but if misx has been enabled already, need to configure
	 * auto clean, auto mask and throttling.
	 */
	gpie = YUSUR2_READ_REG(hw, YUSUR2_GPIE);
	if (!rte_intr_dp_is_en(intr_handle) &&
	    !(gpie & (YUSUR2_GPIE_MSIX_MODE | YUSUR2_GPIE_PBA_SUPPORT)))
		return;

	if (rte_intr_allow_others(intr_handle))
		vec = base = YUSUR2_RX_VEC_START;

	/* setup GPIE for MSI-x mode */
	gpie = YUSUR2_READ_REG(hw, YUSUR2_GPIE);
	gpie |= YUSUR2_GPIE_MSIX_MODE | YUSUR2_GPIE_PBA_SUPPORT |
		YUSUR2_GPIE_OCD | YUSUR2_GPIE_EIAME;
	/* auto clearing and auto setting corresponding bits in EIMS
	 * when MSI-X interrupt is triggered
	 */
	if (hw->mac.type == yusur2_mac_82598EB) {
		YUSUR2_WRITE_REG(hw, YUSUR2_EIAM, YUSUR2_EICS_RTX_QUEUE);
	} else {
		YUSUR2_WRITE_REG(hw, YUSUR2_EIAM_EX(0), 0xFFFFFFFF);
		YUSUR2_WRITE_REG(hw, YUSUR2_EIAM_EX(1), 0xFFFFFFFF);
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_GPIE, gpie);

	/* Populate the IVAR table and set the ITR values to the
	 * corresponding register.
	 */
	if (rte_intr_dp_is_en(intr_handle)) {
		for (queue_id = 0; queue_id < dev->data->nb_rx_queues;
			queue_id++) {
			/* by default, 1:1 mapping */
			yusur2_set_ivar_map(hw, 0, queue_id, vec);
			intr_handle->intr_vec[queue_id] = vec;
			if (vec < base + intr_handle->nb_efd - 1)
				vec++;
		}

		switch (hw->mac.type) {
		case yusur2_mac_82598EB:
			yusur2_set_ivar_map(hw, -1,
					   YUSUR2_IVAR_OTHER_CAUSES_INDEX,
					   YUSUR2_MISC_VEC_ID);
			break;
		case yusur2_mac_82599EB:
		case yusur2_mac_X540:
		case yusur2_mac_X550:
		case yusur2_mac_X550EM_x:
			yusur2_set_ivar_map(hw, -1, 1, YUSUR2_MISC_VEC_ID);
			break;
		default:
			break;
		}
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_EITR(YUSUR2_MISC_VEC_ID),
			YUSUR2_EITR_INTERVAL_US(YUSUR2_QUEUE_ITR_INTERVAL_DEFAULT)
			| YUSUR2_EITR_CNT_WDIS);

	/* set up to autoclear timer, and the vectors */
	mask = YUSUR2_EIMS_ENABLE_MASK;
	mask &= ~(YUSUR2_EIMS_OTHER |
		  YUSUR2_EIMS_MAILBOX |
		  YUSUR2_EIMS_LSC);

	YUSUR2_WRITE_REG(hw, YUSUR2_EIAC, mask);
}

int
yusur2_set_queue_rate_limit(struct rte_eth_dev *dev,
			   uint16_t queue_idx, uint16_t tx_rate)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_rxmode *rxmode;
	uint32_t rf_dec, rf_int;
	uint32_t bcnrc_val;
	uint16_t link_speed = dev->data->dev_link.link_speed;

	if (queue_idx >= hw->mac.max_tx_queues)
		return -EINVAL;

	if (tx_rate != 0) {
		/* Calculate the rate factor values to set */
		rf_int = (uint32_t)link_speed / (uint32_t)tx_rate;
		rf_dec = (uint32_t)link_speed % (uint32_t)tx_rate;
		rf_dec = (rf_dec << YUSUR2_RTTBCNRC_RF_INT_SHIFT) / tx_rate;

		bcnrc_val = YUSUR2_RTTBCNRC_RS_ENA;
		bcnrc_val |= ((rf_int << YUSUR2_RTTBCNRC_RF_INT_SHIFT) &
				YUSUR2_RTTBCNRC_RF_INT_MASK_M);
		bcnrc_val |= (rf_dec & YUSUR2_RTTBCNRC_RF_DEC_MASK);
	} else {
		bcnrc_val = 0;
	}

	rxmode = &dev->data->dev_conf.rxmode;
	/*
	 * Set global transmit compensation time to the MMW_SIZE in RTTBCNRM
	 * register. MMW_SIZE=0x014 if 9728-byte jumbo is supported, otherwise
	 * set as 0x4.
	 */
	if ((rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME) &&
	    (rxmode->max_rx_pkt_len >= YUSUR2_MAX_JUMBO_FRAME_SIZE))
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTBCNRM,
			YUSUR2_MMW_SIZE_JUMBO_FRAME);
	else
		YUSUR2_WRITE_REG(hw, YUSUR2_RTTBCNRM,
			YUSUR2_MMW_SIZE_DEFAULT);

	/* Set RTTBCNRC of queue X */
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTDQSEL, queue_idx);
	YUSUR2_WRITE_REG(hw, YUSUR2_RTTBCNRC, bcnrc_val);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

static int
yusur2vf_add_mac_addr(struct rte_eth_dev *dev, struct rte_ether_addr *mac_addr,
		     __attribute__((unused)) uint32_t index,
		     __attribute__((unused)) uint32_t pool)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int diag;

	/*
	 * On a 82599 VF, adding again the same MAC addr is not an idempotent
	 * operation. Trap this case to avoid exhausting the [very limited]
	 * set of PF resources used to store VF MAC addresses.
	 */
	if (memcmp(hw->mac.perm_addr, mac_addr,
			sizeof(struct rte_ether_addr)) == 0)
		return -1;
	diag = yusur2vf_set_uc_addr_vf(hw, 2, mac_addr->addr_bytes);
	if (diag != 0)
		PMD_DRV_LOG(ERR, "Unable to add MAC address "
			    "%02x:%02x:%02x:%02x:%02x:%02x - diag=%d",
			    mac_addr->addr_bytes[0],
			    mac_addr->addr_bytes[1],
			    mac_addr->addr_bytes[2],
			    mac_addr->addr_bytes[3],
			    mac_addr->addr_bytes[4],
			    mac_addr->addr_bytes[5],
			    diag);
	return diag;
}

static void
yusur2vf_remove_mac_addr(struct rte_eth_dev *dev, uint32_t index)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_ether_addr *perm_addr =
		(struct rte_ether_addr *)hw->mac.perm_addr;
	struct rte_ether_addr *mac_addr;
	uint32_t i;
	int diag;

	/*
	 * The YUSUR2_VF_SET_MACVLAN command of the yusur2-pf driver does
	 * not support the deletion of a given MAC address.
	 * Instead, it imposes to delete all MAC addresses, then to add again
	 * all MAC addresses with the exception of the one to be deleted.
	 */
	(void) yusur2vf_set_uc_addr_vf(hw, 0, NULL);

	/*
	 * Add again all MAC addresses, with the exception of the deleted one
	 * and of the permanent MAC address.
	 */
	for (i = 0, mac_addr = dev->data->mac_addrs;
	     i < hw->mac.num_rar_entries; i++, mac_addr++) {
		/* Skip the deleted MAC address */
		if (i == index)
			continue;
		/* Skip NULL MAC addresses */
		if (rte_is_zero_ether_addr(mac_addr))
			continue;
		/* Skip the permanent MAC address */
		if (memcmp(perm_addr, mac_addr,
				sizeof(struct rte_ether_addr)) == 0)
			continue;
		diag = yusur2vf_set_uc_addr_vf(hw, 2, mac_addr->addr_bytes);
		if (diag != 0)
			PMD_DRV_LOG(ERR,
				    "Adding again MAC address "
				    "%02x:%02x:%02x:%02x:%02x:%02x failed "
				    "diag=%d",
				    mac_addr->addr_bytes[0],
				    mac_addr->addr_bytes[1],
				    mac_addr->addr_bytes[2],
				    mac_addr->addr_bytes[3],
				    mac_addr->addr_bytes[4],
				    mac_addr->addr_bytes[5],
				    diag);
	}
}

static int
yusur2vf_set_default_mac_addr(struct rte_eth_dev *dev,
			struct rte_ether_addr *addr)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	hw->mac.ops.set_rar(hw, 0, (void *)addr, 0, 0);

	return 0;
}

int
yusur2_syn_filter_set(struct rte_eth_dev *dev,
			struct rte_eth_syn_filter *filter,
			bool add)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	uint32_t syn_info;
	uint32_t synqf;

	if (filter->queue >= YUSUR2_MAX_RX_QUEUE_NUM)
		return -EINVAL;

	syn_info = filter_info->syn_info;

	if (add) {
		if (syn_info & YUSUR2_SYN_FILTER_ENABLE)
			return -EINVAL;
		synqf = (uint32_t)(((filter->queue << YUSUR2_SYN_FILTER_QUEUE_SHIFT) &
			YUSUR2_SYN_FILTER_QUEUE) | YUSUR2_SYN_FILTER_ENABLE);

		if (filter->hig_pri)
			synqf |= YUSUR2_SYN_FILTER_SYNQFP;
		else
			synqf &= ~YUSUR2_SYN_FILTER_SYNQFP;
	} else {
		synqf = YUSUR2_READ_REG(hw, YUSUR2_SYNQF);
		if (!(syn_info & YUSUR2_SYN_FILTER_ENABLE))
			return -ENOENT;
		synqf &= ~(YUSUR2_SYN_FILTER_QUEUE | YUSUR2_SYN_FILTER_ENABLE);
	}

	filter_info->syn_info = synqf;
	YUSUR2_WRITE_REG(hw, YUSUR2_SYNQF, synqf);
	YUSUR2_WRITE_FLUSH(hw);
	return 0;
}

static int
yusur2_syn_filter_get(struct rte_eth_dev *dev,
			struct rte_eth_syn_filter *filter)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t synqf = YUSUR2_READ_REG(hw, YUSUR2_SYNQF);

	if (synqf & YUSUR2_SYN_FILTER_ENABLE) {
		filter->hig_pri = (synqf & YUSUR2_SYN_FILTER_SYNQFP) ? 1 : 0;
		filter->queue = (uint16_t)((synqf & YUSUR2_SYN_FILTER_QUEUE) >> 1);
		return 0;
	}
	return -ENOENT;
}

static int
yusur2_syn_filter_handle(struct rte_eth_dev *dev,
			enum rte_filter_op filter_op,
			void *arg)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	MAC_TYPE_FILTER_SUP(hw->mac.type);

	if (filter_op == RTE_ETH_FILTER_NOP)
		return 0;

	if (arg == NULL) {
		PMD_DRV_LOG(ERR, "arg shouldn't be NULL for operation %u",
			    filter_op);
		return -EINVAL;
	}

	switch (filter_op) {
	case RTE_ETH_FILTER_ADD:
		ret = yusur2_syn_filter_set(dev,
				(struct rte_eth_syn_filter *)arg,
				TRUE);
		break;
	case RTE_ETH_FILTER_DELETE:
		ret = yusur2_syn_filter_set(dev,
				(struct rte_eth_syn_filter *)arg,
				FALSE);
		break;
	case RTE_ETH_FILTER_GET:
		ret = yusur2_syn_filter_get(dev,
				(struct rte_eth_syn_filter *)arg);
		break;
	default:
		PMD_DRV_LOG(ERR, "unsupported operation %u", filter_op);
		ret = -EINVAL;
		break;
	}

	return ret;
}


static inline enum yusur2_5tuple_protocol
convert_protocol_type(uint8_t protocol_value)
{
	if (protocol_value == IPPROTO_TCP)
		return YUSUR2_FILTER_PROTOCOL_TCP;
	else if (protocol_value == IPPROTO_UDP)
		return YUSUR2_FILTER_PROTOCOL_UDP;
	else if (protocol_value == IPPROTO_SCTP)
		return YUSUR2_FILTER_PROTOCOL_SCTP;
	else
		return YUSUR2_FILTER_PROTOCOL_NONE;
}

/* inject a 5-tuple filter to HW */
static inline void
yusur2_inject_5tuple_filter(struct rte_eth_dev *dev,
			   struct yusur2_5tuple_filter *filter)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;
	uint32_t ftqf, sdpqf;
	uint32_t l34timir = 0;
	uint8_t mask = 0xff;

	i = filter->index;

	sdpqf = (uint32_t)(filter->filter_info.dst_port <<
				YUSUR2_SDPQF_DSTPORT_SHIFT);
	sdpqf = sdpqf | (filter->filter_info.src_port & YUSUR2_SDPQF_SRCPORT);

	ftqf = (uint32_t)(filter->filter_info.proto &
		YUSUR2_FTQF_PROTOCOL_MASK);
	ftqf |= (uint32_t)((filter->filter_info.priority &
		YUSUR2_FTQF_PRIORITY_MASK) << YUSUR2_FTQF_PRIORITY_SHIFT);
	if (filter->filter_info.src_ip_mask == 0) /* 0 means compare. */
		mask &= YUSUR2_FTQF_SOURCE_ADDR_MASK;
	if (filter->filter_info.dst_ip_mask == 0)
		mask &= YUSUR2_FTQF_DEST_ADDR_MASK;
	if (filter->filter_info.src_port_mask == 0)
		mask &= YUSUR2_FTQF_SOURCE_PORT_MASK;
	if (filter->filter_info.dst_port_mask == 0)
		mask &= YUSUR2_FTQF_DEST_PORT_MASK;
	if (filter->filter_info.proto_mask == 0)
		mask &= YUSUR2_FTQF_PROTOCOL_COMP_MASK;
	ftqf |= mask << YUSUR2_FTQF_5TUPLE_MASK_SHIFT;
	ftqf |= YUSUR2_FTQF_POOL_MASK_EN;
	ftqf |= YUSUR2_FTQF_QUEUE_ENABLE;

	YUSUR2_WRITE_REG(hw, YUSUR2_DAQF(i), filter->filter_info.dst_ip);
	YUSUR2_WRITE_REG(hw, YUSUR2_SAQF(i), filter->filter_info.src_ip);
	YUSUR2_WRITE_REG(hw, YUSUR2_SDPQF(i), sdpqf);
	YUSUR2_WRITE_REG(hw, YUSUR2_FTQF(i), ftqf);

	l34timir |= YUSUR2_L34T_IMIR_RESERVE;
	l34timir |= (uint32_t)(filter->queue <<
				YUSUR2_L34T_IMIR_QUEUE_SHIFT);
	YUSUR2_WRITE_REG(hw, YUSUR2_L34T_IMIR(i), l34timir);
}

/*
 * add a 5tuple filter
 *
 * @param
 * dev: Pointer to struct rte_eth_dev.
 * index: the index the filter allocates.
 * filter: ponter to the filter that will be added.
 * rx_queue: the queue id the filter assigned to.
 *
 * @return
 *    - On success, zero.
 *    - On failure, a negative value.
 */
static int
yusur2_add_5tuple_filter(struct rte_eth_dev *dev,
			struct yusur2_5tuple_filter *filter)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	int i, idx, shift;

	/*
	 * look for an unused 5tuple filter index,
	 * and insert the filter to list.
	 */
	for (i = 0; i < YUSUR2_MAX_FTQF_FILTERS; i++) {
		idx = i / (sizeof(uint32_t) * NBBY);
		shift = i % (sizeof(uint32_t) * NBBY);
		if (!(filter_info->fivetuple_mask[idx] & (1 << shift))) {
			filter_info->fivetuple_mask[idx] |= 1 << shift;
			filter->index = i;
			TAILQ_INSERT_TAIL(&filter_info->fivetuple_list,
					  filter,
					  entries);
			break;
		}
	}
	if (i >= YUSUR2_MAX_FTQF_FILTERS) {
		PMD_DRV_LOG(ERR, "5tuple filters are full.");
		return -ENOSYS;
	}

	yusur2_inject_5tuple_filter(dev, filter);

	return 0;
}

/*
 * remove a 5tuple filter
 *
 * @param
 * dev: Pointer to struct rte_eth_dev.
 * filter: the pointer of the filter will be removed.
 */
static void
yusur2_remove_5tuple_filter(struct rte_eth_dev *dev,
			struct yusur2_5tuple_filter *filter)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	uint16_t index = filter->index;

	filter_info->fivetuple_mask[index / (sizeof(uint32_t) * NBBY)] &=
				~(1 << (index % (sizeof(uint32_t) * NBBY)));
	TAILQ_REMOVE(&filter_info->fivetuple_list, filter, entries);
	rte_free(filter);

	YUSUR2_WRITE_REG(hw, YUSUR2_DAQF(index), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_SAQF(index), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_SDPQF(index), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_FTQF(index), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_L34T_IMIR(index), 0);
}

static int
yusur2vf_dev_set_mtu(struct rte_eth_dev *dev, uint16_t mtu)
{
	struct yusur2_hw *hw;
	uint32_t max_frame = mtu + YUSUR2_ETH_OVERHEAD;
	struct rte_eth_dev_data *dev_data = dev->data;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (mtu < RTE_ETHER_MIN_MTU ||
			max_frame > RTE_ETHER_MAX_JUMBO_FRAME_LEN)
		return -EINVAL;

	/* If device is started, refuse mtu that requires the support of
	 * scattered packets when this feature has not been enabled before.
	 */
	if (dev_data->dev_started && !dev_data->scattered_rx &&
	    (max_frame + 2 * YUSUR2_VLAN_TAG_SIZE >
	     dev->data->min_rx_buf_size - RTE_PKTMBUF_HEADROOM)) {
		PMD_INIT_LOG(ERR, "Stop port first.");
		return -EINVAL;
	}

	/*
	 * When supported by the underlying PF driver, use the YUSUR2_VF_SET_MTU
	 * request of the version 2.0 of the mailbox API.
	 * For now, use the YUSUR2_VF_SET_LPE request of the version 1.0
	 * of the mailbox API.
	 * This call to YUSUR2_SET_LPE action won't work with yusur2 pf drivers
	 * prior to 3.11.33 which contains the following change:
	 * "yusur2: Enable jumbo frames support w/ SR-IOV"
	 */
	yusur2vf_rlpml_set_vf(hw, max_frame);

	/* update max frame size */
	dev->data->dev_conf.rxmode.max_rx_pkt_len = max_frame;
	return 0;
}

static inline struct yusur2_5tuple_filter *
yusur2_5tuple_filter_lookup(struct yusur2_5tuple_filter_list *filter_list,
			struct yusur2_5tuple_filter_info *key)
{
	struct yusur2_5tuple_filter *it;

	TAILQ_FOREACH(it, filter_list, entries) {
		if (memcmp(key, &it->filter_info,
			sizeof(struct yusur2_5tuple_filter_info)) == 0) {
			return it;
		}
	}
	return NULL;
}

/* translate elements in struct rte_eth_ntuple_filter to struct yusur2_5tuple_filter_info*/
static inline int
ntuple_filter_to_5tuple(struct rte_eth_ntuple_filter *filter,
			struct yusur2_5tuple_filter_info *filter_info)
{
	if (filter->queue >= YUSUR2_MAX_RX_QUEUE_NUM ||
		filter->priority > YUSUR2_5TUPLE_MAX_PRI ||
		filter->priority < YUSUR2_5TUPLE_MIN_PRI)
		return -EINVAL;

	switch (filter->dst_ip_mask) {
	case UINT32_MAX:
		filter_info->dst_ip_mask = 0;
		filter_info->dst_ip = filter->dst_ip;
		break;
	case 0:
		filter_info->dst_ip_mask = 1;
		break;
	default:
		PMD_DRV_LOG(ERR, "invalid dst_ip mask.");
		return -EINVAL;
	}

	switch (filter->src_ip_mask) {
	case UINT32_MAX:
		filter_info->src_ip_mask = 0;
		filter_info->src_ip = filter->src_ip;
		break;
	case 0:
		filter_info->src_ip_mask = 1;
		break;
	default:
		PMD_DRV_LOG(ERR, "invalid src_ip mask.");
		return -EINVAL;
	}

	switch (filter->dst_port_mask) {
	case UINT16_MAX:
		filter_info->dst_port_mask = 0;
		filter_info->dst_port = filter->dst_port;
		break;
	case 0:
		filter_info->dst_port_mask = 1;
		break;
	default:
		PMD_DRV_LOG(ERR, "invalid dst_port mask.");
		return -EINVAL;
	}

	switch (filter->src_port_mask) {
	case UINT16_MAX:
		filter_info->src_port_mask = 0;
		filter_info->src_port = filter->src_port;
		break;
	case 0:
		filter_info->src_port_mask = 1;
		break;
	default:
		PMD_DRV_LOG(ERR, "invalid src_port mask.");
		return -EINVAL;
	}

	switch (filter->proto_mask) {
	case UINT8_MAX:
		filter_info->proto_mask = 0;
		filter_info->proto =
			convert_protocol_type(filter->proto);
		break;
	case 0:
		filter_info->proto_mask = 1;
		break;
	default:
		PMD_DRV_LOG(ERR, "invalid protocol mask.");
		return -EINVAL;
	}

	filter_info->priority = (uint8_t)filter->priority;
	return 0;
}

/*
 * add or delete a ntuple filter
 *
 * @param
 * dev: Pointer to struct rte_eth_dev.
 * ntuple_filter: Pointer to struct rte_eth_ntuple_filter
 * add: if true, add filter, if false, remove filter
 *
 * @return
 *    - On success, zero.
 *    - On failure, a negative value.
 */
int
yusur2_add_del_ntuple_filter(struct rte_eth_dev *dev,
			struct rte_eth_ntuple_filter *ntuple_filter,
			bool add)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	struct yusur2_5tuple_filter_info filter_5tuple;
	struct yusur2_5tuple_filter *filter;
	int ret;

	if (ntuple_filter->flags != RTE_5TUPLE_FLAGS) {
		PMD_DRV_LOG(ERR, "only 5tuple is supported.");
		return -EINVAL;
	}

	memset(&filter_5tuple, 0, sizeof(struct yusur2_5tuple_filter_info));
	ret = ntuple_filter_to_5tuple(ntuple_filter, &filter_5tuple);
	if (ret < 0)
		return ret;

	filter = yusur2_5tuple_filter_lookup(&filter_info->fivetuple_list,
					 &filter_5tuple);
	if (filter != NULL && add) {
		PMD_DRV_LOG(ERR, "filter exists.");
		return -EEXIST;
	}
	if (filter == NULL && !add) {
		PMD_DRV_LOG(ERR, "filter doesn't exist.");
		return -ENOENT;
	}

	if (add) {
		filter = rte_zmalloc("yusur2_5tuple_filter",
				sizeof(struct yusur2_5tuple_filter), 0);
		if (filter == NULL)
			return -ENOMEM;
		rte_memcpy(&filter->filter_info,
				 &filter_5tuple,
				 sizeof(struct yusur2_5tuple_filter_info));
		filter->queue = ntuple_filter->queue;
		ret = yusur2_add_5tuple_filter(dev, filter);
		if (ret < 0) {
			rte_free(filter);
			return ret;
		}
	} else
		yusur2_remove_5tuple_filter(dev, filter);

	return 0;
}

/*
 * get a ntuple filter
 *
 * @param
 * dev: Pointer to struct rte_eth_dev.
 * ntuple_filter: Pointer to struct rte_eth_ntuple_filter
 *
 * @return
 *    - On success, zero.
 *    - On failure, a negative value.
 */
static int
yusur2_get_ntuple_filter(struct rte_eth_dev *dev,
			struct rte_eth_ntuple_filter *ntuple_filter)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	struct yusur2_5tuple_filter_info filter_5tuple;
	struct yusur2_5tuple_filter *filter;
	int ret;

	if (ntuple_filter->flags != RTE_5TUPLE_FLAGS) {
		PMD_DRV_LOG(ERR, "only 5tuple is supported.");
		return -EINVAL;
	}

	memset(&filter_5tuple, 0, sizeof(struct yusur2_5tuple_filter_info));
	ret = ntuple_filter_to_5tuple(ntuple_filter, &filter_5tuple);
	if (ret < 0)
		return ret;

	filter = yusur2_5tuple_filter_lookup(&filter_info->fivetuple_list,
					 &filter_5tuple);
	if (filter == NULL) {
		PMD_DRV_LOG(ERR, "filter doesn't exist.");
		return -ENOENT;
	}
	ntuple_filter->queue = filter->queue;
	return 0;
}

/*
 * yusur2_ntuple_filter_handle - Handle operations for ntuple filter.
 * @dev: pointer to rte_eth_dev structure
 * @filter_op:operation will be taken.
 * @arg: a pointer to specific structure corresponding to the filter_op
 *
 * @return
 *    - On success, zero.
 *    - On failure, a negative value.
 */
static int
yusur2_ntuple_filter_handle(struct rte_eth_dev *dev,
				enum rte_filter_op filter_op,
				void *arg)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	MAC_TYPE_FILTER_SUP_EXT(hw->mac.type);

	if (filter_op == RTE_ETH_FILTER_NOP)
		return 0;

	if (arg == NULL) {
		PMD_DRV_LOG(ERR, "arg shouldn't be NULL for operation %u.",
			    filter_op);
		return -EINVAL;
	}

	switch (filter_op) {
	case RTE_ETH_FILTER_ADD:
		ret = yusur2_add_del_ntuple_filter(dev,
			(struct rte_eth_ntuple_filter *)arg,
			TRUE);
		break;
	case RTE_ETH_FILTER_DELETE:
		ret = yusur2_add_del_ntuple_filter(dev,
			(struct rte_eth_ntuple_filter *)arg,
			FALSE);
		break;
	case RTE_ETH_FILTER_GET:
		ret = yusur2_get_ntuple_filter(dev,
			(struct rte_eth_ntuple_filter *)arg);
		break;
	default:
		PMD_DRV_LOG(ERR, "unsupported operation %u.", filter_op);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int
yusur2_add_del_ethertype_filter(struct rte_eth_dev *dev,
			struct rte_eth_ethertype_filter *filter,
			bool add)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	uint32_t etqf = 0;
	uint32_t etqs = 0;
	int ret;
	struct yusur2_ethertype_filter ethertype_filter;

	if (filter->queue >= YUSUR2_MAX_RX_QUEUE_NUM)
		return -EINVAL;

	if (filter->ether_type == RTE_ETHER_TYPE_IPV4 ||
		filter->ether_type == RTE_ETHER_TYPE_IPV6) {
		PMD_DRV_LOG(ERR, "unsupported ether_type(0x%04x) in"
			" ethertype filter.", filter->ether_type);
		return -EINVAL;
	}

	if (filter->flags & RTE_ETHTYPE_FLAGS_MAC) {
		PMD_DRV_LOG(ERR, "mac compare is unsupported.");
		return -EINVAL;
	}
	if (filter->flags & RTE_ETHTYPE_FLAGS_DROP) {
		PMD_DRV_LOG(ERR, "drop option is unsupported.");
		return -EINVAL;
	}

	ret = yusur2_ethertype_filter_lookup(filter_info, filter->ether_type);
	if (ret >= 0 && add) {
		PMD_DRV_LOG(ERR, "ethertype (0x%04x) filter exists.",
			    filter->ether_type);
		return -EEXIST;
	}
	if (ret < 0 && !add) {
		PMD_DRV_LOG(ERR, "ethertype (0x%04x) filter doesn't exist.",
			    filter->ether_type);
		return -ENOENT;
	}

	if (add) {
		etqf = YUSUR2_ETQF_FILTER_EN;
		etqf |= (uint32_t)filter->ether_type;
		etqs |= (uint32_t)((filter->queue <<
				    YUSUR2_ETQS_RX_QUEUE_SHIFT) &
				    YUSUR2_ETQS_RX_QUEUE);
		etqs |= YUSUR2_ETQS_QUEUE_EN;

		ethertype_filter.ethertype = filter->ether_type;
		ethertype_filter.etqf = etqf;
		ethertype_filter.etqs = etqs;
		ethertype_filter.conf = FALSE;
		ret = yusur2_ethertype_filter_insert(filter_info,
						    &ethertype_filter);
		if (ret < 0) {
			PMD_DRV_LOG(ERR, "ethertype filters are full.");
			return -ENOSPC;
		}
	} else {
		ret = yusur2_ethertype_filter_remove(filter_info, (uint8_t)ret);
		if (ret < 0)
			return -ENOSYS;
	}
	YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(ret), etqf);
	YUSUR2_WRITE_REG(hw, YUSUR2_ETQS(ret), etqs);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

static int
yusur2_get_ethertype_filter(struct rte_eth_dev *dev,
			struct rte_eth_ethertype_filter *filter)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	uint32_t etqf, etqs;
	int ret;

	ret = yusur2_ethertype_filter_lookup(filter_info, filter->ether_type);
	if (ret < 0) {
		PMD_DRV_LOG(ERR, "ethertype (0x%04x) filter doesn't exist.",
			    filter->ether_type);
		return -ENOENT;
	}

	etqf = YUSUR2_READ_REG(hw, YUSUR2_ETQF(ret));
	if (etqf & YUSUR2_ETQF_FILTER_EN) {
		etqs = YUSUR2_READ_REG(hw, YUSUR2_ETQS(ret));
		filter->ether_type = etqf & YUSUR2_ETQF_ETHERTYPE;
		filter->flags = 0;
		filter->queue = (etqs & YUSUR2_ETQS_RX_QUEUE) >>
			       YUSUR2_ETQS_RX_QUEUE_SHIFT;
		return 0;
	}
	return -ENOENT;
}

/*
 * yusur2_ethertype_filter_handle - Handle operations for ethertype filter.
 * @dev: pointer to rte_eth_dev structure
 * @filter_op:operation will be taken.
 * @arg: a pointer to specific structure corresponding to the filter_op
 */
static int
yusur2_ethertype_filter_handle(struct rte_eth_dev *dev,
				enum rte_filter_op filter_op,
				void *arg)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	MAC_TYPE_FILTER_SUP(hw->mac.type);

	if (filter_op == RTE_ETH_FILTER_NOP)
		return 0;

	if (arg == NULL) {
		PMD_DRV_LOG(ERR, "arg shouldn't be NULL for operation %u.",
			    filter_op);
		return -EINVAL;
	}

	switch (filter_op) {
	case RTE_ETH_FILTER_ADD:
		ret = yusur2_add_del_ethertype_filter(dev,
			(struct rte_eth_ethertype_filter *)arg,
			TRUE);
		break;
	case RTE_ETH_FILTER_DELETE:
		ret = yusur2_add_del_ethertype_filter(dev,
			(struct rte_eth_ethertype_filter *)arg,
			FALSE);
		break;
	case RTE_ETH_FILTER_GET:
		ret = yusur2_get_ethertype_filter(dev,
			(struct rte_eth_ethertype_filter *)arg);
		break;
	default:
		PMD_DRV_LOG(ERR, "unsupported operation %u.", filter_op);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int
yusur2_dev_filter_ctrl(struct rte_eth_dev *dev,
		     enum rte_filter_type filter_type,
		     enum rte_filter_op filter_op,
		     void *arg)
{
	int ret = 0;

	switch (filter_type) {
	case RTE_ETH_FILTER_NTUPLE:
		ret = yusur2_ntuple_filter_handle(dev, filter_op, arg);
		break;
	case RTE_ETH_FILTER_ETHERTYPE:
		ret = yusur2_ethertype_filter_handle(dev, filter_op, arg);
		break;
	case RTE_ETH_FILTER_SYN:
		ret = yusur2_syn_filter_handle(dev, filter_op, arg);
		break;
	case RTE_ETH_FILTER_FDIR:
		ret = yusur2_fdir_ctrl_func(dev, filter_op, arg);
		break;
	case RTE_ETH_FILTER_L2_TUNNEL:
		ret = yusur2_dev_l2_tunnel_filter_handle(dev, filter_op, arg);
		break;
	case RTE_ETH_FILTER_GENERIC:
		if (filter_op != RTE_ETH_FILTER_GET)
			return -EINVAL;
		*(const void **)arg = &yusur2_flow_ops;
		break;
	default:
		PMD_DRV_LOG(WARNING, "Filter type (%d) not supported",
							filter_type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static u8 *
yusur2_dev_addr_list_itr(__attribute__((unused)) struct yusur2_hw *hw,
			u8 **mc_addr_ptr, u32 *vmdq)
{
	u8 *mc_addr;

	*vmdq = 0;
	mc_addr = *mc_addr_ptr;
	*mc_addr_ptr = (mc_addr + sizeof(struct rte_ether_addr));
	return mc_addr;
}

static int
yusur2_dev_set_mc_addr_list(struct rte_eth_dev *dev,
			  struct rte_ether_addr *mc_addr_set,
			  uint32_t nb_mc_addr)
{
	struct yusur2_hw *hw;
	u8 *mc_addr_list;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mc_addr_list = (u8 *)mc_addr_set;
	return yusur2_update_mc_addr_list(hw, mc_addr_list, nb_mc_addr,
					 yusur2_dev_addr_list_itr, TRUE);
}

static uint64_t
yusur2_read_systime_cyclecounter(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint64_t systime_cycles;

	switch (hw->mac.type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		/* SYSTIMEL stores ns and SYSTIMEH stores seconds. */
		systime_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_SYSTIML);
		systime_cycles += (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_SYSTIMH)
				* NSEC_PER_SEC;
		break;
	default:
		systime_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_SYSTIML);
		systime_cycles |= (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_SYSTIMH)
				<< 32;
	}

	return systime_cycles;
}

static uint64_t
yusur2_read_rx_tstamp_cyclecounter(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint64_t rx_tstamp_cycles;

	switch (hw->mac.type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		/* RXSTMPL stores ns and RXSTMPH stores seconds. */
		rx_tstamp_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_RXSTMPL);
		rx_tstamp_cycles += (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_RXSTMPH)
				* NSEC_PER_SEC;
		break;
	default:
		/* RXSTMPL stores ns and RXSTMPH stores seconds. */
		rx_tstamp_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_RXSTMPL);
		rx_tstamp_cycles |= (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_RXSTMPH)
				<< 32;
	}

	return rx_tstamp_cycles;
}

static uint64_t
yusur2_read_tx_tstamp_cyclecounter(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint64_t tx_tstamp_cycles;

	switch (hw->mac.type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		/* TXSTMPL stores ns and TXSTMPH stores seconds. */
		tx_tstamp_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_TXSTMPL);
		tx_tstamp_cycles += (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_TXSTMPH)
				* NSEC_PER_SEC;
		break;
	default:
		/* TXSTMPL stores ns and TXSTMPH stores seconds. */
		tx_tstamp_cycles = (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_TXSTMPL);
		tx_tstamp_cycles |= (uint64_t)YUSUR2_READ_REG(hw, YUSUR2_TXSTMPH)
				<< 32;
	}

	return tx_tstamp_cycles;
}

static void
yusur2_start_timecounters(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_adapter *adapter = dev->data->dev_private;
	struct rte_eth_link link;
	uint32_t incval = 0;
	uint32_t shift = 0;

	/* Get current link speed. */
	yusur2_dev_link_update(dev, 1);
	rte_eth_linkstatus_get(dev, &link);

	switch (link.link_speed) {
	case ETH_SPEED_NUM_100M:
		incval = YUSUR2_INCVAL_100;
		shift = YUSUR2_INCVAL_SHIFT_100;
		break;
	case ETH_SPEED_NUM_1G:
		incval = YUSUR2_INCVAL_1GB;
		shift = YUSUR2_INCVAL_SHIFT_1GB;
		break;
	case ETH_SPEED_NUM_10G:
	default:
		incval = YUSUR2_INCVAL_10GB;
		shift = YUSUR2_INCVAL_SHIFT_10GB;
		break;
	}

	switch (hw->mac.type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		/* Independent of link speed. */
		incval = 1;
		/* Cycles read will be interpreted as ns. */
		shift = 0;
		/* Fall-through */
	case yusur2_mac_X540:
		YUSUR2_WRITE_REG(hw, YUSUR2_TIMINCA, incval);
		break;
	case yusur2_mac_82599EB:
		incval >>= YUSUR2_INCVAL_SHIFT_82599;
		shift -= YUSUR2_INCVAL_SHIFT_82599;
		YUSUR2_WRITE_REG(hw, YUSUR2_TIMINCA,
				(1 << YUSUR2_INCPER_SHIFT_82599) | incval);
		break;
	default:
		/* Not supported. */
		return;
	}

	memset(&adapter->systime_tc, 0, sizeof(struct rte_timecounter));
	memset(&adapter->rx_tstamp_tc, 0, sizeof(struct rte_timecounter));
	memset(&adapter->tx_tstamp_tc, 0, sizeof(struct rte_timecounter));

	adapter->systime_tc.cc_mask = YUSUR2_CYCLECOUNTER_MASK;
	adapter->systime_tc.cc_shift = shift;
	adapter->systime_tc.nsec_mask = (1ULL << shift) - 1;

	adapter->rx_tstamp_tc.cc_mask = YUSUR2_CYCLECOUNTER_MASK;
	adapter->rx_tstamp_tc.cc_shift = shift;
	adapter->rx_tstamp_tc.nsec_mask = (1ULL << shift) - 1;

	adapter->tx_tstamp_tc.cc_mask = YUSUR2_CYCLECOUNTER_MASK;
	adapter->tx_tstamp_tc.cc_shift = shift;
	adapter->tx_tstamp_tc.nsec_mask = (1ULL << shift) - 1;
}

static int
yusur2_timesync_adjust_time(struct rte_eth_dev *dev, int64_t delta)
{
	struct yusur2_adapter *adapter = dev->data->dev_private;

	adapter->systime_tc.nsec += delta;
	adapter->rx_tstamp_tc.nsec += delta;
	adapter->tx_tstamp_tc.nsec += delta;

	return 0;
}

static int
yusur2_timesync_write_time(struct rte_eth_dev *dev, const struct timespec *ts)
{
	uint64_t ns;
	struct yusur2_adapter *adapter = dev->data->dev_private;

	ns = rte_timespec_to_ns(ts);
	/* Set the timecounters to a new value. */
	adapter->systime_tc.nsec = ns;
	adapter->rx_tstamp_tc.nsec = ns;
	adapter->tx_tstamp_tc.nsec = ns;

	return 0;
}

static int
yusur2_timesync_read_time(struct rte_eth_dev *dev, struct timespec *ts)
{
	uint64_t ns, systime_cycles;
	struct yusur2_adapter *adapter = dev->data->dev_private;

	systime_cycles = yusur2_read_systime_cyclecounter(dev);
	ns = rte_timecounter_update(&adapter->systime_tc, systime_cycles);
	*ts = rte_ns_to_timespec(ns);

	return 0;
}

static int
yusur2_timesync_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t tsync_ctl;
	uint32_t tsauxc;

	/* Stop the timesync system time. */
	YUSUR2_WRITE_REG(hw, YUSUR2_TIMINCA, 0x0);
	/* Reset the timesync system time value. */
	YUSUR2_WRITE_REG(hw, YUSUR2_SYSTIML, 0x0);
	YUSUR2_WRITE_REG(hw, YUSUR2_SYSTIMH, 0x0);

	/* Enable system time for platforms where it isn't on by default. */
	tsauxc = YUSUR2_READ_REG(hw, YUSUR2_TSAUXC);
	tsauxc &= ~YUSUR2_TSAUXC_DISABLE_SYSTIME;
	YUSUR2_WRITE_REG(hw, YUSUR2_TSAUXC, tsauxc);

	yusur2_start_timecounters(dev);

	/* Enable L2 filtering of IEEE1588/802.1AS Ethernet frame types. */
	YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(YUSUR2_ETQF_FILTER_1588),
			(RTE_ETHER_TYPE_1588 |
			 YUSUR2_ETQF_FILTER_EN |
			 YUSUR2_ETQF_1588));

	/* Enable timestamping of received PTP packets. */
	tsync_ctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCRXCTL);
	tsync_ctl |= YUSUR2_TSYNCRXCTL_ENABLED;
	YUSUR2_WRITE_REG(hw, YUSUR2_TSYNCRXCTL, tsync_ctl);

	/* Enable timestamping of transmitted PTP packets. */
	tsync_ctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCTXCTL);
	tsync_ctl |= YUSUR2_TSYNCTXCTL_ENABLED;
	YUSUR2_WRITE_REG(hw, YUSUR2_TSYNCTXCTL, tsync_ctl);

	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

static int
yusur2_timesync_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t tsync_ctl;

	/* Disable timestamping of transmitted PTP packets. */
	tsync_ctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCTXCTL);
	tsync_ctl &= ~YUSUR2_TSYNCTXCTL_ENABLED;
	YUSUR2_WRITE_REG(hw, YUSUR2_TSYNCTXCTL, tsync_ctl);

	/* Disable timestamping of received PTP packets. */
	tsync_ctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCRXCTL);
	tsync_ctl &= ~YUSUR2_TSYNCRXCTL_ENABLED;
	YUSUR2_WRITE_REG(hw, YUSUR2_TSYNCRXCTL, tsync_ctl);

	/* Disable L2 filtering of IEEE1588/802.1AS Ethernet frame types. */
	YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(YUSUR2_ETQF_FILTER_1588), 0);

	/* Stop incrementating the System Time registers. */
	YUSUR2_WRITE_REG(hw, YUSUR2_TIMINCA, 0);

	return 0;
}

static int
yusur2_timesync_read_rx_timestamp(struct rte_eth_dev *dev,
				 struct timespec *timestamp,
				 uint32_t flags __rte_unused)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_adapter *adapter = dev->data->dev_private;
	uint32_t tsync_rxctl;
	uint64_t rx_tstamp_cycles;
	uint64_t ns;

	tsync_rxctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCRXCTL);
	if ((tsync_rxctl & YUSUR2_TSYNCRXCTL_VALID) == 0)
		return -EINVAL;

	rx_tstamp_cycles = yusur2_read_rx_tstamp_cyclecounter(dev);
	ns = rte_timecounter_update(&adapter->rx_tstamp_tc, rx_tstamp_cycles);
	*timestamp = rte_ns_to_timespec(ns);

	return  0;
}

static int
yusur2_timesync_read_tx_timestamp(struct rte_eth_dev *dev,
				 struct timespec *timestamp)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_adapter *adapter = dev->data->dev_private;
	uint32_t tsync_txctl;
	uint64_t tx_tstamp_cycles;
	uint64_t ns;

	tsync_txctl = YUSUR2_READ_REG(hw, YUSUR2_TSYNCTXCTL);
	if ((tsync_txctl & YUSUR2_TSYNCTXCTL_VALID) == 0)
		return -EINVAL;

	tx_tstamp_cycles = yusur2_read_tx_tstamp_cyclecounter(dev);
	ns = rte_timecounter_update(&adapter->tx_tstamp_tc, tx_tstamp_cycles);
	*timestamp = rte_ns_to_timespec(ns);

	return 0;
}

static int
yusur2_get_reg_length(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int count = 0;
	int g_ind = 0;
	const struct reg_info *reg_group;
	const struct reg_info **reg_set = (hw->mac.type == yusur2_mac_82598EB) ?
				    yusur2_regs_mac_82598EB : yusur2_regs_others;

	while ((reg_group = reg_set[g_ind++]))
		count += yusur2_regs_group_count(reg_group);

	return count;
}

static int
yusur2vf_get_reg_length(struct rte_eth_dev *dev __rte_unused)
{
	int count = 0;
	int g_ind = 0;
	const struct reg_info *reg_group;

	while ((reg_group = yusur2vf_regs[g_ind++]))
		count += yusur2_regs_group_count(reg_group);

	return count;
}

static int
yusur2_get_regs(struct rte_eth_dev *dev,
	      struct rte_dev_reg_info *regs)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t *data = regs->data;
	int g_ind = 0;
	int count = 0;
	const struct reg_info *reg_group;
	const struct reg_info **reg_set = (hw->mac.type == yusur2_mac_82598EB) ?
				    yusur2_regs_mac_82598EB : yusur2_regs_others;

	if (data == NULL) {
		regs->length = yusur2_get_reg_length(dev);
		regs->width = sizeof(uint32_t);
		return 0;
	}

	/* Support only full register dump */
	if ((regs->length == 0) ||
	    (regs->length == (uint32_t)yusur2_get_reg_length(dev))) {
		regs->version = hw->mac.type << 24 | hw->revision_id << 16 |
			hw->device_id;
		while ((reg_group = reg_set[g_ind++]))
			count += yusur2_read_regs_group(dev, &data[count],
				reg_group);
		return 0;
	}

	return -ENOTSUP;
}

static int
yusur2vf_get_regs(struct rte_eth_dev *dev,
		struct rte_dev_reg_info *regs)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t *data = regs->data;
	int g_ind = 0;
	int count = 0;
	const struct reg_info *reg_group;

	if (data == NULL) {
		regs->length = yusur2vf_get_reg_length(dev);
		regs->width = sizeof(uint32_t);
		return 0;
	}

	/* Support only full register dump */
	if ((regs->length == 0) ||
	    (regs->length == (uint32_t)yusur2vf_get_reg_length(dev))) {
		regs->version = hw->mac.type << 24 | hw->revision_id << 16 |
			hw->device_id;
		while ((reg_group = yusur2vf_regs[g_ind++]))
			count += yusur2_read_regs_group(dev, &data[count],
						      reg_group);
		return 0;
	}

	return -ENOTSUP;
}

static int
yusur2_get_eeprom_length(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Return unit is byte count */
	return hw->eeprom.word_size * 2;
}

static int
yusur2_get_eeprom(struct rte_eth_dev *dev,
		struct rte_dev_eeprom_info *in_eeprom)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	uint16_t *data = in_eeprom->data;
	int first, length;

	first = in_eeprom->offset >> 1;
	length = in_eeprom->length >> 1;
	if ((first > hw->eeprom.word_size) ||
	    ((first + length) > hw->eeprom.word_size))
		return -EINVAL;

	in_eeprom->magic = hw->vendor_id | (hw->device_id << 16);

	return eeprom->ops.read_buffer(hw, first, length, data);
}

static int
yusur2_set_eeprom(struct rte_eth_dev *dev,
		struct rte_dev_eeprom_info *in_eeprom)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_eeprom_info *eeprom = &hw->eeprom;
	uint16_t *data = in_eeprom->data;
	int first, length;

	first = in_eeprom->offset >> 1;
	length = in_eeprom->length >> 1;
	if ((first > hw->eeprom.word_size) ||
	    ((first + length) > hw->eeprom.word_size))
		return -EINVAL;

	in_eeprom->magic = hw->vendor_id | (hw->device_id << 16);

	return eeprom->ops.write_buffer(hw,  first, length, data);
}

static int
yusur2_get_module_info(struct rte_eth_dev *dev,
		      struct rte_eth_dev_module_info *modinfo)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t status;
	uint8_t sff8472_rev, addr_mode;
	bool page_swap = false;

	/* Check whether we support SFF-8472 or not */
	status = hw->phy.ops.read_i2c_eeprom(hw,
					     YUSUR2_SFF_SFF_8472_COMP,
					     &sff8472_rev);
	if (status != 0)
		return -EIO;

	/* addressing mode is not supported */
	status = hw->phy.ops.read_i2c_eeprom(hw,
					     YUSUR2_SFF_SFF_8472_SWAP,
					     &addr_mode);
	if (status != 0)
		return -EIO;

	if (addr_mode & YUSUR2_SFF_ADDRESSING_MODE) {
		PMD_DRV_LOG(ERR,
			    "Address change required to access page 0xA2, "
			    "but not supported. Please report the module "
			    "type to the driver maintainers.");
		page_swap = true;
	}

	if (sff8472_rev == YUSUR2_SFF_SFF_8472_UNSUP || page_swap) {
		/* We have a SFP, but it does not support SFF-8472 */
		modinfo->type = RTE_ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = RTE_ETH_MODULE_SFF_8079_LEN;
	} else {
		/* We have a SFP which supports a revision of SFF-8472. */
		modinfo->type = RTE_ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = RTE_ETH_MODULE_SFF_8472_LEN;
	}

	return 0;
}

static int
yusur2_get_module_eeprom(struct rte_eth_dev *dev,
			struct rte_dev_eeprom_info *info)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t status = YUSUR2_ERR_PHY_ADDR_INVALID;
	uint8_t databyte = 0xFF;
	uint8_t *data = info->data;
	uint32_t i = 0;

	if (info->length == 0)
		return -EINVAL;

	for (i = info->offset; i < info->offset + info->length; i++) {
		if (i < RTE_ETH_MODULE_SFF_8079_LEN)
			status = hw->phy.ops.read_i2c_eeprom(hw, i, &databyte);
		else
			status = hw->phy.ops.read_i2c_sff8472(hw, i, &databyte);

		if (status != 0)
			return -EIO;

		data[i - info->offset] = databyte;
	}

	return 0;
}

uint16_t
yusur2_reta_size_get(enum yusur2_mac_type mac_type) {
	switch (mac_type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		return ETH_RSS_RETA_SIZE_512;
	case yusur2_mac_X550_vf:
	case yusur2_mac_X550EM_x_vf:
	case yusur2_mac_X550EM_a_vf:
		return ETH_RSS_RETA_SIZE_64;
	case yusur2_mac_X540_vf:
	case yusur2_mac_82599_vf:
		return 0;
	default:
		return ETH_RSS_RETA_SIZE_128;
	}
}

uint32_t
yusur2_reta_reg_get(enum yusur2_mac_type mac_type, uint16_t reta_idx) {
	switch (mac_type) {
	case yusur2_mac_X550:
	case yusur2_mac_X550EM_x:
	case yusur2_mac_X550EM_a:
		if (reta_idx < ETH_RSS_RETA_SIZE_128)
			return YUSUR2_RETA(reta_idx >> 2);
		else
			return YUSUR2_ERETA((reta_idx - ETH_RSS_RETA_SIZE_128) >> 2);
	case yusur2_mac_X550_vf:
	case yusur2_mac_X550EM_x_vf:
	case yusur2_mac_X550EM_a_vf:
		return YUSUR2_VFRETA(reta_idx >> 2);
	default:
		return YUSUR2_RETA(reta_idx >> 2);
	}
}

uint32_t
yusur2_mrqc_reg_get(enum yusur2_mac_type mac_type) {
	switch (mac_type) {
	case yusur2_mac_X550_vf:
	case yusur2_mac_X550EM_x_vf:
	case yusur2_mac_X550EM_a_vf:
		return YUSUR2_VFMRQC;
	default:
		return YUSUR2_MRQC;
	}
}

uint32_t
yusur2_rssrk_reg_get(enum yusur2_mac_type mac_type, uint8_t i) {
	switch (mac_type) {
	case yusur2_mac_X550_vf:
	case yusur2_mac_X550EM_x_vf:
	case yusur2_mac_X550EM_a_vf:
		return YUSUR2_VFRSSRK(i);
	default:
		return YUSUR2_RSSRK(i);
	}
}

bool
yusur2_rss_update_sp(enum yusur2_mac_type mac_type) {
	switch (mac_type) {
	case yusur2_mac_82599_vf:
	case yusur2_mac_X540_vf:
		return 0;
	default:
		return 1;
	}
}

static int
yusur2_dev_get_dcb_info(struct rte_eth_dev *dev,
			struct rte_eth_dcb_info *dcb_info)
{
	struct yusur2_dcb_config *dcb_config =
			YUSUR2_DEV_PRIVATE_TO_DCB_CFG(dev->data->dev_private);
	struct yusur2_dcb_tc_config *tc;
	struct rte_eth_dcb_tc_queue_mapping *tc_queue;
	uint8_t nb_tcs;
	uint8_t i, j;

	if (dev->data->dev_conf.rxmode.mq_mode & ETH_MQ_RX_DCB_FLAG)
		dcb_info->nb_tcs = dcb_config->num_tcs.pg_tcs;
	else
		dcb_info->nb_tcs = 1;

	tc_queue = &dcb_info->tc_queue;
	nb_tcs = dcb_info->nb_tcs;

	if (dcb_config->vt_mode) { /* vt is enabled*/
		struct rte_eth_vmdq_dcb_conf *vmdq_rx_conf =
				&dev->data->dev_conf.rx_adv_conf.vmdq_dcb_conf;
		for (i = 0; i < ETH_DCB_NUM_USER_PRIORITIES; i++)
			dcb_info->prio_tc[i] = vmdq_rx_conf->dcb_tc[i];
		if (RTE_ETH_DEV_SRIOV(dev).active > 0) {
			for (j = 0; j < nb_tcs; j++) {
				tc_queue->tc_rxq[0][j].base = j;
				tc_queue->tc_rxq[0][j].nb_queue = 1;
				tc_queue->tc_txq[0][j].base = j;
				tc_queue->tc_txq[0][j].nb_queue = 1;
			}
		} else {
			for (i = 0; i < vmdq_rx_conf->nb_queue_pools; i++) {
				for (j = 0; j < nb_tcs; j++) {
					tc_queue->tc_rxq[i][j].base =
						i * nb_tcs + j;
					tc_queue->tc_rxq[i][j].nb_queue = 1;
					tc_queue->tc_txq[i][j].base =
						i * nb_tcs + j;
					tc_queue->tc_txq[i][j].nb_queue = 1;
				}
			}
		}
	} else { /* vt is disabled*/
		struct rte_eth_dcb_rx_conf *rx_conf =
				&dev->data->dev_conf.rx_adv_conf.dcb_rx_conf;
		for (i = 0; i < ETH_DCB_NUM_USER_PRIORITIES; i++)
			dcb_info->prio_tc[i] = rx_conf->dcb_tc[i];
		if (dcb_info->nb_tcs == ETH_4_TCS) {
			for (i = 0; i < dcb_info->nb_tcs; i++) {
				dcb_info->tc_queue.tc_rxq[0][i].base = i * 32;
				dcb_info->tc_queue.tc_rxq[0][i].nb_queue = 16;
			}
			dcb_info->tc_queue.tc_txq[0][0].base = 0;
			dcb_info->tc_queue.tc_txq[0][1].base = 64;
			dcb_info->tc_queue.tc_txq[0][2].base = 96;
			dcb_info->tc_queue.tc_txq[0][3].base = 112;
			dcb_info->tc_queue.tc_txq[0][0].nb_queue = 64;
			dcb_info->tc_queue.tc_txq[0][1].nb_queue = 32;
			dcb_info->tc_queue.tc_txq[0][2].nb_queue = 16;
			dcb_info->tc_queue.tc_txq[0][3].nb_queue = 16;
		} else if (dcb_info->nb_tcs == ETH_8_TCS) {
			for (i = 0; i < dcb_info->nb_tcs; i++) {
				dcb_info->tc_queue.tc_rxq[0][i].base = i * 16;
				dcb_info->tc_queue.tc_rxq[0][i].nb_queue = 16;
			}
			dcb_info->tc_queue.tc_txq[0][0].base = 0;
			dcb_info->tc_queue.tc_txq[0][1].base = 32;
			dcb_info->tc_queue.tc_txq[0][2].base = 64;
			dcb_info->tc_queue.tc_txq[0][3].base = 80;
			dcb_info->tc_queue.tc_txq[0][4].base = 96;
			dcb_info->tc_queue.tc_txq[0][5].base = 104;
			dcb_info->tc_queue.tc_txq[0][6].base = 112;
			dcb_info->tc_queue.tc_txq[0][7].base = 120;
			dcb_info->tc_queue.tc_txq[0][0].nb_queue = 32;
			dcb_info->tc_queue.tc_txq[0][1].nb_queue = 32;
			dcb_info->tc_queue.tc_txq[0][2].nb_queue = 16;
			dcb_info->tc_queue.tc_txq[0][3].nb_queue = 16;
			dcb_info->tc_queue.tc_txq[0][4].nb_queue = 8;
			dcb_info->tc_queue.tc_txq[0][5].nb_queue = 8;
			dcb_info->tc_queue.tc_txq[0][6].nb_queue = 8;
			dcb_info->tc_queue.tc_txq[0][7].nb_queue = 8;
		}
	}
	for (i = 0; i < dcb_info->nb_tcs; i++) {
		tc = &dcb_config->tc_config[i];
		dcb_info->tc_bws[i] = tc->path[YUSUR2_DCB_TX_CONFIG].bwg_percent;
	}
	return 0;
}

/* Update e-tag ether type */
static int
yusur2_update_e_tag_eth_type(struct yusur2_hw *hw,
			    uint16_t ether_type)
{
	uint32_t etag_etype;

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	etag_etype = YUSUR2_READ_REG(hw, YUSUR2_ETAG_ETYPE);
	etag_etype &= ~YUSUR2_ETAG_ETYPE_MASK;
	etag_etype |= ether_type;
	YUSUR2_WRITE_REG(hw, YUSUR2_ETAG_ETYPE, etag_etype);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

/* Config l2 tunnel ether type */
static int
yusur2_dev_l2_tunnel_eth_type_conf(struct rte_eth_dev *dev,
				  struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);

	if (l2_tunnel == NULL)
		return -EINVAL;

	switch (l2_tunnel->l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		l2_tn_info->e_tag_ether_type = l2_tunnel->ether_type;
		ret = yusur2_update_e_tag_eth_type(hw, l2_tunnel->ether_type);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Enable e-tag tunnel */
static int
yusur2_e_tag_enable(struct yusur2_hw *hw)
{
	uint32_t etag_etype;

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	etag_etype = YUSUR2_READ_REG(hw, YUSUR2_ETAG_ETYPE);
	etag_etype |= YUSUR2_ETAG_ETYPE_VALID;
	YUSUR2_WRITE_REG(hw, YUSUR2_ETAG_ETYPE, etag_etype);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

/* Enable l2 tunnel */
static int
yusur2_dev_l2_tunnel_enable(struct rte_eth_dev *dev,
			   enum rte_eth_tunnel_type l2_tunnel_type)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		l2_tn_info->e_tag_en = TRUE;
		ret = yusur2_e_tag_enable(hw);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Disable e-tag tunnel */
static int
yusur2_e_tag_disable(struct yusur2_hw *hw)
{
	uint32_t etag_etype;

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	etag_etype = YUSUR2_READ_REG(hw, YUSUR2_ETAG_ETYPE);
	etag_etype &= ~YUSUR2_ETAG_ETYPE_VALID;
	YUSUR2_WRITE_REG(hw, YUSUR2_ETAG_ETYPE, etag_etype);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

/* Disable l2 tunnel */
static int
yusur2_dev_l2_tunnel_disable(struct rte_eth_dev *dev,
			    enum rte_eth_tunnel_type l2_tunnel_type)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		l2_tn_info->e_tag_en = FALSE;
		ret = yusur2_e_tag_disable(hw);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
yusur2_e_tag_filter_del(struct rte_eth_dev *dev,
		       struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t i, rar_entries;
	uint32_t rar_low, rar_high;

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	rar_entries = yusur2_get_num_rx_addrs(hw);

	for (i = 1; i < rar_entries; i++) {
		rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(i));
		rar_low  = YUSUR2_READ_REG(hw, YUSUR2_RAL(i));
		if ((rar_high & YUSUR2_RAH_AV) &&
		    (rar_high & YUSUR2_RAH_ADTYPE) &&
		    ((rar_low & YUSUR2_RAL_ETAG_FILTER_MASK) ==
		     l2_tunnel->tunnel_id)) {
			YUSUR2_WRITE_REG(hw, YUSUR2_RAL(i), 0);
			YUSUR2_WRITE_REG(hw, YUSUR2_RAH(i), 0);

			yusur2_clear_vmdq(hw, i, YUSUR2_CLEAR_VMDQ_ALL);

			return ret;
		}
	}

	return ret;
}

static int
yusur2_e_tag_filter_add(struct rte_eth_dev *dev,
		       struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t i, rar_entries;
	uint32_t rar_low, rar_high;

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	/* One entry for one tunnel. Try to remove potential existing entry. */
	yusur2_e_tag_filter_del(dev, l2_tunnel);

	rar_entries = yusur2_get_num_rx_addrs(hw);

	for (i = 1; i < rar_entries; i++) {
		rar_high = YUSUR2_READ_REG(hw, YUSUR2_RAH(i));
		if (rar_high & YUSUR2_RAH_AV) {
			continue;
		} else {
			yusur2_set_vmdq(hw, i, l2_tunnel->pool);
			rar_high = YUSUR2_RAH_AV | YUSUR2_RAH_ADTYPE;
			rar_low = l2_tunnel->tunnel_id;

			YUSUR2_WRITE_REG(hw, YUSUR2_RAL(i), rar_low);
			YUSUR2_WRITE_REG(hw, YUSUR2_RAH(i), rar_high);

			return ret;
		}
	}

	PMD_INIT_LOG(NOTICE, "The table of E-tag forwarding rule is full."
		     " Please remove a rule before adding a new one.");
	return -EINVAL;
}

static inline struct yusur2_l2_tn_filter *
yusur2_l2_tn_filter_lookup(struct yusur2_l2_tn_info *l2_tn_info,
			  struct yusur2_l2_tn_key *key)
{
	int ret;

	ret = rte_hash_lookup(l2_tn_info->hash_handle, (const void *)key);
	if (ret < 0)
		return NULL;

	return l2_tn_info->hash_map[ret];
}

static inline int
yusur2_insert_l2_tn_filter(struct yusur2_l2_tn_info *l2_tn_info,
			  struct yusur2_l2_tn_filter *l2_tn_filter)
{
	int ret;

	ret = rte_hash_add_key(l2_tn_info->hash_handle,
			       &l2_tn_filter->key);

	if (ret < 0) {
		PMD_DRV_LOG(ERR,
			    "Failed to insert L2 tunnel filter"
			    " to hash table %d!",
			    ret);
		return ret;
	}

	l2_tn_info->hash_map[ret] = l2_tn_filter;

	TAILQ_INSERT_TAIL(&l2_tn_info->l2_tn_list, l2_tn_filter, entries);

	return 0;
}

static inline int
yusur2_remove_l2_tn_filter(struct yusur2_l2_tn_info *l2_tn_info,
			  struct yusur2_l2_tn_key *key)
{
	int ret;
	struct yusur2_l2_tn_filter *l2_tn_filter;

	ret = rte_hash_del_key(l2_tn_info->hash_handle, key);

	if (ret < 0) {
		PMD_DRV_LOG(ERR,
			    "No such L2 tunnel filter to delete %d!",
			    ret);
		return ret;
	}

	l2_tn_filter = l2_tn_info->hash_map[ret];
	l2_tn_info->hash_map[ret] = NULL;

	TAILQ_REMOVE(&l2_tn_info->l2_tn_list, l2_tn_filter, entries);
	rte_free(l2_tn_filter);

	return 0;
}

/* Add l2 tunnel filter */
int
yusur2_dev_l2_tunnel_filter_add(struct rte_eth_dev *dev,
			       struct rte_eth_l2_tunnel_conf *l2_tunnel,
			       bool restore)
{
	int ret;
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	struct yusur2_l2_tn_key key;
	struct yusur2_l2_tn_filter *node;

	if (!restore) {
		key.l2_tn_type = l2_tunnel->l2_tunnel_type;
		key.tn_id = l2_tunnel->tunnel_id;

		node = yusur2_l2_tn_filter_lookup(l2_tn_info, &key);

		if (node) {
			PMD_DRV_LOG(ERR,
				    "The L2 tunnel filter already exists!");
			return -EINVAL;
		}

		node = rte_zmalloc("yusur2_l2_tn",
				   sizeof(struct yusur2_l2_tn_filter),
				   0);
		if (!node)
			return -ENOMEM;

		rte_memcpy(&node->key,
				 &key,
				 sizeof(struct yusur2_l2_tn_key));
		node->pool = l2_tunnel->pool;
		ret = yusur2_insert_l2_tn_filter(l2_tn_info, node);
		if (ret < 0) {
			rte_free(node);
			return ret;
		}
	}

	switch (l2_tunnel->l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_filter_add(dev, l2_tunnel);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	if ((!restore) && (ret < 0))
		(void)yusur2_remove_l2_tn_filter(l2_tn_info, &key);

	return ret;
}

/* Delete l2 tunnel filter */
int
yusur2_dev_l2_tunnel_filter_del(struct rte_eth_dev *dev,
			       struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret;
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	struct yusur2_l2_tn_key key;

	key.l2_tn_type = l2_tunnel->l2_tunnel_type;
	key.tn_id = l2_tunnel->tunnel_id;
	ret = yusur2_remove_l2_tn_filter(l2_tn_info, &key);
	if (ret < 0)
		return ret;

	switch (l2_tunnel->l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_filter_del(dev, l2_tunnel);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * yusur2_dev_l2_tunnel_filter_handle - Handle operations for l2 tunnel filter.
 * @dev: pointer to rte_eth_dev structure
 * @filter_op:operation will be taken.
 * @arg: a pointer to specific structure corresponding to the filter_op
 */
static int
yusur2_dev_l2_tunnel_filter_handle(struct rte_eth_dev *dev,
				  enum rte_filter_op filter_op,
				  void *arg)
{
	int ret;

	if (filter_op == RTE_ETH_FILTER_NOP)
		return 0;

	if (arg == NULL) {
		PMD_DRV_LOG(ERR, "arg shouldn't be NULL for operation %u.",
			    filter_op);
		return -EINVAL;
	}

	switch (filter_op) {
	case RTE_ETH_FILTER_ADD:
		ret = yusur2_dev_l2_tunnel_filter_add
			(dev,
			 (struct rte_eth_l2_tunnel_conf *)arg,
			 FALSE);
		break;
	case RTE_ETH_FILTER_DELETE:
		ret = yusur2_dev_l2_tunnel_filter_del
			(dev,
			 (struct rte_eth_l2_tunnel_conf *)arg);
		break;
	default:
		PMD_DRV_LOG(ERR, "unsupported operation %u.", filter_op);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int
yusur2_e_tag_forwarding_en_dis(struct rte_eth_dev *dev, bool en)
{
	int ret = 0;
	uint32_t ctrl;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_VT_CTL);
	ctrl &= ~YUSUR2_VT_CTL_POOLING_MODE_MASK;
	if (en)
		ctrl |= YUSUR2_VT_CTL_POOLING_MODE_ETAG;
	YUSUR2_WRITE_REG(hw, YUSUR2_VT_CTL, ctrl);

	return ret;
}

/* Enable l2 tunnel forwarding */
static int
yusur2_dev_l2_tunnel_forwarding_enable
	(struct rte_eth_dev *dev,
	 enum rte_eth_tunnel_type l2_tunnel_type)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	int ret = 0;

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		l2_tn_info->e_tag_fwd_en = TRUE;
		ret = yusur2_e_tag_forwarding_en_dis(dev, 1);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Disable l2 tunnel forwarding */
static int
yusur2_dev_l2_tunnel_forwarding_disable
	(struct rte_eth_dev *dev,
	 enum rte_eth_tunnel_type l2_tunnel_type)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	int ret = 0;

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		l2_tn_info->e_tag_fwd_en = FALSE;
		ret = yusur2_e_tag_forwarding_en_dis(dev, 0);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
yusur2_e_tag_insertion_en_dis(struct rte_eth_dev *dev,
			     struct rte_eth_l2_tunnel_conf *l2_tunnel,
			     bool en)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	int ret = 0;
	uint32_t vmtir, vmvir;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (l2_tunnel->vf_id >= pci_dev->max_vfs) {
		PMD_DRV_LOG(ERR,
			    "VF id %u should be less than %u",
			    l2_tunnel->vf_id,
			    pci_dev->max_vfs);
		return -EINVAL;
	}

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	if (en)
		vmtir = l2_tunnel->tunnel_id;
	else
		vmtir = 0;

	YUSUR2_WRITE_REG(hw, YUSUR2_VMTIR(l2_tunnel->vf_id), vmtir);

	vmvir = YUSUR2_READ_REG(hw, YUSUR2_VMVIR(l2_tunnel->vf_id));
	vmvir &= ~YUSUR2_VMVIR_TAGA_MASK;
	if (en)
		vmvir |= YUSUR2_VMVIR_TAGA_ETAG_INSERT;
	YUSUR2_WRITE_REG(hw, YUSUR2_VMVIR(l2_tunnel->vf_id), vmvir);

	return ret;
}

/* Enable l2 tunnel tag insertion */
static int
yusur2_dev_l2_tunnel_insertion_enable(struct rte_eth_dev *dev,
				     struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret = 0;

	switch (l2_tunnel->l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_insertion_en_dis(dev, l2_tunnel, 1);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Disable l2 tunnel tag insertion */
static int
yusur2_dev_l2_tunnel_insertion_disable
	(struct rte_eth_dev *dev,
	 struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	int ret = 0;

	switch (l2_tunnel->l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_insertion_en_dis(dev, l2_tunnel, 0);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
yusur2_e_tag_stripping_en_dis(struct rte_eth_dev *dev,
			     bool en)
{
	int ret = 0;
	uint32_t qde;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	qde = YUSUR2_READ_REG(hw, YUSUR2_QDE);
	if (en)
		qde |= YUSUR2_QDE_STRIP_TAG;
	else
		qde &= ~YUSUR2_QDE_STRIP_TAG;
	qde &= ~YUSUR2_QDE_READ;
	qde |= YUSUR2_QDE_WRITE;
	YUSUR2_WRITE_REG(hw, YUSUR2_QDE, qde);

	return ret;
}

/* Enable l2 tunnel tag stripping */
static int
yusur2_dev_l2_tunnel_stripping_enable
	(struct rte_eth_dev *dev,
	 enum rte_eth_tunnel_type l2_tunnel_type)
{
	int ret = 0;

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_stripping_en_dis(dev, 1);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Disable l2 tunnel tag stripping */
static int
yusur2_dev_l2_tunnel_stripping_disable
	(struct rte_eth_dev *dev,
	 enum rte_eth_tunnel_type l2_tunnel_type)
{
	int ret = 0;

	switch (l2_tunnel_type) {
	case RTE_L2_TUNNEL_TYPE_E_TAG:
		ret = yusur2_e_tag_stripping_en_dis(dev, 0);
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Enable/disable l2 tunnel offload functions */
static int
yusur2_dev_l2_tunnel_offload_set
	(struct rte_eth_dev *dev,
	 struct rte_eth_l2_tunnel_conf *l2_tunnel,
	 uint32_t mask,
	 uint8_t en)
{
	int ret = 0;

	if (l2_tunnel == NULL)
		return -EINVAL;

	ret = -EINVAL;
	if (mask & ETH_L2_TUNNEL_ENABLE_MASK) {
		if (en)
			ret = yusur2_dev_l2_tunnel_enable(
				dev,
				l2_tunnel->l2_tunnel_type);
		else
			ret = yusur2_dev_l2_tunnel_disable(
				dev,
				l2_tunnel->l2_tunnel_type);
	}

	if (mask & ETH_L2_TUNNEL_INSERTION_MASK) {
		if (en)
			ret = yusur2_dev_l2_tunnel_insertion_enable(
				dev,
				l2_tunnel);
		else
			ret = yusur2_dev_l2_tunnel_insertion_disable(
				dev,
				l2_tunnel);
	}

	if (mask & ETH_L2_TUNNEL_STRIPPING_MASK) {
		if (en)
			ret = yusur2_dev_l2_tunnel_stripping_enable(
				dev,
				l2_tunnel->l2_tunnel_type);
		else
			ret = yusur2_dev_l2_tunnel_stripping_disable(
				dev,
				l2_tunnel->l2_tunnel_type);
	}

	if (mask & ETH_L2_TUNNEL_FORWARDING_MASK) {
		if (en)
			ret = yusur2_dev_l2_tunnel_forwarding_enable(
				dev,
				l2_tunnel->l2_tunnel_type);
		else
			ret = yusur2_dev_l2_tunnel_forwarding_disable(
				dev,
				l2_tunnel->l2_tunnel_type);
	}

	return ret;
}

static int
yusur2_update_vxlan_port(struct yusur2_hw *hw,
			uint16_t port)
{
	YUSUR2_WRITE_REG(hw, YUSUR2_VXLANCTRL, port);
	YUSUR2_WRITE_FLUSH(hw);

	return 0;
}

/* There's only one register for VxLAN UDP port.
 * So, we cannot add several ports. Will update it.
 */
static int
yusur2_add_vxlan_port(struct yusur2_hw *hw,
		     uint16_t port)
{
	if (port == 0) {
		PMD_DRV_LOG(ERR, "Add VxLAN port 0 is not allowed.");
		return -EINVAL;
	}

	return yusur2_update_vxlan_port(hw, port);
}

/* We cannot delete the VxLAN port. For there's a register for VxLAN
 * UDP port, it must have a value.
 * So, will reset it to the original value 0.
 */
static int
yusur2_del_vxlan_port(struct yusur2_hw *hw,
		     uint16_t port)
{
	uint16_t cur_port;

	cur_port = (uint16_t)YUSUR2_READ_REG(hw, YUSUR2_VXLANCTRL);

	if (cur_port != port) {
		PMD_DRV_LOG(ERR, "Port %u does not exist.", port);
		return -EINVAL;
	}

	return yusur2_update_vxlan_port(hw, 0);
}

/* Add UDP tunneling port */
static int
yusur2_dev_udp_tunnel_port_add(struct rte_eth_dev *dev,
			      struct rte_eth_udp_tunnel *udp_tunnel)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	if (udp_tunnel == NULL)
		return -EINVAL;

	switch (udp_tunnel->prot_type) {
	case RTE_TUNNEL_TYPE_VXLAN:
		ret = yusur2_add_vxlan_port(hw, udp_tunnel->udp_port);
		break;

	case RTE_TUNNEL_TYPE_GENEVE:
	case RTE_TUNNEL_TYPE_TEREDO:
		PMD_DRV_LOG(ERR, "Tunnel type is not supported now.");
		ret = -EINVAL;
		break;

	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Remove UDP tunneling port */
static int
yusur2_dev_udp_tunnel_port_del(struct rte_eth_dev *dev,
			      struct rte_eth_udp_tunnel *udp_tunnel)
{
	int ret = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (hw->mac.type != yusur2_mac_X550 &&
	    hw->mac.type != yusur2_mac_X550EM_x &&
	    hw->mac.type != yusur2_mac_X550EM_a) {
		return -ENOTSUP;
	}

	if (udp_tunnel == NULL)
		return -EINVAL;

	switch (udp_tunnel->prot_type) {
	case RTE_TUNNEL_TYPE_VXLAN:
		ret = yusur2_del_vxlan_port(hw, udp_tunnel->udp_port);
		break;
	case RTE_TUNNEL_TYPE_GENEVE:
	case RTE_TUNNEL_TYPE_TEREDO:
		PMD_DRV_LOG(ERR, "Tunnel type is not supported now.");
		ret = -EINVAL;
		break;
	default:
		PMD_DRV_LOG(ERR, "Invalid tunnel type");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
yusur2vf_dev_promiscuous_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	switch (hw->mac.ops.update_xcast_mode(hw, YUSUR2VF_XCAST_MODE_PROMISC)) {
	case YUSUR2_SUCCESS:
		ret = 0;
		break;
	case YUSUR2_ERR_FEATURE_NOT_SUPPORTED:
		ret = -ENOTSUP;
		break;
	default:
		ret = -EAGAIN;
		break;
	}

	return ret;
}

static int
yusur2vf_dev_promiscuous_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	switch (hw->mac.ops.update_xcast_mode(hw, YUSUR2VF_XCAST_MODE_NONE)) {
	case YUSUR2_SUCCESS:
		ret = 0;
		break;
	case YUSUR2_ERR_FEATURE_NOT_SUPPORTED:
		ret = -ENOTSUP;
		break;
	default:
		ret = -EAGAIN;
		break;
	}

	return ret;
}

static int
yusur2vf_dev_allmulticast_enable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;
	int mode = YUSUR2VF_XCAST_MODE_ALLMULTI;

	switch (hw->mac.ops.update_xcast_mode(hw, mode)) {
	case YUSUR2_SUCCESS:
		ret = 0;
		break;
	case YUSUR2_ERR_FEATURE_NOT_SUPPORTED:
		ret = -ENOTSUP;
		break;
	default:
		ret = -EAGAIN;
		break;
	}

	return ret;
}

static int
yusur2vf_dev_allmulticast_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret;

	switch (hw->mac.ops.update_xcast_mode(hw, YUSUR2VF_XCAST_MODE_MULTI)) {
	case YUSUR2_SUCCESS:
		ret = 0;
		break;
	case YUSUR2_ERR_FEATURE_NOT_SUPPORTED:
		ret = -ENOTSUP;
		break;
	default:
		ret = -EAGAIN;
		break;
	}

	return ret;
}

static void yusur2vf_mbx_process(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	u32 in_msg = 0;

	/* peek the message first */
	in_msg = YUSUR2_READ_REG(hw, YUSUR2_VFMBMEM);

	/* PF reset VF event */
	if (in_msg == YUSUR2_PF_CONTROL_MSG) {
		/* dummy mbx read to ack pf */
		if (yusur2_read_mbx(hw, &in_msg, 1, 0))
			return;
		_rte_eth_dev_callback_process(dev, RTE_ETH_EVENT_INTR_RESET,
					      NULL);
	}
}

static int
yusur2vf_dev_interrupt_get_status(struct rte_eth_dev *dev)
{
	uint32_t eicr;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	yusur2vf_intr_disable(dev);

	/* read-on-clear nic registers here */
	eicr = YUSUR2_READ_REG(hw, YUSUR2_VTEICR);
	intr->flags = 0;

	/* only one misc vector supported - mailbox */
	eicr &= YUSUR2_VTEICR_MASK;
	if (eicr == YUSUR2_MISC_VEC_ID)
		intr->flags |= YUSUR2_FLAG_MAILBOX;

	return 0;
}

static int
yusur2vf_dev_interrupt_action(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	if (intr->flags & YUSUR2_FLAG_MAILBOX) {
		yusur2vf_mbx_process(dev);
		intr->flags &= ~YUSUR2_FLAG_MAILBOX;
	}

	yusur2vf_intr_enable(dev);

	return 0;
}

static void
yusur2vf_dev_interrupt_handler(void *param)
{
	struct rte_eth_dev *dev = (struct rte_eth_dev *)param;

	yusur2vf_dev_interrupt_get_status(dev);
	yusur2vf_dev_interrupt_action(dev);
}

/**
 *  yusur2_disable_sec_tx_path_generic - Stops the transmit data path
 *  @hw: pointer to hardware structure
 *
 *  Stops the transmit data path and waits for the HW to internally empty
 *  the Tx security block
 **/
int yusur2_disable_sec_tx_path_generic(struct yusur2_hw *hw)
{
#define YUSUR2_MAX_SECTX_POLL 40

	int i;
	int sectxreg;

	sectxreg = YUSUR2_READ_REG(hw, YUSUR2_SECTXCTRL);
	sectxreg |= YUSUR2_SECTXCTRL_TX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXCTRL, sectxreg);
	for (i = 0; i < YUSUR2_MAX_SECTX_POLL; i++) {
		sectxreg = YUSUR2_READ_REG(hw, YUSUR2_SECTXSTAT);
		if (sectxreg & YUSUR2_SECTXSTAT_SECTX_RDY)
			break;
		/* Use interrupt-safe sleep just in case */
		usec_delay(1000);
	}

	/* For informational purposes only */
	if (i >= YUSUR2_MAX_SECTX_POLL)
		PMD_DRV_LOG(DEBUG, "Tx unit being enabled before security "
			 "path fully disabled.  Continuing with init.");

	return YUSUR2_SUCCESS;
}

/**
 *  yusur2_enable_sec_tx_path_generic - Enables the transmit data path
 *  @hw: pointer to hardware structure
 *
 *  Enables the transmit data path.
 **/
int yusur2_enable_sec_tx_path_generic(struct yusur2_hw *hw)
{
	uint32_t sectxreg;

	sectxreg = YUSUR2_READ_REG(hw, YUSUR2_SECTXCTRL);
	sectxreg &= ~YUSUR2_SECTXCTRL_TX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXCTRL, sectxreg);
	YUSUR2_WRITE_FLUSH(hw);

	return YUSUR2_SUCCESS;
}

/* restore n-tuple filter */
static inline void
yusur2_ntuple_filter_restore(struct rte_eth_dev *dev)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	struct yusur2_5tuple_filter *node;

	TAILQ_FOREACH(node, &filter_info->fivetuple_list, entries) {
		yusur2_inject_5tuple_filter(dev, node);
	}
}

/* restore ethernet type filter */
static inline void
yusur2_ethertype_filter_restore(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	int i;

	for (i = 0; i < YUSUR2_MAX_ETQF_FILTERS; i++) {
		if (filter_info->ethertype_mask & (1 << i)) {
			YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(i),
					filter_info->ethertype_filters[i].etqf);
			YUSUR2_WRITE_REG(hw, YUSUR2_ETQS(i),
					filter_info->ethertype_filters[i].etqs);
			YUSUR2_WRITE_FLUSH(hw);
		}
	}
}

/* restore SYN filter */
static inline void
yusur2_syn_filter_restore(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	uint32_t synqf;

	synqf = filter_info->syn_info;

	if (synqf & YUSUR2_SYN_FILTER_ENABLE) {
		YUSUR2_WRITE_REG(hw, YUSUR2_SYNQF, synqf);
		YUSUR2_WRITE_FLUSH(hw);
	}
}

/* restore L2 tunnel filter */
static inline void
yusur2_l2_tn_filter_restore(struct rte_eth_dev *dev)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	struct yusur2_l2_tn_filter *node;
	struct rte_eth_l2_tunnel_conf l2_tn_conf;

	TAILQ_FOREACH(node, &l2_tn_info->l2_tn_list, entries) {
		l2_tn_conf.l2_tunnel_type = node->key.l2_tn_type;
		l2_tn_conf.tunnel_id      = node->key.tn_id;
		l2_tn_conf.pool           = node->pool;
		(void)yusur2_dev_l2_tunnel_filter_add(dev, &l2_tn_conf, TRUE);
	}
}

/* restore rss filter */
static inline void
yusur2_rss_filter_restore(struct rte_eth_dev *dev)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);

	if (filter_info->rss_info.conf.queue_num)
		yusur2_config_rss_filter(dev,
			&filter_info->rss_info, TRUE);
}

static int
yusur2_filter_restore(struct rte_eth_dev *dev)
{
	yusur2_ntuple_filter_restore(dev);
	yusur2_ethertype_filter_restore(dev);
	yusur2_syn_filter_restore(dev);
	yusur2_fdir_filter_restore(dev);
	yusur2_l2_tn_filter_restore(dev);
	yusur2_rss_filter_restore(dev);

	return 0;
}

static void
yusur2_l2_tunnel_conf(struct rte_eth_dev *dev)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (l2_tn_info->e_tag_en)
		(void)yusur2_e_tag_enable(hw);

	if (l2_tn_info->e_tag_fwd_en)
		(void)yusur2_e_tag_forwarding_en_dis(dev, 1);

	(void)yusur2_update_e_tag_eth_type(hw, l2_tn_info->e_tag_ether_type);
}

/* remove all the n-tuple filters */
void
yusur2_clear_all_ntuple_filter(struct rte_eth_dev *dev)
{
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	struct yusur2_5tuple_filter *p_5tuple;

	while ((p_5tuple = TAILQ_FIRST(&filter_info->fivetuple_list)))
		yusur2_remove_5tuple_filter(dev, p_5tuple);
}

/* remove all the ether type filters */
void
yusur2_clear_all_ethertype_filter(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	int i;

	for (i = 0; i < YUSUR2_MAX_ETQF_FILTERS; i++) {
		if (filter_info->ethertype_mask & (1 << i) &&
		    !filter_info->ethertype_filters[i].conf) {
			(void)yusur2_ethertype_filter_remove(filter_info,
							    (uint8_t)i);
			YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(i), 0);
			YUSUR2_WRITE_REG(hw, YUSUR2_ETQS(i), 0);
			YUSUR2_WRITE_FLUSH(hw);
		}
	}
}

/* remove the SYN filter */
void
yusur2_clear_syn_filter(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);

	if (filter_info->syn_info & YUSUR2_SYN_FILTER_ENABLE) {
		filter_info->syn_info = 0;

		YUSUR2_WRITE_REG(hw, YUSUR2_SYNQF, 0);
		YUSUR2_WRITE_FLUSH(hw);
	}
}

/* remove all the L2 tunnel filters */
int
yusur2_clear_all_l2_tn_filter(struct rte_eth_dev *dev)
{
	struct yusur2_l2_tn_info *l2_tn_info =
		YUSUR2_DEV_PRIVATE_TO_L2_TN_INFO(dev->data->dev_private);
	struct yusur2_l2_tn_filter *l2_tn_filter;
	struct rte_eth_l2_tunnel_conf l2_tn_conf;
	int ret = 0;

	while ((l2_tn_filter = TAILQ_FIRST(&l2_tn_info->l2_tn_list))) {
		l2_tn_conf.l2_tunnel_type = l2_tn_filter->key.l2_tn_type;
		l2_tn_conf.tunnel_id      = l2_tn_filter->key.tn_id;
		l2_tn_conf.pool           = l2_tn_filter->pool;
		ret = yusur2_dev_l2_tunnel_filter_del(dev, &l2_tn_conf);
		if (ret < 0)
			return ret;
	}

	return 0;
}

void
yusur2_dev_macsec_setting_save(struct rte_eth_dev *dev,
				struct yusur2_macsec_setting *macsec_setting)
{
	struct yusur2_macsec_setting *macsec =
		YUSUR2_DEV_PRIVATE_TO_MACSEC_SETTING(dev->data->dev_private);

	macsec->offload_en = macsec_setting->offload_en;
	macsec->encrypt_en = macsec_setting->encrypt_en;
	macsec->replayprotect_en = macsec_setting->replayprotect_en;
}

void
yusur2_dev_macsec_setting_reset(struct rte_eth_dev *dev)
{
	struct yusur2_macsec_setting *macsec =
		YUSUR2_DEV_PRIVATE_TO_MACSEC_SETTING(dev->data->dev_private);

	macsec->offload_en = 0;
	macsec->encrypt_en = 0;
	macsec->replayprotect_en = 0;
}

void
yusur2_dev_macsec_register_enable(struct rte_eth_dev *dev,
				struct yusur2_macsec_setting *macsec_setting)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;
	uint8_t en = macsec_setting->encrypt_en;
	uint8_t rp = macsec_setting->replayprotect_en;

	/**
	 * Workaround:
	 * As no yusur2_disable_sec_rx_path equivalent is
	 * implemented for tx in the base code, and we are
	 * not allowed to modify the base code in DPDK, so
	 * just call the hand-written one directly for now.
	 * The hardware support has been checked by
	 * yusur2_disable_sec_rx_path().
	 */
	yusur2_disable_sec_tx_path_generic(hw);

	/* Enable Ethernet CRC (required by MACsec offload) */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_HLREG0);
	ctrl |= YUSUR2_HLREG0_TXCRCEN | YUSUR2_HLREG0_RXCRCSTRP;
	YUSUR2_WRITE_REG(hw, YUSUR2_HLREG0, ctrl);

	/* Enable the TX and RX crypto engines */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_SECTXCTRL);
	ctrl &= ~YUSUR2_SECTXCTRL_SECTX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXCTRL, ctrl);

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_SECRXCTRL);
	ctrl &= ~YUSUR2_SECRXCTRL_SECRX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECRXCTRL, ctrl);

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_SECTXMINIFG);
	ctrl &= ~YUSUR2_SECTX_MINSECIFG_MASK;
	ctrl |= 0x3;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXMINIFG, ctrl);

	/* Enable SA lookup */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_LSECTXCTRL);
	ctrl &= ~YUSUR2_LSECTXCTRL_EN_MASK;
	ctrl |= en ? YUSUR2_LSECTXCTRL_AUTH_ENCRYPT :
		     YUSUR2_LSECTXCTRL_AUTH;
	ctrl |= YUSUR2_LSECTXCTRL_AISCI;
	ctrl &= ~YUSUR2_LSECTXCTRL_PNTHRSH_MASK;
	ctrl |= YUSUR2_MACSEC_PNTHRSH & YUSUR2_LSECTXCTRL_PNTHRSH_MASK;
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXCTRL, ctrl);

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_LSECRXCTRL);
	ctrl &= ~YUSUR2_LSECRXCTRL_EN_MASK;
	ctrl |= YUSUR2_LSECRXCTRL_STRICT << YUSUR2_LSECRXCTRL_EN_SHIFT;
	ctrl &= ~YUSUR2_LSECRXCTRL_PLSH;
	if (rp)
		ctrl |= YUSUR2_LSECRXCTRL_RP;
	else
		ctrl &= ~YUSUR2_LSECRXCTRL_RP;
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXCTRL, ctrl);

	/* Start the data paths */
	yusur2_enable_sec_rx_path(hw);
	/**
	 * Workaround:
	 * As no yusur2_enable_sec_rx_path equivalent is
	 * implemented for tx in the base code, and we are
	 * not allowed to modify the base code in DPDK, so
	 * just call the hand-written one directly for now.
	 */
	yusur2_enable_sec_tx_path_generic(hw);
}

void
yusur2_dev_macsec_register_disable(struct rte_eth_dev *dev)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	/**
	 * Workaround:
	 * As no yusur2_disable_sec_rx_path equivalent is
	 * implemented for tx in the base code, and we are
	 * not allowed to modify the base code in DPDK, so
	 * just call the hand-written one directly for now.
	 * The hardware support has been checked by
	 * yusur2_disable_sec_rx_path().
	 */
	yusur2_disable_sec_tx_path_generic(hw);

	/* Disable the TX and RX crypto engines */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_SECTXCTRL);
	ctrl |= YUSUR2_SECTXCTRL_SECTX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECTXCTRL, ctrl);

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_SECRXCTRL);
	ctrl |= YUSUR2_SECRXCTRL_SECRX_DIS;
	YUSUR2_WRITE_REG(hw, YUSUR2_SECRXCTRL, ctrl);

	/* Disable SA lookup */
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_LSECTXCTRL);
	ctrl &= ~YUSUR2_LSECTXCTRL_EN_MASK;
	ctrl |= YUSUR2_LSECTXCTRL_DISABLE;
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXCTRL, ctrl);

	ctrl = YUSUR2_READ_REG(hw, YUSUR2_LSECRXCTRL);
	ctrl &= ~YUSUR2_LSECRXCTRL_EN_MASK;
	ctrl |= YUSUR2_LSECRXCTRL_DISABLE << YUSUR2_LSECRXCTRL_EN_SHIFT;
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXCTRL, ctrl);

	/* Start the data paths */
	yusur2_enable_sec_rx_path(hw);
	/**
	 * Workaround:
	 * As no yusur2_enable_sec_rx_path equivalent is
	 * implemented for tx in the base code, and we are
	 * not allowed to modify the base code in DPDK, so
	 * just call the hand-written one directly for now.
	 */
	yusur2_enable_sec_tx_path_generic(hw);
}

RTE_PMD_REGISTER_PCI(net_yusur2, rte_yusur2_pmd);
RTE_PMD_REGISTER_PCI_TABLE(net_yusur2, pci_id_yusur2_map);
RTE_PMD_REGISTER_KMOD_DEP(net_yusur2, "* igb_uio | uio_pci_generic | vfio-pci");
RTE_PMD_REGISTER_PCI(net_yusur2_vf, rte_yusur2vf_pmd);
RTE_PMD_REGISTER_PCI_TABLE(net_yusur2_vf, pci_id_yusur2vf_map);
RTE_PMD_REGISTER_KMOD_DEP(net_yusur2_vf, "* igb_uio | vfio-pci");
RTE_PMD_REGISTER_PARAM_STRING(net_yusur2_vf,
			      YUSUR2VF_DEVARG_PFLINK_FULLCHK "=<0|1>");

RTE_INIT(yusur2_init_log)
{
	yusur2_logtype_init = rte_log_register("pmd.net.yusur2.init");
	if (yusur2_logtype_init >= 0)
		rte_log_set_level(yusur2_logtype_init, RTE_LOG_NOTICE);
	yusur2_logtype_driver = rte_log_register("pmd.net.yusur2.driver");
	if (yusur2_logtype_driver >= 0)
		rte_log_set_level(yusur2_logtype_driver, RTE_LOG_NOTICE);
#ifdef RTE_LIBRTE_YUSUR2_DEBUG_RX
	yusur2_logtype_rx = rte_log_register("pmd.net.yusur2.rx");
	if (yusur2_logtype_rx >= 0)
		rte_log_set_level(yusur2_logtype_rx, RTE_LOG_DEBUG);
#endif

#ifdef RTE_LIBRTE_YUSUR2_DEBUG_TX
	yusur2_logtype_tx = rte_log_register("pmd.net.yusur2.tx");
	if (yusur2_logtype_tx >= 0)
		rte_log_set_level(yusur2_logtype_tx, RTE_LOG_DEBUG);
#endif

#ifdef RTE_LIBRTE_YUSUR2_DEBUG_TX_FREE
	yusur2_logtype_tx_free = rte_log_register("pmd.net.yusur2.tx_free");
	if (yusur2_logtype_tx_free >= 0)
		rte_log_set_level(yusur2_logtype_tx_free, RTE_LOG_DEBUG);
#endif
}
