/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <inttypes.h>

#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_eal.h>
#include <rte_ether.h>
#include <rte_ethdev_driver.h>
#include <rte_memcpy.h>
#include <rte_malloc.h>
#include <rte_random.h>

#include "base/yusur2_common.h"
#include "yusur2_ethdev.h"
#include "rte_pmd_yusur2.h"

#define YUSUR2_MAX_VFTA     (128)
#define YUSUR2_VF_MSG_SIZE_DEFAULT 1
#define YUSUR2_VF_GET_QUEUE_MSG_SIZE 5
#define YUSUR2_ETHERTYPE_FLOW_CTRL 0x8808

static inline uint16_t
dev_num_vf(struct rte_eth_dev *eth_dev)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(eth_dev);

	return pci_dev->max_vfs;
}

static inline
int yusur2_vf_perm_addr_gen(struct rte_eth_dev *dev, uint16_t vf_num)
{
	unsigned char vf_mac_addr[RTE_ETHER_ADDR_LEN];
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	uint16_t vfn;

	for (vfn = 0; vfn < vf_num; vfn++) {
		rte_eth_random_addr(vf_mac_addr);
		/* keep the random address as default */
		memcpy(vfinfo[vfn].vf_mac_addresses, vf_mac_addr,
			   RTE_ETHER_ADDR_LEN);
	}

	return 0;
}

static inline int
yusur2_mb_intr_setup(struct rte_eth_dev *dev)
{
	struct yusur2_interrupt *intr =
		YUSUR2_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	intr->mask |= YUSUR2_EICR_MAILBOX;

	return 0;
}

int yusur2_pf_host_init(struct rte_eth_dev *eth_dev)
{
	struct yusur2_vf_info **vfinfo =
		YUSUR2_DEV_PRIVATE_TO_P_VFDATA(eth_dev->data->dev_private);
	struct yusur2_mirror_info *mirror_info =
	YUSUR2_DEV_PRIVATE_TO_PFDATA(eth_dev->data->dev_private);
	struct yusur2_uta_info *uta_info =
	YUSUR2_DEV_PRIVATE_TO_UTA(eth_dev->data->dev_private);
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	uint16_t vf_num;
	uint8_t nb_queue;
	int ret = 0;

	PMD_INIT_FUNC_TRACE();

	RTE_ETH_DEV_SRIOV(eth_dev).active = 0;
	vf_num = dev_num_vf(eth_dev);
	if (vf_num == 0)
		return ret;

	*vfinfo = rte_zmalloc("vf_info", sizeof(struct yusur2_vf_info) * vf_num, 0);
	if (*vfinfo == NULL)
		rte_panic("Cannot allocate memory for private VF data\n");

	ret = rte_eth_switch_domain_alloc(&(*vfinfo)->switch_domain_id);
	if (ret) {
		PMD_INIT_LOG(ERR,
			"failed to allocate switch domain for device %d", ret);
		rte_free(*vfinfo);
		*vfinfo = NULL;
		return ret;
	}

	memset(mirror_info, 0, sizeof(struct yusur2_mirror_info));
	memset(uta_info, 0, sizeof(struct yusur2_uta_info));
	hw->mac.mc_filter_type = 0;

	if (vf_num >= ETH_32_POOLS) {
		nb_queue = 2;
		RTE_ETH_DEV_SRIOV(eth_dev).active = ETH_64_POOLS;
	} else if (vf_num >= ETH_16_POOLS) {
		nb_queue = 4;
		RTE_ETH_DEV_SRIOV(eth_dev).active = ETH_32_POOLS;
	} else {
		nb_queue = 8;
		RTE_ETH_DEV_SRIOV(eth_dev).active = ETH_16_POOLS;
	}

	RTE_ETH_DEV_SRIOV(eth_dev).nb_q_per_pool = nb_queue;
	RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx = vf_num;
	RTE_ETH_DEV_SRIOV(eth_dev).def_pool_q_idx = (uint16_t)(vf_num * nb_queue);

	yusur2_vf_perm_addr_gen(eth_dev, vf_num);

	/* init_mailbox_params */
	hw->mbx.ops.init_params(hw);

	/* set mb interrupt mask */
	yusur2_mb_intr_setup(eth_dev);

	return ret;
}

void yusur2_pf_host_uninit(struct rte_eth_dev *eth_dev)
{
	struct yusur2_vf_info **vfinfo;
	uint16_t vf_num;
	int ret;

	PMD_INIT_FUNC_TRACE();

	RTE_ETH_DEV_SRIOV(eth_dev).active = 0;
	RTE_ETH_DEV_SRIOV(eth_dev).nb_q_per_pool = 0;
	RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx = 0;
	RTE_ETH_DEV_SRIOV(eth_dev).def_pool_q_idx = 0;

	vf_num = dev_num_vf(eth_dev);
	if (vf_num == 0)
		return;

	vfinfo = YUSUR2_DEV_PRIVATE_TO_P_VFDATA(eth_dev->data->dev_private);
	if (*vfinfo == NULL)
		return;

	ret = rte_eth_switch_domain_free((*vfinfo)->switch_domain_id);
	if (ret)
		PMD_INIT_LOG(WARNING, "failed to free switch domain: %d", ret);

	rte_free(*vfinfo);
	*vfinfo = NULL;
}

static void
yusur2_add_tx_flow_control_drop_filter(struct rte_eth_dev *eth_dev)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	struct yusur2_filter_info *filter_info =
		YUSUR2_DEV_PRIVATE_TO_FILTER_INFO(eth_dev->data->dev_private);
	uint16_t vf_num;
	int i;
	struct yusur2_ethertype_filter ethertype_filter;

	if (!hw->mac.ops.set_ethertype_anti_spoofing) {
		PMD_DRV_LOG(INFO, "ether type anti-spoofing is not supported.\n");
		return;
	}

	i = yusur2_ethertype_filter_lookup(filter_info,
					  YUSUR2_ETHERTYPE_FLOW_CTRL);
	if (i >= 0) {
		PMD_DRV_LOG(ERR, "A ether type filter entity for flow control already exists!\n");
		return;
	}

	ethertype_filter.ethertype = YUSUR2_ETHERTYPE_FLOW_CTRL;
	ethertype_filter.etqf = YUSUR2_ETQF_FILTER_EN |
				YUSUR2_ETQF_TX_ANTISPOOF |
				YUSUR2_ETHERTYPE_FLOW_CTRL;
	ethertype_filter.etqs = 0;
	ethertype_filter.conf = TRUE;
	i = yusur2_ethertype_filter_insert(filter_info,
					  &ethertype_filter);
	if (i < 0) {
		PMD_DRV_LOG(ERR, "Cannot find an unused ether type filter entity for flow control.\n");
		return;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_ETQF(i),
			(YUSUR2_ETQF_FILTER_EN |
			YUSUR2_ETQF_TX_ANTISPOOF |
			YUSUR2_ETHERTYPE_FLOW_CTRL));

	vf_num = dev_num_vf(eth_dev);
	for (i = 0; i < vf_num; i++)
		hw->mac.ops.set_ethertype_anti_spoofing(hw, true, i);
}

int yusur2_pf_host_configure(struct rte_eth_dev *eth_dev)
{
	uint32_t vtctl, fcrth;
	uint32_t vfre_slot, vfre_offset;
	uint16_t vf_num;
	const uint8_t VFRE_SHIFT = 5;  /* VFRE 32 bits per slot */
	const uint8_t VFRE_MASK = (uint8_t)((1U << VFRE_SHIFT) - 1);
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	uint32_t gpie, gcr_ext;
	uint32_t vlanctrl;
	int i;

	vf_num = dev_num_vf(eth_dev);
	if (vf_num == 0)
		return -1;

	/* enable VMDq and set the default pool for PF */
	vtctl = YUSUR2_READ_REG(hw, YUSUR2_VT_CTL);
	vtctl |= YUSUR2_VMD_CTL_VMDQ_EN;
	vtctl &= ~YUSUR2_VT_CTL_POOL_MASK;
	vtctl |= RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx
		<< YUSUR2_VT_CTL_POOL_SHIFT;
	vtctl |= YUSUR2_VT_CTL_REPLEN;
	YUSUR2_WRITE_REG(hw, YUSUR2_VT_CTL, vtctl);

	vfre_offset = vf_num & VFRE_MASK;
	vfre_slot = (vf_num >> VFRE_SHIFT) > 0 ? 1 : 0;

	/* Enable pools reserved to PF only */
	YUSUR2_WRITE_REG(hw, YUSUR2_VFRE(vfre_slot), (~0U) << vfre_offset);
	YUSUR2_WRITE_REG(hw, YUSUR2_VFRE(vfre_slot ^ 1), vfre_slot - 1);
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTE(vfre_slot), (~0U) << vfre_offset);
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTE(vfre_slot ^ 1), vfre_slot - 1);

	/* PFDMA Tx General Switch Control Enables VMDQ loopback */
	YUSUR2_WRITE_REG(hw, YUSUR2_PFDTXGSWC, YUSUR2_PFDTXGSWC_VT_LBEN);

	/* clear VMDq map to perment rar 0 */
	hw->mac.ops.clear_vmdq(hw, 0, YUSUR2_CLEAR_VMDQ_ALL);

	/* clear VMDq map to scan rar 127 */
	YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_LO(hw->mac.num_rar_entries), 0);
	YUSUR2_WRITE_REG(hw, YUSUR2_MPSAR_HI(hw->mac.num_rar_entries), 0);

	/* set VMDq map to default PF pool */
	hw->mac.ops.set_vmdq(hw, 0, RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx);

	/*
	 * SW msut set GCR_EXT.VT_Mode the same as GPIE.VT_Mode
	 */
	gcr_ext = YUSUR2_READ_REG(hw, YUSUR2_GCR_EXT);
	gcr_ext &= ~YUSUR2_GCR_EXT_VT_MODE_MASK;

	gpie = YUSUR2_READ_REG(hw, YUSUR2_GPIE);
	gpie &= ~YUSUR2_GPIE_VTMODE_MASK;
	gpie |= YUSUR2_GPIE_MSIX_MODE | YUSUR2_GPIE_PBA_SUPPORT;

	switch (RTE_ETH_DEV_SRIOV(eth_dev).active) {
	case ETH_64_POOLS:
		gcr_ext |= YUSUR2_GCR_EXT_VT_MODE_64;
		gpie |= YUSUR2_GPIE_VTMODE_64;
		break;
	case ETH_32_POOLS:
		gcr_ext |= YUSUR2_GCR_EXT_VT_MODE_32;
		gpie |= YUSUR2_GPIE_VTMODE_32;
		break;
	case ETH_16_POOLS:
		gcr_ext |= YUSUR2_GCR_EXT_VT_MODE_16;
		gpie |= YUSUR2_GPIE_VTMODE_16;
		break;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_GCR_EXT, gcr_ext);
	YUSUR2_WRITE_REG(hw, YUSUR2_GPIE, gpie);

	/*
	 * enable vlan filtering and allow all vlan tags through
	 */
	vlanctrl = YUSUR2_READ_REG(hw, YUSUR2_VLNCTRL);
	vlanctrl |= YUSUR2_VLNCTRL_VFE; /* enable vlan filters */
	YUSUR2_WRITE_REG(hw, YUSUR2_VLNCTRL, vlanctrl);

	/* VFTA - enable all vlan filters */
	for (i = 0; i < YUSUR2_MAX_VFTA; i++)
		YUSUR2_WRITE_REG(hw, YUSUR2_VFTA(i), 0xFFFFFFFF);

	/* Enable MAC Anti-Spoofing */
	hw->mac.ops.set_mac_anti_spoofing(hw, FALSE, vf_num);

	/* set flow control threshold to max to avoid tx switch hang */
	for (i = 0; i < YUSUR2_DCB_MAX_TRAFFIC_CLASS; i++) {
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTL_82599(i), 0);
		fcrth = YUSUR2_READ_REG(hw, YUSUR2_RXPBSIZE(i)) - 32;
		YUSUR2_WRITE_REG(hw, YUSUR2_FCRTH_82599(i), fcrth);
	}

	yusur2_add_tx_flow_control_drop_filter(eth_dev);

	return 0;
}

static void
set_rx_mode(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_data *dev_data = dev->data;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	//TODO:
}

static int
yusur2_disable_vf_mc_promisc(struct rte_eth_dev *dev, uint32_t vf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t vmolr;

	vmolr = YUSUR2_READ_REG(hw, YUSUR2_VMOLR(vf));

	PMD_DRV_LOG(INFO, "VF %u: disabling multicast promiscuous\n", vf);

	vmolr &= ~YUSUR2_VMOLR_MPE;

	YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vf), vmolr);

	return 0;
}

static int
yusur2_vf_set_mac_addr(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	int rar_entry = hw->mac.num_rar_entries - (vf + 1);
	uint8_t *new_mac = (uint8_t *)(&msgbuf[1]);

	if (rte_is_valid_assigned_ether_addr(
			(struct rte_ether_addr *)new_mac)) {
		rte_memcpy(vfinfo[vf].vf_mac_addresses, new_mac, 6);
		return hw->mac.ops.set_rar(hw, rar_entry, new_mac, vf, YUSUR2_RAH_AV);
	}
	return -1;
}

void yusur2_pf_mbx_process(struct rte_eth_dev *eth_dev)
{
	uint16_t vf;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);

	//TODO:
}
