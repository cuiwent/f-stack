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
	u32 fctrl, vmolr = YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_AUPE;
	uint16_t vfn = dev_num_vf(dev);

	/* Check for Promiscuous and All Multicast modes */
	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);

	/* set all bits that we expect to always be set */
	fctrl &= ~YUSUR2_FCTRL_SBP; /* disable store-bad-packets */
	fctrl |= YUSUR2_FCTRL_BAM;

	/* clear the bits we are changing the status of */
	fctrl &= ~(YUSUR2_FCTRL_UPE | YUSUR2_FCTRL_MPE);

	if (dev_data->promiscuous) {
		fctrl |= (YUSUR2_FCTRL_UPE | YUSUR2_FCTRL_MPE);
		vmolr |= (YUSUR2_VMOLR_ROPE | YUSUR2_VMOLR_MPE);
	} else {
		if (dev_data->all_multicast) {
			fctrl |= YUSUR2_FCTRL_MPE;
			vmolr |= YUSUR2_VMOLR_MPE;
		} else {
			vmolr |= YUSUR2_VMOLR_ROMPE;
		}
	}

	if (hw->mac.type != yusur2_mac_82598EB) {
		vmolr |= YUSUR2_READ_REG(hw, YUSUR2_VMOLR(vfn)) &
			 ~(YUSUR2_VMOLR_MPE | YUSUR2_VMOLR_ROMPE |
			   YUSUR2_VMOLR_ROPE);
		YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vfn), vmolr);
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);

	yusur2_vlan_hw_strip_config(dev);
}

static inline void
yusur2_vf_reset_event(struct rte_eth_dev *dev, uint16_t vf)
{
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	int rar_entry = hw->mac.num_rar_entries - (vf + 1);
	uint32_t vmolr = YUSUR2_READ_REG(hw, YUSUR2_VMOLR(vf));

	vmolr |= (YUSUR2_VMOLR_ROPE |
			YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_AUPE);
	YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vf), vmolr);

	YUSUR2_WRITE_REG(hw, YUSUR2_VMVIR(vf), 0);

	/* reset multicast table array for vf */
	vfinfo[vf].num_vf_mc_hashes = 0;

	/* reset rx mode */
	set_rx_mode(dev);

	hw->mac.ops.clear_rar(hw, rar_entry);
}

static inline void
yusur2_vf_reset_msg(struct rte_eth_dev *dev, uint16_t vf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t reg;
	uint32_t reg_offset, vf_shift;
	const uint8_t VFRE_SHIFT = 5;  /* VFRE 32 bits per slot */
	const uint8_t VFRE_MASK = (uint8_t)((1U << VFRE_SHIFT) - 1);
	uint8_t  nb_q_per_pool;
	int i;

	vf_shift = vf & VFRE_MASK;
	reg_offset = (vf >> VFRE_SHIFT) > 0 ? 1 : 0;

	/* enable transmit for vf */
	reg = YUSUR2_READ_REG(hw, YUSUR2_VFTE(reg_offset));
	reg |= (reg | (1 << vf_shift));
	YUSUR2_WRITE_REG(hw, YUSUR2_VFTE(reg_offset), reg);

	/* enable all queue drop for IOV */
	nb_q_per_pool = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
	for (i = vf * nb_q_per_pool; i < (vf + 1) * nb_q_per_pool; i++) {
		YUSUR2_WRITE_FLUSH(hw);
		reg = YUSUR2_QDE_ENABLE | YUSUR2_QDE_WRITE;
		reg |= i << YUSUR2_QDE_IDX_SHIFT;
		YUSUR2_WRITE_REG(hw, YUSUR2_QDE, reg);
	}

	/* enable receive for vf */
	reg = YUSUR2_READ_REG(hw, YUSUR2_VFRE(reg_offset));
	reg |= (reg | (1 << vf_shift));
	YUSUR2_WRITE_REG(hw, YUSUR2_VFRE(reg_offset), reg);

	/* Enable counting of spoofed packets in the SSVPC register */
	reg = YUSUR2_READ_REG(hw, YUSUR2_VMECM(reg_offset));
	reg |= (1 << vf_shift);
	YUSUR2_WRITE_REG(hw, YUSUR2_VMECM(reg_offset), reg);

	yusur2_vf_reset_event(dev, vf);
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
yusur2_vf_reset(struct rte_eth_dev *dev, uint16_t vf, uint32_t *msgbuf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	unsigned char *vf_mac = vfinfo[vf].vf_mac_addresses;
	int rar_entry = hw->mac.num_rar_entries - (vf + 1);
	uint8_t *new_mac = (uint8_t *)(&msgbuf[1]);

	yusur2_vf_reset_msg(dev, vf);

	hw->mac.ops.set_rar(hw, rar_entry, vf_mac, vf, YUSUR2_RAH_AV);

	/* Disable multicast promiscuous at reset */
	yusur2_disable_vf_mc_promisc(dev, vf);

	/* reply to reset with ack and vf mac address */
	msgbuf[0] = YUSUR2_VF_RESET | YUSUR2_VT_MSGTYPE_ACK;
	rte_memcpy(new_mac, vf_mac, RTE_ETHER_ADDR_LEN);
	/*
	 * Piggyback the multicast filter type so VF can compute the
	 * correct vectors
	 */
	msgbuf[3] = hw->mac.mc_filter_type;
	yusur2_write_mbx(hw, msgbuf, YUSUR2_VF_PERMADDR_MSG_LEN, vf);

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

static int
yusur2_vf_set_multicast(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	int nb_entries = (msgbuf[0] & YUSUR2_VT_MSGINFO_MASK) >>
		YUSUR2_VT_MSGINFO_SHIFT;
	uint16_t *hash_list = (uint16_t *)&msgbuf[1];
	uint32_t mta_idx;
	uint32_t mta_shift;
	const uint32_t YUSUR2_MTA_INDEX_MASK = 0x7F;
	const uint32_t YUSUR2_MTA_BIT_SHIFT = 5;
	const uint32_t YUSUR2_MTA_BIT_MASK = (0x1 << YUSUR2_MTA_BIT_SHIFT) - 1;
	uint32_t reg_val;
	int i;
	u32 vmolr = YUSUR2_READ_REG(hw, YUSUR2_VMOLR(vf));

	/* Disable multicast promiscuous first */
	yusur2_disable_vf_mc_promisc(dev, vf);

	/* only so many hash values supported */
	nb_entries = RTE_MIN(nb_entries, YUSUR2_MAX_VF_MC_ENTRIES);

	/* store the mc entries  */
	vfinfo->num_vf_mc_hashes = (uint16_t)nb_entries;
	for (i = 0; i < nb_entries; i++) {
		vfinfo->vf_mc_hashes[i] = hash_list[i];
	}

	if (nb_entries == 0) {
		vmolr &= ~YUSUR2_VMOLR_ROMPE;
		YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vf), vmolr);
		return 0;
	}

	for (i = 0; i < vfinfo->num_vf_mc_hashes; i++) {
		mta_idx = (vfinfo->vf_mc_hashes[i] >> YUSUR2_MTA_BIT_SHIFT)
				& YUSUR2_MTA_INDEX_MASK;
		mta_shift = vfinfo->vf_mc_hashes[i] & YUSUR2_MTA_BIT_MASK;
		reg_val = YUSUR2_READ_REG(hw, YUSUR2_MTA(mta_idx));
		reg_val |= (1 << mta_shift);
		YUSUR2_WRITE_REG(hw, YUSUR2_MTA(mta_idx), reg_val);
	}

	vmolr |= YUSUR2_VMOLR_ROMPE;
	YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vf), vmolr);

	return 0;
}

static int
yusur2_vf_set_vlan(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	int add, vid;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));

	add = (msgbuf[0] & YUSUR2_VT_MSGINFO_MASK)
		>> YUSUR2_VT_MSGINFO_SHIFT;
	vid = (msgbuf[1] & YUSUR2_VLVF_VLANID_MASK);

	if (add)
		vfinfo[vf].vlan_count++;
	else if (vfinfo[vf].vlan_count)
		vfinfo[vf].vlan_count--;
	return hw->mac.ops.set_vfta(hw, vid, vf, (bool)add, false);
}

static int
yusur2_set_vf_lpe(struct rte_eth_dev *dev, __rte_unused uint32_t vf, uint32_t *msgbuf)
{
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t new_mtu = msgbuf[1];
	uint32_t max_frs;
	int max_frame = new_mtu + RTE_ETHER_HDR_LEN + RTE_ETHER_CRC_LEN;

	/* X540 and X550 support jumbo frames in IOV mode */
	if (hw->mac.type != yusur2_mac_X540 &&
		hw->mac.type != yusur2_mac_X550 &&
		hw->mac.type != yusur2_mac_X550EM_x &&
		hw->mac.type != yusur2_mac_X550EM_a)
		return -1;

	if (max_frame < RTE_ETHER_MIN_LEN ||
			max_frame > RTE_ETHER_MAX_JUMBO_FRAME_LEN)
		return -1;

	max_frs = (YUSUR2_READ_REG(hw, YUSUR2_MAXFRS) &
		   YUSUR2_MHADD_MFS_MASK) >> YUSUR2_MHADD_MFS_SHIFT;
	if (max_frs < new_mtu) {
		max_frs = new_mtu << YUSUR2_MHADD_MFS_SHIFT;
		YUSUR2_WRITE_REG(hw, YUSUR2_MAXFRS, max_frs);
	}

	return 0;
}

static int
yusur2_negotiate_vf_api(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	uint32_t api_version = msgbuf[1];
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);

	switch (api_version) {
	case yusur2_mbox_api_10:
	case yusur2_mbox_api_11:
	case yusur2_mbox_api_12:
	case yusur2_mbox_api_13:
		vfinfo[vf].api_version = (uint8_t)api_version;
		return 0;
	default:
		break;
	}

	PMD_DRV_LOG(ERR, "Negotiate invalid api version %u from VF %d\n",
		api_version, vf);

	return -1;
}

static int
yusur2_get_vf_queues(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	uint32_t default_q = vf * RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
	struct rte_eth_conf *eth_conf;
	struct rte_eth_vmdq_dcb_tx_conf *vmdq_dcb_tx_conf;
	u8 num_tcs;
	struct yusur2_hw *hw;
	u32 vmvir;
#define YUSUR2_VMVIR_VLANA_MASK		0xC0000000
#define YUSUR2_VMVIR_VLAN_VID_MASK	0x00000FFF
#define YUSUR2_VMVIR_VLAN_UP_MASK	0x0000E000
#define VLAN_PRIO_SHIFT			13
	u32 vlana;
	u32 vid;
	u32 user_priority;

	/* Verify if the PF supports the mbox APIs version or not */
	switch (vfinfo[vf].api_version) {
	case yusur2_mbox_api_20:
	case yusur2_mbox_api_11:
	case yusur2_mbox_api_12:
	case yusur2_mbox_api_13:
		break;
	default:
		return -1;
	}

	/* Notify VF of Rx and Tx queue number */
	msgbuf[YUSUR2_VF_RX_QUEUES] = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
	msgbuf[YUSUR2_VF_TX_QUEUES] = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;

	/* Notify VF of default queue */
	msgbuf[YUSUR2_VF_DEF_QUEUE] = default_q;

	/* Notify VF of number of DCB traffic classes */
	eth_conf = &dev->data->dev_conf;
	switch (eth_conf->txmode.mq_mode) {
	case ETH_MQ_TX_NONE:
	case ETH_MQ_TX_DCB:
		PMD_DRV_LOG(ERR, "PF must work with virtualization for VF %u"
			", but its tx mode = %d\n", vf,
			eth_conf->txmode.mq_mode);
		return -1;

	case ETH_MQ_TX_VMDQ_DCB:
		vmdq_dcb_tx_conf = &eth_conf->tx_adv_conf.vmdq_dcb_tx_conf;
		switch (vmdq_dcb_tx_conf->nb_queue_pools) {
		case ETH_16_POOLS:
			num_tcs = ETH_8_TCS;
			break;
		case ETH_32_POOLS:
			num_tcs = ETH_4_TCS;
			break;
		default:
			return -1;
		}
		break;

	/* ETH_MQ_TX_VMDQ_ONLY,  DCB not enabled */
	case ETH_MQ_TX_VMDQ_ONLY:
		hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
		vmvir = YUSUR2_READ_REG(hw, YUSUR2_VMVIR(vf));
		vlana = vmvir & YUSUR2_VMVIR_VLANA_MASK;
		vid = vmvir & YUSUR2_VMVIR_VLAN_VID_MASK;
		user_priority =
			(vmvir & YUSUR2_VMVIR_VLAN_UP_MASK) >> VLAN_PRIO_SHIFT;
		if ((vlana == YUSUR2_VMVIR_VLANA_DEFAULT) &&
			((vid !=  0) || (user_priority != 0)))
			num_tcs = 1;
		else
			num_tcs = 0;
		break;

	default:
		PMD_DRV_LOG(ERR, "PF work with invalid mode = %d\n",
			eth_conf->txmode.mq_mode);
		return -1;
	}
	msgbuf[YUSUR2_VF_TRANS_VLAN] = num_tcs;

	return 0;
}

static int
yusur2_set_vf_mc_promisc(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct yusur2_vf_info *vfinfo =
		*(YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int xcast_mode = msgbuf[1];	/* msgbuf contains the flag to enable */
	u32 vmolr, fctrl, disable, enable;

	switch (vfinfo[vf].api_version) {
	case yusur2_mbox_api_12:
		/* promisc introduced in 1.3 version */
		if (xcast_mode == YUSUR2VF_XCAST_MODE_PROMISC)
			return -EOPNOTSUPP;
		break;
		/* Fall threw */
	case yusur2_mbox_api_13:
		break;
	default:
		return -1;
	}

	if (vfinfo[vf].xcast_mode == xcast_mode)
		goto out;

	switch (xcast_mode) {
	case YUSUR2VF_XCAST_MODE_NONE:
		disable = YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_ROMPE |
			  YUSUR2_VMOLR_MPE | YUSUR2_VMOLR_UPE | YUSUR2_VMOLR_VPE;
		enable = 0;
		break;
	case YUSUR2VF_XCAST_MODE_MULTI:
		disable = YUSUR2_VMOLR_MPE | YUSUR2_VMOLR_UPE | YUSUR2_VMOLR_VPE;
		enable = YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_ROMPE;
		break;
	case YUSUR2VF_XCAST_MODE_ALLMULTI:
		disable = YUSUR2_VMOLR_UPE | YUSUR2_VMOLR_VPE;
		enable = YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_ROMPE | YUSUR2_VMOLR_MPE;
		break;
	case YUSUR2VF_XCAST_MODE_PROMISC:
		if (hw->mac.type <= yusur2_mac_82599EB)
			return -1;

		fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);
		if (!(fctrl & YUSUR2_FCTRL_UPE)) {
			/* VF promisc requires PF in promisc */
			PMD_DRV_LOG(ERR,
			       "Enabling VF promisc requires PF in promisc\n");
			return -1;
		}

		disable = 0;
		enable = YUSUR2_VMOLR_BAM | YUSUR2_VMOLR_ROMPE |
			 YUSUR2_VMOLR_MPE | YUSUR2_VMOLR_UPE | YUSUR2_VMOLR_VPE;
		break;
	default:
		return -1;
	}

	vmolr = YUSUR2_READ_REG(hw, YUSUR2_VMOLR(vf));
	vmolr &= ~disable;
	vmolr |= enable;
	YUSUR2_WRITE_REG(hw, YUSUR2_VMOLR(vf), vmolr);
	vfinfo[vf].xcast_mode = xcast_mode;

out:
	msgbuf[1] = xcast_mode;

	return 0;
}

static int
yusur2_rcv_msg_from_vf(struct rte_eth_dev *dev, uint16_t vf)
{
	uint16_t mbx_size = YUSUR2_VFMAILBOX_SIZE;
	uint16_t msg_size = YUSUR2_VF_MSG_SIZE_DEFAULT;
	uint32_t msgbuf[YUSUR2_VFMAILBOX_SIZE];
	int32_t retval;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	struct rte_pmd_yusur2_mb_event_param ret_param;

	retval = yusur2_read_mbx(hw, msgbuf, mbx_size, vf);
	if (retval) {
		PMD_DRV_LOG(ERR, "Error mbx recv msg from VF %d", vf);
		return retval;
	}

	/* do nothing with the message already been processed */
	if (msgbuf[0] & (YUSUR2_VT_MSGTYPE_ACK | YUSUR2_VT_MSGTYPE_NACK))
		return retval;

	/* flush the ack before we write any messages back */
	YUSUR2_WRITE_FLUSH(hw);

	/**
	 * initialise structure to send to user application
	 * will return response from user in retval field
	 */
	ret_param.retval = RTE_PMD_YUSUR2_MB_EVENT_PROCEED;
	ret_param.vfid = vf;
	ret_param.msg_type = msgbuf[0] & 0xFFFF;
	ret_param.msg = (void *)msgbuf;

	/* perform VF reset */
	if (msgbuf[0] == YUSUR2_VF_RESET) {
		int ret = yusur2_vf_reset(dev, vf, msgbuf);

		vfinfo[vf].clear_to_send = true;

		/* notify application about VF reset */
		_rte_eth_dev_callback_process(dev, RTE_ETH_EVENT_VF_MBOX,
					      &ret_param);
		return ret;
	}

	/**
	 * ask user application if we allowed to perform those functions
	 * if we get ret_param.retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED
	 * then business as usual,
	 * if 0, do nothing and send ACK to VF
	 * if ret_param.retval > 1, do nothing and send NAK to VF
	 */
	_rte_eth_dev_callback_process(dev, RTE_ETH_EVENT_VF_MBOX,
				      &ret_param);

	retval = ret_param.retval;

	/* check & process VF to PF mailbox message */
	switch ((msgbuf[0] & 0xFFFF)) {
	case YUSUR2_VF_SET_MAC_ADDR:
		if (retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED)
			retval = yusur2_vf_set_mac_addr(dev, vf, msgbuf);
		break;
	case YUSUR2_VF_SET_MULTICAST:
		if (retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED)
			retval = yusur2_vf_set_multicast(dev, vf, msgbuf);
		break;
	case YUSUR2_VF_SET_LPE:
		if (retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED)
			retval = yusur2_set_vf_lpe(dev, vf, msgbuf);
		break;
	case YUSUR2_VF_SET_VLAN:
		if (retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED)
			retval = yusur2_vf_set_vlan(dev, vf, msgbuf);
		break;
	case YUSUR2_VF_API_NEGOTIATE:
		retval = yusur2_negotiate_vf_api(dev, vf, msgbuf);
		break;
	case YUSUR2_VF_GET_QUEUES:
		retval = yusur2_get_vf_queues(dev, vf, msgbuf);
		msg_size = YUSUR2_VF_GET_QUEUE_MSG_SIZE;
		break;
	case YUSUR2_VF_UPDATE_XCAST_MODE:
		if (retval == RTE_PMD_YUSUR2_MB_EVENT_PROCEED)
			retval = yusur2_set_vf_mc_promisc(dev, vf, msgbuf);
		break;
	default:
		PMD_DRV_LOG(DEBUG, "Unhandled Msg %8.8x", (unsigned)msgbuf[0]);
		retval = YUSUR2_ERR_MBX;
		break;
	}

	/* response the VF according to the message process result */
	if (retval)
		msgbuf[0] |= YUSUR2_VT_MSGTYPE_NACK;
	else
		msgbuf[0] |= YUSUR2_VT_MSGTYPE_ACK;

	msgbuf[0] |= YUSUR2_VT_MSGTYPE_CTS;

	yusur2_write_mbx(hw, msgbuf, msg_size, vf);

	return retval;
}

static inline void
yusur2_rcv_ack_from_vf(struct rte_eth_dev *dev, uint16_t vf)
{
	uint32_t msg = YUSUR2_VT_MSGTYPE_NACK;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct yusur2_vf_info *vfinfo =
		*YUSUR2_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);

	if (!vfinfo[vf].clear_to_send)
		yusur2_write_mbx(hw, &msg, 1, vf);
}

void yusur2_pf_mbx_process(struct rte_eth_dev *eth_dev)
{
	uint16_t vf;
	struct yusur2_hw *hw =
		YUSUR2_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);

	for (vf = 0; vf < dev_num_vf(eth_dev); vf++) {
		/* check & process vf function level reset */
		if (!yusur2_check_for_rst(hw, vf))
			yusur2_vf_reset_event(eth_dev, vf);

		/* check & process vf mailbox messages */
		if (!yusur2_check_for_msg(hw, vf))
			yusur2_rcv_msg_from_vf(eth_dev, vf);

		/* check & process acks from vf */
		if (!yusur2_check_for_ack(hw, vf))
			yusur2_rcv_ack_from_vf(eth_dev, vf);
	}
}
