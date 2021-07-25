/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#include <rte_ethdev_driver.h>

#include "base/yusur2_api.h"
#include "yusur2_ethdev.h"
#include "rte_pmd_yusur2.h"

int
rte_pmd_yusur2_set_vf_vlan_anti_spoof(uint16_t port, uint16_t vf, uint8_t on)
{
	struct yusur2_hw *hw;
	struct yusur2_mac_info *mac;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mac = &hw->mac;

	mac->ops.set_vlan_anti_spoofing(hw, on, vf);

	return 0;
}

int
rte_pmd_yusur2_set_vf_mac_anti_spoof(uint16_t port, uint16_t vf, uint8_t on)
{
	struct yusur2_hw *hw;
	struct yusur2_mac_info *mac;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mac = &hw->mac;
	mac->ops.set_mac_anti_spoofing(hw, on, vf);

	return 0;
}

int
rte_pmd_yusur2_set_vf_vlan_insert(uint16_t port, uint16_t vf, uint16_t vlan_id)
{
	struct yusur2_hw *hw;
	uint32_t ctrl;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (vlan_id > RTE_ETHER_MAX_VLAN_ID)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_VMVIR(vf));
	if (vlan_id) {
		ctrl = vlan_id;
		ctrl |= YUSUR2_VMVIR_VLANA_DEFAULT;
	} else {
		ctrl = 0;
	}

	YUSUR2_WRITE_REG(hw, YUSUR2_VMVIR(vf), ctrl);

	return 0;
}

int
rte_pmd_yusur2_set_tx_loopback(uint16_t port, uint8_t on)
{
	struct yusur2_hw *hw;
	uint32_t ctrl;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	ctrl = YUSUR2_READ_REG(hw, YUSUR2_PFDTXGSWC);
	/* enable or disable VMDQ loopback */
	if (on)
		ctrl |= YUSUR2_PFDTXGSWC_VT_LBEN;
	else
		ctrl &= ~YUSUR2_PFDTXGSWC_VT_LBEN;

	YUSUR2_WRITE_REG(hw, YUSUR2_PFDTXGSWC, ctrl);

	return 0;
}

int
rte_pmd_yusur2_set_all_queues_drop_en(uint16_t port, uint8_t on)
{
	struct yusur2_hw *hw;
	uint32_t reg_value;
	int i;
	int num_queues = (int)(YUSUR2_QDE_IDX_MASK >> YUSUR2_QDE_IDX_SHIFT);
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	for (i = 0; i <= num_queues; i++) {
		reg_value = YUSUR2_QDE_WRITE |
				(i << YUSUR2_QDE_IDX_SHIFT) |
				(on & YUSUR2_QDE_ENABLE);
		YUSUR2_WRITE_REG(hw, YUSUR2_QDE, reg_value);
	}

	return 0;
}

int
rte_pmd_yusur2_set_vf_split_drop_en(uint16_t port, uint16_t vf, uint8_t on)
{
	struct yusur2_hw *hw;
	uint32_t reg_value;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	/* only support VF's 0 to 63 */
	if ((vf >= pci_dev->max_vfs) || (vf > 63))
		return -EINVAL;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	reg_value = YUSUR2_READ_REG(hw, YUSUR2_SRRCTL(vf));
	if (on)
		reg_value |= YUSUR2_SRRCTL_DROP_EN;
	else
		reg_value &= ~YUSUR2_SRRCTL_DROP_EN;

	YUSUR2_WRITE_REG(hw, YUSUR2_SRRCTL(vf), reg_value);

	return 0;
}

int
rte_pmd_yusur2_set_vf_rx(uint16_t port, uint16_t vf, uint8_t on)
{
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	uint32_t reg, addr;
	uint32_t val;
	const uint8_t bit1 = 0x1;
	struct yusur2_hw *hw;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (yusur2_vt_check(hw) < 0)
		return -ENOTSUP;

	/* for vf >= 32, set bit in PFVFRE[1], otherwise PFVFRE[0] */
	if (vf >= 32) {
		addr = YUSUR2_VFRE(1);
		val = bit1 << (vf - 32);
	} else {
		addr = YUSUR2_VFRE(0);
		val = bit1 << vf;
	}

	reg = YUSUR2_READ_REG(hw, addr);

	if (on)
		reg |= val;
	else
		reg &= ~val;

	YUSUR2_WRITE_REG(hw, addr, reg);

	return 0;
}

int
rte_pmd_yusur2_set_vf_tx(uint16_t port, uint16_t vf, uint8_t on)
{
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	uint32_t reg, addr;
	uint32_t val;
	const uint8_t bit1 = 0x1;

	struct yusur2_hw *hw;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	pci_dev = RTE_ETH_DEV_TO_PCI(dev);

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vf >= pci_dev->max_vfs)
		return -EINVAL;

	if (on > 1)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (yusur2_vt_check(hw) < 0)
		return -ENOTSUP;

	/* for vf >= 32, set bit in PFVFTE[1], otherwise PFVFTE[0] */
	if (vf >= 32) {
		addr = YUSUR2_VFTE(1);
		val = bit1 << (vf - 32);
	} else {
		addr = YUSUR2_VFTE(0);
		val = bit1 << vf;
	}

	reg = YUSUR2_READ_REG(hw, addr);

	if (on)
		reg |= val;
	else
		reg &= ~val;

	YUSUR2_WRITE_REG(hw, addr, reg);

	return 0;
}

int
rte_pmd_yusur2_set_vf_vlan_filter(uint16_t port, uint16_t vlan,
				 uint64_t vf_mask, uint8_t vlan_on)
{
	struct rte_eth_dev *dev;
	int ret = 0;
	uint16_t vf_idx;
	struct yusur2_hw *hw;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	if (vlan > RTE_ETHER_MAX_VLAN_ID || vf_mask == 0)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (yusur2_vt_check(hw) < 0)
		return -ENOTSUP;

	for (vf_idx = 0; vf_idx < 64; vf_idx++) {
		if (vf_mask & ((uint64_t)(1ULL << vf_idx))) {
			ret = hw->mac.ops.set_vfta(hw, vlan, vf_idx,
						   vlan_on, false);
			if (ret < 0)
				return ret;
		}
	}

	return ret;
}

int
rte_pmd_yusur2_set_vf_rate_limit(uint16_t port, uint16_t vf,
				uint16_t tx_rate, uint64_t q_msk)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_set_vf_rate_limit(dev, vf, tx_rate, q_msk);
}

int
rte_pmd_yusur2_macsec_enable(uint16_t port, uint8_t en, uint8_t rp)
{
	struct rte_eth_dev *dev;
	struct yusur2_macsec_setting macsec_setting;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	macsec_setting.offload_en = 1;
	macsec_setting.encrypt_en = en;
	macsec_setting.replayprotect_en = rp;

	yusur2_dev_macsec_setting_save(dev, &macsec_setting);

	yusur2_dev_macsec_register_enable(dev, &macsec_setting);

	return 0;
}

int
rte_pmd_yusur2_macsec_disable(uint16_t port)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	yusur2_dev_macsec_setting_reset(dev);

	yusur2_dev_macsec_register_disable(dev);

	return 0;
}

int
rte_pmd_yusur2_macsec_config_txsc(uint16_t port, uint8_t *mac)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	uint32_t ctrl;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	ctrl = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXSCL, ctrl);

	ctrl = mac[4] | (mac[5] << 8);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXSCH, ctrl);

	return 0;
}

int
rte_pmd_yusur2_macsec_config_rxsc(uint16_t port, uint8_t *mac, uint16_t pi)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	uint32_t ctrl;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	ctrl = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXSCL, ctrl);

	pi = rte_cpu_to_be_16(pi);
	ctrl = mac[4] | (mac[5] << 8) | (pi << 16);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXSCH, ctrl);

	return 0;
}

int
rte_pmd_yusur2_macsec_select_txsa(uint16_t port, uint8_t idx, uint8_t an,
				 uint32_t pn, uint8_t *key)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	uint32_t ctrl, i;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (idx != 0 && idx != 1)
		return -EINVAL;

	if (an >= 4)
		return -EINVAL;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Set the PN and key */
	pn = rte_cpu_to_be_32(pn);
	if (idx == 0) {
		YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXPN0, pn);

		for (i = 0; i < 4; i++) {
			ctrl = (key[i * 4 + 0] <<  0) |
			       (key[i * 4 + 1] <<  8) |
			       (key[i * 4 + 2] << 16) |
			       (key[i * 4 + 3] << 24);
			YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXKEY0(i), ctrl);
		}
	} else {
		YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXPN1, pn);

		for (i = 0; i < 4; i++) {
			ctrl = (key[i * 4 + 0] <<  0) |
			       (key[i * 4 + 1] <<  8) |
			       (key[i * 4 + 2] << 16) |
			       (key[i * 4 + 3] << 24);
			YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXKEY1(i), ctrl);
		}
	}

	/* Set AN and select the SA */
	ctrl = (an << idx * 2) | (idx << 4);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECTXSA, ctrl);

	return 0;
}

int
rte_pmd_yusur2_macsec_select_rxsa(uint16_t port, uint8_t idx, uint8_t an,
				 uint32_t pn, uint8_t *key)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	uint32_t ctrl, i;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];

	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (idx != 0 && idx != 1)
		return -EINVAL;

	if (an >= 4)
		return -EINVAL;

	/* Set the PN */
	pn = rte_cpu_to_be_32(pn);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXPN(idx), pn);

	/* Set the key */
	for (i = 0; i < 4; i++) {
		ctrl = (key[i * 4 + 0] <<  0) |
		       (key[i * 4 + 1] <<  8) |
		       (key[i * 4 + 2] << 16) |
		       (key[i * 4 + 3] << 24);
		YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXKEY(idx, i), ctrl);
	}

	/* Set the AN and validate the SA */
	ctrl = an | (1 << 2);
	YUSUR2_WRITE_REG(hw, YUSUR2_LSECRXSA(idx), ctrl);

	return 0;
}

int
rte_pmd_yusur2_upd_fctrl_sbp(uint16_t port, int enable)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	uint32_t fctrl;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);
	dev = &rte_eth_devices[port];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (!hw)
		return -ENOTSUP;

	fctrl = YUSUR2_READ_REG(hw, YUSUR2_FCTRL);

	/* If 'enable' set the SBP bit else clear it */
	if (enable)
		fctrl |= YUSUR2_FCTRL_SBP;
	else
		fctrl &= ~(YUSUR2_FCTRL_SBP);

	YUSUR2_WRITE_REG(hw, YUSUR2_FCTRL, fctrl);
	return 0;
}

#ifdef RTE_LIBRTE_YUSUR2_BYPASS
int
rte_pmd_yusur2_bypass_init(uint16_t port_id)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	yusur2_bypass_init(dev);
	return 0;
}

int
rte_pmd_yusur2_bypass_state_show(uint16_t port_id, uint32_t *state)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_state_show(dev, state);
}

int
rte_pmd_yusur2_bypass_state_set(uint16_t port_id, uint32_t *new_state)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_state_store(dev, new_state);
}

int
rte_pmd_yusur2_bypass_event_show(uint16_t port_id,
				uint32_t event,
				uint32_t *state)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_event_show(dev, event, state);
}

int
rte_pmd_yusur2_bypass_event_store(uint16_t port_id,
				 uint32_t event,
				 uint32_t state)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_event_store(dev, event, state);
}

int
rte_pmd_yusur2_bypass_wd_timeout_store(uint16_t port_id, uint32_t timeout)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_wd_timeout_store(dev, timeout);
}

int
rte_pmd_yusur2_bypass_ver_show(uint16_t port_id, uint32_t *ver)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_ver_show(dev, ver);
}

int
rte_pmd_yusur2_bypass_wd_timeout_show(uint16_t port_id, uint32_t *wd_timeout)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_wd_timeout_show(dev, wd_timeout);
}

int
rte_pmd_yusur2_bypass_wd_reset(uint16_t port_id)
{
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);

	dev = &rte_eth_devices[port_id];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	return yusur2_bypass_wd_reset(dev);
}
#endif

/**
 *  rte_pmd_yusur2_acquire_swfw - Acquire SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SWFW semaphore and get the shared phy token as needed
 */
STATIC s32 rte_pmd_yusur2_acquire_swfw(struct yusur2_hw *hw, u32 mask)
{
	int retries = FW_PHY_TOKEN_RETRIES;
	s32 status = YUSUR2_SUCCESS;
//TODO:
#if 0

	while (--retries) {
		status = yusur2_acquire_swfw_semaphore(hw, mask);
		if (status) {
			PMD_DRV_LOG(ERR, "Get SWFW sem failed, Status = %d\n",
				    status);
			return status;
		}
		status = yusur2_get_phy_token(hw);
		if (status == YUSUR2_SUCCESS)
			return YUSUR2_SUCCESS;

		if (status == YUSUR2_ERR_TOKEN_RETRY)
			PMD_DRV_LOG(ERR, "Get PHY token failed, Status = %d\n",
				    status);

		yusur2_release_swfw_semaphore(hw, mask);
		if (status != YUSUR2_ERR_TOKEN_RETRY) {
			PMD_DRV_LOG(ERR,
				    "Retry get PHY token failed, Status=%d\n",
				    status);
			return status;
		}
	}
	PMD_DRV_LOG(ERR, "swfw acquisition retries failed!: PHY ID = 0x%08X\n",
		    hw->phy.id);
#endif
	return status;
}

/**
 *  rte_pmd_yusur2_release_swfw_sync - Release SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SWFW semaphore and puts the shared phy token as needed
 */
STATIC void rte_pmd_yusur2_release_swfw(struct yusur2_hw *hw, u32 mask)
{
//TODO:
#if 0
	yusur2_put_phy_token(hw);
	yusur2_release_swfw_semaphore(hw, mask);
#endif
}

int
rte_pmd_yusur2_mdio_lock(uint16_t port)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	u32 swfw_mask;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);
	dev = &rte_eth_devices[port];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (!hw)
		return -ENOTSUP;

	if (hw->bus.lan_id)
		swfw_mask = YUSUR2_GSSR_PHY1_SM;
	else
		swfw_mask = YUSUR2_GSSR_PHY0_SM;

	if (rte_pmd_yusur2_acquire_swfw(hw, swfw_mask))
		return YUSUR2_ERR_SWFW_SYNC;

	return YUSUR2_SUCCESS;
}

int
rte_pmd_yusur2_mdio_unlock(uint16_t port)
{
	struct rte_eth_dev *dev;
	struct yusur2_hw *hw;
	u32 swfw_mask;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);

	dev = &rte_eth_devices[port];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (!hw)
		return -ENOTSUP;

	if (hw->bus.lan_id)
		swfw_mask = YUSUR2_GSSR_PHY1_SM;
	else
		swfw_mask = YUSUR2_GSSR_PHY0_SM;

	rte_pmd_yusur2_release_swfw(hw, swfw_mask);

	return YUSUR2_SUCCESS;
}

int
rte_pmd_yusur2_mdio_unlocked_read(uint16_t port, uint32_t reg_addr,
				 uint32_t dev_type, uint16_t *phy_data)
{
	struct yusur2_hw *hw;
	struct rte_eth_dev *dev;
	u32 i, data, command;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);
	dev = &rte_eth_devices[port];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (!hw)
		return -ENOTSUP;

	/* Setup and write the read command */
	command = (reg_addr << YUSUR2_MSCA_DEV_TYPE_SHIFT) |
		  (dev_type << YUSUR2_MSCA_PHY_ADDR_SHIFT) |
		  YUSUR2_MSCA_OLD_PROTOCOL | YUSUR2_MSCA_READ_AUTOINC |
		  YUSUR2_MSCA_MDI_COMMAND;

	YUSUR2_WRITE_REG(hw, YUSUR2_MSCA, command);

	/* Check every 10 usec to see if the access completed.
	 * The MDI Command bit will clear when the operation is
	 * complete
	 */
	for (i = 0; i < YUSUR2_MDIO_COMMAND_TIMEOUT; i++) {
		usec_delay(10);

		command = YUSUR2_READ_REG(hw, YUSUR2_MSCA);
		if (!(command & YUSUR2_MSCA_MDI_COMMAND))
			break;
	}
	if (command & YUSUR2_MSCA_MDI_COMMAND)
		return YUSUR2_ERR_PHY;

	/* Read operation is complete.  Get the data from MSRWD */
	data = YUSUR2_READ_REG(hw, YUSUR2_MSRWD);
	data >>= YUSUR2_MSRWD_READ_DATA_SHIFT;
	*phy_data = (u16)data;

	return 0;
}

int
rte_pmd_yusur2_mdio_unlocked_write(uint16_t port, uint32_t reg_addr,
				  uint32_t dev_type, uint16_t phy_data)
{
	struct yusur2_hw *hw;
	u32 i, command;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port, -ENODEV);
	dev = &rte_eth_devices[port];
	if (!is_yusur2_supported(dev))
		return -ENOTSUP;

	hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (!hw)
		return -ENOTSUP;

	/* Put the data in the MDI single read and write data register*/
	YUSUR2_WRITE_REG(hw, YUSUR2_MSRWD, (u32)phy_data);

	/* Setup and write the write command */
	command = (reg_addr << YUSUR2_MSCA_DEV_TYPE_SHIFT) |
		  (dev_type << YUSUR2_MSCA_PHY_ADDR_SHIFT) |
		  YUSUR2_MSCA_OLD_PROTOCOL | YUSUR2_MSCA_WRITE |
		  YUSUR2_MSCA_MDI_COMMAND;

	YUSUR2_WRITE_REG(hw, YUSUR2_MSCA, command);

	/* Check every 10 usec to see if the access completed.
	 * The MDI Command bit will clear when the operation is
	 * complete
	 */
	for (i = 0; i < YUSUR2_MDIO_COMMAND_TIMEOUT; i++) {
		usec_delay(10);

		command = YUSUR2_READ_REG(hw, YUSUR2_MSCA);
		if (!(command & YUSUR2_MSCA_MDI_COMMAND))
			break;
	}
	if (command & YUSUR2_MSCA_MDI_COMMAND) {
		ERROR_REPORT1(YUSUR2_ERROR_POLLING,
			      "PHY write cmd didn't complete\n");
		return YUSUR2_ERR_PHY;
	}
	return 0;
}
