/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2015 Intel Corporation
 */
#ifndef _YUSUR2_REGS_H_
#define _YUSUR2_REGS_H_

#include "yusur2_ethdev.h"

struct yusur2_hw;
struct reg_info {
	uint32_t base_addr;
	uint32_t count;
	uint32_t stride;
	const char *name;
};

static const struct reg_info yusur2_regs_general[] = {
	{YUSUR2_CTRL, 1, 1, "YUSUR2_CTRL"},
	{YUSUR2_STATUS, 1, 1, "YUSUR2_STATUS"},
	{YUSUR2_CTRL_EXT, 1, 1, "YUSUR2_CTRL_EXT"},
	{YUSUR2_ESDP, 1, 1, "YUSUR2_ESDP"},
	{YUSUR2_EODSDP, 1, 1, "YUSUR2_EODSDP"},
	{YUSUR2_LEDCTL, 1, 1, "YUSUR2_LEDCTL"},
	{YUSUR2_FRTIMER, 1, 1, "YUSUR2_FRTIMER"},
	{YUSUR2_TCPTIMER, 1, 1, "YUSUR2_TCPTIMER"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_nvm[] = {
	{YUSUR2_EEC, 1, 1, "YUSUR2_EEC"},
	{YUSUR2_EERD, 1, 1, "YUSUR2_EERD"},
	{YUSUR2_FLA, 1, 1, "YUSUR2_FLA"},
	{YUSUR2_EEMNGCTL, 1, 1, "YUSUR2_EEMNGCTL"},
	{YUSUR2_EEMNGDATA, 1, 1, "YUSUR2_EEMNGDATA"},
	{YUSUR2_FLMNGCTL, 1, 1, "YUSUR2_FLMNGCTL"},
	{YUSUR2_FLMNGDATA, 1, 1, "YUSUR2_FLMNGDATA"},
	{YUSUR2_FLMNGCNT, 1, 1, "YUSUR2_FLMNGCNT"},
	{YUSUR2_FLOP, 1, 1, "YUSUR2_FLOP"},
	{YUSUR2_GRC,  1, 1, "YUSUR2_GRC"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_interrupt[] = {
	{YUSUR2_EICS, 1, 1, "YUSUR2_EICS"},
	{YUSUR2_EIMS, 1, 1, "YUSUR2_EIMS"},
	{YUSUR2_EIMC, 1, 1, "YUSUR2_EIMC"},
	{YUSUR2_EIAC, 1, 1, "YUSUR2_EIAC"},
	{YUSUR2_EIAM, 1, 1, "YUSUR2_EIAM"},
	{YUSUR2_EITR(0), 24, 4, "YUSUR2_EITR"},
	{YUSUR2_IVAR(0), 24, 4, "YUSUR2_IVAR"},
	{YUSUR2_MSIXT, 1, 1, "YUSUR2_MSIXT"},
	{YUSUR2_MSIXPBA, 1, 1, "YUSUR2_MSIXPBA"},
	{YUSUR2_PBACL(0),  1, 4, "YUSUR2_PBACL"},
	{YUSUR2_GPIE, 1, 1, ""},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_fctl_mac_82598EB[] = {
	{YUSUR2_PFCTOP, 1, 1, ""},
	{YUSUR2_FCTTV(0), 4, 4, ""},
	{YUSUR2_FCRTV, 1, 1, ""},
	{YUSUR2_TFCS, 1, 1, ""},
	{YUSUR2_FCRTL(0), 8, 8, "YUSUR2_FCRTL"},
	{YUSUR2_FCRTH(0), 8, 8, "YUSUR2_FCRTH"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_fctl_others[] = {
	{YUSUR2_PFCTOP, 1, 1, ""},
	{YUSUR2_FCTTV(0), 4, 4, ""},
	{YUSUR2_FCRTV, 1, 1, ""},
	{YUSUR2_TFCS, 1, 1, ""},
	{YUSUR2_FCRTL_82599(0), 8, 4, "YUSUR2_FCRTL"},
	{YUSUR2_FCRTH_82599(0), 8, 4, "YUSUR2_FCRTH"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_rxdma[] = {
	{YUSUR2_RDBAL(0), 64, 0x40, "YUSUR2_RDBAL"},
	{YUSUR2_RDBAH(0), 64, 0x40, "YUSUR2_RDBAH"},
	{YUSUR2_RDLEN(0), 64, 0x40, "YUSUR2_RDLEN"},
	{YUSUR2_RDH(0), 64, 0x40, "YUSUR2_RDH"},
	{YUSUR2_RDT(0), 64, 0x40, "YUSUR2_RDT"},
	{YUSUR2_RXDCTL(0), 64, 0x40, "YUSUR2_RXDCTL"},
	{YUSUR2_SRRCTL(0), 16, 0x4, "YUSUR2_SRRCTL"},
	{YUSUR2_DCA_RXCTRL(0), 16, 4, "YUSUR2_DCA_RXCTRL"},
	{YUSUR2_RDRXCTL, 1, 1, "YUSUR2_RDRXCTL"},
	{YUSUR2_RXPBSIZE(0), 8, 4, "YUSUR2_RXPBSIZE"},
	{YUSUR2_RXCTRL, 1, 1, "YUSUR2_RXCTRL"},
	{YUSUR2_DROPEN, 1, 1, "YUSUR2_DROPEN"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_rx[] = {
	{YUSUR2_RXCSUM, 1, 1, "YUSUR2_RXCSUM"},
	{YUSUR2_RFCTL, 1, 1, "YUSUR2_RFCTL"},
	{YUSUR2_RAL(0), 16, 8, "YUSUR2_RAL"},
	{YUSUR2_RAH(0), 16, 8, "YUSUR2_RAH"},
	{YUSUR2_PSRTYPE(0), 1, 4, "YUSUR2_PSRTYPE"},
	{YUSUR2_FCTRL, 1, 1, "YUSUR2_FCTRL"},
	{YUSUR2_VLNCTRL, 1, 1, "YUSUR2_VLNCTRL"},
	{YUSUR2_MCSTCTRL, 1, 1, "YUSUR2_MCSTCTRL"},
	{YUSUR2_MRQC, 1, 1, "YUSUR2_MRQC"},
	{YUSUR2_VMD_CTL, 1, 1, "YUSUR2_VMD_CTL"},
	{YUSUR2_IMIR(0), 8, 4, "YUSUR2_IMIR"},
	{YUSUR2_IMIREXT(0), 8, 4, "YUSUR2_IMIREXT"},
	{YUSUR2_IMIRVP, 1, 1, "YUSUR2_IMIRVP"},
	{0, 0, 0, ""}
};

static struct reg_info yusur2_regs_tx[] = {
	{YUSUR2_TDBAL(0), 32, 0x40, "YUSUR2_TDBAL"},
	{YUSUR2_TDBAH(0), 32, 0x40, "YUSUR2_TDBAH"},
	{YUSUR2_TDLEN(0), 32, 0x40, "YUSUR2_TDLEN"},
	{YUSUR2_TDH(0), 32, 0x40, "YUSUR2_TDH"},
	{YUSUR2_TDT(0), 32, 0x40, "YUSUR2_TDT"},
	{YUSUR2_TXDCTL(0), 32, 0x40, "YUSUR2_TXDCTL"},
	{YUSUR2_TDWBAL(0), 32, 0x40, "YUSUR2_TDWBAL"},
	{YUSUR2_TDWBAH(0), 32, 0x40, "YUSUR2_TDWBAH"},
	{YUSUR2_DTXCTL, 1, 1, "YUSUR2_DTXCTL"},
	{YUSUR2_DCA_TXCTRL(0), 16, 4, "YUSUR2_DCA_TXCTRL"},
	{YUSUR2_TXPBSIZE(0), 8, 4, "YUSUR2_TXPBSIZE"},
	{YUSUR2_MNGTXMAP, 1, 1, "YUSUR2_MNGTXMAP"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_wakeup[] = {
	{YUSUR2_WUC, 1, 1, "YUSUR2_WUC"},
	{YUSUR2_WUFC, 1, 1, "YUSUR2_WUFC"},
	{YUSUR2_WUS, 1, 1, "YUSUR2_WUS"},
	{YUSUR2_IPAV, 1, 1, "YUSUR2_IPAV"},
	{YUSUR2_IP4AT, 1, 1, "YUSUR2_IP4AT"},
	{YUSUR2_IP6AT, 1, 1, "YUSUR2_IP6AT"},
	{YUSUR2_WUPL, 1, 1, "YUSUR2_WUPL"},
	{YUSUR2_WUPM, 1, 1, "YUSUR2_WUPM"},
	{YUSUR2_FHFT(0), 1, 1, "YUSUR2_FHFT"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_dcb[] = {
	{YUSUR2_RMCS, 1, 1, "YUSUR2_RMCS"},
	{YUSUR2_DPMCS, 1, 1, "YUSUR2_DPMCS"},
	{YUSUR2_PDPMCS, 1, 1, "YUSUR2_PDPMCS"},
	{YUSUR2_RUPPBMR, 1, 1, "YUSUR2_RUPPBMR"},
	{YUSUR2_RT2CR(0), 8, 4, "YUSUR2_RT2CR"},
	{YUSUR2_RT2SR(0), 8, 4, "YUSUR2_RT2SR"},
	{YUSUR2_TDTQ2TCCR(0), 8, 0x40, "YUSUR2_TDTQ2TCCR"},
	{YUSUR2_TDTQ2TCSR(0), 8, 0x40, "YUSUR2_TDTQ2TCSR"},
	{YUSUR2_TDPT2TCCR(0), 8, 4, "YUSUR2_TDPT2TCCR"},
	{YUSUR2_TDPT2TCSR(0), 8, 4, "YUSUR2_TDPT2TCSR"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_mac[] = {
	{YUSUR2_PCS1GCFIG, 1, 1, "YUSUR2_PCS1GCFIG"},
	{YUSUR2_PCS1GLCTL, 1, 1, "YUSUR2_PCS1GLCTL"},
	{YUSUR2_PCS1GLSTA, 1, 1, "YUSUR2_PCS1GLSTA"},
	{YUSUR2_PCS1GDBG0, 1, 1, "YUSUR2_PCS1GDBG0"},
	{YUSUR2_PCS1GDBG1, 1, 1, "YUSUR2_PCS1GDBG1"},
	{YUSUR2_PCS1GANA, 1, 1, "YUSUR2_PCS1GANA"},
	{YUSUR2_PCS1GANLP, 1, 1, "YUSUR2_PCS1GANLP"},
	{YUSUR2_PCS1GANNP, 1, 1, "YUSUR2_PCS1GANNP"},
	{YUSUR2_PCS1GANLPNP, 1, 1, "YUSUR2_PCS1GANLPNP"},
	{YUSUR2_HLREG0, 1, 1, "YUSUR2_HLREG0"},
	{YUSUR2_HLREG1, 1, 1, "YUSUR2_HLREG1"},
	{YUSUR2_PAP, 1, 1, "YUSUR2_PAP"},
	{YUSUR2_MACA, 1, 1, "YUSUR2_MACA"},
	{YUSUR2_APAE, 1, 1, "YUSUR2_APAE"},
	{YUSUR2_ARD, 1, 1, "YUSUR2_ARD"},
	{YUSUR2_AIS, 1, 1, "YUSUR2_AIS"},
	{YUSUR2_MSCA, 1, 1, "YUSUR2_MSCA"},
	{YUSUR2_MSRWD, 1, 1, "YUSUR2_MSRWD"},
	{YUSUR2_MLADD, 1, 1, "YUSUR2_MLADD"},
	{YUSUR2_MHADD, 1, 1, "YUSUR2_MHADD"},
	{YUSUR2_TREG, 1, 1, "YUSUR2_TREG"},
	{YUSUR2_PCSS1, 1, 1, "YUSUR2_PCSS1"},
	{YUSUR2_PCSS2, 1, 1, "YUSUR2_PCSS2"},
	{YUSUR2_XPCSS, 1, 1, "YUSUR2_XPCSS"},
	{YUSUR2_SERDESC, 1, 1, "YUSUR2_SERDESC"},
	{YUSUR2_MACS, 1, 1, "YUSUR2_MACS"},
	{YUSUR2_AUTOC, 1, 1, "YUSUR2_AUTOC"},
	{YUSUR2_LINKS, 1, 1, "YUSUR2_LINKS"},
	{YUSUR2_AUTOC2, 1, 1, "YUSUR2_AUTOC2"},
	{YUSUR2_AUTOC3, 1, 1, "YUSUR2_AUTOC3"},
	{YUSUR2_ANLP1, 1, 1, "YUSUR2_ANLP1"},
	{YUSUR2_ANLP2, 1, 1, "YUSUR2_ANLP2"},
	{YUSUR2_ATLASCTL, 1, 1, "YUSUR2_ATLASCTL"},
	{0, 0, 0, ""}
};

static const struct reg_info yusur2_regs_diagnostic[] = {
	{YUSUR2_RDSTATCTL, 1, 1, "YUSUR2_RDSTATCTL"},
	{YUSUR2_RDSTAT(0), 8, 4, "YUSUR2_RDSTAT"},
	{YUSUR2_RDHMPN, 1, 1, "YUSUR2_RDHMPN"},
	{YUSUR2_RIC_DW(0), 4, 4, "YUSUR2_RIC_DW"},
	{YUSUR2_RDPROBE, 1, 1, "YUSUR2_RDPROBE"},
	{YUSUR2_TDHMPN, 1, 1, "YUSUR2_TDHMPN"},
	{YUSUR2_TIC_DW(0), 4, 4, "YUSUR2_TIC_DW"},
	{YUSUR2_TDPROBE, 1, 1, "YUSUR2_TDPROBE"},
	{YUSUR2_TXBUFCTRL, 1, 1, "YUSUR2_TXBUFCTRL"},
	{YUSUR2_TXBUFDATA0, 1, 1, "YUSUR2_TXBUFDATA0"},
	{YUSUR2_TXBUFDATA1, 1, 1, "YUSUR2_TXBUFDATA1"},
	{YUSUR2_TXBUFDATA2, 1, 1, "YUSUR2_TXBUFDATA2"},
	{YUSUR2_TXBUFDATA3, 1, 1, "YUSUR2_TXBUFDATA3"},
	{YUSUR2_RXBUFCTRL, 1, 1, "YUSUR2_RXBUFCTRL"},
	{YUSUR2_RXBUFDATA0, 1, 1, "YUSUR2_RXBUFDATA0"},
	{YUSUR2_RXBUFDATA1, 1, 1, "YUSUR2_RXBUFDATA1"},
	{YUSUR2_RXBUFDATA2, 1, 1, "YUSUR2_RXBUFDATA2"},
	{YUSUR2_RXBUFDATA3, 1, 1, "YUSUR2_RXBUFDATA3"},
	{YUSUR2_PCIE_DIAG(0), 8, 4, ""},
	{YUSUR2_RFVAL, 1, 1, "YUSUR2_RFVAL"},
	{YUSUR2_MDFTC1, 1, 1, "YUSUR2_MDFTC1"},
	{YUSUR2_MDFTC2, 1, 1, "YUSUR2_MDFTC2"},
	{YUSUR2_MDFTFIFO1, 1, 1, "YUSUR2_MDFTFIFO1"},
	{YUSUR2_MDFTFIFO2, 1, 1, "YUSUR2_MDFTFIFO2"},
	{YUSUR2_MDFTS, 1, 1, "YUSUR2_MDFTS"},
	{YUSUR2_PCIEECCCTL, 1, 1, "YUSUR2_PCIEECCCTL"},
	{YUSUR2_PBTXECC, 1, 1, "YUSUR2_PBTXECC"},
	{YUSUR2_PBRXECC, 1, 1, "YUSUR2_PBRXECC"},
	{YUSUR2_MFLCN, 1, 1, "YUSUR2_MFLCN"},
	{0, 0, 0, ""},
};

/* PF registers */
static const struct reg_info *yusur2_regs_others[] = {
				yusur2_regs_general,
				yusur2_regs_nvm, yusur2_regs_interrupt,
				yusur2_regs_fctl_others,
				yusur2_regs_rxdma,
				yusur2_regs_rx,
				yusur2_regs_tx,
				yusur2_regs_wakeup,
				yusur2_regs_dcb,
				yusur2_regs_mac,
				yusur2_regs_diagnostic,
				NULL};

static inline int
yusur2_read_regs(struct yusur2_hw *hw, const struct reg_info *reg,
	uint32_t *reg_buf)
{
	unsigned int i;

	for (i = 0; i < reg->count; i++)
		reg_buf[i] = YUSUR2_READ_REG(hw,
					reg->base_addr + i * reg->stride);
	return reg->count;
};

static inline int
yusur2_regs_group_count(const struct reg_info *regs)
{
	int count = 0;
	int i = 0;

	while (regs[i].count)
		count += regs[i++].count;
	return count;
};

static inline int
yusur2_read_regs_group(struct rte_eth_dev *dev, uint32_t *reg_buf,
					  const struct reg_info *regs)
{
	int count = 0;
	int i = 0;
	struct yusur2_hw *hw = YUSUR2_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	while (regs[i].count)
		count += yusur2_read_regs(hw, &regs[i++], &reg_buf[count]);
	return count;
};

#endif /* _YUSUR2_REGS_H_ */
