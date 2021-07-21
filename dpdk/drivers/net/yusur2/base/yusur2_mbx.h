/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_MBX_H_
#define _YUSUR2_MBX_H_

#include "yusur2_type.h"

#define YUSUR2_VFMAILBOX_SIZE	16 /* 16 32 bit words - 64 bytes */
#define YUSUR2_ERR_MBX		-100

#define YUSUR2_VFMAILBOX		0x002FC
#define YUSUR2_VFMBMEM		0x00200

/* Define mailbox register bits */
#define YUSUR2_VFMAILBOX_REQ	0x00000001 /* Request for PF Ready bit */
#define YUSUR2_VFMAILBOX_ACK	0x00000002 /* Ack PF message received */
#define YUSUR2_VFMAILBOX_VFU	0x00000004 /* VF owns the mailbox buffer */
#define YUSUR2_VFMAILBOX_PFU	0x00000008 /* PF owns the mailbox buffer */
#define YUSUR2_VFMAILBOX_PFSTS	0x00000010 /* PF wrote a message in the MB */
#define YUSUR2_VFMAILBOX_PFACK	0x00000020 /* PF ack the previous VF msg */
#define YUSUR2_VFMAILBOX_RSTI	0x00000040 /* PF has reset indication */
#define YUSUR2_VFMAILBOX_RSTD	0x00000080 /* PF has indicated reset done */
#define YUSUR2_VFMAILBOX_R2C_BITS	0x000000B0 /* All read to clear bits */

#define YUSUR2_PFMAILBOX_STS	0x00000001 /* Initiate message send to VF */
#define YUSUR2_PFMAILBOX_ACK	0x00000002 /* Ack message recv'd from VF */
#define YUSUR2_PFMAILBOX_VFU	0x00000004 /* VF owns the mailbox buffer */
#define YUSUR2_PFMAILBOX_PFU	0x00000008 /* PF owns the mailbox buffer */
#define YUSUR2_PFMAILBOX_RVFU	0x00000010 /* Reset VFU - used when VF stuck */

#define YUSUR2_MBVFICR_VFREQ_MASK	0x0000FFFF /* bits for VF messages */
#define YUSUR2_MBVFICR_VFREQ_VF1		0x00000001 /* bit for VF 1 message */
#define YUSUR2_MBVFICR_VFACK_MASK	0xFFFF0000 /* bits for VF acks */
#define YUSUR2_MBVFICR_VFACK_VF1		0x00010000 /* bit for VF 1 ack */


/* If it's a YUSUR2_VF_* msg then it originates in the VF and is sent to the
 * PF.  The reverse is true if it is YUSUR2_PF_*.
 * Message ACK's are the value or'd with 0xF0000000
 */
#define YUSUR2_VT_MSGTYPE_ACK	0x80000000 /* Messages below or'd with
					    * this are the ACK */
#define YUSUR2_VT_MSGTYPE_NACK	0x40000000 /* Messages below or'd with
					    * this are the NACK */
#define YUSUR2_VT_MSGTYPE_CTS	0x20000000 /* Indicates that VF is still
					    * clear to send requests */
#define YUSUR2_VT_MSGINFO_SHIFT	16
/* bits 23:16 are used for extra info for certain messages */
#define YUSUR2_VT_MSGINFO_MASK	(0xFF << YUSUR2_VT_MSGINFO_SHIFT)

/* definitions to support mailbox API version negotiation */

/*
 * each element denotes a version of the API; existing numbers may not
 * change; any additions must go at the end
 */
enum yusur2_pfvf_api_rev {
	yusur2_mbox_api_10,	/* API version 1.0, linux/freebsd VF driver */
	yusur2_mbox_api_20,	/* API version 2.0, solaris Phase1 VF driver */
	yusur2_mbox_api_11,	/* API version 1.1, linux/freebsd VF driver */
	yusur2_mbox_api_12,	/* API version 1.2, linux/freebsd VF driver */
	yusur2_mbox_api_13,	/* API version 1.3, linux/freebsd VF driver */
	/* This value should always be last */
	yusur2_mbox_api_unknown,	/* indicates that API version is not known */
};

/* mailbox API, legacy requests */
#define YUSUR2_VF_RESET		0x01 /* VF requests reset */
#define YUSUR2_VF_SET_MAC_ADDR	0x02 /* VF requests PF to set MAC addr */
#define YUSUR2_VF_SET_MULTICAST	0x03 /* VF requests PF to set MC addr */
#define YUSUR2_VF_SET_VLAN	0x04 /* VF requests PF to set VLAN */

/* mailbox API, version 1.0 VF requests */
#define YUSUR2_VF_SET_LPE	0x05 /* VF requests PF to set VMOLR.LPE */
#define YUSUR2_VF_SET_MACVLAN	0x06 /* VF requests PF for unicast filter */
#define YUSUR2_VF_API_NEGOTIATE	0x08 /* negotiate API version */

/* mailbox API, version 1.1 VF requests */
#define YUSUR2_VF_GET_QUEUES	0x09 /* get queue configuration */

/* mailbox API, version 1.2 VF requests */
#define YUSUR2_VF_GET_RETA      0x0a    /* VF request for RETA */
#define YUSUR2_VF_GET_RSS_KEY	0x0b    /* get RSS key */
#define YUSUR2_VF_UPDATE_XCAST_MODE	0x0c

/* mode choices for YUSUR2_VF_UPDATE_XCAST_MODE */
enum yusur2vf_xcast_modes {
	YUSUR2VF_XCAST_MODE_NONE = 0,
	YUSUR2VF_XCAST_MODE_MULTI,
	YUSUR2VF_XCAST_MODE_ALLMULTI,
	YUSUR2VF_XCAST_MODE_PROMISC,
};

/* GET_QUEUES return data indices within the mailbox */
#define YUSUR2_VF_TX_QUEUES	1	/* number of Tx queues supported */
#define YUSUR2_VF_RX_QUEUES	2	/* number of Rx queues supported */
#define YUSUR2_VF_TRANS_VLAN	3	/* Indication of port vlan */
#define YUSUR2_VF_DEF_QUEUE	4	/* Default queue offset */

/* length of permanent address message returned from PF */
#define YUSUR2_VF_PERMADDR_MSG_LEN	4
/* word in permanent address message with the current multicast type */
#define YUSUR2_VF_MC_TYPE_WORD		3

#define YUSUR2_PF_CONTROL_MSG		0x0100 /* PF control message */

/* mailbox API, version 2.0 VF requests */
#define YUSUR2_VF_API_NEGOTIATE		0x08 /* negotiate API version */
#define YUSUR2_VF_GET_QUEUES		0x09 /* get queue configuration */
#define YUSUR2_VF_ENABLE_MACADDR		0x0A /* enable MAC address */
#define YUSUR2_VF_DISABLE_MACADDR	0x0B /* disable MAC address */
#define YUSUR2_VF_GET_MACADDRS		0x0C /* get all configured MAC addrs */
#define YUSUR2_VF_SET_MCAST_PROMISC	0x0D /* enable multicast promiscuous */
#define YUSUR2_VF_GET_MTU		0x0E /* get bounds on MTU */
#define YUSUR2_VF_SET_MTU		0x0F /* set a specific MTU */

/* mailbox API, version 2.0 PF requests */
#define YUSUR2_PF_TRANSPARENT_VLAN	0x0101 /* enable transparent vlan */

#define YUSUR2_VF_MBX_INIT_TIMEOUT	2000 /* number of retries on mailbox */
#define YUSUR2_VF_MBX_INIT_DELAY		500  /* microseconds between retries */

s32 yusur2_read_mbx(struct yusur2_hw *, u32 *, u16, u16);
s32 yusur2_write_mbx(struct yusur2_hw *, u32 *, u16, u16);
s32 yusur2_read_posted_mbx(struct yusur2_hw *, u32 *, u16, u16);
s32 yusur2_write_posted_mbx(struct yusur2_hw *, u32 *, u16, u16);
s32 yusur2_check_for_msg(struct yusur2_hw *, u16);
s32 yusur2_check_for_ack(struct yusur2_hw *, u16);
s32 yusur2_check_for_rst(struct yusur2_hw *, u16);
void yusur2_init_mbx_ops_generic(struct yusur2_hw *hw);
void yusur2_init_mbx_params_vf(struct yusur2_hw *);
void yusur2_init_mbx_params_pf(struct yusur2_hw *);

#endif /* _YUSUR2_MBX_H_ */
