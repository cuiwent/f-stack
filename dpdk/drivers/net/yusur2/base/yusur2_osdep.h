/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018-2021 Yusur
 */

#ifndef _YUSUR2_OS_H_
#define _YUSUR2_OS_H_

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <rte_common.h>
#include <rte_debug.h>
#include <rte_cycles.h>
#include <rte_log.h>
#include <rte_byteorder.h>
#include <rte_io.h>

#include "../yusur2_logs.h"
#include "../yusur2_bypass_defines.h"

#define ASSERT(x) if(!(x)) rte_panic("YUSUR2: x")

#define DELAY(x) rte_delay_us_sleep(x)
#define usec_delay(x) DELAY(x)
#define msec_delay(x) DELAY(1000*(x))

#define DEBUGFUNC(F)            DEBUGOUT(F "\n");
#define DEBUGOUT(S, args...)    PMD_DRV_LOG_RAW(DEBUG, S, ##args)
#define DEBUGOUT1(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT2(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT3(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT6(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT7(S, args...)   DEBUGOUT(S, ##args)

#define ERROR_REPORT1(e, S, args...)   DEBUGOUT(S, ##args)
#define ERROR_REPORT2(e, S, args...)   DEBUGOUT(S, ##args)
#define ERROR_REPORT3(e, S, args...)   DEBUGOUT(S, ##args)

#define FALSE               0
#define TRUE                1

#define false               0
#define true                1
#define min(a,b)	RTE_MIN(a,b)

#define EWARN(hw, S, args...)     DEBUGOUT1(S, ##args)

/* Bunch of defines for shared code bogosity */
#define UNREFERENCED_PARAMETER(_p)
#define UNREFERENCED_1PARAMETER(_p)
#define UNREFERENCED_2PARAMETER(_p, _q)
#define UNREFERENCED_3PARAMETER(_p, _q, _r)
#define UNREFERENCED_4PARAMETER(_p, _q, _r, _s)
#define UNREFERENCED_5PARAMETER(_p, _q, _r, _s, _t)

/* Shared code error reporting */
enum {
	YUSUR2_ERROR_SOFTWARE,
	YUSUR2_ERROR_POLLING,
	YUSUR2_ERROR_INVALID_STATE,
	YUSUR2_ERROR_UNSUPPORTED,
	YUSUR2_ERROR_ARGUMENT,
	YUSUR2_ERROR_CAUTION,
};

#define STATIC static
#define YUSUR2_NTOHL(_i)	rte_be_to_cpu_32(_i)
#define YUSUR2_NTOHS(_i)	rte_be_to_cpu_16(_i)
#define YUSUR2_CPU_TO_LE16(_i)  rte_cpu_to_le_16(_i)
#define YUSUR2_CPU_TO_LE32(_i)  rte_cpu_to_le_32(_i)
#define YUSUR2_LE32_TO_CPU(_i)  rte_le_to_cpu_32(_i)
#define YUSUR2_LE32_TO_CPUS(_i) rte_le_to_cpu_32(_i)
#define YUSUR2_CPU_TO_BE16(_i)  rte_cpu_to_be_16(_i)
#define YUSUR2_CPU_TO_BE32(_i)  rte_cpu_to_be_32(_i)
#define YUSUR2_BE32_TO_CPU(_i)  rte_be_to_cpu_32(_i)

typedef uint8_t		u8;
typedef int8_t		s8;
typedef uint16_t	u16;
typedef int16_t		s16;
typedef uint32_t	u32;
typedef int32_t		s32;
typedef uint64_t	u64;
#ifndef __cplusplus
typedef int		bool;
#endif

#define mb()	rte_mb()
#define wmb()	rte_wmb()
#define rmb()	rte_rmb()

#define IOMEM

#define prefetch(x) rte_prefetch0(x)

#define YUSUR2_PCI_REG(reg) rte_read32(reg)

static inline uint32_t yusur2_read_addr(volatile void* addr)
{
	return rte_le_to_cpu_32(YUSUR2_PCI_REG(addr));
}

#define YUSUR2_PCI_REG_WRITE(reg, value)			\
	rte_write32((rte_cpu_to_le_32(value)), reg)

#define YUSUR2_PCI_REG_WRITE_RELAXED(reg, value)		\
	rte_write32_relaxed((rte_cpu_to_le_32(value)), reg)

#define YUSUR2_PCI_REG_ADDR(hw, reg) \
	((volatile uint32_t *)((char *)(hw)->hw_addr + (reg)))

#define YUSUR2_PCI_REG_ARRAY_ADDR(hw, reg, index) \
	YUSUR2_PCI_REG_ADDR((hw), (reg) + ((index) << 2))

/* Not implemented !! */
#define YUSUR2_READ_PCIE_WORD(hw, reg) 0
#define YUSUR2_WRITE_PCIE_WORD(hw, reg, value) do { } while(0)

#define YUSUR2_WRITE_FLUSH(a) YUSUR2_READ_REG(a, YUSUR2_STATUS)

#define YUSUR2_READ_REG(hw, reg) \
	yusur2_read_addr(YUSUR2_PCI_REG_ADDR((hw), (reg)))

#define YUSUR2_WRITE_REG(hw, reg, value) \
	YUSUR2_PCI_REG_WRITE(YUSUR2_PCI_REG_ADDR((hw), (reg)), (value))

#define YUSUR2_READ_REG_ARRAY(hw, reg, index) \
	YUSUR2_PCI_REG(YUSUR2_PCI_REG_ARRAY_ADDR((hw), (reg), (index)))

#define YUSUR2_WRITE_REG_ARRAY(hw, reg, index, value) \
	YUSUR2_PCI_REG_WRITE(YUSUR2_PCI_REG_ARRAY_ADDR((hw), (reg), (index)), (value))

#define YUSUR2_WRITE_REG_THEN_POLL_MASK(hw, reg, val, mask, poll_ms)	\
do {									\
	uint32_t cnt = poll_ms;						\
	YUSUR2_WRITE_REG(hw, (reg), (val));				\
	while (((YUSUR2_READ_REG(hw, (reg))) & (mask)) && (cnt--))	\
		rte_delay_ms(1);					\
} while (0)

#endif /* _YUSUR2_OS_H_ */
