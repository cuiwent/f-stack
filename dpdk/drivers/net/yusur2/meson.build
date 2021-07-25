# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2018-2021 Yusur

cflags += ['-DRTE_LIBRTE_YUSUR2_BYPASS']

allow_experimental_apis = true

subdir('base')
objs = [base_objs]

sources = files(
	'yusur2_bypass.c',
	'yusur2_ethdev.c',
	'yusur2_pf.c',
	'yusur2_rxtx.c',
	'yusur2_tm.c',
	'rte_pmd_yusur2.c'
)

deps += ['hash', 'security']

if arch_subdir == 'x86'
	dpdk_conf.set('RTE_YUSUR2_INC_VECTOR', 1)
	sources += files('yusur2_rxtx_vec_sse.c')
endif

includes += include_directories('base')

install_headers('rte_pmd_yusur2.h')
