# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2018-2021 Yusur

sources = [
	'yusur2_82598.c',
	'yusur2_82599.c',
	'yusur2_api.c',
	'yusur2_common.c',
	'yusur2_dcb_82598.c',
	'yusur2_dcb_82599.c',
	'yusur2_dcb.c',
	'yusur2_hv_vf.c',
	'yusur2_mbx.c',
	'yusur2_phy.c',
	'yusur2_vf.c',
	'yusur2_x540.c',
	'yusur2_x550.c'
]

error_cflags = ['-Wno-unused-value',
		'-Wno-unused-but-set-variable',
		'-Wno-unused-parameter',
		]
c_args = cflags
if allow_experimental_apis
	c_args += '-DALLOW_EXPERIMENTAL_API'
endif
foreach flag: error_cflags
	if cc.has_argument(flag)
		c_args += flag
	endif
endforeach

base_lib = static_library('yusur2_base', sources,
	dependencies: static_rte_eal,
	c_args: c_args)
base_objs = base_lib.extract_all_objects()
