/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

KEEP(*(SORT_NONE( EXCLUDE_FILE( *bl602_glb.o	\
				*bl602_pds.o	\
				*bl602_common.o \
				*bl602_sf_cfg.o \
				*bl602_sf_ctrl.o \
				*bl602_sflash.o \
				*bl602_xip_sflash.o \
				*bl602_ef_ctrl.o) .rodata*)))
