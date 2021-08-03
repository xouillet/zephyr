/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#include <autoconf.h>
#include <linker/linker-tool.h>
#include <arch/riscv/common/linker.ld>

#ifdef CONFIG_RISCV_SOC_INIT_GP_VALUE
SECTIONS
{
	SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
	{
		. = ALIGN(8);
		PROVIDE (__global_pointer$ = __data_ram_start + 0x800);
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
}
#endif
