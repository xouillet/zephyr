/*
 * Copyright (c) 2021, ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief GigaDevice RISC-V MCU series initialization code
 */

#include <device.h>
#include <init.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int gigadevice_riscv_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	SystemInit();

	irq_unlock(key);

	return 0;
}

SYS_INIT(gigadevice_riscv_init, PRE_KERNEL_1, 0);
