/*
 * Copyright (c) 2018, Cypress
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAM4S MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Atmel SAM4S series processor.
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
static int gigadevice_f403_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	SystemInit();

	/*
	 * Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise.
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(gigadevice_f403_init, PRE_KERNEL_1, 0);
