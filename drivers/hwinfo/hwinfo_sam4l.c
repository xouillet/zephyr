/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/hwinfo.h>
#include <init.h>
#include <soc.h>
#include <string.h>

#define MAX_SIZE_UID    15
#define UID_ADDR        0x0080020C

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	size_t len = (length > MAX_SIZE_UID) ? MAX_SIZE_UID : length;
	uint8_t *flash = (uint8_t *) UID_ADDR;
	int i;

	/* Copy the 120-bit unique ID. We cannot use memcpy as it would
	 * execute code from flash.
	 */
	for (i = 0; i < len; i++) {
		buffer[i] = flash[i];
	}

	return len;
}
