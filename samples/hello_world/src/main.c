/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <dfu/mcuboot.h>

#define LOG_LEVEL CONFIG_LOG_LEVEL_INFO
#include <logging/log.h>
LOG_MODULE_REGISTER(hello_world);

void main(void)
{
	LOG_INF("Hello MCUboot World!");

	if (boot_write_img_confirmed() < 0) {
		LOG_ERR("Error to confirm the image");
	}

	if (boot_is_img_confirmed()) {
		LOG_INF("Everything is working.");
	}
}
