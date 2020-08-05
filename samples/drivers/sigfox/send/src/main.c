/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/sigfox.h>
#include <drivers/gpio.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>

#include <sigfox_types.h>
#include <sigfox_api.h>
#include <sigfox_zephyr.h>

#include <main.h>

BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default Sigfox radio specified in DT");

#include <logging/log.h>
LOG_MODULE_REGISTER(sigfox_send, CONFIG_SIGFOX_SEND_LOG_LEVEL);

static struct sigfox_context ctx;
static uint8_t customer_data[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc};
static uint8_t customer_resp[8] =  {0};

void main(void)
{
#if defined(CONFIG_SIGFOX_SAMPLE_USE_LEDS)
	struct device *led0_dev;
	int ret;
#endif

	ctx.dev = device_get_binding(DEFAULT_RADIO);
	if (!ctx.dev) {
		LOG_ERR("%s Device not found", DEFAULT_RADIO);
		return;
	}

	ctx.config.mode = SIGFOX_MODE_TX;

	/* Set SIGFOX Context */
	sigfox_radio_set_context(&ctx);

#if defined(CONFIG_SIGFOX_SAMPLE_USE_LEDS)
	led0_dev = device_get_binding(LED0);
	if (!led0_dev) {
		LOG_ERR("%s Device not found", LED0);
		return;
	}

	ret = gpio_pin_configure(led0_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		LOG_ERR("GPIO configure error");
		return;
	}
#endif

	while (1) {
		LOG_INF("Data sent!");
#if defined(CONFIG_SIGFOX_SAMPLE_USE_LEDS)
		gpio_pin_set(led0_dev, PIN, 1);
		k_msleep(50);
		gpio_pin_set(led0_dev, PIN, 0);
#endif

		SIGFOX_API_send_frame(customer_data, sizeof(customer_data),
				      customer_resp, 2, 0);

		/* Send data at 1min interval */
		k_sleep(K_SECONDS(20));
	}
}
