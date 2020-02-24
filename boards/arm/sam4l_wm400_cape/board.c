/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <soc.h>
#include <drivers/gpio.h>

/* RED LED MODULE */
#define LED_PINS0 {PIN_PA08, GPIO, ID_GPIO, SOC_GPIO_FUNC_OUT_0}
/* GREEN LED MODULE */
#define LED_PINS1 {PIN_PB06, GPIO, ID_GPIO, SOC_GPIO_FUNC_OUT_0}
/* BLUE LED MODULE */
#define LED_PINS2 {PIN_PB07, GPIO, ID_GPIO, SOC_GPIO_FUNC_OUT_0}
/* RED LED CAPE */
#define LED_PINS3 {PIN_PA24, GPIO, ID_GPIO, SOC_GPIO_FUNC_OUT_0}

const struct soc_gpio_pin pins = LED_PINS3;

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define RUN_LOCAL 0

static int board_sam4l_wm400_cape_board_init(struct device *dev)
{
#if (RUN_LOCAL == 1)
	struct device *ctrl_btn;
	struct device *ctrl_led1;
	bool led_is_on = true;
#endif
	struct device *ctrl_led2;
	int ret;

	ARG_UNUSED(dev);

	ctrl_led2 = device_get_binding(DT_GPIO_LEDS_LED_2_GPIOS_CONTROLLER);
	if (ctrl_led2 == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_led2,
				 DT_GPIO_LEDS_LED_2_GPIOS_PIN,
				 DT_GPIO_LEDS_LED_2_GPIOS_FLAGS |
				 GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return 0;
	}

#if (RUN_LOCAL == 1)
	ctrl_led1 = device_get_binding(DT_GPIO_LEDS_LED_1_GPIOS_CONTROLLER);
	if (ctrl_led1 == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_led1,
				 DT_GPIO_LEDS_LED_1_GPIOS_PIN,
				 DT_GPIO_LEDS_LED_1_GPIOS_FLAGS |
				 GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return 0;
	}

	ctrl_btn = device_get_binding(DT_GPIO_KEYS_BUTTON_1_GPIOS_CONTROLLER);
	if (ctrl_btn == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_btn,
				 DT_GPIO_KEYS_BUTTON_1_GPIOS_PIN,
				 DT_GPIO_KEYS_BUTTON_1_GPIOS_FLAGS |
				 GPIO_INPUT);

	if (ret < 0) {
		return 0;
	}
#endif

	soc_gpio_configure(&pins);

#if (RUN_LOCAL == 1)
	while (1) {
		ret = gpio_pin_get(ctrl_btn, DT_GPIO_KEYS_BUTTON_1_GPIOS_PIN);
		gpio_pin_set(ctrl_led1, DT_GPIO_LEDS_LED_1_GPIOS_PIN, ret);

		gpio_pin_set(ctrl_led2, DT_GPIO_LEDS_LED_2_GPIOS_PIN,
			     (int) led_is_on);
		led_is_on = !led_is_on;

		k_sleep(SLEEP_TIME_MS);
	}
#endif
	return 0;
}

SYS_INIT(board_sam4l_wm400_cape_board_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
