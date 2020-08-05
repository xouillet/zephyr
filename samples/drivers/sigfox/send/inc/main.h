/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DEFAULT_RADIO_NODE	DT_ALIAS(sigfox0)
#define DEFAULT_RADIO		DT_LABEL(DEFAULT_RADIO_NODE)

#define LED0_NODE		DT_ALIAS(led0)
#define LED0			DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN			DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define FLAGS			DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
#define FLAGS			0
#endif
