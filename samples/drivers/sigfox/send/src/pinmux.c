/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>

#if defined(CONFIG_BOARD_FRDM_K64F)
#include <fsl_port.h>
#endif

static int sigfox_pinmux_init(struct device *dev)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_BOARD_FRDM_K64F)
	struct device *portb =
		device_get_binding(CONFIG_PINMUX_MCUX_PORTB_NAME);
	struct device *portc =
		device_get_binding(CONFIG_PINMUX_MCUX_PORTC_NAME);

	/* INT */
	pinmux_pin_set(portb, 2, PORT_PCR_MUX(kPORT_MuxAsGpio));
	/* SDN */
	pinmux_pin_set(portc, 3, PORT_PCR_MUX(kPORT_MuxAsGpio));
	/* EEPROM CS */
	pinmux_pin_set(portc, 12, PORT_PCR_MUX(kPORT_MuxAsGpio));
#endif

	return 0;
}

SYS_INIT(sigfox_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
