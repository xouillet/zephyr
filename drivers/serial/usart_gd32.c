/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gigadevice_gd32_usart

/**
 * @brief UART driver for GigaDevice MCU family.
 */
#include <device.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>

#ifndef USART_STAT0
#define USART_STAT0 USART_STAT
#endif

struct gd32_config {
	uint32_t periph_id;
};

static int uart_gd32_init(const struct device *dev)
{
	(void)dev;

	rcu_periph_clock_enable(RCU_GPIOA);
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

	rcu_periph_clock_enable(RCU_USART0);
	usart_deinit(USART0);
	usart_baudrate_set(USART0, 115200);
	usart_word_length_set(USART0, USART_WL_8BIT);
	usart_parity_config(USART0, USART_PM_NONE);
	usart_stop_bit_set(USART0, USART_STB_1BIT);
	usart_parity_config(USART0, USART_PM_NONE);
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_enable(USART0);

	//LED1 @ PF1
	// rcu_periph_clock_enable(RCU_GPIOF);
	// gpio_init(GPIOF, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	// gpio_bit_set(GPIOF, GPIO_PIN_1);

	// usart_data_transmit(USART0, 'O');
	// while(usart_flag_get(USART0, USART_FLAG_TBE)==RESET);
	// usart_data_transmit(USART0, 'K');
	// while(usart_flag_get(USART0, USART_FLAG_TBE)==RESET);

	return 0;
}

static int uart_gd32_poll_in(const struct device *dev, unsigned char *c)
{
	uint32_t status;

	(void)dev;

	status = usart_flag_get(USART0, USART_FLAG_RBNE);

	if (!status) {
		return -EPERM;
	}

	*c = (unsigned char)(usart_data_receive(USART0) & 0xff);

	return 0;
}

static void uart_gd32_poll_out(const struct device *dev, unsigned char c)
{
	usart_data_transmit(USART0, c);

	while(usart_flag_get(USART0, USART_FLAG_TBE)==RESET) {
		;
	}
}

static int uart_gd32_err_check(const struct device *dev)
{
	uint32_t status = USART_STAT0(USART0);
	int errors = 0;

	(void)dev;

	if (status & USART_FLAG_ORERR) {
		usart_flag_clear(USART0, USART_FLAG_ORERR);

		errors |= UART_ERROR_OVERRUN;
	}

	if (status & USART_FLAG_PERR) {
		usart_flag_clear(USART0, USART_FLAG_PERR);

		errors |= UART_ERROR_PARITY;
	}

	if (status & USART_FLAG_FERR) {
		usart_flag_clear(USART0, USART_FLAG_FERR);

		errors |= UART_ERROR_FRAMING;
	}

	usart_flag_clear(USART0, USART_FLAG_NERR);

	return errors;
}

static const struct uart_driver_api uart_gd32_driver_api = {
	.poll_in = uart_gd32_poll_in,
	.poll_out = uart_gd32_poll_out,
	.err_check = uart_gd32_err_check,
};

#define GD32_UART_INIT(n)							\
	static const struct gd32_config gd32_uart##n##_config = {		\
		.periph_id = DT_INST_PROP(n, peripheral_id),			\
	};									\
	DEVICE_DT_INST_DEFINE(n, &uart_gd32_init,				\
			      NULL,						\
			      NULL,						\
			      &gd32_uart##n##_config, PRE_KERNEL_1,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_UART_INIT)
