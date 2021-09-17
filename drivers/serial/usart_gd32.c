/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gigadevice_gd32_usart

#include <device.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>

struct gd32_uart_config {
	uint32_t reg;
	uint32_t rcu_periph_clock;
};

struct gd32_uart_data {
	uint32_t baud_rate;
};

#define DEV_CFG(dev) \
	((const struct gd32_uart_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct gd32_uart_data *const)(dev)->data)

static int uart_gd32_init(const struct device *dev)
{
	const struct gd32_uart_config *const cfg = DEV_CFG(dev);
	struct gd32_uart_data *const data = DEV_DATA(dev);

	/* Driver currently only supports UART-0 */
	if (cfg->reg != USART0) {
		return -EINVAL;
	}

	/* TODO: Rework after add pinctrl */
	rcu_periph_clock_enable(RCU_GPIOA);
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

	rcu_periph_clock_enable(cfg->rcu_periph_clock);
	usart_deinit(cfg->reg);
	usart_baudrate_set(cfg->reg, data->baud_rate);
	usart_word_length_set(cfg->reg, USART_WL_8BIT);
	usart_parity_config(cfg->reg, USART_PM_NONE);
	usart_stop_bit_set(cfg->reg, USART_STB_1BIT);
	usart_parity_config(cfg->reg, USART_PM_NONE);
	usart_receive_config(cfg->reg, USART_RECEIVE_ENABLE);
	usart_transmit_config(cfg->reg, USART_TRANSMIT_ENABLE);
	usart_enable(cfg->reg);

	return 0;
}

static int uart_gd32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct gd32_uart_config *const cfg = DEV_CFG(dev);
	uint32_t status;

	status = usart_flag_get(cfg->reg, USART_FLAG_RBNE);

	if (!status) {
		return -EPERM;
	}

	*c = (unsigned char)(usart_data_receive(cfg->reg) & 0xff);

	return 0;
}

static void uart_gd32_poll_out(const struct device *dev, unsigned char c)
{
	const struct gd32_uart_config *const cfg = DEV_CFG(dev);

	usart_data_transmit(cfg->reg, c);

	while (usart_flag_get(cfg->reg, USART_FLAG_TBE) == RESET) {
		;
	}
}

static int uart_gd32_err_check(const struct device *dev)
{
	const struct gd32_uart_config *const cfg = DEV_CFG(dev);
	uint32_t status = USART_STAT0(cfg->reg);
	int errors = 0;

	if (status & USART_FLAG_ORERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_ORERR);

		errors |= UART_ERROR_OVERRUN;
	}

	if (status & USART_FLAG_PERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_PERR);

		errors |= UART_ERROR_PARITY;
	}

	if (status & USART_FLAG_FERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_FERR);

		errors |= UART_ERROR_FRAMING;
	}

	usart_flag_clear(cfg->reg, USART_FLAG_NERR);

	return errors;
}

static const struct uart_driver_api uart_gd32_driver_api = {
	.poll_in = uart_gd32_poll_in,
	.poll_out = uart_gd32_poll_out,
	.err_check = uart_gd32_err_check,
};

#define GD32_UART_INIT(n)							\
	static struct gd32_uart_data uart##n##_gd32_data = {			\
		.baud_rate = DT_INST_PROP(n, current_speed),			\
	};									\
	static const struct gd32_uart_config uart##n##_gd32_config = {		\
		.reg = DT_INST_REG_ADDR(n),					\
		.rcu_periph_clock = DT_INST_PROP(n, rcu_periph_clock),		\
	};									\
	DEVICE_DT_INST_DEFINE(n, &uart_gd32_init,				\
			      NULL,						\
			      &uart##n##_gd32_data,				\
			      &uart##n##_gd32_config, PRE_KERNEL_1,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_UART_INIT)
