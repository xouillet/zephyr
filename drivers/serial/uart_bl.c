/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bouffalolab_bl_uart

/**
 * @brief UART driver for Bouffalo Lab MCU family.
 */
#include <device.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <bl602_uart.h>
#include <bl602_glb.h>

#define UART_CTS_FLOWCONTROL_ENABLE	(0)
#define UART_RTS_FLOWCONTROL_ENABLE	(0)
#define UART_MSB_FIRST_ENABLE		(0)
#define UART_DEFAULT_RTO_TIMEOUT	(255)
#define UART_CLOCK_DIV			(0)

struct pin_mux_cfg {
	uint8_t pin;
	uint16_t func;
};

static const struct pin_mux_cfg uart_pin_table[][2] = {
	{
		{
			.pin = GLB_GPIO_PIN_7,
			.func = GPIO_FUN_UART0_RX,
		},
		{
			.pin = GLB_GPIO_PIN_16,
			.func = GPIO_FUN_UART0_TX,
		}
	},
	{
		{
			.pin = GLB_GPIO_PIN_3,
			.func = GPIO_FUN_UART1_RX,
		},
		{
			.pin = GLB_GPIO_PIN_2,
			.func = GPIO_FUN_UART1_TX,
		}
	},
};

struct bl_config {
	uint32_t periph_id;
	UART_CFG_Type uart_cfg;
	UART_FifoCfg_Type fifo_cfg;
};

static void uart_bl_pin_cfg(uint32_t id)
{
	uint32_t idx;
	GLB_GPIO_Cfg_Type gpio_cfg;

	gpio_cfg.drive = 0;
	gpio_cfg.smtCtrl = 1;
	gpio_cfg.gpioFun = GPIO_FUN_UART;

	for (idx = 0; idx < 2; idx++) {
		gpio_cfg.gpioMode = GPIO_MODE_AF;
		gpio_cfg.pullType = GPIO_PULL_UP;
		gpio_cfg.gpioPin = uart_pin_table[id][idx].pin;
		GLB_UART_Fun_Sel(gpio_cfg.gpioPin % 8,
				 uart_pin_table[id][idx].func & 0x07);
		GLB_GPIO_Init(&gpio_cfg);
	}
}

static int uart_bl_init(const struct device *dev)
{
	const struct bl_config *config = dev->config;

	uart_bl_pin_cfg(config->periph_id);

	GLB_Set_UART_CLK(1, HBN_UART_CLK_160M, UART_CLOCK_DIV);

	UART_IntMask(config->periph_id, UART_INT_ALL, 1);
	UART_Disable(config->periph_id, UART_TXRX);

	UART_Init(config->periph_id, (UART_CFG_Type *)&config->uart_cfg);
	UART_TxFreeRun(config->periph_id, 1);
	UART_SetRxTimeoutValue(config->periph_id, UART_DEFAULT_RTO_TIMEOUT);
	UART_FifoConfig(config->periph_id, (UART_FifoCfg_Type *)&config->fifo_cfg);
	UART_Enable(config->periph_id, UART_TXRX);

	return 0;
}

static int uart_bl_poll_in(const struct device *dev, unsigned char *c)
{
	const struct bl_config *config = dev->config;

	return UART_ReceiveData(config->periph_id, (uint8_t *)c, 1);
}

static void uart_bl_poll_out(const struct device *dev, unsigned char c)
{
	const struct bl_config *config = dev->config;

	while (UART_GetTxFifoCount(config->periph_id) == 0) {
		;
	}

	(void)UART_SendData(config->periph_id, (uint8_t *)&c, 1);
}

static const struct uart_driver_api uart_bl_driver_api = {
	.poll_in = uart_bl_poll_in,
	.poll_out = uart_bl_poll_out,
};

#define BL_UART_INIT(n)								\
	static const struct bl_config bl_uart##n##_config = {			\
		.periph_id = DT_INST_PROP(n, peripheral_id),			\
										\
		.uart_cfg.baudRate = DT_INST_PROP(n, current_speed),		\
		.uart_cfg.dataBits = UART_DATABITS_8,				\
		.uart_cfg.stopBits = UART_STOPBITS_1,				\
		.uart_cfg.parity = UART_PARITY_NONE,				\
		.uart_cfg.uartClk = SOC_BOUFFALOLAB_BL_PLL160_FREQ_HZ,		\
		.uart_cfg.ctsFlowControl = UART_CTS_FLOWCONTROL_ENABLE,		\
		.uart_cfg.rtsSoftwareControl = UART_RTS_FLOWCONTROL_ENABLE,	\
		.uart_cfg.byteBitInverse = UART_MSB_FIRST_ENABLE,		\
										\
		.fifo_cfg.txFifoDmaThreshold = 1,				\
		.fifo_cfg.rxFifoDmaThreshold = 1,				\
		.fifo_cfg.txFifoDmaEnable = 0,					\
		.fifo_cfg.rxFifoDmaEnable = 0,					\
	};									\
	DEVICE_DT_INST_DEFINE(n, &uart_bl_init,					\
			      NULL,						\
			      NULL,						\
			      &bl_uart##n##_config, PRE_KERNEL_1,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_bl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BL_UART_INIT)
