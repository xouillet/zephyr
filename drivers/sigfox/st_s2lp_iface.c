/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(st_s2lp_iface, CONFIG_SIGFOX_S2LP_LOG_LEVEL);

#include <errno.h>
#include <assert.h>

#include <device.h>
#include <drivers/sigfox.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#include "st_s2lp.h"
#include "st_s2lp_regs.h"
#include "st_s2lp_iface.h"
#include "st_s2lp_rf.h"

int s2lp_iface_wakeup(const struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	const struct s2lp_context *ctx = dev->data;
	uint64_t timeout;
	uint8_t state[2];

	gpio_pin_set(ctx->sdn_gpio, conf->sdn.pin, 1);
	k_msleep(5);
	gpio_pin_set(ctx->sdn_gpio, conf->sdn.pin, 0);

	LOG_DBG("Wakeup");
	k_usleep(500);

	s2lp_iface_cmd(dev, S2LP_CMD_RESET);

	timeout = k_uptime_get() + 50000;
	do {
		if (k_uptime_get() > timeout) {
			return -ETIMEDOUT;
		}

		k_usleep(10);

		s2lp_iface_burst_read(dev, S2LP_MC_STATE1_REG, state, 2);
		state[1] &= S2LP_STATE;
		state[1] >>= 1;
		LOG_DBG("State: %02X-%02X", state[0], state[1]);
	} while (state[0] != 0x52 && state[1] != S2LP_STATE_READY);

	s2lp_sfx_trx_fem_init(dev);

	return 0;
}

void s2lp_iface_shutdown(const struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	const struct s2lp_context *ctx = dev->data;

	/* Disable ISR */
	gpio_pin_interrupt_configure(ctx->irq_gpio,
				conf->dio[ctx->irq_dio_id].pin,
				GPIO_INT_DISABLE);

	gpio_pin_set(ctx->sdn_gpio, conf->sdn.pin, 1);
}

void s2lp_iface_cmd(const struct device *dev,
		    enum s2lp_trx_cmd_t cmd)
{
	const struct s2lp_context *ctx = dev->data;
	uint8_t header[2] = { S2LP_IFACE_CMD, cmd };
	uint8_t status[2];

	const struct spi_buf tx_buf = {
		.buf = &header,
		.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = status,
		.len = 2,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) != 0) {
		LOG_ERR("Failed to exec s2lp_send_cmd");
	}

	LOG_DBG("CMD: %02X, Status: %02X%02X", cmd, status[0], status[1]);
}

uint8_t s2lp_iface_reg_read(const struct device *dev, uint8_t addr)
{
	const struct s2lp_context *ctx = dev->data;
	uint8_t header[2] = { S2LP_IFACE_READ, addr };
	uint8_t status[2];
	uint8_t regval = 0;

	const struct spi_buf tx_buf = {
		.buf = &header,
		.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = &status,
			.len = 2,
		},
		{
			.buf = &regval,
			.len = 1,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	if (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) != 0) {
		LOG_ERR("Failed to exec s2lp_reg_read CMD at address %d",
			addr);
	}

	LOG_DBG("Read Address: %02X, RegVal: %02X, Status: %02X%02X",
		addr, regval, status[0], status[1]);

	return regval;
}

void s2lp_iface_reg_write(const struct device *dev,
			  uint8_t addr,
			  uint8_t data)
{
	const struct s2lp_context *ctx = dev->data;
	uint8_t header[2] = { S2LP_IFACE_WRITE, addr };
	uint8_t status[2];

	const struct spi_buf tx_buf[2] = {
		{
			.buf = &header,
			.len = 2,
		},
		{
			.buf = &data,
			.len = 1,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2,
	};
	const struct spi_buf rx_buf = {
		.buf = &status,
		.len = 2,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) != 0) {
		LOG_ERR("Failed to exec s2lp_reg_write at address %d",
			addr);
	}

	LOG_DBG("Write Address: %02X, RegVal: %02X, Status: %02X%02X",
		addr, data, status[0], status[1]);
}

uint8_t s2lp_iface_bit_read(const struct device *dev,
			    uint8_t addr,
			    uint8_t mask,
			    uint8_t pos)
{
	uint8_t ret;

	ret = s2lp_iface_reg_read(dev, addr);
	ret &= mask;
	ret >>= pos;

	return ret;
}

void s2lp_iface_bit_write(const struct device *dev,
			  uint8_t reg_addr,
			  uint8_t mask,
			  uint8_t pos,
			  uint8_t new_value)
{
	uint8_t current_reg_value;

	current_reg_value = s2lp_iface_reg_read(dev, reg_addr);
	current_reg_value &= ~mask;
	new_value <<= pos;
	new_value &= mask;
	LOG_DBG("R+Mask: 0x%02X, N+Mask: 0x%02X, W: 0x%02X",
		current_reg_value, new_value, current_reg_value | new_value);
	new_value |= current_reg_value;
	s2lp_iface_reg_write(dev, reg_addr, new_value);
}

void s2lp_iface_burst_read(const struct device *dev,
			   uint8_t addr,
			   uint8_t *data,
			   uint8_t length)
{
	const struct s2lp_context *ctx = dev->data;
	uint8_t header[2] = { S2LP_IFACE_READ, addr };
	uint8_t status[2];

	const struct spi_buf tx_buf = {
		.buf = &header,
		.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = &status,
			.len = 2,
		},
		{
			.buf = data,
			.len = length,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) != 0) {
		LOG_ERR("Failed to exec s2lp_frame_read PHR");
	}

	LOG_DBG("Burst R: %02X, length: %02X, Status: %02X%02X",
		addr, length, status[0], status[1]);
	LOG_HEXDUMP_DBG(data, length, "Burst R");
}

void s2lp_iface_burst_write(const struct device *dev,
			    uint8_t addr,
			    uint8_t *data,
			    uint8_t length)
{
	const struct s2lp_context *ctx = dev->data;
	uint8_t header[2] = { S2LP_IFACE_WRITE, addr };
	uint8_t status[2];

	const struct spi_buf tx_buf[2] = {
		{
			.buf = &header,
			.len = 2,
		},
		{
			.buf = data,
			.len = length,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2,
	};
	const struct spi_buf rx_buf = {
		.buf = &status,
		.len = 2,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) != 0) {
		LOG_ERR("Failed to exec s2lp_frame_write");
	}

	LOG_DBG("Burst W: %02X, length: %02X, Status: %02X%02X",
		addr, length, status[0], status[1]);
	LOG_HEXDUMP_DBG(data, length, "Burst W");
}
