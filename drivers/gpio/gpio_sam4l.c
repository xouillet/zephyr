/*
 * Copyright (c) 2018 Justin Watson
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/gpio.h>

#include "gpio_utils.h"

typedef void (*config_func_t)(struct device *dev);

struct gpio_sam_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	Gpio *regs;
	config_func_t config_func;
	u32_t periph_id;
};

struct gpio_sam_runtime {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t cb;
};

#define DEV_CFG(dev) \
	((const struct gpio_sam_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct gpio_sam_runtime * const)(dev)->driver_data)

#define GPIO_SAM_ALL_PINS    0xFFFFFFFF

static int gpio_sam_port_configure(struct device *dev, u32_t mask,
				   gpio_flags_t flags)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	if (flags & GPIO_SINGLE_ENDED) {
		/* TODO: Add support for Open Source, Open Drain mode */
		return -ENOTSUP;
	}

	if (!(flags & (GPIO_OUTPUT | GPIO_INPUT))) {
		/* Neither input nor output mode is selected */

		/* Disable the interrupt. */
		gpio->IERC = mask;
		/* Disable pull-up. */
		gpio->PUERC = mask;
		/* Disable pull-down. */
		gpio->PDERC = mask;
		/* Let the GPIO control the pin (instead of a peripheral). */
		gpio->GPERS = mask;
		/* Disable output. */
		gpio->ODERC = mask;

		return 0;
	}

	/* Setup the pin direcion. */
	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			/* Set the pin. */
			gpio->OVRS = mask;
		}
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			/* Clear the pin. */
			gpio->OVRC = mask;
		}
		/* Enable the output */
		gpio->ODERS = mask;
		gpio->STERC = mask;
	} else {
		/* Disable the output */
		gpio->ODERC = mask;
		gpio->STERS = mask;
	}

	/* Note: Input is always enabled. */

	/* Setup selected Pull resistor.
	 *
	 * A pull cannot be enabled if the opposite pull is enabled.
	 * Clear both pulls, then enable the one we need.
	 */
	gpio->PUERC = mask;
	gpio->PDERC = mask;
	if (flags & GPIO_PULL_UP) {
		/* Enable pull-up. */
		gpio->PUERS = mask;
	/* Setup Pull-down resistor. */
	} else if (flags & GPIO_PULL_DOWN) {
		/* Enable pull-down. */
		gpio->PDERS = mask;
	}

	/* Enable the PIO to control the pin (instead of a peripheral). */
	gpio->GPERS = mask;

	return 0;
}

static int gpio_sam_config(struct device *dev, gpio_pin_t pin,
			   gpio_flags_t flags)
{
	return gpio_sam_port_configure(dev, BIT(pin), flags);
}

static int gpio_sam_port_get_raw(struct device *dev, u32_t *value)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	*value = gpio->PVR;

	return 0;
}

static int gpio_sam_port_set_masked_raw(struct device *dev, u32_t mask,
					u32_t value)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	gpio->OVR = (gpio->PVR & ~mask) | (mask & value);

	return 0;
}

static int gpio_sam_port_set_bits_raw(struct device *dev, u32_t mask)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	/* Set pins. */
	gpio->OVRS = mask;

	return 0;
}

static int gpio_sam_port_clear_bits_raw(struct device *dev, u32_t mask)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	/* Clear pins. */
	gpio->OVRC = mask;

	return 0;
}

static int gpio_sam_port_toggle_bits(struct device *dev, u32_t mask)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	/* Toggle pins. */
	gpio->OVRT = mask;

	return 0;
}

static int gpio_sam_port_interrupt_configure(struct device *dev, u32_t mask,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;

	/* Disable the interrupt. */
	gpio->IERC = mask;
	/* Set default to both edges */
	gpio->IMR0C = mask;
	gpio->IMR1C = mask;

	if (trig != GPIO_INT_TRIG_BOTH) {
		if (trig == GPIO_INT_TRIG_HIGH) {
			gpio->IMR0S = mask;
			gpio->IMR1C = mask;
		} else {
			gpio->IMR0C = mask;
			gpio->IMR1S = mask;
		}
	}

	if (mode != GPIO_INT_MODE_DISABLED) {
		/* Clear any pending interrupts */
		gpio->IFRC = mask;
		/* Enable the interrupt. */
		gpio->IERS = mask;
	}

	return 0;
}

static int gpio_sam_pin_interrupt_configure(struct device *dev,
		gpio_pin_t pin, enum gpio_int_mode mode,
		enum gpio_int_trig trig)
{
	return gpio_sam_port_interrupt_configure(dev, BIT(pin), mode, trig);
}

static void gpio_sam_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);
	Gpio * const gpio = cfg->regs;
	struct gpio_sam_runtime *context = dev->driver_data;
	u32_t int_stat;

	int_stat = gpio->IFR;

	gpio_fire_callbacks(&context->cb, dev, int_stat);

	gpio->IFRC = int_stat;
}

static int gpio_sam_manage_callback(struct device *port,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_sam_runtime *context = port->driver_data;

	return gpio_manage_callback(&context->cb, callback, set);
}

static int gpio_sam_enable_callback(struct device *port,
				    gpio_pin_t pin)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(port);
	Gpio * const gpio = cfg->regs;

	gpio->IERS = BIT(pin);

	return 0;
}

static int gpio_sam_disable_callback(struct device *port,
				     gpio_pin_t pin)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(port);
	Gpio * const gpio = cfg->regs;

	gpio->IERC = BIT(pin);

	return 0;
}

static const struct gpio_driver_api gpio_sam_api = {
	.pin_configure = gpio_sam_config,
	.port_get_raw = gpio_sam_port_get_raw,
	.port_set_masked_raw = gpio_sam_port_set_masked_raw,
	.port_set_bits_raw = gpio_sam_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sam_port_clear_bits_raw,
	.port_toggle_bits = gpio_sam_port_toggle_bits,
	.pin_interrupt_configure = gpio_sam_pin_interrupt_configure,
	.manage_callback = gpio_sam_manage_callback,
	.enable_callback = gpio_sam_enable_callback,
	.disable_callback = gpio_sam_disable_callback,
};

int gpio_sam_init(struct device *dev)
{
	const struct gpio_sam_config * const cfg = DEV_CFG(dev);

	/* The peripheral clock must be enabled for the interrupts to work. */
	soc_pmc_peripheral_enable(cfg->periph_id);

	cfg->config_func(dev);

	return 0;
}

/* PORT A */
#ifdef DT_INST_0_ATMEL_SAM_GPIO_BASE_ADDRESS
static void port_a_sam_config_func(struct device *dev);

static const struct gpio_sam_config port_a_sam_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_0_ATMEL_SAM_GPIO_NGPIOS),
	},
	.regs = (Gpio *)DT_INST_0_ATMEL_SAM_GPIO_BASE_ADDRESS,
	.periph_id = DT_INST_0_ATMEL_SAM_GPIO_PERIPHERAL_ID,
	.config_func = port_a_sam_config_func,
};

static struct gpio_sam_runtime port_a_sam_runtime;

DEVICE_AND_API_INIT(port_a_sam, DT_INST_0_ATMEL_SAM_GPIO_LABEL, gpio_sam_init,
		    &port_a_sam_runtime, &port_a_sam_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam_api);

static void port_a_sam_config_func(struct device *dev)
{
	IRQ_CONNECT(DT_INST_0_ATMEL_SAM_GPIO_IRQ_0,
		    DT_INST_0_ATMEL_SAM_GPIO_IRQ_0_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_a_sam), 0);
	irq_enable(DT_INST_0_ATMEL_SAM_GPIO_IRQ_0);
	IRQ_CONNECT(DT_INST_0_ATMEL_SAM_GPIO_IRQ_1,
		    DT_INST_0_ATMEL_SAM_GPIO_IRQ_1_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_a_sam), 0);
	irq_enable(DT_INST_0_ATMEL_SAM_GPIO_IRQ_1);
	IRQ_CONNECT(DT_INST_0_ATMEL_SAM_GPIO_IRQ_2,
		    DT_INST_0_ATMEL_SAM_GPIO_IRQ_2_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_a_sam), 0);
	irq_enable(DT_INST_0_ATMEL_SAM_GPIO_IRQ_2);
	IRQ_CONNECT(DT_INST_0_ATMEL_SAM_GPIO_IRQ_3,
		    DT_INST_0_ATMEL_SAM_GPIO_IRQ_3_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_a_sam), 0);
	irq_enable(DT_INST_0_ATMEL_SAM_GPIO_IRQ_3);
}

#endif /* DT_INST_0_ATMEL_SAM_GPIO_BASE_ADDRESS */

/* PORT B */
#ifdef DT_INST_2_ATMEL_SAM_GPIO_BASE_ADDRESS
static void port_b_sam_config_func(struct device *dev);

static const struct gpio_sam_config port_b_sam_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_1_ATMEL_SAM_GPIO_NGPIOS),
	},
	.regs = (Gpio *)DT_INST_1_ATMEL_SAM_GPIO_BASE_ADDRESS,
	.periph_id = DT_INST_1_ATMEL_SAM_GPIO_PERIPHERAL_ID,
	.config_func = port_b_sam_config_func,
};

static struct gpio_sam_runtime port_b_sam_runtime;

DEVICE_AND_API_INIT(port_b_sam, DT_INST_1_ATMEL_SAM_GPIO_LABEL, gpio_sam_init,
		    &port_b_sam_runtime, &port_b_sam_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam_api);

static void port_b_sam_config_func(struct device *dev)
{
	IRQ_CONNECT(DT_INST_1_ATMEL_SAM_GPIO_IRQ_0,
		    DT_INST_1_ATMEL_SAM_GPIO_IRQ_0_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_b_sam), 0);
	irq_enable(DT_INST_1_ATMEL_SAM_GPIO_IRQ_0);
	IRQ_CONNECT(DT_INST_1_ATMEL_SAM_GPIO_IRQ_1,
		    DT_INST_1_ATMEL_SAM_GPIO_IRQ_1_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_b_sam), 0);
	irq_enable(DT_INST_1_ATMEL_SAM_GPIO_IRQ_1);
	IRQ_CONNECT(DT_INST_1_ATMEL_SAM_GPIO_IRQ_2,
		    DT_INST_1_ATMEL_SAM_GPIO_IRQ_2_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_b_sam), 0);
	irq_enable(DT_INST_1_ATMEL_SAM_GPIO_IRQ_2);
	IRQ_CONNECT(DT_INST_1_ATMEL_SAM_GPIO_IRQ_3,
		    DT_INST_1_ATMEL_SAM_GPIO_IRQ_3_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_b_sam), 0);
	irq_enable(DT_INST_1_ATMEL_SAM_GPIO_IRQ_3);
}

#endif /* DT_INST_1_ATMEL_SAM_GPIO_BASE_ADDRESS */

/* PORT C */
#ifdef DT_INST_2_ATMEL_SAM_GPIO_BASE_ADDRESS
static void port_c_sam_config_func(struct device *dev);

static const struct gpio_sam_config port_c_sam_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_2_ATMEL_SAM_GPIO_NGPIOS),
	},
	.regs = (Gpio *)DT_INST_2_ATMEL_SAM_GPIO_BASE_ADDRESS,
	.periph_id = DT_INST_2_ATMEL_SAM_GPIO_PERIPHERAL_ID,
	.config_func = port_c_sam_config_func,
};

static struct gpio_sam_runtime port_c_sam_runtime;

DEVICE_AND_API_INIT(port_c_sam, DT_INST_2_ATMEL_SAM_GPIO_LABEL, gpio_sam_init,
		    &port_c_sam_runtime, &port_c_sam_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam_api);

static void port_c_sam_config_func(struct device *dev)
{
	IRQ_CONNECT(DT_INST_2_ATMEL_SAM_GPIO_IRQ_0,
		    DT_INST_2_ATMEL_SAM_GPIO_IRQ_0_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_c_sam), 0);
	irq_enable(DT_INST_2_ATMEL_SAM_GPIO_IRQ_0);
	IRQ_CONNECT(DT_INST_2_ATMEL_SAM_GPIO_IRQ_1,
		    DT_INST_2_ATMEL_SAM_GPIO_IRQ_1_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_c_sam), 0);
	irq_enable(DT_INST_2_ATMEL_SAM_GPIO_IRQ_1);
	IRQ_CONNECT(DT_INST_2_ATMEL_SAM_GPIO_IRQ_2,
		    DT_INST_2_ATMEL_SAM_GPIO_IRQ_2_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_c_sam), 0);
	irq_enable(DT_INST_2_ATMEL_SAM_GPIO_IRQ_2);
	IRQ_CONNECT(DT_INST_2_ATMEL_SAM_GPIO_IRQ_3,
		    DT_INST_2_ATMEL_SAM_GPIO_IRQ_3_PRIORITY,
		    gpio_sam_isr, DEVICE_GET(port_c_sam), 0);
	irq_enable(DT_INST_2_ATMEL_SAM_GPIO_IRQ_3);
}

#endif /* DT_INST_2_ATMEL_SAM_GPIO_BASE_ADDRESS */
