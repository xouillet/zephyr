/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_s2lp

#include <drivers/sigfox.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <zephyr.h>
#include <sys/byteorder.h>

#include "st_s2lp.h"
#include "st_s2lp_regs.h"
#include "st_s2lp_iface.h"
#include "st_s2lp_rf.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_SIGFOX_LOG_LEVEL);

static inline void s2lp_trx_isr_handler(struct device *port,
					struct gpio_callback *cb,
					uint32_t pins)
{
	struct s2lp_context *ctx = CONTAINER_OF(cb,
						struct s2lp_context,
						irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&ctx->trx_wait_isr);
}

static void s2lp_trx_thread_main(void *arg)
{
	struct device *dev = INT_TO_POINTER(arg);
	struct s2lp_context *ctx = dev->data;
	uint32_t isr_status;

	while (true) {
		k_sem_take(&ctx->trx_wait_isr, K_FOREVER);

		s2lp_iface_burst_read(dev, S2LP_IRQ_STATUS3_REG,
				      (uint8_t *) &isr_status,
				      sizeof(uint32_t));

		isr_status = sys_be32_to_cpu(isr_status);

#if (CONFIG_SIGFOX_S2LP_IFACE_LOG_LEVEL != LOG_LEVEL_NONE)
		LOG_DBG("ISR: 0x%08x, FSM: %d", isr_status, ctx->trx.fsm);
#endif

		/* TX only */
		if (ctx->trx.fsm == S2LP_TRX_FSM_STATE_TX) {
			if (isr_status & S2LP_IRQ_MASK_TX_FIFO_FLOW_ERR) {
				k_sem_give(&ctx->trx_sync);
				continue;
			}

			/**
			 * If we are transmitting, we have to tick the
			 * transmission FSM so that actions (fill the TX FIFO
			 * according to the actual state) are done in this
			 * same context.
			 */
			s2lp_sfx_rf_tx_fsm(dev);

			if (ctx->trx.tx.state == S2LP_TX_STATE_NONE) {
				ctx->trx.fsm = S2LP_TRX_FSM_STATE_IDLE;

				k_sem_give(&ctx->trx_sync);
				continue;
			}
/*
 * #if defined(MONARCH_FEATURE_ENABLED) && defined(MONARCH_GPIO_SAMPLING)
 *		} else if (ctx->trx.fsm == S2LP_TRX_FSM_STATE_MONARCH_SCAN) {
 *			ST_MONARCH_API_AFTHR_GPIO_CB();
 *		}
 * #endif
 *		} else {
 *			k_sem_give(&ctx->trx_sync);
 */
		}
	}
}

static inline int configure_s2lp_gpios(const struct device *dev, bool is_isr)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint8_t regs[4] = { conf->dio[0].func, conf->dio[1].func,
			    conf->dio[2].func, conf->dio[3].func };

	if (!is_isr) {
		regs[ctx->irq_dio_id] = 0x32;
	}

	s2lp_iface_burst_write(dev, S2LP_GPIO0_CONF_REG, regs, 4);

	LOG_DBG("GPIO0 configured as %d + %d",
		(regs[0] >> S2LP_GPIO_MODE),
		(regs[0] & S2LP_GPIO_MODE));

	LOG_DBG("GPIO1 configured as %d + %d",
		(regs[1] >> S2LP_GPIO_MODE),
		(regs[1] & S2LP_GPIO_MODE));

	LOG_DBG("GPIO2 configured as %d + %d",
		(regs[2] >> S2LP_GPIO_MODE),
		(regs[2] & S2LP_GPIO_MODE));

	LOG_DBG("GPIO3 configured as %d + %d",
		(regs[3] >> S2LP_GPIO_MODE),
		(regs[3] & S2LP_GPIO_MODE));

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_start(const struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	int ret;

	if (gpio_pin_get(ctx->sdn_gpio, conf->sdn.pin)) {
		LOG_DBG("Already started");
		return SIGFOX_ERR_NONE;
	}

	ret = s2lp_iface_wakeup(dev);
	if (ret) {
		LOG_DBG("WakeUp timeout");
		return ret;
	}

	LOG_DBG("start");

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_config(const struct device *dev,
			      const struct sigfox_modem_config *config)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint32_t f_dig;
	uint8_t tmp;
	int ret;

	LOG_DBG("Mode: %d", config->mode);

	if (config->mode == SIGFOX_MODE_TX && ctx->trx.tx_is_ready) {
		ctx->trx.tx.state = S2LP_TX_STATE_NONE;
		return SIGFOX_ERR_NONE;
	}

	ret = s2lp_iface_wakeup(dev);
	if (ret) {
		LOG_DBG("WakeUp timeout");
		return ret;
	}

#if defined(CONFIG_SIGFOX_S2LP_HAS_TXCO)
	s2lp_iface_reg_write(dev, S2LP_XO_RCO_CONF0_REG, 1, 0xB0);
#endif

	/* To mitigate freq drift use 0x2E
	 *
	 * digital divider - set the bit to 1 only if xtal < 30MHz
	 * so that the divider is disabled + Frequency drift mitigation
	 */
	tmp = (conf->trx_freq < S2LP_DIG_DOMAIN_XTAL_THRESH) ? 0x3E : 0x2E;
	s2lp_iface_reg_write(dev, S2LP_XO_RCO_CONF1_REG, tmp);

	switch (config->mode) {
	case SIGFOX_MODE_CS200K_RX:
		/* configure the RF IC into sensing 200KHz bandwidth to be
		 * able to read out RSSI level RSSI level will outputed during
		 * the wait_for_clear_channel api
		 */
	case SIGFOX_MODE_CS300K_RX:
	{
		/* configure the RF IC into sensing 300KHz bandwidth to be
		 * able to read out RSSI level. This is possible to make this
		 * carrier sense in 2 * 200Kz if you respect the regulation
		 * time for listening RSSI level will outputed during the
		 * wait_for_clear_channel api
		 */
		f_dig = conf->trx_freq;
		if (f_dig > S2LP_DIG_DOMAIN_XTAL_THRESH) {
			f_dig >>= 1;
		}

		tmp = S2LP_CALC_FILTER(conf->trx_freq);
		s2lp_iface_reg_write(dev, S2LP_IF_OFFSET_ANA_REG, tmp);

		tmp = S2LP_CALC_FILTER(f_dig);
		s2lp_iface_reg_write(dev, S2LP_IF_OFFSET_DIG_REG, tmp);

		/* channel filter */
		tmp = (config->mode == SIGFOX_MODE_CS300K_RX) ? 0x81 : 0x51;
		s2lp_iface_reg_write(dev, S2LP_CHFLT_REG, tmp);
		s2lp_iface_reg_write(dev, S2LP_PCKTCTRL3_REG, 0x20);

		/* here the lack of a break is intetional.
		 * it should do the case SFX_RF_MODE_TX also
		 */
	}
	case SIGFOX_MODE_TX:
		s2lp_sfx_rf_tx_init(dev);
		break;
	case SIGFOX_MODE_RX:
		s2lp_sfx_rf_rx_init(dev);
		break;
#if defined(CONFIG_SIGFOX_MONARCH_FEATURE)
	case SIGFOX_MODE_MONARCH:
		s2lp_sfx_rf_rx_monarch_init(dev);
		break;
#endif
	default:
		break;
	}

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_stop(const struct device *dev)
{
	struct s2lp_context *ctx = dev->data;

	LOG_DBG("");

	s2lp_sfx_trx_txrx_stop(dev);
	s2lp_iface_shutdown(dev);

	ctx->trx.tx_is_ready = 0;

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_send(const struct device *dev,
			    enum sigfox_mod_type modulation, uint8_t *buffer,
			    size_t len)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint8_t regs[4] = {0};
#if (S2LP_TX_BY_FIFO_EMPTY_DIO == 0)
	uint64_t timeout;
	uint32_t isr_status;
#endif

	LOG_HEXDUMP_DBG(buffer, len, "payload");

	if (modulation == SIGFOX_NO_MODULATION ||
	   ctx->trx.fsm != S2LP_TRX_FSM_STATE_IDLE) {
		return SIGFOX_ERR_RF_SEND;
	}

	/* The transmission will be a packet and not PN9 */
	ctx->trx.tx.continuous_tx = 0;
	ctx->trx.tx.payload = buffer;
	ctx->trx.tx.size = len;

	configure_s2lp_gpios(dev, (S2LP_TX_BY_FIFO_EMPTY_DIO > 0));

	regs[0] = regs[1] = 0;
	regs[2] = 0x01; /* TX FIFO almost empty interrupt */
	regs[3] = 0x24; /* TX FIFO underflow/overflow error | TX data sent */
	s2lp_iface_burst_write(dev, S2LP_IRQ_MASK3_REG, regs, 4);

	s2lp_sfx_rf_dbpsk_init(dev, modulation);
	s2lp_sfx_rf_load_first_ramp_up(dev);
	s2lp_iface_burst_read(dev, S2LP_IRQ_STATUS3_REG, regs, 4);

	if ((S2LP_TX_BY_FIFO_EMPTY_DIO > 0)) {
		gpio_pin_configure(ctx->irq_gpio,
				conf->dio[ctx->irq_dio_id].pin,
				conf->dio[ctx->irq_dio_id].flags | GPIO_INPUT);

		gpio_pin_interrupt_configure(ctx->irq_gpio,
					conf->dio[ctx->irq_dio_id].pin,
					GPIO_INT_EDGE_TO_ACTIVE);
	} else {
		gpio_pin_configure(ctx->irq_gpio,
				conf->dio[ctx->irq_dio_id].pin,
				(GPIO_ACTIVE_HIGH | GPIO_INPUT));
	}

	k_sem_reset(&ctx->trx_sync);
	s2lp_sfx_trx_tx_start(dev);

#if (S2LP_TX_BY_FIFO_EMPTY_DIO > 0)
	/* Wait TX finish */
	k_sem_take(&ctx->trx_sync, K_FOREVER);

	regs[2] = regs[3] = 0;
	s2lp_iface_burst_write(dev, S2LP_IRQ_MASK3_REG, regs, 4);

	LOG_DBG("ISR TX: FSM: %d", ctx->trx.fsm);

	if (ctx->trx.fsm != S2LP_TRX_FSM_STATE_IDLE) {
		ctx->trx.fsm = S2LP_TRX_FSM_STATE_IDLE;

		//return SIGFOX_ERR_RF_SEND;
	}
#else
	k_sched_lock();
	timeout = k_uptime_get() + 500;
	while (true) {
		if (gpio_pin_get(ctx->irq_gpio,
				 conf->dio[ctx->irq_dio_id].pin)) {
			s2lp_sfx_rf_tx_fsm(dev);
			if (ctx->trx.tx.state == S2LP_TX_STATE_NONE) {
				break;
			}
		}

		if (k_uptime_get() > timeout) {
			LOG_DBG("TX Timeout");
			break;
		}
	}
	k_sched_unlock();
	ctx->trx.fsm = S2LP_TRX_FSM_STATE_IDLE;
	s2lp_iface_burst_read(dev, S2LP_IRQ_STATUS3_REG,
				(uint8_t *) &isr_status,
				sizeof(uint32_t));
	isr_status = sys_be32_to_cpu(isr_status);
	LOG_DBG("POOLING TX: 0x%08x, FSM: %d", isr_status, ctx->trx.fsm);
	if (isr_status & S2LP_IRQ_MASK_TX_FIFO_FLOW_ERR) {
		LOG_DBG("TX FIFO underflow/overflow error");
		//return SIGFOX_ERR_RF_SEND;
	}
#endif

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_recv(const struct device *dev,
			    enum sigfox_rx_state *state, uint8_t *buffer,
			    int16_t *rssi)
{
	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_cca(const struct device *dev,
			   enum sigfox_rx_state *state, uint8_t cs_min,
			   int8_t cs_threshold)
{
	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_sw_freq(const struct device *dev, uint32_t frequency)
{
	s2lp_sfx_rf_change_freq(dev, frequency);

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_tx_cont_start(const struct device *dev,
				     enum sigfox_mod_type modulation)
{
	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_tx_cont_stop(const struct device *dev)
{
	struct s2lp_context *ctx = dev->data;

	s2lp_sfx_trx_txrx_stop(dev);

	ctx->trx.fsm = S2LP_TRX_FSM_STATE_IDLE;

	return SIGFOX_ERR_NONE;
}

static inline int configure_gpios(struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	int i;

	/* Chip SDN line */
	ctx->sdn_gpio = device_get_binding(conf->sdn.devname);
	if (ctx->sdn_gpio == NULL) {
		LOG_ERR("Failed to get instance of %s device",
			conf->sdn.devname);
		return -EINVAL;
	}
	gpio_pin_configure(ctx->sdn_gpio, conf->sdn.pin,
			   GPIO_OUTPUT_ACTIVE | conf->sdn.flags);

	LOG_DBG("SDN GPIO configured on %s:%u", conf->sdn.devname,
		conf->sdn.pin);

	/* Select IRQ line between dio0 and dio3. The install priority is
	 * defined from lower to high index value.
	 */
	ctx->irq_dio_id = -1;

	for (i = 0; i < 4; i++) {
		if ((conf->dio[i].func & S2LP_GPIO_SELECT) ==
		    (S2LP_GPIO_FUNC_N_IRQ << S2LP_GPIO_MODE)) {
			ctx->irq_dio_id = i;
			break;
		}
	}

	if (ctx->irq_dio_id < 0) {
		LOG_ERR("No IRQ GPIO available");
		return -EINVAL;
	}

	ctx->irq_gpio = device_get_binding(conf->dio[ctx->irq_dio_id].devname);
	if (ctx->irq_gpio == NULL) {
		LOG_ERR("Failed to get instance of %s device",
			conf->dio[ctx->irq_dio_id].devname);
		return -EINVAL;
	}

	gpio_pin_configure(ctx->irq_gpio,
			conf->dio[ctx->irq_dio_id].pin,
			conf->dio[ctx->irq_dio_id].flags | GPIO_INPUT);

	LOG_DBG("IRQ GPIO configured on %s:%u connected at DIO%d",
		conf->dio[ctx->irq_dio_id].devname,
		conf->dio[ctx->irq_dio_id].pin,
		ctx->irq_dio_id);

	return SIGFOX_ERR_NONE;
}

static inline int configure_spi(struct device *dev)
{
	struct s2lp_context *ctx = dev->data;
	const struct s2lp_config *conf = dev->config;

	/* Get SPI Driver Instance*/
	ctx->spi = device_get_binding(conf->spi.devname);
	if (!ctx->spi) {
		LOG_ERR("Failed to get instance of %s device",
			conf->spi.devname);

		return -ENODEV;
	}

	/* Apply SPI Config: 8-bit, MSB First, MODE-0 */
	ctx->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	ctx->spi_cfg.slave = conf->spi.addr;
	ctx->spi_cfg.frequency = conf->spi.freq;
	ctx->spi_cfg.cs = NULL;

	/*
	 * Get SPI Chip Select Instance
	 *
	 * This is an optinal feature configured on DTS. Some SPI controllers
	 * automatically set CS line by device slave address. Check your SPI
	 * device driver to understand if you need this option enabled.
	 */
	ctx->spi_cs.gpio_dev = device_get_binding(conf->spi.cs.devname);
	if (ctx->spi_cs.gpio_dev) {
		ctx->spi_cs.gpio_pin = conf->spi.cs.pin;
		ctx->spi_cs.gpio_dt_flags = conf->spi.cs.flags;
		ctx->spi_cs.delay = 40U;

		ctx->spi_cfg.cs = &ctx->spi_cs;

		LOG_DBG("SPI GPIO CS configured on %s:%u", conf->spi.cs.devname,
			conf->spi.cs.pin);
	}

	return SIGFOX_ERR_NONE;
}

static int power_on_and_setup(struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint32_t isr_status = 0;
	uint8_t partnum, version;
	int ret;

	s2lp_iface_shutdown(dev);
	k_usleep(10000);
	ret = s2lp_iface_wakeup(dev);
	if (ret) {
		LOG_INF("WakeUp timeout");
		return ret;
	}

	/* get device identification */
	partnum = s2lp_iface_reg_read(dev, S2LP_DEVICE_INFO1_REG);
	version = s2lp_iface_reg_read(dev, S2LP_DEVICE_INFO0_REG);

	LOG_DBG("Partnum: 0x%02X Version: 0x%02X", partnum, version);

	if ((partnum != S2LP_PARTNUM) || (version != S2LP_VERSION)) {
		LOG_ERR("Invalid transceiver");
		return -ENODEV;
	}

	if (configure_s2lp_gpios(dev, true) != 0) {
		LOG_ERR("Configuring S2LP GPIOS failed");
		return -EIO;
	}

	LOG_DBG("S2LP GPIO configured");

	/* Disable all interrupts route to DIO pin  */
	s2lp_iface_burst_write(dev, S2LP_IRQ_MASK3_REG,
			       (uint8_t *) &isr_status, 4);

	/* Clear all interrupts flags */
	s2lp_iface_burst_read(dev, S2LP_IRQ_STATUS3_REG,
			      (uint8_t *) &isr_status, 4);

	/* Install interrupt handler */
	gpio_init_callback(&ctx->irq_cb, s2lp_trx_isr_handler,
				BIT(conf->dio[ctx->irq_dio_id].pin));

	gpio_add_callback(ctx->irq_gpio, &ctx->irq_cb);

	/* Enable ISR */
	gpio_pin_interrupt_configure(ctx->irq_gpio,
				conf->dio[ctx->irq_dio_id].pin,
				GPIO_INT_EDGE_TO_ACTIVE);

	LOG_DBG("S2LP IRQ installed");

	return SIGFOX_ERR_NONE;
}

static int s2lp_sigfox_init(struct device *dev)
{
	struct s2lp_context *ctx = dev->data;

	LOG_DBG("Initialize S2-LP Transceiver");

	k_sem_init(&ctx->trx_sync, 0, 1);
	k_sem_init(&ctx->trx_wait_isr, 0, 1);

	if (configure_gpios(dev) != 0) {
		LOG_ERR("Configuring GPIOS failed");
		return -EIO;
	}

	LOG_DBG("GPIO configured");

	if (configure_spi(dev) != 0) {
		LOG_ERR("Configuring SPI failed");
		return -EIO;
	}

	LOG_DBG("SPI configured");

	if (power_on_and_setup(dev) != 0) {
		LOG_ERR("Configuring s2lp failed");
		return -EIO;
	}

	LOG_DBG("Power Up and Setup completed");

	k_thread_create(&ctx->trx_thread,
			ctx->trx_stack,
			CONFIG_SIGFOX_S2LP_RX_STACK_SIZE,
			(k_thread_entry_t) s2lp_trx_thread_main,
			dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&ctx->trx_thread, "sigfox_trx");

	return SIGFOX_ERR_NONE;
}

static const struct sigfox_driver_api s2lp_sigfox_api = {
	.start = s2lp_sigfox_start,
	.config = s2lp_sigfox_config,
	.stop = s2lp_sigfox_stop,
	.send = s2lp_sigfox_send,
	.recv = s2lp_sigfox_recv,
	.cca = s2lp_sigfox_cca,
	.sw_freq = s2lp_sigfox_sw_freq,
	.tx_cont_start = s2lp_sigfox_tx_cont_start,
	.tx_cont_stop = s2lp_sigfox_tx_cont_stop,
};

/*
 * Optional features place holders, get a 0 if the "gpio" doesn't exist
 */
#define DRV_INST_GPIO_LABEL(n, gpio_pha)				       \
	UTIL_AND(DT_INST_NODE_HAS_PROP(n, gpio_pha),			       \
		 DT_INST_GPIO_LABEL(n, gpio_pha))

#define DRV_INST_GPIO_PIN(n, gpio_pha)					       \
	UTIL_AND(DT_INST_NODE_HAS_PROP(n, gpio_pha),			       \
		 DT_INST_GPIO_PIN(n, gpio_pha))

#define DRV_INST_GPIO_FLAGS(n, gpio_pha)				       \
	UTIL_AND(DT_INST_NODE_HAS_PROP(n, gpio_pha),			       \
		 DT_INST_GPIO_FLAGS(n, gpio_pha))

#define DRV_INST_GPIO_FUNCTION(n, gpio_pha)				       \
	UTIL_AND(DT_INST_NODE_HAS_PROP(n, gpio_pha),			       \
		 DT_INST_PROP(n, gpio_pha))

#define DRV_INST_SPI_DEV_CS_GPIOS_LABEL(n)				       \
	UTIL_AND(DT_INST_SPI_DEV_HAS_CS_GPIOS(n),			       \
		 DT_INST_SPI_DEV_CS_GPIOS_LABEL(n))

#define DRV_INST_SPI_DEV_CS_GPIOS_PIN(n)				       \
	UTIL_AND(DT_INST_SPI_DEV_HAS_CS_GPIOS(n),			       \
		 DT_INST_SPI_DEV_CS_GPIOS_PIN(n))

#define DRV_INST_SPI_DEV_CS_GPIOS_FLAGS(n)				       \
	UTIL_AND(DT_INST_SPI_DEV_HAS_CS_GPIOS(n),			       \
		 DT_INST_SPI_DEV_CS_GPIOS_FLAGS(n))

#define DRV_DIO_DECL(n, m)						       \
	.dio[m].devname = DRV_INST_GPIO_LABEL(n, dio##m##_gpios),	       \
	.dio[m].pin = DRV_INST_GPIO_PIN(n, dio##m##_gpios),		       \
	.dio[m].flags = DRV_INST_GPIO_FLAGS(n, dio##m##_gpios),		       \
	.dio[m].func = DRV_INST_GPIO_FUNCTION(n, dio##m##_function)

#define SIGFOX_S2LP_DEVICE_CONFIG(n)					       \
	static const struct s2lp_config s2lp_sigfox_config_##n = {	       \
		DRV_DIO_DECL(n, 0),					       \
		DRV_DIO_DECL(n, 1),					       \
		DRV_DIO_DECL(n, 2),					       \
		DRV_DIO_DECL(n, 3),					       \
									       \
		.sdn.devname = DRV_INST_GPIO_LABEL(n, sdn_gpios),	       \
		.sdn.pin = DRV_INST_GPIO_PIN(n, sdn_gpios),		       \
		.sdn.flags = DRV_INST_GPIO_FLAGS(n, sdn_gpios),		       \
									       \
		.spi.devname = DT_INST_BUS_LABEL(n),			       \
		.spi.addr = DT_INST_REG_ADDR(n),			       \
		.spi.freq = DT_INST_PROP(n, spi_max_frequency),		       \
		.spi.cs.devname = DRV_INST_SPI_DEV_CS_GPIOS_LABEL(n),	       \
		.spi.cs.pin = DRV_INST_SPI_DEV_CS_GPIOS_PIN(n),		       \
		.spi.cs.flags = DRV_INST_SPI_DEV_CS_GPIOS_FLAGS(n),	       \
									       \
		.trx_freq = DT_INST_PROP(n, trx_clock_frequency),	       \
									       \
		.ramps.start_duration = DT_INST_PROP(n, trx_start_duration),   \
		.ramps.fdev_neg = DT_INST_PROP(n, trx_fdev_neg),	       \
		.ramps.fdev_pos = DT_INST_PROP(n, trx_fdev_pos),	       \
		.ramps.min_power = DT_INST_PROP(n, trx_min_power),	       \
		.ramps.max_power = DT_INST_PROP(n, trx_max_power),	       \
		.ramps.gain_f1 = DT_INST_PROP(n, trx_gain_f1),		       \
		.ramps.gain_f2 = DT_INST_PROP(n, trx_gain_f2),		       \
									       \
		.trx_ext_pa = DT_INST_NODE_HAS_PROP(n, trx_ext_pa),	       \
		.trx_ext_lna = DT_INST_NODE_HAS_PROP(n, trx_ext_lna),	       \
	};

#define SIGFOX_S2LP_DEVICE_DATA(n)					       \
	static struct s2lp_context s2lp_sigfox_data_##n = {		       \
		.iface = NULL,						       \
		.trx.tx.swap = 0,					       \
		.irq_dio_id = -1,					       \
	};

#define SIGFOX_S2LP_INIT(inst)						       \
	SIGFOX_S2LP_DEVICE_CONFIG(inst);				       \
	SIGFOX_S2LP_DEVICE_DATA(inst);					       \
	DEVICE_AND_API_INIT(s2lp_sifox_##inst, DT_INST_LABEL(inst),	       \
			    &s2lp_sigfox_init, &s2lp_sigfox_data_##inst,       \
			    &s2lp_sigfox_config_##inst, POST_KERNEL,	       \
			    CONFIG_SIGFOX_INIT_PRIO, &s2lp_sigfox_api);

DT_INST_FOREACH_STATUS_OKAY(SIGFOX_S2LP_INIT)
