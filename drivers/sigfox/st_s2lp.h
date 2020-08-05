/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_H_
#define ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_H_

#define S2LP_CALC_FILTER(f_in)		\
	((uint8_t)(((uint64_t)7372800000 / f_in) - 100))
#define S2LP_PN9_INITIALIZER            0x01FF
#if defined(CONFIG_SIGFOX_S2LP_TX_BY_FIFO_EMPTY_DIO)
#define S2LP_TX_BY_FIFO_EMPTY_DIO       0
#else
#define S2LP_TX_BY_FIFO_EMPTY_DIO       1
#endif

enum s2lp_trx_cmd_t {
	S2LP_CMD_TX                     = 0x60,
	S2LP_CMD_RX                     = 0x61,
	S2LP_CMD_READY                  = 0x62,
	S2LP_CMD_STANDBY                = 0x63,
	S2LP_CMD_SLEEP                  = 0x64,
	S2LP_CMD_LOCK_RX                = 0x65,
	S2LP_CMD_LOCK_TX                = 0x66,
	S2LP_CMD_TXRX_ABORT             = 0x67,
	S2LP_CMD_LCD_RELOAD             = 0x68,
	S2LP_CMD_RCO_CALIB              = 0x69,
	S2LP_CMD_RESET                  = 0x70,
	S2LP_CMD_FLUSH_RX_FIFO          = 0x71,
	S2LP_CMD_FLUSH_TX_FIFO          = 0x72,
	S2LP_CMD_AUTO_RETRANSMIT        = 0x73,
};

enum s2lp_trx_t {
	S2LP_STATE_READY                = 0x00,
	S2LP_STATE_SLEEP_A              = 0x01,
	S2LP_STATE_STANDBY              = 0x02,
	S2LP_STATE_SLEEP_B              = 0x03,
	S2LP_STATE_LOCK                 = 0x0C,
	S2LP_STATE_RX                   = 0x30,
	S2LP_STATE_SYNTH_SETUP          = 0x50,
	S2LP_STATE_TX                   = 0x5C,
};

enum s2lp_trx_fsm_state_t {
	S2LP_TRX_FSM_STATE_IDLE,
	S2LP_TRX_FSM_STATE_TX,
	S2LP_TRX_FSM_STATE_RX,
	S2LP_TRX_FSM_STATE_WAIT_TIMER,
	S2LP_TRX_FSM_STATE_WAIT_CLEAR_CH,
	S2LP_TRX_FSM_STATE_MONARCH_SCAN,
};

enum s2lp_trx_tx_state_t {
	S2LP_TX_STATE_NONE,
	S2LP_TX_STATE_RAMP_UP_1,
	S2LP_TX_STATE_RAMP_UP_2,
	S2LP_TX_STATE_DATA,
	S2LP_TX_STATE_CONTINUOS_BPSK,
	S2LP_TX_STATE_RAMP_DOWN_1,
	S2LP_TX_STATE_RAMP_DOWN_2,
	S2LP_TX_STATE_RAMP_DOWN_3,
	S2LP_TX_STATE_STOP,
};

enum s2lp_trx_ramp_buffer_t {
	S2LP_RAMP_FAST,
	S2LP_RAMP_CONST_FAST,
	S2LP_RAMP_DOWN_1,
	S2LP_RAMP_DOWN_2,
	S2LP_RAMP_UP_1,
	S2LP_RAMP_UP_2,
};

enum s2lp_trx_gpio_out_function_t {
	S2LP_GPIO_FUNC_N_IRQ,
	S2LP_GPIO_FUNC_N_POR,
	S2LP_GPIO_FUNC_WAKEUP,
	S2LP_GPIO_FUNC_LOW_BATTERY,
	S2LP_GPIO_FUNC_TX_CLK,
	S2LP_GPIO_FUNC_TX_STATE,
	S2LP_GPIO_FUNC_TXRX_FIFO_ALMOST_EMPTY,
	S2LP_GPIO_FUNC_TXRX_FIFO_ALMOST_FULL,
	S2LP_GPIO_FUNC_RX_DATA,
	S2LP_GPIO_FUNC_RX_CLK,
	S2LP_GPIO_FUNC_RX_STATE,
	S2LP_GPIO_FUNC_N_SLEEP_STANDBY,
	S2LP_GPIO_FUNC_STANDBY,
	S2LP_GPIO_FUNC_ANTENNA_DIVERSITY,
	S2LP_GPIO_FUNC_VALID_PREAMBLE,
	S2LP_GPIO_FUNC_SYNC_WORD,
	S2LP_GPIO_FUNC_RSSI_ABOVE_THR,
	S2LP_GPIO_FUNC_RESERVED_1,
	S2LP_GPIO_FUNC_PA_TX_RX,
	S2LP_GPIO_FUNC_GPIO_HIGH_LEVEL,
	S2LP_GPIO_FUNC_GPIO_LOW_LEVEL,
	S2LP_GPIO_FUNC_SMPS_EN,
	S2LP_GPIO_FUNC_SLEEP,
	S2LP_GPIO_FUNC_READY,
	S2LP_GPIO_FUNC_LOCK,
	S2LP_GPIO_FUNC_WAITING_LOCK,
	S2LP_GPIO_FUNC_TX_DATA_OOK,
	S2LP_GPIO_FUNC_WAITING_READY_2,
	S2LP_GPIO_FUNC_WAITING_TMR_TO_LP,
	S2LP_GPIO_FUNC_WAITING_VCO,
	S2LP_GPIO_FUNC_SYNTH_EN,
	S2LP_GPIO_FUNC_RESERVED_2,
};

enum s2lp_trx_irq_mask_t {
	/* Packet Oriented */
	S2LP_IRQ_MASK_RX_DATA_RDY		= 0x00000001,
	S2LP_IRQ_MASK_RX_DATA_DISCARDED		= 0x00000002,
	S2LP_IRQ_MASK_TX_DATA_SENT		= 0x00000004,
	S2LP_IRQ_MASK_TX_MAX_ATTEMPTS		= 0x00000008,
	S2LP_IRQ_MASK_CRC_ERROR			= 0x00000010,
	S2LP_IRQ_MASK_TX_FIFO_FLOW_ERR		= 0x00000020,
	S2LP_IRQ_MASK_RX_FIFO_FLOW_ERR		= 0x00000040,
	S2LP_IRQ_MASK_TX_FIFO_ALMOST_FULL	= 0x00000080,
	S2LP_IRQ_MASK_TX_FIFO_ALMOST_EMPTY	= 0x00000100,
	S2LP_IRQ_MASK_RX_FIFO_ALMOST_FULL	= 0x00000200,
	S2LP_IRQ_MASK_RX_FIFO_ALMOST_EMPTY	= 0x00000400,
	S2LP_IRQ_MASK_CCA_MAX_BACKOFF_REACH	= 0x00000800,

	/* Signal Quality */
	S2LP_IRQ_MASK_VALID_PREAMBLE_DETECTED	= 0x00001000,
	S2LP_IRQ_MASK_SYNC_WORD_DETECTED	= 0x00002000,
	S2LP_IRQ_MASK_RSSI_ABOVE_THRESHOLD	= 0x00004000,

	/* Device Status */
	S2LP_IRQ_MASK_WAKEUP_TIMEOUT		= 0x00008000,
	S2LP_IRQ_MASK_READY			= 0x00010000,
	S2LP_IRQ_MASK_STANDBY_IN_PROGRESS	= 0x00020000,
	S2LP_IRQ_MASK_LOW_BATTERY		= 0x00040000,
	S2LP_IRQ_MASK_POWER_ON_RESET		= 0x00080000,

	/* Reserved bits from 20 up to 27 */

	/* Timer */
	S2LP_IRQ_MASK_RX_TIMEOUT		= 0x10000000,
	S2LP_IRQ_MASK_SNIFF_TIMEOUT		= 0x20000000,
};

struct s2lp_dt_gpio_t {
	const char *devname;
	uint32_t pin;
	uint32_t flags;
	uint32_t func;
};

struct s2lp_dt_spi_t {
	const char *devname;
	uint32_t freq;
	uint32_t addr;
	struct s2lp_dt_gpio_t cs;
};

struct s2lp_dt_trx_t {
	uint8_t fdev_neg;
	uint8_t fdev_pos;
	uint8_t start_duration;
	uint8_t min_power;
	uint8_t max_power;
	uint8_t gain_f1;
	uint8_t gain_f2;
};

struct s2lp_tx_state_t {
	enum s2lp_trx_tx_state_t state;
	uint8_t  ramp_buffer[82];
	uint8_t  ramp_len;
	uint8_t *payload;
	uint16_t size;
	uint16_t byte_idx;
	uint16_t bit_idx;
	uint16_t pn9;
	uint8_t  continuous_tx;

	volatile uint8_t swap;
};

struct s2lp_config {
	struct s2lp_dt_gpio_t dio[4];
	struct s2lp_dt_gpio_t sdn;

	struct s2lp_dt_spi_t spi;

	struct s2lp_dt_trx_t ramps;
	uint32_t trx_freq;
	bool trx_ext_pa;
	bool trx_ext_lna;
};

struct s2lp_trx_state {
	struct s2lp_tx_state_t tx;
	volatile uint8_t smps_mode;
	/* The tx_is_ready flag is used for CS (ARIB only) and is used in the
	 * rf_init for SFX_RF_MODE_CS_RX. When the rf_init is called with
	 * SIGFOX_MODE_CS_RX, configure also the TX to be faster in case of TX
	 */
	uint8_t tx_is_ready;
	enum s2lp_trx_fsm_state_t fsm;
};

struct s2lp_context {
	struct net_if *iface;

	struct device *sdn_gpio;
	struct device *irq_gpio;

	struct device *spi;
	struct spi_config spi_cfg;
	struct spi_cs_control spi_cs;

	struct gpio_callback irq_cb;
	int irq_dio_id;

	struct k_thread trx_thread;

	K_THREAD_STACK_MEMBER(trx_stack,
			      CONFIG_SIGFOX_S2LP_RX_STACK_SIZE);
	struct k_sem trx_wait_isr;
	struct k_sem trx_sync;

	struct s2lp_trx_state trx;
};

#endif /* ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_H_ */
