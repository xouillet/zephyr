/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/sigfox.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <zephyr.h>

#include "st_s2lp.h"
#include "st_s2lp_regs.h"
#include "st_s2lp_iface.h"
#include "st_s2lp_rf.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(st_s2lp_rf, CONFIG_SIGFOX_S2LP_LOG_LEVEL);

static uint16_t s2lp_freq_get_nearest(uint64_t carrier, uint32_t freq,
				      uint64_t base)
{
	uint64_t tgt1, tgt2;
	uint16_t value;

	/* dr_num = (2**32) * 600 */
	value = (uint16_t) (carrier / freq - base);

	/* understand if we are getting the nearest integer */
	tgt1 = (uint64_t) freq * ((uint64_t) value + base);
	tgt2 = (uint64_t) freq * ((uint64_t) value + 1 + base);
	if (carrier - tgt1 > tgt2 - carrier) {
		value++;
	}

	return value;
}

/* NEED review because we remove RAW frame and use burst FIFO W */
static void s2lp_sfx_rf_gen_tx_ramp(const struct device *dev,
				    enum s2lp_trx_ramp_buffer_t type,
				    uint8_t *fifo, uint8_t *len)
{
	const struct s2lp_config *conf = dev->config;
	uint8_t start_ramp[72] = {0};
	uint8_t start_ramp_tail[27] = { 8,  8,  9,  9, 10,
				       10, 11, 11, 13, 13,
				       15, 16, 17, 18, 20,
				       22, 24, 26, 31, 35,
				       40, 45, 51, 60, 67,
				       70, 80};
	uint8_t aux_buf[] = { 1,  1,  2,   2,
			      3,  4,  6,   8,
			     11, 14, 19,  23,
			     27, 31, 44, 220};
	uint8_t i, j;

	if (conf->ramps.gain_f1 == 0x00) {
		aux_buf[9]  = 15;
		aux_buf[10] = 20;
		aux_buf[11] = 24;
		aux_buf[12] = 30;
		aux_buf[13] = 39;
		aux_buf[14] = 54;
	}

	switch (type) {
	case S2LP_RAMP_FAST:
	{
		*len = 80;

		for (i = 0; i < 15; i++) {
			aux_buf[i]          += conf->ramps.gain_f1;
		}

		for (i = 0, j = 0; i < (16 * 2); i += 2, j++) {
			if (i < 16) {
				fifo[i]     = 0;
				fifo[i + 1] = conf->ramps.max_power;
			}

			fifo[14 + i]        = 0;
			fifo[14 + i + 1]    = aux_buf[j];
		}

		for (i = 0, j = 0; i < (16 * 2); i += 2, j++) {
			fifo[80 - i - 2]    = 0;
			fifo[80 - i - 1]    = aux_buf[j];
		}
		break;
	}
	case S2LP_RAMP_CONST_FAST:
	{
		*len = 80;

		for (i = 0; i < 80; i += 2) {
			fifo[i]     = 0;
			fifo[i + 1] = conf->ramps.max_power;
		}

		break;
	}
	case S2LP_RAMP_DOWN_1:
	{
		*len = 80;

		for (i = 0; i < 46; i++) {
			if (i < 10) {
				start_ramp[i] = 1;
			}
			if (i > 9 && i < 18) {
				start_ramp[i] = 2;
			}
			if (i > 17 && i < 25) {
				start_ramp[i] = 3;
			}
			if (i > 24 && i < 31) {
				start_ramp[i] = 4;
			}
			if (i > 30 && i < 36) {
				start_ramp[i] = 5;
			}
			if (i > 35 && i < 41) {
				start_ramp[i] = 6;
			}
			if (i > 40 && i < 45) {
				start_ramp[i] = 7;
			}
		}

		memcpy(start_ramp + 45, start_ramp_tail,
		       sizeof(start_ramp_tail));

		for (i = 0; i < 72; i++) {
			start_ramp[i] += conf->ramps.gain_f1;
		}

		for (i = 0; i < 16; i += 2) {
			fifo[i]     = 0;
			fifo[i + 1] = conf->ramps.max_power;
		}

		for (i = 0, j = 0; i < (32 * 2); i += 2, j++) {
			fifo[2 * 41 - i - 1] = start_ramp[32 - j - 1] +
					       conf->ramps.gain_f2;
			fifo[2 * 41 - i - 2] = 0;
		}
		break;
	}
	case S2LP_RAMP_DOWN_2:
	{
		*len = 80;

		for (i = 0; i < 46; i++) {
			if (i < 10) {
				start_ramp[i] = 1;
			}
			if (i > 9 && i < 18) {
				start_ramp[i] = 2;
			}
			if (i > 17 && i < 25) {
				start_ramp[i] = 3;
			}
			if (i > 24 && i < 31) {
				start_ramp[i] = 4;
			}
			if (i > 30 && i < 36) {
				start_ramp[i] = 5;
			}
			if (i > 35 && i < 41) {
				start_ramp[i] = 6;
			}
			if (i > 40 && i < 45) {
				start_ramp[i] = 7;
			}
		}

		memcpy(start_ramp + 45, start_ramp_tail,
		       sizeof(start_ramp_tail));

		for (i = 0; i < 72; i++) {
			start_ramp[i] += conf->ramps.gain_f1;
		}

		for (i = 0, j = 0; i < (40 * 2); i += 2, j++) {
			fifo[2 * 40 - i - 2] = 0;
			fifo[2 * 40 - i - 1] = start_ramp[72 - j - 1] +
					       conf->ramps.gain_f2;
		}

		break;
	}
	case S2LP_RAMP_UP_1:
	{
		*len = 80;

		for (i = 0; i < 46; i++) {
			if (i < 10) {
				start_ramp[i] = 1;
			}
			if (i > 9 && i < 18) {
				start_ramp[i] = 2;
			}
			if (i > 17 && i < 25) {
				start_ramp[i] = 3;
			}
			if (i > 24 && i < 31) {
				start_ramp[i] = 4;
			}
			if (i > 30 && i < 36) {
				start_ramp[i] = 5;
			}
			if (i > 35 && i < 41) {
				start_ramp[i] = 6;
			}
			if (i > 40 && i < 45) {
				start_ramp[i] = 7;
			}
		}

		memcpy(start_ramp + 45, start_ramp_tail,
		       sizeof(start_ramp_tail));

		for (i = 0; i < 72; i++) {
			start_ramp[i] += conf->ramps.gain_f1;
		}

		for (i = 0, j = 0; i < 79; i += 2, j++) {
			fifo[i]     = 0;
			fifo[i + 1] = start_ramp[72 - j - 1] +
				      conf->ramps.gain_f2;
		}

		break;
	}
	case S2LP_RAMP_UP_2:
	{
		*len = 64;

		for (i = 0; i < 46; i++) {
			if (i < 10) {
				start_ramp[i] = 1;
			}
			if (i > 9 && i < 18) {
				start_ramp[i] = 2;
			}
			if (i > 17 && i < 25) {
				start_ramp[i] = 3;
			}
			if (i > 24 && i < 31) {
				start_ramp[i] = 4;
			}
			if (i > 30 && i < 36) {
				start_ramp[i] = 5;
			}
			if (i > 35 && i < 41) {
				start_ramp[i] = 6;
			}
			if (i > 40 && i < 45) {
				start_ramp[i] = 7;
			}
		}

		memcpy(start_ramp + 45, start_ramp_tail,
		       sizeof(start_ramp_tail));

		for (i = 0; i < 72; i++) {
			start_ramp[i] += conf->ramps.gain_f1;
		}

		for (i = 0, j = 40; i < 63; i += 2, j++) {
			fifo[i]     = 0;
			fifo[i + 1] = start_ramp[72 - j - 1] +
				      conf->ramps.gain_f2;
		}

		break;
	}
	}
}

/* Function to generate PN9 */
static void s2lp_sfx_rf_tx_pn9_next(uint16_t *last)
{
	uint16_t retval;

	retval =  (((*last & 0x20) >> 5) ^ *last) << 8;
	retval |= (*last >> 1) & 0xff;
	*last  =  retval & 0x1ff;
}

/* See s2lp_sigfox_tx_cont_start for S2LP_PN9_INITIALIZER */
static uint16_t s2lp_sfx_rf_tx_pn9_next_byte(uint16_t state)
{
	int i;

	for (i = 0; i < 8; i++) {
		s2lp_sfx_rf_tx_pn9_next(&state);
	}

	return state;
}

/* Function to transmit a single bit */
static void s2lp_sfx_rf_tx_dbpsk_single_bit(const struct device *dev,
					    uint8_t bit)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;

	if (bit) {
		/* constant pattern that does not change the instantaneous
		 * frequency and keeps power constant to max.
		 */
		s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_CONST_FAST,
					ctx->trx.tx.ramp_buffer,
					&ctx->trx.tx.ramp_len);
	} else {
		/* Give FDEV a peak in the FDEV_PEAK position. This value
		 * should be the opposite of the last one.
		 */
		s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_FAST,
					ctx->trx.tx.ramp_buffer,
					&ctx->trx.tx.ramp_len);

		ctx->trx.tx.swap = (ctx->trx.tx.swap == conf->ramps.fdev_neg) ?
				   conf->ramps.fdev_pos :
				   conf->ramps.fdev_neg;

		ctx->trx.tx.ramp_buffer[80 - 30] = ctx->trx.tx.swap;
	}

	s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
			       ctx->trx.tx.ramp_buffer,
			       ctx->trx.tx.ramp_len);
}

#if defined(CONFIG_SIGFOX_S2LP_HAS_EXTERNAL_PA_LNA)
#if !defined(CONFIG_SIGFOX_S2LP_CUSTOM_EXTERNAL_PA_LNA)
void s2lp_sfx_trx_fem(const struct device *dev,
		     enum s2lp_sfx_trx_fem_fsm_t fem)
{
	uint8_t regs[3] = { 0 };

	switch(fem) {
	case S2LP_FEM_FSM_TX_PA:
	{
		regs[2] = S2LP_GPIO_HIGH_LEVEL; /* CSD */
		regs[0] = S2LP_GPIO_HIGH_LEVEL; /* CTX */
		regs[1] = S2LP_GPIO_HIGH_LEVEL; /* CPS */
		break;
	}
	case S2LP_FEM_FSM_TX_BYPASS:
	{
		regs[2] = S2LP_GPIO_HIGH_LEVEL;
		regs[0] = S2LP_GPIO_HIGH_LEVEL;
		regs[1] = S2LP_GPIO_LOW_LEVEL;
		break;
	}
	case S2LP_FEM_FSM_RX_LNA:
	{
		regs[2] = S2LP_GPIO_HIGH_LEVEL;
		regs[0] = S2LP_GPIO_LOW_LEVEL;
		regs[1] = S2LP_GPIO_HIGH_LEVEL;
		break;
	}
	case S2LP_FEM_FSM_RX_BYPASS:
	{
		regs[2] = S2LP_GPIO_HIGH_LEVEL;
		regs[0] = S2LP_GPIO_LOW_LEVEL;
		regs[1] = S2LP_GPIO_LOW_LEVEL;
		break;
	}
	case S2LP_FEM_FSM_INIT:
	{
		s2lp_iface_reg_write(dev, S2LP_PA_POWER7_REG, 0x25);
		/* without break intentionally */
	}
	default:
		regs[2] = S2LP_GPIO_LOW_LEVEL;
		regs[0] = S2LP_GPIO_LOW_LEVEL;
		regs[1] = S2LP_GPIO_LOW_LEVEL;
		break;
	}

	s2lp_iface_burst_write(dev, S2LP_GPIO0_CONF_REG, regs, sizeof(regs));
}
#endif /* CONFIG_SIGFOX_S2LP_CUSTOM_EXTERNAL_PA_LNA */
#else
#define s2lp_sfx_trx_fem(dev, fem)
#endif /* CONFIG_SIGFOX_S2LP_HAS_EXTERNAL_PA_LNA */

void s2lp_sfx_trx_fem_init(const struct device *dev)
{
	s2lp_sfx_trx_fem(dev, S2LP_FEM_FSM_INIT);
}
void s2lp_sfx_trx_tx_start(const struct device *dev)
{
#if defined(CONFIG_SIGFOX_S2LP_HAS_EXTERNAL_PA_LNA)
	const struct s2lp_config *conf = dev->config;
#endif

	s2lp_sfx_trx_fem(dev, conf->trx_ext_pa ?
			      S2LP_FEM_FSM_TX_PA   :
			      S2LP_FEM_FSM_TX_BYPASS);
	s2lp_iface_cmd(dev, S2LP_CMD_TX);
}

void s2lp_sfx_trx_rx_start(const struct device *dev)
{
#if defined(CONFIG_SIGFOX_S2LP_HAS_EXTERNAL_PA_LNA)
	const struct s2lp_config *conf = dev->config;
#endif

	s2lp_sfx_trx_fem(dev, conf->trx_ext_lna ?
			      S2LP_FEM_FSM_RX_LNA   :
			      S2LP_FEM_FSM_RX_BYPASS);
	s2lp_iface_cmd(dev, S2LP_CMD_RX);
}

void s2lp_sfx_trx_txrx_stop(const struct device *dev)
{
	s2lp_iface_cmd(dev, S2LP_CMD_TXRX_ABORT);
	s2lp_iface_cmd(dev, S2LP_CMD_FLUSH_RX_FIFO);
	s2lp_iface_cmd(dev, S2LP_CMD_FLUSH_TX_FIFO);

	s2lp_sfx_trx_fem(dev, S2LP_FEM_FSM_SHUTDOWN);
}

/* Function used both for BPSK MOD and CONTINUOS BPSK */
void s2lp_sfx_rf_load_first_ramp_up(const struct device *dev)
{
	struct s2lp_context *ctx = dev->data;

	s2lp_iface_cmd(dev, S2LP_CMD_FLUSH_TX_FIFO);

	memset(ctx->trx.tx.ramp_buffer, 0, 0x10);
	s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
			       ctx->trx.tx.ramp_buffer, 0x10);

	s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_UP_1,
			       ctx->trx.tx.ramp_buffer,
			       &ctx->trx.tx.ramp_len);

	s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
			       ctx->trx.tx.ramp_buffer,
			       ctx->trx.tx.ramp_len);

	ctx->trx.tx.state = S2LP_RAMP_UP_2;
	ctx->trx.fsm = S2LP_TRX_FSM_STATE_TX;
}


#if defined(CONFIG_SIGFOX_MONARCH_FEATURE)
void s2lp_sfx_rf_rx_monarch_init(const struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint32_t f_dig;
	uint16_t dr_m;
	uint8_t regs[3];

	/* It informs that monarch scan is ongoing */
	ctx->trx.tx.continuous_tx = 1;

	f_dig = conf->trx_freq;

	/* The digital frequency is half of xtal frequency
	 * for high freq xtals cuts
	 */
	if (f_dig > S2LP_DIG_DOMAIN_XTAL_THRESH) {
		f_dig >>= 1;
	}

	/* SET THE IF TO 300 KHz - It is necessary especially for
	 * low frequency xtals (Both IF equal)
	 */
	regs[0] = (uint8_t) (((uint64_t)7372800000 / conf->trx_freq) - 100);
	regs[1] = (uint8_t) (((uint64_t)7372800000 / f_dig) - 100);
	s2lp_iface_burst_write(dev, S2LP_IF_OFFSET_ANA_REG, regs, 2);

	/* DATARATE MANTISSA COMPUTATION FOR EXP=6 */
	dr_m = s2lp_freq_get_nearest(S2LP_MONARCH_DATARATE, f_dig, 65536);

	/* WRITE DATARATE + MOD TYPE REGS */
	regs[0] = (dr_m >> 8) & 0xFF;  /* MOD4 DATA_RATE_M[15:8] */
	regs[1] = (dr_m) & 0xFF;       /* MOD3 DATA_RATE_M[7:0]  */
	regs[2] = 0x56;                /* MOD2 MOD TYPE [7:4] +
					*   DATA_RATE_E [3:0] = OOK + EXP=6
					*/
	s2lp_iface_burst_write(dev, S2LP_MOD4_REG, regs, 3);

	/* Channel filter */
	/* CHFLT MANTISSA [7:4] +EXPONENT [3:0] 20Khz */
	s2lp_iface_reg_write(dev, S2LP_CHFLT_REG, 0x84);

	/* OOK decay 0 + RSSI TH */
	regs[0] = 0xE0; /* DECAY */
	regs[1] = 0x15; /* RSSI TH old 18 */
	s2lp_iface_burst_write(dev, S2LP_RSSI_FLT_REG, regs, 2);

	/* S2LP GPIO3 Configuration */
#ifdef MONARCH_GPIO_SAMPLING
	regs[0] = 0x42; /* GPIO3_CONF RX data output, OOK Output */
#else
	/* MONARCH SCAN to mux gpio it handler */
	ctx->trx.fsm = S2LP_TRX_FSM_STATE_MONARCH_SCAN;
	regs[0] = 0x3A; /* GPIO3_CONF: TX/RX FIFO ALMOST FULL FLAG */
#endif
	s2lp_iface_reg_write(dev, S2LP_GPIO3_CONF_REG, regs[0]);

	/* SET FIFO */
#ifndef MONARCH_GPIO_SAMPLING
	/* Set Almost full Mux Sel to Select RX fifo */
	regs[0] = s2lp_iface_reg_read(dev, S2LP_PROTOCOL2_REG) | 0x04;

	/* PROTOCOL2 REG --> FIFO_GPIO_OUT_MUX_SEL = 1 */
	s2lp_iface_reg_write(dev, S2LP_PROTOCOL2_REG, regs[0]);
#endif

	/* DIRECT RX THROUGH GPIO/FIFO BYPASSING PKT HANDLER */
#ifdef MONARCH_GPIO_SAMPLING
	regs[0] = 0x20; /* THROUGH GPIO */
#else
	regs[0] = 0x10; /* THROUGH FIFO */
#endif
	s2lp_iface_reg_write(dev, S2LP_PCKTCTRL3_REG, regs[0]);

	/* RX TIMEOUT 0 - TIMERS5 */
	s2lp_iface_reg_write(dev, S2LP_TIMERS5_REG, 0x00);

	/* AGCCTRL5 - AGC Conf */
	s2lp_iface_reg_write(dev, S2LP_AGCCTRL5_REG, 0xF0);

	/* AGCCTRL4 - AGC Conf, Philippe configuration */
	s2lp_iface_reg_write(dev, S2LP_AGCCTRL4_REG, 0x00);
}
#endif

void s2lp_sfx_rf_tx_init(const struct device *dev)
{
	struct s2lp_context *ctx = dev->data;

	/* Select Empty/Full control for TX FIFO */
	s2lp_iface_bit_write(dev, S2LP_PROTOCOL2_REG,
			     S2LP_FIFO_GPIO_OUT_MUX_SEL, 0, 0x00);
	/* TX in direct via FIFO mode */
	s2lp_iface_reg_write(dev, S2LP_PCKTCTRL1_REG, 0x04);

	s2lp_iface_reg_write(dev, S2LP_PA_POWER0_REG, 0x07);

	/* disable NEWMODE */
	s2lp_iface_bit_write(dev, S2LP_PA_CONFIG1_REG, S2LP_FIR_EN, 0, 0);
	s2lp_iface_reg_write(dev, S2LP_SYNTH_CONFIG2_REG, 0xD7);

	/* SMPS switch to 3MHz - ref. Datasheet page 23*/
	s2lp_iface_reg_write(dev, S2LP_PM_CONF3_REG, 0x87);
	s2lp_iface_reg_write(dev, S2LP_PM_CONF2_REG, 0xFC);
	s2lp_iface_reg_write(dev, S2LP_PA_CONFIG0_REG, 0xC8);

#ifdef MON_REF_DES
	if (st_manuf_context->pa_flag == 0) {
		ctx->trx.smps_mode = 7;
	}
#endif

	/* Set the SMPS only for values from 1 to 7 (1.8V) */
	if (ctx->trx.smps_mode > 0 && ctx->trx.smps_mode < 8) {
		/* (16dBm setting, SMPS to 1.8V -> smps_mode = 7) */
		s2lp_iface_reg_write(dev, S2LP_PM_CONF0_REG,
				     (ctx->trx.smps_mode << 4) | 0x02);

		if (ctx->trx.smps_mode > 4) {
			s2lp_iface_reg_write(dev, S2LP_PA_CONFIG0_REG, 0x88);
		}
	}

	/* FIFO AE threshold to 48 bytes */
	s2lp_iface_reg_write(dev, S2LP_FIFO_CONFIG0_REG, 48);

	ctx->trx.tx_is_ready = 1;
}

void s2lp_sfx_rf_rx_init(const struct device *dev)
{
	const struct s2lp_config *conf = dev->config;
	struct s2lp_context *ctx = dev->data;
	uint32_t f_dig;
	uint16_t dr_m;
	uint8_t mod_e, fdev_e, fdev_m;
	uint8_t regs[5];

	f_dig = conf->trx_freq;
	if (f_dig > S2LP_DIG_DOMAIN_XTAL_THRESH) {
		f_dig >>= 1;
	}

	/* modulation is 2GFSK01 @600bps | DR EXPONENT = 1 */
	mod_e = 0x20 | 0x01;

	/* dr_num = (2**32) * 600 */
	dr_m = s2lp_freq_get_nearest(0x25800000000, f_dig, 65536);

	fdev_e = 0;

	/* fdev_num=((2**22)*800) - FDEV=800Hz */
	fdev_m = s2lp_freq_get_nearest(3355443200, conf->trx_freq, 256);

	regs[0] = (uint8_t) (((uint64_t)7372800000 / conf->trx_freq) - 100);
	regs[1] = (uint8_t) (((uint64_t)7372800000 / f_dig) - 100);
	s2lp_iface_burst_write(dev, S2LP_IF_OFFSET_ANA_REG, regs, 2);

	/* write DATARATE mantissa and exponent */
	regs[0] = (dr_m >> 8) & 0xFF;
	regs[1] = (dr_m) & 0xFF;
	regs[2] = mod_e;
	s2lp_iface_burst_write(dev, S2LP_MOD4_REG, regs, 3);

	/* write FDEV mantissa and exponent */
	regs[0] = fdev_e;
	regs[1] = fdev_m;
	s2lp_iface_burst_write(dev, S2LP_MOD1_REG, regs, 2);

	/* Channel Filter */
#ifdef MON_REF_DES
	/* 3.3 KHz - Since there is LNA, we are able to receive even with a
	 * larger filter
	 */
	regs[0] = 0x18;
#else
	/* CHFILTER must be multiplied for Fdig/26e6 to obtain the actual
	 * value (f_dig > 24500000 = 2.1 KHz)
	 */
	regs[0] = (f_dig > 24500000) ? 0x88 : 0x68;
#endif
	s2lp_iface_reg_write(dev, S2LP_CHFLT_REG, regs[0]);

	/* settings of the packet (CRC, FEC, WHIT, ...) + packet length to 15 */
	regs[0] = regs[1] = regs[2] = regs[3] = 0;
	regs[4] = 15;
	s2lp_iface_burst_write(dev, S2LP_PCKTCTRL3_REG, regs, 5);

	/* SYNC LEN */
	s2lp_iface_reg_write(dev, S2LP_PCKTCTRL6_REG, 0x40);

	/* SYNC WORD 16 bits 0xB227 */
	regs[0] = 0x27;
	regs[1] = 0xB2;
	s2lp_iface_burst_write(dev, S2LP_SYNC1_REG, regs, 2);

	/* SYNC WORD LENGTH is 16bits */
	regs[0] = 0x40;
	regs[1] = 0x00;
	s2lp_iface_burst_write(dev, S2LP_PROTOCOL2_REG, regs, 2);

	/* equ ctrl and cs blank */
	s2lp_iface_reg_write(dev, S2LP_ANT_SELECT_CONF_REG, 0x00);

	/* RSSI thr */
	s2lp_iface_reg_write(dev, S2LP_RSSI_TH_REG, 0x07);

	/* CLK rec fast */
	s2lp_iface_reg_write(dev, S2LP_CLOCKREC0_REG, 0x70);

	/* CLK rec slow */
	s2lp_iface_reg_write(dev, S2LP_CLOCKREC1_REG, 0x20);

	/* AFC */
	s2lp_iface_reg_write(dev, S2LP_AFC2_REG, 0x00);

	/* SMPS switch to 0x88 0x00 - old */
	/* SMPS switch to 0x87 0xFC - new */
	regs[0] = 0x88;
	regs[1] = 0x00;
	s2lp_iface_burst_write(dev, S2LP_PM_CONF3_REG, regs, 2);

	ctx->trx.tx_is_ready = 0;
}

void s2lp_sfx_rf_dbpsk_init(const struct device *dev,
			    enum sigfox_mod_type modulation)
{
	const struct s2lp_config *conf = dev->config;
	uint32_t f_dig;
	uint16_t dr_m;
	uint8_t mod_e, fdev_e, fdev_m;
	uint8_t regs[3];

	f_dig = conf->trx_freq;
	if (f_dig > S2LP_DIG_DOMAIN_XTAL_THRESH) {
		f_dig >>= 1;
	}

	/* ETSI datarate is 100bps - chip datarate 500 (500 * 8 / 40 = 100) */
	if (modulation == SIGFOX_DBPSK_100BPS) {
		/* modulation is POLAR | DR EXPONENT = 1 */
		mod_e = 0x60 | 0x01;

		/* dr_num=(2**32)*500 */
		dr_m = s2lp_freq_get_nearest(0x1f400000000, f_dig, 65536);
		fdev_e = (conf->trx_freq > S2LP_DIG_DOMAIN_XTAL_THRESH) ? 0 : 1;

		/* fdev_num=((2**22)*2000) */
		fdev_m = s2lp_freq_get_nearest(8388608000, conf->trx_freq,
					       256 * fdev_e);
	}
	/* FCC datarate is 600bps - chip datarate 3000 (3000 * 8 / 40 = 600) */
	else if (modulation == SIGFOX_DBPSK_600BPS) {
		/* To consider also a margin used to compensate static drifts */
		if (f_dig > 24500000) {
			/* modulation is POLAR | DR EXPONENT = 3
			 * for xtal > 24 MHz
			 */
			mod_e = 0x60 | 0x03;

			/* dr_num=(2**30)*3000 */
			dr_m = s2lp_freq_get_nearest(0x2EE00000000, f_dig,
						     65536);
		} else {
			/* modulation is POLAR | DR EXPONENT = 4
			 * for xtal = 24 MHz
			 */
			mod_e = 0x60 | 0x04;

			/* dr_num=(2**30)*3000 */
			dr_m = s2lp_freq_get_nearest(0x17700000000, f_dig,
						     65536);
		}

		if (conf->trx_freq > S2LP_DIG_DOMAIN_XTAL_THRESH) {
			fdev_e = 2;
			fdev_m = s2lp_freq_get_nearest(25165824000,
						       conf->trx_freq, 256);
		} else if (f_dig > 24500000) {
			fdev_e = 3;
			fdev_m = s2lp_freq_get_nearest(12582912000,
						       conf->trx_freq, 256);
		} else {
			fdev_e = 4;
			fdev_m = s2lp_freq_get_nearest(6291456000,
						       conf->trx_freq, 256);
		}
	} else {
		LOG_ERR("No modulation is not supported");

		dr_m = 0;
		mod_e = 0;
		fdev_e = 0;
		fdev_m = 0;
	}

	/* write DATARATE mantissa and exponent */
	regs[0] = (dr_m >> 8) & 0xFF;
	regs[1] = (dr_m) & 0xFF;
	regs[2] = mod_e;
	s2lp_iface_burst_write(dev, S2LP_MOD4_REG, regs, 3);

	/* write FDEV mantissa and exponent
	 * here the exponent is in | with 0x80 to enable the digital
	 * smooth of the ramps
	 */
	regs[0] = fdev_e | 0x80;
	regs[1] = fdev_m;
	s2lp_iface_burst_write(dev, S2LP_MOD1_REG, regs, 2);
}

void s2lp_sfx_rf_change_freq(const struct device *dev, uint32_t frequency)
{
	const struct s2lp_config *conf = dev->config;
	uint64_t tgt1, tgt2;
	uint32_t synth;
	uint32_t vcofreq;
	uint8_t regs[4];
	uint8_t cp_isel, pfd_split;

	LOG_DBG("Frequency=%d", frequency);

	synth = ((uint64_t)2097152 * frequency / conf->trx_freq);
	tgt1  = (uint64_t)conf->trx_freq * ((uint64_t)synth);
	tgt2  = (uint64_t)conf->trx_freq * ((uint64_t)synth + 1);
	if ((uint64_t)2097152 * frequency - tgt1 >
	    tgt2 - (uint64_t)2097152 * frequency) {
		synth++;
	}

	/* CHARGE PUMP */
	vcofreq = (frequency * 4);

	/* Set the correct charge pump word */
	if (vcofreq >= (uint64_t)3600000000) {
		if (conf->trx_freq > S2LP_DIG_DOMAIN_XTAL_THRESH) {
			cp_isel = 0x02;
			pfd_split = 0;
		} else {
			cp_isel = 0x01;
			pfd_split = 1;
		}
	} else {
		if (conf->trx_freq > S2LP_DIG_DOMAIN_XTAL_THRESH) {
			cp_isel = 0x03;
			pfd_split = 0;
		} else {
			cp_isel = 0x02;
			pfd_split = 1;
		}
	}

	regs[0] = s2lp_iface_reg_read(dev, S2LP_SYNTH_CONFIG2_REG);
	regs[0] &= (~0x04);
	regs[0] |= (pfd_split << 2);
	s2lp_iface_reg_write(dev, S2LP_SYNTH_CONFIG2_REG, regs[0]);

	regs[0] = (((uint8_t)(synth >> 24)) & 0x0F) | (cp_isel << 5);
	regs[1] = (uint8_t)(synth >> 16);
	regs[2] = (uint8_t)(synth >> 8);
	regs[3] = (uint8_t)synth;
	s2lp_iface_burst_write(dev, S2LP_SYNT3_REG, regs, 4);

	LOG_DBG("SYNT=0x%x)\n\r", synth);
}

void s2lp_sfx_rf_tx_fsm(const struct device *dev)
{
	struct s2lp_context *ctx = dev->data;
	uint8_t bit;

	switch (ctx->trx.tx.state) {
	/* When the state machine is here means that we have loaded the very
	 * first part of the RAMP_UP and we need to load the second part
	 * (the values of this part is stored into the variable into the
	 * fifo_start_ramp_up_2)
	 */
	case S2LP_TX_STATE_RAMP_UP_2:
	{
		s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_UP_2,
					ctx->trx.tx.ramp_buffer,
					&ctx->trx.tx.ramp_len);

		s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
				       ctx->trx.tx.ramp_buffer,
				       ctx->trx.tx.ramp_len);

		if (ctx->trx.tx.continuous_tx == 1) {
			ctx->trx.tx.bit_idx = 8;
			ctx->trx.tx.state = S2LP_TX_STATE_CONTINUOS_BPSK;
		} else {
			ctx->trx.tx.byte_idx = 0;
			ctx->trx.tx.bit_idx = 7;
			ctx->trx.tx.state = S2LP_TX_STATE_CONTINUOS_BPSK;
		}
		break;
	}
	case S2LP_TX_STATE_CONTINUOS_BPSK:
	{
		bit = (ctx->trx.tx.pn9 >> ctx->trx.tx.bit_idx) & 0x01;
		s2lp_sfx_rf_tx_dbpsk_single_bit(dev, bit);

		if (ctx->trx.tx.bit_idx == 0) {
			ctx->trx.tx.bit_idx = 8;
			ctx->trx.tx.pn9 =
				s2lp_sfx_rf_tx_pn9_next_byte(ctx->trx.tx.pn9);
		} else {
			ctx->trx.tx.bit_idx--;
		}
		break;
	}
	case S2LP_TX_STATE_DATA:
	{
		/* extract the bit to modulate from the array pointed by
		 * data_to_send
		 */
		bit = (ctx->trx.tx.payload[ctx->trx.tx.byte_idx] >>
		       ctx->trx.tx.bit_idx) & 0x01;

		/* The sigfox protocol assume that a bit '0' should be
		 * represented by a phase inversion
		 */
		s2lp_sfx_rf_tx_dbpsk_single_bit(dev, bit);

		if (ctx->trx.tx.bit_idx == 0) {
			ctx->trx.tx.bit_idx = 7;
			ctx->trx.tx.byte_idx++;

			if (ctx->trx.tx.byte_idx == ctx->trx.tx.size) {
				ctx->trx.tx.byte_idx = 0;
				ctx->trx.tx.state = S2LP_TX_STATE_RAMP_DOWN_1;
			}
		} else {
			ctx->trx.tx.bit_idx--;
		}
		break;
	}
	case S2LP_TX_STATE_RAMP_DOWN_1:
	{
		s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_DOWN_1,
					ctx->trx.tx.ramp_buffer,
					&ctx->trx.tx.ramp_len);

		s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
				       ctx->trx.tx.ramp_buffer,
				       ctx->trx.tx.ramp_len);

		ctx->trx.tx.state = S2LP_TX_STATE_RAMP_DOWN_2;
		break;
	}
	case S2LP_TX_STATE_RAMP_DOWN_2:
	{
		s2lp_sfx_rf_gen_tx_ramp(dev, S2LP_RAMP_DOWN_2,
					ctx->trx.tx.ramp_buffer,
					&ctx->trx.tx.ramp_len);

		s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
				       ctx->trx.tx.ramp_buffer,
				       ctx->trx.tx.ramp_len);

		ctx->trx.tx.state = S2LP_TX_STATE_RAMP_DOWN_3;
		break;
	}
	case S2LP_TX_STATE_RAMP_DOWN_3:
	{
		/* 3rd part of the ramp is just a sequence of zeroes used as
		 * a padding to wait the fifo_ramp_down_2 is over
		 */
		ctx->trx.tx.ramp_len = 64;

		memset(ctx->trx.tx.ramp_buffer, 0x00, ctx->trx.tx.ramp_len);

		s2lp_iface_burst_write(dev, S2LP_FIFO_REG,
				       ctx->trx.tx.ramp_buffer,
				       ctx->trx.tx.ramp_len);

		ctx->trx.tx.state = S2LP_TX_STATE_STOP;
		break;
	}
	case S2LP_TX_STATE_STOP:
	{
		ctx->trx.tx.state = S2LP_TX_STATE_NONE;
		break;
	}
	default:
		LOG_ERR("Invalid State");
		break;
	}
}
