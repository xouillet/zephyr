/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_RF_H_
#define ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_RF_H_

#define S2LP_MONARCH_DATARATE	((uint64_t)16384 * 134217728)

enum s2lp_sfx_trx_fem_fsm_t {
	S2LP_FEM_FSM_INIT,
	S2LP_FEM_FSM_SHUTDOWN,
	S2LP_FEM_FSM_RX_LNA,
	S2LP_FEM_FSM_RX_BYPASS,
	S2LP_FEM_FSM_TX_PA,
	S2LP_FEM_FSM_TX_BYPASS,
};

void s2lp_sfx_trx_fem_init(const struct device *dev);
void s2lp_sfx_trx_fem(const struct device *dev,
		      enum s2lp_sfx_trx_fem_fsm_t fem);
void s2lp_sfx_trx_tx_start(const struct device *dev);
void s2lp_sfx_trx_rx_start(const struct device *dev);
void s2lp_sfx_trx_txrx_stop(const struct device *dev);
void s2lp_sfx_rf_rx_init(const struct device *dev);
#if defined(CONFIG_SIGFOX_MONARCH_FEATURE)
void s2lp_sfx_rf_rx_monarch_init(const struct device *dev);
#endif
void s2lp_sfx_rf_tx_init(const struct device *dev);
void s2lp_sfx_rf_tx_fsm(const struct device *dev);
void s2lp_sfx_rf_dbpsk_init(const struct device *dev,
			    enum sigfox_mod_type modulation);
void s2lp_sfx_rf_load_first_ramp_up(const struct device *dev);
void s2lp_sfx_rf_change_freq(const struct device *dev, uint32_t frequency);

#endif /* ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_RF_H_ */
