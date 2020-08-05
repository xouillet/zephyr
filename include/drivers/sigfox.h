/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SIGFOX_H_
#define ZEPHYR_INCLUDE_DRIVERS_SIGFOX_H_

/**
 * @file
 * @brief Public Sigfox APIs
 */

#include <device.h>

enum sigfox_error_codes {
	SIGFOX_ERR_NONE                                 = 0x00,

	/* MCU      API ERROR CODES : From 0x10 to 0x2F */
	SIGFOX_ERR_MCU_MALLOC                           = 0x11,
	SIGFOX_ERR_MCU_FREE                             = 0x12,
	SIGFOX_ERR_MCU_VOLT_TEMP                        = 0x13,
	SIGFOX_ERR_MCU_DLY                              = 0x14,
	SIGFOX_ERR_MCU_AES                              = 0x15,
	SIGFOX_ERR_MCU_GETNVMEM                         = 0x16,
	SIGFOX_ERR_MCU_SETNVMEM                         = 0x17,
	SIGFOX_ERR_MCU_TIMER_START                      = 0x18,
	SIGFOX_ERR_MCU_TIMER_START_CS                   = 0x19,
	SIGFOX_ERR_MCU_TIMER_STOP_CS                    = 0x1A,
	SIGFOX_ERR_MCU_TIMER_STOP                       = 0x1B,
	SIGFOX_ERR_MCU_TIMER_END                        = 0x1C,
	SIGFOX_ERR_MCU_TEST_REPORT                      = 0x1D,
	SIGFOX_ERR_MCU_GET_VERSION                      = 0x1E,
	SIGFOX_ERR_MCU_GET_ID_PAYLOAD_ENCR_FLAG         = 0x1F,
	SIGFOX_ERR_MCU_GET_PAC                          = 0x20,

	/* RF       API ERROR CODES : From 0x30 to 0x3F */
	SIGFOX_ERR_RF_INIT                              = 0x30,
	SIGFOX_ERR_RF_SEND                              = 0x31,
	SIGFOX_ERR_RF_CHANGE_FREQ                       = 0x32,
	SIGFOX_ERR_RF_STOP                              = 0x33,
	SIGFOX_ERR_RF_WAIT_FRAME                        = 0x34,
	SIGFOX_ERR_RF_WAIT_CLEAR_CHANNEL                = 0x35,
	SIGFOX_ERR_RF_START_CONTINUOUS_TRANSMISSION     = 0x36,
	SIGFOX_ERR_RF_STOP_CONTINUOUS_TRANSMISSION      = 0x37,
	SIGFOX_ERR_RF_GET_VERSION                       = 0x38,

	/* SE       API ERROR CODES : From 0x40 to 0x5F */
	SIGFOX_ERR_SE_INIT                              = 0x40,
	SIGFOX_ERR_SE_OPEN                              = 0x41,
	SIGFOX_ERR_SE_CLOSE                             = 0x42,
	SIGFOX_ERR_SE_SECURE_UP_MSG                     = 0x43,
	SIGFOX_ERR_SE_VERIFY_DL_MSG                     = 0x44,
	SIGFOX_ERR_SE_GET_ID                            = 0x45,
	SIGFOX_ERR_SE_GET_PAC                           = 0x46,
	SIGFOX_ERR_SE_GET_VERSION                       = 0x47,
	SIGFOX_ERR_SE_RC_PERIOD                         = 0x48,
	SIGFOX_ERR_SE_GEN_RCSYNC                        = 0x49,

	/* REPEATER API ERROR CODES : From 0x60 to 0x7F */
	/* MONARCH  API ERROR CODES : From 0x80 to 0x8F */
	SIGFOX_ERR_MONARCH_MALLOC                       = 0x80,
	SIGFOX_ERR_MONARCH_FREE                         = 0x81,
	SIGFOX_ERR_MONARCH_TIMER_START                  = 0x82,
	SIGFOX_ERR_MONARCH_TIMER_STOP                   = 0x83,
	SIGFOX_ERR_MONARCH_CONFIGURE_SEARCH_PATTERN     = 0x84,
	SIGFOX_ERR_MONARCH_STOP_SEARCH_PATTERN          = 0x85,
	SIGFOX_ERR_MONARCH_GET_VERSION                  = 0x86,

	/* ADDONS   API ERROR CODES : From 0xC0 to 0xDF */
	SIGFOX_ERR_INT_DOWNLINK_CONFIGURATION           = 0xE0,
};

enum sigfox_mode {
	SIGFOX_MODE_TX,
	SIGFOX_MODE_RX,
	SIGFOX_MODE_CS200K_RX,
	SIGFOX_MODE_CS300K_RX,
	SIGFOX_MODE_MONARCH,
};

enum sigfox_spectrum_access {
	SIGFOX_FH   = 1, /* Index of Frequency Hopping */
	SIGFOX_LBT  = 2, /* Index of Listen Before Talk */
	SIGFOX_DC   = 4, /* Index of Duty Cycle */
};

enum sigfox_mod_type {
	SIGFOX_NO_MODULATION,
	SIGFOX_DBPSK_100BPS,
	SIGFOX_DBPSK_600BPS,
};

enum sigfox_rx_state {
	SIGFOX_DL_TIMEOUT,
	SIGFOX_DL_PASSED,
};

struct sigfox_modem_config {
	enum sigfox_mode mode;
	uint32_t frequency;
};

/**
 * @typedef sigfox_api_config()
 * @brief Callback API for configuring the sigfox module
 *
 * @see sigfox_config() for argument descriptions.
 */
typedef int (*sigfox_api_start)(const struct device *dev);
typedef int (*sigfox_api_config)(const struct device *dev,
				 const struct sigfox_modem_config *config);
typedef int (*sigfox_api_stop)(const struct device *dev);
typedef int (*sigfox_api_send)(const struct device *dev,
			       enum sigfox_mod_type modulation,
			       uint8_t *buffer,
			       size_t len);
typedef int (*sigfox_api_recv)(const struct device *dev,
			       enum sigfox_rx_state *state,
			       uint8_t *buffer,
			       int16_t *rssi);
typedef int (*sigfox_api_cca)(const struct device *dev,
			      enum sigfox_rx_state *state,
			      uint8_t cs_min,
			      int8_t cs_threshold);
typedef int (*sigfox_api_switch_freq)(const struct device *dev,
				      uint32_t frequency);
typedef int (*sigfox_api_tx_cont_start)(const struct device *dev,
					enum sigfox_mod_type modulation);
typedef int (*sigfox_api_tx_cont_stop)(const struct device *dev);

struct sigfox_driver_api {
	sigfox_api_start		start;
	sigfox_api_config		config;
	sigfox_api_stop			stop;
	sigfox_api_send			send;
	sigfox_api_recv			recv;
	sigfox_api_cca			cca;
	sigfox_api_switch_freq		sw_freq;
	sigfox_api_tx_cont_start	tx_cont_start;
	sigfox_api_tx_cont_stop		tx_cont_stop;
};

/**
 * @brief Configure the sigfox modem
 *
 * @param dev     Sigfox device
 * @param config  Data structure containing the intended configuration for the
		  modem
 * @return 0 on success, negative on error
 */
static inline int sigfox_config(const struct device *dev,
				const struct sigfox_modem_config *config)
{
	const struct sigfox_driver_api *api = dev->api;

	return api->config(dev, config);
}

#endif	/* ZEPHYR_INCLUDE_DRIVERS_SIGFOX_H_ */
