/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/sigfox.h>

#include <sigfox_types.h>
#include <sigfox_api.h>
#include <rf_api.h>

#include <sigfox_zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sigfox_radio, CONFIG_SIGFOX_SEND_LOG_LEVEL);

static struct sigfox_context *sfx_radio_ctx;
/*
 * #define RC2_SET_STD_CONFIG_LM_WORD_0 0x000001FF  LM = Long Message
 * #define RC2_SET_STD_CONFIG_LM_WORD_1 0x00000000
 * #define RC2_SET_STD_CONFIG_LM_WORD_2 0x00000000
 * #define RC2_SET_STD_TIMER_ENABLE     (SFX_TRUE)  En Tmr for FH duty cycle
 * #define RC2_SET_STD_TIMER_DISABLE    (SFX_FALSE) Disable timer feature
 * #define RC2_SET_STD_CONFIG_SM_WORD_0 0x00000001  SM = Short message
 * #define RC2_SET_STD_CONFIG_SM_WORD_1 0x00000000
 * #define RC2_SET_STD_CONFIG_SM_WORD_2 0x00000000
 */
void sigfox_radio_set_context(struct sigfox_context *ctx)
{
	/* In FCC we can choose the macro channel to use by a 86 bits bitmask
	 * In this case we use the first 9 macro channels
	 */
	sfx_u32 config_words[3] = {1, 0, 0};

	sfx_radio_ctx = ctx;

	/* RC2 - Brazil */
	ctx->sfx_rc.open_tx_frequency = 902200000;
	ctx->sfx_rc.open_rx_frequency = 905200000;
	ctx->sfx_rc.macro_channel_width = 192000;
	ctx->sfx_rc.modulation = SIGFOX_DBPSK_600BPS;
	ctx->sfx_rc.spectrum_access = SIGFOX_FH;
	ctx->sfx_rc.specific_rc.open_cs_frequency = 0;
	ctx->sfx_rc.specific_rc.open_cs_bandwidth = 0;
	ctx->sfx_rc.specific_rc.cs_threshold = 0;

	if (SIGFOX_API_open(&sfx_radio_ctx->sfx_rc)) {
		LOG_ERR("open");
	}

	/* Set the standard configuration with default channel to 1 */
	if (SIGFOX_API_set_std_config(config_words, 0)) {
		LOG_ERR("set_std_config");
	}
}

/**
 * SIGFOX RADIO ZEPHYR API
 */

sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_modem_config *config = &sfx_radio_ctx->config;
	const struct sigfox_driver_api *api = dev->api;
	int ret;

	LOG_DBG("");

	ret = api->start(dev);
	if (ret) {
		return ret;
	}

	sfx_radio_ctx->config.mode = rf_mode;

	return api->config(dev, config);
}

sfx_u8 RF_API_stop(void)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;

	LOG_DBG("");

	return api->stop(dev);
}

sfx_u8 RF_API_send(sfx_u8 *stream,
		   sfx_modulation_type_t type,
		   sfx_u8 size)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;

	LOG_DBG("");

	return api->send(dev, type, stream, size);
}

sfx_u8 RF_API_start_continuous_transmission(sfx_modulation_type_t type)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;

	LOG_DBG("");

	return api->tx_cont_start(dev, type);
}

sfx_u8 RF_API_stop_continuous_transmission(void)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;

	LOG_DBG("");

	return api->tx_cont_stop(dev);
}

sfx_u8 RF_API_change_frequency(sfx_u32 frequency)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;

	LOG_DBG("");

	return api->sw_freq(dev, frequency);
}

sfx_u8 RF_API_wait_frame(sfx_u8 *frame,
			 sfx_s16 *rssi,
			 sfx_rx_state_enum_t *state)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;
	enum sigfox_rx_state rx_state;
	int ret;

	LOG_DBG("");

	ret = api->recv(dev, &rx_state, frame, rssi);
	if (ret) {
		return ret;
	}

	(*state) = rx_state;

	return SFX_ERR_NONE;
}

sfx_u8 RF_API_wait_for_clear_channel(sfx_u8 cs_min,
				     sfx_s8 cs_threshold,
				     sfx_rx_state_enum_t *state)
{
	const struct device *dev = sfx_radio_ctx->dev;
	const struct sigfox_driver_api *api = dev->api;
	enum sigfox_rx_state rx_state;
	int ret;

	LOG_DBG("");

	ret = api->cca(dev, &rx_state, cs_min, cs_threshold);
	if (ret) {
		return ret;
	}

	(*state) = rx_state;

	return SFX_ERR_NONE;
}

sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
	(*version) = (sfx_u8 *) CONFIG_SIGFOX_RADIO_API_VERSION;
	(*size) = sizeof(CONFIG_SIGFOX_RADIO_API_VERSION);

	LOG_DBG("Version: %s, Size: %d", *version, *size);

	return SFX_ERR_NONE;
}
