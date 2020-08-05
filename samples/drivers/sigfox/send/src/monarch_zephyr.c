/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sigfox_types.h>
#include <sigfox_monarch_api.h>
#include <monarch_api.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sigfox_monarch, CONFIG_SIGFOX_SEND_LOG_LEVEL);

/**
 * SIGFOX MONARCH MANUFACTUER API
 */

sfx_u8 MONARCH_API_malloc(sfx_u16 size,
			  sfx_u8 **returned_pointer)
{
	return MONARCH_ERR_API_MALLOC;
}

sfx_u8 MONARCH_API_free(sfx_u8 *ptr)
{
	return MONARCH_ERR_API_FREE;
}

sfx_u8 MONARCH_API_timer_start(sfx_u16 timer_value,
			       sfx_timer_unit_enum_t unit,
			       sfx_error_t (*timeout_callback_handler)(void))
{
	return MONARCH_ERR_API_TIMER_START;
}

sfx_u8 MONARCH_API_timer_stop(void)
{
	return MONARCH_ERR_API_TIMER_STOP;
}

sfx_u8 MONARCH_API_configure_search_pattern(
	sfx_monarch_pattern_search_t list_freq_pattern[],
	sfx_u8 size,
	sfx_monarch_listening_mode_t mode,
	sfx_error_t (*monarch_pattern_freq_result_callback_handler)(
		sfx_u32 freq,
		sfx_pattern_enum_t pattern,
		sfx_s16 rssi))
{
	return MONARCH_ERR_API_CONFIGURE_SEARCH_PATTERN;
}

sfx_u8 MONARCH_API_stop_search_pattern(void)
{
	return MONARCH_ERR_API_STOP_SEARCH_PATTERN;
}

sfx_u8 MONARCH_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
	return MONARCH_ERR_API_GET_VERSION;
}
