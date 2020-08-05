/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct sigfox_context {
	struct device *dev;
	struct sigfox_modem_config config;
	struct sfx_rc_t sfx_rc;
};

void sigfox_radio_set_context(struct sigfox_context *ctx);
