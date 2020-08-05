/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@mail.com>
 */

/**
 * FOO.o being placed in section BAR
 * remove warnings:
 *   orphan section '.iar_vfe_header' from FOO.a
 */

. = ALIGN(4);
KEEP(*(.iar_vfe_header))
