/*
 * Copyright (c) 2021, ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board configuration macros
 *
 * This header file is used to specify and describe board-level aspects
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <sys/util.h>
#include <soc_common.h>

#ifndef _ASMLANGUAGE

/* Add include for DTS generated information */
#include <devicetree.h>

#if defined(CONFIG_SOC_SERIES_GD32VF103)
#include <gd32vf103.h>
#else
#error Library does not support the specified device.
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
