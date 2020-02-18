/*
 * Copyright (c) 2019 Gerson Fernando Budke
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the Atmel SAM4L family processors.
 */

#ifndef _ATMEL_SAM4L_SOC_H_
#define _ATMEL_SAM4L_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT
#define DONT_USE_PREDEFINED_CORE_HANDLERS
#define DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS

#if defined(CONFIG_SOC_PART_NUMBER_SAM4LS8C)
#include <sam4ls8c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS8B)
#include <sam4ls8b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS8A)
#include <sam4ls8a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS4C)
#include <sam4ls4c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS4B)
#include <sam4ls4b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS4A)
#include <sam4ls4a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS2C)
#include <sam4ls2c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS2B)
#include <sam4ls2b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LS2A)
#include <sam4ls2a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC8C)
#include <sam4lc8c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC8B)
#include <sam4lc8b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC8A)
#include <sam4lc8a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC4C)
#include <sam4lc4c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC4B)
#include <sam4lc4b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC4A)
#include <sam4lc4a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC2C)
#include <sam4lc2c.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC2B)
#include <sam4lc2b.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAM4LC2A)
#include <sam4lc2a.h>
#else
#error Library does not support the specified device.
#endif

#include "soc_pinmap.h"
#include "../common/soc_pmc.h"
#include "../common/soc_gpio.h"

#endif /* !_ASMLANGUAGE */

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM_HCLK_FREQ_HZ      DT_ARM_CORTEX_M4_0_CLOCK_FREQUENCY

/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAM_MCK_FREQ_HZ       SOC_ATMEL_SAM_HCLK_FREQ_HZ

/** Oscillator identifiers
 *    External Oscillator 0
 *    External 32 kHz oscillator
 *    Internal 32 kHz RC oscillator
 *    Internal 80 MHz RC oscillator
 *    Internal 4-8-12 MHz RCFAST oscillator
 *    Internal 1 MHz RC oscillator
 *    Internal System RC oscillator
 */
#define OSC_ID_OSC0             0
#define OSC_ID_OSC32            1
#define OSC_ID_RC32K            2
#define OSC_ID_RC80M            3
#define OSC_ID_RCFAST           4
#define OSC_ID_RC1M             5
#define OSC_ID_RCSYS            6

/** System clock source
 *    System RC oscillator
 *    Oscillator 0
 *    Phase Locked Loop 0
 *    Digital Frequency Locked Loop
 *    80 MHz RC oscillator
 *    4-8-12 MHz RC oscillator
 *    1 MHz RC oscillator
 */
#define OSC_SRC_RCSYS           0
#define OSC_SRC_OSC0            1
#define OSC_SRC_PLL0            2
#define OSC_SRC_DFLL            3
#define OSC_SRC_RC80M           4
#define OSC_SRC_RCFAST          5
#define OSC_SRC_RC1M            6

#define PM_CLOCK_MASK(bus, per) ((bus << 5) + per)

/** Bus index of maskable module clocks
 */
#define PM_CLK_GRP_CPU          0
#define PM_CLK_GRP_HSB          1
#define PM_CLK_GRP_PBA          2
#define PM_CLK_GRP_PBB          3
#define PM_CLK_GRP_PBC          4
#define PM_CLK_GRP_PBD          5

/** Clocks derived from the CPU clock
 */
#define SYSCLK_OCD              0

/** Clocks derived from the HSB clock
 */
#define SYSCLK_PDCA_HSB         0
#define SYSCLK_HFLASHC_DATA     1
#define SYSCLK_HRAMC1_DATA      2
#define SYSCLK_USBC_DATA        3
#define SYSCLK_CRCCU_DATA       4
#define SYSCLK_PBA_BRIDGE       5
#define SYSCLK_PBB_BRIDGE       6
#define SYSCLK_PBC_BRIDGE       7
#define SYSCLK_PBD_BRIDGE       8
#define SYSCLK_AESA_HSB         9

/** Clocks derived from the PBA clock
 */
#define SYSCLK_IISC             0
#define SYSCLK_SPI              1
#define SYSCLK_TC0              2
#define SYSCLK_TC1              3
#define SYSCLK_TWIM0            4
#define SYSCLK_TWIS0            5
#define SYSCLK_TWIM1            6
#define SYSCLK_TWIS1            7
#define SYSCLK_USART0           8
#define SYSCLK_USART1           9
#define SYSCLK_USART2           10
#define SYSCLK_USART3           11
#define SYSCLK_ADCIFE           12
#define SYSCLK_DACC             13
#define SYSCLK_ACIFC            14
#define SYSCLK_GLOC             15
#define SYSCLK_ABDACB           16
#define SYSCLK_TRNG             17
#define SYSCLK_PARC             18
#define SYSCLK_CATB             19
#define SYSCLK_TWIM2            21
#define SYSCLK_TWIM3            22
#define SYSCLK_LCDCA            23

/** Clocks derived from the PBB clock
 */
#define SYSCLK_HFLASHC_REGS     0
#define SYSCLK_HRAMC1_REGS      1
#define SYSCLK_HMATRIX          2
#define SYSCLK_PDCA_PB          3
#define SYSCLK_CRCCU_REGS       4
#define SYSCLK_USBC_REGS        5
#define SYSCLK_PEVC             6

/** Clocks derived from the PBC clock
 */
#define SYSCLK_PM               0
#define SYSCLK_CHIPID           1
#define SYSCLK_SCIF             2
#define SYSCLK_FREQM            3
#define SYSCLK_GPIO             4

/** Clocks derived from the PBD clock
 */
#define SYSCLK_BPM              0
#define SYSCLK_BSCIF            1
#define SYSCLK_AST              2
#define SYSCLK_WDT              3
#define SYSCLK_EIC              4
#define SYSCLK_PICOUART         5

/** Divided clock mask derived from the PBA clock
 */
#define PBA_DIVMASK_TIMER_CLOCK2     (1u << 0)
#define PBA_DIVMASK_TIMER_CLOCK3     (1u << 2)
#define PBA_DIVMASK_CLK_USART        (1u << 2)
#define PBA_DIVMASK_TIMER_CLOCK4     (1u << 4)
#define PBA_DIVMASK_TIMER_CLOCK5     (1u << 6)
#define PBA_DIVMASK_Msk              (0x7Fu << 0)

#endif /* _ATMEL_SAM4L_SOC_H_ */
