/*
 * Copyright (c) 2021, ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <device.h>
#include <init.h>
#include <bl602_hbn.h>
#include <bl602_glb.h>

#define ROOT_FCLK_DIV			(0)
#define ROOT_BCLK_DIV			(1)
#define ROOT_UART_CLOCK_DIV		(0)

static uint32_t mtimer_get_clk_src_div(void)
{
	return ((SystemCoreClockGet() / (GLB_Get_BCLK_Div() + 1))
		/ 1000 / 1000 - 1);
}

static void system_clock_init(void)
{
	GLB_Set_System_CLK(GLB_PLL_XTAL_40M, GLB_SYS_CLK_PLL160M);
	GLB_Set_System_CLK_Div(ROOT_FCLK_DIV, ROOT_BCLK_DIV);
	GLB_Set_MTimer_CLK(1, GLB_MTIMER_CLK_BCLK, mtimer_get_clk_src_div());
}

static void peripheral_clock_init(void)
{
	GLB_Set_UART_CLK(1, HBN_UART_CLK_160M, ROOT_UART_CLOCK_DIV);
}

#ifdef CONFIG_RISCV_SOC_INIT_GP_VALUE
ulong_t __soc_get_gp_initial_value(void)
{
	extern uint32_t __global_pointer$;
	return (ulong_t)&__global_pointer$;
}
#endif

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

static int bl_riscv_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	SystemInit();

	system_clock_init();
	peripheral_clock_init();

	irq_unlock(key);

	return 0;
}

SYS_INIT(bl_riscv_init, PRE_KERNEL_1, 0);
