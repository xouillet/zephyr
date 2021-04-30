/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief interrupt management code for riscv SOCs supporting the riscv
	  privileged architecture specification
 */
#include <irq.h>

void arch_irq_enable(unsigned int irq)
{
	eclic_enable_interrupt(irq);
}

void arch_irq_disable(unsigned int irq)
{
	uint32_t mie;

	eclic_disable_interrupt(irq);

	/*
	 * Use atomic instruction csrrc to disable device interrupt in mie CSR.
	 * (atomic read and clear bits in CSR register)
	 */
	__asm__ volatile ("csrrc %0, mie, %1\n"
			  : "=r" (mie)
			  : "r" (1 << irq));
};

void arch_irq_priority_set(unsigned int irq, unsigned int prio)
{
	eclic_set_nonvmode(irq);
	eclic_set_level_trig(irq);
	eclic_set_irq_priority(irq, prio);
}

int arch_irq_is_enabled(unsigned int irq)
{
	//return ECLIC_GetEnableIRQ(irq);
	return 0;
}

#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void)
{
	/* ensure that all interrupts are disabled */
	(void)irq_lock();

	__asm__ volatile ("csrwi mie, 0\n"
			  "csrwi mip, 0\n");
}
#endif
