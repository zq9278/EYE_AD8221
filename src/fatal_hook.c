/*
 * Minimal fatal error hook to keep the CPU halted on faults and print the reason.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/fatal.h>

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	printk("FATAL: reason=%u\n", reason);
	if (esf) {
		printk("ESF: pc=0x%08x lr=0x%08x xpsr=0x%08x\n",
		       (unsigned int)esf->basic.pc, (unsigned int)esf->basic.lr,
		       (unsigned int)esf->basic.xpsr);
	}

	/* Halt here so we can see the fault over RTT without auto-reset. */
	k_fatal_halt(reason);
}
