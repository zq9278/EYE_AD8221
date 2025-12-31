/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_uart_stream.h"

#include "emg_sampler.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

K_THREAD_STACK_DEFINE(uart_stream_stack, 768);
static struct k_thread uart_stream_thread;
static const struct device *uart_dev;

/*
 * UART streaming thread.
 *
 * Pulls full-rate samples from the sampler queue and emits a compact binary
 * frame on UART0. Frame format (little-endian):
 *   0xAA 0x55 <len=0x06> <raw> <notch> <envelope>
 */
static void uart_stream_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		struct emg_sample_triple triple;
		int err = emg_sampler_get_uart_triple(&triple, K_FOREVER);
		if (err) {
			continue;
		}

		/* Construct the fixed-length frame to avoid formatting overhead. */
		uint8_t raw_le[2];
		uint8_t notch_le[2];
		uint8_t env_le[2];
		sys_put_le16(triple.raw, raw_le);
		sys_put_le16(triple.notch, notch_le);
		sys_put_le16(triple.envelope, env_le);
		uint8_t frame[9] = {
			0xAA, 0x55, 0x06,
			raw_le[0], raw_le[1],
			notch_le[0], notch_le[1],
			env_le[0], env_le[1],
		};
		for (size_t i = 0; i < sizeof(frame); i++) {
			uart_poll_out(uart_dev, frame[i]);
		}
	}
}

int emg_uart_stream_init(void)
{
	if (!IS_ENABLED(CONFIG_EYE_UART_STREAM)) {
		return 0;
	}

	/* UART device is fixed to uart0; use polling to keep ISR load low. */
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	uart_dev = dev;

	k_thread_create(&uart_stream_thread, uart_stream_stack,
			K_THREAD_STACK_SIZEOF(uart_stream_stack),
			uart_stream_thread_fn, NULL, NULL, NULL,
			K_PRIO_COOP(0), 0, K_NO_WAIT);
	k_thread_name_set(&uart_stream_thread, "emg_uart_stream");

	return 0;
}
