/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_uart_stream.h"

#include "emg_sampler.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/*
 * Use a fixed UART instance for streaming so it still works when
 * CONFIG_UART_CONSOLE/CONFIG_PRINTK are disabled.
 */
#define EMG_UART_NODE DT_NODELABEL(uart0)

/*
 * VOFA+ "JustFloat" framing:
 *   [float32 little-endian] ... then tail 0x00 0x00 0x80 0x7F
 */
static const uint8_t vofa_tail[4] = { 0x00, 0x00, 0x80, 0x7F };

K_THREAD_STACK_DEFINE(uart_stream_stack, 1024);
static struct k_thread uart_stream_thread;

static const struct device *uart_dev;

static void uart_write_bytes(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

static void uart_stream_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

#if IS_ENABLED(CONFIG_EYE_UART_STREAM)
	const uint32_t period_us = 1000000U / CONFIG_EYE_UART_STREAM_HZ;
#else
	const uint32_t period_us = 1000U;
#endif

	for (;;) {
		uint16_t raw = emg_sampler_get_latest_raw_u16();
		uint16_t filtered = emg_sampler_get_latest_sample_u16();
		uint16_t notch = emg_sampler_get_latest_notch_u16();

		float ch1;
		float ch2;
		float ch3;
		if (IS_ENABLED(CONFIG_EYE_UART_STREAM_CENTERED)) {
			ch1 = (float)((int32_t)raw - 2048);
			ch2 = (float)((int32_t)filtered - 2048);
			ch3 = (float)((int32_t)notch - 2048);
		} else {
			ch1 = (float)raw;
			ch2 = (float)filtered;
			ch3 = (float)notch;
		}

		union { float f; uint8_t b[4]; } u1 = { .f = ch1 };
		union { float f; uint8_t b[4]; } u2 = { .f = ch2 };
		union { float f; uint8_t b[4]; } u3 = { .f = ch3 };

		uart_write_bytes(u1.b, sizeof(u1.b));
		uart_write_bytes(u2.b, sizeof(u2.b));
		if (CONFIG_EYE_UART_STREAM_CHANNELS >= 3) {
			uart_write_bytes(u3.b, sizeof(u3.b));
		}
		uart_write_bytes(vofa_tail, sizeof(vofa_tail));

		k_usleep(period_us);
	}
}

int emg_uart_stream_init(void)
{
	if (!IS_ENABLED(CONFIG_EYE_UART_STREAM)) {
		return 0;
	}

	BUILD_ASSERT(!IS_ENABLED(CONFIG_UART_CONSOLE),
		     "Disable CONFIG_UART_CONSOLE when using CONFIG_EYE_UART_STREAM");

	if (!DT_NODE_HAS_STATUS(EMG_UART_NODE, okay)) {
		return -ENODEV;
	}

	uart_dev = DEVICE_DT_GET(EMG_UART_NODE);
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	k_thread_create(&uart_stream_thread, uart_stream_stack,
			K_THREAD_STACK_SIZEOF(uart_stream_stack),
			uart_stream_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&uart_stream_thread, "emg_uart_stream");

	return 0;
}
