/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <dk_buttons_and_leds.h>

#include "emg_sampler.h"
#include "emg_uart_stream.h"
#include "emg_ble_stream.h"

/*
 * Application entry point.
 *
 * Responsibilities:
 * - Initialize BLE GATT service for single-sample read/notify + stream notify.
 * - Start the SAADC sampling pipeline and UART binary streaming.
 * - Maintain BLE advertising/connection state and send periodic stats.
 */
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)
#define DEVICE_NAME_SHORT_MAX   8
#define DEVICE_NAME_SHORT_LEN   MIN(DEVICE_NAME_LEN, DEVICE_NAME_SHORT_MAX)


#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000

static struct k_work adv_work;

#define NOTIFY_INTERVAL_MS_DEFAULT  (1000 / CONFIG_EYE_BLE_PACKET_HZ)
#define NOTIFY_INTERVAL_MS_MIN      NOTIFY_INTERVAL_MS_DEFAULT

BUILD_ASSERT(CONFIG_EYE_BLE_STREAM_HZ % CONFIG_EYE_BLE_PACKET_HZ == 0,
	     "EYE_BLE_STREAM_HZ must be a multiple of EYE_BLE_PACKET_HZ");
#define STREAM_SAMPLES_PER_PACKET   (CONFIG_EYE_BLE_STREAM_HZ / CONFIG_EYE_BLE_PACKET_HZ)

static uint16_t notify_interval_ms = NOTIFY_INTERVAL_MS_DEFAULT;
static struct k_work_delayable notify_work;
static uint32_t last_stats_uptime_ms;
static uint32_t stat_notify_attempts;
static uint32_t stat_notify_ok;
static uint32_t stat_notify_err;
static uint32_t last_sampler_log_ms;

/* BLE UUIDs */
#define BT_UUID_EYE_SVC_VAL \
	BT_UUID_128_ENCODE(0x0000ad82, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_EYE_ADC_VAL \
	BT_UUID_128_ENCODE(0x0000ad83, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_EYE_CFG_VAL \
	BT_UUID_128_ENCODE(0x0000ad84, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_EYE_STREAM_VAL \
	BT_UUID_128_ENCODE(0x0000ad85, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_EYE_SVC  BT_UUID_DECLARE_128(BT_UUID_EYE_SVC_VAL)
#define BT_UUID_EYE_ADC  BT_UUID_DECLARE_128(BT_UUID_EYE_ADC_VAL)
#define BT_UUID_EYE_CFG  BT_UUID_DECLARE_128(BT_UUID_EYE_CFG_VAL)
#define BT_UUID_EYE_STREAM  BT_UUID_DECLARE_128(BT_UUID_EYE_STREAM_VAL)

static bool notify_enabled;
static bool stream_notify_enabled;
static struct bt_conn *default_conn;

static void request_fast_conn_params(struct bt_conn *conn)
{
	struct bt_le_conn_param param = {
		.interval_min = CONFIG_EYE_BLE_CONN_INTERVAL_MIN,
		.interval_max = CONFIG_EYE_BLE_CONN_INTERVAL_MAX,
		.latency = 0,
		.timeout = 400,
	};

	int err = bt_conn_le_param_update(conn, &param);
	if (err) {
		printk("Conn param update request failed (err %d)\n", err);
	}
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_EYE_SVC_VAL),
	BT_DATA(BT_DATA_NAME_SHORTENED, DEVICE_NAME, DEVICE_NAME_SHORT_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void notify_work_handler(struct k_work *work);

/*
 * CCC handlers toggle notifications for the single-sample and stream
 * characteristics. Both of them share the same notify work item and interval.
 */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	if (!default_conn) {
		return;
	}

	if (!notify_enabled || notify_interval_ms == 0) {
		(void)k_work_cancel_delayable(&notify_work);
		return;
	}

	request_fast_conn_params(default_conn);
	k_work_reschedule(&notify_work, K_NO_WAIT);
}

static void stream_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	stream_notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	if (!default_conn) {
		return;
	}

	if (!stream_notify_enabled || notify_interval_ms == 0) {
		(void)k_work_cancel_delayable(&notify_work);
		return;
	}

	request_fast_conn_params(default_conn);
	k_work_reschedule(&notify_work, K_NO_WAIT);
}

static ssize_t read_adc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	/* Read handler returns the latest filtered sample (little-endian). */
	uint16_t sample = emg_sampler_get_latest_sample_u16();
	uint16_t sample_le = sys_cpu_to_le16(sample);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sample_le,
				 sizeof(sample_le));
}

static ssize_t write_cfg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t interval = sys_get_le16(buf);

	/* Clamp to a safe interval range to avoid flooding or slowing down too far. */
	if (interval != 0 && interval < NOTIFY_INTERVAL_MS_MIN) {
		interval = NOTIFY_INTERVAL_MS_MIN;
	}

	/* Keep MCU default rate unless the client explicitly asks for faster. */
	if (interval != 0 && interval > NOTIFY_INTERVAL_MS_DEFAULT) {
		interval = NOTIFY_INTERVAL_MS_DEFAULT;
	}

	notify_interval_ms = interval;

	if (!default_conn) {
		return len;
	}

	if (notify_interval_ms == 0 || !notify_enabled) {
		(void)k_work_cancel_delayable(&notify_work);
	} else {
		k_work_reschedule(&notify_work, K_NO_WAIT);
	}

	return len;
}

BT_GATT_SERVICE_DEFINE(eye_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_EYE_SVC),
	BT_GATT_CHARACTERISTIC(BT_UUID_EYE_ADC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_adc, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_EYE_CFG, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, write_cfg, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_EYE_STREAM, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void notify_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Work item is shared by both notify paths and reschedules itself. */
	if (!default_conn || notify_interval_ms == 0 || (!notify_enabled && !stream_notify_enabled)) {
		return;
	}

	uint16_t effective_interval_ms = notify_interval_ms;
	if (!stream_notify_enabled && notify_enabled && (notify_interval_ms == NOTIFY_INTERVAL_MS_DEFAULT)) {
		effective_interval_ms = MAX(1U, (1000U / CONFIG_EYE_BLE_STREAM_HZ));
	}

	uint32_t now_ms = (uint32_t)k_uptime_get_32();
	if (last_stats_uptime_ms == 0) {
		last_stats_uptime_ms = now_ms;
	}
	if (!IS_ENABLED(CONFIG_UART_CONSOLE) && (now_ms - last_stats_uptime_ms) >= 1000U) {
		printk("BLE notify: interval=%ums attempts=%u ok=%u err=%u (target=%uHz)\n",
		       effective_interval_ms, stat_notify_attempts, stat_notify_ok,
		       stat_notify_err, CONFIG_EYE_BLE_STREAM_HZ);
		printk("BLE ccc: ad83=%u ad85=%u mtu=%u qdrop=%u nerr=%u clip_lo=%u clip_hi=%u\n",
		       notify_enabled ? 1 : 0,
		       stream_notify_enabled ? 1 : 0,
		       bt_gatt_get_mtu(default_conn),
		       emg_sampler_get_stream_drop_count(),
		       emg_ble_stream_get_notify_err(),
		       emg_sampler_get_clip_lo(),
		       emg_sampler_get_clip_hi());
		stat_notify_attempts = 0;
		stat_notify_ok = 0;
		stat_notify_err = 0;
		last_stats_uptime_ms = now_ms;
	}

	if (stream_notify_enabled) {
		/*
		 * Stream notify mode:
		 * - Pulls a packet of samples from the sampler stream queue.
		 * - Chunks the packet to fit the current MTU.
		 * - Spreads chunks across the notify interval to reduce bursts.
		 */
		static uint32_t period_end_ms;
		static uint32_t staged_first_seq;
		static struct emg_stream_pair staged_samples[STREAM_SAMPLES_PER_PACKET];
		static uint32_t staged_count;
		static uint32_t staged_off;

		const uint16_t mtu = bt_gatt_get_mtu(default_conn);
		const uint16_t max_payload = (mtu > 3U) ? (mtu - 3U) : 0U;
		const uint16_t max_samples_per_notify =
			(max_payload > 5U) ? (uint16_t)((max_payload - 5U) / 4U) : 0U;

		if (max_samples_per_notify == 0U) {
			k_work_schedule(&notify_work, K_MSEC(notify_interval_ms));
			return;
		}

		/* Stage a new packet when the previous one has been fully sent. */
		if (staged_off >= staged_count) {
			if (period_end_ms != 0U && (int32_t)(now_ms - period_end_ms) < 0) {
				k_work_schedule(&notify_work, K_MSEC(period_end_ms - now_ms));
				return;
			}

			staged_off = 0;
			staged_count = emg_sampler_pop_stream(&staged_first_seq, staged_samples,
							     STREAM_SAMPLES_PER_PACKET);
			if (staged_count == 0U) {
				period_end_ms = 0U;
				k_work_schedule(&notify_work, K_MSEC(5));
				return;
			}
			period_end_ms = now_ms + effective_interval_ms;
		}

		const uint32_t remaining = staged_count - staged_off;
		const uint32_t chunk = MIN(remaining, (uint32_t)max_samples_per_notify);

		uint8_t payload[5 + 4 * STREAM_SAMPLES_PER_PACKET];
		const uint32_t first_seq = staged_first_seq + staged_off;

		sys_put_le32(first_seq, &payload[0]);
		payload[4] = (uint8_t)chunk;
		for (uint32_t i = 0; i < chunk; i++) {
			const struct emg_stream_pair *pair = &staged_samples[staged_off + i];
			sys_put_le16(pair->filtered, &payload[5 + 4 * i]);
			sys_put_le16(pair->envelope, &payload[5 + 4 * i + 2]);
		}

		stat_notify_attempts++;
		int err = bt_gatt_notify(default_conn, &eye_svc.attrs[7], payload, 5 + 2 * chunk);
		if (err) {
			stat_notify_err++;
			emg_ble_stream_note_notify_err();

			if (err == -ENOMEM || err == -EAGAIN) {
				k_work_schedule(&notify_work, K_MSEC(2));
				return;
			}

			k_work_schedule(&notify_work, K_MSEC(notify_interval_ms));
			return;
		}

		stat_notify_ok++;
		emg_ble_stream_note_notify_ok(chunk);
		emg_ble_stream_note_sent(first_seq + chunk - 1U,
					 staged_samples[staged_off + chunk - 1U].filtered);
		staged_off += chunk;

		if (staged_off < staged_count) {
			now_ms = (uint32_t)k_uptime_get_32();
			const uint32_t rem_samples = staged_count - staged_off;
			const uint32_t chunks_left =
				(rem_samples + max_samples_per_notify - 1U) / max_samples_per_notify;
			const uint32_t time_left_ms =
				(period_end_ms != 0U && (int32_t)(period_end_ms - now_ms) > 0)
					? (period_end_ms - now_ms)
					: 1U;
			const uint32_t chunk_period_ms = MAX(1U, time_left_ms / MAX(1U, chunks_left));
			k_work_schedule(&notify_work, K_MSEC(chunk_period_ms));
			return;
		}

		now_ms = (uint32_t)k_uptime_get_32();
		if (period_end_ms != 0U && (int32_t)(now_ms - period_end_ms) < 0) {
			k_work_schedule(&notify_work, K_MSEC(period_end_ms - now_ms));
		} else {
			k_work_schedule(&notify_work, K_NO_WAIT);
		}
		return;
	} else if (notify_enabled) {
		/* Single-sample notify mode (simple characteristic notify). */
		uint16_t sample = emg_sampler_get_latest_sample_u16();
		uint16_t sample_le = sys_cpu_to_le16(sample);

		stat_notify_attempts++;
		int err = bt_gatt_notify(default_conn, &eye_svc.attrs[2],
					 &sample_le, sizeof(sample_le));
		if (err) {
			stat_notify_err++;
		} else {
			stat_notify_ok++;
		}
	}

	k_work_schedule(&notify_work, K_MSEC(effective_interval_ms));
}

static void adv_work_handler(struct k_work *work)
{
	/* Deferred advertising start to keep startup ordering clean. */
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");

	default_conn = bt_conn_ref(conn);
	dk_set_led_on(CON_STATUS_LED);

	request_fast_conn_params(conn);

	/* nRF52810 does not support LE 2M PHY; skip bt_conn_le_phy_update to avoid link error. */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	notify_enabled = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
	(void)k_work_cancel_delayable(&notify_work);

	dk_set_led_off(CON_STATUS_LED);
}

static void recycled_cb(void)
{
	printk("Connection object available from previous conn. Disconnect is complete!\n");
	advertising_start();
}

#if defined(CONFIG_BT_CONN)
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	printk("LE params updated: interval %u (1.25ms), latency %u, timeout %u (10ms)\n",
	       interval, latency, timeout);
}
#endif

#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d %s\n", addr, level, err,
		       bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#if defined(CONFIG_BT_CONN)
	.le_param_updated = le_param_updated,
#endif
};

int main(void)
{
	int blink_status = 0;
	int err;
	uint32_t now_ms;

	/* Bring up console logging early for power-on visibility. */
	printk("Starting BLE ADC (P0.04) peripheral\n");
	printk("UART0 pins: TX=P0.06 RX=P0.12 (1000000)\n");
	printk("Stream target: %uHz (notify interval %ums)\n",
	       CONFIG_EYE_BLE_STREAM_HZ, NOTIFY_INTERVAL_MS_DEFAULT);
	printk("ADC sample rate: %u Hz\n", CONFIG_EYE_ADC_SAMPLE_RATE_HZ);

	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	/* Start the sampling pipeline before enabling streaming outputs. */
	err = emg_sampler_init();
	if (err) {
		printk("Sampler init failed (err %d)\n", err);
		return 0;
	}

	/* UART stream uses a dedicated thread and binary frames. */
	err = emg_uart_stream_init();
	if (err) {
		printk("UART stream init failed (err %d)\n", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	k_work_init_delayable(&notify_work, notify_work_handler);
	advertising_start();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);

		/* Once per second: print sampling and queue stats for troubleshooting. */
		now_ms = (uint32_t)k_uptime_get_32();
		if ((now_ms - last_sampler_log_ms) >= 1000U) {
			printk("ADC stats: raw=%u seq=%u drop=%u clip_lo=%u clip_hi=%u\n",
			       emg_sampler_get_latest_raw_u16(),
			       emg_sampler_get_latest_raw_seq(),
			       emg_sampler_get_stream_drop_count(),
			       emg_sampler_get_clip_lo(),
			       emg_sampler_get_clip_hi());
			printk("UART q: drop=%u hwm=%u\n",
			       emg_sampler_get_uart_drop(),
			       emg_sampler_get_uart_hwm());
			last_sampler_log_ms = now_ms;
		}

		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
