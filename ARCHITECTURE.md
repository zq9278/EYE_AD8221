# Architecture

## Zephyr System Architecture (NCS v3.1.1)

This project uses Zephyr through nRF Connect SDK. The key layers involved are:

- Application layer: user code in `src/` plus Kconfig options in `prj.conf`.
- Kernel: threads, work queues, timers, atomics, message queues, and ISR dispatch.
- Drivers: nrfx drivers for SAADC, TIMER, PPI, UARTE; console RTT driver.
- Subsystems: Bluetooth host, controller, and Nordic MPSL integration.
- Device tree: board and overlay define pins and peripherals.
- Build system: west invokes CMake, Kconfig merges `prj.conf`, and DTS is
  compiled into generated headers.

Execution model summary:

- Interrupts handle time-critical sampling (SAADC DONE).
- Kernel threads handle streaming (UART thread) and background work (BLE notify
  work item).
- Atomics and message queues bridge ISR and thread context safely.

## Program Architecture (EYE_AD8221)

### Top-level modules

- `src/main.c`
  - Initializes BLE, sampler, and UART stream.
  - Implements GATT service and notification logic.
  - Emits periodic stats and toggles LEDs.

- `src/emg_sampler.c`
  - Configures SAADC, TIMER2, and PPI for continuous sampling.
  - Processes samples: notch -> DC-block -> optional envelope.
  - Publishes data to atomics and queues.

- `src/emg_notch.c`
  - Provides integer biquad notch filtering in Q30.
  - Uses precomputed coefficients for common fs and f0.
  - Falls back to runtime coefficient calculation when needed.

- `src/emg_uart_stream.c`
  - Dedicated thread that outputs binary frames on UART0.
  - Frame format: 0xAA 0x55 <len=0x06> <raw> <notch> <envelope>.

- `src/emg_ble_stream.c`
  - Tracks BLE stream stats for diagnostics.

- `src/fatal_hook.c`
  - Prints fault info and halts the CPU to keep RTT/UART readable.

### Data flow

```
SAADC (AIN2) -> ISR -> process_sample()
  raw -> notch -> high-pass -> envelope
       |            |           |
       |            |           +-> latest env (atomic)
       |            +-> latest filtered (atomic)
       +-> latest raw (atomic)
       +-> UART queue (full rate)
       +-> BLE stream queue (decimated)

BLE GATT notify (work item)
  - single sample: latest filtered
  - stream: packetized queue (filtered + envelope pairs)

UART stream thread
  - outputs binary frame per sample
```

### Timing and rates

- Sample rate: `CONFIG_EYE_ADC_SAMPLE_RATE_HZ` (timer driven).
- BLE stream rate: `CONFIG_EYE_BLE_STREAM_HZ` (decimated from fs).
- BLE packet rate: `CONFIG_EYE_BLE_PACKET_HZ` (notify pacing).

### Concurrency

- SAADC DONE interrupt calls `process_sample()` for each DMA sample.
- UART stream uses a cooperative thread to serialize binary frames.
- BLE notifications are scheduled with a delayable work item.

### Resource notes

- RAM is tight on nRF52810, so queue sizes are intentionally limited.
- UART output uses polling to reduce ISR overhead.
- BLE stream can be disabled at build time with
  `CONFIG_EYE_BLE_STREAM_DISABLED`.
