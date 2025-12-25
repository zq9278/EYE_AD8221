# EYE_AD8221 Data Path (ADC ➜ UART/BLE)

## Overview
- MCU: nRF52 (nrf52dk_nrf52810), Zephyr/NCS v3.1.1
- ADC sampling: 2 kHz on SAADC AIN2 (P0.04), 12-bit, gain 1/6, 40 µs acquisition
- UART (VOFA+): 1,000,000 bps, TX=P0.06, RX=P0.12, 2 channels
  - CH1: raw ADC (2 kHz)
  - CH2: envelope/energy, downsampled and sent at BLE rate
- BLE: custom service 0000ad82-1212-efde-1523-785feabcd123, stream char 0000ad85-1212-efde-1523-785feabcd123 (notify-only)
  - Payload: `first_seq (u32 LE)` + `count (u8)` + `count * sample (u16 LE)`

## Data Flow (firmware)
1) **ADC read** (`src/emg_sampler.c`)
   - SAADC read -> `raw_u16` (0..4095)
   - Stats: clip_lo/clip_hi incremented if saturated
   - `latest_raw_u16` published for UART CH1
2) **Centering**  
   - `centered = raw_u16 - 2048`
3) **Notch (optional, currently disabled)**  
   - If `CONFIG_EYE_NOTCH_ENABLED=y`, apply one or two biquads (`emg_notch_process`), 50/60 Hz selectable, Q configurable.
4) **Envelope / Energy**  
   - If `CONFIG_EYE_ENVELOPE_ENABLED=y` (current):  
     `rect = abs(filtered)`  
     `env = env + ((rect - env) >> CONFIG_EYE_ENVELOPE_TAU_SHIFT)` (first-order IIR, tau ≈ 16 ms at shift=5)  
     `env` is multiplied by 2 (visibility) and clamped to 12-bit.
   - If disabled: use `filtered` (centered or notched) directly.
5) **Decimation to BLE/UART CH2 rate**  
   - `CONFIG_EYE_BLE_STREAM_HZ` (current 200 Hz).  
   - Only every `fs/out_hz` sample is kept for stream.
6) **Publish sample**  
   - `latest_sample_u16` (used by UART CH2 and BLE).  
   - Enqueue into `stream_q` with incrementing `seq`.
7) **BLE notify** (`src/main.c`)  
   - Work handler pops up to `STREAM_SAMPLES_PER_PACKET` from `stream_q`, fragments by MTU into multiple notifies per period.  
   - Uses characteristic `ad85` (notify). Sequence and count allow client to detect gaps.
8) **UART stream** (`src/emg_uart_stream.c`)  
   - Period = 1/`CONFIG_EYE_UART_STREAM_HZ` (2 kHz).  
   - Sends CH1 raw (float32) + CH2 envelope/downsampled (float32) + tail `00 00 80 7F`.

## Key Configs (prj.conf / prj_minimal.conf)
- Sampling: `CONFIG_EYE_ADC_SAMPLE_RATE_HZ=2000`
- Envelope: `CONFIG_EYE_ENVELOPE_ENABLED=y`, `CONFIG_EYE_ENVELOPE_TAU_SHIFT=5` (faster response: lower; slower: higher)
- Stream rate: `CONFIG_EYE_BLE_STREAM_HZ=200`
- Packet rate: `CONFIG_EYE_BLE_PACKET_HZ=50` (≈4 samples/notify)
- Notch (currently off): `CONFIG_EYE_NOTCH_ENABLED=n`, `CONFIG_EYE_NOTCH_FREQ_HZ=50`, `CONFIG_EYE_NOTCH_Q=20`, `CONFIG_EYE_NOTCH_DOUBLE=n`
- UART: `CONFIG_EYE_UART_STREAM_HZ=2000`, `CONFIG_EYE_UART_STREAM_CHANNELS=2`
- Pins/baud: overlay sets UART0 TX=P0.06, RX=P0.12, `current-speed=1000000`

## BLE Client Notes
- Subscribe to 0000ad85... notify.  
- Parse LE payload: u32 seq, u8 count, then `count` u16 samples.  
- Detect gaps: `gap = first_seq - last_seq - 1`; accumulate drops.  
- Expected rate: ~200 samples/s total, fragmented into ~50 notifies/s.

## File Pointers
- Sampling & envelope: `src/emg_sampler.c`
- Notch coeffs: `src/emg_notch.c`
- UART stream: `src/emg_uart_stream.c`
- BLE GATT/service/notify: `src/main.c`
- Config knobs: `prj.conf`, `prj_minimal.conf`, `Kconfig`
