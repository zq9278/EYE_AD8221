# EYE_AD8221 Data Path (ADC -> UART/BLE)

## Overview
- MCU: nRF52 (nrf52dk_nrf52810), Zephyr/NCS v3.1.1
- ADC sampling: 2 kHz on SAADC AIN2 (P0.04), 12-bit, gain 1/6, 40 us acquisition
- UART (VOFA+): 1,000,000 bps, TX=P0.06, RX=P0.12, 3 channels
  - CH1: raw ADC (2 kHz)
  - CH2: envelope/energy, downsampled to BLE rate
  - CH3: notch-filtered (50/60 Hz) sample at BLE rate
- BLE: custom service 0000ad82-1212-efde-1523-785feabcd123, stream char 0000ad85-1212-efde-1523-785feabcd123 (notify-only)
  - Payload: `first_seq (u32 LE)` + `count (u8)` + `count * sample (u16 LE)`

## Data Flow (firmware)
1) **ADC read** (`src/emg_sampler.c`)  
   - SAADC -> `raw_u16` (0..4095), clip counters updated if saturated.  
   - `latest_raw_u16` published for UART CH1.
2) **Centering**  
   - `centered = raw_u16 - 2048`.
3) **Notch (always on)**  
   - Apply one biquad (`emg_notch_process`); optional second pass if `CONFIG_EYE_NOTCH_DOUBLE=y`.  
   - Center frequency set by `CONFIG_EYE_NOTCH_FREQ_HZ` (50/60), Q by `CONFIG_EYE_NOTCH_Q`.
4) **Envelope / energy**  
   - If `CONFIG_EYE_ENVELOPE_ENABLED=y`:  
     `rect = abs(filtered)`; `env = env + ((rect - env) >> CONFIG_EYE_ENVELOPE_TAU_SHIFT)` (first-order IIR).  
     The envelope is multiplied by 2 for visibility then clamped to 12-bit.  
   - If disabled: use the notched sample directly.
5) **Decimation to BLE/UART CH2/CH3 rate**  
   - `CONFIG_EYE_BLE_STREAM_HZ` (current 200 Hz); keep every `fs/out_hz` sample for CH2/CH3 and BLE.  
   - CH1 stays at full 2 kHz on UART.
6) **Publish sample**  
   - `latest_sample_u16` (envelope/filtered) and `latest_notch_u16` (notched) updated.  
   - Enqueue decimated samples into `stream_q` with incrementing `seq`.
7) **BLE notify** (`src/main.c`)  
   - Pops up to `STREAM_SAMPLES_PER_PACKET` from `stream_q`, fragments by MTU into multiple notifies per period.  
   - Uses characteristic `ad85` (notify). `first_seq` and `count` allow the client to detect gaps.
8) **UART stream** (`src/emg_uart_stream.c`)  
   - Period = 1/`CONFIG_EYE_UART_STREAM_HZ` (2 kHz).  
   - Sends CH1 raw (float32, 2 kHz) + CH2 envelope/filtered (float32, 200 Hz) + CH3 notch (float32, 200 Hz) + tail `00 00 80 7F`.

## Key Configs (prj.conf / prj_minimal.conf)
- Sampling: `CONFIG_EYE_ADC_SAMPLE_RATE_HZ=2000`
- Envelope: `CONFIG_EYE_ENVELOPE_ENABLED=y`, `CONFIG_EYE_ENVELOPE_TAU_SHIFT=5` (lower = faster response)
- Notch: `CONFIG_EYE_NOTCH_ENABLED=y`, `CONFIG_EYE_NOTCH_FREQ_HZ=50`, `CONFIG_EYE_NOTCH_Q=20`, `CONFIG_EYE_NOTCH_DOUBLE=n`
- Stream rate: `CONFIG_EYE_BLE_STREAM_HZ=200`
- Packet rate: `CONFIG_EYE_BLE_PACKET_HZ=50`
- UART: `CONFIG_EYE_UART_STREAM_HZ=2000`, `CONFIG_EYE_UART_STREAM_CHANNELS=3`
- Pins/baud: overlay sets UART0 TX=P0.06, RX=P0.12, `current-speed=1000000`
- BLE conn params: min/max interval 6/12 (1.25 ms units), latency 0, timeout 400

## BLE Client Notes
- Subscribe to 0000ad85... notifications.  
- Parse LE payload: u32 `first_seq`, u8 `count`, then `count` u16 samples.  
- Detect gaps: `gap = first_seq - last_seq - 1`; accumulate drops.  
- Expected rate: ~200 samples/s total, fragmented into ~50 notifies/s.

## File Pointers
- Sampling, notch, envelope: `src/emg_sampler.c`
- Notch coeffs: `src/emg_notch.c`
- UART stream framing: `src/emg_uart_stream.c`
- BLE GATT/service/notify: `src/main.c`
- Config knobs: `prj.conf`, `prj_minimal.conf`, `Kconfig`
