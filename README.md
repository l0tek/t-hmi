# t-hmi

Rust-based HMI project for ESP32-S3 (ESP-IDF via `esp-idf-sys`).

## Requirements
- Rust toolchain (edition 2021)
- ESP-IDF toolchain/environment for Rust
- Connected board (serial, e.g. `/dev/ttyACM0`)

## Build
```bash
cargo build
```

## Flash / Run
```bash
cargo run
```

## UI Overview
### Main Menu
- `WiFi`
- `Bluetooth` (placeholder)
- `GPS`
- `Device`
- `Reboot`

### WiFi Menu
- `Scan Wifi Network`
  - Performs active scan and displays SSID + RSSI list.
- `Monitor RSSI`
  - Continuously scans and plots RSSI curves (top networks).
- `Channel Monitor`
  - Sets channel (1..13) and shows packet counters by type:
  - Management / Data / Control
  - Channel can be changed with on-screen left/right buttons.

### Device Menu
- `Batteriestatus`
  - Live VBAT/ADC/raw/SoC/calibration display.
- `GPS`
  - GPS values view (fix/sats/lat/lon + diagnostics).
- `UART Loopback`
  - UART send/receive loopback diagnostics.
- `SD Format`
  - SD mount + format + write/read `test.txt`.
  - Progress is shown on display in percent (`0%`..`100%`).
  - Result screen stays visible until Back button is pressed.

## Host Logging (optional)
If firmware is already flashed, read logs directly:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
tio /dev/ttyACM0 -b 115200
```

Only GPS diagnostics:

```bash
tio /dev/ttyACM0 -b 115200 | stdbuf -oL grep '^\[GPS\]'
```

GPS host logs are only active while UI is on `Device -> GPS`.

## Project Structure
- `src/main.rs`: app state machine and screen navigation
- `src/ui.rs`: drawing/layout for all screens
- `src/wifi.rs`: WiFi init, scan, monitor, channel/promiscuous capture
- `src/device.rs`: battery readout logic
- `src/gps.rs`: GPS reader and UART loopback logic
- `src/sdcard.rs`: SD mount/format/test flow

## Notes
- Defaults/config: `sdkconfig.defaults`
- Build helper: `build.rs`
