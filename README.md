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
- `HTTP SD`
  - Mini HTTP server submenu to expose SD card content over WiFi.
  - Start/Stop from UI (`http://<IP>:8080/sd`).
  - Requires active STA WiFi connection with valid IPv4.
- `WiFi Login`
  - Scan/select AP, password input on device, connect from UI.
  - `Default` button connects using credentials from `wifi_secrets.local`.
  - Connection strategy prefers direct SSID connect (`wifi_connect`) and falls back to AP-pinned connect (`wifi_connect_ap`) if needed.
  - After successful connect, a dedicated `WiFi Verbunden` screen shows:
  - SSID, channel, RSSI, IPv4 address.

## HTTP SD Usage
1. Connect WiFi first (IP must not be `0.0.0.0`).
2. Open `Device -> Mini HTTP`.
3. Tap `Start HTTP SD`.
4. Open `http://<ESP_IP>:8080/sd` from a client in the same network.

## WiFi Default Credentials
Default credentials are injected at build time from `wifi_secrets.local`
and are intentionally git-ignored.

Format:

```text
SSID=<your_ssid>
PASSWORD=<your_password>
```

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
- `src/wifi.rs`: WiFi init, scan, monitor, channel/promiscuous capture, connect helpers
- `src/device.rs`: battery readout logic
- `src/gps.rs`: GPS reader and UART loopback logic
- `src/sdcard.rs`: SD mount/format/test flow
- `src/http_server.rs`: mini HTTP server for SD browse/download

## Notes
- Defaults/config: `sdkconfig.defaults`
- Build helper: `build.rs` (reads `wifi_secrets.local` and sets `WIFI_DEFAULT_*`)
