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
### Boot
- On startup a boot screen shows initialization logs (display/NVS/touch/calibration/WiFi/HTTPD).
- After boot sequence finishes, UI switches to `Main Menu`.

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
  - Live power source + voltage + percent (`USB` / `Akku` / `Unbekannt`).
- `GPS`
  - GPS values view (fix/sats/lat/lon + diagnostics).
- `LoRa Test`
  - UART2 LoRa diagnostic screen for EBYTE E220 modules.
  - Pin mapping on this project: `RX=GPIO18`, `TX=GPIO17`.
  - Sends periodic `ping #NNN` at fixed UART settings (`9600 8N1`).
  - The UI/logging identifies the unmodified E220-900T22D defaults as
    `873.125 MHz / Air 2.4 kbit/s` (channel 23 / `0x17`).
  - The E220 chooses its internal LoRa modulation parameters from the configured
    air data rate; raw `SF/BW/CR` values and LLCC68 IRQ registers are not exposed
    through its transparent UART interface.
  - Shows RX stats/hex and console-style TX/RX status lines.
  - No runtime baud scan and no register probing (configuration mode and
    `C1 00 08` register reads require wired `M0/M1`; `AUX` is recommended).
  - Receiving a transmitted ping requires a second, identically configured E220
    or another radio peer that sends data back; the module does not echo itself.
- `SD Format`
  - SD mount + format + write/read `test.txt`.
  - Progress is shown on display in percent (`0%`..`100%`).
  - Result screen stays visible until Back button is pressed.
- `Mini HTTP`
  - Mini HTTP server submenu to expose SD card content and remote control over WiFi.
  - Start/Stop from UI (`http://<IP>:8080/`).
  - Requires active STA WiFi connection with valid IPv4.
- `WiFi Login`
  - Scan/select AP, password input on device, connect from UI.
  - `Default` button tries all default credentials from `wifi_secrets.local` in order.
- `GPS Logging`
  - Separate screen with logging toggle/status.
  - Can keep logging in background while using other screens.
- `LoRa Logging`
  - Separate screen with logging toggle/status.
  - Can keep logging in background while using other screens.

## LoRa / E220 Wiring and Configuration Access

The current LoRa data connection uses UART2 in transparent mode:

| T-HMI | EBYTE E220 | Direction |
| --- | --- | --- |
| `GPIO17` (UART2 TX) | `RXD` | T-HMI to E220 |
| `GPIO18` (UART2 RX) | `TXD` | E220 to T-HMI |
| `GND` | `GND` | Common ground |

UART TX and RX must be crossed as shown. The firmware uses `9600 8N1`.

### M0/M1 via the free Grove3 connector

Grove3 is free in this project and exposes `GPIO43` and `GPIO44`. These two
signals can control the E220 operating mode:

| Grove3 / T-HMI | EBYTE E220 |
| --- | --- |
| `GPIO43` | `M0` |
| `GPIO44` | `M1` |
| `GND` | `GND` |
| Grove3 supply wire | Leave disconnected and insulate |

Do not connect the Grove3 supply wire to `M0` or `M1`. The E220 mode inputs are
3.3 V logic signals and must not be driven with 5 V. Verify the signal names and
wire order against the T-HMI pinout instead of relying only on cable colors.

The matching cable is the **LILYGO Grove Interface Cable, variant P354**, which
LILYGO lists as compatible with the T-HMI, T-Embed and T-RGB:

- <https://lilygo.cc/products/grove-interface-cable>
- Select `Grove [P354]`, not `QT/Qwiic to Grove [P351]`.
- A standard large Seeed Grove cable is not the same T-HMI-side connector.

### E220 operating modes used here

| M1 | M0 | Mode |
| ---: | ---: | --- |
| `0` | `0` | Normal / transparent transmission |
| `1` | `1` | Configuration mode |

At boot, configure `GPIO43` and `GPIO44` as outputs and drive both low. To read
the E220 configuration:

1. Set `M0=1` and `M1=1`.
2. Wait for the mode change to complete; waiting for `AUX=HIGH` is recommended.
3. Send the bytes `C1 00 08` over UART2 at `9600 8N1`.
4. Read and decode the returned register bytes.
5. Set `M0=0` and `M1=0` to return to transparent operation.

Example GPIO initialization (not yet implemented in the current firmware):

```rust
let mut lora_m0 = PinDriver::output(p.pins.gpio43)?;
let mut lora_m1 = PinDriver::output(p.pins.gpio44)?;

lora_m0.set_low()?;
lora_m1.set_low()?;
```

For a one-time manual register read, `M0` and `M1` can instead be switched
together between `GND` (normal mode) and `3V3` (configuration mode). Always
switch wiring with power removed. Automated register reads require the GPIO
control above; an additional connection from E220 `AUX` to a free T-HMI input
is optional but recommended.

Pins `GPIO11` through `GPIO13` are used by the T-HMI SD-card interface and
`GPIO19`/`GPIO20` are used by native USB, so they should not be substituted for
the Grove3 mode-control pins in this project.

### Header Status Icons
- Header shows `W` (WiFi) and `H` (HTTPD) status icons on all screens.
- Green = active, red = inactive.

## SD Log Files
- GPS logs: `gps_YYYYMMDD_HHMMSS_<runid>.txt`
- LoRa logs: `lora_YYYYMMDD_HHMMSS_<runid>.txt`
- If long filenames are not available on SD/FAT, firmware falls back to short 8.3 names.

## HTTP Dashboard
1. On boot, firmware automatically tries default WiFi credentials from `wifi_secrets.local` in order.
2. If connection + IPv4 succeeds, the HTTP server auto-starts on `http://<ESP_IP>:8080/`.
3. You can still control server state via `Device -> Mini HTTP` (`Start/Stop HTTP SD`).
4. Main routes:
   - `/` Dashboard (status + command buttons)
   - `/logs` Logfile list (view/download)
   - `/sd` SD card explorer
   - `/sd/view?file=<name>` direct text preview (truncated to 16 KiB)
   - `/sd/download?file=<name>` direct download

## HTTP Remote Commands
Commands are queued by HTTP and executed in the main UI loop.

Endpoint:

```text
/api/cmd?cmd=<command>[&param=value]
```

Supported commands:
- `goto_menu`
- `goto_wifi_menu`
- `goto_device_menu`
- `goto_gps`
- `goto_battery`
- `goto_loopback`
- `goto_http_menu`
- `goto_wifi_login`
- `wifi_scan`
- `wifi_monitor`
- `wifi_channel_monitor`
- `wifi_monitor_stop`
- `wifi_connect_default`
- `wifi_connect&ssid=<SSID>&password=<PWD>`
- `toggle_http`
- `sd_format`

Use with care: these commands control live UI/workflows on the device.

## Time Source
- Preferred: NTP (`pool.ntp.org`) after WLAN connect.
- Fallback: GPS time from NMEA (`RMC` / `ZDA`).
- This time is used for GPS/LoRa logfile naming.

## WiFi Default Credentials
Default credentials are injected at build time from `wifi_secrets.local`
and are intentionally git-ignored. Multiple defaults are supported and
tried sequentially.

Format:

```text
# Legacy single default
SSID=<your_ssid>
PASSWORD=<your_password>

# Multiple defaults (recommended)
WIFI_1_SSID=<ssid_1>
WIFI_1_PASSWORD=<password_1>
WIFI_2_SSID=<ssid_2>
WIFI_2_PASSWORD=<password_2>
# ...
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

Only LoRa diagnostics:

```bash
tio /dev/ttyACM0 -b 115200 | stdbuf -oL grep '^\[LORA_LOG\]'
```

## Project Structure
- `src/main.rs`: app state machine, screen navigation, GPS/LoRa SD logging
- `src/ui.rs`: drawing/layout for all screens
- `src/wifi.rs`: WiFi init, scan, monitor, channel/promiscuous capture, connect helpers
- `src/device.rs`: battery readout and source detection logic
- `src/gps.rs`: GPS reader + UART loopback helpers
- `src/lora.rs`: LoRa tester (UART2 diagnostics)
- `src/sdcard.rs`: SD mount/format/test flow
- `src/http_server.rs`: mini HTTP server for dashboard/SD browse/download
- `src/remote.rs`: remote command queue/status bridge (HTTP -> UI loop)

## Notes
- Defaults/config: `sdkconfig.defaults`
- Build helper: `build.rs` (reads `wifi_secrets.local` and sets `WIFI_DEFAULT_*`)
- Long FAT filenames require FATFS LFN options enabled in sdkconfig.
