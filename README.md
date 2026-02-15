# t-hmi

Rust-based HMI project targeting ESP-IDF.

## Requirements
- Rust toolchain (edition 2021)
- ESP-IDF toolchain and environment set up for Rust

## Build
```bash
cargo build
```

## Flash / Run
```bash
cargo run
```

## Host Logging (without `cargo run`)
If firmware is already flashed, you can read logs directly from the serial port:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
tio /dev/ttyACM0 -b 115200
```

Optional: show only GPS diagnostics

```bash
tio /dev/ttyACM0 -b 115200 | stdbuf -oL grep '^\[GPS\]'
```

GPS host logs are only enabled while the UI is on `Device -> GPS`.

## Project Structure
- `src/main.rs`: Application entry point
- `src/ui.rs`: UI/HMI rendering and interaction logic
- `src/device.rs`: Device-specific handling
- `src/gps.rs`: GPS-related functionality

## Notes
- Configuration defaults live in `sdkconfig.defaults`.
- Build tooling is configured in `build.rs`.
