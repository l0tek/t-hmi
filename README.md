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

## Project Structure
- `src/main.rs`: Application entry point
- `src/ui.rs`: UI/HMI rendering and interaction logic
- `src/device.rs`: Device-specific handling
- `src/gps.rs`: GPS-related functionality

## Notes
- Configuration defaults live in `sdkconfig.defaults`.
- Build tooling is configured in `build.rs`.
