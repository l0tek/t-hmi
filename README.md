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

## Notes
- Configuration defaults live in `sdkconfig.defaults`.
- Build tooling is configured in `build.rs`.
