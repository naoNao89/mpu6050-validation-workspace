# mpu6050-driver

[![CI](https://img.shields.io/github/actions/workflow/status/naoNao89/mpu6050-validation-workspace/ci.yml?branch=main&label=CI&logo=github)](https://github.com/naoNao89/mpu6050-validation-workspace/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/mpu6050-driver.svg)](https://crates.io/crates/mpu6050-driver)
[![docs.rs](https://img.shields.io/docsrs/mpu6050-driver)](https://docs.rs/mpu6050-driver)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

`no_std` [embedded-hal](https://docs.rs/embedded-hal/1.0.0) 1.0 driver for the
InvenSense MPU-6050 6-axis IMU (accelerometer + gyroscope).

## Features

- `no_std`, blocking I2C via `embedded-hal` 1.0
- Wake / reset, `WHO_AM_I` identity decode
- Accel/gyro full-scale range, DLPF, and sample-rate divider
- Raw accel/temp/gyro block reads with optional suspicious-sample retry
- FIFO byte reads with diagnostics
- Data-ready and FIFO-overflow interrupt helpers
- Conversion helpers (`raw_to_imu_sample`, temperature in °C)
- Magnitude helpers for a quick stationary smoke check
  (`ImuSample::accel_magnitude_g`, `gyro_magnitude_dps`)

## Quick start

```rust
use mpu6050_driver::{
    AccelRange, Address, Dlpf, GyroRange, Mpu6050, raw_to_imu_sample,
};

# fn run<I2C>(i2c: I2C) -> Result<(), I2C::Error>
# where
#     I2C: embedded_hal::i2c::I2c,
# {
let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);

mpu.wake()?;
mpu.set_accel_range(AccelRange::G2)?;
mpu.set_gyro_range(GyroRange::Dps250)?;
mpu.set_dlpf(Dlpf::Cfg2)?;
mpu.set_sample_rate_divider(4)?; // ~200 Hz with Cfg2

let _identity = mpu.identity()?;
let raw = mpu.read_raw_accel_gyro_temp()?;
let sample = raw_to_imu_sample(raw);
let _accel_mag_g = sample.accel_magnitude_g();
# Ok(())
# }
```

Replace `i2c` with your board’s `embedded_hal::i2c::I2c` implementation.
`Address::Ad0Low` is `0x68` (AD0 tied low); use `Address::Ad0High` for `0x69`.

## Examples

From this crate directory (or after `cargo package` extract):

```bash
cargo run --example convert_raw
cargo run --example wake_and_read
cargo run --example stationary_usability
```

`wake_and_read` uses a mock I2C bus so it runs on the host without hardware.
`stationary_usability` shows a single-sample \|a\|≈1 g / quiet-gyro smoke check
(not clone detection, not a full log analyzer).

## Cargo

```toml
[dependencies]
mpu6050-driver = "0.1.1"
```

## Validating that a module is usable

This driver does **not** judge authenticity or “clone vs original.” `WHO_AM_I`
and `Identity` only report what the bus returned.

**In this crate (lightweight):**

1. Wake and configure ranges.
2. Leave the board **still**.
3. `raw_to_imu_sample` → [`ImuSample::accel_magnitude_g`](https://docs.rs/mpu6050-driver/latest/mpu6050_driver/struct.ImuSample.html) ≈ **1 g**,
   [`gyro_magnitude_dps`](https://docs.rs/mpu6050-driver/latest/mpu6050_driver/struct.ImuSample.html) small.
4. See `examples/stationary_usability.rs` for pass/fail smoke gates.

**Not in this crate (full reports):** multi-sample thresholds, stationary
fraction, and JSON verdicts live in the workspace `imu-tool`
(`analyze` / `stationary-suite`) — optional host tooling only.

## Validation workspace

Developed inside
[naoNao89/mpu6050-validation-workspace](https://github.com/naoNao89/mpu6050-validation-workspace)
(ESP32 firmware + `imu-tool`). Those paths are **not** part of this package.

## License

MIT. See [LICENSE](LICENSE).
