# ESP32-P4 MPU6050 firmware

Board-specific bring-up firmware for an ESP32-P4 connected to a GY-521/MPU6050 module.

Reference wiring for this board profile:

| GY-521/MPU6050 | ESP32-P4 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO20 |
| SDA | GPIO21 |
| XDA | GPIO22 |
| XCL | GPIO23 |
| AD0 | GPIO26 |
| INT | GPIO27 |

XDA/XCL are the MPU6050 auxiliary I2C pins for optional external sensors. They
are documented here because they are part of the wired module header, but the
firmware validates the primary SDA/SCL I2C path for accel/gyro/temp.

ESP32-P4 support intentionally uses a pinned esp-rs fork/pre-v3.0 silicon patch
revision tested with ESP32-P4 rev v1.3/ECO2/pre-v3.0. The related upstream PR
was closed unmerged, so this is not a crates.io-release-track dependency. Match
the minimum chip revision config to your actual silicon before use; the
repository Makefile defaults `P4_CHIP_REVISION=100` for the tested board.
If you run `cargo` directly inside `boards/esp32-p4`, `.cargo/config.toml` also
defaults to revision 100; use the Makefile or override the
`ESP_HAL_CONFIG_MIN_CHIP_REVISION` and `ESP_SYNC_CONFIG_MIN_CHIP_REVISION`
environment variables for other ESP32-P4 revisions.

The `mpu6050-driver` dependency intentionally stays as the crate dependency
(`mpu6050-driver = "0.1.0"`). Local path overrides are only for ad-hoc hardware
testing before the crate is available from the registry used by the board crate.

GPIO26/GPIO27 may overlap USB Full-Speed D-/D+ on some ESP32-P4 boards. They are
listed because they match the current wired setup; verify the exact dev-board
schematic before relying on native USB while these pins are wired. GPIO27/INT is
an MPU output and the firmware treats it as input-only.

The board crate is intentionally standalone. Until `mpu6050-driver = "0.1.0"`
is available from the registry used by Cargo, build it from this repository with
the local patch shown below.

Useful validation features:

```sh
cd boards/esp32-p4
cargo +1.95.0 --config 'patch.crates-io.mpu6050-driver.path="../../crates/mpu6050-driver"' build --release --features boot-probe
cargo +1.95.0 --config 'patch.crates-io.mpu6050-driver.path="../../crates/mpu6050-driver"' build --release --features i2c-probe
cargo +1.95.0 --config 'patch.crates-io.mpu6050-driver.path="../../crates/mpu6050-driver"' build --release --features i2c-bitbang
cargo +1.95.0 --config 'patch.crates-io.mpu6050-driver.path="../../crates/mpu6050-driver"' build --release --features mpu-smoke
cargo +1.95.0 --config 'patch.crates-io.mpu6050-driver.path="../../crates/mpu6050-driver"' run --release --features boot-probe -- --port /dev/cu.usbmodemXXXX
```

- `boot-probe`: verifies app entry, UART logging, and `esp_hal::init`.
- `pin-wiggle`: continuity test for GPIO20/21/22/23/26 with GPIO27/INT read as
  an input. Use only when you understand the wiring; it intentionally drives the
  non-INT pins as GPIO outputs for meter probing.
- `i2c-probe`: low-side-effect I2C diagnostic that logs SCL/SDA/INT idle levels,
  probes WHO_AM_I at 0x68/0x69 with AD0 low and high, and then idles without
  wake or raw streaming. Use this to distinguish hardware no-ACK from runtime
  bring-up issues.
- `i2c-bitbang`: software open-drain ACK probe on SCL=GPIO20/SDA=GPIO21. It toggles
  START/address/ACK/STOP for 0x68 and 0x69 with AD0 low and high, bypassing the
  HAL I2C peripheral so pinmux/peripheral issues can be separated from module
  no-ACK.
- `mpu-smoke`: initializes I2C on SCL=GPIO20/SDA=GPIO21 via software bitbang,
  drives AD0 low, and probes MPU addresses 0x68 and 0x69. If 0x68 responds, raw
  streaming starts. If not, streaming is skipped to avoid timeout spam.

From the repository root, equivalent Make targets are available:

```sh
make check-firmware-p4
make build-p4
make build-p4-full
make build-p4 P4_FEATURES=i2c-probe
make build-p4 P4_FEATURES=i2c-bitbang
make build-p4 P4_FEATURES=mpu-smoke
make flash-p4 PORT=/dev/cu.usbmodemXXXX P4_FEATURES=mpu-smoke
make monitor-p4 PORT=/dev/cu.usbmodemXXXX
make run-p4 PORT=/dev/cu.usbmodemXXXX P4_FEATURES=
```

`make build-p4` defaults to the conservative `boot-probe` feature. Use
`make build-p4-full` or `make build-p4 P4_FEATURES=` for the default/full
pipeline: scan, wake, probe, verification summary, then raw streaming every
100ms.

## Current status

**Verified working on ESP32-P4 rev v1.3 (ECO2).**

- ESP32-P4 HAL I2C peripheral (v3) does not emit transactions on this silicon
  revision — zero interrupts fire, always times out.
- Software bitbang I2C (`BitbangI2c` in `src/bitbang_i2c.rs`) replaces the HAL
  peripheral as the primary transport. It implements the `embedded_hal::i2c::I2c`
  trait using open-drain Flex GPIO pins and a NOP-based delay loop.
- MPU6050 responds at 0x68 (AD0 Low): WHO_AM_I=0x70 (MPU-6500-compatible),
  raw accel/gyro/temp block readable, verification score 8/13
  (`FunctionalRegisterCompatibleImu`).
