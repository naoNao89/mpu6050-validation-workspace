# ESP32-C3 MPU6050 firmware

Board-specific bring-up firmware for the connected ESP32-C3 + MPU6050 setup.

Reference wiring for this board profile:

| GY-521/MPU6050 | ESP32-C3 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO0 |
| SDA | GPIO1 |
| XDA | GPIO3 |
| XCL | GPIO4 |
| AD0 | GPIO5 |
| INT | GPIO6 |

XDA/XCL are the MPU6050 auxiliary I2C pins for optional external sensors. They
are mapped here because they are part of the wired module header, but this
bring-up firmware validates only the primary SDA/SCL I2C path for
accel/gyro/temp. Aux I2C or bypass mode needs separate firmware and tests.

TODO: Add aux-I2C/bypass-mode validation for XDA/XCL before claiming support for
external sensors on the MPU6050 auxiliary I2C bus.

Build, flash, and monitor from the repository root using the root `Makefile` or `run.sh`, for example:

```sh
make build
./run.sh
```

Continuous binary motion stream (v1 frames for `imu-tool --mode binary`):

```sh
MODE=binary DURATION=30 LOG_FILE=logs/motion-binary.log ./run.sh
```

`MODE=binary` builds with `--features binary-frames`. Default text mode is unchanged.

Binary frame `timestamp_us` values are device-side successful **read-completion**
timestamps (after the motion I²C read finishes), not the original GPIO data-ready
edge time. Frame `sequence` values track **emitted frames** and detect transport
loss or decode gaps; they do not encode coalesced data-ready events or failed
sensor reads. `LOG_FILE` in binary mode is a **decoded text sample log** written
by `imu-tool`, not a raw byte capture of the serial stream.

Check the firmware without building host-only workspace members for the embedded
target:

```sh
make check-firmware
```
