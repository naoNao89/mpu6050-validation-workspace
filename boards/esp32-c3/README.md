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

Check the firmware without building host-only workspace members for the embedded
target:

```sh
make check-firmware
```
