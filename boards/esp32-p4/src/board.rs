//! ESP32-P4 board to GY-521/MPU6050 wiring map.

/// Reference dev-board wiring used by this bring-up firmware.
///
/// The repo's board-under-test is an ESP32-P4 board connected to
/// a GY-521/MPU6050 module. Keep these constants aligned with the README so the
/// firmware is explicitly a board sample that exercises this MPU6050 driver
/// stack, rather than an anonymous ESP32-P4 snippet.
pub const BOARD_NAME: &str = "ESP32-P4 dev board";
pub const I2C_BUS_NAME: &str = "I2C0";
pub const I2C_FREQUENCY_KHZ: u32 = 100;

// Human wiring map:
// ESP32-P4 3V3    -> MPU6050 VCC
// ESP32-P4 GND    -> MPU6050 GND
// ESP32-P4 GPIO20 -> MPU6050 SCL
// ESP32-P4 GPIO21 -> MPU6050 SDA
// ESP32-P4 GPIO22 -> MPU6050 XDA
// ESP32-P4 GPIO23 -> MPU6050 XCL
// ESP32-P4 GPIO26 -> MPU6050 AD0
// ESP32-P4 GPIO27 -> MPU6050 INT
//
// GPIO26/GPIO27 can overlap USB Full-Speed signals on some ESP32-P4 boards.
// They are kept here because they match the current wired setup, but the
// `pin-wiggle` validation feature should be used carefully to prove these
// signals reach the module pins on a specific board revision. GPIO27/INT is an
// MPU output and must be treated as an input, not driven as push-pull GPIO.
pub const VCC_PIN_NAME: &str = "3V3";
pub const GND_PIN_NAME: &str = "GND";
pub const SCL_PIN_NAME: &str = "GPIO20";
pub const SDA_PIN_NAME: &str = "GPIO21";
pub const XDA_PIN_NAME: &str = "GPIO22";
pub const XCL_PIN_NAME: &str = "GPIO23";
pub const AD0_PIN_NAME: &str = "GPIO26";
pub const INT_PIN_NAME: &str = "GPIO27";
