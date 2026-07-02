//! ESP32-C3 SuperMini-class board to GY-521/MPU6050 wiring map.

/// Reference dev-board wiring used by this bring-up firmware.
///
/// The repo's board-under-test is an ESP32-C3 SuperMini-class board connected to
/// a GY-521/MPU6050 module. Keep these constants aligned with the README so the
/// firmware is explicitly a board sample that exercises this MPU6050 driver
/// stack, rather than an anonymous ESP32-C3 snippet.
pub const BOARD_NAME: &str = "ESP32-C3 SuperMini-class dev board";
pub const I2C_BUS_NAME: &str = "I2C0";
pub const I2C_FREQUENCY_KHZ: u32 = 100;

// Human wiring map:
// ESP32-C3 3V3   -> MPU6050 VCC
// ESP32-C3 GND   -> MPU6050 GND
// ESP32-C3 GPIO0 -> MPU6050 SCL
// ESP32-C3 GPIO1 -> MPU6050 SDA
// ESP32-C3 GPIO3 -> MPU6050 XDA
// ESP32-C3 GPIO4 -> MPU6050 XCL
// ESP32-C3 GPIO5 -> MPU6050 AD0
// ESP32-C3 GPIO6 -> MPU6050 INT
pub const VCC_PIN_NAME: &str = "3V3";
pub const GND_PIN_NAME: &str = "GND";
pub const SCL_PIN_NAME: &str = "GPIO0";
pub const SDA_PIN_NAME: &str = "GPIO1";
pub const XDA_PIN_NAME: &str = "GPIO3";
pub const XCL_PIN_NAME: &str = "GPIO4";
pub const AD0_PIN_NAME: &str = "GPIO5";
pub const INT_PIN_NAME: &str = "GPIO6";
