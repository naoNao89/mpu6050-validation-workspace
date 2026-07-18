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

/// Concrete HAL pin types for this board profile.
///
/// Pin identity lives only in this module so the rest of the firmware depends on
/// the board map instead of hardcoding GPIO numbers.
#[cfg(target_arch = "riscv32")]
pub type SclPin = esp_hal::peripherals::GPIO0<'static>;
#[cfg(target_arch = "riscv32")]
pub type SdaPin = esp_hal::peripherals::GPIO1<'static>;
#[cfg(target_arch = "riscv32")]
pub type Ad0Pin = esp_hal::peripherals::GPIO5<'static>;
/// MPU INT input pin (`INT_PIN_NAME`).
#[cfg(target_arch = "riscv32")]
pub type IntPin = esp_hal::peripherals::GPIO6<'static>;

/// MPU wiring pins for this board profile.
///
/// GPIO numbers are owned here so firmware uses the board map instead of
/// hardcoding pin literals in `main`.
#[cfg(target_arch = "riscv32")]
pub struct MpuPins {
    pub scl: SclPin,
    pub sda: SdaPin,
    pub ad0: Ad0Pin,
    /// MPU INT input (`INT_PIN_NAME`).
    pub int: IntPin,
}

/// Take the MPU wiring pins defined by this board profile.
///
/// Pin identity (which HAL GPIO is SCL/SDA/AD0/INT) is defined only here.
#[cfg(target_arch = "riscv32")]
#[macro_export]
macro_rules! take_mpu_pins {
    ($peripherals:expr) => {
        $crate::board::MpuPins {
            scl: $peripherals.GPIO0,
            sda: $peripherals.GPIO1,
            ad0: $peripherals.GPIO5,
            int: $peripherals.GPIO6,
        }
    };
}

#[cfg(target_arch = "riscv32")]
pub use take_mpu_pins;
