#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();
use esp_hal as _;

#[embedded_test::tests]
mod tests {
    #[test]
    fn raw_values_convert_to_default_accel_and_gyro_units() {
        mpu6050_driver::test_support::raw_values_convert_to_default_accel_and_gyro_units();
    }

    #[test]
    fn raw_temperature_converts_to_degrees_celsius() {
        mpu6050_driver::test_support::raw_temperature_converts_to_degrees_celsius();
    }
}
