//! Shared pure-logic regression checks for host and embedded test targets.

use crate::RawAccelGyroTemp;

/// Verifies default accelerometer and gyroscope unit conversion.
pub fn raw_values_convert_to_default_accel_and_gyro_units() {
    let raw = RawAccelGyroTemp::new([16_384, -16_384, 8_192], 0, [131, -131, 65]);
    let sample = raw.to_imu_sample();
    assert_eq!(sample.accel_g, [1.0, -1.0, 0.5]);
    assert_eq!(sample.gyro_dps[0], 1.0);
    assert_eq!(sample.gyro_dps[1], -1.0);
    assert!((sample.gyro_dps[2] - (65.0 / 131.0)).abs() < f64::EPSILON);
    assert_eq!(sample.timestamp_s, None);
    assert_eq!(sample.sequence, None);
}

/// Verifies raw temperature conversion to degrees Celsius.
pub fn raw_temperature_converts_to_degrees_celsius() {
    let raw = RawAccelGyroTemp::new([0; 3], 340, [0; 3]);
    assert!((raw.temp_degrees_c() - 37.53).abs() < f64::EPSILON);
}
