//! Convert a synthetic raw register block into engineering units.

use mpu6050_driver::{RawAccelGyroTemp, raw_to_imu_sample};

fn main() {
    // Nominal 1 g on Z at ±2 g scale, ~1 °/s on Z at ±250 °/s scale.
    let raw = RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]);
    let sample = raw_to_imu_sample(raw);

    println!("accel_g = {:?}", sample.accel_g);
    println!("gyro_dps = {:?}", sample.gyro_dps);
    println!("temp_c = {:.2}", raw.temp_degrees_c());
}
