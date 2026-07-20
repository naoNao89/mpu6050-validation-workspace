//! Lightweight **usability** check while the board is still.
//!
//! This is not clone/authenticity detection. It only asks: does acceleration
//! magnitude look like gravity (~1 g) and is the gyro quiet?
//!
//! On hardware: wake the sensor, leave it flat and still, convert samples with
//! [`raw_to_imu_sample`](mpu6050_driver::raw_to_imu_sample), then apply the same
//! checks. For multi-sample pass/fail reports, use the workspace `imu-tool`.

use mpu6050_driver::{ImuSample, RawAccelGyroTemp, raw_to_imu_sample};

/// Loose single-sample gates for a quick smoke check (not metrology).
const ACCEL_MAG_MIN_G: f64 = 0.85;
const ACCEL_MAG_MAX_G: f64 = 1.15;
const GYRO_MAG_MAX_DPS: f64 = 25.0;

fn looks_usable_while_still(sample: &ImuSample) -> bool {
    let a = sample.accel_magnitude_g();
    let g = sample.gyro_magnitude_dps();
    (ACCEL_MAG_MIN_G..=ACCEL_MAG_MAX_G).contains(&a) && g <= GYRO_MAG_MAX_DPS
}

fn report(label: &str, sample: &ImuSample) {
    let a = sample.accel_magnitude_g();
    let w = sample.gyro_magnitude_dps();
    let ok = looks_usable_while_still(sample);
    println!(
        "{label}: |a|={a:.3} g  |w|={w:.3} dps  => {}",
        if ok {
            "usable_smoke_pass"
        } else {
            "usable_smoke_fail"
        }
    );
}

fn main() {
    // Still on a table: ~1 g on Z, small gyro bias.
    let still = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 16_384], 0, [20, -15, 10]));
    // Broken / wrong scale / not still: |a| far from 1 g or large rotation.
    let bad_accel = ImuSample::from_g_dps([0.0, 0.0, 0.1], [0.0, 0.0, 0.0]);
    let spinning = ImuSample::from_g_dps([0.0, 0.0, 1.0], [80.0, 0.0, 0.0]);

    report("still", &still);
    report("bad_accel", &bad_accel);
    report("spinning", &spinning);

    assert!(
        looks_usable_while_still(&still),
        "nominal still sample must pass smoke check"
    );
    assert!(
        !looks_usable_while_still(&bad_accel),
        "|a|<<1 g must fail smoke check"
    );
    assert!(
        !looks_usable_while_still(&spinning),
        "large gyro while 'still' must fail smoke check"
    );
}
