//! Characterization / regression contract for raw → physical conversion.
//!
//! Locks conversion behavior before representation changes (e.g. f64 → f32).
//! Tolerances are tied to sensor quantization (one raw LSB), not arbitrary epsilons.

use mpu6050_driver::{
    ACCEL_LSB_PER_G_2G, GYRO_LSB_PER_DPS_250DPS, ImuSample, RawAccelGyroTemp, TEMP_LSB_PER_DEG_C,
    TEMP_OFFSET_DEG_C, raw_to_imu_sample,
};

/// One accel LSB at ±2 g full scale, in g.
fn accel_lsb_g() -> f64 {
    1.0 / ACCEL_LSB_PER_G_2G
}

/// One gyro LSB at ±250 °/s full scale, in °/s.
fn gyro_lsb_dps() -> f64 {
    1.0 / GYRO_LSB_PER_DPS_250DPS
}

/// Allowed conversion error vs f64 reference: well under one sensor LSB.
fn accel_tol_g() -> f64 {
    accel_lsb_g() / 8.0
}

fn gyro_tol_dps() -> f64 {
    gyro_lsb_dps() / 8.0
}

const BOUNDARY_RAW: [i16; 7] = [i16::MIN, -16_384, -1, 0, 1, 16_384, i16::MAX];

fn ref_accel_g(raw: i16) -> f64 {
    raw as f64 / ACCEL_LSB_PER_G_2G
}

fn ref_gyro_dps(raw: i16) -> f64 {
    raw as f64 / GYRO_LSB_PER_DPS_250DPS
}

#[test]
fn boundary_raw_accel_converts_within_lsb_fraction() {
    for &raw in &BOUNDARY_RAW {
        let sample = raw_to_imu_sample(RawAccelGyroTemp::new([raw, 0, 0], 0, [0, 0, 0]));
        let got = sample.accel_g[0];
        let want = ref_accel_g(raw);
        assert!(
            (got - want).abs() <= accel_tol_g(),
            "accel raw={raw}: got={got} want={want} tol={}",
            accel_tol_g()
        );
        assert!(got.is_finite());
    }
}

#[test]
fn boundary_raw_gyro_converts_within_lsb_fraction() {
    for &raw in &BOUNDARY_RAW {
        let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 0], 0, [raw, 0, 0]));
        let got = sample.gyro_dps[0];
        let want = ref_gyro_dps(raw);
        assert!(
            (got - want).abs() <= gyro_tol_dps(),
            "gyro raw={raw}: got={got} want={want} tol={}",
            gyro_tol_dps()
        );
        assert!(got.is_finite());
    }
}

#[test]
fn full_i16_domain_accel_matches_f64_reference() {
    let tol = accel_tol_g();
    for raw in i16::MIN..=i16::MAX {
        let sample = raw_to_imu_sample(RawAccelGyroTemp::new([raw, 0, 0], 0, [0, 0, 0]));
        let got = sample.accel_g[0];
        let want = ref_accel_g(raw);
        assert!(
            (got - want).abs() <= tol,
            "accel raw={raw}: got={got} want={want} tol={tol}"
        );
    }
}

#[test]
fn full_i16_domain_gyro_matches_f64_reference() {
    let tol = gyro_tol_dps();
    for raw in i16::MIN..=i16::MAX {
        let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 0], 0, [raw, 0, 0]));
        let got = sample.gyro_dps[0];
        let want = ref_gyro_dps(raw);
        assert!(
            (got - want).abs() <= tol,
            "gyro raw={raw}: got={got} want={want} tol={tol}"
        );
    }
}

#[test]
fn nominal_1g_z_and_1dps_z_magnitudes() {
    let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]));
    assert!((sample.accel_magnitude_g() - 1.0).abs() <= accel_tol_g());
    assert!((sample.gyro_magnitude_dps() - 1.0).abs() <= gyro_tol_dps());
    assert!(sample.accel_g.iter().all(|v| v.is_finite()));
    assert!(sample.gyro_dps.iter().all(|v| v.is_finite()));
}

#[test]
fn raw_to_imu_sample_leaves_stream_stamps_unset() {
    // Characterization of 0.1.x: driver conversion does not invent stream metadata.
    let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 0]));
    assert_eq!(sample.timestamp_s, None);
    assert_eq!(sample.sequence, None);
}

#[test]
fn temperature_conversion_stays_finite_at_boundaries() {
    for &raw in &BOUNDARY_RAW {
        let t = RawAccelGyroTemp::new([0, 0, 0], raw, [0, 0, 0]).temp_degrees_c();
        assert!(t.is_finite(), "temp raw={raw} -> {t}");
        let want = raw as f64 / TEMP_LSB_PER_DEG_C + TEMP_OFFSET_DEG_C;
        assert!((t - want).abs() < 1e-12);
    }
}

#[test]
fn magnitude_helpers_match_manual_norm() {
    let sample = ImuSample::from_g_dps([3.0, 4.0, 0.0], [0.0, 5.0, 12.0]);
    assert!((sample.accel_magnitude_g() - 5.0).abs() < 1e-12);
    assert!((sample.gyro_magnitude_dps() - 13.0).abs() < 1e-12);
}
