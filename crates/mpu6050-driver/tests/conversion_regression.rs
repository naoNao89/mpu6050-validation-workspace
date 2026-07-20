//! Characterization / regression contract for raw → physical conversion.
//!
//! Compares driver conversion against an **independent** f64 reference that does
//! not read production scale constants (avoids tautological checks).
//! Tolerances are tied to sensor quantization (one raw LSB).

use mpu6050_driver::{ImuSample, RawAccelGyroTemp, raw_to_imu_sample};

/// Independent ±2 g scale reference (datasheet LSB), not `ACCEL_LSB_PER_G_2G`.
const ACCEL_SCALE_REF: f64 = 16_384.0;
/// Independent ±250 °/s scale reference, not `GYRO_LSB_PER_DPS_250DPS`.
const GYRO_SCALE_REF: f64 = 131.0;
/// Independent temp scale/offset references for boundary finiteness checks.
const TEMP_SCALE_REF: f64 = 340.0;
const TEMP_OFFSET_REF: f64 = 36.53;

/// One accel LSB at ±2 g full scale, in g.
fn accel_lsb_g() -> f64 {
    1.0 / ACCEL_SCALE_REF
}

/// One gyro LSB at ±250 °/s full scale, in °/s.
fn gyro_lsb_dps() -> f64 {
    1.0 / GYRO_SCALE_REF
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
    raw as f64 / ACCEL_SCALE_REF
}

fn ref_gyro_dps(raw: i16) -> f64 {
    raw as f64 / GYRO_SCALE_REF
}

#[test]
fn conversion_preserves_expected_boundary_values() {
    for &raw in &BOUNDARY_RAW {
        let accel = raw_to_imu_sample(RawAccelGyroTemp::new([raw, 0, 0], 0, [0, 0, 0]));
        let got_a = f64::from(accel.accel_g[0]);
        let want_a = ref_accel_g(raw);
        assert!(
            (got_a - want_a).abs() <= accel_tol_g(),
            "accel raw={raw}: got={got_a} want={want_a} tol={}",
            accel_tol_g()
        );
        assert!(got_a.is_finite());

        let gyro = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 0], 0, [raw, 0, 0]));
        let got_g = f64::from(gyro.gyro_dps[0]);
        let want_g = ref_gyro_dps(raw);
        assert!(
            (got_g - want_g).abs() <= gyro_tol_dps(),
            "gyro raw={raw}: got={got_g} want={want_g} tol={}",
            gyro_tol_dps()
        );
        assert!(got_g.is_finite());
    }
}

#[test]
fn conversion_matches_reference_across_full_i16_domain() {
    let a_tol = accel_tol_g();
    let g_tol = gyro_tol_dps();
    for raw in i16::MIN..=i16::MAX {
        let accel = raw_to_imu_sample(RawAccelGyroTemp::new([raw, 0, 0], 0, [0, 0, 0]));
        let got_a = f64::from(accel.accel_g[0]);
        let want_a = ref_accel_g(raw);
        assert!(
            (got_a - want_a).abs() <= a_tol,
            "accel raw={raw}: got={got_a} want={want_a} tol={a_tol}"
        );

        let gyro = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 0], 0, [raw, 0, 0]));
        let got_g = f64::from(gyro.gyro_dps[0]);
        let want_g = ref_gyro_dps(raw);
        assert!(
            (got_g - want_g).abs() <= g_tol,
            "gyro raw={raw}: got={got_g} want={want_g} tol={g_tol}"
        );
    }
}

#[test]
fn nominal_1g_z_and_1dps_z_magnitudes() {
    let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]));
    assert!((f64::from(sample.accel_magnitude_g()) - 1.0).abs() <= accel_tol_g());
    assert!((f64::from(sample.gyro_magnitude_dps()) - 1.0).abs() <= gyro_tol_dps());
    assert!(sample.accel_g.iter().all(|v| v.is_finite()));
    assert!(sample.gyro_dps.iter().all(|v| v.is_finite()));
}

#[test]
fn imu_sample_has_no_stream_stamp_fields() {
    // Structural contract for 0.2: physical fields only (2 × [f32; 3]).
    let sample = raw_to_imu_sample(RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 0]));
    let _ = sample.accel_g;
    let _ = sample.gyro_dps;
    assert_eq!(core::mem::size_of_val(&sample), 24);
}

#[test]
fn temperature_conversion_stays_finite_at_boundaries() {
    for &raw in &BOUNDARY_RAW {
        let t = f64::from(RawAccelGyroTemp::new([0, 0, 0], raw, [0, 0, 0]).temp_degrees_c());
        assert!(t.is_finite(), "temp raw={raw} -> {t}");
        let want = raw as f64 / TEMP_SCALE_REF + TEMP_OFFSET_REF;
        assert!(
            (t - want).abs() <= accel_lsb_g(), // loose vs temp LSB; finiteness is primary
            "temp raw={raw}: got={t} want={want}"
        );
    }
}

#[test]
fn magnitude_helpers_match_manual_norm() {
    let sample = ImuSample::from_g_dps([3.0, 4.0, 0.0], [0.0, 5.0, 12.0]);
    assert!((sample.accel_magnitude_g() - 5.0).abs() < 1e-6);
    assert!((sample.gyro_magnitude_dps() - 13.0).abs() < 1e-6);
}
