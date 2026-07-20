//! Board-owned stream metadata around driver physical samples.
//!
//! `mpu6050-driver::ImuSample` carries engineering units only. Timestamp and
//! sequence are acquisition/transport concerns and live here.

use mpu6050_driver::{ImuSample, RawAccelGyroTemp, raw_to_imu_sample};

/// One motion sample with board-assigned stream identity.
#[derive(Clone, Copy, Debug)]
pub struct StampedSample {
    pub sample: ImuSample,
    pub timestamp_us: u64,
    pub sequence: u64,
}

impl StampedSample {
    /// Convert a raw register block and attach board stream stamps.
    pub fn from_raw(raw: RawAccelGyroTemp, timestamp_us: u64, sequence: u64) -> Self {
        Self {
            sample: raw_to_imu_sample(raw),
            timestamp_us,
            sequence,
        }
    }

    pub fn accel_magnitude_g(self) -> f32 {
        self.sample.accel_magnitude_g()
    }

    pub fn gyro_magnitude_dps(self) -> f32 {
        self.sample.gyro_magnitude_dps()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_are_board_owned_not_driver_fields() {
        let raw = RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]);
        let stamped = StampedSample::from_raw(raw, 1_234_567, 42);
        assert_eq!(stamped.timestamp_us, 1_234_567);
        assert_eq!(stamped.sequence, 42);
        assert!((stamped.accel_magnitude_g() - 1.0).abs() < 1e-5);
        assert!((stamped.gyro_magnitude_dps() - 1.0).abs() < 1e-5);
        // Physical sample has no stamp fields — only accel/gyro arrays.
        assert_eq!(core::mem::size_of_val(&stamped.sample), 24);
    }
}
