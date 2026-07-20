use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

pub const ACCEL_LSB_PER_G_2G: f64 = 16_384.0;
pub const GYRO_LSB_PER_DPS_250DPS: f64 = 131.0;
pub const TEMP_LSB_PER_DEG_C: f64 = 340.0;
pub const TEMP_OFFSET_DEG_C: f64 = 36.53;

#[derive(Clone, Debug)]
pub struct ImuSample {
    pub accel_g: [f64; 3],
    pub gyro_dps: [f64; 3],
    pub timestamp_s: Option<f64>,
    pub sequence: Option<u64>,
}

impl ImuSample {
    pub fn from_g_dps(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self {
            accel_g,
            gyro_dps,
            timestamp_s: None,
            sequence: None,
        }
    }

    pub fn from_si(accel_mps2: [f64; 3], gyro_radps: [f64; 3]) -> Self {
        const STANDARD_GRAVITY_MPS2: f64 = 9.80665;
        Self::from_g_dps(
            accel_mps2.map(|v| v / STANDARD_GRAVITY_MPS2),
            gyro_radps.map(f64::to_degrees),
        )
    }

    pub fn new(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self::from_g_dps(accel_g, gyro_dps)
    }

    /// Acceleration magnitude \(\|a\| = \sqrt{a_x^2 + a_y^2 + a_z^2}\) in g.
    ///
    /// While the board is stationary, this is typically near **1 g** (gravity).
    /// Use it as a lightweight usability check; it is not clone/authenticity
    /// detection and does not replace multi-sample host analysis.
    pub fn accel_magnitude_g(&self) -> f64 {
        let [x, y, z] = self.accel_g;
        libm::sqrt(x * x + y * y + z * z)
    }

    /// Gyroscope magnitude \(\|\omega\|\) in °/s.
    ///
    /// While stationary, expect this near zero aside from bias and noise.
    pub fn gyro_magnitude_dps(&self) -> f64 {
        let [x, y, z] = self.gyro_dps;
        libm::sqrt(x * x + y * y + z * z)
    }
}

/// Raw accel/temp/gyro register block read from ACCEL_XOUT_H.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawAccelGyroTemp {
    pub accel: [i16; 3],
    pub temp: i16,
    pub gyro: [i16; 3],
}

impl RawAccelGyroTemp {
    pub const fn new(accel: [i16; 3], temp: i16, gyro: [i16; 3]) -> Self {
        Self { accel, temp, gyro }
    }

    pub fn to_imu_sample(self) -> ImuSample {
        raw_to_imu_sample(self)
    }

    pub fn temp_degrees_c(self) -> f64 {
        self.temp as f64 / TEMP_LSB_PER_DEG_C + TEMP_OFFSET_DEG_C
    }

    pub const fn is_suspicious(self) -> bool {
        RawSampleSuspicion::classify(self).is_some()
    }
}

/// Reason a raw accel/temp/gyro sample was classified as suspicious.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RawSampleSuspicion {
    AccelSentinel,
    GyroSentinel,
    TempSentinel,
    AccelAllMinusOne,
    GyroAllMinusOne,
    GyroPartialMinusOne,
    GyroPowerOfTwoMinusOne,
}

impl RawSampleSuspicion {
    pub(crate) const fn classify(raw: RawAccelGyroTemp) -> Option<Self> {
        if contains_i16_sentinel(raw.accel) {
            Some(Self::AccelSentinel)
        } else if contains_i16_sentinel(raw.gyro) {
            Some(Self::GyroSentinel)
        } else if raw.temp == i16::MIN || raw.temp == i16::MAX {
            Some(Self::TempSentinel)
        } else if all_minus_one(raw.accel) {
            Some(Self::AccelAllMinusOne)
        } else if all_minus_one(raw.gyro) {
            Some(Self::GyroAllMinusOne)
        } else if contains_power_of_two_minus_one_sentinel(raw.gyro) {
            Some(Self::GyroPowerOfTwoMinusOne)
        } else if partial_minus_one(raw.gyro) {
            Some(Self::GyroPartialMinusOne)
        } else {
            None
        }
    }
}

/// Policy controlling how many suspicious raw reads are retried and whether the
/// final suspicious sample is rejected or accepted when retries are exhausted.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawRetryPolicy {
    pub(crate) retries: usize,
    pub(crate) accept_after_retries: bool,
}

impl RawRetryPolicy {
    pub const fn reject_after_retries(retries: usize) -> Self {
        Self {
            retries,
            accept_after_retries: false,
        }
    }

    pub const fn accept_after_retries(retries: usize) -> Self {
        Self {
            retries,
            accept_after_retries: true,
        }
    }
}

/// Result of a checked raw read, including retry/recovery details.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RawReadOutcome<E> {
    Clean {
        raw: RawAccelGyroTemp,
    },
    Recovered {
        raw: RawAccelGyroTemp,
        first_suspicion: RawSampleSuspicion,
        retries: usize,
    },
    RejectedSuspicious {
        raw: RawAccelGyroTemp,
        suspicion: RawSampleSuspicion,
        retries: usize,
    },
    AcceptedSuspicious {
        raw: RawAccelGyroTemp,
        suspicion: RawSampleSuspicion,
        retries: usize,
    },
    RetryError {
        first_raw: RawAccelGyroTemp,
        first_suspicion: RawSampleSuspicion,
        retries: usize,
        error: E,
    },
}

const fn contains_i16_sentinel(values: [i16; 3]) -> bool {
    values[0] == i16::MIN
        || values[0] == i16::MAX
        || values[1] == i16::MIN
        || values[1] == i16::MAX
        || values[2] == i16::MIN
        || values[2] == i16::MAX
}

const fn all_minus_one(values: [i16; 3]) -> bool {
    values[0] == -1 && values[1] == -1 && values[2] == -1
}

const fn partial_minus_one(values: [i16; 3]) -> bool {
    (values[0] == -1 || values[1] == -1 || values[2] == -1) && !all_minus_one(values)
}

const fn contains_power_of_two_minus_one_sentinel(values: [i16; 3]) -> bool {
    is_power_of_two_minus_one_sentinel(values[0])
        || is_power_of_two_minus_one_sentinel(values[1])
        || is_power_of_two_minus_one_sentinel(values[2])
}

const fn is_power_of_two_minus_one_sentinel(value: i16) -> bool {
    matches!(value, 8191 | 16383)
}

pub fn raw_to_imu_sample(raw: RawAccelGyroTemp) -> ImuSample {
    ImuSample::from_g_dps(
        raw.accel.map(|v| v as f64 / ACCEL_LSB_PER_G_2G),
        raw.gyro.map(|v| v as f64 / GYRO_LSB_PER_DPS_250DPS),
    )
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    /// Reads the 14-byte accel/temp/gyro block starting at `ACCEL_XOUT_H`.
    pub fn read_raw_accel_gyro_temp(&mut self) -> Result<RawAccelGyroTemp, I2C::Error> {
        let mut bytes = [0_u8; 14];
        self.i2c
            .write_read(self.address.as_u8(), &[registers::ACCEL_XOUT_H], &mut bytes)?;
        Ok(RawAccelGyroTemp {
            accel: [
                be_i16(bytes[0], bytes[1]),
                be_i16(bytes[2], bytes[3]),
                be_i16(bytes[4], bytes[5]),
            ],
            temp: be_i16(bytes[6], bytes[7]),
            gyro: [
                be_i16(bytes[8], bytes[9]),
                be_i16(bytes[10], bytes[11]),
                be_i16(bytes[12], bytes[13]),
            ],
        })
    }

    pub fn read_raw_checked(&mut self) -> Result<RawReadOutcome<I2C::Error>, I2C::Error> {
        self.read_raw_with_retry(RawRetryPolicy::reject_after_retries(0_usize))
    }

    pub fn read_raw_with_retry(
        &mut self,
        policy: RawRetryPolicy,
    ) -> Result<RawReadOutcome<I2C::Error>, I2C::Error> {
        let first_raw = self.read_raw_accel_gyro_temp()?;
        let Some(first_suspicion) = RawSampleSuspicion::classify(first_raw) else {
            return Ok(RawReadOutcome::Clean { raw: first_raw });
        };

        let mut retries = 0_usize;
        let mut raw = first_raw;
        let mut suspicion = first_suspicion;

        while retries < policy.retries {
            retries += 1;
            match self.read_raw_accel_gyro_temp() {
                Ok(retry_raw) => {
                    raw = retry_raw;
                    match RawSampleSuspicion::classify(retry_raw) {
                        Some(retry_suspicion) => suspicion = retry_suspicion,
                        None => {
                            return Ok(RawReadOutcome::Recovered {
                                raw: retry_raw,
                                first_suspicion,
                                retries,
                            });
                        }
                    }
                }
                Err(error) => {
                    return Ok(RawReadOutcome::RetryError {
                        first_raw,
                        first_suspicion,
                        retries,
                        error,
                    });
                }
            }
        }

        if policy.accept_after_retries {
            Ok(RawReadOutcome::AcceptedSuspicious {
                raw,
                suspicion,
                retries,
            })
        } else {
            Ok(RawReadOutcome::RejectedSuspicious {
                raw,
                suspicion,
                retries,
            })
        }
    }
}

const fn be_i16(msb: u8, lsb: u8) -> i16 {
    i16::from_be_bytes([msb, lsb])
}
