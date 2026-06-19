#![no_std]

use embedded_hal::i2c::I2c;
use imu_core::ImuSample;

/// MPU6050 I2C address selected by the AD0 pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Address {
    Ad0Low = 0x68,
    Ad0High = 0x69,
}

impl Address {
    const fn as_u8(self) -> u8 {
        self as u8
    }
}

mod registers {
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const FIFO_EN: u8 = 0x23;
    pub const INT_ENABLE: u8 = 0x38;
    pub const INT_STATUS: u8 = 0x3A;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const USER_CTRL: u8 = 0x6A;
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const FIFO_COUNTH: u8 = 0x72;
    pub const FIFO_R_W: u8 = 0x74;
    pub const WHO_AM_I: u8 = 0x75;
}

const ACCEL_RANGE_MASK: u8 = 0x18;
const GYRO_RANGE_MASK: u8 = 0x18;
const SELF_TEST_MASK: u8 = 0xE0;
const USER_CTRL_FIFO_EN: u8 = 1 << 6;
const USER_CTRL_FIFO_RESET: u8 = 1 << 2;
const INT_ENABLE_DATA_RDY: u8 = 1 << 0;
const INT_ENABLE_FIFO_OFLOW: u8 = 1 << 4;
const INT_STATUS_DATA_RDY: u8 = 1 << 0;
const INT_STATUS_FIFO_OFLOW: u8 = 1 << 4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Identity {
    Mpu6050,
    Mpu6500Compatible,
    Unknown(u8),
}

impl Identity {
    const fn from_who_am_i(id: u8) -> Self {
        decode_identity(id)
    }
}

const fn decode_identity(id: u8) -> Identity {
    match id {
        0x68 => Identity::Mpu6050,
        0x70 => Identity::Mpu6500Compatible,
        other => Identity::Unknown(other),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelRange {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelRange {
    const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroRange {
    Dps250 = 0,
    Dps500 = 1,
    Dps1000 = 2,
    Dps2000 = 3,
}

impl GyroRange {
    const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntStatus {
    bits: u8,
}

impl IntStatus {
    pub const fn data_ready(self) -> bool {
        self.bits & INT_STATUS_DATA_RDY != 0
    }

    pub const fn fifo_overflow(self) -> bool {
        self.bits & INT_STATUS_FIFO_OFLOW != 0
    }
}

const FIFO_SOURCES_ACCEL_XYZ_GYRO_XYZ: u8 = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3);

pub const ACCEL_LSB_PER_G_2G: f64 = 16_384.0;
pub const GYRO_LSB_PER_DPS_250DPS: f64 = 131.0;
pub const TEMP_LSB_PER_DEG_C: f64 = 340.0;
pub const TEMP_OFFSET_DEG_C: f64 = 36.53;

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
    const fn classify(raw: RawAccelGyroTemp) -> Option<Self> {
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
    retries: usize,
    accept_after_retries: bool,
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

pub struct Mpu6050<I2C> {
    i2c: I2C,
    address: Address,
}

impl<I2C> Mpu6050<I2C> {
    pub const fn new(i2c: I2C, address: Address) -> Self {
        Self { i2c, address }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn wake(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::PWR_MGMT_1, 0x00)
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::PWR_MGMT_1, 0x80)
    }

    pub fn who_am_i(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(registers::WHO_AM_I)
    }

    pub fn identity(&mut self) -> Result<Identity, I2C::Error> {
        self.who_am_i().map(Identity::from_who_am_i)
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), I2C::Error> {
        self.write_masked(registers::ACCEL_CONFIG, ACCEL_RANGE_MASK, range.bits())
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), I2C::Error> {
        self.write_masked(registers::GYRO_CONFIG, GYRO_RANGE_MASK, range.bits())
    }

    pub fn set_accel_self_test(&mut self, enabled: bool) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::ACCEL_CONFIG,
            SELF_TEST_MASK,
            if enabled { SELF_TEST_MASK } else { 0 },
        )
    }

    pub fn set_gyro_self_test(&mut self, enabled: bool) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::GYRO_CONFIG,
            SELF_TEST_MASK,
            if enabled { SELF_TEST_MASK } else { 0 },
        )
    }

    pub fn reset_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::USER_CTRL, USER_CTRL_FIFO_RESET)
    }
    pub fn enable_motion_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::FIFO_EN, FIFO_SOURCES_ACCEL_XYZ_GYRO_XYZ)
    }
    pub fn disable_fifo_sources(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::FIFO_EN, 0)
    }
    pub fn enable_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::USER_CTRL, USER_CTRL_FIFO_EN)
    }
    pub fn disable_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::USER_CTRL, 0)
    }
    pub fn fifo_count(&mut self) -> Result<u16, I2C::Error> {
        let mut bytes = [0_u8; 2];
        self.i2c
            .write_read(self.address.as_u8(), &[registers::FIFO_COUNTH], &mut bytes)?;
        Ok(u16::from_be_bytes(bytes))
    }
    pub fn read_fifo_bytes(&mut self, bytes: &mut [u8]) -> Result<(), I2C::Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.i2c
            .write_read(self.address.as_u8(), &[registers::FIFO_R_W], bytes)
    }
    pub fn enable_data_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            INT_ENABLE_DATA_RDY,
            INT_ENABLE_DATA_RDY,
        )
    }

    pub fn enable_fifo_overflow_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            INT_ENABLE_FIFO_OFLOW,
            INT_ENABLE_FIFO_OFLOW,
        )
    }
    pub fn int_status(&mut self) -> Result<IntStatus, I2C::Error> {
        self.read_register(registers::INT_STATUS)
            .map(|bits| IntStatus { bits })
    }

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

    fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut value = [0_u8];
        self.i2c
            .write_read(self.address.as_u8(), &[register], &mut value)?;
        Ok(value[0])
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address.as_u8(), &[register, value])
    }

    fn write_masked(&mut self, register: u8, mask: u8, value: u8) -> Result<(), I2C::Error> {
        let current = self.read_register(register)?;
        self.write_register(register, (current & !mask) | (value & mask))
    }
}

const fn be_i16(msb: u8, lsb: u8) -> i16 {
    i16::from_be_bytes([msb, lsb])
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};
    use std::collections::VecDeque;
    use std::vec::Vec;

    const CLEAN_RAW: RawAccelGyroTemp = RawAccelGyroTemp::new([1, 2, 3], 4, [5, 6, 7]);
    const SUSPICIOUS_RAW: RawAccelGyroTemp = RawAccelGyroTemp::new([i16::MAX, 2, 3], 4, [5, 6, 7]);
    const SUSPICIOUS_RETRY_RAW: RawAccelGyroTemp =
        RawAccelGyroTemp::new([1, 2, 3], 4, [-1, -1, -1]);
    const OBSERVED_POWER_OF_TWO_MINUS_ONE_RAW: RawAccelGyroTemp =
        RawAccelGyroTemp::new([1, 2, 3], 4, [16_383, -1, -1]);
    const OBSERVED_PARTIAL_MINUS_ONE_RAW: RawAccelGyroTemp =
        RawAccelGyroTemp::new([1, 2, 3], 4, [704, 8_191, -1]);

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum FakeError {
        Bus,
    }

    impl embedded_hal::i2c::Error for FakeError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    enum FakeResponse {
        Raw(RawAccelGyroTemp),
        Error(FakeError),
    }

    struct FakeI2c {
        responses: VecDeque<FakeResponse>,
        write_read_count: usize,
    }

    impl FakeI2c {
        fn new(responses: Vec<FakeResponse>) -> Self {
            Self {
                responses: responses.into(),
                write_read_count: 0,
            }
        }
    }

    impl ErrorType for FakeI2c {
        type Error = FakeError;
    }

    impl I2c for FakeI2c {
        fn read(&mut self, _address: SevenBitAddress, _read: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&mut self, _address: SevenBitAddress, _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(
            &mut self,
            _address: SevenBitAddress,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            assert_eq!(write, &[registers::ACCEL_XOUT_H]);
            assert_eq!(read.len(), 14);
            self.write_read_count += 1;
            match self.responses.pop_front().expect("missing fake response") {
                FakeResponse::Raw(raw) => {
                    let values = [
                        raw.accel[0],
                        raw.accel[1],
                        raw.accel[2],
                        raw.temp,
                        raw.gyro[0],
                        raw.gyro[1],
                        raw.gyro[2],
                    ];
                    for (chunk, value) in read.chunks_exact_mut(2).zip(values) {
                        chunk.copy_from_slice(&value.to_be_bytes());
                    }
                    Ok(())
                }
                FakeResponse::Error(error) => Err(error),
            }
        }

        fn transaction(
            &mut self,
            _address: SevenBitAddress,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct FifoFakeI2c {
        fifo_bytes: VecDeque<u8>,
        fifo_rw_calls: usize,
    }

    impl FifoFakeI2c {
        fn new(fifo_bytes: Vec<u8>) -> Self {
            Self {
                fifo_bytes: fifo_bytes.into(),
                fifo_rw_calls: 0,
            }
        }
    }

    impl ErrorType for FifoFakeI2c {
        type Error = FakeError;
    }

    impl I2c for FifoFakeI2c {
        fn read(&mut self, _address: SevenBitAddress, _read: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&mut self, _address: SevenBitAddress, _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(
            &mut self,
            _address: SevenBitAddress,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            assert_eq!(write, &[registers::FIFO_R_W]);
            self.fifo_rw_calls += 1;
            for byte in read {
                *byte = self.fifo_bytes.pop_front().expect("missing FIFO byte");
            }
            Ok(())
        }

        fn transaction(
            &mut self,
            _address: SevenBitAddress,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn address_values_match_ad0_pin_state() {
        assert_eq!(Address::Ad0Low.as_u8(), 0x68);
        assert_eq!(Address::Ad0High.as_u8(), 0x69);
    }

    #[test]
    fn fifo_burst_read_uses_single_transaction() {
        const FIFO_TEST_BYTES: usize = 12;
        const FIFO_TEST_FILL_BYTE: u8 = 0xA5;

        let fake = FifoFakeI2c::new(std::vec![FIFO_TEST_FILL_BYTE; FIFO_TEST_BYTES]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        let mut buf = [0_u8; FIFO_TEST_BYTES];

        mpu.read_fifo_bytes(&mut buf).unwrap();

        assert_eq!(buf, [FIFO_TEST_FILL_BYTE; FIFO_TEST_BYTES]);
        assert_eq!(mpu.release().fifo_rw_calls, 1);
    }

    #[test]
    fn fifo_zero_length_read_uses_no_transaction() {
        let fake = FifoFakeI2c::new(std::vec![]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        let mut buf = [];

        mpu.read_fifo_bytes(&mut buf).unwrap();

        assert_eq!(mpu.release().fifo_rw_calls, 0);
    }

    #[test]
    fn raw_values_convert_to_default_accel_and_gyro_units() {
        let raw = RawAccelGyroTemp::new([16_384, -16_384, 8_192], 0, [131, -131, 65]);
        let sample = raw.to_imu_sample();
        assert_eq!(sample.accel_g, [1.0, -1.0, 0.5]);
        assert_eq!(sample.gyro_dps[0], 1.0);
        assert_eq!(sample.gyro_dps[1], -1.0);
        assert!((sample.gyro_dps[2] - (65.0 / 131.0)).abs() < f64::EPSILON);
        assert_eq!(sample.timestamp_s, None);
        assert_eq!(sample.sequence, None);
    }

    #[test]
    fn raw_temperature_converts_to_degrees_celsius() {
        let raw = RawAccelGyroTemp::new([0; 3], 340, [0; 3]);
        assert!((raw.temp_degrees_c() - 37.53).abs() < f64::EPSILON);
    }

    #[test]
    fn regression_raw_sample_flags_gyro_all_minus_one_as_suspicious() {
        let raw = RawAccelGyroTemp::new([1, 2, 3], 25, [-1, -1, -1]);
        assert!(raw.is_suspicious());
    }

    #[test]
    fn regression_raw_sample_flags_observed_gyro_power_of_two_minus_one_as_suspicious() {
        let raw = RawAccelGyroTemp::new([-6428, -10508, -9212], 4096, [16_383, -1, -1]);
        assert!(raw.is_suspicious());
        assert_eq!(
            RawSampleSuspicion::classify(raw),
            Some(RawSampleSuspicion::GyroPowerOfTwoMinusOne)
        );
    }

    #[test]
    fn regression_raw_sample_flags_observed_partial_minus_one_gyro_as_suspicious() {
        let raw = RawAccelGyroTemp::new([-6368, -10576, -9228], 4144, [704, 8191, -1]);
        assert!(raw.is_suspicious());
        assert_eq!(
            RawSampleSuspicion::classify(raw),
            Some(RawSampleSuspicion::GyroPowerOfTwoMinusOne)
        );
    }

    #[test]
    fn regression_raw_sample_flags_partial_minus_one_gyro_without_power_sentinel() {
        let raw = RawAccelGyroTemp::new([1, 2, 3], 4, [700, -1, -320]);
        assert!(raw.is_suspicious());
        assert_eq!(
            RawSampleSuspicion::classify(raw),
            Some(RawSampleSuspicion::GyroPartialMinusOne)
        );
    }

    #[test]
    fn regression_raw_sample_flags_i16_sentinels_as_suspicious() {
        let raw = RawAccelGyroTemp::new([i16::MAX, 2, 3], 25, [4, i16::MIN, 6]);
        assert!(raw.is_suspicious());
    }

    #[test]
    fn regression_raw_sample_accepts_nominal_values() {
        let raw = RawAccelGyroTemp::new([-6500, -9900, -9600], 3700, [720, 190, -320]);
        assert!(!raw.is_suspicious());
    }

    #[test]
    fn primitive_read_performs_one_transaction() {
        let fake = FakeI2c::new(std::vec![FakeResponse::Raw(CLEAN_RAW)]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(mpu.read_raw_accel_gyro_temp(), Ok(CLEAN_RAW));
        assert_eq!(mpu.release().write_read_count, 1);
    }

    #[test]
    fn clean_checked_read_performs_one_transaction_and_returns_clean() {
        let fake = FakeI2c::new(std::vec![FakeResponse::Raw(CLEAN_RAW)]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_checked(),
            Ok(RawReadOutcome::Clean { raw: CLEAN_RAW })
        );
        assert_eq!(mpu.release().write_read_count, 1);
    }

    #[test]
    fn suspicious_checked_read_with_zero_retries_rejects_after_one_transaction() {
        let fake = FakeI2c::new(std::vec![FakeResponse::Raw(SUSPICIOUS_RAW)]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_checked(),
            Ok(RawReadOutcome::RejectedSuspicious {
                raw: SUSPICIOUS_RAW,
                suspicion: RawSampleSuspicion::AccelSentinel,
                retries: 0,
            })
        );
        assert_eq!(mpu.release().write_read_count, 1);
    }

    #[test]
    fn suspicious_then_clean_returns_recovered_after_two_transactions() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(SUSPICIOUS_RAW),
            FakeResponse::Raw(CLEAN_RAW),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)),
            Ok(RawReadOutcome::Recovered {
                raw: CLEAN_RAW,
                first_suspicion: RawSampleSuspicion::AccelSentinel,
                retries: 1,
            })
        );
        assert_eq!(mpu.release().write_read_count, 2);
    }

    #[test]
    fn observed_power_sentinel_then_clean_returns_recovered_after_two_transactions() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(OBSERVED_POWER_OF_TWO_MINUS_ONE_RAW),
            FakeResponse::Raw(CLEAN_RAW),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)),
            Ok(RawReadOutcome::Recovered {
                raw: CLEAN_RAW,
                first_suspicion: RawSampleSuspicion::GyroPowerOfTwoMinusOne,
                retries: 1,
            })
        );
        assert_eq!(mpu.release().write_read_count, 2);
    }

    #[test]
    fn suspicious_then_suspicious_returns_rejected_suspicious() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(SUSPICIOUS_RAW),
            FakeResponse::Raw(SUSPICIOUS_RETRY_RAW),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)),
            Ok(RawReadOutcome::RejectedSuspicious {
                raw: SUSPICIOUS_RETRY_RAW,
                suspicion: RawSampleSuspicion::GyroAllMinusOne,
                retries: 1,
            })
        );
    }

    #[test]
    fn observed_outlier_then_outlier_returns_rejected_suspicious() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(OBSERVED_POWER_OF_TWO_MINUS_ONE_RAW),
            FakeResponse::Raw(OBSERVED_PARTIAL_MINUS_ONE_RAW),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)),
            Ok(RawReadOutcome::RejectedSuspicious {
                raw: OBSERVED_PARTIAL_MINUS_ONE_RAW,
                suspicion: RawSampleSuspicion::GyroPowerOfTwoMinusOne,
                retries: 1,
            })
        );
    }

    #[test]
    fn suspicious_then_bus_error_returns_retry_error_preserving_first_raw_and_suspicion() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(SUSPICIOUS_RAW),
            FakeResponse::Error(FakeError::Bus),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)),
            Ok(RawReadOutcome::RetryError {
                first_raw: SUSPICIOUS_RAW,
                first_suspicion: RawSampleSuspicion::AccelSentinel,
                retries: 1,
                error: FakeError::Bus,
            })
        );
    }

    #[test]
    fn accept_policy_returns_accepted_suspicious_after_retries_exhausted() {
        let fake = FakeI2c::new(std::vec![
            FakeResponse::Raw(SUSPICIOUS_RAW),
            FakeResponse::Raw(SUSPICIOUS_RETRY_RAW),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        assert_eq!(
            mpu.read_raw_with_retry(RawRetryPolicy::accept_after_retries(1)),
            Ok(RawReadOutcome::AcceptedSuspicious {
                raw: SUSPICIOUS_RETRY_RAW,
                suspicion: RawSampleSuspicion::GyroAllMinusOne,
                retries: 1,
            })
        );
    }
}
