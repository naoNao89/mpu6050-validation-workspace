#![no_std]

use embedded_hal::i2c::I2c;
mod config;
mod fifo;
mod interrupt;
mod power;
mod raw;
mod registers;
mod self_test;

pub use config::{AccelRange, Address, Dlpf, DlpfReadError, GyroRange, Identity};
pub use fifo::{FIFO_ACCEL_GYRO_FRAME_BYTES, FifoReadDiagnostics};
pub use interrupt::{IntStatus, InterruptEnable};
pub use raw::{
    ACCEL_LSB_PER_G_2G, GYRO_LSB_PER_DPS_250DPS, ImuSample, RawAccelGyroTemp, RawReadOutcome,
    RawRetryPolicy, RawSampleSuspicion, TEMP_LSB_PER_DEG_C, TEMP_OFFSET_DEG_C, raw_to_imu_sample,
};

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
    pub(crate) fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut value = [0_u8];
        self.i2c
            .write_read(self.address.as_u8(), &[register], &mut value)?;
        Ok(value[0])
    }

    pub(crate) fn write_register(&mut self, register: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address.as_u8(), &[register, value])
    }

    pub(crate) fn write_masked(
        &mut self,
        register: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), I2C::Error> {
        let current = self.read_register(register)?;
        self.write_register(register, (current & !mask) | (value & mask))
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::INT_STATUS_FIFO_OFLOW;
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

    struct FifoDiagnosticFakeI2c {
        queue: VecDeque<(u8, Result<Vec<u8>, FakeError>)>,
        fifo_rw_calls: usize,
    }

    impl FifoDiagnosticFakeI2c {
        fn new(queue: Vec<(u8, Vec<u8>)>) -> Self {
            Self::with_results(
                queue
                    .into_iter()
                    .map(|(reg, data)| (reg, Ok(data)))
                    .collect(),
            )
        }

        fn with_results(queue: Vec<(u8, Result<Vec<u8>, FakeError>)>) -> Self {
            Self {
                queue: queue.into(),
                fifo_rw_calls: 0,
            }
        }
    }

    impl ErrorType for FifoDiagnosticFakeI2c {
        type Error = FakeError;
    }

    impl I2c for FifoDiagnosticFakeI2c {
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
            let (register, data) = self.queue.pop_front().expect("missing diagnostic response");
            assert_eq!(write, &[register]);
            let data = data?;
            if register == registers::FIFO_R_W {
                self.fifo_rw_calls += 1;
            }
            read.copy_from_slice(&data[..read.len()]);
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
        const FIFO_TEST_FILL_BYTE: u8 = 0xA5;

        let fake = FifoFakeI2c::new(std::vec![
            FIFO_TEST_FILL_BYTE;
            FIFO_ACCEL_GYRO_FRAME_BYTES
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        let mut buf = [0_u8; FIFO_ACCEL_GYRO_FRAME_BYTES];

        mpu.read_fifo_bytes(&mut buf).unwrap();

        assert_eq!(buf, [FIFO_TEST_FILL_BYTE; FIFO_ACCEL_GYRO_FRAME_BYTES]);
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
    fn fifo_diagnostics_reports_fields_and_helpers() {
        const FIFO_TWO_FRAMES: u16 = (FIFO_ACCEL_GYRO_FRAME_BYTES as u16) * 2;
        const FIFO_ONE_FRAME: u16 = FIFO_ACCEL_GYRO_FRAME_BYTES as u16;

        let fake = FifoDiagnosticFakeI2c::new(std::vec![
            (
                registers::FIFO_COUNTH,
                FIFO_TWO_FRAMES.to_be_bytes().to_vec()
            ),
            (registers::INT_STATUS, std::vec![0]),
            (
                registers::FIFO_R_W,
                std::vec![0x5A; FIFO_ACCEL_GYRO_FRAME_BYTES]
            ),
            (
                registers::FIFO_COUNTH,
                FIFO_ONE_FRAME.to_be_bytes().to_vec()
            ),
        ]);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        let mut buf = [0_u8; FIFO_ACCEL_GYRO_FRAME_BYTES];

        let diagnostics = mpu.read_fifo_bytes_with_diagnostics(&mut buf).unwrap();

        assert_eq!(diagnostics.fifo_count_before_bytes, FIFO_TWO_FRAMES);
        assert_eq!(diagnostics.fifo_bytes_requested, FIFO_ONE_FRAME);
        assert_eq!(diagnostics.fifo_count_after_bytes, FIFO_ONE_FRAME);
        assert!(!diagnostics.fifo_overflow_seen);
        assert!(diagnostics.int_status_read_ok);
        assert!(diagnostics.read_len_frame_aligned);
        assert!(diagnostics.fifo_count_before_frame_aligned);
        assert!(diagnostics.fifo_count_after_frame_aligned);
        assert!(diagnostics.had_requested_bytes_before_read);
        assert!(diagnostics.fifo_count_delta_ok);
        assert!(diagnostics.frame_usable());
        assert!(!diagnostics.should_reset_fifo());
        assert_eq!(mpu.release().fifo_rw_calls, 1);
    }

    #[test]
    fn fifo_diagnostics_helpers_flag_overflow_and_misalignment() {
        let overflow = FifoReadDiagnostics {
            fifo_count_before_bytes: 24,
            fifo_bytes_requested: 12,
            fifo_count_after_bytes: 12,
            fifo_overflow_seen: true,
            int_status_read_ok: true,
            read_len_frame_aligned: true,
            fifo_count_before_frame_aligned: true,
            fifo_count_after_frame_aligned: true,
            had_requested_bytes_before_read: true,
            fifo_count_delta_ok: true,
        };
        assert!(!overflow.frame_usable());
        assert!(overflow.should_reset_fifo());

        let misaligned_refill = FifoReadDiagnostics {
            fifo_count_before_bytes: 13,
            fifo_bytes_requested: 12,
            fifo_count_after_bytes: 12,
            fifo_overflow_seen: false,
            int_status_read_ok: false,
            read_len_frame_aligned: true,
            fifo_count_before_frame_aligned: false,
            fifo_count_after_frame_aligned: true,
            had_requested_bytes_before_read: true,
            fifo_count_delta_ok: false,
        };
        assert!(!misaligned_refill.frame_usable());
        assert!(misaligned_refill.should_reset_fifo());
    }

    fn run_fifo_diagnostics(
        before: u16,
        int_status: Result<u8, FakeError>,
        read_len: usize,
        after: u16,
    ) -> FifoReadDiagnostics {
        let mut queue = std::vec![
            (registers::FIFO_COUNTH, Ok(before.to_be_bytes().to_vec())),
            (
                registers::INT_STATUS,
                int_status.map(|value| std::vec![value])
            ),
        ];
        if read_len > 0 {
            queue.push((registers::FIFO_R_W, Ok(std::vec![0x5A; read_len])));
        }
        queue.push((registers::FIFO_COUNTH, Ok(after.to_be_bytes().to_vec())));
        let fake = FifoDiagnosticFakeI2c::with_results(queue);
        let mut mpu = Mpu6050::new(fake, Address::Ad0Low);
        let mut buf = std::vec![0_u8; read_len];
        mpu.read_fifo_bytes_with_diagnostics(&mut buf).unwrap()
    }

    #[test]
    fn fifo_diagnostics_zero_before_requested_frame_is_not_usable() {
        let diagnostics = run_fifo_diagnostics(0, Ok(0), FIFO_ACCEL_GYRO_FRAME_BYTES, 0);
        assert!(!diagnostics.frame_usable());
        assert!(!diagnostics.had_requested_bytes_before_read);
        assert!(!diagnostics.fifo_count_delta_ok);
    }

    #[test]
    fn fifo_diagnostics_underflow_requested_two_frames_is_not_usable() {
        let diagnostics = run_fifo_diagnostics(12, Ok(0), FIFO_ACCEL_GYRO_FRAME_BYTES * 2, 0);
        assert!(!diagnostics.frame_usable());
        assert!(!diagnostics.had_requested_bytes_before_read);
        assert!(!diagnostics.fifo_count_delta_ok);
    }

    #[test]
    fn fifo_diagnostics_unaligned_read_length_is_not_usable() {
        let diagnostics = run_fifo_diagnostics(24, Ok(0), FIFO_ACCEL_GYRO_FRAME_BYTES / 2, 18);
        assert!(!diagnostics.read_len_frame_aligned);
        assert!(!diagnostics.frame_usable());
    }

    #[test]
    fn fifo_diagnostics_zero_length_is_not_usable() {
        let diagnostics = run_fifo_diagnostics(0, Ok(0), 0, 0);
        assert!(diagnostics.read_len_frame_aligned);
        assert_eq!(diagnostics.fifo_bytes_requested, 0);
        assert!(!diagnostics.frame_usable());
    }

    #[test]
    fn fifo_diagnostics_int_status_error_is_best_effort() {
        let diagnostics =
            run_fifo_diagnostics(12, Err(FakeError::Bus), FIFO_ACCEL_GYRO_FRAME_BYTES, 0);
        assert!(!diagnostics.int_status_read_ok);
        assert!(!diagnostics.fifo_overflow_seen);
    }

    #[test]
    fn fifo_diagnostics_overflow_bit_is_not_usable() {
        let diagnostics = run_fifo_diagnostics(
            12,
            Ok(INT_STATUS_FIFO_OFLOW),
            FIFO_ACCEL_GYRO_FRAME_BYTES,
            0,
        );
        assert!(diagnostics.fifo_overflow_seen);
        assert!(!diagnostics.frame_usable());
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
