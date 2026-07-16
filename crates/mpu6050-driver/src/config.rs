use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

/// MPU6050 I2C address selected by the AD0 pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Address {
    Ad0Low = 0x68,
    Ad0High = 0x69,
}

impl Address {
    pub(crate) const fn as_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Identity {
    Mpu6050,
    Mpu6500Compatible,
    Unknown(u8),
}

impl Identity {
    pub(crate) const fn from_who_am_i(id: u8) -> Self {
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

/// Digital low-pass filter configuration from the CONFIG register.
///
/// `Cfg2` selects approximately 94 Hz accelerometer bandwidth and approximately
/// 98 Hz gyroscope bandwidth; it is not named for a single bandwidth because the
/// two sensors differ.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Dlpf {
    Cfg0 = 0,
    Cfg1 = 1,
    Cfg2 = 2,
    Cfg3 = 3,
    Cfg4 = 4,
    Cfg5 = 5,
    Cfg6 = 6,
}

impl Dlpf {
    const fn bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0 => Some(Self::Cfg0),
            1 => Some(Self::Cfg1),
            2 => Some(Self::Cfg2),
            3 => Some(Self::Cfg3),
            4 => Some(Self::Cfg4),
            5 => Some(Self::Cfg5),
            6 => Some(Self::Cfg6),
            _ => None,
        }
    }

    /// Returns the configured sample rate in Hz for `divider`.
    ///
    /// The formula is `base_rate / (divider as f32 + 1.0)`, where the base rate is
    /// 8000.0 Hz for `Cfg0` and 1000.0 Hz for `Cfg1` through `Cfg6`. This returns
    /// an approximate floating-point result and does not truncate to an integer.
    /// The 8 kHz configured rate of `Cfg0` does not imply unique accelerometer
    /// data at 8 kHz.
    pub fn sample_rate_hz(self, divider: u8) -> f32 {
        let base_rate = match self {
            Self::Cfg0 => 8000.0,
            Self::Cfg1 | Self::Cfg2 | Self::Cfg3 | Self::Cfg4 | Self::Cfg5 | Self::Cfg6 => 1000.0,
        };
        base_rate / (divider as f32 + 1.0)
    }
}

/// Error returned while reading the digital low-pass filter configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DlpfReadError<E> {
    I2c(E),
    /// The raw masked CONFIG bits are `0b111` (7), which is reserved.
    ReservedConfig,
}

impl GyroRange {
    const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn who_am_i(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(registers::WHO_AM_I)
    }

    pub fn identity(&mut self) -> Result<Identity, I2C::Error> {
        self.who_am_i().map(Identity::from_who_am_i)
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::ACCEL_CONFIG,
            registers::ACCEL_RANGE_MASK,
            range.bits(),
        )
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::GYRO_CONFIG,
            registers::GYRO_RANGE_MASK,
            range.bits(),
        )
    }

    /// Sets the CONFIG register digital low-pass filter field.
    pub fn set_dlpf(&mut self, dlpf: Dlpf) -> Result<(), I2C::Error> {
        self.write_masked(registers::CONFIG, registers::DLPF_CFG_MASK, dlpf.bits())
    }

    /// Reads the CONFIG register digital low-pass filter field.
    pub fn dlpf(&mut self) -> Result<Dlpf, DlpfReadError<I2C::Error>> {
        let value = self
            .read_register(registers::CONFIG)
            .map_err(DlpfReadError::I2c)?;
        Dlpf::from_bits(value & registers::DLPF_CFG_MASK).ok_or(DlpfReadError::ReservedConfig)
    }

    /// Writes the full SMPLRT_DIV register value.
    pub fn set_sample_rate_divider(&mut self, divider: u8) -> Result<(), I2C::Error> {
        self.write_register(registers::SMPLRT_DIV, divider)
    }

    /// Reads the full SMPLRT_DIV register value.
    pub fn sample_rate_divider(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(registers::SMPLRT_DIV)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{Error, ErrorKind, ErrorType, Operation, SevenBitAddress};
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum FakeError {
        Bus,
    }
    impl Error for FakeError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    enum Expected {
        Read(u8, Result<u8, FakeError>),
        Write(u8, u8, Result<(), FakeError>),
    }
    struct FakeI2c {
        expected: VecDeque<Expected>,
    }
    impl FakeI2c {
        fn new(expected: Vec<Expected>) -> Self {
            Self {
                expected: expected.into(),
            }
        }
    }
    impl ErrorType for FakeI2c {
        type Error = FakeError;
    }
    impl I2c for FakeI2c {
        fn read(&mut self, _: SevenBitAddress, _: &mut [u8]) -> Result<(), Self::Error> {
            unreachable!()
        }
        fn write(&mut self, address: SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
            assert_eq!(address, Address::Ad0Low.as_u8());
            match self.expected.pop_front().expect("unexpected write") {
                Expected::Write(register, value, result) => {
                    assert_eq!(bytes, &[register, value]);
                    result
                }
                Expected::Read(..) => panic!("expected read"),
            }
        }
        fn write_read(
            &mut self,
            address: SevenBitAddress,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            assert_eq!(address, Address::Ad0Low.as_u8());
            assert_eq!(read.len(), 1);
            match self.expected.pop_front().expect("unexpected read") {
                Expected::Read(register, result) => {
                    assert_eq!(write, &[register]);
                    read[0] = result?;
                    Ok(())
                }
                Expected::Write(..) => panic!("expected write"),
            }
        }
        fn transaction(
            &mut self,
            _: SevenBitAddress,
            _: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            unreachable!()
        }
    }

    #[test]
    fn dlpf_encodings_decodes_and_rates_are_complete() {
        let all = [
            Dlpf::Cfg0,
            Dlpf::Cfg1,
            Dlpf::Cfg2,
            Dlpf::Cfg3,
            Dlpf::Cfg4,
            Dlpf::Cfg5,
            Dlpf::Cfg6,
        ];
        for (bits, dlpf) in all.into_iter().enumerate() {
            assert_eq!(dlpf.bits(), bits as u8);
            assert_eq!(Dlpf::from_bits(bits as u8), Some(dlpf));
        }
        assert_eq!(Dlpf::from_bits(7), None);
        assert_eq!(Dlpf::Cfg0.sample_rate_hz(0), 8000.0);
        assert_eq!(Dlpf::Cfg1.sample_rate_hz(0), 1000.0);
        assert_eq!(Dlpf::Cfg2.sample_rate_hz(4), 200.0);
        for dlpf in all[1..].iter().copied() {
            assert_eq!(dlpf.sample_rate_hz(255), 1000.0 / 256.0);
        }
        assert!((Dlpf::Cfg0.sample_rate_hz(255) - 31.25).abs() < 0.000_01);
        assert!((Dlpf::Cfg3.sample_rate_hz(2) - 333.333_34).abs() < 0.001);
    }

    #[test]
    fn dlpf_reads_mask_reserved_and_i2c_errors() {
        for (raw, expected) in [
            (2, Ok(Dlpf::Cfg2)),
            (0x3a, Ok(Dlpf::Cfg2)),
            (7, Err(DlpfReadError::ReservedConfig)),
            (0x3f, Err(DlpfReadError::ReservedConfig)),
        ] {
            let mut mpu = Mpu6050::new(
                FakeI2c::new(vec![Expected::Read(registers::CONFIG, Ok(raw))]),
                Address::Ad0Low,
            );
            assert_eq!(mpu.dlpf(), expected);
            assert!(mpu.release().expected.is_empty());
        }
        let mut mpu = Mpu6050::new(
            FakeI2c::new(vec![Expected::Read(registers::CONFIG, Err(FakeError::Bus))]),
            Address::Ad0Low,
        );
        assert_eq!(mpu.dlpf(), Err(DlpfReadError::I2c(FakeError::Bus)));
    }

    #[test]
    fn dlpf_setter_preserves_config_upper_bits_and_propagates_failures() {
        let mut mpu = Mpu6050::new(
            FakeI2c::new(vec![
                Expected::Read(registers::CONFIG, Ok(0xf8)),
                Expected::Write(registers::CONFIG, 0xfa, Ok(())),
            ]),
            Address::Ad0Low,
        );
        assert_eq!(mpu.set_dlpf(Dlpf::Cfg2), Ok(()));
        assert!(mpu.release().expected.is_empty());
        let mut read_error = Mpu6050::new(
            FakeI2c::new(vec![Expected::Read(registers::CONFIG, Err(FakeError::Bus))]),
            Address::Ad0Low,
        );
        assert_eq!(read_error.set_dlpf(Dlpf::Cfg2), Err(FakeError::Bus));
        let mut write_error = Mpu6050::new(
            FakeI2c::new(vec![
                Expected::Read(registers::CONFIG, Ok(0)),
                Expected::Write(registers::CONFIG, 2, Err(FakeError::Bus)),
            ]),
            Address::Ad0Low,
        );
        assert_eq!(write_error.set_dlpf(Dlpf::Cfg2), Err(FakeError::Bus));
    }

    #[test]
    fn divider_reads_writes_and_propagates_errors_unchanged() {
        let mut mpu = Mpu6050::new(
            FakeI2c::new(vec![
                Expected::Write(registers::SMPLRT_DIV, 37, Ok(())),
                Expected::Read(registers::SMPLRT_DIV, Ok(37)),
            ]),
            Address::Ad0Low,
        );
        assert_eq!(mpu.set_sample_rate_divider(37), Ok(()));
        assert_eq!(mpu.sample_rate_divider(), Ok(37));
        assert!(mpu.release().expected.is_empty());
        let mut set_error = Mpu6050::new(
            FakeI2c::new(vec![Expected::Write(
                registers::SMPLRT_DIV,
                1,
                Err(FakeError::Bus),
            )]),
            Address::Ad0Low,
        );
        assert_eq!(set_error.set_sample_rate_divider(1), Err(FakeError::Bus));
        let mut get_error = Mpu6050::new(
            FakeI2c::new(vec![Expected::Read(
                registers::SMPLRT_DIV,
                Err(FakeError::Bus),
            )]),
            Address::Ad0Low,
        );
        assert_eq!(get_error.sample_rate_divider(), Err(FakeError::Bus));
    }

    #[test]
    fn accel_and_gyro_range_masked_writes_remain_unchanged() {
        let mut mpu = Mpu6050::new(
            FakeI2c::new(vec![
                Expected::Read(registers::ACCEL_CONFIG, Ok(0xe7)),
                Expected::Write(registers::ACCEL_CONFIG, 0xf7, Ok(())),
                Expected::Read(registers::GYRO_CONFIG, Ok(0xe7)),
                Expected::Write(registers::GYRO_CONFIG, 0xff, Ok(())),
            ]),
            Address::Ad0Low,
        );
        assert_eq!(mpu.set_accel_range(AccelRange::G8), Ok(()));
        assert_eq!(mpu.set_gyro_range(GyroRange::Dps2000), Ok(()));
        assert!(mpu.release().expected.is_empty());
    }
}
