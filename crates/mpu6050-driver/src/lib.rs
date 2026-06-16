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
        contains_i16_sentinel(self.accel)
            || contains_i16_sentinel(self.gyro)
            || self.temp == i16::MIN
            || self.temp == i16::MAX
            || all_minus_one(self.accel)
            || all_minus_one(self.gyro)
    }
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
    fn read_fifo_byte(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(registers::FIFO_R_W)
    }
    pub fn read_fifo_bytes(&mut self, bytes: &mut [u8]) -> Result<(), I2C::Error> {
        for byte in bytes {
            *byte = self.read_fifo_byte()?;
        }
        Ok(())
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

    #[test]
    fn address_values_match_ad0_pin_state() {
        assert_eq!(Address::Ad0Low.as_u8(), 0x68);
        assert_eq!(Address::Ad0High.as_u8(), 0x69);
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
    fn regression_raw_sample_flags_i16_sentinels_as_suspicious() {
        let raw = RawAccelGyroTemp::new([i16::MAX, 2, 3], 25, [4, i16::MIN, 6]);
        assert!(raw.is_suspicious());
    }

    #[test]
    fn regression_raw_sample_accepts_nominal_values() {
        let raw = RawAccelGyroTemp::new([-6500, -9900, -9600], 3700, [720, 190, -320]);
        assert!(!raw.is_suspicious());
    }
}
