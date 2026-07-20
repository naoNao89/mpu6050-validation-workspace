//! Host-side sketch of the usual bring-up sequence using a mock I2C bus.
//!
//! On real hardware, replace `MockI2c` with your board's
//! `embedded_hal::i2c::I2c` implementation.

use embedded_hal::i2c::{ErrorType, I2c, Operation, SevenBitAddress};
use mpu6050_driver::{AccelRange, Address, Dlpf, GyroRange, Identity, Mpu6050, raw_to_imu_sample};

/// Mock bus that answers the register traffic this example issues.
///
/// - `WHO_AM_I` (0x75) → `0x68` (MPU-6050)
/// - accel/temp/gyro block (0x3B) → 1 g on Z, ~1 °/s on Z, temp raw 0
/// - other reads → 0
struct MockI2c;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MockError;

impl embedded_hal::i2c::Error for MockError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl MockI2c {
    fn fill_read(reg: u8, read: &mut [u8]) {
        read.fill(0);
        match (reg, read.len()) {
            (0x75, 1) => read[0] = 0x68, // WHO_AM_I
            (0x3B, 14) => {
                // ACCEL_ZOUT = 16384 (±2 g → 1 g), TEMP = 0, GYRO_ZOUT = 131 (~1 °/s)
                read[4] = 0x40;
                read[5] = 0x00;
                read[12] = 0x00;
                read[13] = 0x83;
            }
            _ => {}
        }
    }
}

impl I2c for MockI2c {
    fn read(&mut self, _address: SevenBitAddress, read: &mut [u8]) -> Result<(), Self::Error> {
        read.fill(0);
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
        let reg = write.first().copied().unwrap_or(0);
        Self::fill_read(reg, read);
        Ok(())
    }

    fn transaction(
        &mut self,
        _address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut last_write_reg = 0_u8;
        for operation in operations.iter_mut() {
            match operation {
                Operation::Write(bytes) => {
                    last_write_reg = bytes.first().copied().unwrap_or(0);
                }
                Operation::Read(buf) => {
                    Self::fill_read(last_write_reg, buf);
                }
            }
        }
        Ok(())
    }
}

fn main() {
    let mut mpu = Mpu6050::new(MockI2c, Address::Ad0Low);

    mpu.wake().expect("wake");
    mpu.set_accel_range(AccelRange::G2).expect("accel range");
    mpu.set_gyro_range(GyroRange::Dps250).expect("gyro range");
    mpu.set_dlpf(Dlpf::Cfg2).expect("dlpf");
    mpu.set_sample_rate_divider(4).expect("sample rate divider");

    let identity = mpu.identity().expect("identity");
    assert_eq!(
        identity,
        Identity::Mpu6050,
        "example mock must look like a real MPU-6050, not Unknown"
    );

    let raw = mpu.read_raw_accel_gyro_temp().expect("raw sample");
    let sample = raw_to_imu_sample(raw);

    println!("identity={identity:?}");
    println!(
        "accel_g={:?} gyro_dps={:?} temp_c={:.2}",
        sample.accel_g,
        sample.gyro_dps,
        raw.temp_degrees_c()
    );
}
