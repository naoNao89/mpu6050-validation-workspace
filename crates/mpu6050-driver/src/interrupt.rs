use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// The complete `INT_ENABLE` register state.
pub struct InterruptEnable {
    bits: u8,
}

impl InterruptEnable {
    /// Returns whether the data-ready interrupt is enabled.
    pub const fn data_ready(self) -> bool {
        self.bits & registers::INT_ENABLE_DATA_RDY != 0
    }

    /// Returns true only when `DATA_RDY` is the complete `INT_ENABLE` byte.
    ///
    /// This is suitable for verifying that no unsupported interrupt sources
    /// were retained by a read-modify-write operation.
    pub const fn only_data_ready(self) -> bool {
        self.bits == registers::INT_ENABLE_DATA_RDY
    }

    /// Returns whether the FIFO-overflow interrupt is enabled.
    pub const fn fifo_overflow(self) -> bool {
        self.bits & registers::INT_ENABLE_FIFO_OFLOW != 0
    }

    /// Returns true only when the complete `INT_ENABLE` byte is zero.
    ///
    /// Unsupported nonzero bits make this false.
    pub const fn none_enabled(self) -> bool {
        self.bits == 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntStatus {
    pub(crate) bits: u8,
}

impl IntStatus {
    pub const fn data_ready(self) -> bool {
        self.bits & registers::INT_STATUS_DATA_RDY != 0
    }

    pub const fn fifo_overflow(self) -> bool {
        self.bits & registers::INT_STATUS_FIFO_OFLOW != 0
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    /// Reads the complete `INT_ENABLE` register state.
    pub fn interrupt_enable(&mut self) -> Result<InterruptEnable, I2C::Error> {
        self.read_register(registers::INT_ENABLE)
            .map(|bits| InterruptEnable { bits })
    }

    /// Disables all interrupt sources by writing zero to `INT_ENABLE`.
    pub fn disable_all_interrupts(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::INT_ENABLE, 0)
    }

    pub fn enable_data_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            registers::INT_ENABLE_DATA_RDY,
            registers::INT_ENABLE_DATA_RDY,
        )
    }

    pub fn enable_fifo_overflow_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            registers::INT_ENABLE_FIFO_OFLOW,
            registers::INT_ENABLE_FIFO_OFLOW,
        )
    }

    /// Reads and clears the currently latched interrupt status bits.
    ///
    /// Reading `INT_STATUS` clears the reported status bits. If `INT_RD_CLEAR`
    /// is enabled in `INT_PIN_CFG`, other register reads may clear interrupt
    /// status as well.
    ///
    /// This is an explicit status-inspection operation. Do not busy-poll it for
    /// sample acquisition; use the physical INT signal or FIFO.
    pub fn int_status(&mut self) -> Result<IntStatus, I2C::Error> {
        self.read_register(registers::INT_STATUS)
            .map(|bits| IntStatus { bits })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};
    use std::collections::VecDeque;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum FakeError {
        Bus,
    }

    impl embedded_hal::i2c::Error for FakeError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    enum ExpectedOperation {
        Read(u8, Result<u8, FakeError>),
        Write(u8, u8, Result<(), FakeError>),
    }

    struct FakeI2c {
        operations: VecDeque<ExpectedOperation>,
    }

    impl FakeI2c {
        fn new(operations: Vec<ExpectedOperation>) -> Self {
            Self {
                operations: operations.into(),
            }
        }
    }

    impl ErrorType for FakeI2c {
        type Error = FakeError;
    }

    impl I2c for FakeI2c {
        fn read(&mut self, _address: SevenBitAddress, _read: &mut [u8]) -> Result<(), Self::Error> {
            unreachable!("driver uses write_read")
        }

        fn write(&mut self, _address: SevenBitAddress, write: &[u8]) -> Result<(), Self::Error> {
            match self.operations.pop_front().expect("unexpected write") {
                ExpectedOperation::Write(register, value, result) => {
                    assert_eq!(write, &[register, value]);
                    result
                }
                ExpectedOperation::Read(..) => panic!("expected read"),
            }
        }

        fn write_read(
            &mut self,
            _address: SevenBitAddress,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            match self.operations.pop_front().expect("unexpected read") {
                ExpectedOperation::Read(register, result) => {
                    assert_eq!(write, &[register]);
                    match result {
                        Ok(value) => {
                            read.copy_from_slice(&[value]);
                            Ok(())
                        }
                        Err(error) => Err(error),
                    }
                }
                ExpectedOperation::Write(..) => panic!("expected write"),
            }
        }

        fn transaction(
            &mut self,
            _address: SevenBitAddress,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            unreachable!("driver uses write and write_read")
        }
    }

    #[test]
    fn disable_all_interrupts_writes_zero_without_reading() {
        let fake = FakeI2c::new(std::vec![ExpectedOperation::Write(
            registers::INT_ENABLE,
            0,
            Ok(()),
        )]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        assert_eq!(mpu.disable_all_interrupts(), Ok(()));
        assert!(mpu.release().operations.is_empty());
    }

    #[test]
    fn interrupt_enable_reads_and_decodes_complete_register_byte() {
        let fake = FakeI2c::new(std::vec![ExpectedOperation::Read(
            registers::INT_ENABLE,
            Ok(registers::INT_ENABLE_DATA_RDY | registers::INT_ENABLE_FIFO_OFLOW | 0x80),
        )]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        let enable = mpu.interrupt_enable().unwrap();

        assert!(enable.data_ready());
        assert!(enable.fifo_overflow());
        assert!(!enable.only_data_ready());
        assert!(!enable.none_enabled());
        assert!(mpu.release().operations.is_empty());
    }

    #[test]
    fn interrupt_enable_decodes_individual_and_no_enabled_bits() {
        let fake = FakeI2c::new(std::vec![
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(registers::INT_ENABLE_DATA_RDY)),
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(registers::INT_ENABLE_FIFO_OFLOW)),
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(0)),
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(0x80)),
        ]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        let data_ready = mpu.interrupt_enable().unwrap();
        assert!(data_ready.data_ready());
        assert!(data_ready.only_data_ready());
        assert!(!data_ready.fifo_overflow());
        let fifo_overflow = mpu.interrupt_enable().unwrap();
        assert!(!fifo_overflow.data_ready());
        assert!(fifo_overflow.fifo_overflow());
        assert!(mpu.interrupt_enable().unwrap().none_enabled());
        let unsupported = mpu.interrupt_enable().unwrap();
        assert!(!unsupported.none_enabled());
        assert!(!unsupported.only_data_ready());
        assert!(mpu.release().operations.is_empty());
    }

    #[test]
    fn interrupt_operations_propagate_read_and_write_errors() {
        let fake = FakeI2c::new(std::vec![
            ExpectedOperation::Read(registers::INT_ENABLE, Err(FakeError::Bus)),
            ExpectedOperation::Write(registers::INT_ENABLE, 0, Err(FakeError::Bus)),
        ]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        assert_eq!(mpu.interrupt_enable(), Err(FakeError::Bus));
        assert_eq!(mpu.disable_all_interrupts(), Err(FakeError::Bus));
        assert!(mpu.release().operations.is_empty());
    }

    #[test]
    fn masked_enable_methods_preserve_unrelated_bits() {
        let unrelated = 0x82;
        let fake = FakeI2c::new(std::vec![
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(unrelated)),
            ExpectedOperation::Write(
                registers::INT_ENABLE,
                unrelated | registers::INT_ENABLE_DATA_RDY,
                Ok(()),
            ),
            ExpectedOperation::Read(registers::INT_ENABLE, Ok(unrelated)),
            ExpectedOperation::Write(
                registers::INT_ENABLE,
                unrelated | registers::INT_ENABLE_FIFO_OFLOW,
                Ok(()),
            ),
        ]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        assert_eq!(mpu.enable_data_ready_interrupt(), Ok(()));
        assert_eq!(mpu.enable_fifo_overflow_interrupt(), Ok(()));
        assert!(mpu.release().operations.is_empty());
    }

    #[test]
    fn int_status_decoding_is_unchanged() {
        let fake = FakeI2c::new(std::vec![ExpectedOperation::Read(
            registers::INT_STATUS,
            Ok(registers::INT_STATUS_DATA_RDY | registers::INT_STATUS_FIFO_OFLOW),
        )]);
        let mut mpu = Mpu6050::new(fake, crate::Address::Ad0Low);

        let status = mpu.int_status().unwrap();

        assert!(status.data_ready());
        assert!(status.fifo_overflow());
        assert!(mpu.release().operations.is_empty());
    }
}
