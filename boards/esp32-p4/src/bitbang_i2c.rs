use embedded_hal::i2c::{
    ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};
#[cfg(target_arch = "riscv32")]
use esp_hal::gpio::Flex;

const SCL_RELEASE_TIMEOUT: u16 = 1_000;

trait OpenDrainPin {
    fn set_high(&mut self);
    fn set_low(&mut self);
    fn is_high(&self) -> bool;

    fn is_low(&self) -> bool {
        !self.is_high()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{cell::RefCell, rc::Rc, vec::Vec};

    #[derive(Clone)]
    struct Pin {
        name: &'static str,
        high: bool,
        forced_low: bool,
        reads: Rc<RefCell<Vec<bool>>>,
        log: Rc<RefCell<Vec<&'static str>>>,
    }

    impl Pin {
        fn new(name: &'static str, log: Rc<RefCell<Vec<&'static str>>>) -> Self {
            Self { name, high: true, forced_low: false, reads: Rc::new(RefCell::new(Vec::new())), log }
        }

        fn with_reads(self, reads: &[bool]) -> Self {
            *self.reads.borrow_mut() = reads.iter().rev().copied().collect();
            self
        }

        fn stuck_low(mut self) -> Self {
            self.forced_low = true;
            self
        }
    }

    impl OpenDrainPin for Pin {
        fn set_high(&mut self) {
            self.high = true;
            self.log.borrow_mut().push(if self.name == "scl" { "SCLH" } else { "SDAH" });
        }

        fn set_low(&mut self) {
            self.high = false;
            self.log.borrow_mut().push(if self.name == "scl" { "SCLL" } else { "SDAL" });
        }

        fn is_high(&self) -> bool {
            if self.forced_low {
                false
            } else {
                self.reads.borrow_mut().pop().unwrap_or(self.high)
            }
        }
    }

    fn core_with_sda_reads(reads: &[bool]) -> (BitbangCore<Pin, Pin>, Rc<RefCell<Vec<&'static str>>>) {
        let log = Rc::new(RefCell::new(Vec::new()));
        let scl = Pin::new("scl", log.clone());
        let sda = Pin::new("sda", log.clone()).with_reads(reads);
        (BitbangCore::new(scl, sda), log)
    }

    #[test]
    fn start_stop_sequence_releases_scl_before_sda_edges() {
        let (mut i2c, log) = core_with_sda_reads(&[]);
        i2c.start().unwrap();
        i2c.stop().unwrap();
        let log = log.borrow();
        assert!(log.windows(2).any(|w| w == ["SCLH", "SDAL"]));
        assert!(log.windows(2).any(|w| w == ["SCLH", "SDAH"]));
    }

    #[test]
    fn address_ack_and_nack_are_reported() {
        let (mut ack, _) = core_with_sda_reads(&[false]);
        assert_eq!(ack.write_byte(0xD0, Error::AddressNack), Ok(()));
        let (mut nack, _) = core_with_sda_reads(&[true]);
        assert_eq!(nack.write_byte(0xD0, Error::AddressNack), Err(Error::AddressNack));
    }

    #[test]
    fn write_read_uses_repeated_start() {
        let (mut i2c, log) = core_with_sda_reads(&[
            false, false, false, true, false, true, false, true, false, true, false,
        ]);
        i2c.start().unwrap();
        i2c.write_byte(0xD0, Error::AddressNack).unwrap();
        i2c.write_byte(0x75, Error::DataNack).unwrap();
        i2c.start().unwrap();
        i2c.write_byte(0xD1, Error::AddressNack).unwrap();
        let _ = i2c.read_byte(false).unwrap();
        assert!(log.borrow().windows(2).filter(|w| *w == ["SCLH", "SDAL"]).count() >= 2);
    }

    #[test]
    fn read_byte_sends_ack_or_nack_after_byte() {
        let (mut ack, log_ack) = core_with_sda_reads(&[false; 8]);
        assert_eq!(ack.read_byte(true), Ok(0));
        assert!(log_ack.borrow().windows(2).any(|w| w == ["SDAL", "SCLH"]));
        let (mut nack, log_nack) = core_with_sda_reads(&[true; 8]);
        assert_eq!(nack.read_byte(false), Ok(0xFF));
        assert!(log_nack.borrow().windows(2).any(|w| w == ["SDAH", "SCLH"]));
    }

    #[test]
    fn bus_recovery_clocks_nine_times() {
        let (mut i2c, log) = core_with_sda_reads(&[]);
        i2c.recover_bus().unwrap();
        assert_eq!(log.borrow().iter().filter(|&&e| e == "SCLH").count(), 10);
    }

    #[test]
    fn scl_stuck_low_returns_bus_error() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let scl = Pin::new("scl", log.clone()).stuck_low();
        let sda = Pin::new("sda", log);
        let mut i2c = BitbangCore::new(scl, sda);
        assert_eq!(i2c.start(), Err(Error::Bus));
    }

    #[test]
    fn transaction_write_write_has_one_start_one_address() {
        let (mut i2c, log) = core_with_sda_reads(&[false, false, false]);
        let mut ops = [Operation::Write(&[0x75]), Operation::Write(&[0x6B])];

        i2c.transaction(0x68, &mut ops).unwrap();

        let log = log.borrow();
        assert_eq!(count_starts(&log), 1);
        // One address byte/ACK plus two data bytes/ACK; the previous buggy
        // implementation added another START and address byte here.
        assert_eq!(count_scl_highs(&log), 28);
    }

    #[test]
    fn transaction_write_read_has_repeated_start() {
        let (mut i2c, log) = core_with_sda_reads(&[
            false, false, false, true, false, true, false, true, false, true, false,
        ]);
        let mut read = [0u8];
        let mut ops = [Operation::Write(&[0x75]), Operation::Read(&mut read)];

        i2c.transaction(0x68, &mut ops).unwrap();

        let log = log.borrow();
        assert_eq!(count_starts(&log), 2);
        // Direction changes once, so this includes one repeated START and the
        // second address byte for the read phase.
        assert_eq!(count_scl_highs(&log), 38);
    }

    #[test]
    fn transaction_read_read_has_one_start_one_address() {
        let (mut i2c, log) = core_with_sda_reads(&[
            false, false, false, false, true, true, false, false, true, true, false, false, true,
            true, false, false, true,
        ]);
        let mut first = [0u8];
        let mut second = [0u8];
        let mut ops = [Operation::Read(&mut first), Operation::Read(&mut second)];

        i2c.transaction(0x68, &mut ops).unwrap();

        let log = log.borrow();
        assert_eq!(count_starts(&log), 1);
        // One read address byte/ACK plus two read bytes/ACK-or-NACK; no
        // repeated START or second address between the adjacent reads.
        assert_eq!(count_scl_highs(&log), 28);
    }

    fn count_starts(log: &[&str]) -> usize {
        log.windows(2).filter(|w| *w == ["SCLH", "SDAL"]).count()
    }

    fn count_scl_highs(log: &[&str]) -> usize {
        log.iter().filter(|&&e| e == "SCLH").count()
    }
}

#[cfg(target_arch = "riscv32")]
impl OpenDrainPin for Flex<'_> {
    fn set_high(&mut self) {
        Flex::set_high(self);
    }

    fn set_low(&mut self) {
        Flex::set_low(self);
    }

    fn is_high(&self) -> bool {
        Flex::is_high(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    AddressNack,
    DataNack,
    Bus,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::AddressNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::DataNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Self::Bus => ErrorKind::Bus,
        }
    }
}

/// Software bitbang I2C master using open-drain Flex GPIO pins.
///
/// C3 and most other ESP chips can use esp-hal's normal
/// `I2c::new(...).with_sda(...).with_scl(...)` path. On the tested ESP32-P4
/// rev v1.3 board, the HAL I2C v3 peripheral times out with zero interrupts,
/// so this provides the same `embedded_hal::i2c::I2c` interface manually. This
/// simple MPU-focused master does not implement clock stretching; it does clock
/// out a short bus-clear sequence during initialization.
#[cfg(target_arch = "riscv32")]
pub struct BitbangI2c<'a> {
    inner: BitbangCore<Flex<'a>, Flex<'a>>,
}

#[cfg(target_arch = "riscv32")]
impl<'a> BitbangI2c<'a> {
    pub fn new(scl: Flex<'a>, sda: Flex<'a>) -> Self {
        let mut i2c = Self {
            inner: BitbangCore::new(scl, sda),
        };
        let _ = i2c.inner.recover_bus();
        i2c
    }
}

struct BitbangCore<Scl, Sda> {
    scl: Scl,
    sda: Sda,
}

impl<Scl: OpenDrainPin, Sda: OpenDrainPin> BitbangCore<Scl, Sda> {
    fn new(scl: Scl, sda: Sda) -> Self {
        Self { scl, sda }
    }

    fn release_bus(&mut self) -> Result<(), Error> {
        self.sda.set_high();
        self.scl.set_high();
        self.wait_scl_high()
    }

    fn recover_bus(&mut self) -> Result<(), Error> {
        self.sda.set_high();
        for _ in 0..9 {
            self.scl.set_low();
            self.delay();
            self.scl.set_high();
            self.wait_scl_high()?;
            self.delay();
        }
        self.stop()
    }

    fn wait_scl_high(&self) -> Result<(), Error> {
        for _ in 0..SCL_RELEASE_TIMEOUT {
            if self.scl.is_high() {
                return Ok(());
            }
        }
        Err(Error::Bus)
    }

    /// Busy-wait half-period delay (~2µs at 400 MHz, ~20µs at 40 MHz).
    /// I2C is synchronous so exact timing is not critical.
    fn delay(&self) {
        for _ in 0..800u32 {
            unsafe {
                core::arch::asm!("nop");
            }
        }
    }

    /// Generate a START (or repeated START) condition.
    /// SDA HIGH→LOW while SCL HIGH.
    fn start(&mut self) -> Result<(), Error> {
        self.sda.set_high();
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        self.sda.set_low();
        self.delay();
        self.scl.set_low();
        self.delay();
        Ok(())
    }

    /// Generate a STOP condition.
    /// SDA LOW→HIGH while SCL HIGH.
    fn stop(&mut self) -> Result<(), Error> {
        self.sda.set_low();
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        self.sda.set_high();
        self.delay();
        Ok(())
    }

    /// Write a single bit (MSB-first). SCL is LOW on entry and exit.
    fn write_bit(&mut self, bit: bool) -> Result<(), Error> {
        if bit {
            self.sda.set_high(); // release (pull-up pulls HIGH)
        } else {
            self.sda.set_low(); // drive LOW
        }
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        self.scl.set_low();
        self.delay();
        Ok(())
    }

    /// Read a single bit (MSB-first). SCL is LOW on entry and exit.
    fn read_bit(&mut self) -> Result<bool, Error> {
        self.sda.set_high(); // release for slave
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        let bit = self.sda.is_high();
        self.scl.set_low();
        self.delay();
        Ok(bit)
    }

    /// Write a byte and check the ACK bit.
    /// Returns `Ok(())` on ACK (SDA low), `Err(Nack)` on NACK.
    fn write_byte(&mut self, byte: u8, nack_error: Error) -> Result<(), Error> {
        for i in (0..8).rev() {
            self.write_bit(byte & (1 << i) != 0)?;
        }
        // Release SDA so slave can drive ACK
        self.sda.set_high();
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        let ack = self.sda.is_low();
        self.scl.set_low();
        self.delay();
        if ack {
            Ok(())
        } else {
            Err(nack_error)
        }
    }

    /// Read a byte. Sends ACK if `send_ack` is true, NAK otherwise.
    fn read_byte(&mut self, send_ack: bool) -> Result<u8, Error> {
        let mut byte = 0u8;
        for _ in 0..8 {
            byte = (byte << 1) | (self.read_bit()? as u8);
        }
        if send_ack {
            self.sda.set_low(); // ACK
        } else {
            self.sda.set_high(); // NAK
        }
        self.delay();
        self.scl.set_high();
        self.wait_scl_high()?;
        self.delay();
        self.scl.set_low();
        self.delay();
        Ok(byte)
    }

    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error> {
        let mut previous_read = None;

        for op in operations {
            let is_read = matches!(op, Operation::Read(_));
            match previous_read {
                None => {
                    self.start()?;
                    self.write_address(address, is_read)?;
                }
                Some(previous) if previous != is_read => {
                    self.start()?; // repeated START only when changing direction
                    self.write_address(address, is_read)?;
                }
                Some(_) => {
                    // Same direction: continue the data phase without STOP,
                    // repeated START, or another address byte.
                }
            }

            match op {
                Operation::Write(bytes) => {
                    for &byte in bytes.iter() {
                        self.write_byte(byte, Error::DataNack)?;
                    }
                }
                Operation::Read(buffer) => {
                    for i in 0..buffer.len() {
                        self.read_into(buffer, i)?;
                    }
                }
            }

            previous_read = Some(is_read);
        }

        Ok(())
    }

    fn write_address(&mut self, address: SevenBitAddress, read: bool) -> Result<(), Error> {
        self.write_byte((address << 1) | u8::from(read), Error::AddressNack)
    }

    fn read_into(&mut self, buffer: &mut [u8], index: usize) -> Result<(), Error> {
        buffer[index] = self.read_byte(index < buffer.len() - 1)?;
        Ok(())
    }
}

#[cfg(target_arch = "riscv32")]
impl ErrorType for BitbangI2c<'_> {
    type Error = Error;
}

#[cfg(target_arch = "riscv32")]
impl I2c for BitbangI2c<'_> {
    fn read(&mut self, address: SevenBitAddress, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.inner.start()?;
        let result = (|| {
            self.inner
                .write_byte((address << 1) | 1, Error::AddressNack)?;
            for i in 0..buffer.len() {
                buffer[i] = self.inner.read_byte(i < buffer.len() - 1)?;
            }
            Ok(())
        })();
        let stop = self.inner.stop();
        stop.and(result)
    }

    fn write(&mut self, address: SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
        self.inner.start()?;
        let result = (|| {
            self.inner.write_byte(address << 1, Error::AddressNack)?;
            for &byte in bytes {
                self.inner.write_byte(byte, Error::DataNack)?;
            }
            Ok(())
        })();
        let stop = self.inner.stop();
        stop.and(result)
    }

    fn write_read(
        &mut self,
        address: SevenBitAddress,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.inner.start()?;
        let result = (|| {
            self.inner.write_byte(address << 1, Error::AddressNack)?;
            for &byte in bytes {
                self.inner.write_byte(byte, Error::DataNack)?;
            }
            // Repeated start for the read phase
            self.inner.start()?;
            self.inner
                .write_byte((address << 1) | 1, Error::AddressNack)?;
            for i in 0..buffer.len() {
                buffer[i] = self.inner.read_byte(i < buffer.len() - 1)?;
            }
            Ok(())
        })();
        let stop = self.inner.stop();
        stop.and(result)
    }

    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let result = self.inner.transaction(address, operations);
        let stop = self.inner.stop();
        stop.and(result)
    }
}
