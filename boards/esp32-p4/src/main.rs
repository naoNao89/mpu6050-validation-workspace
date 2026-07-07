#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]
#![allow(dead_code)]
#![allow(unused_imports)]

#[cfg(all(feature = "boot-probe", feature = "pin-wiggle"))]
compile_error!("features boot-probe and pin-wiggle are mutually exclusive");
#[cfg(all(feature = "boot-probe", feature = "i2c-probe"))]
compile_error!("features boot-probe and i2c-probe are mutually exclusive");
#[cfg(all(feature = "boot-probe", feature = "i2c-bitbang"))]
compile_error!("features boot-probe and i2c-bitbang are mutually exclusive");
#[cfg(all(feature = "boot-probe", feature = "mpu-smoke"))]
compile_error!("features boot-probe and mpu-smoke are mutually exclusive");
#[cfg(all(feature = "pin-wiggle", feature = "i2c-probe"))]
compile_error!("features pin-wiggle and i2c-probe are mutually exclusive");
#[cfg(all(feature = "pin-wiggle", feature = "i2c-bitbang"))]
compile_error!("features pin-wiggle and i2c-bitbang are mutually exclusive");
#[cfg(all(feature = "pin-wiggle", feature = "mpu-smoke"))]
compile_error!("features pin-wiggle and mpu-smoke are mutually exclusive");
#[cfg(all(feature = "i2c-probe", feature = "i2c-bitbang"))]
compile_error!("features i2c-probe and i2c-bitbang are mutually exclusive");
#[cfg(all(feature = "i2c-probe", feature = "mpu-smoke"))]
compile_error!("features i2c-probe and mpu-smoke are mutually exclusive");
#[cfg(all(feature = "i2c-bitbang", feature = "mpu-smoke"))]
compile_error!("features i2c-bitbang and mpu-smoke are mutually exclusive");

#[cfg(target_arch = "riscv32")]
esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(not(target_arch = "riscv32"))]
fn main() {}

use core::fmt;

#[cfg(any(test, target_arch = "riscv32"))]
mod bitbang_i2c;
mod board;

#[cfg(target_arch = "riscv32")]
use board::{
    AD0_PIN_NAME, BOARD_NAME, GND_PIN_NAME, I2C_BUS_NAME, I2C_FREQUENCY_KHZ, INT_PIN_NAME,
    SCL_PIN_NAME, SDA_PIN_NAME, VCC_PIN_NAME, XCL_PIN_NAME, XDA_PIN_NAME,
};
#[cfg(target_arch = "riscv32")]
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Flex, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Duration, Instant, Rate},
};
#[cfg(target_arch = "riscv32")]
use esp_println::println;
#[cfg(all(feature = "binary-frames", target_arch = "riscv32"))]
use esp_println::Printer;
use mpu6050_driver::Identity;
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::{Address, Mpu6050, RawAccelGyroTemp, RawReadOutcome, RawRetryPolicy};
#[cfg(target_arch = "riscv32")]
use crate::bitbang_i2c::BitbangI2c;

const MPU_ADDR_AD0_LOW: u8 = 0x68;
const MPU_ADDR_AD0_HIGH: u8 = 0x69;

const RAW_STREAM_PERIOD_MS: u32 = 100;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_MAGIC: [u8; 2] = *b"IM";
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_VERSION: u8 = 1;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_PAYLOAD_LEN: u8 = 32;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_LEN: usize = 38;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IdentityVerdict {
    ClassicMpu6050,
    Mpu6500Compatible,
    Unknown,
}

impl IdentityVerdict {
    fn from_identity(identity: Identity) -> Self {
        match identity {
            Identity::Mpu6050 => Self::ClassicMpu6050,
            Identity::Mpu6500Compatible => Self::Mpu6500Compatible,
            Identity::Unknown(_) => Self::Unknown,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::ClassicMpu6050 => "ClassicMpu6050",
            Self::Mpu6500Compatible => "Mpu6500CompatibleNonClassic",
            Self::Unknown => "Unknown",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerificationLevel {
    MarkingOnly,
    I2cResponsive,
    RegisterCompatible,
}

impl VerificationLevel {
    fn from_score(score: u8) -> Self {
        match score {
            0..=1 => Self::MarkingOnly,
            2..=3 => Self::I2cResponsive,
            _ => Self::RegisterCompatible,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::MarkingOnly => "MarkingOnly",
            Self::I2cResponsive => "I2cResponsiveCompatibleDevice",
            Self::RegisterCompatible => "FunctionalRegisterCompatibleImu",
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct VerificationEvidence {
    package_marking_matches: bool,
    i2c_ack: bool,
    identity: Option<Identity>,
    pwr_mgmt_1_readable: bool,
    raw_block_readable: bool,
}

#[derive(Debug, Default, Clone, Copy)]
struct RawIntegrityStats {
    total_reads: u64,
    clean: u64,
    suspicious_first: u64,
    recovered_by_retry: u64,
    rejected_suspicious: u64,
    accepted_suspicious: u64,
    retry_error: u64,
}

impl VerificationEvidence {
    fn score(self) -> u8 {
        let mut score = 0;
        if self.package_marking_matches {
            score += 1;
        }
        if self.i2c_ack {
            score += 2;
        }
        match self.identity {
            Some(Identity::Mpu6050) => score += 4,
            Some(Identity::Mpu6500Compatible) => score += 2,
            Some(Identity::Unknown(_)) | None => {}
        }
        if self.pwr_mgmt_1_readable {
            score += 3;
        }
        if self.raw_block_readable {
            score += 3;
        }
        score
    }

    fn identity_verdict(self) -> IdentityVerdict {
        self.identity
            .map(IdentityVerdict::from_identity)
            .unwrap_or(IdentityVerdict::Unknown)
    }

    fn level(self) -> VerificationLevel {
        VerificationLevel::from_score(self.score())
    }
}

#[derive(Debug, Clone, Copy)]
struct ProbeResult {
    address: u8,
    who_am_i: Option<u8>,
    identity: Option<Identity>,
    pwr_mgmt_1: Option<u8>,
    raw_block_readable: bool,
}

impl ProbeResult {
    fn has_communication_evidence(self) -> bool {
        self.who_am_i.is_some() || self.raw_block_readable
    }
}

struct HexOpt(Option<u8>);
struct DecimalU64(u64);

impl fmt::Display for HexOpt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "0x{:02x}", value),
            None => f.write_str("unreadable"),
        }
    }
}

impl fmt::Display for DecimalU64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut value = self.0;
        let mut digits = [0_u8; 20];
        let mut len = 0;

        if value == 0 {
            return f.write_str("0");
        }

        while value > 0 {
            digits[len] = b'0' + (value % 10) as u8;
            value /= 10;
            len += 1;
        }

        while len > 0 {
            len -= 1;
            f.write_str(core::str::from_utf8(&digits[len..len + 1]).map_err(|_| fmt::Error)?)?;
        }
        Ok(())
    }
}

trait IdentityDescription {
    fn description(self) -> &'static str;
}

impl IdentityDescription for Identity {
    fn description(self) -> &'static str {
        match self {
            Self::Mpu6050 => "MPU-6050-class IMU",
            Self::Mpu6500Compatible => "MPU-6500-compatible / clone / relabeled variant",
            Self::Unknown(_) => "unknown IMU identity",
        }
    }
}

#[cfg(target_arch = "riscv32")]
#[main]
fn main() -> ! {
    #[cfg(feature = "boot-probe")]
    {
        println!("ESP32-P4 boot-probe before-hal-init");
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();
        println!("ESP32-P4 boot-probe after-hal-init");
        log_board_profile();
        loop {
            println!("ESP32-P4 boot-probe alive");
            delay.delay_millis(1000);
        }
    }

    #[cfg(feature = "pin-wiggle")]
    {
        println!("ESP32-P4 pin-wiggle before-hal-init");
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();
        println!("ESP32-P4 pin-wiggle after-hal-init");
        println!("Measure each named MPU-side pin against GND while it is HIGH/LOW");

        // Continuity test only: drive the exact pins from board.rs as plain GPIO
        // outputs. Use carefully on wired/powered modules; SDA/SCL may also be
        // pulled up externally and XDA/XCL/AD0 may be connected on the module.
        // INT is never driven here because it is an MPU output.
        // This avoids the I2C peripheral and makes pinmux/continuity issues
        // visible with a multimeter at the MPU module header.
        println!("WARNING: pin-wiggle drives SDA/SCL/XDA/XCL/AD0 as a continuity test only");
        println!("WARNING: use carefully on wired/powered modules; INT/GPIO27 is input-only");
        let mut scl = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());
        let mut sda = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());
        let mut xda = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());
        let mut xcl = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
        let mut ad0 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
        let int = Input::new(
            peripherals.GPIO27,
            InputConfig::default().with_pull(Pull::Up),
        );

        loop {
            println!("pin-wiggle all LOW");
            sda.set_low();
            scl.set_low();
            xda.set_low();
            xcl.set_low();
            ad0.set_low();
            println!(
                "pin-wiggle INT GPIO27 input={}",
                if int.is_high() { "HIGH" } else { "LOW" }
            );
            delay.delay_millis(1500);

            println!("pin-wiggle SDA GPIO21 HIGH");
            sda.set_high();
            delay.delay_millis(1500);
            sda.set_low();

            println!("pin-wiggle SCL GPIO20 HIGH");
            scl.set_high();
            delay.delay_millis(1500);
            scl.set_low();

            println!("pin-wiggle XDA GPIO22 HIGH");
            xda.set_high();
            delay.delay_millis(1500);
            xda.set_low();

            println!("pin-wiggle XCL GPIO23 HIGH");
            xcl.set_high();
            delay.delay_millis(1500);
            xcl.set_low();

            println!("pin-wiggle AD0 GPIO26 HIGH");
            ad0.set_high();
            delay.delay_millis(1500);
            ad0.set_low();

            println!(
                "pin-wiggle INT GPIO27 input={}",
                if int.is_high() { "HIGH" } else { "LOW" }
            );
            delay.delay_millis(1500);
        }
    }

    #[cfg(feature = "i2c-probe")]
    {
        println!("ESP32-P4 i2c-probe before-hal-init");
        let mut peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();
        println!("ESP32-P4 i2c-probe after-hal-init");
        log_board_profile();

        let scl_probe = Input::new(
            peripherals.GPIO20.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        let sda_probe = Input::new(
            peripherals.GPIO21.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        let int_probe = Input::new(
            peripherals.GPIO27.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        println!(
            "i2c_probe pre_i2c_idle scl={} sda={} int={}",
            if scl_probe.is_high() { "HIGH" } else { "LOW" },
            if sda_probe.is_high() { "HIGH" } else { "LOW" },
            if int_probe.is_high() { "HIGH" } else { "LOW" }
        );
        drop(scl_probe);
        drop(sda_probe);
        drop(int_probe);

        let mut ad0 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
        println!("i2c_probe ad0=LOW pin={}", AD0_PIN_NAME);

        let config = I2cConfig::default()
            .with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ))
            .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(500)));
        let scl = i2c_open_drain_pin(peripherals.GPIO20);
        let sda = i2c_open_drain_pin(peripherals.GPIO21);
        let mut i2c = I2c::new(peripherals.I2C0, config)
            .expect("failed to initialize I2C0")
            .with_sda(sda)
            .with_scl(scl);
        i2c.apply_config(&config)
            .expect("failed to re-apply I2C config after pin routing");
        println!(
            "{} initialized at {} kHz: SCL={} SDA={}",
            I2C_BUS_NAME, I2C_FREQUENCY_KHZ, SCL_PIN_NAME, SDA_PIN_NAME
        );

        hal_raw_probe(&mut i2c, "LOW", MPU_ADDR_AD0_LOW);
        hal_raw_probe(&mut i2c, "LOW", MPU_ADDR_AD0_HIGH);

        let i2c = probe_who_am_i_no_wake(i2c, "LOW", Address::Ad0Low, MPU_ADDR_AD0_LOW);
        let i2c = probe_who_am_i_no_wake(i2c, "LOW", Address::Ad0High, MPU_ADDR_AD0_HIGH);

        ad0.set_high();
        delay.delay_millis(20);
        println!("i2c_probe ad0=HIGH pin={}", AD0_PIN_NAME);
        let i2c = probe_who_am_i_no_wake(i2c, "HIGH", Address::Ad0Low, MPU_ADDR_AD0_LOW);
        let _i2c = probe_who_am_i_no_wake(i2c, "HIGH", Address::Ad0High, MPU_ADDR_AD0_HIGH);

        loop {
            println!("i2c_probe idle: complete, no raw streaming");
            delay.delay_millis(5000);
        }
    }

    #[cfg(feature = "i2c-bitbang")]
    {
        println!("ESP32-P4 i2c-bitbang before-hal-init");
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();
        println!("ESP32-P4 i2c-bitbang after-hal-init");
        log_board_profile();

        let mut scl = bitbang_i2c_pin(peripherals.GPIO20);
        let mut sda = bitbang_i2c_pin(peripherals.GPIO21);
        let mut ad0 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());

        bitbang_release(&mut scl);
        bitbang_release(&mut sda);
        delay.delay_millis(5);

        ad0.set_low();
        delay.delay_millis(5);
        bitbang_probe_address(&mut scl, &mut sda, &delay, "LOW", MPU_ADDR_AD0_LOW, 0xd0);
        bitbang_probe_address(&mut scl, &mut sda, &delay, "LOW", MPU_ADDR_AD0_HIGH, 0xd2);

        ad0.set_high();
        delay.delay_millis(20);
        bitbang_probe_address(&mut scl, &mut sda, &delay, "HIGH", MPU_ADDR_AD0_LOW, 0xd0);
        bitbang_probe_address(&mut scl, &mut sda, &delay, "HIGH", MPU_ADDR_AD0_HIGH, 0xd2);

        bitbang_release(&mut scl);
        bitbang_release(&mut sda);
        loop {
            println!("bitbang_i2c idle: complete, no raw streaming");
            delay.delay_millis(5000);
        }
    }

    #[cfg(feature = "mpu-smoke")]
    {
        println!("ESP32-P4 mpu-smoke before-hal-init");
        let mut peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();
        println!("ESP32-P4 mpu-smoke after-hal-init");

        let scl_probe = Input::new(
            peripherals.GPIO20.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        let sda_probe = Input::new(
            peripherals.GPIO21.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        println!(
            "Pre-I2C idle check: SCL={} SDA={}",
            if scl_probe.is_high() { "HIGH" } else { "LOW" },
            if sda_probe.is_high() { "HIGH" } else { "LOW" }
        );
        drop(scl_probe);
        drop(sda_probe);

        // AD0 low selects the MPU6050's 0x68 address. The smoke test still
        // probes 0x69 afterwards so a bad AD0 connection does not hide as a
        // false negative.
        let _ad0 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
        println!(
            "AD0 driven LOW on {}; expected 7-bit address is 0x68",
            AD0_PIN_NAME
        );

        let scl = i2c_open_drain_pin(peripherals.GPIO20);
        let sda = i2c_open_drain_pin(peripherals.GPIO21);
        let i2c = BitbangI2c::new(scl, sda);
        println!(
            "{} initialized via software bitbang {} kHz: SCL={} SDA={}",
            I2C_BUS_NAME, I2C_FREQUENCY_KHZ, SCL_PIN_NAME, SDA_PIN_NAME
        );

        let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
        let wake_ok = mpu.wake().is_ok();
        println!(
            "driver wake bus_address=0x{:02x} ok={}",
            MPU_ADDR_AD0_LOW, wake_ok
        );
        let probe = probe_imu_driver(&mut mpu, MPU_ADDR_AD0_LOW);
        log_verification_summary(probe);

        let i2c = mpu.release();
        let mut high_mpu = Mpu6050::new(i2c, Address::Ad0High);
        println!(
            "driver wake bus_address=0x{:02x} ok={}",
            MPU_ADDR_AD0_HIGH,
            high_mpu.wake().is_ok()
        );
        let _ = probe_imu_driver(&mut high_mpu, MPU_ADDR_AD0_HIGH);
        let i2c = high_mpu.release();
        let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);

        if !probe.has_communication_evidence() {
            println!(
                "mpu-smoke: no 0x68 WHO_AM_I/raw evidence; raw streaming disabled to avoid repeated timeout spam"
            );
            loop {
                println!("mpu-smoke idle: no communication evidence at 0x68");
                delay.delay_millis(5000);
            }
        }

        let mut raw_sequence = 0;
        let mut integrity_stats = RawIntegrityStats::default();
        loop {
            read_motion_sample_retry_once(
                &mut mpu,
                MPU_ADDR_AD0_LOW,
                &mut raw_sequence,
                &mut integrity_stats,
            );
            delay.delay_millis(RAW_STREAM_PERIOD_MS);
        }
    }

    #[cfg(not(any(
        feature = "boot-probe",
        feature = "pin-wiggle",
        feature = "i2c-probe",
        feature = "i2c-bitbang",
        feature = "mpu-smoke"
    )))]
    {
        let mut peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();

        println!("MPU6050 ESP32-P4 esp-hal I2C bring-up started");
        log_board_profile();

        let scl_probe = Input::new(
            peripherals.GPIO20.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        let sda_probe = Input::new(
            peripherals.GPIO21.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        println!(
            "Pre-I2C idle check: SCL={} SDA={}",
            if scl_probe.is_high() { "HIGH" } else { "LOW" },
            if sda_probe.is_high() { "HIGH" } else { "LOW" }
        );
        drop(scl_probe);
        drop(sda_probe);

        let _ad0 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
        println!(
            "AD0 driven LOW on {}; expected 7-bit address is 0x68",
            AD0_PIN_NAME
        );

        let scl = i2c_open_drain_pin(peripherals.GPIO20);
        let sda = i2c_open_drain_pin(peripherals.GPIO21);
        let i2c = BitbangI2c::new(scl, sda);
        println!(
            "{} initialized via software bitbang {} kHz: SCL={} SDA={}",
            I2C_BUS_NAME, I2C_FREQUENCY_KHZ, SCL_PIN_NAME, SDA_PIN_NAME
        );

        let i2c = scan_candidates(i2c);
        let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
        let wake_ok = mpu.wake().is_ok();
        println!(
            "driver wake bus_address=0x{:02x} ok={}",
            MPU_ADDR_AD0_LOW, wake_ok
        );
        let primary_probe = probe_imu_driver(&mut mpu, MPU_ADDR_AD0_LOW);
        let i2c = mpu.release();
        let mut high_mpu = Mpu6050::new(i2c, Address::Ad0High);
        let _ = probe_imu_driver(&mut high_mpu, MPU_ADDR_AD0_HIGH);
        let i2c = high_mpu.release();
        let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
        log_verification_summary(primary_probe);
        println!(
            "advanced_validation_skipped: not run by default on ESP32-P4 bring-up without real register-read evidence"
        );

        if !primary_probe.has_communication_evidence() {
            println!(
                "raw_stream_skipped: no 0x68 WHO_AM_I/raw evidence; check module power, wiring, pinout, and AD0 before streaming"
            );
            loop {
                println!("idle: no communication evidence at 0x68");
                delay.delay_millis(5000);
            }
        }

        println!(
            "Repeating raw read from 0x68 every {}ms",
            RAW_STREAM_PERIOD_MS
        );
        let mut raw_sequence: u64 = 0;
        let mut integrity_stats = RawIntegrityStats::default();
        loop {
            read_motion_sample_retry_once(
                &mut mpu,
                MPU_ADDR_AD0_LOW,
                &mut raw_sequence,
                &mut integrity_stats,
            );
            delay.delay_millis(RAW_STREAM_PERIOD_MS);
        }
    }
}

#[cfg(target_arch = "riscv32")]
fn log_board_profile() {
    println!("Board profile: {}", BOARD_NAME);
    println!(
        "Wiring: VCC={} GND={} SCL={} SDA={} XDA={} XCL={} AD0={} INT={}",
        VCC_PIN_NAME,
        GND_PIN_NAME,
        SCL_PIN_NAME,
        SDA_PIN_NAME,
        XDA_PIN_NAME,
        XCL_PIN_NAME,
        AD0_PIN_NAME,
        INT_PIN_NAME
    );
}

#[cfg(target_arch = "riscv32")]
type BoardMpu<'a> = Mpu6050<BitbangI2c<'a>>;
/// I2C transport type for `i2c-probe` mode (direct HAL peripheral access).
#[cfg(target_arch = "riscv32")]
type BoardMpuI2c<'a> = I2c<'a, esp_hal::Blocking>;

#[cfg(target_arch = "riscv32")]
fn scan_candidates(i2c: BitbangI2c<'_>) -> BitbangI2c<'_> {
    println!("I2C candidate scan: 0x68, 0x69");
    let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
    match mpu.who_am_i() {
        Ok(value) => println!(
            "ACK/read at 0x{:02x}: WHO_AM_I=0x{:02x}",
            MPU_ADDR_AD0_LOW, value
        ),
        Err(error) => println!("No read at 0x{:02x}: {:?}", MPU_ADDR_AD0_LOW, error),
    }
    let i2c = mpu.release();
    let mut mpu = Mpu6050::new(i2c, Address::Ad0High);
    match mpu.who_am_i() {
        Ok(value) => println!(
            "ACK/read at 0x{:02x}: WHO_AM_I=0x{:02x}",
            MPU_ADDR_AD0_HIGH, value
        ),
        Err(error) => println!("No read at 0x{:02x}: {:?}", MPU_ADDR_AD0_HIGH, error),
    }
    mpu.release()
}

#[cfg(all(feature = "i2c-probe", target_arch = "riscv32"))]
fn probe_who_am_i_no_wake<'a>(
    i2c: I2c<'a, esp_hal::Blocking>,
    ad0_state: &str,
    address: Address,
    address_byte: u8,
) -> I2c<'a, esp_hal::Blocking> {
    let mut mpu = Mpu6050::new(i2c, address);
    match mpu.who_am_i() {
        Ok(value) => println!(
            "i2c_probe ad0={} address=0x{:02x} who_am_i=0x{:02x} ok=true error=none",
            ad0_state, address_byte, value
        ),
        Err(error) => println!(
            "i2c_probe ad0={} address=0x{:02x} who_am_i=unreadable ok=false error={:?}",
            ad0_state, address_byte, error
        ),
    }
    mpu.release()
}

#[cfg(all(feature = "i2c-probe", target_arch = "riscv32"))]
fn hal_raw_probe(i2c: &mut BoardMpuI2c<'_>, ad0_state: &str, address: u8) {
    let write_result = i2c.write(address, &[0x75]);
    println!(
        "i2c_probe_raw ad0={} address=0x{:02x} op=write_reg who_am_i_reg result={:?} interrupts_after={:?}",
        ad0_state,
        address,
        write_result,
        i2c.interrupts()
    );

    let mut value = [0_u8; 1];
    let read_result = i2c.write_read(address, &[0x75], &mut value);
    println!(
        "i2c_probe_raw ad0={} address=0x{:02x} op=write_read who_am_i={} result={:?} interrupts_after={:?}",
        ad0_state,
        address,
        if read_result.is_ok() { HexOpt(Some(value[0])) } else { HexOpt(None) },
        read_result,
        i2c.interrupts()
    );
}

#[cfg(target_arch = "riscv32")]
fn i2c_open_drain_pin(pin: impl esp_hal::gpio::Pin + 'static) -> Flex<'static> {
    let mut pin = Flex::new(pin);
    pin.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
    pin.apply_output_config(
        &OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::Up),
    );
    pin.set_high();
    pin.set_input_enable(true);
    pin.set_output_enable(true);
    pin
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_i2c_pin(pin: impl esp_hal::gpio::Pin + 'static) -> Flex<'static> {
    i2c_open_drain_pin(pin)
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_release(pin: &mut Flex<'_>) {
    pin.set_high();
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_pull_low(pin: &mut Flex<'_>) {
    pin.set_low();
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_delay(delay: &Delay) {
    delay.delay_millis(1);
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_start(scl: &mut Flex<'_>, sda: &mut Flex<'_>, delay: &Delay) {
    bitbang_release(sda);
    bitbang_release(scl);
    bitbang_delay(delay);
    bitbang_pull_low(sda);
    bitbang_delay(delay);
    bitbang_pull_low(scl);
    bitbang_delay(delay);
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_stop(scl: &mut Flex<'_>, sda: &mut Flex<'_>, delay: &Delay) {
    bitbang_pull_low(sda);
    bitbang_delay(delay);
    bitbang_release(scl);
    bitbang_delay(delay);
    bitbang_release(sda);
    bitbang_delay(delay);
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_write_bit(scl: &mut Flex<'_>, sda: &mut Flex<'_>, delay: &Delay, bit: bool) {
    if bit {
        bitbang_release(sda);
    } else {
        bitbang_pull_low(sda);
    }
    bitbang_delay(delay);
    bitbang_release(scl);
    bitbang_delay(delay);
    bitbang_pull_low(scl);
    bitbang_delay(delay);
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_write_byte_read_ack(
    scl: &mut Flex<'_>,
    sda: &mut Flex<'_>,
    delay: &Delay,
    byte: u8,
) -> bool {
    for bit in (0..8).rev() {
        bitbang_write_bit(scl, sda, delay, byte & (1 << bit) != 0);
    }
    bitbang_release(sda);
    bitbang_delay(delay);
    bitbang_release(scl);
    bitbang_delay(delay);
    let ack = sda.is_low();
    bitbang_pull_low(scl);
    bitbang_delay(delay);
    ack
}

#[cfg(all(feature = "i2c-bitbang", target_arch = "riscv32"))]
fn bitbang_probe_address(
    scl: &mut Flex<'_>,
    sda: &mut Flex<'_>,
    delay: &Delay,
    ad0_state: &str,
    address: u8,
    address_write_byte: u8,
) {
    bitbang_release(scl);
    bitbang_release(sda);
    bitbang_delay(delay);
    let scl_idle = if scl.is_high() { "HIGH" } else { "LOW" };
    let sda_idle = if sda.is_high() { "HIGH" } else { "LOW" };
    bitbang_start(scl, sda, delay);
    let ack = bitbang_write_byte_read_ack(scl, sda, delay, address_write_byte);
    bitbang_stop(scl, sda, delay);
    println!(
        "bitbang_i2c ad0={} address=0x{:02x} ack={} scl_idle={} sda_idle={}",
        ad0_state, address, ack, scl_idle, sda_idle
    );
}

#[cfg(target_arch = "riscv32")]
fn probe_imu_driver(mpu: &mut BoardMpu<'_>, address: u8) -> ProbeResult {
    println!("Probing bus_address=0x{:02x}", address);
    let mut who_am_i = None;
    let mut identity = None;
    let pwr_mgmt_1 = None;

    match mpu.who_am_i() {
        Ok(value) => {
            who_am_i = Some(value);
            identity = mpu.identity().ok();
            println!(
                "bus_address=0x{:02x} who_am_i=0x{:02x} identity={}",
                address,
                value,
                identity.unwrap_or(Identity::Unknown(value)).description()
            );
            if let Some(Identity::Unknown(id)) = identity {
                println!(
                    "bus_address=0x{:02x}: unknown WHO_AM_I=0x{:02x}; raw reads will still be attempted",
                    address, id
                );
            }
        }
        Err(error) => println!("0x{:02x}: WHO_AM_I read failed: {:?}", address, error),
    }

    println!("0x{:02x}: PWR_MGMT_1 read failed: not exposed", address);

    let raw_block_readable = mpu.read_raw_accel_gyro_temp().is_ok();
    println!(
        "bus_address=0x{:02x} raw_block_0x3b_readable={}",
        address, raw_block_readable
    );

    ProbeResult {
        address,
        who_am_i,
        identity,
        pwr_mgmt_1,
        raw_block_readable,
    }
}

#[cfg(target_arch = "riscv32")]
fn log_verification_summary(probe: ProbeResult) {
    let identity = probe.identity;
    let evidence = VerificationEvidence {
        package_marking_matches: true,
        i2c_ack: probe.who_am_i.is_some() || probe.pwr_mgmt_1.is_some() || probe.raw_block_readable,
        identity,
        pwr_mgmt_1_readable: probe.pwr_mgmt_1.is_some(),
        raw_block_readable: probe.raw_block_readable,
    };
    let score = evidence.score();
    println!("verification_summary_begin");
    println!("bus_address=0x{:02x}", probe.address);
    match probe.who_am_i {
        Some(value) => println!("who_am_i=0x{:02x}", value),
        None => println!("who_am_i=unreadable"),
    }
    match probe.pwr_mgmt_1 {
        Some(value) => println!("pwr_mgmt_1=0x{:02x}", value),
        None => println!("pwr_mgmt_1=unreadable"),
    }
    println!("identity_verdict={}", evidence.identity_verdict().as_str());
    println!("verification_score={}", score);
    println!("verification_level={}", evidence.level().as_str());
    println!(
        "non_classic_identity={}",
        matches!(identity, Some(Identity::Mpu6500Compatible))
    );
    println!("advanced_tests=not_run_on_p4_bringup_without_register_read_evidence");
    println!("verification_summary_end");
}

#[cfg(target_arch = "riscv32")]
fn read_motion_sample_retry_once(
    mpu: &mut BoardMpu<'_>,
    address: u8,
    raw_sequence: &mut u64,
    integrity_stats: &mut RawIntegrityStats,
) {
    integrity_stats.total_reads = integrity_stats.total_reads.wrapping_add(1);
    match mpu.read_raw_with_retry(RawRetryPolicy::reject_after_retries(1)) {
        Ok(RawReadOutcome::Clean { raw }) => {
            integrity_stats.clean = integrity_stats.clean.wrapping_add(1);
            emit_motion_sample(address, raw_sequence, raw);
        }
        Ok(RawReadOutcome::Recovered {
            raw,
            first_suspicion,
            retries,
        }) => {
            integrity_stats.suspicious_first = integrity_stats.suspicious_first.wrapping_add(1);
            integrity_stats.recovered_by_retry = integrity_stats.recovered_by_retry.wrapping_add(1);
            #[cfg(feature = "binary-frames")]
            let _ = (&first_suspicion, retries);
            #[cfg(not(feature = "binary-frames"))]
            println!(
                "RAW 0x{:02x}: suspicious sample recovered by retry sequence={}",
                address, *raw_sequence
            );
            #[cfg(not(feature = "binary-frames"))]
            {
                emit_raw_integrity_event(
                    *raw_sequence,
                    "recovered",
                    retries,
                    first_suspicion,
                    integrity_stats,
                );
            }
            emit_motion_sample(address, raw_sequence, raw);
        }
        Ok(RawReadOutcome::RejectedSuspicious {
            raw,
            suspicion,
            retries,
        }) => {
            integrity_stats.suspicious_first = integrity_stats.suspicious_first.wrapping_add(1);
            integrity_stats.rejected_suspicious =
                integrity_stats.rejected_suspicious.wrapping_add(1);
            #[cfg(feature = "binary-frames")]
            let _ = (&suspicion, retries);
            #[cfg(not(feature = "binary-frames"))]
            {
                emit_raw_integrity_event(
                    *raw_sequence,
                    "rejected",
                    retries,
                    suspicion,
                    integrity_stats,
                );
            }
            log_suspicious_sample("retry_suspicious_skipped", address, *raw_sequence, raw)
        }
        Ok(RawReadOutcome::RetryError {
            first_suspicion,
            retries,
            error,
            ..
        }) => {
            integrity_stats.suspicious_first = integrity_stats.suspicious_first.wrapping_add(1);
            integrity_stats.retry_error = integrity_stats.retry_error.wrapping_add(1);
            #[cfg(feature = "binary-frames")]
            let _ = (&first_suspicion, retries);
            #[cfg(not(feature = "binary-frames"))]
            {
                emit_raw_integrity_event(
                    *raw_sequence,
                    "retry_error",
                    retries,
                    first_suspicion,
                    integrity_stats,
                );
            }
            println!(
                "RAW 0x{:02x}: suspicious sample retry failed: {:?}",
                address, error
            )
        }
        Ok(RawReadOutcome::AcceptedSuspicious {
            raw,
            suspicion,
            retries,
        }) => {
            integrity_stats.suspicious_first = integrity_stats.suspicious_first.wrapping_add(1);
            integrity_stats.accepted_suspicious =
                integrity_stats.accepted_suspicious.wrapping_add(1);
            #[cfg(feature = "binary-frames")]
            let _ = (&suspicion, retries);
            #[cfg(not(feature = "binary-frames"))]
            {
                emit_raw_integrity_event(
                    *raw_sequence,
                    "accepted",
                    retries,
                    suspicion,
                    integrity_stats,
                );
            }
            emit_motion_sample(address, raw_sequence, raw)
        }
        Err(error) => println!("RAW 0x{:02x}: read failed: {:?}", address, error),
    }
}

#[cfg(all(not(feature = "binary-frames"), target_arch = "riscv32"))]
fn emit_raw_integrity_event(
    sequence: u64,
    outcome: &str,
    retries: usize,
    suspicion: impl fmt::Debug,
    stats: &RawIntegrityStats,
) {
    if stats.accepted_suspicious == 0 {
        println!(
            "raw_integrity_event seq={} outcome={} reason={:?} retries={}",
            sequence, outcome, suspicion, retries
        );
    } else {
        println!(
            "raw_integrity_event seq={} outcome={} reason={:?} retries={} accepted_total={}",
            sequence, outcome, suspicion, retries, stats.accepted_suspicious
        );
    }
}

#[cfg(target_arch = "riscv32")]
fn emit_motion_sample(address: u8, raw_sequence: &mut u64, raw: RawAccelGyroTemp) {
    #[cfg(not(feature = "binary-frames"))]
    let timestamp_us = Instant::now().duration_since_epoch().as_micros() as u64;
    #[cfg(feature = "binary-frames")]
    {
        let timestamp_us = Instant::now().duration_since_epoch().as_micros() as u64;
        let frame = encode_binary_frame(
            address,
            *raw_sequence,
            timestamp_us,
            raw.accel,
            raw.temp,
            raw.gyro,
        );
        Printer::write_bytes(&frame);
    }
    #[cfg(not(feature = "binary-frames"))]
    println!(
        "RAW 0x{:02x}: accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) timestamp_us={} sequence={} timestamp_source=device_instant",
        address,
        raw.accel[0],
        raw.accel[1],
        raw.accel[2],
        raw.temp,
        raw.gyro[0],
        raw.gyro[1],
        raw.gyro[2],
        DecimalU64(timestamp_us),
        *raw_sequence
    );
    *raw_sequence = raw_sequence.wrapping_add(1);
}

#[cfg(target_arch = "riscv32")]
fn log_suspicious_sample(reason: &str, address: u8, sequence: u64, raw: RawAccelGyroTemp) {
    #[cfg(not(feature = "binary-frames"))]
    println!(
        "RAW 0x{:02x}: suspicious sample {}: accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) sequence={}",
        address,
        reason,
        raw.accel[0],
        raw.accel[1],
        raw.accel[2],
        raw.temp,
        raw.gyro[0],
        raw.gyro[1],
        raw.gyro[2],
        sequence
    );
    #[cfg(feature = "binary-frames")]
    let _ = (reason, address, sequence, raw);
}

#[cfg(all(feature = "binary-frames", target_arch = "riscv32"))]
fn encode_binary_frame(
    address: u8,
    sequence: u64,
    timestamp_us: u64,
    accel: [i16; 3],
    temp: i16,
    gyro: [i16; 3],
) -> [u8; BINARY_FRAME_LEN] {
    let mut b = [0u8; BINARY_FRAME_LEN];
    b[0..2].copy_from_slice(&BINARY_FRAME_MAGIC);
    b[2] = BINARY_FRAME_VERSION;
    b[3] = BINARY_FRAME_PAYLOAD_LEN;
    b[4] = address;
    b[6..14].copy_from_slice(&sequence.to_le_bytes());
    b[14..22].copy_from_slice(&timestamp_us.to_le_bytes());
    for (off, v) in [
        (22, accel[0]),
        (24, accel[1]),
        (26, accel[2]),
        (28, temp),
        (30, gyro[0]),
        (32, gyro[1]),
        (34, gyro[2]),
    ] {
        b[off..off + 2].copy_from_slice(&v.to_le_bytes());
    }
    let crc = crc16_ccitt_false(&b[..BINARY_FRAME_LEN - 2]);
    b[BINARY_FRAME_LEN - 2..].copy_from_slice(&crc.to_le_bytes());
    b
}

#[cfg(feature = "binary-frames")]
fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}
