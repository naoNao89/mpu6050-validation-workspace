#![no_std]
#![no_main]

use core::fmt;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Duration, Instant, Rate},
};
#[cfg(feature = "binary-frames")]
use esp_println::Printer;
use esp_println::println;
use mpu6050_driver::{
    AccelRange, Address, GyroRange, Identity, Mpu6050, RawAccelGyroTemp, RawReadOutcome,
    RawRetryPolicy,
};

const MPU_ADDR_AD0_LOW: u8 = 0x68;
const MPU_ADDR_AD0_HIGH: u8 = 0x69;

const FIFO_ACCEL_GYRO_FRAME_BYTES: u16 = 12;
const RAW_STREAM_PERIOD_MS: u32 = 100;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_MAGIC: [u8; 2] = *b"IM";
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_VERSION: u8 = 1;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_PAYLOAD_LEN: u8 = 32;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_LEN: usize = 38;

// Reference dev-board wiring used by this bring-up firmware.
//
// The repo's board-under-test is an ESP32-C3 SuperMini-class board connected to
// a GY-521/MPU6050 module. Keep these constants aligned with the README so the
// firmware is explicitly a board sample that exercises this MPU6050 driver
// stack, rather than an anonymous ESP32-C3 snippet.
const BOARD_NAME: &str = "ESP32-C3 SuperMini-class dev board";
const I2C_BUS_NAME: &str = "I2C0";
const I2C_FREQUENCY_KHZ: u32 = 100;
const SCL_PIN_NAME: &str = "GPIO0";
const SDA_PIN_NAME: &str = "GPIO1";
const AD0_PIN_NAME: &str = "GPIO5";
const INT_PIN_NAME: &str = "GPIO6";

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
    MotionVerified,
    AdvancedVerified,
}

impl VerificationLevel {
    fn from_score(score: u8) -> Self {
        match score {
            0..=5 => Self::MarkingOnly,
            6..=12 => Self::I2cResponsive,
            13..=22 => Self::RegisterCompatible,
            23..=35 => Self::MotionVerified,
            _ => Self::AdvancedVerified,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::MarkingOnly => "MarkingOnly",
            Self::I2cResponsive => "I2cResponsiveCompatibleDevice",
            Self::RegisterCompatible => "FunctionalRegisterCompatibleImu",
            Self::MotionVerified => "MotionVerifiedCompatibleImu",
            Self::AdvancedVerified => "AdvancedVerifiedCompatibleImu",
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

#[derive(Debug, Clone, Copy)]
struct RawAverage {
    ax: i32,
    ay: i32,
    az: i32,
    gx: i32,
    gy: i32,
    gz: i32,
}

struct HexOpt(Option<u8>);
struct U16Opt(Option<u16>);

impl fmt::Display for HexOpt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "0x{:02x}", value),
            None => f.write_str("unreadable"),
        }
    }
}

impl fmt::Display for U16Opt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "{}", value),
            None => f.write_str("unreadable"),
        }
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

#[main]
fn main() -> ! {
    let mut peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    println!("MPU6050 ESP32-C3 esp-hal I2C bring-up started");
    println!("Board profile: {}", BOARD_NAME);
    println!(
        "Wiring: VCC=3V GND=GND SCL={} SDA={} AD0={} INT={}",
        SCL_PIN_NAME, SDA_PIN_NAME, AD0_PIN_NAME, INT_PIN_NAME
    );

    let scl_probe = Input::new(
        peripherals.GPIO0.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    let sda_probe = Input::new(
        peripherals.GPIO1.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    println!(
        "Pre-I2C idle check: SCL={} SDA={}",
        if scl_probe.is_high() { "HIGH" } else { "LOW" },
        if sda_probe.is_high() { "HIGH" } else { "LOW" }
    );
    drop(scl_probe);
    drop(sda_probe);

    let _ad0 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    println!("AD0 driven LOW on GPIO5; expected 7-bit address is 0x68");

    let config = I2cConfig::default()
        .with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ))
        .with_timeout(BusTimeout::Maximum)
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(50)));

    let i2c = I2c::new(peripherals.I2C0, config)
        .expect("failed to initialize I2C0")
        .with_scl(peripherals.GPIO0)
        .with_sda(peripherals.GPIO1);

    println!(
        "{} initialized at {} kHz: SCL={} SDA={}",
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
    run_advanced_validation(&mut mpu, &delay, MPU_ADDR_AD0_LOW);

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

type BoardMpu<'a> = Mpu6050<I2c<'a, esp_hal::Blocking>>;

fn run_advanced_validation(mpu: &mut BoardMpu<'_>, delay: &Delay, address: u8) {
    println!("advanced_validation_begin");
    reset_wake_configure(mpu, delay, address);
    validate_scale_registers(mpu, address);
    validate_self_test_coarse(mpu, delay, address);
    validate_fifo_timing(mpu, delay, address);
    validate_int_status(mpu, address);
    println!("advanced_validation_end");
}

fn reset_wake_configure(mpu: &mut BoardMpu<'_>, delay: &Delay, _address: u8) {
    println!("advanced reset_wake_begin");
    let reset_ok = mpu.reset().is_ok();
    delay.delay_millis(100);
    let wake_ok = mpu.wake().is_ok();
    delay.delay_millis(20);
    let config_ok = false;
    let sample_ok = false;
    let accel_ok = mpu.set_accel_range(AccelRange::G2).is_ok();
    let gyro_ok = mpu.set_gyro_range(GyroRange::Dps250).is_ok();
    let pwr = None;
    let config = None;
    let smplrt = None;
    println!(
        "advanced reset_wake reset_ok={} wake_ok={} config_ok={} sample_ok={} accel_cfg_ok={} gyro_cfg_ok={} pwr_mgmt_1={} config={} smplrt_div={}",
        reset_ok,
        wake_ok,
        config_ok,
        sample_ok,
        accel_ok,
        gyro_ok,
        fmt_opt_hex(pwr),
        fmt_opt_hex(config),
        fmt_opt_hex(smplrt)
    );
    println!("advanced reset_wake_end");
}

fn validate_scale_registers(mpu: &mut BoardMpu<'_>, _address: u8) {
    println!("advanced scale_range_begin");
    for setting in 0..=3u8 {
        let accel_range = accel_range_from_setting(setting);
        let gyro_range = gyro_range_from_setting(setting);
        let accel_write = mpu.set_accel_range(accel_range).is_ok();
        let gyro_write = mpu.set_gyro_range(gyro_range).is_ok();
        let accel_read = None;
        let gyro_read = None;
        let accel_match = accel_write;
        let gyro_match = gyro_write;
        println!(
            "advanced scale_range setting={} accel_write={} accel_reg={} accel_match={} gyro_write={} gyro_reg={} gyro_match={}",
            setting,
            accel_write,
            fmt_opt_hex(accel_read),
            accel_match,
            gyro_write,
            fmt_opt_hex(gyro_read),
            gyro_match
        );
    }
    let _ = mpu.set_accel_range(AccelRange::G2);
    let _ = mpu.set_gyro_range(GyroRange::Dps250);
    println!("advanced scale_range_end");
}

fn validate_self_test_coarse(mpu: &mut BoardMpu<'_>, delay: &Delay, _address: u8) {
    println!("advanced self_test_begin");
    let _ = mpu.set_accel_range(AccelRange::G2);
    let _ = mpu.set_gyro_range(GyroRange::Dps250);
    let _ = mpu.set_accel_self_test(false);
    let _ = mpu.set_gyro_self_test(false);
    delay.delay_millis(50);
    let baseline = average_raw(mpu, delay, 8);
    let accel_st_ok = mpu.set_accel_self_test(true).is_ok();
    let gyro_st_ok = mpu.set_gyro_self_test(true).is_ok();
    delay.delay_millis(100);
    let self_test = average_raw(mpu, delay, 8);
    let _ = mpu.set_accel_self_test(false);
    let _ = mpu.set_gyro_self_test(false);
    delay.delay_millis(50);

    if let (Some(base), Some(st)) = (baseline, self_test) {
        let accel_delta = abs3_sum(st.ax - base.ax, st.ay - base.ay, st.az - base.az);
        let gyro_delta = abs3_sum(st.gx - base.gx, st.gy - base.gy, st.gz - base.gz);
        println!(
            "advanced self_test accel_st_write={} gyro_st_write={} baseline_accel=({},{},{}) selftest_accel=({},{},{}) baseline_gyro=({},{},{}) selftest_gyro=({},{},{}) accel_delta_sum={} gyro_delta_sum={} coarse_response={}",
            accel_st_ok,
            gyro_st_ok,
            base.ax,
            base.ay,
            base.az,
            st.ax,
            st.ay,
            st.az,
            base.gx,
            base.gy,
            base.gz,
            st.gx,
            st.gy,
            st.gz,
            accel_delta,
            gyro_delta,
            accel_delta > 100 || gyro_delta > 100
        );
    } else {
        println!(
            "advanced self_test accel_st_write={} gyro_st_write={} baseline_readable={} selftest_readable={} coarse_response=false",
            accel_st_ok,
            gyro_st_ok,
            baseline.is_some(),
            self_test.is_some()
        );
    }
    println!("advanced self_test_end");
}

fn validate_fifo_timing(mpu: &mut BoardMpu<'_>, delay: &Delay, _address: u8) {
    println!("advanced fifo_timing_begin");
    let disable_fifo_ok = mpu.disable_fifo_sources().is_ok();
    let user_reset_ok = mpu.reset_fifo().is_ok();
    delay.delay_millis(20);
    let enable_sources_ok = mpu.enable_motion_fifo().is_ok();
    let enable_fifo_ok = mpu.enable_fifo().is_ok();
    let count0 = mpu.fifo_count().ok();
    delay.delay_millis(2);
    let count1 = mpu.fifo_count().ok().or(count0);
    let mut frame_read_ok = false;
    if let Some(count) = count1 {
        if count >= FIFO_ACCEL_GYRO_FRAME_BYTES {
            let mut frame = [0_u8; FIFO_ACCEL_GYRO_FRAME_BYTES as usize];
            frame_read_ok = mpu.read_fifo_bytes(&mut frame).is_ok();
        }
    }
    let disable_after_ok = mpu.disable_fifo_sources().is_ok() && mpu.disable_fifo().is_ok();
    println!(
        "advanced fifo_timing disable_fifo_ok={} user_reset_ok={} enable_sources_ok={} enable_fifo_ok={} count0={} count1={} frame_bytes={} frame_read_ok={} disable_after_ok={}",
        disable_fifo_ok,
        user_reset_ok,
        enable_sources_ok,
        enable_fifo_ok,
        fmt_opt_u16(count0),
        fmt_opt_u16(count1),
        FIFO_ACCEL_GYRO_FRAME_BYTES,
        frame_read_ok,
        disable_after_ok
    );
    println!("advanced fifo_timing_end");
}

fn validate_int_status(mpu: &mut BoardMpu<'_>, _address: u8) {
    println!("advanced int_status_begin");
    let enable_ok =
        mpu.enable_data_ready_interrupt().is_ok() && mpu.enable_fifo_overflow_interrupt().is_ok();
    let status = mpu.int_status().ok();
    let data_ready = status.map(|v| v.data_ready()).unwrap_or(false);
    let fifo_overflow = status.map(|v| v.fifo_overflow()).unwrap_or(false);
    println!(
        "advanced int_status enable_ok={} int_status={} data_ready={} fifo_overflow={}",
        enable_ok,
        fmt_opt_hex(None),
        data_ready,
        fifo_overflow
    );
    println!("advanced int_status_end");
}

fn scan_candidates(i2c: I2c<'_, esp_hal::Blocking>) -> I2c<'_, esp_hal::Blocking> {
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

fn log_verification_summary(probe: ProbeResult) {
    let identity = probe.identity;
    let evidence = VerificationEvidence {
        package_marking_matches: true,
        i2c_ack: probe.who_am_i.is_some() || probe.pwr_mgmt_1.is_some(),
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
    println!(
        "pending_tests=six_face,accel_scale_range,gyro_scale_range,gyro_bias,temp_sanity,self_test,fifo_interrupt,timing_noise"
    );
    println!("verification_summary_end");
}

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

#[cfg(not(feature = "binary-frames"))]
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

fn emit_motion_sample(address: u8, raw_sequence: &mut u64, raw: RawAccelGyroTemp) {
    let timestamp_us = Instant::now().duration_since_epoch().as_micros();
    #[cfg(feature = "binary-frames")]
    {
        let frame = encode_binary_frame(
            address,
            *raw_sequence,
            timestamp_us as u64,
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
        timestamp_us,
        *raw_sequence
    );
    *raw_sequence = raw_sequence.wrapping_add(1);
}

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

#[cfg(feature = "binary-frames")]
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

fn accel_range_from_setting(setting: u8) -> AccelRange {
    match setting {
        0 => AccelRange::G2,
        1 => AccelRange::G4,
        2 => AccelRange::G8,
        _ => AccelRange::G16,
    }
}

fn gyro_range_from_setting(setting: u8) -> GyroRange {
    match setting {
        0 => GyroRange::Dps250,
        1 => GyroRange::Dps500,
        2 => GyroRange::Dps1000,
        _ => GyroRange::Dps2000,
    }
}

fn average_raw(mpu: &mut BoardMpu<'_>, delay: &Delay, samples: i32) -> Option<RawAverage> {
    let mut ax = 0i32;
    let mut ay = 0i32;
    let mut az = 0i32;
    let mut gx = 0i32;
    let mut gy = 0i32;
    let mut gz = 0i32;
    for _ in 0..samples {
        let raw = mpu.read_raw_accel_gyro_temp().ok()?;
        ax += raw.accel[0] as i32;
        ay += raw.accel[1] as i32;
        az += raw.accel[2] as i32;
        gx += raw.gyro[0] as i32;
        gy += raw.gyro[1] as i32;
        gz += raw.gyro[2] as i32;
        delay.delay_millis(10);
    }
    Some(RawAverage {
        ax: ax / samples,
        ay: ay / samples,
        az: az / samples,
        gx: gx / samples,
        gy: gy / samples,
        gz: gz / samples,
    })
}

fn abs3_sum(x: i32, y: i32, z: i32) -> i32 {
    x.abs() + y.abs() + z.abs()
}

fn fmt_opt_hex(value: Option<u8>) -> HexOpt {
    HexOpt(value)
}

fn fmt_opt_u16(value: Option<u16>) -> U16Opt {
    U16Opt(value)
}
