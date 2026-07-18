#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]
#![allow(dead_code)]

#[cfg(target_arch = "riscv32")]
esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(not(target_arch = "riscv32"))]
fn main() {}

use core::fmt;

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
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Duration, Instant, Rate},
};
#[cfg(all(feature = "binary-frames", target_arch = "riscv32"))]
use esp_println::Printer;
#[cfg(target_arch = "riscv32")]
use esp_println::println;
use mpu6050_driver::{AccelRange, Dlpf, GyroRange, Identity};
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::{Address, Mpu6050, RawAccelGyroTemp, RawReadOutcome, RawRetryPolicy};

const MPU_ADDR_AD0_LOW: u8 = 0x68;
const MPU_ADDR_AD0_HIGH: u8 = 0x69;

const FIFO_ACCEL_GYRO_FRAME_BYTES: u16 = 12;
const BLOCKED_IDLE_DELAY_MS: u32 = 100;
const TARGET_DLPF: Dlpf = Dlpf::Cfg2;
// Exact value written to the MPU SMPLRT_DIV register.
const TARGET_SMPLRT_DIV: u8 = 4;
// Nominal rate derived from the configured registers:
// 1_000 Hz / (1 + SMPLRT_DIV=4) = 200 Hz.
// This does not verify the physical sensor cadence or host read rate.
const EXPECTED_NOMINAL_SAMPLE_RATE_HZ: f32 = 200.0;
// Floating-point epsilon for the nominal-rate calculation, not a hardware tolerance.
const NOMINAL_RATE_COMPARISON_EPSILON_HZ: f32 = 0.01;
const RAW_EXAMPLE_LIMIT: u64 = 8;
const SUMMARY_PERIOD_US: u64 = 1_000_000;

#[derive(Default, Debug, Clone, Copy)]
struct StartupConditions {
    diagnostics_complete: bool,
    timing_confirmed: bool,
    final_interrupts_zero: bool,
    gpio_configured: bool,
    stale_status_cleared: bool,
    enable_success: bool,
    exact_data_ready_readback: bool,
}

trait DataReadyStartupDevice {
    fn clear_int_status(&mut self) -> bool;
    fn enable_data_ready(&mut self) -> bool;
    fn only_data_ready_enabled(&mut self) -> Option<bool>;
}

fn configure_data_ready_startup(device: &mut impl DataReadyStartupDevice) -> StartupConditions {
    let stale_status_cleared = device.clear_int_status();
    let enable_success = stale_status_cleared && device.enable_data_ready();
    let exact_data_ready_readback =
        enable_success && device.only_data_ready_enabled().unwrap_or(false);
    StartupConditions {
        stale_status_cleared,
        enable_success,
        exact_data_ready_readback,
        ..Default::default()
    }
}

impl StartupConditions {
    const fn allows_acquisition(self) -> bool {
        self.diagnostics_complete
            && self.timing_confirmed
            && self.final_interrupts_zero
            && self.gpio_configured
            && self.stale_status_cleared
            && self.enable_success
            && self.exact_data_ready_readback
    }
}

#[derive(Default, Debug, Clone, Copy)]
struct PendingEvents {
    pending: u32,
    max_pending: u32,
    total: u64,
    /// Number of data-ready ISR events that could not be added to the pending-event
    /// count because that count was already saturated at `u32::MAX`.
    ///
    /// This is a software counter-saturation metric. It is not an MPU FIFO
    /// overflow and does not directly count lost sensor samples.
    events_unrecorded_due_to_pending_saturation: u64,
}
impl PendingEvents {
    fn signal(&mut self) {
        self.total = self.total.saturating_add(1);
        if let Some(next) = self.pending.checked_add(1) {
            self.pending = next;
            self.max_pending = self.max_pending.max(next);
        } else {
            self.events_unrecorded_due_to_pending_saturation = self
                .events_unrecorded_due_to_pending_saturation
                .saturating_add(1);
        }
    }
    fn take_all(&mut self) -> u32 {
        let pending = self.pending;
        self.pending = 0;
        pending
    }
}

#[derive(Debug, Clone, Copy)]
struct AcquisitionStats {
    consumed: u64,
    missed_or_coalesced_events: u64,
    successful_samples: u64,
    motion_i2c_errors: u64,
    status_ack_i2c_errors: u64,
    first_consumed_us: Option<u64>,
    last_consumed_us: Option<u64>,
    first_sample_us: Option<u64>,
    last_sample_us: Option<u64>,
    interval_count: u64,
    interval_min_us: Option<u64>,
    interval_max_us: Option<u64>,
    interval_histogram: [u32; 128],
}
impl Default for AcquisitionStats {
    fn default() -> Self {
        Self {
            consumed: 0,
            missed_or_coalesced_events: 0,
            successful_samples: 0,
            motion_i2c_errors: 0,
            status_ack_i2c_errors: 0,
            first_consumed_us: None,
            last_consumed_us: None,
            first_sample_us: None,
            last_sample_us: None,
            interval_count: 0,
            interval_min_us: None,
            interval_max_us: None,
            interval_histogram: [0; 128],
        }
    }
}
impl AcquisitionStats {
    fn consumed_batch(&mut self, count: u32, now: u64) {
        self.consumed = self.consumed.saturating_add(count as u64);
        self.missed_or_coalesced_events = self
            .missed_or_coalesced_events
            .saturating_add(count.saturating_sub(1) as u64);
        self.first_consumed_us.get_or_insert(now);
        self.last_consumed_us = Some(now);
    }
    fn sample(&mut self, now: u64) {
        if let Some(previous) = self.last_sample_us {
            let interval = now.saturating_sub(previous);
            self.interval_count += 1;
            self.interval_min_us = Some(
                self.interval_min_us
                    .map_or(interval, |value| value.min(interval)),
            );
            self.interval_max_us = Some(
                self.interval_max_us
                    .map_or(interval, |value| value.max(interval)),
            );
            let bin = ((interval / 100) as usize).min(127);
            self.interval_histogram[bin] = self.interval_histogram[bin].saturating_add(1);
        }
        self.first_sample_us.get_or_insert(now);
        self.last_sample_us = Some(now);
        self.successful_samples += 1;
    }
    fn rate(count: u64, first: Option<u64>, last: Option<u64>) -> Option<f32> {
        match (count, first, last) {
            (2.., Some(first), Some(last)) if last > first => {
                Some((count - 1) as f32 * 1_000_000.0 / (last - first) as f32)
            }
            _ => None,
        }
    }
    fn interval_p50_us(&self) -> Option<u64> {
        if self.interval_count == 0 {
            return None;
        }
        let mut seen = 0u64;
        for (bin, count) in self.interval_histogram.iter().enumerate() {
            seen += *count as u64;
            if seen * 2 >= self.interval_count {
                return Some(bin as u64 * 100);
            }
        }
        None
    }
}

trait AcquisitionDevice {
    type Sample;

    fn read_motion(&mut self) -> Option<Self::Sample>;
    fn acknowledge_status(&mut self) -> bool;
}

fn service_pending_batch<D: AcquisitionDevice>(
    device: &mut D,
    stats: &mut AcquisitionStats,
    batch_count: u32,
    consumed_timestamp_us: u64,
    successful_sample_timestamp_us: impl FnOnce() -> u64,
) -> Option<D::Sample> {
    debug_assert!(batch_count > 0);
    stats.consumed_batch(batch_count, consumed_timestamp_us);
    let sample = device.read_motion();
    if sample.is_some() {
        stats.sample(successful_sample_timestamp_us());
    } else {
        stats.motion_i2c_errors = stats.motion_i2c_errors.saturating_add(1);
    }
    if !device.acknowledge_status() {
        stats.status_ack_i2c_errors = stats.status_ack_i2c_errors.saturating_add(1);
    }
    sample
}

#[cfg(target_arch = "riscv32")]
use core::cell::RefCell;
#[cfg(target_arch = "riscv32")]
use critical_section::Mutex;
/// Board INT pin input retained for the data-ready ISR (`board::INT_PIN_NAME`).
#[cfg(target_arch = "riscv32")]
static INT_INPUT: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
#[cfg(target_arch = "riscv32")]
static INT_PENDING: Mutex<RefCell<PendingEvents>> = Mutex::new(RefCell::new(PendingEvents {
    pending: 0,
    max_pending: 0,
    total: 0,
    events_unrecorded_due_to_pending_saturation: 0,
}));

#[cfg(target_arch = "riscv32")]
#[esp_hal::handler]
fn int_data_ready_handler() {
    critical_section::with(|cs| {
        let mut input = INT_INPUT.borrow_ref_mut(cs);
        if let Some(input) = input.as_mut()
            && input.is_interrupt_set()
        {
            input.clear_interrupt();
            INT_PENDING.borrow_ref_mut(cs).signal();
        }
    });
}
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

#[derive(Debug, Clone, Copy, PartialEq)]
struct VerifiedSampleTiming {
    dlpf: Dlpf,
    divider: u8,
    rate_hz: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SampleTimingError {
    DlpfWrite,
    DlpfRead,
    DlpfMismatch,
    DividerWrite,
    DividerRead,
    DividerMismatch,
    InvalidRate,
}

impl SampleTimingError {
    fn as_str(self) -> &'static str {
        match self {
            Self::DlpfWrite => "dlpf_write_failed",
            Self::DlpfRead => "dlpf_readback_failed",
            Self::DlpfMismatch => "dlpf_readback_mismatch",
            Self::DividerWrite => "divider_write_failed",
            Self::DividerRead => "divider_readback_failed",
            Self::DividerMismatch => "divider_readback_mismatch",
            Self::InvalidRate => "sample_rate_calculation_invalid",
        }
    }

    fn progress(self) -> u8 {
        match self {
            Self::DlpfWrite => 0,
            Self::DlpfRead => 1,
            Self::DlpfMismatch => 2,
            Self::DividerWrite => 3,
            Self::DividerRead => 4,
            Self::DividerMismatch => 5,
            Self::InvalidRate => 6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SampleTimingFailure {
    error: SampleTimingError,
    dlpf: Option<Dlpf>,
    divider: Option<u8>,
}

impl SampleTimingFailure {
    fn new(error: SampleTimingError, dlpf: Option<Dlpf>, divider: Option<u8>) -> Self {
        Self {
            error,
            dlpf,
            divider,
        }
    }
}

trait SampleTimingDevice {
    fn set_dlpf(&mut self, dlpf: Dlpf) -> bool;
    fn dlpf(&mut self) -> Option<Dlpf>;
    fn set_sample_rate_divider(&mut self, divider: u8) -> bool;
    fn sample_rate_divider(&mut self) -> Option<u8>;
}

#[cfg(target_arch = "riscv32")]
#[derive(Debug, Clone, Copy)]
struct AdvancedValidationResult {
    interrupt_state_confirmed_zero: bool,
    timing_registers_confirmed: bool,
}

fn configure_sample_timing(
    device: &mut impl SampleTimingDevice,
) -> Result<VerifiedSampleTiming, SampleTimingFailure> {
    if !device.set_dlpf(TARGET_DLPF) {
        return Err(SampleTimingFailure::new(
            SampleTimingError::DlpfWrite,
            None,
            None,
        ));
    }
    let dlpf = device
        .dlpf()
        .ok_or_else(|| SampleTimingFailure::new(SampleTimingError::DlpfRead, None, None))?;
    if dlpf != TARGET_DLPF {
        return Err(SampleTimingFailure::new(
            SampleTimingError::DlpfMismatch,
            Some(dlpf),
            None,
        ));
    }
    if !device.set_sample_rate_divider(TARGET_SMPLRT_DIV) {
        return Err(SampleTimingFailure::new(
            SampleTimingError::DividerWrite,
            Some(dlpf),
            None,
        ));
    }
    let divider = device
        .sample_rate_divider()
        .ok_or(SampleTimingFailure::new(
            SampleTimingError::DividerRead,
            Some(dlpf),
            None,
        ))?;
    if divider != TARGET_SMPLRT_DIV {
        return Err(SampleTimingFailure::new(
            SampleTimingError::DividerMismatch,
            Some(dlpf),
            Some(divider),
        ));
    }
    let rate_hz = dlpf.sample_rate_hz(divider);
    if !rate_hz.is_finite()
        || rate_hz <= 0.0
        || (rate_hz - EXPECTED_NOMINAL_SAMPLE_RATE_HZ).abs() > NOMINAL_RATE_COMPARISON_EPSILON_HZ
    {
        return Err(SampleTimingFailure::new(
            SampleTimingError::InvalidRate,
            Some(dlpf),
            Some(divider),
        ));
    }
    Ok(VerifiedSampleTiming {
        dlpf,
        divider,
        rate_hz,
    })
}

fn stream_startup_allowed(final_interrupts_zero: bool, timing_registers_confirmed: bool) -> bool {
    final_interrupts_zero && timing_registers_confirmed
}

fn calculated_rate_valid(rate_hz: Option<f32>) -> bool {
    rate_hz
        .map(|rate| rate.is_finite() && rate > 0.0)
        .unwrap_or(false)
}

fn calculated_rate_approx_target(rate_hz: Option<f32>) -> bool {
    calculated_rate_valid(rate_hz)
        && (rate_hz.unwrap_or_default() - EXPECTED_NOMINAL_SAMPLE_RATE_HZ).abs()
            <= NOMINAL_RATE_COMPARISON_EPSILON_HZ
}

struct HexOpt(Option<u8>);
struct U16Opt(Option<u16>);
struct U8Opt(Option<u8>);
struct DlpfOpt(Option<Dlpf>);

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

impl fmt::Display for U8Opt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "{}", value),
            None => f.write_str("unreadable"),
        }
    }
}

impl fmt::Display for DlpfOpt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "{:?}", value),
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

#[cfg(target_arch = "riscv32")]
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    println!("MPU6050 ESP32-C3 esp-hal I2C bring-up started");
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
    println!(
        "configured_nominal_sample_rate_hz=200.0 acquisition_mode=int_data_ready_events int_pin={}",
        INT_PIN_NAME
    );
    let reset_reason = esp_hal::system::reset_reason();
    // Reports whether this boot followed a watchdog-triggered reset.
    // This does not configure a watchdog or identify the running firmware image.
    let watchdog_reset = matches!(
        reset_reason,
        Some(
            esp_hal::rtc_cntl::SocResetReason::CoreMwdt0
                | esp_hal::rtc_cntl::SocResetReason::CoreMwdt1
                | esp_hal::rtc_cntl::SocResetReason::CoreRtcWdt
                | esp_hal::rtc_cntl::SocResetReason::Cpu0Mwdt0
                | esp_hal::rtc_cntl::SocResetReason::Cpu0Mwdt1
                | esp_hal::rtc_cntl::SocResetReason::Cpu0RtcWdt
                | esp_hal::rtc_cntl::SocResetReason::SysRtcWdt
                | esp_hal::rtc_cntl::SocResetReason::SysSuperWdt
        )
    );
    println!(
        "boot_reset_reason={:?} watchdog_reset_this_boot={} watchdog_count_this_boot={}",
        reset_reason, watchdog_reset, watchdog_reset as u8
    );

    let mut mpu_pins = board::take_mpu_pins!(peripherals);
    let scl_probe = Input::new(
        mpu_pins.scl.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    let sda_probe = Input::new(
        mpu_pins.sda.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    println!(
        "Pre-I2C idle check: SCL={} SDA={}",
        if scl_probe.is_high() { "HIGH" } else { "LOW" },
        if sda_probe.is_high() { "HIGH" } else { "LOW" }
    );
    drop(scl_probe);
    drop(sda_probe);

    let _ad0 = Output::new(mpu_pins.ad0, Level::Low, OutputConfig::default());
    println!(
        "AD0 driven LOW on {}; expected 7-bit address is 0x68",
        AD0_PIN_NAME
    );

    let config = I2cConfig::default()
        .with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ))
        .with_timeout(BusTimeout::Maximum)
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(50)));

    let i2c = I2c::new(peripherals.I2C0, config)
        .expect("failed to initialize I2C0")
        .with_scl(mpu_pins.scl)
        .with_sda(mpu_pins.sda);

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
    let validation = run_advanced_validation(&mut mpu, &delay, MPU_ADDR_AD0_LOW);
    let mut conditions = StartupConditions {
        diagnostics_complete: true,
        timing_confirmed: validation.timing_registers_confirmed,
        final_interrupts_zero: validation.interrupt_state_confirmed_zero,
        ..Default::default()
    };
    println!(
        "imu_interrupt_policy=explicit_opt_in sources_disabled={} timing_registers_confirmed={} status_polling=off",
        validation.interrupt_state_confirmed_zero, validation.timing_registers_confirmed
    );

    if conditions.diagnostics_complete
        && conditions.timing_confirmed
        && conditions.final_interrupts_zero
    {
        let mut io = Io::new(peripherals.IO_MUX);
        io.set_interrupt_handler(int_data_ready_handler);
        let mut input = Input::new(mpu_pins.int, InputConfig::default().with_pull(Pull::None));
        critical_section::with(|cs| {
            input.listen(Event::RisingEdge);
            INT_INPUT.borrow_ref_mut(cs).replace(input);
        });
        conditions.gpio_configured = true;
        let interrupt_conditions = configure_data_ready_startup(&mut mpu);
        conditions.stale_status_cleared = interrupt_conditions.stale_status_cleared;
        conditions.enable_success = interrupt_conditions.enable_success;
        conditions.exact_data_ready_readback = interrupt_conditions.exact_data_ready_readback;
    }
    println!(
        "data_ready_startup diagnostics_complete={} timing_confirmed={} final_interrupts_zero={} gpio_configured={} stale_status_cleared={} enable_success={} exact_data_ready_readback={} acquisition_started={}",
        conditions.diagnostics_complete,
        conditions.timing_confirmed,
        conditions.final_interrupts_zero,
        conditions.gpio_configured,
        conditions.stale_status_cleared,
        conditions.enable_success,
        conditions.exact_data_ready_readback,
        conditions.allows_acquisition()
    );
    if conditions.allows_acquisition() {
        let mut stats = AcquisitionStats::default();
        let acquisition_start_us = Instant::now().duration_since_epoch().as_micros() as u64;
        let mut last_summary_us = acquisition_start_us;
        loop {
            let batch_count =
                critical_section::with(|cs| INT_PENDING.borrow_ref_mut(cs).take_all());
            if batch_count != 0 {
                let consumed_timestamp_us =
                    Instant::now().duration_since_epoch().as_micros() as u64;
                let sample_for_output = service_pending_batch(
                    &mut mpu,
                    &mut stats,
                    batch_count,
                    consumed_timestamp_us,
                    || Instant::now().duration_since_epoch().as_micros() as u64,
                );
                if let Some(raw) = sample_for_output
                    && stats.successful_samples <= RAW_EXAMPLE_LIMIT
                {
                    let sample_timestamp_us = stats.last_sample_us.unwrap_or_default();
                    println!(
                        "RAW consumed_events={} accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) consumed_timestamp_us={} sample_timestamp_us={}",
                        stats.consumed,
                        raw.accel[0],
                        raw.accel[1],
                        raw.accel[2],
                        raw.temp,
                        raw.gyro[0],
                        raw.gyro[1],
                        raw.gyro[2],
                        consumed_timestamp_us,
                        sample_timestamp_us
                    );
                }
            }
            let now = Instant::now().duration_since_epoch().as_micros() as u64;
            if now.saturating_sub(last_summary_us) >= SUMMARY_PERIOD_US {
                log_acquisition_summary(&stats, acquisition_start_us, now);
                last_summary_us = now;
            }
        }
    }

    println!("data_ready_acquisition_blocked");
    loop {
        delay.delay_millis(BLOCKED_IDLE_DELAY_MS);
    }
}

#[cfg(target_arch = "riscv32")]
type BoardMpu<'a> = Mpu6050<I2c<'a, esp_hal::Blocking>>;

#[cfg(target_arch = "riscv32")]
impl DataReadyStartupDevice for BoardMpu<'_> {
    fn clear_int_status(&mut self) -> bool {
        self.int_status().is_ok()
    }

    fn enable_data_ready(&mut self) -> bool {
        self.enable_data_ready_interrupt().is_ok()
    }

    fn only_data_ready_enabled(&mut self) -> Option<bool> {
        self.interrupt_enable()
            .ok()
            .map(|value| value.only_data_ready())
    }
}

#[cfg(target_arch = "riscv32")]
impl AcquisitionDevice for BoardMpu<'_> {
    type Sample = RawAccelGyroTemp;

    fn read_motion(&mut self) -> Option<Self::Sample> {
        self.read_raw_accel_gyro_temp().ok()
    }

    fn acknowledge_status(&mut self) -> bool {
        self.int_status().is_ok()
    }
}

#[cfg(target_arch = "riscv32")]
fn log_acquisition_summary(stats: &AcquisitionStats, acquisition_start_us: u64, now_us: u64) {
    let pending = critical_section::with(|cs| *INT_PENDING.borrow_ref(cs));
    println!(
        "acquisition_summary configured_nominal_rate_hz=200.0 measured_isr_event_rate_since_start_hz={:?} measured_consumed_event_rate_hz={:?} measured_sample_rate_hz={:?} isr_data_ready_total={} consumed_events={} missed_or_coalesced_events={} successful_samples={} current_pending={} max_pending={} events_unrecorded_due_to_pending_saturation={} motion_i2c_errors={} status_ack_i2c_errors={} total_i2c_errors={} first_consumed_us={:?} last_consumed_us={:?} first_sample_us={:?} last_sample_us={:?} successful_sample_read_completion_intervals={} successful_sample_interval_min_us={:?} successful_sample_interval_p50_us_approx_100us={:?} successful_sample_interval_max_us={:?}",
        if now_us > acquisition_start_us {
            Some(pending.total as f32 * 1_000_000.0 / (now_us - acquisition_start_us) as f32)
        } else {
            None
        },
        AcquisitionStats::rate(
            stats.consumed,
            stats.first_consumed_us,
            stats.last_consumed_us
        ),
        AcquisitionStats::rate(
            stats.successful_samples,
            stats.first_sample_us,
            stats.last_sample_us
        ),
        pending.total,
        stats.consumed,
        stats.missed_or_coalesced_events,
        stats.successful_samples,
        pending.pending,
        pending.max_pending,
        pending.events_unrecorded_due_to_pending_saturation,
        stats.motion_i2c_errors,
        stats.status_ack_i2c_errors,
        stats
            .motion_i2c_errors
            .saturating_add(stats.status_ack_i2c_errors),
        stats.first_consumed_us,
        stats.last_consumed_us,
        stats.first_sample_us,
        stats.last_sample_us,
        stats.interval_count,
        stats.interval_min_us,
        stats.interval_p50_us(),
        stats.interval_max_us
    );
}

#[cfg(target_arch = "riscv32")]
impl SampleTimingDevice for BoardMpu<'_> {
    fn set_dlpf(&mut self, dlpf: Dlpf) -> bool {
        Mpu6050::set_dlpf(self, dlpf).is_ok()
    }
    fn dlpf(&mut self) -> Option<Dlpf> {
        Mpu6050::dlpf(self).ok()
    }
    fn set_sample_rate_divider(&mut self, divider: u8) -> bool {
        Mpu6050::set_sample_rate_divider(self, divider).is_ok()
    }
    fn sample_rate_divider(&mut self) -> Option<u8> {
        Mpu6050::sample_rate_divider(self).ok()
    }
}

#[cfg(target_arch = "riscv32")]
fn run_advanced_validation(
    mpu: &mut BoardMpu<'_>,
    delay: &Delay,
    address: u8,
) -> AdvancedValidationResult {
    println!("advanced_validation_begin");
    let (initial_zero, timing_registers_confirmed) = reset_wake_configure(mpu, delay, address);
    let interrupt_state_confirmed_zero = if initial_zero && timing_registers_confirmed {
        validate_scale_registers(mpu, address);
        validate_self_test_coarse(mpu, delay, address);
        validate_fifo_timing(mpu, delay, address);
        validate_int_status(mpu, delay, address)
    } else {
        initial_zero
    };
    println!("advanced_validation_end");
    AdvancedValidationResult {
        interrupt_state_confirmed_zero,
        timing_registers_confirmed,
    }
}

#[cfg(target_arch = "riscv32")]
fn reset_wake_configure(mpu: &mut BoardMpu<'_>, delay: &Delay, _address: u8) -> (bool, bool) {
    println!("advanced reset_wake_begin");
    let reset_ok = mpu.reset().is_ok();
    delay.delay_millis(100);
    let wake_ok = mpu.wake().is_ok();
    delay.delay_millis(20);
    let int_disable_write_ok = mpu.disable_all_interrupts().is_ok();
    let interrupt_enable = mpu.interrupt_enable().ok();
    let int_readback_ok = interrupt_enable.is_some();
    let int_data_ready = interrupt_enable
        .map(|value| value.data_ready())
        .unwrap_or(false);
    let int_fifo_overflow = interrupt_enable
        .map(|value| value.fifo_overflow())
        .unwrap_or(false);
    let int_none_enabled = interrupt_enable
        .map(|value| value.none_enabled())
        .unwrap_or(false);
    let interrupt_enable_confirmed_zero = int_readback_ok && int_none_enabled;
    let configuration_prerequisites_confirmed =
        reset_ok && wake_ok && interrupt_enable_confirmed_zero;
    let timing = configuration_prerequisites_confirmed.then(|| configure_sample_timing(mpu));
    let timing_registers_confirmed = matches!(timing, Some(Ok(_)));
    let timing_error = timing
        .as_ref()
        .and_then(|result| result.as_ref().err())
        .copied();
    let dlpf = timing.as_ref().and_then(|result| match result {
        Ok(value) => Some(value.dlpf),
        Err(failure) => failure.dlpf,
    });
    let divider = timing.as_ref().and_then(|result| match result {
        Ok(value) => Some(value.divider),
        Err(failure) => failure.divider,
    });
    let rate_hz = dlpf
        .zip(divider)
        .map(|(dlpf, divider)| dlpf.sample_rate_hz(divider));
    let calculated_rate_valid = calculated_rate_valid(rate_hz);
    let calculated_rate_approx_target = calculated_rate_approx_target(rate_hz);
    let timing_error = timing_error.map(|failure| failure.error);
    let progress = timing_error.map_or(7, SampleTimingError::progress);
    let dlpf_write_attempted = timing.is_some();
    let dlpf_write_ok = dlpf_write_attempted && progress >= 1;
    let dlpf_read_attempted = dlpf_write_ok;
    let dlpf_read_ok = dlpf_read_attempted && progress >= 2;
    let dlpf_match = dlpf_read_ok && progress >= 3;
    let divider_write_attempted = dlpf_match;
    let divider_write_ok = divider_write_attempted && progress >= 4;
    let divider_read_attempted = divider_write_ok;
    let divider_read_ok = divider_read_attempted && progress >= 5;
    let divider_match = divider_read_ok && progress >= 6;
    let accel_cfg_ok = timing_registers_confirmed && mpu.set_accel_range(AccelRange::G2).is_ok();
    let gyro_cfg_ok = timing_registers_confirmed && mpu.set_gyro_range(GyroRange::Dps250).is_ok();
    let failure_stage =
        timing_error
            .map(SampleTimingError::as_str)
            .unwrap_or(if timing.is_some() {
                "none"
            } else {
                "dlpf_write_not_attempted"
            });
    println!(
        "advanced reset_wake reset_ok={} wake_ok={} int_disable_write_ok={} int_readback_ok={} int_data_ready={} int_fifo_overflow={} int_none_enabled={} int_confirmed_zero={} configuration_prerequisites_confirmed={} dlpf_write_attempted={} dlpf_write_ok={} dlpf_readback_attempted={} dlpf_readback_ok={} dlpf_match={} dlpf={} sample_divider_write_attempted={} sample_divider_write_ok={} sample_divider_readback_attempted={} sample_divider_readback_ok={} sample_divider_match={} sample_rate_divider={} calculated_sample_rate_hz={:.1} calculated_sample_rate_valid={} calculated_sample_rate_approx_target={} timing_failure_stage={} timing_registers_confirmed={} accel_cfg_ok={} gyro_cfg_ok={}",
        reset_ok,
        wake_ok,
        int_disable_write_ok,
        int_readback_ok,
        int_data_ready,
        int_fifo_overflow,
        int_none_enabled,
        interrupt_enable_confirmed_zero,
        configuration_prerequisites_confirmed,
        dlpf_write_attempted,
        dlpf_write_ok,
        dlpf_read_attempted,
        dlpf_read_ok,
        dlpf_match,
        DlpfOpt(dlpf),
        divider_write_attempted,
        divider_write_ok,
        divider_read_attempted,
        divider_read_ok,
        divider_match,
        U8Opt(divider),
        rate_hz.unwrap_or(f32::NAN),
        calculated_rate_valid,
        calculated_rate_approx_target,
        failure_stage,
        timing_registers_confirmed,
        accel_cfg_ok,
        gyro_cfg_ok
    );
    if let Some(Ok(verified_timing)) = timing {
        println!(
            "configured_nominal_sample_rate_hz={:.1}",
            verified_timing.rate_hz
        );
    }
    println!("advanced reset_wake_end");
    (interrupt_enable_confirmed_zero, timing_registers_confirmed)
}

#[cfg(target_arch = "riscv32")]
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

#[cfg(target_arch = "riscv32")]
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

#[cfg(target_arch = "riscv32")]
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

#[cfg(target_arch = "riscv32")]
fn validate_int_status(mpu: &mut BoardMpu<'_>, delay: &Delay, _address: u8) -> bool {
    println!("advanced int_status_begin");
    let pre_disable_write_ok = mpu.disable_all_interrupts().is_ok();
    let pre_enable = mpu.interrupt_enable().ok();
    let pre_readback_ok = pre_enable.is_some();
    let pre_data_ready = pre_enable.map(|value| value.data_ready()).unwrap_or(false);
    let pre_fifo_overflow = pre_enable
        .map(|value| value.fifo_overflow())
        .unwrap_or(false);
    let pre_none_enabled = pre_enable
        .map(|value| value.none_enabled())
        .unwrap_or(false);
    let pre_confirmed_zero = pre_readback_ok && pre_none_enabled;
    let enable_attempted = pre_confirmed_zero;
    let enable_ok = if enable_attempted {
        mpu.enable_data_ready_interrupt().is_ok()
    } else {
        false
    };
    let status = if enable_ok {
        delay.delay_millis(10);
        mpu.int_status().ok()
    } else {
        None
    };
    let data_ready = status.map(|v| v.data_ready()).unwrap_or(false);
    let fifo_overflow = status.map(|v| v.fifo_overflow()).unwrap_or(false);
    let final_disable_write_ok = mpu.disable_all_interrupts().is_ok();
    let final_enable = mpu.interrupt_enable().ok();
    let final_readback_ok = final_enable.is_some();
    let final_data_ready = final_enable
        .map(|value| value.data_ready())
        .unwrap_or(false);
    let final_fifo_overflow = final_enable
        .map(|value| value.fifo_overflow())
        .unwrap_or(false);
    let final_none_enabled = final_enable
        .map(|value| value.none_enabled())
        .unwrap_or(false);
    let final_confirmed_zero = final_readback_ok && final_none_enabled;
    println!(
        "advanced int_status pre_disable_write_ok={} pre_readback_ok={} pre_data_ready={} pre_fifo_overflow={} pre_none_enabled={} pre_confirmed_zero={} enable_attempted={} enable_ok={} status_read_ok={} data_ready={} fifo_overflow={} final_disable_write_ok={} final_readback_ok={} final_data_ready={} final_fifo_overflow={} final_none_enabled={} final_confirmed_zero={}",
        pre_disable_write_ok,
        pre_readback_ok,
        pre_data_ready,
        pre_fifo_overflow,
        pre_none_enabled,
        pre_confirmed_zero,
        enable_attempted,
        enable_ok,
        status.is_some(),
        data_ready,
        fifo_overflow,
        final_disable_write_ok,
        final_readback_ok,
        final_data_ready,
        final_fifo_overflow,
        final_none_enabled,
        final_confirmed_zero
    );
    println!("advanced int_status_end");
    final_confirmed_zero
}

#[cfg(target_arch = "riscv32")]
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

#[cfg(target_arch = "riscv32")]
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

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum TimingCall {
        SetDlpf,
        ReadDlpf,
        SetDivider,
        ReadDivider,
    }
    struct FakeTimingDevice {
        dlpf_write_ok: bool,
        dlpf_readback: Option<Dlpf>,
        divider_write_ok: bool,
        divider_readback: Option<u8>,
        calls: Vec<TimingCall>,
    }

    impl SampleTimingDevice for FakeTimingDevice {
        fn set_dlpf(&mut self, value: Dlpf) -> bool {
            assert_eq!(value, TARGET_DLPF);
            self.calls.push(TimingCall::SetDlpf);
            self.dlpf_write_ok
        }
        fn dlpf(&mut self) -> Option<Dlpf> {
            self.calls.push(TimingCall::ReadDlpf);
            self.dlpf_readback
        }
        fn set_sample_rate_divider(&mut self, value: u8) -> bool {
            assert_eq!(value, TARGET_SMPLRT_DIV);
            self.calls.push(TimingCall::SetDivider);
            self.divider_write_ok
        }
        fn sample_rate_divider(&mut self) -> Option<u8> {
            self.calls.push(TimingCall::ReadDivider);
            self.divider_readback
        }
    }

    fn assert_failure(
        device: FakeTimingDevice,
        error: SampleTimingError,
        readbacks: (Option<Dlpf>, Option<u8>),
        calls: Vec<TimingCall>,
    ) {
        let mut device = device;
        let failure = configure_sample_timing(&mut device).unwrap_err();
        assert_eq!(
            (failure.error, failure.dlpf, failure.divider),
            (error, readbacks.0, readbacks.1)
        );
        assert_eq!(device.calls, calls);
    }
    #[test]
    fn target_timing_period_and_ranges_are_unchanged() {
        assert_eq!(TARGET_DLPF, Dlpf::Cfg2);
        assert_eq!(TARGET_SMPLRT_DIV, 4);
        assert!(
            (TARGET_DLPF.sample_rate_hz(TARGET_SMPLRT_DIV) - EXPECTED_NOMINAL_SAMPLE_RATE_HZ).abs()
                <= NOMINAL_RATE_COMPARISON_EPSILON_HZ
        );
        assert_eq!(accel_range_from_setting(0), AccelRange::G2);
        assert_eq!(gyro_range_from_setting(0), GyroRange::Dps250);
        assert!(calculated_rate_valid(Some(200.0)));
        assert!(calculated_rate_approx_target(Some(200.0)));
        assert!(!calculated_rate_valid(Some(f32::NAN)));
        assert!(!calculated_rate_valid(Some(f32::INFINITY)));
        assert!(!calculated_rate_valid(Some(0.0)));
        assert!(!calculated_rate_valid(None));
        assert!(!calculated_rate_approx_target(Some(199.0)));
    }
    #[test]
    fn configure_sample_timing_succeeds_in_order() {
        let mut device = FakeTimingDevice {
            dlpf_write_ok: true,
            dlpf_readback: Some(Dlpf::Cfg2),
            divider_write_ok: true,
            divider_readback: Some(4),
            calls: Vec::new(),
        };
        assert_eq!(
            configure_sample_timing(&mut device),
            Ok(VerifiedSampleTiming {
                dlpf: Dlpf::Cfg2,
                divider: 4,
                rate_hz: 200.0
            })
        );
        assert_eq!(
            device.calls,
            vec![
                TimingCall::SetDlpf,
                TimingCall::ReadDlpf,
                TimingCall::SetDivider,
                TimingCall::ReadDivider
            ]
        );
    }
    #[test]
    fn configure_sample_timing_reports_operation_failures_and_mismatches() {
        let cases = [
            (
                false,
                None,
                false,
                None,
                SampleTimingError::DlpfWrite,
                (None, None),
                vec![TimingCall::SetDlpf],
            ),
            (
                true,
                None,
                false,
                None,
                SampleTimingError::DlpfRead,
                (None, None),
                vec![TimingCall::SetDlpf, TimingCall::ReadDlpf],
            ),
            (
                true,
                Some(Dlpf::Cfg3),
                false,
                None,
                SampleTimingError::DlpfMismatch,
                (Some(Dlpf::Cfg3), None),
                vec![TimingCall::SetDlpf, TimingCall::ReadDlpf],
            ),
            (
                true,
                Some(Dlpf::Cfg2),
                false,
                None,
                SampleTimingError::DividerWrite,
                (Some(Dlpf::Cfg2), None),
                vec![
                    TimingCall::SetDlpf,
                    TimingCall::ReadDlpf,
                    TimingCall::SetDivider,
                ],
            ),
            (
                true,
                Some(Dlpf::Cfg2),
                true,
                None,
                SampleTimingError::DividerRead,
                (Some(Dlpf::Cfg2), None),
                vec![
                    TimingCall::SetDlpf,
                    TimingCall::ReadDlpf,
                    TimingCall::SetDivider,
                    TimingCall::ReadDivider,
                ],
            ),
            (
                true,
                Some(Dlpf::Cfg2),
                true,
                Some(5),
                SampleTimingError::DividerMismatch,
                (Some(Dlpf::Cfg2), Some(5)),
                vec![
                    TimingCall::SetDlpf,
                    TimingCall::ReadDlpf,
                    TimingCall::SetDivider,
                    TimingCall::ReadDivider,
                ],
            ),
        ];
        for (
            dlpf_write_ok,
            dlpf_readback,
            divider_write_ok,
            divider_readback,
            error,
            readbacks,
            calls,
        ) in cases
        {
            assert_failure(
                FakeTimingDevice {
                    dlpf_write_ok,
                    dlpf_readback,
                    divider_write_ok,
                    divider_readback,
                    calls: Vec::new(),
                },
                error,
                readbacks,
                calls,
            );
        }
    }
    #[test]
    fn stream_startup_requires_interrupt_zero_and_verified_timing() {
        let ok: Option<Result<VerifiedSampleTiming, SampleTimingFailure>> =
            Some(Ok(VerifiedSampleTiming {
                dlpf: Dlpf::Cfg2,
                divider: 4,
                rate_hz: 200.0,
            }));
        let failed: Option<Result<VerifiedSampleTiming, SampleTimingFailure>> = Some(Err(
            SampleTimingFailure::new(SampleTimingError::DlpfWrite, None, None),
        ));
        let skipped: Option<Result<VerifiedSampleTiming, SampleTimingFailure>> = None;
        assert!(!stream_startup_allowed(false, matches!(ok, Some(Ok(_)))));
        assert!(!stream_startup_allowed(true, matches!(failed, Some(Ok(_)))));
        assert!(!stream_startup_allowed(
            true,
            matches!(skipped, Some(Ok(_)))
        ));
        assert!(stream_startup_allowed(true, matches!(ok, Some(Ok(_)))));
    }

    #[test]
    fn acquisition_requires_diagnostics_and_timing() {
        let complete = StartupConditions {
            diagnostics_complete: true,
            timing_confirmed: true,
            final_interrupts_zero: true,
            gpio_configured: true,
            stale_status_cleared: true,
            enable_success: true,
            exact_data_ready_readback: true,
        };
        assert!(complete.allows_acquisition());
        let mut missing = complete;
        missing.diagnostics_complete = false;
        assert!(!missing.allows_acquisition());
        missing = complete;
        missing.timing_confirmed = false;
        assert!(!missing.allows_acquisition());
    }

    #[test]
    fn acquisition_requires_gpio_and_exact_readback_and_enable() {
        let complete = StartupConditions {
            diagnostics_complete: true,
            timing_confirmed: true,
            final_interrupts_zero: true,
            gpio_configured: true,
            stale_status_cleared: true,
            enable_success: true,
            exact_data_ready_readback: true,
        };
        for condition in [
            StartupConditions {
                gpio_configured: false,
                ..complete
            },
            StartupConditions {
                enable_success: false,
                ..complete
            },
            StartupConditions {
                exact_data_ready_readback: false,
                ..complete
            },
        ] {
            assert!(!condition.allows_acquisition());
        }
    }

    #[test]
    fn pending_events_are_not_collapsed() {
        let mut pending = PendingEvents::default();
        pending.signal();
        pending.signal();
        pending.signal();
        assert_eq!(
            (pending.total, pending.pending, pending.max_pending),
            (3, 3, 3)
        );
        assert_eq!(pending.take_all(), 3);
        assert_eq!(pending.pending, 0);
        assert_eq!(pending.take_all(), 0);
    }

    #[test]
    fn pending_events_saturate_and_record_unrepresented_event() {
        let mut pending = PendingEvents {
            pending: u32::MAX,
            max_pending: u32::MAX,
            total: 9,
            events_unrecorded_due_to_pending_saturation: 0,
        };
        pending.signal();
        assert_eq!(pending.pending, u32::MAX);
        assert_eq!(pending.total, 10);
        assert_eq!(pending.events_unrecorded_due_to_pending_saturation, 1);
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum StartupCall {
        Clear,
        Enable,
        Readback,
    }

    struct FakeDataReadyDevice {
        clear: bool,
        enable: bool,
        readback: Option<bool>,
        calls: Vec<StartupCall>,
    }

    impl DataReadyStartupDevice for FakeDataReadyDevice {
        fn clear_int_status(&mut self) -> bool {
            self.calls.push(StartupCall::Clear);
            self.clear
        }
        fn enable_data_ready(&mut self) -> bool {
            self.calls.push(StartupCall::Enable);
            self.enable
        }
        fn only_data_ready_enabled(&mut self) -> Option<bool> {
            self.calls.push(StartupCall::Readback);
            self.readback
        }
    }

    #[test]
    fn data_ready_startup_stops_after_failed_stale_clear() {
        let mut device = FakeDataReadyDevice {
            clear: false,
            enable: true,
            readback: Some(true),
            calls: Vec::new(),
        };
        let conditions = configure_data_ready_startup(&mut device);
        assert!(!conditions.stale_status_cleared);
        assert_eq!(device.calls, vec![StartupCall::Clear]);
    }

    #[test]
    fn data_ready_startup_stops_after_failed_enable() {
        let mut device = FakeDataReadyDevice {
            clear: true,
            enable: false,
            readback: Some(true),
            calls: Vec::new(),
        };
        let conditions = configure_data_ready_startup(&mut device);
        assert!(!conditions.enable_success);
        assert_eq!(device.calls, vec![StartupCall::Clear, StartupCall::Enable]);
    }

    #[test]
    fn data_ready_startup_requires_readable_exact_readback() {
        for readback in [None, Some(false)] {
            let mut device = FakeDataReadyDevice {
                clear: true,
                enable: true,
                readback,
                calls: Vec::new(),
            };
            let conditions = configure_data_ready_startup(&mut device);
            assert!(!conditions.exact_data_ready_readback);
            assert_eq!(
                device.calls,
                vec![
                    StartupCall::Clear,
                    StartupCall::Enable,
                    StartupCall::Readback
                ]
            );
        }
    }

    #[test]
    fn data_ready_startup_orders_clear_enable_then_exact_readback() {
        let mut device = FakeDataReadyDevice {
            clear: true,
            enable: true,
            readback: Some(true),
            calls: Vec::new(),
        };
        let conditions = configure_data_ready_startup(&mut device);
        assert!(
            conditions.stale_status_cleared
                && conditions.enable_success
                && conditions.exact_data_ready_readback
        );
        assert_eq!(
            device.calls,
            vec![
                StartupCall::Clear,
                StartupCall::Enable,
                StartupCall::Readback
            ]
        );
    }

    struct FakeAcquisitionDevice {
        motion_reads: u32,
        status_acknowledgments: u32,
    }

    impl AcquisitionDevice for FakeAcquisitionDevice {
        type Sample = u8;

        fn read_motion(&mut self) -> Option<Self::Sample> {
            self.motion_reads += 1;
            Some(42)
        }

        fn acknowledge_status(&mut self) -> bool {
            self.status_acknowledgments += 1;
            true
        }
    }

    #[test]
    fn backlog_batch_reads_one_frame_and_counts_coalesced_events() {
        let mut device = FakeAcquisitionDevice {
            motion_reads: 0,
            status_acknowledgments: 0,
        };
        let mut stats = AcquisitionStats::default();

        let sample = service_pending_batch(&mut device, &mut stats, 3, 100, || 110);

        assert_eq!(sample, Some(42));
        assert_eq!(device.motion_reads, 1);
        assert_eq!(device.status_acknowledgments, 1);
        assert_eq!(stats.consumed, 3);
        assert_eq!(stats.successful_samples, 1);
        assert_eq!(stats.missed_or_coalesced_events, 2);
    }

    #[test]
    fn zero_and_one_sample_statistics_are_safe() {
        let mut stats = AcquisitionStats::default();
        assert_eq!(AcquisitionStats::rate(0, None, None), None);
        assert_eq!(stats.interval_p50_us(), None);
        stats.consumed_batch(1, 10);
        stats.sample(10);
        assert_eq!(
            AcquisitionStats::rate(1, stats.first_sample_us, stats.last_sample_us),
            None
        );
        assert_eq!(stats.interval_count, 0);
        assert_eq!(stats.interval_p50_us(), None);
    }
}
