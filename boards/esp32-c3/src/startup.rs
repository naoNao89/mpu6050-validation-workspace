use core::fmt;
#[cfg(target_arch = "riscv32")]
use esp_hal::{delay::Delay, i2c::master::I2c};
#[cfg(target_arch = "riscv32")]
use esp_println::println;
use mpu6050_driver::{AccelRange, Dlpf, GyroRange, Identity};
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::{Address, Mpu6050};

pub(crate) const MPU_ADDR_AD0_LOW: u8 = 0x68;
pub(crate) const MPU_ADDR_AD0_HIGH: u8 = 0x69;

pub(crate) const FIFO_ACCEL_GYRO_FRAME_BYTES: u16 = 12;
pub(crate) const BLOCKED_IDLE_DELAY_MS: u32 = 100;
pub(crate) const TARGET_DLPF: Dlpf = Dlpf::Cfg2;
// Exact value written to the MPU SMPLRT_DIV register.
pub(crate) const TARGET_SMPLRT_DIV: u8 = 4;
// Nominal rate derived from the configured registers:
// 1_000 Hz / (1 + SMPLRT_DIV=4) = 200 Hz.
// This does not verify the physical sensor cadence or host read rate.
pub(crate) const EXPECTED_NOMINAL_SAMPLE_RATE_HZ: f32 = 200.0;
// Floating-point epsilon for the nominal-rate calculation, not a hardware tolerance.
pub(crate) const NOMINAL_RATE_COMPARISON_EPSILON_HZ: f32 = 0.01;

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct StartupConditions {
    pub(crate) diagnostics_complete: bool,
    pub(crate) timing_confirmed: bool,
    pub(crate) final_interrupts_zero: bool,
    pub(crate) gpio_configured: bool,
    pub(crate) stale_status_cleared: bool,
    pub(crate) enable_success: bool,
    pub(crate) exact_data_ready_readback: bool,
}

pub(crate) trait DataReadyStartupDevice {
    fn clear_int_status(&mut self) -> bool;
    fn enable_data_ready(&mut self) -> bool;
    fn only_data_ready_enabled(&mut self) -> Option<bool>;
}

pub(crate) fn configure_data_ready_startup(
    device: &mut impl DataReadyStartupDevice,
) -> StartupConditions {
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
    pub(crate) const fn allows_acquisition(self) -> bool {
        self.diagnostics_complete
            && self.timing_confirmed
            && self.final_interrupts_zero
            && self.gpio_configured
            && self.stale_status_cleared
            && self.enable_success
            && self.exact_data_ready_readback
    }
}
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
pub(crate) struct ProbeResult {
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
pub(crate) struct VerifiedSampleTiming {
    dlpf: Dlpf,
    divider: u8,
    rate_hz: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SampleTimingError {
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
pub(crate) struct SampleTimingFailure {
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

pub(crate) trait SampleTimingDevice {
    fn set_dlpf(&mut self, dlpf: Dlpf) -> bool;
    fn dlpf(&mut self) -> Option<Dlpf>;
    fn set_sample_rate_divider(&mut self, divider: u8) -> bool;
    fn sample_rate_divider(&mut self) -> Option<u8>;
}

#[cfg(target_arch = "riscv32")]
#[derive(Debug, Clone, Copy)]
pub(crate) struct AdvancedValidationResult {
    pub(crate) interrupt_state_confirmed_zero: bool,
    pub(crate) timing_registers_confirmed: bool,
}

pub(crate) fn configure_sample_timing(
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

pub(crate) fn stream_startup_allowed(
    final_interrupts_zero: bool,
    timing_registers_confirmed: bool,
) -> bool {
    final_interrupts_zero && timing_registers_confirmed
}

pub(crate) fn calculated_rate_valid(rate_hz: Option<f32>) -> bool {
    rate_hz
        .map(|rate| rate.is_finite() && rate > 0.0)
        .unwrap_or(false)
}

pub(crate) fn calculated_rate_approx_target(rate_hz: Option<f32>) -> bool {
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

pub(crate) trait IdentityDescription {
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
pub(crate) type BoardMpu<'a> = Mpu6050<I2c<'a, esp_hal::Blocking>>;

#[cfg(target_arch = "riscv32")]
pub(crate) fn initialize_sensor<'a>(
    i2c: I2c<'a, esp_hal::Blocking>,
    delay: &Delay,
) -> (BoardMpu<'a>, StartupConditions) {
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
    let validation = run_advanced_validation(&mut mpu, delay, MPU_ADDR_AD0_LOW);
    let conditions = StartupConditions {
        diagnostics_complete: true,
        timing_confirmed: validation.timing_registers_confirmed,
        final_interrupts_zero: validation.interrupt_state_confirmed_zero,
        ..Default::default()
    };
    println!(
        "imu_interrupt_policy=explicit_opt_in sources_disabled={} timing_registers_confirmed={} status_polling=off",
        validation.interrupt_state_confirmed_zero, validation.timing_registers_confirmed
    );
    (mpu, conditions)
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn log_data_ready_startup(conditions: &StartupConditions) {
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
}

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
pub(crate) fn run_advanced_validation(
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
pub(crate) fn scan_candidates(i2c: I2c<'_, esp_hal::Blocking>) -> I2c<'_, esp_hal::Blocking> {
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
pub(crate) fn probe_imu_driver(mpu: &mut BoardMpu<'_>, address: u8) -> ProbeResult {
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
pub(crate) fn log_verification_summary(probe: ProbeResult) {
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
}
