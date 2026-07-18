#[cfg(target_arch = "riscv32")]
use crate::board::{AD0_PIN_NAME, BOARD_NAME, GND_PIN_NAME, INT_PIN_NAME, SCL_PIN_NAME, SDA_PIN_NAME, VCC_PIN_NAME, XCL_PIN_NAME, XDA_PIN_NAME};
#[cfg(target_arch = "riscv32")]
use esp_hal::{gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull}, peripherals::{GPIO0, GPIO1, GPIO5}};
#[cfg(target_arch = "riscv32")]
use esp_println::println;

#[cfg(target_arch = "riscv32")]
pub(crate) fn report_bringup_banner() {
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
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn report_reset_reason() {
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
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn probe_pre_i2c_idle(scl_pin: &mut GPIO0<'static>, sda_pin: &mut GPIO1<'static>) {
    let scl_probe = Input::new(
        scl_pin.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    let sda_probe = Input::new(
        sda_pin.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    println!(
        "Pre-I2C idle check: SCL={} SDA={}",
        if scl_probe.is_high() { "HIGH" } else { "LOW" },
        if sda_probe.is_high() { "HIGH" } else { "LOW" }
    );
    drop(scl_probe);
    drop(sda_probe);
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn drive_ad0_low(ad0_pin: GPIO5<'static>) -> Output<'static> {
    let ad0 = Output::new(ad0_pin, Level::Low, OutputConfig::default());
    println!(
        "AD0 driven LOW on {}; expected 7-bit address is 0x68",
        AD0_PIN_NAME
    );
    ad0
}
