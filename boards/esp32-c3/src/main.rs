#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]
#![allow(dead_code)]

#[cfg(target_arch = "riscv32")]
esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(not(target_arch = "riscv32"))]
fn main() {}

mod acquisition;
mod board;
mod boot;
mod startup;
mod telemetry;

#[cfg(target_arch = "riscv32")]
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::{
    delay::Delay,
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Duration, Instant, Rate},
};
#[cfg(target_arch = "riscv32")]
use esp_println::println;

#[cfg(target_arch = "riscv32")]
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    boot::inspect_boot();

    let mut mpu_pins = board::take_mpu_pins!(peripherals);
    boot::probe_pre_i2c_idle(&mut mpu_pins.scl, &mut mpu_pins.sda);
    let _ad0 = boot::drive_ad0_low(mpu_pins.ad0);

    let config = I2cConfig::default()
        .with_frequency(Rate::from_khz(board::I2C_FREQUENCY_KHZ))
        .with_timeout(BusTimeout::Maximum)
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(50)));
    let i2c = I2c::new(peripherals.I2C0, config)
        .expect("failed to initialize I2C0")
        .with_scl(mpu_pins.scl)
        .with_sda(mpu_pins.sda);
    println!(
        "{} initialized at {} kHz: SCL={} SDA={}",
        board::I2C_BUS_NAME,
        board::I2C_FREQUENCY_KHZ,
        board::SCL_PIN_NAME,
        board::SDA_PIN_NAME
    );

    let (mut mpu, mut conditions) = startup::initialize_sensor(i2c, &delay);

    if conditions.diagnostics_complete
        && conditions.timing_confirmed
        && conditions.final_interrupts_zero
    {
        acquisition::arm(peripherals.IO_MUX, mpu_pins.int);
        conditions.gpio_configured = true;
        let interrupt_conditions = startup::configure_data_ready_startup(&mut mpu);
        conditions.stale_status_cleared = interrupt_conditions.stale_status_cleared;
        conditions.enable_success = interrupt_conditions.enable_success;
        conditions.exact_data_ready_readback = interrupt_conditions.exact_data_ready_readback;
    }
    startup::log_data_ready_startup(&conditions);

    if conditions.allows_acquisition() {
        let mut stats = acquisition::AcquisitionStats::default();
        let acquisition_start_us = Instant::now().duration_since_epoch().as_micros() as u64;
        let mut last_summary_us = acquisition_start_us;
        loop {
            if let Some(outcome) = acquisition::drain_pending(&mut mpu, &mut stats) {
                if let Some(raw) = outcome.sample {
                    telemetry::maybe_log_raw_example(&stats, &raw, outcome.consumed_timestamp_us);
                }
            }
            let now = Instant::now().duration_since_epoch().as_micros() as u64;
            if now.saturating_sub(last_summary_us) >= telemetry::SUMMARY_PERIOD_US {
                telemetry::log_acquisition_summary(&stats, acquisition_start_us, now);
                last_summary_us = now;
            }
        }
    }

    println!("data_ready_acquisition_blocked");
    loop {
        delay.delay_millis(startup::BLOCKED_IDLE_DELAY_MS);
    }
}
