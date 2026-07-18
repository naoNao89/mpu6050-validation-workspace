#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]
#![allow(dead_code)]

#[cfg(target_arch = "riscv32")]
esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(not(target_arch = "riscv32"))]
fn main() {}

mod board;
mod boot;
mod startup;
mod acquisition;
mod telemetry;

#[cfg(target_arch = "riscv32")]
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::{delay::Delay, gpio::{Event, Input, InputConfig, Io, Pull}, i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout}, main, time::{Duration, Instant, Rate}};
#[cfg(target_arch = "riscv32")]
use esp_println::println;
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::{Address, Mpu6050};

#[cfg(target_arch = "riscv32")]
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    boot::report_bringup_banner();
    boot::report_reset_reason();

    let mut mpu_pins = board::take_mpu_pins!(peripherals);
    boot::probe_pre_i2c_idle(&mut mpu_pins.scl, &mut mpu_pins.sda);
    let _ad0 = boot::drive_ad0_low(mpu_pins.ad0);

    let config = I2cConfig::default()
        .with_frequency(Rate::from_khz(board::I2C_FREQUENCY_KHZ))
        .with_timeout(BusTimeout::Maximum)
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(50)));
    let i2c = I2c::new(peripherals.I2C0, config).expect("failed to initialize I2C0").with_scl(mpu_pins.scl).with_sda(mpu_pins.sda);
    println!("{} initialized at {} kHz: SCL={} SDA={}", board::I2C_BUS_NAME, board::I2C_FREQUENCY_KHZ, board::SCL_PIN_NAME, board::SDA_PIN_NAME);
    let i2c = startup::scan_candidates(i2c);
    let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
    let wake_ok = mpu.wake().is_ok();
    println!("driver wake bus_address=0x{:02x} ok={}", startup::MPU_ADDR_AD0_LOW, wake_ok);
    let primary_probe = startup::probe_imu_driver(&mut mpu, startup::MPU_ADDR_AD0_LOW);
    let i2c = mpu.release();
    let mut high_mpu = Mpu6050::new(i2c, Address::Ad0High);
    let _ = startup::probe_imu_driver(&mut high_mpu, startup::MPU_ADDR_AD0_HIGH);
    let i2c = high_mpu.release();
    let mut mpu = Mpu6050::new(i2c, Address::Ad0Low);
    startup::log_verification_summary(primary_probe);
    let validation = startup::run_advanced_validation(&mut mpu, &delay, startup::MPU_ADDR_AD0_LOW);
    let mut conditions = startup::StartupConditions { diagnostics_complete: true, timing_confirmed: validation.timing_registers_confirmed, final_interrupts_zero: validation.interrupt_state_confirmed_zero, ..Default::default() };
    println!("imu_interrupt_policy=explicit_opt_in sources_disabled={} timing_registers_confirmed={} status_polling=off", validation.interrupt_state_confirmed_zero, validation.timing_registers_confirmed);
    if conditions.diagnostics_complete && conditions.timing_confirmed && conditions.final_interrupts_zero {
        let mut io = Io::new(peripherals.IO_MUX);
        io.set_interrupt_handler(acquisition::int_data_ready_handler);
        let mut input = Input::new(mpu_pins.int, InputConfig::default().with_pull(Pull::None));
        critical_section::with(|cs| { input.listen(Event::RisingEdge); acquisition::INT_INPUT.borrow_ref_mut(cs).replace(input); });
        conditions.gpio_configured = true;
        let interrupt_conditions = startup::configure_data_ready_startup(&mut mpu);
        conditions.stale_status_cleared = interrupt_conditions.stale_status_cleared;
        conditions.enable_success = interrupt_conditions.enable_success;
        conditions.exact_data_ready_readback = interrupt_conditions.exact_data_ready_readback;
    }
    println!("data_ready_startup diagnostics_complete={} timing_confirmed={} final_interrupts_zero={} gpio_configured={} stale_status_cleared={} enable_success={} exact_data_ready_readback={} acquisition_started={}", conditions.diagnostics_complete, conditions.timing_confirmed, conditions.final_interrupts_zero, conditions.gpio_configured, conditions.stale_status_cleared, conditions.enable_success, conditions.exact_data_ready_readback, conditions.allows_acquisition());
    if conditions.allows_acquisition() {
        let mut stats = telemetry::AcquisitionStats::default();
        let acquisition_start_us = Instant::now().duration_since_epoch().as_micros() as u64;
        let mut last_summary_us = acquisition_start_us;
        loop {
            let batch_count = critical_section::with(|cs| acquisition::INT_PENDING.borrow_ref_mut(cs).take_all());
            if batch_count != 0 {
                let consumed_timestamp_us = Instant::now().duration_since_epoch().as_micros() as u64;
                let sample_for_output = acquisition::service_pending_batch(&mut mpu, &mut stats, batch_count, consumed_timestamp_us, || Instant::now().duration_since_epoch().as_micros() as u64);
                if let Some(raw) = sample_for_output && stats.successful_samples <= telemetry::RAW_EXAMPLE_LIMIT {
                    let sample_timestamp_us = stats.last_sample_us.unwrap_or_default();
                    println!("RAW consumed_events={} accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) consumed_timestamp_us={} sample_timestamp_us={}", stats.consumed, raw.accel[0], raw.accel[1], raw.accel[2], raw.temp, raw.gyro[0], raw.gyro[1], raw.gyro[2], consumed_timestamp_us, sample_timestamp_us);
                }
            }
            let now = Instant::now().duration_since_epoch().as_micros() as u64;
            if now.saturating_sub(last_summary_us) >= telemetry::SUMMARY_PERIOD_US { telemetry::log_acquisition_summary(&stats, acquisition_start_us, now); last_summary_us = now; }
        }
    }
    println!("data_ready_acquisition_blocked");
    loop { delay.delay_millis(startup::BLOCKED_IDLE_DELAY_MS); }
}
