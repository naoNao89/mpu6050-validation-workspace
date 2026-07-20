use mpu6050_driver::{
    ACCEL_LSB_PER_G_2G, AccelRange, Address, Dlpf, DlpfReadError, FIFO_ACCEL_GYRO_FRAME_BYTES,
    FifoReadDiagnostics, GYRO_LSB_PER_DPS_250DPS, GyroRange, Identity, IntStatus, InterruptEnable,
    Mpu6050, RawAccelGyroTemp, RawReadOutcome, RawRetryPolicy, RawSampleSuspicion,
    TEMP_LSB_PER_DEG_C, TEMP_OFFSET_DEG_C, raw_to_imu_sample,
};

struct ApiI2c;

#[derive(Debug)]
struct ApiError;

impl embedded_hal::i2c::Error for ApiError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal::i2c::ErrorType for ApiI2c {
    type Error = ApiError;
}

impl embedded_hal::i2c::I2c for ApiI2c {
    fn read(
        &mut self,
        _address: embedded_hal::i2c::SevenBitAddress,
        _read: &mut [u8],
    ) -> Result<(), Self::Error> {
        unreachable!("compile-only public API probe")
    }

    fn write(
        &mut self,
        _address: embedded_hal::i2c::SevenBitAddress,
        _write: &[u8],
    ) -> Result<(), Self::Error> {
        unreachable!("compile-only public API probe")
    }

    fn write_read(
        &mut self,
        _address: embedded_hal::i2c::SevenBitAddress,
        _write: &[u8],
        _read: &mut [u8],
    ) -> Result<(), Self::Error> {
        unreachable!("compile-only public API probe")
    }

    fn transaction(
        &mut self,
        _address: embedded_hal::i2c::SevenBitAddress,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        unreachable!("compile-only public API probe")
    }
}

#[test]
fn crate_root_public_api_still_imports() {
    let raw = RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]);
    let sample = raw_to_imu_sample(raw);
    assert!((sample.accel_magnitude_g() - 1.0).abs() < 1e-9);
    assert!((sample.gyro_magnitude_dps() - 1.0).abs() < 1e-9);
    Mpu6050::new((), Address::Ad0Low).release();

    let diagnostics = FifoReadDiagnostics {
        fifo_count_before_bytes: FIFO_ACCEL_GYRO_FRAME_BYTES as u16,
        fifo_bytes_requested: FIFO_ACCEL_GYRO_FRAME_BYTES as u16,
        fifo_count_after_bytes: 0,
        fifo_overflow_seen: false,
        int_status_read_ok: true,
        read_len_frame_aligned: true,
        fifo_count_before_frame_aligned: true,
        fifo_count_after_frame_aligned: true,
        had_requested_bytes_before_read: true,
        fifo_count_delta_ok: true,
    };

    assert!(diagnostics.frame_usable());
    assert!(!diagnostics.should_reset_fifo());
    assert!(!raw.is_suspicious());
    assert_eq!(raw.temp_degrees_c(), TEMP_OFFSET_DEG_C);
    assert_eq!(ACCEL_LSB_PER_G_2G, 16_384.0);
    assert_eq!(GYRO_LSB_PER_DPS_250DPS, 131.0);
    assert_eq!(TEMP_LSB_PER_DEG_C, 340.0);

    let _ = AccelRange::G2;
    let _ = GyroRange::Dps250;
    let _ = Dlpf::Cfg2.sample_rate_hz(4);
    let _ = DlpfReadError::<()>::ReservedConfig;
    let _ = Identity::Mpu6050;
    let _ = RawRetryPolicy::reject_after_retries(0);
    let _ = RawRetryPolicy::accept_after_retries(1);
    let _ = RawReadOutcome::<()>::Clean { raw };
    let _ = RawSampleSuspicion::GyroPartialMinusOne;

    fn takes_int_status(_status: IntStatus) {}
    let _ = takes_int_status;
    fn takes_interrupt_enable(_enable: InterruptEnable) {}
    let _ = takes_interrupt_enable;
    let _: fn(InterruptEnable) -> bool = InterruptEnable::only_data_ready;

    // Compile-time guards for the externally visible method signatures.
    // These methods are not executed; behavioral I2C tests live with the driver.
    let _: fn(&mut Mpu6050<ApiI2c>, Dlpf) -> Result<(), ApiError> = Mpu6050::<ApiI2c>::set_dlpf;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<Dlpf, DlpfReadError<ApiError>> =
        Mpu6050::<ApiI2c>::dlpf;
    let _: fn(&mut Mpu6050<ApiI2c>, u8) -> Result<(), ApiError> =
        Mpu6050::<ApiI2c>::set_sample_rate_divider;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<u8, ApiError> =
        Mpu6050::<ApiI2c>::sample_rate_divider;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<InterruptEnable, ApiError> =
        Mpu6050::<ApiI2c>::interrupt_enable;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<(), ApiError> =
        Mpu6050::<ApiI2c>::disable_all_interrupts;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<(), ApiError> =
        Mpu6050::<ApiI2c>::enable_data_ready_interrupt;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<(), ApiError> =
        Mpu6050::<ApiI2c>::enable_fifo_overflow_interrupt;
    let _: fn(&mut Mpu6050<ApiI2c>) -> Result<IntStatus, ApiError> = Mpu6050::<ApiI2c>::int_status;
}
