#[cfg(target_arch = "riscv32")]
use crate::acquisition::{AcquisitionStats, pending_snapshot};
#[cfg(target_arch = "riscv32")]
use crate::startup::BoardMpu;
#[cfg(all(not(feature = "binary-frames"), target_arch = "riscv32"))]
use core::fmt;
#[cfg(target_arch = "riscv32")]
use esp_hal::time::Instant;
#[cfg(all(feature = "binary-frames", target_arch = "riscv32"))]
use esp_println::Printer;
#[cfg(target_arch = "riscv32")]
use esp_println::println;
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::{RawAccelGyroTemp, RawReadOutcome, RawRetryPolicy};

pub(crate) const RAW_EXAMPLE_LIMIT: u64 = 8;
pub(crate) const SUMMARY_PERIOD_US: u64 = 1_000_000;

#[cfg(target_arch = "riscv32")]
pub(crate) fn maybe_log_raw_example(
    stats: &AcquisitionStats,
    raw: &RawAccelGyroTemp,
    consumed_timestamp_us: u64,
) {
    if stats.successful_samples <= RAW_EXAMPLE_LIMIT {
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
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_MAGIC: [u8; 2] = *b"IM";
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_VERSION: u8 = 1;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_PAYLOAD_LEN: u8 = 32;
#[cfg(feature = "binary-frames")]
const BINARY_FRAME_LEN: usize = 38;
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
#[cfg(target_arch = "riscv32")]
pub(crate) fn log_acquisition_summary(
    stats: &AcquisitionStats,
    acquisition_start_us: u64,
    now_us: u64,
) {
    let pending = pending_snapshot();
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
