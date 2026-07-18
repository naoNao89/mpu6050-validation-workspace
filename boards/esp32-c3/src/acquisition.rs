#[cfg(target_arch = "riscv32")]
use crate::board::IntPin;
#[cfg(target_arch = "riscv32")]
use esp_hal::{
    gpio::{Event, Input, InputConfig, Io, Pull},
    peripherals::IO_MUX,
    time::Instant,
};
#[cfg(target_arch = "riscv32")]
use mpu6050_driver::RawAccelGyroTemp;

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct PendingEvents {
    pub(crate) pending: u32,
    pub(crate) max_pending: u32,
    pub(crate) total: u64,
    /// Number of data-ready ISR events that could not be added to the pending-event
    /// count because that count was already saturated at `u32::MAX`.
    ///
    /// This is a software counter-saturation metric. It is not an MPU FIFO
    /// overflow and does not directly count lost sensor samples.
    pub(crate) events_unrecorded_due_to_pending_saturation: u64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct AcquisitionStats {
    pub(crate) consumed: u64,
    pub(crate) missed_or_coalesced_events: u64,
    pub(crate) successful_samples: u64,
    pub(crate) motion_i2c_errors: u64,
    pub(crate) status_ack_i2c_errors: u64,
    pub(crate) first_consumed_us: Option<u64>,
    pub(crate) last_consumed_us: Option<u64>,
    pub(crate) first_sample_us: Option<u64>,
    pub(crate) last_sample_us: Option<u64>,
    pub(crate) interval_count: u64,
    pub(crate) interval_min_us: Option<u64>,
    pub(crate) interval_max_us: Option<u64>,
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
    pub(crate) fn consumed_batch(&mut self, count: u32, now: u64) {
        self.consumed = self.consumed.saturating_add(count as u64);
        self.missed_or_coalesced_events = self
            .missed_or_coalesced_events
            .saturating_add(count.saturating_sub(1) as u64);
        self.first_consumed_us.get_or_insert(now);
        self.last_consumed_us = Some(now);
    }
    pub(crate) fn sample(&mut self, now: u64) {
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
    pub(crate) fn rate(count: u64, first: Option<u64>, last: Option<u64>) -> Option<f32> {
        match (count, first, last) {
            (2.., Some(first), Some(last)) if last > first => {
                Some((count - 1) as f32 * 1_000_000.0 / (last - first) as f32)
            }
            _ => None,
        }
    }
    pub(crate) fn interval_p50_us(&self) -> Option<u64> {
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
impl PendingEvents {
    pub(crate) fn signal(&mut self) {
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
    pub(crate) fn take_all(&mut self) -> u32 {
        let pending = self.pending;
        self.pending = 0;
        pending
    }
}

pub(crate) trait AcquisitionDevice {
    type Sample;

    fn read_motion(&mut self) -> Option<Self::Sample>;
    fn acknowledge_status(&mut self) -> bool;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ServiceBatchResult<S> {
    pub(crate) sample: Option<S>,
    /// Present only when `sample` is `Some`; device-side successful-sample time.
    pub(crate) successful_sample_timestamp_us: Option<u64>,
}

pub(crate) fn service_pending_batch<D: AcquisitionDevice>(
    device: &mut D,
    stats: &mut AcquisitionStats,
    batch_count: u32,
    consumed_timestamp_us: u64,
    successful_sample_timestamp_us: impl FnOnce() -> u64,
) -> ServiceBatchResult<D::Sample> {
    debug_assert!(batch_count > 0);
    stats.consumed_batch(batch_count, consumed_timestamp_us);
    let sample = device.read_motion();
    let successful_sample_timestamp_us = if sample.is_some() {
        let timestamp_us = successful_sample_timestamp_us();
        stats.sample(timestamp_us);
        Some(timestamp_us)
    } else {
        stats.motion_i2c_errors = stats.motion_i2c_errors.saturating_add(1);
        None
    };
    if !device.acknowledge_status() {
        stats.status_ack_i2c_errors = stats.status_ack_i2c_errors.saturating_add(1);
    }
    ServiceBatchResult {
        sample,
        successful_sample_timestamp_us,
    }
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

/// Data-ready ISR for the board INT pin (`board::INT_PIN_NAME` / `board::IntPin`).
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

#[cfg(target_arch = "riscv32")]
pub(crate) fn arm(io_mux: IO_MUX<'static>, int_pin: IntPin) {
    let mut io = Io::new(io_mux);
    io.set_interrupt_handler(int_data_ready_handler);
    let mut input = Input::new(int_pin, InputConfig::default().with_pull(Pull::None));
    critical_section::with(|cs| {
        input.listen(Event::RisingEdge);
        INT_INPUT.borrow_ref_mut(cs).replace(input);
    });
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn pending_snapshot() -> PendingEvents {
    critical_section::with(|cs| *INT_PENDING.borrow_ref(cs))
}

#[cfg(target_arch = "riscv32")]
pub(crate) struct DrainOutcome<S> {
    pub(crate) sample: Option<S>,
    pub(crate) consumed_timestamp_us: u64,
    /// Paired with `sample`: set only for a successful motion read.
    pub(crate) successful_sample_timestamp_us: Option<u64>,
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn drain_pending<D: AcquisitionDevice>(
    device: &mut D,
    stats: &mut AcquisitionStats,
) -> Option<DrainOutcome<D::Sample>> {
    let batch_count = critical_section::with(|cs| INT_PENDING.borrow_ref_mut(cs).take_all());
    if batch_count == 0 {
        return None;
    }
    let consumed_timestamp_us = Instant::now().duration_since_epoch().as_micros() as u64;
    let result = service_pending_batch(device, stats, batch_count, consumed_timestamp_us, || {
        Instant::now().duration_since_epoch().as_micros() as u64
    });
    Some(DrainOutcome {
        sample: result.sample,
        consumed_timestamp_us,
        successful_sample_timestamp_us: result.successful_sample_timestamp_us,
    })
}
#[cfg(target_arch = "riscv32")]
impl AcquisitionDevice for crate::startup::BoardMpu<'_> {
    type Sample = RawAccelGyroTemp;

    fn read_motion(&mut self) -> Option<Self::Sample> {
        self.read_raw_accel_gyro_temp().ok()
    }

    fn acknowledge_status(&mut self) -> bool {
        self.int_status().is_ok()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
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
    struct FakeAcquisitionDevice {
        motion_reads: u32,
        status_acknowledgments: u32,
        motion: Option<u8>,
    }

    impl AcquisitionDevice for FakeAcquisitionDevice {
        type Sample = u8;

        fn read_motion(&mut self) -> Option<Self::Sample> {
            self.motion_reads += 1;
            self.motion
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
            motion: Some(42),
        };
        let mut stats = AcquisitionStats::default();

        let result = service_pending_batch(&mut device, &mut stats, 3, 100, || 110);

        assert_eq!(result.sample, Some(42));
        assert_eq!(result.successful_sample_timestamp_us, Some(110));
        assert_eq!(device.motion_reads, 1);
        assert_eq!(device.status_acknowledgments, 1);
        assert_eq!(stats.consumed, 3);
        assert_eq!(stats.successful_samples, 1);
        assert_eq!(stats.missed_or_coalesced_events, 2);
    }

    #[test]
    fn sample_and_successful_sample_timestamp_remain_paired() {
        let mut device = FakeAcquisitionDevice {
            motion_reads: 0,
            status_acknowledgments: 0,
            motion: Some(7),
        };
        let mut stats = AcquisitionStats::default();
        let result = service_pending_batch(&mut device, &mut stats, 1, 50, || 99);
        assert_eq!(
            (result.sample, result.successful_sample_timestamp_us),
            (Some(7), Some(99))
        );
        assert_eq!(stats.last_sample_us, Some(99));
    }

    #[test]
    fn failed_motion_read_has_no_successful_sample_timestamp() {
        let mut device = FakeAcquisitionDevice {
            motion_reads: 0,
            status_acknowledgments: 0,
            motion: None,
        };
        let mut stats = AcquisitionStats::default();
        let result = service_pending_batch(&mut device, &mut stats, 2, 50, || {
            panic!("timestamp must not be taken on failed motion read")
        });
        assert_eq!(result.sample, None);
        assert_eq!(result.successful_sample_timestamp_us, None);
        assert_eq!(stats.successful_samples, 0);
        assert_eq!(stats.motion_i2c_errors, 1);
        assert_eq!(stats.consumed, 2);
        assert_eq!(stats.missed_or_coalesced_events, 1);
        assert_eq!(device.motion_reads, 1);
        assert_eq!(device.status_acknowledgments, 1);
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
