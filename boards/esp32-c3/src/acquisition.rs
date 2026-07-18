use crate::telemetry::AcquisitionStats;
#[cfg(target_arch = "riscv32")]
use esp_hal::{
    gpio::{Event, Input, InputConfig, Io, Pull},
    peripherals::{GPIO6, IO_MUX},
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

pub(crate) fn service_pending_batch<D: AcquisitionDevice>(
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

#[cfg(target_arch = "riscv32")]
pub(crate) fn arm(io_mux: IO_MUX<'static>, int_pin: GPIO6<'static>) {
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
    let sample = service_pending_batch(device, stats, batch_count, consumed_timestamp_us, || {
        Instant::now().duration_since_epoch().as_micros() as u64
    });
    Some(DrainOutcome {
        sample,
        consumed_timestamp_us,
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
    use crate::telemetry::AcquisitionStats;
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
}
