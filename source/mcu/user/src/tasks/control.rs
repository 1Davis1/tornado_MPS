use super::stats::Statistics;
use crate::{
    buffers::{AdcPoints, AdcProducer, DacConsumer},
    hal::RetCode,
    println,
    skifio::{self, Aout, AtomicDin, AtomicDout, DinHandler, XferIn, XferOut},
    Error,
};
use alloc::sync::Arc;
use common::config::Point;
use core::{
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    time::Duration,
};
use freertos::{Duration as FreeRtosDuration, FreeRtosError, Semaphore, Task, TaskPriority};
use ux::u4;

pub type Din = u8;
pub type Dout = u4;

pub struct ControlHandle {
    /// Semaphore to notify that something is ready.
    ready_sem: Semaphore,

    dac_enabled: AtomicBool,

    din: AtomicDin,
    dout: AtomicDout,

    /// Discrete input has changed.
    din_changed: AtomicBool,
    /// Discrete output has changed.
    dout_changed: AtomicBool,

    /// Number of DAC points to write until notified.
    dac_notify_every: AtomicUsize,
    /// Number of ADC points to read until notified.
    adc_notify_every: AtomicUsize,
}

struct ControlDac {
    running: bool,
    buffer: DacConsumer,
    last_point: Point,
    counter: usize,
}

struct ControlAdc {
    buffer: AdcProducer,
    last_point: AdcPoints,
    counter: usize,
}

pub struct Control {
    dac: ControlDac,
    adc: ControlAdc,
    handle: Arc<ControlHandle>,
    stats: Arc<Statistics>,
}

impl ControlHandle {
    fn new() -> Self {
        Self {
            ready_sem: Semaphore::new_binary().unwrap(),
            dac_enabled: AtomicBool::new(false),
            din: AtomicDin::new(0),
            dout: AtomicDout::new(0),
            din_changed: AtomicBool::new(false),
            dout_changed: AtomicBool::new(false),
            dac_notify_every: AtomicUsize::new(0),
            adc_notify_every: AtomicUsize::new(0),
        }
    }
    pub fn configure(&self, dac_notify_every: usize, adc_notify_every: usize) {
        self.dac_notify_every.store(dac_notify_every, Ordering::Release);
        self.adc_notify_every.store(adc_notify_every, Ordering::Release);
    }

    pub fn notify(&self) {
        self.ready_sem.give();
    }
    pub fn wait_ready(&self, timeout: Option<Duration>) -> bool {
        match self.ready_sem.take(match timeout {
            Some(dur) => FreeRtosDuration::ms(dur.as_millis() as u32),
            None => FreeRtosDuration::infinite(),
        }) {
            Ok(()) => true,
            Err(FreeRtosError::Timeout) => false,
            Err(e) => panic!("{:?}", e),
        }
    }

    pub fn set_dac_mode(&self, enabled: bool) {
        self.dac_enabled.store(enabled, Ordering::Release);
    }

    fn update_din(&self, value: Din) -> bool {
        if self.din.swap(value, Ordering::AcqRel) != value {
            self.din_changed.fetch_or(true, Ordering::AcqRel);
            true
        } else {
            false
        }
    }
    pub fn take_din(&self) -> Option<Din> {
        if self.din_changed.fetch_and(false, Ordering::AcqRel) {
            Some(self.din.load(Ordering::Acquire))
        } else {
            None
        }
    }

    pub fn set_dout(&self, value: Dout) {
        let raw = value.into();
        if self.dout.swap(raw, Ordering::AcqRel) != raw {
            self.dout_changed.fetch_or(true, Ordering::AcqRel);
        }
    }
}

impl Control {
    pub fn new(dac_buf: DacConsumer, adc_buf: AdcProducer, stats: Arc<Statistics>) -> (Self, Arc<ControlHandle>) {
        let handle = Arc::new(ControlHandle::new());
        (
            Self {
                dac: ControlDac {
                    running: false,
                    buffer: dac_buf,
                    last_point: 0x7fff,
                    counter: 0,
                },
                adc: ControlAdc {
                    buffer: adc_buf,
                    last_point: AdcPoints::default(),
                    counter: 0,
                },
                handle: handle.clone(),
                stats,
            },
            handle,
        )
    }

    fn make_din_handler(&self) -> impl DinHandler {
        let handle = self.handle.clone();
        move |context, din| {
            if handle.update_din(din) {
                handle.ready_sem.give_from_isr(context);
            }
        }
    }

    fn task_main(mut self) -> ! {
        let handle = self.handle.clone();
        let stats = self.stats.clone();

        let mut skifio = skifio::handle().unwrap();
        skifio.subscribe_din(Some(self.make_din_handler())).unwrap();

        println!("Enter SkifIO loop");
        let iter_counter = 0;
        loop {
            let mut ready = false;

            skifio.set_dac_state(handle.dac_enabled.load(Ordering::Acquire)).unwrap();

            // Wait for 10 kHz sync signal
            match skifio.wait_ready(Some(Duration::from_millis(1000))) {
                Ok(()) => (),
                Err(Error::Hal(RetCode::TimedOut)) => {
                    println!("SkifIO timeout {}", iter_counter);
                    continue;
                }
                Err(e) => panic!("{:?}", e),
            }

            // Write discrete output
            if handle.dout_changed.fetch_and(false, Ordering::AcqRel) {
                skifio.write_dout(handle.dout.load(Ordering::Acquire)).unwrap();
            }

            // Read discrete input
            ready |= handle.update_din(skifio.read_din());

            // Fetch next DAC value from buffer
            let mut dac_value = self.dac.last_point;
            if self.dac.running {
                if let Some(value) = self.dac.buffer.pop() {
                    dac_value = value;
                    self.dac.last_point = value;
                    // Decrement DAC notification counter.
                    if self.dac.counter > 0 {
                        self.dac.counter -= 1;
                    } else {
                        self.dac.counter = handle.dac_notify_every.load(Ordering::Acquire) - 1;
                        ready = true;
                    }
                } else {
                    stats.dac.report_lost_empty(1);
                }
            }

            // Transfer DAC/ADC values to/from SkifIO board.
            {
                // TODO: Check for overflow.
                let dac = dac_value as Aout;
                let adcs = match skifio.transfer(XferOut { dac }) {
                    Ok(XferIn { adcs }) => {
                        self.adc.last_point = adcs;
                        adcs
                    }
                    Err(Error::Hal(RetCode::InvalidData)) => {
                        // CRC check error
                        stats.report_crc_error();
                        self.adc.last_point
                    }
                    Err(e) => panic!("{:?}", e),
                };

                // Handle ADCs
                {
                    // Update ADC value statistics
                    stats.adcs.update_values(adcs);
                    // Push ADC point to buffer.
                    if self.adc.buffer.push(adcs).is_err() {
                        stats.adcs.report_lost_full(1);
                    }

                    // Decrement ADC notification counter.
                    if self.adc.counter > 0 {
                        self.adc.counter -= 1;
                    } else {
                        self.adc.counter = handle.adc_notify_every.load(Ordering::Acquire) - 1;
                        ready = true;
                    }
                }
            }

            if ready {
                // Notify
                handle.ready_sem.give();
            }

            stats.report_sample();
        }
    }

    pub fn run(self, priority: u8) {
        Task::new()
            .name("control")
            .priority(TaskPriority(priority))
            .start(move |_| self.task_main())
            .unwrap();
    }
}
