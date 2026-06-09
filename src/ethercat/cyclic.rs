//! Cyclic process-data engine (the PIT-tick state machine).
//!
//! IgH: the cyclic half of master/master.c (`ecrt_master_send/receive`,
//! `ecrt_domain_queue/process`) plus the SAFE-OP -> OP gating. Driven here from
//! the high-priority PIT cyclic task: each tick processes the previous cycle's
//! reply and sends this cycle's frame (pipelined, allocation-free, no busy-wait).
//!
//! Phases: `Priming` cycles the LRW until the slave is exchanging data, then
//! `RequestingOp` interleaves an AL-control/-status datagram with the LRW (via
//! the `0x8000` multi-datagram framing) so process data keeps flowing while OP
//! is requested, then `Operational` (steady single-LRW exchange).

use crate::board::clock_config::CORE_CLOCK_HZ;
use crate::ethercat::cia402::{Cia402, DriveCommand};
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, ECAT_RX_LEN};
use crate::ethercat::domain::EcDomain;
use crate::ethercat::globals::{al_state, reg, EC_FRAME_HEADER_SIZE};
use crate::hal::host_bridge::{HostBridge, ReplyStatus};
use cortex_m::peripheral::DWT;

/// EtherCAT frame buffer for the cyclic LRW (+ optional appended datagram).
const CYCLIC_BUF: usize = crate::ethercat::domain::MAX_IMAGE + 32;
/// Consecutive responding cycles (WKC > 0) in `Priming` before requesting OP.
const PRIMING_CYCLES: u32 = 3;
/// Consecutive cyclic ticks with a stalled host heartbeat before the host is
/// declared timed out (drives the safe-state quick-stop). Counted in *cyclic
/// ticks*, so it must exceed the cyclic-to-host frame-rate ratio: with the
/// cyclic loop running up to ~4x the host servo rate, several ticks can elapse
/// between host frames even when the host is healthy. 20 tolerates a ~4:1 ratio
/// with margin; raise it if the EtherCAT cycle is much faster than the host.
const HOST_WDOG_LIMIT_CYCLES: u16 = 20;

/// Cyclic engine phase.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// Cycling the LRW, waiting for the slave to start exchanging data.
    Priming,
    /// Process data flowing; interleaving the SAFE-OP -> OP request.
    RequestingOp,
    /// Steady cyclic exchange in OP.
    Operational,
    /// The drive rejected OP (AL error); still cycling so it holds SAFE-OP.
    Faulted,
}

/// What was appended to the last cyclic frame (to interpret the reply).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Appended {
    None,
    Control,
    Status,
    /// A DC system-time-difference read (FPRD 0x092C) for sync monitoring.
    DcDiff,
}

/// A snapshot of cyclic health for reporting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CyclicStatus {
    pub phase: Phase,
    pub wkc: u16,
    pub expected_wkc: u16,
    pub cycles: u32,
    /// Configured cyclic rate (Hz), derived from the period.
    pub rate_hz: u32,
    /// Expected period (us), derived from the period.
    pub period_us: u32,
    /// Shortest observed tick-to-tick interval (us).
    pub jitter_min_us: u32,
    /// Longest observed tick-to-tick interval (us).
    pub jitter_max_us: u32,
    /// Worst-case absolute deviation from the expected period (us).
    pub jitter_worst_us: u32,
    /// Worst-case absolute deviation from the expected period (core cycles).
    pub jitter_worst_cyc: u32,
    /// Latest decoded DC system-time difference (signed ns; 0 if never read).
    pub dc_diff_ns: i32,
    /// Largest-magnitude DC system-time difference seen (signed ns).
    pub dc_diff_max_ns: i32,
    /// Whether at least one DC system-time difference has been read.
    pub dc_valid: bool,
}

/// The cyclic process-data engine for one domain.
pub struct Cyclic {
    domain: EcDomain,
    station: u16,
    phase: Phase,
    tx: [u8; CYCLIC_BUF],
    tx_len: usize,
    rx: [u8; CYCLIC_BUF],
    outstanding: bool,
    expected_index: u8,
    good_cycles: u32,
    total_cycles: u32,
    appended: Appended,
    /// Configured cyclic period (ns); drives the jitter baseline + rate report.
    cycle_ns: u64,
    /// Expected tick-to-tick interval, in core (DWT CYCCNT) cycles.
    expected_cyc: u32,
    /// DWT CYCCNT at the previous tick (for interval measurement).
    last_tick_cyc: u32,
    /// False until the first tick establishes a `last_tick_cyc` baseline.
    have_last_tick: bool,
    /// Shortest / longest observed tick interval (core cycles; 0 = no samples).
    min_interval_cyc: u32,
    max_interval_cyc: u32,
    /// Worst absolute deviation of an interval from `expected_cyc` (core cycles).
    worst_dev_cyc: u32,
    /// Latest / largest-magnitude DC system-time difference (signed ns).
    dc_diff_ns: i32,
    dc_diff_max_ns: i32,
    /// Whether a DC system-time difference has ever been decoded.
    dc_valid: bool,
    /// CiA-402 drive sequencer (controlword owner when a host is attached).
    cia402: Cia402,
}

impl Cyclic {
    /// Create the engine for the slave at `station`, built from the compile-time
    /// bus configuration and running at a `cycle_ns` period. The bring-up FSM has
    /// already reached SAFE-OP. Telemetry stats start fresh (reset on every
    /// `start`).
    pub fn new(station: u16, cycle_ns: u64) -> Self {
        enable_cycle_counter();
        // Expected tick interval in core cycles: period_ns * core_hz / 1e9.
        let expected_cyc =
            (cycle_ns.saturating_mul(CORE_CLOCK_HZ as u64) / 1_000_000_000) as u32;
        Self {
            domain: EcDomain::from_config(&crate::ethercat::config::generated::BUS),
            station,
            phase: Phase::Priming,
            tx: [0; CYCLIC_BUF],
            tx_len: 0,
            rx: [0; CYCLIC_BUF],
            outstanding: false,
            expected_index: 0,
            good_cycles: 0,
            total_cycles: 0,
            appended: Appended::None,
            cycle_ns,
            expected_cyc,
            last_tick_cyc: 0,
            have_last_tick: false,
            min_interval_cyc: u32::MAX,
            max_interval_cyc: 0,
            worst_dev_cyc: 0,
            dc_diff_ns: 0,
            dc_diff_max_ns: 0,
            dc_valid: false,
            cia402: Cia402::new(),
        }
    }

    /// The configured station address of the slave this engine drives. Used by
    /// `stop` to bring the drive down cleanly before tearing the engine down.
    pub fn station(&self) -> u16 {
        self.station
    }

    /// The process-data image (read; inputs are live in OP/SAFE-OP).
    pub fn image(&self) -> &[u8] {
        self.domain.image()
    }

    /// The process-data image (write; the application sets outputs here).
    pub fn image_mut(&mut self) -> &mut [u8] {
        self.domain.image_mut()
    }

    /// A snapshot of cyclic health (process-data + timing/DC telemetry).
    pub fn status(&self) -> CyclicStatus {
        // No interval recorded yet leaves `min` at its sentinel; report 0 instead.
        let has_samples = self.max_interval_cyc != 0;
        CyclicStatus {
            phase: self.phase,
            wkc: self.domain.last_wkc(),
            expected_wkc: self.domain.expected_wkc(),
            cycles: self.total_cycles,
            rate_hz: if self.cycle_ns != 0 {
                (1_000_000_000u64 / self.cycle_ns) as u32
            } else {
                0
            },
            period_us: (self.cycle_ns / 1_000) as u32,
            jitter_min_us: if has_samples { cyc_to_us(self.min_interval_cyc) } else { 0 },
            jitter_max_us: cyc_to_us(self.max_interval_cyc),
            jitter_worst_us: cyc_to_us(self.worst_dev_cyc),
            jitter_worst_cyc: self.worst_dev_cyc,
            dc_diff_ns: self.dc_diff_ns,
            dc_diff_max_ns: self.dc_diff_max_ns,
            dc_valid: self.dc_valid,
        }
    }

    /// Run one cycle: process the previous reply, then send this cycle's frame.
    /// Called from the high-priority PIT task; must stay short and non-blocking.
    pub fn tick(&mut self, dev: &mut Device, index: &mut u8) {
        self.begin_tick();
        if self.outstanding {
            self.receive(dev);
            self.outstanding = false;
        }
        self.send(dev, index);
    }

    /// Run one cycle integrated with the Pi/LinuxCNC host bridge: process the
    /// previous reply, apply the host's staged outputs (and, once present, the
    /// CiA-402 controlword + safe-state), send this cycle's frame, then snapshot
    /// the live inputs + status back into the reply the SPI task will return.
    ///
    /// Keeping every image write inside this (prio-3) path preserves the
    /// invariant that the cyclic task never blocks on the master lock.
    pub fn tick_with_host(&mut self, dev: &mut Device, index: &mut u8, host: &mut HostBridge) {
        self.begin_tick();
        if self.outstanding {
            self.receive(dev);
            self.outstanding = false;
        }
        let stall = host.tick_watchdog();
        let host_timeout = stall >= HOST_WDOG_LIMIT_CYCLES;
        // Apply the host's immediate outputs + the motion sample tagged for this
        // cycle before building the LRW. The CiA-402 controlword and safe-state
        // override the controlword after the raw outputs and before `send`.
        host.apply_outputs(self.domain.image_mut(), self.total_cycles);
        let underrun = host.motion_underrun();
        self.apply_drive_control(host, host_timeout, underrun);
        self.send(dev, index);
        let st = self.reply_status(host_timeout, underrun);
        host.build_reply(self.domain.image(), st);
    }

    /// Timestamp the tick (jitter accounting) and advance the cycle counter.
    fn begin_tick(&mut self) {
        // Timestamp at ISR entry: the interval between consecutive ticks is the
        // realized cycle period; its spread vs `expected_cyc` is the jitter.
        let now = DWT::cycle_count();
        if self.have_last_tick {
            self.record_interval(now.wrapping_sub(self.last_tick_cyc));
        } else {
            self.have_last_tick = true;
        }
        self.last_tick_cyc = now;
        self.total_cycles = self.total_cycles.wrapping_add(1);
    }

    /// CiA-402 drive sequencing + unified safe-state. Runs after the host's raw
    /// outputs are applied and before `send`, so it has the final say on the
    /// controlword written into this cycle's LRW. Skipped entirely when no host
    /// is attached, preserving the manual serial (`pd`) control workflow.
    ///
    /// Quick-stop is forced on any safe-state trigger: an explicit host request,
    /// a host-watchdog timeout, a motion-buffer underrun, or an EtherCAT fault
    /// (the engine dropped to `Faulted`).
    fn apply_drive_control(&mut self, host: &HostBridge, host_timeout: bool, underrun: bool) {
        if !host.has_host() || !self.cia402.has_drives() {
            return;
        }
        let quick_stop = host.quick_stop_requested()
            || host_timeout
            || underrun
            || self.phase == Phase::Faulted;
        let cmd = DriveCommand {
            enable: host.enable_requested(),
            fault_reset: host.fault_reset_requested(),
            quick_stop,
        };
        let img = self.domain.image_mut();
        self.cia402.step(img, cmd);
    }

    /// Build the reply status snapshot for the host frame, with the unified
    /// fault flags (EtherCAT fault / drive fault / host timeout / underrun).
    fn reply_status(&self, host_timeout: bool, underrun: bool) -> ReplyStatus {
        let phase = match self.phase {
            Phase::Priming => 0,
            Phase::RequestingOp => 1,
            Phase::Operational => 2,
            Phase::Faulted => 3,
        };
        let mut fault_flags = 0u8;
        if self.phase == Phase::Faulted {
            fault_flags |= 1 << 0;
        }
        if self.cia402.any_fault(self.domain.image()) {
            fault_flags |= 1 << 1;
        }
        if host_timeout {
            fault_flags |= 1 << 2;
        }
        if underrun {
            fault_flags |= 1 << 3;
        }
        ReplyStatus {
            // Link is "up" when the bus is actually exchanging data this cycle.
            link_up: self.domain.last_wkc() > 0,
            phase,
            wkc: self.domain.last_wkc().min(255) as u8,
            expected_wkc: self.domain.expected_wkc().min(255) as u8,
            cycle_index: self.total_cycles,
            fault_flags,
            host_timeout,
        }
    }

    /// Fold one measured tick interval (core cycles) into the jitter stats.
    fn record_interval(&mut self, interval: u32) {
        if interval < self.min_interval_cyc {
            self.min_interval_cyc = interval;
        }
        if interval > self.max_interval_cyc {
            self.max_interval_cyc = interval;
        }
        let dev = interval.abs_diff(self.expected_cyc);
        if dev > self.worst_dev_cyc {
            self.worst_dev_cyc = dev;
        }
    }

    /// Drain the RX ring for the previous frame's reply (matched by index).
    fn receive(&mut self, dev: &mut Device) {
        for _ in 0..ECAT_RX_LEN {
            match dev.poll(&mut self.rx) {
                Some(len) if len >= 4 && self.rx[3] == self.expected_index => {
                    self.process(len);
                    return;
                }
                Some(_) => continue, // stray frame; keep draining
                None => return,
            }
        }
    }

    /// Parse the reply frame (LRW + optional appended datagram) and update state.
    fn process(&mut self, len: usize) {
        let frame = &self.rx[..len];
        let (lrw, next) = match datagram::parse_at(frame, EC_FRAME_HEADER_SIZE) {
            Some(v) => v,
            None => return,
        };
        let wkc = lrw.working_counter;
        self.domain.apply_reply(&lrw);

        if wkc > 0 {
            self.good_cycles = self.good_cycles.saturating_add(1);
        } else {
            self.good_cycles = 0;
        }
        if self.phase == Phase::Priming && self.good_cycles >= PRIMING_CYCLES {
            self.phase = Phase::RequestingOp;
        }

        // Interpret the appended datagram: the AL status during the OP request,
        // or the DC system-time-difference read while operational. Inlined (not a
        // helper) so the disjoint-field writes don't conflict with `frame`'s
        // borrow of `self.rx`. A zero working counter means the appended read did
        // not reach the slave, so its value is ignored.
        if next != 0 {
            match self.appended {
                Appended::Status => {
                    if let Some((al, _)) = datagram::parse_at(frame, next) {
                        let status = al.data.first().copied().unwrap_or(0);
                        if status & al_state::ERROR != 0 {
                            self.phase = Phase::Faulted;
                        } else if status & al_state::MASK == al_state::OP {
                            self.phase = Phase::Operational;
                        }
                    }
                }
                Appended::DcDiff => {
                    if let Some((dc, _)) = datagram::parse_at(frame, next) {
                        if dc.working_counter > 0 && dc.data.len() >= 4 {
                            let raw =
                                u32::from_le_bytes([dc.data[0], dc.data[1], dc.data[2], dc.data[3]]);
                            let diff = decode_dc_diff(raw);
                            self.dc_diff_ns = diff;
                            self.dc_valid = true;
                            if diff.unsigned_abs() > self.dc_diff_max_ns.unsigned_abs() {
                                self.dc_diff_max_ns = diff;
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }

    /// Build and send this cycle's frame (LRW, plus an AL datagram while
    /// requesting OP).
    fn send(&mut self, dev: &mut Device, index: &mut u8) {
        let i = alloc_index(index);
        self.tx_len = self.domain.build_lrw(&mut self.tx, i);
        self.expected_index = i;
        self.appended = Appended::None;

        if self.phase == Phase::RequestingOp {
            // Alternate: write AL control = OP, then poll AL status, repeatedly,
            // all while the LRW keeps process data flowing.
            let ai = alloc_index(index);
            if self.good_cycles % 2 == 0 {
                self.tx_len = datagram::append(
                    &mut self.tx,
                    ai,
                    Command::Fpwr,
                    self.station,
                    reg::AL_CONTROL,
                    &[al_state::OP, 0],
                );
                self.appended = Appended::Control;
            } else {
                self.tx_len = datagram::append(
                    &mut self.tx,
                    ai,
                    Command::Fprd,
                    self.station,
                    reg::AL_STATUS,
                    &[0u8, 0u8],
                );
                self.appended = Appended::Status;
            }
        } else if self.phase == Phase::Operational {
            // Interleave a DC system-time-difference read (FPRD 0x092C) into the
            // OP frame, alongside the LRW, so sync drift is monitored without a
            // separate transaction disturbing the cycle timing.
            let ai = alloc_index(index);
            self.tx_len = datagram::append(
                &mut self.tx,
                ai,
                Command::Fprd,
                self.station,
                reg::DC_SYS_TIME_DIFF,
                &[0u8; 4],
            );
            self.appended = Appended::DcDiff;
        }

        self.outstanding = dev.send(&self.tx[..self.tx_len]).is_ok();
    }
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}

/// Convert a duration in core (DWT CYCCNT) cycles to microseconds.
#[inline]
fn cyc_to_us(cyc: u32) -> u32 {
    (cyc as u64 * 1_000_000 / CORE_CLOCK_HZ as u64) as u32
}

/// Enable the DWT cycle counter (CYCCNT) for cycle-accurate jitter timing.
/// Idempotent, so it is safe to call on every `start`.
fn enable_cycle_counter() {
    // SAFETY: single-core MCU. DWT/DCB are not driven anywhere else, and
    // enabling the cycle counter is idempotent, so stealing the core
    // peripherals here cannot disturb another owner.
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
}

/// Decode ESC register 0x092C (system-time difference) into a signed nanosecond
/// drift: bits 0..30 are the magnitude and bit 31 is the sign (set = the local
/// clock is behind the reference). Mirrors how IgH reads DC sync deviation.
#[inline]
fn decode_dc_diff(raw: u32) -> i32 {
    let mag = (raw & 0x7FFF_FFFF) as i32;
    if raw & 0x8000_0000 != 0 {
        -mag
    } else {
        mag
    }
}
