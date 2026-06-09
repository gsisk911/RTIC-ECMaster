//! Cyclic process-data engine (the PIT-tick state machine).
//!
//! IgH: the cyclic half of master/master.c (`ecrt_master_send/receive`,
//! `ecrt_domain_queue/process`) plus the SAFE-OP -> OP gating. Driven here from
//! the high-priority PIT cyclic task: each tick processes the previous cycle's
//! reply and sends this cycle's frame (pipelined, allocation-free, no busy-wait).
//!
//! Phases: `Priming` cycles the one whole-bus LRW until data is exchanging, then
//! `RequestingOp` interleaves per-slave AL-control/-status datagrams with the
//! LRW (via the `0x8000` multi-datagram framing) so process data keeps flowing
//! while EACH slave is requested to OP, then `Operational` (steady LRW exchange,
//! plus continuous DC reference-time distribution + drift/AL monitoring once two
//! or more slaves are present). One LRW carries all slaves' process data; the
//! expected working counter scales with the number of slaves (handled by the
//! `domain`). The single-slave path is the N == 1 special case.

use crate::board::clock_config::CORE_CLOCK_HZ;
use crate::board::cycle_timer::{self, PERCLK_HZ};
use crate::ethercat::cia402::{Cia402, DriveCommand};
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, ECAT_RX_LEN};
use crate::ethercat::domain::EcDomain;
use crate::ethercat::globals::{al_state, reg, EC_FRAME_HEADER_SIZE, EC_MAX_SLAVES};
use crate::hal::host_bridge::{HostBridge, ReplyStatus};
use heapless::Vec;

/// EtherCAT frame buffer for the cyclic LRW + its appended datagrams. Sized for
/// the whole-bus image plus the worst-case Operational tail (an ARMW DC-time
/// distribution + one telemetry read), with margin.
const CYCLIC_BUF: usize = crate::ethercat::domain::MAX_IMAGE + 64;
/// Maximum datagrams appended after the LRW in one cycle (Operational multi-
/// slave appends an ARMW + one telemetry read; the rest are single).
const MAX_APPENDED: usize = 3;
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

/// One datagram appended after the cyclic LRW, recorded in build order so the
/// reply walker can dispatch each by kind.
#[derive(Clone, Copy, Debug)]
enum Appended {
    /// AL-control write (= OP) to a slave; nothing to decode from the reply.
    Control,
    /// AL-status read gating the slave at `idx`'s SAFE-OP -> OP step.
    Status { idx: usize },
    /// Continuous DC reference-time distribution (ARMW 0x0910); best-effort.
    DcArmw,
    /// DC system-time-difference read (FPRD 0x092C) for drift monitoring.
    DcDiff,
    /// Operational round-robin AL-status read (catches a slave dropping OP).
    AlPoll,
}

/// The decoded effect of one appended reply, applied after the receive-buffer
/// borrow is released so disjoint engine state can be mutated freely.
#[derive(Clone, Copy)]
enum AppliedReply {
    None,
    /// Slave `idx` reported OP (during `RequestingOp`).
    OpReached { idx: usize },
    /// A slave reported an AL error, or dropped OP while operational.
    Fault,
    /// A decoded DC system-time difference (signed ns).
    Drift(i32),
}

/// One slave the cyclic engine drives to OP and monitors. Built by the master
/// from the discovered topology paired with the compile-time bus config.
#[derive(Clone, Copy)]
pub struct CyclicSlave {
    /// Configured station address (the FPRD/FPWR target).
    pub station: u16,
    /// Ring position (auto-increment order; identifies the DC reference).
    pub ring_pos: u16,
}

/// Per-slave runtime state inside the engine.
#[derive(Clone, Copy)]
struct SlaveRt {
    station: u16,
    ring_pos: u16,
    /// Set once this slave's AL status reads OP during `RequestingOp`.
    reached_op: bool,
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
    /// Best-case (floor) interrupt latency from the PIT fire to the ISR (ns).
    pub latency_min_ns: u32,
    /// Worst-case interrupt latency from the PIT fire to the ISR (ns).
    pub latency_max_ns: u32,
    /// Scheduling jitter: the latency spread (worst - best), in ns. This is the
    /// headline metric -- it goes non-zero only when entry is actually delayed.
    pub jitter_ns: u32,
    /// Worst-case interrupt latency expressed in core (CPU) cycles.
    pub latency_max_cyc: u32,
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
    /// Slaves driven to OP / monitored, in ring order.
    slaves: Vec<SlaveRt, EC_MAX_SLAVES>,
    /// Ring position of the DC reference clock (the ARMW auto-increment base).
    ref_ring_pos: u16,
    phase: Phase,
    tx: [u8; CYCLIC_BUF],
    tx_len: usize,
    rx: [u8; CYCLIC_BUF],
    outstanding: bool,
    expected_index: u8,
    good_cycles: u32,
    total_cycles: u32,
    /// Datagrams appended after the LRW this cycle, in build order.
    appended: Vec<Appended, MAX_APPENDED>,
    /// `RequestingOp`: index of the slave currently being driven to OP.
    op_cursor: usize,
    /// `RequestingOp`: alternates the AL-control write / AL-status read.
    op_ctrl_phase: bool,
    /// `Operational`: alternates the telemetry read between DC drift and a
    /// round-robin per-slave AL-status poll.
    mon_toggle: bool,
    /// `Operational`: rotating slave index for the AL-status poll.
    mon_cursor: usize,
    /// Configured cyclic period (ns); drives the rate/period report.
    cycle_ns: u64,
    /// Best / worst interrupt latency from the PIT fire (perclk ticks). `min`
    /// starts at its sentinel until the first tick records a sample.
    lat_min_ticks: u32,
    lat_max_ticks: u32,
    /// Latest / largest-magnitude DC system-time difference (signed ns).
    dc_diff_ns: i32,
    dc_diff_max_ns: i32,
    /// Whether a DC system-time difference has ever been decoded.
    dc_valid: bool,
    /// CiA-402 drive sequencer (controlword owner when a host is attached).
    cia402: Cia402,
}

impl Cyclic {
    /// Create the engine for `slaves` (all already at SAFE-OP), with the DC
    /// reference at ring position `ref_ring_pos`, running at a `cycle_ns`
    /// period. Built from the compile-time bus configuration; the cyclic engine
    /// drives every slave the rest of the way to OP. Telemetry stats start fresh
    /// (reset on every `start`).
    pub fn new(slaves: &[CyclicSlave], ref_ring_pos: u16, cycle_ns: u64) -> Self {
        let mut rt = Vec::new();
        for s in slaves {
            let _ = rt.push(SlaveRt {
                station: s.station,
                ring_pos: s.ring_pos,
                reached_op: false,
            });
        }
        Self {
            domain: EcDomain::from_config(&crate::ethercat::config::generated::BUS),
            slaves: rt,
            ref_ring_pos,
            phase: Phase::Priming,
            tx: [0; CYCLIC_BUF],
            tx_len: 0,
            rx: [0; CYCLIC_BUF],
            outstanding: false,
            expected_index: 0,
            good_cycles: 0,
            total_cycles: 0,
            appended: Vec::new(),
            op_cursor: 0,
            op_ctrl_phase: true,
            mon_toggle: true,
            mon_cursor: 0,
            cycle_ns,
            lat_min_ticks: u32::MAX,
            lat_max_ticks: 0,
            dc_diff_ns: 0,
            dc_diff_max_ns: 0,
            dc_valid: false,
            cia402: Cia402::new(),
        }
    }

    /// The configured station addresses this engine drives, in ring order. Used
    /// by `stop` to bring every drive down cleanly before tearing it down.
    pub fn stations(&self) -> Vec<u16, EC_MAX_SLAVES> {
        let mut v = Vec::new();
        for s in &self.slaves {
            let _ = v.push(s.station);
        }
        v
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
        // No latency sampled yet leaves `min` at its sentinel; report 0 instead.
        let min_ticks = if self.lat_min_ticks == u32::MAX { 0 } else { self.lat_min_ticks };
        let max_ticks = self.lat_max_ticks;
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
            latency_min_ns: ticks_to_ns(min_ticks),
            latency_max_ns: ticks_to_ns(max_ticks),
            jitter_ns: ticks_to_ns(max_ticks.saturating_sub(min_ticks)),
            latency_max_cyc: ticks_to_core_cyc(max_ticks),
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

    /// Sample the interrupt latency (jitter accounting) and advance the cycle
    /// counter. Called first in each tick so the latency read is as close to ISR
    /// entry as possible.
    fn begin_tick(&mut self) {
        // Absolute interrupt latency from the PIT hardware fire (LDVAL - CVAL).
        // Unlike a tick-to-tick interval -- which is always exactly the period
        // because the PIT and core clocks are synchronous and the WFI wake is
        // deterministic, hiding all jitter -- this measures the real delay from
        // the timer firing to this handler running, so its spread exposes
        // scheduling jitter under load.
        self.record_latency(cycle_timer::latency_ticks());
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

    /// Fold one interrupt-latency sample (PIT ticks) into the min/max stats.
    fn record_latency(&mut self, ticks: u32) {
        if ticks < self.lat_min_ticks {
            self.lat_min_ticks = ticks;
        }
        if ticks > self.lat_max_ticks {
            self.lat_max_ticks = ticks;
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

    /// Parse the reply frame (LRW + any appended datagrams) and update state.
    /// Each appended datagram is decoded into a Copy `AppliedReply` while the
    /// receive buffer is borrowed, then applied afterwards so disjoint engine
    /// state can be mutated without aliasing the `self.rx` borrow.
    fn process(&mut self, len: usize) {
        // Decode the reply while the receive buffer is borrowed, returning only
        // owned/Copy values so the borrow ends before disjoint state is mutated.
        let (lrw_wkc, actions) = {
            let frame = &self.rx[..len];
            let (lrw, mut next) = match datagram::parse_at(frame, EC_FRAME_HEADER_SIZE) {
                Some(v) => v,
                None => return,
            };
            let lrw_wkc = lrw.working_counter;
            self.domain.apply_reply(&lrw);
            // Walk the appended datagrams in the exact order they were built.
            let mut actions: Vec<AppliedReply, MAX_APPENDED> = Vec::new();
            for &kind in self.appended.iter() {
                if next == 0 {
                    break;
                }
                let (dg, n2) = match datagram::parse_at(frame, next) {
                    Some(v) => v,
                    None => break,
                };
                next = n2;
                let _ = actions.push(decode_appended(kind, &dg));
            }
            (lrw_wkc, actions)
        };

        // Process-data health + the Priming -> RequestingOp gate (whole-bus WKC).
        if lrw_wkc > 0 {
            self.good_cycles = self.good_cycles.saturating_add(1);
        } else {
            self.good_cycles = 0;
        }
        if self.phase == Phase::Priming && self.good_cycles >= PRIMING_CYCLES {
            self.phase = Phase::RequestingOp;
        }

        // Apply each appended reply's decoded effect. A zero working counter
        // already decoded to `None`, so a read that did not reach its slave is
        // ignored rather than faulting the bus.
        for a in actions.iter() {
            match *a {
                AppliedReply::OpReached { idx } => {
                    if let Some(s) = self.slaves.get_mut(idx) {
                        s.reached_op = true;
                    }
                }
                AppliedReply::Fault => self.phase = Phase::Faulted,
                AppliedReply::Drift(v) => {
                    self.dc_diff_ns = v;
                    self.dc_valid = true;
                    if v.unsigned_abs() > self.dc_diff_max_ns.unsigned_abs() {
                        self.dc_diff_max_ns = v;
                    }
                }
                AppliedReply::None => {}
            }
        }

        // Enter steady operation only once EVERY slave has reported OP.
        if self.phase == Phase::RequestingOp && self.all_reached_op() {
            self.phase = Phase::Operational;
        }
    }

    /// Whether every driven slave has reported OP.
    fn all_reached_op(&self) -> bool {
        self.slaves.iter().all(|s| s.reached_op)
    }

    /// The station of a non-reference slave -- where the 0x092C system-time
    /// difference reflects real drift from the distributed reference time. Falls
    /// back to the first slave if only the reference is present.
    fn follower_station(&self) -> u16 {
        self.slaves
            .iter()
            .find(|s| s.ring_pos != self.ref_ring_pos)
            .or_else(|| self.slaves.first())
            .map(|s| s.station)
            .unwrap_or(0)
    }

    /// During `RequestingOp`, append one per-slave AL datagram: walk to the next
    /// slave still short of OP, then alternate an AL-control write (= OP) with an
    /// AL-status read, mirroring the v1 single-slave handshake per slave. All the
    /// while the LRW keeps process data flowing.
    fn append_op_request(&mut self, index: &mut u8) {
        while self.op_cursor < self.slaves.len() && self.slaves[self.op_cursor].reached_op {
            self.op_cursor += 1;
            self.op_ctrl_phase = true;
        }
        let idx = self.op_cursor;
        let station = match self.slaves.get(idx) {
            Some(s) => s.station,
            None => return, // all reached OP; the next `process` enters Operational
        };
        let ai = alloc_index(index);
        if self.op_ctrl_phase {
            self.tx_len = datagram::append(
                &mut self.tx,
                ai,
                Command::Fpwr,
                station,
                reg::AL_CONTROL,
                &[al_state::OP, 0],
            );
            let _ = self.appended.push(Appended::Control);
        } else {
            self.tx_len = datagram::append(
                &mut self.tx,
                ai,
                Command::Fprd,
                station,
                reg::AL_STATUS,
                &[0u8, 0u8],
            );
            let _ = self.appended.push(Appended::Status { idx });
        }
        self.op_ctrl_phase = !self.op_ctrl_phase;
    }

    /// During `Operational`, append the monitoring tail. A single slave is its
    /// own DC reference, so only its drift read is meaningful (byte-identical to
    /// the v1 frame). With two or more slaves, first distribute the reference DC
    /// time to the followers (ARMW reads 0x0910 at the reference and writes it to
    /// every other slave so they stay disciplined and 0x092C reflects real
    /// drift), then alternate a follower drift read with a round-robin AL-status
    /// poll so a slave dropping OP is caught.
    fn append_monitor(&mut self, index: &mut u8) {
        if self.slaves.len() <= 1 {
            if let Some(s) = self.slaves.first() {
                let ai = alloc_index(index);
                self.tx_len = datagram::append(
                    &mut self.tx,
                    ai,
                    Command::Fprd,
                    s.station,
                    reg::DC_SYS_TIME_DIFF,
                    &[0u8; 4],
                );
                let _ = self.appended.push(Appended::DcDiff);
            }
            return;
        }

        // Continuous DC reference-time distribution (ARMW @ 0x0910).
        let ai = alloc_index(index);
        let adp = datagram::autoinc_adp(self.ref_ring_pos);
        self.tx_len =
            datagram::append(&mut self.tx, ai, Command::Armw, adp, reg::DC_SYS_TIME, &[0u8; 8]);
        let _ = self.appended.push(Appended::DcArmw);

        // Telemetry: alternate a follower drift read with a per-slave AL poll.
        let ti = alloc_index(index);
        if self.mon_toggle {
            let station = self.follower_station();
            self.tx_len = datagram::append(
                &mut self.tx,
                ti,
                Command::Fprd,
                station,
                reg::DC_SYS_TIME_DIFF,
                &[0u8; 4],
            );
            let _ = self.appended.push(Appended::DcDiff);
        } else {
            self.mon_cursor = (self.mon_cursor + 1) % self.slaves.len();
            let station = self.slaves[self.mon_cursor].station;
            self.tx_len = datagram::append(
                &mut self.tx,
                ti,
                Command::Fprd,
                station,
                reg::AL_STATUS,
                &[0u8, 0u8],
            );
            let _ = self.appended.push(Appended::AlPoll);
        }
        self.mon_toggle = !self.mon_toggle;
    }

    /// Build and send this cycle's frame (the whole-bus LRW, plus per-slave AL
    /// datagrams while requesting OP, or the DC/AL monitoring tail in OP).
    fn send(&mut self, dev: &mut Device, index: &mut u8) {
        let i = alloc_index(index);
        self.tx_len = self.domain.build_lrw(&mut self.tx, i);
        self.expected_index = i;
        self.appended.clear();

        if self.phase == Phase::RequestingOp {
            self.append_op_request(index);
        } else if self.phase == Phase::Operational {
            self.append_monitor(index);
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

/// Convert a duration in PIT ticks (perclk domain) to nanoseconds. PERCLK is
/// the 24 MHz crystal oscillator, so one tick is 1e9 / 24e6 ≈ 41.7 ns.
#[inline]
fn ticks_to_ns(ticks: u32) -> u32 {
    (ticks as u64 * 1_000_000_000 / PERCLK_HZ as u64) as u32
}

/// Convert a duration in PIT ticks (perclk domain) to core (CPU) cycles by the
/// clock ratio, so the jitter can be read in the same units as the CPU budget.
#[inline]
fn ticks_to_core_cyc(ticks: u32) -> u32 {
    (ticks as u64 * CORE_CLOCK_HZ as u64 / PERCLK_HZ as u64) as u32
}

/// Decode one appended reply into the effect to apply after the receive-buffer
/// borrow ends. A zero working counter (the datagram never reached its slave)
/// always decodes to `None`, so a dropped read can never fault the bus.
fn decode_appended(kind: Appended, dg: &datagram::Reply<'_>) -> AppliedReply {
    match kind {
        // Writes / best-effort time distribution carry nothing to interpret.
        Appended::Control | Appended::DcArmw => AppliedReply::None,
        Appended::Status { idx } => {
            if dg.working_counter == 0 {
                return AppliedReply::None;
            }
            let status = dg.data.first().copied().unwrap_or(0);
            if status & al_state::ERROR != 0 {
                AppliedReply::Fault
            } else if status & al_state::MASK == al_state::OP {
                AppliedReply::OpReached { idx }
            } else {
                AppliedReply::None
            }
        }
        Appended::AlPoll => {
            // Operational health: a slave that responded but is no longer OP
            // faults the engine, mirroring the single-slave Faulted semantics.
            if dg.working_counter == 0 {
                return AppliedReply::None;
            }
            let status = dg.data.first().copied().unwrap_or(0);
            if status & al_state::ERROR != 0 || status & al_state::MASK != al_state::OP {
                AppliedReply::Fault
            } else {
                AppliedReply::None
            }
        }
        Appended::DcDiff => {
            if dg.working_counter > 0 && dg.data.len() >= 4 {
                let raw = u32::from_le_bytes([dg.data[0], dg.data[1], dg.data[2], dg.data[3]]);
                AppliedReply::Drift(decode_dc_diff(raw))
            } else {
                AppliedReply::None
            }
        }
    }
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
