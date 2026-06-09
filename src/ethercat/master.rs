//! EtherCAT master orchestration.
//!
//! IgH: master/master.c, master/master.h (`ec_master_t`) plus the embeddable
//! parts of master/module.c - the central object owning the network device,
//! slaves, and the FSMs, and running the work cycle.
//! Rust: owns the `device` transport and a `heapless::Vec<SlaveInfo>`. The bus
//! scan stays blocking (startup, pre-OP). Runtime commands (state change, CoE
//! SDO up/download) run as non-blocking `Op` steppers driven one datagram at a
//! time by `poll_op`, so the async worker (now) and a cyclic PDO task (later)
//! can advance them without blocking.
//! Dropped (kernel-only): `module_init`, the `kthread` FSM loop, `cdev`/`ioctl`,
//! `io_sem`/spinlocks -> RTIC tasks + resources.

use crate::ethercat::config::generated::BUS;
use crate::ethercat::cyclic::{Cyclic, CyclicSlave, CyclicStatus};
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::dc::DcRef;
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::EcError;
use crate::ethercat::fsm_change::FsmChange;
use crate::ethercat::fsm_coe::FsmCoe;
use crate::ethercat::fsm_master;
use crate::ethercat::fsm_scan::{ScanFsm, ScanStep, TraceLine};
use crate::ethercat::fsm_slave_config::FsmSlaveConfig;
use crate::ethercat::globals::{al_state, reg, EC_MAX_SLAVES};
use crate::ethercat::slave::{Mailbox, SlaveInfo};
use crate::ethercat::sync;
use crate::net::ethernet;
use heapless::Vec;

const PUMP_MAX_ATTEMPTS: u32 = 2_000;

/// A runtime request for the master to execute (built by the serial CLI).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Request {
    /// Re-run the (blocking) bus scan.
    Rescan,
    /// Request an AL state on a slave.
    SetState { slave: u16, target: u8 },
    /// CoE SDO upload (read).
    SdoUpload { slave: u16, index: u16, subindex: u8 },
    /// CoE SDO download (write) of `len` little-endian bytes.
    SdoDownload {
        slave: u16,
        index: u16,
        subindex: u8,
        data: [u8; 4],
        len: u8,
    },
    /// Configure EVERY desired slave to SAFE-OP and start the cyclic process-
    /// data engine, running at a `cycle_ns` period (drives both the PIT and the
    /// jitter baseline). The engine then drives all slaves to OP.
    StartCyclic { cycle_ns: u64 },
    /// Stop the cyclic engine.
    StopCyclic,
}

/// The result of a completed [`Request`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Outcome {
    /// Rescan finished; carries the slave count.
    Rescanned(usize),
    /// The slave reached the requested AL state.
    StateReached(u8),
    /// SDO upload finished; bytes are in [`Master::sdo_buf`].
    SdoUploaded(usize),
    /// SDO download acknowledged.
    SdoDownloaded,
    /// The cyclic engine was configured (slave at SAFE-OP) and started.
    CyclicStarted,
    /// The cyclic engine was stopped.
    CyclicStopped,
}

/// The EtherCAT master: owns the transport and the discovered bus topology.
pub struct Master<'a> {
    device: Device<'a>,
    slaves: Vec<SlaveInfo, EC_MAX_SLAVES>,
    index: u8,
    op: Option<Op>,
    sdo_buf: [u8; 4],
    sdo_len: usize,
    cyclic: Option<Cyclic>,
    /// Latest scan progress line, consumed by the worker for serial streaming.
    scan_trace: Option<TraceLine>,
}

impl<'a> Master<'a> {
    /// Create a master over an initialized transport `Device`.
    pub fn new(device: Device<'a>) -> Self {
        Self {
            device,
            slaves: Vec::new(),
            index: 0,
            op: None,
            sdo_buf: [0; 4],
            sdo_len: 0,
            cyclic: None,
            scan_trace: None,
        }
    }

    /// Take the latest bus-scan progress line (set by the non-blocking scan FSM
    /// as each sub-step completes) so the worker can stream it over serial.
    pub fn take_scan_trace(&mut self) -> Option<TraceLine> {
        self.scan_trace.take()
    }

    /// True when the PHY reports link up (safe to start the scan).
    pub fn link_up(&mut self) -> bool {
        ethernet::read_link_state(self.device.enet_mut()).unwrap_or(false)
    }

    /// Run a full (blocking) bus scan; stores the slaves and returns the count.
    pub fn scan(&mut self) -> Result<usize, EcError> {
        self.slaves = fsm_master::scan_bus(&mut self.device)?;
        Ok(self.slaves.len())
    }

    /// Slaves discovered by the most recent successful scan.
    pub fn slaves(&self) -> &[SlaveInfo] {
        &self.slaves
    }

    /// The most recent SDO upload response bytes.
    pub fn sdo_buf(&self) -> &[u8] {
        &self.sdo_buf[..self.sdo_len]
    }

    /// Whether a runtime operation is currently in progress.
    pub fn op_active(&self) -> bool {
        self.op.is_some()
    }

    /// Begin a runtime request. Validates the slave/CoE support up front.
    pub fn begin(&mut self, request: Request) -> Result<(), EcError> {
        let op = match request {
            Request::Rescan => {
                // The scan FSM no longer holds a slave list; it streams each
                // finished slave into `self.slaves`. Clear the previous scan's
                // results so the new scan starts from an empty topology.
                self.slaves.clear();
                Op::Rescan { fsm: ScanFsm::new() }
            }
            Request::SetState { slave, target } => {
                let s = self.slave_copy(slave)?;
                let configure = needs_mailbox(target);
                Op::State {
                    pre: PreOp::new(s.station_addr, s.mailbox(), target, configure),
                }
            }
            Request::SdoUpload {
                slave,
                index,
                subindex,
            } => {
                let s = self.slave_copy(slave)?;
                if !s.supports_coe {
                    return Err(EcError::CoeUnsupported);
                }
                Op::Sdo {
                    slave,
                    pre: PreOp::new(s.station_addr, s.mailbox(), sdo_state(&s), true),
                    coe: None,
                    request: SdoOp::Upload { index, subindex },
                }
            }
            Request::SdoDownload {
                slave,
                index,
                subindex,
                data,
                len,
            } => {
                let s = self.slave_copy(slave)?;
                if !s.supports_coe {
                    return Err(EcError::CoeUnsupported);
                }
                Op::Sdo {
                    slave,
                    pre: PreOp::new(s.station_addr, s.mailbox(), sdo_state(&s), true),
                    coe: None,
                    request: SdoOp::Download {
                        index,
                        subindex,
                        data,
                        len,
                    },
                }
            }
            Request::StartCyclic { cycle_ns } => {
                // Bring up EVERY configured slave to SAFE-OP (one per-slave FSM at
                // a time), then the cyclic engine drives them all to OP. Requires
                // a prior scan; the LRW spans the whole bus, so a single slave is
                // just the N == 1 case. The `slave` CLI position is retained for
                // compatibility but no longer selects a subset.
                if self.slaves.is_empty() {
                    return Err(EcError::NoSuchSlave);
                }
                Op::StartCyclic {
                    seq: ConfigSeq::new(),
                    cycle_ns,
                }
            }
            Request::StopCyclic => {
                // Defense in depth: bring EVERY drive down to PRE-OP before the
                // cyclic LRW stops, so no slave's SM2 (output) watchdog times out
                // and latches an AL error. The down-transition is best-effort
                // (see `drive`); if the engine isn't running there's nothing to
                // bring down. The PIT is already stopped by the `stop` CLI path,
                // so the device is free for the state-change datagrams here.
                let stations = self.cyclic.as_ref().map(|c| c.stations()).unwrap_or_default();
                Op::StopCyclic {
                    down: DownSeq::new(stations),
                }
            }
        };
        self.op = Some(op);
        Ok(())
    }

    /// Advance the active operation by one datagram. Returns `None` while
    /// pending, `Some(result)` when it completes (or fails).
    pub fn poll_op(&mut self) -> Option<Result<Outcome, EcError>> {
        let mut op = self.op.take()?;
        match self.drive(&mut op) {
            Ok(None) => {
                self.op = Some(op);
                None
            }
            Ok(Some(outcome)) => Some(Ok(outcome)),
            Err(e) => Some(Err(e)),
        }
    }

    fn drive(&mut self, op: &mut Op) -> Result<Option<Outcome>, EcError> {
        match op {
            Op::Rescan { fsm } => {
                let step = fsm.step(&mut self.device, &mut self.index)?;
                self.scan_trace = fsm.take_trace();
                // Drain the slave this step finished (at most one) straight into
                // our list, so the FSM never accumulates a per-slave Vec.
                if let Some(slave) = fsm.take_completed_slave() {
                    let _ = self.slaves.push(slave);
                }
                match step {
                    ScanStep::Pending => Ok(None),
                    ScanStep::Done => Ok(Some(Outcome::Rescanned(self.slaves.len()))),
                }
            }
            Op::State { pre } => {
                if pre.step(&mut self.device, &mut self.index)? {
                    self.update_state(pre.station, pre.target);
                    Ok(Some(Outcome::StateReached(pre.target)))
                } else {
                    Ok(None)
                }
            }
            Op::Sdo {
                slave,
                pre,
                coe,
                request,
            } => {
                if coe.is_none() {
                    if !pre.step(&mut self.device, &mut self.index)? {
                        return Ok(None);
                    }
                    self.update_state(pre.station, pre.target);
                    let mbox = self
                        .slaves
                        .get(*slave as usize)
                        .map(|s| s.mailbox())
                        .ok_or(EcError::NoSuchSlave)?;
                    *coe = Some(request.build(mbox));
                    return Ok(None);
                }
                let coe_fsm = coe.as_mut().unwrap();
                if !coe_fsm.step(&mut self.device, &mut self.index)? {
                    return Ok(None);
                }
                match request {
                    SdoOp::Upload { .. } => {
                        let resp = coe_fsm.response();
                        let n = resp.len().min(4);
                        self.sdo_buf[..n].copy_from_slice(&resp[..n]);
                        self.sdo_len = n;
                        Ok(Some(Outcome::SdoUploaded(n)))
                    }
                    SdoOp::Download { .. } => Ok(Some(Outcome::SdoDownloaded)),
                }
            }
            Op::StartCyclic { seq, cycle_ns } => {
                if seq.step(&mut self.device, &mut self.index, &self.slaves)? {
                    // Every configured slave is at SAFE-OP now; reflect it in the
                    // tracked AL state (the cyclic engine drives them to OP next).
                    for cs in seq.ready.iter() {
                        self.update_state(cs.station, al_state::SAFEOP);
                    }
                    self.cyclic =
                        Some(Cyclic::new(&seq.ready, BUS.ref_clock_slave, *cycle_ns));
                    Ok(Some(Outcome::CyclicStarted))
                } else {
                    Ok(None)
                }
            }
            Op::StopCyclic { down } => {
                // Step the per-slave down-transition to PRE-OP while pending. It
                // is best-effort: any error (link down, slave gone) is skipped so
                // `stop` always tears the engine down and the operator regains
                // control. The error-ack on the next `start` is the backstop if
                // this doesn't complete.
                let (done, reached) = down.step(&mut self.device, &mut self.index);
                if let Some(station) = reached {
                    self.update_state(station, al_state::PREOP);
                }
                if !done {
                    return Ok(None);
                }
                self.cyclic = None;
                Ok(Some(Outcome::CyclicStopped))
            }
        }
    }

    /// Whether the cyclic process-data engine is running.
    pub fn cyclic_active(&self) -> bool {
        self.cyclic.is_some()
    }

    /// Advance the cyclic engine by one cycle. Called from the high-priority PIT
    /// task; processes the previous reply and sends this cycle's frame.
    pub fn cyclic_tick(&mut self) {
        if let Some(cyclic) = self.cyclic.as_mut() {
            cyclic.tick(&mut self.device, &mut self.index);
        }
    }

    /// Advance the cyclic engine for one cycle, integrated with the Pi/LinuxCNC
    /// host bridge (apply staged outputs + drive control, then snapshot the
    /// reply). Called from the PIT task while holding both the master and the
    /// shared `HostBridge`.
    pub fn host_cycle(&mut self, host: &mut crate::hal::host_bridge::HostBridge) {
        if let Some(cyclic) = self.cyclic.as_mut() {
            cyclic.tick_with_host(&mut self.device, &mut self.index, host);
        }
    }

    /// A snapshot of cyclic health, if the engine is running.
    pub fn cyclic_status(&self) -> Option<CyclicStatus> {
        self.cyclic.as_ref().map(|c| c.status())
    }

    /// The process-data image (read), if the engine is running.
    pub fn cyclic_image(&self) -> Option<&[u8]> {
        self.cyclic.as_ref().map(|c| c.image())
    }

    /// The process-data image (write; sets outputs), if the engine is running.
    pub fn cyclic_image_mut(&mut self) -> Option<&mut [u8]> {
        self.cyclic.as_mut().map(|c| c.image_mut())
    }

    fn slave_copy(&self, slave: u16) -> Result<SlaveInfo, EcError> {
        self.slaves
            .get(slave as usize)
            .copied()
            .ok_or(EcError::NoSuchSlave)
    }

    fn update_state(&mut self, station: u16, target: u8) {
        for s in self.slaves.iter_mut() {
            if s.station_addr == station {
                s.al_state = target & al_state::MASK;
            }
        }
    }
}

/// Whether a target AL state needs the mailbox sync managers configured.
fn needs_mailbox(target: u8) -> bool {
    matches!(target, al_state::PREOP | al_state::SAFEOP | al_state::OP)
}

/// The state to run a CoE transfer in: keep a slave that is already in a
/// mailbox-capable state (PRE-OP/SAFE-OP/OP) where it is, otherwise bring it up
/// to PRE-OP. Avoids downshifting (halting) a SAFE-OP/OP slave just to do an SDO.
fn sdo_state(s: &SlaveInfo) -> u8 {
    let current = s.al_state & al_state::MASK;
    if needs_mailbox(current) {
        current
    } else {
        al_state::PREOP
    }
}

/// The CoE-specific part of an SDO operation.
enum SdoOp {
    Upload {
        index: u16,
        subindex: u8,
    },
    Download {
        index: u16,
        subindex: u8,
        data: [u8; 4],
        len: u8,
    },
}

impl SdoOp {
    fn build(&self, mbox: Mailbox) -> FsmCoe {
        match self {
            SdoOp::Upload { index, subindex } => FsmCoe::upload(mbox, *index, *subindex),
            SdoOp::Download {
                index,
                subindex,
                data,
                len,
            } => FsmCoe::download(mbox, *index, *subindex, &data[..*len as usize]),
        }
    }
}

/// An in-flight runtime operation.
///
/// Variants embed their FSMs inline (the scan/config FSMs carry per-datagram
/// scratch buffers). The master lives in a single static cell, so the enum size
/// is a one-time static cost; boxing would need an allocator we do not have.
#[allow(clippy::large_enum_variant)]
enum Op {
    Rescan {
        fsm: ScanFsm,
    },
    State {
        pre: PreOp,
    },
    Sdo {
        slave: u16,
        pre: PreOp,
        coe: Option<FsmCoe>,
        request: SdoOp,
    },
    StartCyclic {
        /// Per-slave bring-up sequencer (all configured slaves -> SAFE-OP).
        seq: ConfigSeq,
        cycle_ns: u64,
    },
    StopCyclic {
        /// Best-effort per-slave down-transition to PRE-OP, stepped before the
        /// engine is dropped so no drive's SM watchdog latches an AL error.
        down: DownSeq,
    },
}

/// Brings every configured slave to SAFE-OP in ring order, one slave's
/// `FsmSlaveConfig` at a time, collecting each slave's (station, ring position)
/// for the cyclic engine. Mirrors IgH configuring all slaves before the domain
/// goes operational; here the per-slave FSM is unchanged and simply run N times.
struct ConfigSeq {
    /// Index into `BUS.slaves` of the slave currently being configured.
    idx: usize,
    /// The active per-slave bring-up FSM (created lazily for `idx`).
    fsm: Option<FsmSlaveConfig>,
    /// Slaves collected as each is matched to the scan + configured.
    ready: Vec<CyclicSlave, EC_MAX_SLAVES>,
}

impl ConfigSeq {
    fn new() -> Self {
        Self {
            idx: 0,
            fsm: None,
            ready: Vec::new(),
        }
    }

    /// Advance the bring-up by one datagram. `Ok(true)` once every configured
    /// slave has reached SAFE-OP. Errors if a configured slave is not present on
    /// the scanned bus (don't bring up a partial bus that would fail the WKC).
    fn step(
        &mut self,
        dev: &mut Device,
        index: &mut u8,
        slaves: &[SlaveInfo],
    ) -> Result<bool, EcError> {
        let cfgs = BUS.slaves;
        if self.idx >= cfgs.len() {
            return Ok(true);
        }
        if self.fsm.is_none() {
            let cfg = &cfgs[self.idx];
            // Match the configured slave to a discovered one by ring position.
            let info = slaves
                .iter()
                .find(|s| s.ring_pos == cfg.position)
                .ok_or(EcError::NoSuchSlave)?;
            let _ = self.ready.push(CyclicSlave {
                station: info.station_addr,
                ring_pos: info.ring_pos,
            });
            // DC reference context: a follower (any slave that is not the DC
            // reference) gets static offset/delay compensation measured against
            // the reference before SYNC0 activation. The reference clock and a
            // single-slave bus keep the unchanged per-slave SYNC0 path.
            let dc_ref = match slaves.iter().find(|s| s.ring_pos == BUS.ref_clock_slave) {
                Some(r) if r.station_addr != info.station_addr => DcRef {
                    ref_station: r.station_addr,
                    compensate: true,
                },
                _ => DcRef::reference(),
            };
            self.fsm = Some(FsmSlaveConfig::new(
                info.station_addr,
                info.mailbox(),
                cfg,
                dc_ref,
            ));
        }
        if self.fsm.as_mut().unwrap().step(dev, index)? {
            self.fsm = None;
            self.idx += 1;
        }
        Ok(self.idx >= cfgs.len())
    }
}

/// Walks every cyclic slave down to PRE-OP, one `FsmChange` at a time, before
/// the engine is dropped. Best-effort: a slave that errors (link down, gone) is
/// skipped so the sequence always converges and `stop` always completes.
struct DownSeq {
    stations: Vec<u16, EC_MAX_SLAVES>,
    idx: usize,
    change: Option<FsmChange>,
}

impl DownSeq {
    fn new(stations: Vec<u16, EC_MAX_SLAVES>) -> Self {
        Self {
            stations,
            idx: 0,
            change: None,
        }
    }

    /// Advance by one datagram. Returns `(done, reached_preop)` where `done` is
    /// true once every slave has been driven down (or skipped) and
    /// `reached_preop` is the station that just reached PRE-OP (to update the
    /// tracked AL state).
    fn step(&mut self, dev: &mut Device, index: &mut u8) -> (bool, Option<u16>) {
        if self.idx >= self.stations.len() {
            return (true, None);
        }
        let station = self.stations[self.idx];
        if self.change.is_none() {
            self.change = Some(FsmChange::new(station, al_state::PREOP));
        }
        match self.change.as_mut().unwrap().step(dev, index) {
            Ok(false) => (false, None),
            Ok(true) => {
                self.change = None;
                self.idx += 1;
                (self.idx >= self.stations.len(), Some(station))
            }
            Err(_) => {
                self.change = None;
                self.idx += 1;
                (self.idx >= self.stations.len(), None)
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PreStage {
    ConfigSm0,
    ConfigSm1,
    Change,
}

/// Brings a slave to a target state, configuring the mailbox sync managers
/// first when the target needs the mailbox (PRE-OP and above).
struct PreOp {
    station: u16,
    target: u8,
    configure: bool,
    mbox: Mailbox,
    stage: PreStage,
    pump: Pump,
    tx: [u8; 64],
    tx_len: usize,
    rx: [u8; 128],
    change: FsmChange,
}

impl PreOp {
    fn new(station: u16, mbox: Mailbox, target: u8, configure: bool) -> Self {
        Self {
            station,
            target,
            configure,
            mbox,
            stage: PreStage::ConfigSm0,
            pump: Pump::new(),
            tx: [0; 64],
            tx_len: 0,
            rx: [0; 128],
            change: FsmChange::new(station, target),
        }
    }

    /// Advance one datagram. `Ok(true)` when the target state is reached.
    fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.stage {
            PreStage::ConfigSm0 => {
                if !self.configure {
                    self.stage = PreStage::Change;
                    return Ok(false);
                }
                if self.tx_len == 0 {
                    let mut page = [0u8; sync::PAGE_SIZE];
                    sync::write_mailbox_out(&mut page, self.mbox.rx_offset, self.mbox.rx_size);
                    let i = alloc_index(index);
                    self.tx_len =
                        datagram::build(&mut self.tx, i, Command::Fpwr, self.station, reg::SM0, &page);
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        self.stage = PreStage::ConfigSm1;
                        self.tx_len = 0;
                        Ok(false)
                    }
                }
            }
            PreStage::ConfigSm1 => {
                if self.tx_len == 0 {
                    let mut page = [0u8; sync::PAGE_SIZE];
                    sync::write_mailbox_in(&mut page, self.mbox.tx_offset, self.mbox.tx_size);
                    let i = alloc_index(index);
                    self.tx_len =
                        datagram::build(&mut self.tx, i, Command::Fpwr, self.station, reg::SM1, &page);
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        self.stage = PreStage::Change;
                        self.tx_len = 0;
                        Ok(false)
                    }
                }
            }
            PreStage::Change => self.change.step(dev, index),
        }
    }
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}
