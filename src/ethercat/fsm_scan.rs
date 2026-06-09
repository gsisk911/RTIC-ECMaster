//! Non-blocking bus-scan FSM.
//!
//! IgH: the scan-orchestration subset of master/fsm_master.c +
//! master/fsm_slave_scan.c + master/fsm_sii.c, but driven one datagram at a
//! time (like the SDO/state FSMs) instead of blocking. Each completed sub-step
//! records a short `trace` line so the worker can stream scan progress over
//! serial -- our only window into where a real-bus scan goes wrong without SWD.
//! Rust: a single `ScanFsm` with an explicit `Stage` enum + a `Pump` per
//! datagram; the SII read is an inner `Sii` sub-stepper. Replaces the blocking
//! `fsm_master::scan_bus` on the `Op::Rescan` path.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::EcError;
use crate::ethercat::globals::{
    al_state, mbox, reg, sii, sii_ctrl, EC_MAX_SLAVES, EC_SCAN_FRAME_BUF,
};
use crate::ethercat::slave::SlaveInfo;
use core::fmt::Write;
use heapless::String;

/// Poll budget for one datagram's reply before declaring a timeout. Each unit is
/// one `poll_op` tick (the worker yields between them), so this bounds a stuck
/// transaction without busy-waiting.
const MAX_ATTEMPTS: u32 = 2_000;
/// Status-poll budget for one SII word read before declaring the EEPROM stuck.
const SII_MAX_POLLS: u32 = 400;

/// One step of [`ScanFsm::step`].
pub enum ScanStep {
    /// More datagrams remain; call `step` again next tick.
    Pending,
    /// The scan finished. Each discovered slave was already handed to the caller
    /// one at a time via [`ScanFsm::take_completed_slave`] as the scan completed
    /// it; there is nothing left to drain.
    Done,
}

/// A trace line (matches the worker's response-line width).
pub type TraceLine = String<96>;

fn trace_line(args: core::fmt::Arguments) -> TraceLine {
    let mut s = TraceLine::new();
    let _ = s.write_fmt(args);
    s
}

#[derive(Clone, Copy)]
enum Stage {
    Count,
    Clear,
    Apwr,
    Al,
    Dl,
    Sii(SiiField),
    Done,
}

#[derive(Clone, Copy)]
enum SiiField {
    Vendor,
    Product,
    Revision,
    RxMbox,
    TxMbox,
    Protocols,
}

fn sii_word(field: SiiField) -> u16 {
    match field {
        SiiField::Vendor => sii::VENDOR_ID,
        SiiField::Product => sii::PRODUCT_CODE,
        SiiField::Revision => sii::REVISION,
        SiiField::RxMbox => sii::STD_RX_MBOX,
        SiiField::TxMbox => sii::STD_TX_MBOX,
        SiiField::Protocols => sii::MBOX_PROTOCOLS,
    }
}

/// Identity/base fields accumulated for the slave currently being scanned.
#[derive(Clone, Copy, Default)]
struct Cur {
    ring_pos: u16,
    station: u16,
    al: u8,
    base_type: u8,
    base_fmmu_count: u8,
    base_sync_count: u8,
    vendor: u32,
    product: u32,
    revision: u32,
    rx_mbox: u32,
    tx_mbox: u32,
}

/// Non-blocking bus scan: count slaves, clear addresses, then per slave assign
/// the station address and read AL status, DL/base info, and SII identity.
///
/// Completed slaves are not accumulated here; each is published one at a time
/// through `completed` (drained by the caller via [`ScanFsm::take_completed_slave`])
/// so this FSM carries no per-slave `Vec` (which would inflate the `Op` the
/// driver moves onto the stack each `poll_op`).
pub struct ScanFsm {
    stage: Stage,
    count: u16,
    pump: Pump,
    tx: [u8; EC_SCAN_FRAME_BUF],
    tx_len: usize,
    rx: [u8; EC_SCAN_FRAME_BUF],
    cur: Cur,
    sii: Sii,
    /// The most recently finished slave, awaiting drain by the caller. At most
    /// one slave finishes per `step`, so a single slot never overwrites an
    /// undrained slave.
    completed: Option<SlaveInfo>,
    trace: Option<TraceLine>,
}

impl ScanFsm {
    pub fn new() -> Self {
        Self {
            stage: Stage::Count,
            count: 0,
            pump: Pump::new(),
            tx: [0; EC_SCAN_FRAME_BUF],
            tx_len: 0,
            rx: [0; EC_SCAN_FRAME_BUF],
            cur: Cur::default(),
            sii: Sii::new(),
            completed: None,
            // First flushed line: proves the scan started and is counting.
            trace: Some(trace_line(format_args!("[scan] counting slaves"))),
        }
    }

    /// Take the most recently produced trace line, if any.
    pub fn take_trace(&mut self) -> Option<TraceLine> {
        self.trace.take()
    }

    /// Take the slave the scan just finished, if one is awaiting drain. Call
    /// after each `step`; the caller pushes it into its own slave list.
    pub fn take_completed_slave(&mut self) -> Option<SlaveInfo> {
        self.completed.take()
    }

    /// Advance the scan by exactly one datagram poll.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<ScanStep, EcError> {
        match self.stage {
            Stage::Count => self.step_simple(
                dev,
                index,
                Command::Brd,
                0x0000,
                reg::AL_STATUS,
                2,
                |s, wkc, _data| {
                    s.count = wkc;
                    s.trace = Some(trace_line(format_args!("[scan] count={}", s.count)));
                    s.stage = if s.count == 0 { Stage::Done } else { Stage::Clear };
                    Ok(())
                },
            ),
            Stage::Clear => self.step_simple(
                dev,
                index,
                Command::Bwr,
                0x0000,
                reg::STATION_ADDR,
                2,
                |s, _wkc, _data| {
                    s.trace = Some(trace_line(format_args!("[scan] addresses cleared")));
                    s.begin_slave(0);
                    Ok(())
                },
            ),
            Stage::Apwr => {
                let ring = self.cur.ring_pos;
                let station = self.cur.station;
                let adp = datagram::autoinc_adp(ring);
                self.step_write(
                    dev,
                    index,
                    Command::Apwr,
                    adp,
                    reg::STATION_ADDR,
                    &station.to_le_bytes(),
                    |s, wkc, _data| {
                        if wkc != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        s.trace =
                            Some(trace_line(format_args!("[scan] s{}: addr set", s.cur.station)));
                        s.stage = Stage::Al;
                        Ok(())
                    },
                )
            }
            Stage::Al => {
                let station = self.cur.station;
                self.step_simple(dev, index, Command::Fprd, station, reg::AL_STATUS, 2, |s, wkc, data| {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    s.cur.al = data.first().copied().unwrap_or(0) & al_state::MASK;
                    s.trace = Some(trace_line(format_args!(
                        "[scan] s{}: al=0x{:02X}",
                        s.cur.station, s.cur.al
                    )));
                    s.stage = Stage::Dl;
                    Ok(())
                })
            }
            Stage::Dl => {
                let station = self.cur.station;
                self.step_simple(dev, index, Command::Fprd, station, reg::DL_INFO, 12, |s, wkc, data| {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    s.cur.base_type = data.first().copied().unwrap_or(0);
                    s.cur.base_fmmu_count = data.get(4).copied().unwrap_or(0);
                    s.cur.base_sync_count = data.get(5).copied().unwrap_or(0);
                    s.trace = Some(trace_line(format_args!(
                        "[scan] s{}: base type=0x{:02X} fmmu={} sm={}",
                        s.cur.station, s.cur.base_type, s.cur.base_fmmu_count, s.cur.base_sync_count
                    )));
                    s.sii.reset(sii_word(SiiField::Vendor));
                    s.stage = Stage::Sii(SiiField::Vendor);
                    Ok(())
                })
            }
            Stage::Sii(field) => {
                let station = self.cur.station;
                match self
                    .sii
                    .step(dev, station, index, &mut self.tx, &mut self.rx)?
                {
                    None => Ok(ScanStep::Pending),
                    Some(value) => {
                        self.apply_sii(field, value);
                        Ok(ScanStep::Pending)
                    }
                }
            }
            Stage::Done => Ok(ScanStep::Done),
        }
    }

    /// Start scanning the slave at `ring_pos` (station address `ring_pos + 1`).
    fn begin_slave(&mut self, ring_pos: u16) {
        self.cur = Cur {
            ring_pos,
            station: ring_pos + 1,
            ..Cur::default()
        };
        self.pump.reset();
        self.tx_len = 0;
        self.stage = Stage::Apwr;
    }

    /// Store one SII field, trace it, then advance to the next field/slave.
    fn apply_sii(&mut self, field: SiiField, value: u32) {
        let st = self.cur.station;
        match field {
            SiiField::Vendor => {
                self.cur.vendor = value;
                self.trace = Some(trace_line(format_args!("[scan] s{}: vendor=0x{:08X}", st, value)));
                self.sii.reset(sii_word(SiiField::Product));
                self.stage = Stage::Sii(SiiField::Product);
            }
            SiiField::Product => {
                self.cur.product = value;
                self.trace =
                    Some(trace_line(format_args!("[scan] s{}: product=0x{:08X}", st, value)));
                self.sii.reset(sii_word(SiiField::Revision));
                self.stage = Stage::Sii(SiiField::Revision);
            }
            SiiField::Revision => {
                self.cur.revision = value;
                self.trace = Some(trace_line(format_args!("[scan] s{}: rev=0x{:08X}", st, value)));
                self.sii.reset(sii_word(SiiField::RxMbox));
                self.stage = Stage::Sii(SiiField::RxMbox);
            }
            SiiField::RxMbox => {
                self.cur.rx_mbox = value;
                self.trace = Some(trace_line(format_args!(
                    "[scan] s{}: rxmbox off=0x{:04X} sz={}",
                    st,
                    value as u16,
                    (value >> 16) as u16
                )));
                self.sii.reset(sii_word(SiiField::TxMbox));
                self.stage = Stage::Sii(SiiField::TxMbox);
            }
            SiiField::TxMbox => {
                self.cur.tx_mbox = value;
                self.trace = Some(trace_line(format_args!(
                    "[scan] s{}: txmbox off=0x{:04X} sz={}",
                    st,
                    value as u16,
                    (value >> 16) as u16
                )));
                self.sii.reset(sii_word(SiiField::Protocols));
                self.stage = Stage::Sii(SiiField::Protocols);
            }
            SiiField::Protocols => {
                let protocols = value as u16;
                let supports_coe = protocols & mbox::PROTO_COE != 0;
                self.trace = Some(trace_line(format_args!(
                    "[scan] s{}: proto=0x{:04X} coe={}",
                    st, protocols, supports_coe as u8
                )));
                self.finish_slave(protocols, supports_coe);
            }
        }
    }

    /// Publish the completed slave for the caller to drain, then move to the next
    /// ring position or finish.
    fn finish_slave(&mut self, protocols: u16, supports_coe: bool) {
        let c = self.cur;
        let info = SlaveInfo {
            ring_pos: c.ring_pos,
            station_addr: c.station,
            al_state: c.al,
            base_type: c.base_type,
            base_fmmu_count: c.base_fmmu_count,
            base_sync_count: c.base_sync_count,
            vendor_id: c.vendor,
            product_code: c.product,
            revision: c.revision,
            rx_mbox_offset: c.rx_mbox as u16,
            rx_mbox_size: (c.rx_mbox >> 16) as u16,
            tx_mbox_offset: c.tx_mbox as u16,
            tx_mbox_size: (c.tx_mbox >> 16) as u16,
            mbox_protocols: protocols,
            supports_coe,
        };
        self.completed = Some(info);
        let next = c.ring_pos + 1;
        if next < self.count && (next as usize) < EC_MAX_SLAVES {
            self.begin_slave(next);
        } else {
            self.stage = Stage::Done;
        }
    }

    /// Drive a read datagram (data is `read_len` zero bytes) through one pump
    /// tick, invoking `on_reply(self, wkc, reply_data)` when the matching reply
    /// arrives. The reply payload is copied out first so the callback can borrow
    /// `self` mutably.
    #[allow(clippy::too_many_arguments)] // datagram target (cmd/adp/ado) + reply callback
    fn step_simple(
        &mut self,
        dev: &mut Device,
        index: &mut u8,
        cmd: Command,
        adp: u16,
        ado: u16,
        read_len: usize,
        on_reply: impl FnOnce(&mut Self, u16, &[u8]) -> Result<(), EcError>,
    ) -> Result<ScanStep, EcError> {
        if self.tx_len == 0 {
            let i = alloc_index(index);
            // Scan reads are tiny (AL status = 2 B, DL info = 12 B); the fixed
            // zero buffer bounds the read payload. Keep `read_len <= 12` for any
            // new scan field, or widen `zeros` to match.
            debug_assert!(read_len <= 12, "scan read_len exceeds the zero buffer");
            let zeros = [0u8; 12];
            self.tx_len = datagram::build(&mut self.tx, i, cmd, adp, ado, &zeros[..read_len]);
            self.pump.reset();
        }
        self.pump_reply(dev, on_reply)
    }

    /// Drive a write datagram (carrying `payload`) through one pump tick.
    #[allow(clippy::too_many_arguments)] // datagram target (cmd/adp/ado) + payload + reply callback
    fn step_write(
        &mut self,
        dev: &mut Device,
        index: &mut u8,
        cmd: Command,
        adp: u16,
        ado: u16,
        payload: &[u8],
        on_reply: impl FnOnce(&mut Self, u16, &[u8]) -> Result<(), EcError>,
    ) -> Result<ScanStep, EcError> {
        if self.tx_len == 0 {
            let i = alloc_index(index);
            self.tx_len = datagram::build(&mut self.tx, i, cmd, adp, ado, payload);
            self.pump.reset();
        }
        self.pump_reply(dev, on_reply)
    }

    /// Pump the already-built `self.tx`; on a matching reply, copy out the WKC +
    /// payload and hand them to `on_reply` (which may mutate `self`).
    fn pump_reply(
        &mut self,
        dev: &mut Device,
        on_reply: impl FnOnce(&mut Self, u16, &[u8]) -> Result<(), EcError>,
    ) -> Result<ScanStep, EcError> {
        match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, MAX_ATTEMPTS)? {
            None => Ok(ScanStep::Pending),
            Some(len) => {
                let mut data = [0u8; 16];
                let (wkc, dlen) = {
                    let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                    let n = reply.data.len().min(data.len());
                    data[..n].copy_from_slice(&reply.data[..n]);
                    (reply.working_counter, n)
                };
                self.tx_len = 0;
                on_reply(self, wkc, &data[..dlen])?;
                Ok(ScanStep::Pending)
            }
        }
    }
}

impl Default for ScanFsm {
    fn default() -> Self {
        Self::new()
    }
}

/// Non-blocking SII (EEPROM) word reader: issue the read command, then poll the
/// control/status register until the busy bit clears. Shares the caller's
/// tx/rx scratch buffers.
struct Sii {
    phase: SiiPhase,
    word: u16,
    polls: u32,
    pump: Pump,
    tx_len: usize,
}

#[derive(Clone, Copy)]
enum SiiPhase {
    Command,
    Poll,
}

impl Sii {
    fn new() -> Self {
        Self {
            phase: SiiPhase::Command,
            word: 0,
            polls: 0,
            pump: Pump::new(),
            tx_len: 0,
        }
    }

    /// Restart the reader for a new EEPROM `word` address.
    fn reset(&mut self, word: u16) {
        self.phase = SiiPhase::Command;
        self.word = word;
        self.polls = 0;
        self.pump.reset();
        self.tx_len = 0;
    }

    /// Advance one pump tick. `Ok(None)` pending, `Ok(Some(v))` when the 32-bit
    /// value (two SII words) is ready.
    fn step(
        &mut self,
        dev: &mut Device,
        station: u16,
        index: &mut u8,
        tx: &mut [u8; EC_SCAN_FRAME_BUF],
        rx: &mut [u8; EC_SCAN_FRAME_BUF],
    ) -> Result<Option<u32>, EcError> {
        match self.phase {
            SiiPhase::Command => {
                if self.tx_len == 0 {
                    let cmd = [
                        sii_ctrl::ADDR_MODE_TWO_OCTET,
                        sii_ctrl::CMD_READ,
                        (self.word & 0xFF) as u8,
                        (self.word >> 8) as u8,
                    ];
                    let i = alloc_index(index);
                    self.tx_len =
                        datagram::build(tx, i, Command::Fpwr, station, reg::SII_CONTROL, &cmd);
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &tx[..self.tx_len], rx, MAX_ATTEMPTS)? {
                    None => Ok(None),
                    Some(len) => {
                        let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter == 0 {
                            return Err(EcError::WorkingCounter);
                        }
                        self.phase = SiiPhase::Poll;
                        self.polls = 0;
                        self.tx_len = 0;
                        Ok(None)
                    }
                }
            }
            SiiPhase::Poll => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        tx,
                        i,
                        Command::Fprd,
                        station,
                        reg::SII_CONTROL,
                        &[0u8; 10],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &tx[..self.tx_len], rx, MAX_ATTEMPTS)? {
                    None => Ok(None),
                    Some(len) => {
                        let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
                        // Re-issue the status read on the next tick regardless.
                        self.tx_len = 0;
                        if reply.working_counter == 0 || reply.data.len() < 10 {
                            self.polls += 1;
                            if self.polls >= SII_MAX_POLLS {
                                return Err(EcError::SiiTimeout);
                            }
                            return Ok(None);
                        }
                        let status = reply.data[1];
                        if status & sii_ctrl::STATUS_ERROR != 0 {
                            return Err(EcError::SiiError);
                        }
                        if status & sii_ctrl::STATUS_BUSY == 0 {
                            return Ok(Some(u32::from_le_bytes([
                                reply.data[6],
                                reply.data[7],
                                reply.data[8],
                                reply.data[9],
                            ])));
                        }
                        self.polls += 1;
                        if self.polls >= SII_MAX_POLLS {
                            return Err(EcError::SiiTimeout);
                        }
                        Ok(None)
                    }
                }
            }
        }
    }
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}
