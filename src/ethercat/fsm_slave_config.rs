//! Per-slave configuration FSM (bring-up INIT -> SAFE-OP).
//!
//! IgH: master/fsm_slave_config.c (`ec_fsm_slave_config_t`) - drives one slave
//! through the AL states, clearing/configuring FMMUs and sync managers, applying
//! SDO init values, PDO assignment/mapping, the DC cycle, and the watchdog.
//! Rust: an `enum Phase` + `match` stepped one datagram at a time over
//! `Device::pump`, composing `fsm_change` (AL handshake), `fsm_coe`/`CoeSeq`
//! (CoE), `fsm_pdo`/`fsm_pdo_entry` (PDO config), `dc::FsmDc` (SYNC0), and the
//! `sync`/`fmmu_config` page encoders.
//!
//! v1 brings the slave up to SAFE-OP; the cyclic engine requests OP once the LRW
//! is exchanging with a good working counter.

use crate::ethercat::config::model::SlaveCfg;
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::dc::FsmDc;
use crate::ethercat::ecrt::EcError;
use crate::ethercat::fmmu_config;
use crate::ethercat::fsm_change::FsmChange;
use crate::ethercat::fsm_coe::CoeSeq;
use crate::ethercat::fsm_pdo;
use crate::ethercat::fsm_pdo_entry;
use crate::ethercat::globals::{al_state, fmmu, reg, EC_SYNC_PAGE_SIZE};
use crate::ethercat::slave::Mailbox;
use crate::ethercat::sync;

const PUMP_MAX_ATTEMPTS: u32 = 2_000;
/// Watchdog divider (0x0400): 100 us time base (Beckhoff ESC default).
const WD_DIVIDER_VALUE: u16 = 0x09C2;
/// Process-data watchdog time (0x0420): ~200 ms (2000 * 100 us). Generous so a
/// little cycle jitter during bring-up never faults the drive (tune per ESC).
const WD_PDATA_VALUE: u16 = 0x07D0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Phase {
    ClearFmmus,
    DcClear,
    MboxSm0,
    MboxSm1,
    ToPreop,
    SdoInit,
    PdoMapping,
    PdoAssign,
    PdSm,
    WatchdogDiv,
    WatchdogTime,
    Fmmu,
    DcCycle,
    ToSafeop,
    Done,
}

/// Non-blocking per-slave bring-up FSM.
pub struct FsmSlaveConfig {
    slave: &'static SlaveCfg,
    station: u16,
    mbox: Mailbox,
    phase: Phase,
    pump: Pump,
    tx: [u8; 96],
    tx_len: usize,
    rx: [u8; 128],
    change: Option<FsmChange>,
    seq: Option<CoeSeq>,
    dc: Option<FsmDc>,
    sm_idx: usize,
    pdo_idx: usize,
    fmmu_idx: usize,
}

impl FsmSlaveConfig {
    pub fn new(station: u16, mbox: Mailbox, slave: &'static SlaveCfg) -> Self {
        Self {
            slave,
            station,
            mbox,
            phase: Phase::ClearFmmus,
            pump: Pump::new(),
            tx: [0; 96],
            tx_len: 0,
            rx: [0; 128],
            change: None,
            seq: None,
            dc: None,
            sm_idx: 0,
            pdo_idx: 0,
            fmmu_idx: 0,
        }
    }

    /// Advance one datagram. `Ok(true)` once the slave reaches SAFE-OP.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.phase {
            Phase::ClearFmmus => {
                // Clear all FMMU pages (3 FMMUs on this drive).
                if self.fpwr(dev, index, reg::FMMU_BASE, &[0u8; 48])? {
                    self.goto(Phase::DcClear);
                }
                Ok(false)
            }
            Phase::DcClear => {
                // Disable DC activation before reconfiguring it.
                if self.fpwr(dev, index, reg::DC_ACTIVATION, &[0u8; 2])? {
                    self.goto(Phase::MboxSm0);
                }
                Ok(false)
            }
            Phase::MboxSm0 => {
                let mut page = [0u8; EC_SYNC_PAGE_SIZE as usize];
                sync::write_mailbox_out(&mut page, self.mbox.rx_offset, self.mbox.rx_size);
                if self.fpwr(dev, index, reg::SM0, &page)? {
                    self.goto(Phase::MboxSm1);
                }
                Ok(false)
            }
            Phase::MboxSm1 => {
                let mut page = [0u8; EC_SYNC_PAGE_SIZE as usize];
                sync::write_mailbox_in(&mut page, self.mbox.tx_offset, self.mbox.tx_size);
                if self.fpwr(dev, index, reg::SM1, &page)? {
                    self.goto(Phase::ToPreop);
                }
                Ok(false)
            }
            Phase::ToPreop => {
                if self.run_change(dev, index, al_state::PREOP)? {
                    // SDO init values run as one expedited sequence.
                    let mut seq = CoeSeq::new(self.mbox);
                    for sdo in self.slave.sdo_init {
                        let _ = seq.push(sdo.index, sdo.subindex, sdo.data);
                    }
                    self.seq = Some(seq);
                    self.goto(Phase::SdoInit);
                }
                Ok(false)
            }
            Phase::SdoInit => {
                if self.run_seq(dev, index)? {
                    self.sm_idx = 0;
                    self.pdo_idx = 0;
                    self.goto(Phase::PdoMapping);
                }
                Ok(false)
            }
            Phase::PdoMapping => {
                // Write each PDO's mapping (0x16xx/0x1A0x), SM by SM.
                if self.sm_idx >= self.slave.sms.len() {
                    self.sm_idx = 0;
                    self.goto(Phase::PdoAssign);
                    return Ok(false);
                }
                let sm = &self.slave.sms[self.sm_idx];
                if self.pdo_idx >= sm.pdos.len() {
                    self.sm_idx += 1;
                    self.pdo_idx = 0;
                    return Ok(false);
                }
                if self.seq.is_none() {
                    self.seq = Some(fsm_pdo_entry::build_mapping(self.mbox, &sm.pdos[self.pdo_idx]));
                }
                if self.run_seq(dev, index)? {
                    self.pdo_idx += 1;
                }
                Ok(false)
            }
            Phase::PdoAssign => {
                // Write each SM's PDO assignment (0x1C12/0x1C13).
                if self.sm_idx >= self.slave.sms.len() {
                    self.sm_idx = 0;
                    self.goto(Phase::PdSm);
                    return Ok(false);
                }
                let sm = &self.slave.sms[self.sm_idx];
                if self.seq.is_none() {
                    self.seq = Some(fsm_pdo::build_assign(self.mbox, sm.index, sm.pdos));
                }
                if self.run_seq(dev, index)? {
                    self.sm_idx += 1;
                }
                Ok(false)
            }
            Phase::PdSm => {
                // Process-data SM pages (SM2/SM3): FPWR 0x0800 + n*8.
                if self.sm_idx >= self.slave.sms.len() {
                    self.goto(Phase::WatchdogDiv);
                    return Ok(false);
                }
                let sm = self.slave.sms[self.sm_idx];
                let mut page = [0u8; EC_SYNC_PAGE_SIZE as usize];
                match sm.dir {
                    crate::ethercat::ecrt::EcDirection::Output => {
                        sync::write_process_out(&mut page, sm.phys_start, sm.size, sm.control)
                    }
                    crate::ethercat::ecrt::EcDirection::Input => {
                        sync::write_process_in(&mut page, sm.phys_start, sm.size, sm.control)
                    }
                }
                let ado = reg::SM0 + (sm.index as u16) * EC_SYNC_PAGE_SIZE;
                if self.fpwr(dev, index, ado, &page)? {
                    self.sm_idx += 1;
                }
                Ok(false)
            }
            Phase::WatchdogDiv => {
                if self.fpwr(dev, index, reg::WD_DIVIDER, &WD_DIVIDER_VALUE.to_le_bytes())? {
                    self.goto(Phase::WatchdogTime);
                }
                Ok(false)
            }
            Phase::WatchdogTime => {
                if self.fpwr(dev, index, reg::WD_PDATA, &WD_PDATA_VALUE.to_le_bytes())? {
                    self.fmmu_idx = 0;
                    self.goto(Phase::Fmmu);
                }
                Ok(false)
            }
            Phase::Fmmu => {
                if self.fmmu_idx >= self.slave.fmmus.len() {
                    self.goto(Phase::DcCycle);
                    return Ok(false);
                }
                let mut page = [0u8; fmmu::PAGE_SIZE];
                fmmu_config::write_page(&mut page, &self.slave.fmmus[self.fmmu_idx]);
                let ado = reg::FMMU_BASE + (self.fmmu_idx as u16) * (fmmu::PAGE_SIZE as u16);
                if self.fpwr(dev, index, ado, &page)? {
                    self.fmmu_idx += 1;
                }
                Ok(false)
            }
            Phase::DcCycle => {
                match self.slave.dc {
                    None => {
                        self.goto(Phase::ToSafeop);
                        Ok(false)
                    }
                    Some(cfg) => {
                        if self.dc.is_none() {
                            self.dc = Some(FsmDc::new(self.station, cfg));
                        }
                        if self.dc.as_mut().unwrap().step(dev, index)? {
                            self.dc = None;
                            self.goto(Phase::ToSafeop);
                        }
                        Ok(false)
                    }
                }
            }
            Phase::ToSafeop => {
                if self.run_change(dev, index, al_state::SAFEOP)? {
                    self.phase = Phase::Done;
                    return Ok(true);
                }
                Ok(false)
            }
            Phase::Done => Ok(true),
        }
    }

    fn goto(&mut self, next: Phase) {
        self.phase = next;
        self.tx_len = 0;
    }

    /// Issue an FPWR to `ado`; returns true once acknowledged (WKC == 1).
    fn fpwr(&mut self, dev: &mut Device, index: &mut u8, ado: u16, payload: &[u8]) -> Result<bool, EcError> {
        if self.tx_len == 0 {
            let i = alloc_index(index);
            self.tx_len = datagram::build(&mut self.tx, i, Command::Fpwr, self.station, ado, payload);
            self.pump.reset();
        }
        match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
            None => Ok(false),
            Some(len) => {
                let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                if reply.working_counter != 1 {
                    return Err(EcError::WorkingCounter);
                }
                self.tx_len = 0;
                Ok(true)
            }
        }
    }

    /// Drive the embedded AL state-change FSM toward `target`.
    fn run_change(&mut self, dev: &mut Device, index: &mut u8, target: u8) -> Result<bool, EcError> {
        if self.change.is_none() {
            self.change = Some(FsmChange::new(self.station, target));
        }
        if self.change.as_mut().unwrap().step(dev, index)? {
            self.change = None;
            return Ok(true);
        }
        Ok(false)
    }

    /// Drive the embedded CoE sequence; clears it when complete.
    fn run_seq(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.seq.as_mut() {
            None => Ok(true),
            Some(seq) => {
                if seq.step(dev, index)? {
                    self.seq = None;
                    Ok(true)
                } else {
                    Ok(false)
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
