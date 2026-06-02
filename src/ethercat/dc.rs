//! Distributed Clocks (DC) SYNC0 setup.
//!
//! IgH: no dedicated file -- DC logic is spread across master/master.c,
//! master/slave_config.c (`ecrt_slave_config_dc`) and master/fsm_slave_config.c.
//! This file consolidates the bring-up: latch the (reference) drive's local
//! time, read its DC system time, program the SYNC0 cycle and a future cyclic
//! start time, then activate SYNC0 via the `assignActivate` word.
//! Rust: a non-blocking `enum State` stepper over `Device::pump`, one datagram
//! per step, mirroring the other FSMs.
//!
//! v1 scope: a single drive that is its own reference clock, so cross-slave
//! offset/drift compensation (ARMW/FRMW, register 0x092C) is not needed. The
//! code path generalises to a `ref_clock_slave` + followers later.

use crate::ethercat::config::model::DcCfg;
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::{read_u64_le, write_u16_le, write_u32_le, write_u64_le, EcError};
use crate::ethercat::globals::reg;

const PUMP_MAX_ATTEMPTS: u32 = 2_000;
/// Cyclic start time is placed this far in the future, so SYNC0 activation has
/// settled on every drive before the first pulse.
const START_MARGIN_NS: u64 = 100_000_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    LatchTime,
    ReadTime,
    WriteCycle,
    WriteStart,
    Activate,
    Done,
}

/// Non-blocking DC SYNC0 configuration FSM for one slave.
pub struct FsmDc {
    station: u16,
    cfg: DcCfg,
    state: State,
    pump: Pump,
    tx: [u8; 64],
    tx_len: usize,
    rx: [u8; 128],
    start_time: u64,
}

impl FsmDc {
    pub fn new(station: u16, cfg: DcCfg) -> Self {
        Self {
            station,
            cfg,
            state: State::LatchTime,
            pump: Pump::new(),
            tx: [0; 64],
            tx_len: 0,
            rx: [0; 128],
            start_time: 0,
        }
    }

    /// Advance one datagram. `Ok(true)` once SYNC0 is activated.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.state {
            // Write to the receive-time-latch register to capture local time.
            State::LatchTime => {
                let reply = self.transact(dev, index, Command::Fpwr, reg::DC_RECV_TIME, &[0u8; 4])?;
                if let Some(wkc) = reply {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::ReadTime);
                }
                Ok(false)
            }
            // Read the DC system time and compute a future cyclic start time.
            State::ReadTime => {
                let done = self.poll(dev, index, Command::Fprd, reg::DC_SYS_TIME, &[0u8; 8])?;
                if let Some(len) = done {
                    let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                    let now = if reply.data.len() >= 8 {
                        read_u64_le(&reply.data[0..8])
                    } else {
                        0
                    };
                    let cycle = (self.cfg.sync0_cycle_ns.max(1)) as u64;
                    let target = now.wrapping_add(START_MARGIN_NS);
                    self.start_time = ((target / cycle) + 1) * cycle + self.cfg.sync0_shift_ns as u64;
                    self.advance(State::WriteCycle);
                }
                Ok(false)
            }
            // SYNC0 cycle (and SYNC1) times, written as one 8-byte FPWR @ 0x09A0.
            State::WriteCycle => {
                let mut payload = [0u8; 8];
                write_u32_le(&mut payload[0..4], self.cfg.sync0_cycle_ns);
                write_u32_le(&mut payload[4..8], self.cfg.sync1_cycle_ns);
                let reply = self.transact(dev, index, Command::Fpwr, reg::DC_SYNC0_CYCLE, &payload)?;
                if let Some(wkc) = reply {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::WriteStart);
                }
                Ok(false)
            }
            // Cyclic start time (U64) @ 0x0990.
            State::WriteStart => {
                let mut payload = [0u8; 8];
                write_u64_le(&mut payload, self.start_time);
                let reply = self.transact(dev, index, Command::Fpwr, reg::DC_CYCLE_START, &payload)?;
                if let Some(wkc) = reply {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::Activate);
                }
                Ok(false)
            }
            // Activate SYNC0 via the assignActivate word @ 0x0980.
            State::Activate => {
                let mut payload = [0u8; 2];
                write_u16_le(&mut payload, self.cfg.assign_activate);
                let reply = self.transact(dev, index, Command::Fpwr, reg::DC_ACTIVATION, &payload)?;
                if let Some(wkc) = reply {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.state = State::Done;
                    return Ok(true);
                }
                Ok(false)
            }
            State::Done => Ok(true),
        }
    }

    fn advance(&mut self, next: State) {
        self.state = next;
        self.tx_len = 0;
    }

    /// Issue a write datagram; returns `Some(working_counter)` once the reply
    /// arrives, `None` while pending.
    fn transact(
        &mut self,
        dev: &mut Device,
        index: &mut u8,
        cmd: Command,
        ado: u16,
        payload: &[u8],
    ) -> Result<Option<u16>, EcError> {
        match self.poll(dev, index, cmd, ado, payload)? {
            None => Ok(None),
            Some(len) => {
                let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                Ok(Some(reply.working_counter))
            }
        }
    }

    /// Pump one datagram; returns `Some(reply_len)` once its reply arrives.
    fn poll(
        &mut self,
        dev: &mut Device,
        index: &mut u8,
        cmd: Command,
        ado: u16,
        payload: &[u8],
    ) -> Result<Option<usize>, EcError> {
        if self.tx_len == 0 {
            let i = alloc_index(index);
            self.tx_len = datagram::build(&mut self.tx, i, cmd, self.station, ado, payload);
            self.pump.reset();
        }
        dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)
    }
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}
