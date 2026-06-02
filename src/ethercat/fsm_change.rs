//! AL (Application Layer) state-change FSM (non-blocking).
//!
//! IgH: master/fsm_change.c, master/fsm_change.h (`ec_fsm_change_t`) - the
//! requested/acknowledged AL state handshake: write AL control (0x0120), poll
//! AL status (0x0130), and on failure read the AL status code (0x0134).
//! Rust: `enum State` + `match` stepped by `step()`, one datagram per step via
//! `Device::pump` (no busy-wait), so it can be driven by the async worker now
//! and the cyclic PDO task later. `Result<_, EcError>` instead of int codes.
//! Dropped (kernel-only): `jiffies` timeouts -> bounded step/wait counters.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::EcError;
use crate::ethercat::globals::{al_state, reg};

/// Max poll ticks awaiting a single datagram's reply before declaring timeout.
const PUMP_MAX_ATTEMPTS: u32 = 2_000;
/// Max AL-status re-reads while waiting for the target state to settle.
const MAX_STATE_WAITS: u32 = 1_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    WriteControl,
    ReadStatus,
    ReadStatusCode,
    Done,
}

/// Non-blocking AL state-change FSM for one slave.
pub struct FsmChange {
    station: u16,
    target: u8,
    state: State,
    pump: Pump,
    tx: [u8; 64],
    tx_len: usize,
    rx: [u8; 128],
    waits: u32,
}

impl FsmChange {
    /// Request `target` (an `al_state::*` value) on the slave at `station`.
    pub fn new(station: u16, target: u8) -> Self {
        Self {
            station,
            target,
            state: State::WriteControl,
            pump: Pump::new(),
            tx: [0; 64],
            tx_len: 0,
            rx: [0; 128],
            waits: 0,
        }
    }

    /// Advance one datagram. `Ok(true)` = target reached, `Ok(false)` = pending,
    /// `Err` = transport/timeout or `StateChange(code)` on AL error.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.state {
            State::WriteControl => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fpwr,
                        self.station,
                        reg::AL_CONTROL,
                        &[self.target, 0],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        self.state = State::ReadStatus;
                        self.tx_len = 0;
                        self.waits = 0;
                        Ok(false)
                    }
                }
            }
            State::ReadStatus => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fprd,
                        self.station,
                        reg::AL_STATUS,
                        &[0u8; 2],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        let status = reply.data.first().copied().unwrap_or(0);
                        if status & al_state::ERROR != 0 {
                            self.state = State::ReadStatusCode;
                            self.tx_len = 0;
                            return Ok(false);
                        }
                        if status & al_state::MASK == self.target {
                            self.state = State::Done;
                            return Ok(true);
                        }
                        // Still transitioning: re-read AL status next step.
                        self.tx_len = 0;
                        self.waits += 1;
                        if self.waits >= MAX_STATE_WAITS {
                            return Err(EcError::StateChange(0));
                        }
                        Ok(false)
                    }
                }
            }
            State::ReadStatusCode => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fprd,
                        self.station,
                        reg::AL_STATUS_CODE,
                        &[0u8; 2],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        let code = if reply.data.len() >= 2 {
                            u16::from_le_bytes([reply.data[0], reply.data[1]])
                        } else {
                            0
                        };
                        Err(EcError::StateChange(code))
                    }
                }
            }
            State::Done => Ok(true),
        }
    }
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}
