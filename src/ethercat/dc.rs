//! Distributed Clocks (DC) bring-up: static offset/delay compensation + SYNC0.
//!
//! IgH: no dedicated file -- DC logic is spread across master/fsm_master.c (the
//! bus-wide delay measurement) and master/fsm_slave_config.c (per-slave offset +
//! SYNC0 activation). This file consolidates both for one slave: a non-blocking
//! `enum State` stepper over `Device::pump`, one frame per step, mirroring the
//! other FSMs.
//!
//! Two paths share the FSM:
//!
//! * **Reference clock / single-slave bus** (`DcRef::compensate == false`):
//!   latch the drive's local time, read its system time, program the SYNC0 cycle
//!   and a future cyclic start time, then activate SYNC0. Byte-identical to v1 --
//!   the single-slave wire path is unchanged.
//!
//! * **Follower** (`DcRef::compensate == true`): before touching SYNC0, measure
//!   the static error against the reference and write it once, so the follower
//!   starts aligned instead of letting the continuous cyclic ARMW chase a ~33 ms
//!   clock-start offset down from scratch. Steps:
//!
//!   1. BWR 0x0900 -- latch every port's local receive time, bus-wide.
//!   2. FPRD the reference + follower port receive times (0x0900, 16 B) in one
//!      frame -> propagation delay from the within-slave port-1/port-0 deltas.
//!   3. FPWR the propagation delay to 0x0928 (U32).
//!   4. FPRD the reference system time + follower system time + follower offset
//!      (0x0910/0x0910/0x0920) in one frame, so the two clocks are sampled a
//!      single frame-transit apart (sub-us) -> `new_offset = old_offset +
//!      (reference_system_time - follower_system_time)`.
//!   5. FPWR the offset to 0x0920 (U64) -- removes the bulk clock-start error.
//!
//!   Then the same SYNC0 cycle/start/activate steps run. The continuous ARMW in
//!   the cyclic engine now only maintains the tiny residual (0x092C is sub-us
//!   from the first read) rather than converging the full offset.

use crate::ethercat::config::model::DcCfg;
use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::{
    read_u32_le, read_u64_le, write_u16_le, write_u32_le, write_u64_le, EcError,
};
use crate::ethercat::globals::{reg, EC_FRAME_HEADER_SIZE};

const PUMP_MAX_ATTEMPTS: u32 = 2_000;
/// Cyclic start time is placed this far in the future, so SYNC0 activation has
/// settled on every drive before the first pulse.
const START_MARGIN_NS: u64 = 100_000_000;
/// Largest plausible one-way propagation delay (ns) for a short EtherCAT line.
/// A measured delay outside `0..=MAX_DELAY_NS` is treated as a bad reading and
/// written as 0 (offset-only), which still yields sub-us alignment.
const MAX_DELAY_NS: u32 = 10_000;

/// Reference-clock context for one slave's DC bring-up.
///
/// For the reference clock itself (or a single-slave bus) `compensate` is false
/// and [`FsmDc`] runs the plain per-slave SYNC0 setup, preserving the v1 wire
/// path exactly. For a follower it carries the reference's configured station so
/// the static offset/delay can be measured against it before SYNC0 activation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DcRef {
    /// Configured station address of the DC reference clock slave.
    pub ref_station: u16,
    /// Whether this slave is a follower that needs offset/delay compensation.
    pub compensate: bool,
}

impl DcRef {
    /// The no-compensation case: this slave is its own reference (single-slave
    /// bus or the reference clock). Keeps `FsmDc` on the v1 path.
    pub const fn reference() -> Self {
        Self {
            ref_station: 0,
            compensate: false,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    // Follower-only static compensation, measured against the reference and
    // applied before SYNC0 is activated (mirrors IgH's DC delay measurement).
    CompLatch,
    CompReadPorts,
    CompWriteDelay,
    CompReadTimes,
    CompWriteOffset,
    // Per-slave SYNC0 setup. The only path for the reference / single-slave bus;
    // followers join it at `WriteCycle` after compensation.
    LatchTime,
    ReadTime,
    WriteCycle,
    WriteStart,
    Activate,
    Done,
}

/// Non-blocking DC configuration FSM for one slave.
pub struct FsmDc {
    station: u16,
    cfg: DcCfg,
    dc_ref: DcRef,
    state: State,
    pump: Pump,
    tx: [u8; 96],
    tx_len: usize,
    rx: [u8; 128],
    start_time: u64,
    /// System-time offset to write to 0x0920 (follower path).
    offset: u64,
    /// Propagation delay to write to 0x0928 (follower path).
    delay: u32,
}

impl FsmDc {
    pub fn new(station: u16, cfg: DcCfg, dc_ref: DcRef) -> Self {
        let state = if dc_ref.compensate {
            State::CompLatch
        } else {
            State::LatchTime
        };
        Self {
            station,
            cfg,
            dc_ref,
            state,
            pump: Pump::new(),
            tx: [0; 96],
            tx_len: 0,
            rx: [0; 128],
            start_time: 0,
            offset: 0,
            delay: 0,
        }
    }

    /// Advance one datagram. `Ok(true)` once SYNC0 is activated.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.state {
            // Broadcast-write the receive-time-latch register so every slave
            // latches each port's local receive time at the same frame.
            State::CompLatch => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len =
                        datagram::build(&mut self.tx, i, Command::Bwr, 0x0000, reg::DC_RECV_TIME, &[0u8; 4]);
                    self.pump.reset();
                }
                if let Some(len) = self.pump_current(dev)? {
                    let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                    if reply.working_counter == 0 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::CompReadPorts);
                }
                Ok(false)
            }
            // Read the reference's and follower's port receive times (16 B each)
            // in one frame; the within-slave port deltas give the cable delay.
            State::CompReadPorts => {
                if self.tx_len == 0 {
                    let i0 = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i0,
                        Command::Fprd,
                        self.dc_ref.ref_station,
                        reg::DC_RECV_TIME,
                        &[0u8; 16],
                    );
                    let i1 = alloc_index(index);
                    self.tx_len = datagram::append(
                        &mut self.tx,
                        i1,
                        Command::Fprd,
                        self.station,
                        reg::DC_RECV_TIME,
                        &[0u8; 16],
                    );
                    self.pump.reset();
                }
                if let Some(len) = self.pump_current(dev)? {
                    let (ref_ports, fol_ports) = parse_two_port_reads(&self.rx[..len])?;
                    self.delay = propagation_delay(&ref_ports, &fol_ports);
                    self.advance(State::CompWriteDelay);
                }
                Ok(false)
            }
            // Write the propagation delay (U32) to 0x0928.
            State::CompWriteDelay => {
                let mut payload = [0u8; 4];
                write_u32_le(&mut payload, self.delay);
                if let Some(wkc) =
                    self.transact(dev, index, Command::Fpwr, reg::DC_SYS_TIME_DELAY, &payload)?
                {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::CompReadTimes);
                }
                Ok(false)
            }
            // Read the reference system time, follower system time, and follower
            // offset (8 B each) in one frame -> the two clocks are sampled a
            // single frame-transit apart, so the offset is accurate to sub-us.
            State::CompReadTimes => {
                if self.tx_len == 0 {
                    let i0 = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i0,
                        Command::Fprd,
                        self.dc_ref.ref_station,
                        reg::DC_SYS_TIME,
                        &[0u8; 8],
                    );
                    let i1 = alloc_index(index);
                    self.tx_len = datagram::append(
                        &mut self.tx,
                        i1,
                        Command::Fprd,
                        self.station,
                        reg::DC_SYS_TIME,
                        &[0u8; 8],
                    );
                    let i2 = alloc_index(index);
                    self.tx_len = datagram::append(
                        &mut self.tx,
                        i2,
                        Command::Fprd,
                        self.station,
                        reg::DC_SYS_TIME_OFFSET,
                        &[0u8; 8],
                    );
                    self.pump.reset();
                }
                if let Some(len) = self.pump_current(dev)? {
                    let (ref_sys, fol_sys, old_off) = parse_three_u64_reads(&self.rx[..len])?;
                    // Accumulate so repeated `start`s stay convergent: the
                    // follower system time already includes `old_off`, so the
                    // new absolute offset is `old_off + (ref - follower)`.
                    self.offset = old_off.wrapping_add(ref_sys.wrapping_sub(fol_sys));
                    // Compute the SYNC0 start time from the reference timebase so
                    // every follower lands on the same cycle boundary phase.
                    self.start_time = cyclic_start(&self.cfg, ref_sys);
                    self.advance(State::CompWriteOffset);
                }
                Ok(false)
            }
            // Write the system-time offset (U64) to 0x0920 -- removes the bulk
            // clock-start error -- then join the SYNC0 cycle write.
            State::CompWriteOffset => {
                let mut payload = [0u8; 8];
                write_u64_le(&mut payload, self.offset);
                if let Some(wkc) =
                    self.transact(dev, index, Command::Fpwr, reg::DC_SYS_TIME_OFFSET, &payload)?
                {
                    if wkc != 1 {
                        return Err(EcError::WorkingCounter);
                    }
                    self.advance(State::WriteCycle);
                }
                Ok(false)
            }
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
                    self.start_time = cyclic_start(&self.cfg, now);
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

    /// Issue a single-datagram write/read to `self.station`; returns
    /// `Some(working_counter)` once the reply arrives, `None` while pending.
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

    /// Build (on the first call) and pump one single-datagram frame addressed to
    /// `self.station`; returns `Some(reply_len)` once its reply arrives.
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
        self.pump_current(dev)
    }

    /// Pump the already-built `self.tx`; returns `Some(reply_len)` on a match.
    fn pump_current(&mut self, dev: &mut Device) -> Result<Option<usize>, EcError> {
        dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)
    }
}

/// Future SYNC0 cyclic start time: the next cycle boundary at least
/// `START_MARGIN_NS` ahead of `now`, plus the configured shift.
#[inline]
fn cyclic_start(cfg: &DcCfg, now: u64) -> u64 {
    let cycle = (cfg.sync0_cycle_ns.max(1)) as u64;
    let target = now.wrapping_add(START_MARGIN_NS);
    ((target / cycle) + 1) * cycle + cfg.sync0_shift_ns as u64
}

/// One-way propagation delay from the reference to the follower on a two-slave
/// line, from the per-slave port-1/port-0 receive-time deltas (each within one
/// slave's local timebase): `((ref_p1 - ref_p0) - (fol_p1 - fol_p0)) / 2`. The
/// reference's delta is the round trip below its downstream port (cable to the
/// follower and back); the follower (leaf) contributes only its own loopback,
/// so the difference halved is the one-way cable delay. Returns 0 (offset-only,
/// still sub-us) for any out-of-range result -- a non-line topology, swapped
/// ports, or a stale latch.
fn propagation_delay(ref_ports: &[u32; 4], fol_ports: &[u32; 4]) -> u32 {
    let ref_rt = ref_ports[1].wrapping_sub(ref_ports[0]) as i64;
    let fol_rt = fol_ports[1].wrapping_sub(fol_ports[0]) as i64;
    let one_way = (ref_rt - fol_rt) / 2;
    if (0..=MAX_DELAY_NS as i64).contains(&one_way) {
        one_way as u32
    } else {
        0
    }
}

/// Parse two consecutive FPRD replies (16-byte port-receive-time reads) from a
/// combined frame, returning each slave's four 32-bit port times. Errors if a
/// datagram is missing/short or did not reach its slave (WKC != 1).
fn parse_two_port_reads(frame: &[u8]) -> Result<([u32; 4], [u32; 4]), EcError> {
    let (d0, n1) = datagram::parse_at(frame, EC_FRAME_HEADER_SIZE).ok_or(EcError::FrameTooShort)?;
    let (d1, _) = datagram::parse_at(frame, n1).ok_or(EcError::FrameTooShort)?;
    if d0.working_counter != 1 || d1.working_counter != 1 {
        return Err(EcError::WorkingCounter);
    }
    Ok((read_ports(d0.data)?, read_ports(d1.data)?))
}

fn read_ports(data: &[u8]) -> Result<[u32; 4], EcError> {
    if data.len() < 16 {
        return Err(EcError::FrameTooShort);
    }
    Ok([
        read_u32_le(&data[0..4]),
        read_u32_le(&data[4..8]),
        read_u32_le(&data[8..12]),
        read_u32_le(&data[12..16]),
    ])
}

/// Parse three consecutive 8-byte FPRD replies (reference system time, follower
/// system time, follower system-time offset) from a combined frame.
fn parse_three_u64_reads(frame: &[u8]) -> Result<(u64, u64, u64), EcError> {
    let (d0, n1) = datagram::parse_at(frame, EC_FRAME_HEADER_SIZE).ok_or(EcError::FrameTooShort)?;
    let (d1, n2) = datagram::parse_at(frame, n1).ok_or(EcError::FrameTooShort)?;
    let (d2, _) = datagram::parse_at(frame, n2).ok_or(EcError::FrameTooShort)?;
    if d0.working_counter != 1 || d1.working_counter != 1 || d2.working_counter != 1 {
        return Err(EcError::WorkingCounter);
    }
    Ok((read_u64(d0.data)?, read_u64(d1.data)?, read_u64(d2.data)?))
}

fn read_u64(data: &[u8]) -> Result<u64, EcError> {
    if data.len() < 8 {
        return Err(EcError::FrameTooShort);
    }
    Ok(read_u64_le(&data[0..8]))
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}
