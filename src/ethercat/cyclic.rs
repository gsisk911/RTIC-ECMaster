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

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, ECAT_RX_LEN};
use crate::ethercat::domain::EcDomain;
use crate::ethercat::globals::{al_state, reg, EC_FRAME_HEADER_SIZE};

/// EtherCAT frame buffer for the cyclic LRW (+ optional appended datagram).
const CYCLIC_BUF: usize = crate::ethercat::domain::MAX_IMAGE + 32;
/// Consecutive responding cycles (WKC > 0) in `Priming` before requesting OP.
const PRIMING_CYCLES: u32 = 3;

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
}

/// A snapshot of cyclic health for reporting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CyclicStatus {
    pub phase: Phase,
    pub wkc: u16,
    pub expected_wkc: u16,
    pub cycles: u32,
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
}

impl Cyclic {
    /// Create the engine for the slave at `station`, built from the compile-time
    /// bus configuration. The bring-up FSM has already reached SAFE-OP.
    pub fn new(station: u16) -> Self {
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
        }
    }

    /// The process-data image (read; inputs are live in OP/SAFE-OP).
    pub fn image(&self) -> &[u8] {
        self.domain.image()
    }

    /// The process-data image (write; the application sets outputs here).
    pub fn image_mut(&mut self) -> &mut [u8] {
        self.domain.image_mut()
    }

    /// A snapshot of cyclic health.
    pub fn status(&self) -> CyclicStatus {
        CyclicStatus {
            phase: self.phase,
            wkc: self.domain.last_wkc(),
            expected_wkc: self.domain.expected_wkc(),
            cycles: self.total_cycles,
        }
    }

    /// Run one cycle: process the previous reply, then send this cycle's frame.
    /// Called from the high-priority PIT task; must stay short and non-blocking.
    pub fn tick(&mut self, dev: &mut Device, index: &mut u8) {
        self.total_cycles = self.total_cycles.wrapping_add(1);
        if self.outstanding {
            self.receive(dev);
            self.outstanding = false;
        }
        self.send(dev, index);
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

        // Interpret an appended AL datagram (only meaningful during OP request).
        if next != 0 && self.appended == Appended::Status {
            if let Some((al, _)) = datagram::parse_at(frame, next) {
                let status = al.data.first().copied().unwrap_or(0);
                if status & al_state::ERROR != 0 {
                    self.phase = Phase::Faulted;
                } else if status & al_state::MASK == al_state::OP {
                    self.phase = Phase::Operational;
                }
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
