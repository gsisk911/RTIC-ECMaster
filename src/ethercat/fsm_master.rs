//! Top-level master FSM (scan orchestration).
//!
//! IgH: master/fsm_master.c, master/fsm_master.h (`ec_fsm_master_t`) - the
//! master's top FSM: broadcast-read the bus to count slaves, clear station
//! addresses, measure DC delays, then run the per-slave scan/config FSMs.
//! Rust: v1 implements the scan-orchestration subset as blocking helpers
//! (`count_slaves`, `clear_addresses`, `scan_bus`). The C function-pointer state
//! machine and idle/operation phases are added with the cyclic feature.
//! Dropped (kernel-only): the `kthread` master loop + `jiffies` scheduling ->
//! a one-shot RTIC task calling `scan_bus`; `kmalloc`'d slave list ->
//! `heapless::Vec<SlaveInfo, EC_MAX_SLAVES>`.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::Device;
use crate::ethercat::ecrt::EcError;
use crate::ethercat::fsm_slave_scan;
use crate::ethercat::globals::{reg, EC_MAX_SLAVES, EC_SCAN_FRAME_BUF};
use crate::ethercat::slave::SlaveInfo;
use heapless::Vec;

/// Count responding slaves via a broadcast read of AL status (0x0130).
///
/// The working counter of a BRD equals the number of slaves that processed it.
pub fn count_slaves(dev: &mut Device, index: &mut u8) -> Result<u16, EcError> {
    let mut tx = [0u8; EC_SCAN_FRAME_BUF];
    let mut rx = [0u8; EC_SCAN_FRAME_BUF];
    let n = datagram::build(&mut tx, *index, Command::Brd, 0x0000, reg::AL_STATUS, &[0u8; 2]);
    *index = index.wrapping_add(1);
    let len = dev.transact(&tx[..n], &mut rx)?;
    let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
    Ok(reply.working_counter)
}

/// Broadcast-write zero to the station-address register (0x0010) so no slave
/// keeps a stale configured address before the scan assigns fresh ones.
pub fn clear_addresses(dev: &mut Device, index: &mut u8) -> Result<(), EcError> {
    let mut tx = [0u8; EC_SCAN_FRAME_BUF];
    let mut rx = [0u8; EC_SCAN_FRAME_BUF];
    let n = datagram::build(&mut tx, *index, Command::Bwr, 0x0000, reg::STATION_ADDR, &[0u8; 2]);
    *index = index.wrapping_add(1);
    dev.transact(&tx[..n], &mut rx)?;
    Ok(())
}

/// Scan the whole bus: count slaves, clear addresses, then scan each slave.
///
/// Returns the discovered slaves in ring order.
pub fn scan_bus(dev: &mut Device) -> Result<Vec<SlaveInfo, EC_MAX_SLAVES>, EcError> {
    let mut index = 0u8;
    let count = count_slaves(dev, &mut index)?;
    clear_addresses(dev, &mut index)?;

    let mut slaves = Vec::new();
    for ring_pos in 0..count {
        if ring_pos as usize >= EC_MAX_SLAVES {
            return Err(EcError::TooManySlaves);
        }
        let info = fsm_slave_scan::scan_slave(dev, ring_pos, &mut index)?;
        slaves.push(info).map_err(|_| EcError::TooManySlaves)?;
    }
    Ok(slaves)
}
