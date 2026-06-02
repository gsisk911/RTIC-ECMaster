//! Per-slave scan FSM.
//!
//! IgH: master/fsm_slave_scan.c, master/fsm_slave_scan.h
//! (`ec_fsm_slave_scan_t`) - drives one slave through the scan sequence:
//! assign station address, read AL status and base/DL info, then read identity
//! and categories from the SII (and optionally PDO config over CoE).
//! Rust: v1 implements the identity subset as a blocking `scan_slave` (the C
//! state progression START->ADDRESS->STATE->BASE->SII collapses to straight-line
//! calls); `Result<_, EcError>` instead of int codes. The full DC/SM/PDO/category
//! parse is added with the configuration feature.
//! Dropped (kernel-only): the function-pointer `state` member + scheduling waits
//! -> direct calls on the cyclic task.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::Device;
use crate::ethercat::ecrt::EcError;
use crate::ethercat::fsm_sii;
use crate::ethercat::globals::{al_state, mbox, reg, sii, EC_SCAN_FRAME_BUF};
use crate::ethercat::slave::SlaveInfo;

/// Scan one slave at `ring_pos`, returning its identity/base information.
///
/// Assigns the configured station address `ring_pos + 1` by auto-increment,
/// then reads AL status, DL/base info, and SII vendor/product/revision. `index`
/// is the running datagram index, advanced once per datagram sent.
pub fn scan_slave(dev: &mut Device, ring_pos: u16, index: &mut u8) -> Result<SlaveInfo, EcError> {
    let station = ring_pos + 1;
    let mut tx = [0u8; EC_SCAN_FRAME_BUF];
    let mut rx = [0u8; EC_SCAN_FRAME_BUF];

    // 1. Assign the configured station address by auto-increment position.
    let adp = datagram::autoinc_adp(ring_pos);
    let n = datagram::build(
        &mut tx,
        *index,
        Command::Apwr,
        adp,
        reg::STATION_ADDR,
        &station.to_le_bytes(),
    );
    *index = index.wrapping_add(1);
    let len = dev.transact(&tx[..n], &mut rx)?;
    let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
    if reply.working_counter != 1 {
        return Err(EcError::WorkingCounter);
    }

    // 2. Read AL status using the configured address.
    let n = datagram::build(&mut tx, *index, Command::Fprd, station, reg::AL_STATUS, &[0u8; 2]);
    *index = index.wrapping_add(1);
    let len = dev.transact(&tx[..n], &mut rx)?;
    let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
    // IgH checks WKC==1 here; otherwise the slave dropped the read and the
    // echoed zero payload would masquerade as a valid (state 0) response.
    if reply.working_counter != 1 {
        return Err(EcError::WorkingCounter);
    }
    let al = reply.data.first().copied().unwrap_or(0) & al_state::MASK;

    // 3. Read DL/base information (12 bytes at 0x0000).
    let n = datagram::build(&mut tx, *index, Command::Fprd, station, reg::DL_INFO, &[0u8; 12]);
    *index = index.wrapping_add(1);
    let len = dev.transact(&tx[..n], &mut rx)?;
    let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
    if reply.working_counter != 1 {
        return Err(EcError::WorkingCounter);
    }
    let base_type = reply.data.first().copied().unwrap_or(0);
    let base_fmmu_count = reply.data.get(4).copied().unwrap_or(0);
    let base_sync_count = reply.data.get(5).copied().unwrap_or(0);

    // 4. Read identity from the SII (vendor / product / revision).
    let vendor_id = fsm_sii::sii_read_u32(dev, station, sii::VENDOR_ID, index)?;
    let product_code = fsm_sii::sii_read_u32(dev, station, sii::PRODUCT_CODE, index)?;
    let revision = fsm_sii::sii_read_u32(dev, station, sii::REVISION, index)?;

    // 5. Read the standard mailbox configuration from the SII. Each 32-bit read
    // returns two adjacent words: low = offset/protocols, high = size.
    let rx_mbox = fsm_sii::sii_read_u32(dev, station, sii::STD_RX_MBOX, index)?;
    let tx_mbox = fsm_sii::sii_read_u32(dev, station, sii::STD_TX_MBOX, index)?;
    let mbox_protocols = fsm_sii::sii_read_u32(dev, station, sii::MBOX_PROTOCOLS, index)? as u16;

    Ok(SlaveInfo {
        ring_pos,
        station_addr: station,
        al_state: al,
        base_type,
        base_fmmu_count,
        base_sync_count,
        vendor_id,
        product_code,
        revision,
        rx_mbox_offset: rx_mbox as u16,
        rx_mbox_size: (rx_mbox >> 16) as u16,
        tx_mbox_offset: tx_mbox as u16,
        tx_mbox_size: (tx_mbox >> 16) as u16,
        mbox_protocols,
        supports_coe: mbox_protocols & mbox::PROTO_COE != 0,
    })
}
