//! Sync manager configuration page encoding.
//!
//! IgH: master/sync.c, master/sync.h (`ec_sync_t`) - the helper that renders a
//! sync manager's 8-byte ESC configuration page written at `0x0800 + n*8`.
//! Rust: a free function building the page into a byte slice via `ecrt` LE
//! helpers instead of `EC_WRITE_*` pointer macros.
//! Dropped (kernel-only): none of note (pure encoding).
//!
//! Page layout (8 bytes): start(2), length(2), control(1), status(1 ro),
//! enable(1), pdi-control(1).

use crate::ethercat::ecrt::{write_u16_le, write_u8};
use crate::ethercat::globals::{sm, EC_SYNC_PAGE_SIZE};

/// Size of one sync-manager configuration page, in bytes.
pub const PAGE_SIZE: usize = EC_SYNC_PAGE_SIZE as usize;

/// Write an 8-byte sync-manager configuration page into `page`.
pub fn write_page(page: &mut [u8], phys_start: u16, length: u16, control: u8, enable: u8) {
    write_u16_le(&mut page[0..2], phys_start);
    write_u16_le(&mut page[2..4], length);
    write_u8(&mut page[4..5], control);
    write_u8(&mut page[5..6], 0); // status register is read-only; write 0
    write_u8(&mut page[6..7], enable);
    write_u8(&mut page[7..8], 0); // PDI control
}

/// Write the RxMailbox (SM0, master->slave) configuration page.
pub fn write_mailbox_out(page: &mut [u8], phys_start: u16, size: u16) {
    write_page(page, phys_start, size, sm::CONTROL_MBOX_WRITE, sm::ENABLE);
}

/// Write the TxMailbox (SM1, slave->master) configuration page.
pub fn write_mailbox_in(page: &mut [u8], phys_start: u16, size: u16) {
    write_page(page, phys_start, size, sm::CONTROL_MBOX_READ, sm::ENABLE);
}

/// Write a process-data output SM page (SM2 / RxPDO, master->slave). `control`
/// comes from the slave's SII SM category (typically `sm::CONTROL_PD_OUT`).
pub fn write_process_out(page: &mut [u8], phys_start: u16, size: u16, control: u8) {
    let enable = if size > 0 { sm::ENABLE } else { 0 };
    write_page(page, phys_start, size, control, enable);
}

/// Write a process-data input SM page (SM3 / TxPDO, slave->master). `control`
/// comes from the slave's SII SM category (typically `sm::CONTROL_PD_IN`).
pub fn write_process_in(page: &mut [u8], phys_start: u16, size: u16, control: u8) {
    let enable = if size > 0 { sm::ENABLE } else { 0 };
    write_page(page, phys_start, size, control, enable);
}
