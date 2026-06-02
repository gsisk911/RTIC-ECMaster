//! SII / EEPROM read FSM.
//!
//! IgH: master/fsm_sii.c, master/fsm_sii.h (`ec_fsm_sii_t`) - reads (and writes)
//! the slave's SII EEPROM through the ESC SII interface registers: issue a read
//! by writing the control/status + address (0x0502/0x0504), then poll the
//! status byte and take the data word(s) from 0x0508.
//! Rust: v1 implements the read path as a blocking `sii_read_u32` (the C FSM's
//! function-pointer states collapse to a straight-line request + poll loop);
//! `Result<_, EcError>` replaces int codes; bytes via `from_le_bytes`.
//! Dropped (kernel-only): `jiffies`-based `SII_TIMEOUT` -> a bounded poll loop
//! with `cortex_m::asm::delay`.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::Device;
use crate::ethercat::ecrt::EcError;
use crate::ethercat::globals::{reg, sii_ctrl, EC_SCAN_FRAME_BUF};

/// Maximum status-poll iterations before declaring the EEPROM stuck busy.
const SII_POLL_ATTEMPTS: u32 = 200;
/// Busy-wait between status polls (~10 us at the M7 core clock).
const SII_POLL_DELAY_CYCLES: u32 = 6_000;

/// Read 32 bits (two SII words) at `word_offset` from a slave's EEPROM.
///
/// Uses the configured station address (FPWR/FPRD on register 0x0502). `index`
/// is the running datagram index, advanced once per datagram sent.
pub fn sii_read_u32(
    dev: &mut Device,
    station: u16,
    word_offset: u16,
    index: &mut u8,
) -> Result<u32, EcError> {
    let mut tx = [0u8; EC_SCAN_FRAME_BUF];
    let mut rx = [0u8; EC_SCAN_FRAME_BUF];

    // 1. Issue the read: write control/status (0x0502) + word address (0x0504).
    let cmd = [
        sii_ctrl::ADDR_MODE_TWO_OCTET, // 0x0502: two-address-octet mode
        sii_ctrl::CMD_READ,            // 0x0503: read command
        (word_offset & 0xFF) as u8,    // 0x0504: address low byte
        (word_offset >> 8) as u8,      // 0x0505: address high byte
    ];
    let n = datagram::build(&mut tx, *index, Command::Fpwr, station, reg::SII_CONTROL, &cmd);
    *index = index.wrapping_add(1);
    let len = dev.transact(&tx[..n], &mut rx)?;
    let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;
    if reply.working_counter == 0 {
        return Err(EcError::WorkingCounter);
    }

    // 2. Poll control/status (read 10 bytes from 0x0502) until the read finishes.
    for _ in 0..SII_POLL_ATTEMPTS {
        let n = datagram::build(&mut tx, *index, Command::Fprd, station, reg::SII_CONTROL, &[0u8; 10]);
        *index = index.wrapping_add(1);
        let len = dev.transact(&tx[..n], &mut rx)?;
        let reply = datagram::parse(&rx[..len]).ok_or(EcError::FrameTooShort)?;

        if reply.working_counter == 0 || reply.data.len() < 10 {
            cortex_m::asm::delay(SII_POLL_DELAY_CYCLES);
            continue;
        }

        let status = reply.data[1]; // register 0x0503
        if status & sii_ctrl::STATUS_ERROR != 0 {
            return Err(EcError::SiiError);
        }
        if status & sii_ctrl::STATUS_BUSY == 0 {
            // Read complete: data word(s) live at register 0x0508 = offset +6.
            return Ok(u32::from_le_bytes([
                reply.data[6],
                reply.data[7],
                reply.data[8],
                reply.data[9],
            ]));
        }
        cortex_m::asm::delay(SII_POLL_DELAY_CYCLES);
    }

    Err(EcError::SiiTimeout)
}
