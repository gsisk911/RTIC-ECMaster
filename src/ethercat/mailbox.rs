//! Low-level EtherCAT mailbox framing.
//!
//! IgH: master/mailbox.c, master/mailbox.h - the 6-byte mailbox header (length,
//! address, channel+priority, type+counter) and protocol multiplexing used by
//! CoE/FoE/SoE/EoE/VoE.
//! Rust: free functions that build/parse the header into byte slices via the
//! `ecrt` LE helpers; `Option`/`Result` instead of int returns.
//! Dropped (kernel-only): none of note (pure framing).

use crate::ethercat::ecrt::{read_u16_le, write_u16_le};
use crate::ethercat::globals::mbox;

/// Write the 6-byte mailbox header. `data_len` is the length of the service
/// data that follows the header (e.g. CoE header + SDO body).
pub fn write_header(buf: &mut [u8], data_len: u16, mbox_type: u8, counter: u8) {
    write_u16_le(&mut buf[0..2], data_len);
    write_u16_le(&mut buf[2..4], 0); // address: master writes 0
    buf[4] = 0; // channel (bits 0..5) + priority (bits 6..7)
    buf[5] = (mbox_type & 0x0F) | ((counter & 0x07) << 4);
}

/// A parsed mailbox header.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Header {
    pub data_len: u16,
    pub mbox_type: u8,
    pub counter: u8,
}

/// Parse a 6-byte mailbox header from the start of `buf`.
pub fn parse_header(buf: &[u8]) -> Option<Header> {
    if buf.len() < mbox::HEADER_SIZE {
        return None;
    }
    let flags = buf[5];
    Some(Header {
        data_len: read_u16_le(&buf[0..2]),
        mbox_type: flags & 0x0F,
        counter: (flags >> 4) & 0x07,
    })
}
