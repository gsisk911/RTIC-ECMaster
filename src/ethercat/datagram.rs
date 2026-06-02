//! EtherCAT datagram encoding/decoding.
//!
//! IgH: master/datagram.c, master/datagram.h (`ec_datagram_t`,
//! `ec_datagram_type_t`) - one datagram's command, address, payload and the
//! trailing working counter, plus the per-command builders
//! (`ec_datagram_aprd/apwr/fprd/fpwr/brd/bwr/...`).
//! Rust: a `#[repr(u8)] enum Command` replaces the `EC_DATAGRAM_*` `#define`s;
//! `build`/`parse` operate on byte slices with `to_le_bytes`/`from_le_bytes`
//! instead of the `EC_READ_*`/`EC_WRITE_*` pointer macros. v1 packs exactly one
//! datagram per EtherCAT frame (blocking request/response).
//! Dropped (kernel-only): the kmalloc'd datagram + `list_head` queue and the
//! `jiffies` send timestamp -> a caller-provided buffer, no queue.
//!
//! Frame wire layout produced by `build` (all little-endian except the Ethernet
//! EtherType, which the `device` layer writes big-endian):
//!
//! ```text
//! [ EtherCAT frame header (2) | datagram header (10) | payload | WKC (2) | pad ]
//!   len(0..10) | type=1            cmd idx ADP ADO len irq
//! ```

use crate::ethercat::globals::{
    EC_DATAGRAM_FOOTER_SIZE, EC_DATAGRAM_HEADER_SIZE, EC_FRAME_HEADER_SIZE, EC_MIN_ECAT_FRAME,
};

/// EtherCAT datagram command type (`ec_datagram_type_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Command {
    /// Auto-increment physical read.
    Aprd = 0x01,
    /// Auto-increment physical write.
    Apwr = 0x02,
    /// Auto-increment physical read-write.
    Aprw = 0x03,
    /// Configured-address physical read.
    Fprd = 0x04,
    /// Configured-address physical write.
    Fpwr = 0x05,
    /// Configured-address physical read-write.
    Fprw = 0x06,
    /// Broadcast read.
    Brd = 0x07,
    /// Broadcast write.
    Bwr = 0x08,
    /// Broadcast read-write.
    Brw = 0x09,
    /// Logical read.
    Lrd = 0x0A,
    /// Logical write.
    Lwr = 0x0B,
    /// Logical read-write.
    Lrw = 0x0C,
    /// Auto-increment physical read, multiple write (DC).
    Armw = 0x0D,
    /// Configured-address physical read, multiple write (DC).
    Frmw = 0x0E,
}

/// A parsed datagram reply (a view into the received EtherCAT frame).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Reply<'a> {
    pub command: u8,
    pub index: u8,
    pub working_counter: u16,
    pub data: &'a [u8],
}

/// Auto-increment address position (ADP) for a given ring position.
///
/// Each slave increments ADP by one as the frame passes; a slave executes the
/// command only when it sees ADP == 0, so we send `-ring_position`.
#[inline]
pub fn autoinc_adp(ring_pos: u16) -> u16 {
    0u16.wrapping_sub(ring_pos)
}

/// Build a single-datagram EtherCAT frame into `buf`.
///
/// `adp`/`ado` are the address-position / address-offset words (auto-increment,
/// configured station, or broadcast depending on `cmd`). Returns the total
/// EtherCAT frame length (padded to the 60-byte Ethernet minimum).
pub fn build(buf: &mut [u8], index: u8, cmd: Command, adp: u16, ado: u16, payload: &[u8]) -> usize {
    let plen = payload.len();
    // Real EtherCAT content following the frame header.
    let datagram_len = EC_DATAGRAM_HEADER_SIZE + plen + EC_DATAGRAM_FOOTER_SIZE;

    // EtherCAT frame header: length (bits 0..10) | type 1 (-> 0x1000).
    let frame_hdr = ((datagram_len as u16) & 0x07FF) | 0x1000;
    buf[0..2].copy_from_slice(&frame_hdr.to_le_bytes());

    // Datagram header (10 bytes).
    buf[2] = cmd as u8;
    buf[3] = index;
    buf[4..6].copy_from_slice(&adp.to_le_bytes());
    buf[6..8].copy_from_slice(&ado.to_le_bytes());
    // Length word: payload length (bits 0..10); bit 15 ("more follows") = 0.
    buf[8..10].copy_from_slice(&((plen as u16) & 0x07FF).to_le_bytes());
    // IRQ word (master writes 0).
    buf[10..12].copy_from_slice(&0u16.to_le_bytes());

    // Payload.
    buf[12..12 + plen].copy_from_slice(payload);

    // Working-counter footer (master writes 0; slaves increment).
    buf[12 + plen..14 + plen].copy_from_slice(&0u16.to_le_bytes());

    // Pad the EtherCAT region to the Ethernet 60-byte minimum.
    let mut total = EC_FRAME_HEADER_SIZE + datagram_len;
    while total < EC_MIN_ECAT_FRAME {
        buf[total] = 0;
        total += 1;
    }
    total
}

/// Append another datagram to a frame already produced by [`build`]. Sets the
/// previously-last datagram's "more datagrams follow" bit (length-word `0x8000`)
/// and writes the new datagram + working-counter footer after it. Returns the
/// new total EtherCAT frame length (re-padded to the 60-byte minimum).
///
/// Used to pack a background datagram (DC monitor / mailbox poll) into the same
/// frame as the cyclic LRW, mirroring IgH's external datagram ring. (v1's cyclic
/// path sends a single LRW; the receive side must offset-walk datagrams before
/// this is used on the hot path.)
pub fn append(buf: &mut [u8], index: u8, cmd: Command, adp: u16, ado: u16, payload: &[u8]) -> usize {
    let region = (u16::from_le_bytes([buf[0], buf[1]]) & 0x07FF) as usize;
    let region_end = EC_FRAME_HEADER_SIZE + region;

    // Walk to the last datagram and set its "more follows" bit.
    let mut off = EC_FRAME_HEADER_SIZE;
    loop {
        let len_off = off + 6;
        let plen = (u16::from_le_bytes([buf[len_off], buf[len_off + 1]]) & 0x07FF) as usize;
        let next = off + EC_DATAGRAM_HEADER_SIZE + plen + EC_DATAGRAM_FOOTER_SIZE;
        if next >= region_end {
            let raw = u16::from_le_bytes([buf[len_off], buf[len_off + 1]]) | 0x8000;
            buf[len_off..len_off + 2].copy_from_slice(&raw.to_le_bytes());
            off = next;
            break;
        }
        off = next;
    }

    // Write the new datagram header + payload + zero working counter at `off`.
    let plen = payload.len();
    buf[off] = cmd as u8;
    buf[off + 1] = index;
    buf[off + 2..off + 4].copy_from_slice(&adp.to_le_bytes());
    buf[off + 4..off + 6].copy_from_slice(&ado.to_le_bytes());
    buf[off + 6..off + 8].copy_from_slice(&((plen as u16) & 0x07FF).to_le_bytes());
    buf[off + 8..off + 10].copy_from_slice(&0u16.to_le_bytes());
    buf[off + 10..off + 10 + plen].copy_from_slice(payload);
    buf[off + 10 + plen..off + 12 + plen].copy_from_slice(&0u16.to_le_bytes());

    // Update the frame-header datagram-region length and re-pad.
    let new_region = region + EC_DATAGRAM_HEADER_SIZE + plen + EC_DATAGRAM_FOOTER_SIZE;
    let frame_hdr = ((new_region as u16) & 0x07FF) | 0x1000;
    buf[0..2].copy_from_slice(&frame_hdr.to_le_bytes());

    let mut total = EC_FRAME_HEADER_SIZE + new_region;
    while total < EC_MIN_ECAT_FRAME {
        buf[total] = 0;
        total += 1;
    }
    total
}

/// Parse the single datagram from a received EtherCAT frame.
///
/// `frame` is the EtherCAT frame (without the Ethernet header). Returns `None`
/// if the buffer is too short for the declared data length + working counter.
pub fn parse(frame: &[u8]) -> Option<Reply<'_>> {
    if frame.len() < EC_FRAME_HEADER_SIZE + EC_DATAGRAM_HEADER_SIZE + EC_DATAGRAM_FOOTER_SIZE {
        return None;
    }
    let command = frame[2];
    let index = frame[3];
    let data_len = (u16::from_le_bytes([frame[8], frame[9]]) & 0x07FF) as usize;
    let data_start = EC_FRAME_HEADER_SIZE + EC_DATAGRAM_HEADER_SIZE; // 12
    let wkc_start = data_start + data_len;
    if frame.len() < wkc_start + EC_DATAGRAM_FOOTER_SIZE {
        return None;
    }
    let data = &frame[data_start..wkc_start];
    let working_counter = u16::from_le_bytes([frame[wkc_start], frame[wkc_start + 1]]);
    Some(Reply {
        command,
        index,
        working_counter,
        data,
    })
}

/// Parse the datagram at byte offset `off` within an EtherCAT `frame` (the first
/// datagram is at `EC_FRAME_HEADER_SIZE`). Returns the reply and the offset of
/// the next datagram, or `0` when the "more datagrams follow" bit is clear.
/// Used to walk a multi-datagram cyclic frame (LRW + an appended datagram).
pub fn parse_at(frame: &[u8], off: usize) -> Option<(Reply<'_>, usize)> {
    if frame.len() < off + EC_DATAGRAM_HEADER_SIZE + EC_DATAGRAM_FOOTER_SIZE {
        return None;
    }
    let command = frame[off];
    let index = frame[off + 1];
    let len_word = u16::from_le_bytes([frame[off + 6], frame[off + 7]]);
    let data_len = (len_word & 0x07FF) as usize;
    let more = len_word & 0x8000 != 0;
    let data_start = off + EC_DATAGRAM_HEADER_SIZE;
    let wkc_start = data_start + data_len;
    if frame.len() < wkc_start + EC_DATAGRAM_FOOTER_SIZE {
        return None;
    }
    let data = &frame[data_start..wkc_start];
    let working_counter = u16::from_le_bytes([frame[wkc_start], frame[wkc_start + 1]]);
    let next = if more {
        wkc_start + EC_DATAGRAM_FOOTER_SIZE
    } else {
        0
    };
    Some((
        Reply {
            command,
            index,
            working_counter,
            data,
        },
        next,
    ))
}
