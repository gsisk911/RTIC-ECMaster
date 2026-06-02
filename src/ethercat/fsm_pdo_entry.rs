//! PDO mapping over CoE.
//!
//! IgH: master/fsm_pdo_entry.c, master/fsm_pdo_entry.h (`ec_fsm_pdo_entry_t`) -
//! reads/writes a PDO's mapping object (`0x1600..`/`0x1A00..`) to set which
//! object-dictionary entries the PDO carries.
//! Rust: a builder that turns a PDO's entry list into a [`CoeSeq`] of expedited
//! SDO downloads (clear count -> write each packed entry -> set count), stepped
//! one datagram at a time by the caller (`fsm_slave_config`).

use crate::ethercat::config::model::PdoCfg;
use crate::ethercat::fsm_coe::CoeSeq;
use crate::ethercat::slave::Mailbox;

/// Pack one mapping entry into the CoE `u32`: `(index << 16) | (sub << 8) | bits`.
#[inline]
fn pack(index: u16, subindex: u8, bit_length: u8) -> u32 {
    ((index as u32) << 16) | ((subindex as u32) << 8) | (bit_length as u32)
}

/// Build the mapping sequence for one PDO: `0x16xx:00 = 0`,
/// `0x16xx:n = packed_entry`, `0x16xx:00 = count`.
pub fn build_mapping(mbox: Mailbox, pdo: &PdoCfg) -> CoeSeq {
    let obj = pdo.index;
    let mut seq = CoeSeq::new(mbox);
    let _ = seq.push(obj, 0x00, &[0]); // clear entry count
    for (i, entry) in pdo.entries.iter().enumerate() {
        let value = pack(entry.index, entry.subindex, entry.bit_length);
        let _ = seq.push(obj, (i + 1) as u8, &value.to_le_bytes());
    }
    let _ = seq.push(obj, 0x00, &[pdo.entries.len() as u8]); // commit count
    seq
}
