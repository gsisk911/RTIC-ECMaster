//! PDO assignment over CoE.
//!
//! IgH: master/fsm_pdo.c, master/fsm_pdo.h (`ec_fsm_pdo_t`) - reads/writes the
//! sync-manager PDO assignment object (`0x1C10 + sync_index`) to assign which
//! PDOs a sync manager carries.
//! Rust: a builder that turns the desired PDO list into a [`CoeSeq`] of
//! expedited SDO downloads (clear count -> write each PDO index -> set count),
//! stepped one datagram at a time by the caller (`fsm_slave_config`).

use crate::ethercat::config::model::PdoCfg;
use crate::ethercat::fsm_coe::CoeSeq;
use crate::ethercat::globals::coe;
use crate::ethercat::slave::Mailbox;

/// Build the assignment sequence for sync manager `sync_index` (2 = SM2/RxPDO,
/// 3 = SM3/TxPDO): `0x1C1x:00 = 0`, `0x1C1x:n = pdo_index`, `0x1C1x:00 = count`.
pub fn build_assign(mbox: Mailbox, sync_index: u8, pdos: &[PdoCfg]) -> CoeSeq {
    let obj = coe::pdo_assign(sync_index);
    let mut seq = CoeSeq::new(mbox);
    let _ = seq.push(obj, 0x00, &[0]); // clear assigned count
    for (i, pdo) in pdos.iter().enumerate() {
        let _ = seq.push(obj, (i + 1) as u8, &pdo.index.to_le_bytes());
    }
    let _ = seq.push(obj, 0x00, &[pdos.len() as u8]); // commit count
    seq
}
