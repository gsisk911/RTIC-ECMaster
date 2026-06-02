//! One FMMU configuration entry.
//!
//! IgH: master/fmmu_config.c, master/fmmu_config.h (`ec_fmmu_config_t`) - a
//! single FMMU mapping a slave's sync-manager region between logical (domain)
//! and physical addresses, plus the helper rendering its 16-byte ESC page.
//! Rust: the desired mapping lives in `config::model::FmmuCfg`; this module
//! renders that into the 16-byte page written to FMMU register `0x0600 + n*16`
//! via the `ecrt` LE helpers.
//! Dropped (kernel-only): none of note.

use crate::ethercat::config::model::FmmuCfg;
use crate::ethercat::ecrt::{write_u16_le, write_u32_le, EcDirection};
use crate::ethercat::globals::fmmu;

/// Size of one FMMU configuration page, in bytes.
pub const PAGE_SIZE: usize = fmmu::PAGE_SIZE;

/// Render the 16-byte FMMU page for `cfg` into `page`.
///
/// Layout (IgH `ec_fmmu_config_page`): logical start (U32), data size (U16),
/// logical start bit `0x00`, logical end bit `0x07` (whole bytes), physical
/// start (U16), physical start bit `0x00`, direction, enable `0x0001`,
/// reserved `0x0000`.
pub fn write_page(page: &mut [u8], cfg: &FmmuCfg) {
    write_u32_le(&mut page[0..4], cfg.logical_start);
    write_u16_le(&mut page[4..6], cfg.size);
    page[6] = 0x00; // logical start bit
    page[7] = 0x07; // logical end bit (byte-aligned regions)
    write_u16_le(&mut page[8..10], cfg.phys_start);
    page[10] = 0x00; // physical start bit
    page[11] = match cfg.dir {
        EcDirection::Input => fmmu::DIR_INPUT,
        EcDirection::Output => fmmu::DIR_OUTPUT,
    };
    write_u16_le(&mut page[12..14], 0x0001); // enable
    write_u16_le(&mut page[14..16], 0x0000); // reserved
}
