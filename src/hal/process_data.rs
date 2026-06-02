//! HAL <-> process-data binding.
//!
//! Resolves named `PinCfg`s (from `config::generated::BUS.pins`) to typed reads
//! and writes over the `domain` process image. The application and a future
//! `cia402` layer read inputs / write outputs by name instead of touching raw
//! byte offsets.
//!
//! v1 assumes byte-aligned multi-bit fields (true for the test drive); single
//! bits honour `bit_pos`. Non-byte-aligned multi-bit fields are not yet handled.

use crate::ethercat::config::generated::BUS;
use crate::hal::pin::{HalType, PinCfg};

/// Look up a named process-data pin.
pub fn find(name: &str) -> Option<&'static PinCfg> {
    BUS.pin(name)
}

/// All process-data pins (for listing).
pub fn all() -> &'static [PinCfg] {
    BUS.pins
}

/// Read a pin's current value from the process image (signed-aware).
pub fn read_value(image: &[u8], pin: &PinCfg) -> i64 {
    let off = pin.byte_offset as usize;
    match pin.hal_type {
        HalType::Bit => {
            if off < image.len() {
                ((image[off] >> pin.bit_pos) & 1) as i64
            } else {
                0
            }
        }
        HalType::U32 | HalType::S32 => {
            let nbytes = (pin.bit_len as usize).div_ceil(8);
            let mut raw: u64 = 0;
            for i in 0..nbytes {
                if off + i < image.len() {
                    raw |= (image[off + i] as u64) << (8 * i);
                }
            }
            if pin.hal_type == HalType::S32 {
                sign_extend(raw, pin.bit_len)
            } else {
                raw as i64
            }
        }
    }
}

/// Write a value into a pin's location in the process image (outputs).
pub fn write_value(image: &mut [u8], pin: &PinCfg, value: i64) {
    let off = pin.byte_offset as usize;
    match pin.hal_type {
        HalType::Bit => {
            if off < image.len() {
                let mask = 1u8 << pin.bit_pos;
                if value != 0 {
                    image[off] |= mask;
                } else {
                    image[off] &= !mask;
                }
            }
        }
        HalType::U32 | HalType::S32 => {
            let nbytes = (pin.bit_len as usize).div_ceil(8);
            let raw = value as u64;
            for i in 0..nbytes {
                if off + i < image.len() {
                    image[off + i] = (raw >> (8 * i)) as u8;
                }
            }
        }
    }
}

/// Sign-extend the low `bits` of `raw` to a full `i64`.
fn sign_extend(raw: u64, bits: u8) -> i64 {
    if bits == 0 || bits >= 64 {
        return raw as i64;
    }
    let shift = 64 - bits as u32;
    ((raw << shift) as i64) >> shift
}
