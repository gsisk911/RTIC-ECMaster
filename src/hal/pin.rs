//! HAL pin types.
//!
//! `HalType` (bit/u32/s32) and the resolved `PinCfg` (name + type + direction +
//! image location) come from the compile-time `config::model`, so the HAL layer
//! reuses them rather than redefining a parallel set. Binding to process data
//! lives in `process_data`.

pub use crate::ethercat::config::model::{HalType, PinCfg};
pub use crate::ethercat::ecrt::EcDirection;

/// Whether a pin is a master output (master -> drive / RxPDO).
pub fn is_output(pin: &PinCfg) -> bool {
    matches!(pin.dir, EcDirection::Output)
}
