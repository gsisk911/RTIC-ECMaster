//! HAL: named-pin abstraction over the EtherCAT process data.
//!
//! Exposes the `halPin` names from `ethercat-conf.xml` (e.g.
//! `die-cylinder-statusword`, `shuttle-target-position`) as typed, directional
//! pins so the rest of the application reads/writes process data by name
//! instead of touching raw `pdi` offsets.
//!
//! - `pin`          - HalPin + HalType (bit/u32/s32), direction, value storage
//! - `process_data` - binds named pins to `ethercat::pdi` offsets (read/write)

pub mod host_bridge;
pub mod motion_buffer;
pub mod pin;
pub mod process_data;
pub mod spi_layout;
pub mod spi_layout_generated;
