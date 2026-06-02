//! Modbus register interface (diagnostics and configuration).
//!
//! The Modbus holding-register slave and its generated register map. Used for
//! device configuration and health/status reporting; `register_map.rs` is
//! generated from `registers.json` by `scripts/generate_registers.py`.

pub mod modbus_slave;
pub mod register_map;
