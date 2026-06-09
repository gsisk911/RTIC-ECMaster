//! Streamed-motion layout for the host SPI bridge (stable re-export).
//!
//! The concrete constants live in `spi_layout_generated.rs`, produced by
//! `scripts/generate_ethercat_config.py` from the bus XML's `class="motion"`
//! entries and `<motionStream>` blocks (the same generator pass that writes
//! `config/generated.rs`). This module is the stable import path the firmware
//! motion buffer and host bridge use; regenerating only touches the generated
//! file.

pub use crate::hal::spi_layout_generated::{
    MAX_SAMPLES_PER_FRAME, STREAM_FIELDS, STREAM_SAMPLE_BYTES,
};
