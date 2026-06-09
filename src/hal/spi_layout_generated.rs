//! GENERATED host-SPI streamed-motion layout -- do not edit by hand.
//!
//! Produced by `scripts/generate_ethercat_config.py` (run `make config`) from the
//! bus XML's `class="motion"` entries + `<motionStream>` blocks. Shared by the
//! firmware motion buffer and the Pi HAL component. Regenerate and commit.


use crate::ethercat::config::model::StreamField;

/// Bytes in one streamed motion sample (sum of the per-axis fields).
pub const STREAM_SAMPLE_BYTES: usize = 0;

/// Maximum streamed samples per SPI frame (batch-refill cap).
pub const MAX_SAMPLES_PER_FRAME: usize = 0;

/// The streamed fields: where each lands in the cyclic image.
pub const STREAM_FIELDS: &[StreamField] = &[];
