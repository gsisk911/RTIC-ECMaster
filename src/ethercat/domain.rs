//! Process-data domain: the cyclic image and its offset map.
//!
//! IgH: master/domain.c, master/domain.h (`ec_domain_t`) - aggregates the
//! registered PDO entries, owns the contiguous process-data image, and builds
//! the cyclic LRW/LRD/LWR datagrams. `ecrt_domain_process/queue` run the
//! exchange.
//! Rust: a fixed-capacity image buffer plus the resolved input byte ranges and
//! the expected working counter, all taken from the compile-time
//! `config::generated::BUS`. The image is a `[u8; MAX_IMAGE]` (no kmalloc) and
//! `ecrt_domain_data` becomes a borrowed slice.
//!
//! v1 emits one LRW per cycle covering the whole image at logical address 0.

use crate::ethercat::config::model::BusCfg;
use crate::ethercat::datagram::{self, Command, Reply};
use crate::ethercat::ecrt::EcDirection;
use crate::ethercat::globals::EC_MAX_SLAVES;
use heapless::Vec;

/// Maximum process-image size, in bytes (fixed, no heap). Sized for the planned
/// multi-slave bus (~306 B) with margin; v1's single drive uses 55 B.
pub const MAX_IMAGE: usize = 512;

/// Per-cycle working-counter health, mirroring `ec_wc_state_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WkcState {
    /// No slave responded (WKC == 0).
    Zero,
    /// Some but not all expected slaves responded.
    Incomplete,
    /// All expected slaves exchanged data (WKC == expected).
    Complete,
}

/// The process-data domain: one contiguous image exchanged every cycle.
pub struct EcDomain {
    image: [u8; MAX_IMAGE],
    len: usize,
    expected_wkc: u16,
    last_wkc: u16,
    /// Byte ranges (start, len) the slaves write (TxPDO / inputs); only these
    /// are copied back from a reply so application-written outputs are not lost.
    input_ranges: Vec<(usize, usize), EC_MAX_SLAVES>,
}

impl EcDomain {
    /// Build the domain from the compile-time bus configuration.
    pub fn from_config(bus: &BusCfg) -> Self {
        let len = bus.image_size.min(MAX_IMAGE);
        let mut expected_wkc = 0u16;
        let mut input_ranges = Vec::new();
        for slave in bus.slaves {
            for fmmu in slave.fmmus {
                match fmmu.dir {
                    EcDirection::Output => expected_wkc += 2,
                    EcDirection::Input => {
                        expected_wkc += 1;
                        let _ = input_ranges
                            .push((fmmu.logical_start as usize, fmmu.size as usize));
                    }
                }
            }
        }
        Self {
            image: [0; MAX_IMAGE],
            len,
            expected_wkc,
            last_wkc: 0,
            input_ranges,
        }
    }

    /// The process-data image (read).
    pub fn image(&self) -> &[u8] {
        &self.image[..self.len]
    }

    /// The process-data image (write; the application sets outputs here).
    pub fn image_mut(&mut self) -> &mut [u8] {
        &mut self.image[..self.len]
    }

    /// Expected per-cycle working counter (outputs*2 + inputs).
    pub fn expected_wkc(&self) -> u16 {
        self.expected_wkc
    }

    /// Working counter observed on the most recent reply.
    pub fn last_wkc(&self) -> u16 {
        self.last_wkc
    }

    /// Build the cyclic LRW datagram (whole image, logical address 0) into
    /// `buf`. Returns the EtherCAT frame length.
    pub fn build_lrw(&self, buf: &mut [u8], index: u8) -> usize {
        // Logical address 0: ADP = low 16 bits, ADO = high 16 bits.
        datagram::build(buf, index, Command::Lrw, 0x0000, 0x0000, &self.image[..self.len])
    }

    /// Apply a received LRW reply: copy the input ranges back into the image and
    /// record the working counter.
    pub fn apply_reply(&mut self, reply: &Reply) -> WkcState {
        for &(start, size) in &self.input_ranges {
            let end = (start + size).min(self.len);
            if start < end && end <= reply.data.len() {
                self.image[start..end].copy_from_slice(&reply.data[start..end]);
            }
        }
        self.last_wkc = reply.working_counter;
        if reply.working_counter == 0 {
            WkcState::Zero
        } else if reply.working_counter == self.expected_wkc {
            WkcState::Complete
        } else {
            WkcState::Incomplete
        }
    }
}
