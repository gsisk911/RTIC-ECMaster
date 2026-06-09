//! Host-bridge frame codec + shared staging state.
//!
//! Defines the fixed full-duplex SPI frame exchanged with the Raspberry Pi /
//! LinuxCNC host and the shared `HostBridge` state that decouples the prio-2
//! LPSPI task from the prio-3 cyclic task. The SPI task only moves bytes in/out
//! of this struct (it never locks the EtherCAT master); the cyclic task applies
//! the staged outputs to the process image and snapshots inputs/status back.
//!
//! v1 is immediate-only: the whole output region is sent each frame and the
//! whole input region is returned each frame. The motion look-ahead stream
//! (`motion_buffer`) layers on top without changing the header layout.
//!
//! Frame layout (little-endian; CRC-16/CCITT over all preceding bytes):
//!
//! ```text
//! MOSI (host -> Teensy):
//!   [0..2]  magic            0xA7EC
//!   [2]     version
//!   [3]     flags            bit0 enable, bit1 fault-reset, bit2 quick-stop
//!   [4..6]  host_seq
//!   [6..8]  host_wdog        host heartbeat (must advance every frame)
//!   [8..8+OUT] immediate outputs (whole output image region)
//!   [.. ]   stream_count (u8) + samples   (motion_buffer; 0 in immediate-only)
//!   [..2]   crc16
//!
//! MISO (Teensy -> host):
//!   [0..2]  magic            0xA7EC
//!   [2]     version
//!   [3]     status_flags     bit0 link, bit1 op, bit2 fault, bit3 host-timeout
//!   [4..6]  seq_echo
//!   [6..8]  teensy_wdog
//!   [8..12] cycle_index
//!   [12]    wkc
//!   [13]    expected_wkc
//!   [14]    phase            0 prime,1 req-op,2 op,3 fault
//!   [15]    fault_flags
//!   [16]    buf_depth0       motion buffer fill (axis 0; per-axis later)
//!   [17]    reserved
//!   [18..18+IN] immediate inputs (whole input image region)
//!   [..2]   crc16
//! ```

use crate::ethercat::config::generated::BUS;
use crate::hal::motion_buffer::MotionRing;
use crate::hal::spi_layout::{MAX_SAMPLES_PER_FRAME, STREAM_FIELDS, STREAM_SAMPLE_BYTES};

/// Frame magic (`0xA7EC`, little-endian on the wire).
pub const MAGIC: u16 = 0xA7EC;
/// Protocol version; bump on any layout change.
pub const VERSION: u8 = 1;

/// Host intent flag bits (MOSI `flags`).
pub mod flag {
    pub const ENABLE: u8 = 1 << 0;
    pub const FAULT_RESET: u8 = 1 << 1;
    pub const QUICK_STOP: u8 = 1 << 2;
}

/// Reply status-flag bits (MISO `status_flags`).
pub mod status {
    pub const LINK: u8 = 1 << 0;
    pub const OPERATIONAL: u8 = 1 << 1;
    pub const FAULT: u8 = 1 << 2;
    pub const HOST_TIMEOUT: u8 = 1 << 3;
}

/// Total output (RxPDO) image bytes across the configured bus.
pub const OUT_BYTES: usize = out_bytes();
/// Total input (TxPDO) image bytes across the configured bus.
pub const IN_BYTES: usize = BUS.image_size - OUT_BYTES;

const MOSI_HDR: usize = 8;
const MISO_HDR: usize = 18;
const CRC_LEN: usize = 2;
/// One leading byte for the streamed-sample count (`stream_count`).
const STREAM_HDR: usize = 1;
/// Per-sample stride on the wire: 4-byte absolute cycle-index tag + payload.
const SAMPLE_STRIDE: usize = 4 + STREAM_SAMPLE_BYTES;

/// Reserved bytes for streamed motion samples in the MOSI frame (the batch-
/// refill region). Zero when no `<motionStream>` is configured.
pub const HOST_STREAM_BYTES: usize = MAX_SAMPLES_PER_FRAME * SAMPLE_STRIDE;

/// MOSI frame length (header + immediate outputs + streamed region + CRC).
pub const MOSI_LEN: usize = MOSI_HDR + OUT_BYTES + STREAM_HDR + HOST_STREAM_BYTES + CRC_LEN;
/// MISO frame length.
pub const MISO_LEN: usize = MISO_HDR + IN_BYTES + CRC_LEN;
/// The wire frame length (full-duplex: both directions clock the same count).
pub const FRAME_LEN: usize = if MOSI_LEN > MISO_LEN { MOSI_LEN } else { MISO_LEN };

const fn out_bytes() -> usize {
    let mut sum = 0usize;
    let mut i = 0usize;
    while i < BUS.slaves.len() {
        sum += BUS.slaves[i].out_size as usize;
        i += 1;
    }
    sum
}

/// Status the cyclic task feeds into the reply frame.
#[derive(Clone, Copy)]
pub struct ReplyStatus {
    pub link_up: bool,
    pub phase: u8,
    pub wkc: u8,
    pub expected_wkc: u8,
    pub cycle_index: u32,
    pub fault_flags: u8,
    pub host_timeout: bool,
}

/// Shared bridge state between the LPSPI task (prio 2) and the cyclic task
/// (prio 3). Holds the last validated outputs and the prebuilt reply.
pub struct HostBridge {
    /// Last validated immediate outputs (host -> drive).
    staged_out: [u8; OUT_BYTES],
    flags: u8,
    host_seq: u16,
    host_wdog: u16,
    /// host_wdog value the cyclic task last observed (stall detection).
    last_seen_wdog: u16,
    /// Cyclic ticks since `host_wdog` last advanced.
    wdog_stall_cycles: u16,
    out_valid: bool,
    /// Prebuilt MISO reply (staged by the cyclic task, sent by the SPI task).
    staged_in: [u8; FRAME_LEN],
    teensy_wdog: u16,
    crc_errs: u32,
    seq_errs: u32,
    frames_in: u32,
    /// Look-ahead motion samples (empty/inactive when no `<motionStream>`).
    motion: MotionRing,
}

impl HostBridge {
    pub const fn new() -> Self {
        Self {
            staged_out: [0; OUT_BYTES],
            flags: 0,
            host_seq: 0,
            host_wdog: 0,
            last_seen_wdog: 0,
            wdog_stall_cycles: 0,
            out_valid: false,
            staged_in: [0; FRAME_LEN],
            teensy_wdog: 0,
            crc_errs: 0,
            seq_errs: 0,
            frames_in: 0,
            motion: MotionRing::new(STREAM_SAMPLE_BYTES),
        }
    }

    /// Validate and absorb an inbound MOSI frame (called by the SPI task). On a
    /// bad magic/CRC the frame is dropped and `crc_errs` incremented; the staged
    /// outputs are left unchanged so a corrupt frame never reaches the drive.
    pub fn ingest(&mut self, mosi: &[u8]) {
        if mosi.len() < MOSI_LEN {
            self.crc_errs = self.crc_errs.wrapping_add(1);
            return;
        }
        let frame = &mosi[..MOSI_LEN];
        let magic = u16::from_le_bytes([frame[0], frame[1]]);
        if magic != MAGIC {
            self.crc_errs = self.crc_errs.wrapping_add(1);
            return;
        }
        let crc_pos = MOSI_LEN - CRC_LEN;
        let want = u16::from_le_bytes([frame[crc_pos], frame[crc_pos + 1]]);
        if crc16(&frame[..crc_pos]) != want {
            self.crc_errs = self.crc_errs.wrapping_add(1);
            return;
        }

        let seq = u16::from_le_bytes([frame[4], frame[5]]);
        if self.out_valid && seq == self.host_seq {
            // Duplicate frame (host did not advance): not an error, ignore.
            self.seq_errs = self.seq_errs.wrapping_add(1);
        }
        self.flags = frame[3];
        self.host_seq = seq;
        self.host_wdog = u16::from_le_bytes([frame[6], frame[7]]);
        self.staged_out
            .copy_from_slice(&frame[MOSI_HDR..MOSI_HDR + OUT_BYTES]);
        self.out_valid = true;
        self.frames_in = self.frames_in.wrapping_add(1);

        // Streamed motion block: a count byte then up to MAX_SAMPLES_PER_FRAME
        // tagged samples (batch refill). Inert when no `<motionStream>`.
        if self.motion.active() {
            let stream_off = MOSI_HDR + OUT_BYTES;
            let count = (frame[stream_off] as usize).min(MAX_SAMPLES_PER_FRAME);
            let samples = &frame[stream_off + STREAM_HDR..MOSI_LEN - CRC_LEN];
            for i in 0..count {
                let base = i * SAMPLE_STRIDE;
                if base + SAMPLE_STRIDE > samples.len() {
                    break;
                }
                let tag = u32::from_le_bytes([
                    samples[base],
                    samples[base + 1],
                    samples[base + 2],
                    samples[base + 3],
                ]);
                self.motion
                    .push(tag, &samples[base + 4..base + SAMPLE_STRIDE]);
            }
        }
    }

    /// Apply outputs into the process image before the LRW is built (called by
    /// the cyclic task with the current absolute cycle index): the staged
    /// immediate outputs, then the motion sample tagged for this cycle. No-op
    /// until the first valid frame.
    pub fn apply_outputs(&mut self, image: &mut [u8], cycle_index: u32) {
        if !self.out_valid {
            return;
        }
        let n = OUT_BYTES.min(image.len());
        image[..n].copy_from_slice(&self.staged_out[..n]);

        if self.motion.active() {
            if let Some(slot) = self.motion.pop(cycle_index) {
                let payload = self.motion.payload(slot);
                for f in STREAM_FIELDS {
                    let so = f.sample_off as usize;
                    let io = f.image_off as usize;
                    let len = f.len as usize;
                    if so + len <= payload.len() && io + len <= image.len() {
                        image[io..io + len].copy_from_slice(&payload[so..so + len]);
                    }
                }
            }
        }
    }

    /// Advance the host-watchdog stall counter once per cyclic tick. Returns the
    /// number of consecutive ticks the host heartbeat has been stalled.
    pub fn tick_watchdog(&mut self) -> u16 {
        if self.host_wdog != self.last_seen_wdog {
            self.last_seen_wdog = self.host_wdog;
            self.wdog_stall_cycles = 0;
        } else if self.out_valid {
            self.wdog_stall_cycles = self.wdog_stall_cycles.saturating_add(1);
        }
        self.wdog_stall_cycles
    }

    /// Build the MISO reply from the live input image + status (called by the
    /// cyclic task after the tick). Increments the Teensy heartbeat.
    pub fn build_reply(&mut self, image: &[u8], st: ReplyStatus) {
        self.teensy_wdog = self.teensy_wdog.wrapping_add(1);
        let f = &mut self.staged_in;
        f[..MISO_LEN].fill(0);
        f[0..2].copy_from_slice(&MAGIC.to_le_bytes());
        f[2] = VERSION;
        let mut sflags = 0u8;
        if st.link_up {
            sflags |= status::LINK;
        }
        if st.phase == 2 {
            sflags |= status::OPERATIONAL;
        }
        if st.phase == 3 {
            sflags |= status::FAULT;
        }
        if st.host_timeout {
            sflags |= status::HOST_TIMEOUT;
        }
        f[3] = sflags;
        f[4..6].copy_from_slice(&self.host_seq.to_le_bytes());
        f[6..8].copy_from_slice(&self.teensy_wdog.to_le_bytes());
        f[8..12].copy_from_slice(&st.cycle_index.to_le_bytes());
        f[12] = st.wkc;
        f[13] = st.expected_wkc;
        f[14] = st.phase;
        f[15] = st.fault_flags;
        f[16] = self.motion.depth().min(255) as u8;
        f[17] = 0;
        let in_off = BUS.image_size - IN_BYTES; // = OUT_BYTES
        let n = IN_BYTES.min(image.len().saturating_sub(in_off));
        f[MISO_HDR..MISO_HDR + n].copy_from_slice(&image[in_off..in_off + n]);
        let crc_pos = MISO_LEN - CRC_LEN;
        let crc = crc16(&f[..crc_pos]);
        f[crc_pos..crc_pos + 2].copy_from_slice(&crc.to_le_bytes());
    }

    /// The staged reply bytes (sent by the SPI task on the next frame).
    pub fn reply(&self) -> &[u8] {
        &self.staged_in[..MISO_LEN]
    }

    // ── Host intent (consumed by cia402 / safe-state) ──
    pub fn enable_requested(&self) -> bool {
        self.out_valid && self.flags & flag::ENABLE != 0
    }
    pub fn fault_reset_requested(&self) -> bool {
        self.out_valid && self.flags & flag::FAULT_RESET != 0
    }
    pub fn quick_stop_requested(&self) -> bool {
        self.flags & flag::QUICK_STOP != 0
    }
    pub fn has_host(&self) -> bool {
        self.out_valid
    }

    // ── Motion stream ──
    pub fn motion_active(&self) -> bool {
        self.motion.active()
    }
    /// An active motion stream that has started but had no sample for the
    /// current cycle (the look-ahead buffer underran) -> safe-state quick-stop.
    pub fn motion_underrun(&self) -> bool {
        self.motion.active() && self.motion.underrun()
    }
    pub fn motion_depth(&self) -> usize {
        self.motion.depth()
    }

    // ── Diagnostics ──
    pub fn crc_errs(&self) -> u32 {
        self.crc_errs
    }
    pub fn seq_errs(&self) -> u32 {
        self.seq_errs
    }
    pub fn frames_in(&self) -> u32 {
        self.frames_in
    }
    pub fn host_wdog(&self) -> u16 {
        self.host_wdog
    }
    pub fn teensy_wdog(&self) -> u16 {
        self.teensy_wdog
    }
    pub fn stall_cycles(&self) -> u16 {
        self.wdog_stall_cycles
    }
}

impl Default for HostBridge {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        let mut i = 0;
        while i < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            i += 1;
        }
    }
    crc
}
