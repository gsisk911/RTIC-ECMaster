//! Motion look-ahead ring buffer.
//!
//! Holds future motion samples streamed by the Pi, each tagged with the absolute
//! EtherCAT cycle index it must be applied on. The cyclic task pops the sample
//! tagged for the current cycle and writes its fields into the process image
//! before the LRW is built. Absolute-index tags + batch refill make the stream
//! self-healing under host jitter / dropped SPI frames (design doc Sec 7).
//!
//! Underrun policy: while the host has begun streaming, a tick with no sample
//! for the current cycle is an underrun (the look-ahead depth was the entire
//! jitter window) -> the safe-state forces a CiA-402 quick-stop.

/// Maximum look-ahead depth (samples) held in the ring.
pub const MOTION_RING_DEPTH: usize = 32;
/// Maximum bytes in one streamed sample (compile-time bound on the payload).
pub const MOTION_SAMPLE_MAX: usize = 64;

/// A per-cycle motion sample ring (FIFO keyed by absolute cycle index).
pub struct MotionRing {
    data: [[u8; MOTION_SAMPLE_MAX]; MOTION_RING_DEPTH],
    tag: [u32; MOTION_RING_DEPTH],
    head: usize,
    len: usize,
    sample_len: usize,
    underrun: bool,
    started: bool,
}

impl MotionRing {
    pub const fn new(sample_len: usize) -> Self {
        Self {
            data: [[0; MOTION_SAMPLE_MAX]; MOTION_RING_DEPTH],
            tag: [0; MOTION_RING_DEPTH],
            head: 0,
            len: 0,
            sample_len: if sample_len > MOTION_SAMPLE_MAX {
                MOTION_SAMPLE_MAX
            } else {
                sample_len
            },
            underrun: false,
            started: false,
        }
    }

    /// Whether a motion stream is configured (sample payload is non-empty).
    pub fn active(&self) -> bool {
        self.sample_len > 0
    }

    /// Push one future sample tagged for absolute cycle `tag`. Returns false if
    /// the ring is full (the host is too far ahead; the sample is dropped and
    /// the host's flow control will back off).
    pub fn push(&mut self, tag: u32, payload: &[u8]) -> bool {
        if self.len >= MOTION_RING_DEPTH {
            return false;
        }
        let slot = (self.head + self.len) % MOTION_RING_DEPTH;
        let n = self.sample_len.min(payload.len());
        self.data[slot][..n].copy_from_slice(&payload[..n]);
        for b in &mut self.data[slot][n..self.sample_len] {
            *b = 0;
        }
        self.tag[slot] = tag;
        self.len += 1;
        self.started = true;
        true
    }

    /// Pop the sample for cycle `current`, discarding any stale (already-passed)
    /// samples first. Returns the ring slot index whose payload to apply, or
    /// `None` when there is no sample for this cycle.
    ///
    /// Underrun is set only when the buffer has fully **drained** (`len == 0`)
    /// after starting — i.e. the host stopped feeding it. A non-empty buffer that
    /// holds only *future* samples (the normal priming/leading state) is **not**
    /// an underrun; it returns `None` without arming the safe-state, so the drive
    /// holds position until the stream's tags catch up to the cycle index.
    pub fn pop(&mut self, current: u32) -> Option<usize> {
        while self.len > 0 && cycle_before(self.tag[self.head], current) {
            self.head = (self.head + 1) % MOTION_RING_DEPTH;
            self.len -= 1;
        }
        if self.len > 0 && self.tag[self.head] == current {
            let slot = self.head;
            self.head = (self.head + 1) % MOTION_RING_DEPTH;
            self.len -= 1;
            self.underrun = false;
            Some(slot)
        } else {
            // Drained (not merely leading) => the host fell behind => underrun.
            self.underrun = self.started && self.len == 0;
            None
        }
    }

    /// The payload bytes in `slot` (length = `sample_len`).
    pub fn payload(&self, slot: usize) -> &[u8] {
        &self.data[slot][..self.sample_len]
    }

    /// Current buffered depth (samples ahead).
    pub fn depth(&self) -> usize {
        self.len
    }

    /// Whether the last `pop` underran an active, started stream.
    pub fn underrun(&self) -> bool {
        self.underrun
    }
}

/// Wrapping "is `a` strictly before `b`" for the 32-bit cycle index (treats the
/// difference as signed so the comparison survives counter wrap).
#[inline]
fn cycle_before(a: u32, b: u32) -> bool {
    (b.wrapping_sub(a) as i32) > 0
}
