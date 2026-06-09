//! EtherCAT frame transport seam (the master's link to the NIC).
//!
//! IgH: master/device.c, master/device.h (`ec_device_t`) - the abstraction over
//! the attached NIC for sending/receiving raw EtherCAT frames and tracking link
//! state. In IgH the backend is a patched Linux `net_device` reached through the
//! `ecdev_*` hooks; here it is the Teensy 4.1 RMII ENET driver in `crate::net`.
//! Rust: builds the 14-byte Ethernet header (broadcast dst, fixed src,
//! EtherType 0x88A4 big-endian) and matches replies by datagram index; the
//! borrow-checked `send_frame`/`poll_raw` helpers replace `sk_buff` handling.
//! Dropped (kernel-only): `net_device`/`sk_buff`/NAPI -> ENET DMA buffers;
//! interrupt-driven RX -> polled `poll_raw`; `ecdev_*` registration -> direct
//! ownership of the `EnetDevice`.

use crate::ethercat::ecrt::EcError;
use crate::ethercat::globals::{ETHERCAT_ETHERTYPE, ETH_HEADER_LEN};
use crate::net::enet_driver::EnetDevice;

/// ENET instance index used for EtherCAT (ENET1 on the Teensy 4.1).
pub const ECAT_ENET_INST: u8 = 1;
/// Per-descriptor frame buffer size.
pub const ECAT_MTU: usize = 1536;
/// RX descriptor ring length.
pub const ECAT_RX_LEN: usize = 4;
/// TX descriptor ring length.
pub const ECAT_TX_LEN: usize = 4;

/// Concrete ENET device type used by the EtherCAT master.
pub type EcatEnet<'a> = EnetDevice<'a, ECAT_ENET_INST, ECAT_MTU, ECAT_RX_LEN, ECAT_TX_LEN>;

/// Number of poll iterations awaited for a datagram reply before timing out.
const TRANSACT_POLL_ATTEMPTS: u32 = 50_000;
/// Busy-wait between poll iterations (~sub-microsecond at the M7 core clock).
const TRANSACT_POLL_DELAY_CYCLES: u32 = 200;

/// The master's network device: an owned ENET driver plus our source MAC.
pub struct Device<'a> {
    enet: EcatEnet<'a>,
    src_mac: [u8; 6],
}

/// Per-transaction state for [`Device::pump`]: tracks whether the request has
/// been sent and how many poll ticks have elapsed waiting for its reply.
#[derive(Clone, Copy, Debug)]
pub struct Pump {
    sent: bool,
    expected: u8,
    attempts: u32,
}

impl Pump {
    pub const fn new() -> Self {
        Self {
            sent: false,
            expected: 0,
            attempts: 0,
        }
    }

    /// Reset so the next `pump` re-sends its frame (use when starting a new
    /// transaction / FSM stage).
    pub fn reset(&mut self) {
        self.sent = false;
        self.attempts = 0;
    }
}

impl Default for Pump {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> Device<'a> {
    /// Wrap an initialized ENET driver as the EtherCAT transport.
    pub fn new(enet: EcatEnet<'a>, src_mac: [u8; 6]) -> Self {
        Self { enet, src_mac }
    }

    /// Mutable access to the underlying ENET driver (for link-state polling).
    pub fn enet_mut(&mut self) -> &mut EcatEnet<'a> {
        &mut self.enet
    }

    /// Send one EtherCAT frame (frame header + datagram(s) + WKC) wrapped in an
    /// Ethernet header. The frame is padded to the 60-byte Ethernet minimum.
    ///
    /// Only the 14-byte Ethernet header is built on the stack; the payload is
    /// copied straight into the TX DMA buffer by `EnetDevice::send_frame`, so no
    /// MTU-sized frame buffer is placed on this (scan-path) stack frame.
    pub fn send(&mut self, ecat_frame: &[u8]) -> Result<(), EcError> {
        let n = ETH_HEADER_LEN + ecat_frame.len();
        if n > ECAT_MTU {
            return Err(EcError::Transport);
        }
        let mut header = [0u8; ETH_HEADER_LEN];
        header[0..6].copy_from_slice(&[0xFF; 6]); // destination: broadcast
        header[6..12].copy_from_slice(&self.src_mac); // source
        header[12..14].copy_from_slice(&ETHERCAT_ETHERTYPE.to_be_bytes()); // 0x88A4 (big-endian)
        if self.enet.send_frame(&header, ecat_frame) {
            Ok(())
        } else {
            Err(EcError::Transport)
        }
    }

    /// Poll once for a received EtherCAT frame. Non-EtherCAT frames are consumed
    /// and ignored. On success, the EtherCAT payload (everything after the
    /// Ethernet header) is copied into `out` and its length returned.
    pub fn poll(&mut self, out: &mut [u8]) -> Option<usize> {
        let copied = self.enet.poll_raw(|frame| {
            if frame.len() < ETH_HEADER_LEN {
                return 0usize;
            }
            // EtherType 0x88A4 is big-endian on the wire.
            if frame[12] != 0x88 || frame[13] != 0xA4 {
                return 0usize;
            }
            let payload = &frame[ETH_HEADER_LEN..];
            let n = payload.len().min(out.len());
            out[..n].copy_from_slice(&payload[..n]);
            n
        })?;
        if copied > 0 {
            Some(copied)
        } else {
            None
        }
    }

    /// Non-blocking single-datagram transaction tracker.
    ///
    /// Drives one request/reply without busy-waiting: the first `poll` sends the
    /// (already-built, index-stable) frame; subsequent `poll`s check the RX ring
    /// once each for the matching reply. This is the primitive the protocol FSMs
    /// step once per driver tick (the async worker now, the cyclic PDO task
    /// later), so SDO/state work never stalls the executor or the PDO cycle.
    pub fn pump<'b>(
        &mut self,
        pump: &mut Pump,
        frame: &[u8],
        rx: &'b mut [u8],
        max_attempts: u32,
    ) -> Result<Option<usize>, EcError> {
        if !pump.sent {
            self.send(frame)?;
            pump.expected = frame.get(3).copied().unwrap_or(0);
            pump.sent = true;
            pump.attempts = 0;
            return Ok(None);
        }
        if let Some(n) = self.poll(rx) {
            if n >= 4 && rx[3] == pump.expected {
                pump.sent = false;
                return Ok(Some(n));
            }
        }
        pump.attempts += 1;
        if pump.attempts >= max_attempts {
            pump.sent = false;
            return Err(EcError::Timeout);
        }
        Ok(None)
    }

    /// Send one EtherCAT frame and block until the reply with the matching
    /// datagram index returns (or the deadline elapses). Returns the EtherCAT
    /// frame length copied into `out`.
    ///
    /// Blocking request/response, used only for the startup bus scan (pre-OP,
    /// before any cyclic PDO). The SDO/state FSMs use the non-blocking `pump`.
    pub fn transact(&mut self, ecat_frame: &[u8], out: &mut [u8]) -> Result<usize, EcError> {
        let expected_index = ecat_frame.get(3).copied().unwrap_or(0);
        self.send(ecat_frame)?;
        for _ in 0..TRANSACT_POLL_ATTEMPTS {
            if let Some(n) = self.poll(out) {
                if n >= 4 && out[3] == expected_index {
                    return Ok(n);
                }
                // Unmatched/stray EtherCAT frame: keep waiting for our reply.
            }
            cortex_m::asm::delay(TRANSACT_POLL_DELAY_CYCLES);
        }
        Err(EcError::Timeout)
    }
}
