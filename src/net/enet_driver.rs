//! ENET MAC driver with smoltcp `Device` trait implementation.
//!
//! Vendored from `rt1062-eth-rs` by Tim Vrakas (MIT license).
//! Adapted for this project; uses `log::info!` for USB serial logging.
//! Original ENET register setup and DMA logic kept intact.

use smoltcp::phy::{self, Checksum, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use core::ptr::addr_of;
use core::sync::atomic;

use imxrt_ral as ral;
use ral::enet;

use crate::net::boot_diag;
use crate::net::enet_ring::*;

const MDIO_TIMEOUT_LOOPS: u32 = 5_000_000;

#[derive(Clone, Copy, Debug, Default)]
pub struct EnetMacStats {
    pub rx_packets: u32,
    pub tx_packets: u32,
    pub rx_ok: u32,
    pub tx_ok: u32,
    pub rx_crc: u32,
    pub rx_align: u32,
    pub rx_macerr: u32,
    pub tx_macerr: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct EnetDebugSnapshot<const RX_LEN: usize, const TX_LEN: usize> {
    pub last_rx_progress_ms: u32,
    pub last_tx_progress_ms: u32,
    pub rx_pos: usize,
    pub tx_pos: usize,
    pub rx_flags: [u16; RX_LEN],
    pub rx_lens: [u16; RX_LEN],
    pub tx_flags: [u16; TX_LEN],
    pub tx_lens: [u16; TX_LEN],
    pub rdar: u32,
    pub tdar: u32,
    pub eir: u32,
    pub eimr: u32,
    pub ecr: u32,
}

/// ENET MAC + DMA device.
///
/// Generic parameters:
///   - `INST`: ENET instance index (1 for ENET1 on Teensy 4.1)
///   - `MTU`: max frame size per buffer (typically 1536)
///   - `RX_LEN`: number of RX descriptors
///   - `TX_LEN`: number of TX descriptors
pub struct EnetDevice<
    'a,
    const INST: u8,
    const MTU: usize,
    const RX_LEN: usize,
    const TX_LEN: usize,
> {
    tx_pos: usize,
    rx_pos: usize,
    last_rx_progress_ms: u32,
    last_tx_progress_ms: u32,
    enet_inst: enet::Instance<INST>,
    pub txdt: &'a mut TxDT<MTU, TX_LEN>,
    pub rxdt: &'a mut RxDT<MTU, RX_LEN>,
}

impl<'a, const INST: u8, const MTU: usize, const RX_LEN: usize, const TX_LEN: usize>
    EnetDevice<'a, INST, MTU, RX_LEN, TX_LEN>
{
    fn configure_registers(&mut self) {
        // MII speed (initial, overwritten below with calculated value)
        ral::write_reg!(enet, self.enet_inst, MSCR, MII_SPEED: 9);
        // Interrupt mask: all off (we poll)
        ral::write_reg!(enet, self.enet_inst, EIMR, 0x0);
        // RX control: RMII mode, no loopback, promiscuous, CRC forward, max frame 1522
        ral::modify_reg!(enet, self.enet_inst, RCR,
            RMII_MODE: 1, MII_MODE: 1, LOOP: 0, PROM: 1,
            CRCFWD: 1, DRT: 0, MAX_FL: 1522, NLC: 1, PADEN: 1
        );
        // Byte-swap for little-endian host
        ral::modify_reg!(enet, self.enet_inst, ECR, DBSWP: 1);
        // Full duplex
        ral::modify_reg!(enet, self.enet_inst, TCR, FDEN: 1);
        // Store and forward TX
        ral::modify_reg!(enet, self.enet_inst, TFWR, STRFWD: 1);

        // Calculate MII speed for 150 MHz IPG clock
        let source_clock_hz: u32 = 150_000_000;
        const SMI_MDC_FREQUENCY_HZ: u32 = 2_500_000;
        let mii_speed =
            (source_clock_hz + 2 * SMI_MDC_FREQUENCY_HZ - 1) / (2 * SMI_MDC_FREQUENCY_HZ) - 1;
        let hold_time =
            (10 + 1_000_000_000 / source_clock_hz - 1) / (1_000_000_000 / source_clock_hz) - 1;
        ral::modify_reg!(enet, self.enet_inst, MSCR,
            HOLDTIME: hold_time, MII_SPEED: mii_speed
        );
    }

    fn reset_descriptor_tables(&mut self) {
        self.tx_pos = 0;
        self.rx_pos = 0;
        let tx_len = self.txdt.desc.len();
        let rx_len = self.rxdt.desc.len();

        for (idx, element) in self.txdt.desc.iter_mut().enumerate() {
            element.buffer = addr_of!(self.txdt.bufs[idx][0]);
            element.len = 0;
            element.flags = if idx + 1 == tx_len { 0x2000 } else { 0x0000 };
        }

        for (idx, element) in self.rxdt.desc.iter_mut().enumerate() {
            element.buffer = addr_of!(self.rxdt.bufs[idx][0]);
            element.len = 0;
            element.flags = if idx + 1 == rx_len { 0xA000 } else { 0x8000 };
        }

        ral::write_reg!(
            enet, self.enet_inst, RDSR,
            addr_of!(self.rxdt.desc[0]) as u32
        );
        ral::write_reg!(
            enet, self.enet_inst, TDSR,
            addr_of!(self.txdt.desc[0]) as u32
        );

        // Match the hardware RX buffer size to the per-descriptor frame buffer.
        ral::write_reg!(enet, self.enet_inst, MRBR, R_BUF_SIZE: MTU as u32);
    }

    fn enable_dma(&mut self) {
        // Memory barrier before enabling
        atomic::fence(atomic::Ordering::SeqCst);

        // Enable ENET + IEEE 1588 timestamp
        ral::modify_reg!(enet, self.enet_inst, ECR, ETHEREN: 1, EN1588: 1);
        // Signal that RX descriptors are ready
        ral::write_reg!(enet, self.enet_inst, RDAR, RDAR: 1);
    }

    fn wait_for_mii(&mut self, flag: u32, code: u8, op: &str) -> bool {
        for _ in 0..MDIO_TIMEOUT_LOOPS {
            if ral::read_reg!(enet, self.enet_inst, EIR, MII) != 0 {
                ral::write_reg!(enet, self.enet_inst, EIR, MII: 1);
                return true;
            }
            core::hint::spin_loop();
        }

        if boot_diag::record(flag, code) {
            log::warn!("[enet] timeout waiting for MDIO {} completion", op);
        }
        false
    }

    /// Create and initialise the ENET device.
    ///
    /// `enet_inst` is the RAL ENET instance (e.g. `enet::ENET1::instance()`).
    /// `rxdt` / `txdt` are the static descriptor tables.
    pub fn new(
        enet_inst: enet::Instance<INST>,
        rxdt: &'a mut RxDT<MTU, RX_LEN>,
        txdt: &'a mut TxDT<MTU, TX_LEN>,
    ) -> EnetDevice<'a, INST, MTU, RX_LEN, TX_LEN> {
        let mut device = EnetDevice {
            tx_pos: 0,
            rx_pos: 0,
            last_rx_progress_ms: 0,
            last_tx_progress_ms: 0,
            enet_inst,
            rxdt,
            txdt,
        };

        device.configure_registers();
        device.reset_descriptor_tables();

        log::info!("[enet] descriptor tables configured");
        device.enable_dma();

        device
    }

    /// Reinitialize the MAC/DMA state after a link drop.
    pub fn restart(&mut self) {
        ral::modify_reg!(enet, self.enet_inst, ECR, ETHEREN: 0);
        atomic::fence(atomic::Ordering::SeqCst);
        self.configure_registers();
        self.reset_descriptor_tables();
        self.enable_dma();
    }

    /// Write a PHY register via MDIO.
    pub fn mdio_write(&mut self, phyaddr: u8, regaddr: u8, data: u16) -> bool {
        ral::write_reg!(enet, self.enet_inst, MMFR,
            ST: 1, OP: 1, TA: 0,
            PA: phyaddr as u32, RA: regaddr as u32, DATA: data as u32
        );
        self.wait_for_mii(
            boot_diag::FLAG_MDIO_WRITE_TIMEOUT,
            boot_diag::ERR_MDIO_WRITE_TIMEOUT,
            "write",
        )
    }

    /// Read a PHY register via MDIO.
    pub fn mdio_read(&mut self, phyaddr: u8, regaddr: u8) -> Option<u16> {
        ral::write_reg!(enet, self.enet_inst, MMFR,
            ST: 1, OP: 2, TA: 0,
            PA: phyaddr as u32, RA: regaddr as u32
        );
        if !self.wait_for_mii(
            boot_diag::FLAG_MDIO_READ_TIMEOUT,
            boot_diag::ERR_MDIO_READ_TIMEOUT,
            "read",
        ) {
            return None;
        }
        let data = ral::read_reg!(enet, self.enet_inst, MMFR, DATA) as u16;
        Some(data)
    }

    pub fn mac_stats(&self) -> EnetMacStats {
        EnetMacStats {
            rx_packets: ral::read_reg!(enet, self.enet_inst, RMON_R_PACKETS),
            tx_packets: ral::read_reg!(enet, self.enet_inst, RMON_T_PACKETS),
            rx_ok: ral::read_reg!(enet, self.enet_inst, IEEE_R_FRAME_OK),
            tx_ok: ral::read_reg!(enet, self.enet_inst, IEEE_T_FRAME_OK),
            rx_crc: ral::read_reg!(enet, self.enet_inst, IEEE_R_CRC),
            rx_align: ral::read_reg!(enet, self.enet_inst, IEEE_R_ALIGN),
            rx_macerr: ral::read_reg!(enet, self.enet_inst, IEEE_R_MACERR),
            tx_macerr: ral::read_reg!(enet, self.enet_inst, IEEE_T_MACERR),
        }
    }

    /// Send one complete Ethernet frame (header + payload) on the next free TX
    /// descriptor. Returns `false` if that descriptor is still owned by the DMA
    /// engine (ring momentarily full).
    ///
    /// Raw Layer-2 path for the EtherCAT master; bypasses the smoltcp tokens.
    pub fn send_raw(&mut self, frame: &[u8]) -> bool {
        let pos = self.tx_pos;
        if (self.txdt.desc[pos].flags & 0x8000) != 0 {
            // R (ready) still set: descriptor owned by DMA, prior frame in flight.
            return false;
        }
        let len = frame.len().min(MTU);
        self.txdt.bufs[pos][..len].copy_from_slice(&frame[..len]);
        let desc = &mut self.txdt.desc[pos];
        desc.len = len as u16;
        // R (ready) | L (last) | TC (append CRC); preserves the W (wrap) bit.
        desc.flags |= 0x8C00;
        atomic::fence(atomic::Ordering::SeqCst);
        ral::write_reg!(enet, self.enet_inst, TDAR, TDAR: 1);
        self.tx_pos = if pos + 1 < self.txdt.desc.len() {
            pos + 1
        } else {
            0
        };
        true
    }

    /// Poll the next RX descriptor for a received frame. When present, the frame
    /// bytes are passed to `f`, then the descriptor is returned to the DMA
    /// engine and RX is re-armed. Returns `None` when no frame is ready.
    ///
    /// Raw Layer-2 path for the EtherCAT master; bypasses the smoltcp tokens.
    pub fn poll_raw<R>(&mut self, f: impl FnOnce(&[u8]) -> R) -> Option<R> {
        let pos = self.rx_pos;
        if (self.rxdt.desc[pos].flags & 0x8000) != 0 {
            // E (empty) still set: descriptor owned by DMA, no frame yet.
            return None;
        }
        let len = (self.rxdt.desc[pos].len as usize).min(MTU);
        let result = f(&self.rxdt.bufs[pos][..len]);
        atomic::fence(atomic::Ordering::SeqCst);
        // Return descriptor to ENET (set E bit; W preserved) and re-arm RX.
        self.rxdt.desc[pos].flags |= 0x8000;
        ral::write_reg!(enet, self.enet_inst, RDAR, RDAR: 1);
        self.rx_pos = if pos + 1 < self.rxdt.desc.len() {
            pos + 1
        } else {
            0
        };
        Some(result)
    }

    pub fn debug_snapshot(&self) -> EnetDebugSnapshot<RX_LEN, TX_LEN> {
        let mut rx_flags = [0u16; RX_LEN];
        let mut rx_lens = [0u16; RX_LEN];
        let mut tx_flags = [0u16; TX_LEN];
        let mut tx_lens = [0u16; TX_LEN];

        for (idx, desc) in self.rxdt.desc.iter().enumerate() {
            rx_flags[idx] = desc.flags;
            rx_lens[idx] = desc.len;
        }
        for (idx, desc) in self.txdt.desc.iter().enumerate() {
            tx_flags[idx] = desc.flags;
            tx_lens[idx] = desc.len;
        }

        EnetDebugSnapshot {
            last_rx_progress_ms: self.last_rx_progress_ms,
            last_tx_progress_ms: self.last_tx_progress_ms,
            rx_pos: self.rx_pos,
            tx_pos: self.tx_pos,
            rx_flags,
            rx_lens,
            tx_flags,
            tx_lens,
            rdar: ral::read_reg!(enet, self.enet_inst, RDAR),
            tdar: ral::read_reg!(enet, self.enet_inst, TDAR),
            eir: ral::read_reg!(enet, self.enet_inst, EIR),
            eimr: ral::read_reg!(enet, self.enet_inst, EIMR),
            ecr: ral::read_reg!(enet, self.enet_inst, ECR),
        }
    }
}

// ── smoltcp Device trait ────────────────────────────────────────────────

impl<const INST: u8, const MTU: usize, const RX_LEN: usize, const TX_LEN: usize> phy::Device
    for EnetDevice<'_, INST, MTU, RX_LEN, TX_LEN>
{
    type RxToken<'a> = EnetRxToken<'a, INST, MTU, RX_LEN> where Self: 'a;
    type TxToken<'a> = EnetTxToken<'a, INST, MTU, TX_LEN> where Self: 'a;

    fn receive(&mut self, timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let rxd = &mut self.rxdt.desc[self.rx_pos];
        if (rxd.flags & 0x8000) == 0 {
            let now_ms = timestamp.total_millis() as u32;
            self.last_rx_progress_ms = now_ms;
            let enet_ptr = &*self.enet_inst as *const enet::RegisterBlock;
            Some((
                EnetRxToken {
                    enet_ptr,
                    rx_pos: &mut self.rx_pos,
                    rxdt: &mut self.rxdt,
                },
                EnetTxToken {
                    enet_ptr,
                    timestamp_ms: now_ms,
                    tx_pos: &mut self.tx_pos,
                    last_tx_progress_ms: &mut self.last_tx_progress_ms,
                    txdt: &mut self.txdt,
                },
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, timestamp: Instant) -> Option<Self::TxToken<'_>> {
        let desc: &mut TxDescriptor = &mut self.txdt.desc[self.tx_pos];
        if (desc.flags & 0x8000) == 0x0 {
            let enet_ptr = &*self.enet_inst as *const enet::RegisterBlock;
            Some(EnetTxToken {
                enet_ptr,
                timestamp_ms: timestamp.total_millis() as u32,
                tx_pos: &mut self.tx_pos,
                last_tx_progress_ms: &mut self.last_tx_progress_ms,
                txdt: &mut self.txdt,
            })
        } else {
            None
        }
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps.checksum.ipv4 = Checksum::Both;
        caps
    }
}

/// RX token for smoltcp frame consumption.
pub struct EnetRxToken<'a, const INST: u8, const MTU: usize, const RX_LEN: usize> {
    enet_ptr: *const enet::RegisterBlock,
    rx_pos: &'a mut usize,
    rxdt: &'a mut RxDT<MTU, RX_LEN>,
}

impl<'a, const INST: u8, const MTU: usize, const RX_LEN: usize> phy::RxToken
    for EnetRxToken<'a, INST, MTU, RX_LEN>
{
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let rxd = &mut self.rxdt.desc[*self.rx_pos];
        let result = f(&mut self.rxdt.bufs[*self.rx_pos]);
        atomic::fence(atomic::Ordering::SeqCst);
        // Return descriptor to ENET (set E bit)
        rxd.flags |= 0x8000;
        let enet_inst: enet::Instance<INST> = unsafe { enet::Instance::new(self.enet_ptr) };
        ral::write_reg!(enet, enet_inst, RDAR, RDAR: 1);
        if *self.rx_pos < (self.rxdt.desc.len() - 1) {
            *self.rx_pos += 1;
        } else {
            *self.rx_pos = 0;
        }
        result
    }
}

/// TX token for smoltcp frame transmission.
pub struct EnetTxToken<'a, const INST: u8, const MTU: usize, const TX_LEN: usize> {
    enet_ptr: *const enet::RegisterBlock,
    timestamp_ms: u32,
    tx_pos: &'a mut usize,
    last_tx_progress_ms: &'a mut u32,
    txdt: &'a mut TxDT<MTU, TX_LEN>,
}

impl<'a, const INST: u8, const MTU: usize, const TX_LEN: usize> phy::TxToken
    for EnetTxToken<'a, INST, MTU, TX_LEN>
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.txdt.bufs[*self.tx_pos][0..len]);
        let desc: &mut TxDescriptor = &mut self.txdt.desc[*self.tx_pos];
        *self.last_tx_progress_ms = self.timestamp_ms;
        desc.len = len as u16;
        // Set R (ready), L (last), TC (transmit CRC) flags
        desc.flags |= 0x8C00;
        atomic::fence(atomic::Ordering::SeqCst);
        // Poke TDAR to start transmission
        let enet_inst: enet::Instance<INST> = unsafe { enet::Instance::new(self.enet_ptr) };
        ral::write_reg!(enet, enet_inst, TDAR, TDAR: 1);

        if *self.tx_pos < (self.txdt.desc.len() - 1) {
            *self.tx_pos += 1;
        } else {
            *self.tx_pos = 0;
        }
        result
    }
}
