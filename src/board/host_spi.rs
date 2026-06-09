//! LPSPI3 SPI-slave transport for the Raspberry Pi / LinuxCNC host bridge.
//!
//! The Teensy is the SPI **peripheral** (slave); the Pi is the controller. Each
//! transaction is a fixed-size, full-duplex frame: the Pi clocks `frame_len`
//! bytes, simultaneously sending the host->Teensy frame (MOSI) and receiving the
//! Teensy->host frame (MISO) that was staged from the previous cyclic tick.
//!
//! The pinned `imxrt-hal` only exposes a master LPSPI driver plus a bare
//! `set_peripheral_enable` mode bit, so this is a raw `imxrt-ral` register
//! driver in the same style as the ENET driver (`src/net/enet_driver.rs`): raw
//! register access, fixed static buffers, and memory fences. v1 is FIFO/
//! interrupt-driven (one word per RX-data interrupt); eDMA (LPSPI3 RX req 15 /
//! TX req 16) is a later optimization and can replace the inner FIFO loop
//! without changing this module's interface.
//!
//! Pins are configurable (Teensy pin numbers come from `.cargo/config.toml`
//! `[env]`, like the LEDs). Only the pads that physically route to LPSPI3 are
//! accepted; [`configure_pads`] documents and enforces the valid set.

use core::sync::atomic::{fence, Ordering};
use imxrt_ral as ral;

/// Maximum host frame size (bytes). The real length is set at construction from
/// the bridge layout; this only bounds the static buffers.
pub const HOST_SPI_MAX_FRAME: usize = 600;

/// LPSPI hardware TX/RX FIFO depth (words) on the i.MX RT1062. Used to bound how
/// far ahead the TX FIFO is pre-filled.
const FIFO_DEPTH: usize = 16;

/// 8-bit SPI words (FRAMESZ = bits - 1).
const FRAME_BITS: u32 = 8;

/// The LPSPI3 slave transport.
pub struct HostSpi {
    lpspi: ral::lpspi::Instance<3>,
    frame_len: usize,
    /// Inbound (MOSI) bytes for the frame currently being clocked.
    rx: [u8; HOST_SPI_MAX_FRAME],
    rx_idx: usize,
    /// Outbound (MISO) bytes being clocked out this frame.
    tx: [u8; HOST_SPI_MAX_FRAME],
    tx_idx: usize,
    /// Outbound bytes staged by the cyclic task for the *next* frame.
    next_tx: [u8; HOST_SPI_MAX_FRAME],
}

/// Outcome of servicing the LPSPI interrupt.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ServiceEvent {
    /// No complete frame yet (still mid-transfer or spurious).
    Pending,
    /// A full `frame_len`-byte frame arrived; read it with [`HostSpi::rx_frame`].
    FrameComplete,
}

impl HostSpi {
    /// Acquire LPSPI3 and configure it as an 8-bit, mode-0 SPI slave for a
    /// `frame_len`-byte frame. Pads must already be muxed via [`configure_pads`].
    ///
    /// SAFETY: steals the LPSPI3 instance. Call once from `init` on the single
    /// core; nothing else touches LPSPI3.
    pub fn new(frame_len: usize) -> Self {
        let lpspi = unsafe { ral::lpspi::LPSPI3::instance() };
        let mut me = Self {
            lpspi,
            frame_len: frame_len.min(HOST_SPI_MAX_FRAME),
            rx: [0; HOST_SPI_MAX_FRAME],
            rx_idx: 0,
            tx: [0; HOST_SPI_MAX_FRAME],
            tx_idx: 0,
            next_tx: [0; HOST_SPI_MAX_FRAME],
        };
        me.init();
        me
    }

    /// Reset and configure the peripheral for slave operation, then enable the
    /// RX-data + error interrupts. Leaves the engine ready to clock its first
    /// frame (TX FIFO pre-filled with zeros until the cyclic task stages data).
    fn init(&mut self) {
        // Disable + full reset (module, then both FIFOs). Each macro borrows the
        // instance field briefly, so no binding is held across the `&mut self`
        // helper call below.
        ral::write_reg!(ral::lpspi, &self.lpspi, CR, MEN: 0, RST: 1);
        ral::write_reg!(ral::lpspi, &self.lpspi, CR, RST: 0, RTF: 1, RRF: 1);
        ral::write_reg!(ral::lpspi, &self.lpspi, CR, 0);

        // Slave mode (clear MASTER).
        ral::modify_reg!(ral::lpspi, &self.lpspi, CFGR1, MASTER: 0);

        // FIFO watermarks: interrupt as soon as any RX word lands; keep TX able
        // to be topped up. RXWATER = 0 -> RDF when RXCOUNT > 0.
        ral::write_reg!(ral::lpspi, &self.lpspi, FCR, RXWATER: 0, TXWATER: 0);

        // Transmit command: 8-bit frames, mode 0 (CPOL=0, CPHA=0), MSB-first,
        // PCS0. PRESCALE/baud are ignored in slave mode (clock is sourced by the
        // controller). Neither direction masked (full duplex).
        ral::write_reg!(
            ral::lpspi, &self.lpspi, TCR,
            FRAMESZ: FRAME_BITS - 1,
            CPOL: 0,
            CPHA: 0,
            LSBF: 0,
            PCS: 0,
            RXMSK: 0,
            TXMSK: 0
        );

        // Pre-fill the TX FIFO so the first transfer clocks defined bytes.
        self.reload_tx_fifo();

        // Enable the module, then unmask RX-data + error interrupts (RTIC unmasks
        // the NVIC line for the bound task).
        ral::modify_reg!(ral::lpspi, &self.lpspi, CR, MEN: 1);
        ral::write_reg!(ral::lpspi, &self.lpspi, IER, RDIE: 1, REIE: 1, TEIE: 1);
    }

    /// Stage the outbound (MISO) frame to be clocked on the *next* transaction.
    /// Called by the cyclic task after it snapshots inputs/status. Excess bytes
    /// beyond `frame_len` are ignored; short slices are zero-padded.
    pub fn set_next_tx(&mut self, frame: &[u8]) {
        let n = frame.len().min(self.frame_len);
        self.next_tx[..n].copy_from_slice(&frame[..n]);
        for b in &mut self.next_tx[n..self.frame_len] {
            *b = 0;
        }
    }

    /// The most recently completed inbound (MOSI) frame.
    pub fn rx_frame(&self) -> &[u8] {
        &self.rx[..self.frame_len]
    }

    /// Service the LPSPI interrupt: drain received words (loading the matching
    /// TX words), and report when a full frame has arrived. Must be short and
    /// non-blocking (runs in the prio-2 host-SPI task).
    pub fn service(&mut self) -> ServiceEvent {
        // Clear any error flags (w1c). A FIFO under/overrun resets the frame so a
        // half-shifted frame is never delivered.
        let (tef, ref_) = ral::read_reg!(ral::lpspi, &self.lpspi, SR, TEF, REF);
        if tef != 0 || ref_ != 0 {
            ral::write_reg!(ral::lpspi, &self.lpspi, SR, TEF: 1, REF: 1);
            self.restart_frame();
            return ServiceEvent::Pending;
        }

        let mut completed = false;
        // Drain all currently-available RX words.
        while ral::read_reg!(ral::lpspi, &self.lpspi, FSR, RXCOUNT) > 0 {
            let word = ral::read_reg!(ral::lpspi, &self.lpspi, RDR, DATA) as u8;
            if self.rx_idx < self.frame_len {
                self.rx[self.rx_idx] = word;
                self.rx_idx += 1;
            }
            // Keep the TX FIFO fed for the bytes still to be clocked this frame.
            self.push_tx_word();

            if self.rx_idx >= self.frame_len {
                completed = true;
                break;
            }
        }

        if completed {
            fence(Ordering::SeqCst);
            ServiceEvent::FrameComplete
        } else {
            ServiceEvent::Pending
        }
    }

    /// Begin the next frame: swap the staged outbound bytes in, reset indices,
    /// and reload the TX FIFO. Called after the task has consumed a completed
    /// frame.
    pub fn begin_next_frame(&mut self) {
        self.tx[..self.frame_len].copy_from_slice(&self.next_tx[..self.frame_len]);
        self.rx_idx = 0;
        self.tx_idx = 0;
        // Flush stale FIFO contents, then pre-fill from the new outbound frame.
        ral::modify_reg!(ral::lpspi, &self.lpspi, CR, RTF: 1, RRF: 1);
        fence(Ordering::SeqCst);
        self.reload_tx_fifo();
    }

    /// Re-prime after an error: drop the partial frame and resync to a frame
    /// boundary (the controller will re-assert CS for the next frame).
    fn restart_frame(&mut self) {
        self.rx_idx = 0;
        self.tx_idx = 0;
        ral::modify_reg!(ral::lpspi, &self.lpspi, CR, RTF: 1, RRF: 1);
        fence(Ordering::SeqCst);
        self.reload_tx_fifo();
    }

    /// Fill the TX FIFO up to its depth (or end of frame) from `tx`.
    fn reload_tx_fifo(&mut self) {
        while self.tx_idx < self.frame_len {
            let used = ral::read_reg!(ral::lpspi, &self.lpspi, FSR, TXCOUNT) as usize;
            if used >= FIFO_DEPTH {
                break;
            }
            self.push_tx_word();
        }
    }

    /// Push one outbound byte (or a zero pad past the end of frame) into TDR if
    /// there is room and bytes remain.
    fn push_tx_word(&mut self) {
        if self.tx_idx >= self.frame_len {
            return;
        }
        if (ral::read_reg!(ral::lpspi, &self.lpspi, FSR, TXCOUNT) as usize) >= FIFO_DEPTH {
            return;
        }
        let b = self.tx[self.tx_idx];
        self.tx_idx += 1;
        ral::write_reg!(ral::lpspi, &self.lpspi, TDR, DATA: b as u32);
    }
}

/// Mux the FRAME_READY GPIO output pad (ALT5 = GPIO). The Teensy **toggles**
/// this line once per completed SPI frame (an edge strobe, not a level), so the
/// host can edge-trigger its next read or detect a stalled Teensy (the line
/// stops toggling). Configurable; the supported pins are the free AD_B0 GPIO
/// pads. The fast-GPIO output itself is built by the caller from
/// [`teensy_pin_map`](crate::board::teensy_pin_map).
///
/// SAFETY: writes IOMUXC mux/pad registers. Called once from `init`.
pub fn configure_frame_ready_pad(teensy_pin: u8) {
    unsafe {
        let mux = ral::iomuxc::IOMUXC::instance();
        const PAD: u32 = 0x10B0;
        match teensy_pin {
            24 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B0_12, MUX_MODE: 5, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B0_12, PAD);
            }
            25 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B0_13, MUX_MODE: 5, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B0_13, PAD);
            }
            _ => panic!("HOST_SPI_FRAME_READY_PIN must be 24 (GPIO_AD_B0_12) or 25 (GPIO_AD_B0_13)"),
        }
    }
}

/// Mux the four LPSPI3 signal pads (SDO out, SCK/SDI/PCS0 in) for the configured
/// Teensy pins and wire the slave-mode input daisies. Only the pins that route
/// to LPSPI3 are accepted (the AD_B1_12..15 group on ALT2, or the AD_B0_00..03
/// group on ALT7); any other pin panics with a clear message, matching the
/// board's `configure_led_pad` convention.
///
/// SAFETY: writes IOMUXC mux/pad/select-input registers. Called once from `init`.
pub fn configure_pads(sdo_pin: u8, sck_pin: u8, sdi_pin: u8, cs_pin: u8) {
    unsafe {
        let mux = ral::iomuxc::IOMUXC::instance();
        // Standard input pad config: keeper, fast slew. Output (SDO) gets a
        // stronger drive. Value 0x10B0 mirrors the board's existing pad config.
        const PAD: u32 = 0x10B0;

        // SDO (peripheral output).
        match sdo_pin {
            26 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B1_14, MUX_MODE: 2, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B1_14, PAD);
                ral::write_reg!(ral::iomuxc, mux, LPSPI3_SDO_SELECT_INPUT, DAISY: 1);
            }
            _ => panic!("HOST_SPI_SDO_PIN must be 26 (GPIO_AD_B1_14 / LPSPI3_SDO)"),
        }

        // SCK (input from controller).
        match sck_pin {
            27 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B1_15, MUX_MODE: 2, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B1_15, PAD);
                ral::write_reg!(ral::iomuxc, mux, LPSPI3_SCK_SELECT_INPUT, DAISY: 1);
            }
            _ => panic!("HOST_SPI_SCK_PIN must be 27 (GPIO_AD_B1_15 / LPSPI3_SCK)"),
        }

        // SDI (input from controller).
        match sdi_pin {
            39 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B1_13, MUX_MODE: 2, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B1_13, PAD);
                ral::write_reg!(ral::iomuxc, mux, LPSPI3_SDI_SELECT_INPUT, DAISY: 1);
            }
            _ => panic!("HOST_SPI_SDI_PIN must be 39 (GPIO_AD_B1_13 / LPSPI3_SDI)"),
        }

        // PCS0 / CS (input from controller).
        match cs_pin {
            38 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B1_12, MUX_MODE: 2, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B1_12, PAD);
                ral::write_reg!(ral::iomuxc, mux, LPSPI3_PCS0_SELECT_INPUT, DAISY: 1);
            }
            0 => {
                ral::write_reg!(ral::iomuxc, mux, SW_MUX_CTL_PAD_GPIO_AD_B0_03, MUX_MODE: 7, SION: 0);
                ral::write_reg!(ral::iomuxc, mux, SW_PAD_CTL_PAD_GPIO_AD_B0_03, PAD);
                ral::write_reg!(ral::iomuxc, mux, LPSPI3_PCS0_SELECT_INPUT, DAISY: 0);
            }
            _ => panic!("HOST_SPI_CS_PIN must be 38 (GPIO_AD_B1_12) or 0 (GPIO_AD_B0_03)"),
        }
    }
}
