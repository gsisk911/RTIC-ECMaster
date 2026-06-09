//! Generic Teensy 4.1 Rust foundation that blinks two configured LEDs.

#![no_std]
#![no_main]

mod board;
mod ethercat;
mod hal;
mod net;

use core::sync::atomic::{AtomicBool, Ordering};
use core::ptr::{addr_of, addr_of_mut};
use core::{cell::UnsafeCell, fmt::Write as _, mem::MaybeUninit};
use board::fast_gpio::FastGpioOutput;
use board::teensy_pin_map::{teensy_pin_to_fast_gpio, FastGpio, TeensyBoard};
use core::panic::PanicInfo;
use cortex_m_rt::{exception, ExceptionFrame};
use imxrt_ral as ral;
use ral::iomuxc;
use rtic_monotonics::systick::prelude::*;
use teensy4_bsp as bsp;
use usb_device::{
    bus::UsbBusAllocator,
    class_prelude::UsbClass,
    device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    endpoint::EndpointAddress,
    UsbError,
    UsbDirection,
};
use usbd_serial::CdcAcmClass;
use net::enet_driver::EnetDevice;
use net::enet_ring::{RxDT, TxDT};
use ethercat::cli;
use ethercat::cyclic::{CyclicStatus, Phase as CyclicPhase};
use ethercat::device::{Device, ECAT_MTU, ECAT_RX_LEN, ECAT_TX_LEN};
use ethercat::ecrt::EcError;
use ethercat::master::{Master, Outcome, Request};
use hal::process_data as pdi;
use board::host_spi::{self, HostSpi, ServiceEvent};
use hal::host_bridge::{HostBridge, FRAME_LEN};
use rtic::Mutex;

systick_monotonic!(Mono, 1_000);

const USB_VID_PID: UsbVidPid = UsbVidPid(0x16C0, 0x0483);
const USB_PRODUCT: &str = "Teensy Rust Modbus Base";
const USB_MAX_PACKET_SIZE: usize = 512;
const USB_EP0_CONTROL_PACKET_SIZE: usize = 64;
const USB_ENDPOINT_BYTES: usize = USB_MAX_PACKET_SIZE * 2 + USB_EP0_CONTROL_PACKET_SIZE * 2 + 128;
const FW_NAME: &str = "teensy-rust-modbus-base";
const FW_VERSION: &str = env!("CARGO_PKG_VERSION");
/// Git build-provenance tag captured at compile time by `build.rs`
/// (`v<pkg>-g<short-sha>[-dirty]`, or `v<pkg>-nogit` outside a git checkout),
/// surfaced in the boot report so the running build is identifiable over serial.
const FW_TAG: &str = env!("FW_TAG");
const LED_INDICATOR_TEENSY_PIN: u8 = parse_u8(env!("LED_INDICATOR_PIN"));
const LED_A_TEENSY_PIN: u8 = parse_u8(env!("BASE_LED_A_PIN"));
const LED_B_TEENSY_PIN: u8 = parse_u8(env!("BASE_LED_B_PIN"));
const BLINK_HZ: u32 = parse_u32(env!("BASE_LED_BLINK_HZ"));
// Raspberry Pi / LinuxCNC host-SPI bridge pins (LPSPI3 slave); see host_spi.rs.
const HOST_SPI_SDO_TEENSY_PIN: u8 = parse_u8(env!("HOST_SPI_SDO_PIN"));
const HOST_SPI_SCK_TEENSY_PIN: u8 = parse_u8(env!("HOST_SPI_SCK_PIN"));
const HOST_SPI_SDI_TEENSY_PIN: u8 = parse_u8(env!("HOST_SPI_SDI_PIN"));
const HOST_SPI_CS_TEENSY_PIN: u8 = parse_u8(env!("HOST_SPI_CS_PIN"));
const HOST_SPI_FRAME_READY_TEENSY_PIN: u8 = parse_u8(env!("HOST_SPI_FRAME_READY_PIN"));
const LED_SWAP_PERIOD_MS: u32 = 1_000 / (BLINK_HZ * 2);
/// Lines in the one-time boot banner emitted once per USB attach.
const BOOT_BANNER_LINES: u8 = 2;
const BOARD: TeensyBoard = TeensyBoard::Teensy41;
/// Source MAC for EtherCAT frames (locally-administered; slaves ignore it).
const ECAT_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

static USB_ENDPOINT_MEMORY: imxrt_usbd::EndpointMemory<USB_ENDPOINT_BYTES> =
    imxrt_usbd::EndpointMemory::new();
static USB_ENDPOINT_STATE: imxrt_usbd::EndpointState<6> = imxrt_usbd::EndpointState::new();

type UsbBus = imxrt_usbd::BusAdapter;
type UsbAllocator = UsbBusAllocator<UsbBus>;
type UsbMonitor = board::usb_bootloader::Monitor<UsbBus>;
type UsbSerial = CdcAcmClass<'static, UsbBus>;
type UsbDeviceInstance = UsbDevice<'static, UsbBus>;
type EcatMaster = Master<'static>;

/// `Send` wrapper so the master can live in an RTIC resource. The master holds
/// `&'static mut` ENET descriptor tables containing raw pointers (`!Send`).
pub struct EcatMasterCell(EcatMaster);

// SAFETY: single-core MCU. The cell is a `local` resource owned exclusively by
// the `ethercat_worker` task; the raw pointers inside reference static DMA
// descriptor tables and are never accessed from another context.
unsafe impl Send for EcatMasterCell {}

/// `Send` wrapper for the LPSPI3 slave transport (the RAL instance holds raw
/// register pointers). Owned exclusively by the prio-2 `host_spi_task`.
pub struct HostSpiCell(HostSpi);

// SAFETY: single-core MCU. The cell is a `local` resource owned by exactly one
// task; the LPSPI register pointers are never touched from another context.
unsafe impl Send for HostSpiCell {}

/// Maximum lines in one command response (enough for `pdos`/`pd` dumps).
const ECAT_RESP_LINES: usize = 40;
/// A multi-line command response produced by the worker.
type RespLines = heapless::Vec<heapless::String<96>, ECAT_RESP_LINES>;

/// Byte capacity of one command's full (multi-line) response.
const ECAT_OUT_CAP: usize = ECAT_RESP_LINES * 98;

/// Command-response output: a single flat byte buffer the worker fills and
/// `usb_isr` drains in USB-packet-sized chunks. Flattening all response lines
/// into one buffer lets a multi-line reply flush in as few host transfers as
/// possible (one packet carries many lines), so the response arrives promptly
/// and intact instead of dribbling out one line per USB frame.
pub struct EcatOut {
    buf: heapless::Vec<u8, ECAT_OUT_CAP>,
    cursor: usize,
}

impl EcatOut {
    const fn new() -> Self {
        Self {
            buf: heapless::Vec::new(),
            cursor: 0,
        }
    }

    /// Flatten the worker's response lines into the buffer (CRLF-terminated).
    fn set_lines(&mut self, lines: &RespLines) {
        self.buf.clear();
        self.cursor = 0;
        for line in lines {
            let _ = self.buf.extend_from_slice(line.as_bytes());
            let _ = self.buf.extend_from_slice(b"\r\n");
        }
    }

    /// Load a single CRLF-terminated line as the pending output, replacing any
    /// prior content. Used to stream one scan-progress line at a time.
    fn load_line(&mut self, line: &str) {
        self.buf.clear();
        self.cursor = 0;
        let _ = self.buf.extend_from_slice(line.as_bytes());
        let _ = self.buf.extend_from_slice(b"\r\n");
    }

    /// Bytes not yet handed to the host.
    fn remaining(&self) -> &[u8] {
        &self.buf[self.cursor.min(self.buf.len())..]
    }

    fn pending(&self) -> bool {
        self.cursor < self.buf.len()
    }

    fn advance(&mut self, n: usize) {
        self.cursor = (self.cursor + n).min(self.buf.len());
    }
}

/// Set true by `usb_isr` once the USB device is configured. The EtherCAT worker
/// waits on this before touching the (shared) master, so the master lock --
/// whose priority ceiling is raised by the cyclic PIT task and masks `usb_isr`
/// -- never stalls USB enumeration during the blocking boot scan.
static USB_READY: AtomicBool = AtomicBool::new(false);

static USB_BUS: Singleton<UsbAllocator> = Singleton::uninit();
static USB_MONITOR: Singleton<UsbMonitor> = Singleton::uninit();
static USB_SERIAL: Singleton<UsbSerial> = Singleton::uninit();
static USB_DEVICE: Singleton<UsbDeviceInstance> = Singleton::uninit();

// EtherCAT ENET DMA descriptor tables (static so the EnetDevice can hold
// 'static mutable references; 64-byte aligned via RxDT/TxDT).
static ECAT_RXDT: Singleton<RxDT<ECAT_MTU, ECAT_RX_LEN>> = Singleton::uninit();
static ECAT_TXDT: Singleton<TxDT<ECAT_MTU, ECAT_TX_LEN>> = Singleton::uninit();

const fn parse_u8(value: &str) -> u8 {
    let parsed = parse_u32(value);
    if parsed > u8::MAX as u32 {
        panic!("configured value does not fit in u8");
    }
    parsed as u8
}

const fn parse_u32(value: &str) -> u32 {
    let bytes = value.as_bytes();
    let mut index = 0;
    let mut parsed = 0u32;

    if bytes.is_empty() {
        panic!("configured value must not be empty");
    }

    while index < bytes.len() {
        let byte = bytes[index];
        if byte < b'0' || byte > b'9' {
            panic!("configured value must be numeric");
        }
        parsed = parsed * 10 + (byte - b'0') as u32;
        index += 1;
    }

    parsed
}

fn led_fast_gpio(teensy_pin: u8) -> FastGpio {
    match teensy_pin_to_fast_gpio(BOARD, teensy_pin) {
        Some(gpio) => gpio,
        _ => panic!("unsupported base LED pin"),
    }
}

fn configure_led_pad(teensy_pin: u8) {
    unsafe {
        let mux = iomuxc::IOMUXC::instance();

        match teensy_pin {
            4 => {
                ral::write_reg!(iomuxc, mux, SW_MUX_CTL_PAD_GPIO_EMC_06, MUX_MODE: 5, SION: 0);
                ral::write_reg!(iomuxc, mux, SW_PAD_CTL_PAD_GPIO_EMC_06, 0x10B0);
            }
            5 => {
                ral::write_reg!(iomuxc, mux, SW_MUX_CTL_PAD_GPIO_EMC_08, MUX_MODE: 5, SION: 0);
                ral::write_reg!(iomuxc, mux, SW_PAD_CTL_PAD_GPIO_EMC_08, 0x10B0);
            }
            13 => {
                ral::write_reg!(iomuxc, mux, SW_MUX_CTL_PAD_GPIO_B0_03, MUX_MODE: 5, SION: 0);
                ral::write_reg!(iomuxc, mux, SW_PAD_CTL_PAD_GPIO_B0_03, 0x10B0);
            }
            _ => panic!("unsupported base LED pin"),
        }
    }
}

fn drive_bootloader_safe_outputs() {
    let mut indicator = FastGpioOutput::new(led_fast_gpio(LED_INDICATOR_TEENSY_PIN));
    let mut led_a = FastGpioOutput::new(led_fast_gpio(LED_A_TEENSY_PIN));
    let mut led_b = FastGpioOutput::new(led_fast_gpio(LED_B_TEENSY_PIN));

    indicator.clear();
    led_a.clear();
    led_b.clear();
}

/// A scalar snapshot of the host-bridge state, copied out under the lock so the
/// (slower) string formatting runs *outside* the ceiling-3 critical section and
/// does not add jitter to the cyclic tick.
#[derive(Clone, Copy)]
struct HostSnapshot {
    has_host: bool,
    frames_in: u32,
    crc_errs: u32,
    seq_errs: u32,
    host_wdog: u16,
    stall: u16,
    teensy_wdog: u16,
    motion_active: bool,
    motion_depth: usize,
    motion_underrun: bool,
}

/// Snapshot the host bridge under the lock (cheap, no formatting).
fn host_snapshot(hb: &mut HostBridge) -> HostSnapshot {
    HostSnapshot {
        has_host: hb.has_host(),
        frames_in: hb.frames_in(),
        crc_errs: hb.crc_errs(),
        seq_errs: hb.seq_errs(),
        host_wdog: hb.host_wdog(),
        stall: hb.stall_cycles(),
        teensy_wdog: hb.teensy_wdog(),
        motion_active: hb.motion_active(),
        motion_depth: hb.motion_depth(),
        motion_underrun: hb.motion_underrun(),
    }
}

/// Format the host-bridge diagnostics (outside the lock) for the `host` command.
fn host_lines(s: HostSnapshot) -> RespLines {
    let mut lines = RespLines::new();
    let mut l: heapless::String<96> = heapless::String::new();
    let _ = write!(
        l,
        "[host] link={} frames={} crc_err={} seq_err={}",
        if s.has_host { "on" } else { "off" },
        s.frames_in,
        s.crc_errs,
        s.seq_errs
    );
    let _ = lines.push(l);
    let mut l: heapless::String<96> = heapless::String::new();
    let _ = write!(
        l,
        "[host] host_wdog={} stall={} teensy_wdog={}",
        s.host_wdog, s.stall, s.teensy_wdog
    );
    let _ = lines.push(l);
    let mut l: heapless::String<96> = heapless::String::new();
    let _ = write!(
        l,
        "[host] motion active={} depth={} underrun={}",
        s.motion_active as u8, s.motion_depth, s.motion_underrun as u8
    );
    let _ = lines.push(l);
    let mut l: heapless::String<96> = heapless::String::new();
    let _ = write!(
        l,
        "[host] frame mosi={} miso={} len={}",
        hal::host_bridge::MOSI_LEN,
        hal::host_bridge::MISO_LEN,
        FRAME_LEN
    );
    let _ = lines.push(l);
    lines
}

unsafe fn init_usb(peripherals: impl imxrt_usbd::Peripherals) {
    let bus = imxrt_usbd::BusAdapter::without_critical_sections(
        peripherals,
        &USB_ENDPOINT_MEMORY,
        &USB_ENDPOINT_STATE,
        imxrt_usbd::Speed::High,
    );
    bus.set_interrupts(true);

    let bus = USB_BUS.write(UsbBusAllocator::new(bus));
    USB_MONITOR.write(board::usb_bootloader::Monitor::new());
    USB_SERIAL.write(CdcAcmClass::new(bus, USB_MAX_PACKET_SIZE as u16));

    let device = UsbDeviceBuilder::new(bus, USB_VID_PID)
        .product(USB_PRODUCT)
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(USB_EP0_CONTROL_PACKET_SIZE as u8)
        .build();

    for idx in 1..8 {
        for direction in [UsbDirection::In, UsbDirection::Out] {
            let address = EndpointAddress::from_parts(idx, direction);
            device.bus().enable_zlt(address);
        }
    }

    USB_DEVICE.write(device);
}

unsafe fn poll_usb(configured: &mut bool) -> bool {
    let device = USB_DEVICE.assume_init_mut();
    let monitor = USB_MONITOR.assume_init_mut();
    let serial = USB_SERIAL.assume_init_mut();

    let mut classes: [&mut dyn UsbClass<UsbBus>; 2] = [monitor, serial];
    let _ = device.poll(&mut classes);

    if device.state() != UsbDeviceState::Configured {
        *configured = false;
        return false;
    }

    if !*configured {
        device.bus().configure();
        *configured = true;
    }

    true
}

fn serial_try_write_line(serial: &mut UsbSerial, args: core::fmt::Arguments<'_>) -> bool {
    let mut line = heapless::String::<192>::new();
    let _ = line.write_fmt(args);
    let _ = line.push_str("\r\n");

    match serial.write_packet(line.as_bytes()) {
        Ok(_) => true,
        Err(UsbError::WouldBlock) => false,
        Err(_) => true,
    }
}

/// Emit one line of the one-time boot banner. Returns true once the line has
/// been flushed to the host (so the caller can advance to the next line). The
/// console mirrors the IgH `ethercat` tool: a short banner on attach, then
/// silence until a command response is produced.
fn serial_try_banner_line(serial: &mut UsbSerial, line: u8) -> bool {
    match line {
        0 => serial_try_write_line(
            serial,
            format_args!("[boot] {} {} ({})", FW_NAME, FW_VERSION, FW_TAG),
        ),
        1 => serial_try_write_line(
            serial,
            format_args!("[boot] EtherCAT master over RMII ENET; type 'help' for commands"),
        ),
        _ => true,
    }
}

/// Idle poll cadence (ms) when no command is queued. Kept small so a freshly
/// typed command is picked up by the worker almost immediately.
const ECAT_IDLE_MS: u32 = 1;

/// Auto-emit cadence (ms) for `monitor` mode: one compact telemetry line per
/// period while the cyclic engine runs. ~500 ms is slow enough to never crowd
/// the command channel, fast enough to watch rate/wkc/jitter/DC live.
const MON_PERIOD_MS: u32 = 500;

/// Cooperative yield: re-poll on the very next executor pass instead of sleeping
/// a whole monotonic tick between datagrams. RTIC's generated waker sets the
/// task pending and re-pends its dispatcher, so a self-wake here yields to any
/// higher-/equal-priority task (USB ISR, a future cyclic PDO task) and then
/// resumes immediately. This keeps the SDO/state FSMs strictly non-blocking
/// while removing the ~1 ms/datagram floor that dominated command turnaround.
async fn yield_now() {
    let mut yielded = false;
    core::future::poll_fn(|cx| {
        if yielded {
            core::task::Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    })
    .await
}

fn al_state_name(state: u8) -> &'static str {
    use ethercat::globals::al_state;
    match state & al_state::MASK {
        al_state::INIT => "INIT",
        al_state::PREOP => "PREOP",
        al_state::BOOT => "BOOT",
        al_state::SAFEOP => "SAFEOP",
        al_state::OP => "OP",
        _ => "?",
    }
}

fn resp_push(out: &mut RespLines, args: core::fmt::Arguments<'_>) {
    let mut s: heapless::String<96> = heapless::String::new();
    let _ = s.write_fmt(args);
    let _ = out.push(s);
}

fn ecat_err_text(e: EcError) -> heapless::String<64> {
    let mut s: heapless::String<64> = heapless::String::new();
    match e {
        EcError::SdoAbort(c) => {
            let _ = write!(s, "SDO abort 0x{:08X}", c);
        }
        EcError::StateChange(c) => {
            let _ = write!(s, "AL status code 0x{:04X}", c);
        }
        EcError::NoSuchSlave => {
            let _ = s.push_str("no such slave");
        }
        EcError::CoeUnsupported => {
            let _ = s.push_str("slave has no CoE");
        }
        EcError::MailboxTimeout => {
            let _ = s.push_str("mailbox timeout");
        }
        EcError::Timeout => {
            let _ = s.push_str("timeout");
        }
        other => {
            let _ = write!(s, "{:?}", other);
        }
    }
    s
}

fn ecat_format_slaves<M: Mutex<T = EcatMasterCell>>(master: &mut M, out: &mut RespLines) {
    master.lock(|m| {
        let master = &m.0;
        resp_push(out, format_args!("[ecat] {} slave(s)", master.slaves().len()));
        for s in master.slaves() {
            resp_push(
                out,
                format_args!(
                    "[ecat] slave {} station={} vid=0x{:08X} pid=0x{:08X} al=0x{:02X} coe={}",
                    s.ring_pos,
                    s.station_addr,
                    s.vendor_id,
                    s.product_code,
                    s.al_state,
                    if s.supports_coe { "yes" } else { "no" }
                ),
            );
        }
    });
}

/// Drive a master request to completion, locking the shared master once per
/// datagram and yielding between steps so USB and the cyclic PIT task run.
async fn ecat_drive<M: Mutex<T = EcatMasterCell>>(
    master: &mut M,
    req: Request,
) -> Result<Outcome, EcError> {
    master.lock(|m| m.0.begin(req))?;
    loop {
        match master.lock(|m| m.0.poll_op()) {
            None => yield_now().await,
            Some(res) => return res,
        }
    }
}

/// Publish one line to the host immediately and wait (bounded) until `usb_isr`
/// has drained it, so streamed lines don't overwrite each other mid-flush. With
/// no host attached the drain never completes, so the wait is capped.
async fn emit_line<O: Mutex<T = EcatOut>>(out: &mut O, line: &str) {
    out.lock(|o| o.load_line(line));
    cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
    for _ in 0..100 {
        if !out.lock(|o| o.pending()) {
            break;
        }
        Mono::delay(2_u32.millis()).await;
    }
}

/// Drive a bus rescan while streaming each scan sub-step's trace line over
/// serial as it completes. Unlike the batch `ecat_run_command` path, progress
/// is visible line-by-line, so a step that faults the firmware still leaves its
/// predecessors on the host's console -- our primary no-SWD scan diagnostic.
async fn run_rescan_traced<M, O>(master: &mut M, out: &mut O)
where
    M: Mutex<T = EcatMasterCell>,
    O: Mutex<T = EcatOut>,
{
    if master.lock(|m| m.0.cyclic_active()) {
        emit_line(out, "[ecat] cyclic active; 'stop' before bus commands").await;
        return;
    }
    if let Err(e) = master.lock(|m| m.0.begin(Request::Rescan)) {
        let mut s: heapless::String<96> = heapless::String::new();
        let _ = write!(s, "[ecat] error: {}", ecat_err_text(e));
        emit_line(out, s.as_str()).await;
        return;
    }
    loop {
        let (res, trace) = master.lock(|m| (m.0.poll_op(), m.0.take_scan_trace()));
        if let Some(line) = trace {
            emit_line(out, line.as_str()).await;
        }
        match res {
            None => yield_now().await,
            Some(result) => {
                let mut s: heapless::String<96> = heapless::String::new();
                match result {
                    Ok(Outcome::Rescanned(n)) => {
                        let _ = write!(s, "[ecat] rescan complete: {} slave(s); type 'slaves'", n);
                    }
                    Ok(_) => {
                        let _ = write!(s, "[ecat] unexpected outcome");
                    }
                    Err(e) => {
                        let _ = write!(s, "[ecat] error: {}", ecat_err_text(e));
                    }
                }
                emit_line(out, s.as_str()).await;
                return;
            }
        }
    }
}

fn reject_busy(out: &mut RespLines) {
    resp_push(out, format_args!("[ecat] cyclic active; 'stop' before bus commands"));
}

fn cyclic_phase_name(p: CyclicPhase) -> &'static str {
    match p {
        CyclicPhase::Priming => "priming",
        CyclicPhase::RequestingOp => "requesting-op",
        CyclicPhase::Operational => "OP",
        CyclicPhase::Faulted => "faulted",
    }
}

/// Format up to 64 bytes of the process image as 16-byte hex rows.
fn dump_image(out: &mut RespLines, img: &[u8]) {
    for (row, chunk) in img.chunks(16).enumerate() {
        let mut s: heapless::String<96> = heapless::String::new();
        let _ = write!(s, "[ecat] {:04X}:", row * 16);
        for b in chunk {
            let _ = write!(s, " {:02X}", b);
        }
        let _ = out.push(s);
    }
}

/// Handle the `pd` command: dump image / read pin / write pin.
fn ecat_pd<M: Mutex<T = EcatMasterCell>>(
    master: &mut M,
    out: &mut RespLines,
    pin: Option<heapless::String<48>>,
    value: Option<i64>,
) {
    match pin {
        None => {
            // Sized to dump a multi-slave process image (the planned bus is
            // ~306 B); `pd <pin>` reads any single pin regardless of this cap.
            let mut img = [0u8; 320];
            let info = master.lock(|m| {
                let st = m.0.cyclic_status();
                let n = m.0.cyclic_image().map(|im| {
                    let n = im.len().min(img.len());
                    img[..n].copy_from_slice(&im[..n]);
                    n
                });
                (st, n)
            });
            match info {
                (Some(st), Some(n)) => {
                    resp_push(
                        out,
                        format_args!(
                            "[ecat] cyclic {} wkc={}/{} cycles={}",
                            cyclic_phase_name(st.phase),
                            st.wkc,
                            st.expected_wkc,
                            st.cycles
                        ),
                    );
                    dump_image(out, &img[..n]);
                }
                _ => resp_push(out, format_args!("[ecat] cyclic not running")),
            }
        }
        Some(name) => match pdi::find(name.as_str()) {
            None => resp_push(out, format_args!("[ecat] unknown pin '{}'", name)),
            Some(p) => match value {
                Some(v) => {
                    let ok = master.lock(|m| match m.0.cyclic_image_mut() {
                        Some(im) => {
                            pdi::write_value(im, p, v);
                            true
                        }
                        None => false,
                    });
                    if ok {
                        resp_push(out, format_args!("[ecat] {} <= {}", name, v));
                    } else {
                        resp_push(out, format_args!("[ecat] cyclic not running"));
                    }
                }
                None => {
                    let val = master.lock(|m| m.0.cyclic_image().map(|im| pdi::read_value(im, p)));
                    match val {
                        Some(v) => resp_push(out, format_args!("[ecat] {} = {} (0x{:X})", name, v, v)),
                        None => resp_push(out, format_args!("[ecat] cyclic not running")),
                    }
                }
            },
        },
    }
}

/// Handle the `stats` command: report the cyclic engine's configured rate,
/// cycle/working-counter health, tick jitter, and DC sync error. One item per
/// line so the parent can scrape it while sweeping rates from 100 Hz to 4 kHz.
fn ecat_stats<M: Mutex<T = EcatMasterCell>>(master: &mut M, out: &mut RespLines) {
    match master.lock(|m| m.0.cyclic_status()) {
        None => resp_push(out, format_args!("[ecat] cyclic not running")),
        Some(st) => {
            resp_push(
                out,
                format_args!(
                    "[ecat] cyclic {} rate={}Hz period={}us cycles={}",
                    cyclic_phase_name(st.phase),
                    st.rate_hz,
                    st.period_us,
                    st.cycles
                ),
            );
            resp_push(out, format_args!("[ecat] wkc={}/{}", st.wkc, st.expected_wkc));
            resp_push(
                out,
                format_args!(
                    "[ecat] latency min={}ns max={}ns jitter={}ns (worst {} cyc)",
                    st.latency_min_ns, st.latency_max_ns, st.jitter_ns, st.latency_max_cyc
                ),
            );
            if st.dc_valid {
                resp_push(
                    out,
                    format_args!(
                        "[ecat] dc-sync latest={}ns max={}ns",
                        st.dc_diff_ns, st.dc_diff_max_ns
                    ),
                );
            } else {
                resp_push(out, format_args!("[ecat] dc-sync n/a (no reading yet)"));
            }
        }
    }
}

/// Compact single-line cyclic telemetry for `monitor` auto-emit, e.g.
/// `[mon] 4000Hz cyc=12345 wkc=3/3 jit=0us dc=0ns`. Reuses the same
/// `CyclicStatus` snapshot the `stats` command formats, kept to one line so a
/// passive `view_teensy_serial.py` viewer can scroll it live.
fn mon_line(st: &CyclicStatus) -> heapless::String<96> {
    let mut s: heapless::String<96> = heapless::String::new();
    let _ = write!(
        s,
        "[mon] {}Hz cyc={} wkc={}/{} jit={}ns dc={}ns",
        st.rate_hz, st.cycles, st.wkc, st.expected_wkc, st.jitter_ns, st.dc_diff_ns
    );
    s
}

/// Periodic `monitor` emit, driven by the prio-1 worker's idle loop. Snapshots
/// the cyclic status under the master lock (the same brief copy `stats` does --
/// no extra lock hold, nothing added to the prio-3 cyclic tick), then publishes
/// one line over the shared `ecat_out` path -- but only when that channel is
/// idle, so it never clobbers a command reply mid-flush. Emitting is independent
/// of who's connected: `usb_isr` ships it when a host is attached, else it waits
/// there harmlessly. `running` makes exactly one "stopped" line print when the
/// engine halts, then stays quiet until it restarts.
fn maybe_emit_monitor<M, O>(master: &mut M, out: &mut O, running: &mut bool)
where
    M: Mutex<T = EcatMasterCell>,
    O: Mutex<T = EcatOut>,
{
    let st = master.lock(|m| m.0.cyclic_status());
    let mut line: heapless::String<96> = heapless::String::new();
    let stopping = match st {
        Some(st) => {
            *running = true;
            line = mon_line(&st);
            false
        }
        None => {
            if !*running {
                return; // engine stopped and already announced; stay silent
            }
            let _ = line.push_str("[mon] cyclic stopped");
            true
        }
    };
    // Publish only when the channel is idle so we never clobber a reply. The
    // one-shot "stopped" line keeps `running` set until it actually goes out,
    // so a momentarily busy channel just defers it instead of dropping it.
    let emitted = out.lock(|o| {
        if o.pending() {
            false
        } else {
            o.load_line(line.as_str());
            true
        }
    });
    if emitted {
        if stopping {
            *running = false;
        }
        cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
    }
}

/// Read the persisted `CRASHLOG` on demand (if `magic` matches) and append the
/// formatted `[crash] ...` lines to a command response. Non-destructive: the
/// record persists until `crashclear`, so a saved crash can be re-read. This is
/// the only reader of `CRASHLOG` outside the fault handlers, and it never runs
/// on the boot/init path.
fn ecat_crashlog(out: &mut RespLines) {
    let mut report: CrashReport = heapless::Vec::new();
    // SAFETY: CRASHLOG lives in `.uninit`; every field is a plain integer. We
    // read `magic` first and only snapshot the record when it matches. The
    // fault handlers that write it run only on a crash, above this task.
    let have = unsafe {
        let p = CRASHLOG.as_mut_ptr();
        if addr_of!((*p).magic).read_volatile() == CRASHLOG_MAGIC {
            let log = core::ptr::read(p);
            fmt_crash_report(&mut report, &log);
            true
        } else {
            false
        }
    };
    if have {
        for line in &report {
            resp_push(out, format_args!("{}", line.as_str()));
        }
    } else {
        resp_push(out, format_args!("[crash] none recorded"));
    }
}

/// Execute one parsed serial command, returning the response lines.
async fn ecat_run_command<M: Mutex<T = EcatMasterCell>>(
    master: &mut M,
    cmd: cli::Command,
) -> RespLines {
    let mut out: RespLines = heapless::Vec::new();
    // Bus-driving commands are rejected while the cyclic engine owns the device.
    let busy = master.lock(|m| m.0.cyclic_active());
    match cmd {
        cli::Command::Empty => {}
        // `host` and `monitor` are served by the worker directly (they read the
        // host bridge / toggle worker-local streaming state, not a master op);
        // these arms are unreachable but keep the match exhaustive.
        cli::Command::Host => {}
        cli::Command::Monitor(_) => {}
        cli::Command::Help => {
            for line in cli::HELP {
                resp_push(&mut out, format_args!("{}", line));
            }
        }
        cli::Command::Error(msg) => {
            resp_push(&mut out, format_args!("[ecat] error: {}", msg));
        }
        cli::Command::Slaves => ecat_format_slaves(master, &mut out),
        cli::Command::Status => {
            let (link, n, cyc) =
                master.lock(|m| (m.0.link_up(), m.0.slaves().len(), m.0.cyclic_status()));
            resp_push(&mut out, format_args!("[ecat] fw {} ({})", FW_VERSION, FW_TAG));
            resp_push(
                &mut out,
                format_args!("[ecat] link={} slaves={}", if link { "up" } else { "down" }, n),
            );
            if let Some(st) = cyc {
                resp_push(
                    &mut out,
                    format_args!(
                        "[ecat] cyclic {} {}Hz wkc={}/{} cycles={} ('stats' for detail)",
                        cyclic_phase_name(st.phase),
                        st.rate_hz,
                        st.wkc,
                        st.expected_wkc,
                        st.cycles
                    ),
                );
            }
        }
        cli::Command::Rescan if busy => reject_busy(&mut out),
        cli::Command::Rescan => match ecat_drive(master, Request::Rescan).await {
            Ok(Outcome::Rescanned(n)) => {
                resp_push(&mut out, format_args!("[ecat] rescan: {} slave(s)", n));
                ecat_format_slaves(master, &mut out);
            }
            Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
            Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
        },
        cli::Command::States { .. } if busy => reject_busy(&mut out),
        cli::Command::States { slave, target } => {
            match ecat_drive(master, Request::SetState { slave, target }).await {
                Ok(Outcome::StateReached(s)) => resp_push(
                    &mut out,
                    format_args!("[ecat] slave {} -> {}", slave, al_state_name(s)),
                ),
                Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
                Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
            }
        }
        cli::Command::Upload { .. } if busy => reject_busy(&mut out),
        cli::Command::Upload {
            slave,
            index,
            subindex,
            ty,
        } => match ecat_drive(master, Request::SdoUpload { slave, index, subindex }).await {
            Ok(Outcome::SdoUploaded(n)) => {
                let mut buf = [0u8; 4];
                master.lock(|m| {
                    let b = m.0.sdo_buf();
                    let k = b.len().min(4);
                    buf[..k].copy_from_slice(&b[..k]);
                });
                let val = cli::format_value(ty, &buf[..n.min(4)]);
                resp_push(
                    &mut out,
                    format_args!("[ecat] {}:0x{:04X}:{:02X} = {}", slave, index, subindex, val),
                );
            }
            Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
            Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
        },
        cli::Command::Download { .. } if busy => reject_busy(&mut out),
        cli::Command::Download {
            slave,
            index,
            subindex,
            ty: _,
            data,
            len,
        } => match ecat_drive(
            master,
            Request::SdoDownload { slave, index, subindex, data, len },
        )
        .await
        {
            Ok(Outcome::SdoDownloaded) => resp_push(
                &mut out,
                format_args!("[ecat] {}:0x{:04X}:{:02X} written", slave, index, subindex),
            ),
            Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
            Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
        },
        cli::Command::Start { .. } if busy => {
            resp_push(&mut out, format_args!("[ecat] cyclic already running; 'stop' first"))
        }
        cli::Command::Start { slave: _, rate_hz } => {
            // Resolve the requested rate to a cyclic period: an explicit `-r<hz>`
            // overrides the compile-time configured period. `start` brings up the
            // WHOLE configured bus (the `-p` position is ignored now that one LRW
            // spans all slaves).
            let cycle_ns = match rate_hz {
                Some(hz) => 1_000_000_000u64 / hz as u64,
                None => ethercat::config::generated::BUS.cycle_ns,
            };
            match ecat_drive(master, Request::StartCyclic { cycle_ns }).await {
                Ok(Outcome::CyclicStarted) => {
                    // Configure + start the PIT only now (deliberate action), not at boot.
                    let load = board::cycle_timer::configure(cycle_ns);
                    board::cycle_timer::start();
                    let n = ethercat::config::generated::BUS.slaves.len();
                    resp_push(
                        &mut out,
                        format_args!(
                            "[ecat] {} slave(s) configured; cyclic PDO started at {} Hz",
                            n,
                            board::cycle_timer::actual_hz(load)
                        ),
                    );
                }
                Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
                Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
            }
        }
        cli::Command::Stop => {
            board::cycle_timer::stop();
            let _ = ecat_drive(master, Request::StopCyclic).await;
            resp_push(&mut out, format_args!("[ecat] cyclic PDO stopped"));
        }
        cli::Command::Stats => ecat_stats(master, &mut out),
        cli::Command::Pdos => {
            resp_push(&mut out, format_args!("[ecat] {} process-data pins:", pdi::all().len()));
            for p in pdi::all() {
                resp_push(
                    &mut out,
                    format_args!(
                        "[ecat] {} {} off={} bit={} len={}",
                        if hal::pin::is_output(p) { "OUT" } else { "IN " },
                        p.name,
                        p.byte_offset,
                        p.bit_pos,
                        p.bit_len
                    ),
                );
            }
        }
        cli::Command::Pd { pin, value } => ecat_pd(master, &mut out, pin, value),
        cli::Command::Crashlog => ecat_crashlog(&mut out),
        cli::Command::Crashclear => {
            // SAFETY: CRASHLOG is plain integers in `.uninit`; clearing `magic`
            // on demand from the worker can't race the fault handlers, which run
            // only on a crash (above this task) and fully rewrite the record.
            unsafe {
                let p = CRASHLOG.as_mut_ptr();
                addr_of_mut!((*p).magic).write_volatile(0);
                cortex_m::asm::dsb();
            }
            resp_push(&mut out, format_args!("[crash] cleared"));
        }
    }
    out
}

struct Singleton<T> {
    value: UnsafeCell<MaybeUninit<T>>,
}

unsafe impl<T> Sync for Singleton<T> {}

impl<T> Singleton<T> {
    const fn uninit() -> Self {
        Self {
            value: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    #[allow(clippy::mut_from_ref)] // single-core, init-once interior mutability
    unsafe fn write(&self, value: T) -> &mut T {
        (*self.value.get()).write(value)
    }

    #[allow(clippy::mut_from_ref)] // single-core, init-once interior mutability
    unsafe fn assume_init_mut(&self) -> &mut T {
        (*self.value.get()).assume_init_mut()
    }

    /// Raw pointer to the (possibly uninitialized) storage, for field-by-field
    /// writes from a minimal-stack fault handler that must not build a `&mut T`.
    unsafe fn as_mut_ptr(&self) -> *mut T {
        (*self.value.get()).as_mut_ptr()
    }
}

/// Show a boot-progress code on the 3 LEDs (indicator=pin13 as bit2,
/// led_a=pin4 as bit1, led_b=pin5 as bit0) and hold it briefly so it can be
/// read by eye. If boot hangs, the final frozen code marks the last `init`
/// stage reached -- our only window without USB or SWD.
fn boot_stage(
    indicator: &mut FastGpioOutput,
    led_a: &mut FastGpioOutput,
    led_b: &mut FastGpioOutput,
    code: u8,
) {
    indicator.write(code & 0b100 != 0);
    led_a.write(code & 0b010 != 0);
    led_b.write(code & 0b001 != 0);
    cortex_m::asm::delay(board::clock_config::CORE_CLOCK_HZ / 3); // ~330 ms
}

/// Build a `FastGpioOutput` for a board LED without panicking on an unmapped
/// pin -- a panic/fault handler must never re-panic. The IOMUXC mux + GDIR were
/// set during `init`; we re-`init` defensively in case the fault preceded that.
fn fault_led(teensy_pin: u8) -> Option<FastGpioOutput> {
    let gpio = teensy_pin_to_fast_gpio(BOARD, teensy_pin)?;
    let out = FastGpioOutput::new(gpio);
    unsafe { out.init() };
    Some(out)
}

/// Crash fallback signal on the indicator LED (pin 13 -- the only LED on the
/// RMII board): repeat `class` long pulses, then `count` short pulses, then a
/// pause, forever.
///
/// The `panic` handler parks here (class 1) after saving the message, so a
/// boot-time panic halts on a readable LED code instead of reboot-looping (the
/// `HardFault` handler instead persists a `CrashLog` and reboots for USB
/// recovery). The saved context is retrievable on demand via `crashlog`.
fn blink_diag(class: u8, count: u8) -> ! {
    let hz = board::clock_config::CORE_CLOCK_HZ;
    let mut ind = fault_led(LED_INDICATOR_TEENSY_PIN);
    loop {
        for _ in 0..class {
            if let Some(o) = ind.as_mut() {
                o.set();
            }
            cortex_m::asm::delay(hz * 6 / 10); // ~0.6 s long pulse
            if let Some(o) = ind.as_mut() {
                o.clear();
            }
            cortex_m::asm::delay(hz * 3 / 10);
        }
        cortex_m::asm::delay(hz * 4 / 10); // separator before the count
        for _ in 0..count {
            if let Some(o) = ind.as_mut() {
                o.set();
            }
            cortex_m::asm::delay(hz / 8); // ~0.12 s short pulse
            if let Some(o) = ind.as_mut() {
                o.clear();
            }
            cortex_m::asm::delay(hz / 5);
        }
        cortex_m::asm::delay(hz * 2); // ~2 s gap before repeating
    }
}

/// DTCM addresses below this mark the main stack as nearly exhausted: with the
/// 64 KiB stack (TEENSY4_STACK_SIZE) the initial SP `_stack_start` is ~0x2001_0000,
/// and the stack grows down toward the bottom of DTCM at 0x2000_0000. A faulting
/// SP, exception-frame pointer, or BFAR under the guard (1 KiB above the DTCM
/// floor) is the signature of a stack overflow.
const FAULT_STACK_GUARD: u32 = 0x2000_0400;
/// Marks a valid `CrashLog` left in non-zeroed RAM by a fault/panic handler.
const CRASHLOG_MAGIC: u32 = 0xC0FF_EE00;
const CRASH_KIND_HARDFAULT: u32 = 1;
const CRASH_KIND_PANIC: u32 = 2;
/// Bytes reserved for a panic message/location captured into the crash log.
const CRASH_MSG_CAP: usize = 96;

/// Fault/panic context persisted across a soft reset.
///
/// Lives in the `.uninit` section (NOLOAD; never cleared by startup), so a
/// handler can record the crash, `sys_reset()`, and let the next boot replay it
/// over USB -- far more reliable than keeping the faulting USB device alive,
/// which the host drops and will not re-enumerate from fault context. `magic`
/// gates validity and is cleared once the context is latched for printing. All
/// fields are plain integers, so any residual RAM bit pattern reads as a valid
/// value.
#[repr(C)]
struct CrashLog {
    magic: u32,
    kind: u32,
    pc: u32,
    lr: u32,
    sp_frame: u32,
    msp: u32,
    r0: u32,
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    xpsr: u32,
    cfsr: u32,
    hfsr: u32,
    bfar: u32,
    mmfar: u32,
    msg_len: u32,
    msg: [u8; CRASH_MSG_CAP],
}

/// Persisted crash record. `#[link_section = ".uninit.CRASHLOG"]` keeps it out
/// of `.bss` (which the runtime zeroes), so its contents survive the soft reset.
#[link_section = ".uninit.CRASHLOG"]
static CRASHLOG: Singleton<CrashLog> = Singleton::uninit();

/// Max lines in a formatted crash report (HardFault: 4 register lines + a hint).
/// Lines are kept <= 96 chars so they pass losslessly through the command
/// response buffer (`RespLines`) when surfaced by the `crashlog` command.
const CRASH_REPORT_LINES: usize = 6;
type CrashReport = heapless::Vec<heapless::String<96>, CRASH_REPORT_LINES>;

/// `fmt::Write` sink that fills a fixed byte buffer (volatile, overflow dropped).
/// Lets the panic handler capture the message straight into `CRASHLOG` with no
/// heap and no large stack buffer.
struct MsgSink {
    ptr: *mut u8,
    cap: usize,
    len: usize,
}

impl core::fmt::Write for MsgSink {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len >= self.cap {
                break;
            }
            // SAFETY: `ptr..ptr+cap` is the CRASHLOG message buffer; `len < cap`.
            unsafe { self.ptr.add(self.len).write_volatile(b) };
            self.len += 1;
        }
        Ok(())
    }
}

/// Format the persisted crash context into serial lines (all fields in hex).
/// Every line is kept <= 96 chars (the register dump is split across two lines)
/// so nothing is truncated when relayed through the command response buffer.
fn fmt_crash_report(report: &mut CrashReport, log: &CrashLog) {
    let mut line: heapless::String<96> = heapless::String::new();
    match log.kind {
        CRASH_KIND_HARDFAULT => {
            let _ = write!(
                line,
                "[crash] HARDFAULT pc=0x{:08X} lr=0x{:08X} frame_sp=0x{:08X} msp=0x{:08X}",
                log.pc, log.lr, log.sp_frame, log.msp
            );
            let _ = report.push(line);
            let mut line: heapless::String<96> = heapless::String::new();
            let _ = write!(
                line,
                "[crash] cfsr=0x{:08X} hfsr=0x{:08X} bfar=0x{:08X} mmfar=0x{:08X}",
                log.cfsr, log.hfsr, log.bfar, log.mmfar
            );
            let _ = report.push(line);
            let mut line: heapless::String<96> = heapless::String::new();
            let _ = write!(
                line,
                "[crash] r0=0x{:08X} r1=0x{:08X} r2=0x{:08X} r3=0x{:08X}",
                log.r0, log.r1, log.r2, log.r3
            );
            let _ = report.push(line);
            let mut line: heapless::String<96> = heapless::String::new();
            let _ = write!(
                line,
                "[crash] r12=0x{:08X} xpsr=0x{:08X}",
                log.r12, log.xpsr
            );
            let _ = report.push(line);
            if log.sp_frame < FAULT_STACK_GUARD
                || log.msp < FAULT_STACK_GUARD
                || (log.bfar != 0 && log.bfar < FAULT_STACK_GUARD)
            {
                let mut line: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    line,
                    "[crash] hint: SP/frame/BFAR below 0x{:08X}; suspect stack overflow",
                    FAULT_STACK_GUARD
                );
                let _ = report.push(line);
            }
        }
        CRASH_KIND_PANIC => {
            let _ = line.push_str("[crash] PANIC ");
            let n = (log.msg_len as usize).min(CRASH_MSG_CAP);
            for &b in &log.msg[..n] {
                let c = if (0x20..=0x7E).contains(&b) { b as char } else { '.' };
                let _ = line.push(c);
            }
            let _ = report.push(line);
        }
        _ => {
            let _ = write!(line, "[crash] UNKNOWN kind=0x{:08X}", log.kind);
            let _ = report.push(line);
        }
    }
}

/// Rust panic: record the message/location into `CRASHLOG`, then park on the
/// panic LED code (class 1) and HALT. Deliberately does NOT `sys_reset()`: a
/// panic on the boot/init path would otherwise become an infinite no-USB reboot
/// loop (the regression this build fixes). A panic is rare at runtime and a halt
/// is the strictly safer failure mode; the saved message is retrievable on
/// demand via the `crashlog` command after a manual reboot.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // SAFETY: a panic runs above every task with exclusive access; CRASHLOG is
    // plain integers in `.uninit`. We only store, never read uninitialized data.
    unsafe {
        let p = CRASHLOG.as_mut_ptr();
        addr_of_mut!((*p).magic).write_volatile(CRASHLOG_MAGIC);
        addr_of_mut!((*p).kind).write_volatile(CRASH_KIND_PANIC);
        let mut sink = MsgSink {
            ptr: addr_of_mut!((*p).msg) as *mut u8,
            cap: CRASH_MSG_CAP,
            len: 0,
        };
        let _ = write!(sink, "{}", info);
        addr_of_mut!((*p).msg_len).write_volatile(sink.len as u32);
        cortex_m::asm::dsb();
    }
    // HALT on the panic LED code (1 long pulse). Never reset here.
    blink_diag(1, 0)
}

/// CPU fault (bus/usage/alignment/stacking): record the stacked exception
/// context and SCB fault-status registers into `CRASHLOG`, then reboot so the
/// dump is replayed over USB on the next boot. Kept minimal-stack (field stores
/// + reset; no formatting, no large locals) because the fault may itself be a
/// stack overflow.
#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    let scb = &*cortex_m::peripheral::SCB::PTR;
    let p = CRASHLOG.as_mut_ptr();
    addr_of_mut!((*p).magic).write_volatile(CRASHLOG_MAGIC);
    addr_of_mut!((*p).kind).write_volatile(CRASH_KIND_HARDFAULT);
    addr_of_mut!((*p).pc).write_volatile(frame.pc());
    addr_of_mut!((*p).lr).write_volatile(frame.lr());
    addr_of_mut!((*p).sp_frame).write_volatile(frame as *const ExceptionFrame as u32);
    addr_of_mut!((*p).msp).write_volatile(cortex_m::register::msp::read());
    addr_of_mut!((*p).r0).write_volatile(frame.r0());
    addr_of_mut!((*p).r1).write_volatile(frame.r1());
    addr_of_mut!((*p).r2).write_volatile(frame.r2());
    addr_of_mut!((*p).r3).write_volatile(frame.r3());
    addr_of_mut!((*p).r12).write_volatile(frame.r12());
    addr_of_mut!((*p).xpsr).write_volatile(frame.xpsr());
    addr_of_mut!((*p).cfsr).write_volatile(scb.cfsr.read());
    addr_of_mut!((*p).hfsr).write_volatile(scb.hfsr.read());
    addr_of_mut!((*p).bfar).write_volatile(scb.bfar.read());
    addr_of_mut!((*p).mmfar).write_volatile(scb.mmfar.read());
    addr_of_mut!((*p).msg_len).write_volatile(0);
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset()
}

#[rtic::app(device = teensy4_bsp, dispatchers = [GPIO6_7_8_9, LPUART8, GPT1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        ecat_cmd: Option<cli::Command>,
        ecat_out: EcatOut,
        // Owned by the worker (prio 1) and the cyclic PIT task (prio 3); the
        // PIT task is the highest-priority user, so its lock never blocks.
        ecat_master: EcatMasterCell,
        // Host-bridge staging, shared by the prio-2 LPSPI task and the prio-3
        // cyclic task. The SPI task only moves bytes here; it never locks the
        // master, so the cyclic task's master lock still never blocks.
        host_bridge: HostBridge,
    }

    #[local]
    struct Local {
        usb_configured: bool,
        banner_done: bool,
        banner_line: u8,
        led_a: FastGpioOutput,
        led_b: FastGpioOutput,
        ecat_line: heapless::String<128>,
        host_spi: HostSpiCell,
        frame_ready: FastGpioOutput,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut instances: teensy4_bsp::board::Instances = cx.device.into();
        board::clock_config::prepare(
            &mut instances.CCM,
            &mut instances.CCM_ANALOG,
            &mut instances.DCDC,
        );
        let indicator_gpio = led_fast_gpio(LED_INDICATOR_TEENSY_PIN);
        configure_led_pad(LED_INDICATOR_TEENSY_PIN);
        let mut indicator = FastGpioOutput::new(indicator_gpio);
        unsafe {
            indicator.init();
        }
        board::clock_config::flash_indicator(&mut indicator);

        let bsp::board::Resources {
            usb,
            mut gpio2,
            ..
        } = bsp::board::t41(instances);

        let led_a_gpio = led_fast_gpio(LED_A_TEENSY_PIN);
        configure_led_pad(LED_A_TEENSY_PIN);
        let mut led_a = FastGpioOutput::new(led_a_gpio);
        unsafe {
            led_a.init();
        }

        let led_b_gpio = led_fast_gpio(LED_B_TEENSY_PIN);
        configure_led_pad(LED_B_TEENSY_PIN);
        let mut led_b = FastGpioOutput::new(led_b_gpio);
        unsafe {
            led_b.init();
        }

        Mono::start(cx.core.SYST, board::clock_config::CORE_CLOCK_HZ);

        // Stage 1: clocks + LEDs + monotonic up. (RMII-only firmware: the legacy
        // W5500 SPI / Modbus path is NOT brought up -- it would hang here on a
        // board with no W5500 chip, and its SCK shares the pin-13 LED.)
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 1);

        unsafe { init_usb(usb) };
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 2); // USB peripheral configured
        log::info!("[boot] {} {} ({})", FW_NAME, FW_VERSION, FW_TAG);
        log::info!(
            "[boot] core {} Hz (req {}), IPG {} Hz, VDD_SOC {} mV",
            board::clock_config::CORE_CLOCK_HZ,
            board::clock_config::PROFILE.requested_hz,
            board::clock_config::PROFILE.ipg_hz,
            board::clock_config::PROFILE.vdd_soc_mv,
        );

        // ── EtherCAT ENET bring-up (raw Layer-2 transport; RMII only) ──
        unsafe { net::ethernet::setup_clocks_and_pins(&mut gpio2) };
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 3); // ENET clocks/pads/PHY reset
        let ecat_rxdt = unsafe { ECAT_RXDT.write(RxDT::default()) };
        let ecat_txdt = unsafe { ECAT_TXDT.write(TxDT::default()) };
        let enet_inst = unsafe { ral::enet::ENET1::instance() };
        let mut ecat_enet = EnetDevice::new(enet_inst, ecat_rxdt, ecat_txdt);
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 4); // ENET MAC/DMA up
        net::ethernet::setup_phy(&mut ecat_enet);
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 5); // PHY MDIO configured
        let ecat_master = Master::new(Device::new(ecat_enet, ECAT_MAC));
        boot_stage(&mut indicator, &mut led_a, &mut led_b, 6); // master built; init complete
        log::info!("[ecat] ENET initialised; scheduling bus scan");

        // ── Raspberry Pi / LinuxCNC host bridge (LPSPI3 SPI slave) ──
        // Mux the configurable LPSPI3 pads + the FRAME_READY GPIO, then bring up
        // the slave transport sized to the generated bridge frame. The LPSPI and
        // eDMA clock gates are already enabled by `prepare_clocks_and_power`.
        host_spi::configure_pads(
            HOST_SPI_SDO_TEENSY_PIN,
            HOST_SPI_SCK_TEENSY_PIN,
            HOST_SPI_SDI_TEENSY_PIN,
            HOST_SPI_CS_TEENSY_PIN,
        );
        host_spi::configure_frame_ready_pad(HOST_SPI_FRAME_READY_TEENSY_PIN);
        let host_spi_dev = HostSpi::new(FRAME_LEN);
        let frame_ready_gpio = match teensy_pin_to_fast_gpio(BOARD, HOST_SPI_FRAME_READY_TEENSY_PIN)
        {
            Some(g) => g,
            None => panic!("unsupported HOST_SPI_FRAME_READY_PIN"),
        };
        let frame_ready = FastGpioOutput::new(frame_ready_gpio);
        unsafe { frame_ready.init() };

        // The PIT cyclic-tick timer is configured lazily by the `start` command.

        // After init returns, blink_leds takes over pins 4/5 (alternating
        // heartbeat) = "init complete, tasks running".
        blink_leds::spawn().ok();
        ethercat_worker::spawn().ok();

        (
            Shared {
                ecat_cmd: None,
                ecat_out: EcatOut::new(),
                ecat_master: EcatMasterCell(ecat_master),
                host_bridge: HostBridge::new(),
            },
            Local {
                usb_configured: false,
                banner_done: false,
                banner_line: 0,
                led_a,
                led_b,
                ecat_line: heapless::String::new(),
                host_spi: HostSpiCell(host_spi_dev),
                frame_ready,
            },
        )
    }

    #[task(local = [led_a, led_b], priority = 1)]
    async fn blink_leds(cx: blink_leds::Context) {
        loop {
            cx.local.led_a.write(true);
            cx.local.led_b.write(false);
            Mono::delay(LED_SWAP_PERIOD_MS.millis()).await;

            cx.local.led_a.write(false);
            cx.local.led_b.write(true);
            Mono::delay(LED_SWAP_PERIOD_MS.millis()).await;
        }
    }

    #[task(
        binds = USB_OTG1,
        shared = [ecat_cmd, ecat_out],
        local = [
            usb_configured,
            banner_done,
            banner_line,
            ecat_line
        ],
        priority = 2
    )]
    fn usb_isr(mut cx: usb_isr::Context) {
        let usb_ready = unsafe { poll_usb(cx.local.usb_configured) };
        if usb_ready {
            USB_READY.store(true, Ordering::Relaxed);
        }
        if usb_ready {
            let serial = unsafe { USB_SERIAL.assume_init_mut() };

            // 1) Drain typed input. Build the current line and, on a newline,
            //    parse + queue one command. No echo: the console stays silent
            //    until a response is ready (IgH-style request/response), so a
            //    reply is the only thing the host ever sees -- nothing can
            //    corrupt the response or arrive as garbled echo.
            let mut rxbuf = [0u8; 64];
            if let Ok(n) = serial.read_packet(&mut rxbuf) {
                for &b in &rxbuf[..n] {
                    match b {
                        b'\r' | b'\n' => {
                            if !cx.local.ecat_line.is_empty() {
                                let cmd = cli::parse(cx.local.ecat_line.as_str());
                                cx.shared.ecat_cmd.lock(|slot| {
                                    if slot.is_none() {
                                        *slot = Some(cmd);
                                    }
                                });
                                cx.local.ecat_line.clear();
                            }
                        }
                        0x08 | 0x7F => {
                            let _ = cx.local.ecat_line.pop();
                        }
                        0x20..=0x7E => {
                            let _ = cx.local.ecat_line.push(b as char);
                        }
                        _ => {}
                    }
                }
            }

            // 2) Command responses take strict priority and flush as fast as the
            //    host collects them: up to a full USB packet of the (flattened,
            //    multi-line) reply per write, so it arrives promptly and intact.
            let mut wrote_response = false;
            let pending = cx.shared.ecat_out.lock(|o| o.pending());
            if pending {
                let mut chunk = [0u8; USB_MAX_PACKET_SIZE];
                let take = cx.shared.ecat_out.lock(|o| {
                    let rem = o.remaining();
                    let n = rem.len().min(USB_MAX_PACKET_SIZE);
                    chunk[..n].copy_from_slice(&rem[..n]);
                    n
                });
                match serial.write_packet(&chunk[..take]) {
                    Ok(_) => {
                        cx.shared.ecat_out.lock(|o| o.advance(take));
                        wrote_response = true;
                    }
                    Err(UsbError::WouldBlock) => {}
                    Err(_) => cx.shared.ecat_out.lock(|o| o.advance(take)),
                }
            } else if !*cx.local.banner_done {
                // 3a) One-time boot banner, emitted once per USB attach. Scan
                // results are streamed by the worker on demand (`rescan`/`start`),
                // so there is no separate boot-scan summary to announce here.
                if serial_try_banner_line(serial, *cx.local.banner_line) {
                    *cx.local.banner_line += 1;
                    if *cx.local.banner_line >= BOOT_BANNER_LINES {
                        *cx.local.banner_done = true;
                    }
                }
            }

            // 4) If more response remains and the endpoint just accepted a write,
            //    re-pend so we continue draining on the next USB frame without
            //    waiting for an unrelated interrupt. Only re-pend after progress
            //    (never on WouldBlock) so we never spin while the host is busy.
            if wrote_response && cx.shared.ecat_out.lock(|o| o.pending()) {
                cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
            }
        } else {
            // Host detached: re-arm the one-time banner for the next attach.
            *cx.local.banner_done = false;
            *cx.local.banner_line = 0;
            cx.local.ecat_line.clear();
        }

        if board::usb_bootloader::take_bootloader_request() {
            board::usb_bootloader::shutdown_and_enter(drive_bootloader_safe_outputs);
        }
        if board::usb_bootloader::take_reboot_request() {
            board::usb_bootloader::reboot();
        }
    }

    #[task(shared = [ecat_cmd, ecat_out, ecat_master, host_bridge], priority = 1)]
    async fn ethercat_worker(mut cx: ethercat_worker::Context) {
        // Let USB enumerate before touching the (shared) master. The master
        // lock's ceiling is raised to the cyclic PIT task's priority (3), which
        // masks usb_isr (2); holding it across the blocking boot scan would
        // otherwise stall enumeration. Bounded so we proceed even with no host.
        for _ in 0..200 {
            if USB_READY.load(Ordering::Relaxed) {
                break;
            }
            Mono::delay(50_u32.millis()).await;
        }

        // Cooperative ("round-robin") boot: do NOT auto-scan the bus here. The
        // bus scan is a blocking busy-wait that monopolizes the priority-1
        // executor and holds the master lock (whose ceiling, raised by the
        // priority-3 cyclic task, masks usb_isr) -- that froze blink_leds and
        // the USB CLI. The scan now runs only on demand via `rescan` (and as the
        // first step of `start`), once the operator confirms the network.
        cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);

        // `monitor` streaming state, owned entirely by this task: `monitor_on`
        // gates auto-emit, `mon_running` tracks the engine for the one-shot
        // "stopped" line, and `last_mon` paces the cadence. `last_mon` is a raw
        // monotonic tick count; the 1 kHz `Mono` makes 1 tick == 1 ms, so the
        // wrapping delta below is the elapsed milliseconds.
        let mut monitor_on = false;
        let mut mon_running = false;
        let mut last_mon = Mono::now().ticks();

        // Command loop: take a parsed command, drive it (locking the master per
        // datagram, yielding between), publish the response, and wake USB.
        loop {
            let cmd = cx.shared.ecat_cmd.lock(|slot| slot.take());
            match cmd {
                // `rescan` streams its progress line-by-line over serial (each
                // scan sub-step), so it owns the output channel directly rather
                // than returning a single batched response.
                Some(cli::Command::Rescan) => {
                    run_rescan_traced(&mut cx.shared.ecat_master, &mut cx.shared.ecat_out).await;
                }
                // `host` reads the host-bridge staging state (shared with the SPI
                // + cyclic tasks), not the master, so it is served here directly.
                Some(cli::Command::Host) => {
                    let snap = cx.shared.host_bridge.lock(host_snapshot);
                    let lines = host_lines(snap);
                    cx.shared.ecat_out.lock(|o| o.set_lines(&lines));
                    cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
                }
                // `monitor on|off` toggles this task's auto-emit; handled here
                // (not in `ecat_run_command`) because the state is task-local.
                Some(cli::Command::Monitor(mode)) => {
                    monitor_on = match mode {
                        cli::MonitorMode::On => true,
                        cli::MonitorMode::Off => false,
                        cli::MonitorMode::Toggle => !monitor_on,
                    };
                    // Pace the first auto-line a full period after the ack.
                    last_mon = Mono::now().ticks();
                    let mut lines = RespLines::new();
                    resp_push(
                        &mut lines,
                        format_args!(
                            "[ecat] monitor {} (auto-stats ~{}ms)",
                            if monitor_on { "on" } else { "off" },
                            MON_PERIOD_MS
                        ),
                    );
                    cx.shared.ecat_out.lock(|o| o.set_lines(&lines));
                    cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
                }
                Some(cmd) => {
                    let lines = ecat_run_command(&mut cx.shared.ecat_master, cmd).await;
                    cx.shared.ecat_out.lock(|o| o.set_lines(&lines));
                    cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
                }
                None => Mono::delay(ECAT_IDLE_MS.millis()).await,
            }

            // While monitoring, auto-emit one compact telemetry line ~every
            // `MON_PERIOD_MS`, independent of who's connected. Best-effort and
            // non-blocking: it borrows the master only for the brief status
            // snapshot `stats` already uses, never the prio-3 cyclic tick.
            if monitor_on {
                let now = Mono::now().ticks();
                if now.wrapping_sub(last_mon) >= MON_PERIOD_MS {
                    last_mon = now;
                    maybe_emit_monitor(
                        &mut cx.shared.ecat_master,
                        &mut cx.shared.ecat_out,
                        &mut mon_running,
                    );
                }
            }
        }
    }

    /// High-priority cyclic process-data tick, fired by PIT channel 0. Owns the
    /// device for the duration of one short, non-blocking cycle (process the
    /// previous reply, send this cycle's LRW). Priority above `usb_isr` so the
    /// cycle is not delayed by USB or the worker.
    #[task(binds = PIT, shared = [ecat_master, host_bridge], priority = 3)]
    fn cyclic(cx: cyclic::Context) {
        board::cycle_timer::clear_interrupt();
        (cx.shared.ecat_master, cx.shared.host_bridge)
            .lock(|m, host| m.0.host_cycle(host));
    }

    /// LPSPI3 SPI-slave interrupt: drain the host's transaction. On a completed
    /// full-duplex frame, hand the inbound bytes to the shared `HostBridge`,
    /// stage the reply the cyclic task prepared, and re-arm. Priority 2 (above
    /// the worker, below the cyclic tick); it never touches the EtherCAT master.
    #[task(binds = LPSPI3, shared = [host_bridge], local = [host_spi, frame_ready], priority = 2)]
    fn host_spi_task(mut cx: host_spi_task::Context) {
        let spi = &mut cx.local.host_spi.0;
        if spi.service() != ServiceEvent::FrameComplete {
            return;
        }
        let mut reply = [0u8; FRAME_LEN];
        cx.shared.host_bridge.lock(|hb| {
            hb.ingest(spi.rx_frame());
            let r = hb.reply();
            reply[..r.len()].copy_from_slice(r);
        });
        spi.set_next_tx(&reply);
        spi.begin_next_frame();
        // Strobe FRAME_READY so the host can align its next read / detect stalls.
        cx.local.frame_ready.toggle();
    }
}
