//! Generic Teensy 4.1 Rust foundation that blinks two configured LEDs.

#![no_std]
#![no_main]

mod board;
mod ethercat;
mod hal;
mod net;

use core::sync::atomic::{AtomicBool, Ordering};
use core::{cell::UnsafeCell, fmt::Write as _, mem::MaybeUninit};
use board::fast_gpio::FastGpioOutput;
use board::teensy_pin_map::{teensy_pin_to_fast_gpio, FastGpio, TeensyBoard};
use imxrt_ral as ral;
use panic_halt as _;
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
use ethercat::cyclic::Phase as CyclicPhase;
use ethercat::device::{Device, ECAT_MTU, ECAT_RX_LEN, ECAT_TX_LEN};
use ethercat::ecrt::EcError;
use ethercat::globals::EC_MAX_SLAVES;
use ethercat::master::{Master, Outcome, Request};
use ethercat::slave::SlaveInfo;
use hal::process_data as pdi;
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
const W5500_SPI_HZ: u32 = parse_u32(env!("W5500_SPI_HZ"));
const W5500_RESET_TEENSY_PIN: u8 = parse_u8(env!("W5500_RESET_PIN"));
const W5500_INT_TEENSY_PIN: u8 = parse_u8(env!("W5500_INT_PIN"));
const LED_SWAP_PERIOD_MS: u32 = 1_000 / (BLINK_HZ * 2);
const W5500_POLL_PERIOD_MS: u32 = 1_000;
/// Lines in the one-time boot banner emitted once per USB attach.
const BOOT_BANNER_LINES: u8 = 2;
const BOARD: TeensyBoard = TeensyBoard::Teensy41;
/// Source MAC for EtherCAT frames (locally-administered; slaves ignore it).
const ECAT_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

const _: () = {
    if W5500_RESET_TEENSY_PIN != 40 {
        panic!("W5500 reset pin must be Teensy pin 40");
    }
    if W5500_INT_TEENSY_PIN != 41 {
        panic!("W5500 interrupt pin must be Teensy pin 41");
    }
};

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

/// Maximum lines in one command response (enough for `pdos`/`pd` dumps).
const ECAT_RESP_LINES: usize = 40;
/// A multi-line command response produced by the worker.
type RespLines = heapless::Vec<heapless::String<96>, ECAT_RESP_LINES>;

/// Bus-scan result shared from the worker to the USB reporter (boot report).
pub struct EcatScan {
    slaves: heapless::Vec<SlaveInfo, EC_MAX_SLAVES>,
    done: bool,
    failed: bool,
}

impl EcatScan {
    const fn new() -> Self {
        Self {
            slaves: heapless::Vec::new(),
            done: false,
            failed: false,
        }
    }
}

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

fn delay_cycles_from_us(us: u32) {
    cortex_m::asm::delay((board::clock_config::CORE_CLOCK_HZ / 1_000_000) * us);
}

fn drive_bootloader_safe_outputs() {
    let mut indicator = FastGpioOutput::new(led_fast_gpio(LED_INDICATOR_TEENSY_PIN));
    let mut led_a = FastGpioOutput::new(led_fast_gpio(LED_A_TEENSY_PIN));
    let mut led_b = FastGpioOutput::new(led_fast_gpio(LED_B_TEENSY_PIN));

    indicator.clear();
    led_a.clear();
    led_b.clear();
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
            let mut img = [0u8; 64];
            let info = master.lock(|m| {
                let st = m.0.cyclic_status();
                let n = m.0.cyclic_image().map(|im| {
                    let n = im.len().min(64);
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
                        "[ecat] cyclic {} wkc={}/{} cycles={}",
                        cyclic_phase_name(st.phase),
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
        cli::Command::Start { slave } => match ecat_drive(master, Request::StartCyclic { slave }).await {
            Ok(Outcome::CyclicStarted) => {
                // Configure + start the PIT only now (deliberate action), not at boot.
                board::cycle_timer::configure(ethercat::config::generated::BUS.cycle_ns);
                board::cycle_timer::start();
                resp_push(
                    &mut out,
                    format_args!("[ecat] slave {} configured; cyclic PDO started", slave),
                );
            }
            Ok(_) => resp_push(&mut out, format_args!("[ecat] unexpected outcome")),
            Err(e) => resp_push(&mut out, format_args!("[ecat] error: {}", ecat_err_text(e))),
        },
        cli::Command::Stop => {
            board::cycle_timer::stop();
            let _ = ecat_drive(master, Request::StopCyclic).await;
            resp_push(&mut out, format_args!("[ecat] cyclic PDO stopped"));
        }
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

#[rtic::app(device = teensy4_bsp, dispatchers = [GPIO6_7_8_9, LPUART8, GPT1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        ecat_scan: EcatScan,
        ecat_cmd: Option<cli::Command>,
        ecat_out: EcatOut,
        // Owned by the worker (prio 1) and the cyclic PIT task (prio 3); the
        // PIT task is the highest-priority user, so its lock never blocks.
        ecat_master: EcatMasterCell,
    }

    #[local]
    struct Local {
        usb_configured: bool,
        banner_done: bool,
        banner_line: u8,
        scan_announced: bool,
        led_a: FastGpioOutput,
        led_b: FastGpioOutput,
        ecat_line: heapless::String<128>,
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

        // The PIT cyclic-tick timer is configured lazily by the `start` command.

        // After init returns, blink_leds takes over pins 4/5 (alternating
        // heartbeat) = "init complete, tasks running".
        blink_leds::spawn().ok();
        ethercat_worker::spawn().ok();

        (
            Shared {
                ecat_scan: EcatScan::new(),
                ecat_cmd: None,
                ecat_out: EcatOut::new(),
                ecat_master: EcatMasterCell(ecat_master),
            },
            Local {
                usb_configured: false,
                banner_done: false,
                banner_line: 0,
                scan_announced: false,
                led_a,
                led_b,
                ecat_line: heapless::String::new(),
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
        shared = [ecat_scan, ecat_cmd, ecat_out],
        local = [
            usb_configured,
            banner_done,
            banner_line,
            scan_announced,
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
                // 3a) One-time boot banner, emitted once per USB attach.
                if serial_try_banner_line(serial, *cx.local.banner_line) {
                    *cx.local.banner_line += 1;
                    if *cx.local.banner_line >= BOOT_BANNER_LINES {
                        *cx.local.banner_done = true;
                    }
                }
            } else if !*cx.local.scan_announced {
                // 3b) One-time scan summary, once the worker finishes the scan.
                let summary = cx.shared.ecat_scan.lock(|s| {
                    if !s.done {
                        None
                    } else {
                        Some((s.failed, s.slaves.len()))
                    }
                });
                if let Some((failed, count)) = summary {
                    let ok = if failed {
                        serial_try_write_line(serial, format_args!("[ecat] bus scan failed"))
                    } else {
                        serial_try_write_line(
                            serial,
                            format_args!("[ecat] scan complete: {} slave(s); type 'slaves'", count),
                        )
                    };
                    if ok {
                        *cx.local.scan_announced = true;
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
            // Host detached: re-arm the one-time banner/summary for next attach.
            *cx.local.banner_done = false;
            *cx.local.banner_line = 0;
            *cx.local.scan_announced = false;
            cx.local.ecat_line.clear();
        }

        if board::usb_bootloader::take_bootloader_request() {
            board::usb_bootloader::shutdown_and_enter(drive_bootloader_safe_outputs);
        }
        if board::usb_bootloader::take_reboot_request() {
            board::usb_bootloader::reboot();
        }
    }

    #[task(shared = [ecat_scan, ecat_cmd, ecat_out, ecat_master], priority = 1)]
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

        // Wait (bounded) for the PHY link before the initial bus scan.
        let mut linked = false;
        for _ in 0..50 {
            if cx.shared.ecat_master.lock(|m| m.0.link_up()) {
                linked = true;
                break;
            }
            Mono::delay(100_u32.millis()).await;
        }
        if !linked {
            log::warn!("[ecat] PHY link not up; attempting scan anyway");
        }

        // Initial bus scan (blocking; the PIT is not yet running, so holding the
        // master lock here only briefly delays USB).
        let scan = cx.shared.ecat_master.lock(|m| {
            let result = m.0.scan();
            let mut snapshot: heapless::Vec<SlaveInfo, EC_MAX_SLAVES> = heapless::Vec::new();
            if result.is_ok() {
                for slave in m.0.slaves() {
                    let _ = snapshot.push(*slave);
                }
            }
            (result, snapshot)
        });
        match scan {
            (Ok(n), snapshot) => {
                log::info!("[ecat] scan complete: {} slave(s)", n);
                cx.shared.ecat_scan.lock(|state| {
                    state.slaves = snapshot;
                    state.failed = false;
                    state.done = true;
                });
            }
            (Err(e), _) => {
                log::warn!("[ecat] bus scan failed: {:?}", e);
                cx.shared.ecat_scan.lock(|state| {
                    state.slaves.clear();
                    state.failed = true;
                    state.done = true;
                });
            }
        }

        // Nudge the USB reporter so the one-time scan summary flushes promptly.
        cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);

        // Command loop: take a parsed command, drive it (locking the master per
        // datagram, yielding between), publish the response, and wake USB.
        loop {
            let cmd = cx.shared.ecat_cmd.lock(|slot| slot.take());
            match cmd {
                Some(cmd) => {
                    let lines = ecat_run_command(&mut cx.shared.ecat_master, cmd).await;
                    cx.shared.ecat_out.lock(|o| o.set_lines(&lines));
                    cortex_m::peripheral::NVIC::pend(teensy4_bsp::Interrupt::USB_OTG1);
                }
                None => Mono::delay(ECAT_IDLE_MS.millis()).await,
            }
        }
    }

    /// High-priority cyclic process-data tick, fired by PIT channel 0. Owns the
    /// device for the duration of one short, non-blocking cycle (process the
    /// previous reply, send this cycle's LRW). Priority above `usb_isr` so the
    /// cycle is not delayed by USB or the worker.
    #[task(binds = PIT, shared = [ecat_master], priority = 3)]
    fn cyclic(mut cx: cyclic::Context) {
        board::cycle_timer::clear_interrupt();
        cx.shared.ecat_master.lock(|m| m.0.cyclic_tick());
    }
}
