//! Pass-through USB CDC bootloader / reboot request monitor.
//!
//! This module does **not** own USB. It does not create a `UsbDevice`, allocate
//! endpoints, set a VID/PID, or replace your project's USB stack. It only
//! provides a tiny `UsbClass` watcher that must be placed first in the existing
//! `UsbDevice::poll()` class chain.
//!
//! ## How to use
//!
//! 1. Add `mod usb_bootloader;` in `main.rs`.
//! 2. Store a `usb_bootloader::Monitor<YourUsbBus>` beside your existing USB
//!    classes.
//!
//! ```ignore
//! let monitor = usb_bootloader::Monitor::new();
//! ```
//!
//! 3. In your USB interrupt/task, put the monitor first in the poll chain. It
//!    will inspect CDC `SET_LINE_CODING` requests, latch matching actions, and
//!    return without accepting or rejecting the transfer. The real CDC class
//!    still handles the request normally.
//!
//! ```ignore
//! let _ = usb_device.poll(&mut [
//!     &mut monitor,
//!     &mut serial,
//!     &mut other_usb_class,
//! ]);
//! ```
//!
//! 4. After `poll()` returns, consume any latched action and perform the
//!    no-return operation outside the USB class callback:
//!
//! ```ignore
//! if usb_bootloader::take_bootloader_request() {
//!     usb_bootloader::shutdown_and_enter(drive_safe_outputs);
//! }
//!
//! if usb_bootloader::take_reboot_request() {
//!     usb_bootloader::reboot();
//! }
//! ```
//!
//! Host behavior:
//! - CDC `SET_LINE_CODING` to 134 baud, 8N1 requests Teensy bootloader mode.
//! - CDC `SET_LINE_CODING` to 135 baud, 8N1 requests a normal CPU reboot.
//!
//! Keep exactly one USB owner in your project. This monitor is only the first
//! `UsbClass` in that owner's poll chain.
//!
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

use usb_device::{
    bus::UsbBus,
    class_prelude::{ControlOut, UsbClass},
    control::{Recipient, Request, RequestType},
    UsbDirection,
};

const CDC_LINE_CODING_LEN: usize = 7;
const TEENSY_BOOTLOADER_BAUD: u32 = 134;
const TEENSY_NORMAL_REBOOT_BAUD: u32 = 135;
const CDC_STOP_BITS_ONE: u8 = 0;
const CDC_PARITY_NONE: u8 = 0;
const CDC_DATA_BITS_8: u8 = 8;
const CDC_SET_LINE_CODING: u8 = 0x20;
const CDC_COMM_INTERFACE_INDEX: u16 = 0;

static BOOTLOADER_REQUESTED: AtomicBool = AtomicBool::new(false);
static REBOOT_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Pass-through CDC control request monitor.
///
/// Put this before the real CDC class in `UsbDevice::poll()`.
pub struct Monitor<B: UsbBus> {
    _bus: PhantomData<B>,
}

impl<B: UsbBus> Monitor<B> {
    pub const fn new() -> Self {
        Self { _bus: PhantomData }
    }
}

impl<B: UsbBus> UsbClass<B> for Monitor<B> {
    fn control_out(&mut self, xfer: ControlOut<B>) {
        if is_soft_reboot_control_request(xfer.request(), xfer.data()) {
            request_bootloader();
        } else if is_normal_reboot_control_request(xfer.request(), xfer.data()) {
            request_reboot();
        }
    }
}

/// Returns true when a CDC line-coding payload is the PJRC bootloader request.
pub fn is_teensy_soft_reboot_line_coding(data: &[u8]) -> bool {
    matches_line_coding(data, TEENSY_BOOTLOADER_BAUD)
}

/// Returns true when a CDC line-coding payload is the app-level normal reboot request.
pub fn is_teensy_normal_reboot_line_coding(data: &[u8]) -> bool {
    matches_line_coding(data, TEENSY_NORMAL_REBOOT_BAUD)
}

fn matches_line_coding(data: &[u8], expected_baud: u32) -> bool {
    if data.len() != CDC_LINE_CODING_LEN {
        return false;
    }

    let baud = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    baud == expected_baud
        && data[4] == CDC_STOP_BITS_ONE
        && data[5] == CDC_PARITY_NONE
        && data[6] == CDC_DATA_BITS_8
}

/// Latch a bootloader request from interrupt/control-transfer context.
#[inline]
pub fn request_bootloader() {
    BOOTLOADER_REQUESTED.store(true, Ordering::SeqCst);
}

/// Clear and return the latched bootloader request.
#[inline]
pub fn take_bootloader_request() -> bool {
    BOOTLOADER_REQUESTED.swap(false, Ordering::SeqCst)
}

/// Latch a normal reboot request from interrupt/control-transfer context.
#[inline]
pub fn request_reboot() {
    REBOOT_REQUESTED.store(true, Ordering::SeqCst);
}

/// Clear and return the latched normal reboot request.
#[inline]
pub fn take_reboot_request() -> bool {
    REBOOT_REQUESTED.swap(false, Ordering::SeqCst)
}

/// Run app-specific emergency shutdown work and enter the Teensy bootloader.
pub fn shutdown_and_enter(shutdown: impl FnOnce()) -> ! {
    shutdown();
    enter_teensy_bootloader()
}

/// Perform a normal CPU reset. This does not enter the Teensy bootloader.
pub fn reboot() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Enter the Teensy 4.x bootloader. This function never returns.
pub fn enter_teensy_bootloader() -> ! {
    cortex_m::interrupt::disable();

    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("bkpt #251", options(noreturn));
    }

    #[cfg(not(target_arch = "arm"))]
    loop {
        core::hint::spin_loop();
    }
}

fn is_soft_reboot_control_request(request: &Request, data: &[u8]) -> bool {
    request.direction == UsbDirection::Out
        && request.request_type == RequestType::Class
        && request.recipient == Recipient::Interface
        && request.request == CDC_SET_LINE_CODING
        && request.index == CDC_COMM_INTERFACE_INDEX
        && is_teensy_soft_reboot_line_coding(data)
}

fn is_normal_reboot_control_request(request: &Request, data: &[u8]) -> bool {
    request.direction == UsbDirection::Out
        && request.request_type == RequestType::Class
        && request.recipient == Recipient::Interface
        && request.request == CDC_SET_LINE_CODING
        && request.index == CDC_COMM_INTERFACE_INDEX
        && is_teensy_normal_reboot_line_coding(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOOTLOADER_LINE_CODING: [u8; 7] = [0x86, 0, 0, 0, 0, 0, 8];
    const REBOOT_LINE_CODING: [u8; 7] = [0x87, 0, 0, 0, 0, 0, 8];

    fn clear_requests() {
        BOOTLOADER_REQUESTED.store(false, Ordering::SeqCst);
        REBOOT_REQUESTED.store(false, Ordering::SeqCst);
    }

    fn set_line_coding_request() -> Request {
        Request {
            direction: UsbDirection::Out,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: CDC_SET_LINE_CODING,
            value: 0,
            index: 0,
            length: 7,
        }
    }

    #[test]
    fn detects_soft_reboot_control_request() {
        let request = set_line_coding_request();

        assert!(is_soft_reboot_control_request(&request, &BOOTLOADER_LINE_CODING));
        assert!(!is_soft_reboot_control_request(&request, &REBOOT_LINE_CODING));
    }

    #[test]
    fn detects_normal_reboot_control_request() {
        let request = set_line_coding_request();

        assert!(is_normal_reboot_control_request(&request, &REBOOT_LINE_CODING));
        assert!(!is_normal_reboot_control_request(
            &request,
            &BOOTLOADER_LINE_CODING
        ));
    }

    #[test]
    fn ignores_non_magic_control_request() {
        let mut request = set_line_coding_request();
        request.request = 0x22;

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x86, 0, 0, 0, 0, 0, 8],
        ));
        assert!(!is_normal_reboot_control_request(
            &request,
            &[0x87, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn ignores_non_cdc_interface_request() {
        let mut request = set_line_coding_request();
        request.index = 1;

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x86, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn ignores_non_out_request() {
        let mut request = set_line_coding_request();
        request.direction = UsbDirection::In;

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x86, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn ignores_non_class_request() {
        let mut request = set_line_coding_request();
        request.request_type = RequestType::Vendor;

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x86, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn ignores_non_interface_request() {
        let mut request = set_line_coding_request();
        request.recipient = Recipient::Device;

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x86, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn ignores_non_magic_line_coding() {
        let request = set_line_coding_request();

        assert!(!is_soft_reboot_control_request(
            &request,
            &[0x00, 0, 0, 0, 0, 0, 8],
        ));
    }

    #[test]
    fn latches_bootloader_and_reboot_requests() {
        clear_requests();

        request_bootloader();
        request_reboot();

        assert!(take_bootloader_request());
        assert!(take_reboot_request());
        assert!(!take_bootloader_request());
        assert!(!take_reboot_request());
    }
}
