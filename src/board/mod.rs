//! Core Teensy 4.1 platform support.
//!
//! Low-level board bring-up that is independent of the application protocol:
//! clock tree configuration, fast GPIO output, the Teensy pin-number map, and
//! the USB CDC soft-bootloader monitor.

pub mod clock_config;
pub mod cycle_timer;
pub mod fast_gpio;
pub mod teensy_pin_map;
pub mod usb_bootloader;
