//! Direct fast GPIO output writes.

use crate::board::teensy_pin_map::FastGpio;

/// A single fast GPIO output.
pub struct FastGpioOutput {
    gpio: FastGpio,
    level: bool,
}

impl FastGpioOutput {
    /// Create a fast GPIO output from the board pin map.
    pub const fn new(gpio: FastGpio) -> Self {
        Self { gpio, level: false }
    }

    /// Enable fast routing, set direction to output, and drive low.
    ///
    /// # Safety
    /// Accesses raw GPIO and IOMUXC_GPR registers. Must only be called once per
    /// pin after the IOMUXC mux has routed the pad to this GPIO.
    pub unsafe fn init(&self) {
        enable_fast_gpio(self.gpio);

        let base = gpio_base(self.gpio.group);
        let mask = self.gpio.mask();
        let gdir = core::ptr::read_volatile((base + GPIO_GDIR) as *const u32);

        core::ptr::write_volatile((base + GPIO_GDIR) as *mut u32, gdir | mask);
        core::ptr::write_volatile((base + GPIO_DR_CLEAR) as *mut u32, mask);
    }

    /// Drive the output high.
    #[inline]
    pub fn set(&mut self) {
        self.level = true;
        self.write_raw(GPIO_DR_SET);
    }

    /// Drive the output low.
    #[inline]
    pub fn clear(&mut self) {
        self.level = false;
        self.write_raw(GPIO_DR_CLEAR);
    }

    /// Toggle the output.
    #[inline]
    pub fn toggle(&mut self) {
        self.write(!self.level);
    }

    /// Drive the output to `level`.
    #[inline]
    pub fn write(&mut self, level: bool) {
        if level {
            self.set();
        } else {
            self.clear();
        }
    }

    #[inline]
    fn write_raw(&self, register_offset: usize) {
        unsafe {
            core::ptr::write_volatile(
                (gpio_base(self.gpio.group) + register_offset) as *mut u32,
                self.gpio.mask(),
            );
        }
    }
}

const IOMUXC_GPR_BASE: usize = 0x400A_C000;
const GPIO_GDIR: usize = 0x04;
const GPIO_DR_SET: usize = 0x84;
const GPIO_DR_CLEAR: usize = 0x88;

unsafe fn enable_fast_gpio(gpio: FastGpio) {
    let gpr_offset = match gpio.group {
        6 => Some(0x68), // GPR26: GPIO1 → GPIO6
        7 => Some(0x6C), // GPR27: GPIO2 → GPIO7
        8 => Some(0x70), // GPR28: GPIO3 → GPIO8
        9 => Some(0x74), // GPR29: GPIO4 → GPIO9
        _ => None,
    };
    if let Some(off) = gpr_offset {
        let gpr = (IOMUXC_GPR_BASE + off) as *mut u32;
        let val = core::ptr::read_volatile(gpr);
        core::ptr::write_volatile(gpr, val | gpio.mask());
    }
}

fn gpio_base(group: u8) -> usize {
    match group {
        1 => 0x401B_8000,
        2 => 0x401B_C000,
        3 => 0x401C_0000,
        4 => 0x401C_4000,
        5 => 0x400C_0000,
        // Fast GPIO aliases (GPIO6-9 mirror GPIO1-4 at different addresses)
        6 => 0x4200_0000,
        7 => 0x4200_4000,
        8 => 0x4200_8000,
        9 => 0x4200_C000,
        _ => panic!("invalid GPIO group"),
    }
}
