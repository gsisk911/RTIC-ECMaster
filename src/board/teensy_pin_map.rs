//! Teensy digital pin to fast GPIO mapping.
//!
//! The `teensy4-pins` crate names pads by their reference-manual GPIO block
//! (GPIO1..GPIO4). Teensy's fast GPIO registers expose those same bits through
//! GPIO6..GPIO9, which is what `FastGpioOutput` writes.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FastGpio {
    pub group: u8,
    pub bit: u8,
}

impl FastGpio {
    pub const fn new(group: u8, bit: u8) -> Self {
        Self { group, bit }
    }

    pub const fn mask(self) -> u32 {
        1u32 << self.bit
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)]
pub enum TeensyBoard {
    Teensy40,
    Teensy41,
}

pub const fn teensy_pin_to_fast_gpio(board: TeensyBoard, pin: u8) -> Option<FastGpio> {
    match board {
        TeensyBoard::Teensy40 => lookup_pin(&TEENSY_40_PINS, pin),
        TeensyBoard::Teensy41 => lookup_pin(&TEENSY_41_PINS, pin),
    }
}

#[allow(dead_code)]
pub const fn fast_gpio_to_teensy_pin(board: TeensyBoard, gpio: FastGpio) -> Option<u8> {
    match board {
        TeensyBoard::Teensy40 => lookup_gpio(&TEENSY_40_PINS, gpio),
        TeensyBoard::Teensy41 => lookup_gpio(&TEENSY_41_PINS, gpio),
    }
}

const fn lookup_pin(pins: &[FastGpio], pin: u8) -> Option<FastGpio> {
    let index = pin as usize;
    if index < pins.len() {
        Some(pins[index])
    } else {
        None
    }
}

#[allow(dead_code)]
const fn lookup_gpio(pins: &[FastGpio], gpio: FastGpio) -> Option<u8> {
    let mut index = 0usize;
    while index < pins.len() {
        let candidate = pins[index];
        if candidate.group == gpio.group && candidate.bit == gpio.bit {
            return Some(index as u8);
        }
        index += 1;
    }
    None
}

// Common pins 0-33 match Teensy 4.0 and 4.1. The values follow the
// `teensy4-pins` ALT5 GPIO block/offset table with GPIO1..GPIO4 shifted to
// Teensy's fast GPIO6..GPIO9 registers.
const COMMON_PINS: [FastGpio; 34] = [
    FastGpio::new(6, 3),  // 0
    FastGpio::new(6, 2),  // 1
    FastGpio::new(9, 4),  // 2
    FastGpio::new(9, 5),  // 3
    FastGpio::new(9, 6),  // 4
    FastGpio::new(9, 8),  // 5
    FastGpio::new(7, 10), // 6
    FastGpio::new(7, 17), // 7
    FastGpio::new(7, 16), // 8
    FastGpio::new(7, 11), // 9
    FastGpio::new(7, 0),  // 10
    FastGpio::new(7, 2),  // 11
    FastGpio::new(7, 1),  // 12
    FastGpio::new(7, 3),  // 13
    FastGpio::new(6, 18), // 14
    FastGpio::new(6, 19), // 15
    FastGpio::new(6, 23), // 16
    FastGpio::new(6, 22), // 17
    FastGpio::new(6, 17), // 18
    FastGpio::new(6, 16), // 19
    FastGpio::new(6, 26), // 20
    FastGpio::new(6, 27), // 21
    FastGpio::new(6, 24), // 22
    FastGpio::new(6, 25), // 23
    FastGpio::new(6, 12), // 24
    FastGpio::new(6, 13), // 25
    FastGpio::new(6, 30), // 26
    FastGpio::new(6, 31), // 27
    FastGpio::new(8, 18), // 28
    FastGpio::new(9, 31), // 29
    FastGpio::new(8, 23), // 30
    FastGpio::new(8, 22), // 31
    FastGpio::new(7, 12), // 32
    FastGpio::new(9, 7),  // 33
];

const TEENSY_40_PINS: [FastGpio; 40] = [
    COMMON_PINS[0],
    COMMON_PINS[1],
    COMMON_PINS[2],
    COMMON_PINS[3],
    COMMON_PINS[4],
    COMMON_PINS[5],
    COMMON_PINS[6],
    COMMON_PINS[7],
    COMMON_PINS[8],
    COMMON_PINS[9],
    COMMON_PINS[10],
    COMMON_PINS[11],
    COMMON_PINS[12],
    COMMON_PINS[13],
    COMMON_PINS[14],
    COMMON_PINS[15],
    COMMON_PINS[16],
    COMMON_PINS[17],
    COMMON_PINS[18],
    COMMON_PINS[19],
    COMMON_PINS[20],
    COMMON_PINS[21],
    COMMON_PINS[22],
    COMMON_PINS[23],
    COMMON_PINS[24],
    COMMON_PINS[25],
    COMMON_PINS[26],
    COMMON_PINS[27],
    COMMON_PINS[28],
    COMMON_PINS[29],
    COMMON_PINS[30],
    COMMON_PINS[31],
    COMMON_PINS[32],
    COMMON_PINS[33],
    FastGpio::new(8, 15), // 34
    FastGpio::new(8, 14), // 35
    FastGpio::new(8, 13), // 36
    FastGpio::new(8, 12), // 37
    FastGpio::new(8, 17), // 38
    FastGpio::new(8, 16), // 39
];

const TEENSY_41_PINS: [FastGpio; 55] = [
    COMMON_PINS[0],
    COMMON_PINS[1],
    COMMON_PINS[2],
    COMMON_PINS[3],
    COMMON_PINS[4],
    COMMON_PINS[5],
    COMMON_PINS[6],
    COMMON_PINS[7],
    COMMON_PINS[8],
    COMMON_PINS[9],
    COMMON_PINS[10],
    COMMON_PINS[11],
    COMMON_PINS[12],
    COMMON_PINS[13],
    COMMON_PINS[14],
    COMMON_PINS[15],
    COMMON_PINS[16],
    COMMON_PINS[17],
    COMMON_PINS[18],
    COMMON_PINS[19],
    COMMON_PINS[20],
    COMMON_PINS[21],
    COMMON_PINS[22],
    COMMON_PINS[23],
    COMMON_PINS[24],
    COMMON_PINS[25],
    COMMON_PINS[26],
    COMMON_PINS[27],
    COMMON_PINS[28],
    COMMON_PINS[29],
    COMMON_PINS[30],
    COMMON_PINS[31],
    COMMON_PINS[32],
    COMMON_PINS[33],
    FastGpio::new(7, 29), // 34
    FastGpio::new(7, 28), // 35
    FastGpio::new(7, 18), // 36
    FastGpio::new(7, 19), // 37
    FastGpio::new(6, 28), // 38
    FastGpio::new(6, 29), // 39
    FastGpio::new(6, 20), // 40
    FastGpio::new(6, 21), // 41
    FastGpio::new(8, 15), // 42
    FastGpio::new(8, 14), // 43
    FastGpio::new(8, 13), // 44
    FastGpio::new(8, 12), // 45
    FastGpio::new(8, 17), // 46
    FastGpio::new(8, 16), // 47
    FastGpio::new(9, 24), // 48
    FastGpio::new(9, 27), // 49
    FastGpio::new(9, 28), // 50
    FastGpio::new(9, 22), // 51
    FastGpio::new(9, 26), // 52
    FastGpio::new(9, 25), // 53
    FastGpio::new(9, 29), // 54
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_current_fast_gpio_outputs_to_teensy_4x_pins() {
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy40, FastGpio::new(6, 2)),
            Some(1)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy40, FastGpio::new(9, 4)),
            Some(2)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy40, FastGpio::new(9, 5)),
            Some(3)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy40, FastGpio::new(9, 6)),
            Some(4)
        );

        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(6, 2)),
            Some(1)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(9, 4)),
            Some(2)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(9, 5)),
            Some(3)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(9, 6)),
            Some(4)
        );
    }

    #[test]
    fn converts_teensy_4x_pins_back_to_fast_gpio_outputs() {
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 1),
            Some(FastGpio::new(6, 2))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 2),
            Some(FastGpio::new(9, 4))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 3),
            Some(FastGpio::new(9, 5))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 4),
            Some(FastGpio::new(9, 6))
        );

        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy41, 1),
            Some(FastGpio::new(6, 2))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy41, 2),
            Some(FastGpio::new(9, 4))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy41, 3),
            Some(FastGpio::new(9, 5))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy41, 4),
            Some(FastGpio::new(9, 6))
        );
    }

    #[test]
    fn maps_common_high_ad_b1_pins_to_their_fast_gpio_offsets() {
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 26),
            Some(FastGpio::new(6, 30))
        );
        assert_eq!(
            teensy_pin_to_fast_gpio(TeensyBoard::Teensy40, 27),
            Some(FastGpio::new(6, 31))
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(6, 30)),
            Some(26)
        );
        assert_eq!(
            fast_gpio_to_teensy_pin(TeensyBoard::Teensy41, FastGpio::new(6, 31)),
            Some(27)
        );
    }
}
