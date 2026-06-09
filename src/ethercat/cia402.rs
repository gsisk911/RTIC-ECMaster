//! CiA 402 drive state machine.
//!
//! IgH: none. CiA 402 is NOT part of the IgH `master/` core (IgH only moves
//! process data; drive profiles live in the application). This is the
//! application/interface layer that walks each servo from Switch-On-Disabled ->
//! Ready-To-Switch-On -> Switched-On -> Operation-Enabled over the controlword
//! (0x6040) / statusword (0x6041), plus fault reset and quick-stop.
//!
//! The Teensy owns the controlword: the Pi/LinuxCNC host sends *intent* (enable,
//! fault-reset, quick-stop) over the SPI bridge and reads status; it never
//! hand-toggles the controlword through the buffered path. Driven once per
//! cyclic tick over the process image via `hal` pins. Drives are auto-discovered
//! from the configured `*-controlword` / `*-statusword` pin pairs.

use crate::ethercat::config::generated::BUS;
use crate::ethercat::config::model::PinCfg;
use crate::ethercat::globals::EC_MAX_SLAVES;
use crate::hal::pin::is_output;
use crate::hal::process_data as pdi;
use heapless::Vec;

// Statusword (0x6041) bits.
const SW_FAULT: u16 = 1 << 3;
/// State decode mask + values (low 7 bits, ignoring warning/remote/etc.).
const SW_STATE_MASK: u16 = 0x006F;
const SW_SWITCH_ON_DISABLED: u16 = 0x0040;
const SW_READY_TO_SWITCH_ON: u16 = 0x0021;
const SW_SWITCHED_ON: u16 = 0x0023;
const SW_OPERATION_ENABLED: u16 = 0x0027;

// Controlword (0x6040) command words.
const CW_SHUTDOWN: u16 = 0x0006; // -> Ready to switch on
const CW_SWITCH_ON: u16 = 0x0007; // -> Switched on
const CW_ENABLE_OP: u16 = 0x000F; // -> Operation enabled
const CW_QUICK_STOP: u16 = 0x0002; // -> Quick-stop active
const CW_FAULT_RESET: u16 = 0x0080; // bit7 rising edge clears a fault
const CW_DISABLE_VOLTAGE: u16 = 0x0000;

/// Host intent for one cyclic tick (from the SPI bridge flags + safe-state).
#[derive(Clone, Copy)]
pub struct DriveCommand {
    /// The host wants the drives enabled (Operation Enabled).
    pub enable: bool,
    /// Pulse a fault reset.
    pub fault_reset: bool,
    /// Force a controlled quick-stop (host request, watchdog, or underrun).
    pub quick_stop: bool,
}

/// One discovered drive's controlword/statusword binding.
struct Binding {
    cw: &'static PinCfg,
    sw: &'static PinCfg,
    /// Tracks the controlword bit7 level so fault reset produces a real edge.
    fr_high: bool,
}

/// Per-bus CiA 402 sequencer over all discovered drives.
pub struct Cia402 {
    drives: Vec<Binding, EC_MAX_SLAVES>,
}

impl Cia402 {
    /// Discover drives by pairing each `*-controlword` output pin with the
    /// `*-statusword` input pin that shares its prefix.
    pub fn new() -> Self {
        let mut drives = Vec::new();
        for cw in BUS.pins {
            if !is_output(cw) {
                continue;
            }
            let prefix = match cw.name.strip_suffix("-controlword") {
                Some(p) => p,
                None => continue,
            };
            for sw in BUS.pins {
                if is_output(sw) {
                    continue;
                }
                if let Some(p) = sw.name.strip_suffix("-statusword") {
                    if p == prefix {
                        let _ = drives.push(Binding {
                            cw,
                            sw,
                            fr_high: false,
                        });
                        break;
                    }
                }
            }
        }
        Self { drives }
    }

    /// Whether any drive was discovered (else this layer is inert).
    pub fn has_drives(&self) -> bool {
        !self.drives.is_empty()
    }

    /// Aggregate fault state across drives (any statusword fault bit set).
    pub fn any_fault(&self, image: &[u8]) -> bool {
        self.drives
            .iter()
            .any(|d| (pdi::read_value(image, d.sw) as u16) & SW_FAULT != 0)
    }

    /// Walk every drive one step: compute and write each controlword from its
    /// statusword + the host command.
    pub fn step(&mut self, image: &mut [u8], cmd: DriveCommand) {
        for d in self.drives.iter_mut() {
            let sw = pdi::read_value(image, d.sw) as u16;
            let cw = next_controlword(sw, cmd, &mut d.fr_high);
            pdi::write_value(image, d.cw, cw as i64);
        }
    }
}

impl Default for Cia402 {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the controlword to drive `sw` toward the commanded state. `fr_high`
/// is toggled to make fault reset (controlword bit7) a real rising edge.
fn next_controlword(sw: u16, cmd: DriveCommand, fr_high: &mut bool) -> u16 {
    let state = sw & SW_STATE_MASK;

    if sw & SW_FAULT != 0 {
        // Faulted: pulse fault-reset if requested, else hold voltage off.
        if cmd.fault_reset {
            *fr_high = !*fr_high;
            return if *fr_high {
                CW_FAULT_RESET
            } else {
                CW_DISABLE_VOLTAGE
            };
        }
        *fr_high = false;
        return CW_DISABLE_VOLTAGE;
    }
    *fr_high = false;

    // A quick-stop is only meaningful once the drive is at least switched on.
    if cmd.quick_stop && state == SW_OPERATION_ENABLED {
        return CW_QUICK_STOP;
    }

    if state == SW_SWITCH_ON_DISABLED {
        return CW_SHUTDOWN;
    }
    if state == SW_READY_TO_SWITCH_ON {
        return if cmd.enable && !cmd.quick_stop {
            CW_SWITCH_ON
        } else {
            CW_SHUTDOWN
        };
    }
    if state == SW_SWITCHED_ON {
        return if cmd.enable && !cmd.quick_stop {
            CW_ENABLE_OP
        } else {
            CW_SWITCH_ON
        };
    }
    if state == SW_OPERATION_ENABLED {
        return if cmd.enable && !cmd.quick_stop {
            CW_ENABLE_OP
        } else {
            CW_SWITCH_ON // disable operation, stay switched on
        };
    }
    CW_SHUTDOWN
}
