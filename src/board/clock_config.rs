use imxrt_hal::{
    ccm::{self, clock_gate},
    dcdc,
};
use imxrt_ral as ral;

use crate::board::fast_gpio::FastGpioOutput;

const OVERCLOCK_STEP_HZ: u32 = 28_000_000;
const OVERCLOCK_MAX_MV: u32 = 1_575;
const INDICATOR_FLASH_ON_MS: u32 = 100;
const INDICATOR_FLASH_OFF_MS: u32 = 500;

pub const REQUESTED_CORE_CLOCK_HZ: u32 = parse_u32(env!("TEENSY4_CORE_CLOCK_HZ"));
pub const ALLOW_OVERCLOCK: bool = parse_bool(env!("TEENSY4_ALLOW_OVERCLOCK"));
pub const ALLOW_MAX_VOLTAGE: bool = parse_bool(env!("TEENSY4_ALLOW_MAX_VOLTAGE"));
pub const PROFILE: ClockProfile = ClockProfile::new(REQUESTED_CORE_CLOCK_HZ);
pub const CORE_CLOCK_HZ: u32 = PROFILE.core_hz;
pub const INDICATOR_FLASH_COUNT: u32 = CORE_CLOCK_HZ / 100_000_000;

#[derive(Clone, Copy)]
pub struct ClockProfile {
    pub requested_hz: u32,
    pub core_hz: u32,
    pub ipg_hz: u32,
    pub vdd_soc_mv: u32,
    pll1_div_select: u32,
    arm_divider: u32,
    ahb_divider: u32,
    ipg_divider: u32,
}

impl ClockProfile {
    pub const fn new(requested_hz: u32) -> Self {
        assert_supported_clock(requested_hz);

        let mut arm_divider = 1;
        let mut ahb_divider = 1;

        while requested_hz as u64 * arm_divider as u64 * (ahb_divider as u64) < 648_000_000 {
            if arm_divider < 8 {
                arm_divider += 1;
            } else if ahb_divider < 5 {
                ahb_divider += 1;
                arm_divider = 1;
            } else {
                break;
            }
        }

        let scaled_hz = requested_hz as u64 * arm_divider as u64 * ahb_divider as u64;
        let mut pll1_div_select = ((scaled_hz + 6_000_000) / 12_000_000) as u32;

        if pll1_div_select > 108 {
            pll1_div_select = 108;
        }
        if pll1_div_select < 54 {
            pll1_div_select = 54;
        }

        let core_hz = 12_000_000 * pll1_div_select / arm_divider / ahb_divider;
        if core_hz > 600_000_000 && !ALLOW_OVERCLOCK {
            panic!("TEENSY4_ALLOW_OVERCLOCK must be true above 600 MHz");
        }

        let required_vdd_soc_mv = required_voltage_for_hz(core_hz);
        if required_vdd_soc_mv > OVERCLOCK_MAX_MV && !ALLOW_MAX_VOLTAGE {
            panic!("TEENSY4_ALLOW_MAX_VOLTAGE must be true for this profile");
        }

        let mut ipg_divider = core_hz.div_ceil(150_000_000);
        if ipg_divider > 4 {
            ipg_divider = 4;
        }

        Self {
            requested_hz,
            core_hz,
            ipg_hz: core_hz / ipg_divider,
            vdd_soc_mv: clamp_voltage(required_vdd_soc_mv),
            pll1_div_select,
            arm_divider,
            ahb_divider,
            ipg_divider,
        }
    }
}

pub fn apply(
    ccm: &mut ral::ccm::CCM,
    ccm_analog: &mut ral::ccm_analog::CCM_ANALOG,
    dcdc: &mut ral::dcdc::DCDC,
) {
    let profile = PROFILE;
    let old_vdd_soc_mv = dcdc::target_vdd_soc(dcdc);

    if old_vdd_soc_mv < profile.vdd_soc_mv {
        dcdc::set_target_vdd_soc(dcdc, profile.vdd_soc_mv);
    }

    configure_arm_clock(ccm, ccm_analog, profile);

    if old_vdd_soc_mv > profile.vdd_soc_mv {
        dcdc::set_target_vdd_soc(dcdc, profile.vdd_soc_mv);
    }
}

pub fn prepare(
    ccm: &mut ral::ccm::CCM,
    ccm_analog: &mut ral::ccm_analog::CCM_ANALOG,
    dcdc: &mut ral::dcdc::DCDC,
) {
    teensy4_bsp::board::prepare_clocks_and_power(ccm, ccm_analog, dcdc);
    apply(ccm, ccm_analog, dcdc);
}

pub fn flash_indicator(indicator: &mut FastGpioOutput) {
    let mut flashes = 0;

    while flashes < INDICATOR_FLASH_COUNT {
        indicator.set();
        delay_ms(INDICATOR_FLASH_ON_MS);
        indicator.clear();
        flashes += 1;

        let off_time_ms = if flashes == INDICATOR_FLASH_COUNT {
            INDICATOR_FLASH_OFF_MS * 2
        } else {
            INDICATOR_FLASH_OFF_MS
        };
        delay_ms(off_time_ms);
    }
}

fn configure_arm_clock(
    ccm: &mut ral::ccm::CCM,
    ccm_analog: &mut ral::ccm_analog::CCM_ANALOG,
    profile: ClockProfile,
) {
    clock_gate::IPG_CLOCK_GATES
        .iter()
        .for_each(|locator| locator.set(ccm, clock_gate::OFF));

    if ccm::ahb_clk::selection(ccm) == ccm::ahb_clk::Selection::PeriphClk2Sel {
        ccm::ahb_clk::set_selection(ccm, ccm::ahb_clk::Selection::PrePeriphClkSel);
    }

    ccm::periph_clk2::set_divider(ccm, 1);
    ccm::periph_clk2::set_selection(ccm, ccm::periph_clk2::Selection::Osc);
    ccm::ahb_clk::set_selection(ccm, ccm::ahb_clk::Selection::PeriphClk2Sel);

    ccm::analog::pll1::restart(ccm_analog, profile.pll1_div_select);
    ccm::arm_divider::set_divider(ccm, profile.arm_divider);
    ccm::ahb_clk::set_divider(ccm, profile.ahb_divider);

    ccm::pre_periph_clk::set_selection(ccm, ccm::pre_periph_clk::Selection::Pll1);
    ccm::ahb_clk::set_selection(ccm, ccm::ahb_clk::Selection::PrePeriphClkSel);
    ccm::ipg_clk::set_divider(ccm, profile.ipg_divider);

    clock_gate::IPG_CLOCK_GATES
        .iter()
        .for_each(|locator| locator.set(ccm, clock_gate::ON));
}

fn delay_ms(ms: u32) {
    cortex_m::asm::delay((CORE_CLOCK_HZ / 1_000).saturating_mul(ms));
}

const fn assert_supported_clock(hz: u32) {
    match hz {
        24_000_000 | 150_000_000 | 396_000_000 | 450_000_000 | 528_000_000
        | 600_000_000 | 720_000_000 | 816_000_000 | 912_000_000 | 960_000_000
        | 1_008_000_000 => {}
        _ => panic!("unsupported TEENSY4_CORE_CLOCK_HZ profile"),
    }
}

const fn required_voltage_for_hz(hz: u32) -> u32 {
    if hz <= 240_000_000 {
        950 + (hz / 32_000_000) * 25
    } else if hz <= 456_000_000 {
        1_150
    } else if hz <= 600_000_000 {
        1_150 + ((hz - 456_000_000) / 36_000_000) * 25
    } else {
        1_250 + ((hz - 600_000_000) / OVERCLOCK_STEP_HZ) * 25
    }
}

const fn clamp_voltage(mv: u32) -> u32 {
    if mv > OVERCLOCK_MAX_MV {
        OVERCLOCK_MAX_MV
    } else {
        mv
    }
}

const fn parse_u32(value: &str) -> u32 {
    let bytes = value.as_bytes();
    let mut index = 0;
    let mut parsed = 0u32;

    if bytes.is_empty() {
        panic!("TEENSY4_CORE_CLOCK_HZ must not be empty");
    }

    while index < bytes.len() {
        let byte = bytes[index];
        if byte < b'0' || byte > b'9' {
            panic!("TEENSY4_CORE_CLOCK_HZ must be numeric");
        }
        parsed = parsed * 10 + (byte - b'0') as u32;
        index += 1;
    }

    parsed
}

const fn parse_bool(value: &str) -> bool {
    let bytes = value.as_bytes();

    if bytes.len() == 4
        && bytes[0] == b't'
        && bytes[1] == b'r'
        && bytes[2] == b'u'
        && bytes[3] == b'e'
    {
        true
    } else if bytes.len() == 5
        && bytes[0] == b'f'
        && bytes[1] == b'a'
        && bytes[2] == b'l'
        && bytes[3] == b's'
        && bytes[4] == b'e'
    {
        false
    } else {
        panic!("boolean config values must be true or false");
    }
}
