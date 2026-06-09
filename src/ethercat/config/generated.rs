//! GENERATED EtherCAT bus configuration -- do not edit by hand.
//!
//! Produced by `scripts/generate_ethercat_config.py` (run `make config`). Holds one
//! `pub const BUS: BusCfg` consumed by the bring-up FSM, the process-data domain,
//! and the HAL pin layer. Regenerate and commit; never hand-edit.


use super::model::{BusCfg, DcCfg, FmmuCfg, HalType, PdoCfg, PinCfg, SdoInit, SlaveCfg, SmCfg};

use crate::ethercat::ecrt::{EcDirection, EcPdoEntryInfo};


const fn e(index: u16, subindex: u8, bit_length: u8) -> EcPdoEntryInfo {
    EcPdoEntryInfo { index, subindex, bit_length }
}


// --- Slave 0: product 0x00001B00 ---

const S0_RX_P0_ENTRIES: &[EcPdoEntryInfo] = &[
    e(0x6040, 0x00, 16),
    e(0x607A, 0x00, 32),
    e(0x60FF, 0x00, 32),
    e(0x60B8, 0x00, 16),
    e(0x60FE, 0x01, 32),
];

const S0_RX_PDOS: &[PdoCfg] = &[
    PdoCfg { index: 0x1600, entries: S0_RX_P0_ENTRIES },
];

const S0_TX_P0_ENTRIES: &[EcPdoEntryInfo] = &[
    e(0x603F, 0x00, 16),
    e(0x6041, 0x00, 16),
    e(0x6064, 0x00, 32),
    e(0x60FD, 0x00, 32),
    e(0x606C, 0x00, 32),
    e(0x60F4, 0x00, 32),
    e(0x60B9, 0x00, 16),
    e(0x60BA, 0x00, 32),
    e(0x60BB, 0x00, 32),
    e(0x60BC, 0x00, 32),
    e(0x60BD, 0x00, 32),
    e(0x6061, 0x00, 8),
];

const S0_TX_PDOS: &[PdoCfg] = &[
    PdoCfg { index: 0x1A00, entries: S0_TX_P0_ENTRIES },
];

const S0_SMS: &[SmCfg] = &[
    SmCfg { index: 2, phys_start: 0x1200, control: 0x64, dir: EcDirection::Output, size: 16, pdos: S0_RX_PDOS },
    SmCfg { index: 3, phys_start: 0x1300, control: 0x20, dir: EcDirection::Input, size: 39, pdos: S0_TX_PDOS },
];

const S0_FMMUS: &[FmmuCfg] = &[
    FmmuCfg { logical_start: 0, size: 16, phys_start: 0x1200, dir: EcDirection::Output },
    FmmuCfg { logical_start: 16, size: 39, phys_start: 0x1300, dir: EcDirection::Input },
];

const S0_SDO_INIT: &[SdoInit] = &[
    SdoInit { index: 0x6060, subindex: 0x00, data: &[0x08] },
];

const SLAVES: &[SlaveCfg] = &[
    SlaveCfg { position: 0, vendor_id: 0x00000994, product_code: 0x00001B00, sms: S0_SMS, fmmus: S0_FMMUS, dc: Some(DcCfg { assign_activate: 0x0300, sync0_cycle_ns: 10000000, sync0_shift_ns: 0, sync1_cycle_ns: 0 }), sdo_init: S0_SDO_INIT, out_size: 16, in_size: 39 },
];

const PINS: &[PinCfg] = &[
    PinCfg { name: "drive0-controlword", byte_offset: 0, bit_pos: 0, bit_len: 16, hal_type: HalType::U32, dir: EcDirection::Output },
    PinCfg { name: "drive0-target-position", byte_offset: 2, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Output },
    PinCfg { name: "drive0-target-velocity", byte_offset: 6, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Output },
    PinCfg { name: "drive0-touch-probe-function", byte_offset: 10, bit_pos: 0, bit_len: 16, hal_type: HalType::U32, dir: EcDirection::Output },
    PinCfg { name: "drive0-digital-outputs", byte_offset: 12, bit_pos: 0, bit_len: 32, hal_type: HalType::U32, dir: EcDirection::Output },
    PinCfg { name: "drive0-error-code", byte_offset: 16, bit_pos: 0, bit_len: 16, hal_type: HalType::U32, dir: EcDirection::Input },
    PinCfg { name: "drive0-statusword", byte_offset: 18, bit_pos: 0, bit_len: 16, hal_type: HalType::U32, dir: EcDirection::Input },
    PinCfg { name: "drive0-actual-position", byte_offset: 20, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-digital-inputs", byte_offset: 24, bit_pos: 0, bit_len: 32, hal_type: HalType::U32, dir: EcDirection::Input },
    PinCfg { name: "drive0-actual-velocity", byte_offset: 28, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-follow-error", byte_offset: 32, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-touch-probe-status", byte_offset: 36, bit_pos: 0, bit_len: 16, hal_type: HalType::U32, dir: EcDirection::Input },
    PinCfg { name: "drive0-touch-probe-pos1-positive", byte_offset: 38, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-touch-probe-pos1-negative", byte_offset: 42, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-touch-probe-pos2-positive", byte_offset: 46, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-touch-probe-pos2-negative", byte_offset: 50, bit_pos: 0, bit_len: 32, hal_type: HalType::S32, dir: EcDirection::Input },
    PinCfg { name: "drive0-op-mode-display", byte_offset: 54, bit_pos: 0, bit_len: 8, hal_type: HalType::S32, dir: EcDirection::Input },
];

pub const BUS: BusCfg = BusCfg {
    cycle_ns: 10000000,
    ref_clock_slave: 0,
    slaves: SLAVES,
    pins: PINS,
    image_size: 55,
};

