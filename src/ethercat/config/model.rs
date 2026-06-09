//! Configuration data model mirroring `ethercat-conf.xml`.
//!
//! Defines the in-memory representation of the desired network as fixed
//! `'static` const tables. The values are produced at COMPILE TIME by
//! `scripts/generate_ethercat_config.py` (which merges the lcec-style bus XML
//! with the vendor ESI) and emitted into `config::generated::BUS`. Nothing here
//! parses XML on the MCU -- these are plain `Copy` PODs consumed by the bring-up
//! FSM (`fsm_slave_config`), the process-data `domain`, and the `hal` pin layer.

use crate::ethercat::ecrt::{EcDirection, EcPdoEntryInfo};

/// HAL representation width for a named process-data pin (the `halType`
/// attribute in the XML), independent of the wire bit length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HalType {
    /// Single bit at `(byte_offset, bit_pos)`.
    Bit,
    /// Unsigned, zero-extended from the wire bit length.
    U32,
    /// Signed, sign-extended from the wire bit length.
    S32,
}

/// One PDO (RxPDO `0x16xx` / TxPDO `0x1A0x`) and its ordered mapped entries.
#[derive(Clone, Copy, Debug)]
pub struct PdoCfg {
    pub index: u16,
    pub entries: &'static [EcPdoEntryInfo],
}

/// One process-data sync manager (SM2 outputs / SM3 inputs).
#[derive(Clone, Copy, Debug)]
pub struct SmCfg {
    /// Sync-manager index (2 = outputs, 3 = inputs).
    pub index: u8,
    /// Physical start address in the ESC (from the slave ESI/SII).
    pub phys_start: u16,
    /// SM control byte (from the ESI; e.g. `0x64` out, `0x20` in).
    pub control: u8,
    /// Direction of the data this SM carries.
    pub dir: EcDirection,
    /// Total mapped bytes for this SM (sum of assigned entry bit lengths / 8).
    pub size: u16,
    /// Assigned PDOs, in order.
    pub pdos: &'static [PdoCfg],
}

/// One FMMU mapping (logical domain region <-> physical SM region).
#[derive(Clone, Copy, Debug)]
pub struct FmmuCfg {
    /// Logical start address in the domain image.
    pub logical_start: u32,
    /// Mapped length, in bytes.
    pub size: u16,
    /// Physical SM start address on the slave.
    pub phys_start: u16,
    /// Direction (`Output` = master writes; `Input` = master reads).
    pub dir: EcDirection,
}

/// Distributed-clock configuration for a slave.
#[derive(Clone, Copy, Debug)]
pub struct DcCfg {
    /// DC activation word (`assignActivate`, e.g. `0x0300` for SYNC0).
    pub assign_activate: u16,
    /// SYNC0 cycle time, ns.
    pub sync0_cycle_ns: u32,
    /// SYNC0 shift time, ns.
    pub sync0_shift_ns: u32,
    /// SYNC1 cycle time, ns (0 if unused).
    pub sync1_cycle_ns: u32,
}

/// One SDO init value applied during bring-up (expedited only, <= 4 bytes).
#[derive(Clone, Copy, Debug)]
pub struct SdoInit {
    pub index: u16,
    pub subindex: u8,
    pub data: &'static [u8],
}

/// One streamed motion field: a slice of a host motion sample copied verbatim
/// into the process image. `sample_off` is the field's byte offset within the
/// streamed sample payload; `image_off` is its byte offset in the cyclic image.
/// Produced by the generator from `<motionStream>` (see `hal::spi_layout`).
#[derive(Clone, Copy, Debug)]
pub struct StreamField {
    pub sample_off: u16,
    pub image_off: u32,
    pub len: u8,
}

/// One named process-data pin resolved to a location in the domain image.
#[derive(Clone, Copy, Debug)]
pub struct PinCfg {
    pub name: &'static str,
    pub byte_offset: u32,
    pub bit_pos: u8,
    pub bit_len: u8,
    pub hal_type: HalType,
    pub dir: EcDirection,
}

/// Desired configuration for one slave.
#[derive(Clone, Copy, Debug)]
pub struct SlaveCfg {
    pub position: u16,
    pub vendor_id: u32,
    pub product_code: u32,
    pub sms: &'static [SmCfg],
    pub fmmus: &'static [FmmuCfg],
    pub dc: Option<DcCfg>,
    pub sdo_init: &'static [SdoInit],
    /// Output (RxPDO) image bytes for this slave.
    pub out_size: u16,
    /// Input (TxPDO) image bytes for this slave.
    pub in_size: u16,
}

/// The whole desired bus (one process-data domain).
#[derive(Clone, Copy, Debug)]
pub struct BusCfg {
    /// Cyclic period, ns (the master cycle and the SYNC0 base).
    pub cycle_ns: u64,
    /// Reference-clock slave position (DC).
    pub ref_clock_slave: u16,
    pub slaves: &'static [SlaveCfg],
    pub pins: &'static [PinCfg],
    /// Total process-image size, bytes (sum of all slaves' out + in regions).
    pub image_size: usize,
}

impl BusCfg {
    /// Look up a named pin.
    pub fn pin(&self, name: &str) -> Option<&PinCfg> {
        self.pins.iter().find(|p| p.name == name)
    }
}
