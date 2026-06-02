//! Public EtherCAT realtime API surface + little-endian access helpers.
//!
//! IgH: include/ecrt.h (the application/realtime API and its public structs,
//! plus the `EC_READ_*` / `EC_WRITE_*` data-access macros).
//! Rust: the C macros become small inline slice helpers (`read_u16_le`,
//! `write_u32_le`, ...) that are bounds-checked by the slice; the opaque kernel
//! handles (`ec_master_t*` etc.) are replaced by owned Rust types in the
//! sibling modules. Error returns become `Result<_, EcError>`.
//! Dropped (kernel-only): the ioctl-backed userspace/library split; on the M7
//! the application links the master directly and calls these types in-process.

/// Errors surfaced by the master and its state machines.
///
/// IgH returns negative `int` error codes; we use a typed enum instead.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EcError {
    /// No matching reply observed before the transaction deadline.
    Timeout,
    /// Working counter was not the expected value for the command.
    WorkingCounter,
    /// A received frame was too short to parse.
    FrameTooShort,
    /// SII/EEPROM reported a command/acknowledge error.
    SiiError,
    /// SII/EEPROM stayed busy past its deadline.
    SiiTimeout,
    /// More slaves were found than the fixed `EC_MAX_SLAVES` capacity.
    TooManySlaves,
    /// The Ethernet/transport layer could not accept the frame.
    Transport,
    /// Referenced a slave position that was not discovered.
    NoSuchSlave,
    /// The slave does not support CoE (no CoE bit in its mailbox protocols).
    CoeUnsupported,
    /// The slave's mailbox stayed empty/unfilled past its deadline.
    MailboxTimeout,
    /// A received mailbox frame was malformed or an unexpected protocol/service.
    MailboxProtocol,
    /// An AL state transition failed; carries the AL status code (0x0134).
    StateChange(u16),
    /// A CoE SDO transfer was aborted by the slave; carries the abort code.
    SdoAbort(u32),
}

/// Sync-manager direction, mirroring `ec_direction_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EcDirection {
    /// Master -> slave (outputs, e.g. SM2 / RxPDO).
    Output,
    /// Slave -> master (inputs, e.g. SM3 / TxPDO).
    Input,
}

/// One PDO entry mapping, mirroring `ec_pdo_entry_info_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct EcPdoEntryInfo {
    pub index: u16,
    pub subindex: u8,
    pub bit_length: u8,
}

/// One PDO, mirroring `ec_pdo_info_t` (entries held by the owner, not boxed).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct EcPdoInfo {
    pub index: u16,
    pub n_entries: u8,
}

/// One sync-manager configuration, mirroring `ec_sync_info_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct EcSyncInfo {
    pub index: u8,
    pub dir: EcDirection,
    pub n_pdos: u8,
}

/// A registered PDO-entry's resolved location in the process image.
///
/// IgH: the `*offset`/`*bit_position` written back by
/// `ecrt_domain_reg_pdo_entry_list`. Here we return the pair by value.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct EcPdoEntryReg {
    pub byte_offset: u32,
    pub bit_position: u8,
}

/// Public per-slave identity/state summary, mirroring `ec_slave_info_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct EcSlaveInfo {
    pub position: u16,
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision_number: u32,
    pub al_state: u8,
}

/// Aggregate master/bus state, mirroring `ec_master_state_t`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct EcMasterState {
    pub slaves_responding: u16,
    pub al_states: u8,
    pub link_up: bool,
}

// ---- Little-endian data-access helpers (replace EC_READ_*/EC_WRITE_*) ----
//
// Every multi-byte field inside an EtherCAT frame is little-endian. These read
// from / write to the start of the provided slice; pass a subslice to address
// an offset (e.g. `read_u16_le(&buf[6..])`), matching IgH's `data + n` idiom.

#[inline]
pub fn read_u8(buf: &[u8]) -> u8 {
    buf[0]
}

#[inline]
pub fn read_u16_le(buf: &[u8]) -> u16 {
    u16::from_le_bytes([buf[0], buf[1]])
}

#[inline]
pub fn read_u32_le(buf: &[u8]) -> u32 {
    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
}

#[inline]
pub fn write_u8(buf: &mut [u8], value: u8) {
    buf[0] = value;
}

#[inline]
pub fn write_u16_le(buf: &mut [u8], value: u16) {
    buf[0..2].copy_from_slice(&value.to_le_bytes());
}

#[inline]
pub fn write_u32_le(buf: &mut [u8], value: u32) {
    buf[0..4].copy_from_slice(&value.to_le_bytes());
}

#[inline]
pub fn read_u64_le(buf: &[u8]) -> u64 {
    u64::from_le_bytes([
        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
    ])
}

#[inline]
pub fn write_u64_le(buf: &mut [u8], value: u64) {
    buf[0..8].copy_from_slice(&value.to_le_bytes());
}
