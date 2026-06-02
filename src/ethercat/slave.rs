//! Discovered slave runtime model.
//!
//! IgH: master/slave.c, master/slave.h (`ec_slave_t`) - the runtime state of one
//! physical slave: identity (vendor/product/revision/serial), DL/base info,
//! ports/topology, current AL state, sync managers, PDOs, and the SII image.
//! Rust: a plain owned struct (no intrusive `list_head`); the master holds a
//! `heapless::Vec<SlaveInfo, EC_MAX_SLAVES>` instead of a kernel linked list.
//! Dropped (kernel-only): `kobject`/sysfs entries, per-slave `kthread` hooks.
//!
//! v1 populates only the identity/base fields read by the bus scan; the full
//! SM/PDO/SII-category model is added with the configuration feature.

/// Identity and base information for one slave, as produced by the bus scan.
///
/// IgH: the subset of `ec_slave_t` filled by `ec_fsm_slave_scan` (base info at
/// register 0x0000 and SII identity words 0x0008/0x000A/0x000C).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct SlaveInfo {
    /// Ring position (auto-increment order, 0-based).
    pub ring_pos: u16,
    /// Configured station address assigned during the scan (`ring_pos + 1`).
    pub station_addr: u16,
    /// Application-layer state nibble read from AL status (0x0130).
    pub al_state: u8,
    /// Base type byte (DL info register 0x0000).
    pub base_type: u8,
    /// Number of supported FMMUs (DL info byte 0x0004).
    pub base_fmmu_count: u8,
    /// Number of supported sync managers (DL info byte 0x0005).
    pub base_sync_count: u8,
    /// Vendor ID (SII word 0x0008).
    pub vendor_id: u32,
    /// Product code (SII word 0x000A).
    pub product_code: u32,
    /// Revision number (SII word 0x000C).
    pub revision: u32,
    /// RxMailbox physical start address (SII word 0x0018 low).
    pub rx_mbox_offset: u16,
    /// RxMailbox size in bytes (SII word 0x0018 high).
    pub rx_mbox_size: u16,
    /// TxMailbox physical start address (SII word 0x001A low).
    pub tx_mbox_offset: u16,
    /// TxMailbox size in bytes (SII word 0x001A high).
    pub tx_mbox_size: u16,
    /// Supported mailbox protocols bitmask (SII word 0x001C low).
    pub mbox_protocols: u16,
    /// Whether the slave advertises CoE support.
    pub supports_coe: bool,
}

/// Mailbox sync-manager parameters needed to run a CoE transfer.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Mailbox {
    pub station_addr: u16,
    pub rx_offset: u16,
    pub rx_size: u16,
    pub tx_offset: u16,
    pub tx_size: u16,
}

impl SlaveInfo {
    /// The mailbox sync-manager parameters for this slave.
    pub fn mailbox(&self) -> Mailbox {
        Mailbox {
            station_addr: self.station_addr,
            rx_offset: self.rx_mbox_offset,
            rx_size: self.rx_mbox_size,
            tx_offset: self.tx_mbox_offset,
            tx_size: self.tx_mbox_size,
        }
    }
}
