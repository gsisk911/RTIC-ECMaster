//! Core constants, limits, and ESC register map.
//!
//! IgH: master/globals.h (constants, AL states, register offsets, size limits).
//! Rust: grouped `const` modules and `#[repr(u8)]`-friendly values instead of C
//! `#define`s. The datagram command codes live in `datagram.rs` (`Command`),
//! mirroring IgH where `ec_datagram_type_t` is declared in `datagram.h`.
//! Dropped (kernel-only): `EC_DBG`/`EC_ERR`/`printk` debug machinery -> the
//! `log` facade at the call sites; `kmalloc`-driven dynamic limits -> the fixed
//! `EC_MAX_*` capacities below (no heap on the M7).

/// EtherType for EtherCAT frames (written big-endian in the Ethernet header).
pub const ETHERCAT_ETHERTYPE: u16 = 0x88A4;

/// EtherCAT frame header size, in bytes (length + type word).
pub const EC_FRAME_HEADER_SIZE: usize = 2;
/// Datagram header size, in bytes (cmd, idx, address[4], len+flags, irq).
pub const EC_DATAGRAM_HEADER_SIZE: usize = 10;
/// Datagram footer size, in bytes (working counter).
pub const EC_DATAGRAM_FOOTER_SIZE: usize = 2;
/// Address field length inside the datagram header (ADP[2] + ADO[2]).
pub const EC_ADDR_LEN: usize = 4;

/// Maximum Ethernet frame we build/parse (no jumbo frames).
pub const EC_MAX_FRAME_SIZE: usize = 1514;
/// Ethernet header length (dst[6] + src[6] + ethertype[2]).
pub const ETH_HEADER_LEN: usize = 14;
/// Minimum padded length of the EtherCAT portion so the Ethernet frame reaches
/// the 60-byte minimum (`ETH_ZLEN - ETH_HLEN` = 60 - 14). IgH pads to this.
pub const EC_MIN_ECAT_FRAME: usize = 46;

// ---- Fixed capacities (replace IgH's kmalloc'd, unbounded lists) ----

/// Maximum number of slaves we track on the bus.
pub const EC_MAX_SLAVES: usize = 32;
/// Maximum sync managers per slave.
pub const EC_MAX_SYNC_MANAGERS: usize = 16;
/// Maximum FMMUs per slave.
pub const EC_MAX_FMMUS: usize = 16;
/// Maximum PDOs per sync manager.
pub const EC_MAX_PDOS: usize = 32;
/// Maximum entries per PDO.
pub const EC_MAX_PDO_ENTRIES: usize = 32;

/// Sync-manager configuration page size in the ESC (bytes per SM at 0x0800+).
pub const EC_SYNC_PAGE_SIZE: u16 = 8;

/// Scratch frame buffer size for scan-time datagrams. Scan datagrams are tiny
/// (<= 12-byte payloads, so <= the 46-byte padded minimum); this keeps the
/// blocking helpers off the heap with a small stack footprint.
pub const EC_SCAN_FRAME_BUF: usize = 128;

/// ESC slave register offsets (the ADO used with FPRD/FPWR/APRD/APWR/BRD/BWR).
///
/// IgH: scattered `#define`s across master/globals.h and the FSM sources.
pub mod reg {
    /// DL information (type, revision, build, FMMU/SM counts, ports, features).
    pub const DL_INFO: u16 = 0x0000;
    /// Configured station address (assigned during scan).
    pub const STATION_ADDR: u16 = 0x0010;
    /// Configured station alias (from SII).
    pub const STATION_ALIAS: u16 = 0x0012;
    /// DL status (per-port link/loop/communication state).
    pub const DL_STATUS: u16 = 0x0110;
    /// AL control (request a state transition).
    pub const AL_CONTROL: u16 = 0x0120;
    /// AL status (current application-layer state).
    pub const AL_STATUS: u16 = 0x0130;
    /// AL status code (transition error diagnostics).
    pub const AL_STATUS_CODE: u16 = 0x0134;
    /// SII/EEPROM access assignment (0x00 grants EEPROM to the master).
    pub const SII_ACCESS: u16 = 0x0500;
    /// SII/EEPROM control/status (command byte + busy/error bits).
    pub const SII_CONTROL: u16 = 0x0502;
    /// SII/EEPROM word address.
    pub const SII_ADDRESS: u16 = 0x0504;
    /// SII/EEPROM read data (up to 2 words / 4 bytes per read).
    pub const SII_DATA: u16 = 0x0508;
    /// First sync-manager configuration page (mailbox out / RxMailbox).
    pub const SM0: u16 = 0x0800;
    /// Second sync-manager configuration page (mailbox in / TxMailbox).
    pub const SM1: u16 = 0x0808;
    /// Sync-manager 0 status byte (page 0 + 5).
    pub const SM0_STATUS: u16 = 0x0805;
    /// Sync-manager 1 status byte (page 1 + 5); mailbox-full bit lives here.
    pub const SM1_STATUS: u16 = 0x080D;
    /// DC receive-time latch / per-port receive times.
    pub const DC_RECV_TIME: u16 = 0x0900;
    /// DC system time.
    pub const DC_SYS_TIME: u16 = 0x0910;
    /// DC system time offset + transmission delay.
    pub const DC_SYS_TIME_OFFSET: u16 = 0x0920;
    /// DC system-time difference (drift; magnitude + sign bit, ns) - 0x092C.
    pub const DC_SYS_TIME_DIFF: u16 = 0x092C;
    /// DC activation register (the `assignActivate` word, e.g. 0x0300) - 0x0980.
    pub const DC_ACTIVATION: u16 = 0x0980;
    /// DC cyclic-operation start time (U64, ns) - 0x0990.
    pub const DC_CYCLE_START: u16 = 0x0990;
    /// DC SYNC0 cycle time (U32, ns) - 0x09A0.
    pub const DC_SYNC0_CYCLE: u16 = 0x09A0;
    /// DC SYNC1 cycle time (U32, ns) - 0x09A4.
    pub const DC_SYNC1_CYCLE: u16 = 0x09A4;
    /// Third sync-manager configuration page (process-data out / RxPDO) - 0x0810.
    pub const SM2: u16 = 0x0810;
    /// Fourth sync-manager configuration page (process-data in / TxPDO) - 0x0818.
    pub const SM3: u16 = 0x0818;
    /// FMMU configuration base; 16 bytes per FMMU at 0x0600 + n*16.
    pub const FMMU_BASE: u16 = 0x0600;
    /// Watchdog divider register - 0x0400.
    pub const WD_DIVIDER: u16 = 0x0400;
    /// Process-data watchdog time register - 0x0420.
    pub const WD_PDATA: u16 = 0x0420;
}

/// SII/EEPROM image word offsets (word-addressed; multiply by 2 for bytes).
///
/// IgH: master/slave.h / fsm_slave_scan.c category + identity parsing.
pub mod sii {
    /// Configured station alias.
    pub const ALIAS: u16 = 0x0004;
    /// Vendor ID (32-bit, spans words 0x0008..0x0009).
    pub const VENDOR_ID: u16 = 0x0008;
    /// Product code (32-bit, spans words 0x000A..0x000B).
    pub const PRODUCT_CODE: u16 = 0x000A;
    /// Revision number (32-bit).
    pub const REVISION: u16 = 0x000C;
    /// Serial number (32-bit).
    pub const SERIAL: u16 = 0x000E;
    /// Standard RxMailbox: offset (low u16) + size (high u16), word 0x0018.
    pub const STD_RX_MBOX: u16 = 0x0018;
    /// Standard TxMailbox: offset (low u16) + size (high u16), word 0x001A.
    pub const STD_TX_MBOX: u16 = 0x001A;
    /// Supported mailbox protocols bitmask (low u16), word 0x001C.
    pub const MBOX_PROTOCOLS: u16 = 0x001C;
}

/// Mailbox + CoE protocol constants.
///
/// IgH: master/mailbox.h, master/fsm_coe.c.
pub mod mbox {
    /// Mailbox header size, in bytes (length, address, channel/prio, type/cnt).
    pub const HEADER_SIZE: usize = 6;
    /// Mailbox protocol type: CoE.
    pub const TYPE_COE: u8 = 0x03;
    /// CoE supported-protocol bit in SII word 0x001C.
    pub const PROTO_COE: u16 = 0x0004;
    /// CoE service (bits 12..15 of the CoE header): SDO request.
    pub const COE_SDO_REQUEST: u16 = 0x02;
    /// CoE service: SDO response.
    pub const COE_SDO_RESPONSE: u16 = 0x03;
    /// SM status mailbox-full bit (bit 3): data available to read.
    pub const SM_STATUS_MBOX_FULL: u8 = 0x08;
}

/// Sync-manager control/enable byte values for mailbox SMs.
///
/// IgH: master/slave_config.c / fsm_slave_config.c mailbox SM setup.
pub mod sm {
    /// Mailbox-write SM (master->slave / RxMailbox): mailbox mode, write, ECAT.
    pub const CONTROL_MBOX_WRITE: u8 = 0x26;
    /// Mailbox-read SM (slave->master / TxMailbox): mailbox mode, read, ECAT.
    pub const CONTROL_MBOX_READ: u8 = 0x22;
    /// Process-data output SM (master->slave / SM2 / RxPDO): buffered (3-buffer)
    /// mode, write, ECAT, watchdog enable. Concrete CiA-402 value.
    pub const CONTROL_PD_OUT: u8 = 0x64;
    /// Process-data input SM (slave->master / SM3 / TxPDO): buffered, read, ECAT.
    pub const CONTROL_PD_IN: u8 = 0x20;
    /// SM enable bit (activate register).
    pub const ENABLE: u8 = 0x01;
}

/// FMMU configuration (the 16-byte ESC page at 0x0600 + n*16).
///
/// IgH: master/fmmu_config.c `ec_fmmu_config_page`.
pub mod fmmu {
    /// Size of one FMMU configuration page, in bytes.
    pub const PAGE_SIZE: usize = 16;
    /// Direction byte: input (read, slave -> master / TxPDO).
    pub const DIR_INPUT: u8 = 0x01;
    /// Direction byte: output (write, master -> slave / RxPDO).
    pub const DIR_OUTPUT: u8 = 0x02;
}

/// CoE object indices used for PDO assignment.
///
/// IgH: master/fsm_pdo.c (the `0x1C10 + sync_index` assignment objects).
pub mod coe {
    /// PDO assignment object for sync manager `n` (`0x1C10 + n`).
    #[inline]
    pub const fn pdo_assign(sync_index: u8) -> u16 {
        0x1C10 + sync_index as u16
    }
}

/// Maximum allowed DC system-time drift before going cyclic, in nanoseconds.
pub const EC_DC_MAX_DRIFT_NS: u32 = 10_000;

/// Maximum process-data bytes in one logical datagram (the per-frame budget:
/// ~1500 - Ethernet/EtherCAT/datagram headers - working counter).
pub const EC_MAX_DATA_SIZE: usize = 1486;

/// SII control/status (register 0x0502/0x0503) command + status bits.
pub mod sii_ctrl {
    /// Low control byte value selecting "two address octets" addressing.
    pub const ADDR_MODE_TWO_OCTET: u8 = 0x80;
    /// Read command (written to the 0x0503 command byte).
    pub const CMD_READ: u8 = 0x01;
    /// Write command.
    pub const CMD_WRITE: u8 = 0x02;
    /// Command/acknowledge error bit in the status byte.
    pub const STATUS_ERROR: u8 = 0x20;
    /// Busy bits (bit7 = busy, bit0 = read in progress); mask 0x81.
    pub const STATUS_BUSY: u8 = 0x81;
}

/// Application-layer (AL) states, low nibble of AL status (0x0130).
///
/// IgH: `ec_al_state_t` in master/globals.h.
pub mod al_state {
    pub const INIT: u8 = 0x01;
    pub const PREOP: u8 = 0x02;
    pub const BOOT: u8 = 0x03;
    pub const SAFEOP: u8 = 0x04;
    pub const OP: u8 = 0x08;
    /// Error flag OR'd into the AL status (0x0130) when a transition fails.
    pub const ERROR: u8 = 0x10;
    /// AL control (0x0120) acknowledge bit (bit 4, same position as `ERROR`):
    /// written together with the requested state to clear a latched AL error.
    /// Mirrors IgH `ec_fsm_change`'s error-acknowledge handshake.
    pub const ACK_ERROR: u8 = 0x10;
    /// Mask isolating the state nibble from the AL status byte.
    pub const MASK: u8 = 0x0F;
}
