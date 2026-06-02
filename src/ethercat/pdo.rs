//! A PDO (Process Data Object) and its entry list.
//!
//! IgH: master/pdo.c, master/pdo.h (`ec_pdo_t`) - one PDO identified by index
//! (e.g. 0x1600/0x1A00) owning an ordered list of mapped entries.
//! Rust: owned struct holding a fixed-capacity entry list (`EC_MAX_PDO_ENTRIES`)
//! instead of an intrusive kernel list; pairs with `ecrt::EcPdoInfo`.
//! Dropped (kernel-only): none of note.

// TODO: define EcPdo (index + heapless::Vec<EcPdoEntry, EC_MAX_PDO_ENTRIES>).
