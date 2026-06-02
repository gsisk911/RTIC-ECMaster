//! A single mapped PDO entry.
//!
//! IgH: master/pdo_entry.c, master/pdo_entry.h (`ec_pdo_entry_t`) - one mapped
//! object: index, subindex, and bit length within a PDO.
//! Rust: thin owned struct; equivalent to `ecrt::EcPdoEntryInfo` but as part of
//! the discovered/desired model rather than the public API surface.
//! Dropped (kernel-only): none of note.

// TODO: define EcPdoEntry (index, subindex, bit_length).
