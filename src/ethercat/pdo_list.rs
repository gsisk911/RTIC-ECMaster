//! Ordered collection of PDOs (per sync manager).
//!
//! IgH: master/pdo_list.c, master/pdo_list.h (`ec_pdo_list_t`) - the ordered set
//! of PDOs assigned to a sync manager, with add/clear/compare helpers.
//! Rust: a fixed-capacity `heapless::Vec<EcPdo, EC_MAX_PDOS>` wrapper; `Drop`
//! replaces the manual `_clear`.
//! Dropped (kernel-only): intrusive `list_head` linkage.

// TODO: define EcPdoList wrapping heapless::Vec<EcPdo, EC_MAX_PDOS>.
