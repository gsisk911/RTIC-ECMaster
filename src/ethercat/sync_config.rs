//! Desired sync-manager configuration (PDO assignment) for a slave config.
//!
//! IgH: master/sync_config.c, master/sync_config.h (`ec_sync_config_t`) - the
//! application's intended PDO assignment for one sync manager, consumed when
//! building the `ec_sync_info_t` array passed to `ecrt_slave_config_pdos`.
//! Rust: owned struct with a fixed-capacity PDO list (no kmalloc).
//! Dropped (kernel-only): none of note.

// TODO: define EcSyncConfig (dir + assigned PDO list) feeding ecrt::EcSyncInfo.
