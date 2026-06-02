//! An SDO (CoE object dictionary entry) descriptor.
//!
//! IgH: master/sdo.c, master/sdo.h (`ec_sdo_t`) - description of one object in a
//! slave's CoE dictionary (index, object code, name, sub-entries).
//! Rust: owned struct with a fixed-capacity sub-entry list; only needed when we
//! fetch the dictionary, so v1 leaves it a stub.
//! Dropped (kernel-only): sysfs exposure, intrusive list linkage.

// TODO: define EcSdo (index, object_code, max_subindex, entries).
