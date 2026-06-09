//! EtherCAT master and its components.
//!
//! A file-for-file Rust mirror of the IgH EtherCAT Master (EtherLab) `master/`
//! core, adapted from the Linux kernel module to a bare-metal Cortex-M7 RTIC
//! application. The network is driven over raw Layer-2 frames (EtherType
//! `0x88A4`) provided by the `net` ENET driver instead of a Linux `net_device`.
//!
//! Layout follows IgH's flat `master/` directory (no subfolders). Kernel-only
//! glue (`module.c`, `cdev.c`, `ioctl.c`, `debug.c`, RTDM) is dropped; locking
//! becomes RTIC resources / `critical-section`, allocation becomes `heapless`
//! and fixed arrays, and `EC_DBG`/`EC_ERR` become the `log` facade.
//!
//! v1 scope: bus scan implemented; SDO/CoE, PDO, DC, and the per-slave
//! configuration FSMs are scaffolded with their IgH-sourced responsibilities.

#![allow(dead_code)]

// Public API surface + shared constants (include/ecrt.h, master/globals.h)
pub mod ecrt;
pub mod globals;

// Configuration model + parser (our additions, populated by a later feature)
pub mod config;

// Serial command interface (mirrors the IgH `ethercat` CLI tool)
pub mod cli;

// Frame transport (master/device.c, master/datagram.c)
pub mod datagram;
pub mod device;

// Discovered runtime model + desired configuration model
pub mod domain;
pub mod fmmu_config;
pub mod mailbox;
pub mod pdo;
pub mod pdo_entry;
pub mod pdo_list;
pub mod sdo;
pub mod sdo_entry;
pub mod sdo_request;
pub mod slave;
pub mod slave_config;
pub mod sync;
pub mod sync_config;

// Protocol state machines (master/fsm_*.c)
pub mod fsm_change;
pub mod fsm_coe;
pub mod fsm_master;
pub mod fsm_scan;
pub mod fsm_pdo;
pub mod fsm_pdo_entry;
pub mod fsm_sii;
pub mod fsm_slave;
pub mod fsm_slave_config;
pub mod fsm_slave_scan;

// Top-level master + cross-cutting distributed clocks
pub mod cyclic;
pub mod dc;
pub mod master;

// Application/interface layer (NOT an IgH master/ file; lives here for now)
pub mod cia402;
