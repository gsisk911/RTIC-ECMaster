//! Link layer: the Teensy 4.1 built-in RMII ENET driver (vendored
//! `rt1062-eth-rs`: `enet_driver`, `enet_ring`, `ethernet`, `boot_diag`) that
//! carries the raw EtherCAT Layer-2 frames. RMII-only: the legacy W5500 SPI
//! controller and the Modbus stack have been dropped from this firmware.

#[allow(dead_code)]
pub mod boot_diag;
#[allow(dead_code)]
pub mod enet_driver;
#[allow(dead_code)]
pub mod enet_ring;
#[allow(dead_code)]
pub mod ethernet;
