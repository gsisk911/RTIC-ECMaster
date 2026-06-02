//! Link layer: network controllers and frame transport.
//!
//! Houses the Teensy 4.1 built-in RMII ENET driver (vendored `rt1062-eth-rs`:
//! `enet_driver`, `enet_ring`, `ethernet`, `boot_diag`) which will carry raw
//! EtherCAT Layer-2 frames, plus the legacy `w5500_spi` controller (kept in the
//! tree but with its network connection disabled).

#[allow(dead_code)]
pub mod boot_diag;
#[allow(dead_code)]
pub mod enet_driver;
#[allow(dead_code)]
pub mod enet_ring;
#[allow(dead_code)]
pub mod ethernet;
pub mod w5500_spi;
