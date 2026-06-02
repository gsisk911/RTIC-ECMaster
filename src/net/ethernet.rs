//! Teensy 4.1 Ethernet setup: clock tree, IOMUXC pad mux, PHY init.
//!
//! Uses the vendored `rt1062-eth-rs` driver (`enet_driver` / `enet_ring`)
//! for the ENET DMA engine and smoltcp `Device` implementation.
//!
//! The setup sequence here is adapted from the `teensy_demo.rs` example
//! in Tim Vrakas's `rt1062-eth-rs` (MIT license), which is the most
//! battle-tested Rust Ethernet init for the Teensy 4.1 / DP83825I PHY.
//!
//! ## Call order
//!
//! 1. `setup_clocks_and_pins()` — enable ENET PLL, clock gate, IOMUXC pads, PHY reset
//! 2. `EnetDevice::new(...)` — init ENET MAC + DMA (from enet_driver module)
//! 3. `setup_phy(device)` — configure DP83825I registers via MDIO

use imxrt_hal as hal;
use imxrt_ral as ral;
use ral::{ccm, ccm_analog, iomuxc, iomuxc_gpr};
use hal::iomuxc::PullKeeper;

use crate::net::boot_diag;
use crate::net::enet_driver::EnetDevice;

const PLL_LOCK_TIMEOUT_LOOPS: u32 = 10_000_000;
const PHY_ADDR: u8 = 0;
const PHY_REG_BMSR: u8 = 0x01;
const PHY_LINK_UP_MASK: u16 = 0x0004;

/// Configure ENET clocks (PLL, CCM gate) and IOMUXC pads for Teensy 4.1.
///
/// Also handles the DP83825I PHY hardware reset via GPIO_B0_14 (RST)
/// and GPIO_B0_15 (SHDN) pins.
///
/// # Safety
/// Accesses CCM, CCM_ANALOG, IOMUXC, IOMUXC_GPR, and GPIO registers.
/// Must be called once during init before creating the EnetDevice.
pub unsafe fn setup_clocks_and_pins(
    gpio2: &mut hal::gpio::Port<2>,
) {
    let ccm1 = ccm::CCM::instance();
    let ccm_analog1 = ccm_analog::CCM_ANALOG::instance();
    let mux_gpr1 = iomuxc_gpr::IOMUXC_GPR::instance();
    let mux1 = iomuxc::IOMUXC::instance();

    // Enable ENET clock gate in CCM_CCGR1 (CG5 = bits 11:10)
    ral::modify_reg!(ccm, ccm1, CCGR1, CG5: 3);

    // Configure the ENET PLL for 50 MHz reference clock
    ral::write_reg!(ccm_analog, ccm_analog1, PLL_ENET_CLR,
        BYPASS_CLK_SRC: 3, ENET2_DIV_SELECT: 3, DIV_SELECT: 3, POWERDOWN: 1
    );
    ral::write_reg!(ccm_analog, ccm_analog1, PLL_ENET_SET,
        ENET_25M_REF_EN: 1, ENABLE: 1, BYPASS: 1, DIV_SELECT: 1
    );

    // Wait for PLL lock, but don't hang forever if ENET clocking is broken.
    let mut pll_locked = false;
    for _ in 0..PLL_LOCK_TIMEOUT_LOOPS {
        if ral::read_reg!(ccm_analog, ccm_analog1, PLL_ENET, LOCK) != 0 {
            pll_locked = true;
            break;
        }
        core::hint::spin_loop();
    }
    if pll_locked {
        ral::write_reg!(ccm_analog, ccm_analog1, PLL_ENET_CLR, BYPASS: 1);
    } else if boot_diag::record(boot_diag::FLAG_PLL_TIMEOUT, boot_diag::ERR_PLL_TIMEOUT) {
        log::warn!("[enet] timeout waiting for PLL_ENET lock");
    }

    // Route ref clock to ENET TX_CLK pin (output direction)
    ral::modify_reg!(iomuxc_gpr, mux_gpr1, GPR1,
        ENET1_CLK_SEL: 0, ENET_IPG_CLK_S_EN: 0, ENET1_TX_CLK_DIR: 1
    );

    // ── IOMUXC pad configuration ────────────────────────────────────
    let mut pads = hal::iomuxc::into_pads(ral::iomuxc::Instance::instance());

    const ENET_IO_PD: hal::iomuxc::Config = hal::iomuxc::Config::zero()
        .set_pull_keeper(Some(PullKeeper::Pulldown100k))
        .set_speed(hal::iomuxc::Speed::Max)
        .set_drive_strength(hal::iomuxc::DriveStrength::R0_5)
        .set_slew_rate(hal::iomuxc::SlewRate::Fast);

    const ENET_IO_PU: hal::iomuxc::Config = ENET_IO_PD
        .set_pull_keeper(Some(PullKeeper::Pullup22k));

    const XI_CONFIG: hal::iomuxc::Config = hal::iomuxc::Config::zero()
        .set_drive_strength(hal::iomuxc::DriveStrength::R0_6)
        .set_slew_rate(hal::iomuxc::SlewRate::Fast);

    // RXD0 — GPIO_B1_04, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p04, ENET_IO_PD);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p04, 3);

    // RXD1 — GPIO_B1_05, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p05, ENET_IO_PD);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p05, 3);

    // CRS_DV — GPIO_B1_06, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p06, ENET_IO_PD);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p06, 3);

    // RXER — GPIO_B1_11, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p11, ENET_IO_PD);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p11, 3);

    // TXD0 — GPIO_B1_07, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p07, ENET_IO_PU);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p07, 3);

    // TXD1 — GPIO_B1_08, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p08, ENET_IO_PU);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p08, 3);

    // TX_EN — GPIO_B1_09, ALT3
    hal::iomuxc::configure(&mut pads.gpio_b1.p09, ENET_IO_PU);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p09, 3);

    // REF_CLK — GPIO_B1_10, ALT6 + SION (50 MHz output to PHY)
    hal::iomuxc::configure(&mut pads.gpio_b1.p10, XI_CONFIG);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p10, 6);
    hal::iomuxc::set_sion(&mut pads.gpio_b1.p10);

    // MDIO — GPIO_B1_15, ALT0
    hal::iomuxc::configure(&mut pads.gpio_b1.p15, ENET_IO_PU);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p15, 0);

    // MDC — GPIO_B1_14, ALT0
    hal::iomuxc::configure(&mut pads.gpio_b1.p14, ENET_IO_PU);
    hal::iomuxc::alternate(&mut pads.gpio_b1.p14, 0);

    // IOMUXC daisy-chain (input select) registers
    ral::write_reg!(iomuxc, mux1, ENET_MDIO_SELECT_INPUT, DAISY: 2);
    ral::write_reg!(iomuxc, mux1, ENET0_RXDATA_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(iomuxc, mux1, ENET1_RXDATA_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(iomuxc, mux1, ENET_RXEN_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(iomuxc, mux1, ENET_RXERR_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(iomuxc, mux1, ENET_IPG_CLK_RMII_SELECT_INPUT, DAISY: 1);

    // ── PHY hardware reset ──────────────────────────────────────────
    // GPIO_B0_15 = PHY_SHDN, GPIO_B0_14 = PHY_RST (active low)
    let phy_shdn = gpio2.output(pads.gpio_b0.p15);
    let phy_rst = gpio2.output(pads.gpio_b0.p14);

    phy_shdn.clear();
    phy_rst.clear();
    cortex_m::asm::delay(150_000_000 / 20); // ~50 ms at 150 MHz
    phy_shdn.set();
    cortex_m::asm::delay(150_000_000 / 20); // ~50 ms
    phy_rst.set();
    cortex_m::asm::delay(150_000_000 / 20); // ~50 ms settle

    log::info!("[enet] clocks, pads, and PHY reset complete");
}

/// Configure the DP83825I PHY registers via MDIO.
///
/// Call after creating the `EnetDevice` (which sets up MDIO speed).
pub fn setup_phy<const INST: u8, const MTU: usize, const RX: usize, const TX: usize>(
    dev: &mut EnetDevice<'_, INST, MTU, RX, TX>,
) {
    // LED shows link status, active high
    let ledcr_ok = dev.mdio_write(PHY_ADDR, 0x18, 0x0280);
    // Configure for 50 MHz clock input
    let rcsr_ok = dev.mdio_write(PHY_ADDR, 0x17, 0x0081);

    // Read back for debug
    let rcsr = dev.mdio_read(PHY_ADDR, 0x17);
    let ledcr = dev.mdio_read(PHY_ADDR, 0x18);
    let phycr = dev.mdio_read(PHY_ADDR, 0x19);
    match (ledcr_ok, rcsr_ok, rcsr, ledcr, phycr) {
        (true, true, Some(rcsr), Some(ledcr), Some(phycr)) => {
            log::info!("[phy] RCSR={:#06x}, LEDCR={:#06x}, PHYCR={:#06x}", rcsr, ledcr, phycr);
        }
        _ => {
            log::warn!("[phy] MDIO setup incomplete; PHY debug registers unavailable");
        }
    }
}

pub fn read_link_state<const INST: u8, const MTU: usize, const RX: usize, const TX: usize>(
    dev: &mut EnetDevice<'_, INST, MTU, RX, TX>,
) -> Option<bool> {
    let _ = dev.mdio_read(PHY_ADDR, PHY_REG_BMSR);
    let bmsr_live = dev.mdio_read(PHY_ADDR, PHY_REG_BMSR)?;
    Some((bmsr_live & PHY_LINK_UP_MASK) != 0)
}

pub fn log_phy_status<const INST: u8, const MTU: usize, const RX: usize, const TX: usize>(
    dev: &mut EnetDevice<'_, INST, MTU, RX, TX>,
) {
    let bmcr = dev.mdio_read(PHY_ADDR, 0x00);
    let bmsr_latched = dev.mdio_read(PHY_ADDR, PHY_REG_BMSR);
    let bmsr_live = dev.mdio_read(PHY_ADDR, PHY_REG_BMSR);
    let phycr = dev.mdio_read(PHY_ADDR, 0x19);

    match (bmcr, bmsr_latched, bmsr_live, phycr) {
        (Some(bmcr), Some(bmsr_latched), Some(bmsr_live), Some(phycr)) => {
            let link_up = (bmsr_live & PHY_LINK_UP_MASK) != 0;
            let autoneg_complete = (bmsr_live & 0x0020) != 0;
            log::info!(
                "[phy] BMCR={:#06x}, BMSR(latched)={:#06x}, BMSR(live)={:#06x}, PHYCR={:#06x}, link_up={}, autoneg_complete={}",
                bmcr,
                bmsr_latched,
                bmsr_live,
                phycr,
                link_up,
                autoneg_complete
            );
        }
        _ => {
            log::warn!("[phy] unable to read post-boot PHY status");
        }
    }
}
