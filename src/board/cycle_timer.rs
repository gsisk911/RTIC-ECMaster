//! Periodic Interrupt Timer (PIT) for the cyclic EtherCAT process-data loop.
//!
//! Drives the high-priority `cyclic` RTIC task at a fixed period with low jitter
//! (the cycle is hardware-timed, not derived from the 1 kHz SysTick monotonic).
//! PIT channel 0 is clocked from the 24 MHz crystal oscillator so the period is
//! independent of the core/IPG clock, and is sized for up to 4 kHz (250 us); v1
//! runs at the configured `BUS.cycle_ns` (100 Hz). The channel is configured
//! stopped at init and started when cyclic operation begins.

use imxrt_ral as ral;

/// PIT input clock: the 24 MHz oscillator (selected in [`configure`]).
pub const PERCLK_HZ: u32 = 24_000_000;

/// Compute the PIT load value (period - 1, in PIT ticks) for `cycle_ns`.
pub fn load_value(cycle_ns: u64) -> u32 {
    let ticks = cycle_ns.saturating_mul(PERCLK_HZ as u64) / 1_000_000_000;
    (ticks.max(1) - 1) as u32
}

/// The actual cyclic rate (Hz) produced by a given PIT `load` value. Integer
/// PIT division means the realized rate can differ slightly from the requested
/// one; this reports what the hardware will actually run at.
pub fn actual_hz(load: u32) -> u32 {
    PERCLK_HZ / (load + 1)
}

/// Configure PIT channel 0 for a `cycle_ns` period: select the 24 MHz
/// oscillator as PERCLK, gate on the PIT, enable the module, program the load
/// value, and enable (but do not start) the channel interrupt. Returns the load
/// value programmed.
///
/// SAFETY note: steals the CCM and PIT instances. Called once from `init` on the
/// single core before the PIT is used; nothing else drives PERCLK as a timer.
pub fn configure(cycle_ns: u64) -> u32 {
    let ccm = unsafe { ral::ccm::CCM::instance() };
    // PERCLK from the 24 MHz oscillator (PERCLK_CLK_SEL = 1), no divide.
    ral::modify_reg!(ral::ccm, ccm, CSCMR1, PERCLK_CLK_SEL: 1, PERCLK_PODF: 0);
    // Enable the PIT clock gate (CCGR1 field CG6).
    ral::modify_reg!(ral::ccm, ccm, CCGR1, CG6: 0b11);

    let pit = unsafe { ral::pit::PIT::instance() };
    // Enable the PIT module (clear MDIS); keep running while debugger halted off.
    ral::modify_reg!(ral::pit, pit, MCR, MDIS: 0, FRZ: 0);

    let load = load_value(cycle_ns);
    ral::write_reg!(ral::pit::timer, &pit.TIMER[0], TCTRL, 0); // stop channel 0
    ral::write_reg!(ral::pit::timer, &pit.TIMER[0], LDVAL, load);
    ral::write_reg!(ral::pit::timer, &pit.TIMER[0], TFLG, TIF: 1); // clear pending
    load
}

/// Start the periodic interrupt on channel 0 (timer + interrupt enable).
pub fn start() {
    let pit = unsafe { ral::pit::PIT::instance() };
    ral::write_reg!(ral::pit::timer, &pit.TIMER[0], TFLG, TIF: 1);
    ral::modify_reg!(ral::pit::timer, &pit.TIMER[0], TCTRL, TIE: 1, TEN: 1);
}

/// Stop the periodic interrupt on channel 0.
pub fn stop() {
    let pit = unsafe { ral::pit::PIT::instance() };
    ral::modify_reg!(ral::pit::timer, &pit.TIMER[0], TCTRL, TIE: 0, TEN: 0);
}

/// Clear the channel-0 interrupt flag (call at the top of the cyclic ISR).
pub fn clear_interrupt() {
    let pit = unsafe { ral::pit::PIT::instance() };
    ral::write_reg!(ral::pit::timer, &pit.TIMER[0], TFLG, TIF: 1);
}
