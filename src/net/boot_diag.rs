use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

pub const FLAG_PLL_TIMEOUT: u32 = 1 << 0;
pub const FLAG_MDIO_WRITE_TIMEOUT: u32 = 1 << 1;
pub const FLAG_MDIO_READ_TIMEOUT: u32 = 1 << 2;
pub const FLAG_ADC_INIT_TIMEOUT: u32 = 1 << 3;
pub const FLAG_ADC_READ_TIMEOUT: u32 = 1 << 4;

pub const ERR_PLL_TIMEOUT: u8 = 1;
pub const ERR_MDIO_WRITE_TIMEOUT: u8 = 2;
pub const ERR_MDIO_READ_TIMEOUT: u8 = 3;
pub const ERR_ADC_INIT_TIMEOUT: u8 = 4;
pub const ERR_ADC_READ_TIMEOUT: u8 = 5;

static BOOT_FLAGS: AtomicU32 = AtomicU32::new(0);
static FIRST_ERROR: AtomicU8 = AtomicU8::new(0);

pub fn record(flag: u32, code: u8) -> bool {
    let previous = BOOT_FLAGS.fetch_or(flag, Ordering::Relaxed);
    if previous & flag == 0 {
        let _ = FIRST_ERROR.compare_exchange(0, code, Ordering::Relaxed, Ordering::Relaxed);
        true
    } else {
        false
    }
}

pub fn emit_logs() {
    let flags = BOOT_FLAGS.load(Ordering::Relaxed);
    if flags == 0 {
        return;
    }

    let first_error = FIRST_ERROR.load(Ordering::Relaxed);
    log::warn!(
        "[boot] startup diagnostics flags=0x{:08X}, first_error={}",
        flags,
        first_error
    );

    if flags & FLAG_PLL_TIMEOUT != 0 {
        log::warn!("[boot] timed out waiting for ENET PLL lock");
    }
    if flags & FLAG_MDIO_WRITE_TIMEOUT != 0 {
        log::warn!("[boot] timed out waiting for MDIO write completion");
    }
    if flags & FLAG_MDIO_READ_TIMEOUT != 0 {
        log::warn!("[boot] timed out waiting for MDIO read completion");
    }
    if flags & FLAG_ADC_INIT_TIMEOUT != 0 {
        log::warn!("[boot] timed out waiting for ADC2 calibration");
    }
    if flags & FLAG_ADC_READ_TIMEOUT != 0 {
        log::warn!("[boot] timed out waiting for ADC2 conversion");
    }
}
