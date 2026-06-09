# Compile-time configuration flow

The EtherCAT bus is configured **entirely at compile time** — there is no XML
parser on the MCU. A small Python generator turns a LinuxCNC/lcec-style bus
description plus the vendor ESI into a Rust constant table that the firmware
links in.

```text
ethercat-conf.bohign.xml  ─┐
   (the desired bus)        │
                            ├─►  scripts/generate_ethercat_config.py  ─►  src/ethercat/config/generated.rs
Bohign_MS_ECAT_V2.5.xml   ─┘        (run via `make config`)                (the BUS table + PINS map)
   (the vendor ESI)
```

The generated file is **committed**; regenerate it with `make config` and never
hand-edit it.

---

## 1. The two inputs

### `ethercat-conf.bohign.xml` — the desired bus

This is the source of truth for *what you want the bus to do*: the master cycle,
each slave's vendor/product, the Distributed-Clocks config, SDO init values, and
the sync-manager → PDO → entry mapping with `halPin` names. It is the LinuxCNC
"lcec" dialect (the same dialect as a plain `ethercat-conf.xml`). The committed
file defines **two** identical drives (`drive0` / `drive1`, ring positions 0 and
1); the excerpt below shows the first — the second is identical apart from its
`idx` and its `drive1-` pin namespace (pin names must be unique across the bus).

```xml
<masters>
  <master idx="0" appTimePeriod="10000000" refClockSyncCycles="1" refClockSlaveIdx="0">
    <slave idx="0" type="generic" vid="0x00000994" pid="0x00001B00" configPdos="true" name="drive0">

      <!-- Distributed Clocks: SYNC0 enabled (DC-Synchron op-mode 0x0300). -->
      <dcConf assignActivate="0x0300" sync0Cycle="*1" sync0Shift="0"/>

      <!-- Init SDO: Modes of Operation = 8 (CSP). -->
      <sdoConfig idx="0x6060" subIdx="0x00">
        <sdoDataRaw data="08"/>
      </sdoConfig>

      <!-- SM2 / RxPDO1 0x1600 : outputs (master -> drive), 16 bytes. -->
      <syncManager idx="2" dir="out">
        <pdo idx="1600">
          <pdoEntry idx="6040" subIdx="00" bitLen="16" halType="u32" halPin="drive0-controlword"/>
          <pdoEntry idx="607a" subIdx="00" bitLen="32" halType="s32" halPin="drive0-target-position"/>
          ...
        </pdo>
      </syncManager>

      <!-- SM3 / TxPDO1 0x1A00 : inputs (drive -> master), 39 bytes. -->
      <syncManager idx="3" dir="in">
        <pdo idx="1a00"> ... </pdo>
      </syncManager>

    </slave>
  </master>
</masters>
```

Attribute reference:

| XML | Meaning |
| --- | --- |
| `master @appTimePeriod` | Master cycle, **nanoseconds** (`10000000` = 100 Hz; `250000` = 4 kHz). This is also the SYNC0 base. |
| `master @refClockSlaveIdx` | Reference-clock slave ring position (DC). |
| `slave @idx` | Ring position (0-based). |
| `slave @vid` / `@pid` | Vendor ID / product code (hex `0x…` or `#x…`). The `pid` is how a slave is matched to its ESI device. |
| `dcConf @assignActivate` | DC activation word (vendor-specific; `0x0300` = enable SYNC0). |
| `dcConf @sync0Cycle` | `*N` = `N × appTimePeriod`; a literal = nanoseconds. |
| `dcConf @sync0Shift` | SYNC0 shift, ns (folded into the cyclic start-time math). |
| `sdoConfig @idx`/`@subIdx` + `sdoDataRaw @data` | An SDO init value (space-separated **little-endian** hex bytes) applied during bring-up. **Must be ≤ 4 bytes** (expedited). |
| `syncManager @idx`/`@dir` | SM index (`2` = outputs, `3` = inputs) and direction (`out`/`in`). |
| `pdo @idx` | PDO index (bare hex, e.g. `1600`/`1a00`). |
| `pdoEntry @idx`/`@subIdx`/`@bitLen` | The mapped object index/subindex (bare hex) and its wire bit length. |
| `pdoEntry @halType` | HAL representation: `bit`, `u32` (unsigned), or `s32` (signed). Independent of `bitLen`. |
| `pdoEntry @halPin` | The name you read/write the field by in the `pd` command. Omit it for padding entries. |

### `Bohign_MS_ECAT_V2.5.xml` — the vendor ESI

The EtherCAT Slave Information file (device description). The generator uses it
for one thing the bus XML doesn't carry: each device's **process-data
sync-manager physical start addresses and control bytes** (SM2/SM3). Each bus
`<slave>` is matched to an ESI `<Device>` by **product code**.

---

## 2. The generator (`scripts/generate_ethercat_config.py`)

Stdlib only (`xml.etree.ElementTree`). It:

1. Parses the ESI into `product_code → [ (SM start, SM control), … ]`, ordered by
   appearance (SM0, SM1, SM2, SM3, …).
2. Parses the bus XML: cycle, reference clock, and per slave the DC config, SDO
   init list, and the out/in sync managers with their PDOs and entries.
3. For each slave, matches the ESI device by `pid` and pulls **SM2/SM3** physical
   start + control byte (it indexes the ESI SM list directly as `sms[2]`,
   `sms[3]`). Process data lives in SM2/SM3, so the device **must declare at least
   4 sync managers** (SM0..SM3) — a mismatched (e.g. second-slave) ESI fails with a
   clear message instead of an `IndexError`.
4. Computes the process-image layout: **all outputs first, then all inputs**.
   Each slave's output region is appended at the running output cursor and its
   input region at the running input cursor (which starts after all outputs).
   Each entry's `(byte_offset, bit_position)` is the SM's logical base plus the
   accumulated preceding-entry bits.
5. Validates each SDO-init payload fits an expedited transfer (≤ 4 bytes) — it
   **errors out** otherwise. Also validates **pin-name uniqueness**: pin names
   index the process image globally, so a duplicate `halPin` across slaves is
   rejected at generate time — namespace pins per slave (`drive0-*`, `drive1-*`)
   rather than let a duplicate silently alias the first slave's offset.
6. Renders `generated.rs`: the per-slave `SmCfg`/`FmmuCfg`/`SdoInit`/`DcCfg`
   tables, the `SLAVES` array, the `PINS` array, and the top-level `BUS`.
7. Emits the **host-bridge layout** for the Pi/LinuxCNC SPI bridge from the same
   pass: `src/hal/spi_layout_generated.rs` (the streamed-motion field table) and
   `linuxcnc/teensy_bridge_layout.h` (the Pi HAL frame/pin contract), so the wire
   format cannot drift from either end. See
   [`linuxcnc-spi-bridge.md`](linuxcnc-spi-bridge.md).

Run it through the Makefile (which sets the default input paths):

```sh
make config
# equivalently:
python3 scripts/generate_ethercat_config.py \
    --bus ethercat-conf.bohign.xml \
    --esi Bohign_MS_ECAT_V2.5.xml \
    --out src/ethercat/config/generated.rs
```

It prints a one-line summary, e.g.
`wrote …/generated.rs (2 slave(s), 110-byte image, 34 pins)`.

### Assumptions / limitations to know

- **One PDO per process-data SM.** The image/pin computation uses each SM's first
  PDO. Multiple assigned PDOs per SM would need generator changes.
- **ESI SM ordering.** SM2/SM3 are taken as the 3rd/4th `<Sm>` in the ESI device.
  An ESI that lists SMs out of that order needs adjustment.
- **SDO init ≤ 4 bytes.** Enforced (build-time error). Segmented SDO is out of
  scope.
- **Byte-aligned multi-bit fields.** The HAL pin layer (`src/hal/process_data.rs`)
  assumes multi-bit fields start on a byte boundary (true for the test drive);
  single `bit` pins honour `bit_pos`. Non-byte-aligned multi-bit fields are not
  yet handled.

---

## 3. The config model (`src/ethercat/config/model.rs`)

`generated.rs` is plain `Copy` POD tables of these structs, consumed by the
bring-up FSM, the process-data `domain`, and the HAL pin layer:

| Struct | Holds |
| --- | --- |
| `BusCfg` | `cycle_ns`, `ref_clock_slave`, `slaves`, `pins`, `image_size`. |
| `SlaveCfg` | `position`, `vendor_id`, `product_code`, `sms`, `fmmus`, `dc`, `sdo_init`, `out_size`, `in_size`. |
| `SmCfg` | `index` (2/3), `phys_start`, `control`, `dir`, `size`, `pdos`. |
| `PdoCfg` | `index` (e.g. `0x1600`), `entries` (`&[EcPdoEntryInfo]`). |
| `FmmuCfg` | `logical_start`, `size`, `phys_start`, `dir`. |
| `DcCfg` | `assign_activate`, `sync0_cycle_ns`, `sync0_shift_ns`, `sync1_cycle_ns`. |
| `SdoInit` | `index`, `subindex`, `data` (`&[u8]`, little-endian). |
| `PinCfg` | `name`, `byte_offset`, `bit_pos`, `bit_len`, `hal_type` (`Bit`/`U32`/`S32`), `dir`. |

`BusCfg::pin(name)` looks a pin up by name (used by the HAL layer and the `pd`
command).

---

## 4. The generated table for the two drives

`make config` on the committed XML produces this `BUS` (abridged) for the two
identical YAKO/Bohign drives — a **110-byte image** (`2 × (16 output + 39 input)`
= 32 output bytes + 78 input bytes), a **100 Hz** cycle, **DC SYNC0**, and one SDO
init per drive (`0x6060 = 8`, CSP mode):

```rust
// src/ethercat/config/generated.rs (excerpt; drive0's SM/SDO shown, drive1 identical)
const S0_SMS: &[SmCfg] = &[
    SmCfg { index: 2, phys_start: 0x1200, control: 0x64, dir: EcDirection::Output, size: 16, pdos: S0_RX_PDOS },
    SmCfg { index: 3, phys_start: 0x1300, control: 0x20, dir: EcDirection::Input,  size: 39, pdos: S0_TX_PDOS },
];
// Layout is ALL outputs first, then ALL inputs, so each slave's input FMMU starts
// after BOTH drives' output blocks (drive0 out 0..16, drive1 out 16..32).
const S0_FMMUS: &[FmmuCfg] = &[
    FmmuCfg { logical_start: 0,  size: 16, phys_start: 0x1200, dir: EcDirection::Output },
    FmmuCfg { logical_start: 32, size: 39, phys_start: 0x1300, dir: EcDirection::Input },
];
const S1_FMMUS: &[FmmuCfg] = &[
    FmmuCfg { logical_start: 16, size: 16, phys_start: 0x1200, dir: EcDirection::Output },
    FmmuCfg { logical_start: 71, size: 39, phys_start: 0x1300, dir: EcDirection::Input },
];
const S0_SDO_INIT: &[SdoInit] = &[
    SdoInit { index: 0x6060, subindex: 0x00, data: &[0x08] },  // CSP mode (also S1_SDO_INIT)
];

pub const BUS: BusCfg = BusCfg {
    cycle_ns: 10000000,        // 100 Hz
    ref_clock_slave: 0,
    slaves: SLAVES,            // two SlaveCfg (drive0 + drive1), both vid 0x994 / pid 0x1B00, DC 0x0300
    pins: PINS,                // 34 named pins (17 per drive; see below)
    image_size: 110,
};
```

### The process image and pin map

Outputs occupy bytes `0..32` (drive0 `0..16`, drive1 `16..32`); inputs `32..110`
(drive0 `32..71`, drive1 `71..110`). A few of the 34 generated pins:

| Pin | Dir | Offset | Bits | Type |
| --- | --- | --- | --- | --- |
| `drive0-controlword` | OUT | 0 | 16 | U32 |
| `drive0-target-position` | OUT | 2 | 32 | S32 |
| `drive0-digital-outputs` | OUT | 12 | 32 | U32 |
| `drive1-controlword` | OUT | 16 | 16 | U32 |
| `drive0-error-code` | IN | 32 | 16 | U32 |
| `drive0-statusword` | IN | 34 | 16 | U32 |
| `drive0-actual-position` | IN | 36 | 32 | S32 |
| `drive0-op-mode-display` | IN | 70 | 8 | S32 |
| `drive1-statusword` | IN | 73 | 16 | U32 |

(`pdos` over serial prints the full list with offsets at runtime.)

---

## 5. Retargeting a different drive

The committed mapping is the stock RxPDO1 (`0x1600`) + TxPDO1 (`0x1A00`) layout,
which is identical across the MS/ESD common-layout family. Depending on how
different the new target is:

1. **Same family, different model:** change only the slave's `pid` in
   `ethercat-conf.bohign.xml` (the ESI must contain that product code).
2. **Different mapping:** edit the `<syncManager>/<pdo>/<pdoEntry>` block to match
   the drive's PDOs, and update the `halPin` names.
3. **Different vendor/ESI:** point the generator at the new ESI file.
4. **Change the cycle rate:** set `master @appTimePeriod` (e.g. `250000` for
   4 kHz). With `sync0Cycle="*1"`, SYNC0 follows automatically.
5. **Add slaves:** append more `<slave>` blocks; the image grows and the working
   counter is recomputed (`+3` per drive). **Namespace each slave's `halPin` names**
   (`drive0-*`, `drive1-*`) — a duplicate name is rejected at generate time. (The
   committed two-drive bus is hardware-verified.)

Then regenerate and rebuild:

```sh
make config                 # rewrites src/ethercat/config/generated.rs
# (optionally pass non-default files)
make config ECAT_BUS_XML=my-bus.xml ECAT_ESI_XML=my-vendor.xml
make hex                    # rebuild the firmware
# review `git diff` on generated.rs, then commit it
```

After flashing, `rescan` then `start` (it brings up the whole bus) and confirm
`status` shows `cyclic OP <rate>Hz wkc=…` with the expected working counter (a
slave with SM2 + SM3 contributes `+3`; the two-drive bus → `6/6`). See
[`cli-reference.md`](cli-reference.md) and the
[README quick start](../README.md#quick-start--using-it-as-a-driver).

> The historical brief [`pdo-planning-input.md`](pdo-planning-input.md) has a
> deeper walk-through of the IgH PDO/domain mechanics and an ESC register
> cheat-sheet, if you need to extend the mapping logic itself.
