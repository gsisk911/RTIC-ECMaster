# PDO (Cyclic Process Data) — Planning Input

> **What this document is.** This is the *context brief* for the EtherCAT **PDO
> (cyclic process-data) feature** of the RTIC EtherCAT master firmware. It is
> **not** the implementation plan. It is the self-contained background a later
> *planning* agent will consume to produce the PDO implementation plan.
>
> **What is already done & hardware-verified:** bus **SCAN** and CoE **SDO
> read/write** plus an IgH-`ethercat`-tool-style serial **CLI**.
> **What this feature adds:** cyclic **PDO** exchange (process data), targeted to
> run at **up to 4 kHz (250 µs cycle)** with **DC SYNC0**.
>
> **Hard constraints for the planner (from the user / repo rules):**
> - Do **not** run `main.py`/firmware; this is a Rust `no_std` firmware, the user
>   flashes/runs hardware themselves. Other files may be built/tested.
> - `no_std`, **no heap** — everything is `heapless` / fixed arrays.
> - Pi-5 note in the global rules (`lgpio`) does **not** apply: target is a
>   **Teensy 4.1 (i.MX RT1062, Cortex-M7)**, RTIC 2, raw Layer-2 EtherCAT.
> - Keep files/functions short and single-purpose; use the project config files;
>   prefix scratch/refactor files `temporary_*`.

**Project:** `/Users/griffinsisk/Documents/Documents_Griffins_Work_Mac/github/RTIC-ECMaster`
**Target:** Teensy 4.1 / i.MX RT1062 Cortex-M7 @ **600 MHz** (IPG bus 150 MHz),
RMII ENET (ENET1), EtherType **0x88A4**, RTIC 2 (`thumbv7em-none-eabihf`).
**Design intent:** a file-for-file Rust port of the **IgH EtherCAT Master
(EtherLab) `master/`** core, stable-1.6.

> **Note on prior planning docs:** the task referenced `.cursor/plans/`, but no
> such directory exists in this repo (git has **no commits yet**; `.cursor/plans/`
> is absent). The authoritative record of prior decisions is therefore
> [`docs/ethercat-v1-followups.md`](./ethercat-v1-followups.md) plus the
> **scaffolded module docstrings** in `src/ethercat/` (each empty PDO module
> already states its IgH source file and intended responsibility). Those are
> folded into this brief.

---

## Table of contents

1. [Current state — what the PDO layer builds on](#1-current-state--what-the-pdo-layer-builds-on)
2. [IgH PDO/DOMAIN mechanics to mirror](#2-igh-pdodomain-mechanics-to-mirror)
3. [Config mapping — lcec XML → PDO layer](#3-config-mapping--lcec-xml--pdo-layer)
4. [The 4 kHz / 250 µs constraints](#4-the-4-khz--250-µs-constraints)
5. [Open questions / decisions for the planner](#5-open-questions--decisions-for-the-planner)
- [Appendix A — register & constant cheat-sheet](#appendix-a--register--constant-cheat-sheet-hex)
- [Appendix B — IgH source URLs used](#appendix-b--igh-source-urls-used)

---

## 1. Current state — what the PDO layer builds on

### 1.1 Module layout (flat `ethercat/`, mirrors IgH `master/`)

`src/ethercat/mod.rs` lays the code out as a flat mirror of IgH's `master/`
directory. Kernel-only glue (`module.c`, `cdev.c`, `ioctl.c`) is dropped;
locking becomes RTIC resources / `critical-section`, allocation becomes
`heapless` + fixed arrays, `EC_DBG`/`EC_ERR` become the `log` facade.

```16:45:src/ethercat/mod.rs
#![allow(dead_code)]

// Public API surface + shared constants (include/ecrt.h, master/globals.h)
pub mod ecrt;
pub mod globals;
...
// Discovered runtime model + desired configuration model
pub mod domain;
pub mod fmmu_config;
pub mod mailbox;
pub mod pdo;
pub mod pdo_entry;
pub mod pdo_list;
...
pub mod sync;
pub mod sync_config;
```

**Implemented & hardware-verified today:** `datagram`, `device`, `mailbox`,
`fsm_sii`, `fsm_slave_scan`, `fsm_master` (scan), `fsm_coe` (SDO), `fsm_change`
(AL state), `slave` (`SlaveInfo`), `master` (scan + `poll_op`), `cli`, `globals`,
`ecrt` (API structs + LE helpers).

**Scaffolded-but-empty (PDO needs these — see §1.8):** `domain`, `fmmu_config`,
`pdo`, `pdo_entry`, `pdo_list`, `sync_config`, `slave_config`,
`fsm_slave_config`, `fsm_pdo`, `fsm_pdo_entry`, `dc`, `config/{model,parser}`,
`hal/{pin,process_data}`, `cia402`. `sync` has only the **mailbox** SM helpers.

### 1.2 Transport seam — `Device` + the `Pump` primitive (`device.rs`)

`Device` owns the ENET driver and the source MAC. `send()` wraps an EtherCAT
frame in the 14-byte Ethernet header (broadcast dst, fixed src, `0x88A4`
big-endian) and pads to the 60-byte minimum; `poll()` filters `0x88A4` frames
and copies the EtherCAT payload out.

The key reusable primitive is **`Pump`** — a non-blocking, send-once/poll-once
single-datagram tracker. Every protocol FSM steps it **once per driver tick**, so
no work busy-waits:

```128:141:src/ethercat/device.rs
    /// Non-blocking single-datagram transaction tracker.
    ///
    /// Drives one request/reply without busy-waiting: the first `poll` sends the
    /// (already-built, index-stable) frame; subsequent `poll`s check the RX ring
    /// once each for the matching reply. This is the primitive the protocol FSMs
    /// step once per driver tick (the async worker now, the cyclic PDO task
    /// later), so SDO/state work never stalls the executor or the PDO cycle.
    pub fn pump<'b>(
        &mut self,
        pump: &mut Pump,
        frame: &[u8],
        rx: &'b mut [u8],
        max_attempts: u32,
    ) -> Result<Option<usize>, EcError> {
```

- `transact()` (blocking busy-wait, `cortex_m::asm::delay`) exists **only for the
  startup scan** (pre-OP). The PDO path must **not** use it.
- **Reply matching is by datagram index byte only** (`rx[3]`). Followup #3 in
  `ethercat-v1-followups.md` flags that the index is a wrapping `u8`; once the
  cyclic frame interleaves a background datagram, robust index allocation /
  command-byte matching matters more.
- `ECAT_MTU = 1536`, **`ECAT_RX_LEN = 4`, `ECAT_TX_LEN = 4`** (shallow rings).

### 1.3 Datagram build/parse + the `0x8000` "more datagrams follow" bit (`datagram.rs`)

`build()` writes **exactly one** datagram per frame today. The wire format
already documents the multi-datagram bit — the **length word's bit 15
(`0x8000`)** is the "more EtherCAT datagrams follow" flag, currently hard-zeroed:

```97:100:src/ethercat/datagram.rs
    // Length word: payload length (bits 0..10); bit 15 ("more follows") = 0.
    buf[8..10].copy_from_slice(&((plen as u16) & 0x07FF).to_le_bytes());
    // IRQ word (master writes 0).
    buf[10..12].copy_from_slice(&0u16.to_le_bytes());
```

Frame wire layout (all little-endian except the Ethernet EtherType):

```text
[ EtherCAT frame header (2) | datagram header (10) | payload | WKC (2) | pad ]
  len(0..10)|type=1 (0x1000)   cmd idx ADP ADO len+flags irq
```

Datagram header byte map (offsets relative to EtherCAT frame start):
`[2]=cmd [3]=index [4..6]=ADP [6..8]=ADO [8..10]=len(0..10)+flags(11..15) [10..12]=IRQ`.

`Command` already includes the logical/DC codes the PDO layer needs:
**`Lrd = 0x0A`, `Lwr = 0x0B`, `Lrw = 0x0C`**, `Armw = 0x0D`, `Frmw = 0x0E`.

> **Logical addressing already works through `build()`.** IgH logical commands
> (`LRD/LWR/LRW`) put a **single 32-bit logical address** in the 4-byte address
> field (`EC_WRITE_U32(datagram->address, offset)`). The repo's `build(buf, idx,
> cmd, adp, ado, payload)` writes `adp` at `[4..6]` and `ado` at `[6..8]` little-
> endian — i.e. passing `adp = (offset & 0xFFFF)`, `ado = (offset >> 16)` encodes
> the 32-bit logical address byte-for-byte. The planner can reuse `build()` or add
> a thin `build_logical(buf, idx, cmd, logical_u32, payload)` wrapper for clarity.

`parse()` returns a `Reply { command, index, working_counter, data }`. It masks
`data_len` with `0x07FF` and is bounds-checked (see followup testing gaps).

### 1.4 `Master` + `poll_op` orchestration (`master.rs`)

`Master` owns the `Device`, a `heapless::Vec<SlaveInfo, EC_MAX_SLAVES>`, a
wrapping datagram `index: u8`, and an `Option<Op>`. Runtime requests run as
**non-blocking `Op` steppers**, advanced one datagram per `poll_op()` call:

```164:176:src/ethercat/master.rs
    /// Advance the active operation by one datagram. Returns `None` while
    /// pending, `Some(result)` when it completes (or fails).
    pub fn poll_op(&mut self) -> Option<Result<Outcome, EcError>> {
        let mut op = self.op.take()?;
        match self.drive(&mut op) {
            Ok(None) => {
                self.op = Some(op);
                None
            }
            Ok(Some(outcome)) => Some(Ok(outcome)),
            Err(e) => Some(Err(e)),
        }
    }
```

The **`PreOp` stepper is the template for PDO bring-up.** It already does, one
datagram per `step()`: configure mailbox **SM0** (FPWR `reg::SM0`) → **SM1**
(FPWR `reg::SM1`) → AL state change (`FsmChange`). The PDO bring-up extends this
same shape with SM2/SM3, FMMU, DC, PDO-assignment, SAFE-OP and OP stages.
`PreOp` carries its own `Pump`, `tx: [u8; 64]`, `rx: [u8; 128]`, and uses
`alloc_index()` to take the next index. (The PDO image needs much larger buffers
— see §4.)

`Request`/`Outcome` enums today cover `Rescan`, `SetState`, `SdoUpload`,
`SdoDownload`. PDO adds new variants (e.g. `ConfigurePdo`, `StartCyclic`,
`StopCyclic`) — or a separate cyclic path (see §5).

### 1.5 The CoE FSM (`fsm_coe.rs`) — the reusable SDO stepper

`FsmCoe` is a fully-working, non-blocking **expedited SDO** (≤ 4 bytes)
up/download FSM: `WriteRequest → WaitMailbox → ReadResponse → Done`, one datagram
per `step()` via `Pump`. **PDO assignment (0x1C1x) and PDO mapping
(0x1600/0x1A00) are written as ordinary expedited SDO downloads**, so the PDO-
config FSMs (`fsm_pdo`, `fsm_pdo_entry`) sit directly on top of `FsmCoe`:

- `FsmCoe::download(mbox, index, subindex, &data)` / `FsmCoe::upload(mbox, index, subindex)`.
- Buffers `tx/rx: [u8; 320]`; writes the **full** `rx_size` (mailbox-full only
  latches on the last byte); re-checks `SM1_STATUS` mailbox-full across steps.
- Aborts surface as `EcError::SdoAbort(code)`.
- **Limit:** expedited only (≤ 4 bytes). A PDO mapping entry value is exactly a
  4-byte `u32` (`index<<16 | subindex<<8 | bitlen`), so this is sufficient for
  fixed mapping. Complete-access / segmented mapping is out of scope for v1 PDO.

### 1.6 `SlaveInfo` model (`slave.rs`) + constants (`globals.rs`)

`SlaveInfo` is a `Copy` struct populated by the scan: `ring_pos`, `station_addr`
(= `ring_pos + 1`), `al_state`, base type/FMMU-count/SM-count, vendor/product/
revision, the **RxMailbox/TxMailbox offset+size** (SII 0x0018/0x001A), mailbox
protocols, and `supports_coe`. It exposes `.mailbox() -> Mailbox`. The docstring
notes "the full SM/PDO/SII-category model is added with the configuration
feature" — i.e. **PDO config must extend the model** (per-SM physical start/length
and the SM2/SM3 sync-manager category from SII, or rely entirely on the XML).

`globals.rs` already defines the ESC register map the PDO layer extends:

| Const | Value | Use |
| --- | --- | --- |
| `reg::AL_CONTROL` / `AL_STATUS` / `AL_STATUS_CODE` | `0x0120` / `0x0130` / `0x0134` | state change |
| `reg::SM0` / `SM1` | `0x0800` / `0x0808` | mailbox SM pages |
| `EC_SYNC_PAGE_SIZE` | `8` | bytes per SM page (so **SM2 = 0x0810, SM3 = 0x0818** — not yet defined) |
| `reg::DC_RECV_TIME` / `DC_SYS_TIME` / `DC_SYS_TIME_OFFSET` | `0x0900` / `0x0910` / `0x0920` | DC (SYNC0 regs not yet defined) |
| `EC_MAX_SLAVES` / `EC_MAX_SYNC_MANAGERS` / `EC_MAX_FMMUS` | `32` / `16` / `16` | fixed caps |
| `EC_MAX_PDOS` / `EC_MAX_PDO_ENTRIES` | `32` / `32` | fixed caps |
| `al_state::{INIT,PREOP,SAFEOP,OP}` | `0x01/0x02/0x04/0x08` | + `ERROR=0x10`, `MASK=0x0F` |

**Gaps the PDO layer must add to `globals.rs`:** `SM2`/`SM3` (0x0810/0x0818),
FMMU register base (**0x0600**, 16 bytes each), DC activation **0x0980**, DC
SYNC0/1 cycle **0x09A0**, DC cyclic start **0x0990**, DC sys-time-diff **0x092C**,
watchdog divider **0x0400** / process-data watchdog **0x0420**, process-data SM
control bytes (0x64 out / 0x20 in), and CoE objects 0x1C12/0x1C13/0x1600/0x1A00.

### 1.7 RTIC app + the `ethercat_worker` task (`main.rs`)

- **Monotonic:** `systick_monotonic!(Mono, 1_000)` — **1 kHz SysTick**. This is
  the coarse timer the 250 µs cycle cannot use (§4.1).
- **Dispatchers:** `#[rtic::app(device = teensy4_bsp, dispatchers = [GPIO6_7_8_9, LPUART8, GPT1])]`.
  ⚠️ **`GPT1` is currently consumed as a software-task dispatcher** — relevant to
  the GPT-vs-PIT timer decision (§4.1/§5).
- **The single owner of the device:** the async `ethercat_worker` task
  (priority 1, `local = [ecat_master]`) owns the `Master` exclusively. It does the
  initial blocking scan, then loops taking CLI commands and driving them with
  `poll_op` (yielding `Mono::delay(ECAT_STEP_MS=1ms)` between datagrams):

```1120:1172:src/main.rs
    #[task(shared = [ecat_scan, ecat_cmd, ecat_out], local = [ecat_master], priority = 1)]
    async fn ethercat_worker(mut cx: ethercat_worker::Context) {
        let master = &mut cx.local.ecat_master.0;
        ...
        // Command loop: take a parsed command, execute its non-blocking FSM to
        // completion (yielding between datagrams), and publish the response.
        loop {
            let cmd = cx.shared.ecat_cmd.lock(|slot| slot.take());
            match cmd {
                Some(cmd) => {
                    let lines = ecat_run_command(master, cmd).await;
                    cx.shared.ecat_out.lock(|o| o.set(lines));
                }
                None => Mono::delay(ECAT_IDLE_MS.millis()).await,
            }
        }
    }
```

- Other priority-1 async tasks: `blink_leds`, `poll_w5500` (disabled).
  `usb_isr` (priority 2, binds `USB_OTG1`) drives the serial CLI + reporting.
- The master holds `&'static mut` ENET descriptor tables (raw pointers, `!Send`);
  it lives in a `Send` wrapper `EcatMasterCell` justified by single-core ownership.
- **Constraint for PDO:** exactly one task may own the `Device`. The cyclic PDO
  task and the background FSM stepping (SDO/state/PDO-config) must be driven from
  **the same owner** (§4.2, §5).

### 1.8 Scaffolded-but-empty modules the PDO layer fills

Each is a docstring-only stub (`// TODO: ...`) that already names its IgH source
and responsibility. The planner implements these:

| File | IgH source | Responsibility |
| --- | --- | --- |
| `domain.rs` | `master/domain.c` | Process-data image (absorbs old `pdi.rs`), FMMU configs, **(byte offset, bit pos)** entry map, builds the cyclic LRW/LRD/LWR. |
| `fmmu_config.rs` | `master/fmmu_config.c` | One FMMU mapping (logical↔physical) + its **16-byte ESC page**. |
| `pdo.rs` | `master/pdo.c` | One PDO (0x1600/0x1A00) + its entry list (`heapless::Vec<_, EC_MAX_PDO_ENTRIES>`). |
| `pdo_entry.rs` | `master/pdo_entry.c` | One mapped entry: index, subindex, bit length. |
| `pdo_list.rs` | `master/pdo_list.c` | Ordered PDOs per SM (`heapless::Vec<_, EC_MAX_PDOS>`). |
| `sync_config.rs` | `master/sync_config.c` | Desired SM PDO assignment (dir + PDO list) → `ecrt::EcSyncInfo`. |
| `slave_config.rs` | `master/slave_config.c` | Desired slave config: expected id, SM/PDO, DC, watchdog, SDO init. |
| `sync.rs` (extend) | `master/sync.c` | Add **process-data** SM page helpers (has mailbox ones only). |
| `fsm_slave_config.rs` | `master/fsm_slave_config.c` | Bring-up INIT→PREOP→SAFEOP→OP (`State` enum already declared). |
| `fsm_pdo.rs` | `master/fsm_pdo.c` | PDO **assignment** (0x1C1x) over CoE (`State` enum declared). |
| `fsm_pdo_entry.rs` | `master/fsm_pdo_entry.c` | PDO **mapping** (0x1600/0x1A00) over CoE (`State` enum declared). |
| `dc.rs` | scattered in IgH | DC SYNC0/1 cycle+shift, `assignActivate` (0x0980), reference clock. |
| `config/model.rs` + `config/parser.rs` | (our addition) | `ethercat-conf.xml` model + parser. |
| `hal/pin.rs` + `hal/process_data.rs` | (our addition) | Named-pin layer over the image. |
| `cia402.rs` | (not IgH core) | CiA 402 controlword/statusword drive FSM (app layer). |

`fsm_slave_config.rs` already declares the exact IgH state progression:

```11:27:src/ethercat/fsm_slave_config.rs
/// Configuration states, mirroring the IgH progression.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Init,
    ClearFmmus,
    DcClear,
    MboxSync,
    Preop,
    SdoConf,
    PdoConf,
    Fmmu,
    DcCycle,
    Safeop,
    Op,
    End,
    Error,
}
```

The public API structs the PDO layer fills in are already defined in `ecrt.rs`:
`EcDirection {Output, Input}`, `EcPdoEntryInfo {index, subindex, bit_length}`,
`EcPdoInfo {index, n_entries}`, `EcSyncInfo {index, dir, n_pdos}`, and the
resolved-location type **`EcPdoEntryReg { byte_offset: u32, bit_position: u8 }`**,
plus the `read_/write_*_le` helpers (the `EC_READ_*`/`EC_WRITE_*` replacements).

### 1.9 HAL named-pin layer (`hal/`)

`hal/mod.rs` describes binding `halPin` names from `ethercat-conf.xml` (e.g.
`die-cylinder-statusword`) to typed, directional pins over `ethercat::pdi`
(the process image, which now lives in `domain.rs`). `hal/pin.rs` (HalPin +
`HalType` bit/u32/s32 + direction) and `hal/process_data.rs` (offset binding +
typed read/write) are TODO. This is **where `halType`/`halPin` meet the process-
image offsets** (§3.5).

### 1.10 ENET transport timing facts (`net/enet_driver.rs`)

- Raw L2 path: `send_raw(&[u8]) -> bool` (false if the next TX descriptor is still
  DMA-owned) and `poll_raw(|frame| ...) -> Option<R>`. RMII, **promiscuous**,
  CRC-forward, full-duplex, store-and-forward TX. 100 Mbit/s EtherCAT line rate.
- **Rings are only 4 deep** (`ECAT_RX_LEN/TX_LEN = 4`) and **only `atomic::fence`**
  is used (no SCB cache maintenance) — see the cache-coherency caveat in §4.6 and
  followup #2: the descriptor tables/buffers **must** live in non-cached memory.

---

## 2. IgH PDO/DOMAIN mechanics to mirror

All file references below are **IgH EtherCAT Master, branch `stable-1.6`**
(URLs in Appendix B). Quoted hex/field layouts were read directly from those
sources.

### 2.1 The process-data DOMAIN image (`master/domain.c`)

A **domain** aggregates the registered PDO entries across slaves, owns one
**contiguous process-data image** (`domain->data`, size `domain->data_size`), and
builds the cyclic datagram(s).

- **Image growth:** `ec_domain_add_fmmu_config()` simply does
  `domain->data_size += fmmu->data_size;` and appends the FMMU. So the image is
  the **concatenation of each registered FMMU's region, in registration order**.
- **`ec_domain_finish(domain, base_address)`** allocates the image, then walks the
  FMMU list and:
  1. corrects each FMMU's `logical_start_address += base_address` (the master
     gives the domain a 32-bit logical base, default `0x00000000`);
  2. packs FMMUs into **datagram pairs** of up to **`EC_MAX_DATA_SIZE`** bytes
     each (≈ 1486 = 1500 − 2 frame hdr − 10 datagram hdr − 2 WKC). If everything
     fits (it does here — see §3.6 totals), there is **one** cyclic datagram.
- **Cyclic API (per cycle):**
  - `ecrt_domain_queue(domain)` → enqueues the domain's datagram(s) for sending
    (`ec_master_queue_datagram`).
  - `ecrt_domain_process(domain)` → after receive, sums each datagram's working
    counter into `domain->working_counter`, compares to
    `domain->expected_working_counter`, sets `EC_WC_ZERO/INCOMPLETE/COMPLETE`.
- The Rust port replaces the kmalloc'd image with a **fixed `[u8; N]`** and the
  intrusive FMMU list with a `heapless::Vec`. (Redundancy / multi-device code in
  `domain.c` is **out of scope** — single NIC.)

### 2.2 FMMU config + the 16-byte ESC page (`master/fmmu_config.c`)

An **FMMU** maps a slave's sync-manager region between the **logical** (domain
image) address space and the slave's **physical** SM address. IgH creates **one
FMMU per slave per direction** that covers that SM's whole region:

- `ec_fmmu_config_init()`: `fmmu->logical_start_address = domain->data_size;`
  (the running image offset), `fmmu->data_size = total mapped PDO bytes for that
  SM direction`, then `ec_domain_add_fmmu_config()`.
- **`ec_fmmu_config_page()` — the 16-byte page written to ESC FMMU register
  `0x0600 + n*16`** (read verbatim from source):

```text
data[0..4]   = logical_start_address      (U32)
data[4..6]   = data_size (bytes)          (U16)
data[6]      = 0x00   logical start bit
data[7]      = 0x07   logical end bit       (whole bytes → bits 0..7)
data[8..10]  = sync->physical_start_address (U16)   // the SM's phys start
data[10]     = 0x00   physical start bit
data[11]     = dir: 0x01 = INPUT (read, slave→master), 0x02 = OUTPUT (write)
data[12..14] = 0x0001 enable
data[14..16] = 0x0000 reserved
```

So an **input** FMMU (TxPDO / SM3) has `dir=0x01`; an **output** FMMU (RxPDO /
SM2) has `dir=0x02`.

### 2.3 Cyclic datagrams LRW/LRD/LWR + logical addressing (`master/datagram.c`)

Logical commands carry a **single 32-bit logical address** in the 4-byte address
field — `ec_datagram_lrw/lrd/lwr(datagram, offset, size)` all do
`EC_WRITE_U32(datagram->address, offset)`. Which command is used depends on what
the datagram covers (`datagram_pair.c`, §2.4):

| Datagram covers | Command | Why |
| --- | --- | --- |
| outputs **and** inputs | **`LRW`** (0x0C) | one datagram both writes outputs and reads inputs in the same logical region |
| outputs only | `LWR` (0x0B) | master→slaves |
| inputs only | `LRD` (0x0A) | slaves→master |

For the repo's bus (every drive has both SM2 and SM3), the cyclic frame is a
**single LRW** over the whole image. The repo's `datagram::build` already encodes
this (pass `adp = offset & 0xFFFF`, `ado = offset >> 16`).

### 2.4 Working counter for the cyclic datagram (`master/datagram_pair.c`)

`ec_datagram_pair_init()` sets the **expected working counter** by counting how
many slave-configs (FMMUs) of each direction share the datagram:

```text
LRW (both): expected_WC = used[OUTPUT] * 2 + used[INPUT]
LWR (out) : expected_WC = used[OUTPUT]
LRD (in)  : expected_WC = used[INPUT]
```

i.e. for an **LRW**, each slave that **reads** the frame's output region adds **+2**
and each slave that **writes** its input region adds **+1**. A slave with both
(SM2+SM3) contributes **+3**. **This is the master's primary per-cycle health
check** — `WC == expected` ⇒ all slaves exchanged data this cycle; a drop ⇒
`INCOMPLETE`/`ZERO`. (Worked number for the repo's 8-slave bus: §3.6 → **WC = 24**.)

### 2.5 Entry → `(byte_offset, bit_position)` resolution (`master/slave_config.c`)

This is the exact algorithm behind `ecrt_domain_reg_pdo_entry_list` (which loops
calling `ecrt_slave_config_reg_pdo_entry` per entry, storing each returned
`offset` and `bit_position`). Read verbatim, `ecrt_slave_config_reg_pdo_entry`:

```text
for each sync manager (sync_index):
    bit_offset = 0
    for each assigned PDO in sync_config:
        for each entry in PDO:
            if entry != (index, subindex):
                bit_offset += entry->bit_length      # accumulate
            else:
                bit_pos    = bit_offset % 8
                sync_offset = ec_slave_config_prepare_fmmu(sc, domain, sync_index, dir)
                            # = the SM's logical byte base in the image
                            #   (creates the FMMU on first touch; returns its
                            #    logical_start_address)
                return  sync_offset + bit_offset / 8     # the BYTE offset
                # *bit_position = bit_pos
```

**Result per entry:** `byte_offset = sm_logical_base + (Σ preceding entry bits)/8`,
`bit_position = (Σ preceding entry bits) % 8`. Multi-bit entries that don't
byte-align are allowed only if the caller accepts a `bit_position` (else IgH
errors "does not byte-align"). `ec_slave_config_prepare_fmmu()` is idempotent per
(domain, sync_index): first touch allocates the FMMU (growing the image by the
SM's total size); later entries in the same SM reuse that base.

> This maps onto the repo's `ecrt::EcPdoEntryReg { byte_offset, bit_position }`.
> The HAL/`cia402` layers then read/write each named pin at
> `image[byte_offset]` with `bit_position`/`bit_length`.

### 2.6 Process-data sync managers SM2/SM3 (`master/sync.c`)

The PDO SM pages are written during bring-up (FPWR to **0x0810 / 0x0818**).
`ec_sync_page()` builds the **8-byte** page:

```text
data[0..2] = physical_start_address  (U16)
data[2..4] = data_size (bytes)       (U16)   // total mapped PDO bytes for the SM
data[4]    = control byte            (U8)
data[5]    = 0x00 (status, read-only)
data[6..8] = enable                  (U16)   // 0x0001 when size>0 & PDOs xfer
```

The control byte starts from the **SII SM-category** `control_register` and IgH
forces the direction bits: `control bit2 = (dir==OUTPUT?1:0)`, `bit3 = 0`, and
`bit6 = watchdog enable`. Direction is read back as
`(control & 0x0C) >> 2`: `0x0 → INPUT`, `0x1 → OUTPUT`. **Typical concrete values
for CiA 402 drives: SM2 (outputs) = `0x64`, SM3 (inputs) = `0x20`.** (IgH derives
the base from SII; a fixed-config v1 may hardcode `0x64`/`0x20`.)

### 2.7 PDO **assignment** (0x1C1x) + **mapping** (0x1600/0x1A00) over CoE

(`master/fsm_pdo.c`, `master/fsm_pdo_entry.c` — both expedited SDOs over `FsmCoe`.)

**Assignment object = `0x1C10 + sync_index`** → **SM2 = `0x1C12`, SM3 = `0x1C13`**.
Subindex 0 = count; subindex N = the assigned PDO index. Reconfigure sequence:

```text
1. (read)   SDO upload 0x1C1x:00  → current PDO count
2. (clear)  SDO download 0x1C1x:00 = 0
3. for each PDO p:  SDO download 0x1C1x:p = pdo_index   (e.g. 0x1600)
            and configure that PDO's mapping (below)
4. (commit) SDO download 0x1C1x:00 = count
```

**Mapping object = the PDO index itself (0x1600.. / 0x1A00..)**. Subindex 0 =
entry count; subindex N = a packed **u32**:

```text
entry_u32 = (index << 16) | (subindex << 8) | bit_length

1. SDO download 0x1600:00 = 0
2. for each entry e:  SDO download 0x1600:e = entry_u32
3. SDO download 0x1600:00 = count
```

Example (slave 0, `0x6040:00`, 16-bit ctrlword) → `0x60400010`. Each of these is a
4-byte expedited SDO download — directly supported by `FsmCoe::download`.

> **v1 simplification:** if a slave's default mapping already matches the XML, the
> mapping write can be skipped and only the **assignment** confirmed. The planner
> decides whether to always rewrite or to read-compare-then-write (IgH compares).

### 2.8 Bring-up FSM order + SAFE-OP→OP gating + DC registers (`master/fsm_slave_config.c`)

The verbatim IgH state order (maps onto `fsm_slave_config::State`), with the
datagram each stage issues:

```text
start → init (→ request/confirm INIT/PREOP transition)
  → clear_fmmus      FPWR 0x0600, (clear all FMMU pages)
  → clear_sync       FPWR 0x0800, zero all SM pages
  → dc_clear_assign  FPWR 0x0980 = 0x0000      (disable DC activation first)
  → mbox_sync        FPWR 0x0800 (SM0/SM1 mailbox pages)         [already done by PreOp]
  → (assign_pdi / boot_preop → reach PREOP)
  → sdo_conf         CoE SDO downloads of the XML <sdoConfig> init values
  → pdo_conf         fsm_pdo: assignment 0x1C1x + mapping 0x16xx/0x1Axx  (§2.7)
  → watchdog_divider FPWR 0x0400 = divider
  → watchdog         FPWR 0x0420 = intervals  (process-data watchdog time)
  → pdo_sync         FPWR 0x0810/0x0818 (SM2/SM3 process-data pages, §2.6)
  → fmmu             FPWR 0x0600+n*16 (FMMU pages, §2.2)
  → dc_cycle         FPWR 0x09A0 (8B): U32 sync0_cycle, U32 sync1_cycle
  → dc_sync_check    FPRD 0x092C (4B) system-time-difference, wait < 10 µs drift
  → dc_start         FPWR 0x0990 (8B U64): cyclic start time = app_time + 100 ms, cycle-aligned
  → dc_assign        FPWR 0x0980 = assignActivate (e.g. 0x0300)   (enable SYNC0)
  → wait_safeop / safeop  (→ reach SAFE-OP; slave checks SM2/SM3+FMMU+inputs valid)
  → op               (→ reach OP)  → end
```

**SAFE-OP→OP gating — the critical ordering rule.** A slave refuses **SAFE-OP**
until SM2/SM3 + FMMUs are configured and (with DC) SYNC0 is running; it refuses
**OP** until it is **receiving valid process-data frames with a good working
counter**. Therefore the master must already be **cyclically sending the LRW**
(outputs populated, ideally CiA 402 controlword walking the drive toward
operation) **before/while** requesting OP. Requesting OP with no process data, a
zero WC, or stalled SYNC0 yields an AL status code at `0x0134`
(`EcError::StateChange(code)`). DC drift must be `< EC_DC_MAX_SYNC_DIFF_NS`
(10 µs) before `dc_start`.

> Today `fsm_change.rs` requests the target AL state **directly**. EtherCAT only
> allows **single-step** transitions (INIT→PREOP→SAFEOP→OP); `ethercat-v1-
> followups.md` already flags this. The PDO bring-up must **step** through the
> intermediate states (this is exactly what `fsm_slave_config` does).

---

## 3. Config mapping — lcec XML → PDO layer

### 3.1 The schema and the repo's `ethercat-conf.xml`

`ethercat-conf.xml` is **LinuxCNC "lcec"** format. The repo file defines **one
master, 8 slaves** (`vid/pid` per slave; `configPdos="true"`):

```text
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="1" refClockSlaveIdx="0">
    <slave idx="0" type="generic" vid="0x000000B7" pid="0x000002C1" configPdos="true" name="0">
      <dcConf assignActivate="0x0300" sync0Cycle="*1" sync0Shift="0"/>
      <syncManager idx="2" dir="out"> <pdo idx="1600"> <pdoEntry .../> ... </pdo> </syncManager>
      <syncManager idx="3" dir="in">  <pdo idx="1a00"> <pdoEntry .../> ... </pdo> </syncManager>
    </slave>
    ... slaves 1..7 ...
```

Slave roster (real data, useful for worked examples):

| idx | vid | pid | role / `halPin` prefix | profile |
| --- | --- | --- | --- | --- |
| 0 | `0x000000B7` | `0x000002C1` | `die-cylinder-*` | CiA 402 CSP servo |
| 1 | `0x000000B7` | `0x000002C1` | `shuttle-*` (+ 3 `<sdoConfig>`) | CiA 402 CSP servo |
| 2 | `0x000000B7` | `0x000002C1` | `die-st-in-*` | CiA 402 CSP servo |
| 3 | `0x000000B7` | `0x000002C1` | `die-st-out-*` | CiA 402 CSP servo |
| 4 | `0xE0000002` | `0x00000100` | `do5..do8 / di1..di4` | **digital I/O** (bit-packed) |
| 5 | `0x0000002F` | `0x00020000` | `unwind-*` (+ 3 `<sdoConfig>`) | CiA 402 CSV/CSP servo |
| 6 | `0x0000002F` | `0x00020000` | `rewind-*` (+ 3 `<sdoConfig>`) | CiA 402 CSV/CSP servo |
| 7 | `0x0000002F` | `0x00020000` | `waste-*` (+ 3 `<sdoConfig>`) | CiA 402 CSV/CSP servo |

> The **physically connected drive for bring-up is a YAKO ESD2505PE (CiA 402)** —
> treat it as one of the CSP servos above; the controlword/statusword/position
> objects below are exactly what it exchanges.

### 3.2 `syncManager` / `pdo` / `pdoEntry` → `ecrt_slave_config_pdos`

The nesting maps directly onto the IgH config model the PDO layer builds:

| lcec XML | PDO-layer target (mirrors IgH) |
| --- | --- |
| `<syncManager idx="2" dir="out">` | `EcSyncInfo { index: 2, dir: Output, .. }` → SM2/`0x1C12`, control `0x64` |
| `<syncManager idx="3" dir="in">` | `EcSyncInfo { index: 3, dir: Input, .. }` → SM3/`0x1C13`, control `0x20` |
| `<pdo idx="1600">` | `EcPdoInfo { index: 0x1600, .. }` assigned to SM2 (`ecrt_slave_config_pdo_assign_add`) |
| `<pdoEntry idx="6040" subIdx="00" bitLen="16">` | `EcPdoEntryInfo { index: 0x6040, subindex: 0, bit_length: 16 }` mapping entry (`pdo_mapping_add`) |

The aggregate per slave is IgH's `ec_sync_info_t[]` passed to
`ecrt_slave_config_pdos` (assign all PDOs + their mappings), then each entry is
registered into the domain (`reg_pdo_entry`, §2.5) to obtain its offset.

### 3.3 `<sdoConfig>` → SDO init values (`ecrt_slave_config_sdo`)

```text
<sdoConfig idx="0x607D" subIdx="0x01"> <sdoDataRaw data="60 79 FE FF"/> </sdoConfig>
```

= "before going operational, SDO-download the **little-endian** byte string to
`0x607D:01`". Example decodes: `60 79 FE FF` = `0xFFFE7960` = **−100000** (CiA 402
min software position limit); slave 1's `A0 86 01 00` = `0x000186A0` = **+100000**
(max). These run in the **`sdo_conf`** bring-up stage (§2.8), in XML order, via
`FsmCoe::download`. **Caveat:** several `<sdoDataRaw>` are 4 bytes (fits expedited)
but the schema allows longer strings — anything > 4 bytes needs non-expedited SDO
(out of scope; v1 should validate ≤ 4 or document the limit).

### 3.4 `<dcConf>` → `ecrt_slave_config_dc` (assignActivate, `sync0Cycle="*N"`)

```text
<dcConf assignActivate="0x0300" sync0Cycle="*1" sync0Shift="0"/>
```

| lcec attr | Meaning | → register (§2.8) |
| --- | --- | --- |
| `assignActivate` | DC activation word, **vendor-specific** (0x0300 = enable SYNC0; 0x0730 = SYNC0+SYNC1) | FPWR **0x0980** |
| `sync0Cycle="*N"` | `N × appTimePeriod` ns. Here `*1 × 1000000 = 1,000,000 ns = 1 ms`. A literal (e.g. `250000`) = ns. | U32 @ **0x09A0** |
| `sync0Shift` | SYNC0 shift time (ns) | shift in `dc_start` calc |
| `sync1Cycle` (opt) | second pulse; typically `appTimePeriod − sync0Cycle` | U32 @ **0x09A4** |

> **The repo XML is currently set for 1 kHz** (`appTimePeriod=1000000`,
> `sync0Cycle="*1"`). **For the 4 kHz / 250 µs target, `appTimePeriod` becomes
> `250000`** (and `sync0Cycle="*1"` then yields 250 µs SYNC0). The planner should
> treat appTimePeriod as the cycle source of truth.

### 3.5 `halType` / `halPin` → process-image offsets (the HAL layer, `src/hal/`)

Each `<pdoEntry>` with a `halPin` becomes a **named, typed, directional pin** bound
to a `(byte_offset, bit_position, bit_length)` in the domain image (resolved by
§2.5). `halType` ∈ {`bit`, `u32`, `s32`} chooses the access width:

| `halType` | Image access | Notes |
| --- | --- | --- |
| `bit` | 1 bit at `(byte_offset, bit_position)` | digital I/O channels (slave 4) |
| `u32` | unsigned, `bit_length` ∈ {8,16,32} | statusword/error/digital-inputs; read/written zero-extended |
| `s32` | signed, sign-extended from `bit_length` | position/velocity/torque (often 16 or 32 bit) |

`halType` is the **HAL representation width**, independent of the wire
`bitLen` — e.g. `<... bitLen="16" halType="u32">` is a 16-bit wire field surfaced
as a 32-bit HAL value. Padding entries have **`idx="0000"` and no `halPin`** (e.g.
slave 4's `<pdoEntry idx="0000" subIdx="00" bitLen="8" halType="u32"/>`): they
**consume bits in the image but bind no pin** (the resolver must still advance
`bit_offset`).

### 3.6 Worked example A — slave 0 `die-cylinder` (CiA 402 CSP), full offset map

Computed with the §2.5 algorithm, assuming this slave's SM2 region starts at image
byte `O` and SM3 region at byte `I` (the actual bases come from registration
order; the **intra-SM** layout below is exact).

**SM2 / RxPDO `0x1600` (outputs, master→slave), 9 bytes total:**

| entry | object | bitLen | byte off (in SM2) | halType / halPin |
| --- | --- | --- | --- | --- |
| 0 | `0x6040:00` controlword | 16 | `O+0` | u32 `die-cylinder-ctrlword` |
| 1 | `0x607A:00` target position | 32 | `O+2` | s32 `die-cylinder-target-position` |
| 2 | `0x6060:00` modes of operation | 8 | `O+6` | s32 `die-cylinder-op-mode` |
| 3 | `0x60B8:00` touch-probe function | 16 | `O+7` | u32 `die-cylinder-touch-probe-function` |

**SM3 / TxPDO `0x1A00` (inputs, slave→master), 31 bytes total:**

| entry | object | bitLen | byte off (in SM3) | halType / halPin |
| --- | --- | --- | --- | --- |
| 0 | `0x6041:00` statusword | 16 | `I+0` | u32 `die-cylinder-statusword` |
| 1 | `0x6064:00` position actual | 32 | `I+2` | s32 `die-cylinder-actual-position` |
| 2 | `0x606C:00` velocity actual | 32 | `I+6` | s32 `die-cylinder-actual-velocity` |
| 3 | `0x6077:00` torque actual | 16 | `I+10` | s32 `die-cylinder-actual-torque` |
| 4 | `0x60B9:00` touch-probe status | 16 | `I+12` | u32 |
| 5 | `0x60BC:00` touch-probe pos 2 | 32 | `I+14` | s32 |
| 6 | `0x60FD:00` digital inputs | 32 | `I+18` | u32 |
| 7 | `0x4020:01` dip-in state | 32 | `I+22` | u32 |
| 8 | `0x603F:00` error code | 16 | `I+26` | u32 |
| 9 | `0x6061:00` op-mode display | 8 | `I+28` | s32 |
| 10 | `0x4007:01` board temperature | 16 | `I+29` | u32 |

**Whole-bus image size (sum of all 8 slaves' SM2+SM3):**

| | s0 | s1 | s2 | s3 | s4(IO) | s5 | s6 | s7 | total |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| out (B) | 9 | 13 | 7 | 13 | 8 | 17 | 17 | 17 | **101** |
| in (B) | 31 | 25 | 25 | 25 | 24 | 25 | 25 | 25 | **205** |

→ **≈ 306-byte domain image**, comfortably **one LRW** (< `EC_MAX_DATA_SIZE`
≈ 1486). Expected working counter = `8 inputs × 1 + 8 outputs × 2` = **24**.

### 3.7 Worked example B — slave 4 digital I/O (bit packing)

Demonstrates `bit_position` ≠ 0. SM2 RxPDO `0x1604` packs 8 single-bit outputs +
an 8-bit pad:

```text
<pdoEntry idx="7040" subIdx="01" bitLen="1" halType="bit" halPin="do5-ch1"/>  → byte b+0, bit 0
<pdoEntry idx="7040" subIdx="02" bitLen="1" halType="bit" halPin="do5-ch2"/>  → byte b+0, bit 1
... (ch3..ch8) ...                                                           → byte b+0, bits 2..7
<pdoEntry idx="0000" subIdx="00" bitLen="8" halType="u32"/>                   → byte b+1 (pad, no pin)
```

So `do5-ch1` resolves to `(byte_offset=b, bit_position=0, bit_length=1)`. The HAL
`bit` accessor must mask `1 << bit_position` at `image[byte_offset]`. SM3's
`<pdoEntry idx="6002" subIdx="03" bitLen="14" ...>` shows a **14-bit** field that
does **not** byte-align — the resolver must carry the bit offset across it.

### 3.8 CiA 402 object reference (for the YAKO ESD2505PE and the CSP servos)

Standard CiA 402 objects appearing in the XML (and on the YAKO drive):

| object | dir | meaning | typical PDO |
| --- | --- | --- | --- |
| `0x6040:00` controlword | out | drive state machine command (16-bit) | RxPDO 0x1600 |
| `0x6041:00` statusword | in | drive state machine status (16-bit) | TxPDO 0x1A00 |
| `0x6060:00` modes of operation | out | 8 = CSP, 9 = CSV, 10 = CST | RxPDO |
| `0x6061:00` modes of operation display | in | echo of active mode | TxPDO |
| `0x607A:00` target position | out | CSP setpoint (s32) | RxPDO |
| `0x6064:00` position actual value | in | feedback (s32) | TxPDO |
| `0x60FF:00` target velocity | out | CSV setpoint (slaves 5–7) | RxPDO |
| `0x606C:00` velocity actual value | in | feedback (s32) | TxPDO |
| `0x6071:00` target torque / `0x6077:00` torque actual | out/in | CST / monitoring | Rx/Tx |

CiA 402 state walk (drive `cia402.rs` writes controlword, reads statusword each
cycle): *Switch-On-Disabled → Ready-To-Switch-On (0x06) → Switched-On (0x07) →
Operation-Enabled (0x0F)*, with fault reset (0x80). This is the **valid output
data** the slave needs before it will hold **OP** (§2.8).

---

## 4. The 4 kHz / 250 µs constraints

> These were already identified in `ethercat-v1-followups.md` ("when moving to
> cyclic operation, convert the FSMs to the non-blocking IgH stepping model …
> driven by a monotonic-timed task") and in the scaffolding. They are firm
> requirements, expanded here.

### 4.1 A hardware timer (GPT/PIT) is required — `Mono` (1 kHz SysTick) is too coarse

`systick_monotonic!(Mono, 1_000)` ticks at **1 ms**; the cycle is **250 µs**. The
cyclic task must be driven by a **hardware timer**:

- **PIT** (Periodic Interrupt Timer, on the **75 MHz** perclk): 4 channels,
  free-running, very low jitter, simple periodic IRQ — natural fit for a fixed
  250 µs tick. Not currently used by the app.
- **GPT** (General Purpose Timer): higher-resolution, supports compare/capture and
  is usable as an `rtic-monotonics` source. **But `GPT1` is already consumed as an
  RTIC software-task dispatcher** in `#[rtic::app(... dispatchers = [GPIO6_7_8_9,
  LPUART8, GPT1])]`, so the planner must use **GPT2** (or PIT), or free GPT1 by
  swapping in another unused IRQ as a dispatcher.

The 250 µs timer ISR should run at a **priority above** the priority-1 worker so
the cycle is not delayed by background work, but the **device is still owned by
one context** (§4.2). Decision deferred to §5.

### 4.2 Multi-datagram framing — one frame per cycle (`0x8000` bit)

To keep **one EtherCAT frame per cycle** while still making progress on background
mailbox/SDO/state work, pack **two datagrams into one frame**:

```text
[ frame hdr | LRW (process data) [len.bit15 = 1] | FPRD/FPWR (1 bg mailbox/SDO) [len.bit15 = 0] | WKC | WKC ]
```

- The **first** datagram (the PDO **LRW**) sets the length-word **`0x8000`
  ("more datagrams follow")** bit; the trailing background datagram clears it.
- `datagram.rs::build` is currently single-datagram and hard-zeros bit 15
  (§1.3). The planner adds a **multi-datagram builder** (append datagrams,
  set/clear the "more" bit, two WKC footers) and a **multi-datagram parser**
  (walk datagrams via each length word until "more" = 0, match each by index).
- This lets the existing **`FsmCoe`/`PreOp`/state FSMs run "in the background"**
  one datagram per cycle, interleaved with PDO, **without a second frame** — so
  SDO reads, watchdog pokes, and slow state changes don't cost a whole extra
  round trip at 4 kHz. (IgH does exactly this: the cyclic domain datagram(s) plus
  queued "non-application" datagrams share frames.)

### 4.3 Bounded, allocation-free `step()`

Every per-cycle code path must be **O(1)-ish and heap-free** (already true of the
`Pump`/`FsmCoe`/`PreOp` model). For the cycle:

- Pre-build/own all buffers: the **process image `[u8; N]`**, the cyclic TX frame
  buffer, and per-FSM `tx`/`rx` scratch (note `PreOp` uses `[u8;64]`/`[u8;128]`
  — **too small for a ~306-byte LRW**; the cyclic path needs an image-sized TX
  buffer, ~`14 + 12 + 306 + 2 ≈ 334`+ bytes, plus RX).
- No `heapless` growth, no formatting, no logging on the hot path.
- The cyclic step is strictly: *copy outputs into image → build/queue LRW (+bg) →
  send → (next cycle) poll/parse → `domain_process` (WC) → copy inputs out*.
  IgH's **send-this-cycle / process-last-cycle** pipelining avoids blocking on the
  round trip within a cycle.

### 4.4 DC SYNC0 at 250 µs

`<dcConf>` drives SYNC0 (§2.8/§3.4). For 250 µs: `appTimePeriod = 250000`,
`assignActivate = 0x0300`, SYNC0 cycle 250000 ns. The master's **app-time / cyclic
start-time** math (FPWR 0x0990, start = `app_time + 100 ms`, cycle-aligned) and the
**drift check** (FPRD 0x092C < 10 µs before going cyclic) must be ported into
`dc.rs`. The Teensy ENET also has IEEE-1588 timestamping enabled (`EN1588`) which
*could* feed app-time, but a free-running monotonic app-time counter is the
simpler v1 reference.

### 4.5 Timing budget & jitter/determinism

At 100 Mbit/s, ~306 data bytes + headers ≈ a **~350-byte frame ≈ 28 µs serialize**
one way; round trip through 8 slaves (propagation + per-slave processing) is still
well under **250 µs**, leaving margin for the interleaved background datagram.
Determinism risks to call out for the planner:

- The **4-deep TX/RX rings** and `send_raw` returning `false` when DMA-busy — at
  4 kHz the cycle must confirm the previous frame drained; a missed/aliased reply
  must degrade gracefully (mark WC incomplete, not stall).
- **Index aliasing** (followup #3): with a PDO + background datagram every cycle,
  the wrapping `u8` index needs disciplined allocation so a late reply can't be
  mismatched.
- Background FSM work must never extend a cycle — strictly one background datagram
  per frame, bounded.

### 4.6 DMA cache-coherency reminder (followup #2)

`send_raw`/`poll_raw` use only `atomic::fence(SeqCst)` (no SCB clean/invalidate),
so the ENET descriptor tables + buffers **must** be in **non-cached** memory
(DTCM or MPU-marked). At 4 kHz every cycle touches these buffers; **confirm the
`ECAT_RXDT`/`ECAT_TXDT` statics' placement** (and the new image/TX buffers if DMA
reads them directly) before trusting cyclic data. If they can land in cacheable
OCRAM, add explicit cache maintenance.

---

## 5. Open questions / decisions for the planner

1. **Cycle timer: GPT2 vs PIT.** PIT (75 MHz perclk, simple periodic IRQ) vs GPT2
   (`rtic-monotonics`-friendly, finer control). `GPT1` is taken as a dispatcher.
   Decide the source, its IRQ **priority** (must preempt the priority-1 worker but
   coordinate device ownership), and whether `Mono` (SysTick) stays for
   non-cyclic timing. *Recommend stating the chosen timer + priority explicitly.*

2. **How the cyclic task integrates with the single device owner.** Exactly one
   context may touch the `Device`/`Master`. Options: (a) the timer ISR
   *signals* the existing priority-1 `ethercat_worker`, which runs the cyclic step
   **and** the background FSM step each tick; (b) move device ownership into a
   dedicated high-priority cyclic task and demote CLI/SDO to messages it services
   between cycles. Must preserve "one owner does **both** PDO + background FSM
   stepping." Define the message/handoff between CLI (`usb_isr`) and the cyclic
   owner.

3. **Where the process-data image lives & how it's sized.** A `static`/`'static
   mut` `[u8; N]` (with `N` a compile-time max, e.g. 512 or 1024) owned by the
   cyclic task, vs sized from the parsed config. Decide cache placement (§4.6),
   locking for HAL/`cia402` access (RTIC resource vs single-owner + snapshot), and
   how inputs/outputs are exposed to the rest of the app.

4. **Is the lcec XML parser in scope for the PDO phase?** `config/{model,parser}`
   are scaffolded. Options: (a) parse `ethercat-conf.xml` at runtime (needs a
   `no_std` XML reader + storage for ~8 slaves × ~12 entries); (b) **codegen /
   `build.rs`** the config into a static Rust table; (c) hand-write a static
   `EcSyncInfo[]`/mapping table for v1 and defer parsing. *Recommend (b) or (c) for
   v1* to keep the hot path and binary size bounded, but the user's config-file
   rule favors keeping `ethercat-conf.xml` authoritative. Decide and note it.

5. **How SAFE-OP→OP is reached & what "valid process data" requires.** Confirm the
   single-step transition fix (INIT→PREOP→SAFEOP→OP via `fsm_slave_config`, not the
   current direct `fsm_change`), the requirement that the **LRW is already cycling
   with good WC** before requesting OP, the role of `cia402.rs` walking the
   controlword, and how AL-status-code failures (`0x0134`) are surfaced/retried.

6. **Watchdog configuration.** Decide divider (0x0400) + process-data watchdog
   intervals (0x0420) values, and the master-side policy when WC goes
   `INCOMPLETE`/`ZERO` (e.g. drop to SAFE-OP, zero outputs, alarm). At 250 µs the
   SM watchdog must tolerate the cycle but catch a real stall.

7. **DC reference clock & app-time.** Which slave is the DC reference (XML
   `refClockSlaveIdx="0"`), how app-time is generated on the M7 (free-running
   counter vs ENET 1588), drift-compensation scope (ARMW/FRMW vs none for v1), and
   whether SYNC1 is used.

8. **v1 PDO scope boundary.** Recommended: **one domain**, **fixed mapping from the
   XML** (assignment confirmed; mapping rewritten only if needed), **CiA 402
   CSP** (mode 8) for the YAKO/servos with controlword/statusword/target-position/
   actual-position, the digital-I/O slave bit-packed, **expedited SDO init values
   only (≤ 4 B)**, single LRW per cycle + one interleaved background datagram, DC
   SYNC0 only. Explicitly defer: redundancy, multi-domain, segmented/complete-
   access SDO, EoE/FoE/SoE, runtime re-mapping, hot-plug.

---

## Appendix A — register & constant cheat-sheet (hex)

**ESC registers (ADO for FPRD/FPWR), PDO-relevant:**

| addr | size | meaning |
| --- | --- | --- |
| `0x0120` / `0x0130` / `0x0134` | 2 | AL control / AL status / AL status code |
| `0x0400` / `0x0420` | 2 | watchdog divider / process-data watchdog time |
| `0x0600 + n*16` | 16 | FMMU `n` config page (§2.2) |
| `0x0800 + n*8` | 8 | SM `n` config page → SM0 `0x0800`, SM1 `0x0808`, **SM2 `0x0810`, SM3 `0x0818`** |
| `0x0980` | 2 | DC activation (write `assignActivate`, e.g. `0x0300`; clear to 0 first) |
| `0x0990` | 8 (U64) | DC cyclic operation start time |
| `0x092C` | 4 | DC system-time difference (drift check, < 10 µs) |
| `0x09A0` / `0x09A4` | 4 each | DC SYNC0 / SYNC1 cycle time (written as one 8-byte FPWR @ 0x09A0) |

**Datagram commands (`datagram.rs::Command`):** `LRD=0x0A`, `LWR=0x0B`,
**`LRW=0x0C`**, `ARMW=0x0D`, `FRMW=0x0E` (logical address = U32 in bytes [4..8]).
Datagram length word: bits 0..10 = length, **bit 15 (`0x8000`) = "more datagrams
follow"**.

**Process-data SM control bytes:** SM2 (out) `0x64`, SM3 (in) `0x20`
(bit2 = direction, bit6 = watchdog enable; base from SII SM category).

**CoE objects:** PDO assignment `0x1C12` (SM2) / `0x1C13` (SM3); PDO mapping
`0x1600..`/`0x1A00..`; mapping entry value = `(index<<16)|(subindex<<8)|bitlen`.

**Working counter (cyclic LRW):** `expected = outputs*2 + inputs` (repo bus → 24).

**Timing:** core 600 MHz, IPG 150 MHz, PIT perclk 75 MHz; target cycle 250 µs
(4 kHz); `appTimePeriod` becomes `250000`; image ≈ 306 B (one LRW).

## Appendix B — IgH source URLs used

All from `gitlab.com/etherlab.org/ethercat`, branch **`stable-1.6`**:

- `master/domain.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/domain.c
- `master/fmmu_config.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/fmmu_config.c
- `master/datagram.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/datagram.c
- `master/datagram_pair.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/datagram_pair.c
- `master/slave_config.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/slave_config.c
- `master/fsm_slave_config.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/fsm_slave_config.c
- `master/sync.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/sync.c
- `master/fsm_pdo.c` — https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/master/fsm_pdo.c

lcec config-format reference: LinuxCNC `linuxcnc-ethercat` (`lcec_conf`) +
OpenCN LCEC docs — https://mecatronyx.gitlab.io/opencnc/opencn/components/lcec/lcec.html

**Repo files cited:** `src/ethercat/{mod,device,datagram,master,globals,ecrt,fsm_coe,slave,fsm_slave_config,sync}.rs`,
`src/main.rs`, `src/net/enet_driver.rs`, `ethercat-conf.xml`, `.cargo/config.toml`,
`docs/ethercat-v1-followups.md`.
