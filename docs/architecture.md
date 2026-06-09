# Architecture

This document explains how the firmware is structured: the file-for-file IgH
port, the non-blocking FSM/stepper model that makes a kernel driver fit a
single-core MCU, the RTIC task layout, the cooperative boot, and the cyclic
process-data engine.

For *what* it does and how to drive it, see the [README](../README.md) and
[`cli-reference.md`](cli-reference.md). For *how the bus is configured at compile
time*, see [`config-flow.md`](config-flow.md).

---

## 1. Design: a file-for-file IgH port

The EtherCAT code in `src/ethercat/` is a Rust mirror of the IgH EtherCAT Master
(EtherLab) `master/` core (branch `stable-1.6`). The directory is **flat**, like
IgH's `master/`, and each module's `//!` header records the IgH source file it
ports and what was adapted or dropped. The module list lives in
[`src/ethercat/mod.rs`](../src/ethercat/mod.rs).

The systematic adaptations from a Linux kernel module to a bare-metal RTIC app:

| IgH (Linux kernel) | This port (Teensy / RTIC) |
| --- | --- |
| Patched `net_device` + `sk_buff` + NAPI | The RMII ENET DMA driver in `src/net/`, polled (`send_frame` / `poll_raw`) |
| `module.c`, `cdev.c`, `ioctl.c`, `debug.c`, RTDM | Dropped (no kernel/userspace split; the app links the master in-process) |
| `io_sem` / spinlocks / the kthread FSM loop | RTIC tasks + shared resources + `critical-section` |
| `kmalloc` lists (slaves, PDOs, image) | `heapless::Vec` and fixed `[u8; N]` arrays (no heap) |
| `EC_DBG` / `EC_ERR` / `printk` | the `log` facade and serial trace lines |
| Negative `int` error codes | a typed `EcError` enum (`src/ethercat/ecrt.rs`) |

### Module map

| Module | IgH source | Role |
| --- | --- | --- |
| `ecrt.rs` | `include/ecrt.h` | Public API types (`EcError`, `EcDirection`, PDO/sync info structs) + little-endian access helpers. |
| `globals.rs` | `master/globals.h` | Constants, fixed `EC_MAX_*` capacities, AL states, the ESC register map (`reg::*`), SII map, mailbox/SM/FMMU constants. |
| `datagram.rs` | `master/datagram.c` | `Command` enum + `build` / `append` / `parse` / `parse_at` for one (or multi-) datagram frames. |
| `device.rs` | `master/device.c` | Transport seam over the ENET driver: Ethernet framing, `transact` (blocking), and the non-blocking `pump` primitive. |
| `slave.rs` | `master/slave.c` | `SlaveInfo` (scan identity/base subset) and `Mailbox` parameters. |
| `mailbox.rs` | `master/mailbox.c` | 6-byte mailbox header build/parse + protocol multiplexing. |
| `sync.rs` | `master/sync.c` | Sync-manager page encoders (mailbox SM0/SM1 and process-data SM2/SM3). |
| `fmmu_config.rs` | `master/fmmu_config.c` | The 16-byte FMMU ESC page encoder. |
| `domain.rs` | `master/domain.c` | The process-data image + working-counter math + the cyclic LRW builder. |
| `master.rs` | `master/master.c` | Top-level master: owns the transport + discovered slaves; runs runtime `Request`s as non-blocking `Op` steppers via `poll_op`. |
| `fsm_scan.rs` | `fsm_master.c` + `fsm_slave_scan.c` + `fsm_sii.c` | Non-blocking bus scan (count, clear, per-slave identity), streaming trace lines. |
| `fsm_change.rs` | `master/fsm_change.c` | Non-blocking AL state-change handshake. |
| `fsm_coe.rs` | `master/fsm_coe.c` | Non-blocking expedited (≤ 4-byte) SDO upload/download (+ a `CoeSeq` batch helper). |
| `fsm_pdo.rs` / `fsm_pdo_entry.rs` | `master/fsm_pdo*.c` | PDO assignment (`0x1C1x`) and mapping (`0x16xx`/`0x1A0x`) over CoE. |
| `fsm_slave_config.rs` | `master/fsm_slave_config.c` | Per-slave bring-up INIT → SAFE-OP (composes the FSMs above). |
| `dc.rs` | scattered in IgH | Distributed-clocks SYNC0 bring-up FSM. |
| `cyclic.rs` | the cyclic half of `master/master.c` | The PIT-tick process-data engine + SAFE-OP → OP gating. |
| `config/{model,generated}.rs` | (project addition) | Compile-time bus config (see [`config-flow.md`](config-flow.md)). |
| `cli.rs` | `tool/` (`ethercat` CLI) | Serial line parser → master `Request`s (interface layer, not an IgH `master/` file). |
| `cia402.rs` | (not IgH core) | CiA-402 drive state machine (scaffolding; app/interface layer). |

> `fsm_master.rs` / `fsm_slave_scan.rs` retain a blocking `scan_bus` path
> (`Device::transact`), but the **live** scan path (`rescan`, and the scan you
> run before `start`) is the non-blocking `ScanFsm` in `fsm_scan.rs`. The blocking
> path is not used on the worker/cyclic hot path.

---

## 2. The non-blocking FSM/stepper model

A Linux driver can block a kernel thread on `wait_event`/`schedule()`. On a
single-core MCU with a 100 Hz–4 kHz cyclic deadline, **nothing may busy-wait on
the wire**. The whole protocol layer is therefore rebuilt as cooperative state
machines that each advance **exactly one datagram per call** and yield. Three
pieces make this work.

### 2.1 `Pump` — the one-datagram transaction tracker (`device.rs`)

`Pump` is a tiny send-once / poll-once tracker. The first call sends an
already-built frame; each later call checks the RX ring **once** for the matching
reply (by datagram index byte), counting attempts toward a timeout. It never
spins.

```rust
// device.rs (shape)
pub fn pump(&mut self, pump: &mut Pump, frame: &[u8], rx: &mut [u8], max_attempts: u32)
    -> Result<Option<usize>, EcError>;
//        Ok(None)      => still waiting (call again next tick)
//        Ok(Some(len)) => reply arrived
//        Err(Timeout)  => max_attempts elapsed with no matching reply
```

Every protocol FSM owns a `Pump` plus its own `tx`/`rx` scratch buffers, and
calls `dev.pump(...)` once per step. (Replies are matched on the index byte only;
this is safe for the strictly one-outstanding-datagram model — see follow-up #3
in [`ethercat-v1-followups.md`](ethercat-v1-followups.md).)

`Device` also provides:
- `send(frame)` — wraps an EtherCAT frame in the 14-byte Ethernet header
  (broadcast dst, fixed src MAC, `0x88A4` big-endian) and hands it to the ENET
  DMA; only the header is built on the stack.
- `poll(out)` — pops one received frame, filters EtherType `0x88A4`, copies the
  EtherCAT payload out.
- `transact(frame, out)` — a **blocking** send-and-wait (busy-wait via
  `cortex_m::asm::delay`), retained only for the legacy blocking scan; the cyclic
  and worker paths never call it.

ENET ring facts: per-descriptor MTU `1536`, RX/TX rings **4 deep**.

### 2.2 Per-datagram `step()` FSMs

Each protocol stage is an `enum` of phases stepped by `step(dev, index) -> Result<bool, EcError>`,
returning `Ok(false)` while pending and `Ok(true)` when complete. They embed
their `Pump` and scratch buffers inline so no heap is needed. Examples:

- **`FsmChange`** — AL control write → poll AL status → read status code.
- **`FsmCoe`** / **`CoeSeq`** — expedited SDO write/upload (+ a batch sequence for
  SDO-init and PDO config); buffers are `[u8; 320]`.
- **`FsmDc`** — DC SYNC0 setup (§5).
- **`FsmSlaveConfig`** — the per-slave bring-up that *composes* the above (§4).
- **`ScanFsm`** — the bus scan, with an inner `Sii` sub-stepper (§3).

`index: &mut u8` is the master's single wrapping datagram-index counter, shared so
every datagram across all FSMs gets a distinct index.

### 2.3 `Master::poll_op` and the `Op` enum (`master.rs`)

`Master` owns the `Device`, a `heapless::Vec<SlaveInfo>`, the datagram `index`,
the optional in-flight `Op`, and the optional `Cyclic` engine. A serial command
becomes a `Request`; `Master::begin(request)` validates it (slave exists, CoE
supported, config present) and builds the matching `Op`. Then the driver calls
`poll_op()` repeatedly:

```rust
// master.rs (shape)
pub fn poll_op(&mut self) -> Option<Result<Outcome, EcError>> {
    let mut op = self.op.take()?;
    match self.drive(&mut op) {      // advance ONE datagram
        Ok(None)          => { self.op = Some(op); None }   // pending
        Ok(Some(outcome)) => Some(Ok(outcome)),             // done
        Err(e)            => Some(Err(e)),                   // failed
    }
}
```

`Op` variants: `Rescan { ScanFsm }`, `State { PreOp }`, `Sdo { PreOp, FsmCoe, … }`,
`StartCyclic { FsmSlaveConfig }`, `StopCyclic`. (`PreOp` is a small helper that
configures the mailbox SMs then runs `FsmChange`, used for `states` and the SDO
pre-bring-up.) Because the master lives in one static cell, the large `Op` enum is
a one-time static cost rather than a per-call allocation.

### 2.4 The cooperative driver in `main.rs`

The async worker drives `poll_op` to completion, **locking the shared master once
per datagram and yielding between steps** so USB and the cyclic PIT task run:

```rust
// main.rs::ecat_drive (shape)
master.lock(|m| m.0.begin(req))?;
loop {
    match master.lock(|m| m.0.poll_op()) {
        None      => yield_now().await,   // self-wake; resume next executor pass
        Some(res) => return res,
    }
}
```

`yield_now()` is a `poll_fn` that wakes itself by ref and returns `Pending` once,
so it yields to any higher-/equal-priority task and resumes immediately — keeping
the FSMs strictly non-blocking without a per-datagram sleep.

---

## 3. The bus scan (`fsm_scan.rs`)

`ScanFsm` is the non-blocking scan. Its stages, each one datagram:

1. **Count** — `BRD` of AL status `0x0130`; the working counter is the slave
   count.
2. **Clear** — `BWR` of station address `0x0010` (zero all addresses).
3. Then, **per slave** (ring position `r`, station `r+1`):
   - **Apwr** — `APWR` assign the configured station address.
   - **Al** — `FPRD` AL status `0x0130`.
   - **Dl** — `FPRD` DL/base info `0x0000` (type, FMMU/SM counts).
   - **Sii** — read SII identity via an inner `Sii` sub-stepper (vendor, product,
     revision, Rx/Tx mailbox offset+size, mailbox protocols). The `Sii` stepper
     issues the SII read command, then polls the SII control/status register until
     the busy bit clears.

Each completed sub-step records a short **trace line** (`[scan] s1: vendor=0x…`),
which the worker streams over serial as the scan runs. This is the primary
no-SWD diagnostic: if a real-bus scan faults the firmware, every step up to the
fault is already on the host console. Completed slaves are handed to the caller
one at a time (`take_completed_slave`) so the FSM carries no per-slave `Vec`.

The worker path that streams these lines is `run_rescan_traced` in `main.rs`;
working-counter checks reject a partial response rather than fabricate a
`SlaveInfo`.

---

## 4. Per-slave bring-up (`fsm_slave_config.rs`)

`FsmSlaveConfig` walks one slave from INIT to **SAFE-OP**, composing the smaller
FSMs. It is the `Op::StartCyclic` body. Phase order (each issues one or more
datagrams via `Device::pump`):

```text
ClearFmmus    FPWR 0x0600  zero all FMMU pages
DcClear       FPWR 0x0980  disable DC activation before reconfiguring
MboxSm0       FPWR 0x0800  mailbox-out SM page (SM0)
MboxSm1       FPWR 0x0808  mailbox-in  SM page (SM1)
ToPreop       FsmChange    → PRE-OP
SdoInit       CoeSeq       expedited SDO-init values from the config (e.g. 0x6060 = 8 / CSP)
PdoMapping    CoeSeq       per-PDO mapping (0x16xx / 0x1A0x) over CoE
PdoAssign     CoeSeq       per-SM PDO assignment (0x1C12 / 0x1C13) over CoE
PdSm          FPWR 0x0810/0x0818  process-data SM pages (SM2 out, SM3 in)
WatchdogDiv   FPWR 0x0400  watchdog divider (100 µs base)
WatchdogTime  FPWR 0x0420  process-data watchdog (~200 ms, generous for bring-up)
Fmmu          FPWR 0x0600+n*16  FMMU pages (output + input)
DcCycle       FsmDc        DC SYNC0 setup (§5), if the slave has a dcConf
ToSafeop      FsmChange    → SAFE-OP
Done
```

Reaching **OP** is *not* part of this FSM. SAFE-OP is the last static step; OP is
requested by the cyclic engine only once process data is actually flowing with a
good working counter (§6). This mirrors EtherCAT's rule that a slave refuses OP
until it is receiving valid process-data frames.

---

## 5. Distributed Clocks SYNC0 (`dc.rs`)

`FsmDc` brings up SYNC0 for one slave. v1 targets a single drive that is its own
reference clock, so cross-slave offset/drift compensation (ARMW/FRMW, register
`0x092C`) is **not** performed. States:

```text
LatchTime   FPWR 0x0900  latch the drive's local time
ReadTime    FPRD 0x0910  read DC system time; compute a future, cycle-aligned start
WriteCycle  FPWR 0x09A0  SYNC0 (and SYNC1) cycle times (8 bytes)
WriteStart  FPWR 0x0990  cyclic-operation start time (U64)
Activate    FPWR 0x0980  write the assignActivate word (e.g. 0x0300 = SYNC0)
Done
```

The start time is placed `START_MARGIN_NS = 100 ms` in the future and rounded up
to a cycle boundary (plus the configured `sync0Shift`), so SYNC0 activation has
settled on the drive before the first pulse. The DC start-time math is the
highest-risk part of the bring-up; it is verified working at 100 Hz.

---

## 6. The cyclic process-data engine (`cyclic.rs` + `domain.rs`)

### 6.1 The domain image (`domain.rs`)

`EcDomain` owns one contiguous process-data image (`[u8; MAX_IMAGE]`, `MAX_IMAGE
= 512`), the expected working counter, and the input byte-ranges. It is built
from the compile-time `BUS`:

- **Expected WKC** = `outputs × 2 + inputs` per FMMU (a slave with SM2 + SM3
  contributes `+3`). For the v1 single drive that is `3`.
- `build_lrw(buf, index)` builds one **LRW** datagram covering the whole image at
  logical address 0.
- `apply_reply(reply)` copies **only the input ranges** back into the image (so
  application-written outputs are preserved) and records the working counter as
  `Zero` / `Incomplete` / `Complete`.

### 6.2 The engine (`cyclic.rs`)

`Cyclic` runs the image exchange, pipelined (process last cycle's reply, then send
this cycle's frame). Phases:

```text
Priming      cycle the LRW until the slave responds (WKC>0) for PRIMING_CYCLES (3)
RequestingOp keep the LRW flowing while interleaving an AL-control(=OP) / AL-status
             datagram in the SAME frame (the 0x8000 "more datagrams follow" bit),
             so process data never stops while OP is requested
Operational  steady single-LRW exchange in OP
Faulted      the drive rejected OP (AL error); keep cycling so it holds SAFE-OP
```

`tick(dev, index)` is what the high-priority PIT task calls each period:

```rust
// cyclic.rs (shape)
pub fn tick(&mut self, dev: &mut Device, index: &mut u8) {
    self.total_cycles += 1;
    if self.outstanding { self.receive(dev); self.outstanding = false; } // last reply
    self.send(dev, index);                                               // this frame
}
```

`receive` matches the reply by index and walks the (possibly multi-) datagram
frame with `datagram::parse_at`; during `RequestingOp` it reads the appended AL
status to detect OP or an AL error. `send` builds the LRW and, while requesting
OP, appends an alternating AL-control / AL-status datagram. Everything on this
path is allocation-free and never busy-waits.

A `CyclicStatus { phase, wkc, expected_wkc, cycles }` snapshot backs `status` and
`pd` (no-arg). The **HAL pin layer** (`src/hal/`) reads/writes named pins over the
image: `read_value` / `write_value` handle `bit` (mask at `bit_pos`), `u32`
(zero-extend), and `s32` (sign-extend) per the pin's `hal_type`.

---

## 7. RTIC task layout

The RTIC application is in [`src/main.rs`](../src/main.rs):

```rust
#[rtic::app(device = teensy4_bsp, dispatchers = [GPIO6_7_8_9, LPUART8, GPT1])]
```

| Task | Binds / kind | Priority | Role |
| --- | --- | --- | --- |
| `init` | startup | — | Clocks, LEDs, SysTick monotonic (`Mono`, 1 kHz), USB, ENET, build the `Master`; spawn `blink_leds` + `ethercat_worker`. The PIT is configured lazily by `start`. |
| `ethercat_worker` | async software task | 1 | The command loop: take a parsed command, drive its non-blocking FSM to completion (yielding between datagrams), publish the response, wake USB. Owns the cyclic-engine lifecycle. |
| `blink_leds` | async software task | 1 | LED heartbeat (alternates pins 4/5) = "tasks running". |
| `usb_isr` | `binds = USB_OTG1` | 2 | Poll the USB CDC device, parse typed lines into commands, queue them, and flush command responses promptly; also emits the one-time boot banner + scan summary and handles the bootloader/reboot requests. |
| `cyclic` | `binds = PIT` | 3 | One short, non-blocking cyclic tick: clear the PIT flag, lock the master, `cyclic_tick()`. Highest priority so the cycle is not delayed by USB or the worker. |

### The shared `ecat_master` resource

```rust
#[shared]
struct Shared {
    ecat_scan: EcatScan,          // scan result for the boot report
    ecat_cmd:  Option<cli::Command>,  // usb_isr -> worker command slot
    ecat_out:  EcatOut,           // worker -> usb_isr response bytes
    ecat_master: EcatMasterCell,  // the Master, locked by worker (1) and cyclic (3)
}
```

`ecat_master` is locked by both `ethercat_worker` (priority 1) and the `cyclic`
PIT task (priority 3). Under RTIC's priority-ceiling protocol the lock's ceiling
is therefore **3**, which **masks `usb_isr` (priority 2)** whenever the master is
held. This is the key constraint that shapes the boot (§8). The cyclic task is the
highest-priority user, so *its* lock never blocks. `EcatMasterCell` is a `Send`
wrapper justified by single-core exclusive ownership (the master holds `&'static
mut` ENET descriptor tables containing raw pointers).

---

## 8. Cooperative boot

`init` deliberately does **not** scan the bus, and `ethercat_worker` waits for USB
before touching the master:

1. `init` brings up clocks/USB/ENET and spawns the worker. It shows a 3-bit LED
   progress code per stage (§10) so a boot hang freezes on the last stage reached.
2. `ethercat_worker` waits (bounded, ≤ 200 × 50 ms) for `USB_READY` (set by
   `usb_isr` once the device is configured) before its first master access.
3. It then **does not auto-scan** — it pends `usb_isr` and enters the command loop.

Why: the scan (especially the legacy blocking one) monopolizes the priority-1
executor and holds the master lock; because that lock masks `usb_isr` (§7), an
auto-scan at boot would stall USB enumeration and freeze `blink_leds`. So the scan
runs only on demand (`rescan`), and `start` requires a prior `rescan` to have
populated the slave list. The legacy W5500/Modbus path is also skipped in `init`
(it would hang without a W5500 chip and shares the pin-13 LED).

---

## 9. Build-time knobs

`.cargo/config.toml` pins the target/linker/CPU and supplies compile-time config
through `[env]` vars that `src/main.rs` reads with `env!()` const parsers:

| Env var | Default | Meaning |
| --- | --- | --- |
| `TEENSY4_CORE_CLOCK_HZ` | `600000000` | Core clock. Profiles above 600 MHz also need `TEENSY4_ALLOW_OVERCLOCK=true`; the highest need `TEENSY4_ALLOW_MAX_VOLTAGE=true`. |
| `TEENSY4_ALLOW_OVERCLOCK` | `true` | Gate for > 600 MHz profiles. |
| `TEENSY4_ALLOW_MAX_VOLTAGE` | `false` | Gate for max-DCDC-voltage profiles. |
| `TEENSY4_STACK_SIZE` | `65536` | Main stack size, in bytes (DTCM). See below. |
| `LED_INDICATOR_PIN` / `BASE_LED_A_PIN` / `BASE_LED_B_PIN` | `13` / `4` / `5` | Heartbeat / boot-code LED pins. |
| `BASE_LED_BLINK_HZ` | `2` | Heartbeat rate. |

### Why 64 KB of stack

The IgH-derived FSMs carry multi-KB scratch buffers, and `Master::poll_op` moves a
~1.7 KB `Op` local onto the stack each call (the `Op` enum embeds whichever FSM is
running, with its `tx`/`rx` arrays). The deep startup/scan call path overflowed
the imxrt-rt **16 KB** default and faulted. Raising `TEENSY4_STACK_SIZE` to 64 KB
fixes it with margin; DTCM is 320 KB, so `.vectors`/`.data`/`.bss` still fit
comfortably. (The persisted crash log and the heap live in OCRAM, not DTCM.)
`teensy4-bsp` wires this env var into imxrt-rt's `RuntimeBuilder::stack_size`,
regenerating the linker script's `.stack`.

### `FW_TAG`

`build.rs` captures a git build-provenance tag into the `FW_TAG` env
(`v<pkg>-g<short-sha>[-dirty]`, or `v<pkg>-nogit` outside a checkout). It appears
in the boot banner and `status` output so the exact running build is identifiable
over serial.

---

## 10. Crash diagnostics & fault handling

The firmware has no SWD in normal use, so it self-reports faults. A `CrashLog`
struct lives in a `.uninit.CRASHLOG` link section (NOLOAD; never zeroed by
startup), so a handler can record the crash, reset, and let the next boot replay
it over USB.

### Two crash classes

- **CPU HardFault** (`HardFault` exception handler): records the stacked exception
  frame (`pc`, `lr`, `sp`, `msp`, `r0–r3`, `r12`, `xpsr`), the SCB fault-status
  registers (`cfsr`, `hfsr`, `bfar`, `mmfar`), and `send_stage` (how far the first
  ENET send had progressed), then **`sys_reset()`**. The handler is minimal-stack
  (field stores only, no formatting) because the fault may itself be a stack
  overflow. The board reboots and is recoverable over USB; read the dump with
  `crashlog`.
- **Rust panic** (`panic_handler`): records the panic message/location, then
  **halts** on an LED code (it does *not* reset, to avoid a no-USB reboot loop if
  the panic is on the boot path). The message is retrievable with `crashlog` after
  a manual reboot.

`crashlog` formats the saved record into `[crash] …` lines (non-destructive;
re-readable until `crashclear`). For a HardFault whose `sp`/frame/`bfar` falls
below `FAULT_STACK_GUARD = 0x2000_0400`, it appends a **stack-overflow hint**.
Field reference:

| Field | Meaning |
| --- | --- |
| `pc` / `lr` | Faulting program counter / link register. |
| `frame_sp` / `msp` | Stacked exception-frame pointer / main stack pointer. |
| `cfsr` / `hfsr` | Configurable / Hard fault status registers (the fault cause bits). |
| `bfar` / `mmfar` | Bus-fault / mem-manage fault address (valid only when the matching CFSR bit is set). |
| `r0–r3`, `r12`, `xpsr` | The stacked caller context. |
| `send_stage` | `0`/`1`/`2` — how far `Device::send` got (localizes a fault in the first ENET send). |

### Boot-stage and fault LED codes

During `init`, a 3-bit code is shown on the indicator (pin 13 = bit 2), LED A
(pin 4 = bit 1), LED B (pin 5 = bit 0) and held briefly per stage, so a boot hang
freezes on the last stage reached:

| Code | Stage reached |
| --- | --- |
| 1 | clocks + LEDs + monotonic up |
| 2 | USB peripheral configured |
| 3 | ENET clocks/pads + PHY reset |
| 4 | ENET MAC/DMA up |
| 5 | PHY MDIO configured |
| 6 | master built (init complete) |

After `init`, `blink_leds` alternates pins 4/5 = "tasks running". A **panic**
parks on the indicator LED as **1 long pulse** (class 1), forever. A HardFault
reboots instead of blinking (its evidence is the persisted `crashlog`).
