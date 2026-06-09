# Teensy ↔ Raspberry Pi / LinuxCNC SPI Bridge — Design

> **What this document is.** The design brief for connecting the Teensy EtherCAT
> master to a Raspberry Pi running LinuxCNC (HAL), so the heavy real-time work
> (EtherCAT cyclic PDO, DC/SYNC0, CiA-402 drive sequencing, drive safety) stays
> on the MCU and the Pi only exchanges process data with it over a host bus.
>
> It is a **planning brief**, not an implementation plan: it records the
> architecture decisions, the wire contract, the XML schema additions, and the
> firmware/host work each implies, so a later implementation agent (or the user)
> can build it in bounded steps. It deliberately mirrors the style and depth of
> [`docs/pdo-planning-input.md`](./pdo-planning-input.md).
>
> **Hard constraints (repo rules):** `no_std`, **no heap** (`heapless` / fixed
> arrays); the user flashes/runs hardware — do not run firmware; keep
> files/functions short and single-purpose; the **bus XML stays the single source
> of truth** for configuration; scratch/refactor files are prefixed `temporary_*`.

**Project:** `/Users/griffinsisk/Documents/Documents_Griffins_Work_Mac/github/RTIC-ECMaster`
**Hosts:** Teensy 4.1 (i.MX RT1062 Cortex-M7 @ 600 MHz) ⟷ Raspberry Pi 5 (PREEMPT_RT, LinuxCNC HAL).
**Host bus:** SPI (Pi master / Teensy LPSPI slave) on a shared PCB / Pi-HAT, target 40 MHz + GPIO sync line(s).

---

## Table of contents

1. [Goals & non-goals](#1-goals--non-goals)
2. [System topology & responsibility split](#2-system-topology--responsibility-split)
3. [Why SPI (bus choice)](#3-why-spi-bus-choice)
4. [The contract comes from the bus XML](#4-the-contract-comes-from-the-bus-xml)
5. [XML schema additions](#5-xml-schema-additions)
6. [The SPI frame format](#6-the-spi-frame-format)
7. [The motion look-ahead buffer](#7-the-motion-look-ahead-buffer)
8. [Clocking, sync & the GPIO line](#8-clocking-sync--the-gpio-line)
9. [Safety, watchdog & the unified safe state](#9-safety-watchdog--the-unified-safe-state)
10. [Latency analysis](#10-latency-analysis)
11. [Firmware changes (Teensy)](#11-firmware-changes-teensy)
12. [Host changes (Pi / LinuxCNC HAL)](#12-host-changes-pi--linuxcnc-hal)
13. [Phased implementation plan](#13-phased-implementation-plan)
14. [Open questions / decisions](#14-open-questions--decisions)

---

## 1. Goals & non-goals

**Goal.** Make the Teensy a self-contained, hard-real-time EtherCAT motion
controller that LinuxCNC drives over SPI the way LinuxCNC drives a Mesa card over
`hm2_rpspi` — one process-data exchange per servo cycle — while the Teensy
absorbs all EtherCAT timing, drive-state sequencing, and fail-safe behavior.

**In scope (v1):**

- A fixed-layout, full-duplex SPI exchange between Pi (master) and Teensy (slave).
- A **single source of truth**: the bus XML generates the EtherCAT config *and*
  the SPI frame layout *and* the LinuxCNC HAL pin table.
- A **look-ahead buffer** for motion setpoints so Pi scheduling jitter never
  reaches the drives.
- Teensy-owned **CiA-402** drive sequencing and a unified **safe-state** path
  (buffer underrun, host watchdog timeout, EtherCAT fault → drive quick-stop).

**Out of scope (v1):** motion/trajectory planning on the MCU (that stays in
LinuxCNC); hot-plug/topology changes over SPI; multi-master; phase-locking the Pi
servo thread to EtherCAT DC (the Pi free-runs and works ahead — §8); encryption.

**Non-negotiable principle.** The Pi computes *what* to do; the Teensy guarantees
*when* and *safely*. Anything whose failure must be deterministic (stop ramps,
watchdog, OP gating) lives on the Teensy.

---

## 2. System topology & responsibility split

Chosen split: **Hybrid** — smart-enough Teensy, motion-owning Pi.

```text
  LinuxCNC (Pi 5, PREEMPT_RT)                 Teensy 4.1 (RTIC, single-core M7)
  ┌──────────────────────────┐   SPI 40 MHz   ┌──────────────────────────────┐   RMII   ┌────────┐
  │ traj planner / kinematics │  ◄──────────►  │ SPI-slave task (LPSPI)        │  L2     │ drives │
  │ PID / GUI / G-code        │  + GPIO ready  │ motion look-ahead buffer      │ 0x88A4  │ + I/O  │
  │ HAL driver (generated)    │                │ CiA-402 state + safety        │ ◄─────► │ (8 ESC │
  │   - sends setpoints ahead │                │ cyclic PDO engine (PIT/SYNC0) │         │  nodes)│
  │   - reads feedback fresh  │                │ EtherCAT master (IgH port)    │         │        │
  └──────────────────────────┘                └──────────────────────────────┘         └────────┘
```

| Concern | Owner | Notes |
| --- | --- | --- |
| Trajectory / kinematics / PID / GUI | **Pi (LinuxCNC)** | Mature motion stack; don't reimplement on the MCU. |
| Per-axis setpoint stream (pos/vel/torque) | **Pi → Teensy buffer** | Streamed ahead; see §7. |
| CiA-402 drive state machine (enable/fault/quick-stop) | **Teensy** (`src/ethercat/cia402.rs`) | Currently a stub; this design requires implementing it. |
| EtherCAT cyclic PDO, DC/SYNC0, WKC health | **Teensy** (`src/ethercat/cyclic.rs`, `dc.rs`) | Already implemented; the bridge feeds/reads its image. |
| Hard safe-state (stop on fault/stall) | **Teensy** | Deterministic; the only thing with the real clock. |
| Configuration (drives, PDO map, HAL names) | **Bus XML** | Single source of truth (§4). |

**Why not put trajectory on the MCU.** The Teensy is single-core (i.MX RT1062 has
one M7); there is no second core to isolate interpolation from the EtherCAT ISR.
The *distributed* system already is the "dual core": Teensy = hard-RT core, Pi =
soft-RT motion core. If on-MCU dual-core is ever truly needed, the natural upgrade
is an **i.MX RT1170 (M7 + M4)** — a board change, explicitly deferred.

---

## 3. Why SPI (bus choice)

Decision: **SPI**, Pi master / Teensy LPSPI **slave**, full-duplex, one
transaction per servo cycle, plus **one GPIO "frame-ready" line** (and an optional
dedicated hardware E-stop line).

| Candidate | Verdict |
| --- | --- |
| **SPI** | ✅ Full-duplex = a process-image *swap* in one transaction (write outputs while reading inputs). ~5–6 Pi pins. Proven LinuxCNC realtime path (`hm2_rpspi` → Mesa). |
| SDIO | ❌ Tied to SD/Wi-Fi on the Pi; no clean realtime slave path. |
| QSPI / SQI | ❌ Overkill for a ~300-byte image; quad-SPI **slave** on the RT1062 is awkward. (QSPI *does* matter on the Teensy side — but for **PSRAM**, not the host link; see §7.4.) |

**Bandwidth.** On a PCB/Pi-HAT in close proximity, **40 MHz** is realistic
(≈ **200 ns/byte**). The full 8-node machine image is ≈ **306 bytes**
(`docs/pdo-planning-input.md` §3.6); with headers + a steady-state streamed motion
block + CRC, a nominal frame is ~512 bytes ≈ **~100 µs**. That is ~40 % of a 250 µs
(4 kHz) cycle and trivial at 1 kHz — and because LPSPI is a separate peripheral with
its own DMA, the transfer overlaps the EtherCAT round-trip rather than adding to it,
so it **fits with headroom**. Note the streamed block is *variable* length under
batch refill (§7.1): a large post-stall top-up across many axes can exceed the
nominal size, so the **worst-case refill frame must be bounded** against the cycle
budget (§7.3, open-Q §14.6).

**Pins (Teensy side).** LPSPI is entirely unused by the current firmware. Prefer
**LPSPI3** (pins **26/27/39**) which is free; **LPSPI4** (11/12/13) collides with
the pin-13 indicator LED unless `LED_INDICATOR_PIN` is relocated. The Teensy runs
as the **slave** (Pi clocks); add a Teensy→Pi **`FRAME_READY`** GPIO (any free
header pin via `src/board/teensy_pin_map.rs`).

---

## 4. The contract comes from the bus XML

The wire contract is **derived from the same XML** that already drives the
EtherCAT config — not hand-maintained. Today `scripts/generate_ethercat_config.py`
turns `ethercat-conf.xml` into `src/ethercat/config/generated.rs` (the `BUS`
table + `PINS` map). This design extends that one generator pass to emit **three**
artifacts from the one XML:

```text
                          ethercat-conf.xml  (single source of truth)
                                   │
                 scripts/generate_ethercat_config.py  (make config)
              ┌────────────────────┼─────────────────────────┐
              ▼                    ▼                         ▼
   src/ethercat/config/    Teensy SPI frame layout    Pi HAL pin table
     generated.rs            (Rust packed struct        (.h / HAL comp pin
   (BUS + PINS, today)        + offsets, §6)             list, §12)
```

Because all three come from one pass, the wire format **cannot silently disagree**
with either end. The `halPin` names *are* the LinuxCNC HAL pin names — e.g.
`die-cylinder-target-position` in the XML becomes `<comp>.die-cylinder-target-position`
in HAL.

> **Which XML is canonical?** The generator's `--bus` argument selects the source
> file; it currently defaults to `ethercat-conf.bohign.xml` (the verified
> single-drive bench config), while `ethercat-conf.xml` holds the full 8-node
> machine. "Single source of truth" means *whichever* `--bus` file a given build
> targets — both follow the same schema (§5). The worked numbers here use the
> 8-node `ethercat-conf.xml`.
>
> **Generator limitation to lift first.** The current generator reads only the
> **first PDO per sync manager** (`sm["pdos"][0]` in
> `scripts/generate_ethercat_config.py`). That is fine for the single-PDO drives,
> but the digital-I/O node (slave 4) assigns *multiple* PDOs per SM. Since the SPI
> layout artifact needs the **complete** per-SM entry list, this must be generalized
> to iterate all PDOs before the multi-PDO node is bridged.

The existing pieces this builds on (verified in the tree):

- `src/ethercat/config/model.rs` — `PinCfg { name, byte_offset, bit_pos, bit_len, hal_type, dir }`
  and `BusCfg { cycle_ns, slaves, pins, image_size }`.
- `src/ethercat/config/generated.rs` — the concrete `PINS` array with resolved
  `byte_offset`s into the cyclic image (the bridge maps these to SPI frame offsets).
- `src/hal/process_data.rs` — `find(name)`, `read_value(image, pin)`,
  `write_value(image, pin, value)`; and `master.cyclic_image()/cyclic_image_mut()`
  expose the image the bridge bridges. **No new image is introduced** — the SPI
  task reads/writes the *existing* cyclic image through this API.

---

## 5. XML schema additions

Three additions to the lcec-style XML, all backward-compatible (defaults preserve
today's behavior).

### 5.1 `class` on `<pdoEntry>` — streamed vs immediate

Marks which output entries are part of the buffered motion stream. Default is
`immediate`.

```xml
<!-- streamed: goes through the look-ahead buffer (§7) -->
<pdoEntry idx="607a" subIdx="00" bitLen="32" halType="s32"
          halPin="shuttle-target-position" class="motion"/>

<!-- immediate (default): latest-value-wins, never buffered -->
<pdoEntry idx="6040" subIdx="00" bitLen="16" halType="u32"
          halPin="shuttle-ctrlword"/>
```

Rule of thumb: **commands that must react now stay immediate** — controlword,
`op-mode` (`6060`), digital outputs. **Feedback is always immediate** (an `in`
direction is never streamed). Only forward motion setpoints are streamed.

### 5.2 `<motionStream>` per slave — the buffered register set + lead

Because the streamed set depends on the drive's mode (CSP → position, CSV →
velocity, CST → torque, plus feed-forwards), it is **configurable per slave**. A
block placed under the slave's PDO config lists which `halPin`s form that axis's
buffered sample, and the desired look-ahead depth:

```xml
<slave idx="1" ... name="shuttle">
  ...
  <syncManager idx="2" dir="out"> ... </syncManager>
  <syncManager idx="3" dir="in">  ... </syncManager>

  <!-- This axis streams a (position, vel-FF, torque-FF) triple, 10 frames ahead -->
  <motionStream lead="10">
    <streamEntry halPin="shuttle-target-position"/>
    <streamEntry halPin="shuttle-velocity-offset"/>
    <streamEntry halPin="shuttle-torque-offset"/>
  </motionStream>
</slave>
```

The generator computes each axis's **sample width** (sum of referenced entry
widths) and emits the streamed-block layout. `<streamEntry>` references pins **by
`halPin` name** (matches the rest of the contract); the referenced entries must
also carry `class="motion"`. A CSV axis would list `*-target-velocity`; a CST axis
`*-target-torque`. `lead` may be set per axis (default falls back to a master-level
value).

### 5.3 Quick-stop option code via `<sdoConfig>`

To make the safe-state ramp deterministic per drive, configure the CiA-402
**quick-stop option code (`0x605A`)** explicitly rather than relying on drive
defaults (this uses the existing `<sdoConfig>` path, §3.3 of the PDO brief):

```xml
<sdoConfig idx="0x605A" subIdx="0x00"><sdoDataRaw data="02 00"/></sdoConfig>
```

(`0x605A = 2` = "decelerate on quick-stop ramp then stay in Quick-Stop-Active" —
choose per drive/machine.)

---

## 6. The SPI frame format

One full-duplex transaction per servo cycle. The Pi clocks; both directions
transfer simultaneously. The frame splits into **two classes**: immediate
(latest-wins) and streamed (buffered motion).

### 6.1 Pi → Teensy (MOSI)

```text
┌─────────── header ───────────┐┌─── immediate-out ───┐┌──── streamed-out ────┐┌─CRC─┐
│ magic │ ver │ host_seq │ wdog ││ controlwords,        ││ count │ sample[0..n] ││ crc │
│ flags                         ││ op-modes, DO bits…   ││  each tagged w/ idx  ││ 16  │
└───────────────────────────────┘└─────────────────────┘└──────────────────────┘└─────┘
```

- `magic`/`ver` — framing + protocol version (lets firmware and HAL evolve safely).
- `host_seq` — increments per frame; echoed back for round-trip tracking.
- `host_wdog` — host heartbeat counter (§9).
- `flags` — host intents (e.g. request-enable, fault-reset, request-quick-stop).
- **immediate-out** — every *non-streamed* output pin, laid out exactly as in the
  EtherCAT output image (controlwords, `op-mode`, digital outputs).
- **streamed-out** — `count` then `count` motion samples, **each tagged with the
  absolute EtherCAT cycle index it applies to** (§7). `count` varies per frame
  (batch refill); samples may span axes.
- `crc` — CRC-16 over the whole frame (SPI has no framing/error detection).

### 6.2 Teensy → Pi (MISO)

```text
┌──────────────── header ────────────────┐┌──── immediate-in ────┐┌─CRC─┐
│ seq_echo │ teensy_wdog │ cycle_index    ││ statuswords, actual   ││ crc │
│ link/wkc │ cyclic_phase │ buf_depth[*]  ││ pos/vel, errors, DI…  ││ 16  │
│ fault_flags                             ││                       ││     │
└─────────────────────────────────────────┘└──────────────────────┘└─────┘
```

- `cycle_index` — the Teensy's **authoritative** EtherCAT cycle counter. This is
  the existing `Cyclic::total_cycles` (surfaced today as `CyclicStatus.cycles`, a
  wrapping `u32` incremented per tick in `src/ethercat/cyclic.rs`); the bridge just
  exposes it as the absolute index. The Pi tags streamed samples relative to this.
- `link/wkc`, `cyclic_phase` — from `CyclicStatus` (`src/ethercat/cyclic.rs`):
  `Priming/RequestingOp/Operational/Faulted` + `wkc/expected_wkc`.
- `buf_depth[*]` — per-axis motion-buffer fill level → the Pi's flow-control input.
- `fault_flags` — EtherCAT fault, drive fault (CiA-402), underrun, watchdog state.
- **immediate-in** — all feedback pins (statuswords, actual position/velocity,
  error codes, digital inputs), always freshest (never delayed).

### 6.3 Layout source

Field order/offsets/types in both directions are **generated from the XML** (§4),
keyed off the existing `PINS` map. Immediate fields mirror the EtherCAT image
offsets 1:1; streamed fields are pulled out into the streamed block per
`<motionStream>`.

---

## 7. The motion look-ahead buffer

This is the mechanism that lets the Pi's PREEMPT_RT servo thread jitter (or even
burst/stall briefly) without the drives ever seeing it. It is the **Klipper model**
applied to CSP/CSV/CST: the host plans ahead and streams a queue of future,
time-tagged setpoints; the MCU executes exactly one per SYNC0 tick.

### 7.1 Absolute cycle-index tags + batch refill

Every streamed sample is tagged with the **absolute EtherCAT cycle index** it must
be applied on. The Teensy owns that counter and reports it (`cycle_index`, §6.2).
The Pi keeps the buffer topped to depth `N` by sending, each frame, however many
new samples are needed (`batch refill`), each tagged with its target index.

Why absolute tags + batch (vs "one sample per frame"):

- **Self-healing under jitter / dropped frames.** A missed SPI frame is recovered
  by the next frame sending two samples (`k+9`, `k+10`); "one per frame"
  permanently drains the buffer by one on every miss.
- **Unambiguous late samples.** A sample tagged for an already-passed index is
  simply discarded.

### 7.2 The consumer (Teensy)

On each SYNC0/PIT tick, **before the LRW is built**, the cyclic engine pops the
sample tagged for the current `cycle_index` from each axis ring and writes its
fields into the EtherCAT **output image** via `hal::process_data::write_value`;
the immediate-out fields from the last SPI frame are applied in the same step. The
rest of the tick (build LRW → send → process last reply) then proceeds as today.

Because `Cyclic::tick()` → `send()` → `domain::build_lrw()` is where the LRW is
assembled (`src/ethercat/cyclic.rs`), this pop+apply is a **new hook inside
`Cyclic::tick`** (or a pre-tick call the PIT task makes), not an unchanged path —
`cyclic.rs` will gain this hook (see §11, item 4). Keeping all image writes inside
the prio-3 cyclic task is deliberate: it preserves the architecture invariant that
the highest-priority cyclic task never blocks on the master lock (§11, item 1).

### 7.3 Flow control

`buf_depth[*]` in every status frame is the closed loop: the Pi watches each
axis's fill level and adjusts how far ahead it pushes to hold the target lead `N`.
`N × cycle` must exceed the Pi's **worst-case** scheduling spike (the "max cycle
time" problem), not its average — at 1 kHz, `N = 10` buys 10 ms of cover; at 4 kHz
the same depth is only 2.5 ms, so deeper buffers pair with higher EtherCAT rates.
`N` is a config knob (`<motionStream lead=…>`), tuned empirically against the
telemetry.

### 7.4 Deep buffering with QSPI PSRAM (optional, future)

The Teensy 4.1 has two QSPI footprints on **FlexSPI2** (`EXTMEM`), independent of
the program-flash bus (FlexSPI1) — populating PSRAM (e.g. 8–16 MB) lets the buffer
hold **seconds of motion**, even whole profile segments, rather than a ~10-deep
ring. Design notes:

- Keep a **small hot ring in fast memory** (DTCM/OCRAM) for the next few ticks; use
  PSRAM as the **deep store**. The per-tick consumer pops one sample — trivial
  bandwidth; PSRAM access latency is hidden by the M7 data cache for sequential
  reads.
- This enables a "stream a whole planned move, then top up" mode (more Klipper-like)
  without changing the wire protocol — only the buffer depth bound grows.
- Deferred to a later phase; v1 sizes the ring for DTCM/OCRAM.

### 7.5 Underrun policy

The buffer depth **is** the entire jitter-tolerance window: the Teensy consumes it
normally, and the **first tick that finds the ring empty** triggers a CiA-402
**quick-stop** on the affected axis (§9). No "hold last value for M ticks" — if the
Pi has gone quiet long enough to drain `N` samples, that *is* the fault. The
quick-stop ramp is the per-drive `0x605A` configured in §5.3.

---

## 8. Clocking, sync & the GPIO line

- **The EtherCAT DC SYNC0 on the Teensy is the authoritative motion clock.** The
  Pi's servo thread is **not** phase-locked to it; it free-runs at its nominal rate
  and *works ahead* (§7), so DC jitter and Pi jitter are decoupled by the buffer.
- **`FRAME_READY` GPIO (Teensy → Pi):** informational for v1 — the Teensy pulses it
  when a fresh status frame is staged, so the Pi can align its SPI read or detect a
  stalled Teensy. v1 does **not** gate the Pi servo thread off this line (the
  free-run + look-ahead model makes that unnecessary); a later phase could use it as
  a base-thread tick to phase-lock the Pi if a use case demands it.
- Optional **hardware E-stop GPIO** in parallel with the software watchdog, so a
  safe-state can be asserted independent of SPI.

---

## 9. Safety, watchdog & the unified safe state

All fault sources converge on one deterministic, Teensy-owned **safe-state**
handler that drives the affected axes to **CiA-402 Quick-Stop** (then optionally
Disable), using the controlword logic that lives in `cia402.rs`:

| Trigger | Detected by | Action |
| --- | --- | --- |
| **Motion buffer underrun** | cyclic consumer, ring empty (§7.5) | quick-stop affected axis |
| **Host watchdog timeout** | `host_wdog` not advancing for K cycles | quick-stop all axes; flag in `fault_flags` |
| **EtherCAT fault** | WKC `Incomplete/Zero`, AL error (`cyclic.rs`) | quick-stop / hold SAFE-OP per existing engine |
| **Drive fault** | CiA-402 statusword fault bit | reflect to `fault_flags`; latch |
| **Hardware E-stop** (optional) | dedicated GPIO | immediate quick-stop/disable |

The Teensy is also the **CiA-402 sequencer** (currently `src/ethercat/cia402.rs`
is a stub): it walks Switch-On-Disabled → … → Operation-Enabled on host
`request-enable`, performs fault-reset on request, and owns quick-stop. The Pi
sends *intent* (`flags`) and reads *status*; it never hand-toggles the controlword
through the buffered path.

Bidirectional heartbeat: both `host_wdog` and `teensy_wdog` increment per frame;
each side faults if the other stalls (Pi HAL raises a `*.fault` pin → LinuxCNC
E-stop; Teensy → safe-state as above).

---

## 10. Latency analysis

The look-ahead adds **`N` cycles of fixed latency to the command direction only**.
Be explicit about who pays:

- **Feedback is immediate** (not buffered) — the Pi always reads the freshest
  actuals/status.
- **Command (setpoint) is `N` cycles ahead** — at 1 kHz, `N = 10` ⇒ 10 ms; at 4 kHz
  ⇒ 2.5 ms.
- **Normal contouring / CSP following:** invisible. The trajectory is planned ahead
  and deterministic; a fixed transport delay merely shifts it in time.
- **Feedback-reactive features** (probing, spindle-synced threading/rigid tapping,
  adaptive feed): see the command latency in their loop. Mitigation: keep feedback
  immediate (it is) and let the Pi **shrink the lead** when entering a reactive mode
  (smaller `N` = less latency, less jitter immunity). This must be stated plainly so
  nobody wires a tight loop expecting zero command delay.

The asymmetry — **feedback fresh, command `N`-ahead** — is the whole trick.

> **Trade-off to keep in view.** Shrinking the lead `N` for a reactive mode directly
> reduces the jitter margin that the hard underrun→quick-stop (§7.5) depends on: a
> smaller buffer is more likely to be drained by a Pi latency spike and trip a
> quick-stop. The reactive-mode lead must still cover the Pi's worst-case spike, or
> the mode must accept that risk explicitly.

---

## 11. Firmware changes (Teensy)

Grounded in the current tree (`src/ethercat/`, `src/hal/`, `src/board/`,
`src/main.rs`):

1. **LPSPI slave driver + RTIC task.** New `src/board/host_spi.rs` (LPSPI3 slave,
   DMA or ISR per transaction) + a new RTIC task bound to the LPSPI IRQ. **It must
   not touch the `ecat_master` lock.** The `ecat_master` lock ceiling is 3 (locked
   by the prio-3 `cyclic` task and the prio-1 worker; see `docs/architecture.md`
   §7), and the architecture invariant is that the highest-priority cyclic task
   *never blocks* on it. So the SPI task only DMAs the raw frame in/out of **plain
   shared scratch buffers** (no lock); it does **not** write the cyclic image
   directly. Image writes are deferred to the cyclic task (item 2). Pick the LPSPI
   instance: LPSPI3 (pins 26/27/39) unless the pin-13 indicator LED is relocated.
2. **Bridge module** `src/hal/host_bridge.rs` — the codec between the shared SPI
   buffers and the cyclic image. **The cyclic (prio-3) task owns all image writes:**
   each tick it applies immediate-out fields + popped motion samples to the image
   via `hal::process_data::write_value`, and snapshots the immediate-in fields +
   status header into the outbound SPI buffer via `read_value`, using
   `master.cyclic_image()/cyclic_image_mut()` (both return `Option`, `None` when
   cyclic isn't running — handle that, as `ecat_pd` already does in `src/main.rs`).
   This keeps the cyclic-never-blocks invariant intact (no other task locks the
   master for image access).
3. **Motion buffer** `src/hal/motion_buffer.rs` — per-axis ring of tagged samples,
   `push(sample, target_index)` (from the SPI task, into a buffer the cyclic task
   drains) / `pop(current_index)` (from cyclic task), depth + underrun reporting.
   Fixed-size (`heapless`) for v1; PSRAM-backed deep store later (§7.4).
4. **Cyclic hook** `src/ethercat/cyclic.rs` — add the pre-LRW pop+apply hook (§7.2)
   inside `Cyclic::tick`/`send`. This is a real edit to the cyclic engine, not an
   unchanged path.
5. **Implement `src/ethercat/cia402.rs`** — controlword/statusword state machine,
   fault-reset, quick-stop, driven each cyclic tick; consumed by the safe-state
   handler (§9). (Currently a `// TODO` stub.)
6. **Safe-state handler** — unify the triggers in §9; integrate with the existing
   `Phase::Faulted` handling in `cyclic.rs`.
7. **Generator** `scripts/generate_ethercat_config.py` — parse §5 additions
   (`class`, `<motionStream>`); first lift the **one-PDO-per-SM** limitation (§4)
   so multi-PDO nodes map fully; then emit the SPI frame layout (a Rust module,
   e.g. `src/hal/spi_layout_generated.rs`) and the Pi HAL pin table (§12) alongside
   `generated.rs`.
8. **Config model** `src/ethercat/config/model.rs` — add the streamed-entry / lead
   metadata to `PinCfg`/`SlaveCfg` (or a parallel table) so the layout is
   `'static` and heap-free.

Keep each file short and single-purpose (repo rule). Nothing on the cyclic hot path
allocates, formats, or logs.

---

## 12. Host changes (Pi / LinuxCNC HAL)

- A **realtime HAL component** (C, modeled on `hm2_rpspi`'s SPI access pattern) that,
  in the servo thread: builds the MOSI frame (immediate outputs + batch-refill
  motion samples tagged to `cycle_index + lead`), does one SPI transfer, verifies
  CRC + `seq`, and publishes the immediate-in fields as HAL pins.
- **Pin names = `halPin` names** from the XML (generated table), so the HAL file is
  predictable and matches the firmware exactly.
- Flow control: read `buf_depth[*]`, hold target lead; raise a `*.fault` HAL pin on
  watchdog/CRC/`fault_flags`.
- The generator emits the pin list / a `.h` so the component and firmware never
  drift (§4).

LinuxCNC precedent to follow: SPI-from-realtime-servo-thread to an external motion
device is exactly what `hm2_rpspi` + Mesa does — a proven in-tree pattern.

---

## 13. Phased implementation plan

1. **Transport bring-up.** LPSPI3 slave + a trivial fixed-size loopback frame
   (header + CRC + echo). Validate 40 MHz on the PCB, measure transfer time, prove
   `FRAME_READY` timing. No motion yet.
2. **Immediate-only bridge.** Map the existing image (the verified single-drive
   `generated.rs`) into immediate-in/out; a minimal Pi HAL component reads/writes
   pins. Equivalent capability to the serial `pd` command, but realtime.
3. **CiA-402 + safe state.** Implement `cia402.rs`, host watchdog, quick-stop, and
   the unified safe-state handler. The Pi sends enable intent + reads status.
4. **Look-ahead buffer.** Add `<motionStream>` + `class` to the XML and the
   generator; implement the per-axis ring, batch refill, absolute-index tags, and
   `buf_depth` telemetry/flow-control. Tune `N`.
5. **Multi-axis + IO.** Scale to the full `ethercat-conf.xml` (8 nodes); validate
   WKC and buffer behavior under load.
6. **(Optional) PSRAM deep buffer** (§7.4) and **(optional) Pi phase-lock** off
   `FRAME_READY` (§8) — only if a use case requires them.

Each phase is independently testable on the bench; the user flashes/runs hardware.

---

## 14. Open questions / decisions

Locked from the planning discussion:

- **Bus:** SPI, Teensy LPSPI slave, 40 MHz, GPIO `FRAME_READY` (informational v1). ✔
- **Split:** Hybrid — Teensy owns CiA-402 + safety + EtherCAT timing; Pi owns motion. ✔
- **Contract:** generated from the bus XML (three artifacts, one pass). ✔
- **Stream fill:** batch refill with absolute cycle-index tags. ✔
- **Streamed set:** configurable per drive via `<motionStream>` (CSP/CSV/CST). ✔
- **Underrun:** consume entire buffer, then CiA-402 quick-stop. ✔
- **Pin class marker:** explicit `class="motion"` on `<pdoEntry>`. ✔

To resolve during implementation:

1. **Header field widths + exact CRC** (CRC-16/CCITT is sufficient for ~512-byte
   frames); finalize `magic`/`ver` bytes and `flags`/`fault_flags` bit assignments.
2. **`<streamEntry>` reference style** — by `halPin` name (chosen, readable) and the
   rule that referenced entries must also be `class="motion"`.
3. **Lead scope** — per-axis (`<motionStream lead>`) with a master-level fallback.
4. **LPSPI task priority & ownership** — resolved toward: the SPI task only DMAs
   raw frames in/out of plain shared buffers and **never** locks `ecat_master`; the
   prio-3 cyclic task owns all cyclic-image reads/writes (immediate + popped motion),
   preserving the cyclic-never-blocks invariant (§11, items 1–2). Confirm the exact
   handoff buffers/signaling and pick the LPSPI instance (LPSPI3 unless the LED moves).
5. **PSRAM** — confirm parts/footprint and whether v1 sizes only the DTCM/OCRAM ring.
6. **DMA cache coherency** — same caveat as the ENET path (`docs/pdo-planning-input.md`
   §4.6): any DMA-touched SPI buffers must be in non-cached memory or get explicit
   cache maintenance.

---

**Related docs:** [`docs/architecture.md`](./architecture.md) (RTIC tasks,
priorities, the cyclic engine), [`docs/config-flow.md`](./config-flow.md) (XML →
`generated.rs`), [`docs/pdo-planning-input.md`](./pdo-planning-input.md) (PDO/IgH
mechanics, image sizing, the 4 kHz constraints).
