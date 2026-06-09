# Teensy Rust EtherCAT Master

A bare-metal `no_std`, RTIC 2 EtherCAT **master** for the Teensy 4.1 (NXP
i.MX RT1062, Cortex-M7). It is a file-for-file Rust port of the IgH EtherCAT
Master (EtherLab) `master/` core, driven over the Teensy's built-in RMII ENET as
raw Layer-2 EtherCAT frames (EtherType `0x88A4`) — no IP stack and no Linux
`net_device`.

It works end-to-end on hardware today: it scans the bus, does CoE SDO
read/write, walks the EtherCAT state machine (INIT → PRE-OP → SAFE-OP → OP),
performs full slave configuration (sync managers, FMMUs, PDO assignment/mapping
over CoE, Distributed Clocks SYNC0), and runs a PIT-timer-driven cyclic
process-data engine with named, typed read/write of the process image over a
serial command line.

> **A note on the crate name.** The project began as a generic Teensy Modbus
> base, so the crate is still literally named `teensy-rust-modbus-base` in
> `Cargo.toml` (and in `FW_NAME` / the produced `.hex`). Renaming is a
> build-affecting change and is intentionally out of scope. The active
> application is the EtherCAT master; the Modbus/W5500 foundation is retained but
> inactive (see [Legacy Modbus base](#legacy-modbus-base-inactive)).

---

## Status

**Verified on hardware** against a single **YAKO ESD2505PE** servo drive (also
sold under the Bohign label; vendor `0x00000994`, product `0x00001B00`,
revision `0x00000001`):

- **Bus scan** — broadcast slave count, station-address assignment, AL status,
  DL/base info, and SII identity (vendor / product / revision / mailbox config).
  Runs on demand via `rescan`, streaming each sub-step over serial.
- **CoE SDO read/write** — expedited (≤ 4-byte) SDO upload/download over the
  slave mailbox, with automatic mailbox sync-manager setup, an INIT → PRE-OP
  bring-up, and SDO-abort handling.
- **AL state machine** — single-step AL transitions (`states`), plus the full
  multi-stage bring-up driven by `start`.
- **Full slave configuration** — clears FMMUs/DC, configures mailbox + process
  data sync managers (SM2/SM3), applies SDO init values, writes PDO assignment
  (`0x1C12`/`0x1C13`) and mapping (`0x1600`/`0x1A00`) over CoE, sets the
  watchdog, programs the FMMUs, and brings up **Distributed Clocks SYNC0**.
- **Cyclic process data** — a **PIT-timer-driven** engine exchanges the process
  image with one LRW per cycle, gates SAFE-OP → OP on a healthy working counter,
  and exposes the image as named, typed pins. **Verified at ~100 Hz with
  `wkc = 3/3`.** The engine is architected for much higher rates (the cycle is
  set in the bus XML; the hardware timer is sized for up to ~4 kHz / 250 µs).

### Scope and limitations (v1)

- **Tested topology is a single drive.** The compile-time configuration targets
  one YAKO/Bohign drive. The architecture (process-image domain, per-slave
  config FSM, working-counter math) is built for multiple slaves, but multi-slave
  operation is not yet hardware-verified, and cross-slave DC drift compensation
  (ARMW/FRMW) is deferred.
- **Cycle rate.** Verified at 100 Hz. The design target is up to ~4 kHz; higher
  rates are a change of the configured cycle plus on-hardware validation.
- **CiA-402 motion control is deferred.** The firmware exchanges process data and
  exposes the CiA-402 objects as pins (controlword, statusword, target/actual
  position, etc.), but the controlword/statusword *drive state machine*
  (`src/ethercat/cia402.rs`) is scaffolding, not an active control loop. You
  enable the drive by writing the controlword by hand (see the
  [Quick start](#quick-start--using-it-as-a-driver)).
- **SDO transfers are expedited only** (≤ 4 bytes). Segmented / complete-access
  transfers are deferred.

See [`docs/ethercat-v1-followups.md`](docs/ethercat-v1-followups.md) for the full
list of deferred correctness/robustness follow-ups.

---

## Hardware and transport

- **Board:** Teensy 4.1 (i.MX RT1062 Cortex-M7), target `thumbv7em-none-eabihf`,
  `target-cpu=cortex-m7`, core clock 600 MHz.
- **Transport:** the built-in RMII ENET (`ENET1`) carries EtherCAT frames. The
  master sends raw Ethernet frames with a broadcast destination, a fixed
  locally-administered source MAC (`02:00:00:00:00:01`), and EtherType `0x88A4`.
  There is no smoltcp/IP path on the EtherCAT side.
- **Wiring:** connect the Teensy 4.1 Ethernet (RMII magjack/PHY) directly to the
  EtherCAT drive's IN port. No switch.
- **USB:** enumerates as a PJRC USB CDC serial device (VID/PID `16C0:0483`) for
  the command interface and the soft-bootloader trigger.

---

## Quick start — using it as a driver

This walks from a flashed board to reading and writing live process data.

1. **Build, configure, and flash** (details in [Build & flash](#build-flash--config)):

   ```sh
   make config     # only if you changed the bus XML; regenerates generated.rs
   make flash      # builds the .hex, soft-reboots to bootloader, programs it
   ```

2. **Open the serial console** (115200 baud; the firmware does not echo, so
   enable local echo). Any interactive terminal works:

   ```sh
   python -m serial.tools.miniterm --raw 115200    # pick the 16C0:0483 port
   ```

   To watch output with timestamps (read-only logger), see
   [`docs/serial-monitoring.md`](docs/serial-monitoring.md).

   On attach you get a one-time banner:

   ```text
   [boot] teensy-rust-modbus-base 0.1.0 (v0.1.0-g1a2b3c4)
   [boot] EtherCAT master over RMII ENET; type 'help' for commands
   ```

3. **Scan the bus** to discover the drive:

   ```text
   rescan
   ```

   ```text
   [scan] counting slaves
   [scan] count=1
   [scan] addresses cleared
   [scan] s1: addr set
   [scan] s1: al=0x01
   [scan] s1: vendor=0x00000994
   [scan] s1: product=0x00001B00
   [scan] s1: proto=0x0004 coe=1
   [ecat] rescan complete: 1 slave(s); type 'slaves'
   ```

4. **Configure + start cyclic process data** on slave 0 (this runs the full
   INIT → SAFE-OP bring-up and starts the PIT cyclic engine, which then drives
   the drive to OP):

   ```text
   start -p0
   ```

   ```text
   [ecat] slave 0 configured; cyclic PDO started
   ```

   Confirm it reached OP with a full working counter:

   ```text
   status
   ```

   ```text
   [ecat] fw 0.1.0 (v0.1.0-g1a2b3c4)
   [ecat] link=up slaves=1
   [ecat] cyclic OP wkc=3/3 cycles=12840
   ```

5. **Read and write the process image by pin name.** List the pins with `pdos`;
   read an input; write an output:

   ```text
   pd drive0-statusword
   [ecat] drive0-statusword = 569 (0x239)

   pd drive0-controlword 15
   [ecat] drive0-controlword <= 15
   ```

   `pd` with no arguments dumps the whole process image plus cyclic status. To
   bring a CiA-402 drive to *Operation Enabled*, write the controlword fault-reset
   then the enable sequence (`0x80` → `0x06` → `0x07` → `0x0F`) and watch the
   statusword. Stop the engine with `stop` before issuing any bus command again.

Full command documentation: [`docs/cli-reference.md`](docs/cli-reference.md).

---

## Build, flash & config

The project builds with Cargo for `thumbv7em-none-eabihf`. The target, linker
script, and CPU are pinned in `.cargo/config.toml`, so a plain build needs no
extra flags.

| Command | What it does |
| --- | --- |
| `make` / `make hex` | Release build, then `rust-objcopy` to `target/thumbv7em-none-eabihf/release/teensy-rust-modbus-base.hex`. |
| `make config` | Regenerate `src/ethercat/config/generated.rs` from the bus XML + vendor ESI. Run after editing the XML, then commit the result. |
| `make flash` | `make hex`, request the soft bootloader, then `teensy_loader_cli -mmcu=imxrt1062 -w`. |
| `make reboot` | Soft-reboot the firmware (normal restart) without flashing. |
| `cargo build --release` | Build the ELF only (no `.hex`). |

**Flashing** uses a host-driven soft reboot into HalfKay: `tools/soft_reboot_teensy.py`
opens the CDC port at **134 baud** (a USB CDC line-coding request, not serial
text), which the firmware interprets as "enter bootloader". Then
`teensy_loader_cli -w` programs the `.hex`. The physical **Program button** is
always the fallback recovery path.

> **Why a 64 KB stack?** `.cargo/config.toml` sets `TEENSY4_STACK_SIZE = "65536"`.
> The IgH-derived FSMs carry multi-KB scratch buffers (the master's `poll_op`
> moves a ~1.7 KB `Op` local onto the stack, plus per-FSM `tx`/`rx` scratch); the
> 16 KB default overflowed during the deep startup/scan call path. DTCM is 320 KB,
> so this leaves ample room. See
> [`docs/architecture.md`](docs/architecture.md#9-build-time-knobs).

Other compile-time knobs (core clock, LED pins) and the `FW_TAG` git-provenance
stamp are documented in [`docs/config-flow.md`](docs/config-flow.md) and
[`docs/architecture.md`](docs/architecture.md).

> Per the project rule, **do not flash or run firmware unless you are
> intentionally testing on hardware.**

---

## Serial CLI cheat-sheet

Connect to the USB CDC port and type commands; the forms mirror the IgH
userspace `ethercat` tool. Numbers are base-from-prefix (`0x…` hex, `0b…` binary,
else decimal). The console is request/response and does not echo.

| Command | Description |
| --- | --- |
| `help` (`?`) | Print the command list. |
| `slaves` | List discovered slaves (position, station, vendor/product, AL state, CoE). |
| `status` (`info`) | Firmware tag, link state, slave count, cyclic phase/WKC. |
| `rescan` | Re-run the bus scan (streams progress). |
| `states -p<pos> <INIT\|PREOP\|SAFEOP\|OP>` | Request an AL state on a slave. |
| `upload -p<pos> [-t<type>] <idx> <sub>` | SDO read (typed value, or raw hex). |
| `download -p<pos> -t<type> <idx> <sub> <value>` | SDO write (expedited). |
| `start [-p<pos>]` | Configure the slave + start the cyclic PDO engine (drives to OP). |
| `stop` | Stop the cyclic PDO engine. |
| `pdos` | List process-data pins (name, image offset, bit length, direction). |
| `pd [<pin> [<value>]]` | Dump image / read pin / write output pin. |
| `crashlog` / `crashclear` | Show / clear the saved fault/panic context. |

SDO types (expedited, ≤ 4 bytes): `bool`, `int8`, `int16`, `int32`, `uint8`,
`uint16`, `uint32`.

While the cyclic engine is running, **bus-mutating** commands (`rescan`,
`states`, `upload`, `download`, `start`) are rejected — `stop` first.
`pd` / `pdos` / `slaves` / `status` stay available (they read the image or
cached topology, not the live bus).

> A parallel effort is adding a cycle-rate option to `start` and a cyclic
> **telemetry / `stats`** view (jitter, DC sync error). Those are described at a
> high level in [`docs/cli-reference.md`](docs/cli-reference.md#in-progress); the
> exact syntax there is marked **TODO — verify against the final `cli.rs`**.

See the full reference with example output:
[`docs/cli-reference.md`](docs/cli-reference.md).

---

## Crash diagnostics

The firmware persists the last fault/panic context across a reboot and replays it
on demand with `crashlog`:

- A **CPU HardFault** records the stacked exception frame + SCB fault-status
  registers, then **auto-reboots** (recoverable over USB); the dump is retrievable
  with `crashlog` on the next boot.
- A **Rust panic** records the message and **halts** on an LED code (it does not
  reboot, to avoid a no-USB boot loop); the message is retrievable with `crashlog`
  after a manual reboot.

```text
crashlog
[crash] HARDFAULT pc=0x6000A1B2 lr=0x6000A0FF frame_sp=0x20003F80 msp=0x20003F80
[crash] cfsr=0x00008200 hfsr=0x40000000 bfar=0x00000000 mmfar=0x00000000
[crash] r0=0x00000000 r1=0x20001234 r2=0x00000004 r3=0x00000000
[crash] r12=0x00000000 xpsr=0x61000000 send_stage=2
```

Field meanings, the stack-overflow hint, and the on-board LED fault codes are in
[`docs/architecture.md`](docs/architecture.md#10-crash-diagnostics--fault-handling).

---

## Documentation

| Doc | Contents |
| --- | --- |
| [`docs/architecture.md`](docs/architecture.md) | IgH file-for-file mapping, the non-blocking FSM/stepper model, the RTIC task layout and priorities, the cooperative boot, the cyclic engine, and crash handling. |
| [`docs/config-flow.md`](docs/config-flow.md) | How the bus XML + vendor ESI become `generated.rs` (the `BUS` table and `PINS` map), the config structs, and how to retarget a different drive. |
| [`docs/cli-reference.md`](docs/cli-reference.md) | Every serial command with real example output. |
| [`docs/serial-monitoring.md`](docs/serial-monitoring.md) | Using `scripts/view_teensy_serial.py` to watch the bring-up and cyclic output live. |
| [`docs/ethercat-v1-followups.md`](docs/ethercat-v1-followups.md) | Deferred correctness/robustness/test follow-ups from the v1 reviews. |
| [`docs/pdo-planning-input.md`](docs/pdo-planning-input.md) | Historical planning brief for the PDO feature (IgH mechanics, register cheat-sheet). |

Each `src/ethercat/*.rs` module header also documents the IgH source file it
ports and the kernel-only pieces that were adapted or dropped.

---

## Repository layout

```text
src/
  main.rs                  RTIC app: init, ethercat_worker, usb_isr, cyclic (PIT), blink_leds
  ethercat/                file-for-file mirror of IgH master/ (flat, no subfolders)
    config/                compile-time bus config: model.rs (+ generated.rs from make config)
  hal/                     named-pin (process-data) layer over the domain image
  board/                   clocks, fast GPIO, pin map, PIT cycle timer, USB soft-bootloader
  net/                     RMII ENET driver (raw L2 transport); legacy W5500 SPI (inactive)
  modbus/                  legacy Modbus register map (inactive)
scripts/
  generate_ethercat_config.py   bus XML + ESI -> src/ethercat/config/generated.rs
  view_teensy_serial.py         read-only timestamped serial logger
tools/
  soft_reboot_teensy.py         host-driven reboot / bootloader trigger
ethercat-conf.bohign.xml   the desired bus (lcec/LinuxCNC dialect)
Bohign_MS_ECAT_V2.5.xml    vendor ESI (device description)
```

---

## Legacy Modbus base (inactive)

The original Modbus/W5500 foundation is still in the tree but is **not brought up
or run**: `init` deliberately skips the W5500 SPI / Modbus path (it would hang on
a board with no W5500 chip, and its SCK shares the pin-13 LED), and no Modbus task
is spawned. The holding-register map (`src/modbus/register_map.rs`, mirrored in
`src/modbus/registers.json`) and the W5500 SPI driver (`src/net/w5500_spi.rs`)
remain for reference; the crate still depends on `rmodbus`. Re-enabling Modbus is
out of scope for the EtherCAT firmware.

---

## License / provenance

This is a Rust port of the IgH EtherCAT Master (EtherLab), branch `stable-1.6`.
The kernel-only glue (`module.c`, `cdev.c`, `ioctl.c`, RTDM) is dropped; locking
becomes RTIC resources / `critical-section`, allocation becomes `heapless` and
fixed arrays, and the `EC_DBG`/`EC_ERR` macros become the `log` facade.
