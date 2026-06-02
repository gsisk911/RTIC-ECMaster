# Teensy Rust EtherCAT Master

A `no_std`, RTIC 2 EtherCAT **master** for the Teensy 4.1 (NXP i.MX RT1062,
Cortex-M7). It is a file-for-file Rust port of the IgH EtherCAT Master
(EtherLab) `master/` core, driven over the Teensy's built-in RMII ENET as raw
Layer-2 EtherCAT frames (EtherType `0x88A4`) — no IP stack and no Linux
`net_device`.

The project began as a generic Teensy Modbus base. That foundation (USB CDC
soft-bootloader, clock/board setup, a Modbus holding-register map, and a
W5500-over-SPI driver) is retained but inactive; the active application is the
EtherCAT master built over the on-chip ENET. The crate is still literally named
`teensy-rust-modbus-base` in `Cargo.toml` and in `FW_NAME` for historical
reasons — renaming is a build-affecting change and out of scope here.

## Status

Implemented and verified on hardware (a YAKO ESD2505PE drive, vendor `0x0994`,
product `0x1B00`):

- **Bus scan.** Counts slaves with a broadcast read (`BRD` of AL status
  `0x0130`), assigns configured station addresses (`APWR` `0x0010`), reads AL
  status + DL/base info, and reads SII identity (vendor / product / revision)
  and standard mailbox configuration. Runs once at boot and on demand via
  `rescan`.
- **CoE SDO read/write** over the slave mailbox, as non-blocking state machines
  (the IgH `exec()`-per-step model, one datagram per step via a `Pump`
  primitive). Includes automatic mailbox sync-manager configuration and an
  `INIT`→`PRE-OP` bring-up before the transfer, plus SDO abort handling.
  Verified: read `0x1000` device type = `0x00040192`; wrote and read back
  `0x6060`; a bad index returns SDO abort `0x06020000`.
- **Cyclic PDO process data** (new; builds, pending hardware verification). A
  per-slave bring-up FSM (INIT→PRE-OP→SAFE-OP) clears/configures FMMUs and sync
  managers, applies SDO init values, writes PDO assignment (`0x1C12`/`0x1C13`)
  and mapping (`0x1600`/`0x1A00`) over CoE, and sets up **Distributed Clocks
  SYNC0**; a **PIT-timer-driven cyclic engine** then exchanges the process-data
  image with one LRW per cycle, gates SAFE-OP→OP on a healthy working counter,
  and exposes the image as named pins. The desired bus is fixed at compile time
  (see [Compile-time bus configuration](#compile-time-bus-configuration)).
- **Serial command interface** mirroring the IgH `ethercat` CLI tool (`slaves`,
  `rescan`, `upload`, `download`, `states`, plus `start`, `stop`, `pdos`, `pd`,
  `help`). See [Serial command interface](#serial-command-interface).

The PDO/DC/cyclic path is implemented and the firmware builds, but has **not yet
been verified on hardware** — that is the next bring-up step (DC SYNC0 start-time
math is the highest-risk piece). Multi-slave drift compensation, CiA 402 motion,
and segmented SDO remain deferred; see `docs/ethercat-v1-followups.md`.

## Hardware and transport

- **Board:** Teensy 4.1 (i.MX RT1062 Cortex-M7), target `thumbv7em-none-eabihf`,
  `target-cpu=cortex-m7`.
- **Transport:** the built-in RMII ENET (`ENET1`) carries EtherCAT frames. The
  master sends raw Ethernet frames with a broadcast destination, a fixed
  locally-administered source MAC (`02:00:00:00:00:01`), and EtherType `0x88A4`.
  Frames are exchanged through the vendored ENET DMA driver in `src/net/`
  (`send_raw` / `poll_raw`); there is no smoltcp/IP path on the EtherCAT side.
- **USB:** enumerates as a PJRC USB CDC serial device (VID/PID `16C0:0483`) for
  the command interface and the soft-bootloader trigger.

## Architecture

The EtherCAT code lives in `src/ethercat/` in a single flat directory that
mirrors IgH's `master/` layout (no subfolders). Kernel-only glue (`module.c`,
`cdev.c`, `ioctl.c`, RTDM) is dropped; locking becomes RTIC resources /
`critical-section`, allocation becomes `heapless` and fixed-size arrays, and the
`EC_DBG`/`EC_ERR` macros become the `log` facade. Each module's doc comment
records the IgH source file it ports and what was adapted or dropped.

### Implemented modules

| Module | IgH source | Role |
| --- | --- | --- |
| `src/ethercat/ecrt.rs` | `include/ecrt.h` | Public API types (`EcError`, slave/PDO/sync info) and little-endian access helpers (`read_*`/`write_*`). |
| `src/ethercat/globals.rs` | `master/globals.h` | Core constants, fixed `EC_MAX_*` capacities, AL states, and the ESC register / SII map. |
| `src/ethercat/datagram.rs` | `master/datagram.c` | `Command` enum plus `build`/`parse` for one datagram per EtherCAT frame. |
| `src/ethercat/device.rs` | `master/device.c` | Transport seam over the ENET driver: Ethernet header framing, `transact` (blocking, scan-only), and the non-blocking `pump` primitive. |
| `src/ethercat/slave.rs` | `master/slave.c` | `SlaveInfo` (the scan identity/base subset) and `Mailbox` parameters. |
| `src/ethercat/mailbox.rs` | `master/mailbox.c` | 6-byte mailbox header build/parse and protocol multiplexing. |
| `src/ethercat/sync.rs` | `master/sync.c` | Sync-manager configuration page encoding (mailbox SMs). |
| `src/ethercat/master.rs` | `master/master.c` | Top-level master: owns the transport and discovered slaves, runs the blocking scan, and drives runtime `Request`s as non-blocking `Op` steppers (`poll_op`). |
| `src/ethercat/fsm_master.rs` | `master/fsm_master.c` | Scan orchestration: count slaves, clear station addresses, scan each slave. |
| `src/ethercat/fsm_slave_scan.rs` | `master/fsm_slave_scan.c` | Per-slave scan: assign address, read AL status + DL/base info, read SII identity and mailbox config. |
| `src/ethercat/fsm_sii.rs` | `master/fsm_sii.c` | SII/EEPROM read path via the ESC SII registers. |
| `src/ethercat/fsm_coe.rs` | `master/fsm_coe.c` | Non-blocking expedited (≤ 4-byte) SDO upload/download with abort handling. |
| `src/ethercat/fsm_change.rs` | `master/fsm_change.c` | Non-blocking AL state-change handshake (write control, poll status, read status code). |
| `src/ethercat/cli.rs` | `tool/` (`ethercat` CLI) | Serial line parser → master `Request`s (interface layer, not an IgH `master/` file). |

### Scaffolded modules

These carry their IgH-sourced doc comments and any shared type/enum definitions,
but their bodies are `TODO` and are filled in as the master grows toward cyclic
PDO exchange:

- Configuration model + parser: `src/ethercat/config/` (`model.rs`,
  `parser.rs`) — turns `ethercat-conf.xml` into typed config (a project
  addition, not an IgH file).
- Desired-config + process data: `src/ethercat/slave_config.rs`,
  `src/ethercat/sync_config.rs`, `src/ethercat/fmmu_config.rs`,
  `src/ethercat/domain.rs`.
- PDO / SDO object model: `src/ethercat/pdo.rs`, `src/ethercat/pdo_entry.rs`,
  `src/ethercat/pdo_list.rs`, `src/ethercat/sdo.rs`, `src/ethercat/sdo_entry.rs`,
  `src/ethercat/sdo_request.rs`.
- State machines: `src/ethercat/fsm_slave.rs` (per-slave dispatcher),
  `src/ethercat/fsm_slave_config.rs` (INIT→OP bring-up),
  `src/ethercat/fsm_pdo.rs`, `src/ethercat/fsm_pdo_entry.rs`.
- Distributed clocks: `src/ethercat/dc.rs`.
- Application layer: `src/ethercat/cia402.rs` (CiA 402 drive state machine; an
  app-layer file that will move under an interface layer once one exists).

### RTIC tasks

The RTIC application is in `src/main.rs`:

- `init` — brings up the clocks, LEDs, USB, and ENET, constructs the `Master`,
  and spawns the heartbeat and EtherCAT tasks.
- `ethercat_worker` (priority 1) — waits (bounded) for the PHY link, runs the
  initial bus scan, then loops: take a parsed command, step its non-blocking FSM
  to completion (yielding the executor between datagrams), and publish the
  response.
- `usb_isr` (binds `USB_OTG1`, priority 2) — polls the USB CDC device, reads
  typed input, parses a command on each line and queues it for the worker, and
  returns command responses promptly. It also emits a short boot/identity report
  and the scan summary, and periodically prints a health report.
- `blink_leds` (priority 1) — LED heartbeat.
- `poll_w5500` (priority 1) — the legacy Modbus-TCP-over-W5500 poller; defined
  but **not** spawned (the on-chip ENET is the EtherCAT transport instead).

## Serial command interface

Connect to the USB CDC serial port and type commands. The interface mirrors the
forms of the IgH userspace `ethercat` tool. Numeric arguments are parsed
base-from-prefix: `0x…` hex, `0b…` binary, otherwise decimal.

| Command | Description |
| --- | --- |
| `slaves` | List discovered slaves (position, station, vendor/product, AL state, CoE support). |
| `rescan` | Re-run the bus scan. |
| `states -p<pos> <INIT\|PREOP\|SAFEOP\|OP>` | Request an AL state on a slave. |
| `upload -p<pos> [-t<type>] <index> <sub>` | SDO read. With a type, prints the typed value; without one, raw hex bytes. |
| `download -p<pos> -t<type> <index> <sub> <value>` | SDO write of an expedited value. |
| `start [-p<pos>]` | Configure the slave to SAFE-OP and start the cyclic PDO engine (drives toward OP). |
| `stop` | Stop the cyclic PDO engine. |
| `pdos` | List the resolved process-data pins (name, image offset, bit length, direction). |
| `pd [<pin> [<value>]]` | No args: dump the process image + cyclic status. `<pin>`: read a named pin. `<pin> <value>`: write an output pin. |
| `status` (or `info`) | Print firmware tag, link state, slave count, and cyclic phase/WKC. |
| `help` (or `?`) | Print the command list. |

While the cyclic engine is running, bus-mutating commands (`rescan`, `states`,
`upload`, `download`, `start`) are rejected — `stop` first. `pd`/`pdos`/`slaves`/
`status` stay available (they read the image or topology, not the live bus).

Supported SDO types (expedited, ≤ 4 bytes): `bool`, `int8`, `int16`, `int32`,
`uint8`, `uint16`, `uint32`. `upload`/`download` also accept the short aliases
`up`/`down`, and `states` accepts `state`.

Upload output follows the IgH `outputData` form, `0x<hex> <decimal>` (hex width
matches the type). For example:

```text
upload -p0 -tuint16 0x6041 0
[ecat] 0:0x6041:00 = 0x0000 0

download -p0 -tint8 0x6060 0 8
[ecat] 0:0x6060:00 written
```

The console is request/response, like the IgH `ethercat` tool: on USB attach the
firmware prints a one-time boot banner, and once the initial bus scan finishes it
prints a one-time scan summary. After that it stays quiet — each command you type
returns a single, immediate response with no background streaming. The firmware
does not echo input, so enable your terminal's local echo to see what you type.

### Limitations

- State changes are requested as a single step. `INIT` ↔ `PRE-OP` works, but
  multi-step jumps (e.g. `OP` from `INIT`) are not yet driven through the
  intermediate states and will report an AL status code. Reaching `SAFE-OP`/`OP`
  also needs PDO/FMMU configuration, which is part of the cyclic phase.
- SDO transfers are expedited only (≤ 4 bytes); segmented / complete-access
  transfers are deferred.

## Compile-time bus configuration

The cyclic PDO layer is configured entirely at **compile time** — there is no XML
parser on the MCU. A small Python generator turns a LinuxCNC/lcec-style bus
description plus the vendor ESI into a Rust constant table:

```text
ethercat-conf.bohign.xml  ─┐
                           ├─►  scripts/generate_ethercat_config.py  ─►  src/ethercat/config/generated.rs
Bohign_MS_ECAT_V2.5.xml   ─┘                (make config)
```

- `ethercat-conf.bohign.xml` — the desired bus (master cycle, slave vid/pid, DC
  config, SDO init values, and the `syncManager`/`pdo`/`pdoEntry` mapping with
  `halPin` names). Same dialect as `ethercat-conf.xml`.
- `Bohign_MS_ECAT_V2.5.xml` — the vendor ESI; supplies each slave's SM2/SM3
  physical start addresses and control bytes.
- The generator matches each slave to its ESI device by product code, computes
  every entry's `(byte_offset, bit_position)` in the process image, validates
  SDO-init payloads fit an expedited transfer (≤ 4 bytes), and emits
  `generated.rs`. The output is committed; regenerate with `make config` and
  never hand-edit it.

The v1 config targets **one YAKO ESD2505PE** drive (`0x00000994` / `0x00001B00`)
with the stock RxPDO1 (`0x1600`, 16 B) + TxPDO1 (`0x1A00`, 39 B) mapping, a
55-byte process image, a **100 Hz** cycle, and DC SYNC0. The cycle is hardware-
timed by **PIT channel 0** (24 MHz oscillator) and the engine is built for up to
**4 kHz**: change `appTimePeriod` in the XML (`250000` = 4 kHz) and re-run
`make config`.

## Build

The project builds with Cargo for the `thumbv7em-none-eabihf` target. The
target, linker script, and CPU are pinned in `.cargo/config.toml`, so a plain
build needs no extra flags:

```sh
cargo build --release
# or, to also produce the flashable .hex:
make
```

`make` (default target `hex`) builds the release ELF and runs `rust-objcopy` to
emit `target/thumbv7em-none-eabihf/release/teensy-rust-modbus-base.hex`.

### Compile-time configuration

`.cargo/config.toml` pins the build and supplies the firmware's compile-time
configuration through `[env]` variables, which `src/main.rs` reads with `env!()`
const parsers:

- `TEENSY4_CORE_CLOCK_HZ` — core clock (default `600000000`). Profiles above
  600 MHz also require `TEENSY4_ALLOW_OVERCLOCK=true`, and the highest-voltage
  profiles require `TEENSY4_ALLOW_MAX_VOLTAGE=true`.
- `LED_INDICATOR_PIN`, `BASE_LED_A_PIN`, `BASE_LED_B_PIN`, `BASE_LED_BLINK_HZ` —
  heartbeat LED pins and rate.
- `W5500_SPI_HZ`, `W5500_RESET_PIN`, `W5500_INT_PIN` — legacy W5500 SPI settings
  (the reset/interrupt pins are compile-time asserted to Teensy pins 40/41).

`build.rs` also captures a git build-provenance tag into the `FW_TAG`
environment (`v<pkg>-g<short-sha>[-dirty]`, or `v<pkg>-nogit` outside a git
checkout) so the exact running build is identifiable in the boot report.

## Flash

```sh
make flash
```

`make flash` builds the `.hex`, requests the soft bootloader (via
`tools/soft_reboot_teensy.py`), and programs the board with
`teensy_loader_cli -mmcu=imxrt1062 -w`. You can also request the bootloader
directly from a host:

```sh
teensy_loader_cli -s
```

The physical Program button remains the fallback recovery path. Do not flash or
run firmware unless you are intentionally testing on hardware.

## Soft Bootloader

The firmware enumerates as PJRC USB CDC serial VID/PID `16C0:0483` so host tools
can request HalfKay/program mode without pressing the physical Program button.
The trigger is not ASCII text written to the serial stream. It is a USB CDC
class control request:

- Request: `SET_LINE_CODING` (`0x20`)
- Baud: `134`
- Stop bits: `1`
- Parity: none
- Data bits: `8`

`src/board/usb_bootloader.rs` provides a pass-through monitor that must be first
in the USB class poll chain. It observes endpoint-zero control requests before
the CDC class accepts them. The observer does not accept, reject, own the USB
device, set a VID/PID, or read from the CDC bulk OUT endpoint.

After the request is latched, `src/main.rs` drives the configured LED outputs low
and calls the no-return bootloader path in `src/board/usb_bootloader.rs`. That
path disables interrupts and executes the Teensy 4.x bootloader breakpoint
`bkpt #251`.

Reliability limits: this can preempt locked lower-priority RTIC tasks while USB
interrupts and the USB peripheral are still functioning. It cannot recover from
hard faults, globally disabled interrupts, clock failure, wedged USB hardware, or
a higher-priority interrupt that never returns. Keep the physical Program button
as the fallback recovery path.

## Legacy Modbus base (inactive)

The original Modbus foundation is still compiled in but not started:

- The holding-register map lives in `src/modbus/register_map.rs` (mirrored in
  `src/modbus/registers.json`) and defines IP / subnet / gateway / unit-id /
  status registers.
- The W5500 SPI chip is brought up and reported at boot, but the
  Modbus-TCP-over-W5500 poller (`poll_w5500`) is not spawned, so no Modbus
  traffic is served. Re-enabling it is a one-line `poll_w5500::spawn()` change.

## Configuration files

- `.cargo/config.toml` — build target, linker, CPU, and the compile-time `[env]`
  configuration described above. (The root `config.toml` is only a pointer to
  it.)
- `ethercat-conf.xml` — an IgH/EtherLab-style desired-bus description (slaves,
  sync managers, PDO mappings, SDO init values, DC config). It is the input the
  `src/ethercat/config/` parser will consume to drive per-slave configuration;
  it is not yet read by the firmware.

## Further reading

- `docs/ethercat-v1-followups.md` — deferred correctness-hardening, robustness,
  and test-coverage follow-ups from the v1 scan and CoE/SDO reviews against IgH
  `stable-1.6`.
- Each `src/ethercat/*.rs` module header documents the IgH source it ports and
  the kernel-only pieces that were adapted or dropped.
