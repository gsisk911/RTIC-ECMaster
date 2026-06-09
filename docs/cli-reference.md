# Serial CLI reference

The firmware exposes a serial command line over the USB CDC port, mirroring the
forms of the IgH userspace `ethercat` tool. This documents every command and its
real output. The parser is in [`src/ethercat/cli.rs`](../src/ethercat/cli.rs);
the command handlers are in [`src/main.rs`](../src/main.rs) (`ecat_run_command`).

For how to connect a terminal and watch the output, see
[`serial-monitoring.md`](serial-monitoring.md).

---

## Conventions

- **Connection:** USB CDC serial, **115200 baud** (VID/PID `16C0:0483`).
- **No echo.** The console is request/response: it stays silent until a command
  produces a response. Enable your terminal's **local echo** to see what you type.
- **One response per line entered.** Submit a command with Enter (`\r`/`\n`);
  backspace/delete edit the current line.
- **Numbers are base-from-prefix:** `0x…` hex, `0b…` binary, otherwise decimal.
  This applies to indices, subindices, values, and `pd` values.
- **All responses are prefixed** `[ecat]`, `[scan]`, or `[crash]`.
- **On USB attach** the firmware prints a one-time banner, and after a scan
  completes, a one-time scan summary. Otherwise it only speaks when spoken to.

```text
[boot] teensy-rust-modbus-base 0.1.0 (v0.1.0-g1a2b3c4)
[boot] EtherCAT master over RMII ENET; type 'help' for commands
```

### The "cyclic busy" rule

While the cyclic PDO engine is running, **bus-mutating** commands are rejected so
they can't disturb the live cycle:

```text
rescan
[ecat] cyclic active; 'stop' before bus commands
```

Rejected while running: `rescan`, `states`, `upload`, `download`, `start`. Still
available: `pd`, `pdos`, `slaves`, `status` (they read the image or cached
topology, not the live bus). Use `stop` first to run a bus command.

---

## Commands

### `help` (alias `?`)

Prints the command list:

```text
[ecat] commands (IgH ethercat tool form):
  slaves                                   list discovered slaves
  status                                   firmware, link state, slave count
  rescan                                   re-run the bus scan
  states -p<pos> <INIT|PREOP|SAFEOP|OP>    request an AL state
  upload -p<pos> -t<type> <idx> <sub>      SDO read (0x.. hex, else decimal)
  download -p<pos> -t<type> <idx> <sub> <value>   SDO write
  start [-p<pos>]                          configure + start cyclic PDO (to OP)
  stop                                     stop cyclic PDO
  pdos                                     list process-data pins and offsets
  pd [<pin> [<value>]]                     dump image / read pin / write pin
  crashlog                                 show the saved fault/panic context
  crashclear                               clear the saved fault/panic context
  types: bool int8 int16 int32 uint8 uint16 uint32
```

### `slaves`

List the slaves discovered by the last scan (cached; does not touch the bus).

```text
slaves
[ecat] 1 slave(s)
[ecat] slave 0 station=1 vid=0x00000994 pid=0x00001B00 al=0x01 coe=yes
```

Fields: ring position, station address (`= position + 1`), vendor ID, product
code, AL-status byte (`0x01`=INIT, `0x02`=PRE-OP, `0x04`=SAFE-OP, `0x08`=OP), and
CoE support. Empty until you `rescan`.

### `status` (alias `info`)

Firmware tag, PHY link state, slave count, and (if running) the cyclic snapshot.

```text
status
[ecat] fw 0.1.0 (v0.1.0-g1a2b3c4)
[ecat] link=up slaves=1
[ecat] cyclic OP wkc=3/3 cycles=128407
```

The third line appears only while the cyclic engine runs. `wkc=3/3` is
observed/expected working counter; phase is `priming`, `requesting-op`, `OP`, or
`faulted`.

### `rescan`

Re-run the bus scan, **streaming each sub-step** over serial as it completes (the
primary no-SWD diagnostic). Must be run before `start`.

```text
rescan
[scan] rescan: start
[scan] rescan: begun (FSM built)
[scan] counting slaves
[scan] count=1
[scan] addresses cleared
[scan] s1: addr set
[scan] s1: al=0x01
[scan] s1: base type=0x05 fmmu=3 sm=4
[scan] s1: vendor=0x00000994
[scan] s1: product=0x00001B00
[scan] s1: rev=0x00000001
[scan] s1: rxmbox off=0x1000 sz=128
[scan] s1: txmbox off=0x1080 sz=128
[scan] s1: proto=0x0004 coe=1
[ecat] rescan complete: 1 slave(s); type 'slaves'
```

`s1` is the station address. On failure a line like
`[ecat] error: working counter` (or another `EcError`) ends the stream.

### `states -p<pos> <INIT|PREOP|SAFEOP|OP>` (alias `state`)

Request a single AL state transition on a slave.

```text
states -p0 PREOP
[ecat] slave 0 -> PREOP
```

EtherCAT only allows single-step transitions, and this command requests the
target directly. `INIT` ↔ `PREOP` works; a multi-step jump (e.g. `OP` from
`INIT`) returns an AL status code:

```text
states -p0 OP
[ecat] error: AL status code 0x0011
```

Reaching SAFE-OP/OP needs full PDO/FMMU/DC configuration — use `start`, not
`states`.

### `upload -p<pos> [-t<type>] <idx> <sub>` (alias `up`)

CoE SDO read (expedited, ≤ 4 bytes). With `-t<type>` the value is decoded; without
it, raw little-endian hex bytes are shown. Output mirrors IgH `outputData`:
`0x<hex, type width> <decimal>`.

```text
upload -p0 -tuint16 0x6041 0
[ecat] 0:0x6041:00 = 0x0239 569

upload -p0 0x6041 0
[ecat] 0:0x6041:00 = 0x39 0x02
```

Errors surface as `[ecat] error: …`, e.g. `SDO abort 0x06020000` for a bad index,
`slave has no CoE`, or `no such slave`.

### `download -p<pos> -t<type> <idx> <sub> <value>` (alias `down`)

CoE SDO write (expedited). `-t<type>` is **required** (it sets the byte width);
the value is encoded little-endian.

```text
download -p0 -tint8 0x6060 0 8
[ecat] 0:0x6060:00 written
```

(`0x6060 = 8` selects CiA-402 CSP mode.)

### `start [-p<pos>] [-r<hz>]`

Run the full per-slave bring-up (INIT → SAFE-OP: clears FMMUs/DC, configures
mailbox + process-data SMs, applies SDO init values, writes PDO assignment +
mapping over CoE, sets the watchdog and FMMUs, brings up DC SYNC0), then start the
PIT cyclic engine, which drives the slave to OP. `-p` defaults to `0`. The optional
`-r<hz>` sets the cyclic rate (50 – 8000 Hz; default = the compile-time rate) — see
[Cyclic rate control](#cyclic-rate-control-telemetry--live-monitoring). Requires a
prior `rescan`.

```text
start -p0 -r1000
[ecat] slave 0 configured; cyclic PDO started at 1000 Hz
```

If already running: `[ecat] cyclic already running; 'stop' first`. Confirm OP with
`status` (`cyclic OP 1000Hz wkc=3/3`).

### `stop`

Stop the PIT timer and the cyclic engine, releasing the bus for other commands.
The drive is first brought cleanly down to PRE-OP, so its output watchdog never
latches an AL error — repeated `start → stop → start` needs no manual `states
INIT` between cycles.

```text
stop
[ecat] cyclic PDO stopped
```

### `pdos`

List the resolved process-data pins (from the compile-time config), with image
offset, bit position, and bit length.

```text
pdos
[ecat] 17 process-data pins:
[ecat] OUT drive0-controlword off=0 bit=0 len=16
[ecat] OUT drive0-target-position off=2 bit=0 len=32
[ecat] OUT drive0-target-velocity off=6 bit=0 len=32
...
[ecat] IN  drive0-statusword off=18 bit=0 len=16
[ecat] IN  drive0-actual-position off=20 bit=0 len=32
...
```

`OUT` = master → drive (RxPDO); `IN ` = drive → master (TxPDO).

### `pd [<pin> [<value>]]`

The process-data accessor. Three forms:

**No argument — dump the image + cyclic status** (up to 64 bytes, 16 per row):

```text
pd
[ecat] cyclic OP wkc=3/3 cycles=128511
[ecat] 0000: 0F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[ecat] 0010: 00 00 37 02 A1 B2 0C 00 ...
```

**One argument — read a named pin** (signed-aware; shown decimal + hex):

```text
pd drive0-statusword
[ecat] drive0-statusword = 569 (0x239)
```

**Two arguments — write an output pin** (the value is staged into the image and
sent next cycle):

```text
pd drive0-controlword 15
[ecat] drive0-controlword <= 15
```

If the engine isn't running: `[ecat] cyclic not running`. Unknown name:
`[ecat] unknown pin '<name>'`.

### `crashlog` / `crashclear`

Show or clear the fault/panic context persisted across the last reboot. See
[Crash diagnostics](#crash-diagnostics) below.

```text
crashlog
[crash] none recorded

crashclear
[crash] cleared
```

---

## SDO data types

`upload`/`download` accept these expedited (≤ 4-byte) native types:

| Type | Bytes | Signed | Code |
| --- | --- | --- | --- |
| `bool` | 1 | no | `0x0001` |
| `int8` | 1 | yes | `0x0002` |
| `int16` | 2 | yes | `0x0003` |
| `int32` | 4 | yes | `0x0004` |
| `uint8` | 1 | no | `0x0005` |
| `uint16` | 2 | no | `0x0006` |
| `uint32` | 4 | no | `0x0007` |

Signed types are decoded sign-extended; the printed hex width matches the type
size.

---

## Error messages

Parser/validation errors echo as `[ecat] error: <msg>` — e.g. `unknown command;
type 'help'`, `requires -p<pos>`, `unknown type; see 'help'`,
`download requires -t<type>`, `invalid index`, `unknown state (INIT|PREOP|SAFEOP|OP)`.

Transfer/bus errors carry the master's `EcError`:

| Text | Cause |
| --- | --- |
| `SDO abort 0x<8 hex>` | The slave aborted the SDO (e.g. `0x06020000` = object not present). |
| `AL status code 0x<4 hex>` | An AL transition failed (read at register `0x0134`). |
| `no such slave` | The position wasn't discovered (run `rescan`). |
| `slave has no CoE` | The slave's mailbox protocols don't include CoE. |
| `mailbox timeout` / `timeout` | No mailbox data / no reply before the deadline. |
| `working counter` | A configuration datagram wasn't acknowledged as expected. |

---

## Crash diagnostics

`crashlog` replays the last persisted fault/panic. A **HardFault** records the
CPU context and **auto-reboots** (recoverable over USB); a **panic** records its
message and **halts** (retrievable after a manual reboot). The record persists
until `crashclear`.

```text
crashlog
[crash] HARDFAULT pc=0x6000A1B2 lr=0x6000A0FF frame_sp=0x20003F80 msp=0x20003F80
[crash] cfsr=0x00008200 hfsr=0x40000000 bfar=0x00000000 mmfar=0x00000000
[crash] r0=0x00000000 r1=0x20001234 r2=0x00000004 r3=0x00000000
[crash] r12=0x00000000 xpsr=0x61000000 send_stage=2
```

A panic instead prints `[crash] PANIC <message>`. If the stack pointer / frame /
`bfar` falls below `0x2000_0400`, a `[crash] hint: … suspect stack overflow` line
is appended. Full field meanings and the LED fault codes are in
[`architecture.md`](architecture.md#10-crash-diagnostics--fault-handling).

---

## Cyclic rate control, telemetry & live monitoring

These extend `start`/`status` for running and observing the cyclic engine across
rates. All three are implemented and hardware-verified (100 Hz – 4 kHz, wkc 3/3,
216 k+ cycles soaked at 4 kHz with zero faults).

### `start [-p<pos>] [-r<hz>]` — rate control

`start` accepts an optional `-r<hz>` to set the cyclic rate at runtime, so you can
launch the engine at any rate without recompiling. Omit `-r` to use the
compile-time `BUS.cycle_ns`.

- **Range:** 50 – 8000 Hz. The PIT cycle timer (`src/board/cycle_timer.rs`) and the
  DC SYNC0 cycle both derive from this period.
- **Default:** the configured rate (currently 100 Hz) when `-r` is omitted.

```text
start -p0 -r4000
[ecat] slave 0 configured; cyclic PDO started at 4000 Hz
```

`stop` now brings the drive cleanly down to PRE-OP first, and the bring-up FSM
acknowledges a latched AL error (writes AL Control `0x0120` with the ack bit), so
repeated `start → stop → start` cycles work with **no manual `states INIT`**
between them (the SM2 output watchdog no longer latches an AL `0x001B` error).

### `stats` — cyclic telemetry (one-shot)

Reports the engine's timing/health beyond `status`. Allowed while running.

```text
stats
[ecat] cyclic OP rate=4000Hz period=250us cycles=216748
[ecat] wkc=3/3
[ecat] jitter min=250us max=250us worst=0us (0 cyc)
[ecat] dc-sync latest=0ns max=0ns
```

- **jitter** — tick-to-tick interval measured from the DWT cycle counter at PIT-ISR
  entry: shortest/longest interval and the worst absolute deviation from the
  expected period (µs and core cycles). `0` on a quiescent board is expected and
  correct — the highest-priority PIT task wakes the CPU from WFI with deterministic
  latency, and the PIT/core clocks are synchronous.
- **dc-sync** — the slave's DC system-time difference (ESC register `0x092C`),
  decoded to signed ns (latest + largest magnitude). `0 ns` with a **single** drive
  is correct: that drive is the DC reference clock. Prints `dc-sync n/a (no reading
  yet)` until the first read. Drift becomes meaningful with 2+ slaves.

### `monitor [on|off]` — live streaming telemetry

Toggles periodic auto-emission of a compact telemetry line (~every 500 ms) while
the engine runs, so a read-only viewer such as
[`view_teensy_serial.py`](serial-monitoring.md) can watch the PDO task live. Bare
`monitor` (or `mon`) toggles. Driven by the priority-1 worker — it never touches
the priority-3 cyclic tick or holds the master lock longer than `stats`.

```text
monitor on
[ecat] monitor on (auto-stats ~500ms)
[mon] 4000Hz cyc=216748 wkc=3/3 jit=0us dc=0ns
[mon] 4000Hz cyc=218748 wkc=3/3 jit=0us dc=0ns
```

Emission is independent of who is connected (it keeps streaming after you
disconnect) and prints a single `[mon] cyclic stopped` line when the engine halts.
Typical use: send `start -p0 -r<hz>` then `monitor on`, disconnect, then open
`view_teensy_serial.py` to watch the live stream.
