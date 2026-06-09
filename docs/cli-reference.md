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
- **All responses are prefixed** `[boot]`, `[ecat]`, `[scan]`, `[crash]`, `[host]`,
  or `[mon]`.
- **On USB attach** the firmware prints a one-time banner. Otherwise it only speaks
  when spoken to — the bus is **not** auto-scanned at boot, so run `rescan` first.

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
available: `pd`, `pdos`, `slaves`, `status`, `stats`, `monitor`, `host` (they read
the image, cached topology, or telemetry — not the live bus). Use `stop` first to
run a bus command.

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
  start [-p<pos>] [-r<hz>]                 configure + start cyclic PDO (50..8000 Hz)
  stop                                     stop cyclic PDO
  stats                                    cyclic rate, jitter, DC sync error
  monitor [on|off]                         stream stats ~every 500ms (bare = toggle)
  pdos                                     list process-data pins and offsets
  pd [<pin> [<value>]]                     dump image / read pin / write pin
  host                                     Pi/LinuxCNC SPI bridge diagnostics
  crashlog                                 show the saved fault/panic context
  crashclear                               clear the saved fault/panic context
  types: bool int8 int16 int32 uint8 uint16 uint32
```

### `slaves`

List the slaves discovered by the last scan (cached; does not touch the bus).

```text
slaves
[ecat] 2 slave(s)
[ecat] slave 0 station=1 vid=0x00000994 pid=0x00001B00 al=0x01 coe=yes
[ecat] slave 1 station=2 vid=0x00000994 pid=0x00001B00 al=0x01 coe=yes
```

Fields: ring position, station address (`= position + 1`), vendor ID, product
code, AL-status byte (`0x01`=INIT, `0x02`=PRE-OP, `0x04`=SAFE-OP, `0x08`=OP), and
CoE support. Empty until you `rescan`. The committed bus is two identical drives,
so both rows show the same vendor/product.

### `status` (alias `info`)

Firmware tag, PHY link state, slave count, and (if running) the cyclic snapshot.

```text
status
[ecat] fw 0.1.0 (v0.1.0-g1a2b3c4)
[ecat] link=up slaves=2
[ecat] cyclic OP 100Hz wkc=6/6 cycles=128407 ('stats' for detail)
```

The third line appears only while the cyclic engine runs. It carries the cyclic
rate, `wkc=6/6` (observed/expected working counter — `+3` per drive, so two drives
expect `6`), and the cycle count; the phase is `priming`, `requesting-op`, `OP`, or
`faulted`. Use `stats` for latency/jitter + DC.

### `rescan`

Re-run the bus scan, **streaming each sub-step** over serial as it completes (the
primary no-SWD diagnostic). Must be run before `start`.

```text
rescan
[scan] counting slaves
[scan] count=2
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
[scan] s2: addr set
[scan] s2: al=0x01
[scan] s2: base type=0x05 fmmu=3 sm=4
[scan] s2: vendor=0x00000994
[scan] s2: product=0x00001B00
[scan] s2: rev=0x00000001
[scan] s2: rxmbox off=0x1000 sz=128
[scan] s2: txmbox off=0x1080 sz=128
[scan] s2: proto=0x0004 coe=1
[ecat] rescan complete: 2 slave(s); type 'slaves'
```

`s1`/`s2` are the station addresses (ring position + 1), streamed per slave in ring
order. On failure a line like `[ecat] error: working counter` (or another
`EcError`) ends the stream.

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

Bring up the **whole configured bus** and start cyclic process data. It runs the
full per-slave bring-up (INIT → SAFE-OP: clears FMMUs/DC, configures mailbox +
process-data SMs, applies SDO init values, writes PDO assignment + mapping over
CoE, sets the watchdog and FMMUs, brings up DC SYNC0) on **every** configured slave
in ring order, then starts the PIT cyclic engine, which drives them all to OP. One
LRW spans all slaves, so `-p` is **accepted but ignored** — it no longer selects a
subset. The optional `-r<hz>` sets the cyclic rate (50 – 8000 Hz; default = the
compile-time rate) — see
[Cyclic rate control](#cyclic-rate-control-telemetry--live-monitoring). Requires a
prior `rescan`.

```text
start -r1000
[ecat] 2 slave(s) configured; cyclic PDO started at 1000 Hz
```

If already running: `[ecat] cyclic already running; 'stop' first`. Confirm OP with
`status` (`cyclic OP 1000Hz wkc=6/6`).

### `stop`

Stop the PIT timer and the cyclic engine, releasing the bus for other commands.
**Every** drive is first walked cleanly down to PRE-OP (in ring order), so no
slave's output watchdog latches an AL error — repeated `start → stop → start` needs
no manual `states INIT` between cycles.

```text
stop
[ecat] cyclic PDO stopped
```

### `pdos`

List the resolved process-data pins (from the compile-time config), with image
offset, bit position, and bit length.

```text
pdos
[ecat] 34 process-data pins:
[ecat] OUT drive0-controlword off=0 bit=0 len=16
[ecat] OUT drive0-target-position off=2 bit=0 len=32
[ecat] OUT drive0-target-velocity off=6 bit=0 len=32
...
[ecat] OUT drive1-controlword off=16 bit=0 len=16
...
[ecat] IN  drive0-statusword off=34 bit=0 len=16
[ecat] IN  drive0-actual-position off=36 bit=0 len=32
...
[ecat] IN  drive1-statusword off=73 bit=0 len=16
...
```

`OUT` = master → drive (RxPDO); `IN ` = drive → master (TxPDO). All outputs come
first (drive0 `0..16`, drive1 `16..32`), then all inputs (drive0 `32..71`, drive1
`71..110`), so the two drives' pins share the same `driveN-` shape at stacked
offsets.

### `pd [<pin> [<value>]]`

The process-data accessor. Three forms:

**No argument — dump the whole process image + cyclic status** (16 bytes per row;
110 B for the two-drive bus). Bytes `0..16` are drive0 outputs, `16..32` drive1
outputs, then the inputs:

```text
pd
[ecat] cyclic OP wkc=6/6 cycles=128511
[ecat] 0000: 0F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[ecat] 0010: 0F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[ecat] 0020: 00 00 37 02 A1 B2 0C 00 ...
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
[crash] r12=0x00000000 xpsr=0x61000000
```

A panic instead prints `[crash] PANIC <message>`. If the stack pointer / frame /
`bfar` falls below `0x2000_0400`, a `[crash] hint: … suspect stack overflow` line
is appended. Full field meanings and the LED fault codes are in
[`architecture.md`](architecture.md#10-crash-diagnostics--fault-handling).

---

## Cyclic rate control, telemetry & live monitoring

These extend `start`/`status` for running and observing the cyclic engine across
rates. All three are implemented and hardware-verified (100 Hz – 4 kHz): two drives
reach OP at `wkc = 6/6` at both 100 Hz and 4 kHz, and an earlier single-drive run
soaked 216 k+ cycles at 4 kHz (`wkc = 3/3`) with zero faults.

### `start [-p<pos>] [-r<hz>]` — rate control

`start` accepts an optional `-r<hz>` to set the cyclic rate at runtime, so you can
launch the engine at any rate without recompiling. Omit `-r` to use the
compile-time `BUS.cycle_ns`.

- **Range:** 50 – 8000 Hz. The PIT cycle timer (`src/board/cycle_timer.rs`) and the
  DC SYNC0 cycle both derive from this period.
- **Default:** the configured rate (currently 100 Hz) when `-r` is omitted.

```text
start -r4000
[ecat] 2 slave(s) configured; cyclic PDO started at 4000 Hz
```

`stop` now brings **every** drive cleanly down to PRE-OP first, and the bring-up
FSM acknowledges a latched AL error (writes AL Control `0x0120` with the ack bit),
so repeated `start → stop → start` cycles work with **no manual `states INIT`**
between them (the SM2 output watchdog no longer latches an AL `0x001B` error).

### `stats` — cyclic telemetry (one-shot)

Reports the engine's timing/health beyond `status`. Allowed while running.

```text
stats
[ecat] cyclic OP rate=4000Hz period=250us cycles=216748
[ecat] wkc=6/6
[ecat] latency min=500ns max=500ns jitter=0ns (worst 300 cyc)
[ecat] dc-sync latest=-118ns max=140ns
```

- **latency** — the **absolute interrupt latency**: the delay from the PIT hardware
  fire to the cyclic ISR actually running, read at ISR entry from the PIT
  down-counter (`LDVAL − CVAL`). The line reports `min` / `max` (ns), the
  **jitter** = `max − min` (the headline number), and the worst-case latency in
  core cycles. On a lightly-loaded board it sits ~**500 ns flat** (≈300 core cycles
  at 600 MHz): the PIT is the highest-priority interrupt, so the only thing that can
  defer entry is a lower-priority section briefly holding the master lock (whose
  priority ceiling masks the cyclic IRQ) — e.g. the worker's MDIO link read during
  `status` — which rarely coincides with a fire. Jitter therefore reads ≈0 when idle
  and **rises under sustained load** (e.g. the SPI host bridge). This replaces the
  old tick-to-tick interval, which was always exactly the period (the PIT/core
  clocks are synchronous and the WFI wake is deterministic, hiding all jitter).
- **dc-sync** — a follower's DC system-time difference (ESC register `0x092C`),
  decoded to signed ns (latest + largest magnitude). With **2+ slaves** the master
  distributes the reference DC time every cycle (an ARMW of `0x0910`
  auto-increment-addressed at the reference slave), so this is **real follower
  drift**: it converges under the correction to ≈0.9 ms residual at 100 Hz,
  tightening to **±~140 ns at 4 kHz** (a faster correction rate = tighter sync). The
  remaining **static** delay/offset compensation (`0x0900` → `0x0920`) is deferred,
  which is why the low-rate residual is still large. With a single slave it reads
  `0 ns` (that drive is its own reference). Prints `dc-sync n/a (no reading yet)`
  until the first read.

### `monitor [on|off]` — live streaming telemetry

Toggles periodic auto-emission of a compact telemetry line (~every 500 ms) while
the engine runs, so a read-only viewer such as
[`view_teensy_serial.py`](serial-monitoring.md) can watch the PDO task live. Bare
`monitor` (or `mon`) toggles. Driven by the priority-1 worker — it never touches
the priority-3 cyclic tick or holds the master lock longer than `stats`.

```text
monitor on
[ecat] monitor on (auto-stats ~500ms)
[mon] 4000Hz cyc=216748 wkc=6/6 jit=0ns dc=-118ns
[mon] 4000Hz cyc=218748 wkc=6/6 jit=0ns dc=-122ns
```

Emission is independent of who is connected (it keeps streaming after you
disconnect) and prints a single `[mon] cyclic stopped` line when the engine halts.
Typical use: send `start -p0 -r<hz>` then `monitor on`, disconnect, then open
`view_teensy_serial.py` to watch the live stream.
