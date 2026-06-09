# Serial monitoring guide

How to watch the firmware's serial output live — the `[boot]` banner, the
streamed `[scan]` traces during configuration, and command responses such as
`status` and `pd` during the cyclic PDO task.

The repo ships a read-only logger, [`scripts/view_teensy_serial.py`](../scripts/view_teensy_serial.py),
that auto-detects the Teensy CDC port and prints every received line with a
timestamp. For *issuing* commands you need a terminal that also sends keystrokes
(the logger only reads) — see [Two ways to use the port](#two-ways-to-use-the-port).

---

## Install

```sh
pip install -r scripts/requirements.txt   # pyserial (and pymodbus)
```

---

## `view_teensy_serial.py` — the read-only logger

Auto-detect a connected Teensy and stream its output:

```sh
python scripts/view_teensy_serial.py
```

Pin the device explicitly by USB VID/PID (the firmware enumerates as
`16C0:0483`):

```sh
python scripts/view_teensy_serial.py --vid 0x16C0 --pid 0x0483
```

List the serial ports it can see and exit:

```sh
python scripts/view_teensy_serial.py --list
```

Wait for the board to appear (handy right after `make flash`, while the device
re-enumerates):

```sh
python scripts/view_teensy_serial.py --wait
```

### What you'll see

Each received line is printed as `timestamp | text`:

```text
Opening /dev/cu.usbmodem123456 (USB Serial)
Listening for serial output from VID=0x16C0 PID=0x0483
Press Ctrl-C to stop.

2026-06-04 08:19:01.412 | [boot] teensy-rust-modbus-base 0.1.0 (v0.1.0-g1a2b3c4)
2026-06-04 08:19:01.460 | [boot] EtherCAT master over RMII ENET; type 'help' for commands
2026-06-04 08:19:14.002 | [scan] counting slaves
2026-06-04 08:19:14.051 | [scan] count=2
2026-06-04 08:19:14.190 | [scan] s1: vendor=0x00000994
2026-06-04 08:19:14.232 | [scan] s2: vendor=0x00000994
2026-06-04 08:19:14.260 | [ecat] rescan complete: 2 slave(s); type 'slaves'
2026-06-04 08:19:31.118 | [ecat] cyclic OP 100Hz wkc=6/6 cycles=128407 ('stats' for detail)
```

### All flags

| Flag | Default | Meaning |
| --- | --- | --- |
| `--vid <id>` | (auto) | USB vendor ID (decimal or `0x…`). Pass with `--pid`. |
| `--pid <id>` | (auto) | USB product ID. Pass with `--vid`. |
| `--serial-number <s>` | — | Disambiguate when several boards share a VID/PID. |
| `--baudrate <n>` | `115200` | Serial baud (the firmware uses 115200). |
| `--timeout <s>` | `0.25` | Read timeout, seconds. |
| `--list` | — | Print detected ports and exit. |
| `--wait` | off | Wait for a matching board to appear instead of erroring out. |
| `--wait-interval <s>` | `1.0` | Rescan interval when `--wait` is set. |
| `--binary-velocity` | off | Decode a legacy binary telemetry stream (not the EtherCAT text CLI; see note). |

**Auto-detection** matches USB VID `0x16C0` (any PID) or port metadata containing
`teensy` / `imxrt` / `imxrt-log`. If exactly one candidate is found it is opened;
if several are found it lists them and asks you to narrow with `--serial-number`
or explicit `--vid`/`--pid`.

> **`--binary-velocity`** decodes a temporary, fixed-format binary telemetry frame
> (a leftover 5 kHz velocity stream), not the EtherCAT text CLI. Leave it off for
> normal EtherCAT monitoring; the default text mode is what shows the `[boot]` /
> `[scan]` / `[ecat]` lines.

---

## Two ways to use the port

A serial port has a **single owner** — one program at a time. The logger above is
**read-only** (it never sends), so it cannot type `rescan` / `start` / `pd`.
Choose based on what you're doing:

### A. Drive *and* watch — an interactive terminal (recommended)

To issue commands and see responses in one window, use a terminal that sends
keystrokes. `miniterm` ships with pyserial (already installed):

```sh
python -m serial.tools.miniterm --echo 115200
# it lists ports; pick the 16C0:0483 device. Quit with Ctrl-]
```

`--echo` shows what you type (the firmware itself does not echo). Other options:
`screen /dev/cu.usbmodemXXXXXX 115200`, `picocom -b 115200 /dev/cu.usbmodemXXXXXX`,
or any serial monitor (Arduino IDE, etc.). This is the setup for the
[README quick start](../README.md#quick-start--using-it-as-a-driver): type
`rescan`, `start`, `pd …` and read the replies inline.

### B. Passive capture — `view_teensy_serial.py`

Use the logger when you want a **timestamped, read-only record** — e.g. capturing
the `[boot]` banner and the streamed `[scan]` traces during a bring-up, or piping
to a file:

```sh
python scripts/view_teensy_serial.py --wait | tee bringup.log
```

Because it can't send, you'd trigger the commands from an interactive terminal at
another time (you cannot hold the port in both tools at once).

---

## Watching a bring-up and the cyclic task

The firmware is request/response, so the interesting output is produced **when you
type a command** (in setup A). What to watch for:

**During configuration**

- `rescan` streams `[scan] …` lines, one per scan sub-step (count, address-clear,
  per-slave AL/DL/SII identity), ending with `[ecat] rescan complete: N slave(s)`.
  If the firmware faults mid-scan, every step up to the fault is already on screen
  — this is the main no-SWD diagnostic.
- `start` runs the full INIT → SAFE-OP bring-up on **every** configured slave and
  starts the cyclic engine; it replies `[ecat] 2 slave(s) configured; cyclic PDO
  started at 100 Hz` (or the `-r<hz>` rate). One LRW spans the whole bus, so `-p` is
  accepted but ignored. Add `-r<hz>` to launch at another rate (50 – 8000 Hz).

**During the cyclic PDO task**

- `status` shows `link=… slaves=…` and, while cycling, `cyclic OP 100Hz wkc=6/6
  cycles=…` — watch that the phase reaches `OP` and the working counter is full
  (`6/6` for the two-drive bus). `stats` / `monitor` add interrupt latency / jitter
  and DC sync error.
- `pd` (no args) dumps the live process image + cyclic status; `pd <pin>` reads a
  named input (e.g. `pd drive0-statusword`); `pd <pin> <value>` writes an output
  (e.g. `pd drive0-controlword 15`).

See [`cli-reference.md`](cli-reference.md) for the full command set and output
formats.

---

## Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| "Could not auto-detect a Teensy" | Board not enumerated yet (run with `--wait`), or right after flashing it's still re-enumerating. Check `--list`. |
| "Multiple Teensy-like serial devices" | Pass `--serial-number` or explicit `--vid`/`--pid`. |
| "Failed to open/read serial port" | Another program already owns the port (e.g. a `miniterm`/`screen` session, or another logger). Close it — single owner only. |
| Nothing prints after the banner | Expected: the console is silent until a command produces output. Type a command in an interactive terminal (setup A). |
| Typed characters don't appear | The firmware doesn't echo. Enable local echo (`miniterm --echo`). |
