"""Temporary cyclic-PDO bring-up driver over the Teensy CDC serial console.

Walks the master through cyclic process-data exchange and watches it reach OP:
  scan -> start (configure to SAFE-OP + start the PIT cyclic engine) -> poll the
  process image / working counter while the SAFE-OP -> OP transition completes ->
  read inputs (statusword, actual position) -> write an output (controlword) ->
  stop.

This only talks to the serial port; it does NOT run firmware. Flash the board
(`make flash`) and connect the EtherCAT drive first, then run:

    python3 temporary_pdo_test.py

The drive must be a YAKO/Bohign unit matching ethercat-conf.bohign.xml (the
stock RxPDO1/TxPDO1 mapping). See the README / docs/pdo-planning-input.md.
"""

from __future__ import annotations

import sys
import time

import serial
from serial.tools import list_ports

VID = 0x16C0
PID = 0x0483
BAUD = 115200

INITIAL_OBSERVE_S = 1.5
POLL_SLEEP_S = 0.0003
IDLE_GAP_S = 0.10

# `start` runs the whole INIT->SAFE-OP bring-up (FMMU/SM/PDO/DC), so give it room.
COMMAND_CAPS = {
    "start": (10.0, 6.0),  # (first-byte timeout, response drain cap)
}
DEFAULT_CAP = (6.0, 2.5)

# After `start`, poll the image/status this many times to watch priming -> OP.
STATUS_POLLS = 8
STATUS_POLL_GAP_S = 0.4


def find_port() -> str | None:
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def print_lines(data: bytes) -> None:
    for line in data.decode("utf-8", errors="replace").splitlines():
        line = line.strip()
        if line:
            print(f"    {line}")


def dump(connection: serial.Serial, seconds: float) -> None:
    end = time.time() + seconds
    buf = bytearray()
    while time.time() < end:
        chunk = connection.read(512)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(POLL_SLEEP_S)
    print_lines(bytes(buf))


def send(connection: serial.Serial, cmd: str) -> bytes:
    first_to, drain_cap = COMMAND_CAPS.get(cmd.split()[0], DEFAULT_CAP)
    connection.reset_input_buffer()
    connection.write((cmd + "\n").encode())
    connection.flush()
    t_send = time.perf_counter()

    deadline = t_send + first_to
    buf = bytearray()
    t_first = None
    while time.perf_counter() < deadline:
        chunk = connection.read(512)
        if chunk:
            t_first = time.perf_counter()
            buf.extend(chunk)
            break
        time.sleep(POLL_SLEEP_S)
    if t_first is None:
        print(f"--- > {cmd}  [NO RESPONSE within {first_to:.1f}s] ---")
        return b""

    cap = t_first + drain_cap
    last_rx = t_first
    while time.perf_counter() < cap:
        chunk = connection.read(512)
        if chunk:
            buf.extend(chunk)
            last_rx = time.perf_counter()
        elif time.perf_counter() - last_rx >= IDLE_GAP_S:
            break
        else:
            time.sleep(POLL_SLEEP_S)
    print(f"--- > {cmd}  [{(t_first - t_send) * 1e3:.1f} ms] ---")
    print_lines(bytes(buf))
    return bytes(buf)


def main() -> int:
    port = find_port()
    if not port:
        print("ERROR: no Teensy CDC port (16C0:0483) found")
        return 1

    print(f"opening {port} @ {BAUD}")
    with serial.Serial(port, BAUD, timeout=0) as connection:
        time.sleep(0.3)
        connection.reset_input_buffer()
        print("--- initial capture (banner + scan summary) ---")
        dump(connection, INITIAL_OBSERVE_S)

        send(connection, "slaves")
        send(connection, "pdos")
        send(connection, "start")

        print("\n=== watching SAFE-OP -> OP (process image + working counter) ===")
        for _ in range(STATUS_POLLS):
            send(connection, "pd")
            time.sleep(STATUS_POLL_GAP_S)

        print("\n=== read inputs / write an output ===")
        send(connection, "pd drive0-statusword")
        send(connection, "pd drive0-actual-position")
        send(connection, "pd drive0-controlword 6")  # CiA 402 'shutdown'
        send(connection, "pd drive0-statusword")

        send(connection, "stop")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
