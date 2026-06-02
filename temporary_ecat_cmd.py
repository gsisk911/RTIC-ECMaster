"""Temporary EtherCAT serial command driver + turnaround meter.

Opens the Teensy CDC port, prints the one-time boot banner / scan summary it
captures on attach, then sends each command and measures the command-turnaround
time: the interval from finishing the write (command + newline) to the first
response byte coming back. The firmware no longer echoes typed characters and
stays silent until a command's response is ready (IgH `ethercat`-tool style), so
the first received byte is the start of the real response -- the measurement is
clean with no echo/stream to subtract.

Usage:
    # default verification suite (slaves, SDOs, write+readback, bad-index abort)
    python3 temporary_ecat_cmd.py

    # explicit commands
    python3 temporary_ecat_cmd.py "slaves" "upload -p0 -tuint32 0x1000 0"

    # repeat each command N times (to separate first-SDO vs subsequent-SDO cost)
    python3 temporary_ecat_cmd.py --repeat 3 "upload -p0 -tuint16 0x6041 0"
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
FIRST_BYTE_TIMEOUT_S = 6.0
IDLE_GAP_S = 0.10
RESPONSE_CAP_S = 2.5
# Non-blocking reads + a short poll sleep give sub-millisecond timing resolution.
# (A blocking pyserial timeout quantizes read() to the timeout boundary and would
# inflate the measured turnaround to ~the timeout value, hiding the real latency.)
POLL_SLEEP_S = 0.0003

# Default suite mirrors the hardware verification checklist.
DEFAULT_SUITE = [
    "slaves",
    "upload -p0 -tuint32 0x1000 0",   # first SDO: includes INIT->PRE-OP bring-up
    "upload -p0 -tuint16 0x6041 0",   # subsequent SDO (statusword)
    "upload -p0 -tuint16 0x6041 0",   # subsequent SDO again
    "download -p0 -tint8 0x6060 0 8", # write modes-of-operation = 8 (CSV)
    "upload -p0 -tint8 0x6060 0",     # read it back
    "upload -p0 -tuint16 0x9999 0",   # bad index -> SDO abort
]


def find_port() -> str | None:
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def print_lines(data: bytes) -> None:
    text = data.decode("utf-8", errors="replace")
    for line in text.splitlines():
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


def send_and_measure(connection: serial.Serial, cmd: str) -> tuple[float | None, bytes]:
    """Send one command and time send->first-response-byte.

    Returns (turnaround_seconds_or_None, full_response_bytes). Uses non-blocking
    reads + a short poll sleep so the first response byte is timestamped the
    instant it arrives. The full response is then gathered until the stream goes
    idle for IDLE_GAP_S (or RESPONSE_CAP_S overall after the first byte).
    """
    connection.reset_input_buffer()
    connection.write((cmd + "\n").encode())
    connection.flush()
    t_send = time.perf_counter()

    # Busy-poll for the first response byte.
    deadline = t_send + FIRST_BYTE_TIMEOUT_S
    buf = bytearray()
    t_first: float | None = None
    while time.perf_counter() < deadline:
        chunk = connection.read(512)
        if chunk:
            t_first = time.perf_counter()
            buf.extend(chunk)
            break
        time.sleep(POLL_SLEEP_S)
    if t_first is None:
        return None, bytes(buf)

    # Drain the rest of the (multi-line) response until it goes idle.
    cap = t_first + RESPONSE_CAP_S
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
    return t_first - t_send, bytes(buf)


def main() -> int:
    args = sys.argv[1:]
    repeat = 1
    if args and args[0] == "--repeat":
        if len(args) < 2:
            print("ERROR: --repeat needs a count")
            return 1
        repeat = max(1, int(args[1]))
        args = args[2:]
    commands = args if args else DEFAULT_SUITE

    port = find_port()
    if not port:
        print("ERROR: no Teensy CDC port (16C0:0483) found")
        return 1

    print(f"opening {port} @ {BAUD}")
    results: list[tuple[str, float | None]] = []
    with serial.Serial(port, BAUD, timeout=0) as connection:
        time.sleep(0.3)
        connection.reset_input_buffer()
        print("--- initial capture (banner + scan summary) ---")
        dump(connection, INITIAL_OBSERVE_S)

        for cmd in commands:
            for _ in range(repeat):
                turnaround, data = send_and_measure(connection, cmd)
                if turnaround is None:
                    print(f"--- > {cmd}  [NO RESPONSE within {FIRST_BYTE_TIMEOUT_S:.1f}s] ---")
                else:
                    print(f"--- > {cmd}  [turnaround {turnaround * 1e3:.1f} ms] ---")
                print_lines(data)
                results.append((cmd, turnaround))

    print("\n=== turnaround summary (send -> first response byte) ===")
    for i, (cmd, turnaround) in enumerate(results):
        ms = "timeout" if turnaround is None else f"{turnaround * 1e3:7.1f} ms"
        print(f"  [{i}] {ms}  {cmd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
