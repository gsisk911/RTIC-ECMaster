"""
Simple serial terminal viewer for a connected Teensy.

Finds a USB serial device by VID/PID or auto-detects a likely Teensy,
opens the matching port, and prints incoming text lines to stdout until
interrupted.

Usage:
    pip install -r scripts/requirements.txt
    python scripts/view_teensy_serial.py
    python scripts/view_teensy_serial.py --vid 0x16C0 --pid 0x0483

Optional filters:
    python scripts/view_teensy_serial.py --list
    python scripts/view_teensy_serial.py --vid 0x16C0 --pid 0x0483 --serial-number ABC123
"""

from __future__ import annotations

import argparse
import struct
import time
from collections.abc import Iterable
from datetime import datetime

import serial
from serial.tools import list_ports


DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT_S = 0.25
CORE_CLOCK_HZ = 600_000_000
VELOCITY_FRAME_SYNC = 0xA55A
VELOCITY_FRAME_STRUCT = struct.Struct("<Hi h h i I".replace(" ", ""))
KNOWN_TEENSY_USB_IDS = {
    (0x16C0, None),      # Common PJRC / Teensy USB VID
    (0x5824, 0x27DD),    # Historical imxrt-log CDC VID/PID used by older builds
}
KNOWN_TEENSY_KEYWORDS = (
    "teensy",
    "imxrt-log",
    "imxrt",
    "encsim-telemetry",
)


def parse_usb_id(value: str) -> int:
    """Parse a decimal or hex VID/PID value from the CLI."""
    text = value.strip().lower()
    if not text:
        raise argparse.ArgumentTypeError("USB IDs must not be empty.")

    try:
        if text.startswith("0x"):
            return int(text, 16)
        return int(text, 10)
    except ValueError:
        try:
            return int(text, 16)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid USB ID {value!r}. Use decimal or hex like 0x16C0."
            ) from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="View Teensy serial output by USB VID/PID or auto-detect a connected Teensy.",
    )
    parser.add_argument("--vid", type=parse_usb_id, help="USB vendor ID, for example 0x16C0")
    parser.add_argument("--pid", type=parse_usb_id, help="USB product ID, for example 0x0483")
    parser.add_argument(
        "--serial-number",
        help="Optional USB serial number filter when multiple boards share the same VID/PID.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Serial baudrate passed to pyserial (default: {DEFAULT_BAUDRATE}).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"Read timeout in seconds (default: {DEFAULT_TIMEOUT_S}).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected serial ports and exit.",
    )
    parser.add_argument(
        "--binary-velocity",
        action="store_true",
        help="Decode the temporary 5kHz velocity telemetry binary stream.",
    )
    parser.add_argument(
        "--wait",
        action="store_true",
        help="Wait for a matching Teensy serial device to appear instead of exiting immediately.",
    )
    parser.add_argument(
        "--wait-interval",
        type=float,
        default=1.0,
        help="Seconds between port rescan attempts when --wait is enabled (default: 1.0).",
    )
    return parser


def format_usb_id(value: int | None) -> str:
    if value is None:
        return "----"
    return f"0x{value:04X}"


def describe_port(port: list_ports.ListPortInfo) -> str:
    details = [
        port.device,
        port.description or "unknown",
        f"VID={format_usb_id(port.vid)}",
        f"PID={format_usb_id(port.pid)}",
    ]
    if port.serial_number:
        details.append(f"SER={port.serial_number}")
    return " | ".join(details)


def is_teensy_candidate(port: list_ports.ListPortInfo) -> bool:
    """Return True when port metadata strongly suggests a Teensy."""
    for known_vid, known_pid in KNOWN_TEENSY_USB_IDS:
        if port.vid != known_vid:
            continue
        if known_pid is None or port.pid == known_pid:
            return True

    fields = (
        port.manufacturer,
        port.product,
        port.description,
        port.hwid,
    )
    haystack = " ".join(field.lower() for field in fields if field)
    return any(keyword in haystack for keyword in KNOWN_TEENSY_KEYWORDS)


def iter_ports() -> list[list_ports.ListPortInfo]:
    return sorted(list_ports.comports(), key=lambda port: port.device)


def print_ports(ports: Iterable[list_ports.ListPortInfo]) -> None:
    ports = list(ports)
    if not ports:
        print("No serial ports detected.")
        return

    print("Detected serial ports:")
    for port in ports:
        print(f"  - {describe_port(port)}")


def find_matching_ports(
    ports: Iterable[list_ports.ListPortInfo],
    vid: int,
    pid: int,
    serial_number: str | None,
) -> list[list_ports.ListPortInfo]:
    matches = []
    for port in ports:
        if port.vid != vid or port.pid != pid:
            continue
        if serial_number and port.serial_number != serial_number:
            continue
        matches.append(port)
    return matches


def auto_detect_port(
    ports: Iterable[list_ports.ListPortInfo],
    serial_number: str | None,
) -> list_ports.ListPortInfo | None:
    candidates = []
    for port in ports:
        if not is_teensy_candidate(port):
            continue
        if serial_number and port.serial_number != serial_number:
            continue
        candidates.append(port)

    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        print("ERROR: Could not auto-detect a Teensy serial device.")
    else:
        print("ERROR: Multiple Teensy-like serial devices were detected.")
        print("Use --serial-number to choose one board or pass --vid and --pid explicitly.")
        for port in candidates:
            print(f"  - {describe_port(port)}")
        return None

    print()
    print_ports(ports)
    return None


def try_select_port(args: argparse.Namespace) -> list_ports.ListPortInfo | None:
    ports = iter_ports()
    if args.list:
        print_ports(ports)
        raise SystemExit(0)

    if args.vid is None and args.pid is None:
        port = auto_detect_port(ports, args.serial_number)
        if port is None:
            return None
        return port

    if args.vid is None or args.pid is None:
        raise SystemExit("ERROR: Pass both --vid and --pid, or omit both to auto-detect.")

    matches = find_matching_ports(ports, args.vid, args.pid, args.serial_number)
    if not matches:
        print(
            "ERROR: No serial device matched "
            f"VID={format_usb_id(args.vid)} PID={format_usb_id(args.pid)}"
            + (
                f" SER={args.serial_number}"
                if args.serial_number
                else ""
            )
        )
        print()
        print_ports(ports)
        return None

    if len(matches) > 1:
        print("ERROR: Multiple matching serial devices found. Narrow the selection with --serial-number.")
        for port in matches:
            print(f"  - {describe_port(port)}")
        raise SystemExit(1)

    return matches[0]


def select_port(args: argparse.Namespace) -> list_ports.ListPortInfo:
    port = try_select_port(args)
    if port is not None:
        return port

    if not args.wait:
        raise SystemExit(1)

    wait_interval = max(args.wait_interval, 0.1)
    print(f"Waiting for matching Teensy serial device (poll every {wait_interval:.1f}s). Press Ctrl-C to stop.")
    while True:
        time.sleep(wait_interval)
        port = try_select_port(args)
        if port is not None:
            return port


def stream_binary_velocity(port: list_ports.ListPortInfo, baudrate: int, timeout: float) -> None:
    print(f"Opening {port.device} ({port.description})")
    print(
        "Listening for binary velocity telemetry from "
        f"VID={format_usb_id(port.vid)} PID={format_usb_id(port.pid)}"
        + (f" SER={port.serial_number}" if port.serial_number else "")
    )
    print("Columns: sample_index position bucket_delta moving_sum reported_rate cycle_delta dt_us")
    print("Press Ctrl-C to stop.\n")

    frame_bytes = VELOCITY_FRAME_STRUCT.size
    sync_bytes = VELOCITY_FRAME_SYNC.to_bytes(2, "little")
    buffer = bytearray()
    sample_index = 0
    last_cycle_count = None

    with serial.Serial(port.device, baudrate=baudrate, timeout=timeout) as connection:
        while True:
            chunk = connection.read(4096)
            if not chunk:
                continue

            buffer.extend(chunk)
            while len(buffer) >= frame_bytes:
                sync_index = buffer.find(sync_bytes)
                if sync_index < 0:
                    del buffer[:-1]
                    break
                if sync_index > 0:
                    del buffer[:sync_index]
                    if len(buffer) < frame_bytes:
                        break

                sync, position, bucket_delta, moving_sum, reported_rate, cycle_count = VELOCITY_FRAME_STRUCT.unpack(
                    buffer[:frame_bytes]
                )
                if sync != VELOCITY_FRAME_SYNC:
                    del buffer[0]
                    continue

                cycle_delta = 0 if last_cycle_count is None else (cycle_count - last_cycle_count) & 0xFFFFFFFF
                dt_us = (cycle_delta * 1_000_000.0) / CORE_CLOCK_HZ if cycle_delta else 0.0
                print(
                    f"{sample_index},{position},{bucket_delta},{moving_sum},{reported_rate},{cycle_delta},{dt_us:.3f}",
                    flush=False,
                )
                sample_index += 1
                last_cycle_count = cycle_count
                del buffer[:frame_bytes]


def stream_serial_output(port: list_ports.ListPortInfo, baudrate: int, timeout: float) -> None:
    print(f"Opening {port.device} ({port.description})")
    print(
        "Listening for serial output from "
        f"VID={format_usb_id(port.vid)} PID={format_usb_id(port.pid)}"
        + (f" SER={port.serial_number}" if port.serial_number else "")
    )
    print("Press Ctrl-C to stop.\n")

    with serial.Serial(port.device, baudrate=baudrate, timeout=timeout) as connection:
        while True:
            raw_line = connection.readline()
            if not raw_line:
                continue

            text = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"{timestamp} | {text}")


def main() -> None:
    args = build_parser().parse_args()
    if args.serial_number is not None:
        args.serial_number = args.serial_number.strip() or None
    port = select_port(args)

    try:
        if args.binary_velocity:
            stream_binary_velocity(port, args.baudrate, args.timeout)
        else:
            stream_serial_output(port, args.baudrate, args.timeout)
    except serial.SerialException as exc:
        raise SystemExit(f"ERROR: Failed to open/read serial port: {exc}") from exc
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
