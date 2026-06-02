#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, Optional


try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except Exception as e:  # pragma: no cover
    serial = None  # type: ignore
    list_ports = None  # type: ignore
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None


DEFAULT_VID = 0x16C0
DEFAULT_PID = 0x0483
TEENSY_SOFT_REBOOT_BAUD_BOOTLOADER = 134
TEENSY_SOFT_REBOOT_BAUD_NORMAL = 135
DEFAULT_SETTLE_SECONDS = 0.25


class SoftRebootError(Exception):
    pass


def parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from exc


def physical_port_key(port: object) -> str:
    serial_number = getattr(port, "serial_number", None)
    if serial_number:
        return f"serial:{serial_number}"

    device = str(getattr(port, "device", ""))
    for prefix in ("/dev/cu.", "/dev/tty."):
        if device.startswith(prefix):
            return f"device:{device.removeprefix(prefix)}"
    return f"device:{device}"


def prefer_callout_port(ports: list[object]) -> object:
    return sorted(
        ports,
        key=lambda port: (
            not str(getattr(port, "device", "")).startswith("/dev/cu."),
            str(getattr(port, "device", "")),
        ),
    )[0]


def select_teensy_port(
    ports: Iterable[object],
    *,
    vid: int,
    pid: int,
    serial_number: Optional[str] = None,
    explicit_port: Optional[str] = None,
) -> str:
    if explicit_port:
        return explicit_port

    matches = []
    for port in ports:
        if getattr(port, "vid", None) != vid or getattr(port, "pid", None) != pid:
            continue
        if serial_number and getattr(port, "serial_number", None) != serial_number:
            continue
        matches.append(port)

    if not matches:
        raise SoftRebootError(
            f"no Teensy CDC port found for VID:PID {vid:04x}:{pid:04x}"
        )

    physical_ports: dict[str, list[object]] = {}
    for port in matches:
        physical_ports.setdefault(physical_port_key(port), []).append(port)

    if len(physical_ports) > 1:
        devices = ", ".join(
            sorted(str(getattr(prefer_callout_port(group), "device", "")) for group in physical_ports.values())
        )
        raise SoftRebootError(
            "multiple matching Teensy devices found "
            f"({devices}); pass --port or --serial"
        )

    return str(getattr(prefer_callout_port(next(iter(physical_ports.values()))), "device"))


def trigger_soft_reboot(port: str, *, baudrate: int, settle_seconds: float) -> None:
    if serial is None:
        raise SoftRebootError(f"pyserial is required: {_IMPORT_ERROR}")

    try:
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1,
        ):
            time.sleep(settle_seconds)
    except Exception as exc:
        raise SoftRebootError(f"failed to send soft reboot on {port}: {exc}") from exc


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Find the Teensy USB CDC port and trigger reboot by setting "
            "CDC line coding. Defaults to normal reboot at 135 baud."
        )
    )
    parser.add_argument("--port", help="explicit serial port, e.g. /dev/cu.usbmodem14201")
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID, help="USB VID, default 0x16c0")
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID, help="USB PID, default 0x0483")
    parser.add_argument("--serial", dest="serial_number", help="optional USB serial number filter")
    parser.add_argument(
        "--settle",
        type=float,
        default=DEFAULT_SETTLE_SECONDS,
        help="seconds to keep the port open after setting the reboot baud",
    )
    parser.add_argument(
        "--bootloader",
        action="store_true",
        help="enter Teensy bootloader with 134 baud instead of normal reboot with 135 baud",
    )
    parser.add_argument("--quiet", action="store_true", help="suppress status output")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if list_ports is None:
        print(f"error: pyserial is required: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    try:
        port = select_teensy_port(
            list_ports.comports(),
            vid=args.vid,
            pid=args.pid,
            serial_number=args.serial_number,
            explicit_port=args.port,
        )
        baudrate = (
            TEENSY_SOFT_REBOOT_BAUD_BOOTLOADER
            if args.bootloader
            else TEENSY_SOFT_REBOOT_BAUD_NORMAL
        )
        if not args.quiet:
            mode = "bootloader" if args.bootloader else "normal"
            print(f"Sending Teensy {mode} reboot on {port} at {baudrate} baud")
        trigger_soft_reboot(port, baudrate=baudrate, settle_seconds=args.settle)
    except SoftRebootError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if args.bootloader:
            print(
                "Soft reboot failed; press the Teensy program button when the loader waits.",
                file=sys.stderr,
            )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
