#!/usr/bin/env python3
"""Minimal Modbus TCP register smoke test for the Teensy W5500 firmware."""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys


DEFAULT_HOST = "10.0.0.55"
DEFAULT_PORT = 502
DEFAULT_UNIT_ID = 1
REGISTER_COUNT = 0x23
EXPECTED_REGISTERS = {
    0x0000: 10,
    0x0002: 0,
    0x0004: 0,
    0x0006: 55,
    0x0008: 255,
    0x000A: 255,
    0x000C: 255,
    0x000E: 0,
    0x0010: 10,
    0x0012: 0,
    0x0014: 0,
    0x0016: 1,
    0x0018: 0,
    0x0020: 1,
    0x0022: 0,
}


def read_holding_registers(
    host: str,
    port: int,
    unit_id: int,
    start: int,
    quantity: int,
    timeout: float,
) -> list[int]:
    transaction_id = 1
    pdu = struct.pack(">BHH", 0x03, start, quantity)
    mbap = struct.pack(">HHHB", transaction_id, 0, len(pdu) + 1, unit_id)

    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(mbap + pdu)
        header = recv_exact(sock, 7)
        rx_transaction_id, protocol_id, length, rx_unit_id = struct.unpack(">HHHB", header)
        payload = recv_exact(sock, length - 1)

    if rx_transaction_id != transaction_id:
        raise AssertionError(f"transaction mismatch: {rx_transaction_id} != {transaction_id}")
    if protocol_id != 0:
        raise AssertionError(f"unexpected protocol id: {protocol_id}")
    if rx_unit_id != unit_id:
        raise AssertionError(f"unit id mismatch: {rx_unit_id} != {unit_id}")
    if not payload or payload[0] != 0x03:
        raise AssertionError(f"unexpected function response: {payload.hex()}")
    byte_count = payload[1]
    if byte_count != quantity * 2:
        raise AssertionError(f"byte count mismatch: {byte_count} != {quantity * 2}")

    return [
        struct.unpack(">H", payload[2 + index * 2 : 4 + index * 2])[0]
        for index in range(quantity)
    ]


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed while reading Modbus response")
        chunks.extend(chunk)
    return bytes(chunks)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("MODBUS_HOST", DEFAULT_HOST))
    parser.add_argument("--port", type=int, default=int(os.environ.get("MODBUS_PORT", DEFAULT_PORT)))
    parser.add_argument(
        "--unit-id",
        type=int,
        default=int(os.environ.get("MODBUS_UNIT_ID", DEFAULT_UNIT_ID)),
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    registers = read_holding_registers(
        args.host,
        args.port,
        args.unit_id,
        0,
        REGISTER_COUNT,
        args.timeout,
    )

    for address, expected in EXPECTED_REGISTERS.items():
        actual = registers[address]
        if actual != expected:
            raise AssertionError(
                f"register 0x{address:04X}: expected {expected}, got {actual}"
            )

    print(f"OK: read {len(EXPECTED_REGISTERS)} expected registers from {args.host}:{args.port}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
