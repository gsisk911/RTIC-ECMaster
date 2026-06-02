#!/usr/bin/env python3
"""Generate `src/modbus/register_map.rs` from `src/modbus/registers.json`.

The JSON file is the source of truth for the Rust Modbus register contract.
Each register entry must contain:

- `address`: hex string, e.g. `0x0100`
- `name`: snake_case logical name
- `default`: u16 default value
- `saveable`: bool persisted to EEPROM
"""

from pathlib import Path
import json
import re


ROOT = Path(__file__).resolve().parents[1]
JSON_PATH = ROOT / "src" / "modbus" / "registers.json"
RUST_PATH = ROOT / "src" / "modbus" / "register_map.rs"


def const_name(name: str) -> str:
    return "REG_" + re.sub(r"[^A-Z0-9]+", "_", name.upper()).strip("_")


def parse_registers():
    with JSON_PATH.open("r", encoding="utf-8") as f:
        data = json.load(f)

    registers = data["registers"]
    seen_addr = set()
    seen_name = set()
    parsed = []
    last_address = -1
    for entry in registers:
        address = int(entry["address"], 16)
        name = entry["name"]
        default = int(entry["default"]) & 0xFFFF
        saveable = bool(entry.get("saveable", False))
        if address & 0x1:
            raise ValueError(f"register {name} must use an even address (got 0x{address:04X})")
        if address in seen_addr:
            raise ValueError(f"duplicate address 0x{address:04X}")
        if name in seen_name:
            raise ValueError(f"duplicate name {name}")
        if address <= last_address:
            raise ValueError(
                f"registers must be strictly ascending: {name} at 0x{address:04X} after 0x{last_address:04X}"
            )
        seen_addr.add(address)
        seen_name.add(name)
        last_address = address
        parsed.append(
            {
                "address": address,
                "name": name,
                "default": default,
                "saveable": saveable,
                "const": const_name(name),
            }
        )
    return parsed


def render(registers):
    const_lines = []
    for reg in registers:
        const_lines.append(
            f"pub const {reg['const']}: u16 = 0x{reg['address']:04X};"
        )

    table_lines = []
    for reg in registers:
        saveable = "true" if reg["saveable"] else "false"
        table_lines.append(
            "    RegisterDef { "
            f"address: 0x{reg['address']:04X}, "
            f"default_value: {reg['default']}, "
            f'name: "{reg["name"]}", '
            f"saveable: {saveable} "
            "},"
        )

    return f"""//! Minimal Modbus holding-register map for the generic device base.
//!
//! This file is generated from `src/modbus/registers.json` by
//! `scripts/generate_registers.py`.

/// A single register definition: address, power-on default, human name,
/// and whether the register is persisted to EEPROM.
#[derive(Clone, Copy)]
pub struct RegisterDef {{
    pub address: u16,
    pub default_value: u16,
    pub name: &'static str,
    pub saveable: bool,
}}

{chr(10).join(const_lines)}

pub const USER_REGISTERS: &[RegisterDef] = &[
{chr(10).join(table_lines)}
];
"""


def main():
    registers = parse_registers()
    rendered = render(registers)
    RUST_PATH.write_text(rendered, encoding="utf-8")
    print(f"[generate_registers] wrote {RUST_PATH}")


if __name__ == "__main__":
    main()
