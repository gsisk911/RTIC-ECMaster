#!/usr/bin/env python3
"""Generate `src/ethercat/config/generated.rs` from a lcec-style bus XML plus a
vendor ESI, at COMPILE TIME (no XML is parsed on the MCU).

Inputs:
  --bus  ethercat-conf.bohign.xml   the desired bus (master cycle, slaves, PDO
                                     assignment/mapping, DC, SDO init, halPins)
  --esi  Bohign_MS_ECAT_V2.5.xml     the vendor device description; provides each
                                     slave's SM2/SM3 physical start + control byte

Output:
  src/ethercat/config/generated.rs   one `pub const BUS: BusCfg`

The script: matches each bus <slave> to an ESI <Device> by product code, pulls
the process-data SM physical addresses + control bytes, computes the per-entry
(byte_offset, bit_position) over the domain image (all outputs then all inputs),
validates SDO-init payloads fit an expedited transfer (<= 4 bytes), and renders
the Rust const tables.  Stdlib only (xml.etree.ElementTree).

Run via `make config`, then commit the regenerated file (never hand-edit it).
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUS = ROOT / "ethercat-conf.bohign.xml"
DEFAULT_ESI = ROOT / "Bohign_MS_ECAT_V2.5.xml"
RUST_PATH = ROOT / "src" / "ethercat" / "config" / "generated.rs"

HAL_TYPES = {"bit": "HalType::Bit", "u32": "HalType::U32", "s32": "HalType::S32"}


def parse_hex(text):
    """Parse an XML hex value: '0x1B00', '#x1B00', or plain decimal."""
    text = text.strip()
    if text.startswith("#x") or text.startswith("#X"):
        return int(text[2:], 16)
    if text.startswith("0x") or text.startswith("0X"):
        return int(text, 16)
    return int(text, 10)


def esi_devices(esi_path):
    """Map ESI product code -> {'sm': {idx: (phys_start, control)}}."""
    tree = ET.parse(esi_path)
    devices = {}
    for dev in tree.iter("Device"):
        type_el = dev.find("Type")
        if type_el is None or "ProductCode" not in type_el.attrib:
            continue
        product = parse_hex(type_el.attrib["ProductCode"])
        sms = []  # ordered SM list; index by appearance (SM0, SM1, SM2, SM3...)
        for sm in dev.findall("Sm"):
            start = parse_hex(sm.attrib.get("StartAddress", "0"))
            control = parse_hex(sm.attrib.get("ControlByte", "0"))
            sms.append((start, control))
        devices[product] = sms
    return devices


def entry_fields(entry):
    """(index, subindex, bit_len, hal_type, hal_pin) for a <pdoEntry>.

    In the lcec dialect, pdoEntry idx/subIdx are bare hex (no `0x`).
    """
    idx = int(entry.attrib["idx"], 16)
    sub = int(entry.attrib.get("subIdx", "0"), 16)
    bits = int(entry.attrib["bitLen"])
    hal_type = entry.attrib.get("halType", "u32").lower()
    hal_pin = entry.attrib.get("halPin")
    return idx, sub, bits, hal_type, hal_pin


def parse_bus(bus_path):
    tree = ET.parse(bus_path)
    master = tree.find(".//master")
    cycle_ns = int(master.attrib.get("appTimePeriod", "1000000"))
    ref_clock = int(master.attrib.get("refClockSlaveIdx", "0"))

    slaves = []
    for slave in master.findall("slave"):
        pos = int(slave.attrib["idx"])
        vid = parse_hex(slave.attrib["vid"])
        pid = parse_hex(slave.attrib["pid"])

        dc = slave.find("dcConf")
        dc_cfg = None
        if dc is not None:
            sync0 = dc.attrib.get("sync0Cycle", "*1")
            sync0_ns = cycle_ns if sync0.startswith("*") else int(sync0)
            if sync0.startswith("*"):
                sync0_ns = cycle_ns * int(sync0[1:])
            dc_cfg = {
                "assign_activate": parse_hex(dc.attrib.get("assignActivate", "0")),
                "sync0_cycle_ns": sync0_ns,
                "sync0_shift_ns": int(dc.attrib.get("sync0Shift", "0")),
                "sync1_cycle_ns": 0,
            }

        sdo_init = []
        for sdo in slave.findall("sdoConfig"):
            raw = sdo.find("sdoDataRaw")
            data = [int(b, 16) for b in raw.attrib["data"].split()]
            if len(data) > 4:
                raise SystemExit(
                    f"slave {pos} SDO 0x{parse_hex(sdo.attrib['idx']):04X}: "
                    f"{len(data)} bytes > 4 (expedited only)"
                )
            sdo_init.append(
                {
                    "index": parse_hex(sdo.attrib["idx"]),
                    "subindex": parse_hex(sdo.attrib.get("subIdx", "0")),
                    "data": data,
                }
            )

        out_sms, in_sms = [], []
        for sm in slave.findall("syncManager"):
            sm_idx = int(sm.attrib["idx"])
            direction = sm.attrib["dir"]  # 'out' (RxPDO) or 'in' (TxPDO)
            pdos = []
            for pdo in sm.findall("pdo"):
                entries = [entry_fields(e) for e in pdo.findall("pdoEntry")]
                pdos.append({"index": parse_hex("0x" + pdo.attrib["idx"]), "entries": entries})
            sm_rec = {"index": sm_idx, "dir": direction, "pdos": pdos}
            (out_sms if direction == "out" else in_sms).append(sm_rec)

        slaves.append(
            {
                "position": pos,
                "vid": vid,
                "pid": pid,
                "dc": dc_cfg,
                "sdo_init": sdo_init,
                "out_sms": out_sms,
                "in_sms": in_sms,
            }
        )
    return cycle_ns, ref_clock, slaves


def sm_bytes(sms):
    total = 0
    for sm in sms:
        for pdo in sm["pdos"]:
            total += sum(bits for (_, _, bits, _, _) in pdo["entries"])
    return total // 8


def resolve(cycle_ns, ref_clock, slaves, esi):
    """Attach SM phys/control from the ESI and compute image offsets + pins."""
    # First pass: per-slave out/in sizes.
    for s in slaves:
        s["out_size"] = sm_bytes(s["out_sms"])
        s["in_size"] = sm_bytes(s["in_sms"])
    total_out = sum(s["out_size"] for s in slaves)

    pins = []
    out_cursor = 0
    in_cursor = total_out
    for s in slaves:
        sms = esi.get(s["pid"])
        if sms is None:
            raise SystemExit(f"product 0x{s['pid']:04X} not found in ESI")
        # ESI SM order is SM0, SM1, SM2, SM3, ...; index directly.
        s["sm_phys"] = {2: sms[2][0], 3: sms[3][0]}
        s["sm_ctrl"] = {2: sms[2][1], 3: sms[3][1]}
        s["out_logical"] = out_cursor
        s["in_logical"] = in_cursor

        for sm in s["out_sms"]:
            bit_acc = 0
            for (idx, sub, bits, hal_type, hal_pin) in sm["pdos"][0]["entries"] if sm["pdos"] else []:
                if hal_pin:
                    pins.append(
                        {
                            "name": hal_pin,
                            "byte_offset": out_cursor + bit_acc // 8,
                            "bit_pos": bit_acc % 8,
                            "bit_len": bits,
                            "hal_type": hal_type,
                            "dir": "Output",
                        }
                    )
                bit_acc += bits
        for sm in s["in_sms"]:
            bit_acc = 0
            for (idx, sub, bits, hal_type, hal_pin) in sm["pdos"][0]["entries"] if sm["pdos"] else []:
                if hal_pin:
                    pins.append(
                        {
                            "name": hal_pin,
                            "byte_offset": in_cursor + bit_acc // 8,
                            "bit_pos": bit_acc % 8,
                            "bit_len": bits,
                            "hal_type": hal_type,
                            "dir": "Input",
                        }
                    )
                bit_acc += bits
        out_cursor += s["out_size"]
        in_cursor += s["in_size"]

    return {"cycle_ns": cycle_ns, "ref_clock": ref_clock, "slaves": slaves, "pins": pins, "image": in_cursor}


def render_entries(name, sm_list):
    lines = [f"const {name}: &[EcPdoEntryInfo] = &["]
    for sm in sm_list:
        for pdo in sm["pdos"]:
            for (idx, sub, bits, _, _) in pdo["entries"]:
                lines.append(f"    e(0x{idx:04X}, 0x{sub:02X}, {bits}),")
    lines.append("];")
    return "\n".join(lines)


def render_pdos(name, entries_name, sm_list):
    pdo = sm_list[0]["pdos"][0]
    return (
        f"const {name}: &[PdoCfg] = &[PdoCfg {{\n"
        f"    index: 0x{pdo['index']:04X},\n"
        f"    entries: {entries_name},\n"
        f"}}];"
    )


def render(cfg):
    out = []
    out.append(
        "//! GENERATED EtherCAT bus configuration -- do not edit by hand.\n"
        "//!\n"
        "//! Produced by `scripts/generate_ethercat_config.py` (run `make config`)."
        " Holds one\n"
        "//! `pub const BUS: BusCfg` consumed by the bring-up FSM, the process-data"
        " domain,\n"
        "//! and the HAL pin layer. Regenerate and commit; never hand-edit.\n"
    )
    out.append(
        "use super::model::{BusCfg, DcCfg, FmmuCfg, HalType, PdoCfg, PinCfg, SdoInit, SlaveCfg, SmCfg};"
    )
    out.append("use crate::ethercat::ecrt::{EcDirection, EcPdoEntryInfo};\n")
    out.append(
        "const fn e(index: u16, subindex: u8, bit_length: u8) -> EcPdoEntryInfo {\n"
        "    EcPdoEntryInfo { index, subindex, bit_length }\n"
        "}\n"
    )

    slave_consts = []
    for s in cfg["slaves"]:
        p = s["position"]
        out.append(f"// --- Slave {p}: product 0x{s['pid']:08X} ---")
        out.append(render_entries(f"S{p}_RX_ENTRIES", s["out_sms"]))
        out.append(render_entries(f"S{p}_TX_ENTRIES", s["in_sms"]))
        out.append(render_pdos(f"S{p}_RX_PDOS", f"S{p}_RX_ENTRIES", s["out_sms"]))
        out.append(render_pdos(f"S{p}_TX_PDOS", f"S{p}_TX_ENTRIES", s["in_sms"]))
        out.append(
            f"const S{p}_SMS: &[SmCfg] = &[\n"
            f"    SmCfg {{ index: 2, phys_start: 0x{s['sm_phys'][2]:04X}, control: 0x{s['sm_ctrl'][2]:02X}, dir: EcDirection::Output, size: {s['out_size']}, pdos: S{p}_RX_PDOS }},\n"
            f"    SmCfg {{ index: 3, phys_start: 0x{s['sm_phys'][3]:04X}, control: 0x{s['sm_ctrl'][3]:02X}, dir: EcDirection::Input, size: {s['in_size']}, pdos: S{p}_TX_PDOS }},\n"
            f"];"
        )
        out.append(
            f"const S{p}_FMMUS: &[FmmuCfg] = &[\n"
            f"    FmmuCfg {{ logical_start: {s['out_logical']}, size: {s['out_size']}, phys_start: 0x{s['sm_phys'][2]:04X}, dir: EcDirection::Output }},\n"
            f"    FmmuCfg {{ logical_start: {s['in_logical']}, size: {s['in_size']}, phys_start: 0x{s['sm_phys'][3]:04X}, dir: EcDirection::Input }},\n"
            f"];"
        )
        if s["sdo_init"]:
            rows = []
            for sdo in s["sdo_init"]:
                data = ", ".join(f"0x{b:02X}" for b in sdo["data"])
                rows.append(
                    f"    SdoInit {{ index: 0x{sdo['index']:04X}, subindex: 0x{sdo['subindex']:02X}, data: &[{data}] }},"
                )
            out.append(f"const S{p}_SDO_INIT: &[SdoInit] = &[\n" + "\n".join(rows) + "\n];")
        else:
            out.append(f"const S{p}_SDO_INIT: &[SdoInit] = &[];")

        dc = s["dc"]
        dc_str = "None"
        if dc:
            dc_str = (
                f"Some(DcCfg {{ assign_activate: 0x{dc['assign_activate']:04X}, "
                f"sync0_cycle_ns: {dc['sync0_cycle_ns']}, sync0_shift_ns: {dc['sync0_shift_ns']}, "
                f"sync1_cycle_ns: {dc['sync1_cycle_ns']} }})"
            )
        slave_consts.append(
            f"    SlaveCfg {{ position: {p}, vendor_id: 0x{s['vid']:08X}, product_code: 0x{s['pid']:08X}, "
            f"sms: S{p}_SMS, fmmus: S{p}_FMMUS, dc: {dc_str}, sdo_init: S{p}_SDO_INIT, "
            f"out_size: {s['out_size']}, in_size: {s['in_size']} }},"
        )

    out.append("const SLAVES: &[SlaveCfg] = &[\n" + "\n".join(slave_consts) + "\n];")

    pin_rows = []
    for pin in cfg["pins"]:
        pin_rows.append(
            f"    PinCfg {{ name: {json.dumps(pin['name'])}, byte_offset: {pin['byte_offset']}, "
            f"bit_pos: {pin['bit_pos']}, bit_len: {pin['bit_len']}, "
            f"hal_type: {HAL_TYPES[pin['hal_type']]}, dir: EcDirection::{pin['dir']} }},"
        )
    out.append("const PINS: &[PinCfg] = &[\n" + "\n".join(pin_rows) + "\n];")

    out.append(
        "pub const BUS: BusCfg = BusCfg {\n"
        f"    cycle_ns: {cfg['cycle_ns']},\n"
        f"    ref_clock_slave: {cfg['ref_clock']},\n"
        "    slaves: SLAVES,\n"
        "    pins: PINS,\n"
        f"    image_size: {cfg['image']},\n"
        "};\n"
    )
    return "\n\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bus", type=Path, default=DEFAULT_BUS)
    ap.add_argument("--esi", type=Path, default=DEFAULT_ESI)
    ap.add_argument("--out", type=Path, default=RUST_PATH)
    args = ap.parse_args()

    esi = esi_devices(args.esi)
    cycle_ns, ref_clock, slaves = parse_bus(args.bus)
    cfg = resolve(cycle_ns, ref_clock, slaves, esi)
    args.out.write_text(render(cfg) + "\n")
    print(
        f"[generate_ethercat_config] wrote {args.out} "
        f"({len(cfg['slaves'])} slave(s), {cfg['image']}-byte image, {len(cfg['pins'])} pins)"
    )


if __name__ == "__main__":
    main()
