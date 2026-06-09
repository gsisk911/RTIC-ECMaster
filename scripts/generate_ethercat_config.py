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
SPI_LAYOUT_PATH = ROOT / "src" / "hal" / "spi_layout_generated.rs"
PI_HEADER_PATH = ROOT / "linuxcnc" / "teensy_bridge_layout.h"

HAL_TYPES = {"bit": "HalType::Bit", "u32": "HalType::U32", "s32": "HalType::S32"}

# Host SPI frame header sizes (must match src/hal/host_bridge.rs).
MOSI_HDR = 8
MISO_HDR = 18
CRC_LEN = 2
STREAM_HDR = 1


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
    """(index, subindex, bit_len, hal_type, hal_pin, cls) for a <pdoEntry>.

    In the lcec dialect, pdoEntry idx/subIdx are bare hex (no `0x`). `class`
    (default "immediate") marks an output entry as part of the look-ahead motion
    stream (`class="motion"`) vs the immediate output block.
    """
    idx = int(entry.attrib["idx"], 16)
    sub = int(entry.attrib.get("subIdx", "0"), 16)
    bits = int(entry.attrib["bitLen"])
    hal_type = entry.attrib.get("halType", "u32").lower()
    hal_pin = entry.attrib.get("halPin")
    cls = entry.attrib.get("class", "immediate").lower()
    return idx, sub, bits, hal_type, hal_pin, cls


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

        # Optional look-ahead motion stream: `<motionStream lead="N">`. The
        # streamed *fields* are marked per-entry with class="motion"; this block
        # only carries the lead depth (a master default applies otherwise).
        motion = slave.find("motionStream")
        motion_lead = int(motion.attrib.get("lead", "0")) if motion is not None else 0

        slaves.append(
            {
                "position": pos,
                "vid": vid,
                "pid": pid,
                "dc": dc_cfg,
                "sdo_init": sdo_init,
                "out_sms": out_sms,
                "in_sms": in_sms,
                "motion_lead": motion_lead,
            }
        )
    return cycle_ns, ref_clock, slaves


def sm_bytes(sms):
    total = 0
    for sm in sms:
        for pdo in sm["pdos"]:
            total += sum(bits for (_, _, bits, _, _, _) in pdo["entries"])
    return total // 8


def resolve(cycle_ns, ref_clock, slaves, esi):
    """Attach SM phys/control from the ESI and compute image offsets + pins."""
    # First pass: per-slave out/in sizes.
    for s in slaves:
        s["out_size"] = sm_bytes(s["out_sms"])
        s["in_size"] = sm_bytes(s["in_sms"])
    total_out = sum(s["out_size"] for s in slaves)

    pins = []
    # Streamed motion fields (class="motion"), in image order: (sample_off,
    # image_off, len_bytes). One combined sample per cycle across all axes.
    motion_fields = []
    sample_off = 0
    out_cursor = 0
    in_cursor = total_out
    for s in slaves:
        sms = esi.get(s["pid"])
        if sms is None:
            raise SystemExit(f"product 0x{s['pid']:04X} not found in ESI")
        # Process data lives in SM2/SM3, so the device must declare >= 4 SMs.
        # Guard explicitly so a mismatched second-slave ESI fails with a clear
        # message instead of an IndexError.
        if len(sms) < 4:
            raise SystemExit(
                f"slave {s['position']} product 0x{s['pid']:04X}: ESI declares "
                f"{len(sms)} sync manager(s), need >= 4 (SM0..SM3)"
            )
        # ESI SM order is SM0, SM1, SM2, SM3, ...; index directly.
        s["sm_phys"] = {2: sms[2][0], 3: sms[3][0]}
        s["sm_ctrl"] = {2: sms[2][1], 3: sms[3][1]}
        s["out_logical"] = out_cursor
        s["in_logical"] = in_cursor

        # Outputs: iterate every PDO in the SM (multi-PDO slaves map fully).
        for sm in s["out_sms"]:
            bit_acc = 0
            for pdo in sm["pdos"]:
                for (idx, sub, bits, hal_type, hal_pin, cls) in pdo["entries"]:
                    byte_off = out_cursor + bit_acc // 8
                    if hal_pin:
                        pins.append(
                            {
                                "name": hal_pin,
                                "byte_offset": byte_off,
                                "bit_pos": bit_acc % 8,
                                "bit_len": bits,
                                "hal_type": hal_type,
                                "dir": "Output",
                            }
                        )
                    if cls == "motion":
                        if bits % 8 != 0 or bit_acc % 8 != 0:
                            raise SystemExit(
                                f"motion entry {hal_pin or hex(idx)} must be byte-aligned"
                            )
                        nbytes = bits // 8
                        motion_fields.append(
                            {"sample_off": sample_off, "image_off": byte_off, "len": nbytes}
                        )
                        sample_off += nbytes
                    bit_acc += bits
        # Inputs: iterate every PDO in the SM.
        for sm in s["in_sms"]:
            bit_acc = 0
            for pdo in sm["pdos"]:
                for (idx, sub, bits, hal_type, hal_pin, cls) in pdo["entries"]:
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

    # Pin names index the process image globally, so they MUST be unique across
    # all slaves (BUS.pin() / the HAL / CiA-402 drive discovery resolve by name).
    # With a multi-slave bus the lcec convention namespaces them per slave
    # (drive0-*, drive1-*); catch a missing prefix at generate time rather than
    # let a duplicate silently alias to the first slave's offset.
    seen_pins = set()
    for p in pins:
        if p["name"] in seen_pins:
            raise SystemExit(
                f"duplicate halPin name '{p['name']}'; namespace pins per slave "
                f"(e.g. drive0-*, drive1-*)"
            )
        seen_pins.add(p["name"])

    stream_sample_bytes = sample_off
    leads = [s["motion_lead"] for s in slaves if s["motion_lead"] > 0]
    if stream_sample_bytes > 0:
        default_lead = max(leads) if leads else 8
        # Batch cap: enough to refill the whole look-ahead in one frame, bounded
        # by the firmware ring depth (motion_buffer::MOTION_RING_DEPTH = 32).
        max_samples = min(max(default_lead, 1), 32)
    else:
        default_lead = 0
        max_samples = 0

    return {
        "cycle_ns": cycle_ns,
        "ref_clock": ref_clock,
        "slaves": slaves,
        "pins": pins,
        "image": in_cursor,
        "out_bytes": total_out,
        "in_bytes": in_cursor - total_out,
        "motion_fields": motion_fields,
        "stream_sample_bytes": stream_sample_bytes,
        "default_lead": default_lead,
        "max_samples_per_frame": max_samples,
    }


def render_pdo_block(prefix, sm_list):
    """Emit per-PDO entry consts + the `PdoCfg` array for a slave direction,
    supporting multiple PDOs per sync manager. Returns (text, pdos_const_name)."""
    chunks = []
    pdo_refs = []
    k = 0
    for sm in sm_list:
        for pdo in sm["pdos"]:
            ename = f"{prefix}_P{k}_ENTRIES"
            lines = [f"const {ename}: &[EcPdoEntryInfo] = &["]
            for (idx, sub, bits, _, _, _) in pdo["entries"]:
                lines.append(f"    e(0x{idx:04X}, 0x{sub:02X}, {bits}),")
            lines.append("];")
            chunks.append("\n".join(lines))
            pdo_refs.append((pdo["index"], ename))
            k += 1
    pname = f"{prefix}_PDOS"
    plines = [f"const {pname}: &[PdoCfg] = &["]
    for (pidx, ename) in pdo_refs:
        plines.append(f"    PdoCfg {{ index: 0x{pidx:04X}, entries: {ename} }},")
    plines.append("];")
    chunks.append("\n".join(plines))
    return "\n\n".join(chunks), pname


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
        rx_text, rx_pdos = render_pdo_block(f"S{p}_RX", s["out_sms"])
        tx_text, tx_pdos = render_pdo_block(f"S{p}_TX", s["in_sms"])
        out.append(rx_text)
        out.append(tx_text)
        out.append(
            f"const S{p}_SMS: &[SmCfg] = &[\n"
            f"    SmCfg {{ index: 2, phys_start: 0x{s['sm_phys'][2]:04X}, control: 0x{s['sm_ctrl'][2]:02X}, dir: EcDirection::Output, size: {s['out_size']}, pdos: {rx_pdos} }},\n"
            f"    SmCfg {{ index: 3, phys_start: 0x{s['sm_phys'][3]:04X}, control: 0x{s['sm_ctrl'][3]:02X}, dir: EcDirection::Input, size: {s['in_size']}, pdos: {tx_pdos} }},\n"
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


def frame_lengths(cfg):
    """Compute the host SPI frame lengths from the config (mirrors host_bridge)."""
    out_b = cfg["out_bytes"]
    in_b = cfg["in_bytes"]
    stride = 4 + cfg["stream_sample_bytes"]
    host_stream = cfg["max_samples_per_frame"] * stride
    mosi = MOSI_HDR + out_b + STREAM_HDR + host_stream + CRC_LEN
    miso = MISO_HDR + in_b + CRC_LEN
    return {
        "out_bytes": out_b,
        "in_bytes": in_b,
        "sample_stride": stride,
        "host_stream_bytes": host_stream,
        "mosi_len": mosi,
        "miso_len": miso,
        "frame_len": max(mosi, miso),
    }


def render_spi_layout(cfg):
    """Render src/hal/spi_layout_generated.rs (streamed-motion layout)."""
    fields = cfg["motion_fields"]
    out = []
    out.append(
        "//! GENERATED host-SPI streamed-motion layout -- do not edit by hand.\n"
        "//!\n"
        "//! Produced by `scripts/generate_ethercat_config.py` (run `make config`)"
        " from the\n"
        "//! bus XML's `class=\"motion\"` entries + `<motionStream>` blocks. Shared by"
        " the\n"
        "//! firmware motion buffer and the Pi HAL component. Regenerate and commit.\n"
    )
    out.append("use crate::ethercat::config::model::StreamField;")
    out.append(
        f"/// Bytes in one streamed motion sample (sum of the per-axis fields).\n"
        f"pub const STREAM_SAMPLE_BYTES: usize = {cfg['stream_sample_bytes']};"
    )
    out.append(
        f"/// Maximum streamed samples per SPI frame (batch-refill cap).\n"
        f"pub const MAX_SAMPLES_PER_FRAME: usize = {cfg['max_samples_per_frame']};"
    )
    # DEFAULT_LEAD is a host-side concept (emitted in the Pi header), not used by
    # the firmware, so it is intentionally not emitted into the Rust layout.
    if fields:
        rows = "\n".join(
            f"    StreamField {{ sample_off: {f['sample_off']}, "
            f"image_off: {f['image_off']}, len: {f['len']} }},"
            for f in fields
        )
        out.append(
            "/// The streamed fields: where each lands in the cyclic image.\n"
            f"pub const STREAM_FIELDS: &[StreamField] = &[\n{rows}\n];"
        )
    else:
        out.append(
            "/// The streamed fields: where each lands in the cyclic image.\n"
            "pub const STREAM_FIELDS: &[StreamField] = &[];"
        )
    return "\n\n".join(out)


def render_pi_header(cfg):
    """Render the Pi-side C header (the shared frame/pin contract for the HAL
    component). One source of truth with the firmware: regenerated together."""
    fl = frame_lengths(cfg)
    lines = []
    lines.append("/* GENERATED Teensy<->Pi host-bridge frame contract -- do not edit by hand. */")
    lines.append("/* Produced by scripts/generate_ethercat_config.py (make config). */")
    lines.append("#ifndef TEENSY_BRIDGE_LAYOUT_H")
    lines.append("#define TEENSY_BRIDGE_LAYOUT_H")
    lines.append("")
    lines.append("#define TEENSY_MAGIC 0xA7ECu")
    lines.append("#define TEENSY_VERSION 1u")
    lines.append(f"#define TEENSY_MOSI_LEN {fl['mosi_len']}")
    lines.append(f"#define TEENSY_MISO_LEN {fl['miso_len']}")
    lines.append(f"#define TEENSY_FRAME_LEN {fl['frame_len']}")
    lines.append(f"#define TEENSY_OUT_BYTES {fl['out_bytes']}")
    lines.append(f"#define TEENSY_IN_BYTES {fl['in_bytes']}")
    lines.append(f"#define TEENSY_MOSI_HDR {MOSI_HDR}")
    lines.append(f"#define TEENSY_MISO_HDR {MISO_HDR}")
    lines.append(f"#define TEENSY_STREAM_OFF {MOSI_HDR + fl['out_bytes']}")
    lines.append(f"#define TEENSY_SAMPLE_BYTES {cfg['stream_sample_bytes']}")
    lines.append(f"#define TEENSY_SAMPLE_STRIDE {fl['sample_stride']}")
    lines.append(f"#define TEENSY_MAX_SAMPLES_PER_FRAME {cfg['max_samples_per_frame']}")
    lines.append(f"#define TEENSY_DEFAULT_LEAD {cfg['default_lead']}")
    lines.append("")
    lines.append("/* MOSI flags / MISO status bits. */")
    lines.append("#define TEENSY_FLAG_ENABLE      (1u<<0)")
    lines.append("#define TEENSY_FLAG_FAULT_RESET (1u<<1)")
    lines.append("#define TEENSY_FLAG_QUICK_STOP  (1u<<2)")
    lines.append("#define TEENSY_ST_LINK          (1u<<0)")
    lines.append("#define TEENSY_ST_OPERATIONAL   (1u<<1)")
    lines.append("#define TEENSY_ST_FAULT         (1u<<2)")
    lines.append("#define TEENSY_ST_HOST_TIMEOUT  (1u<<3)")
    lines.append("")
    lines.append("/* One process-data pin: name, frame byte offset, bit pos/len, type, dir. */")
    lines.append("typedef struct {")
    lines.append("    const char *name;")
    lines.append("    int frame_off;   /* byte offset within the MOSI (out) or MISO (in) frame */")
    lines.append("    int bit_pos;     /* bit offset within frame_off (for 'b' pins) */")
    lines.append("    int bit_len;")
    lines.append("    char type;       /* 'b' bit, 'u' u32, 's' s32 */")
    lines.append("    char dir;        /* 'o' output (host->drive), 'i' input (drive->host) */")
    lines.append("} teensy_pin_t;")
    lines.append("")
    type_ch = {"bit": "'b'", "u32": "'u'", "s32": "'s'"}
    rows = []
    for pin in cfg["pins"]:
        if pin["dir"] == "Output":
            frame_off = MOSI_HDR + pin["byte_offset"]
            d = "'o'"
        else:
            frame_off = MISO_HDR + (pin["byte_offset"] - cfg["out_bytes"])
            d = "'i'"
        rows.append(
            f'    {{ "{pin["name"]}", {frame_off}, {pin["bit_pos"]}, {pin["bit_len"]}, '
            f'{type_ch[pin["hal_type"]]}, {d} }},'
        )
    lines.append("static const teensy_pin_t TEENSY_PINS[] = {")
    lines.extend(rows)
    lines.append("};")
    lines.append(f"#define TEENSY_PIN_COUNT {len(cfg['pins'])}")
    lines.append("")
    lines.append("/* Streamed motion fields: which output pin sources each sample slice. */")
    lines.append("typedef struct {")
    lines.append("    int sample_off;  /* byte offset within the motion sample payload */")
    lines.append("    int pin_index;   /* index into TEENSY_PINS providing the value */")
    lines.append("    int len;         /* byte length */")
    lines.append("} teensy_motion_t;")
    lines.append("")
    motion_rows = []
    for f in cfg["motion_fields"]:
        # Find the output pin whose image offset matches this motion field.
        pin_index = next(
            (
                i
                for i, p in enumerate(cfg["pins"])
                if p["dir"] == "Output" and p["byte_offset"] == f["image_off"]
            ),
            -1,
        )
        motion_rows.append(
            f"    {{ {f['sample_off']}, {pin_index}, {f['len']} }},"
        )
    lines.append("static const teensy_motion_t TEENSY_MOTION[] = {")
    lines.extend(motion_rows)
    if not motion_rows:
        lines.append("    { 0, -1, 0 } /* none */")
    lines.append("};")
    lines.append(
        f"#define TEENSY_MOTION_COUNT {len(cfg['motion_fields'])}"
    )
    lines.append("")
    lines.append("#endif /* TEENSY_BRIDGE_LAYOUT_H */")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bus", type=Path, default=DEFAULT_BUS)
    ap.add_argument("--esi", type=Path, default=DEFAULT_ESI)
    ap.add_argument("--out", type=Path, default=RUST_PATH)
    ap.add_argument("--spi-layout", type=Path, default=SPI_LAYOUT_PATH)
    ap.add_argument("--pi-header", type=Path, default=PI_HEADER_PATH)
    args = ap.parse_args()

    esi = esi_devices(args.esi)
    cycle_ns, ref_clock, slaves = parse_bus(args.bus)
    cfg = resolve(cycle_ns, ref_clock, slaves, esi)
    args.out.write_text(render(cfg) + "\n")
    args.spi_layout.write_text(render_spi_layout(cfg) + "\n")
    args.pi_header.parent.mkdir(parents=True, exist_ok=True)
    args.pi_header.write_text(render_pi_header(cfg) + "\n")
    fl = frame_lengths(cfg)
    print(
        f"[generate_ethercat_config] wrote {args.out} "
        f"({len(cfg['slaves'])} slave(s), {cfg['image']}-byte image, {len(cfg['pins'])} pins)"
    )
    print(
        f"[generate_ethercat_config] wrote {args.spi_layout} + {args.pi_header} "
        f"(frame {fl['frame_len']} B, {cfg['stream_sample_bytes']}-B motion sample, "
        f"lead {cfg['default_lead']})"
    )


if __name__ == "__main__":
    main()
