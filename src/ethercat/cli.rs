//! Serial command interface, mirroring the IgH `ethercat` command-line tool.
//!
//! IgH: tool/ (`CommandUpload.cpp`, `CommandDownload.cpp`, `DataTypeHandler.cpp`).
//! Not an IgH `master/` file -- this is our interface layer, equivalent to the
//! userspace `ethercat` CLI, parsing typed serial lines into master `Request`s.
//! Rust: a `no_std` line parser (no allocation beyond fixed `heapless` buffers).
//! Mirrors the IgH forms: `upload -p<pos> -t<type> <index> <sub>`,
//! `download -p<pos> -t<type> <index> <sub> <value>`, `states -p<pos> <STATE>`,
//! `slaves`, `rescan`. Numbers are base-from-prefix (`0x` hex, `0b` binary, else
//! decimal); upload output is `0x<hex> <dec>` per IgH `outputData`.
//!
//! v1 supports the native datatypes that fit an expedited SDO (<= 4 bytes):
//! bool, int8/16/32, uint8/16/32. Larger/float/string types are deferred.

use crate::ethercat::globals::al_state;
use core::fmt::Write as _;

/// An SDO datatype (subset of IgH `DataTypeHandler::dataTypes`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SdoType {
    pub name: &'static str,
    pub code: u16,
    pub size: u8,
}

/// `monitor` auto-emit control: explicit on/off, or toggle for the bare command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MonitorMode {
    On,
    Off,
    Toggle,
}

/// Lowest cyclic rate accepted by `start -r<hz>` (Hz).
pub const RATE_MIN_HZ: u32 = 50;
/// Highest cyclic rate accepted by `start -r<hz>` (Hz). The PIT is sized for up
/// to 4 kHz in normal use; this leaves headroom for experimentation.
pub const RATE_MAX_HZ: u32 = 8_000;

const TYPES: &[SdoType] = &[
    SdoType { name: "bool", code: 0x0001, size: 1 },
    SdoType { name: "int8", code: 0x0002, size: 1 },
    SdoType { name: "int16", code: 0x0003, size: 2 },
    SdoType { name: "int32", code: 0x0004, size: 4 },
    SdoType { name: "uint8", code: 0x0005, size: 1 },
    SdoType { name: "uint16", code: 0x0006, size: 2 },
    SdoType { name: "uint32", code: 0x0007, size: 4 },
];

/// Lines printed for the `help` command.
pub const HELP: &[&str] = &[
    "[ecat] commands (IgH ethercat tool form):",
    "  slaves                                   list discovered slaves",
    "  status                                   firmware, link state, slave count",
    "  rescan                                   re-run the bus scan",
    "  states -p<pos> <INIT|PREOP|SAFEOP|OP>    request an AL state",
    "  upload -p<pos> -t<type> <idx> <sub>      SDO read (0x.. hex, else decimal)",
    "  download -p<pos> -t<type> <idx> <sub> <value>   SDO write",
    "  start [-p<pos>] [-r<hz>]                 configure + start cyclic PDO (50..8000 Hz)",
    "  stop                                     stop cyclic PDO",
    "  stats                                    cyclic rate, jitter, DC sync error",
    "  monitor [on|off]                         stream stats ~every 500ms (bare = toggle)",
    "  pdos                                     list process-data pins and offsets",
    "  pd [<pin> [<value>]]                     dump image / read pin / write pin",
    "  host                                     Pi/LinuxCNC SPI bridge diagnostics",
    "  crashlog                                 show the saved fault/panic context",
    "  crashclear                               clear the saved fault/panic context",
    "  types: bool int8 int16 int32 uint8 uint16 uint32",
];

/// A parsed serial command.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Command {
    Empty,
    Help,
    Slaves,
    Status,
    Rescan,
    States {
        slave: u16,
        target: u8,
    },
    Upload {
        slave: u16,
        index: u16,
        subindex: u8,
        ty: Option<SdoType>,
    },
    Download {
        slave: u16,
        index: u16,
        subindex: u8,
        ty: SdoType,
        data: [u8; 4],
        len: u8,
    },
    /// Configure a slave and start the cyclic process-data engine at an optional
    /// rate (`-r<hz>`); `rate_hz` is `None` to use the compile-time rate.
    Start {
        slave: u16,
        rate_hz: Option<u32>,
    },
    /// Stop the cyclic engine.
    Stop,
    /// Report cyclic-engine telemetry (rate, jitter, DC sync error).
    Stats,
    /// Enable/disable periodic auto-emit of compact telemetry over serial.
    Monitor(MonitorMode),
    /// List the resolved process-data pins (name -> offset).
    Pdos,
    /// Read/write the process image: no pin = dump; pin = read; pin+value = write.
    Pd {
        pin: Option<heapless::String<48>>,
        value: Option<i64>,
    },
    /// Pi/LinuxCNC host-SPI bridge diagnostics (link, watchdog, CRC/seq errors,
    /// motion buffer depth, frame sizes).
    Host,
    /// Show the saved fault/panic context persisted across the last reboot.
    Crashlog,
    /// Clear the saved fault/panic context.
    Crashclear,
    Error(heapless::String<96>),
}

/// Find a datatype by name.
pub fn find_type(name: &str) -> Option<SdoType> {
    TYPES.iter().copied().find(|t| t.name == name)
}

fn err(msg: &str) -> Command {
    let mut s = heapless::String::new();
    let _ = s.push_str(msg);
    Command::Error(s)
}

/// Parse one serial line into a [`Command`].
pub fn parse(line: &str) -> Command {
    let line = line.trim();
    if line.is_empty() {
        return Command::Empty;
    }
    let mut toks: heapless::Vec<&str, 16> = heapless::Vec::new();
    for t in line.split_whitespace() {
        if toks.push(t).is_err() {
            return err("too many arguments");
        }
    }
    match toks[0] {
        "help" | "?" => Command::Help,
        "slaves" => Command::Slaves,
        "status" | "info" => Command::Status,
        "rescan" => Command::Rescan,
        "states" | "state" => parse_states(&toks[1..]),
        "upload" | "up" => parse_sdo(&toks[1..], false),
        "download" | "down" => parse_sdo(&toks[1..], true),
        "start" => parse_start(&toks[1..]),
        "stop" => Command::Stop,
        "stats" => Command::Stats,
        "monitor" | "mon" => parse_monitor(&toks[1..]),
        "pdos" => Command::Pdos,
        "pd" => parse_pd(&toks[1..]),
        "host" => Command::Host,
        "crashlog" => Command::Crashlog,
        "crashclear" => Command::Crashclear,
        _ => err("unknown command; type 'help'"),
    }
}

fn parse_start(toks: &[&str]) -> Command {
    let o = match parse_opts(toks) {
        Ok(o) => o,
        Err(e) => return e,
    };
    match o.rate {
        Some(hz) if !(RATE_MIN_HZ..=RATE_MAX_HZ).contains(&hz) => {
            err("rate out of range (50..8000 Hz)")
        }
        rate_hz => Command::Start {
            slave: o.position.unwrap_or(0),
            rate_hz,
        },
    }
}

/// Parse `monitor [on|off]`: bare command toggles, `on`/`off` are explicit.
fn parse_monitor(toks: &[&str]) -> Command {
    match toks.first().copied() {
        None => Command::Monitor(MonitorMode::Toggle),
        Some(t) if t.eq_ignore_ascii_case("on") => Command::Monitor(MonitorMode::On),
        Some(t) if t.eq_ignore_ascii_case("off") => Command::Monitor(MonitorMode::Off),
        _ => err("usage: monitor [on|off]"),
    }
}

fn pin_name(s: &str) -> heapless::String<48> {
    let mut o = heapless::String::new();
    let _ = o.push_str(s);
    o
}

fn parse_pd(toks: &[&str]) -> Command {
    match toks.len() {
        0 => Command::Pd {
            pin: None,
            value: None,
        },
        1 => Command::Pd {
            pin: Some(pin_name(toks[0])),
            value: None,
        },
        2 => match parse_int(toks[1]) {
            Some(v) => Command::Pd {
                pin: Some(pin_name(toks[0])),
                value: Some(v),
            },
            None => err("invalid value"),
        },
        _ => err("usage: pd [<pin> [<value>]]"),
    }
}

/// Parsed options: position, type name, cyclic rate, and the positional
/// arguments. `rate` is only consumed by `start`; other commands ignore it.
struct Opts<'a> {
    position: Option<u16>,
    ty: Option<&'a str>,
    rate: Option<u32>,
    args: heapless::Vec<&'a str, 8>,
}

fn parse_opts<'a>(toks: &[&'a str]) -> Result<Opts<'a>, Command> {
    let mut o = Opts {
        position: None,
        ty: None,
        rate: None,
        args: heapless::Vec::new(),
    };
    let mut i = 0;
    while i < toks.len() {
        let t = toks[i];
        if t == "-p" || t == "--position" {
            i += 1;
            let v = toks.get(i).ok_or_else(|| err("missing value for -p"))?;
            o.position = Some(parse_uint(v).ok_or_else(|| err("invalid position"))? as u16);
        } else if let Some(rest) = t.strip_prefix("-p") {
            o.position = Some(parse_uint(rest).ok_or_else(|| err("invalid position"))? as u16);
        } else if t == "-r" || t == "--rate" {
            i += 1;
            let v = toks.get(i).ok_or_else(|| err("missing value for -r"))?;
            o.rate = Some(parse_uint(v).ok_or_else(|| err("invalid rate"))? as u32);
        } else if let Some(rest) = t.strip_prefix("-r") {
            o.rate = Some(parse_uint(rest).ok_or_else(|| err("invalid rate"))? as u32);
        } else if t == "-t" || t == "--type" {
            i += 1;
            o.ty = Some(toks.get(i).copied().ok_or_else(|| err("missing value for -t"))?);
        } else if let Some(rest) = t.strip_prefix("--type=") {
            o.ty = Some(rest);
        } else if let Some(rest) = t.strip_prefix("-t") {
            o.ty = Some(rest);
        } else if t.starts_with('-') {
            return Err(err("unknown option"));
        } else if o.args.push(t).is_err() {
            return Err(err("too many arguments"));
        }
        i += 1;
    }
    Ok(o)
}

fn parse_states(toks: &[&str]) -> Command {
    let o = match parse_opts(toks) {
        Ok(o) => o,
        Err(e) => return e,
    };
    let slave = match o.position {
        Some(p) => p,
        None => return err("states requires -p<pos>"),
    };
    if o.args.len() != 1 {
        return err("usage: states -p<pos> <INIT|PREOP|SAFEOP|OP>");
    }
    let target = match al_state_from_name(o.args[0]) {
        Some(s) => s,
        None => return err("unknown state (INIT|PREOP|SAFEOP|OP)"),
    };
    Command::States { slave, target }
}

fn parse_sdo(toks: &[&str], download: bool) -> Command {
    let o = match parse_opts(toks) {
        Ok(o) => o,
        Err(e) => return e,
    };
    let slave = match o.position {
        Some(p) => p,
        None => return err("requires -p<pos>"),
    };
    let want_args = if download { 3 } else { 2 };
    if o.args.len() != want_args {
        return if download {
            err("usage: download -p<pos> -t<type> <idx> <sub> <value>")
        } else {
            err("usage: upload -p<pos> [-t<type>] <idx> <sub>")
        };
    }
    let index = match parse_uint(o.args[0]) {
        Some(v) if v <= u16::MAX as u64 => v as u16,
        _ => return err("invalid index"),
    };
    let subindex = match parse_uint(o.args[1]) {
        Some(v) if v <= u8::MAX as u64 => v as u8,
        _ => return err("invalid subindex"),
    };
    let ty = match o.ty {
        Some(name) => match find_type(name) {
            Some(t) => Some(t),
            None => return err("unknown type; see 'help'"),
        },
        None => None,
    };

    if download {
        let ty = match ty {
            Some(t) => t,
            None => return err("download requires -t<type>"),
        };
        match encode_value(ty, o.args[2]) {
            Some((data, len)) => Command::Download {
                slave,
                index,
                subindex,
                ty,
                data,
                len,
            },
            None => err("invalid value for type"),
        }
    } else {
        Command::Upload {
            slave,
            index,
            subindex,
            ty,
        }
    }
}

fn al_state_from_name(name: &str) -> Option<u8> {
    if name.eq_ignore_ascii_case("init") {
        Some(al_state::INIT)
    } else if name.eq_ignore_ascii_case("preop") {
        Some(al_state::PREOP)
    } else if name.eq_ignore_ascii_case("safeop") {
        Some(al_state::SAFEOP)
    } else if name.eq_ignore_ascii_case("op") {
        Some(al_state::OP)
    } else {
        None
    }
}

/// Parse an unsigned integer, base-from-prefix (`0x` hex, `0b` binary, else
/// decimal). Mirrors the IgH `resetiosflags(ios::basefield)` behavior.
pub fn parse_uint(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(h, 16).ok()
    } else if let Some(b) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        u64::from_str_radix(b, 2).ok()
    } else {
        s.parse::<u64>().ok()
    }
}

/// Parse a signed integer (for download values), base-from-prefix.
fn parse_int(s: &str) -> Option<i64> {
    let s = s.trim();
    if let Some(rest) = s.strip_prefix('-') {
        Some(-(parse_uint(rest)? as i64))
    } else {
        // Accept both unsigned-range and small signed decimals.
        parse_uint(s).map(|v| v as i64)
    }
}

/// Encode a value string into little-endian bytes for `ty`.
fn encode_value(ty: SdoType, s: &str) -> Option<([u8; 4], u8)> {
    let v = parse_int(s)?;
    let size = ty.size as usize;
    let raw = v as u64;
    let mut out = [0u8; 4];
    for (i, byte) in out.iter_mut().enumerate().take(size) {
        *byte = (raw >> (8 * i)) as u8;
    }
    Some((out, ty.size))
}

/// Format an SDO upload result for display, mirroring IgH `outputData`:
/// `0x<hex, type width> <decimal>`. Without a type, raw hex bytes.
pub fn format_value(ty: Option<SdoType>, data: &[u8]) -> heapless::String<80> {
    let mut s = heapless::String::new();
    match ty {
        None => {
            for (i, b) in data.iter().enumerate() {
                if i > 0 {
                    let _ = s.push(' ');
                }
                let _ = write!(s, "0x{:02x}", b);
            }
        }
        Some(t) => {
            let size = (t.size as usize).min(data.len());
            let mut raw: u32 = 0;
            for i in 0..size {
                raw |= (data[i] as u32) << (8 * i);
            }
            let hexw = (t.size as usize) * 2;
            let signed = matches!(t.code, 0x0002 | 0x0003 | 0x0004);
            if signed {
                let _ = write!(s, "0x{:0width$x} {}", raw, sign_extend(raw, t.size), width = hexw);
            } else {
                let _ = write!(s, "0x{:0width$x} {}", raw, raw, width = hexw);
            }
        }
    }
    s
}

fn sign_extend(raw: u32, size: u8) -> i32 {
    match size {
        1 => raw as u8 as i8 as i32,
        2 => raw as u16 as i16 as i32,
        _ => raw as i32,
    }
}
