//! CoE (CANopen over EtherCAT) mailbox FSM (non-blocking).
//!
//! IgH: master/fsm_coe.c, master/fsm_coe.h (`ec_fsm_coe_t`) - SDO up/download
//! over the CoE mailbox. v1 implements expedited transfers (<= 4 bytes).
//! Rust: `enum State` + `match` stepped by `step()`, one datagram per step via
//! `Device::pump` (no busy-wait): write the request to the RxMailbox, re-check
//! the TxMailbox-full SM status across steps, then read + parse the response.
//! `Result<_, EcError>` with `SdoAbort(code)` on a slave abort.
//! Dropped (kernel-only): wait-queue blocking -> bounded step/wait counters;
//! segmented/complete-access transfers are deferred.

use crate::ethercat::datagram::{self, Command};
use crate::ethercat::device::{Device, Pump};
use crate::ethercat::ecrt::{read_u32_le, write_u16_le, EcError};
use crate::ethercat::globals::{mbox, reg};
use crate::ethercat::mailbox;
use crate::ethercat::slave::Mailbox;

const PUMP_MAX_ATTEMPTS: u32 = 2_000;
/// Max TxMailbox-full re-checks before declaring the mailbox never filled.
const MAX_MBOX_WAITS: u32 = 1_000;
/// Upper bound on the mailbox read/write length (bounds the 320-byte buffers:
/// 14-byte datagram overhead + 256 = 270).
const MAX_MBOX_READ: usize = 256;
/// Upper bound on the RxMailbox write length.
const MAX_MBOX_WRITE: usize = 256;

/// Expedited SDO transfer direction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SdoKind {
    Download,
    Upload,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    WriteRequest,
    WaitMailbox,
    ReadResponse,
    Done,
}

/// Non-blocking expedited-SDO FSM over one slave's CoE mailbox.
pub struct FsmCoe {
    mbox: Mailbox,
    kind: SdoKind,
    index: u16,
    subindex: u8,
    req_data: [u8; 4],
    req_len: u8,
    state: State,
    pump: Pump,
    tx: [u8; 320],
    tx_len: usize,
    rx: [u8; 320],
    result: [u8; 4],
    result_len: usize,
    waits: u32,
}

impl FsmCoe {
    /// Build an SDO download (write) of `data` (<= 4 bytes) to `index:subindex`.
    pub fn download(mbox: Mailbox, index: u16, subindex: u8, data: &[u8]) -> Self {
        let mut req_data = [0u8; 4];
        let len = data.len().min(4);
        req_data[..len].copy_from_slice(&data[..len]);
        Self::new(mbox, SdoKind::Download, index, subindex, req_data, len as u8)
    }

    /// Build an SDO upload (read) of `index:subindex`.
    pub fn upload(mbox: Mailbox, index: u16, subindex: u8) -> Self {
        Self::new(mbox, SdoKind::Upload, index, subindex, [0u8; 4], 0)
    }

    fn new(mbox: Mailbox, kind: SdoKind, index: u16, subindex: u8, req_data: [u8; 4], req_len: u8) -> Self {
        Self {
            mbox,
            kind,
            index,
            subindex,
            req_data,
            req_len,
            state: State::WriteRequest,
            pump: Pump::new(),
            tx: [0; 320],
            tx_len: 0,
            rx: [0; 320],
            result: [0; 4],
            result_len: 0,
            waits: 0,
        }
    }

    /// The SDO upload response bytes (valid after `step` returns `Ok(true)`).
    pub fn response(&self) -> &[u8] {
        &self.result[..self.result_len]
    }

    /// Advance one datagram. `Ok(true)` = transfer complete, `Ok(false)` =
    /// pending, `Err` = transport/timeout or `SdoAbort(code)`.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        match self.state {
            State::WriteRequest => {
                if self.tx_len == 0 {
                    // Write the FULL configured RxMailbox size: a mailbox SM only
                    // raises "mailbox full" when the master writes its LAST byte.
                    // The header still carries the real service length (10); the
                    // remaining bytes are zero padding. (Mirrors IgH
                    // ec_slave_mbox_prepare_send, which FPWRs the configured size.)
                    let write_len = (self.mbox.rx_size as usize).clamp(16, MAX_MBOX_WRITE);
                    let mut mb = [0u8; MAX_MBOX_WRITE];
                    self.build_request(&mut mb[..16]);
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fpwr,
                        self.mbox.station_addr,
                        self.mbox.rx_offset,
                        &mb[..write_len],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        self.state = State::WaitMailbox;
                        self.tx_len = 0;
                        self.waits = 0;
                        Ok(false)
                    }
                }
            }
            State::WaitMailbox => {
                if self.tx_len == 0 {
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fprd,
                        self.mbox.station_addr,
                        reg::SM1_STATUS,
                        &[0u8; 1],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        let status = reply.data.first().copied().unwrap_or(0);
                        self.tx_len = 0;
                        if reply.working_counter == 1 && status & mbox::SM_STATUS_MBOX_FULL != 0 {
                            self.state = State::ReadResponse;
                            return Ok(false);
                        }
                        self.waits += 1;
                        if self.waits >= MAX_MBOX_WAITS {
                            return Err(EcError::MailboxTimeout);
                        }
                        Ok(false)
                    }
                }
            }
            State::ReadResponse => {
                if self.tx_len == 0 {
                    let read_len = (self.mbox.tx_size as usize).clamp(16, MAX_MBOX_READ);
                    let zeros = [0u8; MAX_MBOX_READ];
                    let i = alloc_index(index);
                    self.tx_len = datagram::build(
                        &mut self.tx,
                        i,
                        Command::Fprd,
                        self.mbox.station_addr,
                        self.mbox.tx_offset,
                        &zeros[..read_len],
                    );
                    self.pump.reset();
                }
                match dev.pump(&mut self.pump, &self.tx[..self.tx_len], &mut self.rx, PUMP_MAX_ATTEMPTS)? {
                    None => Ok(false),
                    Some(len) => {
                        let reply = datagram::parse(&self.rx[..len]).ok_or(EcError::FrameTooShort)?;
                        if reply.working_counter != 1 {
                            return Err(EcError::WorkingCounter);
                        }
                        let (result, result_len) = parse_sdo_response(self.kind, reply.data)?;
                        self.result = result;
                        self.result_len = result_len;
                        self.state = State::Done;
                        Ok(true)
                    }
                }
            }
            State::Done => Ok(true),
        }
    }

    /// Build the 16-byte mailbox+CoE+SDO request into `mb`.
    fn build_request(&self, mb: &mut [u8]) {
        // Mailbox header: data_len = CoE header (2) + SDO (8) = 10.
        mailbox::write_header(&mut mb[0..6], 10, mbox::TYPE_COE, 0);
        // CoE header: service (SDO request) in bits 12..15.
        write_u16_le(&mut mb[6..8], mbox::COE_SDO_REQUEST << 12);
        match self.kind {
            SdoKind::Download => {
                let size = self.req_len.min(4);
                // Download expedited: ccs=1, e=1, s=1, n=4-size.
                mb[8] = 0x23 | ((4 - size) << 2);
                write_u16_le(&mut mb[9..11], self.index);
                mb[11] = self.subindex;
                mb[12..16].copy_from_slice(&self.req_data);
            }
            SdoKind::Upload => {
                mb[8] = 0x40; // upload request (ccs=2)
                write_u16_le(&mut mb[9..11], self.index);
                mb[11] = self.subindex;
                mb[12..16].copy_from_slice(&[0u8; 4]);
            }
        }
    }

}

/// Parse a mailbox SDO response. Returns the (data, len) payload for an upload
/// (empty for a download), or `Err(SdoAbort)` / `Err(MailboxProtocol)`.
fn parse_sdo_response(kind: SdoKind, data: &[u8]) -> Result<([u8; 4], usize), EcError> {
    let header = mailbox::parse_header(data).ok_or(EcError::MailboxProtocol)?;
    if header.mbox_type != mbox::TYPE_COE {
        return Err(EcError::MailboxProtocol);
    }
    // Layout after the 6-byte mailbox header: CoE header (2), SDO command (1),
    // index (2), subindex (1), data/abort (4).
    if data.len() < 16 {
        return Err(EcError::MailboxProtocol);
    }
    let cmd = data[8];
    if cmd & 0xE0 == 0x80 {
        // Abort: 4-byte abort code follows index/subindex.
        return Err(EcError::SdoAbort(read_u32_le(&data[12..16])));
    }
    let mut out = [0u8; 4];
    let len = match kind {
        // Download response ccs=3 (0x60): any non-abort is success.
        SdoKind::Download => 0,
        SdoKind::Upload => {
            // v1 supports only expedited responses (e-bit set, <= 4 bytes). A
            // normal/segmented response carries a 4-byte "complete size" field
            // here, not data, so reject it rather than return wrong bytes.
            if cmd & 0x02 == 0 {
                return Err(EcError::MailboxProtocol);
            }
            let size = (4 - ((cmd >> 2) & 0x03) as usize).min(4);
            out[..size].copy_from_slice(&data[12..12 + size]);
            size
        }
    };
    Ok((out, len))
}

#[inline]
fn alloc_index(index: &mut u8) -> u8 {
    let i = *index;
    *index = index.wrapping_add(1);
    i
}

/// Maximum expedited SDO writes in one [`CoeSeq`] (a PDO mapping is at most
/// `EC_MAX_PDO_ENTRIES` + clear + count).
pub const MAX_SEQ: usize = 36;

/// One expedited SDO write in a sequence.
#[derive(Clone, Copy)]
pub struct SdoWrite {
    pub index: u16,
    pub subindex: u8,
    pub data: [u8; 4],
    pub len: u8,
}

/// A fixed, non-blocking sequence of expedited SDO downloads over one mailbox,
/// driven one datagram per `step` via an inner [`FsmCoe`]. The reusable
/// primitive behind PDO assignment (`fsm_pdo`) and mapping (`fsm_pdo_entry`).
pub struct CoeSeq {
    mbox: Mailbox,
    ops: heapless::Vec<SdoWrite, MAX_SEQ>,
    cur: usize,
    coe: Option<FsmCoe>,
}

impl CoeSeq {
    /// Start an empty sequence on `mbox`.
    pub fn new(mbox: Mailbox) -> Self {
        Self {
            mbox,
            ops: heapless::Vec::new(),
            cur: 0,
            coe: None,
        }
    }

    /// Queue one expedited SDO download (<= 4 bytes). Returns `Err` if full.
    pub fn push(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), ()> {
        let mut buf = [0u8; 4];
        let len = data.len().min(4);
        buf[..len].copy_from_slice(&data[..len]);
        self.ops
            .push(SdoWrite {
                index,
                subindex,
                data: buf,
                len: len as u8,
            })
            .map_err(|_| ())
    }

    /// Advance one datagram. `Ok(true)` = whole sequence complete.
    pub fn step(&mut self, dev: &mut Device, index: &mut u8) -> Result<bool, EcError> {
        if self.cur >= self.ops.len() {
            return Ok(true);
        }
        if self.coe.is_none() {
            let op = self.ops[self.cur];
            self.coe = Some(FsmCoe::download(
                self.mbox,
                op.index,
                op.subindex,
                &op.data[..op.len as usize],
            ));
        }
        if self.coe.as_mut().unwrap().step(dev, index)? {
            self.coe = None;
            self.cur += 1;
        }
        Ok(self.cur >= self.ops.len() && self.coe.is_none())
    }
}
