# EtherCAT v1 (bus scan) — deferred follow-ups

Tracking items deferred after the v1 bus-scan implementation. None block the v1
scan working; they are correctness-hardening, robustness, and test-coverage
items to address as the master grows toward cyclic PDO exchange.

Source: correctness review of the scan port against IgH `stable-1.6`.

## Resolved (already fixed in v1)

- `scan_slave` now verifies the working counter on all three per-slave reads
  (APWR station-address, FPRD AL-status `0x0130`, FPRD DL/base `0x0000`), matching
  IgH `fsm_slave_scan.c`. Previously a slave that ACK'd the APWR but dropped the
  AL/base reads could be recorded with a bogus `al_state=0` / zero SM+FMMU counts.

## Residual risks

### 1. Blocking scan starves the priority-1 RTIC executor
> **Update (resolved on the live path):** the runtime scan is now the
> non-blocking [`src/ethercat/fsm_scan.rs`](../src/ethercat/fsm_scan.rs)
> (`ScanFsm`), stepped one datagram at a time via `Device::pump` and reached by
> `rescan` (and the scan implied before `start`). It streams `[scan]` trace lines
> and yields between datagrams, so it no longer monopolizes the executor. The
> boot path also no longer auto-scans (see "Cooperative boot" in
> [`architecture.md`](architecture.md#8-cooperative-boot)). A blocking
> `Device::transact` / `fsm_master::scan_bus` path still exists but is off the
> worker/cyclic hot path. The note below describes the original blocking design.

- Where: [src/main.rs](../src/main.rs) `ethercat_worker` task (the boot scan and
  the `rescan` command) + [src/ethercat/device.rs](../src/ethercat/device.rs)
  `transact`.
- Detail: `Master::scan()` is a synchronous busy-wait (`cortex_m::asm::delay`
  loops in `transact`, up to 50_000 iters/datagram, plus up to 200-iter SII poll
  loops) with no `.await`. While it runs it monopolizes the priority-1
  dispatcher, freezing the same-priority `blink_leds` task for the scan duration
  (worst-case pathological timeouts: seconds). `usb_isr` (priority 2) still
  preempts, so output is unaffected — this is a liveness/responsiveness concern,
  not a wrong-result bug. The runtime command FSMs (state change, CoE SDO) no
  longer have this property: they step one datagram at a time via
  `Device::pump` and `ecat_drive` yields the executor (`Mono::delay`) between
  steps. Only the scan path (`Master::scan` → `fsm_master`/`fsm_slave_scan`/
  `fsm_sii`, reached at boot and via `rescan`) still blocks.
- Fix direction: the non-blocking IgH stepping model (`enum State` + per-step
  `step()` over `Device::pump`) is implemented for the runtime FSMs
  (`FsmChange`, `FsmCoe`). The remaining work is to convert the scan helpers
  (`fsm_master`/`fsm_slave_scan`/`fsm_sii`, still straight-line blocking calls)
  to the same model so the boot scan and `rescan` stop monopolizing the
  dispatcher.

### 2. DMA buffer cache coherency relies on memory placement
- Where: [src/net/enet_driver.rs](../src/net/enet_driver.rs) `send_raw`/`poll_raw`; the
  `ECAT_RXDT`/`ECAT_TXDT` statics in [src/main.rs](../src/main.rs).
- Detail: the raw TX/RX helpers use only `atomic::fence(SeqCst)` (a DMB), not
  SCB cache clean/invalidate. On the Cortex-M7 this is correct only if the
  descriptor tables and buffers live in non-cached memory (TCM, or an MPU-marked
  region). The logic mirrors the existing (working) smoltcp token path, so the
  placement is presumably already non-cached — but confirm the new
  `ECAT_RXDT`/`ECAT_TXDT` statics land in the same non-cached region. If they
  were placed in cached OCRAM, TX would send stale bytes and RX would read stale
  replies despite the code being logically correct.
- Fix direction: verify the linker section / MPU config for these statics; add
  explicit cache maintenance (`SCB::clean_dcache_by_slice` / `invalidate`) if
  they can land in cacheable memory.

### 3. Replies matched on index only (not command), index wraps
- Where: [src/ethercat/device.rs](../src/ethercat/device.rs) `transact`
  (blocking scan) and `pump` (the non-blocking runtime primitive).
- Detail: replies are matched on the datagram index byte (`out[3]`/`rx[3]`)
  only. Safe for the strictly one-outstanding-datagram model, but the index is a
  wrapping `u8` and a single `sii_read_u32` poll loop can emit ~200 datagrams, so
  indices repeat within one scan. A sufficiently delayed stray reply from a prior
  transaction whose index wrapped back could theoretically be accepted.
  Implausible on a controlled broadcast bus, but unguarded. `pump` has the same
  property (it tracks `pump.expected` from the request's index byte only).
- Fix direction: also match the command byte (`out[2]`), and/or carry a
  monotonically-increasing transaction tag; drop frames whose WKC/echo doesn't
  match the request shape.

## Testing gaps (no test harness yet)

- `datagram::parse` adversarial length fields: a reply whose masked `data_len`
  (`& 0x07FF`) exceeds the 128-byte buffer must return `None` (not panic), and
  `data_len == 0` (empty data slice). Bounds-correct by inspection, untested.
- `datagram::autoinc_adp` wraparound: assert `ring_pos=0 -> 0x0000` and
  `ring_pos=1 -> 0xFFFF` (the `-ring_pos` identity).
- `fsm_sii` terminal cases: `STATUS_ERROR (0x20) -> SiiError`, busy bit `0x81`
  clearing -> data at `[6..10]`, the 200-iteration `SiiTimeout`, and the
  `data.len() < 10` / `WKC == 0` retry path.
- Partial slave response: APWR `WKC=1` but a later read `WKC=0` must reject
  (return `Err`) rather than fabricate a `SlaveInfo` (the resolved finding's
  scenario).
- Note: unit-testing `no_std` modules needs a host-target test setup (e.g. a
  `std`-gated test module or a separate test crate) since the firmware target is
  `thumbv7em-none-eabihf`.

## CoE SDO v1 review follow-ups

Source: correctness review of the CoE SDO / mailbox / state-change port against
IgH `stable-1.6` + ETG.1000.

Resolved in this feature:
- RxMailbox write now spans the full configured `rx_size` (mailbox SMs only raise
  "mailbox full" on the last-byte write) -- was the P0 that made every SDO time out.
- Non-expedited (>4-byte) SDO upload responses are rejected cleanly instead of
  returning 4 bytes of the "complete size" field.
- `master.rs` SDO slave lookup is bounds-checked (no panic on OOB index).
- CoE runs in the slave's current mailbox-capable state (PRE-OP/SAFE-OP/OP) and
  only transitions up to PRE-OP from INIT -- no longer downshifts a running slave.

Still deferred:
- Mailbox read/write >256 bytes: [src/ethercat/fsm_coe.rs](../src/ethercat/fsm_coe.rs)
  clamps to `MAX_MBOX_READ/WRITE = 256` (buffers are 320). Slaves with mailboxes
  larger than 256 B would not get their last byte written/read (full flag not
  set/cleared). Enlarge the buffers + caps, or enforce <= 256 B mailboxes.
- Multi-step AL transitions: [src/ethercat/fsm_change.rs](../src/ethercat/fsm_change.rs)
  requests the target directly. EtherCAT only allows single-step (INIT->PREOP->
  SAFEOP->OP); `states -p<n> OP` from INIT returns `StateChange(0x0011)`. Drive
  intermediate states like IgH's master FSM (and reaching SAFE-OP/OP also needs
  SM2/SM3 + FMMU + PDO, which is the PDO phase).
- Stale TxMailbox not flushed before a transfer: correctness currently relies on
  the SMs being re-configured each SDO. IgH checks/clears the mailbox first.
- CoE response service field (header bits 12..15 == 3) is not validated; only the
  mailbox type and SDO command byte are checked.
- Mailbox header address field is written as 0 vs IgH's slave station address
  (CoE-safe; matters for EoE routing).

## Multi-slave bring-up & Distributed Clocks (v1.x)

Source: hardware bring-up of a two-drive bus (both YAKO/Bohign `0x994` / `0x1B00`).

Done in this feature:
- **Multi-slave cyclic operation.** `start` brings up the **whole configured bus**:
  `ConfigSeq` runs the per-slave `FsmSlaveConfig` once per slave to SAFE-OP, then the
  cyclic engine drives **every** slave to OP with per-slave AL gating. One LRW spans
  all slaves and the working counter scales (`+3` per drive); `stop`'s `DownSeq`
  walks every slave back down to PRE-OP. Hardware-verified with two drives reaching
  OP at `wkc = 6/6`, at both 100 Hz and 4 kHz. (The `-p` CLI position is retained for
  compatibility but no longer selects a subset.)
- **Static DC delay/offset compensation.** The per-slave bring-up, before SYNC0
  activation, latches port receive times (`BWR 0x0900`), then per follower writes
  the propagation delay (`0x0928`) and the system-time offset
  (`0x0920` = reference − local, from a coincident single-frame read of both
  system times) so the followers start DC-aligned. Hardware-verified: the
  follower's `0x092C` reads **sub-µs from the first cyclic sample** — ±~77 ns at
  100 Hz and ±~126 ns at 4 kHz (vs 33.8 ms → 0.9 ms converging with the ARMW
  alone). The delay is exact for a line topology; a branched/out-of-range reading
  falls back to offset-only (still sub-µs).
- **Continuous DC reference-time distribution.** In OP with two or more slaves the
  master appends an ARMW of `0x0910` each cycle (auto-increment-addressed at the
  reference slave) to maintain the followers after the static alignment, and reads
  real drift from a follower's `0x092C`.
- **Generator guards for a multi-slave bus.** The config generator is N-slave and
  now rejects a duplicate `halPin` name (pin names index the image globally, so they
  must be namespaced per slave, e.g. `drive0-*` / `drive1-*`) and requires each
  matched ESI device to declare ≥ 4 sync managers (SM0..SM3).

Still deferred:
- **Per-drive SM sync-type + SYNC0-shift options.** Surface the SM2 (outputs,
  master→slave / "m2s") and SM3 (inputs, slave→master / "s2m") **sync-mode**
  parameters (CoE `0x1C32:01` / `0x1C33:01` — free-run / SM-synchronous /
  DC-SYNC0 / DC-SYNC1) and the **SYNC0 shift** as per-drive config options
  (XML `dcConf`/`syncManager` attrs → `model.rs` → codegen → DC/SM bring-up).
  `sync0Shift` is already parsed from the XML but not yet wired into the DC
  activation; the `0x1C32`/`0x1C33` sync-type SDO writes are not yet emitted.
  These are normally per-drive options.
- **First-`start`-after-reflash from a stale drive state.** If the master is
  reflashed (or reset) while the cyclic LRW is running, the drives lose process
  data and watchdog-drop with a latched state; the next `start` can fail the
  SAFE-OP transition (observed `AL status code 0x0000` = timeout). A
  `states -p<n> INIT` on each drive clears it and `start` then succeeds; the clean
  `stop` → `start` path is unaffected. Bring-up should detect a stale/error
  follower and force it through INIT (resetting stale DC) before the static
  compensation runs.
- **Per-drive host enable.** The CiA-402 sequencer (`Cia402::step`) applies one
  enable / fault-reset / quick-stop command **bus-wide** to every discovered drive;
  per-drive host intent is not yet wired through the SPI bridge.
- **Jitter under sustained load** is not yet characterized on hardware. The new
  PIT-CVAL interrupt-latency metric (`latency`/`jitter` in `stats`) exposes it, but
  the load sweep (e.g. driving the SPI host bridge hard) has not been run.

## Other deferred (from the plan, not the review)

- Drop the `smoltcp` dependency from the EtherCAT path (only its `phy` tokens +
  `Instant` are used; the raw `send_raw`/`poll_raw` path no longer needs it).
- `cia402.rs` is app/interface-layer, not IgH core; move it under the interface
  layer once that layer exists.
