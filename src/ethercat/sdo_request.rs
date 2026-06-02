//! Asynchronous CoE SDO up/download request object.
//!
//! IgH: master/sdo_request.c, master/sdo_request.h (`ec_sdo_request_t`) - a
//! queued SDO read/write with a data buffer, state, and result, driven by
//! `fsm_coe`. Backs `ecrt_sdo_request_*` and the startup `<sdoConfig>` writes.
//! Rust: owned struct with a fixed-capacity byte buffer; state is an `enum`
//! polled by `fsm_coe` rather than a kernel wait-queue.
//! Dropped (kernel-only): `wait_queue_head`/completion -> FSM polling.

// TODO: define EcSdoRequest (index, subindex, data buf, dir, state, result).
