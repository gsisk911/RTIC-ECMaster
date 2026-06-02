//! Per-slave FSM dispatcher.
//!
//! IgH: master/fsm_slave.c, master/fsm_slave.h (`ec_fsm_slave_t`) - runs the
//! scan/config sub-FSMs for one slave and services its pending mailbox requests
//! (SDO/FoE/SoE/EoE/reg) one at a time.
//! Rust: the C function-pointer `state` member becomes an `enum State` stepped
//! by `match`; sub-FSMs are owned values rather than embedded structs.
//! Dropped (kernel-only): `down_interruptible` request locking -> single-owner
//! borrow; v1 dispatches only the scan.

/// Dispatcher states (subset; expands with config + mailbox requests).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Idle,
    Scan,
    Config,
}

// TODO: implement the per-slave dispatcher stepping scan/config/requests.
