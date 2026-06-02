//! Desired slave configuration (what the application requests).
//!
//! IgH: master/slave_config.c, master/slave_config.h (`ec_slave_config_t`) - the
//! application's intended configuration of a slave: alias/position, expected
//! vendor/product, SM/PDO assignment, DC parameters, watchdog, and SDO init
//! values. Backbone of the `ecrt_slave_config_*` calls.
//! Rust: an owned struct with fixed-capacity `heapless` collections instead of
//! kmalloc'd lists; `Result` instead of int returns.
//! Dropped (kernel-only): semaphore-guarded request queues -> in-process calls.

// TODO: define EcSlaveConfig (position/alias, expected id, SM/PDO, DC, SDO init).
