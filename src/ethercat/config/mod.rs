//! EtherCAT configuration: model + parser for `ethercat-conf.xml`.
//!
//! - `model`  - data structures mirroring the XML (Master, Slave, SyncManager,
//!              Pdo, PdoEntry, SdoConfig, DcConf, HalType)
//! - `parser` - turns the `ethercat-conf.xml` document into the model

pub mod generated;
pub mod model;
pub mod parser;
