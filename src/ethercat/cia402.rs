//! CiA 402 drive state machine.
//!
//! IgH: none. CiA 402 is NOT part of the IgH `master/` core (IgH only moves
//! process data; drive profiles live in the application). This file is our
//! application/interface layer and is slated to move under the interface layer
//! once the master core is complete. Kept here during scaffolding for locality.
//!
//! Per-drive control logic over the controlword (0x6040) / statusword (0x6041)
//! that walks a servo from Switch-On-Disabled -> Ready-To-Switch-On ->
//! Switched-On -> Operation-Enabled (and fault handling), plus modes of
//! operation (0x6060/0x6061). Operates on `domain` process data via `hal` pins.

// TODO: implement the CiA 402 controlword/statusword state machine.
