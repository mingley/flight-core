//! MHS-shaped hardware driver for this workspace.
//!
//! The [Model Hardware Standard](https://modelhardwarestandard.com) is a
//! gated research preview (Anthropic + HHMI Janelia). Official schemas are
//! **not** public. This crate is an **MHS-shaped** adapter: discovery, tags
//! compiled into a reference file, read/write primitives, CLI, demo HTTP, and
//! a stdio MCP tool surface. It does **not** claim official MHS certification.
//!
//! Writes go through [`robot_lab::Lab::act_through_attach`]. There is no raw
//! NED velocity that skips `legal_cmds`. Reads and discovery do not step the
//! plant. Chain files step explicitly (P12: one [`robot_lab::WorldSession`]
//! step per tick). Catalog skips stay P11 (inland has no hull; open_water has
//! no rover).
//!
//! Control paths the public MHS write-up names — MCP, CLI, and code/API —
//! map onto [`mcp`], the `flight-mhs` binary, and [`Driver`].

#![deny(unsafe_code)]

mod chain;
mod driver;
mod error;
mod limits;
mod mcp;
mod schema;
mod surface;
mod tags;

#[cfg(test)]
mod tests;

pub use chain::{ChainDoc, ChainOp, ChainReport};
pub use driver::{queued_action, Driver};
pub use error::{MhsError, MhsFailure};
pub use limits::{DriverLimits, LimitReject};
pub use mcp::{handle_rpc, serve_stdio};
pub use schema::{
    validate_chain_report, validate_discovery, validate_read, validate_reference, validate_write,
    CHAIN_REPORT_SCHEMA, DISCOVERY_SCHEMA, READ_SCHEMA, REFERENCE_SCHEMA, WRITE_SCHEMA,
};
pub use surface::{
    preview_write, read_channel, DeviceReference, DeviceStub, Discovery, Measure, ReadResult,
    SafetyLimit, WriteCapability, WriteOk, WriteRequest,
};
pub use tags::{DeviceTag, DEVICE_ENV, DEVICE_LAB};

/// Adapter profile id. Not an official MHS version.
pub const PROFILE: &str = "flight-core.mhs-shaped.v0";
/// `shaped` means public MHS concepts over this SDK. Not `official`.
pub const CONFORMANCE: &str = "shaped";
pub const SPEC_URL: &str = "https://modelhardwarestandard.com";
pub const SPEC_NOTE: &str = "MHS-shaped adapter for flight-core. Official Model Hardware Standard is a research preview and not open-sourced; this crate does not implement a private wire format. Writes use Lab::act_through_attach. Safety lives in typestate, legal_cmds, driver numeric limits, and remaining-spec P1–P14 — not in prompt text.";
