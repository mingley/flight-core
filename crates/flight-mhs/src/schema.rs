//! JSON Schema for discovery / reference / read / write / chain (NEXT E1).

use robot_lab::validate_instance;

pub const DISCOVERY_SCHEMA: &str = include_str!("../schemas/discovery.json");
pub const REFERENCE_SCHEMA: &str = include_str!("../schemas/reference.json");
pub const READ_SCHEMA: &str = include_str!("../schemas/read.json");
pub const WRITE_SCHEMA: &str = include_str!("../schemas/write.json");
pub const CHAIN_REPORT_SCHEMA: &str = include_str!("../schemas/chain_report.json");

pub fn validate_discovery(instance: &serde_json::Value) -> Result<(), String> {
    validate_instance(DISCOVERY_SCHEMA, instance)
}

pub fn validate_reference(instance: &serde_json::Value) -> Result<(), String> {
    validate_instance(REFERENCE_SCHEMA, instance)
}

pub fn validate_read(instance: &serde_json::Value) -> Result<(), String> {
    validate_instance(READ_SCHEMA, instance)
}

pub fn validate_write(instance: &serde_json::Value) -> Result<(), String> {
    validate_instance(WRITE_SCHEMA, instance)
}

pub fn validate_chain_report(instance: &serde_json::Value) -> Result<(), String> {
    validate_instance(CHAIN_REPORT_SCHEMA, instance)
}
