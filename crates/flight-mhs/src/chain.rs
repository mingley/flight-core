//! Chain files: driver commands from one or more devices without per-tick reasoning.

use serde::{Deserialize, Serialize};

use crate::error::MhsFailure;
use crate::surface::ReadResult;

fn default_dt() -> f32 {
    0.02
}

fn one() -> u32 {
    1
}

/// One operation in an MHS-shaped code file.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum ChainOp {
    Write {
        device: String,
        channel: String,
        #[serde(default)]
        vn: f32,
        #[serde(default)]
        ve: f32,
        #[serde(default)]
        vd: f32,
        #[serde(default)]
        yaw_rate: f32,
    },
    Read {
        device: String,
        channel: String,
    },
    Step {
        #[serde(default = "default_dt")]
        dt: f32,
        #[serde(default = "one")]
        n: u32,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChainDoc {
    #[serde(default)]
    pub scenario: Option<String>,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default)]
    pub dt: Option<f32>,
    pub ops: Vec<ChainOp>,
}

impl ChainDoc {
    pub fn parse(text: &str) -> Result<Self, crate::MhsError> {
        let trimmed = text.trim();
        if trimmed.is_empty() {
            return Err(crate::MhsError::Chain("empty chain".into()));
        }
        if trimmed.starts_with('[') {
            let ops: Vec<ChainOp> = serde_json::from_str(trimmed)
                .map_err(|e| crate::MhsError::Chain(format!("chain array: {e}")))?;
            return Ok(Self {
                scenario: None,
                seed: None,
                dt: None,
                ops,
            });
        }
        if trimmed.starts_with('{') {
            if let Ok(doc) = serde_json::from_str::<Self>(trimmed) {
                return Ok(doc);
            }
        }
        let mut ops = Vec::new();
        for (i, line) in trimmed.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let op: ChainOp = serde_json::from_str(line)
                .map_err(|e| crate::MhsError::Chain(format!("chain jsonl line {}: {e}", i + 1)))?;
            ops.push(op);
        }
        Ok(Self {
            scenario: None,
            seed: None,
            dt: None,
            ops,
        })
    }
}

/// Outcome of [`crate::Driver::run_chain`].
#[derive(Clone, Debug, Serialize)]
pub struct ChainReport {
    pub scenario: String,
    pub seed: u64,
    pub ops: u32,
    pub steps: u32,
    pub t: f32,
    pub all_hold: bool,
    pub broken: Vec<String>,
    pub certificates: Vec<String>,
    pub rejects: Vec<MhsFailure>,
    pub reads: Vec<ReadResult>,
    pub ok: bool,
}
