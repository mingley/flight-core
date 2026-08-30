//! MHS-shaped driver errors. Not the official Model Hardware Standard wire format.

use robot_lab::{LabCmd, LabError, RejectTrace};
use serde::Serialize;

use crate::limits::LimitReject;

/// Driver-side failure. Writes never skip [`robot_lab::Lab::act_through_attach`].
#[derive(Clone, Debug)]
pub enum MhsError {
    UnknownDevice {
        id: String,
        invariant: Option<&'static str>,
    },
    UnknownChannel {
        device: String,
        channel: String,
    },
    NotLegal {
        device: String,
        cmd: LabCmd,
    },
    Limit(LimitReject),
    ReadOnly {
        device: String,
    },
    UnknownScenario(String),
    Protocol(String),
    Chain(String),
}

impl MhsError {
    pub fn unknown_device(scenario: &str, id: impl Into<String>) -> Self {
        let id = id.into();
        let invariant = catalog_omit(scenario, &id);
        Self::UnknownDevice { id, invariant }
    }

    pub fn code(&self) -> &'static str {
        match self {
            Self::UnknownDevice { .. } => "unknown_device",
            Self::UnknownChannel { .. } => "unknown_channel",
            Self::NotLegal { .. } => "not_legal",
            Self::Limit(_) => "limit",
            Self::ReadOnly { .. } => "read_only",
            Self::UnknownScenario(_) => "unknown_scenario",
            Self::Protocol(_) => "protocol",
            Self::Chain(_) => "chain",
        }
    }

    pub fn invariant(&self) -> Option<&str> {
        match self {
            Self::UnknownDevice { invariant, .. } => *invariant,
            _ => None,
        }
    }

    /// JSON an agent or HTTP route can log. `ok` is always false.
    pub fn as_failure(&self, reject: Option<RejectTrace>) -> MhsFailure {
        MhsFailure {
            ok: false,
            code: self.code().into(),
            error: self.to_string(),
            invariant: self.invariant().map(str::to_string),
            reject,
            limit: match self {
                Self::Limit(l) => Some(l.clone()),
                _ => None,
            },
        }
    }
}

impl From<LabError> for MhsError {
    fn from(e: LabError) -> Self {
        match e {
            LabError::UnknownRobot(id) => Self::UnknownDevice {
                id,
                invariant: None,
            },
            LabError::NotLegal { robot, cmd } => Self::NotLegal { device: robot, cmd },
            LabError::UnknownScenario(s) => Self::UnknownScenario(s),
            other => Self::Protocol(other.to_string()),
        }
    }
}

impl std::fmt::Display for MhsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownDevice { id, invariant } => match invariant {
                Some(p) => write!(f, "unknown device '{id}' ({p})"),
                None => write!(f, "unknown device '{id}'"),
            },
            Self::UnknownChannel { device, channel } => {
                write!(f, "unknown channel '{channel}' on '{device}'")
            }
            Self::NotLegal { device, cmd } => write!(f, "not legal now: {device} {cmd}"),
            Self::Limit(l) => write!(f, "driver limit {}: {}", l.id, l.prose),
            Self::ReadOnly { device } => write!(f, "device '{device}' is read-only"),
            Self::UnknownScenario(s) => write!(f, "unknown scenario '{s}'"),
            Self::Protocol(s) | Self::Chain(s) => f.write_str(s),
        }
    }
}

impl std::error::Error for MhsError {}

/// Serialized driver failure (HTTP / MCP / CLI).
#[derive(Clone, Debug, Serialize)]
pub struct MhsFailure {
    pub ok: bool,
    pub code: String,
    pub error: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub invariant: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reject: Option<RejectTrace>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limit: Option<LimitReject>,
}

pub(crate) fn catalog_omit(scenario: &str, id: &str) -> Option<&'static str> {
    let inland_hull = scenario == "inland" && matches!(id, "skiff" | "surveyor");
    let open_rover = scenario == "open_water" && id == "rover";
    (inland_hull || open_rover).then_some("P11")
}
