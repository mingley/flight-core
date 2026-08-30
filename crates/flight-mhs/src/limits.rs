//! Driver-side numeric limits. Enforced here, not in the agent prompt.

use serde::{Deserialize, Serialize};

/// Numeric clamps applied when a write is otherwise legal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DriverLimits {
    /// Max |v| for aerial `velocity` (m/s, NED).
    pub aerial_speed_mps: f32,
    /// Max |v| for ground `drive` (m/s, NED).
    pub ground_speed_mps: f32,
    /// Max |v| for marine `thrust` (m/s, NED).
    pub marine_speed_mps: f32,
    /// Max |yaw_rate| (rad/s).
    pub yaw_rate_rps: f32,
    /// Max |pose| for aerial `position` (m, NED).
    pub position_m: f32,
    /// Max |wind| (m/s).
    pub wind_mps: f32,
    /// Max |current| (m/s).
    pub current_mps: f32,
    /// Max wave amplitude (m). Matches kernel `set_waves` clamp.
    pub wave_amp_m: f32,
}

impl DriverLimits {
    pub const DEFAULT: Self = Self {
        aerial_speed_mps: 12.0,
        ground_speed_mps: 6.0,
        marine_speed_mps: 6.0,
        yaw_rate_rps: 2.0,
        position_m: 200.0,
        wind_mps: 30.0,
        current_mps: 5.0,
        wave_amp_m: 2.5,
    };
}

impl Default for DriverLimits {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Why a write bounced at the driver numeric gate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LimitReject {
    pub id: String,
    pub device: String,
    pub channel: String,
    pub prose: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub got: Option<f32>,
    pub unit: String,
}

impl LimitReject {
    pub(crate) fn finite(device: &str, channel: &str) -> Self {
        Self {
            id: "finite".into(),
            device: device.into(),
            channel: channel.into(),
            prose: "write values must be finite".into(),
            max: None,
            got: None,
            unit: "".into(),
        }
    }

    pub(crate) fn over(
        id: &str,
        device: &str,
        channel: &str,
        prose: impl Into<String>,
        max: f32,
        got: f32,
        unit: &str,
    ) -> Self {
        Self {
            id: id.into(),
            device: device.into(),
            channel: channel.into(),
            prose: prose.into(),
            max: Some(max),
            got: Some(got),
            unit: unit.into(),
        }
    }
}

pub(crate) fn hypot3(a: f32, b: f32, c: f32) -> f32 {
    (a * a + b * b + c * c).sqrt()
}

pub(crate) fn all_finite(values: &[f32]) -> bool {
    values.iter().all(|v| v.is_finite())
}
