//! Robotics sensor samples: timestamps, frames, units, health, and drop detection.
//!
//! This sits *above* `embedded-hal`. A BMI088 driver still talks SPI; this crate
//! describes what an IMU *means* to a vehicle.

use crate::frames::{Body, Frame};
use crate::time::MonotonicInstant;
use crate::units::RadianPerSecond;
use crate::units::{Celsius, Qty};
use crate::vector::{Acceleration, AngularVelocity};
use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SensorHealth {
    Ok,
    Degraded,
    Timeout,
    Saturated,
    Invalid,
}

impl SensorHealth {
    pub const fn is_usable(self) -> bool {
        matches!(self, Self::Ok | Self::Degraded)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuCovariance {
    /// Diagonal of accel covariance, (m/s²)².
    pub accel_var: [f32; 3],
    /// Diagonal of gyro covariance, (rad/s)².
    pub gyro_var: [f32; 3],
}

impl ImuCovariance {
    pub fn is_valid(self) -> bool {
        self.accel_var
            .iter()
            .chain(self.gyro_var.iter())
            .all(|v| v.is_finite() && *v >= 0.0)
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuSample<F: Frame> {
    pub timestamp: MonotonicInstant,
    pub accel: Acceleration<F>,
    pub gyro: AngularVelocity<RadianPerSecond, F>,
    pub covariance: Option<ImuCovariance>,
    pub temperature: Option<Qty<Celsius>>,
    pub status: SensorHealth,
    pub sequence: u32,
}

impl<F: Frame> ImuSample<F> {
    pub fn is_finite(self) -> bool {
        self.accel.is_finite() && self.gyro.is_finite()
    }

    pub fn is_usable(self) -> bool {
        self.status.is_usable()
            && self.is_finite()
            && self.covariance.map(ImuCovariance::is_valid).unwrap_or(true)
    }
}

pub type BodyImuSample = ImuSample<Body>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorError {
    Timeout,
    Hardware,
    InvalidSample,
    Dropout {
        expected: u32,
        got: u32,
        missed: u32,
    },
}

impl fmt::Display for SensorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => write!(f, "sensor timeout"),
            Self::Hardware => write!(f, "sensor hardware fault"),
            Self::InvalidSample => write!(f, "sensor produced a non-finite or invalid sample"),
            Self::Dropout {
                expected,
                got,
                missed,
            } => write!(
                f,
                "sensor sequence dropout: expected {expected}, got {got}, missed {missed}"
            ),
        }
    }
}

/// IMU the controller samples. Real, recorded, simulated, fuzzed, or symbolic.
pub trait Imu {
    type Frame: Frame;
    fn sample(&mut self) -> Result<ImuSample<Self::Frame>, SensorError>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCommand {
    /// Normalized thrust in `[0, 1]` for up to 8 motors.
    pub motors: [u16; 8],
    pub count: u8,
    /// Collective thrust in newtons, if known.
    pub collective_n: Option<u16>,
}

impl ActuatorCommand {
    pub fn idle(count: u8) -> Self {
        Self {
            motors: [0; 8],
            count,
            collective_n: Some(0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorError {
    NotArmed,
    Disabled,
    Saturating,
}

impl fmt::Display for ActuatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotArmed => write!(f, "actuators commanded while disarmed"),
            Self::Disabled => write!(f, "actuators disabled"),
            Self::Saturating => write!(f, "actuator command saturated"),
        }
    }
}

pub trait Actuators {
    fn apply(&mut self, command: ActuatorCommand) -> Result<(), ActuatorError>;
}

/// Tracks IMU sequence numbers and reports drops / latency.
#[derive(Clone, Debug, Default)]
pub struct SequenceTracker {
    last: Option<u32>,
    drops: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DropReport {
    pub missed: u32,
    pub total_drops: u32,
}

impl SequenceTracker {
    pub const fn new() -> Self {
        Self {
            last: None,
            drops: 0,
        }
    }

    pub fn observe(&mut self, sequence: u32) -> DropReport {
        let missed = match self.last {
            None => 0,
            Some(prev) => sequence.wrapping_sub(prev).saturating_sub(1),
        };
        self.last = Some(sequence);
        self.drops = self.drops.saturating_add(missed);
        DropReport {
            missed,
            total_drops: self.drops,
        }
    }

    pub fn total_drops(&self) -> u32 {
        self.drops
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_sequence_drops() {
        let mut t = SequenceTracker::new();
        assert_eq!(t.observe(0).missed, 0);
        assert_eq!(t.observe(1).missed, 0);
        let r = t.observe(5);
        assert_eq!(r.missed, 3);
        assert_eq!(r.total_drops, 3);
    }

    #[test]
    fn rejects_negative_covariance() {
        let c = ImuCovariance {
            accel_var: [1.0, -0.1, 1.0],
            gyro_var: [0.01, 0.01, 0.01],
        };
        assert!(!c.is_valid());
    }
}
