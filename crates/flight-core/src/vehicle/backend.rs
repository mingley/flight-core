//! Vehicle backends: the hardware / simulation boundary.
//!
//! A [`Backend`] is the thing a [`super::typestate::Vehicle`] talks to. The
//! simulated backend lives in `flight-sim`; PX4, ROS 2, and MAVLink backends
//! live in their own crates. All of them implement this trait so the vehicle
//! state machine does not care which one is plugged in.
//!
//! ## Capability flags
//!
//! [`BackendCapabilities`] is a bag of booleans the vehicle inspects before
//! issuing a command. A simulated backend claims everything; a real PX4
//! companion may not support optical flow or actuator-direct mode. The
//! vehicle refuses commands the backend cannot honour rather than sending
//! them and hoping.
//!
//! ## Telemetry
//!
//! [`BackendTelemetry`] is the snapshot the vehicle reads after each step:
//! NED position and velocity, attitude, angular velocity, acceleration,
//! armed state, and battery remaining. Units are the crate's newtype units,
//! not raw floats.

use crate::error::CoreError;
use crate::imu::ImuSample;
use crate::units::{
    Acceleration, AngularVelocity, Force, Length, LinearVelocity, Mass, Power, Torque,
};
use crate::vector::Attitude;

/// Kind of backend, used in logs and capability checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendKind {
    Simulated,
    Px4,
    Ros2,
    Mavlink,
}

/// Capability flags. Defaults are conservative (everything `false`); each
/// backend fills in what it actually supports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BackendCapabilities {
    pub offboard: bool,
    pub gps: bool,
    pub rangefinder: bool,
    pub optical_flow: bool,
    pub rc_override: bool,
    pub actuator_direct: bool,
}

impl BackendCapabilities {
    /// Simulated backend: every flag true.
    pub fn simulated() -> Self {
        Self {
            offboard: true,
            gps: true,
            rangefinder: true,
            optical_flow: true,
            rc_override: true,
            actuator_direct: true,
        }
    }
}

/// Snapshot produced by [`Backend::snapshot`]. All linear quantities are NED.
#[derive(Debug, Clone)]
pub struct BackendTelemetry {
    pub position_ned: [Length; 3],
    pub velocity_ned: [LinearVelocity; 3],
    pub attitude: Attitude,
    pub angular_velocity: AngularVelocity,
    pub acceleration: Acceleration,
    pub armed: bool,
    pub battery_remaining: f64,
}

/// Error type returned by backend operations. Carries a human-readable
/// `detail` string so logs can say *why* without the caller matching on
/// twenty variants.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BackendError {
    Unavailable { detail: String },
    Timeout { detail: String },
    Rejected { detail: String },
    Protocol { detail: String },
}

impl std::fmt::Display for BackendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unavailable { detail } => write!(f, "backend unavailable: {detail}"),
            Self::Timeout { detail } => write!(f, "backend timeout: {detail}"),
            Self::Rejected { detail } => write!(f, "backend rejected: {detail}"),
            Self::Protocol { detail } => write!(f, "backend protocol: {detail}"),
        }
    }
}

impl std::error::Error for BackendError {}

impl From<BackendError> for CoreError {
    fn from(err: BackendError) -> Self {
        CoreError::Backend(err.to_string())
    }
}

/// The hardware / simulation boundary.
///
/// Implementors own the connection to the vehicle (or the in-process world)
/// and translate the vehicle's requests into whatever the underlying system
/// understands. The vehicle never talks to PX4 or Gazebo directly.
pub trait Backend {
    fn kind(&self) -> BackendKind;
    fn capabilities(&self) -> BackendCapabilities;

    /// Advance the backend by `dt`. For the simulated backend this steps the
    /// physics world; for a hardware backend this is typically a no-op that
    /// drains telemetry.
    fn step(&mut self, dt: std::time::Duration) -> Result<(), BackendError>;

    /// Current telemetry snapshot. May fail if the backend has not produced
    /// a sample yet (e.g. first step of a simulated world).
    fn snapshot(&self) -> Result<BackendTelemetry, BackendError>;

    /// Last successful snapshot, if any. Used by the vehicle to keep a
    /// stale-but-usable reading when the latest `snapshot()` fails.
    fn last_telemetry(&self) -> Option<BackendTelemetry>;

    /// Apply a body-frame force and torque. Simulated backends apply this to
    /// the rigid body; hardware backends may map it onto actuator setpoints.
    fn send_force_torque(&mut self, force: Force, torque: Torque) -> Result<(), BackendError>;

    /// Override the vehicle mass (payload attach / detach). Hardware backends
    /// that cannot change mass at runtime return [`BackendError::Rejected`].
    fn set_mass(&mut self, mass: Mass) -> Result<(), BackendError>;

    fn last_error(&self) -> Option<&BackendError>;

    /// Latest IMU sample, or an error if the backend has no IMU yet.
    fn imu(&self) -> Result<ImuSample, BackendError>;

    fn dropped_frames(&self) -> u64;
    fn last_step_us(&self) -> u64;
    fn estimated_power(&self) -> Option<Power>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::Attitude;

    struct NullBackend;

    impl Backend for NullBackend {
        fn kind(&self) -> BackendKind {
            BackendKind::Simulated
        }
        fn capabilities(&self) -> BackendCapabilities {
            BackendCapabilities::simulated()
        }
        fn step(&mut self, _dt: std::time::Duration) -> Result<(), BackendError> {
            Ok(())
        }
        fn snapshot(&self) -> Result<BackendTelemetry, BackendError> {
            Ok(BackendTelemetry {
                position_ned: [Length::ZERO; 3],
                velocity_ned: [LinearVelocity::ZERO; 3],
                attitude: Attitude::IDENTITY,
                angular_velocity: AngularVelocity::ZERO,
                acceleration: Acceleration::ZERO,
                armed: false,
                battery_remaining: 1.0,
            })
        }
        fn last_telemetry(&self) -> Option<BackendTelemetry> {
            None
        }
        fn send_force_torque(
            &mut self,
            _force: Force,
            _torque: Torque,
        ) -> Result<(), BackendError> {
            Ok(())
        }
        fn set_mass(&mut self, _mass: Mass) -> Result<(), BackendError> {
            Ok(())
        }
        fn last_error(&self) -> Option<&BackendError> {
            None
        }
        fn imu(&self) -> Result<ImuSample, BackendError> {
            Err(BackendError::Unavailable {
                detail: "null backend has no IMU".into(),
            })
        }
        fn dropped_frames(&self) -> u64 {
            0
        }
        fn last_step_us(&self) -> u64 {
            0
        }
        fn estimated_power(&self) -> Option<Power> {
            None
        }
    }

    #[test]
    fn null_backend_step_succeeds() {
        let mut b = NullBackend;
        assert!(b.step(std::time::Duration::from_millis(10)).is_ok());
    }

    #[test]
    fn capabilities_simulated_all_true() {
        let c = BackendCapabilities::simulated();
        assert!(c.offboard && c.gps && c.rangefinder && c.optical_flow);
    }
}
