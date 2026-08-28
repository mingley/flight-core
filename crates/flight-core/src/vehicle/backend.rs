//! Vehicle backend trait, telemetry, and command types.

use crate::frames::{Body, Ned};
use crate::safety::Phase;
use crate::sensors::{ActuatorCommand, ImuSample, SensorHealth};
use crate::time::MonotonicInstant;
use crate::units::{Meter, Qty};
use crate::vector::{Position, Velocity};
use core::fmt;
use core::future::Future;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConnectionInfo {
    pub system_id: u8,
    pub component_id: u8,
    pub autopilot: AutopilotKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AutopilotKind {
    Simulated,
    Px4,
    Unknown,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PreflightReport {
    pub imu_healthy: bool,
    pub estimator_valid: bool,
    pub battery_ok: bool,
    pub gps_ok: bool,
    pub notes: PreflightNotes,
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PreflightNotes {
    pub imu_std_accel: f32,
    pub imu_std_gyro: f32,
    pub samples: u32,
}

impl PreflightReport {
    pub fn ready(&self) -> bool {
        self.imu_healthy && self.estimator_valid && self.battery_ok
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorThrust {
    /// Normalized `[0, 1]` per motor.
    pub motors: [f32; 8],
    pub count: u8,
}

impl MotorThrust {
    pub fn hover(count: u8, fraction: f32) -> Self {
        let mut motors = [0.0; 8];
        let n = count.min(8) as usize;
        for m in motors.iter_mut().take(n) {
            *m = fraction.clamp(0.0, 1.0);
        }
        Self {
            motors,
            count: n as u8,
        }
    }

    pub fn to_actuator(self) -> ActuatorCommand {
        let mut motors = [0u16; 8];
        for (slot, frac) in motors.iter_mut().zip(self.motors.iter()) {
            *slot = (frac.clamp(0.0, 1.0) * 65535.0) as u16;
        }
        ActuatorCommand {
            motors,
            count: self.count,
            collective_n: None,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Telemetry {
    pub timestamp: MonotonicInstant,
    pub phase: Phase,
    pub position: Position<Ned>,
    pub velocity: Velocity<Ned>,
    pub yaw_rad: f32,
    pub imu: Option<ImuSample<Body>>,
    pub imu_health: SensorHealth,
    pub imu_healthy: bool,
    pub estimator_valid: bool,
    pub armed: bool,
    pub actuators_enabled: bool,
    pub offboard: bool,
    pub failsafe: bool,
    pub heartbeat_age_secs: f32,
    pub last_command: &'static str,
}

impl Telemetry {
    pub fn altitude_agl(&self) -> Qty<Meter> {
        self.position.altitude_agl()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendError {
    Disconnected,
    Timeout,
    Protocol,
    Rejected(&'static str),
    Io,
}

impl fmt::Display for BackendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "backend disconnected"),
            Self::Timeout => write!(f, "backend timeout"),
            Self::Protocol => write!(f, "backend protocol error"),
            Self::Rejected(r) => write!(f, "backend rejected: {r}"),
            Self::Io => write!(f, "backend I/O error"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for BackendError {}

/// Hardware / SITL / replay / symbolic vehicle.
///
/// The controller talks only to this trait. It does not know whether the IMU
/// is a BMI088, an MCAP recording, a physics sim, or a Kani nondet value.
pub trait VehicleBackend: Send {
    fn connect(&mut self) -> impl Future<Output = Result<ConnectionInfo, BackendError>> + Send;
    fn preflight(&mut self) -> impl Future<Output = Result<PreflightReport, BackendError>> + Send;
    fn arm(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn disarm(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn enter_offboard(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_velocity_ned(
        &mut self,
        velocity: Velocity<Ned>,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_position_ned(
        &mut self,
        position: Position<Ned>,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_motor_thrust(
        &mut self,
        thrust: MotorThrust,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn enable_actuators(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn disable_actuators(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn tick(
        &mut self,
        dt_secs: f32,
    ) -> impl Future<Output = Result<Telemetry, BackendError>> + Send;
    fn telemetry(&mut self) -> impl Future<Output = Result<Telemetry, BackendError>> + Send;
    fn trigger_failsafe(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
}

/// Backend that succeeds every call. Used to unit-test typestate wiring.
#[derive(Clone, Debug, Default)]
pub struct NullBackend {
    pub armed: bool,
    pub offboard: bool,
    pub actuators: bool,
    pub velocity: Option<Velocity<Ned>>,
    pub ticks: u32,
}

impl VehicleBackend for NullBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        Ok(ConnectionInfo {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotKind::Simulated,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        Ok(PreflightReport {
            imu_healthy: true,
            estimator_valid: true,
            battery_ok: true,
            gps_ok: true,
            notes: PreflightNotes {
                imu_std_accel: 0.01,
                imu_std_gyro: 0.001,
                samples: 50,
            },
        })
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        self.armed = true;
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        self.armed = false;
        self.actuators = false;
        self.offboard = false;
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        self.offboard = true;
        Ok(())
    }

    async fn set_velocity_ned(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        self.velocity = Some(velocity);
        Ok(())
    }

    async fn set_position_ned(&mut self, _position: Position<Ned>) -> Result<(), BackendError> {
        Ok(())
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        Ok(())
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        self.actuators = true;
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        self.actuators = false;
        Ok(())
    }

    async fn tick(&mut self, _dt_secs: f32) -> Result<Telemetry, BackendError> {
        self.ticks += 1;
        self.telemetry().await
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        Ok(Telemetry {
            timestamp: MonotonicInstant::from_millis(u64::from(self.ticks) * 10),
            phase: Phase::Ready,
            position: Position::ned(0.0, 0.0, 0.0),
            velocity: self.velocity.unwrap_or_else(Velocity::zero),
            yaw_rad: 0.0,
            imu: None,
            imu_health: SensorHealth::Ok,
            imu_healthy: true,
            estimator_valid: true,
            armed: self.armed,
            actuators_enabled: self.actuators,
            offboard: self.offboard,
            failsafe: false,
            heartbeat_age_secs: 0.0,
            last_command: "null",
        })
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        Ok(())
    }
}
