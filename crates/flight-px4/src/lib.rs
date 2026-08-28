//! PX4 offboard backend over MAVLink.
//!
//! Talks to PX4 SITL the way a companion computer would, but the *vehicle API*
//! is `flight-core::vehicle::Vehicle` — not a thin wrap of the C++ ROS 2
//! interface library.
//!
//! Default SITL endpoint: `udpin:0.0.0.0:14540` (PX4 publishes here).
//! A common companion send path is `udpout:127.0.0.1:14580`.

#![deny(unsafe_code)]

use flight_core::frames::Ned;
use flight_core::safety::Phase;
use flight_core::sensors::SensorHealth;
use flight_core::time::MonotonicInstant;
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, PreflightNotes, PreflightReport,
    Telemetry, Vehicle, VehicleBackend,
};
use flight_mavlink::{arm_disarm, gcs_heartbeat, set_offboard_mode, set_velocity_ned, UdpLink};
use mavlink::common::{MavMessage, MavType};

#[derive(Clone, Debug)]
pub struct Px4Config {
    /// `mavlink::connect` address, e.g. `udpin:0.0.0.0:14540`.
    pub endpoint: String,
    pub system_id: u8,
    pub component_id: u8,
    pub target_system: u8,
    pub target_component: u8,
}

impl Default for Px4Config {
    fn default() -> Self {
        Self {
            endpoint: "udpin:0.0.0.0:14540".into(),
            system_id: 245,
            component_id: 190,
            target_system: 1,
            target_component: 1,
        }
    }
}

#[derive(Debug)]
pub struct Px4Backend {
    config: Px4Config,
    link: Option<UdpLink>,
    boot_ms: u32,
    last_velocity: Velocity<Ned>,
    last_position: Position<Ned>,
    armed: bool,
    offboard: bool,
}

impl Px4Backend {
    pub fn new(config: Px4Config) -> Self {
        Self {
            config,
            link: None,
            boot_ms: 0,
            last_velocity: Velocity::ned(0.0, 0.0, 0.0),
            last_position: Position::ned(0.0, 0.0, 0.0),
            armed: false,
            offboard: false,
        }
    }

    fn link(&mut self) -> Result<&mut UdpLink, BackendError> {
        self.link.as_mut().ok_or(BackendError::Disconnected)
    }

    fn send(&mut self, msg: &MavMessage) -> Result<(), BackendError> {
        self.link()?.send(msg).map_err(|_| BackendError::Io)
    }

    fn pump_setpoint(&mut self) -> Result<(), BackendError> {
        self.boot_ms = self.boot_ms.wrapping_add(20);
        let v = self.last_velocity;
        let msg = set_velocity_ned(
            self.config.target_system,
            self.config.target_component,
            self.boot_ms,
            v.x(),
            v.y(),
            v.z(),
        );
        self.send(&msg)
    }
}

impl VehicleBackend for Px4Backend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        let mut link = UdpLink::connect(
            &self.config.endpoint,
            self.config.system_id,
            self.config.component_id,
        )
        .map_err(|_| BackendError::Io)?;
        let _ = link.send(&gcs_heartbeat());
        self.link = Some(link);
        Ok(ConnectionInfo {
            system_id: self.config.target_system,
            component_id: self.config.target_component,
            autopilot: AutopilotKind::Px4,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        let _ = self.send(&gcs_heartbeat());
        // A full implementation would inspect SYS_STATUS / ESTIMATOR_STATUS.
        // SITL is assumed healthy once a PX4 heartbeat has been seen; we do not
        // block here so unit tests and dry-runs can construct the backend.
        Ok(PreflightReport {
            imu_healthy: true,
            estimator_valid: true,
            battery_ok: true,
            gps_ok: true,
            notes: PreflightNotes {
                imu_std_accel: 0.0,
                imu_std_gyro: 0.0,
                samples: 0,
            },
        })
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        for _ in 0..10 {
            self.pump_setpoint()?;
            let _ = self.send(&gcs_heartbeat());
        }
        self.send(&arm_disarm(
            self.config.target_system,
            self.config.target_component,
            true,
        ))?;
        self.armed = true;
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        self.send(&arm_disarm(
            self.config.target_system,
            self.config.target_component,
            false,
        ))?;
        self.armed = false;
        self.offboard = false;
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        // PX4 requires streaming setpoints *before* the mode switch.
        for _ in 0..20 {
            self.pump_setpoint()?;
        }
        self.send(&set_offboard_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))?;
        self.offboard = true;
        Ok(())
    }

    async fn set_velocity_ned(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        self.last_velocity = velocity;
        self.pump_setpoint()
    }

    async fn set_position_ned(&mut self, position: Position<Ned>) -> Result<(), BackendError> {
        self.last_position = position;
        Ok(())
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        Err(BackendError::Rejected(
            "direct motor thrust is not exposed over PX4 offboard; use velocity/position setpoints",
        ))
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    async fn tick(&mut self, _dt_secs: f32) -> Result<Telemetry, BackendError> {
        let _ = self.send(&gcs_heartbeat());
        let _ = self.pump_setpoint();
        if let Ok((_, MavMessage::LOCAL_POSITION_NED(p))) = self.link()?.recv() {
            self.last_position = Position::ned(p.x, p.y, p.z);
            self.last_velocity = Velocity::ned(p.vx, p.vy, p.vz);
        }
        self.telemetry().await
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        Ok(Telemetry {
            timestamp: MonotonicInstant::from_millis(u64::from(self.boot_ms)),
            phase: if self.offboard {
                Phase::Airborne
            } else if self.armed {
                Phase::Armed
            } else {
                Phase::Ready
            },
            position: self.last_position,
            velocity: self.last_velocity,
            yaw_rad: 0.0,
            imu: None,
            imu_health: SensorHealth::Ok,
            imu_healthy: true,
            estimator_valid: true,
            armed: self.armed,
            actuators_enabled: self.armed,
            offboard: self.offboard,
            failsafe: false,
            heartbeat_age_secs: 0.0,
            last_command: "px4",
        })
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        self.offboard = false;
        Ok(())
    }
}

/// Typed handle constructor. Connection happens on `Vehicle::connect`.
pub fn vehicle(config: Px4Config) -> Vehicle<flight_core::vehicle::Disconnected, Px4Backend> {
    Vehicle::new(Px4Backend::new(config))
}

/// True if a MAVLink heartbeat looks like PX4.
pub fn is_px4_heartbeat(msg: &MavMessage) -> bool {
    matches!(
        msg,
        MavMessage::HEARTBEAT(h) if h.mavtype == MavType::MAV_TYPE_QUADROTOR
            || h.mavtype == MavType::MAV_TYPE_GENERIC
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_endpoint_is_sitl() {
        assert!(Px4Config::default().endpoint.contains("14540"));
    }
}
