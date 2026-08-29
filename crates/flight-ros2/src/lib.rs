//! Rust-native companion-computer interface for PX4 external flight modes.
//!
//! The official PX4 ROS 2 Interface Library is C++. This crate is the same
//! idea in `flight-core` types:
//!
//! - [`ExternalFlightMode`] / [`VelocityMode`] produce an [`OffboardSetpoint`]
//! - [`plant`] applies those setpoints to a verified [`flight_sim::WorldSession`]
//!   ([`plant::FleetPlant`] grants drone / rover / skiff / surveyor on coastal
//!   and harbor, air+ground inland, air+hulls on open water, then one step;
//!   [`plant::apply_failsafe`] / [`plant::apply_estop`] / [`plant::apply_marine_failsafe`]
//!   walk the same attach trips; [`plant::apply_disarm`] / [`plant::apply_recover_ready`] /
//!   [`plant::apply_reset`] / [`plant::apply_recover`] and [`plant::FleetPlant::recover_safety`]
//!   walk the matching recover / disarm / reset attach helpers;
//!   [`plant::apply_land`] / [`plant::apply_touchdown`] / [`plant::apply_park`] /
//!   [`plant::apply_dock`] and [`plant::FleetPlant::return_all`] walk the home
//!   return after a grant;
//!   [`plant::apply_airborne`] / [`plant::apply_station`] / [`plant::apply_resume`]
//!   and [`plant::FleetPlant::airborne`] / [`plant::FleetPlant::station_all`] /
//!   [`plant::FleetPlant::resume_all`] / [`plant::FleetPlant::dock_all`] /
//!   [`plant::FleetPlant::park_all`] / [`plant::apply_hold`] /
//!   [`plant::FleetPlant::hold`] walk climb-complete, hull station, hull dock,
//!   rover halt, and a drone NED pose hold through `attach_hold`)
//! - [`px4`] serializes that setpoint as ROS 2 CDR `px4_msgs` (NED, `NaN` unused)
//! - `ned_velocity_to_ros_twist_linear` maps NED → ENU for `geometry_msgs/Twist`
//!
//! Enable `--features rclrs` (ROS 2 Jazzy sourced) for a production `OffboardNode`
//! that publishes `geometry_msgs/msg/Twist` and `PlantNode` / `FleetPlantNode`
//! that subscribe and step the verified catalog plant.

#![deny(unsafe_code)]

use flight_core::frames::Ned;
use flight_core::vector::{Position, Velocity};

pub mod plant;
pub mod px4;

#[cfg(feature = "rclrs")]
mod geometry;
#[cfg(feature = "rclrs")]
pub mod node;

/// A PX4 external mode implemented on a companion computer.
pub trait ExternalFlightMode {
    type Setpoint: Send;

    fn name(&self) -> &'static str;
    fn on_activate(&mut self);
    fn on_deactivate(&mut self);
    fn update(&mut self, dt_secs: f32) -> Self::Setpoint;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OffboardSetpoint {
    pub velocity_ned: Option<Velocity<Ned>>,
    pub position_ned: Option<Position<Ned>>,
    pub yaw_rad: Option<f32>,
}

/// Example mode: hold a NED velocity. Mirrors the typed `Vehicle::set_velocity` path.
pub struct VelocityMode {
    pub velocity: Velocity<Ned>,
    active: bool,
}

impl VelocityMode {
    pub fn new(velocity: Velocity<Ned>) -> Self {
        Self {
            velocity,
            active: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }
}

impl ExternalFlightMode for VelocityMode {
    type Setpoint = OffboardSetpoint;

    fn name(&self) -> &'static str {
        "flight_core_velocity"
    }

    fn on_activate(&mut self) {
        self.active = true;
    }

    fn on_deactivate(&mut self) {
        self.active = false;
    }

    fn update(&mut self, _dt_secs: f32) -> Self::Setpoint {
        OffboardSetpoint {
            velocity_ned: Some(self.velocity),
            position_ned: None,
            yaw_rad: None,
        }
    }
}

/// REP-103 Twist linear part: east, north, up (metres / second).
pub fn ned_velocity_to_ros_twist_linear(v: Velocity<Ned>) -> [f64; 3] {
    let enu = v.to_enu();
    [f64::from(enu.x()), f64::from(enu.y()), f64::from(enu.z())]
}

/// Inverse of [`ned_velocity_to_ros_twist_linear`].
pub fn ros_twist_linear_to_ned(linear: [f64; 3]) -> Velocity<Ned> {
    Velocity::<flight_core::frames::Enu>::new(linear[0] as f32, linear[1] as f32, linear[2] as f32)
        .to_ned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    #[test]
    fn velocity_mode_emits_ned() {
        let mut m = VelocityMode::new(Velocity::<Ned>::ned(1.0, 0.0, 0.0));
        m.on_activate();
        let sp = m.update(0.02);
        assert_eq!(sp.velocity_ned.unwrap().x(), 1.0);
        assert!(m.is_active());
        m.on_deactivate();
        assert!(!m.is_active());
    }

    #[test]
    fn ned_east_is_ros_twist_x() {
        let v = Velocity::<Ned>::ned(0.0, 1.5, -0.25);
        let lin = ned_velocity_to_ros_twist_linear(v);
        assert!((lin[0] - 1.5).abs() < 1e-6);
        assert!((lin[1] - 0.0).abs() < 1e-6);
        assert!((lin[2] - 0.25).abs() < 1e-6);
        let back = ros_twist_linear_to_ned(lin);
        assert!((back.x() - v.x()).abs() < 1e-5);
        assert!((back.y() - v.y()).abs() < 1e-5);
        assert!((back.z() - v.z()).abs() < 1e-5);
    }
}
