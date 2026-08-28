//! Rust-native companion-computer *interface* for PX4 external flight modes.
//!
//! The official PX4 ROS 2 Interface Library is C++ (Python bindings incomplete).
//! `rclrs` exists but offers no stability guarantee and still has executor,
//! allocation, and ABI issues. This crate defines the API we would expose on
//! top of a healthy `rclrs` — without taking that dependency yet.
//!
//! When `rclrs` is production-ready, a feature-gated implementation can live
//! here without changing `flight-core` vehicle types.

#![deny(unsafe_code)]

use flight_core::frames::Ned;
use flight_core::vector::{Position, Velocity};

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
}
