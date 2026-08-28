//! Point-mass + yaw physics in NED. Deterministic, no heap in the step.

use flight_core::frames::{Body, Ned};
use flight_core::units::RadianPerSecond;
use flight_core::vector::{Acceleration, AngularVelocity, Position, Velocity};

/// Gravity in NED (positive down), m/s².
pub const GRAVITY_NED: f32 = 9.80665;

#[derive(Clone, Copy, Debug)]
pub struct Physics {
    pub position_m: [f32; 3],
    pub velocity_mps: [f32; 3],
    pub yaw_rad: f32,
    pub yaw_rate: f32,
    pub mass_kg: f32,
}

impl Physics {
    pub fn grounded(mass_kg: f32) -> Self {
        Self {
            position_m: [0.0, 0.0, 0.0],
            velocity_mps: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            yaw_rate: 0.0,
            mass_kg,
        }
    }

    pub fn position(&self) -> Position<Ned> {
        Position::ned(self.position_m[0], self.position_m[1], self.position_m[2])
    }

    pub fn velocity(&self) -> Velocity<Ned> {
        Velocity::ned(
            self.velocity_mps[0],
            self.velocity_mps[1],
            self.velocity_mps[2],
        )
    }

    pub fn on_ground(&self) -> bool {
        self.position_m[2] >= -0.02 && self.velocity_mps[2] >= -0.05
    }

    /// Integrate net inertial acceleration (NED) and optional yaw rate command.
    pub fn step(&mut self, a_net_ned: [f32; 3], yaw_rate_cmd: f32, dt: f32) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        for ((pos, vel), acc) in self
            .position_m
            .iter_mut()
            .zip(self.velocity_mps.iter_mut())
            .zip(a_net_ned)
        {
            *vel += acc * dt;
            *pos += *vel * dt;
        }
        self.yaw_rate = yaw_rate_cmd;
        self.yaw_rad = wrap_pi(self.yaw_rad + self.yaw_rate * dt);

        if self.position_m[2] > 0.0 {
            self.position_m[2] = 0.0;
            if self.velocity_mps[2] > 0.0 {
                self.velocity_mps[2] = 0.0;
            }
            self.velocity_mps[0] *= 0.7;
            self.velocity_mps[1] *= 0.7;
        }
    }

    /// Specific force in body (yaw-only attitude).
    pub fn body_accel(&self, a_net_ned: [f32; 3]) -> Acceleration<Body> {
        let fx = a_net_ned[0];
        let fy = a_net_ned[1];
        let fz = a_net_ned[2] - GRAVITY_NED;
        let (s, c) = (self.yaw_rad.sin(), self.yaw_rad.cos());
        Acceleration::body(c * fx + s * fy, -s * fx + c * fy, fz)
    }

    pub fn body_gyro(&self) -> AngularVelocity<RadianPerSecond, Body> {
        AngularVelocity::body_rad(0.0, 0.0, self.yaw_rate)
    }
}

fn wrap_pi(a: f32) -> f32 {
    let pi = core::f32::consts::PI;
    let mut x = (a + pi) % (2.0 * pi);
    if x < 0.0 {
        x += 2.0 * pi;
    }
    x - pi
}
