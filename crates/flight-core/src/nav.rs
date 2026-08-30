//! Allocation-free attitude estimator.
//!
//! Complementary (Mahony-style) filter, fixed-size state, no panics, no `unsafe`.
//! Intended as the start of a trusted `no_std` navigation core — not a full ESKF.
//!
//! Not wired into `robot-world::World::try_step`. The plant quaternion is
//! physics truth (`mech::quat_integrate` / `unit_attitude`). Kernel
//! `estimator_valid` is a safety bit, not [`ComplementaryAttitude::is_valid`].
//! A companion navigation loop may post [`crate::safety::Event::EstimatorInvalid`]
//! when [`imu_trips_estimator`] is true. Warm-up (fewer than 8 good samples)
//! is not a trip.

use crate::frames::Body;
use crate::sensors::ImuSample;
use crate::units::RadianPerSecond;
use crate::vector::{Acceleration, AngularVelocity};

const KP: f32 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct ComplementaryAttitude {
    /// Body-to-NED quaternion `(w, x, y, z)`.
    q: [f32; 4],
    samples: u32,
    valid: bool,
}

impl Default for ComplementaryAttitude {
    fn default() -> Self {
        Self::new()
    }
}

impl ComplementaryAttitude {
    pub const fn new() -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            samples: 0,
            valid: false,
        }
    }

    pub const fn quaternion(self) -> [f32; 4] {
        self.q
    }

    pub const fn is_valid(self) -> bool {
        self.valid
    }

    pub const fn sample_count(self) -> u32 {
        self.samples
    }

    /// Clear the validity bit without writing a quaternion.
    ///
    /// Used when the IMU sample is unusable (`SensorHealth::Invalid`, NaN)
    /// so callers can trip kernel `estimator_valid` without touching plant pose.
    pub fn invalidate(&mut self) {
        self.valid = false;
    }

    /// Integrate gyro and tilt-correct with accelerometer specific force.
    ///
    /// Returns `false` if the sample is unusable (non-finite or bad `dt`).
    /// Unusable samples also clear [`Self::is_valid`]. Never panics.
    ///
    /// A usable sample that has not yet reached eight ticks also returns
    /// `false`. That warm-up is **not** [`imu_trips_estimator`].
    pub fn update(
        &mut self,
        gyro: AngularVelocity<RadianPerSecond, Body>,
        accel: Acceleration<Body>,
        dt: f32,
    ) -> bool {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            self.valid = false;
            return false;
        }
        if !gyro.is_finite() || !accel.is_finite() {
            self.valid = false;
            return false;
        }

        let mut gx = gyro.x();
        let mut gy = gyro.y();
        let mut gz = gyro.z();

        let an = accel.norm();
        if an > 1.0 && an < 30.0 {
            let ax = accel.x() / an;
            let ay = accel.y() / an;
            let az = accel.z() / an;

            let [qw, qx, qy, qz] = self.q;
            // Estimated gravity in body from current quaternion (NED z-down).
            let vx = 2.0 * (qx * qz - qw * qy);
            let vy = 2.0 * (qw * qx + qy * qz);
            let vz = qw * qw - qx * qx - qy * qy + qz * qz;

            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            gx += KP * ex;
            gy += KP * ey;
            gz += KP * ez;
        }

        let [qw, qx, qy, qz] = self.q;
        let half = 0.5 * dt;
        let nqw = qw + (-qx * gx - qy * gy - qz * gz) * half;
        let nqx = qx + (qw * gx + qy * gz - qz * gy) * half;
        let nqy = qy + (qw * gy - qx * gz + qz * gx) * half;
        let nqz = qz + (qw * gz + qx * gy - qy * gx) * half;
        self.q = normalize4([nqw, nqx, nqy, nqz]);

        self.samples = self.samples.saturating_add(1);
        self.valid = self.samples >= 8 && self.q.iter().all(|c| c.is_finite());
        self.valid
    }

    /// Yaw (heading) in radians, NED.
    pub fn yaw(self) -> f32 {
        let [w, x, y, z] = self.q;
        libm_atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    }
}

fn normalize4(mut q: [f32; 4]) -> [f32; 4] {
    let n = crate::math::sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if n < 1e-12 || !n.is_finite() {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let inv = 1.0 / n;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
    q
}

fn libm_atan2(y: f32, x: f32) -> f32 {
    // core doesn't expose atan2 on all no_std targets; use a tiny approximation
    // via atan(y/x) piecewise so we stay dependency-free.
    if !y.is_finite() || !x.is_finite() {
        return 0.0;
    }
    if x > 0.0 {
        atan_approx(y / x)
    } else if x < 0.0 && y >= 0.0 {
        atan_approx(y / x) + core::f32::consts::PI
    } else if x < 0.0 && y < 0.0 {
        atan_approx(y / x) - core::f32::consts::PI
    } else if x == 0.0 && y > 0.0 {
        core::f32::consts::FRAC_PI_2
    } else if x == 0.0 && y < 0.0 {
        -core::f32::consts::FRAC_PI_2
    } else {
        0.0
    }
}

fn atan_approx(z: f32) -> f32 {
    // minimax-ish rational approximation on [-1, 1], with reciprocal identity.
    let (z, flip) = if z.abs() > 1.0 {
        (1.0 / z, true)
    } else {
        (z, false)
    };
    let z2 = z * z;
    let a = z * (0.999213 + z2 * (-0.321181 + z2 * 0.146276));
    if flip {
        if z.is_sign_negative() {
            -core::f32::consts::FRAC_PI_2 - a
        } else {
            core::f32::consts::FRAC_PI_2 - a
        }
    } else {
        a
    }
}

/// Reject non-physical covariance diagonals (NaN, Inf, negative).
pub fn covariance_diag_ok(diag: &[f32]) -> bool {
    diag.iter().all(|v| v.is_finite() && *v >= 0.0)
}

/// Whether this IMU sample must clear kernel `estimator_valid`.
///
/// Distinct from [`ComplementaryAttitude::is_valid`]: the filter needs eight
/// good samples before it reports valid, and that warm-up must not latch
/// failsafe. A trip is a non-finite sample, invalid sensor health, or a
/// non-physical `dt`.
pub fn imu_trips_estimator(sample: &ImuSample<Body>, dt: f32) -> bool {
    !(dt.is_finite() && dt > 0.0 && dt < 1.0 && sample.is_usable())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::Vector3;

    #[test]
    fn still_imu_stays_level() {
        let mut att = ComplementaryAttitude::new();
        let gyro = AngularVelocity::body_rad(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        for _ in 0..50 {
            att.update(gyro, accel, 0.01);
        }
        assert!(att.is_valid());
        let [w, x, y, z] = att.quaternion();
        assert!(w.abs() > 0.9, "{:?}", att.quaternion());
        assert!(x.abs() < 0.1 && y.abs() < 0.1 && z.abs() < 0.1);
    }

    #[test]
    fn rejects_nan_samples() {
        let mut att = ComplementaryAttitude::new();
        let gyro = AngularVelocity::body_rad(f32::NAN, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        assert!(!att.update(gyro, accel, 0.01));
        assert!(!att.is_valid());
    }

    #[test]
    fn unusable_sample_clears_a_valid_filter() {
        let mut att = ComplementaryAttitude::new();
        let gyro = AngularVelocity::body_rad(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        for _ in 0..50 {
            att.update(gyro, accel, 0.01);
        }
        assert!(att.is_valid());
        let q = att.quaternion();
        assert!(!att.update(AngularVelocity::body_rad(f32::NAN, 0.0, 0.0), accel, 0.01));
        assert!(!att.is_valid());
        assert_eq!(
            att.quaternion(),
            q,
            "invalidate must not write a new attitude"
        );
    }

    #[test]
    fn imu_trips_estimator_ignores_warmup() {
        use crate::sensors::{ImuSample, SensorHealth};
        use crate::time::MonotonicInstant;

        let mut att = ComplementaryAttitude::new();
        let sample = ImuSample {
            timestamp: MonotonicInstant::from_millis(0),
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: AngularVelocity::body_rad(0.0, 0.0, 0.0),
            covariance: None,
            temperature: None,
            status: SensorHealth::Ok,
            sequence: 0,
        };
        assert!(sample.is_usable());
        assert!(!imu_trips_estimator(&sample, 0.01));
        assert!(!att.update(sample.gyro, sample.accel, 0.01));
        assert!(!att.is_valid());

        let mut dead = sample;
        dead.status = SensorHealth::Invalid;
        assert!(imu_trips_estimator(&dead, 0.01));
        let mut nan = sample;
        nan.gyro = AngularVelocity::body_rad(f32::NAN, 0.0, 0.0);
        assert!(imu_trips_estimator(&nan, 0.01));
        assert!(imu_trips_estimator(&sample, f32::NAN));
    }

    #[test]
    fn covariance_validation() {
        assert!(covariance_diag_ok(&[0.0, 1.0, 2.0]));
        assert!(!covariance_diag_ok(&[0.0, -1.0, 2.0]));
        assert!(!covariance_diag_ok(&[f32::NAN]));
    }
}
