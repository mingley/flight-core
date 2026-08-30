//! Frame- and unit-tagged 3-vectors, and pose points that are not free vectors.
//!
//! [`Position`] is a [`Point3`]: two poses cannot be added even in one frame
//! (`tests/ui/position_plus_position.rs`). Mixed frames still fail
//! (`tests/ui/mix_frames.rs`).
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! fn boom(a: Position<Ned>, b: Position<Ned>) {
//!     let _ = a + b;
//! }
//! ```

use crate::frames::{Body, Enu, Frame, Frd, Ned};
use crate::units::{
    DegreePerSecond, Meter, MeterPerSecond, MeterPerSecondSquared, Newton, NewtonMeter,
    RadianPerSecond, Unit,
};
use core::fmt;
use core::marker::PhantomData;
use core::ops::{Add, Div, Mul, Neg, Sub};

/// A 3-vector whose components share a unit and a reference frame.
///
/// Layout is `[x, y, z]` in the named frame (for NED: north, east, down).
#[derive(Clone, Copy, Debug)]
pub struct Vector3<U, F> {
    x: f32,
    y: f32,
    z: f32,
    _unit: PhantomData<U>,
    _frame: PhantomData<F>,
}

impl<U, F> PartialEq for Vector3<U, F> {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }
}

#[cfg(all(feature = "serde", not(creusot)))]
impl<U, F> serde::Serialize for Vector3<U, F> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("Vector3", 3)?;
        s.serialize_field("x", &self.x)?;
        s.serialize_field("y", &self.y)?;
        s.serialize_field("z", &self.z)?;
        s.end()
    }
}

#[cfg(all(feature = "serde", not(creusot)))]
impl<'de, U, F> serde::Deserialize<'de> for Vector3<U, F> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(serde::Deserialize)]
        struct Raw {
            x: f32,
            y: f32,
            z: f32,
        }
        let raw = Raw::deserialize(deserializer)?;
        Ok(Self::new(raw.x, raw.y, raw.z))
    }
}

impl<U, F> Vector3<U, F> {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x,
            y,
            z,
            _unit: PhantomData,
            _frame: PhantomData,
        }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub const fn x(self) -> f32 {
        self.x
    }
    pub const fn y(self) -> f32 {
        self.y
    }
    pub const fn z(self) -> f32 {
        self.z
    }

    pub const fn xyz(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn from_xyz(v: [f32; 3]) -> Self {
        Self::new(v[0], v[1], v[2])
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn norm(self) -> f32 {
        crate::math::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product. Result uses the same unit (caller must ensure that makes sense).
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

#[cfg(not(creusot))]
impl<U: Unit, F: Frame> fmt::Display for Vector3<U, F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{:.3}, {:.3}, {:.3}] {} {}",
            self.x,
            self.y,
            self.z,
            U::NAME,
            F::NAME
        )
    }
}

impl<U, F> Add for Vector3<U, F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<U, F> Sub for Vector3<U, F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl<U, F> Neg for Vector3<U, F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl<U, F> Mul<f32> for Vector3<U, F> {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl<U, F> Div<f32> for Vector3<U, F> {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

/// Point in frame `F`. Distinct from a free [`Vector3<Meter, F>`] displacement.
///
/// `p + d` is a point ([`crate::geometry::Displacement`]). `p - q` is a
/// displacement. `p + q` does not compile (`tests/ui/point_plus_point.rs`,
/// `tests/ui/position_plus_position.rs`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
#[cfg_attr(all(feature = "serde", not(creusot)), serde(transparent))]
pub struct Point3<F> {
    v: Vector3<Meter, F>,
}

impl<F> Point3<F> {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            v: Vector3::new(x, y, z),
        }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub const fn origin() -> Self {
        Self::zero()
    }

    pub const fn from_vector(v: Vector3<Meter, F>) -> Self {
        Self { v }
    }

    pub const fn vector(self) -> Vector3<Meter, F> {
        self.v
    }

    pub const fn x(self) -> f32 {
        self.v.x()
    }

    pub const fn y(self) -> f32 {
        self.v.y()
    }

    pub const fn z(self) -> f32 {
        self.v.z()
    }

    pub const fn xyz(self) -> [f32; 3] {
        self.v.xyz()
    }

    pub fn from_xyz(v: [f32; 3]) -> Self {
        Self::from_vector(Vector3::from_xyz(v))
    }

    pub fn is_finite(self) -> bool {
        self.v.is_finite()
    }
}

#[cfg(not(creusot))]
impl<F: Frame> fmt::Display for Point3<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{:.3}, {:.3}, {:.3}] {} {} (point)",
            self.x(),
            self.y(),
            self.z(),
            Meter::NAME,
            F::NAME
        )
    }
}

/// Pose / setpoint / telemetry sample. Same type as [`Point3`] so two poses
/// cannot be added even in one frame.
pub type Position<F> = Point3<F>;
pub type Velocity<F> = Vector3<MeterPerSecond, F>;
pub type Acceleration<F> = Vector3<MeterPerSecondSquared, F>;
pub type AngularVelocity<U, F> = Vector3<U, F>;
pub type Force<F> = Vector3<Newton, F>;
pub type Torque<F> = Vector3<NewtonMeter, F>;

impl Position<Ned> {
    pub const fn ned(north: f32, east: f32, down: f32) -> Self {
        Self::new(north, east, down)
    }

    pub fn to_enu(self) -> Position<Enu> {
        Position::<Enu>::new(self.y(), self.x(), -self.z())
    }

    /// Height above the local tangent plane (positive up).
    pub fn altitude_agl(self) -> crate::units::Meters {
        crate::units::Qty::new(-self.z())
    }
}

impl Position<Enu> {
    pub const fn enu(east: f32, north: f32, up: f32) -> Self {
        Self::new(east, north, up)
    }

    pub fn to_ned(self) -> Position<Ned> {
        Position::<Ned>::new(self.y(), self.x(), -self.z())
    }
}

impl Velocity<Ned> {
    pub const fn ned(north: f32, east: f32, down: f32) -> Self {
        Self::new(north, east, down)
    }

    pub fn to_enu(self) -> Velocity<Enu> {
        Velocity::<Enu>::new(self.y(), self.x(), -self.z())
    }
}

impl Velocity<Enu> {
    pub fn to_ned(self) -> Velocity<Ned> {
        Velocity::<Ned>::new(self.y(), self.x(), -self.z())
    }
}

impl Acceleration<Ned> {
    pub const fn ned(north: f32, east: f32, down: f32) -> Self {
        Self::new(north, east, down)
    }

    pub fn to_enu(self) -> Acceleration<Enu> {
        Acceleration::<Enu>::new(self.y(), self.x(), -self.z())
    }
}

impl Acceleration<Body> {
    pub const fn body(x: f32, y: f32, z: f32) -> Self {
        Self::new(x, y, z)
    }
}

impl AngularVelocity<RadianPerSecond, Body> {
    pub const fn body_rad(x: f32, y: f32, z: f32) -> Self {
        Self::new(x, y, z)
    }

    pub fn to_degrees(self) -> AngularVelocity<DegreePerSecond, Body> {
        let s = 180.0 / core::f32::consts::PI;
        AngularVelocity::new(self.x() * s, self.y() * s, self.z() * s)
    }
}

impl AngularVelocity<DegreePerSecond, Body> {
    pub const fn body_deg(x: f32, y: f32, z: f32) -> Self {
        Self::new(x, y, z)
    }

    pub fn to_radians(self) -> AngularVelocity<RadianPerSecond, Body> {
        let s = core::f32::consts::PI / 180.0;
        AngularVelocity::new(self.x() * s, self.y() * s, self.z() * s)
    }
}

impl<U: Unit> Vector3<U, Body> {
    pub fn into_frd(self) -> Vector3<U, Frd> {
        Vector3::new(self.x(), self.y(), self.z())
    }
}

impl<U: Unit> Vector3<U, Frd> {
    pub fn into_body(self) -> Vector3<U, Body> {
        Vector3::new(self.x(), self.y(), self.z())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ned_enu_roundtrip() {
        let p = Position::ned(1.0, 2.0, -3.0);
        let q = p.to_enu().to_ned();
        assert_eq!(p, q);
        assert!((p.altitude_agl().get() - 3.0).abs() < 1e-6);
    }

    fn position_is_the_point_type(p: Position<Ned>) -> Point3<Ned> {
        p
    }

    #[test]
    fn position_is_point3() {
        let p = Position::ned(1.0, 2.0, 3.0);
        let q = position_is_the_point_type(p);
        assert_eq!(q.xyz(), [1.0, 2.0, 3.0]);
        assert!(q.is_finite());
        assert_eq!(Point3::<Ned>::origin(), Position::zero());
    }

    #[test]
    fn angular_velocity_conversion() {
        let w = AngularVelocity::<DegreePerSecond, Body>::body_deg(0.0, 0.0, 180.0);
        let r = w.to_radians();
        assert!((r.z() - core::f32::consts::PI).abs() < 1e-5);
    }
}
