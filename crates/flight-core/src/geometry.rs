//! Geometry that cannot compose across unrelated frames or quantities.
//!
//! `Transform<A, B> * Transform<B, C>` is legal. `Transform<A, B> * Transform<D, C>`
//! does not compile. [`Point3`] vs [`Displacement`] vs velocity / acceleration /
//! force / torque / [`Orientation`] vs angular velocity stay distinct so a
//! velocity cannot be added to a point, two points cannot be added, and an
//! [`Orientation`] cannot be passed where [`crate::vector::AngularVelocity`] is
//! required.
//!
//! A rigid transform maps a point as `R p + t` and a free vector (velocity,
//! force, …) by rotation only. Copper already has compile-time frame ids
//! (`cu_transform`). This module is the contract-surface geometry for
//! flight-core; see `docs/copper.md` for interop rather than a second robotics
//! runtime.

use crate::frames::Frame;
use crate::units::Meter;
use crate::vector::{Acceleration, Force, Torque, Vector3, Velocity};
use core::marker::PhantomData;
use core::ops::{Add, Mul, Neg, Sub};

pub use crate::vector::Point3;

/// Displacement (free vector) in frame `F`. Distinct from [`Point3`]
/// ([`crate::vector::Position`] is the same type: a pose, not a free vector).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Displacement<F> {
    v: Vector3<Meter, F>,
}

impl<F: Frame> Displacement<F> {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            v: Vector3::new(x, y, z),
        }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub const fn vector(self) -> Vector3<Meter, F> {
        self.v
    }
}

impl<F: Frame> Add for Displacement<F> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.v.x() + rhs.v.x(),
            self.v.y() + rhs.v.y(),
            self.v.z() + rhs.v.z(),
        )
    }
}

impl<F: Frame> Neg for Displacement<F> {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(-self.v.x(), -self.v.y(), -self.v.z())
    }
}

/// Rotation taking coordinates of `From` into `To`.
#[derive(Clone, Copy, Debug)]
pub struct Rotation<From, To> {
    m: [[f32; 3]; 3],
    _from: PhantomData<From>,
    _to: PhantomData<To>,
}

impl<From: Frame, To: Frame> Rotation<From, To> {
    pub const fn from_matrix(m: [[f32; 3]; 3]) -> Self {
        Self {
            m,
            _from: PhantomData,
            _to: PhantomData,
        }
    }

    pub const fn identity() -> Self {
        Self::from_matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    }

    pub const fn matrix(self) -> [[f32; 3]; 3] {
        self.m
    }

    pub fn apply<U>(self, v: Vector3<U, From>) -> Vector3<U, To> {
        let [x, y, z] = v.xyz();
        Vector3::new(
            self.m[0][0] * x + self.m[0][1] * y + self.m[0][2] * z,
            self.m[1][0] * x + self.m[1][1] * y + self.m[1][2] * z,
            self.m[2][0] * x + self.m[2][1] * y + self.m[2][2] * z,
        )
    }

    /// Rotate a point (no translation). Distinct from [`Transform::apply_point`].
    pub fn apply_point(self, p: Point3<From>) -> Point3<To> {
        Point3::from_vector(self.apply(p.vector()))
    }

    pub fn then<C: Frame>(self, next: Rotation<To, C>) -> Rotation<From, C> {
        Rotation::from_matrix(matmul(next.m, self.m))
    }
}

impl<F: Frame> Add<Displacement<F>> for Point3<F> {
    type Output = Self;

    fn add(self, d: Displacement<F>) -> Self {
        Self::new(self.x() + d.v.x(), self.y() + d.v.y(), self.z() + d.v.z())
    }
}

impl<F: Frame> Sub<Displacement<F>> for Point3<F> {
    type Output = Self;

    fn sub(self, d: Displacement<F>) -> Self {
        self + (-d)
    }
}

impl<F: Frame> Sub for Point3<F> {
    type Output = Displacement<F>;

    fn sub(self, rhs: Self) -> Displacement<F> {
        Displacement::new(self.x() - rhs.x(), self.y() - rhs.y(), self.z() - rhs.z())
    }
}

/// Attitude of frame `F` relative to a parent (NED by convention). Distinct
/// from [`crate::vector::AngularVelocity`]: you cannot pass an orientation
/// where a rate is required.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orientation<F> {
    q: [f32; 4],
    _frame: PhantomData<F>,
}

impl<F: Frame> Orientation<F> {
    pub const fn from_xyzw(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self {
            q: [x, y, z, w],
            _frame: PhantomData,
        }
    }

    pub const fn identity() -> Self {
        Self::from_xyzw(0.0, 0.0, 0.0, 1.0)
    }

    pub const fn xyzw(self) -> [f32; 4] {
        self.q
    }
}

/// Rigid transform: coordinates of `From` expressed in `To`.
///
/// A point maps as `R p + t`. A free vector (displacement, velocity, force)
/// maps by rotation only.
#[derive(Clone, Copy, Debug)]
pub struct Transform<From, To> {
    rotation: Rotation<From, To>,
    translation: Displacement<To>,
}

impl<From: Frame, To: Frame> Transform<From, To> {
    pub const fn new(rotation: Rotation<From, To>, translation: Displacement<To>) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            translation: Displacement::zero(),
        }
    }

    pub const fn rotation(self) -> Rotation<From, To> {
        self.rotation
    }

    pub const fn translation(self) -> Displacement<To> {
        self.translation
    }

    /// Transform a point: `R p + t`.
    pub fn apply_point(self, p: Point3<From>) -> Point3<To> {
        let r = self.rotation.apply(p.vector());
        Point3::new(
            r.x() + self.translation.v.x(),
            r.y() + self.translation.v.y(),
            r.z() + self.translation.v.z(),
        )
    }

    /// Transform a free vector (rotation only).
    pub fn apply_displacement(self, d: Displacement<From>) -> Displacement<To> {
        Displacement {
            v: self.rotation.apply(d.v),
        }
    }

    /// Transform a velocity (rotation only). Distinct from [`Self::apply_point`].
    pub fn apply_velocity(self, v: Velocity<From>) -> Velocity<To> {
        self.rotation.apply(v)
    }

    /// Transform an acceleration (rotation only).
    pub fn apply_acceleration(self, a: Acceleration<From>) -> Acceleration<To> {
        self.rotation.apply(a)
    }

    /// Transform a force (rotation only). Distinct from [`Self::apply_torque`].
    pub fn apply_force(self, f: Force<From>) -> Force<To> {
        self.rotation.apply(f)
    }

    /// Transform a torque (rotation only).
    pub fn apply_torque(self, tau: Torque<From>) -> Torque<To> {
        self.rotation.apply(tau)
    }

    /// `self` then `next`: `From → To → C`.
    pub fn then<C: Frame>(self, next: Transform<To, C>) -> Transform<From, C> {
        let rotation = self.rotation.then(next.rotation);
        let rotated = next.rotation.apply(self.translation.v);
        let translation = Displacement::new(
            rotated.x() + next.translation.v.x(),
            rotated.y() + next.translation.v.y(),
            rotated.z() + next.translation.v.z(),
        );
        Transform {
            rotation,
            translation,
        }
    }
}

impl<A: Frame, B: Frame, C: Frame> Mul<Transform<B, C>> for Transform<A, B> {
    type Output = Transform<A, C>;

    fn mul(self, rhs: Transform<B, C>) -> Self::Output {
        self.then(rhs)
    }
}

/// 3×3 covariance of a typed quantity (velocity, position, …).
#[derive(Clone, Copy, Debug)]
pub struct Covariance<T> {
    data: [f32; 9],
    _ty: PhantomData<T>,
}

impl<T> Covariance<T> {
    pub const fn from_diag(x: f32, y: f32, z: f32) -> Self {
        Self {
            data: [x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z],
            _ty: PhantomData,
        }
    }

    pub const fn data(self) -> [f32; 9] {
        self.data
    }
}

const fn matmul(a: [[f32; 3]; 3], b: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut o = [[0.0; 3]; 3];
    let mut i = 0;
    while i < 3 {
        let mut j = 0;
        while j < 3 {
            o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            j += 1;
        }
        i += 1;
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{Body, Ned};
    use crate::vector::{Acceleration, Force, Position, Torque, Velocity};

    #[test]
    fn identity_then_is_identity() {
        let t: Transform<Ned, Body> = Transform::identity();
        let u: Transform<Body, Ned> = Transform::identity();
        let w = t.then(u);
        let p = w
            .rotation()
            .apply_point(Position::<Ned>::ned(1.0, 2.0, 3.0));
        assert!((p.x() - 1.0).abs() < 1e-6);
        assert!((p.y() - 2.0).abs() < 1e-6);
        assert!((p.z() - 3.0).abs() < 1e-6);
    }

    #[test]
    fn covariance_is_tagged_by_quantity() {
        let _c = Covariance::<Velocity<Ned>>::from_diag(0.1, 0.1, 0.2);
        let _d = Covariance::<Position<Ned>>::from_diag(0.4, 0.4, 0.8);
    }

    #[test]
    fn point_plus_displacement_is_a_point() {
        let p = Position::<Ned>::ned(1.0, 2.0, 3.0);
        let d = Displacement::<Ned>::new(0.5, 0.0, -1.0);
        let q = p + d;
        assert!((q.x() - 1.5).abs() < 1e-6);
        assert!((q.y() - 2.0).abs() < 1e-6);
        assert!((q.z() - 2.0).abs() < 1e-6);
        let back = q - d;
        assert!((back.x() - 1.0).abs() < 1e-6);
        let delta = q - p;
        assert!((delta.vector().x() - 0.5).abs() < 1e-6);
        assert!((delta.vector().z() + 1.0).abs() < 1e-6);
    }

    #[test]
    fn apply_point_adds_translation_velocity_does_not() {
        let t = Transform::<Ned, Body>::new(Rotation::identity(), Displacement::new(1.0, 2.0, 3.0));
        let p = t.apply_point(Point3::<Ned>::origin());
        assert!((p.x() - 1.0).abs() < 1e-6);
        assert!((p.y() - 2.0).abs() < 1e-6);
        assert!((p.z() - 3.0).abs() < 1e-6);
        let d = t.apply_displacement(Displacement::<Ned>::new(4.0, 0.0, 0.0));
        assert!((d.vector().x() - 4.0).abs() < 1e-6);
        let v = t.apply_velocity(Velocity::<Ned>::ned(4.0, 0.0, 0.0));
        assert!((v.x() - 4.0).abs() < 1e-6);
        let f = t.apply_force(Force::<Ned>::new(0.5, 0.0, 0.0));
        assert!((f.x() - 0.5).abs() < 1e-6);
        let tau = t.apply_torque(Torque::<Ned>::new(0.1, 0.0, 0.0));
        assert!((tau.x() - 0.1).abs() < 1e-6);
        let a = t.apply_acceleration(Acceleration::<Ned>::ned(0.0, 0.0, 9.81));
        assert!((a.z() - 9.81).abs() < 1e-6);
        let o = Orientation::<Ned>::identity();
        assert_eq!(o.xyzw()[3], 1.0);
    }
}
