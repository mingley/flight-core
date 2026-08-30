//! Geometry that cannot compose across unrelated frames.
//!
//! `Transform<A, B> * Transform<B, C>` is legal. `Transform<A, B> * Transform<D, C>`
//! does not compile. Points, displacements, and free vectors stay distinct so a
//! velocity cannot be added to a point by accident at this layer.
//!
//! Copper already has compile-time frame ids (`cu_transform`). This module is
//! the contract-surface geometry for flight-core; see `docs/copper.md` for
//! interop rather than a second robotics runtime.

use crate::frames::Frame;
use crate::units::Meter;
use crate::vector::Vector3;
use core::marker::PhantomData;
use core::ops::Mul;

/// Displacement (free vector) in frame `F`. Distinct from [`crate::vector::Position`].
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

    pub fn then<C: Frame>(self, next: Rotation<To, C>) -> Rotation<From, C> {
        Rotation::from_matrix(matmul(next.m, self.m))
    }
}

/// Point in frame `F`. Distinct from a free [`Displacement`].
pub type Point3<F> = crate::vector::Position<F>;

/// Rigid transform: coordinates of `From` expressed in `To`.
///
/// `p_to = R * p_from + t`.
#[derive(Clone, Copy, Debug)]
pub struct Transform<From, To> {
    rotation: Rotation<From, To>,
    translation: Vector3<Meter, To>,
}

impl<From: Frame, To: Frame> Transform<From, To> {
    pub const fn new(rotation: Rotation<From, To>, translation: Vector3<Meter, To>) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            translation: Vector3::zero(),
        }
    }

    pub const fn rotation(self) -> Rotation<From, To> {
        self.rotation
    }

    pub const fn translation(self) -> Vector3<Meter, To> {
        self.translation
    }

    /// Transform a point: `R p + t`.
    pub fn apply_point(self, p: crate::vector::Position<From>) -> crate::vector::Position<To> {
        let r = self.rotation.apply(p);
        crate::vector::Position::new(
            r.x() + self.translation.x(),
            r.y() + self.translation.y(),
            r.z() + self.translation.z(),
        )
    }

    /// Transform a free vector (rotation only).
    pub fn apply_displacement(self, d: Displacement<From>) -> Displacement<To> {
        Displacement {
            v: self.rotation.apply(d.v),
        }
    }

    /// `self` then `next`: `From → To → C`.
    pub fn then<C: Frame>(self, next: Transform<To, C>) -> Transform<From, C> {
        let rotation = self.rotation.then(next.rotation);
        let translation = next.rotation.apply(self.translation);
        let translation = Vector3::new(
            translation.x() + next.translation.x(),
            translation.y() + next.translation.y(),
            translation.z() + next.translation.z(),
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
    use crate::vector::{Position, Velocity};

    #[test]
    fn identity_then_is_identity() {
        let t: Transform<Ned, Body> = Transform::identity();
        let u: Transform<Body, Ned> = Transform::identity();
        let w = t.then(u);
        let p = w.rotation().apply(Position::<Ned>::ned(1.0, 2.0, 3.0));
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
    fn displacement_is_not_a_point_alias_in_docs() {
        let d = Displacement::<Ned>::new(1.0, 0.0, 0.0);
        assert!((d.vector().x() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn apply_point_adds_translation() {
        let t = Transform::<Ned, Body>::new(Rotation::identity(), Vector3::new(1.0, 2.0, 3.0));
        let p = t.apply_point(Position::<Ned>::ned(0.0, 0.0, 0.0));
        assert!((p.x() - 1.0).abs() < 1e-6);
        assert!((p.y() - 2.0).abs() < 1e-6);
        assert!((p.z() - 3.0).abs() < 1e-6);
        let d = t.apply_displacement(Displacement::<Ned>::new(4.0, 0.0, 0.0));
        assert!((d.vector().x() - 4.0).abs() < 1e-6);
    }
}
