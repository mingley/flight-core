//! NED waypoints and paths.
//!
//! These types are data. Execution is a sequence of legal attach +
//! `set_position` / `set_velocity` / hold / drive / thrust. There is no
//! kernel event that consumes a path.

use crate::frames::Ned;
use crate::vector::Position;

/// One NED pose target. Meters, z-down.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub position: Position<Ned>,
}

impl Waypoint {
    pub const fn ned(n: f32, e: f32, d: f32) -> Self {
        Self {
            position: Position::<Ned>::ned(n, e, d),
        }
    }

    pub fn n(self) -> f32 {
        self.position.x()
    }

    pub fn e(self) -> f32 {
        self.position.y()
    }

    pub fn d(self) -> f32 {
        self.position.z()
    }

    /// Euclidean distance in meters to a NED pose.
    pub fn distance_m(self, n: f32, e: f32, d: f32) -> f32 {
        let dn = self.n() - n;
        let de = self.e() - e;
        let dd = self.d() - d;
        crate::math::sqrtf(dn * dn + de * de + dd * dd)
    }
}

const MAX: usize = 8;

/// Ordered NED path. Capacity is eight waypoints (no allocation).
#[derive(Clone, Copy, Debug)]
pub struct NedPath {
    points: [Waypoint; MAX],
    len: u8,
}

impl NedPath {
    pub const fn empty() -> Self {
        Self {
            points: [Waypoint::ned(0.0, 0.0, 0.0); MAX],
            len: 0,
        }
    }

    pub const fn two(a: Waypoint, b: Waypoint) -> Self {
        let mut path = Self::empty();
        path.points[0] = a;
        path.points[1] = b;
        path.len = 2;
        path
    }

    pub fn push(&mut self, wp: Waypoint) -> bool {
        let i = self.len as usize;
        if i >= MAX {
            return false;
        }
        self.points[i] = wp;
        self.len += 1;
        true
    }

    pub fn len(self) -> usize {
        self.len as usize
    }

    pub fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn get(self, i: usize) -> Option<Waypoint> {
        if i < self.len as usize {
            Some(self.points[i])
        } else {
            None
        }
    }

    pub fn waypoints(&self) -> &[Waypoint] {
        &self.points[..self.len as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_point_path_is_ned_meters() {
        let path = NedPath::two(
            Waypoint::ned(10.0, 0.0, -2.0),
            Waypoint::ned(10.0, 2.0, -2.0),
        );
        assert_eq!(path.len(), 2);
        let a = path.get(0).unwrap();
        let b = path.get(1).unwrap();
        assert_eq!(a.n(), 10.0);
        assert_eq!(a.e(), 0.0);
        assert_eq!(a.d(), -2.0);
        assert!(b.distance_m(10.0, 0.0, -2.0) > 1.9);
        assert!(b.distance_m(10.0, 2.0, -2.0) < 1e-6);
        assert!(path.get(2).is_none());
    }

    #[test]
    fn path_rejects_a_ninth_waypoint() {
        let mut path = NedPath::empty();
        for i in 0..8 {
            assert!(path.push(Waypoint::ned(i as f32, 0.0, 0.0)));
        }
        assert!(!path.push(Waypoint::ned(8.0, 0.0, 0.0)));
        assert_eq!(path.len(), 8);
    }
}
