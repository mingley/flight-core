//! Revocable actuation authority.
//!
//! A Rust typestate such as `Vehicle<Offboard>` is **evidence**, not a
//! permanent fact about the physical vehicle. PX4, the verified world, or a
//! sibling handle can failsafe while this process still holds the old type.
//!
//! [`ActuationPermit`] is the missing half: non-`Clone` authority bound to one
//! vehicle, one safety epoch, and an optional lease. The live backend epoch is
//! the hardware/plant boundary. A permit that does not match is memory, not
//! authority.

use crate::time::{Duration, MonotonicInstant};
use core::fmt;

/// Stable identity for one physical (or simulated) vehicle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VehicleId(u8);

impl VehicleId {
    pub const fn from_raw(id: u8) -> Self {
        Self(id)
    }

    pub const fn raw(self) -> u8 {
        self.0
    }

    /// Deterministic id from a body name (`"drone"`, `"rover"`, …).
    pub fn from_name(id: &str) -> Self {
        let mut h: u8 = 0;
        for b in id.as_bytes() {
            h = h.wrapping_mul(31).wrapping_add(*b);
        }
        Self(h)
    }
}

/// Monotonic revocation counter. Not packed into [`crate::safety::SafetyState`]
/// (that word stays 16-bit for exhaustive Kani).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SafetyEpoch(pub u32);

impl SafetyEpoch {
    pub const ZERO: Self = Self(0);

    pub const fn raw(self) -> u32 {
        self.0
    }

    pub const fn saturating_next(self) -> Self {
        Self(self.0.saturating_add(1))
    }
}

/// Why a permit has no authority at the hardware/backend boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthorityReject {
    Missing,
    StaleEpoch,
    Expired,
    WrongVehicle,
    StaleHeartbeat,
}

impl AuthorityReject {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Missing => "missing_permit",
            Self::StaleEpoch => "stale_epoch",
            Self::Expired => "permit_expired",
            Self::WrongVehicle => "wrong_vehicle",
            Self::StaleHeartbeat => "stale_heartbeat",
        }
    }
}

impl fmt::Display for AuthorityReject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Evidence of actuation authority. Not [`Clone`]: copying it would duplicate
/// a lease the type system is trying to keep unique.
#[derive(Debug)]
pub struct ActuationPermit {
    vehicle: VehicleId,
    epoch: SafetyEpoch,
    issued_at: MonotonicInstant,
    max_age: Option<Duration>,
}

impl ActuationPermit {
    pub const fn unbounded(
        vehicle: VehicleId,
        epoch: SafetyEpoch,
        issued_at: MonotonicInstant,
    ) -> Self {
        Self {
            vehicle,
            epoch,
            issued_at,
            max_age: None,
        }
    }

    pub const fn bounded(
        vehicle: VehicleId,
        epoch: SafetyEpoch,
        issued_at: MonotonicInstant,
        max_age: Duration,
    ) -> Self {
        Self {
            vehicle,
            epoch,
            issued_at,
            max_age: Some(max_age),
        }
    }

    pub const fn vehicle(&self) -> VehicleId {
        self.vehicle
    }

    pub const fn epoch(&self) -> SafetyEpoch {
        self.epoch
    }

    pub const fn issued_at(&self) -> MonotonicInstant {
        self.issued_at
    }

    pub const fn max_age(&self) -> Option<Duration> {
        self.max_age
    }

    /// Compile-time-shaped check against live plant/backend reality.
    pub fn check(
        &self,
        live_epoch: SafetyEpoch,
        live_vehicle: VehicleId,
        now: MonotonicInstant,
    ) -> Result<(), AuthorityReject> {
        if self.vehicle != live_vehicle {
            return Err(AuthorityReject::WrongVehicle);
        }
        if self.epoch != live_epoch {
            return Err(AuthorityReject::StaleEpoch);
        }
        if let Some(max) = self.max_age {
            if now.saturating_duration_since(self.issued_at) >= max {
                return Err(AuthorityReject::Expired);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matching_epoch_is_live() {
        let p = ActuationPermit::unbounded(
            VehicleId::from_raw(1),
            SafetyEpoch(3),
            MonotonicInstant::ZERO,
        );
        assert!(p
            .check(
                SafetyEpoch(3),
                VehicleId::from_raw(1),
                MonotonicInstant::ZERO
            )
            .is_ok());
    }

    #[test]
    fn stale_epoch_has_no_authority() {
        let p = ActuationPermit::unbounded(
            VehicleId::from_raw(1),
            SafetyEpoch(3),
            MonotonicInstant::ZERO,
        );
        assert_eq!(
            p.check(
                SafetyEpoch(4),
                VehicleId::from_raw(1),
                MonotonicInstant::ZERO
            ),
            Err(AuthorityReject::StaleEpoch)
        );
    }

    #[test]
    fn expired_lease_has_no_authority() {
        let p = ActuationPermit::bounded(
            VehicleId::from_raw(1),
            SafetyEpoch(0),
            MonotonicInstant::ZERO,
            Duration::from_millis(20),
        );
        assert_eq!(
            p.check(
                SafetyEpoch(0),
                VehicleId::from_raw(1),
                MonotonicInstant::from_millis(20)
            ),
            Err(AuthorityReject::Expired)
        );
    }

    #[test]
    fn wrong_vehicle_is_rejected() {
        let p = ActuationPermit::unbounded(
            VehicleId::from_raw(1),
            SafetyEpoch(0),
            MonotonicInstant::ZERO,
        );
        assert_eq!(
            p.check(
                SafetyEpoch(0),
                VehicleId::from_raw(2),
                MonotonicInstant::ZERO
            ),
            Err(AuthorityReject::WrongVehicle)
        );
    }

    #[test]
    fn name_ids_are_stable_for_known_bodies() {
        assert_ne!(VehicleId::from_name("drone"), VehicleId::from_name("rover"));
        assert_eq!(VehicleId::from_name("drone"), VehicleId::from_name("drone"));
    }
}
