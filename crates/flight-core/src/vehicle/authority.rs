//! Shared permit issue/check used by aerial, ground, and marine typestate.

use super::backend::VehicleBackend;
use crate::contracts::{ActuationPermit, AuthorityReject, SafetyEpoch, VehicleId};
use crate::time::Duration;

pub(super) fn issue<B: VehicleBackend>(backend: &B) -> ActuationPermit {
    ActuationPermit::unbounded(
        VehicleId::from_raw(backend.authority_vehicle_id()),
        SafetyEpoch(backend.authority_epoch()),
        backend.authority_now(),
    )
}

pub(super) fn issue_bounded<B: VehicleBackend>(backend: &B, max_age: Duration) -> ActuationPermit {
    ActuationPermit::bounded(
        VehicleId::from_raw(backend.authority_vehicle_id()),
        SafetyEpoch(backend.authority_epoch()),
        backend.authority_now(),
        max_age,
    )
}

pub(super) fn require<B: VehicleBackend>(
    permit: Option<&ActuationPermit>,
    backend: &B,
) -> Result<(), AuthorityReject> {
    let Some(p) = permit else {
        return Err(AuthorityReject::Missing);
    };
    p.check(
        SafetyEpoch(backend.authority_epoch()),
        VehicleId::from_raw(backend.authority_vehicle_id()),
        backend.authority_now(),
    )
}
