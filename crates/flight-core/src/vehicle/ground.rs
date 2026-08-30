//! Typestate ground platform: `Parked` cannot command a twist or hold a pose.

use super::backend::{BackendError, NullBackend, Telemetry, VehicleBackend};
use crate::frames::{Body, Ned};
use crate::ground::{self, GroundEvent, GroundPhase, GroundReject, GroundState};
use crate::units::RadianPerSecond;
use crate::vector::{AngularVelocity, Velocity};
use core::fmt;
use core::marker::PhantomData;

/// Rotate a planar body twist (x forward, y right) into NED using heading.
///
/// Heading `0` faces north. Positive yaw is north toward east.
pub fn body_xy_to_ned(yaw_rad: f32, forward: f32, right: f32) -> [f32; 2] {
    let (s, c) = yaw_rad.sin_cos();
    [forward * c - right * s, forward * s + right * c]
}

/// Parked: wheels locked. [`GroundVehicle::set_twist`] does not exist.
#[derive(Clone, Copy, Debug, Default)]
pub struct Parked;

/// Moving: drive commands are legal.
#[derive(Clone, Copy, Debug, Default)]
pub struct Moving;

/// Emergency stop: drive commands do not exist.
#[derive(Clone, Copy, Debug, Default)]
pub struct EStopped;

/// Marker: kernel `EStop` is a consume-self method.
///
/// Parked and Moving chassis can trip. Already-estopped cannot — attach those
/// as [`BackendError::Protocol`].
pub trait CanTripEstop {}
impl CanTripEstop for Parked {}
impl CanTripEstop for Moving {}

/// Ground platform whose methods exist only in legal phases.
#[derive(Debug)]
pub struct GroundVehicle<S, B> {
    backend: B,
    safety: GroundState,
    permit: Option<crate::contracts::ActuationPermit>,
    _state: PhantomData<S>,
}

/// Drive / E-stop error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundError {
    Safety(GroundReject),
    Backend(BackendError),
    StaleAuthority(crate::contracts::AuthorityReject),
}

impl fmt::Display for GroundError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GroundError::Safety(r) => write!(f, "ground safety: {r}"),
            GroundError::Backend(b) => write!(f, "ground backend: {b}"),
            GroundError::StaleAuthority(r) => write!(f, "ground stale authority: {r}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for GroundError {}

impl GroundError {
    /// Collapse a ground error into a [`BackendError`] for session helpers.
    pub fn into_backend(self) -> BackendError {
        match self {
            Self::Backend(b) => b,
            Self::Safety(_) => BackendError::Rejected("ground safety"),
            Self::StaleAuthority(_) => BackendError::Rejected("stale_authority"),
        }
    }
}

impl From<GroundError> for BackendError {
    fn from(e: GroundError) -> Self {
        e.into_backend()
    }
}

/// Which consume-self typestate [`GroundHandle::from_state`] binds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum GroundKind {
    Parked,
    Moving,
    #[cfg_attr(feature = "serde", serde(rename = "estopped"))]
    EStopped,
}

impl GroundKind {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Parked => "parked",
            Self::Moving => "moving",
            Self::EStopped => "estopped",
        }
    }

    pub const fn grants_actuation(self) -> bool {
        matches!(self, Self::Moving)
    }
}

impl fmt::Display for GroundKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Map a live chassis onto the consume-self typestate `attach` uses.
pub fn ground_kind(safety: GroundState) -> GroundKind {
    if safety.estop {
        return GroundKind::EStopped;
    }
    match safety.phase {
        GroundPhase::Parked => GroundKind::Parked,
        GroundPhase::Moving => GroundKind::Moving,
        GroundPhase::EStop => GroundKind::EStopped,
    }
}

impl<S, B> GroundVehicle<S, B> {
    pub fn safety(&self) -> GroundState {
        self.safety
    }

    pub fn phase(&self) -> GroundPhase {
        self.safety.phase
    }

    pub fn backend(&self) -> &B {
        &self.backend
    }

    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    pub fn into_backend(self) -> B {
        self.backend
    }

    fn retarget<T>(self) -> GroundVehicle<T, B> {
        GroundVehicle {
            backend: self.backend,
            safety: self.safety,
            permit: self.permit,
            _state: PhantomData,
        }
    }
}

impl<S, B: VehicleBackend> GroundVehicle<S, B> {
    /// Advance the plant. Illegal from nowhere — every ground phase can observe.
    pub async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, GroundError> {
        self.backend
            .tick(dt_secs)
            .await
            .map_err(GroundError::Backend)
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, GroundError> {
        self.backend.telemetry().await.map_err(GroundError::Backend)
    }

    fn push_ground(&mut self) -> Result<(), GroundError> {
        self.backend
            .sync_ground(self.safety)
            .map_err(GroundError::Backend)
    }
}

impl<S: CanTripEstop, B: VehicleBackend> GroundVehicle<S, B> {
    /// Emergency stop without an async runtime. Drive commands cease to exist.
    pub fn emergency_stop_now(mut self) -> GroundVehicle<EStopped, B> {
        self.safety = ground::ground_step(self.safety, GroundEvent::EStop).unwrap_or(GroundState {
            phase: GroundPhase::EStop,
            drive_enabled: false,
            estop: true,
        });
        let _ = self.backend.trigger_failsafe_now();
        let _ = self.push_ground();
        self.retarget()
    }
}

impl<B: VehicleBackend> GroundVehicle<Parked, B> {
    /// Parked platform with drive disabled.
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            safety: GroundState::parked(),
            permit: None,
            _state: PhantomData,
        }
    }

    /// Enable drive. Compiles only from parked (`tests/ui/moving_release.rs`,
    /// `tests/ui/estopped_release.rs`).
    pub fn enable_drive(mut self) -> Result<GroundVehicle<Moving, B>, GroundError> {
        self.safety =
            ground::ground_step(self.safety, GroundEvent::Release).map_err(GroundError::Safety)?;
        self.push_ground()?;
        let mut v = self.retarget();
        v.permit = Some(super::authority::issue(&v.backend));
        Ok(v)
    }

    /// Emergency stop from parked.
    pub fn emergency_stop(self) -> GroundVehicle<EStopped, B> {
        self.emergency_stop_now()
    }
}

impl GroundVehicle<Parked, NullBackend> {
    pub fn null() -> Self {
        Self::new(NullBackend::default())
    }
}

impl<B: VehicleBackend> GroundVehicle<Moving, B> {
    /// Body-frame twist. Exists only while moving.
    pub async fn set_twist(
        &mut self,
        forward: Velocity<Body>,
        yaw_rate: AngularVelocity<RadianPerSecond, Body>,
    ) -> Result<(), GroundError> {
        let tel = self
            .backend
            .telemetry()
            .await
            .map_err(GroundError::Backend)?;
        let [vn, ve] = body_xy_to_ned(tel.yaw_rad, forward.x(), forward.y());
        self.backend
            .set_yaw_rate(yaw_rate.z())
            .map_err(GroundError::Backend)?;
        self.set_velocity_ned(Velocity::<Ned>::ned(vn, ve, 0.0))
            .await
    }

    /// NED velocity. Exists only while moving.
    pub async fn set_velocity_ned(&mut self, v: Velocity<Ned>) -> Result<(), GroundError> {
        self.set_velocity_ned_now(v)
    }

    /// Same grant as [`Self::set_velocity_ned`] without stepping the plant.
    pub fn set_velocity_ned_now(&mut self, v: Velocity<Ned>) -> Result<(), GroundError> {
        super::authority::require(self.permit.as_ref(), &self.backend)
            .map_err(GroundError::StaleAuthority)?;
        self.safety = ground::ground_step(self.safety, GroundEvent::DriveCommand)
            .map_err(GroundError::Safety)?;
        self.push_ground()?;
        self.backend
            .set_velocity_ned_now(v)
            .map_err(GroundError::Backend)
    }

    /// Hold at the current NED pose. Same kernel grant as a drive command
    /// (`DriveCommand`). Compiles only while moving (`tests/ui/parked_hold.rs`,
    /// `tests/ui/estopped_hold.rs`).
    pub fn hold_now(&mut self) -> Result<(), GroundError> {
        super::authority::require(self.permit.as_ref(), &self.backend)
            .map_err(GroundError::StaleAuthority)?;
        self.safety = ground::ground_step(self.safety, GroundEvent::DriveCommand)
            .map_err(GroundError::Safety)?;
        self.push_ground()?;
        self.backend.hold_now().map_err(GroundError::Backend)
    }

    /// Same grant as [`Self::hold_now`], then tick the plant 20 ms.
    pub async fn hold(&mut self) -> Result<(), GroundError> {
        self.hold_now()?;
        self.backend
            .tick(0.02)
            .await
            .map_err(GroundError::Backend)?;
        Ok(())
    }

    /// Return to parked without an async runtime. Drive commands cease to exist.
    pub fn park_now(mut self) -> GroundVehicle<Parked, B> {
        self.safety = ground::ground_step(self.safety, GroundEvent::Halt).unwrap_or(GroundState {
            phase: GroundPhase::Parked,
            drive_enabled: false,
            estop: false,
        });
        let _ = self.backend.halt_now();
        let _ = self.push_ground();
        self.retarget()
    }

    /// Return to parked (zero command, drive disabled).
    pub async fn park(mut self) -> GroundVehicle<Parked, B> {
        let _ = self
            .backend
            .set_velocity_ned(Velocity::<Ned>::ned(0.0, 0.0, 0.0))
            .await;
        self.park_now()
    }

    /// Emergency stop. Drive commands cease to exist.
    pub async fn emergency_stop(self) -> GroundVehicle<EStopped, B> {
        self.emergency_stop_now()
    }
}

impl<B: VehicleBackend> GroundVehicle<EStopped, B> {
    /// Clear E-stop back to parked. Compiles only from E-stop
    /// (`tests/ui/parked_reset.rs`, `tests/ui/moving_reset.rs`).
    pub fn reset(mut self) -> Result<GroundVehicle<Parked, B>, GroundError> {
        self.safety = ground::ground_step(self.safety, GroundEvent::ClearEstop)
            .map_err(GroundError::Safety)?;
        self.push_ground()?;
        Ok(self.retarget())
    }
}

/// Consume-self chassis bound to a live plant phase. [`GroundVehicle::new`]
/// always starts `Parked` and does not read the world; use this after
/// `attach_drive` so the typestate matches the chassis.
#[derive(Debug)]
pub enum GroundHandle<B> {
    Parked(GroundVehicle<Parked, B>),
    Moving(GroundVehicle<Moving, B>),
    EStopped(GroundVehicle<EStopped, B>),
}

impl<B: VehicleBackend> GroundHandle<B> {
    pub fn from_state(backend: B, safety: GroundState) -> Self {
        match ground_kind(safety) {
            GroundKind::Parked => Self::Parked(wrap_ground(backend, safety)),
            GroundKind::Moving => Self::Moving(wrap_ground(backend, safety)),
            GroundKind::EStopped => Self::EStopped(wrap_ground(backend, safety)),
        }
    }
}

impl<B> GroundHandle<B> {
    pub fn kind(&self) -> GroundKind {
        match self {
            Self::Parked(_) => GroundKind::Parked,
            Self::Moving(_) => GroundKind::Moving,
            Self::EStopped(_) => GroundKind::EStopped,
        }
    }

    pub fn safety(&self) -> GroundState {
        match self {
            Self::Parked(v) => v.safety(),
            Self::Moving(v) => v.safety(),
            Self::EStopped(v) => v.safety(),
        }
    }

    pub fn backend(&self) -> &B {
        match self {
            Self::Parked(v) => v.backend(),
            Self::Moving(v) => v.backend(),
            Self::EStopped(v) => v.backend(),
        }
    }

    pub fn backend_mut(&mut self) -> &mut B {
        match self {
            Self::Parked(v) => v.backend_mut(),
            Self::Moving(v) => v.backend_mut(),
            Self::EStopped(v) => v.backend_mut(),
        }
    }

    pub fn into_backend(self) -> B {
        match self {
            Self::Parked(v) => v.into_backend(),
            Self::Moving(v) => v.into_backend(),
            Self::EStopped(v) => v.into_backend(),
        }
    }
}

fn wrap_ground<S, B: VehicleBackend>(backend: B, safety: GroundState) -> GroundVehicle<S, B> {
    let permit = if ground_kind(safety).grants_actuation() {
        Some(super::authority::issue(&backend))
    } else {
        None
    };
    GroundVehicle {
        backend,
        safety,
        permit,
        _state: PhantomData,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::Position;

    #[tokio::test]
    async fn parked_then_drive() {
        let v = GroundVehicle::<Parked, NullBackend>::null();
        let mut v = v.enable_drive().unwrap();
        assert_eq!(v.phase(), GroundPhase::Moving);
        v.set_twist(
            Velocity::<Body>::new(0.5, 0.0, 0.0),
            AngularVelocity::body_rad(0.0, 0.0, 0.1),
        )
        .await
        .unwrap();
        let parked = v.park().await;
        assert_eq!(parked.phase(), GroundPhase::Parked);
        assert_eq!(
            parked.backend().ground.map(|s| s.phase),
            Some(GroundPhase::Parked)
        );
    }

    #[test]
    fn emergency_stop_now_returns_estopped_without_a_runtime() {
        let v = GroundVehicle::<Parked, NullBackend>::null()
            .enable_drive()
            .unwrap();
        assert_eq!(v.phase(), GroundPhase::Moving);
        let stopped = v.emergency_stop_now();
        assert_eq!(stopped.phase(), GroundPhase::EStop);
        assert!(stopped.safety().estop);
        assert!(!stopped.safety().drive_enabled);
        assert_eq!(
            stopped.backend().ground.map(|s| s.phase),
            Some(GroundPhase::EStop)
        );
        assert!(stopped.backend().velocity.is_none());
        let parked = stopped.reset().unwrap();
        assert_eq!(parked.phase(), GroundPhase::Parked);
    }

    #[test]
    fn hold_now_tracks_telemetry_pose() {
        let backend = NullBackend {
            position: Some(Position::<Ned>::ned(1.5, -0.25, 0.0)),
            ..NullBackend::default()
        };
        let mut v = GroundVehicle::<Parked, _>::new(backend)
            .enable_drive()
            .unwrap();
        v.hold_now().unwrap();
        let p = v.backend().position.unwrap();
        assert_eq!((p.x(), p.y(), p.z()), (1.5, -0.25, 0.0));
        assert_eq!(
            v.backend().ground.map(|s| s.phase),
            Some(GroundPhase::Moving)
        );
    }

    #[test]
    fn park_now_returns_parked_without_a_runtime() {
        let v = GroundVehicle::<Parked, NullBackend>::null()
            .enable_drive()
            .unwrap();
        assert_eq!(v.phase(), GroundPhase::Moving);
        let parked = v.park_now();
        assert_eq!(parked.phase(), GroundPhase::Parked);
        assert!(!parked.safety().drive_enabled);
        assert_eq!(
            parked.backend().ground.map(|s| s.phase),
            Some(GroundPhase::Parked)
        );
    }

    #[tokio::test]
    async fn body_twist_follows_heading() {
        let backend = NullBackend {
            yaw_rad: core::f32::consts::FRAC_PI_2,
            ..NullBackend::default()
        };
        let mut v = GroundVehicle::<Parked, _>::new(backend)
            .enable_drive()
            .unwrap();
        v.set_twist(
            Velocity::<Body>::new(1.0, 0.0, 0.0),
            AngularVelocity::body_rad(0.0, 0.0, 0.25),
        )
        .await
        .unwrap();
        let sp = v.backend().velocity.unwrap();
        assert!((sp.x() - 0.0).abs() < 1e-5, "north {}", sp.x());
        assert!((sp.y() - 1.0).abs() < 1e-5, "east {}", sp.y());
        assert!((v.backend().yaw_rate - 0.25).abs() < 1e-6);
        assert_eq!(
            v.backend().ground.map(|s| s.phase),
            Some(GroundPhase::Moving)
        );
    }

    #[test]
    fn body_xy_to_ned_identity_and_east() {
        let n = body_xy_to_ned(0.0, 1.0, 0.0);
        assert!((n[0] - 1.0).abs() < 1e-6 && n[1].abs() < 1e-6);
        let e = body_xy_to_ned(core::f32::consts::FRAC_PI_2, 1.0, 0.0);
        assert!(e[0].abs() < 1e-5 && (e[1] - 1.0).abs() < 1e-5);
        let crab = body_xy_to_ned(0.0, 0.0, 1.0);
        assert!(crab[0].abs() < 1e-6 && (crab[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn estop_from_parked() {
        let v = GroundVehicle::<Parked, NullBackend>::null().emergency_stop();
        assert!(v.safety().estop);
        let parked = v.reset().unwrap();
        assert_eq!(parked.phase(), GroundPhase::Parked);
    }

    #[test]
    fn emergency_stop_now_from_parked() {
        let v = GroundVehicle::<Parked, NullBackend>::null().emergency_stop_now();
        assert_eq!(v.phase(), GroundPhase::EStop);
        assert!(v.safety().estop);
        assert!(!v.safety().drive_enabled);
        assert_eq!(
            v.backend().ground.map(|s| s.phase),
            Some(GroundPhase::EStop)
        );
    }

    #[test]
    fn ground_error_collapses_to_backend() {
        assert_eq!(
            GroundError::Backend(BackendError::Timeout).into_backend(),
            BackendError::Timeout
        );
        assert_eq!(
            GroundError::Safety(GroundReject::IllegalPhase).into_backend(),
            BackendError::Rejected("ground safety")
        );
        let via_from: BackendError = GroundError::Safety(GroundReject::EStopped).into();
        assert_eq!(via_from, BackendError::Rejected("ground safety"));
    }

    #[test]
    fn handle_backend_accessors_round_trip() {
        let mut h = GroundHandle::from_state(NullBackend::default(), GroundState::parked());
        h.backend_mut().yaw_rad = 0.25;
        assert!((h.backend().yaw_rad - 0.25).abs() < 1e-6);
        let backend = h.into_backend();
        assert!((backend.yaw_rad - 0.25).abs() < 1e-6);
    }

    #[test]
    fn from_state_preserves_every_packed_ground_machine() {
        use crate::ground::{ground_invariants, unpack_ground};
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            let h = GroundHandle::from_state(NullBackend::default(), s);
            assert_eq!(h.safety(), s, "bits={bits}");
            assert_eq!(h.kind(), ground_kind(s), "bits={bits}");
            match (h.kind(), s.phase) {
                (GroundKind::Parked, GroundPhase::Parked)
                | (GroundKind::Moving, GroundPhase::Moving)
                | (GroundKind::EStopped, GroundPhase::EStop) => {}
                _ => panic!("attach mismatch {s:?}"),
            }
        }
    }

    #[test]
    fn ground_kind_maps_parked_moving_and_estop() {
        assert_eq!(ground_kind(GroundState::parked()), GroundKind::Parked);
        let moving = ground::ground_step(GroundState::parked(), GroundEvent::Release).unwrap();
        assert_eq!(ground_kind(moving), GroundKind::Moving);
        let mut estop = GroundState::parked();
        estop.estop = true;
        estop.phase = GroundPhase::EStop;
        assert_eq!(ground_kind(estop), GroundKind::EStopped);
    }

    #[test]
    fn revoke_rejects_drive_while_typestate_is_still_moving() {
        let mut v = GroundVehicle::<Parked, NullBackend>::null()
            .enable_drive()
            .unwrap();
        v.set_velocity_ned_now(Velocity::<Ned>::ned(0.4, 0.0, 0.0))
            .unwrap();
        v.backend_mut().revoke_authority();
        let err = v
            .set_velocity_ned_now(Velocity::<Ned>::ned(0.4, 0.0, 0.0))
            .unwrap_err();
        assert!(matches!(
            err,
            GroundError::StaleAuthority(crate::contracts::AuthorityReject::StaleEpoch)
        ));
        assert_eq!(v.phase(), GroundPhase::Moving);
    }
}
