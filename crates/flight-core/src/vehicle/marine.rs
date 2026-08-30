//! Typestate marine platform: `Docked` cannot command thrust.

use super::backend::{BackendError, NullBackend, Telemetry, VehicleBackend};
use crate::frames::Ned;
use crate::marine::{self, MarineEvent, MarinePhase, MarineReject, MarineState};
use crate::vector::Velocity;
use core::fmt;
use core::marker::PhantomData;

/// Alongside: thrust commands do not exist.
#[derive(Clone, Copy, Debug, Default)]
pub struct Docked;

/// Making way: thrust is legal.
#[derive(Clone, Copy, Debug, Default)]
pub struct Underway;

/// Holding station: small thrust is legal.
#[derive(Clone, Copy, Debug, Default)]
pub struct StationKeep;

/// Marine failsafe: thrust commands do not exist.
#[derive(Clone, Copy, Debug, Default)]
pub struct MarineFailsafe;

/// Marker: kernel `Failsafe` is a consume-self method.
///
/// Underway and StationKeep hulls can trip. Docked and already-failsafe cannot
/// — attach those as [`BackendError::Protocol`]. Docked has no
/// `declare_failsafe` (see `tests/ui/docked_failsafe.rs`).
pub trait CanTripMarineFailsafe {}
impl CanTripMarineFailsafe for Underway {}
impl CanTripMarineFailsafe for StationKeep {}

/// Marker: kernel `Dock` is a consume-self method.
///
/// Underway and StationKeep can come alongside. Docked and failsafe cannot
/// — attach those as [`BackendError::Protocol`].
pub trait CanDock {}
impl CanDock for Underway {}
impl CanDock for StationKeep {}

/// Marker: kernel `ThrustCommand` is a consume-self method.
///
/// Underway and StationKeep can command NED thrust. Docked and failsafe cannot
/// — attach those as [`BackendError::Rejected`].
pub trait CanThrust {}
impl CanThrust for Underway {}
impl CanThrust for StationKeep {}

/// Marine platform whose methods exist only in legal phases.
#[derive(Debug)]
pub struct MarineVehicle<S, B> {
    backend: B,
    safety: MarineState,
    permit: Option<crate::contracts::ActuationPermit>,
    _state: PhantomData<S>,
}

/// Dock / thrust error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarineError {
    Safety(MarineReject),
    Backend(BackendError),
    StaleAuthority(crate::contracts::AuthorityReject),
}

impl fmt::Display for MarineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MarineError::Safety(r) => write!(f, "marine safety: {r}"),
            MarineError::Backend(b) => write!(f, "marine backend: {b}"),
            MarineError::StaleAuthority(r) => write!(f, "marine stale authority: {r}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for MarineError {}

impl MarineError {
    /// Collapse a marine error into a [`BackendError`] for session helpers.
    pub fn into_backend(self) -> BackendError {
        match self {
            Self::Backend(b) => b,
            Self::Safety(_) => BackendError::Rejected("marine safety"),
            Self::StaleAuthority(_) => BackendError::Rejected("stale_authority"),
        }
    }
}

impl From<MarineError> for BackendError {
    fn from(e: MarineError) -> Self {
        e.into_backend()
    }
}

/// Which consume-self typestate [`MarineHandle::from_state`] binds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum MarineKind {
    Docked,
    Underway,
    StationKeep,
    Failsafe,
}

impl MarineKind {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Docked => "docked",
            Self::Underway => "underway",
            Self::StationKeep => "station_keep",
            Self::Failsafe => "failsafe",
        }
    }

    pub const fn grants_actuation(self) -> bool {
        matches!(self, Self::Underway | Self::StationKeep)
    }
}

impl fmt::Display for MarineKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Map a live hull onto the consume-self typestate `attach` uses.
pub fn marine_kind(safety: MarineState) -> MarineKind {
    if safety.failsafe {
        return MarineKind::Failsafe;
    }
    match safety.phase {
        MarinePhase::Docked => MarineKind::Docked,
        MarinePhase::Underway => MarineKind::Underway,
        MarinePhase::StationKeep => MarineKind::StationKeep,
        MarinePhase::Failsafe => MarineKind::Failsafe,
    }
}

impl<S, B> MarineVehicle<S, B> {
    pub fn safety(&self) -> MarineState {
        self.safety
    }

    pub fn phase(&self) -> MarinePhase {
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

    fn retarget<T>(self) -> MarineVehicle<T, B> {
        MarineVehicle {
            backend: self.backend,
            safety: self.safety,
            permit: self.permit,
            _state: PhantomData,
        }
    }
}

impl<S, B: VehicleBackend> MarineVehicle<S, B> {
    /// Advance the plant. Every marine phase can observe.
    pub async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, MarineError> {
        self.backend
            .tick(dt_secs)
            .await
            .map_err(MarineError::Backend)
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, MarineError> {
        self.backend.telemetry().await.map_err(MarineError::Backend)
    }

    fn push_marine(&mut self) -> Result<(), MarineError> {
        self.backend
            .sync_marine(self.safety)
            .map_err(MarineError::Backend)
    }

    fn command_thrust_now(&mut self, v: Velocity<Ned>) -> Result<(), MarineError> {
        super::authority::require(self.permit.as_ref(), &self.backend)
            .map_err(MarineError::StaleAuthority)?;
        self.safety = marine::marine_step(self.safety, MarineEvent::ThrustCommand)
            .map_err(MarineError::Safety)?;
        self.push_marine()?;
        self.backend
            .set_velocity_ned_now(v)
            .map_err(MarineError::Backend)
    }

    fn apply_dock(mut self) -> MarineVehicle<Docked, B> {
        self.safety = marine::marine_step(self.safety, MarineEvent::Dock).unwrap_or(MarineState {
            phase: MarinePhase::Docked,
            thrust_enabled: false,
            failsafe: false,
        });
        let _ = self.push_marine();
        self.retarget()
    }

    fn apply_failsafe(mut self) -> MarineVehicle<MarineFailsafe, B> {
        self.safety =
            marine::marine_step(self.safety, MarineEvent::Failsafe).unwrap_or(MarineState {
                phase: MarinePhase::Failsafe,
                thrust_enabled: false,
                failsafe: true,
            });
        let _ = self.backend.trigger_failsafe_now();
        let _ = self.push_marine();
        self.retarget()
    }
}

impl<S: CanTripMarineFailsafe, B: VehicleBackend> MarineVehicle<S, B> {
    /// Flood / leak / loss of control.
    pub fn declare_failsafe(self) -> MarineVehicle<MarineFailsafe, B> {
        self.apply_failsafe()
    }
}

impl<S: CanDock, B: VehicleBackend> MarineVehicle<S, B> {
    /// Come alongside without an async runtime. Thrust commands cease to exist.
    pub fn dock_now(self) -> MarineVehicle<Docked, B> {
        self.apply_dock()
    }

    /// Come alongside. Thrust commands cease to exist.
    pub async fn dock(mut self) -> MarineVehicle<Docked, B> {
        let _ = self
            .backend
            .set_velocity_ned(Velocity::<Ned>::ned(0.0, 0.0, 0.0))
            .await;
        self.dock_now()
    }
}

impl<S: CanThrust, B: VehicleBackend> MarineVehicle<S, B> {
    /// NED velocity through water. Exists only while thrust is granted.
    pub async fn set_ned_velocity(&mut self, v: Velocity<Ned>) -> Result<(), MarineError> {
        self.set_ned_velocity_now(v)
    }

    /// Same grant as [`Self::set_ned_velocity`] without stepping the plant.
    pub fn set_ned_velocity_now(&mut self, v: Velocity<Ned>) -> Result<(), MarineError> {
        self.command_thrust_now(v)
    }

    /// Hold at the current NED pose. Distinct from [`MarineVehicle::hold_station`]
    /// (the StationKeep machine). Compiles only while thrust is granted
    /// (`tests/ui/docked_hold.rs`, `tests/ui/marine_failsafe_hold.rs`).
    pub fn hold_now(&mut self) -> Result<(), MarineError> {
        super::authority::require(self.permit.as_ref(), &self.backend)
            .map_err(MarineError::StaleAuthority)?;
        self.safety = marine::marine_step(self.safety, MarineEvent::ThrustCommand)
            .map_err(MarineError::Safety)?;
        self.push_marine()?;
        self.backend.hold_now().map_err(MarineError::Backend)
    }

    /// Same grant as [`Self::hold_now`], then tick the plant 20 ms.
    pub async fn hold(&mut self) -> Result<(), MarineError> {
        self.hold_now()?;
        self.backend
            .tick(0.02)
            .await
            .map_err(MarineError::Backend)?;
        Ok(())
    }
}

impl<B: VehicleBackend> MarineVehicle<Docked, B> {
    /// Docked hull with thrust disabled.
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            safety: MarineState::docked(),
            permit: None,
            _state: PhantomData,
        }
    }

    /// Cast off. Compiles only from docked.
    pub fn undock(mut self) -> Result<MarineVehicle<Underway, B>, MarineError> {
        self.safety =
            marine::marine_step(self.safety, MarineEvent::Undock).map_err(MarineError::Safety)?;
        self.push_marine()?;
        let mut v = self.retarget();
        v.permit = Some(super::authority::issue(&v.backend));
        Ok(v)
    }
}

impl MarineVehicle<Docked, NullBackend> {
    pub fn null() -> Self {
        Self::new(NullBackend::default())
    }
}

impl<B: VehicleBackend> MarineVehicle<Underway, B> {
    /// Switch to station keeping. Compiles only from Underway
    /// (`tests/ui/docked_station.rs`, `tests/ui/station_station.rs`,
    /// `tests/ui/failsafe_station.rs`).
    pub fn hold_station(mut self) -> Result<MarineVehicle<StationKeep, B>, MarineError> {
        self.safety =
            marine::marine_step(self.safety, MarineEvent::Station).map_err(MarineError::Safety)?;
        self.push_marine()?;
        Ok(self.retarget())
    }
}

impl<B: VehicleBackend> MarineVehicle<StationKeep, B> {
    /// Resume making way. Compiles only from StationKeep
    /// (`tests/ui/docked_resume.rs`, `tests/ui/underway_resume.rs`,
    /// `tests/ui/failsafe_resume.rs`).
    pub fn resume(mut self) -> Result<MarineVehicle<Underway, B>, MarineError> {
        self.safety =
            marine::marine_step(self.safety, MarineEvent::Resume).map_err(MarineError::Safety)?;
        self.push_marine()?;
        Ok(self.retarget())
    }
}

impl<B: VehicleBackend> MarineVehicle<MarineFailsafe, B> {
    /// After a leak, the hull is recovered docked with thrust disabled.
    /// Compiles only from Failsafe (`tests/ui/docked_recover.rs` and siblings).
    pub fn recover_docked(mut self) -> Result<MarineVehicle<Docked, B>, MarineError> {
        self.safety =
            marine::marine_step(self.safety, MarineEvent::Recover).map_err(MarineError::Safety)?;
        self.push_marine()?;
        Ok(self.retarget())
    }
}

/// Consume-self hull bound to a live plant phase. [`MarineVehicle::new`]
/// always starts `Docked` and does not read the world; use this after
/// `attach_undock` so the typestate matches the hull.
#[derive(Debug)]
pub enum MarineHandle<B> {
    Docked(MarineVehicle<Docked, B>),
    Underway(MarineVehicle<Underway, B>),
    StationKeep(MarineVehicle<StationKeep, B>),
    Failsafe(MarineVehicle<MarineFailsafe, B>),
}

impl<B: VehicleBackend> MarineHandle<B> {
    pub fn from_state(backend: B, safety: MarineState) -> Self {
        match marine_kind(safety) {
            MarineKind::Docked => Self::Docked(wrap_marine(backend, safety)),
            MarineKind::Underway => Self::Underway(wrap_marine(backend, safety)),
            MarineKind::StationKeep => Self::StationKeep(wrap_marine(backend, safety)),
            MarineKind::Failsafe => Self::Failsafe(wrap_marine(backend, safety)),
        }
    }
}

impl<B> MarineHandle<B> {
    pub fn kind(&self) -> MarineKind {
        match self {
            Self::Docked(_) => MarineKind::Docked,
            Self::Underway(_) => MarineKind::Underway,
            Self::StationKeep(_) => MarineKind::StationKeep,
            Self::Failsafe(_) => MarineKind::Failsafe,
        }
    }

    pub fn safety(&self) -> MarineState {
        match self {
            Self::Docked(v) => v.safety(),
            Self::Underway(v) => v.safety(),
            Self::StationKeep(v) => v.safety(),
            Self::Failsafe(v) => v.safety(),
        }
    }

    pub fn backend(&self) -> &B {
        match self {
            Self::Docked(v) => v.backend(),
            Self::Underway(v) => v.backend(),
            Self::StationKeep(v) => v.backend(),
            Self::Failsafe(v) => v.backend(),
        }
    }

    pub fn backend_mut(&mut self) -> &mut B {
        match self {
            Self::Docked(v) => v.backend_mut(),
            Self::Underway(v) => v.backend_mut(),
            Self::StationKeep(v) => v.backend_mut(),
            Self::Failsafe(v) => v.backend_mut(),
        }
    }

    pub fn into_backend(self) -> B {
        match self {
            Self::Docked(v) => v.into_backend(),
            Self::Underway(v) => v.into_backend(),
            Self::StationKeep(v) => v.into_backend(),
            Self::Failsafe(v) => v.into_backend(),
        }
    }
}

fn wrap_marine<S, B: VehicleBackend>(backend: B, safety: MarineState) -> MarineVehicle<S, B> {
    let permit = if marine_kind(safety).grants_actuation() {
        Some(super::authority::issue(&backend))
    } else {
        None
    };
    MarineVehicle {
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

    #[test]
    fn hold_now_tracks_telemetry_pose_from_underway() {
        let backend = NullBackend {
            position: Some(Position::<Ned>::ned(2.0, 0.5, 0.0)),
            ..NullBackend::default()
        };
        let mut v = MarineVehicle::<Docked, _>::new(backend).undock().unwrap();
        v.hold_now().unwrap();
        let p = v.backend().position.unwrap();
        assert_eq!((p.x(), p.y(), p.z()), (2.0, 0.5, 0.0));
        assert_eq!(
            v.backend().marine.map(|s| s.phase),
            Some(MarinePhase::Underway)
        );
    }

    #[test]
    fn hold_now_tracks_telemetry_pose_from_station_keep() {
        let backend = NullBackend {
            position: Some(Position::<Ned>::ned(-1.0, 3.0, 0.0)),
            ..NullBackend::default()
        };
        let mut v = MarineVehicle::<Docked, _>::new(backend)
            .undock()
            .unwrap()
            .hold_station()
            .unwrap();
        v.hold_now().unwrap();
        let p = v.backend().position.unwrap();
        assert_eq!((p.x(), p.y(), p.z()), (-1.0, 3.0, 0.0));
        assert_eq!(
            v.backend().marine.map(|s| s.phase),
            Some(MarinePhase::StationKeep)
        );
    }

    #[tokio::test]
    async fn docked_then_undock_thrust() {
        let v = MarineVehicle::<Docked, NullBackend>::null();
        let mut v = v.undock().unwrap();
        assert_eq!(v.phase(), MarinePhase::Underway);
        v.set_ned_velocity(Velocity::<Ned>::ned(0.4, 0.0, 0.0))
            .await
            .unwrap();
        let docked = v.dock().await;
        assert_eq!(docked.phase(), MarinePhase::Docked);
        assert_eq!(
            docked.backend().marine.map(|s| s.phase),
            Some(MarinePhase::Docked)
        );
    }

    #[test]
    fn dock_now_returns_docked_without_a_runtime() {
        let underway = MarineVehicle::<Docked, NullBackend>::null()
            .undock()
            .unwrap();
        let station = underway.hold_station().unwrap();
        assert_eq!(station.phase(), MarinePhase::StationKeep);
        let docked = station.dock_now();
        assert_eq!(docked.phase(), MarinePhase::Docked);
        assert!(!docked.safety().thrust_enabled);
        assert_eq!(
            docked.backend().marine.map(|s| s.phase),
            Some(MarinePhase::Docked)
        );
    }

    #[test]
    fn failsafe_kills_thrust_grant() {
        let v = MarineVehicle::<Docked, NullBackend>::null()
            .undock()
            .unwrap()
            .declare_failsafe();
        assert!(v.safety().failsafe);
        assert!(!v.safety().thrust_enabled);
        let docked = v.recover_docked().unwrap();
        assert_eq!(docked.phase(), MarinePhase::Docked);
    }

    #[test]
    fn failsafe_from_station_keep_kills_thrust_grant() {
        let v = MarineVehicle::<Docked, NullBackend>::null()
            .undock()
            .unwrap()
            .hold_station()
            .unwrap()
            .declare_failsafe();
        assert_eq!(v.phase(), MarinePhase::Failsafe);
        assert!(v.safety().failsafe);
        assert!(!v.safety().thrust_enabled);
        assert_eq!(
            v.backend().marine.map(|s| s.phase),
            Some(MarinePhase::Failsafe)
        );
        let docked = v.recover_docked().unwrap();
        assert_eq!(docked.phase(), MarinePhase::Docked);
    }

    #[test]
    fn marine_error_collapses_to_backend() {
        assert_eq!(
            MarineError::Backend(BackendError::Io).into_backend(),
            BackendError::Io
        );
        assert_eq!(
            MarineError::Safety(MarineReject::IllegalPhase).into_backend(),
            BackendError::Rejected("marine safety")
        );
        let via_from: BackendError = MarineError::Safety(MarineReject::InFailsafe).into();
        assert_eq!(via_from, BackendError::Rejected("marine safety"));
    }

    #[test]
    fn handle_backend_accessors_round_trip() {
        let mut h = MarineHandle::from_state(NullBackend::default(), MarineState::docked());
        h.backend_mut().yaw_rad = 1.5;
        assert!((h.backend().yaw_rad - 1.5).abs() < 1e-6);
        let backend = h.into_backend();
        assert!((backend.yaw_rad - 1.5).abs() < 1e-6);
    }

    #[test]
    fn from_state_preserves_every_packed_marine_machine() {
        use crate::marine::{marine_invariants, unpack_marine};
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            let h = MarineHandle::from_state(NullBackend::default(), s);
            assert_eq!(h.safety(), s, "bits={bits}");
            assert_eq!(h.kind(), marine_kind(s), "bits={bits}");
            match (h.kind(), s.phase) {
                (MarineKind::Docked, MarinePhase::Docked)
                | (MarineKind::Underway, MarinePhase::Underway)
                | (MarineKind::StationKeep, MarinePhase::StationKeep)
                | (MarineKind::Failsafe, MarinePhase::Failsafe) => {}
                _ => panic!("attach mismatch {s:?}"),
            }
        }
    }

    #[test]
    fn marine_kind_maps_docked_underway_station_and_failsafe() {
        assert_eq!(marine_kind(MarineState::docked()), MarineKind::Docked);
        let underway = marine::marine_step(MarineState::docked(), MarineEvent::Undock).unwrap();
        assert_eq!(marine_kind(underway), MarineKind::Underway);
        let station = marine::marine_step(underway, MarineEvent::Station).unwrap();
        assert_eq!(marine_kind(station), MarineKind::StationKeep);
        let mut fs = MarineState::docked();
        fs.failsafe = true;
        fs.phase = MarinePhase::Failsafe;
        assert_eq!(marine_kind(fs), MarineKind::Failsafe);
    }

    #[test]
    fn revoke_rejects_thrust_while_typestate_is_still_underway() {
        let mut v = MarineVehicle::<Docked, NullBackend>::null()
            .undock()
            .unwrap();
        v.set_ned_velocity_now(Velocity::<Ned>::ned(0.4, 0.0, 0.0))
            .unwrap();
        v.backend_mut().revoke_authority();
        let err = v
            .set_ned_velocity_now(Velocity::<Ned>::ned(0.4, 0.0, 0.0))
            .unwrap_err();
        assert!(matches!(
            err,
            MarineError::StaleAuthority(crate::contracts::AuthorityReject::StaleEpoch)
        ));
        assert_eq!(v.phase(), MarinePhase::Underway);
    }
}
