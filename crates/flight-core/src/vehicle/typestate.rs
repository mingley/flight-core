//! Typestate vehicle API.
//!
//! Illegal operations are not methods, so they cannot be written:
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! use flight_core::vehicle::{MotorThrust, Vehicle};
//! fn boom<B>(mut vehicle: Vehicle<Disarmed, B>, thrust: MotorThrust) {
//!     let _ = vehicle.set_motor_thrust(thrust);
//! }
//! ```
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! use flight_core::vehicle::Vehicle;
//! fn boom<B>(vehicle: Vehicle<Disconnected, B>) {
//!     let _ = vehicle.arm();
//! }
//! ```

use super::backend::{BackendError, MotorThrust, NullBackend, Telemetry, VehicleBackend};
use crate::contracts::{ActuationPermit, AuthorityReject};
use crate::frames::Ned;
use crate::safety::{self, Event, Phase, Reject, SafetyState};
use crate::temporal::Command;
use crate::time::Duration;
use crate::units::{Meter, Qty};
use crate::vector::{Position, Velocity};
use core::fmt;
use core::marker::PhantomData;

mod sealed {
    pub trait Sealed {}
}

pub trait State: sealed::Sealed + Copy + Clone + fmt::Debug + Send + Sync + 'static {
    const NAME: &'static str;
}

macro_rules! state {
    ($name:ident, $label:literal) => {
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name;
        impl sealed::Sealed for $name {}
        impl State for $name {
            const NAME: &'static str = $label;
        }
    };
}

state!(Disconnected, "disconnected");
state!(Disarmed, "disarmed");
state!(PreflightReady, "preflight_ready");
state!(Armed, "armed");
state!(Offboard, "offboard");
state!(Takeoff, "takeoff");
state!(Airborne, "airborne");
state!(Landing, "landing");
state!(Failsafe, "failsafe");
state!(Recovery, "recovery");

/// Marker: motor / actuator commands are legal.
///
/// Armed, Offboard, Takeoff, Airborne, and Landing compile `set_motor_thrust`.
/// Ready, Failsafe, Recovery, Disarmed, and Disconnected do not
/// (`tests/ui/ready_thrust.rs`, `tests/ui/failsafe_motor.rs`, and siblings).
pub trait MotorsEnabled: State {}
impl MotorsEnabled for Armed {}
impl MotorsEnabled for Offboard {}
impl MotorsEnabled for Takeoff {}
impl MotorsEnabled for Airborne {}
impl MotorsEnabled for Landing {}

/// Marker: velocity / position offboard setpoints are legal.
///
/// Offboard, Takeoff, Airborne, and Landing compile `set_velocity` /
/// `set_position` / `hold`. Ready, Armed, Failsafe, Recovery, Disarmed, and
/// Disconnected do not (`tests/ui/ready_velocity.rs`,
/// `tests/ui/ready_position.rs`, `tests/ui/ready_hold.rs`, and siblings).
pub trait OffboardControl: State {}
impl OffboardControl for Offboard {}
impl OffboardControl for Takeoff {}
impl OffboardControl for Airborne {}
impl OffboardControl for Landing {}

/// Marker: kernel `TriggerFailsafe` is a consume-self method.
///
/// Ready and Armed pad vehicles can trip. Already-failsafe, Recovery, Disarmed,
/// and Disconnected cannot — attach those as [`BackendError::Protocol`].
pub trait CanTripFailsafe: State {}
impl CanTripFailsafe for PreflightReady {}
impl CanTripFailsafe for Armed {}
impl CanTripFailsafe for Offboard {}
impl CanTripFailsafe for Takeoff {}
impl CanTripFailsafe for Airborne {}
impl CanTripFailsafe for Landing {}

/// Marker: kernel `Touchdown` is a consume-self method.
///
/// Landing and Failsafe can touch down to Ready. Armed, Offboard, Takeoff,
/// Airborne, Ready, Recovery, Disarmed, and Disconnected cannot — attach those
/// as [`BackendError::Protocol`].
pub trait CanTouchdown: State {}
impl CanTouchdown for Landing {}
impl CanTouchdown for Failsafe {}

/// Marker: kernel `Land` is a consume-self method.
///
/// Takeoff and Airborne can enter Landing. Offboard without Takeoff, Armed,
/// Ready, Landing, Disconnected, Failsafe, Recovery, and Disarmed cannot —
/// attach those as [`BackendError::Protocol`].
pub trait CanBeginLand: State {}
impl CanBeginLand for Takeoff {}
impl CanBeginLand for Airborne {}

/// Marker: kernel `Disarm` is a consume-self method back to Ready.
///
/// Ready, Armed, Offboard, Takeoff, Airborne, and Landing can disarm to Ready.
/// Failsafe disarms to Recovery (not this trait). Recovery, Disarmed, and
/// Disconnected cannot — attach those as [`BackendError::Protocol`].
pub trait CanDisarm: State {}
impl CanDisarm for PreflightReady {}
impl CanDisarm for Armed {}
impl CanDisarm for Offboard {}
impl CanDisarm for Takeoff {}
impl CanDisarm for Airborne {}
impl CanDisarm for Landing {}

#[derive(Debug)]
pub struct Inner<B> {
    pub backend: B,
    pub safety: SafetyState,
    pub connection_system_id: u8,
    pub permit: Option<crate::contracts::ActuationPermit>,
}

#[derive(Debug)]
pub struct Vehicle<S: State, B> {
    inner: Inner<B>,
    _state: PhantomData<S>,
}

#[derive(Debug)]
pub struct TransitionError<S: State, B> {
    pub vehicle: Vehicle<S, B>,
    pub error: ErrorKind,
}

impl<S: State, B> TransitionError<S, B> {
    pub fn into_parts(self) -> (Vehicle<S, B>, ErrorKind) {
        (self.vehicle, self.error)
    }
}

impl<S: State, B> fmt::Display for TransitionError<S, B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.error)
    }
}

#[cfg(feature = "std")]
impl<S: State, B: fmt::Debug> std::error::Error for TransitionError<S, B> {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Safety(Reject),
    Backend(BackendError),
    PreflightFailed,
    Timeout,
    StaleAuthority(AuthorityReject),
}

impl fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Safety(r) => write!(f, "safety: {r}"),
            Self::Backend(b) => write!(f, "backend: {b}"),
            Self::PreflightFailed => write!(f, "preflight checks failed"),
            Self::Timeout => write!(f, "timed out waiting for the vehicle"),
            Self::StaleAuthority(r) => write!(f, "stale authority: {r}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ErrorKind {}

impl ErrorKind {
    /// Collapse a typestate error into a [`BackendError`] for APIs that return
    /// a live backend instead of [`TransitionError`].
    pub fn into_backend(self) -> BackendError {
        match self {
            Self::Backend(b) => b,
            Self::Timeout => BackendError::Timeout,
            Self::Safety(_) => BackendError::Rejected("safety"),
            Self::PreflightFailed => BackendError::Rejected("preflight"),
            Self::StaleAuthority(_) => BackendError::Rejected("stale_authority"),
        }
    }
}

impl From<ErrorKind> for BackendError {
    fn from(e: ErrorKind) -> Self {
        e.into_backend()
    }
}

impl<S: State, B> Vehicle<S, B> {
    pub fn safety(&self) -> SafetyState {
        self.inner.safety
    }

    pub fn phase(&self) -> Phase {
        self.inner.safety.phase
    }

    pub fn backend(&self) -> &B {
        &self.inner.backend
    }

    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.inner.backend
    }

    pub fn into_backend(self) -> B {
        self.inner.backend
    }

    fn apply_event(&mut self, event: Event) -> Result<(), ErrorKind> {
        self.inner.safety = safety::step(self.inner.safety, event).map_err(ErrorKind::Safety)?;
        Ok(())
    }

    fn apply_all(&mut self, events: &[Event]) -> Result<(), ErrorKind> {
        self.inner.safety =
            safety::step_all(self.inner.safety, events).map_err(ErrorKind::Safety)?;
        Ok(())
    }

    fn retarget<T: State>(self) -> Vehicle<T, B> {
        Vehicle {
            inner: self.inner,
            _state: PhantomData,
        }
    }

    fn fail<T: State>(self, error: ErrorKind) -> TransitionError<T, B> {
        TransitionError {
            vehicle: Vehicle {
                inner: self.inner,
                _state: PhantomData,
            },
            error,
        }
    }
}

impl<S: State, B: VehicleBackend> Vehicle<S, B> {
    /// Live permit, if this handle currently holds actuation evidence.
    pub fn permit(&self) -> Option<&ActuationPermit> {
        self.inner.permit.as_ref()
    }

    fn issue_unbounded_permit(&mut self) {
        self.inner.permit = Some(super::authority::issue(&self.inner.backend));
    }

    /// Permit epoch, vehicle id, and lease. Heartbeat age is not part of this
    /// check: entering offboard applies `HeartbeatFresh` and must not require
    /// the bit it is about to set.
    fn require_permit(&self) -> Result<(), ErrorKind> {
        super::authority::require(self.inner.permit.as_ref(), &self.inner.backend)
            .map_err(ErrorKind::StaleAuthority)
    }

    fn require_live_permit(&self) -> Result<(), ErrorKind> {
        self.require_permit()?;
        if let Some(age) = self.inner.backend.authority_heartbeat_age_ms() {
            // HeartbeatFresh owns the bound; admit is the kernel TCB. Fail closed
            // if they ever disagree.
            if crate::temporal::HeartbeatFresh::check_age(age).is_err()
                || !crate::contracts::AerialOffboard::admit(age, 0)
            {
                return Err(ErrorKind::StaleAuthority(AuthorityReject::StaleHeartbeat));
            }
        }
        Ok(())
    }

    fn require_command_age(&self, command_age_ms: u32) -> Result<(), ErrorKind> {
        if crate::temporal::CommandFresh::<()>::check_age(command_age_ms).is_err()
            || !crate::contracts::AerialOffboard::admit(0, command_age_ms)
        {
            return Err(ErrorKind::StaleAuthority(AuthorityReject::StaleCommand));
        }
        Ok(())
    }
}

impl<B: VehicleBackend> Vehicle<Disconnected, B> {
    pub fn new(backend: B) -> Self {
        Self {
            inner: Inner {
                backend,
                safety: SafetyState::disconnected(),
                connection_system_id: 0,
                permit: None,
            },
            _state: PhantomData,
        }
    }

    pub async fn connect(
        mut self,
    ) -> Result<Vehicle<Disarmed, B>, TransitionError<Disconnected, B>> {
        match self.inner.backend.connect().await {
            Ok(info) => {
                self.inner.connection_system_id = info.system_id;
                if let Err(error) =
                    self.apply_all(&[Event::Connect, Event::InitComplete, Event::Initialized])
                {
                    return Err(self.fail(error));
                }
                Ok(self.retarget())
            }
            Err(e) => Err(self.fail(ErrorKind::Backend(e))),
        }
    }
}

impl Vehicle<Disconnected, NullBackend> {
    pub fn null() -> Self {
        Self::new(NullBackend::default())
    }
}

impl<B: VehicleBackend> Vehicle<Disarmed, B> {
    pub async fn verify_preflight(
        mut self,
    ) -> Result<Vehicle<PreflightReady, B>, TransitionError<Disarmed, B>> {
        match self.inner.backend.preflight().await {
            Ok(report) => {
                if !report.ready() {
                    return Err(self.fail(ErrorKind::PreflightFailed));
                }
                match self.inner.safety.phase {
                    Phase::Ready => Ok(self.retarget()),
                    Phase::Preflight => {
                        if let Err(error) = self.apply_all(&[
                            Event::ImuHealthy,
                            Event::EstimatorValid,
                            Event::PreflightPassed,
                        ]) {
                            return Err(self.fail(error));
                        }
                        Ok(self.retarget())
                    }
                    _ => Err(self.fail(ErrorKind::Safety(Reject::IllegalPhase))),
                }
            }
            Err(e) => Err(self.fail(ErrorKind::Backend(e))),
        }
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

impl<B: VehicleBackend> Vehicle<PreflightReady, B> {
    pub async fn arm(self) -> Result<Vehicle<Armed, B>, TransitionError<PreflightReady, B>> {
        self.arm_now()
    }

    /// Same as [`Self::arm`] when the backend completes without parking.
    /// Compiles only from Ready (`tests/ui/armed_arm.rs` and siblings).
    pub fn arm_now(mut self) -> Result<Vehicle<Armed, B>, TransitionError<PreflightReady, B>> {
        match self.inner.backend.arm_now() {
            Ok(()) => {
                if let Err(error) = self.apply_event(Event::Arm) {
                    return Err(self.fail(error));
                }
                self.issue_unbounded_permit();
                Ok(self.retarget())
            }
            Err(e) => Err(self.fail(ErrorKind::Backend(e))),
        }
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

impl<B: VehicleBackend> Vehicle<Armed, B> {
    pub async fn enter_offboard(self) -> Result<Vehicle<Offboard, B>, TransitionError<Armed, B>> {
        self.enter_offboard_now()
    }

    /// Same as [`Self::enter_offboard`] when the backend completes without parking.
    /// Compiles only from Armed (`tests/ui/ready_offboard.rs` and siblings).
    ///
    /// Checks the permit **epoch** so a leftover `Vehicle<Armed>` after failsafe
    /// or an async PX4 disarm cannot switch modes. Heartbeat freshness is
    /// applied next (`HeartbeatFresh`); it is not a precondition of entry.
    pub fn enter_offboard_now(mut self) -> Result<Vehicle<Offboard, B>, TransitionError<Armed, B>> {
        if let Err(error) = self.require_permit() {
            return Err(self.fail(error));
        }
        if let Err(error) = self.apply_event(Event::HeartbeatFresh) {
            return Err(self.fail(error));
        }
        match self.inner.backend.enter_offboard_now() {
            Ok(()) => {
                if let Err(error) = self.apply_event(Event::EnterOffboard) {
                    return Err(self.fail(error));
                }
                if let Err(error) = self.apply_event(Event::EnableActuators) {
                    return Err(self.fail(error));
                }
                if let Err(e) = self.inner.backend.enable_actuators_now() {
                    return Err(self.fail(ErrorKind::Backend(e)));
                }
                self.issue_unbounded_permit();
                Ok(self.retarget())
            }
            Err(e) => Err(self.fail(error_from_backend(e))),
        }
    }

    /// Enter offboard and bind a time-bounded actuation lease.
    pub fn acquire_offboard_control_now(
        self,
        lease: Duration,
    ) -> Result<Vehicle<Offboard, B>, TransitionError<Armed, B>> {
        let mut v = self.enter_offboard_now()?;
        v.inner.permit = Some(super::authority::issue_bounded(&v.inner.backend, lease));
        Ok(v)
    }

    /// Same as [`Self::acquire_offboard_control_now`].
    pub async fn acquire_offboard_control(
        self,
        lease: Duration,
    ) -> Result<Vehicle<Offboard, B>, TransitionError<Armed, B>> {
        self.acquire_offboard_control_now(lease)
    }

    pub async fn takeoff(
        self,
        altitude_agl: Qty<Meter>,
    ) -> Result<Vehicle<Airborne, B>, TransitionError<Armed, B>> {
        match self.enter_offboard().await {
            Ok(v) => match v.takeoff(altitude_agl).await {
                Ok(air) => Ok(air),
                Err(e) => Err(TransitionError {
                    vehicle: e.vehicle.force_armed_view(),
                    error: e.error,
                }),
            },
            Err(e) => Err(e),
        }
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

impl<B> Vehicle<Offboard, B> {
    fn force_armed_view(self) -> Vehicle<Armed, B> {
        self.retarget()
    }
}

impl<B: VehicleBackend> Vehicle<Offboard, B> {
    pub async fn takeoff(
        self,
        altitude_agl: Qty<Meter>,
    ) -> Result<Vehicle<Airborne, B>, TransitionError<Offboard, B>> {
        let mut climbing = match self.start_takeoff_now() {
            Ok(v) => v,
            Err(e) => return Err(e),
        };

        let target = altitude_agl.get().max(0.3);
        let mut ticks = 0u32;
        loop {
            let climb = Velocity::<Ned>::ned(0.0, 0.0, -1.2);
            if let Err(error) = climbing.command_velocity(climb) {
                return Err(TransitionError {
                    vehicle: climbing.retarget(),
                    error,
                });
            }
            match climbing.inner.backend.tick(0.02).await {
                Ok(tel) => {
                    ticks += 1;
                    if tel.altitude_agl().get() >= target {
                        break;
                    }
                    if ticks > 5_000 {
                        return Err(TransitionError {
                            vehicle: climbing.retarget(),
                            error: ErrorKind::Timeout,
                        });
                    }
                }
                Err(e) => {
                    return Err(TransitionError {
                        vehicle: climbing.retarget(),
                        error: ErrorKind::Backend(e),
                    });
                }
            }
        }
        let hover = Velocity::<Ned>::ned(0.0, 0.0, 0.0);
        match apply_airborne(climbing) {
            Ok(mut air) => {
                if let Err(error) = air.command_velocity(hover) {
                    return Err(air.fail(error));
                }
                Ok(air)
            }
            Err(e) => Err(TransitionError {
                vehicle: e.vehicle.retarget(),
                error: e.error,
            }),
        }
    }

    /// Enable actuators and enter the takeoff phase without blocking on altitude.
    pub async fn start_takeoff(self) -> Result<Vehicle<Takeoff, B>, TransitionError<Offboard, B>> {
        self.start_takeoff_now()
    }

    /// Same as [`Self::start_takeoff`] when the backend completes without parking.
    /// Consumes Offboard so `begin_land_now` is not a method until Takeoff fires.
    /// Compiles only from Offboard (`tests/ui/ready_takeoff.rs` and siblings).
    pub fn start_takeoff_now(
        mut self,
    ) -> Result<Vehicle<Takeoff, B>, TransitionError<Offboard, B>> {
        if let Err(error) = self.require_live_permit() {
            return Err(self.fail(error));
        }
        if let Err(e) = self.inner.backend.takeoff_now() {
            return Err(self.fail(ErrorKind::Backend(e)));
        }
        if let Err(error) = self.apply_event(Event::Takeoff) {
            return Err(self.fail(error));
        }
        if let Err(e) = self.inner.backend.enable_actuators_now() {
            return Err(self.fail(ErrorKind::Backend(e)));
        }
        Ok(self.retarget())
    }
}

impl<B: VehicleBackend> Vehicle<Takeoff, B> {
    /// Record that the climb completed (`Takeoff → Airborne`) without stepping
    /// the plant. Consumes Takeoff so attach and the live handle agree.
    /// Compiles only from Takeoff (`tests/ui/offboard_airborne.rs` and siblings).
    pub fn declare_airborne_now(self) -> Result<Vehicle<Airborne, B>, TransitionError<Takeoff, B>> {
        apply_airborne(self)
    }

    /// Same as [`Self::declare_airborne_now`].
    pub fn declare_airborne(self) -> Result<Vehicle<Airborne, B>, TransitionError<Takeoff, B>> {
        self.declare_airborne_now()
    }
}

impl<S: CanBeginLand, B: VehicleBackend> Vehicle<S, B> {
    pub async fn land(mut self) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
        match descend_and_disarm(&mut self.inner).await {
            Ok(()) => Ok(self.retarget()),
            Err(error) => Err(self.fail(error)),
        }
    }

    /// Takeoff or Airborne → landing without stepping the plant.
    pub fn begin_land_now(self) -> Result<Vehicle<Landing, B>, TransitionError<S, B>> {
        apply_land(self)
    }
}

crate::impl_aerial_offboard_now!();

impl<S: OffboardControl, B: VehicleBackend> Vehicle<S, B> {
    pub async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.tick(dt_secs).await, &self.inner.safety)
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }

    fn command_velocity(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
        self.set_velocity_now(velocity)
    }

    /// Re-issue a time-bounded offboard lease. Fails if the live epoch already
    /// revoked the current permit.
    pub fn acquire_offboard_control_now(&mut self, lease: Duration) -> Result<(), ErrorKind> {
        self.require_live_permit()?;
        self.inner.permit = Some(super::authority::issue_bounded(&self.inner.backend, lease));
        Ok(())
    }

    /// Same as [`Self::acquire_offboard_control_now`].
    pub async fn acquire_offboard_control(&mut self, lease: Duration) -> Result<(), ErrorKind> {
        self.acquire_offboard_control_now(lease)
    }
}

impl<S: CanDisarm, B: VehicleBackend> Vehicle<S, B> {
    /// Ready / Armed / Offboard / Takeoff / Airborne / Landing → Ready.
    /// Clears the command. Failsafe uses [`Vehicle<Failsafe>::disarm_now`]
    /// into Recovery instead.
    pub fn disarm_now(self) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
        apply_disarm(self)
    }
}

impl<S: CanTripFailsafe, B: VehicleBackend> Vehicle<S, B> {
    pub async fn failsafe(self) -> Result<Vehicle<Failsafe, B>, TransitionError<S, B>> {
        self.failsafe_now()
    }

    /// Trip failsafe without stepping the plant. Mission commands cease to exist.
    pub fn failsafe_now(self) -> Result<Vehicle<Failsafe, B>, TransitionError<S, B>> {
        apply_failsafe(self)
    }
}

impl<S: CanTouchdown, B: VehicleBackend> Vehicle<S, B> {
    /// Landing or Failsafe → Ready without stepping the plant. Clears the
    /// backend command. Same kernel `Touchdown` from either phase.
    pub fn touchdown_now(self) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
        apply_touchdown(self)
    }
}

impl<S: MotorsEnabled, B: VehicleBackend> Vehicle<S, B> {
    pub async fn set_motor_thrust(&mut self, thrust: MotorThrust) -> Result<(), ErrorKind> {
        if !self.inner.safety.actuators_enabled {
            self.apply_event(Event::EnableActuators)?;
            self.inner
                .backend
                .enable_actuators()
                .await
                .map_err(ErrorKind::Backend)?;
        }
        self.require_live_permit()?;
        self.apply_event(Event::HeartbeatFresh)?;
        self.apply_event(Event::MissionCommand)?;
        self.inner
            .backend
            .set_motor_thrust(thrust)
            .await
            .map_err(ErrorKind::Backend)
    }
}

impl<B: VehicleBackend> Vehicle<Failsafe, B> {
    /// Failsafe → Recovery. Unarms; failsafe stays latched until
    /// [`Vehicle<Recovery>::recover_now`].
    pub fn disarm_now(self) -> Result<Vehicle<Recovery, B>, TransitionError<Failsafe, B>> {
        apply_failsafe_disarm(self)
    }

    pub async fn disarm(self) -> Result<Vehicle<Recovery, B>, TransitionError<Failsafe, B>> {
        self.disarm_now()
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

impl<B: VehicleBackend> Vehicle<Recovery, B> {
    /// Recovery → Ready. Clears the failsafe latch. Illegal unless the live
    /// machine is `Phase::Recovery` (disarmed). Compiles only from Recovery
    /// (`tests/ui/ready_recover.rs` and siblings).
    pub fn recover_now(self) -> Result<Vehicle<PreflightReady, B>, TransitionError<Recovery, B>> {
        apply_recover(self)
    }

    pub async fn recover(self) -> Result<Vehicle<PreflightReady, B>, TransitionError<Recovery, B>> {
        self.recover_now()
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

async fn descend_and_disarm<B: VehicleBackend>(inner: &mut Inner<B>) -> Result<(), ErrorKind> {
    inner.backend.land_now().map_err(ErrorKind::Backend)?;
    inner.safety = safety::step(inner.safety, Event::Land).map_err(ErrorKind::Safety)?;
    let mut ticks = 0u32;
    loop {
        inner.safety =
            safety::step(inner.safety, Event::HeartbeatFresh).map_err(ErrorKind::Safety)?;
        super::authority::require(inner.permit.as_ref(), &inner.backend)
            .map_err(ErrorKind::StaleAuthority)?;
        inner.safety =
            safety::step(inner.safety, Event::MissionCommand).map_err(ErrorKind::Safety)?;
        inner
            .backend
            .set_velocity_ned(Velocity::<Ned>::ned(0.0, 0.0, 0.8))
            .await
            .map_err(ErrorKind::Backend)?;
        let tel = inner.backend.tick(0.02).await.map_err(ErrorKind::Backend)?;
        ticks += 1;
        if tel.altitude_agl().get() <= 0.08 {
            break;
        }
        if ticks > 8_000 {
            return Err(ErrorKind::Timeout);
        }
    }
    inner.backend.touchdown_now().map_err(ErrorKind::Backend)?;
    inner.safety = safety::step(inner.safety, Event::Touchdown).map_err(ErrorKind::Safety)?;
    // PX4 / point-mass backends still need a real disarm; world touchdown
    // already cleared the grant (Disarm from Ready is a no-op on the machine).
    inner.backend.disarm().await.map_err(ErrorKind::Backend)?;
    Ok(())
}

fn overlay_safety(
    raw: Result<Telemetry, BackendError>,
    safety: &SafetyState,
) -> Result<Telemetry, ErrorKind> {
    let mut tel = raw.map_err(ErrorKind::Backend)?;
    tel.phase = safety.phase;
    tel.armed = safety.armed;
    tel.actuators_enabled = safety.actuators_enabled;
    tel.offboard = safety.offboard;
    tel.failsafe = safety.failsafe;
    tel.imu_healthy = safety.imu_healthy;
    tel.estimator_valid = safety.estimator_valid;
    Ok(tel)
}

fn error_from_backend(e: BackendError) -> ErrorKind {
    ErrorKind::Backend(e)
}

fn apply_airborne<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<Airborne, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.reached_altitude_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::ReachedAltitude) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_land<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<Landing, B>, TransitionError<S, B>> {
    if let Err(error) = vehicle.require_live_permit() {
        return Err(vehicle.fail(error));
    }
    match vehicle.inner.backend.land_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::Land) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_touchdown<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.touchdown_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::Touchdown) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_failsafe_disarm<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<Recovery, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.disarm_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::Disarm) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_recover<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.recover_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::Recover) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_failsafe<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<Failsafe, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.trigger_failsafe_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::TriggerFailsafe) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

fn apply_disarm<S: State, B: VehicleBackend>(
    mut vehicle: Vehicle<S, B>,
) -> Result<Vehicle<PreflightReady, B>, TransitionError<S, B>> {
    match vehicle.inner.backend.disarm_now() {
        Ok(()) => {
            if let Err(error) = vehicle.apply_event(Event::Disarm) {
                return Err(vehicle.fail(error));
            }
            Ok(vehicle.retarget())
        }
        Err(e) => Err(vehicle.fail(ErrorKind::Backend(e))),
    }
}

/// Which consume-self typestate [`VehicleHandle::from_state`] binds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AerialKind {
    Disconnected,
    Disarmed,
    PreflightReady,
    Armed,
    Offboard,
    Takeoff,
    Airborne,
    Landing,
    Failsafe,
    Recovery,
}

impl AerialKind {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Disconnected => "disconnected",
            Self::Disarmed => "disarmed",
            Self::PreflightReady => "preflight_ready",
            Self::Armed => "armed",
            Self::Offboard => "offboard",
            Self::Takeoff => "takeoff",
            Self::Airborne => "airborne",
            Self::Landing => "landing",
            Self::Failsafe => "failsafe",
            Self::Recovery => "recovery",
        }
    }

    /// Armed through Landing hold motor or offboard authority.
    pub const fn grants_actuation(self) -> bool {
        matches!(
            self,
            Self::Armed | Self::Offboard | Self::Takeoff | Self::Airborne | Self::Landing
        )
    }
}

impl fmt::Display for AerialKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Map a live aerial machine onto the consume-self typestate `attach` uses.
pub fn aerial_kind(safety: SafetyState) -> AerialKind {
    if safety.phase == Phase::Recovery {
        return AerialKind::Recovery;
    }
    if safety.failsafe {
        return AerialKind::Failsafe;
    }
    match safety.phase {
        Phase::Disconnected => AerialKind::Disconnected,
        Phase::Connected | Phase::Initializing | Phase::Preflight => AerialKind::Disarmed,
        Phase::Ready => AerialKind::PreflightReady,
        Phase::Armed if safety.offboard => AerialKind::Offboard,
        Phase::Armed => AerialKind::Armed,
        Phase::Takeoff => AerialKind::Takeoff,
        Phase::Airborne => AerialKind::Airborne,
        Phase::Landing => AerialKind::Landing,
        Phase::Failsafe => AerialKind::Failsafe,
        Phase::Recovery => AerialKind::Recovery,
    }
}

/// Consume-self aerial vehicle bound to a live plant phase.
/// [`Vehicle::new`] always starts [`Disconnected`] and does not read the world.
#[derive(Debug)]
pub enum VehicleHandle<B> {
    Disconnected(Vehicle<Disconnected, B>),
    Disarmed(Vehicle<Disarmed, B>),
    PreflightReady(Vehicle<PreflightReady, B>),
    Armed(Vehicle<Armed, B>),
    Offboard(Vehicle<Offboard, B>),
    Takeoff(Vehicle<Takeoff, B>),
    Airborne(Vehicle<Airborne, B>),
    Landing(Vehicle<Landing, B>),
    Failsafe(Vehicle<Failsafe, B>),
    Recovery(Vehicle<Recovery, B>),
}

impl<B: VehicleBackend> VehicleHandle<B> {
    pub fn from_state(backend: B, safety: SafetyState) -> Self {
        match aerial_kind(safety) {
            AerialKind::Disconnected => Self::Disconnected(wrap(backend, safety)),
            AerialKind::Disarmed => Self::Disarmed(wrap(backend, safety)),
            AerialKind::PreflightReady => Self::PreflightReady(wrap(backend, safety)),
            AerialKind::Armed => Self::Armed(wrap(backend, safety)),
            AerialKind::Offboard => Self::Offboard(wrap(backend, safety)),
            AerialKind::Takeoff => Self::Takeoff(wrap(backend, safety)),
            AerialKind::Airborne => Self::Airborne(wrap(backend, safety)),
            AerialKind::Landing => Self::Landing(wrap(backend, safety)),
            AerialKind::Failsafe => Self::Failsafe(wrap(backend, safety)),
            AerialKind::Recovery => Self::Recovery(wrap(backend, safety)),
        }
    }
}

impl<B> VehicleHandle<B> {
    pub fn kind(&self) -> AerialKind {
        match self {
            Self::Disconnected(_) => AerialKind::Disconnected,
            Self::Disarmed(_) => AerialKind::Disarmed,
            Self::PreflightReady(_) => AerialKind::PreflightReady,
            Self::Armed(_) => AerialKind::Armed,
            Self::Offboard(_) => AerialKind::Offboard,
            Self::Takeoff(_) => AerialKind::Takeoff,
            Self::Airborne(_) => AerialKind::Airborne,
            Self::Landing(_) => AerialKind::Landing,
            Self::Failsafe(_) => AerialKind::Failsafe,
            Self::Recovery(_) => AerialKind::Recovery,
        }
    }

    pub fn safety(&self) -> SafetyState {
        match self {
            Self::Disconnected(v) => v.safety(),
            Self::Disarmed(v) => v.safety(),
            Self::PreflightReady(v) => v.safety(),
            Self::Armed(v) => v.safety(),
            Self::Offboard(v) => v.safety(),
            Self::Takeoff(v) => v.safety(),
            Self::Airborne(v) => v.safety(),
            Self::Landing(v) => v.safety(),
            Self::Failsafe(v) => v.safety(),
            Self::Recovery(v) => v.safety(),
        }
    }

    pub fn backend(&self) -> &B {
        match self {
            Self::Disconnected(v) => v.backend(),
            Self::Disarmed(v) => v.backend(),
            Self::PreflightReady(v) => v.backend(),
            Self::Armed(v) => v.backend(),
            Self::Offboard(v) => v.backend(),
            Self::Takeoff(v) => v.backend(),
            Self::Airborne(v) => v.backend(),
            Self::Landing(v) => v.backend(),
            Self::Failsafe(v) => v.backend(),
            Self::Recovery(v) => v.backend(),
        }
    }

    pub fn backend_mut(&mut self) -> &mut B {
        match self {
            Self::Disconnected(v) => v.backend_mut(),
            Self::Disarmed(v) => v.backend_mut(),
            Self::PreflightReady(v) => v.backend_mut(),
            Self::Armed(v) => v.backend_mut(),
            Self::Offboard(v) => v.backend_mut(),
            Self::Takeoff(v) => v.backend_mut(),
            Self::Airborne(v) => v.backend_mut(),
            Self::Landing(v) => v.backend_mut(),
            Self::Failsafe(v) => v.backend_mut(),
            Self::Recovery(v) => v.backend_mut(),
        }
    }

    pub fn into_backend(self) -> B {
        match self {
            Self::Disconnected(v) => v.into_backend(),
            Self::Disarmed(v) => v.into_backend(),
            Self::PreflightReady(v) => v.into_backend(),
            Self::Armed(v) => v.into_backend(),
            Self::Offboard(v) => v.into_backend(),
            Self::Takeoff(v) => v.into_backend(),
            Self::Airborne(v) => v.into_backend(),
            Self::Landing(v) => v.into_backend(),
            Self::Failsafe(v) => v.into_backend(),
            Self::Recovery(v) => v.into_backend(),
        }
    }
}

fn wrap<S: State, B: VehicleBackend>(backend: B, safety: SafetyState) -> Vehicle<S, B> {
    let permit = if aerial_kind(safety).grants_actuation() {
        Some(super::authority::issue(&backend))
    } else {
        None
    };
    Vehicle {
        inner: Inner {
            backend,
            safety,
            connection_system_id: 0,
            permit,
        },
        _state: PhantomData,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::AerialOffboard;
    use crate::safety::Reject;
    use crate::units::Qty;

    #[test]
    fn generated_now_commands_match_the_dsl_table() {
        assert_eq!(OFFBOARD_NOW_COMMANDS, AerialOffboard::COMMANDS);
        assert_eq!(AerialOffboard::GATE, "OffboardControl");
    }

    #[tokio::test]
    async fn happy_mission_with_null_backend() {
        let v = Vehicle::<Disconnected, NullBackend>::null()
            .connect()
            .await
            .unwrap()
            .verify_preflight()
            .await
            .unwrap()
            .arm()
            .await
            .unwrap();
        assert!(v.safety().armed);
        let mut v = v.enter_offboard().await.unwrap();
        v.set_velocity(Velocity::<Ned>::ned(0.5, 0.0, 0.0))
            .await
            .unwrap();
        assert!(v.safety().offboard);
        let _ = Qty::<Meter>::from_meters(1.0);
    }

    fn ready_safety() -> SafetyState {
        safety::step_all(
            SafetyState::disconnected(),
            &[
                Event::Connect,
                Event::InitComplete,
                Event::Initialized,
                Event::ImuHealthy,
                Event::EstimatorValid,
                Event::PreflightPassed,
            ],
        )
        .unwrap()
    }

    #[test]
    fn from_state_maps_world_ready_to_preflight_ready() {
        let h = VehicleHandle::from_state(NullBackend::default(), ready_safety());
        assert!(matches!(h, VehicleHandle::PreflightReady(_)));
        assert_eq!(h.safety().phase, Phase::Ready);
        assert!(!h.safety().armed);
    }

    #[test]
    fn from_state_maps_takeoff_grant_to_takeoff() {
        let s = safety::step_all(
            ready_safety(),
            &[
                Event::Arm,
                Event::HeartbeatFresh,
                Event::EnterOffboard,
                Event::EnableActuators,
                Event::Takeoff,
            ],
        )
        .unwrap();
        assert_eq!(s.phase, Phase::Takeoff);
        let h = VehicleHandle::from_state(NullBackend::default(), s);
        assert!(matches!(h, VehicleHandle::Takeoff(_)));
        assert!(h.safety().offboard && h.safety().actuators_enabled);
    }

    #[test]
    fn handle_backend_accessors_round_trip() {
        let mut h = VehicleHandle::from_state(NullBackend::default(), ready_safety());
        h.backend_mut().yaw_rad = 0.5;
        assert!((h.backend().yaw_rad - 0.5).abs() < 1e-6);
        let backend = h.into_backend();
        assert!((backend.yaw_rad - 0.5).abs() < 1e-6);
    }

    #[test]
    fn from_state_preserves_every_packed_machine() {
        use crate::safety::{check_invariants, unpack};
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            let h = VehicleHandle::from_state(NullBackend::default(), s);
            assert_eq!(h.safety(), s, "bits={bits}");
            assert_eq!(h.kind(), aerial_kind(s), "bits={bits}");
        }
    }

    #[test]
    fn aerial_kind_maps_ready_takeoff_and_failsafe() {
        assert_eq!(aerial_kind(ready_safety()), AerialKind::PreflightReady);
        let takeoff = safety::step_all(
            ready_safety(),
            &[
                Event::Arm,
                Event::HeartbeatFresh,
                Event::EnterOffboard,
                Event::EnableActuators,
                Event::Takeoff,
            ],
        )
        .unwrap();
        assert_eq!(aerial_kind(takeoff), AerialKind::Takeoff);
        let mut fs = ready_safety();
        fs.failsafe = true;
        assert_eq!(aerial_kind(fs), AerialKind::Failsafe);
        let recovering = safety::step_all(
            ready_safety(),
            &[
                Event::Arm,
                Event::HeartbeatFresh,
                Event::EnterOffboard,
                Event::EnableActuators,
                Event::TriggerFailsafe,
                Event::Disarm,
            ],
        )
        .unwrap();
        assert_eq!(recovering.phase, Phase::Recovery);
        assert!(recovering.failsafe);
        assert_eq!(aerial_kind(recovering), AerialKind::Recovery);
        let h = VehicleHandle::from_state(NullBackend::default(), recovering);
        assert!(matches!(h, VehicleHandle::Recovery(_)));
    }

    #[test]
    fn error_kind_collapses_to_backend() {
        assert_eq!(
            ErrorKind::Backend(BackendError::Io).into_backend(),
            BackendError::Io
        );
        assert_eq!(ErrorKind::Timeout.into_backend(), BackendError::Timeout);
        assert_eq!(
            ErrorKind::Safety(Reject::IllegalPhase).into_backend(),
            BackendError::Rejected("safety")
        );
        assert_eq!(
            ErrorKind::PreflightFailed.into_backend(),
            BackendError::Rejected("preflight")
        );
        let via_from: BackendError = ErrorKind::Timeout.into();
        assert_eq!(via_from, BackendError::Timeout);
        assert_eq!(
            ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch).into_backend(),
            BackendError::Rejected("stale_authority")
        );
    }

    #[test]
    fn arm_now_enters_offboard_without_a_runtime() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let armed = drone.arm_now().unwrap();
        assert!(armed.safety().armed);
        let offboard = armed.enter_offboard_now().unwrap();
        assert!(offboard.safety().offboard && offboard.safety().actuators_enabled);
        let mut climbing = offboard.start_takeoff_now().unwrap();
        assert_eq!(climbing.phase(), Phase::Takeoff);
        climbing
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
            .unwrap();
        assert!(climbing.backend().velocity.is_some());
        climbing
            .set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
            .unwrap();
        assert!(climbing.backend().position.is_some());
        let landing = climbing.begin_land_now().unwrap();
        assert_eq!(landing.phase(), Phase::Landing);
    }

    #[test]
    fn hold_now_tracks_telemetry_pose() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut climbing = drone
            .arm_now()
            .unwrap()
            .enter_offboard_now()
            .unwrap()
            .start_takeoff_now()
            .unwrap();
        climbing.backend_mut().position = Some(Position::<Ned>::ned(1.5, -0.25, -3.0));
        climbing.hold_now().unwrap();
        let p = climbing.backend().position.unwrap();
        assert_eq!((p.x(), p.y(), p.z()), (1.5, -0.25, -3.0));
    }

    #[test]
    fn declare_airborne_now_consumes_takeoff() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let climbing = offboard.start_takeoff_now().unwrap();
        let airborne = climbing.declare_airborne_now().unwrap();
        assert_eq!(airborne.phase(), Phase::Airborne);
        let landing = airborne.begin_land_now().unwrap();
        assert_eq!(landing.phase(), Phase::Landing);
    }

    #[test]
    fn failsafe_now_consumes_offboard() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let fs = offboard.failsafe_now().unwrap();
        assert!(fs.safety().failsafe);
        assert_eq!(fs.phase(), Phase::Failsafe);
    }

    #[test]
    fn failsafe_now_from_ready_and_armed() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let from_ready = drone.failsafe_now().unwrap();
        assert!(from_ready.safety().failsafe);
        assert_eq!(from_ready.phase(), Phase::Failsafe);

        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let armed = drone.arm_now().unwrap();
        let from_armed = armed.failsafe_now().unwrap();
        assert!(from_armed.safety().failsafe);
        assert_eq!(from_armed.phase(), Phase::Failsafe);
    }

    #[test]
    fn begin_land_now_then_touchdown_now_returns_ready() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let climbing = offboard.start_takeoff_now().unwrap();
        let mut landing = climbing.begin_land_now().unwrap();
        assert_eq!(landing.phase(), Phase::Landing);
        landing
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, 0.8))
            .unwrap();
        let ready = landing.touchdown_now().unwrap();
        assert_eq!(ready.phase(), Phase::Ready);
        assert!(!ready.safety().armed);
        assert!(!ready.safety().actuators_enabled);
        let armed = ready.arm_now().unwrap();
        assert!(armed.safety().armed);
        assert_eq!(armed.phase(), Phase::Armed);
    }

    #[test]
    fn disarm_now_from_offboard_returns_ready() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let ready = offboard.disarm_now().unwrap();
        assert_eq!(ready.phase(), Phase::Ready);
        assert!(!ready.safety().armed);
        assert!(!ready.safety().actuators_enabled);
        assert!(!ready.safety().offboard);
        let still = ready.disarm_now().unwrap();
        assert_eq!(still.phase(), Phase::Ready);
    }

    #[test]
    fn failsafe_touchdown_now_returns_ready() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let fs = offboard.failsafe_now().unwrap();
        assert!(fs.safety().failsafe);
        let ready = fs.touchdown_now().unwrap();
        assert_eq!(ready.phase(), Phase::Ready);
        assert!(!ready.safety().failsafe);
        assert!(!ready.safety().armed);
        assert!(!ready.safety().actuators_enabled);
        let armed = ready.arm_now().unwrap();
        assert!(armed.safety().armed);
    }

    #[test]
    fn failsafe_from_ready_then_touchdown_now_returns_ready() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let fs = drone.failsafe_now().unwrap();
        assert!(fs.safety().failsafe);
        let ready = fs.touchdown_now().unwrap();
        assert_eq!(ready.phase(), Phase::Ready);
        assert!(!ready.safety().failsafe);
        assert!(!ready.safety().armed);
    }

    #[test]
    fn failsafe_disarm_now_enters_recovery_then_recover_now_returns_ready() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let fs = offboard.failsafe_now().unwrap();
        let recovering = fs.disarm_now().unwrap();
        assert_eq!(recovering.phase(), Phase::Recovery);
        assert!(recovering.safety().failsafe);
        assert!(!recovering.safety().armed);
        assert!(!recovering.safety().actuators_enabled);
        let ready = recovering.recover_now().unwrap();
        assert_eq!(ready.phase(), Phase::Ready);
        assert!(!ready.safety().failsafe);
        assert!(!ready.safety().armed);
        let armed = ready.arm_now().unwrap();
        assert!(armed.safety().armed);
        assert_eq!(armed.phase(), Phase::Armed);
    }

    #[test]
    fn revoke_makes_offboard_setpoint_stale_while_type_is_still_offboard() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut v = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        v.set_velocity_now(Velocity::<Ned>::ned(1.0, 0.0, 0.0))
            .unwrap();
        v.backend_mut().revoke_authority();
        let err = v
            .set_velocity_now(Velocity::<Ned>::ned(1.0, 0.0, 0.0))
            .unwrap_err();
        assert!(matches!(
            err,
            ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch)
        ));
        assert!(v.safety().offboard);
    }

    #[test]
    fn bounded_lease_expires_when_the_backend_clock_advances() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut v = drone
            .arm_now()
            .unwrap()
            .acquire_offboard_control_now(Duration::from_millis(20))
            .unwrap();
        v.set_velocity_now(Velocity::<Ned>::ned(0.1, 0.0, 0.0))
            .unwrap();
        v.backend_mut().ticks = 3;
        let err = v
            .set_velocity_now(Velocity::<Ned>::ned(0.1, 0.0, 0.0))
            .unwrap_err();
        assert!(matches!(
            err,
            ErrorKind::StaleAuthority(AuthorityReject::Expired)
        ));
    }

    #[test]
    fn stamped_command_older_than_bound_has_no_actuation_authority() {
        use crate::temporal::Command;
        use crate::time::MonotonicInstant;
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut v = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        let fresh = Command::new(Velocity::<Ned>::ned(0.2, 0.0, 0.0), MonotonicInstant::ZERO);
        v.apply_velocity_command_now(fresh).unwrap();
        v.backend_mut().ticks = 10; // 100 ms; command_age_ok is age < 100
        let stale = Command::new(Velocity::<Ned>::ned(0.2, 0.0, 0.0), MonotonicInstant::ZERO);
        let err = v.apply_velocity_command_now(stale).unwrap_err();
        assert!(matches!(
            err,
            ErrorKind::StaleAuthority(AuthorityReject::StaleCommand)
        ));
        assert!(v.safety().offboard);
    }

    #[test]
    fn stale_armed_handle_cannot_enter_offboard() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut armed = drone.arm_now().unwrap();
        assert!(armed.safety().armed);
        armed.backend_mut().revoke_authority();
        let err = armed.enter_offboard_now().unwrap_err();
        assert!(matches!(
            err.error,
            ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch)
        ));
        assert!(err.vehicle.safety().armed);
    }

    #[test]
    fn stale_offboard_handle_cannot_start_takeoff() {
        let VehicleHandle::PreflightReady(drone) =
            VehicleHandle::from_state(NullBackend::default(), ready_safety())
        else {
            panic!("ready maps to PreflightReady");
        };
        let mut offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
        offboard.backend_mut().revoke_authority();
        let err = offboard.start_takeoff_now().unwrap_err();
        assert!(matches!(
            err.error,
            ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch)
        ));
        assert!(err.vehicle.safety().offboard);
    }
}
