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
use crate::frames::Ned;
use crate::safety::{self, Event, Phase, Reject, SafetyState};
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
state!(Airborne, "airborne");
state!(Landing, "landing");
state!(Failsafe, "failsafe");

/// Marker: motor / actuator commands are legal.
pub trait MotorsEnabled: State {}
impl MotorsEnabled for Armed {}
impl MotorsEnabled for Offboard {}
impl MotorsEnabled for Airborne {}
impl MotorsEnabled for Landing {}

/// Marker: velocity / position offboard setpoints are legal.
pub trait OffboardControl: State {}
impl OffboardControl for Offboard {}
impl OffboardControl for Airborne {}

#[derive(Debug)]
pub struct Inner<B> {
    pub backend: B,
    pub safety: SafetyState,
    pub connection_system_id: u8,
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
}

impl fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Safety(r) => write!(f, "safety: {r}"),
            Self::Backend(b) => write!(f, "backend: {b}"),
            Self::PreflightFailed => write!(f, "preflight checks failed"),
            Self::Timeout => write!(f, "timed out waiting for the vehicle"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ErrorKind {}

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

impl<B: VehicleBackend> Vehicle<Disconnected, B> {
    pub fn new(backend: B) -> Self {
        Self {
            inner: Inner {
                backend,
                safety: SafetyState::disconnected(),
                connection_system_id: 0,
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
    pub async fn arm(mut self) -> Result<Vehicle<Armed, B>, TransitionError<PreflightReady, B>> {
        match self.inner.backend.arm().await {
            Ok(()) => {
                if let Err(error) = self.apply_event(Event::Arm) {
                    return Err(self.fail(error));
                }
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
    pub async fn enter_offboard(
        mut self,
    ) -> Result<Vehicle<Offboard, B>, TransitionError<Armed, B>> {
        if let Err(error) = self.apply_event(Event::HeartbeatFresh) {
            return Err(self.fail(error));
        }
        match self.inner.backend.enter_offboard().await {
            Ok(()) => {
                if let Err(error) = self.apply_event(Event::EnterOffboard) {
                    return Err(self.fail(error));
                }
                if let Err(error) = self.apply_event(Event::EnableActuators) {
                    return Err(self.fail(error));
                }
                if let Err(e) = self.inner.backend.enable_actuators().await {
                    return Err(self.fail(ErrorKind::Backend(e)));
                }
                Ok(self.retarget())
            }
            Err(e) => Err(self.fail(error_from_backend(e))),
        }
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
        mut self,
        altitude_agl: Qty<Meter>,
    ) -> Result<Vehicle<Airborne, B>, TransitionError<Offboard, B>> {
        if let Err(error) = self.start_takeoff().await {
            return Err(self.fail(error));
        }

        let target = altitude_agl.get().max(0.3);
        let mut ticks = 0u32;
        loop {
            let climb = Velocity::<Ned>::ned(0.0, 0.0, -1.2);
            if let Err(error) = self.command_velocity(climb).await {
                return Err(self.fail(error));
            }
            match self.inner.backend.tick(0.02).await {
                Ok(tel) => {
                    ticks += 1;
                    if tel.altitude_agl().get() >= target {
                        break;
                    }
                    if ticks > 5_000 {
                        return Err(self.fail(ErrorKind::Timeout));
                    }
                }
                Err(e) => return Err(self.fail(ErrorKind::Backend(e))),
            }
        }
        if let Err(error) = self.apply_event(Event::ReachedAltitude) {
            return Err(self.fail(error));
        }
        let hover = Velocity::<Ned>::ned(0.0, 0.0, 0.0);
        if let Err(error) = self.command_velocity(hover).await {
            return Err(self.fail(error));
        }
        Ok(self.retarget())
    }

    /// Enable actuators and enter the takeoff phase without blocking on altitude.
    pub async fn start_takeoff(&mut self) -> Result<(), ErrorKind> {
        self.apply_event(Event::Takeoff)?;
        self.inner
            .backend
            .enable_actuators()
            .await
            .map_err(ErrorKind::Backend)?;
        Ok(())
    }

    /// Record that the climb completed (`Takeoff → Airborne` in the safety machine).
    pub fn declare_airborne(&mut self) -> Result<(), ErrorKind> {
        self.apply_event(Event::ReachedAltitude)
    }

    pub async fn land(mut self) -> Result<Vehicle<Disarmed, B>, TransitionError<Offboard, B>> {
        match descend_and_disarm(&mut self.inner).await {
            Ok(()) => Ok(self.retarget()),
            Err(error) => Err(self.fail(error)),
        }
    }
}

impl<B: VehicleBackend> Vehicle<Airborne, B> {
    pub async fn land(mut self) -> Result<Vehicle<Disarmed, B>, TransitionError<Airborne, B>> {
        match descend_and_disarm(&mut self.inner).await {
            Ok(()) => Ok(self.retarget()),
            Err(error) => Err(self.fail(error)),
        }
    }
}

impl<S: OffboardControl, B: VehicleBackend> Vehicle<S, B> {
    pub async fn set_velocity(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
        self.command_velocity(velocity).await?;
        self.inner
            .backend
            .tick(0.02)
            .await
            .map_err(ErrorKind::Backend)?;
        Ok(())
    }

    pub async fn set_position(&mut self, position: Position<Ned>) -> Result<(), ErrorKind> {
        self.apply_event(Event::HeartbeatFresh)?;
        self.apply_event(Event::MissionCommand)?;
        self.inner
            .backend
            .set_position_ned(position)
            .await
            .map_err(ErrorKind::Backend)?;
        self.inner
            .backend
            .tick(0.02)
            .await
            .map_err(ErrorKind::Backend)?;
        Ok(())
    }

    pub async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.tick(dt_secs).await, &self.inner.safety)
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }

    pub async fn failsafe(mut self) -> Result<Vehicle<Failsafe, B>, TransitionError<S, B>> {
        if let Err(e) = self.inner.backend.trigger_failsafe().await {
            return Err(self.fail(ErrorKind::Backend(e)));
        }
        if let Err(error) = self.apply_event(Event::TriggerFailsafe) {
            return Err(self.fail(error));
        }
        Ok(self.retarget())
    }

    async fn command_velocity(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
        self.apply_event(Event::HeartbeatFresh)?;
        self.apply_event(Event::MissionCommand)?;
        self.inner
            .backend
            .set_velocity_ned(velocity)
            .await
            .map_err(ErrorKind::Backend)
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
    pub async fn disarm(mut self) -> Result<Vehicle<Disarmed, B>, TransitionError<Failsafe, B>> {
        match self.inner.backend.disarm().await {
            Ok(()) => {
                if let Err(error) = self.apply_event(Event::Disarm) {
                    return Err(self.fail(error));
                }
                let _ = self.apply_event(Event::Recover);
                Ok(self.retarget())
            }
            Err(e) => Err(self.fail(ErrorKind::Backend(e))),
        }
    }

    pub async fn telemetry(&mut self) -> Result<Telemetry, ErrorKind> {
        overlay_safety(self.inner.backend.telemetry().await, &self.inner.safety)
    }
}

async fn descend_and_disarm<B: VehicleBackend>(inner: &mut Inner<B>) -> Result<(), ErrorKind> {
    inner.safety = safety::step(inner.safety, Event::Land).map_err(ErrorKind::Safety)?;
    let mut ticks = 0u32;
    loop {
        inner.safety =
            safety::step(inner.safety, Event::HeartbeatFresh).map_err(ErrorKind::Safety)?;
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
    inner.backend.disarm().await.map_err(ErrorKind::Backend)?;
    inner.safety = safety::step(inner.safety, Event::Touchdown).map_err(ErrorKind::Safety)?;
    Ok(())
}

fn overlay_safety(
    result: Result<Telemetry, BackendError>,
    safety: &SafetyState,
) -> Result<Telemetry, ErrorKind> {
    let mut tel = result.map_err(ErrorKind::Backend)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Qty;

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
}
