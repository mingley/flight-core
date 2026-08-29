use flight_core::marine::{MarineEvent, MarineState};
use flight_core::time::{Clock, MonotonicInstant};
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, CanDock, CanTripMarineFailsafe, ConnectionInfo, MarineVehicle,
    MotorThrust, PreflightReport, Telemetry, VehicleBackend,
};
use robot_world::World;

use super::session::WorldSession;
use super::shared::{
    flush_body, marine_event, preflight_from, require_body, require_body_mut, telemetry_body,
    tick_body, Setpoint,
};

/// Surface or underwater hull in the same verified scene.
#[derive(Clone, Debug)]
pub struct MarineWorldBackend {
    session: WorldSession,
    body_id: &'static str,
    setpoint: Option<Setpoint>,
    yaw_rate: f32,
    last_command: &'static str,
    imu_seq: u32,
}

impl MarineWorldBackend {
    pub fn coastal_skiff(seed: u64) -> Self {
        WorldSession::coastal(seed).marine("skiff")
    }

    pub fn coastal_surveyor(seed: u64) -> Self {
        WorldSession::coastal(seed).marine("surveyor")
    }

    pub fn from_session(session: WorldSession, body_id: &'static str) -> Self {
        Self {
            session,
            body_id,
            setpoint: None,
            yaw_rate: 0.0,
            last_command: "idle",
            imu_seq: 0,
        }
    }

    pub fn session(&self) -> &WorldSession {
        &self.session
    }

    pub fn body_id(&self) -> &'static str {
        self.body_id
    }

    pub fn world(&self) -> World {
        self.session.world()
    }

    /// Snapshot telemetry without stepping the plant.
    pub fn telemetry_now(&mut self) -> Result<Telemetry, BackendError> {
        telemetry_body(
            &self.session,
            self.body_id,
            &mut self.imu_seq,
            self.last_command,
        )
    }

    /// Write the current thrust command onto the hull without stepping the world.
    pub fn flush(&self) -> Result<(), BackendError> {
        flush_body(
            &self.session,
            self.body_id,
            self.setpoint,
            Some(self.yaw_rate),
        )
    }

    /// Docked → Underway. Same walk as [`WorldSession::attach_undock`].
    pub fn grant_undock(&mut self) -> Result<(), BackendError> {
        *self = self.session.attach_undock(self.body_id)?;
        self.last_command = "grant_undock";
        Ok(())
    }

    /// Trip marine failsafe without an async context (HITL miss path).
    ///
    /// Fires kernel `Failsafe` — the same event
    /// [`WorldSession::attach_marine_failsafe`] walks. Already-tripped is
    /// idempotent.
    pub fn failsafe_now(&mut self) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::Failsafe)?;
        self.setpoint = None;
        self.last_command = "failsafe";
        Ok(())
    }

    /// NED velocity without an async context.
    ///
    /// Fires `ThrustCommand`. Docked or failsafe hulls are `Rejected`.
    pub fn set_velocity_now(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::ThrustCommand)?;
        self.setpoint = Some(Setpoint::Velocity(velocity));
        self.last_command = "set_velocity";
        Ok(())
    }

    /// Underway → StationKeep. Thrust remains granted for a hold.
    pub fn station_now(&mut self) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::Station)?;
        self.last_command = "station";
        Ok(())
    }

    /// StationKeep → Underway.
    pub fn resume_now(&mut self) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::Resume)?;
        self.last_command = "resume";
        Ok(())
    }

    /// Underway / station / failsafe → Docked. Clears the handle setpoint
    /// and the body's command so thrust stops with the grant.
    pub fn dock_now(&mut self) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::Dock)?;
        self.setpoint = None;
        self.last_command = "dock";
        Ok(())
    }

    /// Bind a consume-self [`MarineVehicle`] to the live hull without
    /// resetting it to docked. [`MarineVehicle::new`] always starts `Docked`.
    pub fn attach(self) -> Result<flight_core::vehicle::MarineHandle<Self>, BackendError> {
        let safety = {
            let plant = self.session.lock();
            let body = require_body(&plant.world, self.body_id)?;
            body.marine.ok_or(BackendError::Protocol)?
        };
        Ok(flight_core::vehicle::MarineHandle::from_state(self, safety))
    }
}

pub(crate) fn marine_failsafe<S: CanTripMarineFailsafe>(
    v: MarineVehicle<S, MarineWorldBackend>,
) -> MarineWorldBackend {
    v.declare_failsafe().into_backend()
}

pub(crate) fn marine_dock<S: CanDock>(
    v: MarineVehicle<S, MarineWorldBackend>,
) -> MarineWorldBackend {
    v.dock_now().into_backend()
}

impl Clock for MarineWorldBackend {
    fn now(&self) -> MonotonicInstant {
        self.session.lock().clock.now()
    }
}

impl VehicleBackend for MarineWorldBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        let _ = require_body(&self.session.lock().world, self.body_id)?;
        self.last_command = "connect";
        Ok(ConnectionInfo {
            system_id: 3,
            component_id: 1,
            autopilot: AutopilotKind::Simulated,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        self.last_command = "preflight";
        preflight_from(&self.session, self.body_id)
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        Err(BackendError::Rejected(
            "marine platforms use MarineVehicle::undock",
        ))
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        marine_event(&self.session, self.body_id, MarineEvent::Dock)?;
        self.setpoint = None;
        self.last_command = "disarm";
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        Err(BackendError::Rejected("marine has no offboard mode"))
    }

    async fn set_velocity_ned(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.set_velocity_now(velocity)
    }

    async fn set_position_ned(
        &mut self,
        position: Position<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.setpoint = Some(Setpoint::Position(position));
        self.last_command = "set_position";
        Ok(())
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        Err(BackendError::Rejected("marine has no motor mixer"))
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        self.setpoint = None;
        Ok(())
    }

    async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, BackendError> {
        tick_body(
            &self.session,
            self.body_id,
            self.setpoint,
            Some(self.yaw_rate),
            dt_secs,
            &mut self.imu_seq,
            self.last_command,
        )
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        telemetry_body(
            &self.session,
            self.body_id,
            &mut self.imu_seq,
            self.last_command,
        )
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        MarineWorldBackend::failsafe_now(self)
    }

    fn sync_marine(&mut self, safety: MarineState) -> Result<(), BackendError> {
        let mut plant = self.session.lock();
        let body = require_body_mut(&mut plant.world, self.body_id)?;
        if body.marine.is_none() {
            return Err(BackendError::Protocol);
        }
        body.marine = Some(safety);
        if !safety.thrust_enabled {
            body.clear_command();
            self.setpoint = None;
        }
        Ok(())
    }

    fn set_yaw_rate(&mut self, yaw_rate: f32) -> Result<(), BackendError> {
        self.yaw_rate = yaw_rate;
        Ok(())
    }
}
