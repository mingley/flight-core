use flight_core::safety::Event;
use flight_core::time::{Clock, MonotonicInstant};
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, CanBeginLand, CanDisarm, CanTouchdown, CanTripFailsafe,
    ConnectionInfo, MotorThrust, OffboardControl, PreflightReport, Telemetry, Vehicle,
    VehicleBackend,
};
use robot_world::World;

use super::session::WorldSession;
use super::shared::{
    aerial_event, flush_body, preflight_from, require_body, telemetry_body, tick_body, Setpoint,
};

/// Aerial vehicle whose physics is one body in a mechanically verified world.
#[derive(Clone, Debug)]
pub struct WorldBackend {
    session: WorldSession,
    body_id: &'static str,
    setpoint: Option<Setpoint>,
    last_command: &'static str,
    imu_seq: u32,
}

impl WorldBackend {
    pub fn coastal(seed: u64) -> Self {
        WorldSession::coastal(seed).aerial("drone")
    }

    pub fn inland(seed: u64) -> Self {
        WorldSession::inland(seed).aerial("drone")
    }

    /// Harbor shoreline: same four-body mix as coastal, tighter basin.
    pub fn harbor(seed: u64) -> Self {
        WorldSession::harbor(seed).aerial("drone")
    }

    /// Open water: drone over swell. No rover in the scene.
    pub fn open_water(seed: u64) -> Self {
        WorldSession::open_water(seed).aerial("drone")
    }

    pub fn new(world: World, drone_id: &'static str) -> Self {
        WorldSession::from_world(world).aerial(drone_id)
    }

    pub fn from_session(session: WorldSession, body_id: &'static str) -> Self {
        Self {
            session,
            body_id,
            setpoint: None,
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

    pub fn with_world_mut<R>(&self, f: impl FnOnce(&mut World) -> R) -> R {
        self.session.with_world_mut(f)
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

    /// Write the current setpoint onto the body without stepping the world.
    pub fn flush(&self) -> Result<(), BackendError> {
        flush_body(&self.session, self.body_id, self.setpoint, None)
    }

    /// Arm, offboard, and takeoff. Same walk as [`WorldSession::attach_takeoff`].
    ///
    /// Clears the handle setpoint so a later flush cannot revive a hover
    /// that `enter_offboard_now` staged. The plant is Takeoff; `Land` is legal.
    pub fn grant_offboard(&mut self) -> Result<(), BackendError> {
        *self = self.session.attach_takeoff(self.body_id)?;
        self.setpoint = None;
        self.last_command = "grant_offboard";
        Ok(())
    }

    /// Trip aerial failsafe without an async context (HITL miss path).
    pub fn failsafe_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::TriggerFailsafe)?;
        self.setpoint = None;
        self.last_command = "failsafe";
        Ok(())
    }

    /// NED velocity setpoint without an async context.
    ///
    /// Fires the same `HeartbeatFresh` + `MissionCommand` pair JSON `velocity`
    /// uses. Disarmed, failsafe, or actuator-disabled vehicles are `Rejected`.
    pub fn set_velocity_now(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.accept_mission(Setpoint::Velocity(velocity), "set_velocity")
    }

    /// NED position setpoint without an async context.
    ///
    /// Same grant gate as [`Self::set_velocity_now`].
    pub fn set_position_now(
        &mut self,
        position: Position<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.accept_mission(Setpoint::Position(position), "set_position")
    }

    /// Armed → takeoff. Same grant `Vehicle::start_takeoff_now` writes.
    pub fn takeoff_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::Takeoff)?;
        self.last_command = "takeoff";
        Ok(())
    }

    /// Takeoff → airborne. Same grant `Vehicle::declare_airborne_now` writes.
    pub fn reached_altitude_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::ReachedAltitude)?;
        self.last_command = "airborne";
        Ok(())
    }

    /// Takeoff or airborne → landing. Leaves the current setpoint so a
    /// descent velocity can be flushed on the same tick.
    pub fn land_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::Land)?;
        self.last_command = "land";
        Ok(())
    }

    /// Landing (or failsafe) → Ready. Clears the handle setpoint and the
    /// body's command so rotors do not keep thrusting after touchdown.
    pub fn touchdown_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::Touchdown)?;
        self.setpoint = None;
        self.last_command = "touchdown";
        Ok(())
    }

    /// Recovery → Ready. Same grant `Vehicle::recover_now` writes.
    pub fn recover_now(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::Recover)?;
        self.setpoint = None;
        self.last_command = "recover";
        Ok(())
    }

    /// One verified world step. Same work as [`VehicleBackend::tick`].
    pub fn step_now(&mut self, dt_secs: f32) -> Result<Telemetry, BackendError> {
        tick_body(
            &self.session,
            self.body_id,
            self.setpoint,
            None,
            dt_secs,
            &mut self.imu_seq,
            self.last_command,
        )
    }

    fn drone_event(&mut self, e: Event) -> Result<(), BackendError> {
        aerial_event(&self.session, self.body_id, e)
    }

    fn accept_mission(
        &mut self,
        setpoint: Setpoint,
        last: &'static str,
    ) -> Result<(), BackendError> {
        self.drone_event(Event::HeartbeatFresh)?;
        self.drone_event(Event::MissionCommand)?;
        self.setpoint = Some(setpoint);
        self.last_command = last;
        Ok(())
    }

    /// Bind a consume-self [`Vehicle`] to the live aerial machine without
    /// walking [`Vehicle::new`]'s disconnected connect path.
    pub fn attach(self) -> Result<flight_core::vehicle::VehicleHandle<Self>, BackendError> {
        let safety = {
            let plant = self.session.lock();
            let body = require_body(&plant.world, self.body_id)?;
            body.aerial.ok_or(BackendError::Protocol)?
        };
        Ok(flight_core::vehicle::VehicleHandle::from_state(
            self, safety,
        ))
    }
}

pub(crate) fn aerial_failsafe<S: CanTripFailsafe>(
    v: Vehicle<S, WorldBackend>,
) -> Result<WorldBackend, BackendError> {
    Ok(v.failsafe_now()
        .map_err(|e| e.error.into_backend())?
        .into_backend())
}

pub(crate) fn aerial_touchdown<S: CanTouchdown>(
    v: Vehicle<S, WorldBackend>,
) -> Result<WorldBackend, BackendError> {
    Ok(v.touchdown_now()
        .map_err(|e| e.error.into_backend())?
        .into_backend())
}

pub(crate) fn aerial_land<S: CanBeginLand>(
    v: Vehicle<S, WorldBackend>,
) -> Result<WorldBackend, BackendError> {
    Ok(v.begin_land_now()
        .map_err(|e| e.error.into_backend())?
        .into_backend())
}

pub(crate) fn aerial_hold<S: OffboardControl>(
    mut v: Vehicle<S, WorldBackend>,
) -> Result<WorldBackend, BackendError> {
    v.hold_now().map_err(|e| e.into_backend())?;
    let backend = v.into_backend();
    backend.flush()?;
    Ok(backend)
}
pub(crate) fn aerial_disarm<S: CanDisarm>(
    v: Vehicle<S, WorldBackend>,
) -> Result<WorldBackend, BackendError> {
    Ok(v.disarm_now()
        .map_err(|e| e.error.into_backend())?
        .into_backend())
}

impl Clock for WorldBackend {
    fn now(&self) -> MonotonicInstant {
        self.session.lock().clock.now()
    }
}

impl VehicleBackend for WorldBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        let _ = require_body(&self.session.lock().world, self.body_id)?;
        self.last_command = "connect";
        Ok(ConnectionInfo {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotKind::Simulated,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        self.last_command = "preflight";
        preflight_from(&self.session, self.body_id)
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::Arm)?;
        self.last_command = "arm";
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        let _ = self.drone_event(Event::Disarm);
        self.session.with_world_mut(|w| {
            if let Some(b) = w.body_mut(self.body_id) {
                b.clear_command();
            }
        });
        self.setpoint = None;
        self.last_command = "disarm";
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::HeartbeatFresh)?;
        self.drone_event(Event::EnterOffboard)?;
        self.setpoint = Some(Setpoint::Velocity(Velocity::ned(0.0, 0.0, 0.0)));
        self.last_command = "offboard";
        Ok(())
    }

    async fn set_velocity_ned(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.accept_mission(Setpoint::Velocity(velocity), "set_velocity")
    }

    async fn set_position_ned(
        &mut self,
        position: Position<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.accept_mission(Setpoint::Position(position), "set_position")
    }

    async fn set_motor_thrust(&mut self, thrust: MotorThrust) -> Result<(), BackendError> {
        let _ = thrust;
        self.last_command = "motor_thrust";
        Ok(())
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        self.drone_event(Event::EnableActuators)?;
        self.last_command = "enable_actuators";
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        let _ = self.drone_event(Event::Disarm);
        self.setpoint = None;
        self.last_command = "disable_actuators";
        Ok(())
    }

    async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, BackendError> {
        tick_body(
            &self.session,
            self.body_id,
            self.setpoint,
            None,
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
        self.drone_event(Event::TriggerFailsafe)?;
        self.setpoint = None;
        self.last_command = "failsafe";
        Ok(())
    }

    fn takeoff_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::takeoff_now(self)
    }

    fn reached_altitude_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::reached_altitude_now(self)
    }

    fn land_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::land_now(self)
    }

    fn touchdown_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::touchdown_now(self)
    }

    fn recover_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::recover_now(self)
    }

    fn trigger_failsafe_now(&mut self) -> Result<(), BackendError> {
        WorldBackend::failsafe_now(self)
    }
}
