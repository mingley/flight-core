use flight_core::ground::{GroundEvent, GroundPhase, GroundState};
use flight_core::time::{Clock, MonotonicInstant};
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, CanTripEstop, ConnectionInfo, GroundVehicle, MotorThrust, Moving,
    PreflightReport, Telemetry, VehicleBackend,
};
use robot_world::World;

use super::session::WorldSession;
use super::shared::{
    flush_body, ground_event, preflight_from, require_body, require_body_mut, telemetry_body,
    tick_body, Setpoint,
};

/// Ground chassis in the same verified scene as [`WorldBackend`].
#[derive(Clone, Debug)]
pub struct GroundWorldBackend {
    session: WorldSession,
    body_id: &'static str,
    setpoint: Option<Setpoint>,
    yaw_rate: f32,
    last_command: &'static str,
    imu_seq: u32,
}

impl GroundWorldBackend {
    pub fn inland(seed: u64) -> Self {
        WorldSession::inland(seed).ground("rover")
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

    /// Write the current twist onto the chassis without stepping the world.
    pub fn flush(&self) -> Result<(), BackendError> {
        flush_body(
            &self.session,
            self.body_id,
            self.setpoint,
            Some(self.yaw_rate),
        )
    }

    /// Parked → Moving. Same walk as [`WorldSession::attach_drive`].
    pub fn grant_drive(&mut self) -> Result<(), BackendError> {
        *self = self.session.attach_drive(self.body_id)?;
        self.last_command = "grant_drive";
        Ok(())
    }

    /// Trip chassis E-stop without an async context (HITL miss path).
    ///
    /// Fires kernel `EStop` — the same event [`WorldSession::attach_estop`]
    /// walks. Legal from Parked or Moving; already-tripped is idempotent.
    pub fn failsafe_now(&mut self) -> Result<(), BackendError> {
        ground_event(&self.session, self.body_id, GroundEvent::EStop)?;
        self.setpoint = None;
        self.last_command = "estop";
        Ok(())
    }

    /// NED velocity without an async context.
    ///
    /// Fires `DriveCommand`. Parked or E-stopped chassis are `Rejected`.
    pub fn set_velocity_now(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        ground_event(&self.session, self.body_id, GroundEvent::DriveCommand)?;
        self.setpoint = Some(Setpoint::Velocity(velocity));
        self.last_command = "set_velocity";
        Ok(())
    }

    /// Moving → Parked. Clears the handle setpoint and the body's command.
    pub fn halt_now(&mut self) -> Result<(), BackendError> {
        ground_event(&self.session, self.body_id, GroundEvent::Halt)?;
        self.setpoint = None;
        self.last_command = "halt";
        Ok(())
    }

    /// Bind a consume-self [`GroundVehicle`] to the live chassis without
    /// resetting it to parked. [`GroundVehicle::new`] always starts `Parked`.
    pub fn attach(self) -> Result<flight_core::vehicle::GroundHandle<Self>, BackendError> {
        let safety = {
            let plant = self.session.lock();
            let body = require_body(&plant.world, self.body_id)?;
            body.ground.ok_or(BackendError::Protocol)?
        };
        Ok(flight_core::vehicle::GroundHandle::from_state(self, safety))
    }
}

pub(crate) fn ground_estop<S: CanTripEstop>(
    v: GroundVehicle<S, GroundWorldBackend>,
) -> GroundWorldBackend {
    v.emergency_stop_now().into_backend()
}

pub(crate) fn ground_hold(
    mut v: GroundVehicle<Moving, GroundWorldBackend>,
) -> Result<GroundWorldBackend, BackendError> {
    v.hold_now().map_err(|e| e.into_backend())?;
    let backend = v.into_backend();
    backend.flush()?;
    Ok(backend)
}

impl Clock for GroundWorldBackend {
    fn now(&self) -> MonotonicInstant {
        self.session.lock().clock.now()
    }
}

impl VehicleBackend for GroundWorldBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        let _ = require_body(&self.session.lock().world, self.body_id)?;
        self.last_command = "connect";
        Ok(ConnectionInfo {
            system_id: 2,
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
            "ground platforms use GroundVehicle::enable_drive",
        ))
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        let phase = {
            let plant = self.session.lock();
            let body = require_body(&plant.world, self.body_id)?;
            body.ground.ok_or(BackendError::Protocol)?.phase
        };
        match phase {
            GroundPhase::Moving => {
                ground_event(&self.session, self.body_id, GroundEvent::Halt)?;
            }
            GroundPhase::EStop => {
                ground_event(&self.session, self.body_id, GroundEvent::ClearEstop)?;
            }
            GroundPhase::Parked => {}
        }
        self.setpoint = None;
        self.last_command = "disarm";
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        Err(BackendError::Rejected("ground has no offboard mode"))
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
        Err(BackendError::Rejected("ground has no motor mixer"))
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
        GroundWorldBackend::failsafe_now(self)
    }

    fn sync_ground(&mut self, safety: GroundState) -> Result<(), BackendError> {
        let mut plant = self.session.lock();
        let body = require_body_mut(&mut plant.world, self.body_id)?;
        if body.ground.is_none() {
            return Err(BackendError::Protocol);
        }
        body.ground = Some(safety);
        if !safety.drive_enabled {
            body.clear_command();
            self.setpoint = None;
        }
        Ok(())
    }

    fn set_yaw_rate(&mut self, yaw_rate: f32) -> Result<(), BackendError> {
        self.yaw_rate = yaw_rate;
        Ok(())
    }

    fn halt_now(&mut self) -> Result<(), BackendError> {
        GroundWorldBackend::halt_now(self)
    }
}
