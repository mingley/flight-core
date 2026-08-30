use std::sync::{Arc, Mutex};

use flight_core::frames::Body as BodyFrame;
use flight_core::sensors::{Imu, ImuSample, SensorError};
use flight_core::time::{Clock, Duration, VirtualClock};
use flight_core::vehicle::{BackendError, GroundHandle, MarineHandle, VehicleHandle};
use robot_world::World;

use super::aerial::{
    aerial_disarm, aerial_failsafe, aerial_hold, aerial_land, aerial_touchdown, WorldBackend,
};
use super::ground::{ground_estop, ground_hold, GroundWorldBackend};
use super::marine::{marine_dock, marine_failsafe, marine_hold, MarineWorldBackend};
use super::shared::{body_imu, clamp_dt, Plant};

/// Shared verified scene. Clone to put several typestate vehicles in one world.
#[derive(Clone, Debug)]
pub struct WorldSession {
    inner: Arc<Mutex<Plant>>,
}
impl WorldSession {
    pub fn from_world(world: World) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Plant {
                world,
                clock: VirtualClock::new(),
            })),
        }
    }

    pub fn coastal(seed: u64) -> Self {
        Self::from_world(World::coastal(seed))
    }

    pub fn inland(seed: u64) -> Self {
        Self::from_world(World::inland(seed))
    }

    pub fn harbor(seed: u64) -> Self {
        Self::from_world(World::harbor(seed))
    }

    pub fn open_water(seed: u64) -> Self {
        Self::from_world(World::open_water(seed))
    }

    pub fn named(name: &str, seed: u64) -> Option<Self> {
        World::named(name, seed).map(Self::from_world)
    }

    pub fn aerial(&self, body_id: &'static str) -> WorldBackend {
        WorldBackend::from_session(self.clone(), body_id)
    }

    pub fn ground(&self, body_id: &'static str) -> GroundWorldBackend {
        GroundWorldBackend::from_session(self.clone(), body_id)
    }

    pub fn marine(&self, body_id: &'static str) -> MarineWorldBackend {
        MarineWorldBackend::from_session(self.clone(), body_id)
    }

    /// Bind Ready, walk arm → offboard, return the live aerial backend.
    ///
    /// Actuators are granted without firing Takeoff, so `Land` is not yet
    /// legal. [`Self::attach_takeoff`] walks the extra Takeoff event the HITL
    /// rack and ROS 2 plants need.
    pub fn attach_offboard(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::PreflightReady(ready) => {
                let offboard = ready
                    .arm_now()
                    .map_err(|e| e.error.into_backend())?
                    .enter_offboard_now()
                    .map_err(|e| e.error.into_backend())?;
                Ok(offboard.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Ready, walk arm → offboard → takeoff, return the live aerial backend.
    ///
    /// [`WorldBackend::grant_offboard`] is this walk. A drone that is not Ready
    /// is [`BackendError::Protocol`].
    pub fn attach_takeoff(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::PreflightReady(ready) => {
                let takeoff = ready
                    .arm_now()
                    .map_err(|e| e.error.into_backend())?
                    .enter_offboard_now()
                    .map_err(|e| e.error.into_backend())?
                    .start_takeoff_now()
                    .map_err(|e| e.error.into_backend())?;
                Ok(takeoff.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Offboard and fire Takeoff. Ready is [`BackendError::Protocol`] —
    /// use [`Self::attach_takeoff`] for the full grant. PX4 `NAV_TAKEOFF`
    /// after ARM is this walk.
    pub fn attach_start_takeoff(
        &self,
        body_id: &'static str,
    ) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Offboard(offboard) => {
                let takeoff = offboard
                    .start_takeoff_now()
                    .map_err(|e| e.error.into_backend())?;
                Ok(takeoff.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Parked and enable drive. A chassis that is not Parked is
    /// [`BackendError::Protocol`].
    pub fn attach_drive(&self, body_id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        match self.ground(body_id).attach()? {
            GroundHandle::Parked(parked) => {
                let moving = parked.enable_drive().map_err(|e| e.into_backend())?;
                Ok(moving.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Hold at the current NED pose while the chassis is Moving.
    /// Parked / EStop are [`BackendError::Protocol`].
    pub fn attach_ground_hold(
        &self,
        body_id: &'static str,
    ) -> Result<GroundWorldBackend, BackendError> {
        match self.ground(body_id).attach()? {
            GroundHandle::Moving(v) => ground_hold(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Moving and halt to Parked. Clears the live command.
    pub fn attach_park(&self, body_id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        match self.ground(body_id).attach()? {
            GroundHandle::Moving(moving) => Ok(moving.park_now().into_backend()),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Parked or Moving and trip E-stop.
    pub fn attach_estop(&self, body_id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        match self.ground(body_id).attach()? {
            GroundHandle::Parked(v) => Ok(ground_estop(v)),
            GroundHandle::Moving(v) => Ok(ground_estop(v)),
            GroundHandle::EStopped(_) => Err(BackendError::Protocol),
        }
    }

    /// Bind Docked and undock. A hull that is not Docked is
    /// [`BackendError::Protocol`].
    pub fn attach_undock(&self, body_id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Docked(docked) => {
                let underway = docked.undock().map_err(|e| e.into_backend())?;
                Ok(underway.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Underway and hold station.
    pub fn attach_station(
        &self,
        body_id: &'static str,
    ) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Underway(underway) => {
                let station = underway.hold_station().map_err(|e| e.into_backend())?;
                Ok(station.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Hold at the current NED pose while the hull is Underway or StationKeep.
    /// Distinct from [`Self::attach_station`]. Docked / Failsafe are Protocol.
    pub fn attach_marine_hold(
        &self,
        body_id: &'static str,
    ) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Underway(v) => marine_hold(v),
            MarineHandle::StationKeep(v) => marine_hold(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind StationKeep and resume making way.
    pub fn attach_resume(&self, body_id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::StationKeep(station) => {
                let underway = station.resume().map_err(|e| e.into_backend())?;
                Ok(underway.into_backend())
            }
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Underway or StationKeep and dock. Docked is [`BackendError::Protocol`].
    pub fn attach_dock(&self, body_id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Underway(v) => Ok(marine_dock(v)),
            MarineHandle::StationKeep(v) => Ok(marine_dock(v)),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Takeoff or Airborne and enter landing. Offboard without Takeoff
    /// is [`BackendError::Protocol`] — same rule as [`Vehicle::begin_land_now`].
    pub fn attach_land(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Takeoff(v) => aerial_land(v),
            VehicleHandle::Airborne(v) => aerial_land(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Landing or Failsafe and touch down to Ready. Clears the live
    /// command. Same kernel `Touchdown` from either phase.
    pub fn attach_touchdown(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Landing(v) => aerial_touchdown(v),
            VehicleHandle::Failsafe(v) => aerial_touchdown(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Takeoff and declare airborne. A drone that is not climbing is
    /// [`BackendError::Protocol`].
    pub fn attach_airborne(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Takeoff(v) => Ok(v
                .declare_airborne_now()
                .map_err(|e| e.error.into_backend())?
                .into_backend()),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Hold at the current NED pose while attach is OffboardControl.
    /// Ready / Armed / Failsafe / Recovery are [`BackendError::Protocol`].
    pub fn attach_hold(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Offboard(v) => aerial_hold(v),
            VehicleHandle::Takeoff(v) => aerial_hold(v),
            VehicleHandle::Airborne(v) => aerial_hold(v),
            VehicleHandle::Landing(v) => aerial_hold(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Ready, Armed, Offboard, Takeoff, Airborne, or Landing and trip
    /// aerial failsafe. Already-failsafe is [`BackendError::Protocol`].
    pub fn attach_failsafe(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::PreflightReady(v) => aerial_failsafe(v),
            VehicleHandle::Armed(v) => aerial_failsafe(v),
            VehicleHandle::Offboard(v) => aerial_failsafe(v),
            VehicleHandle::Takeoff(v) => aerial_failsafe(v),
            VehicleHandle::Airborne(v) => aerial_failsafe(v),
            VehicleHandle::Landing(v) => aerial_failsafe(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Ready, Armed, Offboard, Takeoff, Airborne, or Landing and disarm
    /// to Ready. Failsafe is [`BackendError::Protocol`].
    pub fn attach_disarm(&self, body_id: &'static str) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::PreflightReady(v) => aerial_disarm(v),
            VehicleHandle::Armed(v) => aerial_disarm(v),
            VehicleHandle::Offboard(v) => aerial_disarm(v),
            VehicleHandle::Takeoff(v) => aerial_disarm(v),
            VehicleHandle::Airborne(v) => aerial_disarm(v),
            VehicleHandle::Landing(v) => aerial_disarm(v),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind E-stop and clear back to Parked.
    pub fn attach_reset(&self, body_id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        match self.ground(body_id).attach()? {
            GroundHandle::EStopped(stopped) => Ok(stopped
                .reset()
                .map_err(|e| e.into_backend())?
                .into_backend()),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind Underway or StationKeep and trip marine failsafe.
    pub fn attach_marine_failsafe(
        &self,
        body_id: &'static str,
    ) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Underway(v) => Ok(marine_failsafe(v)),
            MarineHandle::StationKeep(v) => Ok(marine_failsafe(v)),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind marine Failsafe and recover docked.
    pub fn attach_recover(
        &self,
        body_id: &'static str,
    ) -> Result<MarineWorldBackend, BackendError> {
        match self.marine(body_id).attach()? {
            MarineHandle::Failsafe(fs) => Ok(fs
                .recover_docked()
                .map_err(|e| e.into_backend())?
                .into_backend()),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Bind aerial Failsafe (disarm then recover) or Recovery and return Ready.
    /// Distinct from marine [`Self::attach_recover`].
    pub fn attach_recover_ready(
        &self,
        body_id: &'static str,
    ) -> Result<WorldBackend, BackendError> {
        match self.aerial(body_id).attach()? {
            VehicleHandle::Failsafe(v) => Ok(v
                .disarm_now()
                .map_err(|e| e.error.into_backend())?
                .recover_now()
                .map_err(|e| e.error.into_backend())?
                .into_backend()),
            VehicleHandle::Recovery(v) => Ok(v
                .recover_now()
                .map_err(|e| e.error.into_backend())?
                .into_backend()),
            _ => Err(BackendError::Protocol),
        }
    }

    /// Snapshot of the plant. Cheap: a handful of rigid bodies.
    pub fn world(&self) -> World {
        self.with_world(World::clone)
    }

    pub fn with_world<R>(&self, f: impl FnOnce(&World) -> R) -> R {
        f(&self.lock().world)
    }

    pub fn with_world_mut<R>(&self, f: impl FnOnce(&mut World) -> R) -> R {
        f(&mut self.lock().world)
    }

    /// One verified step. Property failure is `BackendError::Rejected` and
    /// the plant stays at the previous legal snapshot.
    pub fn step(&self, dt_secs: f32) -> Result<(), BackendError> {
        let dt = clamp_dt(dt_secs);
        let mut plant = self.lock();
        if plant.world.try_step(dt).is_err() {
            return Err(BackendError::Rejected("property violation"));
        }
        plant.clock.advance(Duration::from_secs_f32(dt));
        Ok(())
    }

    /// IMU that reads this body's plant specific force and rate without
    /// stepping. Wrap in [`crate::FuzzedImu`] so a controller can observe
    /// noisy samples while [`Self::step`] stays the verified physics.
    pub fn imu(&self, body_id: &'static str) -> WorldImu {
        WorldImu::new(self.clone(), body_id)
    }

    pub(crate) fn lock(&self) -> std::sync::MutexGuard<'_, Plant> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Plant IMU for one [`WorldSession`] body. Sampling never calls
/// [`WorldSession::step`].
#[derive(Clone, Debug)]
pub struct WorldImu {
    session: WorldSession,
    body_id: &'static str,
    seq: u32,
}

impl WorldImu {
    pub fn new(session: WorldSession, body_id: &'static str) -> Self {
        Self {
            session,
            body_id,
            seq: 0,
        }
    }
}

impl Imu for WorldImu {
    type Frame = BodyFrame;

    fn sample(&mut self) -> Result<ImuSample<BodyFrame>, SensorError> {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        let plant = self.session.lock();
        let body = plant
            .world
            .body(self.body_id)
            .ok_or(SensorError::Hardware)?;
        Ok(body_imu(body, plant.clock.now(), seq))
    }
}
