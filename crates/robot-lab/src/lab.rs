use flight_core::domain::Domain;
use flight_core::vehicle::{
    aerial_kind, BackendError, GroundHandle, MarineHandle, VehicleBackend, VehicleHandle,
};
use robot_world::World;
use serde::{Deserialize, Serialize};

use crate::apply::apply_action_world;
use crate::cmd::LabCmd;
use crate::observe::Observation;
use crate::script::script_tick;
use crate::{AerialKind, GroundWorldBackend, MarineWorldBackend, WorldBackend, WorldSession};

/// Research lab over a named verified world.
///
/// [`Lab::coastal`] / [`Lab::harbor`] / [`Lab::inland`] / [`Lab::open_water`]
/// match the HITL / ROS 2 / PX4 catalogs. [`Lab::open`] is the same by name.
///
/// Clone snapshots the plant. Handles from [`Lab::session`] share the live Mutex.
#[derive(Debug)]
pub struct Lab {
    session: WorldSession,
    pub message: String,
    /// Successful `act` calls, timestamped, for replay.
    pub log: Vec<TimedAction>,
    /// Last failed `act` / `act_through_attach` (NEXT A4). Cleared on success.
    pub(crate) reject_trace: Option<crate::RejectTrace>,
}

impl Clone for Lab {
    fn clone(&self) -> Self {
        Self {
            session: WorldSession::from_world(self.world()),
            message: self.message.clone(),
            log: self.log.clone(),
            reject_trace: self.reject_trace.clone(),
        }
    }
}

impl Lab {
    pub fn coastal(seed: u64) -> Self {
        Self::from_world(World::coastal(seed))
    }

    /// Harbor shoreline: drone, rover, skiff, surveyor on a tighter basin.
    pub fn harbor(seed: u64) -> Self {
        Self::from_world(World::harbor(seed))
    }

    /// Inland air + ground. No hull in the scene.
    pub fn inland(seed: u64) -> Self {
        Self::from_world(World::inland(seed))
    }

    /// Open water: drone + skiff + surveyor. No rover in the scene.
    pub fn open_water(seed: u64) -> Self {
        Self::from_world(World::open_water(seed))
    }

    pub fn open(name: &str, seed: u64) -> Result<Self, LabError> {
        let world =
            World::named(name, seed).ok_or_else(|| LabError::UnknownScenario(name.into()))?;
        Ok(Self::from_world(world))
    }

    pub fn scenarios() -> &'static [&'static str] {
        World::SCENARIOS
    }

    fn from_world(world: World) -> Self {
        Self {
            message: format!("{} world ready", world.scenario),
            session: WorldSession::from_world(world),
            log: Vec::new(),
            reject_trace: None,
        }
    }

    /// Live plant. Typestate `Vehicle` / `GroundVehicle` / `MarineVehicle`
    /// handles from this session step the same scene `act` mutates.
    pub fn session(&self) -> &WorldSession {
        &self.session
    }

    pub fn aerial(&self, id: &'static str) -> WorldBackend {
        self.session.aerial(id)
    }

    pub fn ground(&self, id: &'static str) -> GroundWorldBackend {
        self.session.ground(id)
    }

    pub fn marine(&self, id: &'static str) -> MarineWorldBackend {
        self.session.marine(id)
    }

    /// Consume-self aerial typestate on the live machine (world drones start Ready).
    pub fn aerial_vehicle(
        &self,
        id: &'static str,
    ) -> Result<VehicleHandle<WorldBackend>, BackendError> {
        self.aerial(id).attach()
    }

    /// Consume-self ground typestate on the live chassis.
    pub fn ground_vehicle(
        &self,
        id: &'static str,
    ) -> Result<GroundHandle<GroundWorldBackend>, BackendError> {
        self.ground(id).attach()
    }

    /// Consume-self marine typestate on the live hull.
    pub fn marine_vehicle(
        &self,
        id: &'static str,
    ) -> Result<MarineHandle<MarineWorldBackend>, BackendError> {
        self.marine(id).attach()
    }

    /// Ready → arm → offboard → takeoff. Same grant HITL and ROS 2 plants use.
    pub fn attach_takeoff(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_takeoff(id)
    }

    /// Ready → arm → offboard without Takeoff (PX4 ARM).
    pub fn attach_offboard(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_offboard(id)
    }

    /// Offboard → Takeoff. Same walk PX4 `NAV_TAKEOFF` uses after ARM.
    pub fn attach_start_takeoff(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_start_takeoff(id)
    }

    /// Parked → Moving.
    pub fn attach_drive(&self, id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        self.session.attach_drive(id)
    }

    /// Docked → Underway.
    pub fn attach_undock(&self, id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_undock(id)
    }

    /// Takeoff or Airborne → Landing.
    pub fn attach_land(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_land(id)
    }

    /// Landing or Failsafe → Ready.
    pub fn attach_touchdown(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_touchdown(id)
    }

    /// Moving → Parked.
    pub fn attach_park(&self, id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        self.session.attach_park(id)
    }

    /// Parked or Moving → E-stop.
    pub fn attach_estop(&self, id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        self.session.attach_estop(id)
    }

    /// Underway → StationKeep.
    pub fn attach_station(&self, id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_station(id)
    }

    /// StationKeep → Underway.
    pub fn attach_resume(&self, id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_resume(id)
    }

    /// Underway or StationKeep → Docked.
    pub fn attach_dock(&self, id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_dock(id)
    }

    /// Takeoff → Airborne.
    pub fn attach_airborne(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_airborne(id)
    }

    /// OffboardControl → current NED pose hold.
    pub fn attach_hold(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_hold(id)
    }

    /// Ready, Armed, Offboard, Takeoff, Airborne, or Landing → Failsafe.
    pub fn attach_failsafe(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_failsafe(id)
    }

    /// E-stop → Parked.
    pub fn attach_reset(&self, id: &'static str) -> Result<GroundWorldBackend, BackendError> {
        self.session.attach_reset(id)
    }

    /// Underway or StationKeep → marine Failsafe.
    pub fn attach_marine_failsafe(
        &self,
        id: &'static str,
    ) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_marine_failsafe(id)
    }

    /// Marine Failsafe → Docked.
    pub fn attach_recover(&self, id: &'static str) -> Result<MarineWorldBackend, BackendError> {
        self.session.attach_recover(id)
    }

    /// Aerial Failsafe → Recovery → Ready, or Recovery → Ready.
    pub fn attach_recover_ready(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_recover_ready(id)
    }

    /// Ready / Armed / Offboard / Takeoff / Airborne / Landing → Ready.
    pub fn attach_disarm(&self, id: &'static str) -> Result<WorldBackend, BackendError> {
        self.session.attach_disarm(id)
    }

    /// Snapshot of the plant. Clone of `Lab` also snapshots so the copy is independent.
    pub fn world(&self) -> World {
        self.session.world()
    }

    pub fn with_world<R>(&self, f: impl FnOnce(&World) -> R) -> R {
        self.session.with_world(f)
    }

    pub fn with_world_mut<R>(&self, f: impl FnOnce(&mut World) -> R) -> R {
        self.session.with_world_mut(f)
    }

    pub fn observe(&self) -> Observation {
        Observation::from_lab(self)
    }

    /// NEXT A1: callable `(robot, cmd)` tools plus `env_cmds` for this snapshot.
    pub fn legal_tools(&self) -> crate::LegalTools {
        self.observe().tools()
    }

    /// One JSON object per line — the record an agent or replay tool stores.
    pub fn write_jsonl<W: std::io::Write>(&self, mut w: W) -> std::io::Result<()> {
        serde_json::to_writer(&mut w, &self.observe())?;
        w.write_all(b"\n")
    }

    pub fn all_hold(&self) -> bool {
        self.with_world(World::all_hold)
    }

    /// Property ids that failed on the last `try_step` (vector order). Empty
    /// when [`Self::all_hold`]. Same list as [`Observation::broken`].
    pub fn broken(&self) -> Vec<String> {
        self.with_world(|w| {
            w.last_properties
                .iter()
                .filter(|p| !p.holds)
                .map(|p| p.id.to_string())
                .collect()
        })
    }

    pub fn step(&mut self, dt: f32) {
        if self.session.step(dt).is_err() {
            let ids = self.broken();
            self.message = format!("PROPERTY VIOLATION: {}", ids.join(", "));
        }
    }

    /// Scripted coastal policy used by the live demo and the example.
    ///
    /// Walks the same attach helpers and consume-self now-APIs as
    /// [`Self::act_through_attach`]. Velocity ticks are not logged. Failsafe
    /// and Recovery are left alone (the demo failsafe button stops this
    /// policy; recover is an explicit act).
    pub fn apply_script(&mut self) {
        script_tick(self);
    }

    pub fn act(&mut self, action: AgentAction) -> Result<(), LabError> {
        if let Err(e) = self.ensure_tool(&action) {
            return Err(self.note_reject(&action, e));
        }
        let t = self.with_world(|w| w.t);
        if let Err(e) = self.apply_action(&action) {
            return Err(self.note_reject(&action, e));
        }
        self.clear_reject();
        self.log.push(TimedAction { t, action });
        Ok(())
    }

    /// Walk consume-self attach helpers and now-APIs, then log the intent.
    ///
    /// Grants that are several kernel events (`Takeoff` from Ready, aerial
    /// recover from Failsafe) expand the log so [`Self::replay_until`] stays
    /// faithful. A command attach rejects as [`BackendError::Protocol`] falls
    /// back to JSON `act`. Replay walks the same helpers without pushing to
    /// [`Self::log`].
    pub fn act_through_attach(&mut self, action: AgentAction) -> Result<(), LabError> {
        if let Err(e) = self.ensure_tool_or_attach_grant(&action) {
            return Err(self.note_reject(&action, e));
        }
        let t = self.with_world(|w| w.t);
        match self.try_attach(t, &action, true) {
            Ok(true) => {
                self.clear_reject();
                Ok(())
            }
            Ok(false) => {
                if let Err(e) = self.apply_action(&action) {
                    return Err(self.note_reject(&action, e));
                }
                self.clear_reject();
                self.log.push(TimedAction { t, action });
                Ok(())
            }
            Err(e) => Err(self.note_reject(&action, e)),
        }
    }

    fn try_attach(&mut self, t: f32, action: &AgentAction, log: bool) -> Result<bool, LabError> {
        let Some(id) = intern_robot(&action.robot) else {
            return Ok(false);
        };
        match action.cmd {
            LabCmd::SetWind | LabCmd::SetWaves | LabCmd::SetCurrent | LabCmd::SetCharge => {
                Ok(false)
            }
            LabCmd::EnableActuators => self.try_enable_actuators(t, id, action, log),
            LabCmd::Takeoff => self.try_attach_takeoff(t, id, action, log),
            LabCmd::Airborne => self.attach_apply(t, action, self.session.attach_airborne(id), log),
            LabCmd::Land => self.attach_apply(t, action, self.session.attach_land(id), log),
            LabCmd::Touchdown => {
                self.attach_apply(t, action, self.session.attach_touchdown(id), log)
            }
            LabCmd::Failsafe => match self.with_world(|w| w.body(id).map(|b| b.domain)) {
                Some(Domain::Aerial) => {
                    self.attach_apply(t, action, self.session.attach_failsafe(id), log)
                }
                Some(Domain::Ground) => {
                    self.attach_apply(t, action, self.session.attach_estop(id), log)
                }
                Some(Domain::Surface | Domain::Underwater) => {
                    self.attach_apply(t, action, self.session.attach_marine_failsafe(id), log)
                }
                None => Ok(false),
            },
            LabCmd::Disarm => self.attach_apply(t, action, self.session.attach_disarm(id), log),
            LabCmd::Release => self.attach_apply(t, action, self.session.attach_drive(id), log),
            LabCmd::Halt | LabCmd::Park => {
                self.attach_apply(t, action, self.session.attach_park(id), log)
            }
            LabCmd::Estop => self.attach_apply(t, action, self.session.attach_estop(id), log),
            LabCmd::Clear => self.attach_apply(t, action, self.session.attach_reset(id), log),
            LabCmd::Undock => self.attach_apply(t, action, self.session.attach_undock(id), log),
            LabCmd::Dock => self.attach_apply(t, action, self.session.attach_dock(id), log),
            LabCmd::Station => self.attach_apply(t, action, self.session.attach_station(id), log),
            LabCmd::Resume => self.attach_apply(t, action, self.session.attach_resume(id), log),
            LabCmd::Recover => self.try_attach_recover(t, id, action, log),
            LabCmd::Arm => match self.aerial_vehicle(id) {
                Ok(VehicleHandle::PreflightReady(v)) => {
                    v.arm_now()
                        .map_err(|e| lab_from_backend(e.error.into_backend()))?;
                    self.note_attach(t, action, log);
                    self.message = format!("{id} arm");
                    Ok(true)
                }
                _ => Ok(false),
            },
            LabCmd::Offboard => match self.aerial_vehicle(id) {
                Ok(VehicleHandle::Armed(v)) => {
                    v.enter_offboard_now()
                        .map_err(|e| lab_from_backend(e.error.into_backend()))?;
                    self.note_attach(t, action, log);
                    self.message = format!("{id} offboard");
                    Ok(true)
                }
                _ => Ok(false),
            },
            LabCmd::Velocity => self.try_aerial_velocity(t, id, action, log),
            LabCmd::Position => self.try_aerial_position(t, id, action, log),
            LabCmd::Hold => self.attach_apply(t, action, self.session.attach_hold(id), log),
            LabCmd::Drive => self.try_ground_drive(t, id, action, log),
            LabCmd::Thrust => self.try_marine_thrust(t, id, action, log),
        }
    }

    fn try_attach_takeoff(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        match self.session.attach_takeoff(id) {
            Ok(_) => {
                if log {
                    self.push_log(t, AgentAction::new(id, LabCmd::Arm));
                    self.push_log(t, AgentAction::new(id, LabCmd::Offboard));
                    self.push_log(t, AgentAction::new(id, LabCmd::EnableActuators));
                    self.push_log(t, action.clone());
                }
                self.message = format!("{id} takeoff");
                Ok(true)
            }
            Err(BackendError::Protocol) => match self.session.attach_start_takeoff(id) {
                Ok(_) => {
                    self.note_attach(t, action, log);
                    self.message = format!("{id} takeoff");
                    Ok(true)
                }
                Err(BackendError::Protocol) => Ok(false),
                Err(e) => Err(lab_from_backend(e)),
            },
            Err(e) => Err(lab_from_backend(e)),
        }
    }

    fn try_enable_actuators(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        match self.aerial_vehicle(id) {
            Ok(h) => match h.kind() {
                AerialKind::Armed
                | AerialKind::Offboard
                | AerialKind::Takeoff
                | AerialKind::Airborne
                | AerialKind::Landing => {
                    h.into_backend()
                        .enable_actuators_now()
                        .map_err(lab_from_backend)?;
                    self.note_attach(t, action, log);
                    self.message = format!("{id} enable_actuators");
                    Ok(true)
                }
                _ => Ok(false),
            },
            Err(_) => Ok(false),
        }
    }

    fn try_attach_recover(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        match self.with_world(|w| w.body(id).map(|b| b.domain)) {
            Some(Domain::Aerial) => {
                let kind = self.with_world(|w| w.body(id).and_then(|b| b.aerial).map(aerial_kind));
                match self.session.attach_recover_ready(id) {
                    Ok(_) => {
                        if log {
                            if kind == Some(AerialKind::Failsafe) {
                                self.push_log(t, AgentAction::new(id, LabCmd::Disarm));
                            }
                            self.push_log(t, action.clone());
                        }
                        self.message = format!("{id} recover");
                        Ok(true)
                    }
                    Err(BackendError::Protocol) => Ok(false),
                    Err(e) => Err(lab_from_backend(e)),
                }
            }
            Some(Domain::Surface | Domain::Underwater) => {
                self.attach_apply(t, action, self.session.attach_recover(id), log)
            }
            _ => Ok(false),
        }
    }

    fn try_aerial_velocity(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        use flight_core::frames::Ned;
        use flight_core::vector::Velocity;

        let v = Velocity::<Ned>::ned(action.vn, action.ve, action.vd);
        match self.aerial_vehicle(id).map_err(lab_from_backend)? {
            VehicleHandle::Offboard(mut drone) => {
                drone
                    .set_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Takeoff(mut drone) => {
                drone
                    .set_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Airborne(mut drone) => {
                drone
                    .set_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Landing(mut drone) => {
                drone
                    .set_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            _ => return Ok(false),
        }
        self.note_attach(t, action, log);
        self.message = format!("{id} velocity");
        Ok(true)
    }

    fn try_aerial_position(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        use flight_core::frames::Ned;
        use flight_core::vector::Position;

        let p = Position::<Ned>::ned(action.vn, action.ve, action.vd);
        match self.aerial_vehicle(id).map_err(lab_from_backend)? {
            VehicleHandle::Offboard(mut drone) => {
                drone
                    .set_position_now(p)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Takeoff(mut drone) => {
                drone
                    .set_position_now(p)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Airborne(mut drone) => {
                drone
                    .set_position_now(p)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            VehicleHandle::Landing(mut drone) => {
                drone
                    .set_position_now(p)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                drone.backend().flush().map_err(lab_from_backend)?;
            }
            _ => return Ok(false),
        }
        self.note_attach(t, action, log);
        self.message = format!("{id} position");
        Ok(true)
    }

    fn try_ground_drive(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        use flight_core::frames::Ned;
        use flight_core::vector::Velocity;

        let v = Velocity::<Ned>::ned(action.vn, action.ve, action.vd);
        match self.ground_vehicle(id).map_err(lab_from_backend)? {
            GroundHandle::Moving(mut rover) => {
                rover
                    .set_velocity_ned_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                rover.backend().flush().map_err(lab_from_backend)?;
                self.note_attach(t, action, log);
                self.message = format!("{id} drive");
                Ok(true)
            }
            _ => Ok(false),
        }
    }

    fn try_marine_thrust(
        &mut self,
        t: f32,
        id: &'static str,
        action: &AgentAction,
        log: bool,
    ) -> Result<bool, LabError> {
        use flight_core::frames::Ned;
        use flight_core::vector::Velocity;

        let v = Velocity::<Ned>::ned(action.vn, action.ve, action.vd);
        match self.marine_vehicle(id).map_err(lab_from_backend)? {
            MarineHandle::Underway(mut hull) => {
                hull.set_ned_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                hull.backend().flush().map_err(lab_from_backend)?;
                self.note_attach(t, action, log);
                self.message = format!("{id} thrust");
                Ok(true)
            }
            MarineHandle::StationKeep(mut hull) => {
                hull.set_ned_velocity_now(v)
                    .map_err(|e| lab_from_backend(e.into_backend()))?;
                hull.backend().flush().map_err(lab_from_backend)?;
                self.note_attach(t, action, log);
                self.message = format!("{id} thrust");
                Ok(true)
            }
            _ => Ok(false),
        }
    }

    fn attach_apply<T>(
        &mut self,
        t: f32,
        action: &AgentAction,
        result: Result<T, BackendError>,
        log: bool,
    ) -> Result<bool, LabError> {
        match result {
            Ok(_) => {
                self.note_attach(t, action, log);
                self.message = format!("{} {}", action.robot, action.cmd);
                Ok(true)
            }
            Err(BackendError::Protocol) => Ok(false),
            Err(e) => Err(lab_from_backend(e)),
        }
    }

    fn note_attach(&mut self, t: f32, action: &AgentAction, log: bool) {
        if log {
            self.push_log(t, action.clone());
        }
    }

    fn push_log(&mut self, t: f32, action: AgentAction) {
        self.log.push(TimedAction { t, action });
    }

    /// Apply a timed action log, stepping `dt` until `t_end`.
    ///
    /// Each action walks attach helpers and now-APIs the same way
    /// [`Self::act_through_attach`] does, without appending to [`Self::log`].
    /// Logged Takeoff after Arm/Offboard walks `start_takeoff_now`. Logged
    /// Recover after Disarm walks Recovery `attach_recover_ready`. Logged
    /// EnableActuators on an armed aerial machine walks `enable_actuators_now`.
    /// Environment commands and Protocol attach fall back to JSON events.
    pub fn replay_until(
        &mut self,
        log: &[TimedAction],
        dt: f32,
        t_end: f32,
    ) -> Result<(), LabError> {
        let mut i = 0;
        while self.with_world(|w| w.t) + 1e-6 < t_end {
            while i < log.len() && log[i].t <= self.with_world(|w| w.t) + 1e-6 {
                let action = log[i].action.clone();
                if let Err(e) = self.ensure_tool_or_attach_grant(&action) {
                    return Err(self.note_reject(&action, e));
                }
                match self.try_attach(log[i].t, &action, false) {
                    Ok(false) => {
                        if let Err(e) = self.apply_action(&action) {
                            return Err(self.note_reject(&action, e));
                        }
                        self.clear_reject();
                    }
                    Ok(true) => self.clear_reject(),
                    Err(e) => return Err(self.note_reject(&action, e)),
                }
                i += 1;
            }
            self.step(dt);
        }
        Ok(())
    }

    pub fn write_actions_jsonl<W: std::io::Write>(&self, mut w: W) -> std::io::Result<()> {
        for a in &self.log {
            serde_json::to_writer(&mut w, a)?;
            w.write_all(b"\n")?;
        }
        Ok(())
    }

    /// Foxglove-compatible MCAP: current observation plus the timed action log.
    pub fn write_mcap<W: std::io::Write>(&self, w: W) -> std::io::Result<W> {
        let mut bag = crate::bag::McapBag::new(w)?;
        bag.write_observation(&self.observe())?;
        for action in &self.log {
            bag.write_action(action)?;
        }
        bag.finish()
    }

    fn apply_action(&mut self, action: &AgentAction) -> Result<(), LabError> {
        let message = self
            .session
            .with_world_mut(|world| apply_action_world(world, action))?;
        self.message = message;
        Ok(())
    }

    /// NEXT A1: reject unknown robots and cmds not in `legal_cmds` / `env_cmds`
    /// before kernel or attach. Environment cmds do not need a robot id.
    fn ensure_tool(&self, action: &AgentAction) -> Result<(), LabError> {
        if LabCmd::ENV.contains(&action.cmd) {
            return Ok(());
        }
        let id = action.robot.as_str();
        if id.is_empty() {
            return Err(LabError::UnknownRobot(action.robot.clone()));
        }
        self.with_world(|world| {
            let Some(body) = world.body(id) else {
                return Err(LabError::UnknownRobot(action.robot.clone()));
            };
            if action.cmd.on_legal_list(body) {
                Ok(())
            } else {
                Err(LabError::NotLegal {
                    robot: action.robot.clone(),
                    cmd: action.cmd,
                })
            }
        })
    }

    /// Same gate as [`Self::ensure_tool`], plus the Ready/Armed/Offboard
    /// `Takeoff` attach grant (`attach_takeoff` / `attach_start_takeoff`).
    /// Kernel Takeoff from Ready stays illegal on [`Self::act`] (P2).
    fn ensure_tool_or_attach_grant(&self, action: &AgentAction) -> Result<(), LabError> {
        match self.ensure_tool(action) {
            Ok(()) => Ok(()),
            Err(LabError::NotLegal { .. }) if self.attach_takeoff_grant(action) => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn attach_takeoff_grant(&self, action: &AgentAction) -> bool {
        if action.cmd != LabCmd::Takeoff {
            return false;
        }
        let Some(id) = intern_robot(&action.robot) else {
            return false;
        };
        self.with_world(|w| {
            w.body(id)
                .and_then(|b| b.aerial)
                .map(aerial_kind)
                .is_some_and(|k| {
                    matches!(
                        k,
                        AerialKind::PreflightReady | AerialKind::Armed | AerialKind::Offboard
                    )
                })
        })
    }
}

fn intern_robot(id: &str) -> Option<&'static str> {
    Some(match id {
        "drone" => "drone",
        "rover" => "rover",
        "skiff" => "skiff",
        "surveyor" => "surveyor",
        _ => return None,
    })
}

fn lab_from_backend(e: BackendError) -> LabError {
    match e {
        BackendError::Disconnected => LabError::UnknownRobot("disconnected".into()),
        BackendError::Protocol => LabError::WrongDomain,
        BackendError::Rejected(r) => LabError::UnknownCommand(r.into()),
        other => LabError::UnknownCommand(other.to_string()),
    }
}

/// JSON action from an agent or the demo console.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AgentAction {
    #[serde(default)]
    pub robot: String,
    pub cmd: LabCmd,
    #[serde(default)]
    pub vn: f32,
    #[serde(default)]
    pub ve: f32,
    #[serde(default)]
    pub vd: f32,
    #[serde(default)]
    pub yaw_rate: f32,
}

impl AgentAction {
    pub fn new(robot: impl Into<String>, cmd: LabCmd) -> Self {
        Self {
            robot: robot.into(),
            cmd,
            vn: 0.0,
            ve: 0.0,
            vd: 0.0,
            yaw_rate: 0.0,
        }
    }

    pub fn ned(mut self, vn: f32, ve: f32, vd: f32) -> Self {
        self.vn = vn;
        self.ve = ve;
        self.vd = vd;
        self
    }

    pub fn parse_json(s: &str) -> Result<Self, LabError> {
        serde_json::from_str(s).map_err(|e| LabError::UnknownCommand(e.to_string()))
    }
}

/// An `act` stamped with world time, written as JSONL for replay.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TimedAction {
    pub t: f32,
    #[serde(flatten)]
    pub action: AgentAction,
}

#[derive(Clone, Debug)]
pub enum LabError {
    UnknownRobot(String),
    UnknownCommand(String),
    UnknownScenario(String),
    WrongDomain,
    /// Command is not in this body's `legal_cmds` (or `env_cmds`) right now.
    NotLegal {
        robot: String,
        cmd: LabCmd,
    },
    Aerial(flight_core::safety::Reject),
    Ground(flight_core::ground::GroundReject),
    Marine(flight_core::marine::MarineReject),
}

impl std::fmt::Display for LabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LabError::UnknownRobot(id) => write!(f, "unknown robot '{id}'"),
            LabError::UnknownCommand(c) => write!(f, "unknown command '{c}'"),
            LabError::UnknownScenario(s) => write!(f, "unknown scenario '{s}'"),
            LabError::WrongDomain => write!(f, "command does not apply to this robot's domain"),
            LabError::NotLegal { robot, cmd } => write!(f, "not legal now: {robot} {cmd}"),
            LabError::Aerial(r) => write!(f, "aerial safety rejected: {r}"),
            LabError::Ground(r) => write!(f, "ground safety rejected: {r}"),
            LabError::Marine(r) => write!(f, "marine safety rejected: {r}"),
        }
    }
}

impl std::error::Error for LabError {}
