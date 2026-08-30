//! Apply companion-computer setpoints onto a verified [`WorldSession`].
//!
//! `geometry_msgs/Twist` linear is REP-103 ENU. PX4 `TrajectorySetpoint`
//! velocity is NED. Aerial, ground, and marine bodies take the same Twist
//! mapping. `yaw_rad` on [`OffboardSetpoint`] is heading, not a yaw rate,
//! and is not written here. Twist apply is gated on live attach kinds:
//! aerial Offboard / Takeoff / Airborne / Landing, ground Moving, marine
//! Underway / StationKeep.
//! Failsafe / E-stop trips walk [`WorldSession::attach_failsafe`] /
//! [`WorldSession::attach_estop`] / [`WorldSession::attach_marine_failsafe`].
//! Recover walks [`WorldSession::attach_disarm`] / [`WorldSession::attach_recover_ready`] /
//! [`WorldSession::attach_reset`] / [`WorldSession::attach_recover`].
//! Return walks [`WorldSession::attach_land`] / [`WorldSession::attach_touchdown`] /
//! [`WorldSession::attach_park`] / [`WorldSession::attach_dock`].
//! Airborne / station / resume / dock / park / hold walk [`WorldSession::attach_airborne`] /
//! [`WorldSession::attach_station`] / [`WorldSession::attach_resume`] /
//! [`WorldSession::attach_dock`] / [`WorldSession::attach_park`] /
//! [`WorldSession::attach_hold`]. An idle Twist (absent drone field) leaves
//! that hold in place; a live Twist velocity clears it.

use flight_core::contracts::{evaluate_trace, AerialOffboard, LeftoverContract, TraceSample};
use flight_core::safety::Event;
use flight_core::temporal::Sequence;
use flight_core::vehicle::{
    BackendError, GroundHandle, MarineHandle, VehicleBackend, VehicleHandle,
};
use flight_sim::{GroundWorldBackend, MarineWorldBackend, WorldBackend, WorldSession};

use crate::{ros_twist_linear_to_ned, OffboardSetpoint};

/// Write a PX4-shaped offboard setpoint onto an aerial body. Does not step.
/// Ready, failsafe, and recovery are [`BackendError::Rejected`].
pub fn apply_offboard(
    backend: &mut WorldBackend,
    sp: &OffboardSetpoint,
) -> Result<(), BackendError> {
    require_aerial_setpoint(backend)?;
    match (sp.velocity_ned, sp.position_ned) {
        (Some(v), _) => backend.set_velocity_now(v)?,
        (None, Some(p)) => backend.set_position_now(p)?,
        (None, None) => {}
    }
    backend.flush()
}

/// Hold the drone at its current NED pose through [`WorldSession::attach_hold`].
/// Ready / Armed / Failsafe / Recovery are [`BackendError::Protocol`].
pub fn apply_hold(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_hold(backend.body_id())?;
    Ok(())
}

/// REP-103 Twist linear (east, north, up) → NED velocity on the plant.
pub fn apply_twist_linear(
    backend: &mut WorldBackend,
    linear: [f64; 3],
) -> Result<(), BackendError> {
    apply_offboard(
        backend,
        &OffboardSetpoint {
            velocity_ned: Some(ros_twist_linear_to_ned(linear)),
            position_ned: None,
            yaw_rad: None,
        },
    )
}

/// Publish one setpoint and take one verified world step.
pub fn step_offboard(
    backend: &mut WorldBackend,
    sp: &OffboardSetpoint,
    dt: f32,
) -> Result<(), BackendError> {
    apply_offboard(backend, sp)?;
    backend.session().step(dt)
}

/// REP-103 Twist linear onto a ground chassis. Drive must already be live
/// (`attach_drive` / Moving).
pub fn apply_twist_linear_ground(
    backend: &mut GroundWorldBackend,
    linear: [f64; 3],
) -> Result<(), BackendError> {
    match backend.session().ground(backend.body_id()).attach()? {
        GroundHandle::Moving(_) => {}
        _ => return Err(BackendError::Rejected("drive setpoint")),
    }
    backend.set_velocity_now(ros_twist_linear_to_ned(linear))?;
    backend.flush()
}

/// REP-103 Twist linear onto a hull. Thrust must already be live
/// (`attach_undock` / Underway or StationKeep).
pub fn apply_twist_linear_marine(
    backend: &mut MarineWorldBackend,
    linear: [f64; 3],
) -> Result<(), BackendError> {
    match backend.session().marine(backend.body_id()).attach()? {
        MarineHandle::Underway(_) | MarineHandle::StationKeep(_) => {}
        _ => return Err(BackendError::Rejected("thrust setpoint")),
    }
    backend.set_velocity_now(ros_twist_linear_to_ned(linear))?;
    backend.flush()
}

/// Trip aerial failsafe through [`WorldSession::attach_failsafe`].
/// Already-failsafe is [`BackendError::Protocol`].
pub fn apply_failsafe(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_failsafe(backend.body_id())?;
    Ok(())
}

/// Trip chassis E-stop through [`WorldSession::attach_estop`].
/// Already-stopped is [`BackendError::Protocol`].
pub fn apply_estop(backend: &mut GroundWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_estop(backend.body_id())?;
    Ok(())
}

/// Trip hull failsafe through [`WorldSession::attach_marine_failsafe`].
/// Docked or already-failsafe is [`BackendError::Protocol`].
pub fn apply_marine_failsafe(backend: &mut MarineWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_marine_failsafe(backend.body_id())?;
    Ok(())
}

/// Disarm an aerial body through [`WorldSession::attach_disarm`].
/// Failsafe is [`BackendError::Protocol`] (`CanDisarm` stops at Landing).
pub fn apply_disarm(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_disarm(backend.body_id())?;
    Ok(())
}

/// Recover an aerial Failsafe or Recovery body to Ready through
/// [`WorldSession::attach_recover_ready`]. Already-Ready is Protocol.
pub fn apply_recover_ready(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_recover_ready(backend.body_id())?;
    Ok(())
}

/// Clear chassis E-stop through [`WorldSession::attach_reset`].
/// Parked or Moving is [`BackendError::Protocol`].
pub fn apply_reset(backend: &mut GroundWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_reset(backend.body_id())?;
    Ok(())
}

/// Recover a hull from marine Failsafe to Docked through
/// [`WorldSession::attach_recover`]. Docked or Underway is Protocol.
pub fn apply_recover(backend: &mut MarineWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_recover(backend.body_id())?;
    Ok(())
}

/// Enter landing through [`WorldSession::attach_land`].
/// Offboard without Takeoff is [`BackendError::Protocol`].
pub fn apply_land(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_land(backend.body_id())?;
    Ok(())
}

/// Touch down through [`WorldSession::attach_touchdown`].
/// Ready, Armed, and Offboard are [`BackendError::Protocol`].
pub fn apply_touchdown(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_touchdown(backend.body_id())?;
    Ok(())
}

/// Halt a moving chassis through [`WorldSession::attach_park`].
/// Parked or E-stopped is [`BackendError::Protocol`].
pub fn apply_park(backend: &mut GroundWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_park(backend.body_id())?;
    Ok(())
}

/// Come alongside through [`WorldSession::attach_dock`].
/// Docked or marine Failsafe is [`BackendError::Protocol`].
pub fn apply_dock(backend: &mut MarineWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_dock(backend.body_id())?;
    Ok(())
}

/// Takeoff → Airborne through [`WorldSession::attach_airborne`].
/// Ready, Offboard, Airborne, and Landing are [`BackendError::Protocol`].
pub fn apply_airborne(backend: &mut WorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_airborne(backend.body_id())?;
    Ok(())
}

/// Hold station through [`WorldSession::attach_station`].
/// Docked, StationKeep, and Failsafe are [`BackendError::Protocol`].
pub fn apply_station(backend: &mut MarineWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_station(backend.body_id())?;
    Ok(())
}

/// Resume Underway through [`WorldSession::attach_resume`].
/// Docked, Underway, and Failsafe are [`BackendError::Protocol`].
pub fn apply_resume(backend: &mut MarineWorldBackend) -> Result<(), BackendError> {
    let session = backend.session().clone();
    *backend = session.attach_resume(backend.body_id())?;
    Ok(())
}

/// Companion-shaped inject of a kernel revoke event onto a plant body.
/// [`WorldSession::inject_revoke`] first; leftover OffboardControl bound
/// from this plant's grant must then fail `leftover_commands_stale`.
pub fn inject_revoke(backend: &WorldBackend, event: Event) -> Result<(), BackendError> {
    backend.session().inject_revoke(backend.body_id(), event)
}

/// Bind leftover OffboardControl (inland grant is Takeoff) before
/// [`apply_failsafe`]. After the attach trip, leftover `COMMANDS` are
/// `StaleAuthority`. ROS 2-shaped leftover — not a clone of world
/// `run_revoke_table`'s Offboard grant.
pub fn leftover_after_failsafe(seed: u64) -> Result<(), BackendError> {
    let mut plant = FleetPlant::inland(seed);
    plant.grant_all()?;
    let VehicleHandle::Takeoff(mut leftover) = plant.session().aerial("drone").attach()? else {
        return Err(BackendError::Rejected("inland grant must bind Takeoff"));
    };
    if leftover.leftover_commands_stale().is_ok() {
        return Err(BackendError::Rejected("leftover_already_stale"));
    }
    apply_failsafe(plant.drone())?;
    leftover
        .leftover_commands_stale()
        .map_err(|_| BackendError::Rejected("leftover_offboard_still_has_authority"))?;
    Ok(())
}

/// Bind leftover OffboardControl (inland grant is Takeoff) before
/// [`apply_disarm`]. After the attach trip, leftover `COMMANDS` are
/// `StaleAuthority`. Disarm must not latch failsafe.
pub fn leftover_after_disarm(seed: u64) -> Result<(), BackendError> {
    let mut plant = FleetPlant::inland(seed);
    plant.grant_all()?;
    let VehicleHandle::Takeoff(mut leftover) = plant.session().aerial("drone").attach()? else {
        return Err(BackendError::Rejected("inland grant must bind Takeoff"));
    };
    if leftover.leftover_commands_stale().is_ok() {
        return Err(BackendError::Rejected("leftover_already_stale"));
    }
    apply_disarm(plant.drone())?;
    if leftover
        .backend()
        .world()
        .body("drone")
        .and_then(|b| b.aerial)
        .is_some_and(|s| s.failsafe)
    {
        return Err(BackendError::Rejected("disarm_latched_failsafe"));
    }
    leftover
        .leftover_commands_stale()
        .map_err(|_| BackendError::Rejected("leftover_offboard_still_has_authority"))?;
    Ok(())
}

/// Same leftover OffboardControl `COMMANDS` check as world / PX4 / HITL,
/// after [`apply_disarm`] at the ROS 2 plant boundary.
pub fn run_ros2_disarm_leftover() -> Result<usize, String> {
    leftover_after_disarm(1).map_err(|e| format!("ros2 leftover disarm: {e}"))?;
    Ok(1)
}

/// Same leftover OffboardControl `COMMANDS` check as world / PX4 / HITL,
/// after [`apply_failsafe`] at the ROS 2 plant boundary.
pub fn run_ros2_failsafe_leftover() -> Result<usize, String> {
    leftover_after_failsafe(1).map_err(|e| format!("ros2 leftover failsafe: {e}"))?;
    Ok(1)
}

/// Same leftover OffboardControl `COMMANDS` check as world / PX4 / HITL,
/// for every `REVOKE_ON` event, bound from the inland FleetPlant Takeoff
/// grant. Epoch monotonicity is a first-class [`Sequence`]. Lives here
/// because `flight-sim` cannot depend on this crate.
pub fn run_ros2_revoke_table() -> Result<usize, String> {
    let mut n = 0;
    for e in AerialOffboard::REVOKE_ON {
        let mut plant = FleetPlant::inland(1);
        plant
            .grant_all()
            .map_err(|err| format!("grant before {e:?}: {err}"))?;
        let VehicleHandle::Takeoff(mut leftover) = plant
            .session()
            .aerial("drone")
            .attach()
            .map_err(|err| format!("bind Takeoff before {e:?}: {err}"))?
        else {
            return Err(format!("inland grant must bind Takeoff before {e:?}"));
        };
        if leftover.leftover_commands_stale().is_ok() {
            return Err(format!("leftover already stale before ROS 2 inject {e:?}"));
        }
        let mut seq = Sequence::new();
        seq.observe(leftover.backend().authority_epoch())
            .map_err(|_| format!("sequence before {e:?}"))?;
        plant
            .inject_revoke(*e)
            .map_err(|err| format!("inject {e:?}: {err}"))?;
        seq.observe(leftover.backend().authority_epoch())
            .map_err(|_| format!("epoch jumped backward after {e:?}"))?;
        if leftover.backend().authority_epoch() == 0 {
            return Err(format!("event {e:?} did not bump epoch"));
        }
        let failsafe = leftover
            .backend()
            .world()
            .body("drone")
            .and_then(|b| b.aerial)
            .is_some_and(|s| s.failsafe);
        match e {
            Event::Disconnect | Event::Disarm => {
                if failsafe {
                    return Err(format!("{e:?} must not latch failsafe"));
                }
            }
            Event::TriggerFailsafe
            | Event::HeartbeatStale
            | Event::EstimatorInvalid
            | Event::ImuUnhealthy => {
                if !failsafe {
                    return Err(format!("{e:?} must latch failsafe"));
                }
            }
            _ => {}
        }
        leftover
            .leftover_commands_stale()
            .map_err(|err| format!("leftover after {e:?}: {err}"))?;
        n += 1;
    }
    Ok(n)
}

fn drone_trace(backend: &WorldBackend) -> Result<TraceSample, String> {
    let world = backend.world();
    let body = world.body("drone").ok_or("no drone")?;
    let aerial = body.aerial.ok_or("no aerial")?;
    Ok(TraceSample {
        t_secs: world.t,
        armed: aerial.armed,
        actuators_enabled: aerial.actuators_enabled,
        failsafe: aerial.failsafe,
        epoch: body.authority_epoch,
        heartbeat_age_ms: 0,
        command: body.command,
        altitude_m: body.altitude_agl(),
        command_age_ms: 0,
        estimator_ts_ms: (world.t * 1000.0) as u64,
    })
}

/// Leftover OffboardControl after `contract.inject`, evaluated against
/// `contract.require`. Inland grant is Takeoff. Lives here because
/// `flight-sim` cannot depend on this crate.
pub fn run_ros2_leftover_contract(
    contract: LeftoverContract,
) -> Result<LeftoverContractReport, String> {
    let mut plant = FleetPlant::inland(1);
    plant
        .grant_all()
        .map_err(|err| format!("{} grant: {err}", contract.name))?;
    let VehicleHandle::Takeoff(mut leftover) = plant
        .session()
        .aerial("drone")
        .attach()
        .map_err(|err| format!("{} bind Takeoff: {err}", contract.name))?
    else {
        return Err(format!("{}: inland grant must bind Takeoff", contract.name));
    };
    if leftover.leftover_commands_stale().is_ok() {
        return Err(format!(
            "{}: leftover already stale before inject",
            contract.name
        ));
    }
    let before = drone_trace(leftover.backend())?;
    if before.failsafe {
        return Err(format!(
            "{}: failsafe already latched before inject",
            contract.name
        ));
    }
    plant
        .inject_revoke(contract.inject)
        .map_err(|err| format!("{} inject {:?}: {err}", contract.name, contract.inject))?;
    leftover
        .leftover_commands_stale()
        .map_err(|err| format!("{} leftover after inject: {err}", contract.name))?;
    let after = drone_trace(leftover.backend())?;
    if !after.failsafe {
        return Err(format!(
            "{}: {:?} must latch failsafe",
            contract.name, contract.inject
        ));
    }
    if after.epoch <= before.epoch {
        return Err(format!("{}: inject did not bump epoch", contract.name));
    }
    let samples = vec![before, after];
    evaluate_trace(&samples, contract.require)
        .map_err(|e| format!("{} {} at {}", contract.name, e.requirement, e.index))?;
    AerialOffboard::evaluate(&samples).map_err(|e| {
        format!(
            "{} capability {} at {}",
            contract.name, e.requirement, e.index
        )
    })?;
    Ok(LeftoverContractReport {
        name: contract.name,
        inject: contract.inject,
        samples,
    })
}

/// Every distinct leftover contract at the ROS 2 plant.
pub fn run_ros2_leftover_contracts() -> Result<Vec<LeftoverContractReport>, String> {
    AerialOffboard::LEFTOVER_CONTRACTS
        .iter()
        .copied()
        .map(run_ros2_leftover_contract)
        .collect()
}

/// Leftover OffboardControl after `EstimatorInvalid`.
pub fn run_ros2_gps_loss() -> Result<Ros2GpsLossReport, String> {
    let report = run_ros2_leftover_contract(AerialOffboard::GPS_LOSS_CONTRACT)?;
    Ok(Ros2GpsLossReport {
        samples: report.samples,
    })
}

/// Result of [`run_ros2_leftover_contract`].
#[derive(Clone, Debug)]
pub struct LeftoverContractReport {
    pub name: &'static str,
    pub inject: Event,
    pub samples: Vec<TraceSample>,
}

/// Result of [`run_ros2_gps_loss`].
#[derive(Clone, Debug)]
pub struct Ros2GpsLossReport {
    pub samples: Vec<TraceSample>,
}

fn require_aerial_setpoint(backend: &WorldBackend) -> Result<(), BackendError> {
    match backend.session().aerial(backend.body_id()).attach()? {
        VehicleHandle::Offboard(_)
        | VehicleHandle::Takeoff(_)
        | VehicleHandle::Airborne(_)
        | VehicleHandle::Landing(_) => Ok(()),
        _ => Err(BackendError::Rejected("offboard setpoint")),
    }
}

/// Optional REP-103 Twist linear (east, north, up) for each catalog body.
/// Absent bodies (inland hulls, open-water rover) ignore the matching field.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FleetTwist {
    pub drone: Option<[f64; 3]>,
    pub rover: Option<[f64; 3]>,
    pub skiff: Option<[f64; 3]>,
    pub surveyor: Option<[f64; 3]>,
}

/// Drone plus the catalog's rover and hulls on one [`WorldSession`].
/// Flush every live handle, then one step.
pub struct FleetPlant {
    session: WorldSession,
    drone: WorldBackend,
    rover: Option<GroundWorldBackend>,
    skiff: Option<MarineWorldBackend>,
    surveyor: Option<MarineWorldBackend>,
}

impl FleetPlant {
    pub fn coastal(seed: u64) -> Self {
        Self::from_catalog(WorldSession::coastal(seed), true, true)
    }

    /// Harbor fleet: drone, rover, skiff, surveyor on a tighter shoreline.
    pub fn harbor(seed: u64) -> Self {
        Self::from_catalog(WorldSession::harbor(seed), true, true)
    }

    /// Inland drone + rover. No hull — marine Twists are ignored.
    pub fn inland(seed: u64) -> Self {
        Self::from_catalog(WorldSession::inland(seed), true, false)
    }

    /// Open water: drone + skiff + surveyor. No rover — ground Twists are ignored.
    pub fn open_water(seed: u64) -> Self {
        Self::from_catalog(WorldSession::open_water(seed), false, true)
    }

    fn from_catalog(session: WorldSession, with_rover: bool, with_hulls: bool) -> Self {
        let drone = session.aerial("drone");
        let rover = if with_rover {
            Some(session.ground("rover"))
        } else {
            None
        };
        let (skiff, surveyor) = if with_hulls {
            (
                Some(session.marine("skiff")),
                Some(session.marine("surveyor")),
            )
        } else {
            (None, None)
        };
        Self {
            session,
            drone,
            rover,
            skiff,
            surveyor,
        }
    }

    pub fn session(&self) -> &WorldSession {
        &self.session
    }

    pub fn drone(&mut self) -> &mut WorldBackend {
        &mut self.drone
    }

    pub fn rover(&mut self) -> Option<&mut GroundWorldBackend> {
        self.rover.as_mut()
    }

    pub fn skiff(&mut self) -> Option<&mut MarineWorldBackend> {
        self.skiff.as_mut()
    }

    pub fn surveyor(&mut self) -> Option<&mut MarineWorldBackend> {
        self.surveyor.as_mut()
    }

    /// Aerial takeoff, chassis drive, hull undock — consume-self typestate.
    /// Skips bodies the catalog omitted.
    pub fn grant_all(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_takeoff("drone")?;
        if self.rover.is_some() {
            self.rover = Some(self.session.attach_drive("rover")?);
        }
        if self.skiff.is_some() {
            self.skiff = Some(self.session.attach_undock("skiff")?);
        }
        if self.surveyor.is_some() {
            self.surveyor = Some(self.session.attach_undock("surveyor")?);
        }
        Ok(())
    }

    /// Write ENU Twist onto granted bodies. Does not step.
    /// Ungranted machines return `BackendError::Rejected`.
    /// Twists for bodies the catalog omitted are ignored.
    pub fn apply_twists(&mut self, twist: FleetTwist) -> Result<(), BackendError> {
        if let Some(lin) = twist.drone {
            apply_twist_linear(&mut self.drone, lin)?;
        }
        if let (Some(lin), Some(rover)) = (twist.rover, self.rover.as_mut()) {
            apply_twist_linear_ground(rover, lin)?;
        }
        if let (Some(lin), Some(skiff)) = (twist.skiff, self.skiff.as_mut()) {
            apply_twist_linear_marine(skiff, lin)?;
        }
        if let (Some(lin), Some(surveyor)) = (twist.surveyor, self.surveyor.as_mut()) {
            apply_twist_linear_marine(surveyor, lin)?;
        }
        Ok(())
    }

    /// Trip drone failsafe, rover E-stop, and hull failsafe through attach.
    /// Skips bodies the catalog omitted.
    pub fn trip_safety(&mut self) -> Result<(), BackendError> {
        apply_failsafe(&mut self.drone)?;
        if let Some(rover) = self.rover.as_mut() {
            apply_estop(rover)?;
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_marine_failsafe(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_marine_failsafe(surveyor)?;
        }
        Ok(())
    }

    /// Recover drone Ready, rover Parked, and hulls Docked through attach.
    /// Call after [`Self::trip_safety`]. Already-recovered is Protocol.
    pub fn recover_safety(&mut self) -> Result<(), BackendError> {
        apply_recover_ready(&mut self.drone)?;
        if let Some(rover) = self.rover.as_mut() {
            apply_reset(rover)?;
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_recover(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_recover(surveyor)?;
        }
        Ok(())
    }

    /// Land then touchdown, park the rover if present, dock hulls if present.
    /// Call after [`Self::grant_all`]. Already-home is Protocol.
    pub fn return_all(&mut self) -> Result<(), BackendError> {
        apply_land(&mut self.drone)?;
        apply_touchdown(&mut self.drone)?;
        if let Some(rover) = self.rover.as_mut() {
            apply_park(rover)?;
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_dock(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_dock(surveyor)?;
        }
        Ok(())
    }

    /// Takeoff → Airborne. Ready, Offboard, Airborne, and Landing are Protocol.
    pub fn airborne(&mut self) -> Result<(), BackendError> {
        apply_airborne(&mut self.drone)
    }

    /// Hold station on every hull the catalog included.
    /// Inland (no hull) and already-station / docked are Protocol.
    pub fn station_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_station(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_station(surveyor)?;
        }
        Ok(())
    }

    /// Resume Underway on every hull the catalog included.
    /// Inland (no hull) and already-underway / docked are Protocol.
    pub fn resume_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_resume(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_resume(surveyor)?;
        }
        Ok(())
    }

    /// Dock every hull the catalog included (Underway or StationKeep).
    /// Inland (no hull) and already-docked / failsafe are Protocol.
    pub fn dock_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        if let Some(skiff) = self.skiff.as_mut() {
            apply_dock(skiff)?;
        }
        if let Some(surveyor) = self.surveyor.as_mut() {
            apply_dock(surveyor)?;
        }
        Ok(())
    }

    /// Halt the rover if the catalog included one.
    /// Open water (no rover) and already-parked / e-stop are Protocol.
    pub fn park_all(&mut self) -> Result<(), BackendError> {
        let Some(rover) = self.rover.as_mut() else {
            return Err(BackendError::Protocol);
        };
        apply_park(rover)
    }

    /// Hold the drone at its current NED pose. OffboardControl only;
    /// Ready after [`Self::return_all`] is [`BackendError::Protocol`].
    pub fn hold(&mut self) -> Result<(), BackendError> {
        apply_hold(&mut self.drone)
    }

    /// DSL revoke inject on the catalog drone. Same path as
    /// [`WorldSession::inject_revoke`].
    pub fn inject_revoke(&self, event: Event) -> Result<(), BackendError> {
        self.session.inject_revoke("drone", event)
    }

    pub fn step(&self, dt: f32) -> Result<(), BackendError> {
        self.session.step(dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ExternalFlightMode, OffboardSetpoint, VelocityMode};
    use flight_core::frames::Ned;
    use flight_core::vector::{Position, Velocity};
    use flight_sim::WorldSession;

    fn pose(session: &WorldSession, id: &str) -> [f32; 3] {
        session.world().body(id).expect(id).position_m
    }

    fn drone_pose(session: &WorldSession) -> ([f32; 3], f32) {
        let world = session.world();
        let b = world.body("drone").expect("drone");
        (b.position_m, b.altitude_agl())
    }

    #[test]
    fn enu_up_twist_climbs_ned() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        apply_twist_linear(&mut drone, [0.0, 0.0, 1.2]).unwrap();
        let (p0, _) = drone_pose(&session);
        for _ in 0..40 {
            session.step(0.02).unwrap();
        }
        let (p1, alt) = drone_pose(&session);
        assert!(
            p1[2] < p0[2],
            "NED z-down: ENU-up Twist must climb, {p0:?} → {p1:?}"
        );
        assert!(alt > 0.15);
        assert!(session.world().all_hold());
    }

    #[test]
    fn ready_drone_rejects_enu_twist() {
        let session = WorldSession::coastal(1);
        let mut drone = session.aerial("drone");
        assert!(matches!(
            apply_twist_linear(&mut drone, [0.0, 0.0, 1.2]),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
        assert!(session.world().body("drone").unwrap().command.is_none());
    }

    #[test]
    fn velocity_mode_steps_the_plant() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        let mut mode = VelocityMode::new(Velocity::<Ned>::ned(0.0, 0.0, -1.0));
        mode.on_activate();
        let (_, alt0) = drone_pose(&session);
        for _ in 0..40 {
            step_offboard(&mut drone, &mode.update(0.02), 0.02).unwrap();
        }
        let (_, alt1) = drone_pose(&session);
        assert!(alt1 > alt0 + 0.2, "alt {alt0} → {alt1}");
        assert!(session.world().all_hold());
    }

    #[test]
    fn position_setpoint_climbs_toward_ned_target() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        let (p0, _) = drone_pose(&session);
        apply_offboard(
            &mut drone,
            &OffboardSetpoint {
                velocity_ned: None,
                position_ned: Some(Position::<Ned>::ned(p0[0], p0[1], p0[2] - 4.0)),
                yaw_rad: None,
            },
        )
        .unwrap();
        for _ in 0..80 {
            session.step(0.02).unwrap();
        }
        let (p1, alt) = drone_pose(&session);
        assert!(p1[2] < p0[2] - 0.3, "position P-loop climb {p0:?} → {p1:?}");
        assert!(alt > 0.3);
        assert!(session.world().all_hold());
    }

    #[test]
    fn enu_north_twist_drives_moving_rover() {
        let session = WorldSession::inland(1);
        let mut rover = session.attach_drive("rover").unwrap();
        let n0 = pose(&session, "rover")[0];
        apply_twist_linear_ground(&mut rover, [0.0, 0.8, 0.0]).unwrap();
        for _ in 0..40 {
            session.step(0.02).unwrap();
        }
        let n1 = pose(&session, "rover")[0];
        assert!(
            n1 > n0 + 0.15,
            "ENU-north Twist must drive NED north {n0} → {n1}"
        );
        assert!(session.world().all_hold());
    }

    #[test]
    fn parked_rover_rejects_twist() {
        let session = WorldSession::inland(1);
        let mut rover = session.ground("rover");
        let n0 = pose(&session, "rover")[0];
        assert!(apply_twist_linear_ground(&mut rover, [0.0, 1.0, 0.0]).is_err());
        for _ in 0..20 {
            session.step(0.02).unwrap();
        }
        let n1 = pose(&session, "rover")[0];
        assert!(
            (n1 - n0).abs() < 0.05,
            "parked chassis must not take Twist {n0} → {n1}"
        );
        assert!(session.world().all_hold());
    }

    #[test]
    fn enu_east_twist_makes_way_on_underway_skiff() {
        let session = WorldSession::coastal(1);
        let mut skiff = session.attach_undock("skiff").unwrap();
        let e0 = pose(&session, "skiff")[1];
        apply_twist_linear_marine(&mut skiff, [0.6, 0.0, 0.0]).unwrap();
        for _ in 0..40 {
            session.step(0.02).unwrap();
        }
        let e1 = pose(&session, "skiff")[1];
        assert!(
            e1 > e0 + 0.08,
            "ENU-east Twist must drive NED east {e0} → {e1}"
        );
        assert!(session.world().all_hold());
    }

    #[test]
    fn lab_json_act_and_ros_twist_share_plant() {
        use robot_lab::{AgentAction, Lab, LabCmd};

        let mut lab = Lab::coastal(1);
        for (robot, cmd) in [
            ("drone", LabCmd::Arm),
            ("drone", LabCmd::Offboard),
            ("drone", LabCmd::EnableActuators),
            ("drone", LabCmd::Takeoff),
            ("rover", LabCmd::Release),
        ] {
            lab.act(AgentAction::new(robot, cmd)).unwrap();
        }

        let obs0 = lab.observe();
        let alt0 = obs0.robots.iter().find(|r| r.id == "drone").unwrap().alt;
        let n0 = obs0.robots.iter().find(|r| r.id == "rover").unwrap().n;

        apply_twist_linear(&mut lab.aerial("drone"), [0.0, 0.0, 1.0]).unwrap();
        apply_twist_linear_ground(&mut lab.ground("rover"), [0.0, -0.8, 0.0]).unwrap();
        for _ in 0..40 {
            lab.step(0.02);
        }

        let obs = lab.observe();
        assert!(obs.all_hold, "broken {:?}", obs.properties);
        let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
        let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
        assert!(drone.alt > alt0 + 0.15, "alt {alt0} → {}", drone.alt);
        assert!(rover.n < n0 - 0.1, "n {n0} → {}", rover.n);
        let tel = lab.aerial("drone").telemetry_now().unwrap();
        assert!((tel.position.z() - drone.d).abs() < 1e-4);
    }

    #[test]
    fn fleet_plant_grants_and_moves_four_domains() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        let world = plant.session().world();
        assert_eq!(
            world.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Takeoff
        );
        assert_eq!(
            world.body("rover").unwrap().ground.unwrap().phase,
            flight_core::ground::GroundPhase::Moving
        );
        assert_eq!(
            world.body("skiff").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        assert_eq!(
            world.body("surveyor").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        assert!(world.body("drone").unwrap().actuators_granted());
        assert!(world.body("rover").unwrap().ground.unwrap().drive_enabled);
        assert!(world.body("skiff").unwrap().marine.unwrap().thrust_enabled);
        assert!(
            world
                .body("surveyor")
                .unwrap()
                .marine
                .unwrap()
                .thrust_enabled
        );
        let alt0 = world.body("drone").unwrap().altitude_agl();
        let n0 = world.body("rover").unwrap().position_m[0];
        let e0 = world.body("skiff").unwrap().position_m[1];
        let sn0 = world.body("surveyor").unwrap().position_m[0];
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.6, 0.0, 0.0]),
            surveyor: Some([0.0, 0.4, 0.0]),
        };
        for _ in 0..40 {
            plant.apply_twists(cmd).unwrap();
            plant.step(0.02).unwrap();
        }
        let world = plant.session().world();
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let alt1 = world.body("drone").unwrap().altitude_agl();
        let n1 = world.body("rover").unwrap().position_m[0];
        let e1 = world.body("skiff").unwrap().position_m[1];
        let sn1 = world.body("surveyor").unwrap().position_m[0];
        assert!(alt1 > alt0 + 0.15, "drone alt {alt0} → {alt1}");
        assert!(n1 < n0 - 0.1, "rover south {n0} → {n1}");
        assert!(e1 > e0 + 0.08, "skiff east {e0} → {e1}");
        assert!(sn1 > sn0 + 0.12, "surveyor north {sn0} → {sn1}");
    }

    #[test]
    fn harbor_plant_grants_four_bodies() {
        let mut plant = FleetPlant::harbor(1);
        assert_eq!(plant.session().world().scenario, "harbor");
        plant.grant_all().unwrap();
        let world = plant.session().world();
        assert_eq!(
            world.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Takeoff
        );
        assert_eq!(
            world.body("rover").unwrap().ground.unwrap().phase,
            flight_core::ground::GroundPhase::Moving
        );
        assert_eq!(
            world.body("skiff").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        assert_eq!(
            world.body("surveyor").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        plant
            .apply_twists(FleetTwist {
                drone: Some([0.0, 0.0, 1.0]),
                rover: Some([0.0, -0.4, 0.0]),
                skiff: Some([0.4, 0.0, 0.0]),
                surveyor: Some([0.0, 0.3, 0.0]),
            })
            .unwrap();
        plant.step(0.02).unwrap();
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn inland_plant_grants_air_and_ground_without_hulls() {
        let mut plant = FleetPlant::inland(1);
        let world = plant.session().world();
        assert_eq!(world.scenario, "inland");
        assert!(world.body("skiff").is_none());
        assert!(world.body("surveyor").is_none());
        plant.grant_all().unwrap();
        let world = plant.session().world();
        assert_eq!(
            world.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Takeoff
        );
        assert_eq!(
            world.body("rover").unwrap().ground.unwrap().phase,
            flight_core::ground::GroundPhase::Moving
        );
        let alt0 = world.body("drone").unwrap().altitude_agl();
        let n0 = world.body("rover").unwrap().position_m[0];
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.8, 0.0, 0.0]),
            surveyor: Some([0.0, 0.5, 0.0]),
        };
        for _ in 0..40 {
            plant.apply_twists(cmd).unwrap();
            plant.step(0.02).unwrap();
        }
        let world = plant.session().world();
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let alt1 = world.body("drone").unwrap().altitude_agl();
        let n1 = world.body("rover").unwrap().position_m[0];
        assert!(alt1 > alt0 + 0.15, "drone alt {alt0} → {alt1}");
        assert!(n1 < n0 - 0.1, "rover south {n0} → {n1}");
        assert!(world.body("skiff").is_none());
    }

    #[test]
    fn open_water_plant_grants_air_and_hulls_without_rover() {
        let mut plant = FleetPlant::open_water(1);
        let world = plant.session().world();
        assert_eq!(world.scenario, "open_water");
        assert!(world.body("rover").is_none());
        plant.grant_all().unwrap();
        let world = plant.session().world();
        assert_eq!(
            world.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Takeoff
        );
        assert_eq!(
            world.body("skiff").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        assert_eq!(
            world.body("surveyor").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Underway
        );
        let alt0 = world.body("drone").unwrap().altitude_agl();
        let e0 = world.body("skiff").unwrap().position_m[1];
        let sn0 = world.body("surveyor").unwrap().position_m[0];
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.6, 0.0, 0.0]),
            surveyor: Some([0.0, 0.4, 0.0]),
        };
        for _ in 0..40 {
            plant.apply_twists(cmd).unwrap();
            plant.step(0.02).unwrap();
        }
        let world = plant.session().world();
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let alt1 = world.body("drone").unwrap().altitude_agl();
        let e1 = world.body("skiff").unwrap().position_m[1];
        let sn1 = world.body("surveyor").unwrap().position_m[0];
        assert!(alt1 > alt0 + 0.15, "drone alt {alt0} → {alt1}");
        assert!(e1 > e0 + 0.08, "skiff east {e0} → {e1}");
        assert!(sn1 > sn0 + 0.12, "surveyor north {sn0} → {sn1}");
        assert!(world.body("rover").is_none());
    }

    #[test]
    fn fleet_plant_without_grant_rejects_twist() {
        let mut plant = FleetPlant::coastal(1);
        let world = plant.session().world();
        let alt0 = world.body("drone").unwrap().altitude_agl();
        let n0 = world.body("rover").unwrap().position_m[0];
        let e0 = world.body("skiff").unwrap().position_m[1];
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -1.0, 0.0]),
            skiff: Some([0.8, 0.0, 0.0]),
            surveyor: Some([0.0, 0.5, 0.0]),
        };
        assert!(plant.apply_twists(cmd).is_err());
        for _ in 0..20 {
            plant.step(0.02).unwrap();
        }
        let world = plant.session().world();
        assert!(world.all_hold());
        let alt1 = world.body("drone").unwrap().altitude_agl();
        let n1 = world.body("rover").unwrap().position_m[0];
        let e1 = world.body("skiff").unwrap().position_m[1];
        assert!((alt1 - alt0).abs() < 0.2, "disarmed climb {alt0} → {alt1}");
        assert!((n1 - n0).abs() < 0.08, "parked drive {n0} → {n1}");
        assert!(
            (e1 - e0).abs() < 0.25,
            "docked skiff must not make way {e0} → {e1}"
        );
        let thrust = world.body("surveyor").unwrap().last_thrust;
        assert!(
            thrust.iter().all(|c| c.abs() < 1e-6),
            "docked surveyor thrust {thrust:?}"
        );
    }

    #[test]
    fn apply_failsafe_walks_attach_and_rejects_twist() {
        let session = WorldSession::coastal(1);
        let mut drone = session.aerial("drone");
        apply_failsafe(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("Ready failsafe must bind Failsafe, got {:?}", other.kind()),
        }
        assert!(matches!(
            apply_twist_linear(&mut drone, [0.0, 0.0, 1.2]),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
        assert!(matches!(
            apply_failsafe(&mut drone),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn fleet_plant_trip_safety_walks_attach() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        plant.trip_safety().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().ground("rover").attach().unwrap() {
            GroundHandle::EStopped(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Failsafe(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Failsafe(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.6, 0.0, 0.0]),
            surveyor: Some([0.0, 0.4, 0.0]),
        };
        assert!(plant.apply_twists(cmd).is_err());
        assert!(matches!(plant.trip_safety(), Err(BackendError::Protocol)));
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn apply_disarm_walks_takeoff_to_ready() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        apply_disarm(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("disarm must bind Ready, got {:?}", other.kind()),
        }
        assert!(!session.world().body("drone").unwrap().aerial.unwrap().armed);
        assert!(matches!(
            apply_twist_linear(&mut drone, [0.0, 0.0, 1.2]),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
        apply_disarm(&mut drone).unwrap();
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_recover_ready_walks_failsafe_to_ready() {
        let session = WorldSession::coastal(1);
        let mut drone = session.aerial("drone");
        apply_failsafe(&mut drone).unwrap();
        apply_recover_ready(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("recover must bind Ready, got {:?}", other.kind()),
        }
        assert!(
            !session
                .world()
                .body("drone")
                .unwrap()
                .aerial
                .unwrap()
                .failsafe
        );
        assert!(matches!(
            apply_recover_ready(&mut drone),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_reset_clears_estop_to_parked() {
        let session = WorldSession::inland(1);
        let mut rover = session.ground("rover");
        apply_estop(&mut rover).unwrap();
        match session.ground("rover").attach().unwrap() {
            GroundHandle::EStopped(_) => {}
            other => panic!("estop must bind EStopped, got {:?}", other.kind()),
        }
        apply_reset(&mut rover).unwrap();
        match session.ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("reset must bind Parked, got {:?}", other.kind()),
        }
        assert!(matches!(
            apply_reset(&mut rover),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_recover_docks_failsafe_hull() {
        let session = WorldSession::coastal(1);
        let mut skiff = session.attach_undock("skiff").unwrap();
        apply_marine_failsafe(&mut skiff).unwrap();
        apply_recover(&mut skiff).unwrap();
        match session.marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("recover must bind Docked, got {:?}", other.kind()),
        }
        assert!(
            !session
                .world()
                .body("skiff")
                .unwrap()
                .marine
                .unwrap()
                .failsafe
        );
        assert!(matches!(
            apply_recover(&mut skiff),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn fleet_plant_recover_safety_after_trip() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        plant.trip_safety().unwrap();
        plant.recover_safety().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(
            plant.recover_safety(),
            Err(BackendError::Protocol)
        ));
        plant.grant_all().unwrap();
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.6, 0.0, 0.0]),
            surveyor: Some([0.0, 0.4, 0.0]),
        };
        plant.apply_twists(cmd).unwrap();
        plant.step(0.02).unwrap();
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn apply_land_then_touchdown_returns_ready() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        apply_land(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::Landing(_) => {}
            other => panic!("land must bind Landing, got {:?}", other.kind()),
        }
        apply_touchdown(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("touchdown must bind Ready, got {:?}", other.kind()),
        }
        assert!(!session.world().body("drone").unwrap().aerial.unwrap().armed);
        assert!(matches!(
            apply_land(&mut drone),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_park_halts_moving_rover() {
        let session = WorldSession::inland(1);
        let mut rover = session.attach_drive("rover").unwrap();
        apply_park(&mut rover).unwrap();
        match session.ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("park must bind Parked, got {:?}", other.kind()),
        }
        assert!(
            !session
                .world()
                .body("rover")
                .unwrap()
                .ground
                .unwrap()
                .drive_enabled
        );
        assert!(matches!(
            apply_park(&mut rover),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_dock_comes_alongside() {
        let session = WorldSession::coastal(1);
        let mut skiff = session.attach_undock("skiff").unwrap();
        apply_dock(&mut skiff).unwrap();
        match session.marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("dock must bind Docked, got {:?}", other.kind()),
        }
        assert!(
            !session
                .world()
                .body("skiff")
                .unwrap()
                .marine
                .unwrap()
                .thrust_enabled
        );
        assert!(matches!(
            apply_dock(&mut skiff),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn fleet_plant_return_all_after_grant() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        plant.return_all().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        let cmd = FleetTwist {
            drone: Some([0.0, 0.0, 1.2]),
            rover: Some([0.0, -0.8, 0.0]),
            skiff: Some([0.6, 0.0, 0.0]),
            surveyor: Some([0.0, 0.4, 0.0]),
        };
        assert!(plant.apply_twists(cmd).is_err());
        assert!(matches!(plant.return_all(), Err(BackendError::Protocol)));
        plant.grant_all().unwrap();
        plant.apply_twists(cmd).unwrap();
        plant.step(0.02).unwrap();
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn inland_plant_trip_and_return_skip_hulls() {
        let mut plant = FleetPlant::inland(1);
        plant.grant_all().unwrap();
        plant.trip_safety().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().ground("rover").attach().unwrap() {
            GroundHandle::EStopped(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        assert!(plant.session().world().body("skiff").is_none());
        plant.recover_safety().unwrap();
        plant.grant_all().unwrap();
        plant.return_all().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        assert!(plant.session().world().body("skiff").is_none());
        assert!(matches!(plant.return_all(), Err(BackendError::Protocol)));
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn open_water_plant_trip_and_return_skip_rover() {
        let mut plant = FleetPlant::open_water(1);
        plant.grant_all().unwrap();
        plant.trip_safety().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Failsafe(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        assert!(plant.session().world().body("rover").is_none());
        plant.recover_safety().unwrap();
        plant.grant_all().unwrap();
        plant.return_all().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(plant.session().world().body("rover").is_none());
        assert!(matches!(plant.return_all(), Err(BackendError::Protocol)));
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn apply_airborne_declares_climb_complete() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        apply_airborne(&mut drone).unwrap();
        match session.aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!("airborne must bind Airborne, got {:?}", other.kind()),
        }
        assert!(matches!(
            apply_airborne(&mut drone),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_station_then_resume_on_a_hull() {
        let session = WorldSession::coastal(1);
        let mut skiff = session.attach_undock("skiff").unwrap();
        apply_station(&mut skiff).unwrap();
        match session.marine("skiff").attach().unwrap() {
            MarineHandle::StationKeep(_) => {}
            other => panic!("station must bind StationKeep, got {:?}", other.kind()),
        }
        assert!(matches!(
            apply_station(&mut skiff),
            Err(BackendError::Protocol)
        ));
        apply_resume(&mut skiff).unwrap();
        match session.marine("skiff").attach().unwrap() {
            MarineHandle::Underway(_) => {}
            other => panic!("resume must bind Underway, got {:?}", other.kind()),
        }
        assert!(matches!(
            apply_resume(&mut skiff),
            Err(BackendError::Protocol)
        ));
        assert!(session.world().all_hold());
    }

    #[test]
    fn fleet_plant_airborne_station_resume() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        plant.airborne().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        assert!(matches!(plant.airborne(), Err(BackendError::Protocol)));
        plant.station_all().unwrap();
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::StationKeep(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::StationKeep(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(plant.station_all(), Err(BackendError::Protocol)));
        plant.resume_all().unwrap();
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Underway(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Underway(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(plant.resume_all(), Err(BackendError::Protocol)));
        plant.step(0.02).unwrap();
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn inland_plant_station_is_protocol() {
        let mut plant = FleetPlant::inland(1);
        plant.grant_all().unwrap();
        assert!(matches!(plant.station_all(), Err(BackendError::Protocol)));
        assert!(matches!(plant.resume_all(), Err(BackendError::Protocol)));
        plant.airborne().unwrap();
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
    }

    #[test]
    fn fleet_plant_dock_all_from_underway() {
        for mut plant in [
            FleetPlant::coastal(1),
            FleetPlant::harbor(1),
            FleetPlant::open_water(1),
        ] {
            let name = plant.session().world().scenario;
            plant.grant_all().unwrap();
            plant.dock_all().expect(name);
            match plant.session().marine("skiff").attach().unwrap() {
                MarineHandle::Docked(_) => {}
                other => panic!("{name} skiff {:?}", other.kind()),
            }
            match plant.session().marine("surveyor").attach().unwrap() {
                MarineHandle::Docked(_) => {}
                other => panic!("{name} surveyor {:?}", other.kind()),
            }
            assert!(
                matches!(plant.dock_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            plant.step(0.02).unwrap();
            assert!(plant.session().world().all_hold(), "{name}");
        }
    }

    #[test]
    fn inland_plant_dock_all_is_protocol() {
        let mut plant = FleetPlant::inland(1);
        plant.grant_all().unwrap();
        assert!(matches!(plant.dock_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn fleet_plant_dock_all_from_station() {
        let mut plant = FleetPlant::coastal(1);
        plant.grant_all().unwrap();
        plant.station_all().unwrap();
        plant.dock_all().unwrap();
        match plant.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match plant.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(plant.dock_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn fleet_plant_park_all_from_moving() {
        for mut plant in [
            FleetPlant::coastal(1),
            FleetPlant::harbor(1),
            FleetPlant::inland(1),
        ] {
            let name = plant.session().world().scenario;
            plant.grant_all().unwrap();
            plant.park_all().expect(name);
            match plant.session().ground("rover").attach().unwrap() {
                GroundHandle::Parked(_) => {}
                other => panic!("{name} rover {:?}", other.kind()),
            }
            assert!(
                matches!(plant.park_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            plant.step(0.02).unwrap();
            assert!(plant.session().world().all_hold(), "{name}");
        }
    }

    #[test]
    fn open_water_plant_park_all_is_protocol() {
        let mut plant = FleetPlant::open_water(1);
        plant.grant_all().unwrap();
        assert!(matches!(plant.park_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn apply_hold_sets_ned_pose_and_twist_clears_it() {
        let session = WorldSession::coastal(1);
        let mut drone = session.attach_takeoff("drone").unwrap();
        apply_hold(&mut drone).unwrap();
        let pose = session.world().body("drone").unwrap().position_m;
        assert_eq!(session.world().body("drone").unwrap().hold_ned, Some(pose));
        session.step(0.02).unwrap();
        assert!(session.world().body("drone").unwrap().hold_ned.is_some());
        apply_twist_linear(&mut drone, [0.0, 0.0, 1.0]).unwrap();
        assert!(session.world().body("drone").unwrap().hold_ned.is_none());
        assert!(session.world().all_hold());
    }

    #[test]
    fn apply_hold_before_grant_is_protocol() {
        let session = WorldSession::coastal(1);
        let mut drone = session.aerial("drone");
        assert!(matches!(
            apply_hold(&mut drone),
            Err(BackendError::Protocol)
        ));
    }

    #[test]
    fn fleet_plant_hold_survives_idle_twist() {
        for mut plant in [
            FleetPlant::coastal(1),
            FleetPlant::harbor(1),
            FleetPlant::inland(1),
            FleetPlant::open_water(1),
        ] {
            let name = plant.session().world().scenario;
            plant.grant_all().unwrap();
            plant.hold().expect(name);
            let pose = plant.session().world().body("drone").unwrap().position_m;
            assert_eq!(
                plant.session().world().body("drone").unwrap().hold_ned,
                Some(pose),
                "{name}"
            );
            plant.step(0.02).unwrap();
            assert!(
                plant
                    .session()
                    .world()
                    .body("drone")
                    .unwrap()
                    .hold_ned
                    .is_some(),
                "{name}"
            );
            plant
                .apply_twists(FleetTwist {
                    drone: Some([0.0, 0.0, 1.0]),
                    ..FleetTwist::default()
                })
                .expect(name);
            assert!(
                plant
                    .session()
                    .world()
                    .body("drone")
                    .unwrap()
                    .hold_ned
                    .is_none(),
                "{name} live Twist must win"
            );
            plant.step(0.02).unwrap();
            assert!(plant.session().world().all_hold(), "{name}");
        }
    }

    #[test]
    fn fleet_plant_hold_after_return_is_protocol() {
        let mut plant = FleetPlant::inland(1);
        plant.grant_all().unwrap();
        plant.return_all().unwrap();
        assert!(matches!(plant.hold(), Err(BackendError::Protocol)));
    }

    #[test]
    fn leftover_commands_stale_after_apply_failsafe() {
        leftover_after_failsafe(1).expect("leftover after apply_failsafe");
        assert_eq!(run_ros2_failsafe_leftover().expect("runner"), 1);
    }

    #[test]
    fn leftover_commands_stale_after_apply_disarm() {
        leftover_after_disarm(1).expect("leftover after apply_disarm");
        assert_eq!(run_ros2_disarm_leftover().expect("runner"), 1);
    }

    #[test]
    fn leftover_commands_stale_after_every_dsl_revoke() {
        let n = run_ros2_revoke_table().expect("ros2 leftover revoke table");
        assert_eq!(n, AerialOffboard::REVOKE_ON.len());
    }

    #[test]
    fn leftover_offboard_gps_loss_satisfies_world_contract() {
        let report = run_ros2_gps_loss().expect("ros2 gps-loss");
        assert_eq!(report.samples.len(), 2);
        assert!(!report.samples[0].failsafe);
        assert!(report.samples[1].failsafe);
        assert!(report.samples[1].epoch > report.samples[0].epoch);
    }

    #[test]
    fn leftover_named_contracts_satisfy_world_monitors() {
        let reports = run_ros2_leftover_contracts().expect("ros2 leftover contracts");
        assert_eq!(reports.len(), AerialOffboard::LEFTOVER_CONTRACTS.len());
        for (report, contract) in reports.iter().zip(AerialOffboard::LEFTOVER_CONTRACTS) {
            assert_eq!(report.name, contract.name);
            assert_eq!(report.inject, contract.inject);
            assert_eq!(report.samples.len(), 2);
            assert!(!report.samples[0].failsafe);
            assert!(report.samples[1].failsafe);
            assert!(report.samples[1].epoch > report.samples[0].epoch);
        }
    }

    #[test]
    fn inject_revoke_rejects_non_revoke_and_bumps_epoch() {
        let mut plant = FleetPlant::inland(1);
        plant.grant_all().unwrap();
        assert!(matches!(
            inject_revoke(plant.drone(), Event::MissionCommand),
            Err(BackendError::Rejected("not a revoke inject"))
        ));
        plant.inject_revoke(Event::TriggerFailsafe).unwrap();
        assert!(
            plant
                .session()
                .world()
                .body("drone")
                .unwrap()
                .authority_epoch
                > 0
        );
        assert!(
            plant
                .session()
                .world()
                .body("drone")
                .unwrap()
                .aerial
                .unwrap()
                .failsafe
        );
    }
}
