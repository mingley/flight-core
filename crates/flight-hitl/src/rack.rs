//! Multi-vehicle HITL rack over one verified [`WorldSession`].
//!
//! One frame: apply commands **only if the previous frame met its deadline**,
//! flush every handle, step the plant once, then score wall time against the
//! budget and OffboardControl [`Rate`]. A miss trips failsafe through [`WorldSession::attach_failsafe`] /
//! [`WorldSession::attach_estop`] / [`WorldSession::attach_marine_failsafe`]
//! when the live kind allows it, otherwise the idempotent `failsafe_now`
//! re-trip, and zeros the next applied command.
//! [`WorldRack::coastal`] and [`WorldRack::harbor`] grant through
//! [`WorldSession::attach_takeoff`] / [`WorldSession::attach_drive`] /
//! [`WorldSession::attach_undock`] (skiff and surveyor). [`WorldRack::inland`]
//! is drone + rover (no hull). [`WorldRack::open_water`] is drone + hulls
//! (no rover). On-time frames write NED setpoints only while attach is
//! Offboard-control / Moving / Underway / StationKeep.
//! [`WorldRack::recover_deadline`] walks
//! [`WorldSession::attach_recover_ready`] / [`WorldSession::attach_reset`] /
//! [`WorldSession::attach_recover`] and clears the miss latch;
//! [`WorldRack::grant_all`] re-walks takeoff / drive / undock so the next
//! on-time frame can command. [`WorldRack::return_all`] walks land then
//! touchdown, park, and dock (skipping bodies the catalog omitted) so the rack
//! is Ready / Parked / Docked — the inverse of grant. [`WorldRack::airborne`]
//! is Takeoff → Airborne. [`WorldRack::station_all`] / [`WorldRack::resume_all`]
//! hold and resume hulls the catalog included (inland is Protocol).
//! [`WorldRack::dock_all`] docks those hulls from Underway or StationKeep
//! (inland and already-docked are Protocol). [`WorldRack::park_all`] halts
//! the rover when the catalog included one (open water is Protocol).
//! [`WorldRack::hold`] writes the drone's current NED pose through
//! [`WorldSession::attach_hold`]. Idle frames leave the hold in place;
//! a live aerial velocity frame clears it. Ready after return is Protocol.

use std::net::UdpSocket;
use std::time::Instant;

use flight_core::contracts::{AerialOffboard, TraceSample};
use flight_core::hitl::{
    command_after_deadline, deadline_outcome, hitl_apply_allowed, DeadlineOutcome, DeadlineSpec,
};
use flight_core::safety::Event;
use flight_core::temporal::{Rate, Sequence};
use flight_core::vector::Velocity;
use flight_core::vehicle::{
    BackendError, GroundHandle, MarineHandle, VehicleBackend, VehicleHandle,
};
use flight_sim::{GroundWorldBackend, MarineWorldBackend, WorldBackend, WorldSession};
use robot_world::World;

use crate::protocol::{self, Command, Sample};

#[derive(Clone, Copy, Debug, Default)]
pub struct RackCommand {
    pub aerial: [f32; 3],
    pub ground: [f32; 3],
    /// Surface hull (skiff) NED velocity.
    pub marine: [f32; 3],
    /// Underwater hull (surveyor) NED velocity.
    pub underwater: [f32; 3],
}

impl RackCommand {
    /// Build a rack command from decoded FCH1 datagrams.
    /// Slots: 0 drone, 1 rover, 2 skiff, 3 surveyor. `apply == 0` writes
    /// zero for that slot so a miss cannot revive a hold or a live setpoint.
    pub fn from_fch1(cmds: &[Command]) -> Self {
        let mut out = Self::default();
        for c in cmds {
            let v = if c.apply == 0 {
                [0.0, 0.0, 0.0]
            } else {
                c.velocity_ned
            };
            match c.slot {
                0 => out.aerial = v,
                1 => out.ground = v,
                2 => out.marine = v,
                3 => out.underwater = v,
                _ => {}
            }
        }
        out
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RackFrame {
    pub t: f32,
    pub compute_ns: u64,
    pub outcome: DeadlineOutcome,
    pub applied_aerial: [f32; 3],
    pub all_hold: bool,
    pub missed_total: u64,
}

impl RackFrame {
    pub fn missed(self) -> bool {
        self.outcome.missed()
    }
}

/// Coastal / harbor fleet, inland air+ground, or open-water air+hulls on one deadline rack.
pub struct WorldRack {
    session: WorldSession,
    drone: WorldBackend,
    rover: Option<GroundWorldBackend>,
    skiff: Option<MarineWorldBackend>,
    surveyor: Option<MarineWorldBackend>,
    spec: DeadlineSpec,
    /// OffboardControl loop rate. Fail-closed with [`DeadlineSpec::period_ns`].
    rate: Rate,
    last_missed: bool,
    missed: u64,
    frames: u64,
    /// Optional datagram dump a physical logger / I/O card can listen to.
    out: Option<UdpSocket>,
}

impl WorldRack {
    pub fn coastal(seed: u64) -> Result<Self, BackendError> {
        Self::from_catalog(WorldSession::coastal(seed), true, true)
    }

    /// Harbor fleet: drone, rover, skiff, surveyor on a tighter shoreline.
    pub fn harbor(seed: u64) -> Result<Self, BackendError> {
        Self::from_catalog(WorldSession::harbor(seed), true, true)
    }

    /// Inland drone + rover. No hull — marine commands are ignored.
    pub fn inland(seed: u64) -> Result<Self, BackendError> {
        Self::from_catalog(WorldSession::inland(seed), true, false)
    }

    /// Open water: drone + skiff + surveyor. No rover — ground commands are ignored.
    pub fn open_water(seed: u64) -> Result<Self, BackendError> {
        Self::from_catalog(WorldSession::open_water(seed), false, true)
    }

    fn from_catalog(
        session: WorldSession,
        with_rover: bool,
        with_hulls: bool,
    ) -> Result<Self, BackendError> {
        let drone = session.attach_takeoff("drone")?;
        let rover = if with_rover {
            Some(session.attach_drive("rover")?)
        } else {
            None
        };
        let (skiff, surveyor) = if with_hulls {
            (
                Some(session.attach_undock("skiff")?),
                Some(session.attach_undock("surveyor")?),
            )
        } else {
            (None, None)
        };
        Ok(Self {
            session,
            drone,
            rover,
            skiff,
            surveyor,
            spec: DeadlineSpec::HZ_50,
            rate: Rate::HZ_50,
            last_missed: false,
            missed: 0,
            frames: 0,
            out: None,
        })
    }

    pub fn with_spec(mut self, spec: DeadlineSpec) -> Self {
        if spec.valid() {
            self.spec = spec;
            self.rate = Rate::from_period_ns(spec.period_ns);
        }
        self
    }

    /// Bind a UDP socket and send `FCH1` samples each frame (best-effort).
    pub fn mirror_udp(&mut self, addr: &str) -> Result<(), BackendError> {
        let sock = UdpSocket::bind("127.0.0.1:0").map_err(|_| BackendError::Io)?;
        sock.connect(addr).map_err(|_| BackendError::Io)?;
        sock.set_nonblocking(true).map_err(|_| BackendError::Io)?;
        self.out = Some(sock);
        Ok(())
    }

    pub fn spec(&self) -> DeadlineSpec {
        self.spec
    }

    /// Named OffboardControl rate this rack admits against (lockstep `period_ns`).
    pub fn rate(&self) -> Rate {
        self.rate
    }

    pub fn missed(&self) -> u64 {
        self.missed
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn world(&self) -> World {
        self.session.world()
    }

    pub fn session(&self) -> &WorldSession {
        &self.session
    }

    /// Recover after a deadline miss: aerial Ready, rover Parked, hulls Docked.
    /// Clears the miss latch so the next on-time frame can apply commands
    /// once [`Self::grant_all`] re-walks takeoff / drive / undock.
    /// Already-recovered machines are [`BackendError::Protocol`].
    pub fn recover_deadline(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_recover_ready("drone")?;
        reset_rover(&self.session, &mut self.rover)?;
        recover_hull(&self.session, &mut self.skiff, "skiff")?;
        recover_hull(&self.session, &mut self.surveyor, "surveyor")?;
        self.last_missed = false;
        Ok(())
    }

    /// Re-walk takeoff / drive / undock after [`Self::recover_deadline`].
    pub fn grant_all(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_takeoff("drone")?;
        drive_rover(&self.session, &mut self.rover)?;
        undock_hull(&self.session, &mut self.skiff, "skiff")?;
        undock_hull(&self.session, &mut self.surveyor, "surveyor")?;
        Ok(())
    }

    /// Land then touchdown, park the rover if present, dock hulls if present.
    /// Call after grant. Already-home is [`BackendError::Protocol`].
    pub fn return_all(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_land("drone")?;
        self.drone = self.session.attach_touchdown("drone")?;
        park_rover(&self.session, &mut self.rover)?;
        dock_hull(&self.session, &mut self.skiff, "skiff")?;
        dock_hull(&self.session, &mut self.surveyor, "surveyor")?;
        Ok(())
    }

    /// Takeoff → Airborne. Ready, Offboard, Airborne, and Landing are Protocol.
    pub fn airborne(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_airborne("drone")?;
        Ok(())
    }

    /// Hold station on every hull the catalog included.
    /// Inland (no hull) and already-station / docked are Protocol.
    pub fn station_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        station_hull(&self.session, &mut self.skiff, "skiff")?;
        station_hull(&self.session, &mut self.surveyor, "surveyor")?;
        Ok(())
    }

    /// Resume Underway on every hull the catalog included.
    /// Inland (no hull) and already-underway / docked are Protocol.
    pub fn resume_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        resume_hull(&self.session, &mut self.skiff, "skiff")?;
        resume_hull(&self.session, &mut self.surveyor, "surveyor")?;
        Ok(())
    }

    /// Dock every hull the catalog included (Underway or StationKeep).
    /// Inland (no hull) and already-docked / failsafe are Protocol.
    pub fn dock_all(&mut self) -> Result<(), BackendError> {
        if self.skiff.is_none() && self.surveyor.is_none() {
            return Err(BackendError::Protocol);
        }
        dock_hull(&self.session, &mut self.skiff, "skiff")?;
        dock_hull(&self.session, &mut self.surveyor, "surveyor")?;
        Ok(())
    }

    /// Halt the rover if the catalog included one.
    /// Open water (no rover) and already-parked / e-stop are Protocol.
    pub fn park_all(&mut self) -> Result<(), BackendError> {
        if self.rover.is_none() {
            return Err(BackendError::Protocol);
        }
        park_rover(&self.session, &mut self.rover)?;
        Ok(())
    }

    /// Hold the drone at its current NED pose. OffboardControl only;
    /// Ready / Armed / Failsafe / Recovery are [`BackendError::Protocol`].
    pub fn hold(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_hold("drone")?;
        Ok(())
    }

    /// One rack frame using wall time for the deadline.
    pub fn frame(&mut self, dt: f32, cmd: RackCommand) -> Result<RackFrame, BackendError> {
        let t0 = Instant::now();
        let report = self.frame_timed(dt, cmd, None)?;
        let compute_ns = t0.elapsed().as_nanos() as u64;
        self.finish(report, compute_ns)
    }

    /// One rack frame with an injected compute duration (tests, replay).
    pub fn frame_with_compute(
        &mut self,
        dt: f32,
        cmd: RackCommand,
        compute_ns: u64,
    ) -> Result<RackFrame, BackendError> {
        let report = self.frame_timed(dt, cmd, Some(compute_ns))?;
        self.finish(report, compute_ns)
    }

    fn frame_timed(
        &mut self,
        dt: f32,
        cmd: RackCommand,
        compute_ns: Option<u64>,
    ) -> Result<Inner, BackendError> {
        let missed = self.last_missed;
        let aerial = command_after_deadline(missed, cmd.aerial);
        let ground = command_after_deadline(missed, cmd.ground);
        let marine = command_after_deadline(missed, cmd.marine);
        let underwater = command_after_deadline(missed, cmd.underwater);
        debug_assert!(hitl_apply_allowed(missed) || aerial == [0.0, 0.0, 0.0]);

        if !missed {
            self.write_granted_setpoints(aerial, ground, marine, underwater)?;
        }

        self.drone.flush()?;
        if let Some(rover) = self.rover.as_ref() {
            rover.flush()?;
        }
        if let Some(skiff) = self.skiff.as_ref() {
            skiff.flush()?;
        }
        if let Some(surveyor) = self.surveyor.as_ref() {
            surveyor.flush()?;
        }
        self.session.step(dt)?;
        self.emit_samples();

        Ok(Inner {
            applied_aerial: aerial,
            compute_override: compute_ns,
        })
    }

    /// Write NED setpoints only while attach is an offboard-control / Moving /
    /// Underway kind. Failsafe after a miss attaches Failsafe / EStopped /
    /// marine Failsafe, so this is a no-op and the rack still steps.
    fn write_granted_setpoints(
        &mut self,
        aerial: [f32; 3],
        ground: [f32; 3],
        marine: [f32; 3],
        underwater: [f32; 3],
    ) -> Result<(), BackendError> {
        match self.session.aerial("drone").attach()? {
            VehicleHandle::Offboard(_)
            | VehicleHandle::Takeoff(_)
            | VehicleHandle::Airborne(_)
            | VehicleHandle::Landing(_) => {
                let holding = self
                    .session
                    .world()
                    .body("drone")
                    .is_some_and(|b| b.hold_ned.is_some());
                let velocity_live = aerial.iter().copied().any(|c| c.abs() > 1e-6);
                if velocity_live || !holding {
                    self.drone
                        .set_velocity_now(Velocity::ned(aerial[0], aerial[1], aerial[2]))?;
                }
            }
            _ => {}
        }
        write_rover(&self.session, &mut self.rover, ground)?;
        write_hull(&self.session, &mut self.skiff, "skiff", marine)?;
        write_hull(&self.session, &mut self.surveyor, "surveyor", underwater)?;
        Ok(())
    }

    fn finish(&mut self, inner: Inner, compute_ns: u64) -> Result<RackFrame, BackendError> {
        let ns = inner.compute_override.unwrap_or(compute_ns);
        let outcome = deadline_outcome(ns, self.spec);
        let due = flight_core::temporal::Deadline::at(
            flight_core::time::MonotonicInstant::from_nanos(self.spec.budget_ns),
        );
        let compute = flight_core::time::MonotonicInstant::from_nanos(ns);
        // Fail closed: typed Deadline, kernel DeadlineSpec, and Rate period
        // must agree. OffboardControl ⇒ this loop rate.
        let rate_ok = self.rate.period_ns() == self.spec.period_ns && self.rate.admits(ns);
        let missed = outcome.missed() || !due.met(compute) || !rate_ok;
        if missed {
            self.missed = self.missed.saturating_add(1);
            self.last_missed = true;
            self.trip_deadline_failsafe()?;
        } else {
            self.last_missed = false;
        }
        self.frames = self.frames.saturating_add(1);
        let world = self.session.world();
        Ok(RackFrame {
            t: world.t,
            compute_ns: ns,
            outcome: if missed && !outcome.missed() {
                DeadlineOutcome::Missed {
                    compute_ns: ns,
                    budget_ns: self.spec.budget_ns,
                }
            } else {
                outcome
            },
            applied_aerial: inner.applied_aerial,
            all_hold: world.all_hold(),
            missed_total: self.missed,
        })
    }

    /// First miss walks attach typestate. Later misses are already Failsafe /
    /// E-stopped, so attach returns Protocol and `failsafe_now` re-trips.
    fn trip_deadline_failsafe(&mut self) -> Result<(), BackendError> {
        match self.session.attach_failsafe("drone") {
            Ok(drone) => self.drone = drone,
            Err(BackendError::Protocol) => self.drone.failsafe_now()?,
            Err(e) => return Err(e),
        }
        if self.rover.is_some() {
            match self.session.attach_estop("rover") {
                Ok(rover) => self.rover = Some(rover),
                Err(BackendError::Protocol) => {
                    if let Some(rover) = self.rover.as_mut() {
                        rover.failsafe_now()?;
                    }
                }
                Err(e) => return Err(e),
            }
        }
        if self.skiff.is_some() {
            match self.session.attach_marine_failsafe("skiff") {
                Ok(skiff) => self.skiff = Some(skiff),
                Err(BackendError::Protocol) => {
                    if let Some(skiff) = self.skiff.as_mut() {
                        skiff.failsafe_now()?;
                    }
                }
                Err(e) => return Err(e),
            }
        }
        if self.surveyor.is_some() {
            match self.session.attach_marine_failsafe("surveyor") {
                Ok(surveyor) => self.surveyor = Some(surveyor),
                Err(BackendError::Protocol) => {
                    if let Some(surveyor) = self.surveyor.as_mut() {
                        surveyor.failsafe_now()?;
                    }
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn emit_samples(&self) {
        let Some(sock) = self.out.as_ref() else {
            return;
        };
        let world = self.session.world();
        let t_ns = (world.t * 1e9) as u64;
        for (slot, id) in [(0u8, "drone"), (1, "rover"), (2, "skiff"), (3, "surveyor")] {
            let Some(b) = world.body(id) else {
                continue;
            };
            let pkt = protocol::encode_sample(Sample {
                slot,
                t_plant_ns: t_ns,
                position_ned: b.position_m,
                velocity_ned: b.velocity_mps,
            });
            let _ = sock.send(&pkt);
        }
    }
}

struct Inner {
    applied_aerial: [f32; 3],
    compute_override: Option<u64>,
}

fn recover_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_recover(id)?);
    }
    Ok(())
}

fn undock_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_undock(id)?);
    }
    Ok(())
}

fn dock_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_dock(id)?);
    }
    Ok(())
}

fn station_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_station(id)?);
    }
    Ok(())
}

fn resume_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_resume(id)?);
    }
    Ok(())
}

fn write_hull(
    session: &WorldSession,
    slot: &mut Option<MarineWorldBackend>,
    id: &'static str,
    v: [f32; 3],
) -> Result<(), BackendError> {
    let Some(hull) = slot.as_mut() else {
        return Ok(());
    };
    match session.marine(id).attach()? {
        MarineHandle::Underway(_) | MarineHandle::StationKeep(_) => {
            hull.set_velocity_now(Velocity::ned(v[0], v[1], v[2]))?;
        }
        _ => {}
    }
    Ok(())
}

fn reset_rover(
    session: &WorldSession,
    slot: &mut Option<GroundWorldBackend>,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_reset("rover")?);
    }
    Ok(())
}

fn drive_rover(
    session: &WorldSession,
    slot: &mut Option<GroundWorldBackend>,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_drive("rover")?);
    }
    Ok(())
}

fn park_rover(
    session: &WorldSession,
    slot: &mut Option<GroundWorldBackend>,
) -> Result<(), BackendError> {
    if slot.is_some() {
        *slot = Some(session.attach_park("rover")?);
    }
    Ok(())
}

fn write_rover(
    session: &WorldSession,
    slot: &mut Option<GroundWorldBackend>,
    v: [f32; 3],
) -> Result<(), BackendError> {
    let Some(rover) = slot.as_mut() else {
        return Ok(());
    };
    if let GroundHandle::Moving(_) = session.ground("rover").attach()? {
        rover.set_velocity_now(Velocity::ned(v[0], v[1], v[2]))?;
    }
    Ok(())
}

fn drone_trace(world: &World) -> Result<TraceSample, BackendError> {
    let body = world.body("drone").ok_or(BackendError::Protocol)?;
    let aerial = body.aerial.ok_or(BackendError::Protocol)?;
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

impl WorldRack {
    /// On-time frame then a deadline miss. Same contract evaluator as
    /// `flight-test --backend hitl` (`Scenario::HITL_MISS`).
    ///
    /// Returns the sample before the miss and after, so
    /// [`flight_core::contracts::Requirement::EpochBumped`] can compare
    /// against the live epoch.
    pub fn contract_deadline_miss(seed: u64) -> Result<Vec<TraceSample>, BackendError> {
        let mut rack = Self::inland(seed)?;
        let cmd = RackCommand {
            aerial: [0.0, 0.0, -1.2],
            ..RackCommand::default()
        };
        rack.frame_with_compute(0.02, cmd, 1_000_000)?;
        let before = drone_trace(&rack.world())?;
        rack.frame_with_compute(0.02, cmd, 50_000_000)?;
        Ok(vec![before, drone_trace(&rack.world())?])
    }

    /// Bind leftover OffboardControl (inland grant is Takeoff) before a rack
    /// deadline miss. After the miss trips failsafe, every generated
    /// `COMMANDS` method is `StaleAuthority` while the handle is still typed
    /// OffboardControl. HITL-shaped leftover — not a clone of world
    /// `run_revoke_table`.
    pub fn leftover_after_deadline_miss(seed: u64) -> Result<(), BackendError> {
        let mut rack = Self::inland(seed)?;
        if rack.rate().period_ns() != rack.spec().period_ns {
            return Err(BackendError::Rejected("rate_period_lockstep"));
        }
        let VehicleHandle::Takeoff(mut leftover) = rack.session.aerial("drone").attach()? else {
            return Err(BackendError::Protocol);
        };
        if leftover.leftover_commands_stale().is_ok() {
            return Err(BackendError::Rejected("leftover_already_stale"));
        }
        let cmd = RackCommand {
            aerial: [0.0, 0.0, -1.2],
            ..RackCommand::default()
        };
        rack.frame_with_compute(0.02, cmd, 1_000_000)?;
        if leftover.leftover_commands_stale().is_ok() {
            return Err(BackendError::Rejected("leftover_stale_after_on_time_frame"));
        }
        rack.frame_with_compute(0.02, cmd, 50_000_000)?;
        leftover
            .leftover_commands_stale()
            .map_err(|_| BackendError::Rejected("leftover_offboard_still_has_authority"))?;
        Ok(())
    }

    /// Same leftover OffboardControl `COMMANDS` check as world / PX4, after a
    /// rack deadline miss. Lives here because `flight-sim` cannot depend on
    /// this crate (cycle: this crate already uses `flight-sim` for the plant).
    pub fn run_hitl_leftover_deadline_miss() -> Result<usize, String> {
        Self::leftover_after_deadline_miss(1).map_err(|e| format!("hitl leftover: {e}"))?;
        Ok(1)
    }

    /// Companion-shaped inject of a kernel revoke event onto the rack drone.
    /// [`WorldSession::inject_revoke`] first; leftover OffboardControl bound
    /// from this rack's grant must then fail `leftover_commands_stale`.
    pub fn inject_revoke(&self, event: Event) -> Result<(), BackendError> {
        self.session.inject_revoke("drone", event)
    }

    /// Same leftover OffboardControl `COMMANDS` check as world / PX4, for every
    /// `REVOKE_ON` event, bound from the inland rack Takeoff grant. Epoch
    /// monotonicity is a first-class [`Sequence`]. Lives here because
    /// `flight-sim` cannot depend on this crate.
    pub fn run_hitl_revoke_table() -> Result<usize, String> {
        let mut n = 0;
        for e in AerialOffboard::REVOKE_ON {
            let rack = Self::inland(1).map_err(|err| format!("rack before {e:?}: {err}"))?;
            let VehicleHandle::Takeoff(mut leftover) = rack
                .session
                .aerial("drone")
                .attach()
                .map_err(|err| format!("bind Takeoff before {e:?}: {err}"))?
            else {
                return Err(format!("inland grant must bind Takeoff before {e:?}"));
            };
            if leftover.leftover_commands_stale().is_ok() {
                return Err(format!("leftover already stale before HITL inject {e:?}"));
            }
            let mut seq = Sequence::new();
            seq.observe(leftover.backend().authority_epoch())
                .map_err(|_| format!("sequence before {e:?}"))?;
            rack.inject_revoke(*e)
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
}

/// Decode a companion command datagram (hardware rack → this process).
pub fn command_from_datagram(buf: &[u8]) -> Option<Command> {
    protocol::decode_command(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn climb() -> RackCommand {
        RackCommand {
            aerial: [0.0, 0.0, -1.2],
            ground: [-0.4, 0.0, 0.0],
            marine: [0.0, 0.3, 0.0],
            underwater: [0.25, 0.0, 0.0],
        }
    }

    #[test]
    fn coastal_walks_consume_self_typestate() {
        use flight_core::ground::GroundPhase;
        use flight_core::marine::MarinePhase;
        use flight_core::safety::Phase;

        let rack = WorldRack::coastal(1).expect("rack");
        let w = rack.world();
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        assert_eq!(
            w.body("rover").unwrap().ground.unwrap().phase,
            GroundPhase::Moving
        );
        assert_eq!(
            w.body("skiff").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
        assert_eq!(
            w.body("surveyor").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
    }

    #[test]
    fn inland_walks_air_and_ground_without_a_hull() {
        use flight_core::ground::GroundPhase;
        use flight_core::safety::Phase;

        let mut rack = WorldRack::inland(1).expect("rack");
        let w = rack.world();
        assert!(w.body("skiff").is_none());
        assert!(w.body("surveyor").is_none());
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        assert_eq!(
            w.body("rover").unwrap().ground.unwrap().phase,
            GroundPhase::Moving
        );
        let n0 = w.body("rover").unwrap().position_m[0];
        let mut last_alt = 0.0;
        for _ in 0..40 {
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect("frame");
            assert!(!f.missed(), "{:?}", f.outcome);
            assert!(f.all_hold);
            last_alt = rack.world().body("drone").unwrap().altitude_agl();
        }
        let n1 = rack.world().body("rover").unwrap().position_m[0];
        assert!(last_alt > 0.4, "alt={last_alt}");
        assert!(n1 < n0 - 0.1, "rover south {n0} → {n1}");
        assert_eq!(rack.missed(), 0);
    }

    #[test]
    fn harbor_walks_four_bodies_on_the_shoreline() {
        use flight_core::ground::GroundPhase;
        use flight_core::marine::MarinePhase;
        use flight_core::safety::Phase;

        let mut rack = WorldRack::harbor(1).expect("rack");
        let w = rack.world();
        assert_eq!(w.scenario, "harbor");
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        assert_eq!(
            w.body("rover").unwrap().ground.unwrap().phase,
            GroundPhase::Moving
        );
        assert_eq!(
            w.body("skiff").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
        assert_eq!(
            w.body("surveyor").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("frame");
        assert!(!f.missed(), "{:?}", f.outcome);
        assert!(f.all_hold);
    }

    #[test]
    fn open_water_walks_air_and_hulls_without_a_rover() {
        use flight_core::marine::MarinePhase;
        use flight_core::safety::Phase;

        let mut rack = WorldRack::open_water(1).expect("rack");
        let w = rack.world();
        assert_eq!(w.scenario, "open_water");
        assert!(w.body("rover").is_none());
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        assert_eq!(
            w.body("skiff").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
        assert_eq!(
            w.body("surveyor").unwrap().marine.unwrap().phase,
            MarinePhase::Underway
        );
        let alt0 = w.body("drone").unwrap().altitude_agl();
        let e0 = w.body("skiff").unwrap().position_m[1];
        let mut last_alt = alt0;
        for _ in 0..40 {
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect("frame");
            assert!(!f.missed(), "{:?}", f.outcome);
            assert!(f.all_hold);
            last_alt = rack.world().body("drone").unwrap().altitude_agl();
        }
        let e1 = rack.world().body("skiff").unwrap().position_m[1];
        assert!(last_alt > alt0 + 0.3, "alt {alt0} → {last_alt}");
        assert!(e1 > e0 + 0.08, "skiff east {e0} → {e1}");
        assert!(rack.world().body("rover").is_none());
        assert_eq!(rack.missed(), 0);
    }

    #[test]
    fn on_time_frames_hold_and_climb() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        let mut last_alt = 0.0;
        for _ in 0..40 {
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect("frame");
            assert!(!f.missed(), "{:?}", f.outcome);
            assert!(f.all_hold);
            assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
            last_alt = rack.world().body("drone").unwrap().altitude_agl();
        }
        assert!(last_alt > 0.4, "alt={last_alt}");
        assert_eq!(rack.missed(), 0);
    }

    #[test]
    fn injected_miss_zeros_command_and_trips_failsafe() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.frame_with_compute(0.02, climb(), 1_000_000)
            .expect("warm");
        let miss = rack
            .frame_with_compute(0.02, climb(), 50_000_000)
            .expect("miss");
        assert!(miss.missed());
        assert_eq!(miss.applied_aerial, [0.0, 0.0, -1.2]);
        let w = rack.world();
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Failsafe
        );
        assert_eq!(
            w.body("rover").unwrap().ground.unwrap().phase,
            flight_core::ground::GroundPhase::EStop
        );
        assert_eq!(
            w.body("skiff").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Failsafe
        );
        assert_eq!(
            w.body("surveyor").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Failsafe
        );
        assert!(w.body("drone").unwrap().failsafe());
        assert!(w.body("drone").unwrap().authority_epoch > 0);
        let drone = w.body("drone").unwrap();
        let aerial = drone.aerial.unwrap();
        let sample = flight_core::contracts::TraceSample {
            t_secs: w.t,
            armed: aerial.armed,
            actuators_enabled: aerial.actuators_enabled,
            failsafe: aerial.failsafe,
            epoch: drone.authority_epoch,
            heartbeat_age_ms: 0,
            command: drone.command,
            altitude_m: drone.altitude_agl(),
            command_age_ms: 0,
            estimator_ts_ms: (w.t * 1000.0) as u64,
        };
        flight_core::contracts::evaluate_trace(
            &[sample],
            &[
                flight_core::contracts::Requirement::ActuatorsImplyArmed,
                flight_core::contracts::Requirement::NeverActuateWhileDisarmed,
            ],
        )
        .expect("HITL miss still satisfies the contract monitors");
        let next = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("after miss");
        assert_eq!(next.applied_aerial, [0.0, 0.0, 0.0]);
        assert!(next.all_hold);
        let later = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("failsafe rack still steps");
        assert!(rack.world().body("drone").unwrap().failsafe());
        assert!(later.all_hold);
    }

    #[test]
    fn recover_deadline_then_grant_all_commands_again() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.frame_with_compute(0.02, climb(), 1_000_000)
            .expect("warm");
        let miss = rack
            .frame_with_compute(0.02, climb(), 50_000_000)
            .expect("miss");
        assert!(miss.missed());
        rack.recover_deadline().expect("recover");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match rack.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match rack.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(
            rack.recover_deadline(),
            Err(BackendError::Protocol)
        ));
        rack.grant_all().expect("re-grant");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("re-grant drone {:?}", other.kind()),
        }
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("after recover");
        assert!(!f.missed());
        assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
        assert!(f.all_hold);
        assert!(!rack.world().body("drone").unwrap().failsafe());
    }

    #[test]
    fn inland_recover_deadline_has_no_hull() {
        let mut rack = WorldRack::inland(1).expect("rack");
        rack.frame_with_compute(0.02, climb(), 1_000_000)
            .expect("warm");
        let miss = rack
            .frame_with_compute(0.02, climb(), 50_000_000)
            .expect("miss");
        assert!(miss.missed());
        assert!(rack.world().body("skiff").is_none());
        assert!(rack.world().body("surveyor").is_none());
        rack.recover_deadline().expect("recover");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        rack.grant_all().expect("re-grant");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("re-grant drone {:?}", other.kind()),
        }
        assert!(rack.world().body("skiff").is_none());
        assert!(rack.world().body("surveyor").is_none());
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("after recover");
        assert!(!f.missed());
        assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
        assert!(f.all_hold);
    }

    #[test]
    fn open_water_injected_miss_trips_hulls_not_rover() {
        let mut rack = WorldRack::open_water(1).expect("rack");
        rack.frame_with_compute(0.02, climb(), 1_000_000)
            .expect("warm");
        let miss = rack
            .frame_with_compute(0.02, climb(), 50_000_000)
            .expect("miss");
        assert!(miss.missed());
        let w = rack.world();
        assert!(w.body("rover").is_none());
        assert_eq!(
            w.body("drone").unwrap().aerial.unwrap().phase,
            flight_core::safety::Phase::Failsafe
        );
        assert_eq!(
            w.body("skiff").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Failsafe
        );
        assert_eq!(
            w.body("surveyor").unwrap().marine.unwrap().phase,
            flight_core::marine::MarinePhase::Failsafe
        );
        rack.recover_deadline().expect("recover");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        assert!(rack.world().body("rover").is_none());
        rack.grant_all().expect("re-grant");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("re-grant drone {:?}", other.kind()),
        }
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("after recover");
        assert!(!f.missed());
        assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
        assert!(f.all_hold);
    }

    #[test]
    fn return_all_walks_home_then_protocol() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.return_all().expect("return");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match rack.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match rack.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(rack.return_all(), Err(BackendError::Protocol)));
        let alt = rack.world().body("drone").unwrap().altitude_agl();
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("home rack still steps");
        assert!(!f.missed());
        assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
        assert!(f.all_hold);
        let alt1 = rack.world().body("drone").unwrap().altitude_agl();
        assert!(
            (alt1 - alt).abs() < 0.15,
            "ready drone must not climb {alt} → {alt1}"
        );
        rack.grant_all().expect("re-grant after return");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("re-grant drone {:?}", other.kind()),
        }
    }

    #[test]
    fn inland_return_all_has_no_hull() {
        let mut rack = WorldRack::inland(1).expect("rack");
        assert!(rack.world().body("skiff").is_none());
        assert!(rack.world().body("surveyor").is_none());
        rack.return_all().expect("return");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().ground("rover").attach().unwrap() {
            GroundHandle::Parked(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        assert!(rack.world().body("skiff").is_none());
        assert!(rack.world().body("surveyor").is_none());
        assert!(matches!(rack.return_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn open_water_return_all_has_no_rover() {
        let mut rack = WorldRack::open_water(1).expect("rack");
        assert!(rack.world().body("rover").is_none());
        rack.return_all().expect("return");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match rack.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match rack.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(rack.world().body("rover").is_none());
        assert!(matches!(rack.return_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn airborne_walks_takeoff_then_protocol() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.airborne().expect("airborne");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        assert!(matches!(rack.airborne(), Err(BackendError::Protocol)));
        let f = rack
            .frame_with_compute(0.02, climb(), 1_000_000)
            .expect("airborne rack still steps");
        assert!(!f.missed());
        assert_eq!(f.applied_aerial, [0.0, 0.0, -1.2]);
        assert!(f.all_hold);
    }

    #[test]
    fn station_all_then_resume_all_on_water_worlds() {
        for mut rack in [
            WorldRack::coastal(1).expect("coastal"),
            WorldRack::harbor(1).expect("harbor"),
            WorldRack::open_water(1).expect("open_water"),
        ] {
            let name = rack.world().scenario;
            rack.station_all().expect(name);
            match rack.session().marine("skiff").attach().unwrap() {
                MarineHandle::StationKeep(_) => {}
                other => panic!("{name} skiff {:?}", other.kind()),
            }
            match rack.session().marine("surveyor").attach().unwrap() {
                MarineHandle::StationKeep(_) => {}
                other => panic!("{name} surveyor {:?}", other.kind()),
            }
            assert!(
                matches!(rack.station_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            rack.resume_all().expect(name);
            match rack.session().marine("skiff").attach().unwrap() {
                MarineHandle::Underway(_) => {}
                other => panic!("{name} skiff {:?}", other.kind()),
            }
            match rack.session().marine("surveyor").attach().unwrap() {
                MarineHandle::Underway(_) => {}
                other => panic!("{name} surveyor {:?}", other.kind()),
            }
            assert!(
                matches!(rack.resume_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect(name);
            assert!(!f.missed(), "{name}");
            assert!(f.all_hold, "{name}");
        }
    }

    #[test]
    fn inland_station_and_resume_are_protocol() {
        let mut rack = WorldRack::inland(1).expect("rack");
        assert!(matches!(rack.station_all(), Err(BackendError::Protocol)));
        assert!(matches!(rack.resume_all(), Err(BackendError::Protocol)));
        rack.airborne().expect("inland drone still takeoff");
        match rack.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
    }

    #[test]
    fn dock_all_walks_underway_then_protocol() {
        for mut rack in [
            WorldRack::coastal(1).expect("coastal"),
            WorldRack::harbor(1).expect("harbor"),
            WorldRack::open_water(1).expect("open_water"),
        ] {
            let name = rack.world().scenario;
            rack.dock_all().expect(name);
            match rack.session().marine("skiff").attach().unwrap() {
                MarineHandle::Docked(_) => {}
                other => panic!("{name} skiff {:?}", other.kind()),
            }
            match rack.session().marine("surveyor").attach().unwrap() {
                MarineHandle::Docked(_) => {}
                other => panic!("{name} surveyor {:?}", other.kind()),
            }
            assert!(
                matches!(rack.dock_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect(name);
            assert!(!f.missed(), "{name}");
            assert!(f.all_hold, "{name}");
        }
    }

    #[test]
    fn inland_dock_all_is_protocol() {
        let mut rack = WorldRack::inland(1).expect("rack");
        assert!(matches!(rack.dock_all(), Err(BackendError::Protocol)));
        assert!(matches!(rack.station_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn dock_all_from_station_keep() {
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.station_all().expect("station");
        rack.dock_all().expect("dock from station");
        match rack.session().marine("skiff").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        match rack.session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(matches!(rack.dock_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn park_all_walks_moving_then_protocol() {
        for mut rack in [
            WorldRack::coastal(1).expect("coastal"),
            WorldRack::harbor(1).expect("harbor"),
            WorldRack::inland(1).expect("inland"),
        ] {
            let name = rack.world().scenario;
            rack.park_all().expect(name);
            match rack.session().ground("rover").attach().unwrap() {
                GroundHandle::Parked(_) => {}
                other => panic!("{name} rover {:?}", other.kind()),
            }
            assert!(
                matches!(rack.park_all(), Err(BackendError::Protocol)),
                "{name}"
            );
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect(name);
            assert!(!f.missed(), "{name}");
            assert!(f.all_hold, "{name}");
        }
    }

    #[test]
    fn open_water_park_all_is_protocol() {
        let mut rack = WorldRack::open_water(1).expect("rack");
        assert!(matches!(rack.park_all(), Err(BackendError::Protocol)));
    }

    #[test]
    fn hold_sets_ned_pose_and_zero_frame_keeps_it() {
        for mut rack in [
            WorldRack::inland(1).expect("inland"),
            WorldRack::coastal(1).expect("coastal"),
            WorldRack::harbor(1).expect("harbor"),
            WorldRack::open_water(1).expect("open_water"),
        ] {
            let name = rack.world().scenario;
            rack.hold().expect(name);
            let pose = rack.world().body("drone").unwrap().position_m;
            assert_eq!(
                rack.world().body("drone").unwrap().hold_ned,
                Some(pose),
                "{name}"
            );
            let idle = RackCommand::default();
            let f = rack.frame_with_compute(0.02, idle, 1_000_000).expect(name);
            assert!(!f.missed(), "{name}");
            assert!(f.all_hold, "{name}");
            assert!(
                rack.world().body("drone").unwrap().hold_ned.is_some(),
                "{name} zero command must not clear a hold"
            );
            let f = rack
                .frame_with_compute(0.02, climb(), 1_000_000)
                .expect(name);
            assert!(f.all_hold, "{name}");
            assert!(
                rack.world().body("drone").unwrap().hold_ned.is_none(),
                "{name} live velocity must win"
            );
        }
    }

    #[test]
    fn decoded_apply_zero_does_not_revive_hold() {
        let mut rack = WorldRack::inland(1).expect("rack");
        rack.hold().expect("hold");
        let hold = rack.world().body("drone").unwrap().hold_ned;
        assert!(hold.is_some());
        let wire = protocol::encode_command(Command {
            slot: 0,
            velocity_ned: [0.0, 3.0, -5.0],
            apply: 0,
        });
        let decoded = command_from_datagram(&wire).expect("decode");
        assert_eq!(decoded.apply, 0);
        let cmd = RackCommand::from_fch1(&[decoded]);
        assert_eq!(cmd.aerial, [0.0, 0.0, 0.0]);
        let f = rack
            .frame_with_compute(0.02, cmd, 1_000_000)
            .expect("frame");
        assert!(!f.missed());
        assert!(f.all_hold);
        assert_eq!(rack.world().body("drone").unwrap().hold_ned, hold);
    }

    #[test]
    fn hold_after_return_is_protocol() {
        let mut rack = WorldRack::inland(1).expect("rack");
        rack.return_all().expect("return");
        assert!(matches!(rack.hold(), Err(BackendError::Protocol)));
    }

    #[test]
    fn udp_mirror_sends_magic() {
        let listener = UdpSocket::bind("127.0.0.1:0").unwrap();
        listener
            .set_read_timeout(Some(std::time::Duration::from_millis(400)))
            .unwrap();
        let addr = listener.local_addr().unwrap();
        let mut rack = WorldRack::coastal(1).expect("rack");
        rack.mirror_udp(&addr.to_string()).expect("mirror");
        rack.frame_with_compute(0.02, climb(), 1_000_000)
            .expect("frame");
        let mut buf = [0u8; 64];
        let n = listener.recv(&mut buf).expect("datagram");
        let s = protocol::decode_sample(&buf[..n]).expect("sample");
        assert_eq!(s.slot, 0);
        assert!(s.position_ned.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn contract_deadline_miss_satisfies_hitl_miss_require() {
        let samples = WorldRack::contract_deadline_miss(1).expect("miss");
        assert!(samples.len() >= 2);
        assert!(samples.last().is_some_and(|s| s.failsafe));
        assert!(samples.last().is_some_and(|s| s.epoch > samples[0].epoch));
        flight_core::contracts::evaluate_trace(&samples, flight_sim::Scenario::HITL_MISS.require)
            .expect("HITL rack miss satisfies the same contract as flight-test --backend hitl");
    }

    #[test]
    fn rate_lockstep_with_deadline_spec() {
        let rack = WorldRack::inland(1).expect("rack");
        assert_eq!(rack.rate(), Rate::HZ_50);
        assert_eq!(rack.rate().period_ns(), rack.spec().period_ns);
        assert_eq!(rack.spec().rate(), Rate::HZ_50);
        assert!(rack.rate().admits(1_000_000));
        assert!(!rack.rate().admits(50_000_000));
    }

    #[test]
    fn leftover_commands_stale_after_deadline_miss() {
        WorldRack::leftover_after_deadline_miss(1).expect("leftover after miss");
        assert_eq!(
            WorldRack::run_hitl_leftover_deadline_miss().expect("runner"),
            1
        );
    }

    #[test]
    fn leftover_commands_stale_after_every_dsl_revoke() {
        let n = WorldRack::run_hitl_revoke_table().expect("hitl leftover revoke table");
        assert_eq!(n, AerialOffboard::REVOKE_ON.len());
    }
}
