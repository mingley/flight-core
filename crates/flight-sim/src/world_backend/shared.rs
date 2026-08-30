use std::collections::HashMap;

use flight_core::domain::Domain;
use flight_core::frames::Body as BodyFrame;
use flight_core::ground::{ground_event_revokes_authority, ground_step, GroundEvent};
use flight_core::marine::{marine_event_revokes_authority, marine_step, MarineEvent, MarinePhase};
use flight_core::mech::quat_rotate_inv;
use flight_core::nav::{imu_trips_estimator, ComplementaryAttitude};
use flight_core::safety::{self, event_revokes_authority, Event, Phase};
use flight_core::sensors::{ImuSample, SensorHealth};
use flight_core::time::{Clock, MonotonicInstant, VirtualClock};
use flight_core::vector::{Acceleration, AngularVelocity, Position, Velocity};
use flight_core::vehicle::{BackendError, PreflightNotes, PreflightReport, Telemetry};
use robot_world::{Body, World};

use super::session::WorldSession;

#[derive(Clone, Copy, Debug)]
pub(crate) enum Setpoint {
    Velocity(Velocity<flight_core::frames::Ned>),
    Position(Position<flight_core::frames::Ned>),
}

pub(crate) struct Plant {
    pub(crate) world: World,
    pub(crate) clock: VirtualClock,
    pub(crate) attitude: HashMap<&'static str, ComplementaryAttitude>,
}

impl std::fmt::Debug for Plant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Plant")
            .field("scenario", &self.world.scenario)
            .field("t", &self.world.t)
            .field("bodies", &self.world.bodies.len())
            .finish()
    }
}

pub(crate) fn clamp_dt(dt: f32) -> f32 {
    if dt > 0.0 && dt < 1.0 {
        dt
    } else {
        0.02
    }
}

pub(crate) fn apply_setpoint(body: &mut Body, sp: Option<Setpoint>, yaw_rate: Option<f32>) {
    if let Some(yaw) = yaw_rate {
        body.yaw_cmd = yaw;
    }
    match sp {
        Some(Setpoint::Velocity(v)) => {
            body.set_velocity_command([v.x(), v.y(), v.z()]);
        }
        Some(Setpoint::Position(p)) => {
            body.set_position_hold([p.x(), p.y(), p.z()]);
        }
        None => {}
    }
}

pub(crate) fn require_body<'a>(world: &'a World, id: &str) -> Result<&'a Body, BackendError> {
    world.body(id).ok_or(BackendError::Disconnected)
}

pub(crate) fn require_body_mut<'a>(
    world: &'a mut World,
    id: &str,
) -> Result<&'a mut Body, BackendError> {
    world.body_mut(id).ok_or(BackendError::Disconnected)
}

pub(crate) fn aerial_event(
    session: &WorldSession,
    id: &'static str,
    e: Event,
) -> Result<(), BackendError> {
    let mut plant = session.lock();
    let body = require_body_mut(&mut plant.world, id)?;
    let s = body.aerial.ok_or(BackendError::Protocol)?;
    let n = safety::step(s, e).map_err(|_| BackendError::Rejected("aerial safety"))?;
    let revoke = event_revokes_authority(e) || (n.failsafe && !s.failsafe);
    let failsafe = n.failsafe;
    body.aerial = Some(n);
    if revoke {
        body.bump_authority();
    }
    if failsafe || e == Event::Touchdown || e == Event::Recover {
        body.clear_command();
    }
    Ok(())
}

pub(crate) fn ground_event(
    session: &WorldSession,
    id: &'static str,
    e: GroundEvent,
) -> Result<(), BackendError> {
    let mut plant = session.lock();
    let body = require_body_mut(&mut plant.world, id)?;
    let s = body.ground.ok_or(BackendError::Protocol)?;
    let n = ground_step(s, e).map_err(|_| BackendError::Rejected("ground safety"))?;
    let revoke = ground_event_revokes_authority(e) || (n.estop && !s.estop);
    let estop = n.estop;
    body.ground = Some(n);
    if revoke {
        body.bump_authority();
    }
    if estop || e == GroundEvent::Halt {
        body.clear_command();
    }
    Ok(())
}

pub(crate) fn marine_event(
    session: &WorldSession,
    id: &'static str,
    e: MarineEvent,
) -> Result<(), BackendError> {
    let mut plant = session.lock();
    let body = require_body_mut(&mut plant.world, id)?;
    let s = body.marine.ok_or(BackendError::Protocol)?;
    let n = marine_step(s, e).map_err(|_| BackendError::Rejected("marine safety"))?;
    let revoke = marine_event_revokes_authority(e) || (n.failsafe && !s.failsafe);
    let cut = n.failsafe || !n.thrust_enabled;
    body.marine = Some(n);
    if revoke {
        body.bump_authority();
    }
    if cut {
        body.clear_command();
    }
    Ok(())
}

pub(crate) fn body_imu(body: &Body, now: MonotonicInstant, seq: u32) -> ImuSample<BodyFrame> {
    let inv_m = 1.0 / body.mass_kg.max(1e-6);
    let a_ned = [
        body.last_thrust[0] * inv_m,
        body.last_thrust[1] * inv_m,
        body.last_thrust[2] * inv_m,
    ];
    let a_body = quat_rotate_inv(body.quat, a_ned);
    ImuSample {
        timestamp: now,
        accel: Acceleration::body(a_body[0], a_body[1], a_body[2]),
        gyro: AngularVelocity::body_rad(body.omega_body[0], body.omega_body[1], body.omega_body[2]),
        covariance: None,
        temperature: None,
        status: SensorHealth::Ok,
        sequence: seq,
    }
}

pub(crate) fn snapshot(
    body: &Body,
    now: MonotonicInstant,
    imu_seq: u32,
    last_command: &'static str,
) -> Result<Telemetry, BackendError> {
    let (phase, armed, actuators, offboard, failsafe, imu_healthy, estimator_valid) =
        match body.domain {
            Domain::Aerial => {
                let s = body.aerial.ok_or(BackendError::Protocol)?;
                (
                    s.phase,
                    s.armed,
                    s.actuators_enabled,
                    s.offboard,
                    s.failsafe,
                    s.imu_healthy,
                    s.estimator_valid,
                )
            }
            Domain::Ground => {
                let s = body.ground.ok_or(BackendError::Protocol)?;
                (
                    Phase::Ready,
                    s.drive_enabled,
                    s.drive_enabled,
                    false,
                    s.estop,
                    true,
                    true,
                )
            }
            Domain::Surface | Domain::Underwater => {
                let s = body.marine.ok_or(BackendError::Protocol)?;
                (
                    Phase::Ready,
                    s.thrust_enabled,
                    s.thrust_enabled,
                    s.phase == MarinePhase::Underway,
                    s.failsafe,
                    true,
                    true,
                )
            }
        };
    let imu = body_imu(body, now, imu_seq);
    Ok(Telemetry {
        timestamp: now,
        phase,
        position: Position::ned(body.position_m[0], body.position_m[1], body.position_m[2]),
        velocity: Velocity::ned(
            body.velocity_mps[0],
            body.velocity_mps[1],
            body.velocity_mps[2],
        ),
        yaw_rad: body.yaw_rad,
        imu: Some(imu),
        imu_health: SensorHealth::Ok,
        imu_healthy,
        estimator_valid,
        armed,
        actuators_enabled: actuators,
        offboard,
        failsafe,
        heartbeat_age_secs: match body.aerial {
            Some(s) if !s.offboard_heartbeat_fresh => {
                flight_core::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS as f32 / 1000.0
            }
            _ => 0.0,
        },
        last_command,
    })
}

pub(crate) fn preflight_from(
    session: &WorldSession,
    id: &str,
) -> Result<PreflightReport, BackendError> {
    let plant = session.lock();
    let body = require_body(&plant.world, id)?;
    let (imu_healthy, estimator_valid) = body
        .aerial
        .map(|s| (s.imu_healthy, s.estimator_valid))
        .unwrap_or((true, true));
    Ok(PreflightReport {
        imu_healthy,
        estimator_valid,
        battery_ok: body.charge_j > 0.0,
        gps_ok: true,
        notes: PreflightNotes {
            imu_std_accel: 0.0,
            imu_std_gyro: 0.0,
            samples: 40,
        },
    })
}

pub(crate) fn flush_body(
    session: &WorldSession,
    id: &'static str,
    setpoint: Option<Setpoint>,
    yaw_rate: Option<f32>,
) -> Result<(), BackendError> {
    let mut plant = session.lock();
    let body = require_body_mut(&mut plant.world, id)?;
    apply_setpoint(body, setpoint, yaw_rate);
    Ok(())
}

pub(crate) fn tick_body(
    session: &WorldSession,
    id: &'static str,
    setpoint: Option<Setpoint>,
    yaw_rate: Option<f32>,
    dt_secs: f32,
    imu_seq: &mut u32,
    last_command: &'static str,
) -> Result<Telemetry, BackendError> {
    flush_body(session, id, setpoint, yaw_rate)?;
    session.step(dt_secs)?;
    *imu_seq = imu_seq.wrapping_add(1);
    let plant = session.lock();
    let body = require_body(&plant.world, id)?;
    snapshot(body, plant.clock.now(), *imu_seq, last_command)
}

pub(crate) fn telemetry_body(
    session: &WorldSession,
    id: &'static str,
    imu_seq: &mut u32,
    last_command: &'static str,
) -> Result<Telemetry, BackendError> {
    *imu_seq = imu_seq.wrapping_add(1);
    let plant = session.lock();
    let body = require_body(&plant.world, id)?;
    snapshot(body, plant.clock.now(), *imu_seq, last_command)
}

/// Feed one IMU sample through the complementary filter. Unusable samples
/// clear kernel `estimator_valid` (and latch failsafe if armed) without
/// writing the plant quaternion. Filter warm-up is not a trip.
pub(crate) fn update_nav(
    session: &WorldSession,
    body_id: &'static str,
    sample: ImuSample<BodyFrame>,
    dt: f32,
) -> Result<bool, BackendError> {
    let trip = imu_trips_estimator(&sample, dt);
    let filter_valid = {
        let mut plant = session.lock();
        if require_body(&plant.world, body_id)?.aerial.is_none() {
            return Err(BackendError::Protocol);
        }
        let att = plant.attitude.entry(body_id).or_default();
        if trip {
            att.invalidate();
            false
        } else {
            att.update(sample.gyro, sample.accel, dt);
            att.is_valid()
        }
    };
    if trip {
        aerial_event(session, body_id, Event::EstimatorInvalid)?;
        return Ok(false);
    }
    Ok(filter_valid)
}
