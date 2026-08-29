use flight_core::domain::Domain;
use flight_core::ground::{ground_step, GroundEvent};
use flight_core::marine::{marine_step, MarineEvent};
use flight_core::safety::{self, Event};
use robot_world::{Body, World};

use crate::cmd::{aerial_ok, aerial_ok_seq};
use crate::{AgentAction, LabCmd, LabError};

pub(crate) fn apply_action_world(
    world: &mut World,
    action: &AgentAction,
) -> Result<String, LabError> {
    if let Some(msg) = apply_env_action(world, action) {
        return Ok(msg);
    }

    let id = action.robot.as_str();
    let body = world
        .body_mut(id)
        .ok_or_else(|| LabError::UnknownRobot(action.robot.clone()))?;

    match action.cmd {
        LabCmd::Arm => aerial(body, Event::Arm)?,
        LabCmd::Disarm => aerial(body, Event::Disarm)?,
        LabCmd::Offboard => {
            aerial(body, Event::HeartbeatFresh)?;
            aerial(body, Event::EnterOffboard)?;
        }
        LabCmd::EnableActuators => aerial(body, Event::EnableActuators)?,
        LabCmd::Takeoff => aerial(body, Event::Takeoff)?,
        LabCmd::Airborne => aerial(body, Event::ReachedAltitude)?,
        LabCmd::Land => aerial(body, Event::Land)?,
        LabCmd::Touchdown => aerial(body, Event::Touchdown)?,
        LabCmd::Failsafe => match body.domain {
            Domain::Aerial => aerial(body, Event::TriggerFailsafe)?,
            Domain::Ground => ground(body, GroundEvent::EStop)?,
            Domain::Surface | Domain::Underwater => marine(body, MarineEvent::Failsafe)?,
        },
        LabCmd::Velocity | LabCmd::Drive | LabCmd::Thrust => {
            set_velocity(body, [action.vn, action.ve, action.vd])?;
            body.yaw_cmd = action.yaw_rate;
        }
        LabCmd::Position => {
            set_position(body, [action.vn, action.ve, action.vd])?;
            body.yaw_cmd = action.yaw_rate;
        }
        LabCmd::Hold => {
            let p = body.position_m;
            set_position(body, p)?;
            body.yaw_cmd = action.yaw_rate;
        }
        LabCmd::Release => ground(body, GroundEvent::Release)?,
        LabCmd::Halt | LabCmd::Park => {
            ground(body, GroundEvent::Halt)?;
            body.clear_command();
        }
        LabCmd::Estop => ground(body, GroundEvent::EStop)?,
        LabCmd::Clear => ground(body, GroundEvent::ClearEstop)?,
        LabCmd::Undock => marine(body, MarineEvent::Undock)?,
        LabCmd::Dock => {
            marine(body, MarineEvent::Dock)?;
            body.clear_command();
        }
        LabCmd::Station => marine(body, MarineEvent::Station)?,
        LabCmd::Resume => marine(body, MarineEvent::Resume)?,
        LabCmd::Recover => match body.domain {
            Domain::Aerial => aerial_recover(body)?,
            Domain::Surface | Domain::Underwater => marine(body, MarineEvent::Recover)?,
            Domain::Ground => return Err(LabError::WrongDomain),
        },
        LabCmd::SetCharge => {
            body.charge_j = action.vn.clamp(0.0, body.capacity_j.max(0.0));
        }
        LabCmd::SetWind | LabCmd::SetWaves | LabCmd::SetCurrent => {
            unreachable!("environment commands are applied before the body lookup")
        }
    }
    Ok(format!("{} ← {}", action.robot, action.cmd))
}

pub(crate) fn apply_env_action(world: &mut World, action: &AgentAction) -> Option<String> {
    match action.cmd {
        LabCmd::SetWind => {
            world.env.wind_ned = [action.vn, action.ve, action.vd];
            Some(format!(
                "wind NED [{:.2}, {:.2}, {:.2}]",
                action.vn, action.ve, action.vd
            ))
        }
        LabCmd::SetWaves => {
            world.env.wave_amp = action.vn.clamp(0.0, 2.5);
            if action.ve > 0.0 {
                world.env.wave_k = action.ve;
            }
            if action.vd > 0.0 {
                world.env.wave_omega = action.vd;
            }
            world
                .hydro
                .apply_waves(world.env.wave_amp, world.env.wave_k, world.env.wave_phase);
            Some(format!("waves amp={:.2} m", world.env.wave_amp))
        }
        LabCmd::SetCurrent => {
            let old = world.env.current_ned;
            let new = [action.vn, action.ve, action.vd];
            world.env.current_ned = new;
            world.hydro.shift_current(old, new);
            Some(format!(
                "current NED [{:.2}, {:.2}, {:.2}]",
                action.vn, action.ve, action.vd
            ))
        }
        _ => None,
    }
}

pub(crate) fn aerial(body: &mut Body, e: Event) -> Result<(), LabError> {
    let s = body.aerial.ok_or(LabError::WrongDomain)?;
    let n = safety::step(s, e).map_err(LabError::Aerial)?;
    body.aerial = Some(n);
    if n.failsafe || e == Event::Touchdown || e == Event::Recover {
        body.clear_command();
    }
    Ok(())
}

/// Kernel `Recover` is Recovery → Ready. Failsafe must `Disarm` first.
/// Kernel `Recover` is Recovery → Ready. Failsafe must `Disarm` first.
/// Do not Disarm a live (non-failsafe) machine and then fail Recover — that
/// left Airborne vehicles half-applied.
pub(crate) fn aerial_recover(body: &mut Body) -> Result<(), LabError> {
    if aerial_ok(body, Event::Recover) {
        aerial(body, Event::Recover)
    } else if aerial_ok_seq(body, &[Event::Disarm, Event::Recover]) {
        aerial(body, Event::Disarm)?;
        aerial(body, Event::Recover)
    } else {
        Err(LabError::Aerial(flight_core::safety::Reject::IllegalPhase))
    }
}

pub(crate) fn ground(body: &mut Body, e: GroundEvent) -> Result<(), LabError> {
    let s = body.ground.ok_or(LabError::WrongDomain)?;
    let n = ground_step(s, e).map_err(LabError::Ground)?;
    body.ground = Some(n);
    if n.estop {
        body.clear_command();
    }
    Ok(())
}

pub(crate) fn marine(body: &mut Body, e: MarineEvent) -> Result<(), LabError> {
    let s = body.marine.ok_or(LabError::WrongDomain)?;
    let n = marine_step(s, e).map_err(LabError::Marine)?;
    body.marine = Some(n);
    if n.failsafe {
        body.clear_command();
    }
    Ok(())
}

pub(crate) fn set_velocity(body: &mut Body, v: [f32; 3]) -> Result<(), LabError> {
    match body.domain {
        Domain::Aerial => {
            aerial(body, Event::HeartbeatFresh)?;
            aerial(body, Event::MissionCommand)?;
        }
        Domain::Ground => ground(body, GroundEvent::DriveCommand)?,
        Domain::Surface | Domain::Underwater => marine(body, MarineEvent::ThrustCommand)?,
    }
    body.set_velocity_command(v);
    Ok(())
}

/// JSON fallback for [`LabCmd::Position`]. Stores a plant hold so each
/// verified step rewrites the P-term — never a raw NED velocity.
pub(crate) fn set_position(body: &mut Body, p: [f32; 3]) -> Result<(), LabError> {
    match body.domain {
        Domain::Aerial => {
            aerial(body, Event::HeartbeatFresh)?;
            aerial(body, Event::MissionCommand)?;
        }
        Domain::Ground | Domain::Surface | Domain::Underwater => return Err(LabError::WrongDomain),
    }
    body.set_position_hold(p);
    Ok(())
}
