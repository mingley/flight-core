use crate::{
    AerialKind, AerialMachine, AgentAction, GroundHandle, Lab, LabCmd, MarineHandle, MarineKind,
    Observation, RobotView, VehicleHandle,
};

pub(crate) fn cmd(robot: &str, cmd: LabCmd, vn: f32, ve: f32, vd: f32) -> AgentAction {
    AgentAction::new(robot, cmd).ned(vn, ve, vd)
}

pub(crate) fn robot<'a>(obs: &'a Observation, id: &str) -> Option<&'a RobotView> {
    obs.robots.iter().find(|r| r.id == id)
}

pub(crate) fn probes(obs: &Observation) -> Vec<AgentAction> {
    let mut out = Vec::new();
    if let Some(g) = robot(obs, "rover").and_then(|r| r.ground.as_ref()) {
        if !g.drive_enabled && !g.estop {
            out.push(cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0));
        }
    }
    if let Some(m) = robot(obs, "skiff").and_then(|r| r.marine.as_ref()) {
        if !m.thrust_enabled && !m.failsafe {
            out.push(cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0));
        }
    }
    if let Some(m) = robot(obs, "surveyor").and_then(|r| r.marine.as_ref()) {
        if !m.thrust_enabled && !m.failsafe {
            out.push(cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4));
        }
    }
    if let Some(a) = robot(obs, "drone").and_then(|r| r.aerial.as_ref()) {
        if !a.armed {
            out.push(cmd("drone", LabCmd::Velocity, 0.0, 1.0, 0.0));
        }
    }
    out
}

pub(crate) fn grants_for(obs: &Observation) -> Vec<AgentAction> {
    let mut out = Vec::new();
    if let Some(g) = robot(obs, "rover").and_then(|r| r.ground.as_ref()) {
        if !g.drive_enabled && !g.estop {
            out.push(cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0));
        }
    }
    if let Some(a) = hull_undock(obs, "skiff") {
        out.push(a);
    }
    if let Some(a) = hull_undock(obs, "surveyor") {
        out.push(a);
    }
    out.extend(drone_grant_chain(obs));
    out
}

pub(crate) fn hull_undock(obs: &Observation, id: &str) -> Option<AgentAction> {
    let m = robot(obs, id)?.marine.as_ref()?;
    if !m.thrust_enabled && !m.failsafe && m.kind == MarineKind::Docked {
        return Some(cmd(id, LabCmd::Undock, 0.0, 0.0, 0.0));
    }
    None
}

pub(crate) fn drone_grant_chain(obs: &Observation) -> Vec<AgentAction> {
    let Some(a) = robot(obs, "drone").and_then(|r| r.aerial.as_ref()) else {
        return Vec::new();
    };
    if a.failsafe {
        return Vec::new();
    }
    let mut out = Vec::new();
    if !a.armed {
        out.push(cmd("drone", LabCmd::Arm, 0.0, 0.0, 0.0));
    }
    if !a.armed || !a.offboard {
        out.push(cmd("drone", LabCmd::Offboard, 0.0, 0.0, 0.0));
    }
    if !a.armed || !a.offboard || !a.actuators_enabled {
        out.push(cmd("drone", LabCmd::EnableActuators, 0.0, 0.0, 0.0));
    }
    if matches!(
        a.kind,
        AerialKind::PreflightReady | AerialKind::Armed | AerialKind::Offboard
    ) {
        out.push(cmd("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0));
    }
    out
}

pub(crate) fn motions_for(obs: &Observation) -> Vec<AgentAction> {
    ["rover", "skiff", "surveyor", "drone"]
        .into_iter()
        .filter_map(|id| motion_for(obs, id))
        .collect()
}

pub(crate) fn motion_for(obs: &Observation, id: &str) -> Option<AgentAction> {
    let r = robot(obs, id)?;
    match id {
        "rover" => rover_drive(r),
        "skiff" => hull_thrust(r, 0.05, 0.55, 0.0),
        "surveyor" => hull_thrust(r, 0.25, 0.0, 0.0),
        "drone" => drone_fly(r),
        _ => None,
    }
}

pub(crate) fn rover_drive(r: &RobotView) -> Option<AgentAction> {
    let g = r.ground.as_ref()?;
    if g.drive_enabled && r.terrain_contact {
        Some(cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0))
    } else {
        None
    }
}

pub(crate) fn hull_thrust(r: &RobotView, vn: f32, ve: f32, vd: f32) -> Option<AgentAction> {
    let m = r.marine.as_ref()?;
    if m.thrust_enabled && r.support == "water" {
        Some(cmd(&r.id, LabCmd::Thrust, vn, ve, vd))
    } else {
        None
    }
}

pub(crate) fn drone_fly(r: &RobotView) -> Option<AgentAction> {
    let a: &AerialMachine = r.aerial.as_ref()?;
    if a.failsafe || !a.armed || !a.actuators_enabled || r.support == "water" {
        return None;
    }
    if r.alt < 6.0 {
        Some(cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2))
    } else {
        Some(cmd("drone", LabCmd::Velocity, 0.0, 1.1, 0.0))
    }
}

pub(crate) fn note(lab: &mut Lab, action: AgentAction) {
    let t = lab.with_world(|w| w.t);
    lab.log.push(crate::TimedAction { t, action });
}

pub(crate) fn grant_attached(lab: &mut Lab, obs: &Observation) {
    if robot(obs, "drone").is_some() && lab.attach_takeoff("drone").is_ok() {
        note(lab, cmd("drone", LabCmd::Arm, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::Offboard, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::EnableActuators, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0));
    }
    if robot(obs, "rover").is_some() && lab.attach_drive("rover").is_ok() {
        note(lab, cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0));
    }
    for id in ["skiff", "surveyor"] {
        if robot(obs, id).is_some() && lab.attach_undock(id).is_ok() {
            note(lab, cmd(id, LabCmd::Undock, 0.0, 0.0, 0.0));
        }
    }
}

pub(crate) fn return_attached(lab: &mut Lab, obs: &Observation) {
    if robot(obs, "drone").is_some() {
        if lab.attach_land("drone").is_ok() {
            note(lab, cmd("drone", LabCmd::Land, 0.0, 0.0, 0.0));
        }
        if lab.attach_touchdown("drone").is_ok() {
            note(lab, cmd("drone", LabCmd::Touchdown, 0.0, 0.0, 0.0));
        }
    }
    if robot(obs, "rover").is_some() && lab.attach_park("rover").is_ok() {
        note(lab, cmd("rover", LabCmd::Halt, 0.0, 0.0, 0.0));
    }
    for id in ["skiff", "surveyor"] {
        if robot(obs, id).is_some() && lab.attach_dock(id).is_ok() {
            note(lab, cmd(id, LabCmd::Dock, 0.0, 0.0, 0.0));
        }
    }
}

pub(crate) fn drive_attached(lab: &mut Lab, obs: &Observation) {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    if let Some(r) = robot(obs, "drone") {
        if r.aerial
            .as_ref()
            .is_some_and(|a| a.armed && a.actuators_enabled && !a.failsafe)
            && r.support != "water"
        {
            let (vn, ve, vd) = if r.alt < 6.0 {
                (0.0, 0.0, -1.2)
            } else {
                (0.0, 1.1, 0.0)
            };
            let v = Velocity::<Ned>::ned(vn, ve, vd);
            match lab.aerial_vehicle("drone") {
                Ok(VehicleHandle::Offboard(mut drone)) => {
                    if drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok() {
                        note(lab, cmd("drone", LabCmd::Velocity, vn, ve, vd));
                    }
                }
                Ok(VehicleHandle::Takeoff(mut drone)) => {
                    if drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok() {
                        note(lab, cmd("drone", LabCmd::Velocity, vn, ve, vd));
                    }
                }
                Ok(VehicleHandle::Airborne(mut drone)) => {
                    if drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok() {
                        note(lab, cmd("drone", LabCmd::Velocity, vn, ve, vd));
                    }
                }
                _ => {}
            }
        }
    }
    if let Some(r) = robot(obs, "rover") {
        if r.ground.as_ref().is_some_and(|g| g.drive_enabled) && r.terrain_contact {
            if let Ok(GroundHandle::Moving(mut rover)) = lab.ground_vehicle("rover") {
                if rover
                    .set_velocity_ned_now(Velocity::<Ned>::ned(-0.6, 0.0, 0.0))
                    .is_ok()
                    && rover.backend().flush().is_ok()
                {
                    note(lab, cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0));
                }
            }
        }
    }
    if let Some(r) = robot(obs, "skiff") {
        if r.marine.as_ref().is_some_and(|m| m.thrust_enabled) && r.support == "water" {
            if let Ok(MarineHandle::Underway(mut skiff)) = lab.marine_vehicle("skiff") {
                if skiff
                    .set_ned_velocity_now(Velocity::<Ned>::ned(0.05, 0.55, 0.0))
                    .is_ok()
                    && skiff.backend().flush().is_ok()
                {
                    note(lab, cmd("skiff", LabCmd::Thrust, 0.05, 0.55, 0.0));
                }
            }
        }
    }
    if let Some(r) = robot(obs, "surveyor") {
        if r.marine.as_ref().is_some_and(|m| m.thrust_enabled) && r.support == "water" {
            if let Ok(MarineHandle::Underway(mut surveyor)) = lab.marine_vehicle("surveyor") {
                if surveyor
                    .set_ned_velocity_now(Velocity::<Ned>::ned(0.25, 0.0, 0.0))
                    .is_ok()
                    && surveyor.backend().flush().is_ok()
                {
                    note(lab, cmd("surveyor", LabCmd::Thrust, 0.25, 0.0, 0.0));
                }
            }
        }
    }
}

pub(crate) fn grant_drone_attached(lab: &mut Lab) {
    if lab.attach_takeoff("drone").is_ok() {
        note(lab, cmd("drone", LabCmd::Arm, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::Offboard, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::EnableActuators, 0.0, 0.0, 0.0));
        note(lab, cmd("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0));
        return;
    }
    if let Ok(VehicleHandle::Offboard(offboard)) = lab.aerial_vehicle("drone") {
        if offboard.start_takeoff_now().is_ok() {
            note(lab, cmd("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0));
        }
        return;
    }
    if let Ok(VehicleHandle::Armed(armed)) = lab.aerial_vehicle("drone") {
        if let Ok(offboard) = armed.enter_offboard_now() {
            if offboard.start_takeoff_now().is_ok() {
                note(lab, cmd("drone", LabCmd::Offboard, 0.0, 0.0, 0.0));
                note(lab, cmd("drone", LabCmd::EnableActuators, 0.0, 0.0, 0.0));
                note(lab, cmd("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0));
            }
        }
    }
}

pub(crate) fn drone_velocity_attached(lab: &mut Lab, vn: f32, ve: f32, vd: f32) {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    let v = Velocity::<Ned>::ned(vn, ve, vd);
    let flushed = match lab.aerial_vehicle("drone") {
        Ok(VehicleHandle::Offboard(mut drone)) => {
            drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Takeoff(mut drone)) => {
            drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Airborne(mut drone)) => {
            drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Landing(mut drone)) => {
            drone.set_velocity_now(v).is_ok() && drone.backend().flush().is_ok()
        }
        _ => false,
    };
    if flushed {
        note(lab, cmd("drone", LabCmd::Velocity, vn, ve, vd));
    }
}

pub(crate) fn drone_position_attached(lab: &mut Lab, n: f32, e: f32, d: f32) -> bool {
    use flight_core::frames::Ned;
    use flight_core::vector::Position;

    let p = Position::<Ned>::ned(n, e, d);
    let flushed = match lab.aerial_vehicle("drone") {
        Ok(VehicleHandle::Offboard(mut drone)) => {
            drone.set_position_now(p).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Takeoff(mut drone)) => {
            drone.set_position_now(p).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Airborne(mut drone)) => {
            drone.set_position_now(p).is_ok() && drone.backend().flush().is_ok()
        }
        Ok(VehicleHandle::Landing(mut drone)) => {
            drone.set_position_now(p).is_ok() && drone.backend().flush().is_ok()
        }
        _ => false,
    };
    if flushed {
        note(lab, cmd("drone", LabCmd::Position, n, e, d));
    }
    flushed
}

pub(crate) fn drone_hold_attached(lab: &mut Lab) -> bool {
    if lab.attach_hold("drone").is_ok() {
        note(lab, cmd("drone", LabCmd::Hold, 0.0, 0.0, 0.0));
        true
    } else {
        false
    }
}

pub(crate) fn rover_hold_attached(lab: &mut Lab) -> bool {
    if lab.attach_ground_hold("rover").is_ok() {
        note(lab, cmd("rover", LabCmd::Hold, 0.0, 0.0, 0.0));
        true
    } else {
        false
    }
}

pub(crate) fn rover_drive_attached(lab: &mut Lab, vn: f32, ve: f32, vd: f32) {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    if let Ok(GroundHandle::Moving(mut rover)) = lab.ground_vehicle("rover") {
        if rover
            .set_velocity_ned_now(Velocity::<Ned>::ned(vn, ve, vd))
            .is_ok()
            && rover.backend().flush().is_ok()
        {
            note(lab, cmd("rover", LabCmd::Drive, vn, ve, vd));
        }
    }
}

pub(crate) fn skiff_thrust_attached(lab: &mut Lab, vn: f32, ve: f32, vd: f32) {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    if let Ok(MarineHandle::Underway(mut skiff)) = lab.marine_vehicle("skiff") {
        if skiff
            .set_ned_velocity_now(Velocity::<Ned>::ned(vn, ve, vd))
            .is_ok()
            && skiff.backend().flush().is_ok()
        {
            note(lab, cmd("skiff", LabCmd::Thrust, vn, ve, vd));
        }
    }
}

pub(crate) fn surveyor_thrust_attached(lab: &mut Lab, vn: f32, ve: f32, vd: f32) {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    if let Ok(MarineHandle::Underway(mut surveyor)) = lab.marine_vehicle("surveyor") {
        if surveyor
            .set_ned_velocity_now(Velocity::<Ned>::ned(vn, ve, vd))
            .is_ok()
            && surveyor.backend().flush().is_ok()
        {
            note(lab, cmd("surveyor", LabCmd::Thrust, vn, ve, vd));
        }
    }
}
