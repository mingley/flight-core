use flight_core::vehicle::{
    aerial_kind, ground_kind, marine_kind, AerialKind, GroundHandle, GroundKind, MarineHandle,
    MarineKind, VehicleHandle,
};

use crate::lab::Lab;

pub(crate) fn script_tick(lab: &mut Lab) {
    let t = lab.with_world(|w| w.t);
    // Ids first so a missing inland hull is skipped, not Protocol-panicked.
    let ids: Vec<&'static str> = lab.with_world(|w| w.bodies.iter().map(|b| b.id).collect());
    for id in ids {
        match id {
            "drone" => script_drone(lab, t),
            "rover" => script_rover(lab, t),
            "skiff" => script_skiff(lab, t),
            "surveyor" => script_surveyor(lab, t),
            _ => {}
        }
    }
    if !lab.message.starts_with("PROPERTY") {
        lab.message = format!("script t={t:.1}s");
    }
}

pub(crate) fn script_ned(
    vn: f32,
    ve: f32,
    vd: f32,
) -> flight_core::vector::Velocity<flight_core::frames::Ned> {
    flight_core::vector::Velocity::<flight_core::frames::Ned>::ned(vn, ve, vd)
}

pub(crate) fn script_drone(lab: &mut Lab, t: f32) {
    let Some((kind, alt)) = lab.with_world(|w| {
        w.body("drone")
            .and_then(|b| b.aerial.map(|s| (aerial_kind(s), b.altitude_agl())))
    }) else {
        return;
    };
    match kind {
        AerialKind::PreflightReady => {
            let _ = lab.attach_takeoff("drone");
        }
        AerialKind::Armed => {
            if let Ok(VehicleHandle::Armed(armed)) = lab.aerial_vehicle("drone") {
                if let Ok(offboard) = armed.enter_offboard_now() {
                    let _ = offboard.start_takeoff_now();
                }
            }
        }
        AerialKind::Offboard => {
            if let Ok(VehicleHandle::Offboard(offboard)) = lab.aerial_vehicle("drone") {
                let _ = offboard.start_takeoff_now();
            }
        }
        AerialKind::Takeoff => {
            if let Ok(VehicleHandle::Takeoff(mut drone)) = lab.aerial_vehicle("drone") {
                if drone.set_velocity_now(script_ned(0.0, 0.0, -1.2)).is_ok() {
                    let _ = drone.backend().flush();
                }
                if alt >= 6.0 {
                    let _ = drone.declare_airborne_now();
                }
            }
        }
        AerialKind::Airborne => {
            if t > 22.0 {
                let _ = lab.attach_land("drone");
            } else if let Ok(VehicleHandle::Airborne(mut drone)) = lab.aerial_vehicle("drone") {
                let v = if t > 8.0 {
                    script_ned(0.0, 1.1, 0.0)
                } else {
                    script_ned(0.0, 0.0, 0.0)
                };
                if drone.set_velocity_now(v).is_ok() {
                    let _ = drone.backend().flush();
                }
            }
        }
        AerialKind::Landing => {
            if let Ok(VehicleHandle::Landing(mut drone)) = lab.aerial_vehicle("drone") {
                if drone.set_velocity_now(script_ned(0.0, 0.0, 0.8)).is_ok() {
                    let _ = drone.backend().flush();
                }
                if alt <= 0.12 {
                    let _ = drone.touchdown_now();
                }
            }
        }
        AerialKind::Failsafe
        | AerialKind::Recovery
        | AerialKind::Disconnected
        | AerialKind::Disarmed => {}
    }
}

pub(crate) fn script_rover(lab: &mut Lab, _t: f32) {
    let Some((kind, north)) = lab.with_world(|w| {
        w.body("rover")
            .and_then(|b| b.ground.map(|s| (ground_kind(s), b.position_m[0])))
    }) else {
        return;
    };
    match kind {
        GroundKind::Parked => {
            let _ = lab.attach_drive("rover");
        }
        GroundKind::Moving => {
            if let Ok(GroundHandle::Moving(mut rover)) = lab.ground_vehicle("rover") {
                let v = if north > 2.5 {
                    script_ned(-0.7, 0.15, 0.0)
                } else {
                    script_ned(0.0, 0.0, 0.0)
                };
                if rover.set_velocity_ned_now(v).is_ok() {
                    let _ = rover.backend().flush();
                }
            }
        }
        GroundKind::EStopped => {}
    }
}

pub(crate) fn script_skiff(lab: &mut Lab, t: f32) {
    let Some(kind) = lab.with_world(|w| w.body("skiff").and_then(|b| b.marine.map(marine_kind)))
    else {
        return;
    };
    match kind {
        MarineKind::Docked => {
            let _ = lab.attach_undock("skiff");
        }
        MarineKind::Underway => {
            if t > 14.0 {
                let _ = lab.attach_station("skiff");
            } else if let Ok(MarineHandle::Underway(mut hull)) = lab.marine_vehicle("skiff") {
                if hull
                    .set_ned_velocity_now(script_ned(0.05, 0.55, 0.0))
                    .is_ok()
                {
                    let _ = hull.backend().flush();
                }
            }
        }
        MarineKind::StationKeep => {
            if let Ok(MarineHandle::StationKeep(mut hull)) = lab.marine_vehicle("skiff") {
                if hull.set_ned_velocity_now(script_ned(0.0, 0.0, 0.0)).is_ok() {
                    let _ = hull.backend().flush();
                }
            }
        }
        MarineKind::Failsafe => {}
    }
}

pub(crate) fn script_surveyor(lab: &mut Lab, t: f32) {
    let Some(kind) = lab.with_world(|w| w.body("surveyor").and_then(|b| b.marine.map(marine_kind)))
    else {
        return;
    };
    match kind {
        MarineKind::Docked => {
            let _ = lab.attach_undock("surveyor");
        }
        MarineKind::Underway => {
            let vn = if (t as i32 / 6) % 2 == 0 { 0.25 } else { -0.25 };
            if let Ok(MarineHandle::Underway(mut hull)) = lab.marine_vehicle("surveyor") {
                if hull.set_ned_velocity_now(script_ned(vn, 0.0, 0.0)).is_ok() {
                    let _ = hull.backend().flush();
                }
            }
        }
        MarineKind::StationKeep | MarineKind::Failsafe => {}
    }
}
