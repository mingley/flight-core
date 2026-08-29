//! Probe illegal JSON acts, then attach consume-self typestate and move on one clock.
//! After takeoff the drone `hold_now`s the live pose; the JSON prints `hold_ned`.

use flight_core::frames::Ned;
use flight_core::prelude::*;
use flight_core::vector::Velocity;
use robot_lab::{AgentAction, Lab, LabCmd};

fn main() {
    let mut lab = Lab::coastal(3);
    for (robot, cmd, vn, ve) in [
        ("rover", LabCmd::Drive, -0.6, 0.0),
        ("skiff", LabCmd::Thrust, 0.8, 0.0),
        ("drone", LabCmd::Velocity, 0.0, 1.0),
    ] {
        let err = lab
            .act(AgentAction::new(robot, cmd).ned(vn, ve, 0.0))
            .expect_err("illegal grant must bounce");
        eprintln!("rejected {robot} {cmd}: {err}");
    }

    let VehicleHandle::PreflightReady(drone) = lab.aerial_vehicle("drone").expect("drone") else {
        panic!("world drones start Ready");
    };
    let mut drone = drone
        .arm_now()
        .expect("arm")
        .enter_offboard_now()
        .expect("offboard")
        .start_takeoff_now()
        .expect("takeoff");

    let GroundHandle::Parked(rover) = lab.ground_vehicle("rover").expect("rover") else {
        panic!("world rovers start Parked");
    };
    let mut rover = rover.enable_drive().expect("release");

    let MarineHandle::Docked(skiff) = lab.marine_vehicle("skiff").expect("skiff") else {
        panic!("world hulls start Docked");
    };
    let mut skiff = skiff.undock().expect("undock");

    for _ in 0..80 {
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
            .unwrap();
        drone.backend().flush().unwrap();
        rover
            .set_velocity_ned_now(Velocity::<Ned>::ned(-0.7, 0.0, 0.0))
            .unwrap();
        rover.backend().flush().unwrap();
        skiff
            .set_ned_velocity_now(Velocity::<Ned>::ned(0.0, 0.5, 0.0))
            .unwrap();
        skiff.backend().flush().unwrap();
        lab.session().step(0.02).expect("properties");
    }

    drone.hold_now().expect("hold");
    drone.backend().flush().unwrap();
    lab.session().step(0.02).expect("properties");

    let obs = lab.observe();
    let drone_view = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "scenario": obs.scenario,
            "seed": obs.seed,
            "t": obs.t,
            "all_hold": obs.all_hold,
            "hold_ned": drone_view.hold_ned,
            "robots": obs.robots.iter().map(|r| serde_json::json!({
                "id": r.id,
                "phase": r.phase,
                "support": r.support,
                "terrain_contact": r.terrain_contact,
                "sphere_contact": r.sphere_contact,
                "sphere_partners": r.sphere_partners,
                "n": r.n,
                "e": r.e,
                "alt": r.alt,
                "propulsion_live": r.propulsion_live,
                "hold_ned": r.hold_ned,
            })).collect::<Vec<_>>(),
        }))
        .unwrap()
    );
    if !obs.all_hold {
        std::process::exit(1);
    }
}
