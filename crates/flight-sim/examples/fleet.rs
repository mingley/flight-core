//! Drone, rover, and skiff share one verified world step.
//!
//! `Vehicle::new` always starts Disconnected. World bodies are already Ready /
//! Parked / Docked, so this example `attach()`s consume-self typestate, grants
//! with now-APIs, and flushes every hull before one `WorldSession::step`.
//! `takeoff().await` would tick the whole scene before the rover and skiff
//! even exist.

use flight_core::prelude::*;
use flight_sim::WorldSession;

#[tokio::main]
async fn main() {
    let session = WorldSession::coastal(1);

    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().expect("drone")
    else {
        panic!("world drones start Ready");
    };
    let mut drone = drone
        .arm_now()
        .expect("arm")
        .enter_offboard_now()
        .expect("offboard")
        .start_takeoff_now()
        .expect("takeoff");
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .expect("climb");

    let GroundHandle::Parked(rover) = session.ground("rover").attach().expect("rover") else {
        panic!("world rovers start Parked");
    };
    let mut rover = rover.enable_drive().expect("release");
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.6, 0.1, 0.0))
        .expect("drive");

    let MarineHandle::Docked(skiff) = session.marine("skiff").attach().expect("skiff") else {
        panic!("world hulls start Docked");
    };
    let mut skiff = skiff.undock().expect("undock");
    skiff
        .set_ned_velocity_now(Velocity::<Ned>::ned(0.05, 0.4, 0.0))
        .expect("thrust");

    for _ in 0..120 {
        let alt = session.world().body("drone").unwrap().altitude_agl();
        let vd = if alt < 4.0 { -1.2 } else { 0.0 };
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, vd))
            .expect("drone setpoint");
        drone.backend().flush().expect("drone flush");
        rover.backend().flush().expect("rover flush");
        skiff.backend().flush().expect("skiff flush");
        session.step(0.02).expect("properties hold");
    }

    let world = session.world();
    println!(
        "t={:.2}s  hold={}  drone_alt={:.2}  rover_n={:.2}  skiff_e={:.2}",
        world.t,
        world.all_hold(),
        world.body("drone").unwrap().altitude_agl(),
        world.body("rover").unwrap().position_m[0],
        world.body("skiff").unwrap().position_m[1]
    );
}
