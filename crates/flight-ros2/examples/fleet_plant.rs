//! Drive catalog bodies from ENU Twist through one verified step.

use flight_ros2::plant::{FleetPlant, FleetTwist};

fn main() {
    let mut plant = FleetPlant::coastal(1);
    plant
        .grant_all()
        .expect("grant air / ground / surface / underwater");
    let cmd = FleetTwist {
        drone: Some([0.0, 0.0, 1.0]),
        rover: Some([0.0, -0.6, 0.0]),
        skiff: Some([0.5, 0.0, 0.0]),
        surveyor: Some([0.0, 0.3, 0.0]),
    };
    for k in 0..150 {
        plant.apply_twists(cmd).expect("twist");
        plant.step(0.02).expect("properties");
        if k % 25 == 0 {
            let world = plant.session().world();
            let drone = world.body("drone").expect("drone");
            let rover = world.body("rover").expect("rover");
            let skiff = world.body("skiff").expect("skiff");
            let surveyor = world.body("surveyor").expect("surveyor");
            println!(
                "t={:.2}  hold={}  drone_alt={:.2}  rover_n={:.2}  skiff_e={:.2}  auv_n={:.2}",
                world.t,
                if world.all_hold() { "ALL HOLD" } else { "FAIL" },
                drone.altitude_agl(),
                rover.position_m[0],
                skiff.position_m[1],
                surveyor.position_m[0]
            );
        }
    }
}
