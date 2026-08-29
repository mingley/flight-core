//! PX4 offboard setpoints driving the verified coastal world (no PX4 binary).
//!
//! `Px4Backend` sends `SET_POSITION_TARGET_LOCAL_NED`. `WorldPlant` applies
//! that same message, steps `robot-world`, and publishes `LOCAL_POSITION_NED`.

use flight_mavlink::{arm_disarm, set_offboard_mode, set_velocity_ned};
use flight_px4::WorldPlant;
use mavlink::common::MavMessage;

fn main() {
    let mut plant = WorldPlant::coastal(1);
    plant.apply_mavlink(&arm_disarm(1, 1, true)).expect("arm");
    plant
        .apply_mavlink(&set_offboard_mode(1, 1, true))
        .expect("offboard");
    plant
        .apply_mavlink(&set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2))
        .expect("climb");

    for k in 0..200 {
        let msg = plant.tick(0.02).expect("tick");
        if k % 50 == 49 {
            if let MavMessage::LOCAL_POSITION_NED(p) = msg {
                println!(
                    "t={:.1}s  n={:.2} e={:.2} d={:.2}  hold={}",
                    plant.world().t,
                    p.x,
                    p.y,
                    p.z,
                    plant.world().all_hold()
                );
            }
        }
    }
    let alt = plant.world().body("drone").unwrap().altitude_agl();
    println!("final alt={alt:.2} m");
}
