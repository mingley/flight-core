//! Drive the coastal drone from a ROS 2 Twist (ENU) through the verified plant.

use flight_ros2::plant;
use flight_sim::WorldSession;

fn main() {
    let session = WorldSession::coastal(1);
    let mut drone = session.attach_takeoff("drone").expect("takeoff");
    for k in 0..150 {
        plant::apply_twist_linear(&mut drone, [0.0, 0.0, 1.0]).expect("twist");
        session.step(0.02).expect("properties");
        if k % 25 == 0 {
            let world = session.world();
            let body = world.body("drone").expect("drone");
            println!(
                "t={:.2}  alt={:.2} m  hold={}",
                world.t,
                body.altitude_agl(),
                if world.all_hold() { "ALL HOLD" } else { "FAIL" }
            );
        }
    }
}
