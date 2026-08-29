//! Controller reads [`FuzzedImu`] around [`WorldImu`]. The plant remains
//! [`WorldSession::step`]. Unusable samples trip failsafe (fail closed).
//!
//! ```bash
//! cargo run -p flight-sim --example fuzzed_world
//! ```

use flight_core::sensors::Imu;
use flight_sim::{FuzzedImu, WorldSession};

fn main() {
    let session = WorldSession::inland(3);
    session.attach_takeoff("drone").expect("takeoff");
    let mut imu = FuzzedImu::new(session.imu("drone"), 7, 0.2, 0.05);
    for _ in 0..80 {
        let sample = imu.sample().expect("imu");
        if sample.is_usable() {
            session.attach_hold("drone").expect("hold");
        } else {
            session.attach_failsafe("drone").expect("fail closed");
            break;
        }
        session.step(0.02).expect("properties");
    }
    let world = session.world();
    let drone = world.body("drone").expect("drone");
    println!(
        "all_hold={} failsafe={} hold_ned={:?}",
        world.all_hold(),
        drone.aerial.expect("aerial").failsafe,
        drone.hold_ned
    );
    if !world.all_hold() {
        std::process::exit(1);
    }
}
