//! PX4 SITL hover using the typed vehicle API.
//!
//! Requires a running PX4 SITL instance that publishes MAVLink on UDP 14540.
//! Headless SIH (no Gazebo) is enough:
//!
//! ```text
//! PX4_SIM_MODEL=sihsim_quadx px4 -d          # .deb: sudo apt install ./px4_*.deb
//! docker run --rm --network host -e PX4_SIM_MODEL=sihsim_quadx \
//!   px4io/px4-sitl:v1.18.0-beta2 -d           # Hub has no v1.17.0 tag
//! make px4_sitl gz_x500                      # full Gazebo, from a PX4 tree
//! cargo run -p flight-px4 --example sitl_hover
//! cargo test -p flight-px4 --test sitl_live -- --ignored
//! ```
//!
//! Without SITL this example exits with a connection error — use
//! `cargo run -p flight-sim --example hover` for the in-process vehicle.

use flight_core::prelude::*;
use flight_core::units::Qty;
use flight_px4::{vehicle, Px4Config};

#[tokio::main]
async fn main() {
    let cfg = Px4Config::default();
    println!("connecting to PX4 at {}", cfg.endpoint);

    let vehicle = match vehicle(cfg).connect().await {
        Ok(v) => v,
        Err(e) => {
            eprintln!("could not reach PX4 SITL: {}", e.error);
            eprintln!("start SITL, or run: cargo run -p flight-sim --example hover");
            std::process::exit(1);
        }
    };

    let vehicle = vehicle.verify_preflight().await.expect("preflight");
    let vehicle = vehicle.arm().await.expect("arm");
    let mut vehicle = vehicle
        .enter_offboard()
        .await
        .expect("offboard")
        .takeoff(Qty::from_meters(3.0))
        .await
        .expect("takeoff");

    for _ in 0..20 {
        vehicle
            .set_velocity(Velocity::<Ned>::ned(0.0, 0.0, 0.0))
            .await
            .expect("climb settle");
    }
    for _ in 0..30 {
        vehicle.hold().await.expect("hold");
    }

    let _ = vehicle.land().await.expect("land");
    println!("landed");
}
