//! PX4 SITL hover using the typed vehicle API.
//!
//! Requires a running PX4 SITL instance that publishes MAVLink on UDP 14540:
//!
//! ```text
//! make px4_sitl gz_x500
//! cargo run -p flight-px4 --example sitl_hover
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

    for _ in 0..50 {
        vehicle
            .set_velocity(Velocity::<Ned>::ned(0.0, 0.0, 0.0))
            .await
            .expect("hover");
    }

    let _ = vehicle.land().await.expect("land");
    println!("landed");
}
