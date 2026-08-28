//! Connect → preflight → arm → takeoff → velocity → land, entirely in the sim.

use flight_core::prelude::*;
use flight_core::units::Qty;
use flight_sim::{connect, SimConfig};

#[tokio::main]
async fn main() {
    let vehicle = connect(SimConfig::default()).await.expect("connect");
    println!("connected  phase={}", vehicle.phase());

    let vehicle = vehicle.verify_preflight().await.expect("preflight");
    println!("preflight  imu+estimator ok");

    let vehicle = vehicle.arm().await.expect("arm");
    println!("armed");

    let mut vehicle = vehicle
        .enter_offboard()
        .await
        .expect("offboard")
        .takeoff(Qty::from_meters(5.0))
        .await
        .expect("takeoff");
    println!(
        "airborne   alt={:.2} m",
        vehicle.telemetry().await.unwrap().altitude_agl().get()
    );

    for _ in 0..100 {
        vehicle
            .set_velocity(Velocity::<Ned>::ned(1.2, 0.4, 0.0))
            .await
            .expect("velocity");
    }
    let tel = vehicle.telemetry().await.unwrap();
    println!(
        "cruise     n={:.2} e={:.2} d={:.2}",
        tel.position.x(),
        tel.position.y(),
        tel.position.z()
    );

    let landed = vehicle.land().await.expect("land");
    let tel = landed.backend().physics();
    println!(
        "landed     alt={:.2} m  armed={}",
        tel.position().altitude_agl().get(),
        landed.safety().armed
    );
}
