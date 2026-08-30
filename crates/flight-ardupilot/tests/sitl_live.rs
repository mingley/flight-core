//! Live ArduCopter companion path. Ignored unless a SITL instance is up.
//!
//! Default `cargo test --workspace` skips this. There is no CI job: B6 is
//! loopback-first (leftover table + UDP ingest). A recorded live pass can land
//! later without a second MAVLink stack.
//!
//! ```text
//! sim_vehicle.py -v ArduCopter -f gazebo-iris --no-rebuild -w
//! # GCS / companion often sees UDP 14550
//! cargo test -p flight-ardupilot --test sitl_live -- --ignored --nocapture
//! ```

use flight_ardupilot::{vehicle, ArduPilotConfig};
use flight_core::units::Qty;

#[tokio::test]
#[ignore = "needs a live ArduCopter SITL on UDP 14550 (loopback-only in default CI)"]
async fn sitl_takeoff_hold_land() {
    let cfg = ArduPilotConfig::default();
    let vehicle = match vehicle(cfg.clone()).connect().await {
        Ok(v) => v,
        Err(e) => panic!(
            "could not reach ArduPilot SITL at {}: {}",
            cfg.endpoint, e.error
        ),
    };

    let vehicle = vehicle.verify_preflight().await.expect("preflight");
    let vehicle = vehicle.arm().await.expect("arm");
    let mut vehicle = vehicle
        .enter_offboard()
        .await
        .expect("guided")
        .takeoff(Qty::from_meters(3.0))
        .await
        .expect("takeoff");

    let after_takeoff = vehicle.telemetry().await.expect("telemetry after takeoff");
    assert!(
        after_takeoff.altitude_agl().get() >= 1.0,
        "takeoff must climb through live LOCAL_POSITION_NED, agl={}",
        after_takeoff.altitude_agl().get()
    );

    vehicle.hold().await.expect("hold");
    let _ = vehicle.land().await.expect("land");
}
