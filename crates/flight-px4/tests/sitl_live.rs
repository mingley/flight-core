//! Live PX4 companion path. Ignored unless a SITL instance is up.
//!
//! Default `cargo test --workspace` skips this. The `sitl` CI job (and a
//! local SIH binary) run it with `--ignored`.
//!
//! ```text
//! docker run --rm --network host -e PX4_SIM_MODEL=sihsim_quadx \
//!   px4io/px4-sitl:v1.18.0-beta2 -d
//! cargo test -p flight-px4 --test sitl_live -- --ignored --nocapture
//! ```

use flight_core::prelude::*;
use flight_core::units::Qty;
use flight_core::vehicle::{ErrorKind, VehicleBackend};
use flight_px4::{vehicle, Px4Config};

#[tokio::test]
#[ignore = "needs a live PX4 SITL on UDP 14540 (CI job sitl)"]
async fn sitl_takeoff_hold_land() {
    let cfg = Px4Config::default();
    let vehicle = match vehicle(cfg.clone()).connect().await {
        Ok(v) => v,
        Err(e) => panic!("could not reach PX4 SITL at {}: {}", cfg.endpoint, e.error),
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

    let after_takeoff = vehicle.telemetry().await.expect("telemetry after takeoff");
    assert!(
        after_takeoff.altitude_agl().get() >= 2.5,
        "takeoff must climb through live LOCAL_POSITION_NED, agl={}",
        after_takeoff.altitude_agl().get()
    );

    for _ in 0..10 {
        vehicle
            .set_velocity(Velocity::<Ned>::ned(0.0, 0.0, 0.0))
            .await
            .expect("velocity settle");
    }

    let hold_pose = vehicle
        .telemetry()
        .await
        .expect("pose before hold")
        .position;
    for _ in 0..25 {
        vehicle.hold().await.expect("hold");
    }

    let tel = vehicle.telemetry().await.expect("telemetry after hold");
    assert!(
        tel.position.x().is_finite()
            && tel.position.y().is_finite()
            && tel.position.z().is_finite(),
        "LOCAL_POSITION_NED after hold must be finite, got {:?}",
        tel.position
    );
    let dx = tel.position.x() - hold_pose.x();
    let dy = tel.position.y() - hold_pose.y();
    let dz = tel.position.z() - hold_pose.z();
    let drift = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(
        drift < 2.0,
        "hold should keep the live pose within 2 m, drift={drift} from {:?} to {:?}",
        hold_pose,
        tel.position
    );

    let target = Position::<Ned>::ned(hold_pose.x() + 0.5, hold_pose.y(), hold_pose.z());
    vehicle.set_position(target).await.expect("position");

    let _ = vehicle.land().await.expect("land");
}

#[tokio::test]
#[ignore = "needs a live PX4 SITL on UDP 14540 (CI job sitl)"]
async fn sitl_gps_loss_revokes_leftover_offboard() {
    let cfg = Px4Config::default();
    let vehicle = match vehicle(cfg.clone()).connect().await {
        Ok(v) => v,
        Err(e) => panic!("could not reach PX4 SITL at {}: {}", cfg.endpoint, e.error),
    };

    let vehicle = vehicle.verify_preflight().await.expect("preflight");
    let mut vehicle = vehicle
        .arm()
        .await
        .expect("arm")
        .enter_offboard()
        .await
        .expect("offboard");

    assert!(
        vehicle.leftover_commands_stale().is_err(),
        "live Offboard must have authority before GPS-loss inject"
    );
    let epoch0 = vehicle.backend().authority_epoch();
    let before = vehicle
        .telemetry()
        .await
        .expect("telemetry before")
        .to_trace_sample(epoch0);

    vehicle
        .backend_mut()
        .inject_revoke(Event::EstimatorInvalid)
        .expect("live EstimatorInvalid");
    vehicle
        .leftover_commands_stale()
        .expect("leftover Offboard after live GPS-loss inject");

    let epoch1 = vehicle.backend().authority_epoch();
    assert!(epoch1 > epoch0, "live GPS-loss must bump epoch");
    let after = vehicle
        .telemetry()
        .await
        .expect("telemetry after")
        .to_trace_sample(epoch1);
    assert!(after.failsafe, "live GPS-loss must latch failsafe");

    let err = vehicle
        .set_position(Position::<Ned>::ned(0.0, 0.0, -3.0))
        .await
        .expect_err("leftover set_position after GPS-loss");
    assert!(
        matches!(err, ErrorKind::StaleAuthority(_)),
        "leftover set_position must be StaleAuthority, got {err:?}"
    );

    evaluate_trace(&[before, after], AerialOffboard::GPS_LOSS_REQUIRE)
        .expect("live GPS_LOSS require");
    AerialOffboard::evaluate(&[before, after]).expect("live GPS_LOSS capability");

    let _ = vehicle.backend_mut().disarm().await;
}
