use super::*;
use flight_core::frames::Body as BodyFrame;
use flight_core::ground::GroundPhase;
use flight_core::marine::MarinePhase;
use flight_core::prelude::*;
use flight_core::safety::Phase;
use flight_core::units::Qty;
use flight_core::vehicle::{
    BackendError, GroundHandle, GroundVehicle, MarineHandle, MarineVehicle, VehicleBackend,
    VehicleHandle,
};

#[test]
fn catalogs_match_the_named_world_scenes() {
    let harbor = WorldSession::harbor(1).world();
    assert_eq!(harbor.scenario, "harbor");
    assert!(harbor.body("rover").is_some());
    assert!(harbor.body("skiff").is_some());
    assert!(harbor.body("surveyor").is_some());
    let water = WorldSession::open_water(1).world();
    assert_eq!(water.scenario, "open_water");
    assert!(water.body("rover").is_none());
    assert!(water.body("skiff").is_some());
    assert!(water.body("surveyor").is_some());
    assert!(WorldSession::named("harbor", 1).is_some());
    assert!(WorldSession::named("open_water", 1).is_some());
    assert_eq!(WorldBackend::harbor(1).world().scenario, "harbor");
    assert!(WorldBackend::inland(1).world().body("skiff").is_none());
    assert!(WorldBackend::open_water(1).world().body("rover").is_none());
}

#[test]
fn rejected_step_does_not_advance_clock() {
    use flight_core::time::Clock;
    let session = WorldSession::coastal(1);
    session.with_world_mut(|w| w.hydro.volume0 = 1.0e9);
    let t0 = session.world().t;
    let now0 = session.aerial("drone").now();
    assert!(session.step(0.02).is_err());
    assert_eq!(session.world().t, t0);
    assert_eq!(session.aerial("drone").now(), now0);
    assert!(!session.world().all_hold());
}

#[test]
fn telemetry_now_does_not_step() {
    let session = WorldSession::coastal(1);
    let mut drone = session.aerial("drone");
    let t0 = session.world().t;
    let tel = drone.telemetry_now().unwrap();
    assert_eq!(session.world().t, t0);
    assert_eq!(
        tel.position.x(),
        session.world().body("drone").unwrap().position_m[0]
    );
    assert!(!tel.armed);
    let mut rover = session.ground("rover");
    let g = rover.telemetry_now().unwrap();
    assert_eq!(
        g.position.x(),
        session.world().body("rover").unwrap().position_m[0]
    );
}

#[tokio::test]
async fn typestate_takeoff_in_verified_world() {
    let session = WorldSession::coastal(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drones start Ready");
    };
    let vehicle = drone
        .arm()
        .await
        .unwrap()
        .takeoff(Qty::from_meters(4.0))
        .await
        .expect("takeoff in world");
    let alt = vehicle
        .backend()
        .world()
        .body("drone")
        .unwrap()
        .altitude_agl();
    assert!(alt > 3.5, "alt {alt}");
    assert!(vehicle.backend().world().all_hold());
    assert_eq!(
        vehicle
            .backend()
            .world()
            .last_properties
            .iter()
            .find(|p| p.id == "thrust_only_when_granted")
            .map(|p| p.holds),
        Some(true)
    );
    assert_eq!(
        vehicle
            .backend()
            .world()
            .last_properties
            .iter()
            .find(|p| p.id == "aerial_thrust_along_minus_body_z")
            .map(|p| p.holds),
        Some(true)
    );
}

#[tokio::test]
async fn typestate_land_returns_ready_in_verified_world() {
    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drones start Ready");
    };
    let airborne = drone
        .arm()
        .await
        .unwrap()
        .takeoff(Qty::from_meters(3.0))
        .await
        .expect("takeoff");
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Airborne
    );
    let ready = airborne.land().await.expect("land");
    assert_eq!(ready.phase(), Phase::Ready);
    assert!(!ready.safety().armed);
    assert!(!ready.safety().actuators_enabled);
    let w = session.world();
    let s = w.body("drone").unwrap().aerial.unwrap();
    assert_eq!(s.phase, Phase::Ready);
    assert!(!s.armed);
    assert!(w.body("drone").unwrap().command.is_none());
    assert!(w.all_hold());
    let armed = ready.arm_now().unwrap();
    assert!(armed.safety().armed);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Armed
    );
}

#[tokio::test]
async fn typestate_rover_drives_in_verified_world() {
    let mut rover = GroundVehicle::new(GroundWorldBackend::inland(1))
        .enable_drive()
        .unwrap();
    rover
        .set_velocity_ned(Velocity::<Ned>::ned(-0.9, 0.0, 0.0))
        .await
        .unwrap();
    for _ in 0..100 {
        rover.tick(0.02).await.unwrap();
        assert!(rover.backend().world().all_hold());
    }
    let n = rover.backend().world().body("rover").unwrap().position_m[0];
    assert!(n < 9.5, "rover still at n={n}");
    let g = rover
        .backend()
        .world()
        .body("rover")
        .unwrap()
        .ground
        .unwrap();
    assert!(g.drive_enabled);
}

#[tokio::test]
async fn typestate_skiff_makes_way() {
    let mut skiff = MarineVehicle::new(MarineWorldBackend::coastal_skiff(1))
        .undock()
        .unwrap();
    skiff
        .set_ned_velocity(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .await
        .unwrap();
    let e0 = skiff.backend().world().body("skiff").unwrap().position_m[1];
    for _ in 0..120 {
        skiff.tick(0.02).await.unwrap();
        assert!(skiff.backend().world().all_hold());
    }
    let e1 = skiff.backend().world().body("skiff").unwrap().position_m[1];
    assert!(e1 > e0 + 0.15, "skiff east {e0} -> {e1}");
}

#[test]
fn shared_session_steps_drone_and_rover() {
    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drones start Ready");
    };
    let mut drone = drone
        .arm_now()
        .unwrap()
        .enter_offboard_now()
        .unwrap()
        .start_takeoff_now()
        .unwrap();
    let GroundHandle::Parked(rover) = session.ground("rover").attach().unwrap() else {
        panic!("world rovers start Parked");
    };
    let mut rover = rover.enable_drive().unwrap();
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.7, 0.0, 0.0))
        .unwrap();
    for _ in 0..200 {
        let alt = session.world().body("drone").unwrap().altitude_agl();
        let vd = if alt < 3.0 { -1.2 } else { 0.0 };
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, vd))
            .unwrap();
        drone.backend().flush().unwrap();
        rover.backend().flush().unwrap();
        session.step(0.02).unwrap();
    }
    let world = session.world();
    assert!(world.all_hold(), "{:?}", world.last_properties);
    assert!(world.body("drone").unwrap().altitude_agl() > 2.5);
    assert!(world.body("rover").unwrap().position_m[0] < 10.0);
}

#[test]
fn flush_then_session_step_moves_the_fleet() {
    let session = WorldSession::coastal(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drones start Ready");
    };
    let mut drone = drone
        .arm_now()
        .unwrap()
        .enter_offboard_now()
        .unwrap()
        .start_takeoff_now()
        .unwrap();
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    let GroundHandle::Parked(rover) = session.ground("rover").attach().unwrap() else {
        panic!("world rovers start Parked");
    };
    let mut rover = rover.enable_drive().unwrap();
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.6, 0.0, 0.0))
        .unwrap();
    let MarineHandle::Docked(skiff) = session.marine("skiff").attach().unwrap() else {
        panic!("world hulls start Docked");
    };
    let mut skiff = skiff.undock().unwrap();
    skiff
        .set_ned_velocity_now(Velocity::<Ned>::ned(0.0, 0.5, 0.0))
        .unwrap();
    let t0 = session.world().t;
    for _ in 0..120 {
        let alt = session.world().body("drone").unwrap().altitude_agl();
        let vd = if alt < 3.0 { -1.2 } else { 0.0 };
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, vd))
            .unwrap();
        drone.backend().flush().unwrap();
        rover.backend().flush().unwrap();
        skiff.backend().flush().unwrap();
        session.step(0.02).unwrap();
    }
    let world = session.world();
    assert!((world.t - t0 - 2.4).abs() < 1e-3, "t {}", world.t);
    assert!(world.all_hold(), "{:?}", world.last_properties);
    assert!(world.body("drone").unwrap().altitude_agl() > 2.0);
    assert!(world.body("rover").unwrap().position_m[0] < 14.0);
    assert!(world.body("skiff").unwrap().position_m[1] > -2.0);
}

#[test]
fn attach_drive_then_moves_rover() {
    let session = WorldSession::inland(1);
    let mut rover = session.attach_drive("rover").unwrap();
    let n0 = session.world().body("rover").unwrap().position_m[0];
    rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.flush().unwrap();
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let world = session.world();
    let n1 = world.body("rover").unwrap().position_m[0];
    assert!(n1 < n0 - 0.15, "attach_drive south {n0} → {n1}");
    assert!(world.body("rover").unwrap().ground.unwrap().drive_enabled);
    assert!(world.all_hold());
}

#[test]
fn halt_now_parks_rover_and_clears_command() {
    let session = WorldSession::inland(1);
    let mut rover = session.attach_drive("rover").unwrap();
    rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.flush().unwrap();
    session.step(0.02).unwrap();
    rover.halt_now().unwrap();
    let w = session.world();
    let g = w.body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.drive_enabled);
    assert!(w.body("rover").unwrap().command.is_none());
    assert!(rover.halt_now().is_err());
    assert!(w.all_hold());
}

#[test]
fn attach_undock_then_moves_skiff() {
    let session = WorldSession::coastal(1);
    let mut skiff = session.attach_undock("skiff").unwrap();
    let e0 = session.world().body("skiff").unwrap().position_m[1];
    skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.flush().unwrap();
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let world = session.world();
    let e1 = world.body("skiff").unwrap().position_m[1];
    assert!(e1 > e0 + 0.08, "attach_undock east {e0} → {e1}");
    assert!(world.body("skiff").unwrap().marine.unwrap().thrust_enabled);
    assert!(world.all_hold());
}

#[test]
fn grant_shortcuts_match_attach_helpers() {
    let granted = WorldSession::coastal(1);
    let attached = WorldSession::coastal(1);
    granted.aerial("drone").grant_offboard().unwrap();
    attached.attach_takeoff("drone").unwrap();
    granted.ground("rover").grant_drive().unwrap();
    attached.attach_drive("rover").unwrap();
    granted.marine("skiff").grant_undock().unwrap();
    attached.attach_undock("skiff").unwrap();
    let wa = granted.world();
    let wb = attached.world();
    for id in ["drone", "rover", "skiff"] {
        let a = wa.body(id).unwrap();
        let b = wb.body(id).unwrap();
        assert_eq!(a.aerial, b.aerial, "{id} aerial");
        assert_eq!(a.ground, b.ground, "{id} ground");
        assert_eq!(a.marine, b.marine, "{id} marine");
        assert_eq!(a.command, b.command, "{id} command");
    }
}

#[test]
fn grant_twice_is_protocol() {
    let session = WorldSession::coastal(1);
    let mut drone = session.aerial("drone");
    drone.grant_offboard().unwrap();
    assert!(matches!(
        drone.grant_offboard(),
        Err(BackendError::Protocol)
    ));
    let mut rover = session.ground("rover");
    rover.grant_drive().unwrap();
    assert!(matches!(rover.grant_drive(), Err(BackendError::Protocol)));
    let mut skiff = session.marine("skiff");
    skiff.grant_undock().unwrap();
    assert!(matches!(skiff.grant_undock(), Err(BackendError::Protocol)));
}

#[test]
fn failsafe_now_matches_attach_estop_and_marine_failsafe() {
    let granted = WorldSession::coastal(1);
    let attached = WorldSession::coastal(1);
    granted.ground("rover").grant_drive().unwrap();
    attached.attach_drive("rover").unwrap();
    granted.ground("rover").failsafe_now().unwrap();
    attached.attach_estop("rover").unwrap();
    granted.marine("skiff").grant_undock().unwrap();
    attached.attach_undock("skiff").unwrap();
    granted.marine("skiff").failsafe_now().unwrap();
    attached.attach_marine_failsafe("skiff").unwrap();
    let wa = granted.world();
    let wb = attached.world();
    assert_eq!(
        wa.body("rover").unwrap().ground,
        wb.body("rover").unwrap().ground
    );
    assert_eq!(
        wa.body("skiff").unwrap().marine,
        wb.body("skiff").unwrap().marine
    );
    assert!(wa.body("rover").unwrap().command.is_none());
    assert!(wa.body("skiff").unwrap().command.is_none());
    assert!(granted.ground("rover").failsafe_now().is_ok());
    assert!(granted.marine("skiff").failsafe_now().is_ok());
}

#[test]
fn failsafe_now_matches_attach_failsafe() {
    let kernel = WorldSession::coastal(1);
    let attached = WorldSession::coastal(1);
    kernel.attach_takeoff("drone").unwrap();
    attached.attach_takeoff("drone").unwrap();
    kernel.aerial("drone").failsafe_now().unwrap();
    attached.attach_failsafe("drone").unwrap();
    let wa = kernel.world();
    let wb = attached.world();
    assert_eq!(
        wa.body("drone").unwrap().aerial,
        wb.body("drone").unwrap().aerial
    );
    assert!(wa.body("drone").unwrap().failsafe());
    assert!(wa.body("drone").unwrap().command.is_none());
    assert!(kernel.aerial("drone").failsafe_now().is_ok());
    assert!(matches!(
        attached.attach_failsafe("drone"),
        Err(BackendError::Protocol)
    ));
}

#[test]
fn ground_and_marine_disarm_now_walk_kernel_events() {
    let session = WorldSession::coastal(1);
    let mut rover = session.attach_drive("rover").unwrap();
    rover.disarm_now().unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.drive_enabled);
    assert!(!g.estop);

    let mut rover = session.attach_drive("rover").unwrap();
    rover.failsafe_now().unwrap();
    rover.disarm_now().unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.estop);

    let mut skiff = session.attach_undock("skiff").unwrap();
    skiff.disarm_now().unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.thrust_enabled);
    assert!(!m.failsafe);
}

#[test]
fn dock_now_clears_skiff_thrust() {
    let session = WorldSession::coastal(1);
    let mut skiff = session.attach_undock("skiff").unwrap();
    skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.flush().unwrap();
    session.step(0.02).unwrap();
    skiff.dock_now().unwrap();
    let w = session.world();
    let m = w.body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.thrust_enabled);
    assert!(w.body("skiff").unwrap().command.is_none());
    assert!(w.all_hold());
}

#[test]
fn station_now_then_resume_or_dock() {
    let session = WorldSession::coastal(1);
    let mut skiff = session.attach_undock("skiff").unwrap();
    skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.flush().unwrap();
    for _ in 0..20 {
        session.step(0.02).unwrap();
    }
    skiff.station_now().unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::StationKeep);
    assert!(m.thrust_enabled);

    skiff.resume_now().unwrap();
    assert_eq!(
        session.world().body("skiff").unwrap().marine.unwrap().phase,
        MarinePhase::Underway
    );
    skiff.station_now().unwrap();
    skiff.dock_now().unwrap();
    let w = session.world();
    let m = w.body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.thrust_enabled);
    assert!(w.body("skiff").unwrap().command.is_none());
    assert!(skiff.station_now().is_err());
    assert!(w.all_hold());
}

#[test]
fn land_now_then_touchdown_returns_to_ready() {
    let session = WorldSession::inland(1);
    let mut drone = session.attach_takeoff("drone").unwrap();
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone.flush().unwrap();
    let mut airborne = false;
    for _ in 0..400 {
        session.step(0.02).unwrap();
        let w = session.world();
        let b = w.body("drone").unwrap();
        if b.altitude_agl() >= 2.5 && !b.on_terrain(&w.env) {
            airborne = true;
            break;
        }
    }
    assert!(airborne, "drone never left the pad");
    drone.land_now().unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );

    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, 0.8))
        .unwrap();
    drone.flush().unwrap();
    let mut on_pad = false;
    for _ in 0..400 {
        session.step(0.02).unwrap();
        let w = session.world();
        if w.body("drone").unwrap().on_terrain(&w.env) {
            on_pad = true;
            break;
        }
    }
    assert!(on_pad, "drone never returned to terrain");
    drone.touchdown_now().unwrap();
    let w = session.world();
    let s = w.body("drone").unwrap().aerial.unwrap();
    assert_eq!(s.phase, Phase::Ready);
    assert!(!s.armed);
    assert!(!s.actuators_enabled);
    assert!(w.body("drone").unwrap().command.is_none());
    assert!(w.all_hold());
    assert!(drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .is_err());
}

#[test]
fn now_setpoints_refuse_the_same_illegal_grants_as_lab_act() {
    let inland = WorldSession::inland(1);
    let mut rover = inland.ground("rover");
    assert!(rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .is_err());
    let mut rover = inland.attach_drive("rover").unwrap();
    rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.flush().unwrap();
    rover.halt_now().unwrap();
    assert!(rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .is_err());

    let coastal = WorldSession::coastal(1);
    let mut skiff = coastal.marine("skiff");
    assert!(skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .is_err());
    let mut skiff = coastal.attach_undock("skiff").unwrap();
    skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.flush().unwrap();
    skiff.dock_now().unwrap();
    assert!(skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .is_err());

    let mut drone = coastal.aerial("drone");
    assert!(drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .is_err());
    assert!(drone
        .set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
        .is_err());
    let mut drone = coastal.attach_takeoff("drone").unwrap();
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone
        .set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
        .unwrap();
    drone.flush().unwrap();
}

#[test]
fn attach_reads_live_ground_phase_without_reset() {
    use flight_core::vehicle::GroundHandle;

    let session = WorldSession::inland(1);
    let parked = session.ground("rover").attach().unwrap();
    assert!(matches!(parked, GroundHandle::Parked(_)));
    assert!(
        !session
            .world()
            .body("rover")
            .unwrap()
            .ground
            .unwrap()
            .drive_enabled
    );

    let mut handle = session.ground("rover");
    handle.grant_drive().unwrap();
    let GroundHandle::Moving(rover) = handle.attach().unwrap() else {
        panic!("grant_drive then attach must be Moving, not a fresh Parked");
    };
    assert!(rover.safety().drive_enabled);
    assert!(
        session
            .world()
            .body("rover")
            .unwrap()
            .ground
            .unwrap()
            .drive_enabled
    );
}

#[test]
fn attach_moving_rover_can_command_through_typestate() {
    use flight_core::vehicle::GroundHandle;

    let session = WorldSession::inland(1);
    let mut handle = session.ground("rover");
    handle.grant_drive().unwrap();
    let GroundHandle::Moving(mut rover) = handle.attach().unwrap() else {
        panic!("expected Moving");
    };
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.backend().flush().unwrap();
    let n0 = session.world().body("rover").unwrap().position_m[0];
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let n1 = session.world().body("rover").unwrap().position_m[0];
    assert!(n1 < n0 - 0.15, "attached Moving rover south {n0} → {n1}");
    assert!(session.world().all_hold());
}

#[test]
fn attach_park_now_halts_the_chassis() {
    use flight_core::vehicle::GroundHandle;

    let session = WorldSession::inland(1);
    let mut handle = session.ground("rover");
    handle.grant_drive().unwrap();
    let GroundHandle::Moving(mut rover) = handle.attach().unwrap() else {
        panic!("expected Moving");
    };
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let parked = rover.park_now();
    assert_eq!(parked.phase(), GroundPhase::Parked);
    assert!(!parked.safety().drive_enabled);
    let w = session.world();
    let g = w.body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.drive_enabled);
    assert!(w.body("rover").unwrap().command.is_none());
    assert!(w.all_hold());
    parked.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let w = session.world();
    assert_eq!(
        w.body("rover").unwrap().ground.unwrap().phase,
        GroundPhase::Parked
    );
    assert!(w.body("rover").unwrap().command.is_none());
    assert!(w.all_hold());
}

#[test]
fn attach_emergency_stop_now_trips_the_chassis() {
    use flight_core::vehicle::GroundHandle;

    let session = WorldSession::inland(1);
    let GroundHandle::Parked(rover) = session.ground("rover").attach().unwrap() else {
        panic!("world rovers start Parked");
    };
    let mut rover = rover.enable_drive().unwrap();
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let stopped = rover.emergency_stop_now();
    assert_eq!(stopped.phase(), GroundPhase::EStop);
    assert!(stopped.safety().estop);
    let w = session.world();
    let g = w.body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::EStop);
    assert!(g.estop && !g.drive_enabled);
    assert!(w.body("rover").unwrap().command.is_none());
    stopped.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let w = session.world();
    assert_eq!(
        w.body("rover").unwrap().ground.unwrap().phase,
        GroundPhase::EStop
    );
    assert!(w.body("rover").unwrap().command.is_none());
    assert!(w.all_hold());
}

#[test]
fn attach_reads_live_marine_phase_without_reset() {
    use flight_core::vehicle::MarineHandle;

    let session = WorldSession::coastal(1);
    let docked = session.marine("skiff").attach().unwrap();
    assert!(matches!(docked, MarineHandle::Docked(_)));

    let mut handle = session.marine("skiff");
    handle.grant_undock().unwrap();
    let MarineHandle::Underway(skiff) = handle.attach().unwrap() else {
        panic!("grant_undock then attach must be Underway, not a fresh Docked");
    };
    assert!(skiff.safety().thrust_enabled);
    assert!(
        session
            .world()
            .body("skiff")
            .unwrap()
            .marine
            .unwrap()
            .thrust_enabled
    );
}

#[test]
fn attach_dock_now_clears_skiff_thrust() {
    use flight_core::vehicle::MarineHandle;

    let session = WorldSession::coastal(1);
    let mut handle = session.marine("skiff");
    handle.grant_undock().unwrap();
    let MarineHandle::Underway(mut skiff) = handle.attach().unwrap() else {
        panic!("expected Underway");
    };
    skiff
        .set_ned_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let station = skiff.hold_station().unwrap();
    assert_eq!(station.phase(), MarinePhase::StationKeep);
    let docked = station.dock_now();
    assert_eq!(docked.phase(), MarinePhase::Docked);
    assert!(!docked.safety().thrust_enabled);
    let w = session.world();
    let m = w.body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.thrust_enabled);
    assert!(w.body("skiff").unwrap().command.is_none());
    assert!(w.all_hold());
}

#[test]
fn attach_failsafe_from_station_keep_trips_the_hull() {
    use flight_core::vehicle::MarineHandle;

    let session = WorldSession::coastal(1);
    let MarineHandle::Docked(skiff) = session.marine("skiff").attach().unwrap() else {
        panic!("world hulls start Docked");
    };
    let mut skiff = skiff.undock().unwrap();
    skiff
        .set_ned_velocity_now(Velocity::<Ned>::ned(0.0, 0.4, 0.0))
        .unwrap();
    skiff.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let station = skiff.hold_station().unwrap();
    let fs = station.declare_failsafe();
    assert_eq!(fs.phase(), MarinePhase::Failsafe);
    assert!(fs.safety().failsafe);
    let w = session.world();
    let m = w.body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Failsafe);
    assert!(m.failsafe && !m.thrust_enabled);
    assert!(w.body("skiff").unwrap().command.is_none());
    fs.backend().flush().unwrap();
    session.step(0.02).unwrap();
    let w = session.world();
    assert_eq!(
        w.body("skiff").unwrap().marine.unwrap().phase,
        MarinePhase::Failsafe
    );
    assert!(w.body("skiff").unwrap().command.is_none());
    assert!(w.all_hold());
}

#[test]
fn attach_reads_live_aerial_phase_without_reset() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let ready = session.aerial("drone").attach().unwrap();
    assert!(matches!(ready, VehicleHandle::PreflightReady(_)));
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Ready
    );

    let mut handle = session.aerial("drone");
    handle.grant_offboard().unwrap();
    let VehicleHandle::Takeoff(drone) = handle.attach().unwrap() else {
        panic!("grant_offboard then attach must be Takeoff, not Disconnected");
    };
    assert!(drone.safety().armed && drone.safety().offboard);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Takeoff
    );
}

#[tokio::test]
async fn attach_ready_drone_arms_through_typestate() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drone starts Ready");
    };
    let armed = drone.arm().await.unwrap();
    assert!(armed.safety().armed);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Armed
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_offboard_drone_can_command_through_typestate() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let mut handle = session.aerial("drone");
    handle.grant_offboard().unwrap();
    let VehicleHandle::Takeoff(mut drone) = handle.attach().unwrap() else {
        panic!("expected Takeoff");
    };
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone.backend().flush().unwrap();
    let alt0 = session.world().body("drone").unwrap().altitude_agl();
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let alt1 = session.world().body("drone").unwrap().altitude_agl();
    assert!(alt1 > alt0 + 0.3, "attached Takeoff climb {alt0} → {alt1}");
    assert!(session.world().all_hold());
}

#[test]
fn attach_start_takeoff_now_makes_land_legal_on_the_plant() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drone starts Ready");
    };
    let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
    let climbing = offboard.start_takeoff_now().unwrap();
    assert_eq!(climbing.phase(), Phase::Takeoff);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Takeoff
    );
    let landing = climbing.begin_land_now().unwrap();
    assert_eq!(landing.phase(), Phase::Landing);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_declare_airborne_now_binds_airborne_on_the_plant() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drone starts Ready");
    };
    let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
    let climbing = offboard.start_takeoff_now().unwrap();
    let airborne = climbing.declare_airborne_now().unwrap();
    assert_eq!(airborne.phase(), Phase::Airborne);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Airborne
    );
    let VehicleHandle::Airborne(_) = session.aerial("drone").attach().unwrap() else {
        panic!("ReachedAltitude maps to Airborne");
    };
    let landing = airborne.begin_land_now().unwrap();
    assert_eq!(landing.phase(), Phase::Landing);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_failsafe_now_trips_the_plant() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("world drone starts Ready");
    };
    let offboard = drone.arm_now().unwrap().enter_offboard_now().unwrap();
    let fs = offboard.failsafe_now().unwrap();
    assert!(fs.safety().failsafe);
    assert_eq!(fs.phase(), Phase::Failsafe);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Failsafe
    );
    assert!(session.world().body("drone").unwrap().command.is_none());
    assert!(session.world().all_hold());
    let VehicleHandle::Failsafe(_) = session.aerial("drone").attach().unwrap() else {
        panic!("failsafe maps to Failsafe");
    };
}

#[test]
fn attach_failsafe_from_ready_trips_the_plant() {
    let session = WorldSession::inland(1);
    session.attach_failsafe("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Failsafe);
    assert!(aerial.failsafe && !aerial.offboard);
    let VehicleHandle::Failsafe(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_failsafe from Ready must bind Failsafe");
    };
    assert_eq!(
        session.attach_failsafe("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_estop_from_parked_trips_the_chassis() {
    let session = WorldSession::inland(1);
    session.attach_estop("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::EStop);
    assert!(g.estop && !g.drive_enabled);
    let GroundHandle::EStopped(_) = session.ground("rover").attach().unwrap() else {
        panic!("attach_estop from Parked must bind EStopped");
    };
    assert_eq!(
        session.attach_estop("rover").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_reset("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.estop);
    assert!(session.world().all_hold());
}

#[test]
fn attach_begin_land_now_then_touchdown_now_returns_ready() {
    use flight_core::vehicle::VehicleHandle;

    let session = WorldSession::inland(1);
    let mut handle = session.aerial("drone");
    handle.grant_offboard().unwrap();
    handle
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    handle.flush().unwrap();
    let mut airborne = false;
    for _ in 0..400 {
        session.step(0.02).unwrap();
        let w = session.world();
        let b = w.body("drone").unwrap();
        if b.altitude_agl() >= 2.5 && !b.on_terrain(&w.env) {
            airborne = true;
            break;
        }
    }
    assert!(airborne, "drone never left the pad");

    let VehicleHandle::Takeoff(drone) = session.aerial("drone").attach().unwrap() else {
        panic!("takeoff maps to Takeoff");
    };
    let mut landing = drone.begin_land_now().unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    landing
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, 0.8))
        .unwrap();
    landing.backend().flush().unwrap();
    let mut on_pad = false;
    for _ in 0..400 {
        session.step(0.02).unwrap();
        let w = session.world();
        if w.body("drone").unwrap().on_terrain(&w.env) {
            on_pad = true;
            break;
        }
    }
    assert!(on_pad, "drone never returned to terrain");
    let ready = landing.touchdown_now().unwrap();
    assert_eq!(ready.phase(), Phase::Ready);
    assert!(!ready.safety().armed);
    let w = session.world();
    let s = w.body("drone").unwrap().aerial.unwrap();
    assert_eq!(s.phase, Phase::Ready);
    assert!(w.body("drone").unwrap().command.is_none());
    assert!(w.all_hold());
    let armed = ready.arm_now().unwrap();
    assert!(armed.safety().armed);
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Armed
    );
}

#[test]
fn attach_offboard_grants_actuators_without_takeoff() {
    let session = WorldSession::inland(1);
    let mut drone = session.attach_offboard("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Armed);
    assert!(aerial.armed && aerial.offboard && aerial.actuators_enabled);
    let VehicleHandle::Offboard(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_offboard must bind Offboard, not Takeoff");
    };
    assert!(drone.land_now().is_err(), "Land requires Takeoff");
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone.flush().unwrap();
    let alt0 = session.world().body("drone").unwrap().altitude_agl();
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let alt1 = session.world().body("drone").unwrap().altitude_agl();
    assert!(alt1 > alt0 + 0.3, "attach_offboard climb {alt0} → {alt1}");
    drone.takeoff_now().unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Takeoff
    );
    drone.land_now().unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    assert!(session.world().all_hold());
    assert_eq!(
        session.attach_offboard("drone").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_takeoff_walks_ready_to_takeoff_on_the_plant() {
    let session = WorldSession::inland(1);
    let mut drone = session.attach_takeoff("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Takeoff);
    assert!(aerial.armed && aerial.offboard && aerial.actuators_enabled);
    assert!(session.world().body("drone").unwrap().actuators_granted());
    let VehicleHandle::Takeoff(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_takeoff must bind Takeoff");
    };
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone.flush().unwrap();
    let alt0 = session.world().body("drone").unwrap().altitude_agl();
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let alt1 = session.world().body("drone").unwrap().altitude_agl();
    assert!(alt1 > alt0 + 0.3, "attach_takeoff climb {alt0} → {alt1}");
    drone.land_now().unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    assert!(session.world().all_hold());
    assert_eq!(
        session.attach_takeoff("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert_eq!(
        session.attach_takeoff("rover").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_start_takeoff_walks_offboard_to_takeoff() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_start_takeoff("drone").unwrap_err(),
        BackendError::Protocol
    );
    let _ = session.attach_offboard("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Armed
    );
    let drone = session.attach_start_takeoff("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Takeoff
    );
    let VehicleHandle::Takeoff(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_start_takeoff must bind Takeoff");
    };
    let _ = drone;
    assert_eq!(
        session.attach_start_takeoff("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert_eq!(
        session.attach_offboard("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_drive_enables_a_parked_chassis() {
    let session = WorldSession::inland(1);
    let mut rover = session.attach_drive("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Moving);
    assert!(g.drive_enabled && !g.estop);
    let GroundHandle::Moving(_) = session.ground("rover").attach().unwrap() else {
        panic!("attach_drive must bind Moving");
    };
    rover
        .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.flush().unwrap();
    let n0 = session.world().body("rover").unwrap().position_m[0];
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let n1 = session.world().body("rover").unwrap().position_m[0];
    assert!(n1 < n0 - 0.15, "attach_drive south {n0} → {n1}");
    assert!(session.world().all_hold());
    assert_eq!(
        session.attach_drive("rover").unwrap_err(),
        BackendError::Protocol
    );
    assert_eq!(
        session.attach_drive("drone").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_undock_makes_way_on_a_docked_hull() {
    let session = WorldSession::coastal(1);
    let mut skiff = session.attach_undock("skiff").unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Underway);
    assert!(m.thrust_enabled && !m.failsafe);
    let MarineHandle::Underway(_) = session.marine("skiff").attach().unwrap() else {
        panic!("attach_undock must bind Underway");
    };
    skiff
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
        .unwrap();
    skiff.flush().unwrap();
    let e0 = session.world().body("skiff").unwrap().position_m[1];
    for _ in 0..40 {
        session.step(0.02).unwrap();
    }
    let e1 = session.world().body("skiff").unwrap().position_m[1];
    assert!(e1 > e0 + 0.08, "attach_undock east {e0} → {e1}");
    assert!(session.world().all_hold());
    assert_eq!(
        session.attach_undock("skiff").unwrap_err(),
        BackendError::Protocol
    );
    assert_eq!(
        WorldSession::inland(1).attach_undock("rover").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_land_and_touchdown_walk_takeoff_to_ready() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_land("drone").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_offboard("drone").unwrap();
    assert_eq!(
        session.attach_land("drone").unwrap_err(),
        BackendError::Protocol
    );

    let session = WorldSession::inland(2);
    session.attach_takeoff("drone").unwrap();
    session.attach_land("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Landing
    );
    let VehicleHandle::Landing(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_land must bind Landing");
    };
    session.attach_touchdown("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Ready);
    assert!(!aerial.armed && !aerial.actuators_enabled);
    assert!(session.world().body("drone").unwrap().command.is_none());
    let VehicleHandle::PreflightReady(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_touchdown must bind Ready");
    };
    assert_eq!(
        session.attach_touchdown("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_touchdown_from_failsafe_returns_ready() {
    let session = WorldSession::inland(1);
    session.attach_takeoff("drone").unwrap();
    session.attach_failsafe("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Failsafe
    );
    session.attach_touchdown("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Ready);
    assert!(!aerial.armed && !aerial.failsafe && !aerial.actuators_enabled);
    assert!(session.world().body("drone").unwrap().command.is_none());
    let VehicleHandle::PreflightReady(_) = session.aerial("drone").attach().unwrap() else {
        panic!("failsafe touchdown must bind Ready");
    };
    assert_eq!(
        session.attach_touchdown("drone").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_park_and_estop_walk_the_chassis() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_park("rover").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_drive("rover").unwrap();
    session.attach_park("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.drive_enabled);
    assert!(session.world().body("rover").unwrap().command.is_none());
    let GroundHandle::Parked(_) = session.ground("rover").attach().unwrap() else {
        panic!("attach_park must bind Parked");
    };

    session.attach_drive("rover").unwrap();
    session.attach_estop("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::EStop);
    assert!(g.estop && !g.drive_enabled);
    assert_eq!(
        session.attach_estop("rover").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_station_resume_and_dock_walk_the_hull() {
    let session = WorldSession::coastal(1);
    assert_eq!(
        session.attach_station("skiff").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_undock("skiff").unwrap();
    session.attach_station("skiff").unwrap();
    assert_eq!(
        session.world().body("skiff").unwrap().marine.unwrap().phase,
        MarinePhase::StationKeep
    );
    let MarineHandle::StationKeep(_) = session.marine("skiff").attach().unwrap() else {
        panic!("attach_station must bind StationKeep");
    };
    session.attach_resume("skiff").unwrap();
    assert_eq!(
        session.world().body("skiff").unwrap().marine.unwrap().phase,
        MarinePhase::Underway
    );
    session.attach_dock("skiff").unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.thrust_enabled);
    assert!(session.world().body("skiff").unwrap().command.is_none());
    assert_eq!(
        session.attach_dock("skiff").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_hold_sets_ned_pose_from_offboard_control() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_hold("drone").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_takeoff("drone").unwrap();
    let pose = session.world().body("drone").unwrap().position_m;
    session.attach_hold("drone").unwrap();
    assert_eq!(session.world().body("drone").unwrap().hold_ned, Some(pose));
    session.step(0.02).unwrap();
    assert!(session.world().body("drone").unwrap().hold_ned.is_some());
    assert!(session.world().all_hold());
    session.attach_land("drone").unwrap();
    session.attach_touchdown("drone").unwrap();
    assert_eq!(
        session.attach_hold("drone").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_ground_hold_sets_ned_pose_from_moving() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_ground_hold("rover").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_drive("rover").unwrap();
    let pose = session.world().body("rover").unwrap().position_m;
    session.attach_ground_hold("rover").unwrap();
    assert_eq!(session.world().body("rover").unwrap().hold_ned, Some(pose));
    session.step(0.02).unwrap();
    assert!(session.world().body("rover").unwrap().hold_ned.is_some());
    assert!(session.world().all_hold());
    session.attach_park("rover").unwrap();
    assert!(session.world().body("rover").unwrap().hold_ned.is_none());
    assert_eq!(
        session.attach_ground_hold("rover").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_drive("rover").unwrap();
    session.attach_ground_hold("rover").unwrap();
    assert!(session.world().body("rover").unwrap().hold_ned.is_some());
    session.attach_estop("rover").unwrap();
    assert!(session.world().body("rover").unwrap().hold_ned.is_none());
    assert_eq!(
        session.attach_ground_hold("rover").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_marine_hold_sets_ned_pose_from_underway_and_station() {
    let session = WorldSession::coastal(1);
    assert_eq!(
        session.attach_marine_hold("skiff").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_undock("skiff").unwrap();
    let pose = session.world().body("skiff").unwrap().position_m;
    session.attach_marine_hold("skiff").unwrap();
    assert_eq!(session.world().body("skiff").unwrap().hold_ned, Some(pose));
    session.step(0.02).unwrap();
    assert!(session.world().body("skiff").unwrap().hold_ned.is_some());
    assert!(session.world().all_hold());
    session.attach_dock("skiff").unwrap();
    assert!(session.world().body("skiff").unwrap().hold_ned.is_none());
    assert_eq!(
        session.attach_marine_hold("skiff").unwrap_err(),
        BackendError::Protocol
    );

    session.attach_undock("surveyor").unwrap();
    session.attach_station("surveyor").unwrap();
    let pose = session.world().body("surveyor").unwrap().position_m;
    session.attach_marine_hold("surveyor").unwrap();
    assert_eq!(
        session.world().body("surveyor").unwrap().hold_ned,
        Some(pose)
    );
    assert_eq!(
        session
            .world()
            .body("surveyor")
            .unwrap()
            .marine
            .unwrap()
            .phase,
        MarinePhase::StationKeep
    );
    session.attach_marine_failsafe("surveyor").unwrap();
    assert!(session.world().body("surveyor").unwrap().hold_ned.is_none());
    assert_eq!(
        session.attach_marine_hold("surveyor").unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn fuzzed_world_imu_hold_keeps_properties() {
    use crate::FuzzedImu;
    use flight_core::sensors::Imu;

    let session = WorldSession::inland(3);
    session.attach_takeoff("drone").unwrap();
    let mut imu = FuzzedImu::new(session.imu("drone"), 7, 0.2, 0.05);
    for _ in 0..40 {
        let sample = imu.sample().unwrap();
        assert!(sample.is_finite());
        assert!(sample.is_usable());
        session.update_nav("drone", sample, 0.02).unwrap();
        session.attach_hold("drone").unwrap();
        session.step(0.02).unwrap();
    }
    assert!(session.world().body("drone").unwrap().hold_ned.is_some());
    assert!(
        session
            .world()
            .body("drone")
            .unwrap()
            .aerial
            .unwrap()
            .estimator_valid
    );
    assert!(
        !session
            .world()
            .body("drone")
            .unwrap()
            .aerial
            .unwrap()
            .failsafe
    );
    assert!(session.world().all_hold());
}

fn dead_imu_sample() -> ImuSample<BodyFrame> {
    use flight_core::sensors::{ImuSample, SensorHealth};
    use flight_core::time::MonotonicInstant;
    use flight_core::vector::{Acceleration, AngularVelocity};

    ImuSample {
        timestamp: MonotonicInstant::from_millis(0),
        accel: Acceleration::body(0.0, 0.0, 0.0),
        gyro: AngularVelocity::body_rad(0.0, 0.0, 0.0),
        covariance: None,
        temperature: None,
        status: SensorHealth::Invalid,
        sequence: 0,
    }
}

#[test]
fn nav_warmup_does_not_clear_estimator_valid() {
    let session = WorldSession::inland(3);
    session.attach_takeoff("drone").unwrap();
    let q0 = session.world().body("drone").unwrap().quat;
    let filter_valid = session.update_nav_from_plant("drone", 0.02).unwrap();
    assert!(
        !filter_valid,
        "first plant sample is warm-up, not yet valid"
    );
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert!(aerial.estimator_valid);
    assert!(!aerial.failsafe);
    assert_eq!(session.world().body("drone").unwrap().quat, q0);
    assert!(session.world().all_hold());
}

#[test]
fn nav_update_does_not_write_plant_quaternion() {
    let session = WorldSession::inland(3);
    session.attach_takeoff("drone").unwrap();
    let q0 = session.world().body("drone").unwrap().quat;
    for _ in 0..20 {
        session.update_nav_from_plant("drone", 0.02).unwrap();
    }
    assert_eq!(session.world().body("drone").unwrap().quat, q0);
    assert!(
        session
            .world()
            .body("drone")
            .unwrap()
            .aerial
            .unwrap()
            .estimator_valid
    );
    assert!(session.world().all_hold());
}

#[test]
fn unusable_imu_trips_failsafe_and_still_holds() {
    let session = WorldSession::inland(3);
    session.attach_takeoff("drone").unwrap();
    let q0 = session.world().body("drone").unwrap().quat;
    let sample = dead_imu_sample();
    assert!(!sample.is_usable());
    assert!(!session.update_nav("drone", sample, 0.02).unwrap());
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert!(!aerial.estimator_valid);
    assert!(aerial.failsafe);
    assert_eq!(session.world().body("drone").unwrap().quat, q0);
    session.step(0.02).unwrap();
    assert!(session.world().all_hold());
    assert!(session
        .world()
        .last_properties
        .iter()
        .any(|p| p.id == "unit_attitude" && p.holds));
    assert_eq!(
        session
            .update_nav("rover", dead_imu_sample(), 0.02)
            .unwrap_err(),
        BackendError::Protocol
    );
}

#[test]
fn attach_airborne_failsafe_reset_and_recover_walk_the_machines() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_airborne("drone").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_failsafe("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Failsafe
    );
    assert_eq!(
        session.attach_failsafe("drone").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_recover_ready("drone").unwrap();
    session.attach_takeoff("drone").unwrap();
    session.attach_airborne("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Airborne
    );
    let VehicleHandle::Airborne(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_airborne must bind Airborne");
    };
    session.attach_failsafe("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Failsafe);
    assert!(aerial.failsafe && !aerial.offboard);
    assert!(session.world().body("drone").unwrap().command.is_none());
    let VehicleHandle::Failsafe(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_failsafe must bind Failsafe");
    };
    assert_eq!(
        session.attach_failsafe("drone").unwrap_err(),
        BackendError::Protocol
    );

    session.attach_drive("rover").unwrap();
    session.attach_estop("rover").unwrap();
    session.attach_reset("rover").unwrap();
    let g = session.world().body("rover").unwrap().ground.unwrap();
    assert_eq!(g.phase, GroundPhase::Parked);
    assert!(!g.estop && !g.drive_enabled);
    let GroundHandle::Parked(_) = session.ground("rover").attach().unwrap() else {
        panic!("attach_reset must bind Parked");
    };
    assert_eq!(
        session.attach_reset("rover").unwrap_err(),
        BackendError::Protocol
    );

    let session = WorldSession::coastal(2);
    session.attach_undock("skiff").unwrap();
    session.attach_marine_failsafe("skiff").unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Failsafe);
    assert!(m.failsafe && !m.thrust_enabled);
    let MarineHandle::Failsafe(_) = session.marine("skiff").attach().unwrap() else {
        panic!("attach_marine_failsafe must bind Failsafe");
    };
    session.attach_recover("skiff").unwrap();
    let m = session.world().body("skiff").unwrap().marine.unwrap();
    assert_eq!(m.phase, MarinePhase::Docked);
    assert!(!m.failsafe && !m.thrust_enabled);
    let MarineHandle::Docked(_) = session.marine("skiff").attach().unwrap() else {
        panic!("attach_recover must bind Docked");
    };
    assert_eq!(
        session.attach_recover("skiff").unwrap_err(),
        BackendError::Protocol
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_disarm_walks_offboard_to_ready() {
    let session = WorldSession::inland(1);
    session.attach_offboard("drone").unwrap();
    assert!(session.world().body("drone").unwrap().aerial.unwrap().armed);
    session.attach_disarm("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Ready);
    assert!(!aerial.armed && !aerial.actuators_enabled && !aerial.offboard);
    assert!(session.world().body("drone").unwrap().command.is_none());
    let VehicleHandle::PreflightReady(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_disarm must bind Ready");
    };
    session.attach_disarm("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Ready
    );
    session.attach_takeoff("drone").unwrap();
    session.attach_disarm("drone").unwrap();
    assert_eq!(
        session.world().body("drone").unwrap().aerial.unwrap().phase,
        Phase::Ready
    );
    assert!(session.world().all_hold());
}

#[test]
fn attach_recover_ready_walks_failsafe_through_recovery_to_ready() {
    let session = WorldSession::inland(1);
    assert_eq!(
        session.attach_recover_ready("drone").unwrap_err(),
        BackendError::Protocol
    );
    session.attach_takeoff("drone").unwrap();
    session.attach_failsafe("drone").unwrap();
    session.attach_recover_ready("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Ready);
    assert!(!aerial.failsafe && !aerial.armed && !aerial.actuators_enabled);
    assert!(session.world().body("drone").unwrap().command.is_none());
    let VehicleHandle::PreflightReady(_) = session.aerial("drone").attach().unwrap() else {
        panic!("attach_recover_ready must bind Ready");
    };
    assert_eq!(
        session.attach_recover_ready("drone").unwrap_err(),
        BackendError::Protocol
    );

    let session = WorldSession::inland(2);
    session.attach_takeoff("drone").unwrap();
    session.attach_failsafe("drone").unwrap();
    let VehicleHandle::Failsafe(fs) = session.aerial("drone").attach().unwrap() else {
        panic!("failsafe maps to Failsafe");
    };
    let recovering = fs.disarm_now().unwrap();
    assert_eq!(recovering.phase(), Phase::Recovery);
    assert!(recovering.safety().failsafe);
    let _ = recovering.into_backend();
    let VehicleHandle::Recovery(_) = session.aerial("drone").attach().unwrap() else {
        panic!("Disarm from Failsafe must bind Recovery, not Failsafe");
    };
    session.attach_recover_ready("drone").unwrap();
    let aerial = session.world().body("drone").unwrap().aerial.unwrap();
    assert_eq!(aerial.phase, Phase::Ready);
    assert!(!aerial.failsafe);
    assert!(session.world().all_hold());
}
