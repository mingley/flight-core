use super::support::robot;
use super::*;
use crate::{AerialKind, GroundKind, Lab, LabCmd, MarineKind};

#[test]
fn rover_probe_bounces_then_drives() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let n0 = lab
        .observe()
        .robots
        .iter()
        .find(|r| r.id == "rover")
        .unwrap()
        .n;
    let mut agent = RoverProbe::default();
    let run = lab.research(&mut agent, 0.02, 80);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(run.actions_rejected >= 1, "parked drive must bounce");
    assert_eq!(run.rejects.len(), run.actions_rejected);
    assert!(run.rejects.iter().any(|t| t.cmd == "drive"
        && t.code == "not_legal"
        && t.from_kind.as_deref() == Some("parked")));
    assert!(run.actions_applied >= 2, "release + drive");
    assert!(run.holds("no_terrain_penetration"));
    assert!(run.holds("ground_drive_only_on_contact"));
    assert!(run.holds("no_body_interpenetration"));
    assert!(
        run.properties.len() >= 21,
        "certificate {}",
        run.properties.len()
    );
    let n1 = lab
        .observe()
        .robots
        .iter()
        .find(|r| r.id == "rover")
        .unwrap()
        .n;
    assert!(n1 < n0 - 0.2, "south drive n {n0} → {n1}");
}

#[test]
fn scripted_coastal_agent_holds() {
    let mut lab = Lab::coastal(7);
    let mut agent = ScriptedCoastal;
    let run = lab.research(&mut agent, 0.02, 400);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert_eq!(run.actions_applied, 0);
    assert_eq!(run.actions_rejected, 0);
    assert!(
        run.properties.len() >= 21,
        "certificate {}",
        run.properties.len()
    );
    let obs = lab.observe();
    let drone = robot(&obs, "drone").unwrap();
    assert!(drone.alt > 0.5 || drone.aerial.as_ref().unwrap().kind == AerialKind::PreflightReady);
    assert_eq!(
        robot(&obs, "rover").unwrap().ground.as_ref().unwrap().kind,
        GroundKind::Moving
    );
    assert_eq!(
        robot(&obs, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::Underway
    );
}

#[test]
fn scripted_coastal_agent_holds_on_every_scenario() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = ScriptedCoastal;
        let run = lab.research(&mut agent, 0.02, 160);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
    }
}

#[test]
fn coastal_fleet_probes_then_moves() {
    let mut lab = Lab::coastal(3);
    let start = lab.observe();
    let rover0 = robot(&start, "rover").unwrap();
    let skiff0 = robot(&start, "skiff").unwrap();
    let surveyor0 = robot(&start, "surveyor").unwrap();
    let drone0 = robot(&start, "drone").unwrap();
    assert!(!rover0.ground.as_ref().unwrap().drive_enabled);
    assert!(!skiff0.marine.as_ref().unwrap().thrust_enabled);
    assert!(!drone0.aerial.as_ref().unwrap().armed);

    let mut agent = CoastalFleet::default();
    let run = lab.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(
        run.actions_rejected >= 3,
        "parked drive, docked thrust, disarmed velocity: {}",
        run.actions_rejected
    );
    assert!(
        run.actions_applied >= 10,
        "grants + motion, applied={}",
        run.actions_applied
    );

    let end = lab.observe();
    let rover = robot(&end, "rover").unwrap();
    let skiff = robot(&end, "skiff").unwrap();
    let surveyor = robot(&end, "surveyor").unwrap();
    let drone = robot(&end, "drone").unwrap();
    assert!(rover.ground.as_ref().unwrap().drive_enabled);
    assert!(skiff.marine.as_ref().unwrap().thrust_enabled);
    assert!(surveyor.marine.as_ref().unwrap().thrust_enabled);
    assert!(drone.aerial.as_ref().unwrap().actuators_enabled);
    assert!(
        rover.n < rover0.n - 0.2,
        "rover south {} → {}",
        rover0.n,
        rover.n
    );
    assert!(
        skiff.e > skiff0.e + 0.15,
        "skiff east {} → {}",
        skiff0.e,
        skiff.e
    );
    assert!(
        (surveyor.n - surveyor0.n).abs() > 0.1,
        "surveyor n {} → {}",
        surveyor0.n,
        surveyor.n
    );
    assert!(
        drone.alt > drone0.alt + 0.5,
        "drone alt {} → {}",
        drone0.alt,
        drone.alt
    );
}

#[test]
fn coastal_fleet_grants_the_scene_in_one_tick() {
    let mut lab = Lab::coastal(3);
    let mut agent = CoastalFleet::default();
    let run = lab.research(&mut agent, 0.02, 3);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(run.actions_rejected >= 3);
    assert!(run.actions_applied >= 7, "release + undocks + drone chain");
    let end = lab.observe();
    assert!(
        robot(&end, "rover")
            .unwrap()
            .ground
            .as_ref()
            .unwrap()
            .drive_enabled
    );
    assert!(
        robot(&end, "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .thrust_enabled
    );
    assert!(
        robot(&end, "surveyor")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .thrust_enabled
    );
    assert!(
        robot(&end, "drone")
            .unwrap()
            .aerial
            .as_ref()
            .unwrap()
            .actuators_enabled
    );
    assert_eq!(
        robot(&end, "drone").unwrap().aerial.as_ref().unwrap().kind,
        AerialKind::Takeoff
    );
    assert_eq!(
        robot(&end, "rover").unwrap().ground.as_ref().unwrap().kind,
        GroundKind::Moving
    );
    assert_eq!(
        robot(&end, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::Underway
    );
}

#[test]
fn coastal_fleet_skips_absent_hulls() {
    let mut inland = Lab::open("inland", 3).unwrap();
    let mut agent = CoastalFleet::default();
    let run = inland.research(&mut agent, 0.02, 120);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(
        run.actions_rejected >= 2,
        "parked drive + disarmed velocity: {}",
        run.actions_rejected
    );
    assert!(inland.observe().robots.iter().all(|r| r.id != "skiff"));

    let mut water = Lab::open("open_water", 3).unwrap();
    let mut agent = CoastalFleet::default();
    let run = water.research(&mut agent, 0.02, 160);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(
        run.actions_rejected >= 3,
        "two docked hulls + disarmed drone: {}",
        run.actions_rejected
    );
    assert!(water.observe().robots.iter().all(|r| r.id != "rover"));
}

#[test]
fn coastal_fleet_holds_on_every_scenario() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = CoastalFleet::default();
        let run = lab.research(&mut agent, 0.02, 160);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
    }
}

#[test]
fn typed_fleet_probes_json_then_moves_on_handles() {
    let mut lab = Lab::coastal(3);
    let start = lab.observe();
    let rover0 = robot(&start, "rover").unwrap().n;
    let skiff0 = robot(&start, "skiff").unwrap().e;
    let alt0 = robot(&start, "drone").unwrap().alt;
    let mut agent = TypedFleet::default();
    let run = lab.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(run.actions_rejected >= 3, "illegal JSON probes");
    assert_eq!(
        run.actions_applied, 0,
        "legal grants and motion must use typestate handles, not Lab::act"
    );
    assert!(
        lab.log.iter().any(|a| a.action.cmd == LabCmd::Release),
        "typestate enable_drive must record a replayable release"
    );
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Drive));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff));
    let end = lab.observe();
    let rover = robot(&end, "rover").unwrap();
    let skiff = robot(&end, "skiff").unwrap();
    let surveyor = robot(&end, "surveyor").unwrap();
    let drone = robot(&end, "drone").unwrap();
    assert!(rover.ground.as_ref().unwrap().drive_enabled);
    assert!(skiff.marine.as_ref().unwrap().thrust_enabled);
    assert!(surveyor.marine.as_ref().unwrap().thrust_enabled);
    assert!(drone.aerial.as_ref().unwrap().actuators_enabled);
    assert!(rover.n < rover0 - 0.2, "rover {} → {}", rover0, rover.n);
    assert!(skiff.e > skiff0 + 0.15, "skiff {} → {}", skiff0, skiff.e);
    assert!(drone.alt > alt0 + 0.5, "alt {} → {}", alt0, drone.alt);
}

#[test]
fn typed_fleet_holds_on_every_scenario() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedFleet::default();
        let run = lab.research(&mut agent, 0.02, 160);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
    }
}

#[test]
fn typed_fleet_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedFleet::default();
    let run = live.research(&mut agent, 0.02, 80);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(
        live.log.len() > 8,
        "grants + motion, len={}",
        live.log.len()
    );

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    for id in ["drone", "rover", "skiff", "surveyor"] {
        let a = live.world().body(id).unwrap().position_m;
        let b = replayed.world().body(id).unwrap().position_m;
        for (u, v) in a.iter().zip(b.iter()) {
            assert!((u - v).abs() < 1e-3, "{id} live={a:?} replay={b:?}");
        }
    }
    assert!(replayed.all_hold());
}

#[test]
fn typed_attach_fleet_probes_json_then_moves_on_handles() {
    let mut lab = Lab::coastal(3);
    let start = lab.observe();
    let rover0 = robot(&start, "rover").unwrap().n;
    let skiff0 = robot(&start, "skiff").unwrap().e;
    let alt0 = robot(&start, "drone").unwrap().alt;
    let mut agent = TypedAttachFleet::default();
    let run = lab.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(run.actions_rejected >= 3, "illegal JSON probes");
    assert_eq!(
        run.actions_applied, 0,
        "legal grants and motion must attach consume-self typestate, not Lab::act"
    );
    assert!(
        lab.log.iter().any(|a| a.action.cmd == LabCmd::Release),
        "enable_drive must record a replayable release"
    );
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Drive));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff));
    let end = lab.observe();
    let rover = robot(&end, "rover").unwrap();
    let skiff = robot(&end, "skiff").unwrap();
    let surveyor = robot(&end, "surveyor").unwrap();
    let drone = robot(&end, "drone").unwrap();
    assert!(rover.ground.as_ref().unwrap().drive_enabled);
    assert!(skiff.marine.as_ref().unwrap().thrust_enabled);
    assert!(surveyor.marine.as_ref().unwrap().thrust_enabled);
    assert!(drone.aerial.as_ref().unwrap().actuators_enabled);
    assert!(rover.n < rover0 - 0.2, "rover {} → {}", rover0, rover.n);
    assert!(skiff.e > skiff0 + 0.15, "skiff {} → {}", skiff0, skiff.e);
    assert!(drone.alt > alt0 + 0.5, "alt {} → {}", alt0, drone.alt);
}

#[test]
fn typed_attach_fleet_holds_on_every_scenario() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedAttachFleet::default();
        let run = lab.research(&mut agent, 0.02, 160);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
    }
}

#[test]
fn typed_attach_fleet_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedAttachFleet::default();
    let run = live.research(&mut agent, 0.02, 80);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(
        live.log.len() > 8,
        "grants + motion, len={}",
        live.log.len()
    );

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    for id in ["drone", "rover", "skiff", "surveyor"] {
        let a = live.world().body(id).unwrap().position_m;
        let b = replayed.world().body(id).unwrap().position_m;
        for (u, v) in a.iter().zip(b.iter()) {
            assert!((u - v).abs() < 1e-3, "{id} live={a:?} replay={b:?}");
        }
    }
    assert!(replayed.all_hold());
}

#[test]
fn pad_landing_leaves_then_returns() {
    for name in ["inland", "coastal"] {
        let mut lab = Lab::open(name, 3).unwrap();
        assert!(robot(&lab.observe(), "drone").unwrap().terrain_contact);
        let mut agent = PadLanding::default();
        let run = lab.research(&mut agent, 0.02, 400);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert!(agent.saw_pad, "{name}");
        assert!(agent.left_pad, "{name}");
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        assert!(
            drone.terrain_contact,
            "{name} still airborne alt={}",
            drone.alt
        );
        assert_eq!(
            drone.aerial.as_ref().unwrap().kind,
            AerialKind::PreflightReady
        );
        assert_eq!(drone.aerial.as_ref().unwrap().phase, "ready");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Land),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Touchdown),
            "{name}"
        );
    }
}

#[test]
fn pad_landing_skips_open_water() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = PadLanding::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.saw_pad);
    assert!(!agent.left_pad);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_pad_landing_leaves_then_returns() {
    for name in ["inland", "coastal"] {
        let mut lab = Lab::open(name, 3).unwrap();
        assert!(robot(&lab.observe(), "drone").unwrap().terrain_contact);
        let mut agent = TypedPadLanding::default();
        let run = lab.research(&mut agent, 0.02, 400);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(
            run.actions_applied, 0,
            "{name} legal landings must use typestate handles"
        );
        assert!(run.actions_rejected >= 1, "{name} disarmed velocity probe");
        assert!(agent.saw_pad, "{name}");
        assert!(agent.left_pad, "{name}");
        assert!(run.holds("no_terrain_penetration"));
        assert!(run.holds("aerial_thrust_only_in_air"));
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        assert!(
            drone.terrain_contact,
            "{name} still airborne alt={}",
            drone.alt
        );
        assert_eq!(
            drone.aerial.as_ref().unwrap().kind,
            AerialKind::PreflightReady
        );
        assert_eq!(drone.aerial.as_ref().unwrap().phase, "ready");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Airborne),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Land),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Touchdown),
            "{name}"
        );
    }
}

#[test]
fn typed_pad_landing_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedPadLanding::default();
    let run = live.research(&mut agent, 0.02, 400);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.left_pad);
    assert!(live.log.iter().any(|a| a.action.cmd == LabCmd::Airborne));
    assert!(live.log.iter().any(|a| a.action.cmd == LabCmd::Land));
    assert!(live.log.iter().any(|a| a.action.cmd == LabCmd::Touchdown));

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let a = live.world().body("drone").unwrap().position_m;
    let b = replayed.world().body("drone").unwrap().position_m;
    for (u, v) in a.iter().zip(b.iter()) {
        assert!((u - v).abs() < 1e-3, "live={a:?} replay={b:?}");
    }
    assert_eq!(
        robot(&replayed.observe(), "drone")
            .unwrap()
            .aerial
            .as_ref()
            .unwrap()
            .kind,
        AerialKind::PreflightReady
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_pad_landing_skips_open_water() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedPadLanding::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.saw_pad);
    assert!(!agent.left_pad);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn collision_sweep_hits_and_holds() {
    let mut lab = Lab::open("inland", 3).unwrap();
    lab.with_world_mut(|w| {
        w.body_mut("rover").unwrap().position_m = [6.0, 1.05, 0.0];
        w.body_mut("drone").unwrap().position_m = [6.0, 0.0, 0.0];
    });
    let mut agent = CollisionSweep::default();
    let run = lab.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.hit, "rover never registered sphere_contact");
    assert!(run.actions_rejected >= 1);
    assert!(run.holds("no_body_interpenetration"));
    let end = lab.observe();
    assert!(end.all_hold);
    assert!(end
        .properties
        .iter()
        .any(|p| p.id == "no_body_interpenetration" && p.holds));
}

fn close_rover_on_drone(lab: &Lab) {
    lab.with_world_mut(|w| {
        w.body_mut("rover").unwrap().position_m = [6.0, 1.05, 0.0];
        w.body_mut("drone").unwrap().position_m = [6.0, 0.0, 0.0];
    });
}

#[test]
fn typed_collision_sweep_hits_and_holds() {
    let mut lab = Lab::open("inland", 3).unwrap();
    close_rover_on_drone(&lab);
    let mut agent = TypedCollisionSweep::default();
    let run = lab.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert_eq!(run.actions_applied, 0);
    assert!(run.actions_rejected >= 1);
    assert!(agent.hit, "rover never registered a sphere hit");
    assert!(run.holds("no_body_interpenetration"));
    assert!(run.holds("no_terrain_penetration"));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Release));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Drive));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Halt));
    let end = lab.observe();
    assert!(
        !end.robots
            .iter()
            .find(|r| r.id == "rover")
            .unwrap()
            .ground
            .as_ref()
            .unwrap()
            .drive_enabled
    );
}

#[test]
fn typed_collision_sweep_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    close_rover_on_drone(&live);
    let mut agent = TypedCollisionSweep::default();
    let run = live.research(&mut agent, 0.02, 200);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.hit);

    let mut replayed = Lab::open("inland", 3).unwrap();
    close_rover_on_drone(&replayed);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    for id in ["drone", "rover"] {
        let a = live.world().body(id).unwrap().position_m;
        let b = replayed.world().body(id).unwrap().position_m;
        for (u, v) in a.iter().zip(b.iter()) {
            assert!((u - v).abs() < 1e-3, "{id} live={a:?} replay={b:?}");
        }
    }
    assert!(replayed.all_hold());
}

#[test]
fn typed_station_dock_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let e0 = robot(&lab.observe(), "skiff").unwrap().e;
        let mut agent = TypedStationDock::default();
        let run = lab.research(&mut agent, 0.02, 240);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(run.holds("marine_thrust_only_when_wet"));
        assert!(run.holds("marine_thrust_requires_grant"));
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        let end = lab.observe();
        let skiff = robot(&end, "skiff").unwrap();
        assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert_eq!(skiff.marine.as_ref().unwrap().phase, "docked");
        assert!(!skiff.marine.as_ref().unwrap().thrust_enabled);
        assert!(
            skiff.e > e0 + 0.1,
            "{name} skiff did not make way {} → {}",
            e0,
            skiff.e
        );
    }
}

#[test]
fn typed_station_dock_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedStationDock::default();
    let run = live.research(&mut agent, 0.02, 240);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let a = live.world().body("skiff").unwrap().position_m;
    let b = replayed.world().body("skiff").unwrap().position_m;
    for (u, v) in a.iter().zip(b.iter()) {
        assert!((u - v).abs() < 1e-3, "live={a:?} replay={b:?}");
    }
    assert_eq!(
        robot(&replayed.observe(), "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .phase,
        "docked"
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_station_dock_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedStationDock::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "skiff"));
}

#[test]
fn typed_hull_dock_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let e0 = robot(&lab.observe(), "skiff").unwrap().e;
        let mut agent = TypedHullDock::default();
        let run = lab.research(&mut agent, 0.02, 240);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(run.holds("marine_thrust_only_when_wet"));
        assert!(run.holds("marine_thrust_requires_grant"));
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Station
                && a.action.cmd != LabCmd::Resume
                && a.action.cmd != LabCmd::Failsafe),
            "{name} underway dock must not station, resume, or failsafe"
        );
        let end = lab.observe();
        let skiff = robot(&end, "skiff").unwrap();
        assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert_eq!(skiff.marine.as_ref().unwrap().phase, "docked");
        assert!(!skiff.marine.as_ref().unwrap().thrust_enabled);
        assert!(
            skiff.e > e0 + 0.1,
            "{name} skiff did not make way {} → {}",
            e0,
            skiff.e
        );
    }
}

#[test]
fn typed_hull_dock_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedHullDock::default();
    let run = live.research(&mut agent, 0.02, 240);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let a = live.world().body("skiff").unwrap().position_m;
    let b = replayed.world().body("skiff").unwrap().position_m;
    for (u, v) in a.iter().zip(b.iter()) {
        assert!((u - v).abs() < 1e-3, "live={a:?} replay={b:?}");
    }
    assert_eq!(
        robot(&replayed.observe(), "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .phase,
        "docked"
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_hull_dock_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedHullDock::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "skiff"));
}

#[test]
fn typed_station_resume_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedStationResume::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Resume),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Dock),
            "{name} resume must not dock"
        );
        let end = lab.observe();
        let skiff = robot(&end, "skiff").unwrap();
        assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Underway);
        assert!(skiff.marine.as_ref().unwrap().thrust_enabled);
    }
}

#[test]
fn typed_station_resume_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedStationResume::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    assert_eq!(
        robot(&replayed.observe(), "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .kind,
        MarineKind::Underway
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_station_resume_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedStationResume::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "skiff"));
}

#[test]
fn typed_hull_failsafe_recovers_docked() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedHullFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(run.holds("marine_thrust_requires_grant"));
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let skiff = robot(&end, "skiff").unwrap();
        assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert!(!skiff.marine.as_ref().unwrap().thrust_enabled);
        assert!(!skiff.marine.as_ref().unwrap().failsafe);
    }
}

#[test]
fn typed_hull_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedHullFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    assert_eq!(
        robot(&replayed.observe(), "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .kind,
        MarineKind::Docked
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_hull_failsafe_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedHullFailsafe::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_aerial_failsafe_recovers_ready() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedAerialFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Disarm),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::PreflightReady, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(!a.armed, "{name}");
    }
}

#[test]
fn typed_aerial_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedAerialFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::PreflightReady);
    assert!(!a.failsafe);
    assert!(replayed.all_hold());
}

#[test]
fn typed_aerial_disarm_returns_ready_without_failsafe() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedAerialDisarm::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Disarm),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .all(|a| a.action.cmd != LabCmd::Failsafe && a.action.cmd != LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::PreflightReady, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(!a.armed, "{name}");
    }
}

#[test]
fn typed_aerial_disarm_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedAerialDisarm::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::PreflightReady);
    assert!(!a.armed);
    assert!(replayed.all_hold());
}

#[test]
fn typed_aerial_airborne_lands_from_airborne() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedAerialAirborne::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Airborne),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Land),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Disarm
                && a.action.cmd != LabCmd::Touchdown),
            "{name} airborne land must not failsafe, recover, disarm, or touchdown"
        );
        let end = lab.observe();
        let a = robot(&end, "drone").unwrap().aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::Landing, "{name}");
        assert!(a.armed, "{name}");
        assert!(!a.failsafe, "{name}");
    }
}

#[test]
fn typed_aerial_airborne_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedAerialAirborne::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::Landing);
    assert!(a.armed);
    assert!(replayed.all_hold());
}

#[test]
fn typed_position_hold_takes_off_then_holds() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedPositionHold::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Position),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Disarm
                && a.action.cmd != LabCmd::Land
                && a.action.cmd != LabCmd::Airborne
                && a.action.cmd != LabCmd::Touchdown),
            "{name} position hold must not land, airborne, failsafe, or disarm"
        );
        let end = lab.observe();
        let a = robot(&end, "drone").unwrap().aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::Takeoff, "{name}");
        assert!(a.armed, "{name}");
        assert!(!a.failsafe, "{name}");
    }
}

#[test]
fn typed_position_hold_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedPositionHold::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::Takeoff);
    assert!(a.armed);
    assert!(replayed.all_hold());
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn typed_hold_takes_off_then_holds_current_pose() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedHold::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Disarm
                && a.action.cmd != LabCmd::Land
                && a.action.cmd != LabCmd::Airborne
                && a.action.cmd != LabCmd::Touchdown
                && a.action.cmd != LabCmd::Position),
            "{name} current-pose hold must not name a position or land"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::Takeoff, "{name}");
        assert!(a.armed, "{name}");
        assert!(!a.failsafe, "{name}");
        let hold = drone.hold_ned.unwrap_or_else(|| panic!("{name} hold_ned"));
        assert!(
            (hold[2] + 2.0).abs() > 0.5,
            "{name} current-pose hold must not be d=−2, got {hold:?}"
        );
    }
}

#[test]
fn typed_hold_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedHold::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let drone = robot(&obs, "drone").unwrap();
    let a = drone.aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::Takeoff);
    assert!(a.armed);
    assert!(drone.hold_ned.is_some());
    assert!(replayed.all_hold());
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn typed_pad_disarm_returns_ready_without_takeoff() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedPadDisarm::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Disarm),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Takeoff
                && a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Arm),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::PreflightReady, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(!a.armed, "{name}");
    }
}

#[test]
fn typed_pad_disarm_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedPadDisarm::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::PreflightReady);
    assert!(!a.armed);
    assert!(replayed.all_hold());
}

#[test]
fn typed_pad_failsafe_recovers_ready_without_takeoff() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedPadFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Disarm),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .all(|a| a.action.cmd != LabCmd::Takeoff && a.action.cmd != LabCmd::Arm),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::PreflightReady, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(!a.armed, "{name}");
    }
}

#[test]
fn typed_pad_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedPadFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::PreflightReady);
    assert!(!a.failsafe);
    assert!(!a.armed);
    assert!(replayed.all_hold());
}

#[test]
fn typed_ground_estop_clears_parked_without_drive() {
    for name in ["inland", "coastal", "harbor"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedGroundEstop::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Estop),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Clear),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .all(|a| a.action.cmd != LabCmd::Release && a.action.cmd != LabCmd::Drive),
            "{name}"
        );
        let end = lab.observe();
        let rover = robot(&end, "rover").unwrap();
        let g = rover.ground.as_ref().unwrap();
        assert_eq!(g.kind, GroundKind::Parked, "{name}");
        assert!(!g.estop, "{name}");
        assert!(!g.drive_enabled, "{name}");
    }
}

#[test]
fn typed_ground_estop_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedGroundEstop::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let g = robot(&obs, "rover").unwrap().ground.as_ref().unwrap();
    assert_eq!(g.kind, GroundKind::Parked);
    assert!(!g.estop);
    assert!(!g.drive_enabled);
    assert!(replayed.all_hold());
}

#[test]
fn typed_ground_estop_skips_open_water() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedGroundEstop::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_ground_halt_parks_without_estop() {
    for name in ["inland", "coastal", "harbor"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedGroundHalt::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Release),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Halt),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .all(|a| a.action.cmd != LabCmd::Estop && a.action.cmd != LabCmd::Clear),
            "{name}"
        );
        let end = lab.observe();
        let rover = robot(&end, "rover").unwrap();
        let g = rover.ground.as_ref().unwrap();
        assert_eq!(g.kind, GroundKind::Parked, "{name}");
        assert!(!g.estop, "{name}");
        assert!(!g.drive_enabled, "{name}");
    }
}

#[test]
fn typed_ground_halt_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedGroundHalt::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let g = robot(&obs, "rover").unwrap().ground.as_ref().unwrap();
    assert_eq!(g.kind, GroundKind::Parked);
    assert!(!g.drive_enabled);
    assert!(!g.estop);
    assert!(replayed.all_hold());
}

#[test]
fn typed_ground_halt_skips_open_water() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedGroundHalt::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_ground_hold_holds_current_pose() {
    for name in ["inland", "coastal", "harbor"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedGroundHold::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Release),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .all(|a| a.action.cmd != LabCmd::Estop && a.action.cmd != LabCmd::Halt),
            "{name}"
        );
        let end = lab.observe();
        let rover = robot(&end, "rover").unwrap();
        let g = rover.ground.as_ref().unwrap();
        assert_eq!(g.kind, GroundKind::Moving, "{name}");
        assert!(g.drive_enabled, "{name}");
        let hold = rover.hold_ned.unwrap_or_else(|| panic!("{name} hold_ned"));
        assert!(hold.iter().all(|c| c.is_finite()), "{name} hold {hold:?}");
        assert!(run.holds("position_hold_restores_pose"), "{name}");
    }
}

#[test]
fn typed_ground_hold_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedGroundHold::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let rover = robot(&obs, "rover").unwrap();
    let g = rover.ground.as_ref().unwrap();
    assert_eq!(g.kind, GroundKind::Moving);
    assert!(rover.hold_ned.is_some());
    assert!(replayed.all_hold());
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn typed_ground_hold_skips_open_water() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedGroundHold::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_fleet_return_homes_coastal_bodies() {
    for name in ["coastal", "harbor"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedFleetReturn::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Land),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Touchdown),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Halt),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .any(|a| a.action.robot == "skiff" && a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .any(|a| a.action.robot == "surveyor" && a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Estop
                && a.action.cmd != LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap().aerial.as_ref().unwrap();
        assert_eq!(drone.kind, AerialKind::PreflightReady, "{name}");
        assert!(!drone.armed, "{name}");
        let rover = robot(&end, "rover").unwrap().ground.as_ref().unwrap();
        assert_eq!(rover.kind, GroundKind::Parked, "{name}");
        assert!(!rover.drive_enabled, "{name}");
        let skiff = robot(&end, "skiff").unwrap().marine.as_ref().unwrap();
        assert_eq!(skiff.kind, MarineKind::Docked, "{name}");
        assert!(!skiff.thrust_enabled, "{name}");
        let surveyor = robot(&end, "surveyor").unwrap().marine.as_ref().unwrap();
        assert_eq!(surveyor.kind, MarineKind::Docked, "{name}");
    }
}

#[test]
fn typed_fleet_return_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("coastal", 3).unwrap();
    let mut agent = TypedFleetReturn::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("coastal", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    assert_eq!(
        robot(&obs, "drone").unwrap().aerial.as_ref().unwrap().kind,
        AerialKind::PreflightReady
    );
    assert_eq!(
        robot(&obs, "rover").unwrap().ground.as_ref().unwrap().kind,
        GroundKind::Parked
    );
    assert_eq!(
        robot(&obs, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::Docked
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_fleet_return_inland_has_no_hull_dock() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedFleetReturn::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Land));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Halt));
    assert!(lab.log.iter().all(|a| a.action.cmd != LabCmd::Dock));
    let end = lab.observe();
    assert_eq!(
        robot(&end, "drone").unwrap().aerial.as_ref().unwrap().kind,
        AerialKind::PreflightReady
    );
    assert_eq!(
        robot(&end, "rover").unwrap().ground.as_ref().unwrap().kind,
        GroundKind::Parked
    );
    assert!(robot(&end, "skiff").is_none());
}

#[test]
fn typed_fleet_return_open_water_has_no_rover_halt() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedFleetReturn::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Land));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Dock));
    assert!(lab.log.iter().all(|a| a.action.cmd != LabCmd::Halt));
    let end = lab.observe();
    assert!(robot(&end, "rover").is_none());
    assert_eq!(
        robot(&end, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::Docked
    );
}

#[test]
fn typed_fleet_hold_holds_drone_and_stations_skiff() {
    for name in ["coastal", "harbor"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedFleetHold::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold),
            "{name}"
        );
        assert!(
            lab.log
                .iter()
                .any(|a| a.action.robot == "skiff" && a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Land
                && a.action.cmd != LabCmd::Position
                && a.action.cmd != LabCmd::Resume
                && a.action.cmd != LabCmd::Dock),
            "{name} fleet hold must not land, dock, or name a position"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert!(a.armed, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(drone.hold_ned.is_some(), "{name} hold_ned");
        let skiff = robot(&end, "skiff").unwrap().marine.as_ref().unwrap();
        assert_eq!(skiff.kind, MarineKind::StationKeep, "{name}");
        assert!(lab.all_hold(), "{name}");
    }
}

#[test]
fn typed_fleet_hold_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("coastal", 3).unwrap();
    let mut agent = TypedFleetHold::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("coastal", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    assert!(robot(&obs, "drone").unwrap().hold_ned.is_some());
    assert_eq!(
        robot(&obs, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::StationKeep
    );
    assert!(replayed.all_hold());
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn typed_fleet_hold_inland_has_no_hull_station() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedFleetHold::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold));
    assert!(lab.log.iter().all(|a| a.action.cmd != LabCmd::Station));
    let end = lab.observe();
    assert!(robot(&end, "drone").unwrap().hold_ned.is_some());
    assert!(robot(&end, "skiff").is_none());
}

#[test]
fn typed_fleet_hold_open_water_has_no_rover() {
    let mut lab = Lab::open("open_water", 3).unwrap();
    let mut agent = TypedFleetHold::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold));
    assert!(lab
        .log
        .iter()
        .any(|a| a.action.robot == "skiff" && a.action.cmd == LabCmd::Station));
    assert!(lab.log.iter().all(|a| a.action.cmd != LabCmd::Halt));
    let end = lab.observe();
    assert!(robot(&end, "rover").is_none());
    assert!(robot(&end, "drone").unwrap().hold_ned.is_some());
    assert_eq!(
        robot(&end, "skiff").unwrap().marine.as_ref().unwrap().kind,
        MarineKind::StationKeep
    );
}

#[test]
fn typed_station_failsafe_recovers_docked_from_station_keep() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedStationFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let skiff = robot(&end, "skiff").unwrap();
        assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert!(!skiff.marine.as_ref().unwrap().thrust_enabled);
        assert!(!skiff.marine.as_ref().unwrap().failsafe);
    }
}

#[test]
fn typed_station_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedStationFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let m = robot(&obs, "skiff").unwrap().marine.as_ref().unwrap();
    assert_eq!(m.kind, MarineKind::Docked);
    assert!(!m.failsafe);
    assert!(!m.thrust_enabled);
    assert!(replayed.all_hold());
}

#[test]
fn typed_station_failsafe_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedStationFailsafe::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_failsafe_touchdown_returns_ready_without_recover() {
    for name in ["inland", "coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedFailsafeTouchdown::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Touchdown),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Disarm
                && a.action.cmd != LabCmd::Takeoff),
            "{name}"
        );
        let end = lab.observe();
        let drone = robot(&end, "drone").unwrap();
        let a = drone.aerial.as_ref().unwrap();
        assert_eq!(a.kind, AerialKind::PreflightReady, "{name}");
        assert!(!a.failsafe, "{name}");
        assert!(!a.armed, "{name}");
    }
}

#[test]
fn typed_failsafe_touchdown_log_replays_on_a_fresh_lab() {
    let mut live = Lab::open("inland", 3).unwrap();
    let mut agent = TypedFailsafeTouchdown::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::open("inland", 3).unwrap();
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let a = robot(&obs, "drone").unwrap().aerial.as_ref().unwrap();
    assert_eq!(a.kind, AerialKind::PreflightReady);
    assert!(!a.failsafe);
    assert!(!a.armed);
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_failsafe_recovers_docked() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedSurveyorFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let surveyor = robot(&end, "surveyor").unwrap();
        assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert!(!surveyor.marine.as_ref().unwrap().thrust_enabled);
        assert!(!surveyor.marine.as_ref().unwrap().failsafe);
    }
}

#[test]
fn typed_surveyor_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedSurveyorFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let m = robot(&obs, "surveyor").unwrap().marine.as_ref().unwrap();
    assert_eq!(m.kind, MarineKind::Docked);
    assert!(!m.failsafe);
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_failsafe_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedSurveyorFailsafe::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_surveyor_station_failsafe_recovers_docked_from_station_keep() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedSurveyorStationFailsafe::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Failsafe),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover),
            "{name}"
        );
        let end = lab.observe();
        let surveyor = robot(&end, "surveyor").unwrap();
        assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert!(!surveyor.marine.as_ref().unwrap().thrust_enabled);
        assert!(!surveyor.marine.as_ref().unwrap().failsafe);
    }
}

#[test]
fn typed_surveyor_station_failsafe_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedSurveyorStationFailsafe::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let obs = replayed.observe();
    let m = robot(&obs, "surveyor").unwrap().marine.as_ref().unwrap();
    assert_eq!(m.kind, MarineKind::Docked);
    assert!(!m.failsafe);
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_station_failsafe_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedSurveyorStationFailsafe::default();
    let run = lab.research(&mut agent, 0.02, 20);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
}

#[test]
fn typed_surveyor_station_dock_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let n0 = robot(&lab.observe(), "surveyor").unwrap().n;
        let mut agent = TypedSurveyorStationDock::default();
        let run = lab.research(&mut agent, 0.02, 240);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(run.holds("marine_thrust_only_when_wet"));
        assert!(run.holds("marine_thrust_requires_grant"));
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover
                && a.action.cmd != LabCmd::Resume),
            "{name} station dock must not failsafe, recover, or resume"
        );
        let end = lab.observe();
        let surveyor = robot(&end, "surveyor").unwrap();
        assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert_eq!(surveyor.marine.as_ref().unwrap().phase, "docked");
        assert!(!surveyor.marine.as_ref().unwrap().thrust_enabled);
        assert!(
            (surveyor.n - n0).abs() > 0.1,
            "{name} surveyor did not make way {} → {}",
            n0,
            surveyor.n
        );
    }
}

#[test]
fn typed_surveyor_station_dock_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedSurveyorStationDock::default();
    let run = live.research(&mut agent, 0.02, 240);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let a = live.world().body("surveyor").unwrap().position_m;
    let b = replayed.world().body("surveyor").unwrap().position_m;
    for (u, v) in a.iter().zip(b.iter()) {
        assert!((u - v).abs() < 1e-3, "live={a:?} replay={b:?}");
    }
    assert_eq!(
        robot(&replayed.observe(), "surveyor")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .phase,
        "docked"
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_station_dock_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedSurveyorStationDock::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "surveyor"));
}

#[test]
fn typed_surveyor_dock_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let n0 = robot(&lab.observe(), "surveyor").unwrap().n;
        let mut agent = TypedSurveyorDock::default();
        let run = lab.research(&mut agent, 0.02, 240);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(run.holds("marine_thrust_only_when_wet"));
        assert!(run.holds("marine_thrust_requires_grant"));
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Dock),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Station
                && a.action.cmd != LabCmd::Resume
                && a.action.cmd != LabCmd::Failsafe),
            "{name} underway dock must not station, resume, or failsafe"
        );
        let end = lab.observe();
        let surveyor = robot(&end, "surveyor").unwrap();
        assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Docked);
        assert_eq!(surveyor.marine.as_ref().unwrap().phase, "docked");
        assert!(!surveyor.marine.as_ref().unwrap().thrust_enabled);
        assert!(
            (surveyor.n - n0).abs() > 0.1,
            "{name} surveyor did not make way {} → {}",
            n0,
            surveyor.n
        );
    }
}

#[test]
fn typed_surveyor_dock_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedSurveyorDock::default();
    let run = live.research(&mut agent, 0.02, 240);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    let a = live.world().body("surveyor").unwrap().position_m;
    let b = replayed.world().body("surveyor").unwrap().position_m;
    for (u, v) in a.iter().zip(b.iter()) {
        assert!((u - v).abs() < 1e-3, "live={a:?} replay={b:?}");
    }
    assert_eq!(
        robot(&replayed.observe(), "surveyor")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .phase,
        "docked"
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_dock_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedSurveyorDock::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "surveyor"));
}

#[test]
fn typed_surveyor_station_resume_holds_on_water_worlds() {
    for name in ["coastal", "harbor", "open_water"] {
        let mut lab = Lab::open(name, 3).unwrap();
        let mut agent = TypedSurveyorStationResume::default();
        let run = lab.research(&mut agent, 0.02, 40);
        assert!(run.ok(), "{name} {run} broken={:?}", run.broken);
        assert_eq!(run.actions_applied, 0, "{name}");
        assert!(run.actions_rejected >= 1, "{name}");
        assert!(agent.done, "{name}");
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Undock),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Station),
            "{name}"
        );
        assert!(
            lab.log.iter().any(|a| a.action.cmd == LabCmd::Resume),
            "{name}"
        );
        assert!(
            lab.log.iter().all(|a| a.action.cmd != LabCmd::Dock
                && a.action.cmd != LabCmd::Failsafe
                && a.action.cmd != LabCmd::Recover),
            "{name} resume must not dock, failsafe, or recover"
        );
        let end = lab.observe();
        let surveyor = robot(&end, "surveyor").unwrap();
        assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Underway);
        assert!(surveyor.marine.as_ref().unwrap().thrust_enabled);
    }
}

#[test]
fn typed_surveyor_station_resume_log_replays_on_a_fresh_lab() {
    let mut live = Lab::coastal(3);
    let mut agent = TypedSurveyorStationResume::default();
    let run = live.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(agent.done);

    let mut replayed = Lab::coastal(3);
    replayed
        .replay_until(&live.log, 0.02, live.world().t)
        .unwrap();
    assert_eq!(
        robot(&replayed.observe(), "surveyor")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .kind,
        MarineKind::Underway
    );
    assert!(replayed.all_hold());
}

#[test]
fn typed_surveyor_station_resume_skips_inland() {
    let mut lab = Lab::open("inland", 3).unwrap();
    let mut agent = TypedSurveyorStationResume::default();
    let run = lab.research(&mut agent, 0.02, 40);
    assert!(run.ok(), "{run} broken={:?}", run.broken);
    assert!(!agent.done);
    assert_eq!(run.actions_applied, 0);
    assert!(lab.observe().robots.iter().all(|r| r.id != "surveyor"));
}
