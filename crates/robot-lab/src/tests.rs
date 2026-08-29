use super::*;
use flight_core::vehicle::BackendError;
use robot_world::Body;

#[test]
fn parked_drive_is_rejected() {
    let mut lab = Lab::coastal(1);
    let err = lab
        .act(AgentAction {
            robot: "rover".into(),
            cmd: LabCmd::Drive,
            vn: -1.0,
            ve: 0.0,
            vd: 0.0,
            yaw_rate: 0.0,
        })
        .unwrap_err();
    assert!(matches!(err, LabError::Ground(_)));
}

#[test]
fn release_then_drive() {
    let mut lab = Lab::coastal(1);
    lab.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    lab.act(parse(
        r#"{"robot":"rover","cmd":"drive","vn":-0.4,"ve":0.0}"#,
    ))
    .unwrap();
    for _ in 0..80 {
        lab.step(0.02);
    }
    assert!(lab.all_hold());
    assert!(body(&lab, "rover").position_m[0] < 14.0);
}

#[test]
fn docked_thrust_is_rejected() {
    let mut lab = Lab::coastal(1);
    let err = lab
        .act(parse(r#"{"robot":"skiff","cmd":"thrust","vn":0.5}"#))
        .unwrap_err();
    assert!(matches!(err, LabError::Marine(_)));
}

#[test]
fn scripted_coastal_holds() {
    let mut lab = Lab::coastal(7);
    for _ in 0..900 {
        lab.apply_script();
        lab.step(0.02);
        assert!(
            lab.all_hold(),
            "t={} {:?}",
            lab.world().t,
            lab.world().last_properties
        );
    }
    let drone = body(&lab, "drone");
    assert!(drone.altitude_agl() > 0.5 || drone.phase_name() == "ready");
    let obs = lab.observe();
    assert_eq!(obs.robots.len(), 4);
    assert!(obs.properties.iter().all(|p| p.holds));
    assert!(obs
        .robots
        .iter()
        .all(|r| r.ke.is_finite() && r.pe.is_finite()));
}

#[test]
fn scripted_coastal_walks_attach_on_the_first_tick() {
    let mut lab = Lab::coastal(7);
    lab.apply_script();
    assert!(lab.log.is_empty(), "script velocity ticks are not logged");
    let obs = lab.observe();
    let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
    let skiff = obs.robots.iter().find(|r| r.id == "skiff").unwrap();
    let surveyor = obs.robots.iter().find(|r| r.id == "surveyor").unwrap();
    assert_eq!(drone.aerial.as_ref().unwrap().kind, AerialKind::Takeoff);
    assert_eq!(rover.ground.as_ref().unwrap().kind, GroundKind::Moving);
    assert_eq!(skiff.marine.as_ref().unwrap().kind, MarineKind::Underway);
    assert_eq!(surveyor.marine.as_ref().unwrap().kind, MarineKind::Underway);
}

#[test]
fn scripted_policy_leaves_failsafe_and_estop_alone() {
    let mut lab = Lab::coastal(1);
    lab.attach_takeoff("drone").unwrap();
    lab.attach_failsafe("drone").unwrap();
    lab.attach_drive("rover").unwrap();
    lab.attach_estop("rover").unwrap();
    lab.attach_undock("skiff").unwrap();
    lab.attach_marine_failsafe("skiff").unwrap();
    lab.apply_script();
    let obs = lab.observe();
    assert_eq!(
        obs.robots
            .iter()
            .find(|r| r.id == "drone")
            .unwrap()
            .aerial
            .as_ref()
            .unwrap()
            .kind,
        AerialKind::Failsafe
    );
    assert_eq!(
        obs.robots
            .iter()
            .find(|r| r.id == "rover")
            .unwrap()
            .ground
            .as_ref()
            .unwrap()
            .kind,
        GroundKind::EStopped
    );
    assert_eq!(
        obs.robots
            .iter()
            .find(|r| r.id == "skiff")
            .unwrap()
            .marine
            .as_ref()
            .unwrap()
            .kind,
        MarineKind::Failsafe
    );
}

#[test]
fn json_illegal_then_typestate_grants_move() {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    let mut lab = Lab::coastal(3);
    assert!(matches!(
        lab.act(parse(r#"{"robot":"rover","cmd":"drive","vn":-0.6}"#)),
        Err(LabError::Ground(_))
    ));
    assert!(matches!(
        lab.act(parse(r#"{"robot":"skiff","cmd":"thrust","vn":0.8}"#)),
        Err(LabError::Marine(_))
    ));
    assert!(matches!(
        lab.act(parse(r#"{"robot":"drone","cmd":"velocity","ve":1.0}"#)),
        Err(LabError::Aerial(_))
    ));
    assert!(lab
        .ground("rover")
        .set_velocity_now(Velocity::<Ned>::ned(-0.6, 0.0, 0.0))
        .is_err());
    assert!(lab
        .marine("skiff")
        .set_velocity_now(Velocity::<Ned>::ned(0.8, 0.0, 0.0))
        .is_err());
    assert!(lab
        .aerial("drone")
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 1.0, 0.0))
        .is_err());

    let mut drone = lab.attach_takeoff("drone").unwrap();
    let mut rover = lab.attach_drive("rover").unwrap();
    let mut skiff = lab.attach_undock("skiff").unwrap();

    let start = lab.observe();
    let rover0 = start.robots.iter().find(|r| r.id == "rover").unwrap().n;
    let skiff0 = start.robots.iter().find(|r| r.id == "skiff").unwrap().e;
    let alt0 = start.robots.iter().find(|r| r.id == "drone").unwrap().alt;

    for _ in 0..40 {
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
            .unwrap();
        drone.flush().unwrap();
        rover
            .set_velocity_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
            .unwrap();
        rover.flush().unwrap();
        skiff
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.6, 0.0))
            .unwrap();
        skiff.flush().unwrap();
        lab.session().step(0.02).unwrap();
    }

    let end = lab.observe();
    assert!(end.all_hold, "broken {:?}", end.properties);
    let rover = end.robots.iter().find(|r| r.id == "rover").unwrap();
    let skiff = end.robots.iter().find(|r| r.id == "skiff").unwrap();
    let drone = end.robots.iter().find(|r| r.id == "drone").unwrap();
    assert!(rover.ground.as_ref().unwrap().drive_enabled);
    assert!(skiff.marine.as_ref().unwrap().thrust_enabled);
    assert!(drone.aerial.as_ref().unwrap().actuators_enabled);
    assert!(rover.n < rover0 - 0.15, "rover {} → {}", rover0, rover.n);
    assert!(skiff.e > skiff0 + 0.08, "skiff {} → {}", skiff0, skiff.e);
    assert!(drone.alt > alt0 + 0.3, "alt {} → {}", alt0, drone.alt);
}

fn parse(s: &str) -> AgentAction {
    serde_json::from_str(s).unwrap()
}

#[test]
fn lab_cmd_json_roundtrips_and_rejects_unknown_names() {
    let a = parse(r#"{"robot":"rover","cmd":"drive","vn":-0.5}"#);
    assert_eq!(a.cmd, LabCmd::Drive);
    assert_eq!(a.cmd.as_str(), "drive");
    let json = serde_json::to_string(&a).unwrap();
    assert!(json.contains(r#""cmd":"drive""#));
    let hold = parse(r#"{"robot":"drone","cmd":"position","vn":0.0,"ve":0.0,"vd":-2.0}"#);
    assert_eq!(hold.cmd, LabCmd::Position);
    assert_eq!(hold.cmd.as_str(), "position");
    assert_eq!(hold.vd, -2.0);
    let current = parse(r#"{"robot":"drone","cmd":"hold"}"#);
    assert_eq!(current.cmd, LabCmd::Hold);
    assert_eq!(current.cmd.as_str(), "hold");
    assert!(matches!(
        AgentAction::parse_json(r#"{"robot":"rover","cmd":"explode"}"#),
        Err(LabError::UnknownCommand(_))
    ));
}

#[test]
fn named_constructors_match_open_catalogs() {
    let inland = Lab::inland(1);
    let harbor = Lab::harbor(1);
    let water = Lab::open_water(1);
    assert_eq!(inland.observe().scenario, "inland");
    assert_eq!(harbor.observe().scenario, "harbor");
    assert_eq!(water.observe().scenario, "open_water");
    assert_eq!(
        inland.observe().robots.len(),
        Lab::open("inland", 1).unwrap().observe().robots.len()
    );
    assert!(inland.observe().robots.iter().all(|r| r.id != "skiff"));
    assert!(water.observe().robots.iter().all(|r| r.id != "rover"));
    assert!(harbor.observe().robots.iter().any(|r| r.id == "surveyor"));
    assert_eq!(Lab::coastal(1).observe().scenario, "coastal");
}

#[test]
fn observation_exposes_safety_machines() {
    let mut lab = Lab::coastal(1);
    let obs = lab.observe();
    let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
    let g = rover.ground.as_ref().expect("ground machine");
    assert_eq!(g.phase, "parked");
    assert!(!g.drive_enabled);
    assert!(!rover.propulsion_live);
    assert!(rover.allows(LabCmd::Release));
    assert!(rover.allows(LabCmd::Estop));
    assert!(rover.allows(LabCmd::SetCharge));
    assert!(!rover.allows(LabCmd::Drive));
    assert!(!rover.allows(LabCmd::Halt));
    assert!(!rover.allows(LabCmd::Velocity));
    assert!(!rover.allows(LabCmd::Failsafe));
    assert_eq!(obs.env_cmds, LabCmd::ENV.to_vec());
    let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    let a = drone.aerial.as_ref().expect("aerial machine");
    assert_eq!(a.phase, "ready");
    assert!(!a.armed);
    assert!(!a.actuators_enabled);
    assert!(drone.allows(LabCmd::Arm));
    assert!(drone.allows(LabCmd::Failsafe));
    assert!(!drone.allows(LabCmd::Velocity));
    assert!(!drone.allows(LabCmd::Position));
    assert!(!drone.allows(LabCmd::Hold));
    assert!(!drone.allows(LabCmd::Takeoff));
    let skiff = obs.robots.iter().find(|r| r.id == "skiff").unwrap();
    assert!(skiff.allows(LabCmd::Undock));
    assert!(!skiff.allows(LabCmd::Thrust));
    lab.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    let rover = lab
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "rover")
        .unwrap();
    assert!(rover.ground.as_ref().unwrap().drive_enabled);
    assert!(rover.propulsion_live);
    assert!(rover.allows(LabCmd::Drive));
    assert!(rover.allows(LabCmd::Halt));
    assert!(!rover.allows(LabCmd::Release));
}

#[test]
fn json_touchdown_clears_command_like_touchdown_now() {
    use flight_core::frames::Ned;
    use flight_core::safety::Phase;
    use flight_core::vector::Velocity;

    let mut lab = Lab::open("inland", 1).unwrap();
    let mut drone = lab.attach_takeoff("drone").unwrap();
    drone
        .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
        .unwrap();
    drone.flush().unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"land"}"#)).unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"touchdown"}"#))
        .unwrap();
    let b = body(&lab, "drone");
    assert_eq!(b.aerial.unwrap().phase, Phase::Ready);
    assert!(!b.aerial.unwrap().armed);
    assert!(b.command.is_none());
}

#[test]
fn act_through_attach_takeoff_expands_the_log_for_replay() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Takeoff);
    assert!(body(&lab, "drone").aerial.unwrap().actuators_enabled);
    let cmds: Vec<_> = lab.log.iter().map(|a| a.action.cmd).collect();
    assert_eq!(
        cmds,
        vec![
            LabCmd::Arm,
            LabCmd::Offboard,
            LabCmd::EnableActuators,
            LabCmd::Takeoff
        ]
    );

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert_eq!(
        body(&replayed, "drone").aerial.unwrap().phase,
        Phase::Takeoff
    );
    assert!(replayed.all_hold());
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn position_is_legal_only_on_offboard_control_aerial() {
    let mut lab = Lab::open("inland", 1).unwrap();
    let drone = lab
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "drone")
        .unwrap();
    assert!(!drone.allows(LabCmd::Position));
    assert!(!drone.allows(LabCmd::Hold));
    assert!(!drone.allows(LabCmd::Velocity));
    let rover = Lab::open("inland", 1)
        .unwrap()
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "rover")
        .unwrap();
    assert!(!rover.allows(LabCmd::Position));
    assert!(!rover.allows(LabCmd::Hold));

    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let drone = lab
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "drone")
        .unwrap();
    assert!(drone.allows(LabCmd::Position));
    assert!(drone.allows(LabCmd::Hold));
    assert!(drone.allows(LabCmd::Velocity));
}

#[test]
fn act_through_attach_position_walks_set_position_now() {
    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let pose = body(&lab, "drone").position_m;
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Position).ned(pose[0], pose[1], -2.0))
        .unwrap();
    assert!(
        lab.log.iter().any(|a| a.action.cmd == LabCmd::Position),
        "attach position must log Position"
    );
    let cmd = body(&lab, "drone").command.expect("P-term after flush");
    assert!(
        cmd[2] < -0.5,
        "hold above the pad must command climb, got {cmd:?}"
    );

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed
        .replay_until(&lab.log, 0.02, lab.world().t.max(0.02))
        .unwrap();
    assert!(replayed.all_hold());
    let replayed_cmd = body(&replayed, "drone").command.expect("replayed P-term");
    assert!(replayed_cmd[2] < -0.5, "{replayed_cmd:?}");
}

#[test]
fn json_position_from_ready_is_rejected_and_never_a_velocity() {
    let mut lab = Lab::open("inland", 1).unwrap();
    let err = lab
        .act(AgentAction::new("drone", LabCmd::Position).ned(0.0, 0.0, -2.0))
        .unwrap_err();
    assert!(matches!(err, LabError::Aerial(_)), "{err}");
    assert!(body(&lab, "drone").command.is_none());

    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act(AgentAction::new("drone", LabCmd::Position).ned(0.0, 0.0, -2.0))
        .unwrap();
    let cmd = body(&lab, "drone").command.expect("JSON P-term");
    let pose = body(&lab, "drone").position_m;
    assert!(
        (cmd[2] - flight_core::mech::HOLD_KP * (-2.0 - pose[2])).abs() < 1e-4,
        "JSON position must be a P-term, not NED velocity {cmd:?} pose={pose:?}"
    );

    let err = lab
        .act(AgentAction::new("rover", LabCmd::Position).ned(0.0, 0.0, -2.0))
        .unwrap_err();
    assert!(matches!(err, LabError::WrongDomain), "{err}");
}

#[test]
fn position_hold_tracks_across_steps_without_reflush() {
    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let pose = body(&lab, "drone").position_m;
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Position).ned(pose[0], pose[1], -2.0))
        .unwrap();
    assert_eq!(body(&lab, "drone").hold_ned, Some([pose[0], pose[1], -2.0]));
    let alt0 = body(&lab, "drone").altitude_agl();
    for _ in 0..80 {
        lab.step(0.02);
    }
    let b = body(&lab, "drone");
    let alt1 = b.altitude_agl();
    assert!(alt1 > alt0 + 0.5, "hold must climb {alt0} → {alt1}");
    assert!(
        (alt1 - 2.0).abs() < 0.8,
        "hold must track 2 m AGL, got {alt1}"
    );
    assert_eq!(b.hold_ned, Some([pose[0], pose[1], -2.0]));
    assert!(lab.all_hold());

    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert!(body(&lab, "drone").hold_ned.is_none());
    assert!(body(&lab, "drone").command.is_none());
}

#[test]
fn observation_exposes_the_live_ned_hold() {
    let mut lab = Lab::open("inland", 1).unwrap();
    assert!(view(&lab, "drone").hold_ned.is_none());
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let pose = body(&lab, "drone").position_m;
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Position).ned(pose[0], pose[1], -2.0))
        .unwrap();
    assert_eq!(view(&lab, "drone").hold_ned, Some([pose[0], pose[1], -2.0]));
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert!(view(&lab, "drone").hold_ned.is_none());
}

#[test]
fn act_through_attach_hold_walks_attach_hold() {
    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let pose = body(&lab, "drone").position_m;
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Hold))
        .unwrap();
    assert!(
        lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold),
        "attach hold must log Hold"
    );
    assert_eq!(body(&lab, "drone").hold_ned, Some(pose));
    assert!(lab.all_hold());

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed
        .replay_until(&lab.log, 0.02, lab.world().t.max(0.02))
        .unwrap();
    assert!(replayed.all_hold());
    assert_eq!(body(&replayed, "drone").hold_ned, Some(pose));
    assert!(replayed.log.is_empty(), "replay must not re-log");
}

#[test]
fn operator_hold_survives_idle_steps_without_script() {
    let mut lab = Lab::open("inland", 3).unwrap();
    lab.attach_takeoff("drone").unwrap();
    lab.attach_hold("drone").unwrap();
    let hold = body(&lab, "drone").hold_ned;
    assert!(hold.is_some());
    for _ in 0..20 {
        lab.step(0.02);
    }
    assert_eq!(body(&lab, "drone").hold_ned, hold);
    assert!(lab.all_hold());
}

#[test]
fn apply_script_takeoff_velocity_clears_hold() {
    let mut lab = Lab::open("inland", 3).unwrap();
    lab.attach_takeoff("drone").unwrap();
    lab.attach_hold("drone").unwrap();
    assert!(body(&lab, "drone").hold_ned.is_some());
    lab.apply_script();
    assert!(
            body(&lab, "drone").hold_ned.is_none(),
            "scripted takeoff velocity must wipe hold; the demo stops apply_script after an operator act"
        );
}

#[test]
fn json_hold_from_ready_is_rejected_and_rover_is_wrong_domain() {
    let mut lab = Lab::open("inland", 1).unwrap();
    let err = lab
        .act(AgentAction::new("drone", LabCmd::Hold))
        .unwrap_err();
    assert!(matches!(err, LabError::Aerial(_)), "{err}");
    assert!(body(&lab, "drone").hold_ned.is_none());

    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    let pose = body(&lab, "drone").position_m;
    lab.act(AgentAction::new("drone", LabCmd::Hold)).unwrap();
    assert_eq!(body(&lab, "drone").hold_ned, Some(pose));

    let err = lab
        .act(AgentAction::new("rover", LabCmd::Hold))
        .unwrap_err();
    assert!(matches!(err, LabError::WrongDomain), "{err}");
}

#[test]
fn replay_until_walks_attach_without_relogging() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Recover))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!lab.log.is_empty());

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert!(
        replayed.log.is_empty(),
        "silent attach must not grow the log"
    );
    assert_eq!(body(&replayed, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&replayed, "drone").aerial.unwrap().failsafe);
    assert!(!body(&replayed, "drone").aerial.unwrap().armed);
    assert!(replayed.all_hold());
}

#[test]
fn act_through_attach_takeoff_from_offboard_walks_start_takeoff() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Arm))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Offboard))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Armed);
    let n = lab.log.len();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    assert_eq!(
        lab.log.len(),
        n + 1,
        "offboard takeoff must not expand the grant"
    );
    assert_eq!(lab.log.last().unwrap().action.cmd, LabCmd::Takeoff);
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Takeoff);

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert!(replayed.log.is_empty());
    assert_eq!(
        body(&replayed, "drone").aerial.unwrap().phase,
        Phase::Takeoff
    );
    assert!(replayed.all_hold());
}

#[test]
fn act_through_attach_enable_actuators_from_armed() {
    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Arm))
        .unwrap();
    assert!(!body(&lab, "drone").aerial.unwrap().actuators_enabled);
    lab.act_through_attach(AgentAction::new("drone", LabCmd::EnableActuators))
        .unwrap();
    assert!(body(&lab, "drone").aerial.unwrap().actuators_enabled);
    assert_eq!(lab.log.last().unwrap().action.cmd, LabCmd::EnableActuators);

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert!(replayed.log.is_empty());
    assert!(body(&replayed, "drone").aerial.unwrap().actuators_enabled);
    assert!(replayed.all_hold());
}

#[test]
fn act_through_attach_enable_actuators_from_ready_rejects() {
    let mut lab = Lab::open("inland", 1).unwrap();
    let err = lab
        .act_through_attach(AgentAction::new("drone", LabCmd::EnableActuators))
        .unwrap_err();
    assert!(matches!(err, LabError::Aerial(_)), "{err}");
    assert!(!body(&lab, "drone").aerial.unwrap().actuators_enabled);
    assert!(lab.log.is_empty());
}

#[test]
fn act_through_attach_failsafe_from_ready_walks_attach() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Failsafe);
    assert!(body(&lab, "drone").aerial.unwrap().failsafe);
    assert_eq!(lab.log.len(), 1);
    assert_eq!(lab.log[0].action.cmd, LabCmd::Failsafe);

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert!(replayed.log.is_empty());
    assert_eq!(
        body(&replayed, "drone").aerial.unwrap().phase,
        Phase::Failsafe
    );
    assert!(replayed.all_hold());
}

#[test]
fn act_through_attach_recover_from_failsafe_walks_recovery() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Failsafe);
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Recover))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&lab, "drone").aerial.unwrap().failsafe);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Disarm));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Recover));

    let mut until_failsafe = Vec::new();
    for a in &lab.log {
        until_failsafe.push(a.clone());
        if a.action.cmd == LabCmd::Failsafe {
            break;
        }
    }
    let mut mid = Lab::open("inland", 1).unwrap();
    mid.replay_until(&until_failsafe, 0.02, 0.02).unwrap();
    assert_eq!(body(&mid, "drone").aerial.unwrap().phase, Phase::Failsafe);

    let mut replayed = Lab::open("inland", 1).unwrap();
    replayed.replay_until(&lab.log, 0.02, 0.02).unwrap();
    assert_eq!(body(&replayed, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&replayed, "drone").aerial.unwrap().failsafe);
    assert!(replayed.all_hold());
}

#[test]
fn json_recover_from_failsafe_disarms_then_returns_ready() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Airborne))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Failsafe);
    assert!(view(&lab, "drone").allows(LabCmd::Recover));
    lab.act(AgentAction::new("drone", LabCmd::Recover)).unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&lab, "drone").aerial.unwrap().failsafe);
}

#[test]
fn act_through_attach_recover_after_airborne_failsafe() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Airborne))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Airborne);
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Recover))
        .unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&lab, "drone").aerial.unwrap().failsafe);
    assert!(lab.all_hold());
}

#[test]
fn json_recover_from_airborne_does_not_disarm() {
    use flight_core::safety::Phase;

    let mut lab = Lab::open("inland", 1).unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Takeoff))
        .unwrap();
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Airborne))
        .unwrap();
    let err = lab
        .act(AgentAction::new("drone", LabCmd::Recover))
        .unwrap_err();
    assert!(matches!(err, LabError::Aerial(_)), "{err}");
    let s = body(&lab, "drone").aerial.unwrap();
    assert_eq!(s.phase, Phase::Airborne);
    assert!(s.armed);
    assert!(!s.failsafe);
}

#[test]
fn scripted_airborne_failsafe_then_attach_recover() {
    use flight_core::safety::Phase;

    let mut lab = Lab::coastal(7);
    for _ in 0..500 {
        lab.apply_script();
        lab.step(0.02);
        if body(&lab, "drone").aerial.unwrap().phase == Phase::Airborne {
            break;
        }
    }
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Airborne);
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert!(body(&lab, "drone").aerial.unwrap().failsafe);
    lab.act_through_attach(AgentAction::new("drone", LabCmd::Recover))
        .unwrap();
    let s = body(&lab, "drone").aerial.unwrap();
    assert_eq!(s.phase, Phase::Ready);
    assert!(!s.failsafe);
    assert!(!s.armed);
    assert!(lab.all_hold());
}

#[test]
fn observation_exposes_terrain_contact() {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    let lab = Lab::coastal(1);
    let obs = lab.observe();
    let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
    assert!(rover.terrain_contact);
    assert_eq!(rover.support, "terrain");
    let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    assert!(drone.terrain_contact);
    assert_eq!(drone.support, "terrain");
    let skiff = obs.robots.iter().find(|r| r.id == "skiff").unwrap();
    assert!(!skiff.terrain_contact);
    assert_eq!(skiff.support, "water");
    let surveyor = obs.robots.iter().find(|r| r.id == "surveyor").unwrap();
    assert!(!surveyor.terrain_contact);
    assert_eq!(surveyor.support, "water");
    assert!(surveyor.contact_jn.abs() < 1e-9);

    let inland = Lab::open("inland", 1).unwrap();
    let rover = inland
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "rover")
        .unwrap();
    assert!(rover.terrain_contact);
    assert_eq!(rover.support, "terrain");

    let water = Lab::open("open_water", 1).unwrap();
    let drone = water
        .observe()
        .robots
        .into_iter()
        .find(|r| r.id == "drone")
        .unwrap();
    assert!(
        !drone.terrain_contact,
        "drone over deep water is not on the seabed"
    );
    assert_eq!(drone.support, "air");

    let lab = Lab::coastal(1);
    let mut drone = lab.attach_takeoff("drone").unwrap();
    for _ in 0..80 {
        drone
            .set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
            .unwrap();
        drone.flush().unwrap();
        lab.session().step(0.02).unwrap();
    }
    let end = lab.observe();
    let drone = end.robots.iter().find(|r| r.id == "drone").unwrap();
    assert!(!drone.terrain_contact);
    assert_eq!(drone.support, "air");
    let rover = end.robots.iter().find(|r| r.id == "rover").unwrap();
    assert!(rover.terrain_contact);
    assert_eq!(rover.support, "terrain");
}

#[test]
fn observation_exposes_sphere_contact() {
    let lab = Lab::open("inland", 1).unwrap();
    lab.with_world_mut(|w| {
        let rover = w.body_mut("rover").unwrap();
        rover.position_m = [6.0, 0.05, 0.0];
        rover.velocity_mps = [0.0, -0.8, 0.0];
        let drone = w.body_mut("drone").unwrap();
        drone.velocity_mps = [0.0, 0.4, 0.0];
    });
    lab.session().step(0.02).unwrap();
    let obs = lab.observe();
    assert!(obs.all_hold, "broken {:?}", obs.properties);
    let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
    assert!(
        drone.sphere_contact || rover.sphere_contact,
        "drone jn={} rover jn={}",
        drone.sphere_jn,
        rover.sphere_jn
    );
    assert!(drone.sphere_jn > 0.0 || rover.sphere_jn > 0.0);
    assert!(drone.sphere_contact == (drone.sphere_jn > 1e-6));
    let hit = obs
        .sphere_hits
        .iter()
        .find(|h| h.involves("drone") && h.involves("rover"))
        .expect("pairwise graph");
    assert!(hit.jn > 0.0);
    assert!(drone.sphere_partners.iter().any(|p| p == "rover"));
    assert!(rover.sphere_partners.iter().any(|p| p == "drone"));
}

fn body(lab: &Lab, id: &str) -> Body {
    lab.world().body(id).unwrap().clone()
}

#[test]
fn open_rejects_unknown_scenario() {
    assert!(matches!(
        Lab::open("moon", 1),
        Err(LabError::UnknownScenario(_))
    ));
}

#[test]
fn catalog_worlds_hold() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        for _ in 0..200 {
            lab.apply_script();
            lab.step(0.02);
            assert!(
                lab.all_hold(),
                "{name} t={} {:?}",
                lab.world().t,
                lab.world().last_properties
            );
        }
        assert_eq!(lab.observe().scenario, *name);
    }
}

#[test]
fn set_waves_and_jsonl_roundtrip() {
    let mut lab = Lab::coastal(1);
    lab.act(parse(r#"{"cmd":"set_waves","vn":0.2,"ve":0.4,"vd":1.0}"#))
        .unwrap();
    assert!((lab.world().env.wave_amp - 0.2).abs() < 1e-6);
    lab.step(0.02);
    let mut buf = Vec::new();
    lab.write_jsonl(&mut buf).unwrap();
    let line = std::str::from_utf8(&buf).unwrap();
    assert!(line.contains("\"scenario\":\"coastal\""));
    assert!(line.ends_with('\n'));
}

#[test]
fn write_mcap_includes_observation_and_actions() {
    let mut lab = Lab::coastal(1);
    lab.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    lab.step(0.02);
    let bytes = lab.write_mcap(Vec::new()).unwrap();
    assert!(looks_like_mcap(&bytes));
    let obs = serde_json::to_vec(&lab.observe()).unwrap();
    assert!(bytes.windows(obs.len()).any(|w| w == obs));
    assert!(bytes.windows(7).any(|w| w == b"release"));
}

#[test]
fn set_charge_empty_kills_thrust() {
    let mut lab = Lab::coastal(1);
    lab.act(parse(r#"{"robot":"drone","cmd":"arm"}"#)).unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"offboard"}"#))
        .unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"enable_actuators"}"#))
        .unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"takeoff"}"#))
        .unwrap();
    lab.act(parse(r#"{"robot":"drone","cmd":"set_charge","vn":0}"#))
        .unwrap();
    lab.act(parse(
        r#"{"robot":"drone","cmd":"velocity","vn":0,"ve":0,"vd":-1.2}"#,
    ))
    .unwrap();
    for _ in 0..80 {
        lab.step(0.02);
    }
    let obs = lab.observe();
    let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
    assert_eq!(drone.charge_j, 0.0);
    assert!(drone.alt < 0.3);
    assert!(lab.all_hold());
}

#[test]
fn action_log_replays() {
    let mut a = Lab::coastal(4);
    a.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    a.act(parse(
        r#"{"robot":"rover","cmd":"drive","vn":-0.5,"ve":0.1}"#,
    ))
    .unwrap();
    for _ in 0..60 {
        a.step(0.02);
    }
    let mut buf = Vec::new();
    a.write_actions_jsonl(&mut buf).unwrap();
    let log: Vec<TimedAction> = std::str::from_utf8(&buf)
        .unwrap()
        .lines()
        .filter(|l| !l.is_empty())
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(log.len(), 2);

    let mut b = Lab::coastal(4);
    b.replay_until(&log, 0.02, a.world().t).unwrap();
    let ra = body(&a, "rover").position_m;
    let rb = body(&b, "rover").position_m;
    for (u, v) in ra.iter().zip(rb.iter()) {
        assert!((u - v).abs() < 1e-4, "{ra:?} vs {rb:?}");
    }
    assert!(b.all_hold());
}

#[test]
fn research_probe_holds_on_every_scenario() {
    for name in Lab::scenarios() {
        let mut lab = Lab::open(name, 3).unwrap();
        let report = lab.research_probe(0.02, 80);
        assert!(
            report.ok(),
            "{name}: leaked={:?} broken={:?}",
            report.illegal_leaked,
            report.broken
        );
        assert!(
            report.illegal_rejected >= 8,
            "{name} {}",
            report.illegal_rejected
        );
    }
}

#[test]
fn research_probe_legal_abuse_walks_attach() {
    use flight_core::safety::Phase;

    let mut lab = Lab::coastal(3);
    let report = lab.research_probe(0.02, 40);
    assert!(report.ok(), "{report} leaked={:?}", report.illegal_leaked);
    assert!(
        report.legal_applied >= 14,
        "grants + motion, applied={}",
        report.legal_applied
    );
    assert!(body(&lab, "rover").ground.unwrap().estop);
    let drone = body(&lab, "drone").aerial.unwrap();
    assert_eq!(drone.phase, Phase::Takeoff);
    assert!(drone.actuators_enabled);
    assert_eq!(body(&lab, "drone").charge_j, 0.0);
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Release));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Takeoff));
    assert!(lab.log.iter().any(|a| a.action.cmd == LabCmd::Hold));
    assert!(lab.all_hold());
}

#[test]
fn research_probe_rejects_pad_illegal_cmds() {
    let mut lab = Lab::coastal(3);
    assert!(lab
        .act(AgentAction::new("drone", LabCmd::Position).ned(0.0, 0.0, -2.0))
        .is_err());
    assert!(lab
        .act(AgentAction::new("drone", LabCmd::Airborne))
        .is_err());
    assert!(lab.act(AgentAction::new("skiff", LabCmd::Station)).is_err());
    assert!(lab.act(AgentAction::new("rover", LabCmd::Halt)).is_err());
    lab.act(AgentAction::new("drone", LabCmd::Failsafe))
        .unwrap();
    assert!(lab.act(AgentAction::new("drone", LabCmd::Hold)).is_err());
    let report = {
        let mut probe = Lab::coastal(3);
        probe.research_probe(0.02, 8)
    };
    assert!(report.ok(), "{report} leaked={:?}", report.illegal_leaked);
    assert!(
        report.illegal_rejected >= 12,
        "pad + failsafe hold, rejected={}",
        report.illegal_rejected
    );
}

#[test]
fn typed_telemetry_matches_json_observe() {
    let mut lab = Lab::coastal(1);
    lab.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    let t0 = lab.world().t;
    let tel = lab.ground("rover").telemetry_now().unwrap();
    assert_eq!(lab.world().t, t0);
    let obs = lab.observe();
    let rover = obs.robots.iter().find(|r| r.id == "rover").unwrap();
    assert!((tel.position.x() - rover.n).abs() < 1e-5);
    assert!((tel.position.y() - rover.e).abs() < 1e-5);
    assert!(tel.armed);
    assert!(rover.armed);
}

#[test]
fn typestate_backend_shares_lab_plant() {
    use flight_core::frames::Ned;
    use flight_core::vector::Velocity;

    let lab = Lab::coastal(1);
    lab.attach_drive("rover").unwrap();
    let n0 = body(&lab, "rover").position_m[0];

    let GroundHandle::Moving(mut rover) = lab.ground_vehicle("rover").unwrap() else {
        panic!("attach_drive then attach must be Moving");
    };
    rover
        .set_velocity_ned_now(Velocity::<Ned>::ned(-0.8, 0.0, 0.0))
        .unwrap();
    rover.backend().flush().unwrap();
    for _ in 0..20 {
        lab.session().step(0.02).unwrap();
    }

    let n = body(&lab, "rover").position_m[0];
    assert!(
        n < n0,
        "attached Moving handle must move the lab plant: n0={n0} n={n}"
    );
    assert!(lab.all_hold());
}

#[test]
fn lab_attach_matches_live_machines() {
    let lab = Lab::open("inland", 1).unwrap();
    assert!(matches!(
        lab.aerial_vehicle("drone").unwrap(),
        VehicleHandle::PreflightReady(_)
    ));
    assert!(matches!(
        lab.ground_vehicle("rover").unwrap(),
        GroundHandle::Parked(_)
    ));
    lab.aerial("drone").grant_offboard().unwrap();
    lab.ground("rover").grant_drive().unwrap();
    assert!(matches!(
        lab.aerial_vehicle("drone").unwrap(),
        VehicleHandle::Takeoff(_)
    ));
    assert!(matches!(
        lab.ground_vehicle("rover").unwrap(),
        GroundHandle::Moving(_)
    ));

    let lab = Lab::coastal(1);
    assert!(matches!(
        lab.marine_vehicle("skiff").unwrap(),
        MarineHandle::Docked(_)
    ));
    lab.marine("skiff").grant_undock().unwrap();
    assert!(matches!(
        lab.marine_vehicle("skiff").unwrap(),
        MarineHandle::Underway(_)
    ));
}

#[test]
fn lab_attach_helpers_walk_consume_self_typestate() {
    use flight_core::ground::GroundPhase;
    use flight_core::marine::MarinePhase;
    use flight_core::safety::Phase;

    let lab = Lab::coastal(1);
    lab.attach_takeoff("drone").unwrap();
    lab.attach_drive("rover").unwrap();
    lab.attach_undock("skiff").unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Takeoff);
    assert_eq!(
        body(&lab, "rover").ground.unwrap().phase,
        GroundPhase::Moving
    );
    assert_eq!(
        body(&lab, "skiff").marine.unwrap().phase,
        MarinePhase::Underway
    );
    assert!(matches!(
        lab.aerial_vehicle("drone").unwrap(),
        VehicleHandle::Takeoff(_)
    ));
    assert_eq!(
        lab.attach_takeoff("drone").unwrap_err(),
        BackendError::Protocol
    );

    lab.attach_airborne("drone").unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Airborne);
    lab.attach_failsafe("drone").unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Failsafe);
    lab.attach_recover_ready("drone").unwrap();
    assert_eq!(body(&lab, "drone").aerial.unwrap().phase, Phase::Ready);
    assert!(!body(&lab, "drone").aerial.unwrap().failsafe);
    lab.attach_estop("rover").unwrap();
    lab.attach_reset("rover").unwrap();
    assert_eq!(
        body(&lab, "rover").ground.unwrap().phase,
        GroundPhase::Parked
    );
    lab.attach_marine_failsafe("skiff").unwrap();
    assert_eq!(
        body(&lab, "skiff").marine.unwrap().phase,
        MarinePhase::Failsafe
    );
    lab.attach_recover("skiff").unwrap();
    assert_eq!(
        body(&lab, "skiff").marine.unwrap().phase,
        MarinePhase::Docked
    );
}

#[test]
fn observation_kind_matches_attach_not_plant_phase() {
    let lab = Lab::open("inland", 1).unwrap();
    let drone = view(&lab, "drone");
    let a = drone.aerial.as_ref().unwrap();
    assert_eq!(a.phase, "ready");
    assert_eq!(a.kind, AerialKind::PreflightReady);
    let rover = view(&lab, "rover");
    let g = rover.ground.as_ref().unwrap();
    assert_eq!(g.phase, "parked");
    assert_eq!(g.kind, GroundKind::Parked);

    lab.attach_offboard("drone").unwrap();
    let a = view(&lab, "drone").aerial.unwrap();
    assert_eq!(a.phase, "armed");
    assert_eq!(a.kind, AerialKind::Offboard);

    let lab = Lab::open("inland", 2).unwrap();
    lab.attach_takeoff("drone").unwrap();
    lab.attach_airborne("drone").unwrap();
    let a = view(&lab, "drone").aerial.unwrap();
    assert_eq!(a.phase, "airborne");
    assert_eq!(a.kind, AerialKind::Airborne);

    lab.attach_failsafe("drone").unwrap();
    let a = view(&lab, "drone").aerial.unwrap();
    assert_eq!(a.phase, "failsafe");
    assert_eq!(a.kind, AerialKind::Failsafe);
    let VehicleHandle::Failsafe(fs) = lab.aerial_vehicle("drone").unwrap() else {
        panic!("failsafe maps to Failsafe");
    };
    let _ = fs.disarm_now().unwrap().into_backend();
    let a = view(&lab, "drone").aerial.unwrap();
    assert_eq!(a.phase, "recovery");
    assert_eq!(a.kind, AerialKind::Recovery);
    assert!(view(&lab, "drone").allows(LabCmd::Recover));
    lab.attach_recover_ready("drone").unwrap();
    let a = view(&lab, "drone").aerial.unwrap();
    assert_eq!(a.phase, "ready");
    assert_eq!(a.kind, AerialKind::PreflightReady);

    lab.attach_drive("rover").unwrap();
    lab.attach_estop("rover").unwrap();
    let g = view(&lab, "rover").ground.unwrap();
    assert_eq!(g.phase, "estop");
    assert_eq!(g.kind, GroundKind::EStopped);

    let lab = Lab::coastal(1);
    let m = view(&lab, "skiff").marine.unwrap();
    assert_eq!(m.phase, "docked");
    assert_eq!(m.kind, MarineKind::Docked);
    lab.attach_undock("skiff").unwrap();
    lab.attach_station("skiff").unwrap();
    let m = view(&lab, "skiff").marine.unwrap();
    assert_eq!(m.phase, "station_keep");
    assert_eq!(m.kind, MarineKind::StationKeep);
}

fn view(lab: &Lab, id: &str) -> RobotView {
    lab.observe()
        .robots
        .into_iter()
        .find(|r| r.id == id)
        .unwrap_or_else(|| panic!("missing {id}"))
}

#[test]
fn clone_snapshots_the_plant() {
    let mut a = Lab::coastal(1);
    a.act(parse(r#"{"robot":"rover","cmd":"release"}"#))
        .unwrap();
    a.act(parse(
        r#"{"robot":"rover","cmd":"drive","vn":-0.6,"ve":0.0}"#,
    ))
    .unwrap();
    let b = a.clone();
    for _ in 0..50 {
        a.step(0.02);
    }
    let na = body(&a, "rover").position_m[0];
    let nb = body(&b, "rover").position_m[0];
    assert!(
        (na - nb).abs() > 0.05,
        "clone must not share the Mutex: live={na} snapshot={nb}"
    );
}
