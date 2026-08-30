use serde_json::json;

use crate::{
    handle_rpc, preview_write, read_channel, validate_chain_report, validate_discovery,
    validate_read, validate_reference, validate_write, ChainDoc, ChainOp, Driver, DriverLimits,
    MhsError, WriteRequest, CONFORMANCE, DEVICE_ENV, DEVICE_LAB, PROFILE,
};

fn ids(d: &crate::Discovery) -> Vec<String> {
    d.ids().into_iter().map(str::to_string).collect()
}

#[test]
fn profile_is_shaped_not_official() {
    assert_eq!(CONFORMANCE, "shaped");
    assert!(PROFILE.contains("mhs-shaped"));
    let d = Driver::coastal(1).discover();
    assert!(!d.official);
    assert_eq!(d.conformance, "shaped");
    assert_eq!(d.spec_url, "https://modelhardwarestandard.com");
}

#[test]
fn coastal_discovery_has_four_robots_env_and_lab() {
    let d = Driver::coastal(1).discover();
    let ids = ids(&d);
    assert!(ids.contains(&"drone".into()));
    assert!(ids.contains(&"rover".into()));
    assert!(ids.contains(&"skiff".into()));
    assert!(ids.contains(&"surveyor".into()));
    assert!(ids.contains(&DEVICE_ENV.into()));
    assert!(ids.contains(&DEVICE_LAB.into()));
    validate_discovery(&serde_json::to_value(&d).unwrap()).unwrap();
}

#[test]
fn inland_omits_hulls_open_water_omits_rover() {
    let inland = Driver::open("inland", 1).unwrap().discover();
    let inland_ids = ids(&inland);
    assert!(inland_ids.contains(&"drone".into()));
    assert!(inland_ids.contains(&"rover".into()));
    assert!(!inland_ids.contains(&"skiff".into()));
    assert!(!inland_ids.contains(&"surveyor".into()));

    let open = Driver::open("open_water", 1).unwrap().discover();
    let open_ids = ids(&open);
    assert!(open_ids.contains(&"drone".into()));
    assert!(open_ids.contains(&"skiff".into()));
    assert!(!open_ids.contains(&"rover".into()));
}

#[test]
fn rover_reference_lists_drive_but_parked_legal_now_does_not() {
    let drv = Driver::coastal(1);
    let r = drv.reference("rover").unwrap();
    assert!(!r.official);
    assert_eq!(r.mass_kg, Some(28.0));
    assert!(r.writes.iter().any(|w| w.channel == "drive"));
    assert!(!r.legal_now.iter().any(|c| c == "drive"));
    assert!(r.legal_now.iter().any(|c| c == "release"));
    assert!(r.tags.iter().any(|t| t.key == "drive"));
    validate_reference(&serde_json::to_value(&r).unwrap()).unwrap();
}

#[test]
fn catalog_masses_match_plant() {
    let lab = robot_lab::Lab::coastal(1);
    let world = lab.world();
    for id in ["drone", "rover", "skiff", "surveyor"] {
        let body = world.body(id).unwrap();
        assert_eq!(crate::tags::catalog_mass_kg(id), Some(body.mass_kg), "{id}");
    }
}

#[test]
fn read_does_not_step() {
    let mut drv = Driver::coastal(1);
    let t0 = drv.lab().world().t;
    let pose = drv.read("drone", "pose.ned").unwrap();
    assert_eq!(pose.t, t0);
    assert_eq!(drv.lab().world().t, t0);
    validate_read(&serde_json::to_value(&pose).unwrap()).unwrap();
    drv.step(0.02, 1);
    assert!((drv.lab().world().t - 0.02).abs() < 1e-6);
}

#[test]
fn parked_drive_write_is_not_legal() {
    let mut drv = Driver::coastal(1);
    let err = drv
        .write(&WriteRequest::new("rover", "drive").ned(-1.0, 0.0, 0.0))
        .unwrap_err();
    assert!(matches!(
        err,
        MhsError::NotLegal {
            cmd: robot_lab::LabCmd::Drive,
            ..
        }
    ));
    let f = drv.last_failure(&err);
    assert_eq!(f.code, "not_legal");
    assert!(!f.ok);
}

#[test]
fn docked_thrust_write_is_not_legal() {
    let mut drv = Driver::coastal(1);
    let err = drv
        .write(&WriteRequest::new("skiff", "thrust").ned(0.5, 0.0, 0.0))
        .unwrap_err();
    assert!(matches!(
        err,
        MhsError::NotLegal {
            cmd: robot_lab::LabCmd::Thrust,
            ..
        }
    ));
}

#[test]
fn inland_hull_write_is_p11() {
    let mut drv = Driver::open("inland", 1).unwrap();
    let err = drv
        .write(&WriteRequest::new("skiff", "undock"))
        .unwrap_err();
    match err {
        MhsError::UnknownDevice { id, invariant } => {
            assert_eq!(id, "skiff");
            assert_eq!(invariant, Some("P11"));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn open_water_rover_write_is_p11() {
    let mut drv = Driver::open("open_water", 1).unwrap();
    let err = drv
        .write(&WriteRequest::new("rover", "release"))
        .unwrap_err();
    match err {
        MhsError::UnknownDevice { id, invariant } => {
            assert_eq!(id, "rover");
            assert_eq!(invariant, Some("P11"));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn legal_release_then_drive_does_not_step() {
    let mut drv = Driver::coastal(1);
    drv.write(&WriteRequest::new("rover", "release")).unwrap();
    drv.write(&WriteRequest::new("rover", "drive").ned(-0.4, 0.0, 0.0))
        .unwrap();
    assert_eq!(drv.lab().world().t, 0.0);
    drv.step(0.02, 1);
    assert!(drv.lab().all_hold());
}

#[test]
fn overspeed_drive_is_limit_when_moving() {
    let mut drv = Driver::coastal(1);
    drv.write(&WriteRequest::new("rover", "release")).unwrap();
    let err = drv
        .write(&WriteRequest::new("rover", "drive").ned(100.0, 0.0, 0.0))
        .unwrap_err();
    match err {
        MhsError::Limit(l) => {
            assert_eq!(l.id, "ned_speed");
            assert!(l.got.unwrap() > l.max.unwrap());
        }
        other => panic!("{other:?}"),
    }
    let kind = drv
        .read("rover", "machine")
        .unwrap()
        .value
        .get("kind")
        .unwrap()
        .as_str()
        .unwrap()
        .to_string();
    assert_eq!(kind, "moving");
}

#[test]
fn over_capacity_charge_is_limit() {
    let mut drv = Driver::coastal(1);
    let err = drv
        .write(&WriteRequest::new("rover", "set_charge").ned(9_999_999.0, 0.0, 0.0))
        .unwrap_err();
    assert!(matches!(err, MhsError::Limit(ref l) if l.id == "charge"));
}

#[test]
fn lab_is_read_only() {
    let obs = Driver::coastal(1).lab().observe();
    let err = preview_write(
        &obs,
        &WriteRequest::new(DEVICE_LAB, "all_hold"),
        &DriverLimits::DEFAULT,
    )
    .unwrap_err();
    assert!(matches!(err, MhsError::ReadOnly { .. }));
}

#[test]
fn takeoff_write_is_attach_grant() {
    let mut drv = Driver::coastal(1);
    drv.write(&WriteRequest::new("drone", "takeoff")).unwrap();
    let m = drv.read("drone", "machine").unwrap();
    assert_eq!(m.value["kind"], "takeoff");
}

#[test]
fn chain_grants_then_one_step_per_tick() {
    let mut drv = Driver::open("harbor", 1).unwrap();
    let doc = ChainDoc::parse(
        r#"[
          {"op":"write","device":"drone","channel":"takeoff"},
          {"op":"write","device":"rover","channel":"release"},
          {"op":"write","device":"skiff","channel":"undock"},
          {"op":"write","device":"surveyor","channel":"undock"},
          {"op":"step","dt":0.02,"n":4},
          {"op":"read","device":"lab","channel":"all_hold"}
        ]"#,
    )
    .unwrap();
    let report = drv.run_chain(&doc.ops, 0.02);
    assert!(report.ok, "{:?}", report.rejects);
    assert_eq!(report.steps, 4);
    assert!((report.t - 0.08).abs() < 1e-3, "P12 t={}", report.t);
    assert!(report.all_hold);
    validate_chain_report(&serde_json::to_value(&report).unwrap()).unwrap();
}

#[test]
fn chain_stops_on_parked_drive() {
    let mut drv = Driver::coastal(1);
    let doc = ChainDoc::parse(
        r#"
{"op":"write","device":"rover","channel":"drive","vn":-0.4}
{"op":"step","n":2}
"#,
    )
    .unwrap();
    let report = drv.run_chain(&doc.ops, 0.02);
    assert!(!report.ok);
    assert_eq!(report.steps, 0);
    assert_eq!(report.rejects[0].code, "not_legal");
}

#[test]
fn write_request_matches_schema() {
    let req = WriteRequest::new("rover", "release");
    validate_write(&serde_json::to_value(&req).unwrap()).unwrap();
}

#[test]
fn mcp_tools_list_and_illegal_write() {
    let mut drv = Driver::coastal(1);
    let listed = handle_rpc(
        &mut drv,
        &json!({"jsonrpc":"2.0","id":1,"method":"tools/list"}),
    )
    .unwrap();
    let names: Vec<_> = listed["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert!(names.contains(&"mhs_discover"));
    assert!(names.contains(&"mhs_write"));

    let bounced = handle_rpc(
        &mut drv,
        &json!({
            "jsonrpc":"2.0",
            "id":2,
            "method":"tools/call",
            "params": {
                "name":"mhs_write",
                "arguments": {"device":"rover","channel":"drive","vn":-1.0}
            }
        }),
    )
    .unwrap();
    assert_eq!(bounced["result"]["isError"], true);
}

#[test]
fn idle_catalog_certificates_are_false_until_hold() {
    let r = read_channel(
        &Driver::coastal(1).lab().observe(),
        DEVICE_LAB,
        "certificates",
    )
    .unwrap();
    let ids = r.value.as_array().unwrap();
    assert!(
        !ids.iter().any(|v| v == "fleet_hold_simultaneous"),
        "idle catalog must not claim fleet hold"
    );
}

#[test]
fn chain_op_roundtrip() {
    let op = ChainOp::Write {
        device: "rover".into(),
        channel: "release".into(),
        vn: 0.0,
        ve: 0.0,
        vd: 0.0,
        yaw_rate: 0.0,
    };
    let v = serde_json::to_value(&op).unwrap();
    assert_eq!(v["op"], "write");
}

#[test]
fn all_catalog_references_validate() {
    for name in robot_lab::Lab::scenarios() {
        let drv = Driver::open(name, 1).unwrap();
        for r in drv.references() {
            validate_reference(&serde_json::to_value(&r).unwrap()).unwrap_or_else(|e| {
                panic!("{name} {} {e}", r.id);
            });
        }
    }
}

#[test]
fn preview_unknown_channel() {
    let obs = Driver::coastal(1).lab().observe();
    let err = preview_write(
        &obs,
        &WriteRequest::new("drone", "set_temperature"),
        &DriverLimits::DEFAULT,
    )
    .unwrap_err();
    assert!(matches!(err, MhsError::UnknownChannel { .. }));
}
