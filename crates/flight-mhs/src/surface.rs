//! Discovery document and compiled device reference (tags → structured file).

use robot_lab::{AerialKind, LabCmd, MarineKind, Observation, RobotView, FLEET_HOLD_SIMULTANEOUS};
use serde::{Deserialize, Serialize};

use crate::error::MhsError;
use crate::limits::{DriverLimits, LimitReject};
use crate::tags::{
    catalog_mass_kg, domain_writes, env_tags, lab_tags, robot_tags, speed_limit, DEVICE_ENV,
    DEVICE_LAB,
};
use crate::{CONFORMANCE, PROFILE, SPEC_NOTE, SPEC_URL};

/// Stub in a discovery list — enough for an agent to fetch a reference.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceStub {
    pub id: String,
    pub kind: String,
    pub domain: String,
}

/// Standard-format discovery for one catalog snapshot. Does not step.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Discovery {
    pub profile: String,
    pub conformance: String,
    pub official: bool,
    pub spec_url: String,
    pub note: String,
    pub scenario: String,
    pub seed: u64,
    pub t: f32,
    pub devices: Vec<DeviceStub>,
}

impl Discovery {
    pub fn from_observation(obs: &Observation) -> Self {
        let mut devices: Vec<DeviceStub> = obs
            .robots
            .iter()
            .map(|r| DeviceStub {
                id: r.id.clone(),
                kind: "robot".into(),
                domain: r.domain.clone(),
            })
            .collect();
        devices.push(DeviceStub {
            id: DEVICE_ENV.into(),
            kind: "environment".into(),
            domain: "environment".into(),
        });
        devices.push(DeviceStub {
            id: DEVICE_LAB.into(),
            kind: "lab".into(),
            domain: "lab".into(),
        });
        Self {
            profile: PROFILE.into(),
            conformance: CONFORMANCE.into(),
            official: false,
            spec_url: SPEC_URL.into(),
            note: SPEC_NOTE.into(),
            scenario: obs.scenario.into(),
            seed: obs.seed,
            t: obs.t,
            devices,
        }
    }

    pub fn ids(&self) -> Vec<&str> {
        self.devices.iter().map(|d| d.id.as_str()).collect()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Measure {
    pub channel: String,
    pub prose: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WriteCapability {
    pub channel: String,
    pub prose: String,
    pub needs_values: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SafetyLimit {
    pub id: String,
    pub prose: String,
    pub enforcement: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub remaining_spec: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

/// Compiled reference file for one device.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeviceReference {
    pub id: String,
    pub kind: String,
    pub domain: String,
    pub profile: String,
    pub conformance: String,
    pub official: bool,
    pub spec_url: String,
    pub tags: Vec<crate::tags::DeviceTag>,
    pub measures: Vec<Measure>,
    pub writes: Vec<WriteCapability>,
    pub legal_now: Vec<String>,
    pub safety: Vec<SafetyLimit>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_kg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radius_m: Option<f32>,
}

impl DeviceReference {
    pub fn compile(
        obs: &Observation,
        device: &str,
        limits: &DriverLimits,
    ) -> Result<Self, MhsError> {
        if device == DEVICE_LAB {
            return Ok(lab_reference(obs, limits));
        }
        if device == DEVICE_ENV {
            return Ok(env_reference(obs, limits));
        }
        let robot = obs
            .robots
            .iter()
            .find(|r| r.id == device)
            .ok_or_else(|| MhsError::unknown_device(obs.scenario, device))?;
        Ok(robot_reference(robot, limits))
    }
}

fn measure(channel: &str, prose: &str, unit: Option<&str>) -> Measure {
    Measure {
        channel: channel.into(),
        prose: prose.into(),
        unit: unit.map(str::to_string),
    }
}

fn write_cap(cmd: LabCmd, prose: &str, needs_values: bool) -> WriteCapability {
    WriteCapability {
        channel: cmd.as_str().into(),
        prose: prose.into(),
        needs_values,
    }
}

fn robot_reference(r: &RobotView, limits: &DriverLimits) -> DeviceReference {
    let writes: Vec<WriteCapability> = domain_writes(&r.domain)
        .iter()
        .copied()
        .map(|cmd| {
            let needs = matches!(
                cmd,
                LabCmd::Velocity
                    | LabCmd::Position
                    | LabCmd::Drive
                    | LabCmd::Thrust
                    | LabCmd::SetCharge
            );
            write_cap(cmd, write_prose(cmd), needs)
        })
        .collect();
    let mut safety = vec![
        SafetyLimit {
            id: "legal_cmds".into(),
            prose: "Writes not in legal_now are rejected before attach.".into(),
            enforcement: "machine".into(),
            remaining_spec: None,
            max: None,
            unit: None,
        },
        SafetyLimit {
            id: "finite".into(),
            prose: "Non-finite write values are rejected.".into(),
            enforcement: "numeric".into(),
            remaining_spec: None,
            max: None,
            unit: None,
        },
        SafetyLimit {
            id: "yaw_rate".into(),
            prose: "Absolute yaw rate clamp at the driver.".into(),
            enforcement: "numeric".into(),
            remaining_spec: None,
            max: Some(limits.yaw_rate_rps),
            unit: Some("rad/s".into()),
        },
        SafetyLimit {
            id: "charge".into(),
            prose: "set_charge must stay in [0, capacity_j].".into(),
            enforcement: "numeric".into(),
            remaining_spec: None,
            max: Some(r.capacity_j),
            unit: Some("J".into()),
        },
    ];
    if let Some((max, channel)) = speed_limit(&r.domain, limits) {
        safety.push(SafetyLimit {
            id: "ned_speed".into(),
            prose: format!("|{channel}| NED speed clamp when that write is legal."),
            enforcement: "numeric".into(),
            remaining_spec: None,
            max: Some(max),
            unit: Some("m/s".into()),
        });
    }
    if r.domain == "aerial" {
        safety.push(SafetyLimit {
            id: "position".into(),
            prose: "Aerial position |NED| clamp.".into(),
            enforcement: "numeric".into(),
            remaining_spec: None,
            max: Some(limits.position_m),
            unit: Some("m".into()),
        });
        safety.push(SafetyLimit {
            id: "takeoff_grant".into(),
            prose: "Ready Takeoff is attach-only (P2).".into(),
            enforcement: "machine".into(),
            remaining_spec: Some("P2".into()),
            max: None,
            unit: None,
        });
    }
    if matches!(r.domain.as_str(), "surface" | "underwater") {
        safety.push(SafetyLimit {
            id: "docked_failsafe".into(),
            prose: "Do not add declare_failsafe on Docked (P3).".into(),
            enforcement: "machine".into(),
            remaining_spec: Some("P3".into()),
            max: None,
            unit: None,
        });
    }
    let mut measures = vec![
        measure("identity", "Device id, domain, kind, phase.", None),
        measure("pose.ned", "NED pose, z-down.", Some("m")),
        measure("velocity.ned", "NED velocity, z-down.", Some("m/s")),
        measure(
            "hold_ned",
            "Live pose hold target when tracking.",
            Some("m"),
        ),
        measure("charge", "Stored energy and capacity.", Some("J")),
        measure(
            "legal_cmds",
            "Commands the live machine would accept.",
            None,
        ),
        measure(
            "machine",
            "Attach kind vs plant phase (load-bearing, not debug).",
            None,
        ),
    ];
    if r.domain == "aerial" {
        measures.push(measure(
            "imu",
            "IMU health and estimator_valid safety bit.",
            None,
        ));
    }
    DeviceReference {
        id: r.id.clone(),
        kind: "robot".into(),
        domain: r.domain.clone(),
        profile: PROFILE.into(),
        conformance: CONFORMANCE.into(),
        official: false,
        spec_url: SPEC_URL.into(),
        tags: robot_tags(r),
        measures,
        writes,
        legal_now: r
            .legal_cmds
            .iter()
            .map(|c| c.as_str().to_string())
            .collect(),
        safety,
        mass_kg: catalog_mass_kg(&r.id),
        radius_m: Some(r.radius_m),
    }
}

fn env_reference(obs: &Observation, limits: &DriverLimits) -> DeviceReference {
    DeviceReference {
        id: DEVICE_ENV.into(),
        kind: "environment".into(),
        domain: "environment".into(),
        profile: PROFILE.into(),
        conformance: CONFORMANCE.into(),
        official: false,
        spec_url: SPEC_URL.into(),
        tags: env_tags(),
        measures: vec![
            measure("identity", "Environment device.", None),
            measure("wind.ned", "Wind NED.", Some("m/s")),
            measure("current.ned", "Current NED.", Some("m/s")),
            measure("waves", "Amplitude, wavenumber, omega, phase.", None),
            measure("hydro", "Heightfield volume and grid.", None),
        ],
        writes: domain_writes("environment")
            .iter()
            .copied()
            .map(|c| write_cap(c, write_prose(c), true))
            .collect(),
        legal_now: obs
            .env_cmds
            .iter()
            .map(|c| c.as_str().to_string())
            .collect(),
        safety: vec![
            SafetyLimit {
                id: "wind".into(),
                prose: "Wind |NED| clamp.".into(),
                enforcement: "numeric".into(),
                remaining_spec: None,
                max: Some(limits.wind_mps),
                unit: Some("m/s".into()),
            },
            SafetyLimit {
                id: "current".into(),
                prose: "Current |NED| clamp.".into(),
                enforcement: "numeric".into(),
                remaining_spec: None,
                max: Some(limits.current_mps),
                unit: Some("m/s".into()),
            },
            SafetyLimit {
                id: "wave_amp".into(),
                prose: "Wave amplitude clamp (kernel also clamps).".into(),
                enforcement: "numeric".into(),
                remaining_spec: None,
                max: Some(limits.wave_amp_m),
                unit: Some("m".into()),
            },
        ],
        mass_kg: None,
        radius_m: None,
    }
}

fn lab_reference(_obs: &Observation, _limits: &DriverLimits) -> DeviceReference {
    DeviceReference {
        id: DEVICE_LAB.into(),
        kind: "lab".into(),
        domain: "lab".into(),
        profile: PROFILE.into(),
        conformance: CONFORMANCE.into(),
        official: false,
        spec_url: SPEC_URL.into(),
        tags: lab_tags(),
        measures: vec![
            measure("identity", "Scenario, seed, time.", None),
            measure("t", "World time.", Some("s")),
            measure("all_hold", "Whether the 22-property vector holds.", None),
            measure(
                "broken",
                "Failed property ids from the last try_step (atomic refuse).",
                None,
            ),
            measure("properties", "Named plant properties.", None),
            measure(
                "certificates",
                "Lab certificates such as fleet_hold_simultaneous.",
                None,
            ),
            measure("message", "Last lab message / reject text.", None),
        ],
        writes: vec![],
        legal_now: vec![],
        safety: vec![
            SafetyLimit {
                id: "p12".into(),
                prose: "Writes do not step. Chain/step is one WorldSession::step per tick.".into(),
                enforcement: "machine".into(),
                remaining_spec: Some("P12".into()),
                max: None,
                unit: None,
            },
            SafetyLimit {
                id: "catalog".into(),
                prose: "Inland has no hull; open_water has no rover.".into(),
                enforcement: "catalog".into(),
                remaining_spec: Some("P11".into()),
                max: None,
                unit: None,
            },
        ],
        mass_kg: None,
        radius_m: None,
    }
}

fn write_prose(cmd: LabCmd) -> &'static str {
    match cmd {
        LabCmd::Velocity => "Aerial NED velocity. OffboardControl only.",
        LabCmd::Position => "Aerial NED pose hold target. OffboardControl only.",
        LabCmd::Drive => "Ground NED twist. Moving only.",
        LabCmd::Thrust => "Marine NED velocity. CanThrust only.",
        LabCmd::Hold => "Current-pose NED hold for this domain.",
        LabCmd::Takeoff => "Attach takeoff grant (Ready/Armed/Offboard) or kernel Takeoff.",
        LabCmd::SetCharge => "Stored energy in joules (vn).",
        LabCmd::SetWind => "Wind NED (vn, ve, vd).",
        LabCmd::SetWaves => "Wave amp (vn), optional k (ve) and omega (vd).",
        LabCmd::SetCurrent => "Current NED.",
        LabCmd::Release => "Parked → Moving.",
        LabCmd::Undock => "Docked → Underway.",
        other => match other {
            LabCmd::Arm => "Arm the aerial machine.",
            LabCmd::Disarm => "Disarm (JSON Failsafe Disarm vs PX4 remains P6).",
            LabCmd::Offboard => "Armed → Offboard.",
            LabCmd::EnableActuators => "Enable motors when MotorsEnabled.",
            LabCmd::Airborne => "Takeoff → Airborne.",
            LabCmd::Land => "Begin land from Takeoff or Airborne.",
            LabCmd::Touchdown => "Landing or Failsafe → Ready.",
            LabCmd::Failsafe => "Trip the domain failsafe / E-stop.",
            LabCmd::Halt | LabCmd::Park => "Moving → Parked.",
            LabCmd::Estop => "Parked or Moving → E-stop.",
            LabCmd::Clear => "E-stop → Parked.",
            LabCmd::Dock => "Underway or StationKeep → Docked.",
            LabCmd::Station => "Underway → StationKeep.",
            LabCmd::Resume => "StationKeep → Underway.",
            LabCmd::Recover => "Domain recover path.",
            _ => "Lab command through attach / JSON fallback.",
        },
    }
}

/// Read one channel without stepping.
pub fn read_channel(
    obs: &Observation,
    device: &str,
    channel: &str,
) -> Result<ReadResult, MhsError> {
    let value = if device == DEVICE_LAB {
        read_lab(obs, channel)?
    } else if device == DEVICE_ENV {
        read_env(obs, channel)?
    } else {
        let robot = obs
            .robots
            .iter()
            .find(|r| r.id == device)
            .ok_or_else(|| MhsError::unknown_device(obs.scenario, device))?;
        read_robot(robot, channel)?
    };
    Ok(ReadResult {
        device: device.into(),
        channel: channel.into(),
        value,
        t: obs.t,
    })
}

fn unknown_channel(device: &str, channel: &str) -> MhsError {
    MhsError::UnknownChannel {
        device: device.into(),
        channel: channel.into(),
    }
}

fn read_lab(obs: &Observation, channel: &str) -> Result<serde_json::Value, MhsError> {
    match channel {
        "identity" => Ok(serde_json::json!({
            "id": DEVICE_LAB,
            "scenario": obs.scenario,
            "seed": obs.seed,
            "kind": "lab",
        })),
        "t" => Ok(serde_json::json!(obs.t)),
        "all_hold" => Ok(serde_json::json!(obs.all_hold)),
        "broken" => Ok(serde_json::to_value(&obs.broken).unwrap()),
        "properties" => Ok(serde_json::to_value(&obs.properties).unwrap()),
        "message" => Ok(serde_json::json!(obs.message)),
        "certificates" => {
            let drone_hold = obs
                .robots
                .iter()
                .find(|r| r.id == "drone")
                .map(|r| r.hold_ned.is_some())
                .unwrap_or(true);
            let skiff_ok = obs
                .robots
                .iter()
                .find(|r| r.id == "skiff")
                .map(|r| {
                    r.marine
                        .as_ref()
                        .is_some_and(|m| m.kind == MarineKind::StationKeep)
                })
                .unwrap_or(true);
            let mut ids = Vec::new();
            if drone_hold && skiff_ok {
                ids.push(FLEET_HOLD_SIMULTANEOUS);
            }
            Ok(serde_json::to_value(ids).unwrap())
        }
        _ => Err(unknown_channel(DEVICE_LAB, channel)),
    }
}

fn read_env(obs: &Observation, channel: &str) -> Result<serde_json::Value, MhsError> {
    let e = &obs.environment;
    match channel {
        "identity" => Ok(serde_json::json!({"id": DEVICE_ENV, "kind": "environment"})),
        "wind.ned" => Ok(serde_json::json!({
            "n": e.wind_ned[0], "e": e.wind_ned[1], "d": e.wind_ned[2], "frame": "ned"
        })),
        "current.ned" => Ok(serde_json::json!({
            "n": e.current_ned[0], "e": e.current_ned[1], "d": e.current_ned[2], "frame": "ned"
        })),
        "waves" => Ok(serde_json::json!({
            "amp": e.wave_amp,
            "phase": e.wave_phase,
        })),
        "hydro" => Ok(serde_json::json!({
            "volume": e.hydro_volume,
            "volume0": e.hydro_volume0,
            "gpu": e.hydro_gpu,
        })),
        _ => Err(unknown_channel(DEVICE_ENV, channel)),
    }
}

fn read_robot(r: &RobotView, channel: &str) -> Result<serde_json::Value, MhsError> {
    match channel {
        "identity" => Ok(serde_json::json!({
            "id": r.id,
            "domain": r.domain,
            "phase": r.phase,
            "kind": kind_of(r),
        })),
        "pose.ned" => Ok(serde_json::json!({
            "n": r.n, "e": r.e, "d": r.d, "yaw": r.yaw, "frame": "ned", "z": "down"
        })),
        "velocity.ned" => Ok(serde_json::json!({
            "vn": r.vn, "ve": r.ve, "vd": r.vd, "frame": "ned", "z": "down"
        })),
        "hold_ned" => Ok(serde_json::to_value(r.hold_ned).unwrap()),
        "charge" => Ok(serde_json::json!({
            "charge_j": r.charge_j, "capacity_j": r.capacity_j
        })),
        "legal_cmds" => Ok(serde_json::to_value(
            r.legal_cmds.iter().map(|c| c.as_str()).collect::<Vec<_>>(),
        )
        .unwrap()),
        "machine" => Ok(serde_json::json!({
            "domain": r.domain,
            "phase": r.phase,
            "kind": kind_of(r),
            "armed": r.armed,
            "failsafe": r.failsafe,
        })),
        "imu" => match &r.aerial {
            Some(a) => Ok(serde_json::json!({
                "imu_healthy": a.imu_healthy,
                "estimator_valid": a.estimator_valid,
            })),
            None => Err(unknown_channel(&r.id, channel)),
        },
        _ => Err(unknown_channel(&r.id, channel)),
    }
}

fn kind_of(r: &RobotView) -> serde_json::Value {
    if let Some(a) = &r.aerial {
        return serde_json::to_value(a.kind).unwrap_or(serde_json::Value::Null);
    }
    if let Some(g) = &r.ground {
        return serde_json::to_value(g.kind).unwrap_or(serde_json::Value::Null);
    }
    if let Some(m) = &r.marine {
        return serde_json::to_value(m.kind).unwrap_or(serde_json::Value::Null);
    }
    serde_json::Value::Null
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReadResult {
    pub device: String,
    pub channel: String,
    pub value: serde_json::Value,
    pub t: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WriteRequest {
    pub device: String,
    pub channel: String,
    #[serde(default)]
    pub vn: f32,
    #[serde(default)]
    pub ve: f32,
    #[serde(default)]
    pub vd: f32,
    #[serde(default)]
    pub yaw_rate: f32,
}

impl WriteRequest {
    pub fn new(device: impl Into<String>, channel: impl Into<String>) -> Self {
        Self {
            device: device.into(),
            channel: channel.into(),
            vn: 0.0,
            ve: 0.0,
            vd: 0.0,
            yaw_rate: 0.0,
        }
    }

    pub fn ned(mut self, vn: f32, ve: f32, vd: f32) -> Self {
        self.vn = vn;
        self.ve = ve;
        self.vd = vd;
        self
    }

    pub fn parse_cmd(&self) -> Result<LabCmd, MhsError> {
        LabCmd::ALL
            .into_iter()
            .find(|c| c.as_str() == self.channel)
            .ok_or_else(|| MhsError::UnknownChannel {
                device: self.device.clone(),
                channel: self.channel.clone(),
            })
    }

    pub fn to_action(&self, cmd: LabCmd) -> robot_lab::AgentAction {
        let robot = if LabCmd::ENV.contains(&cmd) {
            String::new()
        } else {
            self.device.clone()
        };
        robot_lab::AgentAction {
            robot,
            cmd,
            vn: self.vn,
            ve: self.ve,
            vd: self.vd,
            yaw_rate: self.yaw_rate,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WriteOk {
    pub ok: bool,
    pub device: String,
    pub channel: String,
    pub message: String,
}

/// Resolve the write channel and reject unknown devices / read-only / non-finite
/// values. Numeric maxima apply only when the command is legal now.
pub fn preview_write(
    obs: &Observation,
    req: &WriteRequest,
    limits: &DriverLimits,
) -> Result<LabCmd, MhsError> {
    if req.device == DEVICE_LAB {
        return Err(MhsError::ReadOnly {
            device: DEVICE_LAB.into(),
        });
    }
    let cmd = req.parse_cmd()?;
    if LabCmd::ENV.contains(&cmd) {
        if req.device != DEVICE_ENV {
            return Err(MhsError::UnknownChannel {
                device: req.device.clone(),
                channel: req.channel.clone(),
            });
        }
    } else if req.device == DEVICE_ENV {
        return Err(MhsError::UnknownChannel {
            device: req.device.clone(),
            channel: req.channel.clone(),
        });
    } else if !obs.robots.iter().any(|r| r.id == req.device) {
        return Err(MhsError::unknown_device(obs.scenario, req.device.clone()));
    }

    if !crate::limits::all_finite(&[req.vn, req.ve, req.vd, req.yaw_rate]) {
        return Err(MhsError::Limit(LimitReject::finite(
            &req.device,
            &req.channel,
        )));
    }

    let allowed = if LabCmd::ENV.contains(&cmd) {
        true
    } else {
        obs.tools().allows(&req.device, cmd) || takeoff_grant(obs, &req.device, cmd)
    };
    if !allowed {
        return Err(MhsError::NotLegal {
            device: req.device.clone(),
            cmd,
        });
    }
    if let Some(limit) = numeric_limit(obs, req, cmd, limits) {
        return Err(MhsError::Limit(limit));
    }
    Ok(cmd)
}

fn takeoff_grant(obs: &Observation, device: &str, cmd: LabCmd) -> bool {
    if cmd != LabCmd::Takeoff {
        return false;
    }
    obs.robots.iter().any(|r| {
        r.id == device
            && r.aerial.as_ref().is_some_and(|a| {
                matches!(
                    a.kind,
                    AerialKind::PreflightReady | AerialKind::Armed | AerialKind::Offboard
                )
            })
    })
}

fn numeric_limit(
    obs: &Observation,
    req: &WriteRequest,
    cmd: LabCmd,
    limits: &DriverLimits,
) -> Option<LimitReject> {
    use crate::limits::hypot3;
    let yaw = req.yaw_rate.abs();
    if matches!(
        cmd,
        LabCmd::Velocity | LabCmd::Drive | LabCmd::Thrust | LabCmd::Position | LabCmd::Hold
    ) && yaw > limits.yaw_rate_rps
    {
        return Some(LimitReject::over(
            "yaw_rate",
            &req.device,
            &req.channel,
            "yaw_rate exceeds driver limit",
            limits.yaw_rate_rps,
            yaw,
            "rad/s",
        ));
    }
    match cmd {
        LabCmd::Velocity => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.aerial_speed_mps).then(|| {
                LimitReject::over(
                    "ned_speed",
                    &req.device,
                    &req.channel,
                    "aerial |v| exceeds driver limit",
                    limits.aerial_speed_mps,
                    got,
                    "m/s",
                )
            })
        }
        LabCmd::Drive => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.ground_speed_mps).then(|| {
                LimitReject::over(
                    "ned_speed",
                    &req.device,
                    &req.channel,
                    "ground |v| exceeds driver limit",
                    limits.ground_speed_mps,
                    got,
                    "m/s",
                )
            })
        }
        LabCmd::Thrust => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.marine_speed_mps).then(|| {
                LimitReject::over(
                    "ned_speed",
                    &req.device,
                    &req.channel,
                    "marine |v| exceeds driver limit",
                    limits.marine_speed_mps,
                    got,
                    "m/s",
                )
            })
        }
        LabCmd::Position => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.position_m).then(|| {
                LimitReject::over(
                    "position",
                    &req.device,
                    &req.channel,
                    "aerial |pose| exceeds driver limit",
                    limits.position_m,
                    got,
                    "m",
                )
            })
        }
        LabCmd::SetCharge => {
            let cap = obs
                .robots
                .iter()
                .find(|r| r.id == req.device)
                .map(|r| r.capacity_j)
                .unwrap_or(0.0);
            if req.vn < 0.0 || req.vn > cap {
                Some(LimitReject::over(
                    "charge",
                    &req.device,
                    &req.channel,
                    "charge outside [0, capacity_j]",
                    cap,
                    req.vn,
                    "J",
                ))
            } else {
                None
            }
        }
        LabCmd::SetWind => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.wind_mps).then(|| {
                LimitReject::over(
                    "wind",
                    &req.device,
                    &req.channel,
                    "wind |NED| exceeds driver limit",
                    limits.wind_mps,
                    got,
                    "m/s",
                )
            })
        }
        LabCmd::SetCurrent => {
            let got = hypot3(req.vn, req.ve, req.vd);
            (got > limits.current_mps).then(|| {
                LimitReject::over(
                    "current",
                    &req.device,
                    &req.channel,
                    "current |NED| exceeds driver limit",
                    limits.current_mps,
                    got,
                    "m/s",
                )
            })
        }
        LabCmd::SetWaves => (req.vn < 0.0 || req.vn > limits.wave_amp_m).then(|| {
            LimitReject::over(
                "wave_amp",
                &req.device,
                &req.channel,
                "wave amplitude outside [0, max]",
                limits.wave_amp_m,
                req.vn,
                "m",
            )
        }),
        _ => None,
    }
}
