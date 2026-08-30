//! Adversarial vehicle scenarios against the same safety contract.
//!
//! In-process world, recorded traces, and (when a SITL/HITL log is supplied)
//! the same [`flight_core::contracts::evaluate_trace`] evaluator. This is not
//! a Gazebo replacement — it is differential conformance to the contract.

use flight_core::contracts::{
    evaluate_trace, parse_trace_jsonl, AerialOffboard, MonitorFail, Requirement, TraceSample,
};
use flight_core::frames::Ned;
use flight_core::safety::{Event, OFFBOARD_HEARTBEAT_MAX_AGE_MS};
use flight_core::temporal::{
    estimate_revoke_event, heartbeat_revoke_event, Estimate, Observation, Sequence,
    ESTIMATE_MAX_AGE_MS,
};
use flight_core::time::MonotonicInstant;
use flight_core::vector::Velocity;
use flight_core::vehicle::{Offboard, Vehicle, VehicleBackend, VehicleHandle};
use robot_world::World;

use super::world_backend::{WorldBackend, WorldSession};

/// Fault injected at a simulation time.
#[derive(Clone, Copy, Debug)]
pub enum Fault {
    GpsDropout { at_secs: f32 },
    HeartbeatStale { at_secs: f32 },
    Failsafe { at_secs: f32 },
    BatterySag { at_secs: f32, percent: u8 },
    WindGust { at_secs: f32, north: f32, east: f32 },
    ImuUnhealthy { at_secs: f32 },
    ImuDelay { at_secs: f32, delay_ms: u32 },
    MotorEfficiency { at_secs: f32, percent: u8 },
}

impl Fault {
    pub const fn at_secs(self) -> f32 {
        match self {
            Self::GpsDropout { at_secs }
            | Self::HeartbeatStale { at_secs }
            | Self::Failsafe { at_secs }
            | Self::BatterySag { at_secs, .. }
            | Self::WindGust { at_secs, .. }
            | Self::ImuUnhealthy { at_secs }
            | Self::ImuDelay { at_secs, .. }
            | Self::MotorEfficiency { at_secs, .. } => at_secs,
        }
    }

    fn kernel_event(self) -> Option<Event> {
        let candidate = match self {
            Self::GpsDropout { .. } => Event::EstimatorInvalid,
            Self::HeartbeatStale { .. } => Event::HeartbeatStale,
            Self::Failsafe { .. } => Event::TriggerFailsafe,
            Self::ImuUnhealthy { .. } => Event::ImuUnhealthy,
            Self::ImuDelay { delay_ms, .. } => estimate_revoke_event(delay_ms)?,
            Self::BatterySag { .. } | Self::WindGust { .. } | Self::MotorEfficiency { .. } => {
                return None
            }
        };
        AerialOffboard::inject(candidate)
    }
}

/// Named scenario: vehicle catalog + faults + contract requirements.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    pub catalog: &'static str,
    pub seed: u64,
    pub dt: f32,
    pub duration_secs: f32,
    pub inject: &'static [Fault],
    pub require: &'static [Requirement],
}

impl Scenario {
    /// GPS loss must latch failsafe and never actuate while disarmed.
    pub const GPS_LOSS: Self = Self {
        name: "gps-loss",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 1.0,
        inject: &[
            Fault::GpsDropout { at_secs: 0.2 },
            Fault::WindGust {
                at_secs: 0.2,
                north: 12.0,
                east: 0.0,
            },
        ],
        require: AerialOffboard::GPS_LOSS_REQUIRE,
    };

    /// Offboard heartbeat loss must latch failsafe.
    pub const HEARTBEAT_LOSS: Self = Self {
        name: "heartbeat-stale",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.6,
        inject: &[Fault::HeartbeatStale { at_secs: 0.1 }],
        require: AerialOffboard::EPOCH_REVOKE_REQUIRE,
    };

    /// Deadline miss / attach failsafe: same epoch revoke as the HITL rack.
    pub const HITL_MISS: Self = Self {
        name: "hitl-miss",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.4,
        inject: &[Fault::Failsafe { at_secs: 0.1 }],
        require: AerialOffboard::EPOCH_REVOKE_REQUIRE,
    };

    /// IMU health bit clear: same leftover Offboard story as heartbeat loss.
    pub const IMU_LOSS: Self = Self {
        name: "imu-loss",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.6,
        inject: &[Fault::ImuUnhealthy { at_secs: 0.1 }],
        require: AerialOffboard::EPOCH_REVOKE_REQUIRE,
    };

    /// IMU transport delay ≥ [`ESTIMATE_MAX_AGE_MS`] is a stale Estimate.
    pub const IMU_DELAY: Self = Self {
        name: "imu-delay",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 1.0,
        inject: &[Fault::ImuDelay {
            at_secs: 0.2,
            delay_ms: ESTIMATE_MAX_AGE_MS.saturating_add(50),
        }],
        require: AerialOffboard::GPS_LOSS_REQUIRE,
    };

    /// Motor efficiency is plant-only. It does not bump the safety epoch.
    pub const MOTOR_EFFICIENCY: Self = Self {
        name: "motor-efficiency",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.6,
        inject: &[Fault::MotorEfficiency {
            at_secs: 0.1,
            percent: 0,
        }],
        require: &[
            Requirement::NeverActuateWhileDisarmed,
            Requirement::ActuatorsImplyArmed,
            Requirement::NoNanCommands,
            Requirement::AltitudeBelow { meters: 120.0 },
        ],
    };

    pub const ALL: &'static [Self] = &[
        Self::GPS_LOSS,
        Self::HEARTBEAT_LOSS,
        Self::HITL_MISS,
        Self::IMU_LOSS,
        Self::IMU_DELAY,
        Self::MOTOR_EFFICIENCY,
    ];

    pub fn by_name(name: &str) -> Option<&'static Self> {
        Self::ALL.iter().find(|s| s.name == name)
    }
}

/// Declare a named scenario. Expands to a [`Scenario`] value.
#[macro_export]
macro_rules! scenario {
    (
        name: $name:expr,
        catalog: $cat:expr,
        inject: [$($fault:expr),* $(,)?],
        require: [$($req:expr),* $(,)?] $(,)?
    ) => {
        $crate::Scenario {
            name: $name,
            catalog: $cat,
            seed: 1,
            dt: 0.02,
            duration_secs: 1.0,
            inject: &[$($fault),*],
            require: &[$($req),*],
        }
    };
}

/// One recorded run of [`Scenario`] against a backend kind.
#[derive(Clone, Debug)]
pub struct ScenarioReport {
    pub name: &'static str,
    pub backend: &'static str,
    pub samples: Vec<TraceSample>,
}

impl ScenarioReport {
    pub fn evaluate(&self, reqs: &[Requirement]) -> Result<(), MonitorFail> {
        evaluate_trace(&self.samples, reqs)
    }

    /// Same capability monitors as [`flight_core::contracts::AerialOffboard::MONITORS`].
    pub fn evaluate_capability(&self) -> Result<(), MonitorFail> {
        AerialOffboard::evaluate(&self.samples)
    }

    pub fn to_jsonl(&self) -> String {
        let mut out = String::new();
        for s in &self.samples {
            let cmd = match s.command {
                Some(c) => format!("[{},{},{}]", c[0], c[1], c[2]),
                None => "null".into(),
            };
            out.push_str(&format!(
                "{{\"t\":{},\"armed\":{},\"actuators\":{},\"failsafe\":{},\"epoch\":{},\"alt\":{},\"heartbeat_age_ms\":{},\"command_age_ms\":{},\"estimator_ts_ms\":{},\"cmd\":{cmd}}}\n",
                s.t_secs, s.armed, s.actuators_enabled, s.failsafe, s.epoch, s.altitude_m,
                s.heartbeat_age_ms, s.command_age_ms, s.estimator_ts_ms
            ));
        }
        out
    }
}

/// Run `scenario` on the verified in-process world after an offboard grant.
pub fn run_world(scenario: &Scenario) -> Result<ScenarioReport, String> {
    let session = WorldSession::named(scenario.catalog, scenario.seed)
        .ok_or_else(|| format!("unknown catalog {}", scenario.catalog))?;
    session
        .attach_offboard("drone")
        .map_err(|e| format!("grant: {e}"))?;
    let mut injected = vec![false; scenario.inject.len()];
    let mut samples = Vec::new();
    let steps = (scenario.duration_secs / scenario.dt).ceil() as u32;
    for k in 0..steps {
        let t = (k as f32) * scenario.dt;
        for (i, fault) in scenario.inject.iter().enumerate() {
            if !injected[i] && t + 1e-6 >= fault.at_secs() {
                apply_fault(&session, *fault).map_err(|e| format!("inject: {e}"))?;
                injected[i] = true;
            }
        }
        session
            .step(scenario.dt)
            .map_err(|e| format!("step: {e}"))?;
        samples.push(sample_drone(&session.world()));
    }
    Ok(ScenarioReport {
        name: scenario.name,
        backend: "world",
        samples,
    })
}

/// HITL-shaped miss: grant, then attach failsafe (the rack miss path).
pub fn run_hitl_miss() -> Result<ScenarioReport, String> {
    let session =
        WorldSession::named("inland", 1).ok_or_else(|| "unknown catalog inland".to_string())?;
    session
        .attach_takeoff("drone")
        .map_err(|e| format!("grant: {e}"))?;
    let mut samples = vec![sample_drone(&session.world())];
    session
        .attach_failsafe("drone")
        .map_err(|e| format!("miss: {e}"))?;
    samples.push(sample_drone(&session.world()));
    Ok(ScenarioReport {
        name: "hitl-miss",
        backend: "hitl",
        samples,
    })
}

/// Inject every DSL revoke event from offboard. Fault-injection generated
/// from [`flight_core::contracts::AerialOffboard::REVOKE_ON`] /
/// [`AerialOffboard::inject`]. A leftover `Vehicle<Offboard>` bound before
/// the inject must refuse every [`AerialOffboard::COMMANDS`] method.
pub fn run_revoke_table() -> Result<ScenarioReport, String> {
    let mut samples = Vec::new();
    for (i, e) in AerialOffboard::REVOKE_ON.iter().enumerate() {
        let inject = AerialOffboard::inject(*e)
            .ok_or_else(|| format!("{e:?} is in REVOKE_ON but inject returned None"))?;
        let session =
            WorldSession::named("inland", 1).ok_or_else(|| "unknown catalog inland".to_string())?;
        session
            .attach_offboard("drone")
            .map_err(|err| format!("grant: {e:?}: {err}"))?;
        let VehicleHandle::Offboard(mut v) = session
            .aerial("drone")
            .attach()
            .map_err(|err| format!("bind Offboard before {e:?}: {err}"))?
        else {
            return Err(format!("attach_offboard must bind Offboard before {e:?}"));
        };
        v.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -0.2))
            .map_err(|err| format!("live set_velocity before {e:?}: {err}"))?;
        let mut seq = Sequence::new();
        seq.observe(v.backend().authority_epoch())
            .map_err(|_| format!("sequence before {e:?}"))?;
        session
            .inject_revoke("drone", inject)
            .map_err(|err| format!("inject {e:?}: {err}"))?;
        seq.observe(v.backend().authority_epoch())
            .map_err(|_| format!("epoch jumped backward after {e:?}"))?;
        leftover_offboard_refuses_commands(&mut v, inject)?;
        // Step after leftover checks so P13 can wipe an ungranted command
        // before the contract sample (Disarm/Disconnect do not clear it).
        session.step(0.02).map_err(|err| format!("step: {err}"))?;
        let mut s = sample_drone(&session.world());
        s.t_secs = (i as f32) * 0.02;
        if s.epoch == 0 {
            return Err(format!("event {e:?} did not bump epoch"));
        }
        samples.push(s);
    }
    Ok(ScenarioReport {
        name: "revoke-table",
        backend: "world",
        samples,
    })
}

/// Leftover typestate stays Offboard. Every kernel command is exercised
/// through [`Vehicle::leftover_commands_stale`] (unknown table command = compile fail).
fn leftover_offboard_refuses_commands(
    v: &mut Vehicle<Offboard, WorldBackend>,
    event: Event,
) -> Result<(), String> {
    v.leftover_commands_stale()
        .map_err(|e| format!("leftover after {event:?}: {e}"))
}

/// Contract requirements that a concatenated revoke-table trace can prove.
/// `EpochBumped` is not here: each event is a separate session (typically
/// epoch 1), so the concatenated samples do not show a bump inside one run.
pub const REVOKE_TABLE_REQUIRE: &[Requirement] = &[
    Requirement::NeverActuateWhileDisarmed,
    Requirement::ActuatorsImplyArmed,
    Requirement::NoNanCommands,
];

/// Same leftover Offboard runner, then JSONL replay and ULog round-trip.
/// Differential conformance of the generated fault table, not a second plant.
pub fn differential_revoke_table() -> Result<ScenarioReport, String> {
    let report = run_revoke_table()?;
    report
        .evaluate(REVOKE_TABLE_REQUIRE)
        .map_err(|e| format!("world: {} at {}", e.requirement, e.index))?;
    report
        .evaluate_capability()
        .map_err(|e| format!("world capability: {} at {}", e.requirement, e.index))?;
    replay_jsonl(&report.to_jsonl(), REVOKE_TABLE_REQUIRE)
        .map_err(|e| format!("jsonl: {} at {}", e.requirement, e.index))?;
    AerialOffboard::evaluate(
        &parse_trace_jsonl(&report.to_jsonl()).map_err(|_| "jsonl parse".to_string())?,
    )
    .map_err(|e| format!("jsonl capability: {} at {}", e.requirement, e.index))?;
    let ulog_bytes = crate::write_ulog(&report.samples);
    let from_ulog = crate::parse_ulog(&ulog_bytes).map_err(|e| format!("ulog roundtrip: {e}"))?;
    evaluate_trace(&from_ulog, REVOKE_TABLE_REQUIRE)
        .map_err(|e| format!("ulog roundtrip: {} at {}", e.requirement, e.index))?;
    AerialOffboard::evaluate(&from_ulog)
        .map_err(|e| format!("ulog capability: {} at {}", e.requirement, e.index))?;
    Ok(report)
}

/// Re-evaluate a previously recorded report (replay / ulog-shaped JSONL is
/// converted into [`TraceSample`] by the caller).
pub fn replay_report(report: &ScenarioReport, reqs: &[Requirement]) -> Result<(), MonitorFail> {
    report.evaluate(reqs)
}

/// Two runs of the same scenario must agree on failsafe time and epoch
/// monotonicity — differential conformance, not a high-fidelity world.
pub fn differential_world(scenario: &Scenario) -> Result<(), String> {
    let a = run_world(scenario)?;
    let b = run_world(scenario)?;
    if a.samples.len() != b.samples.len() {
        return Err("sample count mismatch".into());
    }
    for (i, (x, y)) in a.samples.iter().zip(b.samples.iter()).enumerate() {
        if (x.failsafe != y.failsafe) || (x.epoch != y.epoch) {
            return Err(format!("divergence at sample {i}"));
        }
    }
    replay_report(&a, scenario.require).map_err(|e| format!("contract: {}", e.requirement))?;
    Ok(())
}

/// Same safety contract on the verified world, JSONL replay, and a ULog
/// round-trip. Named leftover contracts also evaluate the converted PX4 SITL
/// corpus (`px4_sitl_<name>.jsonl`). Differential conformance, not a second physics.
pub fn differential_contract(scenario: &Scenario) -> Result<(), String> {
    let reqs = scenario.require;
    let world = run_world(scenario)?;
    world
        .evaluate(reqs)
        .map_err(|e| format!("world: {} at {}", e.requirement, e.index))?;
    world
        .evaluate_capability()
        .map_err(|e| format!("world capability: {} at {}", e.requirement, e.index))?;
    differential_world(scenario)?;
    replay_jsonl(&world.to_jsonl(), reqs)
        .map_err(|e| format!("jsonl: {} at {}", e.requirement, e.index))?;
    let ulog_bytes = crate::write_ulog(&world.samples);
    let from_ulog = crate::parse_ulog(&ulog_bytes).map_err(|e| format!("ulog roundtrip: {e}"))?;
    evaluate_trace(&from_ulog, reqs)
        .map_err(|e| format!("ulog roundtrip: {} at {}", e.requirement, e.index))?;
    AerialOffboard::evaluate(&from_ulog)
        .map_err(|e| format!("ulog capability: {} at {}", e.requirement, e.index))?;
    if scenario.name == Scenario::GPS_LOSS.name {
        let ulog = crate::parse_ulog(include_bytes!("../corpus/gps_loss.ulg"))
            .map_err(|e| format!("ulog: {e}"))?;
        evaluate_trace(&ulog, reqs)
            .map_err(|e| format!("ulog: {} at {}", e.requirement, e.index))?;
        AerialOffboard::evaluate(&ulog)
            .map_err(|e| format!("ulog capability: {} at {}", e.requirement, e.index))?;
    }
    if let Some(sitl) = px4_sitl_leftover_corpus(scenario.name) {
        let sitl = parse_trace_jsonl(sitl).map_err(|_| "px4-sitl: parse_jsonl".to_string())?;
        evaluate_trace(&sitl, reqs)
            .map_err(|e| format!("px4-sitl: {} at {}", e.requirement, e.index))?;
        AerialOffboard::evaluate(&sitl)
            .map_err(|e| format!("px4-sitl capability: {} at {}", e.requirement, e.index))?;
    }
    Ok(())
}

/// Converted PX4 SITL JSONL for a named leftover contract.
/// `flight-sim` does not depend on `flight-px4`; live SIH is `sitl_live`.
pub fn px4_sitl_leftover_corpus(name: &str) -> Option<&'static str> {
    match name {
        "gps-loss" => Some(include_str!("../corpus/px4_sitl_gps_loss.jsonl")),
        "heartbeat-stale" => Some(include_str!("../corpus/px4_sitl_heartbeat_stale.jsonl")),
        "hitl-miss" => Some(include_str!("../corpus/px4_sitl_hitl_miss.jsonl")),
        "imu-loss" => Some(include_str!("../corpus/px4_sitl_imu_loss.jsonl")),
        _ => None,
    }
}

/// Same GPS-loss contract on the verified world, checked-in ULog, and
/// converted PX4 SITL JSONL.
pub fn differential_gps_loss() -> Result<(), String> {
    differential_contract(&Scenario::GPS_LOSS)
}

/// Evaluate a previously recorded JSONL (ulog-shaped conversion or
/// [`ScenarioReport::to_jsonl`]) against `reqs`.
pub fn replay_jsonl(text: &str, reqs: &[Requirement]) -> Result<(), MonitorFail> {
    let samples = parse_trace_jsonl(text).map_err(|_| MonitorFail {
        requirement: "parse_jsonl",
        index: 0,
    })?;
    evaluate_trace(&samples, reqs)
}

fn inject_revoke(session: &WorldSession, e: Event) -> Result<(), String> {
    session
        .inject_revoke("drone", e)
        .map_err(|err| format!("{e:?}: {err}"))
}

fn apply_fault(session: &WorldSession, fault: Fault) -> Result<(), String> {
    match fault {
        Fault::GpsDropout { .. } => {
            let gps = Observation::<(), Ned>::new((), MonotonicInstant::ZERO);
            let est = Estimate::new(gps, false, gps.stamped_at);
            if let Some(e) = est.revoke_event() {
                inject_revoke(session, e)?;
            }
        }
        Fault::HeartbeatStale { .. } => {
            if let Some(e) = heartbeat_revoke_event(OFFBOARD_HEARTBEAT_MAX_AGE_MS) {
                inject_revoke(session, e)?;
            }
        }
        Fault::Failsafe { .. } => {
            inject_revoke(
                session,
                fault
                    .kernel_event()
                    .expect("failsafe is a kernel revoke event"),
            )?;
        }
        Fault::BatterySag { percent, .. } => {
            session.with_world_mut(|w| {
                if let Some(b) = w.body_mut("drone") {
                    let drop = b.charge_j * (f32::from(percent) / 100.0);
                    b.charge_j = (b.charge_j - drop).max(0.0);
                }
            });
        }
        Fault::WindGust { north, east, .. } => {
            session.with_world_mut(|w| {
                w.env.wind_ned[0] += north;
                w.env.wind_ned[1] += east;
            });
        }
        Fault::ImuUnhealthy { .. } => {
            inject_revoke(
                session,
                fault
                    .kernel_event()
                    .expect("imu unhealthy is a kernel revoke event"),
            )?;
        }
        Fault::ImuDelay { delay_ms, .. } => {
            session.with_world_mut(|w| {
                if let Some(b) = w.body_mut("drone") {
                    b.imu_delay_ms = delay_ms;
                }
            });
            if let Some(e) = estimate_revoke_event(delay_ms) {
                inject_revoke(session, e)?;
            }
        }
        Fault::MotorEfficiency { percent, .. } => {
            session.with_world_mut(|w| {
                if let Some(b) = w.body_mut("drone") {
                    b.thrust_scale = (f32::from(percent) / 100.0).clamp(0.0, 1.0);
                }
            });
        }
    }
    Ok(())
}

fn sample_drone(world: &World) -> TraceSample {
    let body = world.body("drone").expect("drone");
    let aerial = body.aerial.expect("aerial");
    TraceSample {
        t_secs: world.t,
        armed: aerial.armed,
        actuators_enabled: aerial.actuators_enabled,
        failsafe: aerial.failsafe,
        epoch: body.authority_epoch,
        heartbeat_age_ms: if aerial.offboard_heartbeat_fresh {
            0
        } else {
            OFFBOARD_HEARTBEAT_MAX_AGE_MS
        },
        command: body.command,
        altitude_m: body.altitude_agl(),
        command_age_ms: 0,
        estimator_ts_ms: body.last_estimator_ts_ms,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::contracts::AerialOffboard;

    #[test]
    fn gps_loss_world_satisfies_contract() {
        let report = run_world(&Scenario::GPS_LOSS).expect("run");
        assert!(core::ptr::eq(
            Scenario::GPS_LOSS.require,
            AerialOffboard::GPS_LOSS_REQUIRE
        ));
        assert!(core::ptr::eq(
            Scenario::IMU_DELAY.require,
            AerialOffboard::GPS_LOSS_REQUIRE
        ));
        assert_eq!(
            Scenario::GPS_LOSS.name,
            AerialOffboard::GPS_LOSS_CONTRACT.name
        );
        assert!(core::ptr::eq(
            Scenario::HEARTBEAT_LOSS.require,
            AerialOffboard::EPOCH_REVOKE_REQUIRE
        ));
        assert_eq!(
            Scenario::HEARTBEAT_LOSS.name,
            AerialOffboard::HEARTBEAT_LOSS_CONTRACT.name
        );
        assert!(core::ptr::eq(
            Scenario::HITL_MISS.require,
            AerialOffboard::EPOCH_REVOKE_REQUIRE
        ));
        assert_eq!(
            Scenario::HITL_MISS.name,
            AerialOffboard::HITL_MISS_CONTRACT.name
        );
        assert!(core::ptr::eq(
            Scenario::IMU_LOSS.require,
            AerialOffboard::EPOCH_REVOKE_REQUIRE
        ));
        assert_eq!(
            Scenario::IMU_LOSS.name,
            AerialOffboard::IMU_LOSS_CONTRACT.name
        );
        assert!(report.samples.iter().any(|s| s.failsafe));
        assert!(report.samples.iter().any(|s| s.epoch > 0));
        report
            .evaluate(Scenario::GPS_LOSS.require)
            .expect("contract");
        report.evaluate_capability().expect("capability monitors");
        differential_world(&Scenario::GPS_LOSS).expect("differential");
        let jsonl = report.to_jsonl();
        assert!(jsonl.contains("failsafe"));
        replay_jsonl(&jsonl, Scenario::GPS_LOSS.require).expect("jsonl replay");
        let ulog = crate::write_ulog(&report.samples);
        let from_ulog = crate::parse_ulog(&ulog).expect("ulog");
        assert_eq!(from_ulog.len(), report.samples.len());
        crate::replay_report(
            &ScenarioReport {
                name: report.name,
                backend: "ulog",
                samples: from_ulog,
            },
            Scenario::GPS_LOSS.require,
        )
        .expect("ulog replay");
    }

    #[test]
    fn gps_loss_same_contract_on_world_ulog_and_sitl() {
        differential_gps_loss().expect("world + ulog + px4-sitl");
    }

    #[test]
    fn leftover_contracts_have_px4_sitl_corpora() {
        for c in AerialOffboard::LEFTOVER_CONTRACTS {
            let sitl = px4_sitl_leftover_corpus(c.name)
                .unwrap_or_else(|| panic!("{} missing px4-sitl corpus", c.name));
            let samples = parse_trace_jsonl(sitl).expect("parse leftover sitl corpus");
            evaluate_trace(&samples, c.require)
                .unwrap_or_else(|e| panic!("{} sitl {} at {}", c.name, e.requirement, e.index));
            AerialOffboard::evaluate(&samples).unwrap_or_else(|e| {
                panic!(
                    "{} sitl capability {} at {}",
                    c.name, e.requirement, e.index
                )
            });
        }
    }

    #[test]
    fn differential_contract_on_every_named_scenario() {
        for s in Scenario::ALL {
            differential_contract(s).unwrap_or_else(|e| panic!("{}: {e}", s.name));
        }
    }

    #[test]
    fn gps_dropout_is_an_invalid_estimate() {
        let gps = Observation::<(), Ned>::new((), MonotonicInstant::ZERO);
        let est = Estimate::new(gps, false, gps.stamped_at);
        assert_eq!(
            est.revoke_event(),
            Fault::GpsDropout { at_secs: 0.0 }.kernel_event()
        );
        assert_eq!(
            Fault::GpsDropout { at_secs: 0.0 }.kernel_event(),
            AerialOffboard::inject(Event::EstimatorInvalid)
        );
        assert_eq!(
            heartbeat_revoke_event(OFFBOARD_HEARTBEAT_MAX_AGE_MS),
            Fault::HeartbeatStale { at_secs: 0.0 }.kernel_event()
        );
        assert_eq!(
            Fault::HeartbeatStale { at_secs: 0.0 }.kernel_event(),
            AerialOffboard::inject(Event::HeartbeatStale)
        );
        assert_eq!(
            Fault::Failsafe { at_secs: 0.0 }.kernel_event(),
            AerialOffboard::inject(Event::TriggerFailsafe)
        );
        assert!(Fault::BatterySag {
            at_secs: 0.0,
            percent: 10
        }
        .kernel_event()
        .is_none());
        assert!(Fault::MotorEfficiency {
            at_secs: 0.0,
            percent: 0
        }
        .kernel_event()
        .is_none());
        assert!(Fault::WindGust {
            at_secs: 0.0,
            north: 1.0,
            east: 0.0
        }
        .kernel_event()
        .is_none());
        assert_eq!(
            Fault::ImuUnhealthy { at_secs: 0.0 }.kernel_event(),
            AerialOffboard::inject(Event::ImuUnhealthy)
        );
        assert_eq!(
            Fault::ImuDelay {
                at_secs: 0.0,
                delay_ms: ESTIMATE_MAX_AGE_MS
            }
            .kernel_event(),
            AerialOffboard::inject(Event::EstimatorInvalid)
        );
        assert!(Fault::ImuDelay {
            at_secs: 0.0,
            delay_ms: ESTIMATE_MAX_AGE_MS.saturating_sub(1)
        }
        .kernel_event()
        .is_none());
        assert!(heartbeat_revoke_event(0).is_none());
    }

    #[test]
    fn gps_loss_revokes_position_control_authority() {
        use flight_core::vector::Position;
        use flight_core::vehicle::VehicleHandle;

        let session = WorldSession::inland(1);
        session.attach_offboard("drone").expect("grant");
        let VehicleHandle::Offboard(mut v) = session.aerial("drone").attach().unwrap() else {
            panic!("attach_offboard must bind Offboard");
        };
        v.set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
            .unwrap();
        apply_fault(&session, Fault::GpsDropout { at_secs: 0.0 }).expect("gps dropout");
        assert!(session.world().body("drone").unwrap().authority_epoch > 0);
        leftover_offboard_refuses_commands(&mut v, Event::EstimatorInvalid)
            .expect("leftover Offboard after gps-loss");
    }

    #[test]
    fn imu_delay_revokes_position_control_authority() {
        use flight_core::vector::Position;
        use flight_core::vehicle::VehicleHandle;

        let session = WorldSession::inland(1);
        session.attach_offboard("drone").expect("grant");
        let VehicleHandle::Offboard(mut v) = session.aerial("drone").attach().unwrap() else {
            panic!("attach_offboard must bind Offboard");
        };
        v.set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
            .unwrap();
        apply_fault(
            &session,
            Fault::ImuDelay {
                at_secs: 0.0,
                delay_ms: ESTIMATE_MAX_AGE_MS,
            },
        )
        .expect("imu delay");
        assert!(session.world().body("drone").unwrap().authority_epoch > 0);
        leftover_offboard_refuses_commands(&mut v, Event::EstimatorInvalid)
            .expect("leftover Offboard after imu-delay");
    }

    #[test]
    fn imu_unhealthy_revokes_leftover_offboard() {
        use flight_core::vehicle::VehicleHandle;

        let session = WorldSession::inland(1);
        session.attach_offboard("drone").expect("grant");
        let VehicleHandle::Offboard(mut v) = session.aerial("drone").attach().unwrap() else {
            panic!("attach_offboard must bind Offboard");
        };
        v.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -0.2))
            .unwrap();
        apply_fault(&session, Fault::ImuUnhealthy { at_secs: 0.0 }).expect("imu unhealthy");
        leftover_offboard_refuses_commands(&mut v, Event::ImuUnhealthy)
            .expect("leftover Offboard after imu-loss");
        assert!(session.world().body("drone").unwrap().authority_epoch > 0);
    }

    #[test]
    fn reconnect_revokes_leftover_offboard() {
        use flight_core::vehicle::{VehicleBackend, VehicleHandle};

        let session = WorldSession::inland(1);
        session.attach_offboard("drone").expect("grant");
        let VehicleHandle::Offboard(mut v) = session.aerial("drone").attach().unwrap() else {
            panic!("attach_offboard must bind Offboard");
        };
        v.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -0.2))
            .unwrap();
        v.backend_mut().connect_now().expect("reconnect");
        leftover_offboard_refuses_commands(&mut v, Event::Connect)
            .expect("leftover Offboard after reconnect");
        assert!(session.world().body("drone").unwrap().authority_epoch > 0);
    }

    #[test]
    fn heartbeat_loss_world_satisfies_contract() {
        let report = run_world(&Scenario::HEARTBEAT_LOSS).expect("run");
        assert!(report.samples.iter().any(|s| s.failsafe));
        report
            .evaluate(Scenario::HEARTBEAT_LOSS.require)
            .expect("contract");
        report.evaluate_capability().expect("capability monitors");
    }

    #[test]
    fn hitl_miss_world_and_attach_share_contract() {
        let world = run_world(&Scenario::HITL_MISS).expect("world");
        world
            .evaluate(Scenario::HITL_MISS.require)
            .expect("world contract");
        world.evaluate_capability().expect("world capability");
        differential_world(&Scenario::HITL_MISS).expect("differential");
        let hitl = run_hitl_miss().expect("hitl");
        assert!(hitl.samples.iter().any(|s| s.failsafe));
        assert!(hitl.samples.iter().any(|s| s.epoch > 0));
        hitl.evaluate(Scenario::HITL_MISS.require)
            .expect("hitl contract");
        hitl.evaluate_capability().expect("hitl capability");
    }

    #[test]
    fn imu_loss_world_satisfies_contract() {
        let report = run_world(&Scenario::IMU_LOSS).expect("run");
        assert!(report.samples.iter().any(|s| s.failsafe));
        assert!(report.samples.iter().any(|s| s.epoch > 0));
        report
            .evaluate(Scenario::IMU_LOSS.require)
            .expect("contract");
        report.evaluate_capability().expect("capability monitors");
    }

    #[test]
    fn imu_delay_world_stamps_lag_and_satisfy_contract() {
        let report = run_world(&Scenario::IMU_DELAY).expect("run");
        assert!(report.samples.iter().any(|s| s.failsafe));
        assert!(report.samples.iter().any(|s| s.epoch > 0));
        report
            .evaluate(Scenario::IMU_DELAY.require)
            .expect("contract");
        report.evaluate_capability().expect("capability monitors");
        let late = report
            .samples
            .iter()
            .filter(|s| s.t_secs + 1e-6 >= 0.5)
            .count();
        assert!(late > 0);
        for s in report.samples.iter().filter(|s| s.t_secs + 1e-6 >= 0.5) {
            let raw = (s.t_secs * 1000.0) as u64;
            assert!(
                s.estimator_ts_ms + u64::from(ESTIMATE_MAX_AGE_MS) <= raw + 1,
                "t={} stamp={} raw={}",
                s.t_secs,
                s.estimator_ts_ms,
                raw
            );
        }
    }

    #[test]
    fn motor_efficiency_does_not_bump_epoch() {
        let report = run_world(&Scenario::MOTOR_EFFICIENCY).expect("run");
        assert!(report.samples.iter().all(|s| s.epoch == 0));
        assert!(report.samples.iter().all(|s| !s.failsafe));
        report
            .evaluate(Scenario::MOTOR_EFFICIENCY.require)
            .expect("contract");
        report.evaluate_capability().expect("capability monitors");
    }

    #[test]
    fn named_scenarios_cover_imu_and_motor_faults() {
        assert_eq!(Scenario::ALL.len(), 6);
        assert_eq!(Scenario::by_name("imu-loss").unwrap().name, "imu-loss");
        assert_eq!(Scenario::by_name("imu-delay").unwrap().name, "imu-delay");
        assert_eq!(
            Scenario::by_name("motor-efficiency").unwrap().name,
            "motor-efficiency"
        );
    }

    #[test]
    fn revoke_table_faults_are_the_dsl_events() {
        let report = differential_revoke_table().expect("revoke table");
        assert_eq!(
            report.samples.len(),
            flight_core::contracts::AerialOffboard::REVOKE_ON.len()
        );
        assert_eq!(REVOKE_TABLE_REQUIRE.len(), 3);
    }

    #[test]
    fn each_dsl_revoke_event_bumps_epoch_from_offboard() {
        for e in AerialOffboard::REVOKE_ON {
            let inject = AerialOffboard::inject(*e).expect("REVOKE_ON is injectable");
            let session = WorldSession::named("inland", 1).expect("catalog");
            session.attach_offboard("drone").expect("grant");
            session
                .inject_revoke("drone", inject)
                .unwrap_or_else(|err| panic!("inject {e:?}: {err}"));
            assert!(
                session.world().body("drone").unwrap().authority_epoch > 0,
                "event {e:?} must bump authority_epoch"
            );
        }
    }

    #[test]
    fn scenario_macro_builds_a_gps_loss_shaped_value() {
        let s = crate::scenario! {
            name: "gps-loss-macro",
            catalog: "inland",
            inject: [Fault::GpsDropout { at_secs: 0.2 }],
            require: [Requirement::NeverActuateWhileDisarmed],
        };
        assert_eq!(s.name, "gps-loss-macro");
        assert_eq!(Scenario::by_name("gps-loss").unwrap().name, "gps-loss");
    }

    #[test]
    fn px4_sitl_converter_corpus_satisfies_gps_loss_contract() {
        let jsonl = include_str!("../corpus/px4_sitl_gps_loss.jsonl");
        replay_jsonl(jsonl, Scenario::GPS_LOSS.require).expect("sitl corpus");
    }
}
