//! Adversarial vehicle scenarios against the same safety contract.
//!
//! In-process world, recorded traces, and (when a SITL/HITL log is supplied)
//! the same [`flight_core::contracts::evaluate_trace`] evaluator. This is not
//! a Gazebo replacement — it is differential conformance to the contract.

use flight_core::contracts::{
    evaluate_trace, parse_trace_jsonl, MonitorFail, Requirement, TraceSample,
};
use flight_core::safety::{Event, COMMAND_MAX_AGE_MS, OFFBOARD_HEARTBEAT_MAX_AGE_MS};
use robot_world::World;

use super::world_backend::shared::aerial_event;
use super::world_backend::WorldSession;

/// Fault injected at a simulation time.
#[derive(Clone, Copy, Debug)]
pub enum Fault {
    GpsDropout { at_secs: f32 },
    HeartbeatStale { at_secs: f32 },
    Failsafe { at_secs: f32 },
    BatterySag { at_secs: f32, percent: u8 },
    WindGust { at_secs: f32, north: f32, east: f32 },
}

impl Fault {
    pub const fn at_secs(self) -> f32 {
        match self {
            Self::GpsDropout { at_secs }
            | Self::HeartbeatStale { at_secs }
            | Self::Failsafe { at_secs }
            | Self::BatterySag { at_secs, .. }
            | Self::WindGust { at_secs, .. } => at_secs,
        }
    }

    fn kernel_event(self) -> Option<Event> {
        match self {
            Self::GpsDropout { .. } => Some(Event::EstimatorInvalid),
            Self::HeartbeatStale { .. } => Some(Event::HeartbeatStale),
            Self::Failsafe { .. } => Some(Event::TriggerFailsafe),
            Self::BatterySag { .. } | Self::WindGust { .. } => None,
        }
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
        require: &[
            Requirement::NeverActuateWhileDisarmed,
            Requirement::ActuatorsImplyArmed,
            Requirement::NoNanCommands,
            Requirement::AltitudeBelow { meters: 120.0 },
            Requirement::PermitEpochMonotonic,
            Requirement::FailsafeWithinMs(250),
            Requirement::EpochBumped,
            Requirement::CommandAgeMs {
                max_ms: COMMAND_MAX_AGE_MS,
            },
            Requirement::EstimatorTimestampsMonotonic,
        ],
    };

    /// Offboard heartbeat loss must latch failsafe.
    pub const HEARTBEAT_LOSS: Self = Self {
        name: "heartbeat-stale",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.6,
        inject: &[Fault::HeartbeatStale { at_secs: 0.1 }],
        require: &[
            Requirement::NeverActuateWhileDisarmed,
            Requirement::ActuatorsImplyArmed,
            Requirement::NoNanCommands,
            Requirement::PermitEpochMonotonic,
            Requirement::FailsafeWithinMs(250),
            Requirement::EpochBumped,
        ],
    };

    /// Deadline miss / attach failsafe: same epoch revoke as the HITL rack.
    pub const HITL_MISS: Self = Self {
        name: "hitl-miss",
        catalog: "inland",
        seed: 1,
        dt: 0.02,
        duration_secs: 0.4,
        inject: &[Fault::Failsafe { at_secs: 0.1 }],
        require: &[
            Requirement::NeverActuateWhileDisarmed,
            Requirement::ActuatorsImplyArmed,
            Requirement::NoNanCommands,
            Requirement::PermitEpochMonotonic,
            Requirement::FailsafeWithinMs(250),
            Requirement::EpochBumped,
        ],
    };

    pub const ALL: &'static [Self] = &[Self::GPS_LOSS, Self::HEARTBEAT_LOSS, Self::HITL_MISS];

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
/// from [`flight_core::contracts::AerialOffboard::REVOKE_ON`].
pub fn run_revoke_table() -> Result<ScenarioReport, String> {
    use flight_core::contracts::AerialOffboard;
    let mut samples = Vec::new();
    for (i, e) in AerialOffboard::REVOKE_ON.iter().enumerate() {
        let session =
            WorldSession::named("inland", 1).ok_or_else(|| "unknown catalog inland".to_string())?;
        session
            .attach_offboard("drone")
            .map_err(|err| format!("grant: {e:?}: {err}"))?;
        aerial_event(session.aerial("drone").session(), "drone", *e)
            .map_err(|err| format!("inject {e:?}: {err}"))?;
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

/// Same GPS-loss contract on the verified world, checked-in ULog, and
/// converted PX4 SITL JSONL. Differential conformance, not a second physics.
pub fn differential_gps_loss() -> Result<(), String> {
    let reqs = Scenario::GPS_LOSS.require;
    let world = run_world(&Scenario::GPS_LOSS)?;
    world
        .evaluate(reqs)
        .map_err(|e| format!("world: {} at {}", e.requirement, e.index))?;
    differential_world(&Scenario::GPS_LOSS)?;
    let ulog = crate::parse_ulog(include_bytes!("../corpus/gps_loss.ulg"))
        .map_err(|e| format!("ulog: {e}"))?;
    evaluate_trace(&ulog, reqs).map_err(|e| format!("ulog: {} at {}", e.requirement, e.index))?;
    replay_jsonl(include_str!("../corpus/px4_sitl_gps_loss.jsonl"), reqs)
        .map_err(|e| format!("px4-sitl: {} at {}", e.requirement, e.index))?;
    Ok(())
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

fn apply_fault(session: &WorldSession, fault: Fault) -> Result<(), String> {
    if let Some(e) = fault.kernel_event() {
        aerial_event(session.aerial("drone").session(), "drone", e)
            .map_err(|err| format!("{err}"))?;
    }
    match fault {
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
        _ => {}
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
        estimator_ts_ms: if world.t <= 0.0 {
            0
        } else {
            (world.t * 1000.0) as u64
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::contracts::AerialOffboard;

    #[test]
    fn gps_loss_world_satisfies_contract() {
        let report = run_world(&Scenario::GPS_LOSS).expect("run");
        assert!(report.samples.iter().any(|s| s.failsafe));
        assert!(report.samples.iter().any(|s| s.epoch > 0));
        report
            .evaluate(Scenario::GPS_LOSS.require)
            .expect("contract");
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
    fn heartbeat_loss_world_satisfies_contract() {
        let report = run_world(&Scenario::HEARTBEAT_LOSS).expect("run");
        assert!(report.samples.iter().any(|s| s.failsafe));
        report
            .evaluate(Scenario::HEARTBEAT_LOSS.require)
            .expect("contract");
    }

    #[test]
    fn hitl_miss_world_and_attach_share_contract() {
        let world = run_world(&Scenario::HITL_MISS).expect("world");
        world
            .evaluate(Scenario::HITL_MISS.require)
            .expect("world contract");
        differential_world(&Scenario::HITL_MISS).expect("differential");
        let hitl = run_hitl_miss().expect("hitl");
        assert!(hitl.samples.iter().any(|s| s.failsafe));
        assert!(hitl.samples.iter().any(|s| s.epoch > 0));
        hitl.evaluate(Scenario::HITL_MISS.require)
            .expect("hitl contract");
    }

    #[test]
    fn revoke_table_faults_are_the_dsl_events() {
        let report = run_revoke_table().expect("revoke table");
        assert_eq!(
            report.samples.len(),
            flight_core::contracts::AerialOffboard::REVOKE_ON.len()
        );
        evaluate_trace(
            &report.samples,
            &[
                Requirement::NeverActuateWhileDisarmed,
                Requirement::ActuatorsImplyArmed,
                Requirement::NoNanCommands,
            ],
        )
        .expect("contract");
    }

    #[test]
    fn each_dsl_revoke_event_bumps_epoch_from_offboard() {
        for e in AerialOffboard::REVOKE_ON {
            let session = WorldSession::named("inland", 1).expect("catalog");
            session.attach_offboard("drone").expect("grant");
            aerial_event(session.aerial("drone").session(), "drone", *e)
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
