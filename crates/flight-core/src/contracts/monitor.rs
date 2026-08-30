//! Runtime monitors generated from the same contract table as the types.
//!
//! Compile-time proofs cannot see PX4, GPS, or a late packet. These checks
//! evaluate a recorded or live trace against the same invariants the kernel
//! and [`super::spec`] describe.

use crate::safety::{
    admit_offboard_command, command_age_ok, estimator_ts_monotonic, OFFBOARD_HEARTBEAT_MAX_AGE_MS,
};
use crate::temporal::{CommandFresh, HeartbeatFresh, Sequence, Timestamp};

/// One sample of physical/control state for contract evaluation.
#[derive(Clone, Copy, Debug)]
pub struct TraceSample {
    pub t_secs: f32,
    pub armed: bool,
    pub actuators_enabled: bool,
    pub failsafe: bool,
    pub epoch: u32,
    pub heartbeat_age_ms: u32,
    pub command: Option<[f32; 3]>,
    pub altitude_m: f32,
    /// Age of the last planner command at this sample. `0` if none.
    pub command_age_ms: u32,
    /// Estimator timestamp in milliseconds. `0` if the log did not carry one.
    pub estimator_ts_ms: u64,
}

impl Default for TraceSample {
    fn default() -> Self {
        Self {
            t_secs: 0.0,
            armed: false,
            actuators_enabled: false,
            failsafe: false,
            epoch: 0,
            heartbeat_age_ms: 0,
            command: None,
            altitude_m: 0.0,
            command_age_ms: 0,
            estimator_ts_ms: 0,
        }
    }
}

impl TraceSample {
    pub fn command_is_nan(self) -> bool {
        match self.command {
            Some(c) => !c[0].is_finite() || !c[1].is_finite() || !c[2].is_finite(),
            None => false,
        }
    }

    pub fn actuating(self) -> bool {
        self.actuators_enabled || self.command.is_some()
    }
}

/// First-class contract requirement. The scenario lab and replay both use this
/// enum so sim, SITL logs, and ulog-shaped traces share one evaluator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Requirement {
    NeverActuateWhileDisarmed,
    ActuatorsImplyArmed,
    PermitEpochMonotonic,
    FailsafeWithinMs(u32),
    NoNanCommands,
    AltitudeBelow {
        meters: f32,
    },
    OffboardHeartbeatFresh,
    /// Some sample's epoch is greater than the first sample (authority revoked).
    EpochBumped,
    /// When a command is present, its age is inside `max_ms`.
    CommandAgeMs {
        max_ms: u32,
    },
    /// Estimator timestamps never jump backward.
    EstimatorTimestampsMonotonic,
    /// Armed, not in failsafe, and actuating ⇒ kernel `admit_offboard_command`.
    OffboardAdmitted,
}

/// Why a trace failed a requirement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MonitorFail {
    pub requirement: &'static str,
    pub index: usize,
}

impl MonitorFail {
    pub const fn name(self) -> &'static str {
        self.requirement
    }
}

/// Evaluate `samples` (time-ordered) against `reqs`.
///
/// `FailsafeWithinMs` is relative to the first sample that is already in
/// failsafe **or** to `t=0` when the scenario injected a loss at the start
/// of the recording. Callers that inject a fault at a later time should
/// slice the trace or pass a prefix that starts at the fault.
pub fn evaluate_trace(samples: &[TraceSample], reqs: &[Requirement]) -> Result<(), MonitorFail> {
    if samples.is_empty() {
        return Ok(());
    }
    for req in reqs {
        match *req {
            Requirement::NeverActuateWhileDisarmed => {
                for (i, s) in samples.iter().enumerate() {
                    if !s.armed && s.actuating() {
                        return Err(MonitorFail {
                            requirement: "never_actuate_while_disarmed",
                            index: i,
                        });
                    }
                }
            }
            Requirement::ActuatorsImplyArmed => {
                for (i, s) in samples.iter().enumerate() {
                    if s.actuators_enabled && !s.armed {
                        return Err(MonitorFail {
                            requirement: "actuators_enabled_implies_armed",
                            index: i,
                        });
                    }
                }
            }
            Requirement::PermitEpochMonotonic => {
                let mut seq = Sequence::new();
                for (i, s) in samples.iter().enumerate() {
                    if seq.observe(s.epoch).is_err() {
                        return Err(MonitorFail {
                            requirement: "permit_epoch_monotonic",
                            index: i,
                        });
                    }
                }
            }
            Requirement::FailsafeWithinMs(ms) => {
                let start = samples[0].t_secs;
                let limit = start + (ms as f32) / 1000.0;
                let ok = samples
                    .iter()
                    .any(|s| s.failsafe && s.t_secs <= limit + 1e-6);
                if !ok {
                    return Err(MonitorFail {
                        requirement: "failsafe_within",
                        index: 0,
                    });
                }
            }
            Requirement::NoNanCommands => {
                for (i, s) in samples.iter().enumerate() {
                    if s.command_is_nan() {
                        return Err(MonitorFail {
                            requirement: "no_nan_commands",
                            index: i,
                        });
                    }
                }
            }
            Requirement::AltitudeBelow { meters } => {
                for (i, s) in samples.iter().enumerate() {
                    if s.altitude_m > meters {
                        return Err(MonitorFail {
                            requirement: "altitude_below",
                            index: i,
                        });
                    }
                }
            }
            Requirement::OffboardHeartbeatFresh => {
                for (i, s) in samples.iter().enumerate() {
                    if !s.armed || s.failsafe {
                        continue;
                    }
                    let typed_stale = HeartbeatFresh::check_age(s.heartbeat_age_ms).is_err();
                    let kernel_stale = s.heartbeat_age_ms >= OFFBOARD_HEARTBEAT_MAX_AGE_MS;
                    if typed_stale || kernel_stale {
                        return Err(MonitorFail {
                            requirement: "offboard_heartbeat_fresh",
                            index: i,
                        });
                    }
                }
            }
            Requirement::EpochBumped => {
                let first = samples[0].epoch;
                let ok = samples.iter().any(|s| s.epoch > first);
                if !ok {
                    return Err(MonitorFail {
                        requirement: "epoch_bumped",
                        index: 0,
                    });
                }
            }
            Requirement::CommandAgeMs { max_ms } => {
                for (i, s) in samples.iter().enumerate() {
                    if s.command.is_none() {
                        continue;
                    }
                    if max_ms == crate::safety::COMMAND_MAX_AGE_MS {
                        if CommandFresh::<()>::check_age(s.command_age_ms).is_err()
                            || !command_age_ok(s.command_age_ms)
                        {
                            return Err(MonitorFail {
                                requirement: "command_age",
                                index: i,
                            });
                        }
                    } else if s.command_age_ms >= max_ms {
                        return Err(MonitorFail {
                            requirement: "command_age",
                            index: i,
                        });
                    }
                }
            }
            Requirement::EstimatorTimestampsMonotonic => {
                let mut prev = Timestamp::from_millis(samples[0].estimator_ts_ms);
                for (i, s) in samples.iter().enumerate().skip(1) {
                    let next = Timestamp::from_millis(s.estimator_ts_ms);
                    if !prev.precedes(next)
                        || !estimator_ts_monotonic(prev.as_millis(), s.estimator_ts_ms)
                    {
                        return Err(MonitorFail {
                            requirement: "estimator_ts_monotonic",
                            index: i,
                        });
                    }
                    prev = next;
                }
            }
            Requirement::OffboardAdmitted => {
                for (i, s) in samples.iter().enumerate() {
                    if s.failsafe || !s.armed {
                        continue;
                    }
                    if s.command.is_none() && !s.actuators_enabled {
                        continue;
                    }
                    if !admit_offboard_command(s.heartbeat_age_ms, s.command_age_ms) {
                        return Err(MonitorFail {
                            requirement: "offboard_admitted",
                            index: i,
                        });
                    }
                }
            }
        }
    }
    Ok(())
}

/// Parse a JSONL contract trace (the format [`TraceSample`] JSONL writers emit).
#[cfg(feature = "std")]
pub fn parse_trace_jsonl(text: &str) -> Result<Vec<TraceSample>, &'static str> {
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        out.push(parse_trace_line(line)?);
    }
    Ok(out)
}

#[cfg(feature = "std")]
fn parse_trace_line(line: &str) -> Result<TraceSample, &'static str> {
    fn num_after<'a>(s: &'a str, key: &str) -> Option<&'a str> {
        let i = s.find(key)?;
        let rest = s[i + key.len()..].trim_start_matches(['"', ':']);
        let rest = rest.trim_start();
        let end = rest.find([',', '}', ']']).unwrap_or(rest.len());
        Some(rest[..end].trim())
    }
    fn bool_after(s: &str, key: &str) -> Option<bool> {
        match num_after(s, key)? {
            "true" => Some(true),
            "false" => Some(false),
            _ => None,
        }
    }
    let t_secs: f32 = num_after(line, "\"t\"")
        .or_else(|| num_after(line, "\"t_secs\""))
        .and_then(|s| s.parse().ok())
        .ok_or("t")?;
    let armed = bool_after(line, "\"armed\"").ok_or("armed")?;
    let actuators = bool_after(line, "\"actuators\"")
        .or_else(|| bool_after(line, "\"actuators_enabled\""))
        .ok_or("actuators")?;
    let failsafe = bool_after(line, "\"failsafe\"").ok_or("failsafe")?;
    let epoch: u32 = num_after(line, "\"epoch\"")
        .and_then(|s| s.parse().ok())
        .ok_or("epoch")?;
    let altitude_m: f32 = num_after(line, "\"alt\"")
        .or_else(|| num_after(line, "\"altitude_m\""))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let heartbeat_age_ms: u32 = num_after(line, "\"heartbeat_age_ms\"")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let command_age_ms: u32 = num_after(line, "\"command_age_ms\"")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let estimator_ts_ms: u64 = num_after(line, "\"estimator_ts_ms\"")
        .or_else(|| num_after(line, "\"estimator_ts\""))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let command = if line.contains("\"cmd\":null") || line.contains("\"command\":null") {
        None
    } else if let Some(i) = line.find("\"cmd\":[") {
        parse_cmd_triple(&line[i + 7..])
    } else {
        None
    };
    Ok(TraceSample {
        t_secs,
        armed,
        actuators_enabled: actuators,
        failsafe,
        epoch,
        heartbeat_age_ms,
        command,
        altitude_m,
        command_age_ms,
        estimator_ts_ms,
    })
}

#[cfg(feature = "std")]
fn parse_cmd_triple(s: &str) -> Option<[f32; 3]> {
    let end = s.find(']')?;
    let mut parts = s[..end].split(',');
    let a = parts.next()?.trim().parse().ok()?;
    let b = parts.next()?.trim().parse().ok()?;
    let c = parts.next()?.trim().parse().ok()?;
    Some([a, b, c])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(armed: bool, actuators: bool) -> TraceSample {
        TraceSample {
            armed,
            actuators_enabled: actuators,
            altitude_m: 1.0,
            ..TraceSample::default()
        }
    }

    #[test]
    fn disarmed_actuation_fails() {
        let s = TraceSample {
            command: Some([0.0, 0.0, -1.0]),
            ..sample(false, false)
        };
        assert!(evaluate_trace(&[s], &[Requirement::NeverActuateWhileDisarmed]).is_err());
    }

    #[test]
    fn armed_command_passes() {
        let s = TraceSample {
            command: Some([0.0, 0.0, -1.0]),
            ..sample(true, true)
        };
        assert!(evaluate_trace(&[s], &[Requirement::NeverActuateWhileDisarmed]).is_ok());
    }

    #[test]
    fn failsafe_window() {
        let a = TraceSample {
            t_secs: 0.0,
            failsafe: false,
            ..sample(true, true)
        };
        let b = TraceSample {
            t_secs: 0.12,
            failsafe: true,
            armed: true,
            actuators_enabled: false,
            command: None,
            ..sample(true, false)
        };
        assert!(evaluate_trace(&[a, b], &[Requirement::FailsafeWithinMs(250)]).is_ok());
        assert!(evaluate_trace(&[a], &[Requirement::FailsafeWithinMs(250)]).is_err());
    }

    #[test]
    fn epoch_bump_and_command_age_and_estimator() {
        let a = TraceSample {
            epoch: 0,
            estimator_ts_ms: 10,
            command: Some([0.0, 0.0, 0.0]),
            command_age_ms: 0,
            ..sample(true, true)
        };
        let b = TraceSample {
            t_secs: 0.2,
            epoch: 1,
            failsafe: true,
            estimator_ts_ms: 210,
            command: None,
            command_age_ms: 0,
            ..sample(true, false)
        };
        assert!(evaluate_trace(&[a, b], &[Requirement::EpochBumped]).is_ok());
        assert!(evaluate_trace(&[a], &[Requirement::EpochBumped]).is_err());
        assert!(evaluate_trace(
            &[a, b],
            &[
                Requirement::CommandAgeMs { max_ms: 100 },
                Requirement::EstimatorTimestampsMonotonic
            ]
        )
        .is_ok());
        let back = TraceSample {
            estimator_ts_ms: 5,
            ..b
        };
        assert!(evaluate_trace(&[a, back], &[Requirement::EstimatorTimestampsMonotonic]).is_err());
        let stale = TraceSample {
            command: Some([1.0, 0.0, 0.0]),
            command_age_ms: 100,
            ..a
        };
        assert!(evaluate_trace(&[stale], &[Requirement::CommandAgeMs { max_ms: 100 }]).is_err());
    }

    #[test]
    fn offboard_admitted_uses_kernel_admit() {
        let ok = TraceSample {
            command: Some([0.0, 0.0, -1.0]),
            heartbeat_age_ms: 0,
            command_age_ms: 0,
            ..sample(true, true)
        };
        assert!(evaluate_trace(&[ok], &[Requirement::OffboardAdmitted]).is_ok());
        let stale = TraceSample {
            heartbeat_age_ms: 250,
            ..ok
        };
        assert!(evaluate_trace(&[stale], &[Requirement::OffboardAdmitted]).is_err());
        let failsafe = TraceSample {
            failsafe: true,
            heartbeat_age_ms: 250,
            ..ok
        };
        assert!(evaluate_trace(&[failsafe], &[Requirement::OffboardAdmitted]).is_ok());
    }

    #[test]
    fn parse_jsonl_roundtrip_fields() {
        let line = "{\"t\":0.2,\"armed\":true,\"actuators\":false,\"failsafe\":true,\"epoch\":1,\"alt\":2.5,\"cmd\":null,\"command_age_ms\":3,\"estimator_ts_ms\":200}\n";
        let s = parse_trace_jsonl(line).unwrap();
        assert_eq!(s.len(), 1);
        assert!(s[0].failsafe);
        assert_eq!(s[0].epoch, 1);
        assert!((s[0].t_secs - 0.2).abs() < 1e-6);
        assert_eq!(s[0].command_age_ms, 3);
        assert_eq!(s[0].estimator_ts_ms, 200);
    }
}
