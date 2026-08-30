//! Runtime monitors generated from the same contract table as the types.
//!
//! Compile-time proofs cannot see PX4, GPS, or a late packet. These checks
//! evaluate a recorded or live trace against the same invariants the kernel
//! and [`super::spec`] describe.

use crate::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS;

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
    AltitudeBelow { meters: f32 },
    OffboardHeartbeatFresh,
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
                let mut prev = samples[0].epoch;
                for (i, s) in samples.iter().enumerate().skip(1) {
                    if s.epoch < prev {
                        return Err(MonitorFail {
                            requirement: "permit_epoch_monotonic",
                            index: i,
                        });
                    }
                    prev = s.epoch;
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
                    if s.armed && !s.failsafe && s.heartbeat_age_ms >= OFFBOARD_HEARTBEAT_MAX_AGE_MS
                    {
                        return Err(MonitorFail {
                            requirement: "offboard_heartbeat_fresh",
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
            t_secs: 0.0,
            armed,
            actuators_enabled: actuators,
            failsafe: false,
            epoch: 0,
            heartbeat_age_ms: 0,
            command: None,
            altitude_m: 1.0,
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
    fn parse_jsonl_roundtrip_fields() {
        let line = "{\"t\":0.2,\"armed\":true,\"actuators\":false,\"failsafe\":true,\"epoch\":1,\"alt\":2.5,\"cmd\":null}\n";
        let s = parse_trace_jsonl(line).unwrap();
        assert_eq!(s.len(), 1);
        assert!(s[0].failsafe);
        assert_eq!(s[0].epoch, 1);
        assert!((s[0].t_secs - 0.2).abs() < 1e-6);
    }
}
