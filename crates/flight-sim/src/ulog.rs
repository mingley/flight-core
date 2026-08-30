//! Native PX4 ULog subset: the same [`TraceSample`] contract as JSONL.
//!
//! This is not a full PX4 log decoder. It writes and reads a packed
//! `fc_trace` message, and can ingest a `vehicle_status`-shaped stream
//! (timestamp / arming_state / failsafe) from converted SITL or real logs.

use flight_core::contracts::TraceSample;
use flight_core::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS;

/// `ULog` + 0x01 0x12 0x35 (PX4 file magic, 7 bytes).
const MAGIC: &[u8; 7] = b"ULog\x01\x12\x35";
const VERSION: u8 = 0;

const MSG_FORMAT: u8 = b'F';
const MSG_ADD: u8 = b'A';
const MSG_DATA: u8 = b'D';

const FC_TRACE: &str = "fc_trace:uint64_t timestamp;uint8_t armed;uint8_t actuators;uint8_t failsafe;uint32_t epoch;uint32_t heartbeat_age_ms;uint32_t command_age_ms;uint64_t estimator_ts_us;float cmd0;float cmd1;float cmd2;uint8_t has_cmd;float altitude_m;";

const FC_TRACE_DATA: usize = 8 + 1 + 1 + 1 + 4 + 4 + 4 + 8 + 4 + 4 + 4 + 1 + 4;

/// Write a ULog whose logged topic is `fc_trace`.
pub fn write_ulog(samples: &[TraceSample]) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(MAGIC);
    out.push(VERSION);
    let t0 = samples
        .first()
        .map(|s| (s.t_secs.max(0.0) * 1_000_000.0) as u64)
        .unwrap_or(0);
    out.extend_from_slice(&t0.to_le_bytes());
    write_msg(&mut out, MSG_FORMAT, FC_TRACE.as_bytes());
    let mut add = Vec::new();
    add.push(0); // multi_id
    add.extend_from_slice(&0u16.to_le_bytes()); // msg_id
    add.extend_from_slice(b"fc_trace");
    write_msg(&mut out, MSG_ADD, &add);
    for s in samples {
        let mut data = Vec::with_capacity(2 + FC_TRACE_DATA);
        data.extend_from_slice(&0u16.to_le_bytes());
        data.extend_from_slice(&((s.t_secs.max(0.0) * 1_000_000.0) as u64).to_le_bytes());
        data.push(u8::from(s.armed));
        data.push(u8::from(s.actuators_enabled));
        data.push(u8::from(s.failsafe));
        data.extend_from_slice(&s.epoch.to_le_bytes());
        data.extend_from_slice(&s.heartbeat_age_ms.to_le_bytes());
        data.extend_from_slice(&s.command_age_ms.to_le_bytes());
        data.extend_from_slice(&(s.estimator_ts_ms.saturating_mul(1000)).to_le_bytes());
        let cmd = s.command.unwrap_or([0.0, 0.0, 0.0]);
        data.extend_from_slice(&cmd[0].to_le_bytes());
        data.extend_from_slice(&cmd[1].to_le_bytes());
        data.extend_from_slice(&cmd[2].to_le_bytes());
        data.push(u8::from(s.command.is_some()));
        data.extend_from_slice(&s.altitude_m.to_le_bytes());
        write_msg(&mut out, MSG_DATA, &data);
    }
    out
}

fn write_msg(out: &mut Vec<u8>, ty: u8, payload: &[u8]) {
    let size = u16::try_from(payload.len()).unwrap_or(u16::MAX);
    out.extend_from_slice(&size.to_le_bytes());
    out.push(ty);
    out.extend_from_slice(payload);
}

/// Parse a ULog that contains `fc_trace` and/or `vehicle_status` data.
pub fn parse_ulog(bytes: &[u8]) -> Result<Vec<TraceSample>, String> {
    if bytes.len() < 16 || bytes.get(..7) != Some(MAGIC.as_slice()) {
        return Err("not a ULog (missing ULog\\x01\\x12\\x35 magic)".into());
    }
    let mut i = 16;
    let mut formats: Vec<(String, String)> = Vec::new(); // name -> format rest
    let mut ids: Vec<(u16, String)> = Vec::new();
    let mut fc = Vec::new();
    let mut status = Vec::new();
    while i + 3 <= bytes.len() {
        let size = u16::from_le_bytes([bytes[i], bytes[i + 1]]) as usize;
        let ty = bytes[i + 2];
        let start = i + 3;
        let end = start.saturating_add(size);
        if end > bytes.len() {
            return Err("truncated ULog message".into());
        }
        let payload = &bytes[start..end];
        match ty {
            MSG_FORMAT => {
                let s = core::str::from_utf8(payload).map_err(|_| "format utf8")?;
                if let Some((name, rest)) = s.split_once(':') {
                    formats.push((name.trim().to_string(), rest.to_string()));
                }
            }
            MSG_ADD => {
                if payload.len() < 3 {
                    return Err("add logged too short".into());
                }
                let msg_id = u16::from_le_bytes([payload[1], payload[2]]);
                let name = core::str::from_utf8(&payload[3..])
                    .unwrap_or("")
                    .trim_matches('\0')
                    .to_string();
                ids.push((msg_id, name));
            }
            MSG_DATA => {
                if payload.len() < 2 {
                    return Err("data too short".into());
                }
                let msg_id = u16::from_le_bytes([payload[0], payload[1]]);
                let data = &payload[2..];
                if let Some((_, name)) = ids.iter().find(|(id, _)| *id == msg_id) {
                    if name == "fc_trace" {
                        if let Some(s) = parse_fc_trace(data) {
                            fc.push(s);
                        }
                    } else if name == "vehicle_status" {
                        if let Some(s) = parse_vehicle_status(data, &formats) {
                            status.push(s);
                        }
                    }
                }
            }
            _ => {}
        }
        i = end;
    }
    if !fc.is_empty() {
        return Ok(fc);
    }
    if status.is_empty() {
        return Err("ULog has no fc_trace or vehicle_status data".into());
    }
    Ok(stamp_status_epochs(status))
}

fn parse_fc_trace(data: &[u8]) -> Option<TraceSample> {
    if data.len() < FC_TRACE_DATA {
        return None;
    }
    let mut o = 0;
    let take = |o: &mut usize, n: usize| -> Option<&[u8]> {
        let s = data.get(*o..*o + n)?;
        *o += n;
        Some(s)
    };
    let ts = u64::from_le_bytes(take(&mut o, 8)?.try_into().ok()?);
    let armed = take(&mut o, 1)?[0] != 0;
    let actuators = take(&mut o, 1)?[0] != 0;
    let failsafe = take(&mut o, 1)?[0] != 0;
    let epoch = u32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let heartbeat_age_ms = u32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let command_age_ms = u32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let estimator_ts_us = u64::from_le_bytes(take(&mut o, 8)?.try_into().ok()?);
    let cmd0 = f32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let cmd1 = f32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let cmd2 = f32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    let has_cmd = take(&mut o, 1)?[0] != 0;
    let altitude_m = f32::from_le_bytes(take(&mut o, 4)?.try_into().ok()?);
    Some(TraceSample {
        t_secs: ts as f32 / 1_000_000.0,
        armed,
        actuators_enabled: actuators,
        failsafe,
        epoch,
        heartbeat_age_ms,
        command: if has_cmd {
            Some([cmd0, cmd1, cmd2])
        } else {
            None
        },
        altitude_m,
        command_age_ms,
        estimator_ts_ms: estimator_ts_us / 1000,
    })
}

fn parse_vehicle_status(data: &[u8], formats: &[(String, String)]) -> Option<TraceSample> {
    let rest = formats
        .iter()
        .find(|(n, _)| n == "vehicle_status")
        .map(|(_, r)| r.as_str())
        .unwrap_or("uint64_t timestamp;uint8_t arming_state;uint8_t nav_state;uint8_t failsafe;");
    let mut fields = Vec::new();
    for part in rest.split(';') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let mut it = part.split_whitespace();
        let ty = it.next()?;
        let name = it.next()?.trim();
        let size = match ty {
            "uint64_t" | "int64_t" => 8,
            "uint32_t" | "int32_t" | "float" => 4,
            "uint16_t" | "int16_t" => 2,
            "uint8_t" | "int8_t" | "bool" => 1,
            _ => return None,
        };
        fields.push((name.to_string(), size));
    }
    let mut o = 0;
    let mut timestamp_us = 0u64;
    let mut arming_state = 0u8;
    let mut failsafe = false;
    for (name, size) in fields {
        let slice = data.get(o..o + size)?;
        o += size;
        match name.as_str() {
            "timestamp" if size == 8 => {
                timestamp_us = u64::from_le_bytes(slice.try_into().ok()?);
            }
            "arming_state" if size == 1 => arming_state = slice[0],
            "failsafe" if size == 1 => failsafe = slice[0] != 0,
            _ => {}
        }
    }
    let armed = arming_state == 2;
    Some(TraceSample {
        t_secs: timestamp_us as f32 / 1_000_000.0,
        armed,
        actuators_enabled: armed && !failsafe,
        failsafe,
        epoch: 0,
        heartbeat_age_ms: if failsafe {
            OFFBOARD_HEARTBEAT_MAX_AGE_MS
        } else {
            0
        },
        command: None,
        altitude_m: 0.0,
        command_age_ms: 0,
        estimator_ts_ms: timestamp_us / 1000,
    })
}

fn stamp_status_epochs(mut samples: Vec<TraceSample>) -> Vec<TraceSample> {
    let mut epoch = 0u32;
    let mut prev_fs = false;
    for s in &mut samples {
        if s.failsafe && !prev_fs {
            epoch = epoch.saturating_add(1);
        }
        s.epoch = epoch;
        prev_fs = s.failsafe;
    }
    samples
}

/// `true` when `bytes` starts with the ULog magic.
pub fn is_ulog(bytes: &[u8]) -> bool {
    bytes.len() >= 7 && bytes[..7] == MAGIC[..]
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::contracts::{evaluate_trace, Requirement};

    fn sample(t: f32, failsafe: bool, epoch: u32) -> TraceSample {
        TraceSample {
            t_secs: t,
            armed: true,
            actuators_enabled: !failsafe,
            failsafe,
            epoch,
            heartbeat_age_ms: if failsafe {
                OFFBOARD_HEARTBEAT_MAX_AGE_MS
            } else {
                0
            },
            command: if failsafe {
                None
            } else {
                Some([0.0, 0.0, -1.0])
            },
            altitude_m: 2.0,
            command_age_ms: 0,
            estimator_ts_ms: (t * 1000.0) as u64,
        }
    }

    #[test]
    fn fc_trace_roundtrip_preserves_contract_fields() {
        let samples = [sample(0.0, false, 0), sample(0.2, true, 1)];
        let bytes = write_ulog(&samples);
        assert!(is_ulog(&bytes));
        let back = parse_ulog(&bytes).expect("parse");
        assert_eq!(back.len(), 2);
        assert!(!back[0].failsafe);
        assert!(back[1].failsafe);
        assert_eq!(back[1].epoch, 1);
        assert_eq!(back[0].command, Some([0.0, 0.0, -1.0]));
        assert!(back[1].command.is_none());
        evaluate_trace(
            &back,
            &[
                Requirement::NeverActuateWhileDisarmed,
                Requirement::ActuatorsImplyArmed,
                Requirement::EpochBumped,
                Requirement::EstimatorTimestampsMonotonic,
            ],
        )
        .expect("contract");
    }

    #[test]
    fn vehicle_status_maps_arming_and_failsafe() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(MAGIC);
        bytes.push(VERSION);
        bytes.extend_from_slice(&0u64.to_le_bytes());
        let fmt = b"vehicle_status:uint64_t timestamp;uint8_t arming_state;uint8_t nav_state;uint8_t failsafe;";
        write_msg(&mut bytes, MSG_FORMAT, fmt);
        let mut add = Vec::new();
        add.push(0);
        add.extend_from_slice(&1u16.to_le_bytes());
        add.extend_from_slice(b"vehicle_status");
        write_msg(&mut bytes, MSG_ADD, &add);
        for (t_us, armed, fs) in [(0u64, true, false), (200_000u64, true, true)] {
            let mut data = Vec::new();
            data.extend_from_slice(&1u16.to_le_bytes());
            data.extend_from_slice(&t_us.to_le_bytes());
            data.push(if armed { 2 } else { 1 });
            data.push(0);
            data.push(u8::from(fs));
            write_msg(&mut bytes, MSG_DATA, &data);
        }
        let samples = parse_ulog(&bytes).expect("status");
        assert_eq!(samples.len(), 2);
        assert!(samples[0].armed && !samples[0].failsafe);
        assert!(samples[1].failsafe);
        assert_eq!(samples[1].epoch, 1);
        evaluate_trace(&samples, &[Requirement::EpochBumped]).expect("epoch");
    }

    #[test]
    fn checked_in_gps_loss_ulog_satisfies_the_world_contract() {
        let bytes = include_bytes!("../corpus/gps_loss.ulg");
        assert!(is_ulog(bytes));
        let samples = parse_ulog(bytes).expect("corpus ulog");
        assert!(samples.iter().any(|s| s.failsafe));
        assert!(samples.iter().any(|s| s.epoch > 0));
        evaluate_trace(&samples, crate::Scenario::GPS_LOSS.require).expect("corpus contract");
    }
}
