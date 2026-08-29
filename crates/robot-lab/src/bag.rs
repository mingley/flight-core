//! Foxglove-compatible MCAP writer for lab traces.
//!
//! Records JSON observations and timed actions with `jsonschema` encoding so a
//! researcher can open the bag in Foxglove without a protobuf pipeline. The
//! writer is a small, dependency-free subset of the MCAP spec: Header, Schema,
//! Channel, Message, DataEnd, Footer.

use crate::{Observation, TimedAction};
use serde::Serialize;
use std::io::{self, Write};

const MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";
const OP_HEADER: u8 = 0x01;
const OP_FOOTER: u8 = 0x02;
const OP_SCHEMA: u8 = 0x03;
const OP_CHANNEL: u8 = 0x04;
const OP_MESSAGE: u8 = 0x05;
const OP_DATA_END: u8 = 0x0F;

const CH_OBS: u16 = 1;
const CH_ACT: u16 = 2;
const SCHEMA_OBS: u16 = 1;
const SCHEMA_ACT: u16 = 2;

const OBS_SCHEMA: &str = r#"{"type":"object","title":"lab.Observation","required":["t","scenario","all_hold","robots","properties","sphere_hits"],"properties":{"t":{"type":"number"},"scenario":{"type":"string"},"seed":{"type":"integer"},"message":{"type":"string"},"all_hold":{"type":"boolean"},"properties":{"type":"array","items":{"type":"object","properties":{"id":{"type":"string"},"holds":{"type":"boolean"},"detail":{"type":"string"}}}},"sphere_hits":{"type":"array","items":{"type":"object","properties":{"a":{"type":"string"},"b":{"type":"string"},"jn":{"type":"number"},"jt":{"type":"number"}}}},"robots":{"type":"array","items":{"type":"object","properties":{"id":{"type":"string"},"legal_cmds":{"type":"array","items":{"type":"string"}},"hold_ned":{"type":["array","null"],"items":{"type":"number"},"minItems":3,"maxItems":3},"aerial":{"type":["object","null"],"properties":{"kind":{"type":"string"}}},"ground":{"type":["object","null"],"properties":{"kind":{"type":"string"}}},"marine":{"type":["object","null"],"properties":{"kind":{"type":"string"}}}}}}}}"#;
const ACT_SCHEMA: &str = r#"{"type":"object","title":"lab.TimedAction","required":["t","cmd"],"properties":{"t":{"type":"number"},"robot":{"type":"string"},"cmd":{"type":"string"},"vn":{"type":"number"},"ve":{"type":"number"},"vd":{"type":"number"},"yaw_rate":{"type":"number"}}}"#;

/// Streaming MCAP bag. Call [`McapBag::finish`] after the last message.
pub struct McapBag<W: Write> {
    inner: W,
    seq_obs: u32,
    seq_act: u32,
}

impl<W: Write> McapBag<W> {
    /// Magic, header, JSON schemas, and channels for observations + actions.
    pub fn new(mut inner: W) -> io::Result<Self> {
        inner.write_all(MAGIC)?;
        write_record(&mut inner, OP_HEADER, &header_payload())?;
        write_record(
            &mut inner,
            OP_SCHEMA,
            &schema_payload(SCHEMA_OBS, "lab.Observation", OBS_SCHEMA.as_bytes()),
        )?;
        write_record(
            &mut inner,
            OP_SCHEMA,
            &schema_payload(SCHEMA_ACT, "lab.TimedAction", ACT_SCHEMA.as_bytes()),
        )?;
        write_record(
            &mut inner,
            OP_CHANNEL,
            &channel_payload(CH_OBS, SCHEMA_OBS, "/lab/observation"),
        )?;
        write_record(
            &mut inner,
            OP_CHANNEL,
            &channel_payload(CH_ACT, SCHEMA_ACT, "/lab/action"),
        )?;
        Ok(Self {
            inner,
            seq_obs: 0,
            seq_act: 0,
        })
    }

    pub fn write_observation(&mut self, obs: &Observation) -> io::Result<()> {
        self.seq_obs = self.seq_obs.wrapping_add(1);
        write_json_message(&mut self.inner, CH_OBS, self.seq_obs, time_ns(obs.t), obs)
    }

    pub fn write_action(&mut self, action: &TimedAction) -> io::Result<()> {
        self.seq_act = self.seq_act.wrapping_add(1);
        write_json_message(
            &mut self.inner,
            CH_ACT,
            self.seq_act,
            time_ns(action.t),
            action,
        )
    }

    /// DataEnd + empty Footer + trailing magic. Consumes the writer.
    pub fn finish(mut self) -> io::Result<W> {
        write_record(&mut self.inner, OP_DATA_END, &u32::to_le_bytes(0))?;
        write_record(&mut self.inner, OP_FOOTER, &footer_payload())?;
        self.inner.write_all(MAGIC)?;
        self.inner.flush()?;
        Ok(self.inner)
    }
}

fn time_ns(t: f32) -> u64 {
    (t.max(0.0) as f64 * 1_000_000_000.0) as u64
}

fn header_payload() -> Vec<u8> {
    let mut p = Vec::new();
    put_str(&mut p, "");
    put_str(&mut p, "flight-core robot-lab");
    p
}

fn footer_payload() -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&0u64.to_le_bytes());
    p.extend_from_slice(&0u64.to_le_bytes());
    p.extend_from_slice(&0u32.to_le_bytes());
    p
}

fn schema_payload(id: u16, name: &str, data: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&id.to_le_bytes());
    put_str(&mut p, name);
    put_str(&mut p, "jsonschema");
    put_bytes(&mut p, data);
    p
}

fn channel_payload(id: u16, schema_id: u16, topic: &str) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&id.to_le_bytes());
    p.extend_from_slice(&schema_id.to_le_bytes());
    put_str(&mut p, topic);
    put_str(&mut p, "json");
    p.extend_from_slice(&0u32.to_le_bytes());
    p
}

fn write_json_message<W: Write, T: Serialize>(
    w: &mut W,
    channel: u16,
    seq: u32,
    t_ns: u64,
    value: &T,
) -> io::Result<()> {
    let json = serde_json::to_vec(value).map_err(json_err)?;
    let mut p = Vec::with_capacity(22 + json.len());
    p.extend_from_slice(&channel.to_le_bytes());
    p.extend_from_slice(&seq.to_le_bytes());
    p.extend_from_slice(&t_ns.to_le_bytes());
    p.extend_from_slice(&t_ns.to_le_bytes());
    p.extend_from_slice(&json);
    write_record(w, OP_MESSAGE, &p)
}

fn write_record<W: Write>(w: &mut W, opcode: u8, payload: &[u8]) -> io::Result<()> {
    w.write_all(&[opcode])?;
    w.write_all(&(payload.len() as u64).to_le_bytes())?;
    w.write_all(payload)
}

fn put_str(buf: &mut Vec<u8>, s: &str) {
    put_bytes(buf, s.as_bytes());
}

fn put_bytes(buf: &mut Vec<u8>, data: &[u8]) {
    buf.extend_from_slice(&(data.len() as u32).to_le_bytes());
    buf.extend_from_slice(data);
}

fn json_err(err: serde_json::Error) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, err)
}

/// True when `bytes` is a complete MCAP file this writer would emit.
pub fn looks_like_mcap(bytes: &[u8]) -> bool {
    const FOOTER_LEN: usize = 1 + 8 + 20;
    if bytes.len() < MAGIC.len() * 2 + FOOTER_LEN {
        return false;
    }
    if !bytes.starts_with(MAGIC) || !bytes.ends_with(MAGIC) {
        return false;
    }
    let footer_at = bytes.len() - MAGIC.len() - FOOTER_LEN;
    bytes[footer_at] == OP_FOOTER
}

/// JSON payloads on `/lab/observation` (channel 1), in write order.
pub fn observation_json(bytes: &[u8]) -> io::Result<Vec<serde_json::Value>> {
    json_on_channel(bytes, CH_OBS)
}

/// JSON payloads on `/lab/action` (channel 2), in write order.
pub fn action_json(bytes: &[u8]) -> io::Result<Vec<serde_json::Value>> {
    json_on_channel(bytes, CH_ACT)
}

/// Schema document named in the bag (`lab.Observation`, `lab.TimedAction`).
pub fn schema_json(bytes: &[u8], name: &str) -> io::Result<String> {
    for (op, payload) in records(bytes)? {
        if op != OP_SCHEMA {
            continue;
        }
        let mut i = 2usize;
        let Some(schema_name) = read_lenstr(payload, &mut i) else {
            continue;
        };
        let Some(_encoding) = read_lenstr(payload, &mut i) else {
            continue;
        };
        let Some(data) = read_lenstr(payload, &mut i) else {
            continue;
        };
        if schema_name == name {
            return Ok(data.to_string());
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!("schema {name} missing"),
    ))
}

fn json_on_channel(bytes: &[u8], channel: u16) -> io::Result<Vec<serde_json::Value>> {
    let mut out = Vec::new();
    for (op, payload) in records(bytes)? {
        if op != OP_MESSAGE || payload.len() < 22 {
            continue;
        }
        let ch = u16::from_le_bytes([payload[0], payload[1]]);
        if ch != channel {
            continue;
        }
        let json = serde_json::from_slice(&payload[22..]).map_err(json_err)?;
        out.push(json);
    }
    Ok(out)
}

fn records(bytes: &[u8]) -> io::Result<Vec<(u8, &[u8])>> {
    if !bytes.starts_with(MAGIC) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "missing MCAP magic",
        ));
    }
    let mut i = MAGIC.len();
    let mut out = Vec::new();
    while i + MAGIC.len() <= bytes.len() && &bytes[i..i + MAGIC.len()] != MAGIC {
        if i + 9 > bytes.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "truncated MCAP record header",
            ));
        }
        let op = bytes[i];
        let mut len_bytes = [0u8; 8];
        len_bytes.copy_from_slice(&bytes[i + 1..i + 9]);
        let len = u64::from_le_bytes(len_bytes) as usize;
        i += 9;
        if i + len > bytes.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "truncated MCAP record",
            ));
        }
        out.push((op, &bytes[i..i + len]));
        i += len;
    }
    Ok(out)
}

fn read_lenstr<'a>(data: &'a [u8], i: &mut usize) -> Option<&'a str> {
    if *i + 4 > data.len() {
        return None;
    }
    let n = u32::from_le_bytes(data[*i..*i + 4].try_into().ok()?) as usize;
    *i += 4;
    if *i + n > data.len() {
        return None;
    }
    let s = std::str::from_utf8(&data[*i..*i + n]).ok()?;
    *i += n;
    Some(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Lab;

    #[test]
    fn empty_bag_has_magic_and_footer() {
        let bag = McapBag::new(Vec::new()).unwrap();
        let bytes = bag.finish().unwrap();
        assert!(looks_like_mcap(&bytes), "len {}", bytes.len());
        assert_eq!(&bytes[..8], MAGIC);
        assert_eq!(&bytes[bytes.len() - 8..], MAGIC);
    }

    #[test]
    fn lab_observation_is_a_json_message() {
        let mut lab = Lab::coastal(1);
        lab.step(0.02);
        let mut bag = McapBag::new(Vec::new()).unwrap();
        bag.write_observation(&lab.observe()).unwrap();
        let bytes = bag.finish().unwrap();
        assert!(looks_like_mcap(&bytes));
        let json = serde_json::to_vec(&lab.observe()).unwrap();
        assert!(
            bytes.windows(json.len()).any(|w| w == json),
            "observation JSON missing from bag"
        );
    }

    #[test]
    fn observation_schema_names_hold_and_legal_cmds() {
        let bag = McapBag::new(Vec::new()).unwrap();
        let bytes = bag.finish().unwrap();
        let schema = schema_json(&bytes, "lab.Observation").unwrap();
        for key in [
            "hold_ned",
            "legal_cmds",
            "kind",
            "sphere_hits",
            "properties",
        ] {
            assert!(schema.contains(key), "schema missing {key}: {schema}");
        }
        let act = schema_json(&bytes, "lab.TimedAction").unwrap();
        assert!(act.contains("cmd"), "{act}");
    }

    #[test]
    fn bag_round_trip_keeps_hold_ned_and_legal_cmds() {
        use crate::{AgentAction, LabCmd, TimedAction};

        let mut lab = Lab::open("inland", 3).unwrap();
        lab.attach_takeoff("drone").expect("takeoff");
        lab.attach_hold("drone").expect("hold");
        lab.step(0.02);
        let obs = lab.observe();
        let drone = obs.robots.iter().find(|r| r.id == "drone").unwrap();
        let hold = drone.hold_ned.expect("hold_ned after attach_hold");
        assert!(!drone.legal_cmds.is_empty());

        let mut bag = McapBag::new(Vec::new()).unwrap();
        bag.write_observation(&obs).unwrap();
        bag.write_action(&TimedAction {
            t: obs.t,
            action: AgentAction::new("drone", LabCmd::Hold),
        })
        .unwrap();
        let bytes = bag.finish().unwrap();

        let messages = observation_json(&bytes).unwrap();
        assert_eq!(messages.len(), 1);
        let robots = messages[0]["robots"].as_array().expect("robots");
        let read = robots.iter().find(|r| r["id"] == "drone").unwrap();
        let read_hold = read["hold_ned"].as_array().expect("hold_ned");
        assert_eq!(read_hold.len(), 3);
        for (a, b) in hold.iter().zip(read_hold) {
            assert!((a - b.as_f64().unwrap() as f32).abs() < 1e-5);
        }
        let cmds = read["legal_cmds"].as_array().expect("legal_cmds");
        assert!(!cmds.is_empty());
        assert!(read["aerial"]["kind"].is_string());
        assert!(messages[0]["properties"].as_array().unwrap().len() >= 21);
        assert!(messages[0]["sphere_hits"].is_array());

        let acts = action_json(&bytes).unwrap();
        assert_eq!(acts.len(), 1);
        assert_eq!(acts[0]["cmd"], "hold");
        assert_eq!(acts[0]["robot"], "drone");
    }
}
