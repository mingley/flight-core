//! Wire format a HITL I/O card can speak without MAVLink or ROS.
//!
//! Little-endian. Magic `FCH1`. One datagram is one sample or one command.
//! Slots: 0 drone, 1 rover, 2 skiff, 3 surveyor. `Command.apply == 0` means
//! do not apply that slot's velocity (deadline miss / idle).

pub const MAGIC: [u8; 4] = *b"FCH1";
pub const VERSION: u8 = 1;
pub const KIND_SAMPLE: u8 = 1;
pub const KIND_COMMAND: u8 = 2;
pub const SAMPLE_LEN: usize = 48;
pub const COMMAND_LEN: usize = 24;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub slot: u8,
    pub t_plant_ns: u64,
    pub position_ned: [f32; 3],
    pub velocity_ned: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Command {
    pub slot: u8,
    pub velocity_ned: [f32; 3],
    /// 1 = apply, 0 = hold/zero (deadline miss on the companion).
    pub apply: u8,
}

pub fn encode_sample(s: Sample) -> [u8; SAMPLE_LEN] {
    let mut b = [0u8; SAMPLE_LEN];
    b[0..4].copy_from_slice(&MAGIC);
    b[4] = VERSION;
    b[5] = KIND_SAMPLE;
    b[6] = s.slot;
    b[8..16].copy_from_slice(&s.t_plant_ns.to_le_bytes());
    put_f32(&mut b[16..28], s.position_ned);
    put_f32(&mut b[28..40], s.velocity_ned);
    b
}

pub fn decode_sample(buf: &[u8]) -> Option<Sample> {
    if buf.len() < SAMPLE_LEN || buf[0..4] != MAGIC || buf[4] != VERSION || buf[5] != KIND_SAMPLE {
        return None;
    }
    Some(Sample {
        slot: buf[6],
        t_plant_ns: u64::from_le_bytes(buf[8..16].try_into().ok()?),
        position_ned: get_f32(&buf[16..28])?,
        velocity_ned: get_f32(&buf[28..40])?,
    })
}

pub fn encode_command(c: Command) -> [u8; COMMAND_LEN] {
    let mut b = [0u8; COMMAND_LEN];
    b[0..4].copy_from_slice(&MAGIC);
    b[4] = VERSION;
    b[5] = KIND_COMMAND;
    b[6] = c.slot;
    b[7] = c.apply;
    put_f32(&mut b[8..20], c.velocity_ned);
    b
}

pub fn decode_command(buf: &[u8]) -> Option<Command> {
    if buf.len() < COMMAND_LEN || buf[0..4] != MAGIC || buf[4] != VERSION || buf[5] != KIND_COMMAND
    {
        return None;
    }
    Some(Command {
        slot: buf[6],
        apply: buf[7],
        velocity_ned: get_f32(&buf[8..20])?,
    })
}

fn put_f32(dst: &mut [u8], v: [f32; 3]) {
    for (i, c) in v.iter().enumerate() {
        dst[i * 4..i * 4 + 4].copy_from_slice(&c.to_le_bytes());
    }
}

fn get_f32(src: &[u8]) -> Option<[f32; 3]> {
    Some([
        f32::from_le_bytes(src[0..4].try_into().ok()?),
        f32::from_le_bytes(src[4..8].try_into().ok()?),
        f32::from_le_bytes(src[8..12].try_into().ok()?),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_roundtrip() {
        let s = Sample {
            slot: 2,
            t_plant_ns: 1_234_567,
            position_ned: [1.5, -2.0, 0.25],
            velocity_ned: [0.1, 0.0, -0.4],
        };
        let b = encode_sample(s);
        assert_eq!(decode_sample(&b), Some(s));
        assert!(decode_sample(&b[..10]).is_none());
    }

    #[test]
    fn command_roundtrip() {
        let c = Command {
            slot: 0,
            velocity_ned: [0.0, 0.0, -1.2],
            apply: 1,
        };
        assert_eq!(decode_command(&encode_command(c)), Some(c));
        let mut bad = encode_command(c);
        bad[0] = b'X';
        assert!(decode_command(&bad).is_none());
    }

    #[test]
    fn apply_zero_roundtrip_keeps_velocity_payload() {
        let c = Command {
            slot: 1,
            velocity_ned: [-0.4, 0.0, 0.0],
            apply: 0,
        };
        assert_eq!(decode_command(&encode_command(c)), Some(c));
    }
}
