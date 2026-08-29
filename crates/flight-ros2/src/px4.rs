//! PX4 ROS 2 `px4_msgs` setpoints, serialized as ROS 2 CDR little-endian.
//!
//! These are the messages the PX4 ROS 2 Interface Library publishes on
//! `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint`. Layout
//! matches PX4 1.15 (`OffboardControlMode` has `thrust_and_torque` /
//! `direct_actuator`). `NaN` means "this field is unused", as in PX4.
//!
//! No `rcl` / `px4_msgs` C library is required. An `rclrs` node can still
//! publish the bytes through a bridge, or use `geometry_msgs/Twist` (ENU) via
//! the `rclrs` feature.

use crate::OffboardSetpoint;
use flight_core::frames::Ned;
use flight_core::vector::Velocity;

/// ROS 2 CDR encapsulation: little-endian, 16-bit representation identifier 1.
pub const CDR_ENCAPSULATION_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// `px4_msgs/msg/OffboardControlMode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OffboardControlMode {
    pub timestamp_us: u64,
    pub position: bool,
    pub velocity: bool,
    pub acceleration: bool,
    pub attitude: bool,
    pub body_rate: bool,
    pub thrust_and_torque: bool,
    pub direct_actuator: bool,
}

impl OffboardControlMode {
    pub fn velocity_only(timestamp_us: u64) -> Self {
        Self {
            timestamp_us,
            position: false,
            velocity: true,
            acceleration: false,
            attitude: false,
            body_rate: false,
            thrust_and_torque: false,
            direct_actuator: false,
        }
    }

    pub fn from_setpoint(timestamp_us: u64, sp: &OffboardSetpoint) -> Self {
        Self {
            timestamp_us,
            position: sp.position_ned.is_some(),
            velocity: sp.velocity_ned.is_some(),
            acceleration: false,
            attitude: false,
            body_rate: false,
            thrust_and_torque: false,
            direct_actuator: false,
        }
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.u64(self.timestamp_us);
        w.bool(self.position);
        w.bool(self.velocity);
        w.bool(self.acceleration);
        w.bool(self.attitude);
        w.bool(self.body_rate);
        w.bool(self.thrust_and_torque);
        w.bool(self.direct_actuator);
        w.finish()
    }

    pub fn from_cdr(bytes: &[u8]) -> Option<Self> {
        let mut r = CdrReader::new(bytes)?;
        Some(Self {
            timestamp_us: r.u64()?,
            position: r.bool()?,
            velocity: r.bool()?,
            acceleration: r.bool()?,
            attitude: r.bool()?,
            body_rate: r.bool()?,
            thrust_and_torque: r.bool()?,
            direct_actuator: r.bool()?,
        })
    }
}

/// `px4_msgs/msg/TrajectorySetpoint`. Unused axes are `NaN`.
#[derive(Clone, Copy, Debug)]
pub struct TrajectorySetpoint {
    pub timestamp_us: u64,
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub acceleration: [f32; 3],
    pub jerk: [f32; 3],
    pub yaw: f32,
    pub yawspeed: f32,
}

impl PartialEq for TrajectorySetpoint {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_us == other.timestamp_us
            && bits3(self.position) == bits3(other.position)
            && bits3(self.velocity) == bits3(other.velocity)
            && bits3(self.acceleration) == bits3(other.acceleration)
            && bits3(self.jerk) == bits3(other.jerk)
            && self.yaw.to_bits() == other.yaw.to_bits()
            && self.yawspeed.to_bits() == other.yawspeed.to_bits()
    }
}

fn bits3(v: [f32; 3]) -> [u32; 3] {
    [v[0].to_bits(), v[1].to_bits(), v[2].to_bits()]
}

impl TrajectorySetpoint {
    pub fn unset(timestamp_us: u64) -> Self {
        Self {
            timestamp_us,
            position: [f32::NAN; 3],
            velocity: [f32::NAN; 3],
            acceleration: [f32::NAN; 3],
            jerk: [f32::NAN; 3],
            yaw: f32::NAN,
            yawspeed: f32::NAN,
        }
    }

    pub fn velocity_ned(timestamp_us: u64, v: Velocity<Ned>) -> Self {
        let mut s = Self::unset(timestamp_us);
        s.velocity = v.xyz();
        s
    }

    pub fn from_setpoint(timestamp_us: u64, sp: &OffboardSetpoint) -> Self {
        let mut s = Self::unset(timestamp_us);
        if let Some(p) = sp.position_ned {
            s.position = p.xyz();
        }
        if let Some(v) = sp.velocity_ned {
            s.velocity = v.xyz();
        }
        if let Some(yaw) = sp.yaw_rad {
            s.yaw = yaw;
        }
        s
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.u64(self.timestamp_us);
        w.f32s(&self.position);
        w.f32s(&self.velocity);
        w.f32s(&self.acceleration);
        w.f32s(&self.jerk);
        w.f32(self.yaw);
        w.f32(self.yawspeed);
        w.finish()
    }

    pub fn from_cdr(bytes: &[u8]) -> Option<Self> {
        let mut r = CdrReader::new(bytes)?;
        Some(Self {
            timestamp_us: r.u64()?,
            position: r.f32s()?,
            velocity: r.f32s()?,
            acceleration: r.f32s()?,
            jerk: r.f32s()?,
            yaw: r.f32()?,
            yawspeed: r.f32()?,
        })
    }
}

/// Pair the PX4 offboard mode flag with the matching trajectory setpoint.
pub fn px4_offboard_pair(
    timestamp_us: u64,
    sp: &OffboardSetpoint,
) -> (OffboardControlMode, TrajectorySetpoint) {
    (
        OffboardControlMode::from_setpoint(timestamp_us, sp),
        TrajectorySetpoint::from_setpoint(timestamp_us, sp),
    )
}

struct CdrWriter {
    payload: Vec<u8>,
}

impl CdrWriter {
    fn new() -> Self {
        Self {
            payload: Vec::with_capacity(64),
        }
    }

    fn align(&mut self, n: usize) {
        let rem = self.payload.len() % n;
        if rem != 0 {
            self.payload.resize(self.payload.len() + (n - rem), 0);
        }
    }

    fn u64(&mut self, v: u64) {
        self.align(8);
        self.payload.extend_from_slice(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.align(4);
        self.payload.extend_from_slice(&v.to_le_bytes());
    }

    fn f32s(&mut self, v: &[f32; 3]) {
        for c in v {
            self.f32(*c);
        }
    }

    fn bool(&mut self, v: bool) {
        self.payload.push(u8::from(v));
    }

    fn finish(self) -> Vec<u8> {
        let mut out = Vec::with_capacity(4 + self.payload.len());
        out.extend_from_slice(&CDR_ENCAPSULATION_LE);
        out.extend(self.payload);
        out
    }
}

struct CdrReader<'a> {
    payload: &'a [u8],
    pos: usize,
}

impl<'a> CdrReader<'a> {
    fn new(bytes: &'a [u8]) -> Option<Self> {
        if bytes.len() < 4 || bytes[..4] != CDR_ENCAPSULATION_LE {
            return None;
        }
        Some(Self {
            payload: &bytes[4..],
            pos: 0,
        })
    }

    fn align(&mut self, n: usize) -> Option<()> {
        let rem = self.pos % n;
        if rem != 0 {
            self.pos = self.pos.checked_add(n - rem)?;
        }
        Some(())
    }

    fn u64(&mut self) -> Option<u64> {
        self.align(8)?;
        let end = self.pos.checked_add(8)?;
        let slice = self.payload.get(self.pos..end)?;
        self.pos = end;
        Some(u64::from_le_bytes(slice.try_into().ok()?))
    }

    fn f32(&mut self) -> Option<f32> {
        self.align(4)?;
        let end = self.pos.checked_add(4)?;
        let slice = self.payload.get(self.pos..end)?;
        self.pos = end;
        Some(f32::from_le_bytes(slice.try_into().ok()?))
    }

    fn f32s(&mut self) -> Option<[f32; 3]> {
        Some([self.f32()?, self.f32()?, self.f32()?])
    }

    fn bool(&mut self) -> Option<bool> {
        let b = *self.payload.get(self.pos)?;
        self.pos += 1;
        Some(b != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ExternalFlightMode, VelocityMode};
    use flight_core::vector::Position;

    #[test]
    fn velocity_setpoint_is_ned_and_nans_the_rest() {
        let v = Velocity::<Ned>::ned(1.0, -0.2, 0.3);
        let s = TrajectorySetpoint::velocity_ned(42, v);
        assert_eq!(s.velocity, [1.0, -0.2, 0.3]);
        assert!(s.position.iter().all(|c| c.is_nan()));
        assert!(s.yaw.is_nan());
        let mode = OffboardControlMode::velocity_only(42);
        assert!(mode.velocity && !mode.position);
    }

    #[test]
    fn trajectory_cdr_roundtrip_keeps_nan_bits() {
        let mut sp = OffboardSetpoint {
            velocity_ned: Some(Velocity::<Ned>::ned(0.5, 0.0, -0.1)),
            position_ned: None,
            yaw_rad: Some(1.25),
        };
        let s = TrajectorySetpoint::from_setpoint(7, &sp);
        let bytes = s.to_cdr();
        assert_eq!(&bytes[..4], &CDR_ENCAPSULATION_LE);
        assert_eq!(TrajectorySetpoint::from_cdr(&bytes).unwrap(), s);
        sp.position_ned = Some(Position::<Ned>::ned(2.0, 3.0, 4.0));
        let (mode, traj) = px4_offboard_pair(8, &sp);
        assert!(mode.position && mode.velocity);
        assert_eq!(OffboardControlMode::from_cdr(&mode.to_cdr()).unwrap(), mode);
        assert_eq!(TrajectorySetpoint::from_cdr(&traj.to_cdr()).unwrap(), traj);
    }

    #[test]
    fn velocity_mode_feeds_px4_pair() {
        let mut m = VelocityMode::new(Velocity::<Ned>::ned(0.0, 1.0, 0.0));
        m.on_activate();
        let sp = m.update(0.02);
        let (mode, traj) = px4_offboard_pair(1, &sp);
        assert!(mode.velocity && !mode.position);
        assert_eq!(traj.velocity, [0.0, 1.0, 0.0]);
    }

    #[test]
    fn rejects_wrong_encapsulation() {
        assert!(OffboardControlMode::from_cdr(&[0, 0, 0, 0]).is_none());
        assert!(TrajectorySetpoint::from_cdr(&[]).is_none());
    }
}
