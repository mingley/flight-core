//! MAVLink helpers for talking to PX4 (and a UDP link).
//!
//! This is not a C++ binding. It builds the messages a typed vehicle backend
//! needs: heartbeat, arm, offboard mode, NAV takeoff/land/loiter, flight termination, and NED velocity / position setpoints.

#![deny(unsafe_code)]

use core::fmt;
use mavlink::common::*;
use mavlink::{connect, Connection, MavConnection, MavHeader};

/// PX4 `PX4_CUSTOM_MAIN_MODE_OFFBOARD`.
pub const PX4_MAIN_MODE_OFFBOARD: u8 = 6;
/// PX4 `PX4_CUSTOM_MAIN_MODE_AUTO`.
pub const PX4_MAIN_MODE_AUTO: u8 = 4;

/// Pack a PX4 custom_mode uint32 (`sub_mode << 24 | main_mode << 16`).
pub const fn px4_custom_mode(main_mode: u8, sub_mode: u8) -> u32 {
    ((sub_mode as u32) << 24) | ((main_mode as u32) << 16)
}

/// Ignore position, acceleration, force, yaw, yaw-rate — velocity only.
pub fn velocity_only_mask() -> PositionTargetTypemask {
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
}

/// Ignore velocity, acceleration, force, yaw, yaw-rate — position only.
pub fn position_only_mask() -> PositionTargetTypemask {
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
}

pub fn gcs_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

pub fn arm_disarm(target_system: u8, target_component: u8, arm: bool) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: if arm { 1.0 } else { 0.0 },
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        target_system,
        target_component,
        confirmation: 0,
    })
}

pub fn set_offboard_mode(target_system: u8, target_component: u8, armed: bool) -> MavMessage {
    let mut base = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    // PX4 `MAV_CMD_DO_SET_MODE` reads param2/param3 as *unpacked* main/sub
    // (`uint8_t custom_main_mode = (uint8_t)cmd.param2`). Packed
    // `px4_custom_mode` belongs on HEARTBEAT / SET_MODE, not here — sending
    // `6 << 16` as f32 truncates to main mode 0 ("Unsupported main mode").
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: base.bits() as f32,
        param2: f32::from(PX4_MAIN_MODE_OFFBOARD),
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        target_system,
        target_component,
        confirmation: 0,
    })
}

pub fn nav_takeoff(target_system: u8, target_component: u8, altitude_m: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: altitude_m,
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        target_system,
        target_component,
        confirmation: 0,
    })
}

pub fn nav_land(target_system: u8, target_component: u8) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_NAV_LAND,
        target_system,
        target_component,
        confirmation: 0,
    })
}

/// `MAV_CMD_NAV_LOITER_UNLIM`. Companion climb-complete: hold here.
pub fn nav_loiter_unlim(target_system: u8, target_component: u8) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_NAV_LOITER_UNLIM,
        target_system,
        target_component,
        confirmation: 0,
    })
}

/// `MAV_CMD_DO_FLIGHTTERMINATION`. `param1 >= 0.5` activates.
pub fn flight_termination(target_system: u8, target_component: u8, terminate: bool) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: if terminate { 1.0 } else { 0.0 },
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_DO_FLIGHTTERMINATION,
        target_system,
        target_component,
        confirmation: 0,
    })
}

pub fn set_velocity_ned(
    target_system: u8,
    target_component: u8,
    time_boot_ms: u32,
    vn: f32,
    ve: f32,
    vd: f32,
) -> MavMessage {
    MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
        time_boot_ms,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: vn,
        vy: ve,
        vz: vd,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        type_mask: velocity_only_mask(),
        target_system,
        target_component,
        coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
    })
}

pub fn set_position_ned(
    target_system: u8,
    target_component: u8,
    time_boot_ms: u32,
    n: f32,
    e: f32,
    d: f32,
) -> MavMessage {
    MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
        time_boot_ms,
        x: n,
        y: e,
        z: d,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        type_mask: position_only_mask(),
        target_system,
        target_component,
        coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
    })
}

/// NED velocity encoded in a PX4 offboard setpoint, if the mask leaves velocity live.
pub fn ned_velocity_from_target(d: &SET_POSITION_TARGET_LOCAL_NED_DATA) -> Option<(f32, f32, f32)> {
    let mask = d.type_mask;
    let vx_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE);
    let vy_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE);
    let vz_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE);
    if vx_off && vy_off && vz_off {
        return None;
    }
    Some((
        if vx_off { 0.0 } else { d.vx },
        if vy_off { 0.0 } else { d.vy },
        if vz_off { 0.0 } else { d.vz },
    ))
}

/// NED position encoded in a PX4 offboard setpoint, if the mask leaves pose live.
pub fn ned_position_from_target(d: &SET_POSITION_TARGET_LOCAL_NED_DATA) -> Option<(f32, f32, f32)> {
    let mask = d.type_mask;
    let x_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE);
    let y_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE);
    let z_off = mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE);
    if x_off && y_off && z_off {
        return None;
    }
    Some((
        if x_off { 0.0 } else { d.x },
        if y_off { 0.0 } else { d.y },
        if z_off { 0.0 } else { d.z },
    ))
}

/// Pose the PX4 companion already knows how to read (`Px4Backend::tick`).
pub fn local_position_ned(
    time_boot_ms: u32,
    n: f32,
    e: f32,
    d: f32,
    vn: f32,
    ve: f32,
    vd: f32,
) -> MavMessage {
    MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
        time_boot_ms,
        x: n,
        y: e,
        z: d,
        vx: vn,
        vy: ve,
        vz: vd,
    })
}

/// Heartbeat a PX4-shaped plant publishes (quadrotor + PX4 autopilot).
pub fn px4_vehicle_heartbeat(armed: bool, custom_mode: u32) -> MavMessage {
    let mut base =
        MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED;
    if armed {
        base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
        base_mode: base,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

pub fn header(system_id: u8, component_id: u8, sequence: u8) -> MavHeader {
    MavHeader {
        system_id,
        component_id,
        sequence,
    }
}

/// UDP MAVLink link (`udpin:0.0.0.0:14540`, `udpout:127.0.0.1:14580`, …).
pub struct UdpLink {
    conn: Connection<MavMessage>,
    header: MavHeader,
}

impl fmt::Debug for UdpLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UdpLink")
            .field("header", &self.header)
            .finish_non_exhaustive()
    }
}

impl UdpLink {
    pub fn connect(address: &str, system_id: u8, component_id: u8) -> Result<Self, String> {
        let conn = connect::<MavMessage>(address).map_err(|e| e.to_string())?;
        Ok(Self {
            conn,
            header: header(system_id, component_id, 0),
        })
    }

    pub fn send(&mut self, msg: &MavMessage) -> Result<(), String> {
        self.header.sequence = self.header.sequence.wrapping_add(1);
        self.conn
            .send(&self.header, msg)
            .map(|_| ())
            .map_err(|e| e.to_string())
    }

    pub fn recv(&mut self) -> Result<(MavHeader, MavMessage), String> {
        self.conn.recv().map_err(|e| e.to_string())
    }

    /// Non-blocking read. `Err` means no complete frame is waiting (or I/O).
    pub fn try_recv(&mut self) -> Result<(MavHeader, MavMessage), String> {
        self.conn.try_recv().map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn custom_mode_packing() {
        assert_eq!(px4_custom_mode(6, 0), 6 << 16);
        assert_eq!(px4_custom_mode(4, 3), (3 << 24) | (4 << 16));
    }

    #[test]
    fn velocity_mask_ignores_position() {
        let mask = velocity_only_mask();
        assert!(mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE));
        assert!(!mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE));
    }

    #[test]
    fn position_mask_ignores_velocity() {
        let mask = position_only_mask();
        assert!(mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE));
        assert!(!mask.contains(PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE));
    }

    #[test]
    fn messages_construct() {
        let _ = gcs_heartbeat();
        let _ = arm_disarm(1, 1, true);
        let offboard = set_offboard_mode(1, 1, true);
        let MavMessage::COMMAND_LONG(m) = &offboard else {
            panic!("offboard");
        };
        assert_eq!(m.command, MavCmd::MAV_CMD_DO_SET_MODE);
        assert!((m.param2 - f32::from(PX4_MAIN_MODE_OFFBOARD)).abs() < 1e-6);
        let takeoff = nav_takeoff(1, 1, 3.0);
        let MavMessage::COMMAND_LONG(t) = &takeoff else {
            panic!("takeoff");
        };
        assert_eq!(t.command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert!((t.param7 - 3.0).abs() < 1e-6);
        let land = nav_land(1, 1);
        let MavMessage::COMMAND_LONG(l) = &land else {
            panic!("land");
        };
        assert_eq!(l.command, MavCmd::MAV_CMD_NAV_LAND);
        let loiter = nav_loiter_unlim(1, 1);
        let MavMessage::COMMAND_LONG(o) = &loiter else {
            panic!("loiter");
        };
        assert_eq!(o.command, MavCmd::MAV_CMD_NAV_LOITER_UNLIM);
        let term = flight_termination(1, 1, true);
        let MavMessage::COMMAND_LONG(f) = &term else {
            panic!("termination");
        };
        assert_eq!(f.command, MavCmd::MAV_CMD_DO_FLIGHTTERMINATION);
        assert!(f.param1 > 0.5);
        let msg = set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.0);
        let MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) = &msg else {
            panic!("velocity setpoint");
        };
        assert_eq!(ned_velocity_from_target(d), Some((0.0, 0.0, -1.0)));
        assert_eq!(ned_position_from_target(d), None);
        let pose = set_position_ned(1, 1, 0, 1.0, -2.0, -4.0);
        let MavMessage::SET_POSITION_TARGET_LOCAL_NED(p) = &pose else {
            panic!("position setpoint");
        };
        assert_eq!(ned_position_from_target(p), Some((1.0, -2.0, -4.0)));
        assert_eq!(ned_velocity_from_target(p), None);
        let _ = local_position_ned(0, 1.0, 2.0, -3.0, 0.0, 0.0, 0.0);
        let _ = px4_vehicle_heartbeat(true, px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0));
    }
}
