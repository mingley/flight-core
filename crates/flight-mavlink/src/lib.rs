//! MAVLink helpers for talking to PX4 (and a UDP link).
//!
//! This is not a C++ binding. It builds the messages a typed vehicle backend
//! needs: heartbeat, arm, offboard mode, and NED velocity setpoints.

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
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: base.bits() as f32,
        param2: px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0) as f32,
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
    fn messages_construct() {
        let _ = gcs_heartbeat();
        let _ = arm_disarm(1, 1, true);
        let _ = set_offboard_mode(1, 1, true);
        let _ = set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.0);
    }
}
