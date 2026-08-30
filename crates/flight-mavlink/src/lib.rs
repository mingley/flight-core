//! MAVLink helpers for talking to PX4 and ArduPilot (and a UDP link).
//!
//! This is not a C++ binding and not a second protocol stack. It builds the
//! messages typed vehicle backends need: heartbeat, arm, PX4 offboard /
//! ArduPilot GUIDED, NAV land, flight termination, and NED velocity / position
//! setpoints.

#![deny(unsafe_code)]

use core::fmt;
use mavlink::common::*;
use mavlink::{connect, Connection, MavConnection, MavHeader};

/// PX4 `PX4_CUSTOM_MAIN_MODE_OFFBOARD`.
pub const PX4_MAIN_MODE_OFFBOARD: u8 = 6;
/// PX4 `PX4_CUSTOM_MAIN_MODE_AUTO`.
pub const PX4_MAIN_MODE_AUTO: u8 = 4;
/// PX4 `PX4_CUSTOM_SUB_MODE_AUTO_RTL`.
pub const PX4_SUB_MODE_AUTO_RTL: u8 = 5;
/// PX4 `PX4_CUSTOM_SUB_MODE_AUTO_LAND`.
pub const PX4_SUB_MODE_AUTO_LAND: u8 = 6;

/// ArduCopter `ModeGuided`. Companion offboard is this mode, not AUTO takeoff.
pub const ARDUPILOT_COPTER_GUIDED: u8 = 4;
/// ArduCopter `ModeRTL`. Failsafe-shaped for leftover Offboard, like PX4 AUTO+RTL.
pub const ARDUPILOT_COPTER_RTL: u8 = 6;
/// ArduCopter `ModeLand`. Land is not failsafe (same split as PX4 AUTO+LAND).
pub const ARDUPILOT_COPTER_LAND: u8 = 9;

/// Pack a PX4 custom_mode uint32 (`sub_mode << 24 | main_mode << 16`).
pub const fn px4_custom_mode(main_mode: u8, sub_mode: u8) -> u32 {
    ((sub_mode as u32) << 24) | ((main_mode as u32) << 16)
}

/// Unpack PX4 `custom_mode` main nibble (`(mode >> 16) & 0xff`).
pub const fn px4_custom_main_mode(custom_mode: u32) -> u8 {
    ((custom_mode >> 16) & 0xff) as u8
}

/// Unpack PX4 `custom_mode` sub nibble (`(mode >> 24) & 0xff`).
pub const fn px4_custom_sub_mode(custom_mode: u32) -> u8 {
    ((custom_mode >> 24) & 0xff) as u8
}

/// Vehicle HEARTBEAT that means the physical vehicle has left the companion's
/// offboard authority: critical/emergency/termination, or AUTO+RTL.
pub fn heartbeat_revokes_authority(h: &HEARTBEAT_DATA) -> bool {
    matches!(
        h.system_status,
        MavState::MAV_STATE_CRITICAL
            | MavState::MAV_STATE_EMERGENCY
            | MavState::MAV_STATE_FLIGHT_TERMINATION
    ) || (px4_custom_main_mode(h.custom_mode) == PX4_MAIN_MODE_AUTO
        && px4_custom_sub_mode(h.custom_mode) == PX4_SUB_MODE_AUTO_RTL)
}

/// `MAV_MODE_FLAG_SAFETY_ARMED` on a PX4-shaped heartbeat.
pub fn heartbeat_reports_armed(h: &HEARTBEAT_DATA) -> bool {
    h.base_mode
        .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
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

/// ArduCopter GUIDED via `MAV_CMD_DO_SET_MODE`. `param2` is the Copter mode
/// number, not a PX4 packed `custom_mode`. Companion `takeoff_now` must
/// re-assert this, not `MAV_CMD_NAV_TAKEOFF` (AUTO).
pub fn set_guided_mode(target_system: u8, target_component: u8, armed: bool) -> MavMessage {
    let mut base =
        MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED;
    if armed {
        base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: base.bits() as f32,
        param2: f32::from(ARDUPILOT_COPTER_GUIDED),
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

/// Copter HEARTBEAT: critical/emergency/termination, or RTL. LAND is not
/// failsafe. Uses Copter `custom_mode` numbers, not PX4 packed AUTO+RTL.
pub fn ardupilot_heartbeat_revokes_authority(h: &HEARTBEAT_DATA) -> bool {
    matches!(
        h.system_status,
        MavState::MAV_STATE_CRITICAL
            | MavState::MAV_STATE_EMERGENCY
            | MavState::MAV_STATE_FLIGHT_TERMINATION
    ) || h.custom_mode == u32::from(ARDUPILOT_COPTER_RTL)
}

/// Heartbeat an ArduCopter-shaped plant publishes.
pub fn ardupilot_vehicle_heartbeat(armed: bool, custom_mode: u32) -> MavMessage {
    ardupilot_vehicle_heartbeat_status(armed, custom_mode, MavState::MAV_STATE_ACTIVE)
}

/// Same as [`ardupilot_vehicle_heartbeat`] with an explicit `system_status`.
pub fn ardupilot_vehicle_heartbeat_status(
    armed: bool,
    custom_mode: u32,
    system_status: MavState,
) -> MavMessage {
    let mut base =
        MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED;
    if armed {
        base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: base,
        system_status,
        mavlink_version: 3,
    })
}

/// Heartbeat a PX4-shaped plant publishes (quadrotor + PX4 autopilot).
pub fn px4_vehicle_heartbeat(armed: bool, custom_mode: u32) -> MavMessage {
    px4_vehicle_heartbeat_status(armed, custom_mode, MavState::MAV_STATE_ACTIVE)
}

/// Same as [`px4_vehicle_heartbeat`] with an explicit `system_status`.
pub fn px4_vehicle_heartbeat_status(
    armed: bool,
    custom_mode: u32,
    system_status: MavState,
) -> MavMessage {
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
        system_status,
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
        assert_eq!(px4_custom_main_mode(px4_custom_mode(6, 0)), 6);
        assert_eq!(
            px4_custom_sub_mode(px4_custom_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL)),
            PX4_SUB_MODE_AUTO_RTL
        );
    }

    #[test]
    fn heartbeat_revokes_on_critical_and_rtl_not_on_active_offboard() {
        let MavMessage::HEARTBEAT(ok) =
            px4_vehicle_heartbeat(true, px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0))
        else {
            panic!("hb");
        };
        assert!(!heartbeat_revokes_authority(&ok));
        assert!(heartbeat_reports_armed(&ok));
        let MavMessage::HEARTBEAT(crit) = px4_vehicle_heartbeat_status(
            true,
            px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0),
            MavState::MAV_STATE_CRITICAL,
        ) else {
            panic!("crit");
        };
        assert!(heartbeat_revokes_authority(&crit));
        let MavMessage::HEARTBEAT(rtl) = px4_vehicle_heartbeat(
            true,
            px4_custom_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL),
        ) else {
            panic!("rtl");
        };
        assert!(heartbeat_revokes_authority(&rtl));
        let MavMessage::HEARTBEAT(land) = px4_vehicle_heartbeat(
            true,
            px4_custom_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND),
        ) else {
            panic!("land");
        };
        assert!(
            !heartbeat_revokes_authority(&land),
            "NAV_LAND AUTO LAND must not look like failsafe"
        );
    }

    #[test]
    fn ardupilot_revokes_on_rtl_and_critical_not_on_guided_or_land() {
        let MavMessage::HEARTBEAT(ok) =
            ardupilot_vehicle_heartbeat(true, u32::from(ARDUPILOT_COPTER_GUIDED))
        else {
            panic!("hb");
        };
        assert!(!ardupilot_heartbeat_revokes_authority(&ok));
        assert!(heartbeat_reports_armed(&ok));
        assert_eq!(ok.autopilot, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA);
        let MavMessage::HEARTBEAT(crit) = ardupilot_vehicle_heartbeat_status(
            true,
            u32::from(ARDUPILOT_COPTER_GUIDED),
            MavState::MAV_STATE_CRITICAL,
        ) else {
            panic!("crit");
        };
        assert!(ardupilot_heartbeat_revokes_authority(&crit));
        let MavMessage::HEARTBEAT(rtl) =
            ardupilot_vehicle_heartbeat(true, u32::from(ARDUPILOT_COPTER_RTL))
        else {
            panic!("rtl");
        };
        assert!(ardupilot_heartbeat_revokes_authority(&rtl));
        let MavMessage::HEARTBEAT(land) =
            ardupilot_vehicle_heartbeat(true, u32::from(ARDUPILOT_COPTER_LAND))
        else {
            panic!("land");
        };
        assert!(
            !ardupilot_heartbeat_revokes_authority(&land),
            "Copter LAND must not look like failsafe"
        );
        let MavMessage::HEARTBEAT(px4_rtl) = px4_vehicle_heartbeat(
            true,
            px4_custom_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL),
        ) else {
            panic!("px4 rtl");
        };
        assert!(
            !ardupilot_heartbeat_revokes_authority(&px4_rtl),
            "PX4 packed AUTO+RTL is not Copter RTL"
        );
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
        let guided = set_guided_mode(1, 1, true);
        let MavMessage::COMMAND_LONG(g) = &guided else {
            panic!("guided");
        };
        assert_eq!(g.command, MavCmd::MAV_CMD_DO_SET_MODE);
        assert!((g.param2 - f32::from(ARDUPILOT_COPTER_GUIDED)).abs() < 1e-6);
        assert!((g.param2 - f32::from(PX4_MAIN_MODE_OFFBOARD)).abs() > 1.0);
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
