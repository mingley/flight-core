use flight_core::domain::Domain;
use flight_core::ground::{ground_step, GroundEvent};
use flight_core::marine::{marine_step, MarineEvent};
use flight_core::safety::{self, Event};
use robot_world::Body;
use serde::{Deserialize, Serialize};

/// Closed set of lab / research commands. JSON keeps the same snake_case
/// strings (`"drive"`, `"undock"`, `"set_wind"`, …); unknown names fail
/// deserialize instead of reaching [`Lab::act`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LabCmd {
    Arm,
    Disarm,
    Offboard,
    EnableActuators,
    Takeoff,
    Airborne,
    Land,
    Touchdown,
    Failsafe,
    Velocity,
    Position,
    Hold,
    Drive,
    Thrust,
    Release,
    Halt,
    Park,
    Estop,
    Clear,
    Undock,
    Dock,
    Station,
    Resume,
    Recover,
    SetCharge,
    SetWind,
    SetWaves,
    SetCurrent,
}

impl LabCmd {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Arm => "arm",
            Self::Disarm => "disarm",
            Self::Offboard => "offboard",
            Self::EnableActuators => "enable_actuators",
            Self::Takeoff => "takeoff",
            Self::Airborne => "airborne",
            Self::Land => "land",
            Self::Touchdown => "touchdown",
            Self::Failsafe => "failsafe",
            Self::Velocity => "velocity",
            Self::Position => "position",
            Self::Hold => "hold",
            Self::Drive => "drive",
            Self::Thrust => "thrust",
            Self::Release => "release",
            Self::Halt => "halt",
            Self::Park => "park",
            Self::Estop => "estop",
            Self::Clear => "clear",
            Self::Undock => "undock",
            Self::Dock => "dock",
            Self::Station => "station",
            Self::Resume => "resume",
            Self::Recover => "recover",
            Self::SetCharge => "set_charge",
            Self::SetWind => "set_wind",
            Self::SetWaves => "set_waves",
            Self::SetCurrent => "set_current",
        }
    }

    pub const ALL: [LabCmd; 28] = [
        Self::Arm,
        Self::Disarm,
        Self::Offboard,
        Self::EnableActuators,
        Self::Takeoff,
        Self::Airborne,
        Self::Land,
        Self::Touchdown,
        Self::Failsafe,
        Self::Velocity,
        Self::Position,
        Self::Hold,
        Self::Drive,
        Self::Thrust,
        Self::Release,
        Self::Halt,
        Self::Park,
        Self::Estop,
        Self::Clear,
        Self::Undock,
        Self::Dock,
        Self::Station,
        Self::Resume,
        Self::Recover,
        Self::SetCharge,
        Self::SetWind,
        Self::SetWaves,
        Self::SetCurrent,
    ];

    pub const ENV: [LabCmd; 3] = [Self::SetWind, Self::SetWaves, Self::SetCurrent];

    /// Whether [`Lab::act`] would pass the safety machine (not contact/wet/air gates).
    pub fn accepted_by(self, body: &Body) -> bool {
        match self {
            Self::SetWind | Self::SetWaves | Self::SetCurrent => false,
            Self::SetCharge => true,
            Self::Arm => aerial_ok(body, Event::Arm),
            Self::Disarm => aerial_ok(body, Event::Disarm),
            Self::Offboard => aerial_ok_seq(body, &[Event::HeartbeatFresh, Event::EnterOffboard]),
            Self::EnableActuators => aerial_ok(body, Event::EnableActuators),
            Self::Takeoff => aerial_ok(body, Event::Takeoff),
            Self::Airborne => aerial_ok(body, Event::ReachedAltitude),
            Self::Land => aerial_ok(body, Event::Land),
            Self::Touchdown => aerial_ok(body, Event::Touchdown),
            Self::Failsafe => match body.domain {
                Domain::Aerial => aerial_ok(body, Event::TriggerFailsafe),
                Domain::Ground => ground_ok(body, GroundEvent::EStop),
                Domain::Surface | Domain::Underwater => marine_ok(body, MarineEvent::Failsafe),
            },
            Self::Velocity | Self::Drive | Self::Thrust => match body.domain {
                Domain::Aerial => {
                    aerial_ok_seq(body, &[Event::HeartbeatFresh, Event::MissionCommand])
                }
                Domain::Ground => ground_ok(body, GroundEvent::DriveCommand),
                Domain::Surface | Domain::Underwater => marine_ok(body, MarineEvent::ThrustCommand),
            },
            Self::Position | Self::Hold => {
                body.domain == Domain::Aerial
                    && aerial_ok_seq(body, &[Event::HeartbeatFresh, Event::MissionCommand])
            }
            Self::Release => ground_ok(body, GroundEvent::Release),
            Self::Halt | Self::Park => ground_ok(body, GroundEvent::Halt),
            Self::Estop => ground_ok(body, GroundEvent::EStop),
            Self::Clear => ground_ok(body, GroundEvent::ClearEstop),
            Self::Undock => marine_ok(body, MarineEvent::Undock),
            Self::Dock => marine_ok(body, MarineEvent::Dock),
            Self::Station => marine_ok(body, MarineEvent::Station),
            Self::Resume => marine_ok(body, MarineEvent::Resume),
            Self::Recover => match body.domain {
                Domain::Aerial => {
                    aerial_ok(body, Event::Recover)
                        || aerial_ok_seq(body, &[Event::Disarm, Event::Recover])
                }
                Domain::Surface | Domain::Underwater => marine_ok(body, MarineEvent::Recover),
                Domain::Ground => false,
            },
        }
    }

    /// [`RobotView::legal_cmds`]: accepted, and not an alias of another name.
    pub(crate) fn on_legal_list(self, body: &Body) -> bool {
        if !self.accepted_by(body) {
            return false;
        }
        match self {
            Self::Park => false,
            Self::Failsafe if body.domain == Domain::Ground => false,
            Self::Velocity | Self::Position | Self::Hold => body.domain == Domain::Aerial,
            Self::Drive => body.domain == Domain::Ground,
            Self::Thrust => matches!(body.domain, Domain::Surface | Domain::Underwater),
            _ => true,
        }
    }
}

impl std::fmt::Display for LabCmd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

pub(crate) fn aerial_ok(body: &Body, e: Event) -> bool {
    body.aerial
        .map(|s| safety::step(s, e).is_ok())
        .unwrap_or(false)
}

pub(crate) fn aerial_ok_seq(body: &Body, events: &[Event]) -> bool {
    let Some(mut s) = body.aerial else {
        return false;
    };
    for &e in events {
        match safety::step(s, e) {
            Ok(n) => s = n,
            Err(_) => return false,
        }
    }
    true
}

pub(crate) fn ground_ok(body: &Body, e: GroundEvent) -> bool {
    body.ground
        .map(|s| ground_step(s, e).is_ok())
        .unwrap_or(false)
}

pub(crate) fn marine_ok(body: &Body, e: MarineEvent) -> bool {
    body.marine
        .map(|s| marine_step(s, e).is_ok())
        .unwrap_or(false)
}
