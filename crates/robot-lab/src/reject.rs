//! Structured attach/act rejection traces (NEXT A4).

use flight_core::domain::Domain;
use flight_core::vehicle::{aerial_kind, ground_kind, marine_kind, AerialKind, MarineKind};
use robot_world::Body;
use serde::Serialize;

use crate::cmd::LabCmd;
use crate::lab::{AgentAction, Lab, LabError};

/// Why an `act` / `act_through_attach` bounced, in a form an agent can log.
///
/// `reject` is the safety-enum (or lab-error) display. `invariant` is a
/// remaining-spec id when this split is one of P1–P13; omitted when the bounce
/// is ordinary “not legal now.”
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RejectTrace {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
    pub robot: String,
    pub cmd: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub from_phase: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub from_kind: Option<String>,
    /// Kernel event the command would have posted (`Takeoff`, `EnterOffboard`, …).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attempted: Option<String>,
    pub reject: String,
    pub code: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub invariant: Option<String>,
}

impl RejectTrace {
    pub(crate) fn capture(lab: &Lab, action: &AgentAction, err: &LabError) -> Self {
        lab.with_world(|world| {
            let body = world.body(action.robot.as_str());
            let domain = body.map(|b| b.domain.name().to_string());
            Self {
                domain,
                robot: action.robot.clone(),
                cmd: action.cmd.as_str().to_string(),
                from_phase: body.map(|b| b.phase_name().to_string()),
                from_kind: body.and_then(kind_name).map(str::to_string),
                attempted: attempted_event(action.cmd, body.map(|b| b.domain)).map(str::to_string),
                reject: reject_display(err),
                code: error_code(err).to_string(),
                invariant: invariant_id(world.scenario, action, err, body).map(str::to_string),
            }
        })
    }
}

impl Lab {
    /// Most recent failed [`Self::act`] / [`Self::act_through_attach`]. Cleared
    /// on the next successful act. Observation `message` also carries the text
    /// (`agent rejected: …`) so agents do not need a new schema field.
    pub fn last_reject(&self) -> Option<&RejectTrace> {
        self.reject_trace.as_ref()
    }

    pub(crate) fn note_reject(&mut self, action: &AgentAction, err: LabError) -> LabError {
        self.reject_trace = Some(RejectTrace::capture(self, action, &err));
        self.message = format!("agent rejected: {err}");
        err
    }

    pub(crate) fn clear_reject(&mut self) {
        self.reject_trace = None;
    }
}

fn kind_name(body: &Body) -> Option<&'static str> {
    if let Some(s) = body.aerial {
        return Some(aerial_kind(s).name());
    }
    if let Some(s) = body.ground {
        return Some(ground_kind(s).name());
    }
    body.marine.map(|s| marine_kind(s).name())
}

fn attempted_event(cmd: LabCmd, domain: Option<Domain>) -> Option<&'static str> {
    Some(match cmd {
        LabCmd::Arm => "Arm",
        LabCmd::Disarm => "Disarm",
        LabCmd::Offboard => "EnterOffboard",
        LabCmd::EnableActuators => "EnableActuators",
        LabCmd::Takeoff => "Takeoff",
        LabCmd::Airborne => "ReachedAltitude",
        LabCmd::Land => "Land",
        LabCmd::Touchdown => "Touchdown",
        LabCmd::Failsafe => match domain {
            Some(Domain::Ground) => "EStop",
            Some(Domain::Surface | Domain::Underwater) => "Failsafe",
            Some(Domain::Aerial) | None => "TriggerFailsafe",
        },
        LabCmd::Velocity | LabCmd::Position => "MissionCommand",
        LabCmd::Hold => match domain {
            Some(Domain::Ground) => "DriveCommand",
            Some(Domain::Surface | Domain::Underwater) => "ThrustCommand",
            _ => "MissionCommand",
        },
        LabCmd::Drive => "DriveCommand",
        LabCmd::Thrust => "ThrustCommand",
        LabCmd::Release => "Release",
        LabCmd::Halt | LabCmd::Park => "Halt",
        LabCmd::Estop => "EStop",
        LabCmd::Clear => "ClearEstop",
        LabCmd::Undock => "Undock",
        LabCmd::Dock => "Dock",
        LabCmd::Station => "Station",
        LabCmd::Resume => "Resume",
        LabCmd::Recover => "Recover",
        LabCmd::SetCharge | LabCmd::SetWind | LabCmd::SetWaves | LabCmd::SetCurrent => {
            return None;
        }
    })
}

fn error_code(err: &LabError) -> &'static str {
    match err {
        LabError::UnknownRobot(_) => "unknown_robot",
        LabError::UnknownCommand(_) => "unknown_command",
        LabError::UnknownScenario(_) => "unknown_scenario",
        LabError::WrongDomain => "wrong_domain",
        LabError::NotLegal { .. } => "not_legal",
        LabError::Aerial(_) => "aerial",
        LabError::Ground(_) => "ground",
        LabError::Marine(_) => "marine",
    }
}

fn reject_display(err: &LabError) -> String {
    match err {
        LabError::Aerial(r) => r.to_string(),
        LabError::Ground(r) => r.to_string(),
        LabError::Marine(r) => r.to_string(),
        other => other.to_string(),
    }
}

/// Remaining-spec split this bounce documents, when the pair is one of those
/// kernel-vs-typestate (or catalog) facts — not every `not_legal`.
fn invariant_id(
    scenario: &str,
    action: &AgentAction,
    err: &LabError,
    body: Option<&Body>,
) -> Option<&'static str> {
    match err {
        LabError::UnknownRobot(id) => catalog_omit(scenario, id),
        LabError::NotLegal { cmd, robot } => not_legal_invariant(scenario, robot, *cmd, body),
        LabError::Aerial(_) => aerial_invariant(action.cmd, body),
        LabError::Marine(_) => marine_invariant(action.cmd, body),
        LabError::WrongDomain if action.cmd == LabCmd::Disarm => Some("P6"),
        _ => None,
    }
}

fn catalog_omit(scenario: &str, id: &str) -> Option<&'static str> {
    let inland_hull = scenario == "inland" && matches!(id, "skiff" | "surveyor");
    let open_rover = scenario == "open_water" && id == "rover";
    (inland_hull || open_rover).then_some("P11")
}

fn not_legal_invariant(
    scenario: &str,
    robot: &str,
    cmd: LabCmd,
    body: Option<&Body>,
) -> Option<&'static str> {
    if let Some(p) = catalog_omit(scenario, robot) {
        return Some(p);
    }
    if let Some(p) = aerial_invariant(cmd, body) {
        return Some(p);
    }
    marine_invariant(cmd, body)
}

fn aerial_invariant(cmd: LabCmd, body: Option<&Body>) -> Option<&'static str> {
    let kind = body.and_then(|b| b.aerial).map(aerial_kind)?;
    match (cmd, kind) {
        (LabCmd::Offboard, AerialKind::PreflightReady) => Some("P1"),
        (LabCmd::Takeoff, AerialKind::PreflightReady) => Some("P2"),
        (LabCmd::Touchdown, AerialKind::Recovery) => Some("P4"),
        (LabCmd::Disarm, AerialKind::Recovery) => Some("P5"),
        _ => None,
    }
}

fn marine_invariant(cmd: LabCmd, body: Option<&Body>) -> Option<&'static str> {
    let kind = body.and_then(|b| b.marine).map(marine_kind)?;
    match (cmd, kind) {
        (LabCmd::Dock, MarineKind::Failsafe) | (LabCmd::Failsafe, MarineKind::Docked) => Some("P3"),
        _ => None,
    }
}
