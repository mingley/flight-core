//! Natural-language tags and closed write sets that compile into a reference file.

use robot_lab::{LabCmd, RobotView};

use crate::limits::DriverLimits;

pub const DEVICE_LAB: &str = "lab";
pub const DEVICE_ENV: &str = "env";

/// One tag: prose an operator (or interviewing agent) would write, compiled
/// into structured measures / writes / limits — not left as the safety boundary.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct DeviceTag {
    pub key: String,
    pub prose: String,
}

pub(crate) fn tag(key: &str, prose: &str) -> DeviceTag {
    DeviceTag {
        key: key.into(),
        prose: prose.into(),
    }
}

pub(crate) fn catalog_mass_kg(id: &str) -> Option<f32> {
    match id {
        "drone" => Some(1.5),
        "rover" => Some(28.0),
        "skiff" => Some(80.0),
        "surveyor" => Some(18.0),
        _ => None,
    }
}

/// LabCmd values this body kind can ever accept (not only `legal_now`).
pub(crate) fn domain_writes(domain: &str) -> &'static [LabCmd] {
    match domain {
        "aerial" => &[
            LabCmd::Arm,
            LabCmd::Disarm,
            LabCmd::Offboard,
            LabCmd::EnableActuators,
            LabCmd::Takeoff,
            LabCmd::Airborne,
            LabCmd::Land,
            LabCmd::Touchdown,
            LabCmd::Failsafe,
            LabCmd::Velocity,
            LabCmd::Position,
            LabCmd::Hold,
            LabCmd::Recover,
            LabCmd::SetCharge,
        ],
        "ground" => &[
            LabCmd::Release,
            LabCmd::Drive,
            LabCmd::Halt,
            LabCmd::Hold,
            LabCmd::Estop,
            LabCmd::Clear,
            LabCmd::SetCharge,
        ],
        "surface" | "underwater" => &[
            LabCmd::Undock,
            LabCmd::Thrust,
            LabCmd::Hold,
            LabCmd::Dock,
            LabCmd::Station,
            LabCmd::Resume,
            LabCmd::Failsafe,
            LabCmd::Recover,
            LabCmd::SetCharge,
        ],
        "environment" => &[LabCmd::SetWind, LabCmd::SetWaves, LabCmd::SetCurrent],
        _ => &[],
    }
}

pub(crate) fn robot_tags(r: &RobotView) -> Vec<DeviceTag> {
    let mut tags = vec![
        tag(
            "frame",
            "NED metres, z-down. Pose n,e,d and velocity vn,ve,vd share that frame.",
        ),
        tag(
            "plant",
            "Rigid-body plant truth. The nav filter never writes the plant quaternion.",
        ),
        tag(
            "safety",
            "Illegal motion is unrepresentable or rejected by the same aerial / ground / marine machines. Driver limits are additional numeric clamps, not a prompt.",
        ),
        tag(
            "hold",
            "Hold is current-pose NED (plant hold_ned). Aerial: OffboardControl. Ground: Moving. Marine: CanThrust (not StationKeep). Position is aerial-only.",
        ),
    ];
    match r.domain.as_str() {
        "aerial" => tags.extend([
            tag(
                "takeoff",
                "Takeoff from Ready is an attach grant (P2): kernel Takeoff from Ready stays illegal on Lab::act. OffboardControl gates velocity / position / hold.",
            ),
            tag(
                "imu",
                "Readable imu_healthy and estimator_valid. Unusable IMU may clear estimator_valid without writing plant attitude.",
            ),
        ]),
        "ground" => tags.push(tag(
            "drive",
            "Drive and ground hold require Moving. Parked / EStop are compile-fail on typestate and not-legal on the driver.",
        )),
        "surface" | "underwater" => tags.push(tag(
            "thrust",
            "Thrust and marine DP require CanThrust (Underway or StationKeep). Docked / Failsafe compile-fail. Do not declare_failsafe on Docked (P3).",
        )),
        _ => {}
    }
    tags
}

pub(crate) fn env_tags() -> Vec<DeviceTag> {
    vec![
        tag(
            "environment",
            "Wind, waves, and current are always-legal env_cmds. They need no robot id.",
        ),
        tag(
            "hydro",
            "Conserved shallow-water heightfield. GPU is optional performance, not a second plant.",
        ),
    ]
}

pub(crate) fn lab_tags() -> Vec<DeviceTag> {
    vec![
        tag(
            "lab",
            "The verified world: catalogs, 22 named properties, lab certificates. Observe does not step.",
        ),
        tag(
            "step",
            "P12: flush all granted setpoints, then one WorldSession::step. Chain files step explicitly; writes do not.",
        ),
        tag(
            "catalog",
            "P11: inland omits hulls; open_water omits the rover. Missing bodies are omitted, not placeholders.",
        ),
        tag(
            "conformance",
            "This adapter is MHS-shaped. Official MHS is a research preview and not open-sourced. official=false.",
        ),
    ]
}

pub(crate) fn speed_limit(domain: &str, limits: &DriverLimits) -> Option<(f32, &'static str)> {
    match domain {
        "aerial" => Some((limits.aerial_speed_mps, "velocity")),
        "ground" => Some((limits.ground_speed_mps, "drive")),
        "surface" | "underwater" => Some((limits.marine_speed_mps, "thrust")),
        _ => None,
    }
}
