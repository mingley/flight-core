use crate::{AgentAction, GroundKind, Lab, LabCmd, Observation};

use super::support::{cmd, note, robot, rover_drive_attached, rover_hold_attached};
use super::ResearchAgent;

/// Inland rover: probe parked drive, then attach drive and sweep south until
/// a sphere hit. Legal motion never goes through [`Lab::act`].
#[derive(Default)]
pub struct TypedCollisionSweep {
    pub(crate) probed: bool,
    pub(crate) hit: bool,
}

impl ResearchAgent for TypedCollisionSweep {
    fn name(&self) -> &'static str {
        "typed_collision_sweep"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(rover) = robot(obs, "rover") else {
            return Vec::new();
        };
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let hit_pair = rover.sphere_contact
            || drone.sphere_contact
            || obs
                .sphere_hits
                .iter()
                .any(|h| h.involves("rover") && h.involves("drone"));
        if hit_pair {
            self.hit = true;
            if rover.ground.as_ref().is_some_and(|g| g.drive_enabled)
                && lab.attach_park("rover").is_ok()
            {
                note(lab, cmd("rover", LabCmd::Halt, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if self.hit {
            return Vec::new();
        }
        let Some(g) = rover.ground.as_ref() else {
            return Vec::new();
        };
        if g.kind == GroundKind::EStopped {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if g.kind == GroundKind::Parked {
                return vec![cmd("rover", LabCmd::Drive, 0.0, -1.2, 0.0)];
            }
        }
        if g.kind == GroundKind::Parked {
            if lab.attach_drive("rover").is_ok() {
                note(lab, cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if g.kind != GroundKind::Moving {
            return Vec::new();
        }
        if !rover.terrain_contact {
            return Vec::new();
        }
        let vn = (drone.n - rover.n).signum() * 1.4;
        let ve = (drone.e - rover.e).signum() * 1.4;
        rover_drive_attached(lab, vn, ve, 0.0);
        Vec::new()
    }
}

/// Inland or coastal rover: probe parked drive, then trip E-stop from Parked
/// (no drive grant) and clear back to Parked. Legal trips never go through
/// [`Lab::act`].
#[derive(Default)]
pub struct TypedGroundEstop {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedGroundEstop {
    fn name(&self) -> &'static str {
        "typed_ground_estop"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(rover) = robot(obs, "rover") else {
            return Vec::new();
        };
        let Some(g) = rover.ground.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if g.kind == GroundKind::Parked {
                return vec![cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0)];
            }
        }
        if g.kind == GroundKind::EStopped {
            if lab.attach_reset("rover").is_ok() {
                note(lab, cmd("rover", LabCmd::Clear, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(g.kind, GroundKind::Parked | GroundKind::Moving)
            && lab.attach_estop("rover").is_ok()
        {
            note(lab, cmd("rover", LabCmd::Estop, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Inland or coastal rover: probe parked drive, then attach drive and halt
/// back to Parked without E-stop. Legal trips never go through [`Lab::act`].
/// Distinct from [`TypedGroundEstop`], which trips from Parked with no drive
/// grant.
#[derive(Default)]
pub struct TypedGroundHalt {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedGroundHalt {
    fn name(&self) -> &'static str {
        "typed_ground_halt"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(rover) = robot(obs, "rover") else {
            return Vec::new();
        };
        let Some(g) = rover.ground.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if g.kind == GroundKind::Parked {
                return vec![cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0)];
            }
        }
        if g.kind == GroundKind::Moving {
            if lab.attach_park("rover").is_ok() {
                note(lab, cmd("rover", LabCmd::Halt, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if g.kind == GroundKind::Parked && lab.attach_drive("rover").is_ok() {
            note(lab, cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Inland, coastal, or harbor rover: probe parked drive, then attach drive
/// and hold the current NED pose. Open water has no rover (P11). Legal
/// motion never goes through [`Lab::act`]. Distinct from [`TypedGroundHalt`],
/// which parks instead of holding.
#[derive(Default)]
pub struct TypedGroundHold {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedGroundHold {
    fn name(&self) -> &'static str {
        "typed_ground_hold"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(rover) = robot(obs, "rover") else {
            return Vec::new();
        };
        let Some(g) = rover.ground.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if g.kind == GroundKind::Parked {
                return vec![cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0)];
            }
        }
        if g.kind == GroundKind::Moving {
            if rover.hold_ned.is_some() || rover_hold_attached(lab) {
                self.done = true;
            }
            return Vec::new();
        }
        if g.kind == GroundKind::Parked && lab.attach_drive("rover").is_ok() {
            note(lab, cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}
