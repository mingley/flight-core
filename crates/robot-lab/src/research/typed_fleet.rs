use crate::{AgentAction, Lab, LabCmd, Observation};

use super::support::{
    cmd, drive_attached, drone_hold_attached, grant_attached, note, probes, return_attached, robot,
};
use super::ResearchAgent;

/// Mixed fleet: JSON-probe illegal grants, then attach consume-self typestate
/// (`Lab::attach_takeoff` / `attach_drive` / `attach_undock`) and NED now-APIs
/// on the live plant.
///
/// Successful motion never goes through [`Lab::act`] (so `actions_applied`
/// stays 0). Matching intents are appended to [`Lab::log`] so
/// [`Lab::replay_until`] can reproduce the run. Missing hulls are skipped.
#[derive(Default)]
pub struct TypedFleet {
    pub(crate) probed: bool,
    pub(crate) granted: bool,
}

impl ResearchAgent for TypedFleet {
    fn name(&self) -> &'static str {
        "typed_fleet"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if !self.probed {
            self.probed = true;
            return probes(obs);
        }
        if !self.granted {
            self.granted = true;
            grant_attached(lab, obs);
            return Vec::new();
        }
        drive_attached(lab, obs);
        Vec::new()
    }
}

/// Mixed fleet: JSON-probe illegal grants, then attach consume-self typestate
/// (`Lab::attach_takeoff` / `attach_drive` / `attach_undock`) and NED now-APIs
/// on the live plant. Legal motion never goes through [`Lab::act`].
#[derive(Default)]
pub struct TypedAttachFleet {
    pub(crate) probed: bool,
    pub(crate) granted: bool,
}

impl ResearchAgent for TypedAttachFleet {
    fn name(&self) -> &'static str {
        "typed_attach_fleet"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if !self.probed {
            self.probed = true;
            return probes(obs);
        }
        if !self.granted {
            self.granted = true;
            grant_attached(lab, obs);
            return Vec::new();
        }
        drive_attached(lab, obs);
        Vec::new()
    }
}

#[derive(Default)]
pub struct TypedFleetReturn {
    pub(crate) probed: bool,
    pub(crate) granted: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedFleetReturn {
    fn name(&self) -> &'static str {
        "typed_fleet_return"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            return probes(obs);
        }
        if !self.granted {
            self.granted = true;
            grant_attached(lab, obs);
            return Vec::new();
        }
        return_attached(lab, obs);
        self.done = true;
        Vec::new()
    }
}

/// Mixed fleet: probe illegal grants, then `grant_attached`, drone
/// `attach_hold`, and skiff `attach_station`. Inland has no hull to station.
/// Open water has no rover to grant. Legal motion never goes through [`Lab::act`].
/// Distinct from [`TypedHold`] (drone only) and [`TypedStationResume`] (skiff only).
#[derive(Default)]
pub struct TypedFleetHold {
    pub(crate) probed: bool,
    pub(crate) granted: bool,
    pub(crate) held: bool,
    pub(crate) stationed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedFleetHold {
    fn name(&self) -> &'static str {
        "typed_fleet_hold"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            return probes(obs);
        }
        if !self.granted {
            self.granted = true;
            grant_attached(lab, obs);
            return Vec::new();
        }
        if robot(obs, "drone").is_some() && !self.held {
            self.held = drone_hold_attached(lab);
        } else if robot(obs, "drone").is_none() {
            self.held = true;
        }
        if robot(obs, "skiff").is_none() {
            self.stationed = true;
        } else if !self.stationed && lab.attach_station("skiff").is_ok() {
            note(lab, cmd("skiff", LabCmd::Station, 0.0, 0.0, 0.0));
            self.stationed = true;
        }
        self.done = self.held && self.stationed;
        Vec::new()
    }
}
