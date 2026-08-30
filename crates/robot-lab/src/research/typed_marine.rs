use crate::{AgentAction, Lab, LabCmd, MarineKind, Observation};

use super::support::{
    cmd, hull_hold_attached, note, robot, skiff_thrust_attached, surveyor_thrust_attached,
};
use super::ResearchAgent;

/// Coastal skiff: probe docked thrust, then attach undock, station-keep, and
/// dock. Legal motion never goes through [`Lab::act`].
#[derive(Default)]
pub struct TypedStationDock {
    pub(crate) probed: bool,
    pub(crate) way: u32,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedStationDock {
    fn name(&self) -> &'static str {
        "typed_station_dock"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(skiff) = robot(obs, "skiff") else {
            return Vec::new();
        };
        let Some(m) = skiff.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
            }
        }
        if m.kind == MarineKind::StationKeep {
            if lab.attach_dock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Dock, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway && skiff.support == "water" {
            self.way += 1;
            if self.way >= 60 {
                if lab.attach_station("skiff").is_ok() {
                    note(lab, cmd("skiff", LabCmd::Station, 0.0, 0.0, 0.0));
                }
                return Vec::new();
            }
            skiff_thrust_attached(lab, 0.05, 0.55, 0.0);
        }
        Vec::new()
    }
}

/// Coastal skiff: probe docked thrust, then attach undock, make way, and
/// `dock_now` from Underway (`CanDock` includes Underway). Legal motion never
/// goes through [`Lab::act`]. Distinct from [`TypedStationDock`], which
/// stations first, and from [`TypedStationResume`], which resumes instead.
#[derive(Default)]
pub struct TypedHullDock {
    pub(crate) probed: bool,
    pub(crate) way: u32,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedHullDock {
    fn name(&self) -> &'static str {
        "typed_hull_dock"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(skiff) = robot(obs, "skiff") else {
            return Vec::new();
        };
        let Some(m) = skiff.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
            }
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway && skiff.support == "water" {
            self.way += 1;
            if self.way >= 60 {
                if lab.attach_dock("skiff").is_ok() {
                    note(lab, cmd("skiff", LabCmd::Dock, 0.0, 0.0, 0.0));
                    self.done = true;
                }
                return Vec::new();
            }
            skiff_thrust_attached(lab, 0.05, 0.55, 0.0);
        }
        Vec::new()
    }
}

/// Coastal skiff: probe docked thrust, then attach undock, hold station,
/// and `resume` back to Underway. Legal motion never goes through [`Lab::act`].
/// Distinct from [`TypedStationDock`], which docks from StationKeep without
/// resuming, and from [`TypedStationFailsafe`], which trips from station.
#[derive(Default)]
pub struct TypedStationResume {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedStationResume {
    fn name(&self) -> &'static str {
        "typed_station_resume"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(skiff) = robot(obs, "skiff") else {
            return Vec::new();
        };
        let Some(m) = skiff.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
            }
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway {
            if lab.attach_station("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Station, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::StationKeep && lab.attach_resume("skiff").is_ok() {
            note(lab, cmd("skiff", LabCmd::Resume, 0.0, 0.0, 0.0));
            self.done = true;
        }
        Vec::new()
    }
}

/// Coastal skiff: probe docked thrust, then attach undock, trip marine
/// failsafe, and recover docked. Legal trips never go through [`Lab::act`].
#[derive(Default)]
pub struct TypedHullFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedHullFailsafe {
    fn name(&self) -> &'static str {
        "typed_hull_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(skiff) = robot(obs, "skiff") else {
            return Vec::new();
        };
        let Some(m) = skiff.marine.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if m.kind == MarineKind::Docked {
                return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
            }
        }
        if m.kind == MarineKind::Failsafe {
            if lab.attach_recover("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if matches!(m.kind, MarineKind::Underway | MarineKind::StationKeep)
            && lab.attach_marine_failsafe("skiff").is_ok()
        {
            note(lab, cmd("skiff", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Coastal skiff: probe docked thrust, then attach undock, station-keep, and
/// trip marine failsafe. Legal trips never go through [`Lab::act`].
#[derive(Default)]
pub struct TypedStationFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedStationFailsafe {
    fn name(&self) -> &'static str {
        "typed_station_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(skiff) = robot(obs, "skiff") else {
            return Vec::new();
        };
        let Some(m) = skiff.marine.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if m.kind == MarineKind::Docked {
                return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
            }
        }
        if m.kind == MarineKind::Failsafe {
            if lab.attach_recover("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway {
            if lab.attach_station("skiff").is_ok() {
                note(lab, cmd("skiff", LabCmd::Station, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::StationKeep && lab.attach_marine_failsafe("skiff").is_ok() {
            note(lab, cmd("skiff", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Coastal AUV: probe docked thrust, then attach undock and trip marine
/// failsafe. Legal trips never go through [`Lab::act`].
#[derive(Default)]
pub struct TypedSurveyorFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedSurveyorFailsafe {
    fn name(&self) -> &'static str {
        "typed_surveyor_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(surveyor) = robot(obs, "surveyor") else {
            return Vec::new();
        };
        let Some(m) = surveyor.marine.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if m.kind == MarineKind::Docked {
                return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
            }
        }
        if m.kind == MarineKind::Failsafe {
            if lab.attach_recover("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if matches!(m.kind, MarineKind::Underway | MarineKind::StationKeep)
            && lab.attach_marine_failsafe("surveyor").is_ok()
        {
            note(lab, cmd("surveyor", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Coastal AUV: probe docked thrust, then attach undock, hold station,
/// trip marine failsafe from StationKeep, and recover docked. Legal trips
/// never go through [`Lab::act`]. Distinct from [`TypedSurveyorFailsafe`],
/// which trips from Underway without station-keeping, and from
/// [`TypedStationFailsafe`], which trips the skiff.
#[derive(Default)]
pub struct TypedSurveyorStationFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedSurveyorStationFailsafe {
    fn name(&self) -> &'static str {
        "typed_surveyor_station_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(surveyor) = robot(obs, "surveyor") else {
            return Vec::new();
        };
        let Some(m) = surveyor.marine.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if m.kind == MarineKind::Docked {
                return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
            }
        }
        if m.kind == MarineKind::Failsafe {
            if lab.attach_recover("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway {
            if lab.attach_station("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Station, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::StationKeep && lab.attach_marine_failsafe("surveyor").is_ok() {
            note(lab, cmd("surveyor", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Coastal AUV: probe docked thrust, then attach undock, make way, hold
/// station, and dock. Legal motion never goes through [`Lab::act`]. Distinct
/// from [`TypedSurveyorStationFailsafe`], which trips from StationKeep,
/// from [`TypedSurveyorDock`], which docks from Underway without station,
/// and from [`TypedStationDock`], which docks the skiff.
#[derive(Default)]
pub struct TypedSurveyorStationDock {
    pub(crate) probed: bool,
    pub(crate) way: u32,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedSurveyorStationDock {
    fn name(&self) -> &'static str {
        "typed_surveyor_station_dock"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(surveyor) = robot(obs, "surveyor") else {
            return Vec::new();
        };
        let Some(m) = surveyor.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
            }
        }
        if m.kind == MarineKind::StationKeep {
            if lab.attach_dock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Dock, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway && surveyor.support == "water" {
            self.way += 1;
            if self.way >= 60 {
                if lab.attach_station("surveyor").is_ok() {
                    note(lab, cmd("surveyor", LabCmd::Station, 0.0, 0.0, 0.0));
                }
                return Vec::new();
            }
            surveyor_thrust_attached(lab, 0.25, 0.0, 0.0);
        }
        Vec::new()
    }
}

/// Coastal AUV: probe docked thrust, then attach undock, make way, and
/// `dock_now` from Underway (`CanDock` includes Underway). Legal motion never
/// goes through [`Lab::act`]. Distinct from [`TypedSurveyorStationDock`], which
/// stations first, and from [`TypedHullDock`], which docks the skiff.
#[derive(Default)]
pub struct TypedSurveyorDock {
    pub(crate) probed: bool,
    pub(crate) way: u32,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedSurveyorDock {
    fn name(&self) -> &'static str {
        "typed_surveyor_dock"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(surveyor) = robot(obs, "surveyor") else {
            return Vec::new();
        };
        let Some(m) = surveyor.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
            }
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway && surveyor.support == "water" {
            self.way += 1;
            if self.way >= 60 {
                if lab.attach_dock("surveyor").is_ok() {
                    note(lab, cmd("surveyor", LabCmd::Dock, 0.0, 0.0, 0.0));
                    self.done = true;
                }
                return Vec::new();
            }
            surveyor_thrust_attached(lab, 0.25, 0.0, 0.0);
        }
        Vec::new()
    }
}

/// Coastal AUV: probe docked thrust, then attach undock, hold station,
/// and `resume` back to Underway. Legal motion never goes through [`Lab::act`].
/// Distinct from [`TypedSurveyorStationDock`], which docks from StationKeep
/// without resuming, from [`TypedSurveyorStationFailsafe`], which trips from
/// station, and from [`TypedStationResume`], which resumes the skiff.
#[derive(Default)]
pub struct TypedSurveyorStationResume {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedSurveyorStationResume {
    fn name(&self) -> &'static str {
        "typed_surveyor_station_resume"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(surveyor) = robot(obs, "surveyor") else {
            return Vec::new();
        };
        let Some(m) = surveyor.marine.as_ref() else {
            return Vec::new();
        };
        if m.failsafe || self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !m.thrust_enabled {
                return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
            }
        }
        if m.kind == MarineKind::Docked {
            if lab.attach_undock("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::Underway {
            if lab.attach_station("surveyor").is_ok() {
                note(lab, cmd("surveyor", LabCmd::Station, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if m.kind == MarineKind::StationKeep && lab.attach_resume("surveyor").is_ok() {
            note(lab, cmd("surveyor", LabCmd::Resume, 0.0, 0.0, 0.0));
            self.done = true;
        }
        Vec::new()
    }
}

/// Coastal, harbor, or open_water hulls: probe docked thrust, undock, then
/// hold the current NED pose. Distinct from [`TypedStationDock`] (StationKeep
/// machine). Inland has no hulls (P11). Legal motion never goes through
/// [`Lab::act`].
#[derive(Default)]
pub struct TypedMarineHold {
    pub(crate) probed: bool,
    pub(crate) skiff_done: bool,
    pub(crate) surveyor_done: bool,
}

impl TypedMarineHold {
    pub fn done(&self) -> bool {
        self.skiff_done && self.surveyor_done
    }
}

impl ResearchAgent for TypedMarineHold {
    fn name(&self) -> &'static str {
        "typed_marine_hold"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if !self.probed {
            self.probed = true;
            if let Some(skiff) = robot(obs, "skiff") {
                if skiff
                    .marine
                    .as_ref()
                    .is_some_and(|m| m.kind == MarineKind::Docked)
                {
                    return vec![cmd("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0)];
                }
            }
            if let Some(surveyor) = robot(obs, "surveyor") {
                if surveyor
                    .marine
                    .as_ref()
                    .is_some_and(|m| m.kind == MarineKind::Docked)
                {
                    return vec![cmd("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4)];
                }
            }
        }
        grant_hull_hold(lab, obs, "skiff", &mut self.skiff_done);
        grant_hull_hold(lab, obs, "surveyor", &mut self.surveyor_done);
        Vec::new()
    }
}

fn grant_hull_hold(lab: &mut Lab, obs: &Observation, id: &'static str, done: &mut bool) {
    if *done {
        return;
    }
    let Some(hull) = robot(obs, id) else {
        return;
    };
    let Some(m) = hull.marine.as_ref() else {
        return;
    };
    if m.failsafe {
        return;
    }
    if m.kind == MarineKind::Docked {
        if lab.attach_undock(id).is_ok() {
            note(lab, cmd(id, LabCmd::Undock, 0.0, 0.0, 0.0));
        }
        return;
    }
    if matches!(m.kind, MarineKind::Underway | MarineKind::StationKeep)
        && (hull.hold_ned.is_some() || hull_hold_attached(lab, id))
    {
        *done = true;
    }
}
