use crate::{AerialKind, AgentAction, Lab, LabCmd, Observation};

use super::support::{
    cmd, drone_hold_attached, drone_position_attached, drone_velocity_attached,
    grant_drone_attached, note, robot,
};
use super::ResearchAgent;

/// Pad landing through consume-self typestate. First tick probes a disarmed
/// velocity (JSON, must bounce). Legal climb / airborne / land / touchdown never
/// go through [`Lab::act`]; [`Lab::attach_airborne`] / [`Lab::attach_land`] /
/// [`Lab::attach_touchdown`] walk the machines. Matching intents are appended
/// to [`Lab::log`] so [`Lab::replay_until`] can reproduce the run.
#[derive(Default)]
pub struct TypedPadLanding {
    pub(crate) probed: bool,
    pub(crate) saw_pad: bool,
    pub(crate) left_pad: bool,
}

impl ResearchAgent for TypedPadLanding {
    fn name(&self) -> &'static str {
        "typed_pad_landing"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if a.failsafe {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if !a.armed {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 1.0, 0.0)];
            }
        }

        if drone.terrain_contact {
            self.saw_pad = true;
            if self.left_pad {
                if a.kind == AerialKind::Landing && lab.attach_touchdown("drone").is_ok() {
                    note(lab, cmd("drone", LabCmd::Touchdown, 0.0, 0.0, 0.0));
                }
                return Vec::new();
            }
            if matches!(
                a.kind,
                AerialKind::PreflightReady
                    | AerialKind::Armed
                    | AerialKind::Offboard
                    | AerialKind::Disarmed
                    | AerialKind::Disconnected
            ) {
                grant_drone_attached(lab);
                return Vec::new();
            }
            if a.armed && a.actuators_enabled {
                drone_velocity_attached(lab, 0.0, 0.0, -1.2);
            }
            return Vec::new();
        }

        if !self.saw_pad {
            return Vec::new();
        }
        self.left_pad = true;
        if a.kind == AerialKind::Landing {
            if a.armed && a.actuators_enabled {
                drone_velocity_attached(lab, 0.0, 0.0, 0.8);
            }
            return Vec::new();
        }
        if drone.alt < 2.5 {
            drone_velocity_attached(lab, 0.0, 0.0, -1.2);
            return Vec::new();
        }
        if a.kind == AerialKind::Takeoff {
            if lab.attach_airborne("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Airborne, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if a.kind == AerialKind::Airborne && lab.attach_land("drone").is_ok() {
            note(lab, cmd("drone", LabCmd::Land, 0.0, 0.0, 0.0));
        }
        if a.armed && a.actuators_enabled {
            drone_velocity_attached(lab, 0.0, 0.0, 0.8);
        }
        Vec::new()
    }
}

/// Inland or coastal drone: probe disarmed velocity, then attach takeoff,
/// trip failsafe, and recover through Recovery to Ready. Legal trips never
/// go through [`Lab::act`].
#[derive(Default)]
pub struct TypedAerialFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedAerialFailsafe {
    fn name(&self) -> &'static str {
        "typed_aerial_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if a.kind == AerialKind::Recovery {
            if lab.attach_recover_ready("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if a.kind == AerialKind::Failsafe {
            if lab.attach_recover_ready("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Disarm, 0.0, 0.0, 0.0));
                note(lab, cmd("drone", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(
            a.kind,
            AerialKind::Takeoff | AerialKind::Airborne | AerialKind::Offboard | AerialKind::Landing
        ) {
            if lab.attach_failsafe("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Failsafe, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed) {
            grant_drone_attached(lab);
        }
        Vec::new()
    }
}

/// Inland or coastal drone: probe disarmed velocity, then attach takeoff
/// and disarm to Ready without failsafe. Legal trips never go through
/// [`Lab::act`]. Distinct from [`TypedAerialFailsafe`], which trips
/// failsafe after takeoff.
#[derive(Default)]
pub struct TypedAerialDisarm {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedAerialDisarm {
    fn name(&self) -> &'static str {
        "typed_aerial_disarm"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if matches!(
            a.kind,
            AerialKind::Takeoff | AerialKind::Airborne | AerialKind::Offboard | AerialKind::Landing
        ) {
            if lab.attach_disarm("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Disarm, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed) {
            grant_drone_attached(lab);
        }
        Vec::new()
    }
}

/// Inland or coastal drone: probe disarmed velocity, then attach takeoff,
/// declare airborne, and begin land. Legal motion never goes through
/// [`Lab::act`]. Distinct from [`TypedPadLanding`], which continues to
/// touchdown, from [`TypedAerialFailsafe`], which trips after takeoff, and
/// from [`TypedAerialDisarm`], which disarms from climb.
#[derive(Default)]
pub struct TypedAerialAirborne {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedAerialAirborne {
    fn name(&self) -> &'static str {
        "typed_aerial_airborne"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if a.kind == AerialKind::Airborne {
            if lab.attach_land("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Land, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if a.kind == AerialKind::Takeoff {
            if lab.attach_airborne("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Airborne, 0.0, 0.0, 0.0));
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed) {
            grant_drone_attached(lab);
        }
        Vec::new()
    }
}

/// Pad drone: probe Ready velocity, then takeoff and hold a NED position
/// through `set_position_now`. Legal motion never goes through [`Lab::act`].
/// Distinct from [`TypedAerialAirborne`], which declares airborne and lands,
/// and from [`TypedPadLanding`], which continues to touchdown.
#[derive(Default)]
pub struct TypedPositionHold {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedPositionHold {
    fn name(&self) -> &'static str {
        "typed_position_hold"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if matches!(
            a.kind,
            AerialKind::Offboard | AerialKind::Takeoff | AerialKind::Airborne | AerialKind::Landing
        ) {
            if drone_position_attached(lab, drone.n, drone.e, -2.0) {
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed) {
            grant_drone_attached(lab);
        }
        Vec::new()
    }
}

/// Pad drone: probe Ready velocity, then takeoff and hold the current NED pose
/// through `Lab::attach_hold`. Legal motion never goes through [`Lab::act`].
/// Distinct from [`TypedPositionHold`], which holds d=−2 via `set_position_now`.
#[derive(Default)]
pub struct TypedHold {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedHold {
    fn name(&self) -> &'static str {
        "typed_hold"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if matches!(
            a.kind,
            AerialKind::Offboard | AerialKind::Takeoff | AerialKind::Airborne | AerialKind::Landing
        ) {
            if drone_hold_attached(lab) {
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed) {
            grant_drone_attached(lab);
        }
        Vec::new()
    }
}

/// Pad drone: probe Ready velocity, then disarm to Ready without takeoff
/// or failsafe. Legal trips never go through [`Lab::act`]. Distinct from
/// [`TypedAerialDisarm`], which grants takeoff first, and from
/// [`TypedPadFailsafe`], which trips failsafe from the pad.
#[derive(Default)]
pub struct TypedPadDisarm {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedPadDisarm {
    fn name(&self) -> &'static str {
        "typed_pad_disarm"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed)
            && lab.attach_disarm("drone").is_ok()
        {
            note(lab, cmd("drone", LabCmd::Disarm, 0.0, 0.0, 0.0));
            self.done = true;
        }
        Vec::new()
    }
}

/// Pad drone: probe Ready velocity, then trip failsafe from Ready (or Armed)
/// without takeoff, and recover through Recovery to Ready. Legal trips never
/// go through [`Lab::act`]. Distinct from [`TypedAerialFailsafe`], which grants
/// takeoff first.
#[derive(Default)]
pub struct TypedPadFailsafe {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedPadFailsafe {
    fn name(&self) -> &'static str {
        "typed_pad_failsafe"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if a.kind == AerialKind::Recovery {
            if lab.attach_recover_ready("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if a.kind == AerialKind::Failsafe {
            if lab.attach_recover_ready("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Disarm, 0.0, 0.0, 0.0));
                note(lab, cmd("drone", LabCmd::Recover, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed)
            && lab.attach_failsafe("drone").is_ok()
        {
            note(lab, cmd("drone", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}

/// Pad drone: probe Ready velocity, trip failsafe, then `attach_touchdown`
/// back to Ready. Legal trips never go through [`Lab::act`]. Distinct from
/// [`TypedPadFailsafe`], which recovers through Recovery, and from
/// [`TypedAerialFailsafe`], which grants takeoff first.
#[derive(Default)]
pub struct TypedFailsafeTouchdown {
    pub(crate) probed: bool,
    pub(crate) done: bool,
}

impl ResearchAgent for TypedFailsafeTouchdown {
    fn name(&self) -> &'static str {
        "typed_failsafe_touchdown"
    }

    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if self.done {
            return Vec::new();
        }
        if !self.probed {
            self.probed = true;
            if a.kind == AerialKind::PreflightReady {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
        }
        if a.kind == AerialKind::Failsafe {
            if lab.attach_touchdown("drone").is_ok() {
                note(lab, cmd("drone", LabCmd::Touchdown, 0.0, 0.0, 0.0));
                self.done = true;
            }
            return Vec::new();
        }
        if matches!(a.kind, AerialKind::PreflightReady | AerialKind::Armed)
            && lab.attach_failsafe("drone").is_ok()
        {
            note(lab, cmd("drone", LabCmd::Failsafe, 0.0, 0.0, 0.0));
        }
        Vec::new()
    }
}
