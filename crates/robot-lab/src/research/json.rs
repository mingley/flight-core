use crate::{AerialKind, AgentAction, Lab, LabCmd, Observation};

use super::support::{cmd, drone_grant_chain, grants_for, motions_for, probes, robot};
use super::ResearchAgent;

/// Inland rover: try parked drive (must bounce), then release and drive south.
#[derive(Default)]
pub struct RoverProbe {
    pub(crate) tried_parked_drive: bool,
}

impl ResearchAgent for RoverProbe {
    fn name(&self) -> &'static str {
        "rover_probe"
    }

    fn act(&mut self, _lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let rover = match robot(obs, "rover").and_then(|r| r.ground.as_ref()) {
            Some(g) => g,
            None => return Vec::new(),
        };
        if !rover.drive_enabled && !rover.estop {
            if !self.tried_parked_drive {
                self.tried_parked_drive = true;
                return vec![cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0)];
            }
            return vec![cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0)];
        }
        if rover.drive_enabled && robot(obs, "rover").is_some_and(|r| r.terrain_contact) {
            return vec![cmd("rover", LabCmd::Drive, -0.6, 0.0, 0.0)];
        }
        Vec::new()
    }
}

/// Demo / catalog policy as a research certificate: [`Lab::apply_script`]
/// (attach helpers + NED now-APIs) then the verified step. Returns no JSON, so
/// `actions_applied` stays 0. Failsafe and e-stop stay idle.
pub struct ScriptedCoastal;

impl ResearchAgent for ScriptedCoastal {
    fn name(&self) -> &'static str {
        "scripted_coastal"
    }

    fn act(&mut self, lab: &mut Lab, _obs: &Observation) -> Vec<AgentAction> {
        lab.apply_script();
        Vec::new()
    }
}

/// Drone on a pad: climb until `terrain_contact` clears, then land until it
/// returns and `touchdown` the aerial machine back to Ready.
#[derive(Default)]
pub struct PadLanding {
    pub(crate) saw_pad: bool,
    pub(crate) left_pad: bool,
}

impl ResearchAgent for PadLanding {
    fn name(&self) -> &'static str {
        "pad_landing"
    }

    fn act(&mut self, _lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        let Some(a) = drone.aerial.as_ref() else {
            return Vec::new();
        };
        if a.failsafe {
            return Vec::new();
        }

        if drone.terrain_contact {
            self.saw_pad = true;
            if self.left_pad {
                if a.kind == AerialKind::Landing {
                    return vec![cmd("drone", LabCmd::Touchdown, 0.0, 0.0, 0.0)];
                }
                return Vec::new();
            }
            let grants = drone_grant_chain(obs);
            if !grants.is_empty() {
                return grants;
            }
            if a.armed && a.actuators_enabled {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
            }
            return Vec::new();
        }

        if !self.saw_pad {
            return Vec::new();
        }
        self.left_pad = true;
        if a.kind == AerialKind::Landing {
            if a.armed && a.actuators_enabled {
                return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, 0.8)];
            }
            return Vec::new();
        }
        if drone.alt < 2.5 {
            return vec![cmd("drone", LabCmd::Velocity, 0.0, 0.0, -1.2)];
        }
        let mut out = Vec::new();
        if a.kind == AerialKind::Takeoff {
            out.push(cmd("drone", LabCmd::Airborne, 0.0, 0.0, 0.0));
        }
        if a.kind == AerialKind::Airborne {
            out.push(cmd("drone", LabCmd::Land, 0.0, 0.0, 0.0));
        }
        if a.armed && a.actuators_enabled {
            out.push(cmd("drone", LabCmd::Velocity, 0.0, 0.0, 0.8));
        }
        out
    }
}

/// Inland rover: probe parked drive, then drive at the drone until
/// `sphere_contact`. The verified step must still separate the hulls.
#[derive(Default)]
pub struct CollisionSweep {
    pub(crate) tried_parked_drive: bool,
    pub(crate) hit: bool,
}

impl ResearchAgent for CollisionSweep {
    fn name(&self) -> &'static str {
        "collision_sweep"
    }

    fn act(&mut self, _lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        let Some(rover) = robot(obs, "rover") else {
            return Vec::new();
        };
        let Some(drone) = robot(obs, "drone") else {
            return Vec::new();
        };
        if rover.sphere_contact
            || drone.sphere_contact
            || obs
                .sphere_hits
                .iter()
                .any(|h| h.involves("rover") && h.involves("drone"))
        {
            self.hit = true;
            if rover.ground.as_ref().is_some_and(|g| g.drive_enabled) {
                return vec![cmd("rover", LabCmd::Halt, 0.0, 0.0, 0.0)];
            }
            return Vec::new();
        }
        if self.hit {
            return Vec::new();
        }
        let Some(g) = rover.ground.as_ref() else {
            return Vec::new();
        };
        if g.estop {
            return Vec::new();
        }
        if !g.drive_enabled {
            if !self.tried_parked_drive {
                self.tried_parked_drive = true;
                return vec![cmd("rover", LabCmd::Drive, 0.0, -1.2, 0.0)];
            }
            return vec![cmd("rover", LabCmd::Release, 0.0, 0.0, 0.0)];
        }
        if !rover.terrain_contact {
            return Vec::new();
        }
        let vn = (drone.n - rover.n).signum() * 1.4;
        let ve = (drone.e - rover.e).signum() * 1.4;
        vec![cmd("rover", LabCmd::Drive, vn, ve, 0.0)]
    }
}

/// Mixed fleet: probe illegal grants from typed machines, then arm / release / undock / move.
///
/// Missing hulls are skipped (`inland` has no skiff; `open_water` has no rover).
/// One tick probes every present domain, the next grants them together, then
/// every hull is commanded before the verified step.
#[derive(Default)]
pub struct CoastalFleet {
    pub(crate) probed: bool,
}

impl ResearchAgent for CoastalFleet {
    fn name(&self) -> &'static str {
        "coastal_fleet"
    }

    fn act(&mut self, _lab: &mut Lab, obs: &Observation) -> Vec<AgentAction> {
        if !self.probed {
            self.probed = true;
            return probes(obs);
        }
        let grants = grants_for(obs);
        if !grants.is_empty() {
            return grants;
        }
        motions_for(obs)
    }
}
