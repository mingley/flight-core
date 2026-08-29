//! Observe → act → step loops that return a property certificate.

mod json;
mod support;
mod typed_aerial;
mod typed_fleet;
mod typed_ground;
mod typed_marine;

#[cfg(test)]
mod tests;

pub use json::{CoastalFleet, CollisionSweep, PadLanding, RoverProbe, ScriptedCoastal};
pub use typed_aerial::{
    TypedAerialAirborne, TypedAerialDisarm, TypedAerialFailsafe, TypedFailsafeTouchdown, TypedHold,
    TypedPadDisarm, TypedPadFailsafe, TypedPadLanding, TypedPositionHold,
};
pub use typed_fleet::{TypedAttachFleet, TypedFleet, TypedFleetHold, TypedFleetReturn};
pub use typed_ground::{TypedCollisionSweep, TypedGroundEstop, TypedGroundHalt};
pub use typed_marine::{
    TypedHullDock, TypedHullFailsafe, TypedStationDock, TypedStationFailsafe, TypedStationResume,
    TypedSurveyorDock, TypedSurveyorFailsafe, TypedSurveyorStationDock,
    TypedSurveyorStationFailsafe, TypedSurveyorStationResume,
};

use crate::{AgentAction, Lab, Observation, RejectTrace, TimedAction};
use robot_world::{Property, SphereHit};
use serde::Serialize;

/// A closed-loop policy over [`Observation`]. Illegal JSON `act` results are
/// counted, not applied. Agents may also grant and command through
/// [`Lab::aerial`] / [`Lab::ground`] / [`Lab::marine`] or attach consume-self
/// typestate via [`Lab::aerial_vehicle`] / [`Lab::ground_vehicle`] /
/// [`Lab::marine_vehicle`] on the same `WorldSession`. The lab still refuses
/// mechanical successors that would break the property vector.
///
/// One tick may return several [`LabCmd`] actions. `Lab::research` applies them
/// through [`Lab::act_through_attach`] in order, then takes one verified step.
pub trait ResearchAgent {
    fn name(&self) -> &'static str;
    fn act(&mut self, lab: &mut Lab, obs: &Observation) -> Vec<AgentAction>;
}

/// Outcome of [`Lab::research`]: what the agent did, and whether properties held.
#[derive(Clone, Debug, Serialize)]
pub struct ResearchRun {
    pub agent: &'static str,
    pub scenario: &'static str,
    pub seed: u64,
    pub steps: u32,
    pub t: f32,
    pub actions_applied: usize,
    pub actions_rejected: usize,
    pub all_hold: bool,
    pub broken: Vec<String>,
    /// Mechanical vector at the end of the run (the research certificate).
    pub properties: Vec<Property>,
    /// Pairwise sphere contacts on the last committed step.
    pub sphere_hits: Vec<SphereHit>,
    /// Structured bounce for each `act_through_attach` that rejected (NEXT A4).
    pub rejects: Vec<RejectTrace>,
}

impl ResearchRun {
    pub fn ok(&self) -> bool {
        self.all_hold
    }

    pub fn holds(&self, id: &str) -> bool {
        self.properties.iter().any(|p| p.id == id && p.holds)
    }

    pub fn hit_between(&self, a: &str, b: &str) -> bool {
        self.sphere_hits
            .iter()
            .any(|h| h.involves(a) && h.involves(b))
    }
}

impl std::fmt::Display for ResearchRun {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} {} seed={} steps={} applied={} rejected={} hold={} props={} hits={}",
            self.agent,
            self.scenario,
            self.seed,
            self.steps,
            self.actions_applied,
            self.actions_rejected,
            self.all_hold,
            self.properties.len(),
            self.sphere_hits.len()
        )
    }
}

impl Lab {
    /// Run `agent` for `steps` ticks. Each tick: observe, apply the agent's
    /// command vector through [`Self::act_through_attach`], then **one**
    /// verified step (P12). Illegal JSON probes still bounce. Typed agents
    /// that grant on handles and return an empty vector keep
    /// `actions_applied == 0`. Stops early if a property fails (the violating
    /// successor was not committed).
    pub fn research(&mut self, agent: &mut impl ResearchAgent, dt: f32, steps: u32) -> ResearchRun {
        self.research_with(agent, dt, steps, |_| {}, |_| {})
    }

    /// Same loop as [`Self::research`], with per-tick observation and newly
    /// logged action hooks for the experiment runner (NEXT A3).
    pub fn research_with(
        &mut self,
        agent: &mut (impl ResearchAgent + ?Sized),
        dt: f32,
        steps: u32,
        mut on_obs: impl FnMut(&Observation),
        mut on_act: impl FnMut(&TimedAction),
    ) -> ResearchRun {
        let mut actions_applied = 0usize;
        let mut actions_rejected = 0usize;
        let mut rejects = Vec::new();
        let mut ran = 0u32;
        for _ in 0..steps {
            let obs = self.observe();
            on_obs(&obs);
            let log_at = self.log.len();
            let actions = agent.act(self, &obs);
            for a in actions {
                match self.act_through_attach(a) {
                    Ok(()) => actions_applied += 1,
                    Err(_) => {
                        actions_rejected += 1;
                        if let Some(t) = self.last_reject() {
                            rejects.push(t.clone());
                        }
                    }
                }
            }
            for timed in &self.log[log_at..] {
                on_act(timed);
            }
            self.step(dt);
            ran += 1;
            if !self.all_hold() {
                break;
            }
        }
        on_obs(&self.observe());
        let world = self.world();
        let broken: Vec<String> = world
            .last_properties
            .iter()
            .filter(|p| !p.holds)
            .map(|p| p.id.to_string())
            .collect();
        ResearchRun {
            agent: agent.name(),
            scenario: world.scenario,
            seed: world.seed,
            steps: ran,
            t: world.t,
            actions_applied,
            actions_rejected,
            all_hold: world.all_hold(),
            broken,
            properties: world.last_properties.clone(),
            sphere_hits: world.last_sphere_hits.clone(),
            rejects,
        }
    }
}
