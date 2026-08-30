//! Adversarial observe/act catalog: illegal commands must bounce, properties must hold.

use crate::{AgentAction, Lab, LabCmd, RejectTrace};
use serde::Serialize;

/// Outcome of [`Lab::research_probe`].
#[derive(Clone, Debug, Serialize)]
pub struct ProbeReport {
    pub scenario: &'static str,
    pub seed: u64,
    pub illegal_rejected: usize,
    pub illegal_leaked: Vec<String>,
    pub legal_applied: usize,
    pub steps: u32,
    pub all_hold: bool,
    pub broken: Vec<String>,
    /// Structured bounce for each illegal probe that rejected (NEXT A4).
    pub illegal_traces: Vec<RejectTrace>,
}

impl ProbeReport {
    pub fn ok(&self) -> bool {
        self.illegal_leaked.is_empty() && self.all_hold && self.illegal_rejected > 0
    }
}

impl std::fmt::Display for ProbeReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} seed={} illegal_rejected={} leaked={} legal={} hold={}",
            self.scenario,
            self.seed,
            self.illegal_rejected,
            self.illegal_leaked.len(),
            self.legal_applied,
            self.all_hold
        )
    }
}

fn action(robot: &str, cmd: LabCmd, vn: f32, ve: f32, vd: f32) -> AgentAction {
    AgentAction::new(robot, cmd).ned(vn, ve, vd)
}

impl Lab {
    /// Attack the lab the way a researcher or agent should: try the illegal
    /// commands first through [`Self::act`] (they must bounce on the JSON
    /// machines — [`Self::act_through_attach`] would grant Ready Takeoff), then
    /// a legal adversarial sequence through [`Self::act_through_attach`], then
    /// step. Properties after that step are the result.
    pub fn research_probe(&mut self, dt: f32, steps: u32) -> ProbeReport {
        let mut illegal_rejected = 0usize;
        let mut illegal_leaked = Vec::new();
        let mut illegal_traces = Vec::new();
        for a in illegal_catalog() {
            let label = format!("{} {}", a.robot, a.cmd);
            match self.act(a) {
                Err(_) => {
                    illegal_rejected += 1;
                    if let Some(t) = self.last_reject() {
                        illegal_traces.push(t.clone());
                    }
                }
                Ok(()) => illegal_leaked.push(label),
            }
        }
        probe_failsafe_hold(
            self,
            &mut illegal_rejected,
            &mut illegal_leaked,
            &mut illegal_traces,
        );

        let mut legal_applied = 0usize;
        for a in legal_abuse() {
            if self.act_through_attach(a).is_ok() {
                legal_applied += 1;
            }
        }

        for _ in 0..steps {
            self.step(dt);
        }

        let world = self.world();
        let broken: Vec<String> = world
            .last_properties
            .iter()
            .filter(|p| !p.holds)
            .map(|p| p.id.to_string())
            .collect();

        ProbeReport {
            scenario: world.scenario,
            seed: world.seed,
            illegal_rejected,
            illegal_leaked,
            legal_applied,
            steps,
            all_hold: world.all_hold(),
            broken,
            illegal_traces,
        }
    }
}

/// Failsafe Hold is illegal on the JSON machine. Probe it on a clone so the
/// main lab stays Ready for the legal-abuse sequence. Illegal phase stays
/// [`Lab::act`] — [`Lab::act_through_attach`] would grant Ready Takeoff.
fn probe_failsafe_hold(
    lab: &Lab,
    rejected: &mut usize,
    leaked: &mut Vec<String>,
    traces: &mut Vec<RejectTrace>,
) {
    let mut scratch = lab.clone();
    if scratch
        .act(action("drone", LabCmd::Failsafe, 0.0, 0.0, 0.0))
        .is_err()
    {
        return;
    }
    match scratch.act(action("drone", LabCmd::Hold, 0.0, 0.0, 0.0)) {
        Err(_) => {
            *rejected += 1;
            if let Some(t) = scratch.last_reject() {
                traces.push(t.clone());
            }
        }
        Ok(()) => leaked.push("drone hold (failsafe)".into()),
    }
}

fn illegal_catalog() -> Vec<AgentAction> {
    vec![
        action("rover", LabCmd::Drive, -1.0, 0.0, 0.0),
        action("skiff", LabCmd::Thrust, 0.8, 0.0, 0.0),
        action("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.4),
        action("drone", LabCmd::Velocity, 0.0, 1.0, 0.0),
        action("drone", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0),
        action("rover", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("skiff", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("surveyor", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Position, 0.0, 0.0, -2.0),
        action("drone", LabCmd::Airborne, 0.0, 0.0, 0.0),
        action("skiff", LabCmd::Station, 0.0, 0.0, 0.0),
        action("rover", LabCmd::Halt, 0.0, 0.0, 0.0),
    ]
}

fn legal_abuse() -> Vec<AgentAction> {
    vec![
        action("", LabCmd::SetWind, 0.0, 12.0, 0.0),
        action("", LabCmd::SetWaves, 1.8, 0.6, 1.4),
        action("", LabCmd::SetCurrent, 0.8, 0.2, 0.0),
        action("rover", LabCmd::Release, 0.0, 0.0, 0.0),
        action("rover", LabCmd::Drive, -2.5, 0.4, 0.0),
        action("rover", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("skiff", LabCmd::Undock, 0.0, 0.0, 0.0),
        action("skiff", LabCmd::Thrust, 1.2, 0.0, 0.0),
        action("skiff", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("surveyor", LabCmd::Undock, 0.0, 0.0, 0.0),
        action("surveyor", LabCmd::Thrust, 0.0, 0.0, 0.6),
        action("surveyor", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Arm, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Offboard, 0.0, 0.0, 0.0),
        action("drone", LabCmd::EnableActuators, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Takeoff, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Position, 10.0, 0.0, -2.0),
        action("drone", LabCmd::Position, 10.0, 2.0, -2.0),
        action("drone", LabCmd::Hold, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Velocity, 0.0, 0.0, -2.0),
        action("drone", LabCmd::SetCharge, 0.0, 0.0, 0.0),
        action("drone", LabCmd::Velocity, 0.0, 3.0, 0.0),
        action("rover", LabCmd::Estop, 0.0, 0.0, 0.0),
        action("rover", LabCmd::Drive, -4.0, 0.0, 0.0),
    ]
}
