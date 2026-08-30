//! Host-side tick of the `no_std` kernel (NEXT B8).
//!
//! Discrete aerial / ground / marine / HITL machines are the trusted
//! computing base. This walk is what a microcontroller companion can copy:
//! `step` / `ground_step` / `marine_step` / `deadline_outcome` only. It is
//! **not** a typestate `Vehicle` handle (remaining-spec §6.1).

use crate::ground::{ground_step, GroundEvent, GroundPhase, GroundState};
use crate::hitl::{command_after_deadline, deadline_outcome, DeadlineOutcome, DeadlineSpec};
use crate::marine::{marine_step, MarineEvent, MarinePhase, MarineState};
use crate::safety::{self, Event, Phase, SafetyState};

/// Phases after one host kernel walk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KernelTick {
    pub aerial: Phase,
    pub ground: GroundPhase,
    pub marine: MarinePhase,
    pub hitl_met: bool,
    pub hitl_miss_zeros: bool,
}

/// Walk connect → takeoff, parked → halt, docked → dock, and a HITL miss-zero.
pub fn kernel_host_tick() -> KernelTick {
    let aerial = safety::step_all(
        SafetyState::disconnected(),
        &[
            Event::Connect,
            Event::InitComplete,
            Event::Initialized,
            Event::ImuHealthy,
            Event::EstimatorValid,
            Event::PreflightPassed,
            Event::Arm,
            Event::HeartbeatFresh,
            Event::EnterOffboard,
            Event::EnableActuators,
            Event::Takeoff,
        ],
    )
    .expect("connect-to-takeoff is legal");

    let mut ground = GroundState::parked();
    ground = ground_step(ground, GroundEvent::Release).expect("release");
    ground = ground_step(ground, GroundEvent::DriveCommand).expect("drive");
    ground = ground_step(ground, GroundEvent::Halt).expect("halt");

    let mut marine = MarineState::docked();
    marine = marine_step(marine, MarineEvent::Undock).expect("undock");
    marine = marine_step(marine, MarineEvent::Station).expect("station");
    marine = marine_step(marine, MarineEvent::Dock).expect("dock");

    let spec = DeadlineSpec::HZ_50;
    let met = matches!(
        deadline_outcome(spec.budget_ns, spec),
        DeadlineOutcome::Met { .. }
    );
    let missed = deadline_outcome(spec.budget_ns + 1, spec);
    let hitl_miss_zeros = matches!(missed, DeadlineOutcome::Missed { .. })
        && command_after_deadline(true, [1.0, 2.0, 3.0]) == [0.0, 0.0, 0.0];

    KernelTick {
        aerial: aerial.phase,
        ground: ground.phase,
        marine: marine.phase,
        hitl_met: met,
        hitl_miss_zeros,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kernel_host_tick_walks_discrete_machines() {
        let t = kernel_host_tick();
        assert_eq!(t.aerial, Phase::Takeoff);
        assert_eq!(t.ground, GroundPhase::Parked);
        assert_eq!(t.marine, MarinePhase::Docked);
        assert!(t.hitl_met);
        assert!(t.hitl_miss_zeros);
    }
}
