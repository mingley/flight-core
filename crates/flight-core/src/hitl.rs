//! Hardware-in-the-loop deadlines.
//!
//! A rack frame either meets its compute budget or it does not. A miss is a
//! first-class outcome: the applied command must be zero (failsafe), never the
//! command that arrived late.

#[cfg(creusot)]
use creusot_contracts::{ensures, logic, open};

/// Compute budget for one control frame. `budget_ns` must be ≤ `period_ns`.
#[derive(Copy)]
#[cfg_attr(not(creusot), derive(Clone, Debug, PartialEq, Eq))]
#[cfg_attr(
    creusot,
    derive(
        creusot_contracts::DeepModel,
        creusot_contracts::Clone,
        creusot_contracts::PartialEq
    )
)]
pub struct DeadlineSpec {
    pub period_ns: u64,
    pub budget_ns: u64,
}

impl DeadlineSpec {
    /// 50 Hz frame, 80 % of the period allowed for compute + plant step.
    pub const HZ_50: Self = Self {
        period_ns: 20_000_000,
        budget_ns: 16_000_000,
    };

    pub const fn valid(self) -> bool {
        self.period_ns > 0 && self.budget_ns > 0 && self.budget_ns <= self.period_ns
    }
}

#[derive(Copy)]
#[cfg_attr(not(creusot), derive(Clone, Debug, PartialEq, Eq))]
#[cfg_attr(
    creusot,
    derive(
        creusot_contracts::DeepModel,
        creusot_contracts::Clone,
        creusot_contracts::PartialEq
    )
)]
pub enum DeadlineOutcome {
    Met { compute_ns: u64 },
    Missed { compute_ns: u64, budget_ns: u64 },
}

impl DeadlineOutcome {
    pub const fn missed(self) -> bool {
        matches!(self, Self::Missed { .. })
    }

    pub const fn compute_ns(self) -> u64 {
        match self {
            Self::Met { compute_ns } | Self::Missed { compute_ns, .. } => compute_ns,
        }
    }
}

#[cfg_attr(
    feature = "creusot",
    creusot_contracts::requires(deadline_spec_ok(spec))
)]
#[cfg_attr(
    feature = "creusot",
    creusot_contracts::ensures(deadline_outcome_ok(result, compute_ns, spec))
)]
pub fn deadline_outcome(compute_ns: u64, spec: DeadlineSpec) -> DeadlineOutcome {
    if compute_ns <= spec.budget_ns {
        DeadlineOutcome::Met { compute_ns }
    } else {
        DeadlineOutcome::Missed {
            compute_ns,
            budget_ns: spec.budget_ns,
        }
    }
}

/// A missed deadline may not apply a new actuator command.
/// The plant-zeroing helper `command_after_deadline` is rustc/Kani: Creusot 0.5
/// has no `DeepModel` for `f32`.
#[cfg_attr(feature = "creusot", creusot_contracts::ensures(result == (missed == false)))]
pub fn hitl_apply_allowed(missed: bool) -> bool {
    !missed
}

/// Command that actually goes to the plant. A miss zeros the setpoint.
#[cfg(not(creusot))]
pub fn command_after_deadline(missed: bool, next: [f32; 3]) -> [f32; 3] {
    if missed {
        [0.0, 0.0, 0.0]
    } else if next[0].is_finite() && next[1].is_finite() && next[2].is_finite() {
        next
    } else {
        [0.0, 0.0, 0.0]
    }
}

#[cfg(not(creusot))]
pub fn hitl_invariants(missed: bool, applied: [f32; 3], next: [f32; 3]) -> bool {
    applied == command_after_deadline(missed, next)
}

#[cfg(creusot)]
use creusot_contracts::predicate;

#[cfg(creusot)]
#[predicate]
fn deadline_spec_ok(spec: DeadlineSpec) -> bool {
    spec.period_ns > 0u64 && spec.budget_ns > 0u64 && spec.budget_ns <= spec.period_ns
}

#[cfg(creusot)]
#[predicate]
fn deadline_outcome_ok(o: DeadlineOutcome, compute_ns: u64, spec: DeadlineSpec) -> bool {
    match o {
        DeadlineOutcome::Missed { .. } => compute_ns > spec.budget_ns,
        DeadlineOutcome::Met { .. } => compute_ns <= spec.budget_ns,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn met_when_under_budget() {
        let spec = DeadlineSpec::HZ_50;
        assert!(!deadline_outcome(1_000, spec).missed());
        assert_eq!(
            deadline_outcome(spec.budget_ns, spec),
            DeadlineOutcome::Met {
                compute_ns: spec.budget_ns
            }
        );
    }

    #[test]
    fn miss_zeros_command() {
        let next = [1.0, -0.4, 0.2];
        assert_eq!(command_after_deadline(true, next), [0.0, 0.0, 0.0]);
        assert_eq!(command_after_deadline(false, next), next);
        assert!(hitl_invariants(true, [0.0, 0.0, 0.0], next));
        assert!(!hitl_invariants(true, next, next));
        assert!(hitl_invariants(false, next, next));
        assert!(!hitl_apply_allowed(true));
        assert!(hitl_apply_allowed(false));
    }

    #[test]
    fn nan_command_is_zero_even_on_time() {
        assert_eq!(
            command_after_deadline(false, [f32::NAN, 0.0, 0.0]),
            [0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn exhaustive_miss_is_zero() {
        let cmds = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -2.0, 0.4]];
        for next in cmds {
            for missed in [false, true] {
                let applied = command_after_deadline(missed, next);
                assert!(hitl_invariants(missed, applied, next));
            }
        }
    }
}
