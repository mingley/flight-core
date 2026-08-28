//! Verification harness for the vehicle safety state machine.
//!
//! Exhaustive tests live in `flight-core::safety` and run under `cargo test`.
//! This crate holds the Kani proofs:
//!
//! ```text
//! cargo install --locked kani-verifier
//! cargo kani -p flight-verify
//! ```
//!
//! The central theorem:
//!
//! > There exists no transition sequence that reaches
//! > `actuators_enabled` while `armed == false`.

#![deny(unsafe_code)]
#![allow(unexpected_cfgs)]

use flight_core::safety::{check_invariants, step, Event, Reject, SafetyState};

/// Inductive step: any invariant-satisfying state, after a successful `step`,
/// still satisfies the invariants, and never enables actuators while disarmed.
pub fn inductive_step(s: SafetyState, e: Event) -> Result<SafetyState, Reject> {
    if !check_invariants(&s) {
        return Err(Reject::IllegalPhase);
    }
    let n = step(s, e)?;
    debug_assert!(check_invariants(&n));
    debug_assert!(!n.actuators_enabled || n.armed);
    Ok(n)
}

#[cfg(kani)]
mod proofs {
    use super::*;
    use flight_core::safety::{unpack, Event, Phase};

    #[kani::proof]
    fn actuators_require_arm() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        let ev: u8 = kani::any();
        kani::assume(ev <= 23);
        let Some(e) = Event::from_u8(ev) else { return };
        if let Ok(n) = step(s, e) {
            assert!(check_invariants(&n));
            assert!(!n.actuators_enabled || n.armed);
            if n.phase == flight_core::safety::Phase::Takeoff
                || n.phase == flight_core::safety::Phase::Airborne
            {
                assert!(n.actuators_enabled);
                assert!(n.armed);
            }
        }
    }

    #[kani::proof]
    fn failsafe_blocks_mission_commands() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        kani::assume(s.failsafe);
        assert!(step(s, Event::MissionCommand).is_err());
        assert!(step(s, Event::Arm).is_err());
        assert!(step(s, Event::Takeoff).is_err());
        assert!(step(s, Event::Land).is_err());
        assert!(step(s, Event::EnterOffboard).is_err());
    }

    #[kani::proof]
    fn arm_requires_sensors() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(mut s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        s.phase = flight_core::safety::Phase::Ready;
        s.armed = false;
        s.actuators_enabled = false;
        s.offboard = false;
        s.failsafe = false;
        kani::assume(check_invariants(&s));
        kani::assume(!s.imu_healthy || !s.estimator_valid);
        assert!(step(s, Event::Arm).is_err());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::safety::{pack, unpack, Event};

    #[test]
    fn kani_wrapper_agrees_with_step() {
        let s = SafetyState::disconnected();
        assert!(inductive_step(s, Event::Connect).is_ok());
        assert!(inductive_step(s, Event::Arm).is_err());
    }

    #[test]
    fn packed_roundtrip() {
        for bits in 0u16..=0x07FF {
            if let Some(s) = unpack(bits) {
                assert_eq!(pack(&s), bits);
            }
        }
    }
}
