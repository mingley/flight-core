//! Ground-vehicle safety machine.
//!
//! ```text
//! Parked ──release──► Moving ──halt──► Parked
//! any ──estop──► EStop ──clear──► Parked
//! drive command  ⇒  Moving ∧ ¬estop
//! ```

#[cfg(not(creusot))]
use core::fmt;

#[cfg(creusot)]
use creusot_contracts::{ensures, logic, open};

#[derive(Copy)]
#[cfg_attr(not(creusot), derive(Clone, Debug, PartialEq, Eq, Hash))]
#[cfg_attr(
    creusot,
    derive(
        creusot_contracts::DeepModel,
        creusot_contracts::Clone,
        creusot_contracts::PartialEq
    )
)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
#[repr(u8)]
pub enum GroundPhase {
    Parked = 0,
    Moving = 1,
    EStop = 2,
}

impl GroundPhase {
    pub const ALL: [GroundPhase; 3] =
        [GroundPhase::Parked, GroundPhase::Moving, GroundPhase::EStop];

    pub const fn name(self) -> &'static str {
        match self {
            GroundPhase::Parked => "parked",
            GroundPhase::Moving => "moving",
            GroundPhase::EStop => "estop",
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(GroundPhase::Parked),
            1 => Some(GroundPhase::Moving),
            2 => Some(GroundPhase::EStop),
            _ => None,
        }
    }
}

#[cfg(not(creusot))]
impl fmt::Display for GroundPhase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

#[derive(Copy)]
#[cfg_attr(not(creusot), derive(Clone, Debug, PartialEq, Eq, Hash))]
#[cfg_attr(
    creusot,
    derive(
        creusot_contracts::DeepModel,
        creusot_contracts::Clone,
        creusot_contracts::PartialEq
    )
)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct GroundState {
    pub phase: GroundPhase,
    pub drive_enabled: bool,
    pub estop: bool,
}

impl GroundState {
    pub const fn parked() -> Self {
        Self {
            phase: GroundPhase::Parked,
            drive_enabled: false,
            estop: false,
        }
    }
}

#[cfg(not(creusot))]
impl Default for GroundState {
    fn default() -> Self {
        Self::parked()
    }
}

#[derive(Copy)]
#[cfg_attr(not(creusot), derive(Clone, Debug, PartialEq, Eq, Hash))]
#[cfg_attr(
    creusot,
    derive(
        creusot_contracts::DeepModel,
        creusot_contracts::Clone,
        creusot_contracts::PartialEq
    )
)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
#[repr(u8)]
pub enum GroundEvent {
    Release = 0,
    Halt = 1,
    EStop = 2,
    ClearEstop = 3,
    DriveCommand = 4,
}

impl GroundEvent {
    pub const ALL: [GroundEvent; 5] = [
        GroundEvent::Release,
        GroundEvent::Halt,
        GroundEvent::EStop,
        GroundEvent::ClearEstop,
        GroundEvent::DriveCommand,
    ];

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(GroundEvent::Release),
            1 => Some(GroundEvent::Halt),
            2 => Some(GroundEvent::EStop),
            3 => Some(GroundEvent::ClearEstop),
            4 => Some(GroundEvent::DriveCommand),
            _ => None,
        }
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
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum GroundReject {
    IllegalPhase,
    EStopped,
}

#[cfg(not(creusot))]
impl fmt::Display for GroundReject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GroundReject::IllegalPhase => write!(f, "illegal ground phase"),
            GroundReject::EStopped => write!(f, "ground vehicle is in E-stop"),
        }
    }
}

pub fn ground_invariants(s: &GroundState) -> bool {
    match s.phase {
        GroundPhase::EStop => s.estop && !s.drive_enabled,
        GroundPhase::Moving => !s.estop,
        GroundPhase::Parked => !s.estop && !s.drive_enabled,
    }
}

#[cfg_attr(feature = "creusot", creusot_contracts::requires(inv_ground(s)))]
#[cfg_attr(
    feature = "creusot",
    creusot_contracts::ensures(match result {
        Ok(n) => inv_ground(n),
        Err(_) => true,
    })
)]
pub fn ground_step(s: GroundState, e: GroundEvent) -> Result<GroundState, GroundReject> {
    let mut n = s;
    match e {
        GroundEvent::Release => {
            if n.phase != GroundPhase::Parked {
                return Err(GroundReject::IllegalPhase);
            }
            n.phase = GroundPhase::Moving;
            n.drive_enabled = true;
        }
        GroundEvent::Halt => {
            if n.phase != GroundPhase::Moving {
                return Err(GroundReject::IllegalPhase);
            }
            n.phase = GroundPhase::Parked;
            n.drive_enabled = false;
        }
        GroundEvent::EStop => {
            n.phase = GroundPhase::EStop;
            n.estop = true;
            n.drive_enabled = false;
        }
        GroundEvent::ClearEstop => {
            if n.phase != GroundPhase::EStop {
                return Err(GroundReject::IllegalPhase);
            }
            n.estop = false;
            n.phase = GroundPhase::Parked;
            n.drive_enabled = false;
        }
        GroundEvent::DriveCommand => {
            if n.estop {
                return Err(GroundReject::EStopped);
            }
            if n.phase != GroundPhase::Moving || !n.drive_enabled {
                return Err(GroundReject::IllegalPhase);
            }
        }
    }
    #[cfg(not(creusot))]
    debug_assert!(ground_invariants(&n));
    Ok(n)
}

#[cfg(not(creusot))]
pub fn pack_ground(s: &GroundState) -> u8 {
    let mut v = s.phase as u8;
    if s.drive_enabled {
        v |= 1 << 2;
    }
    if s.estop {
        v |= 1 << 3;
    }
    v
}

#[cfg(not(creusot))]
pub fn unpack_ground(v: u8) -> Option<GroundState> {
    let phase = GroundPhase::from_u8(v & 0b11)?;
    Some(GroundState {
        phase,
        drive_enabled: v & (1 << 2) != 0,
        estop: v & (1 << 3) != 0,
    })
}

#[cfg(creusot)]
use creusot_contracts::predicate;

#[cfg(creusot)]
#[predicate]
fn inv_ground(s: GroundState) -> bool {
    (s.phase == GroundPhase::EStop && s.estop && s.drive_enabled == false)
        || (s.phase == GroundPhase::Moving && s.estop == false)
        || (s.phase == GroundPhase::Parked && s.estop == false && s.drive_enabled == false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cannot_drive_while_parked() {
        let s = GroundState::parked();
        assert_eq!(
            ground_step(s, GroundEvent::DriveCommand),
            Err(GroundReject::IllegalPhase)
        );
    }

    #[test]
    fn estop_kills_drive() {
        let s = ground_step(GroundState::parked(), GroundEvent::Release).unwrap();
        assert!(s.drive_enabled);
        let s = ground_step(s, GroundEvent::EStop).unwrap();
        assert!(s.estop);
        assert!(!s.drive_enabled);
        assert_eq!(
            ground_step(s, GroundEvent::DriveCommand),
            Err(GroundReject::EStopped)
        );
    }

    #[test]
    fn inductive_ground() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            for e in GroundEvent::ALL {
                if let Ok(n) = ground_step(s, e) {
                    assert!(ground_invariants(&n));
                    assert!(!n.drive_enabled || n.phase == GroundPhase::Moving);
                }
            }
        }
    }

    #[test]
    fn clear_estop_only_from_estop_returns_parked() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            match ground_step(s, GroundEvent::ClearEstop) {
                Ok(n) => {
                    assert_eq!(s.phase, GroundPhase::EStop);
                    assert_eq!(n.phase, GroundPhase::Parked);
                    assert!(!n.estop && !n.drive_enabled);
                    assert!(ground_invariants(&n));
                }
                Err(_) => assert_ne!(s.phase, GroundPhase::EStop),
            }
        }
    }

    #[test]
    fn halt_only_from_moving_returns_parked() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            match ground_step(s, GroundEvent::Halt) {
                Ok(n) => {
                    assert_eq!(s.phase, GroundPhase::Moving);
                    assert_eq!(n.phase, GroundPhase::Parked);
                    assert!(!n.drive_enabled && !n.estop);
                    assert!(ground_invariants(&n));
                }
                Err(_) => assert_ne!(s.phase, GroundPhase::Moving),
            }
        }
    }

    #[test]
    fn estop_always_returns_estopped() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            let n = ground_step(s, GroundEvent::EStop).unwrap();
            assert_eq!(n.phase, GroundPhase::EStop);
            assert!(n.estop && !n.drive_enabled);
            assert!(ground_invariants(&n));
        }
    }

    #[test]
    fn release_only_from_parked_returns_moving() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            match ground_step(s, GroundEvent::Release) {
                Ok(n) => {
                    assert_eq!(s.phase, GroundPhase::Parked);
                    assert_eq!(n.phase, GroundPhase::Moving);
                    assert!(n.drive_enabled && !n.estop);
                    assert!(ground_invariants(&n));
                }
                Err(_) => assert_ne!(s.phase, GroundPhase::Parked),
            }
        }
    }

    #[test]
    fn drive_command_only_when_moving_and_enabled() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_ground(bits) else {
                continue;
            };
            if !ground_invariants(&s) {
                continue;
            }
            match ground_step(s, GroundEvent::DriveCommand) {
                Ok(n) => {
                    assert!(!s.estop);
                    assert_eq!(s.phase, GroundPhase::Moving);
                    assert!(s.drive_enabled);
                    assert_eq!(n, s);
                    assert!(ground_invariants(&n));
                }
                Err(GroundReject::EStopped) => assert!(s.estop),
                Err(GroundReject::IllegalPhase) => {
                    assert!(!s.estop);
                    assert!(s.phase != GroundPhase::Moving || !s.drive_enabled);
                }
            }
        }
    }
}
