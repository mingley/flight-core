//! Surface / underwater safety machine.
//!
//! ```text
//! Docked ──undock──► Underway ──station──► StationKeep ──dock──► Docked
//! any ──failsafe──► Failsafe ──recover──► Docked
//! thrust  ⇒  (Underway ∨ StationKeep) ∧ ¬failsafe
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
pub enum MarinePhase {
    Docked = 0,
    Underway = 1,
    StationKeep = 2,
    Failsafe = 3,
}

impl MarinePhase {
    pub const ALL: [MarinePhase; 4] = [
        MarinePhase::Docked,
        MarinePhase::Underway,
        MarinePhase::StationKeep,
        MarinePhase::Failsafe,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            MarinePhase::Docked => "docked",
            MarinePhase::Underway => "underway",
            MarinePhase::StationKeep => "station_keep",
            MarinePhase::Failsafe => "failsafe",
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MarinePhase::Docked),
            1 => Some(MarinePhase::Underway),
            2 => Some(MarinePhase::StationKeep),
            3 => Some(MarinePhase::Failsafe),
            _ => None,
        }
    }
}

#[cfg(not(creusot))]
impl fmt::Display for MarinePhase {
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
pub struct MarineState {
    pub phase: MarinePhase,
    pub thrust_enabled: bool,
    pub failsafe: bool,
}

impl MarineState {
    pub const fn docked() -> Self {
        Self {
            phase: MarinePhase::Docked,
            thrust_enabled: false,
            failsafe: false,
        }
    }
}

#[cfg(not(creusot))]
impl Default for MarineState {
    fn default() -> Self {
        Self::docked()
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
pub enum MarineEvent {
    Undock = 0,
    Dock = 1,
    Station = 2,
    Resume = 3,
    ThrustCommand = 4,
    Failsafe = 5,
    Recover = 6,
}

impl MarineEvent {
    pub const ALL: [MarineEvent; 7] = [
        MarineEvent::Undock,
        MarineEvent::Dock,
        MarineEvent::Station,
        MarineEvent::Resume,
        MarineEvent::ThrustCommand,
        MarineEvent::Failsafe,
        MarineEvent::Recover,
    ];

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MarineEvent::Undock),
            1 => Some(MarineEvent::Dock),
            2 => Some(MarineEvent::Station),
            3 => Some(MarineEvent::Resume),
            4 => Some(MarineEvent::ThrustCommand),
            5 => Some(MarineEvent::Failsafe),
            6 => Some(MarineEvent::Recover),
            _ => None,
        }
    }
}

/// Failsafe and dock revoke hull thrust authority. Same table as
/// [`MARINE_AUTHORITY_REVOKE_EVENTS`].
pub const fn marine_event_revokes_authority(event: MarineEvent) -> bool {
    matches!(event, MarineEvent::Failsafe | MarineEvent::Dock)
}

/// Events that revoke marine thrust authority.
#[cfg(not(creusot))]
pub const MARINE_AUTHORITY_REVOKE_EVENTS: &[MarineEvent] =
    &[MarineEvent::Failsafe, MarineEvent::Dock];

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
pub enum MarineReject {
    IllegalPhase,
    InFailsafe,
}

#[cfg(not(creusot))]
impl fmt::Display for MarineReject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MarineReject::IllegalPhase => write!(f, "illegal marine phase"),
            MarineReject::InFailsafe => write!(f, "marine failsafe rejects thrust"),
        }
    }
}

pub fn marine_invariants(s: &MarineState) -> bool {
    match s.phase {
        MarinePhase::Failsafe => s.failsafe && !s.thrust_enabled,
        MarinePhase::Underway | MarinePhase::StationKeep => !s.failsafe,
        MarinePhase::Docked => !s.failsafe && !s.thrust_enabled,
    }
}

#[cfg_attr(feature = "creusot", creusot_contracts::requires(inv_marine(s)))]
#[cfg_attr(
    feature = "creusot",
    creusot_contracts::ensures(match result {
        Ok(n) => inv_marine(n),
        Err(_) => true,
    })
)]
pub fn marine_step(s: MarineState, e: MarineEvent) -> Result<MarineState, MarineReject> {
    if s.failsafe {
        match e {
            MarineEvent::Recover | MarineEvent::Failsafe | MarineEvent::Dock => {}
            MarineEvent::ThrustCommand
            | MarineEvent::Undock
            | MarineEvent::Station
            | MarineEvent::Resume => {
                return Err(MarineReject::InFailsafe);
            }
        }
    }
    let mut n = s;
    match e {
        MarineEvent::Undock => {
            if n.phase != MarinePhase::Docked {
                return Err(MarineReject::IllegalPhase);
            }
            n.phase = MarinePhase::Underway;
            n.thrust_enabled = true;
        }
        MarineEvent::Dock => {
            n.phase = MarinePhase::Docked;
            n.thrust_enabled = false;
            n.failsafe = false;
        }
        MarineEvent::Station => {
            if n.phase != MarinePhase::Underway {
                return Err(MarineReject::IllegalPhase);
            }
            n.phase = MarinePhase::StationKeep;
            n.thrust_enabled = true;
        }
        MarineEvent::Resume => {
            if n.phase != MarinePhase::StationKeep {
                return Err(MarineReject::IllegalPhase);
            }
            n.phase = MarinePhase::Underway;
            n.thrust_enabled = true;
        }
        MarineEvent::ThrustCommand => {
            if n.failsafe {
                return Err(MarineReject::InFailsafe);
            }
            if !n.thrust_enabled {
                return Err(MarineReject::IllegalPhase);
            }
        }
        MarineEvent::Failsafe => {
            n.phase = MarinePhase::Failsafe;
            n.failsafe = true;
            n.thrust_enabled = false;
        }
        MarineEvent::Recover => {
            if n.phase != MarinePhase::Failsafe {
                return Err(MarineReject::IllegalPhase);
            }
            n.failsafe = false;
            n.phase = MarinePhase::Docked;
            n.thrust_enabled = false;
        }
    }
    #[cfg(not(creusot))]
    debug_assert!(marine_invariants(&n));
    Ok(n)
}

#[cfg(not(creusot))]
pub fn pack_marine(s: &MarineState) -> u8 {
    let mut v = s.phase as u8;
    if s.thrust_enabled {
        v |= 1 << 2;
    }
    if s.failsafe {
        v |= 1 << 3;
    }
    v
}

#[cfg(not(creusot))]
pub fn unpack_marine(v: u8) -> Option<MarineState> {
    let phase = MarinePhase::from_u8(v & 0b11)?;
    Some(MarineState {
        phase,
        thrust_enabled: v & (1 << 2) != 0,
        failsafe: v & (1 << 3) != 0,
    })
}

#[cfg(creusot)]
use creusot_contracts::predicate;

#[cfg(creusot)]
#[predicate]
fn inv_marine(s: MarineState) -> bool {
    (s.phase == MarinePhase::Failsafe && s.failsafe && s.thrust_enabled == false)
        || (s.phase == MarinePhase::Underway && s.failsafe == false)
        || (s.phase == MarinePhase::StationKeep && s.failsafe == false)
        || (s.phase == MarinePhase::Docked && s.failsafe == false && s.thrust_enabled == false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn docked_cannot_thrust() {
        assert_eq!(
            marine_step(MarineState::docked(), MarineEvent::ThrustCommand),
            Err(MarineReject::IllegalPhase)
        );
    }

    #[test]
    fn undock_then_thrust() {
        let s = marine_step(MarineState::docked(), MarineEvent::Undock).unwrap();
        assert!(s.thrust_enabled);
        assert!(marine_step(s, MarineEvent::ThrustCommand).is_ok());
    }

    #[test]
    fn inductive_marine() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            for e in MarineEvent::ALL {
                if let Ok(n) = marine_step(s, e) {
                    assert!(marine_invariants(&n));
                    assert!(
                        !n.thrust_enabled
                            || matches!(n.phase, MarinePhase::Underway | MarinePhase::StationKeep)
                    );
                }
            }
        }
    }

    #[test]
    fn dock_always_returns_docked() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            let n = marine_step(s, MarineEvent::Dock).unwrap();
            assert_eq!(n.phase, MarinePhase::Docked);
            assert!(!n.thrust_enabled && !n.failsafe);
            assert!(marine_invariants(&n));
        }
    }

    #[test]
    fn failsafe_always_returns_failsafe() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            let n = marine_step(s, MarineEvent::Failsafe).unwrap();
            assert_eq!(n.phase, MarinePhase::Failsafe);
            assert!(n.failsafe && !n.thrust_enabled);
            assert!(marine_invariants(&n));
        }
    }

    #[test]
    fn recover_only_from_failsafe_returns_docked() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            match marine_step(s, MarineEvent::Recover) {
                Ok(n) => {
                    assert_eq!(s.phase, MarinePhase::Failsafe);
                    assert_eq!(n.phase, MarinePhase::Docked);
                    assert!(!n.failsafe && !n.thrust_enabled);
                    assert!(marine_invariants(&n));
                }
                Err(_) => assert_ne!(s.phase, MarinePhase::Failsafe),
            }
        }
    }

    #[test]
    fn undock_only_from_docked_returns_underway() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            match marine_step(s, MarineEvent::Undock) {
                Ok(n) => {
                    assert_eq!(s.phase, MarinePhase::Docked);
                    assert!(!s.failsafe);
                    assert_eq!(n.phase, MarinePhase::Underway);
                    assert!(n.thrust_enabled);
                    assert!(marine_invariants(&n));
                }
                Err(_) => assert!(s.failsafe || s.phase != MarinePhase::Docked),
            }
        }
    }

    #[test]
    fn station_only_from_underway_returns_station_keep() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            match marine_step(s, MarineEvent::Station) {
                Ok(n) => {
                    assert_eq!(s.phase, MarinePhase::Underway);
                    assert!(!s.failsafe);
                    assert_eq!(n.phase, MarinePhase::StationKeep);
                    assert!(n.thrust_enabled);
                    assert!(marine_invariants(&n));
                }
                Err(_) => assert!(s.failsafe || s.phase != MarinePhase::Underway),
            }
        }
    }

    #[test]
    fn resume_only_from_station_keep_returns_underway() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            match marine_step(s, MarineEvent::Resume) {
                Ok(n) => {
                    assert_eq!(s.phase, MarinePhase::StationKeep);
                    assert!(!s.failsafe);
                    assert_eq!(n.phase, MarinePhase::Underway);
                    assert!(n.thrust_enabled);
                    assert!(marine_invariants(&n));
                }
                Err(_) => assert!(s.failsafe || s.phase != MarinePhase::StationKeep),
            }
        }
    }

    #[test]
    fn thrust_command_only_when_granted() {
        for bits in 0u8..=0x0F {
            let Some(s) = unpack_marine(bits) else {
                continue;
            };
            if !marine_invariants(&s) {
                continue;
            }
            match marine_step(s, MarineEvent::ThrustCommand) {
                Ok(n) => {
                    assert!(!s.failsafe);
                    assert!(s.thrust_enabled);
                    assert_eq!(n, s);
                    assert!(marine_invariants(&n));
                }
                Err(MarineReject::InFailsafe) => assert!(s.failsafe),
                Err(MarineReject::IllegalPhase) => {
                    assert!(!s.failsafe);
                    assert!(!s.thrust_enabled);
                }
            }
        }
    }
}
