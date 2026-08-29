//! Pure vehicle safety state machine.
//!
//! This module is deliberately free of I/O, allocation, panic, and `unsafe`.
//! `step` is the single transition function. Tests (and Kani) prove that every
//! successful transition preserves:
//!
//! ```text
//! actuators_enabled  ⇒  armed
//! airborne           ⇒  actuators_enabled
//! armed ∧ ¬failsafe  ⇒  imu_healthy ∧ estimator_valid
//! failsafe           ⇒  mission commands rejected
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
pub enum Phase {
    Disconnected = 0,
    Connected = 1,
    Initializing = 2,
    Preflight = 3,
    Ready = 4,
    Armed = 5,
    Takeoff = 6,
    Airborne = 7,
    Landing = 8,
    Failsafe = 9,
    Recovery = 10,
}

impl Phase {
    pub const ALL: [Phase; 11] = [
        Phase::Disconnected,
        Phase::Connected,
        Phase::Initializing,
        Phase::Preflight,
        Phase::Ready,
        Phase::Armed,
        Phase::Takeoff,
        Phase::Airborne,
        Phase::Landing,
        Phase::Failsafe,
        Phase::Recovery,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Phase::Disconnected => "disconnected",
            Phase::Connected => "connected",
            Phase::Initializing => "initializing",
            Phase::Preflight => "preflight",
            Phase::Ready => "ready",
            Phase::Armed => "armed",
            Phase::Takeoff => "takeoff",
            Phase::Airborne => "airborne",
            Phase::Landing => "landing",
            Phase::Failsafe => "failsafe",
            Phase::Recovery => "recovery",
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Phase::Disconnected),
            1 => Some(Phase::Connected),
            2 => Some(Phase::Initializing),
            3 => Some(Phase::Preflight),
            4 => Some(Phase::Ready),
            5 => Some(Phase::Armed),
            6 => Some(Phase::Takeoff),
            7 => Some(Phase::Airborne),
            8 => Some(Phase::Landing),
            9 => Some(Phase::Failsafe),
            10 => Some(Phase::Recovery),
            _ => None,
        }
    }
}

#[cfg(not(creusot))]
impl fmt::Display for Phase {
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
pub struct SafetyState {
    pub phase: Phase,
    pub armed: bool,
    pub actuators_enabled: bool,
    pub imu_healthy: bool,
    pub estimator_valid: bool,
    pub offboard_heartbeat_fresh: bool,
    pub offboard: bool,
    pub failsafe: bool,
}

impl SafetyState {
    pub const fn disconnected() -> Self {
        Self {
            phase: Phase::Disconnected,
            armed: false,
            actuators_enabled: false,
            imu_healthy: false,
            estimator_valid: false,
            offboard_heartbeat_fresh: false,
            offboard: false,
            failsafe: false,
        }
    }
}

#[cfg(not(creusot))]
impl Default for SafetyState {
    fn default() -> Self {
        Self::disconnected()
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
pub enum Event {
    Connect = 0,
    Disconnect = 1,
    InitComplete = 2,
    Initialized = 3,
    PreflightPassed = 4,
    PreflightFailed = 5,
    ImuHealthy = 6,
    ImuUnhealthy = 7,
    EstimatorValid = 8,
    EstimatorInvalid = 9,
    Arm = 10,
    Disarm = 11,
    EnterOffboard = 12,
    HeartbeatFresh = 13,
    HeartbeatStale = 14,
    EnableActuators = 15,
    DisableActuators = 16,
    Takeoff = 17,
    ReachedAltitude = 18,
    Land = 19,
    Touchdown = 20,
    MissionCommand = 21,
    TriggerFailsafe = 22,
    Recover = 23,
}

impl Event {
    pub const ALL: [Event; 24] = [
        Event::Connect,
        Event::Disconnect,
        Event::InitComplete,
        Event::Initialized,
        Event::PreflightPassed,
        Event::PreflightFailed,
        Event::ImuHealthy,
        Event::ImuUnhealthy,
        Event::EstimatorValid,
        Event::EstimatorInvalid,
        Event::Arm,
        Event::Disarm,
        Event::EnterOffboard,
        Event::HeartbeatFresh,
        Event::HeartbeatStale,
        Event::EnableActuators,
        Event::DisableActuators,
        Event::Takeoff,
        Event::ReachedAltitude,
        Event::Land,
        Event::Touchdown,
        Event::MissionCommand,
        Event::TriggerFailsafe,
        Event::Recover,
    ];

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Event::Connect),
            1 => Some(Event::Disconnect),
            2 => Some(Event::InitComplete),
            3 => Some(Event::Initialized),
            4 => Some(Event::PreflightPassed),
            5 => Some(Event::PreflightFailed),
            6 => Some(Event::ImuHealthy),
            7 => Some(Event::ImuUnhealthy),
            8 => Some(Event::EstimatorValid),
            9 => Some(Event::EstimatorInvalid),
            10 => Some(Event::Arm),
            11 => Some(Event::Disarm),
            12 => Some(Event::EnterOffboard),
            13 => Some(Event::HeartbeatFresh),
            14 => Some(Event::HeartbeatStale),
            15 => Some(Event::EnableActuators),
            16 => Some(Event::DisableActuators),
            17 => Some(Event::Takeoff),
            18 => Some(Event::ReachedAltitude),
            19 => Some(Event::Land),
            20 => Some(Event::Touchdown),
            21 => Some(Event::MissionCommand),
            22 => Some(Event::TriggerFailsafe),
            23 => Some(Event::Recover),
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
pub enum Reject {
    IllegalPhase,
    ImuUnhealthy,
    EstimatorInvalid,
    NotArmed,
    HeartbeatStale,
    InFailsafe,
    ActuatorsDisabled,
}

#[cfg(not(creusot))]
impl fmt::Display for Reject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Reject::IllegalPhase => "illegal phase for this event",
            Reject::ImuUnhealthy => "IMU unhealthy",
            Reject::EstimatorInvalid => "estimator invalid",
            Reject::NotArmed => "vehicle is not armed",
            Reject::HeartbeatStale => "offboard heartbeat stale",
            Reject::InFailsafe => "failsafe is active; mission commands rejected",
            Reject::ActuatorsDisabled => "actuators are not enabled",
        })
    }
}

/// Core safety invariants. `step` must preserve these whenever it returns `Ok`.
pub fn check_invariants(s: &SafetyState) -> bool {
    if s.actuators_enabled && !s.armed {
        return false;
    }
    if s.offboard && !s.armed {
        return false;
    }
    // Failsafe latches through Recovery until Recover / Touchdown clears it.
    if s.phase == Phase::Failsafe && !s.failsafe {
        return false;
    }
    if s.failsafe && !matches!(s.phase, Phase::Failsafe | Phase::Recovery) {
        return false;
    }
    if s.phase == Phase::Disconnected
        && (s.armed || s.actuators_enabled || s.failsafe || s.offboard)
    {
        return false;
    }
    if matches!(
        s.phase,
        Phase::Armed | Phase::Takeoff | Phase::Airborne | Phase::Landing
    ) && !s.armed
    {
        return false;
    }
    if matches!(s.phase, Phase::Takeoff | Phase::Airborne | Phase::Landing) && !s.actuators_enabled
    {
        return false;
    }
    if s.armed && !s.failsafe && (!s.imu_healthy || !s.estimator_valid) {
        return false;
    }
    true
}

#[cfg_attr(feature = "creusot", creusot_contracts::requires(inv_aerial(n)))]
#[cfg_attr(
    feature = "creusot",
    creusot_contracts::ensures(step_ok(result) && failsafe_latched(result))
)]
fn enter_failsafe(mut n: SafetyState) -> SafetyState {
    n.failsafe = true;
    n.phase = Phase::Failsafe;
    n.offboard = false;
    n
}

/// Whether a successful [`step`] successor is still a legal machine.
pub fn step_preserves(n: SafetyState) -> bool {
    check_invariants(&n) && (!n.actuators_enabled || n.armed)
}

/// Single transition function. No panics, no allocation.
#[cfg_attr(feature = "creusot", creusot_contracts::requires(inv_aerial(s)))]
#[cfg_attr(
    feature = "creusot",
    creusot_contracts::ensures(match result {
        Ok(n) => step_ok(n),
        Err(_) => true,
    })
)]
pub fn step(s: SafetyState, e: Event) -> Result<SafetyState, Reject> {
    if s.failsafe {
        match e {
            Event::Disarm
            | Event::Recover
            | Event::DisableActuators
            | Event::TriggerFailsafe
            | Event::Touchdown
            | Event::ImuHealthy
            | Event::ImuUnhealthy
            | Event::EstimatorValid
            | Event::EstimatorInvalid
            | Event::HeartbeatFresh
            | Event::HeartbeatStale
            | Event::Disconnect => {}
            Event::MissionCommand
            | Event::Arm
            | Event::Takeoff
            | Event::EnterOffboard
            | Event::Land
            | Event::EnableActuators
            | Event::PreflightPassed
            | Event::ReachedAltitude => {
                return Err(Reject::InFailsafe);
            }
            _ => {}
        }
    }

    let mut n = s;
    match e {
        Event::Connect => {
            if n.phase != Phase::Disconnected {
                return Err(Reject::IllegalPhase);
            }
            n.phase = Phase::Connected;
        }
        Event::Disconnect => {
            n.phase = Phase::Disconnected;
            n.armed = false;
            n.actuators_enabled = false;
            n.offboard = false;
            n.failsafe = false;
        }
        Event::InitComplete => {
            if n.phase != Phase::Connected {
                return Err(Reject::IllegalPhase);
            }
            n.phase = Phase::Initializing;
        }
        Event::Initialized => {
            if n.phase != Phase::Initializing {
                return Err(Reject::IllegalPhase);
            }
            n.phase = Phase::Preflight;
        }
        Event::PreflightPassed => {
            if n.phase != Phase::Preflight {
                return Err(Reject::IllegalPhase);
            }
            if !n.imu_healthy {
                return Err(Reject::ImuUnhealthy);
            }
            if !n.estimator_valid {
                return Err(Reject::EstimatorInvalid);
            }
            n.phase = Phase::Ready;
        }
        Event::PreflightFailed => {
            if n.phase != Phase::Preflight {
                return Err(Reject::IllegalPhase);
            }
        }
        Event::ImuHealthy => {
            n.imu_healthy = true;
        }
        Event::ImuUnhealthy => {
            // Latch failsafe while `armed ∧ ¬failsafe ⇒ imu_healthy` still holds,
            // then clear the bit. The successor is the same; the intermediate is legal.
            if n.armed {
                n = enter_failsafe(n);
            }
            n.imu_healthy = false;
        }
        Event::EstimatorValid => {
            n.estimator_valid = true;
        }
        Event::EstimatorInvalid => {
            // Same order as ImuUnhealthy: latch first so the IMU/estimator
            // invariant is never broken in an intermediate state.
            if n.armed {
                n = enter_failsafe(n);
            }
            n.estimator_valid = false;
        }
        Event::Arm => {
            if n.phase != Phase::Ready {
                return Err(Reject::IllegalPhase);
            }
            if !n.imu_healthy {
                return Err(Reject::ImuUnhealthy);
            }
            if !n.estimator_valid {
                return Err(Reject::EstimatorInvalid);
            }
            n.armed = true;
            n.phase = Phase::Armed;
        }
        Event::Disarm => {
            if n.phase == Phase::Disconnected {
                return Err(Reject::IllegalPhase);
            }
            n.armed = false;
            n.actuators_enabled = false;
            n.offboard = false;
            if n.failsafe {
                n.phase = Phase::Recovery;
            } else {
                n.phase = Phase::Ready;
                n.failsafe = false;
            }
        }
        Event::EnterOffboard => {
            if n.phase != Phase::Armed
                && n.phase != Phase::Takeoff
                && n.phase != Phase::Airborne
                && n.phase != Phase::Landing
            {
                return Err(Reject::IllegalPhase);
            }
            if !n.armed {
                return Err(Reject::NotArmed);
            }
            if !n.offboard_heartbeat_fresh {
                return Err(Reject::HeartbeatStale);
            }
            n.offboard = true;
        }
        Event::HeartbeatFresh => {
            n.offboard_heartbeat_fresh = true;
        }
        Event::HeartbeatStale => {
            n.offboard_heartbeat_fresh = false;
            if n.offboard {
                n = enter_failsafe(n);
            }
        }
        Event::EnableActuators => {
            if !n.armed {
                return Err(Reject::NotArmed);
            }
            n.actuators_enabled = true;
        }
        Event::DisableActuators => {
            if n.phase == Phase::Takeoff || n.phase == Phase::Airborne || n.phase == Phase::Landing
            {
                return Err(Reject::IllegalPhase);
            }
            n.actuators_enabled = false;
        }
        Event::Takeoff => {
            if n.phase != Phase::Armed {
                return Err(Reject::IllegalPhase);
            }
            if !n.armed {
                return Err(Reject::NotArmed);
            }
            n.actuators_enabled = true;
            n.phase = Phase::Takeoff;
        }
        Event::ReachedAltitude => {
            if n.phase != Phase::Takeoff {
                return Err(Reject::IllegalPhase);
            }
            n.phase = Phase::Airborne;
        }
        Event::Land => {
            if n.phase != Phase::Airborne && n.phase != Phase::Takeoff {
                return Err(Reject::IllegalPhase);
            }
            n.phase = Phase::Landing;
        }
        Event::Touchdown => {
            if n.phase != Phase::Landing && n.phase != Phase::Failsafe {
                return Err(Reject::IllegalPhase);
            }
            n.actuators_enabled = false;
            n.armed = false;
            n.offboard = false;
            n.failsafe = false;
            n.phase = Phase::Ready;
        }
        Event::MissionCommand => {
            if n.failsafe {
                return Err(Reject::InFailsafe);
            }
            if !n.armed {
                return Err(Reject::NotArmed);
            }
            if !n.actuators_enabled {
                return Err(Reject::ActuatorsDisabled);
            }
            if n.offboard && !n.offboard_heartbeat_fresh {
                return Err(Reject::HeartbeatStale);
            }
        }
        Event::TriggerFailsafe => {
            n = enter_failsafe(n);
        }
        Event::Recover => {
            if n.phase != Phase::Recovery {
                return Err(Reject::IllegalPhase);
            }
            if n.armed {
                return Err(Reject::NotArmed);
            }
            n.failsafe = false;
            n.phase = Phase::Ready;
        }
    }

    #[cfg(not(creusot))]
    debug_assert!(
        check_invariants(&n),
        "safety invariant broken after {e:?}: {n:?}"
    );
    Ok(n)
}

#[cfg(not(creusot))]
pub fn step_all(mut s: SafetyState, events: &[Event]) -> Result<SafetyState, Reject> {
    for &e in events {
        s = step(s, e)?;
    }
    Ok(s)
}

/// Pack a state into 16 bits for exhaustive enumeration / Kani `any` wrappers.
///
/// Bit layout: phase[3:0] armed actuators imu estimator hb offboard failsafe
#[cfg(not(creusot))]
pub fn pack(s: &SafetyState) -> u16 {
    let mut v = s.phase as u16;
    if s.armed {
        v |= 1 << 4;
    }
    if s.actuators_enabled {
        v |= 1 << 5;
    }
    if s.imu_healthy {
        v |= 1 << 6;
    }
    if s.estimator_valid {
        v |= 1 << 7;
    }
    if s.offboard_heartbeat_fresh {
        v |= 1 << 8;
    }
    if s.offboard {
        v |= 1 << 9;
    }
    if s.failsafe {
        v |= 1 << 10;
    }
    v
}

#[cfg(not(creusot))]
pub fn unpack(v: u16) -> Option<SafetyState> {
    let phase = Phase::from_u8((v & 0xF) as u8)?;
    Some(SafetyState {
        phase,
        armed: v & (1 << 4) != 0,
        actuators_enabled: v & (1 << 5) != 0,
        imu_healthy: v & (1 << 6) != 0,
        estimator_valid: v & (1 << 7) != 0,
        offboard_heartbeat_fresh: v & (1 << 8) != 0,
        offboard: v & (1 << 9) != 0,
        failsafe: v & (1 << 10) != 0,
    })
}

#[cfg(creusot)]
use creusot_contracts::predicate;

#[cfg(creusot)]
#[predicate]
fn failsafe_latched(s: SafetyState) -> bool {
    (s.failsafe && s.phase == Phase::Failsafe && s.offboard == false)
}

#[cfg(creusot)]
#[predicate]
fn inv_aerial(s: SafetyState) -> bool {
    (s.actuators_enabled && s.armed == false) == false
        && (s.offboard && s.armed == false) == false
        && (s.phase == Phase::Failsafe && s.failsafe == false) == false
        && (s.failsafe && (s.phase == Phase::Failsafe || s.phase == Phase::Recovery) == false)
            == false
        && (s.phase == Phase::Disconnected
            && (s.armed || s.actuators_enabled || s.failsafe || s.offboard))
            == false
        && ((s.phase == Phase::Armed
            || s.phase == Phase::Takeoff
            || s.phase == Phase::Airborne
            || s.phase == Phase::Landing)
            && s.armed == false)
            == false
        && ((s.phase == Phase::Takeoff || s.phase == Phase::Airborne || s.phase == Phase::Landing)
            && s.actuators_enabled == false)
            == false
        && (s.armed
            && s.failsafe == false
            && (s.imu_healthy == false || s.estimator_valid == false))
            == false
}

#[cfg(creusot)]
#[predicate]
fn step_ok(n: SafetyState) -> bool {
    inv_aerial(n) && (n.actuators_enabled == false || n.armed)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn happy_path() -> SafetyState {
        step_all(
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
                Event::Takeoff,
                Event::ReachedAltitude,
            ],
        )
        .unwrap()
    }

    #[test]
    fn connect_preflight_arm_takeoff() {
        let s = happy_path();
        assert_eq!(s.phase, Phase::Airborne);
        assert!(s.armed && s.actuators_enabled && s.offboard);
        assert!(check_invariants(&s));
    }

    #[test]
    fn cannot_arm_without_imu() {
        let s = step_all(
            SafetyState::disconnected(),
            &[
                Event::Connect,
                Event::InitComplete,
                Event::Initialized,
                Event::EstimatorValid,
                Event::PreflightPassed,
            ],
        );
        assert_eq!(s, Err(Reject::ImuUnhealthy));
    }

    #[test]
    fn cannot_enable_actuators_disarmed() {
        let s = step_all(
            SafetyState::disconnected(),
            &[
                Event::Connect,
                Event::InitComplete,
                Event::Initialized,
                Event::ImuHealthy,
                Event::EstimatorValid,
                Event::PreflightPassed,
            ],
        )
        .unwrap();
        assert_eq!(step(s, Event::EnableActuators), Err(Reject::NotArmed));
    }

    #[test]
    fn failsafe_rejects_mission_commands() {
        let s = happy_path();
        let s = step(s, Event::TriggerFailsafe).unwrap();
        assert_eq!(s.phase, Phase::Failsafe);
        assert_eq!(step(s, Event::MissionCommand), Err(Reject::InFailsafe));
        assert_eq!(step(s, Event::Takeoff), Err(Reject::InFailsafe));
    }

    #[test]
    fn imu_loss_while_armed_enters_failsafe() {
        let s = happy_path();
        let s = step(s, Event::ImuUnhealthy).unwrap();
        assert!(s.failsafe);
        assert_eq!(s.phase, Phase::Failsafe);
        assert!(check_invariants(&s));
        // still armed for emergency descent, but mission commands are dead
        assert!(s.armed);
        assert_eq!(step(s, Event::MissionCommand), Err(Reject::InFailsafe));
    }

    #[test]
    fn disarm_in_failsafe_clears_actuators() {
        let s = happy_path();
        let s = step(s, Event::TriggerFailsafe).unwrap();
        let s = step(s, Event::Disarm).unwrap();
        assert!(!s.armed);
        assert!(!s.actuators_enabled);
        assert_eq!(s.phase, Phase::Recovery);
        assert!(check_invariants(&s));
        let s = step(s, Event::Recover).unwrap();
        assert_eq!(s.phase, Phase::Ready);
        assert!(!s.failsafe);
    }

    #[test]
    fn inductive_invariant_on_all_packed_states() {
        let mut preserved = 0u32;
        let mut rejected = 0u32;
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else { continue };
            if !check_invariants(&s) {
                continue;
            }
            for e in Event::ALL {
                match step(s, e) {
                    Ok(n) => {
                        assert!(
                            check_invariants(&n),
                            "invariant broken: {s:?} --{e:?}--> {n:?}"
                        );
                        assert!(
                            !n.actuators_enabled || n.armed,
                            "actuators enabled while disarmed: {s:?} --{e:?}--> {n:?}"
                        );
                        preserved += 1;
                    }
                    Err(_) => rejected += 1,
                }
            }
        }
        assert!(preserved > 100);
        assert!(rejected > 100);
    }

    #[test]
    fn no_reachable_actuators_while_disarmed() {
        let mut queue = [SafetyState::disconnected(); 2048];
        let mut head = 0usize;
        let mut tail = 1usize;
        let mut seen = [false; 2048];
        seen[pack(&SafetyState::disconnected()) as usize] = true;
        let mut visited = 0usize;
        while head < tail {
            let s = queue[head];
            head += 1;
            visited += 1;
            assert!(check_invariants(&s));
            assert!(!s.actuators_enabled || s.armed);
            for e in Event::ALL {
                if let Ok(n) = step(s, e) {
                    let i = pack(&n) as usize;
                    if !seen[i] {
                        seen[i] = true;
                        queue[tail] = n;
                        tail += 1;
                    }
                }
            }
        }
        assert!(visited > 10);
    }

    #[test]
    fn land_only_from_flight_touchdown_returns_ready() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::Land) {
                Ok(n) => {
                    assert!(!s.failsafe, "{s:?}");
                    assert!(matches!(s.phase, Phase::Takeoff | Phase::Airborne), "{s:?}");
                    assert_eq!(n.phase, Phase::Landing);
                    assert!(n.armed && n.actuators_enabled);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(
                        s.failsafe || !matches!(s.phase, Phase::Takeoff | Phase::Airborne),
                        "{s:?}"
                    );
                }
            }
            match step(s, Event::Touchdown) {
                Ok(n) => {
                    assert!(matches!(s.phase, Phase::Landing | Phase::Failsafe), "{s:?}");
                    assert_eq!(n.phase, Phase::Ready);
                    assert!(!n.armed && !n.actuators_enabled && !n.offboard && !n.failsafe);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(
                        !matches!(s.phase, Phase::Landing | Phase::Failsafe),
                        "{s:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn takeoff_only_from_armed_reached_altitude_only_from_takeoff() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::Takeoff) {
                Ok(n) => {
                    assert!(!s.failsafe, "{s:?}");
                    assert_eq!(s.phase, Phase::Armed, "{s:?}");
                    assert!(s.armed, "{s:?}");
                    assert_eq!(n.phase, Phase::Takeoff);
                    assert!(n.armed && n.actuators_enabled);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(s.failsafe || s.phase != Phase::Armed || !s.armed, "{s:?}");
                }
            }
            match step(s, Event::ReachedAltitude) {
                Ok(n) => {
                    assert_eq!(s.phase, Phase::Takeoff, "{s:?}");
                    assert_eq!(n.phase, Phase::Airborne);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert_ne!(s.phase, Phase::Takeoff, "{s:?}");
                }
            }
        }
    }

    #[test]
    fn failsafe_always_returns_failsafe() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            let n = step(s, Event::TriggerFailsafe).unwrap();
            assert_eq!(n.phase, Phase::Failsafe);
            assert!(n.failsafe && !n.offboard);
            assert!(check_invariants(&n));
        }
    }

    #[test]
    fn recover_only_from_recovery_returns_ready() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::Recover) {
                Ok(n) => {
                    assert_eq!(s.phase, Phase::Recovery, "{s:?}");
                    assert!(!s.armed, "{s:?}");
                    assert_eq!(n.phase, Phase::Ready);
                    assert!(!n.failsafe);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(s.phase != Phase::Recovery || s.armed, "{s:?}");
                }
            }
        }
    }

    #[test]
    fn enter_offboard_only_from_flight_phases() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::EnterOffboard) {
                Ok(n) => {
                    assert!(!s.failsafe, "{s:?}");
                    assert!(
                        matches!(
                            s.phase,
                            Phase::Armed | Phase::Takeoff | Phase::Airborne | Phase::Landing
                        ),
                        "{s:?}"
                    );
                    assert!(s.armed && s.offboard_heartbeat_fresh, "{s:?}");
                    assert!(n.offboard);
                    assert_eq!(n.phase, s.phase);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(
                        s.failsafe
                            || !matches!(
                                s.phase,
                                Phase::Armed | Phase::Takeoff | Phase::Airborne | Phase::Landing
                            )
                            || !s.armed
                            || !s.offboard_heartbeat_fresh,
                        "{s:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn disarm_unarms_and_returns_ready_or_recovery() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::Disarm) {
                Ok(n) => {
                    assert_ne!(s.phase, Phase::Disconnected, "{s:?}");
                    assert!(!n.armed && !n.actuators_enabled && !n.offboard);
                    if s.failsafe {
                        assert_eq!(n.phase, Phase::Recovery);
                    } else {
                        assert_eq!(n.phase, Phase::Ready);
                        assert!(!n.failsafe);
                    }
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert_eq!(s.phase, Phase::Disconnected, "{s:?}");
                }
            }
        }
    }

    #[test]
    fn mission_command_requires_armed_actuators() {
        for bits in 0u16..=0x07FF {
            let Some(s) = unpack(bits) else {
                continue;
            };
            if !check_invariants(&s) {
                continue;
            }
            match step(s, Event::MissionCommand) {
                Ok(n) => {
                    assert!(!s.failsafe && s.armed && s.actuators_enabled, "{s:?}");
                    assert!(!s.offboard || s.offboard_heartbeat_fresh, "{s:?}");
                    assert_eq!(n, s);
                    assert!(check_invariants(&n));
                }
                Err(_) => {
                    assert!(
                        s.failsafe
                            || !s.armed
                            || !s.actuators_enabled
                            || (s.offboard && !s.offboard_heartbeat_fresh),
                        "{s:?}"
                    );
                }
            }
        }
    }
}
