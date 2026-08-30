//! Temporal contracts: freshness, sequence, estimates, observations.
//!
//! A value that was valid when an operation *started* is not proof it is
//! valid at actuation time. These types make age, order, and validity
//! first-class so the kernel and monitors can reject stale evidence.

use crate::safety::{COMMAND_MAX_AGE_MS, OFFBOARD_HEARTBEAT_MAX_AGE_MS};
use crate::time::{Duration, MonotonicInstant};
use core::fmt;
use core::marker::PhantomData;

/// Explicit timestamp. Distinct from a raw integer so age and order are typed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Timestamp {
    inner: MonotonicInstant,
}

impl Timestamp {
    pub const ZERO: Self = Self {
        inner: MonotonicInstant::ZERO,
    };

    pub const fn from_instant(now: MonotonicInstant) -> Self {
        Self { inner: now }
    }

    pub const fn from_millis(ms: u64) -> Self {
        Self {
            inner: MonotonicInstant::from_millis(ms),
        }
    }

    pub const fn from_micros(us: u64) -> Self {
        Self {
            inner: MonotonicInstant::from_micros(us),
        }
    }

    pub const fn instant(self) -> MonotonicInstant {
        self.inner
    }

    pub const fn as_nanos(self) -> u64 {
        self.inner.as_nanos()
    }

    pub const fn as_millis(self) -> u64 {
        self.inner.as_nanos() / 1_000_000
    }

    pub fn age_ms(self, now: MonotonicInstant) -> u32 {
        let ms = now.saturating_duration_since(self.inner).as_nanos() / 1_000_000;
        if ms > u32::MAX as u64 {
            u32::MAX
        } else {
            ms as u32
        }
    }

    /// `true` when `self` is not strictly after `later` (monotonic, equal allowed).
    pub const fn precedes(self, later: Self) -> bool {
        self.inner.as_nanos() <= later.inner.as_nanos()
    }
}

/// A value that is only readable while younger than `MAX_AGE_MS`.
#[derive(Clone, Copy, Debug)]
pub struct Fresh<T, const MAX_AGE_MS: u32> {
    value: T,
    stamped_at: MonotonicInstant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FreshnessError {
    Stale { age_ms: u32, max_age_ms: u32 },
}

impl fmt::Display for FreshnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Stale { age_ms, max_age_ms } => {
                write!(f, "stale: {age_ms} ms > {max_age_ms} ms")
            }
        }
    }
}

impl<T, const MAX_AGE_MS: u32> Fresh<T, MAX_AGE_MS> {
    pub const fn new(value: T, stamped_at: MonotonicInstant) -> Self {
        Self { value, stamped_at }
    }

    pub const fn stamped_at(&self) -> MonotonicInstant {
        self.stamped_at
    }

    pub fn age(self, now: MonotonicInstant) -> Duration
    where
        T: Copy,
    {
        now.saturating_duration_since(self.stamped_at)
    }

    /// Age-only check used when a companion reports milliseconds, not a stamp.
    /// Same bound as [`Self::get`]: `age_ms < MAX_AGE_MS`.
    pub const fn check_age(age_ms: u32) -> Result<(), FreshnessError> {
        if age_ms >= MAX_AGE_MS {
            Err(FreshnessError::Stale {
                age_ms,
                max_age_ms: MAX_AGE_MS,
            })
        } else {
            Ok(())
        }
    }

    pub fn get(&self, now: MonotonicInstant) -> Result<&T, FreshnessError> {
        let age_ms = now.saturating_duration_since(self.stamped_at).as_nanos() / 1_000_000;
        let age_ms = if age_ms > u32::MAX as u64 {
            u32::MAX
        } else {
            age_ms as u32
        };
        Self::check_age(age_ms)?;
        Ok(&self.value)
    }
}

/// Heartbeat evidence that must be younger than the offboard contract.
pub type HeartbeatFresh = Fresh<(), { OFFBOARD_HEARTBEAT_MAX_AGE_MS }>;

/// Planner command younger than [`COMMAND_MAX_AGE_MS`].
pub type CommandFresh<T> = Fresh<T, { COMMAND_MAX_AGE_MS }>;

/// Kernel event when heartbeat age is outside the offboard bound.
pub const fn heartbeat_revoke_event(age_ms: u32) -> Option<crate::safety::Event> {
    if HeartbeatFresh::check_age(age_ms).is_ok() {
        None
    } else {
        Some(crate::safety::Event::HeartbeatStale)
    }
}

/// Monotonic sequence numbers. A jump backward is a replay or clock fault.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sequence {
    last: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SequenceError {
    Backward { last: u32, observed: u32 },
}

impl Sequence {
    pub const fn new() -> Self {
        Self { last: None }
    }

    pub fn observe(&mut self, seq: u32) -> Result<(), SequenceError> {
        if let Some(last) = self.last {
            if seq < last {
                return Err(SequenceError::Backward {
                    last,
                    observed: seq,
                });
            }
        }
        self.last = Some(seq);
        Ok(())
    }

    pub const fn last(self) -> Option<u32> {
        self.last
    }
}

/// An observation taken in a named frame at a monotonic time.
#[derive(Clone, Copy, Debug)]
pub struct Observation<T, F> {
    pub value: T,
    pub stamped_at: MonotonicInstant,
    _frame: PhantomData<F>,
}

impl<T, F> Observation<T, F> {
    pub const fn new(value: T, stamped_at: MonotonicInstant) -> Self {
        Self {
            value,
            stamped_at,
            _frame: PhantomData,
        }
    }
}

/// Estimator output with an explicit validity bit. Validity is not implied by
/// the type of `T`.
#[derive(Clone, Copy, Debug)]
pub struct Estimate<T> {
    pub value: T,
    pub valid: bool,
    pub stamped_at: MonotonicInstant,
}

impl<T> Estimate<T> {
    pub const fn new(value: T, valid: bool, stamped_at: MonotonicInstant) -> Self {
        Self {
            value,
            valid,
            stamped_at,
        }
    }

    pub fn validated(&self) -> Option<&T> {
        if self.valid {
            Some(&self.value)
        } else {
            None
        }
    }

    /// Kernel event when this estimate is not usable as actuation evidence.
    pub const fn revoke_event(&self) -> Option<crate::safety::Event> {
        if self.valid {
            None
        } else {
            Some(crate::safety::Event::EstimatorInvalid)
        }
    }
}

/// Instant by which a loop iteration or command must have been applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deadline {
    due: MonotonicInstant,
}

impl Deadline {
    pub const fn at(due: MonotonicInstant) -> Self {
        Self { due }
    }

    pub const fn due(self) -> MonotonicInstant {
        self.due
    }

    pub fn met(self, now: MonotonicInstant) -> bool {
        now <= self.due
    }

    /// Last instant a planner command issued at `issued_at` may actuate.
    /// Matches [`crate::safety::command_age_ok`]: age `< COMMAND_MAX_AGE_MS`.
    pub const fn for_command(issued_at: MonotonicInstant) -> Self {
        let last_ok_ms = (COMMAND_MAX_AGE_MS as u64).saturating_sub(1);
        Self::at(issued_at.saturating_add(Duration::from_millis(last_ok_ms)))
    }
}

/// Time-bounded grant. [`crate::contracts::ActuationPermit`] is the vehicle-bound
/// form; this type is the clock half used by monitors and HITL.
#[derive(Clone, Copy, Debug)]
pub struct Lease {
    issued_at: MonotonicInstant,
    max_age: Duration,
}

impl Lease {
    pub const fn new(issued_at: MonotonicInstant, max_age: Duration) -> Self {
        Self { issued_at, max_age }
    }

    pub fn live(self, now: MonotonicInstant) -> bool {
        now.saturating_duration_since(self.issued_at) < self.max_age
    }

    pub const fn issued_at(self) -> MonotonicInstant {
        self.issued_at
    }

    pub const fn max_age(self) -> Duration {
        self.max_age
    }
}

/// A command stamped when the planner produced it. Actuation must happen
/// before [`Self::issued_at`] plus the deadline.
#[derive(Clone, Copy, Debug)]
pub struct Command<T> {
    pub payload: T,
    pub issued_at: MonotonicInstant,
}

impl<T> Command<T> {
    pub const fn new(payload: T, issued_at: MonotonicInstant) -> Self {
        Self { payload, issued_at }
    }

    pub fn within(&self, now: MonotonicInstant, max_age: Duration) -> bool {
        now.saturating_duration_since(self.issued_at) < max_age
    }

    pub fn age_ms(&self, now: MonotonicInstant) -> u32 {
        Timestamp::from_instant(self.issued_at).age_ms(now)
    }

    pub fn within_command_bound(&self, now: MonotonicInstant) -> bool {
        crate::safety::command_age_ok(self.age_ms(now))
    }

    /// Same bound as [`Self::within_command_bound`], as a [`FreshnessError`].
    pub fn check_age(&self, now: MonotonicInstant) -> Result<(), FreshnessError> {
        CommandFresh::<()>::check_age(self.age_ms(now))
    }

    /// Actuation deadline generated from the kernel command-age bound.
    pub fn deadline(&self) -> Deadline {
        Deadline::for_command(self.issued_at)
    }

    /// Fail closed: typed deadline **and** kernel `command_age_ok`.
    pub fn within_deadline(&self, now: MonotonicInstant) -> bool {
        self.deadline().met(now) && self.within_command_bound(now)
    }
}

/// Named rate in integer hertz. Used by deadline / loop contracts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    hz: u32,
}

impl Rate {
    pub const HZ_50: Self = Self { hz: 50 };

    pub const fn hz(hz: u32) -> Self {
        Self { hz }
    }

    /// Inverse of [`Self::period_ns`]. Zero period is a zero-Hz rate (never admits).
    pub const fn from_period_ns(period_ns: u64) -> Self {
        if period_ns == 0 {
            return Self { hz: 0 };
        }
        Self {
            hz: (1_000_000_000 / period_ns) as u32,
        }
    }

    pub const fn period_ns(self) -> u64 {
        if self.hz == 0 {
            return u64::MAX;
        }
        1_000_000_000 / (self.hz as u64)
    }

    /// OffboardControl ⇒ this loop rate: compute must finish within one period.
    /// Fail closed with [`crate::hitl::DeadlineSpec`] at the HITL rack.
    pub const fn admits(self, compute_ns: u64) -> bool {
        self.hz != 0 && compute_ns <= self.period_ns()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_rejects_old_heartbeat() {
        let h = HeartbeatFresh::new((), MonotonicInstant::ZERO);
        assert!(h.get(MonotonicInstant::from_millis(249)).is_ok());
        assert!(h.get(MonotonicInstant::from_millis(250)).is_err());
    }

    #[test]
    fn sequence_rejects_backward_jump() {
        let mut s = Sequence::new();
        s.observe(3).unwrap();
        assert_eq!(
            s.observe(2),
            Err(SequenceError::Backward {
                last: 3,
                observed: 2
            })
        );
    }

    #[test]
    fn estimate_validated_is_none_when_invalid() {
        let e = Estimate::new(1u8, false, MonotonicInstant::ZERO);
        assert!(e.validated().is_none());
        assert_eq!(
            e.revoke_event(),
            Some(crate::safety::Event::EstimatorInvalid)
        );
        let ok = Estimate::new(1u8, true, MonotonicInstant::ZERO);
        assert!(ok.revoke_event().is_none());
    }

    #[test]
    fn deadline_and_lease_and_command_age() {
        let due = Deadline::at(MonotonicInstant::from_millis(10));
        assert!(due.met(MonotonicInstant::from_millis(10)));
        assert!(!due.met(MonotonicInstant::from_millis(11)));
        let lease = Lease::new(MonotonicInstant::ZERO, Duration::from_millis(250));
        assert!(lease.live(MonotonicInstant::from_millis(249)));
        assert!(!lease.live(MonotonicInstant::from_millis(250)));
        let cmd = Command::new(1u8, MonotonicInstant::ZERO);
        assert!(cmd.within(MonotonicInstant::from_millis(5), Duration::from_millis(10)));
        assert!(!cmd.within(MonotonicInstant::from_millis(10), Duration::from_millis(10)));
        assert!(cmd.within_command_bound(MonotonicInstant::from_millis(99)));
        assert!(!cmd.within_command_bound(MonotonicInstant::from_millis(100)));
        assert!(cmd.check_age(MonotonicInstant::from_millis(99)).is_ok());
        assert!(cmd.check_age(MonotonicInstant::from_millis(100)).is_err());
        assert!(cmd.deadline().met(MonotonicInstant::from_millis(99)));
        assert!(!cmd.deadline().met(MonotonicInstant::from_millis(100)));
        assert!(cmd.within_deadline(MonotonicInstant::from_millis(99)));
        assert!(!cmd.within_deadline(MonotonicInstant::from_millis(100)));
        for age in 0..=300 {
            assert_eq!(
                HeartbeatFresh::check_age(age).is_ok(),
                crate::safety::heartbeat_age_ok(age),
                "heartbeat age {age}"
            );
            assert_eq!(
                CommandFresh::<()>::check_age(age).is_ok(),
                crate::safety::command_age_ok(age),
                "command age {age}"
            );
            assert_eq!(
                HeartbeatFresh::check_age(age).is_ok()
                    && CommandFresh::<()>::check_age(age).is_ok(),
                crate::safety::admit_offboard_command(age, age),
                "admit age {age}"
            );
            assert_eq!(
                heartbeat_revoke_event(age).is_some(),
                !crate::safety::heartbeat_age_ok(age),
                "heartbeat revoke age {age}"
            );
            let stamped = Command::new((), MonotonicInstant::ZERO);
            let now = MonotonicInstant::from_millis(u64::from(age));
            assert_eq!(
                stamped.within_deadline(now),
                crate::safety::command_age_ok(age),
                "command deadline age {age}"
            );
        }
        let ts = Timestamp::from_millis(5);
        assert!(ts.precedes(Timestamp::from_millis(5)));
        assert!(ts.precedes(Timestamp::from_millis(6)));
        assert!(!Timestamp::from_millis(6).precedes(ts));
        assert_eq!(ts.age_ms(MonotonicInstant::from_millis(9)), 4);
        assert_eq!(
            Rate::HZ_50.period_ns(),
            crate::hitl::DeadlineSpec::HZ_50.period_ns
        );
        assert_eq!(
            Rate::from_period_ns(crate::hitl::DeadlineSpec::HZ_50.period_ns),
            Rate::HZ_50
        );
        assert!(crate::hitl::DeadlineSpec::HZ_50.budget_ns <= Rate::HZ_50.period_ns());
        assert!(Rate::HZ_50.admits(crate::hitl::DeadlineSpec::HZ_50.budget_ns));
        assert!(Rate::HZ_50.admits(Rate::HZ_50.period_ns()));
        assert!(!Rate::HZ_50.admits(Rate::HZ_50.period_ns() + 1));
        assert!(!Rate::hz(0).admits(0));
    }
}
