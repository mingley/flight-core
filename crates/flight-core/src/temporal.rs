//! Temporal contracts: freshness, sequence, estimates, observations.
//!
//! A value that was valid when an operation *started* is not proof it is
//! valid at actuation time. These types make age, order, and validity
//! first-class so the kernel and monitors can reject stale evidence.

use crate::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS;
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

    pub fn get(&self, now: MonotonicInstant) -> Result<&T, FreshnessError> {
        let age_ms = now.saturating_duration_since(self.stamped_at).as_nanos() / 1_000_000;
        let age_ms = if age_ms > u32::MAX as u64 {
            u32::MAX
        } else {
            age_ms as u32
        };
        if age_ms >= MAX_AGE_MS {
            return Err(FreshnessError::Stale {
                age_ms,
                max_age_ms: MAX_AGE_MS,
            });
        }
        Ok(&self.value)
    }
}

/// Heartbeat evidence that must be younger than the offboard contract.
pub type HeartbeatFresh = Fresh<(), { OFFBOARD_HEARTBEAT_MAX_AGE_MS }>;

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

    pub const fn period_ns(self) -> u64 {
        if self.hz == 0 {
            return u64::MAX;
        }
        1_000_000_000 / (self.hz as u64)
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
        let ts = Timestamp::from_millis(5);
        assert!(ts.precedes(Timestamp::from_millis(5)));
        assert!(ts.precedes(Timestamp::from_millis(6)));
        assert!(!Timestamp::from_millis(6).precedes(ts));
        assert_eq!(ts.age_ms(MonotonicInstant::from_millis(9)), 4);
    }
}
