//! Monotonic timebase shared by production, simulation, replay, and proofs.

use core::fmt;

/// Nanoseconds since an arbitrary epoch (boot, recording start, or symbolic zero).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct MonotonicInstant {
    nanos: u64,
}

impl MonotonicInstant {
    pub const ZERO: Self = Self { nanos: 0 };

    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    pub const fn from_micros(micros: u64) -> Self {
        Self {
            nanos: micros.saturating_mul(1_000),
        }
    }

    pub const fn from_millis(millis: u64) -> Self {
        Self {
            nanos: millis.saturating_mul(1_000_000),
        }
    }

    pub const fn as_nanos(self) -> u64 {
        self.nanos
    }

    pub fn as_secs_f32(self) -> f32 {
        self.nanos as f32 / 1_000_000_000.0
    }

    pub const fn saturating_add(self, dt: Duration) -> Self {
        Self {
            nanos: self.nanos.saturating_add(dt.nanos),
        }
    }

    pub fn saturating_duration_since(self, earlier: Self) -> Duration {
        Duration {
            nanos: self.nanos.saturating_sub(earlier.nanos),
        }
    }
}

#[cfg(not(creusot))]
impl fmt::Display for MonotonicInstant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.6}s", self.as_secs_f32())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Duration {
    nanos: u64,
}

impl Duration {
    pub const ZERO: Self = Self { nanos: 0 };

    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    pub const fn from_millis(millis: u64) -> Self {
        Self {
            nanos: millis.saturating_mul(1_000_000),
        }
    }

    pub fn from_secs_f32(secs: f32) -> Self {
        if !secs.is_finite() || secs <= 0.0 {
            return Self::ZERO;
        }
        Self {
            nanos: (secs * 1_000_000_000.0) as u64,
        }
    }

    pub const fn as_nanos(self) -> u64 {
        self.nanos
    }

    pub fn as_secs_f32(self) -> f32 {
        self.nanos as f32 / 1_000_000_000.0
    }
}

/// Clock the controller samples. Production, replay, sim, and Kani all implement this.
pub trait Clock {
    fn now(&self) -> MonotonicInstant;
}

/// Deterministic clock that only advances when asked. Used by sim, replay, and proofs.
#[derive(Clone, Debug)]
pub struct VirtualClock {
    now: MonotonicInstant,
}

impl VirtualClock {
    pub const fn new() -> Self {
        Self {
            now: MonotonicInstant::ZERO,
        }
    }

    pub fn advance(&mut self, dt: Duration) {
        self.now = self.now.saturating_add(dt);
    }

    pub fn set(&mut self, now: MonotonicInstant) {
        self.now = now;
    }
}

impl Default for VirtualClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for VirtualClock {
    fn now(&self) -> MonotonicInstant {
        self.now
    }
}

#[cfg(feature = "std")]
impl Clock for std::time::Instant {
    fn now(&self) -> MonotonicInstant {
        // Treat the Instant's elapsed-from-itself as zero; not generally useful.
        // Prefer wrapping a start Instant in WallClock.
        MonotonicInstant::ZERO
    }
}

/// `std::time::Instant` measured from a captured origin.
#[cfg(feature = "std")]
#[derive(Clone, Debug)]
pub struct WallClock {
    origin: std::time::Instant,
}

#[cfg(feature = "std")]
impl WallClock {
    pub fn new() -> Self {
        Self {
            origin: std::time::Instant::now(),
        }
    }
}

#[cfg(feature = "std")]
impl Default for WallClock {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "std")]
impl Clock for WallClock {
    fn now(&self) -> MonotonicInstant {
        MonotonicInstant::from_nanos(self.origin.elapsed().as_nanos() as u64)
    }
}
