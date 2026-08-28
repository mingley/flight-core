//! Strongly typed core for autonomous vehicle control.
//!
//! This crate is the API robotics should have if ownership, capabilities, units,
//! reference frames, and legal state transitions were part of the language.
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! fn boom(a: Position<Ned>, b: Position<Enu>) {
//!     let _ = a + b;
//! }
//! ```
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! fn boom(
//!     w: AngularVelocity<DegreePerSecond, Body>,
//! ) -> AngularVelocity<RadianPerSecond, Body> {
//!     w // different unit
//! }
//! ```
//!
//! # no_std
//!
//! Default features include `std` (vehicle API). The units, frames, sensors,
//! safety machine, and attitude estimator build with `--no-default-features`.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

pub mod frames;
mod math;
pub mod nav;
pub mod safety;
pub mod sensors;
pub mod time;
pub mod units;
pub mod vector;

#[cfg(feature = "std")]
pub mod vehicle;

/// Common types for vehicle applications.
pub mod prelude {
    pub use crate::frames::{Body, Enu, Frame, Frd, Ned};
    pub use crate::sensors::{ActuatorCommand, Actuators, Imu, ImuSample, SensorHealth};
    pub use crate::time::{Clock, Duration, MonotonicInstant, VirtualClock};
    pub use crate::units::{DegreePerSecond, Meter, MeterPerSecond, Qty, RadianPerSecond, Unit};
    pub use crate::vector::{Acceleration, AngularVelocity, Position, Vector3, Velocity};
    #[cfg(feature = "std")]
    pub use crate::vehicle::{
        Airborne, Armed, Disarmed, Disconnected, Failsafe, Offboard, PreflightReady, Vehicle,
    };
}
