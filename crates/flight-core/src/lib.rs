//! Strongly typed core for autonomous vehicle control.
//!
//! This crate is the high-assurance Rust **control boundary** between
//! autonomous software and a physical vehicle: typestate evidence,
//! revocable [`crate::contracts::ActuationPermit`]s, typed units/frames, a
//! pure `no_std` safety kernel, and generated contract tables. PX4, ROS,
//! Copper, and planners run the vehicle. flight-core makes physically
//! invalid authority difficult to express and possible to revoke.
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
//! Default features include `std` (typestate `Vehicle` / `GroundVehicle` /
//! `MarineVehicle` and backends). `--no-default-features` builds units, frames,
//! sensors, safety, hydro, and mech on `no_std`. There is no `no_std` vehicle
//! handle: a microcontroller companion uses the kernel + attitude estimator,
//! not `Vehicle<S, B>`. Do not claim `no_std` vehicles. [`host::kernel_host_tick`]
//! (NEXT B8, `examples/kernel_tick`) walks `step` / `ground_step` /
//! `marine_step` / `deadline_outcome` on the host.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

/// Creusot 0.5 ICEs on `dyn fmt::Write` (Debug/Display bodies). Isolate the four
/// discrete machines so `cargo creusot` does not translate the rest of the crate.
#[cfg(not(creusot))]
pub mod contracts;
#[cfg(not(creusot))]
pub mod domain;
#[cfg(not(creusot))]
pub mod frames;
#[cfg(not(creusot))]
pub mod geometry;
pub mod ground;
pub mod hitl;
#[cfg(not(creusot))]
pub mod host;
#[cfg(not(creusot))]
pub mod hydro;
pub mod marine;
#[cfg(not(creusot))]
mod math;
#[cfg(not(creusot))]
pub mod mech;
#[cfg(not(creusot))]
pub mod nav;
#[cfg(not(creusot))]
pub mod plan;
pub mod safety;
#[cfg(not(creusot))]
pub mod sensors;
#[cfg(not(creusot))]
pub mod temporal;
#[cfg(not(creusot))]
pub mod time;
#[cfg(not(creusot))]
pub mod units;
#[cfg(not(creusot))]
pub mod vector;

/// Consume-self typestate. Creusot 0.5 does not translate `async` methods, so this
/// module is rustc-only. Kernel machines in `safety` / `ground` / `marine` / `hitl`
/// are what `cargo creusot` discharges.
#[cfg(all(feature = "std", not(creusot)))]
pub mod vehicle;

/// Common types for vehicle applications.
pub mod prelude {
    #[cfg(feature = "std")]
    pub use crate::contracts::parse_trace_jsonl;
    #[cfg(not(creusot))]
    pub use crate::contracts::{
        evaluate_trace, ActuationPermit, AerialOffboard, AuthorityReject, Requirement, SafetyEpoch,
        TraceSample, VehicleId,
    };
    #[cfg(not(creusot))]
    pub use crate::domain::{Domain, Medium};
    #[cfg(not(creusot))]
    pub use crate::frames::{Body, Enu, Frame, Frd, Ned};
    #[cfg(not(creusot))]
    pub use crate::geometry::{Covariance, Displacement, Orientation, Point3, Rotation, Transform};
    pub use crate::ground::{ground_step, GroundEvent, GroundPhase, GroundReject, GroundState};
    #[cfg(not(creusot))]
    pub use crate::hitl::{command_after_deadline, hitl_invariants};
    pub use crate::hitl::{deadline_outcome, hitl_apply_allowed, DeadlineOutcome, DeadlineSpec};
    #[cfg(not(creusot))]
    pub use crate::hydro::{
        hydro_invariants, hydro_volume, hydro_volume_conserved, rusanov_flux,
        two_cell_periodic_mass, HydroGrid, HydroInvariants, HydroSample, HydroState, HYDRO_H_DRY,
    };
    pub use crate::marine::{marine_step, MarineEvent, MarinePhase, MarineReject, MarineState};
    #[cfg(not(creusot))]
    pub use crate::mech::{
        aerial_thrust_only_in_air, angular_kinetic_energy, apply_sphere_friction,
        battery_gates_thrust, body_axis_wrench, body_wrench_axes_limited, body_z_thrust_ned,
        buoyancy_ned, buoyancy_only_when_wet, contact_invariants, drag_opposes_flow,
        drain_from_thrust, euler_principal_step, friction_invariants, gravitational_pe_ned,
        ground_thrust_only_on_contact, hold_restores_pose, hold_velocity_ned, kinetic_energy,
        marine_thrust_only_when_wet, mechanical_energy, mechanics_finite, quadratic_drag,
        quat_integrate, quat_is_unit, quat_rotate, quat_rotate_inv, relative_power,
        resolve_sphere_contact, resolve_vertical_contact, rigid_spin_invariants,
        rotation_preserves_length, sphere_contact_invariants, thrust_along_minus_body_z,
        thrust_only_when_granted, vec_cross, vec_dot, SphereBody, SphereContact, SphereFriction,
        SphereSpin, VerticalContact, HOLD_KP, SPHERE_FRICTION_MU,
    };
    #[cfg(not(creusot))]
    pub use crate::nav::{imu_trips_estimator, ComplementaryAttitude};
    #[cfg(not(creusot))]
    pub use crate::plan::{NedPath, Waypoint};
    #[cfg(not(creusot))]
    pub use crate::safety::ContractEdge;
    pub use crate::safety::{
        admit_offboard_command, command_age_ok, estimator_ts_monotonic, event_revokes_authority,
        heartbeat_age_ok, step, Event, Phase, Reject, SafetyState, COMMAND_MAX_AGE_MS,
        OFFBOARD_HEARTBEAT_MAX_AGE_MS,
    };
    #[cfg(not(creusot))]
    pub use crate::sensors::{ActuatorCommand, Actuators, Imu, ImuSample, SensorHealth};
    #[cfg(not(creusot))]
    pub use crate::temporal::{
        estimate_revoke_event, Command, CommandFresh, Deadline, Estimate, EstimateFresh, Fresh,
        HeartbeatFresh, Lease, Observation, Rate, Sequence, Timestamp, ESTIMATE_MAX_AGE_MS,
    };
    #[cfg(not(creusot))]
    pub use crate::time::{Clock, Duration, MonotonicInstant, VirtualClock};
    #[cfg(not(creusot))]
    pub use crate::units::{DegreePerSecond, Meter, MeterPerSecond, Qty, RadianPerSecond, Unit};
    #[cfg(not(creusot))]
    pub use crate::vector::{
        Acceleration, AngularVelocity, Force, Position, Torque, Vector3, Velocity,
    };
    #[cfg(all(feature = "std", not(creusot)))]
    pub use crate::vehicle::{
        aerial_kind, ground_kind, marine_kind, AerialKind, Airborne, Armed, Disarmed, Disconnected,
        Docked, EStopped, Failsafe, GroundHandle, GroundKind, GroundVehicle, Landing,
        MarineFailsafe, MarineHandle, MarineKind, MarineVehicle, Moving, Offboard, Parked,
        PreflightReady, Recovery, StationKeep, Takeoff, Underway, Vehicle, VehicleHandle,
    };
}
