//! Capability and contract surface for physical autonomy.
//!
//! Types here are **evidence and revocable authority**, not permanent truths
//! about the world. The trusted computing base that decides whether a command
//! may become force is [`crate::safety::step`] plus
//! [`crate::safety::event_revokes_authority`]. Everything else — PX4, ROS,
//! planners, this typestate API — is untrusted relative to that kernel.

mod monitor;
mod permit;
mod spec;

#[cfg(feature = "std")]
pub use monitor::parse_trace_jsonl;
pub use monitor::{evaluate_trace, MonitorFail, Requirement, TraceSample};
pub use permit::{ActuationPermit, AuthorityReject, SafetyEpoch, VehicleId};
pub use spec::{
    human_readable_spec, AerialOffboard, LeftoverContract, INV_ACTUATORS_IMPLY_ARMED,
    INV_COMMAND_AGE, INV_ESTIMATOR_TS, INV_OFFBOARD_HEARTBEAT, INV_PERMIT_EPOCH,
};
