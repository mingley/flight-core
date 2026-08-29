//! Observe a verified world, act through typed safety machines, research with
//! a closed-loop agent that returns a property certificate.
//!
//! The snapshot is JSON **and** typed: [`Lab::observe`] for agents, [`LabCmd`] for act / research (the demo posts snake_case JSON and both the live loop and [`Lab::research`] apply it through [`Lab::act_through_attach`]), [`Lab::aerial`] / [`Lab::ground`] / [`Lab::marine`] for `telemetry_now` without stepping.
//! Each robot view names mechanical support (`terrain` / `water` / `air`),
//! `terrain_contact`, the pairwise `sphere_hits` graph (who touched whom,
//! not only a per-body HIT flag), `legal_cmds` — the [`LabCmd`] values
//! the live safety machine would accept — and the live NED `hold_ned` when a
//! pose hold is tracking, so agents see pad, water, collisions, legal acts,
//! and the hold target, not just land-cell drag fluid.
//! After every `step`, mechanical properties are re-checked. `Lab::research`
//! returns that vector on [`ResearchRun`]. Research traces are JSONL or
//! Foxglove-compatible MCAP. [`Observation::tools`] / [`LegalTools`] list the
//! only `(robot, cmd)` pairs plus `env_cmds` an agent may call; [`Lab::act`]
//! and [`Lab::act_through_attach`] reject anything else as [`LabError::NotLegal`]
//! or [`LabError::UnknownRobot`] before domain attach.
//!
//! Observe / act / research share the same [`WorldSession`] plant as the
//! typestate fleet APIs. [`Lab::world`] is a snapshot; [`Lab::session`] is live.
//! [`Lab::aerial_vehicle`] / [`Lab::ground_vehicle`] / [`Lab::marine_vehicle`]
//! attach consume-self typestate to the live machine without resetting it.
//! [`Lab::attach_takeoff`] / [`Lab::attach_start_takeoff`] / [`Lab::attach_drive`] / [`Lab::attach_undock`]
//! / [`Lab::attach_land`] / [`Lab::attach_touchdown`] / [`Lab::attach_airborne`] /
//! [`Lab::attach_hold`] / [`Lab::attach_failsafe`] / [`Lab::attach_estop`] / [`Lab::attach_reset`] /
//! [`Lab::attach_marine_failsafe`] / [`Lab::attach_recover`] / [`Lab::attach_recover_ready`] /
//! [`Lab::attach_station`] / [`Lab::attach_resume`] walk those machines and
//! return the live backend. [`Lab::apply_script`] is the same path (attach
//! helpers and NED now-APIs, then one shared step) — not kernel events on a
//! borrowed body. Velocity ticks are not logged. Position holds walk
//! `set_position_now` (P-term at flush, never a raw NED velocity). Current-pose
//! holds walk [`Lab::attach_hold`] (`LabCmd::Hold`).
//! [`Lab::replay_until`] walks
//! the same attach helpers without re-logging; Protocol falls back to JSON.

#![deny(unsafe_code)]

pub mod bag;
pub mod probe;
pub mod research;

mod apply;
mod cmd;
mod lab;
mod observe;
mod script;

pub use bag::{action_json, looks_like_mcap, observation_json, schema_json, McapBag};
pub use cmd::LabCmd;
pub use flight_core::vehicle::{
    AerialKind, GroundHandle, GroundKind, MarineHandle, MarineKind, VehicleHandle,
};
pub use flight_sim::{GroundWorldBackend, MarineWorldBackend, WorldBackend, WorldSession};
pub use lab::{AgentAction, Lab, LabError, TimedAction};
pub use observe::{
    AerialMachine, EnvView, GroundMachine, LegalTools, MarineMachine, Observation, RobotTool,
    RobotView,
};
pub use probe::ProbeReport;
pub use research::{
    CoastalFleet, CollisionSweep, PadLanding, ResearchAgent, ResearchRun, RoverProbe,
    ScriptedCoastal, TypedAerialAirborne, TypedAerialDisarm, TypedAerialFailsafe, TypedAttachFleet,
    TypedCollisionSweep, TypedFailsafeTouchdown, TypedFleet, TypedFleetHold, TypedFleetReturn,
    TypedGroundEstop, TypedGroundHalt, TypedHold, TypedHullDock, TypedHullFailsafe, TypedPadDisarm,
    TypedPadFailsafe, TypedPadLanding, TypedPositionHold, TypedStationDock, TypedStationFailsafe,
    TypedStationResume, TypedSurveyorDock, TypedSurveyorFailsafe, TypedSurveyorStationDock,
    TypedSurveyorStationFailsafe, TypedSurveyorStationResume,
};

#[cfg(test)]
mod tests;
