//! Deterministic simulation, replay, and fuzzed IMU sources.
//!
//! The controller talks to [`Clock`], [`Imu`], and [`Actuators`]. This crate
//! supplies those traits for:
//!
//! - production-shaped physics (virtual clock + simulated IMU + mixer)
//! - recorded IMU streams (jsonl)
//! - fuzzed IMU (seeded noise around a source), including [`WorldImu`] so a
//!   controller can wrap [`FuzzedImu`] around the verified plant without
//!   substituting that plant's `step`

#![deny(unsafe_code)]

pub mod backend;
pub mod fuzz;
pub mod physics;
pub mod replay;
pub mod scenario;
pub mod ulog;
pub mod world_backend;

pub use backend::{SimBackend, SimConfig};
pub use fuzz::FuzzedImu;
pub use physics::{Physics, GRAVITY_NED};
pub use replay::{JsonlReplay, RecordedSample};
pub use scenario::{
    differential_gps_loss, differential_world, replay_jsonl, replay_report, run_hitl_miss,
    run_revoke_table, run_world, Fault, Scenario, ScenarioReport,
};
pub use ulog::{is_ulog, parse_ulog, write_ulog};
pub use world_backend::{
    GroundWorldBackend, MarineWorldBackend, WorldBackend, WorldImu, WorldSession,
};

use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

/// Connect a simulated vehicle. Same API as PX4 SITL, no hardware.
pub async fn connect(
    config: SimConfig,
) -> Result<Vehicle<Disarmed, SimBackend>, flight_core::vehicle::ErrorKind> {
    Vehicle::<Disconnected, SimBackend>::new(SimBackend::new(config))
        .connect()
        .await
        .map_err(|e| e.error)
}

/// Run one control frame against *any* platform. The controller cannot tell
/// whether `imu` is real, recorded, simulated, or fuzzed.
pub fn control_frame<C, I, A>(
    clock: &C,
    imu: &mut I,
    actuators: &mut A,
    command: ActuatorCommand,
) -> Result<(MonotonicInstant, ImuSample<I::Frame>), flight_core::sensors::SensorError>
where
    C: Clock,
    I: Imu,
    A: Actuators,
{
    let now = clock.now();
    let sample = imu.sample()?;
    let _ = actuators.apply(command);
    let _ = now;
    Ok((now, sample))
}
