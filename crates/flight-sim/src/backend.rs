//! Backend trait, `SimulatedBackend`, and `SimClock`.
//!
//! `SimulatedBackend` is the default in-process implementation used by demos
//! and tests. It is a thin wrapper around [`crate::world::World`]: every
//! [`Backend::step`] call advances the world by `dt` and copies the resulting
//! [`crate::world::WorldSnapshot`] into the [`WorldView`] the rest of the
//! crate reads.
//!
//! Hardware backends (PX4, ROS 2, MAVLink) live in sibling crates and implement
//! the same [`Backend`] trait, so swapping a vehicle from simulation to a real
//! airframe is a type-level change, not a rewrite of the control loop.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use flight_core::Attitude;
use flight_core::vehicle::backend::{
    Backend, BackendCapabilities, BackendError, BackendKind, BackendTelemetry,
};
use flight_core::{
    Acceleration, AngularVelocity, Force, Length, LinearVelocity, Mass, Power, Torque,
};

use crate::world::{EntityKind, World};
use crate::world_view::{ContactEvent, EntityPose, ImuSample, WorldView};

/// Wall-clock helper used by the demo loop to convert `dt` into a monotonic
/// tick count. Not used by the physics step itself -- the world is advanced
/// by the `dt` argument of [`Backend::step`], independent of wall time.
#[derive(Debug, Clone)]
pub struct SimClock {
    start: std::time::Instant,
}

impl Default for SimClock {
    fn default() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }
}

impl SimClock {
    pub fn elapsed(&self) -> std::time::Duration {
        self.start.elapsed()
    }

    pub fn ticks(&self, dt: std::time::Duration) -> u64 {
        (self.elapsed().as_secs_f64() / dt.as_secs_f64()) as u64
    }
}

/// In-process simulated backend. Owns a [`World`] and the last
/// [`WorldView`] snapshot produced by [`Backend::step`].
///
/// `view` is `None` until the first `step` call, after which it is always
/// `Some`. [`SimulatedBackend::world_view`] returns a reference to that
/// snapshot so callers (the demo JSON emitter, the robot-lab observe path)
/// can read entity poses without taking a lock.
#[derive(Debug)]
pub struct SimulatedBackend {
    world: World,
    view: Option<WorldView>,
    tick: u64,
    last_step_us: u64,
    last_telemetry: Option<BackendTelemetry>,
    last_error: Option<BackendError>,
    drop_counter: Arc<AtomicU64>,
    last_drop_count: u64,
}

impl SimulatedBackend {
    /// Construct a backend wrapping `world`. The view is empty until the
    /// first [`Backend::step`].
    pub fn new(world: World) -> Self {
        Self {
            world,
            view: None,
            tick: 0,
            last_step_us: 0,
            last_telemetry: None,
            last_error: None,
            drop_counter: Arc::new(AtomicU64::new(0)),
            last_drop_count: 0,
        }
    }

    /// Shared drop counter the demo loop increments when a client is too
    /// slow to consume a frame. Exposed so the JSON emitter can report it.
    pub fn drop_counter(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.drop_counter)
    }

    /// Borrow the last snapshot. Returns `None` before the first step.
    pub fn world_view(&self) -> Option<&WorldView> {
        self.view.as_ref()
    }

    /// Mutable access to the inner world, used by spawn / despawn / attach
    /// helpers that mutate the scene between steps.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// Immutable access to the inner world.
    pub fn world(&self) -> &World {
        &self.world
    }
}

impl Backend for SimulatedBackend {
    fn kind(&self) -> BackendKind {
        BackendKind::Simulated
    }

    fn capabilities(&self) -> BackendCapabilities {
        BackendCapabilities {
            offboard: true,
            gps: true,
            rangefinder: true,
            optical_flow: false,
            rc_override: false,
            actuator_direct: true,
        }
    }

    fn step(&mut self, dt: std::time::Duration) -> Result<(), BackendError> {
        let started = std::time::Instant::now();
        self.world.step(dt);
        self.tick += 1;
        let snapshot = self.world.snapshot();

        let mut entities = HashMap::new();
        for pose in &snapshot.poses {
            entities.insert(
                pose.id.clone(),
                EntityPose {
                    id: pose.id.clone(),
                    kind: match pose.kind {
                        EntityKind::Quadcopter => "quadcopter".into(),
                        EntityKind::FixedWing => "fixed_wing".into(),
                        EntityKind::Vtol => "vtol".into(),
                        EntityKind::Ground => "ground".into(),
                        EntityKind::Marine => "marine".into(),
                        EntityKind::Payload => "payload".into(),
                    },
                    x: pose.x.as_meters(),
                    y: pose.y.as_meters(),
                    z: pose.z.as_meters(),
                    vx: pose.vx.as_meters_per_sec(),
                    vy: pose.vy.as_meters_per_sec(),
                    vz: pose.vz.as_meters_per_sec(),
                    yaw: pose.yaw.as_radians(),
                    armed: pose.armed,
                    battery: pose.battery,
                    mass_kg: Some(pose.mass.as_kilograms()),
                    parent: pose.parent.clone(),
                    joint: pose.joint.clone(),
                    status: pose.status.clone(),
                    health: pose.health,
                    faults: pose.faults.clone(),
                    attached_to: pose.attached_to.clone(),
                    role: pose.role.clone(),
                    team: pose.team.clone(),
                    heading: pose.heading.map(|h| h.as_radians()),
                    mode: pose.mode.clone(),
                },
            );
        }

        let contacts: Vec<ContactEvent> = snapshot
            .contacts
            .iter()
            .map(|c| ContactEvent {
                a: c.a.clone(),
                b: c.b.clone(),
                x: c.x.as_meters(),
                y: c.y.as_meters(),
                z: c.z.as_meters(),
                impulse: c.impulse,
            })
            .collect();

        self.view = Some(WorldView {
            tick: snapshot.tick,
            t: snapshot.t.as_seconds(),
            entities,
            wind: [
                snapshot.wind[0].as_meters_per_sec(),
                snapshot.wind[1].as_meters_per_sec(),
                snapshot.wind[2].as_meters_per_sec(),
            ],
            dropped_frames: 0,
            contacts,
            constraints: snapshot.constraints.clone(),
        });

        self.last_step_us = started.elapsed().as_micros() as u64;
        self.last_drop_count = self.drop_counter.load(Ordering::Relaxed);
        self.last_error = None;
        Ok(())
    }

    fn snapshot(&self) -> Result<BackendTelemetry, BackendError> {
        let view = self.view.as_ref().ok_or_else(|| BackendError::Unavailable {
            detail: "simulated backend has not stepped yet".into(),
        })?;
        let entity = view
            .entities
            .values()
            .find(|e| e.kind == "quadcopter" || e.kind == "fixed_wing" || e.kind == "vtol")
            .or_else(|| view.entities.values().next())
            .ok_or_else(|| BackendError::Unavailable {
                detail: "simulated world has no entities".into(),
            })?;
        Ok(BackendTelemetry {
            position_ned: [
                Length::from_meters(entity.x),
                Length::from_meters(entity.y),
                Length::from_meters(-entity.z),
            ],
            velocity_ned: [
                LinearVelocity::from_meters_per_sec(entity.vx),
                LinearVelocity::from_meters_per_sec(entity.vy),
                LinearVelocity::from_meters_per_sec(-entity.vz),
            ],
            attitude: Attitude::from_euler_ned(0.0, 0.0, entity.yaw),
            angular_velocity: AngularVelocity::ZERO,
            acceleration: Acceleration::ZERO,
            armed: entity.armed,
            battery_remaining: entity.battery,
        })
    }

    fn last_telemetry(&self) -> Option<BackendTelemetry> {
        self.last_telemetry.clone()
    }

    fn send_force_torque(
        &mut self,
        force: Force,
        _torque: Torque,
    ) -> Result<(), BackendError> {
        let _ = force;
        Ok(())
    }

    fn set_mass(&mut self, mass: Mass) -> Result<(), BackendError> {
        let _ = mass;
        Ok(())
    }

    fn last_error(&self) -> Option<&BackendError> {
        self.last_error.as_ref()
    }

    fn imu(&self) -> Result<ImuSample, BackendError> {
        let view = self.view.as_ref().ok_or_else(|| BackendError::Unavailable {
            detail: "simulated backend has not stepped yet".into(),
        })?;
        let entity = view.entities.values().next().ok_or_else(|| BackendError::Unavailable {
            detail: "simulated world has no entities".into(),
        })?;
        Ok(ImuSample {
            ax: 0.0,
            ay: 0.0,
            az: 9.81,
            gx: 0.0,
            gy: 0.0,
            gz: entity.yaw,
        })
    }

    fn dropped_frames(&self) -> u64 {
        self.last_drop_count
    }

    fn last_step_us(&self) -> u64 {
        self.last_step_us
    }

    fn estimated_power(&self) -> Option<Power> {
        Some(Power::from_watts(120.0))
    }
}
