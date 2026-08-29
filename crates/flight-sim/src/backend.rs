//! Point-mass aerial hover for PX4-shaped demos. **Not** the mechanically
//! verified property vector. Hold, failsafe, catalogs, and research land on
//! [`crate::WorldSession`]. This backend is a single-vehicle mixer + IMU
//! for `examples/hover`, not a multi-domain plant.

use crate::physics::{Physics, GRAVITY_NED};
use flight_core::frames::Body;
use flight_core::nav::ComplementaryAttitude;
use flight_core::safety::Phase;
use flight_core::sensors::{
    ActuatorCommand, ActuatorError, Actuators, Imu, ImuSample, SensorError, SensorHealth,
    SequenceTracker,
};
use flight_core::time::{Clock, Duration, MonotonicInstant, VirtualClock};
use flight_core::units::Qty;
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, PreflightNotes, PreflightReport,
    Telemetry, VehicleBackend,
};

#[derive(Clone, Copy, Debug)]
pub struct SimConfig {
    pub mass_kg: f32,
    pub dt_secs: f32,
    pub velocity_kp: f32,
    pub accel_limit: f32,
    /// If > 0, IMU accel is perturbed by a seeded uniform noise of this std.
    pub imu_accel_noise: f32,
    pub imu_gyro_noise: f32,
    pub seed: u64,
    pub motor_count: u8,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            mass_kg: 1.5,
            dt_secs: 0.01,
            velocity_kp: 2.8,
            accel_limit: 6.0,
            imu_accel_noise: 0.0,
            imu_gyro_noise: 0.0,
            seed: 1,
            motor_count: 4,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Setpoint {
    Velocity(Velocity<flight_core::frames::Ned>),
    Position(Position<flight_core::frames::Ned>),
}

#[derive(Clone, Debug)]
pub struct SimBackend {
    config: SimConfig,
    clock: VirtualClock,
    physics: Physics,
    setpoint: Option<Setpoint>,
    last_net_accel: [f32; 3],
    imu_seq: u32,
    seq: SequenceTracker,
    attitude: ComplementaryAttitude,
    connected: bool,
    armed: bool,
    actuators: bool,
    last_command: &'static str,
    last_command_at: MonotonicInstant,
    rng: u64,
}

impl SimBackend {
    pub fn new(config: SimConfig) -> Self {
        Self {
            rng: config.seed | 1,
            physics: Physics::grounded(config.mass_kg),
            config,
            clock: VirtualClock::new(),
            setpoint: None,
            last_net_accel: [0.0, 0.0, 0.0],
            imu_seq: 0,
            seq: SequenceTracker::new(),
            attitude: ComplementaryAttitude::new(),
            connected: false,
            armed: false,
            actuators: false,
            last_command: "idle",
            last_command_at: MonotonicInstant::ZERO,
        }
    }

    pub fn physics(&self) -> &Physics {
        &self.physics
    }

    pub fn clock(&self) -> &VirtualClock {
        &self.clock
    }

    pub fn estimator_valid(&self) -> bool {
        self.attitude.is_valid()
    }

    fn noise(&mut self, std: f32) -> f32 {
        if std <= 0.0 {
            return 0.0;
        }
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u = (self.rng >> 11) as f32 / (u64::MAX >> 11) as f32;
        (u * 2.0 - 1.0) * std
    }

    fn desired_net_accel(&self) -> [f32; 3] {
        match self.setpoint {
            Some(Setpoint::Velocity(sp)) if self.actuators && self.armed => {
                let v = self.physics.velocity();
                let kp = self.config.velocity_kp;
                [
                    (kp * (sp.x() - v.x()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                    (kp * (sp.y() - v.y()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                    (kp * (sp.z() - v.z()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                ]
            }
            Some(Setpoint::Position(sp)) if self.actuators && self.armed => {
                let p = self.physics.position();
                let pos_kp = 1.2;
                let vel_sp = [
                    pos_kp * (sp.x() - p.x()),
                    pos_kp * (sp.y() - p.y()),
                    pos_kp * (sp.z() - p.z()),
                ];
                let v = self.physics.velocity();
                let kp = self.config.velocity_kp;
                [
                    (kp * (vel_sp[0] - v.x()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                    (kp * (vel_sp[1] - v.y()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                    (kp * (vel_sp[2] - v.z()))
                        .clamp(-self.config.accel_limit, self.config.accel_limit),
                ]
            }
            _ => {
                if self.physics.on_ground() {
                    [0.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, GRAVITY_NED]
                }
            }
        }
    }

    fn step_physics(&mut self, dt: f32) {
        let a = self.desired_net_accel();
        self.last_net_accel = a;
        self.physics.step(a, 0.0, dt);
        self.clock.advance(Duration::from_secs_f32(dt));
        let sample = self.imu_sample();
        self.attitude.update(sample.gyro, sample.accel, dt);
    }

    fn imu_sample(&mut self) -> ImuSample<Body> {
        let mut accel = self.physics.body_accel(self.last_net_accel);
        let mut gyro = self.physics.body_gyro();
        accel = accel
            + flight_core::vector::Vector3::new(
                self.noise(self.config.imu_accel_noise),
                self.noise(self.config.imu_accel_noise),
                self.noise(self.config.imu_accel_noise),
            );
        gyro = gyro
            + flight_core::vector::Vector3::new(
                self.noise(self.config.imu_gyro_noise),
                self.noise(self.config.imu_gyro_noise),
                self.noise(self.config.imu_gyro_noise),
            );
        let sequence = self.imu_seq;
        self.imu_seq = self.imu_seq.wrapping_add(1);
        self.seq.observe(sequence);
        ImuSample {
            timestamp: self.clock.now(),
            accel,
            gyro,
            covariance: None,
            temperature: Some(Qty::new(25.0)),
            status: SensorHealth::Ok,
            sequence,
        }
    }

    fn snapshot(&mut self) -> Telemetry {
        let imu = self.imu_sample();
        Telemetry {
            timestamp: self.clock.now(),
            phase: Phase::Disconnected,
            position: self.physics.position(),
            velocity: self.physics.velocity(),
            yaw_rad: self.physics.yaw_rad,
            imu: Some(imu),
            imu_health: SensorHealth::Ok,
            imu_healthy: true,
            estimator_valid: self.attitude.is_valid(),
            armed: self.armed,
            actuators_enabled: self.actuators,
            offboard: self.setpoint.is_some(),
            failsafe: false,
            heartbeat_age_secs: self
                .clock
                .now()
                .saturating_duration_since(self.last_command_at)
                .as_secs_f32(),
            last_command: self.last_command,
        }
    }
}

impl Clock for SimBackend {
    fn now(&self) -> MonotonicInstant {
        self.clock.now()
    }
}

impl Imu for SimBackend {
    type Frame = Body;

    fn sample(&mut self) -> Result<ImuSample<Body>, SensorError> {
        Ok(self.imu_sample())
    }
}

impl Actuators for SimBackend {
    fn apply(&mut self, command: ActuatorCommand) -> Result<(), ActuatorError> {
        if !self.armed {
            return Err(ActuatorError::NotArmed);
        }
        if !self.actuators {
            return Err(ActuatorError::Disabled);
        }
        let _ = command;
        Ok(())
    }
}

impl VehicleBackend for SimBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        self.connected = true;
        self.last_command = "connect";
        Ok(ConnectionInfo {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotKind::Simulated,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        if !self.connected {
            return Err(BackendError::Disconnected);
        }
        // Spin the IMU long enough for the complementary filter to declare valid.
        for _ in 0..40 {
            self.step_physics(self.config.dt_secs);
        }
        self.last_command = "preflight";
        Ok(PreflightReport {
            imu_healthy: true,
            estimator_valid: self.attitude.is_valid(),
            battery_ok: true,
            gps_ok: true,
            notes: PreflightNotes {
                imu_std_accel: self.config.imu_accel_noise,
                imu_std_gyro: self.config.imu_gyro_noise,
                samples: self.attitude.sample_count(),
            },
        })
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        self.armed = true;
        self.last_command = "arm";
        self.last_command_at = self.clock.now();
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        self.armed = false;
        self.actuators = false;
        self.setpoint = None;
        self.last_command = "disarm";
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        self.setpoint = Some(Setpoint::Velocity(Velocity::ned(0.0, 0.0, 0.0)));
        self.last_command = "offboard";
        self.last_command_at = self.clock.now();
        Ok(())
    }

    async fn set_velocity_ned(
        &mut self,
        velocity: Velocity<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.setpoint = Some(Setpoint::Velocity(velocity));
        self.last_command = "set_velocity";
        self.last_command_at = self.clock.now();
        Ok(())
    }

    async fn set_position_ned(
        &mut self,
        position: Position<flight_core::frames::Ned>,
    ) -> Result<(), BackendError> {
        self.setpoint = Some(Setpoint::Position(position));
        self.last_command = "set_position";
        self.last_command_at = self.clock.now();
        Ok(())
    }

    async fn set_motor_thrust(&mut self, thrust: MotorThrust) -> Result<(), BackendError> {
        if !self.armed {
            return Err(BackendError::Rejected("not armed"));
        }
        let _ = thrust;
        self.last_command = "motor_thrust";
        self.last_command_at = self.clock.now();
        Ok(())
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        if !self.armed {
            return Err(BackendError::Rejected("not armed"));
        }
        self.actuators = true;
        self.last_command = "enable_actuators";
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        self.actuators = false;
        self.setpoint = None;
        self.last_command = "disable_actuators";
        Ok(())
    }

    async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, BackendError> {
        let dt = if dt_secs > 0.0 {
            dt_secs
        } else {
            self.config.dt_secs
        };
        // Substep at the configured rate so takeoff is stable.
        let mut remain = dt;
        while remain > 1e-6 {
            let step = remain.min(self.config.dt_secs);
            self.step_physics(step);
            remain -= step;
        }
        Ok(self.snapshot())
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        Ok(self.snapshot())
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        self.setpoint = None;
        self.last_command = "failsafe";
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::prelude::*;
    use flight_core::units::Qty;

    #[tokio::test]
    async fn connect_preflight_arm_takeoff_land() {
        let vehicle = crate::connect(SimConfig::default()).await.unwrap();
        let vehicle = vehicle.verify_preflight().await.unwrap();
        let vehicle = vehicle.arm().await.unwrap();
        let vehicle = vehicle
            .takeoff(Qty::from_meters(5.0))
            .await
            .expect("takeoff");
        assert!(vehicle.safety().armed);
        assert!(vehicle.safety().actuators_enabled);
        let alt = vehicle.backend().physics().position().altitude_agl().get();
        assert!(alt > 4.5, "altitude {alt}");

        let mut vehicle = vehicle;
        for _ in 0..80 {
            vehicle
                .set_velocity(Velocity::<Ned>::ned(1.5, 0.0, 0.0))
                .await
                .unwrap();
        }
        let north = vehicle.backend().physics().position().x();
        assert!(north > 1.0, "north {north}");

        let landed = vehicle.land().await.expect("land");
        let alt = landed.backend().physics().position().altitude_agl().get();
        assert!(alt < 0.2, "landed altitude {alt}");
        assert!(!landed.safety().armed);
    }

    #[test]
    fn controller_is_source_agnostic() {
        let mut sim = SimBackend::new(SimConfig::default());
        let cmd = ActuatorCommand::idle(4);
        // Not armed: actuator apply fails, but IMU still samples.
        let sample = sim.sample().unwrap();
        assert!(sample.accel.is_finite());
        assert_eq!(sim.apply(cmd), Err(ActuatorError::NotArmed));
    }
}
