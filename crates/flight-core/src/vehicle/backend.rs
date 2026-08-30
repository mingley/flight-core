//! Vehicle backend trait, telemetry, and command types.

use crate::frames::{Body, Ned};
use crate::ground::GroundState;
use crate::marine::MarineState;
use crate::safety::Phase;
use crate::sensors::{ActuatorCommand, ImuSample, SensorHealth};
use crate::time::MonotonicInstant;
use crate::units::{Meter, Qty};
use crate::vector::{Position, Velocity};
use core::fmt;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConnectionInfo {
    pub system_id: u8,
    pub component_id: u8,
    pub autopilot: AutopilotKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AutopilotKind {
    Simulated,
    Px4,
    Unknown,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PreflightReport {
    pub imu_healthy: bool,
    pub estimator_valid: bool,
    pub battery_ok: bool,
    pub gps_ok: bool,
    pub notes: PreflightNotes,
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PreflightNotes {
    pub imu_std_accel: f32,
    pub imu_std_gyro: f32,
    pub samples: u32,
}

impl PreflightReport {
    pub fn ready(&self) -> bool {
        self.imu_healthy && self.estimator_valid && self.battery_ok
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorThrust {
    /// Normalized `[0, 1]` per motor.
    pub motors: [f32; 8],
    pub count: u8,
}

impl MotorThrust {
    pub fn hover(count: u8, fraction: f32) -> Self {
        let mut motors = [0.0; 8];
        let n = count.min(8) as usize;
        for m in motors.iter_mut().take(n) {
            *m = fraction.clamp(0.0, 1.0);
        }
        Self {
            motors,
            count: n as u8,
        }
    }

    pub fn to_actuator(self) -> ActuatorCommand {
        let mut motors = [0u16; 8];
        for (slot, frac) in motors.iter_mut().zip(self.motors.iter()) {
            *slot = (frac.clamp(0.0, 1.0) * 65535.0) as u16;
        }
        ActuatorCommand {
            motors,
            count: self.count,
            collective_n: None,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Telemetry {
    pub timestamp: MonotonicInstant,
    pub phase: Phase,
    pub position: Position<Ned>,
    pub velocity: Velocity<Ned>,
    pub yaw_rad: f32,
    pub imu: Option<ImuSample<Body>>,
    pub imu_health: SensorHealth,
    pub imu_healthy: bool,
    pub estimator_valid: bool,
    pub armed: bool,
    pub actuators_enabled: bool,
    pub offboard: bool,
    pub failsafe: bool,
    pub heartbeat_age_secs: f32,
    pub last_command: &'static str,
}

impl Telemetry {
    pub fn altitude_agl(&self) -> Qty<Meter> {
        self.position.altitude_agl()
    }

    /// Convert a companion/plant snapshot into the contract monitor sample.
    pub fn to_trace_sample(&self, epoch: u32) -> crate::contracts::TraceSample {
        let age = self.heartbeat_age_secs * 1000.0;
        let heartbeat_age_ms = if age <= 0.0 {
            0
        } else if age >= u32::MAX as f32 {
            u32::MAX
        } else {
            age as u32
        };
        crate::contracts::TraceSample {
            t_secs: self.timestamp.as_secs_f32(),
            armed: self.armed,
            actuators_enabled: self.actuators_enabled,
            failsafe: self.failsafe,
            epoch,
            heartbeat_age_ms,
            command: None,
            altitude_m: self.position.altitude_agl().get(),
            command_age_ms: 0,
            estimator_ts_ms: self.timestamp.as_nanos() / 1_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendError {
    Disconnected,
    Timeout,
    Protocol,
    Rejected(&'static str),
    Io,
}

impl fmt::Display for BackendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "backend disconnected"),
            Self::Timeout => write!(f, "backend timeout"),
            Self::Protocol => write!(f, "backend protocol error"),
            Self::Rejected(r) => write!(f, "backend rejected: {r}"),
            Self::Io => write!(f, "backend I/O error"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for BackendError {}

/// Poll a backend future that completes without parking (world / null).
fn poll_ready<F: Future>(fut: F) -> Option<F::Output> {
    let waker = Waker::noop();
    let mut cx = Context::from_waker(waker);
    let mut fut = pin!(fut);
    match fut.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Some(v),
        Poll::Pending => None,
    }
}

/// Hardware / SITL / replay / symbolic vehicle.
///
/// The controller talks only to this trait. It does not know whether the IMU
/// is a BMI088, an MCAP recording, a physics sim, or a Kani nondet value.
pub trait VehicleBackend: Send {
    fn connect(&mut self) -> impl Future<Output = Result<ConnectionInfo, BackendError>> + Send;

    /// [`Self::connect`] without an async runtime. A pending PX4 handshake is
    /// [`BackendError::Timeout`].
    fn connect_now(&mut self) -> Result<ConnectionInfo, BackendError> {
        match poll_ready(self.connect()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }
    fn preflight(&mut self) -> impl Future<Output = Result<PreflightReport, BackendError>> + Send;
    fn arm(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn disarm(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn enter_offboard(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_velocity_ned(
        &mut self,
        velocity: Velocity<Ned>,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_position_ned(
        &mut self,
        position: Position<Ned>,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn set_motor_thrust(
        &mut self,
        thrust: MotorThrust,
    ) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn enable_actuators(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn disable_actuators(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;
    fn tick(
        &mut self,
        dt_secs: f32,
    ) -> impl Future<Output = Result<Telemetry, BackendError>> + Send;
    fn telemetry(&mut self) -> impl Future<Output = Result<Telemetry, BackendError>> + Send;
    fn trigger_failsafe(&mut self) -> impl Future<Output = Result<(), BackendError>> + Send;

    /// Live safety epoch. Permits issued against a previous value have no
    /// authority. Default `0` for backends that do not yet track revocation.
    fn authority_epoch(&self) -> u32 {
        0
    }

    /// Vehicle identity the permit is bound to.
    fn authority_vehicle_id(&self) -> u8 {
        0
    }

    /// Clock sample used for lease expiry. Default is zero (unbounded leases).
    fn authority_now(&self) -> MonotonicInstant {
        MonotonicInstant::ZERO
    }

    /// Increment the safety epoch so every outstanding permit is stale.
    fn revoke_authority(&mut self) {}

    /// Age of the last vehicle heartbeat, in milliseconds.
    ///
    /// `None` means this backend does not track a companion heartbeat (null /
    /// point-mass). PX4 reports elapsed time since the last PX4 HEARTBEAT.
    fn authority_heartbeat_age_ms(&self) -> Option<u32> {
        None
    }

    /// Arm without an async runtime. Default polls [`Self::arm`] when it is
    /// already complete (world / null backends). A pending PX4 handshake is
    /// [`BackendError::Timeout`].
    fn arm_now(&mut self) -> Result<(), BackendError> {
        match poll_ready(self.arm()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Disarm without an async runtime. See [`Self::arm_now`].
    fn disarm_now(&mut self) -> Result<(), BackendError> {
        match poll_ready(self.disarm()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Enter offboard without an async runtime. See [`Self::arm_now`].
    fn enter_offboard_now(&mut self) -> Result<(), BackendError> {
        match poll_ready(self.enter_offboard()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Enable actuators without an async runtime. See [`Self::arm_now`].
    fn enable_actuators_now(&mut self) -> Result<(), BackendError> {
        match poll_ready(self.enable_actuators()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// NED velocity without an async runtime. See [`Self::arm_now`].
    fn set_velocity_ned_now(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        match poll_ready(self.set_velocity_ned(velocity)) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// NED position without an async runtime. See [`Self::arm_now`].
    fn set_position_ned_now(&mut self, position: Position<Ned>) -> Result<(), BackendError> {
        match poll_ready(self.set_position_ned(position)) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Hold at the current NED pose without an async runtime.
    /// Default reads [`Self::telemetry_now`] and writes [`Self::set_position_ned_now`].
    /// PX4 companion backends stream a position `SET_POSITION_TARGET_LOCAL_NED`
    /// at the last estimated pose (disconnected send is [`BackendError::Disconnected`]).
    fn hold_now(&mut self) -> Result<(), BackendError> {
        let position = self.telemetry_now()?.position;
        self.set_position_ned_now(position)
    }

    /// Telemetry without an async runtime. See [`Self::arm_now`].
    fn telemetry_now(&mut self) -> Result<Telemetry, BackendError> {
        match poll_ready(self.telemetry()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Trip failsafe without an async runtime. See [`Self::arm_now`].
    fn trigger_failsafe_now(&mut self) -> Result<(), BackendError> {
        match poll_ready(self.trigger_failsafe()) {
            Some(r) => r,
            None => Err(BackendError::Timeout),
        }
    }

    /// Takeoff without an async runtime. Default is a no-op so point-mass
    /// backends keep using [`Vehicle::start_takeoff_now`]'s local `Takeoff`
    /// event. World backends fire the live event so `Land` is legal on the
    /// plant. PX4 companion backends send `NAV_TAKEOFF`.
    fn takeoff_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Record that the climb completed without an async runtime. Default is a
    /// no-op so point-mass backends keep using
    /// [`Vehicle::declare_airborne_now`]'s local `ReachedAltitude` event.
    /// World backends fire the live event so attach binds Airborne. PX4
    /// companion backends send `NAV_LOITER_UNLIM`.
    fn reached_altitude_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Enter landing without an async runtime. Default is a no-op so
    /// point-mass backends keep using [`Vehicle::land`]. World backends fire
    /// the live `Land` event. PX4 companion backends send `NAV_LAND`.
    fn land_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Touchdown without an async runtime. Default is a no-op; world backends
    /// fire `Touchdown` and clear the command.
    fn touchdown_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Recover from aerial Recovery to Ready without an async runtime.
    /// Default is a no-op so PX4 / point-mass backends keep using the local
    /// `Recover` event. World backends fire the live event so attach binds
    /// PreflightReady.
    fn recover_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Halt a moving chassis without an async runtime. Default is a no-op so
    /// aerial backends stay valid. World ground backends fire `Halt` and
    /// clear the handle setpoint so a later flush cannot revive drive.
    fn halt_now(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    /// Mirror a ground safety machine onto a plant that has a chassis.
    ///
    /// Default is a no-op so PX4 / point-mass aerial backends stay valid.
    fn sync_ground(&mut self, safety: GroundState) -> Result<(), BackendError> {
        let _ = safety;
        Ok(())
    }

    /// Mirror a marine safety machine onto a plant that has a hull.
    fn sync_marine(&mut self, safety: MarineState) -> Result<(), BackendError> {
        let _ = safety;
        Ok(())
    }

    /// Body-frame yaw-rate command (rad/s). Ground and surface plants use this.
    fn set_yaw_rate(&mut self, yaw_rate: f32) -> Result<(), BackendError> {
        let _ = yaw_rate;
        Ok(())
    }
}

/// Backend that succeeds every call. Used to unit-test typestate wiring.
#[derive(Clone, Debug, Default)]
pub struct NullBackend {
    pub armed: bool,
    pub offboard: bool,
    pub actuators: bool,
    pub velocity: Option<Velocity<Ned>>,
    pub position: Option<Position<Ned>>,
    pub ticks: u32,
    /// Heading used when a ground vehicle rotates a body twist into NED.
    pub yaw_rad: f32,
    pub yaw_rate: f32,
    pub ground: Option<GroundState>,
    pub marine: Option<MarineState>,
    pub authority_epoch: u32,
}

impl VehicleBackend for NullBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        self.revoke_authority();
        Ok(ConnectionInfo {
            system_id: 1,
            component_id: 1,
            autopilot: AutopilotKind::Simulated,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        Ok(PreflightReport {
            imu_healthy: true,
            estimator_valid: true,
            battery_ok: true,
            gps_ok: true,
            notes: PreflightNotes {
                imu_std_accel: 0.01,
                imu_std_gyro: 0.001,
                samples: 50,
            },
        })
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        self.armed = true;
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        self.armed = false;
        self.actuators = false;
        self.offboard = false;
        self.revoke_authority();
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        self.offboard = true;
        Ok(())
    }

    async fn set_velocity_ned(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        self.velocity = Some(velocity);
        Ok(())
    }

    async fn set_position_ned(&mut self, position: Position<Ned>) -> Result<(), BackendError> {
        self.position = Some(position);
        Ok(())
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        Ok(())
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        self.actuators = true;
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        self.actuators = false;
        Ok(())
    }

    async fn tick(&mut self, _dt_secs: f32) -> Result<Telemetry, BackendError> {
        self.ticks += 1;
        self.telemetry().await
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        Ok(Telemetry {
            timestamp: MonotonicInstant::from_millis(u64::from(self.ticks) * 10),
            phase: Phase::Ready,
            position: self.position.unwrap_or_else(Position::zero),
            velocity: self.velocity.unwrap_or_else(Velocity::zero),
            yaw_rad: self.yaw_rad,
            imu: None,
            imu_health: SensorHealth::Ok,
            imu_healthy: true,
            estimator_valid: true,
            armed: self.armed,
            actuators_enabled: self.actuators,
            offboard: self.offboard,
            failsafe: false,
            heartbeat_age_secs: 0.0,
            last_command: "null",
        })
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        self.revoke_authority();
        Ok(())
    }

    fn authority_epoch(&self) -> u32 {
        self.authority_epoch
    }

    fn authority_now(&self) -> MonotonicInstant {
        MonotonicInstant::from_millis(u64::from(self.ticks) * 10)
    }

    fn revoke_authority(&mut self) {
        self.authority_epoch = self.authority_epoch.saturating_add(1);
    }

    fn sync_ground(&mut self, safety: GroundState) -> Result<(), BackendError> {
        self.ground = Some(safety);
        if !safety.drive_enabled {
            self.velocity = None;
        }
        Ok(())
    }

    fn sync_marine(&mut self, safety: MarineState) -> Result<(), BackendError> {
        self.marine = Some(safety);
        if !safety.thrust_enabled {
            self.velocity = None;
        }
        Ok(())
    }

    fn set_yaw_rate(&mut self, yaw_rate: f32) -> Result<(), BackendError> {
        self.yaw_rate = yaw_rate;
        Ok(())
    }
}
