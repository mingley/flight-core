//! ArduPilot GUIDED companion over MAVLink.
//!
//! Same [`flight_core::vehicle::Vehicle`] API as PX4 and the verified world.
//! Companion `takeoff_now` / `reached_altitude_now` stay in Copter GUIDED
//! (velocity then position `SET_POSITION_TARGET_LOCAL_NED`). They do **not**
//! send `MAV_CMD_NAV_TAKEOFF` (AUTO). `land_now` / `hold_now` send `NAV_LAND`
//! / a position setpoint at the last estimated pose. Disconnected send is
//! [`BackendError::Disconnected`].
//!
//! Default SITL endpoint: `udpin:0.0.0.0:14550`. Live Copter is optional and
//! `#[ignore]` (`tests/sitl_live.rs`). Default `cargo test` is loopback: leftover
//! Offboard `COMMANDS` after every `REVOKE_ON`, disconnected send, and a UDP
//! ingest of Copter HEARTBEAT / `LOCAL_POSITION_NED`.
//!
//! This crate does not depend on `flight-sim` or `flight-px4`.

#![deny(unsafe_code)]

use flight_core::contracts::{evaluate_trace, AerialOffboard, LeftoverContract, TraceSample};
use flight_core::frames::Ned;
use flight_core::safety::{Event, Phase, OFFBOARD_HEARTBEAT_MAX_AGE_MS};
use flight_core::sensors::SensorHealth;
use flight_core::temporal::{EstimateFresh, Sequence, ESTIMATE_MAX_AGE_MS};
use flight_core::time::MonotonicInstant;
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, PreflightNotes, PreflightReport,
    Telemetry, Vehicle, VehicleBackend, VehicleHandle,
};
use flight_mavlink::{
    ardupilot_heartbeat_revokes_authority, ardupilot_vehicle_heartbeat, arm_disarm,
    flight_termination, gcs_heartbeat, heartbeat_reports_armed, nav_land, set_guided_mode,
    set_position_ned, set_velocity_ned, UdpLink, ARDUPILOT_COPTER_GUIDED,
};
use mavlink::common::{MavMessage, MavType};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct ArduPilotConfig {
    /// `mavlink::connect` address, e.g. `udpin:0.0.0.0:14550`.
    pub endpoint: String,
    pub system_id: u8,
    pub component_id: u8,
    pub target_system: u8,
    pub target_component: u8,
}

impl Default for ArduPilotConfig {
    fn default() -> Self {
        Self {
            endpoint: "udpin:0.0.0.0:14550".into(),
            system_id: 245,
            component_id: 190,
            target_system: 1,
            target_component: 1,
        }
    }
}

#[derive(Debug)]
pub struct ArduPilotBackend {
    config: ArduPilotConfig,
    link: Option<UdpLink>,
    boot_ms: u32,
    last_velocity: Velocity<Ned>,
    last_position: Position<Ned>,
    stream_position: bool,
    armed: bool,
    offboard: bool,
    seen_vehicle: bool,
    seen_local_position: bool,
    failsafe_latched: bool,
    actuation_revoked: bool,
    authority_epoch: u32,
    last_heartbeat: Option<Instant>,
    last_local_position_at: Option<Instant>,
}

impl ArduPilotBackend {
    pub fn new(config: ArduPilotConfig) -> Self {
        Self {
            config,
            link: None,
            boot_ms: 0,
            last_velocity: Velocity::ned(0.0, 0.0, 0.0),
            last_position: Position::ned(0.0, 0.0, 0.0),
            stream_position: false,
            armed: false,
            offboard: false,
            seen_vehicle: false,
            seen_local_position: false,
            failsafe_latched: false,
            actuation_revoked: false,
            authority_epoch: 0,
            last_heartbeat: None,
            last_local_position_at: None,
        }
    }

    fn send(&mut self, msg: &MavMessage) -> Result<(), BackendError> {
        self.link
            .as_mut()
            .ok_or(BackendError::Disconnected)?
            .send(msg)
            .map_err(|_| BackendError::Io)
    }

    fn ingest_inbox(&mut self) {
        for _ in 0..64 {
            let recv = {
                let Some(link) = self.link.as_mut() else {
                    return;
                };
                link.try_recv()
            };
            match recv {
                Ok((_, MavMessage::LOCAL_POSITION_NED(p))) => {
                    self.last_position = Position::ned(p.x, p.y, p.z);
                    self.last_velocity = Velocity::ned(p.vx, p.vy, p.vz);
                    self.seen_local_position = true;
                    self.last_local_position_at = Some(Instant::now());
                }
                Ok((_, MavMessage::HEARTBEAT(h)))
                    if h.autopilot
                        == mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
                        && (h.mavtype == MavType::MAV_TYPE_QUADROTOR
                            || h.mavtype == MavType::MAV_TYPE_GENERIC) =>
                {
                    self.ingest_heartbeat(h);
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
    }

    fn wait_inbox(&mut self, timeout: Duration, pred: impl Fn(&Self) -> bool) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            self.ingest_inbox();
            if pred(self) {
                return true;
            }
            if Instant::now() >= deadline {
                self.ingest_inbox();
                return pred(self);
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// Physical-authority commands after failsafe or a revoking
    /// disarm/disconnect: setpoints, GUIDED entry, climb, actuators, thrust.
    /// Land / disarm / failsafe stay ungated. `pump_setpoint` stays ungated.
    fn refuse_revoked_setpoint(&self) -> Result<(), BackendError> {
        if self.failsafe_latched || self.actuation_revoked {
            return Err(BackendError::Rejected("actuation authority revoked"));
        }
        Ok(())
    }

    pub fn begin_session(&mut self) {
        self.revoke_authority();
    }

    fn local_position_age_ms(&self) -> Option<u32> {
        self.last_local_position_at.map(|t| {
            let ms = t.elapsed().as_millis();
            if ms > u128::from(u32::MAX) {
                u32::MAX
            } else {
                ms as u32
            }
        })
    }

    fn estimator_estimate(&self) -> flight_core::temporal::Estimate<()> {
        match self.local_position_age_ms() {
            None => flight_core::temporal::Estimate::new(
                (),
                false,
                flight_core::time::MonotonicInstant::ZERO,
            ),
            Some(age) => flight_core::temporal::Estimate::new(
                (),
                EstimateFresh::check_age(age).is_ok(),
                flight_core::time::MonotonicInstant::ZERO,
            ),
        }
    }

    fn maybe_revoke_stale_estimator(&mut self) {
        if self.local_position_age_ms().is_none() {
            return;
        }
        if self.estimator_estimate().revoke_event().is_none() {
            return;
        }
        if !self.failsafe_latched {
            self.failsafe_latched = true;
            self.offboard = false;
            self.revoke_authority();
        }
    }

    fn pump_setpoint(&mut self) -> Result<(), BackendError> {
        self.boot_ms = self.boot_ms.wrapping_add(20);
        let msg = if self.stream_position {
            let p = self.last_position;
            set_position_ned(
                self.config.target_system,
                self.config.target_component,
                self.boot_ms,
                p.x(),
                p.y(),
                p.z(),
            )
        } else {
            let v = self.last_velocity;
            set_velocity_ned(
                self.config.target_system,
                self.config.target_component,
                self.boot_ms,
                v.x(),
                v.y(),
                v.z(),
            )
        };
        self.send(&msg)
    }

    pub fn ingest_heartbeat(&mut self, h: mavlink::common::HEARTBEAT_DATA) {
        self.last_heartbeat = Some(Instant::now());
        self.seen_vehicle = true;
        let hb_armed = heartbeat_reports_armed(&h);
        let vehicle_failsafe = ardupilot_heartbeat_revokes_authority(&h);
        let unexpected_disarm = self.armed && !hb_armed;
        if !(vehicle_failsafe || unexpected_disarm) {
            return;
        }
        if unexpected_disarm {
            self.armed = false;
            self.offboard = false;
            self.actuation_revoked = true;
        }
        if vehicle_failsafe {
            self.offboard = false;
        }
        if !self.failsafe_latched {
            if vehicle_failsafe {
                self.failsafe_latched = true;
            }
            self.revoke_authority();
        }
    }

    /// Companion-shaped inject of a kernel revoke event. A leftover
    /// `Vehicle<Offboard>` bound before this call must then fail
    /// [`Vehicle::leftover_commands_stale`].
    pub fn inject_revoke(&mut self, event: Event) -> Result<(), BackendError> {
        let Some(e) = AerialOffboard::inject(event) else {
            return Err(BackendError::Rejected("not a revoke inject"));
        };
        let before = self.authority_epoch;
        match e {
            Event::TriggerFailsafe => {
                let _ = self.trigger_failsafe_now();
            }
            Event::Disarm => {
                self.armed = true;
                let MavMessage::HEARTBEAT(h) =
                    ardupilot_vehicle_heartbeat(false, u32::from(ARDUPILOT_COPTER_GUIDED))
                else {
                    return Err(BackendError::Rejected("heartbeat"));
                };
                self.ingest_heartbeat(h);
            }
            Event::Disconnect => {
                self.link = None;
                self.offboard = false;
                self.armed = false;
                self.failsafe_latched = false;
                self.actuation_revoked = true;
                self.revoke_authority();
            }
            Event::HeartbeatStale => {
                self.last_heartbeat = Some(instant_age_ms(OFFBOARD_HEARTBEAT_MAX_AGE_MS));
                if !self.failsafe_latched {
                    self.failsafe_latched = true;
                    self.offboard = false;
                }
                self.revoke_authority();
            }
            Event::EstimatorInvalid => {
                self.seen_local_position = true;
                self.last_local_position_at = Some(instant_age_ms(ESTIMATE_MAX_AGE_MS));
                self.maybe_revoke_stale_estimator();
            }
            Event::ImuUnhealthy => {
                self.seen_vehicle = false;
                if self.armed && !self.failsafe_latched {
                    self.failsafe_latched = true;
                    self.offboard = false;
                }
                self.revoke_authority();
            }
            _ => return Err(BackendError::Rejected("not a revoke inject")),
        }
        if self.authority_epoch == before {
            self.revoke_authority();
        }
        if self.authority_epoch == before {
            return Err(BackendError::Rejected("revoke inject did not bump epoch"));
        }
        Ok(())
    }
}

fn instant_age_ms(ms: u32) -> Instant {
    Instant::now() - Duration::from_millis(u64::from(ms))
}

impl VehicleBackend for ArduPilotBackend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        self.begin_session();
        let mut link = UdpLink::connect(
            &self.config.endpoint,
            self.config.system_id,
            self.config.component_id,
        )
        .map_err(|_| BackendError::Io)?;
        let _ = link.send(&gcs_heartbeat());
        self.link = Some(link);
        self.seen_vehicle = false;
        self.seen_local_position = false;
        if !self.wait_inbox(Duration::from_secs(15), |b| b.seen_vehicle) {
            self.link = None;
            return Err(BackendError::Timeout);
        }
        Ok(ConnectionInfo {
            system_id: self.config.target_system,
            component_id: self.config.target_component,
            autopilot: AutopilotKind::ArduPilot,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        let _ = self.send(&gcs_heartbeat());
        if !self.wait_inbox(Duration::from_secs(10), |b| b.seen_local_position) {
            return Err(BackendError::Timeout);
        }
        Ok(PreflightReport {
            imu_healthy: true,
            estimator_valid: true,
            battery_ok: true,
            gps_ok: true,
            notes: PreflightNotes {
                imu_std_accel: 0.0,
                imu_std_gyro: 0.0,
                samples: 1,
            },
        })
    }

    async fn arm(&mut self) -> Result<(), BackendError> {
        for _ in 0..10 {
            self.pump_setpoint()?;
            let _ = self.send(&gcs_heartbeat());
            self.ingest_inbox();
            std::thread::sleep(Duration::from_millis(20));
        }
        self.send(&arm_disarm(
            self.config.target_system,
            self.config.target_component,
            true,
        ))?;
        self.armed = true;
        self.actuation_revoked = false;
        Ok(())
    }

    async fn disarm(&mut self) -> Result<(), BackendError> {
        self.send(&arm_disarm(
            self.config.target_system,
            self.config.target_component,
            false,
        ))?;
        self.armed = false;
        self.offboard = false;
        self.actuation_revoked = true;
        self.revoke_authority();
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        self.refuse_revoked_setpoint()?;
        for _ in 0..10 {
            self.pump_setpoint()?;
            self.ingest_inbox();
            std::thread::sleep(Duration::from_millis(20));
        }
        self.send(&set_guided_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))?;
        self.offboard = true;
        Ok(())
    }

    async fn set_velocity_ned(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        self.ingest_inbox();
        self.maybe_revoke_stale_estimator();
        self.refuse_revoked_setpoint()?;
        self.last_velocity = velocity;
        self.stream_position = false;
        self.pump_setpoint()
    }

    async fn set_position_ned(&mut self, position: Position<Ned>) -> Result<(), BackendError> {
        self.ingest_inbox();
        self.maybe_revoke_stale_estimator();
        self.refuse_revoked_setpoint()?;
        self.last_position = position;
        self.stream_position = true;
        self.pump_setpoint()
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        self.refuse_revoked_setpoint()?;
        Err(BackendError::Rejected(
            "direct motor thrust is not exposed over ArduPilot GUIDED; use velocity/position setpoints",
        ))
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
        self.refuse_revoked_setpoint()?;
        Ok(())
    }

    async fn disable_actuators(&mut self) -> Result<(), BackendError> {
        Ok(())
    }

    async fn tick(&mut self, dt_secs: f32) -> Result<Telemetry, BackendError> {
        let _ = self.send(&gcs_heartbeat());
        let _ = self.pump_setpoint();
        self.ingest_inbox();
        let dt = dt_secs.clamp(0.0, 0.2);
        if dt > 0.0 {
            std::thread::sleep(Duration::from_secs_f32(dt));
            self.ingest_inbox();
        }
        self.telemetry().await
    }

    async fn telemetry(&mut self) -> Result<Telemetry, BackendError> {
        Ok(Telemetry {
            timestamp: MonotonicInstant::from_millis(u64::from(self.boot_ms)),
            phase: if self.offboard {
                Phase::Airborne
            } else if self.armed {
                Phase::Armed
            } else {
                Phase::Ready
            },
            position: self.last_position,
            velocity: self.last_velocity,
            yaw_rad: 0.0,
            imu: None,
            imu_health: SensorHealth::Ok,
            imu_healthy: self.seen_vehicle,
            estimator_valid: self.seen_local_position
                && self.estimator_estimate().revoke_event().is_none(),
            armed: self.armed,
            actuators_enabled: self.armed && !self.failsafe_latched,
            offboard: self.offboard,
            failsafe: self.failsafe_latched,
            heartbeat_age_secs: self
                .last_heartbeat
                .map(|t| t.elapsed().as_secs_f32())
                .unwrap_or(1.0e6),
            last_command: "ardupilot",
        })
    }

    async fn trigger_failsafe(&mut self) -> Result<(), BackendError> {
        self.failsafe_latched = true;
        self.offboard = false;
        self.revoke_authority();
        self.send(&flight_termination(
            self.config.target_system,
            self.config.target_component,
            true,
        ))?;
        Ok(())
    }

    fn authority_epoch(&self) -> u32 {
        self.authority_epoch
    }

    fn authority_vehicle_id(&self) -> u8 {
        self.config.target_system
    }

    fn authority_now(&self) -> MonotonicInstant {
        MonotonicInstant::from_millis(u64::from(self.boot_ms))
    }

    fn revoke_authority(&mut self) {
        self.authority_epoch = self.authority_epoch.saturating_add(1);
    }

    fn authority_heartbeat_age_ms(&self) -> Option<u32> {
        Some(
            self.last_heartbeat
                .map(|t| {
                    let ms = t.elapsed().as_millis();
                    if ms > u128::from(u32::MAX) {
                        u32::MAX
                    } else {
                        ms as u32
                    }
                })
                .unwrap_or(u32::MAX),
        )
    }

    /// GUIDED climb. `NAV_TAKEOFF` is AUTO and fights typestate velocity climb.
    fn takeoff_now(&mut self) -> Result<(), BackendError> {
        self.refuse_revoked_setpoint()?;
        for _ in 0..5 {
            self.pump_setpoint()?;
        }
        self.send(&set_guided_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))
    }

    fn reached_altitude_now(&mut self) -> Result<(), BackendError> {
        self.refuse_revoked_setpoint()?;
        self.pump_setpoint()?;
        self.send(&set_guided_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))
    }

    fn land_now(&mut self) -> Result<(), BackendError> {
        self.send(&nav_land(
            self.config.target_system,
            self.config.target_component,
        ))
    }

    fn hold_now(&mut self) -> Result<(), BackendError> {
        self.ingest_inbox();
        let p = self.last_position;
        self.set_position_ned_now(p)
    }
}

pub fn vehicle(
    config: ArduPilotConfig,
) -> Vehicle<flight_core::vehicle::Disconnected, ArduPilotBackend> {
    Vehicle::new(ArduPilotBackend::new(config))
}

fn offboard_safety() -> flight_core::safety::SafetyState {
    flight_core::safety::step_all(
        flight_core::safety::SafetyState::disconnected(),
        &[
            Event::Connect,
            Event::InitComplete,
            Event::Initialized,
            Event::ImuHealthy,
            Event::EstimatorValid,
            Event::PreflightPassed,
            Event::Arm,
            Event::HeartbeatFresh,
            Event::EnterOffboard,
        ],
    )
    .expect("kernel walk to Offboard")
}

/// Same leftover Offboard contract as world `run_revoke_table`, at the
/// ArduPilot companion. `flight-sim` does not depend on this crate.
pub fn run_ardupilot_revoke_table() -> Result<usize, String> {
    let mut n = 0;
    for e in AerialOffboard::REVOKE_ON {
        let inject = AerialOffboard::inject(*e)
            .ok_or_else(|| format!("{e:?} is in REVOKE_ON but inject returned None"))?;
        let mut backend = ArduPilotBackend::new(ArduPilotConfig::default());
        backend.armed = true;
        backend.offboard = true;
        backend.last_heartbeat = Some(Instant::now());
        let VehicleHandle::Offboard(mut v) = VehicleHandle::from_state(backend, offboard_safety())
        else {
            return Err(format!("offboard safety maps to Offboard before {e:?}"));
        };
        if v.leftover_commands_stale().is_ok() {
            return Err(format!(
                "leftover Offboard already stale before ArduPilot inject {e:?}"
            ));
        }
        let mut seq = Sequence::new();
        seq.observe(v.backend().authority_epoch())
            .map_err(|_| format!("sequence before {e:?}"))?;
        v.backend_mut()
            .inject_revoke(inject)
            .map_err(|err| format!("inject {e:?}: {err}"))?;
        seq.observe(v.backend().authority_epoch())
            .map_err(|_| format!("epoch jumped backward after {e:?}"))?;
        if v.backend().authority_epoch() == 0 {
            return Err(format!("event {e:?} did not bump epoch"));
        }
        match inject {
            Event::Disconnect | Event::Disarm => {
                let tel = v
                    .backend_mut()
                    .telemetry_now()
                    .map_err(|err| format!("telemetry after {e:?}: {err}"))?;
                if tel.failsafe {
                    return Err(format!("{e:?} must not latch failsafe"));
                }
            }
            Event::TriggerFailsafe
            | Event::HeartbeatStale
            | Event::EstimatorInvalid
            | Event::ImuUnhealthy => {
                let tel = v
                    .backend_mut()
                    .telemetry_now()
                    .map_err(|err| format!("telemetry after {e:?}: {err}"))?;
                if !tel.failsafe {
                    return Err(format!("{e:?} must latch failsafe"));
                }
            }
            _ => {}
        }
        v.leftover_commands_stale()
            .map_err(|err| format!("leftover after {e:?}: {err}"))?;
        n += 1;
    }
    Ok(n)
}

/// Leftover Offboard after `contract.inject`. `flight-sim` does not depend
/// on this crate.
pub fn run_ardupilot_leftover_contract(
    contract: LeftoverContract,
) -> Result<LeftoverContractReport, String> {
    let mut backend = ArduPilotBackend::new(ArduPilotConfig::default());
    backend.armed = true;
    backend.offboard = true;
    backend.last_heartbeat = Some(Instant::now());
    let VehicleHandle::Offboard(mut v) = VehicleHandle::from_state(backend, offboard_safety())
    else {
        return Err(format!(
            "{}: offboard safety maps to Offboard",
            contract.name
        ));
    };
    if v.leftover_commands_stale().is_ok() {
        return Err(format!(
            "{}: leftover Offboard already stale before inject",
            contract.name
        ));
    }
    let epoch0 = v.backend().authority_epoch();
    let before = v
        .backend_mut()
        .telemetry_now()
        .map_err(|e| format!("{} telemetry before: {e}", contract.name))?
        .to_trace_sample(epoch0);
    if before.failsafe {
        return Err(format!(
            "{}: failsafe already latched before inject",
            contract.name
        ));
    }
    v.backend_mut()
        .inject_revoke(contract.inject)
        .map_err(|e| format!("{} inject {:?}: {e}", contract.name, contract.inject))?;
    v.leftover_commands_stale()
        .map_err(|e| format!("{} leftover after inject: {e}", contract.name))?;
    let epoch1 = v.backend().authority_epoch();
    if epoch1 <= epoch0 {
        return Err(format!("{}: inject did not bump epoch", contract.name));
    }
    let after = v
        .backend_mut()
        .telemetry_now()
        .map_err(|e| format!("{} telemetry after: {e}", contract.name))?
        .to_trace_sample(epoch1);
    if !after.failsafe {
        return Err(format!(
            "{}: {:?} must latch failsafe",
            contract.name, contract.inject
        ));
    }
    let samples = vec![before, after];
    evaluate_trace(&samples, contract.require)
        .map_err(|e| format!("{} {} at {}", contract.name, e.requirement, e.index))?;
    AerialOffboard::evaluate(&samples).map_err(|e| {
        format!(
            "{} capability {} at {}",
            contract.name, e.requirement, e.index
        )
    })?;
    Ok(LeftoverContractReport {
        name: contract.name,
        inject: contract.inject,
        samples,
    })
}

/// Every distinct leftover contract at the ArduPilot GUIDED companion.
pub fn run_ardupilot_leftover_contracts() -> Result<Vec<LeftoverContractReport>, String> {
    AerialOffboard::LEFTOVER_CONTRACTS
        .iter()
        .copied()
        .map(run_ardupilot_leftover_contract)
        .collect()
}

/// Companion GPS-loss: leftover Offboard after `EstimatorInvalid`.
pub fn run_ardupilot_gps_loss() -> Result<ArduPilotGpsLossReport, String> {
    let report = run_ardupilot_leftover_contract(AerialOffboard::GPS_LOSS_CONTRACT)?;
    Ok(ArduPilotGpsLossReport {
        samples: report.samples,
    })
}

/// Result of [`run_ardupilot_leftover_contract`].
#[derive(Clone, Debug)]
pub struct LeftoverContractReport {
    pub name: &'static str,
    pub inject: Event,
    pub samples: Vec<TraceSample>,
}

/// Result of [`run_ardupilot_gps_loss`].
#[derive(Clone, Debug)]
pub struct ArduPilotGpsLossReport {
    pub samples: Vec<TraceSample>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_mavlink::{
        ardupilot_vehicle_heartbeat, ardupilot_vehicle_heartbeat_status, local_position_ned,
        ned_position_from_target, ARDUPILOT_COPTER_GUIDED,
    };
    use mavlink::common::MavState;
    use std::net::UdpSocket;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    use std::thread;

    #[test]
    fn default_endpoint_is_sitl() {
        assert!(ArduPilotConfig::default().endpoint.contains("14550"));
    }

    #[test]
    fn companion_now_apis_require_a_link() {
        let mut b = ArduPilotBackend::new(ArduPilotConfig::default());
        assert!(matches!(b.takeoff_now(), Err(BackendError::Disconnected)));
        assert!(matches!(
            b.reached_altitude_now(),
            Err(BackendError::Disconnected)
        ));
        assert!(matches!(b.land_now(), Err(BackendError::Disconnected)));
        assert!(matches!(b.hold_now(), Err(BackendError::Disconnected)));
        assert!(matches!(
            b.set_position_ned_now(Position::ned(0.0, 0.0, -2.0)),
            Err(BackendError::Disconnected)
        ));
    }

    #[test]
    fn failsafe_revokes_epoch_even_when_disconnected() {
        let mut b = ArduPilotBackend::new(ArduPilotConfig::default());
        assert_eq!(b.authority_epoch(), 0);
        let err = b.trigger_failsafe_now();
        assert!(matches!(err, Err(BackendError::Disconnected)));
        assert_eq!(b.authority_epoch(), 1);
        assert!(b.telemetry_now().unwrap().failsafe);
        assert!(!b.telemetry_now().unwrap().estimator_valid);
    }

    #[test]
    fn failsafe_refuses_setpoint_at_the_backend() {
        let mut b = ArduPilotBackend::new(ArduPilotConfig::default());
        b.armed = true;
        let err = b.trigger_failsafe_now();
        assert!(matches!(err, Err(BackendError::Disconnected)));
        let err = b.set_velocity_ned_now(Velocity::<Ned>::ned(1.0, 0.0, 0.0));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.set_position_ned_now(Position::<Ned>::ned(0.0, 0.0, -1.0));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.hold_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.enter_offboard_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.takeoff_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.enable_actuators_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
    }

    #[test]
    fn unexpected_disarm_heartbeat_revokes_stale_armed_vehicle() {
        use flight_core::contracts::AuthorityReject;
        use flight_core::safety::{step_all, Event, SafetyState};
        use flight_core::vehicle::{ErrorKind, VehicleHandle};
        let mut backend = ArduPilotBackend::new(ArduPilotConfig::default());
        backend.armed = true;
        let safety = step_all(
            SafetyState::disconnected(),
            &[
                Event::Connect,
                Event::InitComplete,
                Event::Initialized,
                Event::ImuHealthy,
                Event::EstimatorValid,
                Event::PreflightPassed,
                Event::Arm,
            ],
        )
        .expect("armed");
        let VehicleHandle::Armed(mut armed) = VehicleHandle::from_state(backend, safety) else {
            panic!("armed safety maps to Armed");
        };
        assert_eq!(armed.backend().authority_epoch(), 0);
        let MavMessage::HEARTBEAT(h) = ardupilot_vehicle_heartbeat(false, 0) else {
            panic!("heartbeat");
        };
        armed.backend_mut().ingest_heartbeat(h);
        assert!(
            armed.backend().authority_epoch() >= 1,
            "async ArduPilot disarm must bump the epoch"
        );
        let err = armed.enter_offboard_now().unwrap_err();
        assert!(
            matches!(
                err.error,
                ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch)
            ),
            "{:?}",
            err.error
        );
        let mut armed = err.vehicle;
        let err = armed
            .set_motor_thrust_now(MotorThrust::hover(4, 0.4))
            .unwrap_err();
        assert!(matches!(
            err,
            ErrorKind::StaleAuthority(AuthorityReject::StaleEpoch)
        ));
        assert!(armed.safety().armed);
        assert!(!armed.safety().actuators_enabled);
    }

    #[test]
    fn unexpected_disarm_refuses_setpoint_at_the_backend() {
        let mut b = ArduPilotBackend::new(ArduPilotConfig::default());
        b.armed = true;
        b.offboard = true;
        let MavMessage::HEARTBEAT(h) = ardupilot_vehicle_heartbeat(false, 0) else {
            panic!("heartbeat");
        };
        b.ingest_heartbeat(h);
        assert!(b.authority_epoch() >= 1);
        assert!(!b.telemetry_now().unwrap().failsafe);
        let err = b.set_velocity_ned_now(Velocity::<Ned>::ned(1.0, 0.0, 0.0));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.set_position_ned_now(Position::<Ned>::ned(0.0, 0.0, -1.0));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.hold_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.enter_offboard_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.takeoff_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.reached_altitude_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.enable_actuators_now();
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.set_motor_thrust_now(MotorThrust::hover(4, 0.4));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        let err = b.land_now();
        assert!(
            matches!(err, Err(BackendError::Disconnected)),
            "land stays an ungated safety action: {err:?}"
        );
    }

    #[test]
    fn stale_local_position_estimate_refuses_setpoint() {
        let mut b = ArduPilotBackend::new(ArduPilotConfig::default());
        b.armed = true;
        b.seen_local_position = true;
        b.last_local_position_at =
            Some(Instant::now() - Duration::from_millis(u64::from(ESTIMATE_MAX_AGE_MS)));
        let err = b.set_position_ned_now(Position::<Ned>::ned(0.0, 0.0, -1.0));
        assert!(matches!(err, Err(BackendError::Rejected(_))), "{err:?}");
        assert!(b.authority_epoch() > 0);
        assert!(b.telemetry_now().unwrap().failsafe);
        assert!(!b.telemetry_now().unwrap().estimator_valid);
    }

    #[test]
    fn leftover_revoke_table_is_the_dsl_events() {
        let n = run_ardupilot_revoke_table().expect("ardupilot leftover");
        assert_eq!(n, AerialOffboard::REVOKE_ON.len());
    }

    #[test]
    fn leftover_offboard_gps_loss_satisfies_world_contract() {
        let report = run_ardupilot_gps_loss().expect("ardupilot gps-loss");
        assert_eq!(report.samples.len(), 2);
        assert!(!report.samples[0].failsafe);
        assert!(report.samples[1].failsafe);
        assert!(report.samples[1].epoch > report.samples[0].epoch);
    }

    #[test]
    fn leftover_named_contracts_satisfy_world_monitors() {
        let reports = run_ardupilot_leftover_contracts().expect("ardupilot leftover contracts");
        assert_eq!(reports.len(), AerialOffboard::LEFTOVER_CONTRACTS.len());
        for (report, contract) in reports.iter().zip(AerialOffboard::LEFTOVER_CONTRACTS) {
            assert_eq!(report.name, contract.name);
            assert_eq!(report.inject, contract.inject);
            assert_eq!(report.samples.len(), 2);
            assert!(!report.samples[0].failsafe);
            assert!(report.samples[1].failsafe);
            assert!(report.samples[1].epoch > report.samples[0].epoch);
        }
    }

    fn ephemeral_udpin() -> (u16, ArduPilotConfig) {
        let probe = UdpSocket::bind("127.0.0.1:0").expect("probe bind");
        let port = probe.local_addr().expect("probe addr").port();
        drop(probe);
        let cfg = ArduPilotConfig {
            endpoint: format!("udpin:127.0.0.1:{port}"),
            ..ArduPilotConfig::default()
        };
        (port, cfg)
    }

    #[tokio::test]
    async fn critical_heartbeat_revokes_epoch_once() {
        let (port, cfg) = ephemeral_udpin();
        let fake = thread::spawn(move || {
            let mut link = UdpLink::connect(&format!("udpout:127.0.0.1:{port}"), 1, 1)
                .expect("fake ardupilot bind");
            for _ in 0..40 {
                let _ = link.send(&ardupilot_vehicle_heartbeat_status(
                    true,
                    u32::from(ARDUPILOT_COPTER_GUIDED),
                    MavState::MAV_STATE_CRITICAL,
                ));
                thread::sleep(Duration::from_millis(20));
            }
        });
        let mut backend = ArduPilotBackend::new(cfg);
        let info = backend.connect().await.expect("connect");
        assert_eq!(info.autopilot, AutopilotKind::ArduPilot);
        assert!(backend.telemetry_now().unwrap().failsafe);
        let epoch = backend.authority_epoch();
        assert!(epoch >= 1);
        backend.ingest_inbox();
        backend.ingest_inbox();
        assert_eq!(backend.authority_epoch(), epoch);
        let _ = fake.join();
    }

    #[tokio::test]
    async fn companion_hold_streams_ingested_local_position() {
        let (port, cfg) = ephemeral_udpin();
        let got = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let got_sender = Arc::clone(&got);
        let stop_sender = Arc::clone(&stop);
        let fake = thread::spawn(move || {
            let mut link = UdpLink::connect(&format!("udpout:127.0.0.1:{port}"), 1, 1)
                .expect("fake ardupilot bind");
            let mut i = 0u32;
            while !stop_sender.load(Ordering::Relaxed) {
                let _ = link.send(&ardupilot_vehicle_heartbeat(
                    false,
                    u32::from(ARDUPILOT_COPTER_GUIDED),
                ));
                let _ = link.send(&local_position_ned(
                    i.saturating_mul(20),
                    1.0,
                    2.0,
                    -3.0,
                    0.0,
                    0.0,
                    0.0,
                ));
                if let Ok((_, MavMessage::SET_POSITION_TARGET_LOCAL_NED(d))) = link.try_recv() {
                    *got_sender.lock().expect("got lock") = ned_position_from_target(&d);
                }
                i = i.wrapping_add(1);
                thread::sleep(Duration::from_millis(20));
            }
        });
        let mut backend = ArduPilotBackend::new(cfg);
        backend
            .connect()
            .await
            .expect("connect waits for Copter heartbeat");
        backend
            .preflight()
            .await
            .expect("preflight waits for LOCAL_POSITION_NED");
        let tel = backend.telemetry().await.expect("telemetry");
        assert!(
            (tel.position.x() - 1.0).abs() < 1e-3
                && (tel.position.y() - 2.0).abs() < 1e-3
                && (tel.position.z() + 3.0).abs() < 1e-3,
            "ingested pose {:?}",
            tel.position
        );
        backend.hold_now().expect("hold at ingested pose");

        let deadline = Instant::now() + Duration::from_secs(3);
        let pose = loop {
            if let Some(p) = *got.lock().expect("got lock") {
                break p;
            }
            if Instant::now() >= deadline {
                panic!("fake Copter never saw a position SET_POSITION_TARGET_LOCAL_NED");
            }
            thread::sleep(Duration::from_millis(20));
        };
        assert!(
            (pose.0 - 1.0).abs() < 0.05
                && (pose.1 - 2.0).abs() < 0.05
                && (pose.2 + 3.0).abs() < 0.05,
            "hold streamed {pose:?}"
        );

        stop.store(true, Ordering::Relaxed);
        let _ = fake.join();
    }
}
