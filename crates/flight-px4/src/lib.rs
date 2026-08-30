//! PX4 offboard backend over MAVLink.
//!
//! Talks to PX4 SITL the way a companion computer would, but the *vehicle API*
//! is `flight-core::vehicle::Vehicle` — not a thin wrap of the C++ ROS 2
//! interface library. Companion `land_now` / `hold_now` send `NAV_LAND` /
//! a position `SET_POSITION_TARGET_LOCAL_NED` at the last estimated pose.
//! `takeoff_now` / `reached_altitude_now` stay in PX4 offboard (velocity
//! climb / hover) — `NAV_TAKEOFF` / `NAV_LOITER_UNLIM` are AUTO and PX4
//! rejects a packed custom_mode on `MAV_CMD_DO_SET_MODE`. Offboard
//! `set_velocity_ned` / `set_position_ned` stream
//! `SET_POSITION_TARGET_LOCAL_NED` (velocity or position mask).
//! [`world_plant::WorldPlant::hold`] walks `attach_hold` on the verified
//! plant (Ready before ARM is Protocol).
//!
//! Default SITL endpoint: `udpin:0.0.0.0:14540` (PX4 publishes here).
//! [`Px4Backend::connect`] waits for a PX4 heartbeat so a bind with nobody
//! sending is [`BackendError::Timeout`], not a silent empty socket.
//! A common companion send path is `udpout:127.0.0.1:14580`.

#![deny(unsafe_code)]

pub mod world_plant;

use flight_core::frames::Ned;
use flight_core::safety::Phase;
use flight_core::sensors::SensorHealth;
use flight_core::time::MonotonicInstant;
use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, PreflightNotes, PreflightReport,
    Telemetry, Vehicle, VehicleBackend,
};
use flight_mavlink::{
    arm_disarm, flight_termination, gcs_heartbeat, heartbeat_reports_armed,
    heartbeat_revokes_authority, nav_land, set_offboard_mode, set_position_ned, set_velocity_ned,
    UdpLink,
};
use mavlink::common::{MavMessage, MavType};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct Px4Config {
    /// `mavlink::connect` address, e.g. `udpin:0.0.0.0:14540`.
    pub endpoint: String,
    pub system_id: u8,
    pub component_id: u8,
    pub target_system: u8,
    pub target_component: u8,
}

impl Default for Px4Config {
    fn default() -> Self {
        Self {
            endpoint: "udpin:0.0.0.0:14540".into(),
            system_id: 245,
            component_id: 190,
            target_system: 1,
            target_component: 1,
        }
    }
}

#[derive(Debug)]
pub struct Px4Backend {
    config: Px4Config,
    link: Option<UdpLink>,
    boot_ms: u32,
    last_velocity: Velocity<Ned>,
    last_position: Position<Ned>,
    stream_position: bool,
    armed: bool,
    offboard: bool,
    seen_px4: bool,
    seen_local_position: bool,
    failsafe_latched: bool,
    authority_epoch: u32,
    last_heartbeat: Option<Instant>,
}

impl Px4Backend {
    pub fn new(config: Px4Config) -> Self {
        Self {
            config,
            link: None,
            boot_ms: 0,
            last_velocity: Velocity::ned(0.0, 0.0, 0.0),
            last_position: Position::ned(0.0, 0.0, 0.0),
            stream_position: false,
            armed: false,
            offboard: false,
            seen_px4: false,
            seen_local_position: false,
            failsafe_latched: false,
            authority_epoch: 0,
            last_heartbeat: None,
        }
    }

    fn link(&mut self) -> Result<&mut UdpLink, BackendError> {
        self.link.as_mut().ok_or(BackendError::Disconnected)
    }

    fn send(&mut self, msg: &MavMessage) -> Result<(), BackendError> {
        self.link()?.send(msg).map_err(|_| BackendError::Io)
    }

    /// Drain waiting MAVLink frames. Live PX4 multiplexes heartbeats, IMU,
    /// and `LOCAL_POSITION_NED` on the same UDP port; a blocking `recv` of
    /// the next frame is not a pose sample.
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
                }
                Ok((_, MavMessage::HEARTBEAT(h)))
                    if h.mavtype == MavType::MAV_TYPE_QUADROTOR
                        || h.mavtype == MavType::MAV_TYPE_GENERIC =>
                {
                    self.ingest_heartbeat(h);
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
    }

    fn refresh_local_position(&mut self) {
        self.ingest_inbox();
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

    /// Apply a decoded PX4 HEARTBEAT (inbox path, tests, and alternative links).
    /// Unexpected disarm or a failsafe-shaped status revokes actuation authority.
    pub fn ingest_heartbeat(&mut self, h: mavlink::common::HEARTBEAT_DATA) {
        self.last_heartbeat = Some(Instant::now());
        self.seen_px4 = true;
        self.apply_vehicle_heartbeat(&h);
    }

    /// PX4 asynchronously left offboard authority. Idempotent: one epoch bump
    /// per latch, so a 1 Hz HEARTBEAT stream does not increment forever.
    fn apply_vehicle_heartbeat(&mut self, h: &mavlink::common::HEARTBEAT_DATA) {
        let hb_armed = heartbeat_reports_armed(h);
        let vehicle_failsafe = heartbeat_revokes_authority(h);
        let unexpected_disarm = self.armed && !hb_armed;
        if !(vehicle_failsafe || unexpected_disarm) {
            return;
        }
        if unexpected_disarm {
            self.armed = false;
            self.offboard = false;
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
}

impl VehicleBackend for Px4Backend {
    async fn connect(&mut self) -> Result<ConnectionInfo, BackendError> {
        let mut link = UdpLink::connect(
            &self.config.endpoint,
            self.config.system_id,
            self.config.component_id,
        )
        .map_err(|_| BackendError::Io)?;
        let _ = link.send(&gcs_heartbeat());
        self.link = Some(link);
        self.seen_px4 = false;
        self.seen_local_position = false;
        // `udpin` always binds. Wait for a PX4-shaped heartbeat so
        // `sitl_hover` exits when nothing is publishing on 14540.
        if !self.wait_inbox(Duration::from_secs(15), |b| b.seen_px4) {
            self.link = None;
            return Err(BackendError::Timeout);
        }
        Ok(ConnectionInfo {
            system_id: self.config.target_system,
            component_id: self.config.target_component,
            autopilot: AutopilotKind::Px4,
        })
    }

    async fn preflight(&mut self) -> Result<PreflightReport, BackendError> {
        let _ = self.send(&gcs_heartbeat());
        // Need at least one `LOCAL_POSITION_NED` so companion hold/position
        // stream a live pose, not the origin default.
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
        self.revoke_authority();
        Ok(())
    }

    async fn enter_offboard(&mut self) -> Result<(), BackendError> {
        // PX4 requires streaming setpoints *before* the mode switch, and
        // the offboard failsafe wants that stream to last ~1 s, not 20
        // back-to-back datagrams.
        for _ in 0..20 {
            self.pump_setpoint()?;
            self.ingest_inbox();
            std::thread::sleep(Duration::from_millis(50));
        }
        self.send(&set_offboard_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))?;
        self.offboard = true;
        Ok(())
    }

    async fn set_velocity_ned(&mut self, velocity: Velocity<Ned>) -> Result<(), BackendError> {
        self.last_velocity = velocity;
        self.stream_position = false;
        self.pump_setpoint()
    }

    async fn set_position_ned(&mut self, position: Position<Ned>) -> Result<(), BackendError> {
        self.last_position = position;
        self.stream_position = true;
        self.pump_setpoint()
    }

    async fn set_motor_thrust(&mut self, _thrust: MotorThrust) -> Result<(), BackendError> {
        Err(BackendError::Rejected(
            "direct motor thrust is not exposed over PX4 offboard; use velocity/position setpoints",
        ))
    }

    async fn enable_actuators(&mut self) -> Result<(), BackendError> {
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
            imu_healthy: self.seen_px4,
            estimator_valid: self.seen_local_position,
            armed: self.armed,
            actuators_enabled: self.armed && !self.failsafe_latched,
            offboard: self.offboard,
            failsafe: self.failsafe_latched,
            heartbeat_age_secs: self
                .last_heartbeat
                .map(|t| t.elapsed().as_secs_f32())
                .unwrap_or(1.0e6),
            last_command: "px4",
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

    /// Offboard climb: keep streaming and re-assert PX4 offboard. `NAV_TAKEOFF`
    /// is AUTO and leaves offboard (`Unsupported main mode` if we packed
    /// custom_mode into `DO_SET_MODE`; even with the unpacked main mode, AUTO
    /// takeoff fights the velocity loop in [`Vehicle::takeoff`]).
    fn takeoff_now(&mut self) -> Result<(), BackendError> {
        for _ in 0..5 {
            self.pump_setpoint()?;
        }
        self.send(&set_offboard_mode(
            self.config.target_system,
            self.config.target_component,
            self.armed,
        ))
    }

    /// Companion climb-complete: stay in offboard. WorldPlant still maps
    /// `MAV_CMD_NAV_LOITER_UNLIM` through `attach_airborne`.
    fn reached_altitude_now(&mut self) -> Result<(), BackendError> {
        self.pump_setpoint()?;
        self.send(&set_offboard_mode(
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

    /// Companion pose hold: position `SET_POSITION_TARGET_LOCAL_NED` at the
    /// last `LOCAL_POSITION_NED` (or the last streamed pose). No link is
    /// [`BackendError::Disconnected`].
    fn hold_now(&mut self) -> Result<(), BackendError> {
        self.refresh_local_position();
        let p = self.last_position;
        self.set_position_ned_now(p)
    }
}

/// Typed handle constructor. Connection happens on `Vehicle::connect`.
pub fn vehicle(config: Px4Config) -> Vehicle<flight_core::vehicle::Disconnected, Px4Backend> {
    Vehicle::new(Px4Backend::new(config))
}

pub use world_plant::WorldPlant;

/// True if a MAVLink heartbeat looks like PX4.
pub fn is_px4_heartbeat(msg: &MavMessage) -> bool {
    matches!(
        msg,
        MavMessage::HEARTBEAT(h) if h.mavtype == MavType::MAV_TYPE_QUADROTOR
            || h.mavtype == MavType::MAV_TYPE_GENERIC
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_mavlink::{local_position_ned, ned_position_from_target, px4_vehicle_heartbeat};
    use std::net::UdpSocket;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    use std::thread;

    #[test]
    fn default_endpoint_is_sitl() {
        assert!(Px4Config::default().endpoint.contains("14540"));
    }

    #[test]
    fn companion_now_apis_require_a_link() {
        let mut b = Px4Backend::new(Px4Config::default());
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
        let mut b = Px4Backend::new(Px4Config::default());
        assert_eq!(b.authority_epoch(), 0);
        let err = b.trigger_failsafe_now();
        assert!(matches!(err, Err(BackendError::Disconnected)));
        assert_eq!(b.authority_epoch(), 1);
        assert!(b.telemetry_now().unwrap().failsafe);
        assert!(!b.telemetry_now().unwrap().estimator_valid);
    }

    #[tokio::test]
    async fn critical_heartbeat_revokes_epoch_once() {
        use flight_mavlink::{
            px4_custom_mode, px4_vehicle_heartbeat_status, PX4_MAIN_MODE_OFFBOARD,
        };
        use mavlink::common::MavState;
        let (port, cfg) = ephemeral_udpin();
        let fake = thread::spawn(move || {
            let mut link =
                UdpLink::connect(&format!("udpout:127.0.0.1:{port}"), 1, 1).expect("fake px4 bind");
            for _ in 0..40 {
                let _ = link.send(&px4_vehicle_heartbeat_status(
                    true,
                    px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0),
                    MavState::MAV_STATE_CRITICAL,
                ));
                thread::sleep(Duration::from_millis(20));
            }
        });
        let mut backend = Px4Backend::new(cfg);
        backend.connect().await.expect("connect");
        assert!(backend.telemetry_now().unwrap().failsafe);
        let epoch = backend.authority_epoch();
        assert!(epoch >= 1);
        backend.ingest_inbox();
        backend.ingest_inbox();
        assert_eq!(
            backend.authority_epoch(),
            epoch,
            "1 Hz critical HEARTBEAT must not keep bumping"
        );
        let _ = fake.join();
    }

    fn ephemeral_udpin() -> (u16, Px4Config) {
        let probe = UdpSocket::bind("127.0.0.1:0").expect("probe bind");
        let port = probe.local_addr().expect("probe addr").port();
        drop(probe);
        let cfg = Px4Config {
            endpoint: format!("udpin:127.0.0.1:{port}"),
            ..Px4Config::default()
        };
        (port, cfg)
    }

    #[tokio::test]
    async fn companion_hold_streams_ingested_local_position() {
        let (port, cfg) = ephemeral_udpin();
        let got = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let got_sender = Arc::clone(&got);
        let stop_sender = Arc::clone(&stop);
        let fake = thread::spawn(move || {
            let mut link =
                UdpLink::connect(&format!("udpout:127.0.0.1:{port}"), 1, 1).expect("fake px4 bind");
            let mut i = 0u32;
            while !stop_sender.load(Ordering::Relaxed) {
                let _ = link.send(&px4_vehicle_heartbeat(false, 0));
                let _ = link.send(&local_position_ned(
                    i.saturating_mul(20),
                    1.0,
                    2.0,
                    -3.0,
                    0.0,
                    0.1,
                    0.0,
                ));
                if let Ok((_, MavMessage::SET_POSITION_TARGET_LOCAL_NED(d))) = link.try_recv() {
                    *got_sender.lock().expect("got lock") = ned_position_from_target(&d);
                }
                i = i.wrapping_add(1);
                thread::sleep(Duration::from_millis(20));
            }
        });

        let mut backend = Px4Backend::new(cfg);
        backend
            .connect()
            .await
            .expect("connect waits for heartbeat");
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
                panic!("fake PX4 never saw a position SET_POSITION_TARGET_LOCAL_NED");
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

    #[test]
    fn unexpected_disarm_heartbeat_revokes_stale_armed_vehicle() {
        use flight_core::contracts::AuthorityReject;
        use flight_core::safety::{step_all, Event, SafetyState};
        use flight_core::vehicle::{ErrorKind, VehicleHandle};
        let mut backend = Px4Backend::new(Px4Config::default());
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
        let MavMessage::HEARTBEAT(h) = px4_vehicle_heartbeat(false, 0) else {
            panic!("heartbeat");
        };
        armed.backend_mut().ingest_heartbeat(h);
        assert!(
            armed.backend().authority_epoch() >= 1,
            "async PX4 disarm must bump the epoch"
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
        assert!(err.vehicle.safety().armed);
    }
}
