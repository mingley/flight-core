//! Verified `robot-world` as the plant behind PX4 offboard MAVLink.
//!
//! [`Px4Backend`](crate::Px4Backend) is the companion: it *sends*
//! `SET_POSITION_TARGET_LOCAL_NED`. [`WorldPlant`] is the scene: it *applies*
//! that same message, steps the mechanically verified world, and *publishes*
//! `LOCAL_POSITION_NED` — the packet `Px4Backend::tick` already reads.
//!
//! A live PX4 binary is optional. This plant is the shared world step.
//! `MAV_CMD_COMPONENT_ARM_DISARM` walks [`WorldSession::attach_offboard`]
//! (actuators granted, Takeoff not yet fired) and [`WorldSession::attach_disarm`]
//! back to Ready. DISARM after failsafe is Protocol on `attach_disarm`
//! (`CanDisarm` stops at Landing), so the plant then walks
//! [`WorldSession::attach_recover_ready`] (Failsafe → Recovery → Ready).
//! `MAV_CMD_NAV_TAKEOFF` consumes Offboard through
//! [`Vehicle::start_takeoff_now`](flight_core::vehicle::Vehicle::start_takeoff_now)
//! via [`WorldSession::attach_start_takeoff`].
//! `MAV_CMD_NAV_LOITER_UNLIM` walks [`WorldSession::attach_airborne`]
//! (Takeoff → Airborne; Ready or already-airborne is Protocol).
//! `MAV_CMD_NAV_LAND` walks [`WorldSession::attach_land`].
//! DISARM from Landing is [`WorldSession::attach_disarm`] back to Ready
//! (`CanDisarm` includes Landing; this is not the Failsafe recover path).
//! `MAV_CMD_DO_FLIGHTTERMINATION` walks [`WorldSession::attach_failsafe`]
//! (`param1 >= 0.5`; inactive param is [`BackendError::Rejected`]).
//! `SET_POSITION_TARGET_LOCAL_NED` is gated on an Offboard / Takeoff /
//! Airborne / Landing attach, then [`WorldBackend::set_velocity_now`] or
//! [`WorldBackend::set_position_now`] (velocity mask wins when both are live).
//! [`WorldPlant::hold`] walks [`WorldSession::attach_hold`] (Ready before ARM
//! is Protocol; a later velocity `SET_POSITION_TARGET_LOCAL_NED` clears the hold).
//! [`WorldPlant::coastal`], [`WorldPlant::harbor`], [`WorldPlant::inland`],
//! and [`WorldPlant::open_water`] sit that same drone in every catalog.

use flight_core::vector::{Position, Velocity};
use flight_core::vehicle::{BackendError, VehicleHandle};
use flight_mavlink::{
    local_position_ned, ned_position_from_target, ned_velocity_from_target, px4_custom_mode,
    px4_vehicle_heartbeat, PX4_MAIN_MODE_OFFBOARD,
};
use flight_sim::WorldBackend;
use mavlink::common::{MavCmd, MavMessage};
use robot_world::World;

/// PX4-shaped plant whose physics is [`WorldBackend`].
#[derive(Clone, Debug)]
pub struct WorldPlant {
    backend: WorldBackend,
    body_id: &'static str,
    boot_ms: u32,
    armed: bool,
    offboard: bool,
}

impl WorldPlant {
    pub fn coastal(seed: u64) -> Self {
        Self::from_backend(WorldBackend::coastal(seed), "drone")
    }

    /// Inland drone over land. No hull in the scene.
    pub fn inland(seed: u64) -> Self {
        Self::from_backend(WorldBackend::inland(seed), "drone")
    }

    /// Harbor drone over the four-body shoreline.
    pub fn harbor(seed: u64) -> Self {
        Self::from_backend(WorldBackend::harbor(seed), "drone")
    }

    /// Open-water drone over swell. No rover in the scene.
    pub fn open_water(seed: u64) -> Self {
        Self::from_backend(WorldBackend::open_water(seed), "drone")
    }

    pub fn from_backend(backend: WorldBackend, body_id: &'static str) -> Self {
        Self {
            backend,
            body_id,
            boot_ms: 0,
            armed: false,
            offboard: false,
        }
    }

    pub fn backend(&self) -> &WorldBackend {
        &self.backend
    }

    pub fn world(&self) -> World {
        self.backend.world()
    }

    /// Apply one MAVLink message the companion (or PX4) would send.
    pub fn apply_mavlink(&mut self, msg: &MavMessage) -> Result<(), BackendError> {
        match msg {
            MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) => {
                if let Some((vn, ve, vd)) = ned_velocity_from_target(d) {
                    self.set_ned_velocity(vn, ve, vd)?;
                } else if let Some((n, e, down)) = ned_position_from_target(d) {
                    self.set_ned_position(n, e, down)?;
                }
            }
            MavMessage::COMMAND_LONG(c) => match c.command {
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                    if c.param1 >= 0.5 {
                        let session = self.backend.session().clone();
                        self.backend = session.attach_offboard(self.body_id)?;
                        self.armed = true;
                        self.offboard = true;
                    } else {
                        self.disarm_or_recover()?;
                    }
                }
                MavCmd::MAV_CMD_DO_SET_MODE => {
                    self.offboard = true;
                }
                MavCmd::MAV_CMD_NAV_TAKEOFF => self.nav_takeoff()?,
                MavCmd::MAV_CMD_NAV_LOITER_UNLIM => self.nav_loiter()?,
                MavCmd::MAV_CMD_NAV_LAND => {
                    let session = self.backend.session().clone();
                    self.backend = session.attach_land(self.body_id)?;
                }
                MavCmd::MAV_CMD_DO_FLIGHTTERMINATION => {
                    if c.param1 >= 0.5 {
                        let session = self.backend.session().clone();
                        self.backend = session.attach_failsafe(self.body_id)?;
                        self.offboard = false;
                    } else {
                        return Err(BackendError::Rejected("flight termination inactive"));
                    }
                }
                _ => {}
            },
            _ => {}
        }
        Ok(())
    }

    /// Ready through Landing: [`WorldSession::attach_disarm`]. Failsafe or
    /// Recovery: [`WorldSession::attach_recover_ready`]. Already-Ready DISARM
    /// still succeeds (`CanDisarm` includes Ready). Other Protocol is a no-op.
    fn disarm_or_recover(&mut self) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        match session.attach_disarm(self.body_id) {
            Ok(backend) => self.backend = backend,
            Err(BackendError::Protocol) => match session.attach_recover_ready(self.body_id) {
                Ok(backend) => self.backend = backend,
                Err(BackendError::Protocol) => {}
                Err(e) => return Err(e),
            },
            Err(e) => return Err(e),
        }
        self.armed = false;
        self.offboard = false;
        Ok(())
    }

    /// Offboard → Takeoff. Ready or already climbing is [`BackendError::Protocol`].
    fn nav_takeoff(&mut self) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        self.backend = session.attach_start_takeoff(self.body_id)?;
        Ok(())
    }

    /// Takeoff → Airborne. Ready, Offboard, Airborne, and Landing are Protocol.
    fn nav_loiter(&mut self) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        self.backend = session.attach_airborne(self.body_id)?;
        Ok(())
    }

    /// NED velocity only while attach is an offboard-control kind.
    fn set_ned_velocity(&mut self, vn: f32, ve: f32, vd: f32) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        match session.aerial(self.body_id).attach()? {
            VehicleHandle::Offboard(_)
            | VehicleHandle::Takeoff(_)
            | VehicleHandle::Airborne(_)
            | VehicleHandle::Landing(_) => self.backend.set_velocity_now(Velocity::ned(vn, ve, vd)),
            _ => Err(BackendError::Rejected("offboard setpoint")),
        }
    }

    /// NED position only while attach is an offboard-control kind.
    fn set_ned_position(&mut self, n: f32, e: f32, down: f32) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        match session.aerial(self.body_id).attach()? {
            VehicleHandle::Offboard(_)
            | VehicleHandle::Takeoff(_)
            | VehicleHandle::Airborne(_)
            | VehicleHandle::Landing(_) => self.backend.set_position_now(Position::ned(n, e, down)),
            _ => Err(BackendError::Rejected("offboard setpoint")),
        }
    }

    /// Hold the drone at its current NED pose through
    /// [`flight_sim::WorldSession::attach_hold`]. OffboardControl only;
    /// Ready before ARM is [`BackendError::Protocol`].
    pub fn hold(&mut self) -> Result<(), BackendError> {
        let session = self.backend.session().clone();
        self.backend = session.attach_hold(self.body_id)?;
        Ok(())
    }

    /// Step the verified world and return the pose PX4 already consumes.
    pub fn tick(&mut self, dt_secs: f32) -> Result<MavMessage, BackendError> {
        self.boot_ms = self
            .boot_ms
            .wrapping_add((dt_secs.max(0.0) * 1000.0) as u32);
        self.backend.step_now(dt_secs)?;
        let world = self.backend.world();
        let body = world.body(self.body_id).ok_or(BackendError::Disconnected)?;
        Ok(local_position_ned(
            self.boot_ms,
            body.position_m[0],
            body.position_m[1],
            body.position_m[2],
            body.velocity_mps[0],
            body.velocity_mps[1],
            body.velocity_mps[2],
        ))
    }

    pub fn heartbeat(&self) -> MavMessage {
        px4_vehicle_heartbeat(self.armed, px4_custom_mode(PX4_MAIN_MODE_OFFBOARD, 0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::safety::Phase;
    use flight_mavlink::{
        arm_disarm, flight_termination, nav_land, nav_loiter_unlim, nav_takeoff, set_offboard_mode,
        set_position_ned, set_velocity_ned,
    };

    #[test]
    fn offboard_setpoint_climbs_in_verified_world() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).expect("arm");
        plant
            .apply_mavlink(&set_offboard_mode(1, 1, true))
            .expect("offboard");
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Armed
        );
        let climb = set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2);
        plant.apply_mavlink(&climb).expect("setpoint");
        for _ in 0..250 {
            let msg = plant.tick(0.02).expect("tick");
            assert!(
                plant.world().all_hold(),
                "{:?}",
                plant.world().last_properties
            );
            let MavMessage::LOCAL_POSITION_NED(_) = msg else {
                panic!("expected LOCAL_POSITION_NED");
            };
        }
        let alt = plant.world().body("drone").unwrap().altitude_agl();
        assert!(alt > 3.0, "alt {alt}");
        assert!(matches!(plant.heartbeat(), MavMessage::HEARTBEAT(_)));
    }

    #[test]
    fn companion_mask_roundtrips_through_the_plant() {
        let msg = set_velocity_ned(1, 1, 20, 0.5, -0.25, 0.0);
        let MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) = &msg else {
            panic!("constructor");
        };
        assert_eq!(ned_velocity_from_target(d), Some((0.5, -0.25, 0.0)));
        let mut plant = WorldPlant::coastal(2);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&msg).unwrap();
        plant.tick(0.02).unwrap();
        let v = plant.world().body("drone").unwrap().command.unwrap();
        assert!((v[0] - 0.5).abs() < 1e-6);
        assert!((v[1] + 0.25).abs() < 1e-6);
    }

    #[test]
    fn companion_position_mask_roundtrips_through_the_plant() {
        let msg = set_position_ned(1, 1, 20, 0.0, 0.0, -4.0);
        let MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) = &msg else {
            panic!("constructor");
        };
        assert_eq!(ned_position_from_target(d), Some((0.0, 0.0, -4.0)));
        assert_eq!(ned_velocity_from_target(d), None);
        let mut plant = WorldPlant::coastal(2);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&msg).unwrap();
        plant.tick(0.02).unwrap();
        assert!(plant.world().body("drone").unwrap().command.is_some());
        assert!(plant.world().all_hold());
    }

    #[test]
    fn position_before_arm_is_rejected() {
        let mut plant = WorldPlant::coastal(1);
        let pose = set_position_ned(1, 1, 0, 0.0, 0.0, -4.0);
        assert!(matches!(
            plant.apply_mavlink(&pose),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
        assert!(plant.world().body("drone").unwrap().command.is_none());
    }

    #[test]
    fn hold_sets_ned_pose_and_velocity_clears_it() {
        for mut plant in [
            WorldPlant::coastal(1),
            WorldPlant::harbor(1),
            WorldPlant::inland(1),
            WorldPlant::open_water(1),
        ] {
            let name = plant.world().scenario;
            plant.apply_mavlink(&arm_disarm(1, 1, true)).expect(name);
            plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).expect(name);
            plant.hold().expect(name);
            let pose = plant.world().body("drone").unwrap().position_m;
            assert_eq!(
                plant.world().body("drone").unwrap().hold_ned,
                Some(pose),
                "{name}"
            );
            plant.tick(0.02).expect(name);
            assert!(
                plant.world().body("drone").unwrap().hold_ned.is_some(),
                "{name}"
            );
            assert!(plant.world().all_hold(), "{name}");
            plant
                .apply_mavlink(&set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2))
                .expect(name);
            plant.tick(0.02).expect(name);
            assert!(
                plant.world().body("drone").unwrap().hold_ned.is_none(),
                "{name} live velocity must win"
            );
            assert!(plant.world().all_hold(), "{name}");
        }
    }

    #[test]
    fn hold_before_arm_is_protocol() {
        let mut plant = WorldPlant::coastal(1);
        assert!(matches!(plant.hold(), Err(BackendError::Protocol)));
        assert!(plant.world().body("drone").unwrap().hold_ned.is_none());
    }

    #[test]
    fn velocity_before_arm_is_rejected() {
        let mut plant = WorldPlant::coastal(1);
        let climb = set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2);
        assert!(matches!(
            plant.apply_mavlink(&climb),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
        assert!(plant.world().body("drone").unwrap().command.is_none());
    }

    #[test]
    fn velocity_after_nav_takeoff_commands_takeoff_kind() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("expected Takeoff, got {:?}", other.kind()),
        }
        plant
            .apply_mavlink(&set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2))
            .unwrap();
        plant.tick(0.02).unwrap();
        let v = plant.world().body("drone").unwrap().command.unwrap();
        assert!((v[2] + 1.2).abs() < 1e-6);
        assert!(plant.world().all_hold());
    }

    #[test]
    fn land_is_rejected_until_nav_takeoff() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        assert!(plant.apply_mavlink(&nav_land(1, 1)).is_err());
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Armed
        );
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        plant.apply_mavlink(&nav_land(1, 1)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Landing
        );
        assert!(plant.world().all_hold());
    }

    #[test]
    fn disarm_after_nav_land_returns_ready() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        plant.apply_mavlink(&nav_land(1, 1)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Landing(_) => {}
            other => panic!("NAV_LAND must attach Landing, got {:?}", other.kind()),
        }
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!(
                "DISARM from Landing must attach Ready (CanDisarm), got {:?}",
                other.kind()
            ),
        }
        let aerial = plant.world().body("drone").unwrap().aerial.unwrap();
        assert_eq!(aerial.phase, Phase::Ready);
        assert!(!aerial.armed && !aerial.failsafe && !aerial.actuators_enabled);
        assert!(plant.world().body("drone").unwrap().command.is_none());
        assert!(plant.world().all_hold());
    }

    #[test]
    fn nav_takeoff_before_arm_is_protocol() {
        let mut plant = WorldPlant::coastal(1);
        assert!(matches!(
            plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)),
            Err(BackendError::Protocol)
        ));
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Ready
        );
    }

    #[test]
    fn nav_takeoff_walks_offboard_start_takeoff() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Offboard(_) => {}
            other => panic!("arm must attach Offboard, got {:?}", other.kind()),
        }
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("NAV_TAKEOFF must attach Takeoff, got {:?}", other.kind()),
        }
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Takeoff
        );
        assert!(matches!(
            plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)),
            Err(BackendError::Protocol)
        ));
    }

    #[test]
    fn nav_loiter_walks_attach_airborne() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        assert!(matches!(
            plant.apply_mavlink(&nav_loiter_unlim(1, 1)),
            Err(BackendError::Protocol)
        ));
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        plant.apply_mavlink(&nav_loiter_unlim(1, 1)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Airborne(_) => {}
            other => panic!(
                "NAV_LOITER_UNLIM must attach Airborne, got {:?}",
                other.kind()
            ),
        }
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Airborne
        );
        assert!(matches!(
            plant.apply_mavlink(&nav_loiter_unlim(1, 1)),
            Err(BackendError::Protocol)
        ));
        plant.apply_mavlink(&nav_land(1, 1)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Landing
        );
        assert!(plant.world().all_hold());
    }

    #[test]
    fn flight_termination_walks_attach_failsafe() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("expected Takeoff, got {:?}", other.kind()),
        }
        plant
            .apply_mavlink(&flight_termination(1, 1, true))
            .unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!(
                "DO_FLIGHTTERMINATION must attach Failsafe, got {:?}",
                other.kind()
            ),
        }
        assert!(
            plant
                .world()
                .body("drone")
                .unwrap()
                .aerial
                .unwrap()
                .failsafe
        );
        assert!(plant.world().all_hold());
        assert!(matches!(
            plant.apply_mavlink(&flight_termination(1, 1, true)),
            Err(BackendError::Protocol)
        ));
        assert!(matches!(
            plant.apply_mavlink(&set_velocity_ned(1, 1, 0, 0.0, 0.0, -1.2)),
            Err(BackendError::Rejected("offboard setpoint"))
        ));
    }

    #[test]
    fn flight_termination_from_ready_trips_without_arm() {
        let mut plant = WorldPlant::coastal(1);
        plant
            .apply_mavlink(&flight_termination(1, 1, true))
            .unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("expected Failsafe from Ready, got {:?}", other.kind()),
        }
        assert!(matches!(
            plant.apply_mavlink(&flight_termination(1, 1, false)),
            Err(BackendError::Rejected("flight termination inactive"))
        ));
    }

    #[test]
    fn disarm_after_flight_termination_recovers_ready() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        plant
            .apply_mavlink(&flight_termination(1, 1, true))
            .unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("expected Failsafe, got {:?}", other.kind()),
        }
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!(
                "DISARM after DO_FLIGHTTERMINATION must attach Ready, got {:?}",
                other.kind()
            ),
        }
        let aerial = plant.world().body("drone").unwrap().aerial.unwrap();
        assert_eq!(aerial.phase, Phase::Ready);
        assert!(!aerial.armed && !aerial.failsafe && !aerial.actuators_enabled);
        assert!(plant.world().all_hold());
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Ready
        );
    }

    #[test]
    fn disarm_walks_attach_typestate_back_to_ready() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Armed
        );
        assert!(plant.world().body("drone").unwrap().aerial.unwrap().armed);
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        let aerial = plant.world().body("drone").unwrap().aerial.unwrap();
        assert_eq!(aerial.phase, Phase::Ready);
        assert!(!aerial.armed && !aerial.actuators_enabled && !aerial.offboard);
        assert!(plant.world().body("drone").unwrap().command.is_none());
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Ready
        );
        assert!(plant.world().all_hold());
    }

    #[test]
    fn disarm_after_arm_termination_recovers_from_offboard() {
        let mut plant = WorldPlant::coastal(1);
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Offboard(_) => {}
            other => panic!("arm must attach Offboard, got {:?}", other.kind()),
        }
        plant
            .apply_mavlink(&flight_termination(1, 1, true))
            .unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("expected Failsafe from Offboard, got {:?}", other.kind()),
        }
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!(
                "DISARM after Offboard termination must attach Ready, got {:?}",
                other.kind()
            ),
        }
        assert!(
            !plant
                .world()
                .body("drone")
                .unwrap()
                .aerial
                .unwrap()
                .failsafe
        );
        assert!(plant.world().all_hold());
    }

    #[test]
    fn arm_after_termination_disarm_walks_offboard() {
        let mut plant = WorldPlant::coastal(1);
        plant
            .apply_mavlink(&flight_termination(1, 1, true))
            .unwrap();
        plant.apply_mavlink(&arm_disarm(1, 1, false)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!(
                "expected Ready after recover DISARM, got {:?}",
                other.kind()
            ),
        }
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Offboard(_) => {}
            other => panic!(
                "ARM after recover must attach Offboard, got {:?}",
                other.kind()
            ),
        }
        assert_eq!(
            plant.world().body("drone").unwrap().aerial.unwrap().phase,
            Phase::Armed
        );
        assert!(plant.world().body("drone").unwrap().aerial.unwrap().armed);
        assert!(plant.world().all_hold());
    }

    #[test]
    fn inland_plant_arms_without_hulls() {
        let mut plant = WorldPlant::inland(1);
        assert_eq!(plant.world().scenario, "inland");
        assert!(plant.world().body("skiff").is_none());
        assert!(plant.world().body("surveyor").is_none());
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("expected Takeoff, got {:?}", other.kind()),
        }
        plant.tick(0.02).unwrap();
        assert!(plant.world().all_hold());
        assert!(plant.world().body("rover").is_some());
    }

    #[test]
    fn harbor_plant_arms_the_four_body_shoreline() {
        let mut plant = WorldPlant::harbor(1);
        assert_eq!(plant.world().scenario, "harbor");
        assert!(plant.world().body("rover").is_some());
        assert!(plant.world().body("skiff").is_some());
        assert!(plant.world().body("surveyor").is_some());
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("expected Takeoff, got {:?}", other.kind()),
        }
        plant.tick(0.02).unwrap();
        assert!(plant.world().all_hold());
    }

    #[test]
    fn open_water_plant_arms_without_a_rover() {
        let mut plant = WorldPlant::open_water(1);
        assert_eq!(plant.world().scenario, "open_water");
        assert!(plant.world().body("rover").is_none());
        assert!(plant.world().body("skiff").is_some());
        plant.apply_mavlink(&arm_disarm(1, 1, true)).unwrap();
        plant.apply_mavlink(&nav_takeoff(1, 1, 5.0)).unwrap();
        match plant.backend().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("expected Takeoff, got {:?}", other.kind()),
        }
        plant.tick(0.02).unwrap();
        assert!(plant.world().all_hold());
    }
}
