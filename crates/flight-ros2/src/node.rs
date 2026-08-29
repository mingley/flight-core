//! Production `rclrs` node: publish or subscribe `geometry_msgs/msg/Twist` (ENU)
//! against a verified [`flight_sim::WorldSession`].

use crate::geometry::Twist;
use crate::plant::{FleetPlant, FleetTwist};
use crate::{ExternalFlightMode, OffboardSetpoint};
use flight_core::frames::Ned;
use flight_core::vector::Velocity;
use flight_core::vehicle::BackendError;
use flight_sim::{WorldBackend, WorldSession};
use rclrs::{
    Context, CreateBasicExecutor, Executor, InitOptions, Node, Publisher, RclrsError, SpinOptions,
    Subscription,
};
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub use rclrs::RclrsError as RosError;

/// Companion-computer offboard publisher on a real ROS 2 graph.
pub struct OffboardNode {
    executor: Executor,
    node: Node,
    publisher: Publisher<Twist>,
    topic: String,
}

impl OffboardNode {
    pub fn new(name: &str, topic: &str) -> Result<Self, RclrsError> {
        Self::with_domain(name, topic, None)
    }

    pub fn with_domain(
        name: &str,
        topic: &str,
        domain_id: Option<usize>,
    ) -> Result<Self, RclrsError> {
        let context = Context::new(
            ["flight-ros2".to_string()],
            InitOptions::new().with_domain_id(domain_id),
        )?;
        let executor = context.create_basic_executor();
        let node = executor.create_node(name)?;
        let publisher = node.create_publisher::<Twist>(topic)?;
        Ok(Self {
            executor,
            node,
            publisher,
            topic: topic.into(),
        })
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }

    pub fn name(&self) -> String {
        self.node.name()
    }

    pub fn publish_setpoint(&self, sp: &OffboardSetpoint) -> Result<(), RclrsError> {
        let twist = match sp.velocity_ned {
            Some(v) => Twist::from_ned_velocity(v),
            None => Twist::default(),
        };
        self.publisher.publish(twist)
    }

    pub fn publish_velocity(&self, v: Velocity<Ned>) -> Result<(), RclrsError> {
        self.publisher.publish(Twist::from_ned_velocity(v))
    }

    pub fn publish_mode(
        &self,
        mode: &mut impl ExternalFlightMode<Setpoint = OffboardSetpoint>,
        dt_secs: f32,
    ) -> Result<(), RclrsError> {
        self.publish_setpoint(&mode.update(dt_secs))
    }

    pub fn spin_once(&mut self, timeout: Duration) -> Vec<RclrsError> {
        self.executor
            .spin(SpinOptions::spin_once().timeout(timeout))
    }
}

/// Subscribe to ENU Twist and apply it to a verified coastal drone plant.
pub struct PlantNode {
    executor: Executor,
    node: Node,
    session: WorldSession,
    drone: WorldBackend,
    latest: Arc<Mutex<Option<[f64; 3]>>>,
    _sub: Subscription<Twist>,
}

impl PlantNode {
    pub fn coastal(name: &str, topic: &str, seed: u64) -> Result<Self, RclrsError> {
        Self::with_domain(name, topic, seed, None)
    }

    pub fn with_domain(
        name: &str,
        topic: &str,
        seed: u64,
        domain_id: Option<usize>,
    ) -> Result<Self, RclrsError> {
        let context = Context::new(
            ["flight-ros2-plant".to_string()],
            InitOptions::new().with_domain_id(domain_id),
        )?;
        let executor = context.create_basic_executor();
        let node = executor.create_node(name)?;
        let session = WorldSession::coastal(seed);
        let drone = session.aerial("drone");
        let latest = Arc::new(Mutex::new(None));
        let slot = Arc::clone(&latest);
        let sub = node.create_subscription(topic, move |msg: Twist| {
            *slot.lock().expect("twist slot") = Some([msg.linear.x, msg.linear.y, msg.linear.z]);
        })?;
        Ok(Self {
            executor,
            node,
            session,
            drone,
            latest,
            _sub: sub,
        })
    }

    pub fn name(&self) -> String {
        self.node.name()
    }

    pub fn node(&self) -> &Node {
        &self.node
    }

    pub fn session(&self) -> &WorldSession {
        &self.session
    }

    pub fn grant_offboard(&mut self) -> Result<(), BackendError> {
        self.drone = self.session.attach_takeoff("drone")?;
        Ok(())
    }

    /// Trip aerial failsafe through [`crate::plant::apply_failsafe`].
    pub fn trip_failsafe(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_failsafe(&mut self.drone)
    }

    /// Recover Ready through [`crate::plant::apply_recover_ready`].
    pub fn recover_ready(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_recover_ready(&mut self.drone)
    }

    /// Disarm to Ready through [`crate::plant::apply_disarm`].
    pub fn disarm(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_disarm(&mut self.drone)
    }

    /// Enter landing through [`crate::plant::apply_land`].
    pub fn land(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_land(&mut self.drone)
    }

    /// Touch down through [`crate::plant::apply_touchdown`].
    pub fn touchdown(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_touchdown(&mut self.drone)
    }

    /// Takeoff → Airborne through [`crate::plant::apply_airborne`].
    pub fn airborne(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_airborne(&mut self.drone)
    }

    /// Hold the drone at its current NED pose through [`crate::plant::apply_hold`].
    pub fn hold(&mut self) -> Result<(), BackendError> {
        crate::plant::apply_hold(&mut self.drone)
    }

    /// Drain the ROS 2 graph, apply the latest Twist, take one verified step.
    pub fn spin_step(&mut self, dt: f32, timeout: Duration) -> Result<(), BackendError> {
        let _ = self
            .executor
            .spin(SpinOptions::spin_once().timeout(timeout));
        if let Some(lin) = *self.latest.lock().expect("twist") {
            crate::plant::apply_twist_linear(&mut self.drone, lin)?;
        }
        self.session.step(dt)
    }
}

/// Topic names for [`FleetPlantNode`] (REP-103 Twist, one per catalog body).
/// Inland ignores hull topics; open water ignores the rover topic.
#[derive(Clone, Copy, Debug)]
pub struct FleetTopics<'a> {
    pub drone: &'a str,
    pub rover: &'a str,
    pub skiff: &'a str,
    pub surveyor: &'a str,
}

impl FleetTopics<'static> {
    pub const COASTAL: Self = Self {
        drone: "/drone/cmd_vel",
        rover: "/rover/cmd_vel",
        skiff: "/skiff/cmd_vel",
        surveyor: "/surveyor/cmd_vel",
    };
}

/// Subscribe ENU Twist per platform and step one verified catalog fleet.
pub struct FleetPlantNode {
    executor: Executor,
    node: Node,
    plant: FleetPlant,
    latest: Arc<Mutex<FleetTwist>>,
    drone_topic: String,
    rover_topic: String,
    skiff_topic: String,
    surveyor_topic: String,
    _drone: Subscription<Twist>,
    _rover: Subscription<Twist>,
    _skiff: Subscription<Twist>,
    _surveyor: Subscription<Twist>,
}

impl FleetPlantNode {
    pub fn coastal(name: &str, seed: u64) -> Result<Self, RclrsError> {
        Self::with_domain(name, seed, None)
    }

    /// Harbor shoreline fleet on the same Twist topics as coastal.
    pub fn harbor(name: &str, seed: u64) -> Result<Self, RclrsError> {
        Self::with_plant(name, None, FleetTopics::COASTAL, FleetPlant::harbor(seed))
    }

    /// Inland air + ground. Hull Twists are ignored.
    pub fn inland(name: &str, seed: u64) -> Result<Self, RclrsError> {
        Self::with_plant(name, None, FleetTopics::COASTAL, FleetPlant::inland(seed))
    }

    /// Open water air + hulls. Rover Twists are ignored.
    pub fn open_water(name: &str, seed: u64) -> Result<Self, RclrsError> {
        Self::with_plant(
            name,
            None,
            FleetTopics::COASTAL,
            FleetPlant::open_water(seed),
        )
    }

    pub fn with_domain(
        name: &str,
        seed: u64,
        domain_id: Option<usize>,
    ) -> Result<Self, RclrsError> {
        Self::with_topics(name, seed, domain_id, FleetTopics::COASTAL)
    }

    pub fn with_topics(
        name: &str,
        seed: u64,
        domain_id: Option<usize>,
        topics: FleetTopics<'_>,
    ) -> Result<Self, RclrsError> {
        Self::with_plant(name, domain_id, topics, FleetPlant::coastal(seed))
    }

    /// Same Twist subscriptions as coastal, over an arbitrary [`FleetPlant`].
    pub fn with_plant(
        name: &str,
        domain_id: Option<usize>,
        topics: FleetTopics<'_>,
        plant: FleetPlant,
    ) -> Result<Self, RclrsError> {
        let context = Context::new(
            ["flight-ros2-fleet".to_string()],
            InitOptions::new().with_domain_id(domain_id),
        )?;
        let executor = context.create_basic_executor();
        let node = executor.create_node(name)?;
        let latest = Arc::new(Mutex::new(FleetTwist::default()));
        let drone_slot = Arc::clone(&latest);
        let rover_slot = Arc::clone(&latest);
        let skiff_slot = Arc::clone(&latest);
        let surveyor_slot = Arc::clone(&latest);
        let drone_sub = node.create_subscription(topics.drone, move |msg: Twist| {
            drone_slot.lock().expect("twist").drone =
                Some([msg.linear.x, msg.linear.y, msg.linear.z]);
        })?;
        let rover_sub = node.create_subscription(topics.rover, move |msg: Twist| {
            rover_slot.lock().expect("twist").rover =
                Some([msg.linear.x, msg.linear.y, msg.linear.z]);
        })?;
        let skiff_sub = node.create_subscription(topics.skiff, move |msg: Twist| {
            skiff_slot.lock().expect("twist").skiff =
                Some([msg.linear.x, msg.linear.y, msg.linear.z]);
        })?;
        let surveyor_sub = node.create_subscription(topics.surveyor, move |msg: Twist| {
            surveyor_slot.lock().expect("twist").surveyor =
                Some([msg.linear.x, msg.linear.y, msg.linear.z]);
        })?;
        Ok(Self {
            executor,
            node,
            plant,
            latest,
            drone_topic: topics.drone.into(),
            rover_topic: topics.rover.into(),
            skiff_topic: topics.skiff.into(),
            surveyor_topic: topics.surveyor.into(),
            _drone: drone_sub,
            _rover: rover_sub,
            _skiff: skiff_sub,
            _surveyor: surveyor_sub,
        })
    }

    pub fn name(&self) -> String {
        self.node.name()
    }

    pub fn node(&self) -> &Node {
        &self.node
    }

    pub fn topics(&self) -> [&str; 4] {
        [
            &self.drone_topic,
            &self.rover_topic,
            &self.skiff_topic,
            &self.surveyor_topic,
        ]
    }

    pub fn plant(&self) -> &FleetPlant {
        &self.plant
    }

    pub fn plant_mut(&mut self) -> &mut FleetPlant {
        &mut self.plant
    }

    pub fn grant_all(&mut self) -> Result<(), BackendError> {
        self.plant.grant_all()
    }

    /// Trip every live catalog body through [`FleetPlant::trip_safety`].
    pub fn trip_safety(&mut self) -> Result<(), BackendError> {
        self.plant.trip_safety()
    }

    /// Recover every live catalog body through [`FleetPlant::recover_safety`].
    pub fn recover_safety(&mut self) -> Result<(), BackendError> {
        self.plant.recover_safety()
    }

    /// Land, park, and dock every live catalog body through [`FleetPlant::return_all`].
    pub fn return_all(&mut self) -> Result<(), BackendError> {
        self.plant.return_all()
    }

    /// Takeoff → Airborne through [`FleetPlant::airborne`].
    pub fn airborne(&mut self) -> Result<(), BackendError> {
        self.plant.airborne()
    }

    /// Hold station on catalog hulls through [`FleetPlant::station_all`].
    pub fn station_all(&mut self) -> Result<(), BackendError> {
        self.plant.station_all()
    }

    /// Resume Underway on catalog hulls through [`FleetPlant::resume_all`].
    pub fn resume_all(&mut self) -> Result<(), BackendError> {
        self.plant.resume_all()
    }

    /// Dock catalog hulls through [`FleetPlant::dock_all`].
    pub fn dock_all(&mut self) -> Result<(), BackendError> {
        self.plant.dock_all()
    }

    /// Halt the rover through [`FleetPlant::park_all`].
    pub fn park_all(&mut self) -> Result<(), BackendError> {
        self.plant.park_all()
    }

    /// Hold the drone at its current NED pose through [`FleetPlant::hold`].
    pub fn hold(&mut self) -> Result<(), BackendError> {
        self.plant.hold()
    }

    /// Drain the ROS 2 graph, apply the latest Twists, take one verified step.
    pub fn spin_step(&mut self, dt: f32, timeout: Duration) -> Result<(), BackendError> {
        let _ = self
            .executor
            .spin(SpinOptions::spin_once().timeout(timeout));
        let twist = *self.latest.lock().expect("twist");
        self.plant.apply_twists(twist)?;
        self.plant.step(dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VelocityMode;
    use rclrs::Context;
    use std::sync::{Arc, Mutex};

    #[test]
    fn rclrs_roundtrip_ned_velocity_as_enu_twist() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let topic = format!("/flight_core/cmd_vel_{pid}");
        let domain = 171usize;
        let context = Context::new(
            ["flight-ros2-test".to_string()],
            InitOptions::new().with_domain_id(Some(domain)),
        )
        .expect("rcl context");
        let mut executor = context.create_basic_executor();
        let pub_node = executor
            .create_node(&format!("fc_pub_{pid}"))
            .expect("pub node");
        let sub_node = executor
            .create_node(&format!("fc_sub_{pid}"))
            .expect("sub node");
        let publisher = pub_node
            .create_publisher::<Twist>(topic.as_str())
            .expect("publisher");
        let got = Arc::new(Mutex::new(None));
        let slot = Arc::clone(&got);
        let _sub = sub_node
            .create_subscription(topic.as_str(), move |msg: Twist| {
                *slot.lock().expect("twist slot") = Some(msg);
            })
            .expect("subscription");

        let mut mode = VelocityMode::new(Velocity::<Ned>::ned(0.4, 1.2, -0.3));
        mode.on_activate();
        let twist = Twist::from_ned_velocity(mode.update(0.02).velocity_ned.unwrap());
        publisher.publish(twist).expect("publish");

        let mut seen = None;
        for _ in 0..40 {
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(50)));
            if let Some(msg) = *got.lock().expect("lock") {
                seen = Some(msg);
                break;
            }
        }
        let msg = seen.expect("did not receive Twist on the ROS 2 graph");
        assert!((msg.linear.x - 1.2).abs() < 1e-6, "east {}", msg.linear.x);
        assert!((msg.linear.y - 0.4).abs() < 1e-6, "north {}", msg.linear.y);
        assert!((msg.linear.z - 0.3).abs() < 1e-6, "up {}", msg.linear.z);
    }

    #[test]
    fn offboard_node_publishes_velocity_mode() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut node = OffboardNode::with_domain(
            &format!("fc_smoke_{pid}"),
            &format!("/flight_core/smoke_{pid}"),
            Some(172),
        )
        .expect("offboard node");
        let mut mode = VelocityMode::new(Velocity::<Ned>::ned(0.1, 0.0, 0.0));
        mode.on_activate();
        node.publish_mode(&mut mode, 0.02).expect("publish");
        let _ = node.spin_once(Duration::from_millis(10));
        assert_eq!(node.name(), format!("fc_smoke_{pid}"));
    }

    #[test]
    fn plant_node_climbs_from_enu_twist() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let topic = format!("/flight_core/plant_{pid}");
        let domain = 173usize;
        let mut plant = PlantNode::with_domain(&format!("fc_plant_{pid}"), &topic, 1, Some(domain))
            .expect("plant node");
        plant.grant_offboard().expect("grant");
        let publisher = plant
            .node()
            .create_publisher::<Twist>(topic.as_str())
            .expect("publisher");
        let climb = Twist::from_ned_velocity(Velocity::<Ned>::ned(0.0, 0.0, -1.2));

        let alt0 = plant
            .session()
            .world()
            .body("drone")
            .unwrap()
            .altitude_agl();
        for _ in 0..50 {
            publisher.publish(climb).expect("publish climb");
            plant
                .spin_step(0.02, Duration::from_millis(20))
                .expect("spin step");
        }
        let world = plant.session().world();
        let alt1 = world.body("drone").unwrap().altitude_agl();
        assert!(alt1 > alt0 + 0.15, "ROS Twist plant climb {alt0} → {alt1}");
        assert!(world.all_hold(), "{:?}", world.last_properties);
        assert_eq!(plant.name(), format!("fc_plant_{pid}"));
    }

    #[test]
    fn plant_node_trip_failsafe_then_recover_ready() {
        use flight_core::vehicle::VehicleHandle;

        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = PlantNode::with_domain(
            &format!("fc_plant_fs_{pid}"),
            &format!("/flight_core/plant_fs_{pid}"),
            1,
            Some(175),
        )
        .expect("plant node");
        plant.trip_failsafe().expect("trip");
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("expected Failsafe, got {:?}", other.kind()),
        }
        plant.recover_ready().expect("recover");
        match plant.session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("expected Ready, got {:?}", other.kind()),
        }
        assert!(matches!(plant.recover_ready(), Err(BackendError::Protocol)));
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn fleet_plant_node_moves_four_domains() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let drone_topic = format!("/flight_core/fleet_drone_{pid}");
        let rover_topic = format!("/flight_core/fleet_rover_{pid}");
        let skiff_topic = format!("/flight_core/fleet_skiff_{pid}");
        let surveyor_topic = format!("/flight_core/fleet_surveyor_{pid}");
        let mut plant = FleetPlantNode::with_topics(
            &format!("fc_fleet_{pid}"),
            1,
            Some(174),
            FleetTopics {
                drone: &drone_topic,
                rover: &rover_topic,
                skiff: &skiff_topic,
                surveyor: &surveyor_topic,
            },
        )
        .expect("fleet plant node");
        plant.grant_all().expect("grant");
        let pub_drone = plant
            .node()
            .create_publisher::<Twist>(drone_topic.as_str())
            .expect("drone pub");
        let pub_rover = plant
            .node()
            .create_publisher::<Twist>(rover_topic.as_str())
            .expect("rover pub");
        let pub_skiff = plant
            .node()
            .create_publisher::<Twist>(skiff_topic.as_str())
            .expect("skiff pub");
        let pub_surveyor = plant
            .node()
            .create_publisher::<Twist>(surveyor_topic.as_str())
            .expect("surveyor pub");
        let climb = Twist::from_ned_velocity(Velocity::<Ned>::ned(0.0, 0.0, -1.2));
        let south = Twist::from_ned_velocity(Velocity::<Ned>::ned(-0.8, 0.0, 0.0));
        let east = Twist::from_ned_velocity(Velocity::<Ned>::ned(0.0, 0.6, 0.0));
        let north = Twist::from_ned_velocity(Velocity::<Ned>::ned(0.4, 0.0, 0.0));

        let world0 = plant.plant().session().world();
        let alt0 = world0.body("drone").unwrap().altitude_agl();
        let n0 = world0.body("rover").unwrap().position_m[0];
        let e0 = world0.body("skiff").unwrap().position_m[1];
        let sn0 = world0.body("surveyor").unwrap().position_m[0];
        for _ in 0..50 {
            pub_drone.publish(climb).expect("publish climb");
            pub_rover.publish(south).expect("publish south");
            pub_skiff.publish(east).expect("publish east");
            pub_surveyor.publish(north).expect("publish north");
            plant
                .spin_step(0.02, Duration::from_millis(20))
                .expect("spin step");
        }
        let world = plant.plant().session().world();
        let alt1 = world.body("drone").unwrap().altitude_agl();
        let n1 = world.body("rover").unwrap().position_m[0];
        let e1 = world.body("skiff").unwrap().position_m[1];
        let sn1 = world.body("surveyor").unwrap().position_m[0];
        assert!(alt1 > alt0 + 0.15, "ROS fleet drone {alt0} → {alt1}");
        assert!(n1 < n0 - 0.1, "ROS fleet rover {n0} → {n1}");
        assert!(e1 > e0 + 0.08, "ROS fleet skiff {e0} → {e1}");
        assert!(sn1 > sn0 + 0.1, "ROS fleet surveyor {sn0} → {sn1}");
        assert!(world.all_hold(), "{:?}", world.last_properties);
        assert_eq!(plant.name(), format!("fc_fleet_{pid}"));
        assert_eq!(
            plant.topics(),
            [
                drone_topic.as_str(),
                rover_topic.as_str(),
                skiff_topic.as_str(),
                surveyor_topic.as_str()
            ]
        );
    }

    #[test]
    fn fleet_plant_node_trip_then_recover_safety() {
        use flight_core::vehicle::{GroundHandle, MarineHandle, VehicleHandle};

        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = FleetPlantNode::with_domain(&format!("fc_fleet_fs_{pid}"), 1, Some(176))
            .expect("fleet plant node");
        plant.grant_all().expect("grant");
        plant.trip_safety().expect("trip");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Failsafe(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.plant().session().ground("rover").attach().unwrap() {
            GroundHandle::EStopped(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match plant.plant().session().marine("skiff").attach().unwrap() {
            MarineHandle::Failsafe(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        plant.recover_safety().expect("recover");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        assert!(matches!(
            plant.recover_safety(),
            Err(BackendError::Protocol)
        ));
        plant.grant_all().expect("re-grant");
        assert!(plant.plant().session().world().all_hold());
    }

    #[test]
    fn fleet_plant_node_inland_has_no_hull() {
        use flight_core::vehicle::{GroundHandle, VehicleHandle};

        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = FleetPlantNode::with_plant(
            &format!("fc_fleet_inland_{pid}"),
            Some(177),
            FleetTopics::COASTAL,
            FleetPlant::inland(1),
        )
        .expect("inland fleet node");
        assert!(plant.plant().session().world().body("skiff").is_none());
        plant.grant_all().expect("grant");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.plant().session().ground("rover").attach().unwrap() {
            GroundHandle::Moving(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        assert!(plant.plant().session().world().body("skiff").is_none());
        plant.return_all().expect("return");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::PreflightReady(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        assert!(plant.plant().session().world().all_hold());
    }

    #[test]
    fn fleet_plant_node_open_water_has_no_rover() {
        use flight_core::vehicle::{MarineHandle, VehicleHandle};

        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = FleetPlantNode::with_plant(
            &format!("fc_fleet_water_{pid}"),
            Some(178),
            FleetTopics::COASTAL,
            FleetPlant::open_water(1),
        )
        .expect("open water fleet node");
        assert!(plant.plant().session().world().body("rover").is_none());
        plant.grant_all().expect("grant");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.plant().session().marine("skiff").attach().unwrap() {
            MarineHandle::Underway(_) => {}
            other => panic!("skiff {:?}", other.kind()),
        }
        assert!(plant.plant().session().world().body("rover").is_none());
        plant.return_all().expect("return");
        match plant.plant().session().marine("surveyor").attach().unwrap() {
            MarineHandle::Docked(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(plant.plant().session().world().all_hold());
    }

    #[test]
    fn fleet_plant_node_harbor_grants_four_bodies() {
        use flight_core::vehicle::{GroundHandle, MarineHandle, VehicleHandle};

        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = FleetPlantNode::with_plant(
            &format!("fc_fleet_harbor_{pid}"),
            Some(179),
            FleetTopics::COASTAL,
            FleetPlant::harbor(1),
        )
        .expect("harbor fleet node");
        assert_eq!(plant.plant().session().world().scenario, "harbor");
        plant.grant_all().expect("grant");
        match plant.plant().session().aerial("drone").attach().unwrap() {
            VehicleHandle::Takeoff(_) => {}
            other => panic!("drone {:?}", other.kind()),
        }
        match plant.plant().session().ground("rover").attach().unwrap() {
            GroundHandle::Moving(_) => {}
            other => panic!("rover {:?}", other.kind()),
        }
        match plant.plant().session().marine("surveyor").attach().unwrap() {
            MarineHandle::Underway(_) => {}
            other => panic!("surveyor {:?}", other.kind()),
        }
        assert!(plant.plant().session().world().all_hold());
    }

    #[test]
    fn plant_node_hold_before_grant_is_protocol() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = PlantNode::with_domain(
            &format!("fc_plant_hold_{pid}"),
            &format!("/flight_core/plant_hold_{pid}"),
            1,
            Some(180),
        )
        .expect("plant node");
        assert!(matches!(plant.hold(), Err(BackendError::Protocol)));
        plant.grant_offboard().expect("grant");
        plant.hold().expect("hold after grant");
        let pose = plant.session().world().body("drone").unwrap().position_m;
        assert_eq!(
            plant.session().world().body("drone").unwrap().hold_ned,
            Some(pose)
        );
        assert!(plant.session().world().all_hold());
    }

    #[test]
    fn fleet_plant_node_hold_before_grant_is_protocol() {
        std::env::set_var("ROS_LOCALHOST_ONLY", "1");
        let pid = std::process::id();
        let mut plant = FleetPlantNode::with_domain(&format!("fc_fleet_hold_{pid}"), 1, Some(181))
            .expect("fleet plant node");
        assert!(matches!(plant.hold(), Err(BackendError::Protocol)));
        plant.grant_all().expect("grant");
        plant.hold().expect("hold after grant");
        let pose = plant
            .plant()
            .session()
            .world()
            .body("drone")
            .unwrap()
            .position_m;
        assert_eq!(
            plant
                .plant()
                .session()
                .world()
                .body("drone")
                .unwrap()
                .hold_ned,
            Some(pose)
        );
        assert!(plant.plant().session().world().all_hold());
    }
}
