//! ROS 2 node adapters that wrap `flight_core` simulation into `rclrs` publishers
//! and subscribers.
//!
//! Two node types are provided:
//!
//! * [`SimNode`] — full 6-DoF / 12-state vehicle simulator. Publishes `/odom` and
//!   `/imu`, subscribes to `/cmd_vel` and `/cmd_wrench`.
//! * [`PlantNode`] — SISO plant simulator. Publishes `/plant/state`,
//!   `/plant/output`, `/plant/step`, `/plant/y`, `/plant/u`; subscribes to
//!   `/cmd_force`. Designed to pair with `flight_px4::Px4PlantNode`.
//!
//! Both nodes run a simulation tick on a wall-clock timer whose period matches
//! the configured `dt`. Incoming ROS 2 messages are stored in a mutex and applied
//! at the start of the next tick.
//!
//! # QoS
//!
//! All topics use the ROS 2 default QoS profile (reliable, keep-last 10) unless
//! otherwise noted. Sensor topics (`/odom`, `/imu`) are published at the
//! simulation rate.
//!
//! # Coordinate frames
//!
//! * `/odom` uses `odom` as the parent frame and `base_link` as the child.
//! * `/imu` uses `base_link` as the frame.
//! * `/plant/state` and `/plant/output` use `plant` as the frame.
//!
//! # Threading
//!
//! `rclrs` executor spins in the calling thread (`node.spin()`). The simulation
//! state lives behind a `Mutex` so the timer callback and subscriber callbacks
//! can share it. Lock poisoning is treated as a fatal error (`.unwrap()`).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};
use geometry_msgs::msg::{
    Point, Pose, Quaternion, Twist, Vector3, Wrench, WrenchStamped,
};
use nav_msgs::msg::Odometry;
use rclrs::{
    Context as RclContext, CreateBasicExecutor, CreateTimerOptions, Executor,
    Node, Publisher, QosProfile, SpinOptions, Subscription, Timer,
    Worker, WorkerCommands,
};
use sensor_msgs::msg::Imu;
use std_msgs::msg::{Float64, Header};

use crate::convert::{
    stamp_now, twist_to_force, wrench_to_force, ForceCommand, PlantStateMsg,
};
use flight_core::{
    Plant, SimConfig, VehicleKind, VehicleSim, GRAVITY,
};

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

fn make_header(frame_id: &str) -> Header {
    Header {
        stamp: stamp_now(),
        frame_id: frame_id.to_string(),
    }
}

fn quat_from_yaw(yaw: f64) -> Quaternion {
    let half = yaw * 0.5;
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: half.sin(),
        w: half.cos(),
    }
}

fn quat_from_rpy(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    Quaternion {
        x: sr * cp * cy - cr * sp * sy,
        y: cr * sp * cy + sr * cp * sy,
        z: cr * cp * sy - sr * sp * cy,
        w: cr * cp * cy + sr * sp * sy,
    }
}

// ---------------------------------------------------------------------------
// SimNode — 6-DoF / 12-state vehicle simulator
// ---------------------------------------------------------------------------

/// ROS 2 node that wraps a [`VehicleSim`] and exposes `/odom`, `/imu`,
/// `/cmd_vel`, and `/cmd_wrench`.
///
/// # Topics
///
/// | Topic         | Type                    | Direction | Description                    |
/// |---------------|-------------------------|-----------|--------------------------------|
/// | `/odom`       | `nav_msgs/Odometry`     | pub       | Pose + twist at sim rate       |
/// | `/imu`        | `sensor_msgs/Imu`       | pub       | Linear accel + angular vel     |
/// | `/cmd_vel`    | `geometry_msgs/Twist`   | sub       | Velocity-derived force command |
/// | `/cmd_wrench` | `geometry_msgs/WrenchStamped` | sub | Direct force/torque command |
///
/// # Example
///
/// ```ignore
/// let ctx = RclContext::default_from_env()?;
/// let mut node = SimNode::new(&ctx, "flight_sim", VehicleKind::Ground, SimConfig::default())?;
/// node.spin()?;
/// ```
pub struct SimNode {
    executor: Executor,
    node: Node,
    worker: Worker<SimWorkerData>,
    _timer: Timer,
    _cmd_vel_sub: Subscription<Twist>,
    _cmd_wrench_sub: Subscription<WrenchStamped>,
}

struct SimWorkerData {
    sim: VehicleSim,
    pending_force: ForceCommand,
    odom_pub: Publisher<Odometry>,
    imu_pub: Publisher<Imu>,
}

impl SimNode {
    /// Create a new simulation node.
    ///
    /// `node_name` is the ROS 2 node name (typically `"flight_sim"`).
    /// `kind` selects the vehicle type. `config` sets mass, inertia, dt, etc.
    pub fn new(
        rcl_ctx: &RclContext,
        node_name: &str,
        kind: VehicleKind,
        config: SimConfig,
    ) -> Result<Self> {
        let executor = rcl_ctx.create_basic_executor(node_name.into());
        let node = executor.create_node(node_name)?;

        let qos = QosProfile::default();
        let odom_pub = node.create_publisher::<Odometry>("odom", qos.clone())?;
        let imu_pub = node.create_publisher::<Imu>("imu", qos.clone())?;

        let dt = config.dt;
        let sim = VehicleSim::new(kind, config);

        let worker_data = SimWorkerData {
            sim,
            pending_force: ForceCommand::zero(),
            odom_pub,
            imu_pub,
        };

        let worker: Worker<SimWorkerData> = node.create_worker(worker_data);

        let timer: Timer = {
            let worker_clone = worker.clone();
            node.create_timer(
                Duration::from_secs_f64(dt),
                move || {
                    let mut data = worker_clone.lock().unwrap();
                    let force = data.pending_force;
                    data.sim.step(force.force, force.torque);
                    let snapshot = data.sim.snapshot();
                    let pose = snapshot.pose;
                    let twist = snapshot.twist;
                    let accel = snapshot.accel;

                    let odom = Odometry {
                        header: make_header("odom"),
                        child_frame_id: "base_link".to_string(),
                        pose: Pose {
                            position: Point {
                                x: pose.x,
                                y: pose.y,
                                z: pose.z,
                            },
                            orientation: quat_from_rpy(pose.roll, pose.pitch, pose.yaw),
                        },
                        twist: Twist {
                            linear: Vector3 {
                                x: twist.vx,
                                y: twist.vy,
                                z: twist.vz,
                            },
                            angular: Vector3 {
                                x: twist.wx,
                                y: twist.wy,
                                z: twist.wz,
                            },
                        },
                        ..Default::default()
                    };
                    let _ = data.odom_pub.publish(odom);

                    let imu = Imu {
                        header: make_header("base_link"),
                        orientation: quat_from_rpy(pose.roll, pose.pitch, pose.yaw),
                        angular_velocity: Vector3 {
                            x: twist.wx,
                            y: twist.wy,
                            z: twist.wz,
                        },
                        linear_acceleration: Vector3 {
                            x: accel.ax,
                            y: accel.ay,
                            z: accel.az,
                        },
                        ..Default::default()
                    };
                    let _ = data.imu_pub.publish(imu);
                },
                CreateTimerOptions::default(),
            )?
        };

        let cmd_vel_sub: Subscription<Twist> = {
            let worker_clone = worker.clone();
            node.create_subscription::<Twist, _>(
                "cmd_vel",
                qos.clone(),
                move |msg: Twist| {
                    let mut data = worker_clone.lock().unwrap();
                    data.pending_force = twist_to_force(&msg, 1.0, 1.0);
                },
            )?
        };

        let cmd_wrench_sub: Subscription<WrenchStamped> = {
            let worker_clone = worker.clone();
            node.create_subscription::<WrenchStamped, _>(
                "cmd_wrench",
                qos,
                move |msg: WrenchStamped| {
                    let mut data = worker_clone.lock().unwrap();
                    data.pending_force = wrench_to_force(&msg.wrench);
                },
            )?
        };

        Ok(Self {
            executor,
            node,
            worker,
            _timer: timer,
            _cmd_vel_sub: cmd_vel_sub,
            _cmd_wrench_sub: cmd_wrench_sub,
        })
    }

    /// Spin the executor until interrupted.
    pub fn spin(&mut self) -> Result<()> {
        self.executor.spin(SpinOptions::default()).context("spin failed")
    }

    /// Return a reference to the underlying `rclrs` node.
    pub fn node(&self) -> &Node {
        &self.node
    }
}

// ---------------------------------------------------------------------------
// PlantNode — SISO plant simulator
// ---------------------------------------------------------------------------

/// ROS 2 node that wraps a [`Plant`] (mass-spring-damper) and exposes plant
/// state, output, and a force command subscriber.
///
/// # Topics
///
/// | Topic            | Type                      | Direction | Description              |
/// |------------------|---------------------------|-----------|--------------------------|
/// | `/plant/state`   | `nav_msgs/Odometry`       | pub       | Position + velocity      |
/// | `/plant/output`  | `geometry_msgs/WrenchStamped` | pub   | Measured output wrench   |
/// | `/plant/step`    | `std_msgs/Float64`        | pub       | Current time             |
/// | `/plant/y`       | `std_msgs/Float64`        | pub       | Output (position)        |
/// | `/plant/u`       | `std_msgs/Float64`        | pub       | Applied input            |
/// | `/cmd_force`     | `std_msgs/Float64`        | sub       | Force command            |
///
/// # Pairing with PX4
///
/// `PlantNode` is designed to run alongside `flight_px4::Px4PlantNode`:
///
/// * `Px4PlantNode` publishes `/cmd_force` (the control effort).
/// * `PlantNode` subscribes to `/cmd_force` and publishes `/plant/output`.
/// * `Px4PlantNode` subscribes to `/plant/output` as its measurement.
///
/// # Example
///
/// ```ignore
/// let ctx = RclContext::default_from_env()?;
/// let mut node = PlantNode::new(&ctx, "flight_plant", Plant::default())?;
/// node.spin()?;
/// ```
pub struct PlantNode {
    executor: Executor,
    node: Node,
    worker: Worker<PlantWorkerData>,
    _timer: Timer,
    _cmd_force_sub: Subscription<Float64>,
}

struct PlantWorkerData {
    plant: Plant,
    pending_u: f64,
    state_pub: Publisher<Odometry>,
    output_pub: Publisher<WrenchStamped>,
    step_pub: Publisher<Float64>,
    y_pub: Publisher<Float64>,
    u_pub: Publisher<Float64>,
}

impl PlantNode {
    /// Create a new plant node.
    pub fn new(rcl_ctx: &RclContext, node_name: &str, plant: Plant) -> Result<Self> {
        let executor = rcl_ctx.create_basic_executor(node_name.into());
        let node = executor.create_node(node_name)?;

        let qos = QosProfile::default();
        let state_pub = node.create_publisher::<Odometry>("plant/state", qos.clone())?;
        let output_pub =
            node.create_publisher::<WrenchStamped>("plant/output", qos.clone())?;
        let step_pub = node.create_publisher::<Float64>("plant/step", qos.clone())?;
        let y_pub = node.create_publisher::<Float64>("plant/y", qos.clone())?;
        let u_pub = node.create_publisher::<Float64>("plant/u", qos.clone())?;

        let dt = plant.dt();

        let worker_data = PlantWorkerData {
            plant,
            pending_u: 0.0,
            state_pub,
            output_pub,
            step_pub,
            y_pub,
            u_pub,
        };

        let worker: Worker<PlantWorkerData> = node.create_worker(worker_data);

        let timer: Timer = {
            let worker_clone = worker.clone();
            node.create_timer(
                Duration::from_secs_f64(dt),
                move || {
                    let mut data = worker_clone.lock().unwrap();
                    let u = data.pending_u;
                    data.plant.step(u);
                    let snap = data.plant.snapshot();

                    let odom = Odometry {
                        header: make_header("plant"),
                        child_frame_id: "plant".to_string(),
                        pose: Pose {
                            position: Point {
                                x: snap.position,
                                y: 0.0,
                                z: 0.0,
                            },
                            orientation: quat_from_yaw(0.0),
                        },
                        twist: Twist {
                            linear: Vector3 {
                                x: snap.velocity,
                                y: 0.0,
                                z: 0.0,
                            },
                            angular: Vector3 {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                            },
                        },
                        ..Default::default()
                    };
                    let _ = data.state_pub.publish(odom);

                    let wrench = WrenchStamped {
                        header: make_header("plant"),
                        wrench: Wrench {
                            force: Vector3 {
                                x: snap.output,
                                y: 0.0,
                                z: 0.0,
                            },
                            torque: Vector3 {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                            },
                        },
                    };
                    let _ = data.output_pub.publish(wrench);

                    let _ = data.step_pub.publish(Float64 { data: snap.time });
                    let _ = data.y_pub.publish(Float64 { data: snap.output });
                    let _ = data.u_pub.publish(Float64 { data: u });
                },
                CreateTimerOptions::default(),
            )?
        };

        let cmd_force_sub: Subscription<Float64> = {
            let worker_clone = worker.clone();
            node.create_subscription::<Float64, _>(
                "cmd_force",
                qos,
                move |msg: Float64| {
                    let mut data = worker_clone.lock().unwrap();
                    data.pending_u = msg.data;
                },
            )?
        };

        Ok(Self {
            executor,
            node,
            worker,
            _timer: timer,
            _cmd_force_sub: cmd_force_sub,
        })
    }

    /// Spin the executor until interrupted.
    pub fn spin(&mut self) -> Result<()> {
        self.executor.spin(SpinOptions::default()).context("spin failed")
    }

    /// Return a reference to the underlying `rclrs` node.
    pub fn node(&self) -> &Node {
        &self.node
    }
}
