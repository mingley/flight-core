//! Publish a NED velocity as `geometry_msgs/msg/Twist` on `/cmd_vel`.
//!
//! Requires a sourced ROS 2 Jazzy install:
//!
//! ```bash
//! source /opt/ros/jazzy/setup.bash
//! cargo run -p flight-ros2 --features rclrs --example offboard
//! ```

use flight_core::frames::Ned;
use flight_core::vector::Velocity;
use flight_ros2::node::OffboardNode;
use flight_ros2::{ExternalFlightMode, VelocityMode};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut mode = VelocityMode::new(Velocity::<Ned>::ned(0.5, 0.0, 0.0));
    mode.on_activate();
    let mut node = OffboardNode::new("flight_core_offboard", "/cmd_vel")?;
    eprintln!(
        "rclrs node {} publishing {} (ENU Twist from NED {:?})",
        node.name(),
        node.topic(),
        mode.velocity.xyz()
    );
    for _ in 0..25 {
        node.publish_mode(&mut mode, 0.02)?;
        let _ = node.spin_once(Duration::from_millis(20));
    }
    Ok(())
}
