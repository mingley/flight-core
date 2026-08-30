//! ROS 2 leftover OffboardControl after apply_failsafe and every DSL revoke.
//!
//! `flight-sim` cannot depend on `flight-ros2` (cycle). This bin is the
//! ROS 2 plant leftover check: bind Takeoff, trip failsafe, leftover
//! `COMMANDS` are `StaleAuthority`. The same leftover table then injects
//! every `REVOKE_ON` event through the plant. Does not require rclrs / Jazzy.
//!
//! ```text
//! cargo run -p flight-ros2 --bin flight-test-ros2
//! ```

fn main() {
    let failsafe = flight_ros2::plant::run_ros2_failsafe_leftover().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=ros2-failsafe leftover backend=ros2: {e}");
        std::process::exit(1);
    });
    let n = flight_ros2::plant::run_ros2_revoke_table().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=revoke-table leftover backend=ros2: {e}");
        std::process::exit(1);
    });
    println!("PASS scenario=ros2-failsafe leftover={failsafe} backend=ros2 (apply_failsafe)");
    println!(
        "PASS scenario=revoke-table leftover={n} events={n} backend=ros2 (plant inject_revoke)"
    );
}
