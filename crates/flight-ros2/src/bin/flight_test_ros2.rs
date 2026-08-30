//! ROS 2 leftover OffboardControl after apply_failsafe, apply_disarm, and every DSL revoke.
//!
//! `flight-sim` cannot depend on `flight-ros2` (cycle). This bin is the
//! ROS 2 plant leftover check: bind Takeoff, trip failsafe or disarm, leftover
//! `COMMANDS` are `StaleAuthority`. The leftover table then injects
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
    let disarm = flight_ros2::plant::run_ros2_disarm_leftover().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=ros2-disarm leftover backend=ros2: {e}");
        std::process::exit(1);
    });
    let n = flight_ros2::plant::run_ros2_revoke_table().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=revoke-table leftover backend=ros2: {e}");
        std::process::exit(1);
    });
    let contracts = flight_ros2::plant::run_ros2_leftover_contracts().unwrap_or_else(|e| {
        eprintln!("FAIL leftover-contracts leftover backend=ros2: {e}");
        std::process::exit(1);
    });
    println!("PASS scenario=ros2-failsafe leftover={failsafe} backend=ros2 (apply_failsafe)");
    println!("PASS scenario=ros2-disarm leftover={disarm} backend=ros2 (apply_disarm)");
    println!(
        "PASS scenario=revoke-table leftover={n} events={n} backend=ros2 (plant inject_revoke)"
    );
    for report in contracts {
        println!(
            "PASS scenario={} leftover=1 samples={} backend=ros2 (plant {:?}, leftover contract)",
            report.name,
            report.samples.len(),
            report.inject
        );
    }
}
