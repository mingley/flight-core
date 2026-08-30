//! ArduPilot companion leftover Offboard revoke table and leftover contracts.
//!
//! `flight-sim` does not depend on `flight-ardupilot`. This bin is the
//! ArduPilot-side `flight-test --scenario revoke-table` and named leftover
//! contracts (`gps-loss`, `heartbeat-stale`, `hitl-miss`, `imu-loss`).
//!
//! ```text
//! cargo run -p flight-ardupilot --bin flight-test-ardupilot
//! ```

fn main() {
    let n = flight_ardupilot::run_ardupilot_revoke_table().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=revoke-table backend=ardupilot: {e}");
        std::process::exit(1);
    });
    println!(
        "PASS scenario=revoke-table backend=ardupilot leftover={n} events={n} (companion inject_revoke)"
    );
    let reports = flight_ardupilot::run_ardupilot_leftover_contracts().unwrap_or_else(|e| {
        eprintln!("FAIL leftover-contracts backend=ardupilot: {e}");
        std::process::exit(1);
    });
    for report in reports {
        println!(
            "PASS scenario={} backend=ardupilot leftover=1 samples={} (companion {:?}, leftover contract)",
            report.name,
            report.samples.len(),
            report.inject
        );
    }
}
