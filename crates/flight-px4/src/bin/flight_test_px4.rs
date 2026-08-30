//! PX4 companion leftover Offboard revoke table and leftover contracts.
//!
//! `flight-sim` cannot depend on `flight-px4` (cycle). This bin is the
//! PX4-side `flight-test --scenario revoke-table` and named leftover
//! contracts (`gps-loss`, `heartbeat-stale`, `hitl-miss`, `imu-loss`).
//!
//! ```text
//! cargo run -p flight-px4 --bin flight-test-px4
//! ```

fn main() {
    let n = flight_px4::run_px4_revoke_table().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=revoke-table backend=px4: {e}");
        std::process::exit(1);
    });
    println!(
        "PASS scenario=revoke-table backend=px4 leftover={n} events={n} (companion inject_revoke)"
    );
    let reports = flight_px4::run_px4_leftover_contracts().unwrap_or_else(|e| {
        eprintln!("FAIL leftover-contracts backend=px4: {e}");
        std::process::exit(1);
    });
    for report in reports {
        println!(
            "PASS scenario={} backend=px4 leftover=1 samples={} (companion {:?}, leftover contract)",
            report.name,
            report.samples.len(),
            report.inject
        );
    }
}
