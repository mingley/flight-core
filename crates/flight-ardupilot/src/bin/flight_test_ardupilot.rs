//! ArduPilot companion leftover Offboard revoke table and GPS-loss contract.
//!
//! `flight-sim` does not depend on `flight-ardupilot`. This bin is the
//! ArduPilot-side `flight-test --scenario revoke-table` and `--scenario gps-loss`.
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
    let report = flight_ardupilot::run_ardupilot_gps_loss().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=gps-loss backend=ardupilot: {e}");
        std::process::exit(1);
    });
    println!(
        "PASS scenario=gps-loss backend=ardupilot leftover=1 samples={} (companion EstimatorInvalid, GPS_LOSS require)",
        report.samples.len()
    );
}
