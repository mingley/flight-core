//! GPS-loss scenario: the same contract on the verified world.
//!
//! ```text
//! cargo run -p flight-sim --example gps_loss
//! ```

fn main() {
    let report = flight_sim::run_world(&flight_sim::Scenario::GPS_LOSS).expect("world run");
    flight_sim::replay_report(&report, flight_sim::Scenario::GPS_LOSS.require).expect("contract");
    println!(
        "scenario={} backend={} samples={} failsafe={} epoch_final={}",
        report.name,
        report.backend,
        report.samples.len(),
        report.samples.iter().any(|s| s.failsafe),
        report.samples.last().map(|s| s.epoch).unwrap_or(0)
    );
    print!("{}", report.to_jsonl());
}
