//! Inland rack deadline miss must satisfy `Scenario::HITL_MISS`.

use flight_core::contracts::evaluate_trace;
use flight_hitl::WorldRack;
use flight_sim::Scenario;

fn main() {
    let samples = WorldRack::contract_deadline_miss(1).expect("miss");
    evaluate_trace(&samples, Scenario::HITL_MISS.require).expect("contract");
    println!(
        "PASS hitl-miss rack samples={} failsafe={} epoch_final={}",
        samples.len(),
        samples.iter().any(|s| s.failsafe),
        samples.last().map(|s| s.epoch).unwrap_or(0)
    );
}
