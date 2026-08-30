//! Inland rack deadline miss must satisfy `Scenario::HITL_MISS`.
//! Leftover OffboardControl `COMMANDS` are stale after the miss.

use flight_core::contracts::evaluate_trace;
use flight_hitl::WorldRack;
use flight_sim::Scenario;

fn main() {
    let samples = WorldRack::contract_deadline_miss(1).expect("miss");
    evaluate_trace(&samples, Scenario::HITL_MISS.require).expect("contract");
    WorldRack::leftover_after_deadline_miss(1).expect("leftover");
    println!(
        "PASS hitl-miss rack samples={} failsafe={} epoch_final={} leftover=stale",
        samples.len(),
        samples.iter().any(|s| s.failsafe),
        samples.last().map(|s| s.epoch).unwrap_or(0)
    );
}
