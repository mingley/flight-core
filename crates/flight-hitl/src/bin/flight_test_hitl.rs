//! HITL leftover OffboardControl after a rack deadline miss and every DSL revoke.
//!
//! `flight-sim` cannot depend on `flight-hitl` (cycle). This bin is the
//! HITL-side leftover check: bind Takeoff, miss the compute/`Rate` budget,
//! leftover `COMMANDS` are `StaleAuthority`. The same leftover table then
//! injects every `REVOKE_ON` event through the rack.
//!
//! ```text
//! cargo run -p flight-hitl --bin flight-test-hitl
//! ```

fn main() {
    let miss = flight_hitl::WorldRack::run_hitl_leftover_deadline_miss().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=hitl-miss leftover backend=hitl: {e}");
        std::process::exit(1);
    });
    let n = flight_hitl::WorldRack::run_hitl_revoke_table().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=revoke-table leftover backend=hitl: {e}");
        std::process::exit(1);
    });
    println!("PASS scenario=hitl-miss leftover={miss} backend=hitl (rack deadline miss + Rate)");
    println!(
        "PASS scenario=revoke-table leftover={n} events={n} backend=hitl (rack inject_revoke)"
    );
}
