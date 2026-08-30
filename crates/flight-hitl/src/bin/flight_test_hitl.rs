//! HITL leftover OffboardControl after a rack deadline miss.
//!
//! `flight-sim` cannot depend on `flight-hitl` (cycle). This bin is the
//! HITL-side leftover check: bind Takeoff, miss the compute/`Rate` budget,
//! leftover `COMMANDS` are `StaleAuthority`.
//!
//! ```text
//! cargo run -p flight-hitl --bin flight-test-hitl
//! ```

fn main() {
    let n = flight_hitl::WorldRack::run_hitl_leftover_deadline_miss().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=hitl-miss leftover backend=hitl: {e}");
        std::process::exit(1);
    });
    println!("PASS scenario=hitl-miss leftover={n} backend=hitl (rack deadline miss + Rate)");
}
