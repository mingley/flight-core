//! HITL leftover OffboardControl after a rack deadline miss and every DSL revoke.
//! Faithful FCH1 UDP card (not the in-process plant) is `run_fch1_udp_mock`.
//!
//! `flight-sim` cannot depend on `flight-hitl` (cycle). This bin is the
//! HITL-side leftover check: bind Takeoff, miss the compute/`Rate` budget,
//! leftover `COMMANDS` are `StaleAuthority`. The same leftover table then
//! injects every `REVOKE_ON` event through the rack. Then a loopback FCH1
//! card proves `apply == 0` zeros a slot on the wire.
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
    let gps = flight_hitl::WorldRack::run_hitl_gps_loss().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=gps-loss leftover backend=hitl: {e}");
        std::process::exit(1);
    });
    let mock = flight_hitl::run_fch1_udp_mock().unwrap_or_else(|e| {
        eprintln!("FAIL scenario=fch1-udp-mock backend=hitl: {e}");
        std::process::exit(1);
    });
    println!("PASS scenario=hitl-miss leftover={miss} backend=hitl (rack deadline miss + Rate)");
    println!(
        "PASS scenario=revoke-table leftover={n} events={n} backend=hitl (rack inject_revoke)"
    );
    println!(
        "PASS scenario=gps-loss leftover=1 samples={} backend=hitl (rack EstimatorInvalid, GPS_LOSS require)",
        gps.samples.len()
    );
    println!(
        "PASS scenario=fch1-udp-mock frames={} samples={} backend=hitl (faithful UDP card, not in-process plant)",
        mock.frames, mock.samples_rx
    );
}
