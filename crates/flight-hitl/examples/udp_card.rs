//! Faithful FCH1 UDP card against an inland rack (not the in-process plant).
//!
//! ```text
//! cargo run -p flight-hitl --example udp_card
//! ```

use flight_hitl::run_fch1_udp_mock;

fn main() {
    let report = run_fch1_udp_mock().expect("udp mock");
    print!("{}", report.jsonl);
    println!(
        "PASS fch1-udp-mock frames={} samples={} apply_zero=1 (UDP card, WorldSession plant)",
        report.frames, report.samples_rx
    );
}
