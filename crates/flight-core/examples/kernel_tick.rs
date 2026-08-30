//! Host tick of the `no_std` safety kernel (NEXT B8).
//!
//! Vehicles stay `std`. This binary only walks discrete machines:
//! `safety::step`, `ground_step`, `marine_step`, and HITL `deadline_outcome`.
//!
//! ```text
//! cargo run -p flight-core --example kernel_tick
//! cargo check -p flight-core --no-default-features
//! ```

fn main() {
    let t = flight_core::host::kernel_host_tick();
    println!(
        "PASS kernel-tick aerial={} ground={:?} marine={:?} hitl_met={} miss_zeros={}",
        t.aerial, t.ground, t.marine, t.hitl_met, t.hitl_miss_zeros
    );
    assert_eq!(t.aerial, flight_core::safety::Phase::Takeoff);
    assert!(t.hitl_met && t.hitl_miss_zeros);
}
