//! Stream a Foxglove-compatible MCAP bag of verified lab observations.
//!
//! ```bash
//! cargo run -p robot-lab --example bag inland > inland.mcap
//! ```
//!
//! Open `inland.mcap` in Foxglove (File → Open local file). Topics
//! `/lab/observation` and `/lab/action` are JSON with `jsonschema` encoding
//! from `crates/robot-lab/schemas/`. Observation fields include `hold_ned`,
//! `legal_cmds`, aerial/ground/marine `kind`, `sphere_hits`, and the property
//! vector. NED z-down is documented on the schema.

use robot_lab::{Lab, McapBag};

fn main() {
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "coastal".into());
    let mut lab = Lab::open(&scenario, 1).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let stdout = std::io::stdout();
    let mut bag = McapBag::new(stdout.lock()).expect("mcap header");
    for k in 0..200 {
        lab.apply_script();
        lab.step(0.02);
        if k % 5 == 0 {
            bag.write_observation(&lab.observe()).expect("observation");
        }
        if !lab.all_hold() {
            eprintln!("property violation at t={:.2}", lab.world().t);
            std::process::exit(1);
        }
    }
    for action in &lab.log {
        bag.write_action(action).expect("action");
    }
    let _ = bag.finish().expect("mcap footer");
}
