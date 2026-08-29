//! Adversarial research probe: illegal JSON commands bounce, then a legal
//! attach sequence; properties still hold.

use robot_lab::Lab;

fn main() {
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "coastal".into());
    let mut lab = Lab::open(&scenario, 3).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let report = lab.research_probe(0.02, 120);
    println!("{}", serde_json::to_string_pretty(&report).unwrap());
    if !report.ok() {
        std::process::exit(1);
    }
}
