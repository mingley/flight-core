//! Stream verified observations as JSONL for research / replay.

use robot_lab::Lab;

fn main() {
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "inland".into());
    let mut lab = Lab::open(&scenario, 1).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    for k in 0..200 {
        lab.apply_script();
        lab.step(0.02);
        if k % 10 == 0 {
            lab.write_jsonl(&mut out).expect("jsonl");
        }
        if !lab.all_hold() {
            eprintln!("property violation at t={:.2}", lab.world().t);
            std::process::exit(1);
        }
    }
}
