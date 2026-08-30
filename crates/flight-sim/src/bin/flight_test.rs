//! Contract lab: the same requirements on world, replay JSONL, or a
//! converted PX4/ulog trace.
//!
//! ```text
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend world
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend replay
//! cargo run -p flight-sim --bin flight-test -- --backend replay --replay trace.jsonl
//! ```
//!
//! `--backend px4-sitl` evaluates a converted HEARTBEAT/ulog JSONL (`--replay`).
//! Live SIH is `cargo test -p flight-px4 --test sitl_live -- --ignored`.

use flight_sim::{replay_jsonl, run_world, Scenario};

fn usage() -> ! {
    eprintln!(
        "flight-test --scenario gps-loss|heartbeat-stale [--backend world|replay|px4-sitl] [--replay FILE]"
    );
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut scenario_name = "gps-loss";
    let mut backend = "world";
    let mut replay: Option<String> = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--scenario" => {
                scenario_name = args
                    .get(i + 1)
                    .map(|s| s.as_str())
                    .unwrap_or_else(|| usage());
                i += 2;
            }
            "--backend" => {
                backend = args
                    .get(i + 1)
                    .map(|s| s.as_str())
                    .unwrap_or_else(|| usage());
                i += 2;
            }
            "--replay" => {
                replay = args.get(i + 1).cloned();
                i += 2;
            }
            "-h" | "--help" => usage(),
            other => {
                eprintln!("unknown arg {other}");
                usage();
            }
        }
    }

    let scenario = Scenario::by_name(scenario_name).unwrap_or_else(|| {
        eprintln!("unknown scenario {scenario_name}");
        usage();
    });

    match backend {
        "world" | "sim" => {
            let report = run_world(scenario).expect("world run");
            report.evaluate(scenario.require).expect("contract");
            println!(
                "PASS scenario={} backend=world samples={} failsafe={} epoch_final={}",
                report.name,
                report.samples.len(),
                report.samples.iter().any(|s| s.failsafe),
                report.samples.last().map(|s| s.epoch).unwrap_or(0)
            );
        }
        "replay" => {
            let jsonl = if let Some(path) = replay {
                std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
            } else {
                let report = run_world(scenario).expect("world run");
                report.evaluate(scenario.require).expect("live contract");
                report.to_jsonl()
            };
            replay_jsonl(&jsonl, scenario.require).expect("replay contract");
            println!(
                "PASS scenario={} backend=replay lines={}",
                scenario.name,
                jsonl.lines().count()
            );
        }
        "px4-sitl" => {
            let Some(path) = replay else {
                eprintln!(
                    "px4-sitl backend needs --replay FILE.jsonl (ulog/HEARTBEAT converted to TraceSample JSONL).\n\
                     Live SIH: cargo test -p flight-px4 --test sitl_live -- --ignored"
                );
                std::process::exit(2);
            };
            let jsonl =
                std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
            replay_jsonl(&jsonl, scenario.require).expect("px4-sitl contract");
            println!(
                "PASS scenario={} backend=px4-sitl file={path}",
                scenario.name
            );
        }
        other => {
            eprintln!("unknown backend {other}");
            usage();
        }
    }
}
