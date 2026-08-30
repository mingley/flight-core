//! Contract lab: the same requirements on world, replay JSONL, ULog, or a
//! converted PX4/ulog trace.
//!
//! ```text
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend world
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend replay
//! cargo run -p flight-sim --bin flight-test -- --backend replay --replay trace.jsonl
//! cargo run -p flight-sim --bin flight-test -- --backend replay --replay crates/flight-sim/corpus/gps_loss.ulg
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend px4-sitl --replay crates/flight-sim/corpus/px4_sitl_gps_loss.jsonl
//! cargo run -p flight-sim --bin flight-test -- --scenario gps-loss --backend all
//! cargo run -p flight-sim --bin flight-test -- --scenario hitl-miss --backend hitl
//! cargo run -p flight-sim --bin flight-test -- --scenario revoke-table
//! ```
//!
//! `--backend px4-sitl` evaluates a converted HEARTBEAT/ulog JSONL or `.ulg`
//! (`--replay`). Live SIH is `cargo test -p flight-px4 --test sitl_live -- --ignored`.

use flight_core::contracts::{evaluate_trace, parse_trace_jsonl, Requirement};
use flight_sim::{
    differential_gps_loss, is_ulog, parse_ulog, replay_jsonl, run_hitl_miss, run_revoke_table,
    run_world, write_ulog, Scenario,
};

fn usage() -> ! {
    eprintln!(
        "flight-test --scenario gps-loss|heartbeat-stale|hitl-miss|revoke-table [--backend world|replay|px4-sitl|ulog|hitl|all] [--replay FILE] [--write-ulog FILE]"
    );
    std::process::exit(2);
}

fn scenario_from_args(name: &str) -> &'static Scenario {
    Scenario::by_name(name).unwrap_or_else(|| {
        eprintln!("unknown scenario {name}");
        usage();
    })
}

fn load_trace(path: &str) -> (Vec<flight_core::contracts::TraceSample>, &'static str) {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    if is_ulog(&bytes) || path.ends_with(".ulg") || path.ends_with(".ulog") {
        let samples = parse_ulog(&bytes).unwrap_or_else(|e| panic!("ulog {path}: {e}"));
        (samples, "ulog")
    } else {
        let text = String::from_utf8(bytes).unwrap_or_else(|e| panic!("utf8 {path}: {e}"));
        let samples = parse_trace_jsonl(&text).unwrap_or_else(|e| panic!("jsonl {path}: {e}"));
        (samples, "jsonl")
    }
}

fn evaluate_samples(samples: &[flight_core::contracts::TraceSample], reqs: &[Requirement]) {
    evaluate_trace(samples, reqs)
        .unwrap_or_else(|e| panic!("contract {} at sample {}", e.requirement, e.index));
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut scenario_name = "gps-loss";
    let mut backend = "world";
    let mut replay: Option<String> = None;
    let mut write_ulog_path: Option<String> = None;
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
            "--write-ulog" => {
                write_ulog_path = args.get(i + 1).cloned();
                i += 2;
            }
            "-h" | "--help" => usage(),
            other => {
                eprintln!("unknown arg {other}");
                usage();
            }
        }
    }

    if scenario_name == "revoke-table" {
        let report = run_revoke_table().expect("revoke table");
        evaluate_samples(
            &report.samples,
            &[
                Requirement::NeverActuateWhileDisarmed,
                Requirement::ActuatorsImplyArmed,
                Requirement::NoNanCommands,
            ],
        );
        println!(
            "PASS scenario=revoke-table backend=world samples={} events={}",
            report.samples.len(),
            report.samples.len()
        );
        return;
    }

    let scenario = scenario_from_args(scenario_name);

    match backend {
        "world" | "sim" => {
            let report = run_world(scenario).expect("world run");
            report.evaluate(scenario.require).expect("contract");
            if let Some(path) = write_ulog_path.as_ref() {
                std::fs::write(path, write_ulog(&report.samples))
                    .unwrap_or_else(|e| panic!("write {path}: {e}"));
            }
            println!(
                "PASS scenario={} backend=world samples={} failsafe={} epoch_final={}",
                report.name,
                report.samples.len(),
                report.samples.iter().any(|s| s.failsafe),
                report.samples.last().map(|s| s.epoch).unwrap_or(0)
            );
        }
        "replay" | "ulog" => {
            if let Some(path) = replay {
                let (samples, kind) = load_trace(&path);
                evaluate_samples(&samples, scenario.require);
                println!(
                    "PASS scenario={} backend={backend} format={kind} file={path} samples={}",
                    scenario.name,
                    samples.len()
                );
            } else {
                let report = run_world(scenario).expect("world run");
                report.evaluate(scenario.require).expect("live contract");
                if backend == "ulog" {
                    let bytes = write_ulog(&report.samples);
                    let samples = parse_ulog(&bytes).expect("ulog roundtrip");
                    evaluate_samples(&samples, scenario.require);
                    println!(
                        "PASS scenario={} backend=ulog samples={}",
                        scenario.name,
                        samples.len()
                    );
                } else {
                    replay_jsonl(&report.to_jsonl(), scenario.require).expect("replay contract");
                    println!(
                        "PASS scenario={} backend=replay lines={}",
                        scenario.name,
                        report.samples.len()
                    );
                }
            }
        }
        "px4-sitl" => {
            let Some(path) = replay else {
                eprintln!(
                    "px4-sitl backend needs --replay FILE.jsonl or FILE.ulg \
                     (ulog/HEARTBEAT converted to TraceSample).\n\
                     Checked-in converter corpus: crates/flight-sim/corpus/px4_sitl_gps_loss.jsonl\n\
                     Live SIH: cargo test -p flight-px4 --test sitl_live -- --ignored"
                );
                std::process::exit(2);
            };
            let (samples, kind) = load_trace(&path);
            evaluate_samples(&samples, scenario.require);
            println!(
                "PASS scenario={} backend=px4-sitl format={kind} file={path} samples={}",
                scenario.name,
                samples.len()
            );
        }
        "all" => {
            if scenario.name == "gps-loss" {
                differential_gps_loss().expect("world + ulog + px4-sitl");
                println!("PASS scenario=gps-loss backend=all (world, ulog, px4-sitl corpus)");
            } else {
                let report = run_world(scenario).expect("world run");
                report.evaluate(scenario.require).expect("contract");
                println!(
                    "PASS scenario={} backend=all (world) samples={} failsafe={} epoch_final={}",
                    report.name,
                    report.samples.len(),
                    report.samples.iter().any(|s| s.failsafe),
                    report.samples.last().map(|s| s.epoch).unwrap_or(0)
                );
            }
        }
        "hitl" => {
            let report = run_hitl_miss().expect("hitl miss");
            report
                .evaluate(Scenario::HITL_MISS.require)
                .expect("contract");
            println!(
                "PASS scenario={} backend=hitl samples={} failsafe={} epoch_final={}",
                report.name,
                report.samples.len(),
                report.samples.iter().any(|s| s.failsafe),
                report.samples.last().map(|s| s.epoch).unwrap_or(0)
            );
        }
        other => {
            eprintln!("unknown backend {other}");
            usage();
        }
    }
}
