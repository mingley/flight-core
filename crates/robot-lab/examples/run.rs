//! Closed-loop experiment runner (NEXT A3).
//!
//! ```text
//! cargo run -p robot-lab --example run -- \
//!   --scenario harbor --seeds 1,3 --dt 0.02 --steps 40 \
//!   --agent typed-fleet-hold --out /tmp/harbor-hold --mcap \
//!   --require-property position_hold_restores_pose
//! ```
//!
//! Non-zero exit if `all_hold` is false or `--require-property` does not hold.
//! Each tick is still one `WorldSession::step` (P12).

use robot_lab::Experiment;
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    let spec = match parse_args(env::args().skip(1)) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{e}");
            eprintln!(
                "usage: run --scenario NAME --seed N|--seeds 1,2|--from A --to B \
                 [--dt 0.02] [--steps 40] [--agent typed-fleet-hold] [--jsonl FILE] \
                 --out DIR [--mcap] [--require-property ID]"
            );
            return ExitCode::from(2);
        }
    };
    match spec.execute() {
        Ok(summary) => {
            println!("{}", serde_json::to_string_pretty(&summary).unwrap());
            if summary.all_ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        Err(e) => {
            eprintln!("{e}");
            ExitCode::from(2)
        }
    }
}

fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Experiment, String> {
    let mut scenario = "harbor".to_string();
    let mut seeds: Vec<u64> = Vec::new();
    let mut from = None;
    let mut to = None;
    let mut dt = 0.02f32;
    let mut steps = 40u32;
    let mut agent = "typed-fleet-hold".to_string();
    let mut jsonl = None;
    let mut out = None;
    let mut mcap = false;
    let mut require_property = None;
    let mut it = args.into_iter().peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--scenario" => scenario = req(&mut it, "--scenario")?,
            "--seed" => seeds.push(parse_u64(&req(&mut it, "--seed")?, "--seed")?),
            "--seeds" => {
                for part in req(&mut it, "--seeds")?.split(',') {
                    seeds.push(parse_u64(part.trim(), "--seeds")?);
                }
            }
            "--from" => from = Some(parse_u64(&req(&mut it, "--from")?, "--from")?),
            "--to" => to = Some(parse_u64(&req(&mut it, "--to")?, "--to")?),
            "--dt" => {
                dt = req(&mut it, "--dt")?
                    .parse()
                    .map_err(|_| "invalid --dt".to_string())?;
            }
            "--steps" => steps = parse_u32(&req(&mut it, "--steps")?, "--steps")?,
            "--agent" => agent = req(&mut it, "--agent")?,
            "--jsonl" => jsonl = Some(PathBuf::from(req(&mut it, "--jsonl")?)),
            "--out" => out = Some(PathBuf::from(req(&mut it, "--out")?)),
            "--mcap" => mcap = true,
            "--require-property" => {
                require_property = Some(req(&mut it, "--require-property")?);
            }
            "--help" | "-h" => {
                return Err("closed-loop lab experiment runner".into());
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if let (Some(a), Some(b)) = (from, to) {
        if a > b {
            return Err("--from must be <= --to".into());
        }
        seeds.extend(a..=b);
    }
    if seeds.is_empty() {
        seeds.push(1);
    }
    let out = out.ok_or_else(|| "missing --out DIR".to_string())?;
    Ok(Experiment {
        scenario,
        seeds,
        dt,
        steps,
        agent,
        jsonl,
        out,
        mcap,
        require_property,
    })
}

fn req(it: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{flag} needs a value"))
}

fn parse_u64(s: &str, flag: &str) -> Result<u64, String> {
    s.parse().map_err(|_| format!("invalid {flag} '{s}'"))
}

fn parse_u32(s: &str, flag: &str) -> Result<u32, String> {
    s.parse().map_err(|_| format!("invalid {flag} '{s}'"))
}
