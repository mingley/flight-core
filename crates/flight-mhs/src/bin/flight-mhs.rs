//! MHS-shaped CLI: discover, reference, read, write, chain, mcp.

use flight_mhs::{serve_stdio, ChainDoc, Driver, WriteRequest, SPEC_NOTE};
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let cmd = args.next().unwrap_or_else(|| "help".into());
    match cmd.as_str() {
        "help" | "-h" | "--help" => {
            eprintln!(
                "flight-mhs — MHS-shaped driver (not official MHS)\n\n\
                 {SPEC_NOTE}\n\n\
                 commands:\n\
                   discover  [--scenario NAME] [--seed N]\n\
                   reference [--scenario NAME] [--seed N] [--device ID]\n\
                   read      [--scenario NAME] [--seed N] --device ID --channel CH\n\
                   write     [--scenario NAME] [--seed N] --device ID --channel CH [--vn --ve --vd --yaw-rate]\n\
                   chain     [--scenario NAME] [--seed N] --file PATH [--dt 0.02]\n\
                   mcp       [--scenario NAME] [--seed N]   # newline JSON-RPC on stdio\n"
            );
            ExitCode::SUCCESS
        }
        "mcp" => match parse_open(args) {
            Ok((scenario, seed)) => match Driver::open(&scenario, seed) {
                Ok(d) => {
                    serve_stdio(d);
                    ExitCode::SUCCESS
                }
                Err(e) => fail(&e.to_string()),
            },
            Err(e) => fail(&e),
        },
        other => match run(other, args) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => fail(&e),
        },
    }
}

fn fail(e: &str) -> ExitCode {
    eprintln!("{e}");
    ExitCode::from(1)
}

fn run(cmd: &str, args: impl Iterator<Item = String>) -> Result<(), String> {
    let mut scenario = "coastal".to_string();
    let mut seed = 1u64;
    let mut device = None;
    let mut channel = None;
    let mut vn = 0.0f32;
    let mut ve = 0.0f32;
    let mut vd = 0.0f32;
    let mut yaw_rate = 0.0f32;
    let mut file = None;
    let mut dt = 0.02f32;
    let mut it = args.peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--scenario" => scenario = req(&mut it, "--scenario")?,
            "--seed" => seed = parse_u64(&req(&mut it, "--seed")?, "--seed")?,
            "--device" => device = Some(req(&mut it, "--device")?),
            "--channel" => channel = Some(req(&mut it, "--channel")?),
            "--vn" => vn = parse_f32(&req(&mut it, "--vn")?, "--vn")?,
            "--ve" => ve = parse_f32(&req(&mut it, "--ve")?, "--ve")?,
            "--vd" => vd = parse_f32(&req(&mut it, "--vd")?, "--vd")?,
            "--yaw-rate" => yaw_rate = parse_f32(&req(&mut it, "--yaw-rate")?, "--yaw-rate")?,
            "--file" => file = Some(req(&mut it, "--file")?),
            "--dt" => dt = parse_f32(&req(&mut it, "--dt")?, "--dt")?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let mut driver = Driver::open(&scenario, seed).map_err(|e| e.to_string())?;
    match cmd {
        "discover" => {
            println!(
                "{}",
                serde_json::to_string_pretty(&driver.discover()).unwrap()
            );
        }
        "reference" => {
            let v = match device.as_deref() {
                Some(id) => {
                    serde_json::to_value(driver.reference(id).map_err(|e| e.to_string())?).unwrap()
                }
                None => serde_json::to_value(driver.references()).unwrap(),
            };
            println!("{}", serde_json::to_string_pretty(&v).unwrap());
        }
        "read" => {
            let device = device.ok_or_else(|| "--device required".to_string())?;
            let channel = channel.ok_or_else(|| "--channel required".to_string())?;
            let r = driver.read(&device, &channel).map_err(|e| e.to_string())?;
            println!("{}", serde_json::to_string_pretty(&r).unwrap());
        }
        "write" => {
            let device = device.ok_or_else(|| "--device required".to_string())?;
            let channel = channel.ok_or_else(|| "--channel required".to_string())?;
            let req = WriteRequest {
                device,
                channel,
                vn,
                ve,
                vd,
                yaw_rate,
            };
            match driver.write(&req) {
                Ok(w) => println!("{}", serde_json::to_string_pretty(&w).unwrap()),
                Err(e) => {
                    let f = driver.last_failure(&e);
                    println!("{}", serde_json::to_string_pretty(&f).unwrap());
                    return Err(e.to_string());
                }
            }
        }
        "chain" => {
            let path = file.ok_or_else(|| "--file required".to_string())?;
            let text = fs::read_to_string(&path).map_err(|e| e.to_string())?;
            let doc = ChainDoc::parse(&text).map_err(|e| e.to_string())?;
            if let Some(s) = &doc.scenario {
                driver = Driver::open(s, doc.seed.unwrap_or(seed)).map_err(|e| e.to_string())?;
            }
            let report = driver.run_chain(&doc.ops, doc.dt.unwrap_or(dt));
            println!("{}", serde_json::to_string_pretty(&report).unwrap());
            if !report.ok {
                return Err("chain failed".into());
            }
        }
        other => return Err(format!("unknown command {other}")),
    }
    Ok(())
}

fn parse_open(args: impl Iterator<Item = String>) -> Result<(String, u64), String> {
    let mut scenario = "coastal".to_string();
    let mut seed = 1u64;
    let mut it = args.peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--scenario" => scenario = req(&mut it, "--scenario")?,
            "--seed" => seed = parse_u64(&req(&mut it, "--seed")?, "--seed")?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok((scenario, seed))
}

fn req(it: &mut impl Iterator<Item = String>, name: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{name} needs a value"))
}

fn parse_u64(s: &str, name: &str) -> Result<u64, String> {
    s.parse().map_err(|_| format!("invalid {name}"))
}

fn parse_f32(s: &str, name: &str) -> Result<f32, String> {
    s.parse().map_err(|_| format!("invalid {name}"))
}
