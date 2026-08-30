//! Minimal newline-delimited JSON-RPC MCP (tools only). Not official MHS.

use serde_json::{json, Value};

use crate::driver::Driver;
use crate::surface::WriteRequest;
use crate::ChainDoc;

const PROTOCOL: &str = "2025-03-26";

/// Handle one JSON-RPC object. Notifications return `None`.
pub fn handle_rpc(driver: &mut Driver, req: &Value) -> Option<Value> {
    let method = req.get("method").and_then(Value::as_str)?;
    let id = req.get("id").cloned()?;
    let result = match method {
        "initialize" => json!({
            "protocolVersion": PROTOCOL,
            "capabilities": { "tools": {} },
            "serverInfo": {
                "name": "flight-mhs",
                "version": env!("CARGO_PKG_VERSION"),
            },
            "instructions": crate::SPEC_NOTE,
        }),
        "ping" => json!({}),
        "tools/list" => json!({ "tools": tool_list() }),
        "tools/call" => return Some(rpc_ok(id, call_tool(driver, req))),
        other => {
            return Some(rpc_err(id, -32601, format!("method not found: {other}")));
        }
    };
    Some(rpc_ok(id, result))
}

fn rpc_ok(id: Value, result: Value) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "result": result })
}

fn rpc_err(id: Value, code: i32, message: String) -> Value {
    json!({
        "jsonrpc": "2.0",
        "id": id,
        "error": { "code": code, "message": message }
    })
}

fn tool_text(ok: bool, value: Value) -> Value {
    json!({
        "content": [{ "type": "text", "text": value.to_string() }],
        "isError": !ok,
    })
}

fn call_tool(driver: &mut Driver, req: &Value) -> Value {
    let args = req
        .pointer("/params/arguments")
        .cloned()
        .unwrap_or_else(|| json!({}));
    let name = req
        .pointer("/params/name")
        .and_then(Value::as_str)
        .unwrap_or("");
    match name {
        "mhs_open" => {
            let scenario = args
                .get("scenario")
                .and_then(Value::as_str)
                .unwrap_or("coastal");
            let seed = args.get("seed").and_then(Value::as_u64).unwrap_or(1);
            match Driver::open(scenario, seed) {
                Ok(d) => {
                    *driver = d;
                    tool_text(true, serde_json::to_value(driver.discover()).unwrap())
                }
                Err(e) => tool_text(
                    false,
                    serde_json::to_value(driver.last_failure(&e)).unwrap(),
                ),
            }
        }
        "mhs_discover" => tool_text(true, serde_json::to_value(driver.discover()).unwrap()),
        "mhs_reference" => {
            if let Some(id) = args.get("device").and_then(Value::as_str) {
                match driver.reference(id) {
                    Ok(r) => tool_text(true, serde_json::to_value(r).unwrap()),
                    Err(e) => tool_text(
                        false,
                        serde_json::to_value(driver.last_failure(&e)).unwrap(),
                    ),
                }
            } else {
                tool_text(true, serde_json::to_value(driver.references()).unwrap())
            }
        }
        "mhs_read" => {
            let device = args.get("device").and_then(Value::as_str).unwrap_or("");
            let channel = args.get("channel").and_then(Value::as_str).unwrap_or("");
            match driver.read(device, channel) {
                Ok(r) => tool_text(true, serde_json::to_value(r).unwrap()),
                Err(e) => tool_text(
                    false,
                    serde_json::to_value(driver.last_failure(&e)).unwrap(),
                ),
            }
        }
        "mhs_write" => {
            let req = match serde_json::from_value::<WriteRequest>(args) {
                Ok(r) => r,
                Err(e) => {
                    return tool_text(
                        false,
                        json!({ "ok": false, "code": "protocol", "error": e.to_string() }),
                    )
                }
            };
            match driver.write(&req) {
                Ok(w) => tool_text(true, serde_json::to_value(w).unwrap()),
                Err(e) => tool_text(
                    false,
                    serde_json::to_value(driver.last_failure(&e)).unwrap(),
                ),
            }
        }
        "mhs_step" => {
            let dt = args.get("dt").and_then(Value::as_f64).unwrap_or(0.02) as f32;
            let n = args.get("n").and_then(Value::as_u64).unwrap_or(1) as u32;
            driver.step(dt, n);
            tool_text(
                true,
                json!({
                    "ok": true,
                    "t": driver.lab().world().t,
                    "all_hold": driver.lab().all_hold(),
                    "steps": n,
                }),
            )
        }
        "mhs_chain" => {
            let doc = if let Some(ops) = args.get("ops") {
                let wrapped = json!({ "ops": ops });
                match serde_json::from_value::<ChainDoc>(wrapped) {
                    Ok(d) => d,
                    Err(e) => {
                        return tool_text(
                            false,
                            json!({ "ok": false, "code": "chain", "error": e.to_string() }),
                        )
                    }
                }
            } else if let Some(text) = args.get("text").and_then(Value::as_str) {
                match ChainDoc::parse(text) {
                    Ok(d) => d,
                    Err(e) => {
                        return tool_text(
                            false,
                            serde_json::to_value(driver.last_failure(&e)).unwrap(),
                        )
                    }
                }
            } else {
                return tool_text(
                    false,
                    json!({ "ok": false, "code": "chain", "error": "need ops or text" }),
                );
            };
            let dt = doc.dt.unwrap_or(0.02);
            let report = driver.run_chain(&doc.ops, dt);
            tool_text(report.ok, serde_json::to_value(&report).unwrap())
        }
        other => tool_text(
            false,
            json!({ "ok": false, "code": "unknown_tool", "error": other }),
        ),
    }
}

fn tool_list() -> Vec<Value> {
    vec![
        tool(
            "mhs_open",
            "Open a catalog lab (coastal, harbor, inland, open_water). Replaces the live driver.",
            json!({
                "type": "object",
                "properties": {
                    "scenario": { "type": "string" },
                    "seed": { "type": "integer" }
                }
            }),
        ),
        tool(
            "mhs_discover",
            "List MHS-shaped devices in the current catalog. Does not step.",
            json!({ "type": "object", "properties": {} }),
        ),
        tool(
            "mhs_reference",
            "Compiled reference file (tags, measures, writes, safety limits). Optional device id.",
            json!({
                "type": "object",
                "properties": { "device": { "type": "string" } }
            }),
        ),
        tool(
            "mhs_read",
            "Read a channel. Does not step the plant.",
            json!({
                "type": "object",
                "required": ["device", "channel"],
                "properties": {
                    "device": { "type": "string" },
                    "channel": { "type": "string" }
                }
            }),
        ),
        tool(
            "mhs_write",
            "Write a channel through legal_cmds / attach. Rejects illegal and over-limit writes. Does not step.",
            json!({
                "type": "object",
                "required": ["device", "channel"],
                "properties": {
                    "device": { "type": "string" },
                    "channel": { "type": "string" },
                    "vn": { "type": "number" },
                    "ve": { "type": "number" },
                    "vd": { "type": "number" },
                    "yaw_rate": { "type": "number" }
                }
            }),
        ),
        tool(
            "mhs_step",
            "One or more WorldSession::step ticks (P12: one plant step per tick).",
            json!({
                "type": "object",
                "properties": {
                    "dt": { "type": "number" },
                    "n": { "type": "integer" }
                }
            }),
        ),
        tool(
            "mhs_chain",
            "Run a code file of write/read/step ops without per-tick reasoning.",
            json!({
                "type": "object",
                "properties": {
                    "ops": { "type": "array" },
                    "text": { "type": "string" }
                }
            }),
        ),
    ]
}

fn tool(name: &str, description: &str, input_schema: Value) -> Value {
    json!({
        "name": name,
        "description": description,
        "inputSchema": input_schema,
    })
}

/// Read newline-delimited JSON-RPC from `stdin` until EOF.
pub fn serve_stdio(mut driver: Driver) {
    use std::io::{self, BufRead, Write};
    let stdin = io::stdin();
    let mut stdout = io::stdout();
    for line in stdin.lock().lines() {
        let Ok(line) = line else {
            break;
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let req: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => {
                let _ = writeln!(
                    stdout,
                    "{}",
                    json!({
                        "jsonrpc": "2.0",
                        "id": null,
                        "error": { "code": -32700, "message": e.to_string() }
                    })
                );
                let _ = stdout.flush();
                continue;
            }
        };
        if let Some(resp) = handle_rpc(&mut driver, &req) {
            let _ = writeln!(stdout, "{resp}");
            let _ = stdout.flush();
        }
    }
}
