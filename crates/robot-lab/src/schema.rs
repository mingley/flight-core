//! Locked JSON Schema for lab observe / act (NEXT A2).
//!
//! Documents live in `crates/robot-lab/schemas/`. Foxglove bags embed the
//! observation and timed-action schemas. A crate test validates coastal bags
//! and `examples/bag.rs`-shaped output against them.

use serde_json::Value;

pub const OBSERVATION_SCHEMA: &str = include_str!("../schemas/observation.json");
pub const TIMED_ACTION_SCHEMA: &str = include_str!("../schemas/timed_action.json");
pub const AGENT_ACTION_SCHEMA: &str = include_str!("../schemas/agent_action.json");

/// Validate `instance` against a JSON Schema document (draft subset).
pub fn validate_instance(schema_json: &str, instance: &Value) -> Result<(), String> {
    let schema: Value =
        serde_json::from_str(schema_json).map_err(|e| format!("schema parse: {e}"))?;
    apply(&schema, &schema, instance, "$")
}

fn apply(root: &Value, schema: &Value, instance: &Value, path: &str) -> Result<(), String> {
    if let Some(r) = schema.get("$ref").and_then(Value::as_str) {
        let resolved = resolve(root, r)?;
        return apply(root, resolved, instance, path);
    }

    if let Some(types) = schema.get("type") {
        type_ok(types, instance, path)?;
    }
    if let Some(enum_vals) = schema.get("enum").and_then(Value::as_array) {
        if !enum_vals.iter().any(|v| v == instance) {
            return Err(format!("{path}: {instance} not in enum"));
        }
    }

    if instance.is_null() {
        return Ok(());
    }

    if let Some(obj) = instance.as_object() {
        if schema_includes_type(schema, "object") {
            if let Some(required) = schema.get("required").and_then(Value::as_array) {
                for key in required {
                    let name = key
                        .as_str()
                        .ok_or_else(|| format!("{path}: required name"))?;
                    if !obj.contains_key(name) {
                        return Err(format!("{path}: missing required '{name}'"));
                    }
                }
            }
            let props = schema.get("properties").and_then(Value::as_object);
            let additional = schema
                .get("additionalProperties")
                .and_then(Value::as_bool)
                .unwrap_or(true);
            for (key, val) in obj {
                match props.and_then(|p| p.get(key)) {
                    Some(sub) => apply(root, sub, val, &format!("{path}.{key}"))?,
                    None if additional => {}
                    None => return Err(format!("{path}: unexpected field '{key}'")),
                }
            }
        }
    }

    if let Some(arr) = instance.as_array() {
        if schema_includes_type(schema, "array") {
            if let Some(n) = schema.get("minItems").and_then(Value::as_u64) {
                if (arr.len() as u64) < n {
                    return Err(format!("{path}: minItems {n}"));
                }
            }
            if let Some(n) = schema.get("maxItems").and_then(Value::as_u64) {
                if (arr.len() as u64) > n {
                    return Err(format!("{path}: maxItems {n}"));
                }
            }
            if let Some(items) = schema.get("items") {
                for (i, el) in arr.iter().enumerate() {
                    apply(root, items, el, &format!("{path}[{i}]"))?;
                }
            }
        }
    }

    Ok(())
}

fn schema_includes_type(schema: &Value, want: &str) -> bool {
    match schema.get("type") {
        Some(Value::String(s)) => s == want,
        Some(Value::Array(ts)) => ts.iter().any(|t| t.as_str() == Some(want)),
        None => want == "object" && schema.get("properties").is_some(),
        _ => false,
    }
}

fn type_ok(types: &Value, instance: &Value, path: &str) -> Result<(), String> {
    let allowed: Vec<&str> = match types {
        Value::String(s) => vec![s.as_str()],
        Value::Array(a) => a.iter().filter_map(Value::as_str).collect(),
        _ => return Err(format!("{path}: invalid type keyword")),
    };
    let ok = allowed.iter().any(|t| match *t {
        "object" => instance.is_object(),
        "array" => instance.is_array(),
        "string" => instance.is_string(),
        "boolean" => instance.is_boolean(),
        "null" => instance.is_null(),
        "number" => instance.is_number(),
        "integer" => {
            instance.as_i64().is_some()
                || instance.as_u64().is_some()
                || instance.as_f64().is_some_and(|f| f.fract() == 0.0)
        }
        _ => false,
    });
    if ok {
        Ok(())
    } else {
        Err(format!("{path}: expected type {allowed:?}, got {instance}"))
    }
}

fn resolve<'a>(root: &'a Value, ptr: &str) -> Result<&'a Value, String> {
    let ptr = ptr
        .strip_prefix('#')
        .ok_or_else(|| format!("only local $ref supported: {ptr}"))?;
    let mut cur = root;
    for part in ptr.split('/').filter(|p| !p.is_empty()) {
        cur = cur
            .get(part)
            .ok_or_else(|| format!("$ref {ptr} missing '{part}'"))?;
    }
    Ok(cur)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{action_json, observation_json, AgentAction, Lab, LabCmd, McapBag, TimedAction};

    fn lab_cmd_enum() -> Vec<&'static str> {
        LabCmd::ALL.into_iter().map(LabCmd::as_str).collect()
    }

    fn schema_enum(schema: &str) -> Vec<String> {
        let v: Value = serde_json::from_str(schema).unwrap();
        let cmds = v
            .pointer("/$defs/lab_cmd/enum")
            .or_else(|| v.pointer("/properties/cmd/enum"))
            .and_then(Value::as_array)
            .expect("lab_cmd enum");
        cmds.iter()
            .map(|c| c.as_str().unwrap().to_string())
            .collect()
    }

    #[test]
    fn schemas_lock_lab_cmd_enum_to_rust() {
        let rust: Vec<String> = lab_cmd_enum().into_iter().map(str::to_string).collect();
        for doc in [OBSERVATION_SCHEMA, TIMED_ACTION_SCHEMA, AGENT_ACTION_SCHEMA] {
            assert_eq!(schema_enum(doc), rust, "schema drifted from LabCmd::ALL");
        }
    }

    #[test]
    fn observation_schema_states_ned_z_down_and_optional_hold() {
        assert!(OBSERVATION_SCHEMA.contains("z-down"));
        assert!(OBSERVATION_SCHEMA.contains("hold_ned"));
        assert!(OBSERVATION_SCHEMA.contains("legal_cmds"));
        assert!(OBSERVATION_SCHEMA.contains("not the plant phase string"));
        assert!(OBSERVATION_SCHEMA.contains("Refuse is atomic"));
        let v: Value = serde_json::from_str(OBSERVATION_SCHEMA).unwrap();
        let required = v["required"].as_array().unwrap();
        assert!(!required.iter().any(|k| k == "hold_ned"));
        assert!(required.iter().any(|k| k == "broken"));
        let robot_required = v["$defs"]["robot_view"]["required"].as_array().unwrap();
        assert!(!robot_required.iter().any(|k| k == "hold_ned"));
        assert!(robot_required.iter().any(|k| k == "legal_cmds"));
        let aerial_req = v["$defs"]["aerial_machine"]["required"].as_array().unwrap();
        assert!(aerial_req.iter().any(|k| k == "kind"));
        assert!(aerial_req.iter().any(|k| k == "phase"));
        assert!(aerial_req.iter().any(|k| k == "imu_healthy"));
        assert!(aerial_req.iter().any(|k| k == "estimator_valid"));
    }

    #[test]
    fn coastal_observation_and_action_match_schema() {
        let mut lab = Lab::coastal(1);
        lab.step(0.02);
        let obs = serde_json::to_value(lab.observe()).unwrap();
        validate_instance(OBSERVATION_SCHEMA, &obs).expect("coastal observation");

        lab.attach_takeoff("drone").unwrap();
        lab.attach_hold("drone").unwrap();
        let held = serde_json::to_value(lab.observe()).unwrap();
        validate_instance(OBSERVATION_SCHEMA, &held).expect("held observation");
        assert!(held["robots"]
            .as_array()
            .unwrap()
            .iter()
            .any(|r| r["id"] == "drone" && r.get("hold_ned").is_some()));

        let act = serde_json::to_value(AgentAction::new("drone", LabCmd::Hold)).unwrap();
        validate_instance(AGENT_ACTION_SCHEMA, &act).expect("agent action");
        let timed = serde_json::to_value(&TimedAction {
            t: 0.02,
            action: AgentAction::new("drone", LabCmd::Hold),
        })
        .unwrap();
        validate_instance(TIMED_ACTION_SCHEMA, &timed).expect("timed action");
    }

    #[test]
    fn coastal_bag_and_bag_example_loop_match_schema() {
        let mut lab = Lab::coastal(1);
        let mut bag = McapBag::new(Vec::new()).unwrap();
        for k in 0..40 {
            lab.apply_script();
            lab.step(0.02);
            if k % 5 == 0 {
                bag.write_observation(&lab.observe()).unwrap();
            }
        }
        for action in &lab.log {
            bag.write_action(action).unwrap();
        }
        let bytes = bag.finish().unwrap();
        for msg in observation_json(&bytes).unwrap() {
            validate_instance(OBSERVATION_SCHEMA, &msg).expect("bag observation");
        }
        for msg in action_json(&bytes).unwrap() {
            validate_instance(TIMED_ACTION_SCHEMA, &msg).expect("bag action");
        }
        assert!(
            !observation_json(&bytes).unwrap().is_empty(),
            "bag example loop must write observations"
        );
    }
}
