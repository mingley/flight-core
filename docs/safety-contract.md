# Safety contract (traceability)

Single-source tables live in `flight_core::contracts`. This file is the
human-readable copy those tables emit (`human_readable_spec()`). IDs are
stable; do not reuse a retired id.

## Capabilities

### FC-CAP-AerialOffboard

```
capability AerialOffboard {
  requires heartbeat.age < 250.ms();
  revokes_on [TriggerFailsafe, Disarm, Disconnect, HeartbeatStale, EstimatorInvalid, ImuUnhealthy]
}
```

Generates:

- Rust capability / typestate methods (`OffboardControl`, `acquire_offboard_control`)
- kernel [`event_revokes_authority`](../crates/flight-core/src/safety.rs)
- runtime monitors (`Requirement::OffboardHeartbeatFresh`, `PermitEpochMonotonic`)
- Kani harness `permit_epoch_mismatch_is_stale`
- Kani harness `dsl_revokes_match_kernel`
- mermaid (`AerialOffboard::MERMAID`, [`docs/generated/aerial-offboard.mmd`](generated/aerial-offboard.mmd))
- Graphviz (`AerialOffboard::GRAPHVIZ`)
- this specification
- [`docs/generated/traceability.md`](generated/traceability.md)

A unit test fails if the DSL revoke list and `event_revokes_authority` disagree.

## Invariants

| ID | Statement | Compile-time | Runtime |
| --- | --- | --- | --- |
| FC-INV-001 | `actuators_enabled → armed` | kernel `check_invariants`, Kani `actuators_require_arm` | `Requirement::ActuatorsImplyArmed` |
| FC-INV-002 | `ActuationPermit.epoch == backend.epoch` | non-`Clone` permit | `permit.check` at the backend boundary |
| FC-INV-003 | Offboard heartbeat age `< 250 ms` | `Fresh<(), 250>` / `HeartbeatFresh` | `heartbeat_age_ok`, `Requirement::OffboardHeartbeatFresh` |

## Diagram

```
stateDiagram-v2
    [*] --> Disarmed
    Disarmed --> PreflightReady: verify_preflight
    PreflightReady --> Armed: arm
    Armed --> Offboard: acquire_offboard_control
    Offboard --> Failsafe: TriggerFailsafe|Disarm|Disconnect|HeartbeatStale|EstimatorInvalid|ImuUnhealthy
```

## Fault laboratory

`flight_sim::scenario::Scenario::GPS_LOSS` injects `EstimatorInvalid` on the
verified world, then evaluates the same `Requirement` set. Replay is
`evaluate_trace` on a recorded `TraceSample` JSONL. PX4 SITL and ulog
conformance use that evaluator on converted traces — they do not require a
second contract language.
