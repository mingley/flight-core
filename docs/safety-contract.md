# Safety contract (traceability)

Single-source tables live in `define_aerial_authority!` (`safety.rs`) and
are aliased by `vehicle_contract! { from_kernel }`. This file is the
human-readable copy those tables emit (`human_readable_spec()`). IDs are
stable; do not reuse a retired id.

## Capabilities

### FC-CAP-AerialOffboard

```
capability AerialOffboard {
  requires heartbeat.age < 250.ms();
  requires command.age < 100.ms();
  revokes_on [TriggerFailsafe, Disarm, Disconnect, HeartbeatStale, EstimatorInvalid, ImuUnhealthy]
}
```

Generates (one expansion in the kernel, aliased by the capability type):

- kernel [`event_revokes_authority`](../crates/flight-core/src/safety.rs) / `heartbeat_age_ok` / `command_age_ok` / `admit_offboard_command` / `estimator_ts_monotonic`
- Creusot `ensures` on `event_revokes_authority` (same event list)
- Rust capability `AerialOffboard` (`REVOKE_ON` is `AUTHORITY_REVOKE_EVENTS`; `admit` is `admit_offboard_command`)
- typestate methods (`OffboardControl`, `admit_offboard_now`, `set_velocity_now` / `set_position_now` / `hold_now`)
- runtime monitors (`Requirement::OffboardHeartbeatFresh`, `CommandAgeMs`, `EstimatorTimestampsMonotonic`, `EpochBumped`)
- Kani harness `dsl_revokes_match_kernel` via `prove_aerial_authority!` (table membership + bounds + admit)
- Kani harness `permit_epoch_mismatch_is_stale`
- mermaid (`AerialOffboard::MERMAID`, [`docs/generated/aerial-offboard.mmd`](generated/aerial-offboard.mmd))
- Graphviz (`AerialOffboard::GRAPHVIZ`, [`docs/generated/aerial-offboard.dot`](generated/aerial-offboard.dot))
- transition table (`AerialOffboard::TRANSITIONS`, [`docs/generated/aerial-offboard.transitions.md`](generated/aerial-offboard.transitions.md))
- fault-injection list (`REVOKE_ON` / `run_revoke_table`, [`docs/generated/aerial-offboard.faults.md`](generated/aerial-offboard.faults.md))
- compile-fail names (`AerialOffboard::UI_FORBIDDEN`, including `unsafe_mission.rs`)
- this specification (`AerialOffboard::SPEC`, [`docs/generated/aerial-offboard.spec.txt`](generated/aerial-offboard.spec.txt))
- [`docs/generated/traceability.md`](generated/traceability.md)

A unit test fails if `AerialOffboard::REVOKE_ON` is not the kernel table.

## Invariants

| ID | Statement | Compile-time | Runtime |
| --- | --- | --- | --- |
| FC-INV-001 | `actuators_enabled → armed` | kernel `check_invariants`, Kani `actuators_require_arm` | `Requirement::ActuatorsImplyArmed` |
| FC-INV-002 | `ActuationPermit.epoch == backend.epoch` | non-`Clone` permit | `permit.check` at the backend boundary |
| FC-INV-003 | Offboard heartbeat age `< 250 ms` | `Fresh<(), 250>` / `HeartbeatFresh` | `heartbeat_age_ok`, `Requirement::OffboardHeartbeatFresh` |
| FC-INV-004 | Command age `< 100 ms` at actuation | `Command` / `Timestamp` / `command_age_ok` | `Requirement::CommandAgeMs`, `apply_velocity_command_now` |
| FC-INV-005 | Estimator timestamps never jump backward | `estimator_ts_monotonic` | `Requirement::EstimatorTimestampsMonotonic` |

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
`evaluate_trace` on a recorded `TraceSample` JSONL **or** a native ULog
(`fc_trace`, plus a `vehicle_status` subset). PX4 SITL conformance uses
that evaluator on converted traces — they do not require a second contract
language.
