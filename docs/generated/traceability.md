# Traceability matrix

Generated from `flight_core::contracts` tables. IDs are stable.

| ID | Statement | Types / kernel | Runtime monitor | Kani | Scenario |
| --- | --- | --- | --- | --- | --- |
| FC-CAP-AerialOffboard | Offboard heartbeat `< 250 ms`; command age `< 100 ms`; revoke on failsafe/disarm/disconnect/stale heartbeat/estimator/IMU | `define_aerial_authority!`, `vehicle_contract! from_kernel`, `event_revokes_authority`, `TRANSITIONS` | `Requirement::OffboardHeartbeatFresh`, `CommandAgeMs`, `EpochBumped` | `dsl_revokes_match_kernel` | `Scenario::HEARTBEAT_LOSS`, `GPS_LOSS`, `HITL_MISS`, `revoke-table` |
| FC-INV-001 | `actuators_enabled → armed` | `check_invariants` | `Requirement::ActuatorsImplyArmed` | `actuators_require_arm` | GPS_LOSS, HEARTBEAT_LOSS, HITL miss |
| FC-INV-002 | `ActuationPermit.epoch == backend.epoch` | non-`Clone` permit, `permit.check` | `Requirement::PermitEpochMonotonic` | `permit_epoch_mismatch_is_stale` | world sibling failsafe |
| FC-INV-003 | Offboard heartbeat age `< 250 ms` | `HeartbeatFresh`, `heartbeat_age_ok` | `Requirement::OffboardHeartbeatFresh` | `dsl_revokes_match_kernel` | HEARTBEAT_LOSS |
| FC-INV-004 | Command age `< 100 ms` at actuation | `Timestamp`, `Command`, `command_age_ok` | `Requirement::CommandAgeMs` | `dsl_revokes_match_kernel` | GPS_LOSS, `apply_velocity_command_now` |
| FC-INV-005 | Estimator timestamps monotonic | `estimator_ts_monotonic` | `Requirement::EstimatorTimestampsMonotonic` | `dsl_revokes_match_kernel` | GPS_LOSS, ulog replay |

Human-readable copy: [`safety-contract.md`](../safety-contract.md). Diagram: [`aerial-offboard.mmd`](aerial-offboard.mmd) / [`aerial-offboard.dot`](aerial-offboard.dot). Transitions: [`aerial-offboard.transitions.md`](aerial-offboard.transitions.md). Faults: [`aerial-offboard.faults.md`](aerial-offboard.faults.md).
