# Traceability matrix

Generated from `flight_core::contracts` tables. IDs are stable.

| ID | Statement | Types / kernel | Runtime monitor | Kani | Scenario |
| --- | --- | --- | --- | --- | --- |
| FC-CAP-AerialOffboard | Offboard heartbeat `< 250 ms`; revoke on failsafe/disarm/disconnect/stale heartbeat/estimator/IMU | `vehicle_contract!`, `event_revokes_authority` | `Requirement::OffboardHeartbeatFresh` | `dsl_revokes_match_kernel` | `Scenario::HEARTBEAT_LOSS` |
| FC-INV-001 | `actuators_enabled → armed` | `check_invariants` | `Requirement::ActuatorsImplyArmed` | `actuators_require_arm` | GPS_LOSS, HEARTBEAT_LOSS, HITL miss |
| FC-INV-002 | `ActuationPermit.epoch == backend.epoch` | non-`Clone` permit, `permit.check` | `Requirement::PermitEpochMonotonic` | `permit_epoch_mismatch_is_stale` | world sibling failsafe |
| FC-INV-003 | Offboard heartbeat age `< 250 ms` | `HeartbeatFresh`, `heartbeat_age_ok` | `Requirement::OffboardHeartbeatFresh` | (runtime) | HEARTBEAT_LOSS |

Human-readable copy: [`safety-contract.md`](../safety-contract.md). Diagram: [`aerial-offboard.mmd`](aerial-offboard.mmd).
