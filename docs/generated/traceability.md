# Traceability matrix

Generated from `flight_core::contracts` tables. IDs are stable.

| ID | Statement | Types / kernel | Runtime monitor | Kani | Scenario |
| --- | --- | --- | --- | --- | --- |
| FC-CAP-AerialOffboard | Offboard heartbeat `< 250 ms`; command age `< 100 ms`; revoke on failsafe/disarm/disconnect/stale heartbeat/estimator/IMU | `define_aerial_authority!`, `vehicle_contract! from_kernel`, `event_revokes_authority`, `admit_offboard_command`, `AerialOffboard::admit`, `inject`, `TRANSITIONS` | `Requirement::OffboardHeartbeatFresh`, `CommandAgeMs`, `EpochBumped` | `dsl_revokes_match_kernel` (`prove_aerial_authority!`) | `Scenario::HEARTBEAT_LOSS`, `GPS_LOSS`, `HITL_MISS`, `revoke-table` leftover Offboard `COMMANDS`, `differential_revoke_table`, `differential_contract`, PX4 `inject_revoke` / `flight-test-px4`, HITL `leftover_after_deadline_miss` / `run_hitl_revoke_table` / `flight-test-hitl`, ROS 2 `leftover_after_failsafe` / `leftover_after_disarm` / `run_ros2_revoke_table` / `flight-test-ros2` |
| FC-INV-001 | `actuators_enabled → armed` | `check_invariants` | `Requirement::ActuatorsImplyArmed` | `actuators_require_arm` | GPS_LOSS, HEARTBEAT_LOSS, HITL miss |
| FC-INV-002 | `ActuationPermit.epoch == backend.epoch` | non-`Clone` permit, `permit.check` | `Requirement::PermitEpochMonotonic` | `permit_epoch_mismatch_is_stale` | world sibling failsafe, reconnect leftover Offboard |
| FC-INV-003 | Offboard heartbeat age `< 250 ms` | `HeartbeatFresh`, `heartbeat_age_ok`, `heartbeat_revoke_event` | `Requirement::OffboardHeartbeatFresh` | `dsl_revokes_match_kernel` | HEARTBEAT_LOSS |
| FC-INV-004 | Command age `< 100 ms` at actuation | `Timestamp`, `Command`, `Deadline::for_command`, `command_age_ok`, HITL `Rate::admits` | `Requirement::CommandAgeMs` | `dsl_revokes_match_kernel` | GPS_LOSS, `apply_velocity_command_now`, HITL rack `Rate` |
| FC-INV-005 | Estimator timestamps monotonic | `estimator_ts_monotonic`, `Estimate::revoke_event` | `Requirement::EstimatorTimestampsMonotonic` | `dsl_revokes_match_kernel` | GPS_LOSS, ulog replay |

Human-readable copy: [`safety-contract.md`](../safety-contract.md). Spec: [`aerial-offboard.spec.txt`](aerial-offboard.spec.txt). Diagram: [`aerial-offboard.mmd`](aerial-offboard.mmd) / [`aerial-offboard.dot`](aerial-offboard.dot). Transitions: [`aerial-offboard.transitions.md`](aerial-offboard.transitions.md). Faults: [`aerial-offboard.faults.md`](aerial-offboard.faults.md) / [`aerial-offboard.faults.txt`](aerial-offboard.faults.txt). Creusot listing: [`aerial-offboard.creusot.txt`](aerial-offboard.creusot.txt).
