# AerialOffboard fault injection

`flight_sim::run_revoke_table` injects each `AerialOffboard::REVOKE_ON`
event from an Offboard grant. The same events appear as Offboard → Failsafe
edges in [`aerial-offboard.transitions.md`](aerial-offboard.transitions.md).

| Event | Kernel | Lab |
| --- | --- | --- |
| TriggerFailsafe | `event_revokes_authority` | `Fault::Failsafe` / HITL miss |
| Disarm | `event_revokes_authority` | revoke-table |
| Disconnect | `event_revokes_authority` | revoke-table |
| HeartbeatStale | `event_revokes_authority` | `Scenario::HEARTBEAT_LOSS` |
| EstimatorInvalid | `event_revokes_authority` | `Scenario::GPS_LOSS` |
| ImuUnhealthy | `event_revokes_authority` | revoke-table |

CLI: `cargo run -p flight-sim --bin flight-test -- --scenario revoke-table`
