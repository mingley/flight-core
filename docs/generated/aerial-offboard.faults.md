# AerialOffboard fault injection

`flight_sim::run_revoke_table` injects each `AerialOffboard::REVOKE_ON`
event through `AerialOffboard::inject` from an Offboard grant. A leftover
`Vehicle<Offboard>` bound before the inject cannot run `COMMANDS`
(`set_velocity`, `set_position`, `hold`) — `StaleAuthority`, still typed
Offboard. The same events appear as Offboard → Failsafe edges in
[`aerial-offboard.transitions.md`](aerial-offboard.transitions.md).

| Event | Kernel | Lab |
| --- | --- | --- |
| TriggerFailsafe | `event_revokes_authority` | `Fault::Failsafe` / HITL miss |
| Disarm | `event_revokes_authority` | revoke-table leftover Offboard |
| Disconnect | `event_revokes_authority` | revoke-table leftover Offboard |
| HeartbeatStale | `event_revokes_authority` | `Scenario::HEARTBEAT_LOSS` via `heartbeat_revoke_event` |
| EstimatorInvalid | `event_revokes_authority` | `Scenario::GPS_LOSS` via `Estimate::revoke_event` |
| ImuUnhealthy | `event_revokes_authority` | revoke-table leftover Offboard |

CLI: `cargo run -p flight-sim --bin flight-test -- --scenario revoke-table`
(world leftover Offboard + JSONL + ULog).
