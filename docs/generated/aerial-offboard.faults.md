# AerialOffboard fault injection

`flight_sim::run_revoke_table` injects each `AerialOffboard::REVOKE_ON`
event through `WorldSession::inject_revoke` (`AerialOffboard::inject`
first) from an Offboard grant. A leftover `Vehicle<Offboard>` bound before
the inject cannot run `COMMANDS`
(`set_velocity`, `set_position`, `hold`) — `StaleAuthority`, still typed
Offboard. The same events appear as Offboard → Failsafe edges in
[`aerial-offboard.transitions.md`](aerial-offboard.transitions.md).

| Event | Kernel | Lab |
| --- | --- | --- |
| TriggerFailsafe | `event_revokes_authority` | `Fault::Failsafe` / HITL miss |
| Disarm | `event_revokes_authority` | revoke-table leftover Offboard |
| Disconnect | `event_revokes_authority` | revoke-table leftover Offboard |
| HeartbeatStale | `event_revokes_authority` | `Scenario::HEARTBEAT_LOSS` via `heartbeat_revoke_event` |
| EstimatorInvalid | `event_revokes_authority` | `Scenario::GPS_LOSS` via `Estimate::revoke_event`; `Scenario::IMU_DELAY` via `estimate_revoke_event` |
| ImuUnhealthy | `event_revokes_authority` | `Scenario::IMU_LOSS` / revoke-table leftover Offboard |

CLI: `cargo run -p flight-sim --bin flight-test -- --scenario revoke-table`
(world leftover Offboard + JSONL + ULog). PX4 companion leftover table:
`cargo run -p flight-px4 --bin flight-test-px4` (`Px4Backend::inject_revoke`
for every `REVOKE_ON` event; `flight-sim` does not depend on `flight-px4`).
ArduPilot GUIDED leftover table:
`cargo run -p flight-ardupilot --bin flight-test-ardupilot`
(`ArduPilotBackend::inject_revoke` for every `REVOKE_ON` event;
`flight-sim` does not depend on `flight-ardupilot`; live Copter is
loopback-only, no CI sitl job).
HITL leftover after a rack deadline/`Rate` miss, and leftover after every
`REVOKE_ON` through `WorldRack::inject_revoke`:
`cargo run -p flight-hitl --bin flight-test-hitl`
(`WorldRack::leftover_after_deadline_miss` / `run_hitl_revoke_table` /
`run_fch1_udp_mock`; `flight-sim` does not depend on `flight-hitl`). ROS 2 leftover after
`apply_failsafe`, `apply_disarm`, and every `REVOKE_ON`:
`cargo run -p flight-ros2 --bin flight-test-ros2`
(`plant::leftover_after_failsafe` / `leftover_after_disarm` / `run_ros2_revoke_table`;
`flight-sim` does not depend on `flight-ros2`; no rclrs).
