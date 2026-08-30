# AerialOffboard transitions

Generated from `define_aerial_authority!` (`AERIAL_OFFBOARD_TRANSITIONS`).
Do not edit by hand; `revoke_table_is_the_named_set` token-checks each row.

| from | via | to |
| --- | --- | --- |
| Disarmed | verify_preflight | PreflightReady |
| PreflightReady | arm | Armed |
| Armed | acquire_offboard_control | Offboard |
| Offboard | TriggerFailsafe | Failsafe |
| Offboard | Disarm | Failsafe |
| Offboard | Disconnect | Failsafe |
| Offboard | HeartbeatStale | Failsafe |
| Offboard | EstimatorInvalid | Failsafe |
| Offboard | ImuUnhealthy | Failsafe |
