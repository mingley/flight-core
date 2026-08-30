# Copper: complement, do not compete

Copper is a substantial Rust robotics runtime: deterministic execution and
replay, sim-to-hardware, ROS interop, embedded support, and a flight-controller
demonstration. flight-core is **not** another deterministic robotics runtime.

## Boundary

| Copper | flight-core |
| --- | --- |
| Scheduler, tasks, `cu_transform` frame ids, replay of the Copper graph | Verified capability/contract TCB, revocable permits, aerial/ground/marine safety kernels |
| Run the robot | Decide whether a command has physical authority |

Integration beats replacement. Do not add a generic scheduler, pub/sub, or
physics engine here because Copper already has them.

## Interop (intended)

1. A Copper task holds `Vehicle<S, WorldBackend>` (or PX4) and may only call
   methods the typestate exposes.
2. Setpoints that leave Copper still pass `safety::step` and
   `ActuationPermit::check` at the backend boundary.
3. `cu_transform` frame ids can map onto `flight_core::geometry::Transform<A, B>`
   at the edge. Do not duplicate Copper’s transform graph inside flight-core.
4. Copper replay of task I/O can be converted to `contracts::TraceSample` and
   evaluated with the same `Requirement` set as `flight_sim::scenario`.

There is no `copper` crate dependency in this workspace. Adding one is an
integration crate later, not a fork of Copper’s runtime.
