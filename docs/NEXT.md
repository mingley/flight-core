# Next steps

Ordered work toward [`docs/agentic-spec.md`](agentic-spec.md). Technical dependence, not calendar. An item is done only when **acceptance** is true against the tree, tests, proofs, and (where named) a recorded run.

**Always on:** keep v0 green. Do not regress [`docs/remaining-spec.md`](remaining-spec.md) **P1–P14**. Atomic commits on `main`. No PR pile-up unless someone asks. MSRV 1.85. After demo HTML/`include_str` changes, re-run remaining-spec §8 D2.

v0 functional gaps in remaining-spec are **landed**. This file is the backlog from here.

---

## Phase 0 — Keep the slice honest

| ID | Work | Acceptance |
| --- | --- | --- |
| 0.1 | Invariant CI | fmt, clippy `-D warnings`, workspace tests, `flight-core --no-default-features`, gpu, kani (harness count in lockstep with `flight-verify`), rclrs, creusot (81 libraries on 0.5.0), sitl (`sitl_live --ignored` on `px4io/px4-sitl:v1.18.0-beta2`) stay required. |
| 0.2 | New operator act | Typed agent **and** JSON probe twin. Catalog skips for omitted bodies (P11). |
| 0.3 | New kernel event | trybuild + verify theorems + do not collapse P1–P9. |
| 0.4 | Docs | README “Still ahead” points here and at the north-star spec. Do not mention private remotes or temporary clone names. |

---

## Phase A — Agentic surface (do this first)

Goal: an agent can experiment and understand **without** reading kernel source, and **cannot** smuggle illegal motion.

### A1. Legal-command tool adapter

**Status: landed.** `Observation::tools` / `Lab::legal_tools` enumerate `(robot_id, cmd)` from `legal_cmds` plus `env_cmds`. `Lab::act` / `Lab::act_through_attach` / replay reject `unknown robot` and `not legal now` before kernel or attach. Ready `Takeoff` remains an attach grant on `act_through_attach` only (P2: kernel Takeoff from Ready stays illegal on `act`). P6 JSON Failsafe Disarm → Recovery is unchanged.

**Why:** `legal_cmds` already exists on `RobotView`. Agents still post arbitrary `LabCmd` strings and discover Protocol after the fact.

**Acceptance:**

1. A Rust API (and JSON) that, given an `Observation`, returns the **only** callable robot tools: `(robot_id, cmd)` pairs from `legal_cmds`, plus `env_cmds`.
2. `Lab::act` / `act_through_attach` reject `cmd` not in that set **before** domain attach, with a structured error (`unknown robot`, `not legal now`, kernel reject unchanged).
3. Tests: parked `drive`, docked `thrust`, pad `hold`, inland `undock` (no hull), open_water `drive` (no rover) — all rejected as not-legal, not as a crash.
4. Does not change P6 (JSON Failsafe Disarm vs PX4 DISARM).

### A2. JSON Schema for observe / act

**Status: landed.** Documents in `crates/robot-lab/schemas/` (`observation.json`, `agent_action.json`, `timed_action.json`) are locked to `LabCmd::ALL`. Crate tests validate coastal observations, timed actions, and `examples/bag.rs`-shaped MCAP output. `hold_ned` is optional; `legal_cmds` / `cmd` are closed enums; NED z-down is in the schema descriptions.

**Why:** bags and HTTP are implied contracts. Agents and Foxglove need an explicit one.

**Acceptance:**

1. Schema documents for `Observation` and `AgentAction` / `TimedAction`, generated from or locked to the Rust types.
2. CI (or a crate test) validates a coastal bag plus `examples/bag.rs` output against the schema.
3. Field `hold_ned` remains optional; `legal_cmds` remains an array of closed `LabCmd` strings; NED z-down is stated in the schema description.
4. No silent rename of v0 field names.

### A3. Closed-loop experiment runner

**Status: landed.** `cargo run -p robot-lab --example run` (library: [`Experiment`]) takes `scenario`, seed / seed list / `--from`--`--to`, `dt`, `steps`, typed `--agent` or `--jsonl`, and writes a run directory (`run.json` with `ResearchRun` + git commit, observations JSONL, actions JSONL, optional MCAP). Exit 1 if `all_hold` is false or `--require-property` fails. Harbor seed sweep (`typed-fleet-hold`, seeds 1 and 3) is a crate test. Each tick is still one `WorldSession::step` (P12).

**Why:** `Lab::research` is a library. World-class tooling is a repeatable **run**.

**Acceptance:**

1. CLI or example: `scenario`, `seed` or seed range, `dt`, `steps`, agent name (typed) or JSONL script.
2. Writes a run directory: `run.json` (`ResearchRun` + git commit if available), observations JSONL, actions JSONL, optional MCAP.
3. Non-zero exit if `all_hold` is false or if a named `--require-property` fails.
4. Seed sweep: at least two seeds on `harbor` with `TypedFleetHold` (or successor) both green.
5. Still one `WorldSession::step` per tick (P12).

### A4. Structured rejection traces

**Status: landed.** `RejectTrace` serializes domain, robot, cmd, from phase/kind, attempted kernel event, reject display, code, and remaining-spec id when the bounce is P1–P13. `Lab::last_reject` holds the latest failed act (cleared on success); observation `message` is `agent rejected: …`. `research_probe` fills `illegal_traces`; typed/JSON research fills `ResearchRun.rejects`. Observation schema is unchanged (`additionalProperties: false`). P6 JSON Failsafe Disarm → Recovery is unchanged.

**Why:** understanding is “why did that fail,” not `Protocol`.

**Acceptance:**

1. Attach/act failures serialize: domain, robot, cmd, from phase/kind, attempted event, reject enum display, and if applicable the invariant id (P1–P13) documented for that split.
2. `research_probe` / typed illegal probes include those traces in the report.
3. Demo or observation may surface the last reject; do not require a new GUI product.

### A5. Richer observations for agents

**Status: landed.** `Observation.broken` / `Lab::broken` list the property ids from the last `try_step` in vector order (first failed id first). Refuse is atomic: pose, hydro, and `t` stay; `last_properties` is the rejected vector. Observe does not step. Schema keeps `kind` vs `phase`; aerial `imu_healthy` / `estimator_valid` stay required.

**Why:** plant truth is there; agents still reverse-engineer attach kind vs phase.

**Acceptance:**

1. Observation (or a documented sub-object) includes: property ids that **would** be the first to break on a refused `try_step` when a debug/refuse path exists; otherwise document that refuse is atomic (`all_hold` + `broken`).
2. Keep `kind` vs `phase` in schema and README.
3. IMU health / `estimator_valid` remain visible on aerial machines.
4. No extra `step` to compute observations.

### A6. Local tool server (optional adapter)

**Status: landed.** `flight-demo` binds `FLIGHT_DEMO_BIND` (default `0.0.0.0:47831`) with no auth. GET `/api/lab/observation`, GET `/api/lab/tools` (A1 `legal_cmds` / `env_cmds`), POST `/api/lab/action` (A1 `act_through_attach`), GET `/api/lab/replay`, POST `/api/lab/research` (closed-loop `Lab::research`, one `WorldSession::step` per tick). MHS-shaped (E1, not official): GET `/api/mhs/discover`, GET `/api/mhs/reference`, POST `/api/mhs/read`, POST `/api/mhs/write` (preview `legal_cmds` + numeric limits, then the same pending queue as `/api/lab/action`). No raw NED velocity route. HTML/`include_str` unchanged (no remaining-spec §8 D2).

**Why:** LLM agents speak HTTP/MCP. The lab must not grow auth or cloud.

**Acceptance:**

1. Optional binary or `flight-demo` routes: observe, list legal tools, act, step/research tick, replay metadata.
2. Binds locally (reuse `FLIGHT_DEMO_BIND` or a sibling port). No authentication product. No Vercel.
3. Tools call A1; they do not expose a raw “set NED velocity” that skips `legal_cmds`.
4. If HTML changes, remaining-spec §8 D2.

---

## Phase B — Control depth

Goal: every domain can **hold and move** under the same typestate story; companions stay one API.

### B1. Ground pose hold

**Status: landed.** `GroundVehicle<Moving>::hold_now` + trybuild Parked / EStop. Plant field is the same NED `hold_ned` / `position_hold_restores_pose` as aerial. Restore fact is existing Kani `hold_velocity_restores_pose` (f32; no second harness). `LabCmd::Hold` + `legal_cmds` + `TypedGroundHold` + JSON probe (parked Hold stays not-legal; Moving Hold is legal). Inland / coastal / harbor include the rover; open_water skips (P11). Halt, E-stop, empty battery, and ungranted `clear_command()` wipe hold (P13 spirit, remaining-spec §5.4 A). `LabCmd::Position` remains aerial-only.

**Acceptance:**

1. Compile-fail: no hold from Parked / EStop (trybuild).
2. Plant field analogous to aerial `hold_ned` (or documented ground-frame equivalent) and a named property “command restores pose” when set.
3. Kani-style restore fact or an explicit “integer/kernel only” write-up if f32 lands in Kani like aerial hold.
4. `LabCmd` + `legal_cmds` + typed agent + JSON probe. Inland included; open_water skips rover (P11).
5. Ungranted wipe policy written (follow P13 spirit or a documented B-choice). Do not silently persist hold while ungranted.

### B2. Marine dynamic positioning (NED pose hold)

**Status: landed.** Distinct from `StationKeep`. `MarineVehicle` `hold_now` on `CanThrust` (Underway or StationKeep); Docked / Failsafe compile-fail. Plant field is `hold_ned` / `position_hold_restores_pose`; Kani `hold_velocity_restores_pose` (no second f32 harness). `LabCmd::Hold` + `attach_marine_hold` + `TypedMarineHold` + JSON probe. Inland skips hulls; coastal / harbor / open_water include skiff and surveyor. Dock, failsafe, and ungranted `clear_command()` wipe hold. P3 stands: no `declare_failsafe` on Docked; no dock from Failsafe.

**Acceptance:**

1. Pose target for Underway or StationKeep only; compile-fail from Docked / Failsafe.
2. Plant property + typed agent + JSON probe. Inland skips hulls; coastal/harbor/open_water include skiff and surveyor as specified.
3. Do not add `declare_failsafe` on `Docked`. Do not dock from Failsafe in typestate.

### B3. Estimation loop trips `estimator_valid`

**Status: landed.** `WorldSession::update_nav` / `Lab::update_nav` feed `ComplementaryAttitude`. Unusable IMU (`SensorHealth::Invalid`, non-finite, bad `dt`) posts `Event::EstimatorInvalid`, which clears kernel `estimator_valid` and latches failsafe if armed. Filter warm-up (fewer than eight good samples) does not trip. The plant quaternion is never written; `unit_attitude` stays `mech::quat_integrate`. `ComplementaryAttitude` is not in `World::try_step`. Fuzzed plant IMU stays finite and does not trip. Full ESKF/GNSS fusion is B3b.

**Acceptance:**

1. A navigation update (Mahony/complementary or successor) **may** clear `estimator_valid` on bad IMU without writing plant quaternion.
2. Plant `unit_attitude` remains physics-truth from `mech::quat_integrate`.
3. Kani or lab test: unusable IMU ⇒ failsafe path still `all_hold` (v0 fuzzed IMU stays).
4. Full ESKF/GNSS fusion may land in a later B3b; not required to close B3.

### B4. Typed planning layer

**Status: landed.** `flight_core::plan::{Waypoint, NedPath}` are NED-meter data (eight-point capacity, no allocation). Execution is attach + `set_position` / `set_velocity` / drive / thrust: aerial OffboardControl only, ground Moving only, marine `CanThrust` only. `TypedPathFollow` takes off and follows a two-point path; properties hold; the log replays. JSON probe twin is two `LabCmd::Position` acts after takeoff. No kernel path event.

**Acceptance:**

1. A path/waypoint type in Rust (NED, units).
2. Execution is a sequence of legal `set_position` / `set_velocity` / hold / drive / thrust through attach — no kernel bypass.
3. Aerial: OffboardControl only. Ground: Moving only. Marine: `CanThrust` only.
4. Typed agent follows a two-point path; properties hold; replay works.

### B5. Multi-vehicle coordination certificates

**Status: landed.** Lab certificate `fleet_hold_simultaneous` (stable id, not a try_step property): drone `hold_ned` is set, and a present skiff is StationKeep. Inland omits the hull; open_water omits the rover (P11). `TypedFleetHold` issues it on every catalog. Plant vector stays 22. Each research tick is still one `WorldSession::step` (P12).

**Acceptance:**

1. At least one named property or research certificate beyond pairwise sphere contact: e.g. minimum spacing, or “fleet hold simultaneously” already in `TypedFleetHold` promoted to a plant or lab assertion with a stable id.
2. Catalog skips remain correct (no fake hull inland).
3. P12: still one world step after flushing all grants.

### B6. Additional autopilot backend

**Acceptance:**

1. One more companion (e.g. ArduPilot MAVLink) implementing the same `VehicleBackend` (and domain backends if applicable).
2. Disconnected send is `BackendError::Disconnected`.
3. Documented SITL recipe; live test `#[ignore]` + CI job **or** an explicit “loopback only” decision in remaining-spec style.
4. Do not send AUTO takeoff that fights typestate velocity climb (PX4 lesson: stay in offboard for `takeoff_now`).

### B7. Physical FCH1 recorded run

**Acceptance:**

1. One recorded log against a real card **or** a faithful UDP mock that is not the in-process plant (remaining-spec §4.4 full bar).
2. `apply == 0` still zeros the slot (`RackCommand::from_fch1`). Slot map 0–3 unchanged unless documented with catalog bodies.

### B8. `no_std` kernel deploy (not typestate)

**Acceptance:**

1. Discrete aerial/ground/marine/HITL machines remain usable `no_std` (already the `--no-default-features` story).
2. A documented firmware or `no_std` example ticks `step` / `ground_step` / `marine_step` on host or a board.
3. Vehicles stay `std` unless a separate decision lifts remaining-spec §6.1. Do not bump MSRV.

---

## Phase C — Understanding

### C1. Proof artifacts as agent input

**Acceptance:**

1. A checked-in or generated summary: Creusot crate list + “f32 stays Kani” (hold, buoyancy, hydro mass, HITL miss-zero).
2. Experiment runner (A3) can copy or hash that summary into `run.json`.
3. Harness count and Creusot library count stay in lockstep with README when they change.

### C2. Causal / property traces

**Acceptance:**

1. When `try_step` refuses, the caller can read which property id failed (already implied by the vector — expose it on `Lab` without panicking away the world).
2. ResearchRun `broken` stays the list of failed ids; tests cover at least one induced break in a **clone** (do not ship a catalog that fails).

### C3. Scenario DSL

**Acceptance:**

1. A Rust (not YAML-as-source-of-truth) way to name a scene: catalog or custom body set, seed, wind/current/waves, charges.
2. Custom body sets cannot put a rover in `open_water` or a hull in `inland` **using those catalog names**. New names are new catalogs with an explicit body table.
3. Typed fleet agents skip missing bodies.

### C4. Scale without dropping properties

**Acceptance:**

1. Any hydro resolution change keeps `hydro_height_nonnegative`, `hydro_volume_conserved`, `hydro_land_stays_dry`.
2. Any extra body keeps contact properties and P12.
3. GPU remains optional performance; no CPU/GPU bit-identity requirement.

---

## Phase D — More domains and morphologies

Only after A is usable and B1–B2 have a written status (landed or explicitly deferred in this file).

| ID | Work | Acceptance |
| --- | --- | --- |
| D1 | Manipulator typestate | Consume-self gripper/arm states; compile-fails; contact with existing sphere/terrain properties or a new named property; lab cmd + typed agent. |
| D2 | Extra airframes | VTOL or fixed-wing as a **new** aerial body kind or documented Vehicle configuration; NED z-down; trybuild for new illegal transitions; catalog entry with P11-style body table. |
| D3 | Extra ground morphologies | Tracked or second rover; hold (B1) applies or is explicitly N/A. |
| D4 | Depth hold for AUV | Distinct from surface station if the machine needs it; compile-fail from Docked; property + agent. |
| D5 | N-body catalogs | More than four bodies with fleet certificates (B5); omit-body rules documented. |

---

## Phase E — Model Hardware Standard (shaped adapter)

Official [MHS](https://modelhardwarestandard.com) is a gated research preview (Anthropic + HHMI Janelia). Schemas are not public. This workspace does **not** claim official certification and does **not** guess a private wire format.

It ships an **MHS-shaped** driver so agents can use the public shape — standardized discoverable devices, tags compiled into a reference file, read/write primitives, CLI / HTTP / stdio MCP, safety at the driver — **more efficiently and verifiably** than prose tags: every write is `Lab::act_through_attach`, numeric limits reject before the plant, catalog skips stay P11, chain files step once per tick (P12).

### E1. MHS-shaped driver

**Status: landed.** Crate `flight-mhs`: `Driver` + compiled `DeviceReference`, CLI `flight-mhs`, stdio MCP (`tools/list` / `tools/call`), demo `GET /api/mhs/discover` / `reference` / `POST /api/mhs/read` / `POST /api/mhs/write`. `official: false`, `conformance: "shaped"`.

**Acceptance:**

1. Honest conformance: `official` is false; profile is `flight-core.mhs-shaped.v0`.
2. Discovery lists catalog bodies plus `env` and `lab`. Inland omits hulls; open_water omits the rover (P11).
3. Tags compile to a reference file: measures, writes, `legal_now`, safety limits (machine / numeric / catalog) with remaining-spec ids where they apply.
4. `read` does not step. `write` is `Lab::act_through_attach` only — no raw NED velocity that skips `legal_cmds`.
5. Tests: parked `drive`, docked `thrust`, inland hull, open_water rover — rejected (not-legal or P11), not a crash.
6. Numeric over-limit (e.g. Moving drive |v| above the driver clamp, `set_charge` above capacity) rejected when the write would otherwise be legal.
7. Chain file: multi-device writes then `step` ops; each tick is one `WorldSession::step` (P12). Illegal write stops the chain with a structured reject.
8. CLI + demo HTTP; stdio MCP `mhs_discover` / `mhs_reference` / `mhs_read` / `mhs_write` / `mhs_step` / `mhs_chain`. HTML/`include_str` unchanged (no remaining-spec §8 D2).
9. JSON Schema for discovery / reference / read / write / chain report, validated in crate tests.

---

## Phase F — Verified physical authority (the control boundary)

flight-core is the high-assurance Rust **control boundary**, not another flight
controller and not a Copper competitor. See [`docs/copper.md`](copper.md) and
[`docs/safety-contract.md`](safety-contract.md).

### F1. Revocable capability / evidence model

**Status: landed.** `ActuationPermit` is non-`Clone`, bound to `VehicleId` +
`SafetyEpoch` + optional lease. `VehicleBackend::authority_epoch` is the live
plant/PX4 counter. Setpoints **and** physical-authority mode changes
(`enter_offboard_now`, `start_takeoff_now`, land) check the permit **before**
the backend. World failsafe on a sibling handle increments `Body.authority_epoch`;
the old `Vehicle<Offboard>` is still typed Offboard and is `StaleAuthority`.
An async PX4 disarm HEARTBEAT bumps the epoch; leftover `Vehicle<Armed>` cannot
`enter_offboard_now`. Failsafe / disarm / recover stay ungated (safety actions).

**Acceptance:** NullBackend revoke test; world two-handle failsafe test;
trybuild `permit_is_not_clone`; Kani `permit_epoch_mismatch_is_stale`.

### F2. Tiny verified safety kernel TCB

**Status: landed.** `safety` / `ground` / `marine` remain `no_std`, no alloc,
no unsafe, no async, no IO. `event_revokes_authority` /
`ground_event_revokes_authority` / `marine_event_revokes_authority` are the
epoch bump predicates. Everything else is untrusted relative to `step`.

### F3. Temporal contracts

**Status: landed.** `Fresh` / `HeartbeatFresh` / `CommandFresh` / `Sequence` /
`Estimate` / `Observation` / `Rate` / `Deadline` / `Lease` / `Command` /
`Timestamp`. `Fresh::check_age` is the typed bound; it is the same predicate as
`heartbeat_age_ok` / `command_age_ok`. `require_live_permit` uses
`HeartbeatFresh::check_age` **and** `AerialOffboard::admit`.
`Vehicle::apply_velocity_command_now` rejects `StaleCommand` when command age
≥ 100 ms (`Command::deadline` / `Command::check_age`). An invalid `Estimate` yields
`Event::EstimatorInvalid` (`Estimate::revoke_event`); a stale heartbeat age
yields `Event::HeartbeatStale` (`heartbeat_revoke_event`). GPS-loss and
heartbeat-loss inject those events. PX4 setpoints fail `StaleHeartbeat` when the
last HEARTBEAT is older than 250 ms. Monitors: `CommandAgeMs`,
`EstimatorTimestampsMonotonic`, `EpochBumped` — heartbeat/command/estimator
checks use `HeartbeatFresh` / `CommandFresh` / `Timestamp` and fail closed
if those disagree with the kernel predicates.

### F4. Single-source contract DSL

**Status: landed (tables + generated now-methods + admission + Kani harness; not a second typestate crate).**
`define_aerial_authority!` in `safety.rs` is the table: heartbeat/command bounds,
`event_revokes_authority` (Creusot `ensures` on the same event list),
`admit_offboard_command`, `AERIAL_OFFBOARD_COMMANDS`, diagram/SPEC strings,
`AUTHORITY_REVOKE_EVENTS`, `AERIAL_OFFBOARD_TRANSITIONS`.
`vehicle_contract! { from_kernel }` aliases that table (`AerialOffboard::revokes`
**is** the kernel function; `admit` **is** `admit_offboard_command`;
`inject` is `Some(event)` iff `revokes`;
`COMMANDS` **is** `AERIAL_OFFBOARD_COMMANDS`; `TRANSITIONS` / `GATE` /
`UI_FORBIDDEN` are the capability surface). `impl_aerial_offboard_now!`
generates `admit_offboard_now` / `set_velocity_now` / `set_position_now` /
`hold_now`, the matching async wrappers (`set_velocity` / `set_position` /
`hold`), and `apply_velocity_command_now`. `AerialOffboard::evaluate` runs `MONITORS` (including
`OffboardAdmitted`, which is kernel `admit_offboard_command`).
`prove_aerial_authority!` expands to Kani `dsl_revokes_match_kernel`.
Checked-in generated artifacts under [`docs/generated/`](generated/) must
match the table (`SPEC`, mermaid, Graphviz, `CREUSOT`, `FAULTS`). The macro
does not emit a second Creusot proof file. `run_revoke_table` uses
`inject` so a leftover `Vehicle<Offboard>` refuses every `COMMANDS` method.

### F5. PX4 production-quality backend

**Status: landed.** Failsafe bumps epoch even if the UDP send fails. Telemetry
`estimator_valid` is `seen_local_position`, `imu_healthy` is `seen_px4`,
`failsafe` is latched, `heartbeat_age_secs` is elapsed since last PX4
heartbeat. Ingested HEARTBEAT with `MAV_STATE_CRITICAL` / `EMERGENCY` /
`FLIGHT_TERMINATION` or AUTO+RTL revokes authority **once**. AUTO+LAND
(NAV_LAND) does not latch failsafe. `authority_heartbeat_age_ms` feeds the
permit check. After failsafe is latched, `set_velocity_ned` /
`set_position_ned` return `BackendError::Rejected` at this backend (the
pre-offboard `pump_setpoint` stream is not gated). After a local-position
sample older than 250 ms, `Estimate::revoke_event` latches failsafe and
refuses new setpoints (never-seen pose is not a dropout).

### F6. Torture laboratory / differential conformance

**Status: landed.** `flight_sim::scenario` (`Scenario::GPS_LOSS`,
`HEARTBEAT_LOSS`, `HITL_MISS`, `scenario!`) injects estimator / heartbeat /
failsafe / wind / battery faults on the verified world, evaluates `Requirement`s,
writes JSONL, and differential-runs two world traces. Native ULog subset
(`write_ulog` / `parse_ulog`) round-trips `fc_trace` and can ingest
`vehicle_status`. `cargo run -p flight-sim --bin flight-test` runs
`--backend world|replay|ulog|px4-sitl|hitl` and `--scenario revoke-table`.
Live Gazebo is still out of scope; `--backend px4-sitl` evaluates a converted
JSONL or `.ulg` (checked-in `crates/flight-sim/corpus/px4_sitl_gps_loss.jsonl`).
`--backend hitl` is the attach-failsafe miss path (same contract as
`WorldRack::contract_deadline_miss`). `--backend all` runs
[`differential_contract`](../crates/flight-sim/src/scenario.rs) for every named
scenario (world + JSONL replay + ULog round-trip; gps-loss also the checked-in
ULog and converted PX4 SITL JSONL). GPS-loss posts `Estimate::revoke_event` and
a bound `Vehicle<Offboard>` cannot `set_position_now`. Every DSL revoke event has a world
test that the plant epoch increments and that a leftover Offboard handle
cannot run `set_velocity` / `set_position` / `hold`.

### F7. Typed geometry

**Status: landed.** `Transform<A,B> * Transform<B,C>` only; `Displacement`,
`Point3`, `Orientation<F>` (not `AngularVelocity`), `Force` / `Torque`,
`apply_point` / `apply_displacement`, `Rotation`, `Covariance<T>`.
trybuild `transform_wrong_frames`, `orientation_is_not_angular_velocity`,
`force_is_not_torque`, `velocity_is_not_acceleration`,
`point_is_not_displacement`, `unsafe_mission`. Copper `cu_transform` is
interop, not a copy (`docs/copper.md`).

### F8. Copper integration

**Status: landed (boundary).** [`docs/copper.md`](copper.md). No Copper
dependency. Do not add a scheduler/pubsub/physics engine.

### F9. HITL on the same contract

**Status: landed.** Deadline miss already trips attach failsafe; the plant
epoch now increments. `WorldRack::finish` fail-closes if `temporal::Deadline`
and kernel `deadline_outcome` disagree. `injected_miss_zeros_command_and_trips_failsafe`
asserts `authority_epoch > 0` and evaluates `Requirement::ActuatorsImplyArmed`
on the miss sample. `WorldRack::contract_deadline_miss` and
`flight-test --backend hitl` evaluate `Scenario::HITL_MISS.require`
(including `EpochBumped`). `cargo run -p flight-hitl --example contract_miss`.

### F10. Certification-oriented traceability

**Status: landed (tables).** [`docs/safety-contract.md`](safety-contract.md)
and [`docs/generated/traceability.md`](generated/traceability.md) ids
FC-CAP-AerialOffboard, FC-INV-001..003. `human_readable_spec()`.

---

## Suggested implementation order

1. **A1–A6 landed. B1–B5 landed. E1 landed. F1–F10 landed.** Next: **C1–C3** (proofs, traces, richer scenarios) and live Gazebo if someone needs a world renderer.
2. **C1–C3** (proofs, traces, scenarios) can overlap A3.
3. **B6 / B7 / B8** (more companions, metal, `no_std` tick) when the API is stable.
4. **D\*** morphologies last.

When official MHS is open-sourced, translate `flight-mhs` onto that schema. Do not collapse P1–P14 to make a driver “easier.”

Items in remaining-spec §2 are constraints on **every** step, not a phase.

---

## Done when (north star, not v0)

The north star is not a single checkbox. It is honest to say **world-class agentic tooling** only when Phase **A** is landed, Phase **B** hold/DP (B1–B2) are landed or explicitly deferred with a reason, and P1–P14 still hold. Morphologies (D) can remain open without diluting A+B.

Until then, README “Still ahead” points here.
