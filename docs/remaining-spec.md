# Remaining work spec (v0 invariants)

This file is the **v0 slice**: remaining work and **invariants** relative to

> The best Rust way to **use**, **test**, and **research** robotics across air, ground, and water: typestate vehicle APIs, mechanically verified simulation with clear state, agent observe/act/research workflows, and proven safety/behavior.

That slice is **landed**. The product north star is larger — world-class **agentic** tooling for robotics in Rust (experiment / control / understand, every domain and aspect):

- [`docs/agentic-spec.md`](agentic-spec.md) — north-star spec
- [`docs/NEXT.md`](NEXT.md) — ordered next steps with acceptance

Use **this** document as the invariant list (§2) and the v0 evidence log. Do not redefine v0 around a smaller subset that already passes. Do not “fix” §2. New feature work follows NEXT without collapsing P1–P14.

Land work as atomic commits on `main`. Do not accumulate large diffs. Do not open pull requests unless someone asks.

---

## 1. Status of the goal

The workspace already has a usable slice of that goal. In-scope functional items in this file are **landed**, including a recorded live PX4 SIH pass (§4.1). This file remains the invariant list (§2). Former §13 “out of scope” items that belong in the north star (ground pose hold, marine DP, estimation bit, metal FCH1, scenario scale) moved to [`docs/NEXT.md`](NEXT.md). §13 is now “not v0 / still non-goals.”

**Already true (do not re-implement):**

- Consume-self typestate for aerial / ground / marine vehicles (`Vehicle`, `GroundVehicle`, `MarineVehicle`) with compile-fail UI tests under `crates/flight-core/tests/ui/` (132 `.rs` files).
- Revocable `ActuationPermit` (non-`Clone`) plus backend `authority_epoch`. Failsafe / disarm / disconnect / stale heartbeat / estimator / IMU events increment the plant epoch. Stale permits cannot setpoint even when the Rust typestate is still `Offboard` / `Moving` / `Underway`. Mode-change now-APIs (`enter_offboard_now`, `start_takeoff_now`, `land`) check the live permit before the backend. A leftover `Vehicle<Armed>` after an async PX4 disarm HEARTBEAT is `StaleEpoch`. A leftover `Vehicle<Offboard>` after `connect` / `begin_session` is also `StaleEpoch` (reconnect invalidates outstanding permits). `require_live_permit` uses `HeartbeatFresh::check_age` and `AerialOffboard::admit`. `apply_velocity_command_now` rejects a command older than `COMMAND_MAX_AGE_MS` (`Command::deadline`). An invalid `Estimate` posts `Event::EstimatorInvalid` (`Estimate::revoke_event`); a stale estimate age (`EstimateFresh` / `estimate_revoke_event` / `ESTIMATE_MAX_AGE_MS`) is the same kernel event. GPS-loss and IMU-delay use that evidence and a leftover `Vehicle<Offboard>` cannot `set_position_now`. `Scenario::IMU_LOSS` injects `Event::ImuUnhealthy`. `Scenario::MOTOR_EFFICIENCY` scales plant `thrust_scale` and does not bump the epoch. PX4 `set_velocity_ned` / `set_position_ned` refuse after failsafe is latched, after a revoking disarm/disconnect (`actuation_revoked`), or after a stale local-position Estimate (the pre-offboard `pump_setpoint` stream is not gated). `hold_now` before arm, and after a first `connect` that never armed, is not treated as revoked.
- Single-source aerial authority table in `safety` (`define_aerial_authority!`): kernel revoke predicate, `admit_offboard_command`, `AERIAL_OFFBOARD_COMMANDS`, heartbeat/command bounds, Creusot `ensures`, capability diagrams, `ContractEdge` transition table. `vehicle_contract! { from_kernel }` aliases it (`AerialOffboard::admit`, `COMMANDS`, `inject`). `with_aerial_offboard_commands!` passes the OffboardControl command idents (lockstep `OFFBOARD_NOW_COMMANDS` vs kernel `AERIAL_OFFBOARD_COMMANDS`) to `impl_aerial_offboard_now!`, which generates the OffboardControl now-methods, async wrappers, `for_each_offboard_now`, `leftover_commands_stale`, and `apply_velocity_command_now`. `AerialOffboard::evaluate` runs the generated `MONITORS` (including `OffboardAdmitted`). `prove_aerial_authority!` is the Kani harness. `AerialOffboard::CREUSOT` / `FAULTS` lockstep generated listings. Native ULog subset plus JSONL share `evaluate_trace`. `run_revoke_table` injects each `REVOKE_ON` event through `WorldSession::inject_revoke` (`AerialOffboard::inject` first; non-revoke is Rejected) and a leftover `Vehicle<Offboard>` cannot run `COMMANDS`. `differential_revoke_table` round-trips those leftover samples on JSONL and ULog. `Px4Backend::inject_revoke` / `run_px4_revoke_table` / `flight-test-px4` apply the same leftover `COMMANDS` check at the companion boundary (no `flight-sim` → `flight-px4` dependency). `ArduPilotBackend::inject_revoke` / `run_ardupilot_revoke_table` / `flight-test-ardupilot` are leftover Offboard `COMMANDS` at the Copter GUIDED companion (no `flight-sim` → `flight-ardupilot` dependency; live Copter is loopback-only, no CI sitl job). Named `Fault` kernel events are `AerialOffboard::inject`. `Scenario::HITL_MISS` / `WorldRack::contract_deadline_miss` / `differential_contract` evaluate the same `Requirement`s plus capability monitors. `WorldRack::leftover_after_deadline_miss` / `flight-test-hitl` bind leftover OffboardControl (inland Takeoff) before a rack miss; after the miss every `COMMANDS` method is `StaleAuthority`. `WorldRack::run_hitl_revoke_table` leftover `COMMANDS` after every `REVOKE_ON` through the same `inject_revoke` (Sequence-monotonic epoch). `plant::leftover_after_failsafe` / `plant::leftover_after_disarm` / `run_ros2_revoke_table` / `flight-test-ros2` are leftover OffboardControl at the ROS 2 plant after `apply_failsafe`, `apply_disarm`, and every `REVOKE_ON` (no `flight-sim` → `flight-ros2` / `flight-hitl` dependency). `run_revoke_table` / `run_px4_revoke_table` also observe leftover epoch with `Sequence`. `WorldRack::finish` fail-closes if `temporal::Deadline`, kernel `deadline_outcome`, and `Rate` (period lockstep `DeadlineSpec`) disagree.
- `OffboardControl` gates `set_velocity` / `set_position` / `hold`. `MotorsEnabled` gates `set_motor_thrust`. Recovery is a real aerial typestate.
- One mechanically verified plant: `robot-world::World::try_step` clones, advances, and commits only if all **22** named properties hold. NED z-down. Catalogs `coastal` / `harbor` / `inland` / `open_water`.
- `WorldSession` attach walks (`attach_takeoff`, `attach_drive`, `attach_undock`, `attach_hold`, `attach_ground_hold`, `attach_marine_hold`, failsafe / recover / return / station / airborne, …) shared by HITL, ROS 2, PX4 `WorldPlant`, and `robot-lab`. `WorldSession::inject_revoke` is the shared DSL revoke inject for world `run_revoke_table`, HITL `run_hitl_revoke_table`, and ROS 2 `run_ros2_revoke_table`. `Lab::from_scene` / `WorldSession::from_scene` build a catalog or a custom body table (`robot_world::Scene`); reserved names stay P11; custom names are not registered on `World::named`.
- Aerial position hold: plant `hold_ned`, kernel `hold_velocity_ned` / `HOLD_KP`, Kani `hold_velocity_restores_pose`, `LabCmd::Hold` + `LabCmd::Position`, `TypedHold` / `TypedPositionHold` / `TypedFleetHold`, demo `POST /api/hold`.
- Ground pose hold: same plant `hold_ned` / `position_hold_restores_pose` / Kani restore fact; `GroundVehicle<Moving>::hold_now`; `WorldSession::attach_ground_hold`; `TypedGroundHold`. Parked / EStop compile-fail. `LabCmd::Position` stays aerial-only.
- Marine NED DP: same plant field; `MarineVehicle` `hold_now` on `CanThrust`; `attach_marine_hold`; `TypedMarineHold`. Docked / Failsafe compile-fail. Distinct from `StationKeep`.
- Aerial nav trip: `WorldSession::update_nav` / `Lab::update_nav` feed `ComplementaryAttitude`. Unusable IMU posts `Event::EstimatorInvalid` (clears `estimator_valid`, latches failsafe if armed) and never writes the plant quaternion. Filter warm-up does not trip. `unit_attitude` stays `mech::quat_integrate`.
- Typed NED paths: `Waypoint` / `NedPath` execute through OffboardControl / Moving / CanThrust attach. `TypedPathFollow` is the two-point aerial agent.
- Lab certificate `fleet_hold_simultaneous` (NEXT B5): drone hold plus StationKeep when a skiff exists. Not in the 22-property plant vector. P11 skips still omit missing bodies.
- MHS-shaped adapter (`flight-mhs`, NEXT E1): discovery / compiled reference / read / write / chain / stdio MCP. **Not** official MHS. Writes remain `Lab::act_through_attach`.
- Research loop: `Lab::observe` / `act_through_attach` / `research` / `replay_until` / `research_probe`, typed agents with `actions_applied == 0` for legal motion, JSONL + Foxglove-shaped MCAP. `WorldImu` + `FuzzedImu` read noisy samples without replacing `WorldSession::step`.
- Live PX4 SIH companion path: `sitl_live --ignored` recorded pass (14.59s, `px4io/px4-sitl:v1.18.0-beta2`); CI job `sitl` (takeoff/hold/land **and** leftover Offboard after live `EstimatorInvalid`). `Px4Backend::inject_revoke` covers every `REVOKE_ON` event; `cargo run -p flight-px4 --bin flight-test-px4` is the leftover Offboard table at the companion boundary plus `run_px4_gps_loss` (`AerialOffboard::GPS_LOSS_REQUIRE`). ArduPilot GUIDED companion: `cargo run -p flight-ardupilot --bin flight-test-ardupilot` leftover Offboard after every `REVOKE_ON` plus `run_ardupilot_gps_loss`; live Copter `sitl_live` is `#[ignore]` loopback-only (no CI sitl job; reuse `flight-mavlink`, no second stack). `cargo run -p flight-hitl --bin flight-test-hitl` is leftover OffboardControl `COMMANDS` after a rack deadline/`Rate` miss and after every `REVOKE_ON`, leftover GPS-loss (`run_hitl_gps_loss`), plus `run_fch1_udp_mock` (faithful UDP card; recorded `crates/flight-hitl/corpus/fch1_udp_mock.jsonl`). `cargo run -p flight-ros2 --bin flight-test-ros2` is leftover OffboardControl after `apply_failsafe`, `apply_disarm`, and every `REVOKE_ON`, plus leftover GPS-loss (`run_ros2_gps_loss`; no rclrs).

**Not true yet (v0):** Nothing in-scope that this spec still treats as a feature gap. After demo HTML/`include_str` changes, re-run §8 D2. Do not “fix” §2. Agentic next work is [`docs/NEXT.md`](NEXT.md), not a silent reopen of this file’s landed sections.

---

## 2. Invariants remaining work must preserve

Closing a gap by collapsing these splits is a regression, not progress.

| ID | Split | Keep |
| --- | --- | --- |
| P1 | Kernel vs typestate `EnterOffboard` | Kernel allows Armed / Takeoff / Airborne / Landing. `enter_offboard_now` compiles **only from Armed**. |
| P2 | Kernel vs typestate `Takeoff` | Kernel `Takeoff` is legal from Armed. `start_takeoff_now` compiles **only from Offboard**. |
| P3 | Kernel vs typestate marine dock / failsafe | Kernel `Dock` from Failsafe stays legal; `CanDock` does **not** include Failsafe. Kernel Failsafe from Docked stays legal; `CanTripMarineFailsafe` does **not** include Docked. Do not add `declare_failsafe` on `MarineVehicle<Docked>` without dropping `docked_failsafe` and the README. |
| P4 | Kernel `Touchdown` | Legal from `Landing \| Failsafe` **without** requiring terrain contact. `CanTouchdown` is Landing and Failsafe. Recovery cannot touchdown. |
| P5 | Disarm | `CanDisarm` is Ready through Landing → Ready. Failsafe `disarm_now` → Recovery. Recovery has no `disarm_now`. |
| P6 | Lab JSON Disarm vs PX4 operator DISARM | JSON `LabCmd::Disarm` from Failsafe: `attach_disarm` is Protocol; JSON fallback is kernel Disarm → Recovery. PX4 `COMPONENT_ARM_DISARM` from Failsafe walks `attach_recover_ready` to **Ready**. Keep that split. |
| P7 | `TypedAerialFailsafe` | Grants takeoff, then failsafe, then recover. Logs include Takeoff. Do not change to a pad-only failsafe (that is `TypedPadFailsafe`). |
| P8 | Grant vs attach | `grant_*` are attach walks on an existing handle, not a second event path. `grant_twice_is_protocol` and `grant_shortcuts_match_attach_helpers` stay. |
| P9 | `failsafe_now` / `takeoff_now` / `land_now` | Do not wrap these as `attach_*` in a way that recurses. World handle trait impls call inherent `WorldBackend::land_now` / `recover_now`. |
| P10 | Demo hold element id | Properties banner stays `id="hold"`. DRONE HOLD button stays `id="drone-hold"`. |
| P11 | Catalog bodies | Coastal / harbor: drone + rover + skiff + surveyor. Inland: no hull. Open water: no rover. Omit bodies; do not invent placeholders. |
| P12 | Multi-vehicle step | Flush all granted setpoints, then **one** `WorldSession::step`. |
| P13 | Ungranted aerial | Plant `else { body.clear_command() }` when not granted (wet rotors, empty battery, failsafe). That **wipes** `hold_ned` (§5.4 A). Do not persist hold while ungranted. |
| P14 | MSRV | Workspace `rust-version = "1.85"`. Do not bump MSRV to chase Creusot 0.8 or the Kani installer without an explicit decision. |

---

## 3. Proof and mechanical verification

### 3.1 Discharge Creusot contracts

**Status: landed** for the discrete kernel machines. `cargo creusot prove -- -p flight-core --features creusot` with Creusot **0.5.0** (`nightly-2025-01-31`, isolated; workspace MSRV stays **1.85**) proves **81** libraries, **0** failures. Recorded local pass includes:

- `ground_step` (6 VCs) and `inv_ground`
- `marine_step` (6 VCs) and `inv_marine`
- aerial `step` (191 VCs), `enter_failsafe`, `check_invariants`, `step_preserves`
- HITL `deadline_outcome` and `hitl_apply_allowed`

`why3find.json` pins the `creusot` Why3 package. Creusot 0.5 ICEs on `dyn fmt::Write` and does not translate `async`, so `cargo creusot` compiles only `safety` / `ground` / `marine` / `hitl` (no Debug/Display, no serde derives, no typestate module). rustc still builds the full crate.

CI job `creusot` installs Creusot 0.5.0 from tag `v0.5.0` and runs the same prove command.

#### 3.1.4 f32 facts stay Kani

Creusot 0.5 pearlite has **no `OrdLogic` for `f32`** and ICEs on float literals (`Unsupported literal`). Dummy `#[pure]` is a program function and cannot be called from `requires` / `ensures`. Those kernel facts therefore have **no** Creusot postcondition; Kani already proves them:

- `hold_velocity_ned` / `hold_restores_pose`
- dry buoyancy (`buoyancy_ned` / `buoyancy_only_when_wet`)
- hydro `two_cell_periodic_mass` non-negative
- HITL `command_after_deadline` zeros a miss (and non-finite setpoints); Creusot also lacks `DeepModel` for `f32`

Do not re-attach `f32` `ensures` on 0.5. A later Creusot that can state floats is a toolchain bump under §P14, not a silent MSRV change.

### 3.2 Put Kani in a gate

**Status: landed.** CI job `kani` runs `cargo kani -p flight-verify -j 2 --output-format terse` with `kani-verifier` **0.67.0** (`model-checking/kani-github-action@v1.1`). A recorded local pass on rustc 1.85.0: **45** harnesses, 0 failures. Workspace `rust-version` stays **1.85**; the installer rustc (≥ 1.88) is not MSRV. README harness count and `flight-verify` module theorems stay in lockstep with `#[kani::proof]`. [`docs/generated/proof-summary.txt`](generated/proof-summary.txt) is the agent digest (`Experiment` copies it into `run.json`). New kernel transitions still need a harness when one is feasible — that is an ongoing constraint, not a leftover job.

### 3.3 Named world property for pose hold

**Status: landed.** `position_hold_restores_pose` is the 22nd world property. Vacuous when `hold_ned` is `None`. When set, `command` must be `Some` and `hold_restores_pose(hold, pose, command)` must hold. `World::try_step` refuses a non-finite hold. A granted inland takeoff + `set_position_hold` still yields `all_hold`.

Idle certificates still include the property (true). Do not drop it from the vector.

### 3.4 Attitude estimator in the trusted loop

**Status: landed as a navigation trip, physics-truth plant.** The plant quaternion is **physics truth** (`mech::quat_integrate`, property `unit_attitude`). `ComplementaryAttitude` is not in `World::try_step`. `WorldSession::update_nav` / `Lab::update_nav` may post `Event::EstimatorInvalid` on an unusable IMU sample, which clears kernel `estimator_valid` and latches failsafe if armed. Filter warm-up does not trip. ESKF / GNSS fusion remains out of scope (NEXT B3b).

---

## 4. Companion vehicles: PX4, ROS 2, HITL

### 4.1 Live PX4 SITL as a proven path

**Status: landed.** Recorded local pass, 2026-08-29:

```text
PX4_SIM_MODEL=sihsim_quadx  /opt/px4/bin/px4 /opt/px4/etc -d
  # binary from px4io/px4-sitl:v1.18.0-beta2 (SYS_AUTOSTART=10040, SIH 250 Hz)
cargo test -p flight-px4 --test sitl_live -- --ignored --nocapture
  # 1 passed; 0 failed; finished in 14.59s
```

PX4 log on that run: `Armed by external command`, `Takeoff detected`, `Landing detected`, `Disarmed by landing`. Companion `connect` waits for a PX4 heartbeat; `preflight` waits for `LOCAL_POSITION_NED`; `tick` / `hold_now` drain `try_recv`. `MAV_CMD_DO_SET_MODE` param2 is unpacked `PX4_MAIN_MODE_OFFBOARD` (6). Climb/hold stay in offboard (velocity then position setpoints). `tests/sitl_live.rs` is `#[ignore]` in default workspace tests and **required** in CI job `sitl` (`px4io/px4-sitl:v1.18.0-beta2`, `PX4_SIM_MODEL=sihsim_quadx`, `px4 -d`, host network). Hub has **no** `v1.17.0` tag. Loopback `companion_hold_streams_ingested_local_position` covers ingest without a binary. Disconnected send stays `BackendError::Disconnected`.

**Acceptance (met):**

1. Documented SIH docker / `.deb` / `make px4_sitl gz_x500` plus `cargo run -p flight-px4 --example sitl_hover`.
2. Skipped without SITL (`#[ignore]`); required in job `sitl`.
3. Live `hold` / `set_position` / `set_velocity` through `LOCAL_POSITION_NED`.
4. Disconnected send is `Disconnected`.

### 4.2 Companion `hold_now` on `Px4Backend`

**Status: landed** for the companion send path. `VehicleBackend::hold_now` is the default (telemetry pose → `set_position_ned_now`). `Px4Backend::hold_now` drains waiting `LOCAL_POSITION_NED`, then streams that pose. Disconnected hold is `Disconnected`. `examples/sitl_hover` holds after takeoff. Live SITL is §4.1.

### 4.3 ROS 2 `rclrs` in a gate

**Status: landed.** CI job `rclrs` on `ubuntu-24.04` installs Jazzy (`ros-tooling/setup-ros@v0.7`), then runs `cargo test -p flight-ros2 --features rclrs` with rustc 1.85. Node tests cover inland (no hull), open water (no rover), harbor (four bodies), and `PlantNode` / `FleetPlantNode` `hold` before grant (`BackendError::Protocol`) then after grant (`hold_ned` set). Those methods call `apply_hold` → `attach_hold`. Default workspace tests still skip the `rclrs` feature.

### 4.4 Physical FCH1 I/O

**Current evidence:** `flight-hitl` encodes `FCH1` samples/commands. `WorldRack::bind_io` / `Fch1UdpCard` speak those datagrams on loopback. Miss ⇒ attach failsafe + zero command. No hardware-in-the-loop job against a real card (UDP mock is the recorded pass).

**Status: landed (full bar: UDP mock).** Protocol tests round-trip `apply == 0`. `RackCommand::from_fch1` zeros a slot when `apply == 0`, so a decoded miss cannot revive a hold. Slot map: 0 drone, 1 rover, 2 skiff, 3 surveyor. [`Fch1UdpCard`](../crates/flight-hitl/src/card.rs) is a faithful UDP peer that does **not** step `World`. [`WorldRack::bind_io`] / `drain_io` / `frame_from_io` ingest wire commands. Recorded inland pass: `crates/flight-hitl/corpus/fch1_udp_mock.jsonl` (hull slots ignored, `apply == 0` keeps hold, live climb clears it, samples on slots 0 and 1). `cargo run -p flight-hitl --example udp_card`. A physical card remains optional.

**Acceptance (full “use on a rack”):** one recorded run against a card or a faithful UDP mock that is not the in-process plant.

### 4.5 HITL / ROS 2 / PX4 / lab API parity

Hold, airborne, station, resume, dock, park, return, recover are walked on `WorldSession` in HITL, ROS 2, PX4 `WorldPlant`, and `Lab`. Remaining parity items:

| Gap | Acceptance |
| --- | --- |
| Demo failsafe queued as `LabCmd::Failsafe` | **Landed.** Same `queue_robot` path as hold. |
| `sitl_hover` settle then hold | Example now holds after takeoff (`Vehicle::hold`). Live SITL is §4.1. |
| `examples/typed.rs` hold after takeoff | **Landed.** After the shared step loop, `hold_now` + flush + step; observation JSON includes `hold_ned`. |
| JSON research agents (`PadLanding`, `CoastalFleet`, `CollisionSweep`, `RoverProbe`) | Keep as JSON-probe twins. Do not delete. Typed twins must remain the `actions_applied == 0` path. Any new operator act gets a typed agent **and** a JSON probe. |

---

## 5. Research workflows

### 5.1 Fleet-scale hold / station certificate

**Status: landed.** `TypedFleetHold` probes illegal grants, then `grant_attached`, drone `attach_hold`, and skiff `attach_station`. Inland skips hull station. Open water skips rover halt. End observation: `hold_ned` is `Some`; skiff kind is `StationKeep` when a hull exists. `actions_applied == 0`, log replays, `all_hold`. CLI `typed-fleet-hold`.

### 5.2 Illegal catalog completeness

**Status: landed.** `research_probe` illegal catalog includes parked Drive/Thrust/Velocity, pad Hold/Position/Airborne/Takeoff, parked rover Hold/Halt, docked skiff/surveyor Hold, docked Station, plus Failsafe Hold on a clone (so the main lab stays Ready). Illegal phase stays on `Lab::act`. Legal abuse walks rover Hold after Drive, hull Hold after Undock, then drone Hold then Velocity through `act_through_attach`.

### 5.3 MCAP / JSONL research traces

**Status: landed.** Observation jsonschema names `hold_ned`, `legal_cmds`, `kind`, `sphere_hits`, and the property vector. `observation_json` / `action_json` round-trip a bag and keep hold + legal_cmds. `examples/bag.rs` documents opening the file in Foxglove.

### 5.4 Hold across ungranted steps

**Status: landed as A (wipe).** Ungranted aerial `clear_command()` clears `hold_ned`. Failsafe, empty battery, and wet rotors do not keep a pose target. `Body::clear_command` documents that choice. Do not persist hold while ungranted without a new property.

### 5.5 Fuzzed / recorded IMU vs the verified world

**Status: landed.** `WorldImu` samples a `WorldSession` body without stepping. `FuzzedImu<WorldImu>` drives hold while `WorldSession::step` stays the plant (`fuzzed_world_imu_hold_keeps_properties`). An unusable IMU sample trips failsafe and the property vector still holds. Example `fuzzed_world`.

---

## 6. Typestate and kernel APIs

### 6.1 `no_std` vehicle API

**Status: landed as std-only vehicles.** Crate docs state `Vehicle` / `GroundVehicle` / `MarineVehicle` require `std`. `--no-default-features` is units, frames, sensors, safety, hydro, mech, and the attitude estimator. CI checks that build. There is no `no_std` typestate handle. Host tick of the discrete machines is `flight_core::host::kernel_host_tick` / `cargo run -p flight-core --example kernel_tick` (NEXT B8).

### 6.2 Ground / marine pose hold

**Status: landed (ground and marine DP).** `GroundVehicle<Moving>::hold_now` fires kernel `DriveCommand` then `VehicleBackend::hold_now`. Parked / EStop compile-fail (`parked_hold.rs`, `estopped_hold.rs`). `MarineVehicle` `hold_now` on `CanThrust` (Underway or StationKeep); Docked / Failsafe compile-fail (`docked_hold.rs`, `marine_failsafe_hold.rs`). Distinct from `hold_station` / StationKeep. Plant field is `hold_ned` / `position_hold_restores_pose`. Restore fact is existing Kani `hold_velocity_restores_pose` (f32; no second harness). `LabCmd::Hold` is legal on Moving / Underway / StationKeep; `LabCmd::Position` stays aerial-only. `WorldSession::attach_ground_hold` / `attach_marine_hold` / `TypedGroundHold` / `TypedMarineHold`. Inland skips hulls; open_water skips the rover (P11). Halt, E-stop, dock, failsafe, empty battery, and ungranted `clear_command()` wipe `hold_ned` (P13 spirit, §5.4 A). P3 stands.

**Acceptance:** north-star B1–B2 rows above (now landed). Do not add `declare_failsafe` on `Docked`. Do not dock from Failsafe in typestate.

### 6.3 Kernel `EnterOffboard` vs Armed-only now-API

**Status: landed (docs).** Documented in P1. `flight-verify` theorems already state both layers. Any new backend must not call kernel `EnterOffboard` from Ready to “help” the operator.

---

## 7. Simulation plant

### 7.1 Two physics stacks

**Status: landed.** `robot-world` / `WorldSession` is the mechanically verified plant. `SimBackend` is labeled a point-mass demo (not the property vector). Hold, failsafe, and catalogs land on `WorldSession` first.

### 7.2 GPU hydro bit-identity

**Status: landed as performance path.** GPU is not required to match CPU heightfields bitwise. `gpu_or_cpu_coastal_holds` checks hydro invariants. CI job `gpu` sets `FLIGHT_HYDRO_GPU=1` and installs lavapipe; without an adapter the test still runs the CPU kernel.

### 7.3 Hydro resolution

**Current evidence:** 40×32 cells, 2 m spacing. Fine for the coastal demo; not a coastal-engineering model.

**Acceptance:** out of scope for the product goal. Do not grow the grid without a property-preserving test.

---

## 8. Demo console

**Current evidence:** Live lab on `FLIGHT_DEMO_BIND` (default `0.0.0.0:47831`). Safety / return / maneuver buttons. Hold queues `LabCmd::Hold`. Failsafe queues `LabCmd::Failsafe`. Legal cmds and `hold_ned` render on the drone card. Scripted coastal recycles after t>40 unless the operator has acted. Inland hides hull buttons; open water hides rover.

**Remaining:**

| ID | Gap | Acceptance |
| --- | --- | --- |
| D1 | Failsafe latch vs queue | **Landed.** POST `/api/failsafe` queues `LabCmd::Failsafe` on the same `act_through_attach` path as hold. |
| D2 | Browser verification | After HTML/`include_str` changes, rebuild the binary. Exercise HOLD end-to-end: wait until `hold` is in `legal_cmds`, POST `/api/hold`, observation `message` is `drone hold`, `hold_ned` is `Some` and persists across idle ticks, `all_hold` true. A screenshot is not enough. |
| D3 | Idle hold vs scripted velocity | **Landed.** Idle steps keep `hold_ned`. `apply_script` on Takeoff writes velocity and wipes hold; the demo sets `scripted=false` before queuing operator hold. |
| D4 | Cache | Index is `Cache-Control: no-store`. Keep that. |

---

## 9. Tooling and CI

**Current gate (`.github/workflows/ci.yml`):** Rust 1.85, `cargo fmt --all -- --check`, `clippy --workspace --all-targets -- -D warnings`, `cargo test --workspace`, `cargo check -p flight-core --no-default-features`, job `gpu` (`FLIGHT_HYDRO_GPU=1 cargo test -p robot-world --lib gpu` with lavapipe), job `kani` (`cargo kani -p flight-verify`, kani-verifier 0.67.0, 45 harnesses), job `rclrs` (`cargo test -p flight-ros2 --features rclrs` on Jazzy / ubuntu-24.04), job `creusot` (`cargo creusot prove -- -p flight-core --features creusot`, Creusot 0.5.0, 81 libraries), and job `sitl` (`px4io/px4-sitl:v1.18.0-beta2` SIH + `cargo test -p flight-px4 --test sitl_live -- --ignored`).

**Missing from that gate:** none of the in-scope jobs. After HTML changes, §8 D2 is a local browser check, not a CI job.

**Also:**

- trybuild UI tests run inside `cargo test -p flight-core`. New compile-fails need a `.stderr`. Do not invent stderr; copy ACTUAL from trybuild (`TRYBUILD=overwrite` locally, then review).
- Clippy `-D warnings`: no `default_constructed_unit_structs`; avoid `single_match` that clippy denies.
- `cargo test` accepts **one** filter name.
- Publish: this is a Rust library + long-running sim, not a Vercel app.

---

## 10. Documentation accuracy

**Current evidence:** README “Still ahead” mixed a feature inventory with remaining work. Kani harness count in the run block can drift.

**Acceptance:**

1. This file is the v0 invariant spec. README points here **and** at [`docs/agentic-spec.md`](agentic-spec.md) / [`docs/NEXT.md`](NEXT.md).
2. Theorem lists in `flight-verify` and compile-fail names stay in lockstep when APIs change.
3. Every typed agent in `examples/agent.rs` has a README `cargo run` line.
4. Do not mention private remotes or temporary clone names in docs.

---

## 11. Goal completion audit (v0)

This table is the **v0** goal. It is proven against the tree. Do **not** reopen it as a feature backlog. The agentic north star is [`docs/agentic-spec.md`](agentic-spec.md); do not mark *that* complete until [`docs/NEXT.md`](NEXT.md) Phase A (and B1–B2 or an explicit deferral) is landed without regressing §2.

| Requirement from the goal | Evidence that would prove it | Remaining |
| --- | --- | --- |
| Best Rust **use** path (typestate, one API, sim + companion) | Same `Vehicle<S, B>` against `WorldSession`, `SimBackend`, and live PX4; hold/failsafe/land documented and tested on each | — |
| **Test** in a verified world with clear state | 22 properties refuse bad `step`; catalogs; HITL miss zeros command; attach kinds match plant phase/kind | — |
| **Research** observe / act / certificate | Typed agents + JSON probe + replay + bags + fuzzed IMU vs plant | — |
| **Proven** safety/behavior | trybuild + exhaustive packed machines + Kani + Creusot | — |
| Air, ground, and water | Four catalogs; skiff + surveyor; station/dock/estop typed agents | No extra domain. Do not drop AUV. |
| Atomic commits on main, no PR pile-up | Process, not a code artifact | Ongoing |

---

## 12. Suggested implementation order

Order is technical dependence, not calendar.

1. §10 README pointer + harness count (docs only) — landed.
2. §3.3 hold property on the world vector — landed.
3. §4.2 companion hold send + disconnected test — landed.
4. §8 D1 demo failsafe queue — landed.
5. §5.1 `TypedFleetHold` — landed.
6. §5.2 probe catalog — landed.
7. §5.3 MCAP schemas — landed.
8. §9 CI: GPU hydro job — landed. Kani job — landed. rclrs job — landed. Creusot job — landed. SITL job — landed.
9. §3.1 Creusot toolchain + discharge — landed.
10. §4.1 live SITL — landed (recorded `sitl_live --ignored` pass, 14.59s, SIH v1.18.0-beta2).
11. §3.4 estimator bit or a written “physics-truth only” decision — landed (physics-truth plant; session nav may trip the kernel bit).
12. §5.5 fuzzed IMU against `WorldSession` — landed.

Items in §2 (P1–P14) are constraints on every step, not a phase.

---

## 13. Not v0 / still non-goals

**Moved to the north star** ([`docs/NEXT.md`](NEXT.md)), not “never”:

- Ground GPS/pose hold (NEXT B1) — landed.
- Marine DP / NED pose hold beyond `StationKeep` (NEXT B2) — landed.
- Wiring an estimator so a bad IMU clears `estimator_valid` (NEXT B3) — landed. Full PX4 EKF/RTK/mission planner stays a non-goal.
- Physical FCH1 recorded run (NEXT B7) — landed (faithful UDP mock; physical card optional).
- Scenario DSL (NEXT C3) and hydro/body scale with property-preserving tests (NEXT C4) — landed. Not a coastline-scale ocean product by default.

**Still non-goals** unless a later instruction adds them:

- Native mobile / desktop GUIs, web dashboards beyond `flight-demo`.
- Authentication, multi-user lab, cloud fleet.
- Publishing crates to crates.io (version is `0.1.0`; that is not a functional gap).
- Vercel / serverless deploy.
