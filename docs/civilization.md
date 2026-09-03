# Civilization plan (agent-executable)

This is the ordered work for subsequent agents. Execute it. Do not treat
[`docs/NEXT.md`](NEXT.md) “F1–F10 landed / next is live Gazebo” as the product
end state. That stamp is a v0/backlog label. The product end state is this
file plus the manifesto below.

Land work as **atomic commits on `main`**. Do not open pull requests unless
someone asks. Do not mention private remotes or temporary clone names. Do not
create `AGENTS.md`.

---

## 0. Why this repo exists for civilization

Autonomous software already commands physical vehicles. The civilization-scale
failure is **unauthorized actuation**: a leftover handle, a stale planner
setpoint, a ROS node, or an async PX4 disarm HEARTBEAT that still moves motors
after authority should have ended.

flight-core is the high-assurance Rust **control boundary** between that
software and the vehicle. PX4 / ArduPilot fly. Copper can schedule. ROS 2 can
publish. This repo makes physical authority **typed, revocable, replayable, and
mechanically checked**. Integration beats replacement.

Civilization usefulness is not a Gazebo renderer, a morphology zoo, or another
flight controller. It is:

1. A leftover `Vehicle` after failsafe / disarm / reconnect / GPS-loss / HITL
   miss **has no actuation authority**, even when the Rust type still says Armed
   / Offboard / Takeoff.
2. The same safety contract evaluates on in-process world, PX4 SITL, ULog
   replay, HITL, and ROS 2 plant.
3. An unsafe mission fails `cargo check`. After the fix, `cargo kani` proves no
   unarmed actuation.
4. A third party can put this boundary between a planner and a companion
   without learning kernel source.

Until those four are true **and demonstrated**, the repo is research infra.
Until they are **used on a vehicle path that can kill**, it is not yet doing
civilization work. That is the bar.

---

## 1. Manifesto end state (do not shrink)

All of the following must be true and **verified in the tree**. Do not redefine
success around a smaller subset that already passes.

1. **Revocable capability / evidence.** Types represent evidence and revocable
   authority. Non-`Clone` `ActuationPermit` bound to vehicle id, control epoch,
   time bound. Invalidated by failsafe / disarm / reconnect. Checked at the
   hardware/backend boundary. Stale `Vehicle<Armed>` after async PX4 disarm has
   **no** actuation authority. Stale `Vehicle<Takeoff>` cannot
   `declare_airborne_now`.
2. **Tiny verified safety kernel TCB.** `no_std`, no allocation, no unsafe, no
   async, no IO, no threads, deterministic, bounded. All physical-authority
   commands pass through it on the verified world. Untrusted: PX4, ROS,
   MAVLink, planners, mission code.
3. **Temporal contracts first-class.** Fresh / Timestamp / Estimate /
   Observation / Deadline / Lease / Rate / Sequence. Offboard ⇒ heartbeat age
   bound. Permit epoch == safety epoch. Command age < deadline. Monotonic
   estimator timestamps. Compile-time where possible; runtime monitors where
   physical reality cannot be proved.
4. **Single-source contract DSL** generating typestate/capability API, runtime
   kernel, Kani harnesses, Creusot contracts, runtime monitors, transition
   table, diagrams, tests, fault-injection, human-readable spec, traceability
   IDs. Do **not** generate a second typestate crate (collapses P1).
5. **Frames / geometry** that make misuse unrepresentable: Point vs
   displacement vs velocity vs acceleration vs force vs torque vs orientation
   vs angular velocity. `Transform<A,B>` composition only when frames chain. Do
   not duplicate Copper `cu_transform`.
6. **Adversarial torture laboratory.** Scenario DSL with inject/require;
   runnable against in-process model, record/replay, PX4 SITL, HITL, real logs;
   differential conformance of the **same** safety contract across backends. Not
   a Gazebo competitor.
7. **Positioning.** Do not compete with Copper on deterministic runtime or with
   PX4 as a flight controller. Aerial-focused. Reusable contracts vs domain
   cores may split in-monorepo without collapsing P1–P14.
8. **README eventually demonstrates:** unsafe mission fails `cargo check`; after
   fix, `cargo kani` proves no unarmed actuation; gps-loss revokes
   position-control on a bound; same contract on sim, PX4 SITL, and ulog replay.
9. **Keep v0 green:** remaining-spec P1–P14, trybuild 135, Kani 45 harnesses,
   Creusot 81 libraries, MSRV 1.85, `no_std` kernel path, catalog P11, one
   `WorldSession::step` (P12). If trybuild/Kani/Creusot counts change, update
   remaining-spec, README, `docs/generated/proof-summary.txt`, and
   lockstep tests in the same slice.

**Priority order** (do not reorder because a later item is easier):

1. Revocable permits / epochs
2. Tiny kernel TCB
3. Temporal contracts
4. Contract DSL
5. Production PX4 backend
6. Differential fault lab
7. Rich geometry
8. Copper integration
9. HITL / reference hardware
10. Certification traceability

---

## 2. Law on every slice

[`docs/remaining-spec.md`](remaining-spec.md) **P1–P14** still apply. Closing a
gap by collapsing kernel vs typestate is a regression.

Always:

- `cargo fmt --all`
- `cargo check -p flight-core --no-default-features`
- `cargo test --workspace --exclude flight-ros2` **and** `cargo test -p flight-ros2`
  (workspace clippy/tests exclude `flight-ros2` locally when Jazzy is missing)
- `cargo clippy --workspace --all-targets --exclude flight-ros2 -- -D warnings`
- `cargo clippy -p flight-ros2 --all-targets -- -D warnings` when that crate
  changes
- leftover bins `flight-test-px4` / `flight-test-ardupilot` / `flight-test-hitl`
  / `flight-test-ros2` when those crates change
- `cargo test` takes **one** filter name only
- Nested `$$` in `macro_rules` is unstable on MSRV 1.85 — two ident lists must
  lockstep (`commands:` vs `with_aerial_offboard_commands!`; `revokes_on` vs
  `leftover:`)
- `prove_aerial_authority!` expands inside `#[cfg(kani)] mod proofs`
- Do not add a second Kani harness or Creusot `ensures` without updating 45 / 81
- Do not put `authority_epoch` in packed `SafetyState`
- `ActuationPermit` stays non-`Clone`
- `enter_offboard_now` uses `require_permit` (epoch only), **not**
  `require_live_permit` (heartbeat)
- `pump_setpoint` is not gated; `hold_now` before arm must still work
- Never-seen pose is not a dropout
- `flight-sim` must not depend on `flight-px4`, `flight-hitl`, `flight-ros2`, or
  `flight-ardupilot`
- `--backend px4-sitl` is converted corpus, not live spawn. Live SIH is
  `sitl_live --ignored` + CI job `sitl`
- `LeftoverContract::live_sitl_safe` is `inject != TriggerFailsafe` (live SIH
  leftover must not send `flight_termination` before takeoff/hold/land)
- `World::assemble` is `pub(crate)`. Custom scene names are not in
  `World::SCENARIOS` / `Lab::open`
- Include workspace docs from `crates/*/src/lib.rs` as `../../../docs/...`
- `TRYBUILD=overwrite` rewrites **all** stderr; copy ACTUAL stderr for
  new/changed UI tests only
- Do not add `declare_airborne` to kernel OffboardControl `commands`
- PX4 / ArduPilot `refuse_revoked_setpoint` **must** live on `impl VehicleBackend`
  (inherent-only helpers are invisible to trait default methods)
- After failsafe, `failsafe_latched` may be true while the `actuation_revoked`
  field is still false — trait default refuse must see the latch
- Do not refuse PX4/ArduPilot setpoints merely because `!armed`
- Do not put `safety::step` on NullBackend / SimBackend / PX4 setpoints
  (`hold_now` before arm would fail kernel MissionCommand from Ready)
- Lab certificate `fleet_hold_simultaneous` is **not** in the 22-property plant
  vector
- Do not spend a slice on live Gazebo or D\* morphologies unless this plan’s
  G1–G6 are already proven against the tree

---

## 3. Already true (do not re-implement)

Evidence lives in remaining-spec §1, README, NEXT Phase A/B/E/F stamps, and
tests. Snapshot at the commit that added this file:

- Consume-self typestate (aerial / ground / marine); trybuild **135**
- Non-`Clone` `ActuationPermit`; leftover Armed cannot `enter_offboard_now` or
  `set_motor_thrust_now`; leftover Takeoff cannot `declare_airborne_now`
  (`stale_takeoff_handle_cannot_declare_airborne`,
  `world_disarm_revokes_attached_takeoff_airborne`, ROS 2
  `leftover_after_disarm`, HITL leftover after deadline miss / revoke table,
  PX4 / ArduPilot unexpected-disarm leftover Takeoff)
- Hardware-boundary refuse after `actuation_revoked` / failsafe latch: Null,
  Sim, PX4, ArduPilot, World aerial/ground/marine (yaw and climb included)
- `no_std` kernel path; Creusot 81 on `safety` / `ground` / `marine` / `hitl`;
  Kani 45 including `permit_epoch_mismatch_is_stale` and
  `dsl_revokes_match_kernel`
- Temporal types + monitors; GPS-loss / heartbeat-stale / IMU-loss leftover
  contracts; HITL `Rate` / `Deadline` fail-close
- `define_aerial_authority!` single-source table; generated now-methods;
  leftover `COMMANDS`; diagrams under `docs/generated/`
- `Position` is `Point3`; same-frame pose add does not compile
- Differential contract on world / JSONL / ULog; leftover tables on PX4,
  ArduPilot, HITL, ROS 2; live SIH leftover for live-safe contracts
- Copper: `docs/copper.md` only; no Copper crate dependency
- Certification: tables + `human_readable_spec()` + FC-INV ids
- Agentic lab (observe / legal_cmds / research / MHS-shaped adapter) is v0
  landed and **must stay green**; it is not the civilization bottleneck

---

## 4. Honest gaps (work from evidence, not NEXT stamps)

Treat NEXT “F1–F10 landed” as **partial**. These are still false or incomplete:

| ID | Gap | Evidence |
| --- | --- | --- |
| H1 | Every leftover typed phase-change that grants authority is permit-gated | Armed, Offboard setpoints, Takeoff climb-complete, land, thrust are gated. Audit remaining `apply_*` / attach walks that still step the leftover **local** kernel without `require_permit` / `require_live_permit` when the event is not a safety ungated action (failsafe / land / disarm / recover / touchdown stay ungated). |
| H2 | All physical-authority commands pass through the kernel TCB | World backends step the kernel. Null / Sim / PX4 / ArduPilot refuse at the companion bit so `hold_now` before arm still works. That split is load-bearing. It is **not** “every backend runs `safety::step` on setpoints.” Document and test the split; do not “fix” it by putting kernel MissionCommand on PX4 setpoints. |
| H3 | Same contract on **live** PX4 SITL spawn, not only converted corpus | `--backend px4-sitl` reads `crates/flight-sim/corpus/px4_sitl_*.jsonl`. Live SIH is `sitl_live --ignored`. Do not advertise corpus as live spawn. |
| H4 | Live leftover `TriggerFailsafe` | Loopback-only: PX4 `inject_revoke(TriggerFailsafe)` sends `flight_termination`. Keep `live_sitl_safe` exclusion until a non-destructive live inject exists. |
| H5 | DSL does not emit extra Creusot `ensures` or a second typestate | 81 libraries stay unless you re-run prove. Do not generate typestate from the DSL (P1). Do not add `declare_airborne` to kernel `commands`. |
| H6 | Real-log torture, not only synthetic JSONL | Native ULog subset exists. A recorded **incident-shaped** ULog (GPS-loss / unexpected disarm) evaluated by the same `Requirement`s is still the civilization demo. |
| H7 | Copper integration crate | Docs only. No scheduler/pubsub/physics. An optional adapter that holds `Vehicle<S, B>` is later G8. |
| H8 | HITL on metal | FCH1 UDP mock + corpus is landed. A recorded pass against a physical card is G9. |
| H9 | Certification beyond tables | IDs exist. DO-178C / ASTM / STPA artifacts, independent review, and a requirements-to-test matrix that a DER would accept are not in the tree. |
| H10 | README 8-step story as **one** runnable path | Pieces exist (trybuild, kani job, `flight-test --scenario gps-loss`, leftover bins, ulog replay). A single documented command sequence that a stranger can run is still a gap. |
| H11 | `autonomy-contracts` crate split | Not started. In-monorepo split is allowed; collapsing P1–P14 is not. |

---

## 5. Agent operating rules

1. Read this file, remaining-spec P1–P14, and the current tree **before**
   coding. The working tree is authoritative.
2. Pick the **highest-priority open G-slice** in §6. Do not skip to Gazebo,
   morphologies, a new scheduler, a new MAVLink stack, or a ROS clone.
3. One logical change per commit. Code commit then docs commit when both
   change. Push `origin/main`. No PRs unless asked.
4. Before claiming a slice landed: name the test / bin / trybuild / Kani
   harness that proves it. A compile is not evidence.
5. If you find a leftover typed API that still grants kernel authority after
   revoke, that is G1 — do it **before** G5–G10.
6. Keep leftover Takeoff coverage: `leftover_declare_airborne_stale` on inland
   HITL / ROS 2 leftover (Takeoff bind). Do not drop it when editing revoke
   tables.
7. Do not mark the manifesto complete. Do not call a goal complete until §1
   items 1–9 are proven requirement-by-requirement against the tree.
8. Creusot still compiles only `safety` / `ground` / `marine` / `hitl`. Adding
   `ensures` on typestate will not be proved.

---

## 6. Slices (execute in order)

Each slice: **goal**, **touch**, **prove**, **don’t**. A slice is done only when
**prove** is green on the tree.

### G0. Always-on gates (every turn)

**Goal:** v0 stays green.

**Prove:** fmt, clippy `-D warnings`, workspace tests, `flight-core --no-default-features`,
trybuild 135, Kani 45, Creusot 81, sitl job still required in CI.

**Don’t:** bump MSRV; exclude a CI job to make a slice easier.

### G1. Remaining revocable permits (priority 1)

**Goal:** no leftover typed handle can complete a physical-authority transition
after epoch bump. Safety ungated actions (failsafe, land, disarm, recover,
touchdown, `pump_setpoint`) stay ungated.

**Next work (after leftover Takeoff climb-complete, which has landed):**

1. Audit every leftover `Vehicle` / `GroundVehicle` / `MarineVehicle` method
   that calls `apply_event` or a kernel event **other than** ungated safety.
   If it can succeed after `revoke_authority` on NullBackend, gate it with
   `require_permit` or `require_live_permit` **before** the kernel event
   (pattern: `set_motor_thrust_now`, `start_takeoff_now`, `apply_airborne`).
2. Leftover `Vehicle<Landing>` / `Vehicle<Airborne>` that can still change
   plant grant without a permit: add a leftover helper **only if** inland
   leftover binds that kind. Do not add `declare_airborne` to kernel
   `commands`.
3. Keep PX4 / ArduPilot trait-impl `refuse_revoked_setpoint` covering
   failsafe latch **and** `actuation_revoked`. New trait-default physical
   commands must call it.
4. World leftover after sibling `attach_disarm` / `attach_failsafe` must be
   `StaleAuthority`, not a lucky plant `IllegalPhase`.

**Touch:** `crates/flight-core/src/vehicle/`, world leftover tests,
`flight-px4` / `flight-ardupilot` / `flight-hitl` / `flight-ros2` leftover
tables.

**Prove:** NullBackend revoke test for the new method; world two-handle test;
companion leftover table if inland leftover binds that typestate.

**Don’t:** gate `enter_offboard_now` on heartbeat; gate `hold_now` before arm;
put epoch in packed `SafetyState`; make `ActuationPermit` `Clone`.

### G2. Tiny kernel TCB (priority 2)

**Goal:** verified-world physical authority still goes through
`safety::step` / `ground_step` / `marine_step`. Companions stay PX4-shaped
refuse bits so hold-before-arm works. Document that split in remaining-spec
as load-bearing (not a bug).

**Prove:** world plant tests already exist; add a remaining-spec sentence that
Null/PX4 setpoints must **not** call `safety::step`; a test that `hold_now`
after `connect` (never armed) is not `actuation_revoked`.

**Don’t:** “unify” companions by running kernel MissionCommand on PX4
setpoints.

### G3. Temporal contracts (priority 3)

**Goal:** every leftover revoke that is physically a time bound uses
`HeartbeatFresh` / `CommandFresh` / `EstimateFresh` / `Lease` / `Rate` /
`Sequence` — not a raw `u32` compare that can drift from the kernel
predicate.

**Prove:** monitors fail closed if typed Fresh disagrees with kernel
predicates (already true for several). Close any new setpoint path that
compares ages by hand.

**Don’t:** use `u64::from` in `const fn` for `Deadline::for_command` (MSRV);
treat never-seen pose as dropout.

### G4. Contract DSL (priority 4)

**Goal:** new aerial authority facts come from `define_aerial_authority!`, not
a parallel table.

**Prove:** generated `docs/generated/*` lockstep tests; leftover names
lockstep `AERIAL_OFFBOARD_LEFTOVER`; Kani harness count unchanged unless
intentionally updated.

**Don’t:** emit extra Creusot `ensures` without re-proving 81; generate
typestate; merge the two ident lists (needs MSRV bump); add
`declare_airborne` to `commands: [set_velocity, set_position, hold]`.

### G5. Production PX4 (priority 5)

**Goal:** a companion computer using `flight-px4` on live SIH (then real
hardware) is a trustworthy boundary.

**Next work:**

1. Keep `sitl` CI: takeoff/hold/land **and** leftover live-safe contracts.
2. Leftover Takeoff on live SIH if the live grant is Takeoff (today live
   leftover is Offboard). Do not loop `TriggerFailsafe`.
3. Do not rename `--backend px4-sitl` corpus into “live.” If you add live
   spawn, it is a new backend name or an explicit flag, plus a recorded log.
4. Unexpected disarm HEARTBEAT leftover Armed **and** Takeoff already have
   loopback tests; keep them.

**Prove:** `cargo test -p flight-px4 --test sitl_live -- --ignored` in CI;
`flight-test-px4` leftover table locally.

**Don’t:** second MAVLink stack; `flight-sim` → `flight-px4` dependency;
live `flight_termination` before takeoff.

### G6. Differential fault lab (priority 6)

**Goal:** the same leftover contract is true on world, JSONL, ULog, converted
SITL corpus, HITL rack, ROS 2 plant, PX4 companion.

**Next work:**

1. Keep `run_*_leftover_contracts` lockstep with
   `AerialOffboard::LEFTOVER_CONTRACTS`.
2. Add one **real** ULog (or a documented sanitised public log) that trips
   GPS-loss or unexpected disarm and shows leftover `COMMANDS` stale.
3. Leftover Takeoff climb-complete is already on HITL/ROS 2; world revoke
   table leftover is Offboard — do not force Takeoff into world
   `run_revoke_table` (that grant is `attach_offboard` by design).

**Prove:** `flight-test --backend all` / leftover bins; new ulog corpus
checked in.

**Don’t:** Gazebo; duplicate gps-loss as a second IMU-delay leftover row.

### G7. Rich geometry (priority 7)

**Goal:** more physical misuse unrepresentable without copying Copper.

**Prove:** trybuild count updated in remaining-spec / README /
`docs/agentic-spec.md` if you add a `.rs`; `Position` stays `Point3`.

**Don’t:** `Transform` translation as a free `Vector3<Meter>`; mixed-frame
`Add` on points.

### G8. Copper integration (priority 8)

**Goal:** a Copper task can hold `Vehicle<S, B>` and only call legal methods.
Setpoints still pass permit + kernel at the boundary.

**Prove:** optional adapter crate **or** a documented example that depends on
Copper **without** importing Copper’s scheduler into this workspace’s runtime.

**Don’t:** add a generic scheduler, pub/sub, or physics engine; depend
`flight-sim` on Copper.

### G9. HITL / reference hardware (priority 9)

**Goal:** recorded pass on a physical FCH1 (or successor) card, same leftover
contract as the UDP mock.

**Prove:** corpus + log in `crates/flight-hitl/corpus/`; leftover after a real
deadline miss.

**Don’t:** replace the in-process plant with the card in unit tests.

### G10. Certification traceability (priority 10)

**Goal:** a reviewer can go from FC-CAP / FC-INV ids to kernel predicate to
trybuild / Kani / leftover test without reading folklore.

**Next work:** leftover Takeoff in FC-INV-002 runtime column (landed). Expand
the matrix for leftover Armed, leftover Offboard `COMMANDS`, leftover Takeoff
climb, companion refuse bit vs kernel TCB. Do not claim DO-178C compliance.

**Prove:** `docs/generated/traceability.md` lockstep; remaining-spec does not
contradict the matrix.

**Don’t:** invent a second ID scheme; reuse retired ids.

### G11. Stranger-runnable proof story (civilization interface)

**Goal:** README steps 2–8 are a copy-paste sequence with expected outputs,
runnable on a clean clone (Kani/Creusot/SITL as documented optional jobs).

**Prove:** a `docs/` or `examples/` path that runs trybuild `unsafe_mission`,
points at `cargo kani -p flight-verify`, `flight-test --scenario gps-loss`,
and ulog replay. Do not require Gazebo.

**Don’t:** a chat UI that bypasses `legal_cmds`; auth/cloud fleet; crates.io
publish as a substitute for the proof story (publish is still a non-goal
unless someone asks).

---

## 7. Explicit non-goals (still)

- Live Gazebo / world renderer
- D\* morphologies (VTOL, tracked, extra hulls) before G1–G6
- Another scheduler, pub/sub, physics engine, MAVLink stack, ROS clone, flight
  controller, or huge sensor-driver collection
- Competing with Copper on deterministic runtime
- Putting kernel `MissionCommand` on Null/PX4 setpoints
- Collapsing P1–P14
- Vercel / serverless / multi-user lab / authentication product
- Official MHS until that schema is public; `flight-mhs` stays `official=false`

---

## 8. Done when (manifesto audit)

Do not mark complete because tests are green. For each numbered item in §1,
cite the file, test name, or recorded command that proves it. If any item is
only “consistent with” completion, it is not done.

Civilization add-on (same audit): a stranger can run the README proof story
(G11) and leftover Takeoff/Armed/Offboard have no authority after revoke on
Null, world, PX4 loopback, HITL, and ROS 2 plant.
