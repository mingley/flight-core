# Agentic robotics tooling — north-star spec

This is the product north star for this repository.

> Build world-class **agentic tooling for robotics**, in **Rust**, end to end: **experimenting**, **controlling**, and **understanding** every domain and every aspect of a robot system.

[`docs/remaining-spec.md`](remaining-spec.md) is the **v0 slice** that already landed (typestate vehicles, verified plant, lab observe/act/research, PX4 SIH, Creusot/Kani). Its §2 table (**P1–P14**) is still law. This file is the larger goal. [`docs/NEXT.md`](NEXT.md) is the ordered backlog with acceptance criteria.

Land work as atomic commits on `main`. Do not accumulate large diffs. Do not open pull requests unless someone asks. Do not collapse kernel vs typestate splits to make an agent “easier.”

---

## 1. Why this exists

Robotics tooling today is a pile of languages that do not share a type system:

- C++ autopilots and middleware (PX4, ROS 2) with incomplete Python bindings
- Python notebooks for experiments that cannot see legal state
- Separate C++ simulators whose contact, hydro, and energy models are not the same objects an agent commands
- Proofs, if they exist, live in another repo and another language

An agent — a human, a program, or a language model with tools — cannot **experiment** (run a closed loop and trust the certificate), **control** (the same API on sim and metal), or **understand** (why a command was rejected, which property would break, what the proof actually says) unless those three verbs share one Rust model.

This repository’s design principle stays:

> Don’t bind to a C++ robotics API. Create the API robotics should have had if ownership, capabilities, units, reference frames, contact, and legal state transitions were part of the language.

The north star adds one sentence:

> That API is also the **tool surface** an agent uses. Illegal motion is unrepresentable or rejected by the same machines. A run returns a **certificate**, not a screenshot.

Rust is not a wrapper language here. Kernel machines, plant step, attach walks, sensor traits, MAVLink/ROS 2 companions, HITL framing, JSON/MCAP traces, trybuild, Kani, and Creusot are all Rust. Later domains (manipulators, extra airframes, extra ground morphologies) join as Rust typestate + plant properties, not as a second stack.

---

## 2. What “agentic” means

An agentic robotics tool is one where a caller that is **not** a specialist in the kernel can still:

1. **Experiment** — open a catalog world, set seed and environment, run a policy or a sweep, get a property vector, replay the exact acts, and re-run a counterexample until the certificate is green or the bug is isolated.
2. **Control** — command vehicles through **consume-self typestate** and **attach walks** against simulation, PX4 SITL, ROS 2, and HITL, without a second “unsafe agent API.”
3. **Understand** — read legal commands, machine kind vs plant phase, hold targets, contact, energy, hydro, and proof artifacts as first-class observations, not log archaeology.

Non-goals that look agentic but are not this product:

- A chat UI that bypasses `legal_cmds` and writes raw setpoints
- A cloud fleet / multi-user lab / authentication product
- A Vercel or serverless deploy of the long-running sim
- Replacing PX4’s EKF with a marketing claim

The agent is always a **client of the machines**. `Lab::act` / `act_through_attach` / typed `Vehicle` methods are the only actuators. `World::try_step` is the only commit.

---

## 3. Three verbs

### 3.1 Experiment

**Today (v0, landed):**

- `Lab::open` catalogs: `coastal`, `harbor`, `inland`, `open_water`
- `Lab::observe` / `act` / `act_through_attach` / `research` / `replay_until` / `research_probe`
- Typed agents (`TypedHold`, `TypedFleetHold`, `TypedAerialFailsafe`, …) with `actions_applied == 0` for purely legal motion
- JSON probe twins for operator-shaped acts
- JSONL + Foxglove-shaped MCAP bags
- `WorldImu` + `FuzzedImu` sample the plant without replacing `WorldSession::step`
- Seed is physics (wave phase, gust), not a comment

**World-class means:**

| Capability | Acceptance sketch |
| --- | --- |
| Tool-gated acts | Every agent tool enumerates `legal_cmds` (plus `env_cmds`). A tool call whose `cmd` is not on that list is rejected **before** attach, with a structured reason. |
| Schema | `Observation` and `AgentAction` have JSON Schema (or equivalent) generated from the Rust types and checked in CI against bags. |
| Experiment runner | A CLI/library takes (scenario, seed range, agent or script, dt, steps) and writes a certificate directory: observation/action JSONL, MCAP, `ResearchRun`, broken property ids. |
| Sweeps | Same runner over seeds and named env perturbations (`set_wind` / `set_waves` / `set_current` / `set_charge`) without dropping P11 catalog bodies. |
| Counterexamples | A Kani/trybuild/Creusot failure, or a broken property id, can be turned into a lab replay that reproduces the refusal or the broken vector. |
| Isolation | One failing property / one illegal cmd / one body can be bisected without rewriting the agent. |

### 3.2 Control

**Today (v0, landed):**

- `Vehicle<S, B>`, `GroundVehicle<S, B>`, `MarineVehicle<S, B>` with 125 trybuild compile-fails
- `OffboardControl` gates velocity / position / hold; `MotorsEnabled` gates motor thrust; Recovery is a real aerial typestate
- `WorldSession` attach walks shared by HITL, ROS 2, PX4 `WorldPlant`, and `robot-lab`
- Live PX4 SIH companion (`sitl_live`, CI job `sitl`): unpacked `PX4_MAIN_MODE_OFFBOARD`, climb/hold stay in offboard, `NAV_LAND` for land
- ROS 2 `rclrs` plant nodes; HITL FCH1 framing with miss ⇒ zero command
- Aerial NED pose hold on the plant (`hold_ned`, `position_hold_restores_pose`); ground Moving hold and marine Underway/StationKeep DP use the same field and Kani restore fact

**World-class means:**

| Capability | Acceptance sketch |
| --- | --- |
| One API, many backends | The same typestate methods run on `WorldSession`, PX4 UDP, ROS 2, and HITL. New autopilots implement `VehicleBackend` / domain backends; they do not grow a parallel command enum. |
| Domain-complete hold | Ground GPS/pose hold and marine dynamic positioning are typed, plant-backed, and Kani-checked — not JSON-only. Compile-fail from Parked / EStop / Docked / Failsafe as specified in NEXT. Preserve P3. |
| Estimation in the loop | A navigation filter can clear kernel `estimator_valid` from bad IMU/GNSS **without** replacing plant quaternion physics-truth (`unit_attitude`). |
| Planning as typestate | Waypoints / paths are data. Executing them still requires Offboard / Moving / Underway (or the documented marine station machine). No “planner override” that skips attach. |
| Coordination | Pairwise sphere contact plus lab `fleet_hold_simultaneous`. Formation / right-of-way remain later certificates, not comments in an agent prompt. |
| Metal | At least one recorded physical FCH1 (or faithful rack mock) pass, and companion paths for additional autopilots, still in Rust. |
| Deploy | Discrete kernel machines remain `no_std`-capable. Vehicles stay `std` until an explicit typestate-on-embedded decision (P14 / remaining-spec §6.1). |

### 3.3 Understand

**Today (v0, landed):**

- Observation: pose, energy, contact, `legal_cmds`, `hold_ned`, aerial/ground/marine machines, 22 named properties
- Bags with `/lab/observation` and `/lab/action`
- Creusot 0.5 on discrete machines (81 libraries); Kani 42 harnesses on f32 facts
- Demo console on `FLIGHT_DEMO_BIND` (default `47831`)

**World-class means:**

| Capability | Acceptance sketch |
| --- | --- |
| Why rejected | Every `LabError` / `BackendError::Protocol` / kernel `Reject` is a structured observation an agent can read (machine, event, from-phase, to-phase, which split P1–P14 applies). |
| Kind vs phase | Agents already see attach `kind` vs plant `phase`. Docs and schemas treat that as load-bearing, not debug. |
| Proofs as input | Creusot/Kani summaries (what was proved, what stays Kani-only f32) are artifacts the experiment runner can attach to a run. |
| Causal traces | If a step would break a named property, the refuse path records which body, which field, which property id — not only `all_hold: false`. |
| Scenario DSL | Named scenes beyond the four catalogs, still omitting bodies per P11 rules (inland = no hull, open water = no rover) unless the catalog is explicitly new. |
| Scale | Finer hydro / more bodies only with property-preserving tests. GPU remains a performance path, not bit-identity with CPU. |

---

## 4. Domains

Every domain is a first-class Rust machine + plant body kind. Do not drop AUV. Do not invent placeholder bodies in a catalog that P11 says omits them.

| Domain | v0 vehicle | v0 plant body | Later (see NEXT) |
| --- | --- | --- | --- |
| Aerial | `Vehicle` (Disconnected → … → Airborne / Offboard / Failsafe / Recovery) | `drone` | VTOL, fixed-wing, multi-rotor variants; still NED z-down |
| Ground | `GroundVehicle` (Parked / Moving / EStop) | `rover` | pose hold, tracked/legged morphologies, more than one rover per catalog |
| Surface | `MarineVehicle` (Docked / Underway / StationKeep / Failsafe) | `skiff` | DP pose hold, extra hulls, docking geometry richer than a catalog dock event |
| Underwater | same marine machine, AUV wrench | `surveyor` | depth hold distinct from surface station, tethered ROV later |
| Manipulation | — | — | consume-self arm/gripper typestate; contact already in the sphere/terrain vector |
| Multi-agent | four-body catalogs + `TypedFleet*` | coastal/harbor mix | N-body certificates, shared hold, traffic |
| Space-adjacent | — | — | only if a vacuum/orbital plant is specified with properties; not a web app |

Catalogs stay:

| Scenario | Bodies |
| --- | --- |
| `coastal` / `harbor` | drone + rover + skiff + surveyor |
| `inland` | drone + rover (no hull) |
| `open_water` | drone + skiff + surveyor (no rover) |

New catalogs must declare bodies explicitly and get a typed agent skip list (see `TypedFleetHold`).

---

## 5. Aspects (every one in Rust)

“Every aspect” means the agent, the operator API, and the plant share the **same** model for that aspect. A Python-only planner or a C++-only estimator is a companion **behind** a Rust backend, not a second source of truth.

| Aspect | v0 | North star |
| --- | --- | --- |
| Units and frames | `Vector3<U, F>`, NED ↔ ENU explicit | Same; no silent frame mix in agent JSON (schema states NED vs ENU per field) |
| Safety machines | aerial / ground / marine packed kernels + typestate | Unchanged splits P1–P9; new events get both layers and a trybuild |
| Mechanics | rigid bodies, quaternion physics-truth, Coulomb sphere contact, terrain | Richer contact only with named properties |
| Hydro | Rusanov Saint-Venant, GPU optional | Finer grid only with volume/land-dry tests; not a coastal-engineering product by default |
| Energy | battery gates thrust | Power/thermal models as properties when added |
| Sensing | `Imu` traits, `WorldImu`, fuzz, jsonl replay | Camera/GNSS/DVL as typed samples feeding estimation, not replacing the plant |
| Estimation | `estimator_valid` bit; complementary filter not in `try_step`; `update_nav` may trip the bit | ESKF/GNSS fusion is a navigation crate, plant pose stays physics-truth until a written decision |
| Control | velocity / position / aerial hold / twists / wrenches / ground hold / marine DP | Allocation, actuator limits as machines |
| Planning | `Waypoint` / `NedPath`; execute through legal attach | Richer planners still Offboard / Moving / CanThrust |
| Comms | MAVLink, ROS 2 CDR/`rclrs`, HITL FCH1, demo HTTP | More autopilots; still Rust. No cloud bus required |
| Verification | trybuild, exhaustive packed machines, Kani, Creusot, 22 plant properties | Proofs as agent artifacts; new properties for new physics |
| Human I/O | `flight-demo` console | Keep; not a second product. Native GUIs stay optional |
| Agent I/O | JSON act/observe, research trait, bags | Schema, tool adapter, experiment runner, rejection traces |

---

## 6. Architecture (stable)

```text
                    ┌─────────────────────────────────────────┐
                    │  Agent / operator / LLM tools / demo / MCP        │
                    │  observe · legal_cmds · MHS-shaped read/write     │
                    └──────────────────┬──────────────────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────────────────┐
                    │  flight-mhs  Driver  (discover · reference · chain)  │
                    │  official=false · writes = Lab::act_through_attach  │
                    └──────────────────┬──────────────────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────────────────┐
                    │  robot-lab  Lab  (JSON + attach walks)               │
                    └──────────────────┬──────────────────────┘
                                       │
     flight-px4 / flight-ros2 / flight-hitl / Vehicle<S,B>
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │  WorldSession attach walks              │
                    │  flush all grants → one World::try_step │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │  robot-world  22 named properties       │
                    │  NED z-down · catalogs · hydro · contact│
                    └─────────────────────────────────────────┘

     flight-core   units, frames, sensors, kernels, typestate, mech, hydro
     flight-verify Kani harnesses (f32 facts)
     Creusot       discrete machines (cfg(creusot) subset)
     flight-sim    SimBackend = point-mass demo, not the property vector
```

Rules that do not change:

- P12: flush all granted setpoints, then **one** `WorldSession::step`.
- P13: ungranted aerial `clear_command()` **wipes** `hold_ned`.
- Plant quaternion is physics truth. Kernel `estimator_valid` is a safety bit. `WorldSession::update_nav` may clear that bit on unusable IMU.
- `grant_*` are attach walks. Do not wrap `failsafe_now` / `takeoff_now` / `land_now` as recursive `attach_*`.
- `SimBackend` is not the verified world.
- MSRV **1.85** (P14). Creusot 0.5 / Kani installer rustc are not MSRV bumps.
- Publish: library + long-running sim, not Vercel.

---

## 7. Agent contract (normative)

### 7.1 Observe

`Lab::observe` → `Observation` is the agent’s world state. Required fields that must remain (names may grow, not silently rename):

- Time, scenario, seed, `all_hold`, property vector
- Environment (wind, current, hydro metadata)
- Per robot: id, domain, phase, attach kind, pose/twist, energy, contact, `legal_cmds`, optional `hold_ned`, domain machine

Agents **must** treat `legal_cmds` as the tool enum for that body at that time. `env_cmds` is the tool enum for environment acts.

### 7.2 Act

`AgentAction` is `{ robot, cmd, vn, ve, vd, yaw_rate }` with `LabCmd` as the closed command set. New operator acts need:

1. A `LabCmd` variant and `accepted_by` / `legal_cmds` wiring
2. An attach walk (not a second event path)
3. A typed research agent **and** a JSON probe twin (remaining-spec §4.5)
4. Catalog skips where P11 omits the body

JSON `Disarm` from Failsafe vs PX4 operator DISARM stays **P6**. Do not unify them for agent convenience.

### 7.3 Research

`ResearchAgent::act` may grant through handles or return JSON actions. `Lab::research` applies through `act_through_attach`, then one verified step. Typed legal motion keeps `actions_applied == 0`. Probe agents may count rejections.

A world-class runner additionally writes a **run record**: scenario, seed, git commit, dt, steps, agent name, `ResearchRun`, bag paths, proof-artifact hashes when present.

### 7.4 Tools

The first-class agent interface is **not** “any HTTP POST.” It is:

1. Rust: `Lab` + typestate handles
2. JSON: observe / act / research as today, with Schema
3. Optional local tool adapter (OpenAPI or MCP-shaped) that **only** exposes legal commands plus env, observe, step, replay, probe. `flight-mhs` is that adapter for Model Hardware Standard–shaped hosts: discover, compiled reference, read, write, chain. It is not official MHS.

No tool may step the plant twice for one agent tick. No tool may grant two competing setpoints without P12 flush-then-one-step.

---

## 8. Verification bar

A feature is not done because it demos. It is done when:

1. **Types** — illegal calls fail to compile (trybuild) or fail attach with Protocol
2. **Kernel** — packed machine agrees; Creusot still proves the discrete subset; new f32 facts get Kani
3. **Plant** — `World::try_step` still commits only if the named vector holds; new physics ⇒ new named property
4. **Agent** — typed agent + JSON probe; observation shows the new field; replay_until matches
5. **Companion** — if the feature is a vehicle command, WorldPlant / ROS 2 / HITL / Lab share the attach walk
6. **Docs** — README examples, remaining-spec invariants if a new split appears, NEXT box checked with evidence (test name or recorded log)

CI already: fmt, clippy `-D warnings`, workspace tests, no_std check, gpu hydro, kani (42), rclrs Jazzy, creusot (81), sitl SIH. New gates follow the same pattern. `cargo test` still takes **one** filter name. trybuild stays `=1.0.104`.

---

## 9. Relationship to the v0 remaining spec

| Document | Role |
| --- | --- |
| [`docs/remaining-spec.md`](remaining-spec.md) | v0 **invariants** (P1–P14), landed evidence, process notes. §13 lists items that were **not** v0. |
| This file | North star: agentic tooling, all domains, all aspects, Rust. |
| [`docs/NEXT.md`](NEXT.md) | Ordered next work with acceptance. Former §13 items that belong in the north star (ground hold, marine DP, ESKF-as-bit, FCH1 metal, scenario scale) live there as phases, not as “never.” |

v0 is **complete** as a slice. The product is not.

---

## 10. Explicit non-goals (still)

These stay out unless a later instruction adds them:

- Native mobile / desktop GUIs beyond `flight-demo`
- Authentication, multi-user lab, cloud fleet orchestration
- Vercel / serverless deploy
- Publishing to crates.io as a functional gap (version is `0.1.0`)
- Replacing PX4 firmware or claiming a full EKF/RTK/mission planner
- CPU/GPU hydro bit-identity
- Bumping MSRV to chase Creusot 0.8 or the Kani installer without an explicit decision

---

## 11. Definition of “world class” (for this repo)

Not a slogan. All of the following are true together:

1. An agent can **only** act through legal Rust machines, in every domain we ship.
2. Every experiment returns a **mechanical certificate** (property vector + replay).
3. Control on sim and companion does not fork APIs.
4. Understanding is structured (legal_cmds, rejects, proofs), not tribal knowledge.
5. Air, ground, surface, and underwater stay in one plant, one lab, one verification story.
6. New domains/aspects arrive as Rust types and properties, not as bindings.

When a NEXT item lands, point its acceptance at this list. If it weakens (1)–(6) or P1–P14, it does not land.
