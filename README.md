# flight-core

A strongly typed Rust SDK for using, testing, and researching robotics — aerial, ground, surface, and underwater — through **mechanically verified simulation**, and the base for **agentic** tooling (experiment / control / understand) in Rust. Not a C++ wrapper.

The design principle:

> Don't bind to a C++ robotics API. Create the API robotics should have had if ownership, capabilities, units, reference frames, contact, and legal state transitions were part of the language.

```rust
let vehicle: Vehicle<Disarmed, _> = px4.connect().await?;
let vehicle: Vehicle<PreflightReady, _> = vehicle.verify_preflight().await?;
let vehicle: Vehicle<Armed, _> = vehicle.arm().await?;
let vehicle: Vehicle<Offboard, _> = vehicle.enter_offboard().await?;
vehicle.set_velocity(Velocity::<Ned>::ned(1.0, 0.0, 0.0)).await?;

let rover: GroundVehicle<Parked, _> = GroundVehicle::new(backend);
let mut rover: GroundVehicle<Moving, _> = rover.enable_drive()?;
rover.set_twist(forward, yaw_rate).await?;

let mut drone = session.attach_takeoff("drone")?;
let mut rover = session.attach_drive("rover")?;
let mut skiff = session.attach_undock("skiff")?;
drone.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -1.2))?;
drone.set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))?;
drone.hold_now()?;
rover.set_velocity_now(Velocity::<Ned>::ned(-0.6, 0.0, 0.0))?;
skiff.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.4, 0.0))?;

let VehicleHandle::PreflightReady(drone) = session.aerial("drone").attach()?;
let mut drone = drone.arm().await?.takeoff(alt).await?;
let GroundHandle::Parked(rover) = session.ground("rover").attach()?;
let mut rover = rover.enable_drive()?;
```

These do not compile:

```rust
Vehicle::<Disarmed, _>::set_motor_thrust(...)          // motors require an armed typestate
Vehicle::<Disconnected, _>::arm(...)                   // arm requires preflight
Vehicle::<Armed, _>::arm_now(...)                      // arm is Ready only
Vehicle::<Offboard, _>::arm_now(...)                   // Offboard is already past Arm
Vehicle::<Takeoff, _>::arm_now(...)                    // climb is not Ready
Vehicle::<Airborne, _>::arm_now(...)                   // airborne is not Ready
Vehicle::<Landing, _>::arm_now(...)                    // landing is not Ready
Vehicle::<Failsafe, _>::arm_now(...)                   // failsafe is not Ready
Vehicle::<Recovery, _>::arm_now(...)                   // recovery is not Ready
Vehicle::<Disarmed, _>::arm_now(...)                   // Disarmed is not Ready
Vehicle::<Disconnected, _>::arm_now(...)               // arm is Ready, not Disconnected
Vehicle::<PreflightReady, _>::enter_offboard_now(...)  // offboard is Armed only
Vehicle::<Offboard, _>::enter_offboard_now(...)        // already Offboard
Vehicle::<Takeoff, _>::enter_offboard_now(...)         // climb is not Armed offboard entry
Vehicle::<Airborne, _>::enter_offboard_now(...)        // airborne is not Armed
Vehicle::<Landing, _>::enter_offboard_now(...)         // landing is not Armed
Vehicle::<Failsafe, _>::enter_offboard_now(...)        // failsafe is not Armed
Vehicle::<Recovery, _>::enter_offboard_now(...)        // recovery is not Armed
Vehicle::<Disarmed, _>::enter_offboard_now(...)        // Disarmed is not Armed
Vehicle::<Disconnected, _>::enter_offboard_now(...)    // offboard is Armed, not Disconnected
Vehicle::<Disconnected, _>::disarm_now(...)            // pad disarm is Ready through Landing (`CanDisarm`)
Vehicle::<Disarmed, _>::disarm_now(...)                // Disarmed is not CanDisarm
Vehicle::<Disconnected, _>::failsafe_now(...)          // Disconnected is not CanTripFailsafe
Vehicle::<Disarmed, _>::failsafe_now(...)              // pad failsafe is Ready or Armed, not Disarmed
Vehicle::<Failsafe, _>::failsafe_now(...)              // already-failsafe cannot re-trip
Vehicle::<Recovery, _>::failsafe_now(...)              // recovery is not CanTripFailsafe
Vehicle::<PreflightReady, _>::set_velocity(...)        // offboard setpoints are OffboardControl
Vehicle::<Armed, _>::set_velocity(...)                 // Armed is not Offboard; enter_offboard first
Vehicle::<Failsafe, _>::set_velocity(...)              // failsafe is not OffboardControl
Vehicle::<Recovery, _>::set_velocity(...)              // recovery is not OffboardControl
Vehicle::<Disarmed, _>::set_velocity(...)              // Disarmed is not OffboardControl
Vehicle::<Disconnected, _>::set_velocity(...)          // Disconnected is not OffboardControl
Vehicle::<PreflightReady, _>::set_position(...)        // offboard setpoints are OffboardControl
Vehicle::<Armed, _>::set_position(...)                 // Armed is not Offboard; enter_offboard first
Vehicle::<Failsafe, _>::set_position(...)              // failsafe is not OffboardControl
Vehicle::<Recovery, _>::set_position(...)              // recovery is not OffboardControl
Vehicle::<Disarmed, _>::set_position(...)              // Disarmed is not OffboardControl
Vehicle::<Disconnected, _>::set_position(...)          // Disconnected is not OffboardControl
Vehicle::<PreflightReady, _>::hold(...)                // hold is OffboardControl
Vehicle::<Armed, _>::hold(...)                         // Armed is not Offboard; enter_offboard first
Vehicle::<Failsafe, _>::hold(...)                      // failsafe is not OffboardControl
Vehicle::<Recovery, _>::hold(...)                      // recovery is not OffboardControl
Vehicle::<Disarmed, _>::hold(...)                      // Disarmed is not OffboardControl
Vehicle::<Disconnected, _>::hold(...)                  // Disconnected is not OffboardControl
Vehicle::<PreflightReady, _>::set_motor_thrust(...)    // motors are MotorsEnabled (Armed through Landing)
Vehicle::<Failsafe, _>::set_motor_thrust(...)          // failsafe is not MotorsEnabled
Vehicle::<Recovery, _>::set_motor_thrust(...)          // recovery is not MotorsEnabled
Vehicle::<Disarmed, _>::set_motor_thrust(...)          // Disarmed is not MotorsEnabled
Vehicle::<Disconnected, _>::set_motor_thrust(...)      // Disconnected is not MotorsEnabled
Vehicle::<Recovery, _>::disarm_now(...)                // pad disarm is Ready through Landing (`CanDisarm`)
Vehicle::<Offboard, _>::begin_land_now(...)            // land is Takeoff or Airborne (`CanBeginLand`)
Vehicle::<Armed, _>::start_takeoff_now(...)            // climb is Offboard only; enter_offboard first
Vehicle::<PreflightReady, _>::start_takeoff_now(...)   // pad climb is Offboard, not Ready
Vehicle::<Takeoff, _>::start_takeoff_now(...)          // already climbing
Vehicle::<Airborne, _>::start_takeoff_now(...)         // airborne returns via Land, not Takeoff
Vehicle::<Landing, _>::start_takeoff_now(...)          // landing is not Offboard climb
Vehicle::<Failsafe, _>::start_takeoff_now(...)         // failsafe returns via touchdown or Recovery
Vehicle::<Recovery, _>::start_takeoff_now(...)         // recovery is not Offboard
Vehicle::<Disarmed, _>::start_takeoff_now(...)         // Disarmed is not Offboard
Vehicle::<Disconnected, _>::start_takeoff_now(...)     // climb is Offboard, not Disconnected
Vehicle::<Offboard, _>::declare_airborne_now(...)      // airborne is Takeoff only
Vehicle::<Airborne, _>::declare_airborne_now(...)      // already airborne
Vehicle::<Landing, _>::declare_airborne_now(...)       // landing is not climb complete
Vehicle::<PreflightReady, _>::declare_airborne_now(...) // pad is not Takeoff
Vehicle::<Armed, _>::declare_airborne_now(...)         // Armed is not Takeoff
Vehicle::<Failsafe, _>::declare_airborne_now(...)      // failsafe is not Takeoff
Vehicle::<Recovery, _>::declare_airborne_now(...)      // recovery is not Takeoff
Vehicle::<Disarmed, _>::declare_airborne_now(...)      // Disarmed is not Takeoff
Vehicle::<Disconnected, _>::declare_airborne_now(...)  // Disconnected is not Takeoff
Vehicle::<Armed, _>::begin_land_now(...)               // land is CanBeginLand, not Armed
Vehicle::<PreflightReady, _>::begin_land_now(...)      // pad land is CanBeginLand, not Ready
Vehicle::<Landing, _>::begin_land_now(...)             // already landing; next is touchdown
Vehicle::<Disconnected, _>::begin_land_now(...)        // land is CanBeginLand, not Disconnected
Vehicle::<Failsafe, _>::begin_land_now(...)            // failsafe returns via touchdown or Recovery, not Land
Vehicle::<Armed, _>::touchdown_now(...)                // touchdown is Landing or Failsafe (`CanTouchdown`)
Vehicle::<Offboard, _>::touchdown_now(...)             // touchdown is CanTouchdown, not Offboard
Vehicle::<Takeoff, _>::touchdown_now(...)              // climb is not CanTouchdown; land first
Vehicle::<Airborne, _>::touchdown_now(...)             // airborne returns via Land, not Touchdown
Vehicle::<PreflightReady, _>::touchdown_now(...)       // already Ready
Vehicle::<Disarmed, _>::touchdown_now(...)             // Disarmed is not CanTouchdown
Vehicle::<Disconnected, _>::touchdown_now(...)         // Disconnected is not CanTouchdown
Vehicle::<Recovery, _>::touchdown_now(...)             // Recovery is not CanTouchdown; recover_now → Ready
Vehicle::<Recovery, _>::begin_land_now(...)            // land is CanBeginLand, not Recovery
Vehicle::<PreflightReady, _>::recover_now(...)         // recover is Recovery only
Vehicle::<Armed, _>::recover_now(...)                  // Armed is not Recovery
Vehicle::<Offboard, _>::recover_now(...)               // Offboard is not Recovery
Vehicle::<Takeoff, _>::recover_now(...)                // climb is not Recovery
Vehicle::<Airborne, _>::recover_now(...)               // airborne is not Recovery
Vehicle::<Landing, _>::recover_now(...)                // landing is not Recovery
Vehicle::<Failsafe, _>::recover_now(...)               // failsafe disarms to Recovery first
Vehicle::<Disarmed, _>::recover_now(...)               // Disarmed is not Recovery
Vehicle::<Disconnected, _>::recover_now(...)           // Disconnected is not Recovery
GroundVehicle::<EStopped, _>::emergency_stop_now(...)  // estop is Parked or Moving (`CanTripEstop`)
GroundVehicle::<Parked, _>::set_twist(...)             // drive requires Moving
GroundVehicle::<EStopped, _>::set_twist(...)           // drive requires Moving
GroundVehicle::<Parked, _>::park_now(...)              // halt requires Moving
GroundVehicle::<EStopped, _>::park_now(...)            // halt is Moving, not E-stop
GroundVehicle::<Moving, _>::enable_drive(...)          // release is Parked only
GroundVehicle::<EStopped, _>::enable_drive(...)        // E-stop is not Parked; clear first
GroundVehicle::<Parked, _>::reset(...)                 // clear E-stop is EStopped only
GroundVehicle::<Moving, _>::reset(...)                 // clear E-stop is EStopped only
MarineVehicle::<Docked, _>::set_ned_velocity(...)      // thrust is Underway or StationKeep (`CanThrust`)
MarineVehicle::<Docked, _>::dock_now(...)              // dock is Underway or StationKeep (`CanDock`)
MarineVehicle::<MarineFailsafe, _>::dock_now(...)      // dock is CanDock, not failsafe
MarineVehicle::<Underway, _>::undock(...)              // undock is Docked only
MarineVehicle::<StationKeep, _>::undock(...)           // undock is Docked only
MarineVehicle::<MarineFailsafe, _>::undock(...)        // undock is Docked only
MarineVehicle::<Docked, _>::hold_station(...)          // station is Underway only
MarineVehicle::<StationKeep, _>::hold_station(...)     // already station; resume to make way
MarineVehicle::<MarineFailsafe, _>::hold_station(...)  // station is Underway only
MarineVehicle::<Docked, _>::resume(...)                // resume is StationKeep only
MarineVehicle::<Underway, _>::resume(...)              // resume is StationKeep, not Underway
MarineVehicle::<MarineFailsafe, _>::resume(...)        // resume is StationKeep, not failsafe
MarineVehicle::<Docked, _>::recover_docked(...)        // recover is Failsafe only
MarineVehicle::<Underway, _>::recover_docked(...)      // underway is not Failsafe
MarineVehicle::<StationKeep, _>::recover_docked(...)   // station is not Failsafe
MarineVehicle::<Docked, _>::declare_failsafe(...)      // failsafe is Underway or StationKeep (`CanTripMarineFailsafe`)
MarineVehicle::<MarineFailsafe, _>::declare_failsafe(...) // already-failsafe hull cannot re-trip
MarineVehicle::<MarineFailsafe, _>::set_ned_velocity(...) // thrust is CanThrust, not failsafe
Position::<Ned> + Position::<Enu>                      // frames are types
AngularVelocity<DegreePerSecond, Body>                 // where rad/s is required
```

`GroundVehicle::new` / `MarineVehicle::new` / `Vehicle::new` always start Parked / Docked / Disconnected. `WorldSession::attach_takeoff` / `attach_drive` / `attach_undock` walk consume-self typestate (`arm_now` → `enter_offboard_now` → `start_takeoff_now`, `enable_drive`, `undock`) and return the live backend — the same grant HITL, ROS 2, and PX4 plants need. `WorldBackend::grant_offboard` / `GroundWorldBackend::grant_drive` / `MarineWorldBackend::grant_undock` are those same attach walks on an existing handle (not a second event path). `attach_offboard` stops after offboard (PX4 ARM) so `Land` is not legal until Takeoff fires. `attach_start_takeoff` is that Offboard → Takeoff climb (PX4 `NAV_TAKEOFF`). `attach_airborne` is Takeoff → Airborne (PX4 `NAV_LOITER_UNLIM`). `attach_hold` writes the current NED pose while attach is OffboardControl. `attach_ground_hold` writes the current NED pose while the chassis is Moving. `attach_marine_hold` writes the current NED pose while the hull is Underway or StationKeep (not the StationKeep machine). `attach_failsafe` trips Ready / Armed / Offboard / Takeoff / Airborne / Landing. `attach_reset` clears ground E-stop back to Parked. `attach_marine_failsafe` / `attach_recover` trip and recover a hull. `attach_recover_ready` walks aerial Failsafe → Recovery → Ready (`disarm_now` then `recover_now`); Recovery is its own consume-self typestate, not Disarmed. Pad `disarm_now` to Ready is `CanDisarm` (Ready through Landing); Failsafe `disarm_now` still returns Recovery. After `grant_drive` / `grant_undock` / `grant_offboard` on a world handle, `attach()` returns `GroundHandle::Moving` / `MarineHandle::Underway` / `VehicleHandle::Takeoff` bound to the live plant so consume-self typestate matches the chassis (`grant_offboard` includes the Takeoff event). `enter_offboard_now` without takeoff attaches `VehicleHandle::Offboard`. The handle enums expose `backend` / `backend_mut` / `into_backend` so now-APIs and a shared `WorldSession::step` stay on that plant. Research agents that cannot `.await` use `arm_now` / `enter_offboard_now` / `start_takeoff_now` / `declare_airborne_now` (world and null backends complete without parking; `start_takeoff_now` consumes Offboard into Takeoff and writes `Takeoff` on the live plant so `Land` is legal, and `declare_airborne_now` consumes Takeoff into Airborne). Attached Offboard / Takeoff / Airborne / Landing / Moving / Underway vehicles command through `set_velocity_now` / `set_position_now` / `set_velocity_ned_now` / `set_ned_velocity_now` without ticking the world. `GroundVehicle::park_now` is Halt back to Parked on that same plant and clears the handle setpoint so a later flush cannot revive drive. `GroundVehicle::emergency_stop_now` is E-stop from Moving. `MarineVehicle::dock_now` is Dock back to Docked from Underway or StationKeep (`CanDock`). `MarineVehicle::declare_failsafe` is the same from Underway or StationKeep. `begin_land_now` / `touchdown_now` are the same for pad return: landing is `CanBeginLand` (Takeoff or Airborne only), and touchdown is Ready (not a fake Disarmed) from Landing or Failsafe — the kernel already allows `Touchdown` from either. `Vehicle::land` walks the same Land → descent → Touchdown path on the plant (not a Disarm shortcut that skips Landing).

Rust does not automatically “verify” a robot. It lets you move physical-system correctness out of conventions and runtime checks into types, then into exhaustive machines and model checkers (Kani). This repo is that stack: **clear state**, **mechanical invariants after every sim step**, and APIs that make illegal behavior unrepresentable.

## What this is for

- **Use robotics** with one typestate API against sim, PX4 SITL, the verified multi-domain world, replay, or a symbolic harness.
- **Test robotics** in a virtual world whose step function is required to keep contact, drag, buoyancy, and actuation grants true.
- **Let agents research** through `robot-lab`: JSON observe / act over a coastal scene with drone, rover, skiff, and AUV. `Lab::research` returns a property certificate: the full mechanical vector (`properties`) and the last-step `sphere_hits` graph, not only `all_hold`. `Observation.broken` names the property ids from a refused `try_step` (refuse is atomic: pose / hydro / `t` stay); observe does not take an extra step. Each typed aerial / ground / marine machine carries `kind` (`AerialKind` / `GroundKind` / `MarineKind`) — the same map `attach` uses — next to the plant `phase` string, so after `attach_offboard` the agent sees `Offboard` while phase stays `armed`. Aerial views keep `imu_healthy` and `estimator_valid`. `CoastalFleet` reads typed aerial / ground / marine machines (not phase strings), probes illegal parked/docked/disarmed commands in one tick, then grants and moves every hull that is in the scene before the verified step. Rover drive is issued only while `terrain_contact` is true. `TypedFleet` does the same probes as JSON, then `Lab::attach_takeoff` / `attach_drive` / `attach_undock` and NED setpoints on consume-self handles — legal motion never goes through `Lab::act`, but matching intents are written to the action log so `replay_until` can reproduce the run. `TypedAttachFleet` is the same attach policy with a distinct certificate name. `set_velocity_now` / `set_position_now` fire the same `MissionCommand` / `DriveCommand` / `ThrustCommand` events as JSON `act`, so parked, docked, and disarmed setpoints are `Rejected` on both paths. `Lab::act` takes a `LabCmd` enum (same snake_case JSON as the demo console); unknown command names fail deserialize instead of reaching the plant. `LabCmd::Hold` walks `attach_hold` (current NED pose) on aerial OffboardControl, `attach_ground_hold` on a Moving rover, and `attach_marine_hold` on an Underway or StationKeep hull; `LabCmd::Position` holds a named pose (aerial only). Each `RobotView` lists `legal_cmds` — the commands the live aerial / ground / marine machine would accept — so agents can observe legal acts instead of probing strings. The live demo and `Lab::research` apply posted JSON through `Lab::act_through_attach` (attach helpers and now-APIs, with JSON fallback); `replay_until` walks those same helpers without re-logging (JSON fallback when attach is Protocol). `research_probe` keeps the illegal catalog on `Lab::act` so parked Takeoff cannot leak through `attach_takeoff`, and applies the legal sequence through `Lab::act_through_attach`. `PadLanding` watches `terrain_contact`: climb off the pad, land, `touchdown` to Ready. `TypedPadLanding` is the same policy through `Lab::attach_airborne` / `attach_land` / `attach_touchdown` so the certificate keeps `actions_applied == 0` and the log still replays. `CollisionSweep` drives the inland rover into the drone until the `sphere_hits` graph names that pair, and still returns a holding property certificate. `TypedCollisionSweep` is the same policy through `Lab::attach_drive` / `attach_park` so `actions_applied` stays 0 and the log still replays. `TypedStationDock` probes docked thrust, then `Lab::attach_undock` / `attach_station` / `attach_dock` on the skiff (`MarineKind`, not the `"stationkeep"` string that never matched `station_keep`). `TypedHullDock` is the same probes then `Lab::attach_dock` from Underway without station. `TypedStationResume` is the same probes then `Lab::attach_resume` back to Underway without docking. `TypedHullFailsafe` probes docked thrust, then `Lab::attach_undock` / `attach_marine_failsafe` / `attach_recover` so failsafe and recover are research certificates, not JSON. `TypedAerialFailsafe` probes disarmed velocity, then `Lab::attach_takeoff` / `attach_failsafe` / `attach_recover_ready` so aerial Recovery is a research certificate. `TypedAerialDisarm` probes disarmed velocity, then `Lab::attach_takeoff` / `attach_disarm` to Ready without failsafe. `TypedAerialAirborne` probes disarmed velocity, then `Lab::attach_takeoff` / `attach_airborne` / `attach_land` so climb-complete land is a research certificate without touchdown. `TypedPositionHold` probes pad velocity, then `Lab::attach_takeoff` / `set_position_now` so a NED hold is a research certificate without airborne or land. `TypedHold` is the same grant then `Lab::attach_hold` so the live pose (not d=−2) is the certificate. `TypedFleetHold` probes illegal grants, then `grant_attached` / `attach_hold` / `attach_station` so aerial hold and hull station are one certificate (inland skips hull, open water skips rover). `TypedPadDisarm` probes pad velocity, then `Lab::attach_disarm` from Ready (no takeoff). `TypedPadFailsafe` probes pad velocity, then `Lab::attach_failsafe` from Ready (no takeoff) / `attach_recover_ready`. `TypedGroundEstop` probes parked drive, then `Lab::attach_estop` from Parked (no drive grant) / `attach_reset`. `TypedGroundHalt` probes parked drive, then `Lab::attach_drive` / `attach_park` without E-stop. `TypedGroundHold` probes parked drive, then `Lab::attach_drive` / `attach_ground_hold` so a rover NED pose hold is a research certificate (open water skips the rover). `TypedMarineHold` probes docked thrust, then `Lab::attach_undock` / `attach_marine_hold` so skiff and surveyor NED DP is a certificate (inland skips hulls). `TypedFleetReturn` probes illegal grants, then `Lab::attach_takeoff` / `attach_drive` / `attach_undock` and walks home through `attach_land` / `attach_touchdown` / `attach_park` / `attach_dock` (the same return HITL and ROS 2 plants walk). `TypedStationFailsafe` probes docked thrust, then `Lab::attach_undock` / `attach_station` / `attach_marine_failsafe` / `attach_recover`. `TypedFailsafeTouchdown` probes pad velocity, then `Lab::attach_failsafe` from Ready / `attach_touchdown` so Failsafe returns Ready without Recovery. `TypedSurveyorFailsafe` is the AUV counterpart of `TypedHullFailsafe` (`attach_undock` / `attach_marine_failsafe` / `attach_recover`). `TypedSurveyorStationFailsafe` is the AUV counterpart of `TypedStationFailsafe` (`attach_undock` / `attach_station` / `attach_marine_failsafe` / `attach_recover`). `TypedSurveyorStationDock` is the AUV counterpart of `TypedStationDock` (`attach_undock` / `attach_station` / `attach_dock`). `TypedSurveyorDock` is the AUV counterpart of `TypedHullDock` (`attach_undock` / `attach_dock` from Underway). `TypedSurveyorStationResume` is the AUV counterpart of `TypedStationResume` (`attach_undock` / `attach_station` / `attach_resume`). `ScriptedCoastal` is the demo attach policy (`Lab::apply_script`) as a certificate (`actions_applied == 0`). `Lab::session()` is the same `WorldSession` the typestate fleet uses. `Lab::aerial_vehicle` / `ground_vehicle` / `marine_vehicle` attach consume-self typestate to that live plant. `Lab::apply_script` (live demo, `coastal` example, catalog tests) walks `attach_takeoff` / `attach_drive` / `attach_undock` / `attach_land` / `attach_airborne` / `attach_touchdown` / `attach_station` and NED now-APIs, then one shared `step` — not kernel events on a borrowed body. Wind and current are fields, not comments.

## Crates

| Crate | What it is |
| --- | --- |
| `flight-core` | `no_std` units, frames, sensors, safety machines, mech, hydro. Typestate `Vehicle` / `GroundVehicle` / `MarineVehicle` require `std`. |
| `flight-sim` | Point-mass `SimBackend` (demo hover, not the property vector) plus `WorldSession` over the verified `robot-world` plant |
| `robot-world` | Multi-domain world: terrain, wind, current, conserved shallow-water field (CPU or Vulkan compute), sphere contact, battery, rigid spin. Verified `step` |
| `robot-lab` | Scenarios, property vector, agent observe/act JSON over the same `WorldSession` plant as the typestate fleet, timed-action replay, Foxglove MCAP bags |
| `flight-mavlink` | MAVLink messages for heartbeat, arm, offboard, NED velocity |
| `flight-px4` | PX4 offboard backend (`udpin:0.0.0.0:14540`) and `WorldPlant` — same MAVLink setpoints, verified world step; `hold` writes the current NED pose |
| `flight-ros2` | PX4 external modes: ROS 2 CDR `px4_msgs` setpoints (NED), NED→ENU Twist onto aerial / ground / marine `WorldSession` bodies (`FleetPlant` on coastal, harbor, inland, open water; `hold` writes the current NED pose), optional production `rclrs` 0.7 node |
| `flight-hitl` | Deadline-aware HITL rack over every catalog: coastal / harbor (four bodies), inland (no hull), open water (no rover). Verified world as plant, `FCH1` UDP samples, miss ⇒ attach failsafe (or idempotent re-trip) + zero command; on-time frames write NED only while attach is Offboard-control / Moving / Underway / StationKeep; `airborne` / `station_all` / `resume_all` / `dock_all` / `park_all` / `hold` walk climb-complete, hull station, hull dock, rover halt, and NED position hold |
| `flight-verify` | Kani proofs: actuators, drive, thrust, contact, drag, buoyancy, hydro mass, HITL miss, position-hold restore |
| `flight-demo` | Live lab console (safety trips, return, station / resume / airborne / hold) |

```
flight-core     units / frames / safety / ground / marine / mech / hydro / typestate
robot-world     environment + bodies + shallow water (CPU | Vulkan) + verified step
robot-lab       scenario + observe/act + WorldSession plant + properties
flight-sim      production | recorded | fuzzed | symbolic IMU
flight-verify   Kani / exhaustive induction
```

## Run

Requires Rust 1.85+.

```bash
cargo test --workspace
cargo run -p flight-sim --example hover
cargo run -p flight-sim --example fleet   # attach now-APIs, one WorldSession::step
cargo run -p flight-sim --example fuzzed_world  # FuzzedImu around WorldImu; plant still WorldSession::step
cargo run -p robot-lab --example coastal
cargo run -p robot-lab --example research inland
cargo run -p robot-lab --example replay inland
cargo run -p robot-lab --example bag coastal > coastal.mcap  # Foxglove: File → Open; /lab/observation has hold_ned
cargo run -p robot-lab --example probe coastal
cargo run -p robot-lab --example agent inland
cargo run -p robot-lab --example agent typed     # TypedFleet: JSON probes, then attach typestate
cargo run -p robot-lab --example agent coastal   # TypedFleet (default mixed-world agent)
cargo run -p robot-lab --example agent typed-attach # TypedAttachFleet: same attach policy
cargo run -p robot-lab --example agent pad       # PadLanding: leave terrain contact, land, touchdown
cargo run -p robot-lab --example agent typed-pad # TypedPadLanding: attach_airborne / attach_land / attach_touchdown
cargo run -p robot-lab --example agent collision # CollisionSweep: rover hits drone, spheres still separate
cargo run -p robot-lab --example agent typed-collision # TypedCollisionSweep: attach_drive / attach_park
cargo run -p robot-lab --example agent typed-station   # TypedStationDock: attach_undock / attach_station / attach_dock
cargo run -p robot-lab --example agent typed-hull-dock # TypedHullDock: attach_undock / attach_dock from Underway
cargo run -p robot-lab --example agent typed-station-resume # TypedStationResume: attach_undock / attach_station / attach_resume
cargo run -p robot-lab --example agent typed-failsafe  # TypedHullFailsafe: attach_undock / attach_marine_failsafe / attach_recover
cargo run -p robot-lab --example agent typed-aerial    # TypedAerialFailsafe: attach_takeoff / attach_failsafe / attach_recover_ready
cargo run -p robot-lab --example agent typed-aerial-disarm # TypedAerialDisarm: attach_takeoff / attach_disarm (no failsafe)
cargo run -p robot-lab --example agent typed-aerial-airborne # TypedAerialAirborne: attach_takeoff / attach_airborne / attach_land
cargo run -p robot-lab --example agent typed-position-hold # TypedPositionHold: attach_takeoff / set_position_now
cargo run -p robot-lab --example agent typed-hold # TypedHold: attach_takeoff / attach_hold (current pose)
cargo run -p robot-lab --example agent typed-fleet-hold # TypedFleetHold: grant then attach_hold / attach_station
cargo run -p robot-lab --example agent typed-pad-disarm # TypedPadDisarm: attach_disarm from Ready (no takeoff)
cargo run -p robot-lab --example agent typed-pad-failsafe # TypedPadFailsafe: attach_failsafe from Ready / attach_recover_ready
cargo run -p robot-lab --example agent typed-ground-estop # TypedGroundEstop: attach_estop from Parked / attach_reset
cargo run -p robot-lab --example agent typed-ground-halt # TypedGroundHalt: attach_drive / attach_park (no E-stop)
cargo run -p robot-lab --example agent typed-ground-hold # TypedGroundHold: attach_drive / attach_ground_hold
cargo run -p robot-lab --example agent typed-marine-hold # TypedMarineHold: attach_undock / attach_marine_hold
cargo run -p robot-lab --example agent typed-fleet-return # TypedFleetReturn: grant then land / park / dock
cargo run -p robot-lab --example agent typed-station-failsafe # TypedStationFailsafe: attach_station / attach_marine_failsafe / attach_recover
cargo run -p robot-lab --example agent typed-failsafe-touchdown # TypedFailsafeTouchdown: attach_failsafe from Ready / attach_touchdown
cargo run -p robot-lab --example agent typed-surveyor # TypedSurveyorFailsafe: AUV attach_undock / attach_marine_failsafe / attach_recover
cargo run -p robot-lab --example agent typed-surveyor-station # TypedSurveyorStationFailsafe: AUV attach_station / attach_marine_failsafe / attach_recover
cargo run -p robot-lab --example agent typed-surveyor-station-dock # TypedSurveyorStationDock: AUV attach_undock / attach_station / attach_dock
cargo run -p robot-lab --example agent typed-surveyor-dock # TypedSurveyorDock: AUV attach_undock / attach_dock from Underway
cargo run -p robot-lab --example agent typed-surveyor-station-resume # TypedSurveyorStationResume: AUV attach_undock / attach_station / attach_resume
cargo run -p robot-lab --example agent scripted        # ScriptedCoastal: demo attach policy as a property certificate
cargo run -p robot-lab --example typed           # JSON illegal acts, then attach typestate + hold_now
cargo run -p flight-demo          # http://127.0.0.1:47831 (safety, return, station / resume / airborne / hold)
FLIGHT_HYDRO_GPU=1 cargo test -p robot-world --lib gpu
cargo run -p flight-ros2 --example plant
cargo run -p flight-ros2 --example fleet_plant
```

ROS 2 Jazzy. CI job `rclrs` sources Jazzy and runs `cargo test -p flight-ros2 --features rclrs`. Locally:

```bash
source /opt/ros/jazzy/setup.bash
cargo test -p flight-ros2 --features rclrs
cargo run -p flight-ros2 --features rclrs --example offboard   # /cmd_vel as ENU Twist
```

Against PX4 SITL (SIH docker is the headless path; Gazebo is optional):

```bash
docker run --rm --network host -e PX4_SIM_MODEL=sihsim_quadx \
  px4io/px4-sitl:v1.18.0-beta2 -d
cargo run -p flight-px4 --example sitl_hover
cargo test -p flight-px4 --test sitl_live -- --ignored
# or, from a PX4 tree: make px4_sitl gz_x500
cargo run -p flight-px4 --example world_plant   # verified plant, no PX4 binary
```

Kani. CI job `kani` runs `cargo kani -p flight-verify` (**42** harnesses, `kani-verifier` 0.67.0). The `kani-verifier` crate currently needs rustc ≥ 1.88 to *install* (`home` 0.5). This repo's MSRV stays 1.85; Kani then uses its own bundled nightly:

```bash
cargo +1.88.0 install --locked --version 0.67.0 kani-verifier
cargo kani setup
cargo kani -p flight-verify   # 42 proofs: actuators, drive, thrust, contact, drag, buoyancy, hydro mass, HITL miss, position-hold restore, attach kind (air/ground/marine), land/touchdown, takeoff, halt, estop, dock, failsafe, undock, station, resume, recover, enter-offboard, disarm, mission, release
```

Creusot 0.5.0 (install from tag `v0.5.0`; uses `nightly-2025-01-31`, not workspace MSRV):

```bash
cargo creusot prove -- -p flight-core --features creusot
```

Without the installer, `cargo test` still exhaustively checks packed aerial, ground, and marine machines, plus mechanical contact over a grid of states. `cargo kani -p flight-verify` discharges the same facts as bit-precise proofs.

Creusot `#[requires]` / `#[ensures]` on the discrete machines (`step`, `ground_step`, `marine_step`, HITL deadline / apply-allowed) are discharged by `cargo creusot prove -- -p flight-core --features creusot` (Creusot **0.5.0**, `nightly-2025-01-31`; a recorded pass: **81** libraries, 0 failures). Workspace MSRV stays 1.85. `flight-verify` enables the `creusot` feature so rustc compiles the contracts as dummy macros. f32 facts (hold restore, buoyancy, hydro mass, miss-zero command) stay Kani: Creusot 0.5 pearlite cannot state them (`docs/remaining-spec.md` §3.1.4).

## Mechanical verification

Every `World::step` re-evaluates (and **refuses to commit** a successor that fails):

```text
no_terrain_penetration        z ≤ terrain; impulse only on contact
no_body_interpenetration      |p_a − p_b| ≥ r_a + r_b after sphere contact
drag_opposes_relative_flow    F · v_rel ≤ 0
buoyancy_only_when_wet        displaced volume 0 ⇒ hydrostatic force 0
aerial_actuators_require_arm  actuators_enabled ⇒ armed
aerial_thrust_only_in_air     aerial actuator force 0 unless the rotors are in air
ground_drive_requires_moving  drive_enabled ⇒ Moving ∧ ¬estop
ground_drive_only_on_contact  ground actuator force 0 unless on the terrain plane
marine_thrust_requires_grant  thrust ⇒ Underway ∨ StationKeep
marine_thrust_only_when_wet   marine actuator force 0 unless the hull is in water
finite_mechanics              mass, pose, velocity, energy finite
thrust_only_when_granted      actuator force 0 unless the domain machine granted it
relative_drag_power           F_drag · v_rel ≤ 0
battery_gates_thrust          empty energy pack ⇒ thrust 0
unit_attitude                 |q| ≈ 1; angular KE and ω finite
aerial_thrust_along_minus_body_z  quadrotor force ∥ −body z in NED
coulomb_friction_cone        sphere tangent impulse stays inside μ j_n
auv_thrust_on_body_axes      underwater force is a body-axis wrench
hydro_height_nonnegative     shallow-water column h ≥ 0
hydro_volume_conserved       no-flux Saint-Venant step conserves volume
hydro_land_stays_dry         land cells stay dry; hydro state stays finite
position_hold_restores_pose  when hold_ned is set, command · (hold − pose) ≥ 0
```

The coastal scene is virtual and specific: land `n ≥ 0` at `z = 0`, water `n < 0` with a 4 m seabed, east wind, northbound current, and a **conserved shallow-water heightfield** (Rusanov Saint-Venant). The seed sets the initial swell phase; the field then evolves. Set `FLIGHT_HYDRO_GPU=1` (the live demo does this) to run the same sweep on a Vulkan compute shader (lavapipe works). Bodies sample free-surface height and orbital flow from that field. Four platforms share it: drone (air), rover (ground), skiff (surface), surveyor (underwater). Pairwise sphere contact runs after integration; terrain is resolved again so a flattened ground plane cannot restore overlap.

`Lab::open(name, seed)` loads a catalog world. The seed is not a comment: it sets wave phase and a small deterministic gust, so two labs with the same seed replay the same field.

`Lab::open` also loads:

| Scenario | What it is |
| --- | --- |
| `coastal` | Mixed shoreline, four platforms |
| `inland` | All land, gusty wind, drone + rover |
| `harbor` | Same mix, deeper basin, chop, cross-current |
| `open_water` | No land, swell, drone + skiff + AUV |

Research traces: JSONL (`write_jsonl`, `write_actions_jsonl`) or MCAP (`write_mcap` / `McapBag`) with topics `/lab/observation` and `/lab/action` as JSON. Open the `.mcap` in Foxglove.

Agent snapshot (`GET /api/lab/observation`):

```json
{
  "t": 4.2,
  "scenario": "coastal",
  "seed": 1,
  "environment": { "wind_ned": [0, 2, 0], "current_ned": [0.35, 0, 0] },
  "robots": [{ "id": "rover", "domain": "ground", "phase": "moving", "support": "terrain", "terrain_contact": true, "sphere_contact": false, "ground": { "phase": "moving", "drive_enabled": true, "estop": false }, "n": 12.1, "charge_j": 19840 }],
  "properties": [{ "id": "no_terrain_penetration", "holds": true }]
}
```

Agent action (`POST /api/lab/action`):

```json
{"robot":"rover","cmd":"drive","vn":-0.5,"ve":0.2}
{"robot":"skiff","cmd":"undock"}
{"robot":"drone","cmd":"velocity","vn":0,"ve":1.0,"vd":0}
{"robot":"drone","cmd":"set_charge","vn":0}
{"cmd":"set_wind","ve":3.5}
{"cmd":"set_waves","vn":0.2,"ve":0.4,"vd":1.1}
```

Parked drive, docked thrust, disarmed aerial mission commands, and an empty battery are rejected by the same machines (or mechanical gates) the typestate API encodes. Successful `act` calls are timestamped; `replay_until` walks attach typestate (then JSON fallback) into a fresh lab without re-logging. The same `Lab` exposes `aerial` / `ground` / `marine` handles, so a ROS 2 ENU Twist and a JSON `drive` command step one verified plant. Observation `support` is the mechanical hold (`terrain` / `water` / `air`); `terrain_contact` is true on the pad even when the drag fluid at that cell is still air. Observation `hold_ned` is the live pose target (absent when idle, after failsafe, or after a velocity command). `sphere_contact` / `sphere_jn` / `sphere_jt` are the last pairwise sphere hit (Coulomb tangent stays inside μ j_n).

Stream a research trace, then prove replay:

```bash
cargo run -p robot-lab --example research harbor > harbor.jsonl
cargo run -p robot-lab --example replay inland
```

## What is typed

**Units and frames.** `Vector3<U, F>` is a zero-cost 3-vector. Addition requires the same `U` and `F`. NED ↔ ENU is an explicit conversion. Deg/s → rad/s is an explicit conversion. Force and torque are first-class.

**Sensors above `embedded-hal`.** An `ImuSample<Body>` carries a monotonic timestamp, body-frame accel/gyro in SI units, optional covariance, temperature, health, and a sequence number. `Clock` / `Imu` / `Actuators` are traits. Production, simulation, jsonl replay, fuzz, and a symbolic Kani clock all implement the same traits.

**Typestate vehicles.**

```text
Aerial:  Disconnected → … → Armed → Takeoff → Airborne → Landing
         Offboard / Takeoff / Airborne / Landing → Failsafe
         Failsafe ──touchdown──► Ready
         Failsafe ──disarm──► Recovery ──recover──► Ready
Ground:  Parked ──release──► Moving ──halt──► Parked
         any ──estop──► EStop
Marine:  Docked ──undock──► Underway ──station──► StationKeep
         any ──failsafe──► Failsafe ──recover──► Docked
```

## PX4 hole this fills

PX4 has moved companion-computer control toward ROS 2 external modes. The official [PX4 ROS 2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib) is C++, with incomplete Python bindings. There is no first-class Rust API.

`flight-px4` is that API over MAVLink to SITL, with the same `Vehicle<S, B>` type as the simulator. Companion `land_now` / `hold_now` send `NAV_LAND` / a position `SET_POSITION_TARGET_LOCAL_NED` at the last estimated pose. `takeoff_now` / `reached_altitude_now` stay in PX4 offboard so `Vehicle::takeoff` can climb on velocity setpoints (`MAV_CMD_DO_SET_MODE` param2 is unpacked `PX4_MAIN_MODE_OFFBOARD`; WorldPlant still maps `NAV_TAKEOFF` / `NAV_LOITER_UNLIM`). Disconnected send is `BackendError::Disconnected`. Offboard streams velocity or position `SET_POSITION_TARGET_LOCAL_NED`. `flight-ros2` is the ROS 2 companion path: CDR `px4_msgs` setpoints without a C++ library, those setpoints applied to the verified `WorldSession` plant (`FleetPlant` grants through `attach_takeoff` / `attach_drive` / `attach_undock` on every catalog — inland skips hulls, open water skips the rover — then takes one step; `FleetPlant::return_all` walks land+touchdown / park / dock back to Ready / Parked / Docked; `FleetPlant::airborne` / `station_all` / `resume_all` / `dock_all` / `park_all` walk climb-complete, hull station/resume, hull dock, and rover halt; `FleetPlant::hold` writes the drone's current NED pose), and `--features rclrs` for production `rclrs` 0.7 nodes that publish or subscribe `geometry_msgs/Twist` in ENU (`PlantNode` is the drone, `FleetPlantNode` is air + ground + surface + underwater).

## Still ahead

The v0 use / test / research / proof slice, including a recorded live PX4 SIH companion pass, is landed. Kernel vs typestate splits, catalog bodies, and MSRV stay in [`docs/remaining-spec.md`](docs/remaining-spec.md) (do not “fix” P1–P14).

The product north star is world-class **agentic robotics tooling in Rust** — experimenting, controlling, and understanding every domain and aspect:

- [`docs/agentic-spec.md`](docs/agentic-spec.md) — spec
- [`docs/NEXT.md`](docs/NEXT.md) — ordered next steps (Phase B: coordination; hold/DP, estimator trip, and typed paths landed)

A live PX4 SITL binary is optional locally (`cargo run -p flight-px4 --example sitl_hover`). Default `cargo test` skips `sitl_live` (`#[ignore]`). GitHub CI runs fmt, clippy `-D warnings`, workspace tests, `flight-core --no-default-features`, a lavapipe GPU hydro job, `cargo kani -p flight-verify` (42 harnesses, kani-verifier 0.67.0), `cargo test -p flight-ros2 --features rclrs` (ROS 2 Jazzy), `cargo creusot prove -p flight-core` (Creusot 0.5.0, 81 libraries), and job `sitl` (PX4 SIH `px4io/px4-sitl:v1.18.0-beta2` + the ignored companion test).

## License

MIT OR Apache-2.0
