# flight-core

A strongly typed Rust SDK for autonomous vehicle control. Not a C++ wrapper.

The design principle:

> Don't bind to a C++ robotics API. Create the API robotics should have had if ownership, capabilities, units, reference frames, and legal state transitions were part of the language.

```rust
let vehicle: Vehicle<Disarmed, _> = px4.connect().await?;
let vehicle: Vehicle<PreflightReady, _> = vehicle.verify_preflight().await?;
let vehicle: Vehicle<Armed, _> = vehicle.arm().await?;
let vehicle: Vehicle<Offboard, _> = vehicle.enter_offboard().await?;
vehicle.set_velocity(Velocity::<Ned>::ned(1.0, 0.0, 0.0)).await?;
```

These do not compile:

```rust
Vehicle::<Disarmed, _>::set_motor_thrust(...)   // motors require an armed typestate
Vehicle::<Disconnected, _>::arm(...)            // arm requires preflight
Position::<Ned> + Position::<Enu>               // frames are types
AngularVelocity<DegreePerSecond, Body>          // where rad/s is required
```

Rust does not automatically “verify” a robot. It lets you move physical-system correctness out of conventions and runtime checks into types, ownership, bounded memory, and then into model checkers (Kani). This repo is that stack, starting at PX4 SITL / in-process sim.

## Crates

| Crate | What it is |
| --- | --- |
| `flight-core` | `no_std`-capable units, frames, sensors, safety machine, typestate `Vehicle` |
| `flight-sim` | Deterministic clock + IMU + physics. Same controller as production. |
| `flight-mavlink` | MAVLink messages for heartbeat, arm, offboard, NED velocity |
| `flight-px4` | PX4 offboard backend (`udpin:0.0.0.0:14540` by default) |
| `flight-ros2` | External-mode trait matching the PX4 ROS 2 interface *idea*, without `rclrs` |
| `flight-verify` | Kani proofs: no path enables actuators while disarmed |
| `flight-demo` | Live mission console for the in-process vehicle |

```
flight-core
    units / frames / vehicle / state / safety / commands
flight-mavlink
flight-px4
flight-ros2
flight-sim          production | recorded | fuzzed | symbolic IMU
flight-verify       Kani / exhaustive step induction
```

## Run

Requires Rust 1.85+.

```bash
cargo test --workspace
cargo run -p flight-sim --example hover
cargo run -p flight-demo          # http://127.0.0.1:47831
```

Against PX4 SITL (optional, separate install):

```bash
make px4_sitl gz_x500             # in a PX4 tree
cargo run -p flight-px4 --example sitl_hover
```

Kani (optional):

```bash
cargo install --locked kani-verifier
cargo kani setup
cargo kani -p flight-verify
```

Without Kani, `cargo test -p flight-core` still exhaustively checks the inductive invariant over every packed safety state and event.

## What is typed

**Units and frames.** `Vector3<U, F>` is a zero-cost 3-vector. Addition requires the same `U` and `F`. NED \u2194 ENU is an explicit conversion. Deg/s \u2192 rad/s is an explicit conversion.

**Sensors above `embedded-hal`.** An `ImuSample<Body>` carries a monotonic timestamp, body-frame accel/gyro in SI units, optional covariance, temperature, health, and a sequence number. `SequenceTracker` reports dropouts. `Clock` / `Imu` / `Actuators` are traits. Production (STM32 + BMI088), simulation, jsonl replay, fuzz, and a symbolic Kani clock all implement the same traits. The controller does not know which.

**Typestate vehicle.** Legal transitions:

```text
Disconnected → Connected → Initializing → Preflight → Ready
    → Armed → Takeoff → Airborne → Landing → Ready
any state → Failsafe → Recovery / Disarm
```

Runtime invariants (enforced by `safety::step`, proven inductively):

```text
Armed              ⇒ IMU healthy ∧ estimator valid
Offboard           ⇒ command heartbeat fresh
Airborne           ⇒ actuators enabled
Failsafe           ⇒ no mission commands
Motor / velocity   ⇒ vehicle armed
actuators_enabled  ⇒ armed
```

So there is no transition sequence that reaches `ActuatorsEnabled` while `Armed == false`.

**Navigation core (start).** `flight-core::nav::ComplementaryAttitude` is `no_std`, allocation-free, panic-free, `unsafe`-free. It is not yet “the boring trusted stack everybody flies.” It is the shape of that crate: fixed-size state, covariance validation, dropout-aware samples, property tests.

## PX4 hole this fills

PX4 has moved companion-computer control toward ROS 2 external modes. The official [PX4 ROS 2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib) is C++, with incomplete Python bindings. There is no first-class Rust API.

`flight-px4` is that API, initially over MAVLink to SITL, with the same `Vehicle<S, B>` type as the simulator. `flight-ros2` is the external-mode trait we would implement on a stable `rclrs` — we do not depend on `rclrs` yet.

## Architecture

```text
production:   STM32 + BMI088 + motors
simulation:   virtual clock + simulated IMU + physics
replay:       recorded clock + jsonl IMU + fake actuators
verification: symbolic inputs + bounded clock
```

all run the same controller.

## License

MIT OR Apache-2.0
