//! Verification harness for vehicle safety and mechanical invariants.
//!
//! Exhaustive tests live in `flight-core` (`safety`, `ground`, `marine`, `mech`)
//! and run under `cargo test`. This crate holds the Kani proofs:
//!
//! ```text
//! cargo +1.88.0 install --locked --version 0.67.0 kani-verifier
//! cargo kani setup
//! cargo kani -p flight-verify
//! ```
//!
//! CI job `kani` runs that last command (45 `#[kani::proof]` harnesses).
//! Workspace MSRV stays 1.85; rustc ≥ 1.88 is the installer only.
//!
//! Theorems:
//!
//! - Arm is legal only from Ready and returns Armed
//!   (`arm_now` compiles only from PreflightReady; Armed, Offboard, Takeoff,
//!   Airborne, Landing, Failsafe, Recovery, Disarmed, and Disconnected do not)
//! - no transition enables aerial actuators while disarmed
//!   (`set_motor_thrust` compiles only while MotorsEnabled: Armed, Offboard,
//!   Takeoff, Airborne, Landing; Ready, Failsafe, Recovery, Disarmed, and
//!   Disconnected do not)
//! - ground drive_enabled ⇒ Moving ∧ ¬estop
//! - marine thrust_enabled ⇒ Underway ∨ StationKeep
//! - contact never leaves a body inside terrain; resolved contact is on the plane
//! - overlapping spheres are projected apart; impulse only when not separated
//! - Coulomb friction at a sphere stays inside μ j_n
//! - quadratic drag never adds energy along relative flow
//! - buoyancy is zero when displaced volume is zero
//! - actuator force is zero unless the domain machine granted it
//! - ground drive force is zero unless the hull is on the terrain plane
//! - marine thrust is zero unless the hull is in water
//! - aerial thrust is zero unless the rotors are in air
//! - empty battery ⇒ actuator force is zero
//! - torque-free rigid spin keeps a unit quaternion and finite angular KE
//! - body→NED rotation preserves vector length; quadrotor thrust lies on −body z
//! - two-cell periodic shallow water conserves mass and keeps h ≥ 0
//! - a HITL deadline miss applies the zero command, never the late setpoint
//! - a permit issued at epoch N is stale at epoch M ≠ N (`permit_epoch_mismatch_is_stale`)
//! - the kernel revoke table matches `event_revokes_authority` (`dsl_revokes_match_kernel`)
//! - attach maps Ready → PreflightReady, Takeoff → Takeoff, failsafe → Failsafe, Recovery → Recovery
//! - attach maps Parked → Parked, Moving → Moving, E-stop → EStopped
//! - attach maps Docked → Docked, Underway → Underway, StationKeep → StationKeep, failsafe → Failsafe
//! - Land is legal only from Takeoff or Airborne; Touchdown returns Ready, unarmed
//!   (`CanBeginLand` is Takeoff and Airborne; Armed, Ready, Offboard, Landing,
//!   Disconnected, Failsafe, and Recovery do not compile `begin_land_now`.
//!   `CanTouchdown` is Landing and Failsafe; Armed, Offboard, Takeoff, Airborne,
//!   Ready, Disarmed, Disconnected, and Recovery do not compile `touchdown_now`)
//! - Halt is legal only from Moving and returns Parked with drive cleared
//!   (Parked and E-stop do not compile `park_now`)
//! - E-stop from any ground machine returns EStop with drive cleared
//!   (`CanTripEstop` is Parked and Moving)
//! - ClearEstop is legal only from EStop and returns Parked
//!   (`reset` compiles only from EStopped; Parked and Moving do not)
//! - Dock from any marine machine returns Docked with thrust and failsafe cleared
//!   (`CanDock` is Underway and StationKeep; Docked and Failsafe do not compile)
//! - Failsafe from any marine machine returns Failsafe with thrust cleared
//!   (`CanTripMarineFailsafe` is Underway and StationKeep; Docked does not compile)
//! - Recover is legal only from Failsafe and returns Docked
//!   (`recover_docked` compiles only from Failsafe; Docked, Underway, and
//!   StationKeep do not)
//! - Undock is legal only from Docked and returns Underway with thrust
//!   (Underway, StationKeep, and Failsafe do not compile `undock`)
//! - Station is legal only from Underway and returns StationKeep
//!   (Docked, StationKeep, and Failsafe do not compile `hold_station`)
//! - Resume is legal only from StationKeep and returns Underway
//!   (Docked, Underway, and Failsafe do not compile `resume`)
//! - Takeoff is legal only from Armed; ReachedAltitude only from Takeoff
//!   (`start_takeoff_now` compiles only from Offboard; Ready, Armed, Takeoff,
//!   Airborne, Landing, Failsafe, Recovery, Disarmed, and Disconnected do not)
//! - ReachedAltitude / `declare_airborne_now` compiles only from Takeoff
//!   (Offboard, Airborne, Landing, Ready, Armed, Failsafe, Recovery, Disarmed,
//!   and Disconnected do not)
//! - TriggerFailsafe from any aerial machine returns Failsafe
//!   (`CanTripFailsafe` is Ready, Armed, Offboard, Takeoff, Airborne, Landing;
//!   Disarmed, Failsafe, Recovery, and Disconnected do not compile `failsafe_now`)
//! - Recover is legal only from Recovery (disarmed) and returns Ready
//!   (`recover_now` compiles only from Recovery; Ready, Armed, Offboard,
//!   Takeoff, Airborne, Landing, Failsafe, Disarmed, and Disconnected do not)
//! - EnterOffboard is legal only from Armed / Takeoff / Airborne / Landing
//!   (`enter_offboard_now` compiles only from Armed; Ready, Offboard, Takeoff,
//!   Airborne, Landing, Failsafe, Recovery, Disarmed, and Disconnected do not)
//! - Disarm from any connected machine unarms; failsafe → Recovery, else Ready
//!   (`CanDisarm` is Ready, Armed, Offboard, Takeoff, Airborne, Landing)
//! - MissionCommand requires armed actuators and a fresh offboard heartbeat
//!   (`set_velocity` / `set_position` / `hold` compile only under OffboardControl:
//!   Offboard, Takeoff, Airborne, Landing; Ready, Armed, Failsafe, Recovery,
//!   Disarmed, and Disconnected do not)
//! - Release is legal only from Parked and returns Moving with drive enabled
//!   (Moving and E-stop do not compile `enable_drive`)
//! - DriveCommand is legal only from Moving with drive enabled
//!   (Parked and E-stop do not compile `set_twist`)
//! - ThrustCommand is legal only when thrust is granted and not in failsafe
//!   (`CanThrust` is Underway and StationKeep; Docked does not compile)
//! - position hold P-term restores pose: command · (hold − pose) ≥ 0
//!   (`hold_velocity_ned` / `hold_restores_pose`; plant `refresh_hold` uses `HOLD_KP`)
//!
//! The same facts are Creusot `#[requires]` / `#[ensures]` on the kernel
//! (`flight-core` feature `creusot`, enabled here). Dummy macros on rustc.
//! `cargo creusot prove -- -p flight-core --features creusot` (Creusot 0.5.0)
//! discharges aerial `step`, `ground_step`, `marine_step`, and HITL deadline
//! / apply-allowed (recorded: 81 libraries, 0 failures). f32 facts stay Kani.

#![deny(unsafe_code)]
#![allow(unexpected_cfgs)]

use flight_core::safety::{check_invariants, step, Event, Reject, SafetyState};

/// Inductive step: any invariant-satisfying state, after a successful `step`,
/// still satisfies the invariants, and never enables actuators while disarmed.
pub fn inductive_step(s: SafetyState, e: Event) -> Result<SafetyState, Reject> {
    if !check_invariants(&s) {
        return Err(Reject::IllegalPhase);
    }
    let n = step(s, e)?;
    debug_assert!(check_invariants(&n));
    debug_assert!(!n.actuators_enabled || n.armed);
    Ok(n)
}

#[cfg(kani)]
mod proofs {
    use super::*;
    use flight_core::ground::{
        ground_invariants, ground_step, unpack_ground, GroundEvent, GroundPhase,
    };
    use flight_core::marine::{
        marine_invariants, marine_step, unpack_marine, MarineEvent, MarinePhase,
    };
    use flight_core::mech::{
        buoyancy_ned, buoyancy_only_when_wet, contact_invariants, drag_opposes_flow,
        quadratic_drag, resolve_sphere_contact, resolve_vertical_contact,
        sphere_contact_invariants, SphereBody, SphereContact, VerticalContact,
    };
    use flight_core::safety::{unpack, Event, Phase};

    #[kani::proof]
    fn actuators_require_arm() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        let ev: u8 = kani::any();
        kani::assume(ev <= 23);
        let Some(e) = Event::from_u8(ev) else { return };
        if let Ok(n) = step(s, e) {
            assert!(check_invariants(&n));
            assert!(!n.actuators_enabled || n.armed);
            if n.phase == Phase::Takeoff || n.phase == Phase::Airborne {
                assert!(n.actuators_enabled);
                assert!(n.armed);
            }
        }
    }

    #[kani::proof]
    fn failsafe_blocks_mission_commands() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        kani::assume(s.failsafe);
        assert!(step(s, Event::MissionCommand).is_err());
        assert!(step(s, Event::Arm).is_err());
        assert!(step(s, Event::Takeoff).is_err());
        assert!(step(s, Event::Land).is_err());
        assert!(step(s, Event::EnterOffboard).is_err());
    }

    #[kani::proof]
    fn arm_requires_sensors() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(mut s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        s.phase = Phase::Ready;
        s.armed = false;
        s.actuators_enabled = false;
        s.offboard = false;
        s.failsafe = false;
        kani::assume(check_invariants(&s));
        kani::assume(!s.imu_healthy || !s.estimator_valid);
        assert!(step(s, Event::Arm).is_err());
    }

    #[kani::proof]
    fn ground_drive_requires_moving() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        let ev: u8 = kani::any();
        kani::assume(ev <= 4);
        let Some(e) = GroundEvent::from_u8(ev) else {
            return;
        };
        if let Ok(n) = ground_step(s, e) {
            assert!(ground_invariants(&n));
            assert!(!n.drive_enabled || n.phase == GroundPhase::Moving);
            assert!(!n.drive_enabled || !n.estop);
        }
    }

    #[kani::proof]
    fn marine_thrust_requires_grant() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        let ev: u8 = kani::any();
        kani::assume(ev <= 6);
        let Some(e) = MarineEvent::from_u8(ev) else {
            return;
        };
        if let Ok(n) = marine_step(s, e) {
            assert!(marine_invariants(&n));
            assert!(
                !n.thrust_enabled
                    || matches!(n.phase, MarinePhase::Underway | MarinePhase::StationKeep)
            );
        }
    }

    #[kani::proof]
    fn contact_never_penetrates() {
        let zb: u8 = kani::any();
        let vb: u8 = kani::any();
        kani::assume(zb < 9);
        kani::assume(vb < 9);
        let z = zb as f32 * 0.5 - 2.0;
        let vz = vb as f32 * 0.5 - 2.0;
        let before = VerticalContact {
            z,
            vz,
            terrain_z: 0.0,
            impulse: 0.0,
        };
        let after = resolve_vertical_contact(before);
        assert!(contact_invariants(before, after));
        assert!(after.z <= after.terrain_z + 1e-6);
        if before.z >= before.terrain_z {
            assert!(after.on_plane());
        } else {
            assert!(!after.on_plane());
        }
    }

    #[kani::proof]
    fn spheres_never_interpenetrate() {
        let da: u8 = kani::any();
        let va: u8 = kani::any();
        let vb: u8 = kani::any();
        kani::assume(da < 8);
        kani::assume(va < 6);
        kani::assume(vb < 6);
        let d = da as f32 * 0.25;
        let before = SphereContact::pair(
            SphereBody::new(
                [0.0, 0.0, 0.0],
                [va as f32 * 0.5 - 1.25, 0.0, 0.0],
                0.4,
                1.0,
            ),
            SphereBody::new([d, 0.0, 0.0], [vb as f32 * 0.5 - 1.25, 0.0, 0.0], 0.4, 2.0),
        );
        let after = resolve_sphere_contact(before);
        assert!(sphere_contact_invariants(before, after));
        assert!(after.gap() >= -1e-4);
        assert!(after.impulse >= 0.0);
    }

    #[kani::proof]
    fn drag_never_adds_energy() {
        let s: u8 = kani::any();
        kani::assume(s < 6);
        let v = match s {
            0 => [1.0, 0.0, 0.0],
            1 => [-0.5, 0.4, 0.1],
            2 => [0.0, 0.0, 0.0],
            3 => [2.0, -1.0, 0.3],
            4 => [0.0, 0.0, -1.5],
            _ => [-3.0, 0.2, 0.0],
        };
        let f = quadratic_drag(v, 1.225, 0.8, 0.4);
        assert!(drag_opposes_flow(v, f));
        let fw = quadratic_drag(v, 1025.0, 0.6, 0.3);
        assert!(drag_opposes_flow(v, fw));
    }

    #[kani::proof]
    fn buoyancy_zero_when_dry() {
        let f = buoyancy_ned(0.0, 1025.0, 9.81);
        assert!(buoyancy_only_when_wet(0.0, f));
        let wet = buoyancy_ned(0.04, 1025.0, 9.81);
        assert!(wet < 0.0);
        assert!(buoyancy_only_when_wet(0.04, wet));
    }

    #[kani::proof]
    fn thrust_zero_unless_granted() {
        let granted: bool = kani::any();
        let mag: u8 = kani::any();
        kani::assume(mag < 8);
        let t = [mag as f32, 0.0, 0.0];
        let ok = flight_core::mech::thrust_only_when_granted(granted, t);
        if granted {
            assert!(ok);
        } else {
            assert_eq!(ok, mag == 0);
        }
    }

    #[kani::proof]
    fn ground_thrust_zero_off_contact() {
        let on_terrain: bool = kani::any();
        let mag: u8 = kani::any();
        kani::assume(mag < 8);
        let t = [mag as f32, 0.0, 0.0];
        let ok = flight_core::mech::ground_thrust_only_on_contact(on_terrain, t);
        if on_terrain {
            assert!(ok);
        } else {
            assert_eq!(ok, mag == 0);
        }
    }

    #[kani::proof]
    fn marine_thrust_zero_when_dry() {
        let wet: bool = kani::any();
        let mag: u8 = kani::any();
        kani::assume(mag < 8);
        let t = [mag as f32, 0.0, 0.0];
        let ok = flight_core::mech::marine_thrust_only_when_wet(wet, t);
        if wet {
            assert!(ok);
        } else {
            assert_eq!(ok, mag == 0);
        }
    }

    #[kani::proof]
    fn aerial_thrust_zero_when_wet() {
        let in_air: bool = kani::any();
        let mag: u8 = kani::any();
        kani::assume(mag < 8);
        let t = [0.0, 0.0, -(mag as f32)];
        let ok = flight_core::mech::aerial_thrust_only_in_air(in_air, t);
        if in_air {
            assert!(ok);
        } else {
            assert_eq!(ok, mag == 0);
        }
    }

    #[kani::proof]
    fn empty_battery_zero_thrust() {
        let charge_bit: bool = kani::any();
        let mag: u8 = kani::any();
        kani::assume(mag < 8);
        let charge = if charge_bit { 4.0 } else { 0.0 };
        let t = [mag as f32, 0.0, 0.0];
        let ok = flight_core::mech::battery_gates_thrust(charge, t);
        if charge_bit {
            assert!(ok);
        } else {
            assert_eq!(ok, mag == 0);
        }
        let drained = flight_core::mech::drain_from_thrust(0.2, t, 1.0, 1.0);
        assert!(drained >= 0.0);
    }

    #[kani::proof]
    fn rigid_spin_stays_unit() {
        let wx: u8 = kani::any();
        kani::assume(wx < 5);
        let w = [wx as f32 * 0.3 - 0.6, 0.1, -0.2];
        let i = [1.0, 2.0, 3.0];
        let w1 = flight_core::mech::euler_principal_step(w, [0.0, 0.0, 0.0], i, 0.01);
        let q1 = flight_core::mech::quat_integrate([1.0, 0.0, 0.0, 0.0], w1, 0.01);
        assert!(flight_core::mech::rigid_spin_invariants(i, w, w1, q1));
        assert!(flight_core::mech::quat_is_unit(q1));
        assert!(flight_core::mech::angular_kinetic_energy(i, w1) >= 0.0);
    }

    #[kani::proof]
    fn rotate_preserves_length() {
        let k: u8 = kani::any();
        kani::assume(k < 5);
        let v = match k {
            0 => [1.0, 0.0, 0.0],
            1 => [0.0, 1.0, 0.0],
            2 => [0.0, 0.0, 2.0],
            3 => [0.3, -0.4, 0.5],
            _ => [0.0, 0.0, 0.0],
        };
        let q = flight_core::mech::quat_integrate([1.0, 0.0, 0.0, 0.0], [0.1, -0.2, 0.3], 0.4);
        let r = flight_core::mech::quat_rotate(q, v);
        assert!(flight_core::mech::rotation_preserves_length(v, r));
        let t = flight_core::mech::body_z_thrust_ned(q, k as f32);
        assert!(flight_core::mech::thrust_along_minus_body_z(q, t));
    }

    #[kani::proof]
    fn sphere_friction_stays_in_cone() {
        let k: u8 = kani::any();
        kani::assume(k < 4);
        let vx = k as f32 * 0.4 - 0.4;
        let mu = 0.4;
        let after = resolve_sphere_contact(SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [1.0, vx, 0.0], 0.5, 1.0),
            SphereBody::new([0.35, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5, 1.0),
        ));
        let f = flight_core::mech::apply_sphere_friction(
            after,
            flight_core::mech::SphereSpin::new([0.0, 0.0, 0.0], 0.1),
            flight_core::mech::SphereSpin::new([0.0, 0.0, 0.0], 0.1),
            mu,
        );
        assert!(flight_core::mech::friction_invariants(mu, after.impulse, f));
    }

    #[kani::proof]
    fn shallow_water_two_cell_conserves_mass() {
        let a: u8 = kani::any();
        let b: u8 = kani::any();
        kani::assume(a < 5 && b < 5);
        let h = [0.8 + a as f32 * 0.3, 0.8 + b as f32 * 0.3];
        let u = [0.2, -0.1];
        let h1 = flight_core::hydro::two_cell_periodic_mass(h, u, 0.01, 1.0, 9.81);
        assert!(h1[0] >= 0.0 && h1[1] >= 0.0);
        let d = (h[0] + h[1]) - (h1[0] + h1[1]);
        assert!(d < 1e-4 && d > -1e-4);
    }

    #[kani::proof]
    fn hitl_miss_applies_zero() {
        let missed: bool = kani::any();
        let k: u8 = kani::any();
        kani::assume(k < 4);
        let next = [k as f32 * 0.5, -0.2, 0.1];
        let applied = flight_core::hitl::command_after_deadline(missed, next);
        assert!(flight_core::hitl::hitl_invariants(missed, applied, next));
        if missed {
            assert_eq!(applied, [0.0, 0.0, 0.0]);
            assert!(!flight_core::hitl::hitl_apply_allowed(true));
        }
    }

    #[kani::proof]
    fn hold_velocity_restores_pose() {
        let kp: u8 = kani::any();
        kani::assume(kp < 4);
        let en: i8 = kani::any();
        let ee: i8 = kani::any();
        let ed: i8 = kani::any();
        kani::assume(en >= -2 && en <= 2);
        kani::assume(ee >= -2 && ee <= 2);
        kani::assume(ed >= -2 && ed <= 2);
        let hold = [0.0f32, 0.0, 0.0];
        let position = [-(en as f32), -(ee as f32), -(ed as f32)];
        let cmd = flight_core::mech::hold_velocity_ned(hold, position, kp as f32);
        assert!(flight_core::mech::hold_restores_pose(hold, position, cmd));
        if kp == 0 {
            assert_eq!(cmd, [0.0, 0.0, 0.0]);
        }
        assert!(cmd[0] * (en as f32) >= -1e-5);
        assert!(cmd[1] * (ee as f32) >= -1e-5);
        assert!(cmd[2] * (ed as f32) >= -1e-5);
    }

    #[kani::proof]
    fn attach_kind_maps_the_live_aerial_machine() {
        use flight_core::vehicle::{aerial_kind, AerialKind};
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        let k = aerial_kind(s);
        if s.phase == Phase::Recovery {
            assert!(k == AerialKind::Recovery);
            return;
        }
        if s.failsafe {
            assert!(k == AerialKind::Failsafe);
            return;
        }
        match s.phase {
            Phase::Disconnected => assert!(k == AerialKind::Disconnected),
            Phase::Connected | Phase::Initializing | Phase::Preflight => {
                assert!(k == AerialKind::Disarmed)
            }
            Phase::Ready => assert!(k == AerialKind::PreflightReady),
            Phase::Armed if s.offboard => assert!(k == AerialKind::Offboard),
            Phase::Armed => assert!(k == AerialKind::Armed),
            Phase::Takeoff => assert!(k == AerialKind::Takeoff),
            Phase::Airborne => assert!(k == AerialKind::Airborne),
            Phase::Landing => assert!(k == AerialKind::Landing),
            Phase::Failsafe => assert!(k == AerialKind::Failsafe),
            Phase::Recovery => assert!(k == AerialKind::Recovery),
        }
    }

    #[kani::proof]
    fn attach_kind_maps_the_live_ground_machine() {
        use flight_core::vehicle::{ground_kind, GroundKind};
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        let k = ground_kind(s);
        if s.estop {
            assert!(k == GroundKind::EStopped);
            return;
        }
        match s.phase {
            GroundPhase::Parked => assert!(k == GroundKind::Parked),
            GroundPhase::Moving => assert!(k == GroundKind::Moving),
            GroundPhase::EStop => assert!(k == GroundKind::EStopped),
        }
    }

    #[kani::proof]
    fn attach_kind_maps_the_live_marine_machine() {
        use flight_core::vehicle::{marine_kind, MarineKind};
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        let k = marine_kind(s);
        if s.failsafe {
            assert!(k == MarineKind::Failsafe);
            return;
        }
        match s.phase {
            MarinePhase::Docked => assert!(k == MarineKind::Docked),
            MarinePhase::Underway => assert!(k == MarineKind::Underway),
            MarinePhase::StationKeep => assert!(k == MarineKind::StationKeep),
            MarinePhase::Failsafe => assert!(k == MarineKind::Failsafe),
        }
    }

    #[kani::proof]
    fn land_only_from_flight_touchdown_returns_ready() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::Land) {
            Ok(n) => {
                assert!(!s.failsafe);
                assert!(s.phase == Phase::Takeoff || s.phase == Phase::Airborne);
                assert!(n.phase == Phase::Landing);
                assert!(n.armed);
                assert!(n.actuators_enabled);
                assert!(check_invariants(&n));
            }
            Err(_) => {
                assert!(s.failsafe || (s.phase != Phase::Takeoff && s.phase != Phase::Airborne));
            }
        }
        match step(s, Event::Touchdown) {
            Ok(n) => {
                assert!(s.phase == Phase::Landing || s.phase == Phase::Failsafe);
                assert!(n.phase == Phase::Ready);
                assert!(!n.armed);
                assert!(!n.actuators_enabled);
                assert!(!n.offboard);
                assert!(!n.failsafe);
                assert!(check_invariants(&n));
            }
            Err(_) => {
                assert!(s.phase != Phase::Landing && s.phase != Phase::Failsafe);
            }
        }
    }

    #[kani::proof]
    fn halt_only_from_moving_returns_parked() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        match ground_step(s, GroundEvent::Halt) {
            Ok(n) => {
                assert!(s.phase == GroundPhase::Moving);
                assert!(n.phase == GroundPhase::Parked);
                assert!(!n.drive_enabled);
                assert!(!n.estop);
                assert!(ground_invariants(&n));
            }
            Err(_) => assert!(s.phase != GroundPhase::Moving),
        }
    }

    #[kani::proof]
    fn dock_always_returns_docked() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        let n = marine_step(s, MarineEvent::Dock).unwrap();
        assert!(n.phase == MarinePhase::Docked);
        assert!(!n.thrust_enabled);
        assert!(!n.failsafe);
        assert!(marine_invariants(&n));
    }

    #[kani::proof]
    fn takeoff_only_from_armed_reached_altitude_only_from_takeoff() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::Takeoff) {
            Ok(n) => {
                assert!(!s.failsafe);
                assert!(s.phase == Phase::Armed);
                assert!(s.armed);
                assert!(n.phase == Phase::Takeoff);
                assert!(n.armed);
                assert!(n.actuators_enabled);
                assert!(check_invariants(&n));
            }
            Err(_) => {
                assert!(s.failsafe || s.phase != Phase::Armed || !s.armed);
            }
        }
        match step(s, Event::ReachedAltitude) {
            Ok(n) => {
                assert!(s.phase == Phase::Takeoff);
                assert!(n.phase == Phase::Airborne);
                assert!(check_invariants(&n));
            }
            Err(_) => assert!(s.phase != Phase::Takeoff),
        }
    }

    #[kani::proof]
    fn estop_always_returns_estopped() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        let n = ground_step(s, GroundEvent::EStop).unwrap();
        assert!(n.phase == GroundPhase::EStop);
        assert!(n.estop);
        assert!(!n.drive_enabled);
        assert!(ground_invariants(&n));
    }

    #[kani::proof]
    fn clear_estop_only_from_estop_returns_parked() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        match ground_step(s, GroundEvent::ClearEstop) {
            Ok(n) => {
                assert!(s.phase == GroundPhase::EStop);
                assert!(n.phase == GroundPhase::Parked);
                assert!(!n.estop);
                assert!(!n.drive_enabled);
                assert!(ground_invariants(&n));
            }
            Err(_) => assert!(s.phase != GroundPhase::EStop),
        }
    }

    #[kani::proof]
    fn failsafe_always_returns_failsafe() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        let n = marine_step(s, MarineEvent::Failsafe).unwrap();
        assert!(n.phase == MarinePhase::Failsafe);
        assert!(n.failsafe);
        assert!(!n.thrust_enabled);
        assert!(marine_invariants(&n));
    }

    #[kani::proof]
    fn recover_only_from_failsafe_returns_docked() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        match marine_step(s, MarineEvent::Recover) {
            Ok(n) => {
                assert!(s.phase == MarinePhase::Failsafe);
                assert!(n.phase == MarinePhase::Docked);
                assert!(!n.failsafe);
                assert!(!n.thrust_enabled);
                assert!(marine_invariants(&n));
            }
            Err(_) => assert!(s.phase != MarinePhase::Failsafe),
        }
    }

    #[kani::proof]
    fn undock_only_from_docked_returns_underway() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        match marine_step(s, MarineEvent::Undock) {
            Ok(n) => {
                assert!(s.phase == MarinePhase::Docked);
                assert!(!s.failsafe);
                assert!(n.phase == MarinePhase::Underway);
                assert!(n.thrust_enabled);
                assert!(marine_invariants(&n));
            }
            Err(_) => assert!(s.failsafe || s.phase != MarinePhase::Docked),
        }
    }

    #[kani::proof]
    fn station_only_from_underway_returns_station_keep() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        match marine_step(s, MarineEvent::Station) {
            Ok(n) => {
                assert!(s.phase == MarinePhase::Underway);
                assert!(!s.failsafe);
                assert!(n.phase == MarinePhase::StationKeep);
                assert!(n.thrust_enabled);
                assert!(marine_invariants(&n));
            }
            Err(_) => assert!(s.failsafe || s.phase != MarinePhase::Underway),
        }
    }

    #[kani::proof]
    fn resume_only_from_station_keep_returns_underway() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        match marine_step(s, MarineEvent::Resume) {
            Ok(n) => {
                assert!(s.phase == MarinePhase::StationKeep);
                assert!(!s.failsafe);
                assert!(n.phase == MarinePhase::Underway);
                assert!(n.thrust_enabled);
                assert!(marine_invariants(&n));
            }
            Err(_) => assert!(s.failsafe || s.phase != MarinePhase::StationKeep),
        }
    }

    #[kani::proof]
    fn aerial_failsafe_always_returns_failsafe() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        let n = step(s, Event::TriggerFailsafe).unwrap();
        assert!(n.phase == Phase::Failsafe);
        assert!(n.failsafe);
        assert!(!n.offboard);
        assert!(check_invariants(&n));
    }

    #[kani::proof]
    fn recover_only_from_recovery_returns_ready() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::Recover) {
            Ok(n) => {
                assert!(s.phase == Phase::Recovery);
                assert!(!s.armed);
                assert!(n.phase == Phase::Ready);
                assert!(!n.failsafe);
                assert!(check_invariants(&n));
            }
            Err(_) => assert!(s.phase != Phase::Recovery || s.armed),
        }
    }

    #[kani::proof]
    fn enter_offboard_only_from_flight_phases() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::EnterOffboard) {
            Ok(n) => {
                assert!(!s.failsafe);
                assert!(
                    s.phase == Phase::Armed
                        || s.phase == Phase::Takeoff
                        || s.phase == Phase::Airborne
                        || s.phase == Phase::Landing
                );
                assert!(s.armed);
                assert!(s.offboard_heartbeat_fresh);
                assert!(n.offboard);
                assert!(n.phase == s.phase);
                assert!(check_invariants(&n));
            }
            Err(_) => {
                assert!(
                    s.failsafe
                        || (s.phase != Phase::Armed
                            && s.phase != Phase::Takeoff
                            && s.phase != Phase::Airborne
                            && s.phase != Phase::Landing)
                        || !s.armed
                        || !s.offboard_heartbeat_fresh
                );
            }
        }
    }

    #[kani::proof]
    fn disarm_unarms_and_returns_ready_or_recovery() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::Disarm) {
            Ok(n) => {
                assert!(s.phase != Phase::Disconnected);
                assert!(!n.armed);
                assert!(!n.actuators_enabled);
                assert!(!n.offboard);
                if s.failsafe {
                    assert!(n.phase == Phase::Recovery);
                } else {
                    assert!(n.phase == Phase::Ready);
                    assert!(!n.failsafe);
                }
                assert!(check_invariants(&n));
            }
            Err(_) => assert!(s.phase == Phase::Disconnected),
        }
    }

    #[kani::proof]
    fn mission_command_requires_armed_actuators() {
        let bits: u16 = kani::any();
        kani::assume(bits <= 0x07FF);
        let Some(s) = unpack(bits) else { return };
        kani::assume(check_invariants(&s));
        match step(s, Event::MissionCommand) {
            Ok(n) => {
                assert!(!s.failsafe);
                assert!(s.armed);
                assert!(s.actuators_enabled);
                assert!(!s.offboard || s.offboard_heartbeat_fresh);
                assert!(n.phase == s.phase);
                assert!(n.armed == s.armed);
                assert!(check_invariants(&n));
            }
            Err(_) => {
                assert!(
                    s.failsafe
                        || !s.armed
                        || !s.actuators_enabled
                        || (s.offboard && !s.offboard_heartbeat_fresh)
                );
            }
        }
    }

    #[kani::proof]
    fn release_only_from_parked_returns_moving() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        match ground_step(s, GroundEvent::Release) {
            Ok(n) => {
                assert!(s.phase == GroundPhase::Parked);
                assert!(n.phase == GroundPhase::Moving);
                assert!(n.drive_enabled);
                assert!(!n.estop);
                assert!(ground_invariants(&n));
            }
            Err(_) => assert!(s.phase != GroundPhase::Parked),
        }
    }

    #[kani::proof]
    fn drive_command_only_when_moving_and_enabled() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_ground(bits) else { return };
        kani::assume(ground_invariants(&s));
        match ground_step(s, GroundEvent::DriveCommand) {
            Ok(n) => {
                assert!(!s.estop);
                assert!(s.phase == GroundPhase::Moving);
                assert!(s.drive_enabled);
                assert!(n.phase == s.phase);
                assert!(n.drive_enabled);
                assert!(ground_invariants(&n));
            }
            Err(_) => assert!(s.estop || s.phase != GroundPhase::Moving || !s.drive_enabled),
        }
    }

    #[kani::proof]
    fn thrust_command_only_when_granted() {
        let bits: u8 = kani::any();
        kani::assume(bits <= 0x0F);
        let Some(s) = unpack_marine(bits) else { return };
        kani::assume(marine_invariants(&s));
        match marine_step(s, MarineEvent::ThrustCommand) {
            Ok(n) => {
                assert!(!s.failsafe);
                assert!(s.thrust_enabled);
                assert!(n.thrust_enabled);
                assert!(marine_invariants(&n));
            }
            Err(_) => assert!(s.failsafe || !s.thrust_enabled),
        }
    }

    #[kani::proof]
    fn permit_epoch_mismatch_is_stale() {
        use flight_core::contracts::{ActuationPermit, SafetyEpoch, VehicleId};
        use flight_core::time::MonotonicInstant;
        let live: u32 = kani::any();
        let issued: u32 = kani::any();
        kani::assume(live != issued);
        let p = ActuationPermit::unbounded(
            VehicleId::from_raw(1),
            SafetyEpoch(issued),
            MonotonicInstant::ZERO,
        );
        assert!(p
            .check(
                SafetyEpoch(live),
                VehicleId::from_raw(1),
                MonotonicInstant::ZERO
            )
            .is_err());
    }

    #[kani::proof]
    fn dsl_revokes_match_kernel() {
        use flight_core::contracts::AerialOffboard;
        use flight_core::safety::{
            command_age_ok, estimator_ts_monotonic, event_revokes_authority, heartbeat_age_ok,
            Event, AUTHORITY_REVOKE_EVENTS, COMMAND_MAX_AGE_MS, OFFBOARD_HEARTBEAT_MAX_AGE_MS,
        };
        let bits: u8 = kani::any();
        kani::assume(bits <= 23);
        let Some(e) = Event::from_u8(bits) else {
            return;
        };
        let mut in_table = false;
        let mut i = 0;
        while i < AUTHORITY_REVOKE_EVENTS.len() {
            if AUTHORITY_REVOKE_EVENTS[i] == e {
                in_table = true;
                break;
            }
            i += 1;
        }
        assert_eq!(event_revokes_authority(e), in_table);
        assert_eq!(AerialOffboard::revokes(e), event_revokes_authority(e));
        let age: u32 = kani::any();
        assert_eq!(heartbeat_age_ok(age), age < OFFBOARD_HEARTBEAT_MAX_AGE_MS);
        assert_eq!(command_age_ok(age), age < COMMAND_MAX_AGE_MS);
        let prev: u64 = kani::any();
        let next: u64 = kani::any();
        assert_eq!(estimator_ts_monotonic(prev, next), next >= prev);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::ground::{ground_invariants, ground_step, unpack_ground, GroundEvent};
    use flight_core::marine::{marine_invariants, marine_step, unpack_marine, MarineEvent};
    use flight_core::safety::{pack, unpack, Event};

    #[test]
    fn kani_wrapper_agrees_with_step() {
        let s = SafetyState::disconnected();
        assert!(inductive_step(s, Event::Connect).is_ok());
        assert!(inductive_step(s, Event::Arm).is_err());
    }

    #[test]
    fn packed_roundtrip() {
        for bits in 0u16..=0x07FF {
            if let Some(s) = unpack(bits) {
                assert_eq!(pack(&s), bits);
            }
        }
    }

    #[test]
    fn thrust_grant_matches_kernel() {
        use flight_core::mech::thrust_only_when_granted;
        assert!(thrust_only_when_granted(false, [0.0, 0.0, 0.0]));
        assert!(!thrust_only_when_granted(false, [0.4, 0.0, 0.0]));
        assert!(thrust_only_when_granted(true, [0.4, 0.0, 0.0]));
    }

    #[test]
    fn hold_kernel_restores_pose() {
        use flight_core::mech::{hold_restores_pose, hold_velocity_ned, HOLD_KP};
        let hold = [0.0, 1.0, -2.0];
        let pos = [0.5, 1.0, 0.0];
        let cmd = hold_velocity_ned(hold, pos, HOLD_KP);
        assert!(hold_restores_pose(hold, pos, cmd));
        assert!(cmd[0] < 0.0);
        assert_eq!(cmd[1], 0.0);
        assert!(cmd[2] < 0.0);
    }

    #[test]
    fn battery_gate_matches_kernel() {
        use flight_core::mech::{battery_gates_thrust, drain_from_thrust};
        assert!(battery_gates_thrust(0.0, [0.0, 0.0, 0.0]));
        assert!(!battery_gates_thrust(0.0, [1.0, 0.0, 0.0]));
        assert_eq!(drain_from_thrust(0.0, [9.0, 0.0, 0.0], 0.1, 1.0), 0.0);
    }

    #[test]
    fn sphere_kernel_separates_overlap() {
        use flight_core::mech::{
            resolve_sphere_contact, sphere_contact_invariants, SphereBody, SphereContact,
        };
        let before = SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.5, 1.0),
            SphereBody::new([0.1, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.5, 1.0),
        );
        let after = resolve_sphere_contact(before);
        assert!(sphere_contact_invariants(before, after));
        assert!(after.gap() >= -1e-4);
        assert!(after.impulse > 0.0);
    }

    #[test]
    fn friction_kernel_stays_in_cone() {
        use flight_core::mech::{
            apply_sphere_friction, friction_invariants, resolve_sphere_contact, SphereBody,
            SphereContact, SphereSpin, SPHERE_FRICTION_MU,
        };
        let after = resolve_sphere_contact(SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [1.0, 0.6, 0.0], 0.5, 1.0),
            SphereBody::new([0.3, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5, 1.0),
        ));
        let f = apply_sphere_friction(
            after,
            SphereSpin::new([0.0; 3], 0.1),
            SphereSpin::new([0.0; 3], 0.1),
            SPHERE_FRICTION_MU,
        );
        assert!(friction_invariants(SPHERE_FRICTION_MU, after.impulse, f));
    }

    #[test]
    fn ground_and_marine_induction() {
        for bits in 0u8..=0x0F {
            if let Some(s) = unpack_ground(bits) {
                if ground_invariants(&s) {
                    for e in GroundEvent::ALL {
                        if let Ok(n) = ground_step(s, e) {
                            assert!(ground_invariants(&n));
                        }
                    }
                }
            }
            if let Some(s) = unpack_marine(bits) {
                if marine_invariants(&s) {
                    for e in MarineEvent::ALL {
                        if let Ok(n) = marine_step(s, e) {
                            assert!(marine_invariants(&n));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn shallow_water_periodic_mass() {
        use flight_core::hydro::two_cell_periodic_mass;
        for a in 0..5 {
            for b in 0..5 {
                let h = [0.8 + a as f32 * 0.3, 0.8 + b as f32 * 0.3];
                let h1 = two_cell_periodic_mass(h, [0.2, -0.1], 0.01, 1.0, 9.81);
                assert!(h1[0] >= 0.0 && h1[1] >= 0.0);
                assert!(((h[0] + h[1]) - (h1[0] + h1[1])).abs() < 1e-5);
            }
        }
    }
}
