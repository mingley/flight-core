//! Mechanically verified simulation for aerial, ground, surface, and underwater bodies.
//!
//! One [`World::step`] applies gravity, quadratic drag, hydrostatic buoyancy,
//! pairwise sphere contact, and a vertical contact resolver. A successor that
//! would break a mechanical property is refused: pose and time stay put, and
//! `last_properties` names the rejected vector. Water is a
//! conserved shallow-water field, not a prescribed sinusoid. Actuator force is
//! applied only when the matching safety machine grants it. [`Scene`] names a
//! catalog or a custom body table (seed, wind/current/waves, charges) without
//! registering new names on [`World::named`]. After every step the world
//! re-evaluates:
//!
//! ```text
//! no terrain penetration
//! no body interpenetration
//! drag opposes relative flow
//! buoyancy only when wet
//! aerial actuators ⇒ armed
//! ground drive ⇒ Moving
//! ground drive force ⇒ on terrain
//! marine thrust ⇒ Underway ∨ StationKeep
//! marine thrust force ⇒ in water
//! aerial thrust force ⇒ in air
//! finite mechanics
//! thrust only when granted
//! relative drag power ≤ 0
//! battery gates thrust
//! unit attitude
//! aerial thrust ∥ −body z
//! Coulomb tangent ≤ μ j_n
//! AUV thrust is a body-axis wrench
//! shallow-water volume conserved, h ≥ 0, land dry
//! position hold restores pose
//! ```

#![deny(unsafe_code)]

pub mod body;
pub mod env;
pub mod hydro;
pub mod properties;
pub mod scene;
pub mod world;

#[cfg(feature = "gpu")]
pub mod gpu;

pub use body::Body;
pub use env::Environment;
pub use hydro::HydroField;
pub use properties::{all_hold, evaluate, Property};
pub use scene::{Scene, SceneError};
pub use world::{PropertyViolation, SphereHit, World};

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::domain::Domain;
    use flight_core::ground::{ground_step, GroundEvent};
    use flight_core::prelude::*;

    #[test]
    fn idle_coastal_holds_properties() {
        let mut world = World::coastal(1);
        for _ in 0..400 {
            world.step(0.02);
            assert!(world.all_hold(), "{:?}", world.last_properties);
        }
        let rover = world.body("rover").unwrap();
        assert!(rover.position_m[2] <= 1e-3);
        let drone = world.body("drone").unwrap();
        assert!(drone.position_m[2] <= 1e-3);
        assert!(!drone.actuators_granted());
    }

    #[test]
    fn spawn_support_matches_domain_hold() {
        let coastal = World::coastal(1);
        let rover = coastal.body("rover").unwrap();
        assert!(rover.on_terrain(&coastal.env));
        assert_eq!(rover.support(&coastal.hydro, &coastal.env), "terrain");
        let skiff = coastal.body("skiff").unwrap();
        assert!(!skiff.on_terrain(&coastal.env));
        assert_eq!(skiff.support(&coastal.hydro, &coastal.env), "water");
        let water = World::named("open_water", 1).unwrap();
        let drone = water.body("drone").unwrap();
        assert!(!drone.on_terrain(&water.env));
        assert_eq!(drone.support(&water.hydro, &water.env), "air");
    }

    #[test]
    fn parked_rover_rejects_drive() {
        let world = World::coastal(1);
        let rover = world.body("rover").unwrap();
        let s = rover.ground.unwrap();
        assert_eq!(
            ground_step(s, GroundEvent::DriveCommand),
            Err(GroundReject::IllegalPhase)
        );
    }

    #[test]
    fn airborne_rover_drive_produces_no_thrust() {
        let mut falling = World::coastal(1);
        {
            let rover = falling.body_mut("rover").unwrap();
            rover.ground = Some(ground_step(rover.ground.unwrap(), GroundEvent::Release).unwrap());
            rover.command = Some([-3.0, 0.0, 0.0]);
            rover.position_m[2] = -2.5;
        }
        falling.step(0.02);
        let rover = falling.body("rover").unwrap();
        assert!(
            rover.last_thrust.iter().all(|c| c.abs() < 1e-8),
            "{:?}",
            rover.last_thrust
        );
        assert!(!rover.last_on_terrain);
        assert!(falling.all_hold(), "{:?}", falling.last_properties);

        let mut rolling = World::coastal(1);
        {
            let rover = rolling.body_mut("rover").unwrap();
            rover.ground = Some(ground_step(rover.ground.unwrap(), GroundEvent::Release).unwrap());
            rover.command = Some([-3.0, 0.0, 0.0]);
        }
        rolling.step(0.02);
        let rover = rolling.body("rover").unwrap();
        assert!(
            rover.last_thrust.iter().any(|c| c.abs() > 1e-3),
            "{:?}",
            rover.last_thrust
        );
        assert!(rover.last_on_terrain);
        assert!(rolling.all_hold());
    }

    #[test]
    fn dry_skiff_thrust_is_zero() {
        use flight_core::marine::{marine_step, MarineEvent};

        let mut dry = World::coastal(1);
        {
            let skiff = dry.body_mut("skiff").unwrap();
            skiff.marine = Some(marine_step(skiff.marine.unwrap(), MarineEvent::Undock).unwrap());
            skiff.command = Some([0.0, 2.0, 0.0]);
            skiff.position_m[2] = -1.5;
        }
        dry.step(0.02);
        let skiff = dry.body("skiff").unwrap();
        assert!(
            skiff.last_thrust.iter().all(|c| c.abs() < 1e-8),
            "{:?}",
            skiff.last_thrust
        );
        assert!(!skiff.last_wet);
        assert!(dry.all_hold(), "{:?}", dry.last_properties);

        let mut wet = World::coastal(1);
        {
            let skiff = wet.body_mut("skiff").unwrap();
            skiff.marine = Some(marine_step(skiff.marine.unwrap(), MarineEvent::Undock).unwrap());
            skiff.command = Some([0.0, 2.0, 0.0]);
        }
        wet.step(0.02);
        let skiff = wet.body("skiff").unwrap();
        assert!(
            skiff.last_thrust.iter().any(|c| c.abs() > 1e-3),
            "{:?}",
            skiff.last_thrust
        );
        assert!(skiff.last_wet);
        assert!(wet.all_hold());
    }

    #[test]
    fn disarmed_drone_ignores_climb_command() {
        let mut world = World::coastal(1);
        {
            let drone = world.body_mut("drone").unwrap();
            drone.command = Some([0.0, 0.0, -2.0]);
        }
        for _ in 0..80 {
            world.step(0.02);
        }
        let alt = world.body("drone").unwrap().altitude_agl();
        assert!(alt < 0.2, "disarmed drone climbed to {alt}");
        assert!(world.all_hold());
    }

    #[test]
    fn boat_floats_instead_of_sinking() {
        let mut world = World::coastal(1);
        for _ in 0..250 {
            world.step(0.02);
        }
        let skiff = world.body("skiff").unwrap();
        let z = skiff.position_m[2];
        let surface = world.hydro.surface_z(
            skiff.position_m[0],
            skiff.position_m[1],
            world.env.waterline_z,
        );
        assert!(
            (z - surface).abs() < 0.5,
            "skiff not on free surface z={z} surface={surface}"
        );
        assert!(z < 1.5, "skiff sank z={z}");
        assert!(world.all_hold());
    }

    #[test]
    fn auv_stays_wet_and_neutral() {
        let mut world = World::coastal(1);
        for _ in 0..200 {
            world.step(0.02);
        }
        let auv = world.body("surveyor").unwrap();
        assert_eq!(auv.domain, Domain::Underwater);
        assert!(auv.position_m[2] > 0.5);
        assert!(auv.position_m[2] < 3.8);
        assert!(auv.last_displaced > 0.0);
        assert!(world.all_hold());
    }

    #[test]
    fn auv_body_wrench_holds_under_command() {
        use flight_core::marine::{marine_step, MarineEvent};
        let mut world = World::coastal(1);
        {
            let auv = world.body_mut("surveyor").unwrap();
            let s = marine_step(auv.marine.unwrap(), MarineEvent::Undock).unwrap();
            auv.marine = Some(s);
            auv.command = Some([0.4, 0.2, 0.3]);
            auv.yaw_cmd = 0.2;
        }
        for _ in 0..80 {
            world.step(0.02);
            assert!(world.all_hold(), "{:?}", world.last_properties);
            let auv = world.body("surveyor").unwrap();
            let lim = 8.0 * auv.mass_kg;
            assert!(flight_core::mech::body_wrench_axes_limited(
                auv.quat,
                auv.last_thrust,
                lim
            ));
        }
    }

    #[test]
    fn ungranted_bodies_record_zero_thrust() {
        let mut world = World::coastal(1);
        for _ in 0..40 {
            world.step(0.02);
        }
        for b in &world.bodies {
            assert!(!b.actuators_granted(), "{} granted while idle", b.id);
            assert!(
                b.last_thrust.iter().all(|c| c.abs() < 1e-9),
                "{} thrust {:?}",
                b.id,
                b.last_thrust
            );
            assert!(b.last_ke.is_finite() && b.last_pe.is_finite());
        }
        assert!(world.all_hold());
    }

    #[test]
    fn armed_drone_can_climb() {
        use flight_core::safety::{self, Event};
        let mut world = World::coastal(1);
        {
            let drone = world.body_mut("drone").unwrap();
            let s = drone.aerial.unwrap();
            let s = safety::step_all(
                s,
                &[
                    Event::Arm,
                    Event::HeartbeatFresh,
                    Event::EnterOffboard,
                    Event::EnableActuators,
                    Event::Takeoff,
                ],
            )
            .unwrap();
            drone.aerial = Some(s);
            drone.command = Some([0.0, 0.0, -1.2]);
        }
        for _ in 0..250 {
            world.step(0.02);
        }
        let alt = world.body("drone").unwrap().altitude_agl();
        assert!(alt > 3.0, "armed drone alt {alt}");
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let drone = world.body("drone").unwrap();
        assert!(flight_core::mech::thrust_along_minus_body_z(
            drone.quat,
            drone.last_thrust
        ));
    }

    #[test]
    fn submerged_drone_produces_no_thrust() {
        use flight_core::safety::{self, Event};
        let mut world = World::coastal(1);
        {
            let drone = world.body_mut("drone").unwrap();
            let s = safety::step_all(
                drone.aerial.unwrap(),
                &[
                    Event::Arm,
                    Event::HeartbeatFresh,
                    Event::EnterOffboard,
                    Event::EnableActuators,
                    Event::Takeoff,
                ],
            )
            .unwrap();
            drone.aerial = Some(s);
            drone.position_m = [-8.0, 0.0, 1.4];
            drone.command = Some([0.0, 0.0, -1.2]);
        }
        world.step(0.02);
        let drone = world.body("drone").unwrap();
        assert!(drone.last_wet);
        assert!(
            drone.last_thrust.iter().all(|c| c.abs() < 1e-8),
            "{:?}",
            drone.last_thrust
        );
        assert!(world.all_hold(), "{:?}", world.last_properties);
    }

    #[test]
    fn tilt_produces_horizontal_accel() {
        use flight_core::safety::{self, Event};
        let mut world = World::inland(1);
        {
            let drone = world.body_mut("drone").unwrap();
            let s = drone.aerial.unwrap();
            let s = safety::step_all(
                s,
                &[
                    Event::Arm,
                    Event::HeartbeatFresh,
                    Event::EnterOffboard,
                    Event::EnableActuators,
                    Event::Takeoff,
                ],
            )
            .unwrap();
            drone.aerial = Some(s);
            drone.command = Some([0.0, 1.4, -0.4]);
        }
        for _ in 0..200 {
            world.step(0.02);
            assert!(world.all_hold(), "{:?}", world.last_properties);
        }
        let drone = world.body("drone").unwrap();
        assert!(
            drone.velocity_mps[1] > 0.4,
            "east vel {:?}",
            drone.velocity_mps
        );
        assert!(drone.altitude_agl() > 0.3);
        assert!(flight_core::mech::thrust_along_minus_body_z(
            drone.quat,
            drone.last_thrust
        ));
    }

    #[test]
    fn yaw_command_turns_and_keeps_unit_quat() {
        use flight_core::ground::GroundEvent;
        let mut world = World::inland(1);
        {
            let rover = world.body_mut("rover").unwrap();
            let s = ground_step(rover.ground.unwrap(), GroundEvent::Release).unwrap();
            let s = ground_step(s, GroundEvent::DriveCommand).unwrap();
            rover.ground = Some(s);
            rover.yaw_cmd = 0.8;
            rover.command = Some([0.0, 0.0, 0.0]);
        }
        for _ in 0..80 {
            world.step(0.02);
            assert!(world.all_hold(), "{:?}", world.last_properties);
        }
        let rover = world.body("rover").unwrap();
        assert!(rover.yaw_rate.abs() > 0.2, "yaw_rate {}", rover.yaw_rate);
        assert!(rover.last_angular_ke > 0.0);
        assert!(flight_core::mech::quat_is_unit(rover.quat));
    }

    #[test]
    fn empty_battery_blocks_granted_thrust() {
        use flight_core::safety::{self, Event};
        let mut world = World::coastal(1);
        {
            let drone = world.body_mut("drone").unwrap();
            let s = drone.aerial.unwrap();
            let s = safety::step_all(
                s,
                &[
                    Event::Arm,
                    Event::HeartbeatFresh,
                    Event::EnterOffboard,
                    Event::EnableActuators,
                    Event::Takeoff,
                ],
            )
            .unwrap();
            drone.aerial = Some(s);
            drone.charge_j = 0.0;
            drone.last_charge_j = 0.0;
            drone.command = Some([0.0, 0.0, -1.2]);
            assert!(drone.actuators_granted());
            assert!(!drone.propulsion_live());
        }
        for _ in 0..120 {
            world.step(0.02);
        }
        let drone = world.body("drone").unwrap();
        assert!(
            drone.altitude_agl() < 0.3,
            "empty pack climbed {}",
            drone.altitude_agl()
        );
        assert!(drone.last_thrust.iter().all(|c| c.abs() < 1e-9));
        assert!(world.all_hold(), "{:?}", world.last_properties);
    }

    #[test]
    fn zero_motor_efficiency_blocks_granted_thrust() {
        use flight_core::safety::{self, Event};
        let mut world = World::inland(1);
        {
            let drone = world.body_mut("drone").unwrap();
            let s = drone.aerial.unwrap();
            let s = safety::step_all(
                s,
                &[
                    Event::Arm,
                    Event::HeartbeatFresh,
                    Event::EnterOffboard,
                    Event::EnableActuators,
                    Event::Takeoff,
                ],
            )
            .unwrap();
            drone.aerial = Some(s);
            drone.thrust_scale = 0.0;
            drone.command = Some([0.0, 0.0, -1.2]);
            assert!(drone.actuators_granted());
            assert!(drone.propulsion_live());
        }
        for _ in 0..120 {
            world.step(0.02);
        }
        let drone = world.body("drone").unwrap();
        assert!(
            drone.altitude_agl() < 0.3,
            "zero-efficiency pack climbed {}",
            drone.altitude_agl()
        );
        assert!(drone.last_thrust.iter().all(|c| c.abs() < 1e-9));
        assert!(flight_core::mech::thrust_along_minus_body_z(
            drone.quat,
            drone.last_thrust
        ));
        assert!(world.all_hold(), "{:?}", world.last_properties);
    }

    #[test]
    fn imu_delay_step_change_does_not_rewind_estimator_ts() {
        let mut world = World::inland(1);
        world.step(0.02);
        let before = world.body("drone").unwrap().last_estimator_ts_ms;
        assert!(before > 0);
        {
            let drone = world.body_mut("drone").unwrap();
            drone.imu_delay_ms = 300;
        }
        world.step(0.02);
        let after = world.body("drone").unwrap().last_estimator_ts_ms;
        assert!(
            after >= before,
            "estimator ts jumped backward {before} -> {after}"
        );
        assert_eq!(after, before);
        assert!(world.all_hold());
    }

    #[test]
    fn named_scenarios_exist() {
        assert_eq!(World::SCENARIOS.len(), 4);
        for name in World::SCENARIOS {
            let mut w = World::named(name, 2).expect(name);
            assert_eq!(w.scenario, *name);
            for _ in 0..80 {
                w.step(0.02);
                assert!(w.all_hold(), "{name} {:?}", w.last_properties);
            }
        }
        assert!(World::named("vacuum", 1).is_none());
        assert_eq!(World::inland(1).bodies.len(), 2);
        assert_eq!(World::open_water(1).bodies.len(), 3);
    }

    #[test]
    fn seed_replays_and_diverges() {
        let mut a = World::coastal(11);
        let mut b = World::coastal(11);
        let mut c = World::coastal(12);
        for _ in 0..80 {
            a.step(0.02);
            b.step(0.02);
            c.step(0.02);
        }
        assert_eq!(
            a.body("skiff").unwrap().position_m,
            b.body("skiff").unwrap().position_m
        );
        assert_ne!(a.env.wave_phase, c.env.wave_phase);
        assert!(a.all_hold() && c.all_hold());
    }

    #[test]
    fn overlapping_bodies_are_separated() {
        let mut world = World::inland(1);
        {
            let rover = world.body_mut("rover").unwrap();
            rover.position_m = [6.0, 0.05, 0.0];
            rover.velocity_mps = [0.0, -0.8, 0.0];
        }
        {
            let drone = world.body_mut("drone").unwrap();
            drone.velocity_mps = [0.0, 0.4, 0.0];
        }
        world.step(0.02);
        let drone = world.body("drone").unwrap();
        let rover = world.body("rover").unwrap();
        let dx = rover.position_m[0] - drone.position_m[0];
        let dy = rover.position_m[1] - drone.position_m[1];
        let dz = rover.position_m[2] - drone.position_m[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(
            dist + 1e-3 >= drone.radius_m + rover.radius_m,
            "dist {dist} radii {} {}",
            drone.radius_m,
            rover.radius_m
        );
        assert!(drone.last_sphere_impulse > 0.0 || rover.last_sphere_impulse > 0.0);
        let hit = world
            .sphere_hit_between("drone", "rover")
            .expect("pairwise hit graph");
        assert!(hit.jn > 0.0);
        assert!(world.all_hold(), "{:?}", world.last_properties);
    }

    #[test]
    fn glancing_contact_applies_spin() {
        let mut world = World::inland(1);
        {
            let rover = world.body_mut("rover").unwrap();
            rover.position_m = [6.35, 0.45, 0.0];
            rover.velocity_mps = [-0.2, -1.4, 0.0];
            rover.omega_body = [0.0, 0.0, 0.0];
        }
        {
            let drone = world.body_mut("drone").unwrap();
            drone.position_m = [6.0, 0.0, 0.0];
            drone.velocity_mps = [0.0, 0.0, 0.0];
            drone.omega_body = [0.0, 0.0, 0.0];
        }
        world.step(0.02);
        let drone = world.body("drone").unwrap();
        let rover = world.body("rover").unwrap();
        assert!(
            drone.last_tangent_impulse > 0.0 || rover.last_tangent_impulse > 0.0,
            "jt drone={} rover={}",
            drone.last_tangent_impulse,
            rover.last_tangent_impulse
        );
        let spin = drone
            .omega_body
            .iter()
            .chain(rover.omega_body.iter())
            .any(|w| w.abs() > 1e-4);
        assert!(
            spin,
            "omega drone={:?} rover={:?}",
            drone.omega_body, rover.omega_body
        );
        assert!(world.all_hold(), "{:?}", world.last_properties);
    }

    #[test]
    fn coastal_hydro_conserves_volume() {
        let mut world = World::coastal(1);
        let v0 = world.hydro.volume0;
        assert!(v0 > 100.0, "expected a wet patch, volume={v0}");
        for _ in 0..200 {
            world.step(0.02);
            assert!(world.all_hold(), "{:?}", world.last_properties);
        }
        let ids: Vec<_> = world.last_properties.iter().map(|p| p.id).collect();
        assert!(ids.contains(&"hydro_volume_conserved"));
        assert!(ids.contains(&"hydro_height_nonnegative"));
        assert!(ids.contains(&"hydro_land_stays_dry"));
        assert!(ids.contains(&"position_hold_restores_pose"));
        let inland = World::inland(1);
        assert!(inland.hydro.volume0.abs() < 1e-6);
        assert!(inland.hydro.invariants().all());
    }

    #[test]
    fn try_step_refuses_illegal_successor() {
        let mut world = World::coastal(1);
        let t0 = world.t;
        let p0 = world.body("rover").unwrap().position_m;
        let h0 = world.hydro.h.clone();
        world.hydro.volume0 = 1.0e9;
        let err = world.try_step(0.02).expect_err("volume0 is a lie");
        assert!(
            err.broken().contains(&"hydro_volume_conserved"),
            "{:?}",
            err.broken()
        );
        assert_eq!(world.t, t0);
        assert_eq!(world.body("rover").unwrap().position_m, p0);
        assert_eq!(world.hydro.h, h0);
        assert!(!world.all_hold());
    }

    #[test]
    fn position_hold_is_a_named_property() {
        use flight_core::safety::{self, Event};

        let idle = World::inland(1);
        assert!(idle
            .last_properties
            .iter()
            .any(|p| p.id == "position_hold_restores_pose" && p.holds));

        let mut world = World::inland(1);
        {
            let drone = world.body_mut("drone").unwrap();
            drone.aerial = Some(
                safety::step_all(
                    drone.aerial.unwrap(),
                    &[
                        Event::Arm,
                        Event::HeartbeatFresh,
                        Event::EnterOffboard,
                        Event::EnableActuators,
                        Event::Takeoff,
                    ],
                )
                .unwrap(),
            );
            let p = drone.position_m;
            drone.set_position_hold(p);
        }
        world.step(0.02);
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let drone = world.body("drone").unwrap();
        assert!(drone.hold_ned.is_some());
        assert!(world
            .last_properties
            .iter()
            .any(|p| p.id == "position_hold_restores_pose" && p.holds));
    }

    #[test]
    fn try_step_refuses_a_diverging_position_hold() {
        use flight_core::safety::{self, Event};

        let mut world = World::inland(1);
        {
            let drone = world.body_mut("drone").unwrap();
            drone.aerial = Some(
                safety::step_all(
                    drone.aerial.unwrap(),
                    &[
                        Event::Arm,
                        Event::HeartbeatFresh,
                        Event::EnterOffboard,
                        Event::EnableActuators,
                        Event::Takeoff,
                    ],
                )
                .unwrap(),
            );
            drone.hold_ned = Some([f32::NAN, 0.0, 0.0]);
            drone.command = Some([0.0, 0.0, 0.0]);
        }
        let t0 = world.t;
        let err = world
            .try_step(0.02)
            .expect_err("non-finite hold must not commit");
        assert!(
            err.broken().contains(&"position_hold_restores_pose"),
            "{:?}",
            err.broken()
        );
        assert_eq!(world.t, t0);
        assert!(!world.all_hold());
    }

    #[test]
    fn plant_attitude_is_physics_truth_not_the_nav_filter() {
        let mut world = World::inland(1);
        world.try_step(0.02).unwrap();
        let q = world.body("drone").unwrap().quat;
        assert!(flight_core::mech::quat_is_unit(q));
        let est = flight_core::nav::ComplementaryAttitude::new();
        assert!(
            !est.is_valid(),
            "filter starts invalid; the plant still holds"
        );
        assert!(world.all_hold());
        assert!(world
            .last_properties
            .iter()
            .any(|p| p.id == "unit_attitude" && p.holds));
    }
}
