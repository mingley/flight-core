//! Mechanical properties evaluated after every world step.

use crate::body::Body;
use crate::env::Environment;
use crate::hydro::HydroField;
use crate::world::World;
use flight_core::domain::Domain;
use flight_core::ground::{ground_invariants, GroundPhase};
use flight_core::marine::marine_invariants;
use flight_core::mech::{
    aerial_thrust_only_in_air, battery_gates_thrust, body_wrench_axes_limited,
    buoyancy_only_when_wet, contact_invariants, drag_opposes_flow, ground_thrust_only_on_contact,
    hold_restores_pose, marine_thrust_only_when_wet, mechanics_finite, quat_is_unit,
    relative_power, rigid_spin_invariants, thrust_along_minus_body_z, thrust_only_when_granted,
    SPHERE_FRICTION_MU,
};
use flight_core::safety::check_invariants;
use serde::Serialize;

/// Named boolean a researcher or agent can watch.
#[derive(Clone, Debug, Serialize)]
pub struct Property {
    pub id: &'static str,
    pub holds: bool,
    pub detail: String,
}

impl Property {
    fn check(id: &'static str, holds: bool, detail: impl Into<String>) -> Self {
        Self {
            id,
            holds,
            detail: detail.into(),
        }
    }
}

/// Evaluate every mechanical / safety property against the latest step.
pub fn evaluate(world: &World) -> Vec<Property> {
    evaluate_parts(&world.env, &world.bodies, &world.hydro)
}

pub fn evaluate_env(env: &Environment, bodies: &[Body]) -> Vec<Property> {
    evaluate_parts(env, bodies, &HydroField::from_env(env))
}

pub fn evaluate_parts(env: &Environment, bodies: &[Body], hydro: &HydroField) -> Vec<Property> {
    let mut penetration_ok = true;
    let mut contact_ok = true;
    let mut drag_ok = true;
    let mut wet_ok = true;
    let mut aerial_ok = true;
    let mut ground_ok = true;
    let mut marine_ok = true;
    let mut finite_ok = true;
    let mut thrust_ok = true;
    let mut drag_power_ok = true;
    let mut spheres_ok = true;
    let mut battery_ok = true;
    let mut attitude_ok = true;
    let mut body_z_ok = true;
    let mut friction_ok = true;
    let mut auv_ok = true;
    let mut ground_contact_ok = true;
    let mut marine_wet_ok = true;
    let mut aerial_air_ok = true;
    let mut hold_ok = true;

    for b in bodies {
        let terrain = env.terrain_z(b.position_m[0], b.position_m[1]);
        if b.position_m[2] > terrain + 1e-4 {
            penetration_ok = false;
        }
        if !contact_invariants(b.last_contact_before, b.last_contact) {
            contact_ok = false;
        }
        if !drag_opposes_flow(b.last_v_rel, b.last_drag) {
            drag_ok = false;
        }
        if !buoyancy_only_when_wet(b.last_displaced, b.last_buoyancy_z) {
            wet_ok = false;
        }
        if !mechanics_finite(b.mass_kg, b.position_m[2], b.velocity_mps, b.yaw_rate) {
            finite_ok = false;
        }
        if !thrust_only_when_granted(b.actuators_granted(), b.last_thrust) {
            thrust_ok = false;
        }
        if !battery_gates_thrust(b.last_charge_j, b.last_thrust) {
            battery_ok = false;
        }
        if !quat_is_unit(b.quat)
            || !rigid_spin_invariants(b.inertia_diag, b.omega_body, b.omega_body, b.quat)
        {
            attitude_ok = false;
        }
        if b.domain == Domain::Aerial && !thrust_along_minus_body_z(b.quat, b.last_thrust) {
            body_z_ok = false;
        }
        if b.last_tangent_impulse > SPHERE_FRICTION_MU * b.last_sphere_impulse + 1e-3 {
            friction_ok = false;
        }
        if b.domain == Domain::Ground
            && !ground_thrust_only_on_contact(b.last_on_terrain, b.last_thrust)
        {
            ground_contact_ok = false;
        }
        if matches!(b.domain, Domain::Surface | Domain::Underwater)
            && !marine_thrust_only_when_wet(b.last_wet, b.last_thrust)
        {
            marine_wet_ok = false;
        }
        if b.domain == Domain::Aerial && !aerial_thrust_only_in_air(!b.last_wet, b.last_thrust) {
            aerial_air_ok = false;
        }
        if b.domain == Domain::Underwater {
            let lim = 8.0 * b.mass_kg;
            if !body_wrench_axes_limited(b.quat, b.last_thrust, lim) {
                auv_ok = false;
            }
        }
        if relative_power(b.last_v_rel, b.last_drag) > 1e-5 {
            drag_power_ok = false;
        }
        if let Some(hold) = b.hold_ned {
            match b.command {
                Some(cmd) if hold_restores_pose(hold, b.position_m, cmd) => {}
                _ => hold_ok = false,
            }
        }
        match b.domain {
            Domain::Aerial => {
                if let Some(s) = b.aerial {
                    if !check_invariants(&s) || (s.actuators_enabled && !s.armed) {
                        aerial_ok = false;
                    }
                }
            }
            Domain::Ground => {
                if let Some(s) = b.ground {
                    if !ground_invariants(&s) || (s.drive_enabled && s.phase != GroundPhase::Moving)
                    {
                        ground_ok = false;
                    }
                }
            }
            Domain::Surface | Domain::Underwater => {
                if let Some(s) = b.marine {
                    if !marine_invariants(&s) {
                        marine_ok = false;
                    }
                }
            }
        }
    }

    for (i, a) in bodies.iter().enumerate() {
        for b in bodies.iter().skip(i + 1) {
            let dx = b.position_m[0] - a.position_m[0];
            let dy = b.position_m[1] - a.position_m[1];
            let dz = b.position_m[2] - a.position_m[2];
            let dist2 = dx * dx + dy * dy + dz * dz;
            let min_d = a.radius_m.max(0.0) + b.radius_m.max(0.0);
            let limit = (min_d - 1e-3).max(0.0);
            if dist2 < limit * limit {
                spheres_ok = false;
            }
        }
    }

    let hydro_ok = hydro.invariants();

    vec![
        Property::check(
            "no_terrain_penetration",
            penetration_ok && contact_ok,
            "after resolve, z ≤ terrain and impulse only on contact",
        ),
        Property::check(
            "drag_opposes_relative_flow",
            drag_ok,
            "quadratic drag satisfies F · v_rel ≤ 0",
        ),
        Property::check(
            "buoyancy_only_when_wet",
            wet_ok,
            "hydrostatic lift is zero when displaced volume is zero",
        ),
        Property::check(
            "aerial_actuators_require_arm",
            aerial_ok,
            "actuators_enabled ⇒ armed, aerial machine invariants",
        ),
        Property::check(
            "aerial_thrust_only_in_air",
            aerial_air_ok,
            "aerial actuator force is identically zero unless the rotors are in air",
        ),
        Property::check(
            "ground_drive_requires_moving",
            ground_ok,
            "drive_enabled ⇒ Moving ∧ ¬estop",
        ),
        Property::check(
            "ground_drive_only_on_contact",
            ground_contact_ok,
            "ground actuator force is identically zero unless the hull is on the terrain plane",
        ),
        Property::check(
            "marine_thrust_requires_grant",
            marine_ok,
            "thrust_enabled ⇒ Underway ∨ StationKeep, ¬failsafe",
        ),
        Property::check(
            "marine_thrust_only_when_wet",
            marine_wet_ok,
            "marine actuator force is identically zero unless the hull is in water",
        ),
        Property::check(
            "finite_mechanics",
            finite_ok,
            "mass, pose, velocity, and kinetic energy stay finite",
        ),
        Property::check(
            "thrust_only_when_granted",
            thrust_ok,
            "actuator force is identically zero unless the domain machine granted it",
        ),
        Property::check(
            "relative_drag_power_nonpositive",
            drag_power_ok,
            "F_drag · v_rel ≤ 0 in the flow-relative frame",
        ),
        Property::check(
            "no_body_interpenetration",
            spheres_ok,
            "after sphere resolve, |p_a − p_b| ≥ r_a + r_b",
        ),
        Property::check(
            "battery_gates_thrust",
            battery_ok,
            "empty energy pack ⇒ actuator force is identically zero",
        ),
        Property::check(
            "unit_attitude",
            attitude_ok,
            "physics-truth quaternion stays unit length; angular KE and ω stay finite",
        ),
        Property::check(
            "aerial_thrust_along_minus_body_z",
            body_z_ok,
            "quadrotor actuator force is parallel to −body z in NED",
        ),
        Property::check(
            "coulomb_friction_cone",
            friction_ok,
            "sphere tangent impulse stays inside μ j_n",
        ),
        Property::check(
            "auv_thrust_on_body_axes",
            auv_ok,
            "underwater actuator force is a body-axis wrench inside per-thruster limits",
        ),
        Property::check(
            "hydro_height_nonnegative",
            hydro_ok.height_nonnegative,
            "shallow-water column h ≥ 0",
        ),
        Property::check(
            "hydro_volume_conserved",
            hydro_ok.volume_conserved,
            "no-flux Saint-Venant step conserves water volume",
        ),
        Property::check(
            "hydro_land_stays_dry",
            hydro_ok.land_dry && hydro_ok.finite,
            "land cells stay dry; hydro state stays finite",
        ),
        Property::check(
            "position_hold_restores_pose",
            hold_ok,
            "when hold_ned is set, command · (hold − pose) ≥ 0 and the command is finite",
        ),
    ]
}

pub fn all_hold(properties: &[Property]) -> bool {
    properties.iter().all(|p| p.holds)
}
