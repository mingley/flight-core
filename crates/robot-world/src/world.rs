//! Deterministic world step: gravity, drag, buoyancy, granted thrust, contact.

use crate::body::Body;
use crate::env::Environment;
use crate::hydro::HydroField;
use crate::properties::{self, Property};
use flight_core::domain::{Domain, Medium};
use flight_core::mech::{
    angular_kinetic_energy, apply_sphere_friction, body_axis_wrench, body_z_thrust_ned,
    buoyancy_ned, drain_from_thrust, euler_principal_step, gravitational_pe_ned, kinetic_energy,
    quadratic_drag, quat_integrate, quat_rotate, quat_rotate_inv, relative_power,
    resolve_sphere_contact, resolve_vertical_contact, vec_cross, vec_norm, SphereContact,
    SphereSpin, VerticalContact, SPHERE_FRICTION_MU,
};

/// Multi-domain scene. One `step` is the only mutation of pose.
#[derive(Clone, Debug)]
pub struct World {
    pub t: f32,
    pub seed: u64,
    pub scenario: &'static str,
    pub env: Environment,
    pub hydro: HydroField,
    pub bodies: Vec<Body>,
    pub last_properties: Vec<Property>,
    /// Pairwise sphere hits from the last committed (or rejected) step.
    pub last_sphere_hits: Vec<SphereHit>,
}

impl World {
    pub const SCENARIOS: &'static [&'static str] = &["coastal", "inland", "harbor", "open_water"];

    pub fn named(name: &str, seed: u64) -> Option<Self> {
        match name {
            "coastal" => Some(Self::coastal(seed)),
            "inland" => Some(Self::inland(seed)),
            "harbor" => Some(Self::harbor(seed)),
            "open_water" | "open-water" => Some(Self::open_water(seed)),
            _ => None,
        }
    }

    pub(crate) fn assemble(
        scenario: &'static str,
        seed: u64,
        mut env: Environment,
        bodies: Vec<Body>,
    ) -> Self {
        env.apply_seed(seed);
        let hydro = HydroField::from_env(&env);
        let mut world = Self {
            t: 0.0,
            seed,
            scenario,
            env,
            hydro,
            bodies,
            last_properties: Vec::new(),
            last_sphere_hits: Vec::new(),
        };
        world.last_properties = properties::evaluate(&world);
        world
    }

    pub fn coastal(seed: u64) -> Self {
        Self::assemble(
            "coastal",
            seed,
            Environment::coastal(),
            vec![
                Body::aerial_ready("drone"),
                Body::rover("rover"),
                Body::skiff("skiff"),
                Body::surveyor("surveyor"),
            ],
        )
    }

    pub fn inland(seed: u64) -> Self {
        let mut drone = Body::aerial_ready("drone");
        drone.position_m = [6.0, 0.0, 0.0];
        let mut rover = Body::rover("rover");
        rover.position_m = [10.0, 4.0, 0.0];
        Self::assemble("inland", seed, Environment::inland(), vec![drone, rover])
    }

    pub fn harbor(seed: u64) -> Self {
        Self::assemble(
            "harbor",
            seed,
            Environment::harbor(),
            vec![
                Body::aerial_ready("drone"),
                Body::rover("rover"),
                Body::skiff("skiff"),
                Body::surveyor("surveyor"),
            ],
        )
    }

    pub fn open_water(seed: u64) -> Self {
        let mut drone = Body::aerial_ready("drone");
        drone.position_m = [0.0, 0.0, -8.0];
        let mut skiff = Body::skiff("skiff");
        skiff.position_m = [5.0, -1.0, 0.15];
        let mut surveyor = Body::surveyor("surveyor");
        surveyor.position_m = [-6.0, 5.0, 4.0];
        Self::assemble(
            "open_water",
            seed,
            Environment::open_water(),
            vec![drone, skiff, surveyor],
        )
    }

    pub fn body(&self, id: &str) -> Option<&Body> {
        self.bodies.iter().find(|b| b.id == id)
    }

    pub fn body_mut(&mut self, id: &str) -> Option<&mut Body> {
        self.bodies.iter_mut().find(|b| b.id == id)
    }

    pub fn sphere_hit_between(&self, a: &str, b: &str) -> Option<&SphereHit> {
        self.last_sphere_hits
            .iter()
            .find(|h| h.involves(a) && h.involves(b))
    }

    pub fn all_hold(&self) -> bool {
        properties::all_hold(&self.last_properties)
    }

    /// Integrate every body, resolve pairwise spheres, then terrain.
    ///
    /// A successor that would break a mechanical property is not committed.
    /// [`last_properties`](Self::last_properties) still names the rejected vector.
    pub fn step(&mut self, dt: f32) {
        let _ = self.try_step(dt);
    }

    /// Like [`step`], but `Err` if the successor would violate a property.
    /// Pose, hydro, and `t` stay at the previous legal snapshot.
    pub fn try_step(&mut self, dt: f32) -> Result<(), PropertyViolation> {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return Ok(());
        }
        let mut next = self.clone();
        next.advance(dt);
        if next.all_hold() {
            *self = next;
            Ok(())
        } else {
            let properties = next.last_properties;
            self.last_properties = properties.clone();
            Err(PropertyViolation { properties })
        }
    }

    fn advance(&mut self, dt: f32) {
        self.hydro.step(dt, &self.env);
        let env = self.env;
        for body in &mut self.bodies {
            step_body(&env, &self.hydro, body, dt);
        }
        resolve_body_contacts(&mut self.bodies, &mut self.last_sphere_hits, true);
        for body in &mut self.bodies {
            apply_vertical_contact(&env, body);
        }
        resolve_body_contacts(&mut self.bodies, &mut self.last_sphere_hits, false);
        for body in &mut self.bodies {
            apply_vertical_contact(&env, body);
            body.last_ke = kinetic_energy(body.mass_kg, body.velocity_mps);
            body.last_pe = gravitational_pe_ned(body.mass_kg, body.position_m[2], env.gravity);
            body.last_angular_ke = angular_kinetic_energy(body.inertia_diag, body.omega_body);
        }
        self.t += dt;
        stamp_estimators(&mut self.bodies, self.t);
        self.last_properties = properties::evaluate_parts(&self.env, &self.bodies, &self.hydro);
    }
}

/// A [`World::try_step`] successor that failed the property vector.
#[derive(Clone, Debug)]
pub struct PropertyViolation {
    pub properties: Vec<Property>,
}

impl PropertyViolation {
    pub fn broken(&self) -> Vec<&'static str> {
        self.properties
            .iter()
            .filter(|p| !p.holds)
            .map(|p| p.id)
            .collect()
    }
}

impl std::fmt::Display for PropertyViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "property violation: {}", self.broken().join(", "))
    }
}

impl std::error::Error for PropertyViolation {}

/// One pairwise sphere contact from the last contact sweep.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct SphereHit {
    pub a: &'static str,
    pub b: &'static str,
    pub jn: f32,
    pub jt: f32,
}

impl SphereHit {
    fn pair(a: &'static str, b: &'static str, jn: f32, jt: f32) -> Self {
        if a <= b {
            Self { a, b, jn, jt }
        } else {
            Self { a: b, b: a, jn, jt }
        }
    }

    pub fn involves(&self, id: &str) -> bool {
        self.a == id || self.b == id
    }

    pub fn other(&self, id: &str) -> Option<&'static str> {
        if self.a == id {
            Some(self.b)
        } else if self.b == id {
            Some(self.a)
        } else {
            None
        }
    }
}

fn step_body(env: &Environment, hydro: &HydroField, b: &mut Body, dt: f32) {
    let n = b.position_m[0];
    let e = b.position_m[1];
    let z = b.position_m[2];
    let sample = hydro.sample(n, e, env.waterline_z);
    let medium = if sample.still > 0.0 && z > sample.surface_z {
        Medium::Water
    } else {
        Medium::Air
    };
    let rho = env.density(medium);
    let flow = if medium == Medium::Water {
        [sample.un, sample.ue, 0.0]
    } else {
        env.wind_ned
    };
    let v_rel = [
        b.velocity_mps[0] - flow[0],
        b.velocity_mps[1] - flow[1],
        b.velocity_mps[2] - flow[2],
    ];
    let drag = quadratic_drag(v_rel, rho, b.cd, b.area_m2);

    let surface = sample.surface_z;
    let displaced = if medium == Medium::Water && z > surface {
        let depth = (z - surface).max(0.0);
        let frac = (depth / b.draft_m.max(1e-3)).clamp(0.0, 1.0);
        b.hull_volume_m3 * frac
    } else {
        0.0
    };
    let buoyancy_z = buoyancy_ned(displaced, env.water_density, env.gravity);

    let mut force = [
        drag[0],
        drag[1],
        drag[2] + buoyancy_z + b.mass_kg * env.gravity,
    ];

    let on_terrain = b.on_terrain(env);
    b.last_on_terrain = on_terrain;
    let wet = medium == Medium::Water;
    b.last_wet = wet;
    let marine = matches!(b.domain, Domain::Surface | Domain::Underwater);
    let aerial = b.domain == Domain::Aerial;
    let granted = b.propulsion_live()
        && (b.domain != Domain::Ground || on_terrain)
        && (!marine || wet)
        && !(aerial && wet);
    let mut thrust = [0.0; 3];
    // Aerial collective is applied after the attitude step so `last_thrust`
    // is parallel to the snapshot quaternion (the property the lab checks).
    let mut aerial_collective: Option<f32> = None;
    let mut underwater_des: Option<[f32; 3]> = None;
    let mut underwater_limit = 0.0;
    if granted {
        b.refresh_hold();
        let kp = match b.domain {
            Domain::Aerial => 3.2,
            Domain::Ground => 5.0,
            Domain::Surface => 2.4,
            Domain::Underwater => 2.8,
        };
        let limit = match b.domain {
            Domain::Aerial => 18.0 * b.mass_kg,
            Domain::Ground => 12.0 * b.mass_kg,
            Domain::Surface => 10.0 * b.mass_kg,
            Domain::Underwater => 8.0 * b.mass_kg,
        };
        let sp = b.command.unwrap_or([0.0, 0.0, 0.0]);
        let mut f_des = [
            kp * b.mass_kg * (sp[0] - b.velocity_mps[0]),
            kp * b.mass_kg * (sp[1] - b.velocity_mps[1]),
            kp * b.mass_kg * (sp[2] - b.velocity_mps[2]),
        ];
        match b.domain {
            Domain::Aerial => {
                f_des[2] += -b.mass_kg * env.gravity;
            }
            Domain::Ground | Domain::Surface => {
                f_des[2] = 0.0;
            }
            Domain::Underwater => {}
        }
        if b.domain != Domain::Underwater {
            clamp3(&mut f_des, limit);
        }
        let torque = match b.domain {
            Domain::Aerial => {
                let t_mag =
                    (f_des[0] * f_des[0] + f_des[1] * f_des[1] + f_des[2] * f_des[2]).sqrt();
                let zb = quat_rotate(b.quat, [0.0, 0.0, 1.0]);
                let z_des = if t_mag > 1e-3 {
                    vec_norm([-f_des[0], -f_des[1], -f_des[2]])
                } else {
                    zb
                };
                let err_body = quat_rotate_inv(b.quat, vec_cross(zb, z_des));
                aerial_collective = Some(t_mag);
                [
                    b.inertia_diag[0] * (14.0 * err_body[0] - 5.0 * b.omega_body[0]),
                    b.inertia_diag[1] * (14.0 * err_body[1] - 5.0 * b.omega_body[1]),
                    b.inertia_diag[2].max(1e-6) * 8.0 * (b.yaw_cmd - b.omega_body[2]),
                ]
            }
            Domain::Ground | Domain::Surface => {
                let f_body = quat_rotate_inv(b.quat, f_des);
                thrust = quat_rotate(b.quat, [f_body[0], f_body[1], 0.0]);
                thrust[2] = 0.0;
                [
                    -4.0 * b.omega_body[0] * b.inertia_diag[0],
                    -4.0 * b.omega_body[1] * b.inertia_diag[1],
                    b.inertia_diag[2].max(1e-6) * 8.0 * (b.yaw_cmd - b.omega_body[2]),
                ]
            }
            Domain::Underwater => {
                underwater_des = Some(f_des);
                underwater_limit = limit;
                [
                    -4.0 * b.omega_body[0] * b.inertia_diag[0],
                    -4.0 * b.omega_body[1] * b.inertia_diag[1],
                    b.inertia_diag[2].max(1e-6) * 8.0 * (b.yaw_cmd - b.omega_body[2]),
                ]
            }
        };
        b.omega_body = euler_principal_step(b.omega_body, torque, b.inertia_diag, dt);
    } else {
        let damp = [
            -0.45 * b.omega_body[0],
            -0.45 * b.omega_body[1],
            -0.45 * b.omega_body[2],
        ];
        b.omega_body = euler_principal_step(b.omega_body, damp, b.inertia_diag, dt);
        b.clear_command();
    }
    b.quat = quat_integrate(b.quat, b.omega_body, dt);
    if let Some(t_mag) = aerial_collective {
        thrust = body_z_thrust_ned(b.quat, t_mag);
    }
    if let Some(f_des) = underwater_des {
        thrust = body_axis_wrench(b.quat, f_des, underwater_limit);
    }
    let scale = b.thrust_scale.clamp(0.0, 1.0);
    thrust = [thrust[0] * scale, thrust[1] * scale, thrust[2] * scale];
    b.yaw_rate = b.omega_body[2];
    b.yaw_rad = yaw_from_quat(b.quat);
    force[0] += thrust[0];
    force[1] += thrust[1];
    force[2] += thrust[2];

    let inv_m = 1.0 / b.mass_kg.max(1e-6);
    b.velocity_mps[0] += force[0] * inv_m * dt;
    b.velocity_mps[1] += force[1] * inv_m * dt;
    b.velocity_mps[2] += force[2] * inv_m * dt;
    b.position_m[0] += b.velocity_mps[0] * dt;
    b.position_m[1] += b.velocity_mps[1] * dt;
    b.position_m[2] += b.velocity_mps[2] * dt;

    if b.domain == Domain::Surface {
        if let Some(m) = b.marine {
            if m.phase == flight_core::marine::MarinePhase::Docked {
                b.velocity_mps[0] *= 0.15;
                b.velocity_mps[1] *= 0.15;
                b.velocity_mps[2] *= 0.4;
            }
        }
    }

    b.last_drag = drag;
    b.last_v_rel = v_rel;
    b.last_buoyancy_z = buoyancy_z;
    b.last_displaced = displaced;
    b.last_thrust = thrust;
    b.last_drag_power = relative_power(v_rel, drag);
    b.last_charge_j = b.charge_j;
    b.charge_j = drain_from_thrust(b.charge_j, thrust, dt, 0.08);
}

/// Estimator timestamps lag by [`Body::imu_delay_ms`] and never jump backward
/// when that delay steps up (Requirement::EstimatorTimestampsMonotonic).
fn stamp_estimators(bodies: &mut [Body], t: f32) {
    let now_ms = if t <= 0.0 { 0 } else { (t * 1000.0) as u64 };
    for b in bodies {
        let delayed = now_ms.saturating_sub(u64::from(b.imu_delay_ms));
        b.last_estimator_ts_ms = b.last_estimator_ts_ms.max(delayed);
    }
}

fn resolve_body_contacts(bodies: &mut [Body], hits: &mut Vec<SphereHit>, reset_impulse: bool) {
    if reset_impulse {
        for b in bodies.iter_mut() {
            b.last_sphere_impulse = 0.0;
            b.last_tangent_impulse = 0.0;
        }
        hits.clear();
    }
    let n = bodies.len();
    let sweeps = n.max(1);
    for _ in 0..sweeps {
        for i in 0..n {
            for j in (i + 1)..n {
                let (left, right) = bodies.split_at_mut(j);
                let a = &mut left[i];
                let b = &mut right[0];
                let after = resolve_sphere_contact(SphereContact::pair(a.sphere(), b.sphere()));
                let friction = apply_sphere_friction(
                    after,
                    SphereSpin::new(a.omega_body, a.inertia_diag[0]),
                    SphereSpin::new(b.omega_body, b.inertia_diag[0]),
                    SPHERE_FRICTION_MU,
                );
                a.apply_sphere(friction.contact.a);
                b.apply_sphere(friction.contact.b);
                a.omega_body = friction.a.omega;
                b.omega_body = friction.b.omega;
                a.last_sphere_impulse = a.last_sphere_impulse.max(friction.contact.impulse);
                b.last_sphere_impulse = b.last_sphere_impulse.max(friction.contact.impulse);
                a.last_tangent_impulse = a.last_tangent_impulse.max(friction.tangent_impulse);
                b.last_tangent_impulse = b.last_tangent_impulse.max(friction.tangent_impulse);
                accumulate_hit(
                    hits,
                    a.id,
                    b.id,
                    friction.contact.impulse,
                    friction.tangent_impulse,
                );
            }
        }
    }
}

fn accumulate_hit(
    hits: &mut Vec<SphereHit>,
    id_a: &'static str,
    id_b: &'static str,
    jn: f32,
    jt: f32,
) {
    if jn <= 1e-6 {
        return;
    }
    let hit = SphereHit::pair(id_a, id_b, jn, jt);
    if let Some(existing) = hits.iter_mut().find(|h| h.a == hit.a && h.b == hit.b) {
        if hit.jn > existing.jn {
            *existing = hit;
        }
    } else {
        hits.push(hit);
    }
}

fn apply_vertical_contact(env: &Environment, b: &mut Body) {
    let terrain_z = env.terrain_z(b.position_m[0], b.position_m[1]);
    let before = VerticalContact {
        z: b.position_m[2],
        vz: b.velocity_mps[2],
        terrain_z,
        impulse: 0.0,
    };
    let after = resolve_vertical_contact(before);
    b.position_m[2] = after.z;
    b.velocity_mps[2] = after.vz;
    b.last_contact_before = before;
    b.last_contact = after;

    let on_contact = (after.z - terrain_z).abs() < 1e-3;
    if on_contact {
        let granted = b.propulsion_live();
        match b.domain {
            Domain::Ground => {
                let damp = if granted { 0.97 } else { 0.35 };
                b.velocity_mps[0] *= damp;
                b.velocity_mps[1] *= damp;
            }
            Domain::Aerial if !granted => {
                b.velocity_mps[0] *= 0.65;
                b.velocity_mps[1] *= 0.65;
            }
            _ => {}
        }
    }
}

fn clamp3(t: &mut [f32; 3], limit: f32) {
    for slot in t.iter_mut() {
        *slot = slot.clamp(-limit, limit);
    }
}

fn wrap_pi(a: f32) -> f32 {
    let pi = core::f32::consts::PI;
    let mut x = (a + pi) % (2.0 * pi);
    if x < 0.0 {
        x += 2.0 * pi;
    }
    x - pi
}

fn yaw_from_quat(q: [f32; 4]) -> f32 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    wrap_pi((2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)))
}
