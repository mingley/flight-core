//! Mechanical primitives: terrain, sphere contact, drag, buoyancy, energy.
//!
//! These are the facts a simulator must keep true after every step:
//!
//! ```text
//! terrain impulse  ⇒  body was at or below the terrain
//! after terrain    ⇒  z ≤ terrain
//! on the plane     ⇒  z + ε ≥ terrain (pad / ground / seabed)
//! sphere impulse   ⇒  bodies were not strictly separated
//! after spheres    ⇒  |p_a − p_b| ≥ r_a + r_b
//! Coulomb friction ⇒  |j_t| ≤ μ j_n
//! quadratic drag   ⇒  F · v_rel ≤ 0
//! position hold    ⇒  command · (hold − pose) ≥ 0
//! buoyancy         ⇒  0 when dry
//! thrust           ⇒  0 unless the domain machine granted actuation
//! ground drive     ⇒  0 unless the hull is on the terrain plane
//! marine thrust    ⇒  0 unless the hull is in water
//! aerial thrust    ⇒  0 unless the rotors are in air
//! unit quaternion  ⇒  |q| ≈ 1 after a rigid-spin step
//! body→NED rotate  ⇒  |R v| = |v|
//! shallow water    ⇒  h ≥ 0, land dry, volume conserved
//! ```

use crate::frames::Ned;
use crate::vector::{Force, Vector3, Velocity};

/// Packed vertical contact (NED z-down). Terrain is a plane at `terrain_z`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalContact {
    pub z: f32,
    pub vz: f32,
    pub terrain_z: f32,
    pub impulse: f32,
}

impl VerticalContact {
    pub const fn airborne(z: f32, vz: f32, terrain_z: f32) -> Self {
        Self {
            z,
            vz,
            terrain_z,
            impulse: 0.0,
        }
    }

    pub fn penetrating(self) -> bool {
        self.z > self.terrain_z
    }

    /// On or within 0.1 mm of the terrain plane (NED z-down). After
    /// [`resolve_vertical_contact`], this is resting on the pad or seabed.
    pub fn on_plane(self) -> bool {
        self.z + 1e-4 >= self.terrain_z
    }
}

/// Project out of the terrain and kill downward velocity.
///
/// `impulse == 0` whenever the body started strictly above the surface.
pub fn resolve_vertical_contact(mut c: VerticalContact) -> VerticalContact {
    if !c.z.is_finite() || !c.vz.is_finite() || !c.terrain_z.is_finite() {
        return c;
    }
    if c.z < c.terrain_z {
        c.impulse = 0.0;
        return c;
    }
    c.z = c.terrain_z;
    if c.vz > 0.0 {
        c.impulse = c.vz;
        c.vz = 0.0;
    } else {
        c.impulse = 0.0;
    }
    c
}

pub fn contact_invariants(before: VerticalContact, after: VerticalContact) -> bool {
    if after.z > after.terrain_z + 1e-6 {
        return false;
    }
    if before.z < before.terrain_z && after.impulse != 0.0 {
        return false;
    }
    if after.impulse < 0.0 {
        return false;
    }
    true
}

/// One side of a packed sphere–sphere contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereBody {
    pub p: [f32; 3],
    pub v: [f32; 3],
    pub r: f32,
    pub m: f32,
}

impl SphereBody {
    pub const fn new(p: [f32; 3], v: [f32; 3], r: f32, m: f32) -> Self {
        Self { p, v, r, m }
    }
}

/// Packed sphere–sphere contact. `impulse` is the non-negative scalar along the
/// unit normal from A toward B. Zero when the spheres started strictly apart.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereContact {
    pub a: SphereBody,
    pub b: SphereBody,
    pub impulse: f32,
}

impl SphereContact {
    pub const fn pair(a: SphereBody, b: SphereBody) -> Self {
        Self { a, b, impulse: 0.0 }
    }

    /// Signed gap: positive means strictly separated.
    pub fn gap(self) -> f32 {
        dist3(self.a.p, self.b.p) - (self.a.r.max(0.0) + self.b.r.max(0.0))
    }
}

/// Project overlapping spheres apart and kill approaching relative speed.
///
/// `impulse == 0` whenever the spheres started strictly separated. Equal-mass
/// corrections preserve the center of mass when both masses are positive.
pub fn resolve_sphere_contact(mut c: SphereContact) -> SphereContact {
    if !triple_finite(c.a.p)
        || !triple_finite(c.b.p)
        || !triple_finite(c.a.v)
        || !triple_finite(c.b.v)
        || !c.a.r.is_finite()
        || !c.b.r.is_finite()
        || !c.a.m.is_finite()
        || !c.b.m.is_finite()
    {
        return c;
    }
    let ra = c.a.r.max(0.0);
    let rb = c.b.r.max(0.0);
    let min_d = ra + rb;
    let dx = c.b.p[0] - c.a.p[0];
    let dy = c.b.p[1] - c.a.p[1];
    let dz = c.b.p[2] - c.a.p[2];
    let dist = crate::math::sqrtf(dx * dx + dy * dy + dz * dz);
    if dist > min_d + 1e-6 {
        c.impulse = 0.0;
        return c;
    }

    let (nx, ny, nz) = if dist < 1e-8 {
        (1.0, 0.0, 0.0)
    } else {
        (dx / dist, dy / dist, dz / dist)
    };

    let inv_a = if c.a.m > 1e-9 { 1.0 / c.a.m } else { 0.0 };
    let inv_b = if c.b.m > 1e-9 { 1.0 / c.b.m } else { 0.0 };
    let inv_sum = inv_a + inv_b;
    if inv_sum < 1e-12 {
        c.impulse = 0.0;
        return c;
    }

    let penetration = min_d - dist;
    if penetration > 0.0 {
        let corr_a = penetration * (inv_a / inv_sum);
        let corr_b = penetration * (inv_b / inv_sum);
        c.a.p[0] -= nx * corr_a;
        c.a.p[1] -= ny * corr_a;
        c.a.p[2] -= nz * corr_a;
        c.b.p[0] += nx * corr_b;
        c.b.p[1] += ny * corr_b;
        c.b.p[2] += nz * corr_b;
    }

    let rvx = c.b.v[0] - c.a.v[0];
    let rvy = c.b.v[1] - c.a.v[1];
    let rvz = c.b.v[2] - c.a.v[2];
    let v_rel = rvx * nx + rvy * ny + rvz * nz;
    if v_rel >= 0.0 {
        c.impulse = 0.0;
        return c;
    }
    let j = -v_rel / inv_sum;
    c.impulse = j;
    c.a.v[0] -= j * inv_a * nx;
    c.a.v[1] -= j * inv_a * ny;
    c.a.v[2] -= j * inv_a * nz;
    c.b.v[0] += j * inv_b * nx;
    c.b.v[1] += j * inv_b * ny;
    c.b.v[2] += j * inv_b * nz;
    c
}

/// After resolve: no overlap, non-negative impulse, and no impulse if the
/// spheres started strictly apart.
pub fn sphere_contact_invariants(before: SphereContact, after: SphereContact) -> bool {
    if after.impulse < 0.0 {
        return false;
    }
    let min_before = before.a.r.max(0.0) + before.b.r.max(0.0);
    if dist3(before.a.p, before.b.p) > min_before + 1e-6 && after.impulse != 0.0 {
        return false;
    }
    let min_after = after.a.r.max(0.0) + after.b.r.max(0.0);
    if dist3(after.a.p, after.b.p) + 1e-4 < min_after {
        return false;
    }
    if before.a.m > 1e-6 && before.b.m > 1e-6 {
        let com0 = com3(before.a.p, before.a.m, before.b.p, before.b.m);
        let com1 = com3(after.a.p, after.a.m, after.b.p, after.b.m);
        if com0
            .iter()
            .zip(com1.iter())
            .any(|(u, v)| (u - v).abs() > 1e-4)
        {
            return false;
        }
    }
    true
}

/// Default Coulomb coefficient used by the verified world.
pub const SPHERE_FRICTION_MU: f32 = 0.4;

/// Angular velocity and isotropic inertia for frictional sphere contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereSpin {
    pub omega: [f32; 3],
    pub inertia: f32,
}

impl SphereSpin {
    pub const fn new(omega: [f32; 3], inertia: f32) -> Self {
        Self { omega, inertia }
    }
}

/// Normal contact plus the spin state after a Coulomb step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereFriction {
    pub contact: SphereContact,
    pub a: SphereSpin,
    pub b: SphereSpin,
    pub tangent_impulse: f32,
}

/// Tangential impulse at the contact point, clamped by `μ j_n`.
///
/// Frictionless spheres produce no torque (the normal is radial). This kernel
/// is the missing 6-DOF piece: a tangent impulse `r × J_t` spins both bodies
/// and cannot exceed Coulomb's cone. `μ == 0` is a no-op.
pub fn apply_sphere_friction(
    contact: SphereContact,
    spin_a: SphereSpin,
    spin_b: SphereSpin,
    mu: f32,
) -> SphereFriction {
    let mut out = SphereFriction {
        contact,
        a: spin_a,
        b: spin_b,
        tangent_impulse: 0.0,
    };
    if !(mu.is_finite() && mu >= 0.0) || contact.impulse <= 1e-9 || mu < 1e-8 {
        return out;
    }
    if !triple_finite(spin_a.omega)
        || !triple_finite(spin_b.omega)
        || !spin_a.inertia.is_finite()
        || !spin_b.inertia.is_finite()
    {
        return out;
    }

    let n = vec_norm([
        contact.b.p[0] - contact.a.p[0],
        contact.b.p[1] - contact.a.p[1],
        contact.b.p[2] - contact.a.p[2],
    ]);
    if vec_dot(n, n) < 0.5 {
        return out;
    }
    let ra = contact.a.r.max(0.0);
    let rb = contact.b.r.max(0.0);
    let r_a = [n[0] * ra, n[1] * ra, n[2] * ra];
    let r_b = [-n[0] * rb, -n[1] * rb, -n[2] * rb];
    let wa_r = vec_cross(spin_a.omega, r_a);
    let wb_r = vec_cross(spin_b.omega, r_b);
    let v_rel = [
        (contact.b.v[0] + wb_r[0]) - (contact.a.v[0] + wa_r[0]),
        (contact.b.v[1] + wb_r[1]) - (contact.a.v[1] + wa_r[1]),
        (contact.b.v[2] + wb_r[2]) - (contact.a.v[2] + wa_r[2]),
    ];
    let vn = vec_dot(v_rel, n);
    let v_t = [
        v_rel[0] - n[0] * vn,
        v_rel[1] - n[1] * vn,
        v_rel[2] - n[2] * vn,
    ];
    let vt_mag = crate::math::sqrtf(vec_dot(v_t, v_t));
    if vt_mag < 1e-8 {
        return out;
    }
    let t = [v_t[0] / vt_mag, v_t[1] / vt_mag, v_t[2] / vt_mag];
    let inv_a = if contact.a.m > 1e-9 {
        1.0 / contact.a.m
    } else {
        0.0
    };
    let inv_b = if contact.b.m > 1e-9 {
        1.0 / contact.b.m
    } else {
        0.0
    };
    let ia = spin_a.inertia.max(1e-9);
    let ib = spin_b.inertia.max(1e-9);
    let k = inv_a + inv_b + (ra * ra) / ia + (rb * rb) / ib;
    if k < 1e-12 {
        return out;
    }
    let j = (vt_mag / k).min(mu * contact.impulse);
    out.tangent_impulse = j;
    out.contact.a.v[0] += j * inv_a * t[0];
    out.contact.a.v[1] += j * inv_a * t[1];
    out.contact.a.v[2] += j * inv_a * t[2];
    out.contact.b.v[0] -= j * inv_b * t[0];
    out.contact.b.v[1] -= j * inv_b * t[1];
    out.contact.b.v[2] -= j * inv_b * t[2];
    let ja = [j * t[0], j * t[1], j * t[2]];
    let jb = [-ja[0], -ja[1], -ja[2]];
    let tau_a = vec_cross(r_a, ja);
    let tau_b = vec_cross(r_b, jb);
    out.a.omega = [
        spin_a.omega[0] + tau_a[0] / ia,
        spin_a.omega[1] + tau_a[1] / ia,
        spin_a.omega[2] + tau_a[2] / ia,
    ];
    out.b.omega = [
        spin_b.omega[0] + tau_b[0] / ib,
        spin_b.omega[1] + tau_b[1] / ib,
        spin_b.omega[2] + tau_b[2] / ib,
    ];
    out
}

/// Coulomb cone: `0 ≤ j_t ≤ μ j_n`, and ω stays finite.
pub fn friction_invariants(mu: f32, j_n: f32, after: SphereFriction) -> bool {
    if !after.tangent_impulse.is_finite() || after.tangent_impulse < -1e-6 {
        return false;
    }
    if after.tangent_impulse > mu.max(0.0) * j_n.max(0.0) + 1e-4 {
        return false;
    }
    triple_finite(after.a.omega) && triple_finite(after.b.omega)
}

fn dist3(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    crate::math::sqrtf(dx * dx + dy * dy + dz * dz)
}

fn com3(a: [f32; 3], ma: f32, b: [f32; 3], mb: f32) -> [f32; 3] {
    let s = ma + mb;
    if s <= 0.0 {
        return [0.0, 0.0, 0.0];
    }
    [
        (ma * a[0] + mb * b[0]) / s,
        (ma * a[1] + mb * b[1]) / s,
        (ma * a[2] + mb * b[2]) / s,
    ]
}

fn triple_finite(v: [f32; 3]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite()
}

/// Quadratic drag. Always opposes relative flow when coefficients are non-negative.
pub fn quadratic_drag(v_rel: [f32; 3], rho: f32, cd: f32, area: f32) -> [f32; 3] {
    if rho < 0.0 || cd < 0.0 || area < 0.0 {
        return [0.0, 0.0, 0.0];
    }
    let speed2 = v_rel[0] * v_rel[0] + v_rel[1] * v_rel[1] + v_rel[2] * v_rel[2];
    if speed2 < 1e-12 {
        return [0.0, 0.0, 0.0];
    }
    let speed = crate::math::sqrtf(speed2);
    let k = -0.5 * rho * cd * area * speed;
    [k * v_rel[0], k * v_rel[1], k * v_rel[2]]
}

pub fn drag_opposes_flow(v_rel: [f32; 3], f: [f32; 3]) -> bool {
    let dot = v_rel[0] * f[0] + v_rel[1] * f[1] + v_rel[2] * f[2];
    dot <= 1e-5
}

/// P-gain the plant uses for [`hold_velocity_ned`].
pub const HOLD_KP: f32 = 1.4;

/// Restoring NED velocity for a position hold.
///
/// When `kp ≥ 0` and every component is finite, the command has the same
/// sign as the pose error (`hold − position`) on each axis.
/// Creusot 0.5 pearlite cannot state that postcondition (`f32` has no
/// `OrdLogic`; float literals ICE). Kani `hold_velocity_restores_pose` does.
pub fn hold_velocity_ned(hold: [f32; 3], position: [f32; 3], kp: f32) -> [f32; 3] {
    if !(kp.is_finite() && kp >= 0.0 && triple_finite(hold) && triple_finite(position)) {
        return [0.0, 0.0, 0.0];
    }
    [
        kp * (hold[0] - position[0]),
        kp * (hold[1] - position[1]),
        kp * (hold[2] - position[2]),
    ]
}

/// Command · pose error is non-negative: the hold never drives away.
pub fn hold_restores_pose(hold: [f32; 3], position: [f32; 3], cmd: [f32; 3]) -> bool {
    if !triple_finite(hold) || !triple_finite(position) || !triple_finite(cmd) {
        return false;
    }
    let dot = (hold[0] - position[0]) * cmd[0]
        + (hold[1] - position[1]) * cmd[1]
        + (hold[2] - position[2]) * cmd[2];
    dot >= -1e-5
}

/// Hydrostatic buoyancy in NED (negative z is up). Zero when dry.
pub fn buoyancy_ned(displaced_m3: f32, density: f32, gravity: f32) -> f32 {
    if displaced_m3 <= 0.0 || density <= 0.0 || gravity <= 0.0 {
        return 0.0;
    }
    -displaced_m3 * density * gravity
}

pub fn buoyancy_only_when_wet(displaced_m3: f32, force_z_ned: f32) -> bool {
    if displaced_m3 <= 0.0 {
        force_z_ned.abs() < 1e-9
    } else {
        force_z_ned <= 0.0
    }
}

/// Translational kinetic energy. Must stay finite.
pub fn kinetic_energy(mass_kg: f32, v: [f32; 3]) -> f32 {
    if mass_kg < 0.0 {
        return 0.0;
    }
    0.5 * mass_kg * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Gravitational potential in NED (z positive down): `U = −m g z`.
/// Climbing (decreasing z) stores energy; falling releases it.
pub fn gravitational_pe_ned(mass_kg: f32, z: f32, gravity: f32) -> f32 {
    if mass_kg < 0.0 || !gravity.is_finite() || gravity < 0.0 {
        return 0.0;
    }
    -mass_kg * gravity * z
}

/// KE + gravitational PE. Buoyancy and drag are accounted separately.
pub fn mechanical_energy(mass_kg: f32, z: f32, v: [f32; 3], gravity: f32) -> f32 {
    kinetic_energy(mass_kg, v) + gravitational_pe_ned(mass_kg, z, gravity)
}

/// Instantaneous power of a force in the relative-flow frame: `F · v_rel`.
pub fn relative_power(v_rel: [f32; 3], f: [f32; 3]) -> f32 {
    v_rel[0] * f[0] + v_rel[1] * f[1] + v_rel[2] * f[2]
}

/// Actuator force is identically zero unless the safety machine granted it.
pub fn thrust_only_when_granted(granted: bool, thrust: [f32; 3]) -> bool {
    if !thrust.iter().all(|c| c.is_finite()) {
        return false;
    }
    granted || thrust.iter().all(|c| c.abs() < 1e-9)
}

/// Ground actuator force is identically zero unless the hull is on the terrain plane.
pub fn ground_thrust_only_on_contact(on_terrain: bool, thrust: [f32; 3]) -> bool {
    if !thrust.iter().all(|c| c.is_finite()) {
        return false;
    }
    on_terrain || thrust.iter().all(|c| c.abs() < 1e-9)
}

/// Marine actuator force is identically zero unless the hull is in water.
pub fn marine_thrust_only_when_wet(wet: bool, thrust: [f32; 3]) -> bool {
    if !thrust.iter().all(|c| c.is_finite()) {
        return false;
    }
    wet || thrust.iter().all(|c| c.abs() < 1e-9)
}

/// Aerial actuator force is identically zero unless the rotors are in air.
pub fn aerial_thrust_only_in_air(in_air: bool, thrust: [f32; 3]) -> bool {
    if !thrust.iter().all(|c| c.is_finite()) {
        return false;
    }
    in_air || thrust.iter().all(|c| c.abs() < 1e-9)
}

/// An empty energy pack cannot produce actuator force.
pub fn battery_gates_thrust(charge_j: f32, thrust: [f32; 3]) -> bool {
    if !charge_j.is_finite() || !thrust.iter().all(|c| c.is_finite()) {
        return false;
    }
    charge_j > 0.0 || thrust.iter().all(|c| c.abs() < 1e-9)
}

/// Subtract propulsion work. Never returns a negative charge.
pub fn drain_from_thrust(charge_j: f32, thrust: [f32; 3], dt: f32, watts_per_newton: f32) -> f32 {
    if !charge_j.is_finite() || charge_j <= 0.0 {
        return 0.0;
    }
    if !(dt.is_finite() && dt > 0.0) || watts_per_newton < 0.0 {
        return charge_j;
    }
    let mag2 = thrust[0] * thrust[0] + thrust[1] * thrust[1] + thrust[2] * thrust[2];
    let mag = crate::math::sqrtf(mag2);
    (charge_j - mag * watts_per_newton * dt).max(0.0)
}

pub fn mechanics_finite(mass_kg: f32, z: f32, v: [f32; 3], yaw_rate: f32) -> bool {
    mass_kg.is_finite()
        && mass_kg > 0.0
        && z.is_finite()
        && v.iter().all(|c| c.is_finite())
        && yaw_rate.is_finite()
        && kinetic_energy(mass_kg, v).is_finite()
        && gravitational_pe_ned(mass_kg, z, 9.80665).is_finite()
}

/// Principal-axis rotational kinetic energy. Negative inertia is treated as empty.
pub fn angular_kinetic_energy(i_diag: [f32; 3], omega: [f32; 3]) -> f32 {
    if i_diag.iter().any(|i| !i.is_finite() || *i < 0.0) || !triple_finite(omega) {
        return 0.0;
    }
    0.5 * (i_diag[0] * omega[0] * omega[0]
        + i_diag[1] * omega[1] * omega[1]
        + i_diag[2] * omega[2] * omega[2])
}

/// One explicit Euler step of Euler's equations on principal axes.
///
/// `I_i α_i = τ_i + (I_j − I_k) ω_j ω_k`. Non-finite or non-positive inertia
/// leaves `omega` unchanged.
pub fn euler_principal_step(
    omega: [f32; 3],
    torque: [f32; 3],
    i_diag: [f32; 3],
    dt: f32,
) -> [f32; 3] {
    if !(dt.is_finite()
        && dt > 0.0
        && dt < 1.0
        && triple_finite(omega)
        && triple_finite(torque)
        && i_diag.iter().all(|i| i.is_finite() && *i >= 1e-9))
    {
        return omega;
    }
    let (wx, wy, wz) = (omega[0], omega[1], omega[2]);
    let (ix, iy, iz) = (i_diag[0], i_diag[1], i_diag[2]);
    [
        wx + dt * (torque[0] + (iy - iz) * wy * wz) / ix,
        wy + dt * (torque[1] + (iz - ix) * wz * wx) / iy,
        wz + dt * (torque[2] + (ix - iy) * wx * wy) / iz,
    ]
}

/// Unit-quaternion attitude `[w, x, y, z]`. Integrate body rates, then renormalize.
pub fn quat_integrate(q: [f32; 4], omega_body: [f32; 3], dt: f32) -> [f32; 4] {
    if !(dt.is_finite() && dt > 0.0 && q.iter().all(|c| c.is_finite()) && triple_finite(omega_body))
    {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let wq = [0.0, omega_body[0], omega_body[1], omega_body[2]];
    let dq = quat_mul(q, wq);
    let n = [
        q[0] + 0.5 * dt * dq[0],
        q[1] + 0.5 * dt * dq[1],
        q[2] + 0.5 * dt * dq[2],
        q[3] + 0.5 * dt * dq[3],
    ];
    quat_renorm(n)
}

pub fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub fn quat_renorm(q: [f32; 4]) -> [f32; 4] {
    let mag2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    let mag = crate::math::sqrtf(mag2);
    if mag < 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / mag, q[1] / mag, q[2] / mag, q[3] / mag]
}

/// `|q| ≈ 1` and every component finite.
pub fn quat_is_unit(q: [f32; 4]) -> bool {
    if !q.iter().all(|c| c.is_finite()) {
        return false;
    }
    let mag2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    (mag2 - 1.0).abs() < 1e-3
}

pub fn quat_conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Rotate a body-frame vector into NED. `q` is the body→NED attitude.
pub fn quat_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    if !q.iter().all(|c| c.is_finite()) || !triple_finite(v) {
        return [0.0, 0.0, 0.0];
    }
    let qv = [0.0, v[0], v[1], v[2]];
    let r = quat_mul(quat_mul(q, qv), quat_conj(q));
    [r[1], r[2], r[3]]
}

/// Rotate a NED vector into the body frame.
pub fn quat_rotate_inv(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    quat_rotate(quat_conj(q), v)
}

pub fn vec_dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn vec_cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn vec_norm(v: [f32; 3]) -> [f32; 3] {
    let m = crate::math::sqrtf(vec_dot(v, v));
    if m < 1e-9 {
        return [0.0, 0.0, 0.0];
    }
    [v[0] / m, v[1] / m, v[2] / m]
}

pub fn rotation_preserves_length(before: [f32; 3], after: [f32; 3]) -> bool {
    if !triple_finite(before) || !triple_finite(after) {
        return false;
    }
    let a = crate::math::sqrtf(vec_dot(before, before));
    let b = crate::math::sqrtf(vec_dot(after, after));
    (a - b).abs() <= 1e-4 * (1.0 + a.max(b))
}

/// Collective along −body z, expressed in NED. Identity attitude → `[0, 0, −T]`.
pub fn body_z_thrust_ned(q: [f32; 4], collective: f32) -> [f32; 3] {
    quat_rotate(q, [0.0, 0.0, -collective])
}

/// Quadrotor fact: NED thrust is parallel to −body z (or is zero).
pub fn thrust_along_minus_body_z(q: [f32; 4], thrust_ned: [f32; 3]) -> bool {
    if !triple_finite(thrust_ned) || !q.iter().all(|c| c.is_finite()) {
        return false;
    }
    let mag_t = crate::math::sqrtf(vec_dot(thrust_ned, thrust_ned));
    if mag_t < 1e-6 {
        return true;
    }
    let axis = body_z_thrust_ned(q, 1.0);
    let mag_a = crate::math::sqrtf(vec_dot(axis, axis));
    if mag_a < 1e-6 {
        return false;
    }
    let c = vec_cross(thrust_ned, axis);
    let mag_c = crate::math::sqrtf(vec_dot(c, c));
    mag_c <= 1e-3 * mag_t * mag_a + 1e-5 && vec_dot(thrust_ned, axis) >= -1e-4 * mag_t * mag_a
}

/// Vectored AUV: saturate desired NED force on body axes, then rotate back.
///
/// Identity attitude is a per-axis clamp in NED. A roll lets a heave command
/// spend budget on sway in the world frame, which is what a body-fixed
/// thruster layout actually does.
pub fn body_axis_wrench(q: [f32; 4], f_ned: [f32; 3], axis_limit: f32) -> [f32; 3] {
    if !(axis_limit.is_finite() && axis_limit >= 0.0 && triple_finite(f_ned)) {
        return [0.0, 0.0, 0.0];
    }
    let mut f_body = quat_rotate_inv(q, f_ned);
    f_body[0] = f_body[0].clamp(-axis_limit, axis_limit);
    f_body[1] = f_body[1].clamp(-axis_limit, axis_limit);
    f_body[2] = f_body[2].clamp(-axis_limit, axis_limit);
    quat_rotate(q, f_body)
}

/// Body-frame components of `thrust_ned` each sit inside `±axis_limit`.
pub fn body_wrench_axes_limited(q: [f32; 4], thrust_ned: [f32; 3], axis_limit: f32) -> bool {
    if !triple_finite(thrust_ned) || !q.iter().all(|c| c.is_finite()) || !axis_limit.is_finite() {
        return false;
    }
    let fb = quat_rotate_inv(q, thrust_ned);
    fb.iter().all(|c| c.abs() <= axis_limit + 1e-3) && rotation_preserves_length(fb, thrust_ned)
}

/// Torque-free Euler + quaternion step stays finite with unit attitude.
pub fn rigid_spin_invariants(
    i_diag: [f32; 3],
    omega0: [f32; 3],
    omega1: [f32; 3],
    q1: [f32; 4],
) -> bool {
    if !triple_finite(omega1) || !quat_is_unit(q1) {
        return false;
    }
    angular_kinetic_energy(i_diag, omega0).is_finite()
        && angular_kinetic_energy(i_diag, omega1).is_finite()
}

pub fn force_ned(fx: f32, fy: f32, fz: f32) -> Force<Ned> {
    Vector3::new(fx, fy, fz)
}

pub fn velocity_ned(vn: f32, ve: f32, vd: f32) -> Velocity<Ned> {
    Velocity::ned(vn, ve, vd)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn airborne_gets_no_impulse() {
        let before = VerticalContact::airborne(-2.0, 0.5, 0.0);
        let after = resolve_vertical_contact(before);
        assert_eq!(after.impulse, 0.0);
        assert_eq!(after.z, -2.0);
        assert!(contact_invariants(before, after));
    }

    #[test]
    fn penetrating_is_projected() {
        let before = VerticalContact {
            z: 0.4,
            vz: 1.2,
            terrain_z: 0.0,
            impulse: 0.0,
        };
        let after = resolve_vertical_contact(before);
        assert_eq!(after.z, 0.0);
        assert_eq!(after.vz, 0.0);
        assert!(after.impulse > 0.0);
        assert!(after.on_plane());
        assert!(contact_invariants(before, after));
    }

    #[test]
    fn airborne_is_not_on_plane() {
        let after = resolve_vertical_contact(VerticalContact::airborne(-2.0, 0.5, 0.0));
        assert!(!after.on_plane());
        let pad = resolve_vertical_contact(VerticalContact::airborne(0.0, 0.0, 0.0));
        assert!(pad.on_plane());
    }

    #[test]
    fn drag_never_adds_energy() {
        let v = [1.0, -0.5, 0.2];
        let f = quadratic_drag(v, 1.225, 0.8, 0.4);
        assert!(drag_opposes_flow(v, f));
    }

    #[test]
    fn hold_command_restores_pose() {
        let hold = [1.0, -2.0, 0.5];
        let pos = [2.0, -2.0, 0.0];
        let cmd = hold_velocity_ned(hold, pos, HOLD_KP);
        assert!(hold_restores_pose(hold, pos, cmd));
        assert!(cmd[0] < 0.0);
        assert_eq!(cmd[1], 0.0);
        assert!(cmd[2] > 0.0);
        assert_eq!(hold_velocity_ned(hold, pos, -1.0), [0.0, 0.0, 0.0]);
        let parked = hold_velocity_ned(hold, hold, HOLD_KP);
        assert_eq!(parked, [0.0, 0.0, 0.0]);
        assert!(hold_restores_pose(hold, hold, parked));
    }

    #[test]
    fn dry_hull_has_no_buoyancy() {
        let f = buoyancy_ned(0.0, 1025.0, 9.81);
        assert_eq!(f, 0.0);
        assert!(buoyancy_only_when_wet(0.0, f));
    }

    #[test]
    fn wet_hull_lifts() {
        let f = buoyancy_ned(0.05, 1025.0, 9.81);
        assert!(f < 0.0);
        assert!(buoyancy_only_when_wet(0.05, f));
    }

    #[test]
    fn falling_converts_pe_to_ke() {
        let m = 2.0;
        let g = 9.81;
        let e0 = mechanical_energy(m, -10.0, [0.0, 0.0, 0.0], g);
        let dt = 0.5;
        let vz = g * dt;
        let z = -10.0 + 0.5 * g * dt * dt;
        let e1 = mechanical_energy(m, z, [0.0, 0.0, vz], g);
        assert!((e1 - e0).abs() < 1e-3, "{e0} -> {e1}");
    }

    #[test]
    fn ungranted_thrust_must_be_zero() {
        assert!(thrust_only_when_granted(false, [0.0, 0.0, 0.0]));
        assert!(!thrust_only_when_granted(false, [1.0, 0.0, 0.0]));
        assert!(thrust_only_when_granted(true, [4.0, 0.0, -9.81]));
    }

    #[test]
    fn airborne_ground_drive_must_be_zero() {
        assert!(ground_thrust_only_on_contact(true, [4.0, 0.0, 0.0]));
        assert!(ground_thrust_only_on_contact(false, [0.0, 0.0, 0.0]));
        assert!(!ground_thrust_only_on_contact(false, [1.0, 0.0, 0.0]));
    }

    #[test]
    fn dry_hull_marine_thrust_must_be_zero() {
        assert!(marine_thrust_only_when_wet(true, [4.0, 0.0, 0.0]));
        assert!(marine_thrust_only_when_wet(false, [0.0, 0.0, 0.0]));
        assert!(!marine_thrust_only_when_wet(false, [1.0, 0.0, 0.0]));
    }

    #[test]
    fn submerged_rotors_must_be_zero() {
        assert!(aerial_thrust_only_in_air(true, [0.0, 0.0, -9.81]));
        assert!(aerial_thrust_only_in_air(false, [0.0, 0.0, 0.0]));
        assert!(!aerial_thrust_only_in_air(false, [0.0, 0.0, -4.0]));
    }

    #[test]
    fn empty_battery_rejects_thrust() {
        assert!(battery_gates_thrust(0.0, [0.0, 0.0, 0.0]));
        assert!(!battery_gates_thrust(0.0, [2.0, 0.0, 0.0]));
        assert!(battery_gates_thrust(12.0, [2.0, 0.0, 0.0]));
    }

    #[test]
    fn drain_never_goes_negative() {
        let next = drain_from_thrust(0.5, [40.0, 0.0, 0.0], 1.0, 1.0);
        assert_eq!(next, 0.0);
        assert_eq!(drain_from_thrust(8.0, [0.0, 0.0, 0.0], 0.1, 1.0), 8.0);
        assert_eq!(drain_from_thrust(0.0, [10.0, 0.0, 0.0], 0.1, 1.0), 0.0);
    }

    #[test]
    fn torque_free_spin_keeps_energy() {
        let i = [2.0, 3.0, 4.0];
        let mut w = [0.4, -0.2, 0.7];
        let e0 = angular_kinetic_energy(i, w);
        let q0 = [1.0, 0.0, 0.0, 0.0];
        let mut q = q0;
        let dt = 0.002;
        for _ in 0..400 {
            w = euler_principal_step(w, [0.0, 0.0, 0.0], i, dt);
            q = quat_integrate(q, w, dt);
        }
        let e1 = angular_kinetic_energy(i, w);
        assert!((e1 - e0).abs() / e0.max(1e-6) < 0.02, "{e0} -> {e1}");
        assert!(quat_is_unit(q));
        assert!(rigid_spin_invariants(i, w, w, q));
    }

    #[test]
    fn yaw_rate_turns_heading() {
        let q = quat_integrate([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.5);
        assert!(quat_is_unit(q));
        // θ = 0.5 rad about +z → q ≈ [cos(θ/2), 0, 0, sin(θ/2)]
        let half = 0.25_f32;
        assert!((q[0] - half.cos()).abs() < 0.02);
        assert!(q[1].abs() < 0.02 && q[2].abs() < 0.02);
        assert!((q[3] - half.sin()).abs() < 0.02);
    }

    #[test]
    fn exhaustive_euler_stays_finite() {
        let inertias = [[1.0, 1.0, 1.0], [1.0, 2.0, 3.0]];
        let rates = [0.0, 0.5, -1.2];
        for i in inertias {
            for wx in rates {
                for wy in rates {
                    for wz in rates {
                        let w0 = [wx, wy, wz];
                        let w1 = euler_principal_step(w0, [0.0, 0.1, 0.0], i, 0.01);
                        let q1 = quat_integrate([1.0, 0.0, 0.0, 0.0], w1, 0.01);
                        assert!(rigid_spin_invariants(i, w0, w1, q1), "{w0:?} -> {w1:?}");
                    }
                }
            }
        }
    }

    #[test]
    fn relative_drag_power_is_nonpositive() {
        let v = [1.0, -0.5, 0.2];
        let f = quadratic_drag(v, 1.225, 0.8, 0.4);
        assert!(relative_power(v, f) <= 0.0);
    }

    #[test]
    fn exhaustive_contact_induction() {
        let zs = [-4.0, -0.1, 0.0, 0.05, 1.0];
        let vzs = [-2.0, -0.1, 0.0, 0.3, 2.0];
        let terrains = [-1.0, 0.0, 0.5];
        for z in zs {
            for vz in vzs {
                for terrain_z in terrains {
                    let before = VerticalContact {
                        z,
                        vz,
                        terrain_z,
                        impulse: 0.0,
                    };
                    let after = resolve_vertical_contact(before);
                    assert!(contact_invariants(before, after), "{before:?} -> {after:?}");
                }
            }
        }
    }

    #[test]
    fn separated_spheres_get_no_impulse() {
        let before = SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.4, 2.0),
            SphereBody::new([3.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.4, 2.0),
        );
        let after = resolve_sphere_contact(before);
        assert_eq!(after.impulse, 0.0);
        assert_eq!(after.a.p, before.a.p);
        assert!(sphere_contact_invariants(before, after));
    }

    #[test]
    fn overlapping_spheres_are_projected() {
        let before = SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], 0.5, 1.0),
            SphereBody::new([0.2, 0.0, 0.0], [-0.5, 0.0, 0.0], 0.5, 1.0),
        );
        let after = resolve_sphere_contact(before);
        assert!(after.gap() >= -1e-4, "gap {}", after.gap());
        assert!(after.impulse > 0.0);
        assert!(sphere_contact_invariants(before, after));
        let v_rel = after.b.v[0] - after.a.v[0];
        assert!(v_rel >= -1e-5, "still approaching {v_rel}");
    }

    #[test]
    fn zero_mu_leaves_spin_untouched() {
        let after = resolve_sphere_contact(SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [1.0, 0.2, 0.0], 0.5, 1.0),
            SphereBody::new([0.4, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.5, 1.0),
        ));
        let spin_a = SphereSpin::new([0.0, 0.0, 3.0], 0.1);
        let spin_b = SphereSpin::new([0.0, 0.0, 0.0], 0.1);
        let f = apply_sphere_friction(after, spin_a, spin_b, 0.0);
        assert_eq!(f.tangent_impulse, 0.0);
        assert_eq!(f.a.omega, spin_a.omega);
        assert!(friction_invariants(0.0, after.impulse, f));
    }

    #[test]
    fn glancing_hit_spins_and_stays_in_coulomb_cone() {
        let after = resolve_sphere_contact(SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [1.2, 0.8, 0.0], 0.5, 1.0),
            SphereBody::new([0.3, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5, 1.0),
        ));
        assert!(after.impulse > 0.0);
        let f = apply_sphere_friction(
            after,
            SphereSpin::new([0.0, 0.0, 0.0], 0.1),
            SphereSpin::new([0.0, 0.0, 0.0], 0.1),
            SPHERE_FRICTION_MU,
        );
        assert!(f.tangent_impulse > 0.0);
        assert!(f.a.omega[2].abs() > 1e-4 || f.b.omega[2].abs() > 1e-4);
        assert!(friction_invariants(SPHERE_FRICTION_MU, after.impulse, f));
        assert!(f.tangent_impulse <= SPHERE_FRICTION_MU * after.impulse + 1e-5);
    }

    #[test]
    fn exhaustive_friction_stays_in_cone() {
        let mus = [0.0, 0.2, 0.4, 1.0];
        let speeds = [-1.5, 0.0, 0.4, 1.2];
        for mu in mus {
            for vx in speeds {
                for vy in speeds {
                    let after = resolve_sphere_contact(SphereContact::pair(
                        SphereBody::new([0.0, 0.0, 0.0], [vx, vy, 0.0], 0.5, 1.0),
                        SphereBody::new([0.4, 0.0, 0.0], [-vx * 0.3, 0.0, 0.0], 0.5, 1.5),
                    ));
                    let f = apply_sphere_friction(
                        after,
                        SphereSpin::new([0.0, 0.0, vy], 0.08),
                        SphereSpin::new([0.0, 0.0, 0.0], 0.12),
                        mu,
                    );
                    assert!(
                        friction_invariants(mu, after.impulse, f),
                        "mu={mu} vx={vx} vy={vy} jn={} jt={}",
                        after.impulse,
                        f.tangent_impulse
                    );
                }
            }
        }
    }

    #[test]
    fn overlapping_but_separating_gets_no_velocity_impulse() {
        let before = SphereContact::pair(
            SphereBody::new([0.0, 0.0, 0.0], [-0.4, 0.0, 0.0], 0.5, 1.0),
            SphereBody::new([0.2, 0.0, 0.0], [0.4, 0.0, 0.0], 0.5, 1.0),
        );
        let after = resolve_sphere_contact(before);
        assert_eq!(after.impulse, 0.0);
        assert!(after.gap() >= -1e-4);
        assert!(sphere_contact_invariants(before, after));
    }

    #[test]
    fn exhaustive_sphere_induction() {
        let offsets = [0.0, 0.2, 0.5, 1.0, 2.5];
        let speeds = [-2.0, -0.2, 0.0, 0.3, 1.5];
        let radii = [0.2, 0.5];
        let masses = [1.0, 4.0];
        for d in offsets {
            for va in speeds {
                for vb in speeds {
                    for ra in radii {
                        for rb in radii {
                            for ma in masses {
                                for mb in masses {
                                    let before = SphereContact::pair(
                                        SphereBody::new([0.0, 0.0, 0.0], [va, 0.0, 0.0], ra, ma),
                                        SphereBody::new([d, 0.0, 0.0], [vb, 0.0, 0.0], rb, mb),
                                    );
                                    let after = resolve_sphere_contact(before);
                                    assert!(
                                        sphere_contact_invariants(before, after),
                                        "{before:?} -> {after:?}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn identity_quat_is_identity_rotate() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let v = [1.2, -0.4, 0.7];
        let r = quat_rotate(q, v);
        assert!((r[0] - v[0]).abs() < 1e-5);
        assert!((r[1] - v[1]).abs() < 1e-5);
        assert!((r[2] - v[2]).abs() < 1e-5);
        assert!(rotation_preserves_length(v, r));
        let t = body_z_thrust_ned(q, 9.81);
        assert!((t[0]).abs() < 1e-5 && t[1].abs() < 1e-5);
        assert!((t[2] + 9.81).abs() < 1e-4);
        assert!(thrust_along_minus_body_z(q, t));
    }

    #[test]
    fn yaw_quat_sends_north_to_east() {
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let q = [s, 0.0, 0.0, s];
        let r = quat_rotate(q, [1.0, 0.0, 0.0]);
        assert!(r[0].abs() < 0.02, "{r:?}");
        assert!((r[1] - 1.0).abs() < 0.02, "{r:?}");
        assert!(r[2].abs() < 0.02);
        assert!(rotation_preserves_length([1.0, 0.0, 0.0], r));
        let back = quat_rotate_inv(q, r);
        assert!((back[0] - 1.0).abs() < 0.02);
    }

    #[test]
    fn tilted_thrust_stays_on_minus_body_z() {
        let q = quat_integrate([1.0, 0.0, 0.0, 0.0], [0.0, 0.4, 0.0], 0.5);
        let t = body_z_thrust_ned(q, 4.0);
        assert!(thrust_along_minus_body_z(q, t));
        assert!(!thrust_along_minus_body_z(q, [4.0, 0.0, 0.0]));
    }

    #[test]
    fn body_axis_wrench_clamps_in_body_not_ned() {
        let q = quat_integrate([1.0, 0.0, 0.0, 0.0], [0.0, 0.8, 0.0], 0.6);
        let f_ned = [20.0, 0.0, 0.0];
        let out = body_axis_wrench(q, f_ned, 5.0);
        assert!(body_wrench_axes_limited(q, out, 5.0));
        let fb = quat_rotate_inv(q, out);
        assert!(fb[0].abs() <= 5.0 + 1e-4);
        assert!(fb[1].abs() <= 5.0 + 1e-4);
        assert!(fb[2].abs() <= 5.0 + 1e-4);
        let identity = body_axis_wrench([1.0, 0.0, 0.0, 0.0], [3.0, -9.0, 1.0], 4.0);
        assert!((identity[0] - 3.0).abs() < 1e-5);
        assert!((identity[1] + 4.0).abs() < 1e-5);
        assert!((identity[2] - 1.0).abs() < 1e-5);
    }
}
