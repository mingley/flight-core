//! A rigid body with a domain-tagged safety machine.

use crate::env::Environment;
use flight_core::domain::{Domain, Medium};
use flight_core::ground::GroundState;
use flight_core::marine::MarineState;
use flight_core::mech::VerticalContact;
use flight_core::safety::{self, Event, SafetyState};

/// Simulated platform. Safety bits decide whether actuator force is applied.
#[derive(Clone, Debug)]
pub struct Body {
    pub id: &'static str,
    pub domain: Domain,
    pub mass_kg: f32,
    pub cd: f32,
    pub area_m2: f32,
    pub hull_volume_m3: f32,
    pub draft_m: f32,
    /// Collision sphere radius. Pairwise contact uses this after integration.
    pub radius_m: f32,
    pub position_m: [f32; 3],
    pub velocity_mps: [f32; 3],
    pub yaw_rad: f32,
    pub yaw_rate: f32,
    /// NED velocity setpoint. Ignored unless the domain machine grants actuation.
    pub command: Option<[f32; 3]>,
    /// NED position hold. Each granted step refreshes [`Self::command`] with
    /// [`flight_core::mech::hold_velocity_ned`]. Velocity commands and failsafe
    /// clear this.
    pub hold_ned: Option<[f32; 3]>,
    pub yaw_cmd: f32,
    pub last_drag: [f32; 3],
    pub last_v_rel: [f32; 3],
    pub last_buoyancy_z: f32,
    pub last_displaced: f32,
    pub last_contact_before: VerticalContact,
    pub last_contact: VerticalContact,
    pub last_thrust: [f32; 3],
    pub last_drag_power: f32,
    pub last_ke: f32,
    pub last_pe: f32,
    pub last_sphere_impulse: f32,
    /// Peak Coulomb tangent impulse from the last contact sweep.
    pub last_tangent_impulse: f32,
    /// Stored energy. Propulsion is cut when this hits zero.
    pub capacity_j: f32,
    pub charge_j: f32,
    /// Charge at the start of the last step, used by `battery_gates_thrust`.
    pub last_charge_j: f32,
    /// Terrain support at thrust time, used by `ground_drive_only_on_contact`.
    pub last_on_terrain: bool,
    /// Water column at thrust time, used by `marine_thrust_only_when_wet`.
    pub last_wet: bool,
    /// Principal-axis inertia (body frame), kg·m².
    pub inertia_diag: [f32; 3],
    /// Body-frame angular velocity, rad/s.
    pub omega_body: [f32; 3],
    /// Physics-truth attitude quaternion `[w, x, y, z]` from
    /// `mech::quat_integrate`. Not [`flight_core::nav::ComplementaryAttitude`].
    pub quat: [f32; 4],
    pub last_angular_ke: f32,
    pub aerial: Option<SafetyState>,
    pub ground: Option<GroundState>,
    pub marine: Option<MarineState>,
    /// Revocation counter for actuation permits. Not a kernel packed bit.
    pub authority_epoch: u32,
    /// Plant IMU transport delay. Stamps lag wall time; not a property-vector field.
    pub imu_delay_ms: u32,
    /// Last estimator stamp (ms). Monotonic even if [`Self::imu_delay_ms`] jumps.
    pub last_estimator_ts_ms: u64,
    /// Motor efficiency in `[0, 1]`. Scales granted thrust before `last_thrust`.
    pub thrust_scale: f32,
}

impl Body {
    pub fn aerial_ready(id: &'static str) -> Self {
        let safety = safety::step_all(
            SafetyState::disconnected(),
            &[
                Event::Connect,
                Event::InitComplete,
                Event::Initialized,
                Event::ImuHealthy,
                Event::EstimatorValid,
                Event::PreflightPassed,
            ],
        )
        .expect("connect path is legal");
        Self {
            id,
            domain: Domain::Aerial,
            mass_kg: 1.5,
            cd: 0.8,
            area_m2: 0.12,
            hull_volume_m3: 0.002,
            draft_m: 0.15,
            radius_m: 0.35,
            position_m: [10.0, 0.0, 0.0],
            velocity_mps: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            yaw_rate: 0.0,
            command: None,
            hold_ned: None,
            yaw_cmd: 0.0,
            last_drag: [0.0; 3],
            last_v_rel: [0.0; 3],
            last_buoyancy_z: 0.0,
            last_displaced: 0.0,
            last_contact_before: VerticalContact::airborne(0.0, 0.0, 0.0),
            last_contact: VerticalContact::airborne(0.0, 0.0, 0.0),
            last_thrust: [0.0; 3],
            last_drag_power: 0.0,
            last_ke: 0.0,
            last_pe: 0.0,
            last_sphere_impulse: 0.0,
            last_tangent_impulse: 0.0,
            capacity_j: 4000.0,
            charge_j: 4000.0,
            last_charge_j: 4000.0,
            last_on_terrain: true,
            last_wet: false,
            inertia_diag: sphere_inertia(1.5, 0.35),
            omega_body: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            last_angular_ke: 0.0,
            aerial: Some(safety),
            ground: None,
            marine: None,
            authority_epoch: 0,
            imu_delay_ms: 0,
            last_estimator_ts_ms: 0,
            thrust_scale: 1.0,
        }
    }

    pub fn rover(id: &'static str) -> Self {
        Self {
            id,
            domain: Domain::Ground,
            mass_kg: 28.0,
            cd: 0.9,
            area_m2: 0.4,
            hull_volume_m3: 0.0,
            draft_m: 0.2,
            radius_m: 0.55,
            position_m: [14.0, 3.0, 0.0],
            velocity_mps: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            yaw_rate: 0.0,
            command: None,
            hold_ned: None,
            yaw_cmd: 0.0,
            last_drag: [0.0; 3],
            last_v_rel: [0.0; 3],
            last_buoyancy_z: 0.0,
            last_displaced: 0.0,
            last_contact_before: VerticalContact::airborne(0.0, 0.0, 0.0),
            last_contact: VerticalContact::airborne(0.0, 0.0, 0.0),
            last_thrust: [0.0; 3],
            last_drag_power: 0.0,
            last_ke: 0.0,
            last_pe: 0.0,
            last_sphere_impulse: 0.0,
            last_tangent_impulse: 0.0,
            capacity_j: 20000.0,
            charge_j: 20000.0,
            last_charge_j: 20000.0,
            last_on_terrain: true,
            last_wet: false,
            inertia_diag: sphere_inertia(28.0, 0.55),
            omega_body: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            last_angular_ke: 0.0,
            aerial: None,
            ground: Some(GroundState::parked()),
            marine: None,
            authority_epoch: 0,
            imu_delay_ms: 0,
            last_estimator_ts_ms: 0,
            thrust_scale: 1.0,
        }
    }

    pub fn skiff(id: &'static str) -> Self {
        Self {
            id,
            domain: Domain::Surface,
            mass_kg: 80.0,
            cd: 0.55,
            area_m2: 0.8,
            hull_volume_m3: 0.25,
            draft_m: 0.5,
            radius_m: 1.1,
            position_m: [-6.0, -2.0, 0.12],
            velocity_mps: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            yaw_rate: 0.0,
            command: None,
            hold_ned: None,
            yaw_cmd: 0.0,
            last_drag: [0.0; 3],
            last_v_rel: [0.0; 3],
            last_buoyancy_z: 0.0,
            last_displaced: 0.0,
            last_contact_before: VerticalContact::airborne(0.12, 0.0, 4.0),
            last_contact: VerticalContact::airborne(0.12, 0.0, 4.0),
            last_thrust: [0.0; 3],
            last_drag_power: 0.0,
            last_ke: 0.0,
            last_pe: 0.0,
            last_sphere_impulse: 0.0,
            last_tangent_impulse: 0.0,
            capacity_j: 40000.0,
            charge_j: 40000.0,
            last_charge_j: 40000.0,
            last_on_terrain: false,
            last_wet: true,
            inertia_diag: sphere_inertia(80.0, 1.1),
            omega_body: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            last_angular_ke: 0.0,
            aerial: None,
            ground: None,
            marine: Some(flight_core::marine::MarineState::docked()),
            authority_epoch: 0,
            imu_delay_ms: 0,
            last_estimator_ts_ms: 0,
            thrust_scale: 1.0,
        }
    }

    pub fn surveyor(id: &'static str) -> Self {
        let mass = 18.0;
        let rho = 1025.0;
        Self {
            id,
            domain: Domain::Underwater,
            mass_kg: mass,
            cd: 0.4,
            area_m2: 0.08,
            hull_volume_m3: mass / rho,
            draft_m: 0.3,
            radius_m: 0.45,
            position_m: [-10.0, 4.0, 2.0],
            velocity_mps: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            yaw_rate: 0.0,
            command: None,
            hold_ned: None,
            yaw_cmd: 0.0,
            last_drag: [0.0; 3],
            last_v_rel: [0.0; 3],
            last_buoyancy_z: 0.0,
            last_displaced: 0.0,
            last_contact_before: VerticalContact::airborne(2.0, 0.0, 4.0),
            last_contact: VerticalContact::airborne(2.0, 0.0, 4.0),
            last_thrust: [0.0; 3],
            last_drag_power: 0.0,
            last_ke: 0.0,
            last_pe: 0.0,
            last_sphere_impulse: 0.0,
            last_tangent_impulse: 0.0,
            capacity_j: 8000.0,
            charge_j: 8000.0,
            last_charge_j: 8000.0,
            last_on_terrain: false,
            last_wet: true,
            inertia_diag: sphere_inertia(mass, 0.45),
            omega_body: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            last_angular_ke: 0.0,
            aerial: None,
            ground: None,
            marine: Some(flight_core::marine::MarineState::docked()),
            authority_epoch: 0,
            imu_delay_ms: 0,
            last_estimator_ts_ms: 0,
            thrust_scale: 1.0,
        }
    }

    pub fn bump_authority(&mut self) {
        self.authority_epoch = self.authority_epoch.saturating_add(1);
    }

    pub fn sphere(&self) -> flight_core::mech::SphereBody {
        flight_core::mech::SphereBody::new(
            self.position_m,
            self.velocity_mps,
            self.radius_m,
            self.mass_kg,
        )
    }

    pub fn apply_sphere(&mut self, s: flight_core::mech::SphereBody) {
        self.position_m = s.p;
        self.velocity_mps = s.v;
    }

    /// Drop the actuator command **and** the pose hold. Ungranted aerial
    /// (wet rotors, empty battery, failsafe) must not keep a NED target.
    /// Remaining-spec §5.4 choice A: wipe, do not persist `hold_ned`.
    pub fn clear_command(&mut self) {
        self.command = None;
        self.hold_ned = None;
    }

    pub fn set_velocity_command(&mut self, v: [f32; 3]) {
        self.hold_ned = None;
        self.command = Some(v);
    }

    pub fn set_position_hold(&mut self, p: [f32; 3]) {
        self.hold_ned = Some(p);
        self.refresh_hold();
    }

    /// Rewrite [`Self::command`] from [`Self::hold_ned`]. No-op when idle.
    pub fn refresh_hold(&mut self) {
        let Some(p) = self.hold_ned else {
            return;
        };
        self.command = Some(flight_core::mech::hold_velocity_ned(
            p,
            self.position_m,
            flight_core::mech::HOLD_KP,
        ));
    }

    pub fn actuators_granted(&self) -> bool {
        match self.domain {
            Domain::Aerial => self
                .aerial
                .map(|s| s.actuators_enabled && s.armed && !s.failsafe)
                .unwrap_or(false),
            Domain::Ground => self
                .ground
                .map(|s| s.drive_enabled && !s.estop)
                .unwrap_or(false),
            Domain::Surface | Domain::Underwater => self
                .marine
                .map(|s| s.thrust_enabled && !s.failsafe)
                .unwrap_or(false),
        }
    }

    /// Safety grant plus a non-empty energy pack.
    pub fn propulsion_live(&self) -> bool {
        self.actuators_granted() && self.charge_j > 0.0
    }

    pub fn phase_name(&self) -> &'static str {
        match self.domain {
            Domain::Aerial => self.aerial.map(|s| s.phase.name()).unwrap_or("none"),
            Domain::Ground => self.ground.map(|s| s.phase.name()).unwrap_or("none"),
            Domain::Surface | Domain::Underwater => {
                self.marine.map(|s| s.phase.name()).unwrap_or("none")
            }
        }
    }

    pub fn failsafe(&self) -> bool {
        match self.domain {
            Domain::Aerial => self.aerial.map(|s| s.failsafe).unwrap_or(false),
            Domain::Ground => self.ground.map(|s| s.estop).unwrap_or(false),
            Domain::Surface | Domain::Underwater => {
                self.marine.map(|s| s.failsafe).unwrap_or(false)
            }
        }
    }

    pub fn armed_like(&self) -> bool {
        match self.domain {
            Domain::Aerial => self.aerial.map(|s| s.armed).unwrap_or(false),
            Domain::Ground => self.ground.map(|s| s.drive_enabled).unwrap_or(false),
            Domain::Surface | Domain::Underwater => {
                self.marine.map(|s| s.thrust_enabled).unwrap_or(false)
            }
        }
    }

    pub fn medium(&self, env: &Environment, t: f32) -> Medium {
        env.medium_at_time(self.position_m[0], self.position_m[2], t)
    }

    pub fn medium_in(&self, hydro: &crate::hydro::HydroField, env: &Environment) -> Medium {
        hydro.medium_at(
            self.position_m[0],
            self.position_m[1],
            self.position_m[2],
            env.waterline_z,
        )
    }

    pub fn altitude_agl(&self) -> f32 {
        -self.position_m[2]
    }

    /// Current NED z sits on the terrain plane (pad, ground, or seabed).
    pub fn on_terrain(&self, env: &Environment) -> bool {
        VerticalContact::airborne(
            self.position_m[2],
            self.velocity_mps[2],
            env.terrain_z(self.position_m[0], self.position_m[1]),
        )
        .on_plane()
    }

    /// Mechanical support: terrain plane, water column, or free air.
    /// Land cells still sample `Air` for drag; a rover on the pad is `terrain`.
    pub fn support(&self, hydro: &crate::hydro::HydroField, env: &Environment) -> &'static str {
        if self.on_terrain(env) {
            "terrain"
        } else if self.medium_in(hydro, env) == Medium::Water {
            "water"
        } else {
            "air"
        }
    }

    pub fn energy(&self, gravity: f32) -> (f32, f32) {
        (
            flight_core::mech::kinetic_energy(self.mass_kg, self.velocity_mps),
            flight_core::mech::gravitational_pe_ned(self.mass_kg, self.position_m[2], gravity),
        )
    }
}

fn sphere_inertia(mass: f32, radius: f32) -> [f32; 3] {
    let i = 0.4 * mass * radius * radius;
    [i.max(1e-4), i.max(1e-4), (i * 1.15).max(1e-4)]
}
