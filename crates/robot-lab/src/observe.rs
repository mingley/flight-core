use flight_core::vehicle::{
    aerial_kind, ground_kind, marine_kind, AerialKind, GroundKind, MarineKind,
};
use robot_world::{Body, Property, SphereHit, World};
use serde::{Deserialize, Serialize};

use crate::cmd::LabCmd;
use crate::lab::Lab;

/// Clear-state snapshot for agents and the live console.
#[derive(Clone, Debug, Serialize)]
pub struct Observation {
    pub t: f32,
    pub scenario: &'static str,
    pub seed: u64,
    pub message: String,
    pub all_hold: bool,
    /// Wind / waves / current — always legal, no robot id required.
    pub env_cmds: Vec<LabCmd>,
    pub environment: EnvView,
    pub robots: Vec<RobotView>,
    pub properties: Vec<Property>,
    /// Pairwise sphere contacts this step (`a`/`b` sorted).
    pub sphere_hits: Vec<SphereHit>,
}

#[derive(Clone, Debug, Serialize)]
pub struct EnvView {
    pub wind_ned: [f32; 3],
    pub current_ned: [f32; 3],
    pub waterline_z: f32,
    pub seabed_z: f32,
    pub shoreline_n: f32,
    pub wave_amp: f32,
    pub wave_phase: f32,
    pub hydro_nx: usize,
    pub hydro_ny: usize,
    pub hydro_dx: f32,
    pub hydro_origin_n: f32,
    pub hydro_origin_e: f32,
    pub hydro_volume: f32,
    pub hydro_volume0: f32,
    pub hydro_gpu: bool,
    pub hydro_h: Vec<f32>,
}

#[derive(Clone, Debug, Serialize)]
pub struct RobotView {
    pub id: String,
    pub domain: String,
    pub phase: String,
    pub medium: String,
    /// `terrain`, `water`, or `air` — what holds the hull, not the drag fluid.
    pub support: String,
    /// Current pose is on the terrain plane (pad / ground / seabed).
    pub terrain_contact: bool,
    /// Normal impulse from the last terrain resolve (0 when no hit this step).
    pub contact_jn: f32,
    /// Pairwise sphere hit this step (`last_sphere_impulse > 0`).
    pub sphere_contact: bool,
    /// Peak normal impulse from the last sphere sweep.
    pub sphere_jn: f32,
    /// Peak Coulomb tangent impulse from the last sphere sweep.
    pub sphere_jt: f32,
    /// Other hull ids that shared a sphere hit with this body this step.
    pub sphere_partners: Vec<String>,
    pub n: f32,
    pub e: f32,
    pub d: f32,
    pub vn: f32,
    pub ve: f32,
    pub vd: f32,
    pub alt: f32,
    pub yaw: f32,
    pub armed: bool,
    pub actuators: bool,
    pub failsafe: bool,
    pub ke: f32,
    pub pe: f32,
    pub drag_power: f32,
    pub charge_j: f32,
    pub capacity_j: f32,
    pub angular_ke: f32,
    pub radius_m: f32,
    pub propulsion_live: bool,
    /// [`LabCmd`] values the live safety machine would accept on this body.
    pub legal_cmds: Vec<LabCmd>,
    /// Live NED pose hold. Absent when idle or after failsafe / velocity.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hold_ned: Option<[f32; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aerial: Option<AerialMachine>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ground: Option<GroundMachine>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub marine: Option<MarineMachine>,
}

/// Aerial safety machine as the agent sees it.
#[derive(Clone, Debug, Serialize)]
pub struct AerialMachine {
    pub phase: &'static str,
    /// Consume-self typestate `attach` binds — not the plant phase string.
    /// After `attach_offboard` this is `Offboard` while `phase` stays `"armed"`.
    pub kind: AerialKind,
    pub armed: bool,
    pub actuators_enabled: bool,
    pub offboard: bool,
    pub failsafe: bool,
    pub imu_healthy: bool,
    pub estimator_valid: bool,
}

/// Ground safety machine as the agent sees it.
#[derive(Clone, Debug, Serialize)]
pub struct GroundMachine {
    pub phase: &'static str,
    pub kind: GroundKind,
    pub drive_enabled: bool,
    pub estop: bool,
}

/// Marine safety machine as the agent sees it.
#[derive(Clone, Debug, Serialize)]
pub struct MarineMachine {
    pub phase: &'static str,
    pub kind: MarineKind,
    pub thrust_enabled: bool,
    pub failsafe: bool,
}

impl Observation {
    pub(crate) fn from_lab(lab: &Lab) -> Self {
        lab.with_world(|world| Self {
            t: world.t,
            scenario: world.scenario,
            seed: world.seed,
            message: lab.message.clone(),
            all_hold: world.all_hold(),
            env_cmds: LabCmd::ENV.to_vec(),
            environment: EnvView::from_world(world),
            robots: world
                .bodies
                .iter()
                .map(|b| RobotView::from_body(b, world))
                .collect(),
            properties: world.last_properties.clone(),
            sphere_hits: world.last_sphere_hits.clone(),
        })
    }
}

impl EnvView {
    pub(crate) fn from_world(world: &World) -> Self {
        let env = &world.env;
        Self {
            wind_ned: env.wind_ned,
            current_ned: env.current_ned,
            waterline_z: env.waterline_z,
            seabed_z: env.seabed_z,
            shoreline_n: env.shoreline_n,
            wave_amp: env.wave_amp,
            wave_phase: env.wave_phase,
            hydro_nx: world.hydro.grid.nx,
            hydro_ny: world.hydro.grid.ny,
            hydro_dx: world.hydro.grid.dx,
            hydro_origin_n: world.hydro.grid.origin_n,
            hydro_origin_e: world.hydro.grid.origin_e,
            hydro_volume: world.hydro.volume(),
            hydro_volume0: world.hydro.volume0,
            hydro_gpu: robot_world::gpu::active(),
            hydro_h: world.hydro.h.clone(),
        }
    }
}

impl RobotView {
    pub(crate) fn from_body(b: &Body, world: &World) -> Self {
        Self {
            id: b.id.into(),
            domain: b.domain.name().into(),
            phase: b.phase_name().into(),
            medium: b.medium_in(&world.hydro, &world.env).name().into(),
            support: b.support(&world.hydro, &world.env).into(),
            terrain_contact: b.on_terrain(&world.env),
            contact_jn: b.last_contact.impulse,
            sphere_contact: b.last_sphere_impulse > 1e-6,
            sphere_jn: b.last_sphere_impulse,
            sphere_jt: b.last_tangent_impulse,
            sphere_partners: world
                .last_sphere_hits
                .iter()
                .filter_map(|h| h.other(b.id).map(str::to_string))
                .collect(),
            n: b.position_m[0],
            e: b.position_m[1],
            d: b.position_m[2],
            vn: b.velocity_mps[0],
            ve: b.velocity_mps[1],
            vd: b.velocity_mps[2],
            alt: b.altitude_agl(),
            yaw: b.yaw_rad,
            armed: b.armed_like(),
            actuators: b.actuators_granted(),
            failsafe: b.failsafe(),
            ke: b.last_ke,
            pe: b.last_pe,
            drag_power: b.last_drag_power,
            charge_j: b.charge_j,
            capacity_j: b.capacity_j,
            angular_ke: b.last_angular_ke,
            radius_m: b.radius_m,
            propulsion_live: b.propulsion_live(),
            legal_cmds: LabCmd::ALL
                .into_iter()
                .filter(|c| c.on_legal_list(b))
                .collect(),
            hold_ned: b.hold_ned,
            aerial: b.aerial.map(|s| AerialMachine {
                phase: s.phase.name(),
                kind: aerial_kind(s),
                armed: s.armed,
                actuators_enabled: s.actuators_enabled,
                offboard: s.offboard,
                failsafe: s.failsafe,
                imu_healthy: s.imu_healthy,
                estimator_valid: s.estimator_valid,
            }),
            ground: b.ground.map(|s| GroundMachine {
                phase: s.phase.name(),
                kind: ground_kind(s),
                drive_enabled: s.drive_enabled,
                estop: s.estop,
            }),
            marine: b.marine.map(|s| MarineMachine {
                phase: s.phase.name(),
                kind: marine_kind(s),
                thrust_enabled: s.thrust_enabled,
                failsafe: s.failsafe,
            }),
        }
    }

    pub fn allows(&self, cmd: LabCmd) -> bool {
        self.legal_cmds.contains(&cmd)
    }
}

/// One callable robot tool: a `(robot_id, cmd)` pair from [`RobotView::legal_cmds`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RobotTool {
    pub robot: String,
    pub cmd: LabCmd,
}

/// The only tools an agent may call given an [`Observation`]: environment
/// commands plus per-robot [`LabCmd`] values from `legal_cmds`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegalTools {
    pub env_cmds: Vec<LabCmd>,
    pub robot_tools: Vec<RobotTool>,
}

impl Observation {
    /// NEXT A1: enumerate callable tools without reading kernel source.
    pub fn tools(&self) -> LegalTools {
        LegalTools::from_observation(self)
    }
}

impl LegalTools {
    pub fn from_observation(obs: &Observation) -> Self {
        Self {
            env_cmds: obs.env_cmds.clone(),
            robot_tools: obs
                .robots
                .iter()
                .flat_map(|r| {
                    r.legal_cmds.iter().copied().map(|cmd| RobotTool {
                        robot: r.id.clone(),
                        cmd,
                    })
                })
                .collect(),
        }
    }

    pub fn allows(&self, robot: &str, cmd: LabCmd) -> bool {
        if LabCmd::ENV.contains(&cmd) {
            return true;
        }
        self.robot_tools
            .iter()
            .any(|t| t.robot == robot && t.cmd == cmd)
    }
}
