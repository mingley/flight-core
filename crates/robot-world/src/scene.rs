//! Rust scene DSL: named catalogs or a custom body table (NEXT C3).
//!
//! Reserved catalog names keep P11: `inland` cannot include a hull, and
//! `open_water` cannot include a rover. A new name is a new catalog.

use crate::body::Body;
use crate::env::Environment;
use crate::hydro::HydroField;
use crate::properties;
use crate::world::World;

/// Why a [`Scene`] could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SceneError {
    UnknownCatalog,
    /// Reserved catalog `inland` cannot include a hull.
    InlandHull,
    /// Reserved catalog `open_water` cannot include a rover.
    OpenWaterRover,
    UnknownBody,
    /// Hydro grid is empty, non-finite, or larger than the C4 cap.
    InvalidHydro,
}

impl core::fmt::Display for SceneError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnknownCatalog => write!(f, "unknown catalog"),
            Self::InlandHull => write!(f, "inland catalog cannot include a hull"),
            Self::OpenWaterRover => write!(f, "open_water catalog cannot include a rover"),
            Self::UnknownBody => write!(f, "unknown body"),
            Self::InvalidHydro => write!(f, "invalid hydro grid"),
        }
    }
}

impl std::error::Error for SceneError {}

/// A named verified scene: catalog or custom body set, seed, field overlays,
/// and optional battery charges.
#[derive(Clone, Debug)]
pub struct Scene {
    name: &'static str,
    seed: u64,
    source: SceneSource,
    wind: Option<[f32; 3]>,
    current: Option<[f32; 3]>,
    wave_amp: Option<f32>,
    charges: Vec<(&'static str, f32)>,
    hydro: Option<(usize, usize, f32)>,
}

#[derive(Clone, Debug)]
enum SceneSource {
    Catalog,
    Custom { env: Environment, bodies: Vec<Body> },
}

impl Scene {
    /// One of [`World::SCENARIOS`] (`open-water` canonicalizes to `open_water`).
    pub fn catalog(name: &'static str) -> Result<Self, SceneError> {
        let world = World::named(name, 0).ok_or(SceneError::UnknownCatalog)?;
        Ok(Self {
            name: world.scenario,
            seed: 0,
            source: SceneSource::Catalog,
            wind: None,
            current: None,
            wave_amp: None,
            charges: Vec::new(),
            hydro: None,
        })
    }

    /// New catalog name with an explicit body table. Using the reserved names
    /// `inland` or `open_water` still enforces P11.
    pub fn custom(
        name: &'static str,
        env: Environment,
        bodies: impl IntoIterator<Item = Body>,
    ) -> Result<Self, SceneError> {
        if name.is_empty() {
            return Err(SceneError::UnknownCatalog);
        }
        let bodies: Vec<Body> = bodies.into_iter().collect();
        check_reserved_p11(name, &bodies)?;
        Ok(Self {
            name,
            seed: 0,
            source: SceneSource::Custom { env, bodies },
            wind: None,
            current: None,
            wave_amp: None,
            charges: Vec::new(),
            hydro: None,
        })
    }

    pub fn seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    /// Replaces wind after the catalog seed is applied.
    pub fn wind_ned(mut self, wind: [f32; 3]) -> Self {
        self.wind = Some(wind);
        self
    }

    /// Replaces current after the catalog seed is applied.
    pub fn current_ned(mut self, current: [f32; 3]) -> Self {
        self.current = Some(current);
        self
    }

    /// Surface wave amplitude (metres). Rebuilds the hydro field.
    pub fn waves(mut self, amp: f32) -> Self {
        self.wave_amp = Some(amp);
        self
    }

    pub fn charge(mut self, id: &'static str, joules: f32) -> Self {
        self.charges.push((id, joules));
        self
    }

    /// Rebuild the shallow-water field at `nx` × `ny` with cell size `dx`
    /// (metres). Same origin as the catalog patch. Catalogs stay 40×32 unless
    /// this overlay is set.
    pub fn hydro(mut self, nx: usize, ny: usize, dx: f32) -> Self {
        self.hydro = Some((nx, ny, dx));
        self
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    pub fn build(self) -> Result<World, SceneError> {
        let mut world = match self.source {
            SceneSource::Catalog => {
                World::named(self.name, self.seed).ok_or(SceneError::UnknownCatalog)?
            }
            SceneSource::Custom { env, bodies } => {
                check_reserved_p11(self.name, &bodies)?;
                World::assemble(self.name, self.seed, env, bodies)
            }
        };
        if let Some(v) = self.wind {
            world.env.wind_ned = v;
        }
        if let Some(v) = self.current {
            world.env.current_ned = v;
        }
        if let Some(amp) = self.wave_amp {
            world.env.wave_amp = amp;
        }
        if self.wave_amp.is_some() || self.hydro.is_some() {
            world.hydro = match self.hydro {
                Some((nx, ny, dx)) => {
                    HydroField::from_env_grid(&world.env, hydro_grid(&world.env, nx, ny, dx)?)
                }
                None => HydroField::from_env(&world.env),
            };
        }
        for (id, j) in self.charges {
            let Some(body) = world.body_mut(id) else {
                return Err(SceneError::UnknownBody);
            };
            let j = j.clamp(0.0, body.capacity_j);
            body.charge_j = j;
            body.last_charge_j = j;
        }
        world.last_properties = properties::evaluate(&world);
        Ok(world)
    }
}

/// P11 on reserved catalog names. New names are not restricted here.
fn check_reserved_p11(name: &str, bodies: &[Body]) -> Result<(), SceneError> {
    let has_hull = bodies.iter().any(|b| b.marine.is_some());
    let has_rover = bodies.iter().any(|b| b.ground.is_some());
    match name {
        "inland" if has_hull => Err(SceneError::InlandHull),
        "open_water" | "open-water" if has_rover => Err(SceneError::OpenWaterRover),
        _ => Ok(()),
    }
}

fn hydro_grid(
    env: &Environment,
    nx: usize,
    ny: usize,
    dx: f32,
) -> Result<flight_core::hydro::HydroGrid, SceneError> {
    const MAX: usize = 256;
    if nx == 0 || ny == 0 || nx > MAX || ny > MAX || !dx.is_finite() || dx <= 0.0 {
        return Err(SceneError::InvalidHydro);
    }
    Ok(flight_core::hydro::HydroGrid {
        nx,
        ny,
        dx,
        g: env.gravity.abs(),
        origin_n: crate::hydro::HYDRO_ORIGIN_N,
        origin_e: crate::hydro::HYDRO_ORIGIN_E,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Body, Environment};

    #[test]
    fn catalog_inland_matches_world() {
        let scene = Scene::catalog("inland").unwrap().seed(1).build().unwrap();
        let world = World::inland(1);
        assert_eq!(scene.scenario, "inland");
        assert_eq!(scene.seed, 1);
        assert_eq!(scene.bodies.len(), world.bodies.len());
        assert_eq!(scene.env.wind_ned, world.env.wind_ned);
        assert_eq!(scene.env.current_ned, world.env.current_ned);
        assert_eq!(scene.env.wave_amp, world.env.wave_amp);
        assert!(scene.body("skiff").is_none());
        assert!(scene.body("rover").is_some());
        assert!(scene.all_hold());
    }

    #[test]
    fn catalog_open_water_has_no_rover() {
        let scene = Scene::catalog("open-water")
            .unwrap()
            .seed(2)
            .build()
            .unwrap();
        assert_eq!(scene.scenario, "open_water");
        assert!(scene.body("rover").is_none());
        assert!(scene.body("skiff").is_some());
    }

    #[test]
    fn unknown_catalog_is_an_error() {
        assert_eq!(
            Scene::catalog("pad_pair").unwrap_err(),
            SceneError::UnknownCatalog
        );
    }

    #[test]
    fn inland_name_rejects_a_hull() {
        let err = Scene::custom(
            "inland",
            Environment::inland(),
            [
                Body::aerial_ready("drone"),
                Body::rover("rover"),
                Body::skiff("skiff"),
            ],
        )
        .unwrap_err();
        assert_eq!(err, SceneError::InlandHull);
    }

    #[test]
    fn open_water_name_rejects_a_rover() {
        let err = Scene::custom(
            "open_water",
            Environment::open_water(),
            [
                Body::aerial_ready("drone"),
                Body::rover("rover"),
                Body::skiff("skiff"),
            ],
        )
        .unwrap_err();
        assert_eq!(err, SceneError::OpenWaterRover);
    }

    #[test]
    fn empty_custom_name_is_unknown_catalog() {
        let err =
            Scene::custom("", Environment::inland(), [Body::aerial_ready("drone")]).unwrap_err();
        assert_eq!(err, SceneError::UnknownCatalog);
    }

    #[test]
    fn custom_name_may_use_inland_env_without_hull() {
        let mut world = Scene::custom(
            "pad_pair",
            Environment::inland(),
            [Body::aerial_ready("drone"), Body::rover("rover")],
        )
        .unwrap()
        .seed(3)
        .build()
        .unwrap();
        assert_eq!(world.scenario, "pad_pair");
        assert!(world.body("skiff").is_none());
        assert!(world.body("rover").is_some());
        assert!(world.all_hold());
        world.step(0.02);
        assert!(world.all_hold());
    }

    #[test]
    fn wind_and_charge_overlays() {
        let world = Scene::catalog("inland")
            .unwrap()
            .seed(1)
            .wind_ned([1.0, 0.0, 0.0])
            .current_ned([0.0, 0.0, 0.0])
            .waves(0.0)
            .charge("drone", 12.0)
            .build()
            .unwrap();
        assert_eq!(world.env.wind_ned, [1.0, 0.0, 0.0]);
        assert_eq!(world.env.current_ned, [0.0, 0.0, 0.0]);
        assert_eq!(world.env.wave_amp, 0.0);
        assert_eq!(world.body("drone").unwrap().charge_j, 12.0);
        assert!(world.all_hold());
    }

    #[test]
    fn charge_unknown_body_is_an_error() {
        let err = Scene::catalog("inland")
            .unwrap()
            .charge("skiff", 1.0)
            .build()
            .unwrap_err();
        assert_eq!(err, SceneError::UnknownBody);
    }

    #[test]
    fn invalid_hydro_grid_is_an_error() {
        assert_eq!(
            Scene::catalog("coastal")
                .unwrap()
                .hydro(0, 16, 4.0)
                .build()
                .unwrap_err(),
            SceneError::InvalidHydro
        );
        assert_eq!(
            Scene::catalog("coastal")
                .unwrap()
                .hydro(20, 16, 0.0)
                .build()
                .unwrap_err(),
            SceneError::InvalidHydro
        );
    }

    fn hydro_ids_hold(world: &World) {
        let ids: Vec<_> = world
            .last_properties
            .iter()
            .filter(|p| p.holds)
            .map(|p| p.id)
            .collect();
        for id in [
            "hydro_height_nonnegative",
            "hydro_volume_conserved",
            "hydro_land_stays_dry",
        ] {
            assert!(
                ids.contains(&id),
                "{id} missing/failed {:?}",
                world.last_properties
            );
        }
    }

    #[test]
    fn coastal_half_and_double_hydro_keep_invariants() {
        for (nx, ny, dx) in [(20, 16, 4.0), (80, 64, 1.0)] {
            let mut world = Scene::catalog("coastal")
                .unwrap()
                .seed(1)
                .hydro(nx, ny, dx)
                .build()
                .unwrap();
            assert_eq!(world.hydro.grid.nx, nx);
            assert_eq!(world.hydro.grid.ny, ny);
            assert_eq!(world.hydro.grid.dx, dx);
            assert!(
                world.hydro.volume0 > 100.0,
                "volume0={}",
                world.hydro.volume0
            );
            for _ in 0..80 {
                world.step(0.02);
                assert!(world.all_hold(), "{nx}x{ny} {:?}", world.last_properties);
                hydro_ids_hold(&world);
            }
        }
    }

    #[test]
    fn extra_body_keeps_contact_properties() {
        let mut rover = Body::rover("rover");
        rover.position_m = [14.0, 0.05, 0.0];
        rover.velocity_mps = [0.0, -0.8, 0.0];
        let mut scout = Body::rover("scout");
        scout.position_m = [14.0, 0.0, 0.0];
        scout.velocity_mps = [0.0, 0.4, 0.0];
        let mut world = Scene::custom(
            "pad_trio",
            Environment::inland(),
            [Body::aerial_ready("drone"), rover, scout],
        )
        .unwrap()
        .seed(3)
        .build()
        .unwrap();
        assert_eq!(world.bodies.len(), 3);
        world.step(0.02);
        assert!(world.all_hold(), "{:?}", world.last_properties);
        let rover = world.body("rover").unwrap();
        let scout = world.body("scout").unwrap();
        let dx = rover.position_m[0] - scout.position_m[0];
        let dy = rover.position_m[1] - scout.position_m[1];
        let dz = rover.position_m[2] - scout.position_m[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(
            dist + 1e-3 >= rover.radius_m + scout.radius_m,
            "dist {dist} radii {} {}",
            rover.radius_m,
            scout.radius_m
        );
        assert!(
            rover.last_sphere_impulse > 0.0 || scout.last_sphere_impulse > 0.0,
            "jn rover={} scout={}",
            rover.last_sphere_impulse,
            scout.last_sphere_impulse
        );
        assert!(world
            .last_properties
            .iter()
            .any(|p| p.id == "no_body_interpenetration" && p.holds));
        assert!(world
            .last_properties
            .iter()
            .any(|p| p.id == "no_terrain_penetration" && p.holds));
    }
}
