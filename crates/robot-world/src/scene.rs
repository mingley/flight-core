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
}

impl core::fmt::Display for SceneError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnknownCatalog => write!(f, "unknown catalog"),
            Self::InlandHull => write!(f, "inland catalog cannot include a hull"),
            Self::OpenWaterRover => write!(f, "open_water catalog cannot include a rover"),
            Self::UnknownBody => write!(f, "unknown body"),
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
            world.hydro = HydroField::from_env(&world.env);
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
}
