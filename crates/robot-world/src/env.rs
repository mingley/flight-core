//! Wind, current, density, and terrain. NED throughout (z positive down).

use flight_core::domain::Medium;

/// Surrounding field a body samples at a point.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Environment {
    /// Air-relative flow in NED, m/s.
    pub wind_ned: [f32; 3],
    /// Water-relative flow in NED, m/s.
    pub current_ned: [f32; 3],
    pub gravity: f32,
    pub air_density: f32,
    pub water_density: f32,
    /// Water / land surface in NED down. Zero is the local tangent plane.
    pub waterline_z: f32,
    /// Seabed depth (positive down) for `n < shoreline_n`.
    pub seabed_z: f32,
    /// North of this line is land (`terrain_z = 0`).
    pub shoreline_n: f32,
    /// Surface wave amplitude in meters (0 = flat).
    pub wave_amp: f32,
    pub wave_k: f32,
    pub wave_omega: f32,
    /// Phase offset in radians. Set from the world seed so runs are reproducible.
    pub wave_phase: f32,
}

impl Environment {
    /// Flat land north of `n = 0`, water to the south, 4 m seabed, light breeze.
    pub fn coastal() -> Self {
        Self {
            wind_ned: [0.0, 2.0, 0.0],
            current_ned: [0.35, 0.0, 0.0],
            gravity: 9.80665,
            air_density: 1.225,
            water_density: 1025.0,
            waterline_z: 0.0,
            seabed_z: 4.0,
            shoreline_n: 0.0,
            wave_amp: 0.08,
            wave_k: 0.55,
            wave_omega: 1.4,
            wave_phase: 0.0,
        }
    }

    /// All land: gusty wind, no water, no waves.
    pub fn inland() -> Self {
        let mut e = Self::coastal();
        e.wind_ned = [3.2, 0.8, 0.0];
        e.current_ned = [0.0, 0.0, 0.0];
        e.shoreline_n = -1.0e6;
        e.seabed_z = 0.0;
        e.wave_amp = 0.0;
        e
    }

    /// Mixed quay: same shoreline as coastal, deeper basin, chop, cross-current.
    pub fn harbor() -> Self {
        let mut e = Self::coastal();
        e.wind_ned = [0.5, 1.2, 0.0];
        e.current_ned = [0.12, 0.65, 0.0];
        e.seabed_z = 6.0;
        e.wave_amp = 0.14;
        e.wave_omega = 1.7;
        e
    }

    /// No land in the local tangent patch. Deeper water, larger swell.
    pub fn open_water() -> Self {
        let mut e = Self::coastal();
        e.wind_ned = [1.5, 4.0, 0.0];
        e.current_ned = [0.0, 0.55, 0.0];
        e.shoreline_n = 1.0e6;
        e.seabed_z = 12.0;
        e.wave_amp = 0.22;
        e.wave_k = 0.35;
        e.wave_omega = 1.1;
        e
    }

    /// Instantaneous water surface in NED down. Land returns the tangent plane.
    pub fn water_surface_z(&self, n: f32, t: f32) -> f32 {
        if n >= self.shoreline_n || self.wave_amp.abs() < 1e-9 {
            return self.waterline_z;
        }
        self.waterline_z
            + self.wave_amp * (self.wave_k * n - self.wave_omega * t + self.wave_phase).sin()
    }

    pub fn terrain_z(&self, n: f32, _e: f32) -> f32 {
        if n >= self.shoreline_n {
            0.0
        } else {
            self.seabed_z
        }
    }

    pub fn medium_at(&self, n: f32, z: f32) -> Medium {
        self.medium_at_time(n, z, 0.0)
    }

    pub fn medium_at_time(&self, n: f32, z: f32, t: f32) -> Medium {
        if n < self.shoreline_n && z > self.water_surface_z(n, t) {
            Medium::Water
        } else {
            Medium::Air
        }
    }

    pub fn density(&self, medium: Medium) -> f32 {
        match medium {
            Medium::Air => self.air_density,
            Medium::Water => self.water_density,
            Medium::Soil => 0.0,
        }
    }

    pub fn flow_ned(&self, medium: Medium) -> [f32; 3] {
        match medium {
            Medium::Water => self.current_ned,
            Medium::Air | Medium::Soil => self.wind_ned,
        }
    }

    /// Bake a seed into wave phase and a small mean-preserving gust so two
    /// labs opened with the same seed replay the same field.
    pub fn apply_seed(&mut self, seed: u64) {
        self.wave_phase = unit01(seed, 1) * core::f32::consts::PI * 2.0;
        self.wind_ned[0] += (unit01(seed, 2) - 0.5) * 0.8;
        self.wind_ned[1] += (unit01(seed, 3) - 0.5) * 0.8;
        self.current_ned[0] += (unit01(seed, 4) - 0.5) * 0.06;
        self.current_ned[1] += (unit01(seed, 5) - 0.5) * 0.06;
    }
}

fn unit01(seed: u64, lane: u64) -> f32 {
    let x = seed
        .wrapping_add(lane.wrapping_mul(0xD1B5_4A32_D192_ED03))
        .wrapping_mul(0x9E37_79B9_7F4A_7C15);
    (x >> 40) as u32 as f32 / 16_777_216.0
}

impl Default for Environment {
    fn default() -> Self {
        Self::coastal()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::domain::Medium;

    #[test]
    fn land_has_no_waves() {
        let e = Environment::coastal();
        assert_eq!(e.water_surface_z(3.0, 1.7), 0.0);
        assert_eq!(e.medium_at_time(3.0, -1.0, 1.7), Medium::Air);
    }

    #[test]
    fn sea_surface_moves() {
        let e = Environment::coastal();
        let a = e.water_surface_z(-6.0, 0.0);
        let b = e.water_surface_z(-6.0, 1.1);
        assert!((a - b).abs() > 1e-4);
        assert!(e.medium_at_time(-6.0, 0.5, 0.0) == Medium::Water);
    }

    #[test]
    fn inland_is_all_land() {
        let e = Environment::inland();
        assert_eq!(e.medium_at(10.0, 0.5), Medium::Air);
        assert_eq!(e.terrain_z(-20.0, 0.0), 0.0);
        assert_eq!(e.wave_amp, 0.0);
    }

    #[test]
    fn open_water_is_all_sea() {
        let e = Environment::open_water();
        assert_eq!(e.medium_at_time(0.0, 1.0, 0.0), Medium::Water);
        assert_eq!(e.terrain_z(0.0, 0.0), 12.0);
    }

    #[test]
    fn seed_shifts_phase_and_wind() {
        let mut a = Environment::coastal();
        let mut b = Environment::coastal();
        a.apply_seed(1);
        b.apply_seed(2);
        assert!((a.wave_phase - b.wave_phase).abs() > 1e-4);
        assert_ne!(a.wind_ned, b.wind_ned);
        let za = a.water_surface_z(-6.0, 0.0);
        let zb = b.water_surface_z(-6.0, 0.0);
        assert!((za - zb).abs() > 1e-4);
        let mut a2 = Environment::coastal();
        a2.apply_seed(1);
        assert_eq!(a.wave_phase, a2.wave_phase);
        assert_eq!(a.wind_ned, a2.wind_ned);
    }
}
