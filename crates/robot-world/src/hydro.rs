//! Owned shallow-water field sampled by the world step.

use crate::env::Environment;
use flight_core::domain::Medium;
use flight_core::hydro::{
    apply_wave_mode, hydro_invariants, hydro_volume, HydroGrid, HydroInvariants, HydroSample,
    HydroState,
};

pub const HYDRO_NX: usize = 40;
pub const HYDRO_NY: usize = 32;
pub const HYDRO_DX: f32 = 2.0;
pub const HYDRO_ORIGIN_N: f32 = -48.0;
pub const HYDRO_ORIGIN_E: f32 = -32.0;

/// Cell-centered water column covering the local NED tangent patch.
#[derive(Clone, Debug)]
pub struct HydroField {
    pub grid: HydroGrid,
    pub h: Vec<f32>,
    pub un: Vec<f32>,
    pub ue: Vec<f32>,
    pub still: Vec<f32>,
    pub volume0: f32,
    pub(crate) scratch: Vec<f32>,
}

impl HydroField {
    pub fn from_env(env: &Environment) -> Self {
        let grid = HydroGrid {
            nx: HYDRO_NX,
            ny: HYDRO_NY,
            dx: HYDRO_DX,
            g: env.gravity.abs(),
            origin_n: HYDRO_ORIGIN_N,
            origin_e: HYDRO_ORIGIN_E,
        };
        let n = grid.cells();
        let mut still = vec![0.0; n];
        let mut h = vec![0.0; n];
        let mut un = vec![0.0; n];
        let mut ue = vec![0.0; n];
        let still_col = (env.seabed_z - env.waterline_z).max(0.0);
        for i in 0..grid.nx {
            for j in 0..grid.ny {
                let k = grid.idx(i, j);
                let north = grid.origin_n + (i as f32 + 0.5) * grid.dx;
                if north < env.shoreline_n && still_col > 0.0 {
                    still[k] = still_col;
                    h[k] = still_col;
                    un[k] = env.current_ned[0];
                    ue[k] = env.current_ned[1];
                }
            }
        }
        apply_wave_mode(
            grid,
            &mut h,
            &still,
            env.wave_amp,
            env.wave_k,
            env.wave_phase,
        );
        let volume0 = hydro_volume(&h, grid.dx);
        Self {
            grid,
            h,
            un,
            ue,
            still,
            volume0,
            scratch: vec![0.0; 3 * n],
        }
    }

    pub fn step(&mut self, dt: f32, env: &Environment) {
        #[cfg(feature = "gpu")]
        if crate::gpu::requested() && crate::gpu::advance(self, dt, env) {
            return;
        }
        let mut state = HydroState {
            grid: self.grid,
            h: &mut self.h,
            un: &mut self.un,
            ue: &mut self.ue,
            still: &self.still,
            scratch: &mut self.scratch,
        };
        state.step(dt, env.wind_ned[0], env.wind_ned[1]);
    }

    pub fn sample(&self, n: f32, e: f32, waterline_z: f32) -> HydroSample {
        flight_core::hydro::sample_field(
            self.grid,
            &self.h,
            &self.un,
            &self.ue,
            &self.still,
            n,
            e,
            waterline_z,
        )
    }

    pub fn surface_z(&self, n: f32, e: f32, waterline_z: f32) -> f32 {
        self.sample(n, e, waterline_z).surface_z
    }

    pub fn medium_at(&self, n: f32, e: f32, z: f32, waterline_z: f32) -> Medium {
        let s = self.sample(n, e, waterline_z);
        if s.still > 0.0 && z > s.surface_z {
            Medium::Water
        } else {
            Medium::Air
        }
    }

    pub fn flow_ned(&self, n: f32, e: f32, waterline_z: f32, wind: [f32; 3]) -> [f32; 3] {
        let s = self.sample(n, e, waterline_z);
        if s.still > 0.0 {
            [s.un, s.ue, 0.0]
        } else {
            wind
        }
    }

    pub fn invariants(&self) -> HydroInvariants {
        hydro_invariants(
            &self.h,
            &self.un,
            &self.ue,
            &self.still,
            self.volume0,
            self.grid.dx,
        )
    }

    pub fn volume(&self) -> f32 {
        hydro_volume(&self.h, self.grid.dx)
    }

    /// Shift wet-cell velocity by `new - old` so swell is not wiped.
    pub fn shift_current(&mut self, old: [f32; 3], new: [f32; 3]) {
        let dn = new[0] - old[0];
        let de = new[1] - old[1];
        for k in 0..self.still.len() {
            if self.still[k] > 0.0 {
                self.un[k] += dn;
                self.ue[k] += de;
            }
        }
    }

    /// Re-seed a zero-mean swell. Volume is recorded after the mode is applied.
    pub fn apply_waves(&mut self, amp: f32, k_wave: f32, phase: f32) {
        apply_wave_mode(self.grid, &mut self.h, &self.still, amp, k_wave, phase);
        self.volume0 = self.volume();
    }
}
