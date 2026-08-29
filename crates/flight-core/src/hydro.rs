//! 2-D shallow-water kernel (Saint-Venant).
//!
//! Cell-centered water column `h` and NED horizontal velocity `(un, ue)`.
//! Land is `still == 0` and stays dry. Wet cells use a Rusanov flux with
//! reflecting walls, so volume is a conserved quantity of the discrete step.
//! The same arithmetic is what a Vulkan compute shader runs.

use crate::math::sqrtf;

/// Column thinner than this is treated as dry (no flux, velocity zero).
pub const HYDRO_H_DRY: f32 = 1e-4;

/// Linear Rayleigh friction, 1/s, keeps a long run from ringing.
pub const HYDRO_FRICTION: f32 = 0.08;

/// Wind-stress coupling: acceleration ≈ coeff * wind / h.
pub const HYDRO_WIND_COEFF: f32 = 0.04;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HydroGrid {
    pub nx: usize,
    pub ny: usize,
    pub dx: f32,
    pub g: f32,
    pub origin_n: f32,
    pub origin_e: f32,
}

impl HydroGrid {
    pub fn cells(self) -> usize {
        self.nx.saturating_mul(self.ny)
    }

    pub fn idx(self, i: usize, j: usize) -> usize {
        i * self.ny + j
    }

    pub fn in_bounds(self, i: isize, j: isize) -> bool {
        i >= 0 && j >= 0 && (i as usize) < self.nx && (j as usize) < self.ny
    }
}

/// Sample of the free surface and orbital flow at a NED point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HydroSample {
    /// Water-column thickness, metres.
    pub height: f32,
    /// Still-water column (0 on land).
    pub still: f32,
    pub un: f32,
    pub ue: f32,
    /// Free-surface NED-down. More water ⇒ smaller z (crest).
    pub surface_z: f32,
}

/// Named facts the world re-checks after every hydro step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HydroInvariants {
    pub height_nonnegative: bool,
    pub volume_conserved: bool,
    pub finite: bool,
    pub land_dry: bool,
}

impl HydroInvariants {
    pub fn all(self) -> bool {
        self.height_nonnegative && self.volume_conserved && self.finite && self.land_dry
    }
}

/// Mutable shallow-water buffers. `scratch` must be `3 * nx * ny`.
pub struct HydroState<'a> {
    pub grid: HydroGrid,
    pub h: &'a mut [f32],
    pub un: &'a mut [f32],
    pub ue: &'a mut [f32],
    pub still: &'a [f32],
    pub scratch: &'a mut [f32],
}

impl HydroState<'_> {
    pub fn volume(&self) -> f32 {
        hydro_volume(self.h, self.grid.dx)
    }

    pub fn invariants(&self, volume0: f32) -> HydroInvariants {
        hydro_invariants(self.h, self.un, self.ue, self.still, volume0, self.grid.dx)
    }

    /// Advance the field by `dt`, sub-stepping to a CFL of ~0.45.
    pub fn step(&mut self, dt: f32, wind_n: f32, wind_e: f32) {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return;
        }
        let n = self.grid.cells();
        if self.h.len() < n
            || self.un.len() < n
            || self.ue.len() < n
            || self.still.len() < n
            || self.scratch.len() < 3 * n
        {
            return;
        }
        let nsub = hydro_cfl_substeps(self.grid, self.h, self.un, self.ue, dt);
        let dti = dt / nsub as f32;
        for _ in 0..nsub {
            sweep(self, true, dti);
            sweep(self, false, dti);
            self.relax(dti, wind_n, wind_e);
        }
    }

    /// Wind stress, Rayleigh friction, and dry/land pinning.
    pub fn relax(&mut self, dt: f32, wind_n: f32, wind_e: f32) {
        apply_source(self, dt, wind_n, wind_e);
        pin_land_and_dry(self);
    }

    pub fn sample(&self, n: f32, e: f32, waterline_z: f32) -> HydroSample {
        sample_field(
            self.grid,
            self.h,
            self.un,
            self.ue,
            self.still,
            n,
            e,
            waterline_z,
        )
    }
}

pub fn hydro_volume(h: &[f32], dx: f32) -> f32 {
    let mut v = 0.0;
    for &cell in h {
        if cell.is_finite() {
            v += cell;
        }
    }
    v * dx * dx
}

pub fn hydro_volume_conserved(volume0: f32, volume: f32) -> bool {
    if !volume0.is_finite() || !volume.is_finite() {
        return false;
    }
    let scale = volume0.abs().max(1.0);
    (volume - volume0).abs() <= 2.5e-3 * scale + 1e-3
}

pub fn hydro_height_nonnegative(h: &[f32]) -> bool {
    h.iter().all(|c| c.is_finite() && *c >= -1e-6)
}

pub fn hydro_finite(h: &[f32], un: &[f32], ue: &[f32]) -> bool {
    h.iter()
        .chain(un.iter())
        .chain(ue.iter())
        .all(|c| c.is_finite())
}

pub fn hydro_land_stays_dry(still: &[f32], h: &[f32]) -> bool {
    still
        .iter()
        .zip(h.iter())
        .all(|(s, height)| *s > 0.0 || *height <= HYDRO_H_DRY)
}

pub fn hydro_invariants(
    h: &[f32],
    un: &[f32],
    ue: &[f32],
    still: &[f32],
    volume0: f32,
    dx: f32,
) -> HydroInvariants {
    HydroInvariants {
        height_nonnegative: hydro_height_nonnegative(h),
        volume_conserved: hydro_volume_conserved(volume0, hydro_volume(h, dx)),
        finite: hydro_finite(h, un, ue),
        land_dry: hydro_land_stays_dry(still, h),
    }
}

/// Rusanov flux of `(h, h u)` at an interface. Mass, then momentum.
pub fn rusanov_flux(h_l: f32, u_l: f32, h_r: f32, u_r: f32, g: f32) -> [f32; 2] {
    let hl = sanitize_h(h_l);
    let hr = sanitize_h(h_r);
    let ul = if hl < HYDRO_H_DRY {
        0.0
    } else {
        finite_or(u_l)
    };
    let ur = if hr < HYDRO_H_DRY {
        0.0
    } else {
        finite_or(u_r)
    };
    let fl_m = hl * ul;
    let fr_m = hr * ur;
    let fl_q = mom_flux(hl, ul, g);
    let fr_q = mom_flux(hr, ur, g);
    let s = wavespeed(hl, ul, g).max(wavespeed(hr, ur, g));
    [
        0.5 * (fl_m + fr_m) - 0.5 * s * (hr - hl),
        0.5 * (fl_q + fr_q) - 0.5 * s * (hr * ur - hl * ul),
    ]
}

fn mom_flux(h: f32, u: f32, g: f32) -> f32 {
    if h < HYDRO_H_DRY {
        0.0
    } else {
        h * u * u + 0.5 * g * h * h
    }
}

fn wavespeed(h: f32, u: f32, g: f32) -> f32 {
    let c = if h < HYDRO_H_DRY {
        0.0
    } else {
        sqrtf((g * h).max(0.0))
    };
    u.abs() + c
}

fn sanitize_h(h: f32) -> f32 {
    if h.is_finite() {
        h.max(0.0)
    } else {
        0.0
    }
}

fn finite_or(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}

pub fn hydro_cfl_substeps(grid: HydroGrid, h: &[f32], un: &[f32], ue: &[f32], dt: f32) -> usize {
    let n = grid.cells();
    let mut hmax = 0.0;
    let mut umax = 0.0;
    for k in 0..n {
        if h[k] > hmax {
            hmax = h[k];
        }
        let u = un[k].abs() + ue[k].abs();
        if u > umax {
            umax = u;
        }
    }
    let c = sqrtf((grid.g * hmax).max(0.0)) + umax + 1e-3;
    let nsub = crate::math::ceilf(dt * c / (0.45 * grid.dx.max(1e-3))) as i32;
    nsub.clamp(1, 16) as usize
}

fn sweep(state: &mut HydroState<'_>, along_n: bool, dt: f32) {
    let grid = state.grid;
    let n = grid.cells();
    let HydroState {
        h,
        un,
        ue,
        still,
        scratch,
        grid: _,
    } = state;
    let (h_s, rest) = scratch.split_at_mut(n);
    let (un_s, ue_s) = rest.split_at_mut(n);
    h_s.copy_from_slice(&h[..n]);
    un_s.copy_from_slice(&un[..n]);
    ue_s.copy_from_slice(&ue[..n]);

    if along_n {
        for j in 0..grid.ny {
            for i in 0..grid.nx {
                let k = grid.idx(i, j);
                let fp = face_flux(grid, still, h_s, un_s, i as isize, j as isize, 1, 0);
                let fm = face_flux(grid, still, h_s, un_s, i as isize - 1, j as isize, 1, 0);
                update_cell(h, un, ue, still, k, dt, grid.dx, fp, fm, true);
            }
        }
    } else {
        for i in 0..grid.nx {
            for j in 0..grid.ny {
                let k = grid.idx(i, j);
                let fp = face_flux(grid, still, h_s, ue_s, i as isize, j as isize, 0, 1);
                let fm = face_flux(grid, still, h_s, ue_s, i as isize, j as isize - 1, 0, 1);
                update_cell(h, un, ue, still, k, dt, grid.dx, fp, fm, false);
            }
        }
    }
}

fn wet_cell(grid: HydroGrid, still: &[f32], i: isize, j: isize) -> Option<usize> {
    if !grid.in_bounds(i, j) {
        return None;
    }
    let k = grid.idx(i as usize, j as usize);
    if still[k] <= 0.0 {
        None
    } else {
        Some(k)
    }
}

/// Rusanov flux on the face whose left cell is `(i, j)` and right cell is
/// offset by `(di, dj)`. Missing or land cells become a reflecting ghost.
#[allow(clippy::too_many_arguments)]
fn face_flux(
    grid: HydroGrid,
    still: &[f32],
    h: &[f32],
    u: &[f32],
    i: isize,
    j: isize,
    di: isize,
    dj: isize,
) -> [f32; 2] {
    let left = wet_cell(grid, still, i, j);
    let right = wet_cell(grid, still, i + di, j + dj);
    let (hl, ul, hr, ur) = match (left, right) {
        (Some(l), Some(r)) => (h[l], u[l], h[r], u[r]),
        (Some(l), None) => (h[l], u[l], h[l], -u[l]),
        (None, Some(r)) => (h[r], -u[r], h[r], u[r]),
        (None, None) => (0.0, 0.0, 0.0, 0.0),
    };
    rusanov_flux(hl, ul, hr, ur, grid.g)
}

#[allow(clippy::too_many_arguments)]
fn update_cell(
    h: &mut [f32],
    un: &mut [f32],
    ue: &mut [f32],
    still: &[f32],
    k: usize,
    dt: f32,
    dx: f32,
    flux_plus: [f32; 2],
    flux_minus: [f32; 2],
    along_n: bool,
) {
    if still[k] <= 0.0 {
        h[k] = 0.0;
        un[k] = 0.0;
        ue[k] = 0.0;
        return;
    }
    let inv = dt / dx.max(1e-6);
    let h0 = h[k].max(0.0);
    let mut h1 = h0 - inv * (flux_plus[0] - flux_minus[0]);
    if !h1.is_finite() {
        h1 = 0.0;
    }
    h1 = h1.max(0.0);
    let u = if along_n { un[k] } else { ue[k] };
    let q0 = h0 * u;
    let mut q1 = q0 - inv * (flux_plus[1] - flux_minus[1]);
    if !q1.is_finite() {
        q1 = 0.0;
    }
    let u1 = if h1 < HYDRO_H_DRY { 0.0 } else { q1 / h1 };
    h[k] = h1;
    if along_n {
        un[k] = u1;
    } else {
        ue[k] = u1;
    }
}

fn apply_source(state: &mut HydroState<'_>, dt: f32, wind_n: f32, wind_e: f32) {
    let n = state.grid.cells();
    let damp = 1.0 / (1.0 + dt * HYDRO_FRICTION);
    for k in 0..n {
        if state.still[k] <= 0.0 {
            continue;
        }
        let h = state.h[k].max(HYDRO_H_DRY);
        state.un[k] = (state.un[k] + dt * HYDRO_WIND_COEFF * wind_n / h) * damp;
        state.ue[k] = (state.ue[k] + dt * HYDRO_WIND_COEFF * wind_e / h) * damp;
    }
}

fn pin_land_and_dry(state: &mut HydroState<'_>) {
    let n = state.grid.cells();
    for k in 0..n {
        if state.still[k] <= 0.0 || state.h[k] < HYDRO_H_DRY {
            state.h[k] = if state.still[k] <= 0.0 {
                0.0
            } else {
                state.h[k].max(0.0)
            };
            if state.h[k] < HYDRO_H_DRY {
                state.un[k] = 0.0;
                state.ue[k] = 0.0;
            }
        }
        if state.still[k] <= 0.0 {
            state.h[k] = 0.0;
            state.un[k] = 0.0;
            state.ue[k] = 0.0;
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn sample_field(
    grid: HydroGrid,
    h: &[f32],
    un: &[f32],
    ue: &[f32],
    still: &[f32],
    n: f32,
    e: f32,
    waterline_z: f32,
) -> HydroSample {
    let dx = grid.dx.max(1e-6);
    let fi = ((n - grid.origin_n) / dx - 0.5).clamp(0.0, (grid.nx.saturating_sub(1)) as f32);
    let fj = ((e - grid.origin_e) / dx - 0.5).clamp(0.0, (grid.ny.saturating_sub(1)) as f32);
    let i0 = fi as usize;
    let j0 = fj as usize;
    let i1 = (i0 + 1).min(grid.nx.saturating_sub(1));
    let j1 = (j0 + 1).min(grid.ny.saturating_sub(1));
    let ai = fi - i0 as f32;
    let aj = fj - j0 as f32;
    let bilerp = |a: &[f32]| {
        let v00 = a[grid.idx(i0, j0)];
        let v10 = a[grid.idx(i1, j0)];
        let v01 = a[grid.idx(i0, j1)];
        let v11 = a[grid.idx(i1, j1)];
        (1.0 - ai) * (1.0 - aj) * v00
            + ai * (1.0 - aj) * v10
            + (1.0 - ai) * aj * v01
            + ai * aj * v11
    };
    let height = bilerp(h).max(0.0);
    let still_h = bilerp(still).max(0.0);
    let un_s = bilerp(un);
    let ue_s = bilerp(ue);
    let surface_z = if still_h <= HYDRO_H_DRY {
        waterline_z
    } else {
        waterline_z + (still_h - height)
    };
    HydroSample {
        height,
        still: still_h,
        un: un_s,
        ue: ue_s,
        surface_z,
    }
}

/// Zero-mean sinusoidal column perturbation on wet cells. Volume unchanged.
pub fn apply_wave_mode(
    grid: HydroGrid,
    h: &mut [f32],
    still: &[f32],
    amp: f32,
    k_wave: f32,
    phase: f32,
) {
    let n = grid.cells();
    if h.len() < n || still.len() < n {
        return;
    }
    let amp = if amp.is_finite() { amp } else { 0.0 };
    let mut wet = 0.0;
    let mut sum = 0.0;
    for i in 0..grid.nx {
        for j in 0..grid.ny {
            let idx = grid.idx(i, j);
            if still[idx] <= 0.0 {
                h[idx] = 0.0;
                continue;
            }
            let north = grid.origin_n + (i as f32 + 0.5) * grid.dx;
            let pert = amp * crate::math::sinf(k_wave * north + phase);
            wet += 1.0;
            sum += pert;
            h[idx] = still[idx] + pert;
        }
    }
    let mean = if wet > 0.0 { sum / wet } else { 0.0 };
    for i in 0..n {
        if still[i] > 0.0 {
            h[i] = (h[i] - mean).max(0.0);
        }
    }
}

/// Two-cell periodic 1-D mass update used by proofs and tests.
/// Creusot 0.5 cannot state `h ≥ 0` (no `f32` order). Kani does.
pub fn two_cell_periodic_mass(h: [f32; 2], u: [f32; 2], dt: f32, dx: f32, g: f32) -> [f32; 2] {
    let f01 = rusanov_flux(h[0], u[0], h[1], u[1], g);
    let f10 = rusanov_flux(h[1], u[1], h[0], u[0], g);
    let inv = dt / dx.max(1e-6);
    [
        (h[0] - inv * (f01[0] - f10[0])).max(0.0),
        (h[1] - inv * (f10[0] - f01[0])).max(0.0),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Lake {
        grid: HydroGrid,
        h: Vec<f32>,
        un: Vec<f32>,
        ue: Vec<f32>,
        still: Vec<f32>,
        scratch: Vec<f32>,
    }

    fn lake(nx: usize, ny: usize, depth: f32) -> Lake {
        let n = nx * ny;
        Lake {
            grid: HydroGrid {
                nx,
                ny,
                dx: 1.0,
                g: 9.81,
                origin_n: 0.0,
                origin_e: 0.0,
            },
            h: vec![depth; n],
            un: vec![0.0; n],
            ue: vec![0.0; n],
            still: vec![depth; n],
            scratch: vec![0.0; 3 * n],
        }
    }

    #[test]
    fn lake_at_rest_stays_flat() {
        let mut lake = lake(6, 5, 4.0);
        let v0 = hydro_volume(&lake.h, lake.grid.dx);
        {
            let mut s = HydroState {
                grid: lake.grid,
                h: &mut lake.h,
                un: &mut lake.un,
                ue: &mut lake.ue,
                still: &lake.still,
                scratch: &mut lake.scratch,
            };
            for _ in 0..40 {
                s.step(0.02, 0.0, 0.0);
            }
            assert!(s.invariants(v0).all(), "{:?}", s.invariants(v0));
        }
        for c in &lake.h {
            assert!((c - 4.0).abs() < 1e-3, "{c}");
        }
        for u in lake.un.iter().chain(lake.ue.iter()) {
            assert!(u.abs() < 1e-3, "{u}");
        }
    }

    #[test]
    fn land_strip_stays_dry() {
        let mut lake = lake(8, 4, 3.0);
        for i in 5..8 {
            for j in 0..4 {
                let k = lake.grid.idx(i, j);
                lake.still[k] = 0.0;
                lake.h[k] = 0.0;
            }
        }
        let v0 = hydro_volume(&lake.h, lake.grid.dx);
        let mut s = HydroState {
            grid: lake.grid,
            h: &mut lake.h,
            un: &mut lake.un,
            ue: &mut lake.ue,
            still: &lake.still,
            scratch: &mut lake.scratch,
        };
        for _ in 0..80 {
            s.step(0.02, 1.5, -0.4);
            assert!(s.invariants(v0).all(), "{:?}", s.invariants(v0));
        }
        for i in 5..8 {
            for j in 0..4 {
                let k = lake.grid.idx(i, j);
                assert_eq!(lake.h[k], 0.0);
            }
        }
    }

    #[test]
    fn two_cell_periodic_conserves_mass() {
        let h = [1.2, 0.8];
        let u = [0.4, -0.3];
        let h1 = two_cell_periodic_mass(h, u, 0.01, 1.0, 9.81);
        assert!((h[0] + h[1] - h1[0] - h1[1]).abs() < 1e-5);
        assert!(h1[0] >= 0.0 && h1[1] >= 0.0);
    }

    #[test]
    fn wave_mode_zero_mean() {
        let mut lake = lake(10, 6, 4.0);
        apply_wave_mode(lake.grid, &mut lake.h, &lake.still, 0.2, 0.5, 0.3);
        let v = hydro_volume(&lake.h, lake.grid.dx);
        let v_still = hydro_volume(&lake.still, lake.grid.dx);
        assert!((v - v_still).abs() < 1e-3 * v_still.max(1.0));
    }

    #[test]
    fn reflecting_wall_has_zero_mass_flux() {
        let f = rusanov_flux(2.0, 0.4, 2.0, -0.4, 9.81);
        assert!(f[0].abs() < 1e-5, "{}", f[0]);
    }

    #[test]
    fn swell_propagates_but_volume_holds() {
        let mut lake = lake(12, 6, 4.0);
        apply_wave_mode(lake.grid, &mut lake.h, &lake.still, 0.15, 0.8, 0.0);
        let v0 = hydro_volume(&lake.h, lake.grid.dx);
        {
            let mut s = HydroState {
                grid: lake.grid,
                h: &mut lake.h,
                un: &mut lake.un,
                ue: &mut lake.ue,
                still: &lake.still,
                scratch: &mut lake.scratch,
            };
            for _ in 0..60 {
                s.step(0.02, 0.0, 0.0);
                assert!(s.invariants(v0).all(), "{:?}", s.invariants(v0));
            }
        }
        let spread = lake.h.iter().cloned().fold(0.0_f32, |a, b| a.max(b))
            - lake.h.iter().cloned().fold(f32::MAX, |a, b| a.min(b));
        assert!(spread > 1e-4);
    }
}
