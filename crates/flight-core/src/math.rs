//! Tiny `no_std` math. `f32::{sqrt, ceil, sin}` live in `std`.

pub(crate) fn sqrtf(x: f32) -> f32 {
    if !x.is_finite() || x < 0.0 {
        return 0.0;
    }
    if x == 0.0 {
        return 0.0;
    }
    let mut y = f32::from_bits((x.to_bits() + 0x3f80_0000) >> 1);
    y = 0.5 * (y + x / y);
    y = 0.5 * (y + x / y);
    y = 0.5 * (y + x / y);
    y
}

/// Toward +∞. Inputs that do not fit in `i32` are treated as 0.
pub(crate) fn ceilf(x: f32) -> f32 {
    if !x.is_finite() || x > i32::MAX as f32 || x < i32::MIN as f32 {
        return 0.0;
    }
    let trunc = x as i32;
    if x > 0.0 && x > trunc as f32 {
        (trunc + 1) as f32
    } else {
        trunc as f32
    }
}

/// Sine on ℝ via range reduction to [-π/2, π/2] + Taylor.
pub(crate) fn sinf(mut x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    const PI: f32 = core::f32::consts::PI;
    const TAU: f32 = 2.0 * PI;
    x %= TAU;
    if x > PI {
        x -= TAU;
    } else if x < -PI {
        x += TAU;
    }
    if x > PI * 0.5 {
        x = PI - x;
    } else if x < -PI * 0.5 {
        x = -PI - x;
    }
    let x2 = x * x;
    x * (1.0 - x2 * (1.0 / 6.0) * (1.0 - x2 * (1.0 / 20.0) * (1.0 - x2 * (1.0 / 42.0))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sqrt_matches_std() {
        for v in [0.0, 1.0, 4.0, 9.0, 2.0, 9.81] {
            let a = sqrtf(v);
            let b = v.sqrt();
            assert!((a - b).abs() < 1e-5, "{v}: {a} vs {b}");
        }
    }

    #[test]
    fn ceil_matches_std() {
        for v in [0.0, 0.1, 1.0, 1.9, 16.0, -0.1, -1.7] {
            let a = ceilf(v);
            let b = v.ceil();
            assert!((a - b).abs() < 1e-6, "{v}: {a} vs {b}");
        }
    }

    #[test]
    fn sin_matches_std() {
        for v in [0.0_f32, 0.3, 1.1, -0.8, 3.2, 6.5] {
            let a = sinf(v);
            let b = v.sin();
            assert!((a - b).abs() < 2e-3, "{v}: {a} vs {b}");
        }
    }
}
