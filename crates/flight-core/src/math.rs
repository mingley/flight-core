//! Tiny `no_std` math. `f32::sqrt` lives in `std`.

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
}
