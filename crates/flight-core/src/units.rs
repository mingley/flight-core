//! Scalar quantities always construct; unit tags are phantom.

use core::fmt;
use core::marker::PhantomData;
use core::ops::{Add, Div, Mul, Neg, Sub};

/// Marker for a physical unit (meters, rad/s, …).
pub trait Unit: Copy + Clone + fmt::Debug + Send + Sync + 'static {
    const NAME: &'static str;
}

macro_rules! define_unit {
    ($name:ident, $label:literal) => {
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        #[cfg_attr(
            all(feature = "serde", not(creusot)),
            derive(serde::Serialize, serde::Deserialize)
        )]
        pub struct $name;
        impl Unit for $name {
            const NAME: &'static str = $label;
        }
    };
}

define_unit!(Meter, "m");
define_unit!(MeterPerSecond, "m/s");
define_unit!(MeterPerSecondSquared, "m/s²");
define_unit!(Radian, "rad");
define_unit!(RadianPerSecond, "rad/s");
define_unit!(Degree, "deg");
define_unit!(DegreePerSecond, "deg/s");
define_unit!(Celsius, "°C");
define_unit!(Second, "s");
define_unit!(Newton, "N");
define_unit!(NewtonMeter, "N·m");
define_unit!(Kilogram, "kg");
define_unit!(KilogramPerCubicMeter, "kg/m³");
define_unit!(Dimensionless, "1");

/// Scalar quantity tagged with a unit.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Qty<U> {
    value: f32,
    #[cfg_attr(all(feature = "serde", not(creusot)), serde(skip))]
    _unit: PhantomData<U>,
}

impl<U> PartialEq for Qty<U> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<U> Qty<U> {
    pub const fn new(value: f32) -> Self {
        Self {
            value,
            _unit: PhantomData,
        }
    }

    pub const fn get(self) -> f32 {
        self.value
    }

    pub fn is_finite(self) -> bool {
        self.value.is_finite()
    }

    pub fn abs(self) -> Self {
        Self::new(self.value.abs())
    }
}

#[cfg(not(creusot))]
impl<U: Unit> fmt::Display for Qty<U> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.value, U::NAME)
    }
}

impl<U> Add for Qty<U> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.value + rhs.value)
    }
}

impl<U> Sub for Qty<U> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.value - rhs.value)
    }
}

impl<U> Neg for Qty<U> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.value)
    }
}

impl<U> Mul<f32> for Qty<U> {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.value * rhs)
    }
}

impl<U> Div<f32> for Qty<U> {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self::new(self.value / rhs)
    }
}

impl Qty<Degree> {
    pub fn to_radians(self) -> Qty<Radian> {
        Qty::new(self.value * core::f32::consts::PI / 180.0)
    }
}

impl Qty<Radian> {
    pub fn to_degrees(self) -> Qty<Degree> {
        Qty::new(self.value * 180.0 / core::f32::consts::PI)
    }
}

impl Qty<DegreePerSecond> {
    pub fn to_radians(self) -> Qty<RadianPerSecond> {
        Qty::new(self.value * core::f32::consts::PI / 180.0)
    }
}

impl Qty<RadianPerSecond> {
    pub fn to_degrees(self) -> Qty<DegreePerSecond> {
        Qty::new(self.value * 180.0 / core::f32::consts::PI)
    }
}

pub type Meters = Qty<Meter>;
pub type Seconds = Qty<Second>;

impl Meters {
    pub const fn from_meters(m: f32) -> Self {
        Self::new(m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_unit_adds() {
        let a = Qty::<Meter>::new(1.0);
        let b = Qty::<Meter>::new(2.5);
        assert!((a + b).get() - 3.5 < 1e-6);
    }
}
