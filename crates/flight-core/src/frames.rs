//! Reference frames as types.
//!
//! Adding `Position<Ned>` to `Position<Enu>` does not compile:
//!
//! ```compile_fail
//! use flight_core::prelude::*;
//! fn boom(a: Position<Ned>, b: Position<Enu>) {
//!     let _ = a + b;
//! }
//! ```

use core::fmt;

/// Marker for a coordinate reference frame.
pub trait Frame: Copy + Clone + fmt::Debug + Send + Sync + 'static {
    const NAME: &'static str;
}

macro_rules! define_frame {
    ($(#[$attr:meta])* $name:ident, $label:literal) => {
        $(#[$attr])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        #[cfg_attr(all(feature = "serde", not(creusot)), derive(serde::Serialize, serde::Deserialize))]
        pub struct $name;
        impl Frame for $name {
            const NAME: &'static str = $label;
        }
    };
}

define_frame!(
    /// North-East-Down (PX4 local frame).
    Ned,
    "NED"
);
define_frame!(
    /// East-North-Up (common ROS / geographic frame).
    Enu,
    "ENU"
);
define_frame!(
    /// Vehicle body frame (x forward, y right, z down when aligned with NED).
    Body,
    "BODY"
);
define_frame!(
    /// Forward-Right-Down, alias of the usual aircraft body frame.
    Frd,
    "FRD"
);
define_frame!(
    /// Geographic WGS-84 (lat/lon/alt semantics; not a Cartesian tangent frame).
    Wgs84,
    "WGS84"
);
