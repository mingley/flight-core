//! Vehicle domain and surrounding medium as types, not strings.

use core::fmt;

/// Where the robot is designed to work. Mixing domains at the API is a type error
/// once you hold a domain-tagged handle (`GroundVehicle`, `Vessel`, `Vehicle`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
#[repr(u8)]
pub enum Domain {
    Aerial = 0,
    Ground = 1,
    Surface = 2,
    Underwater = 3,
}

impl Domain {
    pub const fn name(self) -> &'static str {
        match self {
            Domain::Aerial => "aerial",
            Domain::Ground => "ground",
            Domain::Surface => "surface",
            Domain::Underwater => "underwater",
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Domain::Aerial),
            1 => Some(Domain::Ground),
            2 => Some(Domain::Surface),
            3 => Some(Domain::Underwater),
            _ => None,
        }
    }
}

#[cfg(not(creusot))]
impl fmt::Display for Domain {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What the hull/wheels/rotors are immersed in at a sample point.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(
    all(feature = "serde", not(creusot)),
    derive(serde::Serialize, serde::Deserialize)
)]
#[repr(u8)]
pub enum Medium {
    Air = 0,
    Water = 1,
    Soil = 2,
}

impl Medium {
    pub const fn name(self) -> &'static str {
        match self {
            Medium::Air => "air",
            Medium::Water => "water",
            Medium::Soil => "soil",
        }
    }

    pub const fn density_kg_m3(self) -> f32 {
        match self {
            Medium::Air => 1.225,
            Medium::Water => 1025.0,
            Medium::Soil => 0.0,
        }
    }
}
