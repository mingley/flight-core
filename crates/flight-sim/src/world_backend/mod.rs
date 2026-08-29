//! Typestate vehicle backends over one mechanically verified [`robot_world::World`].
//!
//! [`WorldSession`] is the shared plant. Aerial, ground, and marine handles
//! clone it so a drone takeoff and a rover twist step the same contact,
//! battery, and property vector.

mod aerial;
mod ground;
mod marine;
mod session;
pub(crate) mod shared;

#[cfg(test)]
mod tests;

pub use aerial::WorldBackend;
pub use ground::GroundWorldBackend;
pub use marine::MarineWorldBackend;
pub use session::{WorldImu, WorldSession};
