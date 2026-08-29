//! Typestate vehicle backends over one mechanically verified [`World`].
//!
//! [`WorldSession`] is the shared plant. Aerial, ground, and marine handles
//! clone it (`Arc<Mutex<_>>`) so a drone takeoff and a rover twist step the
//! same contact, battery, and property vector. `tick` on any handle advances
//! the whole scene — use one ticker per frame, or accept passenger motion.
//! [`WorldSession::attach_takeoff`] / [`WorldSession::attach_drive`] /
//! [`WorldSession::attach_undock`] / [`WorldSession::attach_land`] /
//! [`WorldSession::attach_touchdown`] / [`WorldSession::attach_airborne`] /
//! [`WorldSession::attach_hold`] /
//! [`WorldSession::attach_failsafe`] / [`WorldSession::attach_reset`] /
//! [`WorldSession::attach_marine_failsafe`] / [`WorldSession::attach_recover`] /
//! [`WorldSession::attach_recover_ready`] / [`WorldSession::attach_disarm`] /
//! [`WorldSession::attach_touchdown`]
//! walk consume-self typestate then return the live backend HITL, ROS 2, PX4,
//! and research agents command through.
