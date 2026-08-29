//! Typestate vehicle backends over one mechanically verified [`World`].
//!
//! [`WorldSession`] is the shared plant. Aerial, ground, and marine handles
//! clone it (`Arc<Mutex<_>>`) so a drone takeoff and a rover twist step the
//! same contact, battery, and property vector. `tick` on any handle advances
//! the whole scene — use one ticker per frame, or accept passenger motion.
