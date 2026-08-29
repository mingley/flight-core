//! Typestate vehicle handle and backend trait.

pub mod backend;
pub mod ground;
pub mod marine;
pub mod typestate;

pub use backend::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, NullBackend, PreflightNotes,
    PreflightReport, Telemetry, VehicleBackend,
};
pub use ground::{
    body_xy_to_ned, ground_kind, CanTripEstop, EStopped, GroundError, GroundHandle, GroundKind,
    GroundVehicle, Moving, Parked,
};
pub use marine::{
    marine_kind, CanDock, CanThrust, CanTripMarineFailsafe, Docked, MarineError, MarineFailsafe,
    MarineHandle, MarineKind, MarineVehicle, StationKeep, Underway,
};
pub use typestate::{
    aerial_kind, AerialKind, Airborne, Armed, CanBeginLand, CanDisarm, CanTouchdown,
    CanTripFailsafe, Disarmed, Disconnected, ErrorKind, Failsafe, Landing, MotorsEnabled, Offboard,
    OffboardControl, PreflightReady, Recovery, State, Takeoff, TransitionError, Vehicle,
    VehicleHandle,
};
