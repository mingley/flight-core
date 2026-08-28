//! Typestate vehicle handle and backend trait.

pub mod backend;
pub mod typestate;

pub use backend::{
    AutopilotKind, BackendError, ConnectionInfo, MotorThrust, NullBackend, PreflightNotes,
    PreflightReport, Telemetry, VehicleBackend,
};
pub use typestate::{
    Airborne, Armed, Disarmed, Disconnected, ErrorKind, Failsafe, Landing, MotorsEnabled, Offboard,
    OffboardControl, PreflightReady, State, TransitionError, Vehicle,
};
