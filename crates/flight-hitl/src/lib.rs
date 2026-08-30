//! Hardware-in-the-loop rack for flight-core vehicles.
//!
//! A rack frame has a compute budget. If the plant step overruns it, the miss
//! is recorded, failsafe is tripped through attach typestate, and the next
//! actuator command is zero — the same kernel [`flight_core::hitl`] proves.
//! Compute must also finish within the OffboardControl [`flight_core::temporal::Rate`]
//! period (lockstep [`DeadlineSpec`] `period_ns`). A leftover OffboardControl
//! handle bound before the miss has no `COMMANDS` authority
//! ([`WorldRack::leftover_after_deadline_miss`]). [`WorldRack::recover_deadline`] / [`WorldRack::grant_all`] walk recover then
//! re-grant so a later on-time frame can command again. [`WorldRack::return_all`]
//! walks land+touchdown / park / dock home (skipping bodies the catalog
//! omitted: inland has no hull, open water has no rover). [`WorldRack::airborne`]
//! / [`WorldRack::station_all`] / [`WorldRack::resume_all`] / [`WorldRack::dock_all`]
//! / [`WorldRack::park_all`] / [`WorldRack::hold`] walk climb-complete, hull
//! station/resume, hull dock, rover halt, and a drone NED pose hold on the
//! same plant. Idle FCH1 frames leave that hold in place; a live velocity
//! frame clears it. After [`WorldRack::return_all`] the drone is Ready, so
//! hold is Protocol. [`WorldRack::harbor`]
//! is the four-body shoreline; [`WorldRack::open_water`] is air + hulls. The
//! plant can be the verified world or a UDP I/O card speaking the `FCH1`
//! datagrams in [`protocol`].

#![deny(unsafe_code)]

pub mod protocol;
pub mod rack;

pub use protocol::{decode_command, decode_sample, encode_command, encode_sample, Command, Sample};
pub use rack::{command_from_datagram, RackCommand, RackFrame, WorldRack};
