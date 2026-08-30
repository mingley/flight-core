//! Hardware-in-the-loop rack for flight-core vehicles.
//!
//! A rack frame has a compute budget. If the plant step overruns it, the miss
//! is recorded, failsafe is tripped through attach typestate, and the next
//! actuator command is zero — the same kernel [`flight_core::hitl`] proves.
//! Compute must also finish within the OffboardControl [`flight_core::temporal::Rate`]
//! period (lockstep [`DeadlineSpec`] `period_ns`). A leftover OffboardControl
//! handle bound before the miss has no `COMMANDS` authority
//! ([`WorldRack::leftover_after_deadline_miss`]). Leftover after every
//! `REVOKE_ON` event is [`WorldRack::run_hitl_revoke_table`]. Leftover GPS-loss
//! (`EstimatorInvalid` + `AerialOffboard::GPS_LOSS_REQUIRE`) is
//! [`WorldRack::run_hitl_gps_loss`]. [`WorldRack::recover_deadline`] / [`WorldRack::grant_all`] walk recover then
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
//! plant is the verified world. A physical or mock I/O card speaks `FCH1` on
//! UDP ([`Fch1UdpCard`]); [`WorldRack::drain_io`] / [`WorldRack::frame_from_io`]
//! apply wire commands through [`RackCommand::from_fch1`]. The card does
//! **not** step the plant.

#![deny(unsafe_code)]

pub mod card;
pub mod protocol;
pub mod rack;

pub use card::{run_fch1_udp_mock, Fch1MockReport, Fch1UdpCard, Fch1WireEvent};
pub use protocol::{decode_command, decode_sample, encode_command, encode_sample, Command, Sample};
pub use rack::{command_from_datagram, HitlGpsLossReport, RackCommand, RackFrame, WorldRack};
