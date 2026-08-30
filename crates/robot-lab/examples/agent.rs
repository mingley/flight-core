//! Closed-loop research agent: observe JSON, act, step, print a property certificate.
//!
//! Inland uses the rover probe. `typed` and the default mixed-world agent use
//! [`TypedFleet`] (JSON illegal probes, then `Lab::attach_takeoff` /
//! `attach_drive` / `attach_undock`). `typed-attach` uses [`TypedAttachFleet`]
//! (same attach policy, distinct certificate name).
//! `pad` uses [`PadLanding`] on inland
//! (climb off terrain contact, then land until the pad returns). `typed-pad` is
//! the same policy through `Lab::attach_airborne` / `attach_land` / `attach_touchdown`
//! so `actions_applied` stays 0 and the log still replays. `collision` uses [`CollisionSweep`] on inland
//! (drive the rover into the drone, prove spheres still separate). `typed-collision`
//! is the same policy through `Lab::attach_drive` / `attach_park`. `typed-station` uses
//! [`TypedStationDock`] on coastal (`Lab::attach_undock` / `attach_station` / `attach_dock`).
//! `typed-hull-dock` uses [`TypedHullDock`] (`Lab::attach_undock` / `attach_dock`
//! from Underway — no station).
//! `typed-station-resume` uses [`TypedStationResume`] (`Lab::attach_undock` /
//! `attach_station` / `attach_resume` — no dock). `typed-failsafe` uses [`TypedHullFailsafe`] (`Lab::attach_undock` /
//! `attach_marine_failsafe` / `attach_recover`). `typed-aerial` uses
//! [`TypedAerialFailsafe`] (`Lab::attach_takeoff` / `attach_failsafe` /
//! `attach_recover_ready`). `typed-aerial-disarm` uses [`TypedAerialDisarm`]
//! (`Lab::attach_takeoff` / `attach_disarm` — no failsafe).
//! `typed-aerial-airborne` uses [`TypedAerialAirborne`]
//! (`Lab::attach_takeoff` / `attach_airborne` / `attach_land` — no touchdown).
//! `typed-position-hold` uses [`TypedPositionHold`]
//! (`Lab::attach_takeoff` / `set_position_now` — no airborne or land).
//! `typed-hold` uses [`TypedHold`]
//! (`Lab::attach_takeoff` / `attach_hold` — current pose, not d=−2).
//! `typed-fleet-hold` uses [`TypedFleetHold`] (`Lab::attach_takeoff` /
//! `attach_drive` / `attach_undock`, then `attach_hold` / `attach_station`).
//! `typed-pad-disarm` uses [`TypedPadDisarm`]
//! (`Lab::attach_disarm` from Ready — no takeoff).
//! `typed-pad-failsafe` uses [`TypedPadFailsafe`]
//! (`Lab::attach_failsafe` from Ready, then `attach_recover_ready` — no takeoff).
//! `typed-ground-estop` uses [`TypedGroundEstop`] (`Lab::attach_estop` from
//! Parked, then `attach_reset` — no drive grant).
//! `typed-ground-halt` uses [`TypedGroundHalt`] (`Lab::attach_drive` /
//! `attach_park` — no E-stop).
//! `typed-ground-hold` uses [`TypedGroundHold`] (`Lab::attach_drive` /
//! `attach_ground_hold` — current NED pose).
//! `typed-fleet-return` uses [`TypedFleetReturn`] (`Lab::attach_takeoff` /
//! `attach_drive` / `attach_undock`, then `attach_land` / `attach_touchdown` /
//! `attach_park` / `attach_dock`).
//! `typed-station-failsafe` uses [`TypedStationFailsafe`] (`Lab::attach_undock` /
//! `attach_station` / `attach_marine_failsafe` / `attach_recover`).
//! `typed-failsafe-touchdown` uses [`TypedFailsafeTouchdown`]
//! (`Lab::attach_failsafe` from Ready, then `attach_touchdown` — no Recovery).
//! `typed-surveyor` uses [`TypedSurveyorFailsafe`] (`Lab::attach_undock` /
//! `attach_marine_failsafe` / `attach_recover` on the AUV).
//! `typed-surveyor-station` uses [`TypedSurveyorStationFailsafe`]
//! (`Lab::attach_undock` / `attach_station` / `attach_marine_failsafe` /
//! `attach_recover` on the AUV).
//! `typed-surveyor-station-dock` uses [`TypedSurveyorStationDock`]
//! (`Lab::attach_undock` / `attach_station` / `attach_dock` on the AUV).
//! `typed-surveyor-dock` uses [`TypedSurveyorDock`]
//! (`Lab::attach_undock` / `attach_dock` from Underway — no station).
//! `typed-surveyor-station-resume` uses [`TypedSurveyorStationResume`]
//! (`Lab::attach_undock` / `attach_station` / `attach_resume` on the AUV).
//! `scripted` uses [`ScriptedCoastal`] (the demo
//! attach policy as a property certificate). Other worlds use [`TypedFleet`].

use robot_lab::{
    CollisionSweep, Lab, PadLanding, RoverProbe, ScriptedCoastal, TypedAerialAirborne,
    TypedAerialDisarm, TypedAerialFailsafe, TypedAttachFleet, TypedCollisionSweep,
    TypedFailsafeTouchdown, TypedFleet, TypedFleetHold, TypedFleetReturn, TypedGroundEstop,
    TypedGroundHalt, TypedGroundHold, TypedHold, TypedHullDock, TypedHullFailsafe, TypedPadDisarm,
    TypedPadFailsafe, TypedPadLanding, TypedPositionHold, TypedStationDock, TypedStationFailsafe,
    TypedStationResume, TypedSurveyorDock, TypedSurveyorFailsafe, TypedSurveyorStationDock,
    TypedSurveyorStationFailsafe, TypedSurveyorStationResume,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let first = args.next().unwrap_or_else(|| "coastal".into());
    let (scenario, kind) = match first.as_str() {
        "typed" => (args.next().unwrap_or_else(|| "coastal".into()), Kind::Typed),
        "typed-attach" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedAttach,
        ),
        "typed-pad" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedPad,
        ),
        "pad" => (args.next().unwrap_or_else(|| "inland".into()), Kind::Pad),
        "collision" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::Collision,
        ),
        "typed-collision" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedCollision,
        ),
        "typed-station" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedStation,
        ),
        "typed-hull-dock" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedHullDock,
        ),
        "typed-station-resume" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedStationResume,
        ),
        "typed-failsafe" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedFailsafe,
        ),
        "typed-aerial" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedAerial,
        ),
        "typed-aerial-disarm" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedAerialDisarm,
        ),
        "typed-aerial-airborne" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedAerialAirborne,
        ),
        "typed-position-hold" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedPositionHold,
        ),
        "typed-hold" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedHold,
        ),
        "typed-fleet-hold" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedFleetHold,
        ),
        "typed-pad-disarm" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedPadDisarm,
        ),
        "typed-pad-failsafe" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedPadFailsafe,
        ),
        "typed-ground-estop" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedGroundEstop,
        ),
        "typed-ground-halt" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedGroundHalt,
        ),
        "typed-ground-hold" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedGroundHold,
        ),
        "typed-fleet-return" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedFleetReturn,
        ),
        "typed-station-failsafe" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedStationFailsafe,
        ),
        "typed-failsafe-touchdown" => (
            args.next().unwrap_or_else(|| "inland".into()),
            Kind::TypedFailsafeTouchdown,
        ),
        "typed-surveyor" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedSurveyor,
        ),
        "typed-surveyor-station" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedSurveyorStation,
        ),
        "typed-surveyor-station-dock" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedSurveyorStationDock,
        ),
        "typed-surveyor-dock" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedSurveyorDock,
        ),
        "typed-surveyor-station-resume" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::TypedSurveyorStationResume,
        ),
        "scripted" => (
            args.next().unwrap_or_else(|| "coastal".into()),
            Kind::Scripted,
        ),
        _ => (first, Kind::Auto),
    };
    let mut lab = Lab::open(&scenario, 3).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let run = match kind {
        Kind::Typed => lab.research(&mut TypedFleet::default(), 0.02, 200),
        Kind::TypedAttach => lab.research(&mut TypedAttachFleet::default(), 0.02, 200),
        Kind::TypedPad => lab.research(&mut TypedPadLanding::default(), 0.02, 400),
        Kind::Pad => lab.research(&mut PadLanding::default(), 0.02, 400),
        Kind::Collision => lab.research(&mut CollisionSweep::default(), 0.02, 240),
        Kind::TypedCollision => lab.research(&mut TypedCollisionSweep::default(), 0.02, 240),
        Kind::TypedStation => lab.research(&mut TypedStationDock::default(), 0.02, 240),
        Kind::TypedHullDock => lab.research(&mut TypedHullDock::default(), 0.02, 240),
        Kind::TypedStationResume => lab.research(&mut TypedStationResume::default(), 0.02, 40),
        Kind::TypedFailsafe => lab.research(&mut TypedHullFailsafe::default(), 0.02, 40),
        Kind::TypedAerial => lab.research(&mut TypedAerialFailsafe::default(), 0.02, 40),
        Kind::TypedAerialDisarm => lab.research(&mut TypedAerialDisarm::default(), 0.02, 40),
        Kind::TypedAerialAirborne => lab.research(&mut TypedAerialAirborne::default(), 0.02, 40),
        Kind::TypedPositionHold => lab.research(&mut TypedPositionHold::default(), 0.02, 40),
        Kind::TypedHold => lab.research(&mut TypedHold::default(), 0.02, 40),
        Kind::TypedFleetHold => lab.research(&mut TypedFleetHold::default(), 0.02, 40),
        Kind::TypedPadDisarm => lab.research(&mut TypedPadDisarm::default(), 0.02, 40),
        Kind::TypedPadFailsafe => lab.research(&mut TypedPadFailsafe::default(), 0.02, 40),
        Kind::TypedGroundEstop => lab.research(&mut TypedGroundEstop::default(), 0.02, 40),
        Kind::TypedGroundHalt => lab.research(&mut TypedGroundHalt::default(), 0.02, 40),
        Kind::TypedGroundHold => lab.research(&mut TypedGroundHold::default(), 0.02, 40),
        Kind::TypedFleetReturn => lab.research(&mut TypedFleetReturn::default(), 0.02, 40),
        Kind::TypedStationFailsafe => lab.research(&mut TypedStationFailsafe::default(), 0.02, 40),
        Kind::TypedFailsafeTouchdown => {
            lab.research(&mut TypedFailsafeTouchdown::default(), 0.02, 40)
        }
        Kind::TypedSurveyor => lab.research(&mut TypedSurveyorFailsafe::default(), 0.02, 40),
        Kind::TypedSurveyorStation => {
            lab.research(&mut TypedSurveyorStationFailsafe::default(), 0.02, 40)
        }
        Kind::TypedSurveyorStationDock => {
            lab.research(&mut TypedSurveyorStationDock::default(), 0.02, 240)
        }
        Kind::TypedSurveyorDock => lab.research(&mut TypedSurveyorDock::default(), 0.02, 240),
        Kind::TypedSurveyorStationResume => {
            lab.research(&mut TypedSurveyorStationResume::default(), 0.02, 40)
        }
        Kind::Scripted => lab.research(&mut ScriptedCoastal, 0.02, 400),
        Kind::Auto if scenario == "inland" => lab.research(&mut RoverProbe::default(), 0.02, 120),
        Kind::Auto => lab.research(&mut TypedFleet::default(), 0.02, 200),
    };
    println!("{}", serde_json::to_string_pretty(&run).unwrap());
    if !run.ok() {
        std::process::exit(1);
    }
}

enum Kind {
    Typed,
    TypedAttach,
    TypedPad,
    Pad,
    Collision,
    TypedCollision,
    TypedStation,
    TypedHullDock,
    TypedStationResume,
    TypedFailsafe,
    TypedAerial,
    TypedAerialDisarm,
    TypedAerialAirborne,
    TypedPositionHold,
    TypedHold,
    TypedFleetHold,
    TypedPadDisarm,
    TypedPadFailsafe,
    TypedGroundEstop,
    TypedGroundHalt,
    TypedGroundHold,
    TypedFleetReturn,
    TypedStationFailsafe,
    TypedFailsafeTouchdown,
    TypedSurveyor,
    TypedSurveyorStation,
    TypedSurveyorStationDock,
    TypedSurveyorDock,
    TypedSurveyorStationResume,
    Scripted,
    Auto,
}
