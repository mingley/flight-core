use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(mut vehicle: Vehicle<PreflightReady, B>, p: Position<Ned>) {
    let _ = vehicle.set_position(p);
}

fn main() {}
