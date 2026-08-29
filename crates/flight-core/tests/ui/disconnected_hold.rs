use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(mut vehicle: Vehicle<Disconnected, B>) {
    let _ = vehicle.hold();
}

fn main() {}
