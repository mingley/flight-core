use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(mut hull: MarineVehicle<Docked, B>) {
    let _ = hull.hold_now();
}

fn main() {}
