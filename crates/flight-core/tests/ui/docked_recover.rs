use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(hull: MarineVehicle<Docked, B>) {
    let _ = hull.recover_docked();
}

fn main() {}
