use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(hull: MarineVehicle<StationKeep, B>) {
    let _ = hull.hold_station();
}

fn main() {}
