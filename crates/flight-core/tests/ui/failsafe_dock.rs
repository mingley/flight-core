use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(hull: MarineVehicle<MarineFailsafe, B>) {
    let _ = hull.dock_now();
}

fn main() {}
