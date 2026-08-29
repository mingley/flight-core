use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(hull: MarineVehicle<MarineFailsafe, B>) {
    let _ = hull.declare_failsafe();
}

fn main() {}
