use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Airborne, B>) {
    let _ = vehicle.touchdown_now();
}

fn main() {}
