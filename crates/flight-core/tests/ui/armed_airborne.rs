use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Armed, B>) {
    let _ = vehicle.declare_airborne_now();
}

fn main() {}
