use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Recovery, B>) {
    let _ = vehicle.failsafe_now();
}

fn main() {}
