use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Takeoff, B>) {
    let _ = vehicle.recover_now();
}

fn main() {}
