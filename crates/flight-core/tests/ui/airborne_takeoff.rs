use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Airborne, B>) {
    let _ = vehicle.start_takeoff_now();
}

fn main() {}
