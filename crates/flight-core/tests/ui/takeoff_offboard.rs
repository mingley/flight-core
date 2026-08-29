use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Takeoff, B>) {
    let _ = vehicle.enter_offboard_now();
}

fn main() {}
