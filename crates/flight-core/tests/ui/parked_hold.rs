use flight_core::prelude::*;
use flight_core::vehicle::GroundVehicle;

fn boom<B>(mut rover: GroundVehicle<Parked, B>) {
    let _ = rover.hold_now();
}

fn main() {}
