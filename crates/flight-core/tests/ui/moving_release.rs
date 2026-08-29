use flight_core::prelude::*;
use flight_core::vehicle::GroundVehicle;

fn boom<B>(rover: GroundVehicle<Moving, B>) {
    let _ = rover.enable_drive();
}

fn main() {}
