use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(vehicle: Vehicle<Failsafe, B>) {
    let _ = vehicle.arm_now();
}

fn main() {}
