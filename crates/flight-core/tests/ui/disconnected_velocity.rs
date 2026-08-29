use flight_core::prelude::*;
use flight_core::vehicle::Vehicle;

fn boom<B>(mut vehicle: Vehicle<Disconnected, B>, v: Velocity<Ned>) {
    let _ = vehicle.set_velocity(v);
}

fn main() {}
