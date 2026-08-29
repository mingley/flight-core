use flight_core::prelude::*;
use flight_core::vehicle::MarineVehicle;

fn boom<B>(mut hull: MarineVehicle<Docked, B>) {
    let _ = hull.set_ned_velocity(Velocity::<Ned>::ned(0.5, 0.0, 0.0));
}

fn main() {}
