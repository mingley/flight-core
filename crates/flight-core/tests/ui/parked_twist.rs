use flight_core::prelude::*;
use flight_core::vehicle::GroundVehicle;

fn boom<B>(mut rover: GroundVehicle<Parked, B>) {
    let _ = rover.set_twist(
        Velocity::<Body>::new(1.0, 0.0, 0.0),
        AngularVelocity::body_rad(0.0, 0.0, 0.0),
    );
}

fn main() {}
