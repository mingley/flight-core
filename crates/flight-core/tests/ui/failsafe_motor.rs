use flight_core::prelude::*;
use flight_core::vehicle::{MotorThrust, Vehicle};

fn boom<B>(mut vehicle: Vehicle<Failsafe, B>, thrust: MotorThrust) {
    let _ = vehicle.set_motor_thrust(thrust);
}

fn main() {}
