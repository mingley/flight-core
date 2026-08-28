use flight_core::prelude::*;

fn boom(
    a: AngularVelocity<RadianPerSecond, Body>,
    b: AngularVelocity<DegreePerSecond, Body>,
) {
    let _ = a + b;
}

fn main() {}
