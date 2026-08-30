use flight_core::prelude::*;

fn need_omega(_: AngularVelocity<RadianPerSecond, Body>) {}

fn boom(o: Orientation<Body>) {
    need_omega(o);
}

fn main() {}
