use flight_core::prelude::*;

fn need_accel(_: Acceleration<Ned>) {}

fn boom(v: Velocity<Ned>) {
    need_accel(v);
}

fn main() {}
