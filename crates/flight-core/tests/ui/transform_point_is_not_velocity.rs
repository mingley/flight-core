use flight_core::prelude::*;

fn boom(t: Transform<Ned, Body>, v: Velocity<Ned>) {
    let _ = t.apply_point(v);
}

fn main() {}
