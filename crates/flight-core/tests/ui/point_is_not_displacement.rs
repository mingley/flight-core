use flight_core::prelude::*;

fn need_point(_: Point3<Ned>) {}

fn boom(d: Displacement<Ned>) {
    need_point(d);
}

fn main() {}
