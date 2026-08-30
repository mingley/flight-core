use flight_core::prelude::*;

fn need_torque(_: Torque<Body>) {}

fn boom(f: Force<Body>) {
    need_torque(f);
}

fn main() {}
