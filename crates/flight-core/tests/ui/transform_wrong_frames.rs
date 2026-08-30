use flight_core::prelude::*;

fn boom(a: Transform<Ned, Body>, b: Transform<Ned, Enu>) {
    let _ = a.then(b);
}

fn main() {}
