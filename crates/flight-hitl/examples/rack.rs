//! Coastal HITL rack: 50 Hz frames, verified world as the plant.

use flight_hitl::{RackCommand, WorldRack};

fn main() {
    let mut rack = WorldRack::coastal(1).expect("rack");
    let cmd = RackCommand {
        aerial: [0.0, 0.0, -1.0],
        ground: [-0.3, 0.1, 0.0],
        marine: [0.0, 0.2, 0.0],
        underwater: [0.2, 0.0, 0.0],
    };
    for k in 0..100 {
        let f = rack.frame(0.02, cmd).expect("frame");
        if k % 25 == 24 {
            let w = rack.world();
            println!(
                "t={:.2}s  compute={}us  miss={}  hold={}  alt={:.2}",
                f.t,
                f.compute_ns / 1000,
                f.missed_total,
                f.all_hold,
                w.body("drone").unwrap().altitude_agl()
            );
        }
        if f.missed() {
            eprintln!("deadline miss at t={:.3}s — failsafe", f.t);
            break;
        }
    }
    println!(
        "frames={} missed={} hold={}",
        rack.frames(),
        rack.missed(),
        rack.world().all_hold()
    );
}
