//! Record agent actions as JSONL, then replay them into a fresh lab through
//! attach typestate (JSON fallback for environment / Protocol).

use robot_lab::{AgentAction, Lab, LabCmd};

fn main() {
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "inland".into());
    let mut live = Lab::open(&scenario, 7).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let _ = live.act_through_attach(AgentAction::new("rover", LabCmd::Release));
    let _ = live.act_through_attach(AgentAction::new("rover", LabCmd::Drive).ned(-0.6, 0.1, 0.0));
    for _ in 0..120 {
        live.step(0.02);
    }

    let mut replay = Lab::open(&scenario, 7).expect("same scenario");
    replay
        .replay_until(&live.log, 0.02, live.world().t)
        .expect("replay");

    let live_world = live.world();
    let replay_world = replay.world();
    let a = live_world.body("rover").expect("rover");
    let b = replay_world.body("rover").expect("rover");
    println!(
        "scenario={scenario} seed={} actions={} t={:.2}",
        live_world.seed,
        live.log.len(),
        live_world.t
    );
    println!(
        "live   rover n={:.4} e={:.4} d={:.4}",
        a.position_m[0], a.position_m[1], a.position_m[2]
    );
    println!(
        "replay rover n={:.4} e={:.4} d={:.4}",
        b.position_m[0], b.position_m[1], b.position_m[2]
    );
    println!(
        "properties {}",
        if replay.all_hold() {
            "ALL HOLD"
        } else {
            "FAIL"
        }
    );
}
