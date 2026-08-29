//! Headless coastal scenario: four domains, properties printed every second.
//!
//! `apply_script` walks attach typestate (same path as the live demo), then
//! one verified `step`.

use robot_lab::Lab;

fn main() {
    let mut lab = Lab::coastal(1);
    println!(
        "scenario={} seed={}",
        lab.world().scenario,
        lab.world().seed
    );
    for k in 0..1200 {
        lab.apply_script();
        lab.step(0.02);
        if k % 50 == 0 {
            let obs = lab.observe();
            let hold = if obs.all_hold { "ALL HOLD" } else { "FAIL" };
            println!("t={:.2}  properties {hold}  {}", obs.t, obs.message);
            for r in &obs.robots {
                println!(
                    "  {:<10} {:<11} {:<12} n={:7.2} e={:7.2} d={:7.2}  {} {}",
                    r.id,
                    r.domain,
                    r.phase,
                    r.n,
                    r.e,
                    r.d,
                    r.medium,
                    if r.actuators { "ACT" } else { "   " }
                );
            }
        }
        if !lab.all_hold() {
            eprintln!("property violation at t={:.2}", lab.world().t);
            for p in lab.world().last_properties {
                if !p.holds {
                    eprintln!("  FAIL {} — {}", p.id, p.detail);
                }
            }
            std::process::exit(1);
        }
    }
    println!("done · 24 s of verified coastal simulation");
}
