//! Closed-loop experiment runner (NEXT A3).
//!
//! One [`Lab::research_with`] tick is observe → act → **one**
//! [`crate::WorldSession::step`] (P12). Artifacts land in a run directory.

use crate::bag::McapBag;
use crate::research::ResearchAgent;
use crate::{
    AgentAction, CoastalFleet, CollisionSweep, Lab, LabError, Observation, PadLanding, ResearchRun,
    RoverProbe, ScriptedCoastal, TimedAction, TypedAerialAirborne, TypedAerialDisarm,
    TypedAerialFailsafe, TypedAttachFleet, TypedCollisionSweep, TypedFailsafeTouchdown, TypedFleet,
    TypedFleetHold, TypedFleetReturn, TypedGroundEstop, TypedGroundHalt, TypedGroundHold,
    TypedHold, TypedHullDock, TypedHullFailsafe, TypedPadDisarm, TypedPadFailsafe, TypedPadLanding,
    TypedPositionHold, TypedStationDock, TypedStationFailsafe, TypedStationResume,
    TypedSurveyorDock, TypedSurveyorFailsafe, TypedSurveyorStationDock,
    TypedSurveyorStationFailsafe, TypedSurveyorStationResume,
};
use serde::Serialize;
use std::cell::RefCell;
use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

/// What to run, and where to write it.
#[derive(Clone, Debug)]
pub struct Experiment {
    pub scenario: String,
    pub seeds: Vec<u64>,
    pub dt: f32,
    pub steps: u32,
    /// Typed agent name (`typed-fleet-hold`, `rover_probe`, …). Ignored when
    /// [`Self::jsonl`] is set.
    pub agent: String,
    pub jsonl: Option<PathBuf>,
    pub out: PathBuf,
    pub mcap: bool,
    pub require_property: Option<String>,
}

/// `run.json` for one seed.
#[derive(Clone, Debug, Serialize)]
pub struct RunRecord {
    pub git_commit: Option<String>,
    pub dt: f32,
    pub steps_requested: u32,
    pub require_property: Option<String>,
    pub property_ok: bool,
    pub run: ResearchRun,
}

/// Sweep result written as `summary.json`.
#[derive(Clone, Debug, Serialize)]
pub struct ExperimentSummary {
    pub git_commit: Option<String>,
    pub scenario: String,
    pub agent: String,
    pub seeds: Vec<u64>,
    pub dt: f32,
    pub steps: u32,
    pub all_ok: bool,
    pub require_property: Option<String>,
    pub records: Vec<RunRecord>,
}

#[derive(Debug)]
pub enum RunError {
    Lab(LabError),
    Io(io::Error),
    Json(serde_json::Error),
    Agent(String),
}

impl std::fmt::Display for RunError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunError::Lab(e) => write!(f, "{e}"),
            RunError::Io(e) => write!(f, "{e}"),
            RunError::Json(e) => write!(f, "{e}"),
            RunError::Agent(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for RunError {}

impl From<LabError> for RunError {
    fn from(e: LabError) -> Self {
        RunError::Lab(e)
    }
}

impl From<io::Error> for RunError {
    fn from(e: io::Error) -> Self {
        RunError::Io(e)
    }
}

impl From<serde_json::Error> for RunError {
    fn from(e: serde_json::Error) -> Self {
        RunError::Json(e)
    }
}

/// `git rev-parse HEAD` when the lab is running inside a checkout.
pub fn git_head() -> Option<String> {
    let out = std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?;
    let s = s.trim();
    if s.is_empty() {
        None
    } else {
        Some(s.to_string())
    }
}

impl Experiment {
    /// Run every seed. Writes `summary.json` plus per-seed `run.json`,
    /// `observations.jsonl`, `actions.jsonl`, and optional `bag.mcap`.
    pub fn execute(&self) -> Result<ExperimentSummary, RunError> {
        if self.seeds.is_empty() {
            return Err(RunError::Agent("no seeds".into()));
        }
        fs::create_dir_all(&self.out)?;
        let git_commit = git_head();
        let mut records = Vec::new();
        let mut all_ok = true;
        let multi = self.seeds.len() > 1;
        for seed in &self.seeds {
            let dir = if multi {
                self.out.join(format!("seed-{seed}"))
            } else {
                self.out.clone()
            };
            let record = self.run_seed(*seed, &dir, git_commit.clone())?;
            if !record.run.all_hold || !record.property_ok {
                all_ok = false;
            }
            records.push(record);
        }
        let summary = ExperimentSummary {
            git_commit,
            scenario: self.scenario.clone(),
            agent: if self.jsonl.is_some() {
                "jsonl".into()
            } else {
                self.agent.clone()
            },
            seeds: self.seeds.clone(),
            dt: self.dt,
            steps: self.steps,
            all_ok,
            require_property: self.require_property.clone(),
            records,
        };
        fs::write(
            self.out.join("summary.json"),
            serde_json::to_vec_pretty(&summary)?,
        )?;
        Ok(summary)
    }

    fn run_seed(
        &self,
        seed: u64,
        dir: &Path,
        git_commit: Option<String>,
    ) -> Result<RunRecord, RunError> {
        fs::create_dir_all(dir)?;
        let mut lab = Lab::open(&self.scenario, seed)?;
        let mut agent = self.make_agent()?;
        let sink = RefCell::new(FileSink::create(dir, self.mcap)?);
        let io_err = RefCell::new(Ok::<(), io::Error>(()));
        let run = lab.research_with(
            &mut *agent,
            self.dt,
            self.steps,
            |obs| {
                if io_err.borrow().is_err() {
                    return;
                }
                if let Err(e) = sink.borrow_mut().observation(obs) {
                    *io_err.borrow_mut() = Err(e);
                }
            },
            |act| {
                if io_err.borrow().is_err() {
                    return;
                }
                if let Err(e) = sink.borrow_mut().action(act) {
                    *io_err.borrow_mut() = Err(e);
                }
            },
        );
        io_err.into_inner()?;
        sink.into_inner().finish()?;
        let property_ok = match &self.require_property {
            None => true,
            Some(id) => run.holds(id),
        };
        let record = RunRecord {
            git_commit,
            dt: self.dt,
            steps_requested: self.steps,
            require_property: self.require_property.clone(),
            property_ok,
            run,
        };
        fs::write(dir.join("run.json"), serde_json::to_vec_pretty(&record)?)?;
        Ok(record)
    }

    fn make_agent(&self) -> Result<Box<dyn ResearchAgent>, RunError> {
        if let Some(path) = &self.jsonl {
            return Ok(Box::new(JsonlScript::load(path, self.dt)?));
        }
        named_agent(&self.agent)
    }
}

struct FileSink {
    obs: BufWriter<File>,
    act: BufWriter<File>,
    bag: Option<McapBag<BufWriter<File>>>,
}

impl FileSink {
    fn create(dir: &Path, mcap: bool) -> io::Result<Self> {
        let bag = if mcap {
            let f = BufWriter::new(File::create(dir.join("bag.mcap"))?);
            Some(McapBag::new(f)?)
        } else {
            None
        };
        Ok(Self {
            obs: BufWriter::new(File::create(dir.join("observations.jsonl"))?),
            act: BufWriter::new(File::create(dir.join("actions.jsonl"))?),
            bag,
        })
    }

    fn observation(&mut self, obs: &Observation) -> io::Result<()> {
        serde_json::to_writer(&mut self.obs, obs)?;
        self.obs.write_all(b"\n")?;
        if let Some(bag) = &mut self.bag {
            bag.write_observation(obs)?;
        }
        Ok(())
    }

    fn action(&mut self, action: &TimedAction) -> io::Result<()> {
        serde_json::to_writer(&mut self.act, action)?;
        self.act.write_all(b"\n")?;
        if let Some(bag) = &mut self.bag {
            bag.write_action(action)?;
        }
        Ok(())
    }

    fn finish(mut self) -> io::Result<()> {
        self.obs.flush()?;
        self.act.flush()?;
        if let Some(bag) = self.bag.take() {
            bag.finish()?;
        }
        Ok(())
    }
}

struct JsonlScript {
    queued: Vec<TimedAction>,
}

impl JsonlScript {
    fn load(path: &Path, dt: f32) -> Result<Self, RunError> {
        let file = File::open(path)?;
        let mut queued = Vec::new();
        let mut n = 0usize;
        for line in BufReader::new(file).lines() {
            let line = line?;
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if let Ok(ta) = serde_json::from_str::<TimedAction>(line) {
                queued.push(ta);
                continue;
            }
            let action: AgentAction = serde_json::from_str(line)?;
            queued.push(TimedAction {
                t: n as f32 * dt,
                action,
            });
            n += 1;
        }
        Ok(Self { queued })
    }
}

impl ResearchAgent for JsonlScript {
    fn name(&self) -> &'static str {
        "jsonl"
    }

    fn act(&mut self, lab: &mut Lab, _obs: &Observation) -> Vec<AgentAction> {
        let t = lab.world().t;
        let mut out = Vec::new();
        self.queued.retain(|ta| {
            if ta.t <= t + 1e-6 {
                out.push(ta.action.clone());
                false
            } else {
                true
            }
        });
        out
    }
}

/// Resolve a typed research agent by CLI / `ResearchAgent::name` string.
pub fn named_agent(name: &str) -> Result<Box<dyn ResearchAgent>, RunError> {
    let key = name.trim().replace('-', "_");
    Ok(match key.as_str() {
        "typed_fleet_hold" => Box::new(TypedFleetHold::default()),
        "typed_fleet" => Box::new(TypedFleet::default()),
        "typed_attach_fleet" => Box::new(TypedAttachFleet::default()),
        "typed_fleet_return" => Box::new(TypedFleetReturn::default()),
        "typed_pad_landing" => Box::new(TypedPadLanding::default()),
        "typed_aerial_failsafe" => Box::new(TypedAerialFailsafe::default()),
        "typed_aerial_disarm" => Box::new(TypedAerialDisarm::default()),
        "typed_aerial_airborne" => Box::new(TypedAerialAirborne::default()),
        "typed_position_hold" => Box::new(TypedPositionHold::default()),
        "typed_hold" => Box::new(TypedHold::default()),
        "typed_pad_disarm" => Box::new(TypedPadDisarm::default()),
        "typed_pad_failsafe" => Box::new(TypedPadFailsafe::default()),
        "typed_failsafe_touchdown" => Box::new(TypedFailsafeTouchdown::default()),
        "typed_collision_sweep" => Box::new(TypedCollisionSweep::default()),
        "typed_ground_estop" => Box::new(TypedGroundEstop::default()),
        "typed_ground_halt" => Box::new(TypedGroundHalt::default()),
        "typed_ground_hold" => Box::new(TypedGroundHold::default()),
        "typed_hull_dock" => Box::new(TypedHullDock::default()),
        "typed_hull_failsafe" => Box::new(TypedHullFailsafe::default()),
        "typed_station_dock" => Box::new(TypedStationDock::default()),
        "typed_station_failsafe" => Box::new(TypedStationFailsafe::default()),
        "typed_station_resume" => Box::new(TypedStationResume::default()),
        "typed_surveyor_dock" => Box::new(TypedSurveyorDock::default()),
        "typed_surveyor_failsafe" => Box::new(TypedSurveyorFailsafe::default()),
        "typed_surveyor_station_dock" => Box::new(TypedSurveyorStationDock::default()),
        "typed_surveyor_station_failsafe" => Box::new(TypedSurveyorStationFailsafe::default()),
        "typed_surveyor_station_resume" => Box::new(TypedSurveyorStationResume::default()),
        "rover_probe" => Box::new(RoverProbe::default()),
        "pad_landing" => Box::new(PadLanding::default()),
        "collision_sweep" => Box::new(CollisionSweep::default()),
        "coastal_fleet" => Box::new(CoastalFleet::default()),
        "scripted_coastal" | "scripted" => Box::new(ScriptedCoastal),
        other => {
            return Err(RunError::Agent(format!(
                "unknown agent '{other}' (try typed-fleet-hold)"
            )))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bag::looks_like_mcap;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "robot-lab-{name}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn harbor_typed_fleet_hold_seed_sweep_writes_run_dir() {
        let out = scratch("harbor-sweep");
        let summary = Experiment {
            scenario: "harbor".into(),
            seeds: vec![1, 3],
            dt: 0.02,
            steps: 40,
            agent: "typed-fleet-hold".into(),
            jsonl: None,
            out: out.clone(),
            mcap: true,
            require_property: Some("position_hold_restores_pose".into()),
        }
        .execute()
        .unwrap();
        assert!(summary.all_ok, "{summary:?}");
        assert_eq!(summary.seeds, vec![1, 3]);
        assert_eq!(summary.records.len(), 2);
        assert!(summary.git_commit.is_some());
        for seed in [1u64, 3] {
            let dir = out.join(format!("seed-{seed}"));
            let run: serde_json::Value =
                serde_json::from_slice(&fs::read(dir.join("run.json")).unwrap()).unwrap();
            assert_eq!(run["run"]["all_hold"], true);
            assert_eq!(run["run"]["agent"], "typed_fleet_hold");
            assert_eq!(run["property_ok"], true);
            assert!(run["git_commit"].as_str().is_some());
            let obs = fs::read_to_string(dir.join("observations.jsonl")).unwrap();
            assert!(obs.lines().count() >= 2);
            let acts = fs::read_to_string(dir.join("actions.jsonl")).unwrap();
            assert!(acts.contains("takeoff"), "{acts}");
            let bag = fs::read(dir.join("bag.mcap")).unwrap();
            assert!(looks_like_mcap(&bag));
        }
        let _ = fs::remove_dir_all(&out);
    }

    #[test]
    fn require_property_failure_is_not_ok() {
        let out = scratch("missing-prop");
        let summary = Experiment {
            scenario: "harbor".into(),
            seeds: vec![1],
            dt: 0.02,
            steps: 4,
            agent: "typed-fleet-hold".into(),
            jsonl: None,
            out: out.clone(),
            mcap: false,
            require_property: Some("not_a_real_property".into()),
        }
        .execute()
        .unwrap();
        assert!(!summary.all_ok);
        assert!(!summary.records[0].property_ok);
        assert!(summary.records[0].run.all_hold);
        let _ = fs::remove_dir_all(&out);
    }

    #[test]
    fn jsonl_script_one_world_step_per_tick() {
        let out = scratch("jsonl-p12");
        let script = out.join("script.jsonl");
        fs::create_dir_all(&out).unwrap();
        fs::write(
            &script,
            concat!(
                r#"{"robot":"","cmd":"set_wind","vn":1.0,"ve":0.0,"vd":0.0}"#,
                "\n",
                r#"{"robot":"","cmd":"set_waves","vn":0.4,"ve":0.2,"vd":1.0}"#,
                "\n",
            ),
        )
        .unwrap();
        let summary = Experiment {
            scenario: "inland".into(),
            seeds: vec![2],
            dt: 0.02,
            steps: 2,
            agent: String::new(),
            jsonl: Some(script),
            out: out.clone(),
            mcap: false,
            require_property: None,
        }
        .execute()
        .unwrap();
        assert!(summary.all_ok);
        assert_eq!(summary.records[0].run.steps, 2);
        let t = summary.records[0].run.t;
        assert!(
            (t - 0.04).abs() < 1e-4,
            "P12: two ticks × dt=0.02 ⇒ t≈0.04, got {t}"
        );
        assert_eq!(summary.records[0].run.actions_applied, 2);
        let acts = fs::read_to_string(out.join("actions.jsonl")).unwrap();
        assert!(acts.contains("set_wind"));
        assert!(acts.contains("set_waves"));
        let _ = fs::remove_dir_all(&out);
    }
}
