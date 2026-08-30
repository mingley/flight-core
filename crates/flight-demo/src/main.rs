//! Live coastal lab: aerial, ground, surface, and underwater bodies
//! with mechanical properties checked every tick.
//!
//! Safety trips and return commands live under the NED map. Drone failsafe /
//! recover / land / touchdown / disarm and rover E-stop / clear / park are
//! offered when those bodies are in the scene. Skiff and AUV failsafe /
//! recover / dock appear on coastal, harbor, and open water (hidden inland).
//! Station / resume on each hull and drone airborne / hold sit under return.
//! POST `/api/lab/action` queues [`LabCmd`] through [`Lab::act_through_attach`]
//! (A1 legal-command gate). GET `/api/lab/observation` is the live snapshot.
//! GET `/api/lab/tools` lists A1 `(robot, cmd)` tools. GET `/api/lab/replay`
//! is action-log metadata. POST `/api/lab/research` runs a closed-loop
//! [`Lab::research`] on a fresh lab with the console scenario/seed (one
//! `WorldSession::step` per tick). MHS-shaped routes: GET `/api/mhs/discover`,
//! GET `/api/mhs/reference`, GET `/api/mhs/reference/{id}`, POST `/api/mhs/read`,
//! POST `/api/mhs/write` (A1 gate + driver numeric limits; queued like
//! `/api/lab/action`). Not official Model Hardware Standard. Binds
//! `FLIGHT_DEMO_BIND` (default `0.0.0.0:47831`). No authentication. No raw NED
//! velocity that skips `legal_cmds`.

use axum::extract::{Path, State};
use axum::http::header::{HeaderMap, HeaderValue, CACHE_CONTROL};
use axum::http::StatusCode;
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::Json;
use axum::Router;
use flight_mhs::{
    preview_write, queued_action, read_channel, DeviceReference, Discovery, DriverLimits,
    WriteRequest,
};
use robot_lab::{named_agent, AgentAction, Lab, LabCmd, LegalTools, Observation, ResearchRun};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::{watch, Mutex};
use tokio::time::sleep;
use tower_http::cors::CorsLayer;

const INDEX: &str = include_str!("index.html");

struct App {
    tx: watch::Sender<Observation>,
    rx: watch::Receiver<Observation>,
    pending: Mutex<Vec<AgentAction>>,
    reset: AtomicBool,
    scripted: AtomicBool,
    scenario: Mutex<String>,
    seed: Mutex<u64>,
    replay: Mutex<ReplayMeta>,
}

#[derive(Deserialize)]
struct OpenReq {
    scenario: String,
    #[serde(default = "default_seed")]
    seed: u64,
}

#[derive(Clone, Debug, Serialize)]
struct ReplayMeta {
    t: f32,
    actions: usize,
    cmds: Vec<String>,
    scenario: String,
    seed: u64,
}

impl ReplayMeta {
    fn from_lab(lab: &Lab) -> Self {
        let world = lab.world();
        Self {
            t: world.t,
            actions: lab.log.len(),
            cmds: lab
                .log
                .iter()
                .map(|a| a.action.cmd.as_str().to_string())
                .collect(),
            scenario: world.scenario.into(),
            seed: world.seed,
        }
    }
}

#[derive(Deserialize)]
struct ResearchReq {
    #[serde(default = "default_research_agent")]
    agent: String,
    #[serde(default = "default_research_steps")]
    steps: u32,
    #[serde(default = "default_research_dt")]
    dt: f32,
}

fn default_research_agent() -> String {
    "typed-fleet-hold".into()
}

fn default_research_steps() -> u32 {
    8
}

fn default_research_dt() -> f32 {
    0.02
}

fn default_seed() -> u64 {
    1
}

#[derive(Serialize)]
struct OkMsg {
    ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    message: Option<String>,
}

#[tokio::main]
async fn main() {
    if std::env::var_os("FLIGHT_HYDRO_GPU").is_none() {
        std::env::set_var("FLIGHT_HYDRO_GPU", "1");
    }
    let lab = Lab::coastal(1);
    let initial = lab.observe();
    let (tx, rx) = watch::channel(initial);
    let app = Arc::new(App {
        tx,
        rx,
        pending: Mutex::new(Vec::new()),
        reset: AtomicBool::new(false),
        scripted: AtomicBool::new(true),
        scenario: Mutex::new("coastal".into()),
        seed: Mutex::new(1),
        replay: Mutex::new(ReplayMeta::from_lab(&lab)),
    });

    let worker = app.clone();
    tokio::spawn(async move {
        loop {
            run_lab(&worker).await;
            sleep(Duration::from_millis(400)).await;
        }
    });

    let bind = std::env::var("FLIGHT_DEMO_BIND").unwrap_or_else(|_| "0.0.0.0:47831".into());
    let listener = TcpListener::bind(&bind).await.expect("bind");
    eprintln!("robot-lab listening on http://{bind}");
    axum::serve(listener, router(app)).await.expect("server");
}

fn router(app: Arc<App>) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/telemetry", get(observation))
        .route("/api/lab/observation", get(observation))
        .route("/api/lab/tools", get(tools))
        .route("/api/lab/replay", get(replay))
        .route("/api/lab/research", post(research))
        .route("/api/lab/action", post(action))
        .route("/api/lab/reset", post(reset))
        .route("/api/lab/open", post(open))
        .route("/api/lab/scenarios", get(scenarios))
        .route("/api/mhs/discover", get(mhs_discover))
        .route("/api/mhs/reference", get(mhs_references))
        .route("/api/mhs/reference/{id}", get(mhs_reference))
        .route("/api/mhs/read", post(mhs_read))
        .route("/api/mhs/write", post(mhs_write))
        .route("/api/failsafe", post(trip))
        .route("/api/recover", post(recover))
        .route("/api/estop", post(estop))
        .route("/api/clear", post(clear))
        .route("/api/skiff-failsafe", post(skiff_failsafe))
        .route("/api/skiff-recover", post(skiff_recover))
        .route("/api/auv-failsafe", post(auv_failsafe))
        .route("/api/auv-recover", post(auv_recover))
        .route("/api/land", post(land))
        .route("/api/touchdown", post(touchdown))
        .route("/api/disarm", post(disarm))
        .route("/api/park", post(park))
        .route("/api/skiff-dock", post(skiff_dock))
        .route("/api/auv-dock", post(auv_dock))
        .route("/api/skiff-station", post(skiff_station))
        .route("/api/skiff-resume", post(skiff_resume))
        .route("/api/auv-station", post(auv_station))
        .route("/api/auv-resume", post(auv_resume))
        .route("/api/airborne", post(airborne))
        .route("/api/hold", post(hold))
        .with_state(app)
        .layer(CorsLayer::permissive())
}

async fn index() -> impl IntoResponse {
    let mut headers = HeaderMap::new();
    headers.insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
    (headers, Html(INDEX))
}

async fn observation(State(app): State<Arc<App>>) -> Json<Observation> {
    Json(app.rx.borrow().clone())
}

async fn tools(State(app): State<Arc<App>>) -> Json<LegalTools> {
    Json(app.rx.borrow().tools())
}

async fn replay(State(app): State<Arc<App>>) -> Json<ReplayMeta> {
    Json(app.replay.lock().await.clone())
}

async fn research(
    State(app): State<Arc<App>>,
    Json(req): Json<ResearchReq>,
) -> Result<Json<ResearchRun>, (StatusCode, Json<OkMsg>)> {
    let scenario = app.scenario.lock().await.clone();
    let seed = *app.seed.lock().await;
    let mut agent = named_agent(&req.agent).map_err(|e| {
        (
            StatusCode::BAD_REQUEST,
            Json(OkMsg {
                ok: false,
                error: Some(e.to_string()),
                message: None,
            }),
        )
    })?;
    let mut lab = Lab::open(&scenario, seed).unwrap_or_else(|_| Lab::coastal(seed));
    Ok(Json(lab.research(&mut *agent, req.dt, req.steps)))
}

async fn action(State(app): State<Arc<App>>, Json(act): Json<AgentAction>) -> impl IntoResponse {
    app.scripted.store(false, Ordering::SeqCst);
    let mut q = app.pending.lock().await;
    let cmd = act.cmd;
    q.push(act);
    Json(OkMsg {
        ok: true,
        error: None,
        message: Some(format!("queued {cmd}")),
    })
}

async fn reset(State(app): State<Arc<App>>) -> Json<OkMsg> {
    app.reset.store(true, Ordering::SeqCst);
    app.scripted.store(true, Ordering::SeqCst);
    Json(OkMsg {
        ok: true,
        error: None,
        message: Some("reset".into()),
    })
}

async fn scenarios() -> Json<&'static [&'static str]> {
    Json(Lab::scenarios())
}

async fn open(State(app): State<Arc<App>>, Json(req): Json<OpenReq>) -> impl IntoResponse {
    if Lab::open(&req.scenario, req.seed).is_err() {
        return Json(OkMsg {
            ok: false,
            error: Some(format!("unknown scenario {}", req.scenario)),
            message: None,
        });
    }
    *app.scenario.lock().await = req.scenario.clone();
    *app.seed.lock().await = req.seed;
    app.scripted.store(true, Ordering::SeqCst);
    app.reset.store(true, Ordering::SeqCst);
    Json(OkMsg {
        ok: true,
        error: None,
        message: Some(format!("opening {}", req.scenario)),
    })
}

async fn trip(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Failsafe, "queued drone failsafe").await
}

async fn recover(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Recover, "queued recover").await
}

async fn estop(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "rover", LabCmd::Estop, "queued rover estop").await
}

async fn clear(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "rover", LabCmd::Clear, "queued rover clear").await
}

async fn skiff_failsafe(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "skiff", LabCmd::Failsafe, "queued skiff failsafe").await
}

async fn skiff_recover(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "skiff", LabCmd::Recover, "queued skiff recover").await
}

async fn auv_failsafe(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "surveyor", LabCmd::Failsafe, "queued auv failsafe").await
}

async fn auv_recover(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "surveyor", LabCmd::Recover, "queued auv recover").await
}

async fn land(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Land, "queued drone land").await
}

async fn touchdown(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Touchdown, "queued drone touchdown").await
}

async fn disarm(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Disarm, "queued drone disarm").await
}

async fn park(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "rover", LabCmd::Park, "queued rover park").await
}

async fn skiff_dock(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "skiff", LabCmd::Dock, "queued skiff dock").await
}

async fn auv_dock(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "surveyor", LabCmd::Dock, "queued auv dock").await
}

async fn skiff_station(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "skiff", LabCmd::Station, "queued skiff station").await
}

async fn skiff_resume(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "skiff", LabCmd::Resume, "queued skiff resume").await
}

async fn auv_station(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "surveyor", LabCmd::Station, "queued auv station").await
}

async fn auv_resume(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "surveyor", LabCmd::Resume, "queued auv resume").await
}

async fn airborne(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Airborne, "queued drone airborne").await
}

async fn hold(State(app): State<Arc<App>>) -> Json<OkMsg> {
    queue_robot(&app, "drone", LabCmd::Hold, "queued drone hold").await
}

async fn queue_robot(
    app: &App,
    robot: &'static str,
    cmd: LabCmd,
    message: &'static str,
) -> Json<OkMsg> {
    app.scripted.store(false, Ordering::SeqCst);
    let mut q = app.pending.lock().await;
    q.push(AgentAction::new(robot, cmd));
    Json(OkMsg {
        ok: true,
        error: None,
        message: Some(message.into()),
    })
}

async fn run_lab(app: &App) {
    app.reset.store(false, Ordering::SeqCst);
    let name = app.scenario.lock().await.clone();
    let seed = *app.seed.lock().await;
    let mut lab = Lab::open(&name, seed).unwrap_or_else(|_| Lab::coastal(seed));
    let _ = app.tx.send(lab.observe());
    *app.replay.lock().await = ReplayMeta::from_lab(&lab);

    loop {
        if app.reset.load(Ordering::SeqCst) {
            return;
        }

        let pending: Vec<AgentAction> = {
            let mut q = app.pending.lock().await;
            q.drain(..).collect()
        };
        if app.scripted.load(Ordering::SeqCst) && lab.with_world(|w| w.t) >= 1.2 {
            lab.apply_script();
        }
        for act in pending {
            if let Err(e) = lab.act_through_attach(act) {
                lab.message = format!("agent rejected: {e}");
            }
        }

        lab.step(0.02);
        let _ = app.tx.send(lab.observe());
        *app.replay.lock().await = ReplayMeta::from_lab(&lab);
        sleep(Duration::from_millis(25)).await;

        if lab.world().t > 40.0 && app.scripted.load(Ordering::SeqCst) {
            return;
        }
    }
}

async fn mhs_discover(State(app): State<Arc<App>>) -> Json<Discovery> {
    Json(Discovery::from_observation(&app.rx.borrow()))
}

async fn mhs_references(State(app): State<Arc<App>>) -> Json<Vec<DeviceReference>> {
    let obs = app.rx.borrow().clone();
    let d = Discovery::from_observation(&obs);
    Json(
        d.devices
            .iter()
            .filter_map(|s| DeviceReference::compile(&obs, &s.id, &DriverLimits::DEFAULT).ok())
            .collect(),
    )
}

async fn mhs_reference(
    State(app): State<Arc<App>>,
    Path(id): Path<String>,
) -> Result<Json<DeviceReference>, (StatusCode, Json<OkMsg>)> {
    let obs = app.rx.borrow().clone();
    DeviceReference::compile(&obs, &id, &DriverLimits::DEFAULT)
        .map(Json)
        .map_err(|e| {
            (
                StatusCode::BAD_REQUEST,
                Json(OkMsg {
                    ok: false,
                    error: Some(e.to_string()),
                    message: None,
                }),
            )
        })
}

#[derive(Deserialize)]
struct MhsReadReq {
    device: String,
    channel: String,
}

async fn mhs_read(
    State(app): State<Arc<App>>,
    Json(req): Json<MhsReadReq>,
) -> Result<Json<flight_mhs::ReadResult>, (StatusCode, Json<OkMsg>)> {
    let obs = app.rx.borrow().clone();
    read_channel(&obs, &req.device, &req.channel)
        .map(Json)
        .map_err(|e| {
            (
                StatusCode::BAD_REQUEST,
                Json(OkMsg {
                    ok: false,
                    error: Some(e.to_string()),
                    message: None,
                }),
            )
        })
}

async fn mhs_write(
    State(app): State<Arc<App>>,
    Json(req): Json<WriteRequest>,
) -> impl IntoResponse {
    let obs = app.rx.borrow().clone();
    match preview_write(&obs, &req, &DriverLimits::DEFAULT) {
        Ok(cmd) => {
            app.scripted.store(false, Ordering::SeqCst);
            app.pending.lock().await.push(queued_action(&req, cmd));
            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "ok": true,
                    "device": req.device,
                    "channel": req.channel,
                    "message": format!("queued {}", req.channel),
                })),
            )
        }
        Err(e) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::to_value(e.as_failure(None)).unwrap()),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use http_body_util::BodyExt;
    use tower::ServiceExt;

    fn test_app() -> Arc<App> {
        let lab = Lab::coastal(1);
        let initial = lab.observe();
        let (tx, rx) = watch::channel(initial);
        Arc::new(App {
            tx,
            rx,
            pending: Mutex::new(Vec::new()),
            reset: AtomicBool::new(false),
            scripted: AtomicBool::new(false),
            scenario: Mutex::new("coastal".into()),
            seed: Mutex::new(1),
            replay: Mutex::new(ReplayMeta::from_lab(&lab)),
        })
    }

    async fn body_json(res: axum::response::Response) -> serde_json::Value {
        let bytes = res.into_body().collect().await.unwrap().to_bytes();
        serde_json::from_slice(&bytes).unwrap()
    }

    #[tokio::test]
    async fn tools_route_lists_a1_legal_cmds_not_parked_drive() {
        let res = router(test_app())
            .oneshot(
                Request::builder()
                    .uri("/api/lab/tools")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let v = body_json(res).await;
        let env: Vec<_> = v["env_cmds"]
            .as_array()
            .unwrap()
            .iter()
            .map(|c| c.as_str().unwrap())
            .collect();
        assert!(env.contains(&"set_wind"));
        let tools = v["robot_tools"].as_array().unwrap();
        assert!(tools
            .iter()
            .any(|t| t["robot"] == "rover" && t["cmd"] == "release"));
        assert!(!tools
            .iter()
            .any(|t| t["robot"] == "rover" && t["cmd"] == "drive"));
    }

    #[tokio::test]
    async fn observation_and_replay_routes() {
        let app = test_app();
        let obs = router(app.clone())
            .oneshot(
                Request::builder()
                    .uri("/api/lab/observation")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(obs.status(), StatusCode::OK);
        let o = body_json(obs).await;
        assert_eq!(o["scenario"], "coastal");
        assert!(o["broken"].as_array().unwrap().is_empty());

        let replay = router(app)
            .oneshot(
                Request::builder()
                    .uri("/api/lab/replay")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(replay.status(), StatusCode::OK);
        let r = body_json(replay).await;
        assert_eq!(r["actions"], 0);
        assert_eq!(r["scenario"], "coastal");
    }

    #[tokio::test]
    async fn research_route_runs_typed_agent_one_step_per_tick() {
        let res = router(test_app())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/lab/research")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        r#"{"agent":"typed-fleet-hold","steps":4,"dt":0.02}"#,
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let run = body_json(res).await;
        assert_eq!(run["agent"], "typed_fleet_hold");
        assert_eq!(run["all_hold"], true);
        assert_eq!(run["steps"], 4);
        let t = run["t"].as_f64().unwrap();
        assert!((t - 0.08).abs() < 1e-3, "P12: 4 × 0.02 ⇒ t≈0.08, got {t}");
    }

    #[tokio::test]
    async fn research_route_rejects_unknown_agent() {
        let res = router(test_app())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/lab/research")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"agent":"not-a-tool"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn mhs_discover_is_shaped_not_official() {
        let res = router(test_app())
            .oneshot(
                Request::builder()
                    .uri("/api/mhs/discover")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let v = body_json(res).await;
        assert_eq!(v["official"], false);
        assert_eq!(v["conformance"], "shaped");
        let ids: Vec<_> = v["devices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|d| d["id"].as_str().unwrap())
            .collect();
        assert!(ids.contains(&"drone"));
        assert!(ids.contains(&"env"));
        assert!(ids.contains(&"lab"));
    }

    #[tokio::test]
    async fn mhs_write_rejects_parked_drive() {
        let res = router(test_app())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/mhs/write")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        r#"{"device":"rover","channel":"drive","vn":-1.0}"#,
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::BAD_REQUEST);
        let v = body_json(res).await;
        assert_eq!(v["ok"], false);
        assert_eq!(v["code"], "not_legal");
    }

    #[tokio::test]
    async fn mhs_read_pose_and_reference_rover() {
        let app = test_app();
        let read = router(app.clone())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/mhs/read")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"device":"drone","channel":"pose.ned"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(read.status(), StatusCode::OK);
        let pose = body_json(read).await;
        assert_eq!(pose["channel"], "pose.ned");
        assert_eq!(pose["value"]["z"], "down");

        let refer = router(app)
            .oneshot(
                Request::builder()
                    .uri("/api/mhs/reference/rover")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(refer.status(), StatusCode::OK);
        let r = body_json(refer).await;
        assert_eq!(r["id"], "rover");
        assert_eq!(r["official"], false);
        assert!(r["writes"]
            .as_array()
            .unwrap()
            .iter()
            .any(|w| w["channel"] == "drive"));
    }
}
