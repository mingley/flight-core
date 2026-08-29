//! Live coastal lab: aerial, ground, surface, and underwater bodies
//! with mechanical properties checked every tick.
//!
//! Safety trips and return commands live under the NED map. Drone failsafe /
//! recover / land / touchdown / disarm and rover E-stop / clear / park are
//! offered when those bodies are in the scene. Skiff and AUV failsafe /
//! recover / dock appear on coastal, harbor, and open water (hidden inland).
//! Station / resume on each hull and drone airborne / hold sit under return.
//! POST `/api/hold` queues [`LabCmd::Hold`], which walks [`Lab::attach_hold`]
//! on the live plant (current NED pose, OffboardControl). POST `/api/failsafe`
//! queues [`LabCmd::Failsafe`] on the same [`Lab::act_through_attach`] path as
//! hold, airborne, and station. Other buttons queue [`LabCmd`] through that
//! path as well.

use axum::extract::State;
use axum::http::header::{HeaderMap, HeaderValue, CACHE_CONTROL};
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::Json;
use axum::Router;
use robot_lab::{AgentAction, Lab, LabCmd, Observation};
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
}

#[derive(Deserialize)]
struct OpenReq {
    scenario: String,
    #[serde(default = "default_seed")]
    seed: u64,
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
    });

    let worker = app.clone();
    tokio::spawn(async move {
        loop {
            run_lab(&worker).await;
            sleep(Duration::from_millis(400)).await;
        }
    });

    let router = Router::new()
        .route("/", get(index))
        .route("/api/telemetry", get(observation))
        .route("/api/lab/observation", get(observation))
        .route("/api/lab/action", post(action))
        .route("/api/lab/reset", post(reset))
        .route("/api/lab/open", post(open))
        .route("/api/lab/scenarios", get(scenarios))
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
        .layer(CorsLayer::permissive());

    let bind = std::env::var("FLIGHT_DEMO_BIND").unwrap_or_else(|_| "0.0.0.0:47831".into());
    let listener = TcpListener::bind(&bind).await.expect("bind");
    eprintln!("robot-lab listening on http://{bind}");
    axum::serve(listener, router).await.expect("server");
}

async fn index() -> impl IntoResponse {
    let mut headers = HeaderMap::new();
    headers.insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
    (headers, Html(INDEX))
}

async fn observation(State(app): State<Arc<App>>) -> Json<Observation> {
    Json(app.rx.borrow().clone())
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
        sleep(Duration::from_millis(25)).await;

        if lab.world().t > 40.0 && app.scripted.load(Ordering::SeqCst) {
            return;
        }
    }
}
