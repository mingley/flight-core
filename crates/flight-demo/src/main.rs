//! Live mission-control dashboard for a simulated `flight-core` vehicle.

use axum::extract::State;
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::Json;
use axum::Router;
use flight_core::prelude::*;
use flight_core::vehicle::Telemetry;
use flight_sim::{connect, SimConfig};
use serde::Serialize;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::watch;
use tokio::time::sleep;
use tower_http::cors::CorsLayer;

const INDEX: &str = include_str!("index.html");

#[derive(Clone, Debug, Serialize)]
struct Snapshot {
    t: f32,
    phase: String,
    armed: bool,
    actuators: bool,
    offboard: bool,
    failsafe: bool,
    imu_healthy: bool,
    estimator_valid: bool,
    n: f32,
    e: f32,
    d: f32,
    vn: f32,
    ve: f32,
    vd: f32,
    alt: f32,
    yaw: f32,
    last_command: String,
    message: String,
}

impl Snapshot {
    fn from_tel(tel: &Telemetry, message: &str) -> Self {
        Self {
            t: tel.timestamp.as_secs_f32(),
            phase: tel.phase.name().into(),
            armed: tel.armed,
            actuators: tel.actuators_enabled,
            offboard: tel.offboard,
            failsafe: tel.failsafe,
            imu_healthy: tel.imu_healthy,
            estimator_valid: tel.estimator_valid,
            n: tel.position.x(),
            e: tel.position.y(),
            d: tel.position.z(),
            vn: tel.velocity.x(),
            ve: tel.velocity.y(),
            vd: tel.velocity.z(),
            alt: tel.altitude_agl().get(),
            yaw: tel.yaw_rad,
            last_command: tel.last_command.into(),
            message: message.into(),
        }
    }
}

struct App {
    tx: watch::Sender<Snapshot>,
    rx: watch::Receiver<Snapshot>,
    trip_failsafe: AtomicBool,
}

#[tokio::main]
async fn main() {
    let initial = Snapshot {
        t: 0.0,
        phase: "disconnected".into(),
        armed: false,
        actuators: false,
        offboard: false,
        failsafe: false,
        imu_healthy: false,
        estimator_valid: false,
        n: 0.0,
        e: 0.0,
        d: 0.0,
        vn: 0.0,
        ve: 0.0,
        vd: 0.0,
        alt: 0.0,
        yaw: 0.0,
        last_command: "boot".into(),
        message: "starting simulated vehicle".into(),
    };
    let (tx, rx) = watch::channel(initial);
    let app = Arc::new(App {
        tx,
        rx,
        trip_failsafe: AtomicBool::new(false),
    });

    let worker = app.clone();
    tokio::spawn(async move {
        loop {
            if let Err(e) = run_mission(&worker).await {
                let mut snap = worker.tx.borrow().clone();
                snap.message = format!("mission error: {e}");
                let _ = worker.tx.send(snap);
            }
            sleep(Duration::from_secs(2)).await;
        }
    });

    let router = Router::new()
        .route("/", get(index))
        .route("/api/telemetry", get(telemetry))
        .route("/api/failsafe", post(trip))
        .with_state(app)
        .layer(CorsLayer::permissive());

    let bind = std::env::var("FLIGHT_DEMO_BIND").unwrap_or_else(|_| "0.0.0.0:47831".into());
    let listener = TcpListener::bind(&bind).await.expect("bind");
    eprintln!("flight-demo listening on http://{bind}");
    axum::serve(listener, router).await.expect("server");
}

async fn index() -> Html<&'static str> {
    Html(INDEX)
}

async fn telemetry(State(app): State<Arc<App>>) -> Json<Snapshot> {
    Json(app.rx.borrow().clone())
}

async fn trip(State(app): State<Arc<App>>) -> impl IntoResponse {
    app.trip_failsafe.store(true, Ordering::SeqCst);
    Json(serde_json::json!({ "ok": true }))
}

fn publish(app: &App, tel: &Telemetry, message: &str) {
    let _ = app.tx.send(Snapshot::from_tel(tel, message));
}

async fn maybe_sleep() {
    sleep(Duration::from_millis(25)).await;
}

async fn run_mission(app: &App) -> Result<(), String> {
    app.trip_failsafe.store(false, Ordering::SeqCst);

    let mut vehicle = connect(SimConfig::default())
        .await
        .map_err(|e| e.to_string())?;
    if let Ok(t) = vehicle.telemetry().await {
        publish(app, &t, "connected · Disarmed");
    }
    maybe_sleep().await;

    let mut vehicle = vehicle
        .verify_preflight()
        .await
        .map_err(|e| e.error.to_string())?;
    if let Ok(t) = vehicle.telemetry().await {
        publish(app, &t, "preflight passed · IMU + estimator valid");
    }
    maybe_sleep().await;

    let vehicle = vehicle.arm().await.map_err(|e| e.error.to_string())?;
    let mut vehicle = vehicle
        .enter_offboard()
        .await
        .map_err(|e| e.error.to_string())?;
    vehicle.start_takeoff().await.map_err(|e| e.to_string())?;
    if let Ok(t) = vehicle.telemetry().await {
        publish(app, &t, "armed · offboard · takeoff");
    }

    loop {
        if app.trip_failsafe.load(Ordering::SeqCst) {
            let mut fs = vehicle.failsafe().await.map_err(|e| e.error.to_string())?;
            if let Ok(t) = fs.telemetry().await {
                publish(app, &t, "FAILSAFE — mission commands rejected");
            }
            sleep(Duration::from_secs(2)).await;
            let mut disarmed = fs.disarm().await.map_err(|e| e.error.to_string())?;
            if let Ok(t) = disarmed.telemetry().await {
                publish(app, &t, "disarmed after failsafe");
            }
            return Ok(());
        }
        vehicle
            .set_velocity(Velocity::<Ned>::ned(0.0, 0.0, -1.2))
            .await
            .map_err(|e| e.to_string())?;
        let t = vehicle.telemetry().await.map_err(|e| e.to_string())?;
        publish(app, &t, "climbing");
        maybe_sleep().await;
        if t.altitude_agl().get() >= 5.0 {
            break;
        }
    }
    vehicle.declare_airborne().map_err(|e| e.to_string())?;

    for i in 0..90 {
        if app.trip_failsafe.load(Ordering::SeqCst) {
            let mut fs = vehicle.failsafe().await.map_err(|e| e.error.to_string())?;
            if let Ok(t) = fs.telemetry().await {
                publish(app, &t, "FAILSAFE — mission commands rejected");
            }
            sleep(Duration::from_secs(2)).await;
            let _ = fs.disarm().await;
            return Ok(());
        }
        let east = if i > 40 { 0.8 } else { 0.0 };
        vehicle
            .set_velocity(Velocity::<Ned>::ned(1.4, east, 0.0))
            .await
            .map_err(|e| e.to_string())?;
        let t = vehicle.telemetry().await.map_err(|e| e.to_string())?;
        publish(app, &t, "velocity NED cruise");
        maybe_sleep().await;
    }

    // Descend with typed land().
    let mut ticks = 0u32;
    loop {
        if app.trip_failsafe.load(Ordering::SeqCst) {
            break;
        }
        vehicle
            .set_velocity(Velocity::<Ned>::ned(0.0, 0.0, 0.9))
            .await
            .map_err(|e| e.to_string())?;
        let t = vehicle.telemetry().await.map_err(|e| e.to_string())?;
        publish(app, &t, "landing");
        maybe_sleep().await;
        ticks += 1;
        if t.altitude_agl().get() <= 0.12 || ticks > 400 {
            break;
        }
    }

    let mut landed = vehicle.land().await.map_err(|e| e.error.to_string())?;
    if let Ok(t) = landed.telemetry().await {
        publish(app, &t, "landed · disarmed");
    }
    sleep(Duration::from_secs(2)).await;
    Ok(())
}
