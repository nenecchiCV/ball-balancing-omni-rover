//! IPC 境界: Tauri command 群。
//! UI 引数の検証と `robot::Session` 呼出しのみを担い、ドメインロジックは持たない。

use serde::Serialize;
use specta::Type;
use tauri::{ipc::Channel, State};
use tokio::sync::broadcast;

use crate::ble::nus::DiscoveredDevice;
use crate::error::AppError;
use crate::protocol::message::{ack_result, control_mode, Request, Response};
use crate::robot::log::LogMessage;
use crate::robot::session::LinkState;
use crate::robot::telemetry::{PoseTelemetry, StatusTelemetry};
use crate::state::AppState;

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct PongInfo {
    pub proto_ver: u8,
    pub robot_caps: u8,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct RobotStatusInfo {
    pub mode: u8,
    pub enable: u8,
    pub estop: u8,
    pub imu_ok: u8,
    pub uptime_ms: u32,
}

#[tauri::command]
#[specta::specta]
pub async fn ble_scan(
    state: State<'_, AppState>,
    timeout_ms: Option<u32>,
) -> Result<Vec<DiscoveredDevice>, AppError> {
    state.session.scan(timeout_ms.unwrap_or(5_000)).await
}

#[tauri::command]
#[specta::specta]
pub async fn ble_connect(state: State<'_, AppState>, address: String) -> Result<(), AppError> {
    state.session.connect(&address).await
}

#[tauri::command]
#[specta::specta]
pub async fn ble_disconnect(state: State<'_, AppState>) -> Result<(), AppError> {
    state.session.disconnect().await;
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn connection_state(state: State<'_, AppState>) -> Result<LinkState, AppError> {
    Ok(state.session.state().await)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_ping(state: State<'_, AppState>) -> Result<PongInfo, AppError> {
    match state.session.request(&Request::Ping).await? {
        Response::Pong {
            proto_ver,
            robot_caps,
            ..
        } => Ok(PongInfo {
            proto_ver,
            robot_caps,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_velocity(
    state: State<'_, AppState>,
    vx: f32,
    vy: f32,
    yaw_rate: f32,
) -> Result<(), AppError> {
    state
        .session
        .send_no_ack(&Request::SetVelocity { vx, vy, yaw_rate })
        .await
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_pose_target(
    state: State<'_, AppState>,
    x: f32,
    y: f32,
    speed_limit: f32,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::SetPoseTarget { x, y, speed_limit })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_stop(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::Stop).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_mode(state: State<'_, AppState>, mode: u8) -> Result<(), AppError> {
    if mode > control_mode::POSITION {
        return Err(AppError::InvalidParam(format!("mode が不正: {mode}")));
    }
    ack_to_result(
        state
            .session
            .request(&Request::SetControlMode { mode })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_reset_odometry(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::ResetOdometry).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_get_status(state: State<'_, AppState>) -> Result<RobotStatusInfo, AppError> {
    match state.session.request(&Request::GetStatus).await? {
        Response::Status {
            mode,
            enable,
            estop,
            imu_ok,
            uptime_ms,
            ..
        } => Ok(RobotStatusInfo {
            mode,
            enable,
            estop,
            imu_ok,
            uptime_ms,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_subscribe(
    state: State<'_, AppState>,
    event_type: u8,
    rate_hz: u8,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::Subscribe {
                event_type,
                rate_hz,
            })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_unsubscribe(
    state: State<'_, AppState>,
    event_type: u8,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::Unsubscribe { event_type })
            .await?,
    )
}

// --- UI 向けイベントストリーム（broadcast → Channel 橋渡し） ------------------

/// 現在値を即時送った後、以後の変化をストリームする。
#[tauri::command]
#[specta::specta]
pub async fn watch_link_state(
    state: State<'_, AppState>,
    channel: Channel<LinkState>,
) -> Result<(), AppError> {
    let current = state.session.state().await;
    let _ = channel.send(current);
    spawn_forward(state.session.subscribe_link(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_telemetry(
    state: State<'_, AppState>,
    channel: Channel<PoseTelemetry>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_telemetry(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_status(
    state: State<'_, AppState>,
    channel: Channel<StatusTelemetry>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_status(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_log(
    state: State<'_, AppState>,
    channel: Channel<LogMessage>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_log(), channel);
    Ok(())
}

fn spawn_forward<T: Clone + serde::Serialize + Send + 'static>(
    mut rx: broadcast::Receiver<T>,
    channel: Channel<T>,
) {
    tauri::async_runtime::spawn(async move {
        loop {
            match rx.recv().await {
                Ok(v) => {
                    if channel.send(v).is_err() {
                        break;
                    }
                }
                // 遅延分は捨てて最新へ追従する。
                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                Err(broadcast::error::RecvError::Closed) => break,
            }
        }
    });
}

fn ack_to_result(resp: Response) -> Result<(), AppError> {
    match resp {
        Response::Ack {
            req_type, result, ..
        } => match result {
            ack_result::OK => Ok(()),
            r => Err(AppError::Rejected(format!(
                "ACK 拒否: req=0x{req_type:02X} result={r}"
            ))),
        },
        _ => Err(AppError::BadResponse),
    }
}
